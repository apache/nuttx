/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_dmac.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/kmalloc.h>

#include <assert.h>
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>

#include "cxd56_dmac.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PM_APP_ADMAC 51
#define PM_APP_SKDMAC 52
#define PM_APP_IDMAC 54

#define DMAC1_REG_BASE 0x4e020000u /* SMP_DMAC */
#define DMAC2_REG_BASE 0x4e021000u /* SMP_SAKE */
#define DMAC3_REG_BASE 0x4e102000u /* IMG_DMAC */

#define is_dmac(n, dev) ((dev) == (struct dmac_register_map *)\
                                   DMAC ## n ## _REG_BASE)

#define NCHANNELS 9

#define __RO volatile const
#define __WO volatile
#define __RW volatile

struct dmac_ch_register_map
{
  __RW uint32_t srcaddr;
  __RW uint32_t destaddr;
  __RW uint32_t lli;
  __RW uint32_t control;
  __RW uint32_t configuration;
  uint32_t reserved[3];
};

struct dmac080_ch_register_map
{
  __RW uint32_t srcaddr;
  __RW uint32_t destaddr;
  __RW uint32_t lli;
  __RW uint32_t control;
  __RW uint32_t configuration;
  __RW uint32_t deflli;
  uint32_t reserved[2];
};

struct dmac_register_map
{
  __RO uint32_t intstatus;
  __RO uint32_t inttcstatus;
  __WO uint32_t inttcclear;
  __RO uint32_t interrorstatus;
  __WO uint32_t interrorclear;
  __RO uint32_t rawinttcstatus;
  __RO uint32_t rawinterrorstatus;
  __RO uint32_t enbldchns;
  __RW uint32_t softbreq;
  __RW uint32_t softsreq;
  __RW uint32_t softlbreq;
  __RW uint32_t softlsreq;
  __RW uint32_t configuration;
  __RW uint32_t sync;

  uint32_t reserved0[50];

  struct dmac_ch_register_map channel[2];
};

struct dmac080_register_map
{
  __RO uint32_t intstatus;
  __RO uint32_t inttcstatus;
  __WO uint32_t inttcclear;
  __RO uint32_t interrorstatus;
  __WO uint32_t interrorclear;
  __RO uint32_t rawinttcstatus;
  __RO uint32_t rawinterrorstatus;
  __RO uint32_t enbldchns;
  __RW uint32_t softbreq;
  __RW uint32_t softsreq;
  __RW uint32_t softlbreq;
  __RW uint32_t softlsreq;
  __RW uint32_t configuration;
  __RW uint32_t sync;
  __RW uint32_t sreqmask;

  uint32_t reserved0[49];

  /* XXX: deflli not supported */

  struct dmac_ch_register_map channel[5];
};

#define DMAC_CH_ENABLE (1u<<0)
#define DMAC_CH_HALT   (1u<<18)
#define DMAC_CH_ACTIVE (1u<<17)

#ifndef itemsof
#define itemsof(a) (sizeof(a)/sizeof(a[0]))
#endif

/****************************************************************************
 * Link list item structure for use scatter/gather operation
 ****************************************************************************/

typedef struct
{
  uint32_t src_addr;          /* Source address */
  uint32_t dest_addr;         /* Destination address */
  uint32_t nextlli;           /* Next link list */
  uint32_t control;           /* Transfer control */
} dmac_lli_t;

#define CXD56_DMAC_M2M   0  /**< Memory to memory */
#define CXD56_DMAC_M2P   1  /**< Memory to peripheral, DMAC controlled */
#define CXD56_DMAC_P2M   2  /**< Peripheral to memory, DMAC controlled */
#define CXD56_DMAC_P2P   3  /**< Peripheral to peripheral */
#define CXD56_DMAC_P2CP  4  /**< P2P destination controlled */
#define CXD56_DMAC_M2CP  5  /**< M2P peripheral controlled */
#define CXD56_DMAC_CP2M  6  /**< P2M peripheral controlled */
#define CXD56_DMAC_CP2P  7  /**< P2P source controlled */

#define CXD56_DMAC_BSIZE1    0     /**< 1 burst */
#define CXD56_DMAC_BSIZE4    1     /**< 4 burst */
#define CXD56_DMAC_BSIZE8    2     /**< 8 burst */
#define CXD56_DMAC_BSIZE16   3     /**< 16 burst */
#define CXD56_DMAC_BSIZE32   4     /**< 32 burst */
#define CXD56_DMAC_BSIZE64   5     /**< 64 burst */
#define CXD56_DMAC_BSIZE128  6     /**< 128 burst */
#define CXD56_DMAC_BSIZE256  7     /**< 256 burst */

#define CXD56_DMAC_LITTLE_ENDIAN  0  /**< Little endian */
#define CXD56_DMAC_BIG_ENDIAN     1  /**< Bit endian */

#define CXD56_DMAC_MASTER1 0 /**< AHB master 1 */
#define CXD56_DMAC_MASTER2 1 /**< AHB master 2 */

/* max transfer size at a time */

#define CXD56_DMAC_MAX_SIZE 0xfff

/****************************************************************************
 * Helper macro for construct transfer control parameter.
 * Each parameters are the same with PD_DmacSetControl().
 *
 * Example:
 * Here is an example for transfer setting with no interrupt,
 * address increments, 4 byte, 4 burst and 16380 bytes (4 x 4095).
 *
 * list.control = PD_DmacCtrlHelper(0, 1, 1,
 *                                  PD_DMAC_WIDTH32, PD_DMAC_WIDTH32,
 *                                  PD_DMAC_BSIZE4, PD_DMAC_BSIZE4,
 *                                  0xfffu);
 ****************************************************************************/

#define DMAC_CTRL_HELPER(intr, di, si, dwidth, swidth, dbsize, sbsize, tsize) \
  (((intr) & 1u) << 31 |                                               \
   ((di) & 1u) << 27 |                                                 \
   ((si) & 1u) << 26 |                                                 \
   ((dwidth) & 7u) << 21 |                                             \
   ((swidth) & 7u) << 18 |                                             \
   ((dbsize) & 7u) << 15 |                                             \
   ((sbsize) & 7u) << 12 |                                             \
   ((tsize) & 0xfffu))

/****************************************************************************
 * Helper macro for construct transfer control parameter
 * (for APP DMAC channel 2 - 6).
 * Each parameters are the same with PD_DmacSetExControl().
 *
 * Example:
 * Here is an example for transfer setting with no interrupt,
 * address increments, 4 byte, 4 burst and 16380 bytes (4 x 4095).
 *
 * list.control = PD_DmacExCtrlHelper(0, 1, 1, 0, 0,
 *                                    PD_DMAC_WIDTH32, PD_DMAC_WIDTH32,
 *                                    PD_DMAC_BSIZE4, PD_DMAC_BSIZE4,
 *                                    0xfffu);
 *
 * If you want to different burst sizes to source and destination,
 * then data may remained in FIFO. In this case, DMAC cannot clear them.
 * Do not use this configuration to transferring unknown size data
 * (especially communication peripherals).
 * I recommend the same setting to burst sizes.
 ****************************************************************************/

#define DMAC_EX_CTRL_HELPER(\
  intr, di, si, dmaster, smaster, dwidth, swidth, dbsize, sbsize, tsize) \
  (((intr) & 1u) << 31 |                                              \
   ((di) & 1u) << 30 |                                                \
   ((si) & 1u) << 29 |                                                \
   ((dmaster) & 1u) << 28 |                                           \
   ((smaster) & 1u) << 27 |                                           \
   ((dwidth) & 3u) << 25 |                                            \
   ((swidth) & 3u) << 23 |                                            \
   ((dbsize) & 3u) << 21 |                                            \
   ((sbsize) & 3u) << 19 |                                            \
   ((tsize) & 0x7ffffu))

static int open_channels = 0;

static int intr_handler_admac0(int irq, void *context, void *arg);
static int intr_handler_admac1(int irq, void *context, void *arg);
static int intr_handler_idmac(int irq, void *context, void *arg);
static int intr_handler_skdmac0(int irq, void *context, void *arg);
static int intr_handler_skdmac1(int irq, void *context, void *arg);
static uint32_t irq_map[] =
{
  CXD56_IRQ_APP_DMAC0,
  CXD56_IRQ_APP_DMAC1,
  CXD56_IRQ_IDMAC,
  CXD56_IRQ_IDMAC,
  CXD56_IRQ_IDMAC,
  CXD56_IRQ_IDMAC,
  CXD56_IRQ_IDMAC,
  CXD56_IRQ_SKDMAC_0,
  CXD56_IRQ_SKDMAC_1,
};

static int (*intc_handler[])(int irq, void *context, void *arg) =
{
  intr_handler_admac0,
  intr_handler_admac1,
  intr_handler_idmac,
  intr_handler_idmac,
  intr_handler_idmac,
  intr_handler_idmac,
  intr_handler_idmac,
  intr_handler_skdmac0,
  intr_handler_skdmac1,
};

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes one DMA channel */

struct dma_channel_s
{
  uint8_t chan;                  /* DMA channel number (0-CXD56_DMA_NCHANNELS) */
  bool inuse;                    /* TRUE: The DMA channel is in use */
  dma_config_t config;           /* Current configuration */
  dmac_lli_t * list;             /* Link list */
  dma_callback_t callback;       /* Callback invoked when the DMA completes */
  void *arg;                     /* Argument passed to callback function */
  unsigned int dummy;            /* Dummy buffer */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is the array of all DMA channels */

static struct dma_channel_s g_dmach[NCHANNELS];
static mutex_t g_dmalock = NXMUTEX_INITIALIZER;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int dma_init(int ch);
static int dma_uninit(int ch);
static int dma_open(int ch);
static int dma_close(int ch);
static int dma_setconfig(int ch, int itc, int ierr, int flowctrl,
                         int destperi, int srcperi);
static int dma_setintrcallback(int ch, dma_callback_t func, void *data);
static int dma_clearintrcallback(int ch);
static int dma_start(int ch, dmac_lli_t *list);
static int dma_stop(int ch);

static int ch2dmac(int ch)
{
  switch (ch)
    {
    case 0: case 1:
        return 1;
    case 2: case 3: case 4: case 5: case 6: /* APP IDMAC */
        return 3;
    case 7: case 8: /* APP SKDMAC */
        return 2;
    default:
        return 0;
    }
}

static struct dmac_register_map *get_device(int ch)
{
  int id = ch2dmac(ch);

  switch (id)
    {
    case 1: return (struct dmac_register_map *)DMAC1_REG_BASE;
    case 2: return (struct dmac_register_map *)DMAC2_REG_BASE;
    case 3: return (struct dmac_register_map *)DMAC3_REG_BASE;
    }

    return NULL;
}

static struct dmac_ch_register_map *get_channel(int ch)
{
  struct dmac_register_map *dev = get_device(ch);
  if (dev == NULL)
    {
      return NULL;
    }

  if (is_dmac(2, dev))
    {
      return &dev->channel[ch - 7];
    }
  else if (is_dmac(3, dev))
    {
      return &((struct dmac080_register_map *)dev)->channel[ch - 2];
    }

  return &dev->channel[ch & 1];
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int get_pmid(int ch)
{
  switch (ch)
    {
    case 0: case 1:
        return PM_APP_ADMAC;
    case 2: case 3: case 4: case 5: case 6:
        return PM_APP_IDMAC;
    case 7: case 8:
        return PM_APP_SKDMAC;
    default:
        break; /* may not comes here */
    }

  return 0;
}

struct dmac_ch_register_frame
{
  uint32_t srcaddr;
  uint32_t destaddr;
  uint32_t lli;
  uint32_t control;
  uint32_t configuration;
};

struct dmac_register_frame
{
  uint32_t configuration;
  struct dmac_ch_register_frame channel[2];
};

static void _dmac_intc_handler(int ch)
{
  struct dmac_register_map *dev = get_device(ch);
  struct dma_channel_s *dmach;
  uint32_t mask;
  int itc;
  int err;

  mask = (1u << (ch & 1));

  if (is_dmac(2, dev))
    {
      mask = 1u << (ch - 7);
    }

  else if (is_dmac(3, dev))
    {
      mask = 1u << (ch - 2);
    }

  itc = dev->inttcstatus & mask;
  err = dev->interrorstatus & mask;
  dev->inttcclear = itc;
  dev->interrorclear = err;

  dmach = &g_dmach[ch];

  if (dmach->callback)
    {
      int flags = itc ? CXD56_DMA_INTR_ITC : 0;
      flags |= err ? CXD56_DMA_INTR_ERR : 0;
      dmach->callback((DMA_HANDLE)dmach, flags, dmach->arg);
    }
}

static int intr_handler_admac0(int irq, void *context, void *arg)
{
  _dmac_intc_handler(0);
  return OK;
}

static int intr_handler_admac1(int irq, void *context, void *arg)
{
  _dmac_intc_handler(1);
  return OK;
}

static int intr_handler_idmac(int irq, void *context, void *arg)
{
  struct dmac_register_map *dev = get_device(2); /* XXX */
  uint32_t stat = dev->intstatus & 0x1f;
  int i;

  for (i = 2; stat; i++, stat >>= 1)
    {
      if (stat & 1)
        {
          _dmac_intc_handler(i);
        }
    }

  return OK;
}

static int intr_handler_skdmac0(int irq, void *context, void *arg)
{
  _dmac_intc_handler(7);
  return OK;
}

static int intr_handler_skdmac1(int irq, void *context, void *arg)
{
  _dmac_intc_handler(8);
  return OK;
}

static void controller_power_on(int ch)
{
  int id = get_pmid(ch);

  if (id == PM_APP_SKDMAC)
    {
      return;
    }

  /* TODO power on */
}

static void controller_power_off(int ch)
{
  int id = get_pmid(ch);

  if (id == PM_APP_SKDMAC) /* Do not disable SKDMAC, leave it to SAKE driver. */
    {
      return;
    }

  /* TODO power off */
}

int dma_init(int ch)
{
  int id = ch2dmac(ch);

  if (!id)
    {
      return -ENODEV;
    }

  controller_power_on(ch);

  irq_attach(irq_map[ch], intc_handler[ch], NULL);

  return 0;
}

int dma_uninit(int ch)
{
  int id = ch2dmac(ch);

  if (!id)
    {
      return -ENODEV;
    }

  controller_power_off(ch);

  return 0;
}

int dma_open(int ch)
{
  struct dmac_register_map *dmac = get_device(ch);
  irqstate_t flags;

  if (dmac == NULL)
    {
      return -ENODEV;
    }

  flags = enter_critical_section();

  if (open_channels & (1u << ch))
    {
      leave_critical_section(flags);
      return -EBUSY;
    }

  open_channels |= 1u << ch;

  leave_critical_section(flags);

  g_dmach[ch].callback = NULL;
  g_dmach[ch].arg = NULL;

  dmac->sync = 0;
  dmac->configuration |= 1;

  return 0;
}

static int dma_close(int ch)
{
  struct dmac_register_map *dmac = get_device(ch);
  uint32_t enabled;
  irqstate_t flags;
  uint32_t chmask;
  int shift;

  if (dmac == NULL)
    {
      return -ENODEV;
    }

  shift = ch & 1;

  if (is_dmac(2, dmac))
    {
      shift = ch - 7;
    }
  else if (is_dmac(3, dmac))
    {
        shift = ch - 2;
    }

  enabled = dmac->enbldchns;
  if (enabled & (1 << shift))
    {
      return -EBUSY;
    }

  dma_clearintrcallback(ch);

  flags = enter_critical_section();

  chmask = (3u << (ch & ~1));

  if (is_dmac(2, dmac))
    {
      chmask = 0x3u << 7;
    }
  else if (is_dmac(3, dmac))
    {
        chmask = 0x1fu << 2;
    }

  open_channels &= ~(1u << ch);

  /* Stop device if both of channels are already closed */

  if (!(open_channels & chmask))
    {
      dmac->configuration &= ~1;
    }

  leave_critical_section(flags);

  return 0;
}

static int dma_setconfig(int ch, int itc, int ierr, int flowctrl,
                         int destperi, int srcperi)
{
  struct dmac_ch_register_map *channel = get_channel(ch);
  if (channel == NULL)
    {
      return -ENODEV;
    }

  channel->configuration = (itc & 1) << 15 |
                           (ierr & 1) << 14 |
                           (flowctrl & 7) << 11 |
                           1 << 25 | 1 << 24 | /* Burst enable */
                           (destperi & 0xf) << 6 |
                           (srcperi & 0xf) << 1;

  return 0;
}

static int dma_setintrcallback(int ch, dma_callback_t func, void *data)
{
  if (ch >= NCHANNELS)
    {
      return -ENODEV;
    }

  g_dmach[ch].callback = func;
  g_dmach[ch].arg = data;

  return 0;
}

static int dma_clearintrcallback(int ch)
{
  if (ch >= NCHANNELS)
    {
      return -ENODEV;
    }

  g_dmach[ch].callback = NULL;
  g_dmach[ch].arg = NULL;

  return 0;
}

static int dma_start(int ch, dmac_lli_t *list)
{
  struct dmac_ch_register_map *channel = get_channel(ch);
  if (channel == NULL)
    {
      return -ENODEV;
    }

  if (list)
    {
      channel->srcaddr = list->src_addr;
      channel->destaddr = list->dest_addr;
      channel->lli = list->nextlli;
      channel->control = list->control;
    }

  channel->configuration |= DMAC_CH_ENABLE;

  return 0;
}

static int dma_stop(int ch)
{
  struct dmac_ch_register_map *channel = get_channel(ch);
  if (channel == NULL)
    {
      return -ENODEV;
    }

  if (!(channel->configuration & DMAC_CH_ENABLE))
    {
      return 0; /* already stopped */
    }

  /* Set HALT and poll Active bit for FIFO is cleaned */

  channel->configuration |= DMAC_CH_HALT;

  (void) channel->lli;
  (void) channel->lli;

  while (channel->configuration & DMAC_CH_ACTIVE);

  channel->configuration &= ~(DMAC_CH_HALT | DMAC_CH_ENABLE);

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cxd56_dmainitialize
 *
 * Description:
 *   Initialize the DMA subsystem
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void weak_function arm_dma_initialize(void)
{
  int i;

  dmainfo("Initialize DMAC\n");

  /* Initialize the channel list  */

  for (i = 0; i < NCHANNELS; i++)
    {
      g_dmach[i].chan = i;
#ifndef CONFIG_CXD56_SUBCORE
      up_enable_irq(irq_map[i]);
#endif
    }
}

/****************************************************************************
 * Name: cxd56_dmachannel
 *
 * Description:
 *  Allocate a DMA channel.
 *  This function gives the caller mutually exclusive
 *  access to a DMA channel.
 *
 *  If no DMA channel is available, then cxd56_dmachannel() will wait until
 *  the holder of a channel relinquishes the channel by calling
 *  cxd56_dmafree().
 *
 * Input parameters:
 *  ch      - DMA channel to use
 *  maxsize - Max size to be transferred in bytes
 *
 * Returned Value:
 *  This function ALWAYS returns a non-NULL, void* DMA channel handle.
 *
 * Assumptions:
 *  - The caller can wait for a DMA channel to be freed if it is not
 *    available.
 *
 ****************************************************************************/

DMA_HANDLE cxd56_dmachannel(int ch, ssize_t maxsize)
{
  struct dma_channel_s *dmach;
  int n;

  /* Get exclusive access to allocate channel */

  nxmutex_lock(&g_dmalock);

  if (ch < 0 || ch >= NCHANNELS)
    {
      dmaerr("Invalid channel number %d.\n", ch);
      goto err;
    }

  dmach = &g_dmach[ch];

  if (maxsize == 0)
    {
      dmaerr("Invalid max size: %d\n", maxsize);
      goto err;
    }

  if (dmach->inuse)
    {
      dmaerr("Channel already in use.\n");
      goto err;
    }

  dmainfo("DMA channel %d\n", dmach->chan);

  n = maxsize / CXD56_DMAC_MAX_SIZE;
  if ((maxsize % CXD56_DMAC_MAX_SIZE) != 0)
    {
      n++;
    }

  dmach->list = (dmac_lli_t *)kmm_malloc(n * sizeof(dmac_lli_t));
  if (dmach->list == NULL)
    {
      dmainfo("Failed to kmm_malloc\n");
      goto err;
    }

  /* Initialize hardware */

  dma_init(dmach->chan);
  dma_open(dmach->chan);

  dmach->inuse  = true;

  nxmutex_unlock(&g_dmalock);
  return (DMA_HANDLE)dmach;

err:
  nxmutex_unlock(&g_dmalock);
  return NULL;
}

/****************************************************************************
 * Name: cxd56_dmafree
 *
 * Description:
 *  Release a DMA channel.  If another thread is waiting for this DMA channel
 *  in a call to cxd56_dmachannel, then this function will re-assign the
 *  DMA channel to that thread and wake it up.
 *
 *  NOTE:  The 'handle' used in this argument must NEVER be used again until
 *  cxd56_dmachannel() is called again to re-gain access to the channel.
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *  - The caller holds the DMA channel.
 *  - There is no DMA in progress
 *
 ****************************************************************************/

void cxd56_dmafree(DMA_HANDLE handle)
{
  struct dma_channel_s *dmach = (struct dma_channel_s *)handle;

  if (dmach == NULL)
    {
      dmaerr("Invalid handle.\n");
      return;
    }

  nxmutex_lock(&g_dmalock);

  if (!dmach->inuse)
    {
      dmaerr("Channel %d already freed.\n", dmach->chan);
      goto err;
    }

  dmainfo("free DMA channel %d\n", dmach->chan);

  kmm_free(dmach->list);

  dma_close(dmach->chan);
  dma_uninit(dmach->chan);

  dmach->inuse = false;

err:
  nxmutex_unlock(&g_dmalock);
}

/****************************************************************************
 * Name: cxd56_rxdmasetup
 *
 * Description:
 *   Configure an RX (peripheral-to-memory) DMA before starting the transfer.
 *
 * Input Parameters:
 *   paddr  - Peripheral address (source)
 *   maddr  - Memory address (destination)
 *   nbytes - Number of bytes to transfer.  Must be an even multiple of the
 *            configured transfer size.
 *   config - Channel configuration selections
 *
 ****************************************************************************/

void cxd56_rxdmasetup(DMA_HANDLE handle, uintptr_t paddr, uintptr_t maddr,
                      size_t nbytes, dma_config_t config)
{
  struct dma_channel_s *dmach = (struct dma_channel_s *)handle;
  int i;
  int list_num;
  uintptr_t dst;
  size_t rest;
  int peri;
  int di;

  DEBUGASSERT(dmach != NULL && dmach->inuse && dmach->list != NULL);

  if (maddr)
    {
      dst = maddr;
      di = 1;
    }
  else
    {
      dst = (uintptr_t)&dmach->dummy;
      di = 0;
    }

  dst  = CXD56_PHYSADDR(dst);
  rest = nbytes;

  list_num = (nbytes + CXD56_DMAC_MAX_SIZE - 1) / CXD56_DMAC_MAX_SIZE;
  for (i = 0; i < list_num - 1; i++)
    {
      dmach->list[i].src_addr = paddr;
      dmach->list[i].dest_addr = dst;
      dmach->list[i].nextlli = CXD56_PHYSADDR(&dmach->list[i + 1]);
      dmach->list[i].control = DMAC_EX_CTRL_HELPER(0, di, 0,           /* interrupt / Dest inc / Src inc */
                               CXD56_DMAC_MASTER1, CXD56_DMAC_MASTER2, /* AHB dst master / AHB src master (fixed) */
                               config.dest_width, config.src_width,    /* Dest / Src transfer width */
                               CXD56_DMAC_BSIZE4, CXD56_DMAC_BSIZE4,   /* Dest / Src burst size (fixed) */
                               CXD56_DMAC_MAX_SIZE);

      dst += CXD56_DMAC_MAX_SIZE;
      rest -= CXD56_DMAC_MAX_SIZE;
    }

  dmach->list[i].src_addr = paddr;
  dmach->list[i].dest_addr = dst;
  dmach->list[i].nextlli = 0;
  dmach->list[i].control = DMAC_EX_CTRL_HELPER(1, di, 0,               /* interrupt / Dest inc / Src inc */
                               CXD56_DMAC_MASTER1, CXD56_DMAC_MASTER2, /* AHB dst master / AHB src master (fixed) */
                               config.dest_width, config.src_width,    /* Dest / Src transfer width */
                               CXD56_DMAC_BSIZE4, CXD56_DMAC_BSIZE4,   /* Dest / Src burst size (fixed) */
                               rest);

  peri = config.channel_cfg & CXD56_DMA_PERIPHERAL_MASK;
  dma_setconfig(dmach->chan, 1, 1, CXD56_DMAC_P2M, 0, peri);
}

/****************************************************************************
 * Name: cxd56_txdmasetup
 *
 * Description:
 *   Configure an TX (memory-to-peripheral) DMA before starting the transfer.
 *
 * Input Parameters:
 *   paddr  - Peripheral address (destination)
 *   maddr  - Memory address (source)
 *   nbytes - Number of bytes to transfer.  Must be an even multiple of the
 *            configured transfer size.
 *   config - Channel configuration selections
 *
 ****************************************************************************/

void cxd56_txdmasetup(DMA_HANDLE handle, uintptr_t paddr, uintptr_t maddr,
                      size_t nbytes, dma_config_t config)
{
  struct dma_channel_s *dmach = (struct dma_channel_s *)handle;
  int i;
  int list_num;
  uintptr_t src;
  size_t rest;
  int peri;
  int si;

  DEBUGASSERT(dmach != NULL && dmach->inuse && dmach->list != NULL);

  if (maddr)
    {
      src = maddr;
      si = 1;
    }
  else
    {
      src = (uintptr_t)&dmach->dummy;
      si = 0;
    }

  src  = CXD56_PHYSADDR(src);
  rest = nbytes;

  list_num = (nbytes + CXD56_DMAC_MAX_SIZE - 1) / CXD56_DMAC_MAX_SIZE;
  for (i = 0; i < list_num - 1; i++)
    {
      dmach->list[i].src_addr = src;
      dmach->list[i].dest_addr = paddr;
      dmach->list[i].nextlli = CXD56_PHYSADDR(&dmach->list[i + 1]);
      dmach->list[i].control = DMAC_EX_CTRL_HELPER(0, 0, si,               /* interrupt / Dest inc / Src inc */
                                   CXD56_DMAC_MASTER2, CXD56_DMAC_MASTER1, /* AHB dst master / AHB src master (fixed) */
                                   config.dest_width, config.src_width,    /* Dest / Src transfer width (fixed) */
                                   CXD56_DMAC_BSIZE1, CXD56_DMAC_BSIZE1,   /* Dest / Src burst size (fixed) */
                                   CXD56_DMAC_MAX_SIZE);

      src += CXD56_DMAC_MAX_SIZE;
      rest -= CXD56_DMAC_MAX_SIZE;
    }

  dmach->list[i].src_addr = src;
  dmach->list[i].dest_addr = paddr;
  dmach->list[i].nextlli = 0;
  dmach->list[i].control = DMAC_EX_CTRL_HELPER(1, 0, si,                   /* interrupt / Dest inc / Src inc */
                                   CXD56_DMAC_MASTER2, CXD56_DMAC_MASTER1, /* AHB dst master / AHB src master (fixed) */
                                   config.dest_width, config.src_width,    /* Dest / Src transfer width (fixed) */
                                   CXD56_DMAC_BSIZE4, CXD56_DMAC_BSIZE4,   /* Dest / Src burst size (fixed) */
                                   rest);

  peri = config.channel_cfg & CXD56_DMA_PERIPHERAL_MASK;
  dma_setconfig(dmach->chan, 1, 1, CXD56_DMAC_M2P, peri, 0);
}

/****************************************************************************
 * Name: cxd56_dmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 * Assumptions:
 *   - DMA handle allocated by cxd56_dmachannel()
 *   - No DMA in progress
 *
 ****************************************************************************/

void cxd56_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg)
{
  struct dma_channel_s *dmach = (struct dma_channel_s *)handle;

  DEBUGASSERT(dmach && dmach->inuse);

  /* Save the DMA complete callback info */

  dma_setintrcallback(dmach->chan, callback, arg);
  dma_start(dmach->chan, dmach->list);
}

/****************************************************************************
 * Name: cxd56_dmastop
 *
 * Description:
 *  Cancel the DMA.  After cxd56_dmastop() is called, the DMA channel is
 *  reset and cxd56_dmasetup() must be called before cxd56_dmastart() can be
 *  called again
 *
 * Assumptions:
 *   - DMA handle allocated by cxd56_dmachannel()
 *
 ****************************************************************************/

void cxd56_dmastop(DMA_HANDLE handle)
{
  struct dma_channel_s *dmach = (struct dma_channel_s *)handle;

  DEBUGASSERT(dmach);

  dma_stop(dmach->chan);
  dma_clearintrcallback(dmach->chan);
}
