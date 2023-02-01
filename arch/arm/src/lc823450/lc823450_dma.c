/****************************************************************************
 * arch/arm/src/lc823450/lc823450_dma.c
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
#include <nuttx/semaphore.h>

#include <sys/param.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>
#include <stdlib.h>

#include <nuttx/arch.h>
#include <nuttx/spinlock.h>
#include <nuttx/mutex.h>

#include "arm_internal.h"
#include "lc823450_dma.h"
#include "lc823450_syscontrol.h"
#include <arch/chip/clk.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DMACBASE                        0x40004000
#define DMACINTSTAT                     (DMACBASE + 0x00)
#define DMACINTTCSTAT                   (DMACBASE + 0x04)
#define DMACINTTCCLR                    (DMACBASE + 0x08)
#define DMACINTERRSTAT                  (DMACBASE + 0x10)
#define DMACCHENABLE                    (DMACBASE + 0x1c)
#define DMACINTERRCLR                   (DMACBASE + 0x08)
#define DMACCONFIG                      (DMACBASE + 0x30)
#define DMACCONFIG_EN                   (1 << 0)
#define DMACSRCADDR(ch)                 (DMACBASE + ((ch) * 0x20) + 0x100)
#define DMACDSTADDR(ch)                 (DMACBASE + ((ch) * 0x20) + 0x104)
#define DMACLLI(ch)                     (DMACBASE + ((ch) * 0x20) + 0x108)
#define DMACCTL(ch)                     (DMACBASE + ((ch) * 0x20) + 0x10c)
#define DMACCTL_INT                     (1 << 31)
#define DMACCFG(ch)                     (DMACBASE + ((ch) * 0x20) + 0x110)
#define DMACCFG_H                       (1 << 18)
#define DMACCFG_ACT                     (1 << 17)
#define DMACCFG_ITC                     (1 << 15)
#define DMACCFG_FLOWCTRL_MASK           (7 << 11)
#define DMACCFG_FLOWCTRL_M2M_DMA        (0 << 11)
#define DMACCFG_FLOWCTRL_M2P_DMA        (1 << 11)
#define DMACCFG_FLOWCTRL_P2M_DMA        (2 << 11)
#define DMACCFG_FLOWCTRL_M2P_PER        (5 << 11)
#define DMACCFG_FLOWCTRL_P2M_PER        (6 << 11)
#define DMACCFG_DSTPERI_SHIFT           6
#define DMACCFG_SRCPERI_SHIFT           1
#define DMACCFG_E                       (1 << 0)

#define LC823450_DMA_EN                 (1 << 26)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct lc823450_phydmach_s
{
  sq_queue_t req_q;        /* DMA Request queue */
  bool inprogress;         /* True: DMA is in progress on this channel */
};

struct lc823450_dmach_s
{
  sq_entry_t q_ent;
  uint8_t chn;
  struct lc823450_phydmach_s *phych;
  dma_callback_t callback; /* DMA completion callback function */
  void *arg;               /* Argument to pass to the callback function */
  uint32_t srcaddr;
  uint32_t destaddr;
  uint32_t ctrl;
  size_t nxfrs;
  uint32_t llist;
};

struct lc823450_dma_s
{
  mutex_t lock;            /* For exclusive access to the DMA channel list */

  /* This is the state of each DMA channel */

  int count;
  struct lc823450_phydmach_s phydmach[DMA_CHANNEL_NUM];
  uint32_t reqline_use;
};

static int phydmastart(struct lc823450_phydmach_s *pdmach);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct lc823450_dma_s g_dma =
{
  .lock = NXMUTEX_INITIALIZER,
};
volatile uint8_t g_dma_inprogress;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dma_interrupt_core
 ****************************************************************************/

static int dma_interrupt_core(void *context)
{
  struct lc823450_phydmach_s *pdmach;
  struct lc823450_dmach_s *dmach;
  sq_entry_t *q_ent;
  irqstate_t flags;

  pdmach = (struct lc823450_phydmach_s *)context;

  flags = spin_lock_irqsave(NULL);
  q_ent = pdmach->req_q.tail;
  DEBUGASSERT(q_ent != NULL);
  dmach = (struct lc823450_dmach_s *)q_ent;

  if (dmach->nxfrs == 0)
    {
      /* finish one transfer */

      sq_remlast(&pdmach->req_q);
      spin_unlock_irqrestore(NULL, flags);

      if (dmach->callback)
        dmach->callback((DMA_HANDLE)dmach, dmach->arg, 0);
    }
  else
    {
      spin_unlock_irqrestore(NULL, flags);
    }

  up_disable_clk(LC823450_CLOCK_DMA);
  phydmastart(pdmach);

  return OK;
}

/****************************************************************************
 * Name: dma_interrupt
 ****************************************************************************/

static int dma_interrupt(int irq, void *context, void *arg)
{
  int i;
  uint32_t stat;
  uint32_t err;

  up_enable_clk(LC823450_CLOCK_DMA);

  stat = getreg32(DMACINTTCSTAT);
  err = getreg32(DMACINTERRSTAT);

  putreg32(stat, DMACINTTCCLR);
  putreg32(err, DMACINTERRCLR);

  for (i = 0; i < DMA_CHANNEL_NUM; i++)
    {
      if (stat & (1 << i))
        {
          dma_interrupt_core((void *)&g_dma.phydmach[i]);
        }

      if (err & (1 << i))
        {
          dmaerr("ERROR %d\n", i);
        }
    }

  up_disable_clk(LC823450_CLOCK_DMA);

  return OK;
}

/****************************************************************************
 * Name: phydmastart
 ****************************************************************************/

static int phydmastart(struct lc823450_phydmach_s *pdmach)
{
  irqstate_t flags;
  int trnum;

  struct lc823450_dmach_s *dmach;
  sq_entry_t *q_ent;

  flags = spin_lock_irqsave(NULL);

  q_ent = pdmach->req_q.tail;

  if (!q_ent)
    {
      pdmach->inprogress = 0;
      spin_unlock_irqrestore(NULL, flags);
      return 0;
    }

  dmach = (struct lc823450_dmach_s *)q_ent;

  trnum = MIN(dmach->nxfrs, LC823450_DMA_MAX_TRANSSIZE);

  pdmach->inprogress = 1;
  up_enable_clk(LC823450_CLOCK_DMA);

  /* start DMA */

  putreg32(dmach->srcaddr, DMACSRCADDR(dmach->chn));
  putreg32(dmach->destaddr, DMACDSTADDR(dmach->chn));

  putreg32(dmach->ctrl | (dmach->llist ? 0 : LC823450_DMA_ITC) | trnum,
           DMACCTL(dmach->chn));
  putreg32(dmach->llist, DMACLLI(dmach->chn));

  dmach->nxfrs -= trnum;

  if (dmach->nxfrs)
    {
      if (dmach->ctrl & LC823450_DMA_SRCINC)
        {
          if (dmach->ctrl & LC823450_DMA_SRCWIDTH_WORD)
            {
              dmach->srcaddr += trnum * 4;
            }
          else if (dmach->ctrl & LC823450_DMA_SRCWIDTH_HWORD)
            {
              dmach->srcaddr += trnum * 2;
            }
          else
            {
              dmach->srcaddr += trnum;
            }
        }

      if (dmach->ctrl & LC823450_DMA_DSTINC)
        {
          if (dmach->ctrl & LC823450_DMA_DSTWIDTH_WORD)
            {
              dmach->destaddr += trnum * 4;
            }
          else if (dmach->ctrl & LC823450_DMA_DSTWIDTH_HWORD)
            {
              dmach->destaddr += trnum * 2;
            }
          else
            {
              dmach->destaddr += trnum;
            }
        }
    }

  while (getreg32(DMACCHENABLE) & (1 << dmach->chn));

  while (getreg32(DMACCFG(dmach->chn)) & (DMACCFG_E | DMACCFG_ACT));

  modifyreg32(DMACCFG(dmach->chn), 0, DMACCFG_ITC | DMACCFG_E);

  spin_unlock_irqrestore(NULL, flags);
  return 0;
}

#ifdef DMA_TEST
int test_buf1[4096];
int test_buf2[4096];
static volatile int test_done;

static void dma_done(DMA_HANDLE handle, void *arg, int result)
{
  int i;
  for (i = 0; i < 256; i++)
    llinfo("test_buf2[%d] = %d\n", i, test_buf2[i]);
  test_done = 1;
}

void lc823450_dma_test(void)
{
  int i;
  for (i = 0; i < 256; i++)
    {
      test_buf1[i] = i;
      test_buf2[i] = 0;
    }

  DMA_HANDLE hdma;
  hdma = lc823450_dmachannel(DMA_CHANNEL_VIRTUAL);
  lc823450_dmasetup(hdma, LC823450_DMA_SRCWIDTH_WORD |
                          LC823450_DMA_DSTWIDTH_WORD |
                          LC823450_DMA_DSTINC |
                          LC823450_DMA_SRCINC,
                    test_buf1, test_buf2, 4084);
  lc823450_dmastart(hdma, dma_done, NULL);
  while (!test_done);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_dma_initialize
 ****************************************************************************/

void arm_dma_initialize(void)
{
  int i;

  for (i = 0; i < DMA_CHANNEL_NUM; i++)
    {
      g_dma.phydmach[i].inprogress = 0;
      sq_init(&g_dma.phydmach[i].req_q);
    }

  if (irq_attach(LC823450_IRQ_DMAC, dma_interrupt, NULL) != 0)
    {
      return;
    }

  up_enable_irq(LC823450_IRQ_DMAC);

  /* Clock & Reset */

  modifyreg32(MCLKCNTBASIC, 0, MCLKCNTBASIC_DMAC_CLKEN);
  modifyreg32(MRSTCNTBASIC, 0, MRSTCNTBASIC_DMAC_RSTB);

  /* DMAC enable */

  modifyreg32(DMACCONFIG, 0, DMACCONFIG_EN);

#ifdef DMA_TEST
  lc823450_dma_test();
#endif

  /* clock disable */

  modifyreg32(MCLKCNTBASIC, MCLKCNTBASIC_DMAC_CLKEN, 0);
}

/****************************************************************************
 * Name: lc823450_dmaconfigure
 *
 * Description:
 *   Configure a DMA request.  Each DMA request may have two different DMA
 *   request sources.  This associates one of the sources with a DMA request.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void lc823450_dmaconfigure(uint8_t dmarequest, bool alternate)
{
}

/****************************************************************************
 * Name: lc823450_dmarequest
 *
 * Description:
 *   Configure a DMA request.
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void lc823450_dmarequest(DMA_HANDLE handle, uint8_t dmarequest)
{
  struct lc823450_dmach_s *dmach = (struct lc823450_dmach_s *)handle;
  uint32_t val;
  int req_line;

  /* search for free request line */

  for (req_line = 0; req_line < 16; req_line++)
    {
      if ((g_dma.reqline_use & (1 << req_line)) == 0)
        {
          g_dma.reqline_use |= 1 << req_line;
          break;
        }
    }

  DEBUGASSERT(req_line != 16);

  up_enable_clk(LC823450_CLOCK_DMA);

  /* DREQ Select */

  val = getreg32(DREQ0_3 + (req_line / 4) * 4);
  val &= ~(0xff << ((req_line % 4) * 8));
  val |= dmarequest << ((req_line % 4) * 8);
  putreg32(val, (DREQ0_3 + (req_line / 4) * 4));

  /* source or dest peri request ? */

  val = getreg32(DMACCFG(dmach->chn));
  switch (dmarequest)
    {
      case DMA_REQUEST_UART0RX:
      case DMA_REQUEST_UART1RX:
      case DMA_REQUEST_UART2RX:
      case DMA_REQUEST_SIORX:
        val &= ~(0xf << DMACCFG_SRCPERI_SHIFT);
        val |= req_line << DMACCFG_SRCPERI_SHIFT;
        val |= DMACCFG_FLOWCTRL_P2M_DMA;
        break;
      case DMA_REQUEST_UART0TX:
      case DMA_REQUEST_UART1TX:
      case DMA_REQUEST_UART2TX:
      case DMA_REQUEST_SIOTX:
        val &= ~(0xf << DMACCFG_DSTPERI_SHIFT);
        val |= req_line << DMACCFG_DSTPERI_SHIFT;
        val |= DMACCFG_FLOWCTRL_M2P_DMA;
        break;
      case DMA_REQUEST_USBDEV: /* ??? */
        val &= ~(0xf << DMACCFG_DSTPERI_SHIFT);
        val |= req_line << DMACCFG_DSTPERI_SHIFT;
        val |= req_line << DMACCFG_SRCPERI_SHIFT;
        val |= DMACCFG_FLOWCTRL_M2P_DMA;
        break;
      default:
        dmaerr("ERROR: Not implemetned\n");
        DEBUGPANIC();
    }

  putreg32(val, DMACCFG(dmach->chn));
  up_disable_clk(LC823450_CLOCK_DMA);
}

/****************************************************************************
 * Name: lc823450_dmareauest_dir
 ****************************************************************************/

void lc823450_dmareauest_dir(DMA_HANDLE handle, uint8_t dmarequest, int m2p)
{
  struct lc823450_dmach_s *dmach = (struct lc823450_dmach_s *)handle;
  uint32_t val;

  up_enable_clk(LC823450_CLOCK_DMA);

  val = getreg32(DMACCFG(dmach->chn));

  val &= ~DMACCFG_FLOWCTRL_MASK;
  if (m2p)
    {
      val |= DMACCFG_FLOWCTRL_M2P_DMA;
    }
  else
    {
      val |= DMACCFG_FLOWCTRL_P2M_DMA;
    }

  putreg32(val, DMACCFG(dmach->chn));
  up_disable_clk(LC823450_CLOCK_DMA);
}

/****************************************************************************
 * Name: lc823450_dmachannel
 *
 * Description:
 *   Allocate a DMA channel.  This function sets aside a DMA channel and
 *   gives the caller exclusive access to the DMA channel.
 *
 * Returned Value:
 *   On success, this function returns a non-NULL, void* DMA channel handle.
 *   NULL is returned on any failure.  This function can fail only if no DMA
 *   channel is available.
 *
 ****************************************************************************/

DMA_HANDLE lc823450_dmachannel(int ch)
{
  struct lc823450_dmach_s *dmach = NULL;

  /* Get exclusive access to the GPDMA state structure */

  dmach = (struct lc823450_dmach_s *)
           kmm_zalloc(sizeof(struct lc823450_dmach_s));

  if (dmach)
    {
      dmach->chn = ch;
    }

  return (DMA_HANDLE)dmach;
}

/****************************************************************************
 * Name: lc823450_dmafree
 *
 * Description:
 *   Release a DMA channel.  NOTE:  The 'handle' used in this argument must
 *   NEVER be used again until lc823450_dmachannel() is called again to
 *   re-gain a valid handle.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void lc823450_dmafree(DMA_HANDLE handle)
{
  struct lc823450_dmach_s *dmach = (DMA_HANDLE)handle;

  DEBUGASSERT(dmach != NULL);
  UNUSED(dmach);

  /* Make sure that the DMA channel was properly stopped */

  lc823450_dmastop(handle);

  /* Mark the channel available.  This is an atomic operation and needs no
   * special protection.
   */

  kmm_free(handle);
}

/****************************************************************************
 * Name: lc823450_dmasetup
 *
 * Description:
 *   Configure DMA for one transfer.
 *
 ****************************************************************************/

int lc823450_dmasetup(DMA_HANDLE handle, uint32_t control,
                      uint32_t srcaddr, uint32_t destaddr, size_t nxfrs)
{
  struct lc823450_dmach_s *dmach = (DMA_HANDLE)handle;

  DEBUGASSERT(dmach != NULL);

  dmach->srcaddr = srcaddr;
  dmach->destaddr = destaddr;
  dmach->ctrl = control;
  dmach->nxfrs = nxfrs;
  dmach->llist = 0;

  return OK;
}

int lc823450_dmallsetup(DMA_HANDLE handle, uint32_t control,
                        uint32_t srcaddr, uint32_t destaddr,
                        size_t nxfrs, uint32_t llist)
{
  struct lc823450_dmach_s *dmach = (DMA_HANDLE)handle;

  DEBUGASSERT(dmach != NULL);

  dmach->srcaddr = srcaddr;
  dmach->destaddr = destaddr;
  dmach->ctrl = control;
  dmach->nxfrs = nxfrs;
  dmach->llist = llist;

  return OK;
}

/****************************************************************************
 * Name: lc823450_dmaremain
 ****************************************************************************/

int lc823450_dmaremain(DMA_HANDLE handle)
{
  struct lc823450_dmach_s *dmach = (DMA_HANDLE)handle;

  return getreg32(DMACCTL(dmach->chn)) & 0xfff;
}

/****************************************************************************
 * Name: lc823450_dmastart
 ****************************************************************************/

int lc823450_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg)
{
  struct lc823450_dmach_s *dmach = (DMA_HANDLE)handle;
  irqstate_t flags;

  DEBUGASSERT(dmach != NULL);

  /* select physical channel */

  flags = spin_lock_irqsave(NULL);

  sq_addfirst(&dmach->q_ent, &g_dma.phydmach[dmach->chn].req_q);

  dmach->callback = callback;
  dmach->arg = arg;

  /* Kick DMAC, if not active */

  if (!g_dma.phydmach[dmach->chn].inprogress)
    {
      phydmastart(&g_dma.phydmach[dmach->chn]);
    }

  spin_unlock_irqrestore(NULL, flags);

  return OK;
}

/****************************************************************************
 * Name: lc823450_dmastop
 ****************************************************************************/

void lc823450_dmastop(DMA_HANDLE handle)
{
  struct lc823450_dmach_s *dmach = (DMA_HANDLE)handle;
  struct lc823450_phydmach_s *pdmach;
  irqstate_t flags;

  DEBUGASSERT(dmach != NULL);

  flags = spin_lock_irqsave(NULL);

  modifyreg32(DMACCFG(dmach->chn), DMACCFG_ITC | DMACCFG_E, 0);

  pdmach = &g_dma.phydmach[dmach->chn];
  if (pdmach)
    {
      if (pdmach->inprogress)
        {
          up_disable_clk(LC823450_CLOCK_DMA);
        }

      pdmach->inprogress = 0;
      sq_rem(&dmach->q_ent, &pdmach->req_q);
    }

  spin_unlock_irqrestore(NULL, flags);
}
