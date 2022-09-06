/****************************************************************************
 * arch/arm/src/samd5e5/sam_dmac.c
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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <arch/samd5e5/chip.h>

#include "arm_internal.h"
#include "sched/sched.h"

#include "sam_dmac.h"
#include "sam_periphclks.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Condition out the whole file unless DMA is selected in the configuration */

#ifdef CONFIG_SAMD5E5_DMAC

/* If SAMD/L support is enabled, then OS DMA support should also be enabled */

#ifndef CONFIG_ARCH_DMA
#  warning "SAMD5E5 DMA enabled but CONFIG_ARCH_DMA disabled"
#endif

/* Number of additional DMA descriptors in LPRAM */

#ifndef CONFIG_SAMD5E5_DMAC_NDESC
#  define CONFIG_SAMD5E5_DMAC_NDESC 0
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The direction of the transfer */

enum sam_dmadir_e
{
  DMADIR_UNKOWN = 0,              /* We don't know the direction of the
                                   * transfer yet */
  DMADIR_TX,                      /* Transmit: Memory to peripheral */
  DMADIR_RX                       /* Receive: Peripheral to memory */
};

/* This structure describes one DMA channel */

struct sam_dmach_s
{
  bool               dc_inuse;    /* TRUE: The DMA channel is in use */
  uint8_t            dc_chan;     /* DMA channel number (0-31) */
  uint8_t            dc_dir;      /* See enum sam_dmadir_e */
  uint32_t           dc_txflags;  /* DMA channel flags for Tx transfers */
  uint32_t           dc_rxflags;  /* DMA channel flags for Rx transfers */
  dma_callback_t     dc_callback; /* Callback invoked when the DMA completes */
  void              *dc_arg;      /* Argument passed to callback function */
#if CONFIG_SAMD5E5_DMAC_NDESC > 0
  struct dma_desc_s *dc_head;     /* First allocated DMA descriptor */
  struct dma_desc_s *dc_tail;     /* DMA descriptor list tail */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#if CONFIG_SAMD5E5_DMAC_NDESC > 0
static void   sam_takedsem(void);
static inline void sam_givedsem(void);
#endif
static void   sam_dmaterminate(struct sam_dmach_s *dmach, int result);
static int    sam_dmainterrupt(int irq, void *context, void *arg);
static struct dma_desc_s *sam_alloc_desc(struct sam_dmach_s *dmach);
static struct dma_desc_s *sam_append_desc(struct sam_dmach_s *dmach,
                uint16_t btctrl, uint16_t btcnt,
                uint32_t srcaddr, uint32_t dstaddr);
static void   sam_free_desc(struct sam_dmach_s *dmach);
static size_t sam_maxtransfer(struct sam_dmach_s *dmach, uint32_t dmaflags);
static uint16_t sam_bytes2beats(struct sam_dmach_s *dmach, uint32_t dmaflags,
                size_t nbytes);
static int    sam_txbuffer(struct sam_dmach_s *dmach, uint32_t paddr,
                uint32_t maddr, uint32_t dmaflags, size_t nbytes);
static int    sam_rxbuffer(struct sam_dmach_s *dmach, uint32_t paddr,
                uint32_t maddr, uint32_t dmaflagsr, size_t nbytes);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* These mutex protect the DMA channel and descriptor tables */

static mutex_t g_chlock;
#if CONFIG_SAMD5E5_DMAC_NDESC > 0
static sem_t g_dsem;
#endif

/* This array describes the state of each DMA channel */

static struct sam_dmach_s g_dmach[SAMD5E5_NDMACHAN];

/* NOTE: Using the same address as the base descriptors for writeback
 * descriptors causes TERR and FERR interrupts to be raised immediately after
 * starting DMA.
 */

static struct dma_desc_s g_base_desc[SAMD5E5_NDMACHAN]
  locate_data(".lpram"), aligned(16);
static struct dma_desc_s g_writeback_desc[SAMD5E5_NDMACHAN]
  locate_data(".lpram"), aligned(16);

#if CONFIG_SAMD5E5_DMAC_NDESC > 0
/* Additional DMA descriptors for (optional) multi-block transfer support.
 * Also positioned in LPRAM.
 */

static struct dma_desc_s g_dma_desc[CONFIG_SAMD5E5_DMAC_NDESC]
  locate_data(".lpram"), aligned(16);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_takedsem() and sam_givedsem()
 *
 * Description:
 *   Used to wait for availability of descriptors in the descriptor table.
 *
 ****************************************************************************/

#if CONFIG_SAMD5E5_DMAC_NDESC > 0
static void sam_takedsem(void)
{
  nxsem_wait_uninterruptible(&g_dsem);
}

static inline void sam_givedsem(void)
{
  nxsem_post(&g_dsem);
}
#endif

/****************************************************************************
 * Name: sam_dmaterminate
 *
 * Description:
 *   Terminate the DMA transfer and disable the DMA channel
 *
 ****************************************************************************/

static void sam_dmaterminate(struct sam_dmach_s *dmach, int result)
{
  irqstate_t flags;
  int chan;

  /* Disable the DMA channel */

  flags = enter_critical_section();
  chan  = dmach->dc_chan;
  putreg8(0, SAM_DMAC_CHCTRLA(chan));

  /* Reset the DMA channel */

  putreg8(DMAC_CHCTRLA_SWRST, SAM_DMAC_CHCTRLA(chan));

  /* Disable all channel interrupts */

  putreg8(1 << dmach->dc_chan, SAM_DMAC_CHINTENCLR(chan));
  leave_critical_section(flags);

  /* Free the DMA descriptor list */

  sam_free_desc(dmach);

  /* Perform the DMA complete callback */

  if (dmach->dc_callback)
    {
      dmach->dc_callback((DMA_HANDLE)dmach, dmach->dc_arg, result);
    }

  dmach->dc_callback = NULL;
  dmach->dc_arg      = NULL;
  dmach->dc_dir      = DMADIR_UNKOWN;
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: sam_dmainterrupt
 *
 * Description:
 *  DMA interrupt handler
 *
 ****************************************************************************/

static int sam_dmainterrupt(int irq, void *context, void *arg)
{
  struct sam_dmach_s *dmach;
  unsigned int chndx;
  uint16_t intpend;
  int chan;

  /* Process all pending channel interrupts */

  while ((intpend = getreg16(SAM_DMAC_INTPEND)) != 0)
    {
      /* Get the channel that generated the interrupt */

      chndx = (intpend & DMAC_INTPEND_ID_MASK) >> DMAC_INTPEND_ID_SHIFT;
      dmach = &g_dmach[chndx];
      chan  = dmach->dc_chan;

      /* Clear all pending channel interrupt */

      putreg8(DMAC_INT_ALL, SAM_DMAC_CHINTFLAG(chan));

      /* Check for transfer error interrupt */

      if ((intpend & DMAC_INTPEND_TERR) != 0)
        {
          /* Yes... Terminate the transfer with an error? */

          sam_dmaterminate(dmach, -EIO);
        }

      /* Check for channel transfer complete interrupt */

      else if ((intpend & DMAC_INTPEND_TCMPL) != 0)
        {
          /* Yes.. Terminate the transfer with success */

          sam_dmaterminate(dmach, OK);
        }

      else if ((intpend & DMAC_INTPEND_TERR) != 0)
        {
          dmaerr("Invalid descriptor. Channel %d\n", chndx);
          sam_dmaterminate(dmach, -EIO);
        }

      /* Check for channel suspend interrupt */

      else if ((intpend & DMAC_INTPEND_SUSP) != 0)
        {
          dmaerr("Channel suspended. Channel %d\n", chndx);

          /* REVISIT: Do we want to do anything here? */
        }
    }

  return OK;
}

/****************************************************************************
 * Name: sam_alloc_desc
 *
 * Description:
 *  Allocate one DMA descriptor.  If the base DMA descriptor list entry is
 *  unused, then that value structure will be returned.  Otherwise, this
 *  function will search for a free descriptor in the g_desc[] list.
 *
 *  NOTE: Descriptor list entries are freed by the DMA interrupt handler.
 *  However, since the setting/clearing of the 'in use' indication is atomic,
 *  no special actions need be performed.  It would be a good thing to add
 *  logic to handle the case where all of the entries are exhausted and we
 *  could wait for some to be freed by the interrupt handler.
 *
 ****************************************************************************/

static struct dma_desc_s *sam_alloc_desc(struct sam_dmach_s *dmach)
{
  struct dma_desc_s *desc;

  /* First check if the base descriptor for the DMA channel is available */

  desc = &g_base_desc[dmach->dc_chan];
  if (desc->srcaddr == 0)
    {
      /* Yes, return a pointer to the base descriptor */

      desc->srcaddr = (uint32_t)-1; /* Any non-zero value */
      return desc;
    }
#if CONFIG_SAMD5E5_DMAC_NDESC > 0
  else
    {
      int i;

      /* Wait if no descriptor is available.  When we get a semaphore count,
       * then there will be at least one free descriptor in the table and
       * it is ours.
       */

      sam_takedsem();

      /* Examine each list entry to find an available one -- i.e., one
       * with srcaddr == 0.  That srcaddr field is set to zero by the DMA
       * transfer complete interrupt handler.  The following should be safe
       * because that is an atomic operation.
       */

      for (i = 0; i < CONFIG_SAMD5E5_DMAC_NDESC; i++)
        {
          desc = &g_dma_desc[i];
          if (desc->srcaddr == 0)
            {
              /* Set the srcaddr to any non-zero value to reserve
               * the descriptor.
               */

              desc->srcaddr = (uint32_t)-1; /* Any non-zero value */

              /* Save a pointer to the first allocated DMA descriptor
               * (for sam_free_desc).  We have to do this because we cannot
               * use the link in the base descriptor; it will be overwritten
               * by the writeback.
               */

              if (dmach->dc_head == NULL)
                {
                  dmach->dc_head = desc;
                }

              return desc;
            }
        }

      /* Because we hold a count from the counting semaphore, the above
       * search loop should always be successful.
       */

      DEBUGPANIC();
    }
#endif

  return NULL;
}

/****************************************************************************
 * Name: sam_append_desc
 *
 * Description:
 *  Allocate and add one descriptor to the DMA channel's descriptor list.
 *
 ****************************************************************************/

static struct dma_desc_s *sam_append_desc(struct sam_dmach_s *dmach,
                                          uint16_t btctrl, uint16_t btcnt,
                                          uint32_t srcaddr, uint32_t dstaddr)
{
  struct dma_desc_s *desc;

  /* Sanity check -- srcaddr == 0 is the indication that the descriptor is
   * unused.  Obviously setting it to zero would break that usage.
   */

  DEBUGASSERT(srcaddr != 0);

  /* Allocate a DMA descriptor */

  desc = sam_alloc_desc(dmach);
  if (desc != NULL)
    {
      /* We have it.  Initialize the new descriptor list entry */

      desc->btctrl   = btctrl;   /* Block Transfer Control Register */
      desc->btcnt    = btcnt;    /* Block Transfer Count Register */
      desc->srcaddr  = srcaddr;  /* Block Transfer Source Address Register */
      desc->dstaddr  = dstaddr;  /* Block Transfer Destination Address Register */
      desc->descaddr = 0;        /* Next Address Descriptor Register */

      /* And then hook it at the tail of the descriptor list */

#if CONFIG_SAMD5E5_DMAC_NDESC > 0
      if (dmach->dc_tail != NULL)
        {
          struct dma_desc_s *prev;

          DEBUGASSERT(desc != g_base_desc[dmach->dc_chan]);

          /* Link the previous tail to the new tail */

          prev->descaddr = (uint32_t)desc;
        }
      else
#endif
        {
          /* There is no previous link.  This is the new head of the list */

          DEBUGASSERT(desc == &g_base_desc[dmach->dc_chan]);
        }

#if CONFIG_SAMD5E5_DMAC_NDESC > 0
      /* In either case, this is the new tail of the list. */

      dmach->dc_tail = desc;
#endif
    }
  else
    {
      dmaerr("Failed to allocate descriptor\n");
    }

  return desc;
}

/****************************************************************************
 * Name: sam_free_desc
 *
 * Description:
 *  Free all descriptors in the DMA channel's descriptor list.
 *
 *  NOTE: Called from the DMA interrupt handler.
 *
 ****************************************************************************/

static void sam_free_desc(struct sam_dmach_s *dmach)
{
  struct dma_desc_s *desc;
#if CONFIG_SAMD5E5_DMAC_NDESC > 0
  struct dma_desc_s *next;
#endif

  /* Get the base descriptor pointer */

  desc           = &g_base_desc[dmach->dc_chan];
#if CONFIG_SAMD5E5_DMAC_NDESC > 0
  next           = dmach->dc_head;

  dmach->dc_head = NULL;
  dmach->dc_tail = NULL;
#endif

  /* Nullify the base descriptor */

  memset(desc, 0, sizeof(struct dma_desc_s));

#if CONFIG_SAMD5E5_DMAC_NDESC > 0
  /* Reset each additional descriptor in the descriptor list (thereby
   * freeing them)
   */

  while (next != NULL)
    {
      desc = next;
      DEBUGASSERT(desc->srcaddr != 0);

      next = (struct dma_desc_s *)desc->descaddr;
      memset(desc, 0, sizeof(struct dma_desc_s));
      sam_givedsem();
    }
#endif
}

/****************************************************************************
 * Name: sam_maxtransfer
 *
 * Description:
 *   Maximum number of bytes that can be sent/received in one transfer
 *
 ****************************************************************************/

static size_t sam_maxtransfer(struct sam_dmach_s *dmach, uint32_t dmaflags)
{
  int beatsize;

  /* The number of bytes per beat is 2**BEATSIZE */

  beatsize = (dmaflags & DMACH_FLAG_BEATSIZE_MASK) >>
              LPSRAM_BTCTRL_STEPSIZE_SHIFT;

  /* Maximum beats is UINT16_MAX */

  return (size_t)UINT16_MAX << beatsize;
}

/****************************************************************************
 * Name: sam_bytes2beats
 *
 * Description:
 *   Convert a count of bytes into a count of beats
 *
 ****************************************************************************/

static uint16_t sam_bytes2beats(struct sam_dmach_s *dmach, uint32_t dmaflags,
                                size_t nbytes)
{
  size_t mask;
  int beatsize;
  size_t nbeats;

  /* The number of bytes per beat is 2**BEATSIZE */

  beatsize = (dmaflags & DMACH_FLAG_BEATSIZE_MASK) >>
              LPSRAM_BTCTRL_STEPSIZE_SHIFT;

  /* The number of beats is then the ceiling of the division */

  mask     = (1 << beatsize) - 1;
  nbeats   = (nbytes + mask) >> beatsize;
  DEBUGASSERT(nbeats <= UINT16_MAX);
  return (uint16_t)nbeats;
}

/****************************************************************************
 * Name: sam_txbuffer
 *
 * Description:
 *   Configure DMA for transmit of one buffer (memory to peripheral).  This
 *   function may be called multiple times to handle large and/or dis-
 *   continuous transfers.
 *
 ****************************************************************************/

static int sam_txbuffer(struct sam_dmach_s *dmach, uint32_t paddr,
                        uint32_t maddr, uint32_t dmaflags, size_t nbytes)
{
  uint16_t btctrl;
  uint16_t btcnt;
  uint16_t tmp;

  DEBUGASSERT(dmach->dc_dir == DMADIR_UNKOWN || dmach->dc_dir == DMADIR_TX);

  /* Set up the Block Transfer Control Register configuration:
   *
   * This are fixed register selections:
   *
   *   LPSRAM_BTCTRL_VALID          - Descriptor is valid
   *   LPSRAM_BTCTRL_EVOSEL_DISABLE - No event output
   *   LPSRAM_BTCTRL_BLOCKACT_INT   - Disable channel and generate interrupt
   *                                  when the last block transfer completes.
   *
   * Other settings come from the channel configuration:
   *
   *   LPSRAM_BTCTRL_BEATSIZE     - Determined by DMACH_FLAG_BEATSIZE
   *   LPSRAM_BTCTRL_SRCINC       - Determined by DMACH_FLAG_MEM_INCREMENT
   *   LPSRAM_BTCTRL_DSTINC       - Determined by DMACH_FLAG_PERIPH_INCREMENT
   *   LPSRAM_BTCTRL_STEPSEL      - Determined by DMACH_FLAG_STEPSEL
   *   LPSRAM_BTCTRL_STEPSIZE     - Determined by DMACH_FLAG_STEPSIZE
   */

  btctrl  = LPSRAM_BTCTRL_VALID | LPSRAM_BTCTRL_EVOSEL_DISABLE |
            LPSRAM_BTCTRL_BLOCKACT_INT;

  tmp     = (dmaflags & DMACH_FLAG_BEATSIZE_MASK) >>
             DMACH_FLAG_BEATSIZE_SHIFT;
  btctrl |= tmp << LPSRAM_BTCTRL_BEATSIZE_SHIFT;

  /* See Addressing on page 264 of the datasheet.
   * When increment is used, we have to adjust the address in the descriptor
   * based on the beat count remaining in the block
   */

  /* Set up the Block Transfer Count Register configuration */

  btcnt = sam_bytes2beats(dmach, dmaflags, nbytes);

  if ((dmaflags & DMACH_FLAG_MEM_INCREMENT) != 0)
    {
      btctrl |= LPSRAM_BTCTRL_SRCINC;
      maddr += nbytes;
    }

  if ((dmaflags & DMACH_FLAG_PERIPH_INCREMENT) != 0)
    {
      btctrl |= LPSRAM_BTCTRL_DSTINC;
      paddr += nbytes;
    }

  if ((dmaflags & DMACH_FLAG_STEPSEL) == DMACH_FLAG_STEPSEL_PERIPH)
    {
      btctrl |= LPSRAM_BTCTRL_STEPSEL;
    }

  tmp     = (dmaflags & DMACH_FLAG_STEPSIZE_MASK) >>
             LPSRAM_BTCTRL_STEPSIZE_SHIFT;
  btctrl |= tmp << LPSRAM_BTCTRL_STEPSIZE_SHIFT;

  /* Add the new descriptor list entry */

  if (!sam_append_desc(dmach, btctrl, btcnt, maddr, paddr))
    {
      return -ENOMEM;
    }

  dmach->dc_dir = DMADIR_TX;
  return OK;
}

/****************************************************************************
 * Name: sam_rxbuffer
 *
 * Description:
 *   Configure DMA for receipt of one buffer (peripheral to memory).  This
 *   function may be called multiple times to handle large and/or dis-
 *   continuous transfers.
 *
 ****************************************************************************/

static int sam_rxbuffer(struct sam_dmach_s *dmach, uint32_t paddr,
                        uint32_t maddr, uint32_t dmaflags, size_t nbytes)
{
  uint16_t btctrl;
  uint16_t btcnt;
  uint16_t tmp;

  DEBUGASSERT(dmach->dc_dir == DMADIR_UNKOWN || dmach->dc_dir == DMADIR_RX);

  /* Set up the Block Transfer Control Register configuration:
   *
   * This are fixed register selections:
   *
   *   LPSRAM_BTCTRL_VALID          - Descriptor is valid
   *   LPSRAM_BTCTRL_EVOSEL_DISABLE - No event output
   *   LPSRAM_BTCTRL_BLOCKACT_INT   - Disable channel and generate interrupt
   *                                  when the last block transfer completes.
   *
   * Other settings come from the channel configuration:
   *
   *   LPSRAM_BTCTRL_BEATSIZE     - Determined by DMACH_FLAG_BEATSIZE
   *   LPSRAM_BTCTRL_SRCINC       - Determined by DMACH_FLAG_PERIPH_INCREMENT
   *   LPSRAM_BTCTRL_DSTINC       - Determined by DMACH_FLAG_MEM_INCREMENT
   *   LPSRAM_BTCTRL_STEPSEL      - Determined by DMACH_FLAG_STEPSEL
   *   LPSRAM_BTCTRL_STEPSIZE     - Determined by DMACH_FLAG_STEPSIZE
   */

  btctrl  = LPSRAM_BTCTRL_VALID | LPSRAM_BTCTRL_EVOSEL_DISABLE |
            LPSRAM_BTCTRL_BLOCKACT_INT;

  tmp     = (dmaflags & DMACH_FLAG_BEATSIZE_MASK) >>
             DMACH_FLAG_BEATSIZE_SHIFT;
  btctrl |= tmp << LPSRAM_BTCTRL_BEATSIZE_SHIFT;

  /* Set up the Block Transfer Count Register configuration */

  btcnt   = sam_bytes2beats(dmach, dmaflags, nbytes);

  if ((dmaflags & DMACH_FLAG_PERIPH_INCREMENT) != 0)
    {
      btctrl |= LPSRAM_BTCTRL_SRCINC;
      paddr += nbytes;
    }

  if ((dmaflags & DMACH_FLAG_MEM_INCREMENT) != 0)
    {
      btctrl |= LPSRAM_BTCTRL_DSTINC;
      maddr += nbytes;
    }

  if ((dmaflags & DMACH_FLAG_STEPSEL) == DMACH_FLAG_STEPSEL_MEM)
    {
      btctrl |= LPSRAM_BTCTRL_STEPSEL;
    }

  tmp     = (dmaflags & DMACH_FLAG_STEPSIZE_MASK) >>
             LPSRAM_BTCTRL_STEPSIZE_SHIFT;
  btctrl |= tmp << LPSRAM_BTCTRL_STEPSIZE_SHIFT;

  /* Add the new descriptor list entry */

  if (!sam_append_desc(dmach, btctrl, btcnt, paddr, maddr))
    {
      return -ENOMEM;
    }

  dmach->dc_dir = DMADIR_RX;
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_dma_initialize
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
  dmainfo("Initialize DMAC\n");
  int i;

  /* Initialize global mutex & semaphore */

  nxmutex_init(&g_chlock);
#if CONFIG_SAMD5E5_DMAC_NDESC > 0
  nxsem_init(&g_dsem, 0, CONFIG_SAMD5E5_DMAC_NDESC);
#endif

  /* Initialized the DMA channel table */

  for (i = 0; i < SAMD5E5_NDMACHAN; i++)
    {
      g_dmach[i].dc_chan = i;
    }

  /* Clear descriptors (this will not be done automatically because they are
   * not in .bss).
   */

  memset(g_base_desc, 0, sizeof(struct dma_desc_s)*SAMD5E5_NDMACHAN);
  memset(g_writeback_desc, 0, sizeof(struct dma_desc_s)*SAMD5E5_NDMACHAN);
#if CONFIG_SAMD5E5_DMAC_NDESC > 0
  memset(g_dma_desc, 0, sizeof(struct dma_desc_s)*CONFIG_SAMD5E5_DMAC_NDESC);
#endif

  /* Enable DMA clocking */

  sam_ahb_dmac_enableperiph();

  /* Disable and reset the DMAC */

  putreg16(0, SAM_DMAC_CTRL);
  putreg16(DMAC_CTRL_SWRST, SAM_DMAC_CTRL);

  /* Attach DMA interrupt vectors */

  irq_attach(SAM_IRQ_DMACH0, sam_dmainterrupt, NULL);
  irq_attach(SAM_IRQ_DMACH1, sam_dmainterrupt, NULL);
  irq_attach(SAM_IRQ_DMACH2, sam_dmainterrupt, NULL);
  irq_attach(SAM_IRQ_DMACH3, sam_dmainterrupt, NULL);
  irq_attach(SAM_IRQ_DMACH4_31, sam_dmainterrupt, NULL);

  /* Set the LPRAM DMA descriptor table addresses.  These can only be
   * written when the DMAC is disabled.
   */

  putreg32((uint32_t)g_base_desc, SAM_DMAC_BASEADDR);
  putreg32((uint32_t)g_writeback_desc, SAM_DMAC_WRBADDR);

  /* Setup the Priority control register for each priority level */
#warning Missing logic

  /* Enable the DMA controller and all priority levels */

  putreg16(DMAC_CTRL_DMAENABLE | DMAC_CTRL_LVLEN0 | DMAC_CTRL_LVLEN1 |
           DMAC_CTRL_LVLEN2, SAM_DMAC_CTRL);

  /* Enable the IRQs at the NVIC (still disabled at the DMA controller) */

  up_enable_irq(SAM_IRQ_DMACH0);
  up_enable_irq(SAM_IRQ_DMACH1);
  up_enable_irq(SAM_IRQ_DMACH2);
  up_enable_irq(SAM_IRQ_DMACH3);
  up_enable_irq(SAM_IRQ_DMACH4_31);
}

/****************************************************************************
 * Name: sam_dmachannel
 *
 * Description:
 *   Allocate a DMA channel.  This function sets aside a DMA channel and
 *   gives the caller exclusive access to the DMA channel.
 *
 *   The naming convention in all of the DMA interfaces is that one side is
 *   the 'peripheral' and the other is 'memory'.  However, the interface
 *   could still be used if, for example, both sides were memory although
 *   the naming would be awkward.
 *
 * Returned Value:
 *   If a DMA channel if the required FIFO size is available, this function
 *   returns a non-NULL, void* DMA channel handle.  NULL is returned on any
 *   failure.
 *
 ****************************************************************************/

DMA_HANDLE sam_dmachannel(uint32_t txflags, uint32_t rxflags)
{
  struct sam_dmach_s *dmach;
  irqstate_t flags;
  unsigned int chndx;
  int ret;

  /* Search for an available DMA channel */

  dmach = NULL;
  ret = nxmutex_lock(&g_chlock);
  if (ret < 0)
    {
      return NULL;
    }

  for (chndx = 0; chndx < SAMD5E5_NDMACHAN; chndx++)
    {
      struct sam_dmach_s *candidate = &g_dmach[chndx];
      if (!candidate->dc_inuse)
        {
          dmach           = candidate;
          dmach->dc_inuse = true;

          /* Set the DMA channel flags */

          dmach->dc_txflags = txflags;
          dmach->dc_rxflags = rxflags;

          /* Disable the DMA channel */

          flags = enter_critical_section();
          putreg8(0, SAM_DMAC_CHCTRLA(chndx));

          /* Reset the channel */

          putreg8(DMAC_CHCTRLA_SWRST, SAM_DMAC_CHCTRLA(chndx));

          /* Disable all channel interrupts */

          putreg8(1 << chndx, SAM_DMAC_CHINTENCLR(chndx));
          leave_critical_section(flags);
          break;
        }
    }

  nxmutex_unlock(&g_chlock);

  dmainfo("chflags: %08x returning dmach: %p\n",  (int)chflags, dmach);
  return (DMA_HANDLE)dmach;
}

/****************************************************************************
 * Name: sam_dmaconfig
 *
 * Description:
 *   There are two channel usage models:  (1) The channel is allocated and
 *   configured in one step.  This is the typical case where a DMA channel
 *   performs a constant role.  The alternative is (2) where the DMA channel
 *   is reconfigured on the fly.  In this case, the chflags provided to
 *   sam_dmachannel are not usedand sam_dmaconfig() is called before each DMA
 *   to configure the DMA channel appropriately.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sam_dmaconfig(DMA_HANDLE handle, uint32_t txflags, uint32_t rxflags)
{
  struct sam_dmach_s *dmach = (struct sam_dmach_s *)handle;

  /* Set the new DMA channel flags. */

  dmainfo("txflags: %08lx  rxflags: %08lx\n",
          (unsigned long)txflags, (unsigned long)rxflags);

  dmach->dc_txflags = txflags;
  dmach->dc_rxflags = rxflags;
}

/****************************************************************************
 * Name: sam_dmafree
 *
 * Description:
 *   Release a DMA channel.  NOTE:  The 'handle' used in this argument must
 *   NEVER be used again until sam_dmachannel() is called again to re-gain
 *   a valid handle.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sam_dmafree(DMA_HANDLE handle)
{
  struct sam_dmach_s *dmach = (struct sam_dmach_s *)handle;

  dmainfo("dmach: %p\n", dmach);
  DEBUGASSERT((dmach != NULL) && (dmach->dc_inuse));

  /* Mark the channel no longer in use.  Clearing the inuse flag is an atomic
   * operation and so should be safe.
   */

  dmach->dc_txflags = 0;
  dmach->dc_rxflags = 0;
  dmach->dc_inuse   = false;
}

/****************************************************************************
 * Name: sam_dmatxsetup
 *
 * Description:
 *   Configure DMA for transmit of one buffer (memory to peripheral).  This
 *   function may be called multiple times to handle large and/or dis-
 *   continuous transfers.  Calls to sam_dmatxsetup() and sam_dmarxsetup()
 *   must not be intermixed on the same transfer, however.
 *
 ****************************************************************************/

int sam_dmatxsetup(DMA_HANDLE handle, uint32_t paddr, uint32_t maddr,
                   size_t nbytes)
{
  struct sam_dmach_s *dmach = (struct sam_dmach_s *)handle;
  ssize_t remaining = (ssize_t)nbytes;
  size_t maxtransfer;
  int ret = OK;

  dmainfo("dmach: %p paddr: %08x maddr: %08x nbytes: %d\n",
          dmach, (int)paddr, (int)maddr, (int)nbytes);
  DEBUGASSERT(dmach);

#if CONFIG_SAMD5E5_DMAC_NDESC > 0
  dmainfo("dc_head: %p dc_tail: %p\n", dmach->dc_head, dmach->dc_tail);
#endif

  /* The maximum transfer size in bytes depends upon the maximum number of
   * transfers and the number of bytes per transfer.
   */

  maxtransfer = sam_maxtransfer(dmach, dmach->dc_txflags);

  /* If this is a large transfer, break it up into smaller buffers */

  while (remaining > maxtransfer)
    {
      /* Set up the maximum size transfer */

      ret = sam_txbuffer(dmach, paddr, maddr, dmach->dc_txflags,
                         maxtransfer);
      if (ret == OK)
        {
          /* Decrement the number of bytes left to transfer */

          remaining -= maxtransfer;

          /* Increment the memory & peripheral address (if it is appropriate
           * do do).
           *
           * REVISIT: What if stepsize is not 1?
           */

          if ((dmach->dc_txflags & DMACH_FLAG_PERIPH_INCREMENT) != 0)
            {
              paddr += maxtransfer;
            }

          if ((dmach->dc_txflags & DMACH_FLAG_MEM_INCREMENT) != 0)
            {
              maddr += maxtransfer;
            }
        }
    }

  /* Then set up the final buffer transfer */

  if (ret == OK && remaining > 0)
    {
      ret = sam_txbuffer(dmach, paddr, maddr, dmach->dc_txflags, remaining);
    }

  return ret;
}

/****************************************************************************
 * Name: sam_dmarxsetup
 *
 * Description:
 *   Configure DMA for receipt of one buffer (peripheral to memory).  This
 *   function may be called multiple times to handle large and/or dis-
 *   continuous transfers.  Calls to sam_dmatxsetup() and sam_dmarxsetup()
 *   must not be intermixed on the same transfer, however.
 *
 ****************************************************************************/

int sam_dmarxsetup(DMA_HANDLE handle, uint32_t paddr, uint32_t maddr,
                   size_t nbytes)
{
  struct sam_dmach_s *dmach = (struct sam_dmach_s *)handle;
  ssize_t remaining = (ssize_t)nbytes;
  size_t maxtransfer;
  int ret = OK;

  dmainfo("dmach: %p paddr: %08x maddr: %08x nbytes: %d\n",
          dmach, (int)paddr, (int)maddr, (int)nbytes);
  DEBUGASSERT(dmach);

#if CONFIG_SAMD5E5_DMAC_NDESC > 0
  dmainfo("dc_head: %p dc_tail: %p\n", dmach->dc_head, dmach->dc_tail);
#endif

  /* The maximum transfer size in bytes depends upon the maximum number of
   * transfers and the number of bytes per transfer.
   */

  maxtransfer = sam_maxtransfer(dmach, dmach->dc_rxflags);

  /* If this is a large transfer, break it up into smaller buffers */

  while (remaining > maxtransfer)
    {
      /* Set up the maximum size transfer */

      ret = sam_rxbuffer(dmach, paddr, maddr, dmach->dc_rxflags,
                         maxtransfer);
      if (ret == OK)
        {
          /* Decrement the number of bytes left to transfer */

          remaining -= maxtransfer;

          /* Increment the memory & peripheral address (if it is appropriate
           * to do so).
           *
           * REVISIT: What if stepsize is not 1?
           */

          if ((dmach->dc_rxflags & DMACH_FLAG_PERIPH_INCREMENT) != 0)
            {
              paddr += maxtransfer;
            }

          if ((dmach->dc_rxflags & DMACH_FLAG_MEM_INCREMENT) != 0)
            {
              maddr += maxtransfer;
            }
        }
    }

  /* Then set up the final buffer transfer */

  if (ret == OK && remaining > 0)
    {
      ret = sam_rxbuffer(dmach, paddr, maddr, dmach->dc_rxflags, remaining);
    }

  return ret;
}

/****************************************************************************
 * Name: sam_dmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 ****************************************************************************/

int sam_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg)
{
  struct sam_dmach_s *dmach = (struct sam_dmach_s *)handle;
  struct dma_desc_s *head;
  irqstate_t flags;
  uint32_t dmaflags;
  uint32_t ctrla;
  uint32_t regval32;
  uint8_t regval8;
  int chan;
  int ret = -EINVAL;

  dmainfo("dmach: %p callback: %p arg: %p\n", dmach, callback, arg);
  DEBUGASSERT(dmach != NULL && dmach->dc_chan < SAMD5E5_NDMACHAN);
  head = &g_base_desc[dmach->dc_chan];
  chan = dmach->dc_chan;

  /* Verify that the DMA has been setup (i.e., at least one entry in the
   * descriptor list, the base entry).
   */

  if (head->srcaddr != 0)
    {
      /* Save the callback info.  This will be invoked when the DMA
       * completes.
       */

      dmach->dc_callback = callback;
      dmach->dc_arg      = arg;

      if (dmach->dc_dir == DMADIR_TX)
        {
          /* Memory to peripheral */

          dmaflags = dmach->dc_txflags;
        }
      else
        {
          /* Peripheral to memory */

          dmaflags = dmach->dc_rxflags;
        }

      /* Clear any pending interrupts from any previous DMAC transfer. */

      flags = enter_critical_section();
      putreg8(0, SAM_DMAC_CHCTRLA(chan));

      /* Setup the Channel Control A Register
       *
       * DMAC_CHCTRLA_ENABLE       - Set later
       * DMAC_CHCTRLA_RUNSTDBY     - Determined by DMACH_FLAG_RUNINSTDBY
       * DMAC_CHCTRLA_TRIGSRC      - Determined by DMACH_FLAG_PERIPH_TRIGSRC
       * DMAC_CHCTRLA_TRIGACT      - Determined by DMACH_FLAG_PERIPH_TRIGACT
       * DMAC_CHCTRLA_BURSTLEN     - Determined by DMACH_FLAG_BURSTLEN
       * DMAC_CHCTRLA_THRESHOLD    - Determined by DMACH_FLAG_THRESHOLD
       */

      ctrla = 0;

      /* Run in STANDBY option */

      if (dmaflags & DMACH_FLAG_RUNINSTDBY)
        {
          ctrla |= DMAC_CHCTRLA_RUNSTDBY;
        }

      /* Trigger source */

      regval32 = (dmaflags & DMACH_FLAG_PERIPH_TRIGSRC_MASK) >>
                  DMACH_FLAG_PERIPH_TRIGSRC_SHIFT;
      ctrla   |= DMAC_CHCTRLA_TRIGSRC(regval32) ;

      /* Trigger action */

      regval32 = (dmaflags & DMAC_CHCTRLA_TRIGACT_MASK) >>
                  DMAC_CHCTRLA_TRIGACT_SHIFT;
      ctrla   |= DMAC_CHCTRLA_TRIGACT(regval32) ;

      /* Burst length */

      regval32 = ((dmaflags & DMACH_FLAG_BURSTLEN_MASK) >>
                  DMACH_FLAG_BURSTLEN_SHIFT) + 1;
      ctrla   |= DMAC_CHCTRLA_BURSTLEN(regval32) ;

      /* Threshold */

      regval32 = ((dmaflags & DMACH_FLAG_BURSTLEN_MASK) >>
                  DMACH_FLAG_BURSTLEN_SHIFT) + 1;
      ctrla   |= DMAC_CHCTRLA_THRESHOLD(regval32) ;

      putreg32(ctrla, SAM_DMAC_CHCTRLA(chan));

      /* Setup the Channel Control B Register
       *
       *   DMAC_CHCTRLB_EVIE=0       - No channel input actions
       *   DMAC_CHCTRLB_EVOE=0       - Channel event output disabled
       *   DMAC_CHCTRLB_TRIGACT_BEAT - One trigger required for beat transfer
       *   DMAC_CHCTRLB_CMD_NOACTION - No action
       */

      putreg32(DMAC_CHCTRLB_CMD_NOACTION, SAM_DMAC_CHCTRLB(chan));

      /* Set up the channel priority register */

      regval8 = (dmaflags & DMACH_FLAG_PRIORITY_MASK) >>
                 DMACH_FLAG_PRIORITY_SHIFT;
      putreg8(DMAC_CHPRILVL(regval8), SAM_DMAC_CHPRILVL(chan));

      /* Setup channel event control register
       *
       *   DMAC_CHEVCTRL_EVACT      - Normal transfer and trigger
       *   DMAC_CHEVCTRL_EVMODE     - Default, block event output selection
       *   DMAC_CHEVCTRL_EVIE       - No channel input actions
       *   DMAC_CHEVCTRL_EVOE       - Channel event output disabled/
       */

      regval8 = DMAC_CHEVCTRL_EVACT_TRIG | DMAC_CHEVCTRL_EVOMODE_DEFAULT;
      putreg8(DMAC_CHPRILVL(regval8), SAM_DMAC_CHEVCTRL(chan));

      /* Enable the channel */

      ctrla  = getreg8(SAM_DMAC_CHCTRLA(chan));
      ctrla |= DMAC_CHCTRLA_ENABLE;
      putreg8(ctrla, SAM_DMAC_CHCTRLA(chan));

      /* Enable DMA channel interrupts */

      putreg8(DMAC_INT_TERR | DMAC_INT_TCMPL, SAM_DMAC_CHINTENSET(chan));
      leave_critical_section(flags);
      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: sam_dmastop
 *
 * Description:
 *   Cancel the DMA.  After sam_dmastop() is called, the DMA channel is
 *   reset and sam_dmarx/txsetup() must be called before sam_dmastart() can
 *   be called again
 *
 ****************************************************************************/

void sam_dmastop(DMA_HANDLE handle)
{
  struct sam_dmach_s *dmach = (struct sam_dmach_s *)handle;
  irqstate_t flags;

  dmainfo("dmach: %p\n", dmach);
  DEBUGASSERT(dmach != NULL);

  flags = enter_critical_section();
  sam_dmaterminate(dmach, -EINTR);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: sam_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by sam_dmachannel()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA_INFO
void sam_dmasample(DMA_HANDLE handle, struct sam_dmaregs_s *regs)
{
  struct sam_dmach_s *dmach = (struct sam_dmach_s *)handle;
  uintptr_t base;
  irqstate_t flags;
  int chan;

  /* Sample DMAC registers. */

  flags            = enter_critical_section();
  chan             = dmach->dc_chan;
  base             = SAM_DMAC_CHAN_BASE(chan);
  regs->chan       = chan;

  regs->ctrl       = getreg16(SAM_DMAC_CTRL);       /* Control Register */
  regs->crcctrl    = getreg16(SAM_DMAC_CRCCTRL);    /* CRC Control Register */
  regs->crcdatain  = getreg32(SAM_DMAC_CRCDATAIN);  /* CRC Data Input Register */
  regs->crcchksum  = getreg32(SAM_DMAC_CRCCHKSUM);  /* CRC Checksum Register */
  regs->crcstatus  = getreg8(SAM_DMAC_CRCSTATUS);   /* CRC Status Register */
  regs->dbgctrl    = getreg8(SAM_DMAC_DBGCTRL);     /* Debug Control Register */
  regs->swtrigctrl = getreg32(SAM_DMAC_SWTRIGCTRL); /* Software Trigger Control Register */
  regs->prictrl0   = getreg32(SAM_DMAC_PRICTRL0);   /* Priority Control 0 Register */
  regs->intpend    = getreg16(SAM_DMAC_INTPEND);    /* Interrupt Pending Register */
  regs->intstatus  = getreg32(SAM_DMAC_INTSTATUS);  /* Interrupt Status Register */
  regs->busych     = getreg32(SAM_DMAC_BUSYCH);     /* Busy Channels Register */
  regs->pendch     = getreg32(SAM_DMAC_PENDCH);     /* Pending Channels Register */
  regs->active     = getreg32(SAM_DMAC_ACTIVE);     /* Active Channels and Levels Register */
  regs->baseaddr   = getreg32(SAM_DMAC_BASEADDR);   /* Descriptor Memory Section Base Address Register */
  regs->wrbaddr    = getreg32(SAM_DMAC_WRBADDR);    /* Write-Back Memory Section Base Address Register */

  regs->chctrla    = getreg32(base + SAM_DMAC_CHCTRLA_OFFSET);  /* Channel Control A Register */
  regs->chctrlb    = getreg8(base + SAM_DMAC_CHCTRLB_OFFSET);   /* Channel Control B Register */
  regs->chprilvl   = getreg8(base + SAM_DMAC_CHINTFLAG_OFFSET); /* Channel Priority Level */
  regs->chevctrl   = getreg8(base + SAM_DMAC_CHINTFLAG_OFFSET); /* Channel Event Control Register */
  regs->chintflag  = getreg8(base + SAM_DMAC_CHINTFLAG_OFFSET); /* Channel Interrupt Flag Status and Clear Register */
  regs->chstatus   = getreg8(base + SAM_DMAC_CHSTATU_OFFSETS);  /* Channel Status Register */

  leave_critical_section(flags);
}
#endif /* CONFIG_DEBUG_DMA_INFO */

/****************************************************************************
 * Name: sam_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by sam_dmachannel()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA_INFO
void sam_dmadump(DMA_HANDLE handle, const struct sam_dmaregs_s *regs,
                 const char *msg)
{
  struct sam_dmach_s *dmach = (struct sam_dmach_s *)handle;
  irqstate_t flags;

  flags            = enter_critical_section();
  dmainfo("%s\n", msg);
  dmainfo("  DMAC Global Registers:\n");
  dmainfo("    CTRL: %04x      CRCCTRL: %04x      "
          "CRCDATAIN: %08x  CRCCHKSUM: %08x\n",
          regs->ctrl, regs->crcctrl, regs->crcdatain, regs->crcchksum);
  dmainfo("    CRCSTATUS: %02x        DBGCTRL: %02x        "
          "SWTRIGCTRL: %08x  PRICTRL0: %08x\n",
          regs->crcstatus, regs->dbgctrl, regs->swtrigctrl, regs->prictrl0);
  dmainfo("    INTPEND: %04x      INTSTATUS: %08x  "
          "BUSYCH: %08x  PENDCH: %08x\n",
          regs->intpend, regs->intstatus, regs->busych, regs->pendch);
  dmainfo("    ACTIVE: %08x  BASEADDR: %08x  WRBADDR: %08x\n",
          regs->active, regs->baseaddr, regs->wrbaddr);

  dmainfo("  DMAC Channel %u Registers:\n", regs->chan);
  dmainfo("     CHCRTRLA: %08x  CHCRTRLB: %02x       "
          "CHPRILVL: %02x        CHPRILVL: %02x\n",
          regs->chctrla, regs->chctrlb, regs->chprilvl, priv->chprilvl);
  dmainfo("     CHINFLAG: %02x        CHSTATUS: %02x\n",
          regs->chintflag, regs->chstatus);
}
#endif /* CONFIG_DEBUG_DMA_INFO */
#endif /* CONFIG_SAMD5E5_DMAC */
