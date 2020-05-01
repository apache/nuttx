/****************************************************************************
 * arch/arm/src/samd2l2/sam_dmac.c
 *
 *   Copyright (C) 2015-2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/semaphore.h>

#include "arm_arch.h"
#include "arm_internal.h"
#include "sched/sched.h"
#include "chip.h"

#include "sam_dmac.h"
#include "sam_periphclks.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Condition out the whole file unless DMA is selected in the configuration */

#ifdef CONFIG_SAMD2L2_DMAC

/* If SAMD/L support is enabled, then OS DMA support should also be enabled */

#ifndef CONFIG_ARCH_DMA
#  warning "SAMD2L2 DMA enabled but CONFIG_ARCH_DMA disabled"
#endif

/* Number of additional DMA descriptors in LPRAM */

#ifndef CONFIG_SAMD2L2_DMAC_NDESC
#  define CONFIG_SAMD2L2_DMAC_NDESC 0
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
  uint8_t            dc_chan;     /* DMA channel number (0-15) */
  uint8_t            dc_dir;      /* See enum sam_dmadir_e */
  uint32_t           dc_flags;    /* DMA channel flags */
  dma_callback_t     dc_callback; /* Callback invoked when the DMA completes */
  void              *dc_arg;      /* Argument passed to callback function */
#if CONFIG_SAMD2L2_DMAC_NDESC > 0
  struct dma_desc_s *dc_head;     /* First allocated DMA descriptor */
  struct dma_desc_s *dc_tail;     /* DMA descriptor list tail */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int   sam_takechsem(void);
static inline void sam_givechsem(void);
#if CONFIG_SAMD2L2_DMAC_NDESC > 0
static void   sam_takedsem(void);
static inline void sam_givedsem(void);
#endif
static void   sam_dmaterminate(struct sam_dmach_s *dmach, int result);
static int    sam_dmainterrupt(int irq, void *context, FAR void *arg);
static struct dma_desc_s *sam_alloc_desc(struct sam_dmach_s *dmach);
static struct dma_desc_s *sam_append_desc(struct sam_dmach_s *dmach,
                uint16_t btctrl, uint16_t btcnt,
                uint32_t srcaddr, uint32_t dstaddr);
static void   sam_free_desc(struct sam_dmach_s *dmach);
static size_t sam_maxtransfer(struct sam_dmach_s *dmach);
static uint16_t sam_bytes2beats(struct sam_dmach_s *dmach, size_t nbytes);
static int    sam_txbuffer(struct sam_dmach_s *dmach, uint32_t paddr,
                uint32_t maddr, size_t nbytes);
static int    sam_rxbuffer(struct sam_dmach_s *dmach, uint32_t paddr,
                uint32_t maddr, size_t nbytes);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* These semaphores protect the DMA channel and descriptor tables */

static sem_t g_chsem;
#if CONFIG_SAMD2L2_DMAC_NDESC > 0
static sem_t g_dsem;
#endif

/* This array describes the state of each DMA channel */

static struct sam_dmach_s g_dmach[SAMD2L2_NDMACHAN];

/* NOTE: Using the same address as the base descriptors for writeback
 * descriptors causes TERR and FERR interrupts to be raised immediately after
 * starting DMA. This was tested on SAMD21G18A, and it would appear that the
 * writeback buffer must be located at a different memory address.
 *
 * - Matt Thompson
 */

static struct dma_desc_s g_base_desc[SAMD2L2_NDMACHAN]
  __attribute__ ((section(".lpram"), aligned(16)));
static struct dma_desc_s g_writeback_desc[SAMD2L2_NDMACHAN]
  __attribute__ ((section(".lpram"), aligned(16)));

#if CONFIG_SAMD2L2_DMAC_NDESC > 0
/* Additional DMA descriptors for (optional) multi-block transfer support.
 * Also positioned in LPRAM.
 */

static struct dma_desc_s g_dma_desc[CONFIG_SAMD2L2_DMAC_NDESC]
  __attribute__ ((section(".lpram"), aligned(16)));
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_takechsem() and sam_givechsem()
 *
 * Description:
 *   Used to get exclusive access to the DMA channel table
 *
 ****************************************************************************/

static int sam_takechsem(void)
{
  return nxsem_wait_uninterruptible(&g_chsem);
}

static inline void sam_givechsem(void)
{
  nxsem_post(&g_chsem);
}

/****************************************************************************
 * Name: sam_takedsem() and sam_givedsem()
 *
 * Description:
 *   Used to wait for availability of descriptors in the descriptor table.
 *
 ****************************************************************************/

#if CONFIG_SAMD2L2_DMAC_NDESC > 0
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

  /* Disable the DMA channel */

  flags = enter_critical_section();
  putreg8(dmach->dc_chan, SAM_DMAC_CHID);
  putreg8(0, SAM_DMAC_CHCTRLA);

  /* Reset the DMA channel */

  putreg8(DMAC_CHCTRLA_SWRST, SAM_DMAC_CHCTRLA);

  /* Disable all channel interrupts */

  putreg8(1 << dmach->dc_chan, SAM_DMAC_CHINTENCLR);
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
}

/****************************************************************************
 * Name: sam_dmainterrupt
 *
 * Description:
 *  DMA interrupt handler
 *
 ****************************************************************************/

static int sam_dmainterrupt(int irq, void *context, FAR void *arg)
{
  struct sam_dmach_s *dmach;
  unsigned int chndx;
  uint16_t intpend;

  /* Process all pending channel interrupts */

  while ((intpend = getreg16(SAM_DMAC_INTPEND)) != 0)
    {
      /* Get the channel that generated the interrupt */

      chndx = (intpend & DMAC_INTPEND_ID_MASK) >> DMAC_INTPEND_ID_SHIFT;
      dmach = &g_dmach[chndx];

      /* Clear all pending channel interrupt */

      putreg8(DMAC_INT_ALL, SAM_DMAC_CHINTFLAG);

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
 * Name: sam_addrincr
 *
 * Description:
 *   Peripheral address increment for each beat.
 *
 ****************************************************************************/

#if 0 // Not used
static size_t sam_addrincr(struct sam_dmach_s *dmach)
{
  size_t beatsize;
  size_t stepsize;
  int shift

  /* How bit is one beat? {1,2,4} */

  shift    = (dmach->dc_flags & DMACH_FLAG_BEATSIZE_MASK) >>
             DMACH_FLAG_BEATSIZE_SHIFT;
  beatsize = (1 << shift);

  /* What is the address increment per beat? {1,4,6,...,128} */

  shift    = (dmach->dc_flags & DMACH_FLAG_STEPSIZE_MASK) >>
             DMACH_FLAG_STEPSIZE_SHIFT;
  stepsize = (1 << shift);

  return (beatsize * stepsize);
}
#endif

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
#if CONFIG_SAMD2L2_DMAC_NDESC > 0
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

      for (i = 0; i < CONFIG_SAMD2L2_DMAC_NDESC; i++)
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

#if CONFIG_SAMD2L2_DMAC_NDESC > 0
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

#if CONFIG_SAMD2L2_DMAC_NDESC > 0
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
#if CONFIG_SAMD2L2_DMAC_NDESC > 0
  struct dma_desc_s *next;
#endif

  /* Get the base descriptor pointer */

  desc           = &g_base_desc[dmach->dc_chan];
#if CONFIG_SAMD2L2_DMAC_NDESC > 0
  next           = dmach->dc_head;

  dmach->dc_head = NULL;
  dmach->dc_tail = NULL;
#endif

  /* Nullify the base descriptor */

  memset(desc, 0, sizeof(struct dma_desc_s));

#if CONFIG_SAMD2L2_DMAC_NDESC > 0
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

static size_t sam_maxtransfer(struct sam_dmach_s *dmach)
{
  int beatsize;

  /* The number of bytes per beat is 2**BEATSIZE */

  beatsize = (dmach->dc_flags & DMACH_FLAG_BEATSIZE_MASK) >>
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

static uint16_t sam_bytes2beats(struct sam_dmach_s *dmach, size_t nbytes)
{
  size_t mask;
  int beatsize;
  size_t nbeats;

  /* The number of bytes per beat is 2**BEATSIZE */

  beatsize = (dmach->dc_flags & DMACH_FLAG_BEATSIZE_MASK) >>
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
                        uint32_t maddr, size_t nbytes)
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

  tmp     = (dmach->dc_flags & DMACH_FLAG_BEATSIZE_MASK) >>
             DMACH_FLAG_BEATSIZE_SHIFT;
  btctrl |= tmp << LPSRAM_BTCTRL_BEATSIZE_SHIFT;

  /* See Addressing on page 264 of the datasheet.
   * When increment is used, we have to adjust the address in the descriptor
   * based on the beat count remaining in the block
   */

  /* Set up the Block Transfer Count Register configuration */

  btcnt   = sam_bytes2beats(dmach, nbytes);

  if ((dmach->dc_flags & DMACH_FLAG_MEM_INCREMENT) != 0)
    {
      btctrl |= LPSRAM_BTCTRL_SRCINC;
      maddr += nbytes;
    }

  if ((dmach->dc_flags & DMACH_FLAG_PERIPH_INCREMENT) != 0)
    {
      btctrl |= LPSRAM_BTCTRL_DSTINC;
      paddr += nbytes;
    }

  if ((dmach->dc_flags & DMACH_FLAG_STEPSEL) == DMACH_FLAG_STEPSEL_PERIPH)
    {
      btctrl |= LPSRAM_BTCTRL_STEPSEL;
    }

  tmp     = (dmach->dc_flags & DMACH_FLAG_STEPSIZE_MASK) >>
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
                        uint32_t maddr, size_t nbytes)
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

  tmp     = (dmach->dc_flags & DMACH_FLAG_BEATSIZE_MASK) >>
             DMACH_FLAG_BEATSIZE_SHIFT;
  btctrl |= tmp << LPSRAM_BTCTRL_BEATSIZE_SHIFT;

  /* Set up the Block Transfer Count Register configuration */

  btcnt   = sam_bytes2beats(dmach, nbytes);

  if ((dmach->dc_flags & DMACH_FLAG_PERIPH_INCREMENT) != 0)
    {
      btctrl |= LPSRAM_BTCTRL_SRCINC;
      paddr += nbytes;
    }

  if ((dmach->dc_flags & DMACH_FLAG_MEM_INCREMENT) != 0)
    {
      btctrl |= LPSRAM_BTCTRL_DSTINC;
      maddr += nbytes;
    }

  if ((dmach->dc_flags & DMACH_FLAG_STEPSEL) == DMACH_FLAG_STEPSEL_MEM)
    {
      btctrl |= LPSRAM_BTCTRL_STEPSEL;
    }

  tmp     = (dmach->dc_flags & DMACH_FLAG_STEPSIZE_MASK) >>
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

  /* Initialize global semaphores */

  nxsem_init(&g_chsem, 0, 1);
#if CONFIG_SAMD2L2_DMAC_NDESC > 0
  nxsem_init(&g_dsem, 0, CONFIG_SAMD2L2_DMAC_NDESC);
#endif

  /* Initialized the DMA channel table */

  for (i = 0; i < SAMD2L2_NDMACHAN; i++)
    {
      g_dmach[i].dc_chan = i;
    }

  /* Clear descriptors (this will not be done automatically because they are
   * not in .bss).
   */

  memset(g_base_desc, 0, sizeof(struct dma_desc_s)*SAMD2L2_NDMACHAN);
  memset(g_writeback_desc, 0, sizeof(struct dma_desc_s)*SAMD2L2_NDMACHAN);
#if CONFIG_SAMD2L2_DMAC_NDESC > 0
  memset(g_dma_desc, 0, sizeof(struct dma_desc_s)*CONFIG_SAMD2L2_DMAC_NDESC);
#endif

  /* Enable peripheral clock */

  sam_dmac_enableperiph();

  /* Disable and reset the DMAC */

  putreg16(0, SAM_DMAC_CTRL);
  putreg16(DMAC_CTRL_SWRST, SAM_DMAC_CTRL);

  /* Attach DMA interrupt vector */

  irq_attach(SAM_IRQ_DMAC, sam_dmainterrupt, NULL);

  /* Set the LPRAM DMA descriptor table addresses.  These can only be
   * written when the DMAC is disabled.
   */

  putreg32((uint32_t)g_base_desc, SAM_DMAC_BASEADDR);
  putreg32((uint32_t)g_writeback_desc, SAM_DMAC_WRBADDR);

  /* Enable the DMA controller and all priority levels */

  putreg16(DMAC_CTRL_DMAENABLE | DMAC_CTRL_LVLEN0 | DMAC_CTRL_LVLEN1 |
           DMAC_CTRL_LVLEN2, SAM_DMAC_CTRL);

  /* Enable the IRQ at the NVIC (still disabled at the DMA controller) */

  up_enable_irq(SAM_IRQ_DMAC);
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

DMA_HANDLE sam_dmachannel(uint32_t chflags)
{
  struct sam_dmach_s *dmach;
  irqstate_t flags;
  unsigned int chndx;
  int ret;

  /* Search for an available DMA channel */

  dmach = NULL;
  ret = sam_takechsem();
  if (ret < 0)
    {
      return NULL;
    }

  for (chndx = 0; chndx < SAMD2L2_NDMACHAN; chndx++)
    {
      struct sam_dmach_s *candidate = &g_dmach[chndx];
      if (!candidate->dc_inuse)
        {
          dmach           = candidate;
          dmach->dc_inuse = true;

          /* Set the DMA channel flags */

          dmach->dc_flags = chflags;

          /* Disable the DMA channel */

          flags = enter_critical_section();
          putreg8(chndx, SAM_DMAC_CHID);
          putreg8(0, SAM_DMAC_CHCTRLA);

          /* Reset the channel */

          putreg8(DMAC_CHCTRLA_SWRST, SAM_DMAC_CHCTRLA);

          /* Disable all channel interrupts */

          putreg8(1 << chndx, SAM_DMAC_CHINTENCLR);
          leave_critical_section(flags);
          break;
        }
    }

  sam_givechsem();

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
 *   sam_dmachannel are not used and sam_dmaconfig() is called before each
 *   DMA to configure the DMA channel appropriately.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sam_dmaconfig(DMA_HANDLE handle, uint32_t chflags)
{
  struct sam_dmach_s *dmach = (struct sam_dmach_s *)handle;

  /* Set the new DMA channel flags. */

  dmainfo("chflags: %08x\n",  (int)chflags);
  dmach->dc_flags = chflags;
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

  dmach->dc_flags = 0;
  dmach->dc_inuse = false;
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

#if CONFIG_SAMD2L2_DMAC_NDESC > 0
  dmainfo("dc_head: %p dc_tail: %p\n", dmach->dc_head, dmach->dc_tail);
#endif

  /* The maximum transfer size in bytes depends upon the maximum number of
   * transfers and the number of bytes per transfer.
   */

  maxtransfer = sam_maxtransfer(dmach);

  /* If this is a large transfer, break it up into smaller buffers */

  while (remaining > maxtransfer)
    {
      /* Set up the maximum size transfer */

      ret = sam_txbuffer(dmach, paddr, maddr, maxtransfer);
      if (ret == OK)
        {
          /* Decrement the number of bytes left to transfer */

          remaining -= maxtransfer;

          /* Increment the memory & peripheral address (if it is appropriate
           * do do).
           *
           * REVISIT: What if stepsize is not 1?
           */

          if ((dmach->dc_flags & DMACH_FLAG_PERIPH_INCREMENT) != 0)
            {
              paddr += maxtransfer;
            }

          if ((dmach->dc_flags & DMACH_FLAG_MEM_INCREMENT) != 0)
            {
              maddr += maxtransfer;
            }
        }
    }

  /* Then set up the final buffer transfer */

  if (ret == OK && remaining > 0)
    {
      ret = sam_txbuffer(dmach, paddr, maddr, remaining);
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

#if CONFIG_SAMD2L2_DMAC_NDESC > 0
  dmainfo("dc_head: %p dc_tail: %p\n", dmach->dc_head, dmach->dc_tail);
#endif

  /* The maximum transfer size in bytes depends upon the maximum number of
   * transfers and the number of bytes per transfer.
   */

  maxtransfer = sam_maxtransfer(dmach);

  /* If this is a large transfer, break it up into smaller buffers */

  while (remaining > maxtransfer)
    {
      /* Set up the maximum size transfer */

      ret = sam_rxbuffer(dmach, paddr, maddr, maxtransfer);
      if (ret == OK)
        {
          /* Decrement the number of bytes left to transfer */

          remaining -= maxtransfer;

          /* Increment the memory & peripheral address (if it is appropriate
           * to do so).
           *
           * REVISIT: What if stepsize is not 1?
           */

          if ((dmach->dc_flags & DMACH_FLAG_PERIPH_INCREMENT) != 0)
            {
              paddr += maxtransfer;
            }

          if ((dmach->dc_flags & DMACH_FLAG_MEM_INCREMENT) != 0)
            {
              maddr += maxtransfer;
            }
        }
    }

  /* Then set up the final buffer transfer */

  if (ret == OK && remaining > 0)
    {
      ret = sam_rxbuffer(dmach, paddr, maddr, remaining);
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
  uint8_t ctrla;
  uint32_t chctrlb;
  uint32_t tmp;
  uint8_t qosctrl;
  uint8_t periphqos;
  uint8_t memqos;
  int ret = -EINVAL;

  dmainfo("dmach: %p callback: %p arg: %p\n", dmach, callback, arg);
  DEBUGASSERT(dmach != NULL && dmach->dc_chan < SAMD2L2_NDMACHAN);
  head = &g_base_desc[dmach->dc_chan];

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

      /* Clear any pending interrupts from any previous DMAC transfer. */

      flags = enter_critical_section();
      putreg8(dmach->dc_chan, SAM_DMAC_CHID);
      putreg8(0, SAM_DMAC_CHCTRLA);

      /* Setup the Channel Control B Register
       *
       *   DMAC_CHCTRLB_EVACT_TRIG   - Normal transfer and trigger
       *   DMAC_CHCTRLB_EVIE=0       - No channel input actions
       *   DMAC_CHCTRLB_EVOE=0       - Channel event output disabled
       *   DMAC_CHCTRLB_LVL          - Determined by DMACH_FLAG_PRIORITY
       *   DMAC_CHCTRLB_TRIGSRC      - Determined by DMACH_FLAG_PERIPH_*XTRIG
       *   DMAC_CHCTRLB_TRIGACT_BEAT - One trigger required for beat transfer
       *   DMAC_CHCTRLB_CMD_NOACTION - No action
       */

      chctrlb = DMAC_CHCTRLB_EVACT_TRIG | DMAC_CHCTRLB_TRIGACT_BEAT |
                DMAC_CHCTRLB_CMD_NOACTION;

      tmp = (dmach->dc_flags & DMACH_FLAG_PRIORITY_MASK) >>
             DMACH_FLAG_PRIORITY_SHIFT;
      chctrlb |= tmp << DMAC_CHCTRLB_LVL_SHIFT;

      if (dmach->dc_dir == DMADIR_TX)
        {
          /* Memory to peripheral */

          tmp = (dmach->dc_flags & DMACH_FLAG_PERIPH_TXTRIG_MASK) >>
                 DMACH_FLAG_PERIPH_TXTRIG_SHIFT;
        }
      else
        {
          /* Peripheral to memory */

          DEBUGASSERT(dmach->dc_dir == DMADIR_RX);
          tmp = (dmach->dc_flags & DMACH_FLAG_PERIPH_RXTRIG_MASK) >>
                 DMACH_FLAG_PERIPH_RXTRIG_SHIFT;
        }

      chctrlb |= tmp << DMAC_CHCTRLB_TRIGSRC_SHIFT;
      putreg32(chctrlb, SAM_DMAC_CHCTRLB);

      /* Setup the Quality of Service Control Register
       *
       *   DMAC_QOSCTRL_WRBQOS_DISABLE - Background
       *   DMAC_QOSCTRL_FQOS, DMAC_QOSCTRL_DQOS - Depend on
       *       DMACH_FLAG_PERIPH_QOS and DMACH_FLAG_MEM_QOS
       */

      periphqos = (dmach->dc_flags & DMACH_FLAG_PERIPH_QOS_MASK) >>
                  DMACH_FLAG_PERIPH_QOS_SHIFT;
      memqos    = (dmach->dc_flags & DMACH_FLAG_MEM_QOS_MASK) >>
                  DMACH_FLAG_MEM_QOS_SHIFT;

      if (dmach->dc_dir == DMADIR_TX)
        {
          /* Memory to peripheral */

          qosctrl = (memqos << DMAC_QOSCTRL_FQOS_SHIFT) |
                    (periphqos << DMAC_QOSCTRL_DQOS_SHIFT);
        }
      else
        {
          /* Peripheral to memory */

          DEBUGASSERT(dmach->dc_dir == DMADIR_RX);
          qosctrl = (periphqos << DMAC_QOSCTRL_FQOS_SHIFT) |
                    (memqos << DMAC_QOSCTRL_DQOS_SHIFT);
        }

      putreg8(qosctrl | DMAC_QOSCTRL_WRBQOS_DISABLE, SAM_DMAC_QOSCTRL);

      /* Enable the channel */

      ctrla = DMAC_CHCTRLA_ENABLE;

#ifdef CONFIG_ARCH_FAMILY_SAML21
      if (dmach->dc_flags & DMACH_FLAG_RUNINSTDBY)
        {
          ctrla |= DMAC_CHCTRLA_RUNSTDBY;
        }
#endif

      putreg8(ctrla, SAM_DMAC_CHCTRLA);

      /* Enable DMA channel interrupts */

      putreg8(DMAC_INT_TERR | DMAC_INT_TCMPL, SAM_DMAC_CHINTENSET);
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
  irqstate_t flags;

  /* Sample DMAC registers. */

  flags            = enter_critical_section();

  regs->ctrl       = getreg16(SAM_DMAC_CTRL);       /* Control Register */
  regs->crcctrl    = getreg16(SAM_DMAC_CRCCTRL);    /* CRC Control Register */
  regs->crcdatain  = getreg32(SAM_DMAC_CRCDATAIN);  /* CRC Data Input Register */
  regs->crcchksum  = getreg32(SAM_DMAC_CRCCHKSUM);  /* CRC Checksum Register */
  regs->crcstatus  = getreg8(SAM_DMAC_CRCSTATUS);   /* CRC Status Register */
  regs->dbgctrl    = getreg8(SAM_DMAC_DBGCTRL);     /* Debug Control Register */
  regs->qosctrl    = getreg8(SAM_DMAC_QOSCTRL);     /* Quality of Service Control Register */
  regs->swtrigctrl = getreg32(SAM_DMAC_SWTRIGCTRL); /* Software Trigger Control Register */
  regs->prictrl0   = getreg32(SAM_DMAC_PRICTRL0);   /* Priority Control 0 Register */
  regs->intpend    = getreg16(SAM_DMAC_INTPEND);    /* Interrupt Pending Register */
  regs->intstatus  = getreg32(SAM_DMAC_INTSTATUS);  /* Interrupt Status Register */
  regs->busych     = getreg32(SAM_DMAC_BUSYCH);     /* Busy Channels Register */
  regs->pendch     = getreg32(SAM_DMAC_PENDCH);     /* Pending Channels Register */
  regs->active     = getreg32(SAM_DMAC_ACTIVE);     /* Active Channels and Levels Register */
  regs->baseaddr   = getreg32(SAM_DMAC_BASEADDR);   /* Descriptor Memory Section Base Address Register */
  regs->wrbaddr    = getreg32(SAM_DMAC_WRBADDR);    /* Write-Back Memory Section Base Address Register */
  regs->chid       = getreg8(SAM_DMAC_CHID);        /* Channel ID Register */
  regs->chctrla    = getreg8(SAM_DMAC_CHCTRLA);     /* Channel Control A Register */
  regs->chctrlb    = getreg32(SAM_DMAC_CHCTRLB);    /* Channel Control B Register */
  regs->chintflag  = getreg8(SAM_DMAC_CHINTFLAG);   /* Channel Interrupt Flag Status and Clear Register */
  regs->chstatus   = getreg8(SAM_DMAC_CHSTATUS);    /* Channel Status Register */

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

  dmainfo("%s\n", msg);
  dmainfo("  DMAC Registers:\n");
  dmainfo("         CTRL: %04x      CRCCTRL: %04x"
          "      CRCDATAIN: %08x  CRCCHKSUM: %08x\n",
          regs->ctrl, regs->crcctrl, regs->crcdatain, regs->crcchksum);
  dmainfo("    CRCSTATUS: %02x        DBGCTRL: %02x"
          "          QOSCTRL: %02x       SWTRIGCTRL: %08x\n",
          regs->crcstatus, regs->dbgctrl, regs->qosctrl, regs->swtrigctrl);
  dmainfo("     PRICTRL0: %08x  INTPEND: %04x"
          "     INTSTATUS: %08x     BUSYCH: %08x\n",
          regs->prictrl0, regs->intpend, regs->intstatus, regs->busych);
  dmainfo("       PENDCH: %08x   ACTIVE: %08x"
          "   BASEADDR: %08x    WRBADDR: %08x\n",
          regs->pendch, regs->active, regs->baseaddr, regs->wrbaddr);
  dmainfo("         CHID: %02x       CHCRTRLA: %02x"
          "         CHCRTRLB: %08x   CHINFLAG: %02x\n",
          regs->chid, regs->chctrla, regs->chctrlb, regs->chintflag);
  dmainfo("     CHSTATUS: %02x\n",
          regs->chstatus);
}
#endif /* CONFIG_DEBUG_DMA_INFO */
#endif /* CONFIG_SAMD2L2_DMAC */
