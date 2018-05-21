/****************************************************************************
 * arch/arm/src/imxrt/imxrt_edma.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Portions of the eDMA logic derive from NXP sample code which has a compatible
 * BSD 3-clause license:
 *
 *   Copyright (c) 2015, Freescale Semiconductor, Inc.
 *   Copyright 2016-2017 NXP
 *   All rights reserved
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
#include <semaphore.h>
#include <queue.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "up_arch.h"
#include "cache.h"
#include "up_internal.h"
#include "sched/sched.h"

#include "chip.h"
#include "chip/imxrt_edma.h"
#include "chip/imxrt_dmamux.h"
#include "imxrt_periphclks.h"
#include "imxrt_edma.h"

#ifdef CONFIG_IMXRT_EDMA

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* TCD Alignment.
 *
 * eDMA TCDs must be aligned with the D-Cache line boundaries to facilitate
 * cache operations on the TCDs when the D-Cache is enabled.
 *
 * NOTE:  The TCDs are 32-bytes in length.  We implicitly assume that the
 * D-Cache line size is also 32-bits.  Otherwise, padding would be required
 * at the ends of the TCDS and buffers to protect data after the end of from
 * invalidation.
 */

#ifdef CONFIG_ARMV7M_DCACHE
/* Align to the cache line size which we assume is >= 8 */

#  define EDMA_ALIGN        ARMV7M_DCACHE_LINESIZE
#  define EDMA_ALIGN_MASK   (EDMA_ALIGN-1)
#  define EDMA_ALIGN_UP(n)  (((n) + EDMA_ALIGN_MASK) & ~EDMA_ALIGN_MASK)

#else
/* Special alignment is not required in this case, but we will align to 8-bytes */

#  define EDMA_ALIGN        8
#  define EDMA_ALIGN_MASK   7
#  define EDMA_ALIGN_UP(n)  (((n) + 7) & ~7)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* State of a DMA channel */

enum imxrt_dmastate_e
{
  IMXRT_DMA_IDLE = 0,             /* No DMA in progress */
  IMXRT_DMA_CONFIGURED,           /* DMA configured, but not yet started */
  IMXRT_DMA_ACTIVE                /* DMA has been started and is in progress */
}

/* This structure describes one DMA channel */

struct imxrt_dmach_s
{
  uint8_t chan;                   /* DMA channel number (0-IMXRT_EDMA_NCHANNELS) */
  bool inuse;                     /* true: The DMA channel is in use */
  uint8_t ttype;                  /* Transfer type: M2M, M2P, P2M, or P2P */
  uint8_t state;                  /* Channel state.  See enum imxrt_dmastate_e */
  uint32_t flags;                 /* DMA channel flags */
  dma_callback_t callback;        /* Callback invoked when the DMA completes */
  void *arg;                      /* Argument passed to callback function */
  uint32_t rxaddr;                /* RX memory address */
  size_t rxsize;                  /* Size of RX memory region */

#if CONFIG_IMXRT_EDMA_NTCD > 0
  /* That TCD list is linked through the DLAST SGA field.  The first transfer
   * to be performed is at the head of the list.  Subsequent TCDs are added at
   * the tail of the list.
   */

  struct imxrt_edmatcd_s *head;   /* First TCD in the list */
  struct imxrt_edmatcd_s *tail;   /* Last TCD in the list */
#endif
};

/* This structure describes the state of the eDMA controller */

struct imxrt_edma_s
{
  /* These semaphores protect the DMA channel and descriptor tables */

  sem_t chsem;                    /* Protects channel table */
#if CONFIG_IMXRT_EDMA_NTCD > 0
  sem_t dsem;                     /* Supports wait for free descriptors */
#endif

  /* This array describes each DMA channel */

  struct imxrt_dmach_s dmach[IMXRT_EDMA_NCHANNELS];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The state of the eDMA */

static struct imxrt_edma_s g_edma;

#if CONFIG_IMXRT_EDMA_NTCD > 0
/* This is a singly-linked list of free TCDs */

static sq_queue_t g_tcd_free;

/* This is a pool of pre-allocated TCDs */

static struct imxrt_edmatcd_s g_tcd_pool[CONFIG_IMXRT_EDMA_NTCD]
              __attribute__((aligned(EDMA_ALIGN)));
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_takechsem() and imxrt_givechsem()
 *
 * Description:
 *   Used to get exclusive access to the DMA channel table for channel
 *   allocation.
 *
 ****************************************************************************/

static void imxrt_takechsem(void)
{
  int ret;

  do
    {
      /* Take the semaphore (perhaps waiting) */

      ret = nxsem_wait(&g_edma.chsem);

      /* The only case that an error should occur here is if the wait was
       * awakened by a signal.
       */

      DEBUGASSERT(ret == OK || ret == -EINTR || ret == -ECANCELED);
    }
  while (ret == -EINTR);
}

static inline void imxrt_givechsem(void)
{
  (void)nxsem_post(&g_edma.chsem);
}

/****************************************************************************
 * Name: imxrt_takedsem() and imxrt_givedsem()
 *
 * Description:
 *   Used to wait for availability of descriptors in the descriptor table.
 *
 ****************************************************************************/

#if CONFIG_IMXRT_EDMA_NTCD > 0
static void imxrt_takedsem(void)
{
  int ret;

  do
    {
      /* Take the semaphore (perhaps waiting) */

      ret = nxsem_wait(&g_edma.dsem);

      /* The only case that an error should occur here is if the wait was
       * awakened by a signal.
       */

      DEBUGASSERT(ret == OK || ret == -EINTR || ret == -ECANCELED);
    }
  while (ret == -EINTR);
}

static inline void imxrt_givedsem(void)
{
  (void)nxsem_post(&g_edma.dsem);
}
#endif

/****************************************************************************
 * Name: imxrt_tcd_initialize()
 *
 * Description:
 *   Initialize the TCD free list from the pool of pre-allocated TCDs.
 *
 * Assumptions:
 *   Called early in the initialization sequence so no special protection is
 *   necessary.
 *
 ****************************************************************************/

#if CONFIG_IMXRT_EDMA_NTCD > 0
static inline void imxrt_tcd_initialize(void)
{
  sq_entry_t *tcd;
  int i;

  /* Add each pre-allocated TCD to the tail of the TCD free list */

  sq_init(&g_tcd_free);
  for (i = 0; i < CONFIG_IMXRT_EDMA_NTCD; i++)
    {
      tcd = (sq_entry_t *)&g_tcd_poll[i];
      sq_add_last(tcd, &g_tcd_free);
    }
}
#endif

/****************************************************************************
 * Name: imxrt_getdmach
 *
 * Description:
 *  Read a eDMA channel register
 *
 ****************************************************************************/

static inline uint32_t imxrt_getdmach(struct imxrt_dmach_s *dmach,
                                      unsigned int offset)
{
  return getreg32(IMXRT_EDMA_TCD_BASE(dmach->chan) + offset);
}

/****************************************************************************
 * Name: imxrt_putdmach
 *
 * Description:
 *  Write a value to a eDMA channel register
 *
 ****************************************************************************/

static inline void imxrt_putdmach(struct imxrt_dmach_s *dmach, uint32_t value,
                                  unsigned int offset)
{
  putreg32(value, IMXRT_EDMA_TCD_BASE(dmach->chan) + offset);
}

/****************************************************************************
 * Name: imxrt_tcd_reset
 *
 * Description:
 *  Reset all TCD registers to default values.  'tcd' is an 'overlay' that
 *  may refer either to either the TCD register set or to an in-memory TCD
 *  structure.
 *
 ****************************************************************************/

static void imxrt_tcd_reset(struct imxrt_edmatcd_s *tcd)
{
  /* Reset channel TCD */

  tcd->saddr    = 0;
  tcd->soff     = 0;
  tcd->attr     = 0;
  tcd->nbytes   = 0;
  tcd->slast    = 0;
  tcd->daddr    = 0;
  tcd->doff     = 0;
  tcd->citer    = 0;
  tcd->dlastsga = 0;

  /* Enable auto disable request feature */

  tcd->csr      = EDMA_TCD_CSR_DREQ;
  tcd->biter    = 0;
}

/****************************************************************************
 * Name: imxrt_tcd_configure
 *
 * Description:
 *  Configure all TCD registers to the specified values.  'tcd' is an
 *  'overlay' that may refer either to either the TCD register set or to an
 *  in-memory TCD structure.
 *
 ****************************************************************************/

#ifdef CONFIG_IMXRT_EDMA_NTCD > 0
void imxrt_tcd_configure(struct imxrt_edmatcd_s *tcd,
                         const struct imxrt_edma_xfrconfig_s *config,
                         struct imxrt_edmatcd_s *next)
#else
void imxrt_tcd_configure(struct imxrt_edmatcd_s *tcd,
                         const struct imxrt_edma_xfrconfig_s *config)
#endif
{
  tcd->saddr  = config->saddr;                        /* Source Address */
  tcd->soff   = config->soff;                         /* Signed Source Address Offset */
  tcd->attr   = EDMA_TCD_ATTR_SSIZE(config->ssize) |  /* Transfer Attributes */
                EDMA_TCD_ATTR_DSIZE(config->destTransferSize);
  tcd->nbytes = config->nbytes;                       /* Signed Minor Loop Offset / Byte Count */
  tcd->slast  = tcd->slast;                           /* Last Source Address Adjustment */
  tcd->daddr  = config->daddr;                        /* Destination Address */
  tcd->doff   = config->doff;                         /* Signed Destination Address Offset */
  tcd->citer  = config->iter;                         /* Current Minor Loop Link, Major Loop Count */
  tcd->biter  = config->iter;                         /* Beginning Minor Loop Link, Major Loop Count */

#ifdef CONFIG_IMXRT_EDMA_NTCD > 0
  /* Enable scatter/gather processing */

  if (next != NULL)
    {
      uint16_t regval16;

      /* Set the next TCD address */

      tcd->dlastsga = (uint32_t)next;

      /* Before calling imxrt_tcd_configure or imxrt_dmach_setconfig, the
       * user must call imxrt_tcd_reset or imxrt_dmach_reset which will set
       * DREQ, so must use "|" or "&" rather than "=".
       *
       *  Clear the DREQ bit because scatter gather has been enabled, so the
       *  previous transfer is not the last transfer, and channel request should
       *  be enabled at the next transfer(the next TCD).
       */

      regval16  = tcd->csr;
      regval16 &= ~EDMA_TCD_CSR_DREQ;
      regval16 |= EDMA_TCD_CSR_ESG
      tcd->csr  = regval16;
    }
#endif
}

/****************************************************************************
 * Name: imxrt_tcd_instantiate
 *
 * Description:
 *   Copy an in-memory TCD into eDMA channel TCD registers
 *
 ****************************************************************************/

#ifdef CONFIG_IMXRT_EDMA_NTCD > 0
static void imxrt_tcd_instantiate(struct imxrt_dmach_s *dmach,
                                  const struct imxrt_edmatcd_s *tcd)
{
  uintptr_t base = IMXRT_EDMA_TCD_BASE(dmach->chan);

  /* Push tcd into hardware TCD register */

  putreg32(tcd->saddr,    base + IMXRT_EDMA_TCD_SADDR_OFFSET);
  putreg16(tcd->soff,     base + IMXRT_EDMA_TCD_SOFF_OFFSET);
  putreg16(tcd->attr,     base + IMXRT_EDMA_TCD_ATTR_OFFSET);
  putreg32(tcd->nbytes,   base + IMXRT_EDMA_TCD_NBYTES_ML_OFFSET);
  putreg32(tcd->slast,    base + IMXRT_EDMA_TCD_SLAST_OFFSET);
  putreg32(tcd->daddr,    base + IMXRT_EDMA_TCD_DADDR_OFFSET);
  putreg16(tcd->doff,     base + IMXRT_EDMA_TCD_DOFF_OFFSET);
  putreg16(tcd->citer,    base + IMXRT_EDMA_TCD_CITER_ELINK_OFFSET);
  putreg32(tcd->dlastsga, base + IMXRT_EDMA_TCD_DLASTSGA_OFFSET);

  /* Clear DONE bit first, otherwise ESG cannot be set */

  putreg16(0,             base + IMXRT_EDMA_TCD_CSR_OFFSET);
  putreg16(tcd->csr,      base + IMXRT_EDMA_TCD_CSR_OFFSET);

  putreg16(tcd->biter,    base + IMXRT_EDMA_TCD_BITER_ELINK_OFFSET);
  base->TCD[channel].BITER_ELINKNO = tcd->biter;
}
#endif

/****************************************************************************
 * Name: imxrt_dmaterminate
 *
 * Description:
 *   Terminate the DMA transfer and disable the DMA channel
 *
 ****************************************************************************/

static void imxrt_dmaterminate(struct imxrt_dmach_s *dmach, int result)
{
  struct imxrt_edma_s *dmac = imxrt_controller(dmach);

  /* Disable channel ERROR interrupts */

  regval8         = EDMA_CEEI(chan);
  putreg8(regval8, IMXRT_EDMA_CEEI);

  /* Disable channel IRQ requests */

  regval8         = EDMA_CERQ(chan);
  putreg8(regval8, IMXRT_EDMA_CERQ);

  /* Clear CSR to disable channel. Because if the given channel started,
   * transfer CSR will be not zero. Because if it is the last transfer, DREQ
   * will be set.  If not, ESG will be set.
   */

  regaddr         = IMXRT_EDMA_TCD_CSR(chan);
  putreg16(0, regaddr);

  /* Cancel all next TCD transfer. */

  regaddr         = IMXRT_EDMA_TCD_DLASTSGA(chan);
  putreg16(0, regaddr);

#if CONFIG_IMXRT_EDMA_NTCD > 0
  /* Return all allocated TCDs to the free list */
#warning Missing logic

#endif

  /* If this was an RX DMA (peripheral-to-memory), then invalidate the cache
   * to force reloads from memory.
   */

  if ((dmach->ttype & TTYPE_2P_MASK) == 0)
    {
      arch_invalidate_dcache(dmach->rxaddr, dmach->rxaddr + dmach->rxsize);
    }

  /* Perform the DMA complete callback */

  if (dmach->callback)
    {
      dmach->callback((DMACH_HANDLE)dmach, dmach->arg, result);
    }

  dmach->callback = NULL;
  dmach->arg      = NULL;
  dmach->state    = IMXRT_DMA_IDLE;
}

/****************************************************************************
 * Name: imxrt_dmach_interrupt
 *
 * Description:
 *  DMA interrupt handler
 *
 ****************************************************************************/

static void imxrt_dmach_interrupt(struct imxrt_dmach_s *dmach)
{
  uintptr_t regaddr;
  uint32_t regval32;
  uint16_t regval16;
  uint8_t regval8;
  unsigned int chan;
  int result;

  /* Check for a pending interrupt on this channel */

  chan     = dmach->chan;
  regval32 = getreg32(IMXRT_EDMA_INT);

  if ((regval32 & EDMA_INT(chan)) != 0)
    {
      /* An interrupt is pending.  This should only happen if the channel is
       * active.
       */

      DEBUGASSERT(dmach->state == IMXRT_DMA_ACTIVE);

      /* Yes.. Get the eDMA TCD Control and Status register value. */

      regaddr = IMXRT_EDMA_TCD_CSR(chan);

      /* Check if the transfer is done */

      if ((regaddr & EDMA_TCD_CSR_DONE) != 0)
        {
          /* Clear the pending DONE interrupt status. */

          regval8 = EDMA_CDNE(chan);
          putreg8(regval8, IMXRT_EDMA_CDNE);
          result   = OK;
        }
      else
        {
          /* Check if any errors have occurred. */

          regval32 = getreg32(IMXRT_EDMA_ERR);
          if ((regval32 & EDMA_ERR(n)(chan)) != 0)
            {
              dmaerr("ERROR: eDMA ES=%08lx\n",
                     (unsigned long)getreg32(IMXRT_EDMA_ES));

              /* Clear the pending error interrupt status. */

              regval8 = EDMA_CERR(chan);
              putreg32(regval8, IMXRT_EDMA_CERR);
              result = -EIO;
            }
        }

      /* Clear the pending interrupt */

      regval8 = EDMA_CINT(chan);
      putreg32(regval8, IMXRT_EDMA_CINT);
      result = -EIO;

      /* Terminate the transfer */

      imxrt_dmaterminate(dmach, result);
    }
}

/****************************************************************************
 * Name: imxrt_edma_interrupt
 *
 * Description:
 *  DMA interrupt handler
 *
 ****************************************************************************/

static int imxrt_edma_interrupt(int irq, void *context, FAR void *arg)
{
  struct imxrt_dmach_s *dmach;
  unsigned int chan;

  /* 'arg' should the DMA channel instance.
   *
   * NOTE that there are only 16 vectors for 32 DMA channels.  The 'arg' will
   * always be the lower-numbered DMA channel.  The other DMA channel will
   * have a channel number 16 greater that this one.
   */

  dmach = (struct imxrt_dmach_s *)arg;
  DEBUGASSERT(dmach != NULL);
  chan  = dmach->chan;
  DEBUGASSERT(chan < IMXRT_EDMA_NCHANNELS && dmach == &g_edma.dmach[chan]);

  /* Check for an interrupt on the lower numbered DMA channel */

  imxrt_dmach_interrupt(dmach);

  /* Check for an interrupt on the higher numbered DMA channel */

  chan += 16;
  DEBUGASSERT(chan < IMXRT_EDMA_NCHANNELS);
  dmach = &g_edma.dmach[chan];
  imxrt_dmach_interrupt(dmach);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_dmainitialize
 *
 * Description:
 *   Initialize the DMA subsystem
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void weak_function up_dmainitialize(void)
{
  uintptr_t regaddr;
  uint32_t regval;
  int i;

  dmainfo("Initialize eDMA\n");

  /* Enable peripheral clocking to eDMA and DMAMUX modules */

  imxrt_clockrun_dma();

  /* Configure the eDMA controller */

  regval = getreg32(IMXRT_EDMA_CR);
  regval &= ~(EDMA_CR_EDBG | EDMA_CR_ERCA | EDMA_CR_HOE | EDMA_CR_CLM |
              EDMA_CR_EMLM);

#ifdef CONFIG_IMXRT_EDMA_EDBG
  regval |= EDMA_CR_EDBG;   /* Enable Debug */
#endif
#ifdef CONFIG_IMXRT_EDMA_ERCA
  regval |= EDMA_CR_ERCA;   /* Enable Round Robin Channel Arbitration */
#endif
#ifdef CONFIG_IMXRT_EDMA_HOE
  regval |= EDMA_CR_HOE;    /* Halt On Error */
#endif
#ifdef CONFIG_IMXRT_EDMA_CLM
  regval |= EDMA_CR_CLM;    /*  Continuous Link Mode */
#endif
#ifdef CONFIG_IMXRT_EDMA_EMLIM
  regval |= EDMA_CR_EMLM;   /*  Enable Minor Loop Mapping */
#endif

  putreg32(regval, IMXRT_EDMA_CR);

  /* Initialize data structures */

  memset(&g_edma, 0, sizeof(struct imxrt_edma_s));
  for (i = 0; i < IMXRT_EDMA_NCHANNELS; i++)
    {
      g_edma.dmach[i].chan = i;
    }

  /* Initialize semaphores */

  nxsem_init(&g_edma.chsem, 0, 1);
#if CONFIG_IMXRT_EDMA_NTCD > 0
  nxsem_init(&g_edma.dsem, 0, CONFIG_IMXRT_EDMA_NTCD);

  /* The 'dsem' is used for signaling rather than mutual exclusion and,
   * hence, should not have priority inheritance enabled.
   */

  nxsem_setprotocol(&g_edma.dsem, SEM_PRIO_NONE);

  /* Initialize the list of of free TCDs from the pool of pre-allocated TCDs. */

  imxrt_tcd_initialize();
#endif

  /* Attach DMA interrupt vectors.
   *
   * NOTE that there are only 16 vectors for 32 DMA channels.
   */

  (void)irq_attach(IMXRT_IRQ_EDMA0_16,  imxrt_edma_interrupt, &g_edma.dmach[0]);
  (void)irq_attach(IMXRT_IRQ_EDMA1_17,  imxrt_edma_interrupt, &g_edma.dmach[1]);
  (void)irq_attach(IMXRT_IRQ_EDMA2_18,  imxrt_edma_interrupt, &g_edma.dmach[2]);
  (void)irq_attach(IMXRT_IRQ_EDMA3_19,  imxrt_edma_interrupt, &g_edma.dmach[3]);
  (void)irq_attach(IMXRT_IRQ_EDMA4_20,  imxrt_edma_interrupt, &g_edma.dmach[4]);
  (void)irq_attach(IMXRT_IRQ_EDMA5_21,  imxrt_edma_interrupt, &g_edma.dmach[5]);
  (void)irq_attach(IMXRT_IRQ_EDMA6_22,  imxrt_edma_interrupt, &g_edma.dmach[6]);
  (void)irq_attach(IMXRT_IRQ_EDMA7_23,  imxrt_edma_interrupt, &g_edma.dmach[7]);
  (void)irq_attach(IMXRT_IRQ_EDMA8_24,  imxrt_edma_interrupt, &g_edma.dmach[8]);
  (void)irq_attach(IMXRT_IRQ_EDMA9_25,  imxrt_edma_interrupt, &g_edma.dmach[9]);
  (void)irq_attach(IMXRT_IRQ_EDMA10_26, imxrt_edma_interrupt, &g_edma.dmach[10]);
  (void)irq_attach(IMXRT_IRQ_EDMA11_27, imxrt_edma_interrupt, &g_edma.dmach[11]);
  (void)irq_attach(IMXRT_IRQ_EDMA12_28, imxrt_edma_interrupt, &g_edma.dmach[12]);
  (void)irq_attach(IMXRT_IRQ_EDMA13_29, imxrt_edma_interrupt, &g_edma.dmach[13]);
  (void)irq_attach(IMXRT_IRQ_EDMA14_30, imxrt_edma_interrupt, &g_edma.dmach[14]);
  (void)irq_attach(IMXRT_IRQ_EDMA15_31, imxrt_edma_interrupt, &g_edma.dmach[15]);

  /* Disable all DMA interrupts at the eDMA controller */

  putreg32(0, IMXRT_EDMA_EEEI); /* Disable error interrupts */

  for (i = 0; i < IMXRT_EDMA_NCHANNELS; i++)
    {
      /* Disable all DMA channels and DMA channel interrupts */

      regaddr = IMXRT_EDMA_TCD_CSR(i);
      putreg(0, regaddr);
    }

  /* Enable the IRQ at the NVIC (still disabled at the eDMA controller) */

  up_enable_irq(IMXRT_IRQ_EDMA0_16);
  up_enable_irq(IMXRT_IRQ_EDMA1_17);
  up_enable_irq(IMXRT_IRQ_EDMA2_18);
  up_enable_irq(IMXRT_IRQ_EDMA3_19);
  up_enable_irq(IMXRT_IRQ_EDMA4_20);
  up_enable_irq(IMXRT_IRQ_EDMA5_21);
  up_enable_irq(IMXRT_IRQ_EDMA6_22);
  up_enable_irq(IMXRT_IRQ_EDMA7_23);
  up_enable_irq(IMXRT_IRQ_EDMA8_24);
  up_enable_irq(IMXRT_IRQ_EDMA9_25);
  up_enable_irq(IMXRT_IRQ_EDMA10_26);
  up_enable_irq(IMXRT_IRQ_EDMA11_27);
  up_enable_irq(IMXRT_IRQ_EDMA12_28);
  up_enable_irq(IMXRT_IRQ_EDMA13_29);
  up_enable_irq(IMXRT_IRQ_EDMA14_30);
  up_enable_irq(IMXRT_IRQ_EDMA15_31);
}

/****************************************************************************
 * Name: imxrt_dmachannel
 *
 *   Allocate a DMA channel.  This function sets aside a DMA channel,
 *   initializes the DMAMUX for the channel, then gives the caller exclusive
 *   access to the DMA channel.
 *
 * Input Parameters:
 *   dmamux - DMAMUX configuration see DMAMUX channel configuration register
 *            bit-field definitions in chip/imxrt_dmamux.h.  Settings include:
 *
 *            DMAMUX_CHCFG_SOURCE     Chip-specific DMA source (required)
 *            DMAMUX_CHCFG_AON        DMA Channel Always Enable (optional)
 *            DMAMUX_CHCFG_TRIG       DMA Channel Trigger Enable (optional)
 *            DMAMUX_CHCFG_ENBL       DMA Mux Channel Enable (required)
 *
 *            A value of zero will disable the DMAMUX channel.
 *
 * Returned Value:
 *   If a DMA channel is available, this function returns a non-NULL, void*
 *   DMA channel handle.  NULL is returned on any failure.
 *
 ****************************************************************************/

DMACH_HANDLE imxrt_dmachannel(uint32_t dmamux)
{
  struct imxrt_dmach_s *dmach;
  unsigned int chndx;

  /* Search for an available DMA channel */

  dmach = NULL;
  imxrt_takechsem();
  for (chndx = 0; chndx < SAM_NDMACHAN; chndx++)
    {
      struct imxrt_dmach_s *candidate = &g_edma.dmach[chndx];
      uintptr_t regaddr;

      if (!candidate->inuse)
        {
          dmach        = candidate;
          dmach->inuse = true;
          dmach->state = IMXRT_DMA_IDLE;

          /* Clear any pending interrupts on the channel */

          DEBUASSERT(chndx == dmach->chan);
          regaddr = IMXRT_EDMA_TCD_CSR(chndx);
          putreg(0, regaddr);

          /* Make sure that the channel is disabled. */

          regval8 = EDMA_CERQ(chndx);
          putreg8(reqval8, IMXRT_EDMA_CERQ);

          /* Set the DMAMUX register associated with this channel */

          regaddr = IMXRT_DMAMUX_CHCF(chndx);
          putreg32(dmamux, regaddr);
          break;
        }
    }

  imxrt_givechsem();

  /* Show the result of the allocation */

  if (dmach != NULL)
    {
      dmainfo("CH%d: returning dmach: %p\n", dmach->chan, dmach);
    }
  else
    {
      dmaerr("ERROR: Failed allocate eDMA channel\n");
    }

  return (DMACH_HANDLE)dmach;
}

/****************************************************************************
 * Name: imxrt_dmafree
 *
 * Description:
 *   Release a DMA channel.  NOTE:  The 'handle' used in this argument must
 *   NEVER be used again until imxrt_dmachannel() is called again to re-gain
 *   a valid handle.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void imxrt_dmafree(DMACH_HANDLE handle)
{
  struct imxrt_dmach_s *dmach = (struct imxrt_dmach_s *)handle;
  uintptr_t regaddr;
  uint8_t regval8;

  dmainfo("dmach: %p\n", dmach);
  DEBUGASSERT(dmach != NULL && dmach->inuse && dmach->state != IMXRT_DMA_ACTIVE);

  /* Mark the channel no longer in use.  Clearing the inuse flag is an atomic
   * operation and so should be safe.
   */

  dmach->flags = 0;
  dmach->inuse = false;                   /* No longer in use */
  dmach->state = IMXRT_DMA_IDLE;          /* Better not be active! */

  /* Make sure that the channel is disabled. */

  regval8 = EDMA_CERQ(chndx);
  putreg8(reqval8, IMXRT_EDMA_CERQ);

  /* Disable the associated DMAMUX */

  regaddr = IMXRT_DMAMUX_CHCF(chndx);
  putreg32(0, regaddr);
}

/****************************************************************************
 * Name: imxrt_tcd_alloc
 *
 * Description:
 *   Allocate an in-memory, TCD
 *
 ****************************************************************************/

#if CONFIG_IMXRT_EDMA_NTCD > 0
struct imxrt_edmatcd_s *imxrt_tcd_alloc(void)
{
  struct imxrt_edmatcd_s *tcd;
  irqstate_t flags;

  /* Take the 'dsem'.  When we hold the the 'dsem', then we know that one
   * TCD is reserved for us in the free list.
   *
   * NOTE: We use a critical section here because we may block waiting for
   * the 'dsem'.  The critical section will be suspended while we are
   * waiting.
   */

  flags = enter_critical_section();
  imxrt_takedsem();

  /* Now there should be a TCD in the free list reserved just for us */

  tcd = (struct imxrt_edmatcd_s *)sq_remfirst(&g_tcd_free);
  DEBUGASSERT(tcd != NULL);

  leave_critical_section(flags);
  return tcd;
}
#endif

/****************************************************************************
 * Name: imxrt_tcd_free()
 *
 * Description:
 *   Free an in-memory, TCD
 *
 ****************************************************************************/

#if CONFIG_IMXRT_EDMA_NTCD > 0
void imxrt_tcd_free(struct imxrt_edmatcd_s *tcd)
{
  irqstate_t flags;

  /* Add the the TCD to the end of the free list and post the 'dsem',
   * possibly waking up another thread that might be waiting for
   * a TCD.
   */

  flags = spin_lock_irqsave();
  sq_add_last((sq_entry_t *)tcd, &g_tcd_free);
  (void)imxrt_givedsem();
  spin_unlock_irqrestore(flags);
}
#endif

/************************************************************************************
 * Name: imxrt_dmach_reset
 *
 * Description:
 *   Sets all TCD registers to default values..
 *
 *   NOTE:  This function enables the auto stop request feature.
 *
 ************************************************************************************/

void imxrt_dmach_reset(DMACH_HANDLE handle)
{
  struct imxrt_dmach_s *dmach = (struct imxrt_dmach_s *)handle;

  dmainfo("dmach: %p\n", dmach);
  DEBUGASSERT(dmach != NULL && dmach->inuse && dmach->state != IMXRT_DMA_ACTIVE);

  imxrt_tcd_reset((struct imxrt_edmatcd_s *)IMXRT_EDMA_TCD_BASE(dmach->chan));
}

/*******************************************************************************
 * Name: imxrt_dmach_initconfig
 *
 * Description:
 *   This function initializes the transfer configuration structure according
 *   to the user-provided input configuration.
 *
 * Input Parameters:
 *   saddr     - eDMA transfer source address.
 *   srcwidth  - eDMA transfer source address width(bytes).
 *   daddr     - eDMA transfer destination address.
 *   destwidth - eDMA transfer destination address width(bytes).
 *   reqsize   - eDMA transfer bytes per channel request.
 *   nbytes    - eDMA transfer bytes to be transferred.
 *   type      - eDMA transfer type.
 *   config    - The user configuration structure of type struct
 *               imxrt_edma_xfrconfig_s.
 *
 *   NOTE: The data address and the data width must be consistent. For example,
 *   if the SRC is 4 bytes, the source address must be 4 bytes aligned, or it
 *   results in  source address error (SAE).
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ******************************************************************************/

int imxrt_dmach_initconfig(void *saddr, uint32_t srcwidth,
                           void *daddr, uint32_t destwidth,
                           uint32_t reqsize, uint32_t nbytes,
                           edma_transfer_type_t type,
                           struct imxrt_edma_xfrconfig_s *config)
{
  config->daddr  = (uint32_t)daddr;
  config->saddr  = (uint32_t)saddr;
  config->nbytes = reqsize;
  config->iter   = nbytes / reqsize;

  switch (srcwidth)
    {
      case 1:
        config->ssize = TCD_ATTR_SIZE_8BIT;
        break;

      case 2:
        config->ssize = TCD_ATTR_SIZE_16BIT;
        break;

      case 4:
        config->ssize = TCD_ATTR_SIZE_32BIT;
        break;

      case 8:
        config->ssize = TCD_ATTR_SIZE_64BIT;
        break;

      case 32:
        config->ssize = TCD_ATTR_SIZE_256BIT;
        break;

      default:
        return -EINVAL;
    }

  switch (destwidth)
    {
        case 1:
            config->dsize = TCD_ATTR_SIZE_8BIT;
            break;

        case 2:
            config->dsize = TCD_ATTR_SIZE_16BIT;
            break;

        case 4U
            config->dsize = TCD_ATTR_SIZE_32BIT;
            break;

        case 8:
            config->dsize = TCD_ATTR_SIZE_64BIT;
            break;

        case 32:
            config->dsize = TCD_ATTR_SIZE_256BIT;
            break;

      default:
        return -EINVAL;
    }

  switch (type)
    {
      case kEDMA_MemoryToMemory:
        config->doff = destwidth;
        config->soff = srcwidth;
        break;

      case kEDMA_MemoryToPeripheral:
        config->doff = 0;
        config->soff = srcwidth;
        break;

      case kEDMA_PeripheralToMemory:
        config->doff = destwidth;
        config->soff = 0;
        break;

      default:
        return -EINVAL;
    }

  return OK;
}

/*******************************************************************************
 * Name: imxrt_dmach_setconfig
 *
 * Description:
 *   This function configures the transfer attribute, including source address,
 *   destination address, transfer size, address offset, and so on. It also
 *   configures the scatter gather feature if the user supplies the TCD address.
 *
 *   Example:
 *
 *      edma_transfer_t config;
 *      struct imxrt_edmatcd_s tcd;
 *      config.saddr = ..;
 *      config.daddr = ..;
 *      ...
 *      dmach = imxrt_dmachannel(dmamux);
 *      ...
 *      tcd = imxrt_tcd_alloc(dmach)
 *      ...
 *      imxrt_dmach_setconfig(dmach, &config, &tcd);
 *
 * Input Parameters:
 *   handle  - DMA channel handle created by imxrt_dmachannel()
 *   channel - eDMA channel number.
 *   config  - Pointer to eDMA transfer configuration structure.
 *   next    - Points to a TCD structure previously allocated via
 *             imxrt_tcd_alloc(). 'next' can be NULL if the caller does not
 *             wish to enable scatter/gather feature.
 *
 *   NOTE: If 'next' is not NULL, it means scatter gather feature is enabled
 *         and DREQ bit is cleared in the previous transfer configuration.
 *         That bit was set in imxrt_dmach_reset().
 *
 ******************************************************************************/

#ifdef CONFIG_IMXRT_EDMA_NTCD > 0
void imxrt_dmach_setconfig(DMACH_HANDLE handle,
                          const struct imxrt_edma_xfrconfig_s *config,
                          struct imxrt_edmatcd_s *next)
{
  imxrt_tcd_configure((struct imxrt_edmatcd_s *)IMXRT_EDMA_TCD_BASE(dmach->chan)),
                      config, next);
}
#else
void imxrt_dmach_setconfig(DMACH_HANDLE handle,
                          const struct imxrt_edma_xfrconfig_s *config)
{
  imxrt_tcd_configure((struct imxrt_edmatcd_s *)IMXRT_EDMA_TCD_BASE(dmach->chan)),
                      config);
}
#endif

/************************************************************************************
 * Name: imxrt_dmasetup
 *
 * Description:
 *   Configure DMA for one Rx (peripheral-to-memory) or Rx (memory-to-peripheral)
 *   transfer of one buffer.
 *
 *   TODO:  This function needs to be called multiple times to handle multiple,
 *   discontinuous transfers.
 *
 ************************************************************************************/

int imxrt_dmasetup(DMACH_HANDLE handle, uint32_t saddr, uint32_t daddr,
                   size_t nbytes, uint32_t chflags);
{
  struct imxrt_dmach_s *dmach = (struct imxrt_dmach_s *)handle;
  int ret = OK;

  DEBUGASSERT(dmach != NULL);
  dmainfo("dmach%u: %p saddr: %08lx maddr: %08lx nbytes: %lu chflags %08x\n",
          dmach, dmach->chan, (unsigned long)pchan, (unsigned long)maddr,
          (unsigned long)nbytes, (unsigned int)chflags);

  /* To initialize the eDMA:
   *
   *   1. Write to the CR if a configuration other than the default is desired.
   *
   *      We always use the global default setup by up_dmainitialize()
   *
   *   2. Write the channel priority levels to the DCHPRIn registers if a
   *      configuration other than the default is desired.
   *   3. Enable error interrupts in the EEI register if so desired.
   *   4. Write the 32-byte TCD for each channel that may request service.
   *
   *      To perform a simple transfer of n bytes of data with one activation, set
   *      the major loop to one (TCDn_CITER = TCDn_BITER = 1). The data transfer
   *      begins after the channel service request is acknowledged and the channel
   *      is selected to execute. After the transfer is complete, the TCDn_CSR[DONE]
   *      bit is set and an interrupt generates if properly enabled.
   *
   *   5. Enable any hardware service requests via the ERQ register.
   *   6. Request channel service via either:
   *      - Software: setting the TCDn_CSR[START]
   *      - Hardware: slave device asserting its eDMA peripheral request signal
   *
   * This function performs steps 1-5.  Step 6 is performed separately by
   * imxrt_dmastart().
   */
#warning Missing logic

  /* Check for an Rx (memory-to-peripheral) DMA transfer */

  dmach->ttype = (chflags & DMACH_FLAG_TTYPE_MASK) >> DMACH_FLAG_TTYPE_SHIFT;
  if (dmach->ttype == TTYPE_P2M)
    {
      /* Save an information so that the DMA interrupt completion logic will
       * will be able to invalidate the cache after the Rx DMA.
       */

      dmach->rxaddr = maddr;
      dmach->rxsize = nbytes;
    }

  /* Check for an Tx (peripheral-to-memory) DMA transfer */

  else if (dmach->ttype != TTYPE_M2P)
    {
      dmaerr("ERROR: Unsupported ttype: %u\n", dmach->ttype);
      return -ENOSYS;
    }

  dmach->state = IMXRT_DMA_CONFIGURED;

  /* Clean caches associated with the DMA memory */

  arch_clean_dcache(maddr, maddr + nbytes);
  return ret;
}

/****************************************************************************
 * Name: imxrt_dmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 ****************************************************************************/

int imxrt_dmastart(DMACH_HANDLE handle, dma_callback_t callback, void *arg)
{
  struct imxrt_dmach_s *dmach = (struct imxrt_dmach_s *)handle;
  irqstate_t flags;
  uintptr_t regaddr;
  uint16_t regval16;
  uint8_t regval8;
  uint8_t chan;

  DEBUGASSERT(dmach != NULL && dmach->state == IMXRT_DMA_CONFIGURED);
  chan            = dmach->chan;
  dmainfo("dmach%u: %p callback: %p arg: %p\n", dmach, chan, callback, arg);

  /* Save the callback info.  This will be invoked whent the DMA commpletes */

  flags           = spin_lock_irqsave();
  dmach->callback = callback;
  dmach->arg      = arg;
  dmach->state    = IMXRT_DMA_ACTIVE;

  /* Enable channel ERROR interrupts */

  regval8         = EDMA_SEEI(chan);
  putreg8(regval8, IMXRT_EDMA_SEEI);

  /* Enable the DONE interrupt when the major iteration count completes. */

  regaddr         = IMXRT_EDMA_TCD_CSR(chan);
  modifyreg16(regaddr, 0, EDMA_TCD_CSR_INTMAJOR);

#if 0 /* Not yet controlled */
  /* Enable the DONE interrupt when the half the major iteration count completes. */

  modifyreg16(regaddr, 0, EDMA_TCD_CSR_INTHALF);
#endif

  /* Enable the DMA request for this channel */

  regval8         = EDMA_SERQ(chan);
  putreg8(regval8, IMXRT_EDMA_SERQ_OFFSET);

  /* Request channel service via either:
   *   - Software: setting the TCDn_CSR[START]
   *   - Hardware: slave device asserting its eDMA peripheral request signal
   *
   * REVISIT: Which case do we need to do the software interrupt?
   */

  regaddr         = IMXRT_EDMA_TCD_CSR(chan);
  regval16        = getreg16(regaddr);
  regval16       |= EDMA_TCD_CSR_START;
  putreg16(regval16, regaddr);

  spin_unlock_irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Name: imxrt_dmastop
 *
 * Description:
 *   Cancel the DMA.  After imxrt_dmastop() is called, the DMA channel is
 *   reset and imxrt_dmarx/txsetup() must be called before imxrt_dmastart()
 *   can be called again
 *
 ****************************************************************************/

void imxrt_dmastop(DMACH_HANDLE handle)
{
  struct imxrt_dmach_s *dmach = (struct imxrt_dmach_s *)handle;
  irqstate_t flags;

  dmainfo("dmach: %p\n", dmach);
  DEBUGASSERT(dmach != NULL);

  flags = spin_lock_irqsave();
  imxrt_dmaterminate(dmach, -EINTR);
  spin_unlock_irqrestore(flags);
}

/****************************************************************************
 * Name: imxrt_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by imxrt_dmachannel()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void imxrt_dmasample(DMACH_HANDLE handle, struct imxrt_dmaregs_s *regs)
{
  struct imxrt_dmach_s *dmach = (struct imxrt_dmach_s *)handle;
  uintptr_t regaddr;
  unsigned int chan;
  irqstate_t flags;

  DEBUGASSERT(dmach != NULL && regs != NULL);
  chan           = dmach->chan;
  regs->chan     = chan;

  /* eDMA Global Registers */

  flags          = spin_lock_irqsave();

  regs->cr       = getreg32(IMXRT_EDMA_CR);   /* Control */
  regs->es       = getreg32(IMXRT_EDMA_ES);   /* Error Status */
  regs->erq      = getreg32(IMXRT_EDMA_ERQ);  /* Enable Request */
  regs->req      = getreg32(IMXRT_EDMA_INT);  /* Interrupt Request */
  regs->err      = getreg32(IMXRT_EDMA_ERR);  /* Error */
  regs->hrs      = getreg32(IMXRT_EDMA_HRS);  /* Hardware Request Status */
  regs->ears     = getreg32(IMXRT_EDMA_EARS); /* Enable Asynchronous Request in Stop */

  /* eDMA Channel registers */

  regaddr        = IMXRT_EDMA_DCHPRI(chan);
  regs->dchpri   = getreg8(regaddr);          /* Channel priority */

  /* eDMA TCD */

  base           = IMXRT_EDMA_TCD_BASE(chan);
  regs->saddr    = getreg32(base + IMXRT_EDMA_TCD_SADDR_OFFSET);
  regs->soff     = getreg16(base + IMXRT_EDMA_TCD_SOFF_OFFSET);
  regs->attr     = getreg16(base + IMXRT_EDMA_TCD_ATTR_OFFSET);
  regs->nbml     = getreg32(base + IMXRT_EDMA_TCD_NBYTES_ML_OFFSET);
  regs->slast    = getreg32(base + IMXRT_EDMA_TCD_SLAST_OFFSET);
  regs->daddr    = getreg32(base + IMXRT_EDMA_TCD_DADDR_OFFSET);
  regs->doff     = getreg16(base + IMXRT_EDMA_TCD_DOFF_OFFSET);
  regs->citer    = getreg16(base + IMXRT_EDMA_TCD_CITER_ELINK_OFFSET);
  regs->dlastsga = getreg32(base + IMXRT_EDMA_TCD_DLASTSGA_OFFSET);
  regs->csr      = getreg16(base + IMXRT_EDMA_TCD_CSR_OFFSET);
  regs->biter    = getreg16(base + IMXRT_EDMA_TCD_BITER_ELINK_OFFSET);

  /* DMAMUX registers */

  regaddr        = IMXRT_DMAMUX_CHCF(chan);
  regs->dmamux   = getreg32(regaddr);         /* Channel configuration */

  spin_unlock_irqrestore(flags);
}
#endif /* CONFIG_DEBUG_DMA */

/****************************************************************************
 * Name: imxrt_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by imxrt_dmachannel()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void imxrt_dmadump(const struct imxrt_dmaregs_s *regs, const char *msg)
{
  unsigned int chan;

  DEBUGASSERT(regs != NULL && msg != NULL);

  chan = regs->chan;
  DEBUGASSERT(chan < IMXRT_EDMA_NCHANNELS);

  dmainfo("%s\n", msg);
  dmainfo("  eDMA Global Registers:\n");
  dmainfo("          CR: %08x\n", regs->cr);
  dmainfo("          ES: %08x\n", regs->es);
  dmainfo("         ERQ: %08x\n", regs->erq);
  dmainfo("         INT: %08x\n", regs->req);
  dmainfo("         ERR: %08x\n", regs->err);
  dmainfo("        EARS: %08x\n", regs->hrs);

  /* eDMA Channel registers */

  dmainfo("  eDMA Channel %u Registers:\n", chan);
  dmainfo("    DCHPRI: %02x\n", regs->dchpri);

  /* eDMA TCD */

  dmainfo("  eDMA Channel %u TCD Registers:\n", chan);
  dmainfo("       SADDR: %08x\n", regs->saddr);
  dmainfo("        SOFF: %04x\n", regs->soff);
  dmainfo("        ATTR: %04x\n", regs->attr);
  dmainfo("        NBML: %05x\n", regs->nbml);
  dmainfo("       SLAST: %05x\n", regs->slast);
  dmainfo("       DADDR: %05x\n", regs->daddr);
  dmainfo("        DOFF: %04x\n", regs->doff);
  dmainfo("       CITER: %04x\n", regs->citer);
  dmainfo("    DLASTSGA: %08x\n", regs->dlastsga);
  dmainfo("         CSR: %04x\n", regs->csr);
  dmainfo("       BITER: %04x\n", regs->biter);

  /* DMAMUX registers */

  dmainfo("  DMAMUX Channel %u Registers:\n", chan);
  dmainfo("      DMAMUX: %08x\n", regs->dmamux);
}
#endif /* CONFIG_DEBUG_DMA */
#endif /* CONFIG_IMXRT_EDMA */
