/****************************************************************************
 * arch/arm/src/s32k1xx/s32k1xx_edma.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * This file was leveraged from the NuttX i.MXRT port.  Portions of that eDMA
 * logic derived from NXP sample code which has a compatible BSD 3-clause
 * license:
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
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/queue.h>
#include <nuttx/spinlock.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>

#include "arm_internal.h"
#include "sched/sched.h"

#include "chip.h"
#include "hardware/s32k1xx_edma.h"
#include "hardware/s32k1xx_dmamux.h"
#include "s32k1xx_edma.h"

#ifdef CONFIG_S32K1XX_EDMA

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
#  define EDMA_ALIGN  ARMV7M_DCACHE_LINESIZE
#else
/* 32 byte alignment for TCDs is required for scatter gather */

#define EDMA_ALIGN        32
#endif

#define EDMA_ALIGN_MASK   (EDMA_ALIGN - 1)
#define EDMA_ALIGN_UP(n)  (((n) + EDMA_ALIGN_MASK) & ~EDMA_ALIGN_MASK)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* State of a DMA channel */

enum s32k1xx_dmastate_e
{
  S32K1XX_DMA_IDLE = 0,             /* No DMA in progress */
  S32K1XX_DMA_CONFIGURED,           /* DMA configured, but not yet started */
  S32K1XX_DMA_ACTIVE                /* DMA has been started and is in progress */
};

/* This structure describes one DMA channel */

struct s32k1xx_dmach_s
{
  uint8_t  chan;                  /* DMA channel number (0-S32K1XX_EDMA_NCHANNELS) */
  bool     inuse;                 /* true: The DMA channel is in use */
  uint8_t  state;                 /* Channel state.  See enum s32k1xx_dmastate_e */
  uint8_t  dmamux;                /* The DMAMUX channel selection */
  uint32_t flags;                 /* DMA channel flags */
  edma_callback_t callback;       /* Callback invoked when the DMA completes */
  void *arg;                      /* Argument passed to callback function */

#if CONFIG_S32K1XX_EDMA_NTCD > 0
  /* That TCD list is linked through the DLAST SGA field.  The first transfer
   * to be performed is at the head of the list.  Subsequent TCDs are added
   * at the tail of the list.
   */

  struct s32k1xx_edmatcd_s *head;   /* First TCD in the list */
  struct s32k1xx_edmatcd_s *tail;   /* Last TCD in the list */
#endif
};

/* This structure describes the state of the eDMA controller */

struct s32k1xx_edma_s
{
  /* These mutex protect the DMA channel and descriptor tables */

  mutex_t chlock;                 /* Protects channel table */
#if CONFIG_S32K1XX_EDMA_NTCD > 0
  sem_t dsem;                     /* Supports wait for free descriptors */
#endif

  /* This array describes each DMA channel */

  struct s32k1xx_dmach_s dmach[S32K1XX_EDMA_NCHANNELS];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The state of the eDMA */

static struct s32k1xx_edma_s g_edma =
{
  .chlock = NXMUTEX_INITIALIZER,
#if CONFIG_S32K1XX_EDMA_NTCD > 0
  .dsem = SEM_INITIALIZER(CONFIG_S32K1XX_EDMA_NTCD),
#endif
};

#if CONFIG_S32K1XX_EDMA_NTCD > 0
/* This is a singly-linked list of free TCDs */

static sq_queue_t g_tcd_free;

/* This is a pool of pre-allocated TCDs */

static struct s32k1xx_edmatcd_s g_tcd_pool[CONFIG_S32K1XX_EDMA_NTCD]
              aligned_data(EDMA_ALIGN);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s32k1xx_takedsem() and s32k1xx_givedsem()
 *
 * Description:
 *   Used to wait for availability of descriptors in the descriptor table.
 *
 ****************************************************************************/

#if CONFIG_S32K1XX_EDMA_NTCD > 0
static void s32k1xx_takedsem(void)
{
  nxsem_wait_uninterruptible(&g_edma.dsem);
}

static inline void s32k1xx_givedsem(void)
{
  nxsem_post(&g_edma.dsem);
}
#endif

/****************************************************************************
 * Name: s32k1xx_tcd_alloc
 *
 * Description:
 *   Allocate an in-memory, TCD
 *
 ****************************************************************************/

#if CONFIG_S32K1XX_EDMA_NTCD > 0
static struct s32k1xx_edmatcd_s *s32k1xx_tcd_alloc(void)
{
  struct s32k1xx_edmatcd_s *tcd;
  irqstate_t flags;

  /* Take the 'dsem'.  When we hold the the 'dsem', then we know that one
   * TCD is reserved for us in the free list.
   *
   * NOTE: We use a critical section here because we may block waiting for
   * the 'dsem'.  The critical section will be suspended while we are
   * waiting.
   */

  flags = enter_critical_section();
  s32k1xx_takedsem();

  /* Now there should be a TCD in the free list reserved just for us */

  tcd = (struct s32k1xx_edmatcd_s *)sq_remfirst(&g_tcd_free);
  DEBUGASSERT(tcd != NULL);

  leave_critical_section(flags);
  return tcd;
}
#endif

/****************************************************************************
 * Name: s32k1xx_tcd_free
 *
 * Description:
 *   Free an in-memory, TCD
 *
 ****************************************************************************/

#if CONFIG_S32K1XX_EDMA_NTCD > 0
static void s32k1xx_tcd_free(struct s32k1xx_edmatcd_s *tcd)
{
  irqstate_t flags;

  /* Add the the TCD to the end of the free list and post the 'dsem',
   * possibly waking up another thread that might be waiting for
   * a TCD.
   */

  flags = spin_lock_irqsave(NULL);
  sq_addlast((sq_entry_t *)tcd, &g_tcd_free);
  s32k1xx_givedsem();
  spin_unlock_irqrestore(NULL, flags);
}
#endif

/****************************************************************************
 * Name: s32k1xx_tcd_initialize()
 *
 * Description:
 *   Initialize the TCD free list from the pool of pre-allocated TCDs.
 *
 * Assumptions:
 *   Called early in the initialization sequence so no special protection is
 *   necessary.
 *
 ****************************************************************************/

#if CONFIG_S32K1XX_EDMA_NTCD > 0
static inline void s32k1xx_tcd_initialize(void)
{
  sq_entry_t *tcd;
  int i;

  /* Add each pre-allocated TCD to the tail of the TCD free list */

  sq_init(&g_tcd_free);
  for (i = 0; i < CONFIG_S32K1XX_EDMA_NTCD; i++)
    {
      tcd = (sq_entry_t *)&g_tcd_pool[i];
      sq_addlast(tcd, &g_tcd_free);
    }
}
#endif

/****************************************************************************
 * Name: s32k1xx_tcd_chanlink
 *
 * Description:
 *   This function configures either a minor link or a major link. The minor
 *   link means the channel link is triggered every time CITER decreases by 1
 *   The major link means that the channel link  is triggered when the CITER
 *   is exhausted.
 *
 *   NOTE: Users should ensure that DONE flag is cleared before calling this
 *   interface, or the configuration is invalid.
 *
 * Input Parameters:
 *   tcd  - Point to the TCD structure.
 *   type - Channel link type.
 *   chan - The linked channel number.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_S32K1XX_EDMA_ELINK
static inline void s32k1xx_tcd_chanlink(uint8_t flags,
                                        struct s32k1xx_dmach_s *linkch,
                                        struct s32k1xx_edmatcd_s *tcd)
{
  uint16_t regval16;

  flags &= EDMA_CONFIG_LINKTYPE_MASK;

  if (linkch == NULL || flags == EDMA_CONFIG_LINKTYPE_LINKNONE)
    {
      /* No link or no link channel provided */

      /* Disable minor links is done in s32k1xx_tcd_configure */

      /* Disable major link */

      tcd->csr   &= ~EDMA_TCD_CSR_MAJORELINK;
    }
  else if (flags == EDMA_CONFIG_LINKTYPE_MINORLINK) /* Minor link config */
    {
      /* Enable minor link */

      tcd->citer |= EDMA_TCD_CITER_ELINK_ELINK;
      tcd->biter |= EDMA_TCD_BITER_ELINK_ELINK;

      /* Set linked channel */

      regval16    = tcd->citer;
      regval16   &= ~EDMA_TCD_CITER_ELINK_LINKCH_MASK;
      regval16   |= EDMA_TCD_CITER_ELINK_LINKCH(linkch->chan);
      tcd->citer  = regval16;

      regval16    = tcd->biter;
      regval16   &= ~EDMA_TCD_BITER_ELINK_LINKCH_MASK;
      regval16   |= EDMA_TCD_BITER_ELINK_LINKCH(linkch->chan);
      tcd->biter  = regval16;
    }
  else /* if (flags == EDMA_CONFIG_LINKTYPE_MAJORLINK)  Major link config */
    {
      /* Enable major link */

      regval16    = tcd->csr;
      regval16   |= EDMA_TCD_CSR_MAJORELINK;
      tcd->csr    = regval16;

      /* Set major linked channel */

      regval16   &= ~EDMA_TCD_CSR_MAJORLINKCH_MASK;
      regval16   |=  EDMA_TCD_CSR_MAJORLINKCH(linkch->chan);
      tcd->csr    = regval16;
    }
}
#endif

/****************************************************************************
 * Name: s32k1xx_tcd_configure
 *
 * Description:
 *  Configure all TCD registers to the specified values.  'tcd' is an
 *  'overlay' that may refer either to either the TCD register set or to an
 *  in-memory TCD structure.
 *
 ****************************************************************************/

static inline void s32k1xx_tcd_configure(struct s32k1xx_edmatcd_s *tcd,
                            const struct s32k1xx_edma_xfrconfig_s *config)
{
  tcd->saddr    = config->saddr;
  tcd->soff     = config->soff;
  tcd->attr     = EDMA_TCD_ATTR_SSIZE(config->ssize) |  /* Transfer Attributes */
                  EDMA_TCD_ATTR_DSIZE(config->dsize);
  tcd->nbytes   = config->nbytes;
  tcd->slast    = config->flags & EDMA_CONFIG_LOOPSRC ?  -config->iter : 0;
  tcd->daddr    = config->daddr;
  tcd->doff     = config->doff;
  tcd->citer    = config->iter & EDMA_TCD_CITER_CITER_MASK;
  tcd->biter    = config->iter & EDMA_TCD_BITER_BITER_MASK;
  tcd->csr      = config->flags & EDMA_CONFIG_LOOP_MASK ?
                                  0 : EDMA_TCD_CSR_DREQ;
  tcd->csr      |= config->flags & EDMA_CONFIG_INTHALF ?
                                  EDMA_TCD_CSR_INTHALF : 0;
  tcd->dlastsga = config->flags & EDMA_CONFIG_LOOPDEST ?  -config->iter : 0;

  /* And special case flags */

#ifdef CONFIG_S32K1XX_EDMA_ELINK
  /* Configure major/minor link mapping */

  s32k1xx_tcd_chanlink(config->flags,
                       (struct s32k1xx_dmach_s *)config->linkch,
                       tcd);
#endif
}

/****************************************************************************
 * Name: s32k1xx_tcd_instantiate
 *
 * Description:
 *   Copy an in-memory TCD into eDMA channel TCD registers
 *
 ****************************************************************************/

#if CONFIG_S32K1XX_EDMA_NTCD > 0
static void s32k1xx_tcd_instantiate(struct s32k1xx_dmach_s *dmach,
                                    const struct s32k1xx_edmatcd_s *tcd)
{
  uintptr_t base = S32K1XX_EDMA_TCD_BASE(dmach->chan);

  /* Push tcd into hardware TCD register */

  /* Clear DONE bit first, otherwise ESG cannot be set */

  putreg16(0,             base + S32K1XX_EDMA_TCD_CSR_OFFSET);

  putreg32(tcd->saddr,    base + S32K1XX_EDMA_TCD_SADDR_OFFSET);
  putreg16(tcd->soff,     base + S32K1XX_EDMA_TCD_SOFF_OFFSET);
  putreg16(tcd->attr,     base + S32K1XX_EDMA_TCD_ATTR_OFFSET);
  putreg32(tcd->nbytes,   base + S32K1XX_EDMA_TCD_NBYTES_ML_OFFSET);
  putreg32(tcd->slast,    base + S32K1XX_EDMA_TCD_SLAST_OFFSET);
  putreg32(tcd->daddr,    base + S32K1XX_EDMA_TCD_DADDR_OFFSET);
  putreg16(tcd->doff,     base + S32K1XX_EDMA_TCD_DOFF_OFFSET);
  putreg16(tcd->citer,    base + S32K1XX_EDMA_TCD_CITER_ELINK_OFFSET);
  putreg32(tcd->dlastsga, base + S32K1XX_EDMA_TCD_DLASTSGA_OFFSET);

  putreg16(tcd->csr,      base + S32K1XX_EDMA_TCD_CSR_OFFSET);

  putreg16(tcd->biter,    base + S32K1XX_EDMA_TCD_BITER_ELINK_OFFSET);
}
#endif

/****************************************************************************
 * Name: s32k1xx_dmaterminate
 *
 * Description:
 *   Terminate the DMA transfer and disable the DMA channel
 *
 ****************************************************************************/

static void s32k1xx_dmaterminate(struct s32k1xx_dmach_s *dmach, int result)
{
#if CONFIG_S32K1XX_EDMA_NTCD > 0
  struct s32k1xx_edmatcd_s *tcd;
  struct s32k1xx_edmatcd_s *next;
#endif
  uintptr_t regaddr;
  uint8_t regval8;
  uint8_t chan;

  /* Disable channel ERROR interrupts */

  chan            = dmach->chan;
  regval8         = EDMA_CEEI(chan);
  putreg8(regval8, S32K1XX_EDMA_CEEI);

  /* Disable channel IRQ requests */

  regval8         = EDMA_CERQ(chan);
  putreg8(regval8, S32K1XX_EDMA_CERQ);

  regaddr         = S32K1XX_EDMA_TCD_CSR(chan);
  putreg16(0, regaddr);

  /* Cancel next TCD transfer. */

  regaddr         = S32K1XX_EDMA_TCD_DLASTSGA(chan);
  putreg32(0, regaddr);

#if CONFIG_S32K1XX_EDMA_NTCD > 0
  /* Return all allocated TCDs to the free list */

  for (tcd = dmach->head; tcd != NULL; tcd = next)
    {
      /* If channel looped to itself we are done
       * if not continue to free tcds in chain
       */

       next = dmach->flags & EDMA_CONFIG_LOOPDEST ?
              NULL : (struct s32k1xx_edmatcd_s *)tcd->dlastsga;
      s32k1xx_tcd_free(tcd);
    }

  dmach->head = NULL;
  dmach->tail = NULL;
#endif

  /* Perform the DMA complete callback */

  if (dmach->callback)
    {
      dmach->callback((DMACH_HANDLE)dmach, dmach->arg, true, result);
    }

  dmach->callback = NULL;
  dmach->arg      = NULL;
  dmach->state    = S32K1XX_DMA_IDLE;
}

/****************************************************************************
 * Name: s32k1xx_edma_interrupt
 *
 * Description:
 *   DMA interrupt handler.  This function clears the channel major
 *   interrupt flag and calls the callback function if it is not NULL.
 *
 *   NOTE:  For the case using TCD queue, when the major iteration count is
 *   exhausted, additional operations are performed.  These include the
 *   final address adjustments and reloading of the BITER field into the
 *   CITER.  Assertion of an optional interrupt request also occurs at this
 *   time, as does a possible fetch of a new TCD from memory using the
 *   scatter/gather address pointer included in the descriptor (if scatter/
 *   gather is enabled).
 *
 ****************************************************************************/

static int s32k1xx_edma_interrupt(int irq, void *context, void *arg)
{
  struct s32k1xx_dmach_s *dmach;
  uintptr_t regaddr;
  uint32_t  regval32;
  uint16_t  regval16;
  uint8_t   regval8;
  uint8_t   chan;
  int       result;

  /* 'arg' should be the DMA channel instance. */

  dmach = (struct s32k1xx_dmach_s *)arg;
  DEBUGASSERT(dmach != NULL);

  chan  = dmach->chan;
  DEBUGASSERT(chan < S32K1XX_EDMA_NCHANNELS && dmach == &g_edma.dmach[chan]);

  /* Check for an eDMA pending interrupt on this channel */

  regval32 = getreg32(S32K1XX_EDMA_INT);
  if ((regval32 & EDMA_INT(chan)) != 0)
    {
      /* An interrupt is pending.  This should only happen if the channel is
       * active.
       */

      DEBUGASSERT(dmach->state == S32K1XX_DMA_ACTIVE);

      /* Clear the pending eDMA channel interrupt */

      regval8 = EDMA_CINT(chan);
      putreg8(regval8, S32K1XX_EDMA_CINT);

      /* Get the eDMA TCD Control and Status register value. */

      regaddr  = S32K1XX_EDMA_TCD_CSR(chan);
      regval16 = getreg16(regaddr);

      /* Check if transfer has finished. */

      if ((regval16 & EDMA_TCD_CSR_DONE) != 0)
        {
          /* Clear the pending DONE interrupt status. */

          regval8 = EDMA_CDNE(chan);
          putreg8(regval8, S32K1XX_EDMA_CDNE);
          result = OK;
        }
      else
        {
#if CONFIG_S32K1XX_EDMA_NTCD > 0
          /* Perform the half or end-of-major-cycle DMA callback */

          if (dmach->callback != NULL)
            {
              dmach->callback((DMACH_HANDLE)dmach, dmach->arg,
                              false, OK);
            }

          return OK;
#else
          /* Otherwise the interrupt was not expected! */

          DEBUGPANIC();
          result = -EPIPE;
#endif
        }

      /* Terminate the transfer when it is done. */

      if ((dmach->flags & EDMA_CONFIG_LOOP_MASK) == 0)
        {
          s32k1xx_dmaterminate(dmach, result);
        }
      else if (dmach->callback != NULL)
        {
          dmach->callback((DMACH_HANDLE)dmach, dmach->arg,
                          true, result);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: s32k1xx_error_interrupt
 *
 * Description:
 *  DMA error interrupt handler
 *
 ****************************************************************************/

static int s32k1xx_error_interrupt(int irq, void *context, void *arg)
{
  uint32_t errstatus;
  uint32_t errmask;
  uint8_t regval8;
  unsigned int chan;

  /* Get the set of pending error interrupts */

  errstatus = getreg32(S32K1XX_EDMA_ERR);

  /* Check for an error on each channel */

  for (chan = 0; chan < S32K1XX_EDMA_NCHANNELS && errstatus != 0; chan++)
    {
      /* Check for a pending error interrupt on each channel */

      errmask = EDMA_ERR(chan);
      if ((errstatus & errmask) != 0)
        {
          dmaerr("ERROR: DMACH%u ES=%08lx\n",
                 chan, (unsigned long)getreg32(S32K1XX_EDMA_ES));

          /* Clear the pending error interrupt status. */

          regval8 = EDMA_CERR(chan);
          putreg8(regval8, S32K1XX_EDMA_CERR);

          /* Remove the bit from the sample ERR register so that perhaps we
           * can exit this loop early.
           */

          errstatus &= ~errmask;

          /* Terminate the transfer on any error */

          s32k1xx_dmaterminate(&g_edma.dmach[chan], -EIO);
        }
    }

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
 *   NOTE:  This assumes that board-specific logic has enabled clocking to
 *   the DMA module as part of the clock initialization logic.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void weak_function arm_dma_initialize(void)
{
  uintptr_t regaddr;
  uint32_t regval;
  int i;

  dmainfo("Initialize eDMA\n");

  /* Configure the eDMA controller */

  regval = getreg32(S32K1XX_EDMA_CR);
  regval &= ~(EDMA_CR_EDBG | EDMA_CR_ERCA | EDMA_CR_HOE | EDMA_CR_CLM |
              EDMA_CR_EMLM);

#ifdef CONFIG_S32K1XX_EDMA_EDBG
  regval |= EDMA_CR_EDBG;   /* Enable Debug */
#endif
#ifdef CONFIG_S32K1XX_EDMA_ERCA
  regval |= EDMA_CR_ERCA;   /* Enable Round Robin Channel Arbitration */
#endif
#ifdef CONFIG_S32K1XX_EDMA_HOE
  regval |= EDMA_CR_HOE;    /* Halt On Error */
#endif
#ifdef CONFIG_S32K1XX_EDMA_CLM
  regval |= EDMA_CR_CLM;    /* Continuous Link Mode */
#endif
#ifdef CONFIG_S32K1XX_EDMA_EMLIM
  regval |= EDMA_CR_EMLM;   /* Enable Minor Loop Mapping */
#endif

  putreg32(regval, S32K1XX_EDMA_CR);

  /* Initialize data structures */

  for (i = 0; i < S32K1XX_EDMA_NCHANNELS; i++)
    {
      g_edma.dmach[i].chan = i;
    }

#if CONFIG_S32K1XX_EDMA_NTCD > 0
  /* Initialize the list of free TCDs from the pool of pre-allocated TCDs. */

  s32k1xx_tcd_initialize();
#endif

  /* Attach DMA interrupt vectors. */

  for (i = 0; i < S32K1XX_EDMA_NCHANNELS; i++)
    {
      irq_attach(S32K1XX_IRQ_DMACH0 + i,
                 s32k1xx_edma_interrupt, &g_edma.dmach[i]);
    }

  /* Attach the DMA error interrupt vector */

  irq_attach(S32K1XX_IRQ_DMACH_ERR, s32k1xx_error_interrupt, NULL);

  /* Disable and clear all error interrupts */

  putreg32(0, S32K1XX_EDMA_EEI);
  putreg32(0xffffffff, S32K1XX_EDMA_ERR);

  /* Disable all DMA channel interrupts at the eDMA controller */

  for (i = 0; i < S32K1XX_EDMA_NCHANNELS; i++)
    {
      /* Disable all DMA channels and DMA channel interrupts */

      regaddr = S32K1XX_EDMA_TCD_CSR(i);
      putreg16(0, regaddr);

      /* Set all TCD entries to 0 so that biter and citer
       * will be 0 when DONE is not set so that s32k1xx_dmach_getcount
       * reports 0.
       */

      memset((void *)S32K1XX_EDMA_TCD_BASE(i), 0,
             sizeof(struct s32k1xx_edmatcd_s));
    }

  /* Clear all pending DMA channel interrupts */

  putreg32(0xffffffff, S32K1XX_EDMA_INT);

  /* Enable the channel interrupts at the NVIC (still disabled at the eDMA
   * controller).
   */

  for (i = 0; i < S32K1XX_EDMA_NCHANNELS; i++)
    {
      up_enable_irq(S32K1XX_IRQ_DMACH0 + i);
    }

  /* Enable the DMA error interrupt */

  up_enable_irq(S32K1XX_IRQ_DMACH_ERR);
}

/****************************************************************************
 * Name: s32k1xx_dmach_alloc
 *
 *   Allocate a DMA channel.  This function sets aside a DMA channel,
 *   initializes the DMAMUX for the channel, then gives the caller exclusive
 *   access to the DMA channel.
 *
 * Input Parameters:
 *   dmamux - DMAMUX configuration see DMAMUX channel configuration register
 *            bit-field definitions in hardware/s32k1xx_dmamux.h.
 *            Settings include:
 *
 *            DMAMUX_CHCFG_SOURCE     Chip-specific DMA source (required)
 *            DMAMUX_CHCFG_TRIG       DMA Channel Trigger Enable (optional)
 *            DMAMUX_CHCFG_ENBL       DMA Mux Channel Enable (required)
 *
 *            A value of zero will disable the DMAMUX channel.
 *   dchpri - DCHPRI channel priority configuration.  See DCHPRI channel
 *            configuration register bit-field definitions in
 *            hardware/s32k1xx_edma.h.  Meaningful settings include:
 *
 *            EDMA_DCHPRI_CHPRI       Channel Arbitration Priority
 *            DCHPRI_DPA              Disable Preempt Ability
 *            DCHPRI_ECP              Enable Channel Preemption
 *
 *            The power-on default, 0x05, is a reasonable choice.
 *
 * Returned Value:
 *   If a DMA channel is available, this function returns a non-NULL, void*
 *   DMA channel handle.  NULL is returned on any failure.
 *
 ****************************************************************************/

DMACH_HANDLE s32k1xx_dmach_alloc(uint8_t dmamux, uint8_t dchpri)
{
  struct s32k1xx_dmach_s *dmach;
  unsigned int chndx;
  int ret;

  /* Search for an available DMA channel */

  dmach = NULL;
  ret = nxmutex_lock(&g_edma.chlock);
  if (ret < 0)
    {
      return NULL;
    }

  for (chndx = 0; chndx < S32K1XX_EDMA_NCHANNELS; chndx++)
    {
      struct s32k1xx_dmach_s *candidate = &g_edma.dmach[chndx];
      uintptr_t regaddr;
      uint8_t regval8;

      if (!candidate->inuse)
        {
          dmach        = candidate;
          dmach->inuse = true;
          dmach->state = S32K1XX_DMA_IDLE;
          dmach->dmamux = dmamux;

          /* Clear any pending interrupts on the channel */

          DEBUGASSERT(chndx == dmach->chan);
          regaddr = S32K1XX_EDMA_TCD_CSR(chndx);
          putreg16(0, regaddr);

          /* Make sure that the channel is disabled. */

          regval8 = EDMA_CERQ(chndx);
          putreg8(regval8, S32K1XX_EDMA_CERQ);

          /* Disable the associated DMAMUX for now */

          putreg8(0, S32K1XX_DMAMUX_CHCFG(chndx));
          break;
        }
    }

  nxmutex_unlock(&g_edma.chlock);

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
 * Name: s32k1xx_dmach_free
 *
 * Description:
 *   Release a DMA channel.  NOTE:  The 'handle' used in this argument must
 *   NEVER be used again until s32k1xx_dmach_alloc() is called again to
 *   re-gain a valid handle.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void s32k1xx_dmach_free(DMACH_HANDLE handle)
{
  struct s32k1xx_dmach_s *dmach = (struct s32k1xx_dmach_s *)handle;
  uint8_t regval8;

  dmainfo("dmach: %p\n", dmach);
  DEBUGASSERT(dmach != NULL && dmach->inuse &&
              dmach->state != S32K1XX_DMA_ACTIVE);

  /* Mark the channel no longer in use.  Clearing the inuse flag is an atomic
   * operation and so should be safe.
   */

  dmach->flags = 0;
  dmach->inuse = false;                   /* No longer in use */
  dmach->state = S32K1XX_DMA_IDLE;        /* Better not be active! */

  /* Make sure that the channel is disabled. */

  regval8 = EDMA_CERQ(dmach->chan);
  putreg8(regval8, S32K1XX_EDMA_CERQ);

  /* Disable the associated DMAMUX */

  putreg8(0, S32K1XX_DMAMUX_CHCFG(dmach->chan));
}

/****************************************************************************
 * Name: s32k1xx_dmach_xfrsetup
 *
 * Description:
 *   This function adds the eDMA transfer to the DMA sequence.  The request
 *   is setup according to the content of the transfer configuration
 *   structure. For "normal" DMA, s32k1xx_dmach_xfrsetup is called only once.
 *   Scatter/gather DMA is accomplished by calling this function repeatedly,
 *   once for each transfer in the sequence.  Scatter/gather DMA processing
 *   is enabled automatically when the second transfer configuration is
 *   received.
 *
 *   This function may be called multiple times to handle multiple,
 *   discontinuous transfers (scatter-gather)
 *
 * Input Parameters:
 *   handle - DMA channel handle created by s32k1xx_dmach_alloc()
 *   config - A DMA transfer configuration instance, populated by the
 *            The content of 'config' describes the transfer
 *
 * Returned Value
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int s32k1xx_dmach_xfrsetup(DMACH_HANDLE *handle,
                           const struct s32k1xx_edma_xfrconfig_s *config)
{
  struct s32k1xx_dmach_s *dmach = (struct s32k1xx_dmach_s *)handle;
#if CONFIG_S32K1XX_EDMA_NTCD > 0
  struct s32k1xx_edmatcd_s *tcd;
  struct s32k1xx_edmatcd_s *prev;
  uint16_t mask = config->flags & EDMA_CONFIG_INTMAJOR ? 0 :
                                  EDMA_TCD_CSR_INTMAJOR;
#endif
  uintptr_t regaddr;
  uint16_t regval16;

  DEBUGASSERT(dmach != NULL);
  dmainfo("dmach%u: %p config: %p\n", dmach->chan, dmach, config);

  dmach->flags  = config->flags;

#if CONFIG_S32K1XX_EDMA_NTCD > 0
  /* Scatter/gather DMA is supported */

  /* Allocate a TCD, waiting if necessary */

  tcd = s32k1xx_tcd_alloc();

  /* Configure current TCD block transfer. */

  s32k1xx_tcd_configure(tcd, config);

  /* Enable the interrupt when the major iteration count completes for this
   * TCD.  For "normal" DMAs, this will correspond to the DMA DONE
   * interrupt; for scatter gather DMAs, multiple interrupts will be
   * generated with the final being the DONE interrupt.
   */

  tcd->csr |= EDMA_TCD_CSR_INTMAJOR;

  /* Is this the first descriptor in the list? */

  if (dmach->head == NULL)
    {
      /* Yes.. add it to the list */

      dmach->head  = tcd;
      dmach->tail  = tcd;

      /* And instantiate the first TCD in the DMA channel TCD registers. */

      s32k1xx_tcd_instantiate(dmach, tcd);
    }
  else
    {
      /* Cannot mix transfer types */

      if (dmach->flags & EDMA_CONFIG_LOOP_MASK)
        {
          s32k1xx_tcd_free(tcd);
          return -EINVAL;
        }

      /* Chain from previous descriptor in the list. */

      /* Enable scatter/gather feature in the previous TCD. */

      prev           = dmach->tail;
      regval16       = prev->csr;
      regval16      &= ~(EDMA_TCD_CSR_DREQ | mask);
      regval16      |= EDMA_TCD_CSR_ESG;

      prev->csr      = regval16;

      prev->dlastsga = (uint32_t)tcd;
      dmach->tail    = tcd;

      /* Clean cache associated with the previous TCD memory */

      up_clean_dcache((uintptr_t)prev,
                      (uintptr_t)prev + sizeof(struct s32k1xx_edmatcd_s));

      /* Check if the TCD block in the DMA channel registers is the same as
       * the previous previous TCD.  This can happen if the previous TCD was
       * the first TCD and has already be loaded into the TCD registers.
       */

      if (dmach->head == prev)
        {
          /* Enable scatter/gather also in the TCD registers. */

          regaddr   = S32K1XX_EDMA_TCD_CSR(dmach->chan);
          regval16  = getreg16(regaddr);
          regval16 &= ~(EDMA_TCD_CSR_DREQ | mask);
          regval16 |= EDMA_TCD_CSR_ESG;
          putreg16(regval16, regaddr);

          regaddr   = S32K1XX_EDMA_TCD_DLASTSGA(dmach->chan);
          putreg32((uint32_t)tcd, regaddr);
        }
    }

  /* Clean cache associated with the TCD memory */

  up_clean_dcache((uintptr_t)tcd,
                  (uintptr_t)tcd + sizeof(struct s32k1xx_edmatcd_s));
#else
  /* Scatter/gather DMA is NOT supported */

  /* Check if eDMA is busy: if the channel has started transfer, CSR will be
   * non-zero.
   */

  regaddr  = S32K1XX_EDMA_TCD_CSR(dmach->chan);
  regval16 = getreg16(regaddr);

  if (regval16 != 0 && (regval16 & EDMA_TCD_CSR_DONE) == 0)
    {
      return -EBUSY;
    }

  /* Configure channel TCD registers to the values specified in config. */

  s32k1xx_tcd_configure((struct s32k1xx_edmatcd_s *)
                        S32K1XX_EDMA_TCD_BASE(dmach->chan), config);

  /* Enable the DONE interrupt when the major iteration count completes. */

  regaddr         = S32K1XX_EDMA_TCD_CSR(dmach->chan);
  modifyreg16(regaddr, 0, EDMA_TCD_CSR_INTMAJOR);
#endif

  /* Set the DMAMUX source and enable and optional trigger */

  putreg8(dmach->dmamux, S32K1XX_DMAMUX_CHCFG(dmach->chan));

  dmach->state = S32K1XX_DMA_CONFIGURED;
  return OK;
}

/****************************************************************************
 * Name: s32k1xx_dmach_start
 *
 * Description:
 *   Start the DMA transfer.  This function should be called after the final
 *   call to s32k1xx_dmach_xfrsetup() in order to avoid race conditions.
 *
 *   At the conclusion of each major DMA loop, a callback to
 *   the user-provided function is made: For "normal" DMAs, this will
 *   correspond to the DMA DONE interrupt; for scatter gather DMAs,
 *   this will be generated with the final TCD.
 *
 *   At the conclusion of the DMA, the DMA channel is reset, all TCDs are
 *   freed, and the callback function is called with the the success/fail
 *   result of the DMA.
 *
 *   NOTE: On Rx DMAs (peripheral-to-memory or memory-to-memory), it is
 *   necessary to invalidate the destination memory.  That is not done
 *   automatically by the DMA module.  Invalidation of the destination memory
 *   regions is the responsibility of the caller.
 *
 * Input Parameters:
 *   handle   - DMA channel handle created by s32k1xx_dmach_alloc()
 *   callback - The callback to be invoked when the DMA is completes or is
 *              aborted.
 *   arg      - An argument that accompanies the callback
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int s32k1xx_dmach_start(DMACH_HANDLE handle, edma_callback_t callback,
                        void *arg)
{
  struct s32k1xx_dmach_s *dmach = (struct s32k1xx_dmach_s *)handle;
  irqstate_t flags;
  uint8_t regval8;
  uint8_t chan;

  DEBUGASSERT(dmach != NULL && dmach->state == S32K1XX_DMA_CONFIGURED);
  chan            = dmach->chan;
  dmainfo("dmach%u: %p callback: %p arg: %p\n", chan, dmach, callback, arg);

  /* Save the callback info.  This will be invoked when the DMA completes */

  flags           = spin_lock_irqsave(NULL);
  dmach->callback = callback;
  dmach->arg      = arg;

#if CONFIG_S32K1XX_EDMA_NTCD > 0
  /* Although it is not recommended, it might be possible to call this
   * function multiple times while adding TCDs on the fly.
   */

  if (dmach->state != S32K1XX_DMA_ACTIVE)
#endif
    {
      dmach->state    = S32K1XX_DMA_ACTIVE;

      /* Enable channel ERROR interrupts */

      regval8         = EDMA_SEEI(chan);
      putreg8(regval8, S32K1XX_EDMA_SEEI);

      /* Enable the DMA request for this channel */

      regval8         = EDMA_SERQ(chan);
      putreg8(regval8, S32K1XX_EDMA_SERQ);
    }

  spin_unlock_irqrestore(NULL, flags);
  return OK;
}

/****************************************************************************
 * Name: s32k1xx_dmach_stop
 *
 * Description:
 *   Cancel the DMA.  After s32k1xx_dmach_stop() is called, the DMA channel
 *   is reset, all TCDs are freed, and s32k1xx_dmarx/txsetup() must be called
 *   before s32k1xx_dmach_start() can be called again.
 *
 * Input Parameters:
 *   handle   - DMA channel handle created by s32k1xx_dmach_alloc()
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void s32k1xx_dmach_stop(DMACH_HANDLE handle)
{
  struct s32k1xx_dmach_s *dmach = (struct s32k1xx_dmach_s *)handle;
  irqstate_t flags;

  dmainfo("dmach: %p\n", dmach);
  DEBUGASSERT(dmach != NULL);

  flags = spin_lock_irqsave(NULL);
  s32k1xx_dmaterminate(dmach, -EINTR);
  spin_unlock_irqrestore(NULL, flags);
}

/****************************************************************************
 * Name: s32k1xx_dmach_getcount
 *
 * Description:
 *   This function checks the TCD (Task Control Descriptor) status for a
 *   specified eDMA channel and returns the the number of major loop counts
 *   that have not finished.
 *
 *   NOTES:
 *   1. This function can only be used to get unfinished major loop count of
 *      transfer without the next TCD, or it might be inaccuracy.
 *   2. The unfinished/remaining transfer bytes cannot be obtained directly
 *      from registers while the channel is running.
 *
 *   Because to calculate the remaining bytes, the initial NBYTES configured
 *   in DMA_TCDn_NBYTES_MLNO register is needed while the eDMA IP does not
 *   support getting it while a channel is active.  In another words, the
 *   NBYTES value reading is always the actual (decrementing) NBYTES value
 *   the dma_engine is working with while a channel is running.
 *   Consequently, to get the remaining transfer bytes, a software-saved
 *   initial value of NBYTES (for example copied before enabling the channel)
 *   is needed. The formula to calculate it is shown below:
 *
 *     RemainingBytes = RemainingMajorLoopCount *
 *                      NBYTES(initially configured)
 *
 * Input Parameters:
 *   handle  - DMA channel handle created by s32k1xx_dmach_alloc()
 *
 * Returned Value:
 *   Major loop count which has not been transferred yet for the current TCD.
 *
 ****************************************************************************/

unsigned int s32k1xx_dmach_getcount(DMACH_HANDLE *handle)
{
  struct s32k1xx_dmach_s *dmach = (struct s32k1xx_dmach_s *)handle;
  unsigned int remaining = 0;
  uintptr_t regaddr;
  uint16_t regval16;

  DEBUGASSERT(dmach != NULL);

  /* If the DMA is done, then the remaining count is zero */

  regaddr  = S32K1XX_EDMA_TCD_CSR(dmach->chan);
  regval16 = getreg16(regaddr);

  if ((regval16 & EDMA_TCD_CSR_DONE) == 0)
    {
      /* Calculate the unfinished bytes */

      regaddr  = S32K1XX_EDMA_TCD_CITER_ELINK(dmach->chan);
      regval16 = getreg16(regaddr);

      if ((regval16 & EDMA_TCD_CITER_ELINK) != 0)
        {
          remaining = (regval16 & EDMA_TCD_CITER_ELINK_CITER_MASK) >>
                      EDMA_TCD_CITER_ELINK_CITER_SHIFT;
        }
      else
        {
          remaining = (regval16 & EDMA_TCD_CITER_CITER_MASK) >>
                      EDMA_TCD_CITER_CITER_SHIFT;
        }
    }

  return remaining;
}

/****************************************************************************
 * Name: s32k1xx_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by s32k1xx_dmach_alloc()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void s32k1xx_dmasample(DMACH_HANDLE handle, struct s32k1xx_dmaregs_s *regs)
{
  struct s32k1xx_dmach_s *dmach = (struct s32k1xx_dmach_s *)handle;
  uintptr_t regaddr;
  unsigned int chan;
  irqstate_t flags;

  DEBUGASSERT(dmach != NULL && regs != NULL);
  chan           = dmach->chan;
  regs->chan     = chan;

  /* eDMA Global Registers */

  flags          = spin_lock_irqsave(NULL);

  regs->cr       = getreg32(S32K1XX_EDMA_CR);   /* Control */
  regs->es       = getreg32(S32K1XX_EDMA_ES);   /* Error Status */
  regs->erq      = getreg32(S32K1XX_EDMA_ERQ);  /* Enable Request */
  regs->req      = getreg32(S32K1XX_EDMA_INT);  /* Interrupt Request */
  regs->err      = getreg32(S32K1XX_EDMA_ERR);  /* Error */
  regs->hrs      = getreg32(S32K1XX_EDMA_HRS);  /* Hardware Request Status */
  regs->ears     = getreg32(S32K1XX_EDMA_EARS); /* Enable Asynchronous Request in Stop */

  /* eDMA Channel registers */

  regaddr        = S32K1XX_EDMA_DCHPRI(chan);
  regs->dchpri   = getreg8(regaddr);          /* Channel priority */

  /* eDMA TCD */

  base           = S32K1XX_EDMA_TCD_BASE(chan);
  regs->saddr    = getreg32(base + S32K1XX_EDMA_TCD_SADDR_OFFSET);
  regs->soff     = getreg16(base + S32K1XX_EDMA_TCD_SOFF_OFFSET);
  regs->attr     = getreg16(base + S32K1XX_EDMA_TCD_ATTR_OFFSET);
  regs->nbml     = getreg32(base + S32K1XX_EDMA_TCD_NBYTES_ML_OFFSET);
  regs->slast    = getreg32(base + S32K1XX_EDMA_TCD_SLAST_OFFSET);
  regs->daddr    = getreg32(base + S32K1XX_EDMA_TCD_DADDR_OFFSET);
  regs->doff     = getreg16(base + S32K1XX_EDMA_TCD_DOFF_OFFSET);
  regs->citer    = getreg16(base + S32K1XX_EDMA_TCD_CITER_ELINK_OFFSET);
  regs->dlastsga = getreg32(base + S32K1XX_EDMA_TCD_DLASTSGA_OFFSET);
  regs->csr      = getreg16(base + S32K1XX_EDMA_TCD_CSR_OFFSET);
  regs->biter    = getreg16(base + S32K1XX_EDMA_TCD_BITER_ELINK_OFFSET);

  /* DMAMUX registers */

  regaddr        = S32K1XX_DMAMUX_CHCFG(chan);
  regs->dmamux   = getreg32(regaddr);         /* Channel configuration */

  spin_unlock_irqrestore(NULL, flags);
}
#endif /* CONFIG_DEBUG_DMA */

/****************************************************************************
 * Name: s32k1xx_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by s32k1xx_dmach_alloc()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void s32k1xx_dmadump(const struct s32k1xx_dmaregs_s *regs, const char *msg)
{
  unsigned int chan;

  DEBUGASSERT(regs != NULL && msg != NULL);

  chan = regs->chan;
  DEBUGASSERT(chan < S32K1XX_EDMA_NCHANNELS);

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
#endif /* CONFIG_S32K1XX_EDMA */
