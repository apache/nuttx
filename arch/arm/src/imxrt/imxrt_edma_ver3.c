/****************************************************************************
 * arch/arm/src/imxrt/imxrt_edma_ver3.c
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: 2019, 2021, 2023 Gregory Nutt.
 * SPDX-FileCopyrightText: 2022 NXP
 * SPDX-FileCopyrightText: 2016-2017 NXP
 * SPDX-FileCopyrightText: 2015, Freescale Semiconductor, Inc.
 * SPDX-FileContributor: Gregory Nutt <gnutt@nuttx.org>
 * SPDX-FileContributor: David Sidrane <david.sidrane@nscdg.com>
 * SPDX-FileContributor: Peter van der Perk <peter.vanderperk@nxp.com>
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
#include "imxrt_edma_ver3.h"
#include "imxrt_clockconfig.h"

#include "hardware/imxrt_ccm.h"
#include "hardware/rt118x/imxrt118x_edma.h"
#include "hardware/imxrt_dmamux.h"

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

#define EDMA_ALIGN        ARMV7M_DCACHE_LINESIZE
#define EDMA_ALIGN_MASK   (EDMA_ALIGN - 1)
#define EDMA_ALIGN_UP(n)  (((n) + EDMA_ALIGN_MASK) & ~EDMA_ALIGN_MASK)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* State of a DMA channel */

enum imxrt_dmastate_e
{
  IMXRT_DMA_IDLE = 0,             /* No DMA in progress */
  IMXRT_DMA_CONFIGURED,           /* DMA configured, but not yet started */
  IMXRT_DMA_ACTIVE                /* DMA has been started and is in progress */
};

/* This structure describes one DMA channel */

struct imxrt_dmach_s
{
  uintptr_t base;                 /* DMA engine base address */
  uint32_t flags;                 /* DMA channel flags */
  bool inuse;                     /* true: The DMA channel is in use */
  uint8_t dmamux;                 /* DMAMUX channel number */
  uint8_t chan;                   /* DMA channel number (either eDMA3 or eDMA4) */
  uint8_t state;                  /* Channel state.  See enum imxrt_dmastate_e */
  edma_callback_t callback;       /* Callback invoked when the DMA completes */
  void *arg;                      /* Argument passed to callback function */
#if CONFIG_IMXRT_EDMA_NTCD > 0
  /* That TCD list is linked through the DLAST SGA field.  The first transfer
   * to be performed is at the head of the list.  Subsequent TCDs are added
   * at the tail of the list.
   */

  struct imxrt_edmatcd_s *head;   /* First TCD in the list */
  struct imxrt_edmatcd_s *tail;   /* Last TCD in the list */
#endif
};

/* This structure describes the state of the eDMA controller */

struct imxrt_edma_s
{
  /* These mutex protect the DMA channel and descriptor tables */

  mutex_t chlock;                 /* Protects channel table */
#if CONFIG_IMXRT_EDMA_NTCD > 0
  sem_t dsem;                     /* Supports wait for free descriptors */
#endif

  /* This array describes each DMA channel */

  struct imxrt_dmach_s dmach[IMXRT_EDMA_NCHANNELS];
  spinlock_t lock;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The state of the eDMA */

static struct imxrt_edma_s g_edma =
{
  .chlock = NXMUTEX_INITIALIZER,
#if CONFIG_IMXRT_EDMA_NTCD > 0
  .dsem = SEM_INITIALIZER(CONFIG_IMXRT_EDMA_NTCD),
#endif
  .lock = SP_UNLOCKED
};

#if CONFIG_IMXRT_EDMA_NTCD > 0
/* This is a singly-linked list of free TCDs */

static sq_queue_t g_tcd_free;

/* This is a pool of pre-allocated TCDs */

static struct imxrt_edmatcd_s g_tcd_pool[CONFIG_IMXRT_EDMA_NTCD]
              aligned_data(EDMA_ALIGN);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_tcd_alloc
 *
 * Description:
 *   Allocate an in-memory, TCD
 *
 ****************************************************************************/

#if CONFIG_IMXRT_EDMA_NTCD > 0
static struct imxrt_edmatcd_s *imxrt_tcd_alloc(void)
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

  nxsem_wait_uninterruptible(&g_edma.dsem);

  /* Now there should be a TCD in the free list reserved just for us */

  flags = spin_lock_irqsave(&g_edma.lock);
  tcd = (struct imxrt_edmatcd_s *)sq_remfirst(&g_tcd_free);
  DEBUGASSERT(tcd != NULL);

  spin_unlock_irqrestore(&g_edma.lock, flags);
  return tcd;
}
#endif

/****************************************************************************
 * Name: imxrt_tcd_free
 *
 * Description:
 *   Free an in-memory, TCD
 *
 ****************************************************************************/

#if CONFIG_IMXRT_EDMA_NTCD > 0
static void imxrt_tcd_free_nolock(struct imxrt_edmatcd_s *tcd)
{
  /* Add the the TCD to the end of the free list and post the 'dsem',
   * possibly waking up another thread that might be waiting for
   * a TCD.
   */

  sq_addlast((sq_entry_t *)tcd, &g_tcd_free);
  nxsem_post(&g_edma.dsem);
}

static void imxrt_tcd_free(struct imxrt_edmatcd_s *tcd)
{
  irqstate_t flags;

  /* Add the the TCD to the end of the free list and post the 'dsem',
   * possibly waking up another thread that might be waiting for
   * a TCD.
   */

  flags = spin_lock_irqsave(&g_edma.lock);
  sched_lock();
  imxrt_tcd_free_nolock(tcd);
  spin_unlock_irqrestore(&g_edma.lock, flags);
  sched_unlock();
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
      tcd = (sq_entry_t *)&g_tcd_pool[i];
      sq_addlast(tcd, &g_tcd_free);
    }
}
#endif

/****************************************************************************
 * Name: imxrt_tcd_chanlink
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

#ifdef CONFIG_IMXRT_EDMA_ELINK
static inline void imxrt_tcd_chanlink(uint8_t flags,
                                     struct imxrt_dmach_s *linkch,
                                     struct imxrt_edmatcd_s *tcd)
{
  uint16_t regval16;

  flags &= EDMA_CONFIG_LINKTYPE_MASK;

  if (linkch == NULL || flags == EDMA_CONFIG_LINKTYPE_LINKNONE)
    {
      /* No link or no link channel provided */

      /* Disable minor links */

      /* Disable major link */

      tcd->csr   &= ~EDMA_TCD_CSR_MAJORELINK;
    }
  else if (flags == EDMA_CONFIG_LINKTYPE_MINORLINK) /* Minor link config */
    {
      /* Enable minor link */

      tcd->citer |= EDMA_TCD_CITER_ELINK;
      tcd->biter |= EDMA_TCD_BITER_ELINK;

      /* Set linked channel */

      regval16    = tcd->citer;
      regval16   &= ~EDMA_TCD_CITER_LINKCH_MASK;
      regval16   |= EDMA_TCD_CITER_LINKCH(linkch->chan);
      tcd->citer  = regval16;

      regval16    = tcd->biter;
      regval16   &= ~EDMA_TCD_BITER_LINKCH_MASK;
      regval16   |= EDMA_TCD_BITER_LINKCH(linkch->chan);
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
 * Name: imxrt_tcd_configure
 *
 * Description:
 *  Configure all TCD registers to the specified values.  'tcd' is an
 *  'overlay' that may refer either to either the TCD register set or to an
 *  in-memory TCD structure.
 *
 ****************************************************************************/

static inline void imxrt_tcd_configure(struct imxrt_edmatcd_s *tcd,
                            const struct imxrt_edma_xfrconfig_s *config)
{
  tcd->saddr    = config->saddr;
  tcd->soff     = config->soff;
  tcd->attr     = EDMA_TCD_ATTR_SSIZE(config->ssize) |  /* Transfer Attributes */
                  EDMA_TCD_ATTR_DSIZE(config->dsize);
#ifdef CONFIG_IMXRT_EDMA_MOD
  tcd->attr     |= EDMA_TCD_ATTR_SMOD(config->smod) |  /* Transfer Attributes */
                   EDMA_TCD_ATTR_DMOD(config->dmod);
#endif
  tcd->nbytes   = config->nbytes;
  tcd->slast    = config->flags & EDMA_CONFIG_LOOPSRC ?
                                  -(config->iter * config->nbytes) : 0;

  tcd->daddr    = config->daddr;
  tcd->doff     = config->doff;
  tcd->citer    = config->iter & EDMA_TCD_CITER_MASK;
  tcd->biter    = config->iter & EDMA_TCD_BITER_MASK;
  tcd->csr      = config->flags & EDMA_CONFIG_LOOP_MASK ?
                                  0 : EDMA_TCD_CSR_DREQ;
  tcd->csr     |= config->flags & EDMA_CONFIG_INTHALF ?
                                  EDMA_TCD_CSR_INTHALF : 0;
  tcd->dlastsga = config->flags & EDMA_CONFIG_LOOPDEST ?
                                  -(config->iter * config->nbytes) : 0;

  /* And special case flags */

#ifdef CONFIG_IMXRT_EDMA_ELINK
  /* Configure major/minor link mapping */

  imxrt_tcd_chanlink(config->flags, (struct imxrt_dmach_s *)config->linkch,
                    tcd);
#endif
}

/****************************************************************************
 * Name: imxrt_tcd_instantiate
 *
 * Description:
 *   Copy an in-memory TCD into eDMA channel TCD registers
 *
 ****************************************************************************/

#if CONFIG_IMXRT_EDMA_NTCD > 0
static void imxrt_tcd_instantiate(struct imxrt_dmach_s *dmach,
                                  const struct imxrt_edmatcd_s *tcd)
{
  uintptr_t base = IMXRT_EDMA_TCD(dmach->base, dmach->chan);

  /* Push tcd into hardware TCD register */

  /* Clear DONE bit first, otherwise ESG cannot be set */

  putreg16(0,             base + IMXRT_EDMA_TCD_CSR_OFFSET);

  putreg32(tcd->saddr,    base + IMXRT_EDMA_TCD_SADDR_OFFSET);
  putreg16(tcd->soff,     base + IMXRT_EDMA_TCD_SOFF_OFFSET);
  putreg16(tcd->attr,     base + IMXRT_EDMA_TCD_ATTR_OFFSET);
  putreg32(tcd->nbytes,   base + IMXRT_EDMA_TCD_NBYTES_OFFSET);
  putreg32(tcd->slast,    base + IMXRT_EDMA_TCD_SLAST_SDA_OFFSET);
  putreg32(tcd->daddr,    base + IMXRT_EDMA_TCD_DADDR_OFFSET);
  putreg16(tcd->doff,     base + IMXRT_EDMA_TCD_DOFF_OFFSET);
  putreg16(tcd->citer,    base + IMXRT_EDMA_TCD_CITER_OFFSET);
  putreg32(tcd->dlastsga, base + IMXRT_EDMA_TCD_DLAST_SGA_OFFSET);

  putreg16(tcd->csr,      base + IMXRT_EDMA_TCD_CSR_OFFSET);

  putreg16(tcd->biter,    base + IMXRT_EDMA_TCD_BITER_OFFSET);
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
  uintptr_t base = IMXRT_EDMA_TCD(dmach->base, dmach->chan);
#if CONFIG_IMXRT_EDMA_NTCD > 0
  struct imxrt_edmatcd_s *tcd;
  struct imxrt_edmatcd_s *next;
#endif
  edma_callback_t callback;
  void *arg;

  irqstate_t flags;

  flags = spin_lock_irqsave(&g_edma.lock);
  sched_lock();

  /* Disable channel IRQ requests */

  putreg32(EDMA_CH_INT, base + IMXRT_EDMA_CH_INT_OFFSET);

  /* Clear CSR to disable channel. Because if the given channel started,
   * transfer CSR will be not zero. Because if it is the last transfer, DREQ
   * will be set.  If not, ESG will be set.
   */

  putreg32(0, base + IMXRT_EDMA_CH_CSR_OFFSET);

  putreg16(0, base + IMXRT_EDMA_TCD_CSR_OFFSET);

  /* Cancel next TCD transfer. */

  putreg32(0, base + IMXRT_EDMA_TCD_DLAST_SGA_OFFSET);

#if CONFIG_IMXRT_EDMA_NTCD > 0
  /* Return all allocated TCDs to the free list */

  for (tcd = dmach->head; tcd != NULL; tcd = next)
    {
      /* If channel looped to itself we are done
       * if not continue to free tcds in chain
       */

       next = dmach->flags & EDMA_CONFIG_LOOPDEST ?
              NULL : (struct imxrt_edmatcd_s *)((uintptr_t)tcd->dlastsga);

       imxrt_tcd_free_nolock(tcd);
    }

  dmach->head = NULL;
  dmach->tail = NULL;
#endif

  /* Perform the DMA complete callback */

  callback = dmach->callback;
  arg      = dmach->arg;

  dmach->callback = NULL;
  dmach->arg      = NULL;
  dmach->state    = IMXRT_DMA_IDLE;

  spin_unlock_irqrestore(&g_edma.lock, flags);
  sched_unlock();

  if (callback)
    {
      callback((DMACH_HANDLE)dmach, arg, true, result);
    }
}

/****************************************************************************
 * Name: imxrt_edma_intstatus
 *
 * Description:
 *   DMA interrupt status per eDMA engine and channel.
 *
 ****************************************************************************/

static inline uint32_t imxrt_edma_intstatus(uintptr_t base, uint8_t chan)
{
  /* The status register varies depending on eDMA instance and channel */

#ifdef IMXRT_DMA3_BASE
  /* eDMA3 uses the normal INT register */

  if (base == IMXRT_DMA3_BASE)
    {
      return getreg32(IMXRT_EDMA_INT);
    }
#endif

#ifdef IMXRT_DMA4_BASE
  /* eDMA4 has two INT status registers, holding 32 statuses each */

  if (chan > 31)
    {
      return getreg32(IMXRT_EDMA_INT_HIGH);
    }

  return getreg32(IMXRT_EDMA_INT_LOW);
#endif
}

/****************************************************************************
 * Name: imxrt_edma_isr
 *
 * Description:
 *   DMA interrupt service routine. The vector handler calls this with the
 *   appropriate parameters.
 *
 ****************************************************************************/

static int imxrt_edma_isr(int irq, void *context, void *arg)
{
  struct imxrt_dmach_s *dmach;
  uintptr_t base;
  uint32_t  regval32;
  uint32_t  errval32;
  uint8_t   chan;
  int       result;

  /* 'arg' should the DMA channel instance. */

  dmach = (struct imxrt_dmach_s *)arg;
  DEBUGASSERT(dmach != NULL);

  chan  = dmach->chan;
  base  = IMXRT_EDMA_TCD(dmach->base, dmach->chan);

  /* Get the eDMA Error Status register value. */

  errval32  = getreg32(base + IMXRT_EDMA_CH_ES_OFFSET);

  if (errval32 & EDMA_CH_ES_ERR)
    {
      DEBUGASSERT(dmach->state == IMXRT_DMA_ACTIVE);

      /* Clear the error */

      putreg32(EDMA_CH_ES_ERR, base + IMXRT_EDMA_CH_ES_OFFSET);

      /* Clear the pending eDMA channel interrupt */

      putreg32(EDMA_CH_INT, base + IMXRT_EDMA_CH_INT_OFFSET);

      imxrt_dmaterminate(dmach, -EIO);
      return OK;
    }

  /* Check for an eDMA pending interrupt on this channel */

  regval32 = imxrt_edma_intstatus(dmach->base, dmach->chan);
  if ((regval32 & EDMA_INT(chan & 31)) != 0)
    {
      /* An interrupt is pending.
       * This should only happen if the channel is active.
       */

      DEBUGASSERT(dmach->state == IMXRT_DMA_ACTIVE);

      /* Clear the pending eDMA channel interrupt */

      putreg32(EDMA_CH_INT, base + IMXRT_EDMA_CH_INT_OFFSET);

      /* Get the eDMA TCD Control and Status register value. */

      regval32 = getreg32(base + IMXRT_EDMA_CH_CSR_OFFSET);

      /* Check if transfer has finished. */

      if ((regval32 & EDMA_CH_CSR_DONE) != 0)
        {
          /* Clear the pending DONE interrupt status. */

          regval32 |= EDMA_CH_CSR_DONE;
          putreg32(regval32, base + IMXRT_EDMA_CH_CSR_OFFSET);
          result = OK;
        }
      else
        {
          /* Perform the half or end-of-major-cycle DMA callback */

          if (dmach->callback != NULL)
            {
              dmach->callback((DMACH_HANDLE)dmach, dmach->arg, false, OK);
            }

          return OK;
        }

      /* Terminate the transfer when it is done. */

      if ((dmach->flags & EDMA_CONFIG_LOOP_MASK) == 0)
        {
          imxrt_dmaterminate(dmach, result);
        }
      else if (dmach->callback != NULL)
        {
          dmach->callback((DMACH_HANDLE)dmach, dmach->arg, true, result);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: imxrt_edma_interrupt
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

static int imxrt_edma_interrupt(int irq, void *context, void *arg)
{
  struct imxrt_dmach_s *dmach = (struct imxrt_dmach_s *)arg;

#ifdef IMXRT_DMA3_BASE
  if ((irq >= IMXRT_IRQ_DMA3_CH0) && (irq <= IMXRT_IRQ_DMA3_CH31))
    {
      /* eDMA3 interrupt has a single source */

      imxrt_edma_isr(irq, context, dmach);
    }
#endif

#ifdef IMXRT_DMA4_BASE
  if ((irq >= IMXRT_IRQ_DMA4_CH0_CH1_CH32_CH33) &&
      (irq <= IMXRT_IRQ_DMA4_CH30_CH31_CH62_CH63))
    {
      /* eDMA4 interrupt has four sources on iMXRT8x */

      imxrt_edma_isr(irq, context, dmach);
      imxrt_edma_isr(irq, context, dmach + 1);
      imxrt_edma_isr(irq, context, dmach + 32);
      imxrt_edma_isr(irq, context, dmach + 33);
    }
#endif

  return OK;
}

/****************************************************************************
 * Name: imxrt_edma_configure
 *
 * Description:
 *   Configure eDMA instance.
 *
 ****************************************************************************/

static void imxrt_edma_configure(uintptr_t base)
{
  uint32_t regval;

  /* Configure the eDMA controllers */

  regval = getreg32(IMXRT_EDMA_CSR(base));
  regval &= ~(EDMA_CSR_EDBG | EDMA_CSR_ERCA | EDMA_CSR_HAE | EDMA_CSR_GCLC |
              EDMA_CSR_GMRC);

#ifdef CONFIG_IMXRT_EDMA_EDBG
  regval |= EDMA_CSR_EDBG;   /* Enable Debug */
#endif
#ifdef CONFIG_IMXRT_EDMA_ERCA
  regval |= EDMA_CSR_ERCA;   /* Enable Round Robin Channel Arbitration */
#endif
#ifdef CONFIG_IMXRT_EDMA_ERGA
  regval |= EDMA_CSR_ERGA;   /* Enable Round Robin Group Arbitration */
#endif
#ifdef CONFIG_IMXRT_EDMA_HOE
  regval |= EDMA_CSR_HAE;    /* Halt On Error */
#endif
#ifdef CONFIG_IMXRT_EDMA_CLM
  regval |= EDMA_CSR_GCLC;   /* Continuous Link Mode */
#endif

  putreg32(regval, IMXRT_EDMA_CSR(base));
}

/****************************************************************************
 * Name: imxrt_find_free_ch
 *
 * Description:
 *   Configure eDMA instance.
 *
 ****************************************************************************/

static struct imxrt_dmach_s * imxrt_find_free_ch(uint16_t dmamux)
{
  struct imxrt_dmach_s *candidate;
  uintptr_t base;
  unsigned int chndx;
  unsigned int offset;
  unsigned int max;

  /* eDMA base for MUX */

  base = imxrt_dmamux_get_dmabase(dmamux);

  /* On RT118x both eDMA3 and eDMA4 have per-channel CH_MUX registers so
   * any free channel of the correct engine can be used.
   */

  offset = imxrt_edma_choffset(base);
  max    = imxrt_edma_chmax(base);

  for (chndx = offset; chndx < max; chndx++)
    {
      candidate = &g_edma.dmach[chndx];
      if (!candidate->inuse)
        {
          return candidate;
        }
    }

  return NULL;
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
  struct imxrt_dmach_s *dmach;
  uintptr_t base;
  int chan;
  int i;

  dmainfo("Initialize eDMA\n");

  /* Configure the instances. */

  dmach = &g_edma.dmach[0];

#ifdef IMXRT_DMA3_BASE
  /* Configure eDMA3's LPCG29 (m33_clk_root). That root is already configured
   * by the M33 boot ROM so we only need to enable the LPCG.
   */

  imxrt_ccm_gate_on(CCM_LPCG_EDMA3, true);

  imxrt_edma_configure(IMXRT_DMA3_BASE);

  /* Initialize the channel */

  for (i = 0; i < DMA3_CHANNEL_COUNT; i++, dmach++)
    {
      dmach->base = IMXRT_DMA3_BASE;
      dmach->chan = i;

      irq_attach(IMXRT_IRQ_DMA3_CH0 + i, imxrt_edma_interrupt, dmach);
    }
#endif

#ifdef IMXRT_DMA4_BASE
  /* Configure eDMA4's clock root and enable its LPCG.
   *
   * wakeup_axi_clk_root = SYS_PLL3_OUT (480 MHz) / 2 = 240 MHz.
   */

  imxrt_ccm_configure_root_clock(CCM_CR_WAKEUP_AXI, SYS_PLL3_OUT, 2);

  /* Enable peripheral clock (eDMA4: wakeup_axi_clk_root, LPCG30) */

  imxrt_ccm_gate_on(CCM_LPCG_EDMA4, true);

  imxrt_edma_configure(IMXRT_DMA4_BASE);

  /* Initialize the channel */

  for (i = 0; i < DMA4_CHANNEL_COUNT; i++, dmach++)
    {
      dmach->base = IMXRT_DMA4_BASE;
      dmach->chan = i;

      /* Each eDMA4 interrupt is shared by four channels: n, n+1, n+32 and
       * n+33.  So IRQ (DMA4_CH0_CH1_CH32_CH33 + k) serves the channels
       * 2k, 2k+1, 2k+32 and 2k+33, and the 64 channels are covered by the
       * 32 lowest channel handles only.  Attach every second channel of
       * the lower half and pass that channel's handle as the argument.
       */

      if (i < (DMA4_CHANNEL_COUNT / 2) && (i & 0x01) == 0)
        {
          irq_attach(IMXRT_IRQ_DMA4_CH0_CH1_CH32_CH33 + (i >> 1),
                     imxrt_edma_interrupt, dmach);
        }
    }
#endif

#if CONFIG_IMXRT_EDMA_NTCD > 0
  /* Initialize the list of free TCDs from the pool of pre-allocated TCDs. */

  imxrt_tcd_initialize();
#endif

  /* Disable all DMA channel interrupts at the eDMA controller */

  for (i = 0; i < IMXRT_EDMA_NCHANNELS; i++)
    {
      /* DMA engine base and TCD channel */

      base = g_edma.dmach[i].base;
      chan = g_edma.dmach[i].chan;

      /* Disable all DMA channels and DMA channel interrupts */

      putreg32(0, IMXRT_EDMA_TCD(base, chan) + IMXRT_EDMA_CH_CSR_OFFSET);

      /* Clear interrupt if any */

      putreg32(1, IMXRT_EDMA_TCD(base, chan) + IMXRT_EDMA_CH_INT_OFFSET);

      /* Set all TCD CSR, biter and citer entries to 0 so that
       * will be 0 when DONE is not set so that imxrt_dmach_getcount
       * reports 0.
       */

      putreg16(0, IMXRT_EDMA_TCD(base, chan) + IMXRT_EDMA_TCD_CSR_OFFSET);
      putreg16(0, IMXRT_EDMA_TCD(base, chan) + IMXRT_EDMA_TCD_CITER_OFFSET);
      putreg16(0, IMXRT_EDMA_TCD(base, chan) + IMXRT_EDMA_TCD_BITER_OFFSET);
    }

#ifdef IMXRT_DMA3_BASE
  /* Clear all pending DMA channel interrupts */

  putreg32(0xffffffff, IMXRT_EDMA_INT);

  /* Enable the channel interrupts at the NVIC (still disabled at the eDMA
   * controller).
   */

  for (i = 0; i < DMA3_IRQ_COUNT; i++)
    {
      up_enable_irq(IMXRT_IRQ_DMA3_CH0 + i);
    }
#endif

#ifdef IMXRT_DMA4_BASE
  /* Clear all pending DMA channel interrupts */

  putreg32(0xffffffff, IMXRT_EDMA_INT_LOW);
  putreg32(0xffffffff, IMXRT_EDMA_INT_HIGH);

  /* Enable the channel interrupts at the NVIC (still disabled at the eDMA
   * controller).
   */

  for (i = 0; i < DMA4_IRQ_COUNT; i++)
    {
      up_enable_irq(IMXRT_IRQ_DMA4_CH0_CH1_CH32_CH33 + i);
    }
#endif
}

/****************************************************************************
 * Name: imxrt_dmach_alloc
 *
 *   Allocate a DMA channel.  This function sets aside a DMA channel,
 *   initializes the DMAMUX for the channel, then gives the caller exclusive
 *   access to the DMA channel.
 *
 * Input Parameters:
 *
 *   dmamux - DMAMUX configuration see DMAMUX channel configuration register
 *            bit-field definitions in hardware/imxrt_dmamux.h.
 *            Settings include:
 *
 *            DMAMUX_CHCFG_SOURCE     Chip-specific DMA source (required)
 *            DMAMUX_CHCFG_TRIG       DMA Channel Trigger Enable (optional)
 *            DMAMUX_CHCFG_ENBL       DMA Mux Channel Enable (required)
 *
 *            A value of zero will disable the DMAMUX channel.
 *   dchpri - DCHPRI channel priority configuration.  See DCHPRI channel
 *            configuration register bit-field definitions in
 *            hardware/imxrt_edma.h.  Meaningful settings include:
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

DMACH_HANDLE imxrt_dmach_alloc(uint16_t dmamux, uint8_t dchpri)
{
  struct imxrt_dmach_s *dmach;
  uintptr_t base;
  uint32_t regval;
  int ret;

  /* Search for an available DMA channel */

  dmach = NULL;
  ret = nxmutex_lock(&g_edma.chlock);
  if (ret < 0)
    {
      return NULL;
    }

  /* Find channel for this DMA MUX */

  dmach = imxrt_find_free_ch(dmamux);
  if (dmach)
    {
      dmach->inuse  = true;
      dmach->state  = IMXRT_DMA_IDLE;
      dmach->dmamux = dmamux & EDMA_MUX_MASK;

      /* TCD register base */

      base = IMXRT_EDMA_TCD(dmach->base, dmach->chan);

      /* Clear any pending interrupts on the channel */

      putreg32(0, base + IMXRT_EDMA_CH_CSR_OFFSET);

      /* Make sure that the channel is disabled. */

      putreg32(EDMA_CH_INT, base + IMXRT_EDMA_CH_INT_OFFSET);

      regval = getreg32(base + IMXRT_EDMA_CH_SBR_OFFSET);
      regval |= EDMA_CH_SBR_EMI | EDMA_CH_SBR_SEC | EDMA_CH_SBR_PAL;
      putreg32(regval, base + IMXRT_EDMA_CH_SBR_OFFSET);

      /* Set the DMAMUX source */

      if (imxrt_edma_tcdhasmux(dmach->base))
        {
          /* Set reset value first to CH MUX */

          putreg8(0, base + IMXRT_EDMA_CH_MUX_OFFSET);
          dmainfo("CH%d: MUX:%u->%p\n", dmach->chan, dmach->dmamux,
                  (void *)(base + IMXRT_EDMA_CH_MUX_OFFSET));
          putreg8(dmach->dmamux, base + IMXRT_EDMA_CH_MUX_OFFSET);
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
 * Name: imxrt_dmach_free
 *
 * Description:
 *   Release a DMA channel.  NOTE:  The 'handle' used in this argument must
 *   NEVER be used again until imxrt_dmach_alloc() is called again to
 *   re-gain a valid handle.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void imxrt_dmach_free(DMACH_HANDLE handle)
{
  struct imxrt_dmach_s *dmach = (struct imxrt_dmach_s *)handle;
  uintptr_t base = IMXRT_EDMA_TCD(dmach->base, dmach->chan);

  dmainfo("dmach: %p\n", dmach);
  DEBUGASSERT(dmach != NULL && dmach->inuse &&
              dmach->state != IMXRT_DMA_ACTIVE);

  /* Mark the channel no longer in use.  Clearing the inuse flag is an atomic
   * operation and so should be safe.
   */

  dmach->flags = 0;
  dmach->inuse = false;                /* No longer in use */
  dmach->state = IMXRT_DMA_IDLE;       /* Better not be active! */

  /* Make sure that the channel is disabled. */

  putreg32(EDMA_CH_INT, base + IMXRT_EDMA_CH_INT_OFFSET);

  /* Disable the associated DMAMUX */

  if (imxrt_edma_tcdhasmux(dmach->base))
    {
      putreg8(0, base + IMXRT_EDMA_CH_MUX_OFFSET);
    }
}

/****************************************************************************
 * Name: imxrt_dmach_xfrsetup
 *
 * Description:
 *   This function adds the eDMA transfer to the DMA sequence.  The request
 *   is setup according to the content of the transfer configuration
 *   structure. For "normal" DMA, imxrt_dmach_xfrsetup is called only once.
 *   Scatter/gather DMA is accomplished by calling this function repeatedly,
 *   once for each transfer in the sequence.  Scatter/gather DMA processing
 *   is enabled automatically when the second transfer configuration is
 *   received.
 *
 *   This function may be called multiple times to handle multiple,
 *   discontinuous transfers (scatter-gather)
 *
 * Input Parameters:
 *   handle - DMA channel handle created by imxrt_dmach_alloc()
 *   config - A DMA transfer configuration instance, populated by the
 *            The content of 'config' describes the transfer
 *
 * Returned Value
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int imxrt_dmach_xfrsetup(DMACH_HANDLE handle,
                        const struct imxrt_edma_xfrconfig_s *config)
{
  struct imxrt_dmach_s *dmach = (struct imxrt_dmach_s *)handle;
  uintptr_t base = IMXRT_EDMA_TCD(dmach->base, dmach->chan);
#if CONFIG_IMXRT_EDMA_NTCD > 0
  struct imxrt_edmatcd_s *tcd;
  struct imxrt_edmatcd_s *prev;
  uint16_t mask = config->flags & EDMA_CONFIG_INTMAJOR ? 0 :
                                  EDMA_TCD_CSR_INTMAJOR;
  uint16_t regval16;
#else
  uint32_t regval32;
#endif

  DEBUGASSERT(dmach != NULL);
  dmainfo("dmach%u: %p config: %p\n", dmach->chan, dmach, config);

  dmach->flags  = config->flags;

#if CONFIG_IMXRT_EDMA_NTCD > 0
  /* Scatter/gather DMA is supported */

  /* Allocate a TCD, waiting if necessary */

  tcd = imxrt_tcd_alloc();

  /* Configure current TCD block transfer. */

  imxrt_tcd_configure(tcd, config);

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

      imxrt_tcd_instantiate(dmach, tcd);
    }
  else
    {
      /* Cannot mix transfer types */

      if (dmach->flags & EDMA_CONFIG_LOOP_MASK)
        {
          imxrt_tcd_free(tcd);
          return -EINVAL;
        }

      /* Chain from previous descriptor in the list. */

      /* Enable scatter/gather feature in the previous TCD. */

      prev           = dmach->tail;
      regval16       = prev->csr;
      regval16      &= ~(EDMA_TCD_CSR_DREQ | mask);
      regval16      |= EDMA_TCD_CSR_ESG;
      prev->csr      = regval16;

      prev->dlastsga = (uint32_t)((uintptr_t)tcd);
      dmach->tail    = tcd;

      /* Clean cache associated with the previous TCD memory */

      up_clean_dcache((uintptr_t)prev,
                      (uintptr_t)prev + sizeof(struct imxrt_edmatcd_s));

      /* Check if the TCD block in the DMA channel registers is the same as
       * the previous previous TCD.  This can happen if the previous TCD was
       * the first TCD and has already be loaded into the TCD registers.
       */

      if (dmach->head == prev)
        {
          /* Enable scatter/gather also in the TCD registers. */

          regval16  = getreg16(base + IMXRT_EDMA_TCD_CSR_OFFSET);
          regval16 &= ~(EDMA_TCD_CSR_DREQ | mask);
          regval16 |= EDMA_TCD_CSR_ESG;
          putreg16(regval16, base + IMXRT_EDMA_TCD_CSR_OFFSET);

          putreg32((uint32_t)((uintptr_t)tcd),
                   base + IMXRT_EDMA_TCD_DLAST_SGA_OFFSET);
        }
    }

  /* Clean cache associated with the TCD memory */

  up_clean_dcache((uintptr_t)tcd,
                  (uintptr_t)tcd + sizeof(struct imxrt_edmatcd_s));
#else

  /* Scatter/gather DMA is NOT supported */

  /* Check if eDMA is busy: if the channel has started transfer, CSR will be
   * non-zero.
   */

  regval32  = getreg32(base + IMXRT_EDMA_CH_CSR_OFFSET);

  if (regval32 != 0 && (regval32 & EDMA_CH_CSR_DONE) == 0)
    {
      return -EBUSY;
    }

  /* Configure channel TCD registers to the values specified in config. */

  imxrt_tcd_configure((struct imxrt_edmatcd_s *)
                     (base + IMXRT_EDMA_TCD_SADDR_OFFSET), config);

  /* Enable the DONE interrupt when the major iteration count completes. */

  modifyreg16(base + IMXRT_EDMA_TCD_CSR_OFFSET, 0, EDMA_TCD_CSR_INTMAJOR);
#endif

  dmach->state = IMXRT_DMA_CONFIGURED;
  return OK;
}

/****************************************************************************
 * Name: imxrt_dmach_start
 *
 * Description:
 *   Start the DMA transfer.  This function should be called after the final
 *   call to imxrt_dmach_xfrsetup() in order to avoid race conditions.
 *
 *   At the conclusion of each major DMA loop, a callback to the user
 *   provided function is made:  |For "normal" DMAs, this will correspond to
 *   the DMA DONE interrupt; for scatter gather DMAs,
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
 *   handle   - DMA channel handle created by imxrt_dmach_alloc()
 *   callback - The callback to be invoked when the DMA is completes or is
 *              aborted.
 *   arg      - An argument that accompanies the callback
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int imxrt_dmach_start(DMACH_HANDLE handle, edma_callback_t callback,
                     void *arg)
{
  struct imxrt_dmach_s *dmach = (struct imxrt_dmach_s *)handle;
  uintptr_t base = IMXRT_EDMA_TCD(dmach->base, dmach->chan);
  irqstate_t flags;
  uint32_t regval;
  uint8_t chan;

  DEBUGASSERT(dmach != NULL && dmach->state == IMXRT_DMA_CONFIGURED);
  chan            = dmach->chan;
  dmainfo("dmach%u: %p callback: %p arg: %p\n", chan, dmach, callback, arg);

  /* Save the callback info.  This will be invoked when the DMA completes */

  flags           = spin_lock_irqsave(&g_edma.lock);
  dmach->callback = callback;
  dmach->arg      = arg;

#if CONFIG_IMXRT_EDMA_NTCD > 0
  /* Although it is not recommended, it might be possible to call this
   * function multiple times while adding TCDs on the fly.
   */

  if (dmach->state != IMXRT_DMA_ACTIVE)
#endif
    {
      dmach->state   = IMXRT_DMA_ACTIVE;

      regval         = getreg32(base + IMXRT_EDMA_CH_CSR_OFFSET);
      regval        |= EDMA_CH_CSR_ERQ | EDMA_CH_CSR_EEI;
      putreg32(regval, base + IMXRT_EDMA_CH_CSR_OFFSET);

      if ((dmach->flags & EDMA_CONFIG_SWSTART) != 0)
        {
          putreg16(getreg16(base + IMXRT_EDMA_TCD_CSR_OFFSET) |
                   EDMA_TCD_CSR_START,
                   base + IMXRT_EDMA_TCD_CSR_OFFSET);
        }
    }

  spin_unlock_irqrestore(&g_edma.lock, flags);
  return OK;
}

/****************************************************************************
 * Name: imxrt_dmach_stop
 *
 * Description:
 *   Cancel the DMA.  After imxrt_dmach_stop() is called, the DMA channel
 *   is reset, all TCDs are freed, and imxrt_dmarx/txsetup() must be called
 *   before imxrt_dmach_start() can be called again.
 *
 * Input Parameters:
 *   handle   - DMA channel handle created by imxrt_dmach_alloc()
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void imxrt_dmach_stop(DMACH_HANDLE handle)
{
  struct imxrt_dmach_s *dmach = (struct imxrt_dmach_s *)handle;

  dmainfo("dmach: %p\n", dmach);
  DEBUGASSERT(dmach != NULL);

  imxrt_dmaterminate(dmach, -EINTR);
}

/****************************************************************************
 * Name: imxrt_dmach_getcount
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
 *   handle  - DMA channel handle created by imxrt_dmach_alloc()
 *
 * Returned Value:
 *   Major loop count which has not been transferred yet for the current TCD.
 *
 ****************************************************************************/

unsigned int imxrt_dmach_getcount(DMACH_HANDLE handle)
{
  struct imxrt_dmach_s *dmach = (struct imxrt_dmach_s *)handle;
  uintptr_t base = IMXRT_EDMA_TCD(dmach->base, dmach->chan);
  unsigned int remaining = 0;
  uintptr_t regval32;
  uint16_t regval16;

  DEBUGASSERT(dmach != NULL);

  /* If the DMA is done, then the remaining count is zero */

  regval32  = getreg32(base + IMXRT_EDMA_CH_CSR_OFFSET);

  if ((regval32 & EDMA_CH_CSR_DONE) == 0)
    {
      /* Calculate the unfinished bytes */

      regval16 = getreg16(base + IMXRT_EDMA_TCD_CITER_OFFSET);

      if ((regval16 & EDMA_TCD_CITER_ELINK) != 0)
        {
          remaining = (regval16 & EDMA_TCD_CITER_MASK_ELINK) >>
                      EDMA_TCD_CITER_SHIFT;
        }
      else
        {
          remaining = (regval16 & EDMA_TCD_CITER_MASK) >>
                       EDMA_TCD_CITER_SHIFT;
        }
    }

  return remaining;
}

/****************************************************************************
 * Name: imxrt_dmach_idle
 *
 * Description:
 *   This function checks if the dma is idle
 *
 * Returned Value:
 *   0  - if idle
 *   !0 - not
 *
 ****************************************************************************/

unsigned int imxrt_dmach_idle(DMACH_HANDLE handle)
{
  struct imxrt_dmach_s *dmach = (struct imxrt_dmach_s *)handle;
  return dmach->state == IMXRT_DMA_IDLE ? 0 : -1;
}

/****************************************************************************
 * Name: imxrt_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by imxrt_dmach_alloc()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void imxrt_dmasample(DMACH_HANDLE handle, struct imxrt_dmaregs_s *regs)
{
  struct imxrt_dmach_s *dmach = (struct imxrt_dmach_s *)handle;
  unsigned int chan;
  irqstate_t flags;
  uintptr_t base = IMXRT_EDMA_TCD(dmach->base, dmach->chan);

  DEBUGASSERT(dmach != NULL && regs != NULL);
  chan           = dmach->chan;
  regs->chan     = chan;

  /* eDMA Global Registers */

  flags          = spin_lock_irqsave(&g_edma.lock);

  /* REVISIT: eDMA4 does not show INT_HIGH / HRS_HIGH values correctly */

  regs->cr       = getreg32(IMXRT_EDMA_CSR(base)); /* Control */
  regs->es       = getreg32(IMXRT_EDMA_ES(base));  /* Error Status */
  regs->req      = getreg32(IMXRT_EDMA_INT);       /* Interrupt Request */
  regs->hrs      = getreg32(IMXRT_EDMA_HRS);       /* Hardware Request Status */

  /* eDMA TCD */

  regs->saddr    = getreg32(base + IMXRT_EDMA_TCD_SADDR_OFFSET);
  regs->soff     = getreg16(base + IMXRT_EDMA_TCD_SOFF_OFFSET);
  regs->attr     = getreg16(base + IMXRT_EDMA_TCD_ATTR_OFFSET);
  regs->nbml     = getreg32(base + IMXRT_EDMA_TCD_NBYTES_OFFSET);
  regs->slast    = getreg32(base + IMXRT_EDMA_TCD_SLAST_SDA_OFFSET);
  regs->daddr    = getreg32(base + IMXRT_EDMA_TCD_DADDR_OFFSET);
  regs->doff     = getreg16(base + IMXRT_EDMA_TCD_DOFF_OFFSET);
  regs->citer    = getreg16(base + IMXRT_EDMA_TCD_CITER_OFFSET);
  regs->dlastsga = getreg32(base + IMXRT_EDMA_TCD_DLAST_SGA_OFFSET);
  regs->csr      = getreg16(base + IMXRT_EDMA_TCD_CSR_OFFSET);
  regs->biter    = getreg16(base + IMXRT_EDMA_TCD_BITER_OFFSET);

  /* DMAMUX registers */

  if (imxrt_edma_tcdhasmux(dmach->base))
    {
      regs->dmamux = getreg32(base + IMXRT_EDMA_CH_MUX_OFFSET);
    }
  else
    {
      regs->dmamux = 0;
    }

  spin_unlock_irqrestore(&g_edma.lock, flags);
}
#endif /* CONFIG_DEBUG_DMA */

/****************************************************************************
 * Name: imxrt_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by imxrt_dmach_alloc()
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
  dmainfo("          CR: %08x\n", (unsigned int)regs->cr);
  dmainfo("          ES: %08x\n", (unsigned int)regs->es);
  dmainfo("         INT: %08x\n", (unsigned int)regs->req);
  dmainfo("        EARS: %08x\n", (unsigned int)regs->hrs);

  /* eDMA Channel registers */

  dmainfo("  eDMA Channel %u Registers:\n", chan);
  dmainfo("    DCHPRI: %02x\n", regs->dchpri);

  /* eDMA TCD */

  dmainfo("  eDMA Channel %u TCD Registers:\n", chan);
  dmainfo("       SADDR: %08x\n", (unsigned int)regs->saddr);
  dmainfo("        SOFF: %04x\n", (unsigned int)regs->soff);
  dmainfo("        ATTR: %04x\n", (unsigned int)regs->attr);
  dmainfo("        NBML: %05x\n", (unsigned int)regs->nbml);
  dmainfo("       SLAST: %05x\n", (unsigned int)regs->slast);
  dmainfo("       DADDR: %05x\n", (unsigned int)regs->daddr);
  dmainfo("        DOFF: %04x\n", (unsigned int)regs->doff);
  dmainfo("       CITER: %04x\n", (unsigned int)regs->citer);
  dmainfo("    DLASTSGA: %08x\n", (unsigned int)regs->dlastsga);
  dmainfo("         CSR: %04x\n", (unsigned int)regs->csr);
  dmainfo("       BITER: %04x\n", (unsigned int)regs->biter);

  /* DMAMUX registers */

  dmainfo("  DMAMUX Channel %u Registers:\n", chan);
  dmainfo("      DMAMUX: %08x\n", (unsigned int)regs->dmamux);
}
#endif /* CONFIG_DEBUG_DMA */
#endif /* CONFIG_IMXRT_EDMA */
