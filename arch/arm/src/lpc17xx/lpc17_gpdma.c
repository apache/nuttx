/****************************************************************************
 * arch/arm/src/lpc17xx/lpc17_gpdma.c
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"

#include "chip/lpc17_syscon.h"
#include "lpc17_gpdma.h"

#ifdef CONFIG_LPC17_GPDMA

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Enables debug output from this file (needs CONFIG_DEBUG too) */

#undef DMA_DEBUG     /* Define to enable debug */
#undef DMA_VERBOSE   /* Define to enable verbose debug */

#ifdef DMA_DEBUG
#  define dmadbg  lldbg
#  ifdef DMA_VERBOSE
#    define spivdbg lldbg
#  else
#    define spivdbg(x...)
#  endif
#else
#  undef DMA_VERBOSE
#  define dmadbg(x...)
#  define spivdbg(x...)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* This structure represents the state of one DMA channel */

struct lpc17_dmach_s
{
  bool inuse;              /* True: The channel is in use */
  dma_callback_t callback; /* DMA completion callback function */
  void *arg;               /* Argument to pass to the callback function */
};

/* This structure represents the state of the LPC17 DMA block */

struct lpc17_gpdma_s
{
  sem_t exclsem;           /* For exclusive access to the DMA channel list */

  /* This is the state of each DMA channel */

  struct lpc17_dmach_s dmach[LPC17_NDMACH];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* The state of the LPC17 DMA block */

static struct lpc17_gpdma_s g_gpdma;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc17_dmainitialize
 *
 * Description:
 *   Initialize the GPDMA subsystem.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void lpc17_dmainitilaize(void)
{
  uint32_t regval;
  int i;

  /* Enable clocking to the GPDMA block */

  regval  = getreg32(LPC17_SYSCON_PCONP);
  regval |= SYSCON_PCONP_PCGPDMA;
  putreg32(regval, LPC17_SYSCON_PCONP);

  /* Reset all channel configurations */

  for (i = 0; i < LPC17_NDMACH; i++)
    {
      putreg32(0, LPC17_DMACH_CONFIG(i));
    }

  /* Clear all DMA interrupts */

  putreg32(DMACH_ALL, LPC17_DMA_INTTCCLR);
  putreg32(DMACH_ALL, LPC17_DMA_INTERRCLR);

  /* Initialize the DMA state structure */

  sem_init(&g_gpdma.exclsem, 0, 1);
}

/****************************************************************************
 * Name: lpc17_dmaconfigure
 *
 * Description:
 *   Configure a DMA request.  Each DMA request may have two different DMA
 *   request sources.  This associates one of the sources with a DMA request.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void lpc17_dmaconfigure(uint8_t dmarequest, bool alternate)
{
  uint32_t regval;

  DEBUGASSERT(dmarequest < LPC17_NDMAREQ);

#ifdef LPC176x
  /* For the LPC176x family, only request numbers 8-15 have DMASEL bits */

  if (dmarequest < 8)
    {
      return;
    }

  dmarequest -= 8;
#endif

  /* Set or clear the DMASEL bit corresponding to the request number */

  regval = getreg32(LPC17_SYSCON_DMAREQSEL);

  if (alternate)
    {
      regval |= (1 << dmarequest);
    }
  else
    {
      regval &= ~(1 << dmarequest);
    }

  putreg32(regval, LPC17_SYSCON_DMAREQSEL);
}

/****************************************************************************
 * Name: lpc17_dmachannel
 *
 * Description:
 *   Allocate a DMA channel.  This function sets aside a DMA channel and
 *   gives the caller exclusive access to the DMA channel.
 *
 * Returned Value:
 *   One success, this function returns a non-NULL, void* DMA channel
 *   handle.  NULL is returned on any failure.  This function can fail only
 *   if no DMA channel is available.
 *
 ****************************************************************************/

DMA_HANDLE lpc17_dmachannel(void)
{
  struct lpc17_dmach_s *dmach = NULL;
  int ret;
  int i;

  /* Get exclusive access to the GPDMA state structure */

  do
    {
      ret = sem_wait(&g_gpdma.exclsem);
      DEBUGASSERT(ret == 0 || errno == EINTR);
    }
  while (ret < 0);

  /* Find an available DMA channel */

  for (i = 0; i < LPC17_NDMACH; i++)
    {
      if (!g_gpdma.dmach[i].inuse)
        {
          /* Found one! */

          dmach = &g_gpdma.dmach[i];
          g_gpdma.dmach[i].inuse = true;
          break;
        }
    }

  /* Return what we found (or not) */

  sem_post(&g_gpdma.exclsem);
  return (DMA_HANDLE)dmach;
}

/****************************************************************************
 * Name: lpc17_dmafree
 *
 * Description:
 *   Release a DMA channel.  NOTE:  The 'handle' used in this argument must
 *   NEVER be used again until lpc17_dmachannel() is called again to re-gain
 *   a valid handle.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void lpc17_dmafree(DMA_HANDLE handle)
{
  struct lpc17_dmach_s *dmach = (DMA_HANDLE)handle;

  DEBUGASSERT(dmach && dmach->inuse);

  /* Mark the channel available.  This is an atomic operation and needs no
   * special protection.
   */

  dmach->inuse = false;
}

/****************************************************************************
 * Name: lpc17_dmasetup
 *
 * Description:
 *   Configure DMA for one transfer.
 *
 ****************************************************************************/

int lpc17_dmasetup(DMA_HANDLE handle, uint32_t control, uint32_t config,
                   uint32_t srcaddr, uint32_t destaddr, size_t nxfrs)
{
  struct lpc17_dmach_s *dmach = (DMA_HANDLE)handle;

  DEBUGASSERT(dmach && dmach->inuse);
#warning "Missing logic"

  return -ENOSYS;
}

/****************************************************************************
 * Name: lpc17_dmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 ****************************************************************************/

int lpc17_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg)
{
  struct lpc17_dmach_s *dmach = (DMA_HANDLE)handle;

  DEBUGASSERT(dmach && dmach->inuse && callback);

  /* Save the callback information */

  dmach->callback = callback;
  dmach->arg      = arg;

  /* Start the DMA */
#warning "Missing logic"
  return -ENOSYS;
}

/****************************************************************************
 * Name: lpc17_dmastop
 *
 * Description:
 *   Cancel the DMA.  After lpc17_dmastop() is called, the DMA channel is
 *   reset and lpc17_dmasetup() must be called before lpc17_dmastart() can be
 *   called again
 *
 ****************************************************************************/

void lpc17_dmastop(DMA_HANDLE handle)
{
  struct lpc17_dmach_s *dmach = (DMA_HANDLE)handle;

  DEBUGASSERT(dmach && dmach->inuse);
#warning "Missing logic"
}

/****************************************************************************
 * Name: lpc17_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void lpc17_dmasample(DMA_HANDLE handle, struct lpc17_dmaregs_s *regs)
{
  struct lpc17_dmach_s *dmach = (DMA_HANDLE)handle;

  DEBUGASSERT(dmach && dmach->inuse);
#warning "Missing logic"
}
#endif /* CONFIG_DEBUG_DMA */

/****************************************************************************
 * Name: lpc17_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void lpc17_dmadump(DMA_HANDLE handle, const struct lpc17_dmaregs_s *regs, const char *msg)
{
  struct lpc17_dmach_s *dmach = (DMA_HANDLE)handle;

  DEBUGASSERT(dmach && dmach->inuse);
#warning "Missing logic"
}
#endif /* CONFIG_DEBUG_DMA */

#endif /* CONFIG_LPC17_GPDMA */
