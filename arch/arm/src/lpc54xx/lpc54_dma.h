/****************************************************************************
 * arch/arm/src/lpc54xx/lpc54_dma.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_LPC54XX_LP54_DMA_H
#define __ARCH_ARM_SRC_LPC54XX_LP54_DMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip/lpc54_dma.h"

#ifdef CONFIG_LPC54_DMA

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

typedef FAR void *DMA_HANDLE;
typedef void (*dma_callback_t)(DMA_HANDLE handle, void *arg, int result);

/* The following is used for sampling DMA registers when CONFIG DEBUG_DMA is selected */

#ifdef CONFIG_DEBUG_DMA
struct lpc54_dmaglobalregs_s
{
  /* Global Registers */

  uint32_t ctrl;         /* DMA control */
  uint32_t intstat;      /* DMA Interrupt status */
  uint32_t srambase;     /* SRAM address of the channel configuration table */
  uint32_t enableset0;   /* DMA Channel enable read and set */
  uint32_t active0;      /* DMA Channel active status */
  uint32_t busy0;        /* DMA Channel busy status */
  uint32_t errint0;      /* DMA Error interrupt status */
  uint32_t intenset0;    /* DMA Interrupt enable read and set */
  uint32_t inta0;        /* DMA Interrupt A status */
  uint32_t intb0;        /* DMA Interrupt B status */
};

struct lpc54_dmachanregs_s
{
  /* Channel Registers */

  uint32_t cfg;          /* DMA Configuration register */
  uint32_t ctlstat;      /* DMA Control and status register */
  uint32_t xfercfg;      /* DMA Transfer configuration register */
};

struct lpc54_dmaregs_s
{
  /* Global Registers */

  struct lpc54_dmaglobalregs_s gbl;

  /* Channel Registers */

  struct lpc54_dmachanregs_s   ch;
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc54_dmachannel
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

DMA_HANDLE lpc54_dmachannel(void);

/****************************************************************************
 * Name: lpc54_dmafree
 *
 * Description:
 *   Release a DMA channel.  NOTE:  The 'handle' used in this argument must
 *   NEVER be used again until lpc54_dmachannel() is called again to re-gain
 *   a valid handle.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void lpc54_dmafree(DMA_HANDLE handle);

/****************************************************************************
 * Name: lpc54_dmasetup
 *
 * Description:
 *   Configure DMA for one transfer.
 *
 ****************************************************************************/

int lpc54_dmarxsetup(DMA_HANDLE handle, uint32_t control, uint32_t config,
                     uint32_t srcaddr, uint32_t destaddr, size_t nbytes);

/****************************************************************************
 * Name: lpc54_dmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 ****************************************************************************/

int lpc54_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg);

/****************************************************************************
 * Name: lpc54_dmastop
 *
 * Description:
 *   Cancel the DMA.  After lpc54_dmastop() is called, the DMA channel is
 *   reset and lpc54_dmasetup() must be called before lpc54_dmastart() can be
 *   called again
 *
 ****************************************************************************/

void lpc54_dmastop(DMA_HANDLE handle);

/****************************************************************************
 * Name: lpc54_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void lpc54_dmasample(DMA_HANDLE handle, struct lpc54_dmaregs_s *regs);
#else
#  define lpc54_dmasample(handle,regs)
#endif

/****************************************************************************
 * Name: lpc54_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void lpc54_dmadump(DMA_HANDLE handle, const struct lpc54_dmaregs_s *regs,
                   const char *msg);
#else
#  define lpc54_dmadump(handle,regs,msg)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_LPC54_DMA */
#endif /* __ARCH_ARM_SRC_LPC54XX_LP54_DMA_H */
