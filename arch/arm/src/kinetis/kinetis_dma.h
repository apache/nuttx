/****************************************************************************
 * arch/arm/src/kinetis/kinetis_dma.h
 *
 *   Copyright (C) 2016-2018 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
 *            Jan Okle <jan@leitwert.ch>
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

#ifndef __ARCH_ARM_SRC_KINETEIS_KINETEIS_DMA_H
#define __ARCH_ARM_SRC_KINETEIS_KINETEIS_DMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

#include "chip/kinetis_dma.h"

/****************************************************************************
 * Pre-processor Declarations
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef FAR void *DMA_HANDLE;
typedef void (*dma_callback_t)(DMA_HANDLE handle, void *arg, int result);

/* The following is used for sampling DMA registers when CONFIG DEBUG_DMA is
 * selected.
 */

#ifdef CONFIG_DEBUG_DMA
struct kinetis_dmaglobalregs_s
{
#warning "Missing logic"
  /* Global Registers */
};

struct kinetis_dmachanregs_s
{
#warning "Missing logic"
  /* Channel Registers */
};

struct kinetis_dmaregs_s
{
  /* Global Registers */

  struct kinetis_dmaglobalregs_s gbl;

  /* Channel Registers */

  struct kinetis_dmachanregs_s   ch;
};
#endif

enum kinetis_dma_direction_e
{
  KINETIS_DMA_DIRECTION_PERIPHERAL_TO_MEMORY,
  KINETIS_DMA_DIRECTION_MEMORY_TO_PERIPHERAL
};

/* Kinetis data transfer size */

enum kinetis_dma_data_sz_e
{
  KINETIS_DMA_DATA_SZ_8BIT = 0,
  KINETIS_DMA_DATA_SZ_16BIT = 1,
  KINETIS_DMA_DATA_SZ_32BIT = 2,
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

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
 * Name: kinetis_dmainitialize
 *
 * Description:
 *   Initialize the GPDMA subsystem.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void kinetis_dmainitialize(void);

/****************************************************************************
 * Name: kinetis_dmachannel
 *
 * Description:
 *   Allocate a DMA channel.  This function sets aside a DMA channel and
 *   gives the caller exclusive access to the DMA channel.
 *
 * Input Parameters:
 *   src         - DMA request source
 *   per_addr    - Address of the peripheral data
 *   per_data_sz - Peripheral data size (register size). Note that if this
 *                 does not agree with the peripheral register size, DMA
 *                 transfers will silently fail during operation.
 *   dir         - transfer direction
 *
 * Returned Value:
 *   One success, this function returns a non-NULL, void * DMA channel
 *   handle.  NULL is returned on any failure.  This function can fail only
 *   if no DMA channel is available.
 *
 ****************************************************************************/

DMA_HANDLE kinetis_dmachannel(uint8_t src,
                              uint32_t per_addr,
                              enum kinetis_dma_data_sz_e per_data_sz,
                              enum kinetis_dma_direction_e dir);

/****************************************************************************
 * Name: kinetis_dmafree
 *
 * Description:
 *   Release a DMA channel.  NOTE:  The 'handle' used in this argument must
 *   NEVER be used again until kinetis_dmachannel() is called again to re-
 *   gain a valid handle.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void kinetis_dmafree(DMA_HANDLE handle);

/****************************************************************************
 * Name: kinetis_dmasetup
 *
 * Description:
 *   Configure DMA for one transfer.
 *
 * Input Parameters:
 *   mem_addr   - Memory address
 *   ntransfers - Number of transfers. Must be 0<= ntransfers <= 0x7FFF
 *   control    - Channel control configuration
 *
 * Returned Value:
 *   result: 0 if ok, negative else
 *
 ****************************************************************************/

int kinetis_dmasetup(DMA_HANDLE handle, uint32_t mem_addr,
                     size_t ntransfers, uint16_t control);

/****************************************************************************
 * Name: kinetis_dmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 ****************************************************************************/

int kinetis_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg);

/****************************************************************************
 * Name: kinetis_dmastop
 *
 * Description:
 *   Cancel the DMA.  After kinetis_dmastop() is called, the DMA channel is
 *   reset and kinetis_dmasetup() must be called before kinetis_dmastart()
 *   can be called again
 *
 ****************************************************************************/

void kinetis_dmastop(DMA_HANDLE handle);

/****************************************************************************
 * Name: kinetis_dmaresidual
 *
 * Description:
 *   Returns the number of transfers left
 *
 * Returned Value:
 *   Residual transfers
 ****************************************************************************/

size_t kinetis_dmaresidual(DMA_HANDLE handle);

/****************************************************************************
 * Name: kinetis_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void kinetis_dmasample(DMA_HANDLE handle, struct kinetis_dmaregs_s *regs);
#else
#  define kinetis_dmasample(handle,regs)
#endif

/****************************************************************************
 * Name: kinetis_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void kinetis_dmadump(DMA_HANDLE handle, const struct kinetis_dmaregs_s *regs,
                     const char *msg);
#else
#  define kinetis_dmadump(handle,regs,msg)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_KINETEIS_KINETEIS_DMA_H */
