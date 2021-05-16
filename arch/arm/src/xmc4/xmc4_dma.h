/****************************************************************************
 * arch/arm/src/xmc4/xmc4_dma.h
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

#ifndef __ARCH_ARM_SRC_XMC4_XMC4_DMA_H
#define __ARCH_ARM_SRC_XMC4_XMC4_DMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

#include "hardware/xmc4_dma.h"

/****************************************************************************
 * Pre-processor Declarations
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef FAR void *DMA_HANDLE;
typedef void (*dma_callback_t)(DMA_HANDLE handle, void *arg, int result);

/* The following is used for sampling DMA registers when CONFIG DEBUG_DMA is
 * selected
 */

#ifdef CONFIG_DEBUG_DMA
struct xmc4_dmaglobalregs_s
{
#warning "Missing logic"
  /* Global Registers */
};

struct xmc4_dmachanregs_s
{
#warning "Missing logic"
  /* Channel Registers */
};

struct xmc4_dmaregs_s
{
  /* Global Registers */

  struct xmc4_dmaglobalregs_s gbl;

  /* Channel Registers */

  struct xmc4_dmachanregs_s   ch;
};
#endif

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
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: xmc4_dmainitialize
 *
 * Description:
 *   Initialize the GPDMA subsystem.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void xmc4_dmainitilaize(void);

/****************************************************************************
 * Name: xmc4_dmachannel
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

DMA_HANDLE xmc4_dmachannel(void);

/****************************************************************************
 * Name: xmc4_dmafree
 *
 * Description:
 *   Release a DMA channel.  NOTE:  The 'handle' used in this argument must
 *   NEVER be used again until xmc4_dmachannel() is called again to re-gain
 *   a valid handle.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void xmc4_dmafree(DMA_HANDLE handle);

/****************************************************************************
 * Name: xmc4_dmasetup
 *
 * Description:
 *   Configure DMA for one transfer.
 *
 ****************************************************************************/

int xmc4_dmarxsetup(DMA_HANDLE handle, uint32_t control, uint32_t config,
                       uint32_t srcaddr, uint32_t destaddr, size_t nbytes);

/****************************************************************************
 * Name: xmc4_dmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 ****************************************************************************/

int xmc4_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg);

/****************************************************************************
 * Name: xmc4_dmastop
 *
 * Description:
 *   Cancel the DMA.  After xmc4_dmastop() is called, the DMA channel is
 *   reset and xmc4_dmasetup() must be called before xmc4_dmastart() can be
 *   called again
 *
 ****************************************************************************/

void xmc4_dmastop(DMA_HANDLE handle);

/****************************************************************************
 * Name: xmc4_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void xmc4_dmasample(DMA_HANDLE handle, struct xmc4_dmaregs_s *regs);
#else
#  define xmc4_dmasample(handle,regs)
#endif

/****************************************************************************
 * Name: xmc4_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void xmc4_dmadump(DMA_HANDLE handle, const struct xmc4_dmaregs_s *regs,
                     const char *msg);
#else
#  define xmc4_dmadump(handle,regs,msg)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_XMC4_XMC4_DMA_H */
