/****************************************************************************
 * arch/arm/src/kl/kl_dma.h
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

#ifndef __ARCH_ARM_SRC_KL_KL_DMA_H
#define __ARCH_ARM_SRC_KL_KL_DMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "kl_config.h"

/****************************************************************************
 * Pre-processor Declarations
 ****************************************************************************/

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
 * Public Types
 ****************************************************************************/

typedef void *DMA_HANDLE;
typedef void (*dma_callback_t)(DMA_HANDLE handle, void *arg, int result);

/* The following is used for sampling DMA registers when CONFIG DEBUG_DMA
 * is selected
 */

#ifdef CONFIG_DEBUG_DMA
struct kl_dmaglobalregs_s
{
#warning "Missing logic"
  /* Global Registers */
};

struct kl_dmachanregs_s
{
#warning "Missing logic"
  /* Channel Registers */
};

struct kl_dmaregs_s
{
  /* Global Registers */

  struct kl_dmaglobalregs_s gbl;

  /* Channel Registers */

  struct kl_dmachanregs_s   ch;
};
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: kl_dmainitialize
 *
 * Description:
 *   Initialize the GPDMA subsystem.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_KL_DMA
void kl_dmainitilaize(void);
#endif

/****************************************************************************
 * Name: kl_dmachannel
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

#ifdef CONFIG_KL_DMA
DMA_HANDLE kl_dmachannel(void);
#endif

/****************************************************************************
 * Name: kl_dmafree
 *
 * Description:
 *   Release a DMA channel.  NOTE:  The 'handle' used in this argument must
 *   NEVER be used again until kl_dmachannel() is called again to re-gain
 *   a valid handle.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_KL_DMA
void kl_dmafree(DMA_HANDLE handle);
#endif

/****************************************************************************
 * Name: kl_dmasetup
 *
 * Description:
 *   Configure DMA for one transfer.
 *
 ****************************************************************************/

#ifdef CONFIG_KL_DMA
int kl_dmarxsetup(DMA_HANDLE handle, uint32_t control, uint32_t config,
                  uint32_t srcaddr, uint32_t destaddr, size_t nbytes);
#endif

/****************************************************************************
 * Name: kl_dmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 ****************************************************************************/

#ifdef CONFIG_KL_DMA
int kl_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg);
#endif

/****************************************************************************
 * Name: kl_dmastop
 *
 * Description:
 *   Cancel the DMA.  After kl_dmastop() is called, the DMA channel is
 *   reset and kl_dmasetup() must be called before kl_dmastart() can be
 *   called again
 *
 ****************************************************************************/

#ifdef CONFIG_KL_DMA
void kl_dmastop(DMA_HANDLE handle);
#endif

/****************************************************************************
 * Name: kl_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG_KL_DMA
#ifdef CONFIG_DEBUG_DMA
void kl_dmasample(DMA_HANDLE handle, struct kl_dmaregs_s *regs);
#else
#  define kl_dmasample(handle,regs)
#endif
#endif

/****************************************************************************
 * Name: kl_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG_KL_DMA
#ifdef CONFIG_DEBUG_DMA
void kl_dmadump(DMA_HANDLE handle, const struct kl_dmaregs_s *regs,
                const char *msg);
#else
#  define kl_dmadump(handle,regs,msg)
#endif
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_KL_KL_DMA_H */
