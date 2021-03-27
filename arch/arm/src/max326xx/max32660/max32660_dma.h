/****************************************************************************
 * arch/arm/src/max326xx/max32660/max32660_dma.h
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

#ifndef __ARCH_ARM_SRC_MAX436XX_MAX32660_MAX32660_DMA_H
#define __ARCH_ARM_SRC_MAX436XX_MAX32660_MAX32660_DMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* DMA channel handle and DMA completion callback function */

typedef FAR void *DMA_HANDLE;
typedef void (*dma_callback_t)(DMA_HANDLE handle, void *arg, int result);

/* The following is used for sampling DMA registers when CONFIG DEBUG_DMA is
 * selected
 */

#ifdef CONFIG_DEBUG_DMA_INFO
struct max326_dmaregs_s
{
  /* Global Registers */

  uint32_t inten;   /* DMA Control register */
  uint32_t intfl;   /* DMA Interrupt Status register */

  /* Channel Registers */

  uint32_t cfg;     /* DMA Channel Configuration Register */
  uint32_t stat;    /* DMA Channel Status Register */
  uint32_t src;     /* DMA Channel Source Register */
  uint32_t dst;     /* DMA Channel Destination Register */
  uint32_t cnt;     /* DMA Channel Count Register */
  uint32_t srcrld;  /* DMA Channel Source Reload Register */
  uint32_t dstrld;  /* DMA Channel Destination Reload Register */
  uint32_t cntrld;  /* DMA Channel Count Reload Register */
};
#endif

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

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
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: max326_dma_channel
 *
 * Description:
 *   Allocate a DMA channel.
 *
 * Returned Value:
 *   If a DMA channel is available, this function returns a non-NULL, void*
 *   DMA channel handle.  NULL is returned on any failure.
 *
 ****************************************************************************/

DMA_HANDLE max326_dma_channel(void);

/****************************************************************************
 * Name: max326_dma_free
 *
 * Description:
 *   Release a DMA channel.  NOTE:  The 'handle' used in this argument must
 *   NEVER be used again until max326_dma_channel() is called again to
 *   re-gain a valid handle.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void max326_dma_free(DMA_HANDLE handle);

/****************************************************************************
 * Name: max326_dma_setup
 *
 * Description:
 *   Configure DMA for one transfer.
 *
 *   NOTE that an image of the DMA CFG is a required input.  Only the
 *   following fields need be provided however.  The DMA logic will handle
 *   the rest.
 *
 *     CFG          DMA priority
 *     REQSEL       Request Select
 *     REQWAIT      Request Wait Enable
 *     TOSEL        Time-Out Select
 *     PSSEL        Pre-Scale Select
 *     SRCWD        Source Width
 *     SRCINC       Source Increment Enable
 *     DSTWD        Destination Width
 *     DSTINC       Destination Increment Enable
 *     BRST         Burst Size
 *
 ****************************************************************************/

int max326_dma_setup(DMA_HANDLE handle, uint32_t cfg, uint32_t saddr,
                     uint32_t daddr, size_t nbytes);

/****************************************************************************
 * Name: max326_dma_append
 *
 * Description:
 *   Append one buffer to the DMA chain.  max326_dma_setup() should have
 *   been called to set up the first buffer.  This function may be called
 *   to chain the DMA to a second buffer.  This is done by setting the
 *   source, destination, and count reload registers.
 *
 *   REVISIT:  Currently, the implementation is limited to a single
 *   appended buffer in the chain.
 *
 ****************************************************************************/

int max326_dma_append(DMA_HANDLE handle, uint32_t saddr, uint32_t daddr,
                      size_t nbytes);

/****************************************************************************
 * Name: max326_dmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 ****************************************************************************/

int max326_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg);

/****************************************************************************
 * Name: max326_dmastop
 *
 * Description:
 *   Cancel the DMA.  After max326_dmastop() is called, the DMA channel is
 *   reset and max326_dma_setup() must be called before max326_dmastart() can
 *   be called again
 *
 ****************************************************************************/

void max326_dmastop(DMA_HANDLE handle);

/****************************************************************************
 * Name: max326_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA_INFO
void max326_dmasample(DMA_HANDLE handle, struct max326_dmaregs_s *regs);
#else
#  define max326_dmasample(handle,regs)
#endif

/****************************************************************************
 * Name: max326_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA_INFO
void max326_dmadump(DMA_HANDLE handle, const struct max326_dmaregs_s *regs,
                    const char *msg);
#else
#  define max326_dmadump(handle,regs,msg)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_MAX436XX_MAX32660_MAX32660_DMA_H */
