/****************************************************************************
 * arch/arm/src/stm32h7/stm32_dma.h
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

#ifndef __ARCH_ARM_SRC_STM32H7_STM32_DMA_H
#define __ARCH_ARM_SRC_STM32H7_STM32_DMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

#include "hardware/stm32_dma.h"
#include "hardware/stm32_dmamux.h"

/* These definitions provide the bit encoding of the 'status' parameter
 * passed to the DMA callback function (see dma_callback_t).
 */

#define DMA_STATUS_FEIF           0                    /* Stream FIFO error (ignored) */
#define DMA_STATUS_DMEIF          DMA_STREAM_DMEIF_BIT /* Stream direct mode error */
#define DMA_STATUS_TEIF           DMA_STREAM_TEIF_BIT  /* Stream Transfer Error */
#define DMA_STATUS_HTIF           DMA_STREAM_HTIF_BIT  /* Stream Half Transfer */
#define DMA_STATUS_TCIF           DMA_STREAM_TCIF_BIT  /* Stream Transfer Complete */

#define DMA_STATUS_ERROR          (DMA_STATUS_FEIF | DMA_STATUS_DMEIF | DMA_STATUS_TEIF)
#define DMA_STATUS_SUCCESS        (DMA_STATUS_TCIF | DMA_STATUS_HTIF)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* DMA channel configuration - common for DMA1 DMA2 MDMA and BDMA */

struct stm32_dma_config_s
{
  uint32_t paddr;      /* Peripheral address */
  uint32_t maddr;      /* Memory address */
  uint32_t cfg1;       /* DMA transfer configuration 1.
                        * Its function depends on DMA controller:
                        *   - DMA1 and DMA2 - SCR register configuration
                        *   - BDMA - CCR register configuration
                        *   - MDMA - CCR register configuration
                        */
  uint32_t cfg2;       /* DMA transfer configuration 2.
                        * Its function depends on DMA controller:
                        *   - MDMA - CTCR register configuration
                        */
  uint32_t ndata;      /* Number of data to transfer */
};

typedef struct stm32_dma_config_s stm32_dmacfg_t;

/* DMA_HANDLE Provides an opaque reference that can be used to represent a
 * DMA stream.
 */

typedef void *DMA_HANDLE;

/* Description:
 *   This is the type of the callback that is used to inform the user of the
 *   completion of the DMA.  NOTE:  The DMA module does *NOT* perform any
 *   cache operations.  It is the responsibility of the DMA client to
 *   invalidate DMA buffers after completion of the DMA RX operations.
 *
 * Input Parameters:
 *   handle - Refers to the DMA channel or stream
 *   status - A bit encoded value that provides the completion status.  See
 *            the DMASTATUS_* definitions above.
 *   arg    - A user-provided value that was provided when stm32_dmastart()
 *            was called.
 */

typedef void (*dma_callback_t)(DMA_HANDLE handle, uint8_t status, void *arg);

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
 * Name: stm32_dmachannel
 *
 * Description:
 *   Allocate a DMA channel.  This function gives the caller mutually
 *   exclusive access to the DMA channel specified by the 'dmamap' argument.
 *   It is common for standard DMA (DMA1, DMA2), master DMA (MDMA) and
 *   basic DMA (BDMA) controllers.
 *
 * Input Parameters:
 *   dmamap - Identifies the stream/channel resource. For the STM32 H7, this
 *     is a bit-encoded  value as provided by the DMAMAP_* definitions
 *     in chip/stm32h7xxxxxxx_dmamux.h
 *
 * Returned Value:
 *   On success, this function returns a non-NULL, void* DMA channel handle.
 *   NULL is returned on any failure.  This function can fail only if no DMA
 *   channel is available.
 *
 * Assumptions:
 *   - The caller does not hold he DMA channel.
 *   - The caller can wait for the DMA channel to be freed if it is no
 *     available.
 *
 ****************************************************************************/

DMA_HANDLE stm32_dmachannel(unsigned int dmamap);

/****************************************************************************
 * Name: stm32_dmafree
 *
 * Description:
 *   Release a DMA channel and unmap DMAMUX if required.
 *
 *   NOTE:  The 'handle' used in this argument must NEVER be used again
 *   until stm32_dmachannel() is called again to re-gain access to the
 *   channel.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   - The caller holds the DMA channel.
 *   - There is no DMA in progress
 *
 ****************************************************************************/

void stm32_dmafree(DMA_HANDLE handle);

/****************************************************************************
 * Name: stm32_dmasetup
 *
 * Description:
 *   Configure DMA before using
 *
 ****************************************************************************/

void stm32_dmasetup(DMA_HANDLE handle, stm32_dmacfg_t *cfg);

/****************************************************************************
 * Name: stm32_dmastart
 ****************************************************************************/

void stm32_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg,
                    bool half);

/****************************************************************************
 * Name: stm32_dmastop
 ****************************************************************************/

void stm32_dmastop(DMA_HANDLE handle);

/****************************************************************************
 * Name: stm32_dmaresidual
 ****************************************************************************/

size_t stm32_dmaresidual(DMA_HANDLE handle);

/****************************************************************************
 * Name: stm32_dmacapable
 ****************************************************************************/

#ifdef CONFIG_STM32H7_DMACAPABLE
bool stm32_dmacapable(DMA_HANDLE handle, stm32_dmacfg_t *cfg);
#else
#  define stm32_dmacapable(handle, cfg) (true)
#endif

/****************************************************************************
 * Name: stm32_dmadump
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA_INFO
void stm32_dmadump(DMA_HANDLE handle, const char *msg);
#else
#  define stm32_dmadump(handle,msg)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32H7_STM32_DMA_H */
