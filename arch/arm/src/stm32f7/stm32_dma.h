/****************************************************************************
 * arch/arm/src/stm32f7/stm32_dma.h
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

#ifndef __ARCH_ARM_SRC_STM32F7_STM32_DMA_H
#define __ARCH_ARM_SRC_STM32F7_STM32_DMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

#include "hardware/stm32_dma.h"

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

struct stm32_dmaregs_s
{
  uint32_t lisr;
  uint32_t hisr;
  uint32_t scr;
  uint32_t sndtr;
  uint32_t spar;
  uint32_t sm0ar;
  uint32_t sm1ar;
  uint32_t sfcr;
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
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_dmachannel
 *
 * Description:
 *   Allocate a DMA channel.  This function gives the caller mutually
 *   exclusive access to the DMA channel specified by the 'chan' argument.
 *   DMA channels are shared on the STM32:  Devices sharing the same DMA
 *   channel cannot do DMA concurrently!  See the DMACHAN_* definitions in
 *   stm32_dma.h.
 *
 *   If the DMA channel is not available, then stm32_dmachannel() will wait
 *   until the holder of the channel relinquishes the channel by calling
 *   stm32_dmafree().  WARNING: If you have two devices sharing a DMA
 *   channel and the code never releases the channel, the stm32_dmachannel
 *   call for the other will hang forever in this function!  Don't let your
 *   design do that!
 *
 *   Hmm.. I suppose this interface could be extended to make a non-blocking
 *   version.  Feel free to do that if that is what you need.
 *
 * Input Parameters:
 *   chan - Identifies the stream/channel resource
 *     For the STM32 F7, this is a bit encoded value as provided by the
 *     the DMAMAP_* definitions  in chip/stm32f7xxxxxxx_dma.h
 *
 * Returned Value:
 *   Provided that 'chan' is valid, this function ALWAYS returns a non-NULL,
 *   void* DMA channel handle.  (If 'chan' is invalid, the function will
 *   assert if debug is enabled or do something ignorant otherwise).
 *
 * Assumptions:
 *   - The caller does not hold he DMA channel.
 *   - The caller can wait for the DMA channel to be freed if it is no
 *     available.
 *
 ****************************************************************************/

DMA_HANDLE stm32_dmachannel(unsigned int chan);

/****************************************************************************
 * Name: stm32_dmafree
 *
 * Description:
 *   Release a DMA channel.  If another thread is waiting for this DMA
 *   channel in a call to stm32_dmachannel, then this function will re-
 *   assign the DMA channel to that thread and wake it up.
 *
 *   NOTE:  The 'handle' used in this argument must NEVER be used again
 *          until stm32_dmachannel() is called again to re-gain access to
 *          the channel.
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

void stm32_dmasetup(DMA_HANDLE handle, uint32_t paddr, uint32_t maddr,
                    size_t ntransfers, uint32_t ccr);

/****************************************************************************
 * Name: stm32_dmastart
 *
 * Description:
 *   Start the DMA transfer.  NOTE:  The DMA module does *NOT* perform any
 *   cache operations.  It is the responsibility of the DMA client to clean
 *   DMA buffers after staring of the DMA TX operations.
 *
 * Assumptions:
 *   - DMA handle allocated by stm32_dmachannel()
 *   - No DMA in progress
 *
 ****************************************************************************/

void stm32_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg,
                    bool half);

/****************************************************************************
 * Name: stm32_dmastop
 *
 * Description:
 *   Cancel the DMA.  After stm32_dmastop() is called, the DMA channel is
 *   reset and stm32_dmasetup() must be called before stm32_dmastart() can be
 *   called again
 *
 * Assumptions:
 *   - DMA handle allocated by stm32_dmachannel()
 *
 ****************************************************************************/

void stm32_dmastop(DMA_HANDLE handle);

/****************************************************************************
 * Name: stm32_dmaresidual
 *
 * Description:
 *   Returns the number of bytes remaining to be transferred
 *
 * Assumptions:
 *   - DMA handle allocated by stm32_dmachannel()
 *
 ****************************************************************************/

size_t stm32_dmaresidual(DMA_HANDLE handle);

/****************************************************************************
 * Name: stm32_dmacapable
 *
 * Description:
 *   Check if the DMA controller can transfer data to/from given memory
 *   address with the given configuration. This depends on the internal
 *   connections in the ARM bus matrix of the processor. Note that this only
 *   applies to memory addresses, it will return false for any peripheral
 *   address.
 *
 * Input Parameters:
 *
 *   maddr - starting memory address
 *   count - number of unit8 or uint16 or uint32 items as defined by MSIZE
 *           of ccr.
 *   ccr   - DMA stream configuration register
 *
 * Returned Value:
 *   True, if transfer is possible.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32F7_DMACAPABLE
bool stm32_dmacapable(uintptr_t maddr, uint32_t count, uint32_t ccr);
#else
#  define stm32_dmacapable(maddr, count, ccr) (true)
#endif

/****************************************************************************
 * Name: stm32_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by stm32_dmachannel()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA_INFO
void stm32_dmasample(DMA_HANDLE handle, struct stm32_dmaregs_s *regs);
#else
#  define stm32_dmasample(handle,regs)
#endif

/****************************************************************************
 * Name: stm32_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by stm32_dmachannel()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA_INFO
void stm32_dmadump(DMA_HANDLE handle, const struct stm32_dmaregs_s *regs,
                   const char *msg);
#else
#  define stm32_dmadump(handle,regs,msg)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32F7_STM32_DMA_H */
