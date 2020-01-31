/************************************************************************************
 * arch/arm/src/stm32/stm32_dma.h
 *
 *   Copyright (C) 2009, 2011-2013, 2018 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32F0L0G0_STM32_DMA_H
#define __ARCH_ARM_SRC_STM32F0L0G0_STM32_DMA_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

#include "hardware/stm32_dma_v1.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* These definitions provide the bit encoding of the 'status' parameter passed to the
 * DMA callback function (see dma_callback_t).
 */

#define DMA_STATUS_TEIF     DMA_CHAN_TEIF_BIT     /* Channel Transfer Error */
#define DMA_STATUS_HTIF     DMA_CHAN_HTIF_BIT     /* Channel Half Transfer */
#define DMA_STATUS_TCIF     DMA_CHAN_TCIF_BIT     /* Channel Transfer Complete */

#define DMA_STATUS_ERROR    (DMA_STATUS_TEIF)
#define DMA_STATUS_SUCCESS  (DMA_STATUS_TCIF | DMA_STATUS_HTIF)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/* DMA_HANDLE provides an opaque are reference that can be used to represent a DMA
 * channel (F1) or a DMA stream (F4).
 */

typedef FAR void *DMA_HANDLE;

/* Description:
 *   This is the type of the callback that is used to inform the user of the
 *   completion of the DMA.
 *
 * Input Parameters:
 *   handle - Refers tot he DMA channel or stream
 *   status - A bit encoded value that provides the completion status.  See the
 *            DMASTATUS_* definitions above.
 *   arg    - A user-provided value that was provided when stm32_dmastart() was
 *            called.
 */

typedef void (*dma_callback_t)(DMA_HANDLE handle, uint8_t status, void *arg);

#ifdef CONFIG_DEBUG_DMA_INFO
struct stm32_dmaregs_s
{
  uint32_t isr;
  uint32_t ccr;
  uint32_t cndtr;
  uint32_t cpar;
  uint32_t cmar;
};
#endif

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

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
 *     For the STM32 F1, this is simply the channel number as provided by
 *     the DMACHAN_* definitions in chip/stm32f10xxx_dma.h.
 *     For the STM32 F4, this is a bit encoded value as provided by the
 *     the DMAMAP_* definitions in chip/stm32f40xxx_dma.h
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
 *   Release a DMA channel.  If another thread is waiting for this DMA channel
 *   in a call to stm32_dmachannel, then this function will re-assign the
 *   DMA channel to that thread and wake it up.  NOTE:  The 'handle' used
 *   in this argument must NEVER be used again until stm32_dmachannel() is
 *   called again to re-gain access to the channel.
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
 *   Start the DMA transfer
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
 *   connections in the ARM bus matrix of the processor. Note that this
 *   only applies to memory addresses, it will return false for any peripheral
 *   address.
 *
 * Returned Value:
 *   True, if transfer is possible.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32F0L0G0_DMACAPABLE
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

/* High performance, zero latency DMA interrupts need some additional
 * interfaces.
 *
 * TODO: For now the interface is different for STM32 DMAv1 and STM32 DMAv2.
 *       It should be unified somehow.
 */

#ifdef CONFIG_ARCH_HIPRI_INTERRUPT

/****************************************************************************
 * Name: stm32_dma_intack
 *
 * Description:
 *   Public visible interface to acknowledge interrupts on DMA channel
 *
 ****************************************************************************/

#if defined(HAVE_IP_DMA_V1)
void stm32_dma_intack(unsigned int chndx, uint32_t isr);
#elif defined(HAVE_IP_DMA_V2)
void stm32_dma_intack(unsigned int controller, uint8_t stream, uint32_t isr);
#endif

/****************************************************************************
 * Name: stm32_dma_intget
 *
 * Description:
 *   Public visible interface to get pending interrupts from DMA channel
 *
 ****************************************************************************/

#if defined(HAVE_IP_DMA_V1)
uint32_t stm32_dma_intget(unsigned int chndx);
#elif defined(HAVE_IP_DMA_V2)
uint8_t  stm32_dma_intget(unsigned int controller, uint8_t stream);
#endif

#endif /* CONFIG_ARCH_HIPRI_INTERRUPT */

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32F0L0G0_STM32_DMA_H */
