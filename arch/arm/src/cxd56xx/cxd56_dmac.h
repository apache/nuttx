/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_dmac.h
 *
 *   Copyright (C) 2009, 2011-2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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

#ifndef __ARCH_ARM_SRC_CXD56XX_CXD56_DMAC_H
#define __ARCH_ARM_SRC_CXD56XX_CXD56_DMAC_H

#include <stdint.h>

#include "cxd56_dmac_common.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CXD56_DMA_PERIPHERAL_MASK       (0x0f)
#define CXD56_DMA_PERIPHERAL_UART2_TX (0)
#define CXD56_DMA_PERIPHERAL_UART2_RX (1)
#define CXD56_DMA_PERIPHERAL_SPI4_TX  (2)
#define CXD56_DMA_PERIPHERAL_SPI4_RX  (3)
#define CXD56_DMA_PERIPHERAL_SPI5_TX  (4)
#define CXD56_DMA_PERIPHERAL_SPI5_RX  (5)

#define CXD56_DMA_INTR_ITC (1u<<0) /**< Terminal count interrupt status */
#define CXD56_DMA_INTR_ERR (1u<<1) /**< Error interrupt status */

#define CXD56_DMAC_WIDTH8   0      /**< 8 bit width */
#define CXD56_DMAC_WIDTH16  1      /**< 16 bit width */
#define CXD56_DMAC_WIDTH32  2      /**< 32 bit width */

/****************************************************************************
 * Public Types
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
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: cxd56_dmachannel
 *
 * Description:
 *  Allocate a DMA channel. This function gives the caller mutually exclusive
 *  access to a DMA channel.
 *
 *  If no DMA channel is available, then cxd56_dmachannel() will wait until
 *  the holder of a channel relinquishes the channel by calling
 *  cxd56_dmafree().
 *
 * Input parameters:
 *   ch      - DMA channel to use
 *   maxsize - Max size to be transferred in bytes
 *
 * Returned Value:
 *   This function ALWAYS returns a non-NULL, void* DMA channel handle.
 *
 * Assumptions:
 *  - The caller can wait for a DMA channel to be freed if it is not
 *    available.
 *
 ****************************************************************************/

DMA_HANDLE cxd56_dmachannel(int ch, ssize_t maxsize);

/****************************************************************************
 * Name: cxd56_dmafree
 *
 * Description:
 *  Release a DMA channel.  If another thread is waiting for this DMA channel
 *  in a call to cxd56_dmachannel, then this function will re-assign the DMA
 *  channel to that thread and wake it up.
 *
 *  NOTE:  The 'handle' used in this argument must NEVER be used again until
 *  cxd56_dmachannel() is called again to re-gain access to the channel.
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *  - The caller holds the DMA channel.
 *  - There is no DMA in progress
 *
 ****************************************************************************/

void cxd56_dmafree(DMA_HANDLE handle);

/****************************************************************************
 * Name: cxd56_rxdmasetup
 *
 * Description:
 *   Configure an RX (peripheral-to-memory) DMA before starting the transfer.
 *
 * Input Parameters:
 *   paddr  - Peripheral address (source)
 *   maddr  - Memory address (destination)
 *   nbytes - Number of bytes to transfer.  Must be an even multiple of the
 *            configured transfer size.
 *   config - Channel configuration selections
 *
 ****************************************************************************/

void cxd56_rxdmasetup(DMA_HANDLE handle, uintptr_t paddr, uintptr_t maddr,
                      size_t nbytes, dma_config_t config);

/****************************************************************************
 * Name: cxd56_txdmasetup
 *
 * Description:
 *   Configure an TX (memory-to-peripheral) DMA before starting the transfer.
 *
 * Input Parameters:
 *   paddr  - Peripheral address (destination)
 *   maddr  - Memory address (source)
 *   nbytes - Number of bytes to transfer.  Must be an even multiple of the
 *            configured transfer size.
 *   config - Channel configuration selections
 *
 ****************************************************************************/

void cxd56_txdmasetup(DMA_HANDLE handle, uintptr_t paddr, uintptr_t maddr,
                      size_t nbytes, dma_config_t config);

/****************************************************************************
 * Name: cxd56_dmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 * Assumptions:
 *   - DMA handle allocated by cxd56_dmachannel()
 *   - No DMA in progress
 *
 ****************************************************************************/

void cxd56_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg);

/****************************************************************************
 * Name: cxd56_dmastop
 *
 * Description:
 *  Cancel the DMA.  After cxd56_dmastop() is called, the DMA channel is reset
 *  and cxd56_dmasetup() must be called before cxd56_dmastart() can be called
 *  again
 *
 * Assumptions:
 *  - DMA handle allocated by cxd56_dmachannel()
 *
 ****************************************************************************/

void cxd56_dmastop(DMA_HANDLE handle);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_CXD56XX_CXD56_DMAC_H */
