/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_dmac.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_ARM_SRC_RP23XX_RP23XX_DMAC_H
#define __ARCH_ARM_SRC_RP23XX_RP23XX_DMAC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

#include "hardware/rp23xx_dma.h"
#include "hardware/rp23xx_dreq.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RP23XX_DMA_NCHANNELS    16

/* DMA data size ************************************************************/

#define RP23XX_DMA_SIZE_BYTE                    0
#define RP23XX_DMA_SIZE_HALFWORD                1
#define RP23XX_DMA_SIZE_WORD                    2

/* Use this as last item in a DMA control block chain */

#define RP23XX_DMA_CTRL_BLOCK_END                   \
                {                                   \
                  RP23XX_DMA_CTRL_TRIG_IRQ_QUIET,   \
                  0,                                \
                  0,                                \
                  0                                 \
                }

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

/* DMA_HANDLE provides an opaque reference that can be used to represent a
 * DMA channel.
 */

typedef void *DMA_HANDLE;

/* Description:
 *   This is the type of the callback that is used to inform the user of the
 *   completion of the DMA.
 *
 * Input Parameters:
 *   handle - Refers to the DMA channel or stream
 *   status - The completion status (0:no error)
 *   arg    - A user-provided value that was provided when rp23xx_dmastart()
 *            was called.
 */

typedef void (*dma_callback_t)(DMA_HANDLE handle, uint8_t status, void *arg);

/* Type of 'config' argument passed to rp23xx_rxdmasetup() and
 * rp23xx_txdmasetup().
 */

typedef struct
{
  uint8_t dreq;
  uint8_t size;
  uint8_t noincr;
} dma_config_t;

/* Type of items in the array items to 'ctrl_blks' argument for
 * rp23xx_ctrl_dmasetup().
 */

typedef struct
  {
    uint32_t  ctrl_trig;
    uintptr_t read_addr;
    uintptr_t write_addr;
    uint32_t  xfer_count;
  } dma_control_block_t;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: rp23xx_dmachannel
 *
 * Description:
 *   Allocate a DMA channel. This function gives the caller mutually
 *   exclusive access to a DMA channel.
 *
 *   If no DMA channel is available, then rp23xx_dmachannel() will wait until
 *   the holder of a channel relinquishes the channel by calling
 *   rp23xx_dmafree().
 *
 * Input parameters:
 *   None
 *
 * Returned Value:
 *   This function ALWAYS returns a non-NULL, void* DMA channel handle.
 *
 * Assumptions:
 *   - The caller can wait for a DMA channel to be freed if it is
 *     not available.
 *
 ****************************************************************************/

DMA_HANDLE rp23xx_dmachannel(void);

/****************************************************************************
 * Name: rp23xx_dmafree
 *
 * Description:
 *   Release a DMA channel.
 *   If another thread is waiting for this DMA channel in a call to
 *   rp23xx_dmachannel, then this function will re-assign the DMA channel to
 *   that thread and wake it up.
 *   NOTE:
 *   The 'handle' used in this argument must NEVER be used again until
 *   rp23xx_dmachannel() is called again to re-gain access to the channel.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   - The caller holds the DMA channel.
 *   - There is no DMA in progress
 *
 ****************************************************************************/

void rp23xx_dmafree(DMA_HANDLE handle);

/****************************************************************************
 * Name: rp23xx_rxdmasetup
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

void rp23xx_rxdmasetup(DMA_HANDLE handle, uintptr_t paddr, uintptr_t maddr,
                       size_t nbytes, dma_config_t config);

/****************************************************************************
 * Name: rp23xx_txdmasetup
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

void rp23xx_txdmasetup(DMA_HANDLE handle, uintptr_t paddr, uintptr_t maddr,
                       size_t nbytes, dma_config_t config);

/****************************************************************************
 * Name: rp23xx_ctrl_dmasetup
 *
 * Description:
 *   Configure a dma channel to send a list of channel control blocks to
 *   a second dma channel..
 *
 * Input Parameters:
 *   control   - the DMA handle that reads the control blocks.  This is
 *               the one that should be started.
 *   transfer  - the DMA handle the transfers data to the peripheral. No
 *               setup of this handle is required by the caller.
 *   ctrl_blks - the array of control blocks to used.  Terminate this
 *               list with an all zero control block.
 *   callback  - callback when last transfer completes
 *   arg       - arg to pass to callback
 *
 ****************************************************************************/

void rp23xx_ctrl_dmasetup(DMA_HANDLE           control,
                          DMA_HANDLE           transfer,
                          dma_control_block_t *ctrl_blks,
                          dma_callback_t       callback,
                          void                *arg);

/****************************************************************************
 * Name: rp23xx_ctrl_dmasetup
 *
 * Description:
 *   Configure a dma channel to send a list of channel control blocks to
 *   a second dma channel..
 *
 * Input Parameters:
 *   control   - the DMA handle that reads the control blocks.  This is
 *               the one that should be started.
 *   size      - transfer size for this block
 *   pacing    - dma pacing register for this block
 *   ctrl      - Additional bits to set in CTRL_TRIG for this block.
 *
 ****************************************************************************/

uint32_t rp23xx_dma_ctrl_blk_ctrl(DMA_HANDLE  control,
                                  int         size,
                                  uint32_t    pacing,
                                  uint32_t    ctrl);

/****************************************************************************
 * Name: rp23xx_dmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 * Assumptions:
 *   - DMA handle allocated by rp23xx_dmachannel()
 *   - No DMA in progress
 *
 ****************************************************************************/

void rp23xx_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg);

/****************************************************************************
 * Name: rp23xx_dmastop
 *
 * Description:
 *   Cancel the DMA.  After rp23xx_dmastop() is called, the DMA channel is
 *   reset and rp23xx_dmasetup() must be called before rp23xx_dmastart() can
 *   be called again.
 *
 * Assumptions:
 *   - DMA handle allocated by rp23xx_dmachannel()
 *
 ****************************************************************************/

void rp23xx_dmastop(DMA_HANDLE handle);

/****************************************************************************
 * Name: rp23xx_dma_register
 *
 * Description:
 *   Get the address of a DMA register based on the given DMA handle that
 *   can be used in the various putreg, getreg and modifyreg functions.
 *
 *   This allows other configuration options not normally supplied.
 *
 * Assumptions:
 *   - DMA handle allocated by rp23xx_dmachannel()
 *
 ****************************************************************************/

uintptr_t rp23xx_dma_register(DMA_HANDLE handle, uint16_t offset);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_RP23XX_RP23XX_DMAC_H */
