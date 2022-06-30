/****************************************************************************
 * arch/arm/src/rp2040/rp2040_dmac.h
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

#ifndef __ARCH_ARM_SRC_RP2040_RP2040_DMAC_H
#define __ARCH_ARM_SRC_RP2040_RP2040_DMAC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

#include "hardware/rp2040_dma.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RP2040_DMA_NCHANNELS    12

/* DREQ channels ************************************************************/

#define RP2040_DMA_DREQ_PIO0_TX0                0
#define RP2040_DMA_DREQ_PIO0_TX1                1
#define RP2040_DMA_DREQ_PIO0_TX2                2
#define RP2040_DMA_DREQ_PIO0_TX3                3
#define RP2040_DMA_DREQ_PIO0_RX0                4
#define RP2040_DMA_DREQ_PIO0_RX1                5
#define RP2040_DMA_DREQ_PIO0_RX2                6
#define RP2040_DMA_DREQ_PIO0_RX3                7
#define RP2040_DMA_DREQ_PIO1_TX0                8
#define RP2040_DMA_DREQ_PIO1_TX1                9
#define RP2040_DMA_DREQ_PIO1_TX2                10
#define RP2040_DMA_DREQ_PIO1_TX3                11
#define RP2040_DMA_DREQ_PIO1_RX0                12
#define RP2040_DMA_DREQ_PIO1_RX1                13
#define RP2040_DMA_DREQ_PIO1_RX2                14
#define RP2040_DMA_DREQ_PIO1_RX3                15
#define RP2040_DMA_DREQ_SPI0_TX                 16
#define RP2040_DMA_DREQ_SPI0_RX                 17
#define RP2040_DMA_DREQ_SPI1_TX                 18
#define RP2040_DMA_DREQ_SPI1_RX                 19
#define RP2040_DMA_DREQ_UART0_TX                20
#define RP2040_DMA_DREQ_UART0_RX                21
#define RP2040_DMA_DREQ_UART1_TX                22
#define RP2040_DMA_DREQ_UART1_RX                23
#define RP2040_DMA_DREQ_PWM_WRAP0               24
#define RP2040_DMA_DREQ_PWM_WRAP1               25
#define RP2040_DMA_DREQ_PWM_WRAP2               26
#define RP2040_DMA_DREQ_PWM_WRAP3               27
#define RP2040_DMA_DREQ_PWM_WRAP4               28
#define RP2040_DMA_DREQ_PWM_WRAP5               29
#define RP2040_DMA_DREQ_PWM_WRAP6               30
#define RP2040_DMA_DREQ_PWM_WRAP7               31
#define RP2040_DMA_DREQ_I2C0_TX                 32
#define RP2040_DMA_DREQ_I2C0_RX                 33
#define RP2040_DMA_DREQ_I2C1_TX                 34
#define RP2040_DMA_DREQ_I2C1_RX                 35
#define RP2040_DMA_DREQ_ADC                     36
#define RP2040_DMA_DREQ_XIP_STREAM              37
#define RP2040_DMA_DREQ_XIP_SSITX               38
#define RP2040_DMA_DREQ_XIP_SSIRX               39

/* DMA data size ************************************************************/

#define RP2040_DMA_SIZE_BYTE                    0
#define RP2040_DMA_SIZE_HALFWORD                1
#define RP2040_DMA_SIZE_WORD                    2

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
 *   arg    - A user-provided value that was provided when rp2040_dmastart()
 *            was called.
 */

typedef void (*dma_callback_t)(DMA_HANDLE handle, uint8_t status, void *arg);

/* Type of 'config' argument passed to rp2040_rxdmasetup() and
 * rp2040_txdmasetup().
 */

typedef struct
{
  uint8_t dreq;
  uint8_t size;
  uint8_t noincr;
} dma_config_t;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: rp2040_dmachannel
 *
 * Description:
 *   Allocate a DMA channel. This function gives the caller mutually
 *   exclusive access to a DMA channel.
 *
 *   If no DMA channel is available, then rp2040_dmachannel() will wait until
 *   the holder of a channel relinquishes the channel by calling
 *   rp2040_dmafree().
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

DMA_HANDLE rp2040_dmachannel(void);

/****************************************************************************
 * Name: rp2040_dmafree
 *
 * Description:
 *   Release a DMA channel.
 *   If another thread is waiting for this DMA channel in a call to
 *   rp2040_dmachannel, then this function will re-assign the DMA channel to
 *   that thread and wake it up.
 *   NOTE:
 *   The 'handle' used in this argument must NEVER be used again until
 *   rp2040_dmachannel() is called again to re-gain access to the channel.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   - The caller holds the DMA channel.
 *   - There is no DMA in progress
 *
 ****************************************************************************/

void rp2040_dmafree(DMA_HANDLE handle);

/****************************************************************************
 * Name: rp2040_rxdmasetup
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

void rp2040_rxdmasetup(DMA_HANDLE handle, uintptr_t paddr, uintptr_t maddr,
                       size_t nbytes, dma_config_t config);

/****************************************************************************
 * Name: rp2040_txdmasetup
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

void rp2040_txdmasetup(DMA_HANDLE handle, uintptr_t paddr, uintptr_t maddr,
                       size_t nbytes, dma_config_t config);

/****************************************************************************
 * Name: rp2040_dmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 * Assumptions:
 *   - DMA handle allocated by rp2040_dmachannel()
 *   - No DMA in progress
 *
 ****************************************************************************/

void rp2040_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg);

/****************************************************************************
 * Name: rp2040_dmastop
 *
 * Description:
 *   Cancel the DMA.  After rp2040_dmastop() is called, the DMA channel is
 *   reset and rp2040_dmasetup() must be called before rp2040_dmastart() can
 *   be called again.
 *
 * Assumptions:
 *   - DMA handle allocated by rp2040_dmachannel()
 *
 ****************************************************************************/

void rp2040_dmastop(DMA_HANDLE handle);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_RP2040_RP2040_DMAC_H */
