/****************************************************************************
 * arch/arm/src/efm32/efm32_dma.h
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

#ifndef __ARCH_ARM_SRC_EFM32_EFM32_DMA_H
#define __ARCH_ARM_SRC_EFM32_EFM32_DMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include "chip.h"
#include "hardware/efm32_dma.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Bit encoded input parameter to efm32_channel().
 * These encodings must fit in the an unsigned integer of type dma_config_t.
 *
 * Current limitations/assumptions in the encoding:
 *
 *   - RX transfers are peripheral to memory
 *   - TX transfers are memory to peripheral
 *   - Memory address is always incremented.
 */

#define EFM32_DMA_SIGSEL_SHIFT           (0)       /* Bits 0-3: _DMA_CH_CTRL_ * value */
#define EFM32_DMA_SIGSEL_MASK            (15 << EFM32_DMA_SIGSEL_SHIFT)
#  define EFM32_DMA_SIGSEL(n)            ((dma_config_t)(n) << EFM32_DMA_SIGSEL_SHIFT)

#define EFM32_DMA_SOURCSEL_SHIFT         (4)       /* Bits 4-9: _DMA_CH_SOURCESEL_* value */
#define EFM32_DMA_SOURCSEL_MASK          (63 << EFM32_DMA_SOURCSEL_SHIFT)
#  define EFM32_DMA_SOURCSEL(n)          ((dma_config_t)(n) << EFM32_DMA_SOURCSEL_SHIFT)

#define EFM32_DMA_XFERSIZE_SHIFT         (10)      /* Bits 10-11: Transfer size */
#define EFM32_DMA_XFERSIZE_MASK          (3 << EFM32_DMA_XFERSIZE_SHIFT)
#  define EFM32_DMA_XFERSIZE_BYTE        (0 << EFM32_DMA_SOURCSEL_SHIFT)
#  define EFM32_DMA_XFERSIZE_HWORD       (1 << EFM32_DMA_SOURCSEL_SHIFT)
#  define EFM32_DMA_XFERSIZE_WORD        (2 << EFM32_DMA_SOURCSEL_SHIFT)

#define EFM32_DMA_SINGLE_MASK            (1 << 12) /* Bit 12: Single or Buffer full request */
#  define EFM32_DMA_SINGLE               (1 << 12) /*         1=Buffer full request */
#  define EFM32_DMA_BUFFER_FULL          (0)       /*         0=Buffer full request */

#define EFM32_DMA_MEMINCR_MASK           (1 << 13) /* Bit 13: Increment memory address */
#  define EFM32_DMA_MEMINCR              (1 << 13) /*         1=Increment memory address */
#  define EFM32_DMA_NOINCR               (0)       /*         0=No memory address increment */

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
 *   status - A bit encoded value that provides the completion status.  See
 *            the DMASTATUS_* definitions above.
 *   arg    - A user-provided value that was provided when efm32_dmastart()
 *            was called.
 */

typedef void (*dma_callback_t)(DMA_HANDLE handle, uint8_t status, void *arg);

#ifdef CONFIG_DEBUG_DMA_INFO
struct efm32_dmaregs_s
{
  uint32_t status;            /* DMA Status Register */
  uint32_t ctrlbase;          /* Channel Control Data Base Pointer Register */
  uint32_t altctrlbase;       /* Channel Alternate Control Data Base Pointer Register */
  uint32_t chwaitstatus;      /* Channel Wait on Request Status Register */
  uint32_t chusebursts;       /* Channel Useburst Set Register */
  uint32_t chreqmasks;        /* Channel Request Mask Set Register */
  uint32_t chens;             /* Channel Enable Set Register */
  uint32_t chalts;            /* Channel Alternate Set Register */
  uint32_t chpris;            /* Channel Priority Set Register */
  uint32_t errorc;            /* Bus Error Clear Register */
  uint32_t chreqstatus;       /* Channel Request Status */
  uint32_t chsreqstatus;      /* Channel Single Request Status */
  uint32_t ifr;               /* Interrupt Flag Register */
  uint32_t ien;               /* Interrupt Enable register */
#if defined(CONFIG_EFM32_EFM32GG)
  uint32_t ctrl;              /* DMA Control Register */
  uint32_t rds;               /* DMA Retain Descriptor State */
  uint32_t loop0;             /* Channel 0 Loop Register */
  uint32_t loop1;             /* Channel 1 Loop Register */
  uint32_t rect0;             /* Channel 0 Rectangle Register */
#endif
  uint32_t chnctrl;           /* Channel n Control Register */
};
#endif

/* Type of 'config' argument passed to efm32_rxdmasetup() and
 * efm32_txdmasetup. See EFM32_DMA_* encodings above.
 * If these encodings exceed 16-bits, then this should be changed to a
 * uint32_t.
 */

typedef uint16_t dma_config_t;

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
 * Name: efm32_dmachannel
 *
 * Description:
 *   Allocate a DMA channel.
 *  This function gives the caller mutually exclusive access to a DMA
 *  channel.
 *
 *   If no DMA channel is available, then efm32_dmachannel() will wait until
 *   the holder of a channel relinquishes the channel by calling
 *   efm32_dmafree().
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   This function ALWAYS returns a non-NULL, void* DMA channel handle.
 *
 * Assumptions:
 *   - The caller can wait for a DMA channel to be freed if it is not
 *     available.
 *
 ****************************************************************************/

DMA_HANDLE efm32_dmachannel(void);

/****************************************************************************
 * Name: efm32_dmafree
 *
 * Description:
 *   Release a DMA channel.
 *   If another thread is waiting for this DMA channel in a call to
 *   efm32_dmachannel, then this function will re-assign the DMA channel to
 *   that thread and wake it up.  NOTE:  The 'handle' used in this argument
 *   must NEVER be used again until efm32_dmachannel() is called again to
 *   re-gain access to the channel.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   - The caller holds the DMA channel.
 *   - There is no DMA in progress
 *
 ****************************************************************************/

void efm32_dmafree(DMA_HANDLE handle);

/****************************************************************************
 * Name: efm32_rxdmasetup
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

void efm32_rxdmasetup(DMA_HANDLE handle, uintptr_t paddr, uintptr_t maddr,
                      size_t nbytes, dma_config_t config);

/****************************************************************************
 * Name: efm32_txdmasetup
 *
 * Description:
 *   Configure an TX (memory-to-memory) DMA before starting the transfer.
 *
 * Input Parameters:
 *   paddr  - Peripheral address (destination)
 *   maddr  - Memory address (source)
 *   nbytes - Number of bytes to transfer.  Must be an even multiple of the
 *            configured transfer size.
 *   config - Channel configuration selections
 *
 ****************************************************************************/

void efm32_txdmasetup(DMA_HANDLE handle, uintptr_t paddr, uintptr_t maddr,
                      size_t nbytes, dma_config_t config);

/****************************************************************************
 * Name: efm32_dmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 * Assumptions:
 *   - DMA handle allocated by efm32_dmachannel()
 *   - No DMA in progress
 *
 ****************************************************************************/

void efm32_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg);

/****************************************************************************
 * Name: efm32_dmastop
 *
 * Description:
 *   Cancel the DMA.
 *   After efm32_dmastop() is called, the DMA channel is reset and
 *   efm32_dmasetup() must be called before efm32_dmastart() can be called
 *   again
 *
 * Assumptions:
 *   - DMA handle allocated by efm32_dmachannel()
 *
 ****************************************************************************/

void efm32_dmastop(DMA_HANDLE handle);

/****************************************************************************
 * Name: efm32_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by efm32_dmachannel()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA_INFO
void efm32_dmasample(DMA_HANDLE handle, struct efm32_dmaregs_s *regs);
#else
#  define efm32_dmasample(handle,regs)
#endif

/****************************************************************************
 * Name: efm32_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by efm32_dmachannel()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA_INFO
void efm32_dmadump(DMA_HANDLE handle, const struct efm32_dmaregs_s *regs,
                   const char *msg);
#else
#  define efm32_dmadump(handle,regs,msg)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_EFM32_EFM32_DMA_H */
