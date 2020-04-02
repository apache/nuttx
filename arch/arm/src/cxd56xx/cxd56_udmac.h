/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_udmac.h
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_CXD56XX_CXD56_UDMAC_H
#define __ARCH_ARM_SRC_CXD56XX_CXD56_UDMAC_H

#include <stdint.h>

#include "cxd56_dmac_common.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Bit encoded input parameter to cxd56_channel(). These encodings must fit
 * in an unsigned integer of type dma_config_t.
 *
 * Current limitations/assumptions in the encoding:
 *
 *   - RX transfers are peripheral to memory
 *   - TX transfers are memory to peripheral
 *   - Memory address is always incremented.
 */

#define CXD56_UDMA_XFERSIZE_SHIFT         (10)      /* Bits 10-11: Transfer size */
#define CXD56_UDMA_XFERSIZE_MASK          (3 << CXD56_UDMA_XFERSIZE_SHIFT)
#define CXD56_UDMA_XFERSIZE_BYTE        (0 << CXD56_UDMA_XFERSIZE_SHIFT)
#define CXD56_UDMA_XFERSIZE_HWORD       (1 << CXD56_UDMA_XFERSIZE_SHIFT)
#define CXD56_UDMA_XFERSIZE_WORD        (2 << CXD56_UDMA_XFERSIZE_SHIFT)

#define CXD56_UDMA_SINGLE_MASK            (1 << 12) /* Bit 12: Single or Buffer full request */
#define CXD56_UDMA_SINGLE               (1 << 12) /*         1=Buffer full request */
#define CXD56_UDMA_BUFFER_FULL          (0)       /*         0=Buffer full request */

#define CXD56_UDMA_MEMINCR_MASK           (1 << 13) /* Bit 13: Increment memory address */
#define CXD56_UDMA_MEMINCR              (1 << 13) /*         1=Increment memory address */
#define CXD56_UDMA_NOINCR               (0)       /*         0=No memory address increment */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
struct cxd56_udmaregs_s
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
  uint32_t done;              /* Done status (CXD5602 only) */
  uint32_t err;               /* Error status (CXD5602 only) */

  uint32_t srcend;
  uint32_t dstend;
  uint32_t ctrl;
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
 * Name: cxd56_udmachannel
 *
 * Description:
 *   Allocate a DMA channel. This function gives the caller mutually
 *   exclusive access to a DMA channel.
 *
 *   If no DMA channel is available, then cxd56_udmachannel() will wait until
 *   the holder of a channel relinquishes the channel by calling
 *   cxd56_udmafree().
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

DMA_HANDLE cxd56_udmachannel(void);

/****************************************************************************
 * Name: cxd56_udmafree
 *
 * Description:
 *   Release a DMA channel.  If another thread is waiting for this DMA
 *   channel in a call to cxd56_udmachannel, then this function will re-assign
 *   the DMA channel to that thread and wake it up.
 *   NOTE:  The 'handle' used in this argument must NEVER be used again until
 *   cxd56_udmachannel() is called again to re-gain access to the channel.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   - The caller holds the DMA channel.
 *   - There is no DMA in progress
 *
 ****************************************************************************/

void cxd56_udmafree(DMA_HANDLE handle);

/****************************************************************************
 * Name: cxd56_rxudmasetup
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

void cxd56_rxudmasetup(DMA_HANDLE handle, uintptr_t paddr, uintptr_t maddr,
                       size_t nbytes, dma_config_t config);

/****************************************************************************
 * Name: cxd56_txudmasetup
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

void cxd56_txudmasetup(DMA_HANDLE handle, uintptr_t paddr, uintptr_t maddr,
                       size_t nbytes, dma_config_t config);

/****************************************************************************
 * Name: cxd56_udmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 * Assumptions:
 *   - DMA handle allocated by cxd56_udmachannel()
 *   - No DMA in progress
 *
 ****************************************************************************/

void cxd56_udmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg);

/****************************************************************************
 * Name: cxd56_udmastop
 *
 * Description:
 *   Cancel the DMA.  After cxd56_udmastop() is called, the DMA channel is
 *   reset and cxd56_udmasetup() must be called before cxd56_udmastart() can
 *   be called again.
 *
 * Assumptions:
 *   - DMA handle allocated by cxd56_udmachannel()
 *
 ****************************************************************************/

void cxd56_udmastop(DMA_HANDLE handle);

/****************************************************************************
 * Name: cxd56_udmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by cxd56_udmachannel()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void cxd56_udmasample(DMA_HANDLE handle, struct cxd56_udmaregs_s *regs);
#else
#define cxd56_udmasample(handle,regs)
#endif

/****************************************************************************
 * Name: cxd56_udmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by cxd56_udmachannel()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void cxd56_udmadump(DMA_HANDLE handle, const struct cxd56_udmaregs_s *regs,
                   const char *msg);
#else
#define cxd56_udmadump(handle,regs,msg)
#endif

void cxd56_udmainitialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_CXD56XX_CXD56_UDMAC_H */
