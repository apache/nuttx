/****************************************************************************
 * arch/arm/src/rm57/rm57_dma.h
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

#ifndef __ARCH_ARM_SRC_RM57_RM57_DMA_H
#define __ARCH_ARM_SRC_RM57_RM57_DMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Bit-encoded 'status' values passed to the DMA callback (dma_callback_t).
 * These mirror the DMA controller's per-channel interrupt types that this
 * driver makes use of (TRM 20.2.9): HBC ("first half of block complete")
 * and BTC ("block transfer complete"). FTC/LFS are not used by this
 * driver (they would fire once per element/frame, i.e. once per byte for
 * the serial RX/TX use case) and have no status bit here.
 */

#define RM57_DMA_STATUS_HBC          (1 << 0) /* Half block complete */
#define RM57_DMA_STATUS_BTC          (1 << 1) /* Block transfer complete */

#define RM57_DMA_STATUS_SUCCESS      (RM57_DMA_STATUS_HBC | RM57_DMA_STATUS_BTC)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* DMA_HANDLE provides an opaque reference to an allocated DMA channel. */

typedef void *DMA_HANDLE;

/* Channel configuration passed to rm57_dmasetup(). 'chctrl' should be
 * built from the DMA_CHCTRL_* bit definitions in hardware/rm57_dma.h
 * (element sizes, addressing modes, transfer type, auto-initiation).
 */

struct rm57_dmacfg_s
{
  uint32_t saddr;     /* Initial source address (ISADDR) */
  uint32_t daddr;     /* Initial destination address (IDADDR) */
  uint16_t nframes;   /* Frame transfer count (0-0x1fff) */
  uint16_t nelems;    /* Element transfer count per frame (0-0x1fff) */
  uint32_t chctrl;    /* Channel control word: element sizes, TTYPE,
                       * ADDMR/ADDMW, AIM (see hardware/rm57_dma.h) */
  uint8_t  parassign; /* Port assignment (DMA_PAR_* in hardware/rm57_dma.h) */
};

/* The type of the callback that is used to inform the DMA client of
 * transfer progress/completion.
 *
 * Input Parameters:
 *   handle - Refers to the DMA channel
 *   status - Bit-encoded completion status (RM57_DMA_STATUS_* above)
 *   arg    - A user-provided value that was given to rm57_dmastart()
 */

typedef void (*dma_callback_t)(DMA_HANDLE handle, uint8_t status,
                                void *arg);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#if defined(__cplusplus)
extern "C"
{
#endif

/****************************************************************************
 * Name: rm57_dmachannel
 *
 * Description:
 *   Allocate a DMA channel and assign it to the given hardware DMA
 *   request line (one of the DMA_REQ_* values in hardware/rm57_dma.h, or
 *   any request number 0-47).  This function gives the caller exclusive
 *   access to the returned channel.
 *
 *   The DMA controller (control packet RAM ECC init, DMA_EN, HBCA/BTCA
 *   interrupt attach) is lazily initialized on the first call to this
 *   function - callers do not need a separate init step, and this avoids
 *   an ordering hazard with up_irqinitialize() (which would otherwise
 *   clobber an interrupt enable done any earlier than OS start).
 *
 * Returned Value:
 *   On success, a non-NULL DMA_HANDLE.  NULL is returned if no channel is
 *   currently available.
 *
 ****************************************************************************/

DMA_HANDLE rm57_dmachannel(unsigned int req);

/****************************************************************************
 * Name: rm57_dmafree
 *
 * Description:
 *   Release a DMA channel.  The DMA must be stopped (rm57_dmastop())
 *   before this is called; 'handle' must not be used again afterward.
 *
 ****************************************************************************/

void rm57_dmafree(DMA_HANDLE handle);

/****************************************************************************
 * Name: rm57_dmasetup
 *
 * Description:
 *   Program a channel's primary control packet (source/destination
 *   address, transfer counts, channel control word).  Must be called
 *   before rm57_dmastart().
 *
 ****************************************************************************/

void rm57_dmasetup(DMA_HANDLE handle, const struct rm57_dmacfg_s *cfg);

/****************************************************************************
 * Name: rm57_dmastart
 *
 * Description:
 *   Enable the channel for hardware-request triggering and arm the BTC
 *   (block transfer complete) interrupt, and the HBC (half block
 *   complete) interrupt if 'half' is true, to invoke 'callback'.
 *
 ****************************************************************************/

void rm57_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg,
                    bool half);

/****************************************************************************
 * Name: rm57_dmastop
 *
 * Description:
 *   Disable hardware-request triggering and interrupts for the channel.
 *
 ****************************************************************************/

void rm57_dmastop(DMA_HANDLE handle);

/****************************************************************************
 * Name: rm57_dmaresidual
 *
 * Description:
 *   Return the number of elements remaining to be transferred.  For the
 *   byte-per-element transfers used by this port's serial DMA, this is the
 *   number of bytes remaining.
 *
 *   The count comes from the DMA's live FIFO state while the channel is
 *   being processed and from its working control packet otherwise; the
 *   latter is only refreshed when the channel is arbitrated out, so an
 *   answer can trail the bytes already in memory by an element or two.  It
 *   is therefore usable as a receive ring position, which only has to be
 *   safe against running ahead of the data, but not as a "transfer
 *   finished" test - a completed channel does not reliably reach zero.  A
 *   channel that has been set up but never triggered reports zero.
 *
 ****************************************************************************/

size_t rm57_dmaresidual(DMA_HANDLE handle);

/****************************************************************************
 * Name: rm57_serial_dma_poll
 *
 * Description:
 *   Checks RX DMA buffers for received bytes that have not yet
 *   accumulated to the point where the DMA half/full interrupt has
 *   triggered.  Only meaningful (and only defined, in rm57_serial.c) when
 *   at least one SCI port has CONFIG_RM57_SCIn_RXDMA enabled.  Boards
 *   using RXDMA and needing low-latency reception of short, infrequent
 *   messages should call this periodically (e.g. from a timer).
 *
 ****************************************************************************/

void rm57_serial_dma_poll(void);

#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_RM57_RM57_DMA_H */
