/****************************************************************************
 * arch/arm/src/n32h7/n32_dma.h
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

#ifndef __ARCH_ARM_SRC_N32H7_N32_DMA_H
#define __ARCH_ARM_SRC_N32H7_N32_DMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <stdint.h>

#include "hardware/n32h7_dma.h"
#include "hardware/n32h7_dmamux.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

#define DMA_INT_TFR   (1 << 0)
#define DMA_INT_BLOCK (1 << 1)
#define DMA_INT_SRCT  (1 << 2)
#define DMA_INT_DSTT  (1 << 3)
#define DMA_INT_ERROR (1 << 4)

/* DMA Channel Initialize configuration structure */

typedef struct
{
  uint32_t src_addr;      /* Source address */
  uint32_t dst_addr;      /* Destination address */
  uint32_t block_size;    /* Block transfer size (max 4095) */
  uint64_t ctrl;          /* CH_CTRL value (combine macros from n32h7_dma.h) */
  uint32_t sg_cfg;        /* CH_SG (source gather), 0 if unused */
  uint32_t ds_cfg;        /* CH_DS (dest scatter), 0 if unused */

  uint8_t  src_hs_if;     /* Source handshake interface (0~7) -> SRCPER */
  uint8_t  dst_hs_if;     /* Dest handshake interface (0~7) -> DSTPER */
  uint8_t  priority;      /* Channel priority (0~7) -> CHPRIOR */

  uint8_t  src_hs_mode;   /* 0: HW handshake, 1: SW handshake -> HSSELSRC */
  uint8_t  dst_hs_mode;   /* 0: HW, 1: SW -> HSSELDST */

  void     *link_list;    /* Linked list pointer (NULL if not used) */
} n32_dmacfg_t;

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
 *   arg    - A user-provided value that was provided when n32_dmastart()
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
 * Name: n32_dmachannel
 *
 * Description:
 *   Allocate a DMA channel.  This function gives the caller mutually
 *   exclusive access to the DMA channel specified by the 'dmamap' argument.
 *   It is common for standard DMA (DMA1, DMA2, DMA3), master DMA (MDMA)
 *   controllers.
 *
 * Input Parameters:
 *   dmamap - Identifies the stream/channel resource. For the N32 H7, this
 *     is a bit-encoded  value as provided by the DMAMAP_* definitions
 *     in chip/n32h7xxxxxxx_dmamux.h
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

DMA_HANDLE n32_dmachannel(unsigned int dmamap);

/****************************************************************************
 * Name: n32_dmafree
 *
 * Description:
 *   Release a DMA channel and unmap DMAMUX if required.
 *
 *   NOTE:  The 'handle' used in this argument must NEVER be used again
 *   until n32_dmachannel() is called again to re-gain access to the
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

void n32_dmafree(DMA_HANDLE handle);

/****************************************************************************
 * Name: n32_dmasetup
 *
 * Description:
 *   Configure DMA before using it.
 *
 * Input Parameters:
 *   handle - DMA channel handle
 *   cfg    - Pointer to DMA configuration structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void n32_dmasetup(DMA_HANDLE handle, n32_dmacfg_t *cfg);

/****************************************************************************
 * Name: n32_dmastart
 *
 * Description:
 *   Start DMA transfer.
 *
 * Input Parameters:
 *   handle - DMA channel handle
 *   callback - Pointer to DMA completion callback
 *   arg      - User-provided value to pass to the callback
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void n32_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg);

/****************************************************************************
 * Name: n32_dmastop
 *
 * Description:
 *   Stop DMA transfer.
 *
 * Input Parameters:
 *   handle - DMA channel handle
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void n32_dmastop(DMA_HANDLE handle);

/****************************************************************************
 * Name: n32_getchanel
 *
 * Description:
 *   Get the current DMA channel.
 *
 * Input Parameters:
 *   handle - DMA channel handle
 *
 * Returned Value:
 *   DMA channel number
 *
 * Assumptions:
 *   - The caller holds the DMA channel.
 *
 ****************************************************************************/

size_t n32_getchanel(DMA_HANDLE handle);

/****************************************************************************
 * Name: n32_dmaptr_src
 *
 * Description:
 *   Get the source DMA pointer.
 *
 * Input Parameters:
 *   handle - DMA channel handle
 *
 * Returned Value:
 *   Source DMA pointer
 *
 * Assumptions:
 *   - The caller holds the DMA channel.
 *
 ****************************************************************************/

uint32_t n32_dmaptr_src(DMA_HANDLE handle);

/****************************************************************************
 * Name: n32_dmaptr_dst
 *
 * Description:
 *   Get the destination DMA pointer.
 *
 * Input Parameters:
 *   handle - DMA channel handle
 *
 * Returned Value:
 *   Destination DMA pointer
 *
 * Assumptions:
 *   - The caller holds the DMA channel.
 *
 ****************************************************************************/

uint32_t n32_dmaptr_dst(DMA_HANDLE handle);

/****************************************************************************
 * Name: n32_getcount
 *
 * Description:
 *   Get the number of bytes to be transferred.
 *
 * Input Parameters:
 *   handle - DMA channel handle
 *
 * Returned Value:
 *   Number of bytes to be transferred
 *
 * Assumptions:
 *   - The caller holds the DMA channel.
 *
 ****************************************************************************/

uint32_t n32_getcount(DMA_HANDLE handle);

/****************************************************************************
 * Name: n32_dmacapable
 *
 * Description:
 *   Check if the DMA channel is capable of the transfer.
 *
 * Input Parameters:
 *   handle - DMA channel handle
 *   cfg    - Pointer to DMA configuration structure
 *
 * Returned Value:
 *   True if the DMA channel is capable of the transfer, False otherwise
 *
 * Assumptions:
 *   - The caller holds the DMA channel.
 *
 ****************************************************************************/

#ifdef CONFIG_N32H7_DMACAPABLE
bool n32_dmacapable(DMA_HANDLE handle, n32_dmacfg_t *cfg);
#else
#  define n32_dmacapable(handle, cfg) (true)
#endif

/****************************************************************************
 * Name: n32_dmadump
 *
 * Description:
 *   Dump the DMA channel configuration.
 *
 * Input Parameters:
 *   handle - DMA channel handle
 *   msg    - Optional message to print before the dump
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   - The caller holds the DMA channel.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA_INFO
void n32_dmadump(DMA_HANDLE handle, const char *msg);
#else
#  define n32_dmadump(handle,msg)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_N32H7_N32_DMA_H */
