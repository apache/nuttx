/****************************************************************************
 * arch/arm/src/gd32f4/gd32f4xx_dma.h
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

#ifndef __ARCH_ARM_SRC_GD32F4_GD32F4XX_DMA_H
#define __ARCH_ARM_SRC_GD32F4_GD32F4XX_DMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

#include "chip.h"

#include "hardware/gd32f4xx_dma.h"

/* The DMA transfer status definitions */

#define DMA_STATUS_ERROR           (DMA_INTF_FEEIF | DMA_INTF_SDEIF | DMA_INTF_TAEIF)
#define DMA_STATUS_SUCCESS         (DMA_INTF_HTFIF | DMA_INTF_FTFIF)

#define DMA_INTF_MASK              (0x3f)

#define DMA_INT_MASK               (DMA_CHXCTL_SDEIE | DMA_CHXCTL_TAEIE | DMA_CHXCTL_HTFIE | DMA_CHXCTL_FTFIE)

/* The DMA controllers */

#define GD32_DMA0                  (GD32_DMA0_BASE)
#define GD32_DMA1                  (GD32_DMA1_BASE)

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
 *   arg    - A user-provided value that was provided when gd32_dma_start()
 *            was called.
 */

typedef void (*dma_callback_t)(DMA_HANDLE handle, uint16_t status,
              void *arg);

#ifdef CONFIG_DEBUG_DMA_INFO
struct gd32_dmaregs_s
{
  uint32_t intf0;
  uint32_t intf1;
  uint32_t chctl;
  uint32_t chcnt;
  uint32_t chpaddr;
  uint32_t chm0addr;
  uint32_t chm1addr;
  uint32_t chfctl;
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* DMA singledata mode initialize struct */

typedef struct
{
  uint32_t periph_addr;            /* Peripheral base address */
  uint32_t memory0_addr;           /* Memory 0 base address */
  uint16_t number;                 /* Channel transfer number */
  uint8_t periph_inc;              /* Peripheral increasing mode */
  uint8_t memory_inc;              /* Memory increasing mode */
  uint8_t periph_memory_width;     /* Transfer data size of peripheral */
  uint8_t circular_mode;           /* DMA circular mode */
  uint8_t direction;               /* Channel data transfer direction */
  uint8_t priority;                /* Channel priority level */
} dma_single_data_parameter_struct;

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
 * Name: gd32_dma_channel_alloc
 *
 * Description:
 *   Allocate a DMA channel.  This function gives the caller mutually
 *   exclusive access to the DMA channel specified by the 'dmamap' argument.
 *   Should note that a DMA channel only can be used by on peripheral at the
 *   time.
 *
 *   If the DMA channel is not available, then gd32_dma_channel_alloc() will
 *   wait until the holder of the channel relinquishes the channel by calling
 *   gd32_dma_channel_free().
 *
 * Input Parameters:
 *   periph_req - Identifies the DMA channle is request by which peripheral
 *
 * Returned Value:
 *   If periph_req is valid, this function ALWAYS returns a non-NULL
 *   void* DMA channel handle.
 *
 ****************************************************************************/

DMA_HANDLE gd32_dma_channel_alloc(uint8_t periph_req);

/****************************************************************************
 * Name: gd32_dma_channel_free
 *
 * Description:
 *   Release a DMA channel.  If another thread is waiting for this DMA
 *   channel in a call to gd32_dma_channel_alloc, then this function will
 *   re-assign the DMA channel to that thread and wake it up.
 *   NOTE:  The 'handle' used in this argument must NEVER be used again
 *   until gd32_dma_channel_alloc() is called again to re-gain access to
 *   the channel.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   - The caller holds the DMA channel.
 *   - There is no DMA in progress
 *
 ****************************************************************************/

void gd32_dma_channel_free(DMA_HANDLE handle);

/****************************************************************************
 * Name: gd32_dma_setup
 *
 * Description:
 *   Configure DMA before using
 *
 ****************************************************************************/

void gd32_dma_setup(DMA_HANDLE handle, void *arg, uint8_t data_mode);

/****************************************************************************
 * Name: gd32_dma_start
 *
 * Description:
 *   Start the DMA transfer
 *
 * Assumptions:
 *   - DMA handle allocated by gd32_dma_channel_alloc()
 *   - No DMA in progress
 *
 ****************************************************************************/

void gd32_dma_start(DMA_HANDLE handle, dma_callback_t callback, void *arg,
                    uint32_t interrupt);

/****************************************************************************
 * Name: gd32_dma_stop
 *
 * Description:
 *   Cancel the DMA.  After gd32_dma_stop() is called, the DMA channel is
 *   reset and gd32_dma_setup() must be called before gd32_dma_start()
 *   can be called again
 *
 * Assumptions:
 *   - DMA handle allocated by gd32_dma_channel_alloc()
 *
 ****************************************************************************/

void gd32_dma_stop(DMA_HANDLE handle);

/****************************************************************************
 * Name: gd32_dma_tansnum_get
 *
 * Description:
 *   Get the number of remaining data to be transferred by the DMA
 *
 * Assumptions:
 *   - DMA handle allocated by gd32_dma_channel_alloc()
 *
 ****************************************************************************/

size_t gd32_dma_tansnum_get(DMA_HANDLE handle);

/****************************************************************************
 * Name: gd32_dma_sample
 *
 * Description:
 *   Sample DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by gd32_dma_channel_alloc()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA_INFO
void gd32_dma_sample(DMA_HANDLE handle, struct gd32_dmaregs_s *regs);
#else
#  define gd32_dma_sample(handle,regs)
#endif

/****************************************************************************
 * Name: gd32_dma_dump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by gd32_dma_channel_alloc()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA_INFO
void gd32_dma_dump(DMA_HANDLE handle, const struct gd32_dmaregs_s *regs,
                   const char *msg);
#else
#  define gd32_dma_dump(handle,regs,msg)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_GD32F4_GD32F4XX_DMA_H */
