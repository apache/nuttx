/****************************************************************************
 * arch/arm/src/stm32h5/stm32_dma.h
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

#ifndef __ARCH_ARM_SRC_STM32H5_STM32_DMA_H
#define __ARCH_ARM_SRC_STM32H5_STM32_DMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

#include <stdint.h>

#include "hardware/stm32_gpdma.h"

#ifdef CONFIG_DEBUG_DMA_INFO
#  error "CONFIG_DEBUG_DMA_INFO not yet implemented."
#  undef CONFIG_DEBUG_DMA_INFO
#endif

#ifdef CONFIG_STM32H5_DMACAPABLE
#  error "CONFIG_STM32H5_DMACAPABLE not yet implemented."
#  undef CONFIG_STM32H5_DMACAPABLE
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* These definitions provide the bit encoding of the 'status' parameter
 * passed to the DMA callback function (see dma_callback_t).
 */

#define DMA_STATUS_TCF        (1 << 0) /* Transfer Complete */
#define DMA_STATUS_HTF        (1 << 1) /* Half Transfer */
#define DMA_STATUS_DTEF       (1 << 2) /* Data transfer error */
#define DMA_STATUS_ULEF       (1 << 3) /* Update link transfer error */
#define DMA_STATUS_USEF       (1 << 4) /* User setting error */
#define DMA_STATUS_SUSPF      (1 << 5) /* Completed suspension flag */
#define DMA_STATUS_TOF        (1 << 6) /* Trigger overrun flag */

#define DMA_STATUS_ERROR      (DMA_STATUS_DTEF | DMA_STATUS_ULEF | DMA_STATUS_USEF | DMA_STATUS_TOF)
#define DMA_STATUS_SUCCESS    (DMA_STATUS_TCF | DMA_STATUS_HTEF)

/* GPDMA Mode Flags: WARNING!! NOT YET IMPLEMENTED! */

#define GPDMACFG_MODE_CIRC    (1 << 0)  /* Enable Circular mode */
#define GPDMACFG_MODE_PFC     (1 << 1)  /* Enable Peripheral flow control */
#define GPDMACFG_MODE_DB      (1 << 2)  /* Enable Double buffer mode */

/* Channel priority level
 * Refer to PRIO field in GPDMA_CxCR register description (RM0481)
 */

#define GPDMACFG_PRIO_LL      (0)   /* Low priority, low weight */
#define GPDMACFG_PRIO_LM      (1)   /* Low priority, mid weight */
#define GPMDACFG_PRIO_LH      (2)   /* Low priority, high weight */
#define GPDMACFG_PRIO_H       (3)   /* High priority */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* GPDMA transfer type enumeration */

enum gpdma_ttype_e
{
  /* Peripheral-to-memory transfer */

  GPDMA_TTYPE_P2M = 0,

  /* Memory-to-peripheral transfer */

  GPDMA_TTYPE_M2P,

  /* Memory-to-memory transfer, linear addressing */

  GPDMA_TTYPE_M2M_LINEAR,

  /* 2D Addressing needed (NOT IMPLEMENTED YET) */

  GPDMA_TTYPE_2D
};

#ifdef CONFIG_DEBUG_DMA_INFO
struct stm32_gpdma_reg_s
{
  uint32_t cxlbar;
  uint32_t cxfcr;
  uint32_t cxsr;
  uint32_t cxcr;
  uint32_t cxtr1;
  uint32_t cxtr2;
  uint32_t cxbr1;
  uint32_t cxsar;
  uint32_t cxdar;
  uint32_t cxtr3;
  uint32_t cxbr2;
  uint32_t cxllr;
};
#endif

struct stm32_gpdma_cfg_s
{
  uint32_t src_addr;
  uint32_t dest_addr;

  /* CxTR1 register for specified channel. */

  uint32_t tr1;

  /* request: Accepts GPDMA_CXTR2_SWREQ, GPDMA_CXTR2_DREQ, and
   * GPDMA_CXTR2_REQSEL(r) for r given by GPDMA_REQ_x
   * macros defined in hardware/stm32h56x_dmasigmap.h
   */

  uint16_t request;

  /* Number of transfers, in units of the data width
   * specified in tr1.
   */

  uint16_t ntransfers;

  /* Priority level: refer to GPDMACFG_PRIO defines above */

  uint8_t  priority;

  /* mode flags, refer to GPDMACFG_MODE_X defines above. */

  uint8_t  mode;
};

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
 *            the DMA_STATUS_* definitions above.
 *   arg    - A user-provided value that was provided when stm32_dmastart()
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
 * Name: stm32_dmachannel
 *
 * Description:
 *   Allocate a DMA channel based on provided transfer type. The driver will
 *   return the first available DMA channel compatible with the transfer type
 *
 * Input Parameters:
 *   type - Type of DMA transfer required
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

DMA_HANDLE stm32_dmachannel(enum gpdma_ttype_e type);

/****************************************************************************
 * Name: stm32_dmafree
 *
 * Description:
 *   Release a DMA channel.
 *
 *   NOTE:  The 'handle' used in this argument must NEVER be used again
 *   until stm32_dmachannel() is called again to re-gain access to the
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

void stm32_dmafree(DMA_HANDLE handle);

/****************************************************************************
 * Name: stm32_dmasetup
 *
 * Description:
 *   Configure DMA before using
 *
 * Input Parameters:
 *   TODO: Figure out what the input parameter needs to be!!! H7 and F7 have
 *         very different implementations.
 *
 ****************************************************************************/

void stm32_dmasetup(DMA_HANDLE handle, struct stm32_gpdma_cfg_s *cfg);

/****************************************************************************
 * Name: stm32_dmastart
 *
 * Description:
 *   Start the DMA transfer.  NOTE:  The DMA module does *NOT* perform any
 *   cache operations.  It is the responsibility of the DMA client to clean
 *   DMA buffers after starting of the DMA TX operations.
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

#ifdef CONFIG_STM32H5_DMACAPABLE
bool stm32_dmacapable(DMA_HANDLE handle, struct stm32_gpdma_cfg_s *cfg);
#else
#  define stm32_dmacapable(handle, cfg) (true)
#endif

/****************************************************************************
 * Name: stm32_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 * Input Parameters:
 *   TODO: Figure these out!! Not sure if I need the struct like F7 or not?
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
void stm32_dmadump(DMA_HANDLE handle, const char *msg);
#else
#  define stm32_dmadump(handle,msg)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* !__ASSEMBLY__*/
#endif /* __ARCH_ARM_SRC_STM32H5_STM32_DMA_H*/
