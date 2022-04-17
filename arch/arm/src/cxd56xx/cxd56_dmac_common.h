/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_dmac_common.h
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

#ifndef __ARCH_ARM_SRC_CXD56XX_CXD56_DMAC_COMMON_H
#define __ARCH_ARM_SRC_CXD56XX_CXD56_DMAC_COMMON_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

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
 *   status - A bit encoded value that provides the completion status.
 *            See the DMASTATUS_* definitions above.
 *   arg    - A user-provided value that was provided when cxd56_dmastart()
 *            was called.
 */

typedef void (*dma_callback_t)(DMA_HANDLE handle, uint8_t status, void *arg);

/* Type of 'config' argument passed to cxd56_rxdmasetup() and
 * cxd56_txdmasetup.
 * See CXD56_DMA_* encodings above.  If these encodings exceed 16-bits, then
 * this should be changed to a uint32_t.
 */

typedef struct
{
    uint16_t channel_cfg;
    uint8_t dest_width;
    uint8_t src_width;
} dma_config_t;

#endif /* __ARCH_ARM_SRC_CXD56XX_CXD56_DMAC_COMMON_H */
