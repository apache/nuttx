/****************************************************************************
 * arch/arm64/src/imx9/imx9_dma_alloc.h
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

#ifndef __ARCH_ARM64_SRC_IMX9_IMX9_DMA_ALLOC_H
#define __ARCH_ARM64_SRC_IMX9_IMX9_DMA_ALLOC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: imx9_dma_alloc_init
 *
 * Description:
 *   Initialize the DMA memory allocator.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int imx9_dma_alloc_init(void);

/****************************************************************************
 * Name: imx9_dma_alloc
 *
 * Description:
 *   Allocate a contiguous block of physical memory for DMA.
 *
 * Input Parameters:
 *   size - Size of the requested block in bytes.
 *
 * Returned Value:
 *   Physical address of the first page on success; NULL on failure.
 *
 ****************************************************************************/

void *imx9_dma_alloc(size_t size);

/****************************************************************************
 * Name: imx9_dma_free
 *
 * Description:
 *   Free a previously allocated DMA memory block.
 *
 * Input Parameters:
 *   memory - Physical address of the first page of DMA memory.
 *   size   - Size of the allocated block in bytes.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void imx9_dma_free(void *memory, size_t size);

#endif /* __ARCH_ARM64_SRC_IMX9_IMX9_DMA_ALLOC_H */
