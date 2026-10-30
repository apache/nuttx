/****************************************************************************
 * include/nuttx/dma-heap.h
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

#ifndef __INCLUDE_NUTTX_DMA_HEAP_H
#define __INCLUDE_NUTTX_DMA_HEAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <fcntl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Valid FD_FLAGS are O_CLOEXEC, O_RDONLY, O_WRONLY, O_RDWR */

#define DMA_HEAP_VALID_FD_FLAGS (O_CLOEXEC | O_ACCMODE)

/* Currently no heap flags */

#define DMA_HEAP_VALID_HEAP_FLAGS 0

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* information needed to export a new dmabuf heap */

struct dma_heap;
struct dma_heap_export_info
{
  FAR const char *name;               /* used for device-node name */
  FAR void *priv;                     /* heap exporter private data */
  CODE FAR struct dma_buf *(*allocate)(FAR struct dma_heap *heap, size_t len,
                                       int fd_flags, int heap_flags);
};

struct dma_heap
{
  FAR const char *name;               /* used for device-node name */
  FAR void *priv;                     /* heap exporter private data */
  CODE FAR struct dma_buf *(*allocate)(FAR struct dma_heap *heap, size_t len,
                                       int fd_flags, int heap_flags);
};

/* Metadata of allocations to dma-heap, Provided by userspace as an
 * argument to the ioctl
 */

struct dma_heap_allocation_data
{
  size_t len;     /* size of the allocation */
  int fd;         /* will be populated with a fd which provides the
                   * handle to the allocated dma-buf
                   */
  int fd_flags;   /* file descriptor flags used when allocating */
  int heap_flags; /* flags passed to heap */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: dma_heap_add
 *
 * Description:
 *   Adds a heap to dmabuf heaps
 *
 * Input Parameters:
 *   exp_info: information needed to register this heap
 *
 ****************************************************************************/

FAR struct dma_heap *
dma_heap_add(FAR const struct dma_heap_export_info *exp_info);

/****************************************************************************
 * Name: dmabuf_system_heap_create
 *
 * Description:
 *   Create and register the system heap for dma-buf allocations
 *
 ****************************************************************************/

int dmabuf_system_heap_create(void);

#endif
