/****************************************************************************
 * drivers/dma-buf/dma-heap.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/dma-buf.h>
#include <nuttx/dma-heap.h>
#include <nuttx/kmalloc.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DMA_HEAP_DEVNAME  "/dev/dma_heap"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int dma_heap_ioctl(FAR struct file *filep, int cmd,
                          unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_dma_heap_fops =
{
  .ioctl = dma_heap_ioctl
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int dma_heap_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct dma_heap *heap = inode->i_private;
  int ret = 0;

  switch (cmd)
    {
      case DMA_BUF_HEAP_IOCTL_ALLOC:
        {
          FAR struct dma_heap_allocation_data *data;
          FAR struct dma_buf *dmabuf;

          data = (FAR void *)(uintptr_t)arg;
          if (data->fd != 0)
            {
              return -EINVAL;
            }

          if (data->fd_flags & ~DMA_HEAP_VALID_FD_FLAGS)
            {
              return -EINVAL;
            }

          if (data->heap_flags & ~DMA_HEAP_VALID_HEAP_FLAGS)
            {
              return -EINVAL;
            }

          dmabuf = heap->allocate(heap, data->len, data->fd_flags,
                                  data->heap_flags);
          if (dmabuf == NULL)
            {
              return -ENOMEM;
            }

          data->fd = dma_buf_fd(dmabuf, data->fd_flags);
          dma_buf_put(dmabuf);
          if (data->fd < 0)
            {
              ret = data->fd;
            }

          break;
        }

      default:
        ret = -ENOTTY;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
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
dma_heap_add(FAR const struct dma_heap_export_info *exp_info)
{
  FAR struct dma_heap *heap;
  char path[64];
  int ret;

  if (exp_info->name == NULL || !strcmp(exp_info->name, ""))
    {
      dmaerr("dma_heap: Cannot add heap without a name\n");
      return NULL;
    }

  if (exp_info->allocate == NULL)
    {
      dmaerr("dma_heap: Cannot add heap with invalid allocate\n");
      return NULL;
    }

  heap = kmm_zalloc(sizeof(*heap));
  if (heap == NULL)
    {
      dmaerr("dma_heap: Cannot allocate memory for dma heap\n");
      return NULL;
    }

  heap->name = exp_info->name;
  heap->allocate = exp_info->allocate;
  heap->priv = exp_info->priv;

  snprintf(path, sizeof(path), "%s/%s", DMA_HEAP_DEVNAME, heap->name);
  ret = register_driver(path, &g_dma_heap_fops, 0666, heap);
  if (ret < 0)
    {
      dmaerr("dma_heap: Failed to register driver:%d\n", ret);
      kmm_free(heap);
      return NULL;
    }

  return heap;
}
