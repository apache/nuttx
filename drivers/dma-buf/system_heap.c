/****************************************************************************
 * drivers/dma-buf/system_heap.c
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

#include <nuttx/arch.h>
#include <nuttx/cache.h>
#include <nuttx/dma-buf.h>
#include <nuttx/dma-heap.h>
#include <nuttx/kmalloc.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

struct system_heap_buffer
{
  FAR struct dma_heap *heap;
  struct list_node attachments;
  size_t len;
  FAR void *buf;
};

struct dma_heap_attachment
{
  struct list_node node;
  FAR void *buf;
  bool mapped;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int system_heap_attach(FAR struct dma_buf *dmabuf,
                              FAR struct dma_buf_attachment *attachment);
static void system_heap_detach(FAR struct dma_buf *dmabuf,
                               FAR struct dma_buf_attachment *attachment);
static uintptr_t
system_heap_map_dma_buf(FAR struct dma_buf_attachment *attachment,
                        enum dma_data_direction direction);
static void
system_heap_unmap_dma_buf(FAR struct dma_buf_attachment *attachment,
                          uintptr_t buf,
                          enum dma_data_direction direction);
static int
system_heap_dma_buf_begin_cpu_access(FAR struct dma_buf *dmabuf,
                                     enum dma_data_direction direction);
static int
system_heap_dma_buf_end_cpu_access(FAR struct dma_buf *dmabuf,
                                   enum dma_data_direction direction);
static int system_heap_mmap(FAR struct dma_buf *dmabuf,
                            FAR struct mm_map_entry_s *vma);
static void system_heap_dma_buf_release(FAR struct dma_buf *dmabuf);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct dma_buf_ops g_system_heap_buf_ops =
{
  .attach = system_heap_attach,
  .detach = system_heap_detach,
  .map_dma_buf = system_heap_map_dma_buf,
  .unmap_dma_buf = system_heap_unmap_dma_buf,
  .begin_cpu_access = system_heap_dma_buf_begin_cpu_access,
  .end_cpu_access = system_heap_dma_buf_end_cpu_access,
  .mmap = system_heap_mmap,
  .release = system_heap_dma_buf_release,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int system_heap_attach(FAR struct dma_buf *dmabuf,
                              FAR struct dma_buf_attachment *attachment)
{
  FAR struct system_heap_buffer *buffer = dmabuf->priv;
  FAR struct dma_heap_attachment *a;

  a = kmm_zalloc(sizeof(*a));
  if (a == NULL)
    {
      return -ENOMEM;
    }

  a->buf = buffer->buf;
  attachment->priv = a;

  list_add_tail(&buffer->attachments, &a->node);

  return 0;
}

static void system_heap_detach(FAR struct dma_buf *dmabuf,
                               FAR struct dma_buf_attachment *attachment)
{
  FAR struct dma_heap_attachment *a = attachment->priv;

  list_delete(&a->node);
  kmm_free(a);
}

static uintptr_t
system_heap_map_dma_buf(FAR struct dma_buf_attachment *attachment,
                        enum dma_data_direction direction)
{
  FAR struct dma_heap_attachment *a = attachment->priv;

  a->mapped = true;
#ifdef CONFIG_ARCH_HAVE_ADDRENV
  return up_addrenv_va_to_pa(a->buf);
#else
  /* Flat / identity-mapped systems: VA is the DMA address */

  return (uintptr_t)a->buf;
#endif
}

static void
system_heap_unmap_dma_buf(FAR struct dma_buf_attachment *attachment,
                          uintptr_t buf,
                          enum dma_data_direction direction)
{
  FAR struct dma_heap_attachment *a = attachment->priv;

  a->mapped = false;
}

static int
system_heap_dma_buf_begin_cpu_access(FAR struct dma_buf *dmabuf,
                                     enum dma_data_direction direction)
{
  FAR struct system_heap_buffer *buffer = dmabuf->priv;
  FAR struct dma_heap_attachment *a;
  bool mapped = false;

  list_for_every_entry(&buffer->attachments, a,
                       struct dma_heap_attachment, node)
    {
      if (a->mapped)
        {
          mapped = true;
          break;
        }
    }

  if (mapped)
    {
      up_invalidate_dcache((uintptr_t)buffer->buf,
                           (uintptr_t)buffer->buf + buffer->len);
    }

  return 0;
}

static int
system_heap_dma_buf_end_cpu_access(FAR struct dma_buf *dmabuf,
                                   enum dma_data_direction direction)
{
  FAR struct system_heap_buffer *buffer = dmabuf->priv;
  FAR struct dma_heap_attachment *a;
  bool mapped = false;

  list_for_every_entry(&buffer->attachments, a,
                       struct dma_heap_attachment, node)
    {
      if (a->mapped)
        {
          mapped = true;
          break;
        }
    }

  if (mapped)
    {
      up_flush_dcache((uintptr_t)buffer->buf,
                      (uintptr_t)buffer->buf + buffer->len);
    }

  return 0;
}

static int system_heap_mmap(FAR struct dma_buf *dmabuf,
                            FAR struct mm_map_entry_s *vma)
{
  FAR struct system_heap_buffer *buffer = dmabuf->priv;

  vma->vaddr = buffer->buf;
  vma->length = buffer->len;
  return 0;
}

static void system_heap_dma_buf_release(FAR struct dma_buf *dmabuf)
{
  FAR struct system_heap_buffer *buffer = dmabuf->priv;

  kumm_free(buffer->buf);
  kmm_free(buffer);
}

static FAR struct dma_buf *
system_heap_allocate(FAR struct dma_heap *heap, size_t len,
                     int fd_flags, int heap_flags)
{
  FAR struct system_heap_buffer *buffer;
  struct dma_buf_export_info exp_info;
  FAR struct dma_buf *dmabuf;

  buffer = kmm_zalloc(sizeof(*buffer));
  if (buffer == NULL)
    {
      return NULL;
    }

  /* Allocate the data buffer with 32-byte alignment so that
   * drivers requiring DMA-aligned addresses (e.g. camera) work
   * correctly when importing this dma_buf.
   */

  buffer->buf = kumm_memalign(32, len);
  if (buffer->buf == NULL)
    {
      kmm_free(buffer);
      return NULL;
    }

  memset(buffer->buf, 0, len);

  list_initialize(&buffer->attachments);
  buffer->heap = heap;
  buffer->len = len;

  exp_info.exp_name = heap->name;
  exp_info.ops = &g_system_heap_buf_ops;
  exp_info.size = buffer->len;
  exp_info.flags = fd_flags;
  exp_info.priv = buffer;
  dmabuf = dma_buf_export(&exp_info);
  if (dmabuf == NULL)
    {
      kumm_free(buffer->buf);
      kmm_free(buffer);
    }

  return dmabuf;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dmabuf_system_heap_create
 *
 * Description:
 *   Create and register the system dma-buf heap.
 *
 ****************************************************************************/

int dmabuf_system_heap_create(void)
{
  struct dma_heap_export_info exp_info;
  FAR struct dma_heap *sys_heap;

  exp_info.name = "system";
  exp_info.allocate = system_heap_allocate;
  exp_info.priv = NULL;

  sys_heap = dma_heap_add(&exp_info);
  if (sys_heap == NULL)
    {
      return -ENOMEM;
    }

  return 0;
}
