/****************************************************************************
 * drivers/dma-buf/dma-buf.c
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

#include <nuttx/config.h>
#include <nuttx/atomic.h>
#include <nuttx/dma-buf.h>
#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/nuttx.h>
#include <nuttx/sched.h>

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int dma_buf_close(FAR struct file *filep);
static off_t dma_buf_seek(FAR struct file *filep, off_t offset, int whence);
static int dma_buf_ioctl(FAR struct file *filep, int cmd,
                         unsigned long arg);
static int dma_buf_mmap(FAR struct file *filep,
                        FAR struct mm_map_entry_s *map);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_dma_buf_fops =
{
  .close  = dma_buf_close,
  .seek   = dma_buf_seek,
  .ioctl  = dma_buf_ioctl,
  .mmap   = dma_buf_mmap,
};

static struct inode g_dma_buf_inode =
{
  NULL,                   /* i_parent */
  NULL,                   /* i_peer */
  NULL,                   /* i_child */
  1,                      /* i_crefs */
  FSNODEFLAG_TYPE_DRIVER, /* i_flags */
  {
    &g_dma_buf_fops       /* u */
  }
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int dma_buf_close(FAR struct file *filep)
{
  FAR struct dma_buf *dmabuf = filep->f_priv;

  DEBUGASSERT(dmabuf != NULL);
  dma_buf_put(dmabuf);

  return OK;
}

static off_t dma_buf_seek(FAR struct file *filep, off_t offset, int whence)
{
  FAR struct dma_buf *dmabuf = filep->f_priv;
  off_t base;

  /* only support discovering the end of the buffer,
   * but also allow SEEK_SET to maintain the idiomatic
   * SEEK_END(0), SEEK_CUR(0) pattern.
   */

  if (whence == SEEK_END)
    {
      base = dmabuf->size;
    }
  else if (whence == SEEK_SET)
    {
      base = 0;
    }
  else
    {
      return -EINVAL;
    }

  if (offset != 0)
    {
      return -EINVAL;
    }

  return base + offset;
}

/****************************************************************************
 * Name: dma_buf_set_name
 *
 * Description:
 *   Set a name to a specific dma_buf to track the usage.
 *   It could support changing the name of the dma-buf if the same
 *   piece of memory is used for multiple purpose between different devices.
 *
 * Input Parameters:
 *   dmabuf - dmabuf buffer that will be renamed.
 *   buf    - A piece of userspace memory that contains the name of
 *            the dma-buf.
 *
 * Returned Value:
 *   Returns 0 on success, or a negative error code on failure.
 *
 ****************************************************************************/

static long dma_buf_set_name(struct dma_buf *dmabuf, FAR const char *buf)
{
  FAR char *name = strdup(buf);

  if (name == NULL)
    {
      return -ENOMEM;
    }

  nxmutex_lock(&dmabuf->lock);
  kmm_free(dmabuf->name);
  dmabuf->name = name;
  nxmutex_unlock(&dmabuf->lock);

  return 0;
}

static int dma_buf_ioctl(FAR struct file *filep, int cmd,
                         unsigned long arg)
{
  FAR struct dma_buf *dmabuf = filep->f_priv;

  switch (cmd)
    {
      case DMA_BUF_IOCTL_SYNC:
        {
          FAR struct dma_buf_sync *sync = (FAR void *)(uintptr_t)arg;
          enum dma_data_direction direction;
          int ret = -EINVAL;

          if (sync->flags & ~DMA_BUF_SYNC_MASK)
            {
              return ret;
            }

          switch (sync->flags & DMA_BUF_SYNC_RW)
            {
              case DMA_BUF_SYNC_READ:
                direction = DMA_FROM_DEVICE;
                break;

              case DMA_BUF_SYNC_WRITE:
                direction = DMA_TO_DEVICE;
                break;

              case DMA_BUF_SYNC_RW:
                direction = DMA_BIDIRECTIONAL;
                break;

              default:
                return ret;
            }

          if (sync->flags & DMA_BUF_SYNC_END)
            {
              ret = dma_buf_end_cpu_access(dmabuf, direction);
            }
          else
            {
              ret = dma_buf_begin_cpu_access(dmabuf, direction);
            }

          return ret;
        }

      case DMA_BUF_SET_NAME:
        {
          return dma_buf_set_name(dmabuf, (FAR const char *)(uintptr_t)arg);
        }

      default:
        return -ENOTTY;
    }
}

static int dma_buf_munmap(FAR struct task_group_s *group,
                          FAR struct mm_map_entry_s *entry,
                          FAR void *start, size_t length)
{
  FAR struct dma_buf *dmabuf = entry->priv.p;
  int ret;

  DEBUGASSERT(dmabuf != NULL && start != NULL);

  ret = mm_map_remove(get_group_mm(group), entry);
  if (ret >= 0)
    {
      /* Drop the reference taken in dma_buf_mmap() */

      dma_buf_put(dmabuf);
    }

  return ret;
}

static int dma_buf_mmap(FAR struct file *filep,
                        FAR struct mm_map_entry_s *map)
{
  FAR struct dma_buf *dmabuf = filep->f_priv;
  int ret;

  if (!dmabuf->ops->mmap || map->length > dmabuf->size)
    {
      return -EINVAL;
    }

  map->priv.p = dmabuf;
  map->munmap = dma_buf_munmap;
  ret = dmabuf->ops->mmap(dmabuf, map);
  if (ret >= 0)
    {
      /* Hold a reference for the lifetime of this mapping so the
       * buffer cannot be released while the mapping is still valid.
       */

      atomic_add(&dmabuf->refcount, 1);
      mm_map_add(get_current_mm(), map);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dma_buf_fd
 *
 * Description:
 *   Returns a file descriptor for the given struct dma_buf
 *
 * Input Parameters:
 *   dmabuf - pointer to dma_buf for which fd is required.
 *   flags  - flags to give to fd
 *
 * Returned Value:
 *   On success, returns an associated 'fd'. Else, returns error.
 *
 ****************************************************************************/

int dma_buf_fd(FAR struct dma_buf *dmabuf, int flags)
{
  int fd;

  if (dmabuf == NULL)
    {
      return -EINVAL;
    }

  atomic_add(&dmabuf->refcount, 1);
  fd = file_allocate_from_inode(&g_dma_buf_inode, flags, 0, dmabuf, 0);
  if (fd < 0)
    {
      dma_buf_put(dmabuf);
    }

  return fd;
}

/****************************************************************************
 * Name: dma_buf_get
 *
 * Description:
 *   Returns the struct dma_buf related to an fd
 *
 * Input Parameters:
 *   fd - fd associated with the struct dma_buf to be returned
 *
 * Returned Value:
 *   On success, returns the struct dma_buf associated with an fd; uses
 *   file's refcounting done by file_get to increase refcount. returns NULL
 *   otherwise.
 *
 ****************************************************************************/

FAR struct dma_buf *dma_buf_get(int fd)
{
  FAR struct file *filep;
  FAR struct dma_buf *dmabuf = NULL;

  if (file_get(fd, &filep) < 0)
    {
      return NULL;
    }

  if (filep->f_inode != NULL &&
      filep->f_inode->u.i_ops == &g_dma_buf_fops)
    {
      dmabuf = filep->f_priv;
      atomic_add(&dmabuf->refcount, 1);
    }

  file_put(filep);
  return dmabuf;
}

/****************************************************************************
 * Name: dma_buf_put
 *
 * Description:
 *   Decreases refcount of the buffer
 *   If, as a result of this call, the refcount becomes 0, the 'release' file
 *   operation related to this fd is called. It calls &dma_buf_ops.release
 *   vfunc in turn, and frees the memory allocated for dmabuf when exported.
 *
 * Input Parameters:
 *   dmabuf - buffer to reduce refcount of
 *
 ****************************************************************************/

void dma_buf_put(FAR struct dma_buf *dmabuf)
{
  if (dmabuf != NULL && atomic_sub(&dmabuf->refcount, 1) == 1)
    {
      dmabuf->ops->release(dmabuf);
      nxmutex_destroy(&dmabuf->lock);
      kmm_free(dmabuf->name);
      kmm_free(dmabuf);
    }
}

/****************************************************************************
 * Name: dma_buf_attach
 *
 * Description:
 *   Wrapper to call dma_buf_dynamic_attach() for drivers which still use
 *   a static mapping.
 *
 * Input Parameters:
 *   dmabuf - buffer to reduce refcount of
 *
 ****************************************************************************/

FAR struct dma_buf_attachment *dma_buf_attach(FAR struct dma_buf *dmabuf)
{
  FAR struct dma_buf_attachment *attach;

  if (dmabuf == NULL)
    {
      return NULL;
    }

  attach = kmm_zalloc(sizeof(struct dma_buf_attachment));
  if (attach == NULL)
    {
      return NULL;
    }

  attach->dmabuf = dmabuf;
  nxmutex_lock(&dmabuf->lock);
  if (dmabuf->ops->attach)
    {
      int ret;

      ret = dmabuf->ops->attach(dmabuf, attach);
      if (ret < 0)
        {
          nxmutex_unlock(&dmabuf->lock);
          kmm_free(attach);
          return NULL;
        }
    }

  list_add_tail(&dmabuf->attachments, &attach->node);
  nxmutex_unlock(&dmabuf->lock);

  return attach;
}

/****************************************************************************
 * Name: dma_buf_detach
 *
 * Description:
 *   Remove the given attachment from dmabuf's attachments list
 *   Clean up a device attachment obtained by calling dma_buf_attach().
 *   Optionally this calls &dma_buf_ops.detach for device-specific detach.
 *
 * Input Parameters:
 *   dmabuf - buffer to detach from.
 *   attach - attachment to be detached; is free'd after this call.
 *
 ****************************************************************************/

void dma_buf_detach(FAR struct dma_buf *dmabuf,
                    FAR struct dma_buf_attachment *attach)
{
  if (dmabuf == NULL || attach == NULL || dmabuf != attach->dmabuf)
    {
      return;
    }

  nxmutex_lock(&dmabuf->lock);
  list_delete(&attach->node);

  if (dmabuf->ops->detach)
    {
      dmabuf->ops->detach(dmabuf, attach);
    }

  nxmutex_unlock(&dmabuf->lock);
  kmm_free(attach);
}

/****************************************************************************
 * Name: dma_buf_begin_cpu_access
 *
 * Description:
 *   Must be called before accessing a dma_buf from the cpu in the kernel.
 *   Calls begin_cpu_access to allow exporter-specific preparations.
 *   Coherency is only guaranteed in the specified range for the specified
 *   access direction.
 *
 *   After the cpu access is complete the caller should call
 *   dma_buf_end_cpu_access(). Only when cpu access is bracketed by both
 *   calls is it guaranteed to be coherent with other DMA access.
 *
 *   This function will also wait for any DMA transactions tracked through
 *   implicit synchronization in &dma_buf.resv. For DMA transactions with
 *   explicit synchronization this function will only ensure cache
 *   coherency, callers must ensure synchronization with such DMA
 *   transactions on their own.
 *
 * Input Parameters:
 *   dmabuf    - buffer to prepare cpu access for.
 *   direction - direction of access.
 *
 * Returned Value:
 *   return negative error values, returns 0 on success.
 *
 ****************************************************************************/

int dma_buf_begin_cpu_access(FAR struct dma_buf *dmabuf,
                             enum dma_data_direction direction)
{
  int ret = 0;

  if (dmabuf == NULL)
    {
      return -EINVAL;
    }

  nxmutex_lock(&dmabuf->lock);
  if (dmabuf->ops->begin_cpu_access)
    {
      ret = dmabuf->ops->begin_cpu_access(dmabuf, direction);
    }

  nxmutex_unlock(&dmabuf->lock);
  return ret;
}

/****************************************************************************
 * Name: dma_buf_end_cpu_access
 *
 * Description:
 *   Must be called after accessing a dma_buf from the cpu in the kernel
 *   context. Calls end_cpu_access to allow exporter-specific actions.
 *   Coherency is only guaranteed in the specified range for the
 *   specified access direction.
 *
 *   This terminates CPU access started with dma_buf_begin_cpu_access().
 *
 * Input Parameters:
 *   dmabuf    - buffer to prepare cpu access for.
 *   direction - direction of access.
 *
 * Returned Value:
 *   return negative error values, returns 0 on success.
 *
 ****************************************************************************/

int dma_buf_end_cpu_access(FAR struct dma_buf *dmabuf,
                           enum dma_data_direction direction)
{
  int ret = 0;

  nxmutex_lock(&dmabuf->lock);
  if (dmabuf->ops->end_cpu_access)
    {
      ret = dmabuf->ops->end_cpu_access(dmabuf, direction);
    }

  nxmutex_unlock(&dmabuf->lock);
  return ret;
}

/****************************************************************************
 * Name: dma_buf_map_attachment
 *
 * Description:
 *   Returns the pointer of address of the attachment, mapped into _device_
 *   address space. Is a wrapper for map_dma_buf() of the dma_buf_ops.
 *
 * Input Parameters:
 *   attach    - attachment whose scatterlist is to be returned
 *   direction - direction of DMA transfer
 *
 * Returned Value:
 *   Returns the attachment's physical address on success, 0 on failure.
 *
 *   A mapping must be unmapped by using dma_buf_unmap_attachment().
 *   Note that the underlying backing storage is pinned for as long as a
 *   mapping exists, therefore users/importers should not hold onto a
 *   mapping for undue amounts of time.
 *
 ****************************************************************************/

uintptr_t dma_buf_map_attachment(FAR struct dma_buf_attachment *attach,
                                 enum dma_data_direction direction)
{
  uintptr_t buf;

  if (attach == NULL || attach->dmabuf == NULL)
    {
      return 0;
    }

  nxmutex_lock(&attach->dmabuf->lock);
  buf = attach->dmabuf->ops->map_dma_buf(attach, direction);
  nxmutex_unlock(&attach->dmabuf->lock);

  return buf;
}

/****************************************************************************
 * Name: dma_buf_unmap_attachment
 *
 * Description:
 *   unmaps and decreases usecount of the buffer;
 *   Is a wrapper for unmap_dma_buf() of dma_buf_ops.
 *
 * Input Parameters:
 *   attach    - attachment to unmap buffer from
 *   buf       - address returned by dma_buf_map_attachment
 *   direction - direction of DMA transfer
 *
 * Returned Value:
 *   Returns, on success, a newly created struct dma_buf object, which wraps
 *   the supplied private data and operations for struct dma_buf_ops.
 *   On either missing ops, or error in allocating struct dma_buf, will
 *   return negative error.
 *
 ****************************************************************************/

void dma_buf_unmap_attachment(FAR struct dma_buf_attachment *attach,
                              uintptr_t buf,
                              enum dma_data_direction direction)
{
  if (attach == NULL || attach->dmabuf == NULL || buf == 0)
    {
      return;
    }

  nxmutex_lock(&attach->dmabuf->lock);
  attach->dmabuf->ops->unmap_dma_buf(attach, buf, direction);
  nxmutex_unlock(&attach->dmabuf->lock);
}

/****************************************************************************
 * Name: dma_buf_export
 *
 * Description:
 *   Creates a new dma_buf, and associates an anon file with this buffer,
 *   so it can be exported.
 *   Also connect the allocator specific data and ops to the buffer.
 *   Additionally, provide a name string for exporter; useful in debugging.
 *
 * Input Parameters:
 *   exp_info - holds all the export related information provided by the
 *              exporter. see &struct dma_buf_export_info for further
 *              details.
 *
 * Returned Value:
 *   Returns, on success, a newly created struct dma_buf object, which wraps
 *   the supplied private data and operations for struct dma_buf_ops.
 *   On either missing ops, or error in allocating struct dma_buf, will
 *   return negative error.
 *
 ****************************************************************************/

FAR struct dma_buf *
dma_buf_export(FAR const struct dma_buf_export_info *exp_info)
{
  FAR struct dma_buf *dmabuf;

  if (exp_info->priv == NULL ||
      exp_info->ops == NULL ||
      exp_info->ops->map_dma_buf == NULL ||
      exp_info->ops->unmap_dma_buf == NULL ||
      exp_info->ops->release == NULL)
    {
      return NULL;
    }

  dmabuf = kmm_zalloc(sizeof(*dmabuf));
  if (dmabuf == NULL)
    {
      return NULL;
    }

  dmabuf->priv     = exp_info->priv;
  dmabuf->ops      = exp_info->ops;
  dmabuf->size     = exp_info->size;
  dmabuf->exp_name = exp_info->exp_name;

  nxmutex_init(&dmabuf->lock);
  list_initialize(&dmabuf->attachments);

  atomic_set(&dmabuf->refcount, 1);

  return dmabuf;
}
