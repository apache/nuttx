/****************************************************************************
 * fs/shm/shmfs.c
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

#include <assert.h>

#include <nuttx/fs/ioctl.h>
#include <nuttx/mm/map.h>
#include <nuttx/arch.h>
#include <nuttx/pgalloc.h>
#include <nuttx/sched.h>

#include "shm/shmfs.h"
#include "inode/inode.h"
#include "sched/sched.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int shmfs_close(FAR struct file *filep);
static ssize_t shmfs_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen);
static ssize_t shmfs_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen);
static int shmfs_truncate(FAR struct file *filep, off_t length);

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int shmfs_unlink(FAR struct inode *inode);
#endif

static int shmfs_mmap(FAR struct file *filep,
                      FAR struct mm_map_entry_s *entry);
static int shmfs_munmap(FAR struct task_group_s *group,
                        FAR struct mm_map_entry_s *entry,
                        FAR void *start,
                        size_t length);

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct file_operations g_shmfs_operations =
{
  NULL,             /* open */
  shmfs_close,      /* close */
  shmfs_read,       /* read */
  shmfs_write,      /* write */
  NULL,             /* seek */
  NULL,             /* ioctl */
  shmfs_mmap,       /* mmap */
  shmfs_truncate,   /* truncate */
  NULL,             /* poll */
  NULL,             /* readv */
  NULL,             /* writev */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  shmfs_unlink      /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: shmfs_read
 ****************************************************************************/

static ssize_t shmfs_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen)
{
  FAR struct shmfs_object_s *sho;
  ssize_t nread;
  off_t startpos;
  off_t endpos;

  DEBUGASSERT(filep->f_inode->i_private != NULL);

  sho = filep->f_inode->i_private;

  if (filep->f_pos > sho->length)
    {
      return 0;
    }

  /* Handle attempts to read beyond the end of the file. */

  startpos = filep->f_pos;
  nread    = buflen;
  endpos   = startpos + buflen;

  if (endpos > sho->length)
    {
      endpos = sho->length;
      nread  = endpos - startpos;
    }

  /* Copy data from the memory object to the user buffer */

  if (sho->paddr != NULL)
    {
      memcpy(buffer, (FAR char *)sho->paddr + startpos, nread);
      filep->f_pos += nread;
    }
  else
    {
      DEBUGASSERT(sho->length == 0 && nread == 0);
    }

  return nread;
}

/****************************************************************************
 * Name: shmfs_write
 ****************************************************************************/

static ssize_t shmfs_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen)
{
  FAR struct shmfs_object_s *sho;
  ssize_t nwritten;
  off_t startpos;
  off_t endpos;

  DEBUGASSERT(filep->f_inode->i_private != NULL);

  sho = filep->f_inode->i_private;

  /* Handle attempts to write beyond the end of the file */

  startpos = filep->f_pos;
  nwritten = buflen;
  endpos   = startpos + buflen;

  /* Desn't support shm auto expand, truncate first */

  if (endpos > sho->length)
    {
      return -EFBIG;
    }

  /* Copy data from the user buffer to the memory object */

  if (sho->paddr != NULL)
    {
      memcpy((FAR char *)sho->paddr + startpos, buffer, nwritten);
      filep->f_pos += nwritten;
    }
  else
    {
      DEBUGASSERT(sho->length == 0 && nwritten == 0);
    }

  return nwritten;
}

/****************************************************************************
 * Name: shmfs_release
 ****************************************************************************/

static int shmfs_release(FAR struct inode *inode)
{
  /* If the file has been unlinked previously, delete the contents.
   * The inode is released after this call, hence checking if i_crefs <= 1.
   */

  if (inode->i_parent == NULL && atomic_load(&inode->i_crefs) <= 1)
    {
      shmfs_free_object(inode->i_private);
      inode->i_private = NULL;
    }

  return OK;
}

/****************************************************************************
 * Name: shmfs_close
 ****************************************************************************/

static int shmfs_close(FAR struct file *filep)
{
  /* Release the shmfs object. The object gets deleted if no-one has
   * reference to it (either mmap or open file) and the object has been
   * unlinked
   */

  return shmfs_release(filep->f_inode);
}

/****************************************************************************
 * Name: shmfs_truncate
 ****************************************************************************/

static int shmfs_truncate(FAR struct file *filep, off_t length)
{
  FAR struct shmfs_object_s *object;
  int ret = 0;

  if (length == 0)
    {
      return -EINVAL;
    }

  inode_lock();
  object = filep->f_inode->i_private;
  if (!object)
    {
      filep->f_inode->i_private = shmfs_alloc_object(length);
      if (!filep->f_inode->i_private)
        {
          filep->f_inode->i_size = 0;
          ret = -EFAULT;
        }
      else
        {
          filep->f_inode->i_size = length;
        }
    }
  else if (object->length != length)
    {
      /* This doesn't support resize */

      ret = -EINVAL;
    }

  inode_unlock();
  return ret;
}

/****************************************************************************
 * Name: shmfs_unlink
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int shmfs_unlink(FAR struct inode *inode)
{
  if (atomic_load(&inode->i_crefs) <= 1)
    {
      shmfs_free_object(inode->i_private);
      inode->i_private = NULL;
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: shmfs_map_object
 ****************************************************************************/

static int shmfs_map_object(FAR struct shmfs_object_s *object,
                            FAR void **vaddr)
{
  int ret = OK;

#ifdef CONFIG_BUILD_KERNEL
  /* Map the physical pages of the shm object with MMU. */

  FAR struct tcb_s *tcb = this_task();
  FAR struct task_group_s *group = tcb->group;
  FAR uintptr_t *pages = (FAR uintptr_t *)&object->paddr;
  uintptr_t mapaddr;
  unsigned int npages;

  /* Find a free vaddr space that satisfies length */

  mapaddr = (uintptr_t)vm_alloc_region(get_group_mm(group), 0,
                                       object->length);
  if (mapaddr == 0)
    {
      return -ENOMEM;
    }

  /* Convert the region size to pages */

  npages = MM_NPAGES(object->length);

  /* Map the memory to user virtual address space */

  ret = up_shmat(pages, npages, mapaddr);
  if (ret < 0)
    {
      vm_release_region(get_group_mm(group), (FAR void *)mapaddr,
                        object->length);
    }
  else
    {
      *vaddr = (FAR void *)mapaddr;
    }
#else
  /* Use the physical address directly */

  *vaddr = object->paddr;
#endif

  return ret;
}

/****************************************************************************
 * Name: shmfs_add_map
 ****************************************************************************/

static int shmfs_add_map(FAR struct mm_map_entry_s *entry,
                         FAR struct inode *inode)
{
  entry->munmap = shmfs_munmap;
  entry->priv.p = (FAR void *)inode;
  return mm_map_add(get_current_mm(), entry);
}

/****************************************************************************
 * Name: shmfs_mmap
 ****************************************************************************/

static int shmfs_mmap(FAR struct file *filep,
                      FAR struct mm_map_entry_s *entry)
{
  FAR struct shmfs_object_s *object;
  int ret = -EINVAL;

  /* We don't support offset at the moment, just mapping the whole object
   * object is NULL if it hasn't been truncated yet
   */

  if (entry->offset != 0)
    {
      return ret;
    }

  /* Keep the inode when mmapped, increase refcount */

  inode_addref(filep->f_inode);
  object = filep->f_inode->i_private;
  if (object)
    {
      ret = shmfs_map_object(object, &entry->vaddr);
    }

  if (ret < 0 ||
      (ret = shmfs_add_map(entry, filep->f_inode)) < 0)
    {
      inode_release(filep->f_inode);
    }

  return ret;
}

/****************************************************************************
 * Name: shmfs_unmap_object
 ****************************************************************************/

static int shmfs_unmap_area(FAR struct task_group_s *group,
                            FAR void *vaddr, size_t length)
{
  int ret = OK;

#ifdef CONFIG_BUILD_KERNEL
  unsigned int npages;

  /* Convert the region size to pages */

  if (group)
    {
      npages = MM_NPAGES(length);

      /* Unmap the memory from user virtual address space */

      ret = up_shmdt((uintptr_t)vaddr, npages);
      if (ret < 0)
        {
          return ret;
        }

      /* Free the virtual address space */

      vm_release_region(get_group_mm(group), vaddr, length);
    }
  else
    {
      return -EINVAL;
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: shmfs_munmap
 ****************************************************************************/

static int shmfs_munmap(FAR struct task_group_s *group,
                        FAR struct mm_map_entry_s *entry,
                        FAR void *start,
                        size_t length)
{
  FAR struct inode *inode;
  int ret;

  /* Partial unmap is not supported yet */

  if (start != entry->vaddr || length != entry->length)
    {
      return -EINVAL;
    }

  inode = (FAR struct inode *)entry->priv.p;

  /* Unmap the virtual memory area from the user's address space */

  ret = shmfs_unmap_area(group, entry->vaddr, entry->length);

  /* Release the shmfs object. The object gets deleted if no-one has
   * reference to it (either mmap or open file) and the object has been
   * unlinked
   */

  if (ret == OK)
    {
      ret = shmfs_release(inode);
    }

  /* Unkeep the inode when unmapped, decrease refcount */

  if (ret == OK)
    {
      inode_release(inode);

      /* Remove the mapping. */

      ret = mm_map_remove(get_group_mm(group), entry);
    }

  return ret;
}
