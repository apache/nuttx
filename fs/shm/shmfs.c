/****************************************************************************
 * fs/shm/shmfs.c
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

#if defined (CONFIG_BUILD_KERNEL)
#include <nuttx/arch.h>
#include <nuttx/pgalloc.h>
#include <nuttx/sched.h>
#endif

#include "shm/shmfs.h"
#include "inode/inode.h"

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

const struct file_operations shmfs_operations =
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
  return -ENOSYS;
}

/****************************************************************************
 * Name: shmfs_write
 ****************************************************************************/

static ssize_t shmfs_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: shmfs_release
 ****************************************************************************/

static int shmfs_release(FAR struct inode *inode)
{
  /* If the file has been unlinked previously, delete the contents.
   * The inode is released after this call, hence checking if i_crefs <= 1.
   */

  int ret = inode_lock();
  if (ret >= 0)
    {
      if (inode->i_parent == NULL &&
          inode->i_crefs <= 1)
        {
          shmfs_free_object(inode->i_private);
          inode->i_private = NULL;
          ret = OK;
        }

      inode_unlock();
    }

  return ret;
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
  int ret;

  if (length == 0)
    {
      return -EINVAL;
    }

  ret = inode_lock();
  if (ret >= 0)
    {
      object = filep->f_inode->i_private;
      if (!object)
        {
          filep->f_inode->i_private = shmfs_alloc_object(length);
          if (!filep->f_inode->i_private)
            {
              ret = -EFAULT;
            }
        }
      else if (object->length != length)
        {
          /* This doesn't support resize */

          ret = -EINVAL;
        }

      inode_unlock();
    }

  return ret;
}

/****************************************************************************
 * Name: shmfs_unlink
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int shmfs_unlink(FAR struct inode *inode)
{
  int ret = inode_lock();

  if (ret >= 0)
    {
      if (inode->i_crefs <= 1)
        {
          shmfs_free_object(inode->i_private);
          inode->i_private = NULL;
        }

      inode_unlock();
    }

  return ret;
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

  FAR struct tcb_s *tcb = nxsched_self();
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
 * Name: shmfs_mmap
 ****************************************************************************/

static int shmfs_mmap(FAR struct file *filep,
                      FAR struct mm_map_entry_s *entry)
{
  FAR struct shmfs_object_s *object;
  int ret = -EINVAL;

  DEBUGASSERT(filep->f_inode != NULL);

  /* We don't support offset at the moment, just mapping the whole object
   * object is NULL if it hasn't been truncated yet
   */

  if (entry->offset != 0)
    {
      return ret;
    }

  /* Keep the inode when mmapped, increase refcount */

  ret = inode_addref(filep->f_inode);
  if (ret >= 0)
    {
      object = (FAR struct shmfs_object_s *)filep->f_inode->i_private;
      if (object)
        {
          ret = shmfs_map_object(object, &entry->vaddr);
        }
      else
        {
          ret = -EINVAL;
        }

      if (ret < 0)
        {
          inode_release(filep->f_inode);
        }
      else
        {
          entry->munmap = shmfs_munmap;
          entry->priv.p = (FAR void *)filep->f_inode;
          mm_map_add(entry);
        }
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

      /* Add the virtual memory back to the shared memory pool */

      vm_release_region(get_group_mm(group), vaddr, length);
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
  int ret;

  /* Partial unmap is not supported yet */

  if (start != entry->vaddr || length != entry->length)
    {
      return -EINVAL;
    }

  /* Unmap the virtual memory area from the user's address space */

  ret = shmfs_unmap_area(group, entry->vaddr, entry->length);

  /* Release the shmfs object. The object gets deleted if no-one has
   * reference to it (either mmap or open file) and the object has been
   * unlinked
   */

  if (ret == OK)
    {
      ret = shmfs_release((FAR struct inode *)entry->priv.p);
    }

  /* Remove the mapping. */

  if (ret == OK)
    {
      ret = mm_map_remove(get_group_mm(group), entry);
    }

  return ret;
}
