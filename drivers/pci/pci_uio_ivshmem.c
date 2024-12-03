/****************************************************************************
 * drivers/pci/pci_uio_ivshmem.c
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

#include <debug.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>

#include <nuttx/fs/fs.h>
#include <nuttx/mm/map.h>
#include <nuttx/pci/pci_ivshmem.h>
#include <nuttx/semaphore.h>
#include <nuttx/spinlock.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define dev_to_udev(dev) \
  ((FAR struct uio_ivshmem_dev_s *)ivshmem_get_driver(dev))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct uio_ivshmem_notify_s;
typedef CODE void (*uio_ivshmem_notify_t)(
  FAR struct uio_ivshmem_notify_s *notify, int32_t newevent);

struct uio_ivshmem_notify_s
{
  struct list_node     node;
  uio_ivshmem_notify_t cb;
};

struct uio_ivshmem_read_s
{
  struct uio_ivshmem_notify_s notify;
  sem_t                       wait;
};

struct uio_ivshmem_poll_s
{
  struct uio_ivshmem_notify_s  notify;
  FAR struct pollfd           *fds;
  FAR void                   **ppriv;
};

struct uio_ivshmem_dev_s
{
  struct ivshmem_driver_s      drv;
  FAR struct ivshmem_device_s *dev;
  char                         name[32];

  spinlock_t                   lock;
  int32_t                      event_count;
  struct list_node             notify_list;
  struct uio_ivshmem_poll_s    polls[CONFIG_PCI_UIO_IVSHMEM_NPOLLWAITERS];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int uio_ivshmem_open(FAR struct file *filep);
static int uio_ivshmem_close(FAR struct file *filep);
static ssize_t uio_ivshmem_read(FAR struct file *filep, FAR char *buffer,
                                size_t buflen);
static ssize_t uio_ivshmem_write(FAR struct file *filep,
                                 FAR const char *buffer, size_t buflen);
static int uio_ivshmem_unmap(FAR struct task_group_s *group,
                             FAR struct mm_map_entry_s *entry,
                             FAR void *start, size_t length);
static int uio_ivshmem_mmap(FAR struct file *filep,
                            FAR struct mm_map_entry_s *map);
static int uio_ivshmem_poll(FAR struct file *filep, FAR struct pollfd *fds,
                            bool setup);

static int uio_ivshmem_probe(FAR struct ivshmem_device_s *dev);
static void uio_ivshmem_remove(FAR struct ivshmem_device_s *dev);

static int uio_ivshmem_interrupt(int irq, FAR void *context, FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_uio_ivshmem_fops =
{
  uio_ivshmem_open,  /* open */
  uio_ivshmem_close, /* close */
  uio_ivshmem_read,  /* read */
  uio_ivshmem_write, /* write */
  NULL,              /* seek */
  NULL,              /* ioctl */
  uio_ivshmem_mmap,  /* mmap */
  NULL,              /* truncate */
  uio_ivshmem_poll,  /* poll */
  NULL,              /* readv */
  NULL               /* writev */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL             /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: uio_ivshmem_add_notify
 ****************************************************************************/

static inline void
uio_ivshmem_add_notify(FAR struct uio_ivshmem_dev_s *dev,
                       FAR struct uio_ivshmem_notify_s *notify)
{
  irqstate_t flags;

  flags = spin_lock_irqsave(&dev->lock);
  list_add_tail(&dev->notify_list, &notify->node);
  spin_unlock_irqrestore(&dev->lock, flags);
}

/****************************************************************************
 * Name: uio_ivshmem_remove_notify
 ****************************************************************************/

static inline void
uio_ivshmem_remove_notify(FAR struct uio_ivshmem_dev_s *dev,
                          FAR struct uio_ivshmem_notify_s *notify)
{
  irqstate_t flags;

  flags = spin_lock_irqsave(&dev->lock);
  list_delete(&notify->node);
  spin_unlock_irqrestore(&dev->lock, flags);
}

/****************************************************************************
 * Name: uio_ivshmem_open
 ****************************************************************************/

static int uio_ivshmem_open(FAR struct file *filep)
{
  FAR struct uio_ivshmem_dev_s *dev;
  irqstate_t flags;

  DEBUGASSERT(filep->f_inode != NULL && filep->f_inode->i_private != NULL);
  dev = filep->f_inode->i_private;

  flags = spin_lock_irqsave(&dev->lock);
  filep->f_priv = (FAR void *)(uintptr_t)dev->event_count;
  spin_unlock_irqrestore(&dev->lock, flags);
  return 0;
}

/****************************************************************************
 * Name: uio_ivshmem_close
 ****************************************************************************/

static int uio_ivshmem_close(FAR struct file *filep)
{
  UNUSED(filep);
  return 0;
}

/****************************************************************************
 * Name: uio_ivshmem_notify_read
 ****************************************************************************/

static void uio_ivshmem_notify_read(FAR struct uio_ivshmem_notify_s *notify,
                                    int32_t newevent)
{
  FAR struct uio_ivshmem_read_s *read =
    (FAR struct uio_ivshmem_read_s *)notify;

  nxsem_post(&read->wait);
}

/****************************************************************************
 * Name: uio_ivshmem_read
 ****************************************************************************/

static ssize_t uio_ivshmem_read(FAR struct file *filep, FAR char *buffer,
                                size_t buflen)
{
  FAR struct uio_ivshmem_dev_s *dev;
  struct uio_ivshmem_read_s read;
  irqstate_t flags;
  int ret = OK;

  if (buflen != sizeof(int32_t))
    {
      return -EINVAL;
    }

  DEBUGASSERT(filep->f_inode != NULL && filep->f_inode->i_private != NULL);
  dev = filep->f_inode->i_private;

  nxsem_init(&read.wait, 0, 0);
  read.notify.cb = uio_ivshmem_notify_read;
  uio_ivshmem_add_notify(dev, &read.notify);

  for (; ; )
    {
      flags = spin_lock_irqsave(&dev->lock);
      if (dev->event_count != (int32_t)(uintptr_t)filep->f_priv)
        {
          filep->f_priv = (FAR void *)(uintptr_t)dev->event_count;
          memcpy(buffer, &dev->event_count, sizeof(int32_t));
          spin_unlock_irqrestore(&dev->lock, flags);
          ret = sizeof(int32_t);
          break;
        }

      spin_unlock_irqrestore(&dev->lock, flags);

      if (filep->f_oflags & O_NONBLOCK)
        {
          ret = -EAGAIN;
          break;
        }

      nxsem_wait_uninterruptible(&read.wait);
    }

  uio_ivshmem_remove_notify(dev, &read.notify);
  nxsem_destroy(&read.wait);
  return ret;
}

/****************************************************************************
 * Name: uio_ivshmem_write
 ****************************************************************************/

static ssize_t uio_ivshmem_write(FAR struct file *filep,
                                 FAR const char *buffer, size_t buflen)
{
  FAR struct uio_ivshmem_dev_s *dev;
  int32_t irq_on;

  DEBUGASSERT(filep->f_inode != NULL && filep->f_inode->i_private != NULL);
  dev = filep->f_inode->i_private;

  if (buflen != sizeof(int32_t))
    {
      return -EINVAL;
    }

  irq_on = *(FAR int32_t *)buffer;
  if (irq_on != 0 && irq_on != 1)
    {
      return -EINVAL;
    }

  return ivshmem_control_irq(dev->dev, irq_on);
}

/****************************************************************************
 * Name: uio_ivshmem_unmap
 ****************************************************************************/

static int uio_ivshmem_unmap(FAR struct task_group_s *group,
                             FAR struct mm_map_entry_s *entry,
                             FAR void *start, size_t length)
{
  off_t offset;
  int ret;

  offset = (uintptr_t)start - (uintptr_t)entry->vaddr;
  if (offset + length < entry->length)
    {
      pcierr("ERROR: Cannot umap without unmapping to the end\n");
      return -ENOSYS;
    }

  /* Okay.. the region is being unmapped to the end.  Make sure the length
   * indicates the unmap length.
   */

  length = entry->length - offset;
  if (length < entry->length)
    {
      pcierr("ERROR: Cannot umap portion of the memory\n");
      return -ENOSYS;
    }

  ret = mm_map_remove(get_group_mm(group), entry);
  if (ret < 0)
    {
      pcierr("ERROR: mm_map_remove failed, ret=%d\n", ret);
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: uio_ivshmem_mmap
 ****************************************************************************/

static int uio_ivshmem_mmap(FAR struct file *filep,
                            FAR struct mm_map_entry_s *map)
{
  FAR struct uio_ivshmem_dev_s *dev;
  size_t shmem_size;
  FAR void *shmem;

  DEBUGASSERT(filep->f_inode != NULL && filep->f_inode->i_private != NULL);
  dev = filep->f_inode->i_private;

  shmem = ivshmem_get_shmem(dev->dev, &shmem_size);

  if (map->offset < 0 || map->offset >= shmem_size ||
      map->length == 0 || map->offset + map->length > shmem_size)
    {
      return -EINVAL;
    }

  map->vaddr = (FAR char *)shmem + map->offset;
  map->priv.p = dev;
  map->munmap = uio_ivshmem_unmap;

  /* Not allow mapped memory overlap */

  if (mm_map_find(get_current_mm(), map->vaddr, map->length) != NULL)
    {
      return -EINVAL;
    }

  return mm_map_add(get_current_mm(), map);
}

/****************************************************************************
 * Name: uio_ivshmem_notify_poll
 ****************************************************************************/

void uio_ivshmem_notify_poll(FAR struct uio_ivshmem_notify_s *notify,
                             int32_t newevent)
{
  FAR struct uio_ivshmem_poll_s *poll =
    (FAR struct uio_ivshmem_poll_s *)notify;

  if (newevent != (int32_t)(uintptr_t)*poll->ppriv)
    {
      poll_notify(&poll->fds, 1, POLLIN | POLLRDNORM);
    }
}

/****************************************************************************
 * Name: uio_ivshmem_poll
 ****************************************************************************/

static int uio_ivshmem_poll(FAR struct file *filep, FAR struct pollfd *fds,
                            bool setup)
{
  FAR struct uio_ivshmem_poll_s *poll;
  FAR struct uio_ivshmem_dev_s *dev;
  irqstate_t flags;
  int i;

  DEBUGASSERT(filep->f_inode != NULL && filep->f_inode->i_private != NULL);
  dev = filep->f_inode->i_private;

  if (setup)
    {
      flags = spin_lock_irqsave(&dev->lock);

      for (i = 0; i < CONFIG_PCI_UIO_IVSHMEM_NPOLLWAITERS; i++)
        {
          poll = &dev->polls[i];
          if (poll->fds == NULL)
            {
              /* Bind the poll structure and this slot */

              poll->fds = fds;
              fds->priv = poll;
              break;
            }
        }

      if (i >= CONFIG_PCI_UIO_IVSHMEM_NPOLLWAITERS)
        {
          spin_unlock_irqrestore(&dev->lock, flags);
          return -EBUSY;
        }

      if (dev->event_count != (int32_t)(uintptr_t)filep->f_priv)
        {
          spin_unlock_irqrestore(&dev->lock, flags);
          poll_notify(&fds, 1, POLLIN | POLLRDNORM);
        }
      else
        {
          spin_unlock_irqrestore(&dev->lock, flags);
        }

      poll->notify.cb = uio_ivshmem_notify_poll;
      poll->ppriv = &filep->f_priv;
      uio_ivshmem_add_notify(dev, &poll->notify);
    }
  else if (fds->priv != NULL)
    {
      poll = fds->priv;

      uio_ivshmem_remove_notify(dev, &poll->notify);

      flags = spin_lock_irqsave(&dev->lock);
      poll->fds = NULL;
      fds->priv = NULL;
      spin_unlock_irqrestore(&dev->lock, flags);
    }

  return 0;
}

/****************************************************************************
 * Name: uio_ivshmem_interrupt
 ****************************************************************************/

static int uio_ivshmem_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct uio_ivshmem_notify_s *notify;
  FAR struct uio_ivshmem_dev_s *dev = arg;
  irqstate_t flags;

  flags = spin_lock_irqsave(&dev->lock);
  dev->event_count++;
  list_for_every_entry(&dev->notify_list, notify,
                       struct uio_ivshmem_notify_s, node)
    {
      notify->cb(notify, dev->event_count);
    }

  spin_unlock_irqrestore(&dev->lock, flags);
  return 0;
}

/****************************************************************************
 * Name: uio_ivshmem_probe
 ****************************************************************************/

static int uio_ivshmem_probe(FAR struct ivshmem_device_s *dev)
{
  FAR struct uio_ivshmem_dev_s *udev = dev_to_udev(dev);
  int ret;

  udev->dev = dev;
  spin_lock_init(&udev->lock);
  list_initialize(&udev->notify_list);

  /* Init the irq and ignore error */

  ivshmem_attach_irq(dev, uio_ivshmem_interrupt, udev);
  ivshmem_control_irq(dev, true);

  snprintf(udev->name, sizeof(udev->name), "/dev/uio%d", udev->drv.id);
  ret = register_driver(udev->name, &g_uio_ivshmem_fops, 0666, udev);
  if (ret < 0)
    {
      pcierr("ERROR: Ivshmem register_driver failed, ret=%d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: uio_ivshmem_remove
 ****************************************************************************/

static void uio_ivshmem_remove(FAR struct ivshmem_device_s *dev)
{
  FAR struct uio_ivshmem_dev_s *udev = dev_to_udev(dev);

  unregister_driver(udev->name);
  ivshmem_detach_irq(dev);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int pci_register_uio_ivshmem_driver(void)
{
  FAR struct uio_ivshmem_dev_s *dev;
  FAR char *start = CONFIG_PCI_UIO_IVSHMEM_IDTABLE;

  do
    {
      dev = kmm_zalloc(sizeof(*dev));
      if (dev == NULL)
        {
          return -ENOMEM;
        }

      dev->drv.id = strtoul(start, &start, 0);
      dev->drv.probe = uio_ivshmem_probe;
      dev->drv.remove = uio_ivshmem_remove;
      if (ivshmem_register_driver(&dev->drv) < 0)
        {
          kmm_free(dev);
        }

      pciinfo("Register ivshmem driver, id=%d\n", dev->drv.id);
    }
  while (*start++ != '\0');

  return 0;
}
