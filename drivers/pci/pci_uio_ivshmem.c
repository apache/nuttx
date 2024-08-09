/****************************************************************************
 * drivers/pci/pci_uio_ivshmem.c
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
#include <stdio.h>

#include <nuttx/fs/fs.h>
#include <nuttx/mm/map.h>
#include <nuttx/pci/pci.h>

#include "pci_drivers.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define UIO_IVSHMEM_SHMEM_BAR   2

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct uio_ivshmem_dev_s
{
  FAR void *shmem;
  size_t    shmem_size;
  char      name[32];
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

static int uio_ivshmem_probe(FAR struct pci_device_s *dev);
static void uio_ivshmem_remove(FAR struct pci_device_s *dev);

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
  NULL               /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL             /* unlink */
#endif
};

static const struct pci_device_id_s g_uio_ivshmem_ids[] =
{
  { PCI_DEVICE(0x1af4, 0x1110) },
  { 0, }
};

static struct pci_driver_s g_uio_ivshmem_drv =
{
  g_uio_ivshmem_ids,  /* PCI id_tables */
  uio_ivshmem_probe,  /* Probe function */
  uio_ivshmem_remove, /* Remove function */
};

static int g_uio_ivshmem_idx = 0;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: uio_ivshmem_open
 ****************************************************************************/

static int uio_ivshmem_open(FAR struct file *filep)
{
  UNUSED(filep);
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
 * Name: uio_ivshmem_read
 ****************************************************************************/

static ssize_t uio_ivshmem_read(FAR struct file *filep, FAR char *buffer,
                                size_t buflen)
{
  UNUSED(filep);
  UNUSED(buffer);
  return buflen;
}

/****************************************************************************
 * Name: uio_ivshmem_write
 ****************************************************************************/

static ssize_t uio_ivshmem_write(FAR struct file *filep,
                                 FAR const char *buffer, size_t buflen)
{
  UNUSED(filep);
  UNUSED(buffer);
  return buflen;
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
  FAR struct uio_ivshmem_dev_s *priv;

  DEBUGASSERT(filep->f_inode != NULL && filep->f_inode->i_private != NULL);

  /* Recover our private data from the struct file instance */

  priv = filep->f_inode->i_private;

  if (map->offset < 0 || map->offset >= priv->shmem_size ||
      map->length == 0 || map->offset + map->length > priv->shmem_size)
    {
      return -EINVAL;
    }

  map->vaddr = (FAR char *)priv->shmem + map->offset;
  map->priv.p = priv;
  map->munmap = uio_ivshmem_unmap;

  /* Not allow mapped memory overlap */

  if (mm_map_find(get_current_mm(), map->vaddr, map->length) != NULL)
    {
      return -EINVAL;
    }

  return mm_map_add(get_current_mm(), map);
}

/****************************************************************************
 * Name: uio_ivshmem_probe
 ****************************************************************************/

static int uio_ivshmem_probe(FAR struct pci_device_s *dev)
{
  FAR struct uio_ivshmem_dev_s *priv;
  int ret;

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  /* Configure the ivshmem device and get share memory address */

  ret = pci_enable_device(dev);
  if (ret < 0)
    {
      pcierr("ERROR: Enable device failed, ret=%d\n", ret);
      goto err_priv;
    }

  pci_set_master(dev);

  priv->shmem = pci_map_bar(dev, UIO_IVSHMEM_SHMEM_BAR);
  if (priv->shmem == NULL)
    {
      ret = -ENOTSUP;
      pcierr("ERROR: Device not support share memory bar\n");
      goto err_master;
    }

  priv->shmem_size = pci_resource_len(dev, UIO_IVSHMEM_SHMEM_BAR);

  pciinfo("shmem addr=%p size=%zu\n", priv->shmem, priv->shmem_size);

  snprintf(priv->name, sizeof(priv->name), "/dev/uio%d", g_uio_ivshmem_idx);
  ret = register_driver(priv->name, &g_uio_ivshmem_fops, 0666, priv);
  if (ret < 0)
    {
      pcierr("ERROR: Ivshmem register_driver failed, ret=%d\n", ret);
      goto err_master;
    }

  g_uio_ivshmem_idx++;
  return ret;

err_master:
  pci_clear_master(dev);
  pci_disable_device(dev);
err_priv:
  kmm_free(priv);
  return ret;
}

/****************************************************************************
 * Name: uio_ivshmem_remove
 ****************************************************************************/

static void uio_ivshmem_remove(FAR struct pci_device_s *dev)
{
  FAR struct uio_ivshmem_dev_s *priv = dev->priv;

  dev->priv = NULL;
  unregister_driver(priv->name);
  pci_clear_master(dev);
  pci_disable_device(dev);
  kmm_free(priv);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int pci_register_uio_ivshmem_driver(void)
{
  return pci_register_driver(&g_uio_ivshmem_drv);
}

