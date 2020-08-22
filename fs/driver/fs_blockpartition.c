/****************************************************************************
 * fs/driver/fs_blockpartition.c
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

#include <errno.h>
#include <sys/mount.h>
#include <sys/stat.h>

#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/kmalloc.h>

#include "driver/driver.h"
#include "inode/inode.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct part_struct_s
{
  FAR struct inode *parent;
  size_t firstsector;
  size_t nsectors;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     part_open(FAR struct inode *inode);
static int     part_close(FAR struct inode *inode);
static ssize_t part_read(FAR struct inode *inode, FAR unsigned char *buffer,
                 size_t start_sector, unsigned int nsectors);
static ssize_t part_write(FAR struct inode *inode,
                 FAR const unsigned char *buffer, size_t start_sector,
                 unsigned int nsectors);
static int     part_geometry(FAR struct inode *inode,
                 FAR struct geometry *geometry);
static int     part_ioctl(FAR struct inode *inode, int cmd,
                 unsigned long arg);
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int     part_unlink(FAR struct inode *inode);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct block_operations g_part_bops =
{
  part_open,     /* open     */
  part_close,    /* close    */
  part_read,     /* read     */
  part_write,    /* write    */
  part_geometry, /* geometry */
  part_ioctl     /* ioctl    */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , part_unlink  /* unlink   */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: part_open
 *
 * Description: Open the block device
 *
 ****************************************************************************/

static int part_open(FAR struct inode *inode)
{
  FAR struct part_struct_s *dev = inode->i_private;
  FAR struct inode *parent = dev->parent;
  int ret = OK;

  /* Open the parent block device */

  if (parent->u.i_bops->open)
    {
      ret = parent->u.i_bops->open(parent);
    }

  return ret;
}

/****************************************************************************
 * Name: part_close
 *
 * Description: close the block device
 *
 ****************************************************************************/

static int part_close(FAR struct inode *inode)
{
  FAR struct part_struct_s *dev = inode->i_private;
  FAR struct inode *parent = dev->parent;
  int ret = OK;

  if (parent->u.i_bops->close)
    {
      ret = parent->u.i_bops->close(parent);
    }

  return ret;
}

/****************************************************************************
 * Name: part_read
 *
 * Description:  Read the specified number of sectors
 *
 ****************************************************************************/

static ssize_t part_read(FAR struct inode *inode, unsigned char *buffer,
                         size_t start_sector, unsigned int nsectors)
{
  FAR struct part_struct_s *dev = inode->i_private;
  FAR struct inode *parent = dev->parent;

  if (start_sector + nsectors > dev->nsectors)
    {
      nsectors = dev->nsectors - start_sector;
    }

  start_sector += dev->firstsector;

  return parent->u.i_bops->read(parent, buffer, start_sector, nsectors);
}

/****************************************************************************
 * Name: part_write
 *
 * Description: Write (or buffer) the specified number of sectors
 *
 ****************************************************************************/

static ssize_t part_write(FAR struct inode *inode,
                          FAR const unsigned char *buffer,
                          size_t start_sector, unsigned int nsectors)
{
  FAR struct part_struct_s *dev = inode->i_private;
  FAR struct inode *parent = dev->parent;

  if (start_sector + nsectors > dev->nsectors)
    {
      nsectors = dev->nsectors - start_sector;
    }

  start_sector += dev->firstsector;

  return parent->u.i_bops->write(parent, buffer, start_sector, nsectors);
}

/****************************************************************************
 * Name: part_geometry
 *
 * Description: Return device geometry
 *
 ****************************************************************************/

static int part_geometry(FAR struct inode *inode, struct geometry *geometry)
{
  FAR struct part_struct_s *dev = inode->i_private;
  FAR struct inode *parent = dev->parent;
  int ret;

  ret = parent->u.i_bops->geometry(parent, geometry);
  if (ret >= 0)
    {
      geometry->geo_nsectors = dev->nsectors;
    }

  return ret;
}

/****************************************************************************
 * Name: part_ioctl
 *
 * Description: Return device geometry
 *
 ****************************************************************************/

static int part_ioctl(FAR struct inode *inode, int cmd, unsigned long arg)
{
  FAR uintptr_t ptr_arg = (uintptr_t)arg;
  FAR struct part_struct_s *dev = inode->i_private;
  FAR struct inode *parent = dev->parent;
  int ret = -ENOTTY;

  if (parent->u.i_bops->ioctl)
    {
      if (cmd == MTDIOC_PROTECT || cmd == MTDIOC_UNPROTECT)
        {
          FAR struct mtd_protect_s *prot =
            (FAR struct mtd_protect_s *)ptr_arg;

          prot->startblock += dev->firstsector;
        }

      ret = parent->u.i_bops->ioctl(parent, cmd, arg);
      if (ret >= 0)
        {
          if (cmd == BIOC_XIPBASE || cmd == MTDIOC_XIPBASE)
            {
              FAR void **base = (FAR void **)ptr_arg;
              struct geometry geo;

              ret = parent->u.i_bops->geometry(parent, &geo);
              if (ret >= 0)
                {
                  *(FAR uint8_t *)base +=
                    dev->firstsector * geo.geo_sectorsize;
                }
            }
          else if (cmd == MTDIOC_GEOMETRY)
            {
              FAR struct mtd_geometry_s *mgeo =
                (FAR struct mtd_geometry_s *)ptr_arg;
              uint32_t blkper = mgeo->erasesize / mgeo->blocksize;

              mgeo->neraseblocks = dev->nsectors / blkper;
            }
        }
    }

  return ret;
}

/****************************************************************************
 * Name: part_unlink
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int part_unlink(FAR struct inode *inode)
{
  FAR struct part_struct_s *dev = inode->i_private;
  FAR struct inode *parent = dev->parent;

  inode_release(parent);
  kmm_free(dev);

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: register_blockpartition
 *
 * Description:
 *   Register a block partition driver inode the pseudo file system.
 *
 * Input Parameters:
 *   partition   - The path to the partition inode
 *   parent      - The path to the parent inode
 *   firstsector - The offset in sectors to the partition
 *   nsectors    - The number of sectors in the partition
 *
 * Returned Value:
 *   Zero on success (with the inode point in 'inode'); A negated errno
 *   value is returned on a failure (all error values returned by
 *   inode_reserve):
 *
 *   EINVAL - 'path' is invalid for this operation
 *   EEXIST - An inode already exists at 'path'
 *   ENOMEM - Failed to allocate in-memory resources for the operation
 *
 ****************************************************************************/

int register_blockpartition(FAR const char *partition,
                            mode_t mode, FAR const char *parent,
                            size_t firstsector, size_t nsectors)
{
  FAR struct part_struct_s *dev;
  int ret;

  /* Allocate a partition device structure */

  dev = kmm_zalloc(sizeof(*dev));
  if (!dev)
    {
      return -ENOMEM;
    }

  dev->firstsector = firstsector;
  dev->nsectors    = nsectors;

  /* Find the block driver */

  if (mode & (S_IWOTH | S_IWGRP | S_IWUSR))
    {
      ret = find_blockdriver(parent, 0, &dev->parent);
    }
  else
    {
      ret = find_blockdriver(parent, MS_RDONLY, &dev->parent);
    }

  if (ret < 0)
    {
      goto errout_free;
    }

  /* Inode private data is a reference to the partition device structure */

  ret = register_blockdriver(partition, &g_part_bops, mode, dev);
  if (ret < 0)
    {
      goto errout_release;
    }

  return OK;

errout_release:
  inode_release(dev->parent);
errout_free:
  kmm_free(dev);
  return ret;
}
