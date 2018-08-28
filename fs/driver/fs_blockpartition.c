/****************************************************************************
 * fs/driver/fs_blockpartition.c
 *
 *   Copyright (C) 2018 Pinecone Inc. All rights reserved.
 *   Author: Xiang Xiao <xiaoxiang@pinecone.net>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
static ssize_t part_read(FAR struct inode *inode, unsigned char *buffer,
                         size_t start_sector, unsigned int nsectors);
#ifdef CONFIG_FS_WRITABLE
static ssize_t part_write(FAR struct inode *inode, const unsigned char *buffer,
                          size_t start_sector, unsigned int nsectors);
#endif
static int     part_geometry(FAR struct inode *inode, struct geometry *geometry);
static int     part_ioctl(FAR struct inode *inode, int cmd, unsigned long arg);

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
#ifdef CONFIG_FS_WRITABLE
  part_write,    /* write    */
#else
  NULL,         /* write    */
#endif
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

#ifdef CONFIG_FS_WRITABLE
static ssize_t part_write(FAR struct inode *inode, const unsigned char *buffer,
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
#endif

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
  FAR struct part_struct_s *dev = inode->i_private;
  FAR struct inode *parent = dev->parent;
  int ret = -ENOTTY;

  if (parent->u.i_bops->ioctl)
    {
      ret = parent->u.i_bops->ioctl(parent, cmd, arg);
      if (ret >= 0 && cmd == BIOC_XIPBASE)
        {
          FAR void **base = (FAR void **)arg;
          struct geometry geo;

          ret = parent->u.i_bops->geometry(parent, &geo);
          if (ret >= 0)
            {
              *base += dev->firstsector * geo.geo_sectorsize;
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

