/****************************************************************************
 * drivers/loop/losetup.c
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

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/mount.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/loop.h>
#include <nuttx/semaphore.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define loop_semgive(d) nxsem_post(&(d)->sem)  /* To match loop_semtake */
#define MAX_OPENCNT     (255)                  /* Limit of uint8_t */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct loop_struct_s
{
  sem_t        sem;          /* For safe read-modify-write operations */
  uint32_t     nsectors;     /* Number of sectors on device */
  off_t        offset;       /* Offset (in bytes) to the first sector */
  uint16_t     sectsize;     /* The size of one sector */
  uint8_t      opencnt;      /* Count of open references to the loop device */
  bool         writeenabled; /* true: can write to device */
  struct file  devfile;      /* File struct of char device/file */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     loop_semtake(FAR struct loop_struct_s *dev);
static int     loop_open(FAR struct inode *inode);
static int     loop_close(FAR struct inode *inode);
static ssize_t loop_read(FAR struct inode *inode, FAR unsigned char *buffer,
                       size_t start_sector, unsigned int nsectors);
static ssize_t loop_write(FAR struct inode *inode,
                          FAR const unsigned char *buffer,
                          size_t start_sector, unsigned int nsectors);
static int     loop_geometry(FAR struct inode *inode,
                             FAR struct geometry *geometry);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct block_operations g_bops =
{
  loop_open,     /* open */
  loop_close,    /* close */
  loop_read,     /* read */
  loop_write,    /* write */
  loop_geometry, /* geometry */
  NULL           /* ioctl */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL         /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: loop_semtake
 ****************************************************************************/

static int loop_semtake(FAR struct loop_struct_s *dev)
{
  return nxsem_wait(&dev->sem);
}

/****************************************************************************
 * Name: loop_open
 *
 * Description: Open the block device
 *
 ****************************************************************************/

static int loop_open(FAR struct inode *inode)
{
  FAR struct loop_struct_s *dev;
  int ret;

  DEBUGASSERT(inode && inode->i_private);
  dev = (FAR struct loop_struct_s *)inode->i_private;

  /* Make sure we have exclusive access to the state structure */

  ret = loop_semtake(dev);
  if (ret == OK)
    {
      if (dev->opencnt == MAX_OPENCNT)
        {
          ret = -EMFILE;
        }
      else
        {
          /* Increment the open count */

          dev->opencnt++;
        }

      loop_semgive(dev);
    }

  return ret;
}

/****************************************************************************
 * Name: loop_close
 *
 * Description: close the block device
 *
 ****************************************************************************/

static int loop_close(FAR struct inode *inode)
{
  FAR struct loop_struct_s *dev;
  int ret;

  DEBUGASSERT(inode && inode->i_private);
  dev = (FAR struct loop_struct_s *)inode->i_private;

  /* Make sure we have exclusive access to the state structure */

  ret = loop_semtake(dev);
  if (ret == OK)
    {
      if (dev->opencnt == 0)
        {
          ret = -EIO;
        }
      else
        {
          /* Decrement the open count */

          dev->opencnt--;
        }

      loop_semgive(dev);
    }

  return ret;
}

/****************************************************************************
 * Name: loop_read
 *
 * Description:  Read the specified number of sectors
 *
 ****************************************************************************/

static ssize_t loop_read(FAR struct inode *inode, FAR unsigned char *buffer,
                         size_t start_sector, unsigned int nsectors)
{
  FAR struct loop_struct_s *dev;
  ssize_t nbytesread;
  off_t offset;
  off_t ret;

  DEBUGASSERT(inode && inode->i_private);
  dev = (FAR struct loop_struct_s *)inode->i_private;

  if (start_sector + nsectors > dev->nsectors)
    {
      ferr("ERROR: Read past end of file\n");
      return -EIO;
    }

  /* Calculate the offset to read the sectors and seek to the position */

  offset = start_sector * dev->sectsize + dev->offset;
  ret = file_seek(&dev->devfile, offset, SEEK_SET);
  if (ret < 0)
    {
      ferr("ERROR: Seek failed for offset=%d: %d\n", (int)offset, (int)ret);
      return -EIO;
    }

  /* Then read the requested number of sectors from that position */

  do
    {
      nbytesread = file_read(&dev->devfile, buffer,
                             nsectors * dev->sectsize);
      if (nbytesread < 0 && nbytesread != -EINTR)
        {
          ferr("ERROR: Read failed: %d\n", nbytesread);
          return (int)nbytesread;
        }
    }
  while (nbytesread < 0);

  /* Return the number of sectors read */

  return nbytesread / dev->sectsize;
}

/****************************************************************************
 * Name: loop_write
 *
 * Description: Write the specified number of sectors
 *
 ****************************************************************************/

static ssize_t loop_write(FAR struct inode *inode,
                          FAR const unsigned char *buffer,
                          size_t start_sector, unsigned int nsectors)
{
  FAR struct loop_struct_s *dev;
  ssize_t nbyteswritten;
  off_t offset;
  off_t ret;

  DEBUGASSERT(inode && inode->i_private);
  dev = (FAR struct loop_struct_s *)inode->i_private;

  /* Calculate the offset to write the sectors and seek to the position */

  offset = start_sector * dev->sectsize + dev->offset;
  ret = file_seek(&dev->devfile, offset, SEEK_SET);
  if (ret < 0)
    {
      ferr("ERROR: Seek failed for offset=%d: %d\n", (int)offset, (int)ret);
    }

  /* Then write the requested number of sectors to that position */

  do
    {
      nbyteswritten = file_write(&dev->devfile, buffer,
                                 nsectors * dev->sectsize);
      if (nbyteswritten < 0 && nbyteswritten != -EINTR)
        {
          ferr("ERROR: nx_write failed: %d\n", nbyteswritten);
          return nbyteswritten;
        }
    }
  while (nbyteswritten < 0);

  /* Return the number of sectors written */

  return nbyteswritten / dev->sectsize;
}

/****************************************************************************
 * Name: loop_geometry
 *
 * Description: Return device geometry
 *
 ****************************************************************************/

static int loop_geometry(FAR struct inode *inode,
                         FAR struct geometry *geometry)
{
  FAR struct loop_struct_s *dev;

  DEBUGASSERT(inode);
  if (geometry)
    {
      dev = (FAR struct loop_struct_s *)inode->i_private;
      geometry->geo_available     = true;
      geometry->geo_mediachanged  = false;
      geometry->geo_writeenabled  = dev->writeenabled;
      geometry->geo_nsectors      = dev->nsectors;
      geometry->geo_sectorsize    = dev->sectsize;
      return OK;
    }

  return -EINVAL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: losetup
 *
 * Description:
 *   Setup the loop device so that it exports the file referenced by
 *   'filename' as a block device.
 *
 ****************************************************************************/

int losetup(FAR const char *devname, FAR const char *filename,
            uint16_t sectsize, off_t offset, bool readonly)
{
  FAR struct loop_struct_s *dev;
  struct stat sb;
  int ret;

  /* Sanity check */

#ifdef CONFIG_DEBUG_FEATURES
  if (devname == NULL || filename == NULL || sectsize == 0)
    {
      return -EINVAL;
    }
#endif

  /* Get the size of the file */

  ret = stat(filename, &sb);
  if (ret < 0)
    {
      ferr("ERROR: Failed to stat %s: %d\n", filename, get_errno());
      return -get_errno();
    }

  /* Check if the file system is big enough for one block */

  if (sb.st_size - offset < sectsize)
    {
      ferr("ERROR: File is too small for blocksize\n");
      return -ERANGE;
    }

  /* Allocate a loop device structure */

  dev = (FAR struct loop_struct_s *)
    kmm_zalloc(sizeof(struct loop_struct_s));
  if (dev == NULL)
    {
      return -ENOMEM;
    }

  /* Initialize the loop device structure. */

  nxsem_init(&dev->sem, 0, 1);
  dev->nsectors  = (sb.st_size - offset) / sectsize;
  dev->sectsize  = sectsize;
  dev->offset    = offset;

  /* Open the file. */

  /* First try to open the device R/W access (unless we are asked
   * to open it readonly).
   */

  ret = -ENOSYS;
  if (!readonly)
    {
      ret = file_open(&dev->devfile, filename, O_RDWR);
    }

  if (ret >= 0)
    {
      dev->writeenabled = true; /* Success */
    }
  else
    {
      /* If that fails, then try to open the device read-only */

      ret = file_open(&dev->devfile, filename, O_RDONLY);
      if (ret < 0)
        {
          ferr("ERROR: Failed to open %s: %d\n", filename, ret);
          goto errout_with_dev;
        }
    }

  /* Inode private data will be reference to the loop device structure */

  ret = register_blockdriver(devname, &g_bops, 0, dev);
  if (ret < 0)
    {
      ferr("ERROR: register_blockdriver failed: %d\n", -ret);
      goto errout_with_file;
    }

  return OK;

errout_with_file:
  file_close(&dev->devfile);

errout_with_dev:
  kmm_free(dev);
  return ret;
}

/****************************************************************************
 * Name: loteardown
 *
 * Description:
 *   Undo the setup performed by losetup
 *
 ****************************************************************************/

int loteardown(FAR const char *devname)
{
  FAR struct loop_struct_s *dev;
  FAR struct inode *inode;
  int ret;

  /* Sanity check */

#ifdef CONFIG_DEBUG_FEATURES
  if (devname == NULL)
    {
      return -EINVAL;
    }
#endif

  /* Open the block driver associated with devname so that we can get the
   * inode reference.
   */

  ret = open_blockdriver(devname, MS_RDONLY, &inode);
  if (ret < 0)
    {
      ferr("ERROR: Failed to open %s: %d\n", devname, -ret);
      return ret;
    }

  /* Inode private data is a reference to the loop device structure */

  dev = (FAR struct loop_struct_s *)inode->i_private;
  close_blockdriver(inode);

  DEBUGASSERT(dev != NULL);

  /* Are there still open references to the device */

  if (dev->opencnt > 0)
    {
      return -EBUSY;
    }

  /* Otherwise, unregister the block device */

  ret = unregister_blockdriver(devname);

  /* Release the device structure */

  if (dev->devfile.f_inode != NULL)
    {
      file_close(&dev->devfile);
    }

  kmm_free(dev);
  return ret;
}
