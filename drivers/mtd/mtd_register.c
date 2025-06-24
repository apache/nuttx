/****************************************************************************
 * drivers/mtd/mtd_register.c
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

#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdlib.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>
#include <string.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdbool.h>
#include <sys/stat.h>

#include <nuttx/mtd/mtd.h>
#include <nuttx/kmalloc.h>
#include <nuttx/fs/ioctl.h>
#ifdef CONFIG_FS_PROCFS
#  include <nuttx/fs/procfs.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mtd_struct_s
{
  FAR struct mtd_dev_s *mtd;  /* MTD layer representing flash partition */
  uint32_t blksize;           /* Size of one write page */
  uint16_t refs;              /* Number of references */
  uint8_t erasestate;         /* Erase state of flash partition */
  size_t size;                /* Size of the partition in bytes */
  mutex_t lock;               /* Lock for the driver access */
  bool readonly;              /* True if the partition supposed to be
                               * read only. This will block write access.
                               */
};

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     mtd_open(FAR struct file *filep);
static int     mtd_close(FAR struct file *filep);
static off_t   mtd_seek(FAR struct file *filep, off_t offset, int whence);
static ssize_t mtd_read(FAR struct file *filep, FAR char *buffer,
                        size_t buflen);
static ssize_t mtd_write(FAR struct file *filep, FAR const char *buffer,
                         size_t buflen);
static int     mtd_ioctl(FAR struct file *filep, int cmd,
                         unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_fops =
{
  mtd_open,     /* open  */
  mtd_close,    /* close */
  mtd_read,     /* read  */
  mtd_write,    /* write */
  mtd_seek,     /* seek  */
  mtd_ioctl     /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mtd_open
 *
 * Description: Open the block device
 *
 ****************************************************************************/

static int mtd_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mtd_struct_s *mtd;
  int ret = OK;

  DEBUGASSERT(inode->i_private);
  mtd = inode->i_private;

  /* Increment the reference count */

  ret = nxmutex_lock(&mtd->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (mtd->refs == 255)
    {
      ret = -EMFILE;
    }
  else
    {
      mtd->refs++;
    }

  nxmutex_unlock(&mtd->lock);
  return ret;
}

/****************************************************************************
 * Name: mtd_close
 *
 * Description: close the block device
 *
 ****************************************************************************/

static int mtd_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mtd_struct_s *mtd;
  int ret = OK;

  DEBUGASSERT(inode->i_private);
  mtd = inode->i_private;

  /* Get exclusive access */

  ret = nxmutex_lock(&mtd->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Decrement the reference count */

  if (mtd->refs == 0)
    {
      ret = -EIO;
    }
  else
    {
      mtd->refs--;
    }

  nxmutex_unlock(&mtd->lock);
  return ret;
}

/****************************************************************************
 * Name: mtd_read
 *
 * Description:  Read the specified number of bytes.
 *
 ****************************************************************************/

static ssize_t mtd_read(FAR struct file *filep, FAR char *buffer, size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mtd_struct_s *dev;
  FAR char *buf;
  off_t startblock;
  off_t offset;
  size_t nblocks;
  ssize_t ret;

  DEBUGASSERT(inode->i_private);
  dev = inode->i_private;

  if (len < 1)
    {
      return 0;
    }

  ret = nxmutex_lock(&dev->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (filep->f_pos >= dev->size)
    {
      /* End of file */

      nxmutex_unlock(&dev->lock);
      return 0;
    }

  ret = MTD_READ(dev->mtd, filep->f_pos, len, (FAR uint8_t *)buffer);
  if (ret < 0)
    {
      if (ret == -ENOSYS)
        {
          /* Byte read not supported, use block read */

          startblock = filep->f_pos / dev->blksize;
          nblocks = (len / dev->blksize) + 1;
          buf = kmm_zalloc(nblocks * dev->blksize);
          if (buf)
            {
              offset = filep->f_pos - (startblock * dev->blksize);
              ret = MTD_BREAD(dev->mtd, startblock, nblocks,
                              (FAR uint8_t *)buffer);
              ret *= dev->blksize;
              if (ret >= offset)
                {
                  memcpy(buffer, buf + offset, len);
                  ret -= offset;
                  filep->f_pos += ret > len ? len : ret;
                }
              else
                {
                  /* We haven't read enough bytes to obtain the desired
                   * offset, return EOF.
                   */

                  ret = 0;
                }

              kmm_free(buf);
            }
          else
            {
              ret = -ENOMEM;
            }
        }
    }
  else
    {
      filep->f_pos += ret;
    }

  nxmutex_unlock(&dev->lock);
  return ret;
}

/****************************************************************************
 * Name: mtd_write
 *
 * Description:  Read the specified number of bytes
 *
 ****************************************************************************/

static ssize_t mtd_write(FAR struct file *filep, FAR const char *buffer,
                         size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mtd_struct_s *dev;
  off_t offset;
  off_t startblock;
  size_t nblocks;
  FAR char *buf;
  ssize_t ret = -EACCES;

  DEBUGASSERT(inode->i_private);
  dev = inode->i_private;

  if (dev->readonly)
    {
      return -EACCES;
    }

  if (len < 1)
    {
      return 0;
    }

  ret = nxmutex_lock(&dev->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (filep->f_pos >= dev->size)
    {
      nxmutex_unlock(&dev->lock);
      return -EFBIG;
    }

#ifdef CONFIG_MTD_BYTE_WRITE
  ret = MTD_WRITE(dev->mtd, filep->f_pos, len, (FAR uint8_t *)buffer);
  if (ret == -ENOSYS)
#endif
    {
      startblock = filep->f_pos / dev->blksize;
      nblocks = (len / dev->blksize) + 1;
      buf = kmm_zalloc(nblocks * dev->blksize);
      if (buf == NULL)
        {
          nxmutex_unlock(&dev->lock);
          return -ENOMEM;
        }

      memset(buf, dev->erasestate, nblocks * dev->blksize);

      offset = filep->f_pos - (startblock * dev->blksize);
      memcpy(buf + offset, buffer, len);

      ret = MTD_BWRITE(dev->mtd, startblock, nblocks, (FAR uint8_t *)buf);
      ret *= dev->blksize;
      if (ret >= offset)
        {
          ret -= offset;
          if (ret > len)
            {
              ret = len;
            }
        }

      kmm_free(buf);
    }

  if (ret > 0)
    {
      filep->f_pos += ret;
    }

  nxmutex_unlock(&dev->lock);
  return ret;
}

/****************************************************************************
 * Name: mtd_seek
 *
 * Description:  Seek to the specific offset.
 *
 ****************************************************************************/

static off_t mtd_seek(FAR struct file *filep, off_t offset, int whence)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mtd_struct_s *dev;
  off_t newpos;
  off_t ret;

  DEBUGASSERT(inode->i_private);

  dev = inode->i_private;
  ret = nxmutex_lock(&dev->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Determine the new, requested file position */

  switch (whence)
    {
    case SEEK_CUR:
      newpos = filep->f_pos + offset;
      break;

    case SEEK_SET:
      newpos = offset;
      break;

    case SEEK_END:
      newpos = (off_t)dev->size + offset;
      break;

    default:

      /* Return EINVAL if the whence argument is invalid */

      nxmutex_unlock(&dev->lock);
      return -EINVAL;
    }

  /* Opengroup.org:
   *
   *  "The lseek() function shall allow the file offset to be set beyond the
   *   end of the existing data in the file. If data is later written at this
   *   point, subsequent reads of data in the gap shall return bytes with the
   *   value 0 until data is actually written into the gap."
   *
   * We can conform to the first part, but not the second. But return -EINVAL
   * if:
   *
   *  "...the resulting file offset would be negative for a regular file,
   *  block special file, or directory."
   */

  if (newpos >= 0)
    {
      filep->f_pos = newpos;
      ret = newpos;
    }
  else
    {
      ret = -EINVAL;
    }

  nxmutex_unlock(&dev->lock);
  return ret;
}

/****************************************************************************
 * Name: mtd_ioctl
 *
 * Description:
 *   Handle IOCTL commands
 *
 ****************************************************************************/

static int mtd_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mtd_struct_s *dev;
  int ret = -ENOTTY;

  DEBUGASSERT(inode->i_private);
  dev = inode->i_private;

  /* Process the call according to the command */

  switch (cmd)
    {
      default:
        {
          /* Currently there are no specific IOCTL calls, let the MTD
           * partition handle the common ones.
           */

          ret = MTD_IOCTL(dev->mtd, cmd, arg);
        }
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int mtd_partition_register(FAR struct mtd_dev_s *mtd, FAR const char *path,
                           bool readonly)
{
  int ret;
  struct mtd_struct_s *dev;
  struct mtd_geometry_s geo;

  if (mtd == NULL)
    {
      return -EINVAL;
    }

  ret = mtd->ioctl(mtd, MTDIOC_GEOMETRY, (unsigned long)(uintptr_t)&geo);
  if (ret < 0)
    {
      ferr("ERROR: MTDIOC_GEOMETRY ioctl failed: %d\n", ret);
      return ret;
    }

  dev = kmm_zalloc(sizeof(struct mtd_struct_s));
  if (dev == NULL)
    {
      return -ENOMEM;
    }

  dev->mtd = mtd;
  ret = mtd->ioctl(mtd, MTDIOC_ERASESTATE,
                   (unsigned long)(uintptr_t)&dev->erasestate);
  if (ret < 0)
    {
      ferr("ERROR: MTDIOC_ERASESTATE ioctl failed: %d\n", ret);
      kmm_free(dev);
      return ret;
    }

  dev->size = geo.erasesize * geo.neraseblocks;
  dev->blksize = geo.blocksize;
  dev->readonly = readonly;
  nxmutex_init(&dev->lock);

  ret = register_driver(path, &g_fops, 0666, (FAR void *)dev);
  if (ret < 0)
    {
      ferr("ERROR: register_driver failed: %d\n", ret);
      kmm_free(dev);
    }

  return ret;
}
