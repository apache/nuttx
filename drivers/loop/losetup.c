/****************************************************************************
 * drivers/loop/losetup.c
 *
 *   Copyright (C) 2008-2009, 2011, 2014-2015, 2017 Gregory Nutt. All
 *     rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
#include <semaphore.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/loop.h>

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
#ifdef CONFIG_FS_WRITABLE
  bool         writeenabled; /* true: can write to device */
#endif
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
#ifdef CONFIG_FS_WRITABLE
static ssize_t loop_write(FAR struct inode *inode,
                          FAR const unsigned char *buffer,
                          size_t start_sector, unsigned int nsectors);
#endif
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
#ifdef CONFIG_FS_WRITABLE
  loop_write,    /* write */
#else
  NULL,          /* write */
#endif
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
  int ret;

  /* Take the semaphore (perhaps waiting) */

  ret = nxsem_wait(&dev->sem);

  /* The only case that an error should occur here is if the wait was
   * awakened by a signal.
   */

  DEBUGASSERT(ret == OK || ret == -EINTR);
  return ret;
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

#ifdef CONFIG_FS_WRITABLE
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
#endif

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
#ifdef CONFIG_FS_WRITABLE
      geometry->geo_writeenabled  = dev->writeenabled;
#else
      geometry->geo_writeenabled  = false;
#endif
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
 *   Setup the loop device so that it exports the file referenced by 'filename'
 *   as a block device.
 *
 ****************************************************************************/

int losetup(FAR const char *devname, FAR const char *filename,
            uint16_t sectsize, off_t offset, bool readonly)
{
  FAR struct loop_struct_s *dev;
  struct stat sb;
  int ret;
  int fd = -1;

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

#ifdef CONFIG_FS_WRITABLE
  /* First try to open the device R/W access (unless we are asked
   * to open it readonly).
   */

  if (!readonly)
    {
      fd = open(filename, O_RDWR);
    }

  if (fd >= 0)
    {
      dev->writeenabled = true; /* Success */
    }
  else
#endif
    {
      /* If that fails, then try to open the device read-only */

      fd = open(filename, O_RDONLY);
      if (fd < 0)
        {
          ret = -get_errno();
          ferr("ERROR: Failed to open %s: %d\n", filename, ret);
          goto errout_with_dev;
        }
    }

  /* Detach the file from the file descriptor */

  ret = file_detach(fd, &dev->devfile);
  if (ret < 0)
    {
      ferr("ERROR: Failed to open %s: %d\n", filename, ret);
      close(fd);
      goto errout_with_dev;
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
  file_close_detached(&dev->devfile);

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

  /* Open the block driver associated with devname so that we can get the inode
   * reference.
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
      (void)file_close_detached(&dev->devfile);
    }

  kmm_free(dev);
  return ret;
}
