/****************************************************************************
 * drivers/loop.c
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/fs.h>

/****************************************************************************
 * Private Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct loop_struct_s
{
  uint32       nsectors;     /* Number of sectors on device */
  uint16       sectsize;     /* The size of one sector */
#ifdef CONFIG_FS_WRITABLE
  boolean      writeenabled; /* TRUE: can write to device */
#endif
  int          fd;           /* Descriptor of char device/file */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     loop_open(FAR struct inode *inode);
static int     loop_close(FAR struct inode *inode);
static ssize_t loop_read(FAR struct inode *inode, unsigned char *buffer,
                       size_t start_sector, unsigned int nsectors);
#ifdef CONFIG_FS_WRITABLE
static ssize_t loop_write(FAR struct inode *inode, const unsigned char *buffer,
                        size_t start_sector, unsigned int nsectors);
#endif
static int     loop_geometry(FAR struct inode *inode, struct geometry *geometry);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct block_operations g_bops =
{
  loop_open,     /* open     */
  loop_close,    /* close    */
  loop_read,     /* read     */
#ifdef CONFIG_FS_WRITABLE
  loop_write,    /* write    */
#else
  NULL,          /* write    */
#endif
  loop_geometry, /* geometry */
  NULL           /* ioctl    */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: loop_open
 *
 * Description: Open the block device
 *
 ****************************************************************************/

static int loop_open(FAR struct inode *inode)
{
  return OK;
}

/****************************************************************************
 * Name: loop_closel
 *
 * Description: close the block device
 *
 ****************************************************************************/

static int loop_close(FAR struct inode *inode)
{
  return OK;
}

/****************************************************************************
 * Name: loop_read
 *
 * Description:  Read the specified numer of sectors
 *
 ****************************************************************************/

static ssize_t loop_read(FAR struct inode *inode, unsigned char *buffer,
                       size_t start_sector, unsigned int nsectors)
{
  struct loop_struct_s *dev;
  size_t nbytesread;
  off_t offset;
  int ret;

  DEBUGASSERT(inode && inode->i_private);
  dev = (struct loop_struct_s *)inode->i_private;

  /* Calculate the offset to read the sectors and seek to the position */

  offset = start_sector * dev->sectsize;
  ret = lseek(dev->fd, offset, SEEK_SET);
  if (ret == (off_t)-1)
    {
      dbg("Seek failed for offset=%d: %d\n", (int)offset, errno);
    }

  /* Then read the requested number of sectors from that position */

  do
    {
      nbytesread = read(dev->fd, buffer, nsectors * dev->sectsize);
      if (nbytesread < 0 && errno != EINTR)
        {
          dbg("Read failed: %d\n", errno);
          return -errno;
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
static ssize_t loop_write(FAR struct inode *inode, const unsigned char *buffer,
                        size_t start_sector, unsigned int nsectors)
{
  struct loop_struct_s *dev;
  size_t nbyteswritten;
  off_t offset;
  int ret;

  DEBUGASSERT(inode && inode->i_private);
  dev = (struct loop_struct_s *)inode->i_private;

  /* Calculate the offset to write the sectors and seek to the position */

  offset = start_sector * dev->sectsize;
  ret = lseek(dev->fd, offset, SEEK_SET);
  if (ret == (off_t)-1)
    {
      dbg("Seek failed for offset=%d: %d\n", (int)offset, errno);
    }

  /* Then write the requested number of sectors to that position */

  do
    {
      nbyteswritten = write(dev->fd, buffer, nsectors * dev->sectsize);
      if (nbyteswritten < 0 && errno != EINTR)
        {
          dbg("Write failed: %d\n", errno);
          return -errno;
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

static int loop_geometry(FAR struct inode *inode, struct geometry *geometry)
{
  struct loop_struct_s *dev;

  DEBUGASSERT(inode);
  if (geometry)
    {
      dev = (struct loop_struct_s *)inode->i_private;
      geometry->geo_available     = TRUE;
      geometry->geo_mediachanged  = FALSE;
#ifdef CONFIG_FS_WRITABLE
      geometry->geo_writeenabled  = dev->writeenabled;
#else
      geometry->geo_writeenabled  = FALSE;
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
 *   Setup the loop device so that it exports the file referenced by 'name'
 *   as a block device.
 *
 ****************************************************************************/

int losetup(const char *name, int minor, uint16 sectsize)
{
  struct loop_struct_s *dev;
  struct stat sb;
  char devname[16];
  int ret;

  /* Sanity check */

#ifdef CONFIG_DEBUG
  if (minor < 0 || minor > 255 || !name || !sectsize)
    {
      return -EINVAL;
    }
#endif

  /* Get the size of the file */

  ret = stat(name, &sb);
  if (ret < 0)
    {
      dbg("Failed to stat %s: %d\n", name, errno);
      return -errno;
    }

  /* Check if the file system is big enough for one block */

  if (sb.st_size < sectsize)
    {
      dbg("File is too small for blocksize\n");
      return -ERANGE;
    }

  /* Allocate a loop device structure */

  dev = (struct loop_struct_s *)malloc(sizeof(struct loop_struct_s));
  if (!dev)
    {
      return -ENOMEM;
    }

  /* Open the file.  First try to open the device W/R */

#ifdef CONFIG_FS_WRITABLE
  dev->writeenabled = FALSE; /* Assume failure */

  dev->fd = open(name, O_RDWR);
  if (dev->fd >= 0)
    {
      dev->writeenabled = TRUE; /* Success */
    }
  else
#endif
    {
      /* If that fails, then try to open the device read-only */

      dev->fd = open(name, O_RDWR);
      if (dev->fd < 0)
        {
          dbg("Failed to open %s: %d\n", name, errno);
          free(dev);
          return -errno;
        }
    }

  /* Initialize the remaining fields */

  dev->nsectors     = sb.st_size / sectsize;  /* Number of sectors on device */
  dev->sectsize     = sectsize;               /* The size of one sector */

  /* Create a loop device name */

 snprintf(devname, 16, "/dev/loop%d", minor);

 /* Inode private data is a reference to the loop device stgructure */

 ret = register_blockdriver(devname, &g_bops, 0, dev);
 if (ret < 0)
   {
     fdbg("register_blockdriver failed: %d\n", -ret);
     free(dev);
   }

  return ret;
}
