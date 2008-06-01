/****************************************************************************
 * drivers/ramdisk.c
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include <nuttx/fs.h>
#include <nuttx/ramdisk.h>

/****************************************************************************
 * Private Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rd_struct_s
{
  uint32  rd_nsectors;     /* Number of sectors on device */
  uint16  rd_sectsize;     /* The size of one sector */
  boolean rd_writeenabled; /* TRUE: can write to ram disk */
  ubyte  *rd_buffer;       /* RAM disk backup memory */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     rd_open(FAR struct inode *inode);
static int     rd_close(FAR struct inode *inode);
static ssize_t rd_read(FAR struct inode *inode, unsigned char *buffer,
                       size_t start_sector, unsigned int nsectors);
static ssize_t rd_write(FAR struct inode *inode, const unsigned char *buffer,
                        size_t start_sector, unsigned int nsectors);
static int     rd_geometry(FAR struct inode *inode, struct geometry *geometry);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct block_operations g_bops =
{
  rd_open,     /* open     */
  rd_close,    /* close    */
  rd_read,     /* read     */
  rd_write,    /* write    */
  rd_geometry, /* geometry */
  NULL         /* ioctl    */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rd_open
 *
 * Description: Open the block device
 *
 ****************************************************************************/

static int rd_open(FAR struct inode *inode)
{
  return OK;
}

/****************************************************************************
 * Name: rd_closel
 *
 * Description: close the block device
 *
 ****************************************************************************/

static int rd_close(FAR struct inode *inode)
{
  return OK;
}

/****************************************************************************
 * Name: rd_read
 *
 * Description:  Read the specified numer of sectors
 *
 ****************************************************************************/

static ssize_t rd_read(FAR struct inode *inode, unsigned char *buffer,
                       size_t start_sector, unsigned int nsectors)
{
  struct rd_struct_s *dev;
  if (inode)
    {
      dev = (struct rd_struct_s *)inode->i_private;
      if (dev &&
          start_sector < dev->rd_nsectors &&
          start_sector + nsectors <= dev->rd_nsectors)
        {
          memcpy(buffer,
                 &dev->rd_buffer[start_sector * dev->rd_sectsize],
                 nsectors * dev->rd_sectsize);
          return nsectors;
        }
    }
  return -EINVAL;
}

/****************************************************************************
 * Name: rd_write
 *
 * Description: Write the specified number of sectors
 *
 ****************************************************************************/

static ssize_t rd_write(FAR struct inode *inode, const unsigned char *buffer,
                        size_t start_sector, unsigned int nsectors)
{
  struct rd_struct_s *dev;
  if (inode)
    {
      dev = (struct rd_struct_s *)inode->i_private;
      if (dev &&
          start_sector < dev->rd_nsectors &&
          start_sector + nsectors <= dev->rd_nsectors)
        {
          memcpy(&dev->rd_buffer[start_sector * dev->rd_sectsize],
                 buffer,
                 nsectors * dev->rd_sectsize);
          return nsectors;
        }
    }
  return -EINVAL;
}

/****************************************************************************
 * Name: rd_geometry
 *
 * Description: Return device geometry
 *
 ****************************************************************************/

static int rd_geometry(FAR struct inode *inode, struct geometry *geometry)
{
  struct rd_struct_s *dev;
  if (inode && geometry)
    {
      dev = (struct rd_struct_s *)inode->i_private;
      geometry->geo_available     = TRUE;
      geometry->geo_mediachanged  = FALSE;
      geometry->geo_writeenabled  = dev->rd_writeenabled;
      geometry->geo_nsectors      = dev->rd_nsectors;
      geometry->geo_sectorsize    = dev->rd_sectsize;
      return OK;
    }
  return -EINVAL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rd_register
 *
 * Description: Register the a ramdisk

 ****************************************************************************/

int rd_register(int minor, ubyte *buffer, uint32 nsectors, uint16 sectsize,
                boolean writeenabled)
{
  struct rd_struct_s *dev;
  char devname[16];
  int ret = -ENOMEM;

  /* Sanity check */

#ifdef CONFIG_DEBUG
  if (minor < 0 || minor > 255 || !buffer || !nsectors || !sectsize)
    {
      return -EINVAL;
    }
#endif

  /* Allocate a ramdisk device structure */

  dev = (struct rd_struct_s *)malloc(sizeof(struct rd_struct_s));
  if (dev)
    {
      /* Initialize the ramdisk device structure */

      dev->rd_nsectors     = nsectors;     /* Number of sectors on device */
      dev->rd_sectsize     = sectsize;     /* The size of one sector */
      dev->rd_writeenabled = writeenabled; /* TRUE: can write to ram disk */
      dev->rd_buffer       = buffer;       /* RAM disk backup memory */

      /* Create a ramdisk device name */

      snprintf(devname, 16, "/dev/ram%d", minor);

      /* Inode private data is a reference to the ramdisk device stgructure */

      ret = register_blockdriver(devname, &g_bops, 0, dev);
      if (ret < 0)
        {
          free(dev);
        }
    }
  return ret;
}
