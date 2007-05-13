/****************************************************************************
 * up_blockdevice.c
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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
#include <string.h>
#include <nuttx/fs.h>
#include <errno.h>

#include "up_internal.h"

/****************************************************************************
 * Private Definitions
 ****************************************************************************/

#define NSECTORS            2048
#define LOGICAL_SECTOR_SIZE 512

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     up_open(FAR struct inode *inode);
static int     up_close(FAR struct inode *inode);
static ssize_t up_read(FAR struct inode *inode, unsigned char *buffer,
                       size_t start_sector, size_t nsectors);
static ssize_t up_write(FAR struct inode *inode, const unsigned char *buffer,
                        size_t start_sector, size_t nsectors);
static int     up_geometry(FAR struct inode *inode, struct geometry *geometry);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct block_operations g_bops =
{
  .open     = up_open,
  .close    = up_close,
  .read     = up_read,
  .write    = up_write,
  .geometry = up_geometry,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_open
 *
 * Description: Open the block device
 *
 ****************************************************************************/

static int up_open(FAR struct inode *inode)
{
  return OK;
}

/****************************************************************************
 * Name: up_closel
 *
 * Description: close the block device
 *
 ****************************************************************************/

static int up_close(FAR struct inode *inode)
{
  return OK;
}

/****************************************************************************
 * Name: up_read
 *
 * Description:  Read the specified numer of sectors
 *
 ****************************************************************************/

static ssize_t up_read(FAR struct inode *inode, unsigned char *buffer,
                       size_t start_sector, size_t nsectors)
{
  if (inode)
    {
      char *src = inode->i_private;
      if (src &&
          start_sector < NSECTORS &&
          start_sector + nsectors < NSECTORS)
        {
          memcpy(buffer,
                 &src[start_sector*LOGICAL_SECTOR_SIZE],
                 nsectors*LOGICAL_SECTOR_SIZE);
          return nsectors;
        }
    }
  return -EINVAL;
}

/****************************************************************************
 * Name: up_write
 *
 * Description: Write the specified number of sectors
 *
 ****************************************************************************/

static ssize_t up_write(FAR struct inode *inode, const unsigned char *buffer,
                        size_t start_sector, size_t nsectors)
{
  if (inode)
    {
      char *dest = inode->i_private;
      if (dest &&
          start_sector < NSECTORS &&
          start_sector + nsectors < NSECTORS)
        {
          memcpy(&dest[start_sector*LOGICAL_SECTOR_SIZE],
                 buffer,
                 nsectors*LOGICAL_SECTOR_SIZE);
          return nsectors;
        }
    }
  return -EINVAL;
}

/****************************************************************************
 * Name: up_geometry
 *
 * Description: Return device geometry
 *
 ****************************************************************************/

static int up_geometry(FAR struct inode *inode, struct geometry *geometry)
 {
   if (geometry)
     {
       geometry->geo_available     = (inode->i_private != NULL);
       geometry->geo_mediachanged  = FALSE;
       geometry->geo_writeenabled  = TRUE;
       geometry->geo_nsectors      = NSECTORS;
       geometry->geo_sectorsize    = LOGICAL_SECTOR_SIZE;
       return OK;
     }
   return -EINVAL;
 }

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_registerblockdevice
 *
 * Description: Register the simulated block device

 ****************************************************************************/

void up_registerblockdevice(void)
{
  /* Inode private data is a filesystem image */
  void *priv = (void*)up_deviceimage();
  (void)register_blockdriver("/dev/blkdev", &g_bops, 0, priv);
}
