/****************************************************************************
 * drivers/ramdisk.c
 *
 *   Copyright (C) 2008-2009, 2011 Gregory Nutt. All rights reserved.
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
#include <sys/ioctl.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/drivers/ramdisk.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Helpers for rdflags */

/* User input flags */

#define RDFLAG_USER            (RDFLAG_WRENABLED | RDFLAG_FUNLINK)

#define RDFLAG_IS_WRENABLED(f) (((f) & RDFLAG_WRENABLED) != 0)
#define RDFLAG_IS_FUNLINK(f)   (((f) & RDFLAG_WRENABLED) != 0)

/* Flag set when the RAM disk block driver is unlink */

#define RDFLAG_UNLINK(f)       do { (f) |= RDFLAG_UNLINKED; } while (0)
#define RDFLAG_IS_UNLINKED(f)  (((f) & RDFLAG_UNLINKED) != 0)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rd_struct_s
{
  uint32_t rd_nsectors;         /* Number of sectors on device */
  uint16_t rd_sectsize;         /* The size of one sector */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  uint8_t rd_crefs;             /* Open reference count */
#endif
  uint8_t rd_flags;             /* See RDFLAG_* definitions */
#ifdef CONFIG_FS_WRITABLE
  FAR uint8_t *rd_buffer;       /* RAM disk backup memory */
#else
  FAR const uint8_t *rd_buffer; /* ROM disk backup memory */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static void    rd_destroy(FAR struct rd_struct_s *dev);

static int     rd_open(FAR struct inode *inode);
static int     rd_close(FAR struct inode *inode);
#endif

static ssize_t rd_read(FAR struct inode *inode, FAR unsigned char *buffer,
                 size_t start_sector, unsigned int nsectors);
#ifdef CONFIG_FS_WRITABLE
static ssize_t rd_write(FAR struct inode *inode,
                 FAR const unsigned char *buffer, size_t start_sector,
                 unsigned int nsectors);
#endif
static int     rd_geometry(FAR struct inode *inode,
                 FAR struct geometry *geometry);
static int     rd_ioctl(FAR struct inode *inode, int cmd,
                 unsigned long arg);

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int     rd_unlink(FAR struct inode *inode);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct block_operations g_bops =
{
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  rd_open,     /* open     */
  rd_close,    /* close    */
#else
  0,           /* open     */
  0,           /* close    */
#endif
  rd_read,     /* read     */
#ifdef CONFIG_FS_WRITABLE
  rd_write,    /* write    */
#else
  NULL,        /* write    */
#endif
  rd_geometry, /* geometry */
  rd_ioctl,    /* ioctl    */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  rd_unlink    /* unlink   */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rd_destroy
 *
 * Description:
 *   Free all resources used by the RAM disk
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static void rd_destroy(FAR struct rd_struct_s *dev)
{
  finfo("Destroying RAM disk\n");

  /* We we configured to free the RAM disk memory when unlinked? */

#ifdef CONFIG_FS_WRITABLE
  if (RDFLAG_IS_UNLINKED(dev->rd_flags))
    {
      /* Yes.. do it */

      kmm_free(dev->rd_buffer);
    }
#endif

  /* And free the block driver itself */

  kmm_free(dev);
}
#endif

/****************************************************************************
 * Name: rd_open
 *
 * Description: Open the block device
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int rd_open(FAR struct inode *inode)
{
  FAR struct rd_struct_s *dev;

  DEBUGASSERT(inode && inode->i_private);
  dev = (FAR struct rd_struct_s *)inode->i_private;

  /* Increment the open reference count */

  dev->rd_crefs++;
  DEBUGASSERT(dev->rd_crefs > 0);

  finfo("rd_crefs: %d\n", dev->rd_crefs);
  return OK;
}
#endif

/****************************************************************************
 * Name: rd_close
 *
 * Description: close the block device
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int rd_close(FAR struct inode *inode)
{
  FAR struct rd_struct_s *dev;

  DEBUGASSERT(inode && inode->i_private);
  dev = (FAR struct rd_struct_s *)inode->i_private;

  /* Increment the open reference count */

  DEBUGASSERT(dev->rd_crefs > 0);
  dev->rd_crefs--;
  finfo("rd_crefs: %d\n", dev->rd_crefs);

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  /* Was that the last open reference to the RAM disk? */

  if (dev->rd_crefs == 0)
    {
      /* Yes..  Have we been unlinked? */

      if (RDFLAG_IS_UNLINKED(dev->rd_flags))
        {
          /* Yes.. Release all of the RAM disk resources */

          rd_destroy(dev);
        }
    }
#endif

  return OK;
}
#endif

/****************************************************************************
 * Name: rd_read
 *
 * Description:  Read the specified number of sectors
 *
 ****************************************************************************/

static ssize_t rd_read(FAR struct inode *inode, unsigned char *buffer,
                       size_t start_sector, unsigned int nsectors)
{
  FAR struct rd_struct_s *dev;

  DEBUGASSERT(inode && inode->i_private);
  dev = (FAR struct rd_struct_s *)inode->i_private;

  finfo("sector: %d nsectors: %d sectorsize: %d\n",
        start_sector, dev->rd_sectsize, nsectors);

  if (start_sector < dev->rd_nsectors &&
      start_sector + nsectors <= dev->rd_nsectors)
    {
       finfo("Transfer %d bytes from %p\n",
             nsectors * dev->rd_sectsize,
             &dev->rd_buffer[start_sector * dev->rd_sectsize]);

       memcpy(buffer,
             &dev->rd_buffer[start_sector * dev->rd_sectsize],
             nsectors * dev->rd_sectsize);
      return nsectors;
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: rd_write
 *
 * Description: Write the specified number of sectors
 *
 ****************************************************************************/

#ifdef CONFIG_FS_WRITABLE
static ssize_t rd_write(FAR struct inode *inode, const unsigned char *buffer,
                        size_t start_sector, unsigned int nsectors)
{
  struct rd_struct_s *dev;

  DEBUGASSERT(inode && inode->i_private);
  dev = (struct rd_struct_s *)inode->i_private;

  finfo("sector: %d nsectors: %d sectorsize: %d\n",
        start_sector, dev->rd_sectsize, nsectors);

  if (!RDFLAG_IS_WRENABLED(dev->rd_flags))
    {
      return -EACCES;
    }
  else if (start_sector < dev->rd_nsectors &&
           start_sector + nsectors <= dev->rd_nsectors)
    {
      finfo("Transfer %d bytes to %p\n",
             nsectors * dev->rd_sectsize,
             &dev->rd_buffer[start_sector * dev->rd_sectsize]);

      memcpy(&dev->rd_buffer[start_sector * dev->rd_sectsize],
             buffer,
             nsectors * dev->rd_sectsize);
      return nsectors;
    }

  return -EFBIG;
}
#endif

/****************************************************************************
 * Name: rd_geometry
 *
 * Description: Return device geometry
 *
 ****************************************************************************/

static int rd_geometry(FAR struct inode *inode, struct geometry *geometry)
{
  struct rd_struct_s *dev;

  finfo("Entry\n");

  DEBUGASSERT(inode);
  if (geometry)
    {
      dev = (struct rd_struct_s *)inode->i_private;
      geometry->geo_available     = true;
      geometry->geo_mediachanged  = false;
#ifdef CONFIG_FS_WRITABLE
      geometry->geo_writeenabled  = RDFLAG_IS_WRENABLED(dev->rd_flags);
#else
      geometry->geo_writeenabled  = false;
#endif
      geometry->geo_nsectors      = dev->rd_nsectors;
      geometry->geo_sectorsize    = dev->rd_sectsize;

      finfo("available: true mediachanged: false writeenabled: %s\n",
            geometry->geo_writeenabled ? "true" : "false");
      finfo("nsectors: %d sectorsize: %d\n",
            geometry->geo_nsectors, geometry->geo_sectorsize);

      return OK;
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: rd_ioctl
 *
 * Description:
 *   Return device geometry
 *
 ****************************************************************************/

static int rd_ioctl(FAR struct inode *inode, int cmd, unsigned long arg)
{
  FAR struct rd_struct_s *dev;
  FAR void **ppv = (void**)((uintptr_t)arg);

  finfo("Entry\n");

  /* Only one ioctl command is supported */

  DEBUGASSERT(inode && inode->i_private);
  if (cmd == BIOC_XIPBASE && ppv)
    {
      dev  = (FAR struct rd_struct_s *)inode->i_private;
      *ppv = (FAR void *)dev->rd_buffer;

      finfo("ppv: %p\n", *ppv);
      return OK;
    }

  return -ENOTTY;
}

/****************************************************************************
 * Name: rd_unlink
 *
 * Description:
 *   The block driver has been unlinked.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int rd_unlink(FAR struct inode *inode)
{
  FAR struct rd_struct_s *dev;

  DEBUGASSERT(inode && inode->i_private);
  dev = (FAR struct rd_struct_s *)inode->i_private;

  /* Mark the pipe unlinked */

  RDFLAG_UNLINK(dev->rd_flags);

  /* Are the any open references to the driver? */

  if (dev->rd_crefs == 0)
    {
      /* No... release all resources held by the block driver */

      rd_destroy(dev);
    }

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ramdisk_register or romdisk_register
 *
 * Description:
 *   Non-standard function to register a ramdisk or a romdisk
 *
 * Input Parameters:
 *   minor:         Selects suffix of device named /dev/ramN, N={1,2,3...}
 *   nsectors:      Number of sectors on device
 *   sectize:       The size of one sector
 *   rdflags:       See RDFLAG_* definitions
 *   buffer:        RAM disk backup memory
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_FS_WRITABLE
int ramdisk_register(int minor, FAR uint8_t *buffer, uint32_t nsectors,
                     uint16_t sectsize, uint8_t rdflags)
#else
int romdisk_register(int minor, FAR const uint8_t *buffer, uint32_t nsectors,
                     uint16_t sectsize)
#endif
{
  struct rd_struct_s *dev;
  char devname[16];
  int ret = -ENOMEM;

  finfo("buffer: %p nsectors: %d sectsize: %d\n", buffer, nsectors, sectsize);

  /* Sanity check */

#ifdef CONFIG_DEBUG_FEATURES
  if (minor < 0 || minor > 255 || !buffer || !nsectors || !sectsize)
    {
      return -EINVAL;
    }
#endif

  /* Allocate a ramdisk device structure */

  dev = (struct rd_struct_s *)kmm_zalloc(sizeof(struct rd_struct_s));
  if (dev)
    {
      /* Initialize the ramdisk device structure */

      dev->rd_nsectors     = nsectors;     /* Number of sectors on device */
      dev->rd_sectsize     = sectsize;     /* The size of one sector */
      dev->rd_buffer       = buffer;       /* RAM disk backup memory */

#ifdef CONFIG_FS_WRITABLE
      dev->rd_flags        = rdflags & RDFLAG_USER;
#endif

      /* Create a ramdisk device name */

      snprintf(devname, 16, "/dev/ram%d", minor);

      /* Inode private data is a reference to the ramdisk device structure */

      ret = register_blockdriver(devname, &g_bops, 0, dev);
      if (ret < 0)
        {
          ferr("register_blockdriver failed: %d\n", -ret);
          kmm_free(dev);
        }
    }

  return ret;
}
