/****************************************************************************
 * drivers/mtd/smart.c
 *
 * Sector Mapped Allocation for Really Tiny (SMART) Flash block driver.
 *
 *   Copyright (C) 2013 Ken Pettit. All rights reserved.
 *   Author: Ken Pettit <pettitkd@gmail.com>
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
#include <stddef.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/mtd.h>
#include <nuttx/smart.h>

/****************************************************************************
 * Private Definitions
 ****************************************************************************/

#define SMART_STATUS_COMMITTED    0x80
#define SMART_STATUS_RELEASED     0x40
#define SMART_STATUS_SIZEBITS     0x1C
#define SMART_STATUS_VERBITS      0x03
#define SMART_STATUS_VERSION      0x01

#define SMART_SECTSIZE_256        0x00
#define SMART_SECTSIZE_512        0x04
#define SMART_SECTSIZE_1024       0x08
#define SMART_SECTSIZE_2048       0x0C
#define SMART_SECTSIZE_4096       0x10
#define SMART_SECTSIZE_8192       0x14
#define SMART_SECTSIZE_16384      0x18

#define SMART_FMT_STAT_UNKNOWN    0
#define SMART_FMT_STAT_FORMATTED  1
#define SMART_FMT_STAT_NOFMT      2

#define SMART_FMT_POS1            sizeof(struct smart_sect_header_s)
#define SMART_FMT_POS2            (SMART_FMT_POS1 + 1)
#define SMART_FMT_POS3            (SMART_FMT_POS1 + 2)
#define SMART_FMT_POS4            (SMART_FMT_POS1 + 3)

#define SMART_FMT_SIG1            'S'
#define SMART_FMT_SIG2            'M'
#define SMART_FMT_SIG3            'R'
#define SMART_FMT_SIG4            'T'

#define SMART_FMT_VERSION_POS     (SMART_FMT_POS1 + 4)
#define SMART_FMT_NAMESIZE_POS    (SMART_FMT_POS1 + 5)
#define SMART_FMT_ROOTDIRS_POS    (SMART_FMT_POS1 + 6)
#define SMARTFS_FMT_AGING_POS     32

#define SMART_FMT_VERSION           1

#define SMART_FIRST_ALLOC_SECTOR    12      /* First logical sector number we will
                                             * use for assignment of requested Alloc
                                             * sectors.  All enries below this are
                                             * reserved (some for root dir entries,
                                             * other for our use, such as format
                                             * sector, etc. */

#if defined(CONFIG_FS_READAHEAD) || (defined(CONFIG_FS_WRITABLE) && defined(CONFIG_FS_WRITEBUFFER))
#  define CONFIG_SMART_RWBUFFER 1
#endif

#ifndef CONFIG_MTD_SMART_SECTOR_SIZE
#  define  CONFIG_MTD_SMART_SECTOR_SIZE 1024
#endif

#ifndef offsetof
#define offsetof(type, member) ( (size_t) &( ( (type *) 0)->member))
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct smart_struct_s
{
  FAR struct mtd_dev_s *mtd;              /* Contained MTD interface */
  struct mtd_geometry_s geo;              /* Device geometry */
  uint16_t              neraseblocks;     /* Number of erase blocks or sub-sectors */
  uint16_t              freesectors;      /* Total number of free sectors */
  uint16_t              mtdBlksPerSector; /* Number of MTD blocks per SMART Sector */
  uint16_t              sectorsPerBlk;    /* Number of sectors per erase block */
  uint16_t              sectorsize;       /* Sector size on device */
  uint16_t              totalsectors;     /* Total number of sectors on device */
  FAR uint16_t         *sMap;             /* Virtual to physical sector map */
  FAR uint8_t          *releasecount;     /* Count of released sectors per erase block */
  FAR uint8_t          *freecount;        /* Count of free sectors per erase block */
  FAR char             *rwbuffer;         /* Our sector read/write buffer */
  const FAR char       *partname;         /* Optional partition name */
  uint8_t               formatversion;    /* Format version on the device */
  uint8_t               formatstatus;     /* Indicates the status of the device format */
  uint8_t               namesize;         /* Length of filenames on this device */
#ifdef CONFIG_SMARTFS_MULTI_ROOT_DIRS
  uint8_t               rootdirentries;   /* Number of root directory entries */
  uint8_t               minor;            /* Minor number of the block entry */
#endif
};

#ifdef CONFIG_SMARTFS_MULTI_ROOT_DIRS
struct smart_multiroot_device_s
{
  FAR struct smart_struct_s*    dev;
  uint8_t                       rootdirnum;
};
#endif

struct smart_sect_header_s
{
  uint8_t               logicalsector[2]; /* The logical sector number */
  uint8_t               seq[2];           /* Incrementing sequence number */
  uint8_t               status;           /* Status of this sector:
                                           * Bit 7:   1 = Not commited
                                           *          0 = commited
                                           * Bit 6:   1 = Not released
                                           *          0 = released
                                           * Bit 5:   Reserved - 1
                                           * Bit 4:   Reserved - 1
                                           * Bit 3:   Reserved - 1
                                           * Bit 2:   Reserved - 1
                                           * Bit 1-0: Format version    */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     smart_open(FAR struct inode *inode);
static int     smart_close(FAR struct inode *inode);
static ssize_t smart_reload(struct smart_struct_s *dev, FAR uint8_t *buffer,
                 off_t startblock, size_t nblocks);
static ssize_t smart_read(FAR struct inode *inode, unsigned char *buffer,
                 size_t start_sector, unsigned int nsectors);
#ifdef CONFIG_FS_WRITABLE
static ssize_t smart_write(FAR struct inode *inode, const unsigned char *buffer,
                 size_t start_sector, unsigned int nsectors);
#endif
static int     smart_geometry(FAR struct inode *inode, struct geometry *geometry);
static int     smart_ioctl(FAR struct inode *inode, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct block_operations g_bops =
{
  smart_open,     /* open     */
  smart_close,    /* close    */
  smart_read,     /* read     */
#ifdef CONFIG_FS_WRITABLE
  smart_write,    /* write    */
#else
  NULL,           /* write    */
#endif
  smart_geometry, /* geometry */
  smart_ioctl     /* ioctl    */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: smart_open
 *
 * Description: Open the block device
 *
 ****************************************************************************/

static int smart_open(FAR struct inode *inode)
{
  fvdbg("Entry\n");
  return OK;
}

/****************************************************************************
 * Name: smart_close
 *
 * Description: close the block device
 *
 ****************************************************************************/

static int smart_close(FAR struct inode *inode)
{
  fvdbg("Entry\n");
  return OK;
}

/****************************************************************************
 * Name: smart_reload
 *
 * Description:  Read the specified numer of sectors
 *
 ****************************************************************************/

static ssize_t smart_reload(struct smart_struct_s *dev, FAR uint8_t *buffer,
                            off_t startblock, size_t nblocks)
{
  ssize_t nread;
  ssize_t mtdBlocks, mtdStartBlock;

  /* Calculate the number of MTD blocks to read */

  mtdBlocks = nblocks * dev->mtdBlksPerSector;

  /* Calculate the first MTD block number */

  mtdStartBlock = startblock * dev->mtdBlksPerSector;

  /* Read the full erase block into the buffer */

  fdbg("Read %d blocks starting at block %d\n", mtdBlocks, mtdStartBlock);
  nread   = MTD_BREAD(dev->mtd, mtdStartBlock, mtdBlocks, buffer);
  if (nread != mtdBlocks)
    {
      fdbg("Read %d blocks starting at block %d failed: %d\n",
           nblocks, startblock, nread);
    }

  return nread;
}

/****************************************************************************
 * Name: smart_read
 *
 * Description:  Read the specified numer of sectors
 *
 ****************************************************************************/

static ssize_t smart_read(FAR struct inode *inode, unsigned char *buffer,
                          size_t start_sector, unsigned int nsectors)
{
  struct smart_struct_s *dev;

  fvdbg("SMART: sector: %d nsectors: %d\n", start_sector, nsectors);

  DEBUGASSERT(inode && inode->i_private);
#ifdef CONFIG_SMARTFS_MULTI_ROOT_DIRS
  dev = ((struct smart_multiroot_device_s*) inode->i_private)->dev;
#else
  dev = (struct smart_struct_s *)inode->i_private;
#endif
  return smart_reload(dev, buffer, start_sector, nsectors);
}

/****************************************************************************
 * Name: smart_write
 *
 * Description: Write (or buffer) the specified number of sectors
 *
 ****************************************************************************/

#ifdef CONFIG_FS_WRITABLE
static ssize_t smart_write(FAR struct inode *inode, const unsigned char *buffer,
                           size_t start_sector, unsigned int nsectors)
{
  struct smart_struct_s *dev;
  off_t  alignedblock;
  off_t  mask;
  off_t  blkstowrite;
  off_t  offset;
  off_t  nextblock;
  off_t  mtdBlksPerErase;
  off_t  eraseblock;
  size_t remaining;
  size_t nxfrd;
  int    ret;
  off_t  mtdstartblock, mtdblockcount;

  fvdbg("sector: %d nsectors: %d\n", start_sector, nsectors);

  DEBUGASSERT(inode && inode->i_private);
#ifdef CONFIG_SMARTFS_MULTI_ROOT_DIRS
  dev = ((struct smart_multiroot_device_s*) inode->i_private)->dev;
#else
  dev = (struct smart_struct_s *)inode->i_private;
#endif

  /* I think maybe we need to lock on a mutex here */

  /* Get the aligned block.  Here is is assumed: (1) The number of R/W blocks
   * per erase block is a power of 2, and (2) the erase begins with that same
   * alignment.
   */

  mask         = dev->sectorsPerBlk - 1;
  alignedblock = ((start_sector + mask) & ~mask) * dev->mtdBlksPerSector;

  /* Convert SMART blocks into MTD blocks */

  mtdstartblock = start_sector * dev->mtdBlksPerSector;
  mtdblockcount = nsectors * dev->mtdBlksPerSector;
  mtdBlksPerErase = dev->mtdBlksPerSector * dev->sectorsPerBlk;

  fvdbg("mtdsector: %d mtdnsectors: %d\n", mtdstartblock, mtdblockcount);

  /* Start at first block to be written */

  remaining = mtdblockcount;
  nextblock = mtdstartblock;
  offset = 0;

  /* Loop for all blocks to be written */

  while (remaining > 0)
    {
      /* If this is an aligned block, then erase the block */

      if (alignedblock == nextblock)
        {
          /* Erase the erase block */

          eraseblock = alignedblock / mtdBlksPerErase;
          ret = MTD_ERASE(dev->mtd, eraseblock, 1);
          if (ret < 0)
            {
              fdbg("Erase block=%d failed: %d\n", eraseblock, ret);

              /* Unlock the mutex if we add one */

              return ret;
            }
        }

      /* Calculate the number of blocks to write. */

      blkstowrite = mtdBlksPerErase;
      if (nextblock != alignedblock)
        {
          blkstowrite = alignedblock - nextblock;
        }

      if (blkstowrite > remaining)
        {
          blkstowrite = remaining;
        }

      /* Try to write to the sector. */

      fdbg("Write MTD block %d from offset %d\n", nextblock, offset);
      nxfrd = MTD_BWRITE(dev->mtd, nextblock, blkstowrite, &buffer[offset]);
      if (nxfrd != blkstowrite)
        {
          /* The block is not empty!!  What to do? */

          fdbg("Write block %d failed: %d.\n", nextblock, nxfrd);

          /* Unlock the mutex if we add one */

          return -EIO;
        }

      /* Then update for amount written */

      nextblock += blkstowrite;
      remaining -= blkstowrite;
      offset += blkstowrite * dev->geo.blocksize;
      alignedblock += mtdBlksPerErase;
    }

  return nsectors;
}
#endif /* CONFIG_FS_WRITABLE */

/****************************************************************************
 * Name: smart_geometry
 *
 * Description: Return device geometry
 *
 ****************************************************************************/

static int smart_geometry(FAR struct inode *inode, struct geometry *geometry)
{
  struct smart_struct_s *dev;
  uint32_t  erasesize;

  fvdbg("Entry\n");

  DEBUGASSERT(inode);
  if (geometry)
    {
#ifdef CONFIG_SMARTFS_MULTI_ROOT_DIRS
      dev = ((struct smart_multiroot_device_s*) inode->i_private)->dev;
#else
      dev = (struct smart_struct_s *)inode->i_private;
#endif
      geometry->geo_available     = true;
      geometry->geo_mediachanged  = false;
#ifdef CONFIG_FS_WRITABLE
      geometry->geo_writeenabled  = true;
#else
      geometry->geo_writeenabled  = false;
#endif

      erasesize = dev->geo.erasesize;
      if (erasesize == 0)
        {
          erasesize = 65536;
        }

      geometry->geo_nsectors      = dev->geo.neraseblocks * erasesize /
                                     dev->sectorsize;
      geometry->geo_sectorsize    = dev->sectorsize;

      fvdbg("available: true mediachanged: false writeenabled: %s\n",
            geometry->geo_writeenabled ? "true" : "false");
      fvdbg("nsectors: %d sectorsize: %d\n",
            geometry->geo_nsectors, geometry->geo_sectorsize);

      return OK;
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: smart_setsectorsize
 *
 * Description: Sets the device's sector size and recalculates sector size
 *              dependant variables.
 *
 ****************************************************************************/

static int smart_setsectorsize(struct smart_struct_s *dev, uint16_t size)
{
  uint32_t  erasesize;
  uint32_t  totalsectors;

  /* Validate the size isn't zero so we don't divide by zero below */

  if (size == 0)
    {
      size = CONFIG_MTD_SMART_SECTOR_SIZE;
    }

  erasesize = dev->geo.erasesize;
  dev->neraseblocks = dev->geo.neraseblocks;

  /* Most FLASH devices have erase size of 64K, but geo.erasesize is only
   * 16 bits, so it will be zero
   */

  if (erasesize == 0)
    {
      erasesize = 65536;
    }

  dev->sectorsize = size;
  dev->mtdBlksPerSector = dev->sectorsize / dev->geo.blocksize;
  dev->sectorsPerBlk = erasesize / dev->sectorsize;

  /* Release any existing rwbuffer and sMap */

  if (dev->sMap != NULL)
    {
      kfree(dev->sMap);
    }

  if (dev->rwbuffer != NULL)
    {
      kfree(dev->rwbuffer);
    }

  /* Allocate a virtual to physical sector map buffer.  Also allocate
   * the storage space for releasecount and freecounts.
   */

  totalsectors = dev->neraseblocks * dev->sectorsPerBlk;
  dev->totalsectors = (uint16_t) totalsectors;

  dev->sMap = (uint16_t *) kmalloc(totalsectors * sizeof(uint16_t) +
              (dev->neraseblocks << 1));
  if (!dev->sMap)
    {
      fdbg("Error allocating SMART virtual map buffer\n");
      kfree(dev);
      return -EINVAL;
    }

  dev->releasecount = (uint8_t *) dev->sMap + (totalsectors * sizeof(uint16_t));
  dev->freecount = dev->releasecount + dev->neraseblocks;

  /* Allocate a read/write buffer */

  dev->rwbuffer = (char *) kmalloc(size);
  if (!dev->rwbuffer)
    {
      fdbg("Error allocating SMART read/write buffer\n");
      kfree(dev->sMap);
      kfree(dev);
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: smart_bytewrite
 *
 * Description: Writes a non-page size count of bytes to the underlying
 *              MTD device.  If the MTD driver supports a direct impl of
 *              write, then it uses it, otherwise it does a read-modify-write
 *              and depends on the architecture of the flash to only program
 *              bits that acutally changed.
 *
 ****************************************************************************/

static ssize_t smart_bytewrite(struct smart_struct_s *dev, size_t offset,
        int nbytes, const uint8_t *buffer)
{
  ssize_t       ret;

#ifdef CONFIG_MTD_BYTE_WRITE
  /* Check if the underlying MTD device supports write */

  if (dev->mtd->write != NULL)
    {
      /* Use the MTD's write method to write individual bytes */

      ret = dev->mtd->write(dev->mtd, offset, nbytes, buffer);
    }
  else
#endif
    {
      /* Perform block-based read-modify-write */

      uint16_t  startblock;
      uint16_t  nblocks;

      /* First calculate the start block and number of blocks affected */

      startblock = offset / dev->geo.blocksize;
      nblocks = (nbytes + dev->geo.blocksize-1) / dev->geo.blocksize;
      DEBUGASSERT(nblocks <= dev->mtdBlksPerSector);

      /* Do a block read */

      ret = MTD_BREAD(dev->mtd, startblock, nblocks, (uint8_t *) dev->rwbuffer);
      if (ret < 0)
        {
          fdbg("Error %d reading from device\n", -ret);
          goto errout;
        }

      /* Modify the data */

      memcpy(&dev->rwbuffer[offset - startblock * dev->geo.blocksize], buffer, nbytes);

      /* Write the data back to the device */

      ret = MTD_BWRITE(dev->mtd, startblock, nblocks, (uint8_t *) dev->rwbuffer);
      if (ret < 0)
        {
          fdbg("Error %d writing to device\n", -ret);
          goto errout;
        }
    }

  ret = nbytes;

errout:
  return ret;
}

/****************************************************************************
 * Name: smart_scan
 *
 * Description: Performs a scan of the MTD device searching for format
 *              information and fills in logical sector mapping, freesector
 *              count, etc.
 *
 ****************************************************************************/

static int smart_scan(struct smart_struct_s *dev)
{
  int       sector;
  int       ret;
  int       offset;
  uint16_t  totalsectors;
  uint16_t  sectorsize;
  uint16_t  logicalsector;
  uint16_t  seq1;
  uint16_t  seq2;
  size_t    readaddress;
  struct    smart_sect_header_s header;
#ifdef CONFIG_SMARTFS_MULTI_ROOT_DIRS
  int       x;
  char      devname[22];
  struct    smart_multiroot_device_s *rootdirdev;
#endif

  fvdbg("Entry\n");

  /* Read the 1st header from the device.  We always keep the
   * 1st sector's header's sectorsize field accurate, even
   * after we erase an MTD block/sector */

  ret = MTD_READ(dev->mtd, 0, sizeof(struct smart_sect_header_s),
                 (uint8_t *) &header);
  if (ret != sizeof(struct smart_sect_header_s))
    {
      goto err_out;
    }

  /* Now set the sectorsize and other sectorsize derived variables */

  if (header.status == CONFIG_SMARTFS_ERASEDSTATE)
    {
      sectorsize = CONFIG_MTD_SMART_SECTOR_SIZE;
    }
  else
    {
      sectorsize = (header.status & SMART_STATUS_SIZEBITS) << 7;
    }

  ret = smart_setsectorsize(dev, sectorsize);
  if (ret != OK)
    {
      goto err_out;
    }

  /* Initialize the device variables */

  totalsectors = dev->neraseblocks * dev->sectorsPerBlk;
  dev->formatstatus = SMART_FMT_STAT_NOFMT;
  dev->freesectors = totalsectors;

  /* Initialize the freecount and releasecount arrays */

  for (sector = 0; sector < dev->neraseblocks; sector++)
    {
      dev->freecount[sector] = dev->sectorsPerBlk;
      dev->releasecount[sector] = 0;
    }

  /* Initialize the sector map */

  for (sector = 0; sector < totalsectors; sector++)
    {
      dev->sMap[sector] = -1;
    }

  /* Now scan the MTD device */

  for (sector = 0; sector < totalsectors; sector++)
    {
      fvdbg("Scan sector %d\n", sector);

      /* Calculate the read address for this sector */

      readaddress = sector * dev->mtdBlksPerSector * dev->geo.blocksize;

      /* Read the header for this sector */

      ret = MTD_READ(dev->mtd, readaddress, sizeof(struct smart_sect_header_s),
                     (uint8_t *) &header);
      if (ret != sizeof(struct smart_sect_header_s))
        {
          goto err_out;
        }

      /* Get the logical sector number for this physical sector */

      logicalsector = *((uint16_t *) header.logicalsector);
#if CONFIG_SMARTFS_ERASEDSTATE == 0x00
      if (logicalsector == 0)
        {
          logicalsector = -1;
        }
#endif

      /* Test if this sector has been committed */

      if ((header.status & SMART_STATUS_COMMITTED) ==
              (CONFIG_SMARTFS_ERASEDSTATE & SMART_STATUS_COMMITTED))
        {
          continue;
        }

      /* This block is commited, therefore not free.  Update the
       * erase block's freecount.
       */

      dev->freecount[sector / dev->sectorsPerBlk]--;
      dev->freesectors--;

      /* Test if this sector has been release and if it has,
       * update the erase block's releasecount.
       */

      if ((header.status & SMART_STATUS_RELEASED) !=
              (CONFIG_SMARTFS_ERASEDSTATE & SMART_STATUS_RELEASED))
        {
          dev->releasecount[sector / dev->sectorsPerBlk]++;
          continue;
        }

      if ((header.status & SMART_STATUS_VERBITS) != SMART_STATUS_VERSION)
        {
          continue;
        }

      /* Validate the logical sector number is in bounds */

      if (logicalsector >= totalsectors)
        {
          /* Error in logical sector read from the MTD device */

          fdbg("Invalid logical sector %d at physical %d.\n",
               logicalsector, sector);
          continue;
        }

      /* If this is logical sector zero, then read in the signature
       * information to validate the format signature.
       */

      if (logicalsector == 0)
        {
          /* Read the sector data */

          ret = MTD_READ(dev->mtd, readaddress, 32,
                         (uint8_t*) dev->rwbuffer);
          if (ret != 32)
            {
              fdbg("Error reading physical sector %d.\n", sector);
              goto err_out;
            }

          /* Validate the format signature */

          if (dev->rwbuffer[SMART_FMT_POS1] != SMART_FMT_SIG1 ||
              dev->rwbuffer[SMART_FMT_POS2] != SMART_FMT_SIG2 ||
              dev->rwbuffer[SMART_FMT_POS3] != SMART_FMT_SIG3 ||
              dev->rwbuffer[SMART_FMT_POS4] != SMART_FMT_SIG4)
           {
             /* Invalid signature on a sector claiming to be sector 0!
              * What should we do?  Release it?*/

             continue;
           }

          /* TODO: May want to validate / save the erase block aging info */

          /* Mark the volume as formatted and set the sector size */

          dev->formatstatus = SMART_FMT_STAT_FORMATTED;
          dev->namesize = dev->rwbuffer[SMART_FMT_NAMESIZE_POS];
          dev->formatversion = dev->rwbuffer[SMART_FMT_VERSION_POS];

#ifdef CONFIG_SMARTFS_MULTI_ROOT_DIRS
          dev->rootdirentries = dev->rwbuffer[SMART_FMT_ROOTDIRS_POS];

          /* If rootdirentries is greater than 1, then we need to register
           * additional block devices.
           */

          for (x = 1; x < dev->rootdirentries; x++)
            {
              if (dev->partname != NULL)
                {
                  snprintf(dev->rwbuffer, sizeof(devname), "/dev/smart%d%s%d",
                          dev->minor, dev->partname, x+1);
                }
              else
                {
                  snprintf(devname, sizeof(devname), "/dev/smart%dd%d", dev->minor,
                           x + 1);
                }

              /* Inode private data is a reference to a struct containing
               * the SMART device structure and the root directory number.
               */

              rootdirdev = (struct smart_multiroot_device_s*) kmalloc(sizeof(*rootdirdev));
              if (rootdirdev == NULL)
                {
                  fdbg("Memory alloc failed\n");
                  ret = -ENOMEM;
                  goto err_out;
                }

              /* Populate the rootdirdev */

              rootdirdev->dev = dev;
              rootdirdev->rootdirnum = x;
              ret = register_blockdriver(dev->rwbuffer, &g_bops, 0, rootdirdev);

              /* Inode private data is a reference to the SMART device structure */

              ret = register_blockdriver(devname, &g_bops, 0, rootdirdev);
            }
#endif
        }

      /* Test for duplicate logical sectors on the device */

      if (dev->sMap[logicalsector] != 0xFFFF)
        {
          /* Uh-oh, we found more than 1 physical sector claiming to be
           * the * same logical sector.  Use the sequence number information
           * to resolve who wins.
           */

          uint16_t loser;

          seq2 = *((uint16_t *) header.seq);

          /* We must re-read the 1st physical sector to get it's seq number */

          readaddress = dev->sMap[logicalsector]  * dev->mtdBlksPerSector * dev->geo.blocksize;
          ret = MTD_READ(dev->mtd, readaddress, sizeof(struct smart_sect_header_s),
                  (uint8_t *) &header);
          if (ret != sizeof(struct smart_sect_header_s))
            {
              goto err_out;
            }

          seq1 = *((uint16_t *) header.seq);

          /* Now determine who wins */

          if (seq1 > 0xFFF0 && seq2 < 10)
            {
              /* Seq 2 is the winner ... we assume it wrapped */

              loser = dev->sMap[logicalsector];
              dev->sMap[logicalsector] = sector;
            }
          else if (seq2 > seq1)
            {
              /* Seq 2 is bigger, so it's the winner */

              loser = dev->sMap[logicalsector];
              dev->sMap[logicalsector] = sector;
            }
          else
            {
              /* We keep the original mapping and seq2 is the loser */

              loser = sector;
            }

          /* Now release the loser sector */

          readaddress = loser  * dev->mtdBlksPerSector * dev->geo.blocksize;
          ret = MTD_READ(dev->mtd, readaddress, sizeof(struct smart_sect_header_s),
                  (uint8_t *) &header);
          if (ret != sizeof(struct smart_sect_header_s))
            {
              goto err_out;
            }

#if CONFIG_SMARTFS_ERASEDSTATE == 0xFF
          header.status &= ~SMART_STATUS_RELEASED;
#else
          header.status |= SMART_STATUS_RELEASED;
#endif
          offset = readaddress + offsetof(struct smart_sect_header_s, status);
          ret = smart_bytewrite(dev, offset, 1, &header.status);
          if (ret < 0)
            {
              fdbg("Error %d releasing duplicate sector\n", -ret);
              goto err_out;
            }
        }

      /* Update the logical to physical sector map */

      dev->sMap[logicalsector] = sector;
    }

  fdbg("SMART Scan\n");
  fdbg("   Erase size:   %10d\n", dev->sectorsPerBlk * dev->sectorsize);
  fdbg("   Erase count:  %10d\n", dev->neraseblocks);
  fdbg("   Sect/block:   %10d\n", dev->sectorsPerBlk);
  fdbg("   MTD Blk/Sect: %10d\n", dev->mtdBlksPerSector);

  ret = OK;

err_out:
  return ret;
}

/****************************************************************************
 * Name: smart_getformat
 *
 * Description:  Populates the SMART format structure based on the format
 *               information for the inode.
 *
 ****************************************************************************/

#ifdef CONFIG_SMARTFS_MULTI_ROOT_DIRS
static inline int smart_getformat(struct smart_struct_s *dev,
                                  struct smart_format_s *fmt,
                                  uint8_t rootdirnum)
#else
static inline int smart_getformat(struct smart_struct_s *dev,
                                  struct smart_format_s *fmt)
#endif
{
  int ret;
  int x;

  fvdbg("Entry\n");
  DEBUGASSERT(fmt);

  /* Test if we know the format status or not.  If we don't know the
   * status, then we must perform a scan of the device to search
   * for the format marker
   */

  if (dev->formatstatus != SMART_FMT_STAT_FORMATTED)
    {
      /* Perform the scan */

      ret = smart_scan(dev);

      if (ret != OK)
        {
          goto err_out;
        }
    }

  /* Now fill in the structure */

  if (dev->formatstatus == SMART_FMT_STAT_FORMATTED)
    {
      fmt->flags = SMART_FMT_ISFORMATTED;
    }
  else
    {
      fmt->flags = 0;
    }

  fmt->sectorsize = dev->sectorsize;
  fmt->availbytes = dev->sectorsize - sizeof(struct smart_sect_header_s);
  fmt->nsectors = dev->neraseblocks * dev->sectorsPerBlk;
  fmt->nfreesectors = dev->freesectors;
  fmt->namesize = dev->namesize;
#ifdef CONFIG_SMARTFS_MULTI_ROOT_DIRS
  fmt->nrootdirentries = dev->rootdirentries;
  fmt->rootdirnum = rootdirnum;
#endif

  /* Add the released sectors to the reported free sector count */

  for (x = 0; x < dev->neraseblocks; x++)
    {
      fmt->nfreesectors += dev->releasecount[x];
    }

  /* Subtract the reserved sector count */

  fmt->nfreesectors -= dev->sectorsPerBlk + 4;

  ret = OK;

err_out:
  return ret;
}

/****************************************************************************
 * Name: smart_llformat
 *
 * Description:  Performs a low-level format of the flash device.  This
 *               involves erasing the device and writing a valid sector
 *               zero (logical) with proper format signature.
 *
 ****************************************************************************/

#ifdef CONFIG_FS_WRITABLE
static inline int smart_llformat(struct smart_struct_s *dev, unsigned long arg)
{
  struct    smart_sect_header_s  *sectorheader;
  size_t    wrcount;
  size_t    totalsectors;
  int       x;
  int       ret;
  uint8_t   sectsize;

  fvdbg("Entry\n");

  /* Erase the MTD device */

  ret = MTD_IOCTL(dev->mtd, MTDIOC_BULKERASE, 0);
  if (ret < 0)
    {
      return ret;
    }

  /* Now construct a logical sector zero header to write to the device.
   * We fill it with zero so when we add sector aging, all the sector
   * ages will already be initialized to zero without needing special
   * logic to deal with a 0xFF erased-state value.
   */

  sectorheader = (struct smart_sect_header_s *) dev->rwbuffer;
  memset(dev->rwbuffer, 0, dev->sectorsize);
  memset(dev->rwbuffer, CONFIG_SMARTFS_ERASEDSTATE, SMARTFS_FMT_AGING_POS);
  *((uint16_t *) sectorheader->seq) = 0;

  sectsize = (CONFIG_MTD_SMART_SECTOR_SIZE >> 9) << 2;
#if ( CONFIG_SMARTFS_ERASEDSTATE == 0xFF )
  *((uint16_t *) sectorheader->logicalsector) = 0;
  sectorheader->status = (uint8_t) ~(SMART_STATUS_COMMITTED | SMART_STATUS_VERBITS |
          SMART_STATUS_SIZEBITS) | SMART_STATUS_VERSION |
          sectsize;
#else
  *((uint16_t *) sectorheader->logicalsector) = 0xFFFF;
  sectorheader->status = (uint8_t) (SMART_STATUS_COMMITTED | SMART_STATUS_VERSION |
          sectsize);
#endif

  /* Now add the format signature to the sector */

  dev->rwbuffer[SMART_FMT_POS1] = SMART_FMT_SIG1;
  dev->rwbuffer[SMART_FMT_POS2] = SMART_FMT_SIG2;
  dev->rwbuffer[SMART_FMT_POS3] = SMART_FMT_SIG3;
  dev->rwbuffer[SMART_FMT_POS4] = SMART_FMT_SIG4;

  dev->rwbuffer[SMART_FMT_VERSION_POS] = SMART_FMT_VERSION;
  dev->rwbuffer[SMART_FMT_NAMESIZE_POS] = CONFIG_SMARTFS_MAXNAMLEN;

  /* Record the number of root directory entries we have */

  dev->rwbuffer[SMART_FMT_ROOTDIRS_POS] = (uint8_t) arg;

  /* Write the sector to the flash */

  wrcount = MTD_BWRITE(dev->mtd, 0, dev->mtdBlksPerSector,
          (uint8_t *) dev->rwbuffer);
  if (wrcount != dev->mtdBlksPerSector)
    {
      /* The block is not empty!!  What to do? */

      fdbg("Write block 0 failed: %d.\n", wrcount);

      /* Unlock the mutex if we add one */

      return -EIO;
    }

  /* Now initialize our internal control variables */

  ret = smart_setsectorsize(dev, CONFIG_MTD_SMART_SECTOR_SIZE);
  if (ret != OK)
    {
      return ret;
    }

  dev->formatstatus = SMART_FMT_STAT_UNKNOWN;
  dev->freesectors = dev->neraseblocks * dev->sectorsPerBlk - 1;
  for (x = 0; x < dev->neraseblocks; x++)
    {
      /* Initialize the released and free counts */

      dev->releasecount[x] = 0;
      dev->freecount[x] = dev->sectorsPerBlk;
    }

  /* Account for the format sector */

  dev->freecount[0]--;

  /* Now initialize the logical to physical sector map */

  dev->sMap[0] = 0;     /* Logical sector zero = physical sector 0 */

  totalsectors = dev->neraseblocks * dev->sectorsPerBlk;
  for (x = 1; x < totalsectors; x++)
    {
      /* Mark all other logical sectors as non-existant */

      dev->sMap[x] = -1;
    }

#ifdef CONFIG_SMARTFS_MULTI_ROOT_DIRS

  /* Un-register any extra directory device entries */

  for (x = 2; x < 8; x++)
    {
      snprintf(dev->rwbuffer, 18, "/dev/smart%dd%d", dev->minor, x);
      unregister_blockdriver(dev->rwbuffer);
    }
#endif

  return OK;
}
#endif /* CONFIG_FS_WRITABLE */

/****************************************************************************
 * Name: smart_findfreephyssector
 *
 * Description:  Finds a free physical sector based on free and released
 *               count logic, taking into account reserved sectors.
 *
 ****************************************************************************/

static int smart_findfreephyssector(struct smart_struct_s *dev)
{
  uint16_t  allocfreecount;
  uint16_t  allocblock;
  uint16_t  physicalsector;
  uint16_t  x;
  uint32_t  readaddr;
  struct    smart_sect_header_s header;
  int       ret;

  /* Determine which erase block we should allocate the new
   * sector from. This is based on the number of free sectors
   * available in each erase block. */

  allocfreecount = 0;
  allocblock = 0xFFFF;
  physicalsector = 0xFFFF;
  for (x = 0; x < dev->neraseblocks; x++)
    {
      /* Test if this block has more free blocks than the
       * currently selected block */

      if (dev->freecount[x] > allocfreecount)
        {
          /* Assign this block to alloc from */

          allocblock = x;
          allocfreecount = dev->freecount[x];
        }
    }

  /* Check if we found an allocblock. */

  if (allocblock == 0xFFFF)
    {
      /* No free sectors found!  Bug? */
      return -EIO;
    }

  /* Now find a free physical sector within this selected
   * erase block to allocate. */

  for (x = allocblock * dev->sectorsPerBlk;
          x < (allocblock+1) * dev->sectorsPerBlk; x++)
    {
      /* Check if this physical sector is available */

      readaddr = x * dev->mtdBlksPerSector * dev->geo.blocksize;
      ret = MTD_READ(dev->mtd, readaddr, sizeof(struct smart_sect_header_s),
              (uint8_t *) &header);
      if (ret != sizeof(struct smart_sect_header_s))
        {
          fvdbg("Error reading phys sector %d\n", physicalsector);
          return -EIO;
        }

      if ((*((uint16_t *) header.logicalsector) == 0xFFFF) &&
          (*((uint16_t *) header.seq) == 0xFFFF) &&
          ((header.status & SMART_STATUS_COMMITTED) ==
           (CONFIG_SMARTFS_ERASEDSTATE & SMART_STATUS_COMMITTED)))
        {
          physicalsector = x;
          break;
        }
    }

  return physicalsector;
}

/****************************************************************************
 * Name: smart_garbagecollect
 *
 * Description:  Performs garbage collection if needed.  This is determined
 *               by the count of released sectors relative to free and
 *               total sectors.
 *
 ****************************************************************************/

#ifdef CONFIG_FS_WRITABLE
static int smart_garbagecollect(struct smart_struct_s *dev)
{
  uint16_t  releasedsectors;
  uint16_t  collectblock;
  uint16_t  releasemax;
  uint16_t  newsector;
  bool      collect = TRUE;
  int       x;
  int       ret;
  size_t    offset;
  struct    smart_sect_header_s *header;
  uint8_t   newstatus;

  while (collect)
    {
      collect = FALSE;

      /* Calculate the number of released sectors on the device */

      releasedsectors = 0;
      collectblock = 0xFFFF;
      releasemax = 0;
      for (x = 0; x < dev->neraseblocks; x++)
        {
          releasedsectors += dev->releasecount[x];
          if (dev->releasecount[x] > releasemax)
            {
              releasemax = dev->releasecount[x];
              collectblock = x;
            }
        }

      /* Test if the released sectors count is greater than the
       * free sectors.  If it is, then we will do garbage collection.
       */

      if (releasedsectors > dev->freesectors)
        collect = TRUE;

      /* Test if we have more reached our reserved free sector limit */

      if (dev->freesectors <= (dev->sectorsPerBlk << 0) + 4)
        collect = TRUE;

      /* Test if we need to garbage collect */

      if (collect)
        {
          if (collectblock == 0xFFFF)
            {
              /* Need to collect, but no sectors with released blocks! */

              ret = -ENOSPC;
              goto errout;
            }

          fdbg("Collecting block %d, free=%d released=%d\n",
              collectblock, dev->freecount[collectblock],
              dev->releasecount[collectblock]);

          if (dev->freecount[collectblock] == 6)
            {
              fdbg("here!\n");
            }

          /* Perform collection on block with the most released sectors.
           * First mark the block as having no free sectors so we don't
           * try to move sectors into the block we are trying to erase.
           */

          dev->freecount[collectblock] = 0;

          /* Next move all live data in the block to a new home. */

          for (x = collectblock * dev->sectorsPerBlk; x <
             (collectblock + 1) * dev->sectorsPerBlk; x++)
            {
              /* Read the next sector from this erase block */

              ret = MTD_BREAD(dev->mtd, x * dev->mtdBlksPerSector,
                  dev->mtdBlksPerSector, (uint8_t *) dev->rwbuffer);
              if (ret != dev->mtdBlksPerSector)
                {
                  fdbg("Error reading sector %d\n", x);
                  ret = -EIO;
                  goto errout;
                }

              /* Test if if the block is in use */

              header = (struct smart_sect_header_s *) dev->rwbuffer;
              if (((header->status & SMART_STATUS_COMMITTED) ==
                  (CONFIG_SMARTFS_ERASEDSTATE & SMART_STATUS_COMMITTED)) ||
                  ((header->status & SMART_STATUS_RELEASED) !=
                   (CONFIG_SMARTFS_ERASEDSTATE & SMART_STATUS_RELEASED)))
                {
                  /* This sector doesn't have live data (free or released).
                   * just continue to the next sector and don't move it.
                   */

                  continue;
                }

              /* Find a new sector where it can live, NOT in this erase block */

              newsector = smart_findfreephyssector(dev);
              if (newsector == 0xFFFF)
                {
                  /* Unable to find a free sector!!! */

                  fdbg("Can't find a free sector for relocation\n");
                  ret = -EIO;
                  goto errout;
                }

              /* Increment the sequence number and clear the "commit" flag */

              (*((uint16_t *) header->seq))++;
              if (*((uint16_t *) header->seq) == 0xFFFF)
                {
                  *((uint16_t *) header->seq) = 1;
                }
#if CONFIG_SMARTFS_ERASEDSTATE == 0xFF
              header->status |= SMART_STATUS_COMMITTED;
#else
              header->status &= ~SMART_STATUS_COMMITTED;
#endif

              /* Write the data to the new physical sector location */

              ret = MTD_BWRITE(dev->mtd, newsector * dev->mtdBlksPerSector,
                               dev->mtdBlksPerSector, (uint8_t *) dev->rwbuffer);

              /* Commit the sector */

              offset = newsector * dev->mtdBlksPerSector * dev->geo.blocksize +
                  offsetof(struct smart_sect_header_s, status);
#if CONFIG_SMARTFS_ERASEDSTATE == 0xFF
              newstatus = header->status & ~SMART_STATUS_COMMITTED;
#else
              newstatus = header->status | SMART_STATUS_COMMITTED;
#endif
              ret = smart_bytewrite(dev, offset, 1, &newstatus);
              if (ret < 0)
                {
                  fdbg("Error %d committing new sector %d\n" -ret, newsector);
                  goto errout;
                }

              /* Release the old physical sector */

#if CONFIG_SMARTFS_ERASEDSTATE == 0xFF
              newstatus = header->status & ~SMART_STATUS_RELEASED;
#else
              newstatus = header->status | SMART_STATUS_RELEASED;
#endif
              offset = x * dev->mtdBlksPerSector * dev->geo.blocksize +
                  offsetof(struct smart_sect_header_s, status);
              ret = smart_bytewrite(dev, offset, 1, &newstatus);
              if (ret < 0)
                {
                  fdbg("Error %d releasing old sector %d\n" -ret, x);
                  goto errout;
                }

              /* Update the variables */

              dev->sMap[*((uint16_t *) header->logicalsector)] = newsector;
              dev->freecount[newsector / dev->sectorsPerBlk]--;
            }

          /* Now erase the erase block */

          MTD_ERASE(dev->mtd, collectblock, 1);

          dev->freesectors += dev->releasecount[collectblock];
          dev->freecount[collectblock] = dev->sectorsPerBlk;
          dev->releasecount[collectblock] = 0;

          /* If this is block zero, then be sure to write the sector size */

          if (collectblock == 0)
            {
              /* Set the sector size in the 1st header */

              uint8_t sectsize = dev->sectorsize >> 7;
#if ( CONFIG_SMARTFS_ERASEDSTATE == 0xFF )
              newstatus = (uint8_t) ~SMART_STATUS_SIZEBITS | sectsize;
#else
              newstatus = (uint8_t) sectsize;
#endif
              /* Write the sector size to the device */

              offset = offsetof(struct smart_sect_header_s, status);
              ret = smart_bytewrite(dev, offset, 1, &newstatus);
              if (ret < 0)
                {
                  fdbg("Error %d setting sector 0 size\n", -ret);
                }
            }

          /* Update the block aging information in the format signature sector */
        }
      else
        {
          /* Test for aging sectors and push them to a new location
           * so we wear evenly.
           */
        }
    }

  return OK;

errout:
  return ret;
}
#endif /* CONFIG_FS_WRITABLE */

/****************************************************************************
 * Name: smart_writesector
 *
 * Description:  Writes data to the specified logical sector.  The sector
 *               should have already been allocated prior to the write.  If
 *               the logical sector already has data on the device, it will
 *               be released and a new physical sector will be created and
 *               mapped to the logical sector.
 *
 ****************************************************************************/

#ifdef CONFIG_FS_WRITABLE
static inline int smart_writesector(struct smart_struct_s *dev, unsigned long arg)
{
  int       ret;
  uint16_t  x;
  bool      needsrelocate = FALSE;
  uint16_t  mtdblock;
  uint16_t  physsector;
  struct    smart_read_write_s *req;
  struct    smart_sect_header_s *header;
  size_t    offset;
  uint8_t   byte;

  fvdbg("Entry\n");
  req = (struct smart_read_write_s *) arg;
  DEBUGASSERT(req->offset <= dev->sectorsize);
  DEBUGASSERT(req->offset+req->count <= dev->sectorsize);

  /* Ensure the logical sector has been allocated */

  if (req->logsector >= dev->totalsectors)
    {
      fdbg("Logical sector %d too large\n", req->logsector);

      ret = -EINVAL;
      goto errout;
    }

  physsector = dev->sMap[req->logsector];
  if (physsector == 0xFFFF)
    {
      fdbg("Logical sector %d not allocated\n", req->logsector);
      ret = -EINVAL;
      goto errout;
    }

  /* Read the sector data into our buffer */

  mtdblock = physsector * dev->mtdBlksPerSector;
  ret = MTD_BREAD(dev->mtd, mtdblock, dev->mtdBlksPerSector, (uint8_t *)
          dev->rwbuffer);
  if (ret != dev->mtdBlksPerSector)
    {
      fdbg("Error reading phys sector %d\n", physsector);
      ret = -EIO;
      goto errout;
    }

  /* Test if we need to relocate the sector to perform the write */

  for (x = 0; x < req->count; x++)
    {
      /* Test if the next byte can be written to the flash */

      byte = dev->rwbuffer[sizeof(struct smart_sect_header_s) + req->offset + x];
#if CONFIG_SMARTFS_ERASEDSTATE == 0xFF
      if (((byte ^ req->buffer[x]) | byte) != byte)
        {
          needsrelocate = TRUE;
          break;
        }
#else
      if (((byte ^ req->buffer[x]) | req->buffer[x]) != req->buffer[x])
        {
          needsrelocate = TRUE;
          break;
        }
#endif
    }

  if (needsrelocate)
    {
      /* Find a new physical sector to save data to */

      physsector = smart_findfreephyssector(dev);
      if (physsector == 0xFFFF)
        {
          fdbg("Error relocating sector %d\n", req->logsector);
          ret = -EIO;
          goto errout;
        }

      /* Update the sequence number to indicate the sector was moved */

      header = (struct smart_sect_header_s *) dev->rwbuffer;
      (*((uint16_t *) header->seq))++;
#if CONFIG_SMARTFS_ERASEDSTATE == 0xFF
      header->status |= SMART_STATUS_COMMITTED;
#else
      header->status &= SMART_STATUS_COMMITTED;
#endif
    }

  /* Now copy the data to the sector buffer. */

  memcpy(&dev->rwbuffer[sizeof(struct smart_sect_header_s) + req->offset],
          req->buffer, req->count);

  /* Now write the sector buffer to the device. */

  if (needsrelocate)
    {
      /* Write the entire sector to the new physical location, uncommitted. */

      ret = MTD_BWRITE(dev->mtd, physsector * dev->mtdBlksPerSector,
              dev->mtdBlksPerSector, (uint8_t *) dev->rwbuffer);
      if (ret != dev->mtdBlksPerSector)
        {
          fdbg("Error writing to physical sector %d\n", physsector);
          ret = -EIO;
          goto errout;
        }

      /* Commit the new physical sector */

#if CONFIG_SMARTFS_ERASEDSTATE == 0xFF
      byte = header->status & ~SMART_STATUS_COMMITTED;
#else
      byte = header->status | SMART_STATUS_COMMITTED;
#endif
      offset = physsector * dev->mtdBlksPerSector * dev->geo.blocksize +
          offsetof(struct smart_sect_header_s, status);
      ret = smart_bytewrite(dev, offset, 1, &byte);
      if (ret != 1)
        {
          fvdbg("Error committing physical sector %d\n", physsector);
          ret = -EIO;
          goto errout;
        }

      /* Release the old physical sector */

#if CONFIG_SMARTFS_ERASEDSTATE == 0xFF
      byte = header->status & ~SMART_STATUS_RELEASED;
#else
      byte = header->status | SMART_STATUS_RELEASED;
#endif
      offset = mtdblock * dev->geo.blocksize +
          offsetof(struct smart_sect_header_s, status);
      ret = smart_bytewrite(dev, offset, 1, &byte);

      /* Update releasecount for released sector and freecount for the
       * newly allocated physical sector. */

      dev->releasecount[dev->sMap[req->logsector] / dev->sectorsPerBlk]++;
      dev->freecount[physsector / dev->sectorsPerBlk]--;
      dev->freesectors--;

      /* Update the sector map */

      dev->sMap[req->logsector] = physsector;

      /* Since we performed a relocation, do garbage collection to
       * ensure we don't fill up our flash with released blocks.
       */

      smart_garbagecollect(dev);
    }
  else
    {
      /* Not relocated.  Just write the portion of the sector that needs
       * to be written. */

      offset = mtdblock * dev->geo.blocksize +
          sizeof(struct smart_sect_header_s) + req->offset;
      ret = smart_bytewrite(dev, offset, req->count, req->buffer);
    }

  ret = OK;

errout:
  return ret;
}
#endif /* CONFIG_FS_WRITABLE */

/****************************************************************************
 * Name: smart_readsector
 *
 * Description:  Reads data from the specified logical sector.  The sector
 *               should have already been allocated prior to the read.
 *
 ****************************************************************************/

static inline int smart_readsector(struct smart_struct_s *dev, unsigned long arg)
{
  int       ret;
  uint32_t  readaddr;
  uint16_t  physsector;
  struct smart_read_write_s *req;
  struct smart_sect_header_s header;

  fvdbg("Entry\n");
  req = (struct smart_read_write_s *) arg;
  DEBUGASSERT(req->offset < dev->sectorsize);
  DEBUGASSERT(req->offset+req->count < dev->sectorsize);

  /* Ensure the logical sector has been allocated */

  if (req->logsector >= dev->totalsectors)
    {
      fdbg("Logical sector %d too large\n", req->logsector);

      ret = -EINVAL;
      goto errout;
    }

  physsector = dev->sMap[req->logsector];
  if (physsector == 0xFFFF)
    {
      fdbg("Logical sector %d not allocated\n", req->logsector);
      ret = -EINVAL;
      goto errout;
    }

  /* Read the sector header data to validate as a sanity check */

  ret = MTD_READ(dev->mtd, physsector * dev->mtdBlksPerSector * dev->geo.blocksize,
          sizeof(struct smart_sect_header_s), (uint8_t *) &header);
  if (ret != sizeof(struct smart_sect_header_s))
    {
      fvdbg("Error reading sector %d header\n", physsector);
      ret = -EIO;
      goto errout;
    }

  /* Do a sanity check on the header data */

  if (((*(uint16_t *) header.logicalsector) != req->logsector) ||
      ((header.status & SMART_STATUS_COMMITTED) ==
       (CONFIG_SMARTFS_ERASEDSTATE & SMART_STATUS_COMMITTED)))
    {
      /* Error in sector header! How do we handle this? */

      fdbg("Error in logical sector %d header, phys=%d\n",
          req->logsector, physsector);
      ret = -EIO;
      goto errout;
    }

  /* Read the sector data into the buffer */

  readaddr = (uint32_t) physsector * dev->mtdBlksPerSector * dev->geo.blocksize +
    req->offset + sizeof(struct smart_sect_header_s);;

  ret = MTD_READ(dev->mtd, readaddr, req->count, (uint8_t *)
          req->buffer);
  if (ret != req->count)
    {
      fdbg("Error reading phys sector %d\n", physsector);
      ret = -EIO;
      goto errout;
    }

errout:
    return ret;
}

/****************************************************************************
 * Name: smart_allocsector
 *
 * Description:  Allocates a new logical sector.  If an argument is given,
 *               then it tries to allocate the specified sector number.
 *
 ****************************************************************************/

#ifdef CONFIG_FS_WRITABLE
static inline int smart_allocsector(struct smart_struct_s *dev, unsigned long requested)
{
  int       x;
  int       ret;
  uint16_t  logsector = 0xFFFF; /* Logical sector number selected */
  uint16_t  physicalsector;     /* The selected physical sector */
  uint16_t  releasecount;
  struct    smart_sect_header_s  *header;
  uint8_t   sectsize;

  /* Validate that we have enough sectors available to perform an
   * allocation.  We have to ensure we keep enough reserved sectors
   * on hand to do released sector garbage collection. */

  releasecount = 0;
  for (x = 0; x < dev->neraseblocks; x++)
    {
      releasecount += dev->releasecount[x];
    }

  if (dev->freesectors <= (dev->sectorsPerBlk << 0) + 4)
    {
      /* We are at our free sector limit.  Test if we have
       * sectors we can release */

      if (releasecount == 0)
        {
          /* No space left!! */

          return -ENOSPC;
        }
    }

  /* Check if a specific sector is being requested and allocate that
   * sector if it isn't already in use */

  if ((requested > 2) && (requested < dev->totalsectors))
    {
      /* Validate the sector is not already allocated */

      if (dev->sMap[requested] == (uint16_t) -1)
        {
          logsector = requested;
        }
    }

  /* Check if we need to scan for an available logical sector */

  if (logsector == 0xFFFF)
    {
      /* Loop through all sectors and find one to allocate */

      for (x = SMART_FIRST_ALLOC_SECTOR; x < dev->totalsectors; x++)
        {
          if (dev->sMap[x] == (uint16_t) -1)
            {
              /* Unused logical sector found.  Use this one */

              logsector = x;
              break;
            }
        }
    }

  /* Test for an error allocating a sector */

  if (logsector == 0xFFFF)
    {
      /* Hmmm.  We think we had enough logical sectors, but
       * something happened and we didn't find any free
       * logical sectors.  What do do?  Report an error?
       * rescan and try again to "self heal" in case of a
       * bug in our code? */

      fdbg("No free logical sector numbers!  Free sectors = %d\n",
              dev->freesectors);

      return -EIO;
    }

  /* Check if we need to do garbage collection.  We have to
   * ensure we keep enough reserved free sectors to per garbage
   * collection as it involves moving sectors from blocks with
   * released sectors into blocks with free sectors, then
   * erasing the vacated block. */

  smart_garbagecollect(dev);

  /* Find a free physical sector */

  physicalsector = smart_findfreephyssector(dev);
  fvdbg("Alloc: log=%d, phys=%d, erase block=%d, free=%d, released=%d\n",
          logsector, physicalsector, physicalsector /
          dev->sectorsPerBlk, dev->freesectors, releasecount);

  /* Create a header to assign the logical sector */

  memset(dev->rwbuffer, CONFIG_SMARTFS_ERASEDSTATE, dev->sectorsize);
  header = (struct smart_sect_header_s *) dev->rwbuffer;
  *((uint16_t *) header->logicalsector) = logsector;
  *((uint16_t *) header->seq) = 0;
  sectsize = dev->sectorsize >> 7;

#if CONFIG_SMARTFS_ERASEDSTATE == 0xFF
  header->status = ~(SMART_STATUS_COMMITTED | SMART_STATUS_SIZEBITS |
          SMART_STATUS_VERBITS) | SMART_STATUS_VERSION | sectsize;
#else
  header->status = SMART_STATUS_COMMITTED | SMART_STATUS_VERSION | sectsize;
#endif

  /* Write the header to the physical sector location */

  x = physicalsector * dev->mtdBlksPerSector;

  fvdbg("Write MTD block %d\n", x);
  ret = MTD_BWRITE(dev->mtd, x, 1, (uint8_t *) dev->rwbuffer);
  if (ret != 1)
    {
      /* The block is not empty!!  What to do? */

      fdbg("Write block %d failed: %d.\n", x, ret);

      /* Unlock the mutex if we add one */

      return -EIO;
    }

  /* Map the sector and update the free sector counts */

  dev->sMap[logsector] = physicalsector;
  dev->freecount[physicalsector / dev->sectorsPerBlk]--;
  dev->freesectors--;

  /* Return the logical sector number */

  return logsector;
}
#endif /* CONFIG_FS_WRITABLE */

/****************************************************************************
 * Name: smart_freesector
 *
 * Description:  Frees a logical sector from the device.  Freeing (also
 *               called releasing) is performed by programming the released
 *               bit in the sector header's status byte.
 *
 ****************************************************************************/

#ifdef CONFIG_FS_WRITABLE
static inline int smart_freesector(struct smart_struct_s *dev, unsigned long
        logicalsector)
{
  int       ret;
  int       readaddr;
  uint16_t  physsector;
  uint16_t  block;
  struct    smart_sect_header_s  header;
  size_t    offset;

  /* Check if the logical sector is within bounds */

  if ((logicalsector > 2) && (logicalsector < dev->totalsectors))
    {
      /* Validate the sector is actually allocated */

      if (dev->sMap[logicalsector] == (uint16_t) -1)
        {
          fdbg("Invalid release - sector %d not allocated\n", logicalsector);
          ret = -EINVAL;
          goto errout;
        }
    }

  /* Okay to release the sector.  Read the sector header info */

  physsector = dev->sMap[logicalsector];
  readaddr = physsector * dev->mtdBlksPerSector * dev->geo.blocksize;
  ret = MTD_READ(dev->mtd, readaddr, sizeof(struct smart_sect_header_s),
                 (uint8_t *) &header);
  if (ret != sizeof(struct smart_sect_header_s))
    {
      goto errout;
    }

  /* Do a sanity check on the logical sector number */

  if (*((uint16_t *) header.logicalsector) != (uint16_t) logicalsector)
    {
      /* Hmmm... something is wrong.  This should always match!  Bug in our code? */

      fdbg("Sector %d logical sector in header doesn't match\n", logicalsector);
      ret = -EINVAL;
      goto errout;
    }

  /* Mark the sector as released */

#if CONFIG_SMARTFS_ERASEDSTATE == 0xFF
  header.status &= ~SMART_STATUS_RELEASED;
#else
  header.status |= SMART_STATUS_RELEASED;
#endif

  /* Write the status back to the device */

  offset = readaddr + offsetof(struct smart_sect_header_s, status);
  ret = smart_bytewrite(dev, offset, 1, &header.status);
  if (ret != 1)
    {
      fdbg("Error updating physicl sector %d status\n", physsector);
      goto errout;
    }

  /* Update the erase block's release count */

  block = physsector / dev->sectorsPerBlk;
  dev->releasecount[block]++;

  /* Unmap this logical sector */

  dev->sMap[logicalsector] = (uint16_t) -1;

  /* If this block has only released blocks, then erase it */

  if (dev->releasecount[block] + dev->freecount[block] == dev->sectorsPerBlk)
    {
      /* Erase the block */

      MTD_ERASE(dev->mtd, block, 1);

      dev->freesectors += dev->releasecount[block];
      dev->releasecount[block] = 0;
      dev->freecount[block] = dev->sectorsPerBlk;
    }

  ret = OK;

errout:
  return ret;
}
#endif /* CONFIG_FS_WRITABLE */

/****************************************************************************
 * Name: smart_ioctl
 *
 * Description: Return device geometry
 *
 ****************************************************************************/

static int smart_ioctl(FAR struct inode *inode, int cmd, unsigned long arg)
{
  struct smart_struct_s *dev ;
  int ret;

  fvdbg("Entry\n");
  DEBUGASSERT(inode && inode->i_private);

#ifdef CONFIG_SMARTFS_MULTI_ROOT_DIRS
  dev = ((struct smart_multiroot_device_s*) inode->i_private)->dev;
#else
  dev = (struct smart_struct_s *)inode->i_private;
#endif

  /* Process the ioctl's we care about first, pass any we don't respond
   * to directly to the underlying MTD device.
   */

  switch (cmd)
    {
    case BIOC_XIPBASE:
      /* The argument accompanying the BIOC_XIPBASE should be non-NULL.  If
       * DEBUG is enabled, we will catch it here instead of in the MTD
       * driver.
       */

#ifdef CONFIG_DEBUG
      if (arg == 0)
        {
          fdbg("ERROR: BIOC_XIPBASE argument is NULL\n");
          return -EINVAL;
        }
#endif

      /* Just change the BIOC_XIPBASE command to the MTDIOC_XIPBASE command. */

      cmd = MTDIOC_XIPBASE;
      break;

    case BIOC_GETFORMAT:

      /* Return the format information for the device */

#ifdef CONFIG_SMARTFS_MULTI_ROOT_DIRS
      ret = smart_getformat(dev, (struct smart_format_s *) arg,
        ((struct smart_multiroot_device_s*) inode->i_private)->rootdirnum);
#else
      ret = smart_getformat(dev, (struct smart_format_s *) arg);
#endif
      goto ok_out;

    case BIOC_READSECT:

      /* Do a logical sector read and return the data */
      ret = smart_readsector(dev, arg);
      goto ok_out;

#ifdef CONFIG_FS_WRITABLE
    case BIOC_LLFORMAT:

      /* Perform a low-level format on the flash */

      ret = smart_llformat(dev, arg);
      goto ok_out;

    case BIOC_ALLOCSECT:

      /* Allocate a logical sector for the upper layer file system */

      ret = smart_allocsector(dev, arg);
      goto ok_out;

    case BIOC_FREESECT:

      /* Free the specified logical sector */

      ret = smart_freesector(dev, arg);
      goto ok_out;

    case BIOC_WRITESECT:

      /* Write to the sector */

      ret = smart_writesector(dev, arg);
      goto ok_out;
#endif /* CONFIG_FS_WRITABLE */

    }

  /* No other block driver ioctl commmands are not recognized by this
   * driver.  Other possible MTD driver ioctl commands are passed through
   * to the MTD driver (unchanged).
   */

  ret = MTD_IOCTL(dev->mtd, cmd, arg);
  if (ret < 0)
    {
      fdbg("ERROR: MTD ioctl(%04x) failed: %d\n", cmd, ret);
    }

ok_out:
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: smart_initialize
 *
 * Description:
 *   Initialize to provide a block driver wrapper around an MTD interface
 *
 * Input Parameters:
 *   minor - The minor device number.  The MTD block device will be
 *      registered as as /dev/smartN where N is the minor number.
 *   mtd - The MTD device that supports the FLASH interface.
 *
 ****************************************************************************/

int smart_initialize(int minor, FAR struct mtd_dev_s *mtd, const char *partname)
{
  struct smart_struct_s *dev;
  int ret = -ENOMEM;
  uint32_t  totalsectors;
#ifdef CONFIG_SMARTFS_MULTI_ROOT_DIRS
  struct smart_multiroot_device_s *rootdirdev;
#endif

  /* Sanity check */

#ifdef CONFIG_DEBUG
  if (minor < 0 || minor > 255 || !mtd)
    {
      return -EINVAL;
    }
#endif

  /* Allocate a SMART device structure */

  dev = (struct smart_struct_s *)kmalloc(sizeof(struct smart_struct_s));
  if (dev)
    {
      /* Initialize the SMART device structure */

      dev->mtd = mtd;

      /* Get the device geometry. (casting to uintptr_t first eliminates
       * complaints on some architectures where the sizeof long is different
       * from the size of a pointer).
       */

      /* Set these to zero in case the device doesn't support them */

      ret = MTD_IOCTL(mtd, MTDIOC_GEOMETRY, (unsigned long)((uintptr_t)&dev->geo));
      if (ret < 0)
        {
          fdbg("MTD ioctl(MTDIOC_GEOMETRY) failed: %d\n", ret);
          kfree(dev);
          goto errout;
        }

      /* Set the sector size to the default for now */

      dev->sMap = NULL;
      dev->rwbuffer = NULL;
      ret = smart_setsectorsize(dev, CONFIG_MTD_SMART_SECTOR_SIZE);
      if (ret != OK)
        {
          kfree(dev);
          goto errout;
        }

      /* Calculate the totalsectors on this device and validate */

      totalsectors = dev->neraseblocks * dev->sectorsPerBlk;
      if (totalsectors > 65534)
        {
          fdbg("SMART Sector size too small for device\n");
          kfree(dev);
          ret = -EINVAL;
          goto errout;
        }
      dev->freesectors = (uint16_t) totalsectors;

      /* Mark the device format status an unknown */

      dev->formatstatus = SMART_FMT_STAT_UNKNOWN;
      dev->namesize = CONFIG_SMARTFS_MAXNAMLEN;
      dev->partname = partname;
#ifdef CONFIG_SMARTFS_MULTI_ROOT_DIRS
      dev->minor = minor;
#endif

      /* Create a MTD block device name */

#ifdef CONFIG_SMARTFS_MULTI_ROOT_DIRS
      if (partname != NULL)
        snprintf(dev->rwbuffer, 18, "/dev/smart%d%sd1", minor, partname);
      else
        snprintf(dev->rwbuffer, 18, "/dev/smart%dd1", minor);

      /* Inode private data is a reference to a struct containing
       * the SMART device structure and the root directory number.
       */

      rootdirdev = (struct smart_multiroot_device_s*) kmalloc(sizeof(*rootdirdev));
      if (rootdirdev == NULL)
        {
          fdbg("register_blockdriver failed: %d\n", -ret);
          kfree(dev->sMap);
          kfree(dev->rwbuffer);
          kfree(dev);
          ret = -ENOMEM;
          goto errout;
        }

      /* Populate the rootdirdev */

      rootdirdev->dev = dev;
      rootdirdev->rootdirnum = 0;
      ret = register_blockdriver(dev->rwbuffer, &g_bops, 0, rootdirdev);

#else
      if (partname != NULL)
        snprintf(dev->rwbuffer, 18, "/dev/smart%d%s", minor, partname);
      else
        snprintf(dev->rwbuffer, 18, "/dev/smart%d", minor);

      /* Inode private data is a reference to the SMART device structure */

      ret = register_blockdriver(dev->rwbuffer, &g_bops, 0, dev);
#endif

      if (ret < 0)
        {
          fdbg("register_blockdriver failed: %d\n", -ret);
          kfree(dev->sMap);
          kfree(dev->rwbuffer);
          kfree(dev);
          goto errout;
        }

      /* Do a scan of the device */

      smart_scan(dev);
    }

errout:
  return ret;
}
