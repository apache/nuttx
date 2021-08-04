/****************************************************************************
 * fs/fat/fs_fat32util.c
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
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/fat.h>

#include "inode/inode.h"
#include "fs_fat32.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fat_checkfsinfo
 *
 * Description:
 *   Read the FAT32 FSINFO sector
 *
 ****************************************************************************/

static int fat_checkfsinfo(struct fat_mountpt_s *fs)
{
  /* Make sure that the fsinfo sector is in the cache */

  if (fat_fscacheread(fs, fs->fs_fsinfo) == OK)
    {
      /* Verify that this is, indeed, an FSINFO sector */

      if (FSI_GETLEADSIG(fs->fs_buffer) == 0x41615252  &&
          FSI_GETSTRUCTSIG(fs->fs_buffer) == 0x61417272 &&
          FSI_GETTRAILSIG(fs->fs_buffer) == BOOT_SIGNATURE32)
        {
          fs->fs_fsifreecount = FSI_GETFREECOUNT(fs->fs_buffer);
          fs->fs_fsinextfree  = FSI_GETNXTFREE(fs->fs_buffer);
          return OK;
        }
    }

  return -ENODEV;
}

/****************************************************************************
 * Name: fat_checkbootrecord
 *
 * Description:
 *   Verify that that currently buffer sector is a valid FAT boot record.
 *   This may refer to either the older (pre-partition) MBR sector that lies
 *   at sector one or to the more common FBR that lies at the beginning of
 *   the partition.
 *
 *   NOTE: The more common FBR naming is used in the file even when parsing
 *   an MBR.  This is possible because the field offsets and meaning are
 *   identical.
 *
 ****************************************************************************/

static int fat_checkbootrecord(struct fat_mountpt_s *fs)
{
  uint32_t ndatasectors;
  uint32_t ntotalfatsects;
  uint16_t rootdirsectors = 0;
  bool     notfat32 = false;

  /* Verify the MBR signature at offset 510 in the sector (true even
   * if the sector size is greater than 512.  All FAT file systems have
   * this signature. On a FAT32 volume, the RootEntCount , FatSz16, and
   * FatSz32 values should always be zero.  The FAT sector size should
   * match the reported hardware sector size.
   */

  if (FBR_GETSIGNATURE(fs->fs_buffer) != BOOT_SIGNATURE16 ||
      FBR_GETBYTESPERSEC(fs->fs_buffer) != fs->fs_hwsectorsize)
    {
      fwarn("WARNING: Signature: %04x FS sectorsize: %d "
            "HW sectorsize: %" PRIdOFF "\n",
            FBR_GETSIGNATURE(fs->fs_buffer),
            FBR_GETBYTESPERSEC(fs->fs_buffer),
            fs->fs_hwsectorsize);

      return -EINVAL;
    }

  /* Verify the FAT32 file system type. The determination of the file
   * system type is based on the number of clusters on the volume:  FAT12
   * volume has <= FAT_MAXCLUST12 (4084) clusters, a FAT16 volume has <=
   * FAT_MAXCLUST16 (Microsoft says < 65,525) clusters, and any larger
   * is FAT32.
   *
   * Get the number of 32-bit directory entries in root directory (zero
   * for FAT32).
   */

  fs->fs_rootentcnt = FBR_GETROOTENTCNT(fs->fs_buffer);
  if (fs->fs_rootentcnt != 0)
    {
      notfat32       = true; /* Must be zero for FAT32 */
      rootdirsectors = (32 * fs->fs_rootentcnt  + fs->fs_hwsectorsize - 1) /
                       fs->fs_hwsectorsize;
    }

  /* Determine the number of sectors in a FAT. */

  fs->fs_nfatsects = FBR_GETFATSZ16(fs->fs_buffer); /* Should be zero */
  if (fs->fs_nfatsects)
    {
      notfat32 = true; /* Must be zero for FAT32 */
    }
  else
    {
      fs->fs_nfatsects = FBR_GETFATSZ32(fs->fs_buffer);
    }

  if (!fs->fs_nfatsects || fs->fs_nfatsects >= fs->fs_hwnsectors)
    {
      fwarn("WARNING: fs_nfatsects %" PRId32
            " fs_hwnsectors: %" PRIdOFF "\n",
            fs->fs_nfatsects, fs->fs_hwnsectors);

      return -EINVAL;
    }

  /* Get the total number of sectors on the volume. */

  fs->fs_fattotsec = FBR_GETTOTSEC16(fs->fs_buffer); /* Should be zero */
  if (fs->fs_fattotsec)
    {
      notfat32 = true; /* Must be zero for FAT32 */
    }
  else
    {
      fs->fs_fattotsec = FBR_GETTOTSEC32(fs->fs_buffer);
    }

  if (!fs->fs_fattotsec || fs->fs_fattotsec > fs->fs_hwnsectors)
    {
      fwarn("WARNING: fs_fattotsec %" PRId32
            " fs_hwnsectors: %" PRIdOFF "\n",
            fs->fs_fattotsec, fs->fs_hwnsectors);

      return -EINVAL;
    }

  /* Get the total number of reserved sectors */

  fs->fs_fatresvdseccount = FBR_GETRESVDSECCOUNT(fs->fs_buffer);
  if (fs->fs_fatresvdseccount > fs->fs_hwnsectors)
    {
      fwarn("WARNING: fs_fatresvdseccount %d"
            " fs_hwnsectors: %" PRIdOFF "\n",
            fs->fs_fatresvdseccount, fs->fs_hwnsectors);

      return -EINVAL;
    }

  /* Get the number of FATs. This is probably two but could have other
   * values.
   */

  fs->fs_fatnumfats = FBR_GETNUMFATS(fs->fs_buffer);
  ntotalfatsects = fs->fs_fatnumfats * fs->fs_nfatsects;

  /* Get the total number of data sectors */

  ndatasectors = fs->fs_fattotsec - fs->fs_fatresvdseccount -
                 ntotalfatsects - rootdirsectors;
  if (ndatasectors > fs->fs_hwnsectors)
    {
      fwarn("WARNING: ndatasectors %" PRId32
            " fs_hwnsectors: %" PRIdOFF "\n",
            ndatasectors, fs->fs_hwnsectors);

      return -EINVAL;
    }

  /* Get the sectors per cluster */

  fs->fs_fatsecperclus = FBR_GETSECPERCLUS(fs->fs_buffer);

  /* Calculate the number of clusters */

  fs->fs_nclusters = ndatasectors / fs->fs_fatsecperclus;

  /* Finally, the test: */

  if (fs->fs_nclusters <= FAT_MAXCLUST12)
    {
      fs->fs_fsinfo = 0;
      fs->fs_type   = FSTYPE_FAT12;
    }
  else if (fs->fs_nclusters <= FAT_MAXCLUST16)
    {
      fs->fs_fsinfo = 0;
      fs->fs_type   = FSTYPE_FAT16;
    }
  else if (!notfat32)
    {
      fs->fs_fsinfo = fs->fs_fatbase + FBR_GETFSINFO(fs->fs_buffer);
      fs->fs_type   = FSTYPE_FAT32;
    }
  else
    {
      fwarn("WARNING: notfat32: %d fs_nclusters: %" PRId32 "\n",
            notfat32, fs->fs_nclusters);

      return -EINVAL;
    }

  /* We have what appears to be a valid FAT filesystem! Save a few more
   * things from the boot record that we will need later.
   */

  fs->fs_fatbase     += fs->fs_fatresvdseccount;

  if (fs->fs_type == FSTYPE_FAT32)
    {
      fs->fs_rootbase = FBR_GETROOTCLUS(fs->fs_buffer);
    }
  else
    {
      fs->fs_rootbase = fs->fs_fatbase + ntotalfatsects;
    }

  fs->fs_database     = fs->fs_fatbase + ntotalfatsects + fs->fs_rootentcnt /
                        DIRSEC_NDIRS(fs);
  fs->fs_fsifreecount = 0xffffffff;

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fat_getuint16
 ****************************************************************************/

uint16_t fat_getuint16(uint8_t *ptr)
{
  /* NOTE that (1) this operation is independent of endian-ness and that (2)
   * byte-by-byte transfer is necessary in any case because the address may
   * be unaligned.
   */

  return ((uint16_t)ptr[1] << 8) | ptr[0];
}

/****************************************************************************
 * Name: fat_getuint32
 ****************************************************************************/

uint32_t fat_getuint32(uint8_t *ptr)
{
  /* NOTE that (1) this operation is independent of endian-ness and that (2)
   * byte-by-byte transfer is necessary in any case because the address may
   * be unaligned.
   */

  return ((uint32_t)fat_getuint16(&ptr[2]) << 16) | fat_getuint16(&ptr[0]);
}

/****************************************************************************
 * Name: fat_putuint16
 ****************************************************************************/

void fat_putuint16(FAR uint8_t *ptr, uint16_t value16)
{
  FAR uint8_t *val = (FAR uint8_t *)&value16;

#ifdef CONFIG_ENDIAN_BIG
  /* If the target is big-endian then the bytes always have to be swapped so
   * that the representation is little endian in the file system.
   */

  ptr[0] = val[1];
  ptr[1] = val[0];

#else
  /* Byte-by-byte transfer is still necessary because the address may be
   * un-aligned.
   */

  ptr[0] = val[0];
  ptr[1] = val[1];
#endif
}

/****************************************************************************
 * Name: fat_putuint32
 ****************************************************************************/

void fat_putuint32(FAR uint8_t *ptr, uint32_t value32)
{
  FAR uint16_t *val = (FAR uint16_t *)&value32;

#ifdef CONFIG_ENDIAN_BIG
  /* If the target is big-endian then the bytes always have to be swapped so
   * that the representation is little endian in the file system.
   */

  fat_putuint16(&ptr[0], val[1]);
  fat_putuint16(&ptr[2], val[0]);

#else
  /* Byte-by-byte transfer is still necessary because the address may be
   * un-aligned.
   */

  fat_putuint16(&ptr[0], val[0]);
  fat_putuint16(&ptr[2], val[1]);
#endif
}

/****************************************************************************
 * Name: fat_semtake
 ****************************************************************************/

int fat_semtake(struct fat_mountpt_s *fs)
{
  return nxsem_wait_uninterruptible(&fs->fs_sem);
}

/****************************************************************************
 * Name: fat_semgive
 ****************************************************************************/

void fat_semgive(struct fat_mountpt_s *fs)
{
  nxsem_post(&fs->fs_sem);
}

/****************************************************************************
 * Name: fat_systime2fattime
 *
 * Description:
 *   Get the system time convert to a time and and date suitable for
 *   writing into the FAT FS.
 *
 *    TIME in LS 16-bits:
 *      Bits 0:4   = 2 second count (0-29 representing 0-58 seconds)
 *      Bits 5-10  = minutes (0-59)
 *      Bits 11-15 = hours (0-23)
 *    DATE in MS 16-bits
 *      Bits 0:4   = Day of month (1-31)
 *      Bits 5:8   = Month of year (1-12)
 *      Bits 9:15  = Year from 1980 (0-127 representing 1980-2107)
 *
 ****************************************************************************/

uint32_t fat_systime2fattime(void)
{
  /* Unless you have a hardware RTC or some other to get accurate time, then
   * there is no reason to support FAT time.
   */

#ifdef CONFIG_FS_FATTIME
  struct timespec ts;
  struct tm tm;
  int ret;

  /* Get the current time in seconds and nanoseconds */

  ret = clock_gettime(CLOCK_REALTIME, &ts);
  if (ret == OK)
    {
      /* Break done the seconds in date and time units */

      if (gmtime_r((FAR const time_t *)&ts.tv_sec, &tm) != NULL)
        {
          /* FAT can only represent dates since 1980.  struct tm can
           * represent dates since 1900.
           */

          if (tm.tm_year >= 80)
            {
              uint16_t fattime;
              uint16_t fatdate;

              fattime  = (tm.tm_sec         >>  1) & 0x001f; /* Bits 0-4: 2 second count (0-29) */
              fattime |= (tm.tm_min         <<  5) & 0x07e0; /* Bits 5-10: minutes (0-59) */
              fattime |= (tm.tm_hour        << 11) & 0xf800; /* Bits 11-15: hours (0-23) */

              fatdate  =  tm.tm_mday               & 0x001f; /* Bits 0-4: Day of month (1-31) */
              fatdate |= ((tm.tm_mon + 1)   <<  5) & 0x01e0; /* Bits 5-8: Month of year (1-12) */
              fatdate |= ((tm.tm_year - 80) <<  9) & 0xfe00; /* Bits 9-15: Year from 1980 */

              return (uint32_t)fatdate << 16 | (uint32_t)fattime;
            }
        }
    }
#endif

  return 0;
}

/****************************************************************************
 * Name: fat_fattime2systime
 *
 * Description:
 *   Convert FAT data and time to a system time_t
 *
 *    16-bit FAT time:
 *      Bits 0:4   = 2 second count (0-29 representing 0-58 seconds)
 *      Bits 5-10  = minutes (0-59)
 *      Bits 11-15 = hours (0-23)
 *    16-bit FAT date:
 *      Bits 0:4   = Day of month (1-31)
 *      Bits 5:8   = Month of year (1-12)
 *      Bits 9:15  = Year from 1980 (0-127 representing 1980-2107)
 *
 ****************************************************************************/

time_t fat_fattime2systime(uint16_t fattime, uint16_t fatdate)
{
  /* Unless you have a hardware RTC or some other to get accurate time, then
   * there is no reason to support FAT time.
   */

#ifdef CONFIG_FS_FATTIME
  struct tm tm;
  unsigned int tmp;

  /* Break out the date and time */

  tm.tm_sec  =  (fattime & 0x001f) <<  1;       /* Bits 0-4: 2 second count (0-29) */
  tm.tm_min  =  (fattime & 0x07e0) >>  5;       /* Bits 5-10: minutes (0-59) */
  tm.tm_hour =  (fattime & 0xf800) >> 11;       /* Bits 11-15: hours (0-23) */

  tm.tm_mday =  (fatdate & 0x001f);             /* Bits 0-4: Day of month (1-31) */
  tmp        = ((fatdate & 0x01e0) >>  5);      /* Bits 5-8: Month of year (1-12) */
  tm.tm_mon  =   tmp > 0 ? tmp - 1 : 0;
  tm.tm_year = ((fatdate & 0xfe00) >>  9) + 80; /* Bits 9-15: Year from 1980 */

  /* Then convert the broken out time into seconds since the epoch */

  return timegm(&tm);
#else
  return 0;
#endif
}

/****************************************************************************
 * Name: fat_mount
 *
 * Description:
 *   This function is called only when the mountpoint is first established.
 *   It initializes the mountpoint structure and verifies that a valid FAT32
 *   filesystem is provided by the block driver.
 *
 *   The caller should hold the mountpoint semaphore
 *
 ****************************************************************************/

int fat_mount(struct fat_mountpt_s *fs, bool writeable)
{
  FAR struct inode *inode;
  struct geometry geo;
  int ret;

  /* Assume that the mount is successful */

  fs->fs_mounted = true;

  /* Check if there is media available */

  inode = fs->fs_blkdriver;
  if (!inode || !inode->u.i_bops || !inode->u.i_bops->geometry ||
      inode->u.i_bops->geometry(inode, &geo) != OK || !geo.geo_available)
    {
      ret = -ENODEV;
      goto errout;
    }

  /* Make sure that that the media is write-able (if write access is
   * needed).
   */

  if (writeable && !geo.geo_writeenabled)
    {
      ret = -EACCES;
      goto errout;
    }

  /* Save the hardware geometry */

  fs->fs_hwsectorsize = geo.geo_sectorsize;
  fs->fs_hwnsectors   = geo.geo_nsectors;

  /* Allocate a buffer to hold one hardware sector */

  fs->fs_buffer = (FAR uint8_t *)fat_io_alloc(fs->fs_hwsectorsize);
  if (!fs->fs_buffer)
    {
      ret = -ENOMEM;
      goto errout;
    }

  /* Search FAT boot record on the drive.  First check the MBR at sector
   * zero.  This could be either the boot record or a partition that refers
   * to the boot record.
   *
   * First read sector zero.  This will be the first access to the drive and
   * a likely failure point.
   */

  fs->fs_fatbase = 0;
  ret = fat_hwread(fs, fs->fs_buffer, 0, 1);
  if (ret < 0)
    {
      goto errout_with_buffer;
    }

  /* Older style MBR (pre-partition table) includes boot information for the
   * partition-less drive.  Check for that case first.
   */

  ret = fat_checkbootrecord(fs);
  if (ret != OK)
    {
      /* The contents of sector 0 is not a boot record.  It could be have
       * DOS partitions, however.  Get the offset into the partition table.
       * This table is at offset MBR_TABLE and is indexed by 16x the
       * partition number.
       */

      int i;
      for (i = 0; i < 4; i++)
        {
          /* Check if the partition exists and, if so, get the bootsector for
           * that partition and see if we can find the boot record there.
           */

          uint8_t part = PART_GETTYPE(i, fs->fs_buffer);
          finfo("Partition %d, offset %d, type %d\n",
                 i, PART_ENTRY(i), part);

          if (part == 0)
            {
              finfo("No partition %d\n", i);
              continue;
            }

          /* There appears to be a partition, get the sector number of the
           * partition (LBA)
           */

          fs->fs_fatbase = PART_GETSTARTSECTOR(i, fs->fs_buffer);

          /* Read the new candidate boot sector */

          ret = fat_hwread(fs, fs->fs_buffer, fs->fs_fatbase, 1);
          if (ret < 0)
            {
              /* Failed to read the sector */

              ferr("ERROR: Failed to read sector %ld: %d\n",
                   (long)fs->fs_fatbase, ret);
              continue;
            }

          /* Check if this is a boot record */

          ret = fat_checkbootrecord(fs);
          if (ret == OK)
            {
              /* Break out of the loop if a valid boot record is found */

              finfo("FBR found in partition %d\n", i);
              break;
            }

          /* Re-read sector 0 so that we can check the next partition */

          finfo("Partition %d is not an FBR\n", i);
          ret = fat_hwread(fs, fs->fs_buffer, 0, 1);
          if (ret < 0)
            {
              ferr("ERROR: Failed to re-read sector 0: %d\n", ret);
              goto errout_with_buffer;
            }
        }

      if (i > 3)
        {
          ferr("ERROR: No valid boot record\n");
          ret = -EINVAL;
          goto errout_with_buffer;
        }
    }

  /* We have what appears to be a valid FAT filesystem! Now read the
   * FSINFO sector (FAT32 only)
   */

  if (fs->fs_type == FSTYPE_FAT32)
    {
      ret = fat_checkfsinfo(fs);
      if (ret != OK)
        {
          goto errout_with_buffer;
        }
    }

  /* Enforce computation of free clusters if configured */

#ifdef CONFIG_FAT_COMPUTE_FSINFO
  ret = fat_computefreeclusters(fs);
  if (ret != OK)
    {
      goto errout_with_buffer;
    }
#endif

  /* We did it! */

  finfo("FAT%d:\n", fs->fs_type == 0 ? 12 : fs->fs_type == 1  ? 16 : 32);
  finfo("\tHW  sector size:     %" PRIdOFF "\n", fs->fs_hwsectorsize);
  finfo("\t    sectors:         %" PRIdOFF "\n", fs->fs_hwnsectors);
  finfo("\tFAT reserved:        %d\n", fs->fs_fatresvdseccount);
  finfo("\t    sectors:         %" PRId32 "\n", fs->fs_fattotsec);
  finfo("\t    start sector:    %" PRIdOFF "\n", fs->fs_fatbase);
  finfo("\t    root sector:     %" PRIdOFF "\n", fs->fs_rootbase);
  finfo("\t    root entries:    %d\n", fs->fs_rootentcnt);
  finfo("\t    data sector:     %" PRIdOFF "\n", fs->fs_database);
  finfo("\t    FSINFO sector:   %" PRIdOFF "\n", fs->fs_fsinfo);
  finfo("\t    Num FATs:        %d\n", fs->fs_fatnumfats);
  finfo("\t    FAT sectors:     %" PRId32 "\n", fs->fs_nfatsects);
  finfo("\t    sectors/cluster: %d\n", fs->fs_fatsecperclus);
  finfo("\t    max clusters:    %" PRId32 "\n", fs->fs_nclusters);
  finfo("\tFSI free count       %" PRId32 "\n", fs->fs_fsifreecount);
  finfo("\t    next free        %" PRId32 "\n", fs->fs_fsinextfree);

  return OK;

errout_with_buffer:
  fat_io_free(fs->fs_buffer, fs->fs_hwsectorsize);
  fs->fs_buffer = 0;

errout:
  fs->fs_mounted = false;
  return ret;
}

/****************************************************************************
 * Name: fat_checkmount
 *
 * Description:
 *   Check if the mountpoint is still valid.
 *
 *   The caller should hold the mountpoint semaphore
 *
 ****************************************************************************/

int fat_checkmount(struct fat_mountpt_s *fs)
{
  /* If the fs_mounted flag is false, then we have already handled the loss
   * of the mount.
   */

  if (fs && fs->fs_mounted)
    {
      /* We still think the mount is healthy.  Check an see if this is
       * still the case
       */

      if (fs->fs_blkdriver)
        {
          struct inode *inode = fs->fs_blkdriver;
          if (inode && inode->u.i_bops && inode->u.i_bops->geometry)
            {
              struct geometry geo;
              int errcode = inode->u.i_bops->geometry(inode, &geo);
              if (errcode == OK && geo.geo_available &&
                  !geo.geo_mediachanged)
                {
                  return OK;
                }
            }
        }

      /* If we get here, the mount is NOT healthy */

      fs->fs_mounted = false;
    }

  return -ENODEV;
}

/****************************************************************************
 * Name: fat_hwread
 *
 * Description:
 *   Read the specified sector into the sector buffer
 *
 ****************************************************************************/

int fat_hwread(struct fat_mountpt_s *fs, uint8_t *buffer,  off_t sector,
               unsigned int nsectors)
{
  int ret = -ENODEV;
  if (fs && fs->fs_blkdriver)
    {
      struct inode *inode = fs->fs_blkdriver;
      if (inode && inode->u.i_bops && inode->u.i_bops->read)
        {
          ssize_t nsectorsread = inode->u.i_bops->read(inode, buffer,
                                                       sector, nsectors);
          if (nsectorsread == nsectors)
            {
              ret = OK;
            }
          else if (nsectorsread < 0)
            {
              ret = nsectorsread;
            }
        }
    }

  return ret;
}

/****************************************************************************
 * Name: fat_hwwrite
 *
 * Description:
 *   Write the sector buffer to the specified sector
 *
 ****************************************************************************/

int fat_hwwrite(struct fat_mountpt_s *fs, uint8_t *buffer, off_t sector,
                unsigned int nsectors)
{
  int ret = -ENODEV;
  if (fs && fs->fs_blkdriver)
    {
      struct inode *inode = fs->fs_blkdriver;
      if (inode && inode->u.i_bops && inode->u.i_bops->write)
        {
          ssize_t nsectorswritten =
              inode->u.i_bops->write(inode, buffer, sector, nsectors);

          if (nsectorswritten == nsectors)
            {
              ret = OK;
            }
          else if (nsectorswritten < 0)
            {
              ret = nsectorswritten;
            }
        }
    }

  return ret;
}

/****************************************************************************
 * Name: fat_cluster2sector
 *
 * Description:
 *   Convert a cluster number to a start sector number
 *
 ****************************************************************************/

off_t fat_cluster2sector(FAR struct fat_mountpt_s *fs,  uint32_t cluster)
{
  cluster -= 2;
  if (cluster >= fs->fs_nclusters - 2)
    {
      return -EINVAL;
    }

  return cluster * fs->fs_fatsecperclus + fs->fs_database;
}

/****************************************************************************
 * Name: fat_getcluster
 *
 * Description:
 *   Get the next cluster start from the FAT.
 *
 * Returned Value:
 *   <0: error, 0:cluster unassigned, >=0: start sector of cluster
 *
 ****************************************************************************/

off_t fat_getcluster(struct fat_mountpt_s *fs, uint32_t clusterno)
{
  /* Verify that the cluster number is within range */

  if (clusterno >= 2 && clusterno < fs->fs_nclusters)
    {
      /* Okay.. Read the next cluster from the FAT.  The way we will do
       * this depends on the type of FAT filesystem we are dealing with.
       */

      switch (fs->fs_type)
        {
          case FSTYPE_FAT12 :
            {
              off_t        fatsector;
              unsigned int fatoffset;
              unsigned int cluster;
              unsigned int fatindex;

              /* FAT12 is more complex because it has 12-bits (1.5 bytes)
               * per FAT entry. Get the offset to the first byte:
               */

              fatoffset = (clusterno * 3) / 2;
              fatsector = fs->fs_fatbase + SEC_NSECTORS(fs, fatoffset);

              /* Read the sector at this offset */

              if (fat_fscacheread(fs, fatsector) < 0)
                {
                  /* Read error */

                  break;
                }

              /* Get the first, LS byte of the cluster from the FAT */

              fatindex = fatoffset & SEC_NDXMASK(fs);
              cluster  = fs->fs_buffer[fatindex];

              /* With FAT12, the second byte of the cluster number may lie in
               * a different sector than the first byte.
               */

              fatindex++;
              if (fatindex >= fs->fs_hwsectorsize)
                {
                  fatsector++;
                  fatindex = 0;

                  if (fat_fscacheread(fs, fatsector) < 0)
                    {
                      /* Read error */

                      break;
                    }
                }

              /* Get the second, MS byte of the cluster for 16-bits.  The
               * does not depend on the endian-ness of the target, but only
               * on the fact that the byte stream is little-endian.
               */

              cluster |= (unsigned int)fs->fs_buffer[fatindex] << 8;

              /* Now, pick out the correct 12 bit cluster start sector
               * value.
               */

              if ((clusterno & 1) != 0)
                {
                  /* Odd.. take the MS 12-bits */

                  cluster >>= 4;
                }
              else
                {
                  /* Even.. take the LS 12-bits */

                  cluster &= 0x0fff;
                }

              return cluster;
            }

          case FSTYPE_FAT16 :
            {
              unsigned int fatoffset = 2 * clusterno;
              off_t        fatsector = fs->fs_fatbase +
                                       SEC_NSECTORS(fs, fatoffset);
              unsigned int fatindex  = fatoffset & SEC_NDXMASK(fs);

              if (fat_fscacheread(fs, fatsector) < 0)
                {
                  /* Read error */

                  break;
                }

              return FAT_GETFAT16(fs->fs_buffer, fatindex);
            }

          case FSTYPE_FAT32 :
            {
              unsigned int fatoffset = 4 * clusterno;
              off_t        fatsector = fs->fs_fatbase +
                                       SEC_NSECTORS(fs, fatoffset);
              unsigned int fatindex  = fatoffset & SEC_NDXMASK(fs);

              if (fat_fscacheread(fs, fatsector) < 0)
                {
                  /* Read error */

                  break;
                }

              return FAT_GETFAT32(fs->fs_buffer, fatindex) & 0x0fffffff;
            }

          default:
              break;
        }
    }

  /* There is no cluster information, or an error occurred */

  return (off_t)-EINVAL;
}

/****************************************************************************
 * Name: fat_putcluster
 *
 * Description:
 *   Write a new cluster into the FAT
 *
 ****************************************************************************/

int fat_putcluster(struct fat_mountpt_s *fs, uint32_t clusterno,
                   off_t nextcluster)
{
  /* Verify that the cluster number is within range.  Zero erases the
   * cluster.
   */

  if (clusterno == 0 || (clusterno >= 2 && clusterno < fs->fs_nclusters))
    {
      /* Okay.. Write the next cluster into the FAT.  The way we will do
       * this depends on the type of FAT filesystem we are dealing with.
       */

      switch (fs->fs_type)
        {
          case FSTYPE_FAT12 :
            {
              off_t        fatsector;
              unsigned int fatoffset;
              unsigned int fatindex;
              uint8_t      value;

              /* FAT12 is more complex because it has 12-bits (1.5 bytes)
               * per FAT entry. Get the offset to the first byte:
               */

              fatoffset = (clusterno * 3) / 2;
              fatsector = fs->fs_fatbase + SEC_NSECTORS(fs, fatoffset);

              /* Make sure that the sector at this offset is in the cache */

              if (fat_fscacheread(fs, fatsector) < 0)
                {
                  /* Read error */

                  break;
                }

              /* Get the LS byte first handling the 12-bit alignment within
               * the 16-bits
               */

              fatindex = fatoffset & SEC_NDXMASK(fs);
              if ((clusterno & 1) != 0)
                {
                  /* Save the LS four bits of the next cluster */

                  value = (fs->fs_buffer[fatindex] & 0x0f) |
                           nextcluster << 4;
                }
              else
                {
                  /* Save the LS eight bits of the next cluster */

                  value = (uint8_t)nextcluster;
                }

              fs->fs_buffer[fatindex] = value;

              /* With FAT12, the second byte of the cluster number may lie in
               * a different sector than the first byte.
               */

              fatindex++;
              if (fatindex >= fs->fs_hwsectorsize)
                {
                  /* Read the next sector */

                  fatsector++;
                  fatindex = 0;

                  /* Set the dirty flag to make sure the sector that we
                   * just modified is written out.
                   */

                  fs->fs_dirty = true;
                  if (fat_fscacheread(fs, fatsector) < 0)
                    {
                      /* Read error */

                      break;
                    }
                }

              /* Output the MS byte first handling the 12-bit alignment
               * within the 16-bits
               */

              if ((clusterno & 1) != 0)
                {
                  /* Save the MS eight bits of the next cluster */

                  value = (uint8_t)(nextcluster >> 4);
                }
              else
                {
                  /* Save the MS four bits of the next cluster */

                  value = (fs->fs_buffer[fatindex] & 0xf0) |
                          ((nextcluster >> 8) & 0x0f);
                }

              fs->fs_buffer[fatindex] = value;
            }
          break;

          case FSTYPE_FAT16 :
            {
              unsigned int fatoffset = 2 * clusterno;
              off_t        fatsector = fs->fs_fatbase +
                                       SEC_NSECTORS(fs, fatoffset);
              unsigned int fatindex  = fatoffset & SEC_NDXMASK(fs);

              if (fat_fscacheread(fs, fatsector) < 0)
                {
                  /* Read error */

                  break;
                }

              FAT_PUTFAT16(fs->fs_buffer, fatindex, nextcluster & 0xffff);
            }
          break;

          case FSTYPE_FAT32 :
            {
              unsigned int fatoffset = 4 * clusterno;
              off_t        fatsector = fs->fs_fatbase +
                                       SEC_NSECTORS(fs, fatoffset);
              unsigned int fatindex  = fatoffset & SEC_NDXMASK(fs);
              uint32_t     val;

              if (fat_fscacheread(fs, fatsector) < 0)
                {
                  /* Read error */

                  break;
                }

              /* Keep the top 4 bits */

              val = FAT_GETFAT32(fs->fs_buffer, fatindex) & 0xf0000000;
              FAT_PUTFAT32(fs->fs_buffer, fatindex,
                           val | (nextcluster & 0x0fffffff));
            }
          break;

          default:
            return -EINVAL;
        }

      /* Mark the modified sector as "dirty" and return success */

      fs->fs_dirty = true;
      return OK;
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: fat_removechain
 *
 * Description:
 *   Remove an entire chain of clusters, starting with 'cluster'
 *
 ****************************************************************************/

int fat_removechain(struct fat_mountpt_s *fs, uint32_t cluster)
{
  int32_t nextcluster;
  int    ret;

  /* Loop while there are clusters in the chain */

  while (cluster >= 2 && cluster < fs->fs_nclusters)
    {
      /* Get the next cluster after the current one */

      nextcluster = fat_getcluster(fs, cluster);
      if (nextcluster < 0)
        {
          /* Error! */

          return nextcluster;
        }

      /* Then nullify current cluster -- removing it from the chain */

      ret = fat_putcluster(fs, cluster, 0);
      if (ret < 0)
        {
          return ret;
        }

      /* Update FSINFINFO data */

      if (fs->fs_fsifreecount != 0xffffffff)
        {
          fs->fs_fsifreecount++;
          fs->fs_fsidirty = true;
        }

      /* Then set up to remove the next cluster */

      cluster = nextcluster;
    }

  return OK;
}

/****************************************************************************
 * Name: fat_extendchain
 *
 * Description:
 *   Add a new cluster to the chain following cluster (if cluster is non-
 *   NULL).  if cluster is zero, then a new chain is created.
 *
 * Returned Value:
 *   <0:error, 0: no free cluster, >=2: new cluster number
 *
 ****************************************************************************/

int32_t fat_extendchain(struct fat_mountpt_s *fs, uint32_t cluster)
{
  off_t    startsector;
  uint32_t newcluster;
  uint32_t startcluster;
  int      ret;

  /* The special value 0 is used when the new chain should start */

  if (cluster == 0)
    {
      /* The FSINFO NextFree entry should be a good starting point
       * in the search for a new cluster
       */

      startcluster = fs->fs_fsinextfree;
      if (startcluster == 0 || startcluster >= fs->fs_nclusters)
        {
          /* But it is bad.. we have to start at the beginning */

          startcluster = 1;
        }
    }
  else
    {
      /* We are extending an existing chain. Verify that this
       * is a valid cluster by examining its start sector.
       */

      startsector = fat_getcluster(fs, cluster);
      if (startsector < 0)
        {
          /* An error occurred, return the error value */

          return startsector;
        }
      else if (startsector < 2)
        {
          /* Oops.. this cluster does not exist. */

          return 0;
        }
      else if (startsector < fs->fs_nclusters)
        {
          /* It is already followed by next cluster */

          return startsector;
        }

      /* Okay.. it checks out */

      startcluster = cluster;
    }

  /* Loop until (1) we discover that there are not free clusters
   * (return 0), an errors occurs (return -errno), or (3) we find
   * the next cluster (return the new cluster number).
   */

  newcluster = startcluster;
  for (; ; )
    {
      /* Examine the next cluster in the FAT */

      newcluster++;
      if (newcluster >= fs->fs_nclusters)
        {
          /* If we hit the end of the available clusters, then
           * wrap back to the beginning because we might have
           * started at a non-optimal place.  But don't continue
           * past the start cluster.
           */

          newcluster = 2;
          if (newcluster > startcluster)
            {
              /* We are back past the starting cluster, then there
               * is no free cluster.
               */

              return 0;
            }
        }

      /* We have a candidate cluster.  Check if the cluster number is
       * mapped to a group of sectors.
       */

      startsector = fat_getcluster(fs, newcluster);
      if (startsector == 0)
        {
          /* Found have found a free cluster break out */

          break;
        }
      else if (startsector < 0)
        {
          /* Some error occurred, return the error number */

          return startsector;
        }

      /* We wrap all the back to the starting cluster?  If so, then
       * there are no free clusters.
       */

      if (newcluster == startcluster)
        {
          return 0;
        }
    }

  /* We get here only if we break out with an available cluster
   * number in 'newcluster'  Now mark that cluster as in-use.
   */

  ret = fat_putcluster(fs, newcluster, 0x0fffffff);
  if (ret < 0)
    {
      /* An error occurred */

      return ret;
    }

  /* And link if to the start cluster (if any) */

  if (cluster)
    {
      /* There is a start cluster -- link it */

      ret = fat_putcluster(fs, cluster, newcluster);
      if (ret < 0)
        {
          return ret;
        }
    }

  /* And update the FINSINFO for the next time we have to search */

  fs->fs_fsinextfree = newcluster;
  if (fs->fs_fsifreecount != 0xffffffff)
    {
      fs->fs_fsifreecount--;
      fs->fs_fsidirty = true;
    }

  /* Return then number of the new cluster that was added to the chain */

  return newcluster;
}

/****************************************************************************
 * Name: fat_nextdirentry
 *
 * Description:
 *   Read the next directory entry from the sector in cache, reading the
 *   next sector(s) in the cluster as necessary.  This function must
 *   return -ENOSPC if it fails because there are no further entries
 *   available in the directory.
 *
 ****************************************************************************/

int fat_nextdirentry(struct fat_mountpt_s *fs, struct fs_fatdir_s *dir)
{
  unsigned int cluster;
  unsigned int ndx;

  /* Increment the index to the next 32-byte directory entry */

  ndx = dir->fd_index + 1;

  /* Check if all of the directory entries in this sectory have
   * been examined.
   */

  if ((ndx & (DIRSEC_NDIRS(fs)-1)) == 0)
    {
      /* Yes, then we will have to read the next sector */

      dir->fd_currsector++;

      /* For FAT12/16, the root directory is a group of sectors relative
       * to the first sector of the fat volume.
       */

      if (!dir->fd_currcluster)
        {
          /* For FAT12/16, the boot record tells us number of 32-bit
           * directories that are contained in the root directory.  This
           * should correspond to an even number of sectors.
           */

          if (ndx >= fs->fs_rootentcnt)
            {
              /* When we index past this count, we have examined all of the
               * entries in the root directory.
               */

              return -ENOSPC;
            }
        }
      else
        {
          /* Not a FAT12/16 root directory, check if we have examined the
           * entire cluster comprising the directory.
           *
           * The current sector within the cluster is the entry number
           * divided byte the number of entries per sector
           */

          int sector = ndx / DIRSEC_NDIRS(fs);

          /* We are finished with the cluster when the last sector of the
           * cluster has been examined.
           */

          if ((sector & (fs->fs_fatsecperclus - 1)) == 0)
            {
              /* Get next cluster */

              cluster = fat_getcluster(fs, dir->fd_currcluster);

              /* Check if a valid cluster was obtained. */

              if (cluster < 2 || cluster >= fs->fs_nclusters)
                {
                  /* No, we have probably reached the end of the cluster
                   * list.
                   */

                  return -ENOSPC;
                }

              /* Initialize for new cluster */

              dir->fd_currcluster = cluster;
              dir->fd_currsector  = fat_cluster2sector(fs, cluster);
              ndx                 = 0;
            }
        }
    }

  /* Save the new index into dir->fd_currsector */

  dir->fd_index = ndx;
  return OK;
}

/****************************************************************************
 * Name: fat_dirtruncate
 *
 * Description:
 *   Truncate an existing file to zero length.
 *
 * Assumptions:
 *   The caller holds mountpoint semaphore, fs_buffer holds the directory
 *   entry, the directory entry sector (fd_sector) is currently in the
 *   sector cache.
 *
 ****************************************************************************/

int fat_dirtruncate(struct fat_mountpt_s *fs, FAR uint8_t *direntry)
{
  unsigned int startcluster;
  uint32_t writetime;
  off_t savesector;
  int ret;

  /* Get start cluster of the file to truncate */

  startcluster = ((uint32_t)DIR_GETFSTCLUSTHI(direntry) << 16) |
                  DIR_GETFSTCLUSTLO(direntry);

  /* Clear the cluster start value in the directory and set the file size
   * to zero.  This makes the file look empty but also have to dispose of
   * all of the clusters in the chain.
   */

  DIR_PUTFSTCLUSTHI(direntry, 0);
  DIR_PUTFSTCLUSTLO(direntry, 0);
  DIR_PUTFILESIZE(direntry, 0);

  /* Set the ARCHIVE attribute and update the write time */

  DIR_PUTATTRIBUTES(direntry, FATATTR_ARCHIVE);

  writetime = fat_systime2fattime();
  DIR_PUTWRTTIME(direntry, writetime & 0xffff);
  DIR_PUTWRTDATE(direntry, writetime >> 16);

  /* This sector needs to be written back to disk eventually */

  fs->fs_dirty = true;

  /* Now remove the entire cluster chain comprising the file */

  savesector = fs->fs_currentsector;
  ret = fat_removechain(fs, startcluster);
  if (ret < 0)
    {
      return ret;
    }

  /* Setup FSINFO to reuse the old start cluster next */

  fs->fs_fsinextfree = startcluster - 1;

  /* Make sure that the directory is still in the cache */

  return fat_fscacheread(fs, savesector);
}

/****************************************************************************
 * Name: fat_dirshrink
 *
 * Description:
 *   Shrink the size existing file to a non-zero length
 *
 * Assumptions:
 *   The caller holds mountpoint semaphore, fs_buffer holds the directory
 *   entry.
 *
 ****************************************************************************/

int fat_dirshrink(struct fat_mountpt_s *fs, FAR uint8_t *direntry,
                  off_t length)
{
  off_t clustersize;
  off_t remaining;
  uint32_t writetime;
  int32_t lastcluster;
  int32_t cluster;
  int ret;

  /* Get start cluster of the file to truncate */

  lastcluster = ((uint32_t)DIR_GETFSTCLUSTHI(direntry) << 16) |
                 DIR_GETFSTCLUSTLO(direntry);

  /* Set the file size to the new length.  */

  DIR_PUTFILESIZE(direntry, length);

  /* Set the ARCHIVE attribute and update the write time */

  DIR_PUTATTRIBUTES(direntry, FATATTR_ARCHIVE);

  writetime = fat_systime2fattime();
  DIR_PUTWRTTIME(direntry, writetime & 0xffff);
  DIR_PUTWRTDATE(direntry, writetime >> 16);

  /* This sector needs to be written back to disk eventually */

  fs->fs_dirty = true;

  /* Now find the cluster change to be removed.  Start with the cluster
   * after the current one (which we know contains data).
   */

  cluster = fat_getcluster(fs, lastcluster);
  if (cluster < 0)
    {
      return cluster;
    }

  clustersize = fs->fs_fatsecperclus * fs->fs_hwsectorsize;
  remaining   = length;

  while (cluster >= 2 && cluster < fs->fs_nclusters)
    {
      /* Will there be data in the next cluster after the shrinkage? */

      if (remaining <= clustersize)
        {
          /* No.. then nullify next cluster -- removing it from the
           * chain.
           */

          ret = fat_putcluster(fs, lastcluster, 0);
          if (ret < 0)
            {
              return ret;
            }

          /* Then free the remainder of the chain */

          ret = fat_removechain(fs, cluster);
          if (ret < 0)
            {
              return ret;
            }

          /* Setup FSINFO to reuse the removed cluster next */

          fs->fs_fsinextfree = cluster - 1;
          break;
        }

      /* Then set up to remove the next cluster */

      lastcluster = cluster;
      cluster     = fat_getcluster(fs, cluster);

      if (cluster < 0)
        {
          return cluster;
        }

      remaining  -= clustersize;
    }

  return OK;
}

/****************************************************************************
 * Name: fat_dirextend
 *
 * Description:
 *   Zero-extend the length of a regular file to 'length'.
 *
 ****************************************************************************/

int fat_dirextend(FAR struct fat_mountpt_s *fs, FAR struct fat_file_s *ff,
                  off_t length)
{
  int32_t cluster;
  off_t remaining;
  off_t pos;
  unsigned int zerosize;
  int sectndx;
  int ret;

  /* We are extending the file.  This is essentially the same as a write
   * except that (1) we write zeros and (2) we don't update the file
   * position.
   */

  pos = ff->ff_size;

  /* Get the first sector to write to. */

  if (!ff->ff_currentsector)
    {
      /* Has the starting cluster been defined? */

      if (ff->ff_startcluster == 0)
        {
          /* No.. we have to create a new cluster chain */

          ff->ff_startcluster     = fat_createchain(fs);
          ff->ff_currentcluster   = ff->ff_startcluster;
          ff->ff_sectorsincluster = fs->fs_fatsecperclus;
        }

      /* The current sector can then be determined from the current cluster
       * and the file offset.
       */

      ret = fat_currentsector(fs, ff, pos);
      if (ret < 0)
        {
          return ret;
        }
    }

  /* Loop until either (1) the file has been fully extended with zeroed data
   * or (2) an error occurs.  We assume we start with the current sector in
   * cache (ff_currentsector)
   */

  sectndx   = pos & SEC_NDXMASK(fs);
  remaining = length - pos;

  while (remaining > 0)
    {
      /* Check if the current write stream has incremented to the next
       * cluster boundary
       */

      if (ff->ff_sectorsincluster < 1)
        {
          /* Extend the current cluster by one (unless lseek was used to
           * move the file position back from the end of the file)
           */

          cluster = fat_extendchain(fs, ff->ff_currentcluster);

          /* Verify the cluster number */

          if (cluster < 0)
            {
              return (int)cluster;
            }
          else if (cluster < 2 || cluster >= fs->fs_nclusters)
            {
              return -ENOSPC;
            }

          /* Setup to zero the first sector from the new cluster */

          ff->ff_currentcluster   = cluster;
          ff->ff_sectorsincluster = fs->fs_fatsecperclus;
          ff->ff_currentsector    = fat_cluster2sector(fs, cluster);
        }

      /* Decide whether we are performing a read-modify-write
       * operation, in which case we have to read the existing sector
       * into the buffer first.
       *
       * There are two cases where we can avoid this read:
       *
       * - If we are performing a whole-sector clear that was rejected
       *   by fat_hwwrite(), i.e. sectndx == 0 and remaining >= sector size.
       *
       * - If the clear is aligned to the beginning of the sector and
       *   extends beyond the end of the file, i.e. sectndx == 0 and
       *   file pos + remaining >= file size.
       */

      if (sectndx == 0 && (remaining >= fs->fs_hwsectorsize ||
          (pos + remaining) >= ff->ff_size))
        {
          /* Flush unwritten data in the sector cache. */

          ret = fat_ffcacheflush(fs, ff);
          if (ret < 0)
            {
              return ret;
            }

          /* Now mark the clean cache buffer as the current sector. */

          ff->ff_cachesector = ff->ff_currentsector;
        }
      else
        {
          /* Read the current sector into memory (perhaps first flushing the
           * old, dirty sector to disk).
           */

          ret = fat_ffcacheread(fs, ff, ff->ff_currentsector);
          if (ret < 0)
            {
              return ret;
            }
        }

      /* Copy the requested part of the sector from the user buffer */

      zerosize = fs->fs_hwsectorsize - sectndx;
      if (zerosize > remaining)
        {
          /* We will not zero to the end of the sector. */

          zerosize = remaining;
        }
      else
        {
          /* We will zero to the end of the buffer (or beyond).  Bump up
           * the current sector number (actually the next sector number).
           */

          ff->ff_sectorsincluster--;
          ff->ff_currentsector++;
        }

      /* Zero the data into the cached sector and make sure that the cached
       * sector is marked "dirty" so that it will be written back.
       */

      memset(&ff->ff_buffer[sectndx], 0, zerosize);
      ff->ff_bflags |= (FFBUFF_DIRTY | FFBUFF_VALID | FFBUFF_MODIFIED);

      /* Set up for the next sector */

      pos       += zerosize;
      remaining -= zerosize;
      sectndx    = pos & SEC_NDXMASK(fs);
    }

  /* The truncation has completed without error.  Update the file size */

  ff->ff_size = length;
  return OK;
}

/****************************************************************************
 * Name: fat_fscacheflush
 *
 * Description:
 *   Flush any dirty sector if fs_buffer as necessary
 *
 ****************************************************************************/

int fat_fscacheflush(struct fat_mountpt_s *fs)
{
  int ret;

  /* Check if the fs_buffer is dirty.  In this case, we will write back the
   * contents of fs_buffer.
   */

  if (fs->fs_dirty)
    {
      /* Write the dirty sector */

      ret = fat_hwwrite(fs, fs->fs_buffer, fs->fs_currentsector, 1);
      if (ret < 0)
        {
          return ret;
        }

      /* Does the sector lie in the FAT region? */

      if (fs->fs_currentsector >= fs->fs_fatbase &&
          fs->fs_currentsector < fs->fs_fatbase + fs->fs_nfatsects)
        {
          int i;

          /* Yes, then make the change in the FAT copy as well */

          for (i = fs->fs_fatnumfats; i >= 2; i--)
            {
              fs->fs_currentsector += fs->fs_nfatsects;
              ret = fat_hwwrite(fs, fs->fs_buffer, fs->fs_currentsector, 1);
              if (ret < 0)
                {
                  return ret;
                }
            }
        }

      /* No longer dirty */

      fs->fs_dirty = false;
    }

  return OK;
}

/****************************************************************************
 * Name: fat_fscacheread
 *
 * Description:
 *   Read the specified sector into the sector cache, flushing any existing
 *   dirty sectors as necessary.
 *
 ****************************************************************************/

int fat_fscacheread(struct fat_mountpt_s *fs, off_t sector)
{
  int ret;

  /* fs->fs_currentsector holds the current sector that is buffered in
   * fs->fs_buffer. If the requested sector is the same as this sector, then
   * we do nothing. Otherwise, we will have to read the new sector.
   */

  if (fs->fs_currentsector != sector)
    {
      /* We will need to read the new sector.  First, flush the cached
       * sector if it is dirty.
       */

      ret = fat_fscacheflush(fs);
      if (ret < 0)
        {
          return ret;
        }

      /* Then read the specified sector into the cache */

      ret = fat_hwread(fs, fs->fs_buffer, sector, 1);
      if (ret < 0)
        {
          return ret;
        }

      /* Update the cached sector number */

      fs->fs_currentsector = sector;
    }

  return OK;
}

/****************************************************************************
 * Name: fat_ffcacheflush
 *
 * Description:
 *   Flush any dirty sectors as necessary
 *
 ****************************************************************************/

int fat_ffcacheflush(struct fat_mountpt_s *fs, struct fat_file_s *ff)
{
  int ret;

  /* Check if the ff_buffer is dirty.  In this case, we will write back the
   * contents of ff_buffer.
   */

  if (ff->ff_cachesector &&
      (ff->ff_bflags & (FFBUFF_DIRTY | FFBUFF_VALID)) ==
       (FFBUFF_DIRTY | FFBUFF_VALID))
    {
      /* Write the dirty sector */

      ret = fat_hwwrite(fs, ff->ff_buffer, ff->ff_cachesector, 1);
      if (ret < 0)
        {
          return ret;
        }

      /* No longer dirty, but still valid */

      ff->ff_bflags &= ~FFBUFF_DIRTY;
    }

  return OK;
}

/****************************************************************************
 * Name: fat_ffcacheread
 *
 * Description:
 *   Read the specified sector into the sector cache, flushing any existing
 *   dirty sectors as necessary.
 *
 ****************************************************************************/

int fat_ffcacheread(struct fat_mountpt_s *fs, struct fat_file_s *ff,
                    off_t sector)
{
  int ret;

  /* ff->ff_cachesector holds the current sector that is buffered in
   * ff->ff_buffer. If the requested sector is the same as this sector, then
   * we do nothing. Otherwise, we will have to read the new sector.
   */

  if (ff->ff_cachesector != sector || (ff->ff_bflags & FFBUFF_VALID) == 0)
    {
      /* We will need to read the new sector.  First, flush the cached
       * sector if it is dirty.
       */

      ret = fat_ffcacheflush(fs, ff);
      if (ret < 0)
        {
          return ret;
        }

      /* Then read the specified sector into the cache */

      ret = fat_hwread(fs, ff->ff_buffer, sector, 1);
      if (ret < 0)
        {
          return ret;
        }

      /* Update the cached sector number */

      ff->ff_cachesector = sector;
      ff->ff_bflags |= FFBUFF_VALID;
    }

  return OK;
}

/****************************************************************************
 * Name: fat_ffcacheread
 *
 * Description:
 *   Invalidate the current file buffer contents
 *
 ****************************************************************************/

int fat_ffcacheinvalidate(struct fat_mountpt_s *fs, struct fat_file_s *ff)
{
  int ret;

  /* Is there anything valid in the buffer now? */

  if ((ff->ff_bflags & FFBUFF_VALID) != 0)
    {
      /* We will invalidate the buffered sector */

      ret = fat_ffcacheflush(fs, ff);
      if (ret < 0)
        {
          return ret;
        }

      /* Then discard the current cache contents */

      ff->ff_bflags     &= ~FFBUFF_VALID;
      ff->ff_cachesector = 0;
    }

  return OK;
}

/****************************************************************************
 * Name: fat_updatefsinfo
 *
 * Description:
 *   Flush everything buffered for the mountpoint and update the FSINFO
 *   sector, if appropriate
 *
 ****************************************************************************/

int fat_updatefsinfo(struct fat_mountpt_s *fs)
{
  int ret;

  /* Flush the fs_buffer if it is dirty */

  ret = fat_fscacheflush(fs);
  if (ret == OK)
    {
      /* The FSINFO sector only has to be update for the case of a FAT32 file
       * system.  Check if the file system type.. If this is a FAT32 file
       * system then the fs_fsidirty flag will indicate if the FSINFO sector
       * needs to be re-written.
       */

      if (fs->fs_type == FSTYPE_FAT32 && fs->fs_fsidirty)
        {
          /* Create an image of the FSINFO sector in the fs_buffer */

          memset(fs->fs_buffer, 0, fs->fs_hwsectorsize);
          FSI_PUTLEADSIG(fs->fs_buffer, 0x41615252);
          FSI_PUTSTRUCTSIG(fs->fs_buffer, 0x61417272);
          FSI_PUTFREECOUNT(fs->fs_buffer, fs->fs_fsifreecount);
          FSI_PUTNXTFREE(fs->fs_buffer, fs->fs_fsinextfree);
          FSI_PUTTRAILSIG(fs->fs_buffer, BOOT_SIGNATURE32);

          /* Then flush this to disk */

          fs->fs_currentsector = fs->fs_fsinfo;
          fs->fs_dirty         = true;
          ret                  = fat_fscacheflush(fs);

          /* No longer dirty */

          fs->fs_fsidirty = false;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: fat_computefreeclusters
 *
 * Description:
 *   Compute the number of free clusters from scratch
 *
 ****************************************************************************/

int fat_computefreeclusters(struct fat_mountpt_s *fs)
{
  /* We have to count the number of free clusters */

  uint32_t nfreeclusters = 0;
  if (fs->fs_type == FSTYPE_FAT12)
    {
      off_t sector;

      /* Examine every cluster in the fat */

      for (sector = 2; sector < fs->fs_nclusters; sector++)
        {
          /* If the cluster is unassigned, then increment the count of free
           * clusters
           */

          if ((uint16_t)fat_getcluster(fs, sector) == 0)
            {
              nfreeclusters++;
            }
        }
    }
  else
    {
      unsigned int cluster;
      off_t        fatsector;
      unsigned int offset;
      int          ret;

      fatsector    = fs->fs_fatbase;
      offset       = fs->fs_hwsectorsize;

      /* Examine each cluster in the fat */

      for (cluster = fs->fs_nclusters; cluster > 0; cluster--)
        {
          /* If we are starting a new sector, then read the new sector in
           * fs_buffer
           */

          if (offset >= fs->fs_hwsectorsize)
            {
              ret = fat_fscacheread(fs, fatsector);
              if (ret < 0)
                {
                  return ret;
                }

              /* Reset the offset to the next FAT entry.
               * Increment the sector number to read next time around.
               */

              offset = 0;
              fatsector++;
            }

          /* FAT16 and FAT32 differ only on the size of each cluster start
           * sector number in the FAT.
           */

          if (fs->fs_type == FSTYPE_FAT16)
            {
              if (FAT_GETFAT16(fs->fs_buffer, offset) == 0)
                {
                  nfreeclusters++;
                }

              offset += 2;
            }
          else
            {
              if (FAT_GETFAT32(fs->fs_buffer, offset) == 0)
                {
                  nfreeclusters++;
                }

              offset += 4;
            }
        }
    }

  fs->fs_fsifreecount = nfreeclusters;
  if (fs->fs_type == FSTYPE_FAT32)
    {
      fs->fs_fsidirty = true;
    }

  return OK;
}

/****************************************************************************
 * Name: fat_nfreeclusters
 *
 * Description:
 *   Get the number of free clusters
 *
 ****************************************************************************/

int fat_nfreeclusters(struct fat_mountpt_s *fs, fsblkcnt_t *pfreeclusters)
{
  /* If number of the first free cluster is valid, then just return that
   * value.
   */

  if (fs->fs_fsifreecount <= fs->fs_nclusters - 2)
    {
      *pfreeclusters = fs->fs_fsifreecount;
      return OK;
    }

  /* Otherwise, we will have to compute the number of free clusters */

  int ret = fat_computefreeclusters(fs);
  if (ret == OK)
    {
      *pfreeclusters = fs->fs_fsifreecount;
    }

  return ret;
}

/****************************************************************************
 * Name: fat_currentsector
 *
 * Description:
 *   Given the file position, set the correct current sector to access.
 *
 ****************************************************************************/

int fat_currentsector(struct fat_mountpt_s *fs, struct fat_file_s *ff,
                      off_t position)
{
  int sectoroffset;
  off_t cluster_start_sector;

  if (position <= ff->ff_size)
    {
      /* sectoroffset is the sector number offset into the current cluster */

      sectoroffset = SEC_NSECTORS(fs, position) & CLUS_NDXMASK(fs);

      /* The current sector is the first sector of the cluster plus
       * the sector offset
       */

      cluster_start_sector = fat_cluster2sector(fs, ff->ff_currentcluster);

      if (cluster_start_sector < 0)
        {
          return cluster_start_sector;
        }

      ff->ff_currentsector = cluster_start_sector + sectoroffset;

      /* The remainder is the number of sectors left in the cluster to be
       * read/written
       */

      ff->ff_sectorsincluster = fs->fs_fatsecperclus - sectoroffset;

      finfo("position=%" PRIdOFF " currentsector=%" PRIdOFF
            " sectorsincluster=%d\n",
            position, ff->ff_currentsector,
            ff->ff_sectorsincluster);

      return OK;
    }

  /* The position does not lie within the file */

  return -ENOSPC;
}
