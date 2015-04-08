/****************************************************************************
 * drivers/mtd/smart.c
 *
 * Sector Mapped Allocation for Really Tiny (SMART) Flash block driver.
 *
 *   Copyright (C) 2013-2014 Ken Pettit. All rights reserved.
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

#include <crc8.h>
#include <crc16.h>
#include <crc32.h>
#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/mtd/smart.h>
#include <nuttx/fs/smart.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

//#define CONFIG_SMART_LOCAL_CHECKFREE

#define SMART_STATUS_COMMITTED    0x80
#define SMART_STATUS_RELEASED     0x40
#define SMART_STATUS_CRC          0x20
#define SMART_STATUS_SIZEBITS     0x1C
#define SMART_STATUS_VERBITS      0x03

#if defined(CONFIG_SMART_CRC_16)
#define SMART_STATUS_VERSION      0x02
#elif defined(CONFIG_SMART_CRC_32)
#define SMART_STATUS_VERSION      0x03
#else
#define SMART_STATUS_VERSION      0x01
#endif

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
#define SMARTFS_FMT_WEAR_POS      36
#define SMART_WEAR_LEVEL_FORMAT_SIG 32
#define SMART_PARTNAME_SIZE         4

#define SMART_FIRST_DIR_SECTOR      3       /* First root directory sector */
#define SMART_FIRST_ALLOC_SECTOR    12      /* First logical sector number we will
                                             * use for assignment of requested Alloc
                                             * sectors.  All enries below this are
                                             * reserved (some for root dir entries,
                                             * other for our use, such as format
                                             * sector, etc. */

#if defined(CONFIG_MTD_SMART_READAHEAD) || (defined(CONFIG_DRVR_WRITABLE) && \
    defined(CONFIG_MTD_SMART_WRITEBUFFER))
#  define SMART_HAVE_RWBUFFER 1
#endif

#ifndef CONFIG_MTD_SMART_SECTOR_SIZE
#  define  CONFIG_MTD_SMART_SECTOR_SIZE 1024
#endif

#ifndef offsetof
#define offsetof(type, member) ( (size_t) &( ( (type *) 0)->member))
#endif

#define SMART_MAX_ALLOCS        6
//#define CONFIG_MTD_SMART_PACK_COUNTS

#ifndef CONFIG_MTD_SMART_ALLOC_DEBUG
#define smart_malloc(d, b, n)   kmm_malloc(b)
#define smart_free(d, p)        kmm_free(p)
#endif

#define SMART_WEAR_FULL_RELOCATE_THRESHOLD  8
#define SMART_WEAR_REORG_THRESHOLD          14
#define SMART_WEAR_MIN_LEVEL                5
#define SMART_WEAR_FORCE_REORG_THRESHOLD    1
#define SMART_WEAR_BIT_DIVIDE               1
#define SMART_WEAR_ZERO_MASK                0x0F
#define SMART_WEAR_BLOCK_MASK               0x01

/* Bit mapping for wear level bits */
/* These are defined to allow updating the wear leveling with the minimum
 * number of sector relocations / maximum use of 1 --> 0 transitions when
 * incrementing the wear level.
 *
 * 0:   1111        8:  1011
 * 1:   1110        9:  1010
 * 2:   1100       10:  0010
 * 3:   1000       11:  1101
 * 4:   0111       12:  1001
 * 5:   0110       13:  0001
 * 6:   0100       14:  0011
 * 7:   0000       15:  0101
 */

static const uint8_t gWearLevelToBitMap4[] =
{
  0x0F, 0x0E, 0x0C, 0x08,     /* Single bit erased (x3) */
  0x07, 0x06, 0x04, 0x00,     /* Single bit erased (x3) */
  0x0B, 0x0A, 0x02,           /* Single bit erased (x2) */
  0x0D, 0x09, 0x01,           /* Single bit erased (x2) */
  0x03,
  0x05
};

/* Map a Wear Level bit pattern back to the wear level */

static const uint8_t gWearBitToLevelMap4[] =
{
  7, 13, 10, 14, 6, 15, 5, 4,
  3, 12, 9,  8,  2, 11, 1, 0
};

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_MTD_SMART_MINIMIZE_RAM
struct smart_cache_s
{
  uint16_t              logical;          /* Logical sector number */
  uint16_t              physical;         /* Associated physical sector */
  uint16_t              birth;            /* The "birthday" of this entry */
};
#endif

/* When CRC is enabled, we allocate sectors in memory only and only write
 * to the device when an actual writesector is performed.  If during the
 * alloc process we do a physical write, we would either have to hold off on
 * writing the CRC value (which creates an invalid state on the device) or
 * we would have to perform a write, release re-write every time which would
 * increase the wear of the device 2x.
 */

#ifdef CONFIG_MTD_SMART_ENABLE_CRC
struct smart_allocsector_s
{
  struct smart_allocsector_s  *next;      /* Pointer to next alloc sector */
  uint16_t              logical;          /* Logical sector number */
  uint16_t              physical;         /* Associated physical sector */
};
#endif

struct smart_struct_s
{
  FAR struct mtd_dev_s *mtd;              /* Contained MTD interface */
  struct mtd_geometry_s geo;              /* Device geometry */

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_SMARTFS)
  uint32_t              unusedsectors;    /* Count of unused sectors (i.e. free when erased) */
  uint32_t              blockerases;      /* Count of unused sectors (i.e. free when erased) */
#endif
  uint16_t              neraseblocks;     /* Number of erase blocks or sub-sectors */
  uint16_t              lastallocblock;   /* Last  block we allocated a sector from */
  uint16_t              freesectors;      /* Total number of free sectors */
  uint16_t              releasesectors;   /* Total number of released sectors */
  uint16_t              mtdBlksPerSector; /* Number of MTD blocks per SMART Sector */
  uint16_t              sectorsPerBlk;    /* Number of sectors per erase block */
  uint16_t              sectorsize;       /* Sector size on device */
  uint16_t              totalsectors;     /* Total number of sectors on device */
  uint32_t              erasesize;        /* Size of an erase block */
  FAR uint8_t          *releasecount;     /* Count of released sectors per erase block */
  FAR uint8_t          *freecount;        /* Count of free sectors per erase block */
  FAR char             *rwbuffer;         /* Our sector read/write buffer */
  char                  partname[SMART_PARTNAME_SIZE]; /* Optional partition name */
  uint8_t               formatversion;    /* Format version on the device */
  uint8_t               formatstatus;     /* Indicates the status of the device format */
  uint8_t               namesize;         /* Length of filenames on this device */
  uint8_t               debuglevel;       /* Debug reporting level */
  uint8_t               availSectPerBlk;  /* Number of usable sectors per erase block */
#ifdef CONFIG_SMARTFS_MULTI_ROOT_DIRS
  uint8_t               rootdirentries;   /* Number of root directory entries */
  uint8_t               minor;            /* Minor number of the block entry */
#endif
#ifdef CONFIG_MTD_SMART_WEAR_LEVEL
  uint8_t               wearflags;        /* Indicates force erase of static blocks needed */
  uint8_t               minwearlevel;     /* Min level in the wear level bits */
  uint8_t               maxwearlevel;     /* Max level in the wear level bits */
  uint8_t              *wearstatus;       /* Array of wear leveling bits */
  uint32_t              uneven_wearcount; /* Number of times the the wear level has gone over max */
#endif
#ifdef CONFIG_MTD_SMART_ENABLE_CRC
  FAR struct smart_allocsector_s  *allocsector; /* Pointer to first alloc sector */
#endif
#ifndef CONFIG_MTD_SMART_MINIMIZE_RAM
  FAR uint16_t         *sMap;             /* Virtual to physical sector map */
#else
  FAR uint8_t          *sBitMap;          /* Virtual sector used bit-map */
  FAR struct smart_cache_s *sCache;       /* Sector cache */
  uint16_t              cache_entries;    /* Number of valid entries in the cache */
  uint16_t              cache_lastlog;    /* Keep track of the last sector accessed */
  uint16_t              cache_lastphys;   /* Keep the physical sector number also */
  uint16_t              cache_nextbirth;  /* Sector cache aging value */
#endif
#ifdef CONFIG_MTD_SMART_SECTOR_ERASE_DEBUG
  FAR uint8_t          *erasecounts;      /* Number of erases for each erase block */
#endif
#ifdef CONFIG_MTD_SMART_ALLOC_DEBUG
  size_t                bytesalloc;
  struct smart_alloc_s  alloc[SMART_MAX_ALLOCS];   /* Array of memory allocations */
#endif
};

#define SMART_WEARFLAGS_FORCE_REORG    0x01
#define SMART_WEARFLAGS_WRITE_NEEDED   0x02

#ifdef CONFIG_SMARTFS_MULTI_ROOT_DIRS
struct smart_multiroot_device_s
{
  FAR struct smart_struct_s*    dev;
  uint8_t                       rootdirnum;
};
#endif

/* Format 1 sector header definition */

#if SMART_STATUS_VERSION == 1
#define SMART_FMT_VERSION           1
struct smart_sect_header_s
{
  uint8_t               logicalsector[2]; /* The logical sector number */
  uint8_t               seq;              /* Incrementing sequence number */
  uint8_t               crc8;             /* CRC-8 or seq number MSB */
  uint8_t               status;           /* Status of this sector:
                                           * Bit 7:   1 = Not commited
                                           *          0 = commited
                                           * Bit 6:   1 = Not released
                                           *          0 = released
                                           * Bit 5:   Sector CRC enable
                                           * Bit 4-2: Sector size on volume
                                           * Bit 1-0: Format version (0x1) */
};
typedef uint8_t crc_t;

/* Format 2 sector header definition.  This is for a 16-bit CRC */

#elif SMART_STATUS_VERSION == 2
#define SMART_FMT_VERSION           2
struct smart_sect_header_s
{
  uint8_t               logicalsector[2]; /* The logical sector number */
  uint8_t               crc16[2];         /* CRC-16 for this sector */
  uint8_t               status;           /* Status of this sector:
                                           * Bit 7:   1 = Not commited
                                           *          0 = commited
                                           * Bit 6:   1 = Not released
                                           *          0 = released
                                           * Bit 5:   Sector CRC enable
                                           * Bit 4-2: Sector size on volume
                                           * Bit 1-0: Format version (0x2) */
  uint8_t               seq;              /* Incrementing sequence number */
};
typedef uint16_t crc_t;

/* Format 3 (32-bit) sector header definition.  Actually, this format
 * isn't used yet and will likely be changed to a format to support
 * NAND devices (possibly with an 18-bit sector size, allowing up to
 * 256K sectors on a larger NAND device, though this would take a fair
 * amount of RAM for management).
 */

#elif SMART_STATUS_VERSION == 3
#error "32-Bit mode not supported yet"
#define SMART_FMT_VERSION           3
struct smart_sect_header_s
{
  uint8_t               logicalsector[4]; /* The logical sector number */
  uint8_t               crc32[4];         /* CRC-32 for this sector */
  uint8_t               status;           /* Status of this sector:
                                           * Bit 7:   1 = Not commited
                                           *          0 = commited
                                           * Bit 6:   1 = Not released
                                           *          0 = released
                                           * Bit 5:   Sector CRC enable
                                           * Bit 4-2: Sector size on volume
                                           * Bit 1-0: Format version (0x3) */
  uint8_t               seq;              /* Incrementing sequence number */
};
typedef uint32_t crc_t;

#endif


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

static int smart_findfreephyssector(FAR struct smart_struct_s *dev, uint8_t canrelocate);

#ifdef CONFIG_FS_WRITABLE
static int smart_writesector(FAR struct smart_struct_s *dev, unsigned long arg);
#endif
static int smart_readsector(FAR struct smart_struct_s *dev, unsigned long arg);

#ifdef CONFIG_MTD_SMART_WEAR_LEVEL
static int smart_read_wearstatus(FAR struct smart_struct_s *dev);
static int smart_relocate_static_data(FAR struct smart_struct_s *dev, uint16_t block);
#endif

static int smart_relocate_sector(FAR struct smart_struct_s *dev,
    uint16_t oldsector, uint16_t newsector);

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
 * Name: smart_malloc
 *
 * Description:  Perform allocations and keep track of amount of allocated
 *               memory for this context.
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_SMART_ALLOC_DEBUG
FAR static void* smart_malloc(FAR struct smart_struct_s *dev,
                    size_t bytes, const char *name)
{
  void*   ret = kmm_malloc(bytes);
  uint8_t x;

  /* Keep track of the total allocation */

  if (ret != NULL)
    {
      dev->bytesalloc += bytes;
    }

  /* Keep track of individual allocs */

  for (x = 0; x < SMART_MAX_ALLOCS; x++)
    {
      if (dev->alloc[x].ptr == NULL)
        {
          dev->alloc[x].ptr = ret;
          dev->alloc[x].size = bytes;
          dev->alloc[x].name = name;
          break;
        }
    }

  fdbg("SMART alloc: %ld\n", dev->bytesalloc);
  return ret;
}
#endif

/****************************************************************************
 * Name: smart_free
 *
 * Description:  Perform smart memory free operation.
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_SMART_ALLOC_DEBUG
static void smart_free(FAR struct smart_struct_s *dev, FAR void* ptr)
{
  uint8_t x;

  for (x = 0; x < SMART_MAX_ALLOCS; x++)
    {
      if (dev->alloc[x].ptr == ptr)
        {
          dev->alloc[x].ptr = NULL;
          dev->bytesalloc -= dev->alloc[x].size;
          kmm_free(ptr);
          break;
        }
    }
}
#endif

/****************************************************************************
 * Name: smart_set_count
 *
 * Description: Set either the freecount or releasecount value for the
 *              specified eraseblock (depending on which pointer is passed).
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_SMART_PACK_COUNTS
static void smart_set_count(FAR struct smart_struct_s *dev, FAR uint8_t* pCount,
                            uint16_t block, uint8_t count)
{
  if (dev->sectorsPerBlk > 16)
    {
      pCount[block] = count;
    }
  else
    {
      /* Save the lower 4 bits of the count in a shared byte */

      if (block & 0x01)
        {
          pCount[block >> 1] = (pCount[block >> 1] & 0xF0) | (count & 0x0F);
        }
      else
        {
          pCount[block >> 1] = (pCount[block >> 1] & 0x0F) | ((count & 0x0F) << 4);
        }

      /* If we have 16 sectors per block, then the upper bit (representing 16)
       * all get packed into shared bytes.
       */

      if (dev->sectorsPerBlk == 16)
        {
          if (count == 16)
            {
              pCount[(dev->geo.neraseblocks >> 1) + (block>>3)] |= 1 << (block & 0x07);
            }
          else
            {
              pCount[(dev->geo.neraseblocks >> 1) + (block>>3)] &= ~(1 << (block & 0x07));
            }
        }
    }
}
#endif

/****************************************************************************
 * Name: smart_get_count
 *
 * Description: Get either the freecount or releasecount value for the
 *              specified eraseblock (depending on which pointer is passed).
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_SMART_PACK_COUNTS
static uint8_t smart_get_count(FAR struct smart_struct_s *dev,
                    FAR uint8_t* pCount, uint16_t block)
{
  uint8_t   count;

  if (dev->sectorsPerBlk > 16)
    {
      count = pCount[block];
    }
  else
    {
      /* Save the lower 4 bits of the count in a shared byte */

      if (block & 0x01)
        {
          count = pCount[block >> 1] & 0x0F;
        }
      else
        {
          count = pCount[block >> 1] >> 4;
        }

      /* If we have 16 sectors per block, then the upper bit (representing 16)
       * all get packed into shared bytes.
       */

      if (dev->sectorsPerBlk == 16)
        {
          if (pCount[(dev->geo.neraseblocks >> 1) + (block>>3)] & (1 << (block & 0x07)))
            {
              count |= 0x10;
            }
        }
    }

  return count;
}
#endif

/****************************************************************************
 * Name: smart_add_count
 *
 * Description: Add the specified value to and eraseblock count.
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_SMART_PACK_COUNTS
static void smart_add_count(struct smart_struct_s *dev, uint8_t* pCount,
                            uint16_t block, int adder)
{
  int16_t   value;

  value = smart_get_count(dev, pCount, block) + adder;
  smart_set_count(dev, pCount, block, value);
}
#endif

/****************************************************************************
 * Name: smart_checkfree
 *
 * Description: A debug routine for validating the free sector count used
 *              during development of the wear leveling code.
 *
 ****************************************************************************/

#ifdef CONFIG_SMART_LOCAL_CHECKFREE
int smart_checkfree(FAR struct smart_struct_s *dev, int lineno)
{
  uint16_t        x, freecount;
#ifdef CONFIG_DEBUG_FS
  uint16_t        blockfree, blockrelease;
  static uint16_t prev_freesectors = 0;
  static uint16_t prev_releasesectors = 0;
  static uint8_t  *prev_freecount = NULL;
  static uint8_t  *prev_releasecount = NULL;
#endif

  freecount = 0;
  for (x = 0; x < dev->neraseblocks; x++)
    {
#ifdef CONFIG_MTD_SMART_PACK_COUNTS
      freecount += smart_get_count(dev, dev->freecount, x);
#else
      freecount += dev->freecount[x];
#endif
    }

  /* Test if the calculated freesectors equals the reported value */

#ifdef CONFIG_DEBUG_FS
  if (freecount != dev->freesectors)
    {
      fdbg("Free count incorrect in line %d!  Calculated=%d, dev->freesectors=%d\n",
           lineno, freecount, dev->freesectors);

      /* Determine what changed from the last time which caused this error */

      fdbg("   ... Prev freesectors=%d, prev releasesectors=%d\n",
           prev_freesectors, prev_releasesectors);

      if (prev_freecount)
        {
          for (x = 0; x < dev->neraseblocks; x++)
            {
#ifdef CONFIG_MTD_SMART_PACK_COUNTS
              blockfree = smart_get_count(dev, dev->freecount, x);
              blockrelease = smart_get_count(dev, dev->releasecount, x);
#else
              blockfree = dev->freecount[x];
              blockrelease = dev->releasecount[x];
#endif
              if (prev_freecount[x] != blockfree || prev_releasecount[x] != blockrelease)
                {
                  /* This block's values are different from the last time ... report it */

                  fdbg("   ... Block %d:  Old Free=%d, old release=%d,    New free=%d, new release = %d\n",
                      x, prev_freecount[x], prev_releasecount[x], blockfree, blockrelease);
                }
            }
        }

      /* Modifiy the freesector count to reflect the actual calculated freecount
         to get us back in line.
       */

      dev->freesectors = freecount;
      return -EIO;
    }

  /* Make a copy of the freecount and releasecount arrays to compare the
   * differences between successive calls so we can evaluate what changed
   * in the event an error is detected.
   */

  if (prev_freecount == NULL)
    {
      prev_freecount = (FAR uint8_t *) smart_malloc(dev, dev->neraseblocks << 1, "Free backup");
      prev_releasecount = prev_freecount + dev->neraseblocks;
    }

  if (prev_freecount != NULL)
    {
      for (x = 0; x < dev->neraseblocks; x++)
        {
#ifdef CONFIG_MTD_SMART_PACK_COUNTS
          prev_freecount[x] = smart_get_count(dev, dev->freecount, x);
          prev_releasecount[x] = smart_get_count(dev, dev->releasecount, x);
#else
          prev_freecount[x] = dev->freecount[x];
          prev_releasecount[x] = dev->releasecount[x];
#endif
        }
    }

  /* Save the previous freesectors count */

  prev_freesectors = dev->freesectors;
  prev_releasesectors = dev->releasesectors;
#endif

  return OK;
}
#endif

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

  fvdbg("Read %d blocks starting at block %d\n", mtdBlocks, mtdStartBlock);
  nread = MTD_BREAD(dev->mtd, mtdStartBlock, mtdBlocks, buffer);
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
static ssize_t smart_write(FAR struct inode *inode,
                FAR const unsigned char *buffer,
                size_t start_sector, unsigned int nsectors)
{
  FAR struct smart_struct_s *dev;
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
  dev = ((FAR struct smart_multiroot_device_s*) inode->i_private)->dev;
#else
  dev = (FAR struct smart_struct_s *)inode->i_private;
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
  FAR struct smart_struct_s *dev;
  uint32_t  erasesize;

  fvdbg("Entry\n");

  DEBUGASSERT(inode);
  if (geometry)
    {
#ifdef CONFIG_SMARTFS_MULTI_ROOT_DIRS
      dev = ((FAR struct smart_multiroot_device_s*) inode->i_private)->dev;
#else
      dev = (FAR struct smart_struct_s *)inode->i_private;
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
          erasesize = 262144;
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

static int smart_setsectorsize(FAR struct smart_struct_s *dev, uint16_t size)
{
  uint32_t  erasesize;
  uint32_t  totalsectors;
  uint32_t  allocsize;

  /* Validate the size isn't zero so we don't divide by zero below */

  if (size == 0)
    {
      size = CONFIG_MTD_SMART_SECTOR_SIZE;
    }

  if (size == dev->sectorsize)
    {
      return OK;
    }

  erasesize = dev->geo.erasesize;
  dev->neraseblocks = dev->geo.neraseblocks;

  /* Most FLASH devices have erase size of 64K, but geo.erasesize is only
   * 16 bits, so it will be zero
   */

  if (erasesize == 0)
    {
      erasesize = 262144;
    }

  dev->erasesize = erasesize;
  dev->sectorsize = size;
  dev->mtdBlksPerSector = dev->sectorsize / dev->geo.blocksize;
  if (erasesize / dev->sectorsize > 256)
    {
      /* We can't throw a dbg message here becasue it is too early.
       * set the erasesize to zero and exit, then we will detect
       * it during mksmartfs or mount.
       */

      dev->erasesize = 0;
      dev->sectorsPerBlk = 256;
      dev->availSectPerBlk = 255;
    }
  else
    {
      /* Set the sectors per erase block and available sectors per erase block */

      dev->sectorsPerBlk = erasesize / dev->sectorsize;
      if (dev->sectorsPerBlk == 256)
        {
          dev->availSectPerBlk = 255;
        }
      else
        {
          dev->availSectPerBlk = dev->sectorsPerBlk;
        }
    }

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_SMARTFS)
  dev->unusedsectors = 0;
  dev->blockerases = 0;
#endif

  /* Release any existing rwbuffer and sMap */

#ifndef CONFIG_MTD_SMART_MINIMIZE_RAM
  if (dev->sMap != NULL)
    {
      smart_free(dev, dev->sMap);
      dev->sMap = NULL;
    }
#else
  if (dev->sBitMap != NULL)
    {
      smart_free(dev, dev->sBitMap);
      dev->sBitMap = NULL;
    }

  dev->cache_entries = 0;
  dev->cache_lastlog = 0xFFFF;
  dev->cache_nextbirth = 0;
#endif

  if (dev->rwbuffer != NULL)
    {
      smart_free(dev, dev->rwbuffer);
      dev->rwbuffer = NULL;
    }

#ifdef CONFIG_MTD_SMART_WEAR_LEVEL
  if (dev->wearstatus != NULL)
    {
      smart_free(dev, dev->wearstatus);
      dev->wearstatus = NULL;
    }
#endif

  /* Allocate a virtual to physical sector map buffer.  Also allocate
   * the storage space for releasecount and freecounts.
   */

  totalsectors = dev->neraseblocks * dev->sectorsPerBlk;

  /* Validate the number of total sectors is small enough for a uint16_t */

  if (totalsectors > 65536)
    {
      dbg("Invalid SMART sector count %ld\n", totalsectors);
      return -EINVAL;
    }
  else if (totalsectors == 65536)
    {
      /* Special case.  We allow 65536 sectors and simply waste 2 sectors
       * to allow a smaller sector size with almost maximum flash usage.
       */

      totalsectors -= 2;
    }

  dev->totalsectors = (uint16_t) totalsectors;

#ifndef CONFIG_MTD_SMART_MINIMIZE_RAM
  allocsize = dev->neraseblocks << 1;
  dev->sMap = (FAR uint16_t *) smart_malloc(dev, totalsectors * sizeof(uint16_t) +
              allocsize, "Sector map");
  if (!dev->sMap)
    {
      fdbg("Error allocating SMART virtual map buffer\n");
      goto errexit;
    }

  dev->releasecount = (FAR uint8_t *) dev->sMap + (totalsectors * sizeof(uint16_t));
  dev->freecount = dev->releasecount + dev->neraseblocks;
#else
  dev->sBitMap = (FAR uint8_t *) smart_malloc(dev, (totalsectors+7) >> 3, "Sector Bitmap");
  if (dev->sBitMap == NULL)
    {
      fdbg("Error allocating SMART sector cache\n");
      goto errexit;
    }

  /* Calculate the alloc size of the freesector and release sector arrays */

#ifdef CONFIG_MTD_SMART_PACK_COUNTS
  if (dev->sectorsPerBlk > 16)
    {
      allocsize = dev->neraseblocks << 1;
    }
  else if (dev->sectorsPerBlk == 16)
    {
      allocsize = dev->neraseblocks + (dev->neraseblocks >> 2);
    }
  else
    {
      allocsize = dev->neraseblocks;
    }

#else
  allocsize = dev->neraseblocks << 1;
#endif

  /* Allocate the sector cache */

  if (dev->sCache == NULL)
    {
      dev->sCache = (FAR struct smart_cache_s *) smart_malloc(dev,
        CONFIG_MTD_SMART_SECTOR_CACHE_SIZE * sizeof(struct smart_cache_s) +
        allocsize, "Sector Cache");
    }

  if (!dev->sCache)
    {
      fdbg("Error allocating SMART sector cache\n");
      goto errexit;
    }

  dev->releasecount = (FAR uint8_t *) dev->sCache + (CONFIG_MTD_SMART_SECTOR_CACHE_SIZE *
      sizeof(struct smart_cache_s));

#ifdef CONFIG_MTD_SMART_PACK_COUNTS
  if (dev->sectorsPerBlk > 16)
    {
      dev->freecount = dev->releasecount + dev->neraseblocks;
    }
  else if (dev->sectorsPerBlk == 16)
    {
      dev->freecount = dev->releasecount + (dev->neraseblocks >> 1) + (dev->neraseblocks >> 3);
    }
  else
    {
      dev->freecount = dev->releasecount + (dev->neraseblocks >> 1);
    }

#else
  dev->freecount = dev->releasecount + dev->neraseblocks;
#endif

#endif  /* CONFIG_MTD_SMART_MINIMIZE_RAM */

#ifdef CONFIG_MTD_SMART_SECTOR_ERASE_DEBUG
  /* Allocate a buffer to hold the erase counts */

  if (dev->erasecounts == NULL)
    {
      dev->erasecounts = (FAR uint8_t *) smart_malloc(dev, dev->neraseblocks, "Erase counts");
    }

  if (!dev->erasecounts)
    {
      fdbg("Error allocating erase count array\n");
      goto errexit;
    }

  memset(dev->erasecounts, 0, dev->neraseblocks);
#endif

#ifdef CONFIG_MTD_SMART_WEAR_LEVEL
  /* Allocate the wear leveling status array */

  dev->wearstatus = (FAR uint8_t *) smart_malloc(dev, dev->neraseblocks >>
      SMART_WEAR_BIT_DIVIDE, "Wear status");
  if (!dev->wearstatus)
    {
      fdbg("Error allocating wear level status array\n");
      goto errexit;
    }

  memset(dev->wearstatus, CONFIG_SMARTFS_ERASEDSTATE, dev->neraseblocks >>
         SMART_WEAR_BIT_DIVIDE);
  dev->wearflags = 0;
  dev->uneven_wearcount = 0;
#endif

  /* Allocate a read/write buffer */

  dev->rwbuffer = (FAR char *) smart_malloc(dev, size, "RW Buffer");
  if (!dev->rwbuffer)
    {
      fdbg("Error allocating SMART read/write buffer\n");
      goto errexit;
    }

  return OK;

  /* On error for any allocation, we jump here and free anything that had
   * previously been allocated.
   */

errexit:

#ifndef CONFIG_MTD_SMART_MINIMIZE_RAM
  if (dev->sMap)
    {
      smart_free(dev, dev->sMap);
    }
#else
  if (dev->sBitMap)
    {
      smart_free(dev, dev->sBitMap);
    }

  if (dev->sCache)
    {
      smart_free(dev, dev->sCache);
    }
#endif

#ifdef CONFIG_MTD_SMART_WEAR_LEVEL
  if (dev->wearstatus)
    {
      smart_free(dev, dev->wearstatus);
    }
#endif

#ifdef CONFIG_MTD_SMART_SECTOR_ERASE_DEBUG
  if (dev->erasecounts)
    {
      smart_free(dev, dev->erasecounts);
    }
#endif

  kmm_free(dev);
  return -ENOMEM;
}

/****************************************************************************
 * Name: smart_bytewrite
 *
 * Description: Writes a non-page size count of bytes to the underlying
 *              MTD device.  If the MTD driver supports a direct impl of
 *              write, then it uses it, otherwise it does a read-modify-write
 *              and depends on the architecture of the flash to only program
 *              bits that actually changed.
 *
 ****************************************************************************/

static ssize_t smart_bytewrite(FAR struct smart_struct_s *dev, size_t offset,
        int nbytes, FAR const uint8_t *buffer)
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

      uint32_t  startblock;
      uint16_t  nblocks;

      /* First calculate the start block and number of blocks affected */

      startblock = offset / dev->geo.blocksize;
      nblocks    = (offset - startblock * dev->geo.blocksize + nbytes +
                    dev->geo.blocksize-1) / dev->geo.blocksize;

      DEBUGASSERT(nblocks <= dev->mtdBlksPerSector);

      /* Do a block read */

      ret = MTD_BREAD(dev->mtd, startblock, nblocks, (FAR uint8_t *) dev->rwbuffer);
      if (ret < 0)
        {
          fdbg("Error %d reading from device\n", -ret);
          goto errout;
        }

      /* Modify the data */

      memcpy(&dev->rwbuffer[offset - startblock * dev->geo.blocksize], buffer, nbytes);

      /* Write the data back to the device */

      ret = MTD_BWRITE(dev->mtd, startblock, nblocks, (FAR uint8_t *) dev->rwbuffer);
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
 * Name: smart_add_sector_to_cache
 *
 * Description: Adds a logical to physical sector maaping to the sector
 *              map cache.  The cache is used to minimize RAM by eliminating
 *              a one-to-one mapping of all logical sectors and only keeping
 *              a fixed number of mappings per the
 *              CONFIG_MTD_SMART_SECTOR_CACHE_SIZE parameter.  Sectors are
 *              automatically managed and removed based on the time since
 *              they were accessed last.
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_SMART_MINIMIZE_RAM
static int smart_add_sector_to_cache(FAR struct smart_struct_s *dev,
            uint16_t logical, uint16_t physical, int line)
{
  uint16_t    index, x;
  uint16_t    oldest;

  /* If we aren't full yet, just add the sector to the end of the list */

  index = 1;
  if (dev->cache_entries < CONFIG_MTD_SMART_SECTOR_CACHE_SIZE)
    {
      index = dev->cache_entries++;
    }
  else
    {
      /* Cache is full.  We must find the least accessed entry and replace it */

      oldest = 0xFFFF;
      for (x = 0; x < CONFIG_MTD_SMART_SECTOR_CACHE_SIZE; x++)
        {
          /* Never replace cache entries for system sectors */

          if (dev->sCache[x].logical < SMART_FIRST_ALLOC_SECTOR)
            continue;

          /* If the hit count is zero, then choose this entry */

          if (dev->sCache[x].birth < oldest)
            {
              oldest = dev->sCache[x].birth;
              index = x;
            }
        }
    }

  /* Now add the sector at index */

  dev->sCache[index].logical = logical;
  dev->sCache[index].physical = physical;
  dev->sCache[index].birth = dev->cache_nextbirth++;
  dev->cache_lastlog = logical;
  dev->cache_lastphys = physical;
  if (dev->debuglevel > 1)
    {
      dbg("Add Cache sector:  Log=%d, Phys=%d at index %d from line %d\n",
          logical, physical, index, line);
    }

  /* Test if the birthdays need to be adjusted */

  if (oldest >= CONFIG_MTD_SMART_SECTOR_CACHE_SIZE + 1024)
    {
      for (x = 0; x < dev->cache_entries; x++)
        {
          dev->sCache[x].birth -= 1024;
        }

      dev->cache_nextbirth -= 1024;
    }

  return index;
}
#endif

/****************************************************************************
 * Name: smart_cache_lookup
 *
 * Description: Perform a cache lookup for the requested logical sector.
 *              If the sector is in the cache, then update the hitcount and
 *              return the physical mapping.  If a cache miss occurs, then
 *              the routine will scan the volume to find the logical sector
 *              and add / replace a cache entry with the newly located sector.
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_SMART_MINIMIZE_RAM
static uint16_t smart_cache_lookup(FAR struct smart_struct_s *dev, uint16_t logical)
{
  int       ret;
  uint16_t  block, sector;
  uint16_t  x, physical, logicalsector;
  struct    smart_sect_header_s header;
  size_t    readaddress;

  physical = 0xFFFF;

  /* Test if searching for the last sector used */

  if (logical == dev->cache_lastlog)
    {
      return dev->cache_lastphys;
    }

  /* First search for the entry in the cache */

  for (x = 0; x < dev->cache_entries; x++)
    {
      if (dev->sCache[x].logical == logical)
        {
          /* Entry found in the cache.  Grab the physical mapping. */

          physical = dev->sCache[x].physical;
          break;
        }
    }

  /* If the entry wasn't found in the cache, then we must search the volume
   * for it and add it to the cache.
   */

  if (physical == 0xFFFF)
    {
      /* Now scan the MTD device.  Instead of scanning start to end, we
       * span the erase blocks and read one sector from each at a time.
       * this helps speed up the search on volumes that aren't full
       * because of sector allocation scheme will use the lower sector
       * numbers in each erase block first.
       */

      for (sector = 0; sector < dev->sectorsPerBlk && physical == 0xFFFF; sector++)
        {
          /* Now scan across each erase block */

          for (block = 0; block < dev->geo.neraseblocks; block++)
            {
              /* Calculate the read address for this sector */

              readaddress = block * dev->erasesize +
                  sector * CONFIG_MTD_SMART_SECTOR_SIZE;

              /* Read the header for this sector */

              ret = MTD_READ(dev->mtd, readaddress,
                  sizeof(struct smart_sect_header_s), (FAR uint8_t *) &header);
              if (ret != sizeof(struct smart_sect_header_s))
                {
                  goto err_out;
                }

              /* Get the logical sector number for this physical sector */

              logicalsector = *((FAR uint16_t *) header.logicalsector);
#if CONFIG_SMARTFS_ERASEDSTATE == 0x00
              if (logicalsector == 0)
                {
                  continue;
                }
#endif

              /* Test if this sector has been committed */

              if ((header.status & SMART_STATUS_COMMITTED) ==
                      (CONFIG_SMARTFS_ERASEDSTATE & SMART_STATUS_COMMITTED))
                {
                  continue;
                }

              /* Test if this sector has been release and skip it if it has */

              if ((header.status & SMART_STATUS_RELEASED) !=
                      (CONFIG_SMARTFS_ERASEDSTATE & SMART_STATUS_RELEASED))
                {
                  continue;
                }

              if ((header.status & SMART_STATUS_VERBITS) != SMART_STATUS_VERSION)
                {
                  continue;
                }

              /* Test if this is the sector we are looking for */

              if (logicalsector == logical)
                {
                  /* This is the sector we are looking for!  Add it to the cache */

                  physical = block * dev->sectorsPerBlk + sector;
                  smart_add_sector_to_cache(dev, logical, physical, __LINE__ );
                  break;
                }
            }
        }
    }

  /* Update the last logical sector found variable */

  dev->cache_lastlog = logical;
  dev->cache_lastphys = physical;

err_out:
  return physical;
}
#endif

/****************************************************************************
 * Name: smart_update_cache
 *
 * Description: Updates a cache entry (if present) replacing the logical
 *              sector's physical sector mapping with the new one provided.
 *              This does not affect the hit count.
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_SMART_MINIMIZE_RAM
static void smart_update_cache(FAR struct smart_struct_s *dev, uint16_t
    logical, uint16_t physical)
{
  uint16_t    x;

  /* Scan through all cache entries and find the logical sector entry */

  for (x = 0; x < dev->cache_entries; x++)
    {
      if (dev->sCache[x].logical == logical)
        {
          /* Entry found.  Update it's physical mapping */

          dev->sCache[x].physical = physical;

          /* If we are freeing a sector, then remove the logical entry from
             the cache.
           */

          if (physical == 0xFFFF)
            {
                dev->sCache[x].logical = dev->sCache[dev->cache_entries-1].logical;
                dev->sCache[x].physical = dev->sCache[dev->cache_entries-1].physical;
                dev->cache_entries--;
            }

          if (dev->debuglevel > 1)
            {
              dbg("Update Cache:  Log=%d, Phys=%d at index %d\n", logical, physical, x);
            }

          break;
        }
    }

  if (dev->cache_lastlog == logical)
    {
      dev->cache_lastphys = physical;
    }
}
#endif

/****************************************************************************
 * Name: smart_get_wear_level
 *
 * Description: Gets the wear level of the specified block.  Wear levels are
 *              encoded to minimize the number of zero to one transitions,
 *              possibly allowing updates to made on NOR devices that have
 *              no CRC enabled.
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_SMART_WEAR_LEVEL
static uint8_t smart_get_wear_level(FAR struct smart_struct_s *dev, uint16_t block)
{
  uint8_t   bits;

  bits = dev->wearstatus[block >> SMART_WEAR_BIT_DIVIDE];
  if (block & 0x01)
    {
      /* Use the upper nibble */

      bits >>= 4;
    }
  else
    {
      /* Use the lower nibble */

      bits &= 0x0F;
    }

  /* Lookup and return the level using the BitToLevel map */

  return gWearBitToLevelMap4[bits];
}
#endif

/****************************************************************************
 * Name: smart_find_wear_minmax
 *
 * Description: Find the minimum and maximum wear levels.  This is used when
 *              we increment the wear level of a minimum value block so that
 *              we can detect if a new minimum exists and perform normalization
 *              of the wear-levels.
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_SMART_WEAR_LEVEL
static void smart_find_wear_minmax(FAR struct smart_struct_s *dev)
{
  uint16_t   x;
  unsigned char level;

  dev->minwearlevel = 15;
  dev->maxwearlevel = 0;

  /* Loop through all erase blocks and find min / max level */

  for (x = 0; x < dev->geo.neraseblocks; x++)
    {
      /* Find wear level of the minimum worn block */

      level = smart_get_wear_level(dev, x);
      if (level < dev->minwearlevel)
        {
          dev->minwearlevel = level;
        }

      /* Find wear level of the maximum worn block */

      if (level > dev->maxwearlevel)
        {
          dev->maxwearlevel = level;
        }
    }

#ifdef CONFIG_MTD_SMART_SECTOR_ERASE_DEBUG
  /* Also adjust the erase counts */
  level = 255;
  for (x = 0; x < dev->geo.neraseblocks; x++)
    {
      if (dev->erasecounts[x] < level)
        {
          level = dev->erasecounts[x];
        }
    }

  if (level != 0)
    {
      for (x = 0; x < dev->geo.neraseblocks; x++)
        {
          dev->erasecounts[x] -= level;
        }
    }

#endif
}
#endif

/****************************************************************************
 * Name: smart_set_wear_level
 *
 * Description: Sets the wear level of the specified block.  The wear level
 *              is a 4-bit field packed 2 entries per byte and is mapped to
 *              a bit field which minimizes the number of 0 to 1 transitions
 *              such that entries can be updated on a NOR flash withough the
 *              need to relocated the format sector (assuming CRC is not
 *              enabled, in which case a relocated is needed for ANY change).
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_SMART_WEAR_LEVEL
static int smart_set_wear_level(FAR struct smart_struct_s *dev, uint16_t block,
                                uint8_t level)
{
  uint8_t   bits, oldlevel;

  /* Get the old wear level to test if we need to update min / max */

  oldlevel = smart_get_wear_level(dev, block);

  /* Get the bit map for this wear level from the static map array */

  if (level > 15)
    {
      dbg("Fatal Design Error!  Wear level > 15, block=%d\n", block);

      /* This is a design flaw, but we still allow processing, otherwise we
       * will corrupt the volume.  It's better to have a few blocks that are
       * worn a bit more than to create an error condition on the volume.
       *
       * Set the level to the maximum value and add to the un-even wear count
       * to keep track of the number of times this has happened.
       */

      level = 15;
      dev->uneven_wearcount++;
    }

  bits = gWearLevelToBitMap4[level];

  if (block & 0x01)
    {
      /* Use the upper nibble */

      dev->wearstatus[block >> SMART_WEAR_BIT_DIVIDE] &= 0x0F;
      dev->wearstatus[block >> SMART_WEAR_BIT_DIVIDE] |= bits << 4;
    }
  else
    {
      /* Use the lower nibble */

      dev->wearstatus[block >> SMART_WEAR_BIT_DIVIDE] &= 0xF0;
      dev->wearstatus[block >> SMART_WEAR_BIT_DIVIDE] |= bits;
    }

  /* Mark wear bits as dirty */

  dev->wearflags |= SMART_WEARFLAGS_WRITE_NEEDED;

  /* Test if min / max need to be updated */

  if (oldlevel + 1 == level)
    {
      /* Test if max needs to be updated */

      if (level > dev->maxwearlevel)
        {
          dev->maxwearlevel = level;
        }

      /* Test if this was the min level.  If it was, then
         we need to rescan for min. */

      if (oldlevel == dev->minwearlevel)
        {
          smart_find_wear_minmax(dev);

          if (oldlevel != dev->minwearlevel)
              fvdbg("##### New min wear level = %d\n", dev->minwearlevel);
        }
    }

  return 0;
}
#endif

/****************************************************************************
 * Name: smart_scan
 *
 * Description: Performs a scan of the MTD device searching for format
 *              information and fills in logical sector mapping, freesector
 *              count, etc.
 *
 ****************************************************************************/

static int smart_scan(FAR struct smart_struct_s *dev)
{
  int       sector;
  int       ret;
  uint16_t  totalsectors;
  uint16_t  sectorsize, prerelease;
  uint16_t  logicalsector;
  uint16_t  loser;
  uint32_t  readaddress;
  uint32_t  offset;
  uint16_t  seq1;
  uint16_t  seq2;
  struct    smart_sect_header_s header;
#ifdef CONFIG_MTD_SMART_MINIMIZE_RAM
  int       dupsector;
  uint16_t  duplogsector;
#endif
#ifdef CONFIG_SMARTFS_MULTI_ROOT_DIRS
  int       x;
  char      devname[22];
  FAR struct smart_multiroot_device_s *rootdirdev;
#endif

  fvdbg("Entry\n");

  /* Find the sector size on the volume by reading headers from
   * sectors of decreasing size.  On a formatted volume, the sector
   * size is saved in the header status byte of seach sector, so
   * by starting with the largest supported sector size and
   * decreasing from there, we will be sure to find data that is
   * a header and not sector data.
   */

  sectorsize = 0xFFFF;
  offset = 16384;

  while (sectorsize == 0xFFFF)
    {
      readaddress = 0;

      while (readaddress < dev->erasesize * dev->geo.neraseblocks)
        {
          /* Read the next sector from the device */

          ret = MTD_READ(dev->mtd, 0, sizeof(struct smart_sect_header_s),
                         (FAR uint8_t *) &header);
          if (ret != sizeof(struct smart_sect_header_s))
            {
              goto err_out;
            }

          if (header.status != CONFIG_SMARTFS_ERASEDSTATE)
            {
              sectorsize = (header.status & SMART_STATUS_SIZEBITS) << 7;
              break;
            }

          readaddress += offset;
        }

      offset >>= 1;
      if (offset < 256 && sectorsize == 0xFFFF)
        {
          sectorsize = CONFIG_MTD_SMART_SECTOR_SIZE;
        }
    }

  /* Now set the sectorsize and other sectorsize derived variables */

  ret = smart_setsectorsize(dev, sectorsize);
  if (ret != OK)
    {
      goto err_out;
    }

  /* Initialize the device variables */

  totalsectors = dev->totalsectors;
  dev->formatstatus = SMART_FMT_STAT_NOFMT;
  dev->freesectors = dev->availSectPerBlk * dev->geo.neraseblocks;
  dev->releasesectors = 0;

  /* Initialize the freecount and releasecount arrays */

  for (sector = 0; sector < dev->neraseblocks; sector++)
    {
      if (sector == dev->neraseblocks - 1 && dev->totalsectors == 65534)
        {
          prerelease = 2;
        }
      else
        {
          prerelease = 0;
        }

#ifdef CONFIG_MTD_SMART_PACK_COUNTS
      smart_set_count(dev, dev->freecount, sector, dev->availSectPerBlk - prerelease);
      smart_set_count(dev, dev->releasecount, sector, prerelease);
#else
      dev->freecount[sector] = dev->availSectPerBlk - prerelease;
      dev->releasecount[sector] = prerelease;
#endif
    }

  /* Initialize the sector map */

#ifndef CONFIG_MTD_SMART_MINIMIZE_RAM
  for (sector = 0; sector < totalsectors; sector++)
    {
      dev->sMap[sector] = -1;
    }
#else
  /* Clear all logical sector used bits */

  memset(dev->sBitMap, 0, (dev->totalsectors + 7) >> 3);
#endif

  /* Now scan the MTD device */

  for (sector = 0; sector < totalsectors; sector++)
    {
      fvdbg("Scan sector %d\n", sector);

      /* Calculate the read address for this sector */

      readaddress = sector * dev->mtdBlksPerSector * dev->geo.blocksize;

      /* Read the header for this sector */

      ret = MTD_READ(dev->mtd, readaddress, sizeof(struct smart_sect_header_s),
                     (FAR uint8_t *) &header);
      if (ret != sizeof(struct smart_sect_header_s))
        {
          goto err_out;
        }

      /* Get the logical sector number for this physical sector */

      logicalsector = *((FAR uint16_t *) header.logicalsector);
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

#ifdef CONFIG_MTD_SMART_PACK_COUNTS
      smart_add_count(dev, dev->freecount, sector / dev->sectorsPerBlk, -1);
#else
      dev->freecount[sector / dev->sectorsPerBlk]--;
#endif
      dev->freesectors--;

      /* Test if this sector has been release and if it has,
       * update the erase block's releasecount.
       */

      if ((header.status & SMART_STATUS_RELEASED) !=
              (CONFIG_SMARTFS_ERASEDSTATE & SMART_STATUS_RELEASED))
        {
          /* Keep track of the total number of released sectors and
           * released sectors per erase block.
           */

          dev->releasesectors++;
#ifdef CONFIG_MTD_SMART_PACK_COUNTS
          smart_add_count(dev, dev->releasecount, sector / dev->sectorsPerBlk, 1);
#else
          dev->releasecount[sector / dev->sectorsPerBlk]++;
#endif
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
                         (FAR uint8_t*) dev->rwbuffer);
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
              if (dev->partname[0] != '\0')
                {
                  snprintf(dev->rwbuffer, sizeof(devname), "/dev/smart%d%sd%d",
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

              rootdirdev = (struct smart_multiroot_device_s*) smart_malloc(dev,
                                sizeof(*rootdirdev), "Root Dir");
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

#ifndef CONFIG_MTD_SMART_MINIMIZE_RAM
      if (dev->sMap[logicalsector] != 0xFFFF)
#else
      if (dev->sBitMap[logicalsector >> 3] & (1 << (logicalsector & 0x07)))
#endif
        {
          /* Uh-oh, we found more than 1 physical sector claiming to be
           * the same logical sector.  Use the sequence number information
           * to resolve who wins.
           */


#if SMART_STATUS_VERSION == 1
          if (header.status & SMART_STATUS_CRC)
            {
              seq2 = header.seq;
            }
          else
            {
              seq2 = *((FAR uint16_t *) &header.seq);
            }
#else
          seq2 = header.seq;
#endif

          /* We must re-read the 1st physical sector to get it's seq number */

#ifndef CONFIG_MTD_SMART_MINIMIZE_RAM
          readaddress = dev->sMap[logicalsector]  * dev->mtdBlksPerSector * dev->geo.blocksize;
#else
          /* For minimize RAM, we have to rescan to find the 1st sector claiming to
           * be this logical sector.
           */

          for (dupsector = 0; dupsector < sector; dupsector++)
            {
              /* Calculate the read address for this sector */

              readaddress = dupsector * dev->mtdBlksPerSector * dev->geo.blocksize;

              /* Read the header for this sector */

              ret = MTD_READ(dev->mtd, readaddress, sizeof(struct smart_sect_header_s),
                             (FAR uint8_t *) &header);
              if (ret != sizeof(struct smart_sect_header_s))
                {
                  goto err_out;
                }

              /* Get the logical sector number for this physical sector */

              duplogsector = *((FAR uint16_t *) header.logicalsector);
#if CONFIG_SMARTFS_ERASEDSTATE == 0x00
              if (duplogsector == 0)
                {
                  duplogsector = -1;
                }
#endif

              /* Test if this sector has been committed */

              if ((header.status & SMART_STATUS_COMMITTED) ==
                      (CONFIG_SMARTFS_ERASEDSTATE & SMART_STATUS_COMMITTED))
                {
                  continue;
                }

              /* Test if this sector has been release and skip it if it has */

              if ((header.status & SMART_STATUS_RELEASED) !=
                      (CONFIG_SMARTFS_ERASEDSTATE & SMART_STATUS_RELEASED))
                {
                  continue;
                }

              if ((header.status & SMART_STATUS_VERBITS) != SMART_STATUS_VERSION)
                {
                  continue;
                }

              /* Now compare if this logical sector matches the current sector */

              if (duplogsector == logicalsector)
                {
                  break;
                }
            }
#endif

          ret = MTD_READ(dev->mtd, readaddress, sizeof(struct smart_sect_header_s),
                  (FAR uint8_t *) &header);
          if (ret != sizeof(struct smart_sect_header_s))
            {
              goto err_out;
            }

#if SMART_STATUS_VERSION == 1
          if (header.status & SMART_STATUS_CRC)
            {
              seq1 = header.seq;
            }
          else
            {
              seq1 = *((FAR uint16_t *) &header.seq);
            }
#else
          seq1 = header.seq;
#endif

          /* Now determine who wins */

          if ((seq1 > 0xFFF0 && seq2 < 10) || seq2 > seq1)
            {
              /* Seq 2 is the winner ... bigger or it wrapped */

#ifndef CONFIG_MTD_SMART_MINIMIZE_RAM
              loser = dev->sMap[logicalsector];
              dev->sMap[logicalsector] = sector;
#else
              loser = dupsector;
#endif
            }
          else
            {
              /* We keep the original mapping and seq2 is the loser */

              loser = sector;
            }

          /* Now release the loser sector */

          readaddress = loser  * dev->mtdBlksPerSector * dev->geo.blocksize;
          ret = MTD_READ(dev->mtd, readaddress, sizeof(struct smart_sect_header_s),
                  (FAR uint8_t *) &header);
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

#ifndef CONFIG_MTD_SMART_MINIMIZE_RAM
      /* Update the logical to physical sector map */

      dev->sMap[logicalsector] = sector;
#else
      /* Mark the logical sector as used in the bitmap */
      dev->sBitMap[logicalsector >> 3] |= 1 << (logicalsector & 0x07);

      if (logicalsector < SMART_FIRST_ALLOC_SECTOR)
        {
          smart_add_sector_to_cache(dev, logicalsector, sector, __LINE__ );
        }
#endif
    }

#if defined (CONFIG_MTD_SMART_WEAR_LEVEL) && (SMART_STATUS_VERSION == 1)
#ifdef CONFIG_MTD_SMART_CONVERT_WEAR_FORMAT

  /* We need to check if we are converting an older format with incorrect
   * wear leveling data in sector zero to the new format.  The old format
   * put all zeros in the wear level bit locations, but the new (better)
   * way is to leave them 0xFF.
   */

#ifndef CONFIG_MTD_SMART_MINIMIZE_RAM
  sector = dev->sMap[0];
#else
  sector = smart_cache_lookup(dev, 0);
#endif

  /* Validate the sector is valid ... may be an unformatted device */

  if (sector != 0xFFFF)
    {
      /* Read the sector data */

      ret = MTD_BREAD(dev->mtd, sector * dev->mtdBlksPerSector,
              dev->mtdBlksPerSector, (uint8_t *) dev->rwbuffer);
      if (ret != dev->mtdBlksPerSector)
        {
          fdbg("Error reading physical sector %d.\n", sector);
          goto err_out;
        }


      /* Check for old format wear leveling */
      if (dev->rwbuffer[SMART_WEAR_LEVEL_FORMAT_SIG] == 0)
        {
          /* Old format detected.  We must relocate sector zero and fill it in with 0xFF */

          uint16_t newsector = smart_findfreephyssector(dev, FALSE);
          if (newsector == 0xFFFF)
            {
              /* Unable to find a free sector!!! */

              fdbg("Can't find a free sector for relocation\n");
              ret = -ENOSPC;
              goto err_out;
            }

          memset(&dev->rwbuffer[SMART_WEAR_LEVEL_FORMAT_SIG], 0xFF,
              dev->mtdBlksPerSector * dev->geo.blocksize -
              SMART_WEAR_LEVEL_FORMAT_SIG);

          smart_relocate_sector(dev, sector, newsector);

          /* Update the free and release sector counts */

          dev->freesectors--;
          dev->releasesectors++;

#ifndef CONFIG_MTD_SMART_MINIMIZE_RAM
          dev->sMap[0] = newsector;
          dev->freecount[newsector / dev->sectorsPerBlk]--;
          dev->releasecount[sector / dev->sectorsPerBlk]++;
#else
          smart_update_cache(dev, 0, newsector);
          smart_add_count(dev, dev->freecount, newsector / dev->sectorsPerBlk, -1);
          smart_add_count(dev, dev->releasecount, sector / dev->sectorsPerBlk, 1);
#endif

        }
    }

#endif  /* CONFIG_MTD_SMART_CONVERT_WEAR_FORMAT */
#endif  /* CONFIG_MTD_SMART_WEAR_LEVEL && SMART_STATUS_VERSION == 1 */

#ifdef CONFIG_MTD_SMART_WEAR_LEVEL
  /* Read the wear leveling status bits */

  smart_read_wearstatus(dev);
#endif

  fdbg("SMART Scan\n");
  fdbg("   Erase size:   %10d\n", dev->sectorsPerBlk * dev->sectorsize);
  fdbg("   Erase count:  %10d\n", dev->neraseblocks);
  fdbg("   Sect/block:   %10d\n", dev->sectorsPerBlk);
  fdbg("   MTD Blk/Sect: %10d\n", dev->mtdBlksPerSector);
#ifdef CONFIG_MTD_SMART_ALLOC_DEBUG
  fdbg("   Allocations:\n");
  for (sector = 0; sector < SMART_MAX_ALLOCS; sector++)
    {
      if (dev->alloc[sector].ptr != NULL)
        {
          fdbg("       %s: %d\n", dev->alloc[sector].name, dev->alloc[sector].size);
        }
    }
#endif

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
static inline int smart_getformat(FAR struct smart_struct_s *dev,
                                  FAR struct smart_format_s *fmt,
                                  uint8_t rootdirnum)
#else
static inline int smart_getformat(FAR struct smart_struct_s *dev,
                                  FAR struct smart_format_s *fmt)
#endif
{
  int ret;

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
  fmt->nsectors = dev->totalsectors;

  fmt->nfreesectors = dev->freesectors;
  fmt->namesize = dev->namesize;
#ifdef CONFIG_SMARTFS_MULTI_ROOT_DIRS
  fmt->nrootdirentries = dev->rootdirentries;
  fmt->rootdirnum = rootdirnum;
#endif

  /* Add the released sectors to the reported free sector count */

  fmt->nfreesectors += dev->releasesectors;

  /* Subtract the reserved sector count */

  fmt->nfreesectors -= dev->sectorsPerBlk + 4;

  ret = OK;

err_out:
  return ret;
}

/****************************************************************************
 * Name: smart_erase_block_if_empty
 *
 * Description:  Tests the specified erase block if it contains all free or
 *               released sectors and erases it.
 *
 ****************************************************************************/

static void smart_erase_block_if_empty(FAR struct smart_struct_s *dev,
        uint16_t block, uint8_t forceerase)
{
  uint16_t  freecount, releasecount, prerelease;

#ifdef CONFIG_MTD_SMART_PACK_COUNTS
  releasecount = smart_get_count(dev, dev->releasecount, block);
  freecount = smart_get_count(dev, dev->freecount, block);
#else
  releasecount = dev->releasecount[block];
  freecount = dev->freecount[block];
#endif

  if ((freecount + releasecount == dev->availSectPerBlk && freecount < 1) || forceerase)
    {
      /* Erase the block */

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_SMARTFS)
      dev->unusedsectors += freecount;
      dev->blockerases++;
#endif
      MTD_ERASE(dev->mtd, block, 1);

#ifdef CONFIG_MTD_SMART_SECTOR_ERASE_DEBUG
      if (dev->erasecounts)
        {
          dev->erasecounts[block]++;
        }
#endif

      /* If wear leveling enabled, then we must add one to the wear status */

#ifdef CONFIG_MTD_SMART_WEAR_LEVEL
      smart_set_wear_level(dev, block, smart_get_wear_level(dev, block) + 1);
#endif

      /* If we have a device with 65534 sectors, then disallow the last two
       * physical sector if this is the last erase block on the device.
       */

      if (block == dev->geo.neraseblocks - 1 && dev->totalsectors == 65534)
        {
          prerelease = 2;
        }
      else
        {
          prerelease = 0;
        }

      dev->freesectors += dev->availSectPerBlk - prerelease - freecount;
      dev->releasesectors -= releasecount - prerelease;

#ifdef CONFIG_MTD_SMART_PACK_COUNTS
      smart_set_count(dev, dev->releasecount, block, prerelease);
      smart_set_count(dev, dev->freecount, block, dev->availSectPerBlk-prerelease);
#else
      dev->releasecount[block] = prerelease;
      dev->freecount[block] = dev->availSectPerBlk - prerelease;
#endif  /* CONFIG_MTD_SMART_PACK_COUNTS */

      /* Now that we have erased this block and updated the release / free counts,
       * if we are in WEAR LEVELING enabled mode, we must check if this erase block's
       * wear level has reached the threshold to warrant moving a minimum wear level
       * block's data into it (i.e. relocating static data to this block so it will
       * be worn less).
       */

#ifdef CONFIG_MTD_SMART_WEAR_LEVEL
      if (!forceerase)
        {
          smart_relocate_static_data(dev, block);
        }
#endif

#ifdef CONFIG_SMART_LOCAL_CHECKFREE
      if (smart_checkfree(dev, __LINE__) != OK)
        {
          fdbg("   ...while eraseing block %d\n", block);
        }
#endif
    }
}

/****************************************************************************
 * Name: smart_relocate_static_data
 *
 * Description:  Tests if the specified block has reached the wear threshold
 *               for static data relocation and if it has, relocates a less
 *               worn block to it.
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_SMART_WEAR_LEVEL
static int smart_relocate_static_data(FAR struct smart_struct_s *dev, uint16_t block)
{
  uint16_t    freecount, x, sector, minblock;
  uint16_t    nextsector, newsector, mincount;
  int         ret;
  FAR struct  smart_sect_header_s *header;
#ifdef CONFIG_MTD_SMART_ENABLE_CRC
  FAR struct smart_allocsector_s *allocsector;
#endif

  /* Now that we have erased this block and updated the release / free counts,
   * if we are in WEAR LEVELING enabled mode, we must check if this erase block's
   * wear level has reached the threshold to warrant moving a minimum wear level
   * block's data into it (i.e. relocating static data to this block so it will
   * be worn less).
   */

  ret = OK;
  header = (FAR struct smart_sect_header_s *) dev->rwbuffer;

#ifdef CONFIG_SMART_LOCAL_CHECKFREE
  if (smart_checkfree(dev, __LINE__) != OK)
    {
      fdbg("   ...about to relocate static data %d\n", block);
    }
#endif

  if (smart_get_wear_level(dev, block) >= SMART_WEAR_FULL_RELOCATE_THRESHOLD)
    {
      /* Okay, this block is getting too worn.  Move a minimum wear level
       * block to it in it's entirity.
       */

      /* Scan all erase blocks (or until we find a minimum wear level block
       * with no free + released blocks.
       */

      freecount = dev->sectorsPerBlk + 1;
      minblock = dev->geo.neraseblocks;
      mincount = 0;
      for (x = 0; x < dev->geo.neraseblocks; x++)
        {
          if (smart_get_wear_level(dev, x) == dev->minwearlevel)
            {
              /* Don't allow the format sector or directory sector to
               * be moved into a worn block.  First get the format and
               * dir sectors.
               */

              mincount++;

#ifdef CONFIG_MTD_SMART_PACK_COUNTS
              if (smart_get_count(dev, dev->releasecount, x) +
                  smart_get_count(dev, dev->freecount, x) < freecount)
                {
                  freecount = smart_get_count(dev, dev->releasecount, x) +
                    smart_get_count(dev, dev->freecount, x);
                  minblock = x;
                }
#else
              if (dev->freecount[x] + dev->releasecount[x] < freecount)
                {
                  freecount = dev->freecount[x] + dev->releasecount[x];
                  minblock = x;
                }
#endif

              /* Break if freecount reaches zero */

              if (freecount == 0)
                {
                  /* We found a minimum wear-level block with no free sectors.
                   * relocate this block to the more highly worn block.
                   */

                  break;
                }
            }
        }

      /* Okay, now move block 'x' to block 'block' and erase block 'x' */

      x = minblock;

      /* We are resuing nextsector and newsector variables here simply as
       * variables for displaying debug data.  I have learned through my
       * years of programming that this is a really good way to create
       * spaghetti code, but I didn't want to add stack variables just
       * for debug data, and I *know* these variables aren't being used
       * yet.
       */

#ifdef CONFIG_MTD_SMART_PACK_COUNTS
      nextsector = smart_get_count(dev, dev->freecount, x);
      newsector = smart_get_count(dev, dev->releasecount, x);
#else
      nextsector = dev->freecount[x];
      newsector = dev->releasecount[x];
#endif
      fvdbg("Moving block %d, wear %d, free %d, released %d to block %d, wear %d\n",
              x, smart_get_wear_level(dev, x),
              nextsector, newsector,
              block, smart_get_wear_level(dev, block));

      nextsector = block * dev->sectorsPerBlk;
      for (sector = x * dev->sectorsPerBlk; sector <
         x * dev->sectorsPerBlk + dev->availSectPerBlk; sector++)
        {
          /* Read the next sector from this erase block */

          ret = MTD_BREAD(dev->mtd, sector * dev->mtdBlksPerSector,
              dev->mtdBlksPerSector, (FAR uint8_t *) dev->rwbuffer);
          if (ret != dev->mtdBlksPerSector)
            {
              fdbg("Error reading sector %d\n", sector);
              ret = -EIO;
              goto errout;
            }

          /* Test if the block is in use */

#ifdef CONFIG_MTD_SMART_ENABLE_CRC

          /* Check if there is a temporary alloc for this physical sector */

          allocsector = dev->allocsector;
          while (allocsector)
            {
              if (allocsector->physical == sector)
                {
                  break;
                }

              allocsector = allocsector->next;
            }

          /* If we found a temp allocation, just update the mapped physical
           * location and move on to the next block ... there is no data to
           * move yet.
           */

          if (allocsector)
            {
              /* Get next sector from 'block' */

              newsector = nextsector++;
              if (newsector == 0xFFFF)
                {
                  /* Unable to find a free sector!!! */

                  fdbg("Can't find a free sector for relocation\n");
                  ret = -ENOSPC;
                  goto errout;
                }

              /* Update the temporary allocation's physical sector */

              allocsector->physical = newsector;
              *((FAR uint16_t *) header->logicalsector) = allocsector->logical;
            }
          else
#endif
            {
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

              newsector = nextsector++;

              /* Relocate the sector data */

              if ((ret = smart_relocate_sector(dev, sector, newsector)) < 0)
                {
                  goto errout;
                }
            }

          dev->freesectors--;

#ifndef CONFIG_MTD_SMART_MINIMIZE_RAM
          dev->sMap[*((FAR uint16_t *) header->logicalsector)] = newsector;
#else
          smart_update_cache(dev, *((FAR uint16_t *) header->logicalsector), newsector);
#endif

#ifdef CONFIG_MTD_SMART_PACK_COUNTS
          smart_add_count(dev, dev->freecount, block, -1);
#else
          dev->freecount[block]--;
#endif  /* CONFIG_MTD_SMART_PACK_COUNTS */
        }

#ifdef CONFIG_SMART_LOCAL_CHECKFREE
      if (smart_checkfree(dev, __LINE__) != OK)
        {
          fdbg("   ...about to erase static block %d\n", block);
        }
#endif

      /* Now erase the block we just relocated, force erasing it */

      smart_erase_block_if_empty(dev, x, TRUE);
    }

#ifdef CONFIG_SMART_LOCAL_CHECKFREE
  if (smart_checkfree(dev, __LINE__) != OK)
    {
      fdbg("   ...done erasing static block %d\n", block);
    }
#endif

errout:
  return ret;
}
#endif

/****************************************************************************
 * Name: smart_calc_sector_crc
 *
 * Description:  Calculate the CRC value for the sector data in the RW buffer
 *               based on the configured CRC size.
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_SMART_ENABLE_CRC
static crc_t smart_calc_sector_crc(FAR struct smart_struct_s *dev)
{
  crc_t     crc = 0;

#ifdef CONFIG_SMART_CRC_8

  /* Calculate CRC on data region of the sector */

  crc = crc8((uint8_t *) &dev->rwbuffer[sizeof(struct smart_sect_header_s)],
      dev->mtdBlksPerSector * dev->geo.blocksize - sizeof(struct smart_sect_header_s));

  /* Add logical sector number and seq to the CRC calculation */

  crc = crc8part((uint8_t *) dev->rwbuffer, 3, crc);

  /* Add status to the CRC calculation */

  crc = crc8part((uint8_t *) &dev->rwbuffer[offsetof(struct smart_sect_header_s,
        status)], 1, crc);

#elif defined(CONFIG_SMART_CRC_16)
  /* Calculate CRC on data region of the sector */

  crc = crc16((uint8_t *) &dev->rwbuffer[sizeof(struct smart_sect_header_s)],
      dev->mtdBlksPerSector * dev->geo.blocksize - sizeof(struct smart_sect_header_s));

  /* Add logical sector number to the CRC calculation */

  crc = crc16part((uint8_t *) dev->rwbuffer, 2, crc);

  /* Add status and seq to the CRC calculation */

  crc = crc16part((uint8_t *) &dev->rwbuffer[offsetof(struct smart_sect_header_s,
        status)], 2, crc);

#elif defined(CONFIG_SMART_CRC_32)
  /* Calculate CRC on data region of the sector */

  crc = crc32((uint8_t *) &dev->rwbuffer[sizeof(struct smart_sect_header_s)],
      dev->mtdBlksPerSector * dev->geo.blocksize - sizeof(struct smart_sect_header_s));

  /* Add logical sector number, status and seq to the CRC calculation */

  crc = crc32part((uint8_t *) dev->rwbuffer, 6, crc);
#else
#error "Unknown CRC size!"
#endif

  return crc;
}
#endif  /* CONFIG_MTD_SMART_ENABLE_CRC  */

/****************************************************************************
 * Name: smart_llformat
 *
 * Description:  Performs a low-level format of the flash device.  This
 *               involves erasing the device and writing a valid sector
 *               zero (logical) with proper format signature.
 *
 ****************************************************************************/

#ifdef CONFIG_FS_WRITABLE
static inline int smart_llformat(FAR struct smart_struct_s *dev, unsigned long arg)
{
  FAR struct  smart_sect_header_s  *sectorheader;
  size_t      wrcount;
  int         x;
  int         ret;
  uint8_t     sectsize, prerelease;

  fvdbg("Entry\n");

  smart_setsectorsize(dev, CONFIG_MTD_SMART_SECTOR_SIZE);

  /* Check for invalid format */
  if (dev->erasesize == 0)
    {
      if (dev->geo.erasesize == 0)
        {
          dev->erasesize = 262144;
        }
      else
        {
          dev->erasesize = dev->geo.erasesize;
        }

      dbg("ERROR:  Invalid geometery ... Sectors per erase block must be 256 or less\n");
      dbg("        Erase block size    = %d\n", dev->erasesize);
      dbg("        Sector size         = %d\n", dev->sectorsize);
      dbg("        Sectors/erase block = %d\n", dev->erasesize / dev->sectorsize);

      return -EINVAL;
    }

  /* Erase the MTD device */

  ret = MTD_IOCTL(dev->mtd, MTDIOC_BULKERASE, 0);
  if (ret < 0)
    {
      return ret;
    }

  /* Now construct a logical sector zero header to write to the device. */

  sectorheader = (FAR struct smart_sect_header_s *) dev->rwbuffer;
  memset(dev->rwbuffer, CONFIG_SMARTFS_ERASEDSTATE, dev->sectorsize);
#if SMART_STATUS_VERSION == 1
#ifdef CONFIG_MTD_SMART_ENABLE_CRC
  /* CRC enabled.  Using an 8-bit sequence number */

  sectorheader->seq = 0;
#else
  /* CRC not enabled.  Using a 16-bit sequence number */

  *((FAR uint16_t *) &sectorheader->seq) = 0;
#endif
#else   /* SMART_STATUS_VERSION == 1 */
  sectorheader->seq = 0;
#endif  /* SMART_STATUS_VERSION == 1 */

  /* Set the sector size of this sector */

  sectsize = (CONFIG_MTD_SMART_SECTOR_SIZE >> 9) << 2;

  /* Set the sector logical sector to zero and setup the header status */

#if ( CONFIG_SMARTFS_ERASEDSTATE == 0xFF )
  *((FAR uint16_t *) sectorheader->logicalsector) = 0;
  sectorheader->status = (uint8_t) ~(SMART_STATUS_COMMITTED | SMART_STATUS_VERBITS |
          SMART_STATUS_SIZEBITS) | SMART_STATUS_VERSION |
          sectsize;
#ifdef CONFIG_MTD_SMART_ENABLE_CRC
  sectorheader->status &= ~SMART_STATUS_CRC;
#endif  /* CONFIG_MTD_SMART_ENABLE_CRC */

#else   /* CONFIG_SMARTFS_ERASEDSTATE == 0xFF */
  *((FAR uint16_t *) sectorheader->logicalsector) = 0xFFFF;
  sectorheader->status = (uint8_t) (SMART_STATUS_COMMITTED | SMART_STATUS_VERSION |
          sectsize);
#ifdef CONFIG_MTD_SMART_ENABLE_CRC
  sectorheader->status |= SMART_STATUS_CRC;
#endif  /* CONFIG_MTD_SMART_ENABLE_CRC */
#endif  /* CONFIG_SMARTFS_ERASEDSTATE == 0xFF */

  /* Now add the format signature to the sector */

  dev->rwbuffer[SMART_FMT_POS1] = SMART_FMT_SIG1;
  dev->rwbuffer[SMART_FMT_POS2] = SMART_FMT_SIG2;
  dev->rwbuffer[SMART_FMT_POS3] = SMART_FMT_SIG3;
  dev->rwbuffer[SMART_FMT_POS4] = SMART_FMT_SIG4;

  dev->rwbuffer[SMART_FMT_VERSION_POS] = SMART_FMT_VERSION;
  dev->rwbuffer[SMART_FMT_NAMESIZE_POS] = CONFIG_SMARTFS_MAXNAMLEN;

  /* Record the number of root directory entries we have */

  dev->rwbuffer[SMART_FMT_ROOTDIRS_POS] = (uint8_t) arg;

#ifdef CONFIG_SMART_CRC_8
  sectorheader->crc8 = smart_calc_sector_crc(dev);
#elif defined(CONFIG_SMART_CRC_16)
  *((uint16_t *) sectorheader->crc16) = smart_calc_sector_crc(dev);
#elif defined(CONFIG_SMART_CRC_32)
  *((uint32_t *) sectorheader->crc32) = smart_calc_sector_crc(dev);
#endif

  /* Write the sector to the flash */

  wrcount = MTD_BWRITE(dev->mtd, 0, dev->mtdBlksPerSector,
          (FAR uint8_t *) dev->rwbuffer);
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
  dev->freesectors = dev->availSectPerBlk * dev->geo.neraseblocks - 1;
  dev->releasesectors = 0;
#ifdef CONFIG_MTD_SMART_WEAR_LEVEL
  dev->uneven_wearcount = 0;
#endif

  /* Initialize the released and free counts */

  for (x = 0; x < dev->neraseblocks; x++)
    {
      /* Test for a geometry with 65536 sectors.  We allow this, though
         we never use the last two sectors in this mode.
       */

      if (x == dev->neraseblocks && dev->totalsectors == 65534)
        {
          prerelease = 2;
        }
      else
        {
          prerelease = 0;
        }
#ifdef CONFIG_MTD_SMART_PACK_COUNTS
      smart_set_count(dev, dev->releasecount, x, prerelease);
      smart_set_count(dev, dev->freecount, x, dev->availSectPerBlk-prerelease);
#else
      dev->releasecount[x] = prerelease;
      dev->freecount[x] = dev->availSectPerBlk-prerelease;
#endif
    }

  /* Account for the format sector */

#ifdef CONFIG_MTD_SMART_PACK_COUNTS
  smart_set_count(dev, dev->freecount, 0, dev->availSectPerBlk - 1);
#else
  dev->freecount[0]--;
#endif

  /* Now initialize the logical to physical sector map */

#ifndef CONFIG_MTD_SMART_MINIMIZE_RAM
  dev->sMap[0] = 0;     /* Logical sector zero = physical sector 0 */
  for (x = 1; x < dev->totalsectors; x++)
    {
      /* Mark all other logical sectors as non-existant */

      dev->sMap[x] = -1;
    }
#endif

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
 * Name: smart_relocate_sector
 *
 * Description:  Relocates the specified sector to the new sector location.
 *
 ****************************************************************************/

static int smart_relocate_sector(FAR struct smart_struct_s *dev,
    uint16_t oldsector, uint16_t newsector)
{
  int         ret;
  size_t      offset;
  FAR struct  smart_sect_header_s *header;
  uint8_t     newstatus;

  header = (FAR struct smart_sect_header_s *) dev->rwbuffer;

  /* Increment the sequence number and clear the "commit" flag */

#if SMART_STATUS_VERSION == 1
  if (header->status & SMART_STATUS_CRC)
    {
#endif
      /* Using 8-bit sequence */

      header->seq++;
      if (header->seq == 0xFF)
        {
          header->seq = 1;
        }
#if SMART_STATUS_VERSION == 1
    }
  else
    {
      /* Using 16-bit sequence and no CRC */

      (*((FAR uint16_t *) &header->seq))++;
      if (*((FAR uint16_t *) &header->seq) == 0xFFFF)
        {
          *((FAR uint16_t *) &header->seq) = 1;
        }
    }
#endif

  /* When CRC is enabled, we must pre-commit the sector and also
   * calculate an updated CRC for the sector prior to writing
   * since we changed the sequence number.
   */

#ifdef CONFIG_MTD_SMART_ENABLE_CRC

  /* First pre-commit the sector */

#if CONFIG_SMARTFS_ERASEDSTATE == 0xFF
  header->status &= ~(SMART_STATUS_COMMITTED | SMART_STATUS_CRC);
#else
  header->status |= SMART_STATUS_COMMITTED | SMART_STATUS_CRC;
#endif

  /* Now calculate the new CRC */

#ifdef CONFIG_SMART_CRC_8
  header->crc8 = smart_calc_sector_crc(dev);
#elif defined(CONFIG_SMART_CRC_16)
  *((uint16_t *) header->crc16) = smart_calc_sector_crc(dev);
#elif defined(CONFIG_SMART_CRC_32)
  *((uint32_t *) header->crc32) = smart_calc_sector_crc(dev);
#endif

  /* Write the data to the new physical sector location */

  ret = MTD_BWRITE(dev->mtd, newsector * dev->mtdBlksPerSector,
                   dev->mtdBlksPerSector, (FAR uint8_t *) dev->rwbuffer);

#else   /* CONFIG_MTD_SMART_ENABLE_CRC */

#if CONFIG_SMARTFS_ERASEDSTATE == 0xFF
  header->status |= SMART_STATUS_COMMITTED;
#else
  header->status &= ~SMART_STATUS_COMMITTED;
#endif

  /* Write the data to the new physical sector location */

  ret = MTD_BWRITE(dev->mtd, newsector * dev->mtdBlksPerSector,
                   dev->mtdBlksPerSector, (FAR uint8_t *) dev->rwbuffer);

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
#endif  /* CONFIG_MTD_SMART_ENABLE_CRC */

  /* Release the old physical sector */

#if CONFIG_SMARTFS_ERASEDSTATE == 0xFF
  newstatus = header->status & ~(SMART_STATUS_RELEASED | SMART_STATUS_COMMITTED);
#else
  newstatus = header->status | SMART_STATUS_RELEASED | SMART_STATUS_COMMITTED;
#endif
  offset = oldsector * dev->mtdBlksPerSector * dev->geo.blocksize +
      offsetof(struct smart_sect_header_s, status);
  ret = smart_bytewrite(dev, offset, 1, &newstatus);
  if (ret < 0)
    {
      fdbg("Error %d releasing old sector %d\n" -ret, oldsector);
    }

#ifndef CONFIG_MTD_SMART_ENABLE_CRC
errout:
#endif

  return ret;
}

/****************************************************************************
 * Name: smart_relocate_block
 *
 * Description:  Relocates the specified MTD erase block by moving any
 *               active sectors to a different erase block and then erases
 *               the selected block.
 *
 ****************************************************************************/

static int smart_relocate_block(FAR struct smart_struct_s *dev, uint16_t block)
{
  uint16_t    newsector, oldrelease;
  int         x;
  int         ret;
  FAR struct  smart_sect_header_s *header;
  uint8_t     prerelease;
  uint16_t    freecount;
#if defined(CONFIG_SMART_LOCAL_CHECKFREE) && defined(CONFIG_DEBUG_FS)
  uint16_t    releasecount;
#endif
#ifdef CONFIG_MTD_SMART_ENABLE_CRC
  FAR struct smart_allocsector_s *allocsector;
#endif

  /* Perform collection on block with the most released sectors.
   * First mark the block as having no free sectors so we don't
   * try to move sectors into the block we are trying to erase.
   */

  header = (FAR struct smart_sect_header_s *) dev->rwbuffer;

#ifdef CONFIG_SMART_LOCAL_CHECKFREE
  if (smart_checkfree(dev, __LINE__) != OK)
    {
      fdbg("   ...while relocating block %d, free=%d\n", block, dev->freesectors);
    }
#endif

#ifdef CONFIG_MTD_SMART_PACK_COUNTS
  freecount = smart_get_count(dev, dev->freecount, block);

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_SMARTFS)
#if defined(CONFIG_SMART_LOCAL_CHECKFREE) && defined(CONFIG_DEBUG_FS)
  releasecount = smart_get_count(dev, dev->releasecount, block);
#endif
#endif

  /* Ensure we aren't relocating a block containing the only free sectors */

  if (freecount >= dev->freesectors)
    {
      fdbg("Program bug!  Relocating the only block (%d) with free sectors!\n", block);
      ret = -EIO;
      goto errout;
    }

  smart_set_count(dev, dev->freecount, block, 0);

#else /* CONFIG_MTD_SMART_PACK_COUNTS */

  freecount = dev->freecount[block];
#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_SMARTFS)
#if defined(CONFIG_SMART_LOCAL_CHECKFREE) && defined(CONFIG_DEBUG_FS)
  releasecount = dev->releasecount[block];
#endif
#endif
  dev->freecount[block] = 0;
#endif

  /* Next move all live data in the block to a new home. */

  for (x = block * dev->sectorsPerBlk; x <
     block * dev->sectorsPerBlk + dev->availSectPerBlk; x++)
    {
      /* Read the next sector from this erase block */

      ret = MTD_BREAD(dev->mtd, x * dev->mtdBlksPerSector,
          dev->mtdBlksPerSector, (FAR uint8_t *) dev->rwbuffer);
      if (ret != dev->mtdBlksPerSector)
        {
          fdbg("Error reading sector %d\n", x);
          ret = -EIO;
          goto errout;
        }

      /* Test if the block is in use */

#ifdef CONFIG_MTD_SMART_ENABLE_CRC

      /* Check if there is a temporary alloc for this physical sector */

      allocsector = dev->allocsector;
      while (allocsector)
        {
          if (allocsector->physical == x)
            break;
          allocsector = allocsector->next;
        }

      /* If we found a temp allocation, just update the mapped physical
       * location and move on to the next block ... there is no data to
       * move yet.
       */

      if (allocsector)
        {
          newsector = smart_findfreephyssector(dev, FALSE);
          if (newsector == 0xFFFF)
            {
              /* Unable to find a free sector!!! */

              fdbg("Can't find a free sector for relocation\n");
              ret = -ENOSPC;
              goto errout;
            }

          /* Update the temporary allocation's physical sector */

          allocsector->physical = newsector;
          *((FAR uint16_t *) header->logicalsector) = allocsector->logical;
        }
      else
#endif
        {
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

          newsector = smart_findfreephyssector(dev, FALSE);
          if (newsector == 0xFFFF)
            {
              /* Unable to find a free sector!!! */

              fdbg("Can't find a free sector for relocation\n");
              ret = -ENOSPC;
              goto errout;
            }

          /* Relocate the sector data */

          if ((ret = smart_relocate_sector(dev, x, newsector)) < 0)
            goto errout;
        }

      /* Update the variables */

#ifndef CONFIG_MTD_SMART_MINIMIZE_RAM
      dev->sMap[*((FAR uint16_t *) header->logicalsector)] = newsector;
#else
      smart_update_cache(dev, *((FAR uint16_t *) header->logicalsector), newsector);
#endif

#ifdef CONFIG_MTD_SMART_PACK_COUNTS
      smart_add_count(dev, dev->freecount, newsector / dev->sectorsPerBlk, -1);
#else
      dev->freecount[newsector / dev->sectorsPerBlk]--;
#endif
    }

  /* Now erase the erase block */

  MTD_ERASE(dev->mtd, block, 1);
#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_SMARTFS)
  dev->unusedsectors += freecount;
  dev->blockerases++;
#endif

#ifdef CONFIG_MTD_SMART_SECTOR_ERASE_DEBUG
  if (dev->erasecounts)
    {
      dev->erasecounts[block]++;
    }
#endif

#ifdef CONFIG_MTD_SMART_WEAR_LEVEL

  /* Update the new wear level count */

  smart_set_wear_level(dev, block, smart_get_wear_level(dev, block) + 1);
#endif

  /* Update the free and release sectors for this erase block. */

  if (x == dev->neraseblocks && dev->totalsectors == 65534)
    {
      /* We can't use the last two sectors on a 65536 sector device,
         so "pre-release" them so they never get allocated.
       */

      prerelease = 2;
    }
  else
    {
      prerelease = 0;
    }

#ifdef CONFIG_MTD_SMART_PACK_COUNTS
  oldrelease = smart_get_count(dev, dev->releasecount, block);
  dev->freesectors += oldrelease-prerelease;
  dev->releasesectors -= oldrelease-prerelease;
  smart_set_count(dev, dev->freecount, block, dev->availSectPerBlk-prerelease);
  smart_set_count(dev, dev->releasecount, block, prerelease);
#else
  oldrelease = dev->releasecount[block];
  dev->freesectors += oldrelease-prerelease;
  dev->releasesectors -= oldrelease-prerelease;
  dev->freecount[block] = dev->availSectPerBlk-prerelease;
  dev->releasecount[block] = prerelease;
#endif

#ifdef CONFIG_SMART_LOCAL_CHECKFREE
  if (smart_checkfree(dev, __LINE__) != OK)
    {
      fdbg("   ...while relocating block %d, free=%d, release=%d, oldrelease=%d\n", block, freecount, releasecount, oldrelease);
    }
#endif

  /* Test if this erase causes the block to reach the full relocate
   * threshold requiring static data relocation.
   */

#ifdef CONFIG_MTD_SMART_WEAR_LEVEL
  smart_relocate_static_data(dev, block);
#endif

  return OK;

errout:
  /* Restore the block's freecount if error */

#ifdef CONFIG_MTD_SMART_PACK_COUNTS
  smart_set_count(dev, dev->freecount, block, freecount);
#else
  dev->freecount[block] = freecount;
#endif
  return ret;
}

/****************************************************************************
 * Name: smart_findfreephyssector
 *
 * Description:  Finds a free physical sector based on free and released
 *               count logic, taking into account reserved sectors.
 *
 ****************************************************************************/

static int smart_findfreephyssector(FAR struct smart_struct_s *dev,
    uint8_t canrelocate)
{
  uint16_t  count, allocfreecount, allocblock;
#ifdef CONFIG_MTD_SMART_WEAR_LEVEL
  uint16_t  wornfreecount, wornblock;
  uint8_t   wearlevel, wornlevel;
  uint8_t   maxwearlevel;
#endif
  uint16_t  physicalsector;
  uint16_t  x, block;
  uint32_t  readaddr;
  struct    smart_sect_header_s header;
  int       ret;

  /* Determine which erase block we should allocate the new
   * sector from. This is based on the number of free sectors
   * available in each erase block. */

#ifdef CONFIG_MTD_SMART_WEAR_LEVEL
retry:
#endif
  allocfreecount = 0;
  allocblock = 0xFFFF;
#ifdef CONFIG_MTD_SMART_WEAR_LEVEL
  wornfreecount = 0;
  wornblock = 0xFFFF;
  wornlevel = 15;
  maxwearlevel = 0;
#endif
  physicalsector = 0xFFFF;
  if (++dev->lastallocblock >= dev->neraseblocks)
    {
      dev->lastallocblock = 0;
    }

  block = dev->lastallocblock;
  for (x = 0; x < dev->neraseblocks; x++)
    {
      /* Test if this block has more free blocks than the
       * currently selected block
       */

#ifdef CONFIG_MTD_SMART_PACK_COUNTS
      count = smart_get_count(dev, dev->freecount, block);
#else
      count = dev->freecount[block];
#endif

#ifdef CONFIG_MTD_SMART_WEAR_LEVEL
      /* Keep track of the block with the max free sectors that is worn */

      wearlevel = smart_get_wear_level(dev, block);
      if (wearlevel >= SMART_WEAR_FULL_RELOCATE_THRESHOLD)
        {
          if (wearlevel > maxwearlevel && count > 0)
            {
              maxwearlevel = wearlevel;
            }

          if (count > wornfreecount || (count > 0 && wearlevel < wornlevel))
            {
              /* Keep track of this block.  If there are only worn blocks with
               * free sectors left, then we will use it.
               */

              if (x < dev->neraseblocks - 1 || !wornfreecount)
                {
                  wornfreecount = count;
                  wornblock = block;
                  wornlevel = wearlevel;
                }
            }
        }
      else
#endif

      if (count > allocfreecount)
        {
          /* Assign this block to alloc from */

          if (x < dev->neraseblocks - 1 || !allocfreecount)
            {
              allocblock = block;
              allocfreecount = count;
            }
        }
      if (++block >= dev->neraseblocks)
        block = 0;
    }

  /* Check if we found an allocblock. */

  if (allocblock == 0xFFFF)
    {
      /* No un-worn blocks with free sectors */

#ifdef CONFIG_MTD_SMART_WEAR_LEVEL

      /* If we are allowed to relocate unworn blocks then do so now */

      if (canrelocate && wornfreecount < (dev->sectorsPerBlk >> 2) && wornlevel == maxwearlevel)
        {
          /* Relocate up to 8 unworn blocks */

          block = 0;
          for (x = 0; x < 8; )
            {
              if (smart_get_wear_level(dev, block) < SMART_WEAR_FORCE_REORG_THRESHOLD)
                {
                  if (smart_relocate_block(dev, block) < 0)
                    {
                      fdbg("Error relocating block while finding free phys sector\n");
                      return -1;
                    }

                  x++;
                }

              block++;
            }
          if (x > 0)
            {
              /* Disable relocate for retry */

              canrelocate = FALSE;
              goto retry;
            }
        }
      else
        {
          dev->wearflags |= SMART_WEARFLAGS_FORCE_REORG;
        }

      /* Test if we found a worn block with free sectors */

      if (wornblock != 0xFFFF)
        {
          allocblock = wornblock;
        }
      else
#endif

      {
        dbg("Program bug!  Expected a free sector, free=%d\n", dev->freesectors);
        for (x = 0; x < dev->neraseblocks; x++)
          {
            printf("%d ", dev->freecount[x]);
          }

        /* No free sectors found!  Bug? */

        return -1;
      }
    }

  /* Now find a free physical sector within this selected
   * erase block to allocate. */

  for (x = allocblock * dev->sectorsPerBlk;
       x < allocblock * dev->sectorsPerBlk + dev->availSectPerBlk; x++)
    {
      /* Check if this physical sector is available. */

#ifdef CONFIG_MTD_SMART_ENABLE_CRC
      /* First check if there is a temporary alloc in place */

      FAR struct smart_allocsector_s* allocsect;
      allocsect = dev->allocsector;

      while (allocsect)
        {
          if (allocsect->physical == x)
            break;
          allocsect = allocsect->next;
        }

      /* If we found this physical sector above, then continue on
       * to the next physical sector in this block ... this one has
       * a temporary allocation assigned.
       */

      if (allocsect)
        {
          continue;
        }
#endif

      /* Now check on the physical media */

      readaddr = x * dev->mtdBlksPerSector * dev->geo.blocksize;
      ret = MTD_READ(dev->mtd, readaddr, sizeof(struct smart_sect_header_s),
              (FAR uint8_t *) &header);
      if (ret != sizeof(struct smart_sect_header_s))
        {
          fdbg("Error reading phys sector %d\n", physicalsector);
          return -1;
        }

      if ((*((FAR uint16_t *) header.logicalsector) == 0xFFFF) &&
#if SMART_STATUS_VERSION == 1
          (*((FAR uint16_t *) &header.seq) == 0xFFFF) &&
#else
          (header.seq == CONFIG_SMARTFS_ERASEDSTATE) &&
#endif
          ((header.status & SMART_STATUS_COMMITTED) ==
           (CONFIG_SMARTFS_ERASEDSTATE & SMART_STATUS_COMMITTED)))
        {
          physicalsector = x;
          dev->lastallocblock = allocblock;
          break;
        }
    }

  if (physicalsector == 0xFFFF)
    {
      dbg("Program bug!  Expected a free sector\n");
    }

  if (physicalsector >= dev->totalsectors)
    {
      dbg("Program bug!  Selected sector too big!!!\n");
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
static int smart_garbagecollect(FAR struct smart_struct_s *dev)
{
  uint16_t  collectblock;
  uint16_t  releasemax;
  bool      collect = TRUE;
  int       x;
  int       ret;
#ifdef CONFIG_MTD_SMART_PACK_COUNTS
  uint8_t   count;
#endif

  while (collect)
    {
      collect = FALSE;

      /* Test if the released sectors count is greater than the
       * free sectors.  If it is, then we will do garbage collection.
       */

      if (dev->releasesectors > dev->freesectors && dev->freesectors <
          (dev->totalsectors >> 5))
        {
          collect = TRUE;
        }

      /* Test if we have more reached our reserved free sector limit */

      if (dev->freesectors <= (dev->sectorsPerBlk << 0) + 4)
        {
          collect = TRUE;
        }

      /* Test if we need to garbage collect */

      if (collect)
        {
          /* Find the block with the most released sectors */

          collectblock = 0xFFFF;
          releasemax = 0;
          for (x = 0; x < dev->neraseblocks; x++)
            {
#ifdef CONFIG_MTD_SMART_WEAR_LEVEL
              /* Don't collect blocks that have been worn completely */

              if (smart_get_wear_level(dev, x) >= SMART_WEAR_REORG_THRESHOLD)
                {
                  continue;
                }
#endif

#ifdef CONFIG_MTD_SMART_PACK_COUNTS
              count = smart_get_count(dev, dev->releasecount, x);
              if (count > releasemax)
                {
                  releasemax = count;
                  collectblock = x;
                }
#else
              if (dev->releasecount[x] > releasemax)
                {
                  releasemax = dev->releasecount[x];
                  collectblock = x;
                }
#endif
          }
          //releasemax = smart_get_count(dev, dev->releasecount, collectblock);

          if (collectblock == 0xFFFF)
            {
              /* Need to collect, but no sectors with released blocks! */

              ret = -ENOSPC;
              goto errout;
            }

#ifdef CONFIG_SMART_LOCAL_CHECKFREE
          if (smart_checkfree(dev, __LINE__) != OK)
            {
              fdbg("   ...before collecting block %d\n", collectblock);
            }
#endif

#ifdef CONFIG_MTD_SMART_PACK_COUNTS
          fvdbg("Collecting block %d, free=%d released=%d, totalfree=%d, totalrelease=%d\n",
              collectblock, smart_get_count(dev, dev->freecount, collectblock),
              smart_get_count(dev, dev->releasecount, collectblock), dev->freesectors, dev->releasesectors);
#else
          fvdbg("Collecting block %d, free=%d released=%d\n",
              collectblock, dev->freecount[collectblock],
              dev->releasecount[collectblock]);
#endif

          /* Relocate the active data in the collection block */

          ret = smart_relocate_block(dev, collectblock);

#ifdef CONFIG_SMART_LOCAL_CHECKFREE
          if (smart_checkfree(dev, __LINE__) != OK)
            {
              fdbg("   ...while collecting block %d\n", collectblock);
            }
#endif

          if (ret != OK)
            {
              goto errout;
            }
        }
    }

  return OK;

errout:
  return ret;
}
#endif /* CONFIG_FS_WRITABLE */

/****************************************************************************
 * Name: smart_write_wearstatus
 *
 * Description:  Writes the wear leveling status bits to sector zero (and
 *               possibly others if it doesn't fit) such that is is persisted
 *               across OS reboots.
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_SMART_WEAR_LEVEL
static int smart_write_wearstatus(struct smart_struct_s *dev)
{
  uint16_t  sector;
  uint16_t  remaining, towrite;
  struct smart_read_write_s req;
  int       ret;
  uint8_t   buffer[8], write_buffer = 0;

  sector = 0;
  remaining = dev->geo.neraseblocks >> 1;
  memset(buffer, 0xFF, sizeof(buffer));

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_SMARTFS)
  if (dev->blockerases > 0)
    {
      *((uint32_t *) buffer) = dev->blockerases;
      write_buffer = 1;
    }
#endif

  /* Write the uneven wear count just prior to the wear bits */

  if (dev->uneven_wearcount != 0)
    {
      *((uint32_t *) &buffer[4]) = dev->uneven_wearcount;
      write_buffer = 1;
    }

  /* Test if we need to write either total block erase count or
     uneven wearcount (or both)
   */

  if (write_buffer)
    {
      req.logsector = sector;
      req.offset = SMARTFS_FMT_WEAR_POS - 8;
      req.count = sizeof(buffer);
      req.buffer = buffer;
      ret = smart_writesector(dev, (unsigned long) &req);
      if (ret != OK)
        {
          goto errout;
        }
    }

  /* Write all wear level bits to logical sector zero, one, two */

  while (remaining)
    {
      /* Calculate the number of bytes to write to this sector */

      towrite = remaining;
      if (towrite > dev->sectorsize - SMARTFS_FMT_WEAR_POS)
        {
          towrite = dev->sectorsize - SMARTFS_FMT_WEAR_POS;
        }

      /* Setup the sector write request (we are our own client) */

      req.logsector = sector;
      req.offset = SMARTFS_FMT_WEAR_POS;
      req.count = towrite;
      req.buffer = &dev->wearstatus[(dev->geo.neraseblocks >> SMART_WEAR_BIT_DIVIDE) -
                    remaining];

      /* Write the sector */

      ret = smart_writesector(dev, (unsigned long) &req);
      if (ret != OK)
        {
          goto errout;
        }

      /* Decrement the remaining count */

      remaining -= towrite;
      if (remaining)
        {
          /* Data doesn't fit in a single sector.  Use the reserved sectors */

          sector++;
          if (sector >= SMART_FIRST_DIR_SECTOR)
            {
              /* Error, wear status bit too large! */
              fdbg("Invalid geometry - wear level status too large\n");
              ret = -EINVAL;
              goto errout;
            }
        }
    }

  /* Now clear the NEEDS_WRITE wear status bit */

  dev->wearflags &= ~SMART_WEARFLAGS_WRITE_NEEDED;
  ret = OK;

errout:
  return ret;
}
#endif

/****************************************************************************
 * Name: smart_read_wearstatus
 *
 * Description:  Reads the wear leveling status bits from sector zero (and
 *               possibly others if it doesn't fit) such that is is persisted
 *               across OS reboots.
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_SMART_WEAR_LEVEL
static inline int smart_read_wearstatus(FAR struct smart_struct_s *dev)
{
  uint16_t  sector;
  uint16_t  remaining, toread;
  struct smart_read_write_s req;
  int       ret;
  uint8_t   buffer[8];

  /* Prepare to read the total block erases and uneven wearcount values */

  sector = 0;
  req.logsector = sector;
  req.offset = SMARTFS_FMT_WEAR_POS - 8;
  req.count = sizeof(buffer);
  req.buffer = buffer;
  ret = smart_readsector(dev, (unsigned long) &req);
  if (ret != sizeof(buffer))
    {
      goto errout;
    }

  /* Get the uneven wearcount value */

  dev->uneven_wearcount = *((uint32_t *) &buffer[4]);

  /* Check for erased state */

#if ( CONFIG_SMARTFS_ERASEDSTATE == 0xFF )
  if (dev->uneven_wearcount == 0xFFFFFFFF)
    {
      dev->uneven_wearcount = 0;
    }
#endif

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_SMARTFS)
  /* Get the block erases count */

  dev->blockerases = *((uint32_t *) buffer);
#if ( CONFIG_SMARTFS_ERASEDSTATE == 0xFF )
  if (dev->blockerases == 0xFFFFFFFF)
    {
      dev->blockerases = 0;
    }
#endif
#endif

  /* Read all wear level bits from the flash */

  remaining = dev->geo.neraseblocks >> 1;
  while (remaining)
    {
      /* Calculate number of bytes to read from this sector */

      toread = remaining;
      if (toread > dev->sectorsize - SMARTFS_FMT_WEAR_POS)
        {
          toread = dev->sectorsize - SMARTFS_FMT_WEAR_POS;
        }

      /* Setup the sector read request (we are our own client) */

      req.logsector = sector;
      req.offset = SMARTFS_FMT_WEAR_POS;
      req.count = toread;
      req.buffer = &dev->wearstatus[(dev->geo.neraseblocks >> SMART_WEAR_BIT_DIVIDE) -
                    remaining];

      /* Read the sector */

      ret = smart_readsector(dev, (unsigned long) &req);
      if (ret != toread)
        {
          goto errout;
        }

      /* Decrement the remaining count */

      remaining -= toread;
      if (remaining)
        {
          /* Data doesn't fit in a single sector.  Use the reserved sectors */

          sector++;
          if (sector >= SMART_FIRST_DIR_SECTOR)
            {
              /* Error, wear status bit too large! */

              fdbg("Invalid geometry - wear level status too large\n");
              ret = -EINVAL;
              goto errout;
            }
        }
    }

  /* Now interrogate the status bits */

  smart_find_wear_minmax(dev);

#ifdef CONFIG_MTD_SMART_SECTOR_ERASE_DEBUG
  /* Set the erase counts equal to the wear levels */

  for (sector = 0; sector < dev->geo.neraseblocks; sector++)
    {
      dev->erasecounts[sector] = smart_get_wear_level(dev, sector);
    }
#endif

  ret = OK;

errout:
  return ret;
}
#endif

/****************************************************************************
 * Name: smart_write_alloc_sector
 *
 * Description:  Writes a newly allocated sector's header to the RW buffer
 *               and updates sector mapping variables.  If CRC isn't enabled
 *               it also writes the header to the device.
 *
 ****************************************************************************/

#ifdef CONFIG_FS_WRITABLE
static int smart_write_alloc_sector(FAR struct smart_struct_s *dev,
                    uint16_t logical, uint16_t physical)
{
  int       ret = 1;
  uint8_t   sectsize;
  FAR struct smart_sect_header_s  *header;

  memset(dev->rwbuffer, CONFIG_SMARTFS_ERASEDSTATE, dev->sectorsize);
  header = (FAR struct smart_sect_header_s *) dev->rwbuffer;
  *((FAR uint16_t *) header->logicalsector) = logical;
#if SMART_STATUS_VERSION == 1
#ifdef CONFIG_MTD_SMART_ENABLE_CRC
  header->seq = 0;
#else
  *((FAR uint16_t *) &header->crc8) = 0;
#endif  /* CONFIG_MTD_SMART_ENABLE_CRC */
#else
  header->seq = 0;
#endif
  sectsize = dev->sectorsize >> 7;

#if CONFIG_SMARTFS_ERASEDSTATE == 0xFF
  header->status = ~(SMART_STATUS_COMMITTED | SMART_STATUS_SIZEBITS |
          SMART_STATUS_VERBITS) | SMART_STATUS_VERSION | sectsize;
#ifdef CONFIG_MTD_SMART_ENABLE_CRC
  header->status &= ~SMART_STATUS_CRC;
#endif  /* CONFIG_MTD_SMART_ENABLE_CRC */
#else
  header->status = SMART_STATUS_COMMITTED | SMART_STATUS_VERSION | sectsize;
#ifdef CONFIG_MTD_SMART_ENABLE_CRC
  header->status |= SMART_STATUS_CRC;
#endif  /* CONFIG_MTD_SMART_ENABLE_CRC */
#endif

  /* Write the header to the physical sector location */

#ifndef CONFIG_MTD_SMART_ENABLE_CRC
  fvdbg("Write MTD block %d\n", physical * dev->mtdBlksPerSector);
  ret = MTD_BWRITE(dev->mtd, physical * dev->mtdBlksPerSector, 1,
      (FAR uint8_t *) dev->rwbuffer);
  if (ret != 1)
    {
      /* The block is not empty!!  What to do? */

      fdbg("Write block %d failed: %d.\n", physical *
          dev->mtdBlksPerSector, ret);

      /* Unlock the mutex if we add one */

      return -EIO;
    }
#endif  /* CONFIG_MTD_SMART_ENABLE_CRC */

  return ret;
}
#endif

/****************************************************************************
 * Name: smart_validate_crc
 *
 * Description:  Validates the CRC data in the sector's header against the
 *               data in the sector.  Assumes the entire sector has been
 *               read into the RW buffer already.
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_SMART_ENABLE_CRC
static int smart_validate_crc(FAR struct smart_struct_s *dev)
{
  crc_t       crc;
  FAR struct  smart_sect_header_s *header;

  /* Calculate CRC on data region of the sector */

  crc = smart_calc_sector_crc(dev);
  header = (FAR struct smart_sect_header_s *) dev->rwbuffer;

#ifdef CONFIG_SMART_CRC_8

  /* Test 8-bit CRC */

  if (crc != header->crc8)
    {
      return -EIO;
    }

#elif defined(CONFIG_SMART_CRC_16)

  /* Test 16-bit CRC */

  if (crc != *((uint16_t *) header->crc16))
    {
      return -EIO;
    }

#elif defined(CONFIG_SMART_CRC_32)

  if (crc != *((uint32_t *) header->crc32))
    {
      return -EIO;
    }

#endif

  /* CRC checkout out okay */

  return OK;
}
#endif

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
static int smart_writesector(FAR struct smart_struct_s *dev,
                    unsigned long arg)
{
  int         ret;
  bool        needsrelocate = FALSE;
  uint32_t    mtdblock;
  uint16_t    physsector, oldphyssector, block;
  FAR struct  smart_read_write_s *req;
  FAR struct  smart_sect_header_s *header;
  size_t      offset;
  uint8_t     byte;
#if defined(CONFIG_MTD_SMART_WEAR_LEVEL) || !defined(CONFIG_MTD_SMART_ENABLE_CRC)
  uint16_t    x;
#endif
#ifdef CONFIG_MTD_SMART_ENABLE_CRC
  FAR struct  smart_allocsector_s *allocsector;
#endif

  fvdbg("Entry\n");
  req = (FAR struct smart_read_write_s *) arg;
  DEBUGASSERT(req->offset <= dev->sectorsize);
  DEBUGASSERT(req->offset+req->count <= dev->sectorsize);

  /* Ensure the logical sector has been allocated */

  if (req->logsector >= dev->totalsectors)
    {
      fdbg("Logical sector %d too large\n", req->logsector);

      ret = -EINVAL;
      goto errout;
    }
  header = (FAR struct smart_sect_header_s *) dev->rwbuffer;

#ifdef CONFIG_MTD_SMART_WEAR_LEVEL
  /* Test if an adjustement to the wear levels is needed */

  if (dev->minwearlevel >= SMART_WEAR_MIN_LEVEL ||
      (dev->minwearlevel > 0 && dev->maxwearlevel >= SMART_WEAR_REORG_THRESHOLD))
    {
      /* Subtract dev->minwearlevel from all wear levels */

      offset = dev->minwearlevel;
      fvdbg("Reducing wear level bits by %d\n", offset);
      for (x = 0; x < dev->geo.neraseblocks; x++)
        {
          smart_set_wear_level(dev, x, smart_get_wear_level(dev, x) - offset);
        }

      dev->minwearlevel -= offset;
      dev->maxwearlevel -= offset;

      /* Now write the new wear bits to the flash */

      dev->wearflags &= ~SMART_WEARFLAGS_FORCE_REORG;
      dev->wearflags |= SMART_WEARFLAGS_WRITE_NEEDED;
  }
#endif

#ifndef CONFIG_MTD_SMART_MINIMIZE_RAM
  physsector = dev->sMap[req->logsector];
#else
  physsector = smart_cache_lookup(dev, req->logsector);
#endif
  if (physsector == 0xFFFF)
    {
      fdbg("Logical sector %d not allocated\n", req->logsector);
      ret = -EINVAL;
      goto errout;
    }

  /* Read the sector data into our buffer */

  mtdblock = physsector * dev->mtdBlksPerSector;
  ret = MTD_BREAD(dev->mtd, mtdblock, dev->mtdBlksPerSector, (FAR uint8_t *)
          dev->rwbuffer);
  if (ret != dev->mtdBlksPerSector)
    {
      fdbg("Error reading phys sector %d\n", physsector);
      ret = -EIO;
      goto errout;
    }

  /* Test if we need to relocate the sector to perform the write */

#ifdef CONFIG_MTD_SMART_ENABLE_CRC
  allocsector = dev->allocsector;
  while (allocsector)
    {
      /* Test if the requested logical sector is a temp alloc */

      if (allocsector->logical == req->logsector)
        {
          break;
        }

      allocsector = allocsector->next;
    }

  /* When CRC is enabled, then we always have to relocate the sector if
   * it is not a temporary alloc (i.e. initial alloc before the very first
   * write operation).
   */

  if (!allocsector)
    {
      needsrelocate = TRUE;
    }

#else
  /* When CRC is not enabled, we may be able to simply add the new data to
   * the sector if it doesn't conflict with existing data on the device.
   * Test if there is a conflict in the data.
   */

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
#endif  /* CONFIG_MTD_SMART_ENABLE_CRC */

  /* If we are not using CRC and on a device that supports re-writing
     bits from 1 to 0 without neededing a block erase, such as NOR
     FLASH, then we can simply update the data in place and don't need
     to relocate the sector.  Test if we need to relocate or not.
   */

  if (needsrelocate)
    {
      /* Find a new physical sector to save data to */

      oldphyssector = physsector;
      physsector = smart_findfreephyssector(dev, FALSE);
      if (physsector == 0xFFFF)
        {
          fdbg("Error relocating sector %d\n", req->logsector);
          ret = -EIO;
          goto errout;
        }

      /* Update the sequence number to indicate the sector was moved */

#if SMART_STATUS_VERSION == 1
      if (header->status & SMART_STATUS_CRC)
        {
#endif
          header->seq++;
          if (header->seq == 0xFF)
            {
              header->seq = 0;
            }
#if SMART_STATUS_VERSION == 1
        }
      else
        {
          (*((FAR uint16_t *) &header->seq))++;
          if (*((FAR uint16_t *) &header->seq) == 0xFFFF)
            *((FAR uint16_t *) &header->seq) = 1;
        }
#else
      header->seq++;
#endif
#if CONFIG_SMARTFS_ERASEDSTATE == 0xFF
      header->status |= SMART_STATUS_COMMITTED;
#else
      header->status &= SMART_STATUS_COMMITTED;
#endif
    }

#ifdef CONFIG_MTD_SMART_ENABLE_CRC
  /* When CRC is enabled and we have a temp alloc, then fill in the RW buffer
   * with the header information prior to copying the write data to the buf.
   */

  if (allocsector)
    {
      smart_write_alloc_sector(dev, allocsector->logical, allocsector->physical);

      /* Remove allocsector from the list and free the memory */

      if (dev->allocsector == allocsector)
        {
          /* We are the head item.  Remove ourselves as head */

          dev->allocsector = allocsector->next;
        }
      else
        {
          FAR struct smart_allocsector_s *prev;

          /* Start at head and find our entry */

          prev = dev->allocsector;
          while (prev && prev->next != allocsector)
            {
              /* Scan the list until we find this entry */

              prev = prev->next;
            }

          if (prev)
            {
              /* Remove from the list */

              prev->next = allocsector->next;
            }
        }

      /* Now free the memory */

      kmm_free(allocsector);
    }

  /* Now copy the data to the sector buffer. */

  memcpy(&dev->rwbuffer[sizeof(struct smart_sect_header_s) + req->offset],
          req->buffer, req->count);

  /* Commit the sector ahead of time.  The CRC will protect us */

#if CONFIG_SMARTFS_ERASEDSTATE == 0xFF
  header->status &= ~(SMART_STATUS_COMMITTED | SMART_STATUS_CRC);
#else
  header->status |= SMART_STATUS_COMMITTED | SMART_STATUS_CRC;
#endif

  /* Now calculate the CRC value for the sector */

#ifdef CONFIG_SMART_CRC_8
  header->crc8 = smart_calc_sector_crc(dev);
#elif defined(CONFIG_SMART_CRC_16)
  *((uint16_t *) header->crc16) = smart_calc_sector_crc(dev);
#elif defined(CONFIG_SMART_CRC_32)
  *((uint32_t *) header->crc32) = smart_calc_sector_crc(dev);
#endif

#else  /* CONFIG_MTD_SMART_ENABLE_CRC */

  /* Now copy the data to the sector buffer. */

  memcpy(&dev->rwbuffer[sizeof(struct smart_sect_header_s) + req->offset],
          req->buffer, req->count);

#endif  /* CONFIG_MTD_SMART_ENABLE_CRC */

  /* Now write the sector buffer to the device. */

  if (needsrelocate)
    {
      /* Write the entire sector to the new physical location, uncommitted. */

      ret = MTD_BWRITE(dev->mtd, physsector * dev->mtdBlksPerSector,
              dev->mtdBlksPerSector, (FAR uint8_t *) dev->rwbuffer);
      if (ret != dev->mtdBlksPerSector)
        {
          fdbg("Error writing to physical sector %d\n", physsector);
          ret = -EIO;
          goto errout;
        }

      /* Commit the new physical sector */

#ifndef CONFIG_MTD_SMART_ENABLE_CRC

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
#endif

      /* Release the old physical sector */

#if CONFIG_SMARTFS_ERASEDSTATE == 0xFF
      byte = header->status & ~(SMART_STATUS_RELEASED | SMART_STATUS_COMMITTED);
#else
      byte = header->status | SMART_STATUS_RELEASED | SMART_STATUS_COMMITTED;
#endif
      offset = mtdblock * dev->geo.blocksize +
          offsetof(struct smart_sect_header_s, status);
      ret = smart_bytewrite(dev, offset, 1, &byte);

      /* Update releasecount for released sector and freecount for the
       * newly allocated physical sector. */

      block = oldphyssector / dev->sectorsPerBlk;
#ifdef CONFIG_MTD_SMART_PACK_COUNTS
      smart_add_count(dev, dev->releasecount, block, 1);
      smart_add_count(dev, dev->freecount, physsector / dev->sectorsPerBlk, -1);
#else
      dev->releasecount[block]++;
      dev->freecount[physsector / dev->sectorsPerBlk]--;
#endif
      dev->freesectors--;
      dev->releasesectors++;

#ifdef CONFIG_SMART_LOCAL_CHECKFREE
      /* Perform debug free count checking enabled */

      smart_checkfree(dev, __LINE__);
#endif

      /* Update the sector map */

#ifndef CONFIG_MTD_SMART_MINIMIZE_RAM
      dev->sMap[req->logsector] = physsector;
#else
      smart_update_cache(dev, req->logsector, physsector);
#endif

      /* Test if releasing the sector created an empty erase block */

      smart_erase_block_if_empty(dev, block, FALSE);

      /* Since we performed a relocation, do garbage collection to
       * ensure we don't fill up our flash with released blocks.
       */

      smart_garbagecollect(dev);
    }
  else  /* needsrelocate */
    {
#ifdef CONFIG_MTD_SMART_ENABLE_CRC
      /* Write the entire sector to FLASH when CRC enabled */

      ret = MTD_BWRITE(dev->mtd, physsector * dev->mtdBlksPerSector,
              dev->mtdBlksPerSector, (FAR uint8_t *) dev->rwbuffer);
      if (ret != dev->mtdBlksPerSector)
        {
          fdbg("Error writing to physical sector %d\n", physsector);
          ret = -EIO;
          goto errout;
        }

      /* Read the sector back and validate the CRC. */

      ret = MTD_BREAD(dev->mtd, physsector * dev->mtdBlksPerSector,
              dev->mtdBlksPerSector, (FAR uint8_t *) dev->rwbuffer);
      if (ret == dev->mtdBlksPerSector)
        {
          /* Validate the CRC of the read-back data */

          ret = smart_validate_crc(dev);
        }

      if (ret != OK)
        {
          /* TODO: Mark this as a bad block! */

          fdbg("Error validating physical sector %d\n", physsector);
          ret = -EIO;
          goto errout;
        }
#else
      /* Not relocated.  Just write the portion of the sector that needs
       * to be written. */

      offset = mtdblock * dev->geo.blocksize +
          sizeof(struct smart_sect_header_s) + req->offset;
      ret = smart_bytewrite(dev, offset, req->count, req->buffer);
#endif
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

static int smart_readsector(FAR struct smart_struct_s *dev,
                    unsigned long arg)
{
  int       ret;
  uint16_t  physsector;
  FAR struct smart_read_write_s *req;
#ifdef CONFIG_MTD_SMART_ENABLE_CRC
#if SMART_STATUS_VERSION == 1
  FAR struct smart_sect_header_s *header;
#endif
#else
  uint32_t  readaddr;
  struct smart_sect_header_s header;
#endif

  fvdbg("Entry\n");
  req = (FAR struct smart_read_write_s *) arg;
  DEBUGASSERT(req->offset < dev->sectorsize);
  DEBUGASSERT(req->offset+req->count < dev->sectorsize);

  /* Ensure the logical sector has been allocated */

  if (req->logsector >= dev->totalsectors)
    {
      fdbg("Logical sector %d too large\n", req->logsector);

      ret = -EINVAL;
      goto errout;
    }

#ifndef CONFIG_MTD_SMART_MINIMIZE_RAM
  physsector = dev->sMap[req->logsector];
#else
  physsector = smart_cache_lookup(dev, req->logsector);
#endif
  if (physsector == 0xFFFF)
    {
      fdbg("Logical sector %d not allocated\n", req->logsector);
      ret = -EINVAL;
      goto errout;
    }

#ifdef CONFIG_MTD_SMART_ENABLE_CRC

    /* When CRC is enabled, we read the entire sector into RAM so we can
     * validate the CRC.
     */

    ret = MTD_BREAD(dev->mtd, physsector * dev->mtdBlksPerSector,
        dev->mtdBlksPerSector, (FAR uint8_t *) dev->rwbuffer);
    if (ret != dev->mtdBlksPerSector)
      {
        /* TODO:  Mark the block bad */

        fdbg("Error reading phys sector %d\n", physsector);
        ret = -EIO;
        goto errout;
      }

#if SMART_STATUS_VERSION == 1
  /* Test if this sector has CRC enabled or not */

  header = (FAR struct smart_sect_header_s *) dev->rwbuffer;
  if ((header->status & SMART_STATUS_CRC) == (CONFIG_SMARTFS_ERASEDSTATE & SMART_STATUS_CRC))
    {
      /* Format VERSION 1 supports either no CRC or 8-bit CRC.  Looks like
       * CRC not enabled for this sector, so skip the CRC test.
       */

    }
  else
#endif
    {
      /* Validate the read CRC against the calculated sector CRC */

      ret = smart_validate_crc(dev);
      if (ret != OK)
        {
          /* TODO: Mark the block bad */

          fdbg("Error validating sector %d CRC during read\n", physsector);
          ret = -EIO;
          goto errout;
        }
    }

  /* Copy data to the output buffer */

  memmove((FAR char *) req->buffer, &dev->rwbuffer[req->offset +
      sizeof(struct smart_sect_header_s)], req->count);
  ret = req->count;

#else /* CONFIG_MTD_SMART_ENABLE_CRC */

  /* Read the sector header data to validate as a sanity check */

  ret = MTD_READ(dev->mtd, physsector * dev->mtdBlksPerSector * dev->geo.blocksize,
          sizeof(struct smart_sect_header_s), (FAR uint8_t *) &header);
  if (ret != sizeof(struct smart_sect_header_s))
    {
      fvdbg("Error reading sector %d header\n", physsector);
      ret = -EIO;
      goto errout;
    }

  /* Do a sanity check on the header data */

  if (((*(FAR uint16_t *) header.logicalsector) != req->logsector) ||
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

  ret = MTD_READ(dev->mtd, readaddr, req->count, (FAR uint8_t *)
          req->buffer);
  if (ret != req->count)
    {
      fdbg("Error reading phys sector %d\n", physsector);
      ret = -EIO;
      goto errout;
    }

#endif

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
static inline int smart_allocsector(FAR struct smart_struct_s *dev,
                    unsigned long requested)
{
  int       x;
  uint16_t  logsector = 0xFFFF; /* Logical sector number selected */
  uint16_t  physicalsector;     /* The selected physical sector */
#ifndef CONFIG_MTD_SMART_ENABLE_CRC
  int       ret;
#endif

  /* Validate that we have enough sectors available to perform an
   * allocation.  We have to ensure we keep enough reserved sectors
   * on hand to do released sector garbage collection. */

  if (dev->freesectors <= (dev->sectorsPerBlk << 0) + 4)
    {
      /* Do a garbage collect and then test freesectors again */

      if (dev->releasesectors + dev->freesectors > dev->availSectPerBlk + 4)
        {

          for (x = 0; x < dev->availSectPerBlk; x++)
            {
              smart_garbagecollect(dev);

              if (dev->freesectors > dev->availSectPerBlk + 4)
                break;
            }

          if (dev->freesectors <= (dev->availSectPerBlk << 0) + 4)
            {
              /* No space left!! */

              return -ENOSPC;
            }
        }
      else
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

#ifndef CONFIG_MTD_SMART_MINIMIZE_RAM
      if (dev->sMap[requested] == (uint16_t) -1)
#else
      if (!(dev->sBitMap[requested >> 3] & (1 << (requested & 0x07))))
#endif
        {
#ifdef CONFIG_MTD_SMART_ENABLE_CRC
          FAR struct smart_allocsector_s *allocsect;

          /* Ensure this logical sector doesn't have a temporary alloc */
          allocsect = dev->allocsector;
          while (allocsect)
            {
              if (allocsect->logical == requested)
                {
                  break;
                }

              allocsect = allocsect->next;
            }

          if (allocsect != NULL)
            {
            }
          else
#endif
            logsector = requested;
        }
    }

  /* Check if we need to scan for an available logical sector */

  if (logsector == 0xFFFF)
    {
      /* Loop through all sectors and find one to allocate */

      for (x = SMART_FIRST_ALLOC_SECTOR; x < dev->totalsectors; x++)
        {
#ifndef CONFIG_MTD_SMART_MINIMIZE_RAM
          if (dev->sMap[x] == (uint16_t) -1)
#else
          if (!(dev->sBitMap[x >> 3] & (1 << (x & 0x07))))
#endif
            {
#ifdef CONFIG_MTD_SMART_ENABLE_CRC
              FAR struct smart_allocsector_s *allocsect;

              /* Ensure this logical sector doesn't have a temporary alloc
               * when CRC is enabled.  With CRC enabled, when a sector is
               * allocated, we don't actually update the FLASH until the
               * very end when we have all data so the CRC can be calculated.
               * Instead, we keep an in-memory linked list of allocated
               * sectors until the write sector occurs.
               */

              allocsect = dev->allocsector;
              while (allocsect)
                {
                  if (allocsect->logical == x)
                    {
                      break;
                    }

                  allocsect = allocsect->next;
                }

              if (allocsect != NULL)
                {
                  /* This logical sector has an in-memory temp alloc */

                  continue;
                }
#endif
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
   * ensure we keep enough reserved free sectors to perform garbage
   * collection as it involves moving sectors from blocks with
   * released sectors into blocks with free sectors, then
   * erasing the vacated block. */

  smart_garbagecollect(dev);

  /* Find a free physical sector */

  physicalsector = smart_findfreephyssector(dev, FALSE);
  fvdbg("Alloc: log=%d, phys=%d, erase block=%d, free=%d, released=%d\n",
          logsector, physicalsector, physicalsector /
          dev->sectorsPerBlk, dev->freesectors, releasecount);

#ifdef CONFIG_MTD_SMART_ENABLE_CRC

  /* When CRC is enabled, we don't write the header to the device until
   * the data is written via writesector.  Just add the allocation to
   * our temporary allocsector list and we'll pick it up later.
   */

  {
    FAR struct smart_allocsector_s *allocsect = (FAR struct smart_allocsector_s *)
      kmm_malloc(sizeof(struct smart_allocsector_s));
    if (allocsect == NULL)
      {
        fdbg("Out of memory allocting sector\n");
        return -ENOMEM;
      }

    /* Fill in the struct and add to the list.  We are protected by the
     * smartfs layer's mutex, so no locking required.
     */

    allocsect->logical = logsector;
    allocsect->physical = physicalsector;
    allocsect->next = dev->allocsector;
    dev->allocsector = allocsect;
  }

#else /* CONFIG_MTD_SMART_ENABLE_CRC */

  /* Write the logical sector to the flash.  We will fill it in with data later. */

  ret = smart_write_alloc_sector(dev, logsector, physicalsector);
  if (ret != 1)
    {
      /* Error writing sector, return error */

      return ret;
    }
#endif  /* CONFIG_MTD_SMART_ENABLE_CRC */

  /* Map the sector and update the free sector counts */

#ifndef CONFIG_MTD_SMART_MINIMIZE_RAM
  dev->sMap[logsector] = physicalsector;
#else
  dev->sBitMap[logsector >> 3] |= (1 << (logsector & 0x07));
  smart_add_sector_to_cache(dev, logsector, physicalsector, __LINE__ );
#endif

#ifdef CONFIG_MTD_SMART_PACK_COUNTS
  smart_add_count(dev, dev->freecount, physicalsector / dev->sectorsPerBlk, -1);
#else
  dev->freecount[physicalsector / dev->sectorsPerBlk]--;
#endif
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
static inline int smart_freesector(FAR struct smart_struct_s *dev,
                    unsigned long logicalsector)
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

#ifndef CONFIG_MTD_SMART_MINIMIZE_RAM
      if (dev->sMap[logicalsector] == (uint16_t) -1)
#else
      if (!(dev->sBitMap[logicalsector >> 3] & (1 << (logicalsector & 0x07))))
#endif
        {
          fdbg("Invalid release - sector %d not allocated\n", logicalsector);
          ret = -EINVAL;
          goto errout;
        }
    }

  /* Okay to release the sector.  Read the sector header info */

#ifndef CONFIG_MTD_SMART_MINIMIZE_RAM
  physsector = dev->sMap[logicalsector];
#else
  physsector = smart_cache_lookup(dev, logicalsector);
#endif
  readaddr = physsector * dev->mtdBlksPerSector * dev->geo.blocksize;
  ret = MTD_READ(dev->mtd, readaddr, sizeof(struct smart_sect_header_s),
                 (FAR uint8_t *) &header);
  if (ret != sizeof(struct smart_sect_header_s))
    {
      goto errout;
    }

  /* Do a sanity check on the logical sector number */

  if (*((FAR uint16_t *) header.logicalsector) != (uint16_t) logicalsector)
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
      fdbg("Error updating physical sector %d status\n", physsector);
      goto errout;
    }

  /* Update the erase block's release count */

  dev->releasesectors++;
  block = physsector / dev->sectorsPerBlk;
#ifdef CONFIG_MTD_SMART_PACK_COUNTS
  smart_add_count(dev, dev->releasecount, block, 1);
#else
  dev->releasecount[block]++;
#endif

  /* Unmap this logical sector */

#ifndef CONFIG_MTD_SMART_MINIMIZE_RAM
  dev->sMap[logicalsector] = (uint16_t) -1;
#else
  dev->sBitMap[logicalsector >> 3] &= ~(1 << (logicalsector & 0x07));
  smart_update_cache(dev, logicalsector, 0xFFFF);
#endif

  /* If this block has only released blocks, then erase it */

  smart_erase_block_if_empty(dev, block, FALSE);
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
  FAR struct smart_struct_s *dev ;
  int ret;
#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_SMARTFS)
  FAR struct mtd_smart_procfs_data_s *procfs_data;
  FAR struct mtd_smart_debug_data_s *debug_data;
#endif

  fvdbg("Entry\n");
  DEBUGASSERT(inode && inode->i_private);

#ifdef CONFIG_SMARTFS_MULTI_ROOT_DIRS
  dev = ((FAR struct smart_multiroot_device_s*) inode->i_private)->dev;
#else
  dev = (FAR struct smart_struct_s *)inode->i_private;
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
      ret = smart_getformat(dev, (FAR struct smart_format_s *) arg,
        ((FAR struct smart_multiroot_device_s*) inode->i_private)->rootdirnum);
#else
      ret = smart_getformat(dev, (FAR struct smart_format_s *) arg);
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

#ifdef CONFIG_MTD_SMART_WEAR_LEVEL
      if (dev->wearflags & SMART_WEARFLAGS_WRITE_NEEDED)
        {
          /* Write new wear status bits to the device */

          smart_write_wearstatus(dev);
        }
#endif

      goto ok_out;
#endif /* CONFIG_FS_WRITABLE */

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_SMARTFS)
    case BIOC_GETPROCFSD:

      /* Get ProcFS data */

      procfs_data = (FAR struct mtd_smart_procfs_data_s *) arg;
      procfs_data->totalsectors = dev->totalsectors;
      procfs_data->sectorsize = dev->sectorsize;
      procfs_data->freesectors = dev->freesectors;
      procfs_data->releasesectors = dev->releasesectors;
      procfs_data->namelen = dev->namesize;
      procfs_data->formatversion = dev->formatversion;
      procfs_data->unusedsectors = dev->unusedsectors;
      procfs_data->blockerases = dev->blockerases;
      procfs_data->sectorsperblk = dev->sectorsPerBlk;

#ifndef CONFIG_MTD_SMART_MINIMIZE_RAM
      procfs_data->formatsector = dev->sMap[0];
      procfs_data->dirsector = dev->sMap[3];
#else
      procfs_data->formatsector = smart_cache_lookup(dev, 0);
      procfs_data->dirsector = smart_cache_lookup(dev, 3);
#endif

#ifdef CONFIG_MTD_SMART_SECTOR_ERASE_DEBUG
      procfs_data->neraseblocks = dev->geo.neraseblocks;
      procfs_data->erasecounts = dev->erasecounts;
#endif
#ifdef CONFIG_MTD_SMART_ALLOC_DEBUG
      procfs_data->allocs = dev->alloc;
      procfs_data->alloccount = SMART_MAX_ALLOCS;
#endif
#ifdef CONFIG_MTD_SMART_WEAR_LEVEL
      procfs_data->uneven_wearcount = dev->uneven_wearcount;
#endif
      ret = OK;
      goto ok_out;
#endif

    case BIOC_DEBUGCMD:
#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_SMARTFS)
      debug_data = (FAR struct mtd_smart_debug_data_s *) arg;
      switch (debug_data->debugcmd)
        {
        case SMART_DEBUG_CMD_SET_DEBUG_LEVEL:
          dev->debuglevel = debug_data->debugdata;
          dbg("Debug level set to %d\n", dev->debuglevel);

          ret = OK;
          goto ok_out;
        }
#endif

      break;
    }

  /* No other block driver ioctl commands are not recognized by this
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

int smart_initialize(int minor, FAR struct mtd_dev_s *mtd, FAR const char *partname)
{
  FAR struct smart_struct_s *dev;
  int ret = -ENOMEM;
  uint32_t  totalsectors;
#ifdef CONFIG_SMARTFS_MULTI_ROOT_DIRS
  FAR struct smart_multiroot_device_s *rootdirdev = NULL;
#endif

  /* Sanity check */

#ifdef CONFIG_DEBUG
  if (minor < 0 || minor > 255 || !mtd)
    {
      return -EINVAL;
    }
#endif

  /* Allocate a SMART device structure */

  dev = (FAR struct smart_struct_s *)kmm_malloc(sizeof(struct smart_struct_s));
  if (dev)
    {
      /* Initialize the SMART device structure */

      dev->mtd = mtd;
#ifdef CONFIG_MTD_SMART_ALLOC_DEBUG
      dev->bytesalloc = 0;
      for (totalsectors = 0; totalsectors < SMART_MAX_ALLOCS; totalsectors++)
        {
          dev->alloc[totalsectors].ptr = NULL;
        }
#endif

      /* Get the device geometry. (casting to uintptr_t first eliminates
       * complaints on some architectures where the sizeof long is different
       * from the size of a pointer).
       */

      /* Set these to zero in case the device doesn't support them */

      ret = MTD_IOCTL(mtd, MTDIOC_GEOMETRY, (unsigned long)((uintptr_t)&dev->geo));
      if (ret < 0)
        {
          fdbg("MTD ioctl(MTDIOC_GEOMETRY) failed: %d\n", ret);
          goto errout;
        }

      /* Set the sector size to the default for now */

#ifndef CONFIG_MTD_SMART_MINIMIZE_RAM
      dev->sMap = NULL;
#else
      dev->sCache = NULL;
      dev->sBitMap = NULL;
#endif
      dev->rwbuffer = NULL;
#ifdef CONFIG_MTD_SMART_SECTOR_ERASE_DEBUG
      dev->erasecounts = NULL;
#endif
#ifdef CONFIG_MTD_SMART_WEAR_LEVEL
      dev->wearstatus = NULL;
#endif
#ifdef CONFIG_MTD_SMART_ENABLE_CRC
      dev->allocsector = NULL;
#endif
      dev->sectorsize = 0;
      ret = smart_setsectorsize(dev, CONFIG_MTD_SMART_SECTOR_SIZE);
      if (ret != OK)
        {
          goto errout;
        }

      /* Calculate the totalsectors on this device and validate */

      totalsectors = dev->neraseblocks * dev->sectorsPerBlk;
      if (totalsectors > 65536)
        {
          fdbg("SMART Sector size too small for device\n");
          ret = -EINVAL;
          goto errout;
        }
      else if (totalsectors == 65536)
        {
          totalsectors -= 2;
        }

      dev->totalsectors = (uint16_t) totalsectors;
      dev->freesectors = (uint16_t) dev->availSectPerBlk * dev->geo.neraseblocks;
      dev->lastallocblock = 0;
      dev->debuglevel = 0;

      /* Mark the device format status an unknown */

      dev->formatstatus = SMART_FMT_STAT_UNKNOWN;
      dev->namesize = CONFIG_SMARTFS_MAXNAMLEN;
      if (partname)
        {
          strncpy(dev->partname, partname, SMART_PARTNAME_SIZE);
        }
      else
        {
          dev->partname[0] = '\0';
        }

#ifdef CONFIG_SMARTFS_MULTI_ROOT_DIRS
      dev->minor = minor;
#endif

      /* Create a MTD block device name */

#ifdef CONFIG_SMARTFS_MULTI_ROOT_DIRS
      if (partname != NULL)
        {
          snprintf(dev->rwbuffer, 18, "/dev/smart%d%sd1", minor, partname);
        }
      else
        {
          snprintf(dev->rwbuffer, 18, "/dev/smart%dd1", minor);
        }

      /* Inode private data is a reference to a struct containing
       * the SMART device structure and the root directory number.
       */

      rootdirdev = (FAR struct smart_multiroot_device_s*) smart_malloc(dev,
          sizeof(*rootdirdev), "Root Dir");
      if (rootdirdev == NULL)
        {
          fdbg("register_blockdriver failed: %d\n", -ret);
          ret = -ENOMEM;
          goto errout;
        }

      /* Populate the rootdirdev */

      rootdirdev->dev = dev;
      rootdirdev->rootdirnum = 0;
      ret = register_blockdriver(dev->rwbuffer, &g_bops, 0, rootdirdev);

#else
      if (partname != NULL)
        {
          snprintf(dev->rwbuffer, 18, "/dev/smart%d%s", minor, partname);
        }
      else
        {
          snprintf(dev->rwbuffer, 18, "/dev/smart%d", minor);
        }

      /* Inode private data is a reference to the SMART device structure */

      ret = register_blockdriver(dev->rwbuffer, &g_bops, 0, dev);
#endif

      if (ret < 0)
        {
          fdbg("register_blockdriver failed: %d\n", -ret);
          goto errout;
        }

      /* Do a scan of the device */

      smart_scan(dev);
    }

  return OK;

errout:
#ifndef CONFIG_MTD_SMART_MINIMIZE_RAM
  smart_free(dev, dev->sMap);
#else
  smart_free(dev, dev->sBitMap);
  smart_free(dev, dev->sCache);
#endif
  smart_free(dev, dev->rwbuffer);
#ifdef CONFIG_MTD_SMART_WEAR_LEVEL
  smart_free(dev, dev->wearstatus);
#endif
#ifdef CONFIG_MTD_SMART_SECTOR_ERASE_DEBUG
  smart_free(dev, dev->erasecounts);
#endif
#ifdef CONFIG_SMARTFS_MULTI_ROOT_DIRS
  if (rootdirdev)
    {
      smart_free(dev,rootdirdev);
    }
#endif

  kmm_free(dev);
  return ret;
}
