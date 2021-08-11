/****************************************************************************
 * drivers/mtd/smart.c
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
#include <sys/ioctl.h>
#include <sys/mount.h>
#include <sys/stat.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/crc8.h>
#include <nuttx/crc16.h>
#include <nuttx/crc32.h>
#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/mtd/smart.h>
#include <nuttx/fs/smart.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if 0 /* Define to enable checking */
#  define CONFIG_SMART_LOCAL_CHECKFREE
#endif

#define SMART_STATUS_COMMITTED    0x80
#define SMART_STATUS_RELEASED     0x40
#define SMART_STATUS_CRC          0x20
#define SMART_STATUS_SIZEBITS     0x1c
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
#define SMART_SECTSIZE_2048       0x0c
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
#define SMART_FIRST_ALLOC_SECTOR    12      /* First logical sector number
                                             * we will use for assignment
                                             * of requested alloc sectors.
                                             * All entries below this are
                                             * reserved (some for root dir
                                             * entries other for our use
                                             * such as format, sector,
                                             * etc.) */

#if defined(CONFIG_MTD_SMART_READAHEAD) || (defined(CONFIG_DRVR_WRITABLE) && \
    defined(CONFIG_MTD_SMART_WRITEBUFFER))
#  define SMART_HAVE_RWBUFFER 1
#endif

#ifndef CONFIG_MTD_SMART_SECTOR_SIZE
#  define CONFIG_MTD_SMART_SECTOR_SIZE 1024
#endif

#ifndef offsetof
#define offsetof(type, member) ( (size_t) &( ( (type *) 0)->member))
#endif

#define SMART_MAX_ALLOCS        10

#ifndef CONFIG_MTD_SMART_ALLOC_DEBUG
#define smart_malloc(d, b, n)   kmm_malloc(b)
#define smart_zalloc(d, b, n)   kmm_zalloc(b)
#define smart_free(d, p)        kmm_free(p)
#endif

#define SMART_WEAR_FULL_RELOCATE_THRESHOLD  8
#define SMART_WEAR_REORG_THRESHOLD          14
#define SMART_WEAR_MIN_LEVEL                5
#define SMART_WEAR_FORCE_REORG_THRESHOLD    1
#define SMART_WEAR_BIT_DIVIDE               1
#define SMART_WEAR_ZERO_MASK                0x0f
#define SMART_WEAR_BLOCK_MASK               0x01

#define SMART_WEARFLAGS_FORCE_REORG         0x01
#define SMART_WEARFLAGS_WRITE_NEEDED        0x02

#define SET_BITMAP(m, n) do { (m)[(n) / 8] |= 1 << ((n) % 8); } while (0)
#define CLR_BITMAP(m, n) do { (m)[(n) / 8] &= ~(1 << ((n) % 8)); } while (0)
#define ISSET_BITMAP(m, n) ((m)[(n) / 8] & (1 << ((n) % 8)))

#ifdef CONFIG_SMARTFS_ALIGNED_ACCESS
#  define SMARTFS_NEXTSECTOR(h) \
  (uint16_t)((FAR const uint8_t *)(h)->nextsector)[1] << 8 | \
  (uint16_t)((FAR const uint8_t *)(h)->nextsector)[0]

#  define SMARTFS_SET_NEXTSECTOR(h, v) \
  do \
    { \
      ((FAR uint8_t *)(h)->nextsector)[0] = (v) & 0xff; \
      ((FAR uint8_t *)(h)->nextsector)[1] = (v) >> 8;   \
    } while (0)

#else
#  define SMARTFS_NEXTSECTOR(h) (*((FAR uint16_t *)(h)->nextsector))
#  define SMARTFS_SET_NEXTSECTOR(h, v) \
  do \
    { \
      ((*((FAR uint16_t *)(h)->nextsector)) = (uint16_t)(v)); \
    } while (0)

#endif

#ifdef CONFIG_MTD_SMART_WEAR_LEVEL

/****************************************************************************
 * Private Data
 ****************************************************************************/

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

static const uint8_t g_wearlevel_to_bitmap4[] =
{
  0x0f, 0x0e, 0x0c, 0x08,     /* Single bit erased (x3) */
  0x07, 0x06, 0x04, 0x00,     /* Single bit erased (x3) */
  0x0b, 0x0a, 0x02,           /* Single bit erased (x2) */
  0x0d, 0x09, 0x01,           /* Single bit erased (x2) */
  0x03,
  0x05
};

/* Map a Wear Level bit pattern back to the wear level */

static const uint8_t g_wearbit_to_levelmap4[] =
{
  7, 13, 10, 14, 6, 15, 5, 4,
  3, 12, 9,  8,  2, 11, 1, 0
};
#endif

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
  uint16_t              mtdblkspersector; /* Number of MTD blocks per SMART Sector */
  uint16_t              sectorsperblk;    /* Number of sectors per erase block */
  uint16_t              sectorsize;       /* Sector size on device */
  uint16_t              totalsectors;     /* Total number of sectors on device */
  uint32_t              erasesize;        /* Size of an erase block */
  FAR uint8_t          *releasecount;     /* Count of released sectors per erase block */
  FAR uint8_t          *freecount;        /* Count of free sectors per erase block */
  FAR char             *rwbuffer;         /* Our sector read/write buffer */
  char                  partname[SMART_PARTNAME_SIZE];
  uint8_t               formatversion;    /* Format version on the device */
  uint8_t               formatstatus;     /* Indicates the status of the device format */
  uint8_t               namesize;         /* Length of filenames on this device */
  uint8_t               debuglevel;       /* Debug reporting level */
  uint8_t               availsectperblk;  /* Number of usable sectors per erase block */
#ifdef CONFIG_SMARTFS_MULTI_ROOT_DIRS
  uint8_t               rootdirentries;   /* Number of root directory entries */
  uint8_t               minor;            /* Minor number of the block entry */
#endif
#ifdef CONFIG_MTD_SMART_WEAR_LEVEL
  uint8_t               wearflags;        /* Indicates force erase of static blocks needed */
  uint8_t               minwearlevel;     /* Min level in the wear level bits */
  uint8_t               maxwearlevel;     /* Max level in the wear level bits */
  uint8_t              *wearstatus;       /* Array of wear leveling bits */
  uint32_t              uneven_wearcount; /* Number of times the wear level has gone over max */
#endif
#ifdef CONFIG_MTD_SMART_ENABLE_CRC
  FAR struct smart_allocsector_s  *allocsector; /* Pointer to first alloc sector */
#endif
#ifndef CONFIG_MTD_SMART_MINIMIZE_RAM
  FAR uint16_t         *smap;             /* Virtual to physical sector map */
#else
  FAR uint8_t          *sbitmap;          /* Virtual sector used bit-map */
  FAR struct smart_cache_s *scache;       /* Sector cache */
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

#ifdef CONFIG_SMARTFS_MULTI_ROOT_DIRS
struct smart_multiroot_device_s
{
  FAR struct smart_struct_s *dev;
  uint8_t               rootdirnum;
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
                                           * Bit 7:   1 = Not committed
                                           *          0 = committed
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
                                           * Bit 7:   1 = Not committed
                                           *          0 = committed
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
                                           * Bit 7:   1 = Not committed
                                           *          0 = committed
                                           * Bit 6:   1 = Not released
                                           *          0 = released
                                           * Bit 5:   Sector CRC enable
                                           * Bit 4-2: Sector size on volume
                                           * Bit 1-0: Format version (0x3) */
  uint8_t               seq;              /* Incrementing sequence number */
};

typedef uint32_t crc_t;

#endif

/* Following two definitions copied from internal definition of fs/smartfs.
 * Because needed to search chain_header and entry_header.
 */

#if defined(CONFIG_MTD_SMART_ENABLE_CRC) && defined(CONFIG_SMART_CRC_32)
struct smart_chain_header_s
{
  uint8_t           nextsector[4]; /* Next logical sector in the chain */
  uint8_t           used[4];       /* Number of bytes used in this sector */
  uint8_t           type;          /* Type of sector entry (file or dir) */
};
#else
struct smart_chain_header_s
{
  uint8_t           type;          /* Type of sector entry (file or dir) */
  uint8_t           nextsector[2]; /* Next logical sector in the chain */
  uint8_t           used[2];       /* Number of bytes used in this sector */
};
#endif

struct smart_entry_header_s
{
  uint16_t          flags;         /* Flags, including permissions:
                                    *  15:   Empty entry
                                    *  14:   Active entry
                                    *  12-0: Permissions bits */
  int16_t           firstsector;   /* Sector number of the name */
  uint32_t          utc;           /* Time stamp */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     smart_open(FAR struct inode *inode);
static int     smart_close(FAR struct inode *inode);
static ssize_t smart_reload(struct smart_struct_s *dev, FAR uint8_t *buffer,
                 off_t startblock, size_t nblocks);
static ssize_t smart_read(FAR struct inode *inode, unsigned char *buffer,
                 blkcnt_t start_sector, unsigned int nsectors);
static ssize_t smart_write(FAR struct inode *inode,
                 FAR const unsigned char *buffer, blkcnt_t start_sector,
                 unsigned int nsectors);
static int     smart_geometry(FAR struct inode *inode,
                 FAR struct geometry *geometry);
static int     smart_ioctl(FAR struct inode *inode, int cmd,
                 unsigned long arg);

static int     smart_findfreephyssector(FAR struct smart_struct_s *dev,
                                        uint8_t canrelocate);

static int     smart_writesector(FAR struct smart_struct_s *dev,
                 unsigned long arg);
static inline int smart_allocsector(FAR struct smart_struct_s *dev,
                 unsigned long requested);
static int     smart_readsector(FAR struct smart_struct_s *dev,
                 unsigned long arg);

#ifdef CONFIG_MTD_SMART_ENABLE_CRC
static int     smart_validate_crc(FAR struct smart_struct_s *dev);
#endif
#ifdef CONFIG_MTD_SMART_WEAR_LEVEL
static int     smart_read_wearstatus(FAR struct smart_struct_s *dev);
static int     smart_relocate_static_data(FAR struct smart_struct_s *dev,
                 uint16_t block);
#endif

static int     smart_relocate_sector(FAR struct smart_struct_s *dev,
                 uint16_t oldsector, uint16_t newsector);

#ifdef CONFIG_MTD_SMART_FSCK
static int     smart_fsck(FAR struct smart_struct_s *dev);
#endif

#ifdef CONFIG_SMART_DEV_LOOP
static ssize_t smart_loop_read(FAR struct file *filep, FAR char *buffer,
                 size_t buflen);
static ssize_t smart_loop_write(FAR struct file *filep,
                 FAR const char *buffer, size_t buflen);
static int     smart_loop_ioctl(FAR struct file *filep, int cmd,
                 unsigned long arg);
#endif /* CONFIG_SMART_DEV_LOOP */

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct block_operations g_bops =
{
  smart_open,     /* open     */
  smart_close,    /* close    */
  smart_read,     /* read     */
  smart_write,    /* write    */
  smart_geometry, /* geometry */
  smart_ioctl     /* ioctl    */
};

#ifdef CONFIG_SMART_DEV_LOOP
static const struct file_operations g_fops =
{
  NULL,             /* open */
  NULL,             /* close */
  smart_loop_read,  /* read */
  smart_loop_write, /* write */
  NULL,             /* seek */
  smart_loop_ioctl, /* ioctl */
};
#endif /* CONFIG_SMART_DEV_LOOP */

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
  finfo("Entry\n");
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
  finfo("Entry\n");
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
FAR static void *smart_malloc(FAR struct smart_struct_s *dev,
                    size_t bytes, const char *name)
{
  FAR void *ret = kmm_malloc(bytes);
  uint8_t x;

  /* Test if we are allocating the dev struct */

  if (dev == NULL)
    {
      dev = ret;
      dev->bytesalloc = 0;
      for (x = 0; x < SMART_MAX_ALLOCS; x++)
        {
          dev->alloc[x].ptr = NULL;
        }
    }

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

  finfo("SMART alloc: %ld\n", dev->bytesalloc);
  return ret;
}
#endif

/****************************************************************************
 * Name: smart_zalloc
 *
 * Description:  Perform allocations and keep track of amount of allocated
 *               memory for this context.
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_SMART_ALLOC_DEBUG
FAR static void *smart_zalloc(FAR struct smart_struct_s *dev,
                    size_t bytes, const char *name)
{
  void *mem;

  mem = smart_malloc(dev, bytes, name);
  if (mem != NULL)
    {
      memset(mem, 0, bytes);
    }

  return mem;
}
#endif

/****************************************************************************
 * Name: smart_free
 *
 * Description:  Perform smart memory free operation.
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_SMART_ALLOC_DEBUG
static void smart_free(FAR struct smart_struct_s *dev, FAR void *ptr)
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
static void smart_set_count(FAR struct smart_struct_s *dev,
                            FAR uint8_t *pcount, uint16_t block,
                            uint8_t count)
{
  if (dev->sectorsperblk > 16)
    {
      pcount[block] = count;
    }
  else
    {
      /* Save the lower 4 bits of the count in a shared byte */

      if (block & 0x01)
        {
          pcount[block >> 1] = (pcount[block >> 1] & 0xf0) |
                               (count & 0x0f);
        }
      else
        {
          pcount[block >> 1] = (pcount[block >> 1] & 0x0f) |
                               ((count & 0x0f) << 4);
        }

      /* If we have 16 sectors per block, then the upper bit (representing
       * 16) all get packed into shared bytes.
       */

      if (dev->sectorsperblk == 16)
        {
          if (count == 16)
            {
              pcount[(dev->geo.neraseblocks >> 1) + (block >> 3)] |=
                1 << (block & 0x07);
            }
          else
            {
              pcount[(dev->geo.neraseblocks >> 1) + (block >> 3)] &=
                ~(1 << (block & 0x07));
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
                    FAR uint8_t *pcount, uint16_t block)
{
  uint8_t count;

  if (dev->sectorsperblk > 16)
    {
      count = pcount[block];
    }
  else
    {
      /* Save the lower 4 bits of the count in a shared byte */

      if (block & 0x01)
        {
          count = pcount[block >> 1] & 0x0f;
        }
      else
        {
          count = pcount[block >> 1] >> 4;
        }

      /* If we have 16 sectors per block, then the upper bit (representing
       * 16) all get packed into shared bytes.
       */

      if (dev->sectorsperblk == 16)
        {
          if (pcount[(dev->geo.neraseblocks >> 1) +
              (block >> 3)] & (1 << (block & 0x07)))
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
static void smart_add_count(struct smart_struct_s *dev, uint8_t *pcount,
                            uint16_t block, int adder)
{
  int16_t   value;

  value = smart_get_count(dev, pcount, block) + adder;
  smart_set_count(dev, pcount, block, value);
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
  uint16_t        x;
  uint16_t        freecount;
#ifdef CONFIG_DEBUG_FS
  uint16_t        blockfree;
  uint16_t        blockrelease;
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
      fwarn("WARNING: Free count incorrect in line %d!  Calculated=%d, "
            "dev->freesectors=%d\n",
            lineno, freecount, dev->freesectors);

      /* Determine what changed from the last time which caused this error */

      fwarn("   ... Prev freesectors=%d, prev releasesectors=%d\n",
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
              if (prev_freecount[x] != blockfree ||
                  prev_releasecount[x] != blockrelease)
                {
                  /* This block's values are different from the last time ...
                   * report it.
                   */

                  fwarn("   ... Block %d:  Old Free=%d, old release=%d,    "
                        "New free=%d, new release = %d\n",
                        x, prev_freecount[x], prev_releasecount[x],
                        blockfree, blockrelease);
                }
            }
        }

      /* Modify the freesector count to reflect the actual calculated
       * freecount to get us back in line.
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
      prev_freecount = (FAR uint8_t *)
        smart_malloc(dev, dev->neraseblocks << 1, "Free backup");
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
 * Description:  Read the specified number of sectors
 *
 ****************************************************************************/

static ssize_t smart_reload(struct smart_struct_s *dev, FAR uint8_t *buffer,
                            off_t startblock, size_t nblocks)
{
  ssize_t nread;
  ssize_t mtdblocks;
  ssize_t mtdstartblock;

  /* Calculate the number of MTD blocks to read */

  mtdblocks = nblocks * dev->mtdblkspersector;

  /* Calculate the first MTD block number */

  mtdstartblock = startblock * dev->mtdblkspersector;

  /* Read the full erase block into the buffer */

  finfo("Read %zu blocks starting at block %zu\n",
        mtdblocks, mtdstartblock);
  nread = MTD_BREAD(dev->mtd, mtdstartblock, mtdblocks, buffer);
  if (nread != mtdblocks)
    {
      ferr("ERROR: Read %zd blocks starting at block %" PRIdOFF
           " failed: %zd\n", nblocks, startblock, nread);
    }

  return nread;
}

/****************************************************************************
 * Name: smart_read
 *
 * Description:  Read the specified number of sectors
 *
 ****************************************************************************/

static ssize_t smart_read(FAR struct inode *inode, unsigned char *buffer,
                          blkcnt_t start_sector, unsigned int nsectors)
{
  FAR struct smart_struct_s *dev;

  finfo("SMART: sector: %" PRIuOFF " nsectors: %u\n",
        start_sector, nsectors);

  DEBUGASSERT(inode && inode->i_private);
#ifdef CONFIG_SMARTFS_MULTI_ROOT_DIRS
  dev = ((FAR struct smart_multiroot_device_s *)inode->i_private)->dev;
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

static ssize_t smart_write(FAR struct inode *inode,
                FAR const unsigned char *buffer,
                blkcnt_t start_sector, unsigned int nsectors)
{
  FAR struct smart_struct_s *dev;
  off_t  alignedblock;
  off_t  mask;
  off_t  blkstowrite;
  off_t  offset;
  off_t  nextblock;
  off_t  mtdblkspererase;
  off_t  eraseblock;
  size_t remaining;
  size_t nxfrd;
  int    ret;
  off_t  mtdstartblock;
  off_t  mtdblockcount;

  finfo("sector: %" PRIuOFF " nsectors: %u\n", start_sector, nsectors);

  DEBUGASSERT(inode && inode->i_private);
#ifdef CONFIG_SMARTFS_MULTI_ROOT_DIRS
  dev = ((FAR struct smart_multiroot_device_s *)inode->i_private)->dev;
#else
  dev = (FAR struct smart_struct_s *)inode->i_private;
#endif

  /* I think maybe we need to lock on a mutex here */

  /* Get the aligned block.  Here is is assumed: (1) The number of R/W blocks
   * per erase block is a power of 2, and (2) the erase begins with that same
   * alignment.
   */

  mask         = dev->sectorsperblk - 1;
  alignedblock = ((start_sector + mask) & ~mask) * dev->mtdblkspersector;

  /* Convert SMART blocks into MTD blocks */

  mtdstartblock = start_sector * dev->mtdblkspersector;
  mtdblockcount = nsectors * dev->mtdblkspersector;
  mtdblkspererase = dev->mtdblkspersector * dev->sectorsperblk;

  finfo("mtdsector: %" PRIdOFF " mtdnsectors: %" PRIdOFF "\n",
        mtdstartblock, mtdblockcount);

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

          eraseblock = alignedblock / mtdblkspererase;
          ret = MTD_ERASE(dev->mtd, eraseblock, 1);
          if (ret < 0)
            {
              ferr("ERROR: Erase block=%" PRIdOFF " failed: %d\n",
                   eraseblock, ret);

              /* Unlock the mutex if we add one */

              return ret;
            }
        }

      /* Calculate the number of blocks to write. */

      blkstowrite = mtdblkspererase;
      if (nextblock != alignedblock)
        {
          blkstowrite = alignedblock - nextblock;
        }

      if (blkstowrite > remaining)
        {
          blkstowrite = remaining;
        }

      /* Try to write to the sector. */

      finfo("Write MTD block %" PRIdOFF " from offset %" PRIdOFF "\n",
            nextblock, offset);
      nxfrd = MTD_BWRITE(dev->mtd, nextblock, blkstowrite, &buffer[offset]);
      if (nxfrd != blkstowrite)
        {
          /* The block is not empty!!  What to do? */

          ferr("ERROR: Write block %" PRIdOFF " failed: %zd.\n",
               nextblock, nxfrd);

          /* Unlock the mutex if we add one */

          return -EIO;
        }

      /* Then update for amount written */

      nextblock += blkstowrite;
      remaining -= blkstowrite;
      offset += blkstowrite * dev->geo.blocksize;
      alignedblock += mtdblkspererase;
    }

  return nsectors;
}

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

  finfo("Entry\n");

  DEBUGASSERT(inode);
  if (geometry)
    {
#ifdef CONFIG_SMARTFS_MULTI_ROOT_DIRS
      dev = ((FAR struct smart_multiroot_device_s *)inode->i_private)->dev;
#else
      dev = (FAR struct smart_struct_s *)inode->i_private;
#endif
      geometry->geo_available     = true;
      geometry->geo_mediachanged  = false;
      geometry->geo_writeenabled  = true;

      erasesize                   = dev->geo.erasesize;
      geometry->geo_nsectors      = dev->geo.neraseblocks * erasesize /
                                    dev->sectorsize;
      geometry->geo_sectorsize    = dev->sectorsize;

      strcpy(geometry->geo_model, dev->geo.model);

      finfo("available: true mediachanged: false writeenabled: %s\n",
            geometry->geo_writeenabled ? "true" : "false");
      finfo("nsectors: %" PRIuOFF " sectorsize: %" PRIi16 "\n",
            geometry->geo_nsectors, geometry->geo_sectorsize);

      return OK;
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: smart_setsectorsize
 *
 * Description: Sets the device's sector size and recalculates sector size
 *              dependent variables.
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

  erasesize             = dev->geo.erasesize;
  dev->neraseblocks     = dev->geo.neraseblocks;
  dev->erasesize        = erasesize;
  dev->sectorsize       = size;
  dev->mtdblkspersector = dev->sectorsize / dev->geo.blocksize;

  DEBUGASSERT(dev->sectorsize >= dev->geo.blocksize);
  DEBUGASSERT(erasesize / dev->sectorsize <= 256);

  if (erasesize / dev->sectorsize > 256)
    {
      /* We can't throw a error message here because it is too early.
       * set the erasesize to zero and exit, then we will detect
       * it during mksmartfs or mount.
       */

      dev->erasesize = 0;
      dev->sectorsperblk = 256;
      dev->availsectperblk = 255;
    }
  else
    {
      /* Set the sectors per erase block and available sectors per erase
       * block
       */

      dev->sectorsperblk = erasesize / dev->sectorsize;
      if (dev->sectorsperblk == 256)
        {
          dev->availsectperblk = 255;
        }
      else if (dev->sectorsperblk == 0)
        {
          return -EINVAL;
        }
      else
        {
          dev->availsectperblk = dev->sectorsperblk;
        }
    }

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_SMARTFS)
  dev->unusedsectors = 0;
  dev->blockerases = 0;
#endif

  /* Release any existing rwbuffer and smap */

#ifndef CONFIG_MTD_SMART_MINIMIZE_RAM
  if (dev->smap != NULL)
    {
      smart_free(dev, dev->smap);
      dev->smap = NULL;
    }

#else
  if (dev->sbitmap != NULL)
    {
      smart_free(dev, dev->sbitmap);
      dev->sbitmap = NULL;
    }

  dev->cache_entries = 0;
  dev->cache_lastlog = 0xffff;
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

  totalsectors = dev->neraseblocks * dev->sectorsperblk;

  /* Validate the number of total sectors is small enough for a uint16_t */

  if (totalsectors > 65536)
    {
      ferr("ERROR: Invalid SMART sector count %" PRIu32 "\n", totalsectors);
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
  dev->smap = (FAR uint16_t *)
    smart_malloc(dev, totalsectors * sizeof(uint16_t) + allocsize,
                 "Sector map");
  if (!dev->smap)
    {
      ferr("ERROR: Error allocating SMART virtual map buffer\n");
      goto errexit;
    }

  dev->releasecount = (FAR uint8_t *) dev->smap +
                      (totalsectors * sizeof(uint16_t));
  dev->freecount = dev->releasecount + dev->neraseblocks;
#else
  dev->sbitmap = (FAR uint8_t *)
    smart_malloc(dev, (totalsectors + 7) >> 3, "Sector Bitmap");
  if (dev->sbitmap == NULL)
    {
      ferr("ERROR: Error allocating SMART sector cache\n");
      goto errexit;
    }

  /* Calculate the alloc size of the freesector and release sector arrays */

#ifdef CONFIG_MTD_SMART_PACK_COUNTS
  if (dev->sectorsperblk > 16)
    {
      allocsize = dev->neraseblocks << 1;
    }
  else if (dev->sectorsperblk == 16)
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

  if (dev->scache == NULL)
    {
      dev->scache = (FAR struct smart_cache_s *) smart_malloc(dev,
        CONFIG_MTD_SMART_SECTOR_CACHE_SIZE * sizeof(struct smart_cache_s) +
        allocsize, "Sector Cache");
    }

  if (!dev->scache)
    {
      ferr("ERROR: Error allocating SMART sector cache\n");
      goto errexit;
    }

  dev->releasecount = (FAR uint8_t *)dev->scache +
    (CONFIG_MTD_SMART_SECTOR_CACHE_SIZE * sizeof(struct smart_cache_s));

#ifdef CONFIG_MTD_SMART_PACK_COUNTS
  if (dev->sectorsperblk > 16)
    {
      dev->freecount = dev->releasecount + dev->neraseblocks;
    }
  else if (dev->sectorsperblk == 16)
    {
      dev->freecount = dev->releasecount + (dev->neraseblocks >> 1) +
                       (dev->neraseblocks >> 3);
    }
  else
    {
      dev->freecount = dev->releasecount + (dev->neraseblocks >> 1);
    }

#else
  dev->freecount = dev->releasecount + dev->neraseblocks;
#endif

#endif /* CONFIG_MTD_SMART_MINIMIZE_RAM */

#ifdef CONFIG_MTD_SMART_SECTOR_ERASE_DEBUG
  /* Allocate a buffer to hold the erase counts */

  if (dev->erasecounts == NULL)
    {
      dev->erasecounts = (FAR uint8_t *)
        smart_malloc(dev, dev->neraseblocks, "Erase counts");
    }

  if (!dev->erasecounts)
    {
      ferr("ERROR: Error allocating erase count array\n");
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
      ferr("ERROR: Error allocating wear level status array\n");
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
      ferr("ERROR: Error allocating SMART read/write buffer\n");
      goto errexit;
    }

  return OK;

  /* On error for any allocation, we jump here and free anything that had
   * previously been allocated.
   */

errexit:
#ifndef CONFIG_MTD_SMART_MINIMIZE_RAM
  if (dev->smap)
    {
      smart_free(dev, dev->smap);
      dev->smap = NULL;
    }

#else
  if (dev->sbitmap)
    {
      smart_free(dev, dev->sbitmap);
      dev->sbitmap = NULL;
    }

  if (dev->scache)
    {
      smart_free(dev, dev->scache);
      dev->scache = NULL;
    }
#endif

#ifdef CONFIG_MTD_SMART_WEAR_LEVEL
  if (dev->wearstatus)
    {
      smart_free(dev, dev->wearstatus);
      dev->wearstatus = NULL;
    }
#endif

#ifdef CONFIG_MTD_SMART_SECTOR_ERASE_DEBUG
  if (dev->erasecounts)
    {
      smart_free(dev, dev->erasecounts);
      dev->erasecounts = NULL;
    }
#endif

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
                    dev->geo.blocksize - 1) / dev->geo.blocksize;

      DEBUGASSERT(nblocks <= dev->mtdblkspersector);

      /* Do a block read */

      ret = MTD_BREAD(dev->mtd, startblock, nblocks,
                      (FAR uint8_t *)dev->rwbuffer);
      if (ret < 0)
        {
          ferr("ERROR: Error %zd reading from device\n", -ret);
          goto errout;
        }

      /* Modify the data */

      memcpy(&dev->rwbuffer[offset - startblock * dev->geo.blocksize],
             buffer, nbytes);

      /* Write the data back to the device */

      ret = MTD_BWRITE(dev->mtd, startblock, nblocks,
                       (FAR uint8_t *) dev->rwbuffer);
      if (ret < 0)
        {
          ferr("ERROR: Error %zd writing to device\n", -ret);
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
 * Description: Adds a logical to physical sector mapping to the sector
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
  uint16_t index;
  uint16_t x;
  uint16_t oldest;

  /* If we aren't full yet, just add the sector to the end of the list */

  index = 1;
  if (dev->cache_entries < CONFIG_MTD_SMART_SECTOR_CACHE_SIZE)
    {
      oldest = 0;
      index  = dev->cache_entries++;
    }
  else
    {
      /* Cache is full.  We must find the least accessed entry and replace
       * it
       */

      oldest = 0xffff;
      for (x = 0; x < CONFIG_MTD_SMART_SECTOR_CACHE_SIZE; x++)
        {
          /* Never replace cache entries for system sectors */

          if (dev->scache[x].logical < SMART_FIRST_ALLOC_SECTOR)
            continue;

          /* If the hit count is zero, then choose this entry */

          if (dev->scache[x].birth < oldest)
            {
              oldest = dev->scache[x].birth;
              index  = x;
            }
        }
    }

  /* Now add the sector at index */

  dev->scache[index].logical = logical;
  dev->scache[index].physical = physical;
  dev->scache[index].birth = dev->cache_nextbirth++;
  dev->cache_lastlog = logical;
  dev->cache_lastphys = physical;

  if (dev->debuglevel > 1)
    {
      _err("Add Cache sector:  Log=%d, Phys=%d at index %d from line %d\n",
          logical, physical, index, line);
    }

  /* Test if the birthdays need to be adjusted */

  if (oldest >= CONFIG_MTD_SMART_SECTOR_CACHE_SIZE + 1024)
    {
      for (x = 0; x < dev->cache_entries; x++)
        {
          dev->scache[x].birth -= 1024;
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
 *              and add / replace a cache entry with the newly located
 *              sector.
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_SMART_MINIMIZE_RAM
static uint16_t smart_cache_lookup(FAR struct smart_struct_s *dev,
                                   uint16_t logical)
{
  int       ret;
  uint16_t  block;
  uint16_t  sector;
  uint16_t  x;
  uint16_t  physical;
  uint16_t  logicalsector;
  struct    smart_sect_header_s header;
  size_t    readaddress;

  physical = 0xffff;

  /* Test if searching for the last sector used */

  if (logical == dev->cache_lastlog)
    {
      return dev->cache_lastphys;
    }

  /* First search for the entry in the cache */

  for (x = 0; x < dev->cache_entries; x++)
    {
      if (dev->scache[x].logical == logical)
        {
          /* Entry found in the cache.  Grab the physical mapping. */

          physical = dev->scache[x].physical;
          break;
        }
    }

  /* If the entry wasn't found in the cache, then we must search the volume
   * for it and add it to the cache.
   */

  if (physical == 0xffff)
    {
      /* Now scan the MTD device.  Instead of scanning start to end, we
       * span the erase blocks and read one sector from each at a time.
       * this helps speed up the search on volumes that aren't full
       * because of sector allocation scheme will use the lower sector
       * numbers in each erase block first.
       */

      for (sector = 0;
           sector < dev->availsectperblk && physical == 0xffff;
           sector++)
        {
          /* Now scan across each erase block */

          for (block = 0; block < dev->geo.neraseblocks; block++)
            {
              /* Calculate the read address for this sector */

              readaddress = block * dev->erasesize +
                  sector * dev->sectorsize;

              /* Read the header for this sector */

              ret = MTD_READ(dev->mtd, readaddress,
                             sizeof(struct smart_sect_header_s),
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

              if ((header.status & SMART_STATUS_VERBITS) !=
                  SMART_STATUS_VERSION)
                {
                  continue;
                }

              /* Test if this is the sector we are looking for */

              if (logicalsector == logical)
                {
                  /* This is the sector we are looking for!  Add it to the
                   * cache
                   */

                  physical = block * dev->sectorsperblk + sector;
                  smart_add_sector_to_cache(dev, logical, physical,
                                            __LINE__);
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
      if (dev->scache[x].logical == logical)
        {
          /* Entry found.  Update it's physical mapping */

          dev->scache[x].physical = physical;

          /* If we are freeing a sector, then remove the logical entry from
           * the cache.
           */

          if (physical == 0xffff)
            {
                dev->scache[x].logical =
                  dev->scache[dev->cache_entries - 1].logical;
                dev->scache[x].physical =
                  dev->scache[dev->cache_entries - 1].physical;
                dev->cache_entries--;
            }

          if (dev->debuglevel > 1)
            {
              _err("Update Cache:  Log=%d, Phys=%d at index %d\n",
                   logical, physical, x);
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
static uint8_t smart_get_wear_level(FAR struct smart_struct_s *dev,
                                    uint16_t block)
{
  uint8_t bits;

  bits = dev->wearstatus[block >> SMART_WEAR_BIT_DIVIDE];
  if (block & 0x01)
    {
      /* Use the upper nibble */

      bits >>= 4;
    }
  else
    {
      /* Use the lower nibble */

      bits &= 0x0f;
    }

  /* Lookup and return the level using the BitToLevel map */

  return g_wearbit_to_levelmap4[bits];
}
#endif

/****************************************************************************
 * Name: smart_find_wear_minmax
 *
 * Description: Find the minimum and maximum wear levels.  This is used when
 *              we increment the wear level of a minimum value block so that
 *              we can detect if a new minimum exists and perform
 *              normalization of the wear-levels.
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
 *              such that entries can be updated on a NOR flash without the
 *              need to relocate the format sector (assuming CRC is not
 *              enabled, in which case a relocated is needed for ANY change).
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_SMART_WEAR_LEVEL
static int smart_set_wear_level(FAR struct smart_struct_s *dev,
                                uint16_t block, uint8_t level)
{
  uint8_t bits;
  uint8_t oldlevel;

  /* Get the old wear level to test if we need to update min / max */

  oldlevel = smart_get_wear_level(dev, block);

  /* Get the bit map for this wear level from the static map array */

  if (level > 15)
    {
      _err("ERROR: Fatal Design Error!  Wear level > 15, block=%d\n", block);

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

  bits = g_wearlevel_to_bitmap4[level];

  if (block & 0x01)
    {
      /* Use the upper nibble */

      dev->wearstatus[block >> SMART_WEAR_BIT_DIVIDE] &= 0x0f;
      dev->wearstatus[block >> SMART_WEAR_BIT_DIVIDE] |= bits << 4;
    }
  else
    {
      /* Use the lower nibble */

      dev->wearstatus[block >> SMART_WEAR_BIT_DIVIDE] &= 0xf0;
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
       * we need to rescan for min.
       */

      if (oldlevel == dev->minwearlevel)
        {
          smart_find_wear_minmax(dev);

          if (oldlevel != dev->minwearlevel)
              finfo("##### New min wear level = %d\n", dev->minwearlevel);
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
  uint16_t  sectorsize;
  uint16_t  prerelease;
  uint16_t  logicalsector;
  uint16_t  winner;
  uint16_t  loser;
  uint32_t  readaddress;
  uint32_t  offset;
  uint16_t  seq1;
  uint16_t  seq2;
  uint16_t  seqwrap;
  struct    smart_sect_header_s header;
#ifdef CONFIG_MTD_SMART_MINIMIZE_RAM
  int       dupsector;
  uint16_t  duplogsector;
#endif
#ifdef CONFIG_SMARTFS_MULTI_ROOT_DIRS
  int       x;
  char      devname[32];
  FAR struct smart_multiroot_device_s *rootdirdev;
#endif
  static const uint16_t sizetbl[8] =
  {
    CONFIG_MTD_SMART_SECTOR_SIZE,
    512, 1024, 4096, 2048, 8192, 16384, 32768
  };

  finfo("Entry\n");

  /* Find the sector size on the volume by reading headers from
   * sectors of decreasing size.  On a formatted volume, the sector
   * size is saved in the header status byte of search sector, so
   * by starting with the largest supported sector size and
   * decreasing from there, we will be sure to find data that is
   * a header and not sector data.
   */

  sectorsize = 0xffff;
  offset = 16384;

  while (sectorsize == 0xffff)
    {
      readaddress = 0;

      while (readaddress < dev->erasesize * dev->geo.neraseblocks)
        {
          /* Read the next sector from the device */

          ret = MTD_READ(dev->mtd, readaddress,
                         sizeof(struct smart_sect_header_s),
                         (FAR uint8_t *) &header);
          if (ret != sizeof(struct smart_sect_header_s))
            {
              goto err_out;
            }

          if (header.status != CONFIG_SMARTFS_ERASEDSTATE)
            {
              sectorsize =
                sizetbl[(header.status & SMART_STATUS_SIZEBITS) >> 2];
              break;
            }

          readaddress += offset;
        }

      if (sectorsize == 0xffff)
        {
          sectorsize = CONFIG_MTD_SMART_SECTOR_SIZE;
        }

      offset >>= 1;
      if (offset < 256 && sectorsize == 0xffff)
        {
          /* No valid sectors found on device.  Default the
           * sector size to the CONFIG value
           */

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

  totalsectors        = dev->totalsectors;
  dev->formatstatus   = SMART_FMT_STAT_NOFMT;
  dev->freesectors    = dev->availsectperblk * dev->geo.neraseblocks;
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
      smart_set_count(dev, dev->freecount, sector,
                      dev->availsectperblk - prerelease);
      smart_set_count(dev, dev->releasecount, sector, prerelease);
#else
      dev->freecount[sector] = dev->availsectperblk - prerelease;
      dev->releasecount[sector] = prerelease;
#endif
    }

  /* Initialize the sector map */

#ifndef CONFIG_MTD_SMART_MINIMIZE_RAM
  for (sector = 0; sector < totalsectors; sector++)
    {
      dev->smap[sector] = -1;
    }
#else
  /* Clear all logical sector used bits */

  memset(dev->sbitmap, 0, (dev->totalsectors + 7) >> 3);
#endif

  /* Now scan the MTD device */

  /* At first, set the loser sector as the invalid value */

  loser = totalsectors;

  for (sector = 0; sector < totalsectors; sector++)
    {
      finfo("Scan sector %d\n", sector);

      winner = sector;

      /* Calculate the read address for this sector */

      readaddress = sector * dev->mtdblkspersector * dev->geo.blocksize;

      /* Read the header for this sector */

      ret = MTD_READ(dev->mtd, readaddress,
                     sizeof(struct smart_sect_header_s),
                     (FAR uint8_t *)&header);
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

      /* This block is committed, therefore not free.  Update the
       * erase block's freecount.
       */

#ifdef CONFIG_MTD_SMART_PACK_COUNTS
      smart_add_count(dev, dev->freecount, sector / dev->sectorsperblk, -1);
#else
      dev->freecount[sector / dev->sectorsperblk]--;
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
          smart_add_count(dev, dev->releasecount,
                          sector / dev->sectorsperblk, 1);
#else
          dev->releasecount[sector / dev->sectorsperblk]++;
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

          ferr("ERROR: Invalid logical sector %d at physical %d.\n",
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
                         (FAR uint8_t *)dev->rwbuffer);
          if (ret != 32)
            {
              ferr("ERROR: Error reading physical sector %d.\n", sector);
              goto err_out;
            }

          /* Validate the format signature */

          if (dev->rwbuffer[SMART_FMT_POS1] != SMART_FMT_SIG1 ||
              dev->rwbuffer[SMART_FMT_POS2] != SMART_FMT_SIG2 ||
              dev->rwbuffer[SMART_FMT_POS3] != SMART_FMT_SIG3 ||
              dev->rwbuffer[SMART_FMT_POS4] != SMART_FMT_SIG4)
            {
              /* Invalid signature on a sector claiming to be sector 0!
               * What should we do?  Release it?
               */

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
                  snprintf(devname, sizeof(devname), "/dev/smart%d%sd%d",
                           dev->minor, dev->partname, x + 1);
                }
              else
                {
                  snprintf(devname, sizeof(devname), "/dev/smart%dd%d",
                           dev->minor, x + 1);
                }

              /* Inode private data is a reference to a struct containing
               * the SMART device structure and the root directory number.
               */

              rootdirdev = (struct smart_multiroot_device_s *)
                smart_malloc(dev, sizeof(*rootdirdev), "Root Dir");
              if (rootdirdev == NULL)
                {
                  ferr("ERROR: Memory alloc failed\n");
                  ret = -ENOMEM;
                  goto err_out;
                }

              /* Populate the rootdirdev */

              rootdirdev->dev = dev;
              rootdirdev->rootdirnum = x;

              /* Inode private data is a reference to the SMART device
               * structure.
               */

              ret = register_blockdriver(devname, &g_bops, 0, rootdirdev);
            }
#endif
        }

      /* Test for duplicate logical sectors on the device */

#ifndef CONFIG_MTD_SMART_MINIMIZE_RAM
      if (dev->smap[logicalsector] != 0xffff)
#else
      if (dev->sbitmap[logicalsector >> 3] & (1 << (logicalsector & 0x07)))
#endif
        {
          /* Uh-oh, we found more than 1 physical sector claiming to be
           * the same logical sector.  Use the sequence number information
           * to resolve who wins.
           */

#if SMART_STATUS_VERSION == 1
          if ((header.status & SMART_STATUS_CRC) !=
                  (CONFIG_SMARTFS_ERASEDSTATE & SMART_STATUS_CRC))
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
          readaddress = dev->smap[logicalsector] * dev->mtdblkspersector *
                        dev->geo.blocksize;
#else
          /* For minimize RAM, we have to rescan to find the 1st sector
           * claiming to be this logical sector.
           */

          for (dupsector = 0; dupsector < sector; dupsector++)
            {
              /* Calculate the read address for this sector */

              readaddress = dupsector * dev->mtdblkspersector *
                            dev->geo.blocksize;

              /* Read the header for this sector */

              ret = MTD_READ(dev->mtd, readaddress,
                             sizeof(struct smart_sect_header_s),
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

              if ((header.status & SMART_STATUS_VERBITS) !=
                  SMART_STATUS_VERSION)
                {
                  continue;
                }

              /* Now compare if this logical sector matches the current
               * sector
               */

              if (duplogsector == logicalsector)
                {
                  break;
                }
            }
#endif

          ret = MTD_READ(dev->mtd, readaddress,
                         sizeof(struct smart_sect_header_s),
                         (FAR uint8_t *) &header);
          if (ret != sizeof(struct smart_sect_header_s))
            {
              goto err_out;
            }

#if SMART_STATUS_VERSION == 1
          if ((header.status & SMART_STATUS_CRC) !=
                  (CONFIG_SMARTFS_ERASEDSTATE & SMART_STATUS_CRC))
            {
              seq1 = header.seq;
              seqwrap = 0xf0;
            }
          else
            {
              seq1 = *((FAR uint16_t *) &header.seq);
              seqwrap = 0xfff0;
            }
#else
          seq1 = header.seq;
          seqwrap = 0xf0;
#endif

          /* Now determine who wins */

          if ((seq1 > seqwrap && seq2 < 10) || seq2 > seq1)
            {
              /* Seq 2 is the winner ... bigger or it wrapped */

              winner = sector;
#ifndef CONFIG_MTD_SMART_MINIMIZE_RAM
              loser = dev->smap[logicalsector];
#else
              loser = dupsector;
#endif
            }
          else
            {
              /* We keep the original mapping and seq2 is the loser */

              loser = sector;
#ifndef CONFIG_MTD_SMART_MINIMIZE_RAM
              winner = dev->smap[logicalsector];
#else
              winner = smart_cache_lookup(dev, logicalsector);
#endif
            }

          finfo("Duplicate Sector winner=%d, loser=%d\n", winner, loser);

#ifdef CONFIG_MTD_SMART_ENABLE_CRC
          /* Check CRC of the winner sector just in case */

          ret = MTD_BREAD(dev->mtd, winner * dev->mtdblkspersector,
                          dev->mtdblkspersector,
                          (FAR uint8_t *) dev->rwbuffer);
          if (ret == dev->mtdblkspersector)
            {
              /* Validate the CRC of the read-back data */

              ret = smart_validate_crc(dev);
            }

          if (ret != OK)
            {
              /* The winner sector has CRC error, so we select the loser
               * sector.  After swapping the winner and the loser sector, we
               * will release the loser sector with CRC error.
               */

              if (sector == winner)
                {
                  /* winner: sector(CRC error) -> origin
                   * loser : origin            -> sector(CRC error)
                   */

                  winner = loser;
                  loser = sector;
                }
              else
                {
                  /* winner: origin(CRC error) -> sector
                   * loser : sector            -> origin(CRC error)
                   */

                  loser = winner;
                  winner = sector;
                }

              finfo("Duplicate Sector winner=%d, loser=%d\n", winner, loser);
            }
#endif /* CONFIG_MTD_SMART_ENABLE_CRC */

          /* Now release the loser sector */

          readaddress = loser  * dev->mtdblkspersector * dev->geo.blocksize;
          ret = MTD_READ(dev->mtd, readaddress,
                         sizeof(struct smart_sect_header_s),
                        (FAR uint8_t *)&header);
          if (ret != sizeof(struct smart_sect_header_s))
            {
              goto err_out;
            }

#if CONFIG_SMARTFS_ERASEDSTATE == 0xff
          header.status &= ~SMART_STATUS_RELEASED;
#else
          header.status |= SMART_STATUS_RELEASED;
#endif
          offset = readaddress +
                   offsetof(struct smart_sect_header_s, status);
          ret    = smart_bytewrite(dev, offset, 1, &header.status);
          if (ret < 0)
            {
              ferr("ERROR: Error %d releasing duplicate sector\n", -ret);
              goto err_out;
            }
        }

      /* Test if this sector is loser of duplicate logical sector */

      if (sector == loser)
        {
          continue;
        }

#ifndef CONFIG_MTD_SMART_MINIMIZE_RAM
      /* Update the logical to physical sector map */

      dev->smap[logicalsector] = winner;
#else
      /* Mark the logical sector as used in the bitmap */

      dev->sbitmap[logicalsector >> 3] |= 1 << (logicalsector & 0x07);

      if (logicalsector < SMART_FIRST_ALLOC_SECTOR)
        {
          smart_add_sector_to_cache(dev, logicalsector, winner, __LINE__);
        }
#endif
    }

#if defined (CONFIG_MTD_SMART_WEAR_LEVEL) && (SMART_STATUS_VERSION == 1)
#ifdef CONFIG_MTD_SMART_CONVERT_WEAR_FORMAT

  /* We need to check if we are converting an older format with incorrect
   * wear leveling data in sector zero to the new format.  The old format
   * put all zeros in the wear level bit locations, but the new (better)
   * way is to leave them 0xff.
   */

#ifndef CONFIG_MTD_SMART_MINIMIZE_RAM
  sector = dev->smap[0];
#else
  sector = smart_cache_lookup(dev, 0);
#endif

  /* Validate the sector is valid ... may be an unformatted device */

  if (sector != 0xffff)
    {
      /* Read the sector data */

      ret = MTD_BREAD(dev->mtd, sector * dev->mtdblkspersector,
              dev->mtdblkspersector, (uint8_t *) dev->rwbuffer);
      if (ret != dev->mtdblkspersector)
        {
          ferr("ERROR: Error reading physical sector %d.\n", sector);
          goto err_out;
        }

      /* Check for old format wear leveling */

      if (dev->rwbuffer[SMART_WEAR_LEVEL_FORMAT_SIG] == 0)
        {
          /* Old format detected.  We must relocate sector zero and fill it
           * in with 0xff.
           */

          uint16_t newsector = smart_findfreephyssector(dev, FALSE);
          if (newsector == 0xffff)
            {
              /* Unable to find a free sector!!! */

              ferr("ERROR: Can't find a free sector for relocation\n");
              ret = -ENOSPC;
              goto err_out;
            }

          memset(&dev->rwbuffer[SMART_WEAR_LEVEL_FORMAT_SIG], 0xff,
              dev->mtdblkspersector * dev->geo.blocksize -
              SMART_WEAR_LEVEL_FORMAT_SIG);

          smart_relocate_sector(dev, sector, newsector);

          /* Update the free and release sector counts */

          dev->freesectors--;
          dev->releasesectors++;

#ifndef CONFIG_MTD_SMART_MINIMIZE_RAM
          dev->smap[0] = newsector;
          dev->freecount[newsector / dev->sectorsperblk]--;
          dev->releasecount[sector / dev->sectorsperblk]++;
#else
          smart_update_cache(dev, 0, newsector);
#ifdef CONFIG_MTD_SMART_PACK_COUNTS
          smart_add_count(dev, dev->freecount,
                          newsector / dev->sectorsperblk, -1);
          smart_add_count(dev, dev->releasecount,
                          sector / dev->sectorsperblk, 1);
#endif
#endif
        }
    }

#endif /* CONFIG_MTD_SMART_CONVERT_WEAR_FORMAT */
#endif /* CONFIG_MTD_SMART_WEAR_LEVEL && SMART_STATUS_VERSION == 1 */

#ifdef CONFIG_MTD_SMART_FSCK
  smart_fsck(dev);
#endif
#ifdef CONFIG_MTD_SMART_WEAR_LEVEL
  /* Read the wear leveling status bits */

  smart_read_wearstatus(dev);
#endif

  finfo("SMART Scan\n");
  finfo("   Erase size:   %10d\n", dev->sectorsperblk * dev->sectorsize);
  finfo("   Erase count:  %10d\n", dev->neraseblocks);
  finfo("   Sect/block:   %10d\n", dev->sectorsperblk);
  finfo("   MTD Blk/Sect: %10d\n", dev->mtdblkspersector);

  /* Validate the geometry */

  if (dev->mtdblkspersector == 0 || dev->sectorsperblk == 0 ||
      dev->sectorsperblk == 0 || dev->sectorsize == 0)
    {
      ferr("ERROR: Invalid Geometry!\n");
      ret = -EINVAL;
      goto err_out;
    }

#ifdef CONFIG_MTD_SMART_ALLOC_DEBUG
  finfo("   Allocations:\n");
  for (sector = 0; sector < SMART_MAX_ALLOCS; sector++)
    {
      if (dev->alloc[sector].ptr != NULL)
        {
          finfo("       %s: %d\n",
                dev->alloc[sector].name, dev->alloc[sector].size);
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

  finfo("Entry\n");
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

  fmt->sectorsize      = dev->sectorsize;
  fmt->availbytes      = dev->sectorsize -
                         sizeof(struct smart_sect_header_s);
  fmt->nsectors        = dev->totalsectors;
  fmt->nfreesectors    = dev->freesectors;
  fmt->namesize        = dev->namesize;
#ifdef CONFIG_SMARTFS_MULTI_ROOT_DIRS
  fmt->nrootdirentries = dev->rootdirentries;
  fmt->rootdirnum      = rootdirnum;
#endif

  /* Add the released sectors to the reported free sector count */

  fmt->nfreesectors   += dev->releasesectors;

  /* Subtract the reserved sector count */

  fmt->nfreesectors   -= dev->sectorsperblk + 4;

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
  uint16_t freecount;
  uint16_t releasecount;
  uint16_t prerelease;

#ifdef CONFIG_MTD_SMART_PACK_COUNTS
  releasecount = smart_get_count(dev, dev->releasecount, block);
  freecount = smart_get_count(dev, dev->freecount, block);
#else
  releasecount = dev->releasecount[block];
  freecount = dev->freecount[block];
#endif

  if ((freecount + releasecount == dev->availsectperblk && freecount < 1) ||
      forceerase)
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

      dev->freesectors += dev->availsectperblk - prerelease - freecount;
      dev->releasesectors -= releasecount - prerelease;

#ifdef CONFIG_MTD_SMART_PACK_COUNTS
      smart_set_count(dev, dev->releasecount, block, prerelease);
      smart_set_count(dev, dev->freecount, block,
                      dev->availsectperblk - prerelease);
#else
      dev->releasecount[block] = prerelease;
      dev->freecount[block] = dev->availsectperblk - prerelease;
#endif

      /* Now that we have erased this block and updated the release / free
       * counts, if we are in WEAR LEVELING enabled mode, we must check if
       * this erase block's wear level has reached the threshold to warrant
       * moving a minimum wear level block's data into it (i.e. relocating
       * static data to this block so it will be worn less).
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
          fwarn("   ...while eraseing block %d\n", block);
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
static int smart_relocate_static_data(FAR struct smart_struct_s *dev,
                                      uint16_t block)
{
  uint16_t freecount;
  uint16_t x;
  uint16_t sector;
  uint16_t minblock;
  uint16_t nextsector;
  uint16_t newsector;
  uint16_t mincount;
  int ret;
  FAR struct smart_sect_header_s *header;
#ifdef CONFIG_MTD_SMART_ENABLE_CRC
  FAR struct smart_allocsector_s *allocsector;
#endif

  /* Now that we have erased this block and updated the release / free
   * counts, if we are in WEAR LEVELING enabled mode, we must check if this
   * erase block's wear level has reached the threshold to warrant moving a
   * minimum wear level block's data into it (i.e. relocating static data to
   * this block so it will be worn less).
   */

  ret = OK;
  header = (FAR struct smart_sect_header_s *) dev->rwbuffer;

#ifdef CONFIG_SMART_LOCAL_CHECKFREE
  if (smart_checkfree(dev, __LINE__) != OK)
    {
      fwarn("   ...about to relocate static data %d\n", block);
    }
#endif

  if (smart_get_wear_level(dev, block) >= SMART_WEAR_FULL_RELOCATE_THRESHOLD)
    {
      /* Okay, this block is getting too worn.  Move a minimum wear level
       * block to it in it's entirety.
       */

      /* Scan all erase blocks (or until we find a minimum wear level block
       * with no free + released blocks.
       */

      freecount = dev->sectorsperblk + 1;
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
                  /* We found a minimum wear-level block with no free
                   * sectors.  relocate this block to the more highly worn
                   * block.
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
      finfo("Moving block %d, wear %d, free %d, "
            "released %d to block %d, wear %d\n",
            x, smart_get_wear_level(dev, x),
            nextsector, newsector,
            block, smart_get_wear_level(dev, block));

      nextsector = block * dev->sectorsperblk;
      for (sector = x * dev->sectorsperblk; sector <
         x * dev->sectorsperblk + dev->availsectperblk; sector++)
        {
          /* Read the next sector from this erase block */

          ret = MTD_BREAD(dev->mtd, sector * dev->mtdblkspersector,
              dev->mtdblkspersector, (FAR uint8_t *) dev->rwbuffer);
          if (ret != dev->mtdblkspersector)
            {
              ferr("ERROR: Error reading sector %d\n", sector);
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
              if (newsector == 0xffff)
                {
                  /* Unable to find a free sector!!! */

                  ferr("ERROR: Can't find a free sector for relocation\n");
                  ret = -ENOSPC;
                  goto errout;
                }

              /* Update the temporary allocation's physical sector */

              allocsector->physical = newsector;
              *((FAR uint16_t *) header->logicalsector) =
                allocsector->logical;
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

              /* Find a new sector where it can live, NOT in this erase
               * block
               */

              newsector = nextsector++;

              /* Relocate the sector data */

              if ((ret = smart_relocate_sector(dev, sector, newsector)) < 0)
                {
                  goto errout;
                }
            }

          dev->freesectors--;

#ifndef CONFIG_MTD_SMART_MINIMIZE_RAM
          dev->smap[*((FAR uint16_t *) header->logicalsector)] = newsector;
#else
          smart_update_cache(dev, *((FAR uint16_t *)header->logicalsector),
                             newsector);
#endif

#ifdef CONFIG_MTD_SMART_PACK_COUNTS
          smart_add_count(dev, dev->freecount, block, -1);
#else
          dev->freecount[block]--;
#endif /* CONFIG_MTD_SMART_PACK_COUNTS */
        }

#ifdef CONFIG_SMART_LOCAL_CHECKFREE
      if (smart_checkfree(dev, __LINE__) != OK)
        {
          fwarn("   ...about to erase static block %d\n", block);
        }
#endif

      /* Now erase the block we just relocated, force erasing it */

      smart_erase_block_if_empty(dev, x, TRUE);
    }

#ifdef CONFIG_SMART_LOCAL_CHECKFREE
  if (smart_checkfree(dev, __LINE__) != OK)
    {
      fwarn("   ...done erasing static block %d\n", block);
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
  crc_t crc = 0;

#ifdef CONFIG_SMART_CRC_8

  /* Calculate CRC on data region of the sector */

  crc = crc8((FAR uint8_t *)
             &dev->rwbuffer[sizeof(struct smart_sect_header_s)],
             dev->mtdblkspersector * dev->geo.blocksize -
             sizeof(struct smart_sect_header_s));

  /* Add logical sector number and seq to the CRC calculation */

  crc = crc8part((FAR uint8_t *)dev->rwbuffer, 3, crc);

  /* Add status to the CRC calculation */

  crc = crc8part((FAR uint8_t *)
    &dev->rwbuffer[offsetof(struct smart_sect_header_s, status)], 1, crc);

#elif defined(CONFIG_SMART_CRC_16)
  /* Calculate CRC on data region of the sector */

  crc = crc16((FAR uint8_t *)
              &dev->rwbuffer[sizeof(struct smart_sect_header_s)],
              dev->mtdblkspersector * dev->geo.blocksize -
              sizeof(struct smart_sect_header_s));

  /* Add logical sector number to the CRC calculation */

  crc = crc16part((FAR uint8_t *) dev->rwbuffer, 2, crc);

  /* Add status and seq to the CRC calculation */

  crc = crc16part((uint8_t *)
    &dev->rwbuffer[offsetof(struct smart_sect_header_s, status)], 2, crc);

#elif defined(CONFIG_SMART_CRC_32)
  /* Calculate CRC on data region of the sector */

  crc = crc32((FAR uint8_t *)
              &dev->rwbuffer[sizeof(struct smart_sect_header_s)],
              dev->mtdblkspersector * dev->geo.blocksize -
              sizeof(struct smart_sect_header_s));

  /* Add logical sector number, status and seq to the CRC calculation */

  crc = crc32part((FAR uint8_t *) dev->rwbuffer, 6, crc);
#else
#error "Unknown CRC size!"
#endif

  return crc;
}
#endif /* CONFIG_MTD_SMART_ENABLE_CRC  */

/****************************************************************************
 * Name: smart_llformat
 *
 * Description:  Performs a low-level format of the flash device.  This
 *               involves erasing the device and writing a valid sector
 *               zero (logical) with proper format signature.
 *
 * Input Parameters:
 *
 *   arg:        Upper 16 bits contains the sector size
 *               Lower 16 bits contains the number of root dir entries
 *
 ****************************************************************************/

static inline int smart_llformat(FAR struct smart_struct_s *dev,
                                 unsigned long arg)
{
  FAR struct smart_sect_header_s *sectorheader;
  size_t wrcount;
  int x;
  int ret;
  uint8_t sectsize;
  uint8_t prerelease;
  uint16_t sectorsize;

  finfo("Entry\n");

  /* Get the sector size from the provided arg */

  sectorsize = arg >> 16;
  if (sectorsize == 0)
    {
      sectorsize = CONFIG_MTD_SMART_SECTOR_SIZE;
    }

  /* Set the sector size for the device */

  ret = smart_setsectorsize(dev, sectorsize);
  if (ret != OK)
    {
      return ret;
    }

  /* Check for invalid format */

  if (dev->erasesize == 0 || dev->sectorsperblk == 0)
    {
      dev->erasesize = dev->geo.erasesize;

      ferr("ERROR:  Invalid geometery ... "
          "Sectors per erase block must be 1-256\n");
      ferr("        Erase block size    = %" PRId32 "\n",
           dev->erasesize);
      ferr("        Sector size         = %d\n",
           dev->sectorsize);
      ferr("        Sectors/erase block = %" PRId32 "\n",
           dev->erasesize / dev->sectorsize);

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
#endif /* SMART_STATUS_VERSION == 1 */

  /* Set the sector size of this sector */

  sectsize = dev->sectorsize < 4096  ? (dev->sectorsize >> 9) :
             dev->sectorsize == 4096 ? 3 : 5 + (dev->sectorsize >> 14);
  sectsize <<= 2;

  /* Set the sector logical sector to zero and setup the header status */

#if ( CONFIG_SMARTFS_ERASEDSTATE == 0xff )
  *((FAR uint16_t *) sectorheader->logicalsector) = 0;
  sectorheader->status = (uint8_t)~(SMART_STATUS_COMMITTED |
                                    SMART_STATUS_VERBITS |
                                    SMART_STATUS_SIZEBITS) |
                                    SMART_STATUS_VERSION |
                                    sectsize;
#ifdef CONFIG_MTD_SMART_ENABLE_CRC
  sectorheader->status &= ~SMART_STATUS_CRC;
#endif /* CONFIG_MTD_SMART_ENABLE_CRC */

#else   /* CONFIG_SMARTFS_ERASEDSTATE == 0xff */
  *((FAR uint16_t *) sectorheader->logicalsector) = 0xffff;
  sectorheader->status = (uint8_t)(SMART_STATUS_COMMITTED |
                                   SMART_STATUS_VERSION |
                                   sectsize);
#ifdef CONFIG_MTD_SMART_ENABLE_CRC
  sectorheader->status |= SMART_STATUS_CRC;
#endif /* CONFIG_MTD_SMART_ENABLE_CRC */
#endif /* CONFIG_SMARTFS_ERASEDSTATE == 0xff */

  /* Now add the format signature to the sector */

  dev->rwbuffer[SMART_FMT_POS1] = SMART_FMT_SIG1;
  dev->rwbuffer[SMART_FMT_POS2] = SMART_FMT_SIG2;
  dev->rwbuffer[SMART_FMT_POS3] = SMART_FMT_SIG3;
  dev->rwbuffer[SMART_FMT_POS4] = SMART_FMT_SIG4;

  dev->rwbuffer[SMART_FMT_VERSION_POS] = SMART_FMT_VERSION;
  dev->rwbuffer[SMART_FMT_NAMESIZE_POS] = CONFIG_SMARTFS_MAXNAMLEN;

  /* Record the number of root directory entries we have */

  dev->rwbuffer[SMART_FMT_ROOTDIRS_POS] = (uint8_t) (arg & 0xff);

#ifdef CONFIG_SMART_CRC_8
  sectorheader->crc8 = smart_calc_sector_crc(dev);
#elif defined(CONFIG_SMART_CRC_16)
  *((uint16_t *) sectorheader->crc16) = smart_calc_sector_crc(dev);
#elif defined(CONFIG_SMART_CRC_32)
  *((uint32_t *) sectorheader->crc32) = smart_calc_sector_crc(dev);
#endif

  /* Write the sector to the flash */

  wrcount = MTD_BWRITE(dev->mtd, 0, dev->mtdblkspersector,
          (FAR uint8_t *) dev->rwbuffer);
  if (wrcount != dev->mtdblkspersector)
    {
      /* The block is not empty!!  What to do? */

      ferr("ERROR: Write block 0 failed: %zu.\n", wrcount);

      /* Unlock the mutex if we add one */

      return -EIO;
    }

  /* Now initialize our internal control variables */

  ret = smart_setsectorsize(dev, sectorsize);
  if (ret != OK)
    {
      return ret;
    }

  dev->formatstatus     = SMART_FMT_STAT_UNKNOWN;
  dev->freesectors      = dev->availsectperblk * dev->geo.neraseblocks - 1;
  dev->releasesectors   = 0;
#ifdef CONFIG_MTD_SMART_WEAR_LEVEL
  dev->uneven_wearcount = 0;
#endif

  /* Initialize the released and free counts */

  for (x = 0; x < dev->neraseblocks; x++)
    {
      /* Test for a geometry with 65536 sectors.  We allow this, though
       * we never use the last two sectors in this mode.
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
      smart_set_count(dev, dev->freecount, x,
                      dev->availsectperblk - prerelease);
#else
      dev->releasecount[x] = prerelease;
      dev->freecount[x] = dev->availsectperblk - prerelease;
#endif
    }

  /* Account for the format sector */

#ifdef CONFIG_MTD_SMART_PACK_COUNTS
  smart_set_count(dev, dev->freecount, 0, dev->availsectperblk - 1);
#else
  dev->freecount[0]--;
#endif

  /* Now initialize the logical to physical sector map */

#ifndef CONFIG_MTD_SMART_MINIMIZE_RAM
  dev->smap[0] = 0;     /* Logical sector zero = physical sector 0 */
  for (x = 1; x < dev->totalsectors; x++)
    {
      /* Mark all other logical sectors as non-existent */

      dev->smap[x] = -1;
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

/****************************************************************************
 * Name: smart_relocate_sector
 *
 * Description:  Relocates the specified sector to the new sector location.
 *
 ****************************************************************************/

static int smart_relocate_sector(FAR struct smart_struct_s *dev,
                                 uint16_t oldsector, uint16_t newsector)
{
  size_t offset;
  FAR struct smart_sect_header_s *header;
  uint8_t newstatus;
  int ret;

  header = (FAR struct smart_sect_header_s *) dev->rwbuffer;

  /* Increment the sequence number and clear the "commit" flag */

#if SMART_STATUS_VERSION == 1
  if ((header->status & SMART_STATUS_CRC) !=
          (CONFIG_SMARTFS_ERASEDSTATE & SMART_STATUS_CRC))
    {
#endif
      /* Using 8-bit sequence */

      header->seq++;
      if (header->seq == 0xff)
        {
          header->seq = 1;
        }
#if SMART_STATUS_VERSION == 1
    }
  else
    {
      /* Using 16-bit sequence and no CRC */

      (*((FAR uint16_t *) &header->seq))++;
      if (*((FAR uint16_t *) &header->seq) == 0xffff)
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

#if CONFIG_SMARTFS_ERASEDSTATE == 0xff
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

  ret = MTD_BWRITE(dev->mtd, newsector * dev->mtdblkspersector,
                   dev->mtdblkspersector, (FAR uint8_t *) dev->rwbuffer);
  if (ret != dev->mtdblkspersector)
    {
      ferr("Error writing to new sector %d\n", newsector);
      goto errout;
    }

#else   /* CONFIG_MTD_SMART_ENABLE_CRC */

#if CONFIG_SMARTFS_ERASEDSTATE == 0xff
  header->status |= SMART_STATUS_COMMITTED;
#else
  header->status &= ~SMART_STATUS_COMMITTED;
#endif

  /* Write the data to the new physical sector location */

  ret = MTD_BWRITE(dev->mtd, newsector * dev->mtdblkspersector,
                   dev->mtdblkspersector, (FAR uint8_t *) dev->rwbuffer);
  if (ret != dev->mtdblkspersector)
    {
      ferr("Error writing to new sector %d\n", newsector);
      goto errout;
    }

  /* Commit the sector */

  offset = newsector * dev->mtdblkspersector * dev->geo.blocksize +
      offsetof(struct smart_sect_header_s, status);
#if CONFIG_SMARTFS_ERASEDSTATE == 0xff
  newstatus = header->status & ~SMART_STATUS_COMMITTED;
#else
  newstatus = header->status | SMART_STATUS_COMMITTED;
#endif
  ret = smart_bytewrite(dev, offset, 1, &newstatus);
  if (ret < 0)
    {
      ferr("ERROR: Error %d committing new sector %d\n" -ret, newsector);
      goto errout;
    }
#endif /* CONFIG_MTD_SMART_ENABLE_CRC */

  /* Release the old physical sector */

#if CONFIG_SMARTFS_ERASEDSTATE == 0xff
  newstatus = header->status & ~(SMART_STATUS_RELEASED |
                                 SMART_STATUS_COMMITTED);
#else
  newstatus = header->status | SMART_STATUS_RELEASED |
              SMART_STATUS_COMMITTED;
#endif
  offset = oldsector * dev->mtdblkspersector * dev->geo.blocksize +
      offsetof(struct smart_sect_header_s, status);
  ret = smart_bytewrite(dev, offset, 1, &newstatus);
  if (ret < 0)
    {
      ferr("ERROR: Error %d releasing old sector %d\n" -ret, oldsector);
    }

errout:
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

static int smart_relocate_block(FAR struct smart_struct_s *dev,
                                uint16_t block)
{
  uint16_t newsector;
  uint16_t oldrelease;
  int x;
  int ret;
  FAR struct smart_sect_header_s *header;
  uint8_t prerelease;
  uint16_t freecount;
#if defined(CONFIG_SMART_LOCAL_CHECKFREE) && defined(CONFIG_DEBUG_FS)
  uint16_t releasecount;
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
      fwarn("   ...while relocating block %d, free=%d\n",
            block, dev->freesectors);
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
      ferr("ERROR: Program bug!  "
           "Relocating the only block (%d) with free sectors!\n",
           block);
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

  for (x = block * dev->sectorsperblk; x <
     block * dev->sectorsperblk + dev->availsectperblk; x++)
    {
      /* Read the next sector from this erase block */

      ret = MTD_BREAD(dev->mtd, x * dev->mtdblkspersector,
          dev->mtdblkspersector, (FAR uint8_t *) dev->rwbuffer);
      if (ret != dev->mtdblkspersector)
        {
          ferr("ERROR: Error reading sector %d\n", x);
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
          if (newsector == 0xffff)
            {
              /* Unable to find a free sector!!! */

              ferr("ERROR: Can't find a free sector for relocation\n");
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
          if (newsector == 0xffff)
            {
              /* Unable to find a free sector!!! */

              ferr("ERROR: Can't find a free sector for relocation\n");
              ret = -ENOSPC;
              goto errout;
            }

          /* Relocate the sector data */

          if ((ret = smart_relocate_sector(dev, x, newsector)) < 0)
            {
              goto errout;
            }
        }

      /* Update the variables */

#ifndef CONFIG_MTD_SMART_MINIMIZE_RAM
      dev->smap[*((FAR uint16_t *) header->logicalsector)] = newsector;
#else
      smart_update_cache(dev, *((FAR uint16_t *) header->logicalsector),
                         newsector);
#endif

#ifdef CONFIG_MTD_SMART_PACK_COUNTS
      smart_add_count(dev, dev->freecount, newsector / dev->sectorsperblk,
                      -1);
#else
      dev->freecount[newsector / dev->sectorsperblk]--;
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
       * so "pre-release" them so they never get allocated.
       */

      prerelease = 2;
    }
  else
    {
      prerelease = 0;
    }

#ifdef CONFIG_MTD_SMART_PACK_COUNTS
  oldrelease               = smart_get_count(dev, dev->releasecount, block);
  dev->freesectors        += oldrelease - prerelease;
  dev->releasesectors     -= oldrelease - prerelease;
  smart_set_count(dev, dev->freecount, block,
                  dev->availsectperblk - prerelease);
  smart_set_count(dev, dev->releasecount, block, prerelease);
#else
  oldrelease               = dev->releasecount[block];
  dev->freesectors        += oldrelease - prerelease;
  dev->releasesectors     -= oldrelease - prerelease;
  dev->freecount[block]    = dev->availsectperblk - prerelease;
  dev->releasecount[block] = prerelease;
#endif

#ifdef CONFIG_SMART_LOCAL_CHECKFREE
  if (smart_checkfree(dev, __LINE__) != OK)
    {
      fwarn("   ...while relocating block %d, "
            "free=%d, release=%d, oldrelease=%d\n",
            block, freecount, releasecount, oldrelease);
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
  uint16_t count;
  uint16_t allocfreecount;
  uint16_t allocblock;
#ifdef CONFIG_MTD_SMART_WEAR_LEVEL
  uint16_t wornfreecount;
  uint16_t wornblock;
  uint8_t wearlevel;
  uint8_t wornlevel;
  uint8_t maxwearlevel;
#endif
  uint16_t physicalsector;
  uint16_t block;
  uint32_t readaddr;
  struct smart_sect_header_s header;
  int ret;
  uint16_t i;

  /* Determine which erase block we should allocate the new
   * sector from. This is based on the number of free sectors
   * available in each erase block.
   */

#ifdef CONFIG_MTD_SMART_WEAR_LEVEL
retry:
#endif
  allocfreecount = 0;
  allocblock = 0xffff;
#ifdef CONFIG_MTD_SMART_WEAR_LEVEL
  wornfreecount = 0;
  wornblock = 0xffff;
  wornlevel = 15;
  maxwearlevel = 0;
#endif
  physicalsector = 0xffff;
  if (++dev->lastallocblock >= dev->neraseblocks)
    {
      dev->lastallocblock = 0;
    }

  block = dev->lastallocblock;
  for (i = 0; i < dev->neraseblocks; i++)
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
              /* Keep track of this block.  If there are only worn blocks
               * with free sectors left, then we will use it.
               */

              if (i < dev->neraseblocks - 1 || !wornfreecount)
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

          if (i < dev->neraseblocks - 1 || !allocfreecount)
            {
              allocblock = block;
              allocfreecount = count;
            }
        }

      if (++block >= dev->neraseblocks)
        {
          block = 0;
        }
    }

  /* Check if we found an allocblock. */

  if (allocblock == 0xffff)
    {
      /* No un-worn blocks with free sectors */

#ifdef CONFIG_MTD_SMART_WEAR_LEVEL

      /* If we are allowed to relocate unworn blocks then do so now */

      if (canrelocate && wornfreecount < (dev->sectorsperblk >> 2) &&
          wornlevel == maxwearlevel)
        {
          /* Relocate up to 8 unworn blocks */

          block = 0;
          for (i = 0; i < 8; )
            {
              if (smart_get_wear_level(dev, block) <
                  SMART_WEAR_FORCE_REORG_THRESHOLD)
                {
                  if (smart_relocate_block(dev, block) < 0)
                    {
                      ferr("ERROR: Error relocating block while finding "
                           "free phys sector\n");
                      return -1;
                    }

                  i++;
                }

              block++;
            }

          if (i > 0)
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

      if (wornblock != 0xffff)
        {
          allocblock = wornblock;
        }
      else
#endif

        {
          char buffer[8 * 12 + 1];
          long remaining;
          int j;
          int k;

          ferr("ERROR: Program bug!  Expected a free sector, free=%d\n",
               dev->freesectors);

          for (i = 0, remaining = dev->neraseblocks;
               remaining > 0;
               i += 8, remaining -= 8)
            {
              for (j = 0, k = 0; j < 8 && j < remaining ; j++)
                {
                  k += sprintf(&buffer[k], "%12d", dev->freecount[i + j]);
                }

              ferr("%04x:%s\n", i, buffer);
            }

          /* No free sectors found!  Bug? */

          return -ENOSPC;
        }
    }

  /* Now find a free physical sector within this selected erase block to
   * allocate.
   */

  for (i = allocblock * dev->sectorsperblk;
       i < allocblock * dev->sectorsperblk + dev->availsectperblk; i++)
    {
      /* Check if this physical sector is available. */

#ifdef CONFIG_MTD_SMART_ENABLE_CRC
      /* First check if there is a temporary alloc in place */

      FAR struct smart_allocsector_s *allocsect;
      allocsect = dev->allocsector;

      while (allocsect)
        {
          if (allocsect->physical == i)
            {
              break;
            }

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

      readaddr = i * dev->mtdblkspersector * dev->geo.blocksize;
      ret = MTD_READ(dev->mtd, readaddr, sizeof(struct smart_sect_header_s),
              (FAR uint8_t *) &header);
      if (ret != sizeof(struct smart_sect_header_s))
        {
          ferr("ERROR: Error reading phys sector %d\n", physicalsector);
          return -1;
        }

      if ((*((FAR uint16_t *) header.logicalsector) == 0xffff) &&
#if SMART_STATUS_VERSION == 1
          (*((FAR uint16_t *) &header.seq) == 0xffff) &&
#else
          (header.seq == CONFIG_SMARTFS_ERASEDSTATE) &&
#endif
          ((header.status & SMART_STATUS_COMMITTED) ==
           (CONFIG_SMARTFS_ERASEDSTATE & SMART_STATUS_COMMITTED)))
        {
          physicalsector = i;
          dev->lastallocblock = allocblock;
          break;
        }
      else
        {
          /* The FLASH may be not erased in the initial delivery state.
           * Just in case for the recovery of this fatal situation,
           * after once erasing the sector, return the sector as a free
           * sector.
           */

          if (1 == dev->availsectperblk)
            {
              MTD_ERASE(dev->mtd, allocblock, 1);
              physicalsector = i;
              dev->lastallocblock = allocblock;
              break;
            }
        }
    }

  if (physicalsector == 0xffff)
    {
      ferr("ERROR: Program bug!  Expected a free sector\n");
    }

  if (physicalsector >= dev->totalsectors)
    {
      ferr("ERROR: Program bug!  Selected sector too big!!!\n");
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

static int smart_garbagecollect(FAR struct smart_struct_s *dev)
{
  uint16_t collectblock;
  uint16_t releasemax;
  bool collect = TRUE;
  int x;
  int ret;
#ifdef CONFIG_MTD_SMART_PACK_COUNTS
  uint8_t count;
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

      if (dev->freesectors <= (dev->sectorsperblk << 0) + 4)
        {
          collect = TRUE;
        }

      /* Test if we need to garbage collect */

      if (collect)
        {
          /* Find the block with the most released sectors */

          collectblock = 0xffff;
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

#if 0
          releasemax = smart_get_count(dev, dev->releasecount, collectblock);
#endif

          if (collectblock == 0xffff)
            {
              /* Need to collect, but no sectors with released blocks! */

              ret = -ENOSPC;
              goto errout;
            }

#ifdef CONFIG_SMART_LOCAL_CHECKFREE
          if (smart_checkfree(dev, __LINE__) != OK)
            {
              fwarn("   ...before collecting block %d\n", collectblock);
            }
#endif

#ifdef CONFIG_MTD_SMART_PACK_COUNTS
          finfo("Collecting block %d, free=%d released=%d, "
                "totalfree=%d, totalrelease=%d\n",
                collectblock,
                smart_get_count(dev, dev->freecount, collectblock),
                smart_get_count(dev, dev->releasecount, collectblock),
                dev->freesectors, dev->releasesectors);
#else
          finfo("Collecting block %d, free=%d released=%d\n",
                collectblock, dev->freecount[collectblock],
                dev->releasecount[collectblock]);
#endif

          /* Relocate the active data in the collection block */

          ret = smart_relocate_block(dev, collectblock);

#ifdef CONFIG_SMART_LOCAL_CHECKFREE
          if (smart_checkfree(dev, __LINE__) != OK)
            {
              fwarn("   ...while collecting block %d\n", collectblock);
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
  uint16_t sector;
  uint16_t remaining;
  uint16_t towrite;
  struct smart_read_write_s req;
  int ret;
  uint8_t buffer[8];
  uint8_t write_buffer = 0;

  sector = 0;
  remaining = dev->geo.neraseblocks >> 1;
  memset(buffer, 0xff, sizeof(buffer));

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
   * uneven wearcount (or both)
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
      if (towrite >
          dev->sectorsize - (SMARTFS_FMT_WEAR_POS +
          sizeof(struct smart_sect_header_s)))
        {
          towrite = dev->sectorsize -
                    (SMARTFS_FMT_WEAR_POS +
                     sizeof(struct smart_sect_header_s));
        }

      /* Setup the sector write request (we are our own client) */

      req.logsector = sector;
      req.offset    = SMARTFS_FMT_WEAR_POS;
      req.count     = towrite;
      req.buffer    =
        &dev->wearstatus[(dev->geo.neraseblocks >> SMART_WEAR_BIT_DIVIDE) -
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

              ferr("ERROR: Invalid geometry "
                   "- wear level status too large\n");
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
  struct smart_read_write_s req;
  uint16_t sector;
  uint16_t physsector;
  uint16_t remaining;
  uint16_t toread;
  uint8_t buffer[8];
  int ret;

  /* Prepare to read the total block erases and uneven wearcount values */

  sector        = 0;
  req.logsector = sector;
  req.offset    = SMARTFS_FMT_WEAR_POS - 8;
  req.count     = sizeof(buffer);
  req.buffer    = buffer;

  ret = smart_readsector(dev, (unsigned long) &req);
  if (ret != sizeof(buffer))
    {
      goto errout;
    }

  /* Get the uneven wearcount value */

  dev->uneven_wearcount = *((uint32_t *) &buffer[4]);

  /* Check for erased state */

#if ( CONFIG_SMARTFS_ERASEDSTATE == 0xff )
  if (dev->uneven_wearcount == 0xffffffff)
    {
      dev->uneven_wearcount = 0;
    }
#endif

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_SMARTFS)
  /* Get the block erases count */

  dev->blockerases = *((uint32_t *) buffer);
#if ( CONFIG_SMARTFS_ERASEDSTATE == 0xff )
  if (dev->blockerases == 0xffffffff)
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
      if (toread > dev->sectorsize -
          (SMARTFS_FMT_WEAR_POS + sizeof(struct smart_sect_header_s)))
        {
          toread = dev->sectorsize -
                   (SMARTFS_FMT_WEAR_POS +
                    sizeof(struct smart_sect_header_s));
        }

      /* Setup the sector read request (we are our own client) */

      req.logsector = sector;
      req.offset    = SMARTFS_FMT_WEAR_POS;
      req.count     = toread;
      req.buffer    =
        &dev->wearstatus[(dev->geo.neraseblocks >> SMART_WEAR_BIT_DIVIDE) -
                         remaining];

      /* Validate wear status sector has been allocated */

#ifndef CONFIG_MTD_SMART_MINIMIZE_RAM
      physsector = dev->smap[req.logsector];
#else
      physsector = smart_cache_lookup(dev, req.logsector);
#endif
      if ((sector != 0) && (physsector == 0xffff))
        {
          /* This logical sector does not exist yet.  We must allocate it */

          ret = smart_allocsector(dev, sector);
          if (ret != sector)
            {
              ferr("ERROR: Unable to allocate wear level status sector %d\n",
                   sector);
              ret = -EINVAL;
              goto errout;
            }
        }

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

              ferr("ERROR: Invalid geometry "
                   "- wear level status too large\n");
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

static int smart_write_alloc_sector(FAR struct smart_struct_s *dev,
                    uint16_t logical, uint16_t physical)
{
  int ret = 1;
  uint8_t sectsize;
  FAR struct smart_sect_header_s *header;

  memset(dev->rwbuffer, CONFIG_SMARTFS_ERASEDSTATE, dev->sectorsize);
  header = (FAR struct smart_sect_header_s *) dev->rwbuffer;
  *((FAR uint16_t *) header->logicalsector) = logical;
#if SMART_STATUS_VERSION == 1
#ifdef CONFIG_MTD_SMART_ENABLE_CRC
  header->seq = 0;
#else
  *((FAR uint16_t *) &header->seq) = 0;
#endif /* CONFIG_MTD_SMART_ENABLE_CRC */
#else
  header->seq = 0;
#endif

  /* Calculate the 3-bit logical sector size in bits 2-4:
   * 000b - 256 bytes
   * 001b - 512 bytes
   * 010b - 1024 bytes
   * 100b - 2048 bytes
   * 011b - 4096 bytes
   * 101b - 8192 bytes
   * 110b - 16384 bytes
   * 110b - 32768 bytes
   */

  sectsize = dev->sectorsize < 4096  ? (dev->sectorsize >> 9) :
             dev->sectorsize == 4096 ? 3 : 5 + (dev->sectorsize >> 14);
  sectsize <<= 2;

#if CONFIG_SMARTFS_ERASEDSTATE == 0xff
  header->status = ~(SMART_STATUS_COMMITTED | SMART_STATUS_SIZEBITS |
          SMART_STATUS_VERBITS) | SMART_STATUS_VERSION | sectsize;
#ifdef CONFIG_MTD_SMART_ENABLE_CRC
  header->status &= ~SMART_STATUS_CRC;
#endif /* CONFIG_MTD_SMART_ENABLE_CRC */
#else
  header->status = SMART_STATUS_COMMITTED | SMART_STATUS_VERSION | sectsize;
#ifdef CONFIG_MTD_SMART_ENABLE_CRC
  header->status |= SMART_STATUS_CRC;
#endif /* CONFIG_MTD_SMART_ENABLE_CRC */
#endif

  /* Write the header to the physical sector location */

#ifndef CONFIG_MTD_SMART_ENABLE_CRC
  finfo("Write MTD block %d\n", physical * dev->mtdblkspersector);
  ret = MTD_BWRITE(dev->mtd, physical * dev->mtdblkspersector, 1,
      (FAR uint8_t *) dev->rwbuffer);
  if (ret != 1)
    {
      /* The block is not empty!!  What to do? */

      ferr("ERROR: Write block %d failed: %d.\n", physical *
           dev->mtdblkspersector, ret);

      /* Unlock the mutex if we add one */

      return -EIO;
    }
#endif /* CONFIG_MTD_SMART_ENABLE_CRC */

  return ret;
}

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
  crc_t crc;
  FAR struct smart_sect_header_s *header;

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

static int smart_writesector(FAR struct smart_struct_s *dev,
                    unsigned long arg)
{
  int ret;
  bool needsrelocate = FALSE;
  uint32_t mtdblock;
  uint16_t physsector;
  uint16_t oldphyssector;
  uint16_t block;
  FAR struct smart_read_write_s *req;
  FAR struct smart_sect_header_s *header;
  size_t offset;
  uint8_t byte;
#if defined(CONFIG_MTD_SMART_WEAR_LEVEL) || !defined(CONFIG_MTD_SMART_ENABLE_CRC)
  uint16_t x;
#endif
#ifdef CONFIG_MTD_SMART_ENABLE_CRC
  FAR struct smart_allocsector_s *allocsector;
#endif

  finfo("Entry\n");
  req = (FAR struct smart_read_write_s *) arg;
  DEBUGASSERT(req->offset <= dev->sectorsize);
  DEBUGASSERT(req->offset + req->count <= dev->sectorsize);

  /* Ensure the logical sector has been allocated */

  if (req->logsector >= dev->totalsectors)
    {
      ferr("ERROR: Logical sector %d too large\n", req->logsector);

      ret = -EINVAL;
      goto errout;
    }

  header = (FAR struct smart_sect_header_s *)dev->rwbuffer;

#ifdef CONFIG_MTD_SMART_WEAR_LEVEL
  /* Test if an adjustment to the wear levels is needed */

  if (dev->minwearlevel >= SMART_WEAR_MIN_LEVEL ||
      (dev->minwearlevel > 0 &&
       dev->maxwearlevel >= SMART_WEAR_REORG_THRESHOLD))
    {
      /* Subtract dev->minwearlevel from all wear levels */

      offset = dev->minwearlevel;
      finfo("Reducing wear level bits by %zu\n", offset);

      for (x = 0; x < dev->geo.neraseblocks; x++)
        {
          smart_set_wear_level(dev, x,
                               smart_get_wear_level(dev, x) - offset);
        }

      dev->minwearlevel -= offset;
      dev->maxwearlevel -= offset;

      /* Now write the new wear bits to the flash */

      dev->wearflags &= ~SMART_WEARFLAGS_FORCE_REORG;
      dev->wearflags |= SMART_WEARFLAGS_WRITE_NEEDED;
    }
#endif

#ifndef CONFIG_MTD_SMART_MINIMIZE_RAM
  physsector = dev->smap[req->logsector];
#else
  physsector = smart_cache_lookup(dev, req->logsector);
#endif
  if (physsector == 0xffff)
    {
      ferr("ERROR: Logical sector %d not allocated\n", req->logsector);
      ret = -EINVAL;
      goto errout;
    }

  /* Read the sector data into our buffer */

  mtdblock = physsector * dev->mtdblkspersector;
  ret = MTD_BREAD(dev->mtd, mtdblock, dev->mtdblkspersector, (FAR uint8_t *)
          dev->rwbuffer);
  if (ret != dev->mtdblkspersector)
    {
      ferr("ERROR: Error reading phys sector %d\n", physsector);
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

      byte = dev->rwbuffer[sizeof(struct smart_sect_header_s) +
                           req->offset + x];
#if CONFIG_SMARTFS_ERASEDSTATE == 0xff
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
#endif /* CONFIG_MTD_SMART_ENABLE_CRC */

  /* If we are not using CRC and on a device that supports re-writing
   * bits from 1 to 0 without needing a block erase, such as NOR
   * FLASH, then we can simply update the data in place and don't need
   * to relocate the sector.  Test if we need to relocate or not.
   */

  if (needsrelocate)
    {
      /* Find a new physical sector to save data to */

      oldphyssector = physsector;
      physsector = smart_findfreephyssector(dev, FALSE);
      if (physsector == 0xffff)
        {
          ferr("ERROR: Error relocating sector %d\n", req->logsector);
          ret = -EIO;
          goto errout;
        }

      /* Update the sequence number to indicate the sector was moved */

#if SMART_STATUS_VERSION == 1
      if ((header->status & SMART_STATUS_CRC) !=
              (CONFIG_SMARTFS_ERASEDSTATE & SMART_STATUS_CRC))
        {
#endif
          header->seq++;
          if (header->seq == 0xff)
            {
              header->seq = 0;
            }
#if SMART_STATUS_VERSION == 1
        }
      else
        {
          (*((FAR uint16_t *) &header->seq))++;
          if (*((FAR uint16_t *) &header->seq) == 0xffff)
            *((FAR uint16_t *) &header->seq) = 1;
        }
#else
      header->seq++;
#endif
#if CONFIG_SMARTFS_ERASEDSTATE == 0xff
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
      smart_write_alloc_sector(dev, allocsector->logical,
                               allocsector->physical);

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

#if CONFIG_SMARTFS_ERASEDSTATE == 0xff
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

#endif /* CONFIG_MTD_SMART_ENABLE_CRC */

  /* Now write the sector buffer to the device. */

  if (needsrelocate)
    {
      /* Write the entire sector to the new physical location, uncommitted. */

      ret = MTD_BWRITE(dev->mtd, physsector * dev->mtdblkspersector,
              dev->mtdblkspersector, (FAR uint8_t *) dev->rwbuffer);
      if (ret != dev->mtdblkspersector)
        {
          ferr("ERROR: Error writing to physical sector %d\n", physsector);
          ret = -EIO;
          goto errout;
        }

      /* Commit the new physical sector */

#ifndef CONFIG_MTD_SMART_ENABLE_CRC

#if CONFIG_SMARTFS_ERASEDSTATE == 0xff
      byte = header->status & ~SMART_STATUS_COMMITTED;
#else
      byte = header->status | SMART_STATUS_COMMITTED;
#endif
      offset = physsector * dev->mtdblkspersector * dev->geo.blocksize +
          offsetof(struct smart_sect_header_s, status);
      ret = smart_bytewrite(dev, offset, 1, &byte);
      if (ret != 1)
        {
          finfo("Error committing physical sector %d\n", physsector);
          ret = -EIO;
          goto errout;
        }
#endif

      /* Release the old physical sector */

#if CONFIG_SMARTFS_ERASEDSTATE == 0xff
      byte = header->status & ~(SMART_STATUS_RELEASED |
                                SMART_STATUS_COMMITTED);
#else
      byte = header->status | SMART_STATUS_RELEASED |
             SMART_STATUS_COMMITTED;
#endif
      offset = mtdblock * dev->geo.blocksize +
          offsetof(struct smart_sect_header_s, status);
      ret = smart_bytewrite(dev, offset, 1, &byte);

      /* Update releasecount for the released sector and freecount for the
       * newly allocated physical sector.
       */

      block = oldphyssector / dev->sectorsperblk;
#ifdef CONFIG_MTD_SMART_PACK_COUNTS
      smart_add_count(dev, dev->releasecount, block, 1);
      smart_add_count(dev, dev->freecount, physsector / dev->sectorsperblk,
                      -1);
#else
      dev->releasecount[block]++;
      dev->freecount[physsector / dev->sectorsperblk]--;
#endif
      dev->freesectors--;
      dev->releasesectors++;

#ifdef CONFIG_SMART_LOCAL_CHECKFREE
      /* Perform debug free count checking enabled */

      smart_checkfree(dev, __LINE__);
#endif

      /* Update the sector map */

#ifndef CONFIG_MTD_SMART_MINIMIZE_RAM
      dev->smap[req->logsector] = physsector;
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

      ret = MTD_BWRITE(dev->mtd, physsector * dev->mtdblkspersector,
              dev->mtdblkspersector, (FAR uint8_t *) dev->rwbuffer);
      if (ret != dev->mtdblkspersector)
        {
          ferr("ERROR: Error writing to physical sector %d\n", physsector);
          ret = -EIO;
          goto errout;
        }

      /* Read the sector back and validate the CRC. */

      ret = MTD_BREAD(dev->mtd, physsector * dev->mtdblkspersector,
              dev->mtdblkspersector, (FAR uint8_t *) dev->rwbuffer);
      if (ret == dev->mtdblkspersector)
        {
          /* Validate the CRC of the read-back data */

          ret = smart_validate_crc(dev);
        }

      if (ret != OK)
        {
          /* TODO: Mark this as a bad block! */

          ferr("ERROR: Error validating physical sector %d\n", physsector);
          ret = -EIO;
          goto errout;
        }
#else
      /* Not relocated.  Just write the portion of the sector that needs
       * to be written.
       */

      offset = mtdblock * dev->geo.blocksize +
          sizeof(struct smart_sect_header_s) + req->offset;
      ret = smart_bytewrite(dev, offset, req->count, req->buffer);
#endif
    }

  ret = OK;

errout:
  return ret;
}

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
  int ret;
  uint16_t physsector;
  FAR struct smart_read_write_s *req;
#ifdef CONFIG_MTD_SMART_ENABLE_CRC
#if SMART_STATUS_VERSION == 1
  FAR struct smart_sect_header_s *header;
#endif
#else
  uint32_t readaddr;
  struct smart_sect_header_s header;
#endif

  finfo("Entry\n");

  req = (FAR struct smart_read_write_s *) arg;
  DEBUGASSERT(req->offset < dev->sectorsize);
  DEBUGASSERT(req->offset + req->count + sizeof(struct smart_sect_header_s)
              <= dev->sectorsize);

  /* Ensure the logical sector has been allocated */

  if (req->logsector >= dev->totalsectors)
    {
      ferr("ERROR: Logical sector %d too large\n", req->logsector);

      return -EINVAL;
    }

#ifndef CONFIG_MTD_SMART_MINIMIZE_RAM
  physsector = dev->smap[req->logsector];
#else
  physsector = smart_cache_lookup(dev, req->logsector);
#endif
  if (physsector == 0xffff)
    {
      ferr("ERROR: Logical sector %d not allocated\n", req->logsector);
      return -EINVAL;
    }

#ifdef CONFIG_MTD_SMART_ENABLE_CRC

  /* When CRC is enabled, we read the entire sector into RAM so we can
   * validate the CRC.
   */

  ret = MTD_BREAD(dev->mtd, physsector * dev->mtdblkspersector,
                  dev->mtdblkspersector, (FAR uint8_t *) dev->rwbuffer);
  if (ret != dev->mtdblkspersector)
    {
      /* TODO:  Mark the block bad */

      ferr("ERROR: Error reading phys sector %d\n", physsector);
      return -EIO;
    }

#if SMART_STATUS_VERSION == 1
  /* Test if this sector has CRC enabled or not */

  header = (FAR struct smart_sect_header_s *) dev->rwbuffer;
  if ((header->status & SMART_STATUS_CRC) ==
      (CONFIG_SMARTFS_ERASEDSTATE & SMART_STATUS_CRC))
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

          ferr("ERROR: Error validating sector %d CRC during read\n",
               physsector);
          return -EIO;
        }
    }

  /* Copy data to the output buffer */

  memmove((FAR char *) req->buffer, &dev->rwbuffer[req->offset +
      sizeof(struct smart_sect_header_s)], req->count);
  ret = req->count;

#else /* CONFIG_MTD_SMART_ENABLE_CRC */

  /* Read the sector header data to validate as a sanity check */

  ret = MTD_READ(dev->mtd, physsector * dev->mtdblkspersector *
                 dev->geo.blocksize, sizeof(struct smart_sect_header_s),
                 (FAR uint8_t *)&header);
  if (ret != sizeof(struct smart_sect_header_s))
    {
      ferr("ERROR: Error reading sector %d header\n", physsector);
      return -EIO;
    }

  /* Do a sanity check on the header data */

  if (((*(FAR uint16_t *) header.logicalsector) != req->logsector) ||
      ((header.status & SMART_STATUS_COMMITTED) ==
       (CONFIG_SMARTFS_ERASEDSTATE & SMART_STATUS_COMMITTED)))
    {
      /* Error in sector header! How do we handle this? */

      ferr("ERROR: Error in logical sector %d header, phys=%d\n",
          req->logsector, physsector);
      return -EIO;
    }

  /* Read the sector data into the buffer */

  readaddr = (uint32_t)physsector * dev->mtdblkspersector *
             dev->geo.blocksize + req->offset +
             sizeof(struct smart_sect_header_s);

  ret = MTD_READ(dev->mtd, readaddr, req->count, (FAR uint8_t *)
          req->buffer);
  if (ret != req->count)
    {
      ferr("ERROR: Error reading phys sector %d\n", physsector);
      return -EIO;
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: smart_allocsector
 *
 * Description:  Allocates a new logical sector.  If an argument is given,
 *               then it tries to allocate the specified sector number.
 *
 ****************************************************************************/

static inline int smart_allocsector(FAR struct smart_struct_s *dev,
                    unsigned long requested)
{
  uint16_t logsector = 0xffff; /* Logical sector number selected */
  uint16_t physicalsector;     /* The selected physical sector */
#ifndef CONFIG_MTD_SMART_ENABLE_CRC
  int ret;
#endif
  int x;

  /* Validate that we have enough sectors available to perform an
   * allocation.  We have to ensure we keep enough reserved sectors
   * on hand to do released sector garbage collection.
   */

  if (dev->freesectors <= (dev->sectorsperblk << 0) + 4)
    {
      /* Do a garbage collect and then test freesectors again */

      if (dev->releasesectors + dev->freesectors > dev->sectorsperblk + 4)
        {
          for (x = 0; x < dev->availsectperblk; x++)
            {
              smart_garbagecollect(dev);

              if (dev->freesectors > dev->availsectperblk + 4)
                {
                  break;
                }
            }

          if (dev->freesectors <= (dev->availsectperblk << 0) + 4)
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
   * sector if it isn't already in use.
   */

  if ((requested > 0) && (requested < dev->totalsectors))
    {
      /* Validate the sector is not already allocated */

#ifndef CONFIG_MTD_SMART_MINIMIZE_RAM
      if (dev->smap[requested] == (uint16_t) -1)
#else
      if (!(dev->sbitmap[requested >> 3] & (1 << (requested & 0x07))))
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

  if (logsector == 0xffff)
    {
      /* Loop through all sectors and find one to allocate */

      for (x = SMART_FIRST_ALLOC_SECTOR; x < dev->totalsectors; x++)
        {
#ifndef CONFIG_MTD_SMART_MINIMIZE_RAM
          if (dev->smap[x] == (uint16_t) -1)
#else
          if (!(dev->sbitmap[x >> 3] & (1 << (x & 0x07))))
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

  if (logsector == 0xffff)
    {
      /* Hmmm.  We think we had enough logical sectors, but
       * something happened and we didn't find any free
       * logical sectors.  What do do?  Report an error?
       * rescan and try again to "self heal" in case of a
       * bug in our code?
       */

      ferr("ERROR: No free logical sector numbers!  Free sectors = %d\n",
              dev->freesectors);

      return -EIO;
    }

  /* Check if we need to do garbage collection.  We have to
   * ensure we keep enough reserved free sectors to perform garbage
   * collection as it involves moving sectors from blocks with
   * released sectors into blocks with free sectors, then
   * erasing the vacated block.
   */

  smart_garbagecollect(dev);

  /* Find a free physical sector */

  physicalsector = smart_findfreephyssector(dev, FALSE);
  finfo("Alloc: log=%d, phys=%d, erase block=%d, free=%d, released=%d\n",
          logsector, physicalsector, physicalsector /
          dev->sectorsperblk, dev->freesectors, dev->releasesectors);

  if (physicalsector == 0xffff)
    {
      return -ENOSPC;
    }

#ifdef CONFIG_MTD_SMART_ENABLE_CRC

  /* When CRC is enabled, we don't write the header to the device until
   * the data is written via writesector.  Just add the allocation to
   * our temporary allocsector list and we'll pick it up later.
   */

    {
      FAR struct smart_allocsector_s *allocsect =
        (FAR struct smart_allocsector_s *)
        kmm_malloc(sizeof(struct smart_allocsector_s));

      if (allocsect == NULL)
        {
          ferr("ERROR: Out of memory allocting sector\n");
          return -ENOMEM;
        }

      /* Fill in the struct and add to the list.  We are protected by the
       * smartfs layer's mutex, so no locking is required.
       */

      allocsect->logical = logsector;
      allocsect->physical = physicalsector;
      allocsect->next = dev->allocsector;
      dev->allocsector = allocsect;
    }

#else /* CONFIG_MTD_SMART_ENABLE_CRC */

  /* Write the logical sector to the flash.  We will fill it in with data
   * later.
   */

  ret = smart_write_alloc_sector(dev, logsector, physicalsector);
  if (ret != 1)
    {
      /* Error writing sector, return error */

      return ret;
    }
#endif /* CONFIG_MTD_SMART_ENABLE_CRC */

  /* Map the sector and update the free sector counts */

#ifndef CONFIG_MTD_SMART_MINIMIZE_RAM
  dev->smap[logsector] = physicalsector;
#else
  dev->sbitmap[logsector >> 3] |= (1 << (logsector & 0x07));
  smart_add_sector_to_cache(dev, logsector, physicalsector, __LINE__);
#endif

#ifdef CONFIG_MTD_SMART_PACK_COUNTS
  smart_add_count(dev, dev->freecount,
                  physicalsector / dev->sectorsperblk, -1);
#else
  dev->freecount[physicalsector / dev->sectorsperblk]--;
#endif
  dev->freesectors--;

  /* Return the logical sector number */

  return logsector;
}

/****************************************************************************
 * Name: smart_freesector
 *
 * Description:  Frees a logical sector from the device.  Freeing (also
 *               called releasing) is performed by programming the released
 *               bit in the sector header's status byte.
 *
 ****************************************************************************/

static inline int smart_freesector(FAR struct smart_struct_s *dev,
                    unsigned long logicalsector)
{
  int ret;
  int readaddr;
  uint16_t physsector;
  uint16_t block;
  struct smart_sect_header_s header;
  size_t offset;

  /* Check if the logical sector is within bounds */

  if ((logicalsector > 2) && (logicalsector < dev->totalsectors))
    {
      /* Validate the sector is actually allocated */

#ifndef CONFIG_MTD_SMART_MINIMIZE_RAM
      if (dev->smap[logicalsector] == (uint16_t) -1)
#else
      if (!(dev->sbitmap[logicalsector >> 3] &
            (1 << (logicalsector & 0x07))))
#endif
        {
          ferr("ERROR: Invalid release - sector %ld not allocated\n",
               logicalsector);
          ret = -EINVAL;
          goto errout;
        }
    }

  /* Okay to release the sector.  Read the sector header info */

#ifndef CONFIG_MTD_SMART_MINIMIZE_RAM
  physsector = dev->smap[logicalsector];
#else
  physsector = smart_cache_lookup(dev, logicalsector);
#endif
  readaddr = physsector * dev->mtdblkspersector * dev->geo.blocksize;
  ret = MTD_READ(dev->mtd, readaddr, sizeof(struct smart_sect_header_s),
                 (FAR uint8_t *) &header);
  if (ret != sizeof(struct smart_sect_header_s))
    {
      goto errout;
    }

  /* Do a sanity check on the logical sector number */

  if (*((FAR uint16_t *) header.logicalsector) != (uint16_t) logicalsector)
    {
      /* Hmmm... something is wrong.  This should always match!  Bug in our
       * code?
       */

      ferr("ERROR: Sector %ld logical sector in header doesn't match\n",
           logicalsector);
      ret = -EINVAL;
      goto errout;
    }

  /* Mark the sector as released */

#if CONFIG_SMARTFS_ERASEDSTATE == 0xff
  header.status &= ~SMART_STATUS_RELEASED;
#else
  header.status |= SMART_STATUS_RELEASED;
#endif

  /* Write the status back to the device */

  offset = readaddr + offsetof(struct smart_sect_header_s, status);
  ret = smart_bytewrite(dev, offset, 1, &header.status);
  if (ret != 1)
    {
      ferr("ERROR: Error updating physical sector %d status\n", physsector);
      goto errout;
    }

  /* Update the erase block's release count */

  dev->releasesectors++;
  block = physsector / dev->sectorsperblk;
#ifdef CONFIG_MTD_SMART_PACK_COUNTS
  smart_add_count(dev, dev->releasecount, block, 1);
#else
  dev->releasecount[block]++;
#endif

  /* Unmap this logical sector */

#ifndef CONFIG_MTD_SMART_MINIMIZE_RAM
  dev->smap[logicalsector] = (uint16_t) -1;
#else
  dev->sbitmap[logicalsector >> 3] &= ~(1 << (logicalsector & 0x07));
  smart_update_cache(dev, logicalsector, 0xffff);
#endif

  /* If this block has only released blocks, then erase it */

  smart_erase_block_if_empty(dev, block, FALSE);
  ret = OK;

errout:
  return ret;
}

/****************************************************************************
 * Name: smart_ioctl
 *
 * Description: Return device geometry
 *
 ****************************************************************************/

static int smart_ioctl(FAR struct inode *inode, int cmd, unsigned long arg)
{
  FAR struct smart_struct_s *dev;
  int ret;
#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_SMARTFS)
  FAR struct mtd_smart_procfs_data_s *procfs_data;
  FAR struct mtd_smart_debug_data_s *debug_data;
#endif

  finfo("Entry\n");
  DEBUGASSERT(inode && inode->i_private);

#ifdef CONFIG_SMARTFS_MULTI_ROOT_DIRS
  dev = ((FAR struct smart_multiroot_device_s *)inode->i_private)->dev;
#else
  dev = (FAR struct smart_struct_s *)inode->i_private;
#endif

  /* Process the ioctl's we care about first, pass any we don't respond
   * to directly to the underlying MTD device.
   */

  switch (cmd)
    {
    case BIOC_GETFORMAT:

      /* Return the format information for the device */

#ifdef CONFIG_SMARTFS_MULTI_ROOT_DIRS
      ret = smart_getformat(dev, (FAR struct smart_format_s *) arg,
                            ((FAR struct smart_multiroot_device_s *)
                            inode->i_private)->rootdirnum);
#else
      ret = smart_getformat(dev, (FAR struct smart_format_s *) arg);
#endif
      goto ok_out;

    case BIOC_READSECT:

      /* Do a logical sector read and return the data */

      ret = smart_readsector(dev, arg);
      goto ok_out;

    case BIOC_LLFORMAT:

      /* Perform a low-level format on the flash */

      ret = smart_llformat(dev, arg);
      goto ok_out;

    case BIOC_ALLOCSECT:

      /* Ensure the FS is not trying to allocate a reserved sector */

      if (arg < 3)
        {
          arg = (unsigned long) -1;
        }

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

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_SMARTFS)
    case BIOC_GETPROCFSD:

      /* Get ProcFS data */

      procfs_data = (FAR struct mtd_smart_procfs_data_s *)arg;
      procfs_data->totalsectors   = dev->totalsectors;
      procfs_data->sectorsize     = dev->sectorsize;
      procfs_data->freesectors    = dev->freesectors;
      procfs_data->releasesectors = dev->releasesectors;
      procfs_data->namelen        = dev->namesize;
      procfs_data->formatversion  = dev->formatversion;
      procfs_data->unusedsectors  = dev->unusedsectors;
      procfs_data->blockerases    = dev->blockerases;
      procfs_data->sectorsperblk  = dev->sectorsperblk;

#ifndef CONFIG_MTD_SMART_MINIMIZE_RAM
      procfs_data->formatsector   = dev->smap[0];
      procfs_data->dirsector      = dev->smap[3];
#else
      procfs_data->formatsector   = smart_cache_lookup(dev, 0);
      procfs_data->dirsector      = smart_cache_lookup(dev, 3);
#endif

#ifdef CONFIG_MTD_SMART_SECTOR_ERASE_DEBUG
      procfs_data->neraseblocks   = dev->geo.neraseblocks;
      procfs_data->erasecounts    = dev->erasecounts;
#endif
#ifdef CONFIG_MTD_SMART_ALLOC_DEBUG
      procfs_data->allocs         = dev->alloc;
      procfs_data->alloccount     = SMART_MAX_ALLOCS;
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
          finfo("Debug level set to %d\n", dev->debuglevel);

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
      ferr("ERROR: MTD ioctl(%04x) failed: %d\n", cmd, ret);
    }

ok_out:
  return ret;
}

#ifdef CONFIG_MTD_SMART_FSCK

/****************************************************************************
 * Name: smart_fsck_crc
 *
 * Description: Validate CRC to check smartfs filesystem
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_SMART_FSCK_ENABLE_CRC
static int smart_fsck_crc(FAR struct smart_struct_s *dev,
                          uint16_t physsector)
{
  int ret;

  ret = MTD_BREAD(dev->mtd, physsector * dev->mtdblkspersector,
                  dev->mtdblkspersector, (FAR uint8_t *)dev->rwbuffer);
  if (ret != dev->mtdblkspersector)
    {
      ferr("ERROR: Error reading phys sector %d\n", physsector);
      return ret;
    }

  ret = smart_validate_crc(dev);
  if (ret != OK)
    {
      ferr("ERROR: Error validating sector %d CRC\n", physsector);
      return ret;
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: smart_fsck_file
 *
 * Description: fsck for file entry
 *
 ****************************************************************************/

static int smart_fsck_file(FAR struct smart_struct_s *dev,
                           FAR uint8_t *checkmap, uint16_t logsector)
{
  int ret = OK;
  ssize_t size;
  uint32_t readaddress;
  FAR struct smart_sect_header_s *header;
  FAR struct smart_chain_header_s *chain;
  FAR uint8_t *usedmap;
  size_t mapsize;
  uint16_t physsector;
  int i;

  if (logsector >= dev->totalsectors)
    {
      ret = -EINVAL;
      return ret;
    }

  /* Allocate a bitmap table for sectors this file is using */

  mapsize = (dev->totalsectors + 7) / 8;
  usedmap = (FAR uint8_t *)kmm_zalloc(mapsize);
  if (!usedmap)
    {
      ferr("ERROR: Out of memory used map\n");
      return OK;
    }

  do
    {
      /* Read the header for file sector */

#ifndef CONFIG_MTD_SMART_MINIMIZE_RAM
      physsector = dev->smap[logsector];
#else
      physsector = smart_cache_lookup(dev, logsector);
#endif

      if (physsector >= dev->totalsectors)
        {
          ret = -ENXIO;
          ferr("ERROR: Invalid phys sector %d\n", physsector);
          break;
        }

#ifdef CONFIG_MTD_SMART_FSCK_ENABLE_CRC
      if (smart_fsck_crc(dev, physsector) != OK)
        {
          ret = -ENOENT;
          ferr("ERROR: CRC phys sector %d\n", physsector);
          break;
        }
#endif

      readaddress = physsector * dev->mtdblkspersector * dev->geo.blocksize;
      size = MTD_READ(dev->mtd, readaddress,
                      sizeof(struct smart_sect_header_s) +
                      sizeof(struct smart_chain_header_s),
                      (uint8_t *)dev->rwbuffer);
      if (size != (sizeof(struct smart_sect_header_s) +
                   sizeof(struct smart_chain_header_s)))
        {
          ret = -EIO;
          ferr("Error reading phys sector %d\n", physsector);
          break;
        }

      header = (struct smart_sect_header_s *) & dev->rwbuffer[0];
      chain = (struct smart_chain_header_s *) &
               dev->rwbuffer[sizeof(struct smart_sect_header_s)];

      /* Test if the sector has live data (not free or not released) */

      if (((header->status & SMART_STATUS_COMMITTED) ==
           (CONFIG_SMARTFS_ERASEDSTATE & SMART_STATUS_COMMITTED)) ||
          ((header->status & SMART_STATUS_RELEASED) !=
           (CONFIG_SMARTFS_ERASEDSTATE & SMART_STATUS_RELEASED)))
        {
          ret = -ENOENT;
          ferr("ERROR: status(%02x) phys sector %d\n",
               header->status, physsector);
          break;
        }

      SET_BITMAP(usedmap, logsector);

      /* next logical sector */

      logsector = SMARTFS_NEXTSECTOR(chain);
    }
  while (logsector != 0xffff);

  if (ret == OK)
    {
      /* These sectors in use are not removed */

      for (i = 0; i < mapsize; i++)
        {
          checkmap[i] &= ~usedmap[i];
        }
    }
  else
    {
      /* This file has any corruption, these sectors will be removed */

      for (i = 0; i < mapsize; i++)
        {
          checkmap[i] |= usedmap[i];
        }
    }

  kmm_free(usedmap);
  return ret;
}

/****************************************************************************
 * Name: smart_fsck_directory
 *
 * Description: fsck for directory entry
 *
 ****************************************************************************/

static int smart_fsck_directory(FAR struct smart_struct_s *dev,
                                FAR uint8_t *checkmap, uint16_t logsector)
{
  int ret = OK;
  int relocate = 0;
  ssize_t size;
  FAR uint8_t *rwbuffer;
  FAR struct smart_sect_header_s *header;
  FAR struct smart_chain_header_s *chain;
  FAR struct smart_entry_header_s *entry;
  uint16_t entrysector;
  uint16_t physsector;
  uint16_t nextsector;
  uint16_t newsector;
  int entrysize;
  FAR uint8_t *bottom;
  FAR uint8_t *cur;
#ifdef CONFIG_DEBUG_FS_INFO
  char entryname[dev->namesize + 1];
#endif

  if ((logsector < SMART_FIRST_DIR_SECTOR) ||
      (logsector >= dev->totalsectors))
    {
      ret = -EINVAL;
      ferr("ERROR: Invalid log sector %d\n", logsector);
      return ret;
    }

  /* Allocate sector buffer for Directory entry */

  rwbuffer = (uint8_t *)kmm_malloc(dev->sectorsize);
  if (!rwbuffer)
    {
      ferr("ERROR: Out of memory sector buffer\n");
      return OK;
    }

  /* Read the Directory entry sector */

#ifndef CONFIG_MTD_SMART_MINIMIZE_RAM
  physsector = dev->smap[logsector];
#else
  physsector = smart_cache_lookup(dev, logsector);
#endif
  if (physsector >= dev->totalsectors)
    {
      ret = -ENXIO;
      ferr("ERROR: Invalid phys sector %d\n", physsector);
      goto errout;
    }

  size = MTD_BREAD(dev->mtd, physsector * dev->mtdblkspersector,
                   dev->mtdblkspersector, rwbuffer);
  if (size != dev->mtdblkspersector)
    {
      ret = -EIO;
      ferr("ERROR: reading phys sector %d\n", physsector);
      goto errout;
    }

  header = (struct smart_sect_header_s *) & rwbuffer[0];
  chain = (struct smart_chain_header_s *) &
           rwbuffer[sizeof(struct smart_sect_header_s)];
  entry = (struct smart_entry_header_s *) &
           rwbuffer[sizeof(struct smart_sect_header_s) +
                    sizeof(struct smart_chain_header_s)];

#ifdef CONFIG_MTD_SMART_FSCK_ENABLE_CRC
  /* Check CRC */

  if (smart_fsck_crc(dev, physsector) != OK)
    {
      ret = -ENOENT;
      ferr("ERROR: CRC phys sector %d\n", physsector);
      goto errout;
    }
#endif

  /* Test if the sector has live data (not free or not released) */

  if (((header->status & SMART_STATUS_COMMITTED) ==
       (CONFIG_SMARTFS_ERASEDSTATE & SMART_STATUS_COMMITTED)) ||
      ((header->status & SMART_STATUS_RELEASED) !=
       (CONFIG_SMARTFS_ERASEDSTATE & SMART_STATUS_RELEASED)))
    {
      ret = -ENOENT;
      ferr("ERROR: status(%02x) phys sector %d\n",
           header->status, physsector);
      goto errout;
    }

  /* Check next sector recursively */

  nextsector = SMARTFS_NEXTSECTOR(chain);

  if (nextsector != 0xffff)
    {
      finfo("Check next log sector %d\n", nextsector);

      ret = smart_fsck_directory(dev, checkmap, nextsector);

      if (ret != OK)
        {
          /* Invalidate the next sector */

          ferr("Invalidate next log sector %d\n", nextsector);

          SMARTFS_SET_NEXTSECTOR(chain, 0xffff);

          /* Set flag to relocate later */

          relocate = 1;
        }
    }

#define SMARTFS_DIRENT_EMPTY      0x8000  /* Set to non-erase state when entry used */
#define SMARTFS_DIRENT_ACTIVE     0x4000  /* Set to erase state when entry is active */
#define SMARTFS_DIRENT_TYPE       0x2000  /* Indicates the type of entry (file/dir) */
#define SMARTFS_DIRENT_DELETING   0x1000  /* Directory entry is being deleted */
#define SMARTFS_DIRENT_RESERVED   0x0E00  /* Reserved bits */

  /* Check file or directory under this directory entry */

  entrysize = sizeof(struct smart_entry_header_s) + dev->namesize;
  bottom = rwbuffer + dev->sectorsize;
  cur = &rwbuffer[sizeof(struct smart_sect_header_s) +
                  sizeof(struct smart_chain_header_s)];

  while ((cur + entrysize) <= bottom)
    {
      ret = OK;

      entry = (struct smart_entry_header_s *)cur;

      if (entry->flags == 0xffff)
        {
          /* Test if the empty entry is exist or not? */

          break;
        }

#ifdef CONFIG_DEBUG_FS_INFO
      strlcpy(entryname,
              (const char *) (cur + sizeof(struct smart_entry_header_s)),
              sizeof(entryname));
      finfo("Check entry (name=%s flags=%02x logsector=%02x)\n",
            entryname, entry->flags, entry->firstsector);
#endif

      if (entry->flags & SMARTFS_DIRENT_ACTIVE)
        {
          entrysector = entry->firstsector;

          if (entry->flags & SMARTFS_DIRENT_TYPE)
            {
              /* This entry is for directory */

              ret = smart_fsck_directory(dev, checkmap, entrysector);
            }
          else
            {
              /* This entry is for file */

              ret = smart_fsck_file(dev, checkmap, entrysector);
            }
        }

      if (ret != OK)
        {
#ifdef CONFIG_DEBUG_FS_INFO
          finfo("Remove entry (name=%s flags=%02x)\n",
                entryname, entry->flags);
#endif

          if ((cur + (2 * entrysize)) <= bottom)
            {
              /* Truncate the current entry and overwrite with next entries */

              memmove(cur, cur + entrysize, bottom - (cur + entrysize));
              memset(bottom - entrysize, CONFIG_SMARTFS_ERASEDSTATE,
                     entrysize);
            }
          else
            {
              /* Only erase the current entry if next entry does not
               * exist
               */

              memset(cur, CONFIG_SMARTFS_ERASEDSTATE, entrysize);
              cur += entrysize;
            }

          relocate = 1;
        }
      else
        {
          cur += entrysize;
        }
    }

  /* Relocate sector */

  if (relocate)
    {
      newsector = smart_findfreephyssector(dev, FALSE);
      if (newsector == 0xffff)
        {
          ret = -ENOSPC;
          ferr("Can't find a free sector for relocation\n");
          goto errout;
        }

      memcpy(dev->rwbuffer, rwbuffer, dev->sectorsize);

      ret = smart_relocate_sector(dev, physsector, newsector);
      if (ret < 0)
        {
          ret = -EIO;
          ferr("Can't relocate\n");
          goto errout;
        }

      /* Update the variables */

#ifndef CONFIG_MTD_SMART_MINIMIZE_RAM
      dev->smap[*((FAR uint16_t *)header->logicalsector)] = newsector;
#else
      smart_update_cache(dev, *((FAR uint16_t *)header->logicalsector),
                         newsector);
#endif

#ifdef CONFIG_MTD_SMART_PACK_COUNTS
      smart_add_count(dev, dev->freecount,
                      newsector / dev->sectorsperblk, -1);
#else
      dev->freecount[newsector / dev->sectorsperblk]--;
#endif
    }

  kmm_free(rwbuffer);
  CLR_BITMAP(checkmap, logsector);
  return OK;

errout:
  kmm_free(rwbuffer);
  SET_BITMAP(checkmap, logsector);
  return ret;
}

/****************************************************************************
 * Name: smart_fsck
 *
 * Description: Check and repair the file system
 *
 ****************************************************************************/

static int smart_fsck(FAR struct smart_struct_s *dev)
{
  uint16_t logsector;
#ifndef CONFIG_MTD_SMART_MINIMIZE_RAM
  uint16_t physsector;
#endif
  FAR uint8_t *checkmap;
  size_t mapsize;
  uint8_t rootdirentries;
  int x;

  finfo("Entry\n");

  /* Allocate a bitmap table for filesystem check */

  mapsize = (dev->totalsectors + 7) / 8;
  checkmap = (FAR uint8_t *)kmm_zalloc(mapsize);
  if (!checkmap)
    {
      ferr("ERROR: Out of memory fsck map\n");
      return -ENOMEM;
    }

  /* Set all of the sectors have live data into the check bitmap */

#ifndef CONFIG_MTD_SMART_MINIMIZE_RAM
  for (logsector = 0; logsector < dev->totalsectors; logsector++)
    {
      physsector = dev->smap[logsector];
      if (physsector < dev->totalsectors)
        {
          SET_BITMAP(checkmap, logsector);
        }
    }
#else
  memcpy(checkmap, dev->sbitmap, mapsize);
#endif

  /* Check if the sector can be available from root directories */

#ifdef CONFIG_SMARTFS_MULTI_ROOT_DIRS
  rootdirentries = dev->rootdirentries;
#else
  rootdirentries = 1;
#endif

  for (x = 0; x < rootdirentries; x++)
    {
      smart_fsck_directory(dev, checkmap, SMART_FIRST_DIR_SECTOR + x);
    }

  /* Release the invalid sector except for format or directory entry sector */

  for (logsector = SMART_FIRST_ALLOC_SECTOR;
       logsector < dev->totalsectors; logsector++)
    {
      if (ISSET_BITMAP(checkmap, logsector))
        {
          smart_freesector(dev, logsector);
        }
    }

  /* Free the bitmap table for filesystem check */

  kmm_free(checkmap);

  return OK;
}

#endif /* CONFIG_MTD_SMART_FSCK */

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

int smart_initialize(int minor, FAR struct mtd_dev_s *mtd,
                     FAR const char *partname)
{
  FAR struct smart_struct_s *dev;
  int ret = -ENOMEM;
  uint32_t totalsectors;
#ifdef CONFIG_SMARTFS_MULTI_ROOT_DIRS
  FAR struct smart_multiroot_device_s *rootdirdev = NULL;
#endif

  /* Sanity check */

#ifdef CONFIG_DEBUG_FEATURES
  if (minor < 0 || minor > 255 || !mtd)
    {
      return -EINVAL;
    }
#endif

  /* Allocate a SMART device structure */

  dev = (FAR struct smart_struct_s *)
    smart_zalloc(NULL, sizeof(struct smart_struct_s), "Dev struct");
  if (dev)
    {
      /* Initialize the SMART device structure */

      dev->mtd = mtd;

      /* Get the device geometry. (casting to uintptr_t first eliminates
       * complaints on some architectures where the sizeof long is different
       * from the size of a pointer).
       */

      /* Set these to zero in case the device doesn't support them */

      ret = MTD_IOCTL(mtd, MTDIOC_GEOMETRY,
                      (unsigned long)((uintptr_t)&dev->geo));
      if (ret < 0)
        {
          ferr("ERROR: MTD ioctl(MTDIOC_GEOMETRY) failed: %d\n", ret);
          goto errout;
        }

      /* Set the sector size to the default for now */

      dev->sectorsize = 0;
      ret = smart_setsectorsize(dev, CONFIG_MTD_SMART_SECTOR_SIZE);
      if (ret != OK)
        {
          goto errout;
        }

      /* Calculate the totalsectors on this device and validate */

      totalsectors = dev->neraseblocks * dev->sectorsperblk;
      if (totalsectors > 65536)
        {
          ferr("ERROR: SMART Sector size too small for device\n");
          ret = -EINVAL;
          goto errout;
        }
      else if (totalsectors == 65536)
        {
          totalsectors   -= 2;
        }

      dev->totalsectors   = (uint16_t)totalsectors;
      dev->freesectors    = (uint16_t)dev->availsectperblk *
                            dev->geo.neraseblocks;
      dev->lastallocblock = 0;
      dev->debuglevel     = 0;

      /* Mark the device format status an unknown */

      dev->formatstatus = SMART_FMT_STAT_UNKNOWN;
      dev->namesize = CONFIG_SMARTFS_MAXNAMLEN;
      if (partname)
        {
          strlcpy(dev->partname, partname, SMART_PARTNAME_SIZE);
        }
      else
        {
          dev->partname[0] = '\0';
        }

#ifdef CONFIG_SMARTFS_MULTI_ROOT_DIRS
      dev->minor = minor;
#endif

      /* Do a scan of the device */

      ret = smart_scan(dev);
      if (ret < 0)
        {
          ferr("ERROR: smart_scan failed: %d\n", -ret);
          goto errout;
        }

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

      rootdirdev = (FAR struct smart_multiroot_device_s *)
        smart_malloc(dev, sizeof(*rootdirdev), "Root Dir");
      if (rootdirdev == NULL)
        {
          ferr("ERROR: register_blockdriver failed: %d\n", -ret);
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
          ferr("ERROR: register_blockdriver failed: %d\n", -ret);
          goto errout;
        }
    }

#ifdef CONFIG_SMART_DEV_LOOP
  register_driver("/dev/smart", &g_fops, 0666, NULL);
#endif

  return OK;

errout:
#ifndef CONFIG_MTD_SMART_MINIMIZE_RAM
  smart_free(dev, dev->smap);
#else
  smart_free(dev, dev->sbitmap);
  smart_free(dev, dev->scache);
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
      smart_free(dev, rootdirdev);
    }
#endif

  kmm_free(dev);
  return ret;
}

/****************************************************************************
 * Name: smart_losetup
 *
 * Description: Dynamically setups up a SMART enabled loop device that
 *              is backed by a file.  The resulting loop device is a
 *              MTD type block device vs. a generic block device.
 *
 ****************************************************************************/

#ifdef CONFIG_SMART_DEV_LOOP
static int smart_losetup(int minor, FAR const char *filename,
                int sectsize, int erasesize, off_t offset, bool readonly)
{
  FAR struct mtd_dev_s *mtd;
  struct stat sb;
  int x;
  int ret;
  char devpath[20];

  /* Try to create a filemtd device using the filename provided */

  mtd = filemtd_initialize(filename, offset, sectsize, erasesize);
  if (mtd == NULL)
    {
      return -ENOENT;
    }

  /* Check if we need to dynamically assign a minor number */

  if (minor == -1)
    {
      /* Start at zero and stat /dev/smartX until no entry found.
       * Searching 0 to 256 should be sufficient.
       */

      for (x = 0; x < 256; x++)
        {
          snprintf(devpath, sizeof(devpath), "/dev/smart%d", x);
          ret = nx_stat(devpath, &sb, 1);
          if (ret < 0)
            {
              /* We can use this minor number */

              minor = x;
              break;
            }
        }
    }

  /* Now create a smart MTD using the filemtd backing it */

  ret = smart_initialize(minor, mtd, NULL);

  if (ret != OK)
    {
      filemtd_teardown(mtd);
    }

  return ret;
}
#endif /* CONFIG_SMART_DEV_LOOP */

/****************************************************************************
 * Name: loteardown
 *
 * Description:
 *   Undo the setup performed by losetup
 *
 ****************************************************************************/

#ifdef CONFIG_SMART_DEV_LOOP
static int smart_loteardown(FAR const char *devname)
{
  FAR struct smart_struct_s *dev;
  FAR struct inode *inode;
  int ret;

  /* Sanity check */

#ifdef CONFIG_DEBUG_FEATURES
  if (!devname)
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

  dev = (FAR struct smart_struct_s *)inode->i_private;

  /* Validate this is a filemtd backended device */

  if (!filemtd_isfilemtd(dev->mtd))
    {
      ferr("ERROR: Device is not a SMART loop: %s\n", devname);
      return -EINVAL;
    }

  close_blockdriver(inode);

  /* Now teardown the filemtd */

  filemtd_teardown(dev->mtd);
  unregister_blockdriver(devname);

  kmm_free(dev);

  return OK;
}
#endif /* CONFIG_SMART_DEV_LOOP */

/****************************************************************************
 * Name: smart_loop_read
 ****************************************************************************/

#ifdef CONFIG_SMART_DEV_LOOP
static ssize_t smart_loop_read(FAR struct file *filep, FAR char *buffer,
                               size_t len)
{
  return 0; /* Return EOF */
}
#endif /* CONFIG_SMART_DEV_LOOP */

/****************************************************************************
 * Name: smart_loop_write
 ****************************************************************************/

#ifdef CONFIG_SMART_DEV_LOOP
static ssize_t smart_loop_write(FAR struct file *filep,
                                FAR const char *buffer, size_t len)
{
  return len; /* Say that everything was written */
}
#endif /* CONFIG_SMART_DEV_LOOP */

/****************************************************************************
 * Name: smart_loop_ioctl
 ****************************************************************************/

#ifdef CONFIG_SMART_DEV_LOOP
static int smart_loop_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg)
{
  int ret;

  switch (cmd)
    {
    /* Command:      LOOPIOC_SETUP
     * Description:  Setup the loop device
     * Argument:     A pointer to a read-only instance of struct losetup_s.
     * Dependencies: The loop device must be enabled (CONFIG_DEV_LOOP=y)
     */

    case SMART_LOOPIOC_SETUP:
      {
        FAR struct smart_losetup_s *setup =
          (FAR struct smart_losetup_s *)((uintptr_t)arg);

        if (setup == NULL)
          {
            ret = -EINVAL;
          }
        else
          {
            ret = smart_losetup(setup->minor, setup->filename,
                                setup->sectsize, setup->erasesize,
                                setup->offset, setup->readonly);
          }
      }
      break;

    /* Command:      LOOPIOC_TEARDOWN
     * Description:  Teardown a loop device previously setup via
     *               LOOPIOC_SETUP
     * Argument:     A read-able pointer to the path of the device to be
     *               torn down
     * Dependencies: The loop device must be enabled (CONFIG_DEV_LOOP=y)
     */

    case SMART_LOOPIOC_TEARDOWN:
      {
        FAR const char *devname = (FAR const char *)((uintptr_t)arg);

        if (devname == NULL)
          {
            ret = -EINVAL;
          }
        else
          {
            ret = smart_loteardown(devname);
          }
       }
       break;

     default:
       ret = -ENOTTY;
    }

  return ret;
}
#endif /* CONFIG_SMART_DEV_LOOP */
