/****************************************************************************
 * arch/arm/src/rp2040/rp2040_flash_mtd.c
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
 * This code implements a Sector Mapped Allocation for Really Tiny (SMART)
 * filesystem in the RP2040 flash memory chip.  It uses the space not
 * otherwise used by the NuttX binary and supports both read and write
 * access.
 *
 * There initial contents of this filesystem may be configured when a NuttX
 * binary is built (using tools/rp2040/make_flash_fs.c), but any changes
 * subsequently written to the filesystem will persist over re-boots of the
 * RP2040.
 *
 * Note: Although read access to any data stored in this filesystem is very
 *       rapid; because of how the RP2040's flash access routines work with
 *       the normal execute-in-place (XIP) access to that chip, no code can
 *       access flash in the normal manner when this filesystem is erasing
 *       or writing blocks.   This means that interrupts must be disabled
 *       during any such access.  The main issue with this is that any
 *       application which requires precise timing or rapid response may
 *       be negatively impacted by such operations.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>

#include "rp2040_flash_mtd.h"
#include "rp2040_rom.h"

#include <sys/types.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define XIP_BASE                 0x10000000
#define XIP_NOCACHE_NOALLOC_BASE 0x13000000
#define FLASH_BLOCK_ERASE_CMD    0x20
#define BOOT_2_SIZE              256

#define FLASH_START_OFFSET (rp2040_smart_flash_start - (uint8_t *) XIP_BASE)
#define FLASH_END_OFFSET   (rp2040_smart_flash_end   - (uint8_t *) XIP_BASE)
#define FLASH_START_READ   (rp2040_smart_flash_start + 0x03000000)

/* Note: There is some ambiguity in terminology when it comes to flash.
 *       Some call the chunk that can be erased a sector where others
 *       call that a block.
 *
 *       Some call the chunk that can be written a sector where others
 *       call that a page.
 */

/* Blocks are the smallest unit that can be erased */

#define FLASH_BLOCK_SIZE   (4 * 1024)
#define FLASH_BLOCK_COUNT  (CONFIG_RP2040_FLASH_LENGTH - FLASH_START_OFFSET)\
                           / FLASH_BLOCK_SIZE

/* Sectors are the smallest unit that can be written */

#define FLASH_SECTOR_SIZE  256
#define FLASH_SECTOR_COUNT (CONFIG_RP2040_FLASH_LENGTH - FLASH_START_OFFSET)\
                           / FLASH_SECTOR_SIZE

#ifdef CONFIG_SMP
#  define OTHER_CPU (up_cpu_index() == 0 ? 1 : 0)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef struct rp2040_flash_dev_s
{
  struct mtd_dev_s mtd_dev;                 /* Embedded mdt_dev structure */
  mutex_t          lock;                    /* file access serialization  */
  uint32_t         boot_2[BOOT_2_SIZE / 4]; /* RAM copy of boot_2         */
} rp2040_flash_dev_t;

typedef void (*connect_internal_flash_f)(void);
typedef void (*flash_exit_xip_f)(void);
typedef void (*flash_range_erase_f)(uint32_t, size_t, uint32_t, uint8_t);
typedef void (*flash_range_program_f)(uint32_t, const uint8_t *, size_t);
typedef void (*flash_flush_cache_f)(void);
typedef void (*flash_enable_xip_f)(void);

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     rp2040_flash_erase      (struct mtd_dev_s *dev,
                                        off_t             startblock,
                                        size_t            nblocks);

static ssize_t rp2040_flash_block_read (struct mtd_dev_s *dev,
                                        off_t             startblock,
                                        size_t            nblocks,
                                        uint8_t          *buffer);

static ssize_t rp2040_flash_block_write(struct mtd_dev_s *dev,
                                        off_t             startblock,
                                        size_t            nblocks,
                                        const uint8_t    *buffer);

static ssize_t rp2040_flash_byte_read  (struct mtd_dev_s *dev,
                                        off_t             offset,
                                        size_t            nbytes,
                                        uint8_t          *buffer);

static int     rp2040_flash_ioctl      (struct mtd_dev_s *dev,
                                        int              cmd,
                                        unsigned long    arg);

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern const uint8_t rp2040_smart_flash_start[256];
extern const uint8_t rp2040_smart_flash_end[0];

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct rp2040_flash_dev_s my_dev =
{
  .mtd_dev =
    {
      rp2040_flash_erase,
      rp2040_flash_block_read,
      rp2040_flash_block_write,
      rp2040_flash_byte_read,
#ifdef CONFIG_MTD_BYTE_WRITE
      NULL,
#endif
      rp2040_flash_ioctl,
      "rp_flash"
  },
  .lock = NXMUTEX_INITIALIZER,
};

static bool initialized = false;

static struct
{
  connect_internal_flash_f connect_internal_flash;
  flash_exit_xip_f         flash_exit_xip;
  flash_range_erase_f      flash_range_erase;
  flash_range_program_f    flash_range_program;
  flash_flush_cache_f      flash_flush_cache;
  flash_enable_xip_f       flash_enable_xip;
} rom_functions;

void *boot_2_copy = NULL;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: do_erase
 ****************************************************************************/

void RAM_CODE(do_erase)(uint32_t addr, size_t count)
{
  /* Note: While we would prefer not to flush the cache, the
   * flash_flush_cache call is needed to remove  CSn IO force.
   */

  __asm__ volatile ("" : : : "memory");

  rom_functions.connect_internal_flash();

  rom_functions.flash_exit_xip();

  /* The range erase will try to erase 65536-byte blocks with the 0xd8 flash
   * command. If it cannot, because either the addr or count are not
   * multiple of 65536, it will fall back to erasing 4096-byte blocks as
   * needed.
   */

  rom_functions.flash_range_erase(addr, count, 65536, 0xd8);

  rom_functions.flash_flush_cache();

  rom_functions.flash_enable_xip();
}

/****************************************************************************
 * Name: do_write
 ****************************************************************************/

void RAM_CODE(do_write)(uint32_t addr, const uint8_t *data, size_t count)
{
  /* Note: While we would prefer not to flush the cache, the
   * flash_flush_cache call is needed to remove  CSn IO force.
   */

  __asm__ volatile ("" : : : "memory");

  rom_functions.connect_internal_flash();

  rom_functions.flash_exit_xip();

  rom_functions.flash_range_program(addr, (uint8_t *)data, count);

  rom_functions.flash_flush_cache();

  rom_functions.flash_enable_xip();
}

/****************************************************************************
 * Name: rp2040_flash_erase
 ****************************************************************************/

static int     rp2040_flash_erase(struct mtd_dev_s *dev,
                                  off_t             startblock,
                                  size_t            nblocks)
{
  rp2040_flash_dev_t *rp_dev = (rp2040_flash_dev_t *) dev;
  irqstate_t          flags;
  int                 ret    = OK;

  finfo("FLASH: erase block:  %8u (0x%08x) count:%5u (0x%08X)\n",
         (unsigned)(startblock),
         (unsigned)(FLASH_BLOCK_SIZE * startblock + FLASH_START_OFFSET),
         nblocks,
         FLASH_BLOCK_SIZE * nblocks);

  ret = nxmutex_lock(&rp_dev->lock);
  if (ret < 0)
    {
      return ret;
    }

  flags = enter_critical_section();

#ifdef CONFIG_SMP
  up_cpu_pause(OTHER_CPU);
#endif

  do_erase(FLASH_BLOCK_SIZE * startblock + FLASH_START_OFFSET,
            FLASH_BLOCK_SIZE * nblocks);

#ifdef CONFIG_SMP
  up_cpu_resume(OTHER_CPU);
#endif

  leave_critical_section(flags);

  ret = nblocks;

  nxmutex_unlock(&rp_dev->lock);
  return ret;
}

/****************************************************************************
 * Name: rp2040_flash_block_read
 ****************************************************************************/

static ssize_t rp2040_flash_block_read(struct mtd_dev_s *dev,
                                       off_t             startblock,
                                       size_t            nblocks,
                                       uint8_t          *buffer)
{
  rp2040_flash_dev_t *rp_dev = (rp2040_flash_dev_t *) dev;
  int                 start;
  int                 length;
  int                 ret   = OK;

  ret = nxmutex_lock(&rp_dev->lock);
  if (ret < 0)
    {
      return ret;
    }

  finfo("FLASH: read sector:  %8u (0x%08x) count:%5u\n",
         (unsigned)(startblock),
         (unsigned)(FLASH_SECTOR_SIZE * startblock + FLASH_START_OFFSET),
         nblocks);

  start  = FLASH_SECTOR_SIZE * startblock;
  length = FLASH_SECTOR_SIZE * nblocks;

  /* This reads starting at XIP_NOCACHE_NOALLOC_BASE to bypass the
   * XIP cache.  This is done because flash programming does not update
   * the cache and we don't want to read stale data.  Since we expect
   * access to the flash filesystem to be rather infrequent this isn't
   * really much of a burden.
   */

  memcpy(buffer, FLASH_START_READ + start, length);

  /* Update the file position */

  nxmutex_unlock(&rp_dev->lock);
  return nblocks;
}

/****************************************************************************
 * Name: rp2040_flash_write
 ****************************************************************************/

static ssize_t rp2040_flash_block_write(struct mtd_dev_s *dev,
                                        off_t             startblock,
                                        size_t            nblocks,
                                        const uint8_t    *buffer)
{
  rp2040_flash_dev_t *rp_dev = (rp2040_flash_dev_t *) dev;
  irqstate_t          flags;
  int                 ret;

  ret = nxmutex_lock(&rp_dev->lock);
  if (ret < 0)
    {
      return ret;
    }

  flags = enter_critical_section();

#ifdef CONFIG_SMP
  up_cpu_pause(OTHER_CPU);
#endif

  do_write(FLASH_SECTOR_SIZE * startblock + FLASH_START_OFFSET,
           buffer,
           FLASH_SECTOR_SIZE * nblocks);

#ifdef CONFIG_SMP
  up_cpu_resume(OTHER_CPU);
#endif

  leave_critical_section(flags);

  finfo("FLASH: write sector: %8u (0x%08x) count:%5u\n",
         (unsigned)(startblock),
         (unsigned)(FLASH_SECTOR_SIZE * startblock + FLASH_START_OFFSET),
         nblocks);

#ifdef CONFIG_DEBUG_FS_INFO
  for (int i = 0; i < FLASH_SECTOR_SIZE * nblocks; i += 16)
    {
      for (int j = 0; j < 16; ++j)
        {
          printf("%02x, ", buffer[i + j]);
        }

        printf("\n");
    }
#endif

  ret = nblocks;

  nxmutex_unlock(&rp_dev->lock);
  return ret;
}

/****************************************************************************
 * Name: rp2040_flash_byte_read
 ****************************************************************************/

static ssize_t rp2040_flash_byte_read(struct mtd_dev_s *dev,
                                      off_t             offset,
                                      size_t            nbytes,
                                      uint8_t          *buffer)
{
  rp2040_flash_dev_t *rp_dev = (rp2040_flash_dev_t *)dev;
  int                 length;
  int                 ret   = OK;

  ret = nxmutex_lock(&rp_dev->lock);
  if (ret < 0)
    {
      return ret;
    }

  length = nbytes;

  finfo("FLASH: read bytes:   %8u (0x%08x) count:%5u\n",
         (unsigned)(offset),
         (unsigned)(offset + FLASH_START_OFFSET),
         nbytes);

  /* This reads starting at XIP_NOCACHE_NOALLOC_BASE to bypass the
   * XIP cache.  This is done because flash programming does not update
   * the cache and we don't want to read stale data.  Since we expect
   * access to the flash filesystem to be rather infrequent this isn't
   * really much of a burden.
   */

  memcpy(buffer, FLASH_START_READ + offset, length);

#ifdef CONFIG_DEBUG_FS_INFO
  for (int j = 0; j < 16 && j < nbytes; ++j)
    {
      printf("%02x, ", buffer[j]);
    }

    printf("\n");
#endif

  /* Update the file position */

  nxmutex_unlock(&rp_dev->lock);
  return length;
}

/****************************************************************************
 * Name: rp2040_flash_ioctl
 ****************************************************************************/

static int rp2040_flash_ioctl(struct mtd_dev_s *dev,
                              int               cmd,
                              unsigned long     arg)
{
  rp2040_flash_dev_t *rp_dev = (rp2040_flash_dev_t *) dev;
  int                 ret    = OK;

  UNUSED(rp_dev);

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          struct mtd_geometry_s *geo = (struct mtd_geometry_s *) arg;

          if (geo != NULL)
            {
              geo->blocksize    = FLASH_SECTOR_SIZE;
              geo->erasesize    = FLASH_BLOCK_SIZE;
              geo->neraseblocks = FLASH_BLOCK_COUNT;
            }

          break;
        }

      case MTDIOC_BULKERASE:
        {
          /* Erase all the filesystem blocks for the device.  Remember that
           * we share this device with XIP memory so we cannot erase entire
           * device.
           */

          ret = rp2040_flash_erase(dev, 0, FLASH_BLOCK_COUNT);

          break;
        }

      default:
        ret = -ENOTTY;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp2040_flash_initialize
 *
 * Description: Bind a block mode driver that uses the built-in rp2040
 * flash programming commands for read/write access to unused flash.
 ****************************************************************************/

struct mtd_dev_s *rp2040_flash_mtd_initialize(void)
{
  if (initialized)
    {
      errno = EBUSY;
      return NULL;
    }

  initialized = true;

  if (FLASH_BLOCK_COUNT < 4)
    {
      errno = ENOMEM;
      return NULL;
    }

  rom_functions.connect_internal_flash = ROM_LOOKUP(ROM_FLASH_CONNECT);
  rom_functions.flash_exit_xip         = ROM_LOOKUP(ROM_FLASH_EXIT_XIP);
  rom_functions.flash_range_erase      = ROM_LOOKUP(ROM_FLASH_ERASE);
  rom_functions.flash_range_program    = ROM_LOOKUP(ROM_FLASH_PROGRAM);
  rom_functions.flash_flush_cache      = ROM_LOOKUP(ROM_FLASH_FLUSH_CACHE);

  /* Instead of using the rom_function for flash_enable_xip, we use the one
   * from boot stage 2 loaded at the beginning of the XIP rom. We do this
   * because the boot_rom version can result in slower access to the the
   * XIP memory.
   *
   * We need to make our own copy of this code in ram since we cannot use
   * the rom until after this call completes.
   */

  memcpy(my_dev.boot_2, (void *) XIP_BASE, BOOT_2_SIZE);
  rom_functions.flash_enable_xip = (flash_enable_xip_f)my_dev.boot_2 + 1;

  /* Do we need to initialize the flash? */

  if (memcmp(rp2040_smart_flash_start, "2040", 4) == 0)
    {
      uint8_t    buffer[FLASH_SECTOR_SIZE];
      irqstate_t flags = enter_critical_section();

      /* OK, we found the "magic" tag... */

      /* Erase all flash beyond what was loaded from NuttX binary. */

      do_erase(FLASH_END_OFFSET,
                CONFIG_RP2040_FLASH_LENGTH - FLASH_END_OFFSET);

      /* Erase the "magic" flag, setting the first two bytes to zero. */

      memcpy(buffer, rp2040_smart_flash_start, FLASH_SECTOR_SIZE);

      buffer[0] = 0;
      buffer[1] = 0;

      do_write(FLASH_START_OFFSET, buffer, FLASH_SECTOR_SIZE);

      leave_critical_section(flags);
    }

  return &(my_dev.mtd_dev);
}
