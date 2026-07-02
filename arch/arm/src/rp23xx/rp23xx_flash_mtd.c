/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_flash_mtd.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
 * This implements an MTD device over the area of the RP2350's on-chip QSPI
 * flash that is not occupied by the NuttX binary.  It is intended to back a
 * LittleFS (or similar) filesystem for persistent local storage.
 *
 * Because the RP2350 normally executes code directly from this same flash
 * (execute-in-place, XIP), no code may be fetched from flash while a block
 * is being erased or programmed.  Therefore the erase/program routines run
 * from RAM (the ".ram_code.*" section, which is copied into RAM with .data
 * at startup) and run with interrupts disabled.
 *
 * The RP2350 flash-programming sequence follows the Raspberry Pi Pico SDK
 * (hardware_flash/flash.c) and the RP2350 datasheet bootrom chapter:
 *
 *   - the bootrom flash helpers are looked up via rom_func_lookup();
 *   - after flash_flush_cache() the bootrom already leaves the flash in a
 *     basic (readable) XIP state, so XIP is re-enabled by calling the
 *     bootrom flash_enter_cmd_xip() helper (no copied BOOTRAM setup
 *     function needed);
 *   - the QMI window-1 (CS1) registers, which flash_exit_xip() disturbs on
 *     the RP2350, are saved before and restored after the operation.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>
#include <nuttx/mtd/mtd.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include "rp23xx_rom.h"
#include "rp23xx_flash_mtd.h"
#include "hardware/rp23xx_memorymap.h"
#include "hardware/rp23xx_qmi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Run this function from RAM (copied along with .data at startup). */

#define RAM_CODE __attribute__((noinline, section(".ram_code.flash_mtd")))

/* Read the filesystem through the normal cached XIP window (0x10000000) —
 * the same alias the CPU executes from, so it is always functional.
 * Freshness after a program/erase is guaranteed because do_flash_op() calls
 * the bootrom flash_flush_cache() (which invalidates the XIP cache) at the
 * end of every operation.  (The uncached alias 0x14000000 was used
 * previously, but reading through it depends on the uncached window being
 * set up and is a needless fault risk during the very first mount-time
 * read.)
 */

#define RP23XX_XIP_READ_BASE  0x10000000

/* QMI window-1 registers clobbered by flash_exit_xip() on the RP2350.
 * (Build the absolute addresses from the QMI base + register offsets; the
 * convenience macros in rp23xx_qmi.h reference an undefined
 * RP23XX_QMI_BASE.)
 */

#define QMI_M1_TIMING_REG  (RP23XX_XIP_QMI_BASE + RP23XX_QMI_M1_TIMING_OFFSET)
#define QMI_M1_RFMT_REG    (RP23XX_XIP_QMI_BASE + RP23XX_QMI_M1_RFMT_OFFSET)
#define QMI_M1_RCMD_REG    (RP23XX_XIP_QMI_BASE + RP23XX_QMI_M1_RCMD_OFFSET)

/* Flash geometry */

#define FLASH_PAGE_SIZE          256       /* Smallest program unit         */
#define FLASH_BLOCK_SIZE         4096      /* Smallest erase unit           */
#define FLASH_BLOCK_ERASE_CMD    0xd8      /* 64KiB block erase (ROM falls  */
#define FLASH_LARGE_BLOCK        65536     /* back to 4KiB where unaligned) */

/* Total flash size and the offset at which the filesystem region begins.
 * The region runs from CONFIG_RP23XX_FLASH_FS_OFFSET to the end of flash.
 * Both must be 4KiB aligned and clear of the NuttX image.
 */

#ifndef CONFIG_RP23XX_FLASH_LENGTH
#  define CONFIG_RP23XX_FLASH_LENGTH    (4 * 1024 * 1024)
#endif

#ifndef CONFIG_RP23XX_FLASH_FS_OFFSET
#  define CONFIG_RP23XX_FLASH_FS_OFFSET (1 * 1024 * 1024)
#endif

#define FLASH_FS_OFFSET   CONFIG_RP23XX_FLASH_FS_OFFSET
#define FLASH_FS_LENGTH   (CONFIG_RP23XX_FLASH_LENGTH - FLASH_FS_OFFSET)
#define FLASH_BLOCK_COUNT (FLASH_FS_LENGTH / FLASH_BLOCK_SIZE)
#define FLASH_PAGE_COUNT  (FLASH_FS_LENGTH / FLASH_PAGE_SIZE)

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef void (*rom_void_fn)(void);
typedef void (*rom_erase_fn)(uint32_t, size_t, uint32_t, uint8_t);
typedef void (*rom_program_fn)(uint32_t, const uint8_t *, size_t);

struct rp23xx_flash_dev_s
{
  struct mtd_dev_s mtd;       /* Embedded MTD interface (must be first)    */
  mutex_t          lock;      /* Serializes access                         */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     rp23xx_flash_erase(struct mtd_dev_s *dev, off_t startblock,
                                  size_t nblocks);
static ssize_t rp23xx_flash_bread(struct mtd_dev_s *dev, off_t startblock,
                                  size_t nblocks, uint8_t *buffer);
static ssize_t rp23xx_flash_bwrite(struct mtd_dev_s *dev, off_t startblock,
                                   size_t nblocks, const uint8_t *buffer);
static ssize_t rp23xx_flash_read(struct mtd_dev_s *dev, off_t offset,
                                 size_t nbytes, uint8_t *buffer);
static int     rp23xx_flash_ioctl(struct mtd_dev_s *dev, int cmd,
                                  unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The bootrom function pointers and the RAM copy of the XIP setup function.
 * All of these live in RAM (.bss/.data) so they may be touched from the
 * RAM-resident do_flash_op() while XIP is disabled.
 */

static struct
{
  rom_void_fn    connect_internal_flash;
  rom_void_fn    flash_exit_xip;
  rom_erase_fn   flash_range_erase;
  rom_program_fn flash_range_program;
  rom_void_fn    flash_flush_cache;
  rom_void_fn    flash_enter_cmd_xip;
} g_rom;

static struct rp23xx_flash_dev_s g_dev =
{
  .mtd =
  {
    .erase  = rp23xx_flash_erase,
    .bread  = rp23xx_flash_bread,
    .bwrite = rp23xx_flash_bwrite,
    .read   = rp23xx_flash_read,
    .ioctl  = rp23xx_flash_ioctl,
    .name   = "rp23xx_flash",
  },
  .lock = NXMUTEX_INITIALIZER,
};

static bool g_initialized = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: do_flash_op
 *
 * Description:
 *   Perform a single erase or program operation against the flash.  This
 *   MUST run from RAM with interrupts disabled: while XIP is exited, the CPU
 *   cannot fetch instructions or read data from the flash.
 *
 *   addr  - byte offset from the start of flash (NOT an XIP address)
 *   data  - source buffer for a program; ignored for an erase
 *   count - number of bytes to program/erase (block/page aligned)
 *   erase - true to erase, false to program
 *
 ****************************************************************************/

static void RAM_CODE do_flash_op(uint32_t addr, const uint8_t *data,
                                 size_t count, bool erase)
{
  /* Save the QMI window-1 registers that flash_exit_xip() disturbs. */

  uint32_t m1_timing = *(volatile uint32_t *)QMI_M1_TIMING_REG;
  uint32_t m1_rfmt   = *(volatile uint32_t *)QMI_M1_RFMT_REG;
  uint32_t m1_rcmd   = *(volatile uint32_t *)QMI_M1_RCMD_REG;

  /* No flash access is permitted past this barrier until XIP is re-enabled */

  __asm__ volatile ("" : : : "memory");

  g_rom.connect_internal_flash();
  g_rom.flash_exit_xip();

  if (erase)
    {
      g_rom.flash_range_erase(addr, count, FLASH_LARGE_BLOCK,
                              FLASH_BLOCK_ERASE_CMD);
    }
  else
    {
      g_rom.flash_range_program(addr, data, count);
    }

  /* flash_flush_cache() also drops the CSn IO force left by exit_xip(). */

  g_rom.flash_flush_cache();

  /* Re-enable XIP.  Use the bootrom's flash_enter_cmd_xip helper, which is
   * always present and restores a working (standard read) XIP mode without
   * depending on a copied boot2/bootram setup function.
   */

  g_rom.flash_enter_cmd_xip();

  /* Restore the QMI window-1 registers. */

  *(volatile uint32_t *)QMI_M1_TIMING_REG = m1_timing;
  *(volatile uint32_t *)QMI_M1_RFMT_REG   = m1_rfmt;
  *(volatile uint32_t *)QMI_M1_RCMD_REG   = m1_rcmd;
}

/****************************************************************************
 * Name: rp23xx_flash_erase
 ****************************************************************************/

static int rp23xx_flash_erase(struct mtd_dev_s *dev, off_t startblock,
                              size_t nblocks)
{
  struct rp23xx_flash_dev_s *priv = (struct rp23xx_flash_dev_s *)dev;
  irqstate_t flags;
  size_t i;
  int ret;

  if (startblock + nblocks > FLASH_BLOCK_COUNT)
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  finfo("erase block %ju count %zu\n", (uintmax_t)startblock, nblocks);

  /* Erase one block per critical section.  do_flash_op() runs with
   * interrupts disabled (XIP is exited during the operation); erasing the
   * whole request in a single critical section would hold interrupts off for
   * many seconds on a large (e.g. bulk) erase and stall the system.  One
   * 4 KiB block at a time keeps each interrupts-off window short.
   */

  for (i = 0; i < nblocks; i++)
    {
      flags = enter_critical_section();
      do_flash_op(FLASH_FS_OFFSET + (startblock + i) * FLASH_BLOCK_SIZE,
                  NULL, FLASH_BLOCK_SIZE, true);
      leave_critical_section(flags);
    }

  nxmutex_unlock(&priv->lock);
  return nblocks;
}

/****************************************************************************
 * Name: rp23xx_flash_bread
 ****************************************************************************/

static ssize_t rp23xx_flash_bread(struct mtd_dev_s *dev, off_t startblock,
                                  size_t nblocks, uint8_t *buffer)
{
  struct rp23xx_flash_dev_s *priv = (struct rp23xx_flash_dev_s *)dev;
  int ret;

  if (startblock + nblocks > FLASH_PAGE_COUNT)
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Read through the cached XIP window; do_flash_op() invalidates the XIP
   * cache via flash_flush_cache() after every program/erase, so this never
   * returns stale data.
   */

  memcpy(buffer,
         (const void *)(RP23XX_XIP_READ_BASE + FLASH_FS_OFFSET +
                        startblock * FLASH_PAGE_SIZE),
         nblocks * FLASH_PAGE_SIZE);

  nxmutex_unlock(&priv->lock);
  return nblocks;
}

/****************************************************************************
 * Name: rp23xx_flash_bwrite
 ****************************************************************************/

static ssize_t rp23xx_flash_bwrite(struct mtd_dev_s *dev, off_t startblock,
                                   size_t nblocks, const uint8_t *buffer)
{
  struct rp23xx_flash_dev_s *priv = (struct rp23xx_flash_dev_s *)dev;
  irqstate_t flags;
  int ret;

  if (startblock + nblocks > FLASH_PAGE_COUNT)
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  finfo("write page %ju count %zu\n", (uintmax_t)startblock, nblocks);

  flags = enter_critical_section();
  do_flash_op(FLASH_FS_OFFSET + startblock * FLASH_PAGE_SIZE, buffer,
              nblocks * FLASH_PAGE_SIZE, false);
  leave_critical_section(flags);

  nxmutex_unlock(&priv->lock);
  return nblocks;
}

/****************************************************************************
 * Name: rp23xx_flash_read
 ****************************************************************************/

static ssize_t rp23xx_flash_read(struct mtd_dev_s *dev, off_t offset,
                                 size_t nbytes, uint8_t *buffer)
{
  struct rp23xx_flash_dev_s *priv = (struct rp23xx_flash_dev_s *)dev;
  int ret;

  if (offset + nbytes > FLASH_FS_LENGTH)
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  memcpy(buffer,
         (const void *)(RP23XX_XIP_READ_BASE + FLASH_FS_OFFSET + offset),
         nbytes);

  nxmutex_unlock(&priv->lock);
  return nbytes;
}

/****************************************************************************
 * Name: rp23xx_flash_ioctl
 ****************************************************************************/

static int rp23xx_flash_ioctl(struct mtd_dev_s *dev, int cmd,
                              unsigned long arg)
{
  int ret = OK;

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          struct mtd_geometry_s *geo = (struct mtd_geometry_s *)arg;

          if (geo != NULL)
            {
              memset(geo, 0, sizeof(*geo));
              geo->blocksize    = FLASH_PAGE_SIZE;
              geo->erasesize    = FLASH_BLOCK_SIZE;
              geo->neraseblocks = FLASH_BLOCK_COUNT;
            }
          break;
        }

      case MTDIOC_BULKERASE:
        ret = rp23xx_flash_erase(dev, 0, FLASH_BLOCK_COUNT);
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp23xx_flash_mtd_initialize
 ****************************************************************************/

struct mtd_dev_s *rp23xx_flash_mtd_initialize(void)
{
  if (g_initialized)
    {
      errno = EBUSY;
      return NULL;
    }

  if (FLASH_BLOCK_COUNT < 4 || (FLASH_FS_OFFSET % FLASH_BLOCK_SIZE) != 0)
    {
      errno = EINVAL;
      return NULL;
    }

  /* Resolve the bootrom flash helper functions. */

  g_rom.connect_internal_flash =
      (rom_void_fn)rom_func_lookup(ROM_FUNC_CONNECT_INTERNAL_FLASH);
  g_rom.flash_exit_xip =
      (rom_void_fn)rom_func_lookup(ROM_FUNC_FLASH_EXIT_XIP);
  g_rom.flash_range_erase =
      (rom_erase_fn)rom_func_lookup(ROM_FUNC_FLASH_RANGE_ERASE);
  g_rom.flash_range_program =
      (rom_program_fn)rom_func_lookup(ROM_FUNC_FLASH_RANGE_PROGRAM);
  g_rom.flash_flush_cache =
      (rom_void_fn)rom_func_lookup(ROM_FUNC_FLASH_FLUSH_CACHE);
  g_rom.flash_enter_cmd_xip =
      (rom_void_fn)rom_func_lookup(ROM_FUNC_FLASH_ENTER_CMD_XIP);

  if (g_rom.connect_internal_flash == NULL ||
      g_rom.flash_exit_xip == NULL ||
      g_rom.flash_range_erase == NULL ||
      g_rom.flash_range_program == NULL ||
      g_rom.flash_flush_cache == NULL ||
      g_rom.flash_enter_cmd_xip == NULL)
    {
      errno = ENODEV;
      return NULL;
    }

  g_initialized = true;

  finfo("rp23xx flash MTD: offset=0x%x length=0x%x blocks=%d\n",
        (unsigned)FLASH_FS_OFFSET, (unsigned)FLASH_FS_LENGTH,
        (int)FLASH_BLOCK_COUNT);

  return &g_dev.mtd;
}
