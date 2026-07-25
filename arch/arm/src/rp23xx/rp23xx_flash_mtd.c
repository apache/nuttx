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
 * MTD driver for the region of RP2350 QSPI flash left over by the NuttX
 * image.
 *
 * What makes this driver different from most is that it answers
 * BIOC_XIPBASE.  The RP2350 maps its external QSPI flash into the address
 * space at 0x10000000 and fetches instructions from it directly, so a
 * filesystem layered on top of this device can hand out genuine flash
 * pointers and let code execute in place.
 *
 * The cost of that arrangement is severe and unavoidable: there is exactly
 * one QSPI interface, and while it is being erased or programmed it cannot
 * serve reads.  Any code fetched from flash during that window hangs the
 * processor.  So every erase and program here
 *
 *   1. runs from SRAM -- the .time_critical section is copied to RAM at
 *      boot by the linker script, which is the same mechanism the Pico SDK
 *      spells __not_in_flash_func(),
 *   2. runs with interrupts disabled, because an ISR vector or handler
 *      living in flash would be fetched mid-erase, and
 *   3. parks the other core, because it is very likely executing from
 *      flash as well.
 *
 * A filesystem using this device for execute-in-place must additionally
 * guarantee it never erases a region that some task is currently executing
 * from.  That is a property of the filesystem, not of this driver; xipfs
 * provides it by refusing to relocate or erase any extent with a live
 * mapping.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/irq.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/mutex.h>
#include <nuttx/spinlock.h>

#include "rp23xx_flash_mtd.h"
#include "rp23xx_rom.h"
#include "hardware/rp23xx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Second, non-caching view of the same flash.  Reads through this alias
 * cannot return data left stale in the XIP cache by a program or erase, so
 * this driver reads through it while still reporting the cached base for
 * execute-in-place.
 */

#define RP23XX_XIP_NOCACHE_BASE   0x14000000

/* Largest flash the XIP window can address, used only to sanity check a
 * pointer before it is called with the flash interface torn down.
 */

#define RP23XX_FLASH_MAX_SIZE     0x04000000

/* Bootrom XIP read modes, and the clock divisor to run them at.  Quad is
 * the fast one; the bootrom validates the part can do it.
 */

#define RP23XX_XIP_MODE_03H_SERIAL 0
#define RP23XX_XIP_MODE_0BH_SERIAL 1
#define RP23XX_XIP_MODE_BBH_DUAL   2
#define RP23XX_XIP_MODE_EBH_QUAD   3
#define RP23XX_XIP_CLKDIV          4

/* Smallest unit that can be programmed, and smallest that can be erased */

#define FLASH_PAGE_SIZE           256
#define FLASH_SECTOR_SIZE         4096

/* Erase command used by the bootrom when a whole 64K block can be erased */

#define FLASH_BLOCK_SIZE          65536
#define FLASH_BLOCK_ERASE_CMD     0xd8

#define FS_OFFSET                 CONFIG_RP23XX_FLASH_MTD_OFFSET
#define FS_SIZE                   CONFIG_RP23XX_FLASH_MTD_SIZE

#define FS_SECTORS                (FS_SIZE / FLASH_SECTOR_SIZE)
#define FS_PAGES                  (FS_SIZE / FLASH_PAGE_SIZE)

/* Functions that must not be fetched from flash while flash is busy.  The
 * rp23xx linker scripts already place .time_critical in RAM.
 */

#define RAM_CODE(name) \
  __attribute__((noinline, section(".time_critical." #name))) name

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rp23xx_flash_dev_s
{
  struct mtd_dev_s mtd;
  mutex_t          lock;
};

typedef void (*connect_internal_flash_f)(void);
typedef void (*flash_exit_xip_f)(void);
typedef void (*flash_range_erase_f)(uint32_t, size_t, uint32_t, uint8_t);
typedef void (*flash_range_program_f)(uint32_t, const uint8_t *, size_t);
typedef void (*flash_flush_cache_f)(void);
typedef void (*flash_enter_cmd_xip_f)(void);
typedef bool (*select_xip_read_mode_f)(uint32_t mode, uint8_t clkdiv);

#ifdef CONFIG_SMP
/* Locks coordinating "pause" and "resume" with the handler that blocks a
 * CPU for the duration of a flash operation.
 */

struct smp_isolation_data_s
{
  volatile spinlock_t cpu_wait;
  volatile spinlock_t cpu_pause;
  volatile spinlock_t cpu_resume;
  struct smp_call_data_s call_data;
};

struct smp_isolation_s
{
  int isolated_cpuid;
  struct smp_isolation_data_s cpu_data[CONFIG_SMP_NCPUS];
};
#endif

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

static struct rp23xx_flash_dev_s g_flash_dev =
{
  .mtd =
    {
      rp23xx_flash_erase,
      rp23xx_flash_bread,
      rp23xx_flash_bwrite,
      rp23xx_flash_read,
#ifdef CONFIG_MTD_BYTE_WRITE
      NULL,
#endif
      rp23xx_flash_ioctl,
#ifdef CONFIG_FTL_BBM
      NULL,
      NULL,
#endif
      "rp23xx_flash"
    },
  .lock = NXMUTEX_INITIALIZER,
};

static bool g_initialized = false;

static struct
{
  connect_internal_flash_f connect_internal_flash;
  flash_exit_xip_f         flash_exit_xip;
  flash_range_erase_f      flash_range_erase;
  flash_range_program_f    flash_range_program;
  flash_flush_cache_f      flash_flush_cache;
  flash_enter_cmd_xip_f    flash_enter_cmd_xip;

  /* Restores a fast XIP read mode.  flash_enter_cmd_xip works everywhere
   * but leaves the flash in a slow 03h serial mode, which costs roughly an
   * order of magnitude of read bandwidth -- and the base firmware executes
   * from this same flash, so it is not a cost confined to the filesystem.
   */

  select_xip_read_mode_f   select_xip_read_mode;
} g_rom;

/* End of the NuttX image in flash, provided by the linker script.  Declared
 * weak so that a RAM-only memory map, which does not define it, still
 * links.
 */

extern uint8_t __flash_binary_end[] __attribute__((weak));

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_SMP

/****************************************************************************
 * Name: pause_cpu_handler
 *
 * Description:
 *   Busy-wait until leave_smp_isolation releases our lock.  This runs on
 *   every CPU except the one performing the flash operation.
 *
 *   Note this handler itself must not be fetched from flash, hence the
 *   RAM_CODE placement.
 *
 ****************************************************************************/

static int RAM_CODE(pause_cpu_handler)(void *const context)
{
  struct smp_isolation_data_s *const cpu_data =
    (struct smp_isolation_data_s *)context;

  spin_lock(&cpu_data->cpu_resume);
  spin_unlock(&cpu_data->cpu_pause);

  spin_lock(&cpu_data->cpu_wait);
  spin_unlock(&cpu_data->cpu_wait);

  spin_unlock(&cpu_data->cpu_resume);

  return OK;
}

/****************************************************************************
 * Name: init_smp_isolation
 ****************************************************************************/

static void init_smp_isolation(struct smp_isolation_s *const data)
{
  struct smp_isolation_data_s *cpu_data;
  int cpuid;

  for (cpuid = 0; cpuid < CONFIG_SMP_NCPUS; cpuid++)
    {
      cpu_data = &data->cpu_data[cpuid];
      spin_lock_init(&cpu_data->cpu_wait);
      spin_lock_init(&cpu_data->cpu_pause);
      spin_lock_init(&cpu_data->cpu_resume);
    }
}

/****************************************************************************
 * Name: enter_smp_isolation
 *
 * Description:
 *   Force every CPU except this one into a RAM-resident busy loop, so that
 *   none of them tries to fetch an instruction from flash while it is being
 *   erased or programmed.
 *
 ****************************************************************************/

static void enter_smp_isolation(struct smp_isolation_s *const data)
{
  struct smp_isolation_data_s *cpu_data;
  int other_cpuid;

  sched_lock();

  data->isolated_cpuid = this_cpu();

  for (other_cpuid = 0; other_cpuid < CONFIG_SMP_NCPUS; other_cpuid++)
    {
      cpu_data = &data->cpu_data[other_cpuid];

      if (other_cpuid != data->isolated_cpuid)
        {
          spin_lock(&cpu_data->cpu_wait);
          spin_lock(&cpu_data->cpu_pause);
          spin_unlock(&cpu_data->cpu_resume);
        }

      nxsched_smp_call_init(&cpu_data->call_data, pause_cpu_handler,
                            cpu_data);
      nxsched_smp_call_single_async(other_cpuid, &cpu_data->call_data);
    }

  /* Wait until every other CPU has actually parked */

  for (other_cpuid = 0; other_cpuid < CONFIG_SMP_NCPUS; other_cpuid++)
    {
      cpu_data = &data->cpu_data[other_cpuid];

      if (other_cpuid != data->isolated_cpuid)
        {
          spin_lock(&cpu_data->cpu_pause);
          spin_unlock(&cpu_data->cpu_pause);
        }
    }
}

/****************************************************************************
 * Name: leave_smp_isolation
 ****************************************************************************/

static void leave_smp_isolation(struct smp_isolation_s *const data)
{
  struct smp_isolation_data_s *cpu_data;
  int other_cpuid;

  for (other_cpuid = 0; other_cpuid < CONFIG_SMP_NCPUS; other_cpuid++)
    {
      cpu_data = &data->cpu_data[other_cpuid];

      if (other_cpuid != data->isolated_cpuid)
        {
          spin_unlock(&cpu_data->cpu_wait);
        }
    }

  for (other_cpuid = 0; other_cpuid < CONFIG_SMP_NCPUS; other_cpuid++)
    {
      cpu_data = &data->cpu_data[other_cpuid];

      if (other_cpuid != data->isolated_cpuid)
        {
          spin_lock(&cpu_data->cpu_resume);
          spin_unlock(&cpu_data->cpu_resume);
        }
    }

  sched_unlock();
}

#endif /* CONFIG_SMP */

/****************************************************************************
 * Name: rp23xx_flash_restore_xip
 *
 * Description:
 *   Put the QSPI interface back into execute-in-place mode.  Must run from
 *   RAM: until it returns, nothing can be fetched from flash.
 *
 ****************************************************************************/

static void RAM_CODE(rp23xx_flash_restore_xip)(void)
{
  g_rom.flash_flush_cache();

  /* Ask the bootrom to put the flash back into a fast quad read mode.  It
   * reports whether it managed to, so a part that cannot do quad falls
   * back rather than leaving the interface unusable.
   */

  if (g_rom.select_xip_read_mode == NULL ||
      !g_rom.select_xip_read_mode(RP23XX_XIP_MODE_EBH_QUAD,
                                  RP23XX_XIP_CLKDIV))
    {
      g_rom.flash_enter_cmd_xip();
    }
}

/****************************************************************************
 * Name: do_erase
 *
 * Description:
 *   Erase a byte range.  Runs from RAM with interrupts already disabled and
 *   the other core already parked.
 *
 ****************************************************************************/

static void RAM_CODE(do_erase)(uint32_t addr, size_t count)
{
  __asm__ volatile ("" : : : "memory");

  g_rom.connect_internal_flash();
  g_rom.flash_exit_xip();

  /* The bootrom erases whole 64K blocks where address and length allow it
   * and falls back to 4K sectors otherwise.
   */

  g_rom.flash_range_erase(addr, count, FLASH_BLOCK_SIZE,
                          FLASH_BLOCK_ERASE_CMD);

  rp23xx_flash_restore_xip();
}

/****************************************************************************
 * Name: do_write
 ****************************************************************************/

static void RAM_CODE(do_write)(uint32_t addr, const uint8_t *data,
                               size_t count)
{
  __asm__ volatile ("" : : : "memory");

  g_rom.connect_internal_flash();
  g_rom.flash_exit_xip();

  g_rom.flash_range_program(addr, data, count);

  rp23xx_flash_restore_xip();
}

/****************************************************************************
 * Name: rp23xx_flash_erase
 ****************************************************************************/

static int rp23xx_flash_erase(struct mtd_dev_s *dev, off_t startblock,
                              size_t nblocks)
{
  struct rp23xx_flash_dev_s *priv = (struct rp23xx_flash_dev_s *)dev;
  irqstate_t flags;
  int ret;

#ifdef CONFIG_SMP
  struct smp_isolation_s smp_isolation;
  init_smp_isolation(&smp_isolation);
#endif

  if (startblock < 0 || startblock + nblocks > FS_SECTORS)
    {
      return -EINVAL;
    }

  finfo("erase sector %ld count %zu\n", (long)startblock, nblocks);

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_SMP
  enter_smp_isolation(&smp_isolation);
#endif

  flags = enter_critical_section();

  do_erase(FS_OFFSET + startblock * FLASH_SECTOR_SIZE,
           nblocks * FLASH_SECTOR_SIZE);

  leave_critical_section(flags);

#ifdef CONFIG_SMP
  leave_smp_isolation(&smp_isolation);
#endif

  nxmutex_unlock(&priv->lock);
  return nblocks;
}

/****************************************************************************
 * Name: rp23xx_flash_bread
 *
 * Description:
 *   Read whole program pages.  Reads go through the non-caching alias so
 *   that data just programmed is never masked by a stale cache line.
 *
 ****************************************************************************/

static ssize_t rp23xx_flash_bread(struct mtd_dev_s *dev, off_t startblock,
                                  size_t nblocks, uint8_t *buffer)
{
  struct rp23xx_flash_dev_s *priv = (struct rp23xx_flash_dev_s *)dev;
  int ret;

  if (startblock < 0 || startblock + nblocks > FS_PAGES)
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  memcpy(buffer,
         (const void *)(RP23XX_XIP_NOCACHE_BASE + FS_OFFSET +
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

#ifdef CONFIG_SMP
  struct smp_isolation_s smp_isolation;
  init_smp_isolation(&smp_isolation);
#endif

  if (startblock < 0 || startblock + nblocks > FS_PAGES)
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_SMP
  enter_smp_isolation(&smp_isolation);
#endif

  flags = enter_critical_section();

  do_write(FS_OFFSET + startblock * FLASH_PAGE_SIZE, buffer,
           nblocks * FLASH_PAGE_SIZE);

  leave_critical_section(flags);

#ifdef CONFIG_SMP
  leave_smp_isolation(&smp_isolation);
#endif

  finfo("write page %ld count %zu\n", (long)startblock, nblocks);

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

  if (offset < 0 || offset + nbytes > FS_SIZE)
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  memcpy(buffer,
         (const void *)(RP23XX_XIP_NOCACHE_BASE + FS_OFFSET + offset),
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

          if (geo == NULL)
            {
              return -EINVAL;
            }

          memset(geo, 0, sizeof(*geo));
          geo->blocksize    = FLASH_PAGE_SIZE;
          geo->erasesize    = FLASH_SECTOR_SIZE;
          geo->neraseblocks = FS_SECTORS;
          strlcpy(geo->model, "rp23xx_flash", sizeof(geo->model));
        }
        break;

      case BIOC_XIPBASE:
        {
          /* The whole point of this driver.  Report the cached execute-in-
           * place view, not the non-caching alias used for reads above:
           * code fetched through this pointer should be cached, and every
           * program and erase path here flushes the cache before returning.
           */

          void **ppv = (void **)arg;

          if (ppv == NULL)
            {
              return -EINVAL;
            }

          *ppv = (void *)(RP23XX_FLASH_BASE + FS_OFFSET);
        }
        break;

      case MTDIOC_ERASESTATE:
        {
          uint8_t *result = (uint8_t *)arg;

          if (result == NULL)
            {
              return -EINVAL;
            }

          *result = 0xff;
        }
        break;

      case MTDIOC_BULKERASE:
        ret = rp23xx_flash_erase(dev, 0, FS_SECTORS);
        if (ret >= 0)
          {
            ret = OK;
          }
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
      set_errno(EBUSY);
      return NULL;
    }

  /* Refuse to hand out a device that overlaps the running image.  Getting
   * this wrong does not fail gracefully: the first erase would destroy the
   * code performing it.
   */

  if (__flash_binary_end != NULL &&
      (uintptr_t)__flash_binary_end > RP23XX_FLASH_BASE + FS_OFFSET)
    {
      merr("ERROR: flash MTD region at 0x%08x overlaps the NuttX image "
           "ending at %p\n",
           (unsigned)(RP23XX_FLASH_BASE + FS_OFFSET), __flash_binary_end);
      set_errno(EINVAL);
      return NULL;
    }

  if ((FS_OFFSET % FLASH_SECTOR_SIZE) != 0 ||
      (FS_SIZE % FLASH_SECTOR_SIZE) != 0 || FS_SIZE == 0)
    {
      merr("ERROR: flash MTD region must be sector aligned\n");
      set_errno(EINVAL);
      return NULL;
    }

  g_rom.connect_internal_flash =
    rom_func_lookup(ROM_FUNC_CONNECT_INTERNAL_FLASH);
  g_rom.flash_exit_xip      = rom_func_lookup(ROM_FUNC_FLASH_EXIT_XIP);
  g_rom.flash_range_erase   = rom_func_lookup(ROM_FUNC_FLASH_RANGE_ERASE);
  g_rom.flash_range_program = rom_func_lookup(ROM_FUNC_FLASH_RANGE_PROGRAM);
  g_rom.flash_flush_cache   = rom_func_lookup(ROM_FUNC_FLASH_FLUSH_CACHE);
  g_rom.flash_enter_cmd_xip =
    rom_func_lookup(ROM_FUNC_FLASH_ENTER_CMD_XIP);

  if (g_rom.connect_internal_flash == NULL ||
      g_rom.flash_exit_xip == NULL ||
      g_rom.flash_range_erase == NULL ||
      g_rom.flash_range_program == NULL ||
      g_rom.flash_flush_cache == NULL ||
      g_rom.flash_enter_cmd_xip == NULL)
    {
      merr("ERROR: bootrom flash functions not found\n");
      set_errno(ENODEV);
      return NULL;
    }

  /* Resolve the fast XIP read mode selector.  Unlike the bootrom's saved
   * XIP setup pointer -- which is a data table entry whose semantics this
   * driver got wrong, and calling it with flash torn down hangs the core
   * unrecoverably -- this is an ordinary ROM function looked up exactly
   * like the others above, and it returns a status.
   */

#ifndef CONFIG_RP23XX_FLASH_MTD_SAFE_XIP
  g_rom.select_xip_read_mode =
    rom_func_lookup(ROM_FUNC_FLASH_SELECT_XIP_READ_MODE);

  if (g_rom.select_xip_read_mode == NULL)
    {
      fwarn("rp23xx_flash: no fast XIP selector; reads will be slow after "
            "every flash operation\n");
    }
#else
  g_rom.select_xip_read_mode = NULL;
#endif

  g_initialized = true;

  finfo("rp23xx_flash: %u sectors of %u bytes at 0x%08x\n",
        (unsigned)FS_SECTORS, (unsigned)FLASH_SECTOR_SIZE,
        (unsigned)(RP23XX_FLASH_BASE + FS_OFFSET));

  return &g_flash_dev.mtd;
}
