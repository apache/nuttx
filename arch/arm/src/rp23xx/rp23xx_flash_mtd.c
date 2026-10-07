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
 *      living in flash would be fetched mid-erase -- one 64K block erase
 *      or one 256 byte page program at a time, and
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
#include "hardware/rp23xx_pads_qspi.h"
#include "hardware/rp23xx_qmi.h"
#include "hardware/rp23xx_xip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Second, non-caching view of the same flash.  Reads through this alias
 * cannot return data left stale in the XIP cache by a program or erase, so
 * this driver reads through it while still reporting the cached base for
 * execute-in-place.
 */

#define RP23XX_XIP_NOCACHE_BASE   0x14000000

/* XIP cache maintenance window.  A clean by set/way must use the top of the
 * window to avoid erratum RP2350-E11, as the Pico SDK does.
 */

#define RP23XX_XIP_MAINT_BASE     0x18000000
#define XIP_CACHE_SIZE            (16 * 1024)
#define XIP_CACHE_LINE_SIZE       8
#define XIP_CACHE_CLEAN_SET_WAY   1
#define XIP_CACHE_CLEAN_BASE      (RP23XX_XIP_MAINT_BASE + 0x04000000 - \
                                   XIP_CACHE_SIZE + XIP_CACHE_CLEAN_SET_WAY)

/* QSPI pads (SCLK, SD0-SD3, SS) and QMI window 1 registers (TIMING, RFMT,
 * RCMD, WFMT, WCMD).  The bootrom flash functions change both.
 */

#define QSPI_PAD_COUNT            6
#define QMI_M1_REG_COUNT          5

/* True for an address in the XIP space (flash or PSRAM), which cannot be
 * accessed while the QMI is in direct mode.
 */

#define IS_XIP_ADDR(a) \
  ((uintptr_t)(a) >= RP23XX_FLASH_BASE && (uintptr_t)(a) < RP23XX_SRAM_BASE)

/* SRAM stack for a flash operation called with its stack in PSRAM */

#define FLASH_OP_STACK_SIZE       1024

/* Largest flash the XIP window can address, used only to sanity check a
 * pointer before it is called with the flash interface torn down.
 */

#define RP23XX_FLASH_MAX_SIZE     0x04000000

/* Size of the XIP setup function the bootrom leaves at the start of boot
 * RAM (datasheet 5.2.7).
 */

#define XIP_SETUP_WORDS           64

/* Smallest unit that can be programmed, and smallest that can be erased */

#define FLASH_PAGE_SIZE           256
#define FLASH_SECTOR_SIZE         4096

/* Erase command used by the bootrom when a whole 64K block can be erased */

#define FLASH_BLOCK_SIZE          65536
#define FLASH_BLOCK_ERASE_CMD     0xd8

/* JEDEC ID: command, manufacturer, memory type, capacity (log2 bytes) */

#define FLASH_READ_ID_CMD         0x9f
#define FLASH_READ_ID_SIZE        4

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

/* One flash operation.  It is static, so that it is in SRAM. */

struct rp23xx_flash_op_s
{
  CODE void (*func)(FAR struct rp23xx_flash_op_s *op);
  uint32_t addr;
  FAR uint8_t *data;
  size_t count;
};

/* QSPI state saved over a flash operation */

struct rp23xx_qspi_state_s
{
  uint32_t pads[QSPI_PAD_COUNT];
  uint32_t m1[QMI_M1_REG_COUNT];
  uint32_t xip_ctrl;
};

typedef void (*connect_internal_flash_f)(void);
typedef void (*flash_exit_xip_f)(void);
typedef void (*flash_range_erase_f)(uint32_t, size_t, uint32_t, uint8_t);
typedef void (*flash_range_program_f)(uint32_t, const uint8_t *, size_t);
typedef void (*flash_flush_cache_f)(void);
typedef void (*flash_enter_cmd_xip_f)(void);
typedef void (*xip_setup_f)(void);

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

static struct rp23xx_flash_op_s g_flash_op;

/* SRAM copy of a page whose source is in the XIP space */

static uint8_t g_flash_page[FLASH_PAGE_SIZE] aligned_data(4);

#ifdef CONFIG_RP23XX_PSRAM
static uint64_t g_flash_stack[FLASH_OP_STACK_SIZE / 8];
#endif

#ifdef CONFIG_SMP
static struct smp_isolation_s g_smp_isolation;
#endif

static struct
{
  connect_internal_flash_f connect_internal_flash;
  flash_exit_xip_f         flash_exit_xip;
  flash_range_erase_f      flash_range_erase;
  flash_range_program_f    flash_range_program;
  flash_flush_cache_f      flash_flush_cache;
  flash_enter_cmd_xip_f    flash_enter_cmd_xip;

  /* SRAM copy of the bootrom XIP setup function.  It restores the read
   * mode and clock divisor found at boot.  NULL if there is none:
   * flash_enter_cmd_xip then gives a slow 03h serial mode.
   */

  xip_setup_f              xip_setup;
} g_rom;

#ifndef CONFIG_RP23XX_FLASH_MTD_SAFE_XIP
static uint32_t g_xip_setup[XIP_SETUP_WORDS];
#endif

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

          nxsched_smp_call_init(&cpu_data->call_data, pause_cpu_handler,
                                cpu_data);
          nxsched_smp_call_single_async(other_cpuid, &cpu_data->call_data);
        }
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
 * Name: rp23xx_flash_begin
 *
 * Description:
 *   Save the QSPI state and put the flash in serial command mode, as the
 *   Pico SDK does.  Must run from RAM: XIP is not available until
 *   rp23xx_flash_end() returns.
 *
 ****************************************************************************/

static void RAM_CODE(rp23xx_flash_begin)(struct rp23xx_qspi_state_s *state)
{
  int i;

  /* Write back dirty PSRAM lines.  flash_flush_cache() discards them. */

  for (i = 0; i < XIP_CACHE_SIZE; i += XIP_CACHE_LINE_SIZE)
    {
      putreg8(0, XIP_CACHE_CLEAN_BASE + i);
    }

  UP_DSB();
  UP_ISB();

  for (i = 0; i < QSPI_PAD_COUNT; i++)
    {
      state->pads[i] = getreg32(RP23XX_PADS_QSPI_GPIO_QSPI_SCLK + 4 * i);
    }

  for (i = 0; i < QMI_M1_REG_COUNT; i++)
    {
      state->m1[i] = getreg32(RP23XX_QMI_M1_TIMING + 4 * i);
    }

  state->xip_ctrl = getreg32(RP23XX_XIP_CTRL_BASE);

  __asm__ volatile ("" : : : "memory");

  g_rom.connect_internal_flash();
  g_rom.flash_exit_xip();
}

/****************************************************************************
 * Name: rp23xx_flash_end
 *
 * Description:
 *   Put the QSPI interface back into execute-in-place mode, then restore
 *   the pads and the chip select 1 (PSRAM) window that the bootrom reset.
 *
 ****************************************************************************/

static void RAM_CODE(rp23xx_flash_end)(struct rp23xx_qspi_state_s *state)
{
  int i;

  g_rom.flash_flush_cache();

  if (g_rom.xip_setup != NULL)
    {
      g_rom.xip_setup();
    }
  else
    {
      g_rom.flash_enter_cmd_xip();
    }

  for (i = 0; i < QSPI_PAD_COUNT; i++)
    {
      putreg32(state->pads[i], RP23XX_PADS_QSPI_GPIO_QSPI_SCLK + 4 * i);
    }

  for (i = 0; i < QMI_M1_REG_COUNT; i++)
    {
      putreg32(state->m1[i], RP23XX_QMI_M1_TIMING + 4 * i);
    }

  putreg32(getreg32(RP23XX_XIP_CTRL_BASE) |
           (state->xip_ctrl & RP23XX_XIP_CTRL_WRITABLE_M1),
           RP23XX_XIP_CTRL_BASE);
}

/****************************************************************************
 * Name: do_erase
 *
 * Description:
 *   Erase one sector or block.  Runs from RAM with interrupts disabled and
 *   the other core parked.
 *
 ****************************************************************************/

static void RAM_CODE(do_erase)(FAR struct rp23xx_flash_op_s *op)
{
  struct rp23xx_qspi_state_s state;

  rp23xx_flash_begin(&state);
  g_rom.flash_range_erase(op->addr, op->count, FLASH_BLOCK_SIZE,
                          FLASH_BLOCK_ERASE_CMD);
  rp23xx_flash_end(&state);
}

/****************************************************************************
 * Name: do_write
 ****************************************************************************/

static void RAM_CODE(do_write)(FAR struct rp23xx_flash_op_s *op)
{
  struct rp23xx_qspi_state_s state;

  rp23xx_flash_begin(&state);
  g_rom.flash_range_program(op->addr, op->data, op->count);
  rp23xx_flash_end(&state);
}

/****************************************************************************
 * Name: do_read_id
 *
 * Description:
 *   Read the JEDEC ID in QMI direct mode, as the Pico SDK flash_do_cmd()
 *   does.
 *
 ****************************************************************************/

static void RAM_CODE(do_read_id)(FAR struct rp23xx_flash_op_s *op)
{
  struct rp23xx_qspi_state_s state;
  size_t tx = 0;
  size_t rx = 0;
  uint32_t csr;

  rp23xx_flash_begin(&state);

  putreg32(getreg32(RP23XX_QMI_DIRECT_CSR) |
           RP23XX_QMI_DIRECT_CSR_ASSERT_CS0N, RP23XX_QMI_DIRECT_CSR);
  putreg32(getreg32(RP23XX_QMI_DIRECT_CSR) | RP23XX_QMI_DIRECT_CSR_EN,
           RP23XX_QMI_DIRECT_CSR);

  while ((getreg32(RP23XX_QMI_DIRECT_CSR) & RP23XX_QMI_DIRECT_CSR_BUSY) != 0)
    {
    }

  while (tx < op->count || rx < op->count)
    {
      csr = getreg32(RP23XX_QMI_DIRECT_CSR);

      if ((csr & RP23XX_QMI_DIRECT_CSR_TXFULL) == 0 && tx < op->count)
        {
          putreg32(tx == 0 ? FLASH_READ_ID_CMD : 0, RP23XX_QMI_DIRECT_TX);
          tx++;
        }

      if ((csr & RP23XX_QMI_DIRECT_CSR_RXEMPTY) == 0 && rx < op->count)
        {
          op->data[rx++] = (uint8_t)getreg32(RP23XX_QMI_DIRECT_RX);
        }
    }

  /* BUSY stays high for half an SCK after the last bit, for CS timing */

  while ((getreg32(RP23XX_QMI_DIRECT_CSR) & RP23XX_QMI_DIRECT_CSR_BUSY) != 0)
    {
    }

  putreg32(getreg32(RP23XX_QMI_DIRECT_CSR) & ~RP23XX_QMI_DIRECT_CSR_EN,
           RP23XX_QMI_DIRECT_CSR);
  putreg32(getreg32(RP23XX_QMI_DIRECT_CSR) &
           ~RP23XX_QMI_DIRECT_CSR_ASSERT_CS0N, RP23XX_QMI_DIRECT_CSR);

  rp23xx_flash_end(&state);
}

/****************************************************************************
 * Name: rp23xx_flash_call
 *
 * Description:
 *   Call g_flash_op.func.  PSRAM is not accessible during the operation,
 *   so if the stack is in PSRAM, switch to an SRAM stack first.
 *
 ****************************************************************************/

static void rp23xx_flash_call(void)
{
#ifdef CONFIG_RP23XX_PSRAM
  if (IS_XIP_ADDR(up_getsp()))
    {
      __asm__ __volatile__
      (
        "mov r4, sp\n\t"
        "mov sp, %[top]\n\t"
        "mov r0, %[op]\n\t"
        "blx %[func]\n\t"
        "mov sp, r4\n\t"
        :
        : [top] "r" (&g_flash_stack[FLASH_OP_STACK_SIZE / 8]),
          [op] "r" (&g_flash_op),
          [func] "r" (g_flash_op.func)
        : "r0", "r1", "r2", "r3", "r4", "r12", "lr", "memory", "cc"
      );

      return;
    }
#endif

  g_flash_op.func(&g_flash_op);
}

/****************************************************************************
 * Name: rp23xx_flash_run
 *
 * Description:
 *   Run g_flash_op with interrupts disabled and the other core parked.
 *   The caller holds the device lock.
 *
 ****************************************************************************/

static void rp23xx_flash_run(void)
{
  irqstate_t flags;

#ifdef CONFIG_SMP
  init_smp_isolation(&g_smp_isolation);
  enter_smp_isolation(&g_smp_isolation);
#endif

  flags = enter_critical_section();
  rp23xx_flash_call();
  leave_critical_section(flags);

#ifdef CONFIG_SMP
  leave_smp_isolation(&g_smp_isolation);
#endif
}

/****************************************************************************
 * Name: rp23xx_flash_size
 *
 * Description:
 *   Return the flash size from its JEDEC ID, or 0 if the ID is not valid.
 *
 ****************************************************************************/

static size_t rp23xx_flash_size(void)
{
  FAR uint8_t *id = g_flash_page;

  if (nxmutex_lock(&g_flash_dev.lock) < 0)
    {
      return 0;
    }

  g_flash_op.func  = do_read_id;
  g_flash_op.data  = id;
  g_flash_op.count = FLASH_READ_ID_SIZE;

  rp23xx_flash_run();
  nxmutex_unlock(&g_flash_dev.lock);

  finfo("rp23xx_flash: JEDEC ID %02x %02x %02x\n", id[1], id[2], id[3]);

  /* Accept a capacity from 64K to 64M */

  if (id[1] == 0x00 || id[1] == 0xff || id[3] < 16 || id[3] > 26)
    {
      return 0;
    }

  return (size_t)1 << id[3];
}

/****************************************************************************
 * Name: rp23xx_flash_erase
 *
 * Description:
 *   Erase one 64K block or 4K sector at a time, so that interrupts are
 *   disabled for one block erase at most.
 *
 ****************************************************************************/

static int rp23xx_flash_erase(struct mtd_dev_s *dev, off_t startblock,
                              size_t nblocks)
{
  struct rp23xx_flash_dev_s *priv = (struct rp23xx_flash_dev_s *)dev;
  uint32_t addr;
  uint32_t end;
  int ret;

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

  addr = FS_OFFSET + startblock * FLASH_SECTOR_SIZE;
  end  = addr + nblocks * FLASH_SECTOR_SIZE;

  while (addr < end)
    {
      g_flash_op.func  = do_erase;
      g_flash_op.addr  = addr;
      g_flash_op.count = FLASH_SECTOR_SIZE;

      if ((addr % FLASH_BLOCK_SIZE) == 0 && end - addr >= FLASH_BLOCK_SIZE)
        {
          g_flash_op.count = FLASH_BLOCK_SIZE;
        }

      rp23xx_flash_run();
      addr += g_flash_op.count;
    }

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
 *
 * Description:
 *   Program one page at a time, so that interrupts are disabled for one
 *   page program at most.  Copy a page from flash or PSRAM to SRAM first.
 *
 ****************************************************************************/

static ssize_t rp23xx_flash_bwrite(struct mtd_dev_s *dev, off_t startblock,
                                   size_t nblocks, const uint8_t *buffer)
{
  struct rp23xx_flash_dev_s *priv = (struct rp23xx_flash_dev_s *)dev;
  size_t i;
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

  for (i = 0; i < nblocks; i++)
    {
      g_flash_op.func  = do_write;
      g_flash_op.addr  = FS_OFFSET + (startblock + i) * FLASH_PAGE_SIZE;
      g_flash_op.data  = (FAR uint8_t *)buffer + i * FLASH_PAGE_SIZE;
      g_flash_op.count = FLASH_PAGE_SIZE;

      if (IS_XIP_ADDR(g_flash_op.data))
        {
          memcpy(g_flash_page, g_flash_op.data, FLASH_PAGE_SIZE);
          g_flash_op.data = g_flash_page;
        }

      rp23xx_flash_run();
    }

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
  size_t size;
#ifndef CONFIG_RP23XX_FLASH_MTD_SAFE_XIP
  int i;
#endif

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

  /* Copy the bootrom XIP setup function out of boot RAM, which is not
   * executable, as the Pico SDK does.  Boot RAM is empty when the image
   * was not started by a flash boot.
   */

#ifndef CONFIG_RP23XX_FLASH_MTD_SAFE_XIP
  for (i = 0; i < XIP_SETUP_WORDS; i++)
    {
      g_xip_setup[i] = getreg32(RP23XX_BOOTRAM_BASE + 4 * i);
    }

  UP_DSB();
  UP_ISB();

  if (g_xip_setup[0] != 0)
    {
      g_rom.xip_setup = (xip_setup_f)((uintptr_t)g_xip_setup | 1);
    }
  else
    {
      fwarn("rp23xx_flash: no XIP setup function in boot RAM; reads will "
            "be slow after every flash operation\n");
    }
#endif

  /* An address past the end of the flash wraps around to the start, where
   * the NuttX image is.  Refuse a region that does not fit.
   */

  size = rp23xx_flash_size();
  if (size == 0)
    {
      fwarn("rp23xx_flash: unknown flash size; not checked\n");
    }
  else if (FS_OFFSET + FS_SIZE > size)
    {
      merr("ERROR: flash MTD region ends at 0x%08x, past the end of the "
           "%zu byte flash\n", (unsigned)(FS_OFFSET + FS_SIZE), size);
      set_errno(EINVAL);
      return NULL;
    }

  g_initialized = true;

  finfo("rp23xx_flash: %u sectors of %u bytes at 0x%08x\n",
        (unsigned)FS_SECTORS, (unsigned)FLASH_SECTOR_SIZE,
        (unsigned)(RP23XX_FLASH_BASE + FS_OFFSET));

  return &g_flash_dev.mtd;
}
