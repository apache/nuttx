/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_spiflash.c
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

#include <stdint.h>
#include <assert.h>
#include <debug.h>
#include <string.h>
#include <sys/types.h>
#include <sys/param.h>
#include <inttypes.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/init.h>
#include <nuttx/kthread.h>
#include <nuttx/signal.h>

#include "sched/sched.h"

#include "xtensa.h"
#include "esp_attr.h"
#include "hardware/esp32s3_efuse.h"
#include "hardware/esp32s3_cache_memory.h"
#include "rom/esp32s3_spiflash.h"
#include "esp32s3_irq.h"
#include "esp32s3_spiflash.h"

#include "soc/extmem_reg.h"
#include "soc/spi_mem_reg.h"
#include "rom/opi_flash.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RO data page in MMU index */

#define DROM0_PAGES_START           (2)
#define DROM0_PAGES_END             (512)

/* MMU base virtual mapped address */

#define VADDR0_START_ADDR           (0x3c020000)

#define VADDR1_START_ADDR           (0x42000000)

/* Flash MMU table for CPU */

#define MMU_TABLE                   ((volatile uint32_t *)DR_REG_MMU_TABLE)

#define MMU_ADDR2PAGE(_addr)        ((_addr) / MMU_PAGE_SIZE)
#define MMU_ADDR2OFF(_addr)         ((_addr) % MMU_PAGE_SIZE)
#define MMU_BYTES2PAGES(_n)         (((_n) + MMU_PAGE_SIZE - 1) / \
                                     MMU_PAGE_SIZE)

#ifdef CONFIG_ESP32S3_SPI_FLASH_DONT_USE_ROM_CODE

#define g_rom_flashchip             (rom_spiflash_legacy_data->chip)

#define MMU_ALIGNUP_SIZE(_s)        (((_s) + MMU_PAGE_SIZE - 1) \
                                     & ~(MMU_PAGE_SIZE - 1))
#define MMU_ALIGNDOWN_SIZE(_s)      ((_s) & ~(MMU_PAGE_SIZE - 1))

/* Flash MMU table for APP CPU */

#define PRO_IRAM0_FIRST_PAGE        (0)
#define IROM0_PAGES_END             (2)

/* SPI port number */

#  define SPI_PORT                  (1)

/* SPI buffer size */

#  define SPI_BUFFER_WORDS          (16)
#  define SPI_BUFFER_BYTES          (SPI_BUFFER_WORDS * 4)

/* SPI flash hardware definition */

#  define FLASH_PAGE_SIZE           (256)
#  define FLASH_SECTOR_SIZE         (4096)

/* SPI flash command */

#  define FLASH_CMD_WRDI            ROM_FLASH_CMD_WRDI
#  define FLASH_CMD_WREN            ROM_FLASH_CMD_WREN
#  define FLASH_CMD_RDSR            ROM_FLASH_CMD_RDSR
#  define FLASH_CMD_SE4B            ROM_FLASH_CMD_SE4B
#  define FLASH_CMD_SE              ROM_FLASH_CMD_ERASE_SEC
#  define FLASH_CMD_PP4B            ROM_FLASH_CMD_PP4B
#  define FLASH_CMD_PP              0x02
#  define FLASH_CMD_FSTRD4B         ROM_FLASH_CMD_FSTRD4B
#  define FLASH_CMD_FSTRD           0x0B

/* SPI flash SR1 bits */

#  define FLASH_SR1_BUSY            ESP_ROM_SPIFLASH_BUSY_FLAG
#  define FLASH_SR1_WREN            ESP_ROM_SPIFLASH_WRENABLE_FLAG

/* SPI flash operation */

#  ifdef CONFIG_ESP32S3_SPI_FLASH_USE_32BIT_ADDRESS
#    define ADDR_BITS(addr)         (((addr) & 0xff000000) ? 32 : 24)
#    define READ_CMD(addr)          (ADDR_BITS(addr) == 32 ? FLASH_CMD_FSTRD4B : \
                                                             FLASH_CMD_FSTRD)
#    define WRITE_CMD(addr)         (ADDR_BITS(addr) == 32 ? FLASH_CMD_PP4B : \
                                                             FLASH_CMD_PP)
#    define ERASE_CMD(addr)         (ADDR_BITS(addr) == 32 ? FLASH_CMD_SE4B : \
                                                             FLASH_CMD_SE)
#    define READ_DUMMY(addr)        (8)
#  else
#    define ADDR_BITS(addr)         24
#    define READ_CMD(addr)          FLASH_CMD_FSTRD
#    define WRITE_CMD(addr)         FLASH_CMD_PP
#    define ERASE_CMD(addr)         FLASH_CMD_SE
#    define READ_DUMMY(addr)        (8)
#  endif

#  define SEND_CMD8_TO_FLASH(cmd)                           \
    esp32s3_spi_trans((cmd), 8,                             \
                      0, 0,                                 \
                      NULL, 0,                              \
                      NULL, 0,                              \
                      0,                                    \
                      false)

#  define READ_SR1_FROM_FLASH(cmd, status)                  \
    esp32s3_spi_trans((cmd), 8,                             \
                      0, 0,                                 \
                      NULL, 0,                              \
                      (status), 1,                          \
                      0,                                    \
                      false)

#  define ERASE_FLASH_SECTOR(addr)                          \
    esp32s3_spi_trans(ERASE_CMD(addr), 8,                   \
                      (addr), ADDR_BITS(addr),              \
                      NULL, 0,                              \
                      NULL, 0,                              \
                      0,                                    \
                      true)

#  define WRITE_DATA_TO_FLASH(addr, buffer, size)           \
    esp32s3_spi_trans(WRITE_CMD(addr), 8,                   \
                      (addr), ADDR_BITS(addr),              \
                      buffer, size,                         \
                      NULL, 0,                              \
                      0,                                    \
                      true)

#  define READ_DATA_FROM_FLASH(addr, buffer, size)          \
    esp32s3_spi_trans(READ_CMD(addr), 8,                    \
                      (addr), ADDR_BITS(addr),              \
                      NULL, 0,                              \
                      buffer, size,                         \
                      READ_DUMMY(addr),                     \
                      false)

#endif /* CONFIG_ESP32S3_SPI_FLASH_DONT_USE_ROM_CODE */

/****************************************************************************
 * Private Functions Declaration
 ****************************************************************************/

static void spiflash_start(void);
static void spiflash_end(void);
static void spi_flash_disable_cache(uint32_t cpuid);
static void spi_flash_restore_cache(uint32_t cpuid);
#ifdef CONFIG_SMP
static int spi_flash_op_block_task(int argc, char *argv[]);
static int spiflash_init_spi_flash_op_block_task(int cpu);
#endif

/****************************************************************************
 * Public Functions Declaration
 ****************************************************************************/

extern uint32_t cache_suspend_icache(void);
extern uint32_t cache_suspend_dcache(void);
extern void cache_resume_icache(uint32_t val);
extern void cache_resume_dcache(uint32_t val);
extern int cache_invalidate_addr(uint32_t addr, uint32_t size);
extern void cache_invalidate_icache_all(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static spi_flash_guard_funcs_t g_spi_flash_guard_funcs =
{
  .start           = spiflash_start,
  .end             = spiflash_end,
};

static uint32_t s_flash_op_cache_state[CONFIG_SMP_NCPUS];

static rmutex_t g_flash_op_mutex;
static volatile bool g_flash_op_can_start = false;
static volatile bool g_flash_op_complete = false;
static volatile bool g_sched_suspended[CONFIG_SMP_NCPUS];
#ifdef CONFIG_SMP
static sem_t g_disable_non_iram_isr_on_core[CONFIG_SMP_NCPUS];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spiflash_suspend_cache
 *
 * Description:
 *   Suspend CPU cache.
 *
 ****************************************************************************/

static void spiflash_suspend_cache(void)
{
  int cpu = up_cpu_index();
#ifdef CONFIG_SMP
  int other_cpu = cpu ? 0 : 1;
#endif

  spi_flash_disable_cache(cpu);
#ifdef CONFIG_SMP
  spi_flash_disable_cache(other_cpu);
#endif
}

/****************************************************************************
 * Name: spiflash_resume_cache
 *
 * Description:
 *   Resume CPU cache.
 *
 ****************************************************************************/

static void spiflash_resume_cache(void)
{
  int cpu = up_cpu_index();
#ifdef CONFIG_SMP
  int other_cpu = cpu ? 0 : 1;
#endif

  spi_flash_restore_cache(cpu);
#ifdef CONFIG_SMP
  spi_flash_restore_cache(other_cpu);
#endif
}

/****************************************************************************
 * Name: spiflash_start
 *
 * Description:
 *   Prepare for an SPI flash operation.
 *
 ****************************************************************************/

static void spiflash_start(void)
{
  struct tcb_s *tcb = this_task();
  int saved_priority = tcb->sched_priority;
  int cpu;
#ifdef CONFIG_SMP
  int other_cpu;
#endif

  nxrmutex_lock(&g_flash_op_mutex);

  /* Temporary raise schedule priority */

  nxsched_set_priority(tcb, SCHED_PRIORITY_MAX);

  cpu = up_cpu_index();
#ifdef CONFIG_SMP
  other_cpu = cpu == 1 ? 0 : 1;
#endif

  DEBUGASSERT(cpu == 0 || cpu == 1);

#ifdef CONFIG_SMP
  DEBUGASSERT(other_cpu == 0 || other_cpu == 1);
  DEBUGASSERT(other_cpu != cpu);
  if (OSINIT_OS_READY())
    {
      g_flash_op_can_start = false;

      cpu = up_cpu_index();
      other_cpu = cpu ? 0 : 1;

      nxsem_post(&g_disable_non_iram_isr_on_core[other_cpu]);

      while (!g_flash_op_can_start)
        {
          /* Busy loop and wait for spi_flash_op_block_task to disable cache
           * on the other CPU
           */
        }
    }
#endif

  g_sched_suspended[cpu] = true;

  sched_lock();

  nxsched_set_priority(tcb, saved_priority);

  esp32s3_irq_noniram_disable();

  spiflash_suspend_cache();
}

/****************************************************************************
 * Name: spiflash_end
 *
 * Description:
 *   Undo all the steps of spiflash_start.
 *
 ****************************************************************************/

static void spiflash_end(void)
{
  const int cpu = up_cpu_index();
#ifdef CONFIG_SMP
  const int other_cpu = cpu ? 0 : 1;
#endif

  DEBUGASSERT(cpu == 0 || cpu == 1);

#ifdef CONFIG_SMP
  DEBUGASSERT(other_cpu == 0 || other_cpu == 1);
  DEBUGASSERT(other_cpu != cpu);
#endif

  cache_invalidate_icache_all();
  spiflash_resume_cache();

  /* Signal to spi_flash_op_block_task that flash operation is complete */

  g_flash_op_complete = true;

  esp32s3_irq_noniram_enable();

  sched_unlock();

  g_sched_suspended[cpu] = false;

#ifdef CONFIG_SMP
  while (g_sched_suspended[other_cpu])
    {
      /* Busy loop and wait for spi_flash_op_block_task to properly finish
       * and resume scheduler
       */
    }
#endif

  nxrmutex_unlock(&g_flash_op_mutex);
}

/****************************************************************************
 * Name: esp32s3_spi_trans
 *
 * Description:
 *   Transmit given command, address and data.
 *
 * Input Parameters:
 *   command      - command value
 *   command_bits - command bits
 *   address      - address value
 *   address_bits - address bits
 *   tx_buffer    - write buffer
 *   tx_bytes     - write buffer size
 *   rx_buffer    - read buffer
 *   rx_bytes     - read buffer size
 *   dummy_bits   - dummy bits
 *   is_program   - true if operation is program or erase
 *
 * Returned Value:
 *   0 if success or a negative value if fail.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_SPI_FLASH_DONT_USE_ROM_CODE
static void esp32s3_spi_trans(uint32_t command,
                              uint32_t command_bits,
                              uint32_t address,
                              uint32_t address_bits,
                              uint32_t *tx_buffer,
                              uint32_t tx_bytes,
                              uint32_t *rx_buffer,
                              uint32_t rx_bytes,
                              uint32_t dummy_bits,
                              bool is_program)
{
  uint32_t regval;
  uint32_t cmd_reg;
  uint32_t user1_reg = getreg32(SPI_MEM_USER1_REG(SPI_PORT));
  uint32_t user_reg = getreg32(SPI_MEM_USER_REG(SPI_PORT));

  /* Initiliaze SPI user register */

  user_reg &= ~(SPI_MEM_USR_ADDR_M | SPI_MEM_USR_DUMMY_M |
                SPI_MEM_USR_MOSI_M | SPI_MEM_USR_MISO_M);
  user_reg |= SPI_MEM_USR_COMMAND_M;

  /* Wait until SPI is idle */

  do
    {
      cmd_reg = getreg32(SPI_MEM_CMD_REG(SPI_PORT));
    }
  while ((cmd_reg & SPI_MEM_USR_M) != 0);

  /* Set command bits and value, and command is always needed */

  regval  = getreg32(SPI_MEM_USER2_REG(SPI_PORT));
  regval &= ~(SPI_MEM_USR_COMMAND_BITLEN_M | SPI_MEM_USR_COMMAND_VALUE_M);
  regval |= ((command_bits - 1) << SPI_MEM_USR_COMMAND_BITLEN_S) |
            (command << SPI_MEM_USR_COMMAND_VALUE_S);
  putreg32(regval, SPI_MEM_USER2_REG(SPI_PORT));

  /* Set address bits and value */

  if (address_bits)
    {
      user1_reg &= ~SPI_MEM_USR_ADDR_BITLEN_M;
      user1_reg |= (address_bits - 1) << SPI_MEM_USR_ADDR_BITLEN_S;

      putreg32(address, SPI_MEM_ADDR_REG(SPI_PORT));

      user_reg |= SPI_MEM_USR_ADDR_M;

      regval  = getreg32(SPI_MEM_CACHE_FCTRL_REG(SPI_PORT));
      if (address_bits > 24)
        {
          regval |= SPI_MEM_CACHE_USR_CMD_4BYTE_M;
        }
      else
        {
          regval &= ~SPI_MEM_CACHE_USR_CMD_4BYTE_M;
        }

      putreg32(regval, SPI_MEM_CACHE_FCTRL_REG(SPI_PORT));
    }

  /* Set dummy */

  if (dummy_bits)
    {
      user1_reg &= ~SPI_MEM_USR_DUMMY_CYCLELEN_M;
      user1_reg |= (dummy_bits - 1) << SPI_MEM_USR_DUMMY_CYCLELEN_S;

      user_reg |= SPI_MEM_USR_DUMMY_M;
    }

  /* Set TX data */

  if (tx_bytes)
    {
      putreg32(tx_bytes * 8 - 1, SPI_MEM_MOSI_DLEN_REG(SPI_PORT));
      for (uint32_t i = 0; i < tx_bytes; i += 4)
        {
          putreg32(tx_buffer[i / 4], SPI_MEM_W0_REG(SPI_PORT) + i);
        }

      user_reg |= SPI_MEM_USR_MOSI_M;
    }

  /* Set RX data */

  if (rx_bytes)
    {
      putreg32(rx_bytes * 8 - 1, SPI_MEM_MISO_DLEN_REG(SPI_PORT));

      user_reg |= SPI_MEM_USR_MISO_M;
    }

  putreg32(user_reg,  SPI_MEM_USER_REG(SPI_PORT));
  putreg32(user1_reg, SPI_MEM_USER1_REG(SPI_PORT));

  /* Set I/O mode */

  regval = getreg32(SPI_MEM_CTRL_REG(SPI_PORT));
  regval &= ~(SPI_MEM_FREAD_QIO_M | SPI_MEM_FREAD_DIO_M |
              SPI_MEM_FREAD_QUAD_M | SPI_MEM_FREAD_DUAL_M |
              SPI_MEM_FCMD_OCT_M | SPI_MEM_FCMD_QUAD_M |
              SPI_MEM_FCMD_DUAL_M | SPI_MEM_FADDR_OCT_M |
              SPI_MEM_FDIN_OCT_M | SPI_MEM_FDOUT_OCT_M |
              SPI_MEM_FDUMMY_OUT_M | SPI_MEM_RESANDRES_M |
              SPI_MEM_WP_REG_M | SPI_MEM_WRSR_2B_M);
  regval |= SPI_MEM_FASTRD_MODE_M;
  putreg32(regval, SPI_MEM_CTRL_REG(SPI_PORT));

  /* Set clock and delay */

  regval = SPI_MEM_FLASH_PES_WAIT_EN_M |
           SPI_MEM_FLASH_PER_WAIT_EN_M;
  putreg32(regval, SPI_MEM_FLASH_SUS_CMD_REG(SPI_PORT));
  putreg32(0, SPI_MEM_CLOCK_GATE_REG(SPI_PORT));

  /* Set if this is program or erase operation */

  if (is_program)
    {
      cmd_reg |= SPI_MEM_FLASH_PE_M;
    }

  /* Start transmision */

  cmd_reg |= SPI_MEM_USR_M;
  putreg32(cmd_reg, SPI_MEM_CMD_REG(SPI_PORT));

  /* Wait until transmission is done */

  while ((getreg32(SPI_MEM_CMD_REG(SPI_PORT)) & SPI_MEM_USR_M) != 0)
    {
      ;
    }

  /* Get read data */

  if (rx_bytes)
    {
      for (uint32_t i = 0; i < rx_bytes; i += 4)
        {
          rx_buffer[i / 4] = getreg32(SPI_MEM_W0_REG(SPI_PORT) + i);
        }
    }
}

/****************************************************************************
 * Name: wait_flash_idle
 *
 * Description:
 *   Wait until flash enters idle state
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void wait_flash_idle(void)
{
  uint32_t status;

  do
    {
      READ_SR1_FROM_FLASH(FLASH_CMD_RDSR, &status);
      if ((status & FLASH_SR1_BUSY) == 0)
        {
          break;
        }
    }
  while (1);
}

/****************************************************************************
 * Name: enable_flash_write
 *
 * Description:
 *   Enable Flash write mode
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void enable_flash_write(void)
{
  uint32_t status;

  do
    {
      SEND_CMD8_TO_FLASH(FLASH_CMD_WREN);
      READ_SR1_FROM_FLASH(FLASH_CMD_RDSR, &status);
      if ((status & FLASH_SR1_WREN) != 0)
        {
          break;
        }
    }
  while (1);
}

/****************************************************************************
 * Name: disable_flash_write
 *
 * Description:
 *   Disable Flash write mode
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void disable_flash_write(void)
{
  uint32_t status;

  do
    {
      SEND_CMD8_TO_FLASH(FLASH_CMD_WRDI);
      READ_SR1_FROM_FLASH(FLASH_CMD_RDSR, &status);
      if ((status & FLASH_SR1_WREN) == 0)
        {
          break;
        }
    }
  while (1);
}

/****************************************************************************
 * Name: spiflash_pagecached
 *
 * Description:
 *   Check if the given page is cached.
 *
 * Input Parameters:
 *   phypage - physical address page.
 *   ptr     - Pointer to the virtual address.
 *
 * Returned Value:
 *   True if flash address has corresponding cache mapping, false otherwise.
 *
 ****************************************************************************/

static bool IRAM_ATTR spiflash_pagecached(uint32_t phypage, uint32_t *ptr)
{
  int start[2];
  int end[2];
  int i;
  int j;

  /* Data ROM start and end pages */

  start[0] = DROM0_PAGES_START;
  end[0]   = DROM0_PAGES_END;

  /* Instruction RAM start and end pages */

  start[1] = PRO_IRAM0_FIRST_PAGE;
  end[1]   = IROM0_PAGES_END;

  for (i = 0; i < 2; i++)
    {
      for (j = start[i]; j < end[i]; j++)
        {
          if (MMU_TABLE[j] == phypage)
            {
              if (i == 0)
                {
                  /* SPI_FLASH_MMAP_DATA */

                  *ptr = (VADDR0_START_ADDR +
                          MMU_PAGE_SIZE * (j - start[0]));
                }
              else
                {
                  /* SPI_FLASH_MMAP_INST */

                  *ptr = (VADDR1_START_ADDR +
                          MMU_PAGE_SIZE * (j - start[1]));
                }

              return true;
            }
        }
    }

  return false;
}

/****************************************************************************
 * Name: spiflash_flushmapped
 *
 * Description:
 *   Writeback PSRAM data and invalidate the cache if the address is mapped.
 *
 * Input Parameters:
 *   start - SPI Flash address.
 *   size  - SPI Flash size.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void IRAM_ATTR spiflash_flushmapped(size_t start, size_t size)
{
  uint32_t page_start;
  uint32_t addr;
  uint32_t page;
  uint32_t vaddr;

  page_start = MMU_ALIGNDOWN_SIZE(start);
  size += (start - page_start);
  size = MMU_ALIGNUP_SIZE(size);

  for (addr = page_start; addr < page_start + size;
       addr += MMU_PAGE_SIZE)
    {
      page = addr / MMU_PAGE_SIZE;
      if (addr >= g_rom_flashchip.chip_size)
        {
          return;
        }

      if (spiflash_pagecached(page, &vaddr))
        {
          cache_invalidate_addr(vaddr, MMU_PAGE_SIZE);
        }
    }
}

/****************************************************************************
 * Name: spiflash_os_yield
 *
 * Description:
 *   Yield to other tasks, called during erase operations.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void IRAM_ATTR spiflash_os_yield(void)
{
  /* Delay 1 tick */

  useconds_t us = TICK2USEC(1);
  nxsig_usleep(us);
}
#endif /* CONFIG_ESP32S3_SPI_FLASH_DONT_USE_ROM_CODE */

/****************************************************************************
 * Name: spi_flash_disable_cache
 *
 * Description:
 *   Disable the I/D cache of a CPU core and save its status value on
 *   s_flash_op_cache_state.
 *
 * Input Parameters:
 *   cpuid - The CPU core whose cache will be disabled.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void spi_flash_disable_cache(uint32_t cpuid)
{
  s_flash_op_cache_state[cpuid] = cache_suspend_icache() << 16;
  s_flash_op_cache_state[cpuid] |= cache_suspend_dcache();
}

/****************************************************************************
 * Name: spi_flash_restore_cache
 *
 * Description:
 *   Restore the I/D cache of a CPU core using the saved status value on
 *   s_flash_op_cache_state.
 *
 * Input Parameters:
 *   cpuid - The CPU core whose cache will be restored.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void spi_flash_restore_cache(uint32_t cpuid)
{
  cache_resume_dcache(s_flash_op_cache_state[cpuid] & 0xffff);
  cache_resume_icache(s_flash_op_cache_state[cpuid] >> 16);
}

#ifdef CONFIG_SMP

/****************************************************************************
 * Name: spi_flash_op_block_task
 *
 * Description:
 *   Disable the non-IRAM interrupts on the other core (the one that isn't
 *   handling the SPI flash operation) and notify that the SPI flash
 *   operation can start. Wait on a busy loop until it's finished and then
 *   reenable the non-IRAM interrups.
 *
 * Input Parameters:
 *   argc          - Not used.
 *   argv          - Not used.
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned to
 *   indicate the nature of any failure.
 *
 ****************************************************************************/

static int spi_flash_op_block_task(int argc, char *argv[])
{
  struct tcb_s *tcb = this_task();
  int cpu = up_cpu_index();

  for (; ; )
    {
      DEBUGASSERT((1 << cpu) & tcb->affinity);
      /* Wait for a SPI flash operation to take place and this (the other
       * core) being asked to disable its non-IRAM interrupts.
       */

      nxsem_wait(&g_disable_non_iram_isr_on_core[cpu]);

      sched_lock();

      esp32s3_irq_noniram_disable();

      /* g_flash_op_complete flag is cleared on *this* CPU, otherwise the
       * other CPU may reset the flag back to false before this task has a
       * chance to check it (if it's preempted by an ISR taking non-trivial
       * amount of time).
       */

      g_flash_op_complete = false;
      g_flash_op_can_start = true;
      while (!g_flash_op_complete)
        {
          /* Busy loop here and wait for the other CPU to finish the SPI
           * flash operation.
           */
        }

      /* Flash operation is complete, re-enable cache */

      spi_flash_restore_cache(cpu);

      /* Restore interrupts that aren't located in IRAM */

      esp32s3_irq_noniram_enable();

      sched_unlock();
    }

  return OK;
}

/****************************************************************************
 * Name: spiflash_init_spi_flash_op_block_task
 *
 * Description:
 *   Starts a kernel thread that waits for a semaphore indicating that a SPI
 *   flash operation is going to take place in the other CPU. It disables
 *   non-IRAM interrupts, indicates to the other core that the SPI flash
 *   operation can start and waits for it to be finished in a busy loop.
 *
 * Input Parameters:
 *   cpu - The CPU core that will run the created task to wait on a busy
 *         loop while the SPI flash operation finishes
 *
 * Returned Value:
 *   0 (OK) on success; A negated errno value on failure.
 *
 ****************************************************************************/

static int spiflash_init_spi_flash_op_block_task(int cpu)
{
  int pid;
  int ret = OK;
  char *argv[2];
  char arg1[32];
  cpu_set_t cpuset;

  snprintf(arg1, sizeof(arg1), "%p", &cpu);
  argv[0] = arg1;
  argv[1] = NULL;

  pid = kthread_create("spiflash_op",
                       SCHED_PRIORITY_MAX,
                       CONFIG_ESP32S3_SPIFLASH_OP_TASK_STACKSIZE,
                       spi_flash_op_block_task,
                       argv);
  if (pid > 0)
    {
      if (cpu < CONFIG_SMP_NCPUS)
        {
          CPU_ZERO(&cpuset);
          CPU_SET(cpu, &cpuset);
          ret = nxsched_set_affinity(pid, sizeof(cpuset), &cpuset);
          if (ret < 0)
            {
              return ret;
            }
        }
    }
  else
    {
      return -EPERM;
    }

  return ret;
}
#endif /* CONFIG_SMP */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_mmap
 *
 * Description:
 *   Mapped SPI Flash address to ESP32-S3's address bus, so that software
 *   can read SPI Flash data by reading data from memory access.
 *
 *   If SPI Flash hardware encryption is enable, the read from mapped
 *   address is decrypted.
 *
 * Input Parameters:
 *   req - SPI Flash mapping requesting parameters
 *
 * Returned Value:
 *   0 if success or a negative value if fail.
 *
 ****************************************************************************/

int esp32s3_mmap(struct spiflash_map_req_s *req)
{
  int ret;
  int i;
  int start_page;
  int flash_page;
  int page_cnt;
  uint32_t mapped_addr;

  spiflash_start();

  for (start_page = DROM0_PAGES_START;
       start_page < DROM0_PAGES_END;
       ++start_page)
    {
      if (MMU_TABLE[start_page] == MMU_INVALID)
        {
          break;
        }
    }

  flash_page = MMU_ADDR2PAGE(req->src_addr);
  page_cnt   = MMU_BYTES2PAGES(MMU_ADDR2OFF(req->src_addr) + req->size);

  if (start_page + page_cnt < DROM0_PAGES_END)
    {
      mapped_addr = (start_page - DROM0_PAGES_START) *
                    MMU_PAGE_SIZE +
                    VADDR0_START_ADDR;

      for (i = 0; i < page_cnt; i++)
        {
          MMU_TABLE[start_page + i] = flash_page + i;
        }

      req->start_page = start_page;
      req->page_cnt = page_cnt;
      req->ptr = (void *)(mapped_addr + MMU_ADDR2OFF(req->src_addr));
      ret = OK;
      int regval = getreg32(EXTMEM_DCACHE_CTRL1_REG);
      regval &= ~EXTMEM_DCACHE_SHUT_CORE0_BUS;
      putreg32(regval, EXTMEM_DCACHE_CTRL1_REG);

#if defined(CONFIG_SMP)
      regval = getreg32(EXTMEM_DCACHE_CTRL1_REG);
      regval &= ~EXTMEM_DCACHE_SHUT_CORE1_BUS;
      putreg32(regval, EXTMEM_DCACHE_CTRL1_REG);
#endif
      cache_invalidate_addr(mapped_addr, page_cnt * MMU_PAGE_SIZE);
    }
  else
    {
      ret = -ENOBUFS;
    }

  spiflash_end();

  return ret;
}

/****************************************************************************
 * Name: esp32s3_ummap
 *
 * Description:
 *   Unmap SPI Flash address in ESP32-S3's address bus, and free resource.
 *
 * Input Parameters:
 *   req - SPI Flash mapping requesting parameters
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32s3_ummap(const struct spiflash_map_req_s *req)
{
  int i;

  spiflash_start();

  for (i = req->start_page; i < req->start_page + req->page_cnt; ++i)
    {
      MMU_TABLE[i] = MMU_INVALID;
    }

  spiflash_end();
}

/****************************************************************************
 * Name: spi_flash_read_encrypted
 *
 * Description:
 *   Read decrypted data from SPI Flash at designated address when
 *   enable SPI Flash hardware encryption.
 *
 * Input Parameters:
 *   addr   - target address
 *   buffer - data buffer pointer
 *   size   - data number
 *
 * Returned Value:
 *   OK if success or a negative value if fail.
 *
 ****************************************************************************/

int spi_flash_read_encrypted(uint32_t addr, void *buffer, uint32_t size)
{
  int ret;
  struct spiflash_map_req_s req =
    {
      .src_addr = addr,
      .size = size
    };

  ret = esp32s3_mmap(&req);
  if (ret < 0)
    {
      return ret;
    }

  memcpy(buffer, req.ptr, size);

  esp32s3_ummap(&req);

  return OK;
}

/****************************************************************************
 * Name: spi_flash_erase_sector
 *
 * Description:
 *   Erase the Flash sector.
 *
 * Parameters:
 *   sector - Sector number, the count starts at sector 0, 4KB per sector.
 *
 * Returned Values: esp_err_t
 *   Zero (OK) is returned or a negative error.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_SPI_FLASH_DONT_USE_ROM_CODE
int spi_flash_erase_sector(uint32_t sector)
{
  int ret = OK;
  uint32_t addr = sector * FLASH_SECTOR_SIZE;

  spiflash_start();

  wait_flash_idle();
  enable_flash_write();

  ERASE_FLASH_SECTOR(addr);

  wait_flash_idle();
  disable_flash_write();
  spiflash_flushmapped(addr, FLASH_SECTOR_SIZE);
  spiflash_end();

  return ret;
}

/****************************************************************************
 * Name: spi_flash_erase_range
 *
 * Description:
 *   Erase a range of flash sectors
 *
 * Parameters:
 *   start_address - Address where erase operation has to start.
 *                   Must be 4kB-aligned
 *   size          - Size of erased range, in bytes. Must be divisible by
 *                   4kB.
 *
 * Returned Values:
 *   Zero (OK) is returned or a negative error.
 *
 ****************************************************************************/

int spi_flash_erase_range(uint32_t start_address, uint32_t size)
{
  int ret = OK;
  uint32_t addr = start_address;

  for (uint32_t i = 0; i < size; i += FLASH_SECTOR_SIZE)
    {
      if (i > 0)
        {
          spiflash_os_yield();
        }

      spiflash_start();
      wait_flash_idle();
      enable_flash_write();

      ERASE_FLASH_SECTOR(addr);
      addr += FLASH_SECTOR_SIZE;
      wait_flash_idle();
      disable_flash_write();
      spiflash_end();
    }

  spiflash_start();
  spiflash_flushmapped(start_address, FLASH_SECTOR_SIZE * size);
  spiflash_end();

  return ret;
}

/****************************************************************************
 * Name: spi_flash_write
 *
 * Description:
 *   Write data to Flash.
 *
 * Parameters:
 *   dest_addr - Destination address in Flash.
 *   src       - Pointer to the source buffer.
 *   size      - Length of data, in bytes.
 *
 * Returned Values:
 *   Zero (OK) is returned or a negative error.
 *
 ****************************************************************************/

int spi_flash_write(uint32_t dest_addr, const void *buffer, uint32_t size)
{
  int ret = OK;
  const uint8_t *tx_buf = (const uint8_t *)buffer;
  uint32_t tx_bytes = size;
  uint32_t tx_addr = dest_addr;
#ifdef CONFIG_ESP32S3_SPIRAM
  bool buffer_in_psram = esp32s3_ptr_extram(buffer);
#endif

  spiflash_start();

  while (tx_bytes)
    {
      uint32_t spi_buffer[SPI_BUFFER_WORDS];
      uint32_t n = FLASH_PAGE_SIZE - tx_addr % FLASH_PAGE_SIZE;
      n = MIN(n, MIN(tx_bytes, SPI_BUFFER_BYTES));

#ifdef CONFIG_ESP32S3_SPIRAM

      /* Re-enable cache, and then copy data from PSRAM to SRAM */

      if (buffer_in_psram)
        {
          spiflash_resume_cache();
        }
#endif

      memcpy(spi_buffer, tx_buf, n);

#ifdef CONFIG_ESP32S3_SPIRAM

      /* Disable cache, and then write data from SRAM to flash */

      if (buffer_in_psram)
        {
          spiflash_suspend_cache();
        }
#endif

      wait_flash_idle();
      enable_flash_write();

      WRITE_DATA_TO_FLASH(tx_addr, spi_buffer, n);

      tx_bytes -= n;
      tx_buf += n;
      tx_addr += n;
    }

  wait_flash_idle();
  disable_flash_write();
  spiflash_flushmapped(dest_addr, size);
  spiflash_end();

  return ret;
}

/****************************************************************************
 * Name: spi_flash_read
 *
 * Description:
 *   Read data from Flash.
 *
 * Parameters:
 *   src_addr - source address of the data in Flash.
 *   dest     - pointer to the destination buffer
 *   size     - length of data
 *
 * Returned Values:
 *   Zero (OK) is returned or a negative error.
 *
 ****************************************************************************/

int spi_flash_read(uint32_t src_addr, void *dest, uint32_t size)
{
  int ret = OK;
  uint8_t *rx_buf = (uint8_t *)dest;
  uint32_t rx_bytes = size;
  uint32_t rx_addr = src_addr;
#ifdef CONFIG_ESP32S3_SPIRAM
  bool buffer_in_psram = esp32s3_ptr_extram(dest);
#endif

  spiflash_start();

  for (uint32_t i = 0; i < size; i += SPI_BUFFER_BYTES)
    {
      uint32_t spi_buffer[SPI_BUFFER_WORDS];
      uint32_t n = MIN(rx_bytes, SPI_BUFFER_BYTES);

      READ_DATA_FROM_FLASH(rx_addr, spi_buffer, n);

#ifdef CONFIG_ESP32S3_SPIRAM

      /* Re-enable cache, and then copy data from SRAM to PSRAM */

      if (buffer_in_psram)
        {
          spiflash_resume_cache();
        }
#endif

      memcpy(rx_buf, spi_buffer, n);
      rx_bytes -= n;
      rx_buf += n;
      rx_addr += n;

#ifdef CONFIG_ESP32S3_SPIRAM

      /* Disable cache, and then read data from flash to SRAM */

      if (buffer_in_psram)
        {
          spiflash_suspend_cache();
        }
#endif
    }

  spiflash_end();

  return ret;
}
#endif /* CONFIG_ESP32S3_SPI_FLASH_DONT_USE_ROM_CODE */

/****************************************************************************
 * Name: esp32s3_spiflash_init
 *
 * Description:
 *   Initialize ESP32-S3 SPI flash driver.
 *
 * Returned Value:
 *   OK if success or a negative value if fail.
 *
 ****************************************************************************/

int esp32s3_spiflash_init(void)
{
  int cpu;
  int ret = OK;

  /* Initializes SPI flash operation lock */

  nxrmutex_init(&g_flash_op_mutex);

#ifdef CONFIG_SMP
  sched_lock();

  for (cpu = 0; cpu < CONFIG_SMP_NCPUS; cpu++)
    {
      nxsem_init(&g_disable_non_iram_isr_on_core[cpu], 0, 0);

      ret = spiflash_init_spi_flash_op_block_task(cpu);
      if (ret != OK)
        {
          return ret;
        }
    }

  sched_unlock();
#else
  UNUSED(cpu);
#endif

  spi_flash_guard_set(&g_spi_flash_guard_funcs);

  return ret;
}

/****************************************************************************
 * Name: esp32s3_flash_encryption_enabled
 *
 * Description:
 *   Check if ESP32-S3 enables SPI Flash encryption.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   True: SPI Flash encryption is enable, False if not.
 *
 ****************************************************************************/

bool esp32s3_flash_encryption_enabled(void)
{
  bool enabled = false;
  uint32_t regval;
  uint32_t flash_crypt_cnt;

  regval = getreg32(EFUSE_RD_REPEAT_DATA1_REG);
  flash_crypt_cnt = (regval >> EFUSE_SPI_BOOT_CRYPT_CNT_S) &
                    EFUSE_SPI_BOOT_CRYPT_CNT_V;

  while (flash_crypt_cnt)
    {
      if (flash_crypt_cnt & 1)
        {
          enabled = !enabled;
        }

      flash_crypt_cnt >>= 1;
    }

  return enabled;
}
