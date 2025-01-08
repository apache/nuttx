/****************************************************************************
 * arch/xtensa/src/esp32/esp32_spiram.c
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

#ifdef CONFIG_ESP32_SPIRAM

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>
#include <string.h>
#include <sys/param.h>
#include <nuttx/spinlock.h>
#include <nuttx/init.h>

#include "esp32_spiram.h"
#include "esp32_spicache.h"
#include "esp32_psram.h"
#include "xtensa.h"
#include "xtensa_attr.h"
#include "hardware/esp32_soc.h"
#include "hardware/esp32_dport.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_SMP
#  define PSRAM_MODE PSRAM_VADDR_MODE_NORMAL
#else
#if CONFIG_ESP32_MEMMAP_SPIRAM_CACHE_EVENODD
#  define PSRAM_MODE PSRAM_VADDR_MODE_EVENODD
#else
#  define PSRAM_MODE PSRAM_VADDR_MODE_LOWHIGH
#endif
#endif

/* Let's to assume SPIFLASH SPEED == SPIRAM SPEED for now */

#if defined(CONFIG_ESP32_SPIRAM_SPEED_40M)
#  define PSRAM_SPEED PSRAM_CACHE_F40M_S40M
#elif defined(CONFIG_ESP32_SPIRAM_SPEED_80M)
#  define PSRAM_SPEED PSRAM_CACHE_F80M_S80M
#else
#  error "FLASH speed can only be equal to or higher than SRAM speed while SRAM is enabled!"
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool spiram_inited = false;

#ifdef CONFIG_SMP
static int pause_cpu_handler(void *cookie);
static struct smp_call_data_s g_call_data =
SMP_CALL_INITIALIZER(pause_cpu_handler, NULL);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pause_cpu_handler
 ****************************************************************************/

#ifdef CONFIG_SMP
static volatile bool g_cpu_wait = true;
static volatile bool g_cpu_pause = false;
static int pause_cpu_handler(void *cookie)
{
  g_cpu_pause = true;
  while (g_cpu_wait);

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

unsigned int IRAM_ATTR cache_sram_mmu_set(int cpu_no, int pid,
                                          unsigned int vaddr,
                                          unsigned int paddr,
                                          int psize, int num)
{
  uint32_t regval;
#ifdef CONFIG_SMP
  int cpu_to_stop = 0;
#endif
  const bool os_ready = OSINIT_OS_READY();
  unsigned int i;
  unsigned int shift;
  unsigned int mask_s;
  unsigned int mmu_addr;
  unsigned int mmu_table_val;
  irqstate_t flags;

  /* address check */

  if ((ADDRESS_CHECK(vaddr, psize)) || (ADDRESS_CHECK(paddr, psize)))
    {
      return MMU_SET_ADDR_ALIGNED_ERROR;
    }

  /* psize check */

  if (psize == 32)
    {
      shift  = 15;
      mask_s = 0;
    }
  else if (psize == 16)
    {
      shift  = 14;
      mask_s = 1;
    }
  else if (psize == 8)
    {
      shift  = 13;
      mask_s = 2;
    }
  else if (psize == 4)
    {
      shift  = 12;
      mask_s = 3;
    }
  else if (psize == 2)
    {
      shift  = 11;
      mask_s = 4;
    }
  else
    {
      return MMU_SET_PAGE_SIZE_ERROR;
    }

  /* mmu value */

  mmu_table_val = paddr >> shift;

  /* mmu_addr */

  if (pid == 0 || pid == 1)
    {
      if (vaddr >= PRO_DRAM1_START_ADDR && vaddr < PRO_DRAM1_END_ADDR(psize))
        {
          mmu_addr = 1152 + ((vaddr & (0x3fffff >> mask_s)) >> shift);
        }
      else
        {
          return MMU_SET_VADDR_OUT_RANGE;
        }
    }
  else
    {
      if (vaddr >= PRO_DRAM1_START_ADDR && vaddr < PRO_DRAM1_END_ADDR(psize))
        {
          mmu_addr = (1024 + (pid << 7)) +
                     ((vaddr & (0x3fffff >> mask_s)) >> shift);
        }
      else
        {
          return MMU_SET_VADDR_OUT_RANGE;
        }
    }

  /* The MMU registers are implemented in such a way that lookups from the
   * cache subsystem may collide with CPU access to the MMU registers. We use
   * the flash guards to make sure the cache is disabled.
   */

  flags = 0; /* suppress GCC warning */
  if (os_ready)
    {
      flags = enter_critical_section();
    }

#ifdef CONFIG_SMP
  /* The other CPU might be accessing the cache at the same time, just by
   * using variables in external RAM.
   */

  if (os_ready)
    {
      cpu_to_stop = this_cpu() == 1 ? 0 : 1;
      g_cpu_wait  = true;
      g_cpu_pause = false;
      nxsched_smp_call_single_async(cpu_to_stop, &g_call_data);
      while (!g_cpu_pause);
    }

  spi_disable_cache(1);
#endif

  spi_disable_cache(0);

  /* mmu change */

  for (i = 0; i < num; i++)
    {
      *(volatile unsigned int *)(CACHE_MMU_ADDRESS_BASE(cpu_no) +
        mmu_addr * 4) = mmu_table_val + i; /* write table */
      mmu_addr++;
    }

  if (cpu_no == 0)
    {
      regval  = getreg32(DPORT_PRO_CACHE_CTRL1_REG);
      regval &= ~DPORT_PRO_CMMU_SRAM_PAGE_MODE_M;
      regval |= mask_s << DPORT_PRO_CMMU_SRAM_PAGE_MODE_S;
      putreg32(regval, DPORT_PRO_CACHE_CTRL1_REG);
    }
  else
    {
      regval  = getreg32(DPORT_APP_CACHE_CTRL1_REG);
      regval &= ~DPORT_APP_CMMU_SRAM_PAGE_MODE_M;
      regval |= mask_s << DPORT_APP_CMMU_SRAM_PAGE_MODE_S;
      putreg32(regval, DPORT_APP_CACHE_CTRL1_REG);
    }

  spi_enable_cache(0);
#ifdef CONFIG_SMP
  spi_enable_cache(1);

  if (os_ready)
    {
      g_cpu_wait = false;
    }
#endif

  if (os_ready)
    {
      leave_critical_section(flags);
    }

  return 0;
}

void IRAM_ATTR esp_spiram_init_cache(void)
{
#ifdef CONFIG_SMP
  uint32_t regval;
#endif

  /* Enable external RAM in MMU */

  cache_sram_mmu_set(0, 0, SOC_EXTRAM_DATA_LOW, 0, 32, 128);

  /* Flush and enable icache for APP CPU */

#ifdef CONFIG_SMP
  regval  = getreg32(DPORT_APP_CACHE_CTRL1_REG);
  regval &= ~DPORT_APP_CACHE_MASK_DRAM1;
  putreg32(regval, DPORT_APP_CACHE_CTRL1_REG);
  cache_sram_mmu_set(1, 0, SOC_EXTRAM_DATA_LOW, 0, 32, 128);
#endif
}

/* Simple RAM test. Writes a word every 32 bytes. Takes about a second
 * to complete for 4MiB. Returns OK when RAM seems OK, ERROR when test
 * fails. WARNING: Do not run this before the 2nd cpu has been initialized
 * (in a two-core system) or after the heap allocator has taken ownership
 * of the memory.
 */

int esp_spiram_test(void)
{
  volatile int *spiram = (volatile int *)PRO_DRAM1_START_ADDR;

  /* Set size value to 4 MB which is related to psize argument on
   * cache_sram_mmu_set() calls. In this SoC, psize is 32 Mbit.
   */

  size_t s = 4 * 1024 * 1024;
  size_t p;
  int errct = 0;
  int initial_err = -1;

  for (p = 0; p < (s / sizeof(int)); p += 8)
    {
      spiram[p] = p ^ 0xaaaaaaaa;
    }

  for (p = 0; p < (s / sizeof(int)); p += 8)
    {
      if (spiram[p] != (p ^ 0xaaaaaaaa))
        {
          errct++;
          if (errct == 1)
            {
              initial_err = p * sizeof(int);
            }

          if (errct < 4)
            {
              merr("SPI SRAM error@%p:%08x/%08x \n", &spiram[p], spiram[p],
                   p ^ 0xaaaaaaaa);
            }
        }
    }

  if (errct != 0)
    {
      merr("SPI SRAM memory test fail. %d/%d writes failed, first @ %X\n",
           errct, s / 32, initial_err + SOC_EXTRAM_DATA_LOW);
      return ERROR;
    }
  else
    {
      minfo("SPI SRAM memory test OK!");
      return OK;
    }
}

int esp_spiram_get_chip_size(void)
{
  int psram_size;

  if (!spiram_inited)
    {
      merr("SPI RAM not initialized");
      return ESP_SPIRAM_SIZE_INVALID;
    }

  psram_size = psram_get_size();
  switch (psram_size)
  {
    case PSRAM_SIZE_16MBITS:
      return ESP_SPIRAM_SIZE_16MBITS;

    case PSRAM_SIZE_32MBITS:
      return ESP_SPIRAM_SIZE_32MBITS;

    case PSRAM_SIZE_64MBITS:
      return ESP_SPIRAM_SIZE_64MBITS;

    default:
      return ESP_SPIRAM_SIZE_INVALID;
  }
}

int esp_spiram_init(void)
{
  int ret;
  ret = psram_enable(PSRAM_SPEED, PSRAM_MODE);
  if (ret != OK)
    {
#ifdef CONFIG_ESP32_SPIRAM_IGNORE_NOTFOUND
      merr("SPI RAM enabled but initialization failed.\
           Bailing out.");
#endif
      return ret;
    }

  /* note: this needs to be set before esp_spiram_get_chip_* /
   * esp_spiram_get_size calls.
   */

  spiram_inited = true;

#if (CONFIG_ESP32_SPIRAM_SIZE != -1)
  if (esp_spiram_get_size() != CONFIG_ESP32_SPIRAM_SIZE)
    {
      merr("Expected %dKiB chip but found %dKiB chip.\
           Bailing out..\n", CONFIG_ESP32_SPIRAM_SIZE / 1024,
           esp_spiram_get_size() / 1024);
      return -EINVAL;
    }
#endif

  minfo("Found %dMBit SPI RAM device\n",
        (esp_spiram_get_size() * 8) / (1024 * 1024));

  minfo("SPI RAM mode: %s\n",
        PSRAM_SPEED == PSRAM_CACHE_F40M_S40M ? "flash 40m sram 40m" : \
        PSRAM_SPEED == PSRAM_CACHE_F80M_S40M ? "flash 80m sram 40m" : \
        PSRAM_SPEED == PSRAM_CACHE_F80M_S80M ? "flash 80m sram 80m" : \
        "ERROR");

  minfo("PSRAM initialized, cache is in %s mode.\n", \
        (PSRAM_MODE == PSRAM_VADDR_MODE_EVENODD) ? "even/odd (2-core)": \
        (PSRAM_MODE == PSRAM_VADDR_MODE_LOWHIGH) ? "low/high (2-core)": \
        (PSRAM_MODE == PSRAM_VADDR_MODE_NORMAL) ? "normal (1-core)":"ERROR");

  return OK;
}

size_t esp_spiram_get_size(void)
{
  int size = esp_spiram_get_chip_size();

  if (size == PSRAM_SIZE_16MBITS)
    {
      return 2 * 1024 * 1024;
    }

  if (size == PSRAM_SIZE_32MBITS)
    {
      return 4 * 1024 * 1024;
    }

  if (size == PSRAM_SIZE_64MBITS)
    {
      return 8 * 1024 * 1024;
    }

  return CONFIG_ESP32_SPIRAM_SIZE;
}

/* Before flushing the cache, if psram is enabled as a memory-mapped thing,
 * we need to write back the data in the cache to the psram first, otherwise
 * it will get lost. For now, we just read 64/128K of random PSRAM memory to
 * do this. Note that this routine assumes some unique mapping for the first
 * 2 banks of the PSRAM memory range, as well as the 2 banks after the 2 MiB
 * mark.
 */

void IRAM_ATTR esp_spiram_writeback_cache(void)
{
  int x;
  uint32_t regval;
  volatile int i = 0;
  volatile uint8_t *psram = (volatile uint8_t *)SOC_EXTRAM_DATA_LOW;
  int cache_was_disabled = 0;

  if (!spiram_inited)
    {
      return;
    }

  /* We need cache enabled for this to work. Re-enable it if needed; make
   * sure we disable it again on exit as well.
   */

  regval = getreg32(DPORT_PRO_CACHE_CTRL_REG);

  if ((regval & DPORT_PRO_CACHE_ENABLE) == 0)
    {
      cache_was_disabled |= (1 << 0);
      regval  = getreg32(DPORT_PRO_CACHE_CTRL_REG);
      regval |= (1 << DPORT_PRO_CACHE_ENABLE_S);
      putreg32(regval, DPORT_PRO_CACHE_CTRL_REG);
    }

#ifdef CONFIG_SMP
  regval = getreg32(DPORT_APP_CACHE_CTRL_REG);

  if ((regval & DPORT_APP_CACHE_ENABLE) == 0)
    {
      cache_was_disabled |= (1 << 1);
      regval  = getreg32(DPORT_APP_CACHE_CTRL_REG);
      regval |= 1 << DPORT_APP_CACHE_ENABLE_S;
      putreg32(regval, DPORT_APP_CACHE_CTRL_REG);
    }
#endif

#if (PSRAM_MODE != PSRAM_VADDR_MODE_LOWHIGH)
  /* Single-core and even/odd mode only have 32K of cache evenly distributed
   * over the address lines. We can clear the cache by just reading 64K
   * worth of cache lines.
   */

  for (x = 0; x < 1024 * 64; x += 32)
    {
      i += psram[x];
    }
#else
  /* Low/high psram cache mode uses one 32K cache for the lowest 2MiB of SPI
   * flash and another 32K for the highest 2MiB. Clear this by reading from
   * both regions. Note: this assumes the amount of external RAM is >2M.
   * If it is 2M or less, what this code does is undefined. If we ever
   * support external RAM chips of 2M or smaller, this may need adjusting.
   */

  for (x = 0; x < 1024 * 64; x += 32)
    {
      i += psram[x];
      i += psram[x + (1024 * 1024 * 2)];
    }
#endif

  if (cache_was_disabled & (1 << 0))
    {
      while (((getreg32(DPORT_PRO_DCACHE_DBUG0_REG) >>
              (DPORT_PRO_CACHE_STATE_S)) &
              (DPORT_PRO_CACHE_STATE)) != 1)
        {
        };

      regval  = getreg32(DPORT_PRO_CACHE_CTRL_REG);
      regval &= ~(1 << DPORT_PRO_CACHE_ENABLE_S);
      putreg32(regval, DPORT_PRO_CACHE_CTRL_REG);
    }

#ifdef CONFIG_SMP
  if (cache_was_disabled & (1 << 1))
    {
      while (((getreg32(DPORT_APP_DCACHE_DBUG0_REG) >>
              (DPORT_APP_CACHE_STATE_S)) &
              (DPORT_APP_CACHE_STATE)) != 1)
        {
        };

      regval  = getreg32(DPORT_APP_CACHE_CTRL_REG);
      regval &= ~(1 << DPORT_APP_CACHE_ENABLE_S);
      putreg32(regval, DPORT_APP_CACHE_CTRL_REG);
    }
#endif
}

/* If SPI RAM(PSRAM) has been initialized
 *
 * Return:
 *   - true SPI RAM has been initialized successfully
 *   - false SPI RAM hasn't been initialized or initialized failed
 */

bool esp_spiram_is_initialized(void)
{
  return spiram_inited;
}

#endif
