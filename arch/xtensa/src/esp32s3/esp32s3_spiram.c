/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_spiram.c
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
#include <stdlib.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>
#include <string.h>
#include <sys/param.h>
#include <nuttx/config.h>
#include <nuttx/spinlock.h>

#include "xtensa.h"
#include "xtensa_attr.h"
#include "esp32s3_psram.h"
#include "esp32s3_spiram.h"
#include "hardware/esp32s3_soc.h"
#include "hardware/esp32s3_cache_memory.h"
#include "hardware/esp32s3_extmem.h"
#include "hardware/esp32s3_iomux.h"

#define PSRAM_MODE PSRAM_VADDR_MODE_NORMAL

#if defined(CONFIG_ESP32S3_SPIRAM)

#if defined(CONFIG_ESP32S3_SPIRAM_SPEED_40M)
#  define PSRAM_SPEED PSRAM_CACHE_S40M
#else  /* #if CONFIG_ESP32S3_SPIRAM_SPEED_80M */
#  define PSRAM_SPEED PSRAM_CACHE_S80M
#endif

static bool g_spiram_inited;

/* These variables are in bytes */

static uint32_t g_allocable_vaddr_start;
static uint32_t g_allocable_vaddr_end;
static DRAM_ATTR uint32_t g_mapped_vaddr_start;

/* Let's export g_mapped_size to export heap */

DRAM_ATTR uint32_t g_mapped_size;

static uint32_t g_instruction_in_spiram;
static uint32_t g_rodata_in_spiram;

#if defined(CONFIG_ESP32S3_SPIRAM_FETCH_INSTRUCTIONS)
static int      g_instr_flash2spiram_offs;
static uint32_t g_instr_start_page;
static uint32_t g_instr_end_page;
#endif

#if defined(CONFIG_ESP32S3_SPIRAM_RODATA)
static int      g_rodata_flash2spiram_offs;
static uint32_t g_rodata_start_page;
static uint32_t g_rodata_end_page;
#endif

#if defined(CONFIG_ESP32S3_SPIRAM_FETCH_INSTRUCTIONS) || \
    defined(CONFIG_ES32S3_SPIRAM_RODATA)
static uint32_t page0_mapped;
static uint32_t page0_page = INVALID_PHY_PAGE;
#endif

/****************************************************************************
 * ROM Function Prototypes
 ****************************************************************************/

extern void cache_writeback_all(void);
extern uint32_t cache_suspend_dcache(void);
extern void cache_resume_dcache(uint32_t val);
extern int cache_dbus_mmu_set(uint32_t ext_ram, uint32_t vaddr,
                              uint32_t paddr, uint32_t psize,
                              uint32_t num, uint32_t fixed);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mmu_valid_space
 *
 * Description:
 *   Calculate MMU valid space.
 *
 * Input Parameters:
 *   start_address - Pointer to store MMU mapped start address
 *
 * Returned Value:
 *   MMU valid space size by bytes.
 *
 ****************************************************************************/

static inline uint32_t mmu_valid_space(uint32_t *start_address)
{
  for (int i = 0; i < FLASH_MMU_TABLE_SIZE; i++)
    {
      if (FLASH_MMU_TABLE[i] & MMU_INVALID)
        {
          *start_address = DRAM0_CACHE_ADDRESS_LOW + i * MMU_PAGE_SIZE;
          return (FLASH_MMU_TABLE_SIZE - i) * MMU_PAGE_SIZE;
        }
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Initially map all psram physical address to virtual address.
 * If psram physical size is larger than virtual address range, then only
 * map the virtual address range.
 */

void IRAM_ATTR esp_spiram_init_cache(void)
{
  uint32_t regval;
  uint32_t psram_size;
  uint32_t mapped_vaddr_size;

  int ret = psram_get_available_size(&psram_size);
  if (ret != OK)
    {
      abort();
    }

  minfo("PSRAM available size = %d\n", psram_size);

  mapped_vaddr_size = mmu_valid_space(&g_mapped_vaddr_start);
  minfo("Virtual address size = %d\n", mapped_vaddr_size);

  if (mapped_vaddr_size < psram_size)
    {
      /* Decide these logics when there's a real PSRAM with larger size */

      g_mapped_size = mapped_vaddr_size;
      mwarn("Virtual address not enough for PSRAM, only %d size is mapped!",
            g_mapped_size);
    }
  else
    {
      g_mapped_size = psram_size;
    }

  /* Suspend DRAM Case during configuration */

  cache_suspend_dcache();

  cache_dbus_mmu_set(MMU_ACCESS_SPIRAM, g_mapped_vaddr_start,
                     0, 64, g_mapped_size >> 16, 0);

  regval = getreg32(EXTMEM_DCACHE_CTRL1_REG);
  regval &= ~EXTMEM_DCACHE_SHUT_CORE0_BUS;
  putreg32(regval, EXTMEM_DCACHE_CTRL1_REG);

#if defined(CONFIG_SMP)
  regval = getreg32(EXTMEM_DCACHE_CTRL1_REG);
  regval &= ~EXTMEM_DCACHE_SHUT_CORE1_BUS;
  putreg32(regval, EXTMEM_DCACHE_CTRL1_REG);
#endif

  cache_resume_dcache(0);

  /* Currently no non-heap stuff on ESP32S3 */

  g_allocable_vaddr_start = g_mapped_vaddr_start;
  g_allocable_vaddr_end = g_mapped_vaddr_start + g_mapped_size;
}

/* Simple RAM test. Writes a word every 32 bytes. Takes about a second
 * to complete for 4MiB. Returns true when RAM seems OK, false when test
 * fails. WARNING: Do not run this before the 2nd cpu has been initialized
 * (in a two-core system) or after the heap allocator has taken ownership
 * of the memory.
 */

bool esp_spiram_test(void)
{
  volatile int *spiram = (volatile int *)g_mapped_vaddr_start;

  size_t s = g_mapped_size;
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
      return false;
    }
  else
    {
      minfo("SPI SRAM memory test OK!");
      return true;
    }
}

uint32_t esp_spiram_instruction_access_enabled(void)
{
  return g_instruction_in_spiram;
}

uint32_t esp_spiram_rodata_access_enabled(void)
{
  return g_rodata_in_spiram;
}

#if defined(CONFIG_ESP32S3_SPIRAM_FETCH_INSTRUCTIONS)
int esp_spiram_enable_instruction_access(void)
{
  /* `pages_for_flash` will be overwritten, however it influences the psram
   * size to be added to the heap allocator.
   */

  abort();
}
#endif

#if defined(CONFIG_ESP32S3_SPIRAM_RODATA)
int esp_spiram_enable_rodata_access(void)
{
  /* `pages_for_flash` will be overwritten, however it influences the psram
   * size to be added to the heap allocator.
   */

  abort();
}
#endif

#if defined(CONFIG_ESP32S3_SPIRAM_FETCH_INSTRUCTIONS)
void instruction_flash_page_info_init(void)
{
  uint32_t instr_page_cnt = ((uint32_t)_instruction_reserved_end -
                            SOC_IROM_LOW + MMU_PAGE_SIZE - 1) /
                            MMU_PAGE_SIZE;

  g_instr_start_page = *(volatile uint32_t *)(DR_REG_MMU_TABLE +
                                              CACHE_IROM_MMU_START);
  g_instr_start_page &= MMU_ADDRESS_MASK;
  g_instr_end_page = g_instr_start_page + instr_page_cnt - 1;
}

uint32_t IRAM_ATTR instruction_flash_start_page_get(void)
{
  return g_instr_start_page;
}

uint32_t IRAM_ATTR instruction_flash_end_page_get(void)
{
  return g_instr_end_page;
}

int IRAM_ATTR instruction_flash2spiram_offset(void)
{
  return g_instr_flash2spiram_offs;
}
#endif

#if defined(CONFIG_ESP32_SPIRAM_RODATA)
void rodata_flash_page_info_init(void)
{
  uint32_t rodata_page_cnt = ((uint32_t)_rodata_reserved_end -
                              ((uint32_t)_rodata_reserved_start &
                              ~ (MMU_PAGE_SIZE - 1)) + MMU_PAGE_SIZE - 1) /
                              MMU_PAGE_SIZE;

  g_rodata_start_page = *(volatile uint32_t *)(DR_REG_MMU_TABLE +
                                               CACHE_DROM_MMU_START);
  g_rodata_start_page &= MMU_ADDRESS_MASK;
  g_rodata_end_page = g_rodata_start_page + rodata_page_cnt - 1;
}

uint32_t IRAM_ATTR rodata_flash_start_page_get(void)
{
  return g_rodata_start_page;
}

uint32_t IRAM_ATTR rodata_flash_end_page_get(void)
{
  return g_rodata_end_page;
}

int IRAM_ATTR g_rodata_flash2spiram_offset(void)
{
  return g_rodata_flash2spiram_offs;
}
#endif

int esp_spiram_init(void)
{
  int r;
  uint32_t psram_physical_size = 0;

  r = psram_enable(PSRAM_SPEED, PSRAM_MODE);
  if (r != OK)
    {
      merr("SPI RAM enabled but initialization failed. Bailing out.\n");
      return r;
    }

  g_spiram_inited = true;

  r = psram_get_physical_size(&psram_physical_size);
  if (r != OK)
    {
      abort();
    }

#if defined(CONFIG_ESP32S3_SPIRAM_SIZE) && (CONFIG_ESP32S3_SPIRAM_SIZE != -1)
  if (psram_physical_size != CONFIG_ESP32S3_SPIRAM_SIZE)
    {
      merr("Expected %dMB chip but found %dMB chip. Bailing out..",
           (CONFIG_ESP32S3_SPIRAM_SIZE / 1024 / 1024),
           (psram_physical_size / 1024 / 1024));
      return ;
    }
#endif

  minfo("Found %dMB SPI RAM device\n", psram_physical_size / (1024 * 1024));
  minfo("Speed: %dMHz\n", CONFIG_ESP32S3_SPIRAM_SPEED);
  minfo("Initialized, cache is in %s mode.\n",
                 (PSRAM_MODE == PSRAM_VADDR_MODE_EVENODD) ?
                  "even/odd (2-core)" :
                 (PSRAM_MODE == PSRAM_VADDR_MODE_LOWHIGH) ?
                  "low/high (2-core)" :
                 (PSRAM_MODE == PSRAM_VADDR_MODE_NORMAL) ?
                  "normal (1-core)" : "ERROR");
  return OK;
}

size_t esp_spiram_get_size(void)
{
  if (!g_spiram_inited)
    {
      merr("SPI RAM not initialized");
      abort();
    }

  uint32_t size = 0;  /* in bytes */
  int ret = psram_get_available_size(&size);

  return ret == OK ? size : 0;
}

/* Before flushing the cache, if psram is enabled as a memory-mapped thing,
 * we need to write back the data in the cache to the psram first, otherwise
 * it will get lost. For now, we just read 64/128K of random PSRAM memory to
 * do this.
 */

void IRAM_ATTR esp_spiram_writeback_cache(void)
{
  cache_writeback_all();
}

/**
 * @brief If SPI RAM(PSRAM) has been initialized
 *
 * @return true SPI RAM has been initialized successfully
 * @return false SPI RAM hasn't been initialized or initialized failed
 */

bool esp_spiram_is_initialized(void)
{
  return g_spiram_inited;
}

uint8_t esp_spiram_get_cs_io(void)
{
  return psram_get_cs_io();
}

/**
 * @brief Get allocable virtual start address
 *
 * @return Allocable virtual start address
 */

uint32_t esp_spiram_allocable_vaddr_start(void)
{
  return g_allocable_vaddr_start;
}

/**
 * @brief Get allocable virtual end address
 *
 * @return Allocable virtual end address
 */

uint32_t esp_spiram_allocable_vaddr_end(void)
{
  return g_allocable_vaddr_end;
}

#endif
