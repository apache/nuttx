/****************************************************************************
 * arch/xtensa/src/esp32s2/esp32s2_spiram.c
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
#include "esp32s2_psram.h"
#include "esp32s2_spiram.h"
#include "hardware/esp32s2_soc.h"
#include "hardware/esp32s2_cache_memory.h"
#include "hardware/esp32s2_extmem.h"
#include "hardware/esp32s2_iomux.h"

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

#define PSRAM_MODE PSRAM_VADDR_MODE_NORMAL

#if defined(CONFIG_ESP32S2_SPIRAM)

#define MMU_PAGE_TO_BYTES(page_id)      ((page_id) << 16)
#define BYTES_TO_MMU_PAGE(bytes)        ((bytes) / MMU_PAGE_SIZE)

#if defined(CONFIG_ESP32S2_SPIRAM_SPEED_40M)
#  define PSRAM_SPEED PSRAM_CACHE_S40M
#else  /* #if CONFIG_ESP32S2_SPIRAM_SPEED_80M */
#  define PSRAM_SPEED PSRAM_CACHE_S80M
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

static bool g_spiram_inited;

/* These variables are in bytes */

static uint32_t g_allocable_vaddr_start;
static uint32_t g_allocable_vaddr_end;
static DRAM_ATTR uint32_t g_mapped_vaddr_start;

#if defined(CONFIG_ESP32S2_SPIRAM_FETCH_INSTRUCTIONS)
static int      g_instr_flash2spiram_offs;
static uint32_t g_instr_start_page;
static uint32_t g_instr_end_page;
#endif

#if defined(CONFIG_ESP32S2_SPIRAM_RODATA)
static int      g_rodata_flash2spiram_offs;
static uint32_t g_rodata_start_page;
static uint32_t g_rodata_end_page;
#endif

#if defined(CONFIG_ESP32S2_SPIRAM_FETCH_INSTRUCTIONS) || \
    defined(CONFIG_ESP32S2_SPIRAM_RODATA)
static uint32_t page0_mapped;
static uint32_t page0_page = INVALID_PHY_PAGE;
#endif

/* Let's export g_mapped_size to export heap */

DRAM_ATTR uint32_t g_mapped_size;

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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mmu_map_psram
 *
 * Description:
 *   Map the PSRAM to MMU
 *
 * Input Parameters:
 *   start_paddr - start of physical PSRAM address
 *   end_paddr   - end of physical PSRAM address
 *   out_start_vaddr - start of virtual address
 *
 * Returned Value:
 *   Zero value (OK) on success or a negative error.
 *
 ****************************************************************************/

int mmu_map_psram(uint32_t start_paddr, uint32_t end_paddr,
                  uint32_t *out_start_vaddr)
{
  /* For now, this function should only run when virtual address is enough
   * Decide these logics when there's a real PSRAM with larger size
   */

  uint32_t map_length = end_paddr - start_paddr;

  if (map_length > SOC_EXTRAM_DATA_SIZE)
    {
      /* Decide these logics when there's a real PSRAM with larger size */

      merr("PSRAM physical size is too large, not support mapping it yet!");
      return -ENOMEM;
    }

  /* should be MMU page aligned */

  assert((start_paddr % MMU_PAGE_SIZE) == 0);

  uint32_t start_vaddr = DPORT_CACHE_ADDRESS_LOW;
  uint32_t end_vaddr = start_vaddr + map_length;
  uint32_t cache_bus_mask = 0;

  cache_bus_mask |= (end_vaddr > 0) ? EXTMEM_PRO_DCACHE_MASK_DPORT : 0;
  cache_bus_mask |= (end_vaddr >= DPORT_ADDRESS_HIGH) ?
                    EXTMEM_PRO_DCACHE_MASK_DRAM1 : 0;
  cache_bus_mask |= (end_vaddr >= DRAM1_ADDRESS_HIGH) ?
                    EXTMEM_PRO_DCACHE_MASK_DRAM0 : 0;

  assert(end_vaddr <= DRAM0_CACHE_ADDRESS_HIGH);

  minfo("start_paddr is %x, map_length is %xB, %d pages",
        start_paddr, map_length, BYTES_TO_MMU_PAGE(map_length));

  /* No need to disable cache, this file is put in Internal RAM */

  cache_dbus_mmu_set(MMU_ACCESS_SPIRAM, start_vaddr, start_paddr, 64,
                     BYTES_TO_MMU_PAGE(map_length), 0);

  REG_CLR_BIT(EXTMEM_PRO_DCACHE_CTRL1_REG, cache_bus_mask);

  *out_start_vaddr = start_vaddr;

  return OK;
}

/****************************************************************************
 * Name: mmu_map_psram
 *
 * Description:
 *   Initialize the CACHE to use with PSRAM
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void IRAM_ATTR esp_spiram_init_cache(void)
{
  int ret;
  uint32_t start_page = 0;

  g_mapped_size = esp_spiram_get_size();

  /* Map the PSRAM physical range to MMU */

  ret = mmu_map_psram(MMU_PAGE_TO_BYTES(start_page),
                      MMU_PAGE_TO_BYTES(start_page) +
                      g_mapped_size, &g_mapped_vaddr_start);
  if (ret < 0)
    {
      merr("MMU PSRAM mapping wrong!");
      abort();
    }

  /* After mapping, we DON'T care about the PSRAM PHYSICAL
   * ADDRESSS ANYMORE!
   */

  g_allocable_vaddr_start = g_mapped_vaddr_start;
  g_allocable_vaddr_end   = g_mapped_vaddr_start + g_mapped_size;
}

/****************************************************************************
 * Name: esp_spiram_test
 *
 * Description:
 *   Simple RAM test. Writes a word every 32 bytes. Takes about a second
 *   to complete for 4MiB. Returns true when RAM seems OK, false when test
 *   fails. WARNING: Do not run this before the 2nd cpu has been initialized
 *   (in a two-core system) or after the heap allocator has taken ownership
 *   of the memory.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   True on success or False on failure
 *
 ****************************************************************************/

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
              merr("SPI SRAM error @ %08x:%08x/%08x \n", &spiram[p],
                   spiram[p], p ^ 0xaaaaaaaa);
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

#if defined(CONFIG_ESP32S2_SPIRAM_RODATA)
void rodata_flash_page_info_init(void)
{
  uint32_t rodata_page_cnt = ((uint32_t)&_rodata_reserved_end -
                              ((uint32_t)&_rodata_reserved_start &
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
  size_t spiram_size;

  r = psram_enable(PSRAM_SPEED, PSRAM_MODE);
  if (r != OK)
    {
      merr("SPI RAM enabled but initialization failed. Bailing out.\n");
      return r;
    }

  g_spiram_inited = true;

  spiram_size = esp_spiram_get_size();

#if defined(CONFIG_ESP32S2_SPIRAM_SIZE) && (CONFIG_ESP32S2_SPIRAM_SIZE != -1)
  if (spiram_size != CONFIG_ESP32S2_SPIRAM_SIZE)
    {
      merr("Expected %dMB chip but found %dMB chip. Bailing out..",
           (CONFIG_ESP32S2_SPIRAM_SIZE / 1024 / 1024),
           (spiram_size / 1024 / 1024));
      return;
    }
#endif

  minfo("Found %dMB SPI RAM device\n", spiram_size / (1024 * 1024));
  minfo("Speed: %dMHz\n", CONFIG_ESP32S2_SPIRAM_SPEED);
  minfo("Initialized, cache is in normal (1-core) mode.\n");
  return OK;
}

size_t esp_spiram_get_size(void)
{
  if (!g_spiram_inited)
    {
      merr("SPI RAM not initialized");
      abort();
    }

  return psram_get_size();
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

#endif
