/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_start.c
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

#include <nuttx/arch.h>
#include <nuttx/init.h>

#include <arch/board/board.h>

#include "esp32c3.h"
#include "esp32c3_clockconfig.h"
#include "esp32c3_irq.h"
#include "esp32c3_lowputc.h"
#include "esp32c3_start.h"
#include "esp32c3_wdt.h"
#include "hardware/esp32c3_cache_memory.h"
#include "hardware/extmem_reg.h"

#ifdef CONFIG_ESP32C3_BROWNOUT_DET
#  include "esp32c3_brownout.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
#  define showprogress(c)     riscv_lowputc(c)
#else
#  define showprogress(c)
#endif

#ifdef CONFIG_ESP32C3_APP_FORMAT_MCUBOOT

#define PRIMARY_SLOT_OFFSET   CONFIG_ESP32C3_OTA_PRIMARY_SLOT_OFFSET

#define HDR_ATTR              __attribute__((section(".entry_addr"))) \
                                __attribute__((used))

/* Cache MMU block size */

#define MMU_BLOCK_SIZE        0x00010000  /* 64 KB */

/* Cache MMU address mask (MMU tables ignore bits which are zero) */

#define MMU_FLASH_MASK        (~(MMU_BLOCK_SIZE - 1))

#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_ESP32C3_APP_FORMAT_MCUBOOT
extern uint32_t _image_irom_vma;
extern uint32_t _image_irom_lma;
extern uint32_t _image_irom_size;

extern uint32_t _image_drom_vma;
extern uint32_t _image_drom_lma;
extern uint32_t _image_drom_size;
#endif

/****************************************************************************
 * ROM Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_ESP32C3_APP_FORMAT_MCUBOOT
extern int ets_printf(const char *fmt, ...);
extern uint32_t cache_suspend_icache(void);
extern void cache_resume_icache(uint32_t val);
extern void cache_invalidate_icache_all(void);
extern int cache_dbus_mmu_set(uint32_t ext_ram, uint32_t vaddr,
                              uint32_t paddr, uint32_t psize, uint32_t num,
                              uint32_t fixed);
extern int cache_ibus_mmu_set(uint32_t ext_ram, uint32_t vaddr,
                              uint32_t paddr, uint32_t psize, uint32_t num,
                              uint32_t fixed);
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_ESP32C3_APP_FORMAT_MCUBOOT
IRAM_ATTR noreturn_function void __start(void);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_ESP32C3_APP_FORMAT_MCUBOOT
HDR_ATTR static void (*_entry_point)(void) = &__start;
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Address of the IDLE thread */

uint8_t g_idlestack[CONFIG_IDLETHREAD_STACKSIZE]
  aligned_data(16) locate_data(".noinit");
uint32_t g_idle_topstack = ESP32C3_IDLESTACK_TOP;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: calc_mmu_pages
 *
 * Description:
 *   Calculate the number of cache pages to map.
 *
 * Input Parameters:
 *   size  - Size of data to map
 *   vaddr - Virtual address where data will be mapped
 *
 * Returned Value:
 *   Number of cache MMU pages required to do the mapping.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32C3_APP_FORMAT_MCUBOOT
static inline uint32_t calc_mmu_pages(uint32_t size, uint32_t vaddr)
{
  return (size + (vaddr - (vaddr & MMU_FLASH_MASK)) + MMU_BLOCK_SIZE - 1) /
    MMU_BLOCK_SIZE;
}
#endif

/****************************************************************************
 * Name: map_rom_segments
 *
 * Description:
 *   Configure the MMU and Cache peripherals for accessing ROM code and data.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32C3_APP_FORMAT_MCUBOOT
static int map_rom_segments(void)
{
  uint32_t rc = 0;
  uint32_t regval;
  uint32_t drom_lma_aligned;
  uint32_t drom_vma_aligned;
  uint32_t drom_page_count;
  uint32_t irom_lma_aligned;
  uint32_t irom_vma_aligned;
  uint32_t irom_page_count;

  size_t partition_offset = PRIMARY_SLOT_OFFSET;
  uint32_t app_irom_lma = partition_offset + (uint32_t)&_image_irom_lma;
  uint32_t app_irom_size = (uint32_t)&_image_irom_size;
  uint32_t app_irom_vma = (uint32_t)&_image_irom_vma;
  uint32_t app_drom_lma = partition_offset + (uint32_t)&_image_drom_lma;
  uint32_t app_drom_size = (uint32_t)&_image_drom_size;
  uint32_t app_drom_vma = (uint32_t)&_image_drom_vma;

  uint32_t autoload = cache_suspend_icache();
  cache_invalidate_icache_all();

  /* Clear the MMU entries that are already set up, so the new app only has
   * the mappings it creates.
   */

  for (size_t i = 0; i < FLASH_MMU_TABLE_SIZE; i++)
    {
      FLASH_MMU_TABLE[i] = MMU_TABLE_INVALID_VAL;
    }

  drom_lma_aligned = app_drom_lma & MMU_FLASH_MASK;
  drom_vma_aligned = app_drom_vma & MMU_FLASH_MASK;
  drom_page_count = calc_mmu_pages(app_drom_size, app_drom_vma);
  rc  = cache_dbus_mmu_set(MMU_ACCESS_FLASH, drom_vma_aligned,
                           drom_lma_aligned, 64, (int)drom_page_count, 0);

  irom_lma_aligned = app_irom_lma & MMU_FLASH_MASK;
  irom_vma_aligned = app_irom_vma & MMU_FLASH_MASK;
  irom_page_count = calc_mmu_pages(app_irom_size, app_irom_vma);
  rc |= cache_ibus_mmu_set(MMU_ACCESS_FLASH, irom_vma_aligned,
                           irom_lma_aligned, 64, (int)irom_page_count, 0);

  regval  = getreg32(EXTMEM_ICACHE_CTRL1_REG);
  regval &= ~(EXTMEM_ICACHE_SHUT_IBUS_M | EXTMEM_ICACHE_SHUT_DBUS_M);
  putreg32(regval, EXTMEM_ICACHE_CTRL1_REG);

  cache_resume_icache(autoload);

  return (int)rc;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __esp32c3_start
 ****************************************************************************/

void __esp32c3_start(void)
{
#ifdef CONFIG_ESP32C3_APP_FORMAT_MCUBOOT
  if (map_rom_segments() != 0)
    {
      ets_printf("Failed to setup XIP, aborting\n");
      while (true);
    }

#endif

  /* Set CPU frequency */

  esp32c3_clockconfig();

#ifdef CONFIG_ESP32C3_BROWNOUT_DET
  /* Initialize hardware brownout check and reset */

  esp32c3_brownout_init();
#endif

  /* Configure the UART so we can get debug output */

  esp32c3_lowsetup();

#ifdef USE_EARLYSERIALINIT
  /* Perform early serial initialization */

  riscv_earlyserialinit();
#endif

  showprogress('A');

  /* Clear .bss.  We'll do this inline (vs. calling memset) just to be
   * certain that there are no issues with the state of global variables.
   */

  for (uint32_t *dest = &_sbss; dest < &_ebss; dest++)
    {
      *dest = 0;
    }

  showprogress('B');

  /* Disable any wdt enabled by bootloader */

  esp32c3_wdt_early_deinit();

  /* Initialize onboard resources */

  esp32c3_board_initialize();

  /* Bring up NuttX */

  nx_start();

  for (; ; );
}
