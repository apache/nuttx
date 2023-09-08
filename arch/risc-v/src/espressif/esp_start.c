/****************************************************************************
 * arch/risc-v/src/espressif/esp_start.c
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

#include <nuttx/arch.h>
#include <nuttx/init.h>

#include "riscv_internal.h"

#include "esp_irq.h"
#include "esp_libc_stubs.h"
#include "esp_lowputc.h"
#include "esp_start.h"

#include "esp_clk_internal.h"
#include "esp_cpu.h"
#include "esp_private/brownout.h"
#include "hal/wdt_hal.h"
#include "soc/ext_mem_defs.h"
#include "soc/extmem_reg.h"
#include "soc/mmu.h"
#include "soc/reg_base.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
#  define showprogress(c)     riscv_lowputc(c)
#else
#  define showprogress(c)
#endif

#ifdef CONFIG_ESPRESSIF_BOOTLOADER_MCUBOOT
#  define PRIMARY_SLOT_OFFSET   CONFIG_ESPRESSIF_OTA_PRIMARY_SLOT_OFFSET
#  define HDR_ATTR              __attribute__((section(".entry_addr"))) \
                                  __attribute__((used))
#  define FLASH_MMU_TABLE       ((volatile uint32_t*) DR_REG_MMU_TABLE)
#  define FLASH_MMU_TABLE_SIZE  (ICACHE_MMU_SIZE/sizeof(uint32_t))
#  define MMU_BLOCK_SIZE        0x00010000  /* 64 KB */
#  define MMU_FLASH_MASK        (~(MMU_BLOCK_SIZE - 1))
#  define CACHE_REG             EXTMEM_ICACHE_CTRL1_REG
#  define CACHE_MASK            (EXTMEM_ICACHE_SHUT_IBUS_M | \
                                 EXTMEM_ICACHE_SHUT_DBUS_M)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_BOOTLOADER_MCUBOOT
extern uint8_t _image_irom_vma[];
extern uint8_t _image_irom_lma[];
extern uint8_t _image_irom_size[];

extern uint8_t _image_drom_vma[];
extern uint8_t _image_drom_lma[];
extern uint8_t _image_drom_size[];
#endif

/****************************************************************************
 * ROM Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_BOOTLOADER_MCUBOOT
extern int ets_printf(const char *fmt, ...) printf_like(1, 2);
extern uint32_t cache_suspend_icache(void);
extern void cache_resume_icache(uint32_t val);
extern void cache_invalidate_icache_all(void);

#ifdef CONFIG_ESPRESSIF_ESP32C3
extern int cache_dbus_mmu_set(uint32_t ext_ram, uint32_t vaddr,
                              uint32_t paddr, uint32_t psize, uint32_t num,
                              uint32_t fixed);
extern int cache_ibus_mmu_set(uint32_t ext_ram, uint32_t vaddr,
                              uint32_t paddr, uint32_t psize, uint32_t num,
                              uint32_t fixed);
#elif defined(CONFIG_ESPRESSIF_ESP32C6)
extern bool ets_efuse_cache_encryption_enabled(void);
extern int cache_mspi_mmu_set(uint32_t sensitive, uint32_t ext_ram,
                              uint32_t vaddr, uint32_t paddr, uint32_t psize,
                              uint32_t num, uint32_t fixed);
#endif

#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_BOOTLOADER_MCUBOOT
IRAM_ATTR noreturn_function void __start(void);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_BOOTLOADER_MCUBOOT
HDR_ATTR static void (*_entry_point)(void) = __start;
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Address of the IDLE thread */

uint8_t g_idlestack[CONFIG_IDLETHREAD_STACKSIZE]
  aligned_data(16) locate_data(".noinit");
uintptr_t g_idle_topstack = ESP_IDLESTACK_TOP;

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

#ifdef CONFIG_ESPRESSIF_BOOTLOADER_MCUBOOT
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

#ifdef CONFIG_ESPRESSIF_BOOTLOADER_MCUBOOT
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
  uint32_t app_irom_lma = partition_offset + (uint32_t)_image_irom_lma;
  uint32_t app_irom_size = (uint32_t)_image_irom_size;
  uint32_t app_irom_vma = (uint32_t)_image_irom_vma;
  uint32_t app_drom_lma = partition_offset + (uint32_t)_image_drom_lma;
  uint32_t app_drom_size = (uint32_t)_image_drom_size;
  uint32_t app_drom_vma = (uint32_t)_image_drom_vma;

  uint32_t autoload = cache_suspend_icache();
  cache_invalidate_icache_all();

  /* Clear the MMU entries that are already set up, so the new app only has
   * the mappings it creates.
   */

  for (size_t i = 0; i < FLASH_MMU_TABLE_SIZE; i++)
    {
      FLASH_MMU_TABLE[i] = MMU_INVALID;
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

  regval  = getreg32(CACHE_REG);
  regval &= ~(CACHE_MASK);
  putreg32(regval, CACHE_REG);

  cache_resume_icache(autoload);

  return (int)rc;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __esp_start
 ****************************************************************************/

void __esp_start(void)
{
#ifdef CONFIG_ESPRESSIF_BOOTLOADER_MCUBOOT
  if (map_rom_segments() != 0)
    {
      ets_printf("Failed to setup XIP, aborting\n");
      while (true);
    }

#endif

#ifdef CONFIG_ESPRESSIF_REGION_PROTECTION
  /* Configure region protection */

  esp_cpu_configure_region_protection();
#endif

  /* Configures the CPU clock, RTC slow and fast clocks, and performs
   * RTC slow clock calibration.
   */

  esp_clk_init();

  /* Disable clock of unused peripherals */

  esp_perip_clk_init();

#ifdef CONFIG_ESPRESSIF_BROWNOUT_DET
  /* Initialize hardware brownout check and reset */

  esp_brownout_init();
#endif

  /* Configure the UART so we can get debug output */

  esp_lowsetup();

#ifdef USE_EARLYSERIALINIT
  /* Perform early serial initialization */

  riscv_earlyserialinit();
#endif

  showprogress('A');

  /* Clear .bss. We'll do this inline (vs. calling memset) just to be
   * certain that there are no issues with the state of global variables.
   */

  for (uint32_t *dest = (uint32_t *)_sbss; dest < (uint32_t *)_ebss; )
    {
      *dest++ = 0;
    }

  /* Setup the syscall table needed by the ROM code */

  esp_setup_syscall_table();

  showprogress('B');

  /* The 2nd stage bootloader enables RTC WDT to monitor any issues that may
   * prevent the startup sequence from finishing correctly. Hence disable it
   * as NuttX is about to start.
   */

  wdt_hal_context_t rwdt_ctx = RWDT_HAL_CONTEXT_DEFAULT();
  wdt_hal_write_protect_disable(&rwdt_ctx);
  wdt_hal_disable(&rwdt_ctx);
  wdt_hal_write_protect_enable(&rwdt_ctx);

  /* Initialize onboard resources */

  esp_board_initialize();

  showprogress('C');

  /* Bring up NuttX */

  nx_start();

  for (; ; );
}
