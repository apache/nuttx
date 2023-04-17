/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_start.c
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
#include <string.h>

#include <nuttx/init.h>
#include <nuttx/irq.h>

#include "xtensa.h"
#include "xtensa_attr.h"

#include "esp32s3_start.h"
#include "esp32s3_lowputc.h"
#include "esp32s3_clockconfig.h"
#include "esp32s3_region.h"
#include "esp32s3_spiram.h"
#include "esp32s3_wdt.h"
#ifdef CONFIG_BUILD_PROTECTED
#  include "esp32s3_userspace.h"
#endif
#include "esp32s3_spi_timing.h"
#include "hardware/esp32s3_cache_memory.h"
#include "hardware/esp32s3_system.h"
#include "hardware/esp32s3_extmem.h"
#include "rom/esp32s3_libc_stubs.h"
#include "rom/esp32s3_spiflash.h"
#include "rom/esp32s3_opi_flash.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
#  define showprogress(c)     xtensa_lowputc(c)
#else
#  define showprogress(c)
#endif

#ifdef CONFIG_ESP32S3_APP_FORMAT_MCUBOOT

#define PRIMARY_SLOT_OFFSET   CONFIG_ESP32S3_OTA_PRIMARY_SLOT_OFFSET

#define HDR_ATTR              locate_code(".entry_addr") used_code

/* Cache MMU address mask (MMU tables ignore bits which are zero) */

#define MMU_FLASH_MASK        (~(MMU_PAGE_SIZE - 1))

#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_APP_FORMAT_MCUBOOT
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

#ifdef CONFIG_ESP32S3_APP_FORMAT_MCUBOOT
extern int ets_printf(const char *fmt, ...) printf_like(1, 2);
extern int cache_dbus_mmu_set(uint32_t ext_ram, uint32_t vaddr,
                              uint32_t paddr, uint32_t psize, uint32_t num,
                              uint32_t fixed);
extern int cache_ibus_mmu_set(uint32_t ext_ram, uint32_t vaddr,
                              uint32_t paddr, uint32_t psize, uint32_t num,
                              uint32_t fixed);
#endif

extern void rom_config_instruction_cache_mode(uint32_t cfg_cache_size,
                                              uint8_t cfg_cache_ways,
                                              uint8_t cfg_cache_line_size);
extern void rom_config_data_cache_mode(uint32_t cfg_cache_size,
                                       uint8_t cfg_cache_ways,
                                       uint8_t cfg_cache_line_size);
extern void cache_invalidate_dcache_all(void);
extern uint32_t cache_suspend_dcache(void);
extern void cache_resume_dcache(uint32_t val);
extern uint32_t cache_set_idrom_mmu_size(uint32_t irom_size,
                                         uint32_t drom_size);
extern void cache_set_idrom_mmu_info(uint32_t instr_page_num,
                                     uint32_t rodata_page_num,
                                     uint32_t rodata_start,
                                     uint32_t rodata_end,
                                     int i_off,
                                     int ro_off);
#ifdef CONFIG_ESP32S3_DATA_CACHE_16KB
extern void cache_invalidate_dcache_all(void);
extern int cache_occupy_addr(uint32_t addr, uint32_t size);
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_APP_FORMAT_MCUBOOT
noreturn_function void __start(void);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_APP_FORMAT_MCUBOOT
HDR_ATTR static void (*_entry_point)(void) = __start;
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern uint8_t _rodata_reserved_start[];
extern uint8_t _rodata_reserved_end[];

/* Address of the CPU0 IDLE thread */

uint32_t g_idlestack[IDLETHREAD_STACKWORDS]
  aligned_data(16) locate_data(".noinit");

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: configure_cpu_caches
 *
 * Description:
 *   Configure the Instruction and Data CPU caches.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void IRAM_ATTR configure_cpu_caches(void)
{
  int s_instr_flash2spiram_off = 0;
  int s_rodata_flash2spiram_off = 0;

  /* Configure the mode of instruction cache: cache size, cache line size. */

  rom_config_instruction_cache_mode(CONFIG_ESP32S3_INSTRUCTION_CACHE_SIZE,
                                CONFIG_ESP32S3_ICACHE_ASSOCIATED_WAYS,
                                CONFIG_ESP32S3_INSTRUCTION_CACHE_LINE_SIZE);

  /* If we need to use SPIRAM, we should use data cache.
   * Configure the mode of data cache: cache size, cache line size.
   */

  cache_suspend_dcache();
  rom_config_data_cache_mode(CONFIG_ESP32S3_DATA_CACHE_SIZE,
                             CONFIG_ESP32S3_DCACHE_ASSOCIATED_WAYS,
                             CONFIG_ESP32S3_DATA_CACHE_LINE_SIZE);
  cache_resume_dcache(0);

  /* Configure the Cache MMU size for instruction and rodata in flash. */

  uint32_t rodata_reserved_start_align =
    (uint32_t)_rodata_reserved_start & ~(MMU_PAGE_SIZE - 1);
  uint32_t cache_mmu_irom_size =
    ((rodata_reserved_start_align - SOC_DROM_LOW) / MMU_PAGE_SIZE) *
      sizeof(uint32_t);

  uint32_t cache_mmu_drom_size =
    (((uint32_t)_rodata_reserved_end - rodata_reserved_start_align +
      MMU_PAGE_SIZE - 1) /
      MMU_PAGE_SIZE) * sizeof(uint32_t);

  cache_set_idrom_mmu_size(cache_mmu_irom_size,
                           CACHE_DROM_MMU_MAX_END - cache_mmu_irom_size);

  cache_set_idrom_mmu_info(cache_mmu_irom_size / sizeof(uint32_t),
                           cache_mmu_drom_size / sizeof(uint32_t),
                           (uint32_t)_rodata_reserved_start,
                           (uint32_t)_rodata_reserved_end,
                           s_instr_flash2spiram_off,
                           s_rodata_flash2spiram_off);

#ifdef CONFIG_ESP32S3_DATA_CACHE_16KB
  cache_invalidate_dcache_all();
  cache_occupy_addr(SOC_DROM_LOW, 0x4000);
#endif
}

/****************************************************************************
 * Name: disable_app_cpu
 *
 * Description:
 *   Disable the APP CPU (Core 1).
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifndef CONFIG_SMP
static void IRAM_ATTR disable_app_cpu(void)
{
  uint32_t regval;

  regval  = getreg32(SYSTEM_CORE_1_CONTROL_0_REG);
  regval &= ~SYSTEM_CONTROL_CORE_1_CLKGATE_EN;
  putreg32(regval, SYSTEM_CORE_1_CONTROL_0_REG);

  /* The clock gating signal of the App core is invalid.
   * We use RUNSTALL and RESETING signals to ensure that the App core stops
   * running in single-core mode.
   */

  regval  = getreg32(SYSTEM_CORE_1_CONTROL_0_REG);
  regval |= SYSTEM_CONTROL_CORE_1_RUNSTALL;
  putreg32(regval, SYSTEM_CORE_1_CONTROL_0_REG);

  regval  = getreg32(SYSTEM_CORE_1_CONTROL_0_REG);
  regval &= ~SYSTEM_CONTROL_CORE_1_RESETING;
  putreg32(regval, SYSTEM_CORE_1_CONTROL_0_REG);
}
#endif

/****************************************************************************
 * Name: __esp32s3_start
 *
 * Description:
 *   Perform base configuration of the chip for code execution.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void noreturn_function IRAM_ATTR __esp32s3_start(void)
{
  uint32_t sp;

  /* Make sure that normal interrupts are disabled.  This is really only an
   * issue when we are started in un-usual ways (such as from IRAM).  In this
   * case, we can at least defer some unexpected interrupts left over from
   * the last program execution.
   */

  up_irq_disable();

  /* Move the stack to a known location.  Although we were given a stack
   * pointer at start-up, we don't know where that stack pointer is
   * positioned with respect to our memory map.  The only safe option is to
   * switch to a well-known IDLE thread stack.
   */

  sp = (uint32_t)g_idlestack + IDLETHREAD_STACKSIZE;
  __asm__ __volatile__("mov sp, %0\n" : : "r"(sp));

  /* Raise an exception in case page 0 is accessed */

  esp32s3_region_protection();

  /* Move CPU0 exception vectors to IRAM */

  __asm__ __volatile__ ("wsr %0, vecbase\n"::"r" (_init_start));

  /* Clear .bss. We'll do this inline (vs. calling memset) just to be
   * certain that there are no issues with the state of global variables.
   */

  for (uint32_t *dest = (uint32_t *)_sbss; dest < (uint32_t *)_ebss; )
    {
      *dest++ = 0;
    }

#ifndef CONFIG_SMP
  /* Make sure that the APP_CPU is disabled for now */

  disable_app_cpu();
#endif

  /* The 2nd stage bootloader enables RTC WDT to check on startup sequence
   * related issues in application. Hence disable that as we are about to
   * start the NuttX environment.
   */

  esp32s3_wdt_early_deinit();

  /* Set CPU frequency configured in board.h */

  esp32s3_clockconfig();

#ifndef CONFIG_SUPPRESS_UART_CONFIG
  /* Configure the UART so we can get debug output */

  esp32s3_lowsetup();
#endif

#ifdef USE_EARLYSERIALINIT
  /* Perform early serial initialization */

  xtensa_earlyserialinit();
#endif

  showprogress('A');

#if defined(CONFIG_ESP32S3_FLASH_MODE_OCT) || \
    defined(CONFIG_ESP32S3_SPIRAM_MODE_OCT)
  esp_rom_opiflash_pin_config();
  esp32s3_spi_timing_set_pin_drive_strength();
#endif

#if defined(CONFIG_ESP32S3_SPIRAM_BOOT_INIT)
  if (esp_spiram_init() != OK)
    {
#  if defined(ESP32S3_SPIRAM_IGNORE_NOTFOUND)
      mwarn("SPIRAM Initialization failed!\n");
#  else
      PANIC();
#  endif
    }
  else
    {
      esp_spiram_init_cache();
      esp_spiram_test();
    }
#endif

  /* Setup the syscall table needed by the ROM code */

  esp_setup_syscall_table();

  /* Initialize onboard resources */

  esp32s3_board_initialize();

  showprogress('B');

#ifdef CONFIG_BUILD_PROTECTED
  /* For the case of the separate user-/kernel-space build, perform whatever
   * platform specific initialization of the user memory is required.
   * Normally this just means initializing the user space .data and .bss
   * segments.
   */

  esp32s3_userspace();
  showprogress('C');
#endif

  /* Bring up NuttX */

  nx_start();
  for (; ; ); /* Should not return */
}

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

#ifdef CONFIG_ESP32S3_APP_FORMAT_MCUBOOT
static inline uint32_t calc_mmu_pages(uint32_t size, uint32_t vaddr)
{
  return (size + (vaddr - (vaddr & MMU_FLASH_MASK)) + MMU_PAGE_SIZE - 1) /
    MMU_PAGE_SIZE;
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

#ifdef CONFIG_ESP32S3_APP_FORMAT_MCUBOOT
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

  uint32_t autoload = cache_suspend_dcache();
  cache_invalidate_dcache_all();

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
  rc = cache_dbus_mmu_set(MMU_ACCESS_FLASH, drom_vma_aligned,
                          drom_lma_aligned, 64, drom_page_count, 0);

  irom_lma_aligned = app_irom_lma & MMU_FLASH_MASK;
  irom_vma_aligned = app_irom_vma & MMU_FLASH_MASK;
  irom_page_count = calc_mmu_pages(app_irom_size, app_irom_vma);
  rc = cache_ibus_mmu_set(MMU_ACCESS_FLASH, irom_vma_aligned,
                          irom_lma_aligned, 64, irom_page_count, 0);

  regval  = getreg32(EXTMEM_DCACHE_CTRL1_REG);
  regval &= EXTMEM_DCACHE_SHUT_CORE0_BUS;
  putreg32(regval, EXTMEM_DCACHE_CTRL1_REG);

  cache_resume_dcache(autoload);

  return (int)rc;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __start
 *
 * Description:
 *   We arrive here after the bootloader finished loading the program from
 *   flash. The hardware is mostly uninitialized, and the app CPU is in
 *   reset. We do have a stack, so we can do the initialization in C.
 *
 *   The app CPU will remain in reset unless CONFIG_SMP is selected and
 *   up_cpu_start() is called later in the bring-up sequence.
 *
 ****************************************************************************/

void IRAM_ATTR __start(void)
{
#ifdef CONFIG_ESP32S3_APP_FORMAT_MCUBOOT
  if (map_rom_segments() != 0)
    {
      ets_printf("Failed to setup XIP, aborting\n");
      while (true);
    }

#endif
  configure_cpu_caches();

  __esp32s3_start();

  while (true); /* Should not return */
}
