/****************************************************************************
 * arch/xtensa/src/esp32s2/esp32s2_start.c
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
#include "esp_attr.h"

#include "hardware/esp32s2_cache_memory.h"
#include "rom/esp32s2_libc_stubs.h"
#include "esp32s2_region.h"
#include "esp32s2_spiram.h"
#include "esp32s2_start.h"
#include "esp32s2_lowputc.h"
#include "esp32s2_wdt.h"
#include "espressif/esp_loader.h"

#include "soc/extmem_reg.h"
#include "hal/mmu_hal.h"
#include "hal/mmu_types.h"
#include "hal/cache_types.h"
#include "hal/cache_ll.h"
#include "hal/cache_hal.h"
#include "hal/sar_ctrl_ll.h"
#include "rom/spi_flash.h"
#include "esp_private/cache_utils.h"
#include "esp_private/startup_internal.h"

#ifdef CONFIG_ESPRESSIF_SIMPLE_BOOT
#  include "bootloader_init.h"
#endif

#include "bootloader_random.h"
#include "bootloader_soc.h"
#include "esp_clk_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
#  define showprogress(c)     xtensa_lowputc(c)
#else
#  define showprogress(c)
#endif

#if defined(CONFIG_ESP32S2_APP_FORMAT_MCUBOOT) || \
    defined (CONFIG_ESPRESSIF_SIMPLE_BOOT)
#  ifdef CONFIG_ESP32S2_APP_FORMAT_MCUBOOT
#    define PRIMARY_SLOT_OFFSET   CONFIG_ESPRESSIF_OTA_PRIMARY_SLOT_OFFSET
#  else
     /* Force offset to the beginning of the whole image */

#    define PRIMARY_SLOT_OFFSET   0x0000
#  endif
#  define HDR_ATTR              __attribute__((section(".entry_addr"))) \
                                __attribute__((used))

#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

#if defined(CONFIG_ESP32S2_APP_FORMAT_MCUBOOT) || \
    defined (CONFIG_ESPRESSIF_SIMPLE_BOOT)
extern uint8_t _image_irom_vma[];
extern uint8_t _image_irom_lma[];
extern uint8_t _image_irom_size[];

extern uint8_t _image_drom_vma[];
extern uint8_t _image_drom_lma[];
extern uint8_t _image_drom_size[];
#endif

#define CACHE_SIZE_8KB  CACHE_SIZE_HALF
#define CACHE_SIZE_16KB CACHE_SIZE_FULL

/****************************************************************************
 * ROM Function Prototypes
 ****************************************************************************/

#if defined(CONFIG_ESP32S2_APP_FORMAT_MCUBOOT) || \
    defined (CONFIG_ESPRESSIF_SIMPLE_BOOT)
extern int ets_printf(const char *fmt, ...) printf_like(1, 2);
#endif

extern uint32_t cache_suspend_icache(void);
extern void cache_resume_icache(uint32_t val);
extern void cache_invalidate_dcache_all(void);
extern void cache_invalidate_icache_all(void);
extern void cache_set_dcache_mode(cache_size_t cache_size, cache_ways_t ways,
                                  cache_line_size_t cache_line_size);
extern void cache_set_icache_mode(cache_size_t cache_size, cache_ways_t ways,
                                  cache_line_size_t cache_line_size);
extern void cache_allocate_sram(cache_layout_t sram0_layout,
                                cache_layout_t sram1_layout,
                                cache_layout_t sram2_layout,
                                cache_layout_t sram3_layout);
extern void esp_config_data_cache_mode(void);
extern void cache_enable_dcache(uint32_t autoload);
extern void esp_config_instruction_cache_mode(void);
extern void esp_config_data_cache_mode(void);

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#if defined(CONFIG_ESP32S2_APP_FORMAT_MCUBOOT) || \
    defined (CONFIG_ESPRESSIF_SIMPLE_BOOT)
noreturn_function void __start(void);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if defined(CONFIG_ESP32S2_APP_FORMAT_MCUBOOT) || \
    defined (CONFIG_ESPRESSIF_SIMPLE_BOOT)
HDR_ATTR static void (*_entry_point)(void) = __start;
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Address of the CPU0 IDLE thread */

uint32_t g_idlestack[IDLETHREAD_STACKWORDS]
  aligned_data(16) locate_data(".noinit");

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __esp32s2_start
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

static void noreturn_function IRAM_ATTR __esp32s2_start(void)
{
#ifndef CONFIG_ESPRESSIF_SIMPLE_BOOT
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

#ifndef CONFIG_ESP32S2_APP_FORMAT_MCUBOOT
  /* Make page 0 access raise an exception */

  esp32s2_region_protection();
#endif

  /* Move CPU0 exception vectors to IRAM */

  __asm__ __volatile__ ("wsr %0, vecbase\n"::"r" (_init_start));

  /* Clear .bss.  We'll do this inline (vs. calling memset) just to be
   * certain that there are no issues with the state of global variables.
   */

  for (uint32_t *dest = (uint32_t *)_sbss; dest < (uint32_t *)_ebss; )
    {
      *dest++ = 0;
    }
#endif

  /* Initialize peripherals parameters */

  esp_perip_clk_init();

  /* RNG is enabled during boot and must be disabled otherwise
   * Wi-Fi gets unstable.
   */

  bootloader_random_disable();
  bootloader_ana_clock_glitch_reset_config(false);

  /* The 2nd stage bootloader enables RTC WDT to check on startup sequence
   * related issues in application. Hence disable that as we are about to
   * start the NuttX environment.
   */

  esp32s2_wdt_early_deinit();

  /* Initialize RTC controller and set CPU frequency */

  esp_clk_init();

  /* Configure the UART so we can get debug output */

#ifndef CONFIG_SUPPRESS_UART_CONFIG
  esp32s2_lowsetup();
#endif

  esp_perip_clk_init();

#ifdef USE_EARLYSERIALINIT
  /* Perform early serial initialization */

  xtensa_earlyserialinit();
#endif

  showprogress('A');

#if defined(CONFIG_ESP32S2_SPIRAM_BOOT_INIT)
  if (esp_spiram_init() != OK)
    {
#  if defined(CONFIG_ESP32S2_SPIRAM_IGNORE_NOTFOUND)
      mwarn("SPIRAM Initialization failed!\n");
#  else
      PANIC();
#  endif
    }
  else
    {
      esp_spiram_init_cache();
#  if defined(CONFIG_ESP32S2_SPIRAM_MEMTEST)
      if (esp_spiram_test() != OK)
        {
            ets_printf("SPIRAM test failed\n");
            PANIC();
        }
#  endif // CONFIG_ESP32S2_SPIRAM_MEMTEST
    }
#endif

  /* Setup the syscall table needed by the ROM code */

  esp_setup_syscall_table();

  /* Initialize onboard resources */

  esp32s2_board_initialize();

  showprogress('B');

  /* Bring up NuttX */

  nx_start();
  for (; ; ); /* Should not return */
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xtensa_soc_initialize
 *
 * Description:
 *   Initialize SoC-specific initialization.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void weak_function xtensa_soc_initialize(void)
{
  sys_startup_fn();
}

/****************************************************************************
 * Name: sys_startup_fn
 *
 * Description:
 *   Execute the system layer startup function for the current CPU core.
 *   This function calls the appropriate startup function from the per-CPU
 *   startup function array (g_startup_fn) based on the current core ID.
 *   The SYS_STARTUP_FN() macro retrieves the core ID, indexes into the
 *   g_startup_fn array, and invokes the corresponding startup function.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sys_startup_fn(void)
{
  SYS_STARTUP_FN();
}

/****************************************************************************
 * Name: __start
 *
 * Description:
 *   We arrive here after the bootloader finished loading the program from
 *   flash. We do have a stack, so we can do the initialization in C.
 *
 ****************************************************************************/

noreturn_function void IRAM_ATTR __start(void)
{
#if defined(CONFIG_ESP32S2_APP_FORMAT_MCUBOOT) || \
    defined(CONFIG_ESPRESSIF_SIMPLE_BOOT)
  size_t partition_offset = PRIMARY_SLOT_OFFSET;
  uint32_t app_irom_start = partition_offset + (uint32_t)_image_irom_lma;
  uint32_t app_irom_size  = (uint32_t)_image_irom_size;
  uint32_t app_irom_vaddr = (uint32_t)_image_irom_vma;
  uint32_t app_drom_start = partition_offset + (uint32_t)_image_drom_lma;
  uint32_t app_drom_size  = (uint32_t)_image_drom_size;
  uint32_t app_drom_vaddr = (uint32_t)_image_drom_vma;

#  ifdef CONFIG_ESPRESSIF_SIMPLE_BOOT
  /* Move CPU0 exception vectors to IRAM */

  __asm__ __volatile__ ("wsr %0, vecbase\n"::"r" (_init_start));

  if (bootloader_init() != 0)
    {
      ets_printf("Hardware init failed, aborting\n");
      while (true);
    }
#  endif

  if (map_rom_segments(app_drom_start, app_drom_vaddr, app_drom_size,
                       app_irom_start, app_irom_vaddr, app_irom_size) != 0)
    {
      ets_printf("Failed to setup XIP, aborting\n");
      while (true);
    }

#endif

  /* Configure the mode of instruction cache : cache size, cache associated
   * ways, cache line size.
   */

  esp_config_instruction_cache_mode();

  /* If we need use SPIRAM, we should use data cache, or if we want to access
   * rodata, we also should use data cache.
   * Configure the mode of data : cache size, cache associated ways, cache
   * line size.
   * Enable data cache, so if we don't use SPIRAM, it just works.
   */

  esp_config_data_cache_mode();
  cache_enable_dcache(0);

  __esp32s2_start();

  while (true); /* Should not return */
}
