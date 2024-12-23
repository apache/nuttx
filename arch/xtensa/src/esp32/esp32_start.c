/****************************************************************************
 * arch/xtensa/src/esp32/esp32_start.c
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

#include <debug.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>

#include <nuttx/init.h>
#include <nuttx/irq.h>

#include "xtensa.h"
#include "xtensa_attr.h"

#include "esp32_clockconfig.h"
#include "esp32_efuse.h"
#include "esp32_region.h"
#include "esp32_start.h"
#include "esp32_spiram.h"
#include "esp32_wdt.h"
#ifdef CONFIG_BUILD_PROTECTED
#  include "esp32_userspace.h"
#endif
#include "hardware/esp32_dport.h"
#include "hardware/esp32_rtccntl.h"
#include "rom/esp32_libc_stubs.h"
#include "espressif/esp_loader.h"

#ifdef CONFIG_ESPRESSIF_SIMPLE_BOOT
#  include "bootloader_init.h"
#  include "esp_rom_spiflash.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
#  define showprogress(c)     up_putc(c)
#else
#  define showprogress(c)
#endif

#if defined(CONFIG_ESP32_APP_FORMAT_MCUBOOT) || \
    defined (CONFIG_ESPRESSIF_SIMPLE_BOOT)
#  ifdef CONFIG_ESP32_APP_FORMAT_MCUBOOT
#    define PRIMARY_SLOT_OFFSET   CONFIG_ESP32_OTA_PRIMARY_SLOT_OFFSET
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

#if defined(CONFIG_ESP32_APP_FORMAT_MCUBOOT) || \
    defined (CONFIG_ESPRESSIF_SIMPLE_BOOT)
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

extern int ets_printf(const char *fmt, ...) printf_like(1, 2);

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#if defined(CONFIG_ESP32_APP_FORMAT_MCUBOOT) || \
    defined (CONFIG_ESPRESSIF_SIMPLE_BOOT)
noreturn_function void __start(void);
#endif

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#ifndef CONFIG_SUPPRESS_UART_CONFIG
extern void esp32_lowsetup(void);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if defined(CONFIG_ESP32_APP_FORMAT_MCUBOOT) || \
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

static noreturn_function void __esp32_start(void)
{
  uint32_t regval unused_data;
  uint32_t chip_rev;
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

  /* Make page 0 access raise an exception */

  esp32_region_protection();

#if defined(CONFIG_ESP32_PID) && defined(CONFIG_BUILD_PROTECTED)
  /* We have 2 VECBASE Addresses: one in CPU and one in DPORT peripheral.
   * CPU has no knowledge of PID hence any PID can change the CPU VECBASE
   * address thus jumping to malicious interrupt vectors with higher
   * privilege.
   * So we configure CPU to use the VECBASE address in DPORT peripheral.
   */

  regval = ((uint32_t)_init_start) >> 10;
  putreg32(regval, DPORT_PRO_VECBASE_SET_REG);

  regval  = getreg32(DPORT_PRO_VECBASE_CTRL_REG);
  regval |= BIT(0) | BIT(1);
  putreg32(regval, DPORT_PRO_VECBASE_CTRL_REG);
#else
  /* Move CPU0 exception vectors to IRAM */

  __asm__ __volatile__ ("wsr %0, vecbase\n"::"r"(_init_start));
#endif

  /* Set .bss to zero */

  memset(_sbss, 0, _ebss - _sbss);

#endif

#ifndef CONFIG_SMP
  /* Make sure that the APP_CPU is disabled for now */

  regval  = getreg32(DPORT_APPCPU_CTRL_B_REG);
  regval &= ~DPORT_APPCPU_CLKGATE_EN;
  putreg32(regval, DPORT_APPCPU_CTRL_B_REG);
#endif

  /* The 2nd stage bootloader enables RTC WDT to check on startup sequence
   * related issues in application. Hence disable that as we are about to
   * start the NuttX environment.
   */

  esp32_wdt_early_deinit();

  /* Set CPU frequency configured in board.h */

  esp32_clockconfig();

#ifndef CONFIG_SUPPRESS_UART_CONFIG
  /* Configure the UART so we can get debug output */

  esp32_lowsetup();
#endif

#ifdef USE_EARLYSERIALINIT
  /* Perform early serial initialization */

  xtensa_earlyserialinit();
#endif

  showprogress('A');

  chip_rev = esp_efuse_hal_chip_revision();

  _info("ESP32 chip revision is v%d.%01d\n", chip_rev / 100, chip_rev % 100);

  if (chip_rev < 300)
    {
#ifndef ESP32_IGNORE_CHIP_REVISION_CHECK
      ets_printf("ERROR: NuttX supports ESP32 chip revision >= v3.0"
                 " (chip revision is v%d.%01d)\n",
                 chip_rev / 100, chip_rev % 100);
      PANIC();
#endif
      ets_printf("WARNING: NuttX supports ESP32 chip revision >= v3.0"
                 " (chip is v%d.%01d).\n"
                 "Ignoring this error and continuing because "
                 "`ESP32_IGNORE_CHIP_REVISION_CHECK` is set...\n"
                 "THIS MAY NOT WORK! DON'T USE THIS CHIP IN PRODUCTION!\n",
                 chip_rev / 100, chip_rev % 100);
    }

#if defined(CONFIG_ESP32_SPIRAM_BOOT_INIT)
  if (esp_spiram_init() != OK)
    {
#  if defined(ESP32_SPIRAM_IGNORE_NOTFOUND)
      mwarn("SPIRAM Initialization failed!\n");
#  else
      PANIC();
#  endif
    }
  else
    {
      esp_spiram_init_cache();
      if (esp_spiram_test() != OK)
        {
          ets_printf("SPIRAM test failed\n");
          PANIC();
        }
    }

  /* Set external memory bss section to zero */

#  ifdef CONFIG_XTENSA_EXTMEM_BSS
     memset(_sbss_extmem, 0, _ebss_extmem - _sbss_extmem);
#  endif

#endif

  /* Setup the syscall table needed by the ROM code */

  esp_setup_syscall_table();

  /* Initialize onboard resources */

  esp32_board_initialize();

  showprogress('B');

  /* For the case of the separate user-/kernel-space build, perform whatever
   * platform specific initialization of the user memory is required.
   * Normally this just means initializing the user space .data and .bss
   * segments.
   */

#ifdef CONFIG_BUILD_PROTECTED
  esp32_userspace();
  showprogress('C');
#endif

  /* Bring up NuttX */

  nx_start();
  for (; ; ); /* Should not return */
}

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

noreturn_function void __start(void)
{
#if defined(CONFIG_ESP32_APP_FORMAT_MCUBOOT) || \
    defined (CONFIG_ESPRESSIF_SIMPLE_BOOT)
  size_t partition_offset = PRIMARY_SLOT_OFFSET;
  uint32_t app_irom_start = partition_offset + (uint32_t)_image_irom_lma;
  uint32_t app_irom_size  = (uint32_t)_image_irom_size;
  uint32_t app_irom_vaddr = (uint32_t)_image_irom_vma;
  uint32_t app_drom_start = partition_offset + (uint32_t)_image_drom_lma;
  uint32_t app_drom_size  = (uint32_t)_image_drom_size;
  uint32_t app_drom_vaddr = (uint32_t)_image_drom_vma;

#  ifdef CONFIG_ESPRESSIF_SIMPLE_BOOT
  uint32_t sp;

  /* Move CPU0 exception vectors to IRAM */

  __asm__ __volatile__ ("wsr %0, vecbase\n"::"r" (_init_start));

  /* Move the stack to a known location.  Although we were given a stack
   * pointer at start-up, we don't know where that stack pointer is
   * positioned with respect to our memory map.  The only safe option is to
   * switch to a well-known IDLE thread stack.
   */

  sp = (uint32_t)g_idlestack + IDLETHREAD_STACKSIZE;
  __asm__ __volatile__("mov sp, %0\n" : : "r"(sp));

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
  __esp32_start();
}
