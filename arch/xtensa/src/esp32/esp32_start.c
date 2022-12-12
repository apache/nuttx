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

#include <stdint.h>
#include <string.h>
#include <assert.h>

#include <nuttx/init.h>
#include <nuttx/irq.h>

#include "xtensa.h"
#include "xtensa_attr.h"

#include "esp32_clockconfig.h"
#include "esp32_region.h"
#include "esp32_start.h"
#include "esp32_spiram.h"
#include "esp32_wdt.h"
#ifdef CONFIG_BUILD_PROTECTED
#  include "esp32_userspace.h"
#endif
#include "hardware/esp32_dport.h"
#include "hardware/esp32_rtccntl.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
#  define showprogress(c)     up_puts(c)
#else
#  define showprogress(c)
#endif

#ifdef CONFIG_ESP32_APP_FORMAT_MCUBOOT

#define PRIMARY_SLOT_OFFSET   CONFIG_ESP32_OTA_PRIMARY_SLOT_OFFSET

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

#ifdef CONFIG_ESP32_APP_FORMAT_MCUBOOT
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

#ifdef CONFIG_ESP32_APP_FORMAT_MCUBOOT
extern int ets_printf(const char *fmt, ...) printf_like(1, 2);
extern void cache_read_enable(int cpu);
extern void cache_read_disable(int cpu);
extern void cache_flush(int cpu);
extern unsigned int cache_flash_mmu_set(int cpu_no, int pid,
                                        unsigned int vaddr,
                                        unsigned int paddr,
                                        int psize, int num);
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_ESP32_APP_FORMAT_MCUBOOT
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

#ifdef CONFIG_ESP32_APP_FORMAT_MCUBOOT
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
  uint32_t sp;
  uint32_t regval unused_data;

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

  showprogress("A");

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
    }

  /* Set external memory bss section to zero */

#  ifdef CONFIG_XTENSA_EXTMEM_BSS
     memset(_sbss_extmem, 0, _ebss_extmem - _sbss_extmem);
#  endif

#endif

  /* Initialize onboard resources */

  esp32_board_initialize();

  showprogress("B");

  /* For the case of the separate user-/kernel-space build, perform whatever
   * platform specific initialization of the user memory is required.
   * Normally this just means initializing the user space .data and .bss
   * segments.
   */

#ifdef CONFIG_BUILD_PROTECTED
  esp32_userspace();
  showprogress("C");
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

#ifdef CONFIG_ESP32_APP_FORMAT_MCUBOOT
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

#ifdef CONFIG_ESP32_APP_FORMAT_MCUBOOT
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

  volatile uint32_t *pro_flash_mmu_table =
    (volatile uint32_t *)DPORT_PRO_FLASH_MMU_TABLE_REG;

  cache_read_disable(0);
  cache_flush(0);

  /* Clear the MMU entries that are already set up, so the new app only has
   * the mappings it creates.
   */

  for (int i = 0; i < DPORT_FLASH_MMU_TABLE_SIZE; i++)
    {
      putreg32(DPORT_FLASH_MMU_TABLE_INVALID_VAL, pro_flash_mmu_table++);
    }

  drom_lma_aligned = app_drom_lma & MMU_FLASH_MASK;
  drom_vma_aligned = app_drom_vma & MMU_FLASH_MASK;
  drom_page_count = calc_mmu_pages(app_drom_size, app_drom_vma);
  rc  = cache_flash_mmu_set(0, 0, drom_vma_aligned, drom_lma_aligned, 64,
                            (int)drom_page_count);
  rc |= cache_flash_mmu_set(1, 0, drom_vma_aligned, drom_lma_aligned, 64,
                            (int)drom_page_count);

  irom_lma_aligned = app_irom_lma & MMU_FLASH_MASK;
  irom_vma_aligned = app_irom_vma & MMU_FLASH_MASK;
  irom_page_count = calc_mmu_pages(app_irom_size, app_irom_vma);
  rc |= cache_flash_mmu_set(0, 0, irom_vma_aligned, irom_lma_aligned, 64,
                            (int)irom_page_count);
  rc |= cache_flash_mmu_set(1, 0, irom_vma_aligned, irom_lma_aligned, 64,
                            (int)irom_page_count);

  regval  = getreg32(DPORT_PRO_CACHE_CTRL1_REG);
  regval &= ~(DPORT_PRO_CACHE_MASK_IRAM0 | DPORT_PRO_CACHE_MASK_DROM0 |
              DPORT_PRO_CACHE_MASK_DRAM1);
  putreg32(regval, DPORT_PRO_CACHE_CTRL1_REG);

  regval  = getreg32(DPORT_APP_CACHE_CTRL1_REG);
  regval &= ~(DPORT_APP_CACHE_MASK_IRAM0 | DPORT_APP_CACHE_MASK_DROM0 |
              DPORT_APP_CACHE_MASK_DRAM1);
  putreg32(regval, DPORT_APP_CACHE_CTRL1_REG);

  cache_read_enable(0);
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

void __start(void)
{
#ifdef CONFIG_ESP32_APP_FORMAT_MCUBOOT
  if (map_rom_segments() != 0)
    {
      ets_printf("Failed to setup XIP, aborting\n");
      while (true);
    }

#endif
  __esp32_start();
}
