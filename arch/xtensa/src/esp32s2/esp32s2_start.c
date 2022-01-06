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
#include "xtensa_attr.h"

#include "hardware/esp32s2_cache_memory.h"
#include "hardware/esp32s2_extmem.h"
#include "esp32s2_clockconfig.h"
#include "esp32s2_region.h"
#include "esp32s2_start.h"
#include "esp32s2_lowputc.h"
#include "esp32s2_wdt.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
#  define showprogress(c)     up_lowputc(c)
#else
#  define showprogress(c)
#endif

#ifdef CONFIG_ESP32S2_APP_FORMAT_MCUBOOT

#define PRIMARY_SLOT_OFFSET   CONFIG_ESP32S2_OTA_PRIMARY_SLOT_OFFSET

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

#ifdef CONFIG_ESP32S2_APP_FORMAT_MCUBOOT
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

#ifdef CONFIG_ESP32S2_APP_FORMAT_MCUBOOT
extern int ets_printf(const char *fmt, ...);
extern uint32_t cache_suspend_icache(void);
extern void cache_resume_icache(uint32_t val);
extern void cache_invalidate_icache_all(void);
extern int cache_ibus_mmu_set(uint32_t ext_ram, uint32_t vaddr,
                              uint32_t paddr, uint32_t psize, uint32_t num,
                              uint32_t fixed);
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_ESP32S2_APP_FORMAT_MCUBOOT
noreturn_function void __start(void);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_ESP32S2_APP_FORMAT_MCUBOOT
HDR_ATTR static void (*_entry_point)(void) = &__start;
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

void IRAM_ATTR __esp32s2_start(void)
{
  uint32_t regval;
  uint32_t sp;

  regval  = getreg32(DR_REG_BB_BASE + 0x48); /* DR_REG_BB_BASE+48 */
  regval &= ~(1 << 14);
  putreg32(regval, DR_REG_BB_BASE + 0x48);

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

  esp32s2_region_protection();

  /* Move CPU0 exception vectors to IRAM */

  __asm__ __volatile__ ("wsr %0, vecbase\n"::"r" (&_init_start));

  /* Set .bss to zero */

  /* Clear .bss.  We'll do this inline (vs. calling memset) just to be
   * certain that there are no issues with the state of global variables.
   */

  for (uint32_t *dest = &_sbss; dest < &_ebss; dest++)
    {
      *dest = 0;
    }

  /* The 2nd stage bootloader enables RTC WDT to check on startup sequence
   * related issues in application. Hence disable that as we are about to
   * start the NuttX environment.
   */

  esp32s2_wdt_early_deinit();

  /* Set CPU frequency configured in board.h */

  esp32s2_clockconfig();

#ifndef CONFIG_SUPPRESS_UART_CONFIG
  /* Configure the UART so we can get debug output */

  esp32s2_lowsetup();
#endif

#ifdef USE_EARLYSERIALINIT
  /* Perform early serial initialization */

  xtensa_earlyserialinit();
#endif

  showprogress('A');

  /* Initialize onboard resources */

  esp32s2_board_initialize();

  showprogress('B');

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

#ifdef CONFIG_ESP32S2_APP_FORMAT_MCUBOOT
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

#ifdef CONFIG_ESP32S2_APP_FORMAT_MCUBOOT
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
  rc = cache_ibus_mmu_set(MMU_ACCESS_FLASH, drom_vma_aligned,
                           drom_lma_aligned, 64, (int)drom_page_count, 0);

  irom_lma_aligned = app_irom_lma & MMU_FLASH_MASK;
  irom_vma_aligned = app_irom_vma & MMU_FLASH_MASK;
  irom_page_count = calc_mmu_pages(app_irom_size, app_irom_vma);

  if (app_irom_lma + app_irom_size > IRAM1_ADDRESS_LOW)
    {
      rc |= cache_ibus_mmu_set(MMU_ACCESS_FLASH, IRAM0_ADDRESS_LOW, 0, 64,
                               64, 1);
      rc |= cache_ibus_mmu_set(MMU_ACCESS_FLASH, IRAM1_ADDRESS_LOW, 0, 64,
                               64, 1);

      regval  = getreg32(EXTMEM_PRO_ICACHE_CTRL1_REG);
      regval &= ~(EXTMEM_PRO_ICACHE_MASK_IRAM1);
      putreg32(regval, EXTMEM_PRO_ICACHE_CTRL1_REG);
    }

  rc |= cache_ibus_mmu_set(MMU_ACCESS_FLASH, irom_vma_aligned,
                           irom_lma_aligned, 64, (int)irom_page_count, 0);

  regval  = getreg32(EXTMEM_PRO_ICACHE_CTRL1_REG);
  regval &= ~(EXTMEM_PRO_ICACHE_MASK_IRAM0 |
              EXTMEM_PRO_ICACHE_MASK_DROM0);
  putreg32(regval, EXTMEM_PRO_ICACHE_CTRL1_REG);

  cache_resume_icache(autoload);

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
#ifdef CONFIG_ESP32S2_APP_FORMAT_MCUBOOT
  if (map_rom_segments() != 0)
    {
      ets_printf("Failed to setup XIP, aborting\n");
      while (true);
    }

#endif
  __esp32s2_start();

  while (true); /* Should not return */
}
