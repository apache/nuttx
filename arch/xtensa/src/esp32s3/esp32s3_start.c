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
#include "esp_attr.h"

#include "esp32s3_start.h"
#include "esp32s3_lowputc.h"
#include "esp32s3_clockconfig.h"
#include "esp32s3_region.h"
#include "esp32s3_rtc.h"
#include "esp32s3_spiram.h"
#include "esp32s3_wdt.h"
#ifdef CONFIG_BUILD_PROTECTED
#  include "esp32s3_userspace.h"
#endif
#include "esp32s3_spi_timing.h"
#include "hardware/esp32s3_cache_memory.h"
#include "hardware/esp32s3_system.h"
#include "rom/esp32s3_libc_stubs.h"
#include "rom/opi_flash.h"
#include "rom/esp32s3_spiflash.h"

#include "hal/mmu_hal.h"
#include "hal/mmu_types.h"
#include "hal/cache_types.h"
#include "hal/cache_ll.h"
#include "hal/cache_hal.h"
#include "soc/extmem_reg.h"
#include "rom/cache.h"
#include "spi_flash_mmap.h"

#ifdef CONFIG_ESPRESSIF_SIMPLE_BOOT
#  include "bootloader_init.h"
#  include "bootloader_flash_priv.h"
#  include "esp_rom_uart.h"
#  include "esp_rom_sys.h"
#  include "esp_app_format.h"
#endif

#include "esp_clk_internal.h"
#include "periph_ctrl.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
#  define showprogress(c)     xtensa_lowputc(c)
#else
#  define showprogress(c)
#endif

#if defined(CONFIG_ESP32S3_APP_FORMAT_MCUBOOT) || \
    defined (CONFIG_ESPRESSIF_SIMPLE_BOOT)
#  ifdef CONFIG_ESP32S3_APP_FORMAT_MCUBOOT
#    define PRIMARY_SLOT_OFFSET   CONFIG_ESP32S3_OTA_PRIMARY_SLOT_OFFSET
     /* Cache MMU address mask (MMU tables ignore bits which are zero) */

#    define MMU_FLASH_MASK        (~(MMU_PAGE_SIZE - 1))
#  else
    /* Force offset to the beginning of the whole image */

#    define PRIMARY_SLOT_OFFSET   0
#  endif
#  define HDR_ATTR              __attribute__((section(".entry_addr"))) \
                                __attribute__((used))
#  define MMU_BLOCK_SIZE        0x00010000  /* 64 KB */
#  define CACHE_REG             EXTMEM_ICACHE_CTRL1_REG
#  define CACHE_MASK            (EXTMEM_ICACHE_SHUT_IBUS_M | \
                                 EXTMEM_ICACHE_SHUT_DBUS_M)

#  define CHECKSUM_ALIGN        16
#  define IS_PADD(addr) (addr == 0)
#  define IS_DRAM(addr) (addr >= SOC_DRAM_LOW && addr < SOC_DRAM_HIGH)
#  define IS_IRAM(addr) (addr >= SOC_IRAM_LOW && addr < SOC_IRAM_HIGH)
#  define IS_IROM(addr) (addr >= SOC_IROM_LOW && addr < SOC_IROM_HIGH)
#  define IS_DROM(addr) (addr >= SOC_DROM_LOW && addr < SOC_DROM_HIGH)
#  define IS_SRAM(addr) (IS_IRAM(addr) || IS_DRAM(addr))
#  define IS_MMAP(addr) (IS_IROM(addr) || IS_DROM(addr))
#  ifdef SOC_RTC_FAST_MEM_SUPPORTED
#    define IS_RTC_FAST_IRAM(addr) \
                        (addr >= SOC_RTC_IRAM_LOW && addr < SOC_RTC_IRAM_HIGH)
#    define IS_RTC_FAST_DRAM(addr) \
                        (addr >= SOC_RTC_DRAM_LOW && addr < SOC_RTC_DRAM_HIGH)
#  else
#    define IS_RTC_FAST_IRAM(addr) 0
#    define IS_RTC_FAST_DRAM(addr) 0
#  endif
#  ifdef SOC_RTC_SLOW_MEM_SUPPORTED
#    define IS_RTC_SLOW_DRAM(addr) \
                        (addr >= SOC_RTC_DATA_LOW && addr < SOC_RTC_DATA_HIGH)
#  else
#    define IS_RTC_SLOW_DRAM(addr) 0
#  endif

#  define IS_NONE(addr) (!IS_IROM(addr) && !IS_DROM(addr) \
                      && !IS_IRAM(addr) && !IS_DRAM(addr) \
                      && !IS_RTC_FAST_IRAM(addr) && !IS_RTC_FAST_DRAM(addr) \
                      && !IS_RTC_SLOW_DRAM(addr) && !IS_PADD(addr))

#  define IS_MAPPING(addr) IS_IROM(addr) || IS_DROM(addr)

#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

#if defined(CONFIG_ESP32S3_APP_FORMAT_MCUBOOT) || \
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

#if defined(CONFIG_ESP32S3_APP_FORMAT_MCUBOOT) || \
    defined (CONFIG_ESPRESSIF_SIMPLE_BOOT)
extern int ets_printf(const char *fmt, ...) printf_like(1, 2);
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
extern int cache_occupy_addr(uint32_t addr, uint32_t size);
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#if defined(CONFIG_ESP32S3_APP_FORMAT_MCUBOOT) || \
    defined (CONFIG_ESPRESSIF_SIMPLE_BOOT)
IRAM_ATTR noreturn_function void __start(void);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if defined(CONFIG_ESP32S3_APP_FORMAT_MCUBOOT) || \
    defined (CONFIG_ESPRESSIF_SIMPLE_BOOT)
HDR_ATTR static void (*_entry_point)(void) = __start;
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern uint8_t _instruction_reserved_start[];
extern uint8_t _instruction_reserved_end[];
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

noinstrument_function static void IRAM_ATTR configure_cpu_caches(void)
{
  int s_instr_flash2spiram_off = 0;
  int s_rodata_flash2spiram_off = 0;
  uint32_t _instruction_size = (uint32_t)&_instruction_reserved_end -
                  (uint32_t)&_instruction_reserved_start;
  uint32_t cache_mmu_irom_size =
        ((_instruction_size + SPI_FLASH_MMU_PAGE_SIZE - 1) /
        SPI_FLASH_MMU_PAGE_SIZE) * sizeof(uint32_t);

  uint32_t _rodata_size = (uint32_t)&_rodata_reserved_end -
                  (uint32_t)&_rodata_reserved_start;
  uint32_t cache_mmu_drom_size =
        ((_rodata_size + SPI_FLASH_MMU_PAGE_SIZE - 1) /
        SPI_FLASH_MMU_PAGE_SIZE) * sizeof(uint32_t);

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

  cache_set_idrom_mmu_size(cache_mmu_irom_size,
                           CACHE_DROM_MMU_MAX_END - cache_mmu_irom_size);

#if CONFIG_SPIRAM_FETCH_INSTRUCTIONS
  s_instr_flash2spiram_off = instruction_flash2spiram_offset();
#endif
#if CONFIG_SPIRAM_RODATA
  s_rodata_flash2spiram_off = rodata_flash2spiram_offset();
#endif

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

noinstrument_function void noreturn_function IRAM_ATTR __esp32s3_start(void)
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

#ifndef CONFIG_ESPRESSIF_SIMPLE_BOOT
  /* Move CPU0 exception vectors to IRAM */

  __asm__ __volatile__ ("wsr %0, vecbase\n"::"r" (_init_start));

  /* Clear .bss. We'll do this inline (vs. calling memset) just to be
   * certain that there are no issues with the state of global variables.
   */

  for (uint32_t *dest = (uint32_t *)_sbss; dest < (uint32_t *)_ebss; )
    {
      *dest++ = 0;
    }
#endif

#ifndef CONFIG_SMP
  /* Make sure that the APP_CPU is disabled for now */

  disable_app_cpu();
#endif

  /* The 2nd stage bootloader enables RTC WDT to check on startup sequence
   * related issues in application. Hence disable that as we are about to
   * start the NuttX environment.
   */

  esp32s3_wdt_early_deinit();

  /* Initialize RTC controller parameters */

  esp32s3_rtc_init();
  esp32s3_rtc_clk_set();

  /* Set CPU frequency configured in board.h */

  esp32s3_clockconfig();

  /* Initialize peripherals parameters */

  esp_perip_clk_init();

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

  /* The PLL provided by bootloader is not stable enough, do calibration
   * again here so that we can use better clock for the timing tuning.
   */

#ifdef CONFIG_ESP32S3_SYSTEM_BBPLL_RECALIB
  esp32s3_rtc_recalib_bbpll();
#endif

  esp32s3_spi_timing_set_mspi_flash_tuning();
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

#if defined(CONFIG_ESP32S3_APP_FORMAT_MCUBOOT) || \
    defined(CONFIG_ESPRESSIF_SIMPLE_BOOT)
static int map_rom_segments(uint32_t app_drom_start, uint32_t app_drom_vaddr,
                            uint32_t app_drom_size, uint32_t app_irom_start,
                            uint32_t app_irom_vaddr, uint32_t app_irom_size)
{
  uint32_t rc = 0;
  uint32_t actual_mapped_len = 0;
  uint32_t app_irom_start_aligned = app_irom_start & MMU_FLASH_MASK;
  uint32_t app_irom_vaddr_aligned = app_irom_vaddr & MMU_FLASH_MASK;
  uint32_t app_drom_start_aligned = app_drom_start & MMU_FLASH_MASK;
  uint32_t app_drom_vaddr_aligned = app_drom_vaddr & MMU_FLASH_MASK;

#ifdef CONFIG_ESPRESSIF_SIMPLE_BOOT
  esp_image_header_t image_header; /* Header for entire image */
  esp_image_segment_header_t WORD_ALIGNED_ATTR segment_hdr;
  bool padding_checksum = false;
  unsigned int segments = 0;
  unsigned int ram_segments = 0;
  unsigned int rom_segments = 0;
  size_t offset = CONFIG_BOOTLOADER_OFFSET_IN_FLASH;

  /* Read image header */

  if (bootloader_flash_read(offset, &image_header,
                            sizeof(esp_image_header_t),
                            true) != ESP_OK)
    {
      ets_printf("Failed to load image header!\n");
      abort();
    }

  offset += sizeof(esp_image_header_t);

  /* Iterate for segment information parsing */

  while (segments++ < 16 && rom_segments < 2)
    {
      /* Read segment header */

      if (bootloader_flash_read(offset, &segment_hdr,
                                sizeof(esp_image_segment_header_t),
                                true) != ESP_OK)
        {
          ets_printf("failed to read segment header at %x\n", offset);
          abort();
        }

      if (IS_NONE(segment_hdr.load_addr))
        {
          break;
        }

      if (IS_RTC_FAST_IRAM(segment_hdr.load_addr) ||
          IS_RTC_FAST_DRAM(segment_hdr.load_addr) ||
          IS_RTC_SLOW_DRAM(segment_hdr.load_addr))
        {
          /* RTC segment is loaded by ROM bootloader */

          ram_segments++;
        }

      ets_printf("%s: lma 0x%08x vma 0x%08x len 0x%-6x (%u)\n",
          IS_NONE(segment_hdr.load_addr) ? "???" :
            IS_RTC_FAST_IRAM(segment_hdr.load_addr) ||
            IS_RTC_FAST_DRAM(segment_hdr.load_addr) ||
            IS_RTC_SLOW_DRAM(segment_hdr.load_addr) ? "rtc" :
              IS_MMAP(segment_hdr.load_addr) ?
                IS_IROM(segment_hdr.load_addr) ? "imap" : "dmap" :
                  IS_PADD(segment_hdr.load_addr) ? "padd" :
                    IS_DRAM(segment_hdr.load_addr) ? "dram" : "iram",
          offset + sizeof(esp_image_segment_header_t),
          segment_hdr.load_addr, segment_hdr.data_len,
          segment_hdr.data_len);

      /* Fix drom and irom produced be the linker, as this
       * is later invalidated by the elf2image command.
       */

      if (IS_DROM(segment_hdr.load_addr) &&
          segment_hdr.load_addr == (uint32_t)_image_drom_vma)
        {
          app_drom_start = offset + sizeof(esp_image_segment_header_t);
          app_drom_start_aligned = app_drom_start & MMU_FLASH_MASK;
          rom_segments++;
        }

      if (IS_IROM(segment_hdr.load_addr) &&
          segment_hdr.load_addr == (uint32_t)_image_irom_vma)
        {
          app_irom_start = offset + sizeof(esp_image_segment_header_t);
          app_irom_start_aligned = app_irom_start & MMU_FLASH_MASK;
          rom_segments++;
        }

      if (IS_SRAM(segment_hdr.load_addr))
        {
          ram_segments++;
        }

      offset += sizeof(esp_image_segment_header_t) + segment_hdr.data_len;
      if (ram_segments == image_header.segment_count && !padding_checksum)
        {
          offset += (CHECKSUM_ALIGN - 1) - (offset % CHECKSUM_ALIGN) + 1;
          padding_checksum = true;
        }
    }

  if (segments == 0 || segments == 16)
    {
      ets_printf("Error parsing segments\n");
    }

  ets_printf("total segments stored %d\n", segments - 1);
#endif

  cache_hal_disable(CACHE_TYPE_ALL);

  /* Clear the MMU entries that are already set up,
   * so the new app only has the mappings it creates.
   */

  mmu_hal_unmap_all();

  mmu_hal_map_region(0, MMU_TARGET_FLASH0,
                     app_drom_vaddr_aligned, app_drom_start_aligned,
                     app_drom_size, &actual_mapped_len);

  mmu_hal_map_region(0, MMU_TARGET_FLASH0,
                     app_irom_vaddr_aligned, app_irom_start_aligned,
                     app_irom_size, &actual_mapped_len);

  /* ------------------Enable corresponding buses--------------------- */

  cache_bus_mask_t bus_mask = cache_ll_l1_get_bus(0, app_drom_vaddr_aligned,
                                                  app_drom_size);
  cache_ll_l1_enable_bus(0, bus_mask);
  bus_mask = cache_ll_l1_get_bus(0, app_irom_vaddr_aligned, app_irom_size);
  cache_ll_l1_enable_bus(0, bus_mask);
#if CONFIG_ESPRESSIF_NUM_CPUS > 1
  bus_mask = cache_ll_l1_get_bus(1, app_drom_vaddr_aligned, app_drom_size);
  cache_ll_l1_enable_bus(1, bus_mask);
  bus_mask = cache_ll_l1_get_bus(1, app_irom_vaddr_aligned, app_irom_size);
  cache_ll_l1_enable_bus(1, bus_mask);
#endif

  /* ------------------Enable Cache----------------------------------- */

  cache_hal_enable(CACHE_TYPE_ALL);

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

noinstrument_function void IRAM_ATTR __start(void)
{
#if defined(CONFIG_ESP32S3_APP_FORMAT_MCUBOOT) || \
    defined(CONFIG_ESPRESSIF_SIMPLE_BOOT)
  size_t partition_offset = PRIMARY_SLOT_OFFSET;
  uint32_t app_irom_start = partition_offset + (uint32_t)_image_irom_lma;
  uint32_t app_irom_size  = (uint32_t)_image_irom_size;
  uint32_t app_irom_vaddr = (uint32_t)_image_irom_vma;
  uint32_t app_drom_start = partition_offset + (uint32_t)_image_drom_lma;
  uint32_t app_drom_size  = (uint32_t)_image_drom_size;
  uint32_t app_drom_vaddr = (uint32_t)_image_drom_vma;

#  ifdef CONFIG_ESPRESSIF_SIMPLE_BOOT
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

  configure_cpu_caches();

  __esp32s3_start();

  while (true); /* Should not return */
}
