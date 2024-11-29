/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_start.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <stdbool.h>
#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/init.h>

#include "riscv_internal.h"

#include "esp_irq.h"
#include "esp_libc_stubs.h"
#include "esp_lowputc.h"
#include "esp_start.h"

#include "esp_clk_internal.h"
#include "esp_private/rtc_clk.h"
#include "esp_cpu.h"
#include "esp_private/brownout.h"
#include "hal/wdt_hal.h"
#include "hal/mmu_hal.h"
#include "hal/mmu_types.h"
#include "hal/cache_types.h"
#include "hal/cache_ll.h"
#include "hal/cache_hal.h"
#include "soc/ext_mem_defs.h"
#include "soc/extmem_reg.h"
#include "soc/mmu.h"
#include "soc/reg_base.h"
#include "spi_flash_mmap.h"
#include "rom/cache.h"

#include "bootloader_init.h"

#ifdef CONFIG_ESPRESSIF_SIMPLE_BOOT
#include "bootloader_flash_priv.h"
#include "esp_rom_uart.h"
#include "esp_rom_sys.h"
#include "esp_app_format.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
#  define showprogress(c)     riscv_lowputc(c)
#else
#  define showprogress(c)
#endif

#if defined(CONFIG_ESPRESSIF_BOOTLOADER_MCUBOOT) || \
    defined (CONFIG_ESPRESSIF_SIMPLE_BOOT)
#ifdef CONFIG_ESPRESSIF_BOOTLOADER_MCUBOOT
#  define PRIMARY_SLOT_OFFSET   CONFIG_ESPRESSIF_OTA_PRIMARY_SLOT_OFFSET
#  define MMU_FLASH_MASK        (~(MMU_BLOCK_SIZE - 1))
#else
/* Force offset to the beginning of the whole image
 */

#  define PRIMARY_SLOT_OFFSET   0
#endif
#  define HDR_ATTR              __attribute__((section(".entry_addr"))) \
                                  __attribute__((used))
#  define FLASH_MMU_TABLE       ((volatile uint32_t*) DR_REG_MMU_TABLE)
#  define FLASH_MMU_TABLE_SIZE  (ICACHE_MMU_SIZE/sizeof(uint32_t))
#  define MMU_BLOCK_SIZE        0x00010000  /* 64 KB */
#  define CACHE_REG             EXTMEM_ICACHE_CTRL1_REG
#  define CACHE_MASK            (EXTMEM_ICACHE_SHUT_IBUS_M | \
                                 EXTMEM_ICACHE_SHUT_DBUS_M)

#  define CHECKSUM_ALIGN        16
#  define IS_PADD(addr) ((addr) == 0)
#  define IS_DRAM(addr) ((addr) >= SOC_DRAM_LOW && (addr) < SOC_DRAM_HIGH)
#  define IS_IRAM(addr) ((addr) >= SOC_IRAM_LOW && (addr) < SOC_IRAM_HIGH)
#  define IS_IROM(addr) ((addr) >= SOC_IROM_LOW && (addr) < SOC_IROM_HIGH)
#  define IS_DROM(addr) ((addr) >= SOC_DROM_LOW && (addr) < SOC_DROM_HIGH)
#  define IS_SRAM(addr) (IS_IRAM(addr) || IS_DRAM(addr))
#  define IS_MMAP(addr) (IS_IROM(addr) || IS_DROM(addr))
#  ifdef SOC_RTC_FAST_MEM_SUPPORTED
#    define IS_RTC_FAST_IRAM(addr) \
                        ((addr) >= SOC_RTC_IRAM_LOW \
                         && (addr) < SOC_RTC_IRAM_HIGH)
#    define IS_RTC_FAST_DRAM(addr) \
                        ((addr) >= SOC_RTC_DRAM_LOW \
                         && (addr) < SOC_RTC_DRAM_HIGH)
#  else
#    define IS_RTC_FAST_IRAM(addr) false
#    define IS_RTC_FAST_DRAM(addr) false
#  endif
#  ifdef SOC_RTC_SLOW_MEM_SUPPORTED
#    define IS_RTC_SLOW_DRAM(addr) \
                        ((addr) >= SOC_RTC_DATA_LOW \
                         && (addr) < SOC_RTC_DATA_HIGH)
#  else
#    define IS_RTC_SLOW_DRAM(addr) false
#  endif
#  define IS_NONE(addr) (!IS_IROM(addr) \
                         && !IS_DROM(addr) \
                         && !IS_IRAM(addr) \
                         && !IS_DRAM(addr) \
                         && !IS_RTC_FAST_IRAM(addr) \
                         && !IS_RTC_FAST_DRAM(addr) \
                         && !IS_RTC_SLOW_DRAM(addr) \
                         && !IS_PADD(addr))

#  define IS_MAPPING(addr) IS_IROM(addr) || IS_DROM(addr)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

#if defined(CONFIG_ESPRESSIF_BOOTLOADER_MCUBOOT) || \
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

#if defined(CONFIG_ESPRESSIF_BOOTLOADER_MCUBOOT) || \
    defined (CONFIG_ESPRESSIF_SIMPLE_BOOT)
extern int ets_printf(const char *fmt, ...) printf_like(1, 2);
#endif

extern void cache_set_idrom_mmu_size(uint32_t irom_size, uint32_t drom_size);

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#if defined(CONFIG_ESPRESSIF_BOOTLOADER_MCUBOOT) || \
    defined (CONFIG_ESPRESSIF_SIMPLE_BOOT)
IRAM_ATTR noreturn_function void __start(void);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if defined(CONFIG_ESPRESSIF_BOOTLOADER_MCUBOOT) || \
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

#if defined(CONFIG_ESPRESSIF_BOOTLOADER_MCUBOOT) || \
    defined (CONFIG_ESPRESSIF_SIMPLE_BOOT)
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

#if defined(CONFIG_ESPRESSIF_BOOTLOADER_MCUBOOT) || \
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
#endif

#ifdef CONFIG_ESPRESSIF_SIMPLE_BOOT

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
          /* Total segment count = (segments - 1) */

          break;
        }

      if (IS_RTC_FAST_IRAM(segment_hdr.load_addr) ||
          IS_RTC_FAST_DRAM(segment_hdr.load_addr) ||
          IS_RTC_SLOW_DRAM(segment_hdr.load_addr))
        {
          /* RTC segment is loaded by ROM bootloader */

          ram_segments++;
        }

      ets_printf("%s: lma 0x%08x vma 0x%08lx len 0x%-6lx (%lu)\n",
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
 * Name: __esp_start
 ****************************************************************************/

void __esp_start(void)
{
#ifdef CONFIG_ESP_ROM_NEEDS_SET_CACHE_MMU_SIZE
  uint32_t _instruction_size;
  uint32_t cache_mmu_irom_size;
#endif

#ifdef CONFIG_ESPRESSIF_SIMPLE_BOOT
  if (bootloader_init() != 0)
    {
      ets_printf("Hardware init failed, aborting\n");
      while (true);
    }
#else
  bootloader_clear_bss_section();
#endif

#if defined(CONFIG_ESPRESSIF_BOOTLOADER_MCUBOOT) || \
    defined(CONFIG_ESPRESSIF_SIMPLE_BOOT)
  size_t partition_offset = PRIMARY_SLOT_OFFSET;
  uint32_t app_irom_start = partition_offset + (uint32_t)_image_irom_lma;
  uint32_t app_irom_size  = (uint32_t)_image_irom_size;
  uint32_t app_irom_vaddr = (uint32_t)_image_irom_vma;
  uint32_t app_drom_start = partition_offset + (uint32_t)_image_drom_lma;
  uint32_t app_drom_size  = (uint32_t)_image_drom_size;
  uint32_t app_drom_vaddr = (uint32_t)_image_drom_vma;

  if (map_rom_segments(app_drom_start, app_drom_vaddr, app_drom_size,
                       app_irom_start, app_irom_vaddr, app_irom_size) != 0)
    {
      ets_printf("Failed to setup XIP, aborting\n");
      while (true);
    }
#endif

#if CONFIG_ESP_ROM_NEEDS_SET_CACHE_MMU_SIZE
  _instruction_size = (uint32_t)&_instruction_reserved_end - \
                      (uint32_t)&_instruction_reserved_start;
  cache_mmu_irom_size =
      ((_instruction_size + SPI_FLASH_MMU_PAGE_SIZE - 1) / \
      SPI_FLASH_MMU_PAGE_SIZE) * sizeof(uint32_t);

  /* Configure the Cache MMU size for instruction and rodata in flash. */

  cache_set_idrom_mmu_size(cache_mmu_irom_size,
                           CACHE_DROM_MMU_MAX_END - cache_mmu_irom_size);
#endif /* CONFIG_ESP_ROM_NEEDS_SET_CACHE_MMU_SIZE */

#if CONFIG_ESP_SYSTEM_BBPLL_RECALIB
  rtc_clk_recalib_bbpll();
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

  /* Setup the syscall table needed by the ROM code */

  esp_setup_syscall_table();

  showprogress('B');

  /* The 2nd stage bootloader enables RTC WDT to monitor any issues that may
   * prevent the startup sequence from finishing correctly. Hence disable it
   * as NuttX is about to start.
   */

  wdt_hal_context_t rwdt_ctx = RWDT_HAL_CONTEXT_DEFAULT();
  wdt_hal_write_protect_disable(&rwdt_ctx);
  wdt_hal_set_flashboot_en(&rwdt_ctx, false);
  wdt_hal_disable(&rwdt_ctx);
  wdt_hal_write_protect_enable(&rwdt_ctx);

  /* Initialize onboard resources */

  esp_board_initialize();

  showprogress('C');

  /* Bring up NuttX */

  nx_start();

  for (; ; );
}
