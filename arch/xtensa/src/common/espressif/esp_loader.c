/****************************************************************************
 * arch/xtensa/src/common/espressif/esp_loader.c
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

#include <stdint.h>
#include <string.h>

#include "xtensa.h"
#include "esp_attr.h"

#include "hal/mmu_hal.h"
#include "hal/mmu_types.h"
#include "hal/cache_types.h"
#include "hal/cache_ll.h"
#include "hal/cache_hal.h"
#include "rom/cache.h"
#include "spi_flash_mmap.h"

#ifndef CONFIG_ARCH_CHIP_ESP32
#  include "soc/extmem_reg.h"
#endif

#  include "bootloader_flash_priv.h"
#ifdef CONFIG_ESPRESSIF_SIMPLE_BOOT
#  include "bootloader_init.h"
#  include "esp_rom_uart.h"
#  include "esp_rom_sys.h"
#  include "esp_app_format.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HDR_ATTR              __attribute__((section(".entry_addr"))) \
                              __attribute__((used))
#define MMU_BLOCK_SIZE        0x00010000  /* 64 KB */
#define CACHE_REG             EXTMEM_ICACHE_CTRL1_REG
#define CACHE_MASK            (EXTMEM_ICACHE_SHUT_IBUS_M | \
                               EXTMEM_ICACHE_SHUT_DBUS_M)

#define CHECKSUM_ALIGN        16
#define IS_PADD(addr) (addr == 0)
#define IS_DRAM(addr) (addr >= SOC_DRAM_LOW && addr < SOC_DRAM_HIGH)
#define IS_IRAM(addr) (addr >= SOC_IRAM_LOW && addr < SOC_IRAM_HIGH)
#define IS_IROM(addr) (addr >= SOC_IROM_LOW && addr < SOC_IROM_HIGH)
#define IS_DROM(addr) (addr >= SOC_DROM_LOW && addr < SOC_DROM_HIGH)
#define IS_SRAM(addr) (IS_IRAM(addr) || IS_DRAM(addr))
#define IS_MMAP(addr) (IS_IROM(addr) || IS_DROM(addr))
#ifdef SOC_RTC_FAST_MEM_SUPPORTED
#  define IS_RTC_FAST_IRAM(addr) \
                      (addr >= SOC_RTC_IRAM_LOW && addr < SOC_RTC_IRAM_HIGH)
#  define IS_RTC_FAST_DRAM(addr) \
                      (addr >= SOC_RTC_DRAM_LOW && addr < SOC_RTC_DRAM_HIGH)
#else
#  define IS_RTC_FAST_IRAM(addr) 0
#  define IS_RTC_FAST_DRAM(addr) 0
#endif
#ifdef SOC_RTC_SLOW_MEM_SUPPORTED
#  define IS_RTC_SLOW_DRAM(addr) \
                      (addr >= SOC_RTC_DATA_LOW && addr < SOC_RTC_DATA_HIGH)
#else
#  define IS_RTC_SLOW_DRAM(addr) 0
#endif

#define IS_NONE(addr) (!IS_IROM(addr) && !IS_DROM(addr) \
                    && !IS_IRAM(addr) && !IS_DRAM(addr) \
                    && !IS_RTC_FAST_IRAM(addr) && !IS_RTC_FAST_DRAM(addr) \
                    && !IS_RTC_SLOW_DRAM(addr) && !IS_PADD(addr))

#define IS_MAPPING(addr) IS_IROM(addr) || IS_DROM(addr)

/****************************************************************************
 * Private Types
 ****************************************************************************/

extern uint8_t _image_irom_vma[];
extern uint8_t _image_irom_lma[];
extern uint8_t _image_irom_size[];

extern uint8_t _image_drom_vma[];
extern uint8_t _image_drom_lma[];
extern uint8_t _image_drom_size[];

/****************************************************************************
 * ROM Function Prototypes
 ****************************************************************************/

extern int ets_printf(const char *fmt, ...) printf_like(1, 2);

#ifdef CONFIG_ARCH_CHIP_ESP32
extern void cache_read_enable(int cpu);
extern void cache_read_disable(int cpu);
extern void cache_flush(int cpu);
extern unsigned int cache_flash_mmu_set(int cpu_no, int pid,
                                        unsigned int vaddr,
                                        unsigned int paddr,
                                        int psize, int num);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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

int map_rom_segments(uint32_t app_drom_start, uint32_t app_drom_vaddr,
                     uint32_t app_drom_size, uint32_t app_irom_start,
                     uint32_t app_irom_vaddr, uint32_t app_irom_size)
{
  uint32_t rc = 0;
  uint32_t actual_mapped_len = 0;
  uint32_t app_irom_start_aligned = app_irom_start & MMU_FLASH_MASK;
  uint32_t app_irom_vaddr_aligned = app_irom_vaddr & MMU_FLASH_MASK;
  uint32_t app_drom_start_aligned = app_drom_start & MMU_FLASH_MASK;
  uint32_t app_drom_vaddr_aligned = app_drom_vaddr & MMU_FLASH_MASK;

#ifdef CONFIG_ARCH_CHIP_ESP32
  uint32_t drom_page_count = 0;
  uint32_t irom_page_count = 0;
#endif

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

#if defined (CONFIG_ESP32S2_APP_FORMAT_MCUBOOT) || \
    defined (CONFIG_ESP32S3_APP_FORMAT_MCUBOOT) || \
    defined (CONFIG_ESP32_APP_FORMAT_MCUBOOT)
  ets_printf("IROM segment aligned lma 0x%08x vma 0x%08x len 0x%06x (%u)\n",
      app_irom_start_aligned, app_irom_vaddr_aligned,
      app_irom_size, app_irom_size);
  ets_printf("DROM segment aligned lma 0x%08x vma 0x%08x len 0x%06x (%u)\n",
      app_drom_start_aligned, app_drom_vaddr_aligned,
      app_drom_size, app_drom_size);
#endif

#ifdef CONFIG_ARCH_CHIP_ESP32
  cache_read_disable(PRO_CPU_NUM);
  cache_flush(PRO_CPU_NUM);
#  ifdef CONFIG_SMP
  cache_flush(APP_CPU_NUM);
  cache_read_enable(APP_CPU_NUM);
#  endif
#else
  cache_hal_disable(CACHE_TYPE_ALL);
#endif

  /* Clear the MMU entries that are already set up,
   * so the new app only has the mappings it creates.
   */

  mmu_hal_unmap_all();

#ifdef CONFIG_ARCH_CHIP_ESP32
  drom_page_count = (app_drom_size + SPI_FLASH_MMU_PAGE_SIZE - 1) /
                              SPI_FLASH_MMU_PAGE_SIZE;
  rc  = cache_flash_mmu_set(0, 0, app_drom_vaddr_aligned,
                            app_drom_start_aligned, 64,
                            (int)drom_page_count);
  rc |= cache_flash_mmu_set(1, 0, app_drom_vaddr_aligned,
                            app_drom_start_aligned, 64,
                            (int)drom_page_count);

  irom_page_count = (app_irom_size + SPI_FLASH_MMU_PAGE_SIZE - 1) /
                              SPI_FLASH_MMU_PAGE_SIZE;
  rc |= cache_flash_mmu_set(0, 0, app_irom_vaddr_aligned,
                            app_irom_start_aligned, 64,
                            (int)irom_page_count);
  rc |= cache_flash_mmu_set(1, 0, app_irom_vaddr_aligned,
                            app_irom_start_aligned, 64,
                            (int)irom_page_count);
#else
  mmu_hal_map_region(0, MMU_TARGET_FLASH0,
                     app_drom_vaddr_aligned, app_drom_start_aligned,
                     app_drom_size, &actual_mapped_len);

  mmu_hal_map_region(0, MMU_TARGET_FLASH0,
                     app_irom_vaddr_aligned, app_irom_start_aligned,
                     app_irom_size, &actual_mapped_len);
#endif

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

#ifdef CONFIG_ARCH_CHIP_ESP32
  cache_read_enable(0);
#else
  cache_hal_enable(CACHE_TYPE_ALL);
#endif
  return (int)rc;
}
