/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_userspace.c
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

#include <assert.h>
#include <nuttx/debug.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <nuttx/nuttx.h>
#include <nuttx/userspace.h>

#include <arch/board/board_memorymap.h>

#include "chip.h"
#include "xtensa.h"
#include "esp_attr.h"
#include "esp_irq.h"
#include "esp32s3_userspace.h"
#include "esp32s3_mmu.h"
#include "esp32s3_pms.h"
#include "esp32s3_wcl.h"
#include "hardware/esp32s3_cache_memory.h"
#include "hardware/esp32s3_rom_layout.h"
#include "hardware/esp32s3_sensitive.h"
#include "hardware/esp32s3_soc.h"

#include "soc/extmem_reg.h"

#ifdef CONFIG_BUILD_PROTECTED

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define USER_IMAGE_OFFSET   CONFIG_ESP32S3_KERNEL_OFFSET + CONFIG_ESP32S3_KERNEL_IMAGE_SIZE

#define MMU_BLOCK0_VADDR    SOC_DROM_LOW
#define MMU_SIZE            0x3f0000
#define MMU_BLOCK63_VADDR   (MMU_BLOCK0_VADDR + MMU_SIZE)

/* Total addressable space is 1GB for the External Memories */

#define EXTMEM_MAX_LENGTH   0x40000000

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct user_image_load_header_s
{
  uintptr_t drom_vma;      /* Destination address (VMA) for DROM region */
  uintptr_t drom_lma;      /* Flash offset (LMA) for start of DROM region */
  uintptr_t drom_size;     /* Size of DROM region */
  uintptr_t iram_vma;      /* Destination address (VMA) for IRAM region */
  uintptr_t iram_lma;      /* Flash offset (LMA) for start of IRAM region */
  uintptr_t iram_size;     /* Size of IRAM region */
  uintptr_t irom_vma;      /* Destination address (VMA) for IROM region */
  uintptr_t irom_lma;      /* Flash offset (LMA) for start of IROM region */
  uintptr_t irom_size;     /* Size of IROM region */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct user_image_load_header_s g_header;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: configure_flash_mmu
 *
 * Description:
 *   Configure the External Flash MMU and Cache for enabling access to code
 *   and read-only data of the userspace image.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static noinline_function IRAM_ATTR void configure_flash_mmu(void)
{
  uint32_t drom_lma_aligned;
  uint32_t drom_vma_aligned;
  uint32_t drom_page_count;
  uint32_t irom_lma_aligned;
  uint32_t irom_vma_aligned;
  uint32_t irom_page_count;

  size_t partition_offset = USER_IMAGE_OFFSET;
  uint32_t app_drom_lma = partition_offset + g_header.drom_lma;
  uint32_t app_drom_size = g_header.drom_size;
  uint32_t app_drom_vma = g_header.drom_vma;
  uint32_t app_irom_lma = partition_offset + g_header.irom_lma;
  uint32_t app_irom_size = g_header.irom_size;
  uint32_t app_irom_vma = g_header.irom_vma;

  uint32_t cache_state = esp32s3_dcache_suspend(false);

  drom_lma_aligned = app_drom_lma & MMU_FLASH_MASK;
  drom_vma_aligned = app_drom_vma & MMU_FLASH_MASK;
  drom_page_count = esp32s3_mmu_calc_pages(app_drom_size, app_drom_vma);
  ASSERT(esp32s3_mmu_map_dbus(SOC_MMU_ACCESS_FLASH, drom_vma_aligned,
                              drom_lma_aligned, drom_page_count) == 0);

  irom_lma_aligned = app_irom_lma & MMU_FLASH_MASK;
  irom_vma_aligned = app_irom_vma & MMU_FLASH_MASK;
  irom_page_count = esp32s3_mmu_calc_pages(app_irom_size, app_irom_vma);
  ASSERT(esp32s3_mmu_map_ibus(SOC_MMU_ACCESS_FLASH, irom_vma_aligned,
                              irom_lma_aligned, irom_page_count) == 0);

  esp32s3_dcache_resume(cache_state);
}

/****************************************************************************
 * Name: map_flash
 *
 * Description:
 *   Map a region of the External Flash memory to Internal RAM.
 *
 * Input Parameters:
 *   src_addr      - Starting External Flash offset to map to Internal RAM
 *   size          - Length of the region to map
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static noinline_function IRAM_ATTR const void *map_flash(uint32_t src_addr,
                                                         uint32_t size)
{
  uint32_t src_addr_aligned;
  uint32_t page_count;

  uint32_t cache_state = esp32s3_dcache_suspend(false);

  src_addr_aligned = src_addr & MMU_FLASH_MASK;
  page_count = esp32s3_mmu_calc_pages(size, src_addr);

  ASSERT(esp32s3_mmu_map_dbus(SOC_MMU_ACCESS_FLASH, MMU_BLOCK63_VADDR,
                              src_addr_aligned, page_count) == 0);

  esp32s3_dcache_resume(cache_state);

  return (void *)(MMU_BLOCK63_VADDR + (src_addr - src_addr_aligned));
}

/****************************************************************************
 * Name: load_header
 *
 * Description:
 *   Load metadata information from image header to enable the correct
 *   configuration of the Flash MMU and Cache and initialization of the
 *   code and data located in Internal RAM.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void load_header(void)
{
  size_t length = sizeof(struct user_image_load_header_s);
  const uint8_t *data =
    (const uint8_t *)map_flash(USER_IMAGE_OFFSET, length);

  DEBUGASSERT(data != NULL);

  memcpy(&g_header, data, length);
}

/****************************************************************************
 * Name: initialize_dram
 *
 * Description:
 *   Initialize data sections of the userspace image.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void initialize_dram(void)
{
  uint8_t *dest;
  uint8_t *end;

  /* Clear all of userspace .bss */

  ASSERT(USERSPACE->us_bssstart != 0 && USERSPACE->us_bssend != 0 &&
         USERSPACE->us_bssstart <= USERSPACE->us_bssend);

  dest = (uint8_t *)USERSPACE->us_bssstart;
  end  = (uint8_t *)USERSPACE->us_bssend;

  while (dest != end)
    {
      *dest++ = 0;
    }

  /* Initialize all of userspace .data */

  ASSERT(USERSPACE->us_datasource != 0 && USERSPACE->us_datastart != 0 &&
         USERSPACE->us_dataend != 0 &&
         USERSPACE->us_datastart <= USERSPACE->us_dataend);

  size_t length = USERSPACE->us_dataend - USERSPACE->us_datastart;
  const uint8_t *src =
    (const uint8_t *)map_flash(USER_IMAGE_OFFSET + USERSPACE->us_datasource,
                               length);

  ASSERT(src != NULL);

  dest = (uint8_t *)USERSPACE->us_datastart;
  end  = (uint8_t *)USERSPACE->us_dataend;

  while (dest != end)
    {
      *dest++ = *src++;
    }
}

/****************************************************************************
 * Name: initialize_iram
 *
 * Description:
 *   Initialize instruction sections of the userspace image.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void initialize_iram(void)
{
  uint32_t *dest;
  uint32_t *end;
  const uint32_t *src =
    (const uint32_t *)map_flash(USER_IMAGE_OFFSET + g_header.iram_lma,
                                g_header.iram_size);

  ASSERT(src != NULL);

  /* This routine will initialize the IRAM region located in Internal SRAM0,
   * which can be accessible using 32-bit loads and stores and if the source
   * and destiny addresses and length are 32-bit aligned.
   */

  dest = (uint32_t *)g_header.iram_vma;
  end  = (uint32_t *)g_header.iram_vma + g_header.iram_size;

  while (dest != end)
    {
      *dest++ = *src++;
    }
}

/****************************************************************************
 * Name: pms_violation_isr
 *
 * Description:
 *   This is the common PMS interrupt handler. It will be invoked the PMS
 *   detects an access violation.
 *
 * Parameters:
 *   cpuint        - CPU interrupt index
 *   context       - Context data from the ISR
 *   arg           - Opaque pointer to the internal driver state structure.
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

static int IRAM_ATTR pms_violation_isr(int cpuint, void *context, void *arg)
{
  PANIC();

  return OK;
}

/****************************************************************************
 * Name: pms_enable_interrupts
 *
 * Description:
 *   Configure top level permission violation interrupts and set World
 *   Controller monitor registers.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void pms_enable_interrupts(void)
{
  /* We use two separate vector tables for WORLD0 and WORLD1,
   * this allows us to handle Windowed exceptions in WORLD1 itself, thus
   * saving CPU cycles.
   * For other vectors, we jump to WORLD0 vector table from WORLD1 vector
   * table.
   * The vector table for WORLD1 is placed right at the start of WORLD1 IRAM.
   */

  esp32s3_wcl_set_vecbase(PMS_WORLD_0, VECTORS_START);
  esp32s3_wcl_set_vecbase(PMS_WORLD_1, UIRAM_START);

  extern void _user_exception_vector(void);
  extern void _xtensa_level3_vector(void);

  esp32s3_wcl_set_world0_entry(1, (uintptr_t)_user_exception_vector);
  esp32s3_wcl_set_world0_entry(2, (uintptr_t)_xtensa_level3_vector);

  /* Enable IRAM0 permission violation interrupt */

  modifyreg32(SENSITIVE_CORE_0_IRAM0_PMS_MONITOR_1_REG,
              SENSITIVE_CORE_0_IRAM0_PMS_MONITOR_VIOLATE_CLR_M,
              SENSITIVE_CORE_0_IRAM0_PMS_MONITOR_VIOLATE_EN);

  /* Enable DRAM0 permission violation interrupt */

  modifyreg32(SENSITIVE_CORE_0_DRAM0_PMS_MONITOR_1_REG,
              SENSITIVE_CORE_0_DRAM0_PMS_MONITOR_VIOLATE_CLR_M,
              SENSITIVE_CORE_0_DRAM0_PMS_MONITOR_VIOLATE_EN);

  /* Enable Flash Instruction Cache permission violation interrupt */

  modifyreg32(EXTMEM_CORE0_ACS_CACHE_INT_CLR_REG,
              EXTMEM_CORE0_IBUS_REJECT_INT_CLR_M,
              0);
  modifyreg32(EXTMEM_CORE0_ACS_CACHE_INT_ENA_REG,
              EXTMEM_CORE0_IBUS_REJECT_INT_ENA_M,
              EXTMEM_CORE0_IBUS_REJECT_INT_ENA);

  /* Enable Flash Data Cache permission violation interrupt */

  modifyreg32(EXTMEM_CORE0_ACS_CACHE_INT_CLR_REG,
              EXTMEM_CORE0_DBUS_REJECT_INT_CLR_M,
              0);
  modifyreg32(EXTMEM_CORE0_ACS_CACHE_INT_ENA_REG,
              EXTMEM_CORE0_DBUS_REJECT_INT_ENA_M,
              EXTMEM_CORE0_DBUS_REJECT_INT_ENA);

  /* Enable Flash Data Cache permission violation interrupt */

  modifyreg32(SENSITIVE_CORE_0_PIF_PMS_MONITOR_1_REG,
              SENSITIVE_CORE_0_PIF_PMS_MONITOR_VIOLATE_CLR_M,
              SENSITIVE_CORE_0_PIF_PMS_MONITOR_VIOLATE_EN);
}

/****************************************************************************
 * Name: pms_configure_iram_access
 *
 * Description:
 *   Configure the access permissions to the IRAM region.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void pms_configure_iram_access(void)
{
  /* Kernel permission to the Instruction Cache */

  esp32s3_pms_configure_icache(PMS_AREA_0, PMS_WORLD_0, PMS_ACCESS_ALL);
  esp32s3_pms_configure_icache(PMS_AREA_1, PMS_WORLD_0, PMS_ACCESS_ALL);

  /* User permission to the Instruction Cache */

  esp32s3_pms_configure_icache(PMS_AREA_0, PMS_WORLD_1, PMS_ACCESS_NONE);
#ifdef CONFIG_ESP32S3_INSTRUCTION_CACHE_16KB
  /* In case the Instruction Cache size is configured to 16KB, the other 16KB
   * block from Internal SRAM0 (Block1) will be used as IRAM.
   */

  esp32s3_pms_configure_icache(PMS_AREA_1, PMS_WORLD_1, PMS_ACCESS_ALL);
#else /* CONFIG_ESP32S3_INSTRUCTION_CACHE_32KB */
  /* In case the Instruction Cache size is configured to 32KB, the WORLD1
   * access permissions to the Internal SRAM0 must be revoked.
   */

  esp32s3_pms_configure_icache(PMS_AREA_1, PMS_WORLD_1, PMS_ACCESS_NONE);
#endif

  /* Set split lines to partition the IRAM into regions */

  esp32s3_pms_set_iram_split_line(PMS_SPLIT_LINE_0, UIRAM_END);
  esp32s3_pms_set_iram_split_line(PMS_SPLIT_LINE_1, KIRAM_END);

  /* Configure Kernel access permissions to each split region */

  esp32s3_pms_configure_iram_region(PMS_AREA_0, PMS_WORLD_0, PMS_ACCESS_ALL);
  esp32s3_pms_configure_iram_region(PMS_AREA_1, PMS_WORLD_0, PMS_ACCESS_ALL);
  esp32s3_pms_configure_iram_region(PMS_AREA_2, PMS_WORLD_0, PMS_ACCESS_ALL);

  /* Configure User access permissions to each split region */

#ifdef CONFIG_ESP32S3_INSTRUCTION_CACHE_16KB
  /* In case the Instruction Cache size is configured to 16KB, the IRAM
   * allocation to WORLD1 will be entirely restricted to Internal SRAM0, so
   * we can safely revoke permissions to the shared Internal SRAM1.
   */

  esp32s3_pms_configure_iram_region(PMS_AREA_0, PMS_WORLD_1,
                                    PMS_ACCESS_NONE);
#else /* CONFIG_ESP32S3_INSTRUCTION_CACHE_32KB */
  /* In case the Instruction Cache size is configured to 32KB, the first
   * block from the shared Internal SRAM1 (Block2) will be allocated to
   * WORLD1.
   */

  esp32s3_pms_configure_iram_region(PMS_AREA_0, PMS_WORLD_1, PMS_ACCESS_ALL);
#endif
  esp32s3_pms_configure_iram_region(PMS_AREA_1, PMS_WORLD_1,
                                    PMS_ACCESS_NONE);
  esp32s3_pms_configure_iram_region(PMS_AREA_2, PMS_WORLD_1,
                                    PMS_ACCESS_NONE);

  /* PMS_AREA_3 corresponds to the region after the main split line,
   * i.e. the entire DRAM.
   */

  esp32s3_pms_configure_iram_region(PMS_AREA_3, PMS_WORLD_0,
                                    PMS_ACCESS_NONE);
  esp32s3_pms_configure_iram_region(PMS_AREA_3, PMS_WORLD_1,
                                    PMS_ACCESS_NONE);
}

/****************************************************************************
 * Name: pms_configure_dram_access
 *
 * Description:
 *   Configure the access permissions to the DRAM region.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void pms_configure_dram_access(void)
{
  /* Kernel permission to the Data Cache */

  esp32s3_pms_configure_dcache(PMS_WORLD_0, PMS_ACCESS_ALL);

  /* User permission to the Data Cache */

  esp32s3_pms_configure_dcache(PMS_WORLD_1, PMS_ACCESS_NONE);

  /* Split line to protect DRAM area reserved for ROM functions.
   * ALIGN_DOWN macro is used to align the address to 256 bit boundary.
   */

  uintptr_t rom_reserve_aligned =
    ALIGN_DOWN(ets_rom_layout_p->dram0_rtos_reserved_start, 256);

  /* Set split lines to partition the DRAM into regions */

  esp32s3_pms_set_dram_split_line(PMS_SPLIT_LINE_0, UDRAM_START);
  esp32s3_pms_set_dram_split_line(PMS_SPLIT_LINE_1, rom_reserve_aligned);

  /* PMS_AREA_0 corresponds to the region before the main split line,
   * i.e entire IRAM.
   */

  esp32s3_pms_configure_dram_region(PMS_AREA_0, PMS_WORLD_0,
                                    PMS_ACCESS_NONE);
  esp32s3_pms_configure_dram_region(PMS_AREA_0, PMS_WORLD_1,
                                    PMS_ACCESS_NONE);

  /* Configure Kernel access permissions to each split region */

  esp32s3_pms_configure_dram_region(PMS_AREA_1, PMS_WORLD_0, PMS_ACCESS_ALL);
  esp32s3_pms_configure_dram_region(PMS_AREA_2, PMS_WORLD_0, PMS_ACCESS_ALL);
  esp32s3_pms_configure_dram_region(PMS_AREA_3, PMS_WORLD_0, PMS_ACCESS_ALL);

  /* Configure User access permissions to each split region */

  esp32s3_pms_configure_dram_region(PMS_AREA_1, PMS_WORLD_1,
                                    PMS_ACCESS_NONE);
  esp32s3_pms_configure_dram_region(PMS_AREA_2, PMS_WORLD_1, PMS_ACCESS_ALL);
  esp32s3_pms_configure_dram_region(PMS_AREA_3, PMS_WORLD_1,
                                    PMS_ACCESS_NONE);
}

/****************************************************************************
 * Name: pms_configure_flash_cache_access
 *
 * Description:
 *   Configure the access permissions to the region dedicated to the
 *   Instruction and Data Caches.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static IRAM_ATTR void pms_configure_flash_cache_access(void)
{
  /* Invalidate Cache */

  uint32_t cache_state = esp32s3_dcache_suspend(true);

  esp32s3_icache_invalidate_all();

  size_t partition_offset = USER_IMAGE_OFFSET;

  /* Total image size must be aligned to the MMU page size (64 KB) to match
   * the required granularity of the PMS split regions for the External
   * Flash.
   * IROM region offset in External Flash plus its size provides an accurate
   * estimate about the User Image size.
   */

  size_t total_size = ALIGN_UP(g_header.irom_lma + g_header.irom_size,
                               MMU_PAGE_SIZE);
  size_t remaining_size = EXTMEM_MAX_LENGTH - partition_offset - total_size;

  uint32_t region0_start_addr = 0x0;
  uint32_t region0_size = partition_offset;
  uint32_t region1_start_addr = region0_start_addr + region0_size;
  uint32_t region1_size = total_size;
  uint32_t region2_start_addr = region1_start_addr + region1_size;
  uint32_t region2_size = remaining_size / 2;
  uint32_t region3_start_addr = region2_start_addr + region2_size;
  uint32_t region3_size = remaining_size / 2;

  esp32s3_pms_set_flash_cache_split_line(PMS_SPLIT_LINE_0,
                                         region0_start_addr, region0_size);
  esp32s3_pms_set_flash_cache_split_line(PMS_SPLIT_LINE_1,
                                         region1_start_addr, region1_size);
  esp32s3_pms_set_flash_cache_split_line(PMS_SPLIT_LINE_2,
                                         region2_start_addr, region2_size);
  esp32s3_pms_set_flash_cache_split_line(PMS_SPLIT_LINE_3,
                                         region3_start_addr, region3_size);

  /* Configure Kernel access permissions to each split region */

  esp32s3_pms_configure_flash_cache_region(PMS_AREA_0, PMS_WORLD_0,
                                           PMS_ACCESS_ALL);

  /* WORLD0 requires access to WORLD1 to load the cache when returning to
   * WORLD1 from WORLD0.
   */

  esp32s3_pms_configure_flash_cache_region(PMS_AREA_1, PMS_WORLD_0,
                                           PMS_ACCESS_ALL);
  esp32s3_pms_configure_flash_cache_region(PMS_AREA_2, PMS_WORLD_0,
                                           PMS_ACCESS_ALL);
  esp32s3_pms_configure_flash_cache_region(PMS_AREA_3, PMS_WORLD_0,
                                           PMS_ACCESS_ALL);

  /* Configure User access permissions to each split region */

  esp32s3_pms_configure_flash_cache_region(PMS_AREA_0, PMS_WORLD_1,
                                           PMS_ACCESS_NONE);
  esp32s3_pms_configure_flash_cache_region(PMS_AREA_1, PMS_WORLD_1,
                                           PMS_ACCESS_ALL);
  esp32s3_pms_configure_flash_cache_region(PMS_AREA_2, PMS_WORLD_1,
                                           PMS_ACCESS_NONE);
  esp32s3_pms_configure_flash_cache_region(PMS_AREA_3, PMS_WORLD_1,
                                           PMS_ACCESS_NONE);

  esp32s3_dcache_resume(cache_state);
}

/****************************************************************************
 * Name: pms_configure_peripheral_access
 *
 * Description:
 *   Configure Kernel and Userspace permissions for accessing the chip's
 *   peripherals.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void pms_configure_peripheral_access(void)
{
  /* Revoke User access permission to every peripheral */

  esp32s3_pms_configure_peripheral(PMS_UART1, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_I2C, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_MISC, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_IO_MUX, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_RTC, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_FE, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_FE2, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_GPIO, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_G0SPI_0, PMS_WORLD_1,
                                   PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_G0SPI_1, PMS_WORLD_1,
                                   PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_UART, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_SYSTIMER, PMS_WORLD_1,
                                   PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_TIMERGROUP1, PMS_WORLD_1,
                                   PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_TIMERGROUP, PMS_WORLD_1,
                                   PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_BB, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_LEDC, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_RMT, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_UHCI0, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_I2C_EXT0, PMS_WORLD_1,
                                   PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_BT, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_PWR, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_WIFIMAC, PMS_WORLD_1,
                                   PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_RWBT, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_I2S1, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_CAN, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_APB_CTRL, PMS_WORLD_1,
                                   PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_SPI_2, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_WORLD_CONTROLLER, PMS_WORLD_1,
                                   PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_DIO, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_AD, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_CACHE_CONFIG, PMS_WORLD_1,
                                   PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_DMA_COPY, PMS_WORLD_1,
                                   PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_INTERRUPT, PMS_WORLD_1,
                                   PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_SENSITIVE, PMS_WORLD_1,
                                   PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_SYSTEM, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_BT_PWR, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_APB_ADC, PMS_WORLD_1,
                                   PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_CRYPTO_DMA, PMS_WORLD_1,
                                   PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_CRYPTO_PERI, PMS_WORLD_1,
                                   PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_USB_WRAP, PMS_WORLD_1,
                                   PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_USB_DEVICE, PMS_WORLD_1,
                                   PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_I2S0, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_HINF, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_PWM0, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_BACKUP, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_SLC, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_PCNT, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_SLCHOST, PMS_WORLD_1,
                                   PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_UART2, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_PWM1, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_SDIO_HOST, PMS_WORLD_1,
                                   PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_I2C_EXT1, PMS_WORLD_1,
                                   PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_SPI_3, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_USB, PMS_WORLD_1, PMS_ACCESS_NONE);
}

/****************************************************************************
 * Name: configure_mpu
 *
 * Description:
 *   Configure the MPU for kernel/userspace separation.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void configure_mpu(void)
{
  /* Configure interrupts for permission violation */

  pms_enable_interrupts();

  /* Define the Internal SRAM1 regions for code and data by configuring the
   * split lines.
   */

  esp32s3_pms_set_sram_main_split_line(KIRAM_END);

  /* Configure Kernel and Userspace permissions for accessing the internal
   * memories.
   */

  /* IROM */

  esp32s3_pms_configure_irom_access();

  /* DROM */

  esp32s3_pms_configure_drom_access();

  /* IRAM */

  pms_configure_iram_access();

  /* DRAM */

  pms_configure_dram_access();

  /* Instruction and Data Caches */

  pms_configure_flash_cache_access();

  /* Configure Kernel and Userspace permissions for accessing the chip's
   * peripherals.
   */

  pms_configure_peripheral_access();
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_userspace
 *
 * Description:
 *   For the case of the separate user/kernel space build, perform whatever
 *   platform specific initialization of the user memory is required.
 *   Normally this just means initializing the userspace .data and .bss
 *   segments.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32s3_userspace(void)
{
  /* Load metadata information from image header */

  load_header();

  /* Configure the Flash MMU for enabling access to the userspace image */

  configure_flash_mmu();

  /* Initialize userspace DRAM */

  initialize_dram();

  /* Initialize userspace IRAM */

  initialize_iram();

  /* Configure MPU to grant access to the userspace */

  configure_mpu();
}

/****************************************************************************
 * Name: esp32s3_pmsirqinitialize
 *
 * Description:
 *   Initialize interrupt handler for the PMS violation ISR.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32s3_pmsirqinitialize(void)
{
  VERIFY(esp_setup_irq(ESP32S3_PERIPH_CORE_0_IRAM0_PMS_MONITOR_VIOLATE,
                       1, ESP_IRQ_TRIGGER_LEVEL, pms_violation_isr, NULL));
  VERIFY(esp_setup_irq(ESP32S3_PERIPH_CORE_0_DRAM0_PMS_MONITOR_VIOLATE,
                       1, ESP_IRQ_TRIGGER_LEVEL, pms_violation_isr, NULL));
  VERIFY(esp_setup_irq(ESP32S3_PERIPH_CACHE_CORE0_ACS,
                       1, ESP_IRQ_TRIGGER_LEVEL, pms_violation_isr, NULL));
  VERIFY(esp_setup_irq(ESP32S3_PERIPH_CORE_0_PIF_PMS_MONITOR_VIOLATE,
                       1, ESP_IRQ_TRIGGER_LEVEL, pms_violation_isr, NULL));

  up_enable_irq(ESP32S3_IRQ_CORE_0_IRAM0_PMS_MONITOR_VIOLATE);
  up_enable_irq(ESP32S3_IRQ_CORE_0_DRAM0_PMS_MONITOR_VIOLATE);
  up_enable_irq(ESP32S3_IRQ_CACHE_CORE0_ACS);
  up_enable_irq(ESP32S3_IRQ_CORE_0_PIF_PMS_MONITOR_VIOLATE);
}

#endif /* CONFIG_BUILD_PROTECTED */
