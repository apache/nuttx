/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_userspace.c
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
#include <debug.h>
#include <stdint.h>
#include <stdlib.h>

#include <nuttx/userspace.h>

#include <arch/board/board_memorymap.h>

#include "riscv_internal.h"
#include "esp32c3.h"
#include "esp32c3_userspace.h"
#include "hardware/esp32c3_cache_memory.h"
#include "hardware/extmem_reg.h"

#ifdef CONFIG_BUILD_PROTECTED

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define USER_IMAGE_OFFSET   CONFIG_ESP32C3_USER_IMAGE_OFFSET

#define MMU_BLOCK0_VADDR    SOC_DROM_LOW
#define MMU_SIZE            0x3f0000
#define MMU_BLOCK63_VADDR   (MMU_BLOCK0_VADDR + MMU_SIZE)

/* Cache MMU block size */

#define MMU_BLOCK_SIZE      0x00010000  /* 64 KB */

/* Cache MMU address mask (MMU tables ignore bits which are zero) */

#define MMU_FLASH_MASK      (~(MMU_BLOCK_SIZE - 1))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct user_image_load_header_s
{
  uintptr_t drom_vma;      /* Destination address (VMA) for DROM region */
  uintptr_t drom_lma;      /* Flash offset (LMA) for start of DROM region */
  uintptr_t drom_size;     /* Size of DROM region */
  uintptr_t irom_vma;      /* Destination address (VMA) for IROM region */
  uintptr_t irom_lma;      /* Flash offset (LMA) for start of IROM region */
  uintptr_t irom_size;     /* Size of IROM region */
};

/****************************************************************************
 * ROM Function Prototypes
 ****************************************************************************/

extern uint32_t cache_suspend_icache(void);
extern void cache_resume_icache(uint32_t val);
extern void cache_invalidate_icache_all(void);
extern int cache_dbus_mmu_set(uint32_t ext_ram, uint32_t vaddr,
                              uint32_t paddr, uint32_t psize, uint32_t num,
                              uint32_t fixed);
extern int cache_ibus_mmu_set(uint32_t ext_ram, uint32_t vaddr,
                              uint32_t paddr, uint32_t psize, uint32_t num,
                              uint32_t fixed);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct user_image_load_header_s g_header;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: calc_mmu_pages
 *
 * Description:
 *   Calculate the required number of MMU pages for mapping a given region
 *   from External Flash into Internal RAM.
 *
 * Input Parameters:
 *   size          - Length of the region to map
 *   vaddr         - Starting External Flash offset to map to Internal RAM
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline uint32_t calc_mmu_pages(uint32_t size, uint32_t vaddr)
{
  return (size + (vaddr - (vaddr & MMU_FLASH_MASK)) + MMU_BLOCK_SIZE - 1) /
    MMU_BLOCK_SIZE;
}

/****************************************************************************
 * Name: configure_mmu
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

static noinline_function IRAM_ATTR void configure_mmu(void)
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

  uint32_t autoload = cache_suspend_icache();
  cache_invalidate_icache_all();

  drom_lma_aligned = app_drom_lma & MMU_FLASH_MASK;
  drom_vma_aligned = app_drom_vma & MMU_FLASH_MASK;
  drom_page_count = calc_mmu_pages(app_drom_size, app_drom_vma);
  DEBUGVERIFY(cache_dbus_mmu_set(MMU_ACCESS_FLASH, drom_vma_aligned,
                                 drom_lma_aligned, 64,
                                 (int)drom_page_count, 0));

  irom_lma_aligned = app_irom_lma & MMU_FLASH_MASK;
  irom_vma_aligned = app_irom_vma & MMU_FLASH_MASK;
  irom_page_count = calc_mmu_pages(app_irom_size, app_irom_vma);
  DEBUGVERIFY(cache_ibus_mmu_set(MMU_ACCESS_FLASH, irom_vma_aligned,
                                 irom_lma_aligned, 64,
                                 (int)irom_page_count, 0));

  cache_resume_icache(autoload);
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

  uint32_t autoload = cache_suspend_icache();
  cache_invalidate_icache_all();

  src_addr_aligned = src_addr & MMU_FLASH_MASK;
  page_count = calc_mmu_pages(size, src_addr);
  DEBUGVERIFY(cache_dbus_mmu_set(MMU_ACCESS_FLASH, MMU_BLOCK63_VADDR,
                                 src_addr_aligned, 64, (int)page_count, 0));

  cache_resume_icache(autoload);

  return (void *)(MMU_BLOCK63_VADDR + (src_addr - src_addr_aligned));
}

/****************************************************************************
 * Name: load_header
 *
 * Description:
 *   Load IROM and DROM information from image header to enable the correct
 *   configuration of the Flash MMU and Cache.
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
 * Name: initialize_data
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

static void initialize_data(void)
{
  uint8_t *dest;
  uint8_t *end;
  size_t length = USERSPACE->us_dataend - USERSPACE->us_datastart;
  const uint8_t *src =
    (const uint8_t *)map_flash(USER_IMAGE_OFFSET + USERSPACE->us_datasource,
                               length);

  DEBUGASSERT(src != NULL);

  dest = (uint8_t *)USERSPACE->us_datastart;
  end  = (uint8_t *)USERSPACE->us_dataend;

  while (dest != end)
    {
      *dest++ = *src++;
    }
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
  const uintptr_t R   = PMPCFG_R;
  const uintptr_t RW  = PMPCFG_R | PMPCFG_W;
  const uintptr_t RX  = PMPCFG_R | PMPCFG_X;
  const uintptr_t RWX = PMPCFG_R | PMPCFG_W | PMPCFG_X;

  /* Ensure PMP had not been previously configured by the bootloader */

  DEBUGASSERT(riscv_configured_pmp_regions() == 0);

  /* Region for the userspace read-only data in SPI Flash */

  riscv_config_pmp_region(0, PMPCFG_A_TOR, UDROM_START, 0);
  riscv_config_pmp_region(1, PMPCFG_A_TOR | R, UDROM_END, 0);

  /* Region for the userspace data.
   * NOTE: User-space heap may extend further than UDRAM_END.
   */

  riscv_config_pmp_region(2, PMPCFG_A_TOR, UDRAM_START, 0);
  riscv_config_pmp_region(3, PMPCFG_A_TOR | RW, SOC_DRAM_HIGH, 0);

  /* Region for the memory-mapped functions located in internal ROM */

  riscv_config_pmp_region(4, PMPCFG_L | PMPCFG_A_NAPOT | R,
                          SOC_DROM_MASK_LOW,
                          SOC_DROM_MASK_HIGH - SOC_DROM_MASK_LOW);

  riscv_config_pmp_region(5, PMPCFG_L | PMPCFG_A_TOR | RX,
                          SOC_IROM_MASK_HIGH,
                          0);

  /* Region for the exception vectors located in internal SRAM area reserved
   * for the kernel space.
   */

  riscv_config_pmp_region(6, PMPCFG_A_NAPOT | RX,
                          VECTORS_START,
                          VECTORS_END - VECTORS_START);

  /* Region for the userspace code located in internal SRAM */

  riscv_config_pmp_region(7, PMPCFG_A_TOR, UIRAM_START, 0);
  riscv_config_pmp_region(8, PMPCFG_A_TOR | RWX, UIRAM_END, 0);

  /* Region for the userspace code in SPI Flash */

  riscv_config_pmp_region(9, PMPCFG_A_TOR, UIROM_START, 0);
  riscv_config_pmp_region(10, PMPCFG_A_TOR | RX, UIROM_END, 0);

  /* Region for peripheral addresses */

  riscv_config_pmp_region(11, PMPCFG_A_TOR, SOC_PERIPHERAL_HIGH, 0);

  /* Region for the remainder of the address space */

  riscv_config_pmp_region(12, PMPCFG_L | PMPCFG_A_TOR, UINT32_MAX, 0);
  riscv_config_pmp_region(13, PMPCFG_L | PMPCFG_A_NA4, UINT32_MAX, 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32c3_userspace
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

void esp32c3_userspace(void)
{
  uint8_t *dest;
  uint8_t *end;

  /* Load IROM and DROM information from image header */

  load_header();

  /* Configure the MMU for enabling access to the userspace image */

  configure_mmu();

  /* Clear all of userspace .bss */

  DEBUGASSERT(USERSPACE->us_bssstart != 0 && USERSPACE->us_bssend != 0 &&
              USERSPACE->us_bssstart <= USERSPACE->us_bssend);

  dest = (uint8_t *)USERSPACE->us_bssstart;
  end  = (uint8_t *)USERSPACE->us_bssend;

  while (dest != end)
    {
      *dest++ = 0;
    }

  /* Initialize all of userspace .data */

  DEBUGASSERT(USERSPACE->us_datasource != 0 &&
              USERSPACE->us_datastart != 0 && USERSPACE->us_dataend != 0 &&
              USERSPACE->us_datastart <= USERSPACE->us_dataend);

  initialize_data();

  /* Configure MPU / PMP to grant access to the userspace */

  configure_mpu();
}

#endif /* CONFIG_BUILD_PROTECTED */
