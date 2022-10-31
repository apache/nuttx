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
#include <debug.h>
#include <stdint.h>
#include <stdlib.h>

#include <nuttx/userspace.h>

#include <arch/board/board_memorymap.h>

#include "chip.h"
#include "xtensa.h"
#include "xtensa_attr.h"
#include "esp32s3_userspace.h"
#include "hardware/esp32s3_cache_memory.h"

#ifdef CONFIG_BUILD_PROTECTED

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define USER_IMAGE_OFFSET   CONFIG_ESP32S3_USER_IMAGE_OFFSET

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

extern uint32_t cache_suspend_dcache(void);
extern void cache_resume_dcache(uint32_t val);
extern void cache_invalidate_dcache_all(void);
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

  uint32_t autoload = cache_suspend_dcache();
  cache_invalidate_dcache_all();

  drom_lma_aligned = app_drom_lma & MMU_FLASH_MASK;
  drom_vma_aligned = app_drom_vma & MMU_FLASH_MASK;
  drom_page_count = calc_mmu_pages(app_drom_size, app_drom_vma);
  ASSERT(cache_dbus_mmu_set(MMU_ACCESS_FLASH, drom_vma_aligned,
                            drom_lma_aligned, 64,
                            (int)drom_page_count, 0) == 0);

  irom_lma_aligned = app_irom_lma & MMU_FLASH_MASK;
  irom_vma_aligned = app_irom_vma & MMU_FLASH_MASK;
  irom_page_count = calc_mmu_pages(app_irom_size, app_irom_vma);
  ASSERT(cache_ibus_mmu_set(MMU_ACCESS_FLASH, irom_vma_aligned,
                            irom_lma_aligned, 64,
                            (int)irom_page_count, 0) == 0);

  cache_resume_dcache(autoload);
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

  uint32_t autoload = cache_suspend_dcache();
  cache_invalidate_dcache_all();

  src_addr_aligned = src_addr & MMU_FLASH_MASK;
  page_count = calc_mmu_pages(size, src_addr);

  ASSERT(cache_dbus_mmu_set(MMU_ACCESS_FLASH, MMU_BLOCK63_VADDR,
                            src_addr_aligned, 64, (int)page_count, 0) == 0);

  cache_resume_dcache(autoload);

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
  uint8_t *dest;
  uint8_t *end;

  /* Load IROM and DROM information from image header */

  load_header();

  /* Configure the Flash MMU for enabling access to the userspace image */

  configure_flash_mmu();

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

  /* Configure MPU to grant access to the userspace */

  configure_mpu();
}

#endif /* CONFIG_BUILD_PROTECTED */
