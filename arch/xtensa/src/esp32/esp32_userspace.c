/****************************************************************************
 * arch/xtensa/src/esp32/esp32_userspace.c
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
#include "esp32_userspace.h"
#include "hardware/esp32_dport.h"
#include "hardware/esp32_pid.h"

#ifdef CONFIG_BUILD_PROTECTED

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define USER_IMAGE_OFFSET   CONFIG_ESP32_USER_IMAGE_OFFSET

#define MMU_BLOCK0_VADDR    SOC_DROM_LOW
#define MMU_SIZE            0x320000
#define MMU_BLOCK50_VADDR   (MMU_BLOCK0_VADDR + MMU_SIZE)

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

extern void cache_read_enable(int cpu);
extern void cache_read_disable(int cpu);
extern void cache_flush(int cpu);
extern unsigned int cache_flash_mmu_set(int cpu_no, int pid,
                                        unsigned int vaddr,
                                        unsigned int paddr,
                                        int psize, int num);

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

  cache_read_disable(0);
  cache_flush(0);

  drom_lma_aligned = app_drom_lma & MMU_FLASH_MASK;
  drom_vma_aligned = app_drom_vma & MMU_FLASH_MASK;
  drom_page_count = calc_mmu_pages(app_drom_size, app_drom_vma);
  ASSERT(cache_flash_mmu_set(0, PIDCTRL_PID_KERNEL, drom_vma_aligned,
                             drom_lma_aligned, 64,
                             (int)drom_page_count) == 0);
  ASSERT(cache_flash_mmu_set(1, PIDCTRL_PID_KERNEL, drom_vma_aligned,
                             drom_lma_aligned, 64,
                             (int)drom_page_count) == 0);
  ASSERT(cache_flash_mmu_set(0, PIDCTRL_PID_USER, drom_vma_aligned,
                             drom_lma_aligned, 64,
                             (int)drom_page_count) == 0);
  ASSERT(cache_flash_mmu_set(1, PIDCTRL_PID_USER, drom_vma_aligned,
                             drom_lma_aligned, 64,
                             (int)drom_page_count) == 0);

  irom_lma_aligned = app_irom_lma & MMU_FLASH_MASK;
  irom_vma_aligned = app_irom_vma & MMU_FLASH_MASK;
  irom_page_count = calc_mmu_pages(app_irom_size, app_irom_vma);
  ASSERT(cache_flash_mmu_set(0, PIDCTRL_PID_KERNEL, irom_vma_aligned,
                             irom_lma_aligned, 64,
                             (int)irom_page_count) == 0);
  ASSERT(cache_flash_mmu_set(1, PIDCTRL_PID_KERNEL, irom_vma_aligned,
                             irom_lma_aligned, 64,
                             (int)irom_page_count) == 0);
  ASSERT(cache_flash_mmu_set(0, PIDCTRL_PID_USER, irom_vma_aligned,
                             irom_lma_aligned, 64,
                             (int)irom_page_count) == 0);
  ASSERT(cache_flash_mmu_set(1, PIDCTRL_PID_USER, irom_vma_aligned,
                             irom_lma_aligned, 64,
                             (int)irom_page_count) == 0);

  cache_read_enable(0);
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

  cache_read_disable(0);
  cache_flush(0);

  src_addr_aligned = src_addr & MMU_FLASH_MASK;
  page_count = calc_mmu_pages(size, src_addr);
  ASSERT(cache_flash_mmu_set(0, PIDCTRL_PID_KERNEL, MMU_BLOCK50_VADDR,
                             src_addr_aligned, 64, (int)page_count) == 0);

  cache_read_enable(0);

  return (void *)(MMU_BLOCK50_VADDR + (src_addr - src_addr_aligned));
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
  /* 8K page size for mappings both IRAM and DRAM */

  REG_SET_FIELD(DPORT_IMMU_PAGE_MODE_REG, DPORT_IMMU_PAGE_MODE, 0);
  REG_SET_FIELD(DPORT_DMMU_PAGE_MODE_REG, DPORT_DMMU_PAGE_MODE, 0);

  /* 1:1 mapping for IRAM */

  REG_SET_FIELD(DPORT_IMMU_TABLE0_REG,  DPORT_IMMU_TABLE0,  0x00);
  REG_SET_FIELD(DPORT_IMMU_TABLE1_REG,  DPORT_IMMU_TABLE1,  0x51);
  REG_SET_FIELD(DPORT_IMMU_TABLE2_REG,  DPORT_IMMU_TABLE2,  0x52);
  REG_SET_FIELD(DPORT_IMMU_TABLE3_REG,  DPORT_IMMU_TABLE3,  0x53);
  REG_SET_FIELD(DPORT_IMMU_TABLE4_REG,  DPORT_IMMU_TABLE4,  0x54);
  REG_SET_FIELD(DPORT_IMMU_TABLE5_REG,  DPORT_IMMU_TABLE5,  0x55);
  REG_SET_FIELD(DPORT_IMMU_TABLE6_REG,  DPORT_IMMU_TABLE6,  0x56);
  REG_SET_FIELD(DPORT_IMMU_TABLE7_REG,  DPORT_IMMU_TABLE7,  0x57);
  REG_SET_FIELD(DPORT_IMMU_TABLE8_REG,  DPORT_IMMU_TABLE8,  0x08);
  REG_SET_FIELD(DPORT_IMMU_TABLE9_REG,  DPORT_IMMU_TABLE9,  0x09);
  REG_SET_FIELD(DPORT_IMMU_TABLE10_REG, DPORT_IMMU_TABLE10, 0x0a);
  REG_SET_FIELD(DPORT_IMMU_TABLE11_REG, DPORT_IMMU_TABLE11, 0x0b);
  REG_SET_FIELD(DPORT_IMMU_TABLE12_REG, DPORT_IMMU_TABLE12, 0x0c);
  REG_SET_FIELD(DPORT_IMMU_TABLE13_REG, DPORT_IMMU_TABLE13, 0x0d);
  REG_SET_FIELD(DPORT_IMMU_TABLE14_REG, DPORT_IMMU_TABLE14, 0x0e);
  REG_SET_FIELD(DPORT_IMMU_TABLE15_REG, DPORT_IMMU_TABLE15, 0x0f);

  REG_SET_FIELD(DPORT_DMMU_TABLE0_REG,  DPORT_DMMU_TABLE0,  0x00);
  REG_SET_FIELD(DPORT_DMMU_TABLE1_REG,  DPORT_DMMU_TABLE1,  0x01);
  REG_SET_FIELD(DPORT_DMMU_TABLE2_REG,  DPORT_DMMU_TABLE2,  0x02);
  REG_SET_FIELD(DPORT_DMMU_TABLE3_REG,  DPORT_DMMU_TABLE3,  0x03);
  REG_SET_FIELD(DPORT_DMMU_TABLE4_REG,  DPORT_DMMU_TABLE4,  0x54);
  REG_SET_FIELD(DPORT_DMMU_TABLE5_REG,  DPORT_DMMU_TABLE5,  0x55);
  REG_SET_FIELD(DPORT_DMMU_TABLE6_REG,  DPORT_DMMU_TABLE6,  0x56);
  REG_SET_FIELD(DPORT_DMMU_TABLE7_REG,  DPORT_DMMU_TABLE7,  0x57);
  REG_SET_FIELD(DPORT_DMMU_TABLE8_REG,  DPORT_DMMU_TABLE8,  0x58);
  REG_SET_FIELD(DPORT_DMMU_TABLE9_REG,  DPORT_DMMU_TABLE9,  0x59);
  REG_SET_FIELD(DPORT_DMMU_TABLE10_REG, DPORT_DMMU_TABLE10, 0x5a);
  REG_SET_FIELD(DPORT_DMMU_TABLE11_REG, DPORT_DMMU_TABLE11, 0x5b);
  REG_SET_FIELD(DPORT_DMMU_TABLE12_REG, DPORT_DMMU_TABLE12, 0x5c);
  REG_SET_FIELD(DPORT_DMMU_TABLE13_REG, DPORT_DMMU_TABLE13, 0x5d);
  REG_SET_FIELD(DPORT_DMMU_TABLE14_REG, DPORT_DMMU_TABLE14, 0x5e);
  REG_SET_FIELD(DPORT_DMMU_TABLE15_REG, DPORT_DMMU_TABLE15, 0x5f);

  /* Configure interrupt vector addresses in PID Controller */

  putreg32(PIDCTRL_INTERRUPT_ENABLE_M, PIDCTRL_INTERRUPT_ENABLE_REG);

  /* Configure the PID Controller to switch to privileged mode (PID 0) when
   * the CPU fetches instruction from the following interrupt vectors:
   * 1) Level 1
   * 2) Window Overflow 4
   * 3) Level 3 (SWINT)
   * 4) Window Underflow 4
   * 5) Window Overflow 8
   * 6) Window Underflow 8
   * 7) NMI
   */

  putreg32(VECTORS_START + 0x340, PIDCTRL_INTERRUPT_ADDR_1_REG);
  putreg32(VECTORS_START + 0x000, PIDCTRL_INTERRUPT_ADDR_2_REG);
  putreg32(VECTORS_START + 0x1c0, PIDCTRL_INTERRUPT_ADDR_3_REG);
  putreg32(VECTORS_START + 0x040, PIDCTRL_INTERRUPT_ADDR_4_REG);
  putreg32(VECTORS_START + 0x080, PIDCTRL_INTERRUPT_ADDR_5_REG);
  putreg32(VECTORS_START + 0x0c0, PIDCTRL_INTERRUPT_ADDR_6_REG);
  putreg32(VECTORS_START + 0x2c0, PIDCTRL_INTERRUPT_ADDR_7_REG);

  putreg32(0, PIDCTRL_LEVEL_REG);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_userspace
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

void esp32_userspace(void)
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
