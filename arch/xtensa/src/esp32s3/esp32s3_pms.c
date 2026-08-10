/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_pms.c
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
#include <stddef.h>
#include <stdint.h>

#include <nuttx/nuttx.h>

#include "chip.h"
#include "xtensa.h"
#include "hardware/esp32s3_apb_ctrl.h"
#include "hardware/esp32s3_cache_memory.h"
#include "hardware/esp32s3_sensitive.h"
#include "hardware/esp32s3_soc.h"

#include "esp32s3_pms.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Helper just for shortening */

#define VALUE_TO_PMS_FIELD(v, f)  VALUE_TO_FIELD(v, SENSITIVE_CORE_X_ ## f)

/* Categories bits for split line configuration */

#define PMS_SRAM_CATEGORY_BELOW   0x0
#define PMS_SRAM_CATEGORY_EQUAL   0x2
#define PMS_SRAM_CATEGORY_ABOVE   0x3

/* Offsets for helping setting values to register fields */

#define ICACHE_PMS_W0_BASE        12
#define ICACHE_PMS_W1_BASE        12
#define ICACHE_PMS_S              3
#define ICACHE_PMS_V              7

#define IRAM_PMS_W0_BASE          0
#define IRAM_PMS_W1_BASE          0
#define IRAM_PMS_S                3
#define IRAM_PMS_V                7

#define DRAM_PMS_W0_BASE          0
#define DRAM_PMS_W1_BASE          12
#define DRAM_PMS_S                2
#define DRAM_PMS_V                3

#define FLASH_CACHE_S             3
#define FLASH_CACHE_V             7

#define PIF_PMS_MAX_REG_ENTRY     16
#define PIF_PMS_V                 3

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const intptr_t g_sram_rg3_level_hlimits[] =
{
  0x4037ffff, /* Block 2 (32KB) */
  0x4038ffff, /* Block 3 (64KB) */
  0x4039ffff, /* Block 4 (64KB) */
  0x403affff, /* Block 5 (64KB) */
  0x403bffff, /* Block 6 (64KB) */
  0x403cffff, /* Block 7 (64KB) */
  0x403dffff  /* Block 8 (64KB) */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: set_iram_split_line
 *
 * Description:
 *   Split the IRAM region into two sub regions.
 *
 * Input Parameters:
 *   addr          - Address for the split line.
 *   sensitive_reg - Register to which the split line configuration will be
 *                   applied.
 *
 ****************************************************************************/

static void set_iram_split_line(uintptr_t addr, const uint32_t sensitive_reg)
{
  /* Set category bits for a given split line */

  uint32_t cat[7] =
    {
      [0 ... 6] = PMS_SRAM_CATEGORY_ABOVE
    };

  for (size_t x = 0; x < 7; x++)
    {
      if (addr <= g_sram_rg3_level_hlimits[x])
        {
          cat[x] = PMS_SRAM_CATEGORY_EQUAL;
          break;
        }
      else
        {
          cat[x] = PMS_SRAM_CATEGORY_BELOW;
        }
    }

  /* Resolve split address' significant bits.
   * Split address must be aligned to 256 bytes.
   */

  uint32_t regval =
    VALUE_TO_PMS_FIELD((addr >> 8), IRAM0_DRAM0_DMA_SRAM_SPLITADDR) |
    VALUE_TO_PMS_FIELD(cat[6], IRAM0_DRAM0_DMA_SRAM_CATEGORY_6) |
    VALUE_TO_PMS_FIELD(cat[5], IRAM0_DRAM0_DMA_SRAM_CATEGORY_5) |
    VALUE_TO_PMS_FIELD(cat[4], IRAM0_DRAM0_DMA_SRAM_CATEGORY_4) |
    VALUE_TO_PMS_FIELD(cat[3], IRAM0_DRAM0_DMA_SRAM_CATEGORY_3) |
    VALUE_TO_PMS_FIELD(cat[2], IRAM0_DRAM0_DMA_SRAM_CATEGORY_2) |
    VALUE_TO_PMS_FIELD(cat[1], IRAM0_DRAM0_DMA_SRAM_CATEGORY_1) |
    VALUE_TO_PMS_FIELD(cat[0], IRAM0_DRAM0_DMA_SRAM_CATEGORY_0);

  putreg32(regval, sensitive_reg);
}

/****************************************************************************
 * Name: set_dram_split_line
 *
 * Description:
 *   Split the DRAM region into two sub regions.
 *
 * Input Parameters:
 *   addr          - Address for the split line.
 *   sensitive_reg - Register to which the split line configuration will be
 *                   applied.
 *
 ****************************************************************************/

static void set_dram_split_line(uintptr_t addr, const uint32_t sensitive_reg)
{
  /* Set category bits for a given split line */

  uint32_t cat[7] =
    {
      [0 ... 6] = PMS_SRAM_CATEGORY_ABOVE
    };

  for (size_t x = 0; x < 7; x++)
    {
      if (addr <= MAP_IRAM_TO_DRAM(g_sram_rg3_level_hlimits[x]))
        {
          cat[x] = PMS_SRAM_CATEGORY_EQUAL;
          break;
        }
      else
        {
          cat[x] = PMS_SRAM_CATEGORY_BELOW;
        }
    }

  /* Resolve split address' significant bits.
   * Split address must be aligned to 256 bytes.
   */

  uint32_t regval =
    VALUE_TO_PMS_FIELD((addr >> 8), DRAM0_DMA_SRAM_LINE_0_SPLITADDR) |
    VALUE_TO_PMS_FIELD(cat[6], DRAM0_DMA_SRAM_LINE_0_CATEGORY_6) |
    VALUE_TO_PMS_FIELD(cat[5], DRAM0_DMA_SRAM_LINE_0_CATEGORY_5) |
    VALUE_TO_PMS_FIELD(cat[4], DRAM0_DMA_SRAM_LINE_0_CATEGORY_4) |
    VALUE_TO_PMS_FIELD(cat[3], DRAM0_DMA_SRAM_LINE_0_CATEGORY_3) |
    VALUE_TO_PMS_FIELD(cat[2], DRAM0_DMA_SRAM_LINE_0_CATEGORY_2) |
    VALUE_TO_PMS_FIELD(cat[1], DRAM0_DMA_SRAM_LINE_0_CATEGORY_1) |
    VALUE_TO_PMS_FIELD(cat[0], DRAM0_DMA_SRAM_LINE_0_CATEGORY_0);

  putreg32(regval, sensitive_reg);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_pms_set_sram_main_split_line
 ****************************************************************************/

void esp32s3_pms_set_sram_main_split_line(uintptr_t addr)
{
  set_iram_split_line(addr,
                SENSITIVE_CORE_X_IRAM0_DRAM0_DMA_SPLIT_LINE_CONSTRAIN_1_REG);
}

/****************************************************************************
 * Name: esp32s3_pms_set_iram_split_line
 ****************************************************************************/

void esp32s3_pms_set_iram_split_line(enum pms_split_line_e line,
                                     uintptr_t addr)
{
  switch (line)
    {
      case PMS_SPLIT_LINE_0:
        {
          set_iram_split_line(addr,
                SENSITIVE_CORE_X_IRAM0_DRAM0_DMA_SPLIT_LINE_CONSTRAIN_2_REG);
        }
        break;
      case PMS_SPLIT_LINE_1:
        {
          set_iram_split_line(addr,
                SENSITIVE_CORE_X_IRAM0_DRAM0_DMA_SPLIT_LINE_CONSTRAIN_3_REG);
        }
        break;
      default:
        {
          PANIC();
        }
        break;
    }
}

/****************************************************************************
 * Name: esp32s3_pms_set_dram_split_line
 ****************************************************************************/

void esp32s3_pms_set_dram_split_line(enum pms_split_line_e line,
                                     uintptr_t addr)
{
  switch (line)
    {
      case PMS_SPLIT_LINE_0:
        {
          set_dram_split_line(addr,
                SENSITIVE_CORE_X_IRAM0_DRAM0_DMA_SPLIT_LINE_CONSTRAIN_4_REG);
        }
        break;
      case PMS_SPLIT_LINE_1:
        {
          set_dram_split_line(addr,
                SENSITIVE_CORE_X_IRAM0_DRAM0_DMA_SPLIT_LINE_CONSTRAIN_5_REG);
        }
        break;
      default:
        {
          PANIC();
        }
        break;
    }
}

/****************************************************************************
 * Name: esp32s3_pms_set_flash_cache_split_line
 ****************************************************************************/

void esp32s3_pms_set_flash_cache_split_line(enum pms_split_line_e line,
                                            uintptr_t addr, size_t length)
{
  /* The starting address of each region should be aligned to 64 KB */

  uintptr_t aligned_addr = ALIGN_DOWN(addr, MMU_PAGE_SIZE);

  /* The length of each region should be the integral multiples of 64 KB */

  size_t length_pages = length / MMU_PAGE_SIZE;

  switch (line)
    {
      case PMS_SPLIT_LINE_0:
        {
          modifyreg32(APB_CTRL_FLASH_ACE0_ADDR_REG,
                      APB_CTRL_FLASH_ACE0_ADDR_S_M,
                      VALUE_TO_FIELD(aligned_addr,
                                     APB_CTRL_FLASH_ACE0_ADDR_S));
          modifyreg32(APB_CTRL_FLASH_ACE0_SIZE_REG,
                      APB_CTRL_FLASH_ACE0_SIZE_M,
                      VALUE_TO_FIELD(length_pages,
                                     APB_CTRL_FLASH_ACE0_SIZE));
        }
        break;
      case PMS_SPLIT_LINE_1:
        {
          modifyreg32(APB_CTRL_FLASH_ACE1_ADDR_REG,
                      APB_CTRL_FLASH_ACE1_ADDR_S_M,
                      VALUE_TO_FIELD(aligned_addr,
                                     APB_CTRL_FLASH_ACE1_ADDR_S));
          modifyreg32(APB_CTRL_FLASH_ACE1_SIZE_REG,
                      APB_CTRL_FLASH_ACE1_SIZE_M,
                      VALUE_TO_FIELD(length_pages,
                                     APB_CTRL_FLASH_ACE1_SIZE));
        }
        break;
      case PMS_SPLIT_LINE_2:
        {
          modifyreg32(APB_CTRL_FLASH_ACE2_ADDR_REG,
                      APB_CTRL_FLASH_ACE2_ADDR_S_M,
                      VALUE_TO_FIELD(aligned_addr,
                                     APB_CTRL_FLASH_ACE2_ADDR_S));
          modifyreg32(APB_CTRL_FLASH_ACE2_SIZE_REG,
                      APB_CTRL_FLASH_ACE2_SIZE_M,
                      VALUE_TO_FIELD(length_pages,
                                     APB_CTRL_FLASH_ACE2_SIZE));
        }
        break;
      case PMS_SPLIT_LINE_3:
        {
          modifyreg32(APB_CTRL_FLASH_ACE3_ADDR_REG,
                      APB_CTRL_FLASH_ACE3_ADDR_S_M,
                      VALUE_TO_FIELD(aligned_addr,
                                     APB_CTRL_FLASH_ACE3_ADDR_S));
          modifyreg32(APB_CTRL_FLASH_ACE3_SIZE_REG,
                      APB_CTRL_FLASH_ACE3_SIZE_M,
                      VALUE_TO_FIELD(length_pages,
                                     APB_CTRL_FLASH_ACE3_SIZE));
        }
        break;
      default:
        {
          PANIC();
        }
        break;
    }
}

/****************************************************************************
 * Name: esp32s3_pms_configure_iram_region
 ****************************************************************************/

void esp32s3_pms_configure_iram_region(enum pms_area_e area,
                                       enum esp32s3_pms_world_e world,
                                       enum pms_flags_e flags)
{
  uint32_t reg;
  uint32_t offset;

  if (world == PMS_WORLD_0)
    {
      reg = SENSITIVE_CORE_X_IRAM0_PMS_CONSTRAIN_2_REG;
      offset = IRAM_PMS_W0_BASE;
    }
  else
    {
      reg = SENSITIVE_CORE_X_IRAM0_PMS_CONSTRAIN_1_REG;
      offset = IRAM_PMS_W1_BASE;
    }

  uint32_t shift = offset + (area * IRAM_PMS_S);
  uint32_t mask = IRAM_PMS_V << shift;
  uint32_t val = (flags & IRAM_PMS_V) << shift;

  modifyreg32(reg, mask, val);
}

/****************************************************************************
 * Name: esp32s3_pms_configure_icache
 ****************************************************************************/

void esp32s3_pms_configure_icache(enum pms_area_e area,
                                  enum esp32s3_pms_world_e world,
                                  enum pms_flags_e flags)
{
  uint32_t reg;
  uint32_t offset;

  if (world == PMS_WORLD_0)
    {
      reg = SENSITIVE_CORE_X_IRAM0_PMS_CONSTRAIN_2_REG;
      offset = ICACHE_PMS_W0_BASE;
    }
  else
    {
      reg = SENSITIVE_CORE_X_IRAM0_PMS_CONSTRAIN_1_REG;
      offset = ICACHE_PMS_W1_BASE;
    }

  uint32_t shift = offset + (area * ICACHE_PMS_S);
  uint32_t mask = ICACHE_PMS_V << shift;
  uint32_t val = (flags & ICACHE_PMS_V) << shift;

  modifyreg32(reg, mask, val);
}

/****************************************************************************
 * Name: esp32s3_pms_configure_dcache
 ****************************************************************************/

void esp32s3_pms_configure_dcache(enum esp32s3_pms_world_e world,
                                  enum pms_flags_e flags)
{
  switch (world)
    {
      case PMS_WORLD_0:
        {
          modifyreg32(SENSITIVE_CORE_X_DRAM0_PMS_CONSTRAIN_1_REG,
    SENSITIVE_CORE_X_DRAM0_PMS_CONSTRAIN_SRAM_WORLD_0_CACHEDATAARRAY_PMS_0_M
  | SENSITIVE_CORE_X_DRAM0_PMS_CONSTRAIN_SRAM_WORLD_0_CACHEDATAARRAY_PMS_1_M,
    VALUE_TO_PMS_FIELD(flags,
                    DRAM0_PMS_CONSTRAIN_SRAM_WORLD_0_CACHEDATAARRAY_PMS_0)
  | VALUE_TO_PMS_FIELD(flags,
                    DRAM0_PMS_CONSTRAIN_SRAM_WORLD_0_CACHEDATAARRAY_PMS_1));
        }
        break;
      case PMS_WORLD_1:
        {
          modifyreg32(SENSITIVE_CORE_X_DRAM0_PMS_CONSTRAIN_1_REG,
    SENSITIVE_CORE_X_DRAM0_PMS_CONSTRAIN_SRAM_WORLD_1_CACHEDATAARRAY_PMS_0_M
  | SENSITIVE_CORE_X_DRAM0_PMS_CONSTRAIN_SRAM_WORLD_1_CACHEDATAARRAY_PMS_1_M,
    VALUE_TO_PMS_FIELD(flags,
                    DRAM0_PMS_CONSTRAIN_SRAM_WORLD_1_CACHEDATAARRAY_PMS_0)
  | VALUE_TO_PMS_FIELD(flags,
                    DRAM0_PMS_CONSTRAIN_SRAM_WORLD_1_CACHEDATAARRAY_PMS_1));
        }
        break;
      default:
        {
          PANIC();
        }
        break;
    }
}

/****************************************************************************
 * Name: esp32s3_pms_configure_dram_region
 ****************************************************************************/

void esp32s3_pms_configure_dram_region(enum pms_area_e area,
                                       enum esp32s3_pms_world_e world,
                                       enum pms_flags_e flags)
{
  uint32_t offset;

  if (world == PMS_WORLD_0)
    {
      offset = DRAM_PMS_W0_BASE;
    }
  else
    {
      offset = DRAM_PMS_W1_BASE;
    }

  uint32_t shift = offset + (area * DRAM_PMS_S);
  uint32_t mask = DRAM_PMS_V << shift;
  uint32_t val = (flags & DRAM_PMS_V) << shift;

  modifyreg32(SENSITIVE_CORE_X_DRAM0_PMS_CONSTRAIN_1_REG, mask, val);
}

/****************************************************************************
 * Name: esp32s3_pms_configure_flash_cache_region
 ****************************************************************************/

void esp32s3_pms_configure_flash_cache_region(enum pms_area_e area,
                                              enum esp32s3_pms_world_e world,
                                              enum pms_flags_e flags)
{
  const uint32_t shift = (FLASH_CACHE_S * world);
  const uint32_t mask = FLASH_CACHE_V << shift;
  uint32_t attr;

  if (flags == PMS_ACCESS_ALL)
    {
      attr = 0b11;
    }
  else if ((flags & PMS_ACCESS_W) != 0)
    {
      PANIC();
    }
  else if ((flags & PMS_ACCESS_X) != 0)
    {
      attr = flags | 0b1;
    }
  else
    {
      attr = flags;
    }

  uint32_t val = 0x40 | (attr & FLASH_CACHE_V) << shift;

  switch (area)
    {
      case PMS_AREA_0:
        {
          modifyreg32(APB_CTRL_FLASH_ACE0_ATTR_REG, mask, val);
        }
        break;
      case PMS_AREA_1:
        {
          modifyreg32(APB_CTRL_FLASH_ACE1_ATTR_REG, mask, val);
        }
        break;
      case PMS_AREA_2:
        {
          modifyreg32(APB_CTRL_FLASH_ACE2_ATTR_REG, mask, val);
        }
        break;
      case PMS_AREA_3:
        {
          modifyreg32(APB_CTRL_FLASH_ACE3_ATTR_REG, mask, val);
        }
        break;
      default:
        {
          PANIC();
        }
        break;
    }
}

/****************************************************************************
 * Name: esp32s3_pms_configure_peripheral
 ****************************************************************************/

void esp32s3_pms_configure_peripheral(enum pms_peripheral_e periph,
                                      enum esp32s3_pms_world_e world,
                                      enum pms_flags_e flags)
{
  uint32_t reg = 0;
  uint32_t reg_off = periph / PIF_PMS_MAX_REG_ENTRY;
  uint32_t bit_field_base = 30 - (2 * (periph % PIF_PMS_MAX_REG_ENTRY));

  switch (world)
    {
      case PMS_WORLD_0:
        {
          reg = SENSITIVE_CORE_0_PIF_PMS_CONSTRAIN_1_REG + (4 * reg_off);
        }
        break;
      case PMS_WORLD_1:
        {
          reg = SENSITIVE_CORE_0_PIF_PMS_CONSTRAIN_5_REG + (4 * reg_off);
        }
        break;
      default:
        {
          PANIC();
        }
        break;
    }

  uint32_t mask = PIF_PMS_V << bit_field_base;
  uint32_t val = (flags & PIF_PMS_V) << bit_field_base;

  modifyreg32(reg, mask, val);
}

/****************************************************************************
 * Name: esp32s3_pms_configure_irom_access
 ****************************************************************************/

void esp32s3_pms_configure_irom_access(void)
{
  /* Kernel permission to the IROM region */

  modifyreg32(SENSITIVE_CORE_X_IRAM0_PMS_CONSTRAIN_2_REG,
              SENSITIVE_CORE_X_IRAM0_PMS_CONSTRAIN_ROM_WORLD_0_PMS_M,
              VALUE_TO_PMS_FIELD(PMS_ACCESS_ALL,
                                 IRAM0_PMS_CONSTRAIN_ROM_WORLD_0_PMS));

  /* User permission to the IROM region */

  modifyreg32(SENSITIVE_CORE_X_IRAM0_PMS_CONSTRAIN_1_REG,
              SENSITIVE_CORE_X_IRAM0_PMS_CONSTRAIN_ROM_WORLD_1_PMS_M,
              VALUE_TO_PMS_FIELD(PMS_ACCESS_NONE,
                                 IRAM0_PMS_CONSTRAIN_ROM_WORLD_1_PMS));
}

/****************************************************************************
 * Name: esp32s3_pms_configure_drom_access
 ****************************************************************************/

void esp32s3_pms_configure_drom_access(void)
{
  /* Kernel permission to the DROM region */

  modifyreg32(SENSITIVE_CORE_X_DRAM0_PMS_CONSTRAIN_1_REG,
              SENSITIVE_CORE_X_DRAM0_PMS_CONSTRAIN_ROM_WORLD_0_PMS_M,
              VALUE_TO_PMS_FIELD(PMS_ACCESS_ALL,
                                 DRAM0_PMS_CONSTRAIN_ROM_WORLD_0_PMS));

  /* User permission to the DROM region */

  modifyreg32(SENSITIVE_CORE_X_DRAM0_PMS_CONSTRAIN_1_REG,
              SENSITIVE_CORE_X_DRAM0_PMS_CONSTRAIN_ROM_WORLD_1_PMS_M,
              VALUE_TO_PMS_FIELD(PMS_ACCESS_NONE,
                                 DRAM0_PMS_CONSTRAIN_ROM_WORLD_1_PMS));
}
