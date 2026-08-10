/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_mmu.c
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

#include <nuttx/nuttx.h>

#include "xtensa.h"
#include "esp_attr.h"

#include "soc/extmem_reg.h"

#include "esp32s3_mmu.h"

/****************************************************************************
 * ROM Function Prototypes
 ****************************************************************************/

extern uint32_t cache_suspend_dcache(void);
extern void cache_resume_dcache(uint32_t val);
extern void cache_invalidate_dcache_all(void);
extern void cache_invalidate_icache_all(void);
extern void cache_writeback_all(void);
extern int cache_dbus_mmu_set(uint32_t ext_ram, uint32_t vaddr,
                              uint32_t paddr, uint32_t psize, uint32_t num,
                              uint32_t fixed);
extern int cache_ibus_mmu_set(uint32_t ext_ram, uint32_t vaddr,
                              uint32_t paddr, uint32_t psize, uint32_t num,
                              uint32_t fixed);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_dcache_suspend
 ****************************************************************************/

uint32_t IRAM_ATTR esp32s3_dcache_suspend(bool needs_wb)
{
  uint32_t dcache_state = cache_suspend_dcache();

  if (needs_wb)
    {
      cache_writeback_all();
    }

  cache_invalidate_dcache_all();

  return dcache_state;
}

/****************************************************************************
 * Name: esp32s3_dcache_resume
 ****************************************************************************/

void IRAM_ATTR esp32s3_dcache_resume(uint32_t cache_state)
{
  uint32_t regval;

  regval = getreg32(EXTMEM_DCACHE_CTRL1_REG);
  regval &= ~EXTMEM_DCACHE_SHUT_CORE0_BUS;
#ifdef CONFIG_SMP
  regval &= ~EXTMEM_DCACHE_SHUT_CORE1_BUS;
#endif
  putreg32(regval, EXTMEM_DCACHE_CTRL1_REG);

  cache_resume_dcache(cache_state);
}

/****************************************************************************
 * Name: esp32s3_icache_invalidate_all
 ****************************************************************************/

void IRAM_ATTR esp32s3_icache_invalidate_all(void)
{
  cache_invalidate_icache_all();
}

/****************************************************************************
 * Name: esp32s3_mmu_calc_pages
 ****************************************************************************/

uint32_t esp32s3_mmu_calc_pages(uint32_t size, uint32_t vaddr)
{
  return (size + (vaddr - (vaddr & MMU_FLASH_MASK)) + MMU_PAGE_SIZE - 1) /
    MMU_PAGE_SIZE;
}

/****************************************************************************
 * Name: esp32s3_mmu_map_dbus
 ****************************************************************************/

int IRAM_ATTR esp32s3_mmu_map_dbus(uint32_t ext_ram, uint32_t vaddr,
                                   uint32_t paddr, uint32_t npages)
{
  return cache_dbus_mmu_set(ext_ram, vaddr, paddr, 64, (int)npages, 0);
}

/****************************************************************************
 * Name: esp32s3_mmu_map_ibus
 ****************************************************************************/

int IRAM_ATTR esp32s3_mmu_map_ibus(uint32_t ext_ram, uint32_t vaddr,
                                   uint32_t paddr, uint32_t npages)
{
  return cache_ibus_mmu_set(ext_ram, vaddr, paddr, 64, (int)npages, 0);
}
