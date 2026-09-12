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

#include <nuttx/irq.h>
#include <nuttx/nuttx.h>

#include "xtensa.h"
#include "esp_attr.h"

#include "soc/ext_mem_defs.h"
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
extern int cache_invalidate_addr(uint32_t addr, uint32_t size);
extern int cache_writeback_addr(uint32_t addr, uint32_t size);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dcache_unshut
 *
 * Description:
 *   Re-open the data cache bus that cache_suspend_dcache() shut.  Taken from
 *   esp_spiram_map(): the range cache maintenance that follows a remap has
 *   to run with the bus open but the cache still suspended, so this is the
 *   half of esp32s3_dcache_resume() that comes before cache_resume_dcache().
 *
 ****************************************************************************/

static void IRAM_ATTR dcache_unshut(void)
{
  uint32_t regval;

  regval = getreg32(EXTMEM_DCACHE_CTRL1_REG);
  regval &= ~EXTMEM_DCACHE_SHUT_CORE0_BUS;
#ifdef CONFIG_SMP
  regval &= ~EXTMEM_DCACHE_SHUT_CORE1_BUS;
#endif
  putreg32(regval, EXTMEM_DCACHE_CTRL1_REG);
}

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

/****************************************************************************
 * Name: esp32s3_mmu_paddr
 ****************************************************************************/

bool esp32s3_mmu_paddr(uint32_t vaddr, uint32_t *paddr)
{
  uint32_t entry = FLASH_MMU_TABLE[MMU_ENTRY_OF(vaddr)];

  if ((entry & MMU_TABLE_INVALID_VAL) != 0)
    {
      return false;
    }

  *paddr = (entry & MMU_ADDRESS_MASK) * MMU_PAGE_SIZE +
           (vaddr & (MMU_PAGE_SIZE - 1));
  return true;
}

/****************************************************************************
 * Name: esp32s3_mmu_unmap
 ****************************************************************************/

void IRAM_ATTR esp32s3_mmu_unmap(uint32_t vaddr, uint32_t npages)
{
  uint32_t entry = MMU_ENTRY_OF(vaddr);
  uint32_t i;

  /* One table, one entry per 64 KB, shared by the instruction and the data
   * bus: the IBUS and DBUS linear addresses are asserted equal in
   * ext_mem_defs.h, so a single write covers both views of the page.
   */

  for (i = 0; i < npages && entry + i < SOC_MMU_ENTRY_NUM; i++)
    {
      FLASH_MMU_TABLE[entry + i] = MMU_TABLE_INVALID_VAL;
    }
}

/****************************************************************************
 * Name: esp32s3_mmu_scratch_map
 ****************************************************************************/

void esp32s3_mmu_scratch_map(uint32_t vaddr, uint32_t paddr)
{
  irqstate_t flags;
  uint32_t   cache_state;

  flags = enter_critical_section();

  /* Suspend the cache, point the entry at the page, then invalidate just
   * this page's lines rather than the whole cache.  The difference matters:
   * esp32s3_dcache_suspend() invalidates everything, which would discard the
   * resident process's dirty lines, and its write-back variant would put a
   * whole-cache write-back inside this critical section.  Neither is
   * acceptable at the rate this is called -- once per page of every process
   * created.
   */

  cache_state = cache_suspend_dcache();

  esp32s3_mmu_map_dbus(SOC_MMU_ACCESS_SPIRAM, vaddr, paddr, 1);

  dcache_unshut();
  cache_invalidate_addr(vaddr, MMU_PAGE_SIZE);

  cache_resume_dcache(cache_state);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: esp32s3_mmu_scratch_unmap
 ****************************************************************************/

void esp32s3_mmu_scratch_unmap(uint32_t vaddr)
{
  irqstate_t flags;
  uint32_t   cache_state;

  /* Push whatever was written through this window out to the page it maps,
   * while it still maps it.  The lines are tagged by virtual address, so
   * repointing the entry with them still dirty would write them back to
   * whichever page the scratch slot is used for next.
   *
   * This runs with interrupts enabled on purpose -- it is a write-back of up
   * to a page, and the caller holds sched_lock(), so nothing else can reach
   * the slot in the meantime.
   */

  cache_writeback_addr(vaddr, MMU_PAGE_SIZE);

  flags = enter_critical_section();

  cache_state = cache_suspend_dcache();

  esp32s3_mmu_unmap(vaddr, 1);

  dcache_unshut();
  cache_invalidate_addr(vaddr, MMU_PAGE_SIZE);

  cache_resume_dcache(cache_state);

  leave_critical_section(flags);
}
