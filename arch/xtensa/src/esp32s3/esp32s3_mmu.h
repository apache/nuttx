/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_mmu.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_MMU_H
#define __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_MMU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

#include "hardware/esp32s3_cache_memory.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Cache MMU address mask (MMU tables ignore bits which are zero) */

#define MMU_FLASH_MASK      (~(MMU_PAGE_SIZE - 1))

/* The cache MMU entry a virtual address resolves to.  There is one table for
 * both buses -- ext_mem_defs.h asserts that the IRAM0 and DRAM0 linear
 * addresses are equal -- so an instruction-bus and a data-bus address 64 MB
 * apart share an entry, and comparing entries is the only way to tell
 * whether two windows collide.
 */

#define MMU_ENTRY_OF(vaddr) (((vaddr) & SOC_MMU_VADDR_MASK) >> 16)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_dcache_suspend
 *
 * Description:
 *   Suspend the data cache access for the CPU, optionally writing back its
 *   contents first, then invalidating it.
 *
 * Input Parameters:
 *   needs_wb - Whether to write back the data cache contents prior to
 *              invalidation.
 *
 * Returned Value:
 *   Current cache state (to be passed to esp32s3_dcache_resume()).
 *
 ****************************************************************************/

uint32_t esp32s3_dcache_suspend(bool needs_wb);

/****************************************************************************
 * Name: esp32s3_dcache_resume
 *
 * Description:
 *   Resume the data cache access for the CPU.
 *
 * Input Parameters:
 *   cache_state - Previously saved data cache state to be restored.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32s3_dcache_resume(uint32_t cache_state);

/****************************************************************************
 * Name: esp32s3_icache_invalidate_all
 *
 * Description:
 *   Invalidate the whole instruction cache.
 *
 ****************************************************************************/

void esp32s3_icache_invalidate_all(void);

/****************************************************************************
 * Name: esp32s3_mmu_calc_pages
 *
 * Description:
 *   Calculate the required number of MMU pages for mapping a given region
 *   at the cache MMU 64 KB page granularity.
 *
 * Input Parameters:
 *   size  - Length of the region to map.
 *   vaddr - Starting offset to map (its intra-page offset is accounted for).
 *
 * Returned Value:
 *   Number of 64 KB MMU pages required.
 *
 ****************************************************************************/

uint32_t esp32s3_mmu_calc_pages(uint32_t size, uint32_t vaddr);

/****************************************************************************
 * Name: esp32s3_mmu_map_dbus
 *
 * Description:
 *   Program the data-bus cache MMU to map a range of 64 KB pages from an
 *   external memory (flash or PSRAM) into the CPU virtual address space.
 *   The caller is responsible for suspending/resuming the data cache around
 *   the mapping change.
 *
 * Input Parameters:
 *   ext_ram - Selects the external memory (SOC_MMU_ACCESS_FLASH or
 *             SOC_MMU_ACCESS_SPIRAM).
 *   vaddr   - 64 KB-aligned virtual base address.
 *   paddr   - 64 KB-aligned physical/flash offset.
 *   npages  - Number of 64 KB pages to map.
 *
 * Returned Value:
 *   Zero (OK) on success; the ROM error code otherwise.
 *
 ****************************************************************************/

int esp32s3_mmu_map_dbus(uint32_t ext_ram, uint32_t vaddr, uint32_t paddr,
                         uint32_t npages);

/****************************************************************************
 * Name: esp32s3_mmu_map_ibus
 *
 * Description:
 *   Program the instruction-bus cache MMU to map a range of 64 KB pages from
 *   an external memory into the CPU virtual address space.  The caller is
 *   responsible for suspending/resuming the data cache around the change.
 *
 * Input Parameters:
 *   ext_ram - Selects the external memory (SOC_MMU_ACCESS_FLASH or
 *             SOC_MMU_ACCESS_SPIRAM).
 *   vaddr   - 64 KB-aligned virtual base address.
 *   paddr   - 64 KB-aligned physical/flash offset.
 *   npages  - Number of 64 KB pages to map.
 *
 * Returned Value:
 *   Zero (OK) on success; the ROM error code otherwise.
 *
 ****************************************************************************/

int esp32s3_mmu_map_ibus(uint32_t ext_ram, uint32_t vaddr, uint32_t paddr,
                         uint32_t npages);

/****************************************************************************
 * Name: esp32s3_mmu_paddr
 *
 * Description:
 *   Ask the cache MMU what a virtual address currently resolves to.  This is
 *   ground truth rather than arithmetic, which matters because where the
 *   kernel's PSRAM window lands is decided at run time (see
 *   up_allocate_pgheap()).
 *
 * Input Parameters:
 *   vaddr - A virtual address inside one of the external memory windows.
 *   paddr - Receives the physical/flash address it maps to.
 *
 * Returned Value:
 *   True if the entry is valid and *paddr was written; false if the entry
 *   maps nothing.
 *
 ****************************************************************************/

bool esp32s3_mmu_paddr(uint32_t vaddr, uint32_t *paddr);

/****************************************************************************
 * Name: esp32s3_mmu_unmap
 *
 * Description:
 *   Invalidate a range of 64 KB cache MMU entries, so that the virtual
 *   addresses they covered map nothing at all.  Note that an access to an
 *   invalidated entry is not necessarily a fault: unless the cache reject
 *   monitors are armed it reads back as zero and swallows writes.  The
 *   caller is responsible for suspending/resuming the data cache around the
 *   change and for invalidating the instruction cache afterwards -- entries
 *   being taken away is exactly when stale instruction lines are left
 *   behind.
 *
 * Input Parameters:
 *   vaddr  - 64 KB-aligned virtual base address.
 *   npages - Number of 64 KB pages to invalidate.
 *
 ****************************************************************************/

void esp32s3_mmu_unmap(uint32_t vaddr, uint32_t npages);

/****************************************************************************
 * Name: esp32s3_mmu_scratch_map
 *
 * Description:
 *   Point one 64 KB data-bus entry at a PSRAM page and make it usable, doing
 *   the cache maintenance itself and confining it to that page.  This is the
 *   kernel's way of reaching a page-pool page, which is otherwise not mapped
 *   at all (see esp32s3_pgmap()).
 *
 *   The caller must already hold the scratch slot -- these entries are
 *   reachable by the unprivileged world like any other external memory
 *   address, so a mapping must not outlive the operation that needs it.
 *
 * Input Parameters:
 *   vaddr - 64 KB-aligned scratch virtual address.
 *   paddr - 64 KB-aligned PSRAM physical address.
 *
 ****************************************************************************/

void esp32s3_mmu_scratch_map(uint32_t vaddr, uint32_t paddr);

/****************************************************************************
 * Name: esp32s3_mmu_scratch_unmap
 *
 * Description:
 *   Take a scratch mapping away again: write back what was written through
 *   it, invalidate the entry, and drop the page's cache lines.
 *
 * Input Parameters:
 *   vaddr - The scratch virtual address returned when it was mapped.
 *
 ****************************************************************************/

void esp32s3_mmu_scratch_unmap(uint32_t vaddr);

#endif /* __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_MMU_H */
