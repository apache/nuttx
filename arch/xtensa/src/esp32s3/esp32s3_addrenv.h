/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_addrenv.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_ADDRENV_H
#define __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_ADDRENV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/addrenv.h>
#include <nuttx/pgalloc.h>

#ifdef CONFIG_ARCH_ADDRENV

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_ARCH_PGPOOL_MAPPING
#  error "ESP32-S3 address environments need CONFIG_ARCH_PGPOOL_MAPPING"
#endif

/* The user address space is split across two disjoint cache-MMU windows:
 * .text lives in the instruction-bus window, .data/.bss and the heap in the
 * data-bus window.  Each window is described by its base and page count.
 */

#define ESP32S3_TEXT_VBASE  (CONFIG_ARCH_TEXT_VBASE)
#define ESP32S3_TEXT_VEND   (CONFIG_ARCH_TEXT_VBASE + \
                             CONFIG_ARCH_TEXT_NPAGES * MM_PGSIZE)
#define ESP32S3_DATA_VBASE  (CONFIG_ARCH_DATA_VBASE)
#define ESP32S3_DATA_VEND   (CONFIG_ARCH_DATA_VBASE + \
                             CONFIG_ARCH_DATA_NPAGES * MM_PGSIZE)
#define ESP32S3_HEAP_VBASE  (CONFIG_ARCH_HEAP_VBASE)
#define ESP32S3_HEAP_VEND   (CONFIG_ARCH_HEAP_VBASE + \
                             CONFIG_ARCH_HEAP_NPAGES * MM_PGSIZE)

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_pgvaddr
 *
 * Description:
 *   Get the kernel-addressable virtual address of a page-pool physical
 *   address.  The page pool (PSRAM) is permanently mapped into the kernel
 *   (WORLD0) address space, so a page allocated by mm_pgalloc() can be
 *   reached directly through this fixed offset.  Returns 0 if the physical
 *   address is not inside the page pool.
 *
 ****************************************************************************/

static inline uintptr_t esp32s3_pgvaddr(uintptr_t paddr)
{
  if (paddr >= CONFIG_ARCH_PGPOOL_PBASE && paddr < CONFIG_ARCH_PGPOOL_PEND)
    {
      return paddr - CONFIG_ARCH_PGPOOL_PBASE + CONFIG_ARCH_PGPOOL_VBASE;
    }

  return 0;
}

/****************************************************************************
 * Name: esp32s3_pgpaddr
 *
 * Description:
 *   Inverse of esp32s3_pgvaddr(): translate a kernel page-pool virtual
 *   address back to its physical address.  Returns 0 if the virtual address
 *   is not inside the mapped page pool.
 *
 ****************************************************************************/

static inline uintptr_t esp32s3_pgpaddr(uintptr_t vaddr)
{
  if (vaddr >= CONFIG_ARCH_PGPOOL_VBASE && vaddr < CONFIG_ARCH_PGPOOL_VEND)
    {
      return vaddr - CONFIG_ARCH_PGPOOL_VBASE + CONFIG_ARCH_PGPOOL_PBASE;
    }

  return 0;
}

/****************************************************************************
 * Name: esp32s3_uservaddr
 *
 * Description:
 *   Return true if vaddr lies inside one of the user (.text/.data/heap)
 *   cache-MMU windows.
 *
 ****************************************************************************/

static inline bool esp32s3_uservaddr(uintptr_t vaddr)
{
  return ((vaddr >= ESP32S3_TEXT_VBASE && vaddr < ESP32S3_TEXT_VEND) ||
          (vaddr >= ESP32S3_DATA_VBASE && vaddr < ESP32S3_DATA_VEND) ||
          (vaddr >= ESP32S3_HEAP_VBASE && vaddr < ESP32S3_HEAP_VEND));
}

/****************************************************************************
 * Name: esp32s3_pgwipe
 *
 * Description:
 *   Zero a page-pool physical page through its kernel virtual mapping.
 *
 ****************************************************************************/

static inline void esp32s3_pgwipe(uintptr_t paddr)
{
  uintptr_t vaddr = esp32s3_pgvaddr(paddr);
  if (vaddr)
    {
      memset((void *)vaddr, 0, MM_PGSIZE);
    }
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_addrenv_mapnew
 *
 * Description:
 *   Make a page that was added to an address environment after that
 *   environment was created -- heap growth through sbrk()/pgalloc() --
 *   visible to the running task.  The cache-MMU windows only ever reflect
 *   the resident environment, so this is a no-op unless 'addrenv' is the one
 *   currently selected.  For any other environment the page is picked up
 *   from the page array by the next up_addrenv_select().
 *
 * Input Parameters:
 *   addrenv - The address environment the page was added to.
 *   vaddr   - The user virtual address the page is mapped at.
 *   paddr   - The physical (page pool) address of the page.
 *
 ****************************************************************************/

void esp32s3_addrenv_mapnew(const arch_addrenv_t *addrenv, uintptr_t vaddr,
                            uintptr_t paddr);

#endif /* CONFIG_ARCH_ADDRENV */
#endif /* __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_ADDRENV_H */
