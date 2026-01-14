/****************************************************************************
 * arch/x86_64/src/common/pgalloc.h
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

#ifndef __ARCH_X86_64_SRC_COMMON_PGALLOC_H
#define __ARCH_X86_64_SRC_COMMON_PGALLOC_H

#ifdef CONFIG_MM_PGALLOC

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <string.h>

#include "addrenv.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: x86_64_pgvaddr
 *
 * Description:
 *   Get virtual address for pgpool physical address. Note: this function
 *   is minimalistic and is only usable for kernel mappings and only tests
 *   if the paddr is in the pgpool. For user mapped addresses this does not
 *   work.
 *
 * Input Parameters:
 *   paddr - Physical pgpool address
 *
 * Return:
 *   vaddr - Virtual address for physical address
 *
 ****************************************************************************/

static inline uintptr_t x86_64_pgvaddr(uintptr_t paddr)
{
#ifdef CONFIG_ARCH_PGPOOL_MAPPING
  if (paddr >= CONFIG_ARCH_PGPOOL_PBASE && paddr < CONFIG_ARCH_PGPOOL_PEND)
    {
      return CONFIG_ARCH_PGPOOL_VBASE + paddr - CONFIG_ARCH_PGPOOL_PBASE;
    }
  else
#endif
  if (paddr >= CONFIG_RAM_START && paddr < CONFIG_RAM_END && paddr != 0)
    {
      return X86_64_LOAD_OFFSET + paddr - CONFIG_RAM_START;
    }

  return 0;
}

static inline uintptr_t x86_64_pgpaddr(uintptr_t vaddr)
{
#ifdef CONFIG_ARCH_PGPOOL_MAPPING
  if (vaddr >= CONFIG_ARCH_PGPOOL_VBASE && vaddr < CONFIG_ARCH_PGPOOL_VEND)
    {
      return CONFIG_ARCH_PGPOOL_PBASE + vaddr - CONFIG_ARCH_PGPOOL_VBASE;
    }
  else
#endif
    if (vaddr >= X86_64_LOAD_OFFSET && vaddr < X86_64_LOAD_OFFSET)
    {
      return CONFIG_RAM_START + vaddr - X86_64_LOAD_OFFSET;
    }

  return 0;
}

/****************************************************************************
 * Name: x86_64_uservaddr
 *
 * Description:
 *   Return true if the virtual address, vaddr, lies in the user address
 *   space.
 *
 ****************************************************************************/

static inline bool x86_64_uservaddr(uintptr_t vaddr)
{
  /* Check if this address is within the range of the virtualized .bss/.data,
   * heap, or stack regions.
   */

  return ((vaddr >= ARCH_ADDRENV_VBASE && vaddr < ARCH_ADDRENV_VEND)
#ifdef CONFIG_ARCH_VMA_MAPPING
          || (vaddr >= CONFIG_ARCH_SHM_VBASE && vaddr < ARCH_SHM_VEND)
#endif
    );
}

/****************************************************************************
 * Name: x86_64_pgwipe
 *
 * Description:
 *   Wipe a page of physical memory, first mapping it into virtual memory.
 *
 * Input Parameters:
 *   paddr - Physical address of page
 *
 ****************************************************************************/

static inline void x86_64_pgwipe(uintptr_t paddr)
{
  uintptr_t vaddr = x86_64_pgvaddr(paddr);
  memset((void *)vaddr, 0, MM_PGSIZE);
}

#endif /* CONFIG_MM_PGALLOC */
#endif  /* __ARCH_X86_64_SRC_COMMON_PGALLOC_H */
