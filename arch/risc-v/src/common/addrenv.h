/****************************************************************************
 * arch/risc-v/src/common/addrenv.h
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

#ifndef __ARCH_RISC_V_SRC_COMMON_ADDRENV_H
#define __ARCH_RISC_V_SRC_COMMON_ADDRENV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>

#include "riscv_internal.h"

#ifdef CONFIG_ARCH_ADDRENV

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Aligned size of the kernel stack */

#ifdef CONFIG_ARCH_KERNEL_STACK
#  define ARCH_KERNEL_STACKSIZE STACK_ALIGN_UP(CONFIG_ARCH_KERNEL_STACKSIZE)
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: riscv_get_pgtable
 *
 * Description:
 *   Get the physical address of the final level page table corresponding to
 *   'vaddr'. If one does not exist, it will be allocated.
 *
 * Input Parameters:
 *   addrenv - Pointer to a structure describing the address environment
 *   vaddr - Virtual address to query for
 *
 * Returned Value:
 *   The physical address of the corresponding final level page table, or
 *   NULL if one does not exist, and there is no free memory to allocate one
 *
 ****************************************************************************/

uintptr_t riscv_get_pgtable(arch_addrenv_t *addrenv, uintptr_t vaddr);

/****************************************************************************
 * Name: riscv_map_pages
 *
 * Description:
 *   Map physical pages into a continuous virtual memory block.
 *
 * Input Parameters:
 *   addrenv - Pointer to a structure describing the address environment.
 *   pages - A pointer to the first element in a array of physical address,
 *     each corresponding to one page of memory.
 *   npages - The number of pages in the list of physical pages to be mapped.
 *   vaddr - The virtual address corresponding to the beginning of the
 *     (continuous) virtual address region.
 *   prot - MMU flags to use.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int riscv_map_pages(arch_addrenv_t *addrenv, uintptr_t *pages,
                    unsigned int npages, uintptr_t vaddr, int prot);

/****************************************************************************
 * Name: riscv_unmap_pages
 *
 * Description:
 *   Unmap a previously mapped virtual memory region.
 *
 * Input Parameters:
 *   addrenv - Pointer to a structure describing the address environment.
 *   vaddr - The virtual address corresponding to the beginning of the
 *     (continuous) virtual address region.
 *   npages - The number of pages to be unmapped
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int riscv_unmap_pages(arch_addrenv_t *addrenv, uintptr_t vaddr,
                      unsigned int npages);

#endif /* CONFIG_ARCH_ADDRENV */
#endif /* __ARCH_RISC_V_SRC_COMMON_ADDRENV_H */
