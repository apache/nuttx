/****************************************************************************
 * arch/arm/src/qemu/hardware/qemu_memorymap.h
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

#ifndef __ARCH_ARM_SRC_QEMU_HARDWARE_QEMU_MEMORYMAP_H
#define __ARCH_ARM_SRC_QEMU_HARDWARE_QEMU_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#undef ARMV7A_PGTABLE_MAPPING /* We do not remap the page table */

/* Check if the user has configured the page table address */

#if !defined(PGTABLE_BASE_PADDR) || !defined(PGTABLE_BASE_VADDR)

/* Sanity check.. if one is undefined, both should be undefined */

#  if defined(PGTABLE_BASE_PADDR) || defined(PGTABLE_BASE_VADDR)
#    error "Only one of PGTABLE_BASE_PADDR or PGTABLE_BASE_VADDR is defined"
#  endif

/* A sanity check, if the configuration says that the page table is read-only
 * and pre-initialized (maybe ROM), then it should have also defined both of
 * the page table base addresses.
 */

#  ifdef CONFIG_ARCH_ROMPGTABLE
#    error "CONFIG_ARCH_ROMPGTABLE defined; PGTABLE_BASE_P/VADDR not defined"
#  endif

#define ARMV7A_PGTABLE_MAPPING 1

#else /* !PGTABLE_BASE_PADDR || !PGTABLE_BASE_VADDR */

/* Sanity check.. if one is defined, both should be defined */

#  if !defined(PGTABLE_BASE_PADDR) || !defined(PGTABLE_BASE_VADDR)
#    error "One of PGTABLE_BASE_PADDR or PGTABLE_BASE_VADDR is undefined"
#  endif

#endif /* !PGTABLE_BASE_PADDR || !PGTABLE_BASE_VADDR */

#endif /* __ARCH_ARM_SRC_QEMU_HARDWARE_QEMU_MEMORYMAP_H */
