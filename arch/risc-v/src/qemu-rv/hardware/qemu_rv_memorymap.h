/****************************************************************************
 * arch/risc-v/src/qemu-rv/hardware/qemu_rv_memorymap.h
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

#ifndef __ARCH_RISCV_SRC_QEMU_RV_HARDWARE_QEMU_RV_MEMORYMAP_H
#define __ARCH_RISCV_SRC_QEMU_RV_HARDWARE_QEMU_RV_MEMORYMAP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Base Address ****************************************************/

#ifdef CONFIG_ARCH_CHIP_QEMU_RV_PLIC
#  define QEMU_RV_PLIC_BASE    CONFIG_ARCH_CHIP_QEMU_RV_PLIC
#else
#  define QEMU_RV_PLIC_BASE    0xc000000
#endif

#ifdef CONFIG_ARCH_CHIP_QEMU_RV_CLINT
#  define QEMU_RV_CLINT_BASE   CONFIG_ARCH_CHIP_QEMU_RV_CLINT
#else
#  define QEMU_RV_CLINT_BASE   0x2000000
#endif

#ifdef CONFIG_ARCH_CHIP_QEMU_RV_ACLINT
#  define QEMU_RV_ACLINT_BASE  CONFIG_ARCH_CHIP_QEMU_RV_ACLINT
#else
#  define QEMU_RV_ACLINT_BASE  0x2f00000
#endif

#define   QEMU_RV_RESET_BASE   0x100000

#ifdef CONFIG_ARCH_USE_S_MODE
#  define QEMU_RV_APLIC_BASE   0x0d000000
#  define QEMU_RV_IMSIC_BASE   0x28000000
#else
#  define QEMU_RV_APLIC_BASE   0x0c000000
#  define QEMU_RV_IMSIC_BASE   0x24000000
#endif

#endif /* __ARCH_RISCV_SRC_QEMU_RV_HARDWARE_QEMU_RV_MEMORYMAP_H */
