/****************************************************************************
 * arch/risc-v/src/c906/hardware/c906_memorymap.h
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

#ifndef __ARCH_RISCV_SRC_C906_HARDWARE_C906_MEMORYMAP_H
#define __ARCH_RISCV_SRC_C906_HARDWARE_C906_MEMORYMAP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Base Address ****************************************************/

#define C906_PLIC_BASE    (0x4000000000ULL)
#define C906_CLINT_BASE   (C906_PLIC_BASE + 0x4000000UL)
#define C906_CORET_BASE   (C906_PLIC_BASE + 0x4000000UL)

#ifdef CONFIG_C906_WITH_QEMU
#define C906_UART0_BASE   0x10015000UL
#else
#define C906_UART0_BASE   0x10015000UL
#endif

#endif /* __ARCH_RISCV_SRC_C906_HARDWARE_C906_MEMORYMAP_H */
