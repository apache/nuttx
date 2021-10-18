/****************************************************************************
 * arch/risc-v/src/k210/hardware/k210_memorymap.h
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

#ifndef __ARCH_RISCV_SRC_K210_HARDWARE_K210_MEMORYMAP_H
#define __ARCH_RISCV_SRC_K210_HARDWARE_K210_MEMORYMAP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Base Address ****************************************************/

#define K210_CLINT_BASE   0x02000000
#define K210_PLIC_BASE    0x0c000000

#ifdef CONFIG_K210_WITH_QEMU
#define K210_UART0_BASE   0x10010000
#else
#define K210_UART0_BASE   0x38000000
#endif
#define K210_GPIOHS_BASE  0x38001000
#define K210_FPIOA_BASE   0x502B0000

#define K210_SYSCTL_BASE  0x50440000

#endif /* __ARCH_RISCV_SRC_K210_HARDWARE_K210_MEMORYMAP_H */
