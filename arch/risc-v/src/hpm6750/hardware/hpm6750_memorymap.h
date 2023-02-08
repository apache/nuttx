/****************************************************************************
 * arch/risc-v/src/hpm6750/hardware/hpm6750_memorymap.h
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

#ifndef __ARCH_RISCV_SRC_HPM6750_HARDWARE_HPM6750_MEMORYMAP_H
#define __ARCH_RISCV_SRC_HPM6750_HARDWARE_HPM6750_MEMORYMAP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Base Address ****************************************************/

#define HPM6750_MCHTMR_BASE  0xE6000000
#define HPM6750_PLIC_BASE    0xE4000000
#define HPM6750_UART0_BASE   0xF0040000
#define HPM6750_SYSCTL_BASE  0xF4000000
#define HPM6750_IOC_BASE     0xF4040000
#define HPM6750_PIOC_BASE    0xF40D8000
#define HPM6750_PLLCTL_BASE  0xF4100000

#endif /* __ARCH_RISCV_SRC_HPM6750_HARDWARE_HPM6750_MEMORYMAP_H */
