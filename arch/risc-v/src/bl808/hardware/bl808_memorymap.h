/****************************************************************************
 * arch/risc-v/src/bl808/hardware/bl808_memorymap.h
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

#ifndef __ARCH_RISCV_SRC_BL808_HARDWARE_BL808_MEMORYMAP_H
#define __ARCH_RISCV_SRC_BL808_HARDWARE_BL808_MEMORYMAP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Base Address ****************************************************/

#define BL808_GLB_BASE     0x20000000ul
#define BL808_M0IC_BASE    0x20000050ul
#define BL808_GPIO_BASE    0x200008c4ul
#define BL808_GPADC_BASE   0x20002000ul
#define BL808_UART0_BASE   0x2000a000ul
#define BL808_UART1_BASE   0x2000a100ul
#define BL808_SPI0_BASE    0x2000a200ul
#define BL808_I2C0_BASE    0x2000a300ul
#define BL808_TIMER0_BASE  0x2000a500ul
#define BL808_I2C1_BASE    0x2000a900ul
#define BL808_UART2_BASE   0x2000aa00ul
#define BL808_AON_BASE     0x2000f000ul
#define BL808_UART3_BASE   0x30002000ul
#define BL808_I2C2_BASE    0x30003000ul
#define BL808_I2C3_BASE    0x30004000ul
#define BL808_MM_GLB_BASE  0x30007000ul
#define BL808_SPI1_BASE    0x30008000ul
#define BL808_TIMER1_BASE  0x30009000ul
#define BL808_PLIC_BASE    0xe0000000ul

#endif /* __ARCH_RISCV_SRC_BL808_HARDWARE_BL808_MEMORYMAP_H */
