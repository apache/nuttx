/****************************************************************************
 * boards/risc-v/litex/arty_a7/include/vexii_memorymap.h
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
 * Auto-generated: 2025-12-28 11:16:55
 * Source: LiteX csr.h
 *
 ****************************************************************************/

#ifndef __BOARDS_RISCV_LITEX_ARTY_A7_INCLUDE_VEXII_MEMORYMAP_H
#define __BOARDS_RISCV_LITEX_ARTY_A7_INCLUDE_VEXII_MEMORYMAP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Custom memory map for LiteX VexiiRISCV core */

/* Core Peripherals */

#define LITEX_CLINT_BASE               0xf0010000UL
#define LITEX_PLIC_BASE                0xf0c00000UL

/* SOC Peripherals */

#define LITEX_UART0_BASE               0xf0001000UL
#define LITEX_TIMER0_BASE              0xf0001800UL

/* Ethernet (if enabled) */

#define LITEX_ETHMAC_BASE              0xf0002000UL
#define LITEX_ETHPHY_BASE              0xf0002800UL

/* Other Peripherals */

#define LITEX_CTRL_BASE                0xf0000000UL
#define LITEX_DDRPHY_BASE              0xf0000800UL
#define LITEX_IDENTIFIER_MEM_BASE      0xf0003000UL
#define LITEX_LEDS_BASE                0xf0003800UL
#define LITEX_SDRAM_BASE               0xf0004000UL

#endif /* __BOARDS_RISCV_LITEX_ARTY_A7_INCLUDE_VEXII_MEMORYMAP_H */
