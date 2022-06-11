/****************************************************************************
 * arch/risc-v/src/litex/hardware/litex_memorymap.h
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

#ifndef __ARCH_RISCV_SRC_LITEX_HARDWARE_LITEX_MEMORYMAP_H
#define __ARCH_RISCV_SRC_LITEX_HARDWARE_LITEX_MEMORYMAP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Base Address ****************************************************/

/* litex vexRiscv does not follow RISC-V privileged specification and
 * uses two additional CSRs: mask and pending.
 */

#define LITEX_CPUTIMER_BASE     0xf0000800
#define LITEX_SDBLOCK2MEM_BASE  0xf0002000
#define LITEX_SDCORE_BASE       0xf0002800
#define LITEX_SDIRQ_BASE        0xf0003000
#define LITEX_SDMEM2BLOCK_BASE  0xf0003800
#define LITEX_SDPHY_BASE        0xf0004000
#define LITEX_TIMER0_BASE       0xf0005000
#define LITEX_UART0_BASE        0xf0005800

#endif /* __ARCH_RISCV_SRC_LITEX_HARDWARE_LITEX_MEMORYMAP_H */
