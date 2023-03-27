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
 * Included Files
 ****************************************************************************/

#ifdef CONFIG_LITEX_USE_CUSTOM_MEMORY_MAP
#include CONFIG_LITEX_CUSTOM_MEMORY_MAP_PATH
#else

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Base Address ****************************************************/

#ifdef CONFIG_LITEX_CORE_VEXRISCV_SMP
    #define LITEX_CLINT_BASE        0xf0010000
    #define LITEX_PLIC_BASE         0xf0c00000
    #define LITEX_ETHMAC_BASE       0xf0002000
    #define LITEX_ETHPHY_BASE       0xf0002800
    #define LITEX_TIMER0_BASE       0xf0001800
    #define LITEX_UART0_BASE        0xf0001000
    #define LITEX_SDBLOCK2MEM_BASE  0xf0004000
    #define LITEX_SDCORE_BASE       0xf0004800
    #define LITEX_SDIRQ_BASE        0xf0005000
    #define LITEX_SDMEM2BLOCK_BASE  0xf0005800
    #define LITEX_SDPHY_BASE        0xf0006000
#else
    #define LITEX_CPUTIMER_BASE     0xf0000800
    #define LITEX_ETHMAC_BASE       0xf0001000
    #define LITEX_ETHPHY_BASE       0xf0001800
    #define LITEX_TIMER0_BASE       0xf0006000
    #define LITEX_UART0_BASE        0xf0006800
    #define LITEX_SDBLOCK2MEM_BASE  0xf0003000
    #define LITEX_SDCORE_BASE       0xf0003800
    #define LITEX_SDIRQ_BASE        0xf0004000
    #define LITEX_SDMEM2BLOCK_BASE  0xf0004800
    #define LITEX_SDPHY_BASE        0xf0005000
#endif

/* GPIO peripheral definitions.
 *  - LITEX_GPIO_BASE is the first 32-bit address which contains a block
 *    of GPIO registers (peripheral). Each block can contain up to 32 pins.
 *  - LITEX_GPIO_OFFSET is the number of bytes between each GPIO peripheral.
 *  - LITEX_GPIO_MAX is the number of peripherals enabled in gateware.
 *
 *  Each peripheral is referenced by an index in the GPIO driver. E.g Index 0
 *  is the first GPIO peripheral at 0xf0008000. Index 1 is at 0xf00080020.
 */

#define LITEX_GPIO_BASE         0xf0008000
#define LITEX_GPIO_OFFSET       0x00000020
#define LITEX_GPIO_MAX          8

/* PWM peripheral definitions.
 *  - LITEX_PWM_BASE is the first 32-bit address which contains a block
 *    of PWM registers (peripheral). Each block controls a single output
 *    channel.
 *  - LITEX_PWM_OFFSET is the number of bytes between each PWM peripheral.
 *  - LITEX_PWM_MAX is the number of peripherals enabled in gateware.
 *
 *  Each peripheral is referenced by an index in the PWM driver. E.g Index 0
 *  is the first PWM peripheral at 0xF0009800. Index 1 is at 0xF0009008C.
 */

#define LITEX_PWM_BASE          0xf0009800
#define LITEX_PWM_OFFSET        0x0000000c
#define LITEX_PWM_MAX           4

#define LITEX_ETHMAC_RXBASE     0x80000000
#define LITEX_ETHMAC_TXBASE     0x80001000

#endif /* CONFIG_LITEX_USE_CUSTOM_MEMORY_MAP */
#endif /* __ARCH_RISCV_SRC_LITEX_HARDWARE_LITEX_MEMORYMAP_H */
