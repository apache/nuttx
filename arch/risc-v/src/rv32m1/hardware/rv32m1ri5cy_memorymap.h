/****************************************************************************
 * arch/risc-v/src/rv32m1/hardware/rv32m1ri5cy_memorymap.h
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

#ifndef __ARCH_RISCV_SRC_RV32M1_HARDWARE_RV32M1RI5CY_MEMORYMAP_H
#define __ARCH_RISCV_SRC_RV32M1_HARDWARE_RV32M1RI5CY_MEMORYMAP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Bus Base Address *********************************************************/

#define RV32M1_AIPS0_BASE      0x40000000
#define RV32M1_AIPS1_BASE      0x41000000
#define RV32M1_FLEXRAM_BASE    0x48000000
#define RV32M1_USB0RAM_BASE    0x48010000
#define RV32M1_GPIO_BASE       0x48020000
#define RV32M1_FLEXBUS_BASE    0xa0000000
#define RV32M1_PPB0_BASE       0xe0000000 /* ARM SYS Modules */
#define RV32M1_PPB1_BASE       0xf0000000 /* NXP SYS Modules */

/* AIPS0 Peripheral Offset **************************************************/

#define RV32M1_MSCM_OFFSET     0x00001000
#define RV32M1_DMA0CTRL_OFFSET 0x00008000
#define RV32M1_DMA0DESC_OFFSET 0x00009000
#define RV32M1_SMC0_OFFSET     0x00020000
#define RV32M1_FTFE_OFFSET     0x00023000
#define RV32M1_WDOG0_OFFSET    0x0002a000
#define RV32M1_PCC0_OFFSET     0x0002b000
#define RV32M1_SCG_OFFSET      0x0002c000
#define RV32M1_LPIT0_OFFSET    0x00030000
#define RV32M1_LPTMR0_OFFSET   0x00032000
#define RV32M1_LPTMR1_OFFSET   0x00033000
#define RV32M1_TSTMRA_OFFSET   0x00034000
#define RV32M1_LPUART0_OFFSET  0x00042000
#define RV32M1_LPUART1_OFFSET  0x00043000
#define RV32M1_LPUART2_OFFSET  0x00044000
#define RV32M1_PORTA_OFFSET    0x00046000
#define RV32M1_PORTB_OFFSET    0x00047000
#define RV32M1_PORTC_OFFSET    0x00048000
#define RV32M1_PORTD_OFFSET    0x00049000
#define RV32M1_INTMUX0_OFFSET  0x0004f000

/* AIPS1 Peripheral Offset **************************************************/

#define RV32M1_GPIOE_OFFSET    0x0000f000
#define RV32M1_PCC1_OFFSET     0x00027000
#define RV32M1_LPTMR2_OFFSET   0x0002b000
#define RV32M1_TSTMRB_OFFSET   0x0002c000
#define RV32M1_RSIM_OFFSET     0x0002f000
#define RV32M1_LPUART3_OFFSET  0x00036000
#define RV32M1_PORTE_OFFSET    0x00037000

/* GPIO Peripheral Offset ***************************************************/

#define RV32M1_GPIOA_OFFSET    0x00000000
#define RV32M1_GPIOB_OFFSET    0x00000040
#define RV32M1_GPIOC_OFFSET    0x00000080
#define RV32M1_GPIOD_OFFSET    0x000000c0

/* PPB0 Peripheral Offset ***************************************************/

#define RV32M1_EU0_OFFSET      0x00041000 /* Event Unit */

/* AIPS0 Peripheral Address *************************************************/

#define RV32M1_SMC0_BASE       (RV32M1_AIPS0_BASE + RV32M1_SMC0_OFFSET)
#define RV32M1_FTFE_BASE       (RV32M1_AIPS0_BASE + RV32M1_FTFE_OFFSET)
#define RV32M1_WDOG0_BASE      (RV32M1_AIPS0_BASE + RV32M1_WDOG0_OFFSET)
#define RV32M1_PCC0_BASE       (RV32M1_AIPS0_BASE + RV32M1_PCC0_OFFSET)
#define RV32M1_SCG_BASE        (RV32M1_AIPS0_BASE + RV32M1_SCG_OFFSET)
#define RV32M1_LPIT0_BASE      (RV32M1_AIPS0_BASE + RV32M1_LPIT0_OFFSET)
#define RV32M1_LPTMR0_BASE     (RV32M1_AIPS0_BASE + RV32M1_LPTMR0_OFFSET)
#define RV32M1_LPTMR1_BASE     (RV32M1_AIPS0_BASE + RV32M1_LPTMR0_OFFSET)
#define RV32M1_TSTMRA_BASE     (RV32M1_AIPS0_BASE + RV32M1_TSTMRA_OFFSET)
#define RV32M1_LPUART0_BASE    (RV32M1_AIPS0_BASE + RV32M1_LPUART0_OFFSET)
#define RV32M1_LPUART1_BASE    (RV32M1_AIPS0_BASE + RV32M1_LPUART1_OFFSET)
#define RV32M1_LPUART2_BASE    (RV32M1_AIPS0_BASE + RV32M1_LPUART2_OFFSET)
#define RV32M1_PORTA_BASE      (RV32M1_AIPS0_BASE + RV32M1_PORTA_OFFSET)
#define RV32M1_PORTB_BASE      (RV32M1_AIPS0_BASE + RV32M1_PORTB_OFFSET)
#define RV32M1_PORTC_BASE      (RV32M1_AIPS0_BASE + RV32M1_PORTC_OFFSET)
#define RV32M1_PORTD_BASE      (RV32M1_AIPS0_BASE + RV32M1_PORTD_OFFSET)
#define RV32M1_INTMUX0_BASE    (RV32M1_AIPS0_BASE + RV32M1_INTMUX0_OFFSET)

/* AIPS1 Peripheral Address *************************************************/

#define RV32M1_PORTE_BASE      (RV32M1_AIPS1_BASE + RV32M1_PORTE_OFFSET)
#define RV32M1_PCC1_BASE       (RV32M1_AIPS1_BASE + RV32M1_PCC1_OFFSET)
#define RV32M1_LPTMR2_BASE     (RV32M1_AIPS1_BASE + RV32M1_LPTMR2_OFFSET)
#define RV32M1_TSTMRB_BASE     (RV32M1_AIPS1_BASE + RV32M1_TSTMRB_OFFSET)
#define RV32M1_RSIM_BASE       (RV32M1_AIPS1_BASE + RV32M1_RSIM_OFFSET)
#define RV32M1_LPUART3_BASE    (RV32M1_AIPS1_BASE + RV32M1_LPUART3_OFFSET)
#define RV32M1_GPIOE_BASE      (RV32M1_AIPS1_BASE + RV32M1_GPIOE_OFFSET)

/* GPIO Peripheral Address **************************************************/

#define RV32M1_GPIOA_BASE      (RV32M1_GPIO_BASE + RV32M1_GPIOA_OFFSET)
#define RV32M1_GPIOB_BASE      (RV32M1_GPIO_BASE + RV32M1_GPIOB_OFFSET)
#define RV32M1_GPIOC_BASE      (RV32M1_GPIO_BASE + RV32M1_GPIOC_OFFSET)
#define RV32M1_GPIOD_BASE      (RV32M1_GPIO_BASE + RV32M1_GPIOD_OFFSET)

/* PPB0 Peripheral Address **************************************************/

#define RV32M1_EU_BASE         (RV32M1_PPB0_BASE + RV32M1_EU0_OFFSET)

#endif /* __ARCH_RISCV_SRC_RV32M1_HARDWARE_RV32M1RI5CY_MEMORYMAP_H */
