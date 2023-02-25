/****************************************************************************
 * arch/risc-v/src/rv32m1/hardware/rv32m1_pcc.h
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

#ifndef __ARCH_RISCV_SRC_RV32M1_HARDWARE_RV32M1_PCC_H
#define __ARCH_RISCV_SRC_RV32M1_HARDWARE_RV32M1_PCC_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define RV32M1_PCC0_LPIT0_OFFSET       0x00c0
#define RV32M1_PCC0_LPUART0_OFFSET     0x0108
#define RV32M1_PCC0_LPUART1_OFFSET     0x010c
#define RV32M1_PCC0_LPUART2_OFFSET     0x0110
#define RV32M1_PCC0_PORTA_OFFSET       0x0118
#define RV32M1_PCC0_PORTB_OFFSET       0x011c
#define RV32M1_PCC0_PORTC_OFFSET       0x0120
#define RV32M1_PCC0_PORTD_OFFSET       0x0124
#define RV32M1_PCC0_INTMUX0_OFFSET     0x013c

#define RV32M2_PCC1_DMA1_OFFSET        0x0020
#define RV32M1_PCC1_GPIOE_OFFSET       0x003c
#define RV32M1_PCC1_XRDC_PAC_OFFSET    0x0058
#define RV32M1_PCC1_XRDC_MRC_OFFSET    0x005c
#define RV32M1_PCC1_SEMA42_1_OFFSET    0x006c
#define RV32M1_PCC1_DMAMUX1_OFFSET     0x0084
#define RV32M1_PCC1_INTMUX1_OFFSET     0x0088
#define RV32M1_PCC1_MUB_OFFSET         0x0090
#define RV32M1_PCC1_CAU3_OFFSET        0x00a0
#define RV32M1_PCC1_TRNG_OFFSET        0x00a4
#define RV32M1_PCC1_LPIT1_OFFSET       0x00a8
#define RV32M1_PCC1_TPM3_OFFSET        0x00b4
#define RV32M1_PCC1_LPI2C3_OFFSET      0x00b8
#define RV32M1_PCC1_LPSPI3_OFFSET      0x00d4
#define RV32M1_PCC1_LPUART3_OFFSET     0x00d8
#define RV32M1_PCC1_PORTE_OFFSET       0x00dc
#define RV32M1_PCC1_MTB_OFFSET         0x0200
#define RV32M1_PCC1_EXT_CLK_OFFSET     0x0204

/* PCC0 Register Addresses **************************************************/

#define RV32M1_PCC_LPIT0         (RV32M1_PCC0_BASE + RV32M1_PCC0_LPIT0_OFFSET)
#define RV32M1_PCC_LPUART0       (RV32M1_PCC0_BASE + RV32M1_PCC0_LPUART0_OFFSET)
#define RV32M1_PCC_LPUART1       (RV32M1_PCC0_BASE + RV32M1_PCC0_LPUART1_OFFSET)
#define RV32M1_PCC_LPUART2       (RV32M1_PCC0_BASE + RV32M1_PCC0_LPUART2_OFFSET)
#define RV32M1_PCC_PORTA         (RV32M1_PCC0_BASE + RV32M1_PCC0_PORTA_OFFSET)
#define RV32M1_PCC_PORTB         (RV32M1_PCC0_BASE + RV32M1_PCC0_PORTB_OFFSET)
#define RV32M1_PCC_PORTC         (RV32M1_PCC0_BASE + RV32M1_PCC0_PORTC_OFFSET)
#define RV32M1_PCC_PORTD         (RV32M1_PCC0_BASE + RV32M1_PCC0_PORTD_OFFSET)
#define RV32M1_PCC_INTMUX0       (RV32M1_PCC0_BASE + RV32M1_PCC0_INTMUX0_OFFSET)

/* PCC1 Register Addresses **************************************************/

#define RV32M1_PCC_DMA1          (RV32M1_PCC1_BASE + RV32M1_PCC1_DMA1_OFFSET)
#define RV32M1_PCC_GPIOE         (RV32M1_PCC1_BASE + RV32M1_PCC1_GPIOE_OFFSET)
#define RV32M1_PCC_XRDC_PAC      (RV32M1_PCC1_BASE + RV32M1_PCC1_XRDC_PAC_OFFSET)
#define RV32M1_PCC_XRDC_MRC      (RV32M1_PCC1_BASE + RV32M1_PCC1_XRDC_MRC_OFFSET)
#define RV32M1_PCC_SEMA42_1      (RV32M1_PCC1_BASE + RV32M1_PCC1_SEMA42_1_OFFSET)
#define RV32M1_PCC_DMAMUX1       (RV32M1_PCC1_BASE + RV32M1_PCC1_DMAMUX1_OFFSET)
#define RV32M1_PCC_INTMUX1       (RV32M1_PCC1_BASE + RV32M1_PCC1_INTMUX1_OFFSET)
#define RV32M1_PCC_MUB           (RV32M1_PCC1_BASE + RV32M1_PCC1_MUB_OFFSET)
#define RV32M1_PCC_CAU3          (RV32M1_PCC1_BASE + RV32M1_PCC1_CAU3_OFFSET)
#define RV32M1_PCC_TRNG          (RV32M1_PCC1_BASE + RV32M1_PCC1_TRNG_OFFSET)
#define RV32M1_PCC_LPIT1         (RV32M1_PCC1_BASE + RV32M1_PCC1_LPIT1_OFFSET)
#define RV32M1_PCC_TPM3          (RV32M1_PCC1_BASE + RV32M1_PCC1_TPM3_OFFSET)
#define RV32M1_PCC_LPI2C3        (RV32M1_PCC1_BASE + RV32M1_PCC1_LPI2C3_OFFSET)
#define RV32M1_PCC_LPSPI3        (RV32M1_PCC1_BASE + RV32M1_PCC1_LPSPI3_OFFSET)
#define RV32M1_PCC_LPUART3       (RV32M1_PCC1_BASE + RV32M1_PCC1_LPUART3_OFFSET)
#define RV32M1_PCC_PORTE         (RV32M1_PCC1_BASE + RV32M1_PCC1_PORTE_OFFSET)
#define RV32M1_PCC_MTB           (RV32M1_PCC1_BASE + RV32M1_PCC1_MTB_OFFSET)
#define RV32M1_PCC_EXT_CLK       (RV32M1_PCC1_BASE + RV32M1_PCC1_EXT_CLK_OFFSET)

/* Register Bitfield Definitions ********************************************/

#define PCC_CLKCFG_PCD_SHIFT         (0)
#define PCC_CLKCFG_PCD_MASK          (7 << PCC_CLKCFG_PCD_SHIFT)
#define PCC_CLKCFG_PCD_DIV1          (0 << PCC_CLKCFG_PCD_SHIFT)
#define PCC_CLKCFG_PCD_DIV2          (1 << PCC_CLKCFG_PCD_SHIFT)
#define PCC_CLKCFG_PCD_DIV3          (2 << PCC_CLKCFG_PCD_SHIFT)
#define PCC_CLKCFG_PCD_DIV4          (3 << PCC_CLKCFG_PCD_SHIFT)
#define PCC_CLKCFG_PCD_DIV5          (4 << PCC_CLKCFG_PCD_SHIFT)
#define PCC_CLKCFG_PCD_DIV6          (5 << PCC_CLKCFG_PCD_SHIFT)
#define PCC_CLKCFG_PCD_DIV7          (6 << PCC_CLKCFG_PCD_SHIFT)
#define PCC_CLKCFG_PCD_DIV8          (7 << PCC_CLKCFG_PCD_SHIFT)

#define PCC_CLKCFG_FRAC              (1 << 3)

#define PCC_CLKCFG_PCS_SHIFT         (24)
#define PCC_CLKCFG_PCS_MASK          (7 << PCC_CLKCFG_PCS_SHIFT)
#define PCC_CLKCFG_PCS_EXTCLK        (0 << PCC_CLKCFG_PCS_SHIFT)
#define PCC_CLKCFG_PCS_SOSC          (1 << PCC_CLKCFG_PCS_SHIFT)
#define PCC_CLKCFG_PCS_SIRC          (2 << PCC_CLKCFG_PCS_SHIFT)
#define PCC_CLKCFG_PCS_FIRC          (3 << PCC_CLKCFG_PCS_SHIFT)
#define PCC_CLKCFG_PCS_LPFLL         (6 << PCC_CLKCFG_PCS_SHIFT)

#define PCC_CLKCFG_INUSE             (1 << 29)

#define PCC_CLKCFG_CGC               (1 << 30) /* Bit30: Clock Gate Control */

#define PCC_CLKCFG_PR                (1 << 31) /* Bit31: Present */

#endif /* __ARCH_RISCV_SRC_RV32M1_HARDWARE_RV32M1_PCC_H */
