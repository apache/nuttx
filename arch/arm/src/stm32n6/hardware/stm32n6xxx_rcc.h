/****************************************************************************
 * arch/arm/src/stm32n6/hardware/stm32n6xxx_rcc.h
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

#ifndef __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_RCC_H
#define __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_RCC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/stm32n6xxx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

/* Peripheral clock enable / set / clear register offsets.  Each enable
 * register (xxxENR) has a paired set register (xxxENSR) that performs an
 * atomic OR and a clear register (xxxENCR) that performs an atomic AND
 * (Reference: RM0486 14.5).  The same triplet pattern applies to the LPENR
 * (sleep mode) registers.
 */

#define STM32_RCC_CR_OFFSET           0x0000  /* Clock control register */
#define STM32_RCC_SR_OFFSET           0x0004  /* Clock status register */
#define STM32_RCC_STOPCR_OFFSET       0x0008  /* Stop mode control register */
#define STM32_RCC_CFGR1_OFFSET        0x0020  /* Clock configuration
                                               * register 1 */
#define STM32_RCC_CFGR2_OFFSET        0x0024  /* Clock configuration
                                               * register 2 */
#define STM32_RCC_BDCR_OFFSET         0x002c  /* Backup domain protection
                                               * register */
#define STM32_RCC_HWRSR_OFFSET        0x0030  /* Reset status register for
                                               * hardware */
#define STM32_RCC_RSR_OFFSET          0x0034  /* Reset register */
#define STM32_RCC_LSECFGR_OFFSET      0x0040  /* LSE configuration register */
#define STM32_RCC_MSICFGR_OFFSET      0x0044  /* MSI configuration register */
#define STM32_RCC_HSICFGR_OFFSET      0x0048  /* HSI configuration register */
#define STM32_RCC_HSIMCR_OFFSET       0x004c  /* HSI Monitor control
                                               * register */
#define STM32_RCC_HSIMSR_OFFSET       0x0050  /* HSI Monitor status
                                               * register */
#define STM32_RCC_HSECFGR_OFFSET      0x0054  /* HSE configuration register */
#define STM32_RCC_PLL1CFGR1_OFFSET    0x0080  /* PLL1 configuration
                                               * register 1 */
#define STM32_RCC_PLL1CFGR2_OFFSET    0x0084  /* PLL1 configuration
                                               * register 2 */
#define STM32_RCC_PLL1CFGR3_OFFSET    0x0088  /* PLL1 configuration
                                               * register 3 */
#define STM32_RCC_PLL2CFGR1_OFFSET    0x0090  /* PLL2 configuration
                                               * register 1 */
#define STM32_RCC_PLL2CFGR2_OFFSET    0x0094  /* PLL2 configuration
                                               * register 2 */
#define STM32_RCC_PLL2CFGR3_OFFSET    0x0098  /* PLL2 configuration
                                               * register 3 */
#define STM32_RCC_PLL3CFGR1_OFFSET    0x00a0  /* PLL3 configuration
                                               * register 1 */
#define STM32_RCC_PLL3CFGR2_OFFSET    0x00a4  /* PLL3 configuration
                                               * register 2 */
#define STM32_RCC_PLL3CFGR3_OFFSET    0x00a8  /* PLL3 configuration
                                               * register 3 */
#define STM32_RCC_PLL4CFGR1_OFFSET    0x00b0  /* PLL4 configuration
                                               * register 1 */
#define STM32_RCC_PLL4CFGR2_OFFSET    0x00b4  /* PLL4 configuration
                                               * register 2 */
#define STM32_RCC_PLL4CFGR3_OFFSET    0x00b8  /* PLL4 configuration
                                               * register 3 */
#define STM32_RCC_IC1CFGR_OFFSET      0x00c4  /* IC1 configuration register */
#define STM32_RCC_IC2CFGR_OFFSET      0x00c8  /* IC2 configuration register */
#define STM32_RCC_IC3CFGR_OFFSET      0x00cc  /* IC3 configuration register */
#define STM32_RCC_IC4CFGR_OFFSET      0x00d0  /* IC4 configuration register */
#define STM32_RCC_IC5CFGR_OFFSET      0x00d4  /* IC5 configuration register */
#define STM32_RCC_IC6CFGR_OFFSET      0x00d8  /* IC6 configuration register */
#define STM32_RCC_IC7CFGR_OFFSET      0x00dc  /* IC7 configuration register */
#define STM32_RCC_IC8CFGR_OFFSET      0x00e0  /* IC8 configuration register */
#define STM32_RCC_IC9CFGR_OFFSET      0x00e4  /* IC9 configuration register */
#define STM32_RCC_IC10CFGR_OFFSET     0x00e8  /* IC10 configuration
                                               * register */
#define STM32_RCC_IC11CFGR_OFFSET     0x00ec  /* IC11 configuration
                                               * register */
#define STM32_RCC_IC12CFGR_OFFSET     0x00f0  /* IC12 configuration
                                               * register */
#define STM32_RCC_IC13CFGR_OFFSET     0x00f4  /* IC13 configuration
                                               * register */
#define STM32_RCC_IC14CFGR_OFFSET     0x00f8  /* IC14 configuration
                                               * register */
#define STM32_RCC_IC15CFGR_OFFSET     0x00fc  /* IC15 configuration
                                               * register */
#define STM32_RCC_IC16CFGR_OFFSET     0x0100  /* IC16 configuration
                                               * register */
#define STM32_RCC_IC17CFGR_OFFSET     0x0104  /* IC17 configuration
                                               * register */
#define STM32_RCC_IC18CFGR_OFFSET     0x0108  /* IC18 configuration
                                               * register */
#define STM32_RCC_IC19CFGR_OFFSET     0x010c  /* IC19 configuration
                                               * register */
#define STM32_RCC_IC20CFGR_OFFSET     0x0110  /* IC20 configuration
                                               * register */
#define STM32_RCC_CIER_OFFSET         0x0124  /* Clock-source interrupt
                                               * enable register */
#define STM32_RCC_CIFR_OFFSET         0x0128  /* Clock-source interrupt
                                               * flag register */
#define STM32_RCC_CICR_OFFSET         0x012c  /* Clock-source interrupt
                                               * clear register */
#define STM32_RCC_CCIPR1_OFFSET       0x0144  /* Clock configuration for
                                               * independent peripheral
                                               * register 1 */
#define STM32_RCC_CCIPR2_OFFSET       0x0148  /* Clock configuration for
                                               * independent peripheral
                                               * register 2 */
#define STM32_RCC_CCIPR3_OFFSET       0x014c  /* Clock configuration for
                                               * independent peripheral
                                               * register 3 */
#define STM32_RCC_CCIPR4_OFFSET       0x0150  /* Clock configuration for
                                               * independent peripheral
                                               * register 4 */
#define STM32_RCC_CCIPR5_OFFSET       0x0154  /* Clock configuration for
                                               * independent peripheral
                                               * register 5 */
#define STM32_RCC_CCIPR6_OFFSET       0x0158  /* Clock configuration for
                                               * independent peripheral
                                               * register 6 */
#define STM32_RCC_CCIPR7_OFFSET       0x015c  /* Clock configuration for
                                               * independent peripheral
                                               * register 7 */
#define STM32_RCC_CCIPR8_OFFSET       0x0160  /* Clock configuration for
                                               * independent peripheral
                                               * register 8 */
#define STM32_RCC_CCIPR9_OFFSET       0x0164  /* Clock configuration for
                                               * independent peripheral
                                               * register 9 */
#define STM32_RCC_CCIPR12_OFFSET      0x0170  /* Clock configuration for
                                               * independent peripheral
                                               * register 12 */
#define STM32_RCC_CCIPR13_OFFSET      0x0174  /* Peripheral kernel clock
                                               * select register 13 */
#define STM32_RCC_CCIPR14_OFFSET      0x0178  /* Clock configuration for
                                               * independent peripheral
                                               * register 14 */
#define STM32_RCC_MISCRSTR_OFFSET     0x0208  /* Miscellaneous
                                               * configurations reset
                                               * register */
#define STM32_RCC_MEMRSTR_OFFSET      0x020c  /* Embedded memories reset
                                               * register */
#define STM32_RCC_AHB1RSTR_OFFSET     0x0210  /* AHB1 reset register */
#define STM32_RCC_AHB2RSTR_OFFSET     0x0214  /* AHB2 reset register */
#define STM32_RCC_AHB3RSTR_OFFSET     0x0218  /* AHB3 reset register */
#define STM32_RCC_AHB4RSTR_OFFSET     0x021c  /* AHB4 reset register */
#define STM32_RCC_AHB5RSTR_OFFSET     0x0220  /* AHB5 reset register */
#define STM32_RCC_APB1LRSTR_OFFSET    0x0224  /* APB1 peripheral reset
                                               * register 1 */
#define STM32_RCC_APB1HRSTR_OFFSET    0x0228  /* APB1 reset register 2 */
#define STM32_RCC_APB2RSTR_OFFSET     0x022c  /* APB2 peripheral reset
                                               * register */
#define STM32_RCC_APB4LRSTR_OFFSET    0x0234  /* APB4 peripheral reset
                                               * register 1 */
#define STM32_RCC_APB4HRSTR_OFFSET    0x0238  /* APB4 reset register 2 */
#define STM32_RCC_APB5RSTR_OFFSET     0x023c  /* APB5 reset register */
#define STM32_RCC_DIVENR_OFFSET       0x0240  /* IC divider enable register */
#define STM32_RCC_BUSENR_OFFSET       0x0244  /* Embedded buses enable
                                               * register */
#define STM32_RCC_MISCENR_OFFSET      0x0248  /* Miscellaneous
                                               * configurations enable
                                               * register */
#define STM32_RCC_MEMENR_OFFSET       0x024c  /* AXI/AHB SRAM clock enable
                                               * register */
#define STM32_RCC_AHB1ENR_OFFSET      0x0250  /* AHB1 enable register */
#define STM32_RCC_AHB2ENR_OFFSET      0x0254  /* AHB2 enable register */
#define STM32_RCC_AHB3ENR_OFFSET      0x0258  /* AHB3 enable register */
#define STM32_RCC_AHB4ENR_OFFSET      0x025c  /* AHB4 peripheral clock
                                               * enable register */
#define STM32_RCC_AHB5ENR_OFFSET      0x0260  /* AHB5 enable register */
#define STM32_RCC_APB1LENR_OFFSET     0x0264  /* APB1 peripheral clock
                                               * enable register 1 */
#define STM32_RCC_APB1HENR_OFFSET     0x0268  /* APB1 enable register 2 */
#define STM32_RCC_APB2ENR_OFFSET      0x026c  /* APB2 peripheral clock
                                               * enable register */
#define STM32_RCC_APB3ENR_OFFSET      0x0270  /* APB3 enable register */
#define STM32_RCC_APB4LENR_OFFSET     0x0274  /* APB4 peripheral clock
                                               * enable register 1 */
#define STM32_RCC_APB4HENR_OFFSET     0x0278  /* APB4 peripheral clock
                                               * enable register 2 */
#define STM32_RCC_APB5ENR_OFFSET      0x027c  /* APB5 enable register */
#define STM32_RCC_BUSLPENR_OFFSET     0x0284  /* Bus clocks enable in Sleep
                                               * mode */
#define STM32_RCC_MISCLPENR_OFFSET    0x0288  /* Miscellaneous
                                               * configurations sleep
                                               * enable register */
#define STM32_RCC_MEMLPENR_OFFSET     0x028c  /* SRAM clocks enable in
                                               * Sleep mode */
#define STM32_RCC_AHB1LPENR_OFFSET    0x0290  /* AHB1 sleep enable register */
#define STM32_RCC_AHB2LPENR_OFFSET    0x0294  /* AHB2 sleep enable register */
#define STM32_RCC_AHB3LPENR_OFFSET    0x0298  /* AHB3 sleep enable register */
#define STM32_RCC_AHB4LPENR_OFFSET    0x029c  /* AHB4 sleep enable register */
#define STM32_RCC_AHB5LPENR_OFFSET    0x02a0  /* AHB5 sleep enable register */
#define STM32_RCC_APB1LLPENR_OFFSET   0x02a4  /* APB1 LP clock enable
                                               * register 1 */
#define STM32_RCC_APB1HLPENR_OFFSET   0x02a8  /* APB1 sleep enable register
                                               * 2 */
#define STM32_RCC_APB2LPENR_OFFSET    0x02ac  /* APB2 LP clock enable
                                               * register */
#define STM32_RCC_APB3LPENR_OFFSET    0x02b0  /* APB3 sleep enable register */
#define STM32_RCC_APB4LLPENR_OFFSET   0x02b4  /* APB4 sleep enable register
                                               * 1 */
#define STM32_RCC_APB4HLPENR_OFFSET   0x02b8  /* APB4 sleep enable register
                                               * 2 */
#define STM32_RCC_APB5LPENR_OFFSET    0x02bc  /* APB5 sleep enable register */
#define STM32_RCC_CSR_OFFSET          0x0800  /* Clock status (set)
                                               * register */
#define STM32_RCC_STOPCSR_OFFSET      0x0808  /* STOPCSR configuration
                                               * register */
#define STM32_RCC_MISCRSTSR_OFFSET    0x0a08  /* Miscellaneous reset
                                               * register */
#define STM32_RCC_MEMRSTSR_OFFSET     0x0a0c  /* Memory reset register */
#define STM32_RCC_AHB1RSTSR_OFFSET    0x0a10  /* AHB1 reset register */
#define STM32_RCC_AHB2RSTSR_OFFSET    0x0a14  /* AHB2 reset register */
#define STM32_RCC_AHB3RSTSR_OFFSET    0x0a18  /* AHB3 reset register */
#define STM32_RCC_AHB4RSTSR_OFFSET    0x0a1c  /* AHB4 reset register */
#define STM32_RCC_AHB5RSTSR_OFFSET    0x0a20  /* AHB5 reset register */
#define STM32_RCC_APB1LRSTSR_OFFSET   0x0a24  /* APB1 reset register 1 */
#define STM32_RCC_APB1HRSTSR_OFFSET   0x0a28  /* APB1 reset register 2 */
#define STM32_RCC_APB2RSTSR_OFFSET    0x0a2c  /* APB2 reset register */
#define STM32_RCC_APB4LRSTSR_OFFSET   0x0a34  /* APB4 reset register 1 */
#define STM32_RCC_APB4HRSTSR_OFFSET   0x0a38  /* APB4 reset register 2 */
#define STM32_RCC_APB5RSTSR_OFFSET    0x0a3c  /* APB5 reset register */
#define STM32_RCC_DIVENSR_OFFSET      0x0a40  /* IC divider enable set
                                               * register */
#define STM32_RCC_BUSENSR_OFFSET      0x0a44  /* Bus enable register */
#define STM32_RCC_MISCENSR_OFFSET     0x0a48  /* Miscellaneous enable
                                               * register */
#define STM32_RCC_MEMENSR_OFFSET      0x0a4c  /* SRAM clock enable set
                                               * register */
#define STM32_RCC_AHB1ENSR_OFFSET     0x0a50  /* AHB1 enable register */
#define STM32_RCC_AHB2ENSR_OFFSET     0x0a54  /* AHB2 enable register */
#define STM32_RCC_AHB3ENSR_OFFSET     0x0a58  /* AHB3 enable register */
#define STM32_RCC_AHB4ENSR_OFFSET     0x0a5c  /* AHB4 clock enable set
                                               * register */
#define STM32_RCC_AHB5ENSR_OFFSET     0x0a60  /* AHB5 enable register */
#define STM32_RCC_APB1LENSR_OFFSET    0x0a64  /* APB1 clock enable set
                                               * register 1 */
#define STM32_RCC_APB1HENSR_OFFSET    0x0a68  /* APB1 enable register 2 */
#define STM32_RCC_APB2ENSR_OFFSET     0x0a6c  /* APB2 clock enable set
                                               * register */
#define STM32_RCC_APB3ENSR_OFFSET     0x0a70  /* APB3 enable register */
#define STM32_RCC_APB4LENSR_OFFSET    0x0a74  /* APB4 enable register 1 */
#define STM32_RCC_APB4HENSR_OFFSET    0x0a78  /* APB4 clock enable set
                                               * register 2 */
#define STM32_RCC_APB5ENSR_OFFSET     0x0a7c  /* APB5 enable register */
#define STM32_RCC_BUSLPENSR_OFFSET    0x0a84  /* Bus LP clock enable set
                                               * register */
#define STM32_RCC_MISCLPENSR_OFFSET   0x0a88  /* Miscellaneous sleep enable
                                               * register */
#define STM32_RCC_MEMLPENSR_OFFSET    0x0a8c  /* SRAM LP clock enable set
                                               * register */
#define STM32_RCC_AHB1LPENSR_OFFSET   0x0a90  /* AHB1 sleep enable register */
#define STM32_RCC_AHB2LPENSR_OFFSET   0x0a94  /* AHB2 sleep enable register */
#define STM32_RCC_AHB3LPENSR_OFFSET   0x0a98  /* AHB3 sleep enable register */
#define STM32_RCC_AHB4LPENSR_OFFSET   0x0a9c  /* AHB4 sleep enable register */
#define STM32_RCC_AHB5LPENSR_OFFSET   0x0aa0  /* AHB5 sleep enable register */
#define STM32_RCC_APB1LLPENSR_OFFSET  0x0aa4  /* APB1 LP clock enable set
                                               * register 1 */
#define STM32_RCC_APB1HLPENSR_OFFSET  0x0aa8  /* APB1 sleep enable register
                                               * 2 */
#define STM32_RCC_APB2LPENSR_OFFSET   0x0aac  /* APB2 LP clock enable set
                                               * register */
#define STM32_RCC_APB3LPENSR_OFFSET   0x0ab0  /* APB3 sleep enable register */
#define STM32_RCC_APB4LLPENSR_OFFSET  0x0ab4  /* APB4 sleep enable register
                                               * 1 */
#define STM32_RCC_APB4HLPENSR_OFFSET  0x0ab8  /* APB4 sleep enable register
                                               * 2 */
#define STM32_RCC_APB5LPENSR_OFFSET   0x0abc  /* APB5 sleep enable register */
#define STM32_RCC_CCR_OFFSET          0x1000  /* Clock control clear
                                               * register */
#define STM32_RCC_STOPCCR_OFFSET      0x1008  /* Stop mode configuration
                                               * clear register */
#define STM32_RCC_MISCRSTCR_OFFSET    0x1208  /* Miscellaneous reset clear
                                               * register */
#define STM32_RCC_MEMRSTCR_OFFSET     0x120c  /* Memory reset clear
                                               * register */
#define STM32_RCC_AHB1RSTCR_OFFSET    0x1210  /* AHB1 reset clear register */
#define STM32_RCC_AHB2RSTCR_OFFSET    0x1214  /* AHB2 reset clear register */
#define STM32_RCC_AHB3RSTCR_OFFSET    0x1218  /* AHB3 reset r clear
                                               * register */
#define STM32_RCC_AHB4RSTCR_OFFSET    0x121c  /* AHB4 reset clear register */
#define STM32_RCC_AHB5RSTCR_OFFSET    0x1220  /* AHB5 reset clear register */
#define STM32_RCC_APB1LRSTCR_OFFSET   0x1224  /* APB1 reset clear register
                                               * 1 */
#define STM32_RCC_APB1HRSTCR_OFFSET   0x1228  /* APB1 reset clear register
                                               * 2 */
#define STM32_RCC_APB2RSTCR_OFFSET    0x122c  /* APB2 reset clear register */
#define STM32_RCC_APB4LRSTCR_OFFSET   0x1234  /* APB4 reset clear register
                                               * 1 */
#define STM32_RCC_APB4HRSTCR_OFFSET   0x1238  /* APB4 reset clear register
                                               * 2 */
#define STM32_RCC_APB5RSTCR_OFFSET    0x123c  /* APB5 reset clear register */
#define STM32_RCC_DIVENCR_OFFSET      0x1240  /* Divider enable clear
                                               * register */
#define STM32_RCC_BUSENCR_OFFSET      0x1244  /* Bus enable clear register */
#define STM32_RCC_MISCENCR_OFFSET     0x1248  /* Miscellaneous enable clear
                                               * register */
#define STM32_RCC_MEMENCR_OFFSET      0x124c  /* Memory enable clear
                                               * register */
#define STM32_RCC_AHB1ENCR_OFFSET     0x1250  /* AHB1 enable clear register */
#define STM32_RCC_AHB2ENCR_OFFSET     0x1254  /* AHB2 enable clear register */
#define STM32_RCC_AHB3ENCR_OFFSET     0x1258  /* AHB3 enable clear register */
#define STM32_RCC_AHB4ENCR_OFFSET     0x125c  /* AHB4 enable clear register */
#define STM32_RCC_AHB5ENCR_OFFSET     0x1260  /* AHB5 enable clear register */
#define STM32_RCC_APB1LENCR_OFFSET    0x1264  /* APB1 enable clear register
                                               * 1 */
#define STM32_RCC_APB1HENCR_OFFSET    0x1268  /* APB1 enable clear register
                                               * 2 */
#define STM32_RCC_APB2ENCR_OFFSET     0x126c  /* APB2 clock enable clear
                                               * register */
#define STM32_RCC_APB3ENCR_OFFSET     0x1270  /* APB3 enable clear register */
#define STM32_RCC_APB4LENCR_OFFSET    0x1274  /* APB4 enable clear register
                                               * 1 */
#define STM32_RCC_APB4HENCR_OFFSET    0x1278  /* APB4 enable clear register
                                               * 2 */
#define STM32_RCC_APB5ENCR_OFFSET     0x127c  /* APB5 enable clear register */
#define STM32_RCC_BUSLPENCR_OFFSET    0x1284  /* Bus sleep enable clear
                                               * register */
#define STM32_RCC_MISCLPENCR_OFFSET   0x1288  /* Miscellaneous sleep enable
                                               * clear register */
#define STM32_RCC_MEMLPENCR_OFFSET    0x128c  /* Memory sleep enable clear
                                               * register */
#define STM32_RCC_AHB1LPENCR_OFFSET   0x1290  /* AHB1 sleep enable clear
                                               * register */
#define STM32_RCC_AHB2LPENCR_OFFSET   0x1294  /* AHB2 sleep enable clear
                                               * register */
#define STM32_RCC_AHB3LPENCR_OFFSET   0x1298  /* AHB3 sleep enable clear
                                               * register */
#define STM32_RCC_AHB4LPENCR_OFFSET   0x129c  /* AHB4 sleep enable clear
                                               * register */
#define STM32_RCC_AHB5LPENCR_OFFSET   0x12a0  /* AHB5 sleep enable clear
                                               * register */
#define STM32_RCC_APB1LLPENCR_OFFSET  0x12a4  /* APB1 sleep enable clear
                                               * register 1 */
#define STM32_RCC_APB1HLPENCR_OFFSET  0x12a8  /* APB1 sleep enable clear
                                               * register 2 */
#define STM32_RCC_APB2LPENCR_OFFSET   0x12ac  /* APB2 sleep enable clear
                                               * register */
#define STM32_RCC_APB3LPENCR_OFFSET   0x12b0  /* APB3 sleep enable clear
                                               * register */
#define STM32_RCC_APB4LLPENCR_OFFSET  0x12b4  /* APB4 sleep enable clear
                                               * register 1 */
#define STM32_RCC_APB4HLPENCR_OFFSET  0x12b8  /* APB4 sleep enable clear
                                               * register 2 */
#define STM32_RCC_APB5LPENCR_OFFSET   0x12bc  /* APB5 sleep enable clear
                                               * register */

/* Register Addresses *******************************************************/

#define STM32_RCC_CR                  (STM32_RCC_BASE + STM32_RCC_CR_OFFSET)
#define STM32_RCC_SR                  (STM32_RCC_BASE + STM32_RCC_SR_OFFSET)
#define STM32_RCC_STOPCR              (STM32_RCC_BASE + STM32_RCC_STOPCR_OFFSET)
#define STM32_RCC_CFGR1               (STM32_RCC_BASE + STM32_RCC_CFGR1_OFFSET)
#define STM32_RCC_CFGR2               (STM32_RCC_BASE + STM32_RCC_CFGR2_OFFSET)
#define STM32_RCC_BDCR                (STM32_RCC_BASE + STM32_RCC_BDCR_OFFSET)
#define STM32_RCC_HWRSR               (STM32_RCC_BASE + STM32_RCC_HWRSR_OFFSET)
#define STM32_RCC_RSR                 (STM32_RCC_BASE + STM32_RCC_RSR_OFFSET)
#define STM32_RCC_LSECFGR             (STM32_RCC_BASE + STM32_RCC_LSECFGR_OFFSET)
#define STM32_RCC_MSICFGR             (STM32_RCC_BASE + STM32_RCC_MSICFGR_OFFSET)
#define STM32_RCC_HSICFGR             (STM32_RCC_BASE + STM32_RCC_HSICFGR_OFFSET)
#define STM32_RCC_HSIMCR              (STM32_RCC_BASE + STM32_RCC_HSIMCR_OFFSET)
#define STM32_RCC_HSIMSR              (STM32_RCC_BASE + STM32_RCC_HSIMSR_OFFSET)
#define STM32_RCC_HSECFGR             (STM32_RCC_BASE + STM32_RCC_HSECFGR_OFFSET)
#define STM32_RCC_PLL1CFGR1           (STM32_RCC_BASE + STM32_RCC_PLL1CFGR1_OFFSET)
#define STM32_RCC_PLL1CFGR2           (STM32_RCC_BASE + STM32_RCC_PLL1CFGR2_OFFSET)
#define STM32_RCC_PLL1CFGR3           (STM32_RCC_BASE + STM32_RCC_PLL1CFGR3_OFFSET)
#define STM32_RCC_PLL2CFGR1           (STM32_RCC_BASE + STM32_RCC_PLL2CFGR1_OFFSET)
#define STM32_RCC_PLL2CFGR2           (STM32_RCC_BASE + STM32_RCC_PLL2CFGR2_OFFSET)
#define STM32_RCC_PLL2CFGR3           (STM32_RCC_BASE + STM32_RCC_PLL2CFGR3_OFFSET)
#define STM32_RCC_PLL3CFGR1           (STM32_RCC_BASE + STM32_RCC_PLL3CFGR1_OFFSET)
#define STM32_RCC_PLL3CFGR2           (STM32_RCC_BASE + STM32_RCC_PLL3CFGR2_OFFSET)
#define STM32_RCC_PLL3CFGR3           (STM32_RCC_BASE + STM32_RCC_PLL3CFGR3_OFFSET)
#define STM32_RCC_PLL4CFGR1           (STM32_RCC_BASE + STM32_RCC_PLL4CFGR1_OFFSET)
#define STM32_RCC_PLL4CFGR2           (STM32_RCC_BASE + STM32_RCC_PLL4CFGR2_OFFSET)
#define STM32_RCC_PLL4CFGR3           (STM32_RCC_BASE + STM32_RCC_PLL4CFGR3_OFFSET)
#define STM32_RCC_IC1CFGR             (STM32_RCC_BASE + STM32_RCC_IC1CFGR_OFFSET)
#define STM32_RCC_IC2CFGR             (STM32_RCC_BASE + STM32_RCC_IC2CFGR_OFFSET)
#define STM32_RCC_IC3CFGR             (STM32_RCC_BASE + STM32_RCC_IC3CFGR_OFFSET)
#define STM32_RCC_IC4CFGR             (STM32_RCC_BASE + STM32_RCC_IC4CFGR_OFFSET)
#define STM32_RCC_IC5CFGR             (STM32_RCC_BASE + STM32_RCC_IC5CFGR_OFFSET)
#define STM32_RCC_IC6CFGR             (STM32_RCC_BASE + STM32_RCC_IC6CFGR_OFFSET)
#define STM32_RCC_IC7CFGR             (STM32_RCC_BASE + STM32_RCC_IC7CFGR_OFFSET)
#define STM32_RCC_IC8CFGR             (STM32_RCC_BASE + STM32_RCC_IC8CFGR_OFFSET)
#define STM32_RCC_IC9CFGR             (STM32_RCC_BASE + STM32_RCC_IC9CFGR_OFFSET)
#define STM32_RCC_IC10CFGR            (STM32_RCC_BASE + STM32_RCC_IC10CFGR_OFFSET)
#define STM32_RCC_IC11CFGR            (STM32_RCC_BASE + STM32_RCC_IC11CFGR_OFFSET)
#define STM32_RCC_IC12CFGR            (STM32_RCC_BASE + STM32_RCC_IC12CFGR_OFFSET)
#define STM32_RCC_IC13CFGR            (STM32_RCC_BASE + STM32_RCC_IC13CFGR_OFFSET)
#define STM32_RCC_IC14CFGR            (STM32_RCC_BASE + STM32_RCC_IC14CFGR_OFFSET)
#define STM32_RCC_IC15CFGR            (STM32_RCC_BASE + STM32_RCC_IC15CFGR_OFFSET)
#define STM32_RCC_IC16CFGR            (STM32_RCC_BASE + STM32_RCC_IC16CFGR_OFFSET)
#define STM32_RCC_IC17CFGR            (STM32_RCC_BASE + STM32_RCC_IC17CFGR_OFFSET)
#define STM32_RCC_IC18CFGR            (STM32_RCC_BASE + STM32_RCC_IC18CFGR_OFFSET)
#define STM32_RCC_IC19CFGR            (STM32_RCC_BASE + STM32_RCC_IC19CFGR_OFFSET)
#define STM32_RCC_IC20CFGR            (STM32_RCC_BASE + STM32_RCC_IC20CFGR_OFFSET)
#define STM32_RCC_CIER                (STM32_RCC_BASE + STM32_RCC_CIER_OFFSET)
#define STM32_RCC_CIFR                (STM32_RCC_BASE + STM32_RCC_CIFR_OFFSET)
#define STM32_RCC_CICR                (STM32_RCC_BASE + STM32_RCC_CICR_OFFSET)
#define STM32_RCC_CCIPR1              (STM32_RCC_BASE + STM32_RCC_CCIPR1_OFFSET)
#define STM32_RCC_CCIPR2              (STM32_RCC_BASE + STM32_RCC_CCIPR2_OFFSET)
#define STM32_RCC_CCIPR3              (STM32_RCC_BASE + STM32_RCC_CCIPR3_OFFSET)
#define STM32_RCC_CCIPR4              (STM32_RCC_BASE + STM32_RCC_CCIPR4_OFFSET)
#define STM32_RCC_CCIPR5              (STM32_RCC_BASE + STM32_RCC_CCIPR5_OFFSET)
#define STM32_RCC_CCIPR6              (STM32_RCC_BASE + STM32_RCC_CCIPR6_OFFSET)
#define STM32_RCC_CCIPR7              (STM32_RCC_BASE + STM32_RCC_CCIPR7_OFFSET)
#define STM32_RCC_CCIPR8              (STM32_RCC_BASE + STM32_RCC_CCIPR8_OFFSET)
#define STM32_RCC_CCIPR9              (STM32_RCC_BASE + STM32_RCC_CCIPR9_OFFSET)
#define STM32_RCC_CCIPR12             (STM32_RCC_BASE + STM32_RCC_CCIPR12_OFFSET)
#define STM32_RCC_CCIPR13             (STM32_RCC_BASE + STM32_RCC_CCIPR13_OFFSET)
#define STM32_RCC_CCIPR14             (STM32_RCC_BASE + STM32_RCC_CCIPR14_OFFSET)
#define STM32_RCC_MISCRSTR            (STM32_RCC_BASE + STM32_RCC_MISCRSTR_OFFSET)
#define STM32_RCC_MEMRSTR             (STM32_RCC_BASE + STM32_RCC_MEMRSTR_OFFSET)
#define STM32_RCC_AHB1RSTR            (STM32_RCC_BASE + STM32_RCC_AHB1RSTR_OFFSET)
#define STM32_RCC_AHB2RSTR            (STM32_RCC_BASE + STM32_RCC_AHB2RSTR_OFFSET)
#define STM32_RCC_AHB3RSTR            (STM32_RCC_BASE + STM32_RCC_AHB3RSTR_OFFSET)
#define STM32_RCC_AHB4RSTR            (STM32_RCC_BASE + STM32_RCC_AHB4RSTR_OFFSET)
#define STM32_RCC_AHB5RSTR            (STM32_RCC_BASE + STM32_RCC_AHB5RSTR_OFFSET)
#define STM32_RCC_APB1LRSTR           (STM32_RCC_BASE + STM32_RCC_APB1LRSTR_OFFSET)
#define STM32_RCC_APB1HRSTR           (STM32_RCC_BASE + STM32_RCC_APB1HRSTR_OFFSET)
#define STM32_RCC_APB2RSTR            (STM32_RCC_BASE + STM32_RCC_APB2RSTR_OFFSET)
#define STM32_RCC_APB4LRSTR           (STM32_RCC_BASE + STM32_RCC_APB4LRSTR_OFFSET)
#define STM32_RCC_APB4HRSTR           (STM32_RCC_BASE + STM32_RCC_APB4HRSTR_OFFSET)
#define STM32_RCC_APB5RSTR            (STM32_RCC_BASE + STM32_RCC_APB5RSTR_OFFSET)
#define STM32_RCC_DIVENR              (STM32_RCC_BASE + STM32_RCC_DIVENR_OFFSET)
#define STM32_RCC_BUSENR              (STM32_RCC_BASE + STM32_RCC_BUSENR_OFFSET)
#define STM32_RCC_MISCENR             (STM32_RCC_BASE + STM32_RCC_MISCENR_OFFSET)
#define STM32_RCC_MEMENR              (STM32_RCC_BASE + STM32_RCC_MEMENR_OFFSET)
#define STM32_RCC_AHB1ENR             (STM32_RCC_BASE + STM32_RCC_AHB1ENR_OFFSET)
#define STM32_RCC_AHB2ENR             (STM32_RCC_BASE + STM32_RCC_AHB2ENR_OFFSET)
#define STM32_RCC_AHB3ENR             (STM32_RCC_BASE + STM32_RCC_AHB3ENR_OFFSET)
#define STM32_RCC_AHB4ENR             (STM32_RCC_BASE + STM32_RCC_AHB4ENR_OFFSET)
#define STM32_RCC_AHB5ENR             (STM32_RCC_BASE + STM32_RCC_AHB5ENR_OFFSET)
#define STM32_RCC_APB1LENR            (STM32_RCC_BASE + STM32_RCC_APB1LENR_OFFSET)
#define STM32_RCC_APB1HENR            (STM32_RCC_BASE + STM32_RCC_APB1HENR_OFFSET)
#define STM32_RCC_APB2ENR             (STM32_RCC_BASE + STM32_RCC_APB2ENR_OFFSET)
#define STM32_RCC_APB3ENR             (STM32_RCC_BASE + STM32_RCC_APB3ENR_OFFSET)
#define STM32_RCC_APB4LENR            (STM32_RCC_BASE + STM32_RCC_APB4LENR_OFFSET)
#define STM32_RCC_APB4HENR            (STM32_RCC_BASE + STM32_RCC_APB4HENR_OFFSET)
#define STM32_RCC_APB5ENR             (STM32_RCC_BASE + STM32_RCC_APB5ENR_OFFSET)
#define STM32_RCC_BUSLPENR            (STM32_RCC_BASE + STM32_RCC_BUSLPENR_OFFSET)
#define STM32_RCC_MISCLPENR           (STM32_RCC_BASE + STM32_RCC_MISCLPENR_OFFSET)
#define STM32_RCC_MEMLPENR            (STM32_RCC_BASE + STM32_RCC_MEMLPENR_OFFSET)
#define STM32_RCC_AHB1LPENR           (STM32_RCC_BASE + STM32_RCC_AHB1LPENR_OFFSET)
#define STM32_RCC_AHB2LPENR           (STM32_RCC_BASE + STM32_RCC_AHB2LPENR_OFFSET)
#define STM32_RCC_AHB3LPENR           (STM32_RCC_BASE + STM32_RCC_AHB3LPENR_OFFSET)
#define STM32_RCC_AHB4LPENR           (STM32_RCC_BASE + STM32_RCC_AHB4LPENR_OFFSET)
#define STM32_RCC_AHB5LPENR           (STM32_RCC_BASE + STM32_RCC_AHB5LPENR_OFFSET)
#define STM32_RCC_APB1LLPENR          (STM32_RCC_BASE + STM32_RCC_APB1LLPENR_OFFSET)
#define STM32_RCC_APB1HLPENR          (STM32_RCC_BASE + STM32_RCC_APB1HLPENR_OFFSET)
#define STM32_RCC_APB2LPENR           (STM32_RCC_BASE + STM32_RCC_APB2LPENR_OFFSET)
#define STM32_RCC_APB3LPENR           (STM32_RCC_BASE + STM32_RCC_APB3LPENR_OFFSET)
#define STM32_RCC_APB4LLPENR          (STM32_RCC_BASE + STM32_RCC_APB4LLPENR_OFFSET)
#define STM32_RCC_APB4HLPENR          (STM32_RCC_BASE + STM32_RCC_APB4HLPENR_OFFSET)
#define STM32_RCC_APB5LPENR           (STM32_RCC_BASE + STM32_RCC_APB5LPENR_OFFSET)
#define STM32_RCC_CSR                 (STM32_RCC_BASE + STM32_RCC_CSR_OFFSET)
#define STM32_RCC_STOPCSR             (STM32_RCC_BASE + STM32_RCC_STOPCSR_OFFSET)
#define STM32_RCC_MISCRSTSR           (STM32_RCC_BASE + STM32_RCC_MISCRSTSR_OFFSET)
#define STM32_RCC_MEMRSTSR            (STM32_RCC_BASE + STM32_RCC_MEMRSTSR_OFFSET)
#define STM32_RCC_AHB1RSTSR           (STM32_RCC_BASE + STM32_RCC_AHB1RSTSR_OFFSET)
#define STM32_RCC_AHB2RSTSR           (STM32_RCC_BASE + STM32_RCC_AHB2RSTSR_OFFSET)
#define STM32_RCC_AHB3RSTSR           (STM32_RCC_BASE + STM32_RCC_AHB3RSTSR_OFFSET)
#define STM32_RCC_AHB4RSTSR           (STM32_RCC_BASE + STM32_RCC_AHB4RSTSR_OFFSET)
#define STM32_RCC_AHB5RSTSR           (STM32_RCC_BASE + STM32_RCC_AHB5RSTSR_OFFSET)
#define STM32_RCC_APB1LRSTSR          (STM32_RCC_BASE + STM32_RCC_APB1LRSTSR_OFFSET)
#define STM32_RCC_APB1HRSTSR          (STM32_RCC_BASE + STM32_RCC_APB1HRSTSR_OFFSET)
#define STM32_RCC_APB2RSTSR           (STM32_RCC_BASE + STM32_RCC_APB2RSTSR_OFFSET)
#define STM32_RCC_APB4LRSTSR          (STM32_RCC_BASE + STM32_RCC_APB4LRSTSR_OFFSET)
#define STM32_RCC_APB4HRSTSR          (STM32_RCC_BASE + STM32_RCC_APB4HRSTSR_OFFSET)
#define STM32_RCC_APB5RSTSR           (STM32_RCC_BASE + STM32_RCC_APB5RSTSR_OFFSET)
#define STM32_RCC_DIVENSR             (STM32_RCC_BASE + STM32_RCC_DIVENSR_OFFSET)
#define STM32_RCC_BUSENSR             (STM32_RCC_BASE + STM32_RCC_BUSENSR_OFFSET)
#define STM32_RCC_MISCENSR            (STM32_RCC_BASE + STM32_RCC_MISCENSR_OFFSET)
#define STM32_RCC_MEMENSR             (STM32_RCC_BASE + STM32_RCC_MEMENSR_OFFSET)
#define STM32_RCC_AHB1ENSR            (STM32_RCC_BASE + STM32_RCC_AHB1ENSR_OFFSET)
#define STM32_RCC_AHB2ENSR            (STM32_RCC_BASE + STM32_RCC_AHB2ENSR_OFFSET)
#define STM32_RCC_AHB3ENSR            (STM32_RCC_BASE + STM32_RCC_AHB3ENSR_OFFSET)
#define STM32_RCC_AHB4ENSR            (STM32_RCC_BASE + STM32_RCC_AHB4ENSR_OFFSET)
#define STM32_RCC_AHB5ENSR            (STM32_RCC_BASE + STM32_RCC_AHB5ENSR_OFFSET)
#define STM32_RCC_APB1LENSR           (STM32_RCC_BASE + STM32_RCC_APB1LENSR_OFFSET)
#define STM32_RCC_APB1HENSR           (STM32_RCC_BASE + STM32_RCC_APB1HENSR_OFFSET)
#define STM32_RCC_APB2ENSR            (STM32_RCC_BASE + STM32_RCC_APB2ENSR_OFFSET)
#define STM32_RCC_APB3ENSR            (STM32_RCC_BASE + STM32_RCC_APB3ENSR_OFFSET)
#define STM32_RCC_APB4LENSR           (STM32_RCC_BASE + STM32_RCC_APB4LENSR_OFFSET)
#define STM32_RCC_APB4HENSR           (STM32_RCC_BASE + STM32_RCC_APB4HENSR_OFFSET)
#define STM32_RCC_APB5ENSR            (STM32_RCC_BASE + STM32_RCC_APB5ENSR_OFFSET)
#define STM32_RCC_BUSLPENSR           (STM32_RCC_BASE + STM32_RCC_BUSLPENSR_OFFSET)
#define STM32_RCC_MISCLPENSR          (STM32_RCC_BASE + STM32_RCC_MISCLPENSR_OFFSET)
#define STM32_RCC_MEMLPENSR           (STM32_RCC_BASE + STM32_RCC_MEMLPENSR_OFFSET)
#define STM32_RCC_AHB1LPENSR          (STM32_RCC_BASE + STM32_RCC_AHB1LPENSR_OFFSET)
#define STM32_RCC_AHB2LPENSR          (STM32_RCC_BASE + STM32_RCC_AHB2LPENSR_OFFSET)
#define STM32_RCC_AHB3LPENSR          (STM32_RCC_BASE + STM32_RCC_AHB3LPENSR_OFFSET)
#define STM32_RCC_AHB4LPENSR          (STM32_RCC_BASE + STM32_RCC_AHB4LPENSR_OFFSET)
#define STM32_RCC_AHB5LPENSR          (STM32_RCC_BASE + STM32_RCC_AHB5LPENSR_OFFSET)
#define STM32_RCC_APB1LLPENSR         (STM32_RCC_BASE + STM32_RCC_APB1LLPENSR_OFFSET)
#define STM32_RCC_APB1HLPENSR         (STM32_RCC_BASE + STM32_RCC_APB1HLPENSR_OFFSET)
#define STM32_RCC_APB2LPENSR          (STM32_RCC_BASE + STM32_RCC_APB2LPENSR_OFFSET)
#define STM32_RCC_APB3LPENSR          (STM32_RCC_BASE + STM32_RCC_APB3LPENSR_OFFSET)
#define STM32_RCC_APB4LLPENSR         (STM32_RCC_BASE + STM32_RCC_APB4LLPENSR_OFFSET)
#define STM32_RCC_APB4HLPENSR         (STM32_RCC_BASE + STM32_RCC_APB4HLPENSR_OFFSET)
#define STM32_RCC_APB5LPENSR          (STM32_RCC_BASE + STM32_RCC_APB5LPENSR_OFFSET)
#define STM32_RCC_CCR                 (STM32_RCC_BASE + STM32_RCC_CCR_OFFSET)
#define STM32_RCC_STOPCCR             (STM32_RCC_BASE + STM32_RCC_STOPCCR_OFFSET)
#define STM32_RCC_MISCRSTCR           (STM32_RCC_BASE + STM32_RCC_MISCRSTCR_OFFSET)
#define STM32_RCC_MEMRSTCR            (STM32_RCC_BASE + STM32_RCC_MEMRSTCR_OFFSET)
#define STM32_RCC_AHB1RSTCR           (STM32_RCC_BASE + STM32_RCC_AHB1RSTCR_OFFSET)
#define STM32_RCC_AHB2RSTCR           (STM32_RCC_BASE + STM32_RCC_AHB2RSTCR_OFFSET)
#define STM32_RCC_AHB3RSTCR           (STM32_RCC_BASE + STM32_RCC_AHB3RSTCR_OFFSET)
#define STM32_RCC_AHB4RSTCR           (STM32_RCC_BASE + STM32_RCC_AHB4RSTCR_OFFSET)
#define STM32_RCC_AHB5RSTCR           (STM32_RCC_BASE + STM32_RCC_AHB5RSTCR_OFFSET)
#define STM32_RCC_APB1LRSTCR          (STM32_RCC_BASE + STM32_RCC_APB1LRSTCR_OFFSET)
#define STM32_RCC_APB1HRSTCR          (STM32_RCC_BASE + STM32_RCC_APB1HRSTCR_OFFSET)
#define STM32_RCC_APB2RSTCR           (STM32_RCC_BASE + STM32_RCC_APB2RSTCR_OFFSET)
#define STM32_RCC_APB4LRSTCR          (STM32_RCC_BASE + STM32_RCC_APB4LRSTCR_OFFSET)
#define STM32_RCC_APB4HRSTCR          (STM32_RCC_BASE + STM32_RCC_APB4HRSTCR_OFFSET)
#define STM32_RCC_APB5RSTCR           (STM32_RCC_BASE + STM32_RCC_APB5RSTCR_OFFSET)
#define STM32_RCC_DIVENCR             (STM32_RCC_BASE + STM32_RCC_DIVENCR_OFFSET)
#define STM32_RCC_BUSENCR             (STM32_RCC_BASE + STM32_RCC_BUSENCR_OFFSET)
#define STM32_RCC_MISCENCR            (STM32_RCC_BASE + STM32_RCC_MISCENCR_OFFSET)
#define STM32_RCC_MEMENCR             (STM32_RCC_BASE + STM32_RCC_MEMENCR_OFFSET)
#define STM32_RCC_AHB1ENCR            (STM32_RCC_BASE + STM32_RCC_AHB1ENCR_OFFSET)
#define STM32_RCC_AHB2ENCR            (STM32_RCC_BASE + STM32_RCC_AHB2ENCR_OFFSET)
#define STM32_RCC_AHB3ENCR            (STM32_RCC_BASE + STM32_RCC_AHB3ENCR_OFFSET)
#define STM32_RCC_AHB4ENCR            (STM32_RCC_BASE + STM32_RCC_AHB4ENCR_OFFSET)
#define STM32_RCC_AHB5ENCR            (STM32_RCC_BASE + STM32_RCC_AHB5ENCR_OFFSET)
#define STM32_RCC_APB1LENCR           (STM32_RCC_BASE + STM32_RCC_APB1LENCR_OFFSET)
#define STM32_RCC_APB1HENCR           (STM32_RCC_BASE + STM32_RCC_APB1HENCR_OFFSET)
#define STM32_RCC_APB2ENCR            (STM32_RCC_BASE + STM32_RCC_APB2ENCR_OFFSET)
#define STM32_RCC_APB3ENCR            (STM32_RCC_BASE + STM32_RCC_APB3ENCR_OFFSET)
#define STM32_RCC_APB4LENCR           (STM32_RCC_BASE + STM32_RCC_APB4LENCR_OFFSET)
#define STM32_RCC_APB4HENCR           (STM32_RCC_BASE + STM32_RCC_APB4HENCR_OFFSET)
#define STM32_RCC_APB5ENCR            (STM32_RCC_BASE + STM32_RCC_APB5ENCR_OFFSET)
#define STM32_RCC_BUSLPENCR           (STM32_RCC_BASE + STM32_RCC_BUSLPENCR_OFFSET)
#define STM32_RCC_MISCLPENCR          (STM32_RCC_BASE + STM32_RCC_MISCLPENCR_OFFSET)
#define STM32_RCC_MEMLPENCR           (STM32_RCC_BASE + STM32_RCC_MEMLPENCR_OFFSET)
#define STM32_RCC_AHB1LPENCR          (STM32_RCC_BASE + STM32_RCC_AHB1LPENCR_OFFSET)
#define STM32_RCC_AHB2LPENCR          (STM32_RCC_BASE + STM32_RCC_AHB2LPENCR_OFFSET)
#define STM32_RCC_AHB3LPENCR          (STM32_RCC_BASE + STM32_RCC_AHB3LPENCR_OFFSET)
#define STM32_RCC_AHB4LPENCR          (STM32_RCC_BASE + STM32_RCC_AHB4LPENCR_OFFSET)
#define STM32_RCC_AHB5LPENCR          (STM32_RCC_BASE + STM32_RCC_AHB5LPENCR_OFFSET)
#define STM32_RCC_APB1LLPENCR         (STM32_RCC_BASE + STM32_RCC_APB1LLPENCR_OFFSET)
#define STM32_RCC_APB1HLPENCR         (STM32_RCC_BASE + STM32_RCC_APB1HLPENCR_OFFSET)
#define STM32_RCC_APB2LPENCR          (STM32_RCC_BASE + STM32_RCC_APB2LPENCR_OFFSET)
#define STM32_RCC_APB3LPENCR          (STM32_RCC_BASE + STM32_RCC_APB3LPENCR_OFFSET)
#define STM32_RCC_APB4LLPENCR         (STM32_RCC_BASE + STM32_RCC_APB4LLPENCR_OFFSET)
#define STM32_RCC_APB4HLPENCR         (STM32_RCC_BASE + STM32_RCC_APB4HLPENCR_OFFSET)
#define STM32_RCC_APB5LPENCR          (STM32_RCC_BASE + STM32_RCC_APB5LPENCR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Clock control register */

#define RCC_CR_LSION                  (1 << 0)   /* Bit 0:  LSI enable */
#define RCC_CR_LSEON                  (1 << 1)   /* Bit 1: LSE oscillator
                                                  * enable */
#define RCC_CR_MSION                  (1 << 2)   /* Bit 2: MSI oscillator
                                                  * enable */
#define RCC_CR_HSION                  (1 << 3)   /* Bit 3:  HSI enable */
#define RCC_CR_HSEON                  (1 << 4)   /* Bit 4: HSE oscillator
                                                  * enable */
#define RCC_CR_PLL1ON                 (1 << 8)   /* Bit 8:  PLL1 enable */
#define RCC_CR_PLL2ON                 (1 << 9)   /* Bit 9: PLL2 enable */
#define RCC_CR_PLL3ON                 (1 << 10)  /* Bit 10: PLL3 enable */
#define RCC_CR_PLL4ON                 (1 << 11)  /* Bit 11: PLL4 enable */

/* Clock status register */

#define RCC_SR_LSIRDY                 (1 << 0)   /* Bit 0:  LSI clock ready */
#define RCC_SR_LSERDY                 (1 << 1)   /* Bit 1: LSE clock ready
                                                  * flag */
#define RCC_SR_MSIRDY                 (1 << 2)   /* Bit 2: MSI clock ready
                                                  * flag */
#define RCC_SR_HSIRDY                 (1 << 3)   /* Bit 3:  HSI clock ready */
#define RCC_SR_HSERDY                 (1 << 4)   /* Bit 4: HSE clock ready
                                                  * flag */
#define RCC_SR_PLL1RDY                (1 << 8)   /* Bit 8: PLL1 clock ready */
#define RCC_SR_PLL2RDY                (1 << 9)   /* Bit 9: PLL2 clock ready
                                                  * flag */
#define RCC_SR_PLL3RDY                (1 << 10)  /* Bit 10: PLL3 clock
                                                  * ready flag */
#define RCC_SR_PLL4RDY                (1 << 11)  /* Bit 11: PLL4 clock
                                                  * ready flag */

/* Stop mode control register (STOPCR) */

#define RCC_STOPCR_MSISTOPEN          (1 << 0)  /* Bit 0: MSI oscillator
                                                 * enable */
#define RCC_STOPCR_HSISTOPEN          (1 << 1)  /* Bit 1: HSI oscillator
                                                 * enable */

/* Clock configuration register 1.  SYSSW = 0b11 selects three IC dividers
 * (IC2 for SYSCLK, IC6 for AHB, IC11 for APB) -- the SVD names this state
 * after the first IC only.
 */

#define RCC_CFGR1_STOPWUCK            (1 << 0)  /* Bit 0: System clock
                                                 * selection after a wake
                                                 * up from system stop */
#define RCC_CFGR1_CPUSW_SHIFT         (16)      /* Bits 16-17: CPU clock
                                                 * switch selection */
#define RCC_CFGR1_CPUSW_MASK          (0x3 << RCC_CFGR1_CPUSW_SHIFT)
#define RCC_CFGR1_CPUSW_HSI           (0 << RCC_CFGR1_CPUSW_SHIFT)
#define RCC_CFGR1_CPUSW_MSI           (1 << RCC_CFGR1_CPUSW_SHIFT)
#define RCC_CFGR1_CPUSW_HSE           (2 << RCC_CFGR1_CPUSW_SHIFT)
#define RCC_CFGR1_CPUSW_IC1           (3 << RCC_CFGR1_CPUSW_SHIFT)
#define RCC_CFGR1_CPUSWS_SHIFT        (20)  /* Bits 20-21: CPU clock switch
                                             * status */
#define RCC_CFGR1_CPUSWS_MASK         (0x3 << RCC_CFGR1_CPUSWS_SHIFT)
#define RCC_CFGR1_CPUSWS_HSI          (0 << RCC_CFGR1_CPUSWS_SHIFT)
#define RCC_CFGR1_CPUSWS_MSI          (1 << RCC_CFGR1_CPUSWS_SHIFT)
#define RCC_CFGR1_CPUSWS_HSE          (2 << RCC_CFGR1_CPUSWS_SHIFT)
#define RCC_CFGR1_CPUSWS_IC1          (3 << RCC_CFGR1_CPUSWS_SHIFT)
#define RCC_CFGR1_SYSSW_SHIFT         (24)  /* Bits 24-25: System clock
                                             * switch selection */
#define RCC_CFGR1_SYSSW_MASK          (0x3 << RCC_CFGR1_SYSSW_SHIFT)
#define RCC_CFGR1_SYSSW_HSI           (0 << RCC_CFGR1_SYSSW_SHIFT)
#define RCC_CFGR1_SYSSW_MSI           (1 << RCC_CFGR1_SYSSW_SHIFT)
#define RCC_CFGR1_SYSSW_HSE           (2 << RCC_CFGR1_SYSSW_SHIFT)
#define RCC_CFGR1_SYSSW_IC2_IC6_IC11  (3 << RCC_CFGR1_SYSSW_SHIFT)
#define RCC_CFGR1_SYSSWS_SHIFT        (28)  /* Bits 28-29: System clock
                                             * switch status */
#define RCC_CFGR1_SYSSWS_MASK         (0x3 << RCC_CFGR1_SYSSWS_SHIFT)
#define RCC_CFGR1_SYSSWS_HSI          (0 << RCC_CFGR1_SYSSWS_SHIFT)
#define RCC_CFGR1_SYSSWS_MSI          (1 << RCC_CFGR1_SYSSWS_SHIFT)
#define RCC_CFGR1_SYSSWS_HSE          (2 << RCC_CFGR1_SYSSWS_SHIFT)
#define RCC_CFGR1_SYSSWS_IC2_IC6_IC11 (3 << RCC_CFGR1_SYSSWS_SHIFT)

/* Clock configuration register 2.  HPRE divides the SYSCLK fed to AHB
 * before it reaches the CPU/peripheral bus matrix.
 */

#define RCC_CFGR2_PPRE1_SHIFT         (0)  /* Bits 0-2: CPU domain APB1
                                            * prescaler */
#define RCC_CFGR2_PPRE1_MASK          (0x7 << RCC_CFGR2_PPRE1_SHIFT)
#define RCC_CFGR2_PPRE1_HCLK          (0 << RCC_CFGR2_PPRE1_SHIFT)
#define RCC_CFGR2_PPRE1_HCLKd2        (1 << RCC_CFGR2_PPRE1_SHIFT)
#define RCC_CFGR2_PPRE1_HCLKd4        (2 << RCC_CFGR2_PPRE1_SHIFT)
#define RCC_CFGR2_PPRE1_HCLKd8        (3 << RCC_CFGR2_PPRE1_SHIFT)
#define RCC_CFGR2_PPRE1_HCLKd16       (4 << RCC_CFGR2_PPRE1_SHIFT)
#define RCC_CFGR2_PPRE1_HCLKd32       (5 << RCC_CFGR2_PPRE1_SHIFT)
#define RCC_CFGR2_PPRE1_HCLKd64       (6 << RCC_CFGR2_PPRE1_SHIFT)
#define RCC_CFGR2_PPRE1_HCLKd128      (7 << RCC_CFGR2_PPRE1_SHIFT)
#define RCC_CFGR2_PPRE2_SHIFT         (4)  /* Bits 4-6: CPU domain APB2
                                            * prescaler */
#define RCC_CFGR2_PPRE2_MASK          (0x7 << RCC_CFGR2_PPRE2_SHIFT)
#define RCC_CFGR2_PPRE2_HCLK          (0 << RCC_CFGR2_PPRE2_SHIFT)
#define RCC_CFGR2_PPRE2_HCLKd2        (1 << RCC_CFGR2_PPRE2_SHIFT)
#define RCC_CFGR2_PPRE2_HCLKd4        (2 << RCC_CFGR2_PPRE2_SHIFT)
#define RCC_CFGR2_PPRE2_HCLKd8        (3 << RCC_CFGR2_PPRE2_SHIFT)
#define RCC_CFGR2_PPRE2_HCLKd16       (4 << RCC_CFGR2_PPRE2_SHIFT)
#define RCC_CFGR2_PPRE2_HCLKd32       (5 << RCC_CFGR2_PPRE2_SHIFT)
#define RCC_CFGR2_PPRE2_HCLKd64       (6 << RCC_CFGR2_PPRE2_SHIFT)
#define RCC_CFGR2_PPRE2_HCLKd128      (7 << RCC_CFGR2_PPRE2_SHIFT)
#define RCC_CFGR2_PPRE4_SHIFT         (12)  /* Bits 12-14: CPU domain APB4
                                             * prescaler */
#define RCC_CFGR2_PPRE4_MASK          (0x7 << RCC_CFGR2_PPRE4_SHIFT)
#define RCC_CFGR2_PPRE4_HCLK          (0 << RCC_CFGR2_PPRE4_SHIFT)
#define RCC_CFGR2_PPRE4_HCLKd2        (1 << RCC_CFGR2_PPRE4_SHIFT)
#define RCC_CFGR2_PPRE4_HCLKd4        (2 << RCC_CFGR2_PPRE4_SHIFT)
#define RCC_CFGR2_PPRE4_HCLKd8        (3 << RCC_CFGR2_PPRE4_SHIFT)
#define RCC_CFGR2_PPRE4_HCLKd16       (4 << RCC_CFGR2_PPRE4_SHIFT)
#define RCC_CFGR2_PPRE4_HCLKd32       (5 << RCC_CFGR2_PPRE4_SHIFT)
#define RCC_CFGR2_PPRE4_HCLKd64       (6 << RCC_CFGR2_PPRE4_SHIFT)
#define RCC_CFGR2_PPRE4_HCLKd128      (7 << RCC_CFGR2_PPRE4_SHIFT)
#define RCC_CFGR2_PPRE5_SHIFT         (16)  /* Bits 16-18: CPU domain APB5
                                             * prescaler */
#define RCC_CFGR2_PPRE5_MASK          (0x7 << RCC_CFGR2_PPRE5_SHIFT)
#define RCC_CFGR2_PPRE5_HCLK          (0 << RCC_CFGR2_PPRE5_SHIFT)
#define RCC_CFGR2_PPRE5_HCLKd2        (1 << RCC_CFGR2_PPRE5_SHIFT)
#define RCC_CFGR2_PPRE5_HCLKd4        (2 << RCC_CFGR2_PPRE5_SHIFT)
#define RCC_CFGR2_PPRE5_HCLKd8        (3 << RCC_CFGR2_PPRE5_SHIFT)
#define RCC_CFGR2_PPRE5_HCLKd16       (4 << RCC_CFGR2_PPRE5_SHIFT)
#define RCC_CFGR2_PPRE5_HCLKd32       (5 << RCC_CFGR2_PPRE5_SHIFT)
#define RCC_CFGR2_PPRE5_HCLKd64       (6 << RCC_CFGR2_PPRE5_SHIFT)
#define RCC_CFGR2_PPRE5_HCLKd128      (7 << RCC_CFGR2_PPRE5_SHIFT)
#define RCC_CFGR2_HPRE_SHIFT          (20)  /* Bits 20-22: AHB clock
                                             * prescaler */
#define RCC_CFGR2_HPRE_MASK           (0x7 << RCC_CFGR2_HPRE_SHIFT)
#define RCC_CFGR2_HPRE_SYSCLK         (0 << RCC_CFGR2_HPRE_SHIFT)
#define RCC_CFGR2_HPRE_SYSCLKd2       (1 << RCC_CFGR2_HPRE_SHIFT)
#define RCC_CFGR2_HPRE_SYSCLKd4       (2 << RCC_CFGR2_HPRE_SHIFT)
#define RCC_CFGR2_HPRE_SYSCLKd8       (3 << RCC_CFGR2_HPRE_SHIFT)
#define RCC_CFGR2_HPRE_SYSCLKd16      (4 << RCC_CFGR2_HPRE_SHIFT)
#define RCC_CFGR2_HPRE_SYSCLKd32      (5 << RCC_CFGR2_HPRE_SHIFT)
#define RCC_CFGR2_HPRE_SYSCLKd64      (6 << RCC_CFGR2_HPRE_SHIFT)
#define RCC_CFGR2_HPRE_SYSCLKd128     (7 << RCC_CFGR2_HPRE_SHIFT)
#define RCC_CFGR2_TIMPRE_SHIFT        (24)  /* Bits 24-25: Timer clock
                                             * prescaler selection */
#define RCC_CFGR2_TIMPRE_MASK         (0x3 << RCC_CFGR2_TIMPRE_SHIFT)
#define RCC_CFGR2_TIMPRE_SYSCLK       (0 << RCC_CFGR2_TIMPRE_SHIFT)
#define RCC_CFGR2_TIMPRE_SYSCLKd2     (1 << RCC_CFGR2_TIMPRE_SHIFT)
#define RCC_CFGR2_TIMPRE_SYSCLKd4     (2 << RCC_CFGR2_TIMPRE_SHIFT)
#define RCC_CFGR2_TIMPRE_SYSCLKd8     (3 << RCC_CFGR2_TIMPRE_SHIFT)

/* Backup domain protection register (BDCR) */

#define RCC_BDCR_VSWRST               (1 << 31)  /* Bit 31: Vswitch (VSW)
                                                  * domain software reset */

/* Reset status register for hardware (HWRSR) */

#define RCC_HWRSR_RMVF                (1 << 16)  /* Bit 16: Remove reset
                                                  * flag */
#define RCC_HWRSR_LCKRSTF             (1 << 17)  /* Bit 17: CPU lockup
                                                  * reset flag */
#define RCC_HWRSR_BORRSTF             (1 << 21)  /* Bit 21: BOR reset flag */
#define RCC_HWRSR_PINRSTF             (1 << 22)  /* Bit 22: Pin reset flag
                                                  * (NRST) */
#define RCC_HWRSR_PORRSTF             (1 << 23)  /* Bit 23: POR/PDR reset
                                                  * flag */
#define RCC_HWRSR_SFTRSTF             (1 << 24)  /* Bit 24: Software system
                                                  * reset flag */
#define RCC_HWRSR_IWDGRSTF            (1 << 26)  /* Bit 26: Independent
                                                  * watchdog reset flag */
#define RCC_HWRSR_WWDGRSTF            (1 << 28)  /* Bit 28: Window watchdog
                                                  * reset flag */
#define RCC_HWRSR_LPWRRSTF            (1 << 30)  /* Bit 30: Illegal Stop or
                                                  * Standby flag */

/* Reset register (RSR) */

#define RCC_RSR_RMVF                  (1 << 16)  /* Bit 16: Remove reset
                                                  * flag */
#define RCC_RSR_LCKRSTF               (1 << 17)  /* Bit 17: CPU lockup
                                                  * reset flag */
#define RCC_RSR_BORRSTF               (1 << 21)  /* Bit 21: BOR reset flag */
#define RCC_RSR_PINRSTF               (1 << 22)  /* Bit 22: Pin reset flag
                                                  * (NRST) */
#define RCC_RSR_PORRSTF               (1 << 23)  /* Bit 23: POR/PDR reset
                                                  * flag */
#define RCC_RSR_SFTRSTF               (1 << 24)  /* Bit 24: Software system
                                                  * reset flag */
#define RCC_RSR_IWDGRSTF              (1 << 26)  /* Bit 26: Independent
                                                  * watchdog reset flag */
#define RCC_RSR_WWDGRSTF              (1 << 28)  /* Bit 28: Window watchdog
                                                  * reset flag */
#define RCC_RSR_LPWRRSTF              (1 << 30)  /* Bit 30: Illegal Stop or
                                                  * Standby flag */

/* LSE configuration register (LSECFGR) */

#define RCC_LSECFGR_LSECSSON          (1 << 7)   /* Bit 7: LSE clock
                                                  * security system (CSS)
                                                  * enable */
#define RCC_LSECFGR_LSECSSRA          (1 << 8)   /* Bit 8: LSE clock
                                                  * security system (CSS)
                                                  * rearm function */
#define RCC_LSECFGR_LSECSSD           (1 << 9)   /* Bit 9: LSE clock
                                                  * security system (CSS)
                                                  * failure detection */
#define RCC_LSECFGR_LSEBYP            (1 << 15)  /* Bit 15: LSE clock
                                                  * bypass */
#define RCC_LSECFGR_LSEEXT            (1 << 16)  /* Bit 16: LSE clock type
                                                  * in bypass mode */
#define RCC_LSECFGR_LSEGFON           (1 << 17)  /* Bit 17: LSE clock
                                                  * glitch filter enable */
#define RCC_LSECFGR_LSEDRV_SHIFT      (18)       /* Bits 18-19: LSE
                                                  * oscillator driving
                                                  * capability */
#define RCC_LSECFGR_LSEDRV_MASK       (0x3 << RCC_LSECFGR_LSEDRV_SHIFT)

/* MSI configuration register (MSICFGR) */

#define RCC_MSICFGR_MSIFREQSEL        (1 << 9)  /* Bit 9: MSI oscillator
                                                 * frequency selection */
#define RCC_MSICFGR_MSITRIM_SHIFT     (16)      /* Bits 16-20: MSI clock
                                                 * trimming */
#define RCC_MSICFGR_MSITRIM_MASK      (0x1f << RCC_MSICFGR_MSITRIM_SHIFT)
#define RCC_MSICFGR_MSICAL_SHIFT      (23)  /* Bits 23-30: MSI clock
                                             * calibration */
#define RCC_MSICFGR_MSICAL_MASK       (0xff << RCC_MSICFGR_MSICAL_SHIFT)

/* HSI configuration register (HSICFGR) */

#define RCC_HSICFGR_HSIDIV_SHIFT      (7)  /* Bits 7-8: HSI clock divider */
#define RCC_HSICFGR_HSIDIV_MASK       (0x3 << RCC_HSICFGR_HSIDIV_SHIFT)
#define RCC_HSICFGR_HSIDIV_1          (0 << RCC_HSICFGR_HSIDIV_SHIFT)
#define RCC_HSICFGR_HSIDIV_2          (1 << RCC_HSICFGR_HSIDIV_SHIFT)
#define RCC_HSICFGR_HSIDIV_4          (2 << RCC_HSICFGR_HSIDIV_SHIFT)
#define RCC_HSICFGR_HSIDIV_8          (3 << RCC_HSICFGR_HSIDIV_SHIFT)
#define RCC_HSICFGR_HSITRIM_SHIFT     (16)  /* Bits 16-22: HSI clock
                                             * trimming */
#define RCC_HSICFGR_HSITRIM_MASK      (0x7f << RCC_HSICFGR_HSITRIM_SHIFT)
#define RCC_HSICFGR_HSICAL_SHIFT      (23)  /* Bits 23-31: HSI clock
                                             * calibration */
#define RCC_HSICFGR_HSICAL_MASK       (0x1ff << RCC_HSICFGR_HSICAL_SHIFT)

/* HSE configuration register (HSECFGR) */

#define RCC_HSECFGR_HSEDIV2SEL        (1 << 6)   /* Bit 6: HSE div2 clock
                                                  * source select */
#define RCC_HSECFGR_HSECSSON          (1 << 7)   /* Bit 7: HSE CSS enable */
#define RCC_HSECFGR_HSECSSD           (1 << 9)   /* Bit 9: HSE CSS failure
                                                  * detection */
#define RCC_HSECFGR_HSECSSBYP         (1 << 10)  /* Bit 10: HSE CSS bypass
                                                  * enable */
#define RCC_HSECFGR_HSECSSBPRE_SHIFT  (11)       /* Bits 11-14: HSE CSS
                                                  * bypass divider */
#define RCC_HSECFGR_HSECSSBPRE_MASK   (0xf << RCC_HSECFGR_HSECSSBPRE_SHIFT)
#define RCC_HSECFGR_HSEBYP            (1 << 15)  /* Bit 15: HSE clock
                                                  * bypass */
#define RCC_HSECFGR_HSEEXT            (1 << 16)  /* Bit 16: HSE clock type
                                                  * in bypass mode */

/* PLL1 configuration register 1 */

#define RCC_PLL1CFGR1_DIVN_SHIFT      (8)  /* Bits 19-8: Feedback divider */
#define RCC_PLL1CFGR1_DIVN_MASK       (0xfff << RCC_PLL1CFGR1_DIVN_SHIFT)
#define RCC_PLL1CFGR1_DIVM_SHIFT      (20)  /* Bits 25-20: Reference
                                             * divider */
#define RCC_PLL1CFGR1_DIVM_MASK       (0x3f << RCC_PLL1CFGR1_DIVM_SHIFT)
#define RCC_PLL1CFGR1_BYP             (1 << 27)  /* Bit 27: PLL1 bypass */
#define RCC_PLL1CFGR1_SEL_SHIFT       (28)       /* Bits 28-30: PLL1 source
                                                  * selection of the
                                                  * reference clock */
#define RCC_PLL1CFGR1_SEL_MASK        (0x7 << RCC_PLL1CFGR1_SEL_SHIFT)
#define RCC_PLL1CFGR1_SEL_HSI         (0 << RCC_PLL1CFGR1_SEL_SHIFT)
#define RCC_PLL1CFGR1_SEL_MSI         (1 << RCC_PLL1CFGR1_SEL_SHIFT)
#define RCC_PLL1CFGR1_SEL_HSE         (2 << RCC_PLL1CFGR1_SEL_SHIFT)
#define RCC_PLL1CFGR1_SEL_I2S_CKIN    (3 << RCC_PLL1CFGR1_SEL_SHIFT)

/* PLL1..PLL4 configuration register 2 (common layout) */

#define RCC_PLL1CFGR2_DIVNFRAC_SHIFT  (0)  /* Bits 0-23: PLL1 fractional
                                            * part of the VCO
                                            * multiplication factor */
#define RCC_PLL1CFGR2_DIVNFRAC_MASK   (0xffffff << RCC_PLL1CFGR2_DIVNFRAC_SHIFT)

/* PLL1 configuration register 3 */

#define RCC_PLL1CFGR3_MODSSRST        (1 << 0)  /* Bit 0: PLL1 modulation
                                                 * spread spectrum reset */
#define RCC_PLL1CFGR3_DACEN           (1 << 1)  /* Bit 1: PLL1 noise
                                                 * canceling DAC enable in
                                                 * fractional mode */
#define RCC_PLL1CFGR3_MODSSDIS        (1 << 2)  /* Bit 2: Modulation spread
                                                 * spectrum disable */
#define RCC_PLL1CFGR3_MODDSEN         (1 << 3)  /* Bit 3: PLL1 modulation
                                                 * spread spectrum (and
                                                 * fractional divide)
                                                 * enable */
#define RCC_PLL1CFGR3_MODSPRDW        (1 << 4)  /* Bit 4: PLL1 modulation
                                                 * spread spectrum down */
#define RCC_PLL1CFGR3_MODDIV_SHIFT    (8)       /* Bits 8-11: PLL1
                                                 * modulation division
                                                 * frequency adjustment */
#define RCC_PLL1CFGR3_MODDIV_MASK     (0xf << RCC_PLL1CFGR3_MODDIV_SHIFT)
#define RCC_PLL1CFGR3_MODSPR_SHIFT    (16)  /* Bits 16-20: PLL1 modulation
                                             * spread depth adjustment */
#define RCC_PLL1CFGR3_MODSPR_MASK     (0x1f << RCC_PLL1CFGR3_MODSPR_SHIFT)
#define RCC_PLL1CFGR3_PDIV2_SHIFT     (24)  /* Bits 26-24: Post-divider 2 */
#define RCC_PLL1CFGR3_PDIV2_MASK      (0x7 << RCC_PLL1CFGR3_PDIV2_SHIFT)
#define RCC_PLL1CFGR3_PDIV1_SHIFT     (27)  /* Bits 29-27: Post-divider 1 */
#define RCC_PLL1CFGR3_PDIV1_MASK      (0x7 << RCC_PLL1CFGR3_PDIV1_SHIFT)
#define RCC_PLL1CFGR3_PDIVEN          (1 << 30)  /* Bit 30: Post-divider
                                                  * and PLL output enable */

/* IC1..IC20 configuration registers -- all share the same layout.  Field
 * SEL selects the PLL source (PLL1..PLL4); field INT is an 8-bit integer
 * divider where INT[7:0] = N-1 yielding a divide ratio of N.
 */

#define RCC_ICCFGR_INT_SHIFT          (16)  /* Bits 16-23: Divider IC1
                                             * integer division factor */
#define RCC_ICCFGR_INT_MASK           (0xff << RCC_ICCFGR_INT_SHIFT)
#define RCC_ICCFGR_SEL_SHIFT          (28)  /* Bits 28-29: Divider IC1
                                             * source selection */
#define RCC_ICCFGR_SEL_MASK           (0x3 << RCC_ICCFGR_SEL_SHIFT)
#define RCC_ICCFGR_SEL_PLL1           (0 << RCC_ICCFGR_SEL_SHIFT)
#define RCC_ICCFGR_SEL_PLL2           (1 << RCC_ICCFGR_SEL_SHIFT)
#define RCC_ICCFGR_SEL_PLL3           (2 << RCC_ICCFGR_SEL_SHIFT)
#define RCC_ICCFGR_SEL_PLL4           (3 << RCC_ICCFGR_SEL_SHIFT)

/* Clock-source interrupt enable register (CIER) */

#define RCC_CIER_LSIRDYIE             (1 << 0)   /* Bit 0: LSI ready
                                                  * interrupt enable */
#define RCC_CIER_LSERDYIE             (1 << 1)   /* Bit 1: LSE ready
                                                  * interrupt enable */
#define RCC_CIER_MSIRDYIE             (1 << 2)   /* Bit 2: MSI ready
                                                  * interrupt enable */
#define RCC_CIER_HSIRDYIE             (1 << 3)   /* Bit 3: HSI ready
                                                  * interrupt enable */
#define RCC_CIER_HSERDYIE             (1 << 4)   /* Bit 4: HSE ready
                                                  * interrupt enable */
#define RCC_CIER_PLL1RDYIE            (1 << 8)   /* Bit 8: PLL1 ready
                                                  * interrupt enable */
#define RCC_CIER_PLL2RDYIE            (1 << 9)   /* Bit 9: PLL2 ready
                                                  * interrupt enable */
#define RCC_CIER_PLL3RDYIE            (1 << 10)  /* Bit 10: PLL3 ready
                                                  * interrupt enable */
#define RCC_CIER_PLL4RDYIE            (1 << 11)  /* Bit 11: PLL4 ready
                                                  * interrupt enable */
#define RCC_CIER_LSECSSIE             (1 << 16)  /* Bit 16: LSE CSS
                                                  * interrupt enable */
#define RCC_CIER_HSECSSIE             (1 << 17)  /* Bit 17: HSE CSS
                                                  * interrupt enable */
#define RCC_CIER_WKUPIE               (1 << 24)  /* Bit 24: CPU wake-up
                                                  * from Stop interrupt
                                                  * enable */

/* Clock-source interrupt flag register (CIFR) */

#define RCC_CIFR_LSIRDYF              (1 << 0)   /* Bit 0: LSI ready
                                                  * interrupt flag */
#define RCC_CIFR_LSERDYF              (1 << 1)   /* Bit 1: LSE ready
                                                  * interrupt flag */
#define RCC_CIFR_MSIRDYF              (1 << 2)   /* Bit 2: MSI ready
                                                  * interrupt flag */
#define RCC_CIFR_HSIRDYF              (1 << 3)   /* Bit 3: HSI ready
                                                  * interrupt flag */
#define RCC_CIFR_HSERDYF              (1 << 4)   /* Bit 4: HSE ready
                                                  * interrupt flag */
#define RCC_CIFR_PLL1RDYF             (1 << 8)   /* Bit 8: PLL1 ready
                                                  * interrupt flag */
#define RCC_CIFR_PLL2RDYF             (1 << 9)   /* Bit 9: PLL2 ready
                                                  * interrupt flag */
#define RCC_CIFR_PLL3RDYF             (1 << 10)  /* Bit 10: PLL3 ready
                                                  * interrupt flag */
#define RCC_CIFR_PLL4RDYF             (1 << 11)  /* Bit 11: PLL4 ready
                                                  * interrupt flag */
#define RCC_CIFR_LSECSSF              (1 << 16)  /* Bit 16: LSE ready
                                                  * interrupt flag */
#define RCC_CIFR_HSECSSF              (1 << 17)  /* Bit 17: HSE ready
                                                  * interrupt flag */
#define RCC_CIFR_WKUPF                (1 << 24)  /* Bit 24: CPU wake-up
                                                  * from Stop interrupt
                                                  * flag */

/* Clock-source interrupt clear register (CICR) */

#define RCC_CICR_LSIRDYC              (1 << 0)   /* Bit 0: LSI ready
                                                  * interrupt clear */
#define RCC_CICR_LSERDYC              (1 << 1)   /* Bit 1: LSE ready
                                                  * interrupt clear */
#define RCC_CICR_MSIRDYC              (1 << 2)   /* Bit 2: MSI ready
                                                  * interrupt clear */
#define RCC_CICR_HSIRDYC              (1 << 3)   /* Bit 3: HSI ready
                                                  * interrupt clear */
#define RCC_CICR_HSERDYC              (1 << 4)   /* Bit 4: HSE ready
                                                  * interrupt clear */
#define RCC_CICR_PLL1RDYC             (1 << 8)   /* Bit 8: PLL1 ready
                                                  * interrupt clear */
#define RCC_CICR_PLL2RDYC             (1 << 9)   /* Bit 9: PLL2 ready
                                                  * interrupt clear */
#define RCC_CICR_PLL3RDYC             (1 << 10)  /* Bit 10: PLL3 ready
                                                  * interrupt clear */
#define RCC_CICR_PLL4RDYC             (1 << 11)  /* Bit 11: PLL4 ready
                                                  * interrupt clear */
#define RCC_CICR_LSECSSC              (1 << 16)  /* Bit 16: LSE ready
                                                  * interrupt clear */
#define RCC_CICR_HSECSSC              (1 << 17)  /* Bit 17: HSE ready
                                                  * interrupt clear */
#define RCC_CICR_WKUPFC               (1 << 24)  /* Bit 24: CPU wake-up
                                                  * ready interrupt clear */

/* Clock configuration for independent peripheral register 1 (CCIPR1) */

#define RCC_CCIPR1_ADF1SEL_SHIFT      (0)  /* Bits 0-2: Source selection
                                            * for the ADF1 kernel clock */
#define RCC_CCIPR1_ADF1SEL_MASK       (0x7 << RCC_CCIPR1_ADF1SEL_SHIFT)
#define RCC_CCIPR1_ADF1SEL_HCLK       (0 << RCC_CCIPR1_ADF1SEL_SHIFT)
#define RCC_CCIPR1_ADF1SEL_CLKP       (1 << RCC_CCIPR1_ADF1SEL_SHIFT)
#define RCC_CCIPR1_ADF1SEL_IC7        (2 << RCC_CCIPR1_ADF1SEL_SHIFT)
#define RCC_CCIPR1_ADF1SEL_IC8        (3 << RCC_CCIPR1_ADF1SEL_SHIFT)
#define RCC_CCIPR1_ADF1SEL_MSI        (4 << RCC_CCIPR1_ADF1SEL_SHIFT)
#define RCC_CCIPR1_ADF1SEL_HSI        (5 << RCC_CCIPR1_ADF1SEL_SHIFT)
#define RCC_CCIPR1_ADF1SEL_I2S_CKIN   (6 << RCC_CCIPR1_ADF1SEL_SHIFT)
#define RCC_CCIPR1_ADF1SEL_TIMG       (7 << RCC_CCIPR1_ADF1SEL_SHIFT)
#define RCC_CCIPR1_ADC12SEL_SHIFT     (4)  /* Bits 4-6: Source selection
                                            * for the ADC12 kernel clock */
#define RCC_CCIPR1_ADC12SEL_MASK      (0x7 << RCC_CCIPR1_ADC12SEL_SHIFT)
#define RCC_CCIPR1_ADC12SEL_HCLK      (0 << RCC_CCIPR1_ADC12SEL_SHIFT)
#define RCC_CCIPR1_ADC12SEL_CLKP      (1 << RCC_CCIPR1_ADC12SEL_SHIFT)
#define RCC_CCIPR1_ADC12SEL_IC7       (2 << RCC_CCIPR1_ADC12SEL_SHIFT)
#define RCC_CCIPR1_ADC12SEL_IC8       (3 << RCC_CCIPR1_ADC12SEL_SHIFT)
#define RCC_CCIPR1_ADC12SEL_MSI       (4 << RCC_CCIPR1_ADC12SEL_SHIFT)
#define RCC_CCIPR1_ADC12SEL_HSI       (5 << RCC_CCIPR1_ADC12SEL_SHIFT)
#define RCC_CCIPR1_ADC12SEL_I2S_CKIN  (6 << RCC_CCIPR1_ADC12SEL_SHIFT)
#define RCC_CCIPR1_ADC12SEL_TIMG      (7 << RCC_CCIPR1_ADC12SEL_SHIFT)
#define RCC_CCIPR1_ADCPRE_SHIFT       (8)  /* Bits 8-15: ADC12 Bus Slave
                                            * clock divider selection (for
                                            * clock ck_icn_s_vencram) */
#define RCC_CCIPR1_ADCPRE_MASK        (0xff << RCC_CCIPR1_ADCPRE_SHIFT)
#define RCC_CCIPR1_DCMIPPSEL_SHIFT    (20)  /* Bits 20-21: Source selection
                                             * for the DCMIPP kernel clock */
#define RCC_CCIPR1_DCMIPPSEL_MASK     (0x3 << RCC_CCIPR1_DCMIPPSEL_SHIFT)
#define RCC_CCIPR1_DCMIPPSEL_PCLK5    (0 << RCC_CCIPR1_DCMIPPSEL_SHIFT)
#define RCC_CCIPR1_DCMIPPSEL_CLKP     (1 << RCC_CCIPR1_DCMIPPSEL_SHIFT)
#define RCC_CCIPR1_DCMIPPSEL_IC17     (2 << RCC_CCIPR1_DCMIPPSEL_SHIFT)
#define RCC_CCIPR1_DCMIPPSEL_HSI      (3 << RCC_CCIPR1_DCMIPPSEL_SHIFT)

/* Clock configuration for independent peripheral register 2 (CCIPR2) */

#define RCC_CCIPR2_ETH1PTPSEL_SHIFT   (0)  /* Bits 0-1: Source selection
                                            * for the ETH1 kernel clock */
#define RCC_CCIPR2_ETH1PTPSEL_MASK    (0x3 << RCC_CCIPR2_ETH1PTPSEL_SHIFT)
#define RCC_CCIPR2_ETH1PTPSEL_HCLK    (0 << RCC_CCIPR2_ETH1PTPSEL_SHIFT)
#define RCC_CCIPR2_ETH1PTPSEL_CLKP    (1 << RCC_CCIPR2_ETH1PTPSEL_SHIFT)
#define RCC_CCIPR2_ETH1PTPSEL_IC13    (2 << RCC_CCIPR2_ETH1PTPSEL_SHIFT)
#define RCC_CCIPR2_ETH1PTPSEL_HSE     (3 << RCC_CCIPR2_ETH1PTPSEL_SHIFT)
#define RCC_CCIPR2_ETH1PTPDIV_SHIFT   (4)  /* Bits 4-7: ETH1 Kernel clock
                                            * divider selection (for clock
                                            * ck_ker_eth1ptp) */
#define RCC_CCIPR2_ETH1PTPDIV_MASK    (0xf << RCC_CCIPR2_ETH1PTPDIV_SHIFT)
#define RCC_CCIPR2_ETH1PWRDOWNACK     (1 << 8)  /* Bit 8: Set and reset by
                                                 * software */
#define RCC_CCIPR2_ETH1CLKSEL_SHIFT   (12)      /* Bits 12-13: Source
                                                 * selection for the ETH1
                                                 * kernel clock */
#define RCC_CCIPR2_ETH1CLKSEL_MASK    (0x3 << RCC_CCIPR2_ETH1CLKSEL_SHIFT)
#define RCC_CCIPR2_ETH1CLKSEL_HCLK    (0 << RCC_CCIPR2_ETH1CLKSEL_SHIFT)
#define RCC_CCIPR2_ETH1CLKSEL_CLKP    (1 << RCC_CCIPR2_ETH1CLKSEL_SHIFT)
#define RCC_CCIPR2_ETH1CLKSEL_IC12    (2 << RCC_CCIPR2_ETH1CLKSEL_SHIFT)
#define RCC_CCIPR2_ETH1CLKSEL_HSE     (3 << RCC_CCIPR2_ETH1CLKSEL_SHIFT)
#define RCC_CCIPR2_ETH1SEL_SHIFT      (16)  /* Bits 16-18: Set and reset by
                                             * software */
#define RCC_CCIPR2_ETH1SEL_MASK       (0x7 << RCC_CCIPR2_ETH1SEL_SHIFT)
#define RCC_CCIPR2_ETH1SEL_MII        (0 << RCC_CCIPR2_ETH1SEL_SHIFT)
#define RCC_CCIPR2_ETH1SEL_RGMII      (1 << RCC_CCIPR2_ETH1SEL_SHIFT)
#define RCC_CCIPR2_ETH1SEL_RMII       (4 << RCC_CCIPR2_ETH1SEL_SHIFT)
#define RCC_CCIPR2_ETH1REFCLKSEL      (1 << 20)  /* Bit 20: Set and reset
                                                  * by software */
#define RCC_CCIPR2_ETH1GTXCLKSEL      (1 << 24)  /* Bit 24: Set and reset
                                                  * by software */

/* Clock configuration for independent peripheral register 3 (CCIPR3) */

#define RCC_CCIPR3_FDCANSEL_SHIFT     (0)  /* Bits 0-1: Source selection
                                            * for the FDCAN kernel clock */
#define RCC_CCIPR3_FDCANSEL_MASK      (0x3 << RCC_CCIPR3_FDCANSEL_SHIFT)
#define RCC_CCIPR3_FDCANSEL_PCLK1     (0 << RCC_CCIPR3_FDCANSEL_SHIFT)
#define RCC_CCIPR3_FDCANSEL_CLKP      (1 << RCC_CCIPR3_FDCANSEL_SHIFT)
#define RCC_CCIPR3_FDCANSEL_IC19      (2 << RCC_CCIPR3_FDCANSEL_SHIFT)
#define RCC_CCIPR3_FDCANSEL_HSE       (3 << RCC_CCIPR3_FDCANSEL_SHIFT)
#define RCC_CCIPR3_FMCSEL_SHIFT       (4)  /* Bits 4-5: Source selection
                                            * for the FMC kernel clock */
#define RCC_CCIPR3_FMCSEL_MASK        (0x3 << RCC_CCIPR3_FMCSEL_SHIFT)
#define RCC_CCIPR3_FMCSEL_HCLK        (0 << RCC_CCIPR3_FMCSEL_SHIFT)
#define RCC_CCIPR3_FMCSEL_CLKP        (1 << RCC_CCIPR3_FMCSEL_SHIFT)
#define RCC_CCIPR3_FMCSEL_IC3         (2 << RCC_CCIPR3_FMCSEL_SHIFT)
#define RCC_CCIPR3_DFTSEL             (1 << 8)  /* Bit 8: Source selection
                                                 * for the DFT kernel clock */

/* Clock configuration for independent peripheral register 4 (CCIPR4) */

#define RCC_CCIPR4_I2C1SEL_SHIFT      (0)  /* Bits 0-2: Source selection
                                            * for the I2C1 kernel clock */
#define RCC_CCIPR4_I2C1SEL_MASK       (0x7 << RCC_CCIPR4_I2C1SEL_SHIFT)
#define RCC_CCIPR4_I2C1SEL_PCLK1      (0 << RCC_CCIPR4_I2C1SEL_SHIFT)
#define RCC_CCIPR4_I2C1SEL_CLKP       (1 << RCC_CCIPR4_I2C1SEL_SHIFT)
#define RCC_CCIPR4_I2C1SEL_IC10       (2 << RCC_CCIPR4_I2C1SEL_SHIFT)
#define RCC_CCIPR4_I2C1SEL_IC15       (3 << RCC_CCIPR4_I2C1SEL_SHIFT)
#define RCC_CCIPR4_I2C1SEL_MSI        (4 << RCC_CCIPR4_I2C1SEL_SHIFT)
#define RCC_CCIPR4_I2C1SEL_HSI        (5 << RCC_CCIPR4_I2C1SEL_SHIFT)
#define RCC_CCIPR4_I2C2SEL_SHIFT      (4)  /* Bits 4-6: Source selection
                                            * for the I2C2 kernel clock */
#define RCC_CCIPR4_I2C2SEL_MASK       (0x7 << RCC_CCIPR4_I2C2SEL_SHIFT)
#define RCC_CCIPR4_I2C2SEL_PCLK1      (0 << RCC_CCIPR4_I2C2SEL_SHIFT)
#define RCC_CCIPR4_I2C2SEL_CLKP       (1 << RCC_CCIPR4_I2C2SEL_SHIFT)
#define RCC_CCIPR4_I2C2SEL_IC10       (2 << RCC_CCIPR4_I2C2SEL_SHIFT)
#define RCC_CCIPR4_I2C2SEL_IC15       (3 << RCC_CCIPR4_I2C2SEL_SHIFT)
#define RCC_CCIPR4_I2C2SEL_MSI        (4 << RCC_CCIPR4_I2C2SEL_SHIFT)
#define RCC_CCIPR4_I2C2SEL_HSI        (5 << RCC_CCIPR4_I2C2SEL_SHIFT)
#define RCC_CCIPR4_I2C3SEL_SHIFT      (8)  /* Bits 8-10: Source selection
                                            * for the I2C3 kernel clock */
#define RCC_CCIPR4_I2C3SEL_MASK       (0x7 << RCC_CCIPR4_I2C3SEL_SHIFT)
#define RCC_CCIPR4_I2C3SEL_PCLK1      (0 << RCC_CCIPR4_I2C3SEL_SHIFT)
#define RCC_CCIPR4_I2C3SEL_CLKP       (1 << RCC_CCIPR4_I2C3SEL_SHIFT)
#define RCC_CCIPR4_I2C3SEL_IC10       (2 << RCC_CCIPR4_I2C3SEL_SHIFT)
#define RCC_CCIPR4_I2C3SEL_IC15       (3 << RCC_CCIPR4_I2C3SEL_SHIFT)
#define RCC_CCIPR4_I2C3SEL_MSI        (4 << RCC_CCIPR4_I2C3SEL_SHIFT)
#define RCC_CCIPR4_I2C3SEL_HSI        (5 << RCC_CCIPR4_I2C3SEL_SHIFT)
#define RCC_CCIPR4_I2C4SEL_SHIFT      (12)  /* Bits 12-14: Source selection
                                             * for the I2C4 kernel clock */
#define RCC_CCIPR4_I2C4SEL_MASK       (0x7 << RCC_CCIPR4_I2C4SEL_SHIFT)
#define RCC_CCIPR4_I2C4SEL_PCLK1      (0 << RCC_CCIPR4_I2C4SEL_SHIFT)
#define RCC_CCIPR4_I2C4SEL_CLKP       (1 << RCC_CCIPR4_I2C4SEL_SHIFT)
#define RCC_CCIPR4_I2C4SEL_IC10       (2 << RCC_CCIPR4_I2C4SEL_SHIFT)
#define RCC_CCIPR4_I2C4SEL_IC15       (3 << RCC_CCIPR4_I2C4SEL_SHIFT)
#define RCC_CCIPR4_I2C4SEL_MSI        (4 << RCC_CCIPR4_I2C4SEL_SHIFT)
#define RCC_CCIPR4_I2C4SEL_HSI        (5 << RCC_CCIPR4_I2C4SEL_SHIFT)
#define RCC_CCIPR4_I3C1SEL_SHIFT      (16)  /* Bits 16-18: Source selection
                                             * for the I3C1 kernel clock */
#define RCC_CCIPR4_I3C1SEL_MASK       (0x7 << RCC_CCIPR4_I3C1SEL_SHIFT)
#define RCC_CCIPR4_I3C1SEL_PCLK1      (0 << RCC_CCIPR4_I3C1SEL_SHIFT)
#define RCC_CCIPR4_I3C1SEL_CLKP       (1 << RCC_CCIPR4_I3C1SEL_SHIFT)
#define RCC_CCIPR4_I3C1SEL_IC10       (2 << RCC_CCIPR4_I3C1SEL_SHIFT)
#define RCC_CCIPR4_I3C1SEL_IC15       (3 << RCC_CCIPR4_I3C1SEL_SHIFT)
#define RCC_CCIPR4_I3C1SEL_MSI        (4 << RCC_CCIPR4_I3C1SEL_SHIFT)
#define RCC_CCIPR4_I3C1SEL_HSI        (5 << RCC_CCIPR4_I3C1SEL_SHIFT)
#define RCC_CCIPR4_I3C2SEL_SHIFT      (20)  /* Bits 20-22: Source selection
                                             * for the I3C2 kernel clock */
#define RCC_CCIPR4_I3C2SEL_MASK       (0x7 << RCC_CCIPR4_I3C2SEL_SHIFT)
#define RCC_CCIPR4_I3C2SEL_PCLK1      (0 << RCC_CCIPR4_I3C2SEL_SHIFT)
#define RCC_CCIPR4_I3C2SEL_CLKP       (1 << RCC_CCIPR4_I3C2SEL_SHIFT)
#define RCC_CCIPR4_I3C2SEL_IC10       (2 << RCC_CCIPR4_I3C2SEL_SHIFT)
#define RCC_CCIPR4_I3C2SEL_IC15       (3 << RCC_CCIPR4_I3C2SEL_SHIFT)
#define RCC_CCIPR4_I3C2SEL_MSI        (4 << RCC_CCIPR4_I3C2SEL_SHIFT)
#define RCC_CCIPR4_I3C2SEL_HSI        (5 << RCC_CCIPR4_I3C2SEL_SHIFT)
#define RCC_CCIPR4_LTDCSEL_SHIFT      (24)  /* Bits 24-25: Source selection
                                             * for the LTDC kernel clock */
#define RCC_CCIPR4_LTDCSEL_MASK       (0x3 << RCC_CCIPR4_LTDCSEL_SHIFT)
#define RCC_CCIPR4_LTDCSEL_PCLK5      (0 << RCC_CCIPR4_LTDCSEL_SHIFT)
#define RCC_CCIPR4_LTDCSEL_CLKP       (1 << RCC_CCIPR4_LTDCSEL_SHIFT)
#define RCC_CCIPR4_LTDCSEL_IC16       (2 << RCC_CCIPR4_LTDCSEL_SHIFT)
#define RCC_CCIPR4_LTDCSEL_HSI        (3 << RCC_CCIPR4_LTDCSEL_SHIFT)

/* Clock configuration for independent peripheral register 5 (CCIPR5) */

#define RCC_CCIPR5_MCO1SEL_SHIFT      (0)  /* Bits 0-2: Source selection
                                            * for the MCO1 kernel clock */
#define RCC_CCIPR5_MCO1SEL_MASK       (0x7 << RCC_CCIPR5_MCO1SEL_SHIFT)
#define RCC_CCIPR5_MCO1PRE_SHIFT      (4)  /* Bits 4-7: MCO1 Kernel clock
                                            * divider selection (for clock
                                            * MCO1) */
#define RCC_CCIPR5_MCO1PRE_MASK       (0xf << RCC_CCIPR5_MCO1PRE_SHIFT)
#define RCC_CCIPR5_MCO2SEL_SHIFT      (8)  /* Bits 8-10: Source selection
                                            * for the MCO2 kernel clock */
#define RCC_CCIPR5_MCO2SEL_MASK       (0x7 << RCC_CCIPR5_MCO2SEL_SHIFT)
#define RCC_CCIPR5_MCO2PRE_SHIFT      (12)  /* Bits 12-15: MCO2 Kernel
                                             * clock divider selection (for
                                             * clock MCO2) */
#define RCC_CCIPR5_MCO2PRE_MASK       (0xf << RCC_CCIPR5_MCO2PRE_SHIFT)
#define RCC_CCIPR5_MDF1SEL_SHIFT      (16)  /* Bits 16-18: Source selection
                                             * for the MDF1 kernel clock */
#define RCC_CCIPR5_MDF1SEL_MASK       (0x7 << RCC_CCIPR5_MDF1SEL_SHIFT)
#define RCC_CCIPR5_MDF1SEL_HCLK       (0 << RCC_CCIPR5_MDF1SEL_SHIFT)
#define RCC_CCIPR5_MDF1SEL_CLKP       (1 << RCC_CCIPR5_MDF1SEL_SHIFT)
#define RCC_CCIPR5_MDF1SEL_IC7        (2 << RCC_CCIPR5_MDF1SEL_SHIFT)
#define RCC_CCIPR5_MDF1SEL_IC8        (3 << RCC_CCIPR5_MDF1SEL_SHIFT)
#define RCC_CCIPR5_MDF1SEL_MSI        (4 << RCC_CCIPR5_MDF1SEL_SHIFT)
#define RCC_CCIPR5_MDF1SEL_HSI        (5 << RCC_CCIPR5_MDF1SEL_SHIFT)
#define RCC_CCIPR5_MDF1SEL_I2S_CKIN   (6 << RCC_CCIPR5_MDF1SEL_SHIFT)
#define RCC_CCIPR5_MDF1SEL_TIMG       (7 << RCC_CCIPR5_MDF1SEL_SHIFT)

/* Clock configuration for independent peripheral register 6 (CCIPR6) */

#define RCC_CCIPR6_XSPI1SEL_SHIFT     (0)  /* Bits 0-1: Source selection
                                            * for the XSPI1 kernel clock */
#define RCC_CCIPR6_XSPI1SEL_MASK      (0x3 << RCC_CCIPR6_XSPI1SEL_SHIFT)
#define RCC_CCIPR6_XSPI1SEL_HCLK      (0 << RCC_CCIPR6_XSPI1SEL_SHIFT)
#define RCC_CCIPR6_XSPI1SEL_CLKP      (1 << RCC_CCIPR6_XSPI1SEL_SHIFT)
#define RCC_CCIPR6_XSPI1SEL_IC3       (2 << RCC_CCIPR6_XSPI1SEL_SHIFT)
#define RCC_CCIPR6_XSPI1SEL_IC4       (3 << RCC_CCIPR6_XSPI1SEL_SHIFT)
#define RCC_CCIPR6_XSPI2SEL_SHIFT     (4)  /* Bits 4-5: Source selection
                                            * for the XSPI2 kernel clock */
#define RCC_CCIPR6_XSPI2SEL_MASK      (0x3 << RCC_CCIPR6_XSPI2SEL_SHIFT)
#define RCC_CCIPR6_XSPI2SEL_HCLK      (0 << RCC_CCIPR6_XSPI2SEL_SHIFT)
#define RCC_CCIPR6_XSPI2SEL_CLKP      (1 << RCC_CCIPR6_XSPI2SEL_SHIFT)
#define RCC_CCIPR6_XSPI2SEL_IC3       (2 << RCC_CCIPR6_XSPI2SEL_SHIFT)
#define RCC_CCIPR6_XSPI2SEL_IC4       (3 << RCC_CCIPR6_XSPI2SEL_SHIFT)
#define RCC_CCIPR6_XSPI3SEL_SHIFT     (8)  /* Bits 8-9: Source selection
                                            * for the XSPI3 kernel clock */
#define RCC_CCIPR6_XSPI3SEL_MASK      (0x3 << RCC_CCIPR6_XSPI3SEL_SHIFT)
#define RCC_CCIPR6_XSPI3SEL_HCLK      (0 << RCC_CCIPR6_XSPI3SEL_SHIFT)
#define RCC_CCIPR6_XSPI3SEL_CLKP      (1 << RCC_CCIPR6_XSPI3SEL_SHIFT)
#define RCC_CCIPR6_XSPI3SEL_IC3       (2 << RCC_CCIPR6_XSPI3SEL_SHIFT)
#define RCC_CCIPR6_XSPI3SEL_IC4       (3 << RCC_CCIPR6_XSPI3SEL_SHIFT)
#define RCC_CCIPR6_OTGPHY1SEL_SHIFT   (12)  /* Bits 12-13: Source selection
                                             * for the OTGPHY1 kernel clock */
#define RCC_CCIPR6_OTGPHY1SEL_MASK    (0x3 << RCC_CCIPR6_OTGPHY1SEL_SHIFT)
#define RCC_CCIPR6_OTGPHY1SEL_HSE_DIV_2 (0 << RCC_CCIPR6_OTGPHY1SEL_SHIFT)
#define RCC_CCIPR6_OTGPHY1SEL_CLKP    (1 << RCC_CCIPR6_OTGPHY1SEL_SHIFT)
#define RCC_CCIPR6_OTGPHY1SEL_IC15    (2 << RCC_CCIPR6_OTGPHY1SEL_SHIFT)
#define RCC_CCIPR6_OTGPHY1SEL_HSE_DIV_2_OSC (3 << RCC_CCIPR6_OTGPHY1SEL_SHIFT)
#define RCC_CCIPR6_OTGPHY1CKREFSEL    (1 << 16)  /* Bit 16: Set and reset
                                                  * by software */
#define RCC_CCIPR6_OTGPHY2SEL_SHIFT   (20)       /* Bits 20-21: Source
                                                  * selection for the
                                                  * OTGPHY2 kernel clock */
#define RCC_CCIPR6_OTGPHY2SEL_MASK    (0x3 << RCC_CCIPR6_OTGPHY2SEL_SHIFT)
#define RCC_CCIPR6_OTGPHY2SEL_HSE_DIV_2 (0 << RCC_CCIPR6_OTGPHY2SEL_SHIFT)
#define RCC_CCIPR6_OTGPHY2SEL_CLKP    (1 << RCC_CCIPR6_OTGPHY2SEL_SHIFT)
#define RCC_CCIPR6_OTGPHY2SEL_IC15    (2 << RCC_CCIPR6_OTGPHY2SEL_SHIFT)
#define RCC_CCIPR6_OTGPHY2SEL_HSE_DIV_2_OSC (3 << RCC_CCIPR6_OTGPHY2SEL_SHIFT)
#define RCC_CCIPR6_OTGPHY2CKREFSEL    (1 << 24)  /* Bit 24: Set and reset
                                                  * by software */

/* Clock configuration for independent peripheral register 7 (CCIPR7) */

#define RCC_CCIPR7_PERSEL_SHIFT       (0)  /* Bits 0-2: Source selection
                                            * for the PER kernel clock */
#define RCC_CCIPR7_PERSEL_MASK        (0x7 << RCC_CCIPR7_PERSEL_SHIFT)
#define RCC_CCIPR7_PERSEL_HSI         (0 << RCC_CCIPR7_PERSEL_SHIFT)
#define RCC_CCIPR7_PERSEL_MSI         (1 << RCC_CCIPR7_PERSEL_SHIFT)
#define RCC_CCIPR7_PERSEL_HSE         (2 << RCC_CCIPR7_PERSEL_SHIFT)
#define RCC_CCIPR7_PERSEL_IC19        (3 << RCC_CCIPR7_PERSEL_SHIFT)
#define RCC_CCIPR7_PERSEL_IC5         (4 << RCC_CCIPR7_PERSEL_SHIFT)
#define RCC_CCIPR7_PERSEL_IC10        (5 << RCC_CCIPR7_PERSEL_SHIFT)
#define RCC_CCIPR7_PERSEL_IC15        (6 << RCC_CCIPR7_PERSEL_SHIFT)
#define RCC_CCIPR7_PERSEL_IC20        (7 << RCC_CCIPR7_PERSEL_SHIFT)
#define RCC_CCIPR7_PSSISEL_SHIFT      (4)  /* Bits 4-5: Source selection
                                            * for the PSSI kernel clock */
#define RCC_CCIPR7_PSSISEL_MASK       (0x3 << RCC_CCIPR7_PSSISEL_SHIFT)
#define RCC_CCIPR7_PSSISEL_HCLK       (0 << RCC_CCIPR7_PSSISEL_SHIFT)
#define RCC_CCIPR7_PSSISEL_CLKP       (1 << RCC_CCIPR7_PSSISEL_SHIFT)
#define RCC_CCIPR7_PSSISEL_IC20       (2 << RCC_CCIPR7_PSSISEL_SHIFT)
#define RCC_CCIPR7_RTCSEL_SHIFT       (8)  /* Bits 8-9: Source selection
                                            * for the RTC kernel clock */
#define RCC_CCIPR7_RTCSEL_MASK        (0x3 << RCC_CCIPR7_RTCSEL_SHIFT)
#define RCC_CCIPR7_RTCSEL_NONE        (0 << RCC_CCIPR7_RTCSEL_SHIFT)
#define RCC_CCIPR7_RTCSEL_LSE         (1 << RCC_CCIPR7_RTCSEL_SHIFT)
#define RCC_CCIPR7_RTCSEL_LSI         (2 << RCC_CCIPR7_RTCSEL_SHIFT)
#define RCC_CCIPR7_RTCSEL_HSE         (3 << RCC_CCIPR7_RTCSEL_SHIFT)
#define RCC_CCIPR7_RTCPRE_SHIFT       (12)  /* Bits 12-17: RTC OSC clock
                                             * divider selection (for clock
                                             * hse_ck) */
#define RCC_CCIPR7_RTCPRE_MASK        (0x3f << RCC_CCIPR7_RTCPRE_SHIFT)
#define RCC_CCIPR7_SAI1SEL_SHIFT      (20)  /* Bits 20-22: Source selection
                                             * for the SAI1 kernel clock */
#define RCC_CCIPR7_SAI1SEL_MASK       (0x7 << RCC_CCIPR7_SAI1SEL_SHIFT)
#define RCC_CCIPR7_SAI1SEL_PCLK2      (0 << RCC_CCIPR7_SAI1SEL_SHIFT)
#define RCC_CCIPR7_SAI1SEL_CLKP       (1 << RCC_CCIPR7_SAI1SEL_SHIFT)
#define RCC_CCIPR7_SAI1SEL_IC7        (2 << RCC_CCIPR7_SAI1SEL_SHIFT)
#define RCC_CCIPR7_SAI1SEL_IC8        (3 << RCC_CCIPR7_SAI1SEL_SHIFT)
#define RCC_CCIPR7_SAI1SEL_MSI        (4 << RCC_CCIPR7_SAI1SEL_SHIFT)
#define RCC_CCIPR7_SAI1SEL_HSI        (5 << RCC_CCIPR7_SAI1SEL_SHIFT)
#define RCC_CCIPR7_SAI1SEL_I2S_CKIN   (6 << RCC_CCIPR7_SAI1SEL_SHIFT)
#define RCC_CCIPR7_SAI1SEL_SPDIFRX1   (7 << RCC_CCIPR7_SAI1SEL_SHIFT)
#define RCC_CCIPR7_SAI2SEL_SHIFT      (24)  /* Bits 24-26: Source selection
                                             * for the SAI2 kernel clock */
#define RCC_CCIPR7_SAI2SEL_MASK       (0x7 << RCC_CCIPR7_SAI2SEL_SHIFT)
#define RCC_CCIPR7_SAI2SEL_PCLK2      (0 << RCC_CCIPR7_SAI2SEL_SHIFT)
#define RCC_CCIPR7_SAI2SEL_CLKP       (1 << RCC_CCIPR7_SAI2SEL_SHIFT)
#define RCC_CCIPR7_SAI2SEL_IC7        (2 << RCC_CCIPR7_SAI2SEL_SHIFT)
#define RCC_CCIPR7_SAI2SEL_IC8        (3 << RCC_CCIPR7_SAI2SEL_SHIFT)
#define RCC_CCIPR7_SAI2SEL_MSI        (4 << RCC_CCIPR7_SAI2SEL_SHIFT)
#define RCC_CCIPR7_SAI2SEL_HSI        (5 << RCC_CCIPR7_SAI2SEL_SHIFT)
#define RCC_CCIPR7_SAI2SEL_I2S_CKIN   (6 << RCC_CCIPR7_SAI2SEL_SHIFT)
#define RCC_CCIPR7_SAI2SEL_SPDIFRX1   (7 << RCC_CCIPR7_SAI2SEL_SHIFT)

/* Clock configuration for independent peripheral register 8 (CCIPR8) */

#define RCC_CCIPR8_SDMMC1SEL_SHIFT    (0)  /* Bits 0-1: Source selection
                                            * for the SDMMC1 kernel clock */
#define RCC_CCIPR8_SDMMC1SEL_MASK     (0x3 << RCC_CCIPR8_SDMMC1SEL_SHIFT)
#define RCC_CCIPR8_SDMMC1SEL_HCLK     (0 << RCC_CCIPR8_SDMMC1SEL_SHIFT)
#define RCC_CCIPR8_SDMMC1SEL_CLKP     (1 << RCC_CCIPR8_SDMMC1SEL_SHIFT)
#define RCC_CCIPR8_SDMMC1SEL_IC4      (2 << RCC_CCIPR8_SDMMC1SEL_SHIFT)
#define RCC_CCIPR8_SDMMC1SEL_IC5      (3 << RCC_CCIPR8_SDMMC1SEL_SHIFT)
#define RCC_CCIPR8_SDMMC2SEL_SHIFT    (4)  /* Bits 4-5: Source selection
                                            * for the SDMMC2 kernel clock */
#define RCC_CCIPR8_SDMMC2SEL_MASK     (0x3 << RCC_CCIPR8_SDMMC2SEL_SHIFT)
#define RCC_CCIPR8_SDMMC2SEL_HCLK     (0 << RCC_CCIPR8_SDMMC2SEL_SHIFT)
#define RCC_CCIPR8_SDMMC2SEL_CLKP     (1 << RCC_CCIPR8_SDMMC2SEL_SHIFT)
#define RCC_CCIPR8_SDMMC2SEL_IC4      (2 << RCC_CCIPR8_SDMMC2SEL_SHIFT)
#define RCC_CCIPR8_SDMMC2SEL_IC5      (3 << RCC_CCIPR8_SDMMC2SEL_SHIFT)

/* Clock configuration for independent peripheral register 9 (CCIPR9) */

#define RCC_CCIPR9_SPDIFRX1SEL_SHIFT  (0)  /* Bits 0-2: Source selection
                                            * for the SPDIFRX1 kernel clock */
#define RCC_CCIPR9_SPDIFRX1SEL_MASK   (0x7 << RCC_CCIPR9_SPDIFRX1SEL_SHIFT)
#define RCC_CCIPR9_SPDIFRX1SEL_PCLK1  (0 << RCC_CCIPR9_SPDIFRX1SEL_SHIFT)
#define RCC_CCIPR9_SPDIFRX1SEL_CLKP   (1 << RCC_CCIPR9_SPDIFRX1SEL_SHIFT)
#define RCC_CCIPR9_SPDIFRX1SEL_IC7    (2 << RCC_CCIPR9_SPDIFRX1SEL_SHIFT)
#define RCC_CCIPR9_SPDIFRX1SEL_MSI    (4 << RCC_CCIPR9_SPDIFRX1SEL_SHIFT)
#define RCC_CCIPR9_SPI1SEL_SHIFT      (4)  /* Bits 4-6: Source selection
                                            * for the SPI1 kernel clock */
#define RCC_CCIPR9_SPI1SEL_MASK       (0x7 << RCC_CCIPR9_SPI1SEL_SHIFT)
#define RCC_CCIPR9_SPI1SEL_PCLK2      (0 << RCC_CCIPR9_SPI1SEL_SHIFT)
#define RCC_CCIPR9_SPI1SEL_CLKP       (1 << RCC_CCIPR9_SPI1SEL_SHIFT)
#define RCC_CCIPR9_SPI1SEL_IC8        (2 << RCC_CCIPR9_SPI1SEL_SHIFT)
#define RCC_CCIPR9_SPI1SEL_IC9        (3 << RCC_CCIPR9_SPI1SEL_SHIFT)
#define RCC_CCIPR9_SPI1SEL_MSI        (4 << RCC_CCIPR9_SPI1SEL_SHIFT)
#define RCC_CCIPR9_SPI1SEL_HSI        (5 << RCC_CCIPR9_SPI1SEL_SHIFT)
#define RCC_CCIPR9_SPI1SEL_I2S_CKIN   (6 << RCC_CCIPR9_SPI1SEL_SHIFT)
#define RCC_CCIPR9_SPI2SEL_SHIFT      (8)  /* Bits 8-10: Source selection
                                            * for the SPI2 kernel clock */
#define RCC_CCIPR9_SPI2SEL_MASK       (0x7 << RCC_CCIPR9_SPI2SEL_SHIFT)
#define RCC_CCIPR9_SPI2SEL_PCLK1      (0 << RCC_CCIPR9_SPI2SEL_SHIFT)
#define RCC_CCIPR9_SPI2SEL_CLKP       (1 << RCC_CCIPR9_SPI2SEL_SHIFT)
#define RCC_CCIPR9_SPI2SEL_IC8        (2 << RCC_CCIPR9_SPI2SEL_SHIFT)
#define RCC_CCIPR9_SPI2SEL_IC9        (3 << RCC_CCIPR9_SPI2SEL_SHIFT)
#define RCC_CCIPR9_SPI2SEL_MSI        (4 << RCC_CCIPR9_SPI2SEL_SHIFT)
#define RCC_CCIPR9_SPI2SEL_HSI        (5 << RCC_CCIPR9_SPI2SEL_SHIFT)
#define RCC_CCIPR9_SPI2SEL_I2S_CKIN   (6 << RCC_CCIPR9_SPI2SEL_SHIFT)
#define RCC_CCIPR9_SPI3SEL_SHIFT      (12)  /* Bits 12-14: Source selection
                                             * for the SPI3 kernel clock */
#define RCC_CCIPR9_SPI3SEL_MASK       (0x7 << RCC_CCIPR9_SPI3SEL_SHIFT)
#define RCC_CCIPR9_SPI3SEL_PCLK1      (0 << RCC_CCIPR9_SPI3SEL_SHIFT)
#define RCC_CCIPR9_SPI3SEL_CLKP       (1 << RCC_CCIPR9_SPI3SEL_SHIFT)
#define RCC_CCIPR9_SPI3SEL_IC8        (2 << RCC_CCIPR9_SPI3SEL_SHIFT)
#define RCC_CCIPR9_SPI3SEL_IC9        (3 << RCC_CCIPR9_SPI3SEL_SHIFT)
#define RCC_CCIPR9_SPI3SEL_MSI        (4 << RCC_CCIPR9_SPI3SEL_SHIFT)
#define RCC_CCIPR9_SPI3SEL_HSI        (5 << RCC_CCIPR9_SPI3SEL_SHIFT)
#define RCC_CCIPR9_SPI3SEL_I2S_CKIN   (6 << RCC_CCIPR9_SPI3SEL_SHIFT)
#define RCC_CCIPR9_SPI4SEL_SHIFT      (16)  /* Bits 16-18: Source selection
                                             * for the SPI4 kernel clock */
#define RCC_CCIPR9_SPI4SEL_MASK       (0x7 << RCC_CCIPR9_SPI4SEL_SHIFT)
#define RCC_CCIPR9_SPI4SEL_PCLK2      (0 << RCC_CCIPR9_SPI4SEL_SHIFT)
#define RCC_CCIPR9_SPI4SEL_CLKP       (1 << RCC_CCIPR9_SPI4SEL_SHIFT)
#define RCC_CCIPR9_SPI4SEL_IC9        (2 << RCC_CCIPR9_SPI4SEL_SHIFT)
#define RCC_CCIPR9_SPI4SEL_IC14       (3 << RCC_CCIPR9_SPI4SEL_SHIFT)
#define RCC_CCIPR9_SPI4SEL_MSI        (4 << RCC_CCIPR9_SPI4SEL_SHIFT)
#define RCC_CCIPR9_SPI4SEL_HSI        (5 << RCC_CCIPR9_SPI4SEL_SHIFT)
#define RCC_CCIPR9_SPI4SEL_HSE        (6 << RCC_CCIPR9_SPI4SEL_SHIFT)
#define RCC_CCIPR9_SPI5SEL_SHIFT      (20)  /* Bits 20-22: Source selection
                                             * for the SPI5 kernel clock */
#define RCC_CCIPR9_SPI5SEL_MASK       (0x7 << RCC_CCIPR9_SPI5SEL_SHIFT)
#define RCC_CCIPR9_SPI5SEL_PCLK2      (0 << RCC_CCIPR9_SPI5SEL_SHIFT)
#define RCC_CCIPR9_SPI5SEL_CLKP       (1 << RCC_CCIPR9_SPI5SEL_SHIFT)
#define RCC_CCIPR9_SPI5SEL_IC9        (2 << RCC_CCIPR9_SPI5SEL_SHIFT)
#define RCC_CCIPR9_SPI5SEL_IC14       (3 << RCC_CCIPR9_SPI5SEL_SHIFT)
#define RCC_CCIPR9_SPI5SEL_MSI        (4 << RCC_CCIPR9_SPI5SEL_SHIFT)
#define RCC_CCIPR9_SPI5SEL_HSI        (5 << RCC_CCIPR9_SPI5SEL_SHIFT)
#define RCC_CCIPR9_SPI5SEL_HSE        (6 << RCC_CCIPR9_SPI5SEL_SHIFT)
#define RCC_CCIPR9_SPI6SEL_SHIFT      (24)  /* Bits 24-26: Source selection
                                             * for the SPI6 kernel clock */
#define RCC_CCIPR9_SPI6SEL_MASK       (0x7 << RCC_CCIPR9_SPI6SEL_SHIFT)
#define RCC_CCIPR9_SPI6SEL_PCLK4      (0 << RCC_CCIPR9_SPI6SEL_SHIFT)
#define RCC_CCIPR9_SPI6SEL_CLKP       (1 << RCC_CCIPR9_SPI6SEL_SHIFT)
#define RCC_CCIPR9_SPI6SEL_IC8        (2 << RCC_CCIPR9_SPI6SEL_SHIFT)
#define RCC_CCIPR9_SPI6SEL_IC9        (3 << RCC_CCIPR9_SPI6SEL_SHIFT)
#define RCC_CCIPR9_SPI6SEL_MSI        (4 << RCC_CCIPR9_SPI6SEL_SHIFT)
#define RCC_CCIPR9_SPI6SEL_HSI        (5 << RCC_CCIPR9_SPI6SEL_SHIFT)
#define RCC_CCIPR9_SPI6SEL_I2S_CKIN   (6 << RCC_CCIPR9_SPI6SEL_SHIFT)

/* Clock configuration for independent peripheral register 12 (CCIPR12) */

#define RCC_CCIPR12_LPTIM1SEL_SHIFT   (8)  /* Bits 8-10: Source selection
                                            * for the LPTIM1 kernel clock */
#define RCC_CCIPR12_LPTIM1SEL_MASK    (0x7 << RCC_CCIPR12_LPTIM1SEL_SHIFT)
#define RCC_CCIPR12_LPTIM1SEL_PCLK1   (0 << RCC_CCIPR12_LPTIM1SEL_SHIFT)
#define RCC_CCIPR12_LPTIM1SEL_CLKP    (1 << RCC_CCIPR12_LPTIM1SEL_SHIFT)
#define RCC_CCIPR12_LPTIM1SEL_IC15    (2 << RCC_CCIPR12_LPTIM1SEL_SHIFT)
#define RCC_CCIPR12_LPTIM1SEL_LSE     (3 << RCC_CCIPR12_LPTIM1SEL_SHIFT)
#define RCC_CCIPR12_LPTIM1SEL_LSI     (4 << RCC_CCIPR12_LPTIM1SEL_SHIFT)
#define RCC_CCIPR12_LPTIM1SEL_TIMG    (5 << RCC_CCIPR12_LPTIM1SEL_SHIFT)
#define RCC_CCIPR12_LPTIM2SEL_SHIFT   (12)  /* Bits 12-14: Source selection
                                             * for the LPTIM2 kernel clock */
#define RCC_CCIPR12_LPTIM2SEL_MASK    (0x7 << RCC_CCIPR12_LPTIM2SEL_SHIFT)
#define RCC_CCIPR12_LPTIM2SEL_PCLK4   (0 << RCC_CCIPR12_LPTIM2SEL_SHIFT)
#define RCC_CCIPR12_LPTIM2SEL_CLKP    (1 << RCC_CCIPR12_LPTIM2SEL_SHIFT)
#define RCC_CCIPR12_LPTIM2SEL_IC15    (2 << RCC_CCIPR12_LPTIM2SEL_SHIFT)
#define RCC_CCIPR12_LPTIM2SEL_LSE     (3 << RCC_CCIPR12_LPTIM2SEL_SHIFT)
#define RCC_CCIPR12_LPTIM2SEL_LSI     (4 << RCC_CCIPR12_LPTIM2SEL_SHIFT)
#define RCC_CCIPR12_LPTIM2SEL_TIMG    (5 << RCC_CCIPR12_LPTIM2SEL_SHIFT)
#define RCC_CCIPR12_LPTIM3SEL_SHIFT   (16)  /* Bits 16-18: Source selection
                                             * for the LPTIM3 kernel clock */
#define RCC_CCIPR12_LPTIM3SEL_MASK    (0x7 << RCC_CCIPR12_LPTIM3SEL_SHIFT)
#define RCC_CCIPR12_LPTIM3SEL_PCLK4   (0 << RCC_CCIPR12_LPTIM3SEL_SHIFT)
#define RCC_CCIPR12_LPTIM3SEL_CLKP    (1 << RCC_CCIPR12_LPTIM3SEL_SHIFT)
#define RCC_CCIPR12_LPTIM3SEL_IC15    (2 << RCC_CCIPR12_LPTIM3SEL_SHIFT)
#define RCC_CCIPR12_LPTIM3SEL_LSE     (3 << RCC_CCIPR12_LPTIM3SEL_SHIFT)
#define RCC_CCIPR12_LPTIM3SEL_LSI     (4 << RCC_CCIPR12_LPTIM3SEL_SHIFT)
#define RCC_CCIPR12_LPTIM3SEL_TIMG    (5 << RCC_CCIPR12_LPTIM3SEL_SHIFT)
#define RCC_CCIPR12_LPTIM4SEL_SHIFT   (20)  /* Bits 20-22: Source selection
                                             * for the LPTIM4 kernel clock */
#define RCC_CCIPR12_LPTIM4SEL_MASK    (0x7 << RCC_CCIPR12_LPTIM4SEL_SHIFT)
#define RCC_CCIPR12_LPTIM4SEL_PCLK4   (0 << RCC_CCIPR12_LPTIM4SEL_SHIFT)
#define RCC_CCIPR12_LPTIM4SEL_CLKP    (1 << RCC_CCIPR12_LPTIM4SEL_SHIFT)
#define RCC_CCIPR12_LPTIM4SEL_IC15    (2 << RCC_CCIPR12_LPTIM4SEL_SHIFT)
#define RCC_CCIPR12_LPTIM4SEL_LSE     (3 << RCC_CCIPR12_LPTIM4SEL_SHIFT)
#define RCC_CCIPR12_LPTIM4SEL_LSI     (4 << RCC_CCIPR12_LPTIM4SEL_SHIFT)
#define RCC_CCIPR12_LPTIM4SEL_TIMG    (5 << RCC_CCIPR12_LPTIM4SEL_SHIFT)
#define RCC_CCIPR12_LPTIM5SEL_SHIFT   (24)  /* Bits 24-26: Source selection
                                             * for the LPTIM5 kernel clock */
#define RCC_CCIPR12_LPTIM5SEL_MASK    (0x7 << RCC_CCIPR12_LPTIM5SEL_SHIFT)
#define RCC_CCIPR12_LPTIM5SEL_PCLK4   (0 << RCC_CCIPR12_LPTIM5SEL_SHIFT)
#define RCC_CCIPR12_LPTIM5SEL_CLKP    (1 << RCC_CCIPR12_LPTIM5SEL_SHIFT)
#define RCC_CCIPR12_LPTIM5SEL_IC15    (2 << RCC_CCIPR12_LPTIM5SEL_SHIFT)
#define RCC_CCIPR12_LPTIM5SEL_LSE     (3 << RCC_CCIPR12_LPTIM5SEL_SHIFT)
#define RCC_CCIPR12_LPTIM5SEL_LSI     (4 << RCC_CCIPR12_LPTIM5SEL_SHIFT)
#define RCC_CCIPR12_LPTIM5SEL_TIMG    (5 << RCC_CCIPR12_LPTIM5SEL_SHIFT)

/* Peripheral kernel clock select register 13 */

#define RCC_CCIPR13_USART1SEL_SHIFT   (0)  /* Bits 0-2: Source selection
                                            * for the USART1 kernel clock */
#define RCC_CCIPR13_USART1SEL_MASK    (0x7 << RCC_CCIPR13_USART1SEL_SHIFT)
#define RCC_CCIPR13_USART1SEL_PCLK2   (0 << RCC_CCIPR13_USART1SEL_SHIFT)
#define RCC_CCIPR13_USART1SEL_CLKP    (1 << RCC_CCIPR13_USART1SEL_SHIFT)
#define RCC_CCIPR13_USART1SEL_IC9     (2 << RCC_CCIPR13_USART1SEL_SHIFT)
#define RCC_CCIPR13_USART1SEL_IC14    (3 << RCC_CCIPR13_USART1SEL_SHIFT)
#define RCC_CCIPR13_USART1SEL_LSE     (4 << RCC_CCIPR13_USART1SEL_SHIFT)
#define RCC_CCIPR13_USART1SEL_MSI     (5 << RCC_CCIPR13_USART1SEL_SHIFT)
#define RCC_CCIPR13_USART1SEL_HSI     (6 << RCC_CCIPR13_USART1SEL_SHIFT)
#define RCC_CCIPR13_USART2SEL_SHIFT   (4)  /* Bits 4-6: Source selection
                                            * for the USART2 kernel clock */
#define RCC_CCIPR13_USART2SEL_MASK    (0x7 << RCC_CCIPR13_USART2SEL_SHIFT)
#define RCC_CCIPR13_USART2SEL_PCLK1   (0 << RCC_CCIPR13_USART2SEL_SHIFT)
#define RCC_CCIPR13_USART2SEL_CLKP    (1 << RCC_CCIPR13_USART2SEL_SHIFT)
#define RCC_CCIPR13_USART2SEL_IC9     (2 << RCC_CCIPR13_USART2SEL_SHIFT)
#define RCC_CCIPR13_USART2SEL_IC14    (3 << RCC_CCIPR13_USART2SEL_SHIFT)
#define RCC_CCIPR13_USART2SEL_LSE     (4 << RCC_CCIPR13_USART2SEL_SHIFT)
#define RCC_CCIPR13_USART2SEL_MSI     (5 << RCC_CCIPR13_USART2SEL_SHIFT)
#define RCC_CCIPR13_USART2SEL_HSI     (6 << RCC_CCIPR13_USART2SEL_SHIFT)
#define RCC_CCIPR13_USART3SEL_SHIFT   (8)  /* Bits 8-10: Source selection
                                            * for the USART3 kernel clock */
#define RCC_CCIPR13_USART3SEL_MASK    (0x7 << RCC_CCIPR13_USART3SEL_SHIFT)
#define RCC_CCIPR13_USART3SEL_PCLK1   (0 << RCC_CCIPR13_USART3SEL_SHIFT)
#define RCC_CCIPR13_USART3SEL_CLKP    (1 << RCC_CCIPR13_USART3SEL_SHIFT)
#define RCC_CCIPR13_USART3SEL_IC9     (2 << RCC_CCIPR13_USART3SEL_SHIFT)
#define RCC_CCIPR13_USART3SEL_IC14    (3 << RCC_CCIPR13_USART3SEL_SHIFT)
#define RCC_CCIPR13_USART3SEL_LSE     (4 << RCC_CCIPR13_USART3SEL_SHIFT)
#define RCC_CCIPR13_USART3SEL_MSI     (5 << RCC_CCIPR13_USART3SEL_SHIFT)
#define RCC_CCIPR13_USART3SEL_HSI     (6 << RCC_CCIPR13_USART3SEL_SHIFT)
#define RCC_CCIPR13_UART4SEL_SHIFT    (12)  /* Bits 12-14: Source selection
                                             * for the UART4 kernel clock */
#define RCC_CCIPR13_UART4SEL_MASK     (0x7 << RCC_CCIPR13_UART4SEL_SHIFT)
#define RCC_CCIPR13_UART4SEL_PCLK1    (0 << RCC_CCIPR13_UART4SEL_SHIFT)
#define RCC_CCIPR13_UART4SEL_CLKP     (1 << RCC_CCIPR13_UART4SEL_SHIFT)
#define RCC_CCIPR13_UART4SEL_IC9      (2 << RCC_CCIPR13_UART4SEL_SHIFT)
#define RCC_CCIPR13_UART4SEL_IC14     (3 << RCC_CCIPR13_UART4SEL_SHIFT)
#define RCC_CCIPR13_UART4SEL_LSE      (4 << RCC_CCIPR13_UART4SEL_SHIFT)
#define RCC_CCIPR13_UART4SEL_MSI      (5 << RCC_CCIPR13_UART4SEL_SHIFT)
#define RCC_CCIPR13_UART4SEL_HSI      (6 << RCC_CCIPR13_UART4SEL_SHIFT)
#define RCC_CCIPR13_UART5SEL_SHIFT    (16)  /* Bits 16-18: Source selection
                                             * for the UART5 kernel clock */
#define RCC_CCIPR13_UART5SEL_MASK     (0x7 << RCC_CCIPR13_UART5SEL_SHIFT)
#define RCC_CCIPR13_UART5SEL_PCLK1    (0 << RCC_CCIPR13_UART5SEL_SHIFT)
#define RCC_CCIPR13_UART5SEL_CLKP     (1 << RCC_CCIPR13_UART5SEL_SHIFT)
#define RCC_CCIPR13_UART5SEL_IC9      (2 << RCC_CCIPR13_UART5SEL_SHIFT)
#define RCC_CCIPR13_UART5SEL_IC14     (3 << RCC_CCIPR13_UART5SEL_SHIFT)
#define RCC_CCIPR13_UART5SEL_LSE      (4 << RCC_CCIPR13_UART5SEL_SHIFT)
#define RCC_CCIPR13_UART5SEL_MSI      (5 << RCC_CCIPR13_UART5SEL_SHIFT)
#define RCC_CCIPR13_UART5SEL_HSI      (6 << RCC_CCIPR13_UART5SEL_SHIFT)
#define RCC_CCIPR13_USART6SEL_SHIFT   (20)  /* Bits 20-22: Source selection
                                             * for the USART6 kernel clock */
#define RCC_CCIPR13_USART6SEL_MASK    (0x7 << RCC_CCIPR13_USART6SEL_SHIFT)
#define RCC_CCIPR13_USART6SEL_PCLK2   (0 << RCC_CCIPR13_USART6SEL_SHIFT)
#define RCC_CCIPR13_USART6SEL_CLKP    (1 << RCC_CCIPR13_USART6SEL_SHIFT)
#define RCC_CCIPR13_USART6SEL_IC9     (2 << RCC_CCIPR13_USART6SEL_SHIFT)
#define RCC_CCIPR13_USART6SEL_IC14    (3 << RCC_CCIPR13_USART6SEL_SHIFT)
#define RCC_CCIPR13_USART6SEL_LSE     (4 << RCC_CCIPR13_USART6SEL_SHIFT)
#define RCC_CCIPR13_USART6SEL_MSI     (5 << RCC_CCIPR13_USART6SEL_SHIFT)
#define RCC_CCIPR13_USART6SEL_HSI     (6 << RCC_CCIPR13_USART6SEL_SHIFT)
#define RCC_CCIPR13_UART7SEL_SHIFT    (24)  /* Bits 24-26: Source selection
                                             * for the UART7 kernel clock */
#define RCC_CCIPR13_UART7SEL_MASK     (0x7 << RCC_CCIPR13_UART7SEL_SHIFT)
#define RCC_CCIPR13_UART7SEL_PCLK1    (0 << RCC_CCIPR13_UART7SEL_SHIFT)
#define RCC_CCIPR13_UART7SEL_CLKP     (1 << RCC_CCIPR13_UART7SEL_SHIFT)
#define RCC_CCIPR13_UART7SEL_IC9      (2 << RCC_CCIPR13_UART7SEL_SHIFT)
#define RCC_CCIPR13_UART7SEL_IC14     (3 << RCC_CCIPR13_UART7SEL_SHIFT)
#define RCC_CCIPR13_UART7SEL_LSE      (4 << RCC_CCIPR13_UART7SEL_SHIFT)
#define RCC_CCIPR13_UART7SEL_MSI      (5 << RCC_CCIPR13_UART7SEL_SHIFT)
#define RCC_CCIPR13_UART7SEL_HSI      (6 << RCC_CCIPR13_UART7SEL_SHIFT)
#define RCC_CCIPR13_UART8SEL_SHIFT    (28)  /* Bits 28-30: Source selection
                                             * for the UART8 kernel clock */
#define RCC_CCIPR13_UART8SEL_MASK     (0x7 << RCC_CCIPR13_UART8SEL_SHIFT)
#define RCC_CCIPR13_UART8SEL_PCLK1    (0 << RCC_CCIPR13_UART8SEL_SHIFT)
#define RCC_CCIPR13_UART8SEL_CLKP     (1 << RCC_CCIPR13_UART8SEL_SHIFT)
#define RCC_CCIPR13_UART8SEL_IC9      (2 << RCC_CCIPR13_UART8SEL_SHIFT)
#define RCC_CCIPR13_UART8SEL_IC14     (3 << RCC_CCIPR13_UART8SEL_SHIFT)
#define RCC_CCIPR13_UART8SEL_LSE      (4 << RCC_CCIPR13_UART8SEL_SHIFT)
#define RCC_CCIPR13_UART8SEL_MSI      (5 << RCC_CCIPR13_UART8SEL_SHIFT)
#define RCC_CCIPR13_UART8SEL_HSI      (6 << RCC_CCIPR13_UART8SEL_SHIFT)

/* Clock configuration for independent peripheral register 14 (CCIPR14) */

#define RCC_CCIPR14_UART9SEL_SHIFT    (0)  /* Bits 0-2: Source selection
                                            * for the UART9 kernel clock */
#define RCC_CCIPR14_UART9SEL_MASK     (0x7 << RCC_CCIPR14_UART9SEL_SHIFT)
#define RCC_CCIPR14_UART9SEL_PCLK2    (0 << RCC_CCIPR14_UART9SEL_SHIFT)
#define RCC_CCIPR14_UART9SEL_CLKP     (1 << RCC_CCIPR14_UART9SEL_SHIFT)
#define RCC_CCIPR14_UART9SEL_IC9      (2 << RCC_CCIPR14_UART9SEL_SHIFT)
#define RCC_CCIPR14_UART9SEL_IC14     (3 << RCC_CCIPR14_UART9SEL_SHIFT)
#define RCC_CCIPR14_UART9SEL_LSE      (4 << RCC_CCIPR14_UART9SEL_SHIFT)
#define RCC_CCIPR14_UART9SEL_MSI      (5 << RCC_CCIPR14_UART9SEL_SHIFT)
#define RCC_CCIPR14_UART9SEL_HSI      (6 << RCC_CCIPR14_UART9SEL_SHIFT)
#define RCC_CCIPR14_USART10SEL_SHIFT  (4)  /* Bits 4-6: Source selection
                                            * for the USART10 kernel clock */
#define RCC_CCIPR14_USART10SEL_MASK   (0x7 << RCC_CCIPR14_USART10SEL_SHIFT)
#define RCC_CCIPR14_USART10SEL_PCLK2  (0 << RCC_CCIPR14_USART10SEL_SHIFT)
#define RCC_CCIPR14_USART10SEL_CLKP   (1 << RCC_CCIPR14_USART10SEL_SHIFT)
#define RCC_CCIPR14_USART10SEL_IC9    (2 << RCC_CCIPR14_USART10SEL_SHIFT)
#define RCC_CCIPR14_USART10SEL_IC14   (3 << RCC_CCIPR14_USART10SEL_SHIFT)
#define RCC_CCIPR14_USART10SEL_LSE    (4 << RCC_CCIPR14_USART10SEL_SHIFT)
#define RCC_CCIPR14_USART10SEL_MSI    (5 << RCC_CCIPR14_USART10SEL_SHIFT)
#define RCC_CCIPR14_USART10SEL_HSI    (6 << RCC_CCIPR14_USART10SEL_SHIFT)
#define RCC_CCIPR14_LPUART1SEL_SHIFT  (8)  /* Bits 8-10: Source selection
                                            * for the LPUART1 kernel clock */
#define RCC_CCIPR14_LPUART1SEL_MASK   (0x7 << RCC_CCIPR14_LPUART1SEL_SHIFT)
#define RCC_CCIPR14_LPUART1SEL_PCLK4  (0 << RCC_CCIPR14_LPUART1SEL_SHIFT)
#define RCC_CCIPR14_LPUART1SEL_CLKP   (1 << RCC_CCIPR14_LPUART1SEL_SHIFT)
#define RCC_CCIPR14_LPUART1SEL_IC9    (2 << RCC_CCIPR14_LPUART1SEL_SHIFT)
#define RCC_CCIPR14_LPUART1SEL_IC14   (3 << RCC_CCIPR14_LPUART1SEL_SHIFT)
#define RCC_CCIPR14_LPUART1SEL_LSE    (4 << RCC_CCIPR14_LPUART1SEL_SHIFT)
#define RCC_CCIPR14_LPUART1SEL_MSI    (5 << RCC_CCIPR14_LPUART1SEL_SHIFT)
#define RCC_CCIPR14_LPUART1SEL_HSI    (6 << RCC_CCIPR14_LPUART1SEL_SHIFT)

/* Miscellaneous configurations reset register (MISCRSTR) */

#define RCC_MISCRSTR_DBGRST           (1 << 0)  /* Bit 0: DBG reset */
#define RCC_MISCRSTR_XSPIPHY1RST      (1 << 4)  /* Bit 4: XSPIPHY1 reset */
#define RCC_MISCRSTR_XSPIPHY2RST      (1 << 5)  /* Bit 5: XSPIPHY2 reset */
#define RCC_MISCRSTR_SDMMC1DLLRST     (1 << 7)  /* Bit 7: SDMMC1DLL reset */
#define RCC_MISCRSTR_SDMMC2DLLRST     (1 << 8)  /* Bit 8: SDMMC2DLL reset */

/* Embedded memories reset register (MEMRSTR) */

#define RCC_MEMRSTR_AXISRAM3RST       (1 << 0)   /* Bit 0: AXISRAM3 reset */
#define RCC_MEMRSTR_AXISRAM4RST       (1 << 1)   /* Bit 1: AXISRAM4 reset */
#define RCC_MEMRSTR_AXISRAM5RST       (1 << 2)   /* Bit 2: AXISRAM5 reset */
#define RCC_MEMRSTR_AXISRAM6RST       (1 << 3)   /* Bit 3: AXISRAM6 reset */
#define RCC_MEMRSTR_AHBSRAM1RST       (1 << 4)   /* Bit 4: AHBSRAM1 reset */
#define RCC_MEMRSTR_AHBSRAM2RST       (1 << 5)   /* Bit 5: AHBSRAM2 reset */
#define RCC_MEMRSTR_AXISRAM1RST       (1 << 7)   /* Bit 7: AXISRAM1 reset */
#define RCC_MEMRSTR_AXISRAM2RST       (1 << 8)   /* Bit 8: AXISRAM2 reset */
#define RCC_MEMRSTR_FLEXRAMRST        (1 << 9)   /* Bit 9: FLEXRAM reset */
#define RCC_MEMRSTR_CACHEAXIRAMRST    (1 << 10)  /* Bit 10: CACHEAXIRAM
                                                  * reset */
#define RCC_MEMRSTR_VENCRAMRST        (1 << 11)  /* Bit 11: VENCRAM reset */
#define RCC_MEMRSTR_BOOTROMRST        (1 << 12)  /* Bit 12: Boot ROM reset */

/* AHB1 reset register (AHB1RSTR) */

#define RCC_AHB1RSTR_GPDMA1RST        (1 << 4)  /* Bit 4: GPDMA1 reset */
#define RCC_AHB1RSTR_ADC12RST         (1 << 5)  /* Bit 5: ADC12 reset */

/* AHB2 reset register (AHB2RSTR) */

#define RCC_AHB2RSTR_RAMCFGRST        (1 << 12)  /* Bit 12: RAMCFG reset */
#define RCC_AHB2RSTR_MDF1RST          (1 << 16)  /* Bit 16: MDF1 reset */
#define RCC_AHB2RSTR_ADF1RST          (1 << 17)  /* Bit 17: ADF1 reset */

/* AHB3 reset register (AHB3RSTR) */

#define RCC_AHB3RSTR_RNGRST           (1 << 0)   /* Bit 0: RNG reset */
#define RCC_AHB3RSTR_HASHRST          (1 << 1)   /* Bit 1: HASH reset */
#define RCC_AHB3RSTR_CRYPRST          (1 << 2)   /* Bit 2: CRYP reset */
#define RCC_AHB3RSTR_SAESRST          (1 << 4)   /* Bit 4: SAES reset */
#define RCC_AHB3RSTR_PKARST           (1 << 8)   /* Bit 8: PKA reset */
#define RCC_AHB3RSTR_IACRST           (1 << 10)  /* Bit 10: IAC reset */

/* AHB4 reset register (AHB4RSTR) */

#define RCC_AHB4RSTR_GPIOARST         (1 << 0)   /* Bit 0: GPIO A reset */
#define RCC_AHB4RSTR_GPIOBRST         (1 << 1)   /* Bit 1: GPIO B reset */
#define RCC_AHB4RSTR_GPIOCRST         (1 << 2)   /* Bit 2: GPIO C reset */
#define RCC_AHB4RSTR_GPIODRST         (1 << 3)   /* Bit 3: GPIO D reset */
#define RCC_AHB4RSTR_GPIOERST         (1 << 4)   /* Bit 4: GPIO E reset */
#define RCC_AHB4RSTR_GPIOFRST         (1 << 5)   /* Bit 5: GPIO F reset */
#define RCC_AHB4RSTR_GPIOGRST         (1 << 6)   /* Bit 6: GPIO G reset */
#define RCC_AHB4RSTR_GPIOHRST         (1 << 7)   /* Bit 7: GPIO H reset */
#define RCC_AHB4RSTR_GPIONRST         (1 << 13)  /* Bit 13: GPIO N reset */
#define RCC_AHB4RSTR_GPIOORST         (1 << 14)  /* Bit 14: GPIO O reset */
#define RCC_AHB4RSTR_GPIOPRST         (1 << 15)  /* Bit 15: GPIO P reset */
#define RCC_AHB4RSTR_GPIOQRST         (1 << 16)  /* Bit 16: GPIO Q reset */
#define RCC_AHB4RSTR_PWRRST           (1 << 18)  /* Bit 18: PWR reset */
#define RCC_AHB4RSTR_CRCRST           (1 << 19)  /* Bit 19: CRC reset */

/* AHB5 reset register (AHB5RSTR) */

#define RCC_AHB5RSTR_HPDMA1RST        (1 << 0)   /* Bit 0: HPDMA1 reset */
#define RCC_AHB5RSTR_DMA2DRST         (1 << 1)   /* Bit 1: DMA2D reset */
#define RCC_AHB5RSTR_JPEGRST          (1 << 3)   /* Bit 3: JPEG reset */
#define RCC_AHB5RSTR_FMCRST           (1 << 4)   /* Bit 4: FMC reset */
#define RCC_AHB5RSTR_XSPI1RST         (1 << 5)   /* Bit 5: XSPI1 reset */
#define RCC_AHB5RSTR_PSSIRST          (1 << 6)   /* Bit 6: PSSI reset */
#define RCC_AHB5RSTR_SDMMC2RST        (1 << 7)   /* Bit 7: SDMMC2 reset */
#define RCC_AHB5RSTR_SDMMC1RST        (1 << 8)   /* Bit 8: SDMMC1 reset */
#define RCC_AHB5RSTR_XSPI2RST         (1 << 12)  /* Bit 12: XSPI2 reset */
#define RCC_AHB5RSTR_XSPIMRST         (1 << 13)  /* Bit 13: XSPIM reset */
#define RCC_AHB5RSTR_XSPI3RST         (1 << 17)  /* Bit 17: XSPI3 reset */
#define RCC_AHB5RSTR_GFXMMURST        (1 << 19)  /* Bit 19: GFXMMU reset */
#define RCC_AHB5RSTR_GPU2DRST         (1 << 20)  /* Bit 20: GPU2D reset */
#define RCC_AHB5RSTR_OTG1PHYCTLRST    (1 << 23)  /* Bit 23: OTG1PHYCTL
                                                  * reset */
#define RCC_AHB5RSTR_OTG2PHYCTLRST    (1 << 24)  /* Bit 24: OTG2PHYCTL
                                                  * reset */
#define RCC_AHB5RSTR_ETH1RST          (1 << 25)  /* Bit 25: ETH1 reset */
#define RCC_AHB5RSTR_OTG1RST          (1 << 26)  /* Bit 26: OTG1 reset */
#define RCC_AHB5RSTR_OTGPHY1RST       (1 << 27)  /* Bit 27: OTGPHY1 reset */
#define RCC_AHB5RSTR_OTGPHY2RST       (1 << 28)  /* Bit 28: OTGPHY2 reset */
#define RCC_AHB5RSTR_OTG2RST          (1 << 29)  /* Bit 29: OTG2 reset */
#define RCC_AHB5RSTR_CACHEAXIRST      (1 << 30)  /* Bit 30: CACHEAXI reset */
#define RCC_AHB5RSTR_NPURST           (1 << 31)  /* Bit 31: NPU reset */

/* APB1 peripheral reset register 1 */

#define RCC_APB1LRSTR_TIM2RST         (1 << 0)   /* Bit 0:  TIM2 reset */
#define RCC_APB1LRSTR_TIM3RST         (1 << 1)   /* Bit 1:  TIM3 reset */
#define RCC_APB1LRSTR_TIM4RST         (1 << 2)   /* Bit 2:  TIM4 reset */
#define RCC_APB1LRSTR_TIM5RST         (1 << 3)   /* Bit 3:  TIM5 reset */
#define RCC_APB1LRSTR_TIM6RST         (1 << 4)   /* Bit 4:  TIM6 reset */
#define RCC_APB1LRSTR_TIM7RST         (1 << 5)   /* Bit 5:  TIM7 reset */
#define RCC_APB1LRSTR_TIM12RST        (1 << 6)   /* Bit 6:  TIM12 reset */
#define RCC_APB1LRSTR_TIM13RST        (1 << 7)   /* Bit 7:  TIM13 reset */
#define RCC_APB1LRSTR_TIM14RST        (1 << 8)   /* Bit 8:  TIM14 reset */
#define RCC_APB1LRSTR_LPTIM1RST       (1 << 9)   /* Bit 9:  LPTIM1 reset */
#define RCC_APB1LRSTR_WWDGRST         (1 << 11)  /* Bit 11: WWDG reset */
#define RCC_APB1LRSTR_TIM10RST        (1 << 12)  /* Bit 12: TIM10 reset */
#define RCC_APB1LRSTR_TIM11RST        (1 << 13)  /* Bit 13: TIM11 reset */
#define RCC_APB1LRSTR_SPI2RST         (1 << 14)  /* Bit 14: SPI2 reset */
#define RCC_APB1LRSTR_SPI3RST         (1 << 15)  /* Bit 15: SPI3 reset */
#define RCC_APB1LRSTR_SPDIFRX1RST     (1 << 16)  /* Bit 16: SPDIFRX1 reset */
#define RCC_APB1LRSTR_USART2RST       (1 << 17)  /* Bit 17: USART2 reset */
#define RCC_APB1LRSTR_USART3RST       (1 << 18)  /* Bit 18: USART3 reset */
#define RCC_APB1LRSTR_UART4RST        (1 << 19)  /* Bit 19: UART4 reset */
#define RCC_APB1LRSTR_UART5RST        (1 << 20)  /* Bit 20: UART5 reset */
#define RCC_APB1LRSTR_I2C1RST         (1 << 21)  /* Bit 21: I2C1 reset */
#define RCC_APB1LRSTR_I2C2RST         (1 << 22)  /* Bit 22: I2C2 reset */
#define RCC_APB1LRSTR_I2C3RST         (1 << 23)  /* Bit 23: I2C3 reset */
#define RCC_APB1LRSTR_I3C1RST         (1 << 24)  /* Bit 24: I3C1 reset */
#define RCC_APB1LRSTR_I3C2RST         (1 << 25)  /* Bit 25: I3C2 reset */
#define RCC_APB1LRSTR_UART7RST        (1 << 30)  /* Bit 30: UART7 reset */
#define RCC_APB1LRSTR_UART8RST        (1 << 31)  /* Bit 31: UART8 reset */

/* APB1 reset register 2 (APB1HRSTR) */

#define RCC_APB1HRSTR_MDIOSRST        (1 << 5)   /* Bit 5: MDIOS reset */
#define RCC_APB1HRSTR_FDCANRST        (1 << 8)   /* Bit 8: FDCAN reset */
#define RCC_APB1HRSTR_UCPD1RST        (1 << 18)  /* Bit 18: UCPD1 reset */

/* APB2 peripheral reset register */

#define RCC_APB2RSTR_TIM1RST          (1 << 0)   /* Bit 0:  TIM1 reset */
#define RCC_APB2RSTR_TIM8RST          (1 << 1)   /* Bit 1:  TIM8 reset */
#define RCC_APB2RSTR_USART1RST        (1 << 4)   /* Bit 4: USART1 reset */
#define RCC_APB2RSTR_USART6RST        (1 << 5)   /* Bit 5: USART6 reset */
#define RCC_APB2RSTR_UART9RST         (1 << 6)   /* Bit 6: UART9 reset */
#define RCC_APB2RSTR_USART10RST       (1 << 7)   /* Bit 7: USART10 reset */
#define RCC_APB2RSTR_SPI1RST          (1 << 12)  /* Bit 12: SPI1 reset */
#define RCC_APB2RSTR_SPI4RST          (1 << 13)  /* Bit 13: SPI4 reset */
#define RCC_APB2RSTR_TIM18RST         (1 << 15)  /* Bit 15: TIM18 reset */
#define RCC_APB2RSTR_TIM15RST         (1 << 16)  /* Bit 16: TIM15 reset */
#define RCC_APB2RSTR_TIM16RST         (1 << 17)  /* Bit 17: TIM16 reset */
#define RCC_APB2RSTR_TIM17RST         (1 << 18)  /* Bit 18: TIM17 reset */
#define RCC_APB2RSTR_TIM9RST          (1 << 19)  /* Bit 19: TIM9 reset */
#define RCC_APB2RSTR_SPI5RST          (1 << 20)  /* Bit 20: SPI5 reset */
#define RCC_APB2RSTR_SAI1RST          (1 << 21)  /* Bit 21: SAI1 reset */
#define RCC_APB2RSTR_SAI2RST          (1 << 22)  /* Bit 22: SAI2 reset */

/* APB4 peripheral reset register 1 */

#define RCC_APB4LRSTR_HDPRST          (1 << 2)   /* Bit 2: HDP reset */
#define RCC_APB4LRSTR_LPUART1RST      (1 << 3)   /* Bit 3: LPUART1 reset */
#define RCC_APB4LRSTR_SPI6RST         (1 << 5)   /* Bit 5: SPI6 reset */
#define RCC_APB4LRSTR_I2C4RST         (1 << 7)   /* Bit 7: I2C4 reset */
#define RCC_APB4LRSTR_LPTIM2RST       (1 << 9)   /* Bit 9:  LPTIM2 reset */
#define RCC_APB4LRSTR_LPTIM3RST       (1 << 10)  /* Bit 10: LPTIM3 reset */
#define RCC_APB4LRSTR_LPTIM4RST       (1 << 11)  /* Bit 11: LPTIM4 reset */
#define RCC_APB4LRSTR_LPTIM5RST       (1 << 12)  /* Bit 12: LPTIM5 reset */
#define RCC_APB4LRSTR_VREFBUFRST      (1 << 15)  /* Bit 15: VREFBUF reset */
#define RCC_APB4LRSTR_RTCRST          (1 << 16)  /* Bit 16: RTC reset */

/* APB4 reset register 2 (APB4HRSTR) */

#define RCC_APB4HRSTR_SYSCFGRST       (1 << 0)  /* Bit 0: SYSCFG reset */
#define RCC_APB4HRSTR_DTSRST          (1 << 2)  /* Bit 2: DTS reset */

/* APB5 reset register (APB5RSTR) */

#define RCC_APB5RSTR_LTDCRST          (1 << 1)  /* Bit 1: LTDC reset */
#define RCC_APB5RSTR_DCMIPPRST        (1 << 2)  /* Bit 2: DCMIPP reset */
#define RCC_APB5RSTR_GFXTIMRST        (1 << 4)  /* Bit 4: GFXTIM reset */
#define RCC_APB5RSTR_VENCRST          (1 << 5)  /* Bit 5: VENC reset */
#define RCC_APB5RSTR_CSIRST           (1 << 6)  /* Bit 6: CSI reset */

/* IC divider enable register */

#define RCC_DIVENR_IC1EN              (1 << 0)   /* Bit 0:  IC1 enable */
#define RCC_DIVENR_IC2EN              (1 << 1)   /* Bit 1:  IC2 enable */
#define RCC_DIVENR_IC3EN              (1 << 2)   /* Bit 2:  IC3 enable */
#define RCC_DIVENR_IC4EN              (1 << 3)   /* Bit 3: IC4 enable */
#define RCC_DIVENR_IC5EN              (1 << 4)   /* Bit 4: IC5 enable */
#define RCC_DIVENR_IC6EN              (1 << 5)   /* Bit 5:  IC6 enable */
#define RCC_DIVENR_IC7EN              (1 << 6)   /* Bit 6: IC7 enable */
#define RCC_DIVENR_IC8EN              (1 << 7)   /* Bit 7: IC8 enable */
#define RCC_DIVENR_IC9EN              (1 << 8)   /* Bit 8: IC9 enable */
#define RCC_DIVENR_IC10EN             (1 << 9)   /* Bit 9: IC10 enable */
#define RCC_DIVENR_IC11EN             (1 << 10)  /* Bit 10: IC11 enable */
#define RCC_DIVENR_IC12EN             (1 << 11)  /* Bit 11: IC12 enable */
#define RCC_DIVENR_IC13EN             (1 << 12)  /* Bit 12: IC13 enable */
#define RCC_DIVENR_IC14EN             (1 << 13)  /* Bit 13: IC14 enable */
#define RCC_DIVENR_IC15EN             (1 << 14)  /* Bit 14: IC15 enable */
#define RCC_DIVENR_IC16EN             (1 << 15)  /* Bit 15: IC16 enable */
#define RCC_DIVENR_IC17EN             (1 << 16)  /* Bit 16: IC17 enable */
#define RCC_DIVENR_IC18EN             (1 << 17)  /* Bit 17: IC18 enable */
#define RCC_DIVENR_IC19EN             (1 << 18)  /* Bit 18: IC19 enable */
#define RCC_DIVENR_IC20EN             (1 << 19)  /* Bit 19: IC20 enable */

/* Embedded buses enable register (BUSENR) */

#define RCC_BUSENR_ACLKNEN            (1 << 0)   /* Bit 0: ACLKN enable */
#define RCC_BUSENR_ACLKNCEN           (1 << 1)   /* Bit 1: ACLKNC enable */
#define RCC_BUSENR_AHBMEN             (1 << 2)   /* Bit 2: AHBM enable */
#define RCC_BUSENR_AHB1EN             (1 << 3)   /* Bit 3: AHB1 enable */
#define RCC_BUSENR_AHB2EN             (1 << 4)   /* Bit 4: AHB2 enable */
#define RCC_BUSENR_AHB3EN             (1 << 5)   /* Bit 5: AHB3 enable */
#define RCC_BUSENR_AHB4EN             (1 << 6)   /* Bit 6: AHB4 enable */
#define RCC_BUSENR_AHB5EN             (1 << 7)   /* Bit 7: AHB5 enable */
#define RCC_BUSENR_APB1EN             (1 << 8)   /* Bit 8: APB1 enable */
#define RCC_BUSENR_APB2EN             (1 << 9)   /* Bit 9: APB2 enable */
#define RCC_BUSENR_APB3EN             (1 << 10)  /* Bit 10: APB3 enable */
#define RCC_BUSENR_APB4EN             (1 << 11)  /* Bit 11: APB4 enable */
#define RCC_BUSENR_APB5EN             (1 << 12)  /* Bit 12: APB5 enable */

/* Miscellaneous configurations enable register (MISCENR) */

#define RCC_MISCENR_DBGEN             (1 << 0)  /* Bit 0: DBG enable */
#define RCC_MISCENR_MCO1EN            (1 << 1)  /* Bit 1: MCO1 enable */
#define RCC_MISCENR_MCO2EN            (1 << 2)  /* Bit 2: MCO2 enable */
#define RCC_MISCENR_XSPIPHYCOMPEN     (1 << 3)  /* Bit 3: XSPIPHYCOMP
                                                 * enable */
#define RCC_MISCENR_PEREN             (1 << 6)  /* Bit 6: PER enable */

/* Peripheral clock enable / set / clear register offsets.  Each enable
 * register (xxxENR) has a paired set register (xxxENSR) that performs an
 * atomic OR and a clear register (xxxENCR) that performs an atomic AND
 * (Reference: RM0486 14.5).  The same triplet pattern applies to the LPENR
 * (sleep mode) registers.
 */

#define RCC_MEMENR_AXISRAM3EN         (1 << 0)   /* Bit 0:  AXISRAM3 enable */
#define RCC_MEMENR_AXISRAM4EN         (1 << 1)   /* Bit 1:  AXISRAM4 enable */
#define RCC_MEMENR_AXISRAM5EN         (1 << 2)   /* Bit 2:  AXISRAM5 enable */
#define RCC_MEMENR_AXISRAM6EN         (1 << 3)   /* Bit 3:  AXISRAM6 enable */
#define RCC_MEMENR_AHBSRAM1EN         (1 << 4)   /* Bit 4: AHBSRAM1 enable */
#define RCC_MEMENR_AHBSRAM2EN         (1 << 5)   /* Bit 5: AHBSRAM2 enable */
#define RCC_MEMENR_BKPSRAMEN          (1 << 6)   /* Bit 6: BKPSRAM enable */
#define RCC_MEMENR_AXISRAM1EN         (1 << 7)   /* Bit 7:  AXISRAM1 enable */
#define RCC_MEMENR_AXISRAM2EN         (1 << 8)   /* Bit 8:  AXISRAM2 enable */
#define RCC_MEMENR_FLEXRAMEN          (1 << 9)   /* Bit 9: FLEXRAM enable */
#define RCC_MEMENR_CACHEAXIRAMEN      (1 << 10)  /* Bit 10: CACHEAXIRAM
                                                  * enable */
#define RCC_MEMENR_VENCRAMEN          (1 << 11)  /* Bit 11: VENCRAM enable */
#define RCC_MEMENR_BOOTROMEN          (1 << 12)  /* Bit 12: Boot ROM enable */

#define RCC_MEMENR_ALLAXISRAM         (RCC_MEMENR_AXISRAM1EN | RCC_MEMENR_AXISRAM2EN | \
                                       RCC_MEMENR_AXISRAM3EN | RCC_MEMENR_AXISRAM4EN | \
                                       RCC_MEMENR_AXISRAM5EN | RCC_MEMENR_AXISRAM6EN)

/* AHB1 enable register (AHB1ENR) */

#define RCC_AHB1ENR_GPDMA1EN          (1 << 4)  /* Bit 4: GPDMA1 enable */
#define RCC_AHB1ENR_ADC12EN           (1 << 5)  /* Bit 5: ADC12 enable */

/* AHB2 enable register (AHB2ENR) */

#define RCC_AHB2ENR_RAMCFGEN          (1 << 12)  /* Bit 12: RAMCFG enable */
#define RCC_AHB2ENR_MDF1EN            (1 << 16)  /* Bit 16: MDF1 enable */
#define RCC_AHB2ENR_ADF1EN            (1 << 17)  /* Bit 17: ADF1 enable */

/* AHB3 enable register (AHB3ENR) */

#define RCC_AHB3ENR_RNGEN             (1 << 0)   /* Bit 0: RNG enable */
#define RCC_AHB3ENR_HASHEN            (1 << 1)   /* Bit 1: HASH enable */
#define RCC_AHB3ENR_CRYPEN            (1 << 2)   /* Bit 2: CRYP enable */
#define RCC_AHB3ENR_SAESEN            (1 << 4)   /* Bit 4: SAES enable */
#define RCC_AHB3ENR_PKAEN             (1 << 8)   /* Bit 8: PKA enable */
#define RCC_AHB3ENR_RIFSCEN           (1 << 9)   /* Bit 9: RIFSC enable */
#define RCC_AHB3ENR_IACEN             (1 << 10)  /* Bit 10: IAC enable */
#define RCC_AHB3ENR_RISAFEN           (1 << 14)  /* Bit 14: RISAF enable */

/* AHB4 peripheral clock enable register */

#define RCC_AHB4ENR_GPIOAEN           (1 << 0)   /* Bit 0:  GPIOA enable */
#define RCC_AHB4ENR_GPIOBEN           (1 << 1)   /* Bit 1:  GPIOB enable */
#define RCC_AHB4ENR_GPIOCEN           (1 << 2)   /* Bit 2:  GPIOC enable */
#define RCC_AHB4ENR_GPIODEN           (1 << 3)   /* Bit 3:  GPIOD enable */
#define RCC_AHB4ENR_GPIOEEN           (1 << 4)   /* Bit 4:  GPIOE enable */
#define RCC_AHB4ENR_GPIOFEN           (1 << 5)   /* Bit 5:  GPIOF enable */
#define RCC_AHB4ENR_GPIOGEN           (1 << 6)   /* Bit 6:  GPIOG enable */
#define RCC_AHB4ENR_GPIOHEN           (1 << 7)   /* Bit 7:  GPIOH enable */
#define RCC_AHB4ENR_GPIONEN           (1 << 13)  /* Bit 13: GPION enable */
#define RCC_AHB4ENR_GPIOOEN           (1 << 14)  /* Bit 14: GPIOO enable */
#define RCC_AHB4ENR_GPIOPEN           (1 << 15)  /* Bit 15: GPIOP enable */
#define RCC_AHB4ENR_GPIOQEN           (1 << 16)  /* Bit 16: GPIOQ enable */
#define RCC_AHB4ENR_PWREN             (1 << 18)  /* Bit 18: PWR enable */
#define RCC_AHB4ENR_CRCEN             (1 << 19)  /* Bit 19: CRC enable */

/* AHB5 enable register (AHB5ENR) */

#define RCC_AHB5ENR_HPDMA1EN          (1 << 0)   /* Bit 0: HPDMA1 enable */
#define RCC_AHB5ENR_DMA2DEN           (1 << 1)   /* Bit 1: DMA2D enable */
#define RCC_AHB5ENR_JPEGEN            (1 << 3)   /* Bit 3: JPEG enable */
#define RCC_AHB5ENR_FMCEN             (1 << 4)   /* Bit 4: FMC enable */
#define RCC_AHB5ENR_XSPI1EN           (1 << 5)   /* Bit 5: XSPI1 enable */
#define RCC_AHB5ENR_PSSIEN            (1 << 6)   /* Bit 6: PSSI enable */
#define RCC_AHB5ENR_SDMMC2EN          (1 << 7)   /* Bit 7: SDMMC2 enable */
#define RCC_AHB5ENR_SDMMC1EN          (1 << 8)   /* Bit 8: SDMMC1 enable */
#define RCC_AHB5ENR_XSPI2EN           (1 << 12)  /* Bit 12: XSPI2 enable */
#define RCC_AHB5ENR_XSPIMEN           (1 << 13)  /* Bit 13: XSPIM enable */
#define RCC_AHB5ENR_MCE1EN            (1 << 14)  /* Bit 14: MCE1 enable */
#define RCC_AHB5ENR_MCE2EN            (1 << 15)  /* Bit 15: MCE2 enable */
#define RCC_AHB5ENR_MCE3EN            (1 << 16)  /* Bit 16: MCE3 enable */
#define RCC_AHB5ENR_XSPI3EN           (1 << 17)  /* Bit 17: XSPI3 enable */
#define RCC_AHB5ENR_MCE4EN            (1 << 18)  /* Bit 18: MCE4 enable */
#define RCC_AHB5ENR_GFXMMUEN          (1 << 19)  /* Bit 19: GFXMMU enable */
#define RCC_AHB5ENR_GPU2DEN           (1 << 20)  /* Bit 20: GPU2D enable */
#define RCC_AHB5ENR_ETH1MACEN         (1 << 22)  /* Bit 22: ETH1MAC enable */
#define RCC_AHB5ENR_ETH1TXEN          (1 << 23)  /* Bit 23: ETH1TX enable */
#define RCC_AHB5ENR_ETH1RXEN          (1 << 24)  /* Bit 24: ETH1RX enable */
#define RCC_AHB5ENR_ETH1EN            (1 << 25)  /* Bit 25: ETH1 enable */
#define RCC_AHB5ENR_OTG1EN            (1 << 26)  /* Bit 26: OTG1 enable */
#define RCC_AHB5ENR_OTGPHY1EN         (1 << 27)  /* Bit 27: OTGPHY1 enable */
#define RCC_AHB5ENR_OTGPHY2EN         (1 << 28)  /* Bit 28: OTGPHY2 enable */
#define RCC_AHB5ENR_OTG2EN            (1 << 29)  /* Bit 29: OTG2 enable */
#define RCC_AHB5ENR_CACHEAXIEN        (1 << 30)  /* Bit 30: CACHEAXI enable */
#define RCC_AHB5ENR_NPUEN             (1 << 31)  /* Bit 31: NPU enable */

/* APB1 peripheral clock enable register 1 */

#define RCC_APB1LENR_TIM2EN           (1 << 0)   /* Bit 0:  TIM2 enable */
#define RCC_APB1LENR_TIM3EN           (1 << 1)   /* Bit 1:  TIM3 enable */
#define RCC_APB1LENR_TIM4EN           (1 << 2)   /* Bit 2:  TIM4 enable */
#define RCC_APB1LENR_TIM5EN           (1 << 3)   /* Bit 3:  TIM5 enable */
#define RCC_APB1LENR_TIM6EN           (1 << 4)   /* Bit 4:  TIM6 enable */
#define RCC_APB1LENR_TIM7EN           (1 << 5)   /* Bit 5:  TIM7 enable */
#define RCC_APB1LENR_TIM12EN          (1 << 6)   /* Bit 6:  TIM12 enable */
#define RCC_APB1LENR_TIM13EN          (1 << 7)   /* Bit 7:  TIM13 enable */
#define RCC_APB1LENR_TIM14EN          (1 << 8)   /* Bit 8:  TIM14 enable */
#define RCC_APB1LENR_LPTIM1EN         (1 << 9)   /* Bit 9:  LPTIM1 enable */
#define RCC_APB1LENR_WWDGEN           (1 << 11)  /* Bit 11: WWDG enable */
#define RCC_APB1LENR_TIM10EN          (1 << 12)  /* Bit 12: TIM10 enable */
#define RCC_APB1LENR_TIM11EN          (1 << 13)  /* Bit 13: TIM11 enable */
#define RCC_APB1LENR_SPI2EN           (1 << 14)  /* Bit 14: SPI2 enable */
#define RCC_APB1LENR_SPI3EN           (1 << 15)  /* Bit 15: SPI3 enable */
#define RCC_APB1LENR_SPDIFRX1EN       (1 << 16)  /* Bit 16: SPDIFRX1 enable */
#define RCC_APB1LENR_USART2EN         (1 << 17)  /* Bit 17: USART2 enable */
#define RCC_APB1LENR_USART3EN         (1 << 18)  /* Bit 18: USART3 enable */
#define RCC_APB1LENR_UART4EN          (1 << 19)  /* Bit 19: UART4 enable */
#define RCC_APB1LENR_UART5EN          (1 << 20)  /* Bit 20: UART5 enable */
#define RCC_APB1LENR_I2C1EN           (1 << 21)  /* Bit 21: I2C1 enable */
#define RCC_APB1LENR_I2C2EN           (1 << 22)  /* Bit 22: I2C2 enable */
#define RCC_APB1LENR_I2C3EN           (1 << 23)  /* Bit 23: I2C3 enable */
#define RCC_APB1LENR_I3C1EN           (1 << 24)  /* Bit 24: I3C1 enable */
#define RCC_APB1LENR_I3C2EN           (1 << 25)  /* Bit 25: I3C2 enable */
#define RCC_APB1LENR_UART7EN          (1 << 30)  /* Bit 30: UART7 enable */
#define RCC_APB1LENR_UART8EN          (1 << 31)  /* Bit 31: UART8 enable */

/* APB1 enable register 2 (APB1HENR) */

#define RCC_APB1HENR_MDIOSEN          (1 << 5)   /* Bit 5: MDIOS enable */
#define RCC_APB1HENR_FDCANEN          (1 << 8)   /* Bit 8: FDCAN enable */
#define RCC_APB1HENR_UCPD1EN          (1 << 18)  /* Bit 18: UCPD1 enable */

/* APB2 peripheral clock enable register */

#define RCC_APB2ENR_TIM1EN            (1 << 0)   /* Bit 0:  TIM1 enable */
#define RCC_APB2ENR_TIM8EN            (1 << 1)   /* Bit 1:  TIM8 enable */
#define RCC_APB2ENR_USART1EN          (1 << 4)   /* Bit 4:  USART1 enable */
#define RCC_APB2ENR_USART6EN          (1 << 5)   /* Bit 5: USART6 enable */
#define RCC_APB2ENR_UART9EN           (1 << 6)   /* Bit 6: UART9 enable */
#define RCC_APB2ENR_USART10EN         (1 << 7)   /* Bit 7: USART10 enable */
#define RCC_APB2ENR_SPI1EN            (1 << 12)  /* Bit 12: SPI1 enable */
#define RCC_APB2ENR_SPI4EN            (1 << 13)  /* Bit 13: SPI4 enable */
#define RCC_APB2ENR_TIM18EN           (1 << 15)  /* Bit 15: TIM18 enable */
#define RCC_APB2ENR_TIM15EN           (1 << 16)  /* Bit 16: TIM15 enable */
#define RCC_APB2ENR_TIM16EN           (1 << 17)  /* Bit 17: TIM16 enable */
#define RCC_APB2ENR_TIM17EN           (1 << 18)  /* Bit 18: TIM17 enable */
#define RCC_APB2ENR_TIM9EN            (1 << 19)  /* Bit 19: TIM9 enable */
#define RCC_APB2ENR_SPI5EN            (1 << 20)  /* Bit 20: SPI5 enable */
#define RCC_APB2ENR_SAI1EN            (1 << 21)  /* Bit 21: SAI1 enable */
#define RCC_APB2ENR_SAI2EN            (1 << 22)  /* Bit 22: SAI2 enable */

/* APB3 enable register (APB3ENR) */

#define RCC_APB3ENR_DFTEN             (1 << 2)  /* Bit 2: DFT enable */

/* APB4 peripheral clock enable register 1 */

#define RCC_APB4LENR_HDPEN            (1 << 2)   /* Bit 2: HDP enable */
#define RCC_APB4LENR_LPUART1EN        (1 << 3)   /* Bit 3: LPUART1 enable */
#define RCC_APB4LENR_SPI6EN           (1 << 5)   /* Bit 5: SPI6 enable */
#define RCC_APB4LENR_I2C4EN           (1 << 7)   /* Bit 7: I2C4 enable */
#define RCC_APB4LENR_LPTIM2EN         (1 << 9)   /* Bit 9:  LPTIM2 enable */
#define RCC_APB4LENR_LPTIM3EN         (1 << 10)  /* Bit 10: LPTIM3 enable */
#define RCC_APB4LENR_LPTIM4EN         (1 << 11)  /* Bit 11: LPTIM4 enable */
#define RCC_APB4LENR_LPTIM5EN         (1 << 12)  /* Bit 12: LPTIM5 enable */
#define RCC_APB4LENR_VREFBUFEN        (1 << 15)  /* Bit 15: VREFBUF enable */
#define RCC_APB4LENR_RTCEN            (1 << 16)  /* Bit 16: RTC enable */
#define RCC_APB4LENR_RTCAPBEN         (1 << 17)  /* Bit 17: RTCAPB enable */

/* APB4 peripheral clock enable register 2 */

#define RCC_APB4HENR_SYSCFGEN         (1 << 0)  /* Bit 0:  SYSCFG enable */
#define RCC_APB4HENR_BSECEN           (1 << 1)  /* Bit 1:  BSEC enable */
#define RCC_APB4HENR_DTSEN            (1 << 2)  /* Bit 2: DTS enable */

/* APB5 enable register (APB5ENR) */

#define RCC_APB5ENR_LTDCEN            (1 << 1)  /* Bit 1: LTDC enable */
#define RCC_APB5ENR_DCMIPPEN          (1 << 2)  /* Bit 2: DCMIPP enable */
#define RCC_APB5ENR_GFXTIMEN          (1 << 4)  /* Bit 4: GFXTIM enable */
#define RCC_APB5ENR_VENCEN            (1 << 5)  /* Bit 5: VENC enable */
#define RCC_APB5ENR_CSIEN             (1 << 6)  /* Bit 6: CSI enable */

/* Bus clock enable in Sleep mode */

#define RCC_BUSLPENR_ACLKNLPEN        (1 << 0)  /* Bit 0: ACLKN clock
                                                 * enable in CSLEEP */
#define RCC_BUSLPENR_ACLKNCLPEN       (1 << 1)  /* Bit 1: ACLKNC clock
                                                 * enable in CSLEEP */

/* Miscellaneous configurations sleep enable register (MISCLPENR) */

#define RCC_MISCLPENR_DBGLPEN         (1 << 0)  /* Bit 0: DBG enable */
#define RCC_MISCLPENR_XSPIPHYCOMPLPEN (1 << 3)  /* Bit 3: XSPIPHYCOMP
                                                 * enable */
#define RCC_MISCLPENR_PERLPEN         (1 << 6)  /* Bit 6: PER enable */

/* SRAM clock enable in Sleep mode */

#define RCC_MEMLPENR_AXISRAM3LPEN     (1 << 0)   /* Bit 0: AXISRAM3 enable */
#define RCC_MEMLPENR_AXISRAM4LPEN     (1 << 1)   /* Bit 1: AXISRAM4 enable */
#define RCC_MEMLPENR_AXISRAM5LPEN     (1 << 2)   /* Bit 2: AXISRAM5 enable */
#define RCC_MEMLPENR_AXISRAM6LPEN     (1 << 3)   /* Bit 3: AXISRAM6 enable */
#define RCC_MEMLPENR_AHBSRAM1LPEN     (1 << 4)   /* Bit 4: AHBSRAM1 enable */
#define RCC_MEMLPENR_AHBSRAM2LPEN     (1 << 5)   /* Bit 5: AHBSRAM2 enable */
#define RCC_MEMLPENR_BKPSRAMLPEN      (1 << 6)   /* Bit 6: BKPSRAM enable */
#define RCC_MEMLPENR_AXISRAM1LPEN     (1 << 7)   /* Bit 7: AXISRAM1 enable */
#define RCC_MEMLPENR_AXISRAM2LPEN     (1 << 8)   /* Bit 8: AXISRAM2 enable */
#define RCC_MEMLPENR_FLEXRAMLPEN      (1 << 9)   /* Bit 9: FLEXRAM enable */
#define RCC_MEMLPENR_CACHEAXIRAMLPEN  (1 << 10)  /* Bit 10: CACHEAXIRAM
                                                  * enable in CSLEEP */
#define RCC_MEMLPENR_VENCRAMLPEN      (1 << 11)  /* Bit 11: VENCRAM enable */
#define RCC_MEMLPENR_BOOTROMLPEN      (1 << 12)  /* Bit 12: Boot ROM enable */

#define RCC_MEMLPENR_ALLAXISRAM       (RCC_MEMLPENR_AXISRAM1LPEN | RCC_MEMLPENR_AXISRAM2LPEN | \
                                       RCC_MEMLPENR_AXISRAM3LPEN | RCC_MEMLPENR_AXISRAM4LPEN | \
                                       RCC_MEMLPENR_AXISRAM5LPEN | RCC_MEMLPENR_AXISRAM6LPEN)

/* AHB1 sleep enable register (AHB1LPENR) */

#define RCC_AHB1LPENR_GPDMA1LPEN      (1 << 4)  /* Bit 4: GPDMA1 enable */
#define RCC_AHB1LPENR_ADC12LPEN       (1 << 5)  /* Bit 5: ADC12 enable */

/* AHB2 sleep enable register (AHB2LPENR) */

#define RCC_AHB2LPENR_RAMCFGLPEN      (1 << 12)  /* Bit 12: RAMCFG enable */
#define RCC_AHB2LPENR_MDF1LPEN        (1 << 16)  /* Bit 16: MDF1 enable */
#define RCC_AHB2LPENR_ADF1LPEN        (1 << 17)  /* Bit 17: ADF1 enable */

/* AHB3 sleep enable register (AHB3LPENR) */

#define RCC_AHB3LPENR_RNGLPEN         (1 << 0)   /* Bit 0: RNG enable */
#define RCC_AHB3LPENR_HASHLPEN        (1 << 1)   /* Bit 1: HASH enable */
#define RCC_AHB3LPENR_CRYPLPEN        (1 << 2)   /* Bit 2: CRYP enable */
#define RCC_AHB3LPENR_SAESLPEN        (1 << 4)   /* Bit 4: SAES enable */
#define RCC_AHB3LPENR_PKALPEN         (1 << 8)   /* Bit 8: PKA enable */
#define RCC_AHB3LPENR_RIFSCLPEN       (1 << 9)   /* Bit 9: RIFSC enable */
#define RCC_AHB3LPENR_IACLPEN         (1 << 10)  /* Bit 10: IAC enable */
#define RCC_AHB3LPENR_RISAFLPEN       (1 << 14)  /* Bit 14: RISAF enable */

/* AHB4 sleep enable register (AHB4LPENR) */

#define RCC_AHB4LPENR_GPIOALPEN       (1 << 0)   /* Bit 0: GPIO A enable */
#define RCC_AHB4LPENR_GPIOBLPEN       (1 << 1)   /* Bit 1: GPIO B enable */
#define RCC_AHB4LPENR_GPIOCLPEN       (1 << 2)   /* Bit 2: GPIO C enable */
#define RCC_AHB4LPENR_GPIODLPEN       (1 << 3)   /* Bit 3: GPIO D enable */
#define RCC_AHB4LPENR_GPIOELPEN       (1 << 4)   /* Bit 4: GPIO E enable */
#define RCC_AHB4LPENR_GPIOFLPEN       (1 << 5)   /* Bit 5: GPIO F enable */
#define RCC_AHB4LPENR_GPIOGLPEN       (1 << 6)   /* Bit 6: GPIO G enable */
#define RCC_AHB4LPENR_GPIOHLPEN       (1 << 7)   /* Bit 7: GPIO H enable */
#define RCC_AHB4LPENR_GPIONLPEN       (1 << 13)  /* Bit 13: GPIO N enable */
#define RCC_AHB4LPENR_GPIOOLPEN       (1 << 14)  /* Bit 14: GPIO O enable */
#define RCC_AHB4LPENR_GPIOPLPEN       (1 << 15)  /* Bit 15: GPIO P enable */
#define RCC_AHB4LPENR_GPIOQLPEN       (1 << 16)  /* Bit 16: GPIO Q enable */
#define RCC_AHB4LPENR_PWRLPEN         (1 << 18)  /* Bit 18: PWR enable */
#define RCC_AHB4LPENR_CRCLPEN         (1 << 19)  /* Bit 19: CRC enable */

/* AHB5 sleep enable register (AHB5LPENR) */

#define RCC_AHB5LPENR_HPDMA1LPEN      (1 << 0)   /* Bit 0: HPDMA1 enable */
#define RCC_AHB5LPENR_DMA2DLPEN       (1 << 1)   /* Bit 1: DMA2D enable */
#define RCC_AHB5LPENR_JPEGLPEN        (1 << 3)   /* Bit 3: JPEG enable */
#define RCC_AHB5LPENR_FMCLPEN         (1 << 4)   /* Bit 4: FMC enable */
#define RCC_AHB5LPENR_XSPI1LPEN       (1 << 5)   /* Bit 5: XSPI1 enable */
#define RCC_AHB5LPENR_PSSILPEN        (1 << 6)   /* Bit 6: PSSI enable */
#define RCC_AHB5LPENR_SDMMC2LPEN      (1 << 7)   /* Bit 7: SDMMC2 enable */
#define RCC_AHB5LPENR_SDMMC1LPEN      (1 << 8)   /* Bit 8: SDMMC1 enable */
#define RCC_AHB5LPENR_XSPI2LPEN       (1 << 12)  /* Bit 12: XSPI2 enable */
#define RCC_AHB5LPENR_XSPIMLPEN       (1 << 13)  /* Bit 13: XSPIM enable */
#define RCC_AHB5LPENR_MCE1LPEN        (1 << 14)  /* Bit 14: MCE1 enable */
#define RCC_AHB5LPENR_MCE2LPEN        (1 << 15)  /* Bit 15: MCE2 enable */
#define RCC_AHB5LPENR_MCE3LPEN        (1 << 16)  /* Bit 16: MCE3 enable */
#define RCC_AHB5LPENR_XSPI3LPEN       (1 << 17)  /* Bit 17: XSPI3 enable */
#define RCC_AHB5LPENR_MCE4LPEN        (1 << 18)  /* Bit 18: MCE4 enable */
#define RCC_AHB5LPENR_GFXMMULPEN      (1 << 19)  /* Bit 19: GFXMMU enable */
#define RCC_AHB5LPENR_GPU2DLPEN       (1 << 20)  /* Bit 20: GPU2D enable */
#define RCC_AHB5LPENR_ETH1MACLPEN     (1 << 22)  /* Bit 22: ETH1MAC enable */
#define RCC_AHB5LPENR_ETH1TXLPEN      (1 << 23)  /* Bit 23: ETH1TX enable */
#define RCC_AHB5LPENR_ETH1RXLPEN      (1 << 24)  /* Bit 24: ETH1RX enable */
#define RCC_AHB5LPENR_ETH1LPEN        (1 << 25)  /* Bit 25: ETH1 enable */
#define RCC_AHB5LPENR_OTG1LPEN        (1 << 26)  /* Bit 26: OTG1 enable */
#define RCC_AHB5LPENR_OTGPHY1LPEN     (1 << 27)  /* Bit 27: OTGPHY1 enable */
#define RCC_AHB5LPENR_OTGPHY2LPEN     (1 << 28)  /* Bit 28: OTGPHY2 enable */
#define RCC_AHB5LPENR_OTG2LPEN        (1 << 29)  /* Bit 29: OTG2 enable */
#define RCC_AHB5LPENR_CACHEAXILPEN    (1 << 30)  /* Bit 30: CACHEAXI enable */
#define RCC_AHB5LPENR_NPULPEN         (1 << 31)  /* Bit 31: NPU enable */

/* APB1 peripheral clock enable in Sleep mode (register 1) */

#define RCC_APB1LLPENR_TIM2LPEN       (1 << 0)   /* Bit 0: TIM2 enable in
                                                  * CSLEEP */
#define RCC_APB1LLPENR_TIM3LPEN       (1 << 1)   /* Bit 1: TIM3 enable in
                                                  * CSLEEP */
#define RCC_APB1LLPENR_TIM4LPEN       (1 << 2)   /* Bit 2: TIM4 enable in
                                                  * CSLEEP */
#define RCC_APB1LLPENR_TIM5LPEN       (1 << 3)   /* Bit 3: TIM5 enable in
                                                  * CSLEEP */
#define RCC_APB1LLPENR_TIM6LPEN       (1 << 4)   /* Bit 4: TIM6 enable in
                                                  * CSLEEP */
#define RCC_APB1LLPENR_TIM7LPEN       (1 << 5)   /* Bit 5: TIM7 enable in
                                                  * CSLEEP */
#define RCC_APB1LLPENR_TIM12LPEN      (1 << 6)   /* Bit 6: TIM12 enable in
                                                  * CSLEEP */
#define RCC_APB1LLPENR_TIM13LPEN      (1 << 7)   /* Bit 7: TIM13 enable in
                                                  * CSLEEP */
#define RCC_APB1LLPENR_TIM14LPEN      (1 << 8)   /* Bit 8: TIM14 enable in
                                                  * CSLEEP */
#define RCC_APB1LLPENR_LPTIM1LPEN     (1 << 9)   /* Bit 9: LPTIM1 enable */
#define RCC_APB1LLPENR_WWDGLPEN       (1 << 11)  /* Bit 11: WWDG enable */
#define RCC_APB1LLPENR_TIM10LPEN      (1 << 12)  /* Bit 12: TIM10 enable in
                                                  * CSLEEP */
#define RCC_APB1LLPENR_TIM11LPEN      (1 << 13)  /* Bit 13: TIM11 enable in
                                                  * CSLEEP */
#define RCC_APB1LLPENR_SPI2LPEN       (1 << 14)  /* Bit 14: SPI2 enable */
#define RCC_APB1LLPENR_SPI3LPEN       (1 << 15)  /* Bit 15: SPI3 enable */
#define RCC_APB1LLPENR_SPDIFRX1LPEN   (1 << 16)  /* Bit 16: SPDIFRX1 enable */
#define RCC_APB1LLPENR_USART2LPEN     (1 << 17)  /* Bit 17: USART2 enable */
#define RCC_APB1LLPENR_USART3LPEN     (1 << 18)  /* Bit 18: USART3 enable */
#define RCC_APB1LLPENR_UART4LPEN      (1 << 19)  /* Bit 19: UART4 enable */
#define RCC_APB1LLPENR_UART5LPEN      (1 << 20)  /* Bit 20: UART5 enable */
#define RCC_APB1LLPENR_I2C1LPEN       (1 << 21)  /* Bit 21: I2C1 enable */
#define RCC_APB1LLPENR_I2C2LPEN       (1 << 22)  /* Bit 22: I2C2 enable */
#define RCC_APB1LLPENR_I2C3LPEN       (1 << 23)  /* Bit 23: I2C3 enable */
#define RCC_APB1LLPENR_I3C1LPEN       (1 << 24)  /* Bit 24: I3C1 enable */
#define RCC_APB1LLPENR_I3C2LPEN       (1 << 25)  /* Bit 25: I3C2 enable */
#define RCC_APB1LLPENR_UART7LPEN      (1 << 30)  /* Bit 30: UART7 enable */
#define RCC_APB1LLPENR_UART8LPEN      (1 << 31)  /* Bit 31: UART8 enable */

#define RCC_APB1LLPENR_ALLTIM         (RCC_APB1LLPENR_TIM2LPEN | RCC_APB1LLPENR_TIM3LPEN | \
                                       RCC_APB1LLPENR_TIM4LPEN | RCC_APB1LLPENR_TIM5LPEN | \
                                       RCC_APB1LLPENR_TIM6LPEN | RCC_APB1LLPENR_TIM7LPEN | \
                                       RCC_APB1LLPENR_TIM10LPEN | RCC_APB1LLPENR_TIM11LPEN | \
                                       RCC_APB1LLPENR_TIM12LPEN | RCC_APB1LLPENR_TIM13LPEN | \
                                       RCC_APB1LLPENR_TIM14LPEN)

/* APB1 sleep enable register 2 (APB1HLPENR) */

#define RCC_APB1HLPENR_MDIOSLPEN      (1 << 5)   /* Bit 5: MDIOS enable in
                                                  * Sleep mode */
#define RCC_APB1HLPENR_FDCANLPEN      (1 << 8)   /* Bit 8: FDCAN enablein
                                                  * Sleep mode */
#define RCC_APB1HLPENR_UCPD1LPEN      (1 << 18)  /* Bit 18: UCPD1 enable in
                                                  * Sleep mode */

/* APB2 peripheral clock enable in Sleep mode */

#define RCC_APB2LPENR_TIM1LPEN        (1 << 0)   /* Bit 0: TIM1 enable in
                                                  * CSLEEP */
#define RCC_APB2LPENR_TIM8LPEN        (1 << 1)   /* Bit 1: TIM8 enable in
                                                  * CSLEEP */
#define RCC_APB2LPENR_USART1LPEN      (1 << 4)   /* Bit 4: USART1 enable in
                                                  * CSLEEP */
#define RCC_APB2LPENR_USART6LPEN      (1 << 5)   /* Bit 5: USART6 enable */
#define RCC_APB2LPENR_UART9LPEN       (1 << 6)   /* Bit 6: UART9 enable */
#define RCC_APB2LPENR_USART10LPEN     (1 << 7)   /* Bit 7: USART10 enable */
#define RCC_APB2LPENR_SPI1LPEN        (1 << 12)  /* Bit 12: SPI1 enable */
#define RCC_APB2LPENR_SPI4LPEN        (1 << 13)  /* Bit 13: SPI4 enable */
#define RCC_APB2LPENR_TIM18LPEN       (1 << 15)  /* Bit 15: TIM18 enable in
                                                  * CSLEEP */
#define RCC_APB2LPENR_TIM15LPEN       (1 << 16)  /* Bit 16: TIM15 enable in
                                                  * CSLEEP */
#define RCC_APB2LPENR_TIM16LPEN       (1 << 17)  /* Bit 17: TIM16 enable in
                                                  * CSLEEP */
#define RCC_APB2LPENR_TIM17LPEN       (1 << 18)  /* Bit 18: TIM17 enable in
                                                  * CSLEEP */
#define RCC_APB2LPENR_TIM9LPEN        (1 << 19)  /* Bit 19: TIM9 enable in
                                                  * CSLEEP */
#define RCC_APB2LPENR_SPI5LPEN        (1 << 20)  /* Bit 20: SPI5 enable */
#define RCC_APB2LPENR_SAI1LPEN        (1 << 21)  /* Bit 21: SAI1 enable */
#define RCC_APB2LPENR_SAI2LPEN        (1 << 22)  /* Bit 22: SAI2 enable */

#define RCC_APB2LPENR_ALLTIM          (RCC_APB2LPENR_TIM1LPEN | RCC_APB2LPENR_TIM8LPEN | \
                                       RCC_APB2LPENR_TIM9LPEN | RCC_APB2LPENR_TIM15LPEN | \
                                       RCC_APB2LPENR_TIM16LPEN | RCC_APB2LPENR_TIM17LPEN | \
                                       RCC_APB2LPENR_TIM18LPEN)

/* APB3 sleep enable register (APB3LPENR) */

#define RCC_APB3LPENR_DFTLPEN         (1 << 2)  /* Bit 2: DFT enable */

/* APB4 sleep enable register 1 (APB4LLPENR) */

#define RCC_APB4LLPENR_HDPLPEN        (1 << 2)   /* Bit 2: HDP enable */
#define RCC_APB4LLPENR_LPUART1LPEN    (1 << 3)   /* Bit 3: LPUART1 enable */
#define RCC_APB4LLPENR_SPI6LPEN       (1 << 5)   /* Bit 5: SPI6 enable */
#define RCC_APB4LLPENR_I2C4LPEN       (1 << 7)   /* Bit 7: I2C4 enable */
#define RCC_APB4LLPENR_LPTIM2LPEN     (1 << 9)   /* Bit 9: LPTIM2 enable */
#define RCC_APB4LLPENR_LPTIM3LPEN     (1 << 10)  /* Bit 10: LPTIM3 enable */
#define RCC_APB4LLPENR_LPTIM4LPEN     (1 << 11)  /* Bit 11: LPTIM4 enable */
#define RCC_APB4LLPENR_LPTIM5LPEN     (1 << 12)  /* Bit 12: LPTIM5 enable */
#define RCC_APB4LLPENR_VREFBUFLPEN    (1 << 15)  /* Bit 15: VREFBUF enable */
#define RCC_APB4LLPENR_RTCLPEN        (1 << 16)  /* Bit 16: RTC enable */
#define RCC_APB4LLPENR_RTCAPBLPEN     (1 << 17)  /* Bit 17: RTCAPB enable */

#define RCC_APB4LLPENR_ALLLPTIM       (RCC_APB4LLPENR_LPTIM2LPEN | RCC_APB4LLPENR_LPTIM3LPEN | \
                                       RCC_APB4LLPENR_LPTIM4LPEN | RCC_APB4LLPENR_LPTIM5LPEN)

/* APB4 sleep enable register 2 (APB4HLPENR) */

#define RCC_APB4HLPENR_SYSCFGLPEN     (1 << 0)  /* Bit 0: SYSCFG enable */
#define RCC_APB4HLPENR_BSECLPEN       (1 << 1)  /* Bit 1: BSEC enable */
#define RCC_APB4HLPENR_DTSLPEN        (1 << 2)  /* Bit 2: DTS enable */

/* APB5 sleep enable register (APB5LPENR) */

#define RCC_APB5LPENR_LTDCLPEN        (1 << 1)  /* Bit 1: LTDC enable */
#define RCC_APB5LPENR_DCMIPPLPEN      (1 << 2)  /* Bit 2: DCMIPP enable */
#define RCC_APB5LPENR_GFXTIMLPEN      (1 << 4)  /* Bit 4: GFXTIM enable */
#define RCC_APB5LPENR_VENCLPEN        (1 << 5)  /* Bit 5: VENC enable */
#define RCC_APB5LPENR_CSILPEN         (1 << 6)  /* Bit 6: CSI enable */

/* Compatibility aliases used by the common STM32 drivers */

#define RCC_CFGR2                     STM32_RCC_CFGR2

#endif /* __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_RCC_H */
