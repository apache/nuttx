/****************************************************************************
 * arch/arm/src/stm32n6/hardware/stm32n6xxx_memorymap.h
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

#ifndef __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_MEMORYMAP_H
#define __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_MEMORYMAP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* STM32N6XXX Address Blocks ************************************************/

#define STM32_CODE_BASE       0x00000000    /* 0x00000000-0x1fffffff: 512Mb code block */
#define STM32_SRAM_BASE       0x34000000    /* 0x34000000: SRAM (no internal flash) */
#define STM32_PERIPH_BASE     0x40000000    /* 0x40000000-0x5fffffff: 512Mb peripheral block */
#define STM32_CORTEX_BASE     0xe0000000    /* 0xe0000000-0xffffffff: 512Mb Cortex-M55 block */

#define STM32_REGION_MASK     0xf0000000
#define STM32_IS_SRAM(a)      ((((uint32_t)(a)) & STM32_REGION_MASK) == 0x30000000)

/* SRAM Addresses ***********************************************************/

#define STM32_SRAM1_BASE      0x34000000    /* SRAM1 base */
#define STM32_SRAM2_BASE      0x34100000    /* SRAM2 base */

/* External Memory **********************************************************/

#define STM32_XSPI1_BANK      0x90000000    /* XSPI1 memory-mapped region */
#define STM32_XSPI2_BANK      0x70000000    /* XSPI2 memory-mapped region */

/* Peripheral Base Addresses ************************************************
 *
 * Each peripheral is physically aliased twice: a Non-Secure alias at
 * 0x4xxxxxxx and a Secure alias at 0x5xxxxxxx (RM0486 section 3.5.1).
 * The same hardware register is reached through either alias.
 *
 * In DEV boot mode the CPU runs in Secure state with SAU disabled and
 * SAU_CTRL.ALLNS=0, so the SAU unilaterally marks every address as
 * Secure regardless of which alias is used; both aliases therefore work
 * for register access from this port.
 *
 * We pick the Secure aliases consistently to match the register listings
 * in the Reference Manual and to leave the Non-Secure aliases free for a
 * future Non-Secure world.
 *
 * Secure alias = Non-Secure alias + 0x10000000
 */

#define STM32_APB1_BASE       0x50000000    /* APB1 (Secure) */
#define STM32_APB2_BASE       0x52000000    /* APB2 (Secure) */
#define STM32_APB4_BASE       0x56000000    /* APB4 (Secure) */
#define STM32_APB5_BASE       0x58000000    /* APB5 (Secure) - LTDC/DCMIPP/CSI2/GFXTIM/VENC */
#define STM32_AHB4_BASE       0x56020000    /* AHB4 (Secure) */

/* APB1 peripherals *********************************************************/

#define STM32_TIM2_BASE       0x50000000
#define STM32_TIM3_BASE       0x50000400
#define STM32_TIM4_BASE       0x50000800
#define STM32_TIM5_BASE       0x50000c00
#define STM32_TIM6_BASE       0x50001000
#define STM32_TIM7_BASE       0x50001400
#define STM32_LPTIM1_BASE     0x50002400
#define STM32_SPI2_BASE       0x50003800
#define STM32_SPI3_BASE       0x50003c00
#define STM32_USART2_BASE     0x50004400
#define STM32_USART3_BASE     0x50004800
#define STM32_UART4_BASE      0x50004c00
#define STM32_UART5_BASE      0x50005000
#define STM32_I2C1_BASE       0x50005400
#define STM32_I2C2_BASE       0x50005800
#define STM32_I2C3_BASE       0x50005c00

/* APB2 peripherals *********************************************************/

#define STM32_TIM1_BASE       0x52000000
#define STM32_TIM8_BASE       0x52000400
#define STM32_USART1_BASE     0x52001000
#define STM32_SPI1_BASE       0x52003000
#define STM32_SPI4_BASE       0x52003400
#define STM32_TIM15_BASE      0x52004000
#define STM32_TIM16_BASE      0x52004400
#define STM32_TIM17_BASE      0x52004800
#define STM32_SPI5_BASE       0x52005000

/* APB4 peripherals *********************************************************/

#define STM32_SPI6_BASE       0x56001400
#define STM32_I2C4_BASE       0x56001c00
#define STM32_LPTIM2_BASE     0x56002400
#define STM32_LPTIM3_BASE     0x56002800
#define STM32_LPTIM4_BASE     0x56002c00
#define STM32_LPTIM5_BASE     0x56003000

/* APB4 peripherals (RTC on APB4 + 0x4000) **********************************/

#define STM32_RTC_BASE        0x56004000
#define STM32_TAMP_BASE       0x56004400
#define STM32_IWDG_BASE       0x56004800
#define STM32_BSEC_BASE       0x56009000
#define STM32_DTS_BASE        0x5600a000

/* AHB4 peripherals *********************************************************/

#define STM32_GPIOA_BASE      0x56020000
#define STM32_GPIOB_BASE      0x56020400
#define STM32_GPIOC_BASE      0x56020800
#define STM32_GPIOD_BASE      0x56020c00
#define STM32_GPIOE_BASE      0x56021000
#define STM32_GPIOF_BASE      0x56021400
#define STM32_GPIOG_BASE      0x56021800
#define STM32_GPIOH_BASE      0x56021c00
#define STM32_GPION_BASE      0x56023400
#define STM32_GPIOO_BASE      0x56023800
#define STM32_GPIOP_BASE      0x56023c00
#define STM32_GPIOQ_BASE      0x56024000

#define STM32_CRC_BASE        0x56024c00
#define STM32_PWR_BASE        0x56024800
#define STM32_EXTI_BASE       0x56025000
#define STM32_RCC_BASE        0x56028000

/* AHB1 peripherals *********************************************************/

#define STM32_GPDMA1_BASE     0x50021000    /* GPDMA1 (AHB1, Secure) */
#define STM32_ADC1_BASE       0x50022000    /* ADC1 (AHB1, Secure) */
#define STM32_ADC2_BASE       0x50022100    /* ADC2 (AHB1, Secure) */
#define STM32_ADC12_COMMON_BASE 0x50022300  /* ADC1/2 common (AHB1, Secure) */

/* AHB3 peripherals *********************************************************/

#define STM32_RNG_BASE        0x54020000    /* RNG (AHB3, Secure) */
#define STM32_RIFSC_BASE      0x54024000    /* RIFSC (AHB3, Secure) */

/* AHB2 peripherals *********************************************************/

#define STM32_RAMCFG_BASE     0x52023000    /* RAMCFG (AHB2, Secure) */

/* AHB5 peripherals *********************************************************/

#define STM32_AHB5_BASE       0x58020000    /* AHB5 (Secure) */
#define STM32_HPDMA1_BASE     0x58020000    /* HPDMA1 (AHB5, Secure) */
#define STM32_XSPI2_BASE      0x5802A000    /* XSPI2 controller (AHB5, Secure) */
#define STM32_XSPIM_BASE      0x5802B400    /* XSPIM IO Manager (AHB5, Secure) */
#define STM32_USB1_HS_PHYC_BASE 0x5803FC00  /* USB1 HS PHY controller (Secure) */
#define STM32_USB1_OTG_HS_BASE  0x58040000  /* USB1 OTG HS (Secure) */
#define STM32_USB2_OTG_HS_BASE  0x58080000  /* USB2 OTG HS (Secure) */
#define STM32_USB2_HS_PHYC_BASE 0x580C0000  /* USB2 HS PHY controller (Secure) */

/* SYSCFG *******************************************************************/

#define STM32_SYSCFG_BASE     0x56008000    /* SYSCFG (Secure) */

#endif /* __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_MEMORYMAP_H */
