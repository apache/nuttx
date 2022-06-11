/****************************************************************************
 * arch/arm/src/stm32wl5/hardware/stm32wl5_memorymap.h
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

#ifndef __ARCH_ARM_SRC_STM32WL5_STM32WL5_MEMORYMAP_H
#define __ARCH_ARM_SRC_STM32WL5_STM32WL5_MEMORYMAP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* STM32WL5XXX Address Blocks ***********************************************/

#define STM32WL5_CODE_BASE      0x00000000    /* 0x0000 0000-0x1fff ffff: 512Mb code block */
#define STM32WL5_SRAM_BASE      0x20000000    /* 0x2000 0000-0x3fff ffff: 512Mb sram block (48k to 256k) */
#define STM32WL5_PERIPH_BASE    0x40000000    /* 0x4000 0000-0x5fff ffff: 512Mb peripheral block */
                                              /* 0x6000 0000-0xdfff ffff: 2048Mb (not used) */
#define STM32WL5_CORTEX_BASE    0xe0000000    /* 0xe000 0000-0xffff ffff: 512Mb Cortex-M4/M0 block */

#define STM32WL5_REGION_MASK    0xf0000000
#define STM32WL5_IS_SRAM(a)     ((((uint32_t)(a)) & STM32WL5_REGION_MASK) == STM32WL5_SRAM_BASE)

/* Code Base Addresses ******************************************************/

#define STM32WL5_BOOT_BASE      0x00000000     /* 0x0000 0000-0x0003 ffff: Aliased boot memory */
                                               /* 0x0004 0000-0x07ff ffff: Reserved */
#define STM32WL5_FLASH_BASE     0x08000000     /* 0x0800 0000-0x0803 ffff: FLASH memory */
                                               /* 0x0804 0000-0x0fff ffff: Reserved */
#define STM32WL5_FLASH_MASK     0xf8000000     /*                          Test if addr in FLASH */
                                               /* 0x1000 0000-0x1ffe 6fff: Reserved */
#define STM32WL5_SYSMEM_BASE    0x1fff0000     /* 0x1fff 0000-0x1fff 6fff: System memory */
#define STM32WL5_OTP_BASE       0x1fff7000     /* 0x1fff 7000-0x1fff 73ff: 1k otp memory */
#define STM32WL5_ENGI_BASE      0x1fff7400     /* 0x1fff 7400-0x1fff 77ff: 1k engi flash */
#define STM32WL5_OPTION_BASE    0x1fff7800     /* 0x1fff 7800-0x1fff 7fff: 2k flash user options */
                                               /* 0x1fff 8000-0x1fff ffff: reserved */
#define STM32WL5_SRAM2_BASE     0x20008000     /* 0x2000 8000-0x2000 ffff: 32k SRAM2 */

/* System Memory Addresses **************************************************/

#define STM32WL5_SYSMEM_PACKAGE 0x1fff7500     /* This bitfield indicates the package
                                               * type.
                                               * 0:  UFBGA73
                                               * 2:  WLCSP59
                                               * 10: UFQFPN48
                                               */
#define STM32WL5_SYSMEM_UID     0x1fff7590     /* The 96-bit unique device identifier */
#define STM32WL5_SYSMEM_FSIZE   0x1fff75E0     /* This bitfield indicates the size of
                                               * the device Flash memory expressed in
                                               * Kbytes. Example: 0x0400 corresponds
                                               * to 1024 Kbytes.
                                               */

/* SRAM Base Addresses ******************************************************/

#define STM32WL5_SRAMBB_BASE    0x22000000     /* 0x22000000-          : SRAM bit-band region */

/* Peripheral Base Addresses ************************************************/

#define STM32WL5_APB1_BASE      0x40000000     /* 0x4000 0000-0x4000 b3ff: APB1 */
                                               /* 0x4000 B400-0x4000 ffff: Reserved */
#define STM32WL5_APB2_BASE      0x40010000     /* 0x4001 0000-0x4001 4bff: APB2 */
                                               /* 0x4001 4c00-0x4001 ffff: Reserved */
#define STM32WL5_AHB1_BASE      0x40020000     /* 0x4002 0000-0x425f ffff: APB1 */
                                               /* 0x4260 0000-0x47ff ffff: Reserved */
#define STM32WL5_AHB2_BASE      0x48000000     /* 0x4800 0000-0x4800 1fff: AHB2 */
                                               /* 0x4800 2000-0x57ff ffff: Reserved */
#define STM32WL5_AHB3_BASE      0x58000000     /* 0x5800 0000-0x5800 4bff: AHB3 */
                                               /* 0x5800 40c0-0x5800 ffff: Reserved */

/* Radio Base Addresses *****************************************************/

#define STM32WL5_APB3_BASE      0x58010000     /* 0x5801 0000-0x5801 03ff: APB3 */
                                               /* 0x5801 0400-0x5801 ffff: Reserved */

/* in datasheet order */

/* APB1 Base Addresses ******************************************************/

#define STM32WL5_TAMP_BASE      0x4000B000
#define STM32WL5_LPTIM3_BASE    0x40009800
#define STM32WL5_LPTIM2_BASE    0x40009400
#define STM32WL5_LPUART1_BASE   0x40008000
#define STM32WL5_LPTIM1_BASE    0x40007C00
#define STM32WL5_DAC_BASE       0x40007400
#define STM32WL5_I2C3_BASE      0x40005C00
#define STM32WL5_I2C2_BASE      0x40005800
#define STM32WL5_I2C1_BASE      0x40005400
#define STM32WL5_USART2_BASE    0x40004400
#define STM32WL5_SPI2S2_BASE    0x40003800
#define STM32WL5_IWDG_BASE      0x40003000
#define STM32WL5_WWDG_BASE      0x40002C00
#define STM32WL5_RTC_BASE       0x40002800
#define STM32WL5_TIM2_BASE      0x40000000

/* APB2 Base Addresses ******************************************************/

#define STM32WL5_TIM17_BASE     0x40014800
#define STM32WL5_TIM16_BASE     0x40014400
#define STM32WL5_USART1_BASE    0x40013800
#define STM32WL5_SPI1_BASE      0x40013000
#define STM32WL5_TIM1_BASE      0x40012C00
#define STM32WL5_ADC_BASE       0x40012400
#define STM32WL5_COMP_BASE      0x40010200
#define STM32WL5_SYSCFG2_BASE   0x40010100
#define STM32WL5_VREFBUF_BASE   0x40010030
#define STM32WL5_SYSCFG_BASE    0x40010000

/* AHB1 Base Addresses ******************************************************/

#define STM32WL5_CRC_BASE       0x40023000
#define STM32WL5_DMAMUX1_BASE   0x40200800
#define STM32WL5_DMA2_BASE      0x40200400
#define STM32WL5_DMA1_BASE      0x40020000

/* AHB2 Base Addresses ******************************************************/

#define STM32WL5_GPIOH_BASE     0x48001C00
#define STM32WL5_GPIOC_BASE     0x48000800
#define STM32WL5_GPIOB_BASE     0x48000400
#define STM32WL5_GPIOA_BASE     0x48000000

/* AHB3 Base Addresses ******************************************************/

#define STM32WL5_GTZC_TZIC_BASE 0x58004800
#define STM32WL5_GTZC_TZSC_BASE 0x58004400
#define STM32WL5_FLASHIF_BASE   0x58004000
#define STM32WL5_PKA2_BASE      0x58003400
#define STM32WL5_PKARAM_BASE    0x58002400
#define STM32WL5_PKA_BASE       0x58002000
#define STM32WL5_AES_BASE       0x58001800
#define STM32WL5_HSEM_BASE      0x58001400
#define STM32WL5_RNG_BASE       0x58001000
#define STM32WL5_IPCC_BASE      0x58000C00
#define STM32WL5_EXTI_BASE      0x58000800
#define STM32WL5_PWR_BASE       0x58000400
#define STM32WL5_RCC_BASE       0x58000000

/* APB3 Base Addresses ******************************************************/

#define STM32WL5_SUBGHZSPI_BASE 0x58010000

/* Cortex-M4 Base Addresses *************************************************/

/* Other registers -- see armv7-m/nvic.h for standard Cortex-M3 registers in
 * this address range
 */

#define STM32WL5_SCS_BASE      0xe000e000
#define STM32WL5_DEBUGMCU_BASE 0xe0042000

#endif /* __ARCH_ARM_SRC_STM32WL5_STM32WL5_MEMORYMAP_H */
