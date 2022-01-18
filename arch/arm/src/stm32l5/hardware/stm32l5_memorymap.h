/****************************************************************************
 * arch/arm/src/stm32l5/hardware/stm32l5_memorymap.h
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

#ifndef __ARCH_ARM_SRC_STM32L5_HARDWARE_STM32L5_MEMORYMAP_H
#define __ARCH_ARM_SRC_STM32L5_HARDWARE_STM32L5_MEMORYMAP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* STM32L5XXX Address Blocks ************************************************/

#define STM32L5_CODE_BASE       0x00000000    /* 0x00000000-0x1fffffff: 512Mb code block */
#define STM32L5_SRAM_BASE       0x20000000    /* 0x20000000-0x3fffffff: 512Mb sram block (48k to 256k) */
#define STM32L5_PERIPH_BASE     0x40000000    /* 0x40000000-0x5fffffff: 512Mb peripheral block */
#define STM32L5_FSMC_BASE12     0x60000000    /* 0x60000000-0x7fffffff: 512Mb FSMC bank1&2 block */
#  define STM32L5_FMC_BANK1     0x60000000    /* 0x60000000-0x6fffffff:   256Mb NOR/SRAM */
#define STM32L5_FSMC_BASE34     0x80000000    /* 0x80000000-0x8fffffff: 512Mb FSMC bank3 / QSPI  block */
#  define STM32L5_FMC_BANK3     0x80000000    /* 0x80000000-0x8fffffff:   256Mb NAND FLASH */
#  define STM32L5_OCTOSPI1_BANK 0x90000000    /* 0x90000000-0x9fffffff:   256Mb QUADSPI */
#define STM32L5_CORTEX_BASE     0xE0000000    /* 0xe0000000-0xffffffff: 512Mb Cortex-M4 block */

#define STM32L5_REGION_MASK     0xF0000000
#define STM32L5_IS_SRAM(a)      ((((uint32_t)(a)) & STM32L5_REGION_MASK) == STM32L5_SRAM_BASE)
#define STM32L5_IS_EXTSRAM(a)   ((((uint32_t)(a)) & STM32L5_REGION_MASK) == STM32L5_FMC_BANK1)

/* Code Base Addresses ******************************************************/

#define STM32L5_BOOT_BASE      0x00000000     /* 0x00000000-0x000fffff: Aliased boot memory */
#define STM32L5_FLASH_BASE     0x08000000     /* 0x08000000-0x0807ffff: FLASH memory */
#define STM32L5_SRAM1_BASE     0x20000000     /* 0x20000000-0x2002ffff: 192k SRAM1 */
#define STM32L5_SRAM2_BASE     0x20030000     /* 0x20030000-0x2003ffff:  64k SRAM2 */

/* System Memory Addresses **************************************************/

#define STM32L5_SYSMEM_UID     0x0BFA0590     /* The 96-bit unique device identifier */
#define STM32L5_SYSMEM_FSIZE   0x0BFA05E0     /* Size of Flash memory in Kbytes. */
#define STM32L5_SYSMEM_PACKAGE 0x0BFA0500     /* Indicates the device's package type. */

/* Peripheral Base Addresses ************************************************/

#define STM32L5_APB1_BASE      0x40000000     /* 0x40000000-0x4000dfff: APB1 */
#define STM32L5_APB2_BASE      0x40010000     /* 0x40010000-0x400167ff: APB2 */
#define STM32L5_AHB1_BASE      0x40020000     /* 0x40020000-0x400333ff: AHB1 */
#define STM32L5_AHB2_BASE      0x42020000     /* 0x42020000-0x420c83ff: AHB2 */
#define STM32L5_AHB3_BASE      0x44020000     /* 0x44020000-0x440213ff: AHB3 */

/* APB1 Base Addresses ******************************************************/

#define STM32L5_UCPD1_BASE      0x4000DC00
#define STM32L5_USB_SRAM_BASE   0x4000D800
#define STM32L5_USB_FS_BASE     0x4000D400
#define STM32L5_FDCAN_RAM_BASE  0x4000AC00
#define STM32L5_FDCAN1_BASE     0x4000A400
#define STM32L5_LPTIM3_BASE     0x40009800
#define STM32L5_LPTIM2_BASE     0x40009400
#define STM32L5_I2C4_BASE       0x40008400
#define STM32L5_LPUART1_BASE    0x40008000
#define STM32L5_LPTIM1_BASE     0x40007C00
#define STM32L5_OPAMP_BASE      0x40007800
#define STM32L5_DAC_BASE        0x40007400
#define STM32L5_PWR_BASE        0x40007000
#define STM32L5_CRS_BASE        0x40006000
#define STM32L5_I2C3_BASE       0x40005C00
#define STM32L5_I2C2_BASE       0x40005800
#define STM32L5_I2C1_BASE       0x40005400
#define STM32L5_UART5_BASE      0x40005000
#define STM32L5_UART4_BASE      0x40004C00
#define STM32L5_USART3_BASE     0x40004800
#define STM32L5_USART2_BASE     0x40004400
#define STM32L5_SPI3_BASE       0x40003C00
#define STM32L5_SPI2_BASE       0x40003800
#define STM32L5_TAMP_BASE       0x40003400
#define STM32L5_IWDG_BASE       0x40003000
#define STM32L5_WWDG_BASE       0x40002C00
#define STM32L5_RTC_BASE        0x40002800
#define STM32L5_TIM7_BASE       0x40001400
#define STM32L5_TIM6_BASE       0x40001000
#define STM32L5_TIM5_BASE       0x40000C00
#define STM32L5_TIM4_BASE       0x40000800
#define STM32L5_TIM3_BASE       0x40000400
#define STM32L5_TIM2_BASE       0x40000000

/* APB2 Base Addresses ******************************************************/

#define STM32L5_DFSDM1_BASE     0x40016000
#define STM32L5_SAI2_BASE       0x40015800
#define STM32L5_SAI1_BASE       0x40015400
#define STM32L5_TIM17_BASE      0x40014800
#define STM32L5_TIM16_BASE      0x40014400
#define STM32L5_TIM15_BASE      0x40014000
#define STM32L5_USART1_BASE     0x40013800
#define STM32L5_TIM8_BASE       0x40013400
#define STM32L5_SPI1_BASE       0x40013000
#define STM32L5_TIM1_BASE       0x40012C00
#define STM32L5_COMP_BASE       0x40010200
#define STM32L5_VREFBUF_BASE    0x40010100
#define STM32L5_SYSCFG_BASE     0x40010000

/* AHB1 Base Addresses ******************************************************/

#define STM32L5_GTZC_BASE       0x40032400
#define STM32L5_ICACHE_BASE     0x40030400
#define STM32L5_EXTI_BASE       0x4002F400
#define STM32L5_TSC_BASE        0x40024000
#define STM32L5_CRC_BASE        0x40023000
#define STM32L5_FLASHIF_BASE    0x40022000
#define STM32L5_RCC_BASE        0x40021000
#define STM32L5_DMAMUX1_BASE    0x40020800
#define STM32L5_DMA2_BASE       0x40020400
#define STM32L5_DMA1_BASE       0x40020000

/* AHB2 Base Addresses ******************************************************/

#define STM32L5_SDMMC1_BASE     0x420C8000
#define STM32L5_OTFDEC1_BASE    0x420C5000
#define STM32L5_PKA_BASE        0x420C2000
#define STM32L5_RNG_BASE        0x420C0800
#define STM32L5_HASH_BASE       0x420C0400
#define STM32L5_AES_BASE        0x420C0000
#define STM32L5_ADC_BASE        0x42028000
#define STM32L5_GPIOH_BASE      0x42021C00
#define STM32L5_GPIOG_BASE      0x42021800
#define STM32L5_GPIOF_BASE      0x42021400
#define STM32L5_GPIOE_BASE      0x42021000
#define STM32L5_GPIOD_BASE      0x42020c00
#define STM32L5_GPIOC_BASE      0x42020800
#define STM32L5_GPIOB_BASE      0x42020400
#define STM32L5_GPIOA_BASE      0x42020000

/* AHB2 Base Addresses ******************************************************/

#define STM32L5_OCTOSPI1_BASE   0x44021000
#define STM32L5_FMC_BASE        0x44020000

#endif /* __ARCH_ARM_SRC_STM32L5_HARDWARE_STM32L5_MEMORYMAP_H */
