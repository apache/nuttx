/****************************************************************************
 * arch/arm/src/stm32c5/hardware/stm32c5xx_memorymap.h
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

#ifndef __ARCH_ARM_SRC_STM32C5_HARDWARE_STM32C5XX_MEMORYMAP_H
#define __ARCH_ARM_SRC_STM32C5_HARDWARE_STM32C5XX_MEMORYMAP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* STM32C5 address blocks ***************************************************/

#define STM32_CODE_BASE         0x00000000
#define STM32_SRAM_BASE         0x20000000
#define STM32_PERIPH_BASE       0x40000000
#define STM32_CORTEX_BASE       0xe0000000

#define STM32_REGION_MASK       0xf0000000
#define STM32_IS_SRAM(a) \
  ((((uint32_t)(a)) & STM32_REGION_MASK) == STM32_SRAM_BASE)

/* Code and SRAM base addresses *********************************************/

#define STM32_BOOT_BASE         0x00000000
#define STM32_FLASH_BASE        0x08000000
#define STM32_FLASH_EXT_USER_BASE 0x08400000
#define STM32_FLASH_EXT_BASE    STM32_FLASH_EXT_USER_BASE
#define STM32_FLASH_OTP_BASE    0x08ffe000
#define STM32_FLASH_EDATA_BASE  0x09000000
#define STM32_FLASH_SYSTEM_BASE 0x0bf80000
#define STM32_SYSMEM_BASE       STM32_FLASH_SYSTEM_BASE
#define STM32_EXITHDPLIB_PFUNC_BASE 0x0bf883e0
#define STM32_SRAM1_BASE        0x20000000
#define STM32_SRAM2_BASE        0x20010000

/* System memory addresses **************************************************/

#define STM32_SYSMEM_UID        0x08fff800
#define STM32_SYSMEM_FSIZE      0x08fff80c
#define STM32_SYSMEM_PACKAGE    0x08fff80e
#define STM32_UID_BASE          STM32_SYSMEM_UID
#define STM32_FLASHSIZE_BASE    STM32_SYSMEM_FSIZE
#define STM32_PACKAGE_BASE      STM32_SYSMEM_PACKAGE

/* Peripheral bus base addresses ********************************************/

#define STM32_APB1_BASE         0x40000000
#define STM32_APB2_BASE         0x40010000
#define STM32_AHB1_BASE         0x40020000
#define STM32_AHB2_BASE         0x42020000
#define STM32_APB3_BASE         0x44000000
#define STM32_AHB3_BASE         0x44020000

/* APB1 peripheral base addresses *******************************************/

#define STM32_TIM2_BASE         0x40000000
#define STM32_TIM5_BASE         0x40000c00
#define STM32_TIM6_BASE         0x40001000
#define STM32_TIM7_BASE         0x40001400
#define STM32_TIM12_BASE        0x40001800
#define STM32_WWDG_BASE         0x40002c00
#define STM32_IWDG_BASE         0x40003000
#define STM32_SPI2_BASE         0x40003800
#define STM32_SPI3_BASE         0x40003c00
#define STM32_COMP1_BASE        0x40004000
#define STM32_USART2_BASE       0x40004400
#define STM32_USART3_BASE       0x40004800
#define STM32_UART4_BASE        0x40004c00
#define STM32_UART5_BASE        0x40005000
#define STM32_I2C1_BASE         0x40005400
#define STM32_I2C2_BASE         0x40005800
#define STM32_I3C1_BASE         0x40005c00
#define STM32_CRS_BASE          0x40006000
#define STM32_FDCAN1_BASE       0x4000a400
#define STM32_FDCAN_CONFIG_BASE 0x4000a500
#define STM32_FDCAN_RAM_BASE    0x4000ac00
#define STM32_SRAMCAN_BASE      STM32_FDCAN_RAM_BASE

/* APB2 peripheral base addresses *******************************************/

#define STM32_TIM1_BASE         0x40012c00
#define STM32_SPI1_BASE         0x40013000
#define STM32_TIM8_BASE         0x40013400
#define STM32_USART1_BASE       0x40013800
#define STM32_TIM15_BASE        0x40014000
#define STM32_TIM16_BASE        0x40014400
#define STM32_TIM17_BASE        0x40014800
#define STM32_USB_DRD_FS_BASE   0x40016000
#define STM32_USB_DRD_FS_RAM_BASE 0x40016400
#define STM32_USB_DRD_BASE      STM32_USB_DRD_FS_BASE
#define STM32_USB_DRD_RAM_BASE  STM32_USB_DRD_FS_RAM_BASE
#define STM32_USB_FS_BASE       STM32_USB_DRD_FS_BASE
#define STM32_USB_FS_RAM_BASE   STM32_USB_DRD_FS_RAM_BASE

/* AHB1 peripheral base addresses *******************************************/

#define STM32_LPDMA1_BASE       0x40020000
#define STM32_LPDMA1_CH0_BASE   0x40020050
#define STM32_LPDMA1_CH1_BASE   0x400200d0
#define STM32_LPDMA1_CH2_BASE   0x40020150
#define STM32_LPDMA1_CH3_BASE   0x400201d0
#define STM32_LPDMA1_CH4_BASE   0x40020250
#define STM32_LPDMA1_CH5_BASE   0x400202d0
#define STM32_LPDMA1_CH6_BASE   0x40020350
#define STM32_LPDMA1_CH7_BASE   0x400203d0
#define STM32_LPDMA2_BASE       0x40021000
#define STM32_LPDMA2_CH0_BASE   0x40021050
#define STM32_LPDMA2_CH1_BASE   0x400210d0
#define STM32_LPDMA2_CH2_BASE   0x40021150
#define STM32_LPDMA2_CH3_BASE   0x400211d0
#define STM32_FLASHIF_BASE      0x40022000
#define STM32_FLASH_R_BASE      STM32_FLASHIF_BASE
#define STM32_CRC_BASE          0x40023000
#define STM32_CORDIC_BASE       0x40023800
#define STM32_RAMCFG_BASE       0x40026000
#define STM32_RAMCFG_SRAM1_BASE 0x40026000
#define STM32_RAMCFG_SRAM2_BASE 0x40026040
#define STM32_ICACHE_BASE       0x40030400

/* AHB2 peripheral base addresses *******************************************/

#define STM32_GPIOA_BASE        0x42020000
#define STM32_GPIOB_BASE        0x42020400
#define STM32_GPIOC_BASE        0x42020800
#define STM32_GPIOD_BASE        0x42020c00
#define STM32_GPIOE_BASE        0x42021000
#define STM32_GPIOH_BASE        0x42021c00
#define STM32_ADC1_BASE         0x42028000
#define STM32_ADC2_BASE         0x42028100
#define STM32_ADC12_COMMON_BASE 0x42028300
#define STM32_ADC12_BASE        STM32_ADC1_BASE
#define STM32_DAC1_BASE         0x42028400
#define STM32_AES_BASE          0x420c0000
#define STM32_HASH_BASE         0x420c0400
#define STM32_RNG_BASE          0x420c0800

/* APB3 peripheral base addresses *******************************************/

#define STM32_SBS_BASE          0x44000400
#define STM32_LPUART1_BASE      0x44002400
#define STM32_LPTIM1_BASE       0x44004400
#define STM32_RTC_BASE          0x44007800
#define STM32_TAMP_BASE         0x44007c00

/* AHB3 peripheral base addresses *******************************************/

#define STM32_PWR_BASE          0x44020800
#define STM32_RCC_BASE          0x44020c00
#define STM32_EXTI_BASE         0x44022000
#define STM32_DBGMCU_BASE       0x44024000

#endif /* __ARCH_ARM_SRC_STM32C5_HARDWARE_STM32C5XX_MEMORYMAP_H */
