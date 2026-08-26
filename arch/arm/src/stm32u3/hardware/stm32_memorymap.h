/****************************************************************************
 * arch/arm/src/stm32u3/hardware/stm32_memorymap.h
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

#ifndef __ARCH_ARM_SRC_STM32U3_HARDWARE_STM32_MEMORYMAP_H
#define __ARCH_ARM_SRC_STM32U3_HARDWARE_STM32_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* STM32U3 address blocks ***************************************************/

#define STM32_CODE_BASE            0x00000000
#define STM32_SRAM_BASE            0x20000000
#define STM32_PERIPH_BASE          0x40000000
#define STM32_EXTRAM_BASE          0x90000000
#define STM32_OCTOSPI1_BANK        0x90000000
#define STM32_EPPB_BASE            0xe0040000
#define STM32_CORTEX_BASE          0xe0000000

#define STM32_REGION_MASK          0xf0000000
#define STM32_IS_SRAM(a) \
  ((((uint32_t)(a)) & STM32_REGION_MASK) == STM32_SRAM_BASE)

/* Code and SRAM base addresses *********************************************/

#define STM32_BOOT_BASE            0x00000000
#define STM32_FLASH_BASE           0x08000000
#define STM32_SYSMEM_BASE          0x0bf80000
#define STM32_SYSTEM_FLASH_BASE    0x0bf80000
#define STM32_FLASH_OTP_BASE       0x0bfa0000
#define STM32_FLASH_ENGY_BASE      0x0bfa0500
#define STM32_SRAM1_BASE           0x20000000
#define STM32_SRAM2_BASE           0x20030000
#define STM32_SRAM3_BASE           0x20040000
#define STM32_SRAM4_BASE           0x20090000

/* System memory addresses **************************************************/

#define STM32_SYSMEM_UID           0x0bfa0700
#define STM32_SYSMEM_FSIZE         0x0bfa07a0
#define STM32_SYSMEM_PACKAGE       0x0bfa0500
#define STM32_SYSMEM_UID64         0x0bfa0a00
#define STM32_PACKAGE_BASE         0x0bfa0500
#define STM32_UID_BASE             0x0bfa0700
#define STM32_FLASHSIZE_BASE       0x0bfa07a0
#define STM32_UID64_BASE           0x0bfa0a00

/* Non-secure peripheral bus base addresses *********************************/

#define STM32_APB1_BASE            0x40000000
#define STM32_APB2_BASE            0x40010000
#define STM32_AHB1_BASE            0x40020000
#define STM32_APB3_BASE            0x40040000
#define STM32_AHB2_BASE            0x42020000

/* APB1 non-secure peripheral base addresses ********************************/

#define STM32_TIM2_BASE            0x40000000
#define STM32_TIM3_BASE            0x40000400
#define STM32_TIM4_BASE            0x40000800
#define STM32_TIM6_BASE            0x40001000
#define STM32_TIM7_BASE            0x40001400
#define STM32_SPI3_BASE            0x40002000
#define STM32_SPI4_BASE            0x40002400
#define STM32_WWDG_BASE            0x40002c00
#define STM32_IWDG_BASE            0x40003000
#define STM32_SPI2_BASE            0x40003800
#define STM32_USART2_BASE          0x40004400
#define STM32_USART3_BASE          0x40004800
#define STM32_UART4_BASE           0x40004c00
#define STM32_UART5_BASE           0x40005000
#define STM32_I2C1_BASE            0x40005400
#define STM32_I2C2_BASE            0x40005800
#define STM32_I3C1_BASE            0x40005c00
#define STM32_CRS_BASE             0x40006000
#define STM32_OPAMP1_BASE          0x40007000
#define STM32_OPAMP2_BASE          0x40007010
#define STM32_OPAMP12_COMMON_BASE  0x40007000
#define STM32_VREFBUF_BASE         0x40007400
#define STM32_RTC_BASE             0x40007800
#define STM32_TAMP_BASE            0x40007c00
#define STM32_I2C4_BASE            0x40008400
#define STM32_LPTIM2_BASE          0x40009400
#define STM32_FDCAN1_BASE          0x4000a400
#define STM32_FDCAN_CONFIG_BASE    0x4000a500
#define STM32_FDCAN2_BASE          0x4000a800
#define STM32_FDCAN_RAM_BASE       0x4000ac00
#define STM32_SRAMCAN_BASE         0x4000ac00

/* APB2 non-secure peripheral base addresses ********************************/

#define STM32_TIM1_BASE            0x40012c00
#define STM32_SPI1_BASE            0x40013000
#define STM32_TIM8_BASE            0x40013400
#define STM32_USART1_BASE          0x40013800
#define STM32_TIM12_BASE           0x40013c00
#define STM32_TIM15_BASE           0x40014000
#define STM32_TIM16_BASE           0x40014400
#define STM32_TIM17_BASE           0x40014800
#define STM32_SAI1_BASE            0x40015400
#define STM32_SAI1_BLOCK_A_BASE    0x40015404
#define STM32_SAI1_BLOCK_B_BASE    0x40015424
#define STM32_USB_DRD_FS_BASE      0x40016000
#define STM32_USB_DRD_FS_RAM_BASE  0x40016400
#define STM32_USB_DRD_BASE         0x40016000
#define STM32_USB_DRD_RAM_BASE     0x40016400
#define STM32_USB_FS_BASE          0x40016000
#define STM32_USB_FS_RAM_BASE      0x40016400
#define STM32_I3C2_BASE            0x40016c00

/* AHB1 non-secure peripheral base addresses ********************************/

/* DMA1 is the GPDMA1 instance.  All STM32 DMA instances of the
 * GPDMA/LPDMA IP family are named DMAx.
 */

#define STM32_DMA1_BASE            0x40020000
#define STM32_FLASHIF_BASE         0x40022000
#define STM32_FLASH_R_BASE         0x40022000
#define STM32_CRC_BASE             0x40023000
#define STM32_TSC_BASE             0x40024000
#define STM32_RAMCFG_BASE          0x40026000
#define STM32_RAMCFG_SRAM1_BASE    0x40026000
#define STM32_RAMCFG_SRAM2_BASE    0x40026040
#define STM32_RAMCFG_SRAM3_BASE    0x40026080
#define STM32_HSP1_BASE            0x4002c000
#define STM32_ICACHE_BASE          0x40030400
#define STM32_PWR_BASE             0x40030800
#define STM32_RCC_BASE             0x40030c00
#define STM32_EXTI_BASE            0x40032000
#define STM32_GTZC_TZSC1_BASE      0x40032400
#define STM32_GTZC_MPCBB1_BASE     0x40032c00
#define STM32_GTZC_MPCBB2_BASE     0x40033000
#define STM32_GTZC_MPCBB3_BASE     0x40033400
#define STM32_GTZC_MPCBB4_BASE     0x40033800
#define STM32_ADF1_BASE            0x40034000
#define STM32_ADF1_FILTER0_BASE    0x40034080

/* APB3 non-secure peripheral base addresses ********************************/

#define STM32_SYSCFG_BASE          0x40040400
#define STM32_LPUART1_BASE         0x40042400
#define STM32_I2C3_BASE            0x40042800
#define STM32_LPTIM1_BASE          0x40044400
#define STM32_LPTIM3_BASE          0x40044800
#define STM32_LPTIM4_BASE          0x40044c00
#define STM32_COMP1_BASE           0x40045400
#define STM32_COMP2_BASE           0x40045404
#define STM32_COMP12_COMMON_BASE   0x40045400

/* AHB2 non-secure peripheral base addresses ********************************/

#define STM32_GPIOA_BASE           0x42020000
#define STM32_GPIOB_BASE           0x42020400
#define STM32_GPIOC_BASE           0x42020800
#define STM32_GPIOD_BASE           0x42020c00
#define STM32_GPIOE_BASE           0x42021000
#define STM32_GPIOF_BASE           0x42021400
#define STM32_GPIOG_BASE           0x42021800
#define STM32_GPIOH_BASE           0x42021c00
#define STM32_ADC1_BASE            0x42028000
#define STM32_ADC2_BASE            0x42028100
#define STM32_ADC12_COMMON_BASE    0x42028300
#define STM32_ADC12_BASE           0x42028000
#define STM32_DAC1_BASE            0x42028400
#define STM32_AES_BASE             0x420c0000
#define STM32_HASH_BASE            0x420c0400
#define STM32_HASH_DIGEST_BASE     0x420c0710
#define STM32_RNG_BASE             0x420c0800
#define STM32_SAES_BASE            0x420c0c00
#define STM32_PKA_BASE             0x420c2000
#define STM32_PKA_RAM_BASE         0x420c2400
#define STM32_CCB_BASE             0x420c7c00
#define STM32_SDMMC1_BASE          0x420c8000
#define STM32_DLYB_SDMMC1_BASE     0x420c8400
#define STM32_DLYB_OCTOSPI1_BASE   0x420cf000
#define STM32_DLYB_OSPI1_BASE      0x420cf000
#define STM32_OCTOSPI1_BASE        0x420d1400
#define STM32_OCTOSPI1_R_BASE      0x420d1400

/* External private peripheral bus ******************************************/

#define STM32_DBGMCU_BASE          0xe0044000

#endif /* __ARCH_ARM_SRC_STM32U3_HARDWARE_STM32_MEMORYMAP_H */
