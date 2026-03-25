/****************************************************************************
 * arch/arm/src/ht32f491x3/hardware/ht32f491x3_memorymap.h
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

#ifndef __ARCH_ARM_SRC_HT32F491X3_HARDWARE_HT32F491X3_MEMORYMAP_H
#define __ARCH_ARM_SRC_HT32F491X3_HARDWARE_HT32F491X3_MEMORYMAP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Address blocks ***********************************************************/

#define HT32_CODE_BASE        0x00000000
#define HT32_SRAM_BASE        0x20000000
#define HT32_PERIPH_BASE      0x40000000
#define HT32_XMC_MEM_BASE     0x60000000
#define HT32_XMC_BANK1        HT32_XMC_MEM_BASE
#define HT32_XMC_REG_BASE     0xa0000000
#define HT32_CORTEX_BASE      0xe0000000

/* Code region **************************************************************/

#define HT32_BOOT_BASE        0x00000000
#define HT32_FLASH_BASE       0x08000000
#define HT32_FLASHREG_BASE    0x40023c00
#define HT32_SYSMEM_BASE      0x1fffa400
#define HT32_USERDATA_BASE    0x1ffff800

/* SRAM region **************************************************************/

#define HT32_SRAMBB_BASE      0x22000000

/* Peripheral region ********************************************************/

#define HT32_APB1_BASE        0x40000000
#define HT32_APB2_BASE        0x40010000
#define HT32_AHB1_BASE        0x40020000
#define HT32_AHB2_BASE        0x50000000

/* APB1 */

#define HT32_TMR2_BASE        0x40000000
#define HT32_TMR3_BASE        0x40000400
#define HT32_TMR4_BASE        0x40000800
#define HT32_TMR6_BASE        0x40001000
#define HT32_TMR7_BASE        0x40001400
#define HT32_TMR12_BASE       0x40001800
#define HT32_TMR13_BASE       0x40001c00
#define HT32_TMR14_BASE       0x40002000
#define HT32_ERTC_BASE        0x40002800
#define HT32_WWDT_BASE        0x40002c00
#define HT32_WDT_BASE         0x40003000
#define HT32_SPI2_BASE        0x40003800
#define HT32_SPI3_BASE        0x40003c00
#define HT32_USART2_BASE      0x40004400
#define HT32_USART3_BASE      0x40004800
#define HT32_USART4_BASE      0x40004c00
#define HT32_USART5_BASE      0x40005000
#define HT32_I2C1_BASE        0x40005400
#define HT32_I2C2_BASE        0x40005800
#define HT32_I2C3_BASE        0x40005c00
#define HT32_CAN1_BASE        0x40006400
#define HT32_CAN2_BASE        0x40006800
#define HT32_PWC_BASE         0x40007000
#define HT32_DAC_BASE         0x40007400
#define HT32_USART7_BASE      0x40007800
#define HT32_USART8_BASE      0x40007c00

/* APB2 */

#define HT32_TMR1_BASE        0x40010000
#define HT32_USART1_BASE      0x40011000
#define HT32_USART6_BASE      0x40011400
#define HT32_ADC1_BASE        0x40012000
#define HT32_ADC_BASE         HT32_ADC1_BASE
#define HT32_ADCCOM_BASE      0x40012300
#define HT32_SPI1_BASE        0x40013000
#define HT32_SCFG_BASE        0x40013800
#define HT32_EXINT_BASE       0x40013c00
#define HT32_TMR9_BASE        0x40014000
#define HT32_TMR10_BASE       0x40014400
#define HT32_TMR11_BASE       0x40014800
#define HT32_ACC_BASE         0x40017400

/* AHB1 */

#define HT32_GPIOA_BASE       0x40020000
#define HT32_GPIOB_BASE       0x40020400
#define HT32_GPIOC_BASE       0x40020800
#define HT32_GPIOD_BASE       0x40020c00
#define HT32_GPIOE_BASE       0x40021000
#define HT32_GPIOF_BASE       0x40021400
#define HT32_CRC_BASE         0x40023000
#define HT32_CRM_BASE         0x40023800
#define HT32_FLASHIF_BASE     0x40023c00
#define HT32_DMA1_BASE        0x40026000
#define HT32_DMA2_BASE        0x40026400

/* AHB2 */

#define HT32_OTGFS1_BASE      0x50000000
#define HT32_OTGFS_BASE       HT32_OTGFS1_BASE

/* Cortex-M4 system control space *******************************************/

#define HT32_SCS_BASE         0xe000e000

#endif /* __ARCH_ARM_SRC_HT32F491X3_HARDWARE_HT32F491X3_MEMORYMAP_H */
