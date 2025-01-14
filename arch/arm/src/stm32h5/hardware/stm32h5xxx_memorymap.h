/****************************************************************************
 * arch/arm/src/stm32h5/hardware/stm32h5xxx_memorymap.h
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

#ifndef __ARCH_ARM_SRC_STM32H5_HARDWARE_STM32H5XXX_MEMORYMAP_H
#define __ARCH_ARM_SRC_STM32H5_HARDWARE_STM32H5XXX_MEMORYMAP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* STM32H5XXX Address Blocks ************************************************/

#define STM32_CODE_BASE       0x00000000    /* 0x00000000-0x1fffffff: 512Mb code block */
#define STM32_SRAM_BASE       0x20000000    /* 0x20000000-0x3fffffff: 512Mb sram block (48k to 256k) */
#define STM32_PERIPH_BASE     0x40000000    /* 0x40000000-0x5fffffff: 512Mb peripheral block */
#define STM32_FSMC_BASE12     0x60000000    /* 0x60000000-0x7fffffff: 512Mb FSMC bank1&2 block */
#define STM32_FMC_BANK1       0x60000000    /* 0x60000000-0x6fffffff: 256Mb NOR/SRAM */
#define STM32_FSMC_BASE34     0x80000000    /* 0x80000000-0x8fffffff: 512Mb FSMC bank3 / QSPI  block */
#define STM32_FMC_BANK3       0x80000000    /* 0x80000000-0x8fffffff: 256Mb NAND FLASH */
#define STM32_OCTOSPI1_BANK   0x90000000    /* 0x90000000-0x9fffffff: 256Mb QUADSPI */
#define STM32_CORTEX_BASE     0xE0000000    /* 0xe0000000-0xffffffff: 512Mb Cortex-M4 block */

#define STM32_REGION_MASK     0xF0000000
#define STM32_IS_SRAM(a)      ((((uint32_t)(a)) & STM32_REGION_MASK) == STM32_SRAM_BASE)
#define STM32_IS_EXTSRAM(a)   ((((uint32_t)(a)) & STM32_REGION_MASK) == STM32_FMC_BANK1)

/* Code Base Addresses ******************************************************/

#define STM32_BOOT_BASE      0x00000000     /* 0x00000000-0x000fffff: Aliased boot memory */
#define STM32_FLASH_BASE     0x08000000     /* 0x08000000-0x0807ffff: FLASH memory */
#define STM32_SRAM1_BASE     0x20000000     /* 0x20000000-0x2002ffff: 192k SRAM1 */
#define STM32_SRAM2_BASE     0x20040000     /* 0x20040000-0x2004ffff:  64k SRAM2 */
#define STM32_SRAM3_BASE     0x20050000     /* 0x20050000-0x2008ffff: 320k SRAM3 */
                                            /* 0x20090000-0x2009ffff: Reserved for ECC */

/* System Memory Addresses **************************************************/

#define STM32_SYSMEM_UID     0x08FFF800     /* The 96-bit unique device identifier */
#define STM32_SYSMEM_FSIZE   0x08FFF80C     /* Size of Flash memory in Kbytes. */
#define STM32_SYSMEM_PACKAGE 0x08FFF80E     /* Indicates the device's package type. */

/* Peripheral Base Addresses ************************************************/

#define STM32_APB1_BASE      0x40000000     /* 0x40000000-0x4000fbff: APB1 */
#define STM32_APB2_BASE      0x40012C00     /* 0x40012c00-0x40016bff: APB2 */
#define STM32_AHB1_BASE      0x40020000     /* 0x40020000-0x400373ff: AHB1 */
#define STM32_AHB2_BASE      0x42020000     /* 0x42020000-0x420c3fff: AHB2 */
#define STM32_APB3_BASE      0x44000400     /* 0x44000400-0x44007fff: APB3 */
#define STM32_AHB3_BASE      0x44020800     /* 0x44020800-0x440243ff: AHB3 */
#define STM32_AHB4_BASE      0x46005000     /* 0x46005000-0x470017ff: AHB4 */

/* APB1 Base Addresses ******************************************************/

#define STM32_UCPD1_BASE      0x4000dc00
#define STM32_FDCAN_SRAM_BASE 0x4000ac00
#define STM32_FDCAN2_BASE     0x4000a800
#define STM32_FDCAN1_BASE     0x4000a400
#define STM32_LPTIM2          0x40009400
#define STM32_DTS_BASE        0x40008c00
#define STM32_UART12_BASE     0x40008400
#define STM32_UART9_BASE      0x40008000
#define STM32_UART8_BASE      0x40007c00
#define STM32_UART7_BASE      0x40007800
#define STM32_HDMICEC_BASE    0x40007000
#define STM32_USART11_BASE    0x40006c00
#define STM32_USART10_BASE    0x40006800
#define STM32_USART6_BASE     0x40006400
#define STM32_CRS_BASE        0x40006000
#define STM32_I3C1_BASE       0x40005c00
#define STM32_I2C2_BASE       0x40005800
#define STM32_I2C1_BASE       0x40005400
#define STM32_UART5_BASE      0x40005000
#define STM32_UART4_BASE      0x40004c00
#define STM32_USART3_BASE     0x40004800
#define STM32_USART2_BASE     0x40004400
#define STM32_SPI3_BASE       0x40003c00
#define STM32_SPI2_BASE       0x40003800
#define STM32_IWDG_BASE       0x40003000
#define STM32_WWDG_BASE       0x40002c00
#define STM32_TIM14_BASE      0x40002000
#define STM32_TIM13_BASE      0x40001c00
#define STM32_TIM12_BASE      0x40001800
#define STM32_TIM7_BASE       0x40001400
#define STM32_TIM6_BASE       0x40001000
#define STM32_TIM5_BASE       0x40000c00
#define STM32_TIM4_BASE       0x40000800
#define STM32_TIM3_BASE       0x40000400
#define STM32_TIM2_BASE       0x40000000

/* APB2 Base Addresses ******************************************************/

#define STM32_USB_FS_RAM_BASE 0x40016400
#define STM32_USB_FS_BASE     0x40016000
#define STM32_SAI2_BASE       0x40015800
#define STM32_SAI1_BASE       0x40015400
#define STM32_SPI6_BASE       0x40015000
#define STM32_SPI4_BASE       0x40014c00
#define STM32_TIM17_BASE      0x40014800
#define STM32_TIM16_BASE      0x40014400
#define STM32_TIM15_BASE      0x40014000
#define STM32_USART1_BASE     0x40013800
#define STM32_TIM8_BASE       0x40013400
#define STM32_SPI1_BASE       0x40013000
#define STM32_TIM1_BASE       0x40012C00

/* AHB1 Base Addresses ******************************************************/

#define STM32_MPC_WM_BKPRAM_BASE 0x40036400
#define STM32_GTZC1_BASE         0x40032400
#define STM32_DCACHE_BASE        0x40031400
#define STM32_ICACHE_BASE        0x40030400
#define STM32_EMAC_BASE          0x40028000
#define STM32_RAMCFG_BASE        0x40026000
#define STM32_FMAC_BASE          0x40023c00
#define STM32_CORDIC_BASE        0x40023800
#define STM32_CRC_BASE           0x40023000
#define STM32_FLASHIF_BASE       0x40022000
#define STM32_DMA2_BASE          0x40021000
#define STM32_DMA1_BASE          0x40020000

/* AHB2 Base Addresses ******************************************************/

#define STM32_PKA_BASE        0x420c2000
#define STM32_SAES_BASE       0x420c0C00
#define STM32_RNG_BASE        0x420c0800
#define STM32_HASH_BASE       0x420c0400
#define STM32_AES_BASE        0x420c0000
#define STM32_PSSI_BASE       0x4202c400
#define STM32_DCMI_BASE       0x4202c000
#define STM32_DAC1_BASE       0x42028400
#define STM32_ADC12_BASE      0x42028000
#define STM32_GPIOI_BASE      0x42022000
#define STM32_GPIOH_BASE      0x42021c00
#define STM32_GPIOG_BASE      0x42021800
#define STM32_GPIOF_BASE      0x42021400
#define STM32_GPIOE_BASE      0x42021000
#define STM32_GPIOD_BASE      0x42020c00
#define STM32_GPIOC_BASE      0x42020800
#define STM32_GPIOB_BASE      0x42020400
#define STM32_GPIOA_BASE      0x42020000

/* APB3 Base Addresses ******************************************************/

#define STM32_TAMP_BASE       0x44007c00
#define STM32_RTC_BASE        0x44007800
#define STM32_VREFBUF_BASE    0x44007400
#define STM32_LPTIM6_BASE     0x44005400
#define STM32_LPTIM5_BASE     0x44005000
#define STM32_LPTIM4_BASE     0x44004c00
#define STM32_LPTIM3_BASE     0x44004800
#define STM32_LPTIM1_BASE     0x44004400
#define STM32_I2C4_BASE       0x44002c00
#define STM32_I2C3_BASE       0x44002800
#define STM32_LPUART1_BASE    0x44002400
#define STM32_SPI5_BASE       0x44002000
#define STM32_SBS_BASE        0x44000400

/* AHB3 Base Addresses ******************************************************/

#define STM32_DEBUG_BASE      0x44024000
#define STM32_EXTI_BASE       0x44022000
#define STM32_RCC_BASE        0x44020C00
#define STM32_PWR_BASE        0x44020800

/* AHB4 Base Addresses ******************************************************/

#define STM32_OCTOSPI1_BASE   0x47001400
#define STM32_FMC_BASE        0x47000400
#define STM32_DLYBOS1_BASE    0x4600f000
#define STM32_SDMMC2_BASE     0x46008c00
#define STM32_DLYBSD2_BASE    0x46008800
#define STM32_DLYBSD1_BASE    0x46008400
#define STM32_SDMMC1_BASE     0x46008000
#define STM32_OTFDEC1_BASE    0x46005000

#endif /* __ARCH_ARM_SRC_STM32H5_HARDWARE_STM32H5XXX_MEMORYMAP_H */
