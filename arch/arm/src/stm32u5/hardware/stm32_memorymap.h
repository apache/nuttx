/****************************************************************************
 * arch/arm/src/stm32u5/hardware/stm32_memorymap.h
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

#ifndef __ARCH_ARM_SRC_STM32U5_STM32_MEMORYMAP_H
#define __ARCH_ARM_SRC_STM32U5_STM32_MEMORYMAP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* STM32U5XXX Address Blocks ************************************************/

#define STM32_CODE_BASE         0x00000000  /* 0x00000000-0x1fffffff: 512Mb code block */
#define STM32_SRAM_BASE         0x20000000  /* 0x20000000-0x3fffffff: 512Mb sram block */
#define STM32_PERIPH_BASE       0x40000000  /* 0x40000000-0x5fffffff: 512Mb peripheral block */
#define STM32_FSMC_BASE12       0x60000000  /* 0x60000000-0x7fffffff: 512Mb FSMC bank1&2 block */
#  define STM32_FMC_BANK1       0x60000000  /* 0x60000000-0x6fffffff:   256Mb NOR/SRAM */
#define STM32_FSMC_BASE34       0x80000000  /* 0x80000000-0x8fffffff: 512Mb FSMC bank3 / QSPI  block */
#  define STM32_FMC_BANK3       0x80000000  /* 0x80000000-0x8fffffff:   256Mb NAND FLASH */
#  define STM32_OCTOSPI1_BANK   0x90000000  /* 0x90000000-0x9fffffff:   256Mb QUADSPI */
#define STM32_CORTEX_BASE       0xE0000000  /* 0xe0000000-0xffffffff: 512Mb Cortex-M4 block */

#define STM32_REGION_MASK       0xF0000000
#define STM32_IS_SRAM(a)        ((((uint32_t)(a)) & STM32U5_REGION_MASK) == STM32U5_SRAM_BASE)
#define STM32_IS_EXTSRAM(a)     ((((uint32_t)(a)) & STM32U5_REGION_MASK) == STM32U5_FMC_BANK1)

/* Code Base Addresses ******************************************************/

#define STM32_BOOT_BASE         0x00000000  /* 0x00000000-0x000fffff: Aliased boot memory */
#define STM32_FLASH_BASE        0x08000000  /* 0x08000000-0x081fffff: FLASH memory */
#define STM32_SRAM1_BASE        0x20000000  /* 0x20000000-0x2002ffff: 192k SRAM1 */
#define STM32_SRAM2_BASE        0x20030000  /* 0x20030000-0x2003ffff:  64k SRAM2 */
#define STM32_SRAM3_BASE        0x20040000  /* 0x20040000-0x200bffff: 512k SRAM3 */

/* System Memory Addresses **************************************************/

#define STM32_SYSMEM_UID        0x0BFA0700  /* The 96-bit unique device identifier */
#define STM32_SYSMEM_FSIZE      0x0BFA07A0  /* Size of Flash memory in Kbytes. */
#define STM32_SYSMEM_PACKAGE    0x0BFA0500  /* Indicates the device's package type. */

/* Peripheral Base Addresses ************************************************/

#define STM32_APB1_BASE         0x40000000  /* 0x40000000-0x40012bff: APB1 */
#define STM32_APB2_BASE         0x40012c00  /* 0x40012c00-0x4001ffff: APB2 */
#define STM32_AHB1_BASE         0x40020000  /* 0x40020000-0x4201ffff: AHB1 */
#define STM32_AHB2_BASE         0x42020000  /* 0x42020000-0x460003ff: AHB2 */
#define STM32_APB3_BASE         0x46000400  /* 0x46000400-0x4601ffff: APB2 */
#define STM32_AHB3_BASE         0x46020000  /* 0x46020000-0x4fffffff: AHB3 */

/* APB1 Base Addresses ******************************************************/

#define STM32_TIM2_BASE         0x40000000
#define STM32_TIM3_BASE         0x40000400
#define STM32_TIM4_BASE         0x40000800
#define STM32_TIM5_BASE         0x40000c00
#define STM32_TIM6_BASE         0x40001000
#define STM32_TIM7_BASE         0x40001400
#define STM32_WWDG_BASE         0x40002c00
#define STM32_IWDG_BASE         0x40003000
#define STM32_SPI2_BASE         0x40003800
#define STM32_USART2_BASE       0x40004400
#define STM32_USART3_BASE       0x40004800
#define STM32_UART4_BASE        0x40004c00
#define STM32_UART5_BASE        0x40005000
#define STM32_I2C1_BASE         0x40005400
#define STM32_I2C2_BASE         0x40005800
#define STM32_CRS_BASE          0x40006000
#define STM32_I2C4_BASE         0x40008400
#define STM32_LPTIM2_BASE       0x40009400
#define STM32_FDCAN1_BASE       0x4000a400
#define STM32_FDCAN_RAM_BASE    0x4000ac00
#define STM32_UCPD1_BASE        0x4000dc00

/* APB2 Base Addresses ******************************************************/

#define STM32_TIM1_BASE         0x40012c00
#define STM32_SPI1_BASE         0x40013000
#define STM32_TIM8_BASE         0x40013400
#define STM32_USART1_BASE       0x40013800
#define STM32_TIM15_BASE        0x40014000
#define STM32_TIM16_BASE        0x40014400
#define STM32_TIM17_BASE        0x40014800
#define STM32_SAI1_BASE         0x40015400
#define STM32_SAI2_BASE         0x40015800

/* AHB1 Base Addresses ******************************************************/

#define STM32_GPDMA1_BASE       0x40020000
#define STM32_CORDIC_BASE       0x40021000
#define STM32_FMAC_BASE         0x40021400
#define STM32_FLASHIF_BASE      0x40022000
#define STM32_CRC_BASE          0x40023000
#define STM32_TSC_BASE          0x40024000
#define STM32_MDF1_BASE         0x40025000
#define STM32_RAMCFG_BASE       0x40026000
#define STM32_DMA2D_BASE        0x4002b000
#define STM32_ICACHE_BASE       0x40030400
#define STM32_DCACHE1_BASE      0x40031400
#define STM32_GTZC1_TZSC_BASE   0x40032400
#define STM32_GTZC1_TZIC_BASE   0x40032800
#define STM32_GTZC1_MPCBB1_BASE 0x40032c00
#define STM32_GTZC1_MPCBB2_BASE 0x40033000
#define STM32_GTZC1_MPCBB3_BASE 0x40033400
#define STM32_BKPSRAM_BASE      0x40036400

/* AHB2 Base Addresses ******************************************************/

#define STM32_GPIOA_BASE        0x42020000
#define STM32_GPIOB_BASE        0x42020400
#define STM32_GPIOC_BASE        0x42020800
#define STM32_GPIOD_BASE        0x42020c00
#define STM32_GPIOE_BASE        0x42021000
#define STM32_GPIOF_BASE        0x42021400
#define STM32_GPIOG_BASE        0x42021800
#define STM32_GPIOH_BASE        0x42021c00
#define STM32_GPIOI_BASE        0x42022000
#define STM32_ADC1_BASE         0x42028000
#define STM32_DCMI_BASE         0x4202c000
#define STM32_PSSI_BASE         0x4202c400
#define STM32_OTG_FS_BASE       0x42040000
#define STM32_AES_BASE          0x420c0000
#define STM32_HASH_BASE         0x420c0400
#define STM32_RNG_BASE          0x420c0800
#define STM32_SAES_BASE         0x420c0c00
#define STM32_PKA_BASE          0x420c2000
#define STM32_OCTOSPIM_BASE     0x420c4000
#define STM32_OTFDEC1_BASE      0x420c5000
#define STM32_OTFDEC2_BASE      0x420c5400
#define STM32_SDMMC1_BASE       0x420c8000
#define STM32_DLYBSD1_BASE      0x420c8400
#define STM32_DLYBSD2_BASE      0x420c8800
#define STM32_SDMMC2_BASE       0x420c8c00
#define STM32_DLYBOS1_BASE      0x420cf000
#define STM32_DLYBOS2_BASE      0x420cf400
#define STM32_FSMC_BASE         0x420d0400
#define STM32_OCTOSPI1_BASE     0x420d1400
#define STM32_OCTOSPI2_BASE     0x420d2400

/* APB3 Base Addresses ******************************************************/

#define STM32_SYSCFG_BASE       0x46000400
#define STM32_SPI3_BASE         0x46002000
#define STM32_LPUART1_BASE      0x46002400
#define STM32_I2C3_BASE         0x46002c00
#define STM32_LPTIM1_BASE       0x46004400
#define STM32_LPTIM3_BASE       0x46004800
#define STM32_LPTIM4_BASE       0x46004c00
#define STM32_OPAMP_BASE        0x46005000
#define STM32_COMP_BASE         0x46005400
#define STM32_VREFBUF_BASE      0x46007400
#define STM32_RTC_BASE          0x46007800
#define STM32_TAMP_BASE         0x46007c00

/* AHB3 Base Addresses ******************************************************/

#define STM32_LPGPIO1_BASE      0x46020000
#define STM32_PWR_BASE          0x46020800
#define STM32_RCC_BASE          0x46020c00
#define STM32_ADC4_BASE         0x46021000
#define STM32_DAC1_BASE         0x46021800
#define STM32_EXTI_BASE         0x46022000
#define STM32_GTZC2_TZSC_BASE   0x46023000
#define STM32_GTZC2_TZIC_BASE   0x46023400
#define STM32_GTZC2_MPCBB4_BASE 0x46023800
#define STM32_ADF1_BASE         0x46024000
#define STM32_LPDMA1_BASE       0x46025000

#endif /* __ARCH_ARM_SRC_STM32U5_STM32_MEMORYMAP_H */
