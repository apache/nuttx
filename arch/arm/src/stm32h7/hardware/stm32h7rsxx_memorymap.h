/****************************************************************************
 * arch/arm/src/stm32h7/hardware/stm32h7rsxx_memorymap.h
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

#ifndef __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32H7RSXX_MEMORYMAP_H
#define __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32H7RSXX_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* STM32H7RSXX Address Blocks ***********************************************/

#define STM32_CODE_BASE          0x00000000 /* 0x00000000-0x1fffffff: 512Mb CODE block */
#define STM32_SRAM_BASE          0x20000000 /* 0x20000000-0x3fffffff: 512Mb SRAM block */
#define STM32_PERIPH_BASE        0x40000000 /* 0x40000000-0x5fffffff: 512Mb peripheral blocks */
#define STM32_FMC_BANK1          0x60000000 /* 0x60000000-0x6fffffff: 256Mb FMC NOR/PSRAM/SRAM */
#define STM32_XSPI2_BASE         0x70000000 /* 0x70000000-0x7fffffff: 256Mb XSPI2 memory-mapped */
#define STM32_FMC_BANK3          0x80000000 /* 0x80000000-0x8fffffff: 256Mb FMC NAND FLASH */
#define STM32_XSPI1_BASE         0x90000000 /* 0x90000000-0x9fffffff: 256Mb XSPI1 memory-mapped */
#define STM32_FMC_BANK5          0xc0000000 /* 0xc0000000-0xcfffffff: 256Mb FMC SDRAM Bank 1 */
#define STM32_FMC_BANK6          0xd0000000 /* 0xd0000000-0xdfffffff: 256Mb FMC SDRAM Bank 2 */
#define STM32_CORTEX_BASE        0xe0000000 /* 0xe0000000-0xffffffff: 512Mb Cortex-M7 block */

#define STM32_REGION_MASK        0xff000000
#define STM32_IS_SRAM(a)         ((((uint32_t)(a)) & STM32_REGION_MASK) == STM32_SRAM_BASE)
#define STM32_IS_EXTSRAM(a)      ((((uint32_t)(a)) & STM32_REGION_MASK) == STM32_FMC_BANK1)

/* Code Base Addresses ******************************************************/

#define STM32_ITCM_BASE          0x00000000 /* 0x00000000-0x0002ffff ITCM SRAM */
#define STM32_FLASH_BASE         0x08000000 /* 0x08000000-0x0800ffff User FLASH */
#define STM32_OTP_BASE           0x08fff000 /* 0x08fff000-0x08fff3ff OTP area */
#define STM32_SYSMEM_UID         0x08fff800 /* Unique device ID */
#define STM32_PACKAGE_BASE       0x08fff80c /* Package data */
#define STM32_SYSTEMFLASH_BASE   0x1ff00000 /* System FLASH */

/* SRAM Base Addresses ******************************************************/

#define STM32_DTCRAM_BASE        0x20000000 /* 0x20000000-0x2002ffff DTCM SRAM */
#define STM32_AXISRAM_BASE       0x24000000 /* AXI SRAM */
#define STM32_AXISRAM1_BASE      0x24000000 /* AXI SRAM1 */
#define STM32_AXISRAM2_BASE      0x24020000 /* AXI SRAM2 */
#define STM32_AXISRAM3_BASE      0x24040000 /* AXI SRAM3 */
#define STM32_AXISRAM4_BASE      0x24060000 /* AXI SRAM4 */
#define STM32_GFXMMU_VBUF0_BASE  0x25000000 /* GFXMMU virtual buffer 0 */
#define STM32_GFXMMU_VBUF1_BASE  0x25400000 /* GFXMMU virtual buffer 1 */
#define STM32_GFXMMU_VBUF2_BASE  0x25800000 /* GFXMMU virtual buffer 2 */
#define STM32_GFXMMU_VBUF3_BASE  0x25c00000 /* GFXMMU virtual buffer 3 */
#define STM32_SRAM1_BASE         0x30000000 /* AHB SRAM1 */
#define STM32_SRAM2_BASE         0x30004000 /* AHB SRAM2 */
#define STM32_SRAM123_BASE       STM32_SRAM1_BASE
#define STM32_BBSRAM_BASE        0x38800000 /* Backup SRAM */

/* Peripheral Base Addresses ************************************************/

#define STM32_APB1_BASE          0x40000000 /* APB1 peripheral region */
#define STM32_AHB1_BASE          0x40020000 /* AHB1 peripheral region */
#define STM32_APB2_BASE          0x42000000 /* APB2 peripheral region */
#define STM32_AHB2_BASE          0x48000000 /* AHB2 peripheral region */
#define STM32_AHB3_BASE          0x48020000 /* AHB3 peripheral region */
#define STM32_APB5_BASE          0x50000000 /* APB5 peripheral region */
#define STM32_AHB5_BASE          0x52000000 /* AHB5 peripheral region */
#define STM32_APB4_BASE          0x58000000 /* APB4 peripheral region */
#define STM32_AHB4_BASE          0x58020000 /* AHB4 peripheral region */

/* APB1 Base Addresses ******************************************************/

#define STM32_TIM2_BASE          0x40000000 /* 0x40000000-0x400003ff TIM2 */
#define STM32_TIM3_BASE          0x40000400 /* 0x40000400-0x400007ff TIM3 */
#define STM32_TIM4_BASE          0x40000800 /* 0x40000800-0x40000bff TIM4 */
#define STM32_TIM5_BASE          0x40000c00 /* 0x40000c00-0x40000fff TIM5 */
#define STM32_TIM6_BASE          0x40001000 /* 0x40001000-0x400013ff TIM6 */
#define STM32_TIM7_BASE          0x40001400 /* 0x40001400-0x400017ff TIM7 */
#define STM32_TIM12_BASE         0x40001800 /* 0x40001800-0x40001bff TIM12 */
#define STM32_TIM13_BASE         0x40001c00 /* 0x40001c00-0x40001fff TIM13 */
#define STM32_TIM14_BASE         0x40002000 /* 0x40002000-0x400023ff TIM14 */
#define STM32_LPTIM1_BASE        0x40002400 /* 0x40002400-0x400027ff LPTIM1 */
#define STM32_WWDG_BASE          0x40002c00 /* 0x40002c00-0x40002fff WWDG */
#define STM32_SPI2_BASE          0x40003800 /* 0x40003800-0x40003bff SPI2 */
#define STM32_SPI3_BASE          0x40003c00 /* 0x40003c00-0x40003fff SPI3 */
#define STM32_SPDIFRX_BASE       0x40004000 /* 0x40004000-0x400043ff SPDIFRX */
#define STM32_USART2_BASE        0x40004400 /* 0x40004400-0x400047ff USART2 */
#define STM32_USART3_BASE        0x40004800 /* 0x40004800-0x40004bff USART3 */
#define STM32_UART4_BASE         0x40004c00 /* 0x40004c00-0x40004fff UART4 */
#define STM32_UART5_BASE         0x40005000 /* 0x40005000-0x400053ff UART5 */
#define STM32_I2C1_BASE          0x40005400 /* 0x40005400-0x400057ff I2C1/I3C1 */
#define STM32_I3C1_BASE          STM32_I2C1_BASE
#define STM32_I2C2_BASE          0x40005800 /* 0x40005800-0x40005bff I2C2 */
#define STM32_I2C3_BASE          0x40005c00 /* 0x40005c00-0x40005fff I2C3 */
#define STM32_CEC_BASE           0x40006c00 /* 0x40006c00-0x40006fff HDMI CEC */
#define STM32_UART7_BASE         0x40007800 /* 0x40007800-0x40007bff UART7 */
#define STM32_UART8_BASE         0x40007c00 /* 0x40007c00-0x40007fff UART8 */
#define STM32_CRS_BASE           0x40008400 /* 0x40008400-0x400087ff CRS */
#define STM32_MDIOS_BASE         0x40009400 /* 0x40009400-0x400097ff MDIOS */
#define STM32_FDCAN1_BASE        0x4000a000 /* 0x4000a000-0x4000a0ff FDCAN1 */
#define STM32_FDCAN_CONFIG_BASE  0x4000a100 /* FDCAN configuration */
#define STM32_FDCAN2_BASE        0x4000a400 /* 0x4000a400-0x4000a4ff FDCAN2 */
#define STM32_CANRAM_BASE        0x4000ac00 /* FDCAN message RAM */
#define STM32_UCPD1_BASE         0x4000ec00 /* 0x4000ec00-0x4000efff UCPD1 */

/* APB2 Base Addresses ******************************************************/

#define STM32_TIM1_BASE          0x42000000 /* 0x42000000-0x420003ff TIM1 */
#define STM32_USART1_BASE        0x42001000 /* 0x42001000-0x420013ff USART1 */
#define STM32_SPI1_BASE          0x42003000 /* 0x42003000-0x420033ff SPI1 */
#define STM32_SPI4_BASE          0x42003400 /* 0x42003400-0x420037ff SPI4 */
#define STM32_TIM15_BASE         0x42004000 /* 0x42004000-0x420043ff TIM15 */
#define STM32_TIM16_BASE         0x42004400 /* 0x42004400-0x420047ff TIM16 */
#define STM32_TIM17_BASE         0x42004800 /* 0x42004800-0x42004bff TIM17 */
#define STM32_TIM9_BASE          0x42004c00 /* 0x42004c00-0x42004fff TIM9 */
#define STM32_SPI5_BASE          0x42005000 /* 0x42005000-0x420053ff SPI5 */
#define STM32_SAI1_BASE          0x42005800 /* 0x42005800-0x42005bff SAI1 */
#define STM32_SAI2_BASE          0x42005c00 /* 0x42005c00-0x42005fff SAI2 */

/* AHB1 Base Addresses ******************************************************/

#define STM32_GPDMA1_BASE        0x40021000 /* 0x40021000-0x40021fff GPDMA1 */
#define STM32_ADC12_BASE         0x40022000 /* 0x40022000-0x400223ff ADC1/2 */
#define STM32_EMAC_BASE          0x40028000 /* 0x40028000-0x4002bfff Ethernet */
#define STM32_ADF1_BASE          0x4002f000 /* 0x4002f000-0x4002ffff ADF1 */
#define STM32_OTGHS_BASE         0x40040000 /* 0x40040000-0x4007ffff USB OTG HS */
#define STM32_OTGFS_BASE         0x40080000 /* 0x40080000-0x400bffff USB OTG FS */

/* AHB2 Base Addresses ******************************************************/

#define STM32_PSSI_BASE          0x48000400 /* 0x48000400-0x480007ff PSSI */
#define STM32_SDMMC2_BASE        0x48002400 /* 0x48002400-0x480027ff SDMMC2 */
#define STM32_DLYBSDMMC2_BASE    0x48002800 /* SDMMC2 delay block */
#define STM32_CORDIC_BASE        0x48004400 /* 0x48004400-0x480047ff CORDIC */

/* AHB3 Base Addresses ******************************************************/

#define STM32_RNG_BASE           0x48020000 /* 0x48020000-0x480203ff RNG */
#define STM32_HASH_BASE          0x48020400 /* 0x48020400-0x480207ff HASH */
#define STM32_CRYP_BASE          0x48020800 /* 0x48020800-0x48020bff CRYP */
#define STM32_SAES_BASE          0x48021000 /* 0x48021000-0x480213ff SAES */
#define STM32_PKA_BASE           0x48022000 /* 0x48022000-0x480223ff PKA */

/* APB5 Base Addresses ******************************************************/

#define STM32_LTDC_BASE          0x50001000 /* 0x50001000-0x50001fff LTDC */
#define STM32_DCMIPP_BASE        0x50002000 /* 0x50002000-0x50002fff DCMIPP */
#define STM32_GFXTIM_BASE        0x50004000 /* 0x50004000-0x500043ff GFXTIM */

/* AHB5 Base Addresses ******************************************************/

#define STM32_HPDMA1_BASE        0x52000000 /* 0x52000000-0x52000fff HPDMA1 */
#define STM32_DMA2D_BASE         0x52001000 /* 0x52001000-0x52001fff DMA2D */
#define STM32_FLASHIF_BASE       0x52002000 /* 0x52002000-0x52002fff FLASH */
#define STM32_ENGI_BYTES_BASE    0x52002800 /* Engineering bytes */
#define STM32_JPEG_BASE          0x52003000 /* 0x52003000-0x52003fff JPEG */
#define STM32_FMC_BASE           0x52004000 /* 0x52004000-0x52004fff FMC */
#define STM32_XSPI1_R_BASE       0x52005000 /* 0x52005000-0x52005fff XSPI1 */
#define STM32_SDMMC1_BASE        0x52007000 /* 0x52007000-0x52007fff SDMMC1 */
#define STM32_DLYBSDMMC1_BASE    0x52008000 /* SDMMC1 delay block */
#define STM32_RAMECC1_BASE       0x52009000 /* 0x52009000-0x520093ff RAMECC1 */
#define STM32_XSPI2_R_BASE       0x5200a000 /* 0x5200a000-0x5200afff XSPI2 */
#define STM32_XSPIM_BASE         0x5200b400 /* 0x5200b400-0x5200b7ff XSPIM */
#define STM32_MCE1_BASE          0x5200b800 /* 0x5200b800-0x5200bbff MCE1 */
#define STM32_MCE2_BASE          0x5200bc00 /* 0x5200bc00-0x5200bfff MCE2 */
#define STM32_MCE3_BASE          0x5200c000 /* 0x5200c000-0x5200c3ff MCE3 */
#define STM32_GFXMMU_BASE        0x52010000 /* 0x52010000-0x52010fff GFXMMU */
#define STM32_GPU2D_BASE         0x52014000 /* 0x52014000-0x52014fff GPU2D */

/* APB4 Base Addresses ******************************************************/

#define STM32_EXTI_BASE          0x58000000 /* 0x58000000-0x580003ff EXTI */
#define STM32_SBS_BASE           0x58000400 /* 0x58000400-0x580007ff SBS */
#define STM32_LPUART1_BASE       0x58000c00 /* 0x58000c00-0x58000fff LPUART1 */
#define STM32_SPI6_BASE          0x58001400 /* 0x58001400-0x580017ff SPI6 */
#define STM32_LPTIM2_BASE        0x58002400 /* 0x58002400-0x580027ff LPTIM2 */
#define STM32_LPTIM3_BASE        0x58002800 /* 0x58002800-0x58002bff LPTIM3 */
#define STM32_LPTIM4_BASE        0x58002c00 /* 0x58002c00-0x58002fff LPTIM4 */
#define STM32_LPTIM5_BASE        0x58003000 /* 0x58003000-0x580033ff LPTIM5 */
#define STM32_VREF_BASE          0x58003c00 /* 0x58003c00-0x58003fff VREFBUF */
#define STM32_RTC_BASE           0x58004000 /* 0x58004000-0x580043ff RTC */
#define STM32_TAMP_BASE          0x58004400 /* 0x58004400-0x580047ff TAMP */
#define STM32_IWDG1_BASE         0x58004800 /* 0x58004800-0x58004bff IWDG */
#define STM32_DTS_BASE           0x58006800 /* 0x58006800-0x58006bff DTS */

/* AHB4 Base Addresses ******************************************************/

#define STM32_GPIOA_BASE         0x58020000 /* 0x58020000-0x580203ff GPIOA */
#define STM32_GPIOB_BASE         0x58020400 /* 0x58020400-0x580207ff GPIOB */
#define STM32_GPIOC_BASE         0x58020800 /* 0x58020800-0x58020bff GPIOC */
#define STM32_GPIOD_BASE         0x58020c00 /* 0x58020c00-0x58020fff GPIOD */
#define STM32_GPIOE_BASE         0x58021000 /* 0x58021000-0x580213ff GPIOE */
#define STM32_GPIOF_BASE         0x58021400 /* 0x58021400-0x580217ff GPIOF */
#define STM32_GPIOG_BASE         0x58021800 /* 0x58021800-0x58021bff GPIOG */
#define STM32_GPIOH_BASE         0x58021c00 /* 0x58021c00-0x58021fff GPIOH */
#define STM32_GPIOM_BASE         0x58023000 /* 0x58023000-0x580233ff GPIOM */
#define STM32_GPION_BASE         0x58023400 /* 0x58023400-0x580237ff GPION */
#define STM32_GPIOO_BASE         0x58023800 /* 0x58023800-0x58023bff GPIOO */
#define STM32_GPIOP_BASE         0x58023c00 /* 0x58023c00-0x58023fff GPIOP */
#define STM32_RCC_BASE           0x58024400 /* 0x58024400-0x580247ff RCC */
#define STM32_PWR_BASE           0x58024800 /* 0x58024800-0x58024bff PWR */
#define STM32_CRC_BASE           0x58024c00 /* 0x58024c00-0x58024fff CRC */
#define STM32_RAMECC2_BASE       0x58027000 /* 0x58027000-0x580273ff RAMECC2 */

/* The DBGMCU registers are accessible by the processor at 0x5c001000. ******/

#define STM32_DEBUGMCU_BASE      0x5c001000

#endif /* __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32H7RSXX_MEMORYMAP_H */
