/****************************************************************************
 * arch/risc-v/src/ch32v/hardware/ch32v_memorymap.h
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

#ifndef __ARCH_RISCV_SRC_CH32V_HARDWARE_CH32V_MEMORYMAP_H
#define __ARCH_RISCV_SRC_CH32V_HARDWARE_CH32V_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

// #include <nuttx/config.h>

/*
 * CH32V notes: These come from CH32FV2x_V3xRM.PDF.
 * Not all chips will have all blocks or have all peripherals brought out to
 * pins. The doc kind of leaves on your own to discover that.
 * This file came from ./hardware/stm32f72xx73xx_memorymap.h
 */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CH32V Address Blocks *****************************************************/

#define CH32V_CODE_BASE      0x00000000     /* 0x00000000-0x1fffffff: 512Mb code block */
#define CH32V_SRAM_BASE      0x20000000     /* 0x20000000-0x3fffffff: 512Mb sram block */
#define CH32V_PERIPH_BASE    0x40000000     /* 0x40000000-0x5fffffff: 512Mb AHB1-2 peripheral blocks */
#define CH32V_FMC_BASE12     0x60000000     /* 0x60000000-0x7fffffff: 512Mb FMC bank1&2 block */
#  define CH32V_FMC_BANK1    0x60000000     /* 0x60000000-0x6fffffff:       256Mb NOR/SRAM */
#  define CH32V_FMC_BANK2    0x70000000     /* 0x70000000-0x7fffffff:       256Mb NAND FLASH */
#define CH32V_FMC_BASE34     0x80000000     /* 0x80000000-0x9fffffff: 512Mb FMC bank3&4 block */
#  define CH32V_FMC_BANK3    0x80000000     /* 0x80000000-0x8fffffff:       256Mb NAND FLASH */
#  define CH32V_FMC_BANK4    0x90000000     /* 0x90000000-0x9fffffff:       256Mb PC CARD */
#define CH32V_FMC_BASE5      0xc0000000     /* 0xc0000000-0xcfffffff: 256Mb FMC SDRAM Bank 1 */
#define CH32V_FMC_BASE6      0xd0000000     /* 0xd0000000-0xdfffffff: 256Mb FMC SDRAM Bank 2 */
#define CH32V_CORTEX_BASE    0xe0000000     /* 0xe0000000-0xffffffff: 512Mb Cortex-M7 block */

#define CH32V_REGION_MASK    0xf0000000
#define CH32V_IS_SRAM(a)     ((((uint32_t)(a)) & CH32V_REGION_MASK) == CH32V_SRAM_BASE)
#define CH32V_IS_EXTSRAM(a)  ((((uint32_t)(a)) & CH32V_REGION_MASK) == CH32V_FMC_BANK1)

/* Code Base Addresses ******************************************************/

#define CH32V_BOOT_BASE      0x00000000     /* 0x00000000-0x000fffff: Aliased boot memory */
#define CH32V_INSTRAM_BASE   0x00000000     /* 0x00000000-0x00003fff: Instruction RAM (ITCM-RAM) */
/* #define CH32V_SYSMEM_ICTM    0x00100000     /* 0x00100000-0x0010edbf: System memory (ITCM) */
/* #define CH32V_FLASH_ITCM     0x00200000     /* 0x00200000-0x002fffff: FLASH memory (ITCM) */
/* #define CH32V_FLASH_AXIM     0x08000000     /* 0x08000000-0x080fffff: FLASH memory (AXIM) */
#define CH32V_FLASH_BASE     CH32V_FLASH_AXIM
#define CH32V_OPTIONS_BASE   0x1fff0000     /* 0x1ff00000-0x1fff001f: Option bytes (AXIM) */

/* SRAM Base Addresses ******************************************************/

#define CH32V_DTCRAM_BASE    0x20000000     /* 0x20000000-0x2000ffff: DTCM-RAM on TCM interface */
/* #define CH32V_SRAM1_BASE     0x20010000     /* 0x20010000-0x2003bfff: System SRAM1 */
/* #define CH32V_SRAM2_BASE     0x2003C000     /* 0x2003c000-0x2003ffff: System SRAM2 */

/* Peripheral Base Addresses ************************************************/

/* #define CH32V_APB1_BASE      0x40000000     /* 0x40000000-0x40007fff: APB1 */
/* #define CH32V_APB2_BASE      0x40010000     /* 0x40010000-0x40016bff: APB2 */
/* #define CH32V_AHB1_BASE      0x40020000     /* 0x40020000-0x4007ffff: APB1 */
/* #define CH32V_AHB2_BASE      0x50000000     /* 0x50000000-0x50060bff: AHB2 */
/* #define CH32V_AHB3_BASE      0x60000000     /* 0x60000000-0xdfffffff: AHB3 */

/* APB1 Base Addresses ******************************************************/

#define CH32V_TIM2_BASE      0x40000000     /* 0x40000000-0x400003ff: TIM2 */
#define CH32V_TIM3_BASE      0x40000400     /* 0x40000400-0x400007ff: TIM3 */
#define CH32V_TIM4_BASE      0x40000800     /* 0x40000800-0x40000bff: TIM4 */
#define CH32V_TIM5_BASE      0x40000c00     /* 0x40000c00-0x40000fff: TIM5 */
#define CH32V_TIM6_BASE      0x40001000     /* 0x40001000-0x400013ff: TIM6 */
#define CH32V_TIM7_BASE      0x40001400     /* 0x40001400-0x400017ff: TIM7 */

/* #define CH32V_TIM12_BASE     0x40001800     /* 0x40001800-0x40001bff: TIM12 */
/* #define CH32V_TIM13_BASE     0x40001c00     /* 0x40001c00-0x40001fff: TIM13 */
/* #define CH32V_TIM14_BASE     0x40002000     /* 0x40002000-0x400023ff: TIM14 */
/* #define CH32V_LPTIM1_BASE    0x40002400     /* 0x40002400-0x400027ff: LPTIM1 */
#define CH32V_USART6_BASE    0x40011800     /* 0x40011400-0x400117ff: USART6 */
#define CH32V_USART7_BASE    0x40011c00     /* 0x40011400-0x400117ff: USART7 */
#define CH32V_USART8_BASE    0x40012000     /* 0x40011400-0x400117ff: USART8 */

#define CH32V_RTC_BASE       0x40002800     /* 0x40002800-0x40002bff: RTC & BKP Registers */
#define CH32V_WWDG_BASE      0x40002c00     /* 0x40002c00-0x40002fff: WWDG */
#define CH32V_IWDG_BASE      0x40003000     /* 0x40003000-0x400033ff: IWDG */
#define CH32V_SPI2_BASE      0x40003800     /* 0x40003800-0x40003bff: SPI2 / I2S2 */
#define CH32V_I2S2_BASE      0x40003800     /* 0x40003800-0x40003bff: SPI2 / I2S2 */
#define CH32V_SPI3_BASE      0x40003c00     /* 0x40003c00-0x40003fff: SPI3 / I2S3 */
#define CH32V_I2S3_BASE      0x40003c00     /* 0x40003c00-0x40003fff: SPI3 / I2S3 */
#define CH32V_USART2_BASE    0x40004400     /* 0x40004400-0x400047ff: USART2 */
#define CH32V_USART3_BASE    0x40004800     /* 0x40004800-0x40004bff: USART3 */
#define CH32V_USART4_BASE    0x40004c00     /* 0x40004c00-0x40004fff: USART4 */
#define CH32V_USART5_BASE    0x40005000     /* 0x40005000-0x400053ff: USART5 */
#define CH32V_I2C1_BASE      0x40005400     /* 0x40005400-0x400057ff: I2C1 */
#define CH32V_I2C2_BASE      0x40005800     /* 0x40005800-0x40005bff: I2C2 */
/* #define CH32V_I2C3_BASE      0x40005c00     /* 0x40005c00-0x40005fff: I2C3 */
/* #define CH32V_I2C3_BASE      0x40005c00     /* 0x40005c00-0x40005fff: I2C3 */
#define CH32V_CAN1_BASE      0x40006400     /* 0x40006400-0x400067ff: CAN1 */
#define CH32V_PWR_BASE       0x40007000     /* 0x40007000-0x400073ff: PWR */
#define CH32V_DAC_BASE       0x40007400     /* 0x40007400-0x400077ff: DAC */
/* #define CH32V_UART7_BASE     0x40007800     /* 0x40007800-0x40007bff: UART7 */
/* #define CH32V_UART8_BASE     0x40007c00     /* 0x40007c00-0x40007fff: UART8 */

/* APB2 Base Addresses ******************************************************/

#define CH32V_TIM1_BASE      0x40012c00     /* 0x40012c00-0x400103ff: TIM1 */
#define CH32V_TIM8_BASE      0x40013400     /* 0x40013400-0x400107ff: TIM8 */
#define CH32V_USART1_BASE    0x40038000     /* 0x40013800-0x400113ff: USART1 */

#if LATER

#define CH32V_USART6_BASE    0x40011400     /* 0x40011400-0x400117ff: USART6 */
#define CH32V_SDMMC2_BASE    0x40011C00     /* 0x4001C000-0x40011fff: SDMMC2 */
/* #define CH32V_ADC_BASE       0x40012000     /* 0x40012000-0x400123ff: ADC1 - ADC2 - ADC3 */
#  define CH32V_ADC1_BASE    0x40012000     /*                        ADC1 */
#  define CH32V_ADC2_BASE    0x40012100     /*                        ADC2 */
#  define CH32V_ADC3_BASE    0x40012200     /*                        ADC3 */
#  define CH32V_ADCCMN_BASE  0x40012300     /*                        Common */
#define CH32V_SDMMC1_BASE    0x40012c00     /* 0x40012c00-0x40012fff: SDMMC1 */
#define CH32V_SPI1_BASE      0x40013000     /* 0x40013000-0x400133ff: SPI1 */
#define CH32V_SPI4_BASE      0x40013400     /* 0x40013400-0x400137ff: SPI4 */
#define CH32V_SYSCFG_BASE    0x40013800     /* 0x40013800-0x40013bff: SYSCFG */
#define CH32V_EXTI_BASE      0x40013c00     /* 0x40013c00-0x40013fff: EXTI */
#define CH32V_TIM9_BASE      0x40014000     /* 0x40014000-0x400143ff: TIM9 */
#define CH32V_TIM10_BASE     0x40014400     /* 0x40014400-0x400147ff: TIM10 */
#define CH32V_TIM11_BASE     0x40014800     /* 0x40014800-0x40014bff: TIM11 */
#define CH32V_SPI5_BASE      0x40015000     /* 0x40015000-0x400153ff: SPI5 */
#define CH32V_SAI1_BASE      0x40015800     /* 0x40015800-0x40015bff: SAI1 */
#define CH32V_SAI2_BASE      0x40015c00     /* 0x40015c00-0x40015fff: SAI2 */
/* #define CH32V_USBPHYC_BASE   0x40017c00     /* 0x40017C00-0x40017fff: OTG PHY HS */

/* AHB1 Base Addresses ******************************************************/

#define CH32V_GPIO_INCR      0x400          /* 1K spacing between GPIO groups */
#define CH32V_GPIOA_BASE     0x40020000     /* 0x40020000-0x400203ff: GPIOA */
#define CH32V_GPIOB_BASE     0x40020400     /* 0x40020400-0x400207ff: GPIOB */
#define CH32V_GPIOC_BASE     0x40020800     /* 0x40020800-0x40020bff: GPIOC */
#define CH32V_GPIOD_BASE     0x40020c00     /* 0x40020c00-0x40020fff: GPIOD */
#define CH32V_GPIOE_BASE     0x40021000     /* 0x40021000-0x400213ff: GPIOE */
#define CH32V_GPIOF_BASE     0x40021400     /* 0x40021400-0x400217ff: GPIOF */
#define CH32V_GPIOG_BASE     0x40021800     /* 0x40021800-0x40021bff: GPIOG */
#define CH32V_GPIOH_BASE     0x40021c00     /* 0x40021c00-0x40021fff: GPIOH */
#define CH32V_GPIOI_BASE     0x40022000     /* 0x40022000-0x400223ff: GPIOI */
#define CH32V_CRC_BASE       0x40023000     /* 0x40023000-0x400233ff: CRC */
#define CH32V_RCC_BASE       0x40023800     /* 0x40023800-0x40023bff: RCC */
#define CH32V_FLASHIF_BASE   0x40023c00     /* 0x40023c00-0x40023fff: Flash interface */
#define CH32V_BKPSRAM_BASE   0x40024000     /* 0x40024000-0x40024fff: BKPSRAM */
#define CH32V_DMA1_BASE      0x40026000     /* 0x40026000-0x400263ff: DMA1 */
#define CH32V_DMA2_BASE      0x40026400     /* 0x40026400-0x400267ff: DMA2 */
#define CH32V_USBOTGHS_BASE  0x40040000     /* 0x40040000-0x4007ffff: USB OTG HS */

/* AHB2 Base Addresses ******************************************************/

#define CH32V_USBOTGFS_BASE  0x50000000     /* 0x50000000-0x5003ffff: USB OTG FS */
#define CH32V_RNG_BASE       0x50060800     /* 0x50060800-0x50060bff: RNG */

/* AHB3 Base Addresses ******************************************************/

#define CH32V_FMCBANK1_BASE  0x60000000     /* 0x60000000-0x6fffffff: FMC bank 1 */
#define CH32V_FMCBANK2_BASE  0x70000000     /* 0x70000000-0x7fffffff: FMC bank 2 */
#define CH32V_FMCBANK3_BASE  0x80000000     /* 0x80000000-0x8fffffff: FMC bank 3 */
#define CH32V_FMCBANK4_BASE  0x90000000     /* 0x90000000-0x9fffffff: FMC bank 4 */
#define CH32V_FMC_BASE       0xa0000000     /* 0xa0000000-0xa0000fff: FMC control registers */
#define CH32V_QUADSPI_BASE   0xa0001000     /* 0xa0001000-0xa0001fff: QuadSPI Control */
#define CH32V_FMCBANK5_BASE  0xc0000000     /* 0xc0000000-0xcfffffff: FMC bank 5 */
#define CH32V_FMCBANK6_BASE  0xd0000000     /* 0xd0000000-0xdfffffff: FMC bank 6 */

/* Cortex-M7 Base Addresses *************************************************/

/* Other registers -- see armv7-m/nvic.h for standard Cortex-M3 registers in
 * this address range
 */

#define CH32V_DEBUGMCU_BASE 0xe0042000
#endif

#endif /* __ARCH_RISCV_SRC_CH32V7_HARDWARE_CH32V_MEMORYMAP_H */
