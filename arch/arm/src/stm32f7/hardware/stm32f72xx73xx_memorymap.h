/************************************************************************************
 * arch/arm/src/stm32f7/hardware/stm32f72xx73xx_memorymap.h
 *
 *   Copyright (C) 2015, 2017 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
 *            Bob Feretich <bob.feretich@rafresearch.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32F7_HARDWARE_STM32F72XXX73XXX_MEMORYMAP_H
#define __ARCH_ARM_SRC_STM32F7_HARDWARE_STM32F72XXX73XXX_MEMORYMAP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_STM32F7_STM32F72XX) || defined(CONFIG_STM32F7_STM32F73XX)

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* STM32F72XX STM32F73XX Address Blocks *********************************************/

#define STM32_CODE_BASE      0x00000000     /* 0x00000000-0x1fffffff: 512Mb code block */
#define STM32_SRAM_BASE      0x20000000     /* 0x20000000-0x3fffffff: 512Mb sram block */
#define STM32_PERIPH_BASE    0x40000000     /* 0x40000000-0x5fffffff: 512Mb AHB1-2 peripheral blocks */
#define STM32_FMC_BASE12     0x60000000     /* 0x60000000-0x7fffffff: 512Mb FMC bank1&2 block */
#  define STM32_FMC_BANK1    0x60000000     /* 0x60000000-0x6fffffff:       256Mb NOR/SRAM */
#  define STM32_FMC_BANK2    0x70000000     /* 0x70000000-0x7fffffff:       256Mb NAND FLASH */
#define STM32_FMC_BASE34     0x80000000     /* 0x80000000-0x9fffffff: 512Mb FMC bank3&4 block */
#  define STM32_FMC_BANK3    0x80000000     /* 0x80000000-0x8fffffff:       256Mb NAND FLASH */
#  define STM32_FMC_BANK4    0x90000000     /* 0x90000000-0x9fffffff:       256Mb PC CARD */
#define STM32_FMC_BASE5      0xc0000000     /* 0xc0000000-0xcfffffff: 256Mb FMC SDRAM Bank 1 */
#define STM32_FMC_BASE6      0xd0000000     /* 0xd0000000-0xdfffffff: 256Mb FMC SDRAM Bank 2 */
#define STM32_CORTEX_BASE    0xe0000000     /* 0xe0000000-0xffffffff: 512Mb Cortex-M7 block */

#define STM32_REGION_MASK    0xf0000000
#define STM32_IS_SRAM(a)     ((((uint32_t)(a)) & STM32_REGION_MASK) == STM32_SRAM_BASE)
#define STM32_IS_EXTSRAM(a)  ((((uint32_t)(a)) & STM32_REGION_MASK) == STM32_FMC_BANK1)

/* Code Base Addresses **************************************************************/

#define STM32_BOOT_BASE      0x00000000     /* 0x00000000-0x000fffff: Aliased boot memory */
#define STM32_INSTRAM_BASE   0x00000000     /* 0x00000000-0x00003fff: Instruction RAM (ITCM-RAM) */
#define STM32_SYSMEM_ICTM    0x00100000     /* 0x00100000-0x0010edbf: System memory (ITCM) */
#define STM32_FLASH_ITCM     0x00200000     /* 0x00200000-0x002fffff: FLASH memory (ITCM) */
#define STM32_FLASH_AXIM     0x08000000     /* 0x08000000-0x080fffff: FLASH memory (AXIM) */
#define STM32_FLASH_BASE     STM32_FLASH_AXIM
#define STM32_OPTIONS_BASE   0x1fff0000     /* 0x1ff00000-0x1fff001f: Option bytes (AXIM) */

/* Information Addresses ************************************************************/

#define STM32_SYSMEM_AXIM    0x1ff00000     /* 0x1ff00000-0x1ff076D7: System memory (AXIM) */
#define STM32_SYSMEM_UID     0x1ff07A10     /* The 96-bit unique device identifier */
#define STM32_OTP_ICTM       0x00107800     /* 0x00107800-0x00107A0F: OTP (ITCM) */
#define STM32_OTP_AXIM       0x1ff07800     /* 0x1ff07800-0x1ff07A0F: OTP (AXIM) */
#define STM32_OPT_BASE       STM32_OTP_AXIM
#define STM32_OPT_SIZE       1056

/* SRAM Base Addresses **************************************************************/

#define STM32_DTCRAM_BASE    0x20000000     /* 0x20000000-0x2000ffff: DTCM-RAM on TCM interface */
#define STM32_SRAM1_BASE     0x20010000     /* 0x20010000-0x2003bfff: System SRAM1 */
#define STM32_SRAM2_BASE     0x2003C000     /* 0x2003c000-0x2003ffff: System SRAM2 */

/* Peripheral Base Addresses ********************************************************/

#define STM32_APB1_BASE      0x40000000     /* 0x40000000-0x40007fff: APB1 */
#define STM32_APB2_BASE      0x40010000     /* 0x40010000-0x40016bff: APB2 */
#define STM32_AHB1_BASE      0x40020000     /* 0x40020000-0x4007ffff: APB1 */
#define STM32_AHB2_BASE      0x50000000     /* 0x50000000-0x50060bff: AHB2 */
#define STM32_AHB3_BASE      0x60000000     /* 0x60000000-0xdfffffff: AHB3 */

/* APB1 Base Addresses **************************************************************/

#define STM32_TIM2_BASE      0x40000000     /* 0x40000000-0x400003ff: TIM2 */
#define STM32_TIM3_BASE      0x40000400     /* 0x40000400-0x400007ff: TIM3 */
#define STM32_TIM4_BASE      0x40000800     /* 0x40000800-0x40000bff: TIM4 */
#define STM32_TIM5_BASE      0x40000c00     /* 0x40000c00-0x40000fff: TIM5 */
#define STM32_TIM6_BASE      0x40001000     /* 0x40001000-0x400013ff: TIM6 */
#define STM32_TIM7_BASE      0x40001400     /* 0x40001400-0x400017ff: TIM7 */
#define STM32_TIM12_BASE     0x40001800     /* 0x40001800-0x40001bff: TIM12 */
#define STM32_TIM13_BASE     0x40001c00     /* 0x40001c00-0x40001fff: TIM13 */
#define STM32_TIM14_BASE     0x40002000     /* 0x40002000-0x400023ff: TIM14 */
#define STM32_LPTIM1_BASE    0x40002400     /* 0x40002400-0x400027ff: LPTIM1 */
#define STM32_RTC_BASE       0x40002800     /* 0x40002800-0x40002bff: RTC & BKP Registers */
#define STM32_WWDG_BASE      0x40002c00     /* 0x40002c00-0x40002fff: WWDG */
#define STM32_IWDG_BASE      0x40003000     /* 0x40003000-0x400033ff: IWDG */
#define STM32_SPI2_BASE      0x40003800     /* 0x40003800-0x40003bff: SPI2 / I2S2 */
#define STM32_I2S2_BASE      0x40003800     /* 0x40003800-0x40003bff: SPI2 / I2S2 */
#define STM32_SPI3_BASE      0x40003c00     /* 0x40003c00-0x40003fff: SPI3 / I2S3 */
#define STM32_I2S3_BASE      0x40003c00     /* 0x40003c00-0x40003fff: SPI3 / I2S3 */
#define STM32_USART2_BASE    0x40004400     /* 0x40004400-0x400047ff: USART2 */
#define STM32_USART3_BASE    0x40004800     /* 0x40004800-0x40004bff: USART3 */
#define STM32_UART4_BASE     0x40004c00     /* 0x40004c00-0x40004fff: UART4 */
#define STM32_UART5_BASE     0x40005000     /* 0x40005000-0x400053ff: UART5 */
#define STM32_I2C1_BASE      0x40005400     /* 0x40005400-0x400057ff: I2C1 */
#define STM32_I2C2_BASE      0x40005800     /* 0x40005800-0x40005bff: I2C2 */
#define STM32_I2C3_BASE      0x40005c00     /* 0x40005c00-0x40005fff: I2C3 */
#define STM32_CAN1_BASE      0x40006400     /* 0x40006400-0x400067ff: CAN1 */
#define STM32_PWR_BASE       0x40007000     /* 0x40007000-0x400073ff: PWR */
#define STM32_DAC_BASE       0x40007400     /* 0x40007400-0x400077ff: DAC */
#define STM32_UART7_BASE     0x40007800     /* 0x40007800-0x40007bff: UART7 */
#define STM32_UART8_BASE     0x40007c00     /* 0x40007c00-0x40007fff: UART8 */

/* APB2 Base Addresses **************************************************************/

#define STM32_TIM1_BASE      0x40010000     /* 0x40010000-0x400103ff: TIM1 */
#define STM32_TIM8_BASE      0x40010400     /* 0x40010400-0x400107ff: TIM8 */
#define STM32_USART1_BASE    0x40011000     /* 0x40011000-0x400113ff: USART1 */
#define STM32_USART6_BASE    0x40011400     /* 0x40011400-0x400117ff: USART6 */
#define STM32_SDMMC2_BASE    0x40011C00     /* 0x4001C000-0x40011fff: SDMMC2 */
#define STM32_ADC_BASE       0x40012000     /* 0x40012000-0x400123ff: ADC1 - ADC2 - ADC3 */
#  define STM32_ADC1_BASE    0x40012000     /*                        ADC1 */
#  define STM32_ADC2_BASE    0x40012100     /*                        ADC2 */
#  define STM32_ADC3_BASE    0x40012200     /*                        ADC3 */
#  define STM32_ADCCMN_BASE  0x40012300     /*                        Common */
#define STM32_SDMMC1_BASE    0x40012c00     /* 0x40012c00-0x40012fff: SDMMC1 */
#define STM32_SPI1_BASE      0x40013000     /* 0x40013000-0x400133ff: SPI1 */
#define STM32_SPI4_BASE      0x40013400     /* 0x40013400-0x400137ff: SPI4 */
#define STM32_SYSCFG_BASE    0x40013800     /* 0x40013800-0x40013bff: SYSCFG */
#define STM32_EXTI_BASE      0x40013c00     /* 0x40013c00-0x40013fff: EXTI */
#define STM32_TIM9_BASE      0x40014000     /* 0x40014000-0x400143ff: TIM9 */
#define STM32_TIM10_BASE     0x40014400     /* 0x40014400-0x400147ff: TIM10 */
#define STM32_TIM11_BASE     0x40014800     /* 0x40014800-0x40014bff: TIM11 */
#define STM32_SPI5_BASE      0x40015000     /* 0x40015000-0x400153ff: SPI5 */
#define STM32_SAI1_BASE      0x40015800     /* 0x40015800-0x40015bff: SAI1 */
#define STM32_SAI2_BASE      0x40015c00     /* 0x40015c00-0x40015fff: SAI2 */
#define STM32_USBPHYC_BASE   0x40017c00     /* 0x40017C00-0x40017fff: OTG PHY HS */

/* AHB1 Base Addresses **************************************************************/

#define STM32_GPIO_INCR      0x400          /* 1K spacing between GPIO groups */
#define STM32_GPIOA_BASE     0x40020000     /* 0x40020000-0x400203ff: GPIOA */
#define STM32_GPIOB_BASE     0x40020400     /* 0x40020400-0x400207ff: GPIOB */
#define STM32_GPIOC_BASE     0x40020800     /* 0x40020800-0x40020bff: GPIOC */
#define STM32_GPIOD_BASE     0x40020c00     /* 0x40020c00-0x40020fff: GPIOD */
#define STM32_GPIOE_BASE     0x40021000     /* 0x40021000-0x400213ff: GPIOE */
#define STM32_GPIOF_BASE     0x40021400     /* 0x40021400-0x400217ff: GPIOF */
#define STM32_GPIOG_BASE     0x40021800     /* 0x40021800-0x40021bff: GPIOG */
#define STM32_GPIOH_BASE     0x40021c00     /* 0x40021c00-0x40021fff: GPIOH */
#define STM32_GPIOI_BASE     0x40022000     /* 0x40022000-0x400223ff: GPIOI */
#define STM32_CRC_BASE       0x40023000     /* 0x40023000-0x400233ff: CRC */
#define STM32_RCC_BASE       0x40023800     /* 0x40023800-0x40023bff: RCC */
#define STM32_FLASHIF_BASE   0x40023c00     /* 0x40023c00-0x40023fff: Flash interface */
#define STM32_BKPSRAM_BASE   0x40024000     /* 0x40024000-0x40024fff: BKPSRAM */
#define STM32_DMA1_BASE      0x40026000     /* 0x40026000-0x400263ff: DMA1 */
#define STM32_DMA2_BASE      0x40026400     /* 0x40026400-0x400267ff: DMA2 */
#define STM32_USBOTGHS_BASE  0x40040000     /* 0x40040000-0x4007ffff: USB OTG HS */

/* AHB2 Base Addresses **************************************************************/

#define STM32_USBOTGFS_BASE  0x50000000     /* 0x50000000-0x5003ffff: USB OTG FS */
#define STM32_RNG_BASE       0x50060800     /* 0x50060800-0x50060bff: RNG */

/* AHB3 Base Addresses **************************************************************/

#define STM32_FMCBANK1_BASE  0x60000000     /* 0x60000000-0x6fffffff: FMC bank 1 */
#define STM32_FMCBANK2_BASE  0x70000000     /* 0x70000000-0x7fffffff: FMC bank 2 */
#define STM32_FMCBANK3_BASE  0x80000000     /* 0x80000000-0x8fffffff: FMC bank 3 */
#define STM32_FMCBANK4_BASE  0x90000000     /* 0x90000000-0x9fffffff: FMC bank 4 */
#define STM32_FMC_BASE       0xa0000000     /* 0xa0000000-0xa0000fff: FMC control registers */
#define STM32_QUADSPI_BASE   0xa0001000     /* 0xa0001000-0xa0001fff: QuadSPI Control */
#define STM32_FMCBANK5_BASE  0xc0000000     /* 0xc0000000-0xcfffffff: FMC bank 5 */
#define STM32_FMCBANK6_BASE  0xd0000000     /* 0xd0000000-0xdfffffff: FMC bank 6 */

/* Cortex-M7 Base Addresses *********************************************************/
/* Other registers -- see armv7-m/nvic.h for standard Cortex-M3 registers in this
 * address range  */

#define STM32_DEBUGMCU_BASE 0xe0042000

#endif /* CONFIG_STM32F7_STM32F72XX || CONFIG_STM32F7_STM32F73XX */
#endif /* __ARCH_ARM_SRC_STM32F7_HARDWARE_STM32F72XX73XX_MEMORYMAP_H */
