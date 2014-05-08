/************************************************************************************
 * arch/arm/src/stm32/chip/stm32l15xxx_memorymap.h
 * For STM32L100xx, STM32L151xx, STM32L152xx and STM32L162xx advanced ARM-based
 * 32-bit MCUs
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __ARCH_ARM_SRC_STM32_CHIP_STM32L15XXX_MEMORYMAP_H
#define __ARCH_ARM_SRC_STM32_CHIP_STM32L15XXX_MEMORYMAP_H

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* STM32F40XXX Address Blocks *******************************************************/

#define STM32_CODE_BASE      0x00000000     /* 0x00000000-0x1fffffff: 512Mb code block */
#define STM32_SRAM_BASE      0x20000000     /* 0x20000000-0x3fffffff: 512Mb sram block */
#define STM32_PERIPH_BASE    0x40000000     /* 0x40000000-0x5fffffff: 512Mb peripheral block */
                                            /* 0x60000000-0xdfffffff: Reserved */
#define STM32_CORTEX_BASE    0xe0000000     /* 0xe0000000-0xffffffff: 512Mb Cortex-M4 block */

#define STM32_REGION_MASK    0xf0000000
#define STM32_IS_SRAM(a)     ((((uint32_t)(a)) & STM32_REGION_MASK) == STM32_SRAM_BASE)

/* Code Base Addresses **************************************************************/

#define STM32_BOOT_BASE      0x00000000     /* 0x00000000-0x07ffffff: Aliased boot memory */
#define STM32_FLASH_BASE     0x08000000     /* 0x08000000-0x0801ffff: FLASH memory */
                                            /* 0x08020000-0x807fffff: Reserved */
#define STM32_EEPROM_BASE    0x08080000     /* 0x10000000-0x1000ffff: 64Kb CCM data RAM */
                                            /* 0x10010000-0x1ffeffff: Reserved */
#define STM32_SYSMEM_BASE    0x1ff00000     /* 0x1ff00000-0x1ff00fff: System memory */
                                            /* 0x1ff01000-0x1fff7fff: Reserved */
#define STM32_OPTION_BASE    0x1ff80000     /* 0x1fffc000-0x1ff8001f: Option bytes */
                                            /* 0x1ff80020-0x1fffffff: Reserved */

/* SRAM Base Addresses **************************************************************/

#define STM32_SRAMBB_BASE    0x22000000     /* SRAM bit-band base */

/* Peripheral Base Addresses ********************************************************/

#define STM32_APB1_BASE      0x40000000     /* 0x40000000-0x40007c03: APB1 */
                                            /* 0x40007c04-0x4000ffff: Reserved */
#define STM32_APB2_BASE      0x40010000     /* 0x40010000-0x40013bff: APB2 */
                                            /* 0x40013c00-0x4001ffff: Reserved */
#define STM32_AHB_BASE       0x40020000     /* 0x40020000-0xa0000fff: AHB */
                                            /* 0xa0001000-0x4fffffff: Reserved */
#define STM32_PERIPHBB_BASE  0x42000000     /* Peripheral bit-band base */

/* APB1 Base Addresses **************************************************************/

#define STM32_TIM2_BASE      0x40000000     /* 0x40000000-0x400003ff TIM2 */
#define STM32_TIM3_BASE      0x40000400     /* 0x40000400-0x400007ff TIM3 */
#define STM32_TIM4_BASE      0x40000800     /* 0x40000800-0x40000bff TIM4 */
#define STM32_TIM5_BASE      0x40000800     /* 0x40000c00-0x40000fff TIM5 */
#define STM32_TIM6_BASE      0x40001000     /* 0x40001000-0x400013ff TIM6 */
#define STM32_TIM7_BASE      0x40001400     /* 0x40001400-0x400017ff TIM7 */
#define STM32_LCD_BASE       0x40002400     /* 0x40002400-0x400027ff LCD */
#define STM32_RTC_BASE       0x40002800     /* 0x40002800-0x40002bff RTC */
#define STM32_WWDG_BASE      0x40002c00     /* 0x40002c00-0x40002fff WWDG */
#define STM32_IWDG_BASE      0x40003000     /* 0x40003000-0x400033ff IWDG */
#define STM32_SPI2_BASE      0x40003800     /* 0x40003800-0x40003bff SPI2 */
#define STM32_SPI3_BASE      0x40003c00     /* 0x40003c00-0x40003fff SPI3 */
#define STM32_USART2_BASE    0x40004400     /* 0x40004400-0x400047ff USART2 */
#define STM32_USART3_BASE    0x40004800     /* 0x40004800-0x40004bff USART3 */
#define STM32_UART4_BASE     0x40004c00     /* 0x40004c00-0x40004fff UART4 */
#define STM32_UART5_BASE     0x40005000     /* 0x40005000-0x400053ff UART5 */
#define STM32_I2C1_BASE      0x40005400     /* 0x40005400-0x400057ff I2C1 */
#define STM32_I2C2_BASE      0x40005800     /* 0x40005800-0x40005bff I2C2 */
#define STM32_USB_BASE       0x40005c00     /* 0x40005c00-0x40005fff USB device FS */
#define STM32_USBRAM_BASE    0x40006000     /* 0x40006000-0x400063ff USB SRAM 512B */
#define STM32_PWR_BASE       0x40007000     /* 0x40007000-0x400073ff PWR */
#define STM32_DAC_BASE       0x40007400     /* 0x40007400-0x400077ff DAC (dual) */
#define STM32_DAC_COMP       0x40007c00     /* 0x40007c00-0x40007c03 COMP */
#define STM32_DAC_RI         0x40007c04     /* 0x40007c04-0x40007c5b RI */
#define STM32_DAC_OPAMP      0x40007c5c     /* 0x40007c5c-0x40007fff OPAMP */

/* APB2 Base Addresses **************************************************************/

#define STM32_SYSCFG_BASE    0x40010000     /* 0x40010000-0x400103FF SYSCFG */
#define STM32_EXTI_BASE      0x40010400     /* 0x40010400-0x400107FF EXTI */
#define STM32_TIM9_BASE      0x40010800     /* 0x40010800-0x40010bff TIM9 */
#define STM32_TIM10_BASE     0x40010c00     /* 0x40010c00-0x40010fff TIM10 */
#define STM32_ADC_BASE       0x40012400     /* 0x40012400-0x400127ff ADC */
#define STM32_SDIO_BASE      0x40012c00     /* 0x40012c00-0x40012fff SDIO */
#define STM32_SPI1_BASE      0x40013000     /* 0x40013000-0x400133ff SPI1 */
#define STM32_USART1_BASE    0x40013800     /* 0x40013800-0x40013bff USART1 */

/* AHB Base Addresses ***************************************************************/

#define STM32_GPIOA_BASE     0x40020000     /* 0x40020000-0x400203ff GPIO Port A */
#define STM32_GPIOB_BASE     0x40020400     /* 0x40020400-0x400207ff GPIO Port B */
#define STM32_GPIOC_BASE     0x40020800     /* 0x40020800-0x40020bff GPIO Port C */
#define STM32_GPIOD_BASE     0x40020c00     /* 0x40020c00-0x40020fff GPIO Port D */
#define STM32_GPIOE_BASE     0x40021000     /* 0x40021000-0x400213ff GPIO Port E */
#define STM32_GPIOH_BASE     0x40021400     /* 0x40021400-0x400217ff GPIO Port H */
#define STM32_GPIOF_BASE     0x40021800     /* 0x40021800-0x40021bff GPIO Port F */
#define STM32_GPIOG_BASE     0x40021c00     /* 0x40021c00-0x40021fff GPIO Port G */
#define STM32_CRC_BASE       0x40023000     /* 0x40023000-0x400233ff CRC */
#define STM32_RCC_BASE       0x40023800     /* 0x40023800-0x40023bff RCC */
#define STM32_FLASHIF_BASE   0x40023c00     /* 0x40023c00-0x40023fff Flash memory interface */
#define STM32_DMA1_BASE      0x40026000     /* 0x40026000-0x400263ff DMA1 */
#define STM32_DMA2_BASE      0x40026400     /* 0x40026400-0x400267ff DMA2 */
#define STM32_AES_BASE       0x50060000     /* 0x50060000-0x500603ff AES */
#define STM32_FSMC_BASE      0xa0000000     /* 0xa0000000-0xa0000fff FSMC */

/* Cortex-M4 Base Addresses *********************************************************/
/* Other registers -- see armv7-m/nvic.h for standard Cortex-M3 registers in this
 * address range
 */

#define STM32_SCS_BASE      0xe000e000
#define STM32_DEBUGMCU_BASE 0xe0042000

#endif /* __ARCH_ARM_SRC_STM32_CHIP_STM32L15XXX_MEMORYMAP_H */

