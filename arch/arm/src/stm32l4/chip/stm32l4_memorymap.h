/************************************************************************************
 * arch/arm/src/stm32l4/chip/stm32l4_memorymap.h
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_STM32L4_STM32L4_MEMORYMAP_H
#define __ARCH_ARM_SRC_STM32L4_STM32L4_MEMORYMAP_H

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* STM32L4XXX Address Blocks ********************************************************/

#define STM32L4_CODE_BASE      0x00000000     /* 0x00000000-0x1fffffff: 512Mb code block */
#define STM32L4_SRAM_BASE      0x20000000     /* 0x20000000-0x3fffffff: 512Mb sram block (48k to 256k) */
#define STM32L4_PERIPH_BASE    0x40000000     /* 0x40000000-0x5fffffff: 512Mb peripheral block */
#define STM32L4_FSMC_BASE12    0x60000000     /* 0x60000000-0x7fffffff: 512Mb FSMC bank1&2 block */
#  define STM32L4_FSMC_BANK1   0x60000000     /* 0x60000000-0x6fffffff:   256Mb NOR/SRAM */
#  define STM32L4_FSMC_BANK2   0x70000000     /* 0x70000000-0x7fffffff:   256Mb NAND FLASH */
#define STM32L4_FSMC_BASE34    0x80000000     /* 0x80000000-0x8fffffff: 512Mb FSMC bank3 / QSPI  block */
#  define STM32L4_FSMC_BANK3   0x80000000     /* 0x80000000-0x8fffffff:   256Mb NAND FLASH */
#  define STM32L4_QSPI_BANK    0x90000000     /* 0x90000000-0x9fffffff:   256Mb QUADSPI */
#define STM32L4_FSMC_BASE      0xa0000000     /* 0xa0000000-0xbfffffff:       FSMC register block */
#define STM32L4_QSPI_BASE      0xa0001000     /* 0xa0001000-0xbfffffff:       QSPI register block */
#define STM32L4_OCTOSPI1_BASE  0xa0001000     /* 0xa0001000-0xa00013ff: OCTOSPI1 register block */
#define STM32L4_OCTOSPI2_BASE  0xa0001400     /* 0xa0001400-0xa00017ff: OCTOSPI2 register block */
                                              /* 0xc0000000-0xdfffffff: 512Mb (not used) */
#define STM32L4_CORTEX_BASE    0xe0000000     /* 0xe0000000-0xffffffff: 512Mb Cortex-M4 block */

#define STM32L4_REGION_MASK    0xf0000000
#define STM32L4_IS_SRAM(a)     ((((uint32_t)(a)) & STM32L4_REGION_MASK) == STM32L4_SRAM_BASE)
#define STM32L4_IS_EXTSRAM(a)  ((((uint32_t)(a)) & STM32L4_REGION_MASK) == STM32L4_FSMC_BANK1)

/* Code Base Addresses **************************************************************/

#define STM32L4_BOOT_BASE      0x00000000     /* 0x00000000-0x000fffff: Aliased boot memory */
                                              /* 0x00100000-0x07ffffff: Reserved */
#define STM32L4_FLASH_BASE     0x08000000     /* 0x08000000-0x080fffff: FLASH memory */
                                              /* 0x08100000-0x0fffffff: Reserved */
#define STM32L4_SRAM2_BASE     0x10000000     /* 0x10000000-0x1000ffff: 16k to 64k SRAM2 */
                                              /* 0x10010000-0x1ffeffff: Reserved */
#define STM32L4_SRAM3_BASE     0x20040000     /* 0x20040000-0x3fffffff: SRAM3 (STM32L4R9xx only, 384k) */
#define STM32L4_SYSMEM_BASE    0x1fff0000     /* 0x1fff0000-0x1fff6fff: System memory */
#define STM32L4_OTP_BASE       0x1fff7000     /* 0x1fff7000-0x1fff73ff: OTP memory */
                                              /* 0x1fff7400-0x1fff77ff: Reserved */
#define STM32L4_OPTION_BASE    0x1fff7800     /* 0x1fff7800-0x1fff780f: Option bytes */
                                              /* 0x1fff7810-0x1ffff7ff: Reserved */
#define STM32L4_OPTION2_BASE   0x1ffff800     /* 0x1ffff800-0x1ffff80f: Option bytes 2 */
                                              /* 0x1ffff810-0x1fffffff: Reserved */

/* System Memory Addresses **********************************************************/

#define STM32L4_SYSMEM_UID     0x1fff7590     /* The 96-bit unique device identifier */
#define STM32L4_SYSMEM_FSIZE   0x1fff75E0     /* This bitfield indicates the size of
                                               * the device Flash memory expressed in
                                               * Kbytes. Example: 0x0400 corresponds
                                               * to 1024 Kbytes.
                                               */
#define STM32L4_SYSMEM_PACKAGE 0x1fff7500     /* This bitfield indicates the package
                                               * type.
                                               * 0:  LQFP64
                                               * 1:  WLCSP64
                                               * 2:  LQFP100
                                               * 3:  UFBGA132
                                               * 4:  LQFP144, WLCSP81 or WLCSP72
                                               * 10: WLCSP49
                                               * 11: UFBGA64
                                               * 12: UFBGA100
                                               * 16: UFBGA169
                                               * 17: WLCSP100
                                               */

/* SRAM Base Addresses **************************************************************/

                                            /* 0x20000000-0x2000ffff: 64k aliased by bit-banding */
                                            /* 0x2001c000-0x2001ffff: 16Kb aliased by bit-banding */
#define STM32L4_SRAMBB_BASE    0x22000000     /* 0x22000000-          : SRAM bit-band region */

/* Peripheral Base Addresses ********************************************************/

#define STM32L4_APB1_BASE      0x40000000     /* 0x40000000-0x400097ff: APB1 */
                                              /* 0x40009800-0x4000ffff: Reserved */
#define STM32L4_APB2_BASE      0x40010000     /* 0x40010000-0x400163ff: APB2 */
                                              /* 0x40016400-0x4001ffff: Reserved */
#define STM32L4_AHB1_BASE      0x40020000     /* 0x40020000-0x400243ff: APB1 */
                                              /* 0x40024400-0x47ffffff: Reserved */
#define STM32L4_AHB2_BASE      0x48000000     /* 0x48000000-0x50060bff: AHB2 */
                                              /* 0x50060c00-0x5fffffff: Reserved */

/* FSMC/QSPI Base Addresses **************************************************************/

#define STM32L4_AHB3_BASE      0x60000000     /* 0x60000000-0xa0000fff: AHB3 */

/* in datasheet order */

/* APB1 Base Addresses **************************************************************/

#define STM32L4_LPTIM2_BASE     0x40009400
#define STM32L4_SWPMI1_BASE     0x40008800
#define STM32L4_I2C4_BASE       0x40008400
#define STM32L4_LPUART1_BASE    0x40008000
#define STM32L4_LPTIM1_BASE     0x40007c00
#define STM32L4_OPAMP_BASE      0x40007800
#define STM32L4_DAC_BASE        0x40007400
#define STM32L4_PWR_BASE        0x40007000
#if defined(CONFIG_STM32L4_STM32L4X3)
#  define STM32L4_USB_SRAM_BASE 0x40006c00
#  define STM32L4_USB_FS_BASE   0x40006800
#else
#  define STM32L4_CAN2_BASE     0x40006800
#endif
#define STM32L4_CAN1_BASE       0x40006400
#define STM32L4_CRS_BASE        0x40006000
#define STM32L4_I2C3_BASE       0x40005c00
#define STM32L4_I2C2_BASE       0x40005800
#define STM32L4_I2C1_BASE       0x40005400
#define STM32L4_UART5_BASE      0x40005000
#define STM32L4_UART4_BASE      0x40004c00
#define STM32L4_USART3_BASE     0x40004800
#define STM32L4_USART2_BASE     0x40004400
#define STM32L4_SPI3_BASE       0x40003c00
#define STM32L4_SPI2_BASE       0x40003800
#define STM32L4_IWDG_BASE       0x40003000
#define STM32L4_WWDG_BASE       0x40002c00
#define STM32L4_RTC_BASE        0x40002800
#define STM32L4_LCD_BASE        0x40002400
#define STM32L4_TIM7_BASE       0x40001400
#define STM32L4_TIM6_BASE       0x40001000
#define STM32L4_TIM5_BASE       0x40000c00
#define STM32L4_TIM4_BASE       0x40000800
#define STM32L4_TIM3_BASE       0x40000400
#define STM32L4_TIM2_BASE       0x40000000

/* APB2 Base Addresses **************************************************************/

#define STM32L4_DSI_BASE        0x40016c00
#define STM32L4_LTDC_BASE       0x40016800
#define STM32L4_DFSDM_BASE      0x40016000
#define STM32L4_SAI2_BASE       0x40015800
#define STM32L4_SAI1_BASE       0x40015400
#define STM32L4_TIM17_BASE      0x40014800
#define STM32L4_TIM16_BASE      0x40014400
#define STM32L4_TIM15_BASE      0x40014000
#define STM32L4_USART1_BASE     0x40013800
#define STM32L4_TIM8_BASE       0x40013400
#define STM32L4_SPI1_BASE       0x40013000
#define STM32L4_TIM1_BASE       0x40012c00
#ifndef CONFIG_STM32L4_STM32L4XR
#  define STM32L4_SDMMC1_BASE   0x40012800
#endif
#define STM32L4_FIREWALL_BASE   0x40011c00
#define STM32L4_EXTI_BASE       0x40010400
#define STM32L4_COMP_BASE       0x40010200
#define STM32L4_VREFBUF_BASE    0x40010030
#define STM32L4_SYSCFG_BASE     0x40010000

/* AHB1 Base Addresses **************************************************************/

#define STM32L4_GFXMMU_BASE     0x4002c000
#define STM32L4_DMA2D_BASE      0x4002b000
#define STM32L4_TSC_BASE        0x40024000
#define STM32L4_CRC_BASE        0x40023000
#define STM32L4_FLASHIF_BASE    0x40022000
#define STM32L4_RCC_BASE        0x40021000
#define STM32L4_DMAMUX1_BASE    0x40020800
#define STM32L4_DMA2_BASE       0x40020400
#define STM32L4_DMA1_BASE       0x40020000

/* AHB2 Base Addresses **************************************************************/

#ifdef CONFIG_STM32L4_STM32L4XR
#  define STM32L4_SDMMC1_BASE   0x50062400
#endif
#define STM32L4_OCTOSPIIOM_BASE 0x50061c00
#define STM32L4_RNG_BASE        0x50060800
#define STM32L4_HASH_BASE       0x50060400
#define STM32L4_AES_BASE        0x50060000
#define STM32L4_DCMI_BASE       0x50050000
#define STM32L4_ADC_BASE        0x50040000
#  define STM32L4_ADC1_BASE     0x50040000     /*                        ADC1 */
#  define STM32L4_ADC2_BASE     0x50040100     /*                        ADC2 */
#  define STM32L4_ADC3_BASE     0x50040200     /*                        ADC3 */
#  define STM32L4_ADCCMN_BASE   0x50040300     /*                        Common */
#define STM32L4_OTGFS_BASE      0x50000000
#define STM32L4_GPIOI_BASE      0x48002000
#define STM32L4_GPIOH_BASE      0x48001c00
#define STM32L4_GPIOG_BASE      0x48001800
#define STM32L4_GPIOF_BASE      0x48001400
#define STM32L4_GPIOE_BASE      0x48001000
#define STM32L4_GPIOD_BASE      0x48000c00
#define STM32L4_GPIOC_BASE      0x48000800
#define STM32L4_GPIOB_BASE      0x48000400
#define STM32L4_GPIOA_BASE      0x48000000

/* Cortex-M4 Base Addresses *********************************************************/
/* Other registers -- see armv7-m/nvic.h for standard Cortex-M3 registers in this
 * address range
 */

#define STM32L4_SCS_BASE      0xe000e000
#define STM32L4_DEBUGMCU_BASE 0xe0042000

#endif /* __ARCH_ARM_SRC_STM32L4_STM32L4_MEMORYMAP_H */

