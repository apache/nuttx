/****************************************************************************
 * arch/arm/src/stm32wb/hardware/stm32wb_memorymap.h
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

#ifndef __ARCH_ARM_SRC_STM32WB_HARDWARE_STM32WB_MEMORYMAP_H
#define __ARCH_ARM_SRC_STM32WB_HARDWARE_STM32WB_MEMORYMAP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* STM32WBXXX Address Blocks ************************************************/

#define STM32WB_CODE_BASE       0x00000000     /* 0x00000000-0x1fffffff: 512Mb code block */
#define STM32WB_SRAM_BASE       0x20000000     /* 0x20000000-0x2003ffff: 256k RAM block */
#define STM32WB_PERIPH_BASE     0x40000000     /* Peripheral base address */
#define STM32WB_CORTEX_BASE     0xe0000000     /* 0xe0000000-0xffffffff: 512Mb Cortex-M4 block */

/* Code Base Addresses ******************************************************/

#define STM32WB_BOOT_BASE       0x00000000     /* 0x00000000-0x000fffff: Aliased boot memory */
#define STM32WB_FLASH_BASE      0x08000000     /* 0x08000000-0x080fffff: FLASH memory */
#define STM32WB_FLASH_MASK      0xf8000000     /* Test if addr in FLASH */
#define STM32WB_SRAM1_BASE      0x20000000     /* 0x20000000-0x2002ffff: 192к RAM1 block */
#define STM32WB_SRAM2A_BASE     0x20030000     /* 0x20030000-0x20037fff: 32k RAM2a block */
#define STM32WB_SRAM2B_BASE     0x20038000     /* 0x20038000-0x2003ffff: 32k RAM2b block */

#define STM32WB_SYSMEM_BASE     0x1fff0000     /* 0x1fff0000-0x20006fff: System memory */
#define STM32WB_OTP_BASE        0x1fff7000     /* 0x1fff7000-0x1fff73ff: OTP memory */
#define STM32WB_OPTION_BASE     0x1fff8000     /* 0x1fff8000-0x1fff8fff: Option bytes */

/* System Memory Addresses **************************************************/

#define STM32WB_SYSMEM_UID      0x1fff7590     /* The 96-bit unique device identifier */
#define STM32WB_SYSMEM_FSIZE    0x1fff75e0     /* This bitfield indicates the size of
                                                * the device Flash memory expressed in
                                                * Kbytes. Example: 0x0400 corresponds
                                                * to 1024 Kbytes.
                                                */
#define STM32WB_SYSMEM_PACKAGE  0x1fff7500     /* This bitfield indicates the package
                                                * type. 5 LSB corresponds to:
                                                * 0x11:  WLCSP100 / UFBGA129
                                                * 0x13:  VFQFPN68
                                                * 0x0A:  UFQPFN48
                                                */

/* SRAM Base Addresses ******************************************************/

#define STM32WB_SRAMBB_BASE     0x22000000     /* 0x22000000-0x227fffff: SRAM bit-band region */

/* Peripheral Base Addresses ************************************************/

#define STM32WB_APB1_BASE       0x40000000     /* 0x40000000-0x400097ff: APB1 */
                                               /* 0x40009800-0x4000ffff: Reserved */
#define STM32WB_APB2_BASE       0x40010000     /* 0x40010000-0x400157ff: APB2 */
                                               /* 0x40015800-0x4001ffff: Reserved */
#define STM32WB_AHB1_BASE       0x40020000     /* 0x40020000-0x400243ff: AHB1 */
                                               /* 0x40024400-0x47ffffff: Reserved */
#define STM32WB_AHB2_BASE       0x48000000     /* 0x48000000-0x500603ff: AHB2 */
                                               /* 0x50060400-0x57ffffff: Reserved */
#define STM32WB_AHB4_BASE       0x58000000     /* 0x58000000-0x580043ff: AHB4 */
                                               /* 0x58004400-0x5fffffff: Reserved */
#define STM32WB_APB3_BASE       0x60000000     /* 0x60000000-0x60001fff: APB3 */
                                               /* 0x60002000-0x8fffffff: Reserved */
#define STM32WB_AHB3_BASE       0x90000000     /* 0x90000000-0xA00013ff: AHB3 */

/* APB1 Base Addresses ******************************************************/

#define STM32WB_TIM2_BASE       0x40000000
#define STM32WB_LCD_BASE        0x40002400
#define STM32WB_RTC_BASE        0x40002800
#define STM32WB_WWDG_BASE       0x40002c00
#define STM32WB_IWDG_BASE       0x40003000
#define STM32WB_SPI2_BASE       0x40003800
#define STM32WB_I2C1_BASE       0x40005400
#define STM32WB_I2C3_BASE       0x40005c00
#define STM32WB_CRS_BASE        0x40006000
#define STM32WB_USB1_BASE       0x40006800
#define STM32WB_USB1_PMAADDR    0x40006c00
#define STM32WB_LPTIM1_BASE     0x40007c00
#define STM32WB_LPUART1_BASE    0x40008000
#define STM32WB_LPTIM2_BASE     0x40009400

/* APB2 Base Addresses ******************************************************/

#define STM32WB_SYSCFG_BASE     0x40010000
#define STM32WB_VREFBUF_BASE    0x40010030
#define STM32WB_COMP1_BASE      0x40010200
#define STM32WB_COMP2_BASE      0x40010204
#define STM32WB_TIM1_BASE       0x40012c00
#define STM32WB_SPI1_BASE       0x40013000
#define STM32WB_USART1_BASE     0x40013800
#define STM32WB_TIM16_BASE      0x40014400
#define STM32WB_TIM17_BASE      0x40014800
#define STM32WB_SAI1_BASE       0x40015400

/* AHB1 Base Addresses ******************************************************/

#define STM32WB_DMA1_BASE       0x40020000
#define STM32WB_DMA2_BASE       0x40020400
#define STM32WB_DMAMUX1_BASE    0x40020800
#define STM32WB_CRC_BASE        0x40023000
#define STM32WB_TSC_BASE        0x40024000

/* AHB2 Base Addresses ******************************************************/

#define STM32WB_GPIOA_BASE      0x48000000
#define STM32WB_GPIOB_BASE      0x48000400
#define STM32WB_GPIOC_BASE      0x48000800
#define STM32WB_GPIOD_BASE      0x48000c00
#define STM32WB_GPIOE_BASE      0x48001000
#define STM32WB_GPIOH_BASE      0x48001c00
#define STM32WB_ADC1_BASE       0x50040000
#define STM32WB_AES1_BASE       0x50060000

/* AHB4 Base Addresses ******************************************************/

#define STM32WB_RCC_BASE        0x58000000
#define STM32WB_PWR_BASE        0x58000400
#define STM32WB_EXTI_BASE       0x58000800
#define STM32WB_IPCC_BASE       0x58000c00
#define STM32WB_RNG_BASE        0x58001000
#define STM32WB_HSEM_BASE       0x58001400
#define STM32WB_AES2_BASE       0x58001800
#define STM32WB_PKA_BASE        0x58002000
#define STM32WB_FLASHREG_BASE   0x58004000

/* APB3 Base Addresses ******************************************************/

#define STM32WB_BLE_BASE        0x60000000
#define STM32WB_RADIO_BASE      0x60000400
#define STM32WB_802154_BASE     0x60001000

/* AHB3 Base Addresses ******************************************************/

#define STM32WB_QSPI_BASE       0x90000000
#define STM32WB_QSPI_BANK       0x90000000     /* 0x90000000-0x9fffffff: 256Mb QSPI memory mapping */
#define STM32WB_QSPIREF_BASE    0xa0001000

/* Cortex-M4 Base Addresses *************************************************/

/* Other registers -- see armv7-m/nvic.h for standard Cortex-M4 registers in
 * this address range
 */

#define STM32WB_SCS_BASE        0xe000e000
#define STM32WB_DEBUGMCU_BASE   0xe0042000

#endif /* __ARCH_ARM_SRC_STM32WB_HARDWARE_STM32WB_MEMORYMAP_H */
