/****************************************************************************
 * arch/arm/src/efm32/hardware/efm32g_memorymap.h
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

/* Reference: "EFM32G Reference Manual, Gecko Series", Energy Micro */

#ifndef __ARCH_ARM_SRC_EFM32_HARDWARE_EFM32G_MEMORYMAP_H
#define __ARCH_ARM_SRC_EFM32_HARDWARE_EFM32G_MEMORYMAP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Memory Base Addresses */

#define EFM32_FLASH_MEM_BASE       0x00000000 /* FLASH base address */
#define EFM32_RAM_CODE_MEM_BASE    0x10000000 /* RAM_CODE base address */
#define EFM32_RAM_MEM_BASE         0x20000000 /* RAM base address */
#define EFM32_PER_MEM_BASE         0x40000000 /* PER base address */
#define EFM32_AES_MEM_BASE         0x400e0000 /* AES base address */
#define EFM32_EBI0_BASE            0x80000000 /* EBI Region 0 */
#define EFM32_EBI1_BASE            0x84000000 /* EBI Region 0 */
#define EFM32_EBI2_BASE            0x88000000 /* EBI Region 0 */
#define EFM32_EBI3_BASE            0x8c000000 /* EBI Region 0 */
#define EFM32_CM3_BASE             0xe0000000 /* CM3 Peripherals */

/* Bit banding area */

#define EFM32_BITBAND_PER_BASE     0x42000000 /* Peripheral Address Space bit-band area */
#define EFM32_BITBAND_RAM_BASE     0x22000000 /* SRAM Address Space bit-band area */

/* Flash and SRAM Addresses */

#define EFM32_FLASH_BASE           0x00000000 /* Flash Base Address */
#define EFM32_SRAM_BASE            0x20000000 /* SRAM Base Address */

/* Peripheral Base Addresses */

#define EFM32_VCMP_BASE            0x40000000 /* VCMP base address */
#define EFM32_ACMP0_BASE           0x40001000 /* ACMP0 base address */
#define EFM32_ACMP1_BASE           0x40001400 /* ACMP1 base address */
#define EFM32_ADC0_BASE            0x40002000 /* ADC0 base address */
#define EFM32_DAC0_BASE            0x40004000 /* DAC0 base address */
#define EFM32_GPIO_BASE            0x40006000 /* GPIO base address */
#define EFM32_I2C0_BASE            0x4000a000 /* I2C0 base address */
#define EFM32_USART0_BASE          0x4000c000 /* USART0 base address */
#define EFM32_USART1_BASE          0x4000c400 /* USART1 base address */
#define EFM32_USART2_BASE          0x4000c800 /* USART2 base address */
#define EFM32_UART0_BASE           0x4000e000 /* UART0 base address */
#define EFM32_TIMER0_BASE          0x40010000 /* TIMER0 base address */
#define EFM32_TIMER1_BASE          0x40010400 /* TIMER1 base address */
#define EFM32_TIMER2_BASE          0x40010800 /* TIMER2 base address */
#define EFM32_RTC_BASE             0x40080000 /* RTC base address */
#define EFM32_LETIMER0_BASE        0x40082000 /* LETIMER0 base address */
#define EFM32_LEUART0_BASE         0x40084000 /* LEUART0 base address */
#define EFM32_LEUART1_BASE         0x40084400 /* LEUART0 base address */
#define EFM32_PCNT0_BASE           0x40086000 /* PCNT0 base address */
#define EFM32_PCNT1_BASE           0x40086400 /* PCNT1 base address */
#define EFM32_PCNT2_BASE           0x40086800 /* PCNT2 base address */
#define EFM32_WDOG_BASE            0x40088000 /* WDOG base address */
#define EFM32_LCD_BASE             0x4008a000 /* LCD base address */
#define EFM32_MSC_BASE             0x400c0000 /* MSC base address */
#define EFM32_DMA_BASE             0x400c2000 /* DMA base address */
#define EFM32_EMU_BASE             0x400c6000 /* EMU base address */
#define EFM32_CMU_BASE             0x400c8000 /* CMU base address */
#define EFM32_RMU_BASE             0x400ca000 /* RMU base address */
#define EFM32_PRS_BASE             0x400cc000 /* PRS base address */
#define EFM32_AES_BASE             0x400e0000 /* AES base address */

#endif /* __ARCH_ARM_SRC_EFM32_HARDWARE_EFM32G_MEMORYMAP_H */
