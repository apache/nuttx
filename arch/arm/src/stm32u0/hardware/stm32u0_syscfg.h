/****************************************************************************
 * arch/arm/src/stm32u0/hardware/stm32u0_syscfg.h
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

#ifndef __ARCH_ARM_SRC_STM32U0_HARDWARE_STM32U0_SYSCFG_H
#define __ARCH_ARM_SRC_STM32U0_HARDWARE_STM32U0_SYSCFG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_SYSCFG_CFGR1_OFFSET     0x0000 /* Configuration register 1 */
#define STM32_SYSCFG_CFGR2_OFFSET     0x0018 /* Configuration register 2 */
#define STM32_SYSCFG_SCSR_OFFSET      0x001c /* Backup SRAM control and status register */
#define STM32_SYSCFG_SKR_OFFSET       0x0020 /* Backup SRAM key register */
#define STM32_SYSCFG_TSCCR_OFFSET     0x0024 /* TSC comparator register */
#define STM32_SYSCFG_ITLINE0_OFFSET   0x0080 /* Interrupt line 0 status */
#define STM32_SYSCFG_ITLINE1_OFFSET   0x0084 /* Interrupt line 1 status */
#define STM32_SYSCFG_ITLINE2_OFFSET   0x0088 /* Interrupt line 2 status */
#define STM32_SYSCFG_ITLINE3_OFFSET   0x008c /* Interrupt line 3 status */
#define STM32_SYSCFG_ITLINE4_OFFSET   0x0090 /* Interrupt line 4 status */
#define STM32_SYSCFG_ITLINE5_OFFSET   0x0094 /* Interrupt line 5 status */
#define STM32_SYSCFG_ITLINE6_OFFSET   0x0098 /* Interrupt line 6 status */
#define STM32_SYSCFG_ITLINE7_OFFSET   0x009c /* Interrupt line 7 status */
#define STM32_SYSCFG_ITLINE8_OFFSET   0x00a0 /* Interrupt line 8 status */
#define STM32_SYSCFG_ITLINE9_OFFSET   0x00a4 /* Interrupt line 9 status */
#define STM32_SYSCFG_ITLINE10_OFFSET  0x00a8 /* Interrupt line 10 status */
#define STM32_SYSCFG_ITLINE11_OFFSET  0x00ac /* Interrupt line 11 status */
#define STM32_SYSCFG_ITLINE12_OFFSET  0x00b0 /* Interrupt line 12 status */
#define STM32_SYSCFG_ITLINE13_OFFSET  0x00b4 /* Interrupt line 13 status */
#define STM32_SYSCFG_ITLINE14_OFFSET  0x00b8 /* Interrupt line 14 status */
#define STM32_SYSCFG_ITLINE15_OFFSET  0x00bc /* Interrupt line 15 status */
#define STM32_SYSCFG_ITLINE16_OFFSET  0x00c0 /* Interrupt line 16 status */
#define STM32_SYSCFG_ITLINE17_OFFSET  0x00c4 /* Interrupt line 17 status */
#define STM32_SYSCFG_ITLINE18_OFFSET  0x00c8 /* Interrupt line 18 status */
#define STM32_SYSCFG_ITLINE19_OFFSET  0x00cc /* Interrupt line 19 status */
#define STM32_SYSCFG_ITLINE20_OFFSET  0x00d0 /* Interrupt line 20 status */
#define STM32_SYSCFG_ITLINE21_OFFSET  0x00d4 /* Interrupt line 21 status */
#define STM32_SYSCFG_ITLINE22_OFFSET  0x00d8 /* Interrupt line 22 status */
#define STM32_SYSCFG_ITLINE23_OFFSET  0x00dc /* Interrupt line 23 status */
#define STM32_SYSCFG_ITLINE24_OFFSET  0x00e0 /* Interrupt line 24 status */
#define STM32_SYSCFG_ITLINE25_OFFSET  0x00e4 /* Interrupt line 25 status */
#define STM32_SYSCFG_ITLINE26_OFFSET  0x00e8 /* Interrupt line 26 status */
#define STM32_SYSCFG_ITLINE27_OFFSET  0x00ec /* Interrupt line 27 status */
#define STM32_SYSCFG_ITLINE28_OFFSET  0x00f0 /* Interrupt line 28 status */
#define STM32_SYSCFG_ITLINE29_OFFSET  0x00f4 /* Interrupt line 29 status */
#define STM32_SYSCFG_ITLINE30_OFFSET  0x00f8 /* Interrupt line 30 status */
#define STM32_SYSCFG_ITLINE31_OFFSET  0x00fc /* Interrupt line 31 status */

/* Register Addresses *******************************************************/

#define STM32_SYSCFG_CFGR1            (STM32_SYSCFG_BASE + STM32_SYSCFG_CFGR1_OFFSET)
#define STM32_SYSCFG_CFGR2            (STM32_SYSCFG_BASE + STM32_SYSCFG_CFGR2_OFFSET)
#define STM32_SYSCFG_SCSR             (STM32_SYSCFG_BASE + STM32_SYSCFG_SCSR_OFFSET)
#define STM32_SYSCFG_SKR              (STM32_SYSCFG_BASE + STM32_SYSCFG_SKR_OFFSET)
#define STM32_SYSCFG_TSCCR            (STM32_SYSCFG_BASE + STM32_SYSCFG_TSCCR_OFFSET)
#define STM32_SYSCFG_ITLINE(n)        (STM32_SYSCFG_BASE +STM32_SYSCFG_ITLINE0_OFFSET + ((n) << 2))
#define STM32_SYSCFG_ITLINE0          (STM32_SYSCFG_BASE +STM32_SYSCFG_ITLINE0_OFFSET)
#define STM32_SYSCFG_ITLINE1          (STM32_SYSCFG_BASE +STM32_SYSCFG_ITLINE1_OFFSET)
#define STM32_SYSCFG_ITLINE2          (STM32_SYSCFG_BASE +STM32_SYSCFG_ITLINE2_OFFSET)
#define STM32_SYSCFG_ITLINE3          (STM32_SYSCFG_BASE +STM32_SYSCFG_ITLINE3_OFFSET)
#define STM32_SYSCFG_ITLINE4          (STM32_SYSCFG_BASE +STM32_SYSCFG_ITLINE4_OFFSET)
#define STM32_SYSCFG_ITLINE5          (STM32_SYSCFG_BASE +STM32_SYSCFG_ITLINE5_OFFSET)
#define STM32_SYSCFG_ITLINE6          (STM32_SYSCFG_BASE +STM32_SYSCFG_ITLINE6_OFFSET)
#define STM32_SYSCFG_ITLINE7          (STM32_SYSCFG_BASE +STM32_SYSCFG_ITLINE7_OFFSET)
#define STM32_SYSCFG_ITLINE8          (STM32_SYSCFG_BASE +STM32_SYSCFG_ITLINE8_OFFSET)
#define STM32_SYSCFG_ITLINE9          (STM32_SYSCFG_BASE +STM32_SYSCFG_ITLINE9_OFFSET)
#define STM32_SYSCFG_ITLINE10         (STM32_SYSCFG_BASE +STM32_SYSCFG_ITLINE10_OFFSET)
#define STM32_SYSCFG_ITLINE11         (STM32_SYSCFG_BASE +STM32_SYSCFG_ITLINE11_OFFSET)
#define STM32_SYSCFG_ITLINE12         (STM32_SYSCFG_BASE +STM32_SYSCFG_ITLINE12_OFFSET)
#define STM32_SYSCFG_ITLINE13         (STM32_SYSCFG_BASE +STM32_SYSCFG_ITLINE13_OFFSET)
#define STM32_SYSCFG_ITLINE14         (STM32_SYSCFG_BASE +STM32_SYSCFG_ITLINE14_OFFSET)
#define STM32_SYSCFG_ITLINE15         (STM32_SYSCFG_BASE +STM32_SYSCFG_ITLINE15_OFFSET)
#define STM32_SYSCFG_ITLINE16         (STM32_SYSCFG_BASE +STM32_SYSCFG_ITLINE16_OFFSET)
#define STM32_SYSCFG_ITLINE17         (STM32_SYSCFG_BASE +STM32_SYSCFG_ITLINE17_OFFSET)
#define STM32_SYSCFG_ITLINE18         (STM32_SYSCFG_BASE +STM32_SYSCFG_ITLINE18_OFFSET)
#define STM32_SYSCFG_ITLINE19         (STM32_SYSCFG_BASE +STM32_SYSCFG_ITLINE19_OFFSET)
#define STM32_SYSCFG_ITLINE20         (STM32_SYSCFG_BASE +STM32_SYSCFG_ITLINE20_OFFSET)
#define STM32_SYSCFG_ITLINE21         (STM32_SYSCFG_BASE +STM32_SYSCFG_ITLINE21_OFFSET)
#define STM32_SYSCFG_ITLINE22         (STM32_SYSCFG_BASE +STM32_SYSCFG_ITLINE22_OFFSET)
#define STM32_SYSCFG_ITLINE23         (STM32_SYSCFG_BASE +STM32_SYSCFG_ITLINE23_OFFSET)
#define STM32_SYSCFG_ITLINE24         (STM32_SYSCFG_BASE +STM32_SYSCFG_ITLINE24_OFFSET)
#define STM32_SYSCFG_ITLINE25         (STM32_SYSCFG_BASE +STM32_SYSCFG_ITLINE25_OFFSET)
#define STM32_SYSCFG_ITLINE26         (STM32_SYSCFG_BASE +STM32_SYSCFG_ITLINE26_OFFSET)
#define STM32_SYSCFG_ITLINE27         (STM32_SYSCFG_BASE +STM32_SYSCFG_ITLINE27_OFFSET)
#define STM32_SYSCFG_ITLINE28         (STM32_SYSCFG_BASE +STM32_SYSCFG_ITLINE28_OFFSET)
#define STM32_SYSCFG_ITLINE29         (STM32_SYSCFG_BASE +STM32_SYSCFG_ITLINE29_OFFSET)
#define STM32_SYSCFG_ITLINE30         (STM32_SYSCFG_BASE +STM32_SYSCFG_ITLINE30_OFFSET)
#define STM32_SYSCFG_ITLINE31         (STM32_SYSCFG_BASE +STM32_SYSCFG_ITLINE31_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* SYSCFG configuration register 1 */

#define SYSCFG_CFGR1_MEMMODE_SHIFT    (0) /* Bits 0-1: Memory mapping selection */
#define SYSCFG_CFGR1_MEMMODE_MASK     (3 << SYSCFG_CFGR1_MEMMODE_SHIFT)
#  define SYSCFG_CFGR1_MEMMODE_FLASH  (0 << SYSCFG_CFGR1_MEMMODE_SHIFT) /* 00: Main Flash at 0x00000000 */
#  define SYSCFG_CFGR1_MEMMODE_SYSTEM (1 << SYSCFG_CFGR1_MEMMODE_SHIFT) /* 01: System Flash at 0x00000000 */
#  define SYSCFG_CFGR1_MEMMODE_SRAM   (3 << SYSCFG_CFGR1_MEMMODE_SHIFT) /* 11: Embedded SRAM at 0x00000000 */
                                                                        /* Bit 2: Reserved */
#define SYSCFG_CFGR1_PA11_RMP         (1 << 3)                          /* Bit 3: PA11 remapping */
#define SYSCFG_CFGR1_PA12_RMP         (1 << 4)                          /* Bit 4: PA12 remapping */
#define SYSCFG_CFGR1_IRPOL            (1 << 5)                          /* Bit 5: IR output polarity selection */
#define SYSCFG_CFGR1_IRMOD_SHIFT      (6)                               /* Bits 6-7: IR mode */
#define SYSCFG_CFGR1_IRMOD_MASK       (3 << SYSCFG_CFGR1_IRMOD_SHIFT)
#  define SYSCFG_CFGR1_IRMOD_TIM16    (0 << SYSCFG_CFGR1_IRMOD_SHIFT) /* 00: TIM16 selected */
#  define SYSCFG_CFGR1_IRMOD_USART1   (1 << SYSCFG_CFGR1_IRMOD_SHIFT) /* 01: USART1 selected */
#  define SYSCFG_CFGR1_IRMOD_USART4   (2 << SYSCFG_CFGR1_IRMOD_SHIFT) /* 10: USART4 selected */
#define SYSCFG_CFGR1_BOOSTEN          (1 << 8)                        /* Bit 8: IO analog switch voltage booster enable */
                                                                      /* Bits 9-15: Reserved */
#define SYSCFG_CFGR1_I2CPB6FMP        (1 << 16)                       /* Bit 16: PB6 FM+ */
#define SYSCFG_CFGR1_I2CPB7FMP        (1 << 17)                       /* Bit 17: PB7 FM+ */
#define SYSCFG_CFGR1_I2CPB8FMP        (1 << 18)                       /* Bit 18: PB8 FM+ */
#define SYSCFG_CFGR1_I2CPB9FMP        (1 << 19)                       /* Bit 19: PB9 FM+ */
                                                                      /* Bits 20-21: Reserved */
#define SYSCFG_CFGR1_I2CPA9FMP        (1 << 22)                       /* Bit 22: PA9 FM+ */
#define SYSCFG_CFGR1_I2CPA10FMP       (1 << 23)                       /* Bit 23: PA10 FM+ */
#define SYSCFG_CFGR1_I2C3FMP          (1 << 24)                       /* Bit 24: I2C3 FM+ */
                                                                      /* Bits 25-31: Reserved */

/* SYSCFG configuration register 2 */

#define SYSCFG_CFGR2_CCL              (1 << 0) /* Bit 0: Cortex-M0+ LOCKUP enable */
#define SYSCFG_CFGR2_SPL              (1 << 1) /* Bit 1: SRAM parity lock */
#define SYSCFG_CFGR2_PVDL             (1 << 2) /* Bit 2: PVD lock */
#define SYSCFG_CFGR2_ECCL             (1 << 3) /* Bit 3: ECC lock */
#define SYSCFG_CFGR2_BKPL             (1 << 4) /* Bit 4: Backup SRAM parity lock */
                                               /* Bits 5-6: Reserved */
#define SYSCFG_CFGR2_BKPF             (1 << 7) /* Bit 7: Backup SRAM parity error flag */
#define SYSCFG_CFGR2_SPF              (1 << 8) /* Bit 8: SRAM parity error flag */
                                               /* Bits 9-31: Reserved */

/* SYSCFG backup SRAM control and status register */

#define SYSCFG_SCSR_SRAM2ER           (1 << 0) /* Bit 0: Backup SRAM erase */
#define SYSCFG_SCSR_SRAM2BSY          (1 << 1) /* Bit 1: Backup SRAM erase busy */

/* SYSCFG backup SRAM key register */

#define SYSCFG_SKR_KEY_SHIFT          (0) /* Bits 0-7: Backup SRAM erase key */
#define SYSCFG_SKR_KEY_MASK           (0xff << SYSCFG_SKR_KEY_SHIFT)

/* SYSCFG TSC comparator register */

#define SYSCFG_TSCCR_G2IO1            (1 << 0) /* Bit 0: G2_IO1 (PB4) */
#define SYSCFG_TSCCR_G2IO3            (1 << 1) /* Bit 1: G2_IO3 (PB6) */
#define SYSCFG_TSCCR_G4IO1            (1 << 2) /* Bit 2: G4_IO1 (PC6) */
#define SYSCFG_TSCCR_G6IO1            (1 << 3) /* Bit 3: G6_IO1 (PD10) */
#define SYSCFG_TSCCR_G7IO2            (1 << 4) /* Bit 4: G7_IO2 (PA9) */
#define SYSCFG_TSCCR_TSCIOCTRL        (1 << 5) /* Bit 5: Comparator mode control */

/* SYSCFG interrupt line 0 status register */

#define SYSCFG_ITLINE0_WWDG           (1 << 0) /* Bit 0: WWDG pending */

/* SYSCFG interrupt line 1 status register */

#define SYSCFG_ITLINE1_PVDOUT         (1 << 0) /* Bit 0: PVD pending */
#define SYSCFG_ITLINE1_PVMOUT1        (1 << 1) /* Bit 1: PVM1 pending */
#define SYSCFG_ITLINE1_PVMOUT3        (1 << 2) /* Bit 2: PVM3 pending */
#define SYSCFG_ITLINE1_PVMOUT4        (1 << 3) /* Bit 3: PVM4 pending */

/* SYSCFG interrupt line 2 status register */

#define SYSCFG_ITLINE2_TAMPER         (1 << 0) /* Bit 0: TAMP pending */
#define SYSCFG_ITLINE2_RTC            (1 << 1) /* Bit 1: RTC pending */

/* SYSCFG interrupt line 3 status register */

#define SYSCFG_ITLINE3_FLASH_ECC      (1 << 0) /* Bit 0: Flash ECC pending */
#define SYSCFG_ITLINE3_FLASH_ITF      (1 << 1) /* Bit 1: Flash interface pending */

/* SYSCFG interrupt line 4 status register */

#define SYSCFG_ITLINE4_RCC            (1 << 0) /* Bit 0: RCC pending */
#define SYSCFG_ITLINE4_CRS            (1 << 1) /* Bit 1: CRS pending */

/* SYSCFG interrupt line 5 status register */

#define SYSCFG_ITLINE5_EXTI0          (1 << 0) /* Bit 0: EXTI 0 pending */
#define SYSCFG_ITLINE5_EXTI1          (1 << 1) /* Bit 1: EXTI 1 pending */

/* SYSCFG interrupt line 6 status register */

#define SYSCFG_ITLINE6_EXTI2          (1 << 0) /* Bit 0: EXTI 2 pending */
#define SYSCFG_ITLINE6_EXTI3          (1 << 1) /* Bit 1: EXTI 3 pending */

/* SYSCFG interrupt line 7 status register */

#define SYSCFG_ITLINE7_EXTI4          (1 << 0)  /* Bit 0: EXTI 4 pending */
#define SYSCFG_ITLINE7_EXTI5          (1 << 1)  /* Bit 1: EXTI 5 pending */
#define SYSCFG_ITLINE7_EXTI6          (1 << 2)  /* Bit 2: EXTI 6 pending */
#define SYSCFG_ITLINE7_EXTI7          (1 << 3)  /* Bit 3: EXTI 7 pending */
#define SYSCFG_ITLINE7_EXTI8          (1 << 4)  /* Bit 4: EXTI 8 pending */
#define SYSCFG_ITLINE7_EXTI9          (1 << 5)  /* Bit 5: EXTI 9 pending */
#define SYSCFG_ITLINE7_EXTI10         (1 << 6)  /* Bit 6: EXTI 10 pending */
#define SYSCFG_ITLINE7_EXTI11         (1 << 7)  /* Bit 7: EXTI 11 pending */
#define SYSCFG_ITLINE7_EXTI12         (1 << 8)  /* Bit 8: EXTI 12 pending */
#define SYSCFG_ITLINE7_EXTI13         (1 << 9)  /* Bit 9: EXTI 13 pending */
#define SYSCFG_ITLINE7_EXTI14         (1 << 10) /* Bit 10: EXTI 14 pending */
#define SYSCFG_ITLINE7_EXTI15         (1 << 11) /* Bit 11: EXTI 15 pending */

/* SYSCFG interrupt line 8 status register */

#define SYSCFG_ITLINE8_USBFS          (1 << 0) /* Bit 0: USB FS pending */

/* SYSCFG interrupt line 9 status register */

#define SYSCFG_ITLINE9_DMA1_CH1       (1 << 0) /* Bit 0: DMA1 CH1 pending */

/* SYSCFG interrupt line 10 status register */

#define SYSCFG_ITLINE10_DMA1_CH2      (1 << 0) /* Bit 0: DMA1 CH2 pending */
#define SYSCFG_ITLINE10_DMA1_CH3      (1 << 1) /* Bit 1: DMA1 CH3 pending */

/* SYSCFG interrupt line 11 status register */

#define SYSCFG_ITLINE11_DMAMUX        (1 << 0) /* Bit 0: DMAMUX pending */
#define SYSCFG_ITLINE11_DMA1_CH4      (1 << 1) /* Bit 1: DMA1 CH4 pending */
#define SYSCFG_ITLINE11_DMA1_CH5      (1 << 2) /* Bit 2: DMA1 CH5 pending */
#define SYSCFG_ITLINE11_DMA1_CH6      (1 << 3) /* Bit 3: DMA1 CH6 pending */
#define SYSCFG_ITLINE11_DMA1_CH7      (1 << 4) /* Bit 4: DMA1 CH7 pending */
#define SYSCFG_ITLINE11_DMA2_CH1      (1 << 5) /* Bit 5: DMA2 CH1 pending */
#define SYSCFG_ITLINE11_DMA2_CH2      (1 << 6) /* Bit 6: DMA2 CH2 pending */
#define SYSCFG_ITLINE11_DMA2_CH3      (1 << 7) /* Bit 7: DMA2 CH3 pending */
#define SYSCFG_ITLINE11_DMA2_CH4      (1 << 8) /* Bit 8: DMA2 CH4 pending */
#define SYSCFG_ITLINE11_DMA2_CH5      (1 << 9) /* Bit 9: DMA2 CH5 pending */

/* SYSCFG interrupt line 12 status register */

#define SYSCFG_ITLINE12_ADC           (1 << 0) /* Bit 0: ADC pending */
#define SYSCFG_ITLINE12_COMP1         (1 << 1) /* Bit 1: COMP1 pending */
#define SYSCFG_ITLINE12_COMP2         (1 << 2) /* Bit 2: COMP2 pending */

/* SYSCFG interrupt line 13 status register */

#define SYSCFG_ITLINE13_TIM1_CCU      (1 << 0) /* Bit 0: TIM1 commutation pending */
#define SYSCFG_ITLINE13_TIM1_TRG      (1 << 1) /* Bit 1: TIM1 trigger pending */
#define SYSCFG_ITLINE13_TIM1_UPD      (1 << 2) /* Bit 2: TIM1 update pending */
#define SYSCFG_ITLINE13_TIM1_BRK      (1 << 3) /* Bit 3: TIM1 break pending */

/* SYSCFG interrupt line 14 status register */

#define SYSCFG_ITLINE14_TIM1_CC1      (1 << 0) /* Bit 0: TIM1 CC1 pending */
#define SYSCFG_ITLINE14_TIM1_CC2      (1 << 1) /* Bit 1: TIM1 CC2 pending */
#define SYSCFG_ITLINE14_TIM1_CC3      (1 << 2) /* Bit 2: TIM1 CC3 pending */
#define SYSCFG_ITLINE14_TIM1_CC4      (1 << 3) /* Bit 3: TIM1 CC4 pending */

/* SYSCFG interrupt line 15 status register */

#define SYSCFG_ITLINE15_TIM2          (1 << 0) /* Bit 0: TIM2 pending */

/* SYSCFG interrupt line 16 status register */

#define SYSCFG_ITLINE16_TIM3          (1 << 0) /* Bit 0: TIM3 pending */

/* SYSCFG interrupt line 17 status register */

#define SYSCFG_ITLINE17_TIM6          (1 << 0) /* Bit 0: TIM6 pending */
#define SYSCFG_ITLINE17_DAC           (1 << 1) /* Bit 1: DAC underrun pending */
#define SYSCFG_ITLINE17_LPTIM1        (1 << 2) /* Bit 2: LPTIM1 pending */

/* SYSCFG interrupt line 18 status register */

#define SYSCFG_ITLINE18_TIM7          (1 << 0) /* Bit 0: TIM7 pending */
#define SYSCFG_ITLINE18_LPTIM2        (1 << 1) /* Bit 1: LPTIM2 pending */

/* SYSCFG interrupt line 19 status register */

#define SYSCFG_ITLINE19_TIM15         (1 << 0) /* Bit 0: TIM15 pending */
#define SYSCFG_ITLINE19_LPTIM3        (1 << 1) /* Bit 1: LPTIM3 pending */

/* SYSCFG interrupt line 20 status register */

#define SYSCFG_ITLINE20_TIM16         (1 << 0) /* Bit 0: TIM16 pending */

/* SYSCFG interrupt line 21 status register */

#define SYSCFG_ITLINE21_TSC_MCE       (1 << 0) /* Bit 0: TSC MCE pending */
#define SYSCFG_ITLINE21_TSC_EOA       (1 << 1) /* Bit 1: TSC EOA pending */

/* SYSCFG interrupt line 22 status register */

#define SYSCFG_ITLINE22_LCD           (1 << 0) /* Bit 0: LCD pending */

/* SYSCFG interrupt line 23 status register */

#define SYSCFG_ITLINE23_I2C1          (1 << 0) /* Bit 0: I2C1 pending */

/* SYSCFG interrupt line 24 status register */

#define SYSCFG_ITLINE24_I2C2          (1 << 0) /* Bit 0: I2C2 pending */
#define SYSCFG_ITLINE24_I2C4          (1 << 1) /* Bit 1: I2C4 pending */
#define SYSCFG_ITLINE24_I2C3          (1 << 2) /* Bit 2: I2C3 pending */

/* SYSCFG interrupt line 25 status register */

#define SYSCFG_ITLINE25_SPI1          (1 << 0) /* Bit 0: SPI1 pending */

/* SYSCFG interrupt line 26 status register */

#define SYSCFG_ITLINE26_SPI2          (1 << 0) /* Bit 0: SPI2 pending */
#define SYSCFG_ITLINE26_SPI3          (1 << 1) /* Bit 1: SPI3 pending */

/* SYSCFG interrupt line 27 status register */

#define SYSCFG_ITLINE27_USART1        (1 << 0) /* Bit 0: USART1 pending */

/* SYSCFG interrupt line 28 status register */

#define SYSCFG_ITLINE28_USART2        (1 << 0) /* Bit 0: USART2 pending */
#define SYSCFG_ITLINE28_LPUART2       (1 << 1) /* Bit 1: LPUART2 pending */

/* SYSCFG interrupt line 29 status register */

#define SYSCFG_ITLINE29_USART3        (1 << 0) /* Bit 0: USART3 pending */
#define SYSCFG_ITLINE29_LPUART1       (1 << 1) /* Bit 1: LPUART1 pending */

/* SYSCFG interrupt line 30 status register */

#define SYSCFG_ITLINE30_USART4        (1 << 0) /* Bit 0: USART4 pending */
#define SYSCFG_ITLINE30_LPUART3       (1 << 1) /* Bit 1: LPUART3 pending */

/* SYSCFG interrupt line 31 status register */

#define SYSCFG_ITLINE31_RNG           (1 << 0) /* Bit 0: RNG pending */
#define SYSCFG_ITLINE31_AES           (1 << 1) /* Bit 1: AES pending */

#endif /* __ARCH_ARM_SRC_STM32U0_HARDWARE_STM32U0_SYSCFG_H */
