/****************************************************************************
 * arch/arm/src/stm32f0l0g0/hardware/stm32g0_syscfg.h
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

#ifndef __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32G0_SYSCFG_H
#define __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32G0_SYSCFG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_SYSCFG_CFGR1_OFFSET      0x0000 /* SYSCFG configuration register 1 */
#define STM32_SYSCFG_CFGR2_OFFSET      0x0018 /* SYSCFG configuration register 2 */
#define STM32_SYSCFG_ITLINE0_OFFSET    0x0080 /* SYSCFG interrupt line 0 status register */
#define STM32_SYSCFG_ITLINE1_OFFSET    0x0084 /* SYSCFG interrupt line 1 status register */
#define STM32_SYSCFG_ITLINE2_OFFSET    0x0088 /* SYSCFG interrupt line 2 status register */
#define STM32_SYSCFG_ITLINE3_OFFSET    0x008c /* SYSCFG interrupt line 3 status register */
#define STM32_SYSCFG_ITLINE4_OFFSET    0x0090 /* SYSCFG interrupt line 4 status register */
#define STM32_SYSCFG_ITLINE5_OFFSET    0x0094 /* SYSCFG interrupt line 5 status register */
#define STM32_SYSCFG_ITLINE6_OFFSET    0x0098 /* SYSCFG interrupt line 6 status register */
#define STM32_SYSCFG_ITLINE7_OFFSET    0x009c /* SYSCFG interrupt line 7 status register */
#define STM32_SYSCFG_ITLINE8_OFFSET    0x00a0 /* SYSCFG interrupt line 8 status register */
#define STM32_SYSCFG_ITLINE9_OFFSET    0x00a4 /* SYSCFG interrupt line 9 status register */
#define STM32_SYSCFG_ITLINE10_OFFSET   0x00a8 /* SYSCFG interrupt line 10 status register */
#define STM32_SYSCFG_ITLINE11_OFFSET   0x00ac /* SYSCFG interrupt line 11 status register */
#define STM32_SYSCFG_ITLINE12_OFFSET   0x00b0 /* SYSCFG interrupt line 12 status register */
#define STM32_SYSCFG_ITLINE13_OFFSET   0x00b4 /* SYSCFG interrupt line 13 status register */
#define STM32_SYSCFG_ITLINE14_OFFSET   0x00b8 /* SYSCFG interrupt line 14 status register */
#define STM32_SYSCFG_ITLINE15_OFFSET   0x00bc /* SYSCFG interrupt line 15 status register */
#define STM32_SYSCFG_ITLINE16_OFFSET   0x00c0 /* SYSCFG interrupt line 16 status register */
#define STM32_SYSCFG_ITLINE17_OFFSET   0x00c4 /* SYSCFG interrupt line 17 status register */
#define STM32_SYSCFG_ITLINE18_OFFSET   0x00c8 /* SYSCFG interrupt line 18 status register */
#define STM32_SYSCFG_ITLINE19_OFFSET   0x00cc /* SYSCFG interrupt line 19 status register */
#define STM32_SYSCFG_ITLINE20_OFFSET   0x00d0 /* SYSCFG interrupt line 20 status register */
#define STM32_SYSCFG_ITLINE21_OFFSET   0x00d4 /* SYSCFG interrupt line 21 status register */
#define STM32_SYSCFG_ITLINE22_OFFSET   0x00d8 /* SYSCFG interrupt line 22 status register */
#define STM32_SYSCFG_ITLINE23_OFFSET   0x00dc /* SYSCFG interrupt line 23 status register */
#define STM32_SYSCFG_ITLINE24_OFFSET   0x00e0 /* SYSCFG interrupt line 24 status register */
#define STM32_SYSCFG_ITLINE25_OFFSET   0x00e4 /* SYSCFG interrupt line 25 status register */
#define STM32_SYSCFG_ITLINE26_OFFSET   0x00e8 /* SYSCFG interrupt line 26 status register */
#define STM32_SYSCFG_ITLINE27_OFFSET   0x00ec /* SYSCFG interrupt line 27 status register */
#define STM32_SYSCFG_ITLINE28_OFFSET   0x00f0 /* SYSCFG interrupt line 28 status register */
#define STM32_SYSCFG_ITLINE29_OFFSET   0x00f4 /* SYSCFG interrupt line 29 status register */
#define STM32_SYSCFG_ITLINE30_OFFSET   0x00f8 /* SYSCFG interrupt line 30 status register */
#define STM32_SYSCFG_ITLINE31_OFFSET   0x00fc /* SYSCFG interrupt line 31 status register */

/* Register Addresses *******************************************************/

#define STM32_SYSCFG_CFGR1             (STM32_SYSCFG_BASE + STM32_SYSCFG_CFGR1_OFFSET)

#define STM32_SYSCFG_ITLINE0           (STM32_SYSCFG_BASE + STM32_SYSCFG_ITLINE0_OFFSET)
#define STM32_SYSCFG_ITLINE1           (STM32_SYSCFG_BASE + STM32_SYSCFG_ITLINE1_OFFSET)
#define STM32_SYSCFG_ITLINE2           (STM32_SYSCFG_BASE + STM32_SYSCFG_ITLINE2_OFFSET)
#define STM32_SYSCFG_ITLINE3           (STM32_SYSCFG_BASE + STM32_SYSCFG_ITLINE3_OFFSET)
#define STM32_SYSCFG_ITLINE4           (STM32_SYSCFG_BASE + STM32_SYSCFG_ITLINE4_OFFSET)
#define STM32_SYSCFG_ITLINE5           (STM32_SYSCFG_BASE + STM32_SYSCFG_ITLINE5_OFFSET)
#define STM32_SYSCFG_ITLINE6           (STM32_SYSCFG_BASE + STM32_SYSCFG_ITLINE6_OFFSET)
#define STM32_SYSCFG_ITLINE7           (STM32_SYSCFG_BASE + STM32_SYSCFG_ITLINE7_OFFSET)
#define STM32_SYSCFG_ITLINE8           (STM32_SYSCFG_BASE + STM32_SYSCFG_ITLINE8_OFFSET)
#define STM32_SYSCFG_ITLINE9           (STM32_SYSCFG_BASE + STM32_SYSCFG_ITLINE9_OFFSET)
#define STM32_SYSCFG_ITLINE10          (STM32_SYSCFG_BASE + STM32_SYSCFG_ITLINE10_OFFSET)
#define STM32_SYSCFG_ITLINE11          (STM32_SYSCFG_BASE + STM32_SYSCFG_ITLINE11_OFFSET)
#define STM32_SYSCFG_ITLINE12          (STM32_SYSCFG_BASE + STM32_SYSCFG_ITLINE12_OFFSET)
#define STM32_SYSCFG_ITLINE13          (STM32_SYSCFG_BASE + STM32_SYSCFG_ITLINE13_OFFSET)
#define STM32_SYSCFG_ITLINE14          (STM32_SYSCFG_BASE + STM32_SYSCFG_ITLINE14_OFFSET)
#define STM32_SYSCFG_ITLINE15          (STM32_SYSCFG_BASE + STM32_SYSCFG_ITLINE15_OFFSET)
#define STM32_SYSCFG_ITLINE16          (STM32_SYSCFG_BASE + STM32_SYSCFG_ITLINE16_OFFSET)
#define STM32_SYSCFG_ITLINE17          (STM32_SYSCFG_BASE + STM32_SYSCFG_ITLINE17_OFFSET)
#define STM32_SYSCFG_ITLINE18          (STM32_SYSCFG_BASE + STM32_SYSCFG_ITLINE18_OFFSET)
#define STM32_SYSCFG_ITLINE19          (STM32_SYSCFG_BASE + STM32_SYSCFG_ITLINE19_OFFSET)
#define STM32_SYSCFG_ITLINE20          (STM32_SYSCFG_BASE + STM32_SYSCFG_ITLINE20_OFFSET)
#define STM32_SYSCFG_ITLINE21          (STM32_SYSCFG_BASE + STM32_SYSCFG_ITLINE21_OFFSET)
#define STM32_SYSCFG_ITLINE22          (STM32_SYSCFG_BASE + STM32_SYSCFG_ITLINE22_OFFSET)
#define STM32_SYSCFG_ITLINE23          (STM32_SYSCFG_BASE + STM32_SYSCFG_ITLINE23_OFFSET)
#define STM32_SYSCFG_ITLINE24          (STM32_SYSCFG_BASE + STM32_SYSCFG_ITLINE24_OFFSET)
#define STM32_SYSCFG_ITLINE25          (STM32_SYSCFG_BASE + STM32_SYSCFG_ITLINE25_OFFSET)
#define STM32_SYSCFG_ITLINE26          (STM32_SYSCFG_BASE + STM32_SYSCFG_ITLINE26_OFFSET)
#define STM32_SYSCFG_ITLINE27          (STM32_SYSCFG_BASE + STM32_SYSCFG_ITLINE27_OFFSET)
#define STM32_SYSCFG_ITLINE28          (STM32_SYSCFG_BASE + STM32_SYSCFG_ITLINE28_OFFSET)
#define STM32_SYSCFG_ITLINE29          (STM32_SYSCFG_BASE + STM32_SYSCFG_ITLINE29_OFFSET)
#define STM32_SYSCFG_ITLINE30          (STM32_SYSCFG_BASE + STM32_SYSCFG_ITLINE30_OFFSET)
#define STM32_SYSCFG_ITLINE31          (STM32_SYSCFG_BASE + STM32_SYSCFG_ITLINE31_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* SYSCFG memory remap register */

#define SYSCFG_CFGR1_MEMMODE_SHIFT     (0)                               /* Bits 1:0 MEM_MODE: Memory mapping selection */
#define SYSCFG_CFGR1_MEMMODE_MASK      (3 << SYSCFG_CFGR1_MEMMODE_SHIFT)
#  define SYSCFG_CFGR1_MEMMODE_FLASH   (0 << SYSCFG_CFGR1_MEMMODE_SHIFT) /* 00: Main Flash at 0x00000000 */
#  define SYSCFG_CFGR1_MEMMODE_SYSTEM  (1 << SYSCFG_CFGR1_MEMMODE_SHIFT) /* 01: System Flash at 0x00000000 */
#  define SYSCFG_CFGR1_MEMMODE_SRAM    (3 << SYSCFG_CFGR1_MEMMODE_SHIFT) /* 11: Embedded SRAM at 0x00000000 */
                                                                         /* Bit 2: Reserved */
#define SYSCFG_CFGR1_PA11_RMP          (1 << 3)                          /* Bit 3: PA11 remapping bit */
#define SYSCFG_CFGR1_PA12_RMP          (1 << 4)                          /* Bit 4: PA12 remapping bit */
#define SYSCFG_CFGR1_IRPOL             (1 << 5)                          /* Bit 5: IR output polarity selection */
#define SYSCFG_CFGR1_IRMOD_SHIFT       (6)                               /* Bits 6-7: IR Modulation Envelope signal selection */
#define SYSCFG_CFGR1_IRMOD_MASK        (3 << SYSCFG_CFGR1_IRMOD_SHIFT)
#  define SYSCFG_CFGR1_IRMOD_TIM16     (0 << SYSCFG_CFGR1_IRMOD_SHIFT)  /* 00: TIM16 selected */
#  define SYSCFG_CFGR1_IRMOD_USART1    (1 << SYSCFG_CFGR1_IRMOD_SHIFT)  /* 01: USART1 selected */
#  define SYSCFG_CFGR1_IRMOD_USART4    (2 << SYSCFG_CFGR1_IRMOD_SHIFT)  /* 10: USART1 selected */
#define SYSCFG_CFGR1_BOOSTEN           (1 << 8)                         /* Bit 8: IO analog switch voltage booster enable */
#define SYSCFG_CFGR1_UCPD1STROBE       (1 << 9)                         /* Bit 9: UCPD1 pull-down configuration strobe */
#define SYSCFG_CFGR1_UCPD2STROBE       (1 << 10)                        /* Bit 10: UCPD2 pull-down configuration strobe */
                                                                        /* Bits 11-15: Reserved */
#define SYSCFG_CFGR1_I2CPB6FMP         (1 << 16)                        /* Bit 16: */
#define SYSCFG_CFGR1_I2CPB7FMP         (1 << 17)                        /* Bit 17: */
#define SYSCFG_CFGR1_I2CPB8FMP         (1 << 18)                        /* Bit 18: */
#define SYSCFG_CFGR1_I2CPB9FMP         (1 << 19)                        /* Bit 19: */
#define SYSCFG_CFGR1_I2C1FMP           (1 << 20)                        /* Bit 20: */
#define SYSCFG_CFGR1_I2C2FMP           (1 << 21)                        /* Bit 21: */
#define SYSCFG_CFGR1_I2CPA9FMP         (1 << 22)                        /* Bit 22: */
#define SYSCFG_CFGR1_I2CPA10FMP        (1 << 23)                        /* Bit 23: */
                                                                        /* Bits 24-31: Reserved */

/* SYSCFG interrupt line 0 status register */

#define SYSCFG_ITLINE0_WWDG            (1 << 0)  /* Bit 0: Window Watchdog interrupt pending flag */

/* SYSCFG interrupt line 1 status register */

#define SYSCFG_ITLINE1_PVDOUT          (1 << 0)  /* Bit 0: PVD supply monitoring interrupt request pending (EXTI line 16) */

/* SYSCFG interrupt line 2 status register */

#define SYSCFG_ITLINE2_RTC_WAKEUP      (1 << 0)  /* Bit 0: RTC Wake Up interrupt request pending (EXTI line 20) */
#define SYSCFG_ITLINE2_RTC_TSTAMP      (1 << 1)  /* Bit 1: RTC Tamper and TimeStamp interrupt request pending (EXTI line 19) */

/* SYSCFG interrupt line 3 status register */

#define SYSCFG_ITLINE3_FLASH_ITF       (1 << 0)  /* Bit 0: Flash interface interrupt request pending */
#define SYSCFG_ITLINE3_FLASH_ECC       (1 << 1)  /* Bit 1: Flash interface ECC interrupt request pending */

/* SYSCFG interrupt line 4 status register */

#define SYSCFG_ITLINE4_RCC             (1 << 0)  /* Bit 0: Reset and clock control interrupt request pending */

/* SYSCFG interrupt line 5 status register */

#define SYSCFG_ITLINE5_EXTI0           (1 << 0)  /* Bit 0: EXTI line 0 interrupt request pending */
#define SYSCFG_ITLINE5_EXTI1           (1 << 1)  /* Bit 1: EXTI line 1 interrupt request pending */

/* SYSCFG interrupt line 6 status register */

#define SYSCFG_ITLINE6_EXTI2           (1 << 0)  /* Bit 0: EXTI line 2 interrupt request pending */
#define SYSCFG_ITLINE6_EXTI3           (1 << 1)  /* Bit 1: EXTI line 3 interrupt request pending */

/* SYSCFG interrupt line 7 status register */

#define SYSCFG_ITLINE7_EXTI4           (1 << 0)  /* Bit 0: EXTI line 4 interrupt request pending */
#define SYSCFG_ITLINE7_EXTI5           (1 << 1)  /* Bit 1: EXTI line 5 interrupt request pending */
#define SYSCFG_ITLINE7_EXTI6           (1 << 2)  /* Bit 2: EXTI line 6 interrupt request pending */
#define SYSCFG_ITLINE7_EXTI7           (1 << 3)  /* Bit 3: EXTI line 7 interrupt request pending */
#define SYSCFG_ITLINE7_EXTI8           (1 << 4)  /* Bit 4: EXTI line 8 interrupt request pending */
#define SYSCFG_ITLINE7_EXTI9           (1 << 5)  /* Bit 5: EXTI line 9 interrupt request pending */
#define SYSCFG_ITLINE7_EXTI10          (1 << 6)  /* Bit 6: EXTI line 10 interrupt request pending */
#define SYSCFG_ITLINE7_EXTI11          (1 << 7)  /* Bit 7: EXTI line 11 interrupt request pending */
#define SYSCFG_ITLINE7_EXTI12          (1 << 8)  /* Bit 8: EXTI line 12 interrupt request pending */
#define SYSCFG_ITLINE7_EXTI13          (1 << 9)  /* Bit 9: EXTI line 13 interrupt request pending */
#define SYSCFG_ITLINE7_EXTI14          (1 << 10) /* Bit 10: EXTI line 14 interrupt request pending */
#define SYSCFG_ITLINE7_EXTI15          (1 << 11) /* Bit 11: EXTI line 15 interrupt request pending */

/* SYSCFG interrupt line 8 status register */

#define SYSCFG_ITLINE8_UCPD1           (1 << 0)  /* Bit 0: UCPD1 interrupt request pending */
#define SYSCFG_ITLINE8_UCPD2           (1 << 1)  /* Bit 1: UCPD2 interrupt request pending */

/* SYSCFG interrupt line 9 status register */

#define SYSCFG_ITLINE9_DMA1_CH1        (1 << 0)  /* Bit 0: DMA1 channel 1 interrupt request pending */

/* SYSCFG interrupt line 10 status register */

#define SYSCFG_ITLINE10_DMA1_CH2       (1 << 0)  /* Bit 0: DMA1 channel 2 interrupt request pending */
#define SYSCFG_ITLINE10_DMA1_CH3       (1 << 1)  /* Bit 1: DMA1 channel 3 interrupt request pending */

/* SYSCFG interrupt line 11 status register */

#define SYSCFG_ITLINE11_DMAMUX         (1 << 0)  /* Bit 0: DMAMUX interrupt request pending */
#define SYSCFG_ITLINE11_DMA1_CH4       (1 << 1)  /* Bit 1: DMA1 channel 4 interrupt request pending */
#define SYSCFG_ITLINE11_DMA1_CH5       (1 << 2)  /* Bit 2: DMA1 channel 5 interrupt request pending */
#define SYSCFG_ITLINE11_DMA1_CH6       (1 << 3)  /* Bit 3: DMA1 channel 6 interrupt request pending */
#define SYSCFG_ITLINE11_DMA1_CH7       (1 << 4)  /* Bit 4: DMA1 channel 7 interrupt request pending */

/* SYSCFG interrupt line 12 status register */

#define SYSCFG_ITLINE12_ADC            (1 << 0)  /* Bit 0: ADC interrupt request pending */
#define SYSCFG_ITLINE12_COMP1          (1 << 1)  /* Bit 1: Comparator 1 interrupt request pending */
#define SYSCFG_ITLINE12_COMP2          (1 << 2)  /* Bit 2: Comparator 2 interrupt request pending */

/* SYSCFG interrupt line 13 status register */

#define SYSCFG_ITLINE13_TIM1_CCU       (1 << 0)  /* Bit 0: TIM1 commutation interrupt request pending */
#define SYSCFG_ITLINE13_TIM1_TRG       (1 << 1)  /* Bit 1: TIM1 triggerinterrupt request pending */
#define SYSCFG_ITLINE13_TIM1_UPD       (1 << 2)  /* Bit 2: TIM1 update interrupt request pending */
#define SYSCFG_ITLINE13_TIM1_BRK       (1 << 3)  /* Bit 3: TIM1 break interrupt request pending */

/* SYSCFG interrupt line 14 status register */

#define SYSCFG_ITLINE14_TIM1_CC        (1 << 0)  /* Bit 0: TIM1 capture compare interrupt request pending */

/* SYSCFG interrupt line 15 status register */

#define SYSCFG_ITLINE15_TIM2           (1 << 0)  /* Bit 0: Timer 2 interrupt request pending */

/* SYSCFG interrupt line 16 status register */

#define SYSCFG_ITLINE16_TIM3           (1 << 0)  /* Bit 0: Timer 3 interrupt request pending */

/* SYSCFG interrupt line 17 status register */

#define SYSCFG_ITLINE17_TIM6           (1 << 0)  /* Bit 0: Timer 6 interrupt request pending */
#define SYSCFG_ITLINE17_DAC            (1 << 1)  /* Bit 1: DAC underrun interrupt request pending */
#define SYSCFG_ITLINE17_LPTIM1         (1 << 2)  /* Bit 2: Low-power timer 1 interrupt request pending */

/* SYSCFG interrupt line 18 status register */

#define SYSCFG_ITLINE18_TIM7           (1 << 0)  /* Bit 0: Timer 7 interrupt request pending */
#define SYSCFG_ITLINE18_LPTIM2         (1 << 1)  /* Bit 1: Low-power timer 2 interrupt request pending */

/* SYSCFG interrupt line 19 status register */

#define SYSCFG_ITLINE19_TIM14          (1 << 0)  /* Bit 0: Timer 14 interrupt request pending */

/* SYSCFG interrupt line 20 status register */

#define SYSCFG_ITLINE20_TIM15          (1 << 0)  /* Bit 0: Timer 15 interrupt request pending */

/* SYSCFG interrupt line 21 status register */

#define SYSCFG_ITLINE21_TIM16          (1 << 0)  /* Bit 0: Timer 16 interrupt request pending */

/* SYSCFG interrupt line 22 status register */

#define SYSCFG_ITLINE22_TIM17          (1 << 0)  /* Bit 0: Timer 17 interrupt request pending */

/* SYSCFG interrupt line 23 status register */

#define SYSCFG_ITLINE23_I2C1           (1 << 0)  /* Bit 0: I2C1 interrupt request pending, combined with EXTI line 23 */

/* SYSCFG interrupt line 24 status register */

#define SYSCFG_ITLINE24_I2C2           (1 << 0)  /* Bit 0: I2C2 interrupt request pending */

/* SYSCFG interrupt line 25 status register */

#define SYSCFG_ITLINE25_SPI1           (1 << 0)  /* Bit 0: SPI1 interrupt request pending */

/* SYSCFG interrupt line 26 status register */

#define SYSCFG_ITLINE26_SPI2           (1 << 0)  /* Bit 0: SPI2 interrupt request pending */

/* SYSCFG interrupt line 27 status register */

#define SYSCFG_ITLINE27_USART1         (1 << 0)  /* Bit 0: USART1 interrupt request pending */

/* SYSCFG interrupt line 28 status register */

#define SYSCFG_ITLINE28_USART2         (1 << 0)  /* Bit 0: USART2 interrupt request pending */

/* SYSCFG interrupt line 29 status register */

#define SYSCFG_ITLINE29_USART3         (1 << 0)  /* Bit 0: USART3 interrupt request pending */
#define SYSCFG_ITLINE29_USART4         (1 << 1)  /* Bit 1: USART4 interrupt request pending */
#define SYSCFG_ITLINE29_LPUART1        (1 << 2)  /* Bit 2: LPUART1 interrupt request pending */

/* SYSCFG interrupt line 30 status register */

#define SYSCFG_ITLINE30_CEC            (1 << 0)  /* Bit 0: CEC interrupt request pending, combined with EXTI line 27 */

/* SYSCFG interrupt line 31 status register */

#define SYSCFG_ITLINE30_RNG            (1 << 0)  /* Bit 0: RNG interrupt request pending */
#define SYSCFG_ITLINE30_AES            (1 << 1)  /* Bit 1: AES interrupt request pending */

#endif /* __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32G0_SYSCFG_H */
