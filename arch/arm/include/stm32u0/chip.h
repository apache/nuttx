/****************************************************************************
 * arch/arm/include/stm32u0/chip.h
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

#ifndef __ARCH_ARM_INCLUDE_STM32U0_CHIP_H
#define __ARCH_ARM_INCLUDE_STM32U0_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* Number of peripheral interrupts */

#define ARMV6M_PERIPHERAL_INTERRUPTS 32

/* Get the flash size for each supported chip */

#if defined(CONFIG_ARCH_CHIP_STM32U073C8) || \
    defined(CONFIG_ARCH_CHIP_STM32U073M8) || \
    defined(CONFIG_ARCH_CHIP_STM32U073R8)
#  define STM32_FLASH_SIZE           (64 * 1024) /* 64Kb */
#elif defined(CONFIG_ARCH_CHIP_STM32U073CB) || \
      defined(CONFIG_ARCH_CHIP_STM32U073MB) || \
      defined(CONFIG_ARCH_CHIP_STM32U073RB)
#  define STM32_FLASH_SIZE           (128 * 1024) /* 128Kb */
#elif defined(CONFIG_ARCH_CHIP_STM32U073CC) || \
      defined(CONFIG_ARCH_CHIP_STM32U073MC) || \
      defined(CONFIG_ARCH_CHIP_STM32U073RC) || \
      defined(CONFIG_ARCH_CHIP_STM32U083CC) || \
      defined(CONFIG_ARCH_CHIP_STM32U083MC) || \
      defined(CONFIG_ARCH_CHIP_STM32U083RC)
#  define STM32_FLASH_SIZE           (256 * 1024) /* 256Kb */
#endif

/* Get customizations for each supported chip.
 * The STM32U073xx and STM32U083xx families share the same peripheral set
 * except for the AES accelerator which is available only on STM32U083xx.
 */

#  define STM32_SRAM_SIZE            (40 * 1024) /* 40Kb = 32Kb SRAM1 + 8Kb
                                                  * backup SRAM2 (contiguous) */

#  define STM32_NATIM                1 /* One advanced timer TIM1 */
#  define STM32_NGTIM16              3 /* 16-bit general up/down timers TIM3,
                                        * TIM15-16 */
#  define STM32_NGTIM32              1 /* 32-bit general up/down timer TIM2 */
#  define STM32_NBTIM                2 /* Two basic timers: TIM6, TIM7 */
                                       /* Three LPTIMER: LPTIM1-3 */
#  define STM32_NSPI                 3 /* Three SPI modules SPI1-3 */
#  define STM32_NI2S                 1 /* One I2S module (SPI1 or I2S1) */
#  define STM32_NI2C                 4 /* Four I2C modules I2C1-4 */
#  define STM32_NDMA                 2 /* DMA1 7-channels, DMA2 5-channels */
#  define STM32_NUSART               4 /* Four USART modules, USART1-4 */
                                       /* Three LPUART: LPUART1-3 */
#  define STM32_NCAN                 0 /* No CAN controllers */
#  define STM32_NLCD                 1 /* One LCD controller */
#  define STM32_NUSBDEV              1 /* One USB full-speed device controller */
#  define STM32_NUSBOTG              0 /* No USB OTG */
#  define STM32_NCEC                 0 /* No HDMI-CEC controller */
#  define STM32_NADC                 1 /* (1) ADC1, 19-channels */

#  define STM32_NDAC                 1 /* One DAC channel */
#  define STM32_NCOMP                2 /* Two Analog Comparators */
#  define STM32_NCRC                 1 /* One CRC module */
#  define STM32_NRNG                 1 /* One Random number generator (RNG) */
#  define STM32_NCAP                 1 /* One Touch sensing controller (TSC) */
#  define STM32_NPORTS               6 /* Six GPIO ports, GPIOA-F */

/* NVIC priority levels *****************************************************/

/* Each priority field holds a priority value, 0-31. The lower the value,
 * the greater the priority of the corresponding interrupt.  The processor
 * implements only bits[7:6] of each field, bits[5:0] read as zero and
 * ignore writes.
 */

#define NVIC_SYSH_PRIORITY_MIN       0xc0 /* All bits[7:6] set is minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT   0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX       0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP      0x40 /* Two bits of interrupt priority used */

#endif /* __ARCH_ARM_INCLUDE_STM32U0_CHIP_H */
