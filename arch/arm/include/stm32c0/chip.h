/****************************************************************************
 * arch/arm/include/stm32c0/chip.h
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

#ifndef __ARCH_ARM_INCLUDE_STM32C0_CHIP_H
#define __ARCH_ARM_INCLUDE_STM32C0_CHIP_H

#define ARMV6M_PERIPHERAL_INTERRUPTS 32

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* Get customizations for each supported chip */

#if defined(CONFIG_ARCH_CHIP_STM32C051XX)
#  define STM32_NATIM           1  /* One advanced timers TIM1 */
#  define STM32_NGTIM16         4  /* Four 16-bit general up/down timers TIM3, TIM14,
                                    *  TIM16 and TIM17 */
#  define STM32_NGTIM32         1  /* One 32-bit general up/down timer TIM2 */
#  define STM32_NBTIM           0  /* No basic timers */
                                   /* One LPTIMER */
#  define STM32_NSPI            2  /* Two SPI modules SPI1-2 */
#  define STM32_NI2S            0  /* No I2S module */
#  define STM32_NI2C            2  /* Two I2C */
#  define STM32_NDMA            1  /* One DMA1, 5-channels */
#  define STM32_NUSART          2  /* Two USART modules, USART1-2 */
#  define STM32_NCAN            0  /* No CAN controllers */
#  define STM32_FDCAN           0  /* No FD CAN */
#  define STM32_NLCD            0  /* No LCD controller */
#  define STM32_NUSBDEV         0  /* No USB full-speed device controller */
#  define STM32_NUSBOTG         0  /* No USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NCEC            0  /* No HDMI-CEC controller */
#  define STM32_NADC            1  /* One 12-bit module */
#  define STM32_NDAC            0  /* No DAC channels */
#  define STM32_NCOMP           0  /* No Analog Comparators */
#  define STM32_NCRC            1  /* One CRC module */
#  define STM32_NRNG            0  /* No Random number generator (RNG) */
#  define STM32_NCAP            0  /* No Capacitive sensing channels */
#  define STM32_NPORTS          5  /* Five GPIO ports, GPIOA-D, F */
#elif defined(CONFIG_ARCH_CHIP_STM32C071XX)
#  define STM32_NATIM           1  /* One advanced timers TIM1 */
#  define STM32_NGTIM16         4  /* 16-bit general up/down timers TIM3, TIM14,
                                    *  TIM16 and TIM17 */
#  define STM32_NGTIM32         1  /* One 32-bit general up/down timer TIM2 */
#  define STM32_NBTIM           0  /* No basic timers */
                                   /* One LPTIMER */
#  define STM32_NSPI            2  /* Two SPI modules SPI1-2 */
#  define STM32_NI2S            0  /* No I2S module */
#  define STM32_NI2C            2  /* Two I2C */
#  define STM32_NDMA            1  /* One DMA1, 5-channels */
#  define STM32_NUSART          2  /* Two USART modules, USART1-2 */
#  define STM32_NCAN            0  /* No CAN controllers */
#  define STM32_FDCAN           0  /* No FD CAN */
#  define STM32_NLCD            0  /* No LCD controller */
#  define STM32_NUSBDEV         1  /* USB full-speed device controller */
#  define STM32_NUSBOTG         0  /* No USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NCEC            0  /* No HDMI-CEC controller */
#  define STM32_NADC            1  /* One 12-bit module */
#  define STM32_NDAC            0  /* No DAC channels */
#  define STM32_NCOMP           0  /* No Analog Comparators */
#  define STM32_NCRC            1  /* One CRC module */
#  define STM32_NRNG            0  /* No Random number generator (RNG) */
#  define STM32_NCAP            0  /* No Capacitive sensing channels */
#  define STM32_NPORTS          5  /* Five GPIO ports, GPIOA-D, F */
#elif defined(CONFIG_ARCH_CHIP_STM32C091XX)
#  define STM32_NATIM           1  /* One advanced timers TIM1 */
#  define STM32_NGTIM16         5  /* 16-bit general up/down timers TIM3, TIM14,
                                    *  TIM15, TIM16 and TIM17 */
#  define STM32_NGTIM32         1  /* One 32-bit general up/down timer TIM2 */
#  define STM32_NBTIM           0  /* No basic timers */
                                   /* One LPTIMER */
#  define STM32_NSPI            2  /* Two SPI modules SPI1-2 */
#  define STM32_NI2S            0  /* No I2S module */
#  define STM32_NI2C            2  /* Two I2C */
#  define STM32_NDMA            1  /* One DMA1, 5-channels */
#  define STM32_NUSART          4  /* Four USART modules, USART1-4 */
#  define STM32_NCAN            0  /* No CAN controllers */
#  define STM32_FDCAN           0  /* No FD CAN */
#  define STM32_NLCD            0  /* No LCD controller */
#  define STM32_NUSBDEV         1  /* USB full-speed device controller */
#  define STM32_NUSBOTG         0  /* No USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NCEC            0  /* No HDMI-CEC controller */
#  define STM32_NADC            1  /* One 12-bit module */
#  define STM32_NDAC            0  /* No DAC channels */
#  define STM32_NCOMP           0  /* No Analog Comparators */
#  define STM32_NCRC            1  /* One CRC module */
#  define STM32_NRNG            0  /* No Random number generator (RNG) */
#  define STM32_NCAP            0  /* No Capacitive sensing channels */
#  define STM32_NPORTS          5  /* Five GPIO ports, GPIOA-D, F */
#elif defined(CONFIG_ARCH_CHIP_STM32C092XX)
#  define STM32_NATIM           1  /* One advanced timers TIM1 */
#  define STM32_NGTIM16         5  /* 16-bit general up/down timers TIM3, TIM14,
                                    *  TIM15, TIM16 and TIM17 */
#  define STM32_NGTIM32         1  /* One 32-bit general up/down timer TIM2 */
#  define STM32_NBTIM           0  /* No basic timers */
                                   /* One LPTIMER */
#  define STM32_NSPI            2  /* Two SPI modules SPI1-2 */
#  define STM32_NI2S            0  /* No I2S module */
#  define STM32_NI2C            2  /* Two I2C */
#  define STM32_NDMA            1  /* One DMA1, 5-channels */
#  define STM32_NUSART          4  /* Four USART modules, USART1-4 */
#  define STM32_NCAN            0  /* No CAN controllers */
#  define STM32_FDCAN           1  /* One FD CAN */
#  define STM32_NLCD            0  /* No LCD controller */
#  define STM32_NUSBDEV         1  /* USB full-speed device controller */
#  define STM32_NUSBOTG         0  /* No USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NCEC            0  /* No HDMI-CEC controller */
#  define STM32_NADC            1  /* One 12-bit module */
#  define STM32_NDAC            0  /* No DAC channels */
#  define STM32_NCOMP           0  /* No Analog Comparators */
#  define STM32_NCRC            1  /* One CRC module */
#  define STM32_NRNG            0  /* No Random number generator (RNG) */
#  define STM32_NCAP            0  /* No Capacitive sensing channels */
#  define STM32_NPORTS          5  /* Five GPIO ports, GPIOA-D, F */
#else
#  error "Unsupported STM32 Cortex M0 chip"
#endif

/* NVIC priority levels *****************************************************/

/* Each priority field holds a priority value, 0-31. The lower the value,
 * the greater the priority of the corresponding interrupt.  The processor
 * implements only bits[7:6] of each field, bits[5:0] read as zero and
 * ignore writes.
 */

#define NVIC_SYSH_PRIORITY_MIN     0xc0 /* All bits[7:6] set is minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT 0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX     0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP    0x40 /* Two bits of interrupt priority used */

#endif /* __ARCH_ARM_INCLUDE_STM32C0_CHIP_H */
