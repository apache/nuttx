/****************************************************************************
 * arch/arm/include/stm32l0/chip.h
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

#ifndef __ARCH_ARM_INCLUDE_STM32L0_CHIP_H
#define __ARCH_ARM_INCLUDE_STM32L0_CHIP_H

#define ARMV6M_PERIPHERAL_INTERRUPTS 32

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* Get customizations for each supported chip */

#if defined(CONFIG_ARCH_CHIP_STM32L071K8)
#  define STM32_NATIM           0  /* No advanced timers */
#  define STM32_NGTIM16         4  /* 16-bit general up/down timers TIM2-3
                                    * (with DMA) and TIM21-22 without DMA */
#  define STM32_NGTIM32         0  /* No 32-bit general up/down timers */
#  define STM32_NBTIM           2  /* 2 basic timers: TIM6, TIM7 with DMA */
                                   /* 1 LPTIMER */
#  define STM32_NSPI            1  /* 1 SPI modules SPI1 */
#  define STM32_NI2S            0  /* 0 I2S module */
#  define STM32_NI2C            2  /* 2 I2C */
#  define STM32_NDMA            1  /* 1 DMA1, 7-channels */
#  define STM32_NUSART          3  /* 3 USART modules, USART1-3 */
                                   /* 1 LPUART */
#  define STM32_NCAN            0  /* 0 CAN controllers */
#  define STM32_NLCD            0  /* 0 LCD */
#  define STM32_NUSBDEV         0  /* 0 USB full-speed device controller */
#  define STM32_NUSBOTG         0  /* 0 USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NCEC            0  /* 0 HDMI-CEC controller */
#  define STM32_NADC            1  /* One 12-bit module */
#  define STM32_NDAC            0  /* 0 DAC channel */
#  define STM32_NCOMP           2  /* 2 Analog Comparators */
#  define STM32_NCRC            0  /* 0 CRC module */
#  define STM32_NRNG            0  /* 0 Random number generator (RNG) */
#  define STM32_NCAP            0  /* 0 Capacitive sensing channels */
#  define STM32_NPORTS          6  /* Six GPIO ports, GPIOA-E, H */

#elif defined(CONFIG_ARCH_CHIP_STM32L053C8)
#  define STM32_NATIM           0  /* No advanced timers */
#  define STM32_NGTIM16         3  /* 16-bit general up/down timers TIM2-3
                                    * (with DMA) and TIM22 without DMA */
#  define STM32_NGTIM32         0  /* No 32-bit general up/down timers */
#  define STM32_NBTIM           1  /* 1 basic timers: TIM6 with DMA */
                                   /* 1 LPTIMER */
#  define STM32_NSPI            2  /* 2 SPI modules SPI1 */
#  define STM32_NI2S            1  /* 1 I2S module */
#  define STM32_NI2C            2  /* 2 I2C */
#  define STM32_NDMA            1  /* 1 DMA1, 7-channels */
#  define STM32_NUSART          2  /* 2 USART modules, USART1-1 */
                                   /* 1 LPUART */
#  define STM32_NCAN            0  /* 0 CAN controllers */
#  define STM32_NLCD            1  /* 1 LCD */
#  define STM32_NUSBDEV         1  /* 1 USB full-speed device controller */
#  define STM32_NUSBOTG         0  /* 0 USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NCEC            0  /* 0 HDMI-CEC controller */
#  define STM32_NADC            1  /* One 12-bit module */
#  define STM32_NDAC            0  /* 0 DAC channel */
#  define STM32_NCOMP           2  /* 2 Analog Comparators */
#  define STM32_NCRC            0  /* 0 CRC module */
#  define STM32_NRNG            0  /* 0 Random number generator (RNG) */
#  define STM32_NCAP            24 /* 24 Capacitive sensing channels */
#  define STM32_NPORTS          6  /* Six GPIO ports, GPIOA-E, H */

#elif defined(CONFIG_ARCH_CHIP_STM32L053R8)
#  define STM32_NATIM           0  /* No advanced timers */
#  define STM32_NGTIM16         3  /* 16-bit general up/down timers TIM2-3
                                    * (with DMA) and TIM22 without DMA */
#  define STM32_NGTIM32         0  /* No 32-bit general up/down timers */
#  define STM32_NBTIM           1  /* 1 basic timers: TIM6 with DMA */
                                   /* 1 LPTIMER */
#  define STM32_NSPI            2  /* 2 SPI modules SPI1 */
#  define STM32_NI2S            1  /* 1 I2S module */
#  define STM32_NI2C            2  /* 2 I2C */
#  define STM32_NDMA            1  /* 1 DMA1, 7-channels */
#  define STM32_NUSART          2  /* 2 USART modules, USART1-1 */
                                   /* 1 LPUART */
#  define STM32_NCAN            0  /* 0 CAN controllers */
#  define STM32_NLCD            1  /* 1 LCD */
#  define STM32_NUSBDEV         1  /* 1 USB full-speed device controller */
#  define STM32_NUSBOTG         0  /* 0 USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NCEC            0  /* 0 HDMI-CEC controller */
#  define STM32_NADC            1  /* One 12-bit module */
#  define STM32_NDAC            0  /* 0 DAC channel */
#  define STM32_NCOMP           2  /* 2 Analog Comparators */
#  define STM32_NCRC            0  /* 0 CRC module */
#  define STM32_NRNG            0  /* 0 Random number generator (RNG) */
#  define STM32_NCAP            24 /* 24 Capacitive sensing channels */
#  define STM32_NPORTS          6  /* Six GPIO ports, GPIOA-E, H */

#elif defined(CONFIG_ARCH_CHIP_STM32L071C8) || defined(CONFIG_ARCH_CHIP_STM32L071V8) || \
      defined(CONFIG_ARCH_CHIP_STM32L071CB) || defined(CONFIG_ARCH_CHIP_STM32L071VB) || \
      defined(CONFIG_ARCH_CHIP_STM32L071RB) || defined(CONFIG_ARCH_CHIP_STM32L071CZ) || \
      defined(CONFIG_ARCH_CHIP_STM32L071VZ) || defined(CONFIG_ARCH_CHIP_STM32L071RZ)
#  define STM32_NATIM           0  /* 0 advanced timers */
#  define STM32_NGTIM16         4  /* 16-bit general up/down timers TIM2-3
                                    * (with DMA) and TIM21-22 without DMA */
#  define STM32_NGTIM32         0  /* 0 32-bit general up/down timers */
#  define STM32_NBTIM           2  /* 2 basic timers: TIM6, TIM7 with DMA */
                                   /* 1 LPTIMER */
#  define STM32_NSPI            2  /* 2 SPI modules SPI1-2 */
#  define STM32_NI2S            1  /* 1 I2S module */
#  define STM32_NI2C            3  /* 3 I2C */
#  define STM32_NDMA            1  /* 1 DMA1, 7-channels */
#  define STM32_NUSART          4  /* 4 USART modules, USART1-4 */
                                   /* 1 LPUART */
#  define STM32_NCAN            0  /* 0 CAN controllers */
#  define STM32_NLCD            0  /* 0 LCD */
#  define STM32_NUSBDEV         0  /* 0 USB full-speed device controller */
#  define STM32_NUSBOTG         0  /* 0 USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NCEC            0  /* 0 HDMI-CEC controller */
#  define STM32_NADC            1  /* One 12-bit module */
#  define STM32_NDAC            0  /* 0 DAC channel */
#  define STM32_NCOMP           2  /* 2 Analog Comparators */
#  define STM32_NCRC            0  /* 0 CRC module */
#  define STM32_NRNG            0  /* 0 Random number generator (RNG) */
#  define STM32_NCAP            0  /* 0 Capacitive sensing channels */
#  define STM32_NPORTS          6  /* Six GPIO ports, GPIOA-E, H */

#elif defined(CONFIG_ARCH_CHIP_STM32L071KB) || defined(CONFIG_ARCH_CHIP_STM32L071KZ)
#  define STM32_NATIM           0  /* 0 advanced timers */
#  define STM32_NGTIM16         4  /* 16-bit general up/down timers TIM2-3
                                    * (with DMA) and TIM21-22 without DMA */
#  define STM32_NGTIM32         0  /* 0 32-bit general up/down timers */
#  define STM32_NBTIM           2  /* 2 basic timers: TIM6, TIM7 with DMA */
                                   /* 1 LPTIMER */
#  define STM32_NSPI            1  /* 1 SPI modules SPI1 */
#  define STM32_NI2S            0  /* 0 I2S module */
#  define STM32_NI2C            3  /* 3 I2C */
#  define STM32_NDMA            1  /* 1 DMA1, 7-channels */
#  define STM32_NUSART          4  /* 4 USART modules, USART1-4 */
                                   /* 1 LPUART */
#  define STM32_NCAN            0  /* 0 CAN controllers */
#  define STM32_NLCD            0  /* 0 LCD */
#  define STM32_NUSBDEV         0  /* 0 USB full-speed device controller */
#  define STM32_NUSBOTG         0  /* 0 USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NCEC            0  /* 0 HDMI-CEC controller */
#  define STM32_NADC            1  /* One 12-bit module */
#  define STM32_NDAC            0  /* 0 DAC channel */
#  define STM32_NCOMP           2  /* 2 Analog Comparators */
#  define STM32_NCRC            0  /* 0 CRC module */
#  define STM32_NRNG            0  /* 0 Random number generator (RNG) */
#  define STM32_NCAP            0  /* 0 Capacitive sensing channels */
#  define STM32_NPORTS          6  /* Six GPIO ports, GPIOA-E, H */

#elif defined(CONFIG_ARCH_CHIP_STM32L072V8) || defined(CONFIG_ARCH_CHIP_STM32L072VB) || \
      defined(CONFIG_ARCH_CHIP_STM32L072VZ)
#  define STM32_NATIM           0  /* No advanced timers */
#  define STM32_NGTIM16         4  /* 16-bit general up/down timers TIM2-3
                                    * (with DMA) and TIM21-22 without DMA */
#  define STM32_NGTIM32         0  /* No 32-bit general up/down timers */
#  define STM32_NBTIM           2  /* Two basic timers: TIM6, TIM7 with DMA */
                                   /* One LPTIMER */
#  define STM32_NSPI            2  /* Two SPI modules SPI1-2 */
#  define STM32_NI2S            1  /* One I2S module */
#  define STM32_NI2C            3  /* Three I2C (2 with SMBus/PMBus) */
#  define STM32_NDMA            1  /* One DMA1, 7-channels */
#  define STM32_NUSART          4  /* Four USART modules, USART1-4 */
                                   /* One LPUART */
#  define STM32_NCAN            0  /* No CAN controllers */
#  define STM32_NLCD            0  /* No LCD */
#  define STM32_NUSBDEV         0  /* No USB full-speed device controller */
#  define STM32_NUSBOTG         1  /* One USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NCEC            0  /* No HDMI-CEC controller */
#  define STM32_NADC            1  /* One 12-bit module */
#  define STM32_NDAC            2  /* Two DAC channels */
#  define STM32_NCOMP           2  /* Two Analog Comparators */
#  define STM32_NCRC            1  /* One CRC module */
#  define STM32_NRNG            1  /* One Random number generator (RNG) */
#  define STM32_NCAP            24 /* Twenty-four Capacitive sensing channels */
#  define STM32_NPORTS          6  /* Six GPIO ports, GPIOA-E, H */

#elif defined(CONFIG_ARCH_CHIP_STM32L072KB) || defined(CONFIG_ARCH_CHIP_STM32L072KZ)
#  define STM32_NATIM           0  /* No advanced timers */
#  define STM32_NGTIM16         4  /* 16-bit general up/down timers TIM2-3
                                    * (with DMA) and TIM21-22 without DMA */
#  define STM32_NGTIM32         0  /* No 32-bit general up/down timers */
#  define STM32_NBTIM           2  /* Two basic timers: TIM6, TIM7 with DMA */
                                   /* One LPTIMER */
#  define STM32_NSPI            2  /* Two SPI modules SPI1-2 */
#  define STM32_NI2C            3  /* Three I2C (2 with SMBus/PMBus) */
#  define STM32_NDMA            1  /* One DMA1, 7-channels */
#  define STM32_NUSART          4  /* Four USART modules, USART1-4 */
                                   /* One LPUART */
#  define STM32_NCAN            0  /* No CAN controllers */
#  define STM32_NLCD            0  /* No LCD */
#  define STM32_NUSBDEV         0  /* No USB full-speed device controller */
#  define STM32_NUSBOTG         1  /* One USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NCEC            0  /* No HDMI-CEC controller */
#  define STM32_NADC            1  /* One 12-bit module */
#  define STM32_NDAC            2  /* Two DAC channels */
#  define STM32_NCOMP           2  /* Two Analog Comparators */
#  define STM32_NCRC            1  /* One CRC module */
#  define STM32_NRNG            1  /* One Random number generator (RNG) */
#  define STM32_NCAP            13 /* Thirteen Capacitive sensing channels */
#  define STM32_NPORTS          6  /* Six GPIO ports, GPIOA-E, H */

#elif defined(CONFIG_ARCH_CHIP_STM32L072CB) || defined(CONFIG_ARCH_CHIP_STM32L072CZ)
#  define STM32_NATIM           0  /* No advanced timers */
#  define STM32_NGTIM16         4  /* 16-bit general up/down timers TIM2-3
                                    * (with DMA) and TIM21-22 without DMA */
#  define STM32_NGTIM32         0  /* No 32-bit general up/down timers */
#  define STM32_NBTIM           2  /* Two basic timers: TIM6, TIM7 with DMA */
                                   /* One LPTIMER */
#  define STM32_NSPI            2  /* Two SPI modules SPI1-2 */
#  define STM32_NI2S            1  /* One I2S module */
#  define STM32_NI2C            3  /* Three I2C (2 with SMBus/PMBus) */
#  define STM32_NDMA            1  /* One DMA1, 7-channels */
#  define STM32_NUSART          4  /* Four USART modules, USART1-4 */
                                   /* One LPUART */
#  define STM32_NCAN            0  /* No CAN controllers */
#  define STM32_NLCD            0  /* No LCD */
#  define STM32_NUSBDEV         0  /* No USB full-speed device controller */
#  define STM32_NUSBOTG         1  /* One USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NCEC            0  /* No HDMI-CEC controller */
#  define STM32_NADC            1  /* One 12-bit module */
#  define STM32_NDAC            2  /* Two DAC channels */
#  define STM32_NCOMP           2  /* Two Analog Comparators */
#  define STM32_NCRC            1  /* One CRC module */
#  define STM32_NRNG            1  /* One Random number generator (RNG) */
#  define STM32_NCAP            18 /* Nineteen Capacitive sensing channels */
#  define STM32_NPORTS          6  /* Six GPIO ports, GPIOA-E, H */

#elif defined(CONFIG_ARCH_CHIP_STM32L072RB) || defined(CONFIG_ARCH_CHIP_STM32L072RZ)
#  define STM32_NATIM           0  /* No advanced timers */
#  define STM32_NGTIM16         4  /* 16-bit general up/down timers TIM2-3
                                    * (with DMA) and TIM21-22 without DMA */
#  define STM32_NGTIM32         0  /* No 32-bit general up/down timers */
#  define STM32_NBTIM           2  /* Two basic timers: TIM6, TIM7 with DMA */
                                   /* One LPTIMER */
#  define STM32_NSPI            2  /* Two SPI modules SPI1-2 */
#  define STM32_NI2S            1  /* One I2S module */
#  define STM32_NI2C            3  /* Three I2C (2 with SMBus/PMBus) */
#  define STM32_NDMA            1  /* One DMA1, 7-channels */
#  define STM32_NUSART          4  /* Four USART modules, USART1-4 */
                                   /* One LPUART */
#  define STM32_NCAN            0  /* No CAN controllers */
#  define STM32_NLCD            0  /* No LCD */
#  define STM32_NUSBDEV         0  /* No USB full-speed device controller */
#  define STM32_NUSBOTG         1  /* One USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NCEC            0  /* No HDMI-CEC controller */
#  define STM32_NADC            1  /* One 12-bit module */
#  define STM32_NDAC            2  /* Two DAC channels */
#  define STM32_NCOMP           2  /* Two Analog Comparators */
#  define STM32_NCRC            1  /* One CRC module */
#  define STM32_NRNG            1  /* One Random number generator (RNG) */
#  define STM32_NCAP            24 /* Twenty-four Capacitive sensing channels */
#  define STM32_NPORTS          6  /* Six GPIO ports, GPIOA-E, H */

#elif defined(CONFIG_ARCH_CHIP_STM32L073V8) || defined(CONFIG_ARCH_CHIP_STM32L073VB) || \
      defined(CONFIG_ARCH_CHIP_STM32L073VZ)
#  define STM32_NATIM           0  /* No advanced timers */
#  define STM32_NGTIM16         4  /* 16-bit general up/down timers TIM2-3
                                    * (with DMA) and TIM21-22 without DMA */
#  define STM32_NGTIM32         0  /* No 32-bit general up/down timers */
#  define STM32_NBTIM           2  /* Two basic timers: TIM6, TIM7 with DMA */
                                   /* One LPTIMER */
#  define STM32_NSPI            2  /* Two SPI modules SPI1-2 */
#  define STM32_NI2S            1  /* One I2S module */
#  define STM32_NI2C            3  /* Three I2C (2 with SMBus/PMBus) */
#  define STM32_NDMA            1  /* One DMA1, 7-channels */
#  define STM32_NUSART          4  /* Four USART modules, USART1-4 */
                                   /* One LPUART */
#  define STM32_NCAN            0  /* No CAN controllers */
#  define STM32_NLCD            1  /* One LCD controller */
#  define STM32_NUSBDEV         0  /* No USB full-speed device controller */
#  define STM32_NUSBOTG         1  /* One USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NCEC            0  /* No HDMI-CEC controller */
#  define STM32_NADC            1  /* One 12-bit module */
#  define STM32_NDAC            2  /* Two DAC channels */
#  define STM32_NCOMP           2  /* Two Analog Comparators */
#  define STM32_NCRC            1  /* One CRC module */
#  define STM32_NRNG            1  /* One Random number generator (RNG) */
#  define STM32_NCAP            24 /* Twenty-four Capacitive sensing channels */
#  define STM32_NPORTS          6  /* Six GPIO ports, GPIOA-E, H */

#elif defined(CONFIG_ARCH_CHIP_STM32L073CB) || defined(CONFIG_ARCH_CHIP_STM32L073CZ)
#  define STM32_NATIM           0  /* No advanced timers */
#  define STM32_NGTIM16         4  /* 16-bit general up/down timers TIM2-3
                                    * (with DMA) and TIM21-22 without DMA */
#  define STM32_NGTIM32         0  /* No 32-bit general up/down timers */
#  define STM32_NBTIM           2  /* Two basic timers: TIM6, TIM7 with DMA */
                                   /* One LPTIMER */
#  define STM32_NSPI            2  /* Two SPI modules SPI1-2 */
#  define STM32_NI2S            1  /* One I2S module */
#  define STM32_NI2C            3  /* Three I2C (2 with SMBus/PMBus) */
#  define STM32_NDMA            1  /* One DMA1, 7-channels */
#  define STM32_NUSART          4  /* Four USART modules, USART1-4 */
                                   /* One LPUART */
#  define STM32_NCAN            0  /* No CAN controllers */
#  define STM32_NLCD            1  /* One LCD controller */
#  define STM32_NUSBDEV         0  /* No USB full-speed device controller */
#  define STM32_NUSBOTG         1  /* One USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NCEC            0  /* No HDMI-CEC controller */
#  define STM32_NADC            1  /* One 12-bit module */
#  define STM32_NDAC            2  /* Two DAC channels */
#  define STM32_NCOMP           2  /* Two Analog Comparators */
#  define STM32_NCRC            1  /* One CRC module */
#  define STM32_NRNG            1  /* One Random number generator (RNG) */
#  define STM32_NCAP            17 /* Seventeen Capacitive sensing channels */
#  define STM32_NPORTS          6  /* Six GPIO ports, GPIOA-E, H */

#elif defined(CONFIG_ARCH_CHIP_STM32L073RB) || defined(CONFIG_ARCH_CHIP_STM32L073RZ)
#  define STM32_NATIM           0  /* No advanced timers */
#  define STM32_NGTIM16         4  /* 16-bit general up/down timers TIM2-3
                                    * (with DMA) and TIM21-22 without DMA */
#  define STM32_NGTIM32         0  /* No 32-bit general up/down timers */
#  define STM32_NBTIM           2  /* Two basic timers: TIM6, TIM7 with DMA */
                                   /* One LPTIMER */
#  define STM32_NSPI            2  /* Two SPI modules SPI1-2 */
#  define STM32_NI2S            1  /* One I2S module */
#  define STM32_NI2C            3  /* Three I2C (2 with SMBus/PMBus) */
#  define STM32_NDMA            1  /* One DMA1, 7-channels */
#  define STM32_NUSART          4  /* Four USART modules, USART1-4 */
                                   /* One LPUART */
#  define STM32_NCAN            0  /* No CAN controllers */
#  define STM32_NLCD            1  /* One LCD controller */
#  define STM32_NUSBDEV         0  /* No USB full-speed device controller */
#  define STM32_NUSBOTG         1  /* One USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NCEC            0  /* No HDMI-CEC controller */
#  define STM32_NADC            1  /* One 12-bit module */
#  define STM32_NDAC            2  /* Two DAC channels */
#  define STM32_NCOMP           2  /* Two Analog Comparators */
#  define STM32_NCRC            1  /* One CRC module */
#  define STM32_NRNG            1  /* One Random number generator (RNG) */
#  define STM32_NCAP            24 /* Twenty-four Capacitive sensing channels */
#  define STM32_NPORTS          6  /* Six GPIO ports, GPIOA-E, H */

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

#endif /* __ARCH_ARM_INCLUDE_STM32L0_CHIP_H */
