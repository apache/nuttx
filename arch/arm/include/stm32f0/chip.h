/****************************************************************************
 * arch/arm/include/stm32f0/chip.h
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

#ifndef __ARCH_ARM_INCLUDE_STM32F0_CHIP_H
#define __ARCH_ARM_INCLUDE_STM32F0_CHIP_H

#define ARMV6M_PERIPHERAL_INTERRUPTS 32

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* Get customizations for each supported chip */

#if defined(CONFIG_ARCH_CHIP_STM32F030RC) || defined(CONFIG_ARCH_CHIP_STM32F030CC)

#  define STM32_FLASH_SIZE      (256 * 1024) /* 256Kb */
#  define STM32_SRAM_SIZE       (32 * 1024)  /*  32Kb */

#  define STM32_NSPI            2  /* Two SPI modules (SPI or I2S) */
#  define STM32_NI2S            0  /* No I2S modules */
#  define STM32_NI2C            2  /* Two I2C modules */
#  define STM32_NDMA            1  /* 1 DMA1, 7-channels */
#  define STM32_NUSART          6  /* Six USARTs modules */
#  define STM32_NCAN            0  /* No CAN controllers */
#  define STM32_NUSBDEV         0  /* One USB full-speed device controller */
#  define STM32_NUSBOTG         0  /* No USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NADC            1  /* One 12-bit module */
#  define STM32_NDAC            0  /* One DAC channel */
#  define STM32_NCOMP           0  /* Two Analog Comparators */
#  define STM32_NCAP            0  /* Capacitive sensing channels (14 on UFQFPN32)) */
#  define STM32_NPORTS          5  /* Five GPIO ports, GPIOA-D, F */

#elif defined(CONFIG_ARCH_CHIP_STM32F051R8)

#  define STM32_FLASH_SIZE      (64 * 1024) /* 64Kb */
#  define STM32_SRAM_SIZE       (8 * 1024)  /*  8Kb */

#  define STM32_NSPI            2  /* Two SPI modules (SPI or I2S) */
#  define STM32_NI2S            2  /* Two I2S modules (SPI or I2S) */
#  define STM32_NI2C            2  /* Two I2C modules */
#  define STM32_NDMA            1  /* 1 DMA1, 7-channels */
#  define STM32_NUSART          2  /* Two USARTs modules */
#  define STM32_NCAN            0  /* No CAN controllers */
#  define STM32_NUSBDEV         1  /* One USB full-speed device controller */
#  define STM32_NUSBOTG         0  /* No USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NADC            1  /* One 12-bit module */
#  define STM32_NDAC            1  /* One DAC channel */
#  define STM32_NCOMP           2  /* Two Analog Comparators */
#  define STM32_NCAP            13 /* Capacitive sensing channels (14 on UFQFPN32)) */
#  define STM32_NPORTS          6  /* Six GPIO ports, GPIOA-F */

#elif defined(CONFIG_ARCH_CHIP_STM32F072C8) || defined(CONFIG_ARCH_CHIP_STM32F072CB)

#  ifdef CONFIG_ARCH_CHIP_STM32F072C8
#    define STM32_FLASH_SIZE    (64 * 1024)  /*  64Kb */
#  else
#    define STM32_FLASH_SIZE    (128 * 1024) /* 128Kb */
#  endif
#  define STM32_SRAM_SIZE       (16 * 1024)  /*  16Kb */

#  define STM32_NATIM           1  /* One advanced timer TIM1 */
#  define STM32_NGTIM16         5  /* 16-bit general up/down timers TIM3, TIM14-17 */
#  define STM32_NGTIM32         1  /* 32-bit general up/down timers TIM2 */
#  define STM32_NBTIM           2  /* 2 basic timers: TIM6, TIM7 */
#  define STM32_NSPI            2  /* Two SPI modules (SPI or I2S) */
#  define STM32_NI2S            2  /* Two I2S modules (SPI or I2S) */
#  define STM32_NI2C            2  /* Two I2C modules */
#  define STM32_NDMA            1  /* 1 DMA1, 7-channels */
#  define STM32_NUSART          4  /* Four USARTs module */
#  define STM32_NCAN            1  /* One CAN controller */
#  define STM32_NUSBDEV         1  /* One USB full-speed device controller */
#  define STM32_NUSBOTG         0  /* No USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NCEC            1  /* One HDMI-CEC controller */
#  define STM32_NADC            1  /* One 12-bit module */
#  define STM32_NDAC            2  /* Two DAC channel */
#  define STM32_NCOMP           2  /* Two Analog Comparators */
#  define STM32_NCAP            17 /* Capacitive sensing channels */
#  define STM32_NPORTS          6  /* Six GPIO ports, GPIOA-F */

#elif defined(CONFIG_ARCH_CHIP_STM32F072R8) || defined(CONFIG_ARCH_CHIP_STM32F072RB)

#  ifdef CONFIG_ARCH_CHIP_STM32F072R8
#    define STM32_FLASH_SIZE    (64*1024)  /*  64Kb */
#  else
#    define STM32_FLASH_SIZE    (128*1024) /* 128Kb */
#  endif
#  define STM32_SRAM_SIZE       (16*1024)  /*  16Kb */

#  define STM32_NATIM           1  /* One advanced timer TIM1 */
#  define STM32_NGTIM16         5  /* 16-bit general up/down timers TIM3, TIM14-17 */
#  define STM32_NGTIM32         1  /* 32-bit general up/down timers TIM2 */
#  define STM32_NBTIM           2  /* 2 basic timers: TIM6, TIM7 */
#  define STM32_NSPI            2  /* Two SPI modules (SPI or I2S) */
#  define STM32_NI2S            2  /* Two I2S modules (SPI or I2S) */
#  define STM32_NI2C            2  /* Two I2C modules */
#  define STM32_NDMA            1  /* 1 DMA1, 7-channels */
#  define STM32_NUSART          4  /* Four USARTs module */
#  define STM32_NCAN            1  /* One CAN controller */
#  define STM32_NUSBDEV         1  /* One USB full-speed device controller */
#  define STM32_NUSBOTG         0  /* No USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NCEC            1  /* One HDMI-CEC controller */
#  define STM32_NADC            1  /* One 12-bit module */
#  define STM32_NDAC            2  /* Two DAC channel */
#  define STM32_NCOMP           2  /* Two Analog Comparators */
#  define STM32_NCAP            18 /* Capacitive sensing channels */
#  define STM32_NPORTS          6  /* Six GPIO ports, GPIOA-F */

#elif defined(CONFIG_ARCH_CHIP_STM32F072V8) || defined(CONFIG_ARCH_CHIP_STM32F072VB)

#  ifdef CONFIG_ARCH_CHIP_STM32F072V8
#    define STM32_FLASH_SIZE    (64 * 1024)  /*  64Kb */
#  else
#    define STM32_FLASH_SIZE    (128 * 1024) /* 128Kb */
#  endif
#  define STM32_SRAM_SIZE       (16 * 1024)  /*  16Kb */

#  define STM32_NATIM           1  /* One advanced timer TIM1 */
#  define STM32_NGTIM16         5  /* 16-bit general up/down timers TIM3, TIM14-17 */
#  define STM32_NGTIM32         1  /* 32-bit general up/down timers TIM2 */
#  define STM32_NBTIM           2  /* 2 basic timers: TIM6, TIM7 */
#  define STM32_NSPI            2  /* Two SPI modules (SPI or I2S) */
#  define STM32_NI2S            2  /* Two I2S modules (SPI or I2S) */
#  define STM32_NI2C            2  /* Two I2C modules */
#  define STM32_NDMA            1  /* 1 DMA1, 7-channels */
#  define STM32_NUSART          4  /* Four USARTs module */
#  define STM32_NCAN            1  /* One CAN controller */
#  define STM32_NUSBDEV         1  /* One USB full-speed device controller */
#  define STM32_NUSBOTG         0  /* No USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NCEC            1  /* One HDMI-CEC controller */
#  define STM32_NADC            1  /* One 12-bit module */
#  define STM32_NDAC            2  /* Two DAC channel */
#  define STM32_NCOMP           2  /* Two Analog Comparators */
#  define STM32_NCAP            24 /* Capacitive sensing channels */
#  define STM32_NPORTS          6  /* Six GPIO ports, GPIOA-F */

#elif defined(CONFIG_ARCH_CHIP_STM32F091CB) || defined(CONFIG_ARCH_CHIP_STM32F091CC)

#  ifdef CONFIG_ARCH_CHIP_STM32F091CB
#    define STM32_FLASH_SIZE    (128 * 1024) /* 128Kb */
#  else
#    define STM32_FLASH_SIZE    (256 * 1024) /* 256Kb */
#  endif
#  define STM32_SRAM_SIZE       (32 * 1024)  /*  32Kb */

#  define STM32_NATIM           1  /* One advanced timer TIM1 */
#  define STM32_NGTIM16         5  /* 16-bit general up/down timers TIM3, TIM14-17 */
#  define STM32_NGTIM32         1  /* 32-bit general up/down timers TIM2 */
#  define STM32_NBTIM           2  /* 2 basic timers: TIM6, TIM7 */
#  define STM32_NSPI            2  /* Two SPI modules (SPI or I2S) */
#  define STM32_NI2S            2  /* Two I2S modules (SPI or I2S) */
#  define STM32_NI2C            2  /* Two I2C modules */
#  define STM32_NDMA            2  /* DMA1, DMA2 */
#  define STM32_NUSART          6  /* Six USARTs modules */
#  define STM32_NCAN            1  /* One CAN controller */
#  define STM32_NUSBDEV         0  /* No USB full-speed device controller */
#  define STM32_NUSBOTG         0  /* No USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NCEC            1  /* One HDMI-CEC controller */
#  define STM32_NADC            1  /* One 12-bit module */
#  define STM32_NDAC            2  /* Two DAC channel */
#  define STM32_NCOMP           2  /* Two Analog Comparators */
#  define STM32_NCAP            17 /* Capacitive sensing channels */
#  define STM32_NPORTS          6  /* Six GPIO ports, GPIOA-F */

#elif defined(CONFIG_ARCH_CHIP_STM32F091RB) || defined(CONFIG_ARCH_CHIP_STM32F091RC) || \
      defined(CONFIG_ARCH_CHIP_STM32F091VB) || defined(CONFIG_ARCH_CHIP_STM32F091VC)

#  if defined(CONFIG_ARCH_CHIP_STM32F091RB) || defined(CONFIG_ARCH_CHIP_STM32F091VB)
#    define STM32_FLASH_SIZE    (128 * 1024) /* 128Kb */
#  else
#    define STM32_FLASH_SIZE    (256 * 1024) /* 256Kb */
#  endif
#  define STM32_SRAM_SIZE       (32 * 1024)  /*  32Kb */

#  define STM32_NATIM           1  /* One advanced timer TIM1 */
#  define STM32_NGTIM16         5  /* 16-bit general up/down timers TIM3, TIM14-17 */
#  define STM32_NGTIM32         1  /* 32-bit general up/down timers TIM2 */
#  define STM32_NBTIM           2  /* 2 basic timers: TIM6, TIM7 */
#  define STM32_NSPI            2  /* Two SPI modules (SPI or I2S) */
#  define STM32_NI2S            2  /* Two I2S modules (SPI or I2S) */
#  define STM32_NI2C            2  /* Two I2C modules */
#  define STM32_NDMA            2  /* DMA1, DMA2 */
#  define STM32_NUSART          8  /* Eight USARTs modules */
#  define STM32_NCAN            1  /* One CAN controller */
#  define STM32_NUSBDEV         0  /* No USB full-speed device controller */
#  define STM32_NUSBOTG         0  /* No USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NCEC            1  /* One HDMI-CEC controller */
#  define STM32_NADC            1  /* One 12-bit module */
#  define STM32_NDAC            2  /* Two DAC channel */
#  define STM32_NCOMP           2  /* Two Analog Comparators */
#  if defined(CONFIG_ARCH_CHIP_STM32F091VB) || defined(CONFIG_ARCH_CHIP_STM32F091VC)
#    define STM32_NCAP          24 /* Capacitive sensing channels */
#  else
#    define STM32_NCAP          18 /* Capacitive sensing channels */
#  endif
#  define STM32_NPORTS          6  /* Six GPIO ports, GPIOA-F */

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

#endif /* __ARCH_ARM_INCLUDE_STM32F0_CHIP_H */
