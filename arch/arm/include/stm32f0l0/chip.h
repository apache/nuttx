/************************************************************************************
 * arch/arm/include/stm32f0l0/chip.h
 *
 *   Copyright (C) 2017-2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Alan Carvalho de Assis <acassis@gmail.com>
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

#ifndef __ARCH_ARM_INCLUDE_STM32F0L0_CHIP_H
#define __ARCH_ARM_INCLUDE_STM32F0L0_CHIP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Get customizations for each supported chip */

#if defined(CONFIG_ARCH_CHIP_STM32F051R8)

#  define STM32_FLASH_SIZE      (64*1024) /* 64Kb */
#  define STM32_SRAM_SIZE       (8*1024)  /*  8Kb */

#  define STM32_NSPI            2  /* Two SPI modules (SPI or I2S) */
#  define STM32_NI2S            2  /* Two I2S modules (SPI or I2S) */
#  define STM32_NI2C            2  /* Two I2C modules */
#  define STM32_NUSART          2  /* Two USARTs modules */
#  define STM32_NCAN            0  /* No CAN controllers */
#  define STM32_NUSBDEV         1  /* One USB full-speed device controller */
#  define STM32_NUSBOTG         0  /* No USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NDAC            1  /* One DAC module */
#  define STM32_NDACCHAN        1  /* One DAC channels */
#  define STM32_NCOMP           2  /* Two Analog Comparators */
#  define STM32_NCAP            13 /* Capacitive sensing channels (14 on UFQFPN32)) */
#  define STM32_NPORTS          6  /* Six GPIO ports, GPIOA-F */

#elif defined(CONFIG_ARCH_CHIP_STM32F072C8) || defined(CONFIG_ARCH_CHIP_STM32F072CB)

#  ifdef CONFIG_ARCH_CHIP_STM32F072C8
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
#  define STM32_NUSART          4  /* Four USARTs module */
#  define STM32_NCAN            1  /* One CAN controller */
#  define STM32_NUSBDEV         1  /* One USB full-speed device controller */
#  define STM32_NUSBOTG         0  /* No USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NCEC            1  /* One HDMI-CEC controller */
#  define STM32_NADC12          1  /* One 12-bit module */
#  define STM32_NADCCHAN        10 /* Ten external channels */
#  define STM32_NADCINT         3  /* Three internal channels */
#  define STM32_NDAC            1  /* One DAC module */
#  define STM32_NDACCHAN        2  /* Two DAC channels */
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
#  define STM32_NUSART          4  /* Four USARTs module */
#  define STM32_NCAN            1  /* One CAN controller */
#  define STM32_NUSBDEV         1  /* One USB full-speed device controller */
#  define STM32_NUSBOTG         0  /* No USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NCEC            1  /* One HDMI-CEC controller */
#  define STM32_NADC12          1  /* One 12-bit module */
#  define STM32_NADCCHAN        16 /* 16 external channels */
#  define STM32_NADCINT         3  /* Three internal channels */
#  define STM32_NDAC            1  /* One DAC module */
#  define STM32_NDACCHAN        2  /* Two DAC channels */
#  define STM32_NCOMP           2  /* Two Analog Comparators */
#  define STM32_NCAP            18 /* Capacitive sensing channels */
#  define STM32_NPORTS          6  /* Six GPIO ports, GPIOA-F */

#elif defined(CONFIG_ARCH_CHIP_STM32F072V8) || defined(CONFIG_ARCH_CHIP_STM32F072VB)

#  ifdef CONFIG_ARCH_CHIP_STM32F072V8
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
#  define STM32_NUSART          4  /* Four USARTs module */
#  define STM32_NCAN            1  /* One CAN controller */
#  define STM32_NUSBDEV         1  /* One USB full-speed device controller */
#  define STM32_NUSBOTG         0  /* No USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NCEC            1  /* One HDMI-CEC controller */
#  define STM32_NADC12          1  /* One 12-bit module */
#  define STM32_NADCCHAN        16 /* 16 external channels */
#  define STM32_NADCINT         3  /* Three internal channels */
#  define STM32_NDAC            1  /* One DAC module */
#  define STM32_NDACCHAN        2  /* Two DAC channels */
#  define STM32_NCOMP           2  /* Two Analog Comparators */
#  define STM32_NCAP            24 /* Capacitive sensing channels */
#  define STM32_NPORTS          6  /* Six GPIO ports, GPIOA-F */

#elif defined(CONFIG_ARCH_CHIP_STM32F091CB) || defined(CONFIG_ARCH_CHIP_STM32F091CC)

#  ifdef CONFIG_ARCH_CHIP_STM32F091CB
#    define STM32_FLASH_SIZE    (128*1024) /* 128Kb */
#  else
#    define STM32_FLASH_SIZE    (256*1024) /* 256Kb */
#  endif
#  define STM32_SRAM_SIZE       (32*1024)  /*  32Kb */

#  define STM32_NATIM           1  /* One advanced timer TIM1 */
#  define STM32_NGTIM16         5  /* 16-bit general up/down timers TIM3, TIM14-17 */
#  define STM32_NGTIM32         1  /* 32-bit general up/down timers TIM2 */
#  define STM32_NBTIM           2  /* 2 basic timers: TIM6, TIM7 */
#  define STM32_NSPI            2  /* Two SPI modules (SPI or I2S) */
#  define STM32_NI2S            2  /* Two I2S modules (SPI or I2S) */
#  define STM32_NI2C            2  /* Two I2C modules */
#  define STM32_NUSART          6  /* Six USARTs modules */
#  define STM32_NCAN            1  /* One CAN controller */
#  define STM32_NUSBDEV         0  /* No USB full-speed device controller */
#  define STM32_NUSBOTG         0  /* No USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NCEC            1  /* One HDMI-CEC controller */
#  define STM32_NADC12          1  /* One 12-bit module */
#  define STM32_NADCCHAN        10 /* 10 external channels */
#  define STM32_NADCINT         3  /* Three internal channels */
#  define STM32_NDAC            1  /* One DAC module */
#  define STM32_NDACCHAN        2  /* Two DAC channels */
#  define STM32_NCOMP           2  /* Two Analog Comparators */
#  define STM32_NCAP            17 /* Capacitive sensing channels */
#  define STM32_NPORTS          6  /* Six GPIO ports, GPIOA-F */

#elif defined(CONFIG_ARCH_CHIP_STM32F091RB) || defined(CONFIG_ARCH_CHIP_STM32F091RC) || \
      defined(CONFIG_ARCH_CHIP_STM32F091VB) || defined(CONFIG_ARCH_CHIP_STM32F091VC)

#  if defined(CONFIG_ARCH_CHIP_STM32F091RB) || defined(CONFIG_ARCH_CHIP_STM32F091VB)
#    define STM32_FLASH_SIZE    (128*1024) /* 128Kb */
#  else
#    define STM32_FLASH_SIZE    (256*1024) /* 256Kb */
#  endif
#  define STM32_SRAM_SIZE       (32*1024)  /*  32Kb */

#  define STM32_NATIM           1  /* One advanced timer TIM1 */
#  define STM32_NGTIM16         5  /* 16-bit general up/down timers TIM3, TIM14-17 */
#  define STM32_NGTIM32         1  /* 32-bit general up/down timers TIM2 */
#  define STM32_NBTIM           2  /* 2 basic timers: TIM6, TIM7 */
#  define STM32_NSPI            2  /* Two SPI modules (SPI or I2S) */
#  define STM32_NI2S            2  /* Two I2S modules (SPI or I2S) */
#  define STM32_NI2C            2  /* Two I2C modules */
#  define STM32_NUSART          8  /* Eight USARTs modules */
#  define STM32_NCAN            1  /* One CAN controller */
#  define STM32_NUSBDEV         0  /* No USB full-speed device controller */
#  define STM32_NUSBOTG         0  /* No USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NCEC            1  /* One HDMI-CEC controller */
#  define STM32_NADC12          1  /* One 12-bit module */
#  define STM32_NADCCHAN        16 /* 16 external channels */
#  define STM32_NADCINT         3  /* Three internal channels */
#  define STM32_NDAC            1  /* One DAC module */
#  define STM32_NDACCHAN        2  /* Two DAC channels */
#  define STM32_NCOMP           2  /* Two Analog Comparators */
#  if defined(CONFIG_ARCH_CHIP_STM32F091VB) || defined(CONFIG_ARCH_CHIP_STM32F091VC)
#    define STM32_NCAP          24 /* Capacitive sensing channels */
#  else
#    define STM32_NCAP          18 /* Capacitive sensing channels */
#  endif
#  define STM32_NPORTS          6  /* Six GPIO ports, GPIOA-F */

/* STM32L EnergyLite Line ***********************************************************/
 
/* STM32L03XX  - With LCD
 * STM32L02XX  - No LCD
 *
 * STM32L0XXX8 - 64KB FLASH, 20KB SRAM, 3KB EEPROM
 * STM32L0XXXB - 128KB FLASH, 20KB SRAM, 6KB EEPROM
 * STM32L0XXXZ - 192KB FLASH, 20KB SRAM, 3KB EEPROM
 *
 * STM32L0XXCX - 48-pins
 * STM32L0XXRX - 64-pins
 * STM32L0XXVX - 100-pins
 */

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
#  define STM32_NADC12          1  /* One 12-bit ADC module */
#  define STM32_NADCCHAN        14 /* 14 channels */
#  define STM32_NADCINT         0  /* ? internal channels vs external? */
#  define STM32_NDAC            2  /* Two DAC module */
#  define STM32_NDACCHAN        2  /* Two DAC channels */
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
#  define STM32_NADC                     1   /* (1) ADC1, 14-channels */
#  define STM32_NDAC            2  /* Two DAC module */
#  define STM32_NDACCHAN        2  /* Two DAC channels */
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
#  define STM32_NADC12          1  /* One 12-bit ADC module */
#  define STM32_NADCCHAN        14 /* 14 channels */
#  define STM32_NADCINT         0  /* ? internal channels vs external? */
#  define STM32_NDAC            2  /* Two DAC module */
#  define STM32_NDACCHAN        2  /* Two DAC channels */
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
#  define STM32_NADC12          1  /* One 12-bit ADC module */
#  define STM32_NADCCHAN        14 /* 14 channels */
#  define STM32_NADCINT         0  /* ? internal channels vs external? */
#  define STM32_NDAC            2  /* Two DAC module */
#  define STM32_NDACCHAN        2  /* Two DAC channels */
#  define STM32_NCOMP           2  /* Two Analog Comparators */
#  define STM32_NCRC            1   /* One CRC module */
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
#  define STM32_NADC12          1  /* One 12-bit ADC module */
#  define STM32_NADCCHAN        14 /* 14 channels */
#  define STM32_NADCINT         0  /* ? internal channels vs external? */
#  define STM32_NDAC            2  /* Two DAC module */
#  define STM32_NDACCHAN        2  /* Two DAC channels */
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
#  define STM32_NADC12          1  /* One 12-bit ADC module */
#  define STM32_NADCCHAN        14 /* 14 channels */
#  define STM32_NADCINT         0  /* ? internal channels vs external? */
#  define STM32_NDAC            2  /* Two DAC module */
#  define STM32_NDACCHAN        2  /* Two DAC channels */
#  define STM32_NCOMP           2  /* Two Analog Comparators */
#  define STM32_NCRC            1   /* One CRC module */
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
#  define STM32_NADC12          1  /* One 12-bit ADC module */
#  define STM32_NADCCHAN        14 /* 14 channels */
#  define STM32_NADCINT         0  /* ? internal channels vs external? */
#  define STM32_NDAC            2  /* Two DAC module */
#  define STM32_NDACCHAN        2  /* Two DAC channels */
#  define STM32_NCOMP           2  /* Two Analog Comparators */
#  define STM32_NCRC            1  /* One CRC module */
#  define STM32_NRNG            1  /* One Random number generator (RNG) */
#  define STM32_NCAP            24 /* Twenty-four Capacitive sensing channels */
#  define STM32_NPORTS          6  /* Six GPIO ports, GPIOA-E, H */

#else
#  error "Unsupported STM32F0xx chip"
#endif

/* NVIC priority levels *************************************************************/
/* Each priority field holds a priority value, 0-31. The lower the value, the greater
 * the priority of the corresponding interrupt. The processor implements only
 * bits[7:6] of each field, bits[5:0] read as zero and ignore writes.
 */

#define NVIC_SYSH_PRIORITY_MIN     0xc0 /* All bits[7:6] set is minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT 0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX     0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP    0x40 /* Two bits of interrupt priority used */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_INCLUDE_STM32F0L0_CHIP_H */
