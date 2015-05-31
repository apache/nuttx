/************************************************************************************
 * arch/arm/include/stm32/chip.h
 *
 *   Copyright (C) 2009, 2011-2014 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_INCLUDE_STM32_CHIP_H
#define __ARCH_ARM_INCLUDE_STM32_CHIP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Get customizations for each supported chip and provide alternate function pin-mapping
 *
 * NOTE: Each GPIO pin may serve either for general purpose I/O or for a special
 * alternate function (such as USART, CAN, USB, SDIO, etc.).  That particular
 * pin-mapping will depend on the package and STM32 family.  If you are incorporating
 * a new STM32 chip into NuttX, you will need to add the pin-mapping to a header file
 * and to include that header file below. The chip-specific pin-mapping is defined in
 * the chip datasheet.
 */

/* STM32L EnergyLite Line ************************************************************/

/* STM32L151XX -- No LCD
 * STM32L152XX -- With LCD
 *
 * STM32L15XCX -- 48-pins
 * STM32L15XRX -- 64-pins
 * STM32L15XVX -- 100-pins
 *
 * STM32L15XX6 -- 32KB FLASH, 10KB SRAM, 4KB EEPROM
 * STM32L15XX8 -- 64KB FLASH, 10KB SRAM, 4KB EEPROM
 * STM32L15XXB -- 128KB FLASH, 16KB SRAM, 4KB EEPROM
 *
 * STM32L15XXC -- 256KB FLASH, 32KB SRAM, 8KB EEPROM (medium+ density)
 */

#if defined(CONFIG_ARCH_CHIP_STM32L151C6) || defined(CONFIG_ARCH_CHIP_STM32L151C8) || \
    defined(CONFIG_ARCH_CHIP_STM32L151CB)
#  define CONFIG_STM32_STM32L15XX        1   /* STM32L151xx and STM32L152xx family */
#  define CONFIG_STM32_ENERGYLITE        1   /* STM32L EnergyLite family */
#  undef  CONFIG_STM32_STM32F10XX            /* STM32F10xxx family */
#  define CONFIG_STM32_LOWDENSITY        1   /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes
                                              * and STM32L15xxx */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_MEDIUMPLUSDENSITY     /* STM32L15xxC w/ 32/256 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  undef  CONFIG_STM32_STM32F30XX            /* STM32F30xxx family */
#  undef  CONFIG_STM32_STM32F37XX            /* STM32F37xxx family */
#  undef  CONFIG_STM32_STM32F40XX            /* STM32F405xx and STM32407xx families */
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    0   /* No advanced timers */
#  define STM32_NGTIM                    6   /* (3) 16-bit general up/down timers TIM2,3,4 with DMA */
                                             /* (3) 16-bit general up timers TIM9, 10, 11 without DMA */
#  define STM32_NBTIM                    2   /* 2 basic timers: TIM6, TIM7 with DMA */
#  define STM32_NDMA                     1   /* DMA1, 7-channels */
#  define STM32_NSPI                     2   /* SPI1-2 */
#  define STM32_NI2S                     0   /* No I2S */
#  define STM32_NUSART                   3   /* USART1-3 */
#  define STM32_NI2C                     2   /* I2C1-2 */
#  define STM32_NCAN                     0   /* No CAN */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  0   /* No USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NGPIO                    37  /* GPIOA-E,H */
#  define STM32_NADC                     1   /* ADC1, 16-channels */
#  define STM32_NDAC                     2   /* DAC 1-2, 2 channels */
                                             /* (2) Comparators */
#  define STM32_NCAPSENSE                13  /* Capacitive sensing channels */
#  define STM32_NCRC                     0   /* No CRC */
#  define STM32_NETHERNET                0   /* No Ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32L151R6) || defined(CONFIG_ARCH_CHIP_STM32L151R8) || \
      defined(CONFIG_ARCH_CHIP_STM32L151RB)
#  define CONFIG_STM32_STM32L15XX        1   /* STM32L151xx and STM32L152xx family */
#  define CONFIG_STM32_ENERGYLITE        1   /* STM32L EnergyLite family */
#  undef  CONFIG_STM32_STM32F10XX            /* STM32F10xxx family */
#  define CONFIG_STM32_LOWDENSITY        1   /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes
                                              * and STM32L15xxx */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_MEDIUMPLUSDENSITY     /* STM32L15xxC w/ 32/256 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  undef  CONFIG_STM32_STM32F30XX            /* STM32F30xxx family */
#  undef  CONFIG_STM32_STM32F37XX            /* STM32F37xxx family */
#  undef  CONFIG_STM32_STM32F40XX            /* STM32F405xx and STM32407xx families */
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    0   /* No advanced timers */
#  define STM32_NGTIM                    6   /* (3) 16-bit general up/down timers TIM2,3,4 with DMA */
                                             /* (3) 16-bit general up timers TIM9, 10, 11 without DMA */
#  define STM32_NBTIM                    2   /* 2 basic timers: TIM6, TIM7 with DMA */
#  define STM32_NDMA                     1   /* DMA1, 7-channels */
#  define STM32_NSPI                     2   /* SPI1-2 */
#  define STM32_NI2S                     0   /* No I2S */
#  define STM32_NUSART                   3   /* USART1-3 */
#  define STM32_NI2C                     2   /* I2C1-2 */
#  define STM32_NCAN                     0   /* No CAN */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  0   /* No USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NGPIO                    51  /* GPIOA-E,H */
#  define STM32_NADC                     1   /* ADC1, 20-channels */
#  define STM32_NDAC                     2   /* DAC 1-2, 2 channels */
                                             /* (2) Comparators */
#  define STM32_NCAPSENSE                20  /* Capacitive sensing channels */
#  define STM32_NCRC                     0   /* No CRC */
#  define STM32_NETHERNET                0   /* No Ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32L151V6) || defined(CONFIG_ARCH_CHIP_STM32L151V8) || \
      defined(CONFIG_ARCH_CHIP_STM32L151VB)
#  define CONFIG_STM32_STM32L15XX        1   /* STM32L151xx and STM32L152xx family */
#  define CONFIG_STM32_ENERGYLITE        1   /* STM32L EnergyLite family */
#  undef  CONFIG_STM32_STM32F10XX            /* STM32F10xxx family */
#  define CONFIG_STM32_LOWDENSITY        1   /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes
                                              * and STM32L15xxx */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_MEDIUMPLUSDENSITY     /* STM32L15xxC w/ 32/256 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  undef  CONFIG_STM32_STM32F30XX            /* STM32F30xxx family */
#  undef  CONFIG_STM32_STM32F37XX            /* STM32F37xxx family */
#  undef  CONFIG_STM32_STM32F40XX            /* STM32F405xx and STM32407xx families */
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    0   /* No advanced timers */
#  define STM32_NGTIM                    6   /* (3) 16-bit general up/down timers TIM2,3,4 with DMA */
                                             /* (3) 16-bit general up timers TIM9, 10, 11 without DMA */
#  define STM32_NBTIM                    2   /* 2 basic timers: TIM6, TIM7 with DMA */
#  define STM32_NDMA                     1   /* DMA1, 7-channels */
#  define STM32_NSPI                     2   /* SPI1-2 */
#  define STM32_NI2S                     0   /* No I2S */
#  define STM32_NUSART                   3   /* USART1-3 */
#  define STM32_NI2C                     2   /* I2C1-2 */
#  define STM32_NCAN                     0   /* No CAN */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  0   /* No USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NGPIO                    83  /* GPIOA-E,H */
#  define STM32_NADC                     1   /* ADC1, 24-channels */
#  define STM32_NDAC                     2   /* DAC 1-2, 2 channels */
                                             /* (2) Comparators */
#  define STM32_NCAPSENSE                20  /* Capacitive sensing channels */
#  define STM32_NCRC                     0   /* No CRC */
#  define STM32_NETHERNET                0   /* No Ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32L152C6) || defined(CONFIG_ARCH_CHIP_STM32L152C8) || \
      defined(CONFIG_ARCH_CHIP_STM32L152CB)
#  define CONFIG_STM32_STM32L15XX        1   /* STM32L151xx and STM32L152xx family */
#  define CONFIG_STM32_ENERGYLITE        1   /* STM32L EnergyLite family */
#  undef  CONFIG_STM32_STM32F10XX            /* STM32F10xxx family */
#  define CONFIG_STM32_LOWDENSITY        1   /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes
                                              * and STM32L15xxx */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_MEDIUMPLUSDENSITY     /* STM32L15xxC w/ 32/256 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  undef  CONFIG_STM32_STM32F30XX            /* STM32F30xxx family */
#  undef  CONFIG_STM32_STM32F37XX            /* STM32F37xxx family */
#  undef  CONFIG_STM32_STM32F40XX            /* STM32F405xx and STM32407xx families */
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    0   /* No advanced timers */
#  define STM32_NGTIM                    6   /* (3) 16-bit general up/down timers TIM2,3,4 with DMA */
                                             /* (3) 16-bit general up timers TIM9, 10, 11 without DMA */
#  define STM32_NBTIM                    2   /* 2 basic timers: TIM6, TIM7 with DMA */
#  define STM32_NDMA                     1   /* DMA1, 7-channels */
#  define STM32_NSPI                     2   /* SPI1-2 */
#  define STM32_NI2S                     0   /* No I2S */
#  define STM32_NUSART                   3   /* USART1-3 */
#  define STM32_NI2C                     2   /* I2C1-2 */
#  define STM32_NCAN                     0   /* No CAN */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NLCD                     1   /* LCD 4x16 */
#  define STM32_NUSBOTG                  0   /* No USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NGPIO                    37  /* GPIOA-E,H */
#  define STM32_NADC                     1   /* ADC1, 16-channels */
#  define STM32_NDAC                     2   /* DAC 1-2, 2 channels */
                                             /* (2) Comparators */
#  define STM32_NCAPSENSE                13  /* Capacitive sensing channels */
#  define STM32_NCRC                     0   /* No CRC */
#  define STM32_NETHERNET                0   /* No Ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32L152R6) || defined(CONFIG_ARCH_CHIP_STM32L152R8) || \
      defined(CONFIG_ARCH_CHIP_STM32L152RB)
#  define CONFIG_STM32_STM32L15XX        1   /* STM32L151xx and STM32L152xx family */
#  define CONFIG_STM32_ENERGYLITE        1   /* STM32L EnergyLite family */
#  undef  CONFIG_STM32_STM32F10XX            /* STM32F10xxx family */
#  define CONFIG_STM32_LOWDENSITY        1   /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes
                                              * and STM32L15xxx */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_MEDIUMPLUSDENSITY     /* STM32L15xxC w/ 32/256 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  undef  CONFIG_STM32_STM32F30XX            /* STM32F30xxx family */
#  undef  CONFIG_STM32_STM32F37XX            /* STM32F37xxx family */
#  undef  CONFIG_STM32_STM32F40XX            /* STM32F405xx and STM32407xx families */
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    0   /* No advanced timers */
#  define STM32_NGTIM                    6   /* (3) 16-bit general up/down timers TIM2,3,4 with DMA */
                                             /* (3) 16-bit general up timers TIM9, 10, 11 without DMA */
#  define STM32_NBTIM                    2   /* 2 basic timers: TIM6, TIM7 with DMA */
#  define STM32_NDMA                     1   /* DMA1, 7-channels */
#  define STM32_NSPI                     2   /* SPI1-2 */
#  define STM32_NI2S                     0   /* No I2S */
#  define STM32_NUSART                   3   /* USART1-3 */
#  define STM32_NI2C                     2   /* I2C1-2 */
#  define STM32_NCAN                     0   /* No CAN */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NLCD                     1   /* LCD 4x32, 8x28 */
#  define STM32_NUSBOTG                  0   /* No USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NGPIO                    51  /* GPIOA-E,H */
#  define STM32_NADC                     1   /* ADC1, 20-channels */
#  define STM32_NDAC                     2   /* DAC 1-2, 2 channels */
                                             /* (2) Comparators */
#  define STM32_NCAPSENSE                20  /* Capacitive sensing channels */
#  define STM32_NCRC                     0   /* No CRC */
#  define STM32_NETHERNET                0   /* No Ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32L152V6) || defined(CONFIG_ARCH_CHIP_STM32L152V8) || \
      defined(CONFIG_ARCH_CHIP_STM32L152VB)
#  define CONFIG_STM32_STM32L15XX        1   /* STM32L151xx and STM32L152xx family */
#  define CONFIG_STM32_ENERGYLITE        1   /* STM32L EnergyLite family */
#  undef  CONFIG_STM32_STM32F10XX            /* STM32F10xxx family */
#  define CONFIG_STM32_LOWDENSITY        1   /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes
                                              * and STM32L15xxx */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_MEDIUMPLUSDENSITY     /* STM32L15xxC w/ 32/256 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  undef  CONFIG_STM32_STM32F30XX            /* STM32F30xxx family */
#  undef  CONFIG_STM32_STM32F37XX            /* STM32F37xxx family */
#  undef  CONFIG_STM32_STM32F40XX            /* STM32F405xx and STM32407xx families */
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    0   /* No advanced timers */
#  define STM32_NGTIM                    6   /* (3) 16-bit general up/down timers TIM2,3,4 with DMA */
                                             /* (3) 16-bit general up timers TIM9, 10, 11 without DMA */
#  define STM32_NBTIM                    2   /* 2 basic timers: TIM6, TIM7 with DMA */
#  define STM32_NDMA                     1   /* DMA1, 7-channels */
#  define STM32_NSPI                     2   /* SPI1-2 */
#  define STM32_NI2S                     0   /* No I2S */
#  define STM32_NUSART                   3   /* USART1-3 */
#  define STM32_NI2C                     2   /* I2C1-2 */
#  define STM32_NCAN                     0   /* No CAN */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NLCD                     1   /* LCD 4x44, 8x40*/
#  define STM32_NUSBOTG                  0   /* No USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NGPIO                    83  /* GPIOA-E,H */
#  define STM32_NADC                     1   /* ADC1, 24-channels */
#  define STM32_NDAC                     2   /* DAC 1-2, 2 channels */
                                             /* (2) Comparators */
#  define STM32_NCAPSENSE                20  /* Capacitive sensing channels */
#  define STM32_NCRC                     0   /* No CRC */
#  define STM32_NETHERNET                0   /* No Ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32L152RC)
#  define CONFIG_STM32_STM32L15XX        1   /* STM32L151xx and STM32L152xx family */
#  define CONFIG_STM32_ENERGYLITE        1   /* STM32L EnergyLite vamily */
#  undef  CONFIG_STM32_STM32F10XX            /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes
                                              * and STM32L15xxx */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  define CONFIG_STM32_MEDIUMPLUSDENSITY 1   /* STM32L15xxC w/ 32/256 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  undef  CONFIG_STM32_STM32F30XX            /* STM32F30xxx family */
#  undef  CONFIG_STM32_STM32F40XX            /* STM32F405xx and STM32407xx families */
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    0   /* No advanced timers */
#  define STM32_NGTIM                    6   /* (3) 16-bit general up/down timers TIM2,3,4 with DMA */
                                             /* (3) 16-bit general up timers TIM9, 10, 11 without DMA */
#  define STM32_NBTIM                    2   /* 2 basic timers: TIM6, TIM7 with DMA */
#  define STM32_NDMA                     2   /* DMA1, 7-channels, DMA2 (5 channels) */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     2   /* I2S1-2, overlapping with SPI2-3 */
#  define STM32_NUSART                   3   /* USART1-3 */
#  define STM32_NI2C                     2   /* I2C1-2 */
#  define STM32_NCAN                     0   /* No CAN */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NLCD                     1   /* LCD 4x44, 8x40*/
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NGPIO                    83  /* GPIOA-E,H */
#  define STM32_NADC                     1   /* ADC1, 24-channels */
#  define STM32_NDAC                     2   /* DAC 1-2, 2 channels */
                                             /* (2) Comparators */
#  define STM32_NCAPSENSE                20  /* Capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32L162ZD)
#  define CONFIG_STM32_STM32L15XX        1   /* STM32L151xx and STM32L152xx family */
#  define CONFIG_STM32_ENERGYLITE        1   /* STM32L EnergyLite vamily */
#  undef  CONFIG_STM32_STM32F10XX            /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes
                                              * and STM32L15xxx */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_MEDIUMPLUSDENSITY     /* STM32L15xxC w/ 32/256 Kbytes */
#  define CONFIG_STM32_HIGHDENSITY       1   /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes, STM32L16x w/ 48/384 Kbytes. */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  undef  CONFIG_STM32_STM32F30XX            /* STM32F30xxx family */
#  undef  CONFIG_STM32_STM32F40XX            /* STM32F405xx and STM32407xx families */
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    0   /* No advanced timers */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM2,3,4 with DMA
                                              * 32-bit general timer TIM5 with DMA */
#  define STM32_NGTIMNDMA                3   /* 16-bit general timers TIM9-11 without DMA */
#  define STM32_NBTIM                    2   /* 2 basic timers: TIM6, TIM7 without DMA */
#  define STM32_NDMA                     2   /* DMA1, 7-channels, DMA2 (5 channels) */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     2   /* I2S1-2, overlapping with SPI2-3 */
#  define STM32_NUSART                   5   /* USART1-3, UART4-5 */
#  define STM32_NI2C                     2   /* I2C1-2 */
#  define STM32_NCAN                     0   /* No CAN */
#  define STM32_NSDIO                    1   /* SDIO */
#  define STM32_NLCD                     1   /* LCD 4x44, 8x40*/
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NGPIO                    115 /* GPIOA-G,H */
#  define STM32_NADC                     1   /* ADC1, 24-channels */
#  define STM32_NDAC                     2   /* DAC 1-2, 2 channels */
                                             /* (2) Comparators */
#  define STM32_NCAPSENSE                20  /* Capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

/* STM32 F100 Value Line ************************************************************/

#elif defined(CONFIG_ARCH_CHIP_STM32F100C8) || defined(CONFIG_ARCH_CHIP_STM32F100CB) \
 || defined(CONFIG_ARCH_CHIP_STM32F100R8) || defined(CONFIG_ARCH_CHIP_STM32F100RB)
#  undef  CONFIG_STM32_STM32L15XX            /* STM32L151xx and STM32L152xx family */
#  undef  CONFIG_STM32_ENERGYLITE            /* STM32L EnergyLite family */
#  define CONFIG_STM32_STM32F10XX        1   /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  define CONFIG_STM32_MEDIUMDENSITY     1   /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_MEDIUMPLUSDENSITY     /* STM32L15xxC w/ 32/256 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  define CONFIG_STM32_VALUELINE         1   /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  undef  CONFIG_STM32_STM32F30XX            /* STM32F30xxx family */
#  undef  CONFIG_STM32_STM32F37XX            /* STM32F37xxx family */
#  undef  CONFIG_STM32_STM32F40XX            /* STM32F405xx and STM32407xx families */
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    1   /* One advanced timer TIM1 */
#  define STM32_NGTIM                    3   /* 16-bit general timers TIM2,3,4 with DMA */
#  define STM32_NBTIM                    2   /* 2 basic timers: TIM6, TIM7 */
// TODO: there are also 3 additional timers (15-17) that don't fit any existing category
#  define STM32_NDMA                     1   /* DMA1 */
#  define STM32_NSPI                     2   /* SPI1-2 */
#  define STM32_NI2S                     0   /* No I2S */
#  define STM32_NUSART                   3   /* USART1-3 */
#  define STM32_NI2C                     2   /* I2C1-2 */
#  define STM32_NCAN                     0   /* No CAN */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  0   /* No USB OTG FS/HS */
#  define STM32_NGPIO                    64  /* GPIOA-D */
#  define STM32_NADC                     1   /* ADC1 */
#  define STM32_NDAC                     2   /* DAC 1-2 */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC1 */
#  define STM32_NETHERNET                0   /* No Ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F100V8) || defined(CONFIG_ARCH_CHIP_STM32F100VB)
#  undef  CONFIG_STM32_STM32L15XX            /* STM32L151xx and STM32L152xx family */
#  undef  CONFIG_STM32_ENERGYLITE            /* STM32L EnergyLite family */
#  define CONFIG_STM32_STM32F10XX        1   /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  define CONFIG_STM32_MEDIUMDENSITY     1   /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_MEDIUMPLUSDENSITY     /* STM32L15xxC w/ 32/256 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  define CONFIG_STM32_VALUELINE         1   /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  undef  CONFIG_STM32_STM32F30XX            /* STM32F30xxx family */
#  undef  CONFIG_STM32_STM32F37XX            /* STM32F37xxx family */
#  undef  CONFIG_STM32_STM32F40XX            /* STM32F405xx and STM32407xx families */
#  define STM32_NFSMC                    0   /* FSMC */
#  define STM32_NATIM                    1   /* One advanced timer TIM1 */
#  define STM32_NGTIM                    3   /* 16-bit general timers TIM2,3,4 with DMA */
#  define STM32_NBTIM                    2   /* 2 basic timers: TIM6, TIM7 */
// TODO: there are also 3 additional timers (15-17) that don't fit any existing category
#  define STM32_NDMA                     1   /* DMA1 */
#  define STM32_NSPI                     2   /* SPI1-2 */
#  define STM32_NI2S                     0   /* No I2S */
#  define STM32_NUSART                   3   /* USART1-3 */
#  define STM32_NI2C                     2   /* I2C1-2 */
#  define STM32_NCAN                     0   /* No CAN */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  0   /* No USB OTG FS/HS */
#  define STM32_NGPIO                    80  /* GPIOA-E */
#  define STM32_NADC                     1   /* ADC1 */
#  define STM32_NDAC                     2   /* DAC 1-2 */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC1 */
#  define STM32_NETHERNET                0   /* No Ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

/* STM32 F100 High-density value Line ************************************************************/

#elif defined(CONFIG_ARCH_CHIP_STM32F100RC) || defined(CONFIG_ARCH_CHIP_STM32F100RD) \
 || defined(CONFIG_ARCH_CHIP_STM32F100RE)
#  undef  CONFIG_STM32_STM32L15XX            /* STM32L151xx and STM32L152xx family */
#  undef  CONFIG_STM32_ENERGYLITE            /* STM32L EnergyLite family */
#  define CONFIG_STM32_STM32F10XX        1   /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_MEDIUMPLUSDENSITY     /* STM32L15xxC w/ 32/256 Kbytes */
#  define CONFIG_STM32_HIGHDENSITY       1   /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  define CONFIG_STM32_VALUELINE         1   /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  undef  CONFIG_STM32_STM32F30XX            /* STM32F30xxx family */
#  undef  CONFIG_STM32_STM32F37XX            /* STM32F37xxx family */
#  undef  CONFIG_STM32_STM32F40XX            /* STM32F405xx and STM32407xx families */
#  define STM32_NFSMC                    0   /* FSMC */
#  define STM32_NATIM                    1   /* One advanced timer TIM1 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM2,3,4,5 with DMA */
#  define STM32_NBTIM                    2   /* 2 basic timers: TIM6, TIM7 */
// TODO: there are also 6 additional timers (12-17) that don't fit any existing category
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     0   /* No I2S */
#  define STM32_NUSART                   5   /* USART1-5 */
#  define STM32_NI2C                     2   /* I2C1-2 */
#  define STM32_NCAN                     0   /* No CAN */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  0   /* No USB OTG FS/HS */
#  define STM32_NGPIO                    64  /* GPIOA-D */
#  define STM32_NADC                     1   /* ADC1 */
#  define STM32_NDAC                     2   /* DAC 1-2 */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC1 */
#  define STM32_NETHERNET                0   /* No Ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F100VC) || defined(CONFIG_ARCH_CHIP_STM32F100VD) \
 || defined(CONFIG_ARCH_CHIP_STM32F100VE)
#  undef  CONFIG_STM32_STM32L15XX            /* STM32L151xx and STM32L152xx family */
#  undef  CONFIG_STM32_ENERGYLITE            /* STM32L EnergyLite family */
#  define CONFIG_STM32_STM32F10XX        1   /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_MEDIUMPLUSDENSITY     /* STM32L15xxC w/ 32/256 Kbytes */
#  define CONFIG_STM32_HIGHDENSITY       1   /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  define CONFIG_STM32_VALUELINE         1   /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  undef  CONFIG_STM32_STM32F30XX            /* STM32F30xxx family */
#  undef  CONFIG_STM32_STM32F37XX            /* STM32F37xxx family */
#  undef  CONFIG_STM32_STM32F40XX            /* STM32F405xx and STM32407xx families */
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    1   /* One advanced timer TIM1 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM2,3,4,5 with DMA */
#  define STM32_NBTIM                    2   /* 2 basic timers: TIM6, TIM7 */
// TODO: there are also 6 additional timers (12-17) that don't fit any existing category
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     0   /* No I2S */
#  define STM32_NUSART                   5   /* USART1-5 */
#  define STM32_NI2C                     2   /* I2C1-2 */
#  define STM32_NCAN                     0   /* No CAN */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  0   /* No USB OTG FS/HS */
#  define STM32_NGPIO                    80  /* GPIOA-E */
#  define STM32_NADC                     1   /* ADC1 */
#  define STM32_NDAC                     2   /* DAC 1-2 */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC1 */
#  define STM32_NETHERNET                0   /* No Ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

/* STM32 F102x8/102xB Medium Density USB Access Family ***************************/

#elif defined(CONFIG_ARCH_CHIP_STM32F102CB)
#  undef  CONFIG_STM32_STM32L15XX            /* STM32L151xx and STM32L152xx family */
#  undef  CONFIG_STM32_ENERGYLITE            /* STM32L EnergyLite vamily */
#  define CONFIG_STM32_STM32F10XX        1   /* STM32F10xx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  define CONFIG_STM32_MEDIUMDENSITY     1   /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  define CONFIG_STM32_USBACCESSLINE     1   /* STM32F102xx */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  undef  CONFIG_STM32_STM32F30XX            /* STM32F30xxx family */
#  undef  CONFIG_STM32_STM32F40XX            /* STM32F405xx and STM32407xx families */
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    0   /* No advanced timer TIM1 */
#  define STM32_NGTIM                    3   /* 16-bit general timers TIM2,3,4 */
#  define STM32_NBTIM                    0   /* No basic timers */
#  define STM32_NDMA                     1   /* DMA */
#  define STM32_NSPI                     2   /* SPI1-2 */
#  define STM32_NI2S                     0   /* No I2S */
#  define STM32_NUSART                   3   /* USART1-3 */
#  define STM32_NI2C                     2   /* I2C1-2 */
#  define STM32_NCAN                     0   /* No CAN */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  0   /* No USB OTG FS/HS */
#  define STM32_NGPIO                    37  /* GPIOA-D */
#  define STM32_NADC                     1   /* ADC1 */
#  define STM32_NDAC                     0   /* No DAC */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC1 */
#  define STM32_NETHERNET                0   /* No ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

/* STM32 F103 Low Density Family *************************************************/

/* STM32F103C4 & STM32F103C6 */

#elif defined(CONFIG_ARCH_CHIP_STM32F103C4)
#  define CONFIG_STM32_STM32F10XX        1   /* STM32F10xxx family */
#  define CONFIG_STM32_LOWDENSITY        1   /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_MEDIUMPLUSDENSITY     /* STM32L15xxC w/ 32/256 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  undef  CONFIG_STM32_STM32F30XX            /* STM32F30xxx family */
#  undef  CONFIG_STM32_STM32F37XX            /* STM32F37xxx family */
#  undef  CONFIG_STM32_STM32F40XX            /* STM32F405xx and STM32407xx families */
#  define STM32_NFSMC                    0   /* FSMC */
#  define STM32_NATIM                    1   /* One advanced timer TIM1 */
#  define STM32_NGTIM                    2   /* General timers TIM2,3 */
#  define STM32_NBTIM                    0   /* No basic timer */
#  define STM32_NDMA                     1   /* DMA1 */
#  define STM32_NSPI                     1   /* SPI1 */
#  define STM32_NI2S                     0   /* No I2S */
#  define STM32_NUSART                   2   /* USART1-2 */
#  define STM32_NI2C                     1   /* I2C1 */
#  define STM32_NCAN                     1   /* bxCAN1 */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NUSBOTG                  0   /* No USB OTG FS/HS */
#  define STM32_NGPIO                    37  /* GPIOA-C */
#  define STM32_NADC                     2   /* ADC1-2 */
#  define STM32_NDAC                     0   /* No DAC */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NTHERNET                 0   /* No Ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

/* STM32 F103 Medium Density Performance Line ***************************************/

#elif defined(CONFIG_ARCH_CHIP_STM32F103T8) || defined(CONFIG_ARCH_CHIP_STM32F103TB)
#  undef  CONFIG_STM32_STM32L15XX            /* STM32L151xx and STM32L152xx family */
#  undef  CONFIG_STM32_ENERGYLITE            /* STM32L EnergyLite family */
#  define CONFIG_STM32_STM32F10XX        1   /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  define CONFIG_STM32_MEDIUMDENSITY     1   /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_MEDIUMPLUSDENSITY     /* STM32L15xxC w/ 32/256 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  define CONFIG_STM32_PERFORMANCELINE   1   /* STM32F103x8 and STM32F103xB */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  undef  CONFIG_STM32_STM32F30XX            /* STM32F30xxx family */
#  undef  CONFIG_STM32_STM32F37XX            /* STM32F37xxx family */
#  undef  CONFIG_STM32_STM32F40XX            /* STM32F405xx and STM32407xx families */
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    1   /* One advanced timer TIM1 */
#  define STM32_NGTIM                    3   /* General timers TIM2,3,4 */
#  define STM32_NBTIM                    0   /* No basic timers */
#  define STM32_NDMA                     1   /* DMA1, 7 channels */
#  define STM32_NSPI                     1   /* SPI1 */
#  define STM32_NI2S                     0   /* No I2S */
#  define STM32_NUSART                   2   /* USART1-2 */
#  define STM32_NI2C                     1   /* I2C1 */
#  define STM32_NCAN                     1   /* bxCAN1 */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  0   /* No USB OTG FS/HS */
#  define STM32_NGPIO                    26  /* GPIOA-E */
#  define STM32_NADC                     2   /* ADC1-2 */
#  define STM32_NDAC                     0   /* No DAC */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NTHERNET                 0   /* No Ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F103C8) || defined(CONFIG_ARCH_CHIP_STM32F103CB)
#  undef  CONFIG_STM32_STM32L15XX            /* STM32L151xx and STM32L152xx family */
#  undef  CONFIG_STM32_ENERGYLITE            /* STM32L EnergyLite family */
#  define CONFIG_STM32_STM32F10XX        1   /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  define CONFIG_STM32_MEDIUMDENSITY     1   /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_MEDIUMPLUSDENSITY     /* STM32L15xxC w/ 32/256 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  define CONFIG_STM32_PERFORMANCELINE   1   /* STM32F103x8 and STM32F103xB */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  undef  CONFIG_STM32_STM32F30XX            /* STM32F30xxx family */
#  undef  CONFIG_STM32_STM32F37XX            /* STM32F37xxx family */
#  undef  CONFIG_STM32_STM32F40XX            /* STM32F405xx and STM32407xx families */
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    1   /* One advanced timer TIM1 */
#  define STM32_NGTIM                    3   /* General timers TIM2,3,4 */
#  define STM32_NBTIM                    0   /* No basic timers */
#  define STM32_NDMA                     1   /* DMA1, 7 channels */
#  define STM32_NSPI                     2   /* SPI1-2 */
#  define STM32_NI2S                     0   /* No I2S */
#  define STM32_NUSART                   3   /* USART1-3 */
#  define STM32_NI2C                     2   /* I2C1-2 */
#  define STM32_NCAN                     1   /* bxCAN1 */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  0   /* No USB OTG FS/HS */
#  define STM32_NGPIO                    37  /* GPIOA-C */
#  define STM32_NADC                     2   /* ADC1-2 */
#  define STM32_NDAC                     0   /* No DAC */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NTHERNET                 0   /* No Ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F103R8) || defined(CONFIG_ARCH_CHIP_STM32F103RB)
#  undef  CONFIG_STM32_STM32L15XX            /* STM32L151xx and STM32L152xx family */
#  undef  CONFIG_STM32_ENERGYLITE            /* STM32L EnergyLite family */
#  define CONFIG_STM32_STM32F10XX        1   /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  define CONFIG_STM32_MEDIUMDENSITY     1   /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_MEDIUMPLUSDENSITY     /* STM32L15xxC w/ 32/256 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  define CONFIG_STM32_PERFORMANCELINE  1    /* STM32F103x8 and STM32F103xB */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  undef  CONFIG_STM32_STM32F30XX            /* STM32F30xxx family */
#  undef  CONFIG_STM32_STM32F37XX            /* STM32F37xxx family */
#  undef  CONFIG_STM32_STM32F40XX            /* STM32F405xx and STM32407xx families */
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    1   /* One advanced timer TIM1 */
#  define STM32_NGTIM                    3   /* General timers TIM2,3,4 */
#  define STM32_NBTIM                    0   /* No basic timers */
#  define STM32_NDMA                     1   /* DMA1, 7 channels */
#  define STM32_NSPI                     2   /* SPI1-2 */
#  define STM32_NI2S                     0   /* No I2S */
#  define STM32_NUSART                   3   /* USART1-3 */
#  define STM32_NI2C                     2   /* I2C1-2 */
#  define STM32_NCAN                     1   /* bxCAN1 */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  0   /* No USB OTG FS/HS */
#  define STM32_NGPIO                    51  /* GPIOA-E */
#  define STM32_NADC                     2   /* ADC1-2 */
#  define STM32_NDAC                     0   /* No DAC */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NTHERNET                 0   /* No Ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

/* STM32 F103 High Density Family ***************************************************/
/* STM32F103RC, STM32F103RD, and STM32F103RE are all provided in 64 pin packages and
 * differ only in the available FLASH and SRAM.
 */

#elif defined(CONFIG_ARCH_CHIP_STM32F103RC) || defined(CONFIG_ARCH_CHIP_STM32F103RD) || \
      defined(CONFIG_ARCH_CHIP_STM32F103RE) || defined(CONFIG_ARCH_CHIP_STM32F103RG)
#  undef  CONFIG_STM32_STM32L15XX            /* STM32L151xx and STM32L152xx family */
#  undef  CONFIG_STM32_ENERGYLITE            /* STM32L EnergyLite family */
#  define CONFIG_STM32_STM32F10XX        1   /* STM32F10xxx family */
#  undef CONFIG_STM32_LOWDENSITY             /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_MEDIUMPLUSDENSITY     /* STM32L15xxC w/ 32/256 Kbytes */
#  define CONFIG_STM32_HIGHDENSITY       1   /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  undef  CONFIG_STM32_STM32F30XX            /* STM32F30xxx family */
#  undef  CONFIG_STM32_STM32F37XX            /* STM32F37xxx family */
#  undef  CONFIG_STM32_STM32F40XX            /* STM32F405xx and STM32407xx */
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    2   /* Two advanced timers TIM1 and TIM8 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM2,3,4,5 with DMA */
#  define STM32_NBTIM                    2   /* Two basic timers TIM6 and TIM7 */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     0   /* No I2S (?) */
#  define STM32_NUSART                   5   /* USART1-5 */
#  define STM32_NI2C                     2   /* I2C1-2 */
#  define STM32_NCAN                     1   /* CAN1 */
#  define STM32_NSDIO                    1   /* SDIO */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  0   /* No USB OTG FS/HS */
#  define STM32_NGPIO                    51  /* GPIOA-D */
#  define STM32_NADC                     2   /* ADC1-2 */
#  define STM32_NDAC                     2   /* DAC1-2 */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No Ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

/* STM32F103VC, STM32F103VD, and STM32F103VE are all provided in 100 pin packages and differ
 * only in the available FLASH and SRAM.
 */

#elif defined(CONFIG_ARCH_CHIP_STM32F103VC) || defined(CONFIG_ARCH_CHIP_STM32F103VE)
#  undef  CONFIG_STM32_STM32L15XX            /* STM32L151xx and STM32L152xx family */
#  undef  CONFIG_STM32_ENERGYLITE            /* STM32L EnergyLite family */
#  define CONFIG_STM32_STM32F10XX        1   /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_MEDIUMPLUSDENSITY     /* STM32L15xxC w/ 32/256 Kbytes */
#  define CONFIG_STM32_HIGHDENSITY       1   /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  undef  CONFIG_STM32_STM32F30XX            /* STM32F30xxx family */
#  undef  CONFIG_STM32_STM32F37XX            /* STM32F37xxx family */
#  undef  CONFIG_STM32_STM32F40XX            /* STM32F405xx and STM32407xx families */
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    2   /* Two advanced timers TIM1 and TIM8 */
#  define STM32_NGTIM                    4   /* General timers TIM2,3,4,5 */
#  define STM32_NBTIM                    2   /* Two basic timers TIM6 and TIM7 */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     0   /* No I2S (?) */
#  define STM32_NUSART                   5   /* USART1-5 */
#  define STM32_NI2C                     2   /* I2C1-2 */
#  define STM32_NCAN                     1   /* bxCAN1 */
#  define STM32_NSDIO                    1   /* SDIO */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  0   /* No USB OTG FS/HS */
#  define STM32_NGPIO                    80  /* GPIOA-E */
#  define STM32_NADC                     3   /* ADC1-3 */
#  define STM32_NDAC                     2   /* DAC1-2 */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NTHERNET                 0   /* No Ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

/* STM32F103ZC, STM32F103ZD, and STM32F103ZE are all provided in 144 pin packages and differ
 * only in the available FLASH and SRAM.
 */

#elif defined(CONFIG_ARCH_CHIP_STM32F103ZE)
#  undef  CONFIG_STM32_STM32L15XX            /* STM32L151xx and STM32L152xx family */
#  undef  CONFIG_STM32_ENERGYLITE            /* STM32L EnergyLite family */
#  define CONFIG_STM32_STM32F10XX        1   /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_MEDIUMPLUSDENSITY     /* STM32L15xxC w/ 32/256 Kbytes */
#  define CONFIG_STM32_HIGHDENSITY       1   /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  undef  CONFIG_STM32_STM32F30XX            /* STM32F30xxx family */
#  undef  CONFIG_STM32_STM32F37XX            /* STM32F37xxx family */
#  undef  CONFIG_STM32_STM32F40XX            /* STM32F405xx and STM32407xx families */
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    1   /* One advanced timer TIM1 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM2,3,4,5 with DMA */
#  define STM32_NBTIM                    0   /* No basic timers */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     0   /* No I2S (?) */
#  define STM32_NUSART                   3   /* USART1-3 */
#  define STM32_NI2C                     2   /* I2C1-2 */
#  define STM32_NCAN                     1   /* CAN1 */
#  define STM32_NSDIO                    1   /* SDIO */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  0   /* No USB OTG FS/HS */
#  define STM32_NGPIO                    112 /* GPIOA-G */
#  define STM32_NADC                     1   /* ADC1 */
#  define STM32_NDAC                     0   /* No DAC */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     0   /* No CRC */
#  define STM32_NETHERNET                0   /* No Ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

/* STM32 F105/F107 Connectivity Line *******************************************************/

#elif defined(CONFIG_ARCH_CHIP_STM32F105VB)
#  undef  CONFIG_STM32_STM32L15XX            /* STM32L151xx and STM32L152xx family */
#  undef  CONFIG_STM32_ENERGYLITE            /* STM32L EnergyLite family */
#  define CONFIG_STM32_STM32F10XX        1   /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_MEDIUMPLUSDENSITY     /* STM32L15xxC w/ 32/256 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  define CONFIG_STM32_CONNECTIVITYLINE  1   /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  undef  CONFIG_STM32_STM32F30XX            /* STM32F30xxx family */
#  undef  CONFIG_STM32_STM32F37XX            /* STM32F37xxx family */
#  undef  CONFIG_STM32_STM32F40XX            /* STM32F405xx and STM32407xx */
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    1   /* One advanced timers TIM1 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM2,3,4,5 with DMA */
#  define STM32_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     2   /* I2S1-2 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   5   /* USART1-3, UART 4-5 */
#  define STM32_NI2C                     2   /* I2C1-2 */
#  define STM32_NCAN                     2   /* CAN1-2 */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS */
#  define STM32_NGPIO                    80  /* GPIOA-E */
#  define STM32_NADC                     2   /* ADC1-2 */
#  define STM32_NDAC                     2   /* DAC1-2 */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* 100/100 Ethernet MAC */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F107VC)
#  undef  CONFIG_STM32_STM32L15XX            /* STM32L151xx and STM32L152xx family */
#  undef  CONFIG_STM32_ENERGYLITE            /* STM32L EnergyLite family */
#  define CONFIG_STM32_STM32F10XX        1   /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_MEDIUMPLUSDENSITY     /* STM32L15xxC w/ 32/256 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  define CONFIG_STM32_CONNECTIVITYLINE  1   /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  undef  CONFIG_STM32_STM32F30XX            /* STM32F30xxx family */
#  undef  CONFIG_STM32_STM32F37XX            /* STM32F37xxx family */
#  undef  CONFIG_STM32_STM32F40XX            /* STM32F405xx and STM32407xx */
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    1   /* One advanced timers TIM1 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM2,3,4,5 with DMA */
#  define STM32_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     2   /* I2S1-2 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   5   /* USART1-3, UART 4-5 */
#  define STM32_NI2C                     1   /* I2C1 */
#  define STM32_NCAN                     2   /* CAN1-2 */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  0   /* No USB OTG FS/HS */
#  define STM32_NGPIO                    80  /* GPIOA-E */
#  define STM32_NADC                     2   /* ADC1-2*/
#  define STM32_NDAC                     2   /* DAC1-2 */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                1   /* 100/100 Ethernet MAC */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

/* STM32 F2 Family ******************************************************************/

#elif defined(CONFIG_ARCH_CHIP_STM32F205RG)  /* UFBGA-176 1024Kb FLASH 128Kb SRAM */
#  undef  CONFIG_STM32_STM32L15XX            /* STM32L151xx and STM32L152xx family */
#  undef  CONFIG_STM32_ENERGYLITE            /* STM32L EnergyLite family */
#  undef  CONFIG_STM32_STM32F10XX            /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_MEDIUMPLUSDENSITY     /* STM32L15xxC w/ 32/256 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  define CONFIG_STM32_STM32F20XX        1   /* STM32F205x and STM32F207x */
#  undef  CONFIG_STM32_STM32F30XX            /* STM32F30xxx family */
#  undef  CONFIG_STM32_STM32F37XX            /* STM32F37xxx family */
#  undef  CONFIG_STM32_STM32F40XX            /* STM32F405xx and STM32407xx */
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    2   /* Two advanced timers TIM1 and 8 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define STM32_NGTIMNDMA                6   /* 16-bit general timers TIM9-14 without DMA */
#  define STM32_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     2   /* I2S1-2 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   6   /* USART1-3 and 6, UART 4-5 */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     2   /* CAN1-2 */
#  define STM32_NSDIO                    1   /* SDIO */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS */
#  define STM32_NGPIO                    51  /* GPIOA-I */
#  define STM32_NADC                     3   /* 12-bit ADC1-3, 16 channels */
#  define STM32_NDAC                     2   /* 12-bit DAC1-2 */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F207IG)  /* UFBGA-176 1024Kb FLASH 128Kb SRAM */
#  undef  CONFIG_STM32_STM32L15XX            /* STM32L151xx and STM32L152xx family */
#  undef  CONFIG_STM32_ENERGYLITE            /* STM32L EnergyLite family */
#  undef  CONFIG_STM32_STM32F10XX            /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_MEDIUMPLUSDENSITY     /* STM32L15xxC w/ 32/256 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  define CONFIG_STM32_STM32F20XX        1   /* STM32F205x and STM32F207x */
#  undef  CONFIG_STM32_STM32F30XX            /* STM32F30xxx family */
#  undef  CONFIG_STM32_STM32F37XX            /* STM32F37xxx family */
#  undef  CONFIG_STM32_STM32F40XX            /* STM32F405xx and STM32407xx */
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    2   /* Two advanced timers TIM1 and 8 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define STM32_NGTIMNDMA                6   /* 16-bit general timers TIM9-14 without DMA */
#  define STM32_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     2   /* I2S1-2 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   6   /* USART1-3 and 6, UART 4-5 */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     2   /* CAN1-2 */
#  define STM32_NSDIO                    1   /* SDIO */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS */
#  define STM32_NGPIO                    140 /* GPIOA-I */
#  define STM32_NADC                     3   /* 12-bit ADC1-3, 24 channels */
#  define STM32_NDAC                     2   /* 12-bit DAC1-2 */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                1   /* 100/100 Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    1   /* Digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F207ZE)  /* LQFP-144 512Kb FLASH 128Kb SRAM */
#  undef  CONFIG_STM32_STM32L15XX            /* STM32L151xx and STM32L152xx family */
#  undef  CONFIG_STM32_ENERGYLITE            /* STM32L EnergyLite family */
#  undef  CONFIG_STM32_STM32F10XX            /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_MEDIUMPLUSDENSITY     /* STM32L15xxC w/ 32/256 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  define CONFIG_STM32_STM32F20XX        1   /* STM32F205x and STM32F207x */
#  undef  CONFIG_STM32_STM32F30XX            /* STM32F30xxx family */
#  undef  CONFIG_STM32_STM32F37XX            /* STM32F37xxx family */
#  undef  CONFIG_STM32_STM32F40XX            /* STM32F405xx and STM32407xx */
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    2   /* Two advanced timers TIM1 and 8 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define STM32_NGTIMNDMA                6   /* 16-bit general timers TIM9-14 without DMA */
#  define STM32_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     2   /* I2S1-2 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   6   /* USART1-3 and 6, UART 4-5 */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     2   /* CAN1-2 */
#  define STM32_NSDIO                    1   /* SDIO */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS */
#  define STM32_NGPIO                    114 /* GPIOA-I */
#  define STM32_NADC                     3   /* 12-bit ADC1-3, 24 channels */
#  define STM32_NDAC                     2   /* 12-bit DAC1-2 */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                1   /* 100/100 Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    1   /* Digital camera interface (DCMI) */

/* STM23 F3 Family ******************************************************************/
/* Part Numbering: STM32Fssscfxxx
 *
 *  Where
 *     sss = 302/303 or 372/373
 *     c   = C (48pins) R (68 pins) V (100 pins)
 *     c   = K (32 pins), C (48 pins), R (68 pins), V (100 pins)
 *     f   = 6 (32KB FLASH), 8 (64KB FLASH), B (128KB FLASH), C (256KB FLASH)
 *     f   = 8 (64KB FLASH), B (128KB FLASH), C (256KB FLASH)
 *     xxx = Package, temperature range, options (ignored here)
 */

#elif defined(CONFIG_ARCH_CHIP_STM32F302K6) || defined(CONFIG_ARCH_CHIP_STM32F302K8)
#  undef  CONFIG_STM32_STM32L15XX            /* STM32L151xx and STM32L152xx family */
#  undef  CONFIG_STM32_ENERGYLITE            /* STM32L EnergyLite vamily */
#  undef  CONFIG_STM32_STM32F10XX            /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  define CONFIG_STM32_STM32F30XX        1   /* STM32F30xxx family */
#  undef  CONFIG_STM32_STM32F40XX            /* STM32F405xx and STM32407xx */
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    1   /* (1) Advanced 16-bit timers with DMA: TIM1 (no TIM8) */
#  define STM32_NGTIM                    6   /* (2) 16-bit general timers with DMA: TIM3 and TIM4
                                              * (1) 32-bit general timers with DMA: TIM2
                                              * (3) 16-bit general timers count-up timers with DMA: TIM15-17 */
#  define STM32_NGTIMNDMA                0   /* All timers have DMA */

#  define STM32_NBTIM                    1   /* (1) Basic timers: TIM6 (no TIM7) */
#  define STM32_NDMA                     1   /* (1) DMA1 (7 channels) */
#  define STM32_NSPI                     2   /* (3) SPI1-3 */
#  define STM32_NI2S                     0   /* (0) No I2S */
#  define STM32_NUSART                   2   /* (2) USART1-2, no UARTs */
#  define STM32_NI2C                     3   /* (3) I2C1-3 */
#  define STM32_NCAN                     1   /* (1) CAN1 */
#  define STM32_NSDIO                    0   /* (0) No SDIO */
#  define STM32_NLCD                     0   /* (0) No LCD */
#  define STM32_NUSBOTG                  0   /* USB FS device, but no USB OTG FS/HS */
#  define STM32_NGPIO                    24  /* GPIOA-F */
#  define STM32_NADC                     1   /* (1) 12-bit ADC1 */
#  define STM32_NDAC                     1   /* (1) 12-bit DAC1 */
#  define STM32_NCAPSENSE                0   /* (0) No capacitive sensing channels */
#  define STM32_NCRC                     1   /* (1) CRC calculation unit */
#  define STM32_NETHERNET                0   /* (0) No Ethernet MAC */
#  define STM32_NRNG                     0   /* (0) No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* (0) No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F302CB) || defined(CONFIG_ARCH_CHIP_STM32F302CC)
#  undef  CONFIG_STM32_STM32L15XX            /* STM32L151xx and STM32L152xx family */
#  undef  CONFIG_STM32_ENERGYLITE            /* STM32L EnergyLite family */
#  undef  CONFIG_STM32_STM32F10XX            /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_MEDIUMPLUSDENSITY     /* STM32L15xxC w/ 32/256 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  define CONFIG_STM32_STM32F30XX        1   /* STM32F30xxx family */
#  undef  CONFIG_STM32_STM32F37XX            /* STM32F37xxx family */
#  undef  CONFIG_STM32_STM32F40XX            /* STM32F405xx and STM32407xx */
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    1   /* (1) Advanced 16-bit timers with DMA: TIM1 (no TIM8) */
#  define STM32_NGTIM                    6   /* (2) 16-bit general timers with DMA: TIM3 and TIM4
                                              * (1) 32-bit general timers with DMA: TIM2
                                              * (3) 16-bit general timers count-up timers with DMA: TIM15-17 */
#  define STM32_NGTIMNDMA                0   /* All timers have DMA */

#  define STM32_NBTIM                    1   /* (1) Basic timers: TIM6 (no TIM7) */
#  define STM32_NDMA                     2   /* (2) DMA1 (7 channels) and DMA2 (5 channels) */
#  define STM32_NSPI                     3   /* (3) SPI1-3 */
#  define STM32_NI2S                     0   /* (0) No I2S */
#  define STM32_NUSART                   3   /* (3) No UART1-3, no UARTs */
#  define STM32_NI2C                     2   /* (2) I2C1-2 */
#  define STM32_NCAN                     1   /* (1) CAN1 */
#  define STM32_NSDIO                    0   /* (0) No SDIO */
#  define STM32_NLCD                     0   /* (0) No LCD */
#  define STM32_NUSBOTG                  0   /* USB FS device, but no USB OTG FS/HS */
#  define STM32_NGPIO                    37  /* GPIOA-F */
#  define STM32_NADC                     2   /* (2) 12-bit ADC1-2 */
#  define STM32_NDAC                     1   /* (1) 12-bit DAC1 */
#  define STM32_NCAPSENSE                0   /* (0) No capacitive sensing channels */
#  define STM32_NCRC                     1   /* (1) CRC calculation unit */
#  define STM32_NETHERNET                0   /* (0) No Ethernet MAC */
#  define STM32_NRNG                     0   /* (0) No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* (0) No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F302RB) || defined(CONFIG_ARCH_CHIP_STM32F302RC)
#  undef  CONFIG_STM32_STM32L15XX            /* STM32L151xx and STM32L152xx family */
#  undef  CONFIG_STM32_ENERGYLITE            /* STM32L EnergyLite family */
#  undef  CONFIG_STM32_STM32F10XX            /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_MEDIUMPLUSDENSITY     /* STM32L15xxC w/ 32/256 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  define CONFIG_STM32_STM32F30XX        1   /* STM32F30xxx family */
#  undef  CONFIG_STM32_STM32F37XX            /* STM32F37xxx family */
#  undef  CONFIG_STM32_STM32F40XX            /* STM32F405xx and STM32407xx */
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    1   /* (1) Advanced 16-bit timers with DMA: TIM1 (no TIM8) */
#  define STM32_NGTIM                    6   /* (2) 16-bit general timers with DMA: TIM3 and TIM4
                                              * (1) 32-bit general timers with DMA: TIM2
                                              * (3) 16-bit general timers count-up timers with DMA: TIM15-17 */
#  define STM32_NGTIMNDMA                0   /* All timers have DMA */

#  define STM32_NBTIM                    1   /* (1) Basic timers: TIM6 (no TIM7) */
#  define STM32_NDMA                     2   /* (2) DMA1 (7 channels) and DMA2 (5 channels) */
#  define STM32_NSPI                     3   /* (3) SPI1-3 */
#  define STM32_NI2S                     0   /* (0) No I2S */
#  define STM32_NUSART                   5   /* (5) USART1-3, UART4-5 */
#  define STM32_NI2C                     2   /* (2) I2C1-2 */
#  define STM32_NCAN                     1   /* (1) CAN1 */
#  define STM32_NSDIO                    0   /* (0) No SDIO */
#  define STM32_NLCD                     0   /* (0) No LCD */
#  define STM32_NUSBOTG                  0   /* USB FS device, but no USB OTG FS/HS */
#  define STM32_NGPIO                    52  /* GPIOA-F */
#  define STM32_NADC                     2   /* (2) 12-bit ADC1-2 */
#  define STM32_NDAC                     1   /* (1) 12-bit DAC1 */
#  define STM32_NCAPSENSE                0   /* (0) No capacitive sensing channels */
#  define STM32_NCRC                     1   /* (1) CRC calculation unit */
#  define STM32_NETHERNET                0   /* (0) No Ethernet MAC */
#  define STM32_NRNG                     0   /* (0) No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* (0) No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F302VB) || defined(CONFIG_ARCH_CHIP_STM32F302VC)
#  undef  CONFIG_STM32_STM32L15XX            /* STM32L151xx and STM32L152xx family */
#  undef  CONFIG_STM32_ENERGYLITE            /* STM32L EnergyLite family */
#  undef  CONFIG_STM32_STM32F10XX            /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_MEDIUMPLUSDENSITY     /* STM32L15xxC w/ 32/256 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  define CONFIG_STM32_STM32F30XX        1   /* STM32F30xxx family */
#  undef  CONFIG_STM32_STM32F37XX            /* STM32F37xxx family */
#  undef  CONFIG_STM32_STM32F40XX            /* STM32F405xx and STM32407xx */
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    1   /* (1) Advanced 16-bit timers with DMA: TIM1 (no TIM8) */
#  define STM32_NGTIM                    6   /* (2) 16-bit general timers with DMA: TIM3 and TIM4
                                              * (1) 32-bit general timers with DMA: TIM2
                                              * (3) 16-bit general timers count-up timers with DMA: TIM15-17 */
#  define STM32_NGTIMNDMA                0   /* All timers have DMA */

#  define STM32_NBTIM                    1   /* (1) Basic timers: TIM6 (no TIM7) */
#  define STM32_NDMA                     2   /* (2) DMA1 (7 channels) and DMA2 (5 channels) */
#  define STM32_NSPI                     3   /* (3) SPI1-3 */
#  define STM32_NI2S                     0   /* (0) No I2S */
#  define STM32_NUSART                   5   /* (5) USART1-3, UART4-5 */
#  define STM32_NI2C                     2   /* (2) I2C1-2 */
#  define STM32_NCAN                     1   /* (1) CAN1 */
#  define STM32_NSDIO                    0   /* (0) No SDIO */
#  define STM32_NLCD                     0   /* (0) No LCD */
#  define STM32_NUSBOTG                  0   /* USB FS device, but no USB OTG FS/HS */
#  define STM32_NGPIO                    87  /* GPIOA-F */
#  define STM32_NADC                     2   /* (2) 12-bit ADC1-2 */
#  define STM32_NDAC                     1   /* (1) 12-bit DAC1 */
#  define STM32_NCAPSENSE                0   /* (0) No capacitive sensing channels */
#  define STM32_NCRC                     1   /* (1) CRC calculation unit */
#  define STM32_NETHERNET                0   /* (0) No Ethernet MAC */
#  define STM32_NRNG                     0   /* (0) No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* (0) No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F303CB) || defined(CONFIG_ARCH_CHIP_STM32F303CC)
#  undef  CONFIG_STM32_STM32L15XX            /* STM32L151xx and STM32L152xx family */
#  undef  CONFIG_STM32_ENERGYLITE            /* STM32L EnergyLite family */
#  undef  CONFIG_STM32_STM32F10XX            /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_MEDIUMPLUSDENSITY     /* STM32L15xxC w/ 32/256 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  define CONFIG_STM32_STM32F30XX        1   /* STM32F30xxx family */
#  undef  CONFIG_STM32_STM32F37XX            /* STM32F37xxx family */
#  undef  CONFIG_STM32_STM32F40XX            /* STM32F405xx and STM32407xx */
#  define STM32_NFSMC                    0   /* No FSMC */

#  define STM32_NATIM                    2   /* (2) Advanced 16-bit timers with DMA: TIM1 and TIM8 */
#  define STM32_NGTIM                    6   /* (2) 16-bit general timers with DMA: TIM3 and TIM4
                                              * (1) 32-bit general timers with DMA: TIM2
                                              * (3) 16-bit general timers count-up timers with DMA: TIM15-17 */
#  define STM32_NGTIMNDMA                0   /* All timers have DMA */
#  define STM32_NBTIM                    2   /* (2) Basic timers: TIM6 and TIM7 */
#  define STM32_NDMA                     2   /* (2) DMA1 (7 channels) and DMA2 (5 channels) */
#  define STM32_NSPI                     3   /* (3) SPI1-3 */
#  define STM32_NI2S                     2   /* (2) I2S1-2 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   3   /* (3) No UART1-3, no UARTs */
#  define STM32_NI2C                     2   /* (2) I2C1-2 */
#  define STM32_NCAN                     1   /* (1) CAN1 */
#  define STM32_NSDIO                    0   /* (0) No SDIO */
#  define STM32_NLCD                     0   /* (0) No LCD */
#  define STM32_NUSBOTG                  0   /* USB FS device, but no USB OTG FS/HS */
#  define STM32_NGPIO                    37  /* GPIOA-F */
#  define STM32_NADC                     3   /* (3) 12-bit ADC1-3 */
#  define STM32_NDAC                     2   /* (2) 12-bit DAC1-2 */
#  define STM32_NCAPSENSE                0   /* (0) No capacitive sensing channels */
#  define STM32_NCRC                     1   /* (1) CRC calculation unit */
#  define STM32_NETHERNET                0   /* (0) No Ethernet MAC */
#  define STM32_NRNG                     0   /* (0) No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* (0) No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F303RB) || defined(CONFIG_ARCH_CHIP_STM32F303RC)
#  undef  CONFIG_STM32_STM32L15XX            /* STM32L151xx and STM32L152xx family */
#  undef  CONFIG_STM32_ENERGYLITE            /* STM32L EnergyLite family */
#  undef  CONFIG_STM32_STM32F10XX            /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_MEDIUMPLUSDENSITY     /* STM32L15xxC w/ 32/256 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  define CONFIG_STM32_STM32F30XX        1   /* STM32F30xxx family */
#  undef  CONFIG_STM32_STM32F37XX            /* STM32F37xxx family */
#  undef  CONFIG_STM32_STM32F40XX            /* STM32F405xx and STM32407xx */
#  define STM32_NFSMC                    0   /* No FSMC */

#  define STM32_NATIM                    2   /* (2) Advanced 16-bit timers with DMA: TIM1 and TIM8 */
#  define STM32_NGTIM                    6   /* (2) 16-bit general timers with DMA: TIM3 and TIM4
                                              * (1) 32-bit general timers with DMA: TIM2
                                              * (3) 16-bit general timers count-up timers with DMA: TIM15-17 */
#  define STM32_NGTIMNDMA                0   /* All timers have DMA */
#  define STM32_NBTIM                    2   /* (2) Basic timers: TIM6 and TIM7 */
#  define STM32_NDMA                     2   /* (2) DMA1 (7 channels) and DMA2 (5 channels) */
#  define STM32_NSPI                     3   /* (3) SPI1-3 */
#  define STM32_NI2S                     2   /* (2) I2S1-2 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   5   /* (5) USART1-3, UART4-5 */
#  define STM32_NI2C                     2   /* (2) I2C1-2 */
#  define STM32_NCAN                     1   /* (1) CAN1 */
#  define STM32_NSDIO                    0   /* (0) No SDIO */
#  define STM32_NLCD                     0   /* (0) No LCD */
#  define STM32_NUSBOTG                  0   /* USB FS device, but no USB OTG FS/HS */
#  define STM32_NGPIO                    52  /* GPIOA-F */
#  define STM32_NADC                     3   /* (3) 12-bit ADC1-3 */
#  define STM32_NDAC                     2   /* (2) 12-bit DAC1-2 */
#  define STM32_NCAPSENSE                0   /* (0) No capacitive sensing channels */
#  define STM32_NCRC                     1   /* (1) CRC calculation unit */
#  define STM32_NETHERNET                0   /* (0) No Ethernet MAC */
#  define STM32_NRNG                     0   /* (0) No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* (0) No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F303VB) || defined(CONFIG_ARCH_CHIP_STM32F303VC)
#  undef  CONFIG_STM32_STM32L15XX            /* STM32L151xx and STM32L152xx family */
#  undef  CONFIG_STM32_ENERGYLITE            /* STM32L EnergyLite family */
#  undef  CONFIG_STM32_STM32F10XX            /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_MEDIUMPLUSDENSITY     /* STM32L15xxC w/ 32/256 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  define CONFIG_STM32_STM32F30XX        1   /* STM32F30xxx family */
#  undef  CONFIG_STM32_STM32F37XX            /* STM32F37xxx family */
#  undef  CONFIG_STM32_STM32F40XX            /* STM32F405xx and STM32407xx */
#  define STM32_NFSMC                    0   /* No FSMC */

#  define STM32_NATIM                    2   /* (2) Advanced 16-bit timers with DMA: TIM1 and TIM8 */
#  define STM32_NGTIM                    6   /* (2) 16-bit general timers with DMA: TIM3 and TIM4
                                              * (1) 32-bit general timers with DMA: TIM2
                                              * (3) 16-bit general timers count-up timers with DMA: TIM15-17 */
#  define STM32_NGTIMNDMA                0   /* All timers have DMA */
#  define STM32_NBTIM                    2   /* (2) Basic timers: TIM6 and TIM7 */
#  define STM32_NDMA                     2   /* (2) DMA1 (7 channels) and DMA2 (5 channels) */
#  define STM32_NSPI                     3   /* (3) SPI1-3 */
#  define STM32_NI2S                     2   /* (2) I2S1-2 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   5   /* (5) USART1-3, UART4-5 */
#  define STM32_NI2C                     2   /* (2) I2C1-2 */
#  define STM32_NCAN                     1   /* (1) CAN1 */
#  define STM32_NSDIO                    0   /* (0) No SDIO */
#  define STM32_NLCD                     0   /* (0) No LCD */
#  define STM32_NUSBOTG                  0   /* USB FS device, but no USB OTG FS/HS */
#  define STM32_NGPIO                    87  /* GPIOA-F */
#  define STM32_NADC                     3   /* (3) 12-bit ADC1-3 */
#  define STM32_NDAC                     2   /* (2) 12-bit DAC1-2 */
#  define STM32_NCAPSENSE                0   /* (0) No capacitive sensing channels */
#  define STM32_NCRC                     1   /* (1) CRC calculation unit */
#  define STM32_NETHERNET                0   /* (0) No Ethernet MAC */
#  define STM32_NRNG                     0   /* (0) No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* (0) No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F373C8) || defined(CONFIG_ARCH_CHIP_STM32F373CB) || defined(CONFIG_ARCH_CHIP_STM32F373CC)
#  undef  CONFIG_STM32_STM32L15XX            /* STM32L151xx and STM32L152xx family */
#  undef  CONFIG_STM32_ENERGYLITE            /* STM32L EnergyLite vamily */
#  undef  CONFIG_STM32_STM32F10XX            /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  undef  CONFIG_STM32_STM32F30XX            /* STM32F30xxx family */
#  define CONFIG_STM32_STM32F37XX        1   /* STM32F37xxx family */
#  undef  CONFIG_STM32_STM32F40XX            /* STM32F405xx and STM32407xx */
#  define STM32_NFSMC                    0   /* No FSMC */

#  define STM32_NATIM                    0   /* (0) Advanced 16-bit timers with DMA: */
#  define STM32_NGTIM                    8   /* (3) 16-bit general timers with DMA: TIM3, TIM4 and TIM19
                                              * (2) 32-bit general timers with DMA: TIM2 and TIM5
                                              * (3) 16-bit general timers count-up timers with DMA: TIM15-17 */
#  define STM32_NGTIMNDMA                3   /* (3) 16-bit general timers count-up timers without DMA: TIM12-14 */
#  define STM32_NBTIM                    3   /* (3) Basic timers: TIM6, TIM7 and TIM18 */
#  define STM32_NDMA                     2   /* (2) DMA1 (7 channels) and DMA2 (5 channels) */
#  define STM32_NSPI                     3   /* (3) SPI1-3 */
#  define STM32_NI2S                     3   /* (3) I2S1-2 (multiplexed with SPI1-3) */
#  define STM32_NUSART                   3   /* (3) USART1-3 */
#  define STM32_NI2C                     2   /* (2) I2C1-2 */
#  define STM32_NCAN                     1   /* (1) CAN1 */
#  define STM32_NSDIO                    0   /* (0) No SDIO */
#  define STM32_NLCD                     0   /* (0) No LCD */
#  define STM32_NUSBOTG                  0   /* USB FS device, but no USB OTG FS/HS */
#  define STM32_NGPIO                    87  /* GPIOA-F */
#  define STM32_NADC                     1   /* (3) 12-bit ADC1 */
#  define STM32_NDAC                     2   /* (2) 12-bit DAC1-2 */
#  define STM32_NCAPSENSE                0   /* (0) No capacitive sensing channels */
#  define STM32_NCRC                     1   /* (1) CRC calculation unit */
#  define STM32_NETHERNET                0   /* (0) No Ethernet MAC */
#  define STM32_NRNG                     0   /* (0) No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* (0) No digital camera interface (DCMI) */



/* STM23 F4 Family ******************************************************************/

#elif defined(CONFIG_ARCH_CHIP_STM32F401RE)  /* LQFP64 package, 512Kb FLASH, 96KiB SRAM */
#  undef  CONFIG_STM32_STM32L15XX            /* STM32L151xx and STM32L152xx family */
#  undef  CONFIG_STM32_ENERGYLITE            /* STM32L EnergyLite family */
#  undef  CONFIG_STM32_STM32F10XX            /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_MEDIUMPLUSDENSITY     /* STM32L15xxC w/ 32/256 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  undef  CONFIG_STM32_STM32F30XX            /* STM32F30xxx family */
#  undef  CONFIG_STM32_STM32F37XX            /* STM32F37xxx family */
#  define CONFIG_STM32_STM32F40XX        1   /* STM32F405xx and STM32407xx */
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    1   /* One advanced timers TIM1 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define STM32_NGTIMNDMA                3   /* 16-bit general timers TIM9-11 without DMA */
#  define STM32_NBTIM                    0   /* No basic timers */
#  define STM32_NDMA                     2   /* DMA1-2 with 8 streams each*/
#  define STM32_NSPI                     4   /* SPI1-4 */
#  define STM32_NI2S                     2   /* I2S1-2 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   6   /* Actually only 3: USART1, 2 and 6 */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     0   /* No CAN */
#  define STM32_NSDIO                    1   /* One SDIO interface */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  1   /* USB OTG FS (only) */
#  define STM32_NGPIO                    50  /* GPIOA-H */
#  define STM32_NADC                     1   /* One 12-bit ADC1, 16 channels */
#  define STM32_NDAC                     0   /* No DAC */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* No CRC */
#  define STM32_NETHERNET                0   /* No Ethernet MAC */
#  define STM32_NRNG                     0   /* No Random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F411RE)  /* LQFP64 package, 512Kb FLASH, 128KiB SRAM */
#  undef  CONFIG_STM32_STM32L15XX            /* STM32L151xx and STM32L152xx family */
#  undef  CONFIG_STM32_ENERGYLITE            /* STM32L EnergyLite family */
#  undef  CONFIG_STM32_STM32F10XX            /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_MEDIUMPLUSDENSITY     /* STM32L15xxC w/ 32/256 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  undef  CONFIG_STM32_STM32F30XX            /* STM32F30xxx family */
#  undef  CONFIG_STM32_STM32F37XX            /* STM32F37xxx family */
#  define CONFIG_STM32_STM32F40XX        1   /* STM32F405xx and STM32407xx */
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    1   /* One advanced timers TIM1 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define STM32_NGTIMNDMA                3   /* 16-bit general timers TIM9-11 without DMA */
#  define STM32_NBTIM                    0   /* No basic timers */
#  define STM32_NDMA                     2   /* DMA1-2 with 8 streams each*/
#  define STM32_NSPI                     4   /* SPI1-4 */
#  define STM32_NI2S                     2   /* I2S1-2 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   6   /* Actually only 3: USART1, 2 and 6 */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     0   /* No CAN */
#  define STM32_NSDIO                    1   /* One SDIO interface */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  1   /* USB OTG FS (only) */
#  define STM32_NGPIO                    50  /* GPIOA-H */
#  define STM32_NADC                     1   /* One 12-bit ADC1, 16 channels */
#  define STM32_NDAC                     0   /* No DAC */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* No CRC */
#  define STM32_NETHERNET                0   /* No Ethernet MAC */
#  define STM32_NRNG                     0   /* No Random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F405RG)  /* LQFP 64 10x10x1.4 1024Kb FLASH 192Kb SRAM */
#  undef  CONFIG_STM32_STM32L15XX            /* STM32L151xx and STM32L152xx family */
#  undef  CONFIG_STM32_ENERGYLITE            /* STM32L EnergyLite family */
#  undef  CONFIG_STM32_STM32F10XX            /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_MEDIUMPLUSDENSITY     /* STM32L15xxC w/ 32/256 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  undef  CONFIG_STM32_STM32F30XX            /* STM32F30xxx family */
#  undef  CONFIG_STM32_STM32F37XX            /* STM32F37xxx family */
#  define CONFIG_STM32_STM32F40XX        1   /* STM32F405xx and STM32407xx */
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    2   /* Two advanced timers TIM1 and 8 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define STM32_NGTIMNDMA                6   /* 16-bit general timers TIM9-14 without DMA */
#  define STM32_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     2   /* I2S1-2 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   6   /* USART1-3 and 6, UART 4-5 */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     2   /* CAN1-2 */
#  define STM32_NSDIO                    1   /* SDIO */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS */
#  define STM32_NGPIO                    139 /* GPIOA-I */
#  define STM32_NADC                     3   /* 12-bit ADC1-3, 16 channels */
#  define STM32_NDAC                     2   /* 12-bit DAC1-2 */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F405VG)  /* LQFP 100 14x14x1.4  1024Kb FLASH 192Kb SRAM */
#  undef  CONFIG_STM32_STM32L15XX            /* STM32L151xx and STM32L152xx family */
#  undef  CONFIG_STM32_ENERGYLITE            /* STM32L EnergyLite family */
#  undef  CONFIG_STM32_STM32F10XX            /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_MEDIUMPLUSDENSITY     /* STM32L15xxC w/ 32/256 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  undef  CONFIG_STM32_STM32F30XX            /* STM32F30xxx family */
#  undef  CONFIG_STM32_STM32F37XX            /* STM32F37xxx family */
#  define CONFIG_STM32_STM32F40XX        1   /* STM32F405xx and STM32407xx */
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    2   /* Two advanced timers TIM1 and 8 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define STM32_NGTIMNDMA                6   /* 16-bit general timers TIM9-14 without DMA */
#  define STM32_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     2   /* I2S1-2 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   6   /* USART1-3 and 6, UART 4-5 */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     2   /* CAN1-2 */
#  define STM32_NSDIO                    1   /* SDIO */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS */
#  define STM32_NGPIO                    139 /* GPIOA-I */
#  define STM32_NADC                     3   /* 12-bit ADC1-3, 16 channels */
#  define STM32_NDAC                     2   /* 12-bit DAC1-2 */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F405ZG)  /* LQFP 144 20x20x1.4 1024Kb FLASH 192Kb SRAM */
#  undef  CONFIG_STM32_STM32L15XX            /* STM32L151xx and STM32L152xx family */
#  undef  CONFIG_STM32_ENERGYLITE            /* STM32L EnergyLite family */
#  undef  CONFIG_STM32_STM32F10XX            /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_MEDIUMPLUSDENSITY     /* STM32L15xxC w/ 32/256 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  undef  CONFIG_STM32_STM32F30XX            /* STM32F30xxx family */
#  undef  CONFIG_STM32_STM32F37XX            /* STM32F37xxx family */
#  define CONFIG_STM32_STM32F40XX        1   /* STM32F405xx and STM32407xx */
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    2   /* Two advanced timers TIM1 and 8 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define STM32_NGTIMNDMA                6   /* 16-bit general timers TIM9-14 without DMA */
#  define STM32_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     2   /* I2S1-2 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   6   /* USART1-3 and 6, UART 4-5 */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     2   /* CAN1-2 */
#  define STM32_NSDIO                    1   /* SDIO */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS */
#  define STM32_NGPIO                    139 /* GPIOA-I */
#  define STM32_NADC                     3   /* 12-bit ADC1-3, 24 channels */
#  define STM32_NDAC                     2   /* 12-bit DAC1-2 */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F407VE)  /* LQFP-100 512Kb FLASH 192Kb SRAM */
#  undef  CONFIG_STM32_STM32L15XX            /* STM32L151xx and STM32L152xx family */
#  undef  CONFIG_STM32_ENERGYLITE            /* STM32L EnergyLite family */
#  undef  CONFIG_STM32_STM32F10XX            /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_MEDIUMPLUSDENSITY     /* STM32L15xxC w/ 32/256 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  undef  CONFIG_STM32_STM32F30XX            /* STM32F30xxx family */
#  undef  CONFIG_STM32_STM32F37XX            /* STM32F37xxx family */
#  define CONFIG_STM32_STM32F40XX        1   /* STM32F405xx and STM32407xx */
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    2   /* Two advanced timers TIM1 and 8 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define STM32_NGTIMNDMA                6   /* 16-bit general timers TIM9-14 without DMA */
#  define STM32_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     2   /* I2S1-2 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   6   /* USART1-3 and 6, UART 4-5 */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     2   /* CAN1-2 */
#  define STM32_NSDIO                    1   /* SDIO */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS */
#  define STM32_NGPIO                    139 /* GPIOA-I */
#  define STM32_NADC                     3   /* 12-bit ADC1-3, 16 channels */
#  define STM32_NDAC                     2   /* 12-bit DAC1-2 */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                1   /* 100/100 Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    1   /* Digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F407VG)  /* LQFP-100 14x14x1.4 1024Kb FLASH 192Kb SRAM */
#  undef  CONFIG_STM32_STM32L15XX            /* STM32L151xx and STM32L152xx family */
#  undef  CONFIG_STM32_ENERGYLITE            /* STM32L EnergyLite family */
#  undef  CONFIG_STM32_STM32F10XX            /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_MEDIUMPLUSDENSITY     /* STM32L15xxC w/ 32/256 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  undef  CONFIG_STM32_STM32F30XX            /* STM32F30xxx family */
#  undef  CONFIG_STM32_STM32F37XX            /* STM32F37xxx family */
#  define CONFIG_STM32_STM32F40XX        1   /* STM32F405xx and STM32407xx */
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    2   /* Two advanced timers TIM1 and 8 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define STM32_NGTIMNDMA                6   /* 16-bit general timers TIM9-14 without DMA */
#  define STM32_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     2   /* I2S1-2 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   6   /* USART1-3 and 6, UART 4-5 */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     2   /* CAN1-2 */
#  define STM32_NSDIO                    1   /* SDIO */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS */
#  define STM32_NGPIO                    139 /* GPIOA-I */
#  define STM32_NADC                     3   /* 12-bit ADC1-3, 16 channels */
#  define STM32_NDAC                     2   /* 12-bit DAC1-2 */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                1   /* 100/100 Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    1   /* Digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F407ZE)  /* LQFP-144 512Kb FLASH 192Kb SRAM */
#  undef  CONFIG_STM32_STM32L15XX            /* STM32L151xx and STM32L152xx family */
#  undef  CONFIG_STM32_ENERGYLITE            /* STM32L EnergyLite family */
#  undef  CONFIG_STM32_STM32F10XX            /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_MEDIUMPLUSDENSITY     /* STM32L15xxC w/ 32/256 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  undef  CONFIG_STM32_STM32F30XX            /* STM32F30xxx family */
#  undef  CONFIG_STM32_STM32F37XX            /* STM32F37xxx family */
#  define CONFIG_STM32_STM32F40XX        1   /* STM32F405xx and STM32407xx */
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    2   /* Two advanced timers TIM1 and 8 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define STM32_NGTIMNDMA                6   /* 16-bit general timers TIM9-14 without DMA */
#  define STM32_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     2   /* I2S1-2 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   6   /* USART1-3 and 6, UART 4-5 */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     2   /* CAN1-2 */
#  define STM32_NSDIO                    1   /* SDIO */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS */
#  define STM32_NGPIO                    139 /* GPIOA-I */
#  define STM32_NADC                     3   /* 12-bit ADC1-3, 24 channels */
#  define STM32_NDAC                     2   /* 12-bit DAC1-2 */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                1   /* 100/100 Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    1   /* Digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F407ZG)  /* LQFP 144 20x20x1.4 1024Kb FLASH 192Kb SRAM */
#  undef  CONFIG_STM32_STM32L15XX            /* STM32L151xx and STM32L152xx family */
#  undef  CONFIG_STM32_ENERGYLITE            /* STM32L EnergyLite family */
#  undef  CONFIG_STM32_STM32F10XX            /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_MEDIUMPLUSDENSITY     /* STM32L15xxC w/ 32/256 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  undef  CONFIG_STM32_STM32F30XX            /* STM32F30xxx family */
#  undef  CONFIG_STM32_STM32F37XX            /* STM32F37xxx family */
#  define CONFIG_STM32_STM32F40XX        1   /* STM32F405xx and STM32407xx */
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    2   /* Two advanced timers TIM1 and 8 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define STM32_NGTIMNDMA                6   /* 16-bit general timers TIM9-14 without DMA */
#  define STM32_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     2   /* I2S1-2 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   6   /* USART1-3 and 6, UART 4-5 */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     2   /* CAN1-2 */
#  define STM32_NSDIO                    1   /* SDIO */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS */
#  define STM32_NGPIO                    139 /* GPIOA-I */
#  define STM32_NADC                     3   /* 12-bit ADC1-3, 24 channels */
#  define STM32_NDAC                     2   /* 12-bit DAC1-2 */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                1   /* 100/100 Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    1   /* Digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F407IE)  /* LQFP 176 24x24x1.4 512Kb FLASH 192Kb SRAM */
#  undef  CONFIG_STM32_STM32L15XX            /* STM32L151xx and STM32L152xx family */
#  undef  CONFIG_STM32_ENERGYLITE            /* STM32L EnergyLite family */
#  undef  CONFIG_STM32_STM32F10XX            /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_MEDIUMPLUSDENSITY     /* STM32L15xxC w/ 32/256 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  undef  CONFIG_STM32_STM32F30XX            /* STM32F30xxx family */
#  undef  CONFIG_STM32_STM32F37XX            /* STM32F37xxx family */
#  define CONFIG_STM32_STM32F40XX        1   /* STM32F405xx and STM32407xx */
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    2   /* Two advanced timers TIM1 and 8 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define STM32_NGTIMNDMA                6   /* 16-bit general timers TIM9-14 without DMA */
#  define STM32_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     2   /* I2S1-2 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   6   /* USART1-3 and 6, UART 4-5 (?) */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     2   /* CAN1-2 */
#  define STM32_NSDIO                    1   /* SDIO */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS */
#  define STM32_NGPIO                    139 /* GPIOA-I */
#  define STM32_NADC                     3   /* 12-bit ADC1-3, 24 channels */
#  define STM32_NDAC                     2   /* 12-bit DAC1-2 */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                1   /* 100/100 Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    1   /* Digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F407IG)  /* BGA 176; LQFP 176 24x24x1.4 1024Kb FLASH 192Kb SRAM */
#  undef  CONFIG_STM32_STM32L15XX            /* STM32L151xx and STM32L152xx family */
#  undef  CONFIG_STM32_ENERGYLITE            /* STM32L EnergyLite family */
#  undef  CONFIG_STM32_STM32F10XX            /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_MEDIUMPLUSDENSITY     /* STM32L15xxC w/ 32/256 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  undef  CONFIG_STM32_STM32F30XX            /* STM32F30xxx family */
#  undef  CONFIG_STM32_STM32F37XX            /* STM32F37xxx family */
#  define CONFIG_STM32_STM32F40XX        1   /* STM32F405xx and STM32407xx */
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    2   /* Two advanced timers TIM1 and 8 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define STM32_NGTIMNDMA                6   /* 16-bit general timers TIM9-14 without DMA */
#  define STM32_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     2   /* I2S1-2 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   6   /* USART1-3 and 6, UART 4-5 */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     2   /* CAN1-2 */
#  define STM32_NSDIO                    1   /* SDIO */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS */
#  define STM32_NGPIO                    139 /* GPIOA-I */
#  define STM32_NADC                     3   /* 12-bit ADC1-3, 24 channels */
#  define STM32_NDAC                     2   /* 12-bit DAC1-2 */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                1   /* 100/100 Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    1   /* Digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F427I)   /* BGA176; LQFP176 1024/2048KiB flash 256KiB SRAM */
#  undef  CONFIG_STM32_STM32L15XX            /* STM32L151xx and STM32L152xx family */
#  undef  CONFIG_STM32_ENERGYLITE            /* STM32L EnergyLite family */
#  undef  CONFIG_STM32_STM32F10XX            /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_MEDIUMPLUSDENSITY     /* STM32L15xxC w/ 32/256 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  undef  CONFIG_STM32_STM32F30XX            /* STM32F30xxx family */
#  undef  CONFIG_STM32_STM32F37XX            /* STM32F37xxx family */
#  define CONFIG_STM32_STM32F40XX        1   /* STM32F405xx, STM32407xx and STM32F427/437 */
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    2   /* Two advanced timers TIM1 and 8 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define STM32_NGTIMNDMA                6   /* 16-bit general timers TIM9-14 without DMA */
#  define STM32_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     6   /* SPI1-6 */
#  define STM32_NI2S                     2   /* I2S1-2 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   8   /* USART1-3 and 6, UART 4-5 and 7-8 */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     2   /* CAN1-2 */
#  define STM32_NSDIO                    1   /* SDIO */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS */
#  define STM32_NGPIO                    139 /* GPIOA-I */
#  define STM32_NADC                     3   /* 12-bit ADC1-3, 24 channels */
#  define STM32_NDAC                     2   /* 12-bit DAC1-2 */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                1   /* 100/100 Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    1   /* Digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F427Z)   /* LQFP144 1024/2048KiB flash 256KiB SRAM */
#  undef  CONFIG_STM32_STM32L15XX            /* STM32L151xx and STM32L152xx family */
#  undef  CONFIG_STM32_ENERGYLITE            /* STM32L EnergyLite family */
#  undef  CONFIG_STM32_STM32F10XX            /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_MEDIUMPLUSDENSITY     /* STM32L15xxC w/ 32/256 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  undef  CONFIG_STM32_STM32F30XX            /* STM32F30xxx family */
#  undef  CONFIG_STM32_STM32F37XX            /* STM32F37xxx family */
#  define CONFIG_STM32_STM32F40XX        1   /* STM32F405xx, STM32407xx and STM32F427/437 */
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    2   /* Two advanced timers TIM1 and 8 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define STM32_NGTIMNDMA                6   /* 16-bit general timers TIM9-14 without DMA */
#  define STM32_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     6   /* SPI1-6 */
#  define STM32_NI2S                     2   /* I2S1-2 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   8   /* USART1-3 and 6, UART 4-5 and 7-8 */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     2   /* CAN1-2 */
#  define STM32_NSDIO                    1   /* SDIO */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS */
#  define STM32_NGPIO                    139 /* GPIOA-I */
#  define STM32_NADC                     3   /* 12-bit ADC1-3, 24 channels */
#  define STM32_NDAC                     2   /* 12-bit DAC1-2 */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                1   /* 100/100 Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    1   /* Digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F427V)   /* LQFP100 1024/2048KiB flash 256KiB SRAM */
#  undef  CONFIG_STM32_STM32L15XX            /* STM32L151xx and STM32L152xx family */
#  undef  CONFIG_STM32_ENERGYLITE            /* STM32L EnergyLite family */
#  undef  CONFIG_STM32_STM32F10XX            /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_MEDIUMPLUSDENSITY     /* STM32L15xxC w/ 32/256 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  undef  CONFIG_STM32_STM32F30XX            /* STM32F30xxx family */
#  undef  CONFIG_STM32_STM32F37XX            /* STM32F37xxx family */
#  define CONFIG_STM32_STM32F40XX        1   /* STM32F405xx, STM32407xx and STM32F427/437 */
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    2   /* Two advanced timers TIM1 and 8 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define STM32_NGTIMNDMA                6   /* 16-bit general timers TIM9-14 without DMA */
#  define STM32_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     4   /* SPI1-4 */
#  define STM32_NI2S                     2   /* I2S1-2 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   8   /* USART1-3 and 6, UART 4-5 and 7-8 */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     2   /* CAN1-2 */
#  define STM32_NSDIO                    1   /* SDIO */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS */
#  define STM32_NGPIO                    139 /* GPIOA-I */
#  define STM32_NADC                     3   /* 12-bit ADC1-3, 24 channels */
#  define STM32_NDAC                     2   /* 12-bit DAC1-2 */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                1   /* 100/100 Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    1   /* Digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F429I)   /* BGA176; LQFP176 1024/2048KiB flash 256KiB SRAM */
#  undef  CONFIG_STM32_STM32L15XX            /* STM32L151xx and STM32L152xx family */
#  undef  CONFIG_STM32_ENERGYLITE            /* STM32L EnergyLite family */
#  undef  CONFIG_STM32_STM32F10XX            /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_MEDIUMPLUSDENSITY     /* STM32L15xxC w/ 32/256 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  undef  CONFIG_STM32_STM32F30XX            /* STM32F30xxx family */
#  undef  CONFIG_STM32_STM32F37XX            /* STM32F37xxx family */
#  define CONFIG_STM32_STM32F40XX        1   /* STM32F405xx, STM32407xx and STM32F427/437 */
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    2   /* Two advanced timers TIM1 and 8 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define STM32_NGTIMNDMA                6   /* 16-bit general timers TIM9-14 without DMA */
#  define STM32_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     6   /* SPI1-6 */
#  define STM32_NI2S                     2   /* I2S1-2 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   8   /* USART1-3 and 6, UART 4-5 and 7-8 */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     2   /* CAN1-2 */
#  define STM32_NSDIO                    1   /* SDIO */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS */
#  define STM32_NGPIO                    139 /* GPIOA-I */
#  define STM32_NADC                     3   /* 12-bit ADC1-3, 24 channels */
#  define STM32_NDAC                     2   /* 12-bit DAC1-2 */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                1   /* 100/100 Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    1   /* Digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F429Z)   /* LQFP144 1024/2048KiB flash 256KiB SRAM */
#  undef  CONFIG_STM32_STM32L15XX            /* STM32L151xx and STM32L152xx family */
#  undef  CONFIG_STM32_ENERGYLITE            /* STM32L EnergyLite family */
#  undef  CONFIG_STM32_STM32F10XX            /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_MEDIUMPLUSDENSITY     /* STM32L15xxC w/ 32/256 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  undef  CONFIG_STM32_STM32F30XX            /* STM32F30xxx family */
#  undef  CONFIG_STM32_STM32F37XX            /* STM32F37xxx family */
#  define CONFIG_STM32_STM32F40XX        1   /* STM32F405xx, STM32407xx and STM32F427/437/429/439 */
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    2   /* Two advanced timers TIM1 and 8 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define STM32_NGTIMNDMA                6   /* 16-bit general timers TIM9-14 without DMA */
#  define STM32_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     6   /* SPI1-6 */
#  define STM32_NI2S                     2   /* I2S1-2 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   8   /* USART1-3 and 6, UART 4-5 and 7-8 */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     2   /* CAN1-2 */
#  define STM32_NSDIO                    1   /* SDIO */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS */
#  define STM32_NGPIO                    139 /* GPIOA-I */
#  define STM32_NADC                     3   /* 12-bit ADC1-3, 24 channels */
#  define STM32_NDAC                     2   /* 12-bit DAC1-2 */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                1   /* 100/100 Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    1   /* Digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F429V)   /* LQFP100 1024/2048KiB flash 256KiB SRAM */
#  undef  CONFIG_STM32_STM32L15XX            /* STM32L151xx and STM32L152xx family */
#  undef  CONFIG_STM32_ENERGYLITE            /* STM32L EnergyLite family */
#  undef  CONFIG_STM32_STM32F10XX            /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_MEDIUMPLUSDENSITY     /* STM32L15xxC w/ 32/256 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  undef  CONFIG_STM32_STM32F30XX            /* STM32F30xxx family */
#  undef  CONFIG_STM32_STM32F37XX            /* STM32F37xxx family */
#  define CONFIG_STM32_STM32F40XX        1   /* STM32F405xx, STM32407xx and STM32F427/437 */
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    2   /* Two advanced timers TIM1 and 8 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define STM32_NGTIMNDMA                6   /* 16-bit general timers TIM9-14 without DMA */
#  define STM32_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     4   /* SPI1-4 */
#  define STM32_NI2S                     2   /* I2S1-2 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   8   /* USART1-3 and 6, UART 4-5 and 7-8 */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     2   /* CAN1-2 */
#  define STM32_NSDIO                    1   /* SDIO */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS */
#  define STM32_NGPIO                    139 /* GPIOA-I */
#  define STM32_NADC                     3   /* 12-bit ADC1-3, 24 channels */
#  define STM32_NDAC                     2   /* 12-bit DAC1-2 */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                1   /* 100/100 Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    1   /* Digital camera interface (DCMI) */

#else
#  error "Unsupported STM32 chip"
#endif

/* NVIC priority levels *************************************************************/

#define NVIC_SYSH_PRIORITY_MIN     0xf0 /* All bits set in minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT 0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX     0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP    0x10 /* Four bits of interrupt priority used */

/* If CONFIG_ARMV7M_USEBASEPRI is selected, then interrupts will be disabled
 * by setting the BASEPRI register to NVIC_SYSH_DISABLE_PRIORITY so that most
 * interrupts will not have execution priority.  SVCall must have execution
 * priority in all cases.
 *
 * In the normal cases, interrupts are not nest-able and all interrupts run
 * at an execution priority between NVIC_SYSH_PRIORITY_MIN and
 * NVIC_SYSH_PRIORITY_MAX (with NVIC_SYSH_PRIORITY_MAX reserved for SVCall).
 *
 * If, in addition, CONFIG_ARCH_HIPRI_INTERRUPT is defined, then special
 * high priority interrupts are supported.  These are not "nested" in the
 * normal sense of the word.  These high priority interrupts can interrupt
 * normal processing but execute outside of OS (although they can "get back
 * into the game" via a PendSV interrupt).
 *
 * In the normal course of things, interrupts must occasionally be disabled
 * using the irqsave() inline function to prevent contention in use of
 * resources that may be shared between interrupt level and non-interrupt
 * level logic.  Now the question arises, if CONFIG_ARCH_HIPRI_INTERRUPT,
 * do we disable all interrupts (except SVCall), or do we only disable the
 * "normal" interrupts.  Since the high priority interrupts cannot interact
 * with the OS, you may want to permit the high priority interrupts even if
 * interrupts are disabled.  The setting CONFIG_ARCH_INT_DISABLEALL can be
 * used to select either behavior:
 *
 *   ----------------------------+--------------+----------------------------
 *   CONFIG_ARCH_HIPRI_INTERRUPT |      NO      |             YES
 *   ----------------------------+--------------+--------------+-------------
 *   CONFIG_ARCH_INT_DISABLEALL  |     N/A      |     YES      |      NO
 *   ----------------------------+--------------+--------------+-------------
 *                               |              |              |    SVCall
 *                               |    SVCall    |    SVCall    |    HIGH
 *   Disable here and below --------> MAXNORMAL ---> HIGH --------> MAXNORMAL
 *                               |              |    MAXNORMAL |
 *   ----------------------------+--------------+--------------+-------------
 */

#if defined(CONFIG_ARCH_HIPRI_INTERRUPT) && defined(CONFIG_ARCH_INT_DISABLEALL)
#  define NVIC_SYSH_MAXNORMAL_PRIORITY  (NVIC_SYSH_PRIORITY_MAX + 2*NVIC_SYSH_PRIORITY_STEP)
#  define NVIC_SYSH_HIGH_PRIORITY       (NVIC_SYSH_PRIORITY_MAX + NVIC_SYSH_PRIORITY_STEP)
#  define NVIC_SYSH_DISABLE_PRIORITY    NVIC_SYSH_HIGH_PRIORITY
#  define NVIC_SYSH_SVCALL_PRIORITY     NVIC_SYSH_PRIORITY_MAX
#else
#  define NVIC_SYSH_MAXNORMAL_PRIORITY  (NVIC_SYSH_PRIORITY_MAX + NVIC_SYSH_PRIORITY_STEP)
#  define NVIC_SYSH_HIGH_PRIORITY       NVIC_SYSH_PRIORITY_MAX
#  define NVIC_SYSH_DISABLE_PRIORITY    NVIC_SYSH_MAXNORMAL_PRIORITY
#  define NVIC_SYSH_SVCALL_PRIORITY     NVIC_SYSH_PRIORITY_MAX
#endif

#endif /* __ARCH_ARM_INCLUDE_STM32_CHIP_H */
