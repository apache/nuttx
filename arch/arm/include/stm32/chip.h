/**************************************************************************************************
 * arch/arm/include/stm32/chip.h
 *
 *   Copyright (C) 2009, 2011-2014, 2017-2018 Gregory Nutt. All rights reserved.
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
 **************************************************************************************************/

#ifndef __ARCH_ARM_INCLUDE_STM32_CHIP_H
#define __ARCH_ARM_INCLUDE_STM32_CHIP_H

/**************************************************************************************************
 * Included Files
 **************************************************************************************************/

#include <nuttx/config.h>

/**************************************************************************************************
 * Pre-processor Definitions
 **************************************************************************************************/

/* Check the STM32 family configuration.
 * It must be done in arch/arm/src/stm32/Kconfig !
 */

#ifdef CONFIG_STM32_STM32F10XX
#  define __HAVE_F1  1
#else
#  define __HAVE_F1  0
#endif
#ifdef CONFIG_STM32_STM32F20XX
#  define __HAVE_F2  1
#else
#  define __HAVE_F2  0
#endif
#ifdef CONFIG_STM32_STM32F30XX
#  define __HAVE_F30 1
#else
#  define __HAVE_F30 0
#endif
#ifdef CONFIG_STM32_STM32F33XX
#  define __HAVE_F33 1
#else
#  define __HAVE_F33 0
#endif
#ifdef CONFIG_STM32_STM32F37XX
#  define __HAVE_F37 1
#else
#  define __HAVE_F37 0
#endif
#ifdef CONFIG_STM32_STM32F4XXX
#  define __HAVE_F4  1
#else
#  define __HAVE_F4  0
#endif
#ifdef CONFIG_STM32_STM32G47XX
#  define __HAVE_G47 1
#else
#  define __HAVE_G47 0
#endif
#ifdef CONFIG_STM32_STM32L15XX
#  define __HAVE_L1  1
#else
#  define __HAVE_L1  0
#endif

#if ((__HAVE_F1 + __HAVE_F2 + __HAVE_F30 + __HAVE_F33 + __HAVE_F37 + __HAVE_F4 + \
      __HAVE_G47 + __HAVE_L1) != 1)
#  error "Only one STM32 family must be selected !"
#endif

#ifdef CONFIG_STM32_LOWDENSITY
#  define __HAVE_LD  1
#else
#  define __HAVE_LD  0
#endif
#ifdef CONFIG_STM32_MEDIUMDENSITY
#  define __HAVE_MD  1
#else
#  define __HAVE_MD  0
#endif
#ifdef CONFIG_STM32_MEDIUMPLUSDENSITY
#  define __HAVE_MPD 1
#else
#  define __HAVE_MPD 0
#endif
#ifdef CONFIG_STM32_HIGHDENSITY
#  define __HAVE_HD  1
#else
#  define __HAVE_HD  0
#endif

#if (__HAVE_LD +__HAVE_MD + __HAVE_MPD + __HAVE_HD) > 1
#  error "Up to one density configuration must be selected"
#endif

/* Get customizations for each supported chip and provide alternate function
 * pin-mapping
 *
 * NOTE: Each GPIO pin may serve either for general purpose I/O or for a
 * special alternate function (such as USART, CAN, USB, SDIO, etc.).  That
 * particular pin-mapping will depend on the package and STM32 family.  If
 * you are incorporating a new STM32 chip into NuttX, you will need to add
 * the pin-mapping to a header file and to include that header file below.
 * The chip-specific pin-mapping is defined in the chip datasheet.
 */

/* STM32L EnergyLite Line *************************************************************************/

/* STM32L151XX -- No LCD
 * STM32L152XX -- With LCD
 *
 * STM32L15XCX -- 48-pins
 * STM32L15XRX -- 64-pins
 * STM32L15XVX -- 100-pins
 * STM32L15XZX -- 144-pins
 *
 * STM32L15XX6 -- 32KB FLASH, 10KB SRAM, 4KB EEPROM
 * STM32L15XX8 -- 64KB FLASH, 10KB SRAM, 4KB EEPROM
 * STM32L15XXB -- 128KB FLASH, 16KB SRAM, 4KB EEPROM
 *
 * STM32L15XXC -- 256KB FLASH, 32KB SRAM, 8KB EEPROM (medium+ density)
 *
 * STM32L16XXD -- 384KB FLASH, 48KB SRAM, 12KB EEPROM (high density)
 * STM32L16XXE -- 512KB FLASH, 80KB SRAM, 16KB EEPROM (high density)
 */

#if defined(CONFIG_ARCH_CHIP_STM32L151C6) || defined(CONFIG_ARCH_CHIP_STM32L151C8) || \
    defined(CONFIG_ARCH_CHIP_STM32L151CB)
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    0   /* No advanced timers */
#  define STM32_NGTIM                    3   /* 16-bit general up/down timers TIM2-4 with DMA */
#  define STM32_NGTIMNDMA                3   /* 16-bit general timers TIM9-11 without DMA */
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
#  define STM32_NADC                     1   /* ADC1, 14-channels */
#  define STM32_NDAC                     2   /* DAC 1, 2 channels */
#  define STM32_NCMP                     2   /* (2) Comparators */
#  define STM32_NCAPSENSE                13  /* Capacitive sensing channels */
#  define STM32_NCRC                     0   /* No CRC */
#  define STM32_NETHERNET                0   /* No Ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32L151R6) || defined(CONFIG_ARCH_CHIP_STM32L151R8) || \
      defined(CONFIG_ARCH_CHIP_STM32L151RB)
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    0   /* No advanced timers */
#  define STM32_NGTIM                    3   /* 16-bit general up/down timers TIM2-4 with DMA */
#  define STM32_NGTIMNDMA                3   /* 16-bit general timers TIM9-11 without DMA */
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
#  define STM32_NDAC                     2   /* DAC , 2 channels */
#  define STM32_NCMP                     2   /* (2) Comparators */
#  define STM32_NCAPSENSE                20  /* Capacitive sensing channels */
#  define STM32_NCRC                     0   /* No CRC */
#  define STM32_NETHERNET                0   /* No Ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32L151V6) || defined(CONFIG_ARCH_CHIP_STM32L151V8) || \
      defined(CONFIG_ARCH_CHIP_STM32L151VB)
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    0   /* No advanced timers */
#  define STM32_NGTIM                    3   /* 16-bit general up/down timers TIM2-4 with DMA */
#  define STM32_NGTIMNDMA                3   /* 16-bit general timers TIM9-11 without DMA */
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
#  define STM32_NDAC                     2   /* DAC 1, 2 channels */
#  define STM32_NCMP                     2   /* (2) Comparators */
#  define STM32_NCAPSENSE                20  /* Capacitive sensing channels */
#  define STM32_NCRC                     0   /* No CRC */
#  define STM32_NETHERNET                0   /* No Ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32L152C6) || defined(CONFIG_ARCH_CHIP_STM32L152C8) || \
      defined(CONFIG_ARCH_CHIP_STM32L152CB)
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    0   /* No advanced timers */
#  define STM32_NGTIM                    3   /* 16-bit general up/down timers TIM2-4 with DMA */
#  define STM32_NGTIMNDMA                3   /* 16-bit general timers TIM9-11 without DMA */
#  define STM32_NBTIM                    2   /* 2 basic timers: TIM6, TIM7 with DMA */
#  define STM32_NDMA                     1   /* DMA1, 7-channels */
#  define STM32_NSPI                     2   /* SPI1-2 */
#  define STM32_NI2S                     0   /* No I2S */
#  define STM32_NUSART                   3   /* USART1-3 */
#  define STM32_NI2C                     2   /* I2C1-2 */
#  define STM32_NCAN                     0   /* No CAN */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NLCD                     1   /* LCD 4x18 */
#  define STM32_NUSBOTG                  0   /* No USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NGPIO                    37  /* GPIOA-E,H */
#  define STM32_NADC                     1   /* ADC1, 14-channels */
#  define STM32_NDAC                     2   /* DAC 1, 2 channels */
#  define STM32_NCMP                     2   /* (2) Comparators */
#  define STM32_NCAPSENSE                13  /* Capacitive sensing channels */
#  define STM32_NCRC                     0   /* No CRC */
#  define STM32_NETHERNET                0   /* No Ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32L152R6) || defined(CONFIG_ARCH_CHIP_STM32L152R8) || \
      defined(CONFIG_ARCH_CHIP_STM32L152RB)
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    0   /* No advanced timers */
#  define STM32_NGTIM                    3   /* 16-bit general up/down timers TIM2-4 with DMA */
#  define STM32_NGTIMNDMA                3   /* 16-bit general timers TIM9-11 without DMA */
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
#  define STM32_NDAC                     2   /* DAC 1, 2 channels */
#  define STM32_NCMP                     2   /* (2) Comparators */
#  define STM32_NCAPSENSE                20  /* Capacitive sensing channels */
#  define STM32_NCRC                     0   /* No CRC */
#  define STM32_NETHERNET                0   /* No Ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32L152V6) || defined(CONFIG_ARCH_CHIP_STM32L152V8) || \
      defined(CONFIG_ARCH_CHIP_STM32L152VB)
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    0   /* No advanced timers */
#  define STM32_NGTIM                    3   /* 16-bit general up/down timers TIM2-4 with DMA */
#  define STM32_NGTIMNDMA                3   /* 16-bit general timers TIM9-11 without DMA */
#  define STM32_NBTIM                    2   /* 2 basic timers: TIM6, TIM7 with DMA */
#  define STM32_NDMA                     1   /* DMA1, 7-channels */
#  define STM32_NSPI                     2   /* SPI1-2 */
#  define STM32_NI2S                     0   /* No I2S */
#  define STM32_NUSART                   3   /* USART1-3 */
#  define STM32_NI2C                     2   /* I2C1-2 */
#  define STM32_NCAN                     0   /* No CAN */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NLCD                     1   /* LCD 4x44, 8x40 */
#  define STM32_NUSBOTG                  0   /* No USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NGPIO                    83  /* GPIOA-E,H */
#  define STM32_NADC                     1   /* ADC1, 24-channels */
#  define STM32_NDAC                     2   /* DAC 1, 2 channels */
#  define STM32_NCMP                     2   /* (2) Comparators */
#  define STM32_NCAPSENSE                20  /* Capacitive sensing channels */
#  define STM32_NCRC                     0   /* No CRC */
#  define STM32_NETHERNET                0   /* No Ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32L152CC)
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    0   /* No advanced timers */
#  define STM32_NGTIM                    3   /* 16-bit general up/down timers TIM2-4 with DMA */
#  define STM32_NGTIMNDMA                3   /* 16-bit general timers TIM9-11 without DMA */
#  define STM32_NBTIM                    2   /* 2 basic timers: TIM6, TIM7 with DMA */
#  define STM32_NDMA                     2   /* DMA1, 7-channels, DMA2 (5 channels) */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     2   /* I2S1-2, overlapping with SPI2-3 */
#  define STM32_NUSART                   3   /* USART1-3 */
#  define STM32_NI2C                     2   /* I2C1-2 */
#  define STM32_NCAN                     0   /* No CAN */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NLCD                     1   /* LCD 4x18 */
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NGPIO                    37  /* GPIOA-E,H */
#  define STM32_NADC                     1   /* ADC1, 14-channels */
#  define STM32_NDAC                     2   /* DAC 1, 2 channels */
#  define STM32_NCMP                     2   /* (2) Comparators */
#  define STM32_NCAPSENSE                16  /* Capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32L152RC)
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    0   /* No advanced timers */
#  define STM32_NGTIM                    3   /* 16-bit general up/down timers TIM2-4 with DMA */
#  define STM32_NGTIMNDMA                3   /* 16-bit general timers TIM9-11 without DMA */
#  define STM32_NBTIM                    2   /* 2 basic timers: TIM6, TIM7 with DMA */
#  define STM32_NDMA                     2   /* DMA1, 7-channels, DMA2 (5 channels) */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     2   /* I2S1-2, overlapping with SPI2-3 */
#  define STM32_NUSART                   3   /* USART1-3 */
#  define STM32_NI2C                     2   /* I2C1-2 */
#  define STM32_NCAN                     0   /* No CAN */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NLCD                     1   /* LCD 4x32, 8x28 */
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NGPIO                    51  /* GPIOA-E,H */
#  define STM32_NADC                     1   /* ADC1, 21-channels */
#  define STM32_NDAC                     2   /* DAC 1, 2 channels */
#  define STM32_NCMP                     2   /* (2) Comparators */
#  define STM32_NCAPSENSE                23  /* Capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32L152VC)
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    0   /* No advanced timers */
#  define STM32_NGTIM                    3   /* 16-bit general up/down timers TIM2-4 with DMA */
#  define STM32_NGTIMNDMA                3   /* 16-bit general timers TIM9-11 without DMA */
#  define STM32_NBTIM                    2   /* 2 basic timers: TIM6, TIM7 with DMA */
#  define STM32_NDMA                     2   /* DMA1, 7-channels, DMA2 (5 channels) */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     2   /* I2S1-2, overlapping with SPI2-3 */
#  define STM32_NUSART                   3   /* USART1-3 */
#  define STM32_NI2C                     2   /* I2C1-2 */
#  define STM32_NCAN                     0   /* No CAN */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NLCD                     1   /* LCD 4x44, 8x40 */
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NGPIO                    83  /* GPIOA-E,H */
#  define STM32_NADC                     1   /* ADC1, 25-channels */
#  define STM32_NDAC                     2   /* DAC 1, 2 channels */
#  define STM32_NCMP                     2   /* (2) Comparators */
#  define STM32_NCAPSENSE                23  /* Capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32L151RE) || defined(CONFIG_ARCH_CHIP_STM32L152RE)
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    0   /* No advanced timers */
#  define STM32_NGTIM                    3   /* 16-bit general up/down timers TIM2-4 with DMA */
#  define STM32_NGTIMNDMA                3   /* 16-bit general timers TIM9-11 without DMA */
#  define STM32_NBTIM                    2   /* 2 basic timers: TIM6, TIM7 with DMA */
#  define STM32_NDMA                     2   /* DMA1, 7-channels, DMA2 (5 channels) */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     2   /* I2S1-2, overlapping with SPI2-3 */
#  define STM32_NUSART                   5   /* USART1-5 */
#  define STM32_NI2C                     2   /* I2C1-2 */
#  define STM32_NCAN                     0   /* No CAN */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NLCD                     1   /* LCD 4x44, 8x40 */
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NGPIO                    51  /* GPIOA-E,H */
#  define STM32_NADC                     1   /* ADC1, 25-channels */
#  define STM32_NDAC                     2   /* DAC 1, 2 channels */
#  define STM32_NCMP                     2   /* (2) Comparators */
#  define STM32_NCAPSENSE                23  /* Capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32L151VE) || defined(CONFIG_ARCH_CHIP_STM32L152VE)
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    0   /* No advanced timers */
#  define STM32_NGTIM                    3   /* 16-bit general up/down timers TIM2-4 with DMA */
#  define STM32_NGTIMNDMA                3   /* 16-bit general timers TIM9-11 without DMA */
#  define STM32_NBTIM                    2   /* 2 basic timers: TIM6, TIM7 with DMA */
#  define STM32_NDMA                     2   /* DMA1, 7-channels, DMA2 (5 channels) */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     2   /* I2S1-2, overlapping with SPI2-3 */
#  define STM32_NUSART                   5   /* USART1-5 */
#  define STM32_NI2C                     2   /* I2C1-2 */
#  define STM32_NCAN                     0   /* No CAN */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NLCD                     1   /* LCD 4x44, 8x40 */
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NGPIO                    83  /* GPIOA-E,H */
#  define STM32_NADC                     1   /* ADC1, 25-channels */
#  define STM32_NDAC                     2   /* DAC 1, 2 channels */
#  define STM32_NCMP                     2   /* (2) Comparators */
#  define STM32_NCAPSENSE                23  /* Capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32L151QE) || defined(CONFIG_ARCH_CHIP_STM32L152QE)
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    0   /* No advanced timers */
#  define STM32_NGTIM                    3   /* 16-bit general up/down timers TIM2-4 with DMA */
#  define STM32_NGTIMNDMA                3   /* 16-bit general timers TIM9-11 without DMA */
#  define STM32_NBTIM                    2   /* 2 basic timers: TIM6, TIM7 with DMA */
#  define STM32_NDMA                     2   /* DMA1, 7-channels, DMA2 (5 channels) */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     2   /* I2S1-2, overlapping with SPI2-3 */
#  define STM32_NUSART                   5   /* USART1-5 */
#  define STM32_NI2C                     2   /* I2C1-2 */
#  define STM32_NCAN                     0   /* No CAN */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NLCD                     1   /* LCD 4x44, 8x40 */
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NGPIO                    109 /* GPIOA-E,H */
#  define STM32_NADC                     1   /* ADC1, 25-channels */
#  define STM32_NDAC                     2   /* DAC 1, 2 channels */
#  define STM32_NCMP                     2   /* (2) Comparators */
#  define STM32_NCAPSENSE                33  /* Capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32L151ZE) || defined(CONFIG_ARCH_CHIP_STM32L152ZE)
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    0   /* No advanced timers */
#  define STM32_NGTIM                    3   /* 16-bit general up/down timers TIM2-4 with DMA */
#  define STM32_NGTIMNDMA                3   /* 16-bit general timers TIM9-11 without DMA */
#  define STM32_NBTIM                    2   /* 2 basic timers: TIM6, TIM7 with DMA */
#  define STM32_NDMA                     2   /* DMA1, 7-channels, DMA2 (5 channels) */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     2   /* I2S1-2, overlapping with SPI2-3 */
#  define STM32_NUSART                   5   /* USART1-5 */
#  define STM32_NI2C                     2   /* I2C1-2 */
#  define STM32_NCAN                     0   /* No CAN */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NLCD                     1   /* LCD 4x44, 8x40 */
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NGPIO                    115 /* GPIOA-E,H */
#  define STM32_NADC                     1   /* ADC1, 25-channels */
#  define STM32_NDAC                     2   /* DAC 1, 2 channels */
#  define STM32_NCMP                     2   /* (2) Comparators */
#  define STM32_NCAPSENSE                34  /* Capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32L162ZD)
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    0   /* No advanced timers */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM2-4 with DMA
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
#  define STM32_NLCD                     1   /* LCD 4x44, 8x40 */
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NGPIO                    115 /* GPIOA-G,H */
#  define STM32_NADC                     1   /* ADC1, 40-channels */
#  define STM32_NDAC                     2   /* DAC 1, 2 channels */
#  define STM32_NCMP                     2   /* (2) Comparators */
#  define STM32_NCAPSENSE                34  /* Capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32L162VE)
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    0   /* No advanced timers */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM2-4 with DMA
                                              * 32-bit general timer TIM5 with DMA */
#  define STM32_NGTIMNDMA                3   /* 16-bit general timers TIM9-11 without DMA */
#  define STM32_NBTIM                    2   /* 2 basic timers: TIM6, TIM7 with DMA */
#  define STM32_NDMA                     2   /* DMA1, 12-channels */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     2   /* I2S1-2, overlapping with SPI2-3 */
#  define STM32_NUSART                   5   /* USART1-3, UART4-5 */
#  define STM32_NI2C                     2   /* I2C1-2 */
#  define STM32_NCAN                     0   /* No CAN */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NLCD                     1   /* LCD 4x44, 8x40*/
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NGPIO                    83  /* GPIOA-G,H */

#  define STM32_NADC                     1   /* ADC1, 25-channels */
#  define STM32_NDAC                     2   /* DAC 1, 2 channels */
#  define STM32_NCMP                     2   /* (2) Comparators */
#  define STM32_NCAPSENSE                23  /* Capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

/* STM32 F100 Value Line **************************************************************************/

#elif defined(CONFIG_ARCH_CHIP_STM32F100C8) || defined(CONFIG_ARCH_CHIP_STM32F100CB) \
 || defined(CONFIG_ARCH_CHIP_STM32F100R8) || defined(CONFIG_ARCH_CHIP_STM32F100RB)
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    1   /* One advanced timer TIM1 */
#  define STM32_NGTIM                    3   /* 16-bit general timers TIM2-4 with DMA */
#  define STM32_NGTIMNDMA                0   /* No 16-bit general timers without DMA */
#  define STM32_NBTIM                    2   /* 2 basic timers: TIM6, TIM7 */

/* TODO: there are also 3 additional timers (15-17) that don't fit any existing category */

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
#  define STM32_NDAC                     2   /* DAC 1, 2 channels */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC1 */
#  define STM32_NETHERNET                0   /* No Ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F100V8) || defined(CONFIG_ARCH_CHIP_STM32F100VB)
#  define STM32_NFSMC                    0   /* FSMC */
#  define STM32_NATIM                    1   /* One advanced timer TIM1 */
#  define STM32_NGTIM                    3   /* 16-bit general timers TIM2-4 with DMA */
#  define STM32_NGTIMNDMA                0   /* No 16-bit general timers without DMA */
#  define STM32_NBTIM                    2   /* 2 basic timers: TIM6, TIM7 */

/* TODO: there are also 3 additional timers (15-17) that don't fit any existing category */

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
#  define STM32_NDAC                     2   /* DAC 1, 2 channels */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC1 */
#  define STM32_NETHERNET                0   /* No Ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

/* STM32 F100 High-density value Line *************************************************************/

#elif defined(CONFIG_ARCH_CHIP_STM32F100RC) || defined(CONFIG_ARCH_CHIP_STM32F100RD) \
 || defined(CONFIG_ARCH_CHIP_STM32F100RE)
#  define STM32_NFSMC                    0   /* FSMC */
#  define STM32_NATIM                    1   /* One advanced timer TIM1 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM2-5 with DMA */
#  define STM32_NGTIMNDMA                0   /* No 16-bit general timers without DMA */
#  define STM32_NBTIM                    2   /* 2 basic timers: TIM6, TIM7 */

/* TODO: there are also 6 additional timers (12-17) that don't fit any existing category */

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
#  define STM32_NDAC                     2   /* DAC 1, 2 channels */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC1 */
#  define STM32_NETHERNET                0   /* No Ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F100VC) || defined(CONFIG_ARCH_CHIP_STM32F100VD) \
 || defined(CONFIG_ARCH_CHIP_STM32F100VE)
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    1   /* One advanced timer TIM1 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM2-5 with DMA */
#  define STM32_NGTIMNDMA                0   /* No 16-bit general timers without DMA */
#  define STM32_NBTIM                    2   /* 2 basic timers: TIM6, TIM7 */

/* TODO: there are also 6 additional timers (12-17) that don't fit any existing category */

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
#  define STM32_NDAC                     2   /* DAC 1, 2 channels */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC1 */
#  define STM32_NETHERNET                0   /* No Ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

/* STM32 F102x8/102xB Medium Density USB Access Family ********************************************/

#elif defined(CONFIG_ARCH_CHIP_STM32F102CB)
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    0   /* No advanced timer TIM1 */
#  define STM32_NGTIM                    3   /* 16-bit general timers TIM2-4 */
#  define STM32_NGTIMNDMA                0   /* No 16-bit general timers without DMA */
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

/* STM32 F103 Low Density Family ******************************************************************/

/* STM32F103C4 & STM32F103C6 */

#elif defined(CONFIG_ARCH_CHIP_STM32F103C4)
#  define STM32_NFSMC                    0   /* FSMC */
#  define STM32_NATIM                    1   /* One advanced timer TIM1 */
#  define STM32_NGTIM                    2   /* General timers TIM2,3 */
#  define STM32_NGTIMNDMA                0   /* No 16-bit general timers without DMA */
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
#  define STM32_NETHERNET                0   /* No Ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

/* STM32 F103 Medium Density Performance Line *****************************************************/

#elif defined(CONFIG_ARCH_CHIP_STM32F103T8) || defined(CONFIG_ARCH_CHIP_STM32F103TB)
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    1   /* One advanced timer TIM1 */
#  define STM32_NGTIM                    3   /* General timers TIM2-4 */
#  define STM32_NGTIMNDMA                0   /* No 16-bit general timers without DMA */
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
#  define STM32_NETHERNET                0   /* No Ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F103C8) || defined(CONFIG_ARCH_CHIP_STM32F103CB)
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    1   /* One advanced timer TIM1 */
#  define STM32_NGTIM                    3   /* General timers TIM2-4 */
#  define STM32_NGTIMNDMA                0   /* No 16-bit general timers without DMA */
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
#  define STM32_NETHERNET                0   /* No Ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F103R8) || defined(CONFIG_ARCH_CHIP_STM32F103RB)
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    1   /* One advanced timer TIM1 */
#  define STM32_NGTIM                    3   /* General timers TIM2-4 */
#  define STM32_NGTIMNDMA                0   /* No 16-bit general timers without DMA */
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
#  define STM32_NETHERNET                0   /* No Ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

/* STM32 F103 High Density Family *****************************************************************/

/* STM32F103RC, STM32F103RD, and STM32F103RE are all provided in 64 pin packages and
 * differ only in the available FLASH and SRAM.
 */

#elif defined(CONFIG_ARCH_CHIP_STM32F103RC) || defined(CONFIG_ARCH_CHIP_STM32F103RD) || \
      defined(CONFIG_ARCH_CHIP_STM32F103RE) || defined(CONFIG_ARCH_CHIP_STM32F103RG)
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    2   /* Two advanced timers TIM1 and TIM8 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM2-5 with DMA */
#  define STM32_NGTIMNDMA                0   /* No 16-bit general timers without DMA */
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
#  define STM32_NDAC                     2   /* DAC1, 2 channels */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No Ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

/* STM32F103VC, STM32F103VD, and STM32F103VE are all provided in 100 pin packages and differ
 * only in the available FLASH and SRAM.
 */

#elif defined(CONFIG_ARCH_CHIP_STM32F103VC) || defined(CONFIG_ARCH_CHIP_STM32F103VE)
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    2   /* Two advanced timers TIM1 and TIM8 */
#  define STM32_NGTIM                    4   /* General timers TIM2-5 */
#  define STM32_NGTIMNDMA                0   /* No 16-bit general timers without DMA */
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
#  define STM32_NDAC                     2   /* DAC1, 2 channels */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No Ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

/* STM32F103ZC, STM32F103ZD, and STM32F103ZE are all provided in 144 pin packages and differ
 * only in the available FLASH and SRAM.
 */

#elif defined(CONFIG_ARCH_CHIP_STM32F103ZE)
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    1   /* One advanced timer TIM1 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM2-5 with DMA */
#  define STM32_NGTIMNDMA                0   /* No 16-bit general timers without DMA */
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

/* STM32 F105/F107 Connectivity Line **************************************************************/

#elif defined(CONFIG_ARCH_CHIP_STM32F105VB)
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    1   /* One advanced timers TIM1 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM2-5 with DMA */
#  define STM32_NGTIMNDMA                0   /* No 16-bit general timers without DMA */
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
#  define STM32_NDAC                     2   /* DAC1, 2 channels */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* 100/100 Ethernet MAC */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F105RB)
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    1   /* One advanced timers TIM1 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM2-5 with DMA */
#  define STM32_NGTIMNDMA                0   /* No 16-bit general timers without DMA */
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
#  define STM32_NGPIO                    51  /* GPIOA-E */
#  define STM32_NADC                     2   /* ADC1-2 */
#  define STM32_NDAC                     2   /* DAC1, 2 channels */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* 100/100 Ethernet MAC */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F107VC)
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    1   /* One advanced timers TIM1 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM2-5 with DMA */
#  define STM32_NGTIMNDMA                0   /* No 16-bit general timers without DMA */
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
#  define STM32_NDAC                     2   /* DAC1, 2 channels */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                1   /* 100/100 Ethernet MAC */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

/* STM32 F2 Family ********************************************************************************/

#elif defined(CONFIG_ARCH_CHIP_STM32F205RG)  /* UFBGA-176 1024Kb FLASH 128Kb SRAM */
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
#  define STM32_NDAC                     2   /* 12-bit DAC1, 2 channels */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F207VC) || defined(CONFIG_ARCH_CHIP_STM32F207VE) || \
      defined(CONFIG_ARCH_CHIP_STM32F207VF) || defined(CONFIG_ARCH_CHIP_STM32F207VG)
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
#  define STM32_NGPIO                    82  /* GPIOA-I */
#  define STM32_NADC                     3   /* 12-bit ADC1-3, 24 channels */
#  define STM32_NDAC                     2   /* 12-bit DAC1, 2 channels */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                1   /* 100/100 Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    1   /* Digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F207IC) || defined(CONFIG_ARCH_CHIP_STM32F207IE) || \
      defined(CONFIG_ARCH_CHIP_STM32F207IF) || defined(CONFIG_ARCH_CHIP_STM32F207IG)
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
#  define STM32_NDAC                     2   /* 12-bit DAC1, 2 channels */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                1   /* 100/100 Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    1   /* Digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F207ZC)  || defined(CONFIG_ARCH_CHIP_STM32F207ZE) || \
      defined(CONFIG_ARCH_CHIP_STM32F207ZF) || defined(CONFIG_ARCH_CHIP_STM32F207ZG)
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
#  define STM32_NDAC                     2   /* 12-bit DAC1, 2 channels */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                1   /* 100/100 Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    1   /* Digital camera interface (DCMI) */

/* STM23 F3 Family ********************************************************************************/

/* Part Numbering: STM32Fssscfxxx
 *
 *  Where
 *     sss = 302/303, 334 or 372/373
 *     c   = C (48pins) R (68 pins) V (100 pins)
 *     c   = K (32 pins), C (48 pins), R (68 pins), V (100 pins)
 *     f   = 6 (32KB FLASH), 8 (64KB FLASH), B (128KB FLASH), C (256KB FLASH)
 *     xxx = Package, temperature range, options (ignored here)
 */

#elif defined(CONFIG_ARCH_CHIP_STM32F302K6) || defined(CONFIG_ARCH_CHIP_STM32F302K8)
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
#  define STM32_NDAC                     1   /* (1) 12-bit DAC1, 1 channel */
#  define STM32_NCMP                     2   /* (2) Ultra-fast analog comparators: COMP2 and COMP4 */
#  define STM32_NPGA                     1   /* (1) Operational amplifiers: OPAMP */
#  define STM32_NCAPSENSE                13  /* (13) Capacitive sensing channels */
#  define STM32_NCRC                     1   /* (1) CRC calculation unit */
#  define STM32_NETHERNET                0   /* (0) No Ethernet MAC */
#  define STM32_NRNG                     0   /* (0) No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* (0) No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F302C6) || defined(CONFIG_ARCH_CHIP_STM32F302C8)
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
#  define STM32_NUSART                   3   /* (3) USART1-3, no UARTs */
#  define STM32_NI2C                     3   /* (3) I2C1-3 */
#  define STM32_NCAN                     1   /* (1) CAN1 */
#  define STM32_NSDIO                    0   /* (0) No SDIO */
#  define STM32_NLCD                     0   /* (0) No LCD */
#  define STM32_NUSBOTG                  0   /* USB FS device, but no USB OTG FS/HS */
#  define STM32_NGPIO                    37  /* GPIOA-F */
#  define STM32_NADC                     1   /* (1) 12-bit ADC1 */
#  define STM32_NDAC                     1   /* (1) 12-bit DAC1, 1 channel */
#  define STM32_NCMP                     3   /* (3) Ultra-fast analog comparators: COMP2, COMP4 and COMP6*/
#  define STM32_NPGA                     1   /* (1) Operational amplifiers: OPAMP */
#  define STM32_NCAPSENSE                17  /* (17) Capacitive sensing channels */
#  define STM32_NCRC                     1   /* (1) CRC calculation unit */
#  define STM32_NETHERNET                0   /* (0) No Ethernet MAC */
#  define STM32_NRNG                     0   /* (0) No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* (0) No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F302R6) || defined(CONFIG_ARCH_CHIP_STM32F302R8)
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
#  define STM32_NUSART                   3   /* (2) USART1-3, no UARTs */
#  define STM32_NI2C                     3   /* (3) I2C1-3 */
#  define STM32_NCAN                     1   /* (1) CAN1 */
#  define STM32_NSDIO                    0   /* (0) No SDIO */
#  define STM32_NLCD                     0   /* (0) No LCD */
#  define STM32_NUSBOTG                  0   /* USB FS device, but no USB OTG FS/HS */
#  define STM32_NGPIO                    51  /* GPIOA-F */
#  define STM32_NADC                     1   /* (1) 12-bit ADC1 */
#  define STM32_NDAC                     1   /* (1) 12-bit DAC1, 1 channel */
#  define STM32_NCMP                     3   /* (3) Ultra-fast analog comparators: COMP2, COMP4 and COMP6*/
#  define STM32_NPGA                     1   /* (1) Operational amplifiers: OPAMP */
#  define STM32_NCAPSENSE                18  /* (18) Capacitive sensing channels */
#  define STM32_NCRC                     1   /* (1) CRC calculation unit */
#  define STM32_NETHERNET                0   /* (0) No Ethernet MAC */
#  define STM32_NRNG                     0   /* (0) No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* (0) No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F302CB) || defined(CONFIG_ARCH_CHIP_STM32F302CC)
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
#  define STM32_NDAC                     1   /* (1) 12-bit DAC1, 1 channel */
#  define STM32_NCAPSENSE                0   /* (0) No capacitive sensing channels */
#  define STM32_NCRC                     1   /* (1) CRC calculation unit */
#  define STM32_NETHERNET                0   /* (0) No Ethernet MAC */
#  define STM32_NRNG                     0   /* (0) No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* (0) No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F302RB) || defined(CONFIG_ARCH_CHIP_STM32F302RC)
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
#  define STM32_NDAC                     1   /* (1) 12-bit DAC1, 1 channel */
#  define STM32_NCAPSENSE                0   /* (0) No capacitive sensing channels */
#  define STM32_NCRC                     1   /* (1) CRC calculation unit */
#  define STM32_NETHERNET                0   /* (0) No Ethernet MAC */
#  define STM32_NRNG                     0   /* (0) No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* (0) No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F302VB) || defined(CONFIG_ARCH_CHIP_STM32F302VC)
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
#  define STM32_NDAC                     1   /* (1) 12-bit DAC1, 1 channel */
#  define STM32_NCAPSENSE                0   /* (0) No capacitive sensing channels */
#  define STM32_NCRC                     1   /* (1) CRC calculation unit */
#  define STM32_NETHERNET                0   /* (0) No Ethernet MAC */
#  define STM32_NRNG                     0   /* (0) No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* (0) No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F303K6) || defined(CONFIG_ARCH_CHIP_STM32F303K8)
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    1   /* (1) Advanced 16-bit timers with DMA: TIM1 */
#  define STM32_NGTIM                    5   /* (1) 16-bit general timers with DMA: TIM3
                                              * (1) 32-bit general timers with DMA: TIM2
                                              * (3) 16-bit general timers count-up timers with DMA: TIM15-17 */
#  define STM32_NGTIMNDMA                0   /* All timers have DMA */
#  define STM32_NBTIM                    2   /* (2) Basic timers: TIM6 and TIM7 */
#  define STM32_NDMA                     1   /* (1) DMA1 (7 channels) */
#  define STM32_NSPI                     1   /* (1) SPI1 */
#  define STM32_NI2S                     0   /* (0) No I2S */
#  define STM32_NUSART                   2   /* (2) USART1-2, no UARTs */
#  define STM32_NI2C                     1   /* (1) I2C1 */
#  define STM32_NCAN                     1   /* (1) CAN1 */
#  define STM32_NSDIO                    0   /* (0) No SDIO */
#  define STM32_NLCD                     0   /* (0) No LCD */
#  define STM32_NUSBOTG                  0   /* No USB OTG FS/HS */
#  define STM32_NGPIO                    25  /* GPIOA-F */
#  define STM32_NADC                     2   /* (2) 12-bit ADC1-2 */
#  define STM32_NDAC                     3   /* (3) 12-bit DAC1-2, 3 channels */
#  define STM32_NCAPSENSE                0   /* (0) No capacitive sensing channels */
#  define STM32_NCRC                     1   /* (1) CRC calculation unit */
#  define STM32_NETHERNET                0   /* (0) No Ethernet MAC */
#  define STM32_NRNG                     0   /* (0) No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* (0) No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F303C6) || defined(CONFIG_ARCH_CHIP_STM32F303C8)
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    1   /* (1) Advanced 16-bit timers with DMA: TIM1 */
#  define STM32_NGTIM                    5   /* (1) 16-bit general timers with DMA: TIM3
                                              * (1) 32-bit general timers with DMA: TIM2
                                              * (3) 16-bit general timers count-up timers with DMA: TIM15-17 */
#  define STM32_NGTIMNDMA                0   /* All timers have DMA */
#  define STM32_NBTIM                    2   /* (2) Basic timers: TIM6 and TIM7 */
#  define STM32_NDMA                     1   /* (1) DMA1 (7 channels) */
#  define STM32_NSPI                     1   /* (1) SPI1 */
#  define STM32_NI2S                     0   /* (0) No I2S */
#  define STM32_NUSART                   3   /* (3) USART1-3, no UARTs */
#  define STM32_NI2C                     1   /* (1) I2C1 */
#  define STM32_NCAN                     1   /* (1) CAN1 */
#  define STM32_NSDIO                    0   /* (0) No SDIO */
#  define STM32_NLCD                     0   /* (0) No LCD */
#  define STM32_NUSBOTG                  0   /* No USB OTG FS/HS */
#  define STM32_NGPIO                    37  /* GPIOA-F */
#  define STM32_NADC                     2   /* (2) 12-bit ADC1-2 */
#  define STM32_NDAC                     3   /* (3) 12-bit DAC1-2, 3 channels */
#  define STM32_NCAPSENSE                0   /* (0) No capacitive sensing channels */
#  define STM32_NCRC                     1   /* (1) CRC calculation unit */
#  define STM32_NETHERNET                0   /* (0) No Ethernet MAC */
#  define STM32_NRNG                     0   /* (0) No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* (0) No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F303CB) || defined(CONFIG_ARCH_CHIP_STM32F303CC)
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
#  define STM32_NADC                     4   /* (3) 12-bit ADC1-4 */
#  define STM32_NDAC                     2   /* (2) 12-bit DAC1, 2 channels */
#  define STM32_NCAPSENSE                0   /* (0) No capacitive sensing channels */
#  define STM32_NCRC                     1   /* (1) CRC calculation unit */
#  define STM32_NETHERNET                0   /* (0) No Ethernet MAC */
#  define STM32_NRNG                     0   /* (0) No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* (0) No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F303RB) || defined(CONFIG_ARCH_CHIP_STM32F303RC)
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
#  define STM32_NADC                     4   /* (3) 12-bit ADC1-4 */
#  define STM32_NDAC                     2   /* (2) 12-bit DAC1, 2 channels */
#  define STM32_NCAPSENSE                0   /* (0) No capacitive sensing channels */
#  define STM32_NCRC                     1   /* (1) CRC calculation unit */
#  define STM32_NETHERNET                0   /* (0) No Ethernet MAC */
#  define STM32_NRNG                     0   /* (0) No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* (0) No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F303RD) || defined(CONFIG_ARCH_CHIP_STM32F303RE)
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    2   /* (2) Advanced 16-bit timers with DMA: TIM1 and TIM8 */
#  define STM32_NGTIM                    6   /* (2) 16-bit general timers with DMA: TIM3 and TIM4
                                              * (1) 32-bit general timers with DMA: TIM2
                                              * (3) 16-bit general timers count-up timers with DMA: TIM15-17 */
#  define STM32_NGTIMNDMA                0   /* All timers have DMA */
#  define STM32_NBTIM                    2   /* (2) Basic timers: TIM6 and TIM7 */
#  define STM32_NDMA                     2   /* (2) DMA1 (7 channels) and DMA2 (5 channels) */
#  define STM32_NSPI                     4   /* (4) SPI1-4 */
#  define STM32_NI2S                     2   /* (2) I2S1-2 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   5   /* (5) USART1-3, UART4-5 */
#  define STM32_NI2C                     3   /* (2) I2C1-3 */
#  define STM32_NCAN                     1   /* (1) CAN1 */
#  define STM32_NSDIO                    0   /* (0) No SDIO */
#  define STM32_NLCD                     0   /* (0) No LCD */
#  define STM32_NUSBOTG                  0   /* USB FS device, but no USB OTG FS/HS */
#  define STM32_NGPIO                    51  /* GPIOA-F */
#  define STM32_NADC                     4   /* (4) 12-bit ADC1-4 */
#  define STM32_NDAC                     2   /* (2) 12-bit DAC1, 2 channels */
#  define STM32_NCAPSENSE                0   /* (0) No capacitive sensing channels */
#  define STM32_NCRC                     1   /* (1) CRC calculation unit */
#  define STM32_NETHERNET                0   /* (0) No Ethernet MAC */
#  define STM32_NRNG                     0   /* (0) No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* (0) No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F303VB) || defined(CONFIG_ARCH_CHIP_STM32F303VC)
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
#  define STM32_NADC                     4   /* (3) 12-bit ADC1-4 */
#  define STM32_NDAC                     2   /* (2) 12-bit DAC1, 2 channels */
#  define STM32_NCAPSENSE                0   /* (0) No capacitive sensing channels */
#  define STM32_NCRC                     1   /* (1) CRC calculation unit */
#  define STM32_NETHERNET                0   /* (0) No Ethernet MAC */
#  define STM32_NRNG                     0   /* (0) No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* (0) No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F303RD) || defined(CONFIG_ARCH_CHIP_STM32F303RE)
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    2   /* (2) Advanced 16-bit timers with DMA: TIM1 and TIM8 */
#  define STM32_NGTIM                    6   /* (5) 16-bit general timers
                                              * (1) 32-bit general timers */
#  define STM32_NGTIMNDMA                0   /* All timers have DMA */
#  define STM32_NBTIM                    2   /* (2) Basic timers: TIM6 and TIM7 */
#  define STM32_NDMA                     2   /* (2) DMA1 (7 channels) and DMA2 (5 channels) */
#  define STM32_NSPI                     4   /* (4) SPI1-4 */
#  define STM32_NI2S                     2   /* (2) I2S1-2 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   5   /* (5) USART1-3, UART4-5 */
#  define STM32_NI2C                     3   /* (3) I2C1-3 */
#  define STM32_NCAN                     1   /* (1) CAN1 */
#  define STM32_NSDIO                    0   /* (0) No SDIO */
#  define STM32_NLCD                     0   /* (0) No LCD */
#  define STM32_NUSBOTG                  0   /* USB FS device, but no USB OTG FS/HS */
#  define STM32_NGPIO                    51  /* GPIOA-F */
#  define STM32_NADC                     4   /* (4) 12-bit ADC1-4 */
#  define STM32_NDAC                     2   /* (2) 12-bit DAC1, 2 channels */
#  define STM32_NCAPSENSE                18  /* (18) No capacitive sensing channels */
#  define STM32_NCRC                     1   /* (1) CRC calculation unit */
#  define STM32_NETHERNET                0   /* (0) No Ethernet MAC */
#  define STM32_NRNG                     0   /* (0) No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* (0) No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F303VD) || defined(CONFIG_ARCH_CHIP_STM32F303VE)
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    3   /* (3) Advanced 16-bit timers with DMA: TIM1, TIM8 and TIM20 */
#  define STM32_NGTIM                    6   /* (5) 16-bit general timers
                                              * (1) 32-bit general timers */
#  define STM32_NGTIMNDMA                0   /* All timers have DMA */
#  define STM32_NBTIM                    2   /* (2) Basic timers: TIM6 and TIM7 */
#  define STM32_NDMA                     2   /* (2) DMA1 (7 channels) and DMA2 (5 channels) */
#  define STM32_NSPI                     4   /* (4) SPI1-4 */
#  define STM32_NI2S                     2   /* (2) I2S1-2 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   5   /* (5) USART1-3, UART4-5 */
#  define STM32_NI2C                     3   /* (3) I2C1-3 */
#  define STM32_NCAN                     1   /* (1) CAN1 */
#  define STM32_NSDIO                    0   /* (0) No SDIO */
#  define STM32_NLCD                     0   /* (0) No LCD */
#  define STM32_NUSBOTG                  0   /* USB FS device, but no USB OTG FS/HS */
#  define STM32_NGPIO                    84  /* GPIOA-F (depends on package) */
#  define STM32_NADC                     4   /* (4) 12-bit ADC1-4 */
#  define STM32_NDAC                     2   /* (2) 12-bit DAC1, 2 channels */
#  define STM32_NCAPSENSE                24  /* (24) No capacitive sensing channels */
#  define STM32_NCRC                     1   /* (1) CRC calculation unit */
#  define STM32_NETHERNET                0   /* (0) No Ethernet MAC */
#  define STM32_NRNG                     0   /* (0) No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* (0) No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F303ZD) || defined(CONFIG_ARCH_CHIP_STM32F303ZE)
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    3   /* (3) Advanced 16-bit timers with DMA: TIM1, TIM8 and TIM20 */
#  define STM32_NGTIM                    6   /* (5) 16-bit general timers
                                              * (1) 32-bit general timers */
#  define STM32_NGTIMNDMA                0   /* All timers have DMA */
#  define STM32_NBTIM                    2   /* (2) Basic timers: TIM6 and TIM7 */
#  define STM32_NDMA                     2   /* (2) DMA1 (7 channels) and DMA2 (5 channels) */
#  define STM32_NSPI                     4   /* (4) SPI1-4 */
#  define STM32_NI2S                     2   /* (2) I2S1-2 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   5   /* (5) USART1-3, UART4-5 */
#  define STM32_NI2C                     3   /* (3) I2C1-3 */
#  define STM32_NCAN                     1   /* (1) CAN1 */
#  define STM32_NSDIO                    0   /* (0) No SDIO */
#  define STM32_NLCD                     0   /* (0) No LCD */
#  define STM32_NUSBOTG                  0   /* USB FS device, but no USB OTG FS/HS */
#  define STM32_NGPIO                    115 /* GPIOA-F */
#  define STM32_NADC                     4   /* (4) 12-bit ADC1-4 */
#  define STM32_NDAC                     2   /* (2) 12-bit DAC1, 2 channels */
#  define STM32_NCAPSENSE                24  /* (24) No capacitive sensing channels */
#  define STM32_NCRC                     1   /* (1) CRC calculation unit */
#  define STM32_NETHERNET                0   /* (0) No Ethernet MAC */
#  define STM32_NRNG                     0   /* (0) No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* (0) No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F334K4) || defined(CONFIG_ARCH_CHIP_STM32F334K6) || defined(CONFIG_ARCH_CHIP_STM32F334K8)
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_HRTIM                    1   /* (1) High-resolution timer 16-bit, 10 channels: HRTIM1 */
#  define STM32_NATIM                    1   /* (1) Advanced 16-bit timers with DMA: TIM1*/
#  define STM32_NGTIM                    5   /* (1) 16-bit general timers with DMA: TIM3
                                              * (1) 32-bit general timers with DMA: TIM2
                                              * (3) 16-bit general timers count-up timers with DMA: TIM15-17 */
#  define STM32_NGTIMNDMA                0   /* All timers have DMA */
#  define STM32_NBTIM                    2   /* (2) Basic timers: TIM6 and TIM7 */
#  define STM32_NDMA                     1   /* (1) DMA1 (7 channels) */
#  define STM32_NSPI                     1   /* (1) SPI1 */
#  define STM32_NI2S                     0   /* (0) No I2S1 */
#  define STM32_NUSART                   2   /* (2) USART1-2 */
#  define STM32_NI2C                     1   /* (1) I2C1 */
#  define STM32_NCAN                     1   /* (1) CAN1 */
#  define STM32_NSDIO                    0   /* (0) No SDIO */
#  define STM32_NLCD                     0   /* (0) No LCD */
#  define STM32_NUSBOTG                  0   /* (0) No USB */
#  define STM32_NGPIO                    25  /* GPIOA-F */
#  define STM32_NADC                     2   /* (2) 12-bit ADC1-2 */
#  define STM32_NDAC                     3   /* (3) 12-bit DAC1-2, 3 channels */
#  define STM32_NCMP                     2   /* (2) Ultra-fast analog comparators: COMP2 and COMP4 */
#  define STM32_NPGA                     1   /* (1) Operational amplifiers: OPAMP */
#  define STM32_NCAPSENSE                14  /* (14) Capacitive sensing channels */
#  define STM32_NCRC                     1   /* (1) CRC calculation unit */
#  define STM32_NETHERNET                0   /* (0) No Ethernet MAC */
#  define STM32_NRNG                     0   /* (0) No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* (0) No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F334C4) || defined(CONFIG_ARCH_CHIP_STM32F334C6) || defined(CONFIG_ARCH_CHIP_STM32F334C8)
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_HRTIM                    1   /* (1) High-resolution timer 16-bit, 10 channels: HRTIM1 */
#  define STM32_NATIM                    1   /* (1) Advanced 16-bit timers with DMA: TIM1*/
#  define STM32_NGTIM                    5   /* (1) 16-bit general timers with DMA: TIM3
                                              * (1) 32-bit general timers with DMA: TIM2
                                              * (3) 16-bit general timers count-up timers with DMA: TIM15-17 */
#  define STM32_NGTIMNDMA                0   /* All timers have DMA */
#  define STM32_NBTIM                    2   /* (2) Basic timers: TIM6 and TIM7 */
#  define STM32_NDMA                     1   /* (1) DMA1 (7 channels) */
#  define STM32_NSPI                     1   /* (1) SPI1 */
#  define STM32_NI2S                     0   /* (0) No I2S1 */
#  define STM32_NUSART                   3   /* (3) USART1-3 */
#  define STM32_NI2C                     1   /* (1) I2C1 */
#  define STM32_NCAN                     1   /* (1) CAN1 */
#  define STM32_NSDIO                    0   /* (0) No SDIO */
#  define STM32_NLCD                     0   /* (0) No LCD */
#  define STM32_NUSBOTG                  0   /* (0) No USB */
#  define STM32_NGPIO                    37  /* GPIOA-F */
#  define STM32_NADC                     2   /* (2) 12-bit ADC1-2 */
#  define STM32_NDAC                     3   /* (3) 12-bit DAC1-2, 3 channels */
#  define STM32_NCMP                     3   /* (3) Ultra-fast analog comparators: COMP2, COMP4 and COMP6 */
#  define STM32_NPGA                     1   /* (1) Operational amplifiers: OPAMP */
#  define STM32_NCAPSENSE                17  /* (17) Capacitive sensing channels */
#  define STM32_NCRC                     1   /* (1) CRC calculation unit */
#  define STM32_NETHERNET                0   /* (0) No Ethernet MAC */
#  define STM32_NRNG                     0   /* (0) No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* (0) No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F334R4) || defined(CONFIG_ARCH_CHIP_STM32F334R6) || defined(CONFIG_ARCH_CHIP_STM32F334R8)
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_HRTIM                    1   /* (1) High-resolution timer 16-bit, 10 channels: HRTIM1 */
#  define STM32_NATIM                    1   /* (1) Advanced 16-bit timers with DMA: TIM1*/
#  define STM32_NGTIM                    5   /* (1) 16-bit general timers with DMA: TIM3
                                              * (1) 32-bit general timers with DMA: TIM2
                                              * (3) 16-bit general timers count-up timers with DMA: TIM15-17 */
#  define STM32_NGTIMNDMA                0   /* All timers have DMA */
#  define STM32_NBTIM                    2   /* (2) Basic timers: TIM6 and TIM7 */
#  define STM32_NDMA                     1   /* (1) DMA1 (7 channels) */
#  define STM32_NSPI                     1   /* (1) SPI1 */
#  define STM32_NI2S                     0   /* (0) No I2S1 */
#  define STM32_NUSART                   3   /* (3) USART1-3 */
#  define STM32_NI2C                     1   /* (1) I2C1 */
#  define STM32_NCAN                     1   /* (1) CAN1 */
#  define STM32_NSDIO                    0   /* (0) No SDIO */
#  define STM32_NLCD                     0   /* (0) No LCD */
#  define STM32_NUSBOTG                  0   /* (0) No USB */
#  define STM32_NGPIO                    51  /* GPIOA-F */
#  define STM32_NADC                     2   /* (2) 12-bit ADC1-2 */
#  define STM32_NDAC                     3   /* (3) 12-bit DAC1-2, 3 channels */
#  define STM32_NCMP                     3   /* (3) Ultra-fast analog comparators: COMP2, COMP4 and COMP6 */
#  define STM32_NPGA                     1   /* (1) Operational amplifiers: OPAMP */
#  define STM32_NCAPSENSE                18  /* (18) Capacitive sensing channels */
#  define STM32_NCRC                     1   /* (1) CRC calculation unit */
#  define STM32_NETHERNET                0   /* (0) No Ethernet MAC */
#  define STM32_NRNG                     0   /* (0) No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* (0) No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F373C8) || defined(CONFIG_ARCH_CHIP_STM32F373CB) || defined(CONFIG_ARCH_CHIP_STM32F373CC)
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
#  define STM32_NADC                     1   /* (1) 12-bit ADC1 */
#  define STM32_NSDADC                   3   /* (3) 16-bit SDADC1-3 */
#  define STM32_NDAC                     3   /* (3) 12-bit DAC1-2, 3 channels */
#  define STM32_NCAPSENSE                0   /* (0) No capacitive sensing channels */
#  define STM32_NCRC                     1   /* (1) CRC calculation unit */
#  define STM32_NETHERNET                0   /* (0) No Ethernet MAC */
#  define STM32_NRNG                     0   /* (0) No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* (0) No digital camera interface (DCMI) */

/* STM23 F4 Family ********************************************************************************/

/* STM32F01xB/C Family Differences:
 *
 * PART        PACKAGE          FLASH SDIO ADC Channels
 * ----------- ---------------- ----- ---- ------------
 * STM32F401CB WLCSP49/UFQFPN48 128Kb No   10
 * STM32F401RB LQFP64           128Kb Yes  16
 * STM32F401VB UFBGA100/LQFP100 128Kb Yes  16
 * STM32F401CC WLCSP49/UFQFPN48 256Kb No   10
 * STM32F401RC LQFP64           256Kb Yes  16
 * STM32F401VC UFBGA100/LQFP100 256Kb Yes  16
 */

#elif defined(CONFIG_ARCH_CHIP_STM32F401CB) || defined(CONFIG_ARCH_CHIP_STM32F401RB) || \
      defined(CONFIG_ARCH_CHIP_STM32F401VB) || defined(CONFIG_ARCH_CHIP_STM32F401CC) || \
      defined(CONFIG_ARCH_CHIP_STM32F401RC) || defined(CONFIG_ARCH_CHIP_STM32F401VC)
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    1   /* One advanced timers TIM1 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define STM32_NGTIMNDMA                3   /* 16-bit general timers TIM9-11 without DMA */
#  define STM32_NBTIM                    0   /* No basic timers */
#  define STM32_NDMA                     2   /* DMA1-2 with 8 streams each*/
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     2   /* I2S2-3 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   3   /* USART1, 2 and 6 */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     0   /* No CAN */
#  if defined(CONFIG_ARCH_CHIP_STM32F401CB) || defined(CONFIG_ARCH_CHIP_STM32F401CC)
#    define STM32_NSDIO                  0   /* No SDIO interface */
#  else
#    define STM32_NSDIO                  1   /* One SDIO interface */
#  endif
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  1   /* USB OTG FS (only) */
#  define STM32_NGPIO                    50  /* GPIOA-H */
#  define STM32_NADC                     1   /* One 12-bit ADC1, 10 or 16 channels */
#  define STM32_NDAC                     0   /* No DAC */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* No CRC */
#  define STM32_NETHERNET                0   /* No Ethernet MAC */
#  define STM32_NRNG                     0   /* No Random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

/* STM32F01xD/E Family Differences:
 *
 * PART        PACKAGE          FLASH SDIO ADC Channels
 * ----------- ---------------- ----- ---- ------------
 * STM32F401CD WLCSP49/UFQFPN48 384Kb No   10
 * STM32F401RD LQFP64           384Kb Yes  16
 * STM32F401VD UFBGA100/LQFP100 384Kb Yes  16
 * STM32F401CE WLCSP49/UFQFPN48 512Kb No   10
 * STM32F401RE LQFP64           512Kb Yes  16
 * STM32F401VE UFBGA100/LQFP100 512Kb Yes  16
 */

#elif defined(CONFIG_ARCH_CHIP_STM32F401CD) || defined(CONFIG_ARCH_CHIP_STM32F401RD) || \
      defined(CONFIG_ARCH_CHIP_STM32F401VD) || defined(CONFIG_ARCH_CHIP_STM32F401CE) || \
      defined(CONFIG_ARCH_CHIP_STM32F401RE) || defined(CONFIG_ARCH_CHIP_STM32F401VE)
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    1   /* One advanced timers TIM1 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define STM32_NGTIMNDMA                3   /* 16-bit general timers TIM9-11 without DMA */
#  define STM32_NBTIM                    0   /* No basic timers */
#  define STM32_NDMA                     2   /* DMA1-2 with 8 streams each*/
#  define STM32_NSPI                     4   /* SPI1-4 */
#  define STM32_NI2S                     2   /* I2S2-3 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   6   /* Actually only 3: USART1, 2 and 6 */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     0   /* No CAN */
#  if defined(CONFIG_ARCH_CHIP_STM32F401CD) || defined(CONFIG_ARCH_CHIP_STM32F401CE)
#    define STM32_NSDIO                  0   /* No SDIO interface */
#  else
#    define STM32_NSDIO                  1   /* One SDIO interface */
#  endif
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  1   /* USB OTG FS (only) */
#  define STM32_NGPIO                    50  /* GPIOA-H */
#  define STM32_NADC                     1   /* One 12-bit ADC1, 10 or 16 channels */
#  define STM32_NDAC                     0   /* No DAC */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* No CRC */
#  define STM32_NETHERNET                0   /* No Ethernet MAC */
#  define STM32_NRNG                     0   /* No Random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F410RB)  /* LQFP64 package, 512Kb FLASH, 96KiB SRAM */
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    1   /* One advanced timers TIM1 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define STM32_NGTIMNDMA                3   /* 16-bit general timers TIM9-11 without DMA */
#  define STM32_NBTIM                    0   /* No basic timers */
#  define STM32_NDMA                     2   /* DMA1-2 with 8 streams each*/
#  define STM32_NSPI                     3   /* SPI1-4 */
#  define STM32_NI2S                     0   /* I2S1-2 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   3   /* Actually only 3: USART1, 2 and 6 */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     0   /* No CAN */
#  define STM32_NSDIO                    0   /* One SDIO interface */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  0   /* USB OTG FS (only) */
#  define STM32_NGPIO                    50  /* GPIOA-H */
#  define STM32_NADC                     1   /* One 12-bit ADC1, 16 channels */
#  define STM32_NDAC                     1   /* 12-bit DAC1, 1 channel */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* No CRC */
#  define STM32_NETHERNET                0   /* No Ethernet MAC */
#  define STM32_NRNG                     1   /* No Random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F411CE)  /* LQFP64 package, 512Kb FLASH, 128KiB SRAM */
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    1   /* One advanced timers TIM1 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define STM32_NGTIMNDMA                3   /* 16-bit general timers TIM9-11 without DMA */
#  define STM32_NBTIM                    0   /* No basic timers */
#  define STM32_NDMA                     2   /* DMA1-2 with 8 streams each*/
#  define STM32_NSPI                     5   /* SPI1-5 */
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
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    1   /* One advanced timers TIM1 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define STM32_NGTIMNDMA                3   /* 16-bit general timers TIM9-11 without DMA */
#  define STM32_NBTIM                    0   /* No basic timers */
#  define STM32_NDMA                     2   /* DMA1-2 with 8 streams each*/
#  define STM32_NSPI                     5   /* SPI1-5 */
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

#elif defined(CONFIG_ARCH_CHIP_STM32F411VE)  /* 100 pin LQFP/BGA package, 512Kb FLASH, 128KiB SRAM */
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    1   /* One advanced timers TIM1 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define STM32_NGTIMNDMA                3   /* 16-bit general timers TIM9-11 without DMA */
#  define STM32_NBTIM                    0   /* No basic timers */
#  define STM32_NDMA                     2   /* DMA1-2 with 8 streams each*/
#  define STM32_NSPI                     5   /* SPI1-5 */
#  define STM32_NI2S                     2   /* I2S1-2 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   6   /* Actually only 3: USART1, 2 and 6 */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     0   /* No CAN */
#  define STM32_NSDIO                    1   /* One SDIO interface */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  1   /* USB OTG FS (only) */
#  define STM32_NGPIO                    81  /* GPIOA-H */
#  define STM32_NADC                     1   /* One 12-bit ADC1, 16 channels */
#  define STM32_NDAC                     0   /* No DAC */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* No CRC */
#  define STM32_NETHERNET                0   /* No Ethernet MAC */
#  define STM32_NRNG                     0   /* No Random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F412CE)  /* UFQFPN48 package, 512Kb FLASH, 256KiB SRAM */
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    2   /* Two advanced timers TIM1 and TIM8 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define STM32_NGTIMNDMA                4   /* 16-bit general timers 9, 12, 13, and 14 without DMA */
#  define STM32_NBTIM                    0   /* 2 basic timers TIM6 and TIM7 */
#  define STM32_NDMA                     2   /* DMA1-2 with 8 streams each*/
#  define STM32_NSPI                     5   /* SPI1-5 */
#  define STM32_NI2S                     3   /* I2S1-3 */
#  define STM32_NUSART                   4   /* USART1, 2, 3 and 6 */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     2   /* 2 CAN */
#  define STM32_NSDIO                    1   /* One SDIO interface */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  1   /* USB OTG FS (only) */
#  define STM32_NGPIO                    32  /* GPIOA-B */
#  define STM32_NADC                     1   /* One 12-bit ADC1, 16 channels */
#  define STM32_NDAC                     0   /* No DAC */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F412ZG)  /* 144 pin LQFP package, 1MB FLASH, 256KiB SRAM */
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    2   /* Two advanced timers TIM1 and TIM8 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define STM32_NGTIMNDMA                6   /* 16-bit general timers TIM9-14 without DMA */
#  define STM32_NBTIM                    2   /* 2 basic timers TIM6 and TIM7 */
#  define STM32_NDMA                     2   /* DMA1-2 with 8 streams each*/
#  define STM32_NSPI                     5   /* SPI1-5 */
#  define STM32_NI2S                     3   /* I2S1-3 */
#  define STM32_NUSART                   6   /* USART1, 2, 3 and 6 */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     2   /* 2 CAN */
#  define STM32_NSDIO                    1   /* One SDIO interface */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  1   /* USB OTG FS (only) */
#  define STM32_NGPIO                    113 /* GPIOA-H */
#  define STM32_NADC                     1   /* One 12-bit ADC1, 16 channels */
#  define STM32_NDAC                     0   /* No DAC */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F405RG)  /* LQFP 64 10x10x1.4 1024Kb FLASH 192Kb SRAM */
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
#  define STM32_NDAC                     2   /* 12-bit DAC1, 2 channels */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F405VG)  /* LQFP 100 14x14x1.4  1024Kb FLASH 192Kb SRAM */
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
#  define STM32_NDAC                     2   /* 12-bit DAC1, 2 channels */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F405ZG)  /* LQFP 144 20x20x1.4 1024Kb FLASH 192Kb SRAM */
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
#  define STM32_NDAC                     2   /* 12-bit DAC1, 2 channels */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F407VE)  /* LQFP-100 512Kb FLASH 192Kb SRAM */
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
#  define STM32_NDAC                     2   /* 12-bit DAC1, 2 channels */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                1   /* 100/100 Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    1   /* Digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F407VG)  /* LQFP-100 14x14x1.4 1024Kb FLASH 192Kb SRAM */
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
#  define STM32_NDAC                     1   /* 12-bit DAC1, 1 channel */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                1   /* 100/100 Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    1   /* Digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F407ZE)  /* LQFP-144 512Kb FLASH 192Kb SRAM */
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
#  define STM32_NDAC                     2   /* 12-bit DAC1, 2 channels */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                1   /* 100/100 Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    1   /* Digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F407ZG)  /* LQFP 144 20x20x1.4 1024Kb FLASH 192Kb SRAM */
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
#  define STM32_NDAC                     2   /* 12-bit DAC1, 2 channels */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                1   /* 100/100 Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    1   /* Digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F407IE)  /* LQFP 176 24x24x1.4 512Kb FLASH 192Kb SRAM */
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
#  define STM32_NDAC                     2   /* 12-bit DAC1, 2 channels */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                1   /* 100/100 Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    1   /* Digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F407IG)  /* BGA 176; LQFP 176 24x24x1.4 1024Kb FLASH 192Kb SRAM */
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
#  define STM32_NDAC                     2   /* 12-bit DAC1, 2 channels */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                1   /* 100/100 Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    1   /* Digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F427I)   /* BGA176; LQFP176 1024/2048KiB flash 256KiB SRAM */
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
#  define STM32_NDAC                     2   /* 12-bit DAC1, 2 channels */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                1   /* 100/100 Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    1   /* Digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F427Z)   /* LQFP144 1024/2048KiB flash 256KiB SRAM */
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
#  define STM32_NDAC                     2   /* 12-bit DAC1, 2 channels */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                1   /* 100/100 Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    1   /* Digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F427V)   /* LQFP100 1024/2048KiB flash 256KiB SRAM */
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
#  define STM32_NDAC                     2   /* 12-bit DAC1, 2 channels */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                1   /* 100/100 Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    1   /* Digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F429I)   /* BGA176; LQFP176 1024/2048KiB flash 256KiB SRAM */
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
#  define STM32_NDAC                     2   /* 12-bit DAC1, 2 channels */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                1   /* 100/100 Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    1   /* Digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F429Z)   /* LQFP144 1024/2048KiB flash 256KiB SRAM */
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
#  define STM32_NDAC                     2   /* 12-bit DAC1, 2 channels */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                1   /* 100/100 Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    1   /* Digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F429V)   /* LQFP100 1024/2048KiB flash 256KiB SRAM */
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
#  define STM32_NDAC                     2   /* 12-bit DAC1, 2 channels */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                1   /* 100/100 Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    1   /* Digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F446M)   /* WLCSP81 256/512KiB flash 128KiB SRAM */
#  define STM32_NFSMC                    0   /* FSMC */
#  define STM32_NATIM                    2   /* Two advanced timers TIM1 and 8 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define STM32_NGTIMNDMA                6   /* 16-bit general timers TIM9-14 without DMA */
#  define STM32_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     4   /* SPI1-4 */
#  define STM32_NI2S                     2   /* I2S1-2 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   6   /* USART1-3 and 6, UART 4-5 */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     2   /* CAN1-2 */
#  define STM32_NSDIO                    1   /* SDIO */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS */
#  define STM32_NGPIO                    114 /* GPIOA-I */
#  define STM32_NADC                     2   /* 12-bit ADC1-3, 14 channels */
#  define STM32_NDAC                     2   /* 12-bit DAC1, 2 channels */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* 100/100 Ethernet MAC */
#  define STM32_NRNG                     0   /* Random number generator (RNG) */
#  define STM32_NDCMI                    1   /* Digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F446R)   /* LQFP64 256/512KiB flash 128KiB SRAM */
#  define STM32_NFSMC                    0   /* FSMC */
#  define STM32_NATIM                    2   /* Two advanced timers TIM1 and 8 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define STM32_NGTIMNDMA                6   /* 16-bit general timers TIM9-14 without DMA */
#  define STM32_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     4   /* SPI1-4 */
#  define STM32_NI2S                     2   /* I2S1-2 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   6   /* USART1-3 and 6, UART 4-5 */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     2   /* CAN1-2 */
#  define STM32_NSDIO                    1   /* SDIO */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS */
#  define STM32_NGPIO                    114 /* GPIOA-I */
#  define STM32_NADC                     2   /* 12-bit ADC1-3, 16 channels */
#  define STM32_NDAC                     2   /* 12-bit DAC1, 2 channels */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* 100/100 Ethernet MAC */
#  define STM32_NRNG                     0   /* Random number generator (RNG) */
#  define STM32_NDCMI                    1   /* Digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F446V)   /* LQFP100 256/512KiB flash 128KiB SRAM */
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    2   /* Two advanced timers TIM1 and 8 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define STM32_NGTIMNDMA                6   /* 16-bit general timers TIM9-14 without DMA */
#  define STM32_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     4   /* SPI1-4 */
#  define STM32_NI2S                     2   /* I2S1-2 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   6   /* USART1-3 and 6, UART 4-5 */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     2   /* CAN1-2 */
#  define STM32_NSDIO                    1   /* SDIO */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS */
#  define STM32_NGPIO                    114 /* GPIOA-I */
#  define STM32_NADC                     2   /* 12-bit ADC1-3, 16 channels */
#  define STM32_NDAC                     2   /* 12-bit DAC1, 2 channels */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* 100/100 Ethernet MAC */
#  define STM32_NRNG                     0   /* Random number generator (RNG) */
#  define STM32_NDCMI                    1   /* Digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F446Z)   /* LQFP144 UFBGA144 256/512KiB flash 128KiB SRAM */
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    2   /* Two advanced timers TIM1 and 8 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define STM32_NGTIMNDMA                6   /* 16-bit general timers TIM9-14 without DMA */
#  define STM32_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     4   /* SPI1-4 */
#  define STM32_NI2S                     2   /* I2S1-2 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   6   /* USART1-3 and 6, UART 4-5 */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     2   /* CAN1-2 */
#  define STM32_NSDIO                    1   /* SDIO */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS */
#  define STM32_NGPIO                    114 /* GPIOA-I */
#  define STM32_NADC                     2   /* 12-bit ADC1-3, 16 channels */
#  define STM32_NDAC                     2   /* 12-bit DAC1, 2 channels */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* 100/100 Ethernet MAC */
#  define STM32_NRNG                     0   /* Random number generator (RNG) */
#  define STM32_NDCMI                    1   /* Digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F429N)   /* TFBGA216 1024/2048KiB flash 256KiB SRAM */
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
#  define STM32_NGPIO                    168 /* GPIOA-K */
#  define STM32_NADC                     3   /* 12-bit ADC1-3, 24 channels */
#  define STM32_NDAC                     2   /* 12-bit DAC1, 2 channels */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                1   /* 100/100 Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    1   /* Digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F469A) || \
      defined(CONFIG_ARCH_CHIP_STM32F469I) || \
      defined(CONFIG_ARCH_CHIP_STM32F469B) || \
      defined(CONFIG_ARCH_CHIP_STM32F469N)
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
#  define STM32_NLCD                     1   /* LCD */
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS */
#  if defined(CONFIG_ARCH_CHIP_STM32F469A)
#    define STM32_NGPIO                  114 /* GPIOA-I */
#  elif defined(CONFIG_ARCH_CHIP_STM32F469I)
#    define STM32_NGPIO                  131 /* GPIOA-I */
#  elif defined(CONFIG_ARCH_CHIP_STM32F469B) || \
        defined(CONFIG_ARCH_CHIP_STM32F469N)
#    define STM32_NGPIO                  161 /* GPIOA-K */
#  endif
#  define STM32_NADC                     3   /* 12-bit ADC1-3, 24 channels */
#  define STM32_NDAC                     2   /* 12-bit DAC1, 2 channels */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  if defined(CONFIG_ARCH_CHIP_STM32F469A)
#    define STM32_NETHERNET              0   /* No Ethernet MAC */
#  elif defined(CONFIG_ARCH_CHIP_STM32F469I) || \
        defined(CONFIG_ARCH_CHIP_STM32F469B) || \
        defined(CONFIG_ARCH_CHIP_STM32F469N)
#    define STM32_NETHERNET              1   /* 100/100 Ethernet MAC */
#  endif
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    1   /* Digital camera interface (DCMI) */

#elif defined (CONFIG_ARCH_CHIP_STM32G474C)
#  define STM32_NFSMC                    0   /* FSMC */
#  define STM32_NATIM                    3   /* (3) Advanced motor control timers TIM1, 8, and 20 with DMA */
#  define STM32_NGTIM                    7   /* (2) 16-bit general timers TIM3 and 4 with DMA
                                              * (2) 32-bit general timers TIM2 and 5 with DMA
                                              * (3) 16-bit general timers count-up timers with DMA: TIM15-17 */
#  define STM32_NGTIMNDMA                0   /* (0) 16-bit general timers TIM9-14 without DMA */
#  define STM32_NBTIM                    2   /* (2) Basic timers, TIM6-7 */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     2   /* I2S2-3 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   3   /* USART1-3 */
#  define STM32_NI2C                     4   /* I2C1-4 */
#  define STM32_NCAN                     3   /* FDCAN1-3 */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  0   /* No USB OTG FS/HS (but there is USB 2.0 full-speed
                                              * with LPM and BCD support) */
#  define STM32_NGPIO                    42  /* GPIOA-C, F-G */
#  define STM32_NADC                     5   /* 12-bit ADC1-5 */
#  define STM32_NDAC                     4   /* 12-bit DAC1-4, 7 channels (3 external, 4 internal) */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined (CONFIG_ARCH_CHIP_STM32G474M)
#  define STM32_NFSMC                    0   /* FSMC */
#  define STM32_NATIM                    3   /* (3) Advanced motor control timers TIM1, 8, and 20 with DMA */
#  define STM32_NGTIM                    7   /* (2) 16-bit general timers TIM3 and 4 with DMA
                                              * (2) 32-bit general timers TIM2 and 5 with DMA
                                              * (3) 16-bit general timers count-up timers with DMA: TIM15-17 */
#  define STM32_NGTIMNDMA                0   /* (0) 16-bit general timers TIM9-14 without DMA */
#  define STM32_NBTIM                    2   /* (2) Basic timers, TIM6-7 */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     4   /* SPI1-4 */
#  define STM32_NI2S                     2   /* I2S2-3 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   5   /* USART1-3 and UART 4-5 */
#  define STM32_NI2C                     4   /* I2C1-4 */
#  define STM32_NCAN                     3   /* FDCAN1-3 */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  0   /* No USB OTG FS/HS (but there is USB 2.0 full-speed
                                              * with LPM and BCD support) */
#  define STM32_NGPIO                    67  /* GPIOA-G */
#  define STM32_NADC                     5   /* 12-bit ADC1-5 */
#  define STM32_NDAC                     4   /* 12-bit DAC1-4, 7 channels (3 external, 4 internal) */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined (CONFIG_ARCH_CHIP_STM32G474R)
#  define STM32_NFSMC                    0   /* FSMC */
#  define STM32_NATIM                    3   /* (3) Advanced motor control timers TIM1, 8, and 20 with DMA */
#  define STM32_NGTIM                    7   /* (2) 16-bit general timers TIM3 and 4 with DMA
                                              * (2) 32-bit general timers TIM2 and 5 with DMA
                                              * (3) 16-bit general timers count-up timers with DMA: TIM15-17 */
#  define STM32_NGTIMNDMA                0   /* (0) 16-bit general timers TIM9-14 without DMA */
#  define STM32_NBTIM                    2   /* (2) Basic timers, TIM6-7 */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     2   /* I2S2-3 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   5   /* USART1-3 and UART 4-5 */
#  define STM32_NI2C                     4   /* I2C1-4 */
#  define STM32_NCAN                     3   /* FDCAN1-3 */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  0   /* No USB OTG FS/HS (but there is USB 2.0 full-speed
                                              * with LPM and BCD support) */
#  define STM32_NGPIO                    52  /* GPIOA-D, F-G */
#  define STM32_NADC                     5   /* 12-bit ADC1-5 */
#  define STM32_NDAC                     4   /* 12-bit DAC1-4, 7 channels (3 external, 4 internal) */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined (CONFIG_ARCH_CHIP_STM32G474Q)
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    3   /* (3) Advanced motor control timers TIM1, 8, and 20 with DMA */
#  define STM32_NGTIM                    7   /* (2) 16-bit general timers TIM3 and 4 with DMA
                                              * (2) 32-bit general timers TIM2 and 5 with DMA
                                              * (3) 16-bit general timers count-up timers with DMA: TIM15-17 */
#  define STM32_NGTIMNDMA                0   /* (0) 16-bit general timers TIM9-14 without DMA */
#  define STM32_NBTIM                    2   /* (2) Basic timers, TIM6-7 */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     4   /* SPI1-4 */
#  define STM32_NI2S                     2   /* I2S2-3 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   5   /* USART1-3 and UART 4-5 */
#  define STM32_NI2C                     4   /* I2C1-4 */
#  define STM32_NCAN                     3   /* FDCAN1-3 */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NLCD                     1   /* LCD parallel interface possible via FMC */
#  define STM32_NUSBOTG                  0   /* No USB OTG FS/HS (but there is USB 2.0 full-speed
                                              * with LPM and BCD support) */
#  define STM32_NGPIO                    107 /* GPIOA-G */
#  define STM32_NADC                     5   /* 12-bit ADC1-5 */
#  define STM32_NDAC                     4   /* 12-bit DAC1-4, 7 channels (3 external, 4 internal) */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined (CONFIG_ARCH_CHIP_STM32G474V)
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    3   /* (3) Advanced motor control timers TIM1, 8, and 20 with DMA */
#  define STM32_NGTIM                    7   /* (2) 16-bit general timers TIM3 and 4 with DMA
                                              * (2) 32-bit general timers TIM2 and 5 with DMA
                                              * (3) 16-bit general timers count-up timers with DMA: TIM15-17 */
#  define STM32_NGTIMNDMA                0   /* (0) 16-bit general timers TIM9-14 without DMA */
#  define STM32_NBTIM                    2   /* (2) Basic timers, TIM6-7 */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     4   /* SPI1-4 */
#  define STM32_NI2S                     2   /* I2S2-3 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   5   /* USART1-3 and UART 4-5 */
#  define STM32_NI2C                     4   /* I2C1-4 */
#  define STM32_NCAN                     3   /* FDCAN1-3 */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NLCD                     1   /* LCD parallel interface possible via FMC */
#  define STM32_NUSBOTG                  0   /* No USB OTG FS/HS (but there is USB 2.0 full-speed
                                              * with LPM and BCD support) */
#  define STM32_NGPIO                    86  /* GPIOA-G */
#  define STM32_NADC                     5   /* 12-bit ADC1-5 */
#  define STM32_NDAC                     4   /* 12-bit DAC1-4, 7 channels (3 external, 4 internal) */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#else
#  error "Unsupported STM32 chip"
#endif

/* NVIC priority levels ***************************************************************************/

#define NVIC_SYSH_PRIORITY_MIN     0xf0 /* All bits set in minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT 0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX     0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP    0x10 /* Four bits of interrupt priority used */

#endif /* __ARCH_ARM_INCLUDE_STM32_CHIP_H */
