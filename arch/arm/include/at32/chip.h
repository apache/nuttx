/****************************************************************************
 * arch/arm/include/at32/chip.h
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

#ifndef __ARCH_ARM_INCLUDE_AT32_CHIP_H
#define __ARCH_ARM_INCLUDE_AT32_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* Check the AT32 family configuration.
 * It must be done in arch/arm/src/at32/Kconfig !
 */

#ifdef CONFIG_AT32_AT32F43XX
#  define __HAVE_F4  1
#else
#  define __HAVE_F4  0
#endif

#if ((__HAVE_F1 + __HAVE_F2 + __HAVE_F30 + __HAVE_F33 + __HAVE_F37 + __HAVE_F4 + \
      __HAVE_G4 + __HAVE_L1) != 1)
#  error "Only one AT32 family must be selected !"
#endif

#ifdef CONFIG_AT32_LOWDENSITY
#  define __HAVE_LD  1
#else
#  define __HAVE_LD  0
#endif
#ifdef CONFIG_AT32_MEDIUMDENSITY
#  define __HAVE_MD  1
#else
#  define __HAVE_MD  0
#endif
#ifdef CONFIG_AT32_MEDIUMPLUSDENSITY
#  define __HAVE_MPD 1
#else
#  define __HAVE_MPD 0
#endif
#ifdef CONFIG_AT32_HIGHDENSITY
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
 * particular pin-mapping will depend on the package and AT32 family.  If
 * you are incorporating a new AT32 chip into NuttX, you will need to add
 * the pin-mapping to a header file and to include that header file below.
 * The chip-specific pin-mapping is defined in the chip datasheet.
 */

/* AT32 F4 Family ***********************************************************/
#if defined(CONFIG_ARCH_CHIP_AT32F435RC)  /* LQFP 64 10x10x1.4 256Kb FLASH 384Kb SRAM */
#  define AT32_NFSMC                    0   /* No FSMC */
#  define AT32_NATIM                    3   /* 3 advanced timers TIM1 and 8 */
#  define AT32_NGTIM                    2   /* 32-bit general timers TIM2 and 5 with DMA */
#  define AT32_NGTIMNDMA                8   /* 16-bit general timers TIM9-14 without DMA */
#  define AT32_NBTIM                    2   /* 2 basic timers, TIM6-7 */
#  define AT32_NDMA                     2   /* DMA1-2 */
#  define AT32_NSPI                     4   /* SPI1-4 */
#  define AT32_NI2S                     4   /* I2S1-2 (multiplexed with SPI2-3) */
#  define AT32_NUSART                   8   /* USART1-3 and 6, UART 4-5 */
#  define AT32_NI2C                     3   /* I2C1-3 */
#  define AT32_NCAN                     2   /* CAN1-2 */
#  define AT32_NSDIO                    2   /* SDIO */
#  define AT32_NLCD                     0   /* No LCD */
#  define AT32_NUSBOTG                  2   /* USB OTG FS/HS */
#  define AT32_NGPIO                    53  /* GPIOA-I */
#  define AT32_NADC                     3   /* 12-bit ADC1-3, 16 channels */
#  define AT32_NDAC                     2   /* 12-bit DAC1, 2 channels */
#  define AT32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define AT32_NCRC                     1   /* CRC */
#  define AT32_NETHERNET                0   /* No Ethernet MAC */
#  define AT32_NRNG                     0   /* Random number generator (RNG) */
#  define AT32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_AT32F435RG)  /* LQFP 64 10x10x1.4 1024Kb FLASH 384Kb SRAM */
#  define AT32_NFSMC                    0   /* No FSMC */
#  define AT32_NATIM                    2   /* Two advanced timers TIM1 and 8 */
#  define AT32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define AT32_NGTIMNDMA                6   /* 16-bit general timers TIM9-14 without DMA */
#  define AT32_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define AT32_NDMA                     2   /* DMA1-2 */
#  define AT32_NSPI                     3   /* SPI1-3 */
#  define AT32_NI2S                     2   /* I2S1-2 (multiplexed with SPI2-3) */
#  define AT32_NUSART                   6   /* USART1-3 and 6, UART 4-5 */
#  define AT32_NI2C                     3   /* I2C1-3 */
#  define AT32_NCAN                     2   /* CAN1-2 */
#  define AT32_NSDIO                    1   /* SDIO */
#  define AT32_NLCD                     0   /* No LCD */
#  define AT32_NUSBOTG                  1   /* USB OTG FS/HS */
#  define AT32_NGPIO                    53  /* GPIOA-I */
#  define AT32_NADC                     3   /* 12-bit ADC1-3, 16 channels */
#  define AT32_NDAC                     2   /* 12-bit DAC1, 2 channels */
#  define AT32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define AT32_NCRC                     1   /* CRC */
#  define AT32_NETHERNET                0   /* No Ethernet MAC */
#  define AT32_NRNG                     1   /* Random number generator (RNG) */
#  define AT32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_AT32F435RM)  /* LQFP 64 10x10x1.4 4032Kb FLASH 384Kb SRAM */
#  define AT32_NFSMC                    0   /* No FSMC */
#  define AT32_NATIM                    2   /* Two advanced timers TIM1 and 8 */
#  define AT32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define AT32_NGTIMNDMA                6   /* 16-bit general timers TIM9-14 without DMA */
#  define AT32_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define AT32_NDMA                     2   /* DMA1-2 */
#  define AT32_NSPI                     3   /* SPI1-3 */
#  define AT32_NI2S                     2   /* I2S1-2 (multiplexed with SPI2-3) */
#  define AT32_NUSART                   6   /* USART1-3 and 6, UART 4-5 */
#  define AT32_NI2C                     3   /* I2C1-3 */
#  define AT32_NCAN                     2   /* CAN1-2 */
#  define AT32_NSDIO                    1   /* SDIO */
#  define AT32_NLCD                     0   /* No LCD */
#  define AT32_NUSBOTG                  1   /* USB OTG FS/HS */
#  define AT32_NGPIO                    53  /* GPIOA-I */
#  define AT32_NADC                     3   /* 12-bit ADC1-3, 16 channels */
#  define AT32_NDAC                     2   /* 12-bit DAC1, 2 channels */
#  define AT32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define AT32_NCRC                     1   /* CRC */
#  define AT32_NETHERNET                0   /* No Ethernet MAC */
#  define AT32_NRNG                     1   /* Random number generator (RNG) */
#  define AT32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_AT32F435VC)  /* LQFP 100 14x14x1.4  256Kb FLASH 384Kb SRAM */
#  define AT32_NFSMC                    1   /* FSMC */
#  define AT32_NATIM                    2   /* Two advanced timers TIM1 and 8 */
#  define AT32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define AT32_NGTIMNDMA                6   /* 16-bit general timers TIM9-14 without DMA */
#  define AT32_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define AT32_NDMA                     2   /* DMA1-2 */
#  define AT32_NSPI                     3   /* SPI1-3 */
#  define AT32_NI2S                     2   /* I2S1-2 (multiplexed with SPI2-3) */
#  define AT32_NUSART                   6   /* USART1-3 and 6, UART 4-5 */
#  define AT32_NI2C                     3   /* I2C1-3 */
#  define AT32_NCAN                     2   /* CAN1-2 */
#  define AT32_NSDIO                    1   /* SDIO */
#  define AT32_NLCD                     0   /* No LCD */
#  define AT32_NUSBOTG                  1   /* USB OTG FS/HS */
#  define AT32_NGPIO                    84  /* GPIOA-I */
#  define AT32_NADC                     3   /* 12-bit ADC1-3, 16 channels */
#  define AT32_NDAC                     2   /* 12-bit DAC1, 2 channels */
#  define AT32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define AT32_NCRC                     1   /* CRC */
#  define AT32_NETHERNET                0   /* No Ethernet MAC */
#  define AT32_NRNG                     1   /* Random number generator (RNG) */
#  define AT32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_AT32F435VG)  /* LQFP 100 14x14x1.4  1024Kb FLASH 384Kb SRAM */
#  define AT32_NFSMC                    1   /* FSMC */
#  define AT32_NATIM                    2   /* Two advanced timers TIM1 and 8 */
#  define AT32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define AT32_NGTIMNDMA                6   /* 16-bit general timers TIM9-14 without DMA */
#  define AT32_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define AT32_NDMA                     2   /* DMA1-2 */
#  define AT32_NSPI                     3   /* SPI1-3 */
#  define AT32_NI2S                     2   /* I2S1-2 (multiplexed with SPI2-3) */
#  define AT32_NUSART                   6   /* USART1-3 and 6, UART 4-5 */
#  define AT32_NI2C                     3   /* I2C1-3 */
#  define AT32_NCAN                     2   /* CAN1-2 */
#  define AT32_NSDIO                    1   /* SDIO */
#  define AT32_NLCD                     0   /* No LCD */
#  define AT32_NUSBOTG                  1   /* USB OTG FS/HS */
#  define AT32_NGPIO                    84  /* GPIOA-I */
#  define AT32_NADC                     3   /* 12-bit ADC1-3, 16 channels */
#  define AT32_NDAC                     2   /* 12-bit DAC1, 2 channels */
#  define AT32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define AT32_NCRC                     1   /* CRC */
#  define AT32_NETHERNET                0   /* No Ethernet MAC */
#  define AT32_NRNG                     1   /* Random number generator (RNG) */
#  define AT32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_AT32F435VM)  /* LQFP 100 14x14x1.4  4032Kb FLASH 384Kb SRAM */
#  define AT32_NFSMC                    1   /* FSMC */
#  define AT32_NATIM                    2   /* Two advanced timers TIM1 and 8 */
#  define AT32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define AT32_NGTIMNDMA                6   /* 16-bit general timers TIM9-14 without DMA */
#  define AT32_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define AT32_NDMA                     2   /* DMA1-2 */
#  define AT32_NSPI                     3   /* SPI1-3 */
#  define AT32_NI2S                     2   /* I2S1-2 (multiplexed with SPI2-3) */
#  define AT32_NUSART                   6   /* USART1-3 and 6, UART 4-5 */
#  define AT32_NI2C                     3   /* I2C1-3 */
#  define AT32_NCAN                     2   /* CAN1-2 */
#  define AT32_NSDIO                    1   /* SDIO */
#  define AT32_NLCD                     0   /* No LCD */
#  define AT32_NUSBOTG                  1   /* USB OTG FS/HS */
#  define AT32_NGPIO                    84  /* GPIOA-I */
#  define AT32_NADC                     3   /* 12-bit ADC1-3, 16 channels */
#  define AT32_NDAC                     2   /* 12-bit DAC1, 2 channels */
#  define AT32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define AT32_NCRC                     1   /* CRC */
#  define AT32_NETHERNET                0   /* No Ethernet MAC */
#  define AT32_NRNG                     1   /* Random number generator (RNG) */
#  define AT32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_AT32F435ZC)  /* LQFP 144 20x20x1.4 256Kb FLASH 384Kb SRAM */
#  define AT32_NFSMC                    1   /* FSMC */
#  define AT32_NATIM                    2   /* Two advanced timers TIM1 and 8 */
#  define AT32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define AT32_NGTIMNDMA                6   /* 16-bit general timers TIM9-14 without DMA */
#  define AT32_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define AT32_NDMA                     2   /* DMA1-2 */
#  define AT32_NSPI                     3   /* SPI1-3 */
#  define AT32_NI2S                     2   /* I2S1-2 (multiplexed with SPI2-3) */
#  define AT32_NUSART                   6   /* USART1-3 and 6, UART 4-5 */
#  define AT32_NI2C                     3   /* I2C1-3 */
#  define AT32_NCAN                     2   /* CAN1-2 */
#  define AT32_NSDIO                    1   /* SDIO */
#  define AT32_NLCD                     0   /* No LCD */
#  define AT32_NUSBOTG                  1   /* USB OTG FS/HS */
#  define AT32_NGPIO                    116 /* GPIOA-I */
#  define AT32_NADC                     3   /* 12-bit ADC1-3, 24 channels */
#  define AT32_NDAC                     2   /* 12-bit DAC1, 2 channels */
#  define AT32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define AT32_NCRC                     1   /* CRC */
#  define AT32_NETHERNET                0   /* No Ethernet MAC */
#  define AT32_NRNG                     1   /* Random number generator (RNG) */
#  define AT32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_AT32F435ZG)  /* LQFP 144 20x20x1.4 1024Kb FLASH 384Kb SRAM */
#  define AT32_NFSMC                    1   /* FSMC */
#  define AT32_NATIM                    2   /* Two advanced timers TIM1 and 8 */
#  define AT32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define AT32_NGTIMNDMA                6   /* 16-bit general timers TIM9-14 without DMA */
#  define AT32_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define AT32_NDMA                     2   /* DMA1-2 */
#  define AT32_NSPI                     3   /* SPI1-3 */
#  define AT32_NI2S                     2   /* I2S1-2 (multiplexed with SPI2-3) */
#  define AT32_NUSART                   6   /* USART1-3 and 6, UART 4-5 */
#  define AT32_NI2C                     3   /* I2C1-3 */
#  define AT32_NCAN                     2   /* CAN1-2 */
#  define AT32_NSDIO                    1   /* SDIO */
#  define AT32_NLCD                     0   /* No LCD */
#  define AT32_NUSBOTG                  1   /* USB OTG FS/HS */
#  define AT32_NGPIO                    116 /* GPIOA-I */
#  define AT32_NADC                     3   /* 12-bit ADC1-3, 24 channels */
#  define AT32_NDAC                     2   /* 12-bit DAC1, 2 channels */
#  define AT32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define AT32_NCRC                     1   /* CRC */
#  define AT32_NETHERNET                0   /* No Ethernet MAC */
#  define AT32_NRNG                     1   /* Random number generator (RNG) */
#  define AT32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_AT32F435ZM)  /* LQFP 144 20x20x1.4 4032Kb FLASH 384Kb SRAM */
#  define AT32_NFSMC                    1   /* FSMC */
#  define AT32_NATIM                    2   /* Two advanced timers TIM1 and 8 */
#  define AT32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define AT32_NGTIMNDMA                6   /* 16-bit general timers TIM9-14 without DMA */
#  define AT32_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define AT32_NDMA                     2   /* DMA1-2 */
#  define AT32_NSPI                     3   /* SPI1-3 */
#  define AT32_NI2S                     2   /* I2S1-2 (multiplexed with SPI2-3) */
#  define AT32_NUSART                   6   /* USART1-3 and 6, UART 4-5 */
#  define AT32_NI2C                     3   /* I2C1-3 */
#  define AT32_NCAN                     2   /* CAN1-2 */
#  define AT32_NSDIO                    1   /* SDIO */
#  define AT32_NLCD                     0   /* No LCD */
#  define AT32_NUSBOTG                  1   /* USB OTG FS/HS */
#  define AT32_NGPIO                    116 /* GPIOA-I */
#  define AT32_NADC                     3   /* 12-bit ADC1-3, 24 channels */
#  define AT32_NDAC                     2   /* 12-bit DAC1, 2 channels */
#  define AT32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define AT32_NCRC                     1   /* CRC */
#  define AT32_NETHERNET                0   /* No Ethernet MAC */
#  define AT32_NRNG                     1   /* Random number generator (RNG) */
#  define AT32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_AT32F437RC)  /* LQFP 64 10x10x1.4 256Kb FLASH 384Kb SRAM */
#  define AT32_NFSMC                    0   /* No FSMC */
#  define AT32_NATIM                    2   /* Two advanced timers TIM1 and 8 */
#  define AT32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define AT32_NGTIMNDMA                6   /* 16-bit general timers TIM9-14 without DMA */
#  define AT32_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define AT32_NDMA                     2   /* DMA1-2 */
#  define AT32_NSPI                     3   /* SPI1-3 */
#  define AT32_NI2S                     2   /* I2S1-2 (multiplexed with SPI2-3) */
#  define AT32_NUSART                   6   /* USART1-3 and 6, UART 4-5 */
#  define AT32_NI2C                     3   /* I2C1-3 */
#  define AT32_NCAN                     2   /* CAN1-2 */
#  define AT32_NSDIO                    1   /* SDIO */
#  define AT32_NLCD                     0   /* No LCD */
#  define AT32_NUSBOTG                  1   /* USB OTG FS/HS */
#  define AT32_NGPIO                    53  /* GPIOA-I */
#  define AT32_NADC                     3   /* 12-bit ADC1-3, 16 channels */
#  define AT32_NDAC                     2   /* 12-bit DAC1, 2 channels */
#  define AT32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define AT32_NCRC                     1   /* CRC */
#  define AT32_NETHERNET                0   /* No Ethernet MAC */
#  define AT32_NRNG                     1   /* Random number generator (RNG) */
#  define AT32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_AT32F437RG)  /* LQFP 64 10x10x1.4 1024Kb FLASH 384Kb SRAM */
#  define AT32_NFSMC                    0   /* No FSMC */
#  define AT32_NATIM                    2   /* Two advanced timers TIM1 and 8 */
#  define AT32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define AT32_NGTIMNDMA                6   /* 16-bit general timers TIM9-14 without DMA */
#  define AT32_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define AT32_NDMA                     2   /* DMA1-2 */
#  define AT32_NSPI                     3   /* SPI1-3 */
#  define AT32_NI2S                     2   /* I2S1-2 (multiplexed with SPI2-3) */
#  define AT32_NUSART                   6   /* USART1-3 and 6, UART 4-5 */
#  define AT32_NI2C                     3   /* I2C1-3 */
#  define AT32_NCAN                     2   /* CAN1-2 */
#  define AT32_NSDIO                    1   /* SDIO */
#  define AT32_NLCD                     0   /* No LCD */
#  define AT32_NUSBOTG                  1   /* USB OTG FS/HS */
#  define AT32_NGPIO                    53  /* GPIOA-I */
#  define AT32_NADC                     3   /* 12-bit ADC1-3, 16 channels */
#  define AT32_NDAC                     2   /* 12-bit DAC1, 2 channels */
#  define AT32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define AT32_NCRC                     1   /* CRC */
#  define AT32_NETHERNET                0   /* No Ethernet MAC */
#  define AT32_NRNG                     1   /* Random number generator (RNG) */
#  define AT32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_AT32F437RM)  /* LQFP 64 10x10x1.4 4032Kb FLASH 384Kb SRAM */
#  define AT32_NFSMC                    0   /* No FSMC */
#  define AT32_NATIM                    2   /* Two advanced timers TIM1 and 8 */
#  define AT32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define AT32_NGTIMNDMA                6   /* 16-bit general timers TIM9-14 without DMA */
#  define AT32_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define AT32_NDMA                     2   /* DMA1-2 */
#  define AT32_NSPI                     3   /* SPI1-3 */
#  define AT32_NI2S                     2   /* I2S1-2 (multiplexed with SPI2-3) */
#  define AT32_NUSART                   6   /* USART1-3 and 6, UART 4-5 */
#  define AT32_NI2C                     3   /* I2C1-3 */
#  define AT32_NCAN                     2   /* CAN1-2 */
#  define AT32_NSDIO                    1   /* SDIO */
#  define AT32_NLCD                     0   /* No LCD */
#  define AT32_NUSBOTG                  1   /* USB OTG FS/HS */
#  define AT32_NGPIO                    53  /* GPIOA-I */
#  define AT32_NADC                     3   /* 12-bit ADC1-3, 16 channels */
#  define AT32_NDAC                     2   /* 12-bit DAC1, 2 channels */
#  define AT32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define AT32_NCRC                     1   /* CRC */
#  define AT32_NETHERNET                0   /* No Ethernet MAC */
#  define AT32_NRNG                     1   /* Random number generator (RNG) */
#  define AT32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_AT32F437VC)  /* LQFP-100 256Kb FLASH 384Kb SRAM */
#  define AT32_NFSMC                    1   /* FSMC */
#  define AT32_NATIM                    2   /* Two advanced timers TIM1 and 8 */
#  define AT32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define AT32_NGTIMNDMA                6   /* 16-bit general timers TIM9-14 without DMA */
#  define AT32_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define AT32_NDMA                     2   /* DMA1-2 */
#  define AT32_NSPI                     3   /* SPI1-3 */
#  define AT32_NI2S                     2   /* I2S1-2 (multiplexed with SPI2-3) */
#  define AT32_NUSART                   6   /* USART1-3 and 6, UART 4-5 */
#  define AT32_NI2C                     3   /* I2C1-3 */
#  define AT32_NCAN                     2   /* CAN1-2 */
#  define AT32_NSDIO                    1   /* SDIO */
#  define AT32_NLCD                     0   /* No LCD */
#  define AT32_NUSBOTG                  1   /* USB OTG FS/HS */
#  define AT32_NGPIO                    84  /* GPIOA-I */
#  define AT32_NADC                     3   /* 12-bit ADC1-3, 16 channels */
#  define AT32_NDAC                     2   /* 12-bit DAC1, 2 channels */
#  define AT32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define AT32_NCRC                     1   /* CRC */
#  define AT32_NETHERNET                1   /* 100/100 Ethernet MAC */
#  define AT32_NRNG                     1   /* Random number generator (RNG) */
#  define AT32_NDCMI                    1   /* Digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_AT32F437VG)  /* LQFP-100 14x14x1.4 1024Kb FLASH 384Kb SRAM */
#  define AT32_NFSMC                    1   /* FSMC */
#  define AT32_NATIM                    3   /* Two advanced timers TIM1 and 8 */
#  define AT32_NGTIM                    2   /* 32-bit general timers TIM2 and 5 with DMA */
#  define AT32_NGTIMNDMA                8   /* 16-bit general timers TIM9-14 without DMA */
#  define AT32_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define AT32_NDMA                     2   /* DMA1-2 */
#  define AT32_NSPI                     4   /* SPI1-3 */
#  define AT32_NI2S                     4   /* I2S1-2 (multiplexed with SPI2-3) */
#  define AT32_NUSART                   8   /* USART1-3 and 6, UART 4-5 */
#  define AT32_NI2C                     3   /* I2C1-3 */
#  define AT32_NCAN                     2   /* CAN1-2 */
#  define AT32_NSDIO                    2   /* SDIO */
#  define AT32_NLCD                     0   /* No LCD */
#  define AT32_NUSBOTG                  2   /* USB OTG FS/HS */
#  define AT32_NGPIO                    84  /* GPIOA-I */
#  define AT32_NADC                     3   /* 12-bit ADC1-3, 16 channels */
#  define AT32_NDAC                     1   /* 12-bit DAC1, 1 channel */
#  define AT32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define AT32_NCRC                     1   /* CRC */
#  define AT32_NETHERNET                1   /* 100/100 Ethernet MAC */
#  define AT32_NRNG                     0   /* Random number generator (RNG) */
#  define AT32_NDCMI                    1   /* Digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_AT32F437VM)  /* LQFP-100 14x14x1.4 4032Kb FLASH 384Kb SRAM */
#  define AT32_NFSMC                    1   /* FSMC */
#  define AT32_NATIM                    3   /* Two advanced timers TIM1 and 8 */
#  define AT32_NGTIM                    2   /* 32-bit general timers TIM2 and 5 with DMA */
#  define AT32_NGTIMNDMA                8   /* 16-bit general timers TIM9-14 without DMA */
#  define AT32_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define AT32_NDMA                     2   /* DMA1-2 */
#  define AT32_NSPI                     4   /* SPI1-3 */
#  define AT32_NI2S                     4   /* I2S1-2 (multiplexed with SPI2-3) */
#  define AT32_NUSART                   8   /* USART1-3 and 6, UART 4-5 */
#  define AT32_NI2C                     3   /* I2C1-3 */
#  define AT32_NCAN                     2   /* CAN1-2 */
#  define AT32_NSDIO                    2   /* SDIO */
#  define AT32_NLCD                     0   /* No LCD */
#  define AT32_NUSBOTG                  2   /* USB OTG FS/HS */
#  define AT32_NGPIO                    84  /* GPIOA-I */
#  define AT32_NADC                     3   /* 12-bit ADC1-3, 16 channels */
#  define AT32_NDAC                     1   /* 12-bit DAC1, 1 channel */
#  define AT32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define AT32_NCRC                     1   /* CRC */
#  define AT32_NETHERNET                1   /* 100/100 Ethernet MAC */
#  define AT32_NRNG                     0   /* Random number generator (RNG) */
#  define AT32_NDCMI                    1   /* Digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_AT32F437ZC)  /* LQFP-144 256Kb FLASH 384Kb SRAM */
#  define AT32_NFSMC                    1   /* FSMC */
#  define AT32_NATIM                    2   /* Two advanced timers TIM1 and 8 */
#  define AT32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define AT32_NGTIMNDMA                6   /* 16-bit general timers TIM9-14 without DMA */
#  define AT32_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define AT32_NDMA                     2   /* DMA1-2 */
#  define AT32_NSPI                     3   /* SPI1-3 */
#  define AT32_NI2S                     2   /* I2S1-2 (multiplexed with SPI2-3) */
#  define AT32_NUSART                   6   /* USART1-3 and 6, UART 4-5 */
#  define AT32_NI2C                     3   /* I2C1-3 */
#  define AT32_NCAN                     2   /* CAN1-2 */
#  define AT32_NSDIO                    1   /* SDIO */
#  define AT32_NLCD                     0   /* No LCD */
#  define AT32_NUSBOTG                  1   /* USB OTG FS/HS */
#  define AT32_NGPIO                    116 /* GPIOA-I */
#  define AT32_NADC                     3   /* 12-bit ADC1-3, 24 channels */
#  define AT32_NDAC                     2   /* 12-bit DAC1, 2 channels */
#  define AT32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define AT32_NCRC                     1   /* CRC */
#  define AT32_NETHERNET                1   /* 100/100 Ethernet MAC */
#  define AT32_NRNG                     1   /* Random number generator (RNG) */
#  define AT32_NDCMI                    1   /* Digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_AT32F437ZG)  /* LQFP 144 20x20x1.4 1024Kb FLASH 384Kb SRAM */
#  define AT32_NFSMC                    1   /* FSMC */
#  define AT32_NATIM                    2   /* Two advanced timers TIM1 and 8 */
#  define AT32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define AT32_NGTIMNDMA                6   /* 16-bit general timers TIM9-14 without DMA */
#  define AT32_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define AT32_NDMA                     2   /* DMA1-2 */
#  define AT32_NSPI                     3   /* SPI1-3 */
#  define AT32_NI2S                     2   /* I2S1-2 (multiplexed with SPI2-3) */
#  define AT32_NUSART                   6   /* USART1-3 and 6, UART 4-5 */
#  define AT32_NI2C                     3   /* I2C1-3 */
#  define AT32_NCAN                     2   /* CAN1-2 */
#  define AT32_NSDIO                    1   /* SDIO */
#  define AT32_NLCD                     0   /* No LCD */
#  define AT32_NUSBOTG                  1   /* USB OTG FS/HS */
#  define AT32_NGPIO                    116 /* GPIOA-I */
#  define AT32_NADC                     3   /* 12-bit ADC1-3, 24 channels */
#  define AT32_NDAC                     2   /* 12-bit DAC1, 2 channels */
#  define AT32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define AT32_NCRC                     1   /* CRC */
#  define AT32_NETHERNET                1   /* 100/100 Ethernet MAC */
#  define AT32_NRNG                     1   /* Random number generator (RNG) */
#  define AT32_NDCMI                    1   /* Digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_AT32F437ZM)  /* LQFP 144 20x20x1.4 4032Kb FLASH 384Kb SRAM */
#  define AT32_NFSMC                    1   /* FSMC */
#  define AT32_NATIM                    2   /* Two advanced timers TIM1 and 8 */
#  define AT32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define AT32_NGTIMNDMA                6   /* 16-bit general timers TIM9-14 without DMA */
#  define AT32_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define AT32_NDMA                     2   /* DMA1-2 */
#  define AT32_NSPI                     3   /* SPI1-3 */
#  define AT32_NI2S                     2   /* I2S1-2 (multiplexed with SPI2-3) */
#  define AT32_NUSART                   6   /* USART1-3 and 6, UART 4-5 */
#  define AT32_NI2C                     3   /* I2C1-3 */
#  define AT32_NCAN                     2   /* CAN1-2 */
#  define AT32_NSDIO                    1   /* SDIO */
#  define AT32_NLCD                     0   /* No LCD */
#  define AT32_NUSBOTG                  1   /* USB OTG FS/HS */
#  define AT32_NGPIO                    116 /* GPIOA-I */
#  define AT32_NADC                     3   /* 12-bit ADC1-3, 24 channels */
#  define AT32_NDAC                     2   /* 12-bit DAC1, 2 channels */
#  define AT32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define AT32_NCRC                     1   /* CRC */
#  define AT32_NETHERNET                1   /* 100/100 Ethernet MAC */
#  define AT32_NRNG                     1   /* Random number generator (RNG) */
#  define AT32_NDCMI                    1   /* Digital camera interface (DCMI) */

#else
#  error "Unsupported AT32 chip"
#endif

/* Peripheral IP versions ***************************************************/

/* Peripheral IP versions are invariant and should be decided here, not in
 * Kconfig.
 *
 * REVISIT: Currently only SPI IP version is handled here, with others being
 *          handled in Kconfig. Those others need to be gradually refactored
 *          and resolved here.
 */

#if defined(CONFIG_AT32_AT32F43XX)
#  define AT32_HAVE_IP_SPI_V2
#else
#  error "Did not resolve peripheral IP versions!"
#endif

/* NVIC priority levels *****************************************************/

#define NVIC_SYSH_PRIORITY_MIN     0xf0 /* All bits set in minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT 0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX     0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP    0x10 /* Four bits of interrupt priority used */

#endif /* __ARCH_ARM_INCLUDE_AT32_CHIP_H */
