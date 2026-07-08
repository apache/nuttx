/****************************************************************************
 * arch/arm/include/stm32f4/chip.h
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

#ifndef __ARCH_ARM_INCLUDE_STM32F4_CHIP_H
#define __ARCH_ARM_INCLUDE_STM32F4_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

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

#if defined(CONFIG_ARCH_CHIP_STM32F401CB) || defined(CONFIG_ARCH_CHIP_STM32F401RB) || \
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
#  define STM32_NUSART                   6   /* Actually only 3: USART1, 2 and 6 */
#  define STM32_NLPUART                  0   /* No LPUART */
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
#  define STM32_NLPUART                  0   /* No LPUART */
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
#  define STM32_NLPUART                  0   /* No LPUART */
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
#  define STM32_NLPUART                  0   /* No LPUART */
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
#  define STM32_NLPUART                  0   /* No LPUART */
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
#  define STM32_NLPUART                  0   /* No LPUART */
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
#  define STM32_NLPUART                  0   /* No LPUART */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     2   /* 2 CAN */
#  define STM32_NSDIO                    1   /* One SDIO interface */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  1   /* USB OTG FS (only) */
#  define STM32_NGPIO                    34  /* GPIOA-B (sans PB11) and 3 Bits of C */
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
#  define STM32_NLPUART                  0   /* No LPUART */
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
#  define STM32_NLPUART                  0   /* No LPUART */
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
#  define STM32_NLPUART                  0   /* No LPUART */
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
#  define STM32_NLPUART                  0   /* No LPUART */
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
#  define STM32_NLPUART                  0   /* No LPUART */
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
#  define STM32_NLPUART                  0   /* No LPUART */
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
#  define STM32_NLPUART                  0   /* No LPUART */
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
#  define STM32_NLPUART                  0   /* No LPUART */
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
#  define STM32_NLPUART                  0   /* No LPUART */
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
#  define STM32_NLPUART                  0   /* No LPUART */
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
#  define STM32_NLPUART                  0   /* No LPUART */
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
#  define STM32_NLPUART                  0   /* No LPUART */
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
#  define STM32_NLPUART                  0   /* No LPUART */
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
#  define STM32_NLPUART                  0   /* No LPUART */
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
#  define STM32_NLPUART                  0   /* No LPUART */
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
#  define STM32_NLPUART                  0   /* No LPUART */
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
#  define STM32_NLPUART                  0   /* No LPUART */
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
#  define STM32_NLPUART                  0   /* No LPUART */
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
#  define STM32_NLPUART                  0   /* No LPUART */
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
#  define STM32_NLPUART                  0   /* No LPUART */
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
#  define STM32_NLPUART                  0   /* No LPUART */
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
#  define STM32_NLPUART                  0   /* No LPUART */
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

#else
#  error "Unsupported STM32 chip"
#endif

/* Peripheral IP versions ***************************************************/

/* Peripheral IP versions are invariant and should be decided here, not in
 * Kconfig.
 *
 * REVISIT: Currently only SPI IP version is handled here, with others being
 *          handled in Kconfig. Those others need to be gradually refactored
 *          and resolved here.
 */

#define STM32_HAVE_IP_SPI_V2

/* NVIC priority levels *****************************************************/

#define NVIC_SYSH_PRIORITY_MIN     0xf0 /* All bits set in minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT 0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX     0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP    0x10 /* Four bits of interrupt priority used */

#endif /* __ARCH_ARM_INCLUDE_STM32F4_CHIP_H */
