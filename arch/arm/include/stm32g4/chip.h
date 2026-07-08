/****************************************************************************
 * arch/arm/include/stm32g4/chip.h
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

#ifndef __ARCH_ARM_INCLUDE_STM32G4_CHIP_H
#define __ARCH_ARM_INCLUDE_STM32G4_CHIP_H

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

#if defined (CONFIG_ARCH_CHIP_STM32G431K)
#  define STM32_NFSMC                    0   /* FSMC */
#  define STM32_NATIM                    2   /* (2) Advanced motor control timers TIM1, 8 with DMA */
#  define STM32_NGTIM                    6   /* (2) 16-bit general timers TIM3 and 4 with DMA
                                              * (1) 32-bit general timers TIM2 with DMA
                                              * (3) 16-bit general timers count-up timers with DMA: TIM15-17 */
#  define STM32_NGTIMNDMA                0   /* (0) 16-bit general timers TIM9-14 without DMA */
#  define STM32_NBTIM                    2   /* (2) Basic timers, TIM6-7 */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     2   /* I2S2-3 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   2   /* USART1-2 */
#  define STM32_NLPUART                  1   /* LPUART1 */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     1   /* FDCAN1 */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  0   /* No USB OTG FS/HS (but there is USB 2.0 full-speed
                                              * with LPM and BCD support) */
#  define STM32_NGPIO                    26  /* GPIOA-G */
#  define STM32_NADC                     2   /* 12-bit ADC1-2 */
#  define STM32_NDAC                     2   /* 12-bit DAC1-2, 4 channels (2 external, 2 internal) */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined (CONFIG_ARCH_CHIP_STM32G431C)
#  define STM32_NFSMC                    0   /* FSMC */
#  define STM32_NATIM                    2   /* (2) Advanced motor control timers TIM1, 8 with DMA */
#  define STM32_NGTIM                    6   /* (2) 16-bit general timers TIM3 and 4 with DMA
                                              * (1) 32-bit general timers TIM2 with DMA
                                              * (3) 16-bit general timers count-up timers with DMA: TIM15-17 */
#  define STM32_NGTIMNDMA                0   /* (0) 16-bit general timers TIM9-14 without DMA */
#  define STM32_NBTIM                    2   /* (2) Basic timers, TIM6-7 */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     2   /* I2S2-3 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   3   /* USART1-3 */
#  define STM32_NLPUART                  1   /* LPUART1 */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     1   /* FDCAN1 */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  0   /* No USB OTG FS/HS (but there is USB 2.0 full-speed
                                              * with LPM and BCD support) */
#  define STM32_NGPIO                    42  /* GPIOA-G */
#  define STM32_NADC                     2   /* 12-bit ADC1-2 */
#  define STM32_NDAC                     2   /* 12-bit DAC1-2, 4 channels (2 external, 2 internal) */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined (CONFIG_ARCH_CHIP_STM32G431R)
#  define STM32_NFSMC                    0   /* FSMC */
#  define STM32_NATIM                    2   /* (2) Advanced motor control timers TIM1, 8 with DMA */
#  define STM32_NGTIM                    6   /* (2) 16-bit general timers TIM3 and 4 with DMA
                                              * (1) 32-bit general timers TIM2 with DMA
                                              * (3) 16-bit general timers count-up timers with DMA: TIM15-17 */
#  define STM32_NGTIMNDMA                0   /* (0) 16-bit general timers TIM9-14 without DMA */
#  define STM32_NBTIM                    2   /* (2) Basic timers, TIM6-7 */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     2   /* I2S2-3 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   4   /* USART1-3 and UART4*/
#  define STM32_NLPUART                  1   /* LPUART1 */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     1   /* FDCAN1 */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  0   /* No USB OTG FS/HS (but there is USB 2.0 full-speed
                                              * with LPM and BCD support) */
#  define STM32_NGPIO                    52  /* GPIOA-G */
#  define STM32_NADC                     2   /* 12-bit ADC1-2 */
#  define STM32_NDAC                     2   /* 12-bit DAC1-2, 4 channels (2 external, 2 internal) */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined (CONFIG_ARCH_CHIP_STM32G431M)
#  define STM32_NFSMC                    0   /* FSMC */
#  define STM32_NATIM                    2   /* (2) Advanced motor control timers TIM1, 8 with DMA */
#  define STM32_NGTIM                    6   /* (2) 16-bit general timers TIM3 and 4 with DMA
                                              * (1) 32-bit general timers TIM2 with DMA
                                              * (3) 16-bit general timers count-up timers with DMA: TIM15-17 */
#  define STM32_NGTIMNDMA                0   /* (0) 16-bit general timers TIM9-14 without DMA */
#  define STM32_NBTIM                    2   /* (2) Basic timers, TIM6-7 */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     2   /* I2S2-3 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   4   /* USART1-3 and UART4*/
#  define STM32_NLPUART                  1   /* LPUART1 */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     1   /* FDCAN1 */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  0   /* No USB OTG FS/HS (but there is USB 2.0 full-speed
                                              * with LPM and BCD support) */
#  define STM32_NGPIO                    66  /* GPIOA-G */
#  define STM32_NADC                     2   /* 12-bit ADC1-2 */
#  define STM32_NDAC                     2   /* 12-bit DAC1-2, 4 channels (2 external, 2 internal) */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined (CONFIG_ARCH_CHIP_STM32G431V)
#  define STM32_NFSMC                    0   /* FSMC */
#  define STM32_NATIM                    2   /* (2) Advanced motor control timers TIM1, 8 with DMA */
#  define STM32_NGTIM                    6   /* (2) 16-bit general timers TIM3 and 4 with DMA
                                              * (1) 32-bit general timers TIM2 with DMA
                                              * (3) 16-bit general timers count-up timers with DMA: TIM15-17 */
#  define STM32_NGTIMNDMA                0   /* (0) 16-bit general timers TIM9-14 without DMA */
#  define STM32_NBTIM                    2   /* (2) Basic timers, TIM6-7 */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     2   /* I2S2-3 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   4   /* USART1-3 and UART4*/
#  define STM32_NLPUART                  1   /* LPUART1 */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     1   /* FDCAN1 */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  0   /* No USB OTG FS/HS (but there is USB 2.0 full-speed
                                              * with LPM and BCD support) */
#  define STM32_NGPIO                    86  /* GPIOA-G */
#  define STM32_NADC                     2   /* 12-bit ADC1-2 */
#  define STM32_NDAC                     2   /* 12-bit DAC1-2, 4 channels (2 external, 2 internal) */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

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
#  define STM32_NLPUART                  1   /* LPUART1 */
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
#  define STM32_NLPUART                  1   /* LPUART1 */
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
#  define STM32_NLPUART                  1   /* LPUART1 */
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
#  define STM32_NLPUART                  1   /* LPUART1 */
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
#  define STM32_NLPUART                  1   /* LPUART1 */
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

/* Peripheral IP versions ***************************************************/

/* Peripheral IP versions are invariant and should be decided here, not in
 * Kconfig.
 *
 * REVISIT: Currently only SPI IP version is handled here, with others being
 *          handled in Kconfig. Those others need to be gradually refactored
 *          and resolved here.
 */

#define STM32_HAVE_IP_SPI_V4

/* NVIC priority levels *****************************************************/

#define NVIC_SYSH_PRIORITY_MIN     0xf0 /* All bits set in minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT 0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX     0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP    0x10 /* Four bits of interrupt priority used */

#endif /* __ARCH_ARM_INCLUDE_STM32G4_CHIP_H */
