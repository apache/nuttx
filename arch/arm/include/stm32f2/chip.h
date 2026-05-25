/****************************************************************************
 * arch/arm/include/stm32f2/chip.h
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

#ifndef __ARCH_ARM_INCLUDE_STM32F2_CHIP_H
#define __ARCH_ARM_INCLUDE_STM32F2_CHIP_H

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

#if defined(CONFIG_ARCH_CHIP_STM32F205RG)  /* UFBGA-176 1024Kb FLASH 128Kb SRAM */
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
#  define STM32_NLPUART                  0   /* No LPUART */
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
#  define STM32_NLPUART                  0   /* No LPUART */
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
#  define STM32_NLPUART                  0   /* No LPUART */
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

/* STM23 F3 Family **********************************************************/

/* Part Numbering: STM32Fssscfxxx
 *
 *  Where
 *     sss = 302/303, 334 or 372/373
 *     c   = C (48pins) R (68 pins) V (100 pins)
 *     c   = K (32 pins), C (48 pins), R (68 pins), V (100 pins)
 *     f   = 6 (32KB FLASH), 8 (64KB FLASH), B (128KB FLASH), C (256KB FLASH)
 *     xxx = Package, temperature range, options (ignored here)
 */

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

#endif /* __ARCH_ARM_INCLUDE_STM32F2_CHIP_H */
