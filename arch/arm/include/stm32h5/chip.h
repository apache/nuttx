/****************************************************************************
 * arch/arm/include/stm32h5/chip.h
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

#ifndef __ARCH_ARM_INCLUDE_STM32H5_CHIP_H
#define __ARCH_ARM_INCLUDE_STM32H5_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

#if defined(CONFIG_STM32H5_STM32H52XXX) || defined(CONFIG_STM32H5_STM32H53XXX)
#  define STM32H5_SRAM1_SIZE       (128*1024)  /* 192Kb SRAM1 on AHB bus Matrix */
#  define STM32H5_SRAM2_SIZE       (80*1024)   /* 80Kb  SRAM2 on AHB bus Matrix */
#  define STM32H5_SRAM3_SIZE       (64*1024)   /* 64Kb  SRAM3 on AHB bus Matrix */
#elif defined(CONFIG_STM32H5_STM32H56XXX) || defined(CONFIG_STM32H5_STM32H57XXX)
#  define STM32H5_SRAM1_SIZE       (256*1024)  /* 192Kb SRAM1 on AHB bus Matrix */
#  define STM32H5_SRAM2_SIZE       (64*1024)   /* 64Kb  SRAM2 on AHB bus Matrix */
#  define STM32H5_SRAM3_SIZE       (320*1024)  /* 320Kb SRAM3 on AHB bus Matrix */
#else
#  error "Unsupported STM32H5 chip"
#endif

#define STM32H5_NFSMC                    (1)   /* Have FSMC memory controller */
#define STM32H5_NATIM                    (2)   /* Two advanced timers TIM1 and TIM8 */
#define STM32H5_NGTIM32                  (2)   /* 32-bit general timers TIM2 and 5 with DMA */
#define STM32H5_NGTIM16                  (2)   /* 16-bit general timers TIM3 and 4 with DMA */
#define STM32H5_NGTIMNDMA                (3)   /* 16-bit general timers TIM15-17 without DMA */
#define STM32H5_NBTIM                    (2)   /* Two basic timers, TIM6-7 */
#define STM32H5_NLPTIM                   (6)   /* Six low-power timers, LPTIM1-LPTIM6. */
#define STM32H5_NRNG                     (1)   /* Random number generator (RNG) */

#if defined(CONFIG_STM32H5_STM32H56XXX) || defined(CONFIG_STM32H5_STM32H57XXX)
#  define STM32H5_NUART                  (6)   /* UART 4-5, 7-8, 9, 12 */
#  define STM32H5_NUSART                 (5)   /* USART 1-3, 6, 10-11 */
#elif defined(CONFIG_STM32H5_STM32H52XXX) || defined(CONFIG_STM32H5_STM32H53XXX)
#  define STM32H5_NUART                  (2)   /* UART 4-5 */
#  define STM32H5_NUSART                 (4)   /* USART 1-3, 6*/
#endif

#define STM32H5_NLPUART                  (1)   /* LPUART 1 */
#define STM32H5_QSPI                     (0)   /* No QuadSPI1 */
#define STM32H5_OCTOSPI                  (1)   /* OCTOSPI1*/

#if defined(CONFIG_STM32H5_STM32H56XXX) || defined(CONFIG_STM32H5_STM32H57XXX)
#  define STM32H5_NSPI                   (6)   /* SPI1-SPI6 */
#  define STM32H5_NI2C                   (4)   /* I2C1-4 */
#elif defined(CONFIG_STM32H5_STM32H52XXX) || defined(CONFIG_STM32H5_STM32H53XXX)
#  define STM32H5_NSPI                   (3)   /* SPI1-SPI3 */
#  define STM32H5_NI2C                   (3)   /* I2C1-3 */
#endif

#define STM32H5_NSWPMI                   (0)   /* No SWPMI1 */
#define STM32H5_NUSBOTGFS                (0)   /* USB OTG FS */
#define STM32H5_NUSBFS                   (1)   /* No USB FS */
#define STM32H5_NCAN                     (2)   /* CAN1 */
#define STM32H5_NSAI                     (2)   /* SAI1-2 */

#if defined(CONFIG_STM32H5_STM32H56XXX) || defined(CONFIG_STM32H5_STM32H57XXX)
#  define STM32H5_NSDMMC                 (2)   /* SDMMC interface */
#elif defined(CONFIG_STM32H5_STM32H52XXX) || defined(CONFIG_STM32H5_STM32H53XXX)
#  define STM32H5_NSDMMC                 (1)   /* SDMMC interface */
#endif

#define STM32H5_NDMA                     (2)   /* DMA1-2 */
#define STM32H5_NPORTS                   (8)   /* 8 GPIO ports, GPIOA-GPIOI */
#define STM32H5_NADC                     (2)   /* 12-bit ADC1, up to 20 channels */
#define STM32H5_NDAC                     (1)   /* 12-bit DAC1 */
#define STM32H5_NCRC                     (1)   /* CRC */
#define STM32H5_NCOMP                    (0)   /* Comparators */
#define STM32H5_NOPAMP                   (0)   /* Operational Amplifiers */

/* NVIC priority levels *****************************************************/

/* 16 Programmable interrupt levels */

#define NVIC_SYSH_PRIORITY_MIN     0xf0 /* All bits set in minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT 0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX     0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP    0x10 /* Four bits of interrupt priority used */

#if defined(CONFIG_STM32H5_HAVE_ETHERNET)
#  define STM32H5_NETHERNET             1   /* Ethernet MAC */
#else
#  define STM32H5_NETHERNET               0   /* No Ethernet MAC */
#endif

#endif /* __ARCH_ARM_INCLUDE_STM32H5_CHIP_H */
