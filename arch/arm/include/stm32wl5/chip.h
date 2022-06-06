/****************************************************************************
 * arch/arm/include/stm32wl5/chip.h
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

#ifndef __ARCH_ARM_INCLUDE_STM32WL5_CHIP_H
#define __ARCH_ARM_INCLUDE_STM32WL5_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

#if defined(CONFIG_STM32WL5_STM32WL5XXX)
#  define STM32WL5_SRAM1_SIZE       (32*1024)   /* 32kB SRAM1 on AHB bus Matrix */
#  define STM32WL5_SRAM2_SIZE       (32*1024)   /* 32kB SRAM2 on AHB bus Matrix */
#else
#  error "Unsupported STM32L5 chip"
#endif

#if defined(CONFIG_STM32WL5_STM32WL5XXX_CPU1)
#  define STM32WL5_NATIM                    1   /* One advanced timer TIM1 */
#  define STM32WL5_NGTIM32                  1   /* 32-bit general timer TIM2 with DMA */
#  define STM32WL5_NGTIM16                  2   /* 16-bit general timers TIM16 and 17 with DMA */
#  define STM32WL5_NLPTIM                   3   /* Three low-power timer, LPTIM1-3 */
#  define STM32WL5_NRNG                     1   /* Random number generator (RNG) */
#  define STM32WL5_NUSART                   2   /* USART 1-2 */
#  define STM32WL5_NLPUART                  1   /* LPUART 1 */
#  define STM32WL5_NSPI                     2   /* SPI1 and SPI2S2 (spi2 shared with i2s) */
#  define STM32WL5_NI2C                     3   /* I2C1-3 */
#  define STM32WL5_NDMA                     2   /* Two DMA channels DMA1-2 */
#  define STM32WL5_NPORTS                   4   /* GPIO{A,B,C,H} */
#  define STM32WL5_NADC                     1   /* ADC1 */
#  define STM32WL5_NDAC                     1   /* DAC1 */
#  define STM32WL5_NCRC                     1   /* CRC1 */
#  define STM32WL5_NCOMP                    1   /* COMP1 */
#endif /* CONFIG_STM32WL5_STM32WL5XXX */

/* NVIC priority levels *****************************************************/

/* 16 Programmable interrupt levels */

#define NVIC_SYSH_PRIORITY_MIN     0xf0 /* All bits set in minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT 0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX     0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP    0x10 /* Four bits of interrupt priority used */

#endif /* __ARCH_ARM_INCLUDE_STM32WL5_CHIP_H */
