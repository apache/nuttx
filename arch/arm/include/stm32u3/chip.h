/****************************************************************************
 * arch/arm/include/stm32u3/chip.h
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

#ifndef __ARCH_ARM_INCLUDE_STM32U3_CHIP_H
#define __ARCH_ARM_INCLUDE_STM32U3_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_STM32_STM32U3C5XX)
#  define STM32_SRAM1_SIZE       0x00030000 /* 192 KiB SRAM1 */
#  define STM32_SRAM2_SIZE       0x00010000 /* 64 KiB SRAM2 */
#  define STM32_SRAM3_SIZE       0x00050000 /* 320 KiB SRAM3 */
#  define STM32_SRAM4_SIZE       0x00010000 /* 64 KiB SRAM4 */

/* All SRAM banks are contiguous on STM32U3C5. */

#  define STM32_PRIMARY_SRAM_SIZE 0x000a0000

#  define STM32_NFSMC           0 /* No external memory controller */
#  define STM32_NATIM           2 /* TIM1 and TIM8 */
#  define STM32_NGTIM32         1 /* TIM2 */
#  define STM32_NGTIM16         3 /* TIM3, TIM4 and TIM12 */
#  define STM32_NGTIMNDMA       3 /* TIM15, TIM16 and TIM17 */
#  define STM32_NBTIM           2 /* TIM6 and TIM7 */
#  define STM32_NLPTIM          4 /* LPTIM1, LPTIM2, LPTIM3 and LPTIM4 */
#  define STM32_NRNG            1 /* Random number generator */
#  define STM32_NUART           2 /* UART4 and UART5 */
#  define STM32_NUSART          3 /* USART1, USART2 and USART3 */
#  define STM32_NLPUART         1 /* LPUART1 */
#  define STM32_QSPI            0 /* No QuadSPI controller */
#  define STM32_OCTOSPI         1 /* OCTOSPI1 */
#  define STM32_NSPI            4 /* SPI1 through SPI4 */
#  define STM32_NI2C            4 /* I2C1 through I2C4 */
#  define STM32_NI3C            2 /* I3C1 and I3C2 */
#  define STM32_NSWPMI          0 /* No SWPMI controller */
#  define STM32_NUSBOTGFS       1 /* USB device/host full speed */
#  define STM32_NUSBFS          0 /* No USB device-only controller */
#  define STM32_NCAN            2 /* FDCAN1 and FDCAN2 */
#  define STM32_NSAI            1 /* SAI1 */
#  define STM32_NSDMMC          1 /* SDMMC1 */
#  define STM32_NDMA            1 /* GPDMA1 */
#  define STM32_NPORTS          8 /* GPIOA through GPIOH */
#  define STM32_NADC            2 /* ADC1 and ADC2 */
#  define STM32_NDAC            1 /* DAC1 */
#  define STM32_NCRC            1 /* CRC */
#  define STM32_NCOMP           2 /* COMP1 and COMP2 */
#  define STM32_NOPAMP          2 /* OPAMP1 and OPAMP2 */
#else
#  error "Unsupported STM32U3 chip"
#endif

/* NVIC priority levels */

#define NVIC_SYSH_PRIORITY_MIN     0xf0
#define NVIC_SYSH_PRIORITY_DEFAULT 0x80
#define NVIC_SYSH_PRIORITY_MAX     0x00
#define NVIC_SYSH_PRIORITY_STEP    0x10

#endif /* __ARCH_ARM_INCLUDE_STM32U3_CHIP_H */
