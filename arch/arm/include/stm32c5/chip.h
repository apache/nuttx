/****************************************************************************
 * arch/arm/include/stm32c5/chip.h
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

#ifndef __ARCH_ARM_INCLUDE_STM32C5_CHIP_H
#define __ARCH_ARM_INCLUDE_STM32C5_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define STM32_SRAM1_SIZE       0x00010000 /* 64 KiB SRAM1 */
#define STM32_SRAM2_SIZE       0x00010000 /* 64 KiB SRAM2 */

/* SRAM1 and SRAM2 are contiguous on STM32C5. */

#define STM32_PRIMARY_SRAM_SIZE 0x00020000

#define STM32_NFSMC           0 /* No external memory controller */
#define STM32_NATIM           2 /* TIM1 and TIM8 */
#define STM32_NGTIM32         2 /* TIM2 and TIM5 */
#define STM32_NGTIM16         1 /* TIM12 */
#define STM32_NGTIMNDMA       3 /* TIM15, TIM16 and TIM17 */
#define STM32_NBTIM           2 /* TIM6 and TIM7 */
#define STM32_NLPTIM          1 /* LPTIM1 */
#define STM32_NRNG            1 /* Random number generator */
#define STM32_NUART           2 /* UART4 and UART5 */
#define STM32_NUSART          3 /* USART1, USART2 and USART3 */
#define STM32_NLPUART         1 /* LPUART1 */
#define STM32_QSPI            0 /* No QuadSPI controller */
#define STM32_OCTOSPI         0 /* No OctoSPI controller */
#define STM32_NSPI            3 /* SPI1 through SPI3 */
#define STM32_NI2C            2 /* I2C1 and I2C2 */
#define STM32_NI3C            1 /* I3C1 */
#define STM32_NSWPMI          0 /* No SWPMI controller */
#define STM32_NUSBOTGFS       1 /* USB device/host full speed */
#define STM32_NUSBFS          0 /* No USB device-only controller */
#define STM32_NCAN            1 /* FDCAN1 */
#define STM32_NSAI            0 /* No SAI controller */
#define STM32_NSDMMC          0 /* No SDMMC controller */
#define STM32_NDMA            2 /* LPDMA1 and LPDMA2 */
#define STM32_NPORTS          8 /* GPIOA-E and GPIOH */
#define STM32_NADC            2 /* ADC1 and ADC2 */
#define STM32_NDAC            1 /* DAC1 */
#define STM32_NCRC            1 /* CRC */
#define STM32_NCOMP           1 /* COMP1 */
#define STM32_NOPAMP          0 /* No operational amplifier */

/* NVIC priority levels */

#define NVIC_SYSH_PRIORITY_MIN     0xf0
#define NVIC_SYSH_PRIORITY_DEFAULT 0x80
#define NVIC_SYSH_PRIORITY_MAX     0x00
#define NVIC_SYSH_PRIORITY_STEP    0x10

#endif /* __ARCH_ARM_INCLUDE_STM32C5_CHIP_H */
