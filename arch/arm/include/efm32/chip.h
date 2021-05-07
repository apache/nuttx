/****************************************************************************
 * arch/arm/include/efm32/chip.h
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

#ifndef __ARCH_ARM_INCLUDE_EFM32_CHIP_H
#define __ARCH_ARM_INCLUDE_EFM32_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* EFM32 EnergyMicro ********************************************************/

/* Tiny Gecko with 32KiB FLASH and 4KiB RAM in a QFN64 package */

#if defined(CONFIG_ARCH_CHIP_EFM32TG840F32)

/* Gecko with 128KiB FLASH and 16KiB SRAM in LQFP100 (EFM32G880F128) or
 * BGA112 (EFM32G890F128) package
 */

#elif defined(CONFIG_ARCH_CHIP_EFM32G880F128) || \
      defined(CONFIG_ARCH_CHIP_EFM32G890F128)

/* Giant Gecko with 1024KiB FLASH and 128KiB RAM in a QFP64 package
 * (EFM32GG332F1024) or BGA112 (EFM32GG990F1024) package
 */

#elif defined(CONFIG_ARCH_CHIP_EFM32GG332F1024) || \
      defined(CONFIG_ARCH_CHIP_EFM32GG990F1024)

#else
#  error "Unsupported EFM32 chip"
#endif

/* NVIC priority levels *****************************************************/

#define NVIC_SYSH_PRIORITY_MIN     0xe0 /* Bits [7:5] set in minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT 0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX     0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP    0x20 /* Three bits of interrupt priority used */

#endif /* __ARCH_ARM_INCLUDE_EFM32_CHIP_H */
