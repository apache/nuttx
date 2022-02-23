/****************************************************************************
 * arch/arm/src/efm32/hardware/efm32_flash.h
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

#ifndef __ARCH_ARM_SRC_EFM32_HARDWARE_EFM32_FLASH_H
#define __ARCH_ARM_SRC_EFM32_HARDWARE_EFM32_FLASH_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_EFM32_EFM32GG)
#   define EFM32_FLASH_PAGESIZE    4096
#elif defined(CONFIG_EFM32_EFM32LG)
#   define EFM32_FLASH_PAGESIZE    2048
#elif defined(CONFIG_EFM32_EFM32WG)
#   define EFM32_FLASH_PAGESIZE    2048
#elif defined(CONFIG_EFM32_EFM32ZG)
#   define EFM32_FLASH_PAGESIZE    1024
#elif defined(CONFIG_EFM32_EFM32G)
#   define EFM32_FLASH_PAGESIZE    512
#elif defined(CONFIG_EFM32_EFM32TG)
#   define EFM32_FLASH_PAGESIZE    512
#endif

#endif /* __ARCH_ARM_SRC_EFM32_HARDWARE_EFM32_FLASH_H */
