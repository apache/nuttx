/****************************************************************************
 * arch/arm/src/common/stm32/stm32_flash.h
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

#ifndef __ARCH_ARM_SRC_COMMON_COMPAT_STM32_FLASH_H
#define __ARCH_ARM_SRC_COMMON_COMPAT_STM32_FLASH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/progmem.h>

#include <sys/types.h>
#include <stdint.h>

#include "chip.h"
#include "hardware/stm32_flash.h"

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#if defined(CONFIG_STM32_HAVE_IP_FLASH_M0_V1)

void stm32_flash_getopt(uint32_t *opt);
int stm32_flash_optmodify(uint32_t clear, uint32_t set);
void stm32_flash_lock(void);
void stm32_flash_unlock(void);

#elif defined(CONFIG_STM32_HAVE_IP_FLASH_M3M4_V1)

int stm32_flash_lock(void);
int stm32_flash_unlock(void);
uint32_t stm32_flash_users_optbytes(uint32_t clrbits, uint32_t setbits);
size_t stm32_eeprom_size(void);
size_t stm32_eeprom_getaddress(void);
ssize_t stm32_eeprom_write(size_t addr, const void *buf, size_t buflen);
ssize_t stm32_eeprom_erase(size_t addr, size_t eraselen);

#endif

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_COMMON_COMPAT_STM32_FLASH_H */
