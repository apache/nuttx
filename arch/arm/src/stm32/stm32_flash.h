/****************************************************************************
 * arch/arm/src/stm32/stm32_flash.h
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

#ifndef __ARCH_ARM_SRC_STM32_STM32_FLASH_H
#define __ARCH_ARM_SRC_STM32_STM32_FLASH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/progmem.h>

#include "chip.h"
#include "hardware/stm32_flash.h"

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int stm32_flash_lock(void);
int stm32_flash_unlock(void);

/****************************************************************************
 * Name: stm32_flash_user_optbytes
 *
 * Description:
 *   Modify the contents of the user option bytes (USR OPT) on the flash.
 *   This does not set OBL_LAUNCH so new options take effect only after
 *   next power reset.
 *
 * Input Parameters:
 *   clrbits - Bits in the option bytes to be cleared
 *   setbits - Bits in the option bytes to be set
 *
 * Returned Value:
 *   Option bytes after operation is completed
 *
 ****************************************************************************/

uint32_t stm32_flash_users_optbytes(uint32_t clrbits, uint32_t setbits);

/****************************************************************************
 * Name: stm32_eeprom_size
 *
 * Description:
 *   Get EEPROM data memory size
 *
 * Returned Value:
 *  Length of EEPROM memory region
 *
 ****************************************************************************/

size_t stm32_eeprom_size(void);

/****************************************************************************
 * Name: stm32_eeprom_getaddress
 *
 * Description:
 *   Get EEPROM data memory address
 *
 * Returned Value:
 *  Address of EEPROM memory region
 *
 ****************************************************************************/

size_t stm32_eeprom_getaddress(void);

/****************************************************************************
 * Name: stm32_eeprom_write
 *
 * Description:
 *    Write buffer to EEPROM data memory address
 *
 * Returned Value:
 *  Number of written bytes or error code.
 *
 ****************************************************************************/

ssize_t stm32_eeprom_write(size_t addr, const void *buf, size_t buflen);

/****************************************************************************
 * Name: stm32_eeprom_erase
 *
 * Description:
 *    Erase memory on EEPROM data memory address
 *
 * Returned Value:
 *  Number of erased bytes or error code.
 *
 ****************************************************************************/

ssize_t stm32_eeprom_erase(size_t addr, size_t eraselen);

#endif /* __ARCH_ARM_SRC_STM32_STM32_FLASH_H */
