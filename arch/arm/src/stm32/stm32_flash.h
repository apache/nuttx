/****************************************************************************
 * arch/arm/src/stm32/stm32_flash.h
 *
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *   Author: Uros Platise <uros.platise@isotel.eu>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
