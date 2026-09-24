/****************************************************************************
 * arch/arm/src/stm32h5/stm32_flash.h
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

#ifndef __ARCH_ARM_SRC_STM32H5_STM32_FLASH_H
#define __ARCH_ARM_SRC_STM32H5_STM32_FLASH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <sys/types.h>

#include "hardware/stm32_flash.h"

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

void stm32_flash_getopt(uint32_t *opt1, uint32_t *opt2);

int stm32_flash_optmodify(uint32_t clear1, uint32_t set1,
                          uint32_t clear2, uint32_t set2);

int stm32_flash_swapbanks(void);

void stm32_flash_lock(void);

void stm32_flash_unlock(void);

int stm32_otp_write(const uint16_t *data, uint16_t len, uint32_t offset);

int stm32_otp_read(uint16_t *data, uint16_t len, uint32_t offset);

uint32_t stm32_otp_getlockstatus(void);

/* Flash high-cycle data (EDATA) low-level access.
 *
 * EDATA can be enabled on the last 1..8 sectors of each physical bank.
 * Each 8 KiB user flash sector becomes a 6 KiB EDATA sector, read and
 * programmed in 16-bit half-words and mapped at STM32_EDATA_BASE.
 *
 * Banks are always physical banks (1 or 2).  The EDATA window follows the
 * SWAP_BANK option in the same way as the user flash, which these functions
 * account for.  Sectors are numbered 0..7 within the EDATA area of a bank,
 * where 0 is the first of the last eight sectors of the bank.
 */

#define STM32_EDATA_BANK_NSECTORS  8
#define STM32_EDATA_SECTOR_SIZE    6144

int stm32_flash_edata_getconfig(int bank);
int stm32_flash_edata_configure(int bank, unsigned int nsectors);
uintptr_t stm32_flash_edata_address(int bank, unsigned int sector);
int stm32_flash_edata_erase(int bank, unsigned int sector);
ssize_t stm32_flash_edata_read(uintptr_t addr, void *buf, size_t count);
ssize_t stm32_flash_edata_write(uintptr_t addr, const void *buf,
                                size_t count);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_STM32H5_STM32_FLASH_H */
