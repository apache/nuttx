/****************************************************************************
 * arch/arm/src/tlsr82/tlsr82_flash.h
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

#ifndef __ARCH_ARM_SRC_TLSR82_TLSR82_FLASH_H
#define __ARCH_ARM_SRC_TLSR82_TLSR82_FLASH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "chip.h"

#include "hardware/tlsr82_mspi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FLASH_VOLTAGE_1V6                    0x00
#define FLASH_VOLTAGE_1V65                   0x01
#define FLASH_VOLTAGE_1V7                    0x02
#define FLASH_VOLTAGE_1V75                   0x03
#define FLASH_VOLTAGE_1V8                    0x04
#define FLASH_VOLTAGE_1V85                   0x05
#define FLASH_VOLTAGE_1V9                    0x06
#define FLASH_VOLTAGE_1V95                   0x07

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
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

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void tlsr82_flash_erase_sector(uint32_t addr);
void tlsr82_flash_read_data(uint32_t addr, uint8_t *buf, uint32_t len);
void tlsr82_flash_write_data(uint32_t addr, const uint8_t *buf,
                             uint32_t len);
uint8_t tlsr82_flash_read_status(uint8_t cmd);
void tlsr82_flash_write_status(uint8_t type, uint16_t status);

#ifdef CONFIG_TLSR82_FLASH_PROTECT
void tlsr82_flash_protect(void);
void tlsr82_flash_unprotect(void);
#else
#  define tlsr82_flash_protect()
#  define tlsr82_flash_unprotect()
#endif

int tlsr82_flash_miduid_check(uint32_t *pmid, uint8_t *puid);

#ifdef CONFIG_TLSR82_FLASH_CALIBRATE
void tlsr82_flash_calibrate(uint32_t mid);
#else
#  define tlsr82_flash_calibrate(mid)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_TLSR82_TLSR82_FLASH_H */
