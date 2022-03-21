/****************************************************************************
 * arch/arm/src/s32k1xx/s32k1xx_progmem.h
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

#ifndef __ARCH_ARM_SRC_S32K1XX_S32K1XX_PROGMEM_H
#define __ARCH_ARM_SRC_S32K1XX_S32K1XX_PROGMEM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include "arm_internal.h"
#include "s32k1xx_config.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DFLASH_SIZE CONFIG_PROGMEM_SIZE

#if (DFLASH_SIZE % 2) == 1
# error "Progmem size has to be a multiple of 2"
#endif

#if defined(CONFIG_ARCH_CHIP_S32K14X) && (DFLASH_SIZE > 64)
# error "Progmem size is bigger than FlexNVM size"
#endif

#if defined(CONFIG_ARCH_CHIP_S32K11X) && (DFLASH_SIZE > 32)
# error "Progmem size is bigger than FlexNVM size"
#endif

/* Base address of the flash segment used for progmem. */

#define S32K1XX_PROGMEM_START_ADDR            0x10000000

#define S32K1XX_PROGMEM_BLOCK_COUNT           1

#define S32K1XX_PROGMEM_BLOCK_SIZE            DFLASH_SIZE * 1024

#define S32K1XX_PROGMEM_BLOCK_SECTOR_SIZE     2048

#define S32K1XX_PROGMEM_PAGE_SIZE             8

#define S32K1XX_PROGMEM_SECTOR_COUNT          S32K1XX_PROGMEM_BLOCK_SIZE / S32K1XX_PROGMEM_BLOCK_SECTOR_SIZE

#define S32K1XX_PROGMEM_PAGE_COUNT            (S32K1XX_PROGMEM_BLOCK_SIZE / S32K1XX_PROGMEM_PAGE_SIZE)

#define S32K1XX_PROGMEM_DFLASH_WRITE_UNIT_SIZE 8

#define S32K1XX_PROGMEM_ERASEDVAL             (0xffu)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void s32k1xx_progmem_init(void);

#endif /* __ARCH_ARM_SRC_S32K1XX_S32K1XX_PROGMEM_H */
