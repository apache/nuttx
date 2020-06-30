/******************************************************************************
 * arch/arm/src/s32k1xx/s32k1xx_progmem.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author:  Gregory Nutt <gnutt@nuttx.org>
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
 ******************************************************************************/

#ifndef __ARCH_ARM_SRC_S32K1XX_PROGMEM_H
#define __ARCH_ARM_SRC_S32K1XX_PROGMEM_H

/******************************************************************************
 * Included Files
 ******************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include "arm_internal.h"
#include "s32k1xx_config.h"

/******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

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

/******************************************************************************
 * Public Function Prototypes
 ******************************************************************************/

void s32k1xx_progmem_init();

#endif /* __ARCH_ARM_SRC_S32K1XX_PROGMEM_H */
