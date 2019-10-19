/****************************************************************************
 *  arch/arm/src/s32k1xx/s32k1xx_flashcfg.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "hardware/s32k1xx_flashcfg.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Configured FLASH configuration bytes */

uint8_t g_flashcfg[16]  __attribute__((section(".flashcfg"))) =
{
  (uint8_t)((CONFIG_S32K1XX_FLASHCFG_BACKDOOR1 >> 24) & 0xff),
  (uint8_t)((CONFIG_S32K1XX_FLASHCFG_BACKDOOR1 >> 16) & 0xff),
  (uint8_t)((CONFIG_S32K1XX_FLASHCFG_BACKDOOR1 >> 8) & 0xff),
  (uint8_t)(CONFIG_S32K1XX_FLASHCFG_BACKDOOR1 & 0xff),
  (uint8_t)((CONFIG_S32K1XX_FLASHCFG_BACKDOOR2 >> 24) & 0xff),
  (uint8_t)((CONFIG_S32K1XX_FLASHCFG_BACKDOOR2 >> 16) & 0xff),
  (uint8_t)((CONFIG_S32K1XX_FLASHCFG_BACKDOOR2 >> 8) & 0xff),
  (uint8_t)(CONFIG_S32K1XX_FLASHCFG_BACKDOOR2 & 0xff),
  (uint8_t)((CONFIG_S32K1XX_FLASHCFG_FPROT >> 24) & 0xff),
  (uint8_t)((CONFIG_S32K1XX_FLASHCFG_FPROT >> 16) & 0xff),
  (uint8_t)((CONFIG_S32K1XX_FLASHCFG_FPROT >> 8) & 0xff),
  (uint8_t)(CONFIG_S32K1XX_FLASHCFG_FPROT & 0xff),
  (uint8_t)CONFIG_S32K1XX_FLASHCFG_FDPROT,
  (uint8_t)CONFIG_S32K1XX_FLASHCFG_FEPROT,
  (uint8_t)CONFIG_S32K1XX_FLASHCFG_FOPT,
  (uint8_t)CONFIG_S32K1XX_FLASHCFG_FSEC
};
