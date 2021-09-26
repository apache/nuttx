/****************************************************************************
 * arch/arm/src/s32k1xx/s32k1xx_flashcfg.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "hardware/s32k1xx_flashcfg.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Configured FLASH configuration bytes.  NOTE:  Little endian assumed. */

const uint8_t g_flashcfg[16]  locate_data(".flashcfg") =
{
  (uint8_t)(CONFIG_S32K1XX_FLASHCFG_BACKDOOR1 & 0xff),         /* 0x0400 */
  (uint8_t)((CONFIG_S32K1XX_FLASHCFG_BACKDOOR1 >> 8) & 0xff),  /* 0x0401 */
  (uint8_t)((CONFIG_S32K1XX_FLASHCFG_BACKDOOR1 >> 16) & 0xff), /* 0x0402 */
  (uint8_t)((CONFIG_S32K1XX_FLASHCFG_BACKDOOR1 >> 24) & 0xff), /* 0x0403 */
  (uint8_t)(CONFIG_S32K1XX_FLASHCFG_BACKDOOR2 & 0xff),         /* 0x0404 */
  (uint8_t)((CONFIG_S32K1XX_FLASHCFG_BACKDOOR2 >> 8) & 0xff),  /* 0x0405 */
  (uint8_t)((CONFIG_S32K1XX_FLASHCFG_BACKDOOR2 >> 16) & 0xff), /* 0x0406 */
  (uint8_t)((CONFIG_S32K1XX_FLASHCFG_BACKDOOR2 >> 24) & 0xff), /* 0x0407 */
  (uint8_t)(CONFIG_S32K1XX_FLASHCFG_FPROT & 0xff),             /* 0x0408 */
  (uint8_t)((CONFIG_S32K1XX_FLASHCFG_FPROT >> 8) & 0xff),      /* 0x0409 */
  (uint8_t)((CONFIG_S32K1XX_FLASHCFG_FPROT >> 16) & 0xff),     /* 0x040a */
  (uint8_t)((CONFIG_S32K1XX_FLASHCFG_FPROT >> 24) & 0xff),     /* 0x040b */
  (uint8_t)CONFIG_S32K1XX_FLASHCFG_FSEC,                       /* 0x040c */
  (uint8_t)CONFIG_S32K1XX_FLASHCFG_FOPT,                       /* 0x040d */
  (uint8_t)CONFIG_S32K1XX_FLASHCFG_FEPROT,                     /* 0x040e */
  (uint8_t)CONFIG_S32K1XX_FLASHCFG_FDPROT                      /* 0x040f */
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

