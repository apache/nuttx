/****************************************************************************
 * libs/libnx/nxglib/nxglib_rgb2yuv.c
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
#include <debug.h>
#include <fixedmath.h>

#include <nuttx/nx/nxglib.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

#define b16_P0813 0x000014d0    /* 0.0813 */
#define b16_P1140 0x00001d2f    /* 0.1140 */
#define b16_P1687 0x00002b30    /* 0.1687 */
#define b16_P2990 0x00004c8b    /* 0.2990 */
#define b16_P3313 0x000054d0    /* 0.3313 */
#define b16_P4187 0x00006b30    /* 0.4187 */
#define b16_P5000 0x00008000    /* 0.5000 */
#define b16_P5870 0x00009646    /* 0.5870 */
#define b16_128P0 0x00800000    /* 128.0 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxgl_rgb2yuv
 *
 * Description:
 *   Convert 8-bit RGB triplet to 8-bit YUV triplet
 *
 ****************************************************************************/

void nxgl_rgb2yuv(uint8_t r, uint8_t g, uint8_t b,
                  uint8_t *y, uint8_t *u, uint8_t *v)
{
  /* Per the JFIF specification:
   *
   * Y =       (0.2990 * R) + (0.5870 * G) + (0.1140 * B)
   * U = 128 - (0.1687 * R) - (0.3313 * G) + (0.5000 * B)
   * V = 128 + (0.5000 * R) - (0.4187 * G) - (0.0813 * B);
   */

  *y = (uint8_t)b16toi(b16muli(b16_P2990, r) +
        b16muli(b16_P5870, g) + b16muli(b16_P1140, b));
  *u = (uint8_t)b16toi(b16_128P0 - b16muli(b16_P1687, r) -
        b16muli(b16_P3313, g) + b16muli(b16_P5000, b));
  *v = (uint8_t)b16toi(b16_128P0 + b16muli(b16_P5000, r) -
        b16muli(b16_P4187, g) - b16muli(b16_P0813, b));
}
