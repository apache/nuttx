/****************************************************************************
 * libs/libnx/nxglib/nxglib_yuv2rgb.c
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

#define b16_P3441 0x0000581a    /*   0.344147 */
#define b16_P7141 0x0000b6d2    /*   0.714142 */
#define b16_1P402 0x000166ea    /*   1.402008 */
#define b16_1P772 0x0001c5a2    /*   1.722003 */
#define b16_128P0 0x00800000    /* 128.000000 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxgl_yuv2rgb
 *
 * Description:
 *   Convert 8-bit RGB triplet to 8-bit YUV triplet
 *
 ****************************************************************************/

void nxgl_yuv2rgb(uint8_t y, uint8_t u, uint8_t v,
                  uint8_t *r, uint8_t *g, uint8_t *b)
{
  b16_t vm128 = itob16(v) - b16_128P0;
  b16_t um128 = itob16(u) - b16_128P0;

  /* Per the JFIF specification:
   *
   * R = Y                         + 1.40200 * (V - 128.0)
   * G = Y - 0.34414 * (U - 128.0) - 0.71414 * (V - 128.0)
   * B = Y + 1.77200 * (U - 128.0)
   */

  *r = (uint8_t)b16toi(itob16(y) +
        b16muli(b16_1P402, vm128));
  *g = (uint8_t)b16toi(itob16(y) - b16muli(b16_P3441, um128) -
        b16muli(b16_P7141, vm128));
  *b = (uint8_t)b16toi(itob16(y) + b16muli(b16_1P772, um128));
}
