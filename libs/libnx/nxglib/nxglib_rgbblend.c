/****************************************************************************
 * libs/libnx/nxglib/nxglib_rgbblend.c
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

#include <stdint.h>
#include <fixedmath.h>
#include <nuttx/video/rgbcolors.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxglib_rgb24_blend and nxglib_rgb565_blend
 *
 * Description:
 *   Blend a single RGB color component.  This is *not* alpha blending:
 *   component2 is assumed to be opaque and "under" a semi-transparent
 *   component1.
 *
 *   The frac1 value could be though as related to the 1/alpha value for
 *   component1.
 *   However, the background, component2, is always treated as though
 *   alpha == 1.
 *
 *   This algorithm is used to handle endpoints as part of the
 *   implementation of anti-aliasing without transparency.
 *
 * Input Parameters:
 *   component1 - The semi-transparent, foreground 8-bit color component
 *   component2 - The opaque, background color component
 *   frac1  - The fractional amount of component1 to blend into component2
 *
 * Returned Value:
 *   The blended 8-bit color component.
 *
 ****************************************************************************/

#if !defined(CONFIG_NX_DISABLE_16BPP) || !defined(CONFIG_NX_DISABLE_24BPP) || \
    !defined(CONFIG_NX_DISABLE_32BPP)

static uint8_t nxglib_blend_component(uint8_t component1, uint8_t component2,
                                     ub8_t frac1)
{
  uint16_t blend;
  uint32_t blendb8;

  /* Use a uint32_t for the intermediate calculation.  Due to rounding this
   * value could exceed ub8MAX (0xffff == 255.999..).
   *
   * Hmm.. that might not actually be possible but this gives me piece of
   * mind and there should not be any particular overhead on a 32-bit
   * processor.
   */

  blendb8 = (uint32_t)((ub16_t)component1 * frac1) +
            (uint32_t)((ub16_t)component2 * (b8ONE - frac1)) +
            (uint32_t)b8HALF;

  /* Now we can snap it down to 16-bits and check for the overflow
   * condition.
   */

  blend = ub8toi(blendb8);
  if (blend > 255)
    {
      blend = 255;
    }

  /* Return the blended value */

  return (uint8_t)blend;
}

#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxglib_rgb24_blend and nxglib_rgb565_blend
 *
 * Description:
 *   Blend a foreground color onto a background color.  This is *not* alpha
 *   blending:  color2 is assumed to be opaque and "under" a semi-
 *   transparent color1.
 *
 *   The frac1 value could be though as related to the 1/alpha value for
 *   color1.  However, the background, color2, is always treated as though
 *   alpha == 1.
 *
 *   This algorithm is used to handle endpoints as part of the
 *   implementation of anti-aliasing without transparency.
 *
 * Input Parameters:
 *   color1 - The semi-transparent, foreground color
 *   color2 - The opaque, background color
 *   frac1  - The fractional amount of color1 to blend into color2
 *
 * Returned Value:
 *   The blended color, encoded just was the input color1 and color2
 *
 ****************************************************************************/

#if !defined(CONFIG_NX_DISABLE_24BPP) || !defined(CONFIG_NX_DISABLE_32BPP)

uint32_t nxglib_rgb24_blend(uint32_t color1, uint32_t color2, ub16_t frac1)
{
  uint8_t r;
  uint8_t g;
  uint8_t b;
  uint8_t bg;
  ub8_t fracb8;

  /* Convert the fraction to ub8_t.  We don't need that much precision to
   * scale an 8-bit color component.
   */

  fracb8 = ub16toub8(frac1);

  /* Some limit checks.  Rounding in the b16 to b8 conversion could cause
   * the fraction exceed one; the loss of precision could cause small b16
   * values to convert to zero.
   */

  if (fracb8 >= b8ONE)
    {
      return color1;
    }
  else if (fracb8 == 0)
    {
      return color2;
    }

  /* Separate and blend each component */

  r  = RGB24RED(color1);
  bg = RGB24RED(color2);
  r  = nxglib_blend_component(r, bg, fracb8);

  g  = RGB24GREEN(color1);
  bg = RGB24GREEN(color2);
  g  = nxglib_blend_component(g, bg, fracb8);

  b  = RGB24BLUE(color1);
  bg = RGB24BLUE(color2);
  b  = nxglib_blend_component(b, bg, fracb8);

  /* Recombine and return the blended value */

  return RGBTO24(r, g, b);
}

#endif

#ifndef CONFIG_NX_DISABLE_16BPP

uint16_t nxglib_rgb565_blend(uint16_t color1, uint16_t color2, ub16_t frac1)
{
  uint8_t r;
  uint8_t g;
  uint8_t b;
  uint8_t bg;
  ub8_t fracb8;

  /* Convert the fraction to ub8_t.  We don't need that much precision. */

  fracb8 = ub16toub8(frac1);

  /* Some limit checks */

  if (fracb8 >= b8ONE)
    {
      return color1;
    }
  else if (fracb8 == 0)
    {
      return color2;
    }

  /* Separate and blend each component */

  r  = RGB16RED(color1);
  bg = RGB16RED(color2);
  r  = nxglib_blend_component(r, bg, fracb8);

  g  = RGB16GREEN(color1);
  bg = RGB16GREEN(color2);
  g  = nxglib_blend_component(g, bg, fracb8);

  b  = RGB16BLUE(color1);
  bg = RGB16BLUE(color2);
  b  = nxglib_blend_component(b, bg, fracb8);

  /* Recombine and return the blended value */

  return RGBTO24(r, g, b);
}

#endif
