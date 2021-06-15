/****************************************************************************
 * graphics/nxglib/nxglib_copyrun.h
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

#ifndef __GRAPHICS_NXGLIB_NXGLIB_COPYRUN_H
#define __GRAPHICS_NXGLIB_NXGLIB_COPYRUN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: nxgl_copyrun_*bpp
 *
 * Description:
 *   Copy a row from an image into run.
 *
 ****************************************************************************/

#if NXGLIB_BITSPERPIXEL == 1
static inline void
nxgl_copyrun_1bpp(FAR const uint8_t *src, FAR uint8_t *dest,
                  unsigned int remainder, size_t npixels)
{
  uint8_t indata;
  uint8_t outdata;
  uint8_t nextdata;
  unsigned int outpixels = 0;

  DEBUGASSERT(remainder > 0 && remainder < 8);

  /* Take only the first 8-remainder pixels from the first byte.
   * remainder is number between 1 and 7 (not zero!) that represents
   * the alignment of the pixel bits in the source.
   */

  indata = *src++;

#ifdef CONFIG_NX_PACKEDMSFIRST
  /* If CONFIG_NX_PACKEDMSFIRST is defined, then bits 0-(remainder-1)
   * are carried over to the first pass through the loop. For
   * example if remainder == 2:
   *
   * indata: xxAA AAAA maps to nextdata: AAAA AAxx
   */

  nextdata = (indata << remainder);

#else
  /* If CONFIG_NX_PACKEDMSFIRST is NOT defined, then bits (7-remainder)-7
   * are carried over to the first pass through the loop.  For example
   * if remainder == 2:
   *
   * indata: AAAA AAxx maps to nextdata: xxAA AAAA
   */

  nextdata = (indata >> remainder);

#endif

  /* Loop until all pixels have been packed into the destination.  Note:
   * a outpixels increments by 8 so a few extra pixels will be packed on
   * the output.  This should not be an issue.
   */

  while (outpixels < npixels)
    {
      /* Get the next byte from the source */

      indata  = *src++;
      outdata = nextdata;

      /* remainder is number between 1 and 7 (not zero!) that represents
       * the alignment of the pixel bits in the source.
       */

#ifdef CONFIG_NX_PACKEDMSFIRST
      /* If CONFIG_NX_PACKEDMSFIRST is defined, then bits (7-remainder)-7
       * are carried over from that last pass through the loop. For
       * example if remainder == 2:
       *
       *   nextdata = AAAA AAxx  - dest     = AAAA AABB
       *   src      = BBCC CCCC  - nextdata = CCCC CCxx
       */

       outdata |= (indata >> (8 - remainder));
       nextdata = (indata << remainder);
#else
      /* If CONFIG_NX_PACKEDMSFIRST is NOT defined, then bits 0-(remainder-1)
       * are carried over from that last pass through the loop .  For
       * example if remainder == 2:
       *
       *   nextdata = xxAA AAAA  - dest     = BBAA AAAA
       *   src      = CCCC CCBB  - nextdata = xxCC CCCC
       */

       outdata |= (indata << (8 - remainder));
       nextdata = (indata >> remainder);
#endif

      /* Transfer the byte to the run buffer */

      *dest++ = outdata;
      outpixels += 8;
    }
}

#elif NXGLIB_BITSPERPIXEL == 2
static inline void
nxgl_copyrun_2bpp(FAR const uint8_t *src, FAR uint8_t *dest,
                  unsigned int remainder, size_t npixels)
{
  uint8_t indata;
  uint8_t outdata;
  uint8_t nextdata;
  unsigned int outpixels = 0;
  unsigned int shift;

  DEBUGASSERT(remainder > 0 && remainder < 4);

  /* Take only the first 8-(2*remainder) pixels from the first byte.
   * remainder is number between 1 and 3 (not zero!) that represents
   * the alignment of the pixel bits in the source.
   */

  indata = *src++;
  shift  = (remainder << 1);

#ifdef CONFIG_NX_PACKEDMSFIRST
  /* If CONFIG_NX_PACKEDMSFIRST is defined, then bits 0-(2*remainder-1)
   * are carried over to the first pass through the loop. For
   * example if remainder == 1:
   *
   * indata: xxAA AAAA maps to nextdata: AAAA AAxx
   */

  nextdata = (indata << shift);

#else
  /* If CONFIG_NX_PACKEDMSFIRST is NOT defined, then bits (7-2*remainder)-7
   * are carried over to the first pass through the loop.  For example
   * if remainder == 1:
   *
   * indata: AAAA AAxx maps to nextdata: xxAA AAAA
   */

  nextdata = (indata >> shift);

#endif

  /* Loop until all pixels have been packed into the destination.  Note:
   * a outpixels increments by 8 so a few extra pixels will be packed on
   * the output.  This should not be an issue.
   */

  while (outpixels < npixels)
    {
      /* Get the next byte from the source */

      indata  = *src++;
      outdata = nextdata;

      /* remainder is number between 1 and 3 (not zero!) that represents
       * the alignment of the pixel bits in the source.
       */

#ifdef CONFIG_NX_PACKEDMSFIRST
      /* If CONFIG_NX_PACKEDMSFIRST is defined, then bits (7-2*remainder)-7
       * are carried over from that last pass through the loop. For example
       * if remainder == 1:
       *
       *   nextdata = AAAA AAxx  - dest     = AAAA AABB
       *   src      = BBCC CCCC  - nextdata = CCCC CCxx
       */

       outdata |= (indata >> (8 - shift));
       nextdata = (indata << shift);
#else
      /* If CONFIG_NX_PACKEDMSFIRST is NOT defined, then bits
       * 0-(2*remainder-1) are carried over from that last pass through the
       * loop.  For example if remainder == 1:
       *
       *   nextdata = xxAA AAAA  - dest     = BBAA AAAA
       *   src      = CCCC CCBB  - nextdata = xxCC CCCC
       */

       outdata |= (indata << (8 - shift));
       nextdata = (indata >> shift);
#endif

      /* Transfer the byte to the run buffer */

      *dest++ = outdata;
      outpixels += 4;
    }
}

#elif NXGLIB_BITSPERPIXEL == 4
static inline void
nxgl_copyrun_4bpp(FAR const uint8_t *src, FAR uint8_t *dest,
                  unsigned int remainder, size_t npixels)
{
  uint8_t indata;
  uint8_t outdata;
  uint8_t nextdata;
  unsigned int outpixels = 0;

  DEBUGASSERT(remainder == 1);

  /* Take only the first 8-remainder pixels from the first byte.
   * remainder is number between 1 and 3 (not zero!) that represents
   * the alignment of the pixel bits in the source.
   */

  indata = *src++;

#ifdef CONFIG_NX_PACKEDMSFIRST
  /* If CONFIG_NX_PACKEDMSFIRST is defined, then bits 0-3
   * are carried over to the first pass through the loop. For
   * example:
   *
   * indata: xxxx AAAA maps to nextdata: AAAA xxxx
   */

  nextdata = (indata << 4);

#else
  /* If CONFIG_NX_PACKEDMSFIRST is NOT defined, then bits 4-7
   * are carried over to the first pass through the loop.  For example:
   *
   * indata: AAAA xxxx maps to nextdata: xxxx AAAA
   */

  nextdata = (indata >> 4);

#endif

  /* Loop until all pixels have been packed into the destination.  Note:
   * a outpixels increments by 8 so a few extra pixels will be packed on
   * the output.  This should not be an issue.
   */

  while (outpixels < npixels)
    {
      /* Get the next byte from the source */

      indata  = *src++;
      outdata = nextdata;

      /* remainder is number between 1 and 3 (not zero!) that represents
       * the alignment of the pixel bits in the source.
       */

#ifdef CONFIG_NX_PACKEDMSFIRST
      /* If CONFIG_NX_PACKEDMSFIRST is defined, then bits 4-7
       * are carried over from that last pass through the loop (or are
       * ignored initially. For example if remainder == 1:
       *
       *   nextdata = AAAA xxxx  - dest     = AAAA BBBB
       *   src      = BBBB CCCC  - nextdata = CCCC xxxx
       */

      outdata |= (indata >> 4);
      nextdata = (indata << 4);
#else
      /* If CONFIG_NX_PACKEDMSFIRST is NOT defined, then bits 0-(remainder-1)
       * are carried over from that last pass through the loop (or are
       * ignored initially).  For example if remainder == 2:
       *
       *   nextdata = xxAA AAAA  - dest     = BBAA AAAA
       *   src      = CCCC CCBB  - nextdata = xxCC CCCC
       */

      outdata |= (indata << 4);
      nextdata = (indata >> 4);
#endif

      /* Transfer the byte to the run buffer */

      *dest++ = outdata;
      outpixels += 2;
    }
}
#endif
#endif /* __GRAPHICS_NXGLIB_NXGLIB_COPYRUN_H */
