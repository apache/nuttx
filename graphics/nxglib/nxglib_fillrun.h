/****************************************************************************
 * graphics/nxglib/nxglib_fillrun.h
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

#ifndef __GRAPHICS_NXGLIB_NXGLIB_FILLRUN_H
#define __GRAPHICS_NXGLIB_NXGLIB_FILLRUN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <string.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if NXGLIB_BITSPERPIXEL < 16
#  define NXGLIB_RUNTYPE uint8_t
#elif NXGLIB_BITSPERPIXEL == 16
#  define NXGLIB_RUNTYPE uint16_t
#else
#  define NXGLIB_RUNTYPE uint32_t
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#if NXGLIB_BITSPERPIXEL == 2
static uint8_t g_wide_2bpp[4] =
{
  0x00, 0x55, 0xaa, 0xff
};
#endif

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: nxgl_fillrun_*bpp
 *
 * Description:
 *   fill a run with the specified color.
 *
 ****************************************************************************/

#if NXGLIB_BITSPERPIXEL == 1
static inline void nxgl_fillrun_1bpp(FAR uint8_t *run, nxgl_mxpixel_t color,
                                     size_t npixels)
{
  /* Get the number of bytes to fill */

  unsigned int nbytes = (npixels + 7) >> 3;

  /* Get the value of the byte to fill */

  uint8_t wide = (color & 1) != 0 ? 0xff : 0x00;

  /* Fill the run with the color (it is okay to run a fractional byte over
   * the end)
   */

  memset(run, wide, nbytes);
}

#elif NXGLIB_BITSPERPIXEL == 2
static inline void nxgl_fillrun_2bpp(FAR uint8_t *run, nxgl_mxpixel_t color,
                                     size_t npixels)
{
  /* Get the number of bytes to fill */

  unsigned int nbytes = (npixels + 3) >> 2;

  /* Get the value of the byte to fill */

  uint8_t wide = g_wide_2bpp[color & 3];

  /* Fill the run with the color (it is okay to run a fractional byte over
   * the end)
   */

  memset(run, wide, nbytes);
}

#elif NXGLIB_BITSPERPIXEL == 4
static inline void nxgl_fillrun_4bpp(FAR uint8_t *run, nxgl_mxpixel_t color,
                                     size_t npixels)
{
  /* Get the number of bytes to fill */

  unsigned int nbytes = (npixels + 1) >> 1;

  /* Get the value of the byte to fill */

  uint8_t narrow = (uint8_t)color & 0x0f;
  uint8_t wide   = narrow | (narrow << 4);

  /* Fill the run with the color (it is okay to run a fractional byte over
   * the end)
   */

  memset(run, wide, nbytes);
}

#elif NXGLIB_BITSPERPIXEL == 8
static inline void nxgl_fillrun_8bpp(FAR uint8_t *run, nxgl_mxpixel_t color,
                                     size_t npixels)
{
  /* Fill the run with the color (it is okay to run a fractional byte overy
   * the end
   */

  memset(run, color, npixels);
}

#elif NXGLIB_BITSPERPIXEL == 16
static inline void nxgl_fillrun_16bpp(FAR uint16_t *run,
                                      nxgl_mxpixel_t color,
                                      size_t npixels)
{
  /* Fill the run with the color (it is okay to run a fractional byte overy
   * the end
   */

  while (npixels-- > 0)
    {
      *run++ = (uint16_t)color;
    }
}

#elif NXGLIB_BITSPERPIXEL == 24
static inline void nxgl_fillrun_24bpp(FAR uint32_t *run,
                                      nxgl_mxpixel_t color,
                                      size_t npixels)
{
  /* Fill the run with the color (it is okay to run a fractional byte overy
   * the end
   */

  while (npixels-- > 0)
    {
      *run++ = (uint32_t)color;
    }
}

#elif NXGLIB_BITSPERPIXEL == 32
static inline void nxgl_fillrun_32bpp(FAR uint32_t *run,
                                      nxgl_mxpixel_t color,
                                      size_t npixels)
{
  /* Fill the run with the color (it is okay to run a fractional byte overy
   * the end
   */

  while (npixels-- > 0)
    {
      *run++ = (uint32_t)color;
    }
}
#else
#  error "Unsupported value of NXGLIB_BITSPERPIXEL"
#endif
#endif /* __GRAPHICS_NXGLIB_NXGLIB_FILLRUN_H */
