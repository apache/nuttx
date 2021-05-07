/****************************************************************************
 * graphics/nxglib/nxglib_bitblit.h
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

#ifndef __GRAPHICS_NXGLIB_NXGLIB_BITBLIT_H
#define __GRAPHICS_NXGLIB_NXGLIB_BITBLIT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/nx/nxglib.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Make sure the bits-per-pixel value has been set by the includer of
 * this header file.
 */

#ifndef NXGLIB_BITSPERPIXEL
#  error "NXGLIB_BITSPERPIXEL must be defined before including this header file"
#endif

/* Anti-aliasing can only be supported for 16-, 24-, and 32-bit RGB types */

#if NXGLIB_BITSPERPIXEL < 16
#  undef CONFIG_NX_ANTIALIASING
#endif

/* Set up bit blit macros for this BPP */

#if NXGLIB_BITSPERPIXEL == 1

#  define NXGL_PIXELSHIFT          3
#  define NXGL_PIXELMASK           7
#  define NXGL_MULTIPIXEL(p)       ((p) ? 0xff : 0x00)
#  define NXGL_PIXEL_T             uint8_t

#elif NXGLIB_BITSPERPIXEL == 2

#  define NXGL_PIXELSHIFT          2
#  define NXGL_PIXELMASK           3
#  define NXGL_MULTIPIXEL(p)       ((uint8_t)(p) << 6 | (uint8_t)(p) << 4 | (uint8_t)(p) << 2 | (p))
#  define NXGL_PIXEL_T             uint8_t

#elif NXGLIB_BITSPERPIXEL == 4

#  define NXGL_PIXELSHIFT          1
#  define NXGL_PIXELMASK           1
#  define NXGL_MULTIPIXEL(p)       ((uint8_t)(p) << 4 | (p))
#  define NXGL_PIXEL_T             uint8_t

#elif NXGLIB_BITSPERPIXEL == 8

#  define NXGL_SCALEX(x)           (x)
#  define NXGL_PIXEL_T             uint8_t

#elif NXGLIB_BITSPERPIXEL == 16

#  define NXGL_SCALEX(x)           ((x) << 1)
#  define NXGL_PIXEL_T             uint16_t
#  define NXGL_BLENDER             nxglib_rgb565_blend

#elif NXGLIB_BITSPERPIXEL == 24

#  define NXGL_SCALEX(x)           (((x) << 1) + (x))
#  define NXGL_PIXEL_T             uint32_t
#  define NXGL_BLENDER             nxglib_rgb24_blend

#elif NXGLIB_BITSPERPIXEL == 32

#  define NXGL_SCALEX(x)           ((x) << 2)
#  define NXGL_PIXEL_T             uint32_t
#  define NXGL_BLENDER             nxglib_rgb24_blend

#endif

#if NXGLIB_BITSPERPIXEL < 8

#  define NXGL_SCALEX(x)           ((x) >> NXGL_PIXELSHIFT)
#  define NXGL_REMAINDERX(x)       ((x) & NXGL_PIXELMASK)
#  define NXGL_ALIGNDOWN(x)        ((x) & ~NXGL_PIXELMASK)
#  define NXGL_ALIGNUP(x)          (((x) + NXGL_PIXELMASK) & ~NXGL_PIXELMASK)

#  define NXGL_MEMSET(dest,value,width) \
   { \
     FAR uint8_t *_ptr = (FAR uint8_t*)(dest); \
     int        _nby = NXGL_SCALEX(width); \
     while (_nby--) \
       { \
         *_ptr++ = (value); \
       } \
   }

#  define NXGL_MEMCPY(dest,src,width) \
   { \
     FAR uint8_t *_dptr = (FAR uint8_t*)(dest); \
     FAR uint8_t *_sptr = (FAR uint8_t*)(src); \
     int        _nby  = NXGL_SCALEX(width); \
     while (_nby--) \
       { \
         *_dptr++ = *_sptr++; \
       } \
   }

#elif NXGLIB_BITSPERPIXEL == 24

#  define NXGL_MEMSET(dest,value,width) \
   { \
     FAR uint8_t *_ptr  = (FAR uint8_t*)(dest); \
     nxgl_coord_t _npix = (width); \
     while (_npix--) \
       { \
         *_ptr++ = (value); \
         *_ptr++ = (value) >> 8; \
         *_ptr++ = (value) >> 16; \
       } \
   }

#  define NXGL_MEMCPY(dest,src,width) \
   { \
     FAR uint8_t *_dptr = (FAR uint8_t*)(dest); \
     FAR uint8_t *_sptr = (FAR uint8_t*)(src); \
     nxgl_coord_t _npix = (width); \
     while (_npix--) \
       { \
         *_dptr++ = *_sptr++; \
         *_dptr++ = *_sptr++; \
         *_dptr++ = *_sptr++; \
       } \
   }

#ifdef CONFIG_NX_ANTIALIASING

#  define NXGL_BLEND(dest,color1,frac) \
   { \
     FAR uint8_t *_dptr = (FAR uint8_t*)(dest); \
     uint32_t color2; \
     uint32_t blend; \
     color2 = ((uint32_t)_dptr[0] << 16) | \
              ((uint32_t)_dptr[1] << 8) | \
               (uint32_t)_dptr[2]; \
     blend =  NXGL_BLENDER(color1, color2, frac); \
     *_dptr++ = (blend >> 16) & 0xff; \
     *_dptr++ = (blend >> 8)  & 0xff; \
     *_dptr++ =  blend        & 0xff; \
   }

#endif /* CONFIG_NX_ANTIALIASING */
#else /* NXGLIB_BITSPERPIXEL == 16 || NXGLIB_BITSPERPIXEL == 32 */

#  define NXGL_MEMSET(dest,value,width) \
   { \
     FAR NXGL_PIXEL_T *_ptr = (FAR NXGL_PIXEL_T*)(dest); \
     nxgl_coord_t     _npix = (width); \
     while (_npix--) \
       { \
         *_ptr++ = (value); \
       } \
   }

#  define NXGL_MEMCPY(dest,src,width) \
   { \
     FAR NXGL_PIXEL_T *_dptr = (FAR NXGL_PIXEL_T*)(dest); \
     FAR NXGL_PIXEL_T *_sptr = (FAR NXGL_PIXEL_T*)(src); \
     nxgl_coord_t      _npix = (width); \
     while (_npix--) \
       { \
         *_dptr++ = *_sptr++; \
       } \
   }

#ifdef CONFIG_NX_ANTIALIASING

#  define NXGL_BLEND(dest,color1,frac) \
   { \
     FAR NXGL_PIXEL_T *_dptr = (FAR NXGL_PIXEL_T*)(dest); \
     NXGL_PIXEL_T color2 = *_dptr; \
     *_dptr = NXGL_BLENDER(color1, color2, frac); \
   }

#endif /* CONFIG_NX_ANTIALIASING */
#endif /* NXGLIB_BITSPERPIXEL */

/* Form a function name by concatenating two strings */

#define _NXGL_FUNCNAME(a,b) a ## b
#define NXGL_FUNCNAME(a,b)  _NXGL_FUNCNAME(a,b)

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __GRAPHICS_NXGLIB_NXGLIB_BITBLIT_H */
