/****************************************************************************
 * graphics/nxglib/nxglib_bitblit.h
 *
 *   Copyright (C) 2008-2011 Gregory Nutt. All rights reserved.
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
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __GRAPHICS_NXGLIB_NXGLIB_BITBLIT_H */
