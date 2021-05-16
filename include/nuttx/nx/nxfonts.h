/****************************************************************************
 * include/nuttx/nx/nxfonts.h
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

#ifndef __INCLUDE_NUTTX_NX_NXFONTS_H
#define __INCLUDE_NUTTX_NX_NXFONTS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxglib.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/* Font Definitions *********************************************************/

/* Select the default font.  If no fonts are selected, then a compilation
 * error is likely down the road.
 */

/* Sans serif fonts */

#if defined(CONFIG_NXFONT_SANS23X27)       /* The "legacy," tiny NuttX font */
# define NXFONT_DEFAULT FONTID_SANS23X27

#elif defined(CONFIG_NXFONT_SANS17X22)
# define NXFONT_DEFAULT FONTID_SANS17X22

#elif defined(CONFIG_NXFONT_SANS20X26)
# define NXFONT_DEFAULT FONTID_SANS20X26

#elif defined(CONFIG_NXFONT_SANS22X29)
# define NXFONT_DEFAULT FONTID_SANS22X29

#elif defined(CONFIG_NXFONT_SANS28X37)
# define NXFONT_DEFAULT FONTID_SANS28X37

#elif defined(CONFIG_NXFONT_SANS39X48)
# define NXFONT_DEFAULT FONTID_SANS39X48

/* Sans serif bold fonts */

#elif defined(CONFIG_NXFONT_SANS17X23B)
# define NXFONT_DEFAULT FONTID_SANS17X23B

#elif defined(CONFIG_NXFONT_SANS20X27B)
# define NXFONT_DEFAULT FONTID_SANS20X27B

#elif defined(CONFIG_NXFONT_SANS22X29B)
# define NXFONT_DEFAULT FONTID_SANS22X29B

#elif defined(CONFIG_NXFONT_SANS28X37B)
# define NXFONT_DEFAULT FONTID_SANS28X37B

#elif defined(CONFIG_NXFONT_SANS40X49B)
# define NXFONT_DEFAULT FONTID_SANS40X49B

/* Serif fonts */

#elif defined(CONFIG_NXFONT_SERIF22X29)
# define NXFONT_DEFAULT FONTID_SERIF22X29

#elif defined(CONFIG_NXFONT_SERIF29X37)
# define NXFONT_DEFAULT FONTID_SERIF29X37

#elif defined(CONFIG_NXFONT_SERIF38X48)
# define NXFONT_DEFAULT FONTID_SERIF38X48

/* Serif bold fonts */

#elif defined(CONFIG_NXFONT_SERIF22X28B)
# define NXFONT_DEFAULT FONTID_SERIF22X28B

#elif defined(CONFIG_NXFONT_SERIF27X38B)
# define NXFONT_DEFAULT FONTID_SERIF27X38B

#elif defined(CONFIG_NXFONT_SERIF38X49B)
# define NXFONT_DEFAULT FONTID_SERIF38X49B

/* Pixel fonts */

#elif defined(CONFIG_NXFONT_PIXEL_UNICODE)
# define NXFONT_DEFAULT FONTID_PIXEL_UNICODE

#elif defined(CONFIG_NXFONT_PIXEL_LCD_MACHINE)
# define NXFONT_DEFAULT FONTID_PIXEL_LCD_MACHINE

/* X11 misc fixed fonts */

#elif defined(CONFIG_NXFONT_X11_MISC_FIXED_4X6)
# define NXFONT_DEFAULT FONTID_X11_MISC_FIXED_4X6

#elif defined(CONFIG_NXFONT_X11_MISC_FIXED_5X7)
# define NXFONT_DEFAULT FONTID_X11_MISC_FIXED_5X7

#elif defined(CONFIG_NXFONT_X11_MISC_FIXED_5X8)
# define NXFONT_DEFAULT FONTID_X11_MISC_FIXED_5X8

#elif defined(CONFIG_NXFONT_X11_MISC_FIXED_6X9)
# define NXFONT_DEFAULT FONTID_X11_MISC_FIXED_6X9

#elif defined(CONFIG_NXFONT_X11_MISC_FIXED_6X10)
# define NXFONT_DEFAULT FONTID_X11_MISC_FIXED_6X10

#elif defined(CONFIG_NXFONT_X11_MISC_FIXED_6X12)
# define NXFONT_DEFAULT FONTID_X11_MISC_FIXED_6X12

#elif defined(CONFIG_NXFONT_X11_MISC_FIXED_6X13)
# define NXFONT_DEFAULT FONTID_X11_MISC_FIXED_6X13

#elif defined(CONFIG_NXFONT_X11_MISC_FIXED_6X13B)
# define NXFONT_DEFAULT FONTID_X11_MISC_FIXED_6X13B

#elif defined(CONFIG_NXFONT_X11_MISC_FIXED_6X13O)
# define NXFONT_DEFAULT FONTID_X11_MISC_FIXED_6X13O

#elif defined(CONFIG_NXFONT_X11_MISC_FIXED_7X13)
# define NXFONT_DEFAULT FONTID_X11_MISC_FIXED_7X13

#elif defined(CONFIG_NXFONT_X11_MISC_FIXED_7X13B)
# define NXFONT_DEFAULT FONTID_X11_MISC_FIXED_7X13B

#elif defined(CONFIG_NXFONT_X11_MISC_FIXED_7X13O)
# define NXFONT_DEFAULT FONTID_X11_MISC_FIXED_7X13O

#elif defined(CONFIG_NXFONT_X11_MISC_FIXED_7X14)
# define NXFONT_DEFAULT FONTID_X11_MISC_FIXED_7X14

#elif defined(CONFIG_NXFONT_X11_MISC_FIXED_7X14B)
# define NXFONT_DEFAULT FONTID_X11_MISC_FIXED_7X14B

#elif defined(CONFIG_NXFONT_X11_MISC_FIXED_8X13)
# define NXFONT_DEFAULT FONTID_X11_MISC_FIXED_8X13

#elif defined(CONFIG_NXFONT_X11_MISC_FIXED_8X13B)
# define NXFONT_DEFAULT FONTID_X11_MISC_FIXED_8X13B

#elif defined(CONFIG_NXFONT_X11_MISC_FIXED_8X13O)
# define NXFONT_DEFAULT FONTID_X11_MISC_FIXED_8X13O

#elif defined(CONFIG_NXFONT_X11_MISC_FIXED_9X15)
# define NXFONT_DEFAULT FONTID_X11_MISC_FIXED_9X15

#elif defined(CONFIG_NXFONT_X11_MISC_FIXED_9X15B)
# define NXFONT_DEFAULT FONTID_X11_MISC_FIXED_9X15B

#elif defined(CONFIG_NXFONT_X11_MISC_FIXED_9X18)
# define NXFONT_DEFAULT FONTID_X11_MISC_FIXED_9X18

#elif defined(CONFIG_NXFONT_X11_MISC_FIXED_9X18B)
# define NXFONT_DEFAULT FONTID_X11_MISC_FIXED_9X18B

#elif defined(CONFIG_NXFONT_X11_MISC_FIXED_10X20)
# define NXFONT_DEFAULT FONTID_X11_MISC_FIXED_10X20

/* Mono-space fonts */

#elif defined(CONFIG_NXFONT_MONO5X8)
# define NXFONT_DEFAULT FONTID_MONO5X8

/* Tom Thumb mono-space 4x6 font */

#elif defined(CONFIG_NXFONT_TOM_THUMB_4X6)
# define NXFONT_DEFAULT FONTID_TOM_THUMB_4X6

#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Font Types ***************************************************************/

/* Font IDs */

enum nx_fontid_e
{
  FONTID_DEFAULT     = 0         /* The default font */

  /* Monospace fonts */

#ifdef CONFIG_NXFONT_MONO5X8
  , FONTID_MONO5X8 = 18          /* The 5x8 monospace font */
#endif

  /* Sans Serif fonts */

#ifdef CONFIG_NXFONT_SANS17X22
  , FONTID_SANS17X22 = 14        /* The 17x22 sans serif font */
#endif

#ifdef CONFIG_NXFONT_SANS20X26
  , FONTID_SANS20X26 = 15        /* The 20x26 sans serif font */
#endif

#ifdef CONFIG_NXFONT_SANS23X27
  , FONTID_SANS23X27 = 1         /* The 23x27 sans serif font */
#endif

#ifdef CONFIG_NXFONT_SANS22X29
  , FONTID_SANS22X29 = 2         /* The 22x29 sans serif font */
#endif

#ifdef CONFIG_NXFONT_SANS28X37
  , FONTID_SANS28X37 = 3         /* The 28x37 sans serif font */
#endif

#ifdef CONFIG_NXFONT_SANS39X48
  , FONTID_SANS39X48 = 4         /* The 39x48 sans serif font */
#endif

  /* Sans Serif bold fonts */

#ifdef CONFIG_NXFONT_SANS17X23B
  , FONTID_SANS17X23B = 16       /* The 17x23 sans bold font */
#endif

#ifdef CONFIG_NXFONT_SANS20X27B
  , FONTID_SANS20X27B = 17       /* The 20x27 sans bold font */
#endif

#ifdef CONFIG_NXFONT_SANS22X29B
  , FONTID_SANS22X29B = 5        /* The 22x29 sans bold font */
#endif

#ifdef CONFIG_NXFONT_SANS28X37B
  , FONTID_SANS28X37B = 6        /* The 28x37 sans bold font */
#endif

#ifdef CONFIG_NXFONT_SANS40X49B
  , FONTID_SANS40X49B = 7        /* The 40x49 sans bold font */
#endif

  /* Serif fonts */

#ifdef CONFIG_NXFONT_SERIF22X29
  , FONTID_SERIF22X29 = 8        /* The 22x29 serif font */
#endif

#ifdef CONFIG_NXFONT_SERIF29X37
  , FONTID_SERIF29X37 = 9        /* The 29x37 serif font */
#endif

#ifdef CONFIG_NXFONT_SERIF38X48
  , FONTID_SERIF38X48 = 10       /* The 38x48 serif font */
#endif

  /* Serif bold fonts */

#ifdef CONFIG_NXFONT_SERIF22X28B
  , FONTID_SERIF22X28B = 11      /* The 22x28 serif bold font */
#endif

#ifdef CONFIG_NXFONT_SERIF27X38B
  , FONTID_SERIF27X38B = 12      /* The 27x38 serif bold font */
#endif

#ifdef CONFIG_NXFONT_SERIF38X49B
  , FONTID_SERIF38X49B = 13      /* The 38x49 serif bold font */
#endif

  /* Pixel fonts */

#ifdef CONFIG_NXFONT_PIXEL_UNICODE
  , FONTID_PIXEL_UNICODE = 19      /* Pixel UniCode font */
#endif

#ifdef CONFIG_NXFONT_PIXEL_LCD_MACHINE
  , FONTID_PIXEL_LCD_MACHINE = 20  /* Pixel lcd machine font */
#endif

  /* X11 misc fixed fonts */

#ifdef CONFIG_NXFONT_X11_MISC_FIXED_4X6
  , FONTID_X11_MISC_FIXED_4X6 = 21      /* X11 misc fixed 4x6 */
#endif

#ifdef CONFIG_NXFONT_X11_MISC_FIXED_5X7
  , FONTID_X11_MISC_FIXED_5X7 = 22      /* X11 misc fixed 5x7 */
#endif

#ifdef CONFIG_NXFONT_X11_MISC_FIXED_5X8
  , FONTID_X11_MISC_FIXED_5X8 = 23      /* X11 misc fixed 5x8 */
#endif

#ifdef CONFIG_NXFONT_X11_MISC_FIXED_6X9
  , FONTID_X11_MISC_FIXED_6X9 = 24      /* X11 misc fixed 6x9 */
#endif

#ifdef CONFIG_NXFONT_X11_MISC_FIXED_6X10
  , FONTID_X11_MISC_FIXED_6X10 = 25     /* X11 misc fixed 6x10 */
#endif

#ifdef CONFIG_NXFONT_X11_MISC_FIXED_6X12
  , FONTID_X11_MISC_FIXED_6X12 = 26     /* X11 misc fixed 6x12 */
#endif

#ifdef CONFIG_NXFONT_X11_MISC_FIXED_6X13
  , FONTID_X11_MISC_FIXED_6X13 = 27     /* X11 misc fixed 6x13 */
#endif

#ifdef CONFIG_NXFONT_X11_MISC_FIXED_6X13B
  , FONTID_X11_MISC_FIXED_6X13B = 28    /* X11 misc fixed 6x13b */
#endif

#ifdef CONFIG_NXFONT_X11_MISC_FIXED_6X13O
  , FONTID_X11_MISC_FIXED_6X13O = 29    /* X11 misc fixed 6x13o */
#endif

#ifdef CONFIG_NXFONT_X11_MISC_FIXED_7X13
  , FONTID_X11_MISC_FIXED_7X13 = 30     /* X11 misc fixed 7x13 */
#endif

#ifdef CONFIG_NXFONT_X11_MISC_FIXED_7X13B
  , FONTID_X11_MISC_FIXED_7X13B = 31    /* X11 misc fixed 7x13b */
#endif

#ifdef CONFIG_NXFONT_X11_MISC_FIXED_7X13O
  , FONTID_X11_MISC_FIXED_7X13O = 32    /* X11 misc fixed 7x13o */
#endif

#ifdef CONFIG_NXFONT_X11_MISC_FIXED_7X14
  , FONTID_X11_MISC_FIXED_7X14 = 33     /* X11 misc fixed 7x14 */
#endif

#ifdef CONFIG_NXFONT_X11_MISC_FIXED_7X14B
  , FONTID_X11_MISC_FIXED_7X14B = 34    /* X11 misc fixed 7x14b */
#endif

#ifdef CONFIG_NXFONT_X11_MISC_FIXED_8X13
  , FONTID_X11_MISC_FIXED_8X13 = 35     /* X11 misc fixed 8x13 */
#endif

#ifdef CONFIG_NXFONT_X11_MISC_FIXED_8X13B
  , FONTID_X11_MISC_FIXED_8X13B = 36    /* X11 misc fixed 8x13b */
#endif

#ifdef CONFIG_NXFONT_X11_MISC_FIXED_8X13O
  , FONTID_X11_MISC_FIXED_8X13O = 37    /* X11 misc fixed 8x13o */
#endif

#ifdef CONFIG_NXFONT_X11_MISC_FIXED_9X15
  , FONTID_X11_MISC_FIXED_9X15 = 38     /* X11 misc fixed 9x15 */
#endif

#ifdef CONFIG_NXFONT_X11_MISC_FIXED_9X15B
  , FONTID_X11_MISC_FIXED_9X15B = 39    /* X11 misc fixed 9x15b */
#endif

#ifdef CONFIG_NXFONT_X11_MISC_FIXED_9X18
  , FONTID_X11_MISC_FIXED_9X18 = 40     /* X11 misc fixed 9x18 */
#endif

#ifdef CONFIG_NXFONT_X11_MISC_FIXED_9X18B
  , FONTID_X11_MISC_FIXED_9X18B = 41    /* X11 misc fixed 9x18b */
#endif

#ifdef CONFIG_NXFONT_X11_MISC_FIXED_10X20
  , FONTID_X11_MISC_FIXED_10X20 = 42    /* X11 misc fixed 10x20 */
#endif

#ifdef CONFIG_NXFONT_TOM_THUMB_4X6
  , FONTID_TOM_THUMB_4X6 = 43           /* Tom Thumb monospace 4x6 */
#endif
};

/* This structures provides the metrics for one glyph */

struct nx_fontmetric_s
{
  uint32_t stride   : 3;      /* Width of one font row in bytes */
  uint32_t width    : 6;      /* Width of the font in bits */
  uint32_t height   : 6;      /* Height of the font in rows */
  uint32_t xoffset  : 6;      /* Top, left-hand corner X-offset in pixels */
  uint32_t yoffset  : 6;      /* Top, left-hand corner y-offset in pixels */
  uint32_t unused   : 5;
};

/* This structure binds the glyph metrics to the glyph bitmap */

struct nx_fontbitmap_s
{
  struct nx_fontmetric_s metric; /* Character metrics */
  FAR const uint8_t *bitmap;     /* Pointer to the character bitmap */
};

/* This structure describes one contiguous grouping of glyphs that
 * can be described by an array starting with encoding 'first' and
 * extending through (first + nchars - 1).
 */

struct nx_fontset_s
{
  uint8_t  first;             /* First bitmap character code */
  uint8_t  nchars;            /* Number of bitmap character codes */
  FAR const struct nx_fontbitmap_s *bitmap;
};

/* This structure describes the overall metrics of the fontset */

struct nx_font_s
{
  uint8_t  mxheight;          /* Max height of one glyph in rows */
  uint8_t  mxwidth;           /* Max width of any glyph in pixels */
  uint8_t  mxbits;            /* Max number of bits per character code */
  uint8_t  spwidth;           /* The width of a space in pixels */
};

/* Finally, this structure defines everything about the font set */

struct nx_fontpackage_s
{
  uint8_t id;                            /* The font ID */
  FAR const struct nx_font_s    metrics; /* Font set metrics */
  FAR const struct nx_fontset_s font7;   /* Fonts for 7-bit encoding */
#if CONFIG_NXFONTS_CHARBITS >= 8
  FAR const struct nx_fontset_s font8;   /* Fonts for 8-bit encoding */
#endif
};

/* Font Cache ***************************************************************/

/* Opaque handle used to reference a font cache */

typedef FAR void *FCACHE;

/* Describes one cached font glyph */

struct nxfonts_glyph_s
{
  FAR struct nxfonts_glyph_s *flink;   /* Implements a singly linked list */
  uint8_t code;                        /* Character code */
  uint8_t height;                      /* Height of this glyph (in rows) */
  uint8_t width;                       /* Width of this glyph (in pixels) */
  uint8_t stride;                      /* Width of the glyph row (in bytes) */
  FAR uint8_t bitmap[1];               /* Bitmap memory, actual size varies */
};

#define SIZEOF_NXFONTS_GLYPH_S(b) (sizeof(struct nxfonts_glyph_s) + (b) - 1)

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
# define EXTERN extern "C"
extern "C"
{
#else
# define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nxf_getfonthandle
 *
 * Description:
 *   Given a numeric font ID, return a handle that may be subsequently be
 *   used to access the font data sets.
 *
 * Input Parameters:
 *   fontid:  Identifies the font set to get
 *
 * Returned Value:
 *   On success, a non-NULL font handle is returned.
 *
 ****************************************************************************/

NXHANDLE nxf_getfonthandle(enum nx_fontid_e fontid);

/****************************************************************************
 * Name: nxf_getfontset
 *
 * Description:
 *   Return information about the current font set
 *
 * Input Parameters:
 *   handle:  A font handle previously returned by nxf_getfonthandle()
 *
 ****************************************************************************/

FAR const struct nx_font_s *nxf_getfontset(NXHANDLE handle);

/****************************************************************************
 * Name: nxf_getbitmap
 *
 * Description:
 *   Return font bitmap information for the selected character encoding.
 *
 * Input Parameters:
 *   handle:  A font handle previously returned by nxf_getfonthandle()
 *   ch:      Character code whose bitmap is requested
 *
 * Returned Value:
 *   An instance of struct nx_fontbitmap_s describing the glyph.
 *
 ****************************************************************************/

FAR const struct nx_fontbitmap_s *nxf_getbitmap(NXHANDLE handle,
                                                uint16_t ch);

/****************************************************************************
 * Name: nxf_convert_*bpp
 *
 * Description:
 *   Convert the 1BPP font to a new pixel depth
 *
 * Input Parameters:
 *   dest   - The destination buffer provided by the caller.
 *   height - The max height of the returned char in rows
 *   width  - The max width of the returned char in pixels
 *   stride - The width of the destination buffer in bytes
 *   bm     - Describes the character glyph to convert
 *   color  - The color to use for '1' bits in the font bitmap
 *            (0 bits are transparent)
 *
 * Returned Value:
 *  OK on Success, ERROR: on failure with errno set appropriately.
 *  (never fails)
 *
 ****************************************************************************/

int nxf_convert_1bpp(FAR uint8_t *dest, uint16_t height,
                     uint16_t width, uint16_t stride,
                     FAR const struct nx_fontbitmap_s *bm,
                     nxgl_mxpixel_t color);
int nxf_convert_2bpp(FAR uint8_t *dest, uint16_t height,
                     uint16_t width, uint16_t stride,
                     FAR const struct nx_fontbitmap_s *bm,
                     nxgl_mxpixel_t color);
int nxf_convert_4bpp(FAR uint8_t *dest, uint16_t height,
                     uint16_t width, uint16_t stride,
                     FAR const struct nx_fontbitmap_s *bm,
                     nxgl_mxpixel_t color);
int nxf_convert_8bpp(FAR uint8_t *dest, uint16_t height,
                     uint16_t width, uint16_t stride,
                     FAR const struct nx_fontbitmap_s *bm,
                     nxgl_mxpixel_t color);
int nxf_convert_16bpp(FAR uint16_t *dest, uint16_t height,
                      uint16_t width, uint16_t stride,
                      FAR const struct nx_fontbitmap_s *bm,
                      nxgl_mxpixel_t color);
int nxf_convert_24bpp(FAR uint32_t *dest, uint16_t height,
                      uint16_t width, uint16_t stride,
                      FAR const struct nx_fontbitmap_s *bm,
                      nxgl_mxpixel_t color);
int nxf_convert_32bpp(FAR uint32_t *dest, uint16_t height,
                      uint16_t width, uint16_t stride,
                      FAR const struct nx_fontbitmap_s *bm,
                      nxgl_mxpixel_t color);

/****************************************************************************
 * Name: nxf_cache_connect
 *
 * Description:
 *   Create a new font cache for the provided 'fontid'.  If the cache
 *   already, then just increment a reference count return the handle for
 *   the existing font cache.
 *
 * Input Parameters:
 *   fontid    - Identifies the font supported by this cache
 *   fgcolor   - Foreground color
 *   bgcolor   - Background color
 *   bpp       - Bits per pixel
 *   maxglyphs - Maximum number of glyphs permitted in the cache
 *
 * Returned Value:
 *   On success a non-NULL handle is returned that then may sequently be
 *   used with nxf_getglyph() to extract fonts from the font cache.  NULL
 *   returned on any failure with the errno value set to indicate the nature
 *   of the error.
 *
 ****************************************************************************/

FCACHE nxf_cache_connect(enum nx_fontid_e fontid,
                         nxgl_mxpixel_t fgcolor, nxgl_mxpixel_t bgcolor,
                         int bpp, int maxglyph);

/****************************************************************************
 * Name: nxf_cache_disconnect
 *
 * Description:
 *   Decrement the reference count on the font cache and, if the reference
 *   count goes to zero, free all resources used by the font cache.  The
 *   font handler is invalid upon return in either case.
 *
 * Input Parameters:
 *   fhandle - A font cache handler previously returned by
 *             nxf_cache_connect();
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxf_cache_disconnect(FCACHE fhandle);

/****************************************************************************
 * Name: nxf_cache_getfonthandle
 *
 * Description:
 *   Return the handle to the font set used by this instance of the font
 *   cache.
 *
 * Input Parameters:
 *   fhandle - A font cache handle previously returned by
 *             nxf_cache_connect();
 *
 * Returned Value:
 *   Zero (OK) is returned if the metrics were
 *
 * Returned Value:
 *   On success, a non-NULL font handle is returned.
 *
 ****************************************************************************/

NXHANDLE nxf_cache_getfonthandle(FCACHE fhandle);

/****************************************************************************
 * Name: nxf_cache_getglyph
 *
 * Description:
 *   Get the font glyph for the character code 'ch' from the font cache.  If
 *   the glyph for that character code does not exist in the font cache, it
 *   be rendered.
 *
 * Returned Value:
 *   On success, a non-NULL pointer to the rendered glyph in the font cache
 *   is returned.  NULL is returned on any failure.
 *
 ****************************************************************************/

FAR const struct nxfonts_glyph_s *nxf_cache_getglyph(FCACHE fhandle,
                                                     uint8_t ch);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_NX_NXFONTS_H */
