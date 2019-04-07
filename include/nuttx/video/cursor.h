/****************************************************************************
 * include/nuttx/video/cursor.h
 *
 *   Copyright (C) 2008-2011, 2013, 2016-2019 Gregory Nutt. All rights
 *     reserved.
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

#ifndef __INCLUDE_NUTTX_VIDEO_CURSOR_H
#define __INCLUDE_NUTTX_VIDEO_CURSOR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>

#ifdef CONFIG_NX
#  include <nuttx/nx/nxtypes.h>
#else
#  include <nuttx/video/fb.h>
#endif

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef CONFIG_NX
/* If NX is defined, make types agree with NX */

typedef nxgl_coord_t   cursor_coord_t;
typedef nxgl_mxpixel_t cursor_color_t[CONFIG_NX_NPLANES];

#else
/* Otherwise use framebuffer and worst case types */

typedef fb_coord_t     cursor_coord_t;
typedef uint32_t       cursor_color_t;

#endif

/* For cursor controllers that support custem cursor images, this structure
 * is used to provide the cursor image.
 *
 * The image is provided a a 2-bits-per-pixel image.  The two bit incoding
 * is as followings:
 *
 * 00 - The transparent background
 * 01 - Color1:  The main color of the cursor
 * 10 - Color2:  The color of any border
 * 11 - Color3:  A blend color for better imaging (fake anti-aliasing).
 */

struct cursor_image_s
{
  cursor_coord_t width;        /* Width of the cursor image in pixels */
  cursor_coord_t height;       /* Height of the cursor image in pixels */
  cursor_color_t color1;       /* Color1 is main color of the cursor */
  cursor_color_t color2;       /* Color2 is color of any border */
  cursor_color_t color3;       /* Color3 is the blended color */
  FAR const uint8_t *image;    /* Pointer to bitmap image data */
};

/* The following structure defines the cursor position.  If CONFIG_NX=y,
 * this structure is equivalent to struct nxgl_pos_s.
 */

struct cursor_pos_s
{
  cursor_coord_t x;            /* X position in pixels */
  cursor_coord_t y;            /* Y position in rows */
};

/* If the hardware supports setting the cursor size, then this structure
 * is used to provide the cursor size.  If CONFIG_NX=y, this structure is
 * equivalent to struct nxgl_size_s.
 */

struct cursor_size_s
{
  cursor_coord_t h;            /* Height in rows */
  cursor_coord_t w;            /* Width in pixels */
};

#endif /* __INCLUDE_NUTTX_VIDEO_CURSOR_H */
