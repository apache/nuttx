/****************************************************************************
 * include/nuttx/nx/nxtypes.h
 *
 *   Copyright (C) 2008-2011, 2019 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_NUTTX_NX_TYPES_H
#define __INCLUDE_NUTTX_NX_TYPES_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <fixedmath.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Pixels *******************************************************************/

/* The size of graphics solutions can be reduced by disabling support for
 * specific resolutions.  One thing we can do, for example, is to select
 * the smallest common pixel representation:
 */

#if !defined(CONFIG_NX_DISABLE_32BPP) || !defined(CONFIG_NX_DISABLE_24BPP)
typedef uint32_t nxgl_mxpixel_t;
#elif !defined(CONFIG_NX_DISABLE_16BPP)
typedef uint16_t nxgl_mxpixel_t;
#else
typedef uint8_t  nxgl_mxpixel_t;
#endif

/* Graphics structures ******************************************************/

/* A given coordinate is limited to the screen height an width.  If either
 * of those values exceed 32,767 pixels, then the following will have to need
 * to change:
 */

typedef int16_t nxgl_coord_t;

/* Describes a point on the display */

struct nxgl_point_s
{
  nxgl_coord_t x;         /* X position, range: 0 to screen width - 1 */
  nxgl_coord_t y;         /* Y position, range: 0 to screen height - 1 */
};

/* Describes the size of a rectangular region */

struct nxgl_size_s
{
  nxgl_coord_t w;        /* Width in pixels */
  nxgl_coord_t h;        /* Height in rows */
};

/* Describes a positioned rectangle on the display */

struct nxgl_rect_s
{
  struct nxgl_point_s pt1; /* Upper, left-hand corner */
  struct nxgl_point_s pt2; /* Lower, right-hand corner */
};

/* Describes a vector starting at pt1 and extending throug pt2 */

struct nxgl_vector_s
{
  struct nxgl_point_s pt1; /* Start position */
  struct nxgl_point_s pt2; /* End position */
};

/* Describes a run, i.e., a horizontal line.  Note that the start/end positions
 * have fractional precision.  This is necessary for good joining of trapezoids
 * when a more complex shape is decomposed into trapezoids
 */

struct nxgl_run_s
{
  b16_t        x1;        /* Left X position, range: 0 to x2 */
  b16_t        x2;        /* Right X position, range: x1 to screen width - 1 */
  nxgl_coord_t y;         /* Top Y position, range: 0 to screen height - 1 */
};

/* Describes a horizontal trapezoid on the display in terms the run at the
 * top of the trapezoid and the run at the bottom
 */

struct nxgl_trapezoid_s
{
  struct nxgl_run_s top;  /* Top run */
  struct nxgl_run_s bot;  /* bottom run */
};

#endif /* __INCLUDE_NUTTX_NX_TYPES_H */
