/****************************************************************************
 * include/nuttx/nx/nxtypes.h
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

/* Describes a vector starting at pt1 and extending through pt2 */

struct nxgl_vector_s
{
  struct nxgl_point_s pt1; /* Start position */
  struct nxgl_point_s pt2; /* End position */
};

/* Describes a run, i.e., a horizontal line.
 * Note that the start/end positions have fractional precision.
 * This is necessary for good joining of trapezoids when a more complex
 * shape is decomposed into trapezoids.
 */

struct nxgl_run_s
{
  b16_t        x1;        /* Left X position, range: 0 to x2 */
  b16_t        x2;        /* Right X position, range: x1 to screen width - 1 */
  nxgl_coord_t y;         /* Top Y position, range: 0 to screen height - 1 */
};

/* Describes a horizontal trapezoid on the display in terms the run at the
 * top of the trapezoid and the run at the bottom.
 */

struct nxgl_trapezoid_s
{
  struct nxgl_run_s top;  /* Top run */
  struct nxgl_run_s bot;  /* bottom run */
};

#endif /* __INCLUDE_NUTTX_NX_TYPES_H */
