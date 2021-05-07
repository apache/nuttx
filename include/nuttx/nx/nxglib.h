/****************************************************************************
 * include/nuttx/nx/nxglib.h
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

#ifndef __INCLUDE_NUTTX_NX_NXGLIB_H
#define __INCLUDE_NUTTX_NX_NXGLIB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <fixedmath.h>

#ifdef CONFIG_NX_LCDDRIVER
#  include <nuttx/lcd/lcd.h>
#else
#  include <nuttx/video/fb.h>
#endif

#include <nuttx/nx/nxtypes.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_NX_NPLANES
#  define CONFIG_NX_NPLANES  1  /* Max number of color planes supported */
#endif

/* Driver Selection *********************************************************/

/* NX_DRIVERTYPE selects either the framebuffer or LCD driver;
 * NX_PLANINFO_TYPE hides the difference in the framebuffer and LCD driver
 * plane types. defines are used instead of a typedefs to avoid type
 * mismatches.
 */

#ifdef CONFIG_NX_LCDDRIVER
#  define NX_DRIVERTYPE    struct lcd_dev_s
#  define NX_PLANEINFOTYPE struct lcd_planeinfo_s
#else
#  define NX_DRIVERTYPE    struct fb_vtable_s
#  define NX_PLANEINFOTYPE struct fb_planeinfo_s
#endif

/* NXGL Macros **************************************************************/

/* Mnemonics for indices */

#define NX_TOP_NDX           (0)
#define NX_LEFT_NDX          (1)
#define NX_RIGHT_NDX         (2)
#define NX_BOTTOM_NDX        (3)

/* Handy macros */

#define ngl_min(a,b)       ((a) < (b) ? (a) : (b))
#define ngl_max(a,b)       ((a) > (b) ? (a) : (b))
#define ngl_swap(a,b,t)    do { t = a; a = b; b = t; } while (0)
#define ngl_clipl(a,mn)    ((a) < (mn) ? (mn) : (a))
#define ngl_clipr(a,mx)    ((a) > (mx) ? (mx) : (a))
#define ngl_clip(a,mx,mn)  ((a) < (mn) ? (mn) : (a) > (mx) ? (mx) : (a))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* NXGLIB types are defined in nxtype.h.  This is done in order to avoid
 * circular include files dependencies by files included by this header
 * file that also require NXGLIB types.
 */

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
 * Name: nxgl_rgb2yuv
 *
 * Description:
 *   Convert 8-bit RGB triplet to 8-bit YUV triplet
 *
 ****************************************************************************/

void nxgl_rgb2yuv(uint8_t r, uint8_t g, uint8_t b,
                  uint8_t *y, uint8_t *u, uint8_t *v);

/****************************************************************************
 * Name: nxgl_yuv2rgb
 *
 * Description:
 *   Convert 8-bit RGB triplet to 8-bit YUV triplet
 *
 ****************************************************************************/

void nxgl_yuv2rgb(uint8_t y, uint8_t u, uint8_t v,
                  uint8_t *r, uint8_t *g, uint8_t *b);

/****************************************************************************
 * Name: nxgl_area2rect
 *
 * Description:
 *   Convert nxgl_rect_s to fb_area_s.
 *
 ****************************************************************************/

void nxgl_area2rect(FAR struct nxgl_rect_s *dest,
                    FAR const struct fb_area_s *src);

/****************************************************************************
 * Name: nxgl_rect2area
 *
 * Description:
 *   Convert nxgl_rect_s to fb_area_s.
 *
 ****************************************************************************/

void nxgl_rect2area(FAR struct fb_area_s *dest,
                    FAR const struct nxgl_rect_s *src);

/****************************************************************************
 * Name: nxgl_rectcopy
 *
 * Description:
 *   This is essentially memcpy for rectangles.  We don't do structure
 *   assignments because some compilers are not good at that.
 *
 ****************************************************************************/

void nxgl_rectcopy(FAR struct nxgl_rect_s *dest,
                   FAR const struct nxgl_rect_s *src);

/****************************************************************************
 * Name: nxgl_rectoffset
 *
 * Description:
 *   Offset the rectangle position by the specified dx, dy values.
 *
 ****************************************************************************/

void nxgl_rectoffset(FAR struct nxgl_rect_s *dest,
                     FAR const struct nxgl_rect_s *src,
                     nxgl_coord_t dx, nxgl_coord_t dy);

/****************************************************************************
 * Name: nxgl_vectoradd
 *
 * Description:
 *   Add two 2x1 vectors and save the result to a third.
 *
 ****************************************************************************/

void nxgl_vectoradd(FAR struct nxgl_point_s *dest,
                    FAR const struct nxgl_point_s *v1,
                    FAR const struct nxgl_point_s *v2);

/****************************************************************************
 * Name: nxgl_vectsubtract
 *
 * Description:
 *   Add subtract vector v2 from vector v1 and return the result in vector
 *   dest
 *
 ****************************************************************************/

void nxgl_vectsubtract(FAR struct nxgl_point_s *dest,
                       FAR const struct nxgl_point_s *v1,
                       FAR const struct nxgl_point_s *v2);

/****************************************************************************
 * Name: nxgl_rectintersect
 *
 * Description:
 *   Return the rectangle representing the intersection of the two rectangles
 *
 ****************************************************************************/

void nxgl_rectintersect(FAR struct nxgl_rect_s *dest,
                        FAR const struct nxgl_rect_s *src1,
                        FAR const struct nxgl_rect_s *src2);

/****************************************************************************
 * Name: nxgl_intersecting
 *
 * Description:
 *   Return true if the rectangles intersect.
 *
 ****************************************************************************/

bool nxgl_intersecting(FAR const struct nxgl_rect_s *rect1,
                       FAR const struct nxgl_rect_s *rect2);

/****************************************************************************
 * Name: nxgl_rectadd
 *
 * Description:
 *   Return the rectangle that contains exactly two other rectangles.
 *
 ****************************************************************************/

void nxgl_rectadd(FAR struct nxgl_rect_s *dest,
                  FAR const struct nxgl_rect_s *src1,
                  FAR const struct nxgl_rect_s *src2);

/****************************************************************************
 * Name: nxgl_rectunion
 *
 * Description:
 *   Given two rectangles, src1 and src2, return the larger rectangle that
 *   contains both, dest.
 *
 ****************************************************************************/

void nxgl_rectunion(FAR struct nxgl_rect_s *dest,
                    FAR const struct nxgl_rect_s *src1,
                    FAR const struct nxgl_rect_s *src2);

/****************************************************************************
 * Name: nxgl_nonintersecting
 *
 * Description:
 *   Return the regions of rectangle rect 1 that do not intersect with
 *   rect2.  This will be four rectangles ,some of which may be
 *   degenerate (and can be picked off with nxgl_nullrect)
 *
 ****************************************************************************/

void nxgl_nonintersecting(FAR struct nxgl_rect_s result[4],
                          FAR const struct nxgl_rect_s *rect1,
                          FAR const struct nxgl_rect_s *rect2);

/****************************************************************************
 * Name: nxgl_rectoverlap
 *
 * Description:
 *   Return true if the two rectangles overlap
 *
 ****************************************************************************/

bool nxgl_rectoverlap(FAR struct nxgl_rect_s *rect1,
                      FAR struct nxgl_rect_s *rect2);

/****************************************************************************
 * Name: nxgl_rectinside
 *
 * Description:
 *   Return true if the point pt lies within rect.
 *
 ****************************************************************************/

bool nxgl_rectinside(FAR const struct nxgl_rect_s *rect,
                     FAR const struct nxgl_point_s *pt);

/****************************************************************************
 * Name: nxgl_rectsize
 *
 * Description:
 *   Return the size of the specified rectangle.
 *
 ****************************************************************************/

void nxgl_rectsize(FAR struct nxgl_size_s *size,
                   FAR const struct nxgl_rect_s *rect);

/****************************************************************************
 * Name: nxgl_nullrect
 *
 * Description:
 *   Return true if the area of the rectangle is <= 0.
 *
 ****************************************************************************/

bool nxgl_nullrect(FAR const struct nxgl_rect_s *rect);

/****************************************************************************
 * Name: nxgl_runoffset
 *
 * Description:
 *   Offset the run position by the specified dx, dy values.
 *
 ****************************************************************************/

void nxgl_runoffset(FAR struct nxgl_run_s *dest,
                    FAR const struct nxgl_run_s *src,
                    nxgl_coord_t dx, nxgl_coord_t dy);

/****************************************************************************
 * Name: nxgl_runcopy
 *
 * Description:
 *   This is essentially memcpy for runs.  We don't do structure assignments
 *   because some compilers are not good at that.
 *
 ****************************************************************************/

void nxgl_runcopy(FAR struct nxgl_run_s *dest,
                  FAR const struct nxgl_run_s *src);

/****************************************************************************
 * Name: nxgl_trapoffset
 *
 * Description:
 *   Offset the trapezoid position by the specified dx, dy values.
 *
 ****************************************************************************/

void nxgl_trapoffset(FAR struct nxgl_trapezoid_s *dest,
                     FAR const struct nxgl_trapezoid_s *src,
                     nxgl_coord_t dx, nxgl_coord_t dy);

/****************************************************************************
 * Name: nxgl_trapcopy
 *
 * Description:
 *   This is essentially memcpy for trapezoids.  We don't do structure
 *   assignments because some compilers are not good at that.
 *
 ****************************************************************************/

void nxgl_trapcopy(FAR struct nxgl_trapezoid_s *dest,
                   FAR const struct nxgl_trapezoid_s *src);

/****************************************************************************
 * Name: nxgl_colorcopy
 *
 * Description:
 *   This is essentially memcpy for colors.  This does very little for us
 *   other than hide all of the conditional compilation for planar colors
 *   in one place.
 *
 ****************************************************************************/

#if CONFIG_NX_NPLANES == 1
#  define nxgl_colorcopy(d,s) do { (d)[0] = s[0]; } while (0)
#else
void nxgl_colorcopy(nxgl_mxpixel_t dest[CONFIG_NX_NPLANES],
                    const nxgl_mxpixel_t src[CONFIG_NX_NPLANES]);
#endif

/****************************************************************************
 * Name: nxgl_colorcmp
 *
 * Description:
 *   This is essentially memcmp for colors.  This does very little for us
 *   other than hide all of the conditional compilation for planar colors
 *   in one place.
 *
 ****************************************************************************/

#if CONFIG_NX_NPLANES == 1
#  define nxgl_colorcmp(d,s) ((d)[0] == s[0])
#else
bool nxgl_colorcmp(const nxgl_mxpixel_t color1[CONFIG_NX_NPLANES],
                   const nxgl_mxpixel_t color2[CONFIG_NX_NPLANES]);
#endif

/****************************************************************************
 * Name: nxgl_splitline
 *
 * Description:
 *   In the general case, a line with width can be represented as a
 *   parallelogram with a triangle at the top and bottom.  Triangles and
 *   parallelograms are both degenerate versions of a trapezoid.  This
 *   function breaks a wide line into triangles and trapezoids.  This
 *   function also detects other degenerate cases:
 *
 *   1. If y1 == y2 then the line is horizontal and is better represented
 *      as a rectangle.
 *   2. If x1 == x2 then the line is vertical and also better represented
 *      as a rectangle.
 *   3. If the width of the line is 1, then there are no triangles at the
 *      top and bottom (this may also be the case if the width is narrow
 *      and the line is near vertical).
 *   4. If the line is oriented is certain angles, it may consist only of
 *      the upper and lower triangles with no trapezoid in between.  In
 *      this case, 3 trapezoids will be returned, but traps[1] will be
 *      degenerate.
 *
 * Input Parameters:
 *   vector - A pointer to the vector described the line to be drawn.
 *   traps  - A pointer to a array of trapezoids (size 3).
 *   rect   - A pointer to a rectangle.
 *
 * Returned Value:
 *   0: Line successfully broken up into three trapezoids.  Values in
 *      traps[0], traps[1], and traps[2] are valid.
 *   1: Line successfully represented by one trapezoid. Value in traps[1]
 *      is valid.
 *   2: Line successfully represented by one rectangle. Value in rect is
 *      valid
 *  <0: On errors, a negated errno value is returned.
 *
 ****************************************************************************/

int nxgl_splitline(FAR struct nxgl_vector_s *vector,
                   FAR struct nxgl_trapezoid_s *traps,
                   FAR struct nxgl_rect_s *rect,
                   nxgl_coord_t linewidth);

/****************************************************************************
 * Name: nxgl_circlepts
 *
 * Description:
 *   Given a description of a circle, return a set of 16 points on the
 *   circumference of the circle.  These points may then be used by
 *   nx_drawcircle() or related APIs to draw a circle outline.
 *
 * Input Parameters:
 *   center - A pointer to the point that is the center of the circle
 *   radius - The radius of the circle in pixels.
 *   circle - A pointer the first entry in an array of 16 points where the
 *            circle points will be returned.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxgl_circlepts(FAR const struct nxgl_point_s *center,
                    nxgl_coord_t radius,
                    FAR struct nxgl_point_s *circle);

/****************************************************************************
 * Name: nxgl_circletraps
 *
 * Description:
 *   Given a description of a a circle, return 8 trapezoids that can be
 *   used to fill the circle by nx_fillcircle() and other interfaces.
 *
 * Input Parameters:
 *   center - A pointer to the point that is the center of the circle
 *   radius - The radius of the circle in pixels.
 *   circle - A pointer the first entry in an array of 8 trapezoids where
 *            the circle description will be returned.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxgl_circletraps(FAR const struct nxgl_point_s *center,
                      nxgl_coord_t radius,
                      FAR struct nxgl_trapezoid_s *circle);

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

uint32_t nxglib_rgb24_blend(uint32_t color1, uint32_t color2, ub16_t frac1);
uint16_t nxglib_rgb565_blend(uint16_t color1, uint16_t color2, ub16_t frac1);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_NX_NXGLIB_H */
