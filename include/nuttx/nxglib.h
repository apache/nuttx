/****************************************************************************
 * include/nuttx/nxglib.h
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#ifndef __INCLUDE_NUTTX_NXGLIB_H
#define __INCLUDE_NUTTX_NXGLIB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <fixedmath.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#define ngl_min(a,b)       ((a) < (b) ? (a) : (b))
#define ngl_max(a,b)       ((a) > (b) ? (a) : (b))
#define ngl_swap(a,b,t)    do { t = a; a = b; b = t; } while (0);
#define ngl_clipl(a,mn)    ((a) < (mn) ? (mn) : (a))
#define ngl_clipr(a,mx)    ((a) > (mx) ? (mx) : (a))
#define ngl_clip(a,mx,mn)  ((a) < (mn) ? (mn) : (a) > (mx) ? (mx) : (a))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Pixels *******************************************************************/

/* The size of graphics solutions can be reduced by disabling support for
 * specific resolutions.  One thing we can do, for example, is to select
 * the smallest common pixel representation:
 */

#if !defined(CONFIG_NXGLIB_DISABLE_32BPP) || defined(CONFIG_NXGLIB_DISABLE_24BPP)
typedef uint32 nxgl_mxpixel_t;
#elif !defined(CONFIG_NXGLIB_DISABLE_16BPP)
typedef uint16 nxgl_mxpixel_t;
#else
typedef ubyte  nxgl_mxpixel_t;
#endif

/* Graphics structures ******************************************************/

/* A given coordinate is limited to the screen height an width.  If either
 * of those values exceed 32,767 pixels, then the following will have to need
 * to change:
 */

typedef sint16 nxgl_coord_t;

/* Describes a point on the display */

struct nxgl_point_s
{
  nxgl_coord_t x;         /* X position, range: 0 to screen width - 1 */
  nxgl_coord_t y;         /* Y position, range: 0 to screen height - 1 */
};

/* Describes a rectangle on the display */

struct nxgl_rect_s
{
  struct nxgl_point_s pt1; /* Upper, left-hand corner */
  struct nxgl_point_s pt2; /* Lower, right-hand corner */
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

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
# define EXTERN extern "C"
extern "C" {
#else
# define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Color conversons *********************************************************/

/****************************************************************************
 * Name: nxgl_rgb2yuv
 *
 * Description:
 *   Convert 8-bit RGB triplet to 8-bit YUV triplet
 *
 ****************************************************************************/

EXTERN void nxgl_rgb2yuv(ubyte r, ubyte g, ubyte b, ubyte *y, ubyte *u, ubyte *v);

/****************************************************************************
 * Name: nxgl_yuv2rgb
 *
 * Description:
 *   Convert 8-bit RGB triplet to 8-bit YUV triplet
 *
 ****************************************************************************/

EXTERN void nxgl_yuv2rgb(ubyte y, ubyte u, ubyte v, ubyte *r, ubyte *g, ubyte *b);

/* Rasterizers **************************************************************/

/****************************************************************************
 * Name: nxgl_fillrectangle_*bpp
 *
 * Descripton:
 *   Fill a rectangle region in the framebuffer memory with a fixed color
 *
 ****************************************************************************/

EXTERN void nxgl_fillrectangle_1bpp(FAR struct fb_planeinfo_s *pinfo,
                                    FAR const struct nxgl_rect_s *rect,
                                    nxgl_mxpixel_t color);
EXTERN void nxgl_fillrectangle_2bpp(FAR struct fb_planeinfo_s *pinfo,
                                    FAR const struct nxgl_rect_s *rect,
                                    nxgl_mxpixel_t color);
EXTERN void nxgl_fillrectangle_4bpp(FAR struct fb_planeinfo_s *pinfo,
                                    FAR const struct nxgl_rect_s *rect,
                                    nxgl_mxpixel_t color);
EXTERN void nxgl_fillrectangle_8bpp(FAR struct fb_planeinfo_s *pinfo,
                                    FAR const struct nxgl_rect_s *rect,
                                    nxgl_mxpixel_t color);
EXTERN void nxgl_fillrectangle_16bpp(FAR struct fb_planeinfo_s *pinfo,
                                     FAR const struct nxgl_rect_s *rect,
                                     nxgl_mxpixel_t color);
EXTERN void nxgl_fillrectangle_24bpp(FAR struct fb_planeinfo_s *pinfo,
                                     FAR const struct nxgl_rect_s *rect,
                                     nxgl_mxpixel_t color);
EXTERN void nxgl_fillrectangle_32bpp(FAR struct fb_planeinfo_s *pinfo,
                                     FAR const struct nxgl_rect_s *rect,
                                     nxgl_mxpixel_t color);

/****************************************************************************
 * Name: nxglib_filltrapezoid_*bpp
 *
 * Descripton:
 *   Fill a trapezoidal region in the framebuffer memory with a fixed color.
 *   This is useful for drawing complex shape -- (most) complex shapes can be
 *   broken into a set of trapezoids.
 *
 ****************************************************************************/

EXTERN void nxglib_filltrapezoid_1bpp(FAR struct fb_videoinfo_s *vinfo,
                                      FAR struct fb_planeinfo_s *pinfo,
                                      FAR const struct nxgl_trapezoid_s *trap,
                                      nxgl_mxpixel_t color);
EXTERN void nxglib_filltrapezoid_2bpp(FAR struct fb_videoinfo_s *vinfo,
                                      FAR struct fb_planeinfo_s *pinfo,
                                      FAR const struct nxgl_trapezoid_s *trap,
                                      nxgl_mxpixel_t color);
EXTERN void nxglib_filltrapezoid_4bpp(FAR struct fb_videoinfo_s *vinfo,
                                      FAR struct fb_planeinfo_s *pinfo,
                                      FAR const struct nxgl_trapezoid_s *trap,
                                      nxgl_mxpixel_t color);
EXTERN void nxglib_filltrapezoid_8bpp(FAR struct fb_videoinfo_s *vinfo,
                                      FAR struct fb_planeinfo_s *pinfo,
                                      FAR const struct nxgl_trapezoid_s *trap,
                                      nxgl_mxpixel_t color);
EXTERN void nxglib_filltrapezoid_16bpp(FAR struct fb_videoinfo_s *vinfo,
                                       FAR struct fb_planeinfo_s *pinfo,
                                       FAR const struct nxgl_trapezoid_s *trap,
                                       nxgl_mxpixel_t color);
EXTERN void nxglib_filltrapezoid_24bpp(FAR struct fb_videoinfo_s *vinfo,
                                       FAR struct fb_planeinfo_s *pinfo,
                                       FAR const struct nxgl_trapezoid_s *trap,
                                       nxgl_mxpixel_t color);
EXTERN void nxglib_filltrapezoid_32bpp(FAR struct fb_videoinfo_s *vinfo,
                                       FAR struct fb_planeinfo_s *pinfo,
                                       FAR const struct nxgl_trapezoid_s *trap,
                                       nxgl_mxpixel_t color);

/****************************************************************************
 * Name: nxgl_moverectangle_*bpp
 *
 * Descripton:
 *   Move a rectangular region from location to another in the
 *   framebuffer memory.
 *
 ****************************************************************************/

EXTERN void nxgl_moverectangle_1bpp(FAR struct fb_planeinfo_s *pinfo,
                                    FAR const struct nxgl_rect_s *rect,
                                    FAR struct nxgl_point_s *offset);
EXTERN void nxgl_moverectangle_2bpp(FAR struct fb_planeinfo_s *pinfo,
                                    FAR const struct nxgl_rect_s *rect,
                                    FAR struct nxgl_point_s *offset);
EXTERN void nxgl_moverectangle_4bpp(FAR struct fb_planeinfo_s *pinfo,
                                    FAR const struct nxgl_rect_s *rect,
                                    FAR struct nxgl_point_s *offset);
EXTERN void nxgl_moverectangle_8bpp(FAR struct fb_planeinfo_s *pinfo,
                                    FAR const struct nxgl_rect_s *rect,
                                    FAR struct nxgl_point_s *offset);
EXTERN void nxgl_moverectangle_16bpp(FAR struct fb_planeinfo_s *pinfo,
                                     FAR const struct nxgl_rect_s *rect,
                                     FAR struct nxgl_point_s *offset);
EXTERN void nxgl_moverectangle_24bpp(FAR struct fb_planeinfo_s *pinfo,
                                     FAR const struct nxgl_rect_s *rect,
                                     FAR struct nxgl_point_s *offset);
EXTERN void nxgl_moverectangle_32bpp(FAR struct fb_planeinfo_s *pinfo,
                                     FAR const struct nxgl_rect_s *rect,
                                     FAR struct nxgl_point_s *offset);

/****************************************************************************
 * Name: nxgl_copyrectangle_*bpp
 *
 * Descripton:
 *   Copy a rectangular bitmap image into the specific position in the
 *   framebuffer memory.
 *
 ****************************************************************************/

EXTERN void nxgl_copyrectangle_1bpp(FAR struct fb_planeinfo_s *pinfo,
                                    FAR const struct nxgl_rect_s *dest,
                                    FAR const void *src,
                                    FAR const struct nxgl_point_s *origin,
                                    unsigned int srcstride);
EXTERN void nxgl_copyrectangle_2bpp(FAR struct fb_planeinfo_s *pinfo,
                                    FAR const struct nxgl_rect_s *dest,
                                    FAR const void *src,
                                    FAR const struct nxgl_point_s *origin,
                                    unsigned int srcstride);
EXTERN void nxgl_copyrectangle_4bpp(FAR struct fb_planeinfo_s *pinfo,
                                    FAR const struct nxgl_rect_s *dest,
                                    FAR const void *src,
                                    FAR const struct nxgl_point_s *origin,
                                    unsigned int srcstride);
EXTERN void nxgl_copyrectangle_8bpp(FAR struct fb_planeinfo_s *pinfo,
                                    FAR const struct nxgl_rect_s *dest,
                                    FAR const void *src,
                                    FAR const struct nxgl_point_s *origin,
                                    unsigned int srcstride);
EXTERN void nxgl_copyrectangle_16bpp(FAR struct fb_planeinfo_s *pinfo,
                                     FAR const struct nxgl_rect_s *dest,
                                     FAR const void *src,
                                     FAR const struct nxgl_point_s *origin,
                                     unsigned int srcstride);
EXTERN void nxgl_copyrectangle_24bpp(FAR struct fb_planeinfo_s *pinfo,
                                     FAR const struct nxgl_rect_s *dest,
                                     FAR const void *src,
                                     FAR const struct nxgl_point_s *origin,
                                     unsigned int srcstride);
EXTERN void nxgl_copyrectangle_32bpp(FAR struct fb_planeinfo_s *pinfo,
                                     FAR const struct nxgl_rect_s *dest,
                                     FAR const void *src,
                                     FAR const struct nxgl_point_s *origin,
                                     unsigned int srcstride);

/****************************************************************************
 * Name: nxgl_rectcopy
 *
 * Description:
 *   This is essentially memcpy for rectangles.  We don't do structure
 *   assignements because some compilers are not good at that.
 *
 ****************************************************************************/

EXTERN void nxgl_rectcopy(FAR struct nxgl_rect_s *dest,
                          FAR const struct nxgl_rect_s *src);

/****************************************************************************
 * Name: nxgl_rectoffset
 *
 * Description:
 *   Offset the rectangle position by the specified dx, dy values.
 *
 ****************************************************************************/

EXTERN void nxgl_rectoffset(FAR struct nxgl_rect_s *dest,
                            FAR const struct nxgl_rect_s *src,
                            nxgl_coord_t dx, nxgl_coord_t dy);

/****************************************************************************
 * Name: nxgl_vectoradd
 *
 * Description:
 *   Add two 2x1 vectors and save the result to a third.
 *
 ****************************************************************************/

EXTERN void nxgl_vectoradd(FAR struct nxgl_point_s *dest,
                           FAR const struct nxgl_point_s *v1,
                           FAR const struct nxgl_point_s *v2);

/****************************************************************************
 * Name: nxgl_rectintersect
 *
 * Description:
 *   Return the rectangle representing the intersection of the two rectangles.
 *
 ****************************************************************************/

EXTERN void nxgl_rectintersect(FAR struct nxgl_rect_s *dest,
                               FAR const struct nxgl_rect_s *src1,
                               FAR const struct nxgl_rect_s *src2);

/****************************************************************************
 * Name: nxgl_nonintersecting
 *
 * Description:
 *   Return the regions of rectangle rect 1 that do not intersect with
 *   rect2.  This may be up to founr rectangles some of which may be
 *   degenerate (and can be picked off with nxgl_nullrect)
 *
 ****************************************************************************/

EXTERN void nxgl_nonintersecting(FAR struct nxgl_rect_s result[4],
                                 FAR const struct nxgl_rect_s *rect1,
                                 FAR const struct nxgl_rect_s *rect2);

/****************************************************************************
 * Name: nxgl_rectoverlap
 *
 * Description:
 *   Return TRUE if the two rectangles overlap
 *
 ****************************************************************************/

EXTERN boolean nxgl_rectoverlap(FAR struct nxgl_rect_s *rect1,
                                FAR struct nxgl_rect_s *rect2);

/****************************************************************************
 * Name: nxgl_nullrect
 *
 * Description:
 *   Return TRUE if the area of the retangle is <= 0.
 *
 ****************************************************************************/

EXTERN boolean nxgl_nullrect(FAR const struct nxgl_rect_s *rect);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_NXGLIB_H */
