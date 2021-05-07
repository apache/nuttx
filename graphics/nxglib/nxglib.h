/****************************************************************************
 * graphics/nxglib/nxglib.h
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

#ifndef __GRAPHICS_NXGLIB_NXBLIC_H
#define __GRAPHICS_NXGLIB_NXBLIC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/nx/nxglib.h>

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

/* Rasterizers **************************************************************/

/****************************************************************************
 * Name: nxgl_setpixel_*bpp / pwfb_setpixel_*bpp
 *
 * Description:
 *   Draw a single pixel in graphics memory at the given position and
 *   with the given color.  This is equivalent to nxgl_fillrectangle_*bpp()
 *   with a 1x1 rectangle but is more efficient.
 *
 ****************************************************************************/

struct nxbe_window_s; /* Forward reference.  See include/nuttx/nx/nxbe.h */

/* For direct access to graphics device memory */

void nxgl_setpixel_1bpp(FAR NX_PLANEINFOTYPE *pinfo,
                        FAR const struct nxgl_point_s *pos, uint8_t color);
void nxgl_setpixel_2bpp(FAR NX_PLANEINFOTYPE *pinfo,
                        FAR const struct nxgl_point_s *pos, uint8_t color);
void nxgl_setpixel_4bpp(FAR NX_PLANEINFOTYPE *pinfo,
                        FAR const struct nxgl_point_s *pos, uint8_t color);
void nxgl_setpixel_8bpp(FAR NX_PLANEINFOTYPE *pinfo,
                        FAR const struct nxgl_point_s *pos, uint8_t color);
void nxgl_setpixel_16bpp(FAR NX_PLANEINFOTYPE *pinfo,
                         FAR const struct nxgl_point_s *pos, uint16_t color);
void nxgl_setpixel_24bpp(FAR NX_PLANEINFOTYPE *pinfo,
                         FAR const struct nxgl_point_s *pos, uint32_t color);
void nxgl_setpixel_32bpp(FAR NX_PLANEINFOTYPE *pinfo,
                         FAR const struct nxgl_point_s *pos, uint32_t color);

#ifdef CONFIG_NX_RAMBACKED
/* For access to per-window framebuffer memory */

void pwfb_setpixel_1bpp(FAR struct nxbe_window_s *bwnd,
                        FAR const struct nxgl_point_s *pos, uint8_t color);
void pwfb_setpixel_2bpp(FAR struct nxbe_window_s *bwnd,
                        FAR const struct nxgl_point_s *pos, uint8_t color);
void pwfb_setpixel_4bpp(FAR struct nxbe_window_s *bwnd,
                        FAR const struct nxgl_point_s *pos, uint8_t color);
void pwfb_setpixel_8bpp(FAR struct nxbe_window_s *bwnd,
                        FAR const struct nxgl_point_s *pos, uint8_t color);
void pwfb_setpixel_16bpp(FAR struct nxbe_window_s *bwnd,
                         FAR const struct nxgl_point_s *pos, uint16_t color);
void pwfb_setpixel_24bpp(FAR struct nxbe_window_s *bwnd,
                         FAR const struct nxgl_point_s *pos, uint32_t color);
void pwfb_setpixel_32bpp(FAR struct nxbe_window_s *bwnd,
                         FAR const struct nxgl_point_s *pos, uint32_t color);
#endif

/****************************************************************************
 * Name: nxgl_fillrectangle_*bpp / pwfb_fillrectangle_*bpp
 *
 * Description:
 *   Fill a rectangle region in the graphics memory with a fixed color
 *
 ****************************************************************************/

/* For direct access to graphics device memory */

void nxgl_fillrectangle_1bpp(FAR NX_PLANEINFOTYPE *pinfo,
                             FAR const struct nxgl_rect_s *rect,
                             uint8_t color);
void nxgl_fillrectangle_2bpp(FAR NX_PLANEINFOTYPE *pinfo,
                             FAR const struct nxgl_rect_s *rect,
                             uint8_t color);
void nxgl_fillrectangle_4bpp(FAR NX_PLANEINFOTYPE *pinfo,
                             FAR const struct nxgl_rect_s *rect,
                             uint8_t color);
void nxgl_fillrectangle_8bpp(FAR NX_PLANEINFOTYPE *pinfo,
                             FAR const struct nxgl_rect_s *rect,
                             uint8_t color);
void nxgl_fillrectangle_16bpp(FAR NX_PLANEINFOTYPE *pinfo,
                              FAR const struct nxgl_rect_s *rect,
                              uint16_t color);
void nxgl_fillrectangle_24bpp(FAR NX_PLANEINFOTYPE *pinfo,
                              FAR const struct nxgl_rect_s *rect,
                              uint32_t color);
void nxgl_fillrectangle_32bpp(FAR NX_PLANEINFOTYPE *pinfo,
                              FAR const struct nxgl_rect_s *rect,
                              uint32_t color);

#ifdef CONFIG_NX_RAMBACKED
/* For access to per-window framebuffer memory */

void pwfb_fillrectangle_1bpp(FAR struct nxbe_window_s *bwnd,
                             FAR const struct nxgl_rect_s *rect,
                             uint8_t color);
void pwfb_fillrectangle_2bpp(FAR struct nxbe_window_s *bwnd,
                             FAR const struct nxgl_rect_s *rect,
                             uint8_t color);
void pwfb_fillrectangle_4bpp(FAR struct nxbe_window_s *bwnd,
                             FAR const struct nxgl_rect_s *rect,
                             uint8_t color);
void pwfb_fillrectangle_8bpp(FAR struct nxbe_window_s *bwnd,
                             FAR const struct nxgl_rect_s *rect,
                             uint8_t color);
void pwfb_fillrectangle_16bpp(FAR struct nxbe_window_s *bwnd,
                              FAR const struct nxgl_rect_s *rect,
                              uint16_t color);
void pwfb_fillrectangle_24bpp(FAR struct nxbe_window_s *bwnd,
                              FAR const struct nxgl_rect_s *rect,
                              uint32_t color);
void pwfb_fillrectangle_32bpp(FAR struct nxbe_window_s *bwnd,
                              FAR const struct nxgl_rect_s *rect,
                              uint32_t color);
#endif

/****************************************************************************
 * Name: nxgl_getrectangle_*bpp / pwfb_getrectangle_*bpp
 *
 * Description:
 *   Fetch a rectangular region from graphics memory.  The source is
 *   expressed as a rectangle.
 *
 ****************************************************************************/

/* For direct access to graphics device memory */

void nxgl_getrectangle_1bpp(FAR NX_PLANEINFOTYPE *pinfo,
                            FAR const struct nxgl_rect_s *rect,
                            FAR void *dest, unsigned int deststride);
void nxgl_getrectangle_2bpp(FAR NX_PLANEINFOTYPE *pinfo,
                            FAR const struct nxgl_rect_s *rect,
                            FAR void *dest, unsigned int deststride);
void nxgl_getrectangle_4bpp(FAR NX_PLANEINFOTYPE *pinfo,
                            FAR const struct nxgl_rect_s *rect,
                            FAR void *dest, unsigned int deststride);
void nxgl_getrectangle_8bpp(FAR NX_PLANEINFOTYPE *pinfo,
                            FAR const struct nxgl_rect_s *rect,
                            FAR void *dest, unsigned int deststride);
void nxgl_getrectangle_16bpp(FAR NX_PLANEINFOTYPE *pinfo,
                             FAR const struct nxgl_rect_s *rect,
                             FAR void *dest, unsigned int deststride);
void nxgl_getrectangle_24bpp(FAR NX_PLANEINFOTYPE *pinfo,
                             FAR const struct nxgl_rect_s *rect,
                             FAR void *dest, unsigned int deststride);
void nxgl_getrectangle_32bpp(FAR NX_PLANEINFOTYPE *pinfo,
                             FAR const struct nxgl_rect_s *rect,
                             FAR void *dest, unsigned int deststride);

#ifdef CONFIG_NX_RAMBACKED
/* For access to per-window framebuffer memory */

void pwfb_getrectangle_1bpp(FAR struct nxbe_window_s *bwnd,
                            FAR const struct nxgl_rect_s *rect,
                            FAR void *dest, unsigned int deststride);
void pwfb_getrectangle_2bpp(FAR struct nxbe_window_s *bwnd,
                            FAR const struct nxgl_rect_s *rect,
                            FAR void *dest, unsigned int deststride);
void pwfb_getrectangle_4bpp(FAR struct nxbe_window_s *bwnd,
                            FAR const struct nxgl_rect_s *rect,
                            FAR void *dest, unsigned int deststride);
void pwfb_getrectangle_8bpp(FAR struct nxbe_window_s *bwnd,
                            FAR const struct nxgl_rect_s *rect,
                            FAR void *dest, unsigned int deststride);
void pwfb_getrectangle_16bpp(FAR struct nxbe_window_s *bwnd,
                             FAR const struct nxgl_rect_s *rect,
                             FAR void *dest, unsigned int deststride);
void pwfb_getrectangle_24bpp(FAR struct nxbe_window_s *bwnd,
                             FAR const struct nxgl_rect_s *rect,
                             FAR void *dest, unsigned int deststride);
void pwfb_getrectangle_32bpp(FAR struct nxbe_window_s *bwnd,
                             FAR const struct nxgl_rect_s *rect,
                             FAR void *dest, unsigned int deststride);
#endif

/****************************************************************************
 * Name: nxglib_filltrapezoid_*bpp / pwfb_filltrapezoid_*bpp
 *
 * Description:
 *   Fill a trapezoidal region in the graphics memory with a fixed color.
 *   Clip the trapezoid to lie within a bounding box.  This is useful for
 *   drawing complex shapes that can be broken into a set of trapezoids.
 *
 ****************************************************************************/

/* For direct access to graphics device memory */

void nxgl_filltrapezoid_1bpp(FAR NX_PLANEINFOTYPE *pinfo,
                             FAR const struct nxgl_trapezoid_s *trap,
                             FAR const struct nxgl_rect_s *bounds,
                             uint8_t color);
void nxgl_filltrapezoid_2bpp(FAR NX_PLANEINFOTYPE *pinfo,
                             FAR const struct nxgl_trapezoid_s *trap,
                             FAR const struct nxgl_rect_s *bounds,
                             uint8_t color);
void nxgl_filltrapezoid_4bpp(FAR NX_PLANEINFOTYPE *pinfo,
                             FAR const struct nxgl_trapezoid_s *trap,
                             FAR const struct nxgl_rect_s *bounds,
                             uint8_t color);
void nxgl_filltrapezoid_8bpp(FAR NX_PLANEINFOTYPE *pinfo,
                             FAR const struct nxgl_trapezoid_s *trap,
                             FAR const struct nxgl_rect_s *bounds,
                             uint8_t color);
void nxgl_filltrapezoid_16bpp(FAR NX_PLANEINFOTYPE *pinfo,
                              FAR const struct nxgl_trapezoid_s *trap,
                              FAR const struct nxgl_rect_s *bounds,
                              uint16_t color);
void nxgl_filltrapezoid_24bpp(FAR NX_PLANEINFOTYPE *pinfo,
                              FAR const struct nxgl_trapezoid_s *trap,
                              FAR const struct nxgl_rect_s *bounds,
                              uint32_t color);
void nxgl_filltrapezoid_32bpp(FAR NX_PLANEINFOTYPE *pinfo,
                              FAR const struct nxgl_trapezoid_s *trap,
                              FAR const struct nxgl_rect_s *bounds,
                              uint32_t color);

#ifdef CONFIG_NX_RAMBACKED
/* For access to per-window framebuffer memory */

void pwfb_filltrapezoid_1bpp(FAR struct nxbe_window_s *bwnd,
                             FAR const struct nxgl_trapezoid_s *trap,
                             FAR const struct nxgl_rect_s *bounds,
                             uint8_t color);
void pwfb_filltrapezoid_2bpp(FAR struct nxbe_window_s *bwnd,
                             FAR const struct nxgl_trapezoid_s *trap,
                             FAR const struct nxgl_rect_s *bounds,
                             uint8_t color);
void pwfb_filltrapezoid_4bpp(FAR struct nxbe_window_s *bwnd,
                             FAR const struct nxgl_trapezoid_s *trap,
                             FAR const struct nxgl_rect_s *bounds,
                             uint8_t color);
void pwfb_filltrapezoid_8bpp(FAR struct nxbe_window_s *bwnd,
                             FAR const struct nxgl_trapezoid_s *trap,
                             FAR const struct nxgl_rect_s *bounds,
                             uint8_t color);
void pwfb_filltrapezoid_16bpp(FAR struct nxbe_window_s *bwnd,
                              FAR const struct nxgl_trapezoid_s *trap,
                              FAR const struct nxgl_rect_s *bounds,
                              uint16_t color);
void pwfb_filltrapezoid_24bpp(FAR struct nxbe_window_s *bwnd,
                              FAR const struct nxgl_trapezoid_s *trap,
                              FAR const struct nxgl_rect_s *bounds,
                              uint32_t color);
void pwfb_filltrapezoid_32bpp(FAR struct nxbe_window_s *bwnd,
                              FAR const struct nxgl_trapezoid_s *trap,
                              FAR const struct nxgl_rect_s *bounds,
                              uint32_t color);
#endif

/****************************************************************************
 * Name: nxgl_moverectangle_*bpp / pwfb_moverectangle_*bpp
 *
 * Description:
 *   Move a rectangular region from location to another in the
 *   framebuffer/LCD memory.  The source is expressed as a rectangle; the
 *   destination position is expressed as a point corresponding to the
 *   translation of the upper, left-hand corner.
 *
 ****************************************************************************/

/* For direct access to graphics device memory */

void nxgl_moverectangle_1bpp(FAR NX_PLANEINFOTYPE *pinfo,
                             FAR const struct nxgl_rect_s *rect,
                             FAR struct nxgl_point_s *offset);
void nxgl_moverectangle_2bpp(FAR NX_PLANEINFOTYPE *pinfo,
                             FAR const struct nxgl_rect_s *rect,
                             FAR struct nxgl_point_s *offset);
void nxgl_moverectangle_4bpp(FAR NX_PLANEINFOTYPE *pinfo,
                             FAR const struct nxgl_rect_s *rect,
                             FAR struct nxgl_point_s *offset);
void nxgl_moverectangle_8bpp(FAR NX_PLANEINFOTYPE *pinfo,
                             FAR const struct nxgl_rect_s *rect,
                             FAR struct nxgl_point_s *offset);
void nxgl_moverectangle_16bpp(FAR NX_PLANEINFOTYPE *pinfo,
                              FAR const struct nxgl_rect_s *rect,
                              FAR struct nxgl_point_s *offset);
void nxgl_moverectangle_24bpp(FAR NX_PLANEINFOTYPE *pinfo,
                              FAR const struct nxgl_rect_s *rect,
                              FAR struct nxgl_point_s *offset);
void nxgl_moverectangle_32bpp(FAR NX_PLANEINFOTYPE *pinfo,
                              FAR const struct nxgl_rect_s *rect,
                              FAR struct nxgl_point_s *offset);

#ifdef CONFIG_NX_RAMBACKED
/* For access to per-window framebuffer memory */

void pwfb_moverectangle_1bpp(FAR struct nxbe_window_s *bwnd,
                             FAR const struct nxgl_rect_s *rect,
                             FAR struct nxgl_point_s *offset);
void pwfb_moverectangle_2bpp(FAR struct nxbe_window_s *bwnd,
                             FAR const struct nxgl_rect_s *rect,
                             FAR struct nxgl_point_s *offset);
void pwfb_moverectangle_4bpp(FAR struct nxbe_window_s *bwnd,
                             FAR const struct nxgl_rect_s *rect,
                             FAR struct nxgl_point_s *offset);
void pwfb_moverectangle_8bpp(FAR struct nxbe_window_s *bwnd,
                             FAR const struct nxgl_rect_s *rect,
                             FAR struct nxgl_point_s *offset);
void pwfb_moverectangle_16bpp(FAR struct nxbe_window_s *bwnd,
                              FAR const struct nxgl_rect_s *rect,
                              FAR struct nxgl_point_s *offset);
void pwfb_moverectangle_24bpp(FAR struct nxbe_window_s *bwnd,
                              FAR const struct nxgl_rect_s *rect,
                              FAR struct nxgl_point_s *offset);
void pwfb_moverectangle_32bpp(FAR struct nxbe_window_s *bwnd,
                              FAR const struct nxgl_rect_s *rect,
                              FAR struct nxgl_point_s *offset);
#endif

/****************************************************************************
 * Name: nxgl_copyrectangle_*bpp / pwfb_copyrectangle_*bpp
 *
 * Description:
 *   Copy a rectangular bitmap image into the specific position in the
 *   graphics memory.
 *
 ****************************************************************************/

/* For direct access to graphics device memory */

void nxgl_copyrectangle_1bpp(FAR NX_PLANEINFOTYPE *pinfo,
                             FAR const struct nxgl_rect_s *dest,
                             FAR const void *src,
                             FAR const struct nxgl_point_s *origin,
                             unsigned int srcstride);
void nxgl_copyrectangle_2bpp(FAR NX_PLANEINFOTYPE *pinfo,
                             FAR const struct nxgl_rect_s *dest,
                             FAR const void *src,
                             FAR const struct nxgl_point_s *origin,
                             unsigned int srcstride);
void nxgl_copyrectangle_4bpp(FAR NX_PLANEINFOTYPE *pinfo,
                             FAR const struct nxgl_rect_s *dest,
                             FAR const void *src,
                             FAR const struct nxgl_point_s *origin,
                             unsigned int srcstride);
void nxgl_copyrectangle_8bpp(FAR NX_PLANEINFOTYPE *pinfo,
                             FAR const struct nxgl_rect_s *dest,
                             FAR const void *src,
                             FAR const struct nxgl_point_s *origin,
                             unsigned int srcstride);
void nxgl_copyrectangle_16bpp(FAR NX_PLANEINFOTYPE *pinfo,
                              FAR const struct nxgl_rect_s *dest,
                              FAR const void *src,
                              FAR const struct nxgl_point_s *origin,
                              unsigned int srcstride);
void nxgl_copyrectangle_24bpp(FAR NX_PLANEINFOTYPE *pinfo,
                              FAR const struct nxgl_rect_s *dest,
                              FAR const void *src,
                              FAR const struct nxgl_point_s *origin,
                              unsigned int srcstride);
void nxgl_copyrectangle_32bpp(FAR NX_PLANEINFOTYPE *pinfo,
                              FAR const struct nxgl_rect_s *dest,
                              FAR const void *src,
                              FAR const struct nxgl_point_s *origin,
                              unsigned int srcstride);

#ifdef CONFIG_NX_RAMBACKED
/* For access to per-window framebuffer memory */

void pwfb_copyrectangle_1bpp(FAR struct nxbe_window_s *bwnd,
                             FAR const struct nxgl_rect_s *dest,
                             FAR const void *src,
                             FAR const struct nxgl_point_s *origin,
                             unsigned int srcstride);
void pwfb_copyrectangle_2bpp(FAR struct nxbe_window_s *bwnd,
                             FAR const struct nxgl_rect_s *dest,
                             FAR const void *src,
                             FAR const struct nxgl_point_s *origin,
                             unsigned int srcstride);
void pwfb_copyrectangle_4bpp(FAR struct nxbe_window_s *bwnd,
                             FAR const struct nxgl_rect_s *dest,
                             FAR const void *src,
                             FAR const struct nxgl_point_s *origin,
                             unsigned int srcstride);
void pwfb_copyrectangle_8bpp(FAR struct nxbe_window_s *bwnd,
                             FAR const struct nxgl_rect_s *dest,
                             FAR const void *src,
                             FAR const struct nxgl_point_s *origin,
                             unsigned int srcstride);
void pwfb_copyrectangle_16bpp(FAR struct nxbe_window_s *bwnd,
                              FAR const struct nxgl_rect_s *dest,
                              FAR const void *src,
                              FAR const struct nxgl_point_s *origin,
                              unsigned int srcstride);
void pwfb_copyrectangle_24bpp(FAR struct nxbe_window_s *bwnd,
                              FAR const struct nxgl_rect_s *dest,
                              FAR const void *src,
                              FAR const struct nxgl_point_s *origin,
                              unsigned int srcstride);
void pwfb_copyrectangle_32bpp(FAR struct nxbe_window_s *bwnd,
                              FAR const struct nxgl_rect_s *dest,
                              FAR const void *src,
                              FAR const struct nxgl_point_s *origin,
                              unsigned int srcstride);
#endif

/****************************************************************************
 * Name: nxgl_cursor_draw_*bpp
 *
 * Description:
 *  Draw the cursor image into the specified position in the graphics memory.
 *
 ****************************************************************************/

struct nxbe_state_s;    /* Forward reference */

#ifdef CONFIG_NX_SWCURSOR
void nxglib_cursor_draw_8bpp(FAR struct nxbe_state_s *be,
                             FAR const struct nxgl_rect_s *bounds,
                             int planeno);
void nxglib_cursor_draw_16bpp(FAR struct nxbe_state_s *be,
                              FAR const struct nxgl_rect_s *bounds,
                              int planeno);
void nxglib_cursor_draw_24bpp(FAR struct nxbe_state_s *be,
                              FAR const struct nxgl_rect_s *bounds,
                              int planeno);
void nxglib_cursor_draw_32bpp(FAR struct nxbe_state_s *be,
                              FAR const struct nxgl_rect_s *bounds,
                              int planeno);
#endif

/****************************************************************************
 * Name: nxgl_cursor_erase_*bpp
 *
 * Description:
 *   Erase the cursor by copying the saved background image into the graphics
 *   memory.
 *
 ****************************************************************************/

#ifdef CONFIG_NX_SWCURSOR
void nxglib_cursor_erase_8bpp(FAR struct nxbe_state_s *be,
                              FAR const struct nxgl_rect_s *bounds,
                              int planeno);
void nxglib_cursor_erase_16bpp(FAR struct nxbe_state_s *be,
                               FAR const struct nxgl_rect_s *bounds,
                               int planeno);
void nxglib_cursor_erase_24bpp(FAR struct nxbe_state_s *be,
                               FAR const struct nxgl_rect_s *bounds,
                               int planeno);
void nxglib_cursor_erase_32bpp(FAR struct nxbe_state_s *be,
                               FAR const struct nxgl_rect_s *bounds,
                               int planeno);
#endif

/****************************************************************************
 * Name: nxgl_cursor_backup_*bpp
 *
 * Description:
 *  Save the background image for subsequent use to erase the cursor from the
 *  device graphics memory.
 *
 ****************************************************************************/

#ifdef CONFIG_NX_SWCURSOR
void nxglib_cursor_backup_8bpp(FAR struct nxbe_state_s *be,
                               FAR const struct nxgl_rect_s *bounds,
                               int planeno);
void nxglib_cursor_backup_16bpp(FAR struct nxbe_state_s *be,
                                FAR const struct nxgl_rect_s *bounds,
                                int planeno);
void nxglib_cursor_backup_24bpp(FAR struct nxbe_state_s *be,
                                FAR const struct nxgl_rect_s *bounds,
                                int planeno);
void nxglib_cursor_backup_32bpp(FAR struct nxbe_state_s *be,
                                FAR const struct nxgl_rect_s *bounds,
                                int planeno);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __GRAPHICS_NXGLIB_NXBLIC_H */
