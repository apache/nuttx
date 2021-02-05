/****************************************************************************
 * graphics/nxbe/nxbe.h
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

#ifndef __GRAPHICS_NXBE_NXBE_H
#define __GRAPHICS_NXBE_NXBE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxglib.h>
#include <nuttx/nx/nxcursor.h>
#include <nuttx/nx/nxbe.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* These are the values for the clipping order provided to nx_clipper */

#define NX_CLIPORDER_TLRB    (0)   /* Top-left-right-bottom */
#define NX_CLIPORDER_TRLB    (1)   /* Top-right-left-bottom */
#define NX_CLIPORDER_BLRT    (2)   /* Bottom-left-right-top */
#define NX_CLIPORDER_BRLT    (3)   /* Bottom-right-left-top */
#define NX_CLIPORDER_DEFAULT NX_CLIPORDER_TLRB

/* Server flags and helper macros:
 *
 * NXBE_STATE_MODAL  - One window is in a focused, modal state
 */

#define NXBE_STATE_MODAL     (1 << 0) /* Bit 0: One window is in a focused,
                                       *        modal state */

/* Helpful flag macros */

#define NXBE_STATE_ISMODAL(nxbe) \
  (((nxbe)->flags & NXBE_STATE_MODAL) != 0)
#define NXBE_STATE_SETMODAL(nxbe) \
  do { (nxbe)->flags |= NXBE_STATE_MODAL; } while (0)
#define NXBE_STATE_CLRMODAL(nxbe) \
  do { (nxbe)->flags &= ~NXBE_STATE_MODAL; } while (0)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Rasterization ************************************************************/

/* A vtable of raster operation function pointers.  The types of the
 * function points must match the device rasterizer types exported by nxglib.
 */

struct nxbe_dev_vtable_s
{
  CODE void (*setpixel)(FAR NX_PLANEINFOTYPE *pinfo,
                        FAR const struct nxgl_point_s *pos,
                        nxgl_mxpixel_t color);
  CODE void (*fillrectangle)(FAR NX_PLANEINFOTYPE *pinfo,
                             FAR const struct nxgl_rect_s *rect,
                             nxgl_mxpixel_t color);
  CODE void (*getrectangle)(FAR NX_PLANEINFOTYPE *pinfo,
                            FAR const struct nxgl_rect_s *rect,
                           FAR void *dest, unsigned int deststride);
  CODE void (*filltrapezoid)(FAR NX_PLANEINFOTYPE *pinfo,
                             FAR const struct nxgl_trapezoid_s *trap,
                             FAR const struct nxgl_rect_s *bounds,
                             nxgl_mxpixel_t color);
  CODE void (*moverectangle)(FAR NX_PLANEINFOTYPE *pinfo,
                             FAR const struct nxgl_rect_s *rect,
                             FAR struct nxgl_point_s *offset);
  CODE void (*copyrectangle)(FAR NX_PLANEINFOTYPE *pinfo,
                             FAR const struct nxgl_rect_s *dest,
                             FAR const void *src,
                             FAR const struct nxgl_point_s *origin,
                             unsigned int srcstride);
};

#ifdef CONFIG_NX_RAMBACKED
/* A vtable of raster operation function pointers.  The types of the
 * function points must match the per-window framebuffer rasterizer types
 * exported by nxglib.
 */

struct nxbe_pwfb_vtable_s
{
  CODE void (*setpixel)(FAR struct nxbe_window_s *bwnd,
                        FAR const struct nxgl_point_s *pos,
                        nxgl_mxpixel_t color);
  CODE void (*fillrectangle)(FAR struct nxbe_window_s *bwnd,
                             FAR const struct nxgl_rect_s *rect,
                             nxgl_mxpixel_t color);
  CODE void (*getrectangle)(FAR struct nxbe_window_s *bwnd,
                            FAR const struct nxgl_rect_s *rect,
                           FAR void *dest, unsigned int deststride);
  CODE void (*filltrapezoid)(FAR struct nxbe_window_s *bwnd,
                             FAR const struct nxgl_trapezoid_s *trap,
                             FAR const struct nxgl_rect_s *bounds,
                             nxgl_mxpixel_t color);
  CODE void (*moverectangle)(FAR struct nxbe_window_s *bwnd,
                             FAR const struct nxgl_rect_s *rect,
                             FAR struct nxgl_point_s *offset);
  CODE void (*copyrectangle)(FAR struct nxbe_window_s *bwnd,
                             FAR const struct nxgl_rect_s *dest,
                             FAR const void *src,
                             FAR const struct nxgl_point_s *origin,
                             unsigned int srcstride);
};
#endif

#ifdef CONFIG_NX_SWCURSOR
/* A vtable of raster operation function pointers.  The types of the
 * function points must match the cursor rasterizer types exported by
 * nxglib.
 */

struct nxbe_cursorops_s
{
  CODE void (*draw)(FAR struct nxbe_state_s *be,
                    FAR const struct nxgl_rect_s *bounds,
                    int planeno);
  CODE void (*erase)(FAR struct nxbe_state_s *be,
                     FAR const struct nxgl_rect_s *bounds,
                     int planeno);
  CODE void (*backup)(FAR struct nxbe_state_s *be,
                      FAR const struct nxgl_rect_s *bounds,
                      int planeno);
};
#endif

/* Encapsulates everything needed support window rasterization commands. */

struct nxbe_plane_s
{
  /* Raster device operations */

  struct nxbe_dev_vtable_s dev;

#ifdef CONFIG_NX_RAMBACKED
  /* Raster per-window framebuffer operations */

  struct nxbe_pwfb_vtable_s pwfb;
#endif

#ifdef CONFIG_NX_SWCURSOR
  /* Cursor device operations */

  struct nxbe_cursorops_s cursor;
#endif

  /* Framebuffer plane info describing destination video plane */

  NX_DRIVERTYPE *driver;
  NX_PLANEINFOTYPE pinfo;
};

/* Clipping *****************************************************************/

/* Clipping callback functions called nxbe_clipper for each visible and
 * obscured region of a rectangle within a window.
 */

struct nxbe_clipops_s
{
  CODE void (*visible)(FAR struct nxbe_clipops_s *cops,
                       FAR struct nxbe_plane_s *plane,
                       FAR const struct nxgl_rect_s *rect);

  CODE void (*obscured)(FAR struct nxbe_clipops_s *cops,
                        FAR struct nxbe_plane_s *plane,
                        FAR const struct nxgl_rect_s *rect);
};

/* Cursor *******************************************************************/

/* Cursor state structure */

#if defined(CONFIG_NX_SWCURSOR)
struct nxbe_cursor_s
{
  bool visible;                             /* True: the cursor is visible */
  struct nxgl_rect_s bounds;                /* Cursor image bounding box */
  nxgl_mxpixel_t color1[CONFIG_NX_NPLANES]; /* Color1 is main color of the cursor */
  nxgl_mxpixel_t color2[CONFIG_NX_NPLANES]; /* Color2 is color of any border */
  nxgl_mxpixel_t color3[CONFIG_NX_NPLANES]; /* Color3 is the blended color */
  size_t allocsize;                         /* Size of the background allocation */
  FAR const uint8_t *image;                 /* Cursor image at 2-bits/pixel */
  FAR nxgl_mxpixel_t *bkgd;                 /* Cursor background in device pixels */
};
#elif defined(CONFIG_NX_HWCURSOR)
struct nxbe_cursor_s
{
  bool visible;                    /* True: the cursor is visible */
  struct nxgl_point_s pos;         /* The current cursor position */
};
#endif

/* Back-end state ***********************************************************/

/* This structure describes the overall back-end window state */

struct nxbe_state_s
{
  uint8_t flags;                     /* NXBE_STATE_* flags */

#if defined(CONFIG_NX_SWCURSOR) || defined(CONFIG_NX_HWCURSOR)
  /* Cursor support */

  struct nxbe_cursor_s cursor;       /* Cursor support */
#endif

  /* The window list (with the background window always at the bottom) */

  FAR struct nxbe_window_s *topwnd;  /* The window at the top of the display */
  struct nxbe_window_s bkgd;         /* The background window is always at the bottom */

  /* At present, only a solid colored background is supported for refills.
   * The following provides the background color.  It would be nice to
   * support background bitmap images as well.
   */

  nxgl_mxpixel_t bgcolor[CONFIG_NX_NPLANES];

  /* vinfo describes the video controller and plane[n].pinfo describes color
   * plane 'n' supported by the video controller.  Most common color models
   * fit in one plane, but this array provides future support for hardware
   * with planar YUV types with 3 or 4 color planes.
   */

  struct fb_videoinfo_s vinfo;

  /* Rasterizing functions selected to match the BPP reported in pinfo[] */

  struct nxbe_plane_s plane[CONFIG_NX_NPLANES];
};

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
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nxbe_colormap
 *
 * Description:
 *   Set the hardware color map to the palette expected by NX
 *
 ****************************************************************************/

#ifdef CONFIG_FB_CMAP
int nxbe_colormap(FAR NX_DRIVERTYPE *dev);
#endif

/****************************************************************************
 * Name: nxbe_notify_rectangle
 *
 * Description:
 *   When CONFIG_NX_UPDATE=y, then the graphics system will callout to
 *   inform some external module that the display has been updated.  This
 *   would be useful in a couple for cases.
 *
 *   - When a serial LCD is used, but a framebuffer is used to access the
 *     LCD.  In this case, the update callout can be used to refresh the
 *     affected region of the display.
 *
 *   - When VNC is enabled.  This is case, this callout is necessary to
 *     update the remote frame buffer to match the local framebuffer.
 *
 *   When this feature is enabled, some external logic must provide this
 *   interface.  This is the function that will handle the notification.  It
 *   receives the rectangular region that was updated on the provided plane.
 *
 ****************************************************************************/

#ifdef CONFIG_NX_UPDATE
void nxbe_notify_rectangle(FAR NX_DRIVERTYPE *dev,
                           FAR const struct nxgl_rect_s *rect);
#endif

/****************************************************************************
 * Name: nx_configure
 *
 * Description:
 *   Configure the back end state structure based on information from the
 *   framebuffer or LCD driver
 *
 ****************************************************************************/

int nxbe_configure(FAR NX_DRIVERTYPE *dev, FAR struct nxbe_state_s *be);

#if defined(CONFIG_NX_SWCURSOR) || defined(CONFIG_NX_HWCURSOR)
/****************************************************************************
 * Name: nxbe_cursor_enable
 *
 * Description:
 *   Enable/disable presentation of the cursor
 *
 * Input Parameters:
 *   be  - The back-end state structure instance
 *   enable - True: show the cursor, false: hide the cursor.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxbe_cursor_enable(FAR struct nxbe_state_s *be, bool enable);

/****************************************************************************
 * Name: nxbe_cursor_setimage
 *
 * Description:
 *   Set the cursor image
 *
 *   The image is provided a a 2-bits-per-pixel image.  The two bit incoding
 *   is as following:
 *
 *   00 - The transparent background
 *   01 - Color1:  The main color of the cursor
 *   10 - Color2:  The color of any border
 *   11 - Color3:  A blend color for better imaging (fake anti-aliasing).
 *
 * Input Parameters:
 *   be  - The back-end state structure instance
 *   image - Describes the cursor image in the expected format.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_NX_HWCURSORIMAGE) || defined(CONFIG_NX_SWCURSOR)
void nxbe_cursor_setimage(FAR struct nxbe_state_s *be,
                          FAR struct nx_cursorimage_s *image);
#endif

/****************************************************************************
 * Name: nxbe_cursor_setposition
 *
 * Description:
 *   Move the cursor to the specified position
 *
 * Input Parameters:
 *   be  - The back-end state structure instance
 *   pos - The new cursor position
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxbe_cursor_setposition(FAR struct nxbe_state_s *be,
                             FAR const struct nxgl_point_s *pos);

#endif /* CONFIG_NX_SWCURSOR || CONFIG_NX_HWCURSOR */

#ifdef CONFIG_NX_SWCURSOR
/****************************************************************************
 * Name: nxbe_cursor_backupdraw and nxbe_cursor_backupdraw_dev
 *
 * Description:
 *   Called after any modification to the display (in window coordinate
 *   frame) to perform the backup-draw operation on one color plane.
 *
 * Input Parameters:
 *   be    - The back-end state structure instance, or
 *   wnd   - Window state structure
 *   rect  - The modified region of the window.  In windows coordinates for
 *           nxbe_cursor_backupdraw(); in graphics device coordinates for
 *           nxbe_cursor_backupdraw_dev().
 *   plane - The plane number to use.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxbe_cursor_backupdraw(FAR struct nxbe_window_s *wnd,
                            FAR const struct nxgl_rect_s *rect, int plane);
void nxbe_cursor_backupdraw_dev(FAR struct nxbe_state_s *be,
                                FAR const struct nxgl_rect_s *rect,
                                int plane);

/****************************************************************************
 * Name: nxbe_cursor_backupdraw_all and nxbe_cursor_backupdraw_devall
 *
 * Description:
 *   Called after any modification to the display to perform the backup-draw
 *   operation on all color planes.
 *
 * Input Parameters:
 *   be    - The back-end state structure instance, or
 *   wnd   - Window state structure
 *   rect  - The modified region of the window.  In windows coordinates for
 *           nxbe_cursor_backupdraw(); in graphics device coordinates for
 *           nxbe_cursor_backupdraw_dev().
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxbe_cursor_backupdraw_all(FAR struct nxbe_window_s *wnd,
                                FAR const struct nxgl_rect_s *rect);
void nxbe_cursor_backupdraw_devall(FAR struct nxbe_state_s *be,
                                   FAR const struct nxgl_rect_s *rect);
#endif /*  */

/****************************************************************************
 * Name: nxbe_closewindow
 *
 * Description:
 *   Close an existing window
 *
 * Input Parameters:
 *   wnd  - The window to be closed (and deallocated)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxbe_closewindow(struct nxbe_window_s *wnd);

/****************************************************************************
 * Name: nxbe_setposition
 *
 * Description:
 *   This function checks for intersections and redraws the display after
 *   a change in the position of a window.
 *
 ****************************************************************************/

void nxbe_setposition(FAR struct nxbe_window_s *wnd,
                      FAR const struct nxgl_point_s *pos);

/****************************************************************************
 * Name: nxbe_setsize
 *
 * Description:
 *   This function checks for intersections and redraws the display after
 *   a change in the size of a window.
 *
 ****************************************************************************/

void nxbe_setsize(FAR struct nxbe_window_s *wnd,
                  FAR const struct nxgl_size_s *size);

/****************************************************************************
 * Name: nxbe_raise
 *
 * Description:
 *   Bring the specified window to the top of the display.
 *
 ****************************************************************************/

void nxbe_raise(FAR struct nxbe_window_s *wnd);

/****************************************************************************
 * Name: nxbe_lower
 *
 * Description:
 *   Lower the specified window to the bottom of the display.
 *
 ****************************************************************************/

void nxbe_lower(FAR struct nxbe_window_s *wnd);

/****************************************************************************
 * Name: nxbe_modal
 *
 * Description:
 *   May be used to either (1) raise a window to the top of the display and
 *   select modal behavior, or (2) disable modal behavior.
 *
 ****************************************************************************/

void nxbe_modal(FAR struct nxbe_window_s *wnd, bool enable);

/****************************************************************************
 * Name: nxbe_setvisibility
 *
 * Description:
 *   Select if the window is visible or hidden.  A hidden window is still
 *   present will will update normally, but will be on the visible on the
 *   display until it is unhidden.
 *
 * Input Parameters:
 *   wnd  - The window to be modified
 *   hide - True: Window will be hidden; false: Window will be visible
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxbe_setvisibility(FAR struct nxbe_window_s *wnd, bool hide);

/****************************************************************************
 * Name: nxbe_setpixel
 *
 * Description:
 *  Set a single pixel in the window to the specified color. This is simply a
 *  degenerate case of nxbe_fill, but may be optimized in some architectures.
 *
 * Input Parameters:
 *   wnd  - The window structure reference
 *   pos  - The pixel location to be set
 *   col  - The color to use in the set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxbe_setpixel(FAR struct nxbe_window_s *wnd,
                   FAR const struct nxgl_point_s *pos,
                   nxgl_mxpixel_t color[CONFIG_NX_NPLANES]);

/****************************************************************************
 * Name: nxbe_fill
 *
 * Description:
 *  Fill the specified rectangle in the window with the specified color
 *
 * Input Parameters:
 *   wnd  - The window structure reference
 *   rect - The location to be filled
 *   col  - The color to use in the fill
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxbe_fill(FAR struct nxbe_window_s *wnd,
               FAR const struct nxgl_rect_s *rect,
               nxgl_mxpixel_t color[CONFIG_NX_NPLANES]);

/****************************************************************************
 * Name: nxbe_filltrapezoid
 *
 * Description:
 *  Fill the specified rectangle in the window with the specified color
 *
 * Input Parameters:
 *   wnd  - The window structure reference
 *   clip - Clipping region (may be null)
 *   rect - The location to be filled
 *   col  - The color to use in the fill
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxbe_filltrapezoid(FAR struct nxbe_window_s *wnd,
                        FAR const struct nxgl_rect_s *clip,
                        FAR const struct nxgl_trapezoid_s *trap,
                        nxgl_mxpixel_t color[CONFIG_NX_NPLANES]);

/****************************************************************************
 * Name: nxbe_getrectangle
 *
 * Description:
 *  Get the raw contents of graphic memory within a rectangular region. NOTE:
 *  Since raw graphic memory is returned, the returned memory content may be
 *  the memory of windows above this one and may not necessarily belong to
 *  this window unless you assure that this is the top window.
 *
 * Input Parameters:
 *   wnd  - The window structure reference
 *   rect - The location to be copied
 *   plane - Specifies the color plane to get from.
 *   dest - The location to copy the memory region
 *   deststride - The width, in bytes, of the dest memory
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxbe_getrectangle(FAR struct nxbe_window_s *wnd,
                       FAR const struct nxgl_rect_s *rect,
                       unsigned int plane,
                       FAR uint8_t *dest, unsigned int deststride);

/****************************************************************************
 * Name: nxbe_move
 *
 * Description:
 *   Move a rectangular region within the window
 *
 * Input Parameters:
 *   wnd    - The window within which the move is to be done
 *   rect   - Describes the rectangular region to move
 *   offset - The offset to move the region
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxbe_move(FAR struct nxbe_window_s *wnd,
               FAR const struct nxgl_rect_s *rect,
               FAR const struct nxgl_point_s *offset);

/****************************************************************************
 * Name: nxbe_bitmap_dev
 *
 * Description:
 *   Copy a rectangular region of a larger image into the rectangle in the
 *   specified window.  The graphics output is written to the graphics
 *   device unconditionally.
 *
 * Input Parameters:
 *   wnd    - The window that will receive the bitmap image
 *   dest   - Describes the rectangular region on the display that will
 *            receive the the bit map (window coordinate frame).
 *   src    - The start of the source image.
 *   origin - The origin of the upper, left-most corner of the full bitmap.
 *            Both dest and origin are in window coordinates, however, origin
 *            may lie outside of the display.
 *   stride - The width of the full source image in pixels.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxbe_bitmap_dev(FAR struct nxbe_window_s *wnd,
                     FAR const struct nxgl_rect_s *dest,
                     FAR const void *src[CONFIG_NX_NPLANES],
                     FAR const struct nxgl_point_s *origin,
                     unsigned int stride);

/****************************************************************************
 * Name: nxbe_bitmap
 *
 * Description:
 *   Copy a rectangular region of a larger image into the rectangle in the
 *   specified window.  This is a front end to nxbe_bitmap_dev() that is
 *   used only if CONFIG_NX_RAMBACKED=y.  If the per-window frame buffer is
 *   selected, then the bit map will be written to both the graphics device
 *   and shadowed in the per-window framebuffer.
 *
 * Input Parameters:
 *   wnd    - The window that will receive the bitmap image
 *   dest   - Describes the rectangular region on the display that will
 *            receive the the bit map (window coordinate frame).
 *   src    - The start of the source image.
 *   origin - The origin of the upper, left-most corner of the full bitmap.
 *            Both dest and origin are in window coordinates, however, origin
 *            may lie outside of the display.
 *   stride - The width of the full source image in bytes.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxbe_bitmap(FAR struct nxbe_window_s *wnd,
                 FAR const struct nxgl_rect_s *dest,
                 FAR const void *src[CONFIG_NX_NPLANES],
                 FAR const struct nxgl_point_s *origin,
                 unsigned int stride);

/****************************************************************************
 * Name: nxbe_flush
 *
 * Description:
 *   After per-window framebuffer has been updated, the modified region must
 *   be written to device graphics memory.  That function is managed by this
 *   simple function.  It does the following:
 *
 *   1) It calls nxbe_bitmap_dev() to copy the modified per-window
 *      framebuffer into device graphics memory.
 *   2) If CONFIG_NX_SWCURSOR is enabled, it calls the cursor "draw"
 *      renderer to update re-draw the currsor image if any portion of
 *      graphics display update overwrote the cursor.  Since these
 *      operations are performed back-to-back, any resulting flicker
 *      should be minimized.
 *
 * Input Parameters (same as for nxbe_bitmap_dev):
 *   wnd    - The window that will receive the bitmap image
 *   dest   - Describes the rectangular region in the window that will
 *            receive the the bit map (window coordinate frame).
 *   src    - The start of the source image.
 *   origin - The origin of the upper, left-most corner of the full bitmap.
 *            Both dest and origin are in window coordinates, however,
 *            origin may lie outside of the display.
 *   stride - The width of the full source image in bytes.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_NX_RAMBACKED
void nxbe_flush(FAR struct nxbe_window_s *wnd,
                 FAR const struct nxgl_rect_s *dest,
                 FAR const void *src[CONFIG_NX_NPLANES],
                 FAR const struct nxgl_point_s *origin,
                 unsigned int stride);
#endif

/****************************************************************************
 * Name: nxbe_redraw
 *
 * Description:
 *   Re-draw the visible portions of the rectangular region for the
 *   specified window
 *
 ****************************************************************************/

void nxbe_redraw(FAR struct nxbe_state_s *be,
                 FAR struct nxbe_window_s *wnd,
                 FAR const struct nxgl_rect_s *rect);

/****************************************************************************
 * Name: nxbe_redrawbelow
 *
 * Description:
 *   Re-draw the visible portions of the rectangular region for all windows
 *   below (and including) the specified window.  This function is called
 *   whenever a window is closed, moved, lowered or re-sized in order to
 *   expose newly visible portions of lower windows.
 *
 ****************************************************************************/

void nxbe_redrawbelow(FAR struct nxbe_state_s *be,
                      FAR struct nxbe_window_s *wnd,
                      FAR const struct nxgl_rect_s *rect);

/****************************************************************************
 * Name: nxbe_isvisible
 *
 * Description:
 *   Return true if the point, pt, in window wnd is visible.  pt is in
 *   absolute screen coordinates
 *
 ****************************************************************************/

bool nxbe_isvisible(FAR struct nxbe_window_s *wnd,
                    FAR const struct nxgl_point_s *pos);

/****************************************************************************
 * Name: nxbe_clipper
 *
 * Description:
 *   Perform flexible clipping operations.  Callbacks are executed for
 *   each obscured and visible portions of the window.
 *
 * Input Parameters:
 *   wnd  - The window to be clipped.
 *   rect   - The region of concern within the window
 *   order - Specifies the order to process the parts of the non-intersecting
 *           sub-rectangles.
 *   cops  - The callbacks to handle obscured and visible parts of the
 *           sub-rectangles.
 *   plane  - The raster operations to be used by the callback functions.
 *           These may vary with different color formats.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxbe_clipper(FAR struct nxbe_window_s *wnd,
                  FAR const struct nxgl_rect_s *dest, uint8_t order,
                  FAR struct nxbe_clipops_s *cops,
                  FAR struct nxbe_plane_s *plane);

/****************************************************************************
 * Name: nxbe_clipnull
 *
 * Description:
 *   The do-nothing clipping callback function
 *
 ****************************************************************************/

void nxbe_clipnull(FAR struct nxbe_clipops_s *cops,
                   FAR struct nxbe_plane_s *plane,
                   FAR const struct nxgl_rect_s *rect);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __GRAPHICS_NXBE_NXBE_H */
