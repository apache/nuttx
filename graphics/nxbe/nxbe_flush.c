/****************************************************************************
 * graphics/nxbe/nxbe_flush.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/nx/nxglib.h>
#include <nuttx/nx/nxbe.h>

#include "nxbe.h"

#ifdef CONFIG_NX_RAMBACKED

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxbe_sprite_refresh
 *
 * Description:
 *   Prior to calling nxbe_bitmap_dev(), update any "sprites" tht need to
 *   be overlaid on the per-window frambuffer.  This could include such
 *   things as OSD functionality, a software cursor, selection boxes, etc.
 *
 * Input Parameters (same as for nxbe_flush):
 *   wnd    - The window that will receive the bitmap image
 *   dest   - Describes the rectangular region on the display that was
 *            modified (in device coordinates)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if 0 /* There are none yet */
void nxbe_sprite_refresh(FAR struct nxbe_window_s *wnd,
                        FAR const struct nxgl_rect_s *dest)
{
  /* Sprite support has not yet been implemented */
}
#endif

/****************************************************************************
 * Name: nxbe_flush
 *
 * Description:
 *   After per-window frambuffer has been updated, the modified region must
 *   be written to device graphics memory.  That function is managed by this
 *   simple function.  It does the following:
 *
 *   1) It calls nxbe_sprite_refresh() to update any "sprite" graphics on top
 *      of the RAM framebuffer.   This could include such things as OSD
 *      functionality, a software cursor, selection boxes, etc.
 *   2) Then it calls nxbe_bitmap_dev() to copy the modified per-window
 *      frambuffer into device memory.
 *
 *   This the "sprite" image is always on top of the device display, this
 *   supports flicker-free software sprites.
 *
 * Input Parameters (same as for nxbe_flush):
 *   wnd    - The window that will receive the bitmap image
 *   dest   - Describes the rectangular on the display that will receive the
 *            the bit map.
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

void nxbe_flush(FAR struct nxbe_window_s *wnd,
                FAR const struct nxgl_rect_s *dest,
                FAR const void *src[CONFIG_NX_NPLANES],
                FAR const struct nxgl_point_s *origin,
                unsigned int stride)
{
  /* Update any "sprite" graphics on top of the display.  These may have been
   * damaged by the preceding framebuffer update.
   */

  nxbe_sprite_refresh(wnd, dest);

  /* Copy the modified per-window frambuffer into device memory.  Since the
   * "sprite" graphics were refreshed after the update, then should be no
   * flicker as you see with a direct update of the device graphics memory.
   */

  nxbe_bitmap_dev(wnd, dest, src, origin, stride);
}

#endif  /* CONFIG_NX_RAMBACKED */

