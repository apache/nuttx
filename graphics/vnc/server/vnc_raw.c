/****************************************************************************
 * graphics/vnc/vnc_raw.c
 *
 *   Copyright (C) 2016-2017 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <assert.h>
#include <errno.h>

#if defined(CONFIG_VNCSERVER_DEBUG) && !defined(CONFIG_DEBUG_GRAPHICS)
#  undef  CONFIG_DEBUG_ERROR
#  undef  CONFIG_DEBUG_WARN
#  undef  CONFIG_DEBUG_INFO
#  undef  CONFIG_DEBUG_GRAPHICS_ERROR
#  undef  CONFIG_DEBUG_GRAPHICS_WARN
#  undef  CONFIG_DEBUG_GRAPHICS_INFO
#  define CONFIG_DEBUG_ERROR          1
#  define CONFIG_DEBUG_WARN           1
#  define CONFIG_DEBUG_INFO           1
#  define CONFIG_DEBUG_GRAPHICS       1
#  define CONFIG_DEBUG_GRAPHICS_ERROR 1
#  define CONFIG_DEBUG_GRAPHICS_WARN  1
#  define CONFIG_DEBUG_GRAPHICS_INFO  1
#endif
#include <debug.h>

#include "vnc_server.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vnc_copy8
 *
 * Description:
 *   Copy a 16/32-bit pixels from the source rectangle to a 8-bit pixel
 *   destination rectangle.
 *
 * Input Parameters:
 *   session      - A reference to the VNC session structure.
 *   row,col      - The upper left X/Y (pixel/row) position of the rectangle
 *   width,height - The width (pixels) and height (rows of the rectangle)
 *   convert      - The function to use to convert from the local framebuffer
 *                  color format to the remote framebuffer color format.
 *
 * Returned Value:
 *   The size of the transfer in bytes.
 *
 ****************************************************************************/

static size_t vnc_copy8(FAR struct vnc_session_s *session,
                         nxgl_coord_t row, nxgl_coord_t col,
                         nxgl_coord_t height, nxgl_coord_t width,
                         vnc_convert8_t convert)
{
  FAR struct rfb_framebufferupdate_s *update;
  FAR const lfb_color_t *srcleft;
  FAR const lfb_color_t *src;
  FAR uint8_t *dest;
  nxgl_coord_t x;
  nxgl_coord_t y;

  /* Destination rectangle start address */

  update = (FAR struct rfb_framebufferupdate_s *)session->outbuf;
  dest   = (FAR uint8_t *)update->rect[0].data;

  /* Source rectangle start address (left/top)*/

  srcleft = (FAR lfb_color_t *)
    (session->fb + RFB_STRIDE * row + RFB_BYTESPERPIXEL * col);

  /* Transfer each row from the source buffer into the update buffer */

  for (y = 0; y < height; y++)
    {
      src = srcleft;
      for (x = 0; x < width; x++)
        {
          *dest++ = convert(*src);
          src++;
        }

      srcleft = (FAR lfb_color_t *)((uintptr_t)srcleft + RFB_STRIDE);
    }

  return (size_t)((uintptr_t)dest - (uintptr_t)update->rect[0].data);
}

/****************************************************************************
 * Name: vnc_copy16
 *
 * Description:
 *   Copy a 16/32-bit pixels from the source rectangle to a 16-bit pixel
 *   destination rectangle.
 *
 * Input Parameters:
 *   session      - A reference to the VNC session structure.
 *   row,col      - The upper left X/Y (pixel/row) position of the rectangle
 *   width,height - The width (pixels) and height (rows of the rectangle)
 *   convert      - The function to use to convert from the local framebuffer
 *                  color format to the remote framebuffer color format.
 *
 * Returned Value:
 *   The size of the transfer in bytes.
 *
 ****************************************************************************/

static size_t vnc_copy16(FAR struct vnc_session_s *session,
                         nxgl_coord_t row, nxgl_coord_t col,
                         nxgl_coord_t height, nxgl_coord_t width,
                         vnc_convert16_t convert)
{
  FAR struct rfb_framebufferupdate_s *update;
  FAR const lfb_color_t *srcleft;
  FAR const lfb_color_t *src;
  FAR uint8_t *dest;
  uint16_t pixel;
  nxgl_coord_t x;
  nxgl_coord_t y;
  bool bigendian;

  /* Destination rectangle start address */

  update = (FAR struct rfb_framebufferupdate_s *)session->outbuf;
  dest   = (FAR uint8_t *)update->rect[0].data;

  /* Source rectangle start address (left/top)*/

  srcleft = (FAR lfb_color_t *)
    (session->fb + RFB_STRIDE * row + RFB_BYTESPERPIXEL * col);

  /* Transfer each row from the source buffer into the update buffer */

  bigendian = session->bigendian;
  for (y = 0; y < height; y++)
    {
      src = srcleft;
      for (x = 0; x < width; x++)
        {
          pixel = convert(*src);

          if (bigendian)
            {
              rfb_putbe16(dest, pixel);
            }
          else
            {
              rfb_putle16(dest, pixel);
            }

          dest += sizeof(uint16_t);
          src++;
        }

      srcleft = (FAR lfb_color_t *)((uintptr_t)srcleft + RFB_STRIDE);
    }

  return (size_t)((uintptr_t)dest - (uintptr_t)update->rect[0].data);
}

/****************************************************************************
 * Name: vnc_copy32
 *
 * Description:
 *   Copy a 16/32-bit pixels from the source rectangle to a 32-bit pixel
 *   destination rectangle.
 *
 * Input Parameters:
 *   session      - A reference to the VNC session structure.
 *   row,col      - The upper left X/Y (pixel/row) position of the rectangle
 *   width,height - The width (pixels) and height (rows of the rectangle)
 *   convert      - The function to use to convert from the local framebuffer
 *                  color format to the remote framebuffer color format.
 *
 * Returned Value:
 *   The size of the transfer in bytes.
 *
 ****************************************************************************/

static size_t vnc_copy32(FAR struct vnc_session_s *session,
                         nxgl_coord_t row, nxgl_coord_t col,
                         nxgl_coord_t height, nxgl_coord_t width,
                         vnc_convert32_t convert)
{
  FAR struct rfb_framebufferupdate_s *update;
  FAR const lfb_color_t *srcleft;
  FAR const lfb_color_t *src;
  FAR uint8_t *dest;
  nxgl_coord_t x;
  nxgl_coord_t y;
  uint32_t pixel;
  bool bigendian;

  /* Destination rectangle start address */

  update = (FAR struct rfb_framebufferupdate_s *)session->outbuf;
  dest   = (FAR uint8_t *)update->rect[0].data;

  /* Source rectangle start address (left/top)*/

  srcleft = (FAR lfb_color_t *)
    (session->fb + RFB_STRIDE * row + RFB_BYTESPERPIXEL * col);

  /* Transfer each row from the source buffer into the update buffer */

  bigendian = session->bigendian;
  for (y = 0; y < height; y++)
    {
      src = srcleft;
      for (x = 0; x < width; x++)
        {
          pixel = convert(*src);

          if (bigendian)
            {
              rfb_putbe32(dest, pixel);
            }
          else
            {
              rfb_putle32(dest, pixel);
            }

          dest += sizeof(uint32_t);
          src++;
        }

      srcleft = (FAR lfb_color_t *)((uintptr_t)srcleft + RFB_STRIDE);
    }

  return (size_t)((uintptr_t)srcleft - (uintptr_t)update->rect[0].data);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vnc_raw
 *
 * Description:
 *  As a fallback, send the framebuffer update using the RAW encoding which
 *  must be supported by all VNC clients.
 *
 * Input Parameters:
 *   session - An instance of the session structure.
 *   rect  - Describes the rectangle in the local framebuffer.
 *
 * Returned Value:
 *   Zero (OK) on success; A negated errno value is returned on failure that
 *   indicates the nature of the failure.  A failure is only returned
 *   in cases of a network failure and unexpected internal failures.
 *
 ****************************************************************************/

int vnc_raw(FAR struct vnc_session_s *session, FAR struct nxgl_rect_s *rect)
{
  FAR struct rfb_framebufferupdate_s *update;
  FAR const uint8_t *src;
  nxgl_coord_t srcwidth;
  nxgl_coord_t srcheight;
  nxgl_coord_t destwidth;
  nxgl_coord_t destheight;
  nxgl_coord_t deststride;
  nxgl_coord_t updwidth;
  nxgl_coord_t updheight;
  nxgl_coord_t width;
  nxgl_coord_t x;
  nxgl_coord_t y;
  unsigned int bytesperpixel;
  unsigned int maxwidth;
  size_t size;
  ssize_t nsent;
  uint8_t colorfmt;

  union
  {
    vnc_convert8_t bpp8;
    vnc_convert16_t bpp16;
    vnc_convert32_t bpp32;
  } convert;

  /* Set up characteristics of the client pixel format to use on this
   * update.  These can change at any time if a SetPixelFormat is
   * received asynchronously.
   */

  bytesperpixel = (session->bpp + 7) >> 3;
  maxwidth      = CONFIG_VNCSERVER_UPDATE_BUFSIZE / bytesperpixel;

  /* Set up the color conversion */

  colorfmt = session->colorfmt;
  switch (colorfmt)
    {
      case FB_FMT_RGB8_222:
        convert.bpp8 = vnc_convert_rgb8_222;
        break;

      case FB_FMT_RGB8_332:
        convert.bpp8 = vnc_convert_rgb8_332;
        break;

      case FB_FMT_RGB16_555:
        convert.bpp16 = vnc_convert_rgb16_555;
        break;

      case FB_FMT_RGB16_565:
        convert.bpp16 = vnc_convert_rgb16_565;
        break;

      case FB_FMT_RGB32:
        convert.bpp32 = vnc_convert_rgb32_888;
        break;

      default:
        gerr("ERROR: Unrecognized color format: %d\n", session->colorfmt);
        return -EINVAL;
    }

  /* Get with width and height of the source and destination rectangles.
   * The source rectangle many be larger than the destination rectangle.
   * In that case, we will have to emit multiple rectangles.
   */

  DEBUGASSERT(rect->pt1.x <= rect->pt2.x);
  srcwidth = rect->pt2.x - rect->pt1.x + 1;

  DEBUGASSERT(rect->pt1.y <= rect->pt2.y);
  srcheight = rect->pt2.y - rect->pt1.y + 1;

  deststride = srcwidth * bytesperpixel;
  if (deststride > maxwidth)
    {
      deststride = maxwidth;
    }

  destwidth  = deststride / bytesperpixel;
  destheight = CONFIG_VNCSERVER_UPDATE_BUFSIZE / deststride;

  if (destheight > srcheight)
    {
      destheight = srcheight;
    }

  /* Format the rectangle header.  We may have to send several update
   * messages if the pre-allocated outbuf is smaller than the rectangle.
   * Each update contains a small "sub-rectangle" of the origin update.
   *
   * Loop until all sub-rectangles have been output.  Start with the
   * top row and transfer rectangles horizontally across each swath.
   * The height of the swath is destwidth (the last may be shorter).
   *
   * NOTE that the loop also terminates of the color format changes
   * asynchronously.
   */

  for (y = rect->pt1.y;
       srcheight > 0 && colorfmt == session->colorfmt;
       srcheight -= updheight, y += updheight)
    {
      /* updheight = Height to update on this pass through the loop.
       * This will be destheight unless fewer than that number of rows
       * remain.
       */

      updheight = destheight;
      if (updheight > srcheight)
        {
          updheight = srcheight;
        }

      /* Loop until this horizontal swath has been sent to the VNC client.
       * Start with the leftmost pixel and transfer rectangles
       * horizontally with width of destwidth until all srcwidth
       * columns have been transferred (the last rectangle may be
       * narrower).
       *
       * NOTE that the loop also terminates of the color format
       * changes asynchronously.
       */

      for (width = srcwidth, x = rect->pt1.x;
           width > 0 && colorfmt == session->colorfmt;
           width -= updwidth, x += updwidth)
        {
          /* updwidth = Width to update on this pass through the loop.
           * This will be destwidth unless fewer than that number of
           * columns remain.
           */

          updwidth = destwidth;
          if (updwidth > width)
            {
              updwidth = width;
            }

          /* Transfer the frame buffer data into the rectangle,
           * performing the necessary color conversions.
           */

          if (bytesperpixel == 1)
            {
              size = vnc_copy8(session, y, x, updheight, updwidth,
                               convert.bpp8);
            }
          else if (bytesperpixel == 2)
            {
              size = vnc_copy16(session, y, x, updheight, updwidth,
                                convert.bpp16);
            }
          else /* bytesperpixel == 4 */
            {
              size = vnc_copy32(session, y, x, updheight, updwidth,
                                convert.bpp32);
            }

          /* Format the FramebufferUpdate message */

          update          = (FAR struct rfb_framebufferupdate_s *)session->outbuf;
          update->msgtype = RFB_FBUPDATE_MSG;
          update->padding = 0;
          rfb_putbe16(update->nrect, 1);

          rfb_putbe16(update->rect[0].xpos, x);
          rfb_putbe16(update->rect[0].ypos, y);
          rfb_putbe16(update->rect[0].width, updwidth);
          rfb_putbe16(update->rect[0].height, updheight);
          rfb_putbe32(update->rect[0].encoding, RFB_ENCODING_RAW);

          DEBUGASSERT(size <= CONFIG_VNCSERVER_UPDATE_BUFSIZE);

          /* We are ready to send the update packet to the VNC client */

          size += SIZEOF_RFB_FRAMEBUFFERUPDATE_S(SIZEOF_RFB_RECTANGE_S(0));
          src   = session->outbuf;

          /* At the very last most, make certain that the color format
           * has not changed asynchronously.
           */

          if (colorfmt == session->colorfmt)
            {
              /* Okay send until all of the bytes are out.  This may
               * loop for the case where TCP write buffering is enabled
               * and there are a limited number of IOBs available.
               */

              do
                {
                  nsent = psock_send(&session->connect, src, size, 0);
                  if (nsent < 0)
                    {
                      gerr("ERROR: Send FrameBufferUpdate failed: %d\n",
                           (int)nsent);
                      return (int)nsent;
                    }

                  DEBUGASSERT(nsent <= size);
                  src  += nsent;
                  size -= nsent;
                }
              while (size > 0);

              updinfo("Sent {(%d, %d),(%d, %d)}\n",
                      x, y, x + updwidth -1, y + updheight - 1);
            }
        }
    }

  return OK;
}
