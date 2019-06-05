/****************************************************************************
 * graphics/vnc/vnc_rre.c
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
 * Private Types
 ****************************************************************************/

struct rre_encode8_s
{
  struct rfb_rrehdr8_s hdr;
  struct rfb_rrerect8_s rect;
};

struct rre_encode16_s
{
  struct rfb_rrehdr16_s hdr;
  struct rfb_rrerect16_s rect;
};

struct rre_encode32_s
{
  struct rfb_rrehdr32_s hdr;
  struct rfb_rrerect32_s rect;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vnc_rreNN
 *
 * Description:
 *  Encode a single RRE sub-rectangle, NN=bits-per-pixel
 *
 * Input Parameters:
 *   session - An instance of the session structure.
 *   dest    - The locate to save the RRE encoded data
 *   rect    - Describes the rectangle in the local framebuffer.
 *   bgcolor - The local color of the pixel data
 *
 * Returned Value:
 *   The size of the framebuffer update message is returned..
 *
 ****************************************************************************/

ssize_t vnc_rre8(FAR struct vnc_session_s *session,
                 FAR struct rre_encode8_s *dest,
                 FAR struct nxgl_rect_s *rect,
                 uint8_t bgcolor)
{
  nxgl_coord_t width;
  nxgl_coord_t height;

  rfb_putbe32(dest->hdr.nsubrects, 1);
  dest->hdr.pixel = bgcolor;

  dest->rect.pixel = bgcolor;
  rfb_putbe16(dest->rect.xpos,   rect->pt1.x);
  rfb_putbe16(dest->rect.ypos,   rect->pt1.y);

  width  = rect->pt2.x - rect->pt1.x + 1;
  height = rect->pt2.y - rect->pt1.y + 1;

  rfb_putbe16(dest->rect.width,  width);
  rfb_putbe16(dest->rect.height, height);

  return sizeof(struct rre_encode8_s);
}

ssize_t vnc_rre16(FAR struct vnc_session_s *session,
                  FAR struct rre_encode16_s *dest,
                  FAR struct nxgl_rect_s *rect,
                  uint16_t bgcolor)
{
  nxgl_coord_t width;
  nxgl_coord_t height;

  rfb_putbe32(dest->hdr.nsubrects, 1);
  rfb_putbe16(dest->hdr.pixel,   bgcolor);

  rfb_putbe16(dest->rect.pixel,  bgcolor);
  rfb_putbe16(dest->rect.xpos,   rect->pt1.x);
  rfb_putbe16(dest->rect.xpos,   rect->pt1.x);
  rfb_putbe16(dest->rect.ypos,   rect->pt1.y);

  width  = rect->pt2.x - rect->pt1.x + 1;
  height = rect->pt2.y - rect->pt1.y + 1;

  rfb_putbe16(dest->rect.width,  width);
  rfb_putbe16(dest->rect.height, height);

  return sizeof(struct rre_encode16_s);
}

ssize_t vnc_rre32(FAR struct vnc_session_s *session,
                  FAR struct rre_encode32_s *dest,
                  FAR struct nxgl_rect_s *rect,
                  uint32_t bgcolor)
{
  nxgl_coord_t width;
  nxgl_coord_t height;

  rfb_putbe32(dest->hdr.nsubrects, 1);
  rfb_putbe32(dest->hdr.pixel,   bgcolor);

  rfb_putbe32(dest->rect.pixel,  bgcolor);
  rfb_putbe16(dest->rect.xpos,   rect->pt1.x);
  rfb_putbe16(dest->rect.xpos,   rect->pt1.x);
  rfb_putbe16(dest->rect.ypos,   rect->pt1.y);

  width  = rect->pt2.x - rect->pt1.x + 1;
  height = rect->pt2.y - rect->pt1.y + 1;

  rfb_putbe16(dest->rect.width,  width);
  rfb_putbe16(dest->rect.height, height);

  return sizeof(struct rre_encode32_s);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vnc_rre
 *
 * Description:
 *  This function does not really do RRE encoding.  It just checks if the
 *  update region is one color then uses the RRE encoding format to send
 *  the constant color rectangle.
 *
 * Input Parameters:
 *   session - An instance of the session structure.
 *   rect  - Describes the rectangle in the local framebuffer.
 *
 * Returned Value:
 *   Zero is returned if RRE coding was not performed (but no error was
 *   encountered).  Otherwise, the size of the framebuffer update message
 *   is returned on success or a negated errno value is returned on failure
 *   that indicates the nature of the failure.  A failure is only
 *   returned in cases of a network failure and unexpected internal failures.
 *
 ****************************************************************************/

int vnc_rre(FAR struct vnc_session_s *session, FAR struct nxgl_rect_s *rect)
{
  FAR struct rfb_framebufferupdate_s *rre;
  FAR struct rfb_rectangle_s *rrect;
  lfb_color_t bgcolor;
  nxgl_coord_t width;
  nxgl_coord_t height;
  size_t nbytes;
  ssize_t nsent;
  int ret;

  /* Check if the client supports the RRE encoding */

  if (session->rre)
    {
      /* Check if the update region contains only a single color */

      ret = vnc_colors(session, rect, 1, &bgcolor);
      if (ret == 1)
        {
          width  = rect->pt2.x - rect->pt1.x + 1;
          height = rect->pt2.y - rect->pt1.y + 1;

          /* Format the FrameBuffer Update with a single RRE encoded
           * rectangle.
           */

          rre           = (FAR struct rfb_framebufferupdate_s *)session->outbuf;
          rre->msgtype  = RFB_FBUPDATE_MSG;
          rre->padding  = 0;
          rfb_putbe16(rre->nrect,          1);

          rrect         = (FAR struct rfb_rectangle_s *)&rre->rect;
          rfb_putbe16(rrect->xpos,         rect->pt1.x);
          rfb_putbe16(rrect->ypos,         rect->pt1.y);
          rfb_putbe16(rrect->width,        width);
          rfb_putbe16(rrect->height,       height);
          rfb_putbe32(rrect->encoding,     RFB_ENCODING_RRE);

          /* The sub-rectangle encoding depends of the remote pixel width */

          nbytes = SIZEOF_RFB_FRAMEBUFFERUPDATE_S(SIZEOF_RFB_RECTANGE_S(0));

          switch (session->colorfmt)
            {
              case FB_FMT_RGB8_222:
                nbytes += vnc_rre8(session,
                                   (FAR struct rre_encode8_s *)rrect->data,
                                   rect, vnc_convert_rgb8_222(bgcolor));
                break;

              case FB_FMT_RGB8_332:
                nbytes += vnc_rre8(session,
                                   (FAR struct rre_encode8_s *)rrect->data,
                                   rect, vnc_convert_rgb8_332(bgcolor));
                break;

              case FB_FMT_RGB16_555:
                nbytes += vnc_rre16(session,
                                    (FAR struct rre_encode16_s *)rrect->data,
                                    rect, vnc_convert_rgb16_555(bgcolor));
                break;

              case FB_FMT_RGB16_565:
                nbytes += vnc_rre16(session,
                                    (FAR struct rre_encode16_s *)rrect->data,
                                    rect, vnc_convert_rgb16_565(bgcolor));
                break;

              case FB_FMT_RGB32:
                nbytes += vnc_rre32(session,
                                    (FAR struct rre_encode32_s *)rrect->data,
                                    rect, vnc_convert_rgb32_888(bgcolor));
                break;

              default:
                gerr("ERROR: Unrecognized color format: %d\n",
                     session->colorfmt);
                return -EINVAL;
            }

          /* At the very last most, make certain that the supported encoding
           * has not changed asynchronously.
           */

          if (session->rre)
            {
              /* Okay send until all of the bytes are out.  This may
               * loop for the case where TCP write buffering is enabled
               * and there are a limited number of IOBs available.
               */

              nsent = psock_send(&session->connect, rre, nbytes, 0);
              if (nsent < 0)
                {
                  gerr("ERROR: Send RRE FrameBufferUpdate failed: %d\n",
                       (int)nsent);
                  return (int)nsent;
                }

              DEBUGASSERT(nsent == nbytes);
              updinfo("Sent {(%d, %d),(%d, %d)}\n",
                      rect->pt1.x, rect->pt1.y, rect->pt2.x, rect->pt2.y);
              return nbytes;
            }

          return -EINVAL;
        }
    }

  return 0;
}
