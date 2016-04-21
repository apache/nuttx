/****************************************************************************
 * graphics/vnc/vnc_rre.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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
#include <debug.h>

#include "vnc_server.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rre_data_s
{
#if RFB_BITSPERPIXEL == 16
  struct rfb_rrehdr16_s hdr;    /*    Header with background pixel */
  struct rfb_rrerect16_s srect; /*    Sub-rectangle */
#elif RFB_BITSPERPIXEL == 32
  struct rfb_rrehdr32_s hdr;    /*    Header with background pixel */
  struct rfb_rrerect32_s srect; /*    Sub-rectangle */
#endif
};

struct rre_rectangle_s
{
  uint8_t xpos[2];              /* U16 X position */
  uint8_t ypos[2];              /* U16 Y position */
  uint8_t width[2];             /* U16 Width */
  uint8_t height[2];            /* U16 Height */
  uint8_t encoding[4];          /* S32 Encoding type = RFB_ENCODING_RRE */
  struct rre_data_s data;       /* Pixel data, actual size varies */
};

struct rre_framebufferupdate_s
{
  uint8_t msgtype;              /* U8  Message type = RFB_FBUPDATE_MSG*/
  uint8_t padding;
  uint8_t nrect[2];             /* U16 Number of rectangles  = 1*/
  struct rre_rectangle_s rect;  /*     RRE encoded rectangle */
};

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
 *   Zero is returned if RRE coding was not performed (but not error was)
 *   encountered.  Otherwise, eith the size of the framebuffer update
 *   message is returned on success or a negated errno value is returned on
 *   failure that indicates the the nature of the failure.  A failure is
 *   only returned in cases of a network failure and unexpected internal
 *   failures.
 *
 ****************************************************************************/

int vnc_rre(FAR struct vnc_session_s *session, FAR struct nxgl_rect_s *rect)
{
  FAR struct rre_framebufferupdate_s *rre;
  FAR struct rre_rectangle_s *rrect;
  FAR struct rre_data_s *rdata;
  lfb_color_t bgcolor;
  nxgl_coord_t width;
  nxgl_coord_t height;
  ssize_t nsent;
  int ret;

  /* Check if the client supports the RRE encoding */

  if (session->rre)
    {
      /* Check if the update region contains only a single color */

      ret = vnc_colors(session, rect, 1, &bgcolor);
      if (ret == 1)
        {
          width         = rect->pt2.x - rect->pt1.x + 1;
          height        = rect->pt2.y - rect->pt1.y + 1;

          /* Format the FrameBuffer Update with a single RRE encoded
           * rectangle.
           */

          rre           = (FAR struct rre_framebufferupdate_s *)session->outbuf;
          rre->msgtype  = RFB_FBUPDATE_MSG;
          rre->padding  = 0;
          rfb_putbe16(rre->nrect,          1);

          rrect         = (FAR struct rre_rectangle_s *)&rre->rect;
          rfb_putbe16(rrect->xpos,         rect->pt1.x);
          rfb_putbe16(rrect->ypos,         rect->pt1.y);
          rfb_putbe16(rrect->width,        width);
          rfb_putbe16(rrect->height,       height);
          rfb_putbe32(rrect->encoding,     RFB_ENCODING_RRE);

          rdata         = (FAR struct rre_data_s *)&rrect->data;
          rfb_putbe32(rdata->hdr.nsubrects, 1);
#if RFB_BITSPERPIXEL == 16
          rfb_putbe16(rdata->hdr.pixel,    bgcolor);
          rfb_putbe16(rdata->srect.pixel,  bgcolor);
#elif RFB_BITSPERPIXEL == 32
          rfb_putbe32(rdata->hdr.pixel,    bgcolor);
          rfb_putbe32(rdata->srect.pixel,  bgcolor);
#endif
          rfb_putbe16(rdata->srect.xpos,   rect->pt1.x);
          rfb_putbe16(rdata->srect.ypos,   rect->pt1.y);
          rfb_putbe16(rdata->srect.width,  width);
          rfb_putbe16(rdata->srect.height, height);

          /* At the very last most, make certain that the supported encoding
           * has not changed asynchronously.
           */

          if (session->rre)
            {
              /* Okay send until all of the bytes are out.  This may
               * loop for the case where TCP write buffering is enabled
               * and there are a limited number of IOBs available.
               */

              nsent = psock_send(&session->connect, rre,
                                 sizeof(struct rre_framebufferupdate_s), 0);
              if (nsent < 0)
                {
                  int errcode = get_errno();
                  gdbg("ERROR: Send RRE FrameBufferUpdate failed: %d\n",
                       errcode);
                  DEBUGASSERT(errcode > 0);
                  return -errcode;
                }

              DEBUGASSERT(nsent == sizeof(struct rre_framebufferupdate_s));
              return sizeof(struct rre_framebufferupdate_s);
            }

          return -EINVAL;
        }
    }

  return 0;
}