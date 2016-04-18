/****************************************************************************
 * graphics/vnc/vnc_negotiate.c
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
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/video/fb.h>
#include <nuttx/video/rfb.h>

#include "vnc_server.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if defined(CONFIG_VNCSERVER_PROTO3p3)
static const char g_vncproto[] = RFB_PROTOCOL_VERSION_3p3;
#elif defined(CONFIG_VNCSERVER_PROTO3p8)
static const char g_vncproto[] = RFB_PROTOCOL_VERSION_3p8;
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vnc_negotiate
 *
 * Description:
 *  Perform the VNC initialization sequence after the client has sucessfully
 *  connected to the server.  Negotiate security, framebuffer and color
 *  properties.
 *
 * Input Parameters:
 *   session - An instance of the session structure.
 *
 * Returned Value:
 *   Returns zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_VNCSERVER_PROTO3p3
int vnc_negotiate(FAR struct vnc_session_s *session)
{
  FAR struct rfb_sectype_s *sectype;
  FAR struct rfb_serverinit_s *serverinit;
  FAR struct rfb_pixelfmt_s *pixelfmt;
  FAR struct rfb_setpixelformat_s *setformat;
  ssize_t nsent;
  ssize_t nrecvd;
  size_t len;
  int errcode;

  /* Inform the client of the VNC protocol version */

  len = strlen(g_vncproto);
  nsent = psock_send(&session->connect, g_vncproto, len, 0);
  if (nsent < 0)
    {
      errcode = get_errno();
      gdbg("ERROR: Send ProtocolVersion failed: %d\n", errcode);
      DEBUGASSERT(errcode > 0);
      return -errcode;
    }

  DEBUGASSERT(nsent == len);

  /* Receive the echo of the protocol string */

  nrecvd = psock_recv(&session->connect, session->inbuf, len, 0);
  if (nrecvd < 0)
    {
      errcode = get_errno();
      gdbg("ERROR: Receive protocol confirmation failed: %d\n", errcode);
      DEBUGASSERT(errcode > 0);
      return -errcode;
    }

  DEBUGASSERT(nrecvd == len);

  /* Tell the client that we won't use any stinkin' security.
   *
   * "Version 3.3 The server decides the security type and sends a single
   *  word:"
   */

  sectype = (FAR struct rfb_sectype_s *)session->outbuf;
  rfb_putbe32(sectype->type, RFB_SECTYPE_NONE);

  nsent = psock_send(&session->connect, sectype,
                     sizeof(struct rfb_sectype_s), 0);
  if (nsent < 0)
    {
      errcode = get_errno();
      gdbg("ERROR: Send Security failed: %d\n", errcode);
      DEBUGASSERT(errcode > 0);
      return -errcode;
    }

  DEBUGASSERT(nsent == sizeof(struct rfb_sectype_s));

  /* Receive the ClientInit message
   *
   * "Once the client and server are sure that they’re happy to talk to one
   *  another using the agreed security type, the protocol passes to the
   *  initialisation phase. The client sends a ClientInit message followed
   *  by the server sending a ServerInit message."
   *
   * In this implementation, the sharing flag is ignored.
   */

  nrecvd = psock_recv(&session->connect, session->inbuf,
                      sizeof(struct rfb_clientinit_s), 0);
  if (nrecvd < 0)
    {
      errcode = get_errno();
      gdbg("ERROR: Receive ClientInit failed: %d\n", errcode);
      DEBUGASSERT(errcode > 0);
      return -errcode;
    }

  DEBUGASSERT(nrecvd == sizeof(struct rfb_clientinit_s));

  /* Send the ClientInit message
   *
   * "After receiving the ClientInit message, the server sends a ServerInit
   *  message. This tells the client the width and height of the server’s 
   *  framebuffer, its pixel format and the name associated with the desktop:
   */

  serverinit          = (FAR struct rfb_serverinit_s *)session->outbuf;

  rfb_putbe16(serverinit->width, CONFIG_VNCSERVER_SCREENWIDTH);
  rfb_putbe16(serverinit->height, CONFIG_VNCSERVER_SCREENHEIGHT);
  rfb_putbe32(serverinit->namelen, 0);

  pixelfmt            = &serverinit->format;
  pixelfmt->bpp       = RFB_BITSPERPIXEL;
  pixelfmt->depth     = RFB_PIXELDEPTH;
  pixelfmt->bigendian = 0;
  pixelfmt->truecolor = RFB_TRUECOLOR;

  rfb_putbe16(pixelfmt->rmax, RFB_RMAX);
  rfb_putbe16(pixelfmt->gmax, RFB_GMAX);
  rfb_putbe16(pixelfmt->bmax, RFB_BMAX);

  pixelfmt->rshift    = RFB_RSHIFT;
  pixelfmt->gshift    = RFB_GSHIFT;
  pixelfmt->bshift    = RFB_BSHIFT;

  nsent = psock_send(&session->connect, serverinit,
                     SIZEOF_RFB_SERVERINIT_S(0), 0);
  if (nsent < 0)
    {
      errcode = get_errno();
      gdbg("ERROR: Send ServerInit failed: %d\n", errcode);
      return -errcode;
    }

  DEBUGASSERT(nsent == SIZEOF_RFB_SERVERINIT_S(0));

  /* We now expect to receive the SetPixelFormat message from the client.
   * This may override some of our framebuffer settings.
   */

  setformat = (FAR struct rfb_setpixelformat_s *)session->inbuf;

  nrecvd = psock_recv(&session->connect, setformat,
                      sizeof(struct rfb_setpixelformat_s), 0);
  if (nrecvd < 0)
    {
      errcode = get_errno();
      gdbg("ERROR: Receive SetFormat failed: %d\n", errcode);
      DEBUGASSERT(errcode > 0);
      return -errcode;
    }
  else if (nrecvd != sizeof(struct rfb_setpixelformat_s))
    {
      /* Must not be a SetPixelFormat message? */

      gdbg("ERROR: SetFormat wrong size: %d\n", (int)nrecvd);
      return -EPROTO;
    }
  else if (setformat->msgtype != RFB_SETPIXELFMT_MSG)
    {
      gdbg("ERROR: Not a SetFormat message: %d\n", (int)setformat->msgtype);
      return -EPROTO;
    }

  /* Check if the client request format is one that we can handle. */

  pixelfmt = &setformat->format;

  if (pixelfmt->truecolor == 0)
    {
      /* At present, we support only TrueColor formats */

      gdbg("ERROR: No support for palette colors\n");
      return -ENOSYS;
    }

  if (pixelfmt->bpp == 16 && pixelfmt->depth == 15)
    {
      session->colorfmt = FB_FMT_RGB16_555;
      session->bpp      = 16;
    }
  else if (pixelfmt->bpp == 16 && pixelfmt->depth == 16)
    {
      session->colorfmt = FB_FMT_RGB16_565;
      session->bpp      = 16;
    }
  else if (pixelfmt->bpp == 32 && pixelfmt->depth == 24)
    {
      session->colorfmt = FB_FMT_RGB32;
      session->bpp      = 32;
    }
  else if (pixelfmt->bpp == 32 && pixelfmt->depth == 32)
    {
      session->colorfmt = FB_FMT_RGB32;
      session->bpp      = 32;
    }
  else
    {
      /* We do not support any other conversions */

      gdbg("ERROR: No support for this BPP=%d and depth=%d\n",
            pixelfmt->bpp, pixelfmt->depth);
      return -ENOSYS;
    }

  /* Receive supported encoding types from client, but ignore them.
   * we will do only raw format.
   */

  (void)psock_recv(&session->connect, session->inbuf,
                   CONFIG_VNCSERVER_INBUFFER_SIZE, 0);

  session->state = VNCSERVER_CONFIGURED;
  return OK;
}
#endif

#ifdef CONFIG_VNCSERVER_PROTO3p8
int vnc_negotiate(FAR struct vnc_session_s *session)
{
  ssize_t nsent;
  ssize_t nrecvd;
  size_t len;

  /* Inform the client of the VNC protocol version */

  len = strlen(g_vncproto);
  nsent = psock_send(&session->connect, g_vncproto, len, 0);
  if (nsent < 0)
    {
      errcode = get_errno();
      gdbg("ERROR: Send ProtocolVersion failed: %d\n", errcode);
      DEBUGASSERT(errcode > 0);
      return -errcode;
    }

  DEBUGASSERT(nsent == len);

  /* Receive the echo of the protocol string */

  nrecvd = psock_recv(&session->connect, session->inbuf, len, 0);
  if (nrecvd <= 0)
    {
      errcode = get_errno();
      gdbg("ERROR: Receive protocol confirmation failed: %d\n", errcode);
      DEBUGASSERT(errcode > 0);
      return -errcode;
    }

  DEBUGASSERT(nrecvd == len);

  /* Offer the client a choice of security -- where None is the only option. */
#warning Missing logic

  return OK;
}
#endif
