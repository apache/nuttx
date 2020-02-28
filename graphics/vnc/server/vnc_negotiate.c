/****************************************************************************
 * graphics/vnc/vnc_negotiate.c
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
#include <string.h>
#include <errno.h>
#include <assert.h>

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

#ifdef CONFIG_NET_SOCKOPTS
#  include <sys/time.h>
#endif

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
static const char g_nosecurity[] = "No security types are supported";
#endif
static const char g_vncname[] = CONFIG_VNCSERVER_NAME;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vnc_negotiate
 *
 * Description:
 *  Perform the VNC initialization sequence after the client has successfully
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

int vnc_negotiate(FAR struct vnc_session_s *session)
{
#ifdef CONFIG_VNCSERVER_PROTO3p3
  FAR struct rfb_sectype_s *sectype;
#else /* ifdef CONFIG_VNCSERVER_PROTO3p8 */
  FAR struct rfb_supported_sectypes_s *sectypes;
  FAR struct rfb_selected_sectype_s *sectype;
  FAR struct rfb_sectype_result_s *secresult;
  FAR struct rfb_sectype_fail_s *secfail;
#endif
  FAR struct rfb_serverinit_s *serverinit;
  FAR struct rfb_pixelfmt_s *pixelfmt;
  FAR struct rfb_setpixelformat_s *setformat;
  FAR struct rfb_setencodings_s *encodings;
  ssize_t nsent;
  ssize_t nrecvd;
  size_t len;

#ifdef CONFIG_NET_SOCKOPTS
  struct timeval tv;
  int ret;

  /* Set a receive timeout so that we don't hang if the client does not
   * respond according to RFB 3.3 protocol.
   */

  tv.tv_sec  = 5;
  tv.tv_usec = 0;

  ret = psock_setsockopt(&session->connect, SOL_SOCKET, SO_RCVTIMEO,
                         &tv, sizeof(struct timeval));
  if (ret < 0)
    {
      gerr("ERROR: Failed to set receive timeout: %d\n", ret);
      return ret;
    }
#endif

  /* Inform the client of the VNC protocol version */

  ginfo("Send protocol version: %s\n", g_vncproto);

  len = strlen(g_vncproto);
  nsent = psock_send(&session->connect, g_vncproto, len, 0);
  if (nsent < 0)
    {
      gerr("ERROR: Send ProtocolVersion failed: %d\n", (int)nsent);
      return (int)nsent;
    }

  DEBUGASSERT(nsent == len);

  /* Receive the echo of the protocol string */

  ginfo("Receive echo from VNC client\n");

  nrecvd = psock_recv(&session->connect, session->inbuf, len, 0);
  if (nrecvd < 0)
    {
      gerr("ERROR: Receive protocol confirmation failed: %d\n", (int)nrecvd);
      return (int)nrecvd;
    }
  else if (nrecvd == 0)
    {
      gwarn("WARNING: Connection closed\n");
      return -ECONNABORTED;
    }

  DEBUGASSERT(nrecvd == len);

#ifdef CONFIG_VNCSERVER_PROTO3p3
  /* Version 3.3: The server decides the security type and sends a single
   * word containing the security type:  Tell the client that we won't use
   * any stinkin' security.
   */

  ginfo("Send SecurityType\n");

  sectype = (FAR struct rfb_sectype_s *)session->outbuf;
  rfb_putbe32(sectype->type, RFB_SECTYPE_NONE);

  nsent = psock_send(&session->connect, sectype,
                     sizeof(struct rfb_sectype_s), 0);
  if (nsent < 0)
    {
      gerr("ERROR: Send Security failed: %d\n", (int)nsent);
      return (int)nsent;
    }

  DEBUGASSERT(nsent == sizeof(struct rfb_sectype_s));

#else /* ifdef CONFIG_VNCSERVER_PROTO3p8 */
  /* Version 3.8: Offer the client a choice of security -- where None is the
   * only option offered.
   */

  ginfo("Send SupportedSecurityTypes\n");

  sectypes         = (FAR struct rfb_supported_sectypes_s *)session->outbuf;
  sectypes->ntypes = 1;
  sectypes->type[0] = RFB_SECTYPE_NONE;

  nsent = psock_send(&session->connect, sectypes,
                     SIZEOF_RFB_SUPPORTED_SECTYPES_S(1), 0);
  if (nsent < 0)
    {
      gerr("ERROR: Send SupportedSecurityTypes failed: %d\n", (int)nsent);
      return (int)nsent;
    }

  DEBUGASSERT(nsent == SIZEOF_RFB_SUPPORTED_SECTYPES_S(1));

  /* If the server listed at least one valid security type supported by the
   * client, the client sends back a single byte indicating which security
   * type is to be used on the connection.
   */

  ginfo("Receive SecurityType\n");

  sectype = (FAR struct rfb_selected_sectype_s *)session->inbuf;

  nrecvd = psock_recv(&session->connect, sectype,
                      sizeof(struct rfb_selected_sectype_s), 0);
  if (nrecvd < 0)
    {
      gerr("ERROR: Receive SecurityType failed: %d\n", (int)nrecvd);
      return (int)nrecvd;
    }
  else if (nrecvd == 0)
    {
      gwarn("WARNING: Connection closed\n");
      return -ECONNABORTED;
    }

  DEBUGASSERT(nrecvd == sizeof(struct rfb_selected_sectype_s));

  ginfo("Send SecurityResult\n");

  secresult = (FAR struct rfb_sectype_result_s *)session->outbuf;

  if (sectype->type != RFB_SECTYPE_NONE)
    {
      gerr("ERROR: Received unsupported SecurityType: %d\n", sectype->type);

      /* REVISIT: Should send the reason string here */

      rfb_putbe32(secresult->result, RFB_SECTYPE_FAIL);

      nsent = psock_send(&session->connect, secresult,
                         sizeof(struct rfb_sectype_result_s), 0);
      if (nsent < 0)
        {
          gerr("ERROR: Send SecurityResult failed: %d\n", (int)nsent);
          return (int)nsent;
        }

      DEBUGASSERT(nsent == sizeof(struct rfb_sectype_result_s));

      ginfo("Send failure reason\n");

      secfail = (FAR struct rfb_sectype_fail_s *)session->outbuf;
      len     = strlen(g_nosecurity);
      rfb_putbe32(secfail->len, len);
      memcpy(secfail->str, g_nosecurity, len);

      nsent = psock_send(&session->connect, secfail,
                         SIZEOF_RFB_SECTYPE_FAIL_S(len), 0);
      if (nsent < 0)
        {
          gerr("ERROR: Send failure reason failed: %d\n", (int)nsent);
          return (int)nsent;
        }

      DEBUGASSERT(nsent == SIZEOF_RFB_SECTYPE_FAIL_S(len));
      return -EPROTONOSUPPORT;
    }

  rfb_putbe32(secresult->result, RFB_SECTYPE_SUCCESS);

  nsent = psock_send(&session->connect, secresult,
                     sizeof(struct rfb_sectype_result_s), 0);
  if (nsent < 0)
    {
      gerr("ERROR: Send SecurityResult failed: %d\n", (int)nsent);
      return (int)nsent;
    }

  DEBUGASSERT(nsent == sizeof(struct rfb_sectype_result_s));
#endif

  /* Receive the ClientInit message
   *
   * "Once the client and server are sure that they’re happy to talk to one
   *  another using the agreed security type, the protocol passes to the
   *  initialization phase. The client sends a ClientInit message followed
   *  by the server sending a ServerInit message."
   *
   * In this implementation, the sharing flag is ignored.
   */

  ginfo("Receive ClientInit\n");

  nrecvd = psock_recv(&session->connect, session->inbuf,
                      sizeof(struct rfb_clientinit_s), 0);
  if (nrecvd < 0)
    {
      gerr("ERROR: Receive ClientInit failed: %d\n", (int)nrecvd);
      return (int)nrecvd;
    }
  else if (nrecvd == 0)
    {
      gwarn("WARNING: Connection closed\n");
      return -ECONNABORTED;
    }

  DEBUGASSERT(nrecvd == sizeof(struct rfb_clientinit_s));

  /* Send the ServerInit message
   *
   * "After receiving the ClientInit message, the server sends a ServerInit
   *  message. This tells the client the width and height of the server’s
   *  framebuffer, its pixel format and the name associated with the desktop:"
   *
   * RealVNC client supports this resolutions:
   *   Full (all available colors) - Max resolution of the platform (TrueColor)
   *   Medium (256 colors) - 256 colors (Paletted)
   *   Low (64 colors) - RGB8 2:2:2 (default, TrueColor)
   *   Very Low (8 colors) - RGB3 1:1:1 (TrueColor)
   */

  ginfo("Send ServerInit\n");

  serverinit          = (FAR struct rfb_serverinit_s *)session->outbuf;

  rfb_putbe16(serverinit->width, CONFIG_VNCSERVER_SCREENWIDTH);
  rfb_putbe16(serverinit->height, CONFIG_VNCSERVER_SCREENHEIGHT);

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

  len                 = strlen(g_vncname);
  rfb_putbe32(serverinit->namelen, len);
  memcpy(serverinit->name, g_vncname, len);

  nsent = psock_send(&session->connect, serverinit,
                     SIZEOF_RFB_SERVERINIT_S(len), 0);
  if (nsent < 0)
    {
      gerr("ERROR: Send ServerInit failed: %d\n", (int)nsent);
      return (int)nsent;
    }

  DEBUGASSERT(nsent == SIZEOF_RFB_SERVERINIT_S(len));

  /* We now expect to receive the SetPixelFormat message from the client.
   * This may override some of our framebuffer settings.
   */

  ginfo("Receive SetPixelFormat\n");

  setformat = (FAR struct rfb_setpixelformat_s *)session->inbuf;

  nrecvd = psock_recv(&session->connect, setformat,
                      sizeof(struct rfb_setpixelformat_s), 0);
  if (nrecvd < 0)
    {
      gerr("ERROR: Receive SetPixelFormat failed: %d\n", (int)nrecvd);
      return (int)nrecvd;
    }
  else if (nrecvd == 0)
    {
      gwarn("WARNING: Connection closed\n");
      return -ECONNABORTED;
    }
  else if (nrecvd != sizeof(struct rfb_setpixelformat_s))
    {
      /* Must not be a SetPixelFormat message? */

      gerr("ERROR: SetFormat wrong size: %d\n", (int)nrecvd);
      return -EPROTO;
    }
  else if (setformat->msgtype != RFB_SETPIXELFMT_MSG)
    {
      gerr("ERROR: Not a SetPixelFormat message: %d\n",
           (int)setformat->msgtype);
      return -EPROTO;
    }

  /* Instantiate the client pixel format, verifying that the client request
   * format is one that we can handle.
   */

  ret = vnc_client_pixelformat(session, &setformat->format);
  if (ret < 0)
    {
      /* We do not support this pixel format */

      gerr("ERROR: PixelFormat not supported\n");
      return ret;
    }

  /* Receive supported encoding types from client. */

  ginfo("Receive encoding types\n");

  encodings = (FAR struct rfb_setencodings_s *)session->inbuf;

  nrecvd = psock_recv(&session->connect, encodings,
                      CONFIG_VNCSERVER_INBUFFER_SIZE, 0);
  if (nrecvd < 0)
    {
      gerr("ERROR: Receive SetEncodings failed: %d\n", (int)nrecvd);
      return (int)nrecvd;
    }
  else if (nrecvd == 0)
    {
      gwarn("WARNING: Connection closed\n");
      return -ECONNABORTED;
    }

  if (encodings->msgtype == RFB_SETENCODINGS_MSG)
    {
      DEBUGASSERT(nrecvd >= SIZEOF_RFB_SETENCODINGS_S(0));

      /* Pick out any mutually supported encodings */

      ret = vnc_client_encodings(session, encodings);
      if (ret < 0)
        {
          gerr("ERROR: vnc_set_encodings failed: %d\n", ret);
          return ret;
        }
    }

  session->state = VNCSERVER_CONFIGURED;
  return OK;
}

/****************************************************************************
 * Name: vnc_client_pixelformat
 *
 * Description:
 *  A Client-to-Sever SetPixelFormat message has been received.  We need to
 *  immediately switch the output color format that we generate.
 *
 * Input Parameters:
 *   session - An instance of the session structure.
 *   pixelfmt - The pixel from the received SetPixelFormat message
 *
 * Returned Value:
 *   Returns zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int vnc_client_pixelformat(FAR struct vnc_session_s *session,
                           FAR struct rfb_pixelfmt_s *pixelfmt)
{
  if (pixelfmt->truecolor == 0)
    {
      /* At present, we support only TrueColor formats */

      gerr("ERROR: No support for palette colors\n");
      return -ENOSYS;
    }

  if (pixelfmt->bpp == 8 && pixelfmt->depth == 6)
    {
      ginfo("Client pixel format: RGB8 2:2:2\n");
      session->colorfmt  = FB_FMT_RGB8_222;
      session->bpp       = 8;
      session->bigendian = false;
    }
  else if (pixelfmt->bpp == 8 && pixelfmt->depth == 8)
    {
      ginfo("Client pixel format: RGB8 3:3:2\n");
      session->colorfmt  = FB_FMT_RGB8_332;
      session->bpp       = 8;
      session->bigendian = false;
    }
  else if (pixelfmt->bpp == 16 && pixelfmt->depth == 15)
    {
      ginfo("Client pixel format: RGB16 5:5:5\n");
      session->colorfmt  = FB_FMT_RGB16_555;
      session->bpp       = 16;
      session->bigendian = (pixelfmt->bigendian != 0) ? true : false;
    }
  else if (pixelfmt->bpp == 16 && pixelfmt->depth == 16)
    {
      ginfo("Client pixel format: RGB16 5:6:5\n");
      session->colorfmt  = FB_FMT_RGB16_565;
      session->bpp       = 16;
      session->bigendian = (pixelfmt->bigendian != 0) ? true : false;
    }
  else if (pixelfmt->bpp == 32 && pixelfmt->depth == 24)
    {
      ginfo("Client pixel format: RGB32 8:8:8\n");
      session->colorfmt  = FB_FMT_RGB32;
      session->bpp       = 32;
      session->bigendian = (pixelfmt->bigendian != 0) ? true : false;
    }
  else if (pixelfmt->bpp == 32 && pixelfmt->depth == 32)
    {
      session->colorfmt  = FB_FMT_RGB32;
      session->bpp       = 32;
      session->bigendian = (pixelfmt->bigendian != 0) ? true : false;
    }
  else
    {
      /* We do not support any other conversions */

      gerr("ERROR: No support for this BPP=%d and depth=%d\n",
            pixelfmt->bpp, pixelfmt->depth);
      return -ENOSYS;
    }

  session->change = true;
  return OK;
}
