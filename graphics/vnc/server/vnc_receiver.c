/****************************************************************************
 * graphics/vnc/vnc_receiver.c
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

#ifdef CONFIG_NET_SOCKOPTS
#  include <sys/time.h>
#endif

#include <nuttx/net/net.h>
#include <nuttx/video/rfb.h>
#include <nuttx/video/vnc.h>
#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxglib.h>

#include "vnc_server.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vnc_read_remainder
 *
 * Description:
 *   After receiving the first byte of a client-to-server message, this
 *   reads in the remainder of the message.
 *
 * Input Parameters:
 *   session - An instance of the session structure.
 *   msglen  - The full length of the message
 *
 * Returned Value:
 *   At present, always returns OK
 *
 ****************************************************************************/

int vnc_read_remainder(FAR struct vnc_session_s *session, size_t msglen,
                       size_t offset)
{
  ssize_t nrecvd;
  size_t ntotal;

  /* Loop until the rest of the message is received. */

  for (ntotal = 0; ntotal < msglen; offset += nrecvd, ntotal += nrecvd)
    {
      /* Receive more of the message */

      nrecvd = psock_recv(&session->connect, &session->inbuf[offset],
                          msglen - ntotal, 0);
      if (nrecvd < 0)
        {
          gerr("ERROR: Receive message failed: %d\n", (int)nrecvd);
          return (int)nrecvd;
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vnc_receiver
 *
 * Description:
 *   This function handles all Client-to-Server messages.
 *
 * Input Parameters:
 *   session - An instance of the session structure.
 *
 * Returned Value:
 *   At present, always returns OK
 *
 ****************************************************************************/

int vnc_receiver(FAR struct vnc_session_s *session)
{
#ifdef CONFIG_NET_SOCKOPTS
  struct timeval tv;
#endif
  ssize_t nrecvd;
  int ret;

  DEBUGASSERT(session);
  ginfo("Receiver running for Display %d\n", session->display);

#ifdef CONFIG_NET_SOCKOPTS
  /* Disable the receive timeout so that we will wait indefinitely for the
   * next Client-to-Server message.
   */

  tv.tv_sec  = 0;
  tv.tv_usec = 0;
  ret = psock_setsockopt(&session->connect, SOL_SOCKET, SO_RCVTIMEO,
                         &tv, sizeof(struct timeval));
  if (ret < 0)
    {
      gerr("ERROR: Failed to disable receive timeout: %d\n", ret);
      return ret;
    }
#endif

  /* Loop until the client disconnects or an unhandled error occurs */

  for (; ; )
    {
      /* Set up to read one byte which should be the message type of the
       * next Client-to-Server message.  We will block here until the message
       * is received.
       */

      nrecvd = psock_recv(&session->connect, session->inbuf, 1, 0);
      if (nrecvd < 0)
        {
          gerr("ERROR: Receive byte failed: %d\n", (int)nrecvd);
          return (int)nrecvd;
        }

      /* A return value of zero means that the connection was gracefully
       * closed by the VNC client.
       */

      else if (nrecvd == 0)
        {
          gwarn("WARNING: Connection closed\n");
          return OK;
        }

      DEBUGASSERT(nrecvd == 1);

      /* The single byte received should be the message type.  Handle the
       * message according to this message type.
       */

      switch (session->inbuf[0])
        {
          case RFB_SETPIXELFMT_MSG:    /* SetPixelFormat */
            {
              ginfo("Received SetPixelFormat\n");

              /* Read the rest of the SetPixelFormat message */

              ret = vnc_read_remainder(session,
                                       sizeof(struct rfb_setpixelformat_s) - 1,
                                       1);
              if (ret < 0)
                {
                  gerr("ERROR: Failed to read SetPixelFormat message: %d\n",
                       ret);
                }
              else
                {
                  FAR struct rfb_setpixelformat_s *setformat =
                    (FAR struct rfb_setpixelformat_s *)session->inbuf;

                  ret = vnc_client_pixelformat(session, &setformat->format);
                  if (ret < 0)
                    {
                      /* We do not support this pixel format */
                      /* REVISIT:  We are going to be putting garbage on the RFB */

                      gerr("ERROR: PixelFormat not supported\n");
                    }
                }
            }
            break;

          case RFB_SETENCODINGS_MSG:   /* SetEncodings */
            {
              FAR struct rfb_setencodings_s *encodings;
              unsigned int nencodings;

              ginfo("Received SetEncodings\n");

              /* Read the SetEncodings message without the following
               * encodings.
               */

              ret = vnc_read_remainder(session,
                                       SIZEOF_RFB_SERVERINIT_S(0) - 1,
                                       1);
              if (ret < 0)
                {
                  gerr("ERROR: Failed to read SetEncodings message: %d\n",
                       ret);
                }
              else
                {
                  /* Read the following encodings */

                  encodings  = (FAR struct rfb_setencodings_s *)session->inbuf;
                  nencodings = rfb_getbe16(encodings->nencodings);

                  ret = vnc_read_remainder(session,
                                           nencodings * sizeof(uint32_t),
                                           SIZEOF_RFB_SERVERINIT_S(0));
                  if (ret < 0)
                    {
                      gerr("ERROR: Failed to read encodings: %d\n",
                           ret);
                    }
                  else
                    {
                      /* Pick out any mutually supported encodings */

                      ret = vnc_client_encodings(session, encodings);
                      if (ret < 0)
                        {
                          gerr("ERROR: vnc_set_encodings failed: %d\n", ret);
                        }
                    }
                }
            }
            break;

          case RFB_FBUPDATEREQ_MSG:    /* FramebufferUpdateRequest */
            {
              FAR struct rfb_fbupdatereq_s *update;
              struct nxgl_rect_s rect;

              ginfo("Received FramebufferUpdateRequest\n");

              /* Read the rest of the FramebufferUpdateRequest message */

              ret = vnc_read_remainder(session,
                                       sizeof(struct rfb_fbupdatereq_s) - 1,
                                       1);
              if (ret < 0)
                {
                  gerr("ERROR: Failed to read FramebufferUpdateRequest message: %d\n",
                       ret);
                }
              else
                {
                  /* Enqueue the update */

                  update = (FAR struct rfb_fbupdatereq_s *)session->inbuf;

                  rect.pt1.x = rfb_getbe16(update->xpos);
                  rect.pt1.y = rfb_getbe16(update->ypos);
                  rect.pt2.x = rect.pt1.x + rfb_getbe16(update->width);
                  rect.pt2.y = rect.pt1.y + rfb_getbe16(update->height);

                  ret = vnc_update_rectangle(session, &rect, false);
                  if (ret < 0)
                    {
                      gerr("ERROR: Failed to queue update: %d\n", ret);
                    }
                }
            }
            break;

          case RFB_KEYEVENT_MSG:       /* KeyEvent */
            {
              FAR struct rfb_keyevent_s *keyevent;

              ginfo("Received KeyEvent\n");

              /* Read the rest of the KeyEvent message */

              ret = vnc_read_remainder(session,
                                       sizeof(struct rfb_keyevent_s) - 1,
                                       1);
              if (ret < 0)
                {
                  gerr("ERROR: Failed to read KeyEvent message: %d\n",
                       ret);
                }
              else
                {
                  /* Inject the key press/release event into NX */

                  keyevent = (FAR struct rfb_keyevent_s *)session->inbuf;
                  vnc_key_map(session, rfb_getbe16(keyevent->key),
                              (bool)keyevent->down);
                }
            }
            break;

          case RFB_POINTEREVENT_MSG:   /* PointerEvent */
            {
#ifdef CONFIG_NX_XYINPUT
              FAR struct rfb_pointerevent_s *event;
              uint8_t buttons;
#endif
              ginfo("Received PointerEvent\n");

              /* Read the rest of the PointerEvent message */

              ret = vnc_read_remainder(session,
                                       sizeof(struct rfb_pointerevent_s) - 1,
                                       1);
              if (ret < 0)
                {
                  gerr("ERROR: Failed to read PointerEvent message: %d\n",
                       ret);
                }
#ifdef CONFIG_NX_XYINPUT
              /* REVISIT:  How will be get the NX handle? */

              else if (session->mouseout != NULL)
                {
                  event = (FAR struct rfb_pointerevent_s *)session->inbuf;

                 /* Map buttons bitmap.  Bits 0-7 are buttons 1-8, 0=up,
                  * 1=down.  By convention Bit 0 = left button, Bit 1 =
                  * middle button, and Bit 2 = right button.
                  */

                  buttons = 0;
                  if ((event->buttons & (1 << 0)) != 0)
                    {
                      buttons |= NX_MOUSE_LEFTBUTTON;
                    }

                  if ((event->buttons & (1 << 1)) != 0)
                    {
                      buttons |= NX_MOUSE_CENTERBUTTON;
                    }

                  if ((event->buttons & (1 << 2)) != 0)
                    {
                      buttons |= NX_MOUSE_RIGHTBUTTON;
                    }

                  session->mouseout(session->arg,
                                    (nxgl_coord_t)rfb_getbe16(event->xpos),
                                    (nxgl_coord_t)rfb_getbe16(event->ypos),
                                    buttons);
                }
#endif
            }
            break;

          case RFB_CLIENTCUTTEXT_MSG:  /* ClientCutText */
            {
              FAR struct rfb_clientcuttext_s *cuttext;
              uint32_t length;

              ginfo("Received ClientCutText\n");

              /* Read the ClientCutText message without the following
               * text.
               */

              ret = vnc_read_remainder(session,
                                       SIZEOF_RFB_CLIENTCUTTEXT_S(0) - 1,
                                       1);
              if (ret < 0)
                {
                  gerr("ERROR: Failed to read ClientCutText message: %d\n",
                       ret);
                }
              else
                {
                  /* Read the following text */

                  cuttext = (FAR struct rfb_clientcuttext_s *)session->inbuf;
                  length  = rfb_getbe32(cuttext->length);

                  ret = vnc_read_remainder(session, length,
                                           SIZEOF_RFB_CLIENTCUTTEXT_S(0));
                  if (ret < 0)
                    {
                      gerr("ERROR: Failed to read text: %d\n",
                           ret);
                    }
                  else
                    {
                      /* REVISIT: ClientCutText is currently ignored */
                    }
                }
            }
            break;

          default:
            gerr("ERROR: Unsynchronized, msgtype=%d\n", session->inbuf[0]);
            return -EPROTO;
        }
    }

  return -ENOSYS;
}

/****************************************************************************
 * Name: vnc_client_encodings
 *
 * Description:
 *   Pick out any mutually supported encodings from the Client-to-Server
 *   SetEncodings message
 *
 * Input Parameters:
 *   session   - An instance of the session structure.
 *   encodings - The received SetEncodings message
 *
 * Returned Value:
 *   At present, always returns OK
 *
 ****************************************************************************/

int vnc_client_encodings(FAR struct vnc_session_s *session,
                         FAR struct rfb_setencodings_s *encodings)
{
  uint32_t encoding;
  unsigned int nencodings;
  unsigned int i;

  DEBUGASSERT(session != NULL && encodings != NULL);

  /* Assume that there are no common encodings (other than RAW) */

  session->rre = false;

  /* Loop for each client supported encoding */

  nencodings = rfb_getbe16(encodings->nencodings);
  for (i = 0; i < nencodings; i++)
    {
      /* Get the next encoding */

      encoding = rfb_getbe32(&encodings->encodings[i << 2]);

      /* Only a limited support for of RRE is available now. */

      if (encoding == RFB_ENCODING_RRE)
        {
          session->rre = true;
        }
    }

  session->change = true;
  return OK;
}

/****************************************************************************
 * Name: vnc_mouse
 *
 * Description:
 *   This is the default keyboard/mouse callout function.  This is simply a
 *   wrapper around nx_mousein().  When
 *   configured using vnc_fbinitialize(), the 'arg' must be the correct
 *   NXHANDLE value.
 *
 * Input Parameters:
 *   See vnc_mouseout_t and vnc_kbdout_t typde definitions above.  These
 *   callouts have arguments that match the inputs to nx_kbdin() and
 *   nx_mousein() (if arg is really of type NXHANDLE).
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_NX_XYINPUT
void vnc_mouseout(FAR void *arg, nxgl_coord_t x, nxgl_coord_t y,
                  uint8_t buttons)
{
  DEBUGASSERT(arg != NULL);
  nx_mousein((NXHANDLE)arg, x, y, buttons);
}
#endif
