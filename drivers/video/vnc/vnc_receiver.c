/****************************************************************************
 * drivers/video/vnc/vnc_receiver.c
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
#include <nuttx/input/mouse.h>

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

                      /* REVISIT:
                       * We are going to be putting garbage on the RFB
                       */

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

                  encodings = (FAR struct rfb_setencodings_s *)
                               session->inbuf;

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
              struct fb_area_s rect;

              ginfo("Received FramebufferUpdateRequest\n");

              /* Read the rest of the FramebufferUpdateRequest message */

              ret = vnc_read_remainder(session,
                                       sizeof(struct rfb_fbupdatereq_s) - 1,
                                       1);
              if (ret < 0)
                {
                  gerr("ERROR: "
                     "Failed to read FramebufferUpdateRequest message: %d\n",
                       ret);
                }
              else
                {
                  /* Enqueue the update */

                  update = (FAR struct rfb_fbupdatereq_s *)session->inbuf;

                  rect.x = rfb_getbe16(update->xpos);
                  rect.y = rfb_getbe16(update->ypos);
                  rect.w = rfb_getbe16(update->width);
                  rect.h = rfb_getbe16(update->height);

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
                  vnc_key_map(session, rfb_getbe32(keyevent->key),
                              (bool)keyevent->down);
                }
            }
            break;

          case RFB_POINTEREVENT_MSG:   /* PointerEvent */
            {
              FAR struct rfb_pointerevent_s *event;
              uint8_t buttons;

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
                      buttons |= MOUSE_BUTTON_1;
                    }

                  if ((event->buttons & (1 << 1)) != 0)
                    {
                      buttons |= MOUSE_BUTTON_2;
                    }

                  if ((event->buttons & (1 << 2)) != 0)
                    {
                      buttons |= MOUSE_BUTTON_3;
                    }

                  session->mouseout(session->arg,
                                    (fb_coord_t)rfb_getbe16(event->xpos),
                                    (fb_coord_t)rfb_getbe16(event->ypos),
                                    buttons);
                }
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
