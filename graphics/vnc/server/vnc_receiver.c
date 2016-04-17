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
#include <debug.h>

#include <nuttx/net/net.h>
#include <nuttx/video/rfb.h>

#include "vnc_server.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vnc_receiver
 *
 * Description:
 *  This function handles all Client-to-Server messages.
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
  ssize_t nrecvd;
  int errcode;

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
          errcode = get_errno();
          gdbg("ERROR: Receive byte failed: %d\n", errcode);
          DEBUGASSERT(errcode > 0);
          return -errcode;
        }

      DEBUGASSERT(nrecvd == 1);

      /* The single byte received should be the message type.  Handle the
       * message according to this message type.
       */

      switch (session->inbuf[0])
        {
          case RFB_SETPIXELFMT_MSG:    /* SetPixelFormat */
#warning Missing logic
            break;

          case RFB_SETENCODINGS_MSG:   /* SetEncodings */
#warning Missing logic
            break;

          case RFB_FBUPDATEREQ_MSG:    /* FramebufferUpdateRequest */
#warning Missing logic
            break;

          case RFB_KEYEVENT_MSG:       /* KeyEvent */
#warning Missing logic
            break;

          case RFB_POINTEREVENT_MSG:   /* PointerEvent */
#warning Missing logic
            break;

          case RFB_CLIENTCUTTEXT_MSG:  /* ClientCutText */
#warning Missing logic
            break;

          default:
            gdbg("ERROR: Unsynchronized, msgtype=%d\n", session->inbuf[0]);
            return -EPROTO;
        }
    }

  return -ENOSYS;
}
