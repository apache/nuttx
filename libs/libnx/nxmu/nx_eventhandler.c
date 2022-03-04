/****************************************************************************
 * libs/libnx/nxmu/nx_eventhandler.c
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

#include <stdint.h>
#include <stdlib.h>
#include <mqueue.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/mqueue.h>
#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxbe.h>
#include <nuttx/nx/nxmu.h>

#include "nxcontext.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nx_connected
 *
 * Description:
 *   The server has completed the connection and is ready.
 *
 ****************************************************************************/

static inline void nx_connected(FAR struct nxmu_conn_s *conn)
{
  DEBUGASSERT(conn->state == NX_CLISTATE_NOTCONNECTED);
  conn->state = NX_CLISTATE_CONNECTED;
}

/****************************************************************************
 * Name: nx_disconnected
 ****************************************************************************/

static inline void nx_disconnected(FAR struct nxmu_conn_s *conn)
{
  /* Close the server and client MQs */

  _MQ_CLOSE(conn->cwrmq);
  _MQ_CLOSE(conn->crdmq);

  /* And free the client structure */

  lib_ufree(conn);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nx_eventhandler
 *
 * Description:
 *   The client code must call this function periodically to process
 *   incoming messages from the server.  If CONFIG_NX_BLOCKING is defined,
 *   then this function not return until a server message is received.
 *
 *   When CONFIG_NX_BLOCKING is not defined, the client must exercise
 *   caution in the looping to assure that it does not eat up all of
 *   the CPU bandwidth calling nx_eventhandler repeatedly.  nx_eventnotify
 *   may be called to get a signal event whenever a new incoming server
 *   event is available.
 *
 * Input Parameters:
 *   handle - the handle returned by nx_connect
 *
 * Returned Value:
 *     OK: No errors occurred.  If CONFIG_NX_BLOCKING is defined, then
 *         one or more server message was processed.
 *  ERROR: An error occurred and errno has been set appropriately.  Of
 *         particular interest, it will return errno == EHOSTDOWN when the
 *         server is disconnected.  After that event, the handle can no
 *         longer be used.
 *
 ****************************************************************************/

int nx_eventhandler(NXHANDLE handle)
{
  FAR struct nxmu_conn_s *conn = (FAR struct nxmu_conn_s *)handle;
  struct nxsvrmsg_s *msg;
  struct nxbe_window_s *wnd;
  char buffer[NX_MXCLIMSGLEN];
  int nbytes;

  /* Get the next message from our incoming message queue */

  do
    {
      nbytes = _MQ_RECEIVE(conn->crdmq, buffer, NX_MXCLIMSGLEN, 0);
      if (nbytes < 0)
        {
          int errcode = _NX_GETERRNO(nbytes);

          /* EINTR is not an error.  The wait was interrupted by a signal and
           * we just need to try reading again.
           */

          if (errcode != EINTR)
            {
              if (errcode == EAGAIN)
                {
                  /* EAGAIN is not an error.  It occurs because the MQ is
                   * opened with O_NONBLOCK and there is no message
                   * available now.
                   */

                  return OK;
                }
              else
                {
                  gerr("ERROR: _MQ_RECEIVE failed: %d\n", errcode);
                  _NX_SETERRNO(nbytes);
                  return ERROR;
                }
            }
        }
    }
  while (nbytes < 0);

  if (nbytes < sizeof(struct nxclimsg_s))
    {
      _NX_SETERRNO(EBADMSG);
      return ERROR;
    }

  /* Dispatch the message appropriately */

  msg = (struct nxsvrmsg_s *)buffer;
  ginfo("Received msgid=%" PRId32 "\n", msg->msgid);
  switch (msg->msgid)
    {
    case NX_CLIMSG_CONNECTED:
      nx_connected(conn);
      break;

    case NX_CLIMSG_DISCONNECTED:
      nx_disconnected(conn);
      set_errno(EHOSTDOWN);
      return ERROR;

    case NX_CLIMSG_REDRAW:
      {
        FAR struct nxclimsg_redraw_s *redraw =
            (FAR struct nxclimsg_redraw_s *)buffer;
        wnd = redraw->wnd;
        DEBUGASSERT(wnd);
        if (wnd->cb->redraw)
          {
            wnd->cb->redraw((NXWINDOW)wnd, &redraw->rect, redraw->more,
                            wnd->arg);
          }
      }
      break;

    case NX_CLIMSG_NEWPOSITION:
      {
        FAR struct nxclimsg_newposition_s *postn =
            (FAR struct nxclimsg_newposition_s *)buffer;
        wnd = postn->wnd;
        DEBUGASSERT(wnd);
        if (wnd->cb->position)
          {
            wnd->cb->position((NXWINDOW)wnd, &postn->size, &postn->pos,
                              &postn->bounds, wnd->arg);
          }
      }
      break;

#ifdef CONFIG_NX_XYINPUT
    case NX_CLIMSG_MOUSEIN:
      {
        FAR struct nxclimsg_mousein_s *mouse =
            (FAR struct nxclimsg_mousein_s *)buffer;
        wnd = mouse->wnd;
        DEBUGASSERT(wnd);
        if (wnd->cb->mousein)
          {
            wnd->cb->mousein((NXWINDOW)wnd, &mouse->pos, mouse->buttons,
                             wnd->arg);
          }
        }
      break;
#endif

#ifdef CONFIG_NX_KBD
    case NX_CLIMSG_KBDIN:
      {
        FAR struct nxclimsg_kbdin_s *kbd =
            (FAR struct nxclimsg_kbdin_s *)buffer;
        wnd = kbd->wnd;
        DEBUGASSERT(wnd);
        if (wnd->cb->kbdin)
          {
            wnd->cb->kbdin((NXWINDOW)wnd, kbd->nch, kbd->ch, wnd->arg);
          }
        }
      break;
#endif

    case NX_CLIMSG_EVENT:
      {
        FAR struct nxclimsg_event_s *event =
            (FAR struct nxclimsg_event_s *)buffer;
        wnd = event->wnd;
        DEBUGASSERT(wnd);
        if (wnd->cb->event)
          {
            wnd->cb->event((NXWINDOW)wnd, event->event, wnd->arg,
                           event->arg);
          }
        }
      break;

    default:
      gerr("ERROR: Unrecognized message opcode: %" PRId32 "\n",
           ((FAR struct nxsvrmsg_s *)buffer)->msgid);
      break;
    }

  return OK;
}
