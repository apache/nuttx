/****************************************************************************
 * graphics/nxmu/nxmu.h
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

#ifndef __GRAPHICS_NXMU_NXMU_H
#define __GRAPHICS_NXMU_NXMU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <mqueue.h>

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxmu.h>

#include "nxbe.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if CONFIG_MQ_MAXMSGSIZE < 64
#error "The NX Server needs CONFIG_MQ_MAXMSGSIZE >= 64"
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Server state structure ***************************************************/

/* This the server NXMU state structure */

struct nxmu_state_s
{
  /* The 'back-end' window status.  Must be first so that instances of
   * struct nxbe_state_s can be simply cast to an instance of struct
   * nxmu_state_s
   */

  struct nxbe_state_s be;

  /* This is the server's connection to itself */

  struct nxmu_conn_s conn;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: nxmu_sendclient
 *
 * Description:
 *  Send a message to the client at NX_CLIMSG_PRIO priority
 *
 * Input Parameters:
 *   conn   - A pointer to the server connection structure
 *   msg    - A pointer to the message to send
 *   msglen - The length of the message in bytes.
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxmu_sendclient(FAR struct nxmu_conn_s *conn,
                    FAR const void *msg, size_t msglen);

/****************************************************************************
 * Name: nxmu_sendclientwindow
 *
 * Description:
 *  Send a message to the client at NX_CLIMSG_PRIO priority
 *
 * Input Parameters:
 *   wnd    - A pointer to the back-end window structure
 *   msg    - A pointer to the message to send
 *   msglen - The length of the message in bytes.
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxmu_sendclientwindow(FAR struct nxbe_window_s *wnd, FAR const void *msg,
                          size_t msglen);

/****************************************************************************
 * Name: nxmu_openwindow
 *
 * Description:
 *   Create a new window.
 *
 * Input Parameters:
 *   be  - The back-end status structure
 *   wnd  - The pre-allocated window structure to be initialized [IN/OUT]
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxmu_openwindow(FAR struct nxbe_state_s *be,
                     FAR struct nxbe_window_s *wnd);

/****************************************************************************
 * Name: nxmu_requestbkgd
 *
 * Description:
 *   Perform the server-side operation for the nx_requestbkgd operation:
 *   Give the client control of the background window connection and receipt
 *   of all background window callbacks.
 *
 *   conn - The client containing connection information [IN]
 *   be   - The server state structure [IN]
 *   cb   - Callbacks used to process window events
 *   arg  - User provided argument (see nx_openwindow, nx_constructwindow)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxmu_requestbkgd(FAR struct nxmu_conn_s *conn,
                      FAR struct nxbe_state_s *be,
                      FAR const struct nx_callback_s *cb,
                      FAR void *arg);

/****************************************************************************
 * Name: nxmu_releasebkgd
 *
 * Description:
 *   Release the background window previously acquired using
 *   nxmu_openbgwindow and return control of the background to NX.
 *
 * Input Parameters:
 *   nxmu - The NXMU state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxmu_releasebkgd(FAR struct nxmu_state_s *nxmu);

/****************************************************************************
 * Name: nxmu_reportposition
 *
 * Description:
 *   Report the new size/position of the window.
 *
 ****************************************************************************/

void nxmu_reportposition(FAR struct nxbe_window_s *wnd);

/****************************************************************************
 * Name: nxmu_redrawreq
 *
 * Description:
 *  Send a message to the client requesting that it to redraw the rectangular
 *  region in the window.
 *
 ****************************************************************************/

void nxmu_redrawreq(FAR struct nxbe_window_s *wnd,
                    FAR const struct nxgl_rect_s *rect);

/****************************************************************************
 * Name: nxmu_redraw
 *
 * Description:
 *   Redraw client window data.  This may involve either sending a message
 *   to the client requesting that it redraw a region of the window.  Or, in
 *   the base that the window supports a per-window framebuffer, this might
 *   amount to an immediate redraw from the framebuffer.
 *
 ****************************************************************************/

void nxmu_redraw(FAR struct nxbe_window_s *wnd,
                 FAR const struct nxgl_rect_s *rect);

/****************************************************************************
 * Name: nxmu_mouseinit
 *
 * Description:
 *   Initialize with the mouse in the center of the display
 *
 ****************************************************************************/

#ifdef CONFIG_NX_XYINPUT
void nxmu_mouseinit(int x, int y);
#endif

/****************************************************************************
 * Name: nxmu_mousereport
 *
 * Description:
 *   Report mouse position info to the specified window
 *
 * Input Parameters:
 *   wnd - The window to receive the mouse report
 *
 * Returned Value:
 *   0: Mouse report sent; >0: Mouse report not sent; <0: An error occurred
 *
 ****************************************************************************/

#ifdef CONFIG_NX_XYINPUT
int nxmu_mousereport(struct nxbe_window_s *wnd);
#endif

/****************************************************************************
 * Name: nxmu_mousein
 *
 * Description:
 *   New positional data has been received from the thread or interrupt
 *   handler that manages some kind of pointing hardware.  Route that
 *   positional data to the appropriate window client.
 *
 ****************************************************************************/

#ifdef CONFIG_NX_XYINPUT
int nxmu_mousein(FAR struct nxmu_state_s *nxmu,
                 FAR const struct nxgl_point_s *pos, int button);
#endif

/****************************************************************************
 * Name: nxmu_kbdin
 *
 * Description:
 *   New keyboard data has been received from the thread or interrupt
 *   handler that manages some kind of keyboard/keypad hardware.  Route that
 *   positional data to the appropriate window client.
 *
 ****************************************************************************/

#ifdef CONFIG_NX_KBD
void nxmu_kbdin(FAR struct nxmu_state_s *nxmu, uint8_t nch, FAR uint8_t *ch);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __GRAPHICS_NXMU_NXMU_H */
