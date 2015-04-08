/****************************************************************************
 * graphics/nxmu/nxfe.h
 *
 *   Copyright (C) 2008-2013 Gregory Nutt. All rights reserved.
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

#ifndef __GRAPHICS_NXMU_NXFE_H
#define __GRAPHICS_NXMU_NXFE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <mqueue.h>
#include <semaphore.h>

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxmu.h>

#include "nxbe.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Server state structure ***************************************************/

/* This the server 'front-end' state structure */

struct nxfe_state_s
{
  /* The 'back-end' window status.  Must be first so that instances of
   * struct nxbe_state_s can be simply cast to an instance of struct
   * nxfe_state_s
   */

  struct nxbe_state_s be;

  /* This is the server's connection to itself */

  struct nxfe_conn_s conn;
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
 * Public Functions
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
 * Return:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxmu_sendclient(FAR struct nxfe_conn_s *conn,
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
 * Return:
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
 * Return:
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
 * Return:
 *   None
 *
 ****************************************************************************/

void nxmu_requestbkgd(FAR struct nxfe_conn_s *conn,
                      FAR struct nxbe_state_s *be,
                      FAR const struct nx_callback_s *cb,
                      FAR void *arg);

/****************************************************************************
 * Name: nxmu_releasebkgd
 *
 * Description:
 *   Release the background window previously acquired using nxmu_openbgwindow
 *   and return control of the background to NX.
 *
 * Input Parameters:
 *   fe - The front-end state structure
 *
 * Return:
 *   None
 *
 ****************************************************************************/

void nxmu_releasebkgd(FAR struct nxfe_state_s *fe);

/****************************************************************************
 * Name: nxfe_reportposition
 *
 * Description:
 *   Report the new size/position of the window.
 *
 ****************************************************************************/

void nxfe_reportposition(FAR struct nxbe_window_s *wnd);

/****************************************************************************
 * Name: nxfe_redrawreq
 *
 * Description:
 *   Request the client that has this window to redraw the rectangular region.
 *
 ****************************************************************************/

void nxfe_redrawreq(FAR struct nxbe_window_s *wnd,
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
int nxmu_mousein(FAR struct nxfe_state_s *fe,
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
void nxmu_kbdin(FAR struct nxfe_state_s *fe, uint8_t nch, FAR uint8_t *ch);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif  /* __GRAPHICS_NXMU_NXFE_H */
