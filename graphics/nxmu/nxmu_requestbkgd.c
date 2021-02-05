/****************************************************************************
 * graphics/nxmu/nxmu_requestbkgd.c
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
#include <debug.h>

#include <nuttx/nx/nx.h>
#include "nxmu.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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
                      FAR void *arg)
{
  DEBUGASSERT(conn != NULL || be != NULL || cb != NULL);

  /* Set the client's callback vtable and and replace the server
   * connection with the clients connection.
   */

  be->bkgd.cb   = cb;
  be->bkgd.arg  = arg;
  be->bkgd.conn = conn;

  /* Report the size/position of the background window to the client */

  nxmu_reportposition((NXWINDOW)&be->bkgd);

  /* Redraw the background window */

  nxmu_redraw(&be->bkgd, &be->bkgd.bounds);

#ifdef CONFIG_NX_XYINPUT
  /* Provide the mouse settings */

  nxmu_mousereport(&be->bkgd);
#endif
}
