/****************************************************************************
 * libs/libnx/nxmu/nx_eventnotify.c
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

#include <stdlib.h>
#include <mqueue.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxmu.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nx_eventnotify
 *
 * Description:
 *   Rather than calling nx_eventhandler periodically, the client may
 *   register to receive a signal when a server event is available.  The
 *   client can then call nv_eventhandler() only when incoming events are
 *   available.
 *
 *   Only one such event is issued.  Upon receipt of the signal, if the
 *   client wishes further notifications, it must call nx_eventnotify again.
 *
 * Input Parameters:
 *   handle - the handle returned by nx_connect
 *
 * Returned Value:
 *      OK: No errors occurred.  If CONFIG_NX_BLOCKING is defined, then
 *          one or more server message was processed.
 *   ERROR: An error occurred and errno has been set appropriately
 *
 ****************************************************************************/

int nx_eventnotify(NXHANDLE handle, int signo)
{
  FAR struct nxmu_conn_s *conn = (FAR struct nxmu_conn_s *)handle;
  struct sigevent se;

  se.sigev_notify          = SIGEV_SIGNAL;
  se.sigev_signo           = signo;
  se.sigev_value.sival_ptr = (FAR void *)handle;

  return mq_notify(conn->crdmq, &se);
}
