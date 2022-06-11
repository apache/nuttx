/****************************************************************************
 * net/netlink/netlink_notifier.c
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

#include <sys/types.h>
#include <assert.h>

#include <nuttx/wqueue.h>

#include "netlink/netlink.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netlink_notifier_setup
 *
 * Description:
 *   Set up to perform a callback to the worker function the Netlink
 *   response data is received.  The worker function will execute on the low
 *   priority worker thread.
 *
 * Input Parameters:
 *   worker - The worker function to execute on the low priority work
 *            queue when Netlink response data is available.
 *   conn   - The Netlink connection where the response is expected.
 *   arg    - A user-defined argument that will be available to the worker
 *            function when it runs.
 *
 * Returned Value:
 *   Zero (OK) is returned if the notification was successfully set up.
 *   A negated error value is returned if an unexpected error occurred
 *   and no notification will occur.
 *
 ****************************************************************************/

int netlink_notifier_setup(worker_t worker, FAR struct netlink_conn_s *conn,
                           FAR void *arg)
{
  struct work_notifier_s info;

  DEBUGASSERT(worker != NULL && conn != NULL);

  if (conn->key > 0)
    {
      return -EBUSY;
    }

  /* This is just a simple wrapper around work_notifer_setup(). */

  info.evtype    = WORK_NETLINK_RESPONSE;
  info.qid       = LPWORK;
  info.qualifier = conn;
  info.arg       = arg;
  info.worker    = worker;

  conn->key      =  work_notifier_setup(&info);
  return conn->key;
}

/****************************************************************************
 * Name: netlink_notifier_teardown
 *
 * Description:
 *   Eliminate a Netlink response notification previously setup by
 *   netlink_notifier_setup().  This function should only be called if the
 *   notification should be aborted prior to the notification.  The
 *   notification will automatically be torn down after the notification.
 *
 * Input Parameters:
 *   conn - Teardown the notification for this Netlink connection.
 *
 ****************************************************************************/

void netlink_notifier_teardown(FAR struct netlink_conn_s *conn)
{
  DEBUGASSERT(conn != NULL);

  /* This is just a simple wrapper around work_notifier_teardown(). */

  if (conn->key > 0)
    {
      work_notifier_teardown(conn->key);
      conn->key = 0;
    }
}

/****************************************************************************
 * Name: netlink_notifier_signal
 *
 * Description:
 *   New Netlink response data is available.  Execute worker thread
 *   functions for all threads that wait for response data.
 *
 * Input Parameters:
 *   conn - The Netlink connection where the response was just buffered.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void netlink_notifier_signal(FAR struct netlink_conn_s *conn)
{
  /* This is just a simple wrapper around work_notifier_signal(). */

  work_notifier_signal(WORK_NETLINK_RESPONSE, conn);
}
