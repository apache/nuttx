/****************************************************************************
 * net/netlink/netlink_notifier.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int netlink_notifier_teardown(FAR struct netlink_conn_s *conn)
{
  DEBUGASSERT(conn != NULL);
  int ret = OK;

  /* This is just a simple wrapper around work_notifier_teardown(). */

  if (conn->key > 0)
    {
      ret = work_notifier_teardown(conn->key);
      conn->key = 0;
    }

  return ret;
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
