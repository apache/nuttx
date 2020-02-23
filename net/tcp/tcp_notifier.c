/****************************************************************************
 * net/tcp/tcp_notifier.c
 *
 *   Copyright (C) 2018-2019 Gregory Nutt. All rights reserved.
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
#include <debug.h>

#include <nuttx/wqueue.h>
#include <nuttx/mm/iob.h>
#include <nuttx/net/tcp.h>

#include "tcp/tcp.h"

#ifdef CONFIG_NET_TCP_NOTIFIER

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tcp_readahead_notifier_setup
 *
 * Description:
 *   Set up to perform a callback to the worker function when an TCP data
 *   is added to the read-ahead buffer.  The worker function will execute
 *   on the low priority worker thread.
 *
 * Input Parameters:
 *   worker - The worker function to execute on the low priority work
 *            queue when data is available in the TCP read-ahead buffer.
 *   conn  - The TCP connection where read-ahead data is needed.
 *   arg    - A user-defined argument that will be available to the worker
 *            function when it runs.
 *
 * Returned Value:
 *   > 0   - The notification is in place.  The returned value is a key that
 *           may be used later in a call to tcp_notifier_teardown().
 *   == 0  - There is already buffered read-ahead data.  No notification
 *           will be provided.
 *   < 0   - An unexpected error occurred and notification will occur.  The
 *           returned value is a negated errno value that indicates the
 *           nature of the failure.
 *
 ****************************************************************************/

int tcp_readahead_notifier_setup(worker_t worker,
                                 FAR struct tcp_conn_s *conn,
                                 FAR void *arg)
{
  struct work_notifier_s info;

  DEBUGASSERT(worker != NULL);

  /* If there is already buffered read-ahead data, then return zero without
   * setting up the notification.
   */

  if (conn->readahead.qh_head != NULL)
    {
      return 0;
    }

  /* Otherwise, this is just a simple wrapper around work_notifer_setup(). */

  info.evtype    = WORK_TCP_READAHEAD;
  info.qid       = LPWORK;
  info.qualifier = conn;
  info.arg       = arg;
  info.worker    = worker;

  return work_notifier_setup(&info);
}

/****************************************************************************
 * Name: tcp_writebuffer_notifier_setup
 *
 * Description:
 *   Set up to perform a callback to the worker function when an TCP write
 *   buffer is emptied.  The worker function will execute on the high
 *   priority worker thread.
 *
 * Input Parameters:
 *   worker - The worker function to execute on the high priority work
 *            queue when all buffer TX data has been sent.
 *   conn   - The TCP connection where buffer write data is pending.
 *   arg    - A user-defined argument that will be available to the worker
 *            function when it runs.
 *
 * Returned Value:
 *   > 0   - The signal notification is in place.  The returned value is a
 *           key that may be used later in a call to
 *           tcp_notifier_teardown().
 *   == 0  - There is already buffered read-ahead data.  No signal
 *           notification will be provided.
 *   < 0   - An unexpected error occurred and no signal will be sent.  The
 *           returned value is a negated errno value that indicates the
 *           nature of the failure.
 *
 ****************************************************************************/

int tcp_writebuffer_notifier_setup(worker_t worker,
                                   FAR struct tcp_conn_s *conn,
                                   FAR void *arg)
{
#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
  struct work_notifier_s info;

  DEBUGASSERT(worker != NULL);

  /* If the write buffers are already empty, then return zero without
   * setting up the notification.
   */

  if (sq_empty(&conn->write_q) && sq_empty(&conn->unacked_q))
    {
      return 0;
    }

  /* Otherwise, this is just a simple wrapper around work_notifer_setup(). */

  info.evtype    = WORK_TCP_WRITEBUFFER;
  info.qid       = LPWORK;
  info.qualifier = conn;
  info.arg       = arg;
  info.worker    = worker;

  return work_notifier_setup(&info);
#else
  return 0;
#endif
}

/****************************************************************************
 * Name: tcp_disconnect_notifier_setup
 *
 * Description:
 *   Set up to perform a callback to the worker function if the TCP
 *   connection is lost.
 *
 * Input Parameters:
 *   worker - The worker function to execute on the low priority work
 *            queue when data is available in the TCP read-ahead buffer.
 *   conn  - The TCP connection where read-ahead data is needed.
 *   arg    - A user-defined argument that will be available to the worker
 *            function when it runs.
 *
 * Returned Value:
 *   > 0   - The notification is in place.  The returned value is a key that
 *           may be used later in a call to tcp_notifier_teardown().
 *   == 0  - No connection has been established.
 *   < 0   - An unexpected error occurred and notification will occur.  The
 *           returned value is a negated errno value that indicates the
 *           nature of the failure.
 *
 ****************************************************************************/

int tcp_disconnect_notifier_setup(worker_t worker,
                                  FAR struct tcp_conn_s *conn,
                                  FAR void *arg)
{
  struct work_notifier_s info;

  DEBUGASSERT(worker != NULL);

  /* If connection has not been established, then return 0. */

  if (conn->tcpstateflags != TCP_ESTABLISHED)
    {
      return 0;
    }

  /* Otherwise, this is just a simple wrapper around work_notifer_setup(). */

  info.evtype    = WORK_TCP_DISCONNECT;
  info.qid       = LPWORK;
  info.qualifier = conn;
  info.arg       = arg;
  info.worker    = worker;

  return work_notifier_setup(&info);
}

/****************************************************************************
 * Name: tcp_notifier_teardown
 *
 * Description:
 *   Eliminate a TCP read-ahead notification previously setup by
 *   tcp_readahead_notifier_setup().  This function should only be called
 *   if the notification should be aborted prior to the notification.  The
 *   notification will automatically be torn down after the notification.
 *
 * Input Parameters:
 *   key - The key value returned from a previous call to
 *         tcp_readahead_notifier_setup().
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int tcp_notifier_teardown(int key)
{
  /* This is just a simple wrapper around work_notifier_teardown(). */

  return work_notifier_teardown(key);
}

/****************************************************************************
 * Name: tcp_readahead_signal
 *
 * Description:
 *   Read-ahead data has been buffered.  Signal all threads waiting for
 *   read-ahead data to become available.
 *
 *   When read-ahead data becomes available, *all* of the workers waiting
 *   for read-ahead data will be executed.  If there are multiple workers
 *   waiting for read-ahead data then only the first to execute will get the
 *   data.  Others will need to call tcp_readahead_notifier_setup() once
 *   again.
 *
 * Input Parameters:
 *   conn  - The TCP connection where read-ahead data was just buffered.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void tcp_readahead_signal(FAR struct tcp_conn_s *conn)
{
  /* This is just a simple wrapper around work_notifier_signal(). */

  work_notifier_signal(WORK_TCP_READAHEAD, conn);
}

/****************************************************************************
 * Name: tcp_writebuffer_signal
 *
 * Description:
 *   All buffer Tx data has been sent.  Signal all threads waiting for the
 *   write buffers to become empty.
 *
 *   When write buffer becomes empty, *all* of the workers waiting
 *   for that event data will be executed.  If there are multiple workers
 *   waiting for read-ahead data then only the first to execute will get the
 *   data.  Others will need to call tcp_writebuffer_notifier_setup() once
 *   again.
 *
 * Input Parameters:
 *   conn  - The TCP connection where read-ahead data was just buffered.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
void tcp_writebuffer_signal(FAR struct tcp_conn_s *conn)
{
  /* This is just a simple wrapper around work_notifier_signal(). */

  work_notifier_signal(WORK_TCP_WRITEBUFFER, conn);
}
#endif

/****************************************************************************
 * Name: tcp_disconnect_signal
 *
 * Description:
 *   The TCP connection has been lost.  Signal all threads monitoring TCP
 *   state events.
 *
 * Input Parameters:
 *   conn  - The TCP connection where read-ahead data was just buffered.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void tcp_disconnect_signal(FAR struct tcp_conn_s *conn)
{
  /* This is just a simple wrapper around work_notifier_signal(). */

  work_notifier_signal(WORK_TCP_DISCONNECT, conn);
}

#endif /* CONFIG_NET_TCP_NOTIFIER */
