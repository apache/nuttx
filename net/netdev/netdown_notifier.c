/****************************************************************************
 * net/netdev/netdown_notifier.c
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
#include <nuttx/mm/iob.h>
#include <nuttx/net/netdev.h>

#include "netdev/netdev.h"

#ifdef CONFIG_NETDOWN_NOTIFIER

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netdown_notifier_setup
 *
 * Description:
 *   Set up to perform a callback to the worker function when the network
 *   goes down.  The worker function will execute on the high priority
 *   worker thread.
 *
 * Input Parameters:
 *   worker - The worker function to execute on the low priority work
 *            queue when data is available in the UDP read-ahead buffer.
 *   dev   - The network driver to be monitored
 *   arg    - A user-defined argument that will be available to the worker
 *            function when it runs.
 * Returned Value:
 *   > 0   - The notification is in place.  The returned value is a key that
 *           may be used later in a call to netdown_notifier_teardown().
 *   == 0  - The the device is already down.  No notification will be
 *           provided.
 *   < 0   - An unexpected error occurred and notification will occur.  The
 *           returned value is a negated errno value that indicates the
 *           nature of the failure.
 *
 ****************************************************************************/

int netdown_notifier_setup(worker_t worker, FAR struct net_driver_s *dev,
                           FAR void *arg)
{
  struct work_notifier_s info;

  DEBUGASSERT(worker != NULL);

  /* If network driver is already down, then return zero without setting up
   * the notification.
   */

  if ((dev->d_flags & IFF_UP) == 0)
    {
      return 0;
    }

  /* Otherwise, this is just a simple wrapper around work_notifer_setup(). */

  info.evtype    = WORK_NET_DOWN;
  info.qid       = LPWORK;
  info.qualifier = dev;
  info.arg       = arg;
  info.worker    = worker;

  return work_notifier_setup(&info);
}

/****************************************************************************
 * Name: netdown_notifier_teardown
 *
 * Description:
 *   Eliminate a network down notification previously setup by
 *   netdown_notifier_setup().  This function should only be called if the
 *   notification should be aborted prior to the notification.  The
 *   notification will automatically be torn down after the notification.
 *
 * Input Parameters:
 *   key - The key value returned from a previous call to
 *         netdown_notifier_setup().
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int netdown_notifier_teardown(int key)
{
  /* This is just a simple wrapper around work_notifier_teardown(). */

  return work_notifier_teardown(key);
}

/****************************************************************************
 * Name: netdown_notifier_signal
 *
 * Description:
 *   A network has gone down has been buffered.  Execute worker thread
 *   functions for all threads monitoring the state of the device.
 *
 * Input Parameters:
 *   dev  - The TCP connection where read-ahead data was just buffered.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void netdown_notifier_signal(FAR struct net_driver_s *dev)
{
  /* This is just a simple wrapper around work_notifier_signal(). */

  work_notifier_signal(WORK_NET_DOWN, dev);
}

#endif /* CONFIG_NETDOWN_NOTIFIER */
