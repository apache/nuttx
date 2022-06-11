/****************************************************************************
 * net/netdev/netdown_notifier.c
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
 *   None.
 *
 ****************************************************************************/

void netdown_notifier_teardown(int key)
{
  /* This is just a simple wrapper around work_notifier_teardown(). */

  work_notifier_teardown(key);
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
