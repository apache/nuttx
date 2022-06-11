/****************************************************************************
 * mm/iob/iob_notifier.c
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

#include "iob.h"

#ifdef CONFIG_IOB_NOTIFIER

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: iob_notifier_setup
 *
 * Description:
 *   Set up to perform a callback to the worker function when an IOB is
 *   available.  The worker function will execute on the selected priority
 *   worker thread.
 *
 * Input Parameters:
 *   qid    - Selects work queue.  Must be HPWORK or LPWORK.
 *   worker - The worker function to execute on the high priority work queue
 *            when the event occurs.
 *   arg    - A user-defined argument that will be available to the worker
 *            function when it runs.
 *
 * Returned Value:
 *   > 0   - The notification is in place.  The returned value is a key that
 *           may be used later in a call to iob_notifier_teardown().
 *   == 0  - There are already free IOBs.  No notification will be provided.
 *   < 0   - An unexpected error occurred and notification will occur.  The
 *           returned value is a negated errno value that indicates the
 *           nature of the failure.
 *
 ****************************************************************************/

int iob_notifier_setup(int qid, worker_t worker, FAR void *arg)
{
  struct work_notifier_s info;

  DEBUGASSERT(worker != NULL);

  info.evtype    = WORK_IOB_AVAIL;
  info.qid       = qid;
  info.qualifier = NULL;
  info.arg       = arg;
  info.worker    = worker;

  return work_notifier_setup(&info);
}

/****************************************************************************
 * Name: iob_notifier_teardown
 *
 * Description:
 *   Eliminate an IOB notification previously setup by iob_notifier_setup().
 *   This function should only be called if the notification should be
 *   aborted prior to the notification.  The notification will automatically
 *   be torn down after the notification.
 *
 * Input Parameters:
 *   key - The key value returned from a previous call to
 *         iob_notifier_setup().
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void iob_notifier_teardown(int key)
{
  /* This is just a simple wrapper around work_notifier_teardown(). */

  work_notifier_teardown(key);
}

/****************************************************************************
 * Name: iob_notifier_signal
 *
 * Description:
 *   An IOB has become available.  Signal all threads waiting for an IOB
 *   that an IOB is available.
 *
 *   When an IOB becomes available, *all* of the workers waiting for an
 *   IOB will be executed.  If there are multiple workers waiting for an IOB
 *   then only the first to execute will get the IOB.  Others will
 *   need to call iob_notify_setup() once again.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void iob_notifier_signal(void)
{
  /* This is just a simple wrapper around work_notifier_signal(). */

  work_notifier_signal(WORK_IOB_AVAIL, NULL);
}

#endif /* CONFIG_IOB_NOTIFIER */
