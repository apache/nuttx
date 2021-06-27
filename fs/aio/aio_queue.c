/****************************************************************************
 * fs/aio/aio_queue.c
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

#include <sched.h>
#include <aio.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/wqueue.h>

#include "aio/aio.h"

#ifdef CONFIG_FS_AIO

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: aio_queue
 *
 * Description:
 *   Schedule the asynchronous I/O on the low priority work queue
 *
 * Input Parameters:
 *   arg - Worker argument.  In this case, a pointer to an instance of
 *     struct aiocb cast to void *.
 *
 * Returned Value:
 *   Zero (OK) on success.  Otherwise, -1 is returned and the errno is set
 *   appropriately.
 *
 ****************************************************************************/

int aio_queue(FAR struct aio_container_s *aioc, worker_t worker)
{
  int ret;

#ifdef CONFIG_PRIORITY_INHERITANCE
  /* Prohibit context switches until we complete the queuing */

  sched_lock();

  /* Make sure that the low-priority worker thread is running at at least
   * the priority specified for this action.
   */

  lpwork_boostpriority(aioc->aioc_prio);
#endif

  /* Schedule the work on the low priority worker thread */

  ret = work_queue(LPWORK, &aioc->aioc_work, worker, aioc, 0);
  if (ret < 0)
    {
      FAR struct aiocb *aiocbp = aioc->aioc_aiocbp;
      DEBUGASSERT(aiocbp);

#ifdef CONFIG_PRIORITY_INHERITANCE
      lpwork_restorepriority(aioc->aioc_prio);
#endif
      aiocbp->aio_result = ret;
      set_errno(-ret);
      ret = ERROR;
    }

#ifdef CONFIG_PRIORITY_INHERITANCE
  /* Now the low-priority work queue might run at its new priority */

  sched_unlock();
#endif
  return ret;
}

#endif /* CONFIG_FS_AIO */
