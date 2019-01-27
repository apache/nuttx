/****************************************************************************
 * fs/aio/aio_queue.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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

#include <sched.h>
#include <aio.h>
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
