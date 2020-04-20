/****************************************************************************
 * fs/aio/aio_signal.c
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
#include <sched.h>
#include <signal.h>
#include <aio.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/signal.h>

#include "aio/aio.h"

#ifdef CONFIG_FS_AIO

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: aio_signal
 *
 * Description:
 *   Signal the client that an I/O has completed.
 *
 * Input Parameters:
 *   pid    - ID of the task to signal
 *   aiocbp - Pointer to the asynchronous I/O state structure that includes
 *            information about how to signal the client
 *
 * Returned Value:
 *   Zero (OK) if the client was successfully signalled.  Otherwise, a
 *   negated errno value is returned.
 *
 * Assumptions:
 *   This function runs only in the context of the worker thread.
 *
 ****************************************************************************/

int aio_signal(pid_t pid, FAR struct aiocb *aiocbp)
{
  union sigval value;
  int status;
  int ret;

  DEBUGASSERT(aiocbp);

  ret = OK; /* Assume success */

  /* Signal the client */

  ret = nxsig_notification(pid, &aiocbp->aio_sigevent,
                           SI_ASYNCIO, &aiocbp->aio_sigwork);
  if (ret < 0)
    {
      ferr("ERROR: nxsig_notification failed: %d\n", ret);
    }

  /* Send the poll signal in any event in case the caller is waiting
   * on sig_suspend();
   */

  value.sival_ptr = aiocbp;
  status = nxsig_queue(pid, SIGPOLL, value);
  if (status < 0)
    {
      ferr("ERROR: nxsig_queue #2 failed: %d\n", status);
      if (ret >= OK)
        {
          ret = status;
        }
    }

  /* Make sure that errno is set correctly on return */

  if (ret < 0)
    {
      set_errno(-ret);
      return ERROR;
    }

  return OK;
}

#endif /* CONFIG_FS_AIO */
