/****************************************************************************
 * fs/aio/aio_signal.c
 *
 *   Copyright (C) 2014-2015 Gregory Nutt. All rights reserved.
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
#ifdef CONFIG_CAN_PASS_STRUCTS
  union sigval value;
#endif
  int status;
  int ret;

  DEBUGASSERT(aiocbp);

  ret = OK; /* Assume success */

  /* Signal the client */

  if (aiocbp->aio_sigevent.sigev_notify == SIGEV_SIGNAL)
    {
#ifdef CONFIG_CAN_PASS_STRUCTS
      ret = nxsig_queue(pid, aiocbp->aio_sigevent.sigev_signo,
                        aiocbp->aio_sigevent.sigev_value);
#else
      ret = nxsig_queue(pid, aiocbp->aio_sigevent.sigev_sign,
                        aiocbp->aio_sigevent.sigev_value.sival_ptr);
#endif
      if (ret < 0)
        {
          ferr("ERROR: nxsig_queue #1 failed: %d\n", ret);
        }
    }

#ifdef CONFIG_SIG_EVTHREAD
  /* Notify the client via a function call */

  else if (aiocbp->aio_sigevent.sigev_notify == SIGEV_THREAD)
    {
      ret = nxsig_notification(pid, &aiocbp->aio_sigevent);
      if (ret < 0)
        {
          ferr("ERROR: nxsig_notification failed: %d\n", ret);
        }
    }
#endif

  /* Send the poll signal in any event in case the caller is waiting
   * on sig_suspend();
   */

#ifdef CONFIG_CAN_PASS_STRUCTS
  value.sival_ptr = aiocbp;
  status = nxsig_queue(pid, SIGPOLL, value);
#else
  status = nxsig_queue(pid, SIGPOLL, aiocbp);
#endif
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
