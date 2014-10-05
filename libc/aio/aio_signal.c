/****************************************************************************
 * libc/aio/aio_signal.c
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

#include <signal.h>
#include <aio.h>
#include <assert.h>

#include "aio/aio.h"

#ifndef CONFIG_LIBC_AIO

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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

int aio_signal(FAR struct aiocb *aiocbp)
{
  int ret;

  DEBUGASSERT(aiocbp);
 
  /* Signal the client */

  if (aiocbp->aio_sigevent.sigev_notify == SIGEV_SIGNAL)
    {
#ifdef CONFIG_CAN_PASS_STRUCTS
      ret = sigqueue(aiocbp->aio_pid, aiocbp->aio_sigevent.sigev_signo,
                     aiocbp->aio_sigevent.sigev_value);
#else
      ret = sigqueue(aiocbp->aio_pid, aiocbp->aio_sigevent.sigev_sign,
                     aiocbp->aio_sigevent.sigev_value.sival_ptr);
#endif
    }

  /* Send the poll signal in any event in case the caller is waiting
   * on sig_suspend();
   */

  else
    {
      ret = kill(aiocbp->aio_pid, SIGPOLL);
    }

  return ret;
}

#endif /* CONFIG_LIBC_AIO */
