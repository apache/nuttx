/****************************************************************************
 * fs/aio/aio_cancel.c
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

#include <aio.h>
#include <sched.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/wqueue.h>

#include "aio/aio.h"

#ifdef CONFIG_FS_AIO

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: aio_read
 *
 * Description:
 *   The aio_cancel() function attempts to cancel one or more asynchronous
 *   I/O requests currently outstanding against file descriptor 'fildes'.
 *   The aiocbp argument points to the asynchronous I/O control block for
 *   a particular request to be cancelled..
 *
 *   Normal asynchronous notification will occur for asynchronous I/O
 *   operations that are successfully cancelled. If there are requests that
 *   cannot be cancelled, then the normal asynchronous completion process
 *   will take place for those requests when they are completed.
 *
 *   For requested operations that are successfully cancelled, the associated
 *   error status will be set to ECANCELED and the return status will be -1.
 *   For requested operations that are not successfully cancelled, the aiocbp
 *   will not be modified by aio_cancel().
 *
 * Input Parameters:
 *   fildes - Not used in this implementation
 *   aiocbp - Points to the asynchronous I/O control block for a particular
 *            request to be cancelled.
 *
 * Returned Value:
 *    The aio_cancel() function will return the value AIO_CANCELED if the
 *    requested operation(s) were cancelled. The value AIO_NOTCANCELED will
 *    be returned if at least one of the requested operation(s) cannot be
 *    cancelled because it is in progress. In this case, the state of the
 *    other operations, if any, referenced in the call to aio_cancel() is
 *    not indicated by the return value of aio_cancel(). The application
 *    may determine the state of affairs for these operations by using
 *    aio_error(). The value AIO_ALLDONE is returned if all of the
 *    operations have already completed. Otherwise, the function will return
 *    -1 and set errno to indicate the error.
 *
 * POSIX Compliance
 *   By standard, this function support the value of NULL for aiocbp.  If
 *   aiocbp is NULL, then all outstanding cancelable asynchronous I/O
 *   requests against fildes will be cancelled.  In this implementation,
 *   nothing is done if aiocbp is NULL and the fildes parameter is always
 *   ignored.  ENOSYS will be returned.
 *
 ****************************************************************************/

int aio_cancel(int fildes, FAR struct aiocb *aiocbp)
{
  FAR struct aio_container_s *aioc;
  int status;
  int ret;

  /* Ignore NULL AIO control blocks (see "POSIX Compliance" above) */

  if (aiocbp)
    {
      set_errno(ENOSYS);
      return ERROR;
    }

  /* Lock the scheduler so that no I/O events can complete on the worker
   * thread until we set complete this operation.
   */

  sched_lock();

  /* Check if the I/O has completed */

  if (aiocbp->aio_result == -EINPROGRESS)
    {
      /* No.. Find the container for this AIO control block */

      aio_lock();
      for (aioc = (FAR struct aio_container_s *)g_aio_pending.head;
           aioc && aioc->aioc_aiocbp != aiocbp;
           aioc = (FAR struct aio_container_s *)aioc->aioc_link.flink);
      aio_unlock();

      /* Did we find the container? */

      if (!aioc)
        {
          /* No.. the aio_result says that that the transfer is pending,
           * but there is no container in the pending I/O list.  I am
           * confused.
           */

          ret = AIO_ALLDONE;
        }
      else
        {
          /* Yes... attempt to cancel the I/O.  There are two possibilities:
           * (1) the work has already been started and is no longer queued,
           * or (2) the work has not been started and is still in the work
           * queue.  Only the second case can be cancelled.  work_cancel()
           * will return -ENOENT in the first case.
           */

          status = work_cancel(LPWORK, &aioc->aioc_work);
          ret = status >= 0 ? AIO_CANCELED : AIO_NOTCANCELED;
        }
    }
  else
    {
      /* The I/O has already completed */

      ret = AIO_ALLDONE;
    }

  sched_unlock();
  return ret;
}

#endif /* CONFIG_FS_AIO */
