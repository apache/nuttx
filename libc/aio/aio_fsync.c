/****************************************************************************
 * libc/aio/aio_fsync.c
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

#include <unistd.h>
#include <aio.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/wqueue.h>

#include "lib_internal.h"
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
 * Name: aio_fsync_worker
 *
 * Description:
 *   This function executes on the worker thread and performs the
 *   asynchronous I/O operation.
 *
 * Input Parameters:
 *   arg - Worker argument.  In this case, a pointer to an instance of
 *     struct aiocb cast to void *.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void aio_fsync_worker(FAR void *arg)
{
  FAR struct aiocb *aiocbp = (FAR struct aiocb *)arg;
  DEBASSERT(arg);
  int ret;

  /* Perform the fsync using aio_fildes */

  ret = fsync(aiocbp->aio_fildes);
  if (ret < 0)
    {
      int errcode = get_errno();
      fdbg("ERROR: fsync failed: %d\n", errode);
      DEBUGASSERT(errcode > 0);
      aicbp->result = -errcode;
    }
  else
    {
      aicbp->result = OK;
    }

  /* Signal the client */

  (void)aio_signal(aiocbp);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: aio_fsync
 *
 * Description:
 *   The aio_fsync() function asynchronously forces all I/O operations
 *   associated with the file indicated by the file descriptor aio_fildes
 *   member of the aiocb structure referenced by the aiocbp argument and
 *   queued at the time of the call to aio_fsync() to the synchronized
 *   I/O completion state. The function call will return when the
 *   synchronization request has been initiated or queued to the file or
 *   device (even when the data cannot be synchronized immediately).
 *
 *   If op is O_DSYNC, all currently queued I/O operations will be
 *   completed as if by a call to fdatasync(); that is, as defined for
 *   synchronized I/O data integrity completion. If op is O_SYNC, all
 *   currently queued I/O operations will be completed as if by a call to
 *   fsync(); that is, as defined for synchronized I/O file integrity
 *   completion. If the aio_fsync() function fails, or if the operation
 *   queued by aio_fsync() fails, then, as for fsync() and fdatasync(),
 *   outstanding I/O operations are not guaranteed to have been completed.
 *   [See "POSIX Compliance" below]
 *
 *   If aio_fsync() succeeds, then it is only the I/O that was queued at
 *   the time of the call to aio_fsync() that is guaranteed to be forced
 *   to the relevant completion state. The completion of subsequent I/O
 *   on the file descriptor is not guaranteed to be completed in a
 *   synchronized fashion.
 *
 *   The aiocbp argument refers to an asynchronous I/O control block. The
 *   aiocbp value may be used as an argument to aio_error() and aio_return()
 *   in order to determine the error status and return status, respectively,
 *   of the asynchronous operation while it is proceeding. When the request
 *   is queued, the error status for the operation is [EINPROGRESS]. When
 *   all data has been successfully transferred, the error status will be
 *   reset to reflect the success or failure of the operation. If the
 *   operation does not complete successfully, the error status for the
 *   operation will be set to indicate the error. The aio_sigevent member
 *   determines the asynchronous notification to occur when all operations
 *   have achieved synchronized I/O completion. All other members of the
 *   structure referenced by aiocbp are ignored. If the control block
 *   referenced by aiocbp becomes an illegal address prior to asynchronous
 *   I/O completion, then the behavior is undefined.
 *
 *   If the aio_fsync() function fails or aiocbp indicates an error condition,
 *   data is not guaranteed to have been successfully transferred.
 *
 * Input Parameters:
 *   op     - Should be either O_SYNC or O_DSYNC.  Ignored in this implementation.
 *   aiocbp - A pointer to an instance of struct aiocb
 *
 * Returned Value:
 *   The aio_fsync() function will return the value 0 if the I/O operation is 
 *   successfully queued; otherwise, the function will return the value -1 and
 *   set errno to indicate the error.
 *
 *   The aio_fsync() function will fail if:
 *
 *     EAGAIN - The requested asynchronous operation was not queued due to
 *       temporary resource limitations.
 *     EBADF - The aio_fildes member of the aiocb structure referenced by
 *       the aiocbp argument is not a valid file descriptor open for writing.
 *     EINVAL - This implementation does not support synchronized I/O for
 *       this file.
 *      EINVAL - A value of op other than O_DSYNC or O_SYNC was specified.
 *
 *   In the event that any of the queued I/O operations fail, aio_fsync()
 *   will return the error condition defined for read() and write(). The
 *   error is returned in the error status for the asynchronous fsync()
 *   operation, which can be retrieved using aio_error().
 *
 * POSIX Compliance
 * - NuttX does not currently make any distinction between O_DYSNC and O_SYNC.
 *   Hence, the 'op' argument is ignored altogether.
 * - Most errors required in the standard are not detected at this point.
 *   There are no pre-queuing checks for the validity of the operation.
 *
 ****************************************************************************/

int aio_fsync(int op, FAR struct aiocb *aiocbp)
{
  int ret;

  DEBUGASSERT(op == O_SYNC /* || op == O_DSYNC */);
  DEBUGASSERT(aiocbp);

  /* The result -EINPROGRESS means that the transfer has not yet completed */

  aiocbp->aio_result = -EINPROGRESS;

  /* Save the ID of the calling, client thread */

  aiocbp->aio_pid = getpid();

  /* Defer the work to the worker thread */

  ret = work_queue(AIO_QUEUE, &aiocbp->aio_work, aio_fsync_worker, aiocbp, 0);
  if (ret < 0)
    {
      aio->aio_result = ret;
      set_errno(ret);
      return ERROR;
    }

  return OK;
}

#endif /* CONFIG_LIBC_AIO */
