/****************************************************************************
 * libc/aio/aio_read.c
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
#include <sched.h>
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
 * Name: aio_read_worker
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

static void aio_read_worker(FAR void *arg)
{
  FAR struct aiocb *aiocbp = (FAR struct aiocb *)arg;
  DEBASSERT(arg);
  ssize_t nread;

  /* Perform the read using:
   *
   *   aio_fildes  - File descriptor
   *   aio_buf     - Location of buffer
   *   aio_nbytes  - Length of transfer
   *   aio_offset  - File offset
   */

 nread = pread(aiocbp->aio_fildes, aiocbp->aio_buf, aiocbp->aio_nbytes,
               aiocbp->aio_offset);

  /* Set the result of the read */

  if (nread < 0)
    {
      int errcode = get_errno();
      fdbg("ERROR: pread failed: %d\n", errode);
      DEBUGASSERT(errcode > 0);
      aicbp->result = -errcode;
    }
  else
    {
      aicbp->result = nread;
    }

  /* Signal the client */

  (void)aio_signal(aiocbp);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: aio_read
 *
 * Description:
 *   The aio_read() function reads aiocbp->aio_nbytes from the file
 *   associated with aiocbp->aio_fildes into the buffer pointed to by
 *   aiocbp->aio_buf. The function call will return when the read request
 *   has been initiated or queued to the file or device (even when the data
 *   cannot be delivered immediately).
 *
 *   The aiocbp value may be used as an argument to aio_error() and
 *   aio_return() in order to determine the error status and return status,
 *   respectively, of the asynchronous operation while it is proceeding. If
 *   an error condition is encountered during queuing, the function call
 *   will return without having initiated or queued the request. The
 *   requested operation takes place at the absolute position in the file as
 *   given by aio_offset, as if lseek() were called immediately prior to the
 *   operation with an offset equal to aio_offset and a whence equal to
 *   SEEK_SET. After a successful call to enqueue an asynchronous I/O
 *   operation, the value of the file offset for the file is unspecified.
 *
 *   The aiocbp->aio_lio_opcode field will be ignored by aio_read().
 *
 *   The aiocbp argument points to an aiocb structure. If the buffer pointed
 *   to by aiocbp->aio_buf or the control block pointed to by aiocbp becomes
 *   an illegal address prior to asynchronous I/O completion, then the
 *   behavior is undefined.
 *
 *   Simultaneous asynchronous operations using the same aiocbp produce
 *   undefined results.
 *
 *   For any system action that changes the process memory space while an
 *   synchronous I/O is outstanding to the address range being changed, the
 *   result of that action is undefined.
 *
 * Input Parameters:
 *   aiocbp - A pointer to an instance of struct aiocb
 *
 * Returned Value:
 *
 *   The aio_read() function will return the value zero if the I/O operation
 *   is successfully queued; otherwise, the function will return the value
 *   -1 and set errno to indicate the error.  The aio_read() function will
 *   ail if:
 *
 *   EAGAIN - The requested asynchronous I/O operation was not queued due to
 *     system resource limitations.
 *
 *   Each of the following conditions may be detected synchronously at the
 *   time of the call to aio_read(), or asynchronously. If any of the
 *   conditions below are detected synchronously, the aio_read() function
 *   will return -1 and set errno to the corresponding value. If any of the
 *   conditions below are detected asynchronously, the return status of the
 *   asynchronous operation is set to -1, and the error status of the
 *   asynchronous operation is set to the corresponding value.
 *
 *   EBADF - The aiocbp->aio_fildes argument is not a valid file descriptor
 *     open for reading.
 *   EINVAL - The file offset value implied by aiocbp->aio_offset would be
 *     invalid, aiocbp->aio_reqprio is not a valid value, or
 *     aiocbp->aio_nbytes is an invalid value.
 *
 *   In the case that the aio_read() successfully queues the I/O operation
 *   but the operation is subsequently cancelled or encounters an error, the
 *   return status of the asynchronous operation is one of the values
 *   normally returned by the read() function call. In addition, the error
 *   status of the asynchronous operation is set to one of the error
 *   statuses normally set by the read() function call, or one of the
 *   following values:
 *
 *   EBADF - The aiocbp->aio_fildes argument is not a valid file descriptor
 *     open for reading.
 *   ECANCELED - The requested I/O was cancelled before the I/O completed
 *     due to an explicit aio_cancel() request.
 *   EINVAL - The file offset value implied by aiocbp->aio_offset would be
 *     invalid.
 *
 * The following condition may be detected synchronously or asynchronously:
 *
 *   EOVERFLOW - The file is a regular file, aiobcp->aio_nbytes is greater
 *     than 0, and the starting offset in aiobcp->aio_offset is before the
 *     end-of-file and is at or beyond the offset maximum in the open file
 *     description associated with aiocbp->aio_fildes.
 *
 * POSIX Compliance:
 * - The POSIX specification of asynchronous I/O implies that a thread is
 *   created for each I/O operation.  The standard requires that if
 *   prioritized I/O is supported for this file, then the asynchronous
 *   operation will be submitted at a priority equal to a base scheduling
 *   priority minus aiocbp->aio_reqprio. If Thread Execution Scheduling is
 *   not supported, then the base scheduling priority is that of the calling
 *   thread.
 *
 *   My initial gut feeling is the creating a new thread on each asynchronous
 *   I/O operation would not be a good use of resources in a deeply embedded
 *   system.  So I decided to execute all asynchronous I/O on a low-priority
 *   or user-space worker thread.  There are two negative consequences of this
 *   decision that need to be revisited:
 *
 *     1) The worker thread runs at a fixed priority making it impossible to
 *        meet the POSIX requirement for asynchronous I/O.  That standard
 *        specifically requires varying priority.
 *     2) On the worker thread, each I/O will still be performed synchronously,
 *        one at a time.  This is not a violation of the POSIX requirement,
 *        but one would think there could be opportunities for concurrent I/O.
 *
 *   In reality, in a small embedded system, there will probably only be one
 *   real file system and, in this case, the I/O will be performed sequentially
 *   anyway.  Most simple embedded hardware will not support any concurrent
 *   accesses.
 *
 * - Most errors required in the standard are not detected at this point.
 *   There are no pre-queuing checks for the validity of the operation.
 *
 ****************************************************************************/

int aio_read(FAR struct aiocb *aiocbp)
{
  int ret;

  DEBUGASSERT(aiocbp);

  /* The result -EINPROGRESS means that the transfer has not yet completed */

  aiocbp->aio_result = -EINPROGRESS;

  /* Save the ID of the calling, client thread */

  aiocbp->aio_pid = getpid();

  /* Defer the work to the worker thread */

  ret = work_queue(AIO_QUEUE, &aiocbp->aio_work, aio_read_worker, aiocbp, 0);
  if (ret < 0)
    {
      aio->aio_result = ret;
      set_errno(ret);
      return ERROR;
    }

  return OK;
}

#endif /* CONFIG_LIBC_AIO */
