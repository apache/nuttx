/****************************************************************************
 * fs/aio/aio_write.c
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

#include <stdarg.h>
#include <unistd.h>
#include <fcntl.h>
#include <sched.h>
#include <aio.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs/fs.h>

#include "aio/aio.h"

#ifdef CONFIG_FS_AIO

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: aio_write_worker
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

static void aio_write_worker(FAR void *arg)
{
  FAR struct aio_container_s *aioc = (FAR struct aio_container_s *)arg;
  FAR struct aiocb *aiocbp;
  pid_t pid;
#ifdef CONFIG_PRIORITY_INHERITANCE
  uint8_t prio;
#endif
  ssize_t nwritten = 0;
  int oflags;

  /* Get the information from the container, decant the AIO control block,
   * and free the container before starting any I/O.  That will minimize
   * the delays by any other threads waiting for a pre-allocated container.
   */

  DEBUGASSERT(aioc && aioc->aioc_aiocbp);
  pid    = aioc->aioc_pid;
#ifdef CONFIG_PRIORITY_INHERITANCE
  prio   = aioc->aioc_prio;
#endif
  aiocbp = aioc_decant(aioc);

  /* Call fcntl(F_GETFL) to get the file open mode. */

  oflags = file_fcntl(aioc->aioc_filep, F_GETFL);
  if (oflags < 0)
    {
      ferr("ERROR: file_fcntl failed: %d\n", oflags);
      aiocbp->aio_result = oflags;
      goto errout;
    }

  /* Perform the write using:
   *
   *   aioc_filep   - File structure pointer
   *   aio_buf      - Location of buffer
   *   aio_nbytes   - Length of transfer
   *   aio_offset   - File offset
   */

  /* Check if O_APPEND is set in the file open flags */

  if ((oflags & O_APPEND) != 0)
    {
      /* Append to the current file position */

      nwritten = file_write(aioc->aioc_filep,
                            (FAR const void *)aiocbp->aio_buf,
                            aiocbp->aio_nbytes);
    }
  else
    {
      nwritten = file_pwrite(aioc->aioc_filep,
                             (FAR const void *)aiocbp->aio_buf,
                             aiocbp->aio_nbytes,
                             aiocbp->aio_offset);
    }

  if (nwritten < 0)
    {
      ferr("ERROR: write/pwrite/send failed: %zd\n", nwritten);
    }

  /* Save the result of the write */

  aiocbp->aio_result = nwritten;

errout:

  /* Signal the client */

  aio_signal(pid, aiocbp);

#ifdef CONFIG_PRIORITY_INHERITANCE
  /* Restore the low priority worker thread default priority */

  lpwork_restorepriority(prio);
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: aio_write
 *
 * Description:
 *   The aio_write() function writes aiocbp->aio_nbytes to the file
 *   associated with aiocbp->aio_fildes from the buffer pointed to by
 *   aiocbp->aio_buf. The function call will return when the write request
 *   has been initiated or queued to the file or device (even when the data
 *   cannot be delivered immediately).
 *
 *   If prioritized I/O is supported for this file, then the asynchronous
 *   operation will be submitted at a priority equal to a base scheduling
 *   priority minus aiocbp->aio_reqprio. If Thread Execution Scheduling is
 *   not supported, then the base scheduling priority is that of the calling
 *   thread (the latter is implemented at present).
 *
 *   The aiocbp value may be used as an argument to aio_error() and
 *   aio_return() in order to determine the error status and return status,
 *   respectively, of the asynchronous operation while it is proceeding.
 *
 *   The aiocbp argument points to an aiocb structure. If the buffer pointed
 *   to by aiocbp->aio_buf or the control block pointed to by aiocbp becomes
 *   an illegal address prior to asynchronous I/O completion, then the
 *   behavior is undefined.
 *
 *   If O_APPEND is not set for the file descriptor aio_fildes, then the
 *   requested operation will take place at the absolute position in the file
 *   as given by aio_offset, as if lseek() were called immediately prior to
 *   the operation with an offset equal to aio_offset and a whence equal to
 *   SEEK_SET. If O_APPEND is set for the file descriptor, write operations
 *   append to the file in the same order as the calls were made. After a
 *   successful call to enqueue an asynchronous I/O operation, the value of
 *   the file offset for the file is unspecified.
 *
 *   The aiocbp->aio_lio_opcode field will be ignored by aio_write().
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
 *   The aio_write() function will return the value zero if the I/O operation
 *   is successfully queued; otherwise, the function will return the value
 *   -1 and set errno to indicate the error.  The aio_write() function will
 *   ail if:
 *
 *   EAGAIN - The requested asynchronous I/O operation was not queued due to
 *     system resource limitations.
 *
 *   Each of the following conditions may be detected synchronously at the
 *   time of the call to aio_write(), or asynchronously. If any of the
 *   conditions below are detected synchronously, the aio_write() function
 *   will return -1 and set errno to the corresponding value. If any of the
 *   conditions below are detected asynchronously, the return status of the
 *   asynchronous operation is set to -1, and the error status of the
 *   asynchronous operation is set to the corresponding value.
 *
 *   EBADF - The aiocbp->aio_fildes argument is not a valid file descriptor
 *     open for writing.
 *   EINVAL - The file offset value implied by aiocbp->aio_offset would be
 *     invalid, aiocbp->aio_reqprio is not a valid value, or
 *     aiocbp->aio_nbytes is an invalid value.
 *
 *   In the case that the aio_write() successfully queues the I/O operation
 *   but the operation is subsequently canceled or encounters an error, the
 *   return status of the asynchronous operation is one of the values
 *   normally returned by the write() function call. In addition, the error
 *   status of the asynchronous operation is set to one of the error
 *   statuses normally set by the write() function call, or one of the
 *   following values:
 *
 *   EBADF - The aiocbp->aio_fildes argument is not a valid file descriptor
 *     open for writing.
 *   EINVAL - The file offset value implied by aiocbp->aio_offset would be
 *     invalid.
 *   ECANCELED - The requested I/O was canceled before the I/O completed
 *     due to an explicit aio_cancel() request.
 *
 * The following condition may be detected synchronously or asynchronously:
 *
 *   EFBIG - The file is a regular file, aiobcp->aio_nbytes is greater
 *     than 0, and the starting offset in aiobcp->aio_offset is at or
 *     beyond the offset maximum in the open file description associated
 *     with aiocbp->aio_fildes.
 *
 * POSIX Compliance:
 * - Most errors required in the standard are not detected at this point.
 *   There are no pre-queuing checks for the validity of the operation.
 *
 ****************************************************************************/

int aio_write(FAR struct aiocb *aiocbp)
{
  FAR struct aio_container_s *aioc;
  int ret;

  DEBUGASSERT(aiocbp);

  /* The result -EINPROGRESS means that the transfer has not yet completed */

  sigwork_init(&aiocbp->aio_sigwork);
  aiocbp->aio_result = -EINPROGRESS;
  aiocbp->aio_priv   = NULL;

  /* Create a container for the AIO control block.  This may cause us to
   * block if there are insufficient resources to satisfy the request.
   */

  aioc = aio_contain(aiocbp);
  if (!aioc)
    {
      /* The errno has already been set (probably EBADF) */

      aiocbp->aio_result = -get_errno();
      return ERROR;
    }

  /* Defer the work to the worker thread */

  ret = aio_queue(aioc, aio_write_worker);
  if (ret < 0)
    {
      /* The result and the errno have already been set */

      aioc_decant(aioc);
      return ERROR;
    }

  return OK;
}

#endif /* CONFIG_FS_AIO */
