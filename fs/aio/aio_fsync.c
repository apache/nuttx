/****************************************************************************
 * fs/aio/aio_fsync.c
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

#include <unistd.h>
#include <fcntl.h>
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
  FAR struct aio_container_s *aioc = (FAR struct aio_container_s *)arg;
  FAR struct aiocb *aiocbp;
  pid_t pid;
#ifdef CONFIG_PRIORITY_INHERITANCE
  uint8_t prio;
#endif
  int ret;

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

  /* Perform the fsync using aioc_filep */

  ret = file_fsync(aioc->aioc_filep);
  if (ret < 0)
    {
      ferr("ERROR: file_fsync failed: %d\n", ret);
      aiocbp->aio_result = ret;
    }
  else
    {
      aiocbp->aio_result = OK;
    }

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
 *   If the aio_fsync() function fails or aiocbp indicates an error
 *   condition, data is not guaranteed to have been successfully transferred.
 *
 * Input Parameters:
 *   op     - Should be either O_SYNC or O_DSYNC.  Ignored in this
 *            implementation.
 *   aiocbp - A pointer to an instance of struct aiocb
 *
 * Returned Value:
 *   The aio_fsync() function will return the value 0 if the I/O operation is
 *   successfully queued; otherwise, the function will return the value -1
 *   and set errno to indicate the error.
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
 * - NuttX does not currently make any distinction between O_DYSNC and
 *   O_SYNC. Hence, the 'op' argument is ignored altogether.
 * - Most errors required in the standard are not detected at this point.
 *   There are no pre-queuing checks for the validity of the operation.
 *
 ****************************************************************************/

int aio_fsync(int op, FAR struct aiocb *aiocbp)
{
  FAR struct aio_container_s *aioc;
  int ret;

  DEBUGASSERT(op == O_SYNC); /* || op == O_DSYNC */
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

  ret = aio_queue(aioc, aio_fsync_worker);
  if (ret < 0)
    {
      /* The result and the errno have already been set */

      aioc_decant(aioc);
      return ERROR;
    }

  return OK;
}

#endif /* CONFIG_FS_AIO */
