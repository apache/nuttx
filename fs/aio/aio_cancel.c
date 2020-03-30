/****************************************************************************
 * fs/aio/aio_cancel.c
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

#include <aio.h>
#include <sched.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/wqueue.h>

#include "aio/aio.h"

#ifdef CONFIG_FS_AIO

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: aio_cancel
 *
 * Description:
 *   The aio_cancel() function attempts to cancel one or more asynchronous
 *   I/O requests currently outstanding against file descriptor 'fildes'.
 *   The aiocbp argument points to the asynchronous I/O control block for
 *   a particular request to be canceled. If aiocbp is NULL, then all
 *   outstanding cancelable asynchronous I/O requests against fildes will
 *   be canceled.
 *
 *   Normal asynchronous notification will occur for asynchronous I/O
 *   operations that are successfully canceled. If there are requests that
 *   cannot be canceled, then the normal asynchronous completion process
 *   will take place for those requests when they are completed.
 *
 *   For requested operations that are successfully canceled, the associated
 *   error status will be set to ECANCELED and the return status will be -1.
 *   For requested operations that are not successfully canceled, the aiocbp
 *   will not be modified by aio_cancel().
 *
 * Input Parameters:
 *   fildes - Not used in this implementation
 *   aiocbp - Points to the asynchronous I/O control block for a particular
 *            request to be canceled.
 *
 * Returned Value:
 *    The aio_cancel() function will return the value AIO_CANCELED if the
 *    requested operation(s) were canceled. The value AIO_NOTCANCELED will
 *    be returned if at least one of the requested operation(s) cannot be
 *    canceled because it is in progress. In this case, the state of the
 *    other operations, if any, referenced in the call to aio_cancel() is
 *    not indicated by the return value of aio_cancel(). The application
 *    may determine the state of affairs for these operations by using
 *    aio_error(). The value AIO_ALLDONE is returned if all of the
 *    operations have already completed. Otherwise, the function will return
 *    -1 and set errno to indicate the error.
 *
 ****************************************************************************/

int aio_cancel(int fildes, FAR struct aiocb *aiocbp)
{
  FAR struct aio_container_s *aioc;
  FAR struct aio_container_s *next;
  pid_t pid;
  int status;
  int ret;

  /* Check if a non-NULL aiocbp was provided */

  /* Lock the scheduler so that no I/O events can complete on the worker
   * thread until we set complete this operation.
   */

  ret = AIO_ALLDONE;
  sched_lock();
  ret = aio_lock();

  if (aiocbp)
    {
      /* Check if the I/O has completed */

      if (aiocbp->aio_result == -EINPROGRESS)
        {
          /* No.. Find the container for this AIO control block */

          for (aioc = (FAR struct aio_container_s *)g_aio_pending.head;
               aioc && aioc->aioc_aiocbp != aiocbp;
               aioc = (FAR struct aio_container_s *)aioc->aioc_link.flink);

          /* Did we find a container for this fildes?  We should; the
           * aio_result says that the transfer is pending.  If not we return
           * AIO_ALLDONE.
           */

          if (aioc)
            {
              /* Yes... attempt to cancel the I/O.  There are two
               * possibilities:* (1) the work has already been started and
               * is no longer queued, or (2) the work has not been started
               * and is still in the work queue.  Only the second case can
               * be canceled.  work_cancel() will return -ENOENT in the
               * first case.
               */

              status = work_cancel(LPWORK, &aioc->aioc_work);
              if (status >= 0)
                {
                  /* Remove the container from the list of pending transfers */

                  pid = aioc->aioc_pid;
                  aioc_decant(aioc);

                  aiocbp->aio_result = -ECANCELED;
                  ret = AIO_CANCELED;

                  /* Signal the client */

                  aio_signal(pid, aiocbp);
                }
              else
                {
                  ret = AIO_NOTCANCELED;
                }
            }
          else
            {
              /* Can't cancel, the operation is running */

              ret = AIO_NOTCANCELED;
            }
        }
    }
  else
    {
      /* No aiocbp.. cancel all outstanding I/O for the fildes */

      next = (FAR struct aio_container_s *)g_aio_pending.head;
      do
        {
          /* Find the next container with this AIO control block */

          for (aioc = next;
               aioc && aioc->aioc_aiocbp->aio_fildes != fildes;
               aioc = (FAR struct aio_container_s *)aioc->aioc_link.flink);

          /* Did we find the container?  We should; the aio_result says
           * that the transfer is pending.  If not we return AIO_ALLDONE.
           */

          if (aioc)
            {
              /* Yes... attempt to cancel the I/O.  There are two
               * possibilities:* (1) the work has already been started and
               * is no longer queued, or (2) the work has not been started
               * and is still in the work queue.  Only the second case can
               * be canceled.  work_cancel() will return -ENOENT in the
               * first case.
               */

              status = work_cancel(LPWORK, &aioc->aioc_work);
              if (status >= 0)
                {
                  /* Remove the container from the list of pending transfers */

                  next   =
                    (FAR struct aio_container_s *)aioc->aioc_link.flink;
                  pid    = aioc->aioc_pid;
                  aiocbp = aioc_decant(aioc);
                  DEBUGASSERT(aiocbp);

                  aiocbp->aio_result = -ECANCELED;
                  if (ret != AIO_NOTCANCELED)
                    {
                      ret = AIO_CANCELED;
                    }

                  /* Signal the client */

                  aio_signal(pid, aiocbp);
                }
              else
                {
                  ret = AIO_NOTCANCELED;
                }
            }
        }
      while (aioc);
    }

  aio_unlock();
  sched_unlock();
  return ret;
}

#endif /* CONFIG_FS_AIO */
