/****************************************************************************
 * libs/libc/aio/lio_listio.c
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
#include <signal.h>
#include <aio.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/signal.h>

#include "libc.h"
#include "aio/aio.h"

#ifdef CONFIG_FS_AIO

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct lio_sighand_s
{
  FAR struct aiocb * const *list;  /* List of I/O operations */
  FAR struct sigevent sig;         /* Describes how to signal the caller */
  int nent;                        /* Number or elements in list[] */
  pid_t pid;                       /* ID of client */
  sigset_t oprocmask;              /* sigprocmask to restore */
  struct sigaction oact;           /* Signal handler to restore */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lio_checkio
 *
 * Description:
 *  Check if all I/O operations in the list are complete.
 *
 * Input Parameters:
 *   list - The list of I/O operations to be performed
 *   nent - The number of elements in the list
 *
 * Returned Value:
 *  Zero (OK) is returned if all I/O completed successfully.
 *  -EINPROGRESS is returned if one or more I/Os have not yet completed.
 *  The negated errno value if first error noted in the case where all I/O
 *  completed but one or more I/Os completed with an error.
 *
 * Assumptions:
 *  The scheduler is locked and no I/O can complete asynchronously with
 *  the logic in this function.
 *
 ****************************************************************************/

static int lio_checkio(FAR struct aiocb * const *list, int nent)
{
  FAR struct aiocb *aiocbp;
  int ret;
  int i;

  ret = OK; /* Assume success */

  /* Check each entry in the list.  Break out of the loop if any entry
   * has not completed.
   */

  for (i = 0; i < nent; i++)
    {
      /* Skip over NULL entries */

      aiocbp = list[i];
      if (aiocbp)
        {
          /* Check if the I/O has completed */

          if (aiocbp->aio_result == -EINPROGRESS)
            {
              /* No.. return -EINPROGRESS */

              return -EINPROGRESS;
            }

          /* Check for an I/O error */

          else if (aiocbp->aio_result < 0 && ret == OK)
            {
              /* Some other error other than -EINPROGRESS */

              ret = aiocbp->aio_result;
            }
        }
    }

  /* All of the I/Os have completed */

  return ret;
}

/****************************************************************************
 * Name: lio_sighandler
 *
 * Description:
 *   Handle the SIGPOLL signal.
 *
 * Input Parameters:
 *   signo   - The number of the signal that we caught (SIGPOLL)
 *   info    - Information accompanying the signal
 *   context - Not used in NuttX
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

static void lio_sighandler(int signo, siginfo_t *info, void *ucontext)
{
  FAR struct aiocb *aiocbp;
  FAR struct lio_sighand_s *sighand;
  int ret;

  DEBUGASSERT(signo == SIGPOLL && info);

  /* The info structure should contain a pointer to the AIO control block */

  aiocbp = (FAR struct aiocb *)info->si_value.sival_ptr;
  DEBUGASSERT(aiocbp && aiocbp->aio_result != -EINPROGRESS);

  /* Recover our private data from the AIO control block */

  sighand = (FAR struct lio_sighand_s *)aiocbp->aio_priv;
  DEBUGASSERT(sighand && sighand->list);
  aiocbp->aio_priv = NULL;

  /* Prevent any asynchronous I/O completions while the signal handler runs */

  sched_lock();

  /* Check if all of the pending I/O has completed */

  ret = lio_checkio(sighand->list, sighand->nent);
  if (ret != -EINPROGRESS)
    {
      /* All pending I/O has completed */

      /* Restore the signal handler */

      sigaction(SIGPOLL, &sighand->oact, NULL);

      /* Restore the sigprocmask */

      sigprocmask(SIG_SETMASK, &sighand->oprocmask, NULL);

      /* Signal the client */

      DEBUGVERIFY(nxsig_notification(sighand->pid, &sighand->sig,
                                     SI_ASYNCIO, &aiocbp->aio_sigwork));

      /* And free the container */

      lib_free(sighand);
    }

  sched_unlock();
}

/****************************************************************************
 * Name: lio_sigsetup
 *
 * Description:
 *   Setup a signal handler to detect when until all I/O completes.
 *
 * Input Parameters:
 *   list - The list of I/O operations to be performed
 *   nent - The number of elements in the list
 *
 * Returned Value:
 *  Zero (OK) is returned if all I/O completed successfully; Otherwise, a
 *  negated errno value is returned corresponding to the first error
 *  detected.
 *
 * Assumptions:
 *  The scheduler is locked and no I/O can complete asynchronously with
 *  the logic in this function.
 *
 ****************************************************************************/

static int lio_sigsetup(FAR struct aiocb * const *list, int nent,
                        FAR struct sigevent *sig)
{
  FAR struct aiocb *aiocbp;
  FAR struct lio_sighand_s *sighand;
  sigset_t set;
  struct sigaction act;
  int status;
  int i;

  /* Allocate a structure to pass data to the signal handler */

  sighand = lib_zalloc(sizeof(struct lio_sighand_s));
  if (!sighand)
    {
      ferr("ERROR: lib_zalloc failed\n");
      return -ENOMEM;
    }

  /* Initialize the allocated structure */

  sighand->list = list;
  sighand->sig  = *sig;
  sighand->nent = nent;
  sighand->pid  = getpid();

  /* Save this structure as the private data attached to each aiocb */

  for (i = 0; i < nent; i++)
    {
      /* Skip over NULL entries in the list */

      aiocbp = list[i];
      if (aiocbp)
        {
          FAR void *priv = NULL;

          /* Check if I/O is pending for  this entry */

          if (aiocbp->aio_result == -EINPROGRESS)
            {
              priv = (FAR void *)sighand;
            }

          aiocbp->aio_priv = priv;
        }
    }

  /* Make sure that SIGPOLL is not blocked */

  sigemptyset(&set);
  sigaddset(&set, SIGPOLL);
  status = sigprocmask(SIG_UNBLOCK, &set, &sighand->oprocmask);
  if (status != OK)
    {
      int errcode = get_errno();
      ferr("ERROR sigprocmask failed: %d\n", errcode);
      DEBUGASSERT(errcode > 0);
      return -errcode;
    }

  /* Attach our signal handler */

  finfo("Registering signal handler\n");

  act.sa_sigaction = lio_sighandler;
  act.sa_flags = SA_SIGINFO;

  sigfillset(&act.sa_mask);
  sigdelset(&act.sa_mask, SIGPOLL);

  status = sigaction(SIGPOLL, &act, &sighand->oact);
  if (status != OK)
    {
      int errcode = get_errno();

      ferr("ERROR sigaction failed: %d\n", errcode);

      DEBUGASSERT(errcode > 0);
      return -errcode;
    }

  return OK;
}

/****************************************************************************
 * Name: lio_waitall
 *
 * Description:
 *  Wait for all I/O operations in the list to be complete.
 *
 * Input Parameters:
 *   list - The list of I/O operations to be performed
 *   nent - The number of elements in the list
 *
 * Returned Value:
 *  Zero (OK) is returned if all I/O completed successfully; Otherwise, a
 *  negated errno value is returned corresponding to the first error
 *  detected.
 *
 * Assumptions:
 *  The scheduler is locked and no I/O can complete asynchronously with
 *  the logic in this function.
 *
 ****************************************************************************/

static int lio_waitall(FAR struct aiocb * const *list, int nent)
{
  sigset_t set;
  int ret;

  /* Loop until all I/O completes */

  for (; ; )
    {
      /* Check if all I/O has completed */

      ret = lio_checkio(list, nent);
      if (ret != -EINPROGRESS)
        {
          /* All I/O has completed.. We are finished.  */

          return ret;
        }

      /* Then wait for SIGPOLL -- indefinitely.
       *
       * NOTE: If completion of the I/O causes other signals to be generated
       * first, then this will wake up and return EINTR instead of success.
       */

      sigemptyset(&set);
      sigaddset(&set, SIGPOLL);

      ret = sigwaitinfo(&set, NULL);
      if (ret < 0)
        {
          int errcode = get_errno();

          /* The most likely reason that we would get here is because some
           * unrelated signal has been received.
           */

          ferr("ERROR: sigwaitinfo failed: %d\n", errcode);
          DEBUGASSERT(errcode > 0);
          return -errcode;
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lio_listio
 *
 * Description:
 *   The lio_listio() function initiates a list of I/O requests with a
 *   single function call.
 *
 *   The 'mode' argument takes one of the values LIO_WAIT or LIO_NOWAIT
 *   declared in <aio.h> and determines whether the function returns when
 *   the I/O operations have been completed, or as soon as the operations
 *   have been queued. If the 'mode' argument is LIO_WAIT, the function will
 *   wait until all I/O is complete and the 'sig' argument will be ignored.
 *
 *   If the 'mode' argument is LIO_NOWAIT, the function will return
 *   immediately, and asynchronous notification will occur, according to the
 *   'sig' argument, when all the I/O operations complete. If 'sig' is NULL,
 *   then no asynchronous notification will occur. If 'sig' is not NULL,
 *   asynchronous notification occurs when all the requests in 'list' have
 *   completed.
 *
 *   The I/O requests enumerated by 'list' are submitted in an unspecified
 *   order.
 *
 *   The 'list' argument is an array of pointers to aiocb structures. The
 *   array contains 'nent 'elements. The array may contain NULL elements,
 *   which will be ignored.
 *
 *   If the buffer pointed to by 'list' or the aiocb structures pointed to
 *   by the elements of the array 'list' become illegal addresses before all
 *   asynchronous I/O completed and, if necessary, the notification is
 *   sent, then the behavior is undefined. If the buffers pointed to by the
 *   aio_buf member of the aiocb structure pointed to by the elements of
 *   the array 'list' become illegal addresses prior to the asynchronous
 *   I/O associated with that aiocb structure being completed, the behavior
 *   is undefined.
 *
 *   The aio_lio_opcode field of each aiocb structure specifies the
 *   operation to be performed. The supported operations are LIO_READ,
 *   LIO_WRITE, and LIO_NOP; these symbols are defined in <aio.h>. The
 *   LIO_NOP operation causes the list entry to be ignored. If the
 *   aio_lio_opcode element is equal to LIO_READ, then an I/O operation is
 *   submitted as if by a call to aio_read() with the aiocbp equal to the
 *   address of the aiocb structure. If the aio_lio_opcode element is equal
 *   to LIO_WRITE, then an I/O operation is submitted as if by a call to
 *   aio_write() with the aiocbp equal to the address of the aiocb
 *   structure.
 *
 *   The aio_fildes member specifies the file descriptor on which the
 *   operation is to be performed.
 *
 *   The aio_buf member specifies the address of the buffer to or from which
 *   the data is transferred.
 *
 *   The aio_nbytes member specifies the number of bytes of data to be
 *   transferred.
 *
 *   The members of the aiocb structure further describe the I/O operation
 *   to be performed, in a manner identical to that of the corresponding
 *   aiocb structure when used by the aio_read() and aio_write() functions.
 *
 *   The 'nent' argument specifies how many elements are members of the list;
 *   that is, the length of the array.
 *
 * Input Parameters:
 *   mode - Either LIO_WAIT or LIO_NOWAIT
 *   list - The list of I/O operations to be performed
 *   nent - The number of elements in the list
 *   sig  - Used to notify the caller when the I/O is performed
 *          asynchronously.
 *
 * Returned Value:
 *   If the mode argument has the value LIO_NOWAIT, the lio_listio()
 *   function will return the value zero if the I/O operations are
 *   successfully queued; otherwise, the function will return the value
 *   -1 and set errno to indicate the error.
 *
 *   If the mode argument has the value LIO_WAIT, the lio_listio() function
 *   will return the value zero when all the indicated I/O has completed
 *   successfully. Otherwise, lio_listio() will return a value of -1 and
 *   set errno to indicate the error.
 *
 *   In either case, the return value only indicates the success or failure
 *   of the lio_listio() call itself, not the status of the individual I/O
 *   requests. In some cases one or more of the I/O requests contained in
 *   the list may fail. Failure of an individual request does not prevent
 *   completion of any other individual request. To determine the outcome
 *   of each I/O request, the application must examine the error status
 *   associated with each aiocb control block. The error statuses so
 *   returned are identical to those returned as the result of an aio_read()
 *   or aio_write() function.
 *
 *   The lio_listio() function will fail if:
 *
 *     EAGAIN - The resources necessary to queue all the I/O requests were
 *       not available. The application may check the error status for each
 *       aiocb to determine the individual request(s) that failed.
 *     EAGAIN - The number of entries indicated by 'nent' would cause the
 *       system-wide limit {AIO_MAX} to be exceeded.
 *     EINVAL - The mode argument is not a proper value, or the value of
 *       'nent' was greater than {AIO_LISTIO_MAX}.
 *     EINTR - A signal was delivered while waiting for all I/O requests to
 *       complete during an LIO_WAIT operation. Note that, since each I/O
 *       operation invoked by lio_listio() may possibly provoke a signal when
 *       it completes, this error return may be caused by the completion of
 *       one (or more) of the very I/O operations being awaited. Outstanding
 *       I/O requests are not cancelled, and the application will examine
 *       each list element to determine whether the request was initiated,
 *       cancelled, or completed.
 *     EIO - One or more of the individual I/O operations failed. The
 *       application may check the error status for each aiocb structure to
 *       determine the individual request(s) that failed.
 *
 *   In addition to the errors returned by the lio_listio() function, if the
 *   lio_listio() function succeeds or fails with errors of EAGAIN, EINTR, or
 *   EIO, then some of the I/O specified by the list may have been initiated.
 *   If the lio_listio() function fails with an error code other than EAGAIN,
 *   EINTR, or EIO, no operations from the list will have been initiated. The
 *   I/O operation indicated by each list element can encounter errors
 *   specific to the individual read or write function being performed. In
 *   this event, the error status for each aiocb control block contains the
 *   associated error code. The error codes that can be set are the same as
 *   would be set by a read() or write() function, with the following
 *   additional error codes possible:
 *
 *     EAGAIN - The requested I/O operation was not queued due to resource
 *       limitations.
 *     ECANCELED - The requested I/O was cancelled before the I/O completed
 *       due to an explicit aio_cancel() request.
 *     EFBIG - The aiocbp->aio_lio_opcode is LIO_WRITE, the file is a
 *       regular file, aiocbp->aio_nbytes is greater than 0, and the
 *       aiocbp->aio_offset is greater than or equal to the offset maximum
 *       in the open file description associated with aiocbp->aio_fildes.
 *     EINPROGRESS - The requested I/O is in progress.
 *     EOVERFLOW - The aiocbp->aio_lio_opcode is LIO_READ, the file is a
 *       regular file, aiocbp->aio_nbytes is greater than 0, and the
 *       aiocbp->aio_offset is before the end-of-file and is greater than
 *       or equal to the offset maximum in the open file description
 *       associated with aiocbp->aio_fildes.
 *
 ****************************************************************************/

int lio_listio(int mode, FAR struct aiocb * const list[], int nent,
               FAR struct sigevent *sig)
{
  FAR struct aiocb *aiocbp = NULL;
  int nqueued;
  int errcode;
  int retcode;
  int status;
  int ret;
  int i;

  DEBUGASSERT(mode == LIO_WAIT || mode == LIO_NOWAIT);
  DEBUGASSERT(list);

  nqueued = 0;    /* No I/O operations yet queued */
  ret     = OK;   /* Assume success */

  /* Lock the scheduler so that no I/O events can complete on the worker
   * thread until we set our wait set up.  Pre-emption will, of course, be
   * re-enabled while we are waiting for the signal.
   */

  sched_lock();

  /* Submit each asynchronous I/O operation in the list, skipping over NULL
   * entries.
   */

  for (i = 0; i < nent; i++)
    {
      /* Skip over NULL entries */

      aiocbp = list[i];
      if (aiocbp)
        {
          /* Submit the operation according to its opcode */

          status = OK;
          switch (aiocbp->aio_lio_opcode)
            {
            case LIO_NOP:
              {
                /* Mark the do-nothing operation complete */

                aiocbp->aio_result = OK;
              }
              break;

            case LIO_READ:
            case LIO_WRITE:
              {
                if (aiocbp->aio_lio_opcode == LIO_READ)
                  {
                    /* Submit the asynchronous read operation */

                    status = aio_read(aiocbp);
                  }
                else
                  {
                    /* Submit the asynchronous write operation */

                    status = aio_write(aiocbp);
                  }

                if (status < 0)
                  {
                    /* Failed to queue the I/O.  Set up the error return. */

                    errcode = get_errno();
                    ferr("ERROR: aio_read/write failed: %d\n", errcode);
                    DEBUGASSERT(errcode > 0);
                    aiocbp->aio_result = -errcode;
                    ret = ERROR;
                  }
                else
                  {
                    /* Increment the count of successfully queue operations */

                    nqueued++;
                  }
              }
              break;

            default:
              {
                /* Make the invalid operation complete with an error */

                ferr("ERROR: Unrecognized opcode: %d\n",
                     aiocbp->aio_lio_opcode);
                aiocbp->aio_result = -EINVAL;
                ret = ERROR;
              }
              break;
            }
        }
    }

  /* If there was any failure in queuing the I/O, EIO will be returned */

  retcode = EIO;

  /* Now what? Three possibilities:
   *
   * Case 1: mode == LIO_WAIT
   *
   *   Ignore the sig argument; Do no return until all I/O completes.
   */

  if (mode == LIO_WAIT)
    {
      /* Don't wait if all if no I/O was queue */

      if (nqueued > 0)
        {
          /* Wait until all I/O completes.  The scheduler will be unlocked
           * while we are waiting.
           */

          status = lio_waitall(list, nent);
          if (status < 0 && ret == OK)
            {
              /* Something bad happened while waiting and this is the first
               * error to be reported.
               */

              retcode = -status;
              ret     = ERROR;
            }
        }
    }

  /* Case 2: mode == LIO_NOWAIT and sig != NULL
   *
   *   If any I/O was queued, then setup to signal the caller when all of
   *   the transfers complete.
   *
   *   If no I/O was queue, then we I suppose that we need to signal the
   *   caller ourself?
   */

  else if (sig != NULL)
    {
      if (nqueued > 0)
        {
          /* Setup a signal handler to detect when until all I/O completes. */

          status = lio_sigsetup(list, nent, sig);
          if (status < 0 && ret == OK)
            {
              /* Something bad happened while setting up the signal and this
               * is the first error to be reported.
               */

              retcode = -status;
              ret     = ERROR;
            }
        }
      else
        {
          status = nxsig_notification(getpid(), sig,
                                      SI_ASYNCIO, &aiocbp->aio_sigwork);
          if (status < 0 && ret == OK)
            {
              /* Something bad happened while performing the notification
               * and this is the first error to be reported.
               */

               retcode = -status;
               ret     = ERROR;
            }
        }
    }

  /* Case 3: mode == LIO_NOWAIT and sig == NULL
   *
   *   Just return now.
   */

  sched_unlock();
  if (ret < 0)
    {
      set_errno(retcode);
      return ERROR;
    }

  return OK;
}

#endif /* CONFIG_FS_AIO */
