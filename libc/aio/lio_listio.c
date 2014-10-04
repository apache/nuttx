/****************************************************************************
 * libc/aio/lio_listio.c
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

#include <pthread.h>
#include <signal.h>
#include <aio.h>
#include <assert.h>
#include <errno.h>

#include "lib_internal.h"

#ifndef CONFIG_DISABLE_PTHREAD

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* This structure is passed to lio_thread() when the it is started by
 * lio_listio().
 */

struct lio_threadparm_s
{
  FAR struct sigevent *sig;        /* Used to notify the caller */
  FAR struct aiocb *const *list;  /* The list of I/O operations to be performed */
  pid_t pid;                       /* ID of caller to be notified */
  int nent;                        /* The number of elements in the list */
};

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
 * Name: lio_dolistio
 *
 * Description:
 *   This function executes the I/O list.  If lio_listio() is call with mode
 *   equal to LIO_WAIT, then this function will be called directly from
 *   lio_listio().  If lio_listio() is called with mode equal to LIO_NOWAIT,
 *   this function will be called from lio_thread.  In either case, this
 *   function will not return until all of the I/O is the list has completed.
 *
 * Input Parameters:
 *   list - The list of I/O operations to be performed
 *   nent - The number of elements in the list
 *
 * Returned Value:
 *   Zero if the I/O list was complete successfully.  Otherwise, a negated
 *   errno value is returned.  NOTE that the errno is not set here because
 *   we may be running asynchronously on a different thread from the caller.
 *
 ****************************************************************************/

static int lio_dolistio(FAR struct aiocb *const *list, int nent)
{
#warning Missing logic
  return -ENOSYS;
}

/****************************************************************************
 * Name: lio_thread
 *
 * Description:
 *   When lio_listio() is called with LIO_NOWAIT, list I/O processing will
 *   occur on this thread.
 *
 * Input Parameters:
 *   arg - An instance of struct lio_thread_parm_s cast to pthread_addr_.t
 *
 * Returned Value:
 *   NULL is returned.
 *
 ****************************************************************************/

static pthread_addr_t lio_thread(pthread_addr_t arg)
{
  FAR struct lio_threadparm_s *parms = (FAR struct lio_threadparm_s *)arg;
  DEBUGASSERT(parms && parms->list);
#warning Missing logic
  return NULL;
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
 *   I/O operation indicated by each list element can encounter errors specific
 *   to the individual read or write function being performed. In this event,
 *   the error status for each aiocb control block contains the associated
 *   error code. The error codes that can be set are the same as would be
 *   set by a read() or write() function, with the following additional
 *   error codes possible:
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

int lio_listio(int mode, FAR struct aiocb *const list[], int nent,
               FAR struct sigevent *sig)
{
#warning Missing logic
  return -ENOSYS;
}

#endif /* !CONFIG_DISABLE_PTHREAD */
