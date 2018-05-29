/****************************************************************************
 * libs/libc/aio/aio_return.c
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
#include <assert.h>
#include <errno.h>

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
 * Name: aio_return
 *
 * Description:
 *   The aio_return() function returns the return status associated with
 *   the aiocb structure referenced by the aiocbp argument. The return
 *   status for an asynchronous I/O operation is the value that would be
 *   returned by the corresponding read(), write(), or fsync() function
 *   call. If the error status for the operation is equal to EINPROGRESS,
 *   then the return status for the operation is undefined. The aio_return()
 *   function may be called exactly once to retrieve the return status of
 *   a given asynchronous operation; thereafter, if the same aiocb structure
 *   is used in a call to aio_return() or aio_error(), an error may be
 *   returned. When the aiocb structure referred to by aiocbp is used to
 *   submit another asynchronous operation, then aio_return() may be
 *   successfully used to retrieve the return status of that operation.
 *
 * Input Parameters:
 *   aiocbp - A pointer to an instance of struct aiocb
 *
 * Returned Value:
 *   If the asynchronous I/O operation has completed, then the return
 *   status, as described for read(), write(), and fsync(), will be
 *   returned. If the asynchronous I/O operation has not yet completed,
 *   the results of aio_return() are undefined.
 *
 *    The aio_return() function may fail if:
 *
 *    EINVAL - The aiocbp argument does not refer to an asynchronous
 *      operation whose return status has not yet been retrieved.
 *
 ****************************************************************************/

ssize_t aio_return(FAR struct aiocb *aiocbp)
{
  DEBUGASSERT(aiocbp);
  if (aiocbp->aio_result < 0)
    {
      set_errno((int)-aiocbp->aio_result);
      return (ssize_t)ERROR;
    }

  return aiocbp->aio_result;
}

#endif /* CONFIG_FS_AIO */
