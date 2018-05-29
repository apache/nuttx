/****************************************************************************
 * libs/libc/aio/aio_error.c
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
 * Name: aio_error
 *
 * Description:
 *   The aio_error() function returns the error status associated with the
 *   aiocb structure referenced by the aiocbp argument. The error status fo
 *   an asynchronous I/O operation is the errno value that would be set by
 *   the corresponding read(), write(), fdatasync(), or fsync() operation. If
 *   the operation has not yet completed, then the error status will be equal
 *   to EINPROGRESS.
 *
 * Input Parameters:
 *   aiocbp - A pointer to an instance of struct aiocb
 *
 * Returned Value:
 *   If the asynchronous I/O operation has completed successfully, then 0
 *   will be returned. If the asynchronous operation has completed
 *   unsuccessfully, then the error status, as described for read(),
 *   write(), fdatasync(), and fsync(), will be returned. If the
 *   asynchronous I/O operation has not yet completed, then EINPROGRESS will
 *   be returned.
 *
 *   The aio_error() function may fail if:
 *
 *     EINVAL - The aiocbp argument does not refer to an asynchronous
 *       operation whose return status has not yet been retrieved.
 *
 ****************************************************************************/

int aio_error(FAR const struct aiocb *aiocbp)
{
  DEBUGASSERT(aiocbp);

  if (aiocbp->aio_result < 0)
    {
      return -aiocbp->aio_result;
    }

  return OK;
}

#endif /* CONFIG_FS_AIO */
