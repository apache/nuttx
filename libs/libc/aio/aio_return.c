/****************************************************************************
 * libs/libc/aio/aio_return.c
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
