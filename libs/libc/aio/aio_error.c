/****************************************************************************
 * libs/libc/aio/aio_error.c
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
