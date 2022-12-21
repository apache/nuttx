/****************************************************************************
 * libs/libc/stdio/lib_fflush.c
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

#include <stdbool.h>
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>

#include <nuttx/fs/fs.h>

#include "libc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fflush
 *
 * Description:
 *  The function fflush() forces a write of all user-space buffered data for
 *  the given output or update stream via the stream's underlying write
 *  function.  The open status of the stream is unaffected.
 *
 *  If the stream argument is NULL, fflush() flushes all open output streams.
 *
 * Returned Value:
 *  OK on success EOF on failure (with errno set appropriately)
 *
 ****************************************************************************/

int fflush(FAR FILE *stream)
{
  int ret;

  /* Is the stream argument NULL? */

  if (!stream)
    {
      /* Yes... then this is a request to flush all streams */

      ret = lib_flushall(lib_get_streams());
    }
  else
    {
      ret = lib_fflush(stream, true);
    }

  /* Check the return value */

  if (ret < 0)
    {
      /* An error occurred during the flush AND/OR we were unable to flush
       * all of the buffered write data. Set the errno value.
       */

      set_errno(-ret);

      /* And return EOF on failure. */

      return EOF;
    }

  return OK;
}
