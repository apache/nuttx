/****************************************************************************
 * libs/libc/stdio/lib_wrflush.c
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
#include <fcntl.h>
#include <errno.h>

#include "libc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_wrflush
 *
 * Description:
 *   This is simply a version of fflush that does not report an error if
 *   the file is not open for writing.
 *
 ****************************************************************************/

int lib_wrflush(FAR FILE *stream)
{
#ifndef CONFIG_STDIO_DISABLE_BUFFERING
  /* Verify that we were passed a valid (i.e., non-NULL) stream */

#ifdef CONFIG_DEBUG_FEATURES
  if (stream == NULL)
    {
      return -EINVAL;
    }
#endif

  /* Do nothing if there is no I/O buffer */

  if (stream->fs_bufstart == NULL)
    {
      return OK;
    }

  /* Verify that the stream is opened for writing... lib_fflush will
   * return an error if it is called for a stream that is not opened for
   * writing.  Check that first so that this function will not fail in
   * that case.
   */

  if ((stream->fs_oflags & O_WROK) == 0)
    {
      /* Report that the success was successful if we attempt to flush a
       * read-only stream.
       */

      return OK;
    }

  /* Flush the stream.   Return success if there is no buffered write data
   * -- i.e., that the stream is opened for writing and  that all of the
   * buffered write data was successfully flushed by lib_fflush().
   */

  return lib_fflush(stream, true);

#else
  /* Verify that we were passed a valid (i.e., non-NULL) stream */

#ifdef CONFIG_DEBUG_FEATURES
  if (!stream)
    {
      return -EINVAL;
    }
#endif

  return OK;
#endif
}
