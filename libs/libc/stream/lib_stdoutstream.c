/****************************************************************************
 * libs/libc/stream/lib_stdoutstream.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <stdio.h>

#include "libc.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stdoutstream_putc
 ****************************************************************************/

static void stdoutstream_putc(FAR struct lib_outstream_s *self, int ch)
{
  FAR struct lib_stdoutstream_s *stream =
                                      (FAR struct lib_stdoutstream_s *)self;
  int result;

  DEBUGASSERT(self && stream->handle);

  /* Loop until the character is successfully transferred or an irrecoverable
   * error occurs.
   */

  do
    {
      result = fputc(ch, stream->handle);
      if (result != EOF)
        {
          self->nput++;
          return;
        }

      /* EINTR (meaning that fputc was interrupted by a signal) is the only
       * recoverable error.
       */
    }
  while (get_errno() == EINTR);
}

/****************************************************************************
 * Name: stdoutstream_puts
 ****************************************************************************/

static ssize_t stdoutstream_puts(FAR struct lib_outstream_s *self,
                                 FAR const void *buffer, size_t len)
{
  FAR struct lib_stdoutstream_s *stream =
                               (FAR struct lib_stdoutstream_s *)self;
  ssize_t result;

  DEBUGASSERT(self && stream->handle);

  /* Loop until the buffer is successfully transferred or an irrecoverable
   * error occurs.
   */

  do
    {
      result = fwrite(buffer, 1, len, stream->handle);
      if (result >= 0)
        {
          self->nput += result;
          return result;
        }

      result = _NX_GETERRVAL(result);

      /* EINTR (meaning that fputc was interrupted by a signal) is the only
       * recoverable error.
       */
    }
  while (result == -EINTR);

  return result;
}

/****************************************************************************
 * Name: stdoutstream_flush
 ****************************************************************************/

#ifndef CONFIG_STDIO_DISABLE_BUFFERING
static int stdoutstream_flush(FAR struct lib_outstream_s *self)
{
  FAR struct lib_stdoutstream_s *stream =
                                (FAR struct lib_stdoutstream_s *)self;

  DEBUGASSERT(stream != NULL && stream->handle != NULL);
  return lib_fflush(stream->handle);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_stdoutstream
 *
 * Description:
 *   Initializes a stream for use with a FILE instance.
 *
 * Input Parameters:
 *   outstream - User allocated, uninitialized instance of struct
 *               lib_stdoutstream_s to be initialized.
 *   handle    - User provided FILE instance (must have been opened for
 *               write access).
 *
 * Returned Value:
 *   None (User allocated instance initialized).
 *
 ****************************************************************************/

void lib_stdoutstream(FAR struct lib_stdoutstream_s *stream,
                      FAR FILE *handle)
{
  /* Select the putc operation */

  stream->common.putc = stdoutstream_putc;
  stream->common.puts = stdoutstream_puts;

  /* Select the correct flush operation.  This flush is only called when
   * a newline is encountered in the output file stream.  However, we do not
   * want to support this line buffering behavior if the file was
   * opened in binary mode.  In binary mode, the newline has no special
   * meaning.
   */

#ifndef CONFIG_STDIO_DISABLE_BUFFERING
  if (handle->fs_bufstart != NULL && (handle->fs_oflags & O_TEXT) != 0)
    {
      stream->common.flush = stdoutstream_flush;
    }
  else
#endif
    {
      stream->common.flush = lib_noflush;
    }

  /* Set the number of bytes put to zero and remember the handle */

  stream->common.nput = 0;
  stream->handle      = handle;
}
