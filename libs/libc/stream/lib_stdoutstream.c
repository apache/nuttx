/****************************************************************************
 * libs/libc/stream/lib_stdoutstream.c
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

#include "libc.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stdoutstream_putc
 ****************************************************************************/

static void stdoutstream_putc(FAR struct lib_outstream_s *this, int ch)
{
  FAR struct lib_stdoutstream_s *sthis =
                               (FAR struct lib_stdoutstream_s *)this;
  int result;

  DEBUGASSERT(this && sthis->stream);

  /* Loop until the character is successfully transferred or an irrecoverable
   * error occurs.
   */

  do
    {
      result = fputc(ch, sthis->stream);
      if (result != EOF)
        {
          this->nput++;
          return;
        }

      /* EINTR (meaning that fputc was interrupted by a signal) is the only
       * recoverable error.
       */
    }
  while (get_errno() == EINTR);
}

/****************************************************************************
 * Name: stdoutstream_flush
 ****************************************************************************/

#ifndef CONFIG_STDIO_DISABLE_BUFFERING
static int stdoutstream_flush(FAR struct lib_outstream_s *this)
{
  FAR struct lib_stdoutstream_s *sthis =
                                (FAR struct lib_stdoutstream_s *)this;

  DEBUGASSERT(sthis != NULL && sthis->stream != NULL);
  return lib_fflush(sthis->stream, true);
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
 *   stream    - User provided stream instance (must have been opened for
 *               write access).
 *
 * Returned Value:
 *   None (User allocated instance initialized).
 *
 ****************************************************************************/

void lib_stdoutstream(FAR struct lib_stdoutstream_s *outstream,
                      FAR FILE *stream)
{
  /* Select the putc operation */

  outstream->public.putc = stdoutstream_putc;

  /* Select the correct flush operation.  This flush is only called when
   * a newline is encountered in the output stream.  However, we do not
   * want to support this line buffering behavior if the stream was
   * opened in binary mode.  In binary mode, the newline has no special
   * meaning.
   */

#ifndef CONFIG_STDIO_DISABLE_BUFFERING
  if (stream->fs_bufstart != NULL && (stream->fs_oflags & O_TEXT) != 0)
    {
      outstream->public.flush = stdoutstream_flush;
    }
  else
#endif
    {
      outstream->public.flush = lib_noflush;
    }

  /* Set the number of bytes put to zero and remember the stream */

  outstream->public.nput = 0;
  outstream->stream      = stream;
}
