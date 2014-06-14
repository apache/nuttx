/****************************************************************************
 * libc/stdio/lib_stdsostream.c
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

#include <fcntl.h>
#include <assert.h>
#include <errno.h>

#include "lib_internal.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stdsostream_putc
 ****************************************************************************/

static void stdsostream_putc(FAR struct lib_sostream_s *this, int ch)
{
  FAR struct lib_stdsostream_s *sthis = (FAR struct lib_stdsostream_s *)this;
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
 * Name: stdsostream_flush
 ****************************************************************************/

#if defined(CONFIG_STDIO_LINEBUFFER) && CONFIG_STDIO_BUFFER_SIZE > 0
static int stdsostream_flush(FAR struct lib_sostream_s *this)
{
  FAR struct lib_stdsostream_s *sthis = (FAR struct lib_stdsostream_s *)this;
  return lib_fflush(sthis->stream, true);
}
#endif

/****************************************************************************
 * Name: stdsostream_seek
 ****************************************************************************/

static off_t stdsostream_seek(FAR struct lib_sostream_s *this, off_t offset,
                              int whence)
{
  FAR struct lib_stdsostream_s *mthis = (FAR struct lib_stdsostream_s *)this;

  DEBUGASSERT(this);
  return fseek(mthis->stream, offset, whence);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_stdsostream
 *
 * Description:
 *   Initializes a stream for use with a FILE instance.
 *
 * Input parameters:
 *   outstream - User allocated, uninitialized instance of struct
 *               lib_stdsostream_s to be initialized.
 *   stream    - User provided stream instance (must have been opened for
 *               write access).
 *
 * Returned Value:
 *   None (User allocated instance initialized).
 *
 ****************************************************************************/

void lib_stdsostream(FAR struct lib_stdsostream_s *outstream,
                     FAR FILE *stream)
{
  /* Select the put operation */

  outstream->public.put = stdsostream_putc;

  /* Select the correct flush operation.  This flush is only called when
   * a newline is encountered in the output stream.  However, we do not
   * want to support this line buffering behavior if the stream was
   * opened in binary mode.  In binary mode, the newline has no special
   * meaning.
   */

#ifdef CONFIG_STDIO_LINEBUFFER
#if CONFIG_STDIO_BUFFER_SIZE > 0
  if ((stream->fs_oflags & O_BINARY) == 0)
    {
      outstream->public.flush = stdsostream_flush;
    }
  else
#endif
    {
      outstream->public.flush = lib_snoflush;
    }
#endif

  /* Select the seek operation */

  outstream->public.seek = stdsostream_seek;

  /* Set the number of bytes put to zero and remember the stream */

  outstream->public.nput = 0;
  outstream->stream     = stream;
}
