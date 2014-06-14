/****************************************************************************
 * libc/stdio/lib_memsostream.c
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

#include <assert.h>

#include "lib_internal.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: memsostream_putc
 ****************************************************************************/

static void memsostream_putc(FAR struct lib_sostream_s *this, int ch)
{
  FAR struct lib_memsostream_s *mthis = (FAR struct lib_memsostream_s *)this;

  DEBUGASSERT(this);

  /* If this will not overrun the buffer, then write the character to the
   * buffer.  Not that buflen was pre-decremented when the stream was
   * created so it is okay to write past the end of the buflen by one.
   */

  if (mthis->offset < mthis->buflen)
    {
      mthis->buffer[mthis->offset] = ch;
      mthis->offset++;
      this->nput++;
      mthis->buffer[mthis->offset] = '\0';
    }
}

/****************************************************************************
 * Name: memsostream_seek
 ****************************************************************************/

static off_t memsostream_seek(FAR struct lib_sostream_s *this, off_t offset,
                              int whence)
{
  FAR struct lib_memsostream_s *mthis = (FAR struct lib_memsostream_s *)this;
  off_t newpos;

  DEBUGASSERT(this);

  switch (whence)
    {
      case SEEK_CUR:
        newpos = (off_t)mthis->offset + offset;
        break;

      case SEEK_SET:
        newpos = offset;
        break;

      case SEEK_END:
        newpos = (off_t)mthis->buflen + offset;
        break;

      default:
        return (off_t)ERROR;
    }

  /* Make sure that the new position is within range */

  if (newpos < 0 || newpos >= (off_t)mthis->buflen)
    {
      return (off_t)ERROR;
    }

  /* Return the new position */

  mthis->offset = (size_t)newpos;
  return newpos;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_memsostream
 *
 * Description:
 *   Initializes a stream for use with a fixed-size memory buffer.
 *
 * Input parameters:
 *   outstream - User allocated, uninitialized instance of struct
 *                  lib_memsostream_s to be initialized.
 *   bufstart     - Address of the beginning of the fixed-size memory buffer
 *   buflen       - Size of the fixed-sized memory buffer in bytes
 *
 * Returned Value:
 *   None (outstream initialized).
 *
 ****************************************************************************/

void lib_memsostream(FAR struct lib_memsostream_s *outstream,
                     FAR char *bufstart, int buflen)
{
  outstream->public.put   = memsostream_putc;
#ifdef CONFIG_STDIO_LINEBUFFER
  outstream->public.flush = lib_snoflush;
#endif
  outstream->public.seek  = memsostream_seek;
  outstream->public.nput  = 0;          /* Total number of characters written */
  outstream->buffer       = bufstart;   /* Start of buffer */
  outstream->offset       = 0;          /* Will be the buffer index */
  outstream->buflen       = buflen - 1; /* Save space for null terminator */
  outstream->buffer[0]    = '\0';       /* Start with an empty string */
}
