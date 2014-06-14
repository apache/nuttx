/****************************************************************************
 * libc/stdio/lib_memsistream.c
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
 * Name: memsistream_getc
 ****************************************************************************/

static int memsistream_getc(FAR struct lib_sistream_s *this)
{
  FAR struct lib_memsistream_s *mthis = (FAR struct lib_memsistream_s *)this;
  int ret;

  DEBUGASSERT(this);

  /* Get the next character (if any) from the buffer */

  if (mthis->offset < mthis->buflen)
    {
      ret = mthis->buffer[mthis->offset];
      mthis->offset++;
      this->nget++;
    }
  else
    {
      ret = EOF;
    }

  return ret;
}

/****************************************************************************
 * Name: memsistream_seek
 ****************************************************************************/

static off_t memsistream_seek(FAR struct lib_sistream_s *this, off_t offset,
                              int whence)
{
  FAR struct lib_memsistream_s *mthis = (FAR struct lib_memsistream_s *)this;
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
 * Name: lib_memsistream
 *
 * Description:
 *   Initializes a stream for use with a fixed-size memory buffer.
 *
 * Input parameters:
 *   instream    - User allocated, uninitialized instance of struct
 *                 lib_memsistream_s to be initialized.
 *   bufstart    - Address of the beginning of the fixed-size memory buffer
 *   buflen      - Size of the fixed-sized memory buffer in bytes
 *
 * Returned Value:
 *   None (instream initialized).
 *
 ****************************************************************************/

void lib_memsistream(FAR struct lib_memsistream_s *instream,
                     FAR const char *bufstart, int buflen)
{
  instream->public.get  = memsistream_getc;
  instream->public.seek = memsistream_seek;
  instream->public.nget = 0;          /* Total number of characters read */
  instream->buffer      = bufstart;   /* Start of buffer */
  instream->offset      = 0;          /* Will be the buffer index */
  instream->buflen      = buflen;     /* Length of the buffer */
}
