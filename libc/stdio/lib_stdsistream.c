/****************************************************************************
 * libc/stdio/lib_stdsistream.c
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
 * Name: stdsistream_getc
 ****************************************************************************/

static int stdsistream_getc(FAR struct lib_sistream_s *this)
{
  FAR struct lib_stdsistream_s *sthis = (FAR struct lib_stdsistream_s *)this;
  int ret;

  DEBUGASSERT(this);

  /* Get the next character from the incoming stream */

  ret = getc(sthis->stream);
  if (ret != EOF)
    {
      this->nget++;
    }

 return ret;
}

/****************************************************************************
 * Name: stdsistream_seek
 ****************************************************************************/

static off_t stdsistream_seek(FAR struct lib_sistream_s *this, off_t offset,
                              int whence)
{
  FAR struct lib_stdsistream_s *mthis = (FAR struct lib_stdsistream_s *)this;

  DEBUGASSERT(this);
  return fseek(mthis->stream, offset, whence);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_stdsistream
 *
 * Description:
 *   Initializes a stream for use with a FILE instance.
 *
 * Input parameters:
 *   instream - User allocated, uninitialized instance of struct
 *              lib_stdsistream_s to be initialized.
 *   stream   - User provided stream instance (must have been opened for
 *              read access).
 *
 * Returned Value:
 *   None (User allocated instance initialized).
 *
 ****************************************************************************/

void lib_stdsistream(FAR struct lib_stdsistream_s *instream,
                     FAR FILE *stream)
{
  instream->public.get  = stdsistream_getc;
  instream->public.seek = stdsistream_seek;
  instream->public.nget = 0;
  instream->stream      = stream;
}
