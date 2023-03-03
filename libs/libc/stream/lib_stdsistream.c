/****************************************************************************
 * libs/libc/stream/lib_stdsistream.c
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

#include <assert.h>

#include "libc.h"

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
 * Name: stdsistream_gets
 ****************************************************************************/

static int stdsistream_gets(FAR struct lib_instream_s *this,
                            FAR void *buffer, int len)
{
  FAR struct lib_stdsistream_s *sthis = (FAR struct lib_stdsistream_s *)this;
  int nread = 0;

  DEBUGASSERT(this);

  /* Get the buffer from the incoming stream */

  nread = fread(buffer, len, 1, sthis->stream);
  if (nread >= 0)
    {
      this->nget += nread;
    }
  else
    {
      nread = _NX_GETERRVAL(nread);
    }

  return nread;
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
 * Input Parameters:
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
  instream->public.getc = stdsistream_getc;
  instream->public.gets = stdsistream_gets;
  instream->public.seek = stdsistream_seek;
  instream->public.nget = 0;
  instream->stream      = stream;
}
