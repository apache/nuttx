/****************************************************************************
 * libs/libc/stream/lib_stdsistream.c
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

#include <assert.h>

#include "libc.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stdsistream_getc
 ****************************************************************************/

static int stdsistream_getc(FAR struct lib_sistream_s *self)
{
  FAR struct lib_stdsistream_s *stream =
                                       (FAR struct lib_stdsistream_s *)self;
  int ret;

  DEBUGASSERT(self);

  /* Get the next character from the incoming file stream */

  ret = getc(stream->handle);
  if (ret != EOF)
    {
      self->nget++;
    }
  else
    {
      ret = _NX_GETERRVAL(ret);
    }

  return ret;
}

/****************************************************************************
 * Name: stdsistream_gets
 ****************************************************************************/

static ssize_t stdsistream_gets(FAR struct lib_instream_s *self,
                                FAR void *buffer, size_t len)
{
  FAR struct lib_stdsistream_s *stream =
                                        (FAR struct lib_stdsistream_s *)self;
  ssize_t nread = 0;

  DEBUGASSERT(self);

  /* Get the buffer from the incoming file stream */

  nread = fread(buffer, len, 1, stream->handle);
  if (nread >= 0)
    {
      self->nget += nread;
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

static off_t stdsistream_seek(FAR struct lib_sistream_s *self, off_t offset,
                              int whence)
{
  FAR struct lib_stdsistream_s *stream =
                                        (FAR struct lib_stdsistream_s *)self;

  DEBUGASSERT(self);
  offset = fseek(stream->handle, offset, whence);
  if (offset < 0)
    {
      offset = _NX_GETERRVAL(offset);
    }

  return offset;
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
 *   stream - User allocated, uninitialized instance of struct
 *            lib_stdsistream_s to be initialized.
 *   handle - User provided file instance (must have been opened for
 *            read access).
 *
 * Returned Value:
 *   None (User allocated instance initialized).
 *
 ****************************************************************************/

void lib_stdsistream(FAR struct lib_stdsistream_s *stream,
                     FAR FILE *handle)
{
  stream->common.getc = stdsistream_getc;
  stream->common.gets = stdsistream_gets;
  stream->common.seek = stdsistream_seek;
  stream->common.nget = 0;
  stream->handle      = handle;
}
