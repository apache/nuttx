/****************************************************************************
 * libs/libc/stream/lib_stdinstream.c
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
#include <stdio.h>

#include "libc.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stdinstream_getc
 ****************************************************************************/

static int stdinstream_getc(FAR struct lib_instream_s *self)
{
  FAR struct lib_stdinstream_s *stream =
                                       (FAR struct lib_stdinstream_s *)self;
  int ret;

  DEBUGASSERT(self);

  /* Get the next character from the incoming stream */

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
 * Name: stdinstream_gets
 ****************************************************************************/

static ssize_t stdinstream_gets(FAR struct lib_instream_s *self,
                                FAR void *buffer, size_t len)
{
  FAR struct lib_stdinstream_s *stream =
                                       (FAR struct lib_stdinstream_s *)self;
  ssize_t nread = 0;

  DEBUGASSERT(self);

  /* Get the buffer from the incoming stream handle */

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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_stdinstream
 *
 * Description:
 *   Initializes a stream for use with a FILE instance.
 *
 * Input Parameters:
 *   instream - User allocated, uninitialized instance of struct
 *              lib_stdinstream_s to be initialized.
 *   handle   - User provided handle instance (must have been opened for
 *              read access).
 *
 * Returned Value:
 *   None (User allocated instance initialized).
 *
 ****************************************************************************/

void lib_stdinstream(FAR struct lib_stdinstream_s *stream,
                     FAR FILE *handle)
{
  stream->common.getc = stdinstream_getc;
  stream->common.gets = stdinstream_gets;
  stream->common.nget = 0;
  stream->handle      = handle;
}
