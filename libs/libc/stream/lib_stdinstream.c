/****************************************************************************
 * libs/libc/stream/lib_stdinstream.c
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
 * Name: stdinstream_getc
 ****************************************************************************/

static int stdinstream_getc(FAR struct lib_instream_s *this)
{
  FAR struct lib_stdinstream_s *sthis = (FAR struct lib_stdinstream_s *)this;
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
 *   stream   - User provided stream instance (must have been opened for
 *              read access).
 *
 * Returned Value:
 *   None (User allocated instance initialized).
 *
 ****************************************************************************/

void lib_stdinstream(FAR struct lib_stdinstream_s *instream,
                     FAR FILE *stream)
{
  instream->public.get  = stdinstream_getc;
  instream->public.nget = 0;
  instream->stream      = stream;
}
