/****************************************************************************
 * libs/libc/stream/lib_meminstream.c
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
 * Name: meminstream_getc
 ****************************************************************************/

static int meminstream_getc(FAR struct lib_instream_s *this)
{
  FAR struct lib_meminstream_s *mthis = (FAR struct lib_meminstream_s *)this;
  int ret;

  DEBUGASSERT(this);

  /* Get the next character (if any) from the buffer */

  if (this->nget < mthis->buflen)
    {
      ret = mthis->buffer[this->nget];
      this->nget++;
    }
  else
    {
      ret = EOF;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_meminstream
 *
 * Description:
 *   Initializes a stream for use with a fixed-size memory buffer.
 *
 * Input Parameters:
 *   instream    - User allocated, uninitialized instance of struct
 *                 lib_meminstream_s to be initialized.
 *   bufstart    - Address of the beginning of the fixed-size memory buffer
 *   buflen      - Size of the fixed-sized memory buffer in bytes
 *
 * Returned Value:
 *   None (instream initialized).
 *
 ****************************************************************************/

void lib_meminstream(FAR struct lib_meminstream_s *instream,
                     FAR const char *bufstart, int buflen)
{
  instream->public.get  = meminstream_getc;
  instream->public.nget = 0;          /* Will be buffer index */
  instream->buffer      = bufstart;   /* Start of buffer */
  instream->buflen      = buflen;     /* Length of the buffer */
}
