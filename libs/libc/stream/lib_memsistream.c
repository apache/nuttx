/****************************************************************************
 * libs/libc/stream/lib_memsistream.c
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
 * Name: meminstream_gets
 ****************************************************************************/

static int memsistream_gets(FAR struct lib_instream_s *this,
                            FAR void *buffer, int len)
{
  FAR struct lib_memsistream_s *mthis = (FAR struct lib_memsistream_s *)this;
  int ret;

  DEBUGASSERT(this);

  /* Get the buffer (if any) from the stream */

  if (this->nget < mthis->buflen)
    {
      ret = mthis->buflen - this->nget;
      ret = ret < len ? ret : len;
      this->nget += ret;
      memcpy(buffer, mthis->buffer, ret);
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
 * Input Parameters:
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
  instream->public.getc = memsistream_getc;
  instream->public.gets = memsistream_gets;
  instream->public.seek = memsistream_seek;
  instream->public.nget = 0;          /* Total number of characters read */
  instream->buffer      = bufstart;   /* Start of buffer */
  instream->offset      = 0;          /* Will be the buffer index */
  instream->buflen      = buflen;     /* Length of the buffer */
}
