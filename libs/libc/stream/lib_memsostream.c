/****************************************************************************
 * libs/libc/stream/lib_memsostream.c
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
#include <errno.h>

#include "libc.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: memsostream_putc
 ****************************************************************************/

static void memsostream_putc(FAR struct lib_sostream_s *self, int ch)
{
  FAR struct lib_memsostream_s *stream =
                                       (FAR struct lib_memsostream_s *)self;

  DEBUGASSERT(self);

  /* If this will not overrun the buffer, then write the character to the
   * buffer.  Note that buflen was pre-decremented when the stream was
   * created so it is okay to write past the end of the buflen by one.
   */

  if (stream->offset < stream->buflen)
    {
      stream->buffer[stream->offset] = ch;
      stream->offset++;
      self->nput++;
      stream->buffer[stream->offset] = '\0';
    }
}

/****************************************************************************
 * Name: memoutstream_puts
 ****************************************************************************/

static ssize_t memsostream_puts(FAR struct lib_sostream_s *self,
                                FAR const void *buf, size_t len)
{
  ssize_t ncopy;
  FAR struct lib_memsostream_s *stream =
                                       (FAR struct lib_memsostream_s *)self;

  DEBUGASSERT(self);

  ncopy = stream->offset + len + 1 < stream->buflen ? len :
          stream->buflen - stream->offset - 1;
  if (ncopy > 0)
    {
      memcpy(stream->buffer + stream->offset, buf, ncopy);
      stream->common.nput += ncopy;
      stream->offset += ncopy;
      stream->buffer[stream->offset] = '\0';
    }

  return ncopy;
}

/****************************************************************************
 * Name: memsostream_seek
 ****************************************************************************/

static off_t memsostream_seek(FAR struct lib_sostream_s *self, off_t offset,
                              int whence)
{
  FAR struct lib_memsostream_s *stream =
                                       (FAR struct lib_memsostream_s *)self;
  off_t newpos;

  DEBUGASSERT(self);

  switch (whence)
    {
      case SEEK_CUR:
        newpos = stream->offset + offset;
        break;

      case SEEK_SET:
        newpos = offset;
        break;

      case SEEK_END:
        newpos = stream->buflen + offset;
        break;

      default:
        return -EINVAL;
    }

  /* Make sure that the new position is within range */

  if (newpos < 0 || newpos >= stream->buflen)
    {
      return -EINVAL;
    }

  /* Return the new position */

  stream->offset = newpos;
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
 * Input Parameters:
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
                     FAR char *bufstart, size_t buflen)
{
  outstream->common.putc  = memsostream_putc;
  outstream->common.puts  = memsostream_puts;
  outstream->common.flush = lib_snoflush;
  outstream->common.seek  = memsostream_seek;
  outstream->common.nput  = 0;          /* Total number of characters written */
  outstream->buffer       = bufstart;   /* Start of buffer */
  outstream->offset       = 0;          /* Will be the buffer index */
  outstream->buflen       = buflen - 1; /* Save space for null terminator */
  outstream->buffer[0]    = '\0';       /* Start with an empty string */
}
