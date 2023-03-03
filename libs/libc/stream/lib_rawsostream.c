/****************************************************************************
 * libs/libc/stream/lib_rawsostream.c
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

#include <nuttx/config.h>

#include <unistd.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/fs/fs.h>

#include "libc.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rawsostream_putc
 ****************************************************************************/

static void rawsostream_putc(FAR struct lib_sostream_s *this, int ch)
{
  FAR struct lib_rawsostream_s *rthis = (FAR struct lib_rawsostream_s *)this;
  char buffer = ch;
  int nwritten;

  DEBUGASSERT(this && rthis->fd >= 0);

  /* Loop until the character is successfully transferred or until an
   * irrecoverable error occurs.
   */

  do
    {
      nwritten = _NX_WRITE(rthis->fd, &buffer, 1);
      if (nwritten == 1)
        {
          this->nput++;
          return;
        }

      /* The only expected error is EINTR, meaning that the write operation
       * was awakened by a signal.  Zero would not be a valid return value
       * from _NX_WRITE().
       */

      nwritten = _NX_GETERRVAL(nwritten);
      DEBUGASSERT(nwritten < 0);
    }
  while (nwritten == -EINTR);
}

/****************************************************************************
 * Name: rawsostream_puts
 ****************************************************************************/

static int rawsostream_puts(FAR struct lib_sostream_s *this,
                            FAR const void *buffer, int len)
{
  FAR struct lib_rawsostream_s *rthis = (FAR struct lib_rawsostream_s *)this;
  int nwritten;

  DEBUGASSERT(this && rthis->fd >= 0);

  /* Loop until the buffer is successfully transferred or until an
   * irrecoverable error occurs.
   */

  do
    {
      nwritten = _NX_WRITE(rthis->fd, buffer, len);
      if (nwritten >= 0)
        {
          this->nput += nwritten;
          return nwritten;
        }

      /* The only expected error is EINTR, meaning that the write operation
       * was awakened by a signal.  Zero would not be a valid return value
       * from _NX_WRITE().
       */

      nwritten = _NX_GETERRVAL(nwritten);
      DEBUGASSERT(nwritten < 0);
    }
  while (nwritten == -EINTR);

  return nwritten;
}

/****************************************************************************
 * Name: rawsostream_seek
 ****************************************************************************/

static off_t rawsostream_seek(FAR struct lib_sostream_s *this, off_t offset,
                              int whence)
{
  FAR struct lib_rawsostream_s *mthis = (FAR struct lib_rawsostream_s *)this;

  DEBUGASSERT(this);
  return _NX_SEEK(mthis->fd, offset, whence);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_rawsostream
 *
 * Description:
 *   Initializes a stream for use with a file descriptor.
 *
 * Input Parameters:
 *   outstream - User allocated, uninitialized instance of struct
 *               lib_rawsostream_s to be initialized.
 *   fd        - User provided file/socket descriptor (must have been opened
 *               for write access).
 *
 * Returned Value:
 *   None (User allocated instance initialized).
 *
 ****************************************************************************/

void lib_rawsostream(FAR struct lib_rawsostream_s *outstream, int fd)
{
  outstream->public.putc  = rawsostream_putc;
  outstream->public.puts  = rawsostream_puts;
  outstream->public.flush = lib_snoflush;
  outstream->public.seek  = rawsostream_seek;
  outstream->public.nput  = 0;
  outstream->fd           = fd;
}
