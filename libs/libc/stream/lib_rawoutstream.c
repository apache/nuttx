/****************************************************************************
 * libs/libc/stream/lib_rawoutstream.c
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
 * Name: rawoutstream_puts
 ****************************************************************************/

static int rawoutstream_puts(FAR struct lib_outstream_s *this,
                             FAR const void *buf, int len)
{
  FAR struct lib_rawoutstream_s *rthis =
                                (FAR struct lib_rawoutstream_s *)this;
  int nwritten = 0;

  do
    {
      nwritten = _NX_WRITE(rthis->fd, buf, len);
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
 * Name: rawoutstream_putc
 ****************************************************************************/

static void rawoutstream_putc(FAR struct lib_outstream_s *this, int ch)
{
  char tmp = ch;
  rawoutstream_puts(this, &tmp, 1);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_rawoutstream
 *
 * Description:
 *   Initializes a stream for use with a file descriptor.
 *
 * Input Parameters:
 *   outstream - User allocated, uninitialized instance of struct
 *               lib_rawoutstream_s to be initialized.
 *   fd        - User provided file/socket descriptor (must have been opened
 *               for write access).
 *
 * Returned Value:
 *   None (User allocated instance initialized).
 *
 ****************************************************************************/

void lib_rawoutstream(FAR struct lib_rawoutstream_s *outstream, int fd)
{
  outstream->public.putc  = rawoutstream_putc;
  outstream->public.puts  = rawoutstream_puts;
  outstream->public.flush = lib_noflush;
  outstream->public.nput  = 0;
  outstream->fd           = fd;
}
