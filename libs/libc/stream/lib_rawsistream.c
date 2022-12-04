/****************************************************************************
 * libs/libc/stream/lib_rawsistream.c
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
 * Name: rawsistream_getc
 ****************************************************************************/

static int rawsistream_getc(FAR struct lib_sistream_s *this)
{
  FAR struct lib_rawsistream_s *rthis = (FAR struct lib_rawsistream_s *)this;
  int nread;
  char ch;

  DEBUGASSERT(this && rthis->fd >= 0);

  /* Attempt to read one character */

  nread = _NX_READ(rthis->fd, &ch, 1);
  if (nread == 1)
    {
      this->nget++;
      return ch;
    }

  /* Return EOF on any failure to read from the incoming byte stream. The
   * only expected error is EINTR meaning that the read was interrupted
   * by a signal.  A Zero return value would indicated an end-of-file
   * confition.
   */

  return EOF;
}

/****************************************************************************
 * Name: rawsistream_seek
 ****************************************************************************/

static off_t rawsistream_seek(FAR struct lib_sistream_s *this, off_t offset,
                              int whence)
{
  FAR struct lib_rawsistream_s *mthis = (FAR struct lib_rawsistream_s *)this;

  DEBUGASSERT(this);
  return _NX_SEEK(mthis->fd, offset, whence);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_rawsistream
 *
 * Description:
 *   Initializes a stream for use with a file descriptor.
 *
 * Input Parameters:
 *   instream - User allocated, uninitialized instance of struct
 *              lib_rawsistream_s to be initialized.
 *   fd       - User provided file/socket descriptor (must have been opened
 *              for the correct access).
 *
 * Returned Value:
 *   None (User allocated instance initialized).
 *
 ****************************************************************************/

void lib_rawsistream(FAR struct lib_rawsistream_s *instream, int fd)
{
  instream->public.getc = rawsistream_getc;
  instream->public.seek = rawsistream_seek;
  instream->public.nget = 0;
  instream->fd          = fd;
}
