/****************************************************************************
 * libs/libc/stream/lib_rawinstream.c
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
 * Name: rawinstream_getc
 ****************************************************************************/

static int rawinstream_getc(FAR struct lib_instream_s *this)
{
  FAR struct lib_rawinstream_s *rthis = (FAR struct lib_rawinstream_s *)this;
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
   * by a signal.  A Zero return value would indicate an end-of-file
   * condition.
   */

  return EOF;
}

/****************************************************************************
 * Name: rawinstream_getc
 ****************************************************************************/

static int rawinstream_gets(FAR struct lib_instream_s *this,
                            FAR void *buffer, int len)
{
  FAR struct lib_rawinstream_s *rthis = (FAR struct lib_rawinstream_s *)this;
  int nread;

  DEBUGASSERT(this && rthis->fd >= 0);

  /* Attempt to read one character */

  nread = _NX_READ(rthis->fd, buffer, len);
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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_rawinstream
 *
 * Description:
 *   Initializes a stream for use with a file descriptor.
 *
 * Input Parameters:
 *   instream - User allocated, uninitialized instance of struct
 *              lib_rawinstream_s to be initialized.
 *   fd       - User provided file/socket descriptor (must have been opened
 *              for the correct access).
 *
 * Returned Value:
 *   None (User allocated instance initialized).
 *
 ****************************************************************************/

void lib_rawinstream(FAR struct lib_rawinstream_s *instream, int fd)
{
  instream->public.getc = rawinstream_getc;
  instream->public.gets = rawinstream_gets;
  instream->public.nget = 0;
  instream->fd          = fd;
}
