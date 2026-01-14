/****************************************************************************
 * libs/libc/stream/lib_fileinstream.c
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

#include <nuttx/config.h>

#include <errno.h>

#include <nuttx/fs/fs.h>

#include "libc.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fileinstream_gets
 ****************************************************************************/

static ssize_t fileinstream_gets(FAR struct lib_instream_s *self,
                                 FAR void *buf, size_t len)
{
  FAR struct lib_fileinstream_s *stream =
                                (FAR struct lib_fileinstream_s *)self;
  ssize_t nread;

  do
    {
      nread = file_read(&stream->file, buf, len);
    }
  while (nread == -EINTR);

  if (nread >= 0)
    {
      self->nget += nread;
    }

  return nread;
}

/****************************************************************************
 * Name: fileinstream_getc
 ****************************************************************************/

static int fileinstream_getc(FAR struct lib_instream_s *self)
{
  unsigned char ch;
  return fileinstream_gets(self, &ch, 1) == 1 ? ch : EOF;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_fileinstream_open
 *
 * Description:
 *   Initializes a stream for use with a file descriptor.
 *
 * Input Parameters:
 *   stream   - User allocated, uninitialized instance of struct
 *              lib_fileinstream_s to be initialized.
 *
 * Returned Value:
 *   None (User allocated instance initialized).
 *
 ****************************************************************************/

int lib_fileinstream_open(FAR struct lib_fileinstream_s *stream,
                          FAR const char *path, int oflag, mode_t mode)
{
  int ret;

  ret = file_open(&stream->file, path, oflag, mode);
  if (ret < 0)
    {
      return ret;
    }

  stream->common.getc = fileinstream_getc;
  stream->common.gets = fileinstream_gets;
  stream->common.nget = 0;

  return 0;
}

/****************************************************************************
 * Name: lib_fileinstream_close
 *
 * Description:
 *  Close the file associated with the stream.
 *
 * Input Parameters:
 *   stream   - User allocated, uninitialized instance of struct
 *              lib_fileinstream_s to be initialized.
 *
 * Returned Value:
 *   None (User allocated instance initialized).
 *
 ****************************************************************************/

void lib_fileinstream_close(FAR struct lib_fileinstream_s *stream)
{
  if (stream != NULL)
    {
      file_close(&stream->file);
      stream->common.nget = 0;
    }
}
