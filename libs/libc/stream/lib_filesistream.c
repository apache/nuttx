/****************************************************************************
 * libs/libc/stream/lib_filesistream.c
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
 * Name: filesistream_gets
 ****************************************************************************/

static ssize_t filesistream_gets(FAR struct lib_sistream_s *self,
                                 FAR void *buf, size_t len)
{
  FAR struct lib_filesistream_s *stream =
                                (FAR struct lib_filesistream_s *)self;
  size_t left = len;
  ssize_t ret = 0;

  while (left >= 0)
    {
      ret = file_read(&stream->file, buf, left);
      if (ret == -EINTR)
        {
          continue;
        }
      else if (ret <= 0)
        {
          break;
        }

      self->nget += ret;
      buf += ret;
      left -= ret;
    }

  return left == len ? ret : len - left;
}

/****************************************************************************
 * Name: filesistream_getc
 ****************************************************************************/

static int filesistream_getc(FAR struct lib_sistream_s *self)
{
  unsigned char ch;
  return filesistream_gets(self, &ch, 1) == 1 ? ch : EOF;
}

/****************************************************************************
 * Name: filesistream_getc
 ****************************************************************************/

static off_t filesistream_seek(FAR struct lib_sistream_s *self, off_t offset,
                               int whence)
{
  FAR struct lib_filesistream_s *stream =
                                (FAR struct lib_filesistream_s *)self;

  return file_seek(&stream->file, offset, whence);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_filesistream_open
 *
 * Description:
 *   Initializes a stream for use with a file descriptor.
 *
 * Input Parameters:
 *   stream   - User allocated, uninitialized instance of struct
 *              lib_filesistream_s to be initialized.
 *
 * Returned Value:
 *   None (User allocated instance initialized).
 *
 ****************************************************************************/

int lib_filesistream_open(FAR struct lib_filesistream_s *stream,
                          FAR const char *path, int oflag, mode_t mode)
{
  int ret;

  ret = file_open(&stream->file, path, oflag, mode);
  if (ret < 0)
    {
      return ret;
    }

  stream->common.getc = filesistream_getc;
  stream->common.gets = filesistream_gets;
  stream->common.seek = filesistream_seek;
  stream->common.nget = 0;

  return 0;
}

/****************************************************************************
 * Name: lib_filesistream_close
 *
 * Description:
 *  Close the file associated with the stream.
 *
 * Input Parameters:
 *   stream   - User allocated, uninitialized instance of struct
 *              lib_filesistream_s to be initialized.
 *
 * Returned Value:
 *   None (User allocated instance initialized).
 *
 ****************************************************************************/

void lib_filesistream_close(FAR struct lib_filesistream_s *stream)
{
  if (stream != NULL)
    {
      file_close(&stream->file);
      stream->common.nget = 0;
    }
}
