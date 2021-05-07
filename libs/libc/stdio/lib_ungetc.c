/****************************************************************************
 * libs/libc/stdio/lib_ungetc.c
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
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <nuttx/fs/fs.h>
#include "libc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ungetc
 ****************************************************************************/

int ungetc(int c, FAR FILE *stream)
{
#if CONFIG_NUNGET_CHARS > 0
  int nungotten;
#endif

  /* Verify that a non-NULL stream was provided */

  if (!stream)
    {
      set_errno(EBADF);
      return EOF;
    }

  /* Stream must be open for read access */

  if ((stream->fs_fd < 0) || ((stream->fs_oflags & O_RDOK) == 0))
    {
      set_errno(EBADF);
      return EOF;
    }

#if CONFIG_NUNGET_CHARS > 0
  nungotten = stream->fs_nungotten;
  if (stream->fs_nungotten < CONFIG_NUNGET_CHARS)
    {
      stream->fs_ungotten[nungotten] = c;
      stream->fs_nungotten = nungotten + 1;
      return c;
    }
  else
#endif
    {
      set_errno(ENOMEM);
      return EOF;
    }
}
