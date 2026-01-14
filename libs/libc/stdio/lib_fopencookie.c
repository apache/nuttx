/****************************************************************************
 * libs/libc/stdio/lib_fopencookie.c
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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <nuttx/tls.h>

#include "libc.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cookie_read_cb
 ****************************************************************************/

static ssize_t cookie_read_cb(FAR void *cookie, FAR char *buf, size_t size)
{
  /* Per specification: if *read is a null pointer, then reads from the
   * custom stream always return end of file.
   */

  return 0;
}

/****************************************************************************
 * Name: cookie_write_cb
 ****************************************************************************/

static ssize_t cookie_write_cb(FAR void *cookie, FAR const char *buf,
                               size_t size)
{
  /* Per specification: if *write is a null pointer, then output to the
   * stream is discarded.
   */

  return size;
}

/****************************************************************************
 * Name: cookie_seek_cb
 ****************************************************************************/

static off_t cookie_seek_cb(FAR void *cookie, FAR off_t *offset, int whence)
{
  /* Per specification: if *seek is a null pointer, then it is not
   * possible to perform seek operations on the stream.
   */

  set_errno(ENOTSUP);
  return -1;
}

/****************************************************************************
 * Name: cookie_close_cb
 ****************************************************************************/

static int cookie_close_cb(FAR void *cookie)
{
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fopencookie
 ****************************************************************************/

FAR FILE *fopencookie(FAR void *cookie, FAR const char *mode,
                      cookie_io_functions_t io_funcs)
{
  FAR FILE *filep = NULL;

  /* Call fdopen to initialize the stream. The function is called with
   * first possible file descriptor (0-2 are reserved). This is only
   * to pass the required checks, file descriptor is then rewritten to
   * cookie.
   */

  filep = fdopen(3, mode);
  if (filep == NULL)
    {
      return NULL;
    }

  /* Assign cookie to file descriptor (fs_cookie) and assign user
   * defined callbacks.
   */

  filep->fs_cookie = cookie;
  filep->fs_iofunc = io_funcs;

  /* Fopencookie Linux specification allows cookie_io_functions_t to be
   * filled only partially and thus some callbacks might be NULL and not
   * used. For this reason we add internal callbacks that are assigned to
   * prevent undefined behaviour and handle correct return value per
   * fopencookie specification.
   */

  if (filep->fs_iofunc.read == NULL)
    {
      filep->fs_iofunc.read = cookie_read_cb;
    }

  if (filep->fs_iofunc.write == NULL)
    {
      filep->fs_iofunc.write = cookie_write_cb;
    }

  if (filep->fs_iofunc.seek == NULL)
    {
      filep->fs_iofunc.seek = cookie_seek_cb;
    }

  if (filep->fs_iofunc.close == NULL)
    {
      filep->fs_iofunc.close = cookie_close_cb;
    }

  return filep;
}
