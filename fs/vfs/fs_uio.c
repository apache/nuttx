/****************************************************************************
 * fs/vfs/fs_uio.c
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

#include <sys/uio.h>
#include <sys/types.h>

#include <nuttx/fs/fs.h>

#include <assert.h>
#include <errno.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: uio_total_len
 *
 * Description:
 *   Return the total length of data in bytes.
 *   Or -EOVERFLOW.
 *
 ****************************************************************************/

ssize_t uio_total_len(FAR const struct uio *uio)
{
  const struct iovec *iov = uio->uio_iov;
  int iovcnt = uio->uio_iovcnt;
  size_t len = 0;
  int i;

  for (i = 0; i < iovcnt; i++)
    {
      if (SSIZE_MAX - len < iov[i].iov_len)
        {
          return -EOVERFLOW;
        }

      len += iov[i].iov_len;
    }

  return len;
}
