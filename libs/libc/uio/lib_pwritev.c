/****************************************************************************
 * libs/libc/uio/lib_pwritev.c
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

#include <sys/types.h>
#include <sys/uio.h>
#include <unistd.h>
#include <errno.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pwritev()
 *
 * Description:
 *   The pwritev() function is equivalent to write(), except it takes
 *   an iov array.
 *
 ****************************************************************************/

ssize_t pwritev(int fildes, FAR const struct iovec *iov, int iovcnt,
                off_t offset)
{
  ssize_t ntotal;
  ssize_t nwritten;
  size_t remaining;
  FAR uint8_t *buffer;
  int i;

  /* Process each entry in the struct iovec array */

  for (i = 0, ntotal = 0; i < iovcnt; i++)
    {
      /* Ignore zero-length writes */

      if (iov[i].iov_len > 0)
        {
          buffer    = iov[i].iov_base;
          remaining = iov[i].iov_len;

          /* Write repeatedly as necessary to write the entire buffer */

          do
            {
              /* NOTE: pwrite() is a cancellation point */

              nwritten = pwrite(fildes, buffer, remaining, offset + ntotal);

              /* Check for a write error */

              if (nwritten < 0)
                {
                  return ntotal ? ntotal : ERROR;
                }

              /* Update pointers and counts in order to handle partial
               * buffer writes.
               */

              buffer    += nwritten;
              remaining -= nwritten;
              ntotal    += nwritten;
            }
          while (remaining > 0);
        }
    }

  return ntotal;
}
