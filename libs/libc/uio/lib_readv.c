/****************************************************************************
 * libs/libc/uio/lib_readv.c
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

#include <nuttx/cancelpt.h>

#include <sys/types.h>
#include <sys/uio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: readv()
 *
 * Description:
 *   The readv() function is equivalent to read(), except as described below.
 *   The readv() function places the input data into the 'iovcnt' buffers
 *   specified by the members of the 'iov' array: iov[0], iov[1], ...,
 *   iov['iovcnt'-1].  The 'iovcnt' argument is valid if greater than 0 and
 *   less than or equal to IOV_MAX as defined in limits.h.
 *
 *   Each iovec entry specifies the base address and length of an area in
 *   memory where data should be placed.  The readv() function will always
 *   fill an area completely before proceeding to the next.
 *
 *   TODO: pon successful completion, readv() will mark for update the
 *   st_atime field of the file.
 *
 * Input Parameters:
 *   filedes - The open file descriptor for the file to be read
 *   iov     - Array of read buffer descriptors
 *   iovcnt  - Number of elements in iov[]
 *
 * Returned Value:
 *   Upon successful completion, readv() will return a non-negative integer
 *   indicating the number of bytes actually read.  Otherwise, the functions
 *   will return -1 and set errno to indicate the error.  See read() for the
 *   list of returned errno values.  In addition, the readv() function will
 *   fail if:
 *
 *    EINVAL.
 *      The sum of the iov_len values in the iov array overflowed an ssize_t
 *      or The 'iovcnt' argument was less than or equal to 0, or greater than
 *      IOV_MAX (Not implemented).
 *
 ****************************************************************************/

ssize_t readv(int fildes, FAR const struct iovec *iov, int iovcnt)
{
  FAR uint8_t *buffer;
  ssize_t nread;
  size_t tocopy;
  size_t total_size;
  size_t offset;
  int i;

  if (iovcnt == 0)
    {
      return 0;
    }

  if (iovcnt == 1)
    {
      return read(fildes, iov->iov_base, iov->iov_len);
    }

  total_size = 0;
  for (i = 0; i < iovcnt; i++)
    {
      total_size += iov[i].iov_len;
    }

  if (total_size == 0)
    {
      return 0;
    }

  enter_cancellation_point();
  buffer = malloc(total_size);
  if (buffer == NULL)
    {
      return -1;
    }

  nread = read(fildes, buffer, total_size);
  if (nread == -1)
    {
      free(buffer);
      leave_cancellation_point();
      return nread;
    }

  tocopy = nread;
  offset = 0;
  for (i = 0; i < iovcnt; i++)
    {
      FAR uint8_t *p = iov[i].iov_base;
      size_t len = iov[i].iov_len;
      if (tocopy <= len)
        {
          memcpy(p, buffer + offset, tocopy);
          break;
        }

      memcpy(p, buffer + offset, len);
      offset += len;
    }

  free(buffer);
  leave_cancellation_point();
  return nread;
}
