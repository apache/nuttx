/****************************************************************************
 * libs/libc/uio/lib_writev.c
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
 * Name: writev()
 *
 * Description:
 *   The writev() function is equivalent to write(), except as described
 *   below. The writev() function will gather output data from the 'iovcnt'
 *   buffers specified by the members of the 'iov' array:
 *   iov[0], iov[1], ..., iov[iovcnt-1].
 *   The 'iovcnt' argument is valid if greater than 0 and less than or equal
 *   to IOV_MAX, as defined in limits.h.
 *
 *   Each iovec entry specifies the base address and length of an area in
 *   memory from which data should be written. The writev() function always
 *   writes a complete area before proceeding to the next.
 *
 *   If 'filedes' refers to a regular file and all of the iov_len members in
 *   the array pointed to by iov are 0, writev() will return 0 and have no
 *   other effect. For other file types, the behavior is unspecified.
 *
 *   TODO: If the sum of the iov_len values is greater than SSIZE_MAX, the
 *   operation will fail and no data will be transferred.
 *
 * Input Parameters:
 *   filedes - The open file descriptor for the file to be write
 *   iov     - Array of write buffer descriptors
 *   iovcnt  - Number of elements in iov[]
 *
 * Returned Value:
 *   Upon successful completion, writev() shall return the number of bytes
 *   actually written. Otherwise, it shall return a value of -1, the file-
 *   pointer shall remain unchanged, and errno shall be set to indicate an
 *   error. See write for the list of returned errno values. In addition,
 *   the writev() function will fail if:
 *
 *    EINVAL.
 *      The sum of the iov_len values in the iov array overflowed an ssize_t
 *      or The 'iovcnt' argument was less than or equal to 0, or greater than
 *      IOV_MAX (Not implemented).
 *
 ****************************************************************************/

ssize_t writev(int fildes, FAR const struct iovec *iov, int iovcnt)
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
              /* NOTE:  write() is a cancellation point */

              nwritten = write(fildes, buffer, remaining);

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
