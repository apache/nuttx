/****************************************************************************
 * libs/libc/unistd/lib_copy_file_range.c
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

#include <sys/types.h>
#include <errno.h>
#include <unistd.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define COPY_BUFSIZE 512

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: copy_file_range
 *
 * Description:
 *   Copy up to len bytes from the file infd to the file outfd.  If inoff
 *   is NULL, the data is read at the file offset of infd, which advances;
 *   if not, it is read at *inoff, which advances, and the file offset of
 *   infd does not change.  outoff works the same way for outfd.
 *
 * Input Parameters:
 *   infd   - The file to read
 *   inoff  - The offset to read at, or NULL
 *   outfd  - The file to write
 *   outoff - The offset to write at, or NULL
 *   len    - The number of bytes to copy
 *   flags  - Must be zero
 *
 * Returned Value:
 *   The number of bytes copied, which can be less than len, or zero at the
 *   end of infd.  On failure, -1 (ERROR) with errno set.
 *
 ****************************************************************************/

ssize_t copy_file_range(int infd, FAR off_t *inoff, int outfd,
                        FAR off_t *outoff, size_t len, unsigned int flags)
{
  char buf[COPY_BUFSIZE];
  ssize_t total = 0;

  if (flags != 0)
    {
      set_errno(EINVAL);
      return ERROR;
    }

  while (len > 0)
    {
      size_t size = len < sizeof(buf) ? len : sizeof(buf);
      ssize_t nread;
      ssize_t done = 0;

      nread = inoff ? pread(infd, buf, size, *inoff) :
                      read(infd, buf, size);
      if (nread <= 0)
        {
          return nread < 0 && total == 0 ? ERROR : total;
        }

      while (done < nread)
        {
          ssize_t nwritten;

          nwritten = outoff ? pwrite(outfd, buf + done, nread - done,
                                     *outoff) :
                              write(outfd, buf + done, nread - done);
          if (nwritten <= 0)
            {
              /* Leave the input after the last byte written */

              if (inoff)
                {
                  *inoff += done;
                }
              else
                {
                  lseek(infd, done - nread, SEEK_CUR);
                }

              return total == 0 && done == 0 ? ERROR : total + done;
            }

          if (outoff)
            {
              *outoff += nwritten;
            }

          done += nwritten;
        }

      if (inoff)
        {
          *inoff += nread;
        }

      total += nread;
      len   -= nread;

      if ((size_t)nread < size)
        {
          break;
        }
    }

  return total;
}
