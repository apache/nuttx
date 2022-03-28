/****************************************************************************
 * libs/libc/unistd/lib_getentropy.c
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

#include <sys/random.h>
#include <errno.h>
#include <unistd.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getentropy
 *
 * Description:
 *   The getentropy() function writes length bytes of high-quality
 *   random data to the buffer starting at the location pointed to by
 *   buffer. The maximum permitted value for the length argument is
 *   256.
 *
 *   A successful call to getentropy() always provides the requested
 *   number of bytes of entropy.
 *
 * Input Parameters:
 *   buffer - Buffer for returned random bytes
 *   length - Number of bytes requested.
 *
 * Returned Value:
 *   On success, this function returns zero.  On error, -1 is
 *   returned, and errno is set to indicate the error.
 *
 ****************************************************************************/

int getentropy(FAR void *buffer, size_t length)
{
  FAR char *pos = buffer;

  if (length > 256)
    {
      errno = EIO;
      return -1;
    }

  while (length > 0)
    {
      int ret = getrandom(pos, length, 0);
      if (ret < 0)
        {
          if (errno == EINTR)
            {
              continue;
            }

          return ret;
        }

      pos += ret;
      length -= ret;
    }

  return 0;
}
