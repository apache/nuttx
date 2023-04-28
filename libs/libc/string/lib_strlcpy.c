/****************************************************************************
 * libs/libc/string/lib_strlcpy.c
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
#include <string.h>

#include "libc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: strlcpy
 *
 * Description:
 *   Copy src to string dst of size dsize.  At most dsize-1 characters
 *   will be copied.  Always NUL terminates (unless dsize == 0).
 *
 * Returned Value:
 *   Returns strlen(src); if retval >= dsize, truncation occurred.
 *
 ****************************************************************************/

#if !defined(CONFIG_LIBC_ARCH_STRLCPY) && defined(LIBC_BUILD_STRING)
size_t strlcpy(FAR char *dst, FAR const char *src, size_t dsize)
{
  FAR const char *osrc = src;
  size_t nleft = dsize;

  if (nleft != 0)
    {
      while (--nleft != 0)
        {
          if ((*dst++ = *src++) == '\0')
            {
              break;
            }
        }
    }

  if (nleft == 0)
    {
      if (dsize != 0)
        {
          *dst = '\0';
        }

      while (*src++);
    }

  return (src - osrc - 1);
}
#endif
