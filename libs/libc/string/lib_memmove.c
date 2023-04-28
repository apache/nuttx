/****************************************************************************
 * libs/libc/string/lib_memmove.c
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

#if !defined(CONFIG_LIBC_ARCH_MEMMOVE) && defined(LIBC_BUILD_STRING)
#undef memmove /* See mm/README.txt */
no_builtin("memmove")
FAR void *memmove(FAR void *dest, FAR const void *src, size_t count)
{
  FAR char *tmp;
  FAR char *s;

  if (dest <= src)
    {
      tmp = (FAR char *) dest;
      s   = (FAR char *) src;

      while (count--)
        {
          *tmp++ = *s++;
        }
    }
  else
    {
      tmp = (FAR char *) dest + count;
      s   = (FAR char *) src + count;

      while (count--)
        {
          *--tmp = *--s;
        }
    }

  return dest;
}
#endif
