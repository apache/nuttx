/****************************************************************************
 * libs/libc/string/lib_strncat.c
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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifndef CONFIG_ARCH_STRNCAT
#undef strncat /* See mm/README.txt */
FAR char *strncat(FAR char *dest, FAR const char *src, size_t n)
{
  FAR char *ret = dest;

  dest  += strlen(dest);
  for (; n > 0 && *src != '\0' ; n--)
    {
      *dest++ = *src++;
    }

  *dest = '\0';

  return ret;
}
#endif
