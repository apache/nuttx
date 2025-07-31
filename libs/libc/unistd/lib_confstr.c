/****************************************************************************
 * libs/libc/unistd/lib_confstr.c
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

#include <unistd.h>
#include <errno.h>
#include <string.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: confstr
 *
 * Description:
 *   The confstr() function shall return the value of configuration-defined
 *   string variables associated with the name argument.
 *
 * Returned Value:
 *   If name is an invalid value, confstr() shall return 0 and set errno to
 *   indicate the error.
 *
 *   If name doesn't have a configuration-defined value, confstr() returns 0
 *   and leaves errno unchanged.
 *
 *   If name has a configuration-defined value, confstr() returns the size
 *   of the buffer that would be needed to hold the entire string value.
 *
 ****************************************************************************/

size_t confstr(int name, FAR char *buf, size_t len)
{
  FAR const char *p;

  switch (name)
    {
      /* System-defined default PATH */

      case _CS_PATH:
        {
#ifdef CONFIG_PATH_INITIAL
          p = CONFIG_PATH_INITIAL;
#else
          p = "\0";
#endif
          break;
        }

     /* Programming environment is not supported on NuttX, so all
      * _CS_POSIX_V6 configuration variables are empty.
      */

      case _CS_POSIX_V6_ILP32_OFF32_CFLAGS:
      case _CS_POSIX_V6_ILP32_OFF32_LDFLAGS:
      case _CS_POSIX_V6_ILP32_OFF32_LIBS:
      case _CS_POSIX_V6_ILP32_OFFBIG_CFLAGS:
      case _CS_POSIX_V6_ILP32_OFFBIG_LDFLAGS:
      case _CS_POSIX_V6_ILP32_OFFBIG_LIBS:
      case _CS_POSIX_V6_LP64_OFF64_CFLAGS:
      case _CS_POSIX_V6_LP64_OFF64_LDFLAGS:
      case _CS_POSIX_V6_LP64_OFF64_LIBS:
      case _CS_POSIX_V6_LPBIG_OFFBIG_CFLAGS:
      case _CS_POSIX_V6_LPBIG_OFFBIG_LDFLAGS:
      case _CS_POSIX_V6_LPBIG_OFFBIG_LIBS:
      case _CS_POSIX_V6_WIDTH_RESTRICTED_ENVS:
        {
          p = "\0";
          break;
        }

      default:
        {
          set_errno(EINVAL);
          return 0;
        }
    }

  if (len != 0 && buf != NULL)
    {
      strlcpy(buf, p, len);
    }

  return (strlen(p) + 1);
}
