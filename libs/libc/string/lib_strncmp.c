/****************************************************************************
 * libs/libc/string/lib_strncmp.c
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

#ifndef CONFIG_LIBC_ARCH_STRNCMP
#undef strncmp /* See mm/README.txt */
int strncmp(FAR const char *cs, FAR const char *ct, size_t nb)
{
  register int result = 0;
  for (; nb > 0; nb--)
    {
      if ((result = (unsigned char)*cs - (unsigned char)*ct++) != 0 ||
          *cs++ == '\0')
        {
          break;
        }
    }

  return result;
}
#endif
