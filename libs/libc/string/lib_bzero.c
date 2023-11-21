/****************************************************************************
 * libs/libc/string/lib_bzero.c
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
#include <strings.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bzero
 ****************************************************************************/

void bzero(FAR void *s, size_t n)
{
#ifdef CONFIG_HOST_MACOS

  /* When macos compiles bzero, it thinks it is equivalent to
   * meset(s, 0, n), so recursion occurs.
   */

  FAR char *dst = s;
  size_t i;

  for (i = 0; i < n; i++)
    {
      dst[i] = 0;
    }

#else
  memset(s, 0, n);
#endif
}
