/****************************************************************************
 * libs/libc/machine/risc-v/arch_strlcpy.c
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

#include <string.h>

#include "arch_word.h"
#include "libc.h"

#ifdef CONFIG_LIBC_ARCH_STRLCPY

/****************************************************************************
 * Public Functions
 ****************************************************************************/

size_t strlcpy(FAR char *dst, FAR const char *src, size_t dsize)
{
  FAR const char *s = src;
  FAR char *d = dst;
  size_t remain = dsize;

  /* Words while the pointers agree about boundaries, there is room for a
   * whole word plus the terminator, and no byte of the word is the
   * terminator.  The cap keeps a byte back so the terminator always
   * fits.
   */

  if ((((uintptr_t)d ^ (uintptr_t)s) & (WORD_BYTES - 1)) == 0 &&
      remain > WORD_BYTES)
    {
      while (!WORD_ALIGNED(s) && remain > 1 && *s != '\0')
        {
          *d++ = *s++;
          remain--;
        }

      while (remain > WORD_BYTES &&
             !WORD_HASZERO(*(FAR const WORD_T *)s))
        {
          *(FAR WORD_T *)d = *(FAR const WORD_T *)s;
          d += WORD_BYTES;
          s += WORD_BYTES;
          remain -= WORD_BYTES;
        }
    }

  while (remain > 1 && *s != '\0')
    {
      *d++ = *s++;
      remain--;
    }

  if (remain > 0)
    {
      *d = '\0';
    }

  /* strlcpy returns the source length regardless of how much was copied,
   * which is how the caller detects truncation.  The remainder is a plain
   * strlen, already the fast one.
   */

  return (s - src) + strlen(s);
}

#endif /* CONFIG_LIBC_ARCH_STRLCPY */
