/****************************************************************************
 * libs/libc/machine/risc-v/arch_memmove.c
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

#ifdef CONFIG_LIBC_ARCH_MEMMOVE

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR void *memmove(FAR void *dest, FAR const void *src, size_t count)
{
  FAR unsigned char *d = dest;
  FAR const unsigned char *s = src;

  if (d == s || count == 0)
    {
      return dest;
    }

  /* Copying forward is memcpy's job, and safe whenever the start of the
   * destination is below the source: by the time a source byte could be
   * overwritten it has already been read.
   */

  if (d < s)
    {
      return memcpy(dest, src, count);
    }

  /* Backward, from the top.  Registers at a time when the two ends sit
   * the same distance past a boundary, bytes otherwise: the mismatched
   * case is rare enough for a backward overlapping copy that it is not
   * worth a shifting loop of its own.
   */

  d += count;
  s += count;

  if ((((uintptr_t)d ^ (uintptr_t)s) & (WORD_BYTES - 1)) == 0 &&
      count >= 2 * WORD_BYTES)
    {
      while (!WORD_ALIGNED(d))
        {
          *--d = *--s;
          count--;
        }

      while (count >= WORD_BYTES)
        {
          d -= WORD_BYTES;
          s -= WORD_BYTES;
          count -= WORD_BYTES;
          *(FAR WORD_T *)d = *(FAR const WORD_T *)s;
        }
    }

  while (count-- > 0)
    {
      *--d = *--s;
    }

  return dest;
}

#endif /* CONFIG_LIBC_ARCH_MEMMOVE */
