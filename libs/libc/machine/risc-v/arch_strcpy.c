/****************************************************************************
 * libs/libc/machine/risc-v/arch_strcpy.c
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

#ifdef CONFIG_LIBC_ARCH_STRCPY

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR char *strcpy(FAR char *dest, FAR const char *src)
{
  FAR const WORD_T *ws;
  FAR WORD_T *wd;
  FAR char *d = dest;

  /* Words only when the two pointers agree about where boundaries fall,
   * since the stores must be aligned and a shifting loop is not worth it
   * for typical string lengths.  Note that this asks whether the two
   * agree modulo the word size, not whether either is aligned: a pair
   * that is offset by the same amount is walked up to the boundary and
   * then copied a word at a time like any other.
   */

  if ((((uintptr_t)d ^ (uintptr_t)src) & (WORD_BYTES - 1)) == 0)
    {
      while (!WORD_ALIGNED(src))
        {
          if ((*d++ = *src++) == '\0')
            {
              return dest;
            }
        }

      /* Cursors of the word type, so that the copy is one load and one
       * store per turn.  Re-casting the char pointers inside the loop
       * costs a reload, since the store through one may alias the other.
       */

      wd = (FAR WORD_T *)d;
      ws = (FAR const WORD_T *)src;

      while (!WORD_HASZERO(*ws))
        {
          *wd++ = *ws++;
        }

      d   = (FAR char *)wd;
      src = (FAR const char *)ws;
    }

  while ((*d++ = *src++) != '\0')
    {
    }

  return dest;
}

#endif /* CONFIG_LIBC_ARCH_STRCPY */
