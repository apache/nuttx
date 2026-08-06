/****************************************************************************
 * libs/libc/machine/risc-v/arch_memcmp.c
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

#ifdef CONFIG_LIBC_ARCH_MEMCMP

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int memcmp(FAR const void *s1, FAR const void *s2, size_t n)
{
  FAR const unsigned char *a = s1;
  FAR const unsigned char *b = s2;

  /* Registers at a time when both pointers sit the same distance past a
   * boundary; a byte head walks them up to it.  Mismatched pointers stay
   * byte by byte: a shifting compare pays for itself far later than a
   * shifting copy does.
   */

  if ((((uintptr_t)a ^ (uintptr_t)b) & (WORD_BYTES - 1)) == 0 &&
      n >= 2 * WORD_BYTES)
    {
      while (!WORD_ALIGNED(a))
        {
          if (*a != *b)
            {
              return *a - *b;
            }

          a++;
          b++;
          n--;
        }

      while (n >= WORD_BYTES &&
             *(FAR const WORD_T *)a == *(FAR const WORD_T *)b)
        {
          a += WORD_BYTES;
          b += WORD_BYTES;
          n -= WORD_BYTES;
        }
    }

  while (n-- > 0)
    {
      if (*a != *b)
        {
          return *a - *b;
        }

      a++;
      b++;
    }

  return 0;
}

#endif /* CONFIG_LIBC_ARCH_MEMCMP */
