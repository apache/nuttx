/****************************************************************************
 * libs/libc/machine/risc-v/arch_strncmp.c
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

#ifdef CONFIG_LIBC_ARCH_STRNCMP

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int strncmp(FAR const char *cs, FAR const char *ct, size_t nb)
{
  FAR const unsigned char *a = (FAR const unsigned char *)cs;
  FAR const unsigned char *b = (FAR const unsigned char *)ct;

  if ((((uintptr_t)a ^ (uintptr_t)b) & (WORD_BYTES - 1)) == 0 &&
      nb >= 2 * WORD_BYTES)
    {
      while (!WORD_ALIGNED(a))
        {
          if (*a != *b || *a == '\0')
            {
              return *a - *b;
            }

          a++;
          b++;
          nb--;
        }

      /* A word is passed over only while the two agree and neither
       * holds the terminator, so whatever the loop stops on can be
       * settled by the byte loop below inside two words.  Differences
       * and terminators are OR-ed into one value so each pair of words
       * costs a single branch.
       */

      while (nb >= 2 * WORD_BYTES)
        {
          WORD_T w0 = ((FAR const WORD_T *)a)[0];
          WORD_T w1 = ((FAR const WORD_T *)a)[1];
          WORD_T x  = (w0 ^ ((FAR const WORD_T *)b)[0]) |
                      (w1 ^ ((FAR const WORD_T *)b)[1]) |
                      WORD_HASZERO(w0) | WORD_HASZERO(w1);

          if (x != 0)
            {
              break;
            }

          a += 2 * WORD_BYTES;
          b += 2 * WORD_BYTES;
          nb -= 2 * WORD_BYTES;
        }
    }

  while (nb-- > 0)
    {
      if (*a != *b || *a == '\0')
        {
          return *a - *b;
        }

      a++;
      b++;
    }

  return 0;
}

#endif /* CONFIG_LIBC_ARCH_STRNCMP */
