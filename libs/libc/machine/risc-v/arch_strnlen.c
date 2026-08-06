/****************************************************************************
 * libs/libc/machine/risc-v/arch_strnlen.c
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

#ifdef CONFIG_LIBC_ARCH_STRNLEN

/****************************************************************************
 * Public Functions
 ****************************************************************************/

size_t strnlen(FAR const char *s, size_t maxlen)
{
  FAR const char *p = s;
  FAR const char *end = s + maxlen;

  /* The cap may sit anywhere, including inside the word that holds the
   * terminator, so the word loop must stop a whole word short of it and
   * leave the rest to the byte tail.
   */

  if (maxlen >= WORD_BYTES)
    {
      while (!WORD_ALIGNED(p))
        {
          if (p == end || *p == '\0')
            {
              return p - s;
            }

          p++;
        }

      while ((size_t)(end - p) >= WORD_BYTES &&
             !WORD_HASZERO(*(FAR const WORD_T *)p))
        {
          p += WORD_BYTES;
        }
    }

  while (p != end && *p != '\0')
    {
      p++;
    }

  return p - s;
}

#endif /* CONFIG_LIBC_ARCH_STRNLEN */
