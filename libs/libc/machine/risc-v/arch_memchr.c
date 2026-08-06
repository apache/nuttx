/****************************************************************************
 * libs/libc/machine/risc-v/arch_memchr.c
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

#ifdef CONFIG_LIBC_ARCH_MEMCHR

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR void *memchr(FAR const void *s, int c, size_t n)
{
  FAR const unsigned char *p = s;
  unsigned char uc = (unsigned char)c;
  WORD_T rep = WORD_REPEAT(uc);

  if (n >= 2 * WORD_BYTES)
    {
      while (!WORD_ALIGNED(p))
        {
          if (n == 0)
            {
              return NULL;
            }

          if (*p == uc)
            {
              return (FAR void *)p;
            }

          p++;
          n--;
        }

      while (n >= WORD_BYTES &&
             !WORD_HASZERO(*(FAR const WORD_T *)p ^ rep))
        {
          p += WORD_BYTES;
          n -= WORD_BYTES;
        }
    }

  while (n-- > 0)
    {
      if (*p == uc)
        {
          return (FAR void *)p;
        }

      p++;
    }

  return NULL;
}

#endif /* CONFIG_LIBC_ARCH_MEMCHR */
