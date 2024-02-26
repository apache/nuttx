/****************************************************************************
 * libs/libc/string/lib_strcat.c
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

#include <string.h>

#include "libc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_LIBC_STRING_OPTIMIZE

#define ALIGNED(x) \
  (((long)(uintptr_t)(x) & (sizeof(long) - 1)) == 0)

/* Macros for detecting endchar */

#if LONG_MAX == 2147483647
#  define DETECTNULL(x) (((x) - 0x01010101) & ~(x) & 0x80808080)
#elif LONG_MAX == 9223372036854775807
/* Nonzero if x (a long int) contains a NULL byte. */

#  define DETECTNULL(x) (((x) - 0x0101010101010101) & ~(x) & 0x8080808080808080)
#endif

#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#if !defined(CONFIG_LIBC_ARCH_STRCAT) && defined(LIBC_BUILD_STRCAT)
#undef strcat /* See mm/README.txt */
nosanitize_address
FAR char *strcat(FAR char *dest, FAR const char *src)
{
#ifdef CONFIG_LIBC_STRING_OPTIMIZE
  FAR char *ret = dest;

  /* Skip over the data in dest as quickly as possible. */

  if (ALIGNED(dest))
    {
      FAR unsigned long *aligned_s1 = (FAR unsigned long *)dest;
      while (!DETECTNULL(*aligned_s1))
        {
          aligned_s1++;
        }

      dest = (FAR char *)aligned_s1;
    }

  while (*dest)
    {
      dest++;
    }

  /* dest now points to the its trailing null character, we can
   * just use strcpy to do the work for us now.
   * ?!? We might want to just include strcpy here.
   * Also, this will cause many more unaligned string copies because
   * dest is much less likely to be aligned.  I don't know if its worth
   * tweaking strcpy to handle this better.
   */

  strcpy(dest, src);
#else
  FAR char *ret = dest;

  dest += strlen(dest);
  while (*src != '\0')
    {
      *dest++ = *src++;
    }

  *dest = '\0';
#endif

  return ret;
}
#endif
