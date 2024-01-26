/****************************************************************************
 * libs/libc/string/lib_memccpy.c
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
#include <sys/types.h>
#include <string.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_LIBC_STRING_OPTIMIZE
/* Nonzero if either x or y is not aligned on a "long" boundary. */

#define UNALIGNED(x, y) \
  (((long)(uintptr_t)(x) & (sizeof(long) - 1)) | ((long)(uintptr_t)(y) & (sizeof(long) - 1)))

/* How many bytes are copied each iteration of the word copy loop. */

#define LITTLEBLOCKSIZE (sizeof(long))

/* Threshhold for punting to the byte copier. */

#define TOO_SMALL(len) ((len) < LITTLEBLOCKSIZE)

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

/****************************************************************************
 * Name: memccpy
 *
 * Description:
 *   The memccpy() function copies bytes from memory area s2 into s1,
 *   stopping after the first occurrence of byte c (converted to an unsigned
 *   char) is copied, or after n bytes are copied, whichever comes first. If
 *   copying takes place between objects that overlap, the behavior is
 *   undefined.
 *
 * Returned Value:
 *   The memccpy() function returns a pointer to the byte after the copy of c
 *   in s1, or a null pointer if c was not found in the first n bytes of s2.
 *
 ****************************************************************************/

#undef memccpy /* See mm/README.txt */
FAR void *memccpy(FAR void *s1, FAR const void *s2, int c, size_t n)
{
#ifdef CONFIG_LIBC_STRING_OPTIMIZE
  FAR void *ptr = NULL;
  FAR unsigned char *pout = (FAR unsigned char *)s1;
  FAR const unsigned char *pin = (FAR const unsigned char *)s2;
  FAR long *paligned_out;
  FAR const long *paligned_in;
  unsigned char endchar = c & 0xff;

  /* If the size is small, or either pin or pout is unaligned,
   * then punt into the byte copy loop.  This should be rare.
   */

  if (!TOO_SMALL(n) && !UNALIGNED(pin, pout))
    {
      unsigned int i;
      unsigned long mask = 0;

      paligned_out = (FAR long *)pout;
      paligned_in = (FAR long *)pin;

      /* The fast code reads the ASCII one word at a time and only
       * performs the bytewise search on word-sized segments if they
       * contain the search character, which is detected by XORing
       * the word-sized segment with a word-sized block of the search
       * character and then detecting for the presence of NULL in the
       * result.
       */

      for (i = 0; i < LITTLEBLOCKSIZE; i++)
        {
          mask = (mask << 8) + endchar;
        }

      /* Copy one long word at a time if possible.  */

      while (n >= LITTLEBLOCKSIZE)
        {
          unsigned long buffer = (unsigned long)(*paligned_in);
          buffer ^= mask;
          if (DETECTNULL(buffer))
            {
              break; /* endchar is found, go byte by byte from here */
            }

          *paligned_out++ = *paligned_in++;
          n -= LITTLEBLOCKSIZE;
        }

      /* Pick up any residual with a byte copier.  */

      pout = (FAR unsigned char *)paligned_out;
      pin = (FAR unsigned char *)paligned_in;
    }

  while (n--)
    {
      if ((*pout++ = *pin++) == endchar)
        {
          ptr = pout;
          break;
        }
    }

  return ptr;
#else
  FAR unsigned char *pout = (FAR unsigned char *)s1;
  FAR unsigned char *pin  = (FAR unsigned char *)s2;

  /* Copy at most n bytes */

  while (n-- > 0)
    {
      /* Copy one byte */

      *pout = *pin++;

      /* Did we just copy the terminating byte c? */

      if (*pout++ == (unsigned char)c)
        {
          /* Yes return a pointer to the byte after the copy of c into s1 */

          return (FAR void *)pout;
        }
    }

  /* C was not found in the first n bytes of s2 */

  return NULL;
#endif
}
