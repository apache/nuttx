/****************************************************************************
 * libs/libc/string/lib_strstr.c
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
#include <stdbool.h>
#include <arpa/inet.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if LONG_MAX == INT32_MAX
  #define LONG_INT_IS_4_BYTES 1
  #define LONG_INT_N_BYTES    4
#else
  #define LONG_INT_N_BYTES    sizeof(long)
#endif

/* Determine the size, in bytes, of a long integer. */

#if defined (CONFIG_ENDIAN_BIG)
  #define ULONG_BIGENDIAN(x) (x)
#elif defined (LONG_INT_IS_4_BYTES)
  #define ULONG_BIGENDIAN(x) htonl(x)
#else
  #undef ULONG_BIGENDIAN
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Finds the first occurrence of the sub-string needle in the
 * string haystack. Returns NULL if needle was not found.
 */

FAR char *strstr(FAR const char *haystack, FAR const char *needle)
{
#ifndef ULONG_BIGENDIAN
  FAR const unsigned char *needle_cmp_end;
#endif
  FAR const unsigned char *i_haystack;
  const char needle_first = *needle;
  FAR const unsigned char *i_needle;
  unsigned long last_haystack_chars;
  unsigned long last_needle_chars;
  FAR const char *sub_start;
  size_t needle_cmp_len;
  bool identical = true;
  unsigned long mask;
  size_t compare_len;
  size_t needle_len;

  if (!*needle)
    {
      return (FAR char *)haystack;
    }

  /* Runs strchr() on the first section of the haystack as it has a lower
   * algorithmic complexity for discarding the first non-matching characters.
   */

  haystack = strchr(haystack, needle_first);
  if (!haystack) /* First character of needle is not in the haystack. */
    {
      return NULL;
    }

  /* First characters of haystack and needle are the same now. Both are
   * guaranteed to be at least one character long.
   * Now computes the sum of the first needle_len characters of haystack
   * minus the sum of characters values of needle.
   */

  i_haystack = (FAR const unsigned char *)haystack + 1;
  i_needle = (FAR const unsigned char *)needle + 1;

  while (*i_haystack && *i_needle)
    {
      identical &= *i_haystack++ == *i_needle++;
    }

  /* i_haystack now references the (needle_len + 1)-th character. */

  if (*i_needle) /* haystack is smaller than needle. */
    {
      return NULL;
    }
  else if (identical)
    {
      return (FAR char *)haystack;
    }

  needle_len = i_needle - (FAR const unsigned char *)needle;

  /* Note:
   * needle_len > 1, because we checked that it isn't zero, and if it
   * is 1 then identical must be true because the first strchr() ensured
   * that the first characters are identical
   */

  sub_start = haystack;
  needle_cmp_len = (needle_len < LONG_INT_N_BYTES) ?
                    needle_len : LONG_INT_N_BYTES;

#ifdef ULONG_BIGENDIAN
  last_needle_chars =
    ULONG_BIGENDIAN(*((FAR unsigned long *)(i_needle - needle_cmp_len)))
    >> (8 * (LONG_INT_N_BYTES - needle_cmp_len));
  last_haystack_chars =
    ULONG_BIGENDIAN(*((FAR unsigned long *)(i_haystack - needle_cmp_len)))
    >> (8 * (LONG_INT_N_BYTES - needle_cmp_len));
#else
  needle_cmp_end = i_needle;

  i_needle -= needle_cmp_len;
  i_haystack -= needle_cmp_len;
  last_needle_chars = 0;
  last_haystack_chars = 0;

  while (i_needle != needle_cmp_end)
    {
      last_needle_chars <<= 8;
      last_needle_chars ^= *i_needle++;
      last_haystack_chars <<= 8;
      last_haystack_chars ^= *i_haystack++;
    }
#endif

  /* At this point:
   * needle is at least two characters long
   * haystack is at least needle_len characters long (also at least two)
   * the first characters of needle and haystack are identical
   */

  if (needle_len > LONG_INT_N_BYTES + 1)
    {
      /* we will call memcmp() only once we know that the LONG_INT_N_BYTES
       * last chars are equal, so it will be enough to compare all but the
       * last LONG_INT_N_BYTES characters
       */

      compare_len = needle_len - LONG_INT_N_BYTES;

      /* iterate through the remainder of haystack while checking for
       * identity of the last LONG_INT_N_BYTES, and checking the rest
       * with memcmp()
       */

      while (*i_haystack)
        {
          last_haystack_chars <<= 8;
          last_haystack_chars ^= *i_haystack++;

          sub_start++;
          if (last_haystack_chars == last_needle_chars &&
              memcmp(sub_start, needle, compare_len) == 0)
            {
              return (FAR char *)sub_start;
            }
        }
    }
  else if (needle_len == LONG_INT_N_BYTES + 1)
    {
      /* iterate through the remainder of haystack while checking for
       * identity of the last LONG_INT_N_BYTES as well as the single
       * additional character, which is the first one
       */

      while (*i_haystack)
        {
          last_haystack_chars <<= 8;
          last_haystack_chars ^= *i_haystack++;

          sub_start++;
          if (last_haystack_chars == last_needle_chars &&
              *sub_start == needle_first)
            {
              return (FAR char *)sub_start;
            }
        }
    }
  else if (needle_len == LONG_INT_N_BYTES)
    {
      /* iterate through the remainder of haystack while checking for
       * identity of the last LONG_INT_N_BYTES characters, which
       * should exactly match the entire needle
       */

      while (*i_haystack)
        {
          last_haystack_chars <<= 8;
          last_haystack_chars ^= *i_haystack++;

          if (last_haystack_chars == last_needle_chars)
            {
              return (FAR char *)(i_haystack - needle_len);
            }
        }
    }
  else /* needle_len < LONG_INT_N_BYTES */
    {
      mask = (((unsigned long)1) << (needle_len * 8)) - 1;
      last_needle_chars &= mask;

      /* iterate through the remainder of haystack, updating the sums'
       * difference and checking for identity whenever the difference
       * is zero
       */

      while (*i_haystack)
        {
          last_haystack_chars <<= 8;
          last_haystack_chars ^= *i_haystack++;
          last_haystack_chars &= mask;

          if (last_haystack_chars == last_needle_chars)
            {
              return (FAR char *)(i_haystack - needle_len);
            }
        }
    }

  return NULL;
}
