/****************************************************************************
 * libs/libc/string/lib_timingsafe_memcmp.c
 *
 * SPDX-License-Identifier: ISC
 * SPDX-FileCopyrightText: 2019 Linaro Limited
 * SPDX-FileCopyrightText: 2004-2007, 2011, 2012 Internet Systems Consortium
 * SPDX-FileCopyrightText: 1999-2001, 2003 Internet Software Consortium
 * SPDX-FileCopyrightText: 2015 J. Perlinger
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND ISC DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS.  IN NO EVENT SHALL ISC BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT
 * OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <limits.h>
#include <string.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: timingsafe_memcmp
 *
 * Description:
 *   Compare two memory areas of n bytes without leaking the offset of the
 *   first differing byte: no early-out and no data dependent branch, one
 *   pass over all n bytes.  It is the ordering counterpart of
 *   timingsafe_bcmp(), which only tells equal from not equal.
 *
 *   The result comes from the first difference: mask is cleared as soon as
 *   one is found, so the bytes that follow cannot change it.  mask is
 *   volatile so that the compiler cannot skip those bytes either.
 *
 * Input Parameters:
 *   b1 - First memory area
 *   b2 - Second memory area
 *   n  - Number of bytes to compare
 *
 * Returned Value:
 *   A negative value if b1 is less than b2, zero if they are equal and a
 *   positive value otherwise.
 *
 ****************************************************************************/

int timingsafe_memcmp(FAR const void *b1, FAR const void *b2, size_t n)
{
  FAR const unsigned char *p1 = b1;
  FAR const unsigned char *p2 = b2;
  volatile int mask = 1u << CHAR_BIT;
  int is_lt = 0;
  int is_gt = 0;

  for (; n > 0; n--)
    {
      is_lt |= mask & (*p1 - *p2);
      is_gt |= mask & (*p2 - *p1);
      mask  &= ~(is_lt | is_gt);
      p1++;
      p2++;
    }

  return (is_gt >> CHAR_BIT) - (is_lt >> CHAR_BIT);
}
