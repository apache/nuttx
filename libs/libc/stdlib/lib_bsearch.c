/****************************************************************************
 * libs/libc/stdlib/lib_bsearch.c
 *
 *   Copyright (c) 1990, 1993
 *   The Regents of the University of California.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *      This product includes software developed by the University of
 *      California, Berkeley and its contributors.
 * 4. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdlib.h>
#include <assert.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bsearch
 *
 * Description:
 *   The bsearch() function will search an array of nel objects, the initial
 *   element of which is pointed to by 'base', for an element that matches
 *   the object pointed to by 'key'. The size of each element in the array
 *   is specified by 'width'. If the nel argument has the value zero, the
 *   comparison function pointed to by 'compar' will not be called and no
 *   match will be found.
 *
 *   The comparison function pointed to by 'compar' will be called with two
 *   arguments that point to the 'key' object and to an array element, in
 *   that order.
 *
 *   The application will ensure that the comparison function pointed to by
 *   'compar 'does not alter the contents of the array. The implementation
 *   may reorder elements of the array between calls to the comparison
 *   function, but will not alter the contents of any individual element.
 *
 *   The implementation will ensure that the first argument is always a
 *   pointer to the 'key'.
 *
 *   When the same objects (consisting of width bytes, irrespective of their
 *   current positions in the array) are passed more than once to the
 *   comparison function, the results will be consistent with one another.
 *   That is, the same object will always compare the same way with the key.
 *
 *   The application will ensure that the function returns an integer less
 *   than, equal to, or greater than 0 if the key object is considered,
 *   respectively, to be less than, to match, or to be greater than the
 *   array element. The application will ensure that the array consists of
 *   all the elements that compare less than, all the elements that compare
 *   equal to, and all the elements that compare greater than the key
 *   object, in that order.
 *
 *   (Based on description from OpenGroup.org).
 *
 * Returned Value:
 *   The bsearch() function will return a pointer to a matching member of
 *   the array, or a null pointer if no match is found. If two or more
 *   members compare equal, which member is returned is unspecified.
 *
 * Notes from the NetBSD version:
 *   The code below is a bit sneaky.  After a comparison fails, we divide
 *   the work in half by moving either left or right. If 'lim' is odd,
 *   moving left simply involves halving 'lim': e.g., when 'lim' is 5 we
 *   look at item 2, so we change 'lim' to 2 so that we will look at items
 *   0 & 1.  If 'lim' is even, the same applies.  If 'lim' is odd, moving
 *   right again involes halving 'lim', this time moving the base up one
 *   item past 'middle': e.g., when 'lim' is 5 we change base to item 3 and
 *   make 'lim' 2 so that we will look at items 3 and 4.  If 'lim' is
 *   even, however, we have to shrink it by one before halving: e.g.,
 *   when 'lim' is 4, we still looked at item 2, so we have to make 'lim'
 *   3, then halve, obtaining 1, so that we will only look at item 3.
 *
 ****************************************************************************/

FAR void *bsearch(FAR const void *key, FAR const void *base, size_t nel,
                  size_t width, CODE int (*compar)(FAR const void *,
                  FAR const void *))
{
  FAR const void *middle;  /* Current entry being tested */
  FAR const char *lower;   /* The lower limit of the search region */
  size_t lim;              /* The number of elements in the region */
  int cmp;                 /* Boolean comparison result */

  DEBUGASSERT(key != NULL);
  DEBUGASSERT(base != NULL || nel == 0);
  DEBUGASSERT(compar != NULL);

  for (lim = nel, lower = (const char *)base; lim != 0; lim >>= 1)
    {
      middle = lower + (lim >> 1) * width;
      cmp    = (*compar)(key, middle);

      if (cmp == 0)
        {
          return (FAR void *)middle;
        }

      if (cmp > 0)
        {
          /* key > middle: move right (else move left) */

          lower = (FAR const char *)middle + width;
          lim--;
        }
    }

  return NULL;
}
