/****************************************************************************
 * libs/libc/stdlib/lib_qsort.c
 *
 *   Copyright (C) 2007, 2009, 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Leveraged from:
 *
 *  Copyright (c) 1992, 1993
 *  The Regents of the University of California.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *    This product includes software developed by the University of
 *    California, Berkeley and its contributors.
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

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdlib.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define min(a, b)  (a) < (b) ? a : b

#define swapcode(TYPE, parmi, parmj, n) \
  { \
    long i = (n) / sizeof (TYPE); \
    register TYPE *pi = (TYPE *)(parmi); \
    register TYPE *pj = (TYPE *)(parmj); \
    do { \
      register TYPE  t = *pi; \
      *pi++ = *pj; \
      *pj++ = t; \
    } while (--i > 0); \
  }

#define SWAPINIT(a, width) \
  swaptype = ((FAR char *)a - (FAR char *)0) % sizeof(long) || \
  width % sizeof(long) ? 2 : width == sizeof(long)? 0 : 1;

#define swap(a, b) \
  if (swaptype == 0) \
    { \
      long t = *(long *)(a); \
      *(long *)(a) = *(long *)(b); \
      *(long *)(b) = t; \
    } \
  else \
    { \
      swapfunc(a, b, width, swaptype); \
    }

#define vecswap(a, b, n) if ((n) > 0) swapfunc(a, b, n, swaptype)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline void swapfunc(FAR char *a, FAR char *b, int n, int swaptype);
static inline FAR char *med3(FAR char *a, FAR char *b, FAR char *c,
                             CODE int (*compar)(FAR const void *,
                             FAR const void *));

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void swapfunc(FAR char *a, FAR char *b, int n, int swaptype)
{
  if (swaptype <= 1)
    {
      swapcode(long, a, b, n)
    }
  else
    {
      swapcode(char, a, b, n)
    }
}

static inline FAR char *med3(FAR char *a, FAR char *b, FAR char *c,
                             CODE int (*compar)(FAR const void *,
                             FAR const void *))
{
  return compar(a, b) < 0 ?
         (compar(b, c) < 0 ? b : (compar(a, c) < 0 ? c : a)) :
         (compar(b, c) > 0 ? b : (compar(a, c) < 0 ? a : c));
}

/****************************************************************************
 * Public Function
 ****************************************************************************/

/****************************************************************************
 * Name: qsort
 *
 * Description:
 *   The qsort() function will sort an array of 'nel' objects, the initial
 *   element of which is pointed to by 'base'. The size of each object, in
 *   bytes, is specified by the 'width" argument. If the 'nel' argument has
 *   the value zero, the comparison function pointed to by 'compar' will not
 *   be called and no rearrangement will take place.
 *
 *   The application will ensure that the comparison function pointed to by
 *   'compar' does not alter the contents of the array. The implementation
 *   may reorder elements of the array between calls to the comparison
 *   function, but will not alter the contents of any individual element.
 *
 *   When the same objects (consisting of 'width" bytes, irrespective of
 *   their current positions in the array) are passed more than once to
 *   the comparison function, the results will be consistent with one
 *   another. That is, they will define a total ordering on the array.
 *
 *   The contents of the array will be sorted in ascending order according
 *   to a comparison function. The 'compar' argument is a pointer to the
 *   comparison function, which is called with two arguments that point to
 *   the elements being compared. The application will ensure that the
 *   function returns an integer less than, equal to, or greater than 0,
 *   if the first argument is considered respectively less than, equal to,
 *   or greater than the second. If two members compare as equal, their
 *   order in the sorted array is unspecified.
 *
 *   (Based on description from OpenGroup.org).
 *
 * Returned Value:
 *   The qsort() function will not return a value.
 *
 * Notes from the original BSD version:
 *   Qsort routine from Bentley & McIlroy's "Engineering a Sort Function".
 *
 ****************************************************************************/

void qsort(FAR void *base, size_t nel, size_t width,
           CODE int(*compar)(FAR const void *, FAR const void *))
{
  FAR char *pa;
  FAR char *pb;
  FAR char *pc;
  FAR char *pd;
  FAR char *pl;
  FAR char *pm;
  FAR char *pn;
  int swaptype;
  int swap_cnt;
  int d;
  int r;

loop:
  SWAPINIT(base, width);
  swap_cnt = 0;

  if (nel < 7)
    {
      for (pm = (FAR char *)base + width;
           pm < (FAR char *)base + nel * width;
           pm += width)
        {
          for (pl = pm;
               pl > (FAR char *)base && compar(pl - width, pl) > 0;
               pl -= width)
            {
              swap(pl, pl - width);
            }
        }

      return;
  }

  pm = (FAR char *)base + (nel / 2) * width;
  if (nel > 7)
    {
      pl = base;
      pn = (FAR char *)base + (nel - 1) * width;
      if (nel > 40)
        {
          d  = (nel / 8) * width;
          pl = med3(pl, pl + d, pl + 2 * d, compar);
          pm = med3(pm - d, pm, pm + d, compar);
          pn = med3(pn - 2 * d, pn - d, pn, compar);
        }

      pm = med3(pl, pm, pn, compar);
    }

  swap(base, pm);
  pa = pb = (FAR char *)base + width;

  pc = pd = (FAR char *)base + (nel - 1) * width;
  for (; ; )
    {
      while (pb <= pc && (r = compar(pb, base)) <= 0)
        {
          if (r == 0)
            {
              swap_cnt = 1;
              swap(pa, pb);
              pa += width;
            }

          pb += width;
        }

      while (pb <= pc && (r = compar(pc, base)) >= 0)
        {
          if (r == 0)
            {
              swap_cnt = 1;
              swap(pc, pd);
              pd -= width;
            }

          pc -= width;
        }

      if (pb > pc)
        {
          break;
        }

      swap(pb, pc);
      swap_cnt = 1;
      pb      += width;
      pc      -= width;
    }

  if (swap_cnt == 0)
    {
      /* Switch to insertion sort */

      for (pm = (FAR char *)base + width;
           pm < (FAR char *)base + nel * width;
           pm += width)
        {
          for (pl = pm;
               pl > (FAR char *)base && compar(pl - width, pl) > 0;
               pl -= width)
            {
              swap(pl, pl - width);
            }
        }

      return;
    }

  pn = (FAR char *)base + nel * width;
  r  = min(pa - (FAR char *)base, pb - pa);
  vecswap(base, pb - r, r);

  r  = min(pd - pc, pn - pd - width);
  vecswap(pb, pn - r, r);

  if ((r = pb - pa) > width)
    {
      qsort(base, r / width, width, compar);
    }

  if ((r = pd - pc) > width)
    {
      /* Iterate rather than recurse to save stack space */

      base = pn - r;
      nel = r / width;
      goto loop;
    }
}
