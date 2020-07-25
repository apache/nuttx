/****************************************************************************
 * libs/libc/stdio/legacy_dtoa.c
 *
 * This file was ported to NuttX by Yolande Cates.
 *
 * Copyright (c) 1990, 1993
 *      The Regents of the University of California.  All rights reserved.
 *
 * This code is derived from software contributed to Berkeley by
 * Chris Torek.
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

#include <nuttx/config.h>

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "libc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DTOA_UNSIGNED_SHIFTS
#  define SIGN_EXTEND(a,b) if (b < 0) a |= 0xffff0000;
#else
#  define SIGN_EXTEND(a,b)      /* no-op */
#endif

#ifdef CONFIG_ENDIAN_BIG
#  define WORD0(x) ((uint32_t *)&x)[0]
#  define WORD1(x) ((uint32_t *)&x)[1]
#else
#  define WORD0(x) ((uint32_t *)&x)[1]
#  define WORD1(x) ((uint32_t *)&x)[0]
#endif

#ifdef CONFIG_ENDIAN_BIG
#  define STOREINC(a,b,c) (((unsigned short *)a)[0] = (unsigned short)b, \
                         ((unsigned short *)a)[1] = (unsigned short)c, a++)
#else
#  define STOREINC(a,b,c) (((unsigned short *)a)[1] = (unsigned short)b, \
                         ((unsigned short *)a)[0] = (unsigned short)c, a++)
#endif

#define EXP_SHIFT   20
#define EXP_SHIFT1  20
#define EXP_MSK1    0x100000
#define EXP_MSK11   0x100000
#define EXP_MASK    0x7ff00000
#define P           53
#define BIAS        1023
#define IEEE_ARITH
#define EMIN        (-1022)
#define EXP_1       0x3ff00000
#define EXP_11      0x3ff00000
#define EBITS       11
#define FRAC_MASK   0xfffff
#define FRAC_MASK1  0xfffff
#define TEN_PMAX    22
#define BLETCH      0x10
#define BNDRY_MASK  0xfffff
#define BNDRY_MASK1 0xfffff
#define LSB         1
#define SIGN_BIT    0x80000000
#define LOG2P       1
#define TINY0       0
#define TINY1       1
#define QUICK_MAX   14
#define SMALL_MAX   14
#define INFINITE(x) (WORD0(x) == 0x7ff00000)    /* sufficient test for here */

#define KMAX        15

#define BCOPY(x,y)  memcpy((char *)&x->sign, (char *)&y->sign, \
                           y->wds*sizeof(long) + 2*sizeof(int))

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

struct bigint_s
{
  FAR struct bigint_s *next;
  int k;
  int maxwds;
  int sign;
  int wds;
  unsigned long x[1];
};

typedef struct bigint_s bigint_t;

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* REVISIT:  __dtoa is not thread safe due to thse two global variables.
 * Options:
 *
 * 1. Allocate on stack.  g_freelist is rather large, however.. around 275
 *    bytes (it could be shrunk a little by using stdint types instead of
 *    int.
 * 2. Semaphore protect the global variables and handle interrupt level
 *    calls as a special case (perhaps refusing them?  Or having a duplicate
 *    set of variables, one for tasks and one for interrupt usage)
 */

static FAR bigint_t *g_freelist[KMAX + 1];
static FAR bigint_t *g_p5s;

#ifdef IEEE_ARITH
static const double g_bigtens[] =
{
  1e16, 1e32, 1e64, 1e128, 1e256
};

#  define n_bigtens 5
#else
static const double g_bigtens[] =
{
  1e16, 1e32
};

#  define n_bigtens 2
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static FAR bigint_t *balloc(int k)
{
  FAR bigint_t *rv;
  int x;

  if ((rv = g_freelist[k]))
    {
      g_freelist[k] = rv->next;
    }
  else
    {
      x           = 1 << k;
      rv          = (FAR bigint_t *)
                    lib_malloc(sizeof(bigint_t) + (x - 1) * sizeof(long));
      rv->k       = k;
      rv->maxwds  = x;
    }

  rv->sign        = 0;
  rv->wds         = 0;
  return rv;
}

static void bfree(FAR bigint_t *v)
{
  if (v != NULL)
    {
      v->next        = g_freelist[v->k];
      g_freelist[v->k] = v;
    }
}

/* Multiply by m and add a */

static FAR bigint_t *multadd(FAR bigint_t *b, int m, int a)
{
  FAR bigint_t *b1;
  FAR unsigned long *x;
  unsigned long y;
#ifdef CONFIG_DTOA_PACK32
  unsigned long xi;
  unsigned long z;
#endif
  int wds;
  int i;

  wds = b->wds;
  x   = b->x;
  i   = 0;

  do
    {
#ifdef CONFIG_DTOA_PACK32
      xi   = *x;
      y    = (xi & 0xffff) * m + a;
      z    = (xi >> 16) * m + (y >> 16);
      a    = (int)(z >> 16);
      *x++ = (z << 16) + (y & 0xffff);
#else
      y    = *x * m + a;
      a    = (int)(y >> 16);
      *x++ = y & 0xffff;
#endif
    }
  while (++i < wds);

  if (a != 0)
    {
      if (wds >= b->maxwds)
        {
          b1 = balloc(b->k + 1);
          BCOPY(b1, b);
          bfree(b);
          b  = b1;
        }

      b->x[wds++] = a;
      b->wds      = wds;
    }

  return b;
}

static int hi0bits(unsigned long x)
{
  int k = 0;

  if ((x & 0xffff0000) == 0)
    {
      k   = 16;
      x <<= 16;
    }

  if ((x & 0xff000000) == 0)
    {
      k  += 8;
      x <<= 8;
    }

  if ((x & 0xf0000000) == 0)
    {
      k  += 4;
      x <<= 4;
    }

  if ((x & 0xc0000000) == 0)
    {
      k  += 2;
      x <<= 2;
    }

  if ((x & 0x80000000) == 0)
    {
      k++;
      if ((x & 0x40000000) == 0)
        {
          return 32;
        }
    }

  return k;
}

static int lo0bits(FAR unsigned long *y)
{
  unsigned long x = *y;
  int k;

  if ((x & 7) != 0)
    {
      if (x & 1)
        {
          return 0;
        }

      if ((x & 2) != 0)
        {
          *y = x >> 1;
          return 1;
        }

      *y = x >> 2;
      return 2;
    }

  k = 0;
  if ((x & 0xffff) == 0)
    {
      k   = 16;
      x >>= 16;
    }

  if ((x & 0xff) == 0)
    {
      k  += 8;
      x >>= 8;
    }

  if ((x & 0xf) == 0)
    {
      k  += 4;
      x >>= 4;
    }

  if ((x & 0x3) == 0)
    {
      k  += 2;
      x >>= 2;
    }

  if ((x & 1) == 0)
    {
      k++;
      x >>= 1;
      if ((!x & 1) != 0)
        {
          return 32;
        }
    }

  *y = x;
  return k;
}

static FAR bigint_t *i2b(int i)
{
  FAR bigint_t *b;

  b       = balloc(1);
  b->x[0] = i;
  b->wds  = 1;
  return b;
}

static FAR bigint_t *mult(FAR bigint_t *a, FAR bigint_t *b)
{
  FAR bigint_t *c;
  FAR unsigned long *x;
  FAR unsigned long *xa;
  FAR unsigned long *xae;
  FAR unsigned long *xb;
  FAR unsigned long *xbe;
  FAR unsigned long *xc;
  FAR unsigned long *xc0;
  unsigned long carry;
  unsigned long y;
  unsigned long z;
#ifdef CONFIG_DTOA_PACK32
  uint32_t z2;
#endif
  int k;
  int wa;
  int wb;
  int wc;

  if (a->wds < b->wds)
    {
      c = a;
      a = b;
      b = c;
    }

  k  = a->k;
  wa = a->wds;
  wb = b->wds;
  wc = wa + wb;

  if (wc > a->maxwds)
    {
      k++;
    }

  c = balloc(k);
  for (x = c->x, xa = x + wc; x < xa; x++)
    {
      *x = 0;
    }

  xa  = a->x;
  xae = xa + wa;
  xb  = b->x;
  xbe = xb + wb;
  xc0 = c->x;

#ifdef CONFIG_DTOA_PACK32
  for (; xb < xbe; xb++, xc0++)
    {
      if ((y = *xb & 0xffff) != 0)
        {
          x     = xa;
          xc    = xc0;
          carry = 0;

          do
            {
              z     = (*x & 0xffff) * y + (*xc & 0xffff) + carry;
              carry = z >> 16;
              z2    = (*x++ >> 16) * y + (*xc >> 16) + carry;
              carry = z2 >> 16;
              STOREINC(xc, z2, z);
            }
          while (x < xae);

          *xc = carry;
        }

      if ((y = *xb >> 16))
        {
          x     = xa;
          xc    = xc0;
          carry = 0;
          z2    = *xc;

          do
            {
              z     = (*x & 0xffff) * y + (*xc >> 16) + carry;
              carry = z >> 16;
              STOREINC(xc, z, z2);
              z2    = (*x++ >> 16) * y + (*xc & 0xffff) + carry;
              carry = z2 >> 16;
            }
          while (x < xae);

          *xc = z2;
        }
    }
#else
  for (; xb < xbe; xc0++)
    {
      if ((y = *xb++))
        {
          x     = xa;
          xc    = xc0;
          carry = 0;

          do
            {
              z     = *x++ * y + *xc + carry;
              carry = z >> 16;
              *xc++ = z & 0xffff;
            }
          while (x < xae);

          *xc = carry;
        }
    }
#endif

  for (xc0 = c->x, xc = xc0 + wc; wc > 0 && *--xc == 0; --wc);
  c->wds = wc;
  return c;
}

static FAR bigint_t *pow5mult(FAR bigint_t *b, int k)
{
  FAR bigint_t *b1;
  FAR bigint_t *p5;
  FAR bigint_t *p51;
  int i;
  static int p05[3] =
  {
    5, 25, 125
  };

  if ((i = k & 3) != 0)
    {
      b = multadd(b, p05[i - 1], 0);
    }

  if ((k >>= 2) == 0)
    {
      return b;
    }

  if ((p5 = g_p5s) == 0)
    {
      /* First time */

      g_p5s    = i2b(625);
      p5       = g_p5s;
      p5->next = 0;
    }

  for (; ; )
    {
      if ((k & 1) != 0)
        {
          b1 = mult(b, p5);
          bfree(b);
          b  = b1;
        }

      if ((k >>= 1) == 0)
        {
          break;
        }

      if ((p51 = p5->next) == 0)
        {
          p5->next  = mult(p5, p5);
          p51       = p5->next;
          p51->next = 0;
        }

      p5 = p51;
    }

  return b;
}

static FAR bigint_t *lshift(FAR bigint_t *b, int k)
{
  FAR bigint_t *b1;
  FAR unsigned long *x;
  FAR unsigned long *x1;
  FAR unsigned long *xe;
  unsigned long z;
  int i;
  int k1;
  int n;
  int n1;

#ifdef CONFIG_DTOA_PACK32
  n = k >> 5;
#else
  n = k >> 4;
#endif
  k1 = b->k;
  n1 = n + b->wds + 1;

  for (i = b->maxwds; n1 > i; i <<= 1)
    {
      k1++;
    }

  b1 = balloc(k1);
  x1 = b1->x;

  for (i = 0; i < n; i++)
    {
      *x1++ = 0;
    }

  x  = b->x;
  xe = x + b->wds;

#ifdef CONFIG_DTOA_PACK32
  if ((k &= 0x1f) != 0)
    {
      k1 = 32 - k;
      z   = 0;

      do
        {
          *x1++ = *x << k | z;
          z     = *x++ >> k1;
        }
      while (x < xe);

      if ((*x1 = z) != 0)
        {
          ++n1;
        }
    }
#else
  if ((k &= 0xf) != 0)
    {
      k1 = 16 - k;
      z   = 0;

      do
        {
          *x1++ = ((*x << k) & 0xffff) | z;
          z     = *x++ >> k1;
        }
      while (x < xe);

      if ((*x1 = z) != 0)
        {
          ++n1;
        }
    }
#endif
  else
    {
      do
        {
          *x1++ = *x++;
        }
      while (x < xe);
    }

  b1->wds = n1 - 1;
  bfree(b);
  return b1;
}

static int cmp(FAR bigint_t *a, FAR bigint_t *b)
{
  FAR unsigned long *xa;
  FAR unsigned long *xa0;
  FAR unsigned long *xb;
  FAR unsigned long *xb0;
  int i;
  int j;

  i = a->wds;
  j = b->wds;

#ifdef CONFIG_DEBUG_LIB
  if (i > 1 && a->x[i - 1] == 0)
    {
      lerr("ERROR: cmp called with a->x[a->wds-1] == 0\n");
    }

  if (j > 1 && b->x[j - 1] == 0)
    {
      lerr("ERROR: cmp called with b->x[b->wds-1] == 0\n");
    }
#endif

  if (i -= j)
    {
      return i;
    }

  xa0 = a->x;
  xa  = xa0 + j;
  xb0 = b->x;
  xb  = xb0 + j;

  for (; ; )
    {
      if (*--xa != *--xb)
        {
          return *xa < *xb ? -1 : 1;
        }

      if (xa <= xa0)
        {
          break;
        }
    }

  return 0;
}

static FAR bigint_t *diff(FAR bigint_t *a, FAR bigint_t *b)
{
  FAR bigint_t *c;
  FAR unsigned long *xa;
  FAR unsigned long *xae;
  FAR unsigned long *xb;
  FAR unsigned long *xbe;
  FAR unsigned long *xc;
  long borrow;                  /* We need signed shifts here. */
  long y;
#ifdef CONFIG_DTOA_PACK32
  int32_t z;
#endif
  int i;
  int wa;
  int wb;

  i = cmp(a, b);
  if (i == 0)
    {
      c       = balloc(0);
      c->wds  = 1;
      c->x[0] = 0;
      return c;
    }

  if (i < 0)
    {
      c = a;
      a = b;
      b = c;
      i = 1;
    }
  else
    {
      i = 0;
    }

  c       = balloc(a->k);
  c->sign = i;
  wa      = a->wds;
  xa      = a->x;
  xae     = xa + wa;
  wb      = b->wds;
  xb      = b->x;
  xbe     = xb + wb;
  xc      = c->x;
  borrow  = 0;

#ifdef CONFIG_DTOA_PACK32
  do
    {
      y      = (*xa & 0xffff) - (*xb & 0xffff) + borrow;
      borrow = y >> 16;
      SIGN_EXTEND(borrow, y);
      z      = (*xa++ >> 16) - (*xb++ >> 16) + borrow;
      borrow = z >> 16;
      SIGN_EXTEND(borrow, z);
      STOREINC(xc, z, y);
    }
  while (xb < xbe);

  while (xa < xae)
    {
      y      = (*xa & 0xffff) + borrow;
      borrow = y >> 16;
      SIGN_EXTEND(borrow, y);
      z      = (*xa++ >> 16) + borrow;
      borrow = z >> 16;
      SIGN_EXTEND(borrow, z);
      STOREINC(xc, z, y);
    }
#else
  do
    {
      y      = *xa++ - *xb++ + borrow;
      borrow = y >> 16;
      SIGN_EXTEND(borrow, y);
      *xc++  = y & 0xffff;
    }
  while (xb < xbe);

  while (xa < xae)
    {
      y      = *xa++ + borrow;
      borrow = y >> 16;
      SIGN_EXTEND(borrow, y);
      *xc++  = y & 0xffff;
    }
#endif

  while (*--xc == 0)
    {
      wa--;
    }

  c->wds = wa;
  return c;
}

static FAR bigint_t *d2b(double d, int *e, int *bits)
{
  FAR bigint_t *b;
  FAR unsigned long *x;
  unsigned long y;
  unsigned long z;
  int de;
  int i;
  int k;

#ifdef CONFIG_DTOA_PACK32
  b = balloc(1);
#else
  b = balloc(2);
#endif
  x = b->x;

  z         = WORD0(d) & FRAC_MASK;
  WORD0(d) &= 0x7fffffff;       /* Clear sign bit, which we ignore */

  de = (int)(WORD0(d) >> EXP_SHIFT);
  if (de != 0)
    {
      z |= EXP_MSK1;
    }

#ifdef CONFIG_DTOA_PACK32
  if ((y = WORD1(d)) != 0)
    {
      if ((k = lo0bits(&y)) != 0)
        {
          x[0] = y | z << (32 - k);
          z  >>= k;
        }
      else
        {
          x[0] = y;
        }

      b->wds = (x[1] = z) ? 2 : 1;
      i      = b->wds;
    }
  else
    {
#ifdef CONFIG_DEBUG_LIB
      if (z == 0)
        {
          lerr("ERROR: Zero passed to d2b\n");
        }
#endif

      k    = lo0bits(&z);
      x[0] = z;
      i    = b->wds = 1;
      k   += 32;
    }
#else
  if ((y = WORD1(d)) != 0)
    {
      if ((k = lo0bits(&y)) != 0)
        {
          if (k >= 16)
            {
              x[0] = y | ((z << (32 - k)) & 0xffff);
              x[1] = z >> (k - 16) & 0xffff;
              x[2] = z >> k;
              i    = 2;
            }
          else
            {
              x[0] = y & 0xffff;
              x[1] = (y >> 16) | ((z << (16 - k)) & 0xffff);
              x[2] = z >> k & 0xffff;
              x[3] = z >> (k + 16);
              i    = 3;
            }
        }
      else
        {
          x[0] = y & 0xffff;
          x[1] = y >> 16;
          x[2] = z & 0xffff;
          x[3] = z >> 16;
          i    = 3;
        }
    }
  else
    {
#ifdef CONFIG_DEBUG_LIB
      if (z == 0)
        {
          lerr("ERROR: Zero passed to d2b\n");
        }
#endif

      k = lo0bits(&z);
      if (k >= 16)
        {
          x[0] = z;
          i    = 0;
        }
      else
        {
          x[0] = z & 0xffff;
          x[1] = z >> 16;
          i    = 1;
        }

      k += 32;
    }

  while (!x[i])
    {
      --i;
    }

  b->wds = i + 1;
#endif
  if (de != 0)
    {
      *e    = de - BIAS - (P - 1) + k;
      *bits = P - k;
    }
  else
    {
      *e    = de - BIAS - (P - 1) + 1 + k;
#ifdef CONFIG_DTOA_PACK32
      *bits = 32 * i - hi0bits(x[i - 1]);
#else
      *bits = (i + 2) * 16 - hi0bits(x[i]);
#endif
    }

  return b;
}

static const double tens[] =
{
  1e0, 1e1, 1e2, 1e3, 1e4, 1e5, 1e6, 1e7, 1e8, 1e9,
  1e10, 1e11, 1e12, 1e13, 1e14, 1e15, 1e16, 1e17, 1e18, 1e19,
  1e20, 1e21, 1e22
};

static int quorem(FAR bigint_t *b, FAR bigint_t *s)
{
  long borrow;
  long y;
  unsigned long carry;
  unsigned long q;
  unsigned long ys;
  FAR unsigned long *bx;
  FAR unsigned long *bxe;
  FAR unsigned long *sx;
  FAR unsigned long *sxe;
#ifdef CONFIG_DTOA_PACK32
  int32_t z;
  uint32_t si;
  uint32_t zs;
#endif
  int n;

  n = s->wds;

#ifdef CONFIG_DEBUG_LIB
  if (b->wds > n)
    {
      lerr("ERROR: oversize b in quorem\n");
    }
#endif

  if (b->wds < n)
    {
      return 0;
    }

  sx   = s->x;
  sxe = sx + --n;
  bx  = b->x;
  bxe = bx + n;
  q   = *bxe / (*sxe + 1);      /* ensure q <= true quotient */

#ifdef CONFIG_DEBUG_LIB
  if (q > 9)
    {
      lerr("ERROR: oversized quotient in quorem\n");
    }
#endif

  if (q != 0)
    {
      borrow = 0;
      carry  = 0;

      do
        {
#ifdef CONFIG_DTOA_PACK32
          si     = *sx++;
          ys     = (si & 0xffff) * q + carry;
          zs     = (si >> 16) * q + (ys >> 16);
          carry  = zs >> 16;
          y      = (*bx & 0xffff) - (ys & 0xffff) + borrow;
          borrow = y >> 16;
          SIGN_EXTEND(borrow, y);
          z      = (*bx >> 16) - (zs & 0xffff) + borrow;
          borrow = z >> 16;
          SIGN_EXTEND(borrow, z);
          STOREINC(bx, z, y);
#else
          ys     = *sx++ * q + carry;
          carry  = ys >> 16;
          y      = *bx - (ys & 0xffff) + borrow;
          borrow = y >> 16;
          SIGN_EXTEND(borrow, y);
          *bx++  = y & 0xffff;
#endif
        }
      while (sx <= sxe);

      if (*bxe == 0)
        {
          bx = b->x;
          while (--bxe > bx && *bxe == 0)
            {
              --n;
            }

          b->wds = n;
        }
    }

  if (cmp(b, s) >= 0)
    {
      q++;
      borrow = 0;
      carry  = 0;
      bx     = b->x;
      sx     = s->x;

      do
        {
#ifdef CONFIG_DTOA_PACK32
          si     = *sx++;
          ys     = (si & 0xffff) + carry;
          zs     = (si >> 16) + (ys >> 16);
          carry  = zs >> 16;
          y      = (*bx & 0xffff) - (ys & 0xffff) + borrow;
          borrow = y >> 16;
          SIGN_EXTEND(borrow, y);
          z      = (*bx >> 16) - (zs & 0xffff) + borrow;
          borrow = z >> 16;
          SIGN_EXTEND(borrow, z);
          STOREINC(bx, z, y);
#else
          ys     = *sx++ + carry;
          carry  = ys >> 16;
          y      = *bx - (ys & 0xffff) + borrow;
          borrow = y >> 16;
          SIGN_EXTEND(borrow, y);
          *bx++  = y & 0xffff;
#endif
        }
      while (sx <= sxe);

      bx  = b->x;
      bxe = bx + n;

      if (*bxe == 0)
        {
          while (--bxe > bx && *bxe == 0)
            {
              --n;
            }

          b->wds = n;
        }
    }

  return q;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* dtoa for IEEE arithmetic (dmg): convert double to ASCII string.
 *
 * Inspired by "How to Print Floating-Point Numbers Accurately" by
 * Guy L. Steele, Jr. and Jon L. White [Proc. ACM SIGPLAN '90, pp. 92-101].
 *
 * Modifications:
 *      1. Rather than iterating, we use a simple numeric overestimate
 *         to determine k = floor(log10(d)).  We scale relevant
 *         quantities using O(log2(k)) rather than O(k) multiplications.
 *      2. For some modes > 2 (corresponding to ecvt and fcvt), we don't
 *         try to generate digits strictly left to right.  Instead, we
 *         compute with fewer bits and propagate the carry if necessary
 *         when rounding the final digit up.  This is often faster.
 *      3. Under the assumption that input will be rounded nearest,
 *         mode 0 renders 1e23 as 1e23 rather than 9.999999999999999e22.
 *         That is, we allow equality in stopping tests when the
 *         round-nearest rule will give the same floating-point value
 *         as would satisfaction of the stopping test with strict
 *         inequality.
 *      4. We remove common factors of powers of 2 from relevant
 *         quantities.
 *      5. When converting floating-point integers less than 1e16,
 *         we use floating-point arithmetic rather than resorting
 *         to multiple-precision integers.
 *      6. When asked to produce fewer than 15 digits, we first try
 *         to get by with floating-point arithmetic; we resort to
 *         multiple-precision integer arithmetic only if we cannot
 *         guarantee that the floating-point calculation has given
 *         the correctly rounded result.  For k requested digits and
 *         "uniformly" distributed input, the probability is
 *         something like 10^(k-15) that we must resort to the int32_t
 *         calculation.
 */

FAR char *__dtoa(double d, int mode, int ndigits, FAR int *decpt,
                 FAR int *sign, FAR char **rve)
{
  /* Arguments ndigits, decpt, sign are similar to those of ecvt and fcvt;
   * trailing zeros are suppressed from the returned string.  If not null,
   * *rve is set to point to the end of the return value.  If d is +-Infinity
   * or NaN, then *decpt is set to 9999.
   *
   * mode: 0 ==> shortest string that yields d when read in and rounded to
   * nearest. 1 ==> like 0, but with Steele & White stopping rule; e.g. with
   * IEEE P754 arithmetic , mode 0 gives 1e23 whereas mode 1 gives
   * 9.999999999999999e22. 2 ==> max(1,ndigits) significant digits.  This
   * gives a return value similar to that of ecvt, except that trailing zeros
   * are suppressed. 3 ==> through ndigits past the decimal point.  This
   * gives a return value similar to that from fcvt, except that trailing
   * zeros are suppressed, and ndigits can be negative. 4-9 should give the
   * same return values as 2-3, i.e., 4 <= mode <= 9 ==> same return as mode
   * 2 + (mode & 1).  These modes are mainly for debugging; often they run
   * slower but sometimes faster than modes 2-3. 4,5,8,9 ==> left-to-right
   * digit generation. 6-9 ==> don't try fast floating-point estimate (if
   * applicable).
   *
   * Values of mode other than 0-9 are treated as mode 0.
   *
   * Sufficient space is allocated to the return value to hold the suppressed
   * trailing zeros.
   */

  static FAR bigint_t *result;
  static int result_k;
  FAR bigint_t *b;
  FAR bigint_t *b1;
  FAR bigint_t *delta;
  FAR bigint_t *mlo = NULL;
  FAR bigint_t *mhi;
  FAR bigint_t *s;
  FAR char *st;
  FAR char *st0;
  double d2;
  double ds;
  double eps;
  long l;
  unsigned long x;
  int denorm;
  int bbits;
  int b2;
  int b5;
  int be;
  int dig;
  int i;
  int ieps;
  int ilim = 0;
  int ilim0;
  int ilim1 = 0;
  int j;
  int j_1;
  int k;
  int k0;
  int k_check;
  int leftright;
  int m2;
  int m5;
  int s2;
  int s5;
  int spec_case = 0;
  int try_quick;

  if (result != 0)
    {
      result->k      = result_k;
      result->maxwds = 1 << result_k;
      bfree(result);
      result         = 0;
    }

  if ((WORD0(d) & SIGN_BIT) != 0)
    {
      /* Set sign for everything, including 0's and NaNs */

      *sign     = 1;
      WORD0(d) &= ~SIGN_BIT;    /* clear sign bit */
    }
  else
    {
      *sign     = 0;
    }

#if defined(IEEE_ARITH)
#  ifdef IEEE_ARITH
  if ((WORD0(d) & EXP_MASK) == EXP_MASK)
#else
  if (WORD0(d) == 0x8000)
#endif
    {
      /* Infinity or NaN */

      *decpt = 9999;
      st =
#ifdef IEEE_ARITH
        !WORD1(d) && !(WORD0(d) & 0xfffff) ? "Infinity" :
#endif
        "NaN";

      if (rve != NULL)
        {
          *rve =
#ifdef IEEE_ARITH
            st[3] ? st + 8 :
#endif
            st + 3;
        }

      return st;
    }
#endif

  if (d == 0)
    {
      *decpt = 1;
      st = "0";

      if (rve != NULL)
        {
          *rve = st + 1;
        }

      return st;
    }

  b = d2b(d, &be, &bbits);
  i = (int)(WORD0(d) >> EXP_SHIFT1 & (EXP_MASK >> EXP_SHIFT1));
  if (i != 0)
    {
      d2         = d;
      WORD0(d2) &= FRAC_MASK1;
      WORD0(d2) |= EXP_11;

      /* log(x) ~=~ log(1.5) + (x-1.5)/1.5 log10(x) = log(x) / log(10) ~=~
       * log(1.5)/log(10) + (x-1.5)/(1.5*log(10)) log10(d) =
       * (i-BIAS)*log(2)/log(10) + log10(d2) This suggests computing an
       * approximation k to log10(d) by k = (i - BIAS)*0.301029995663981 + (
       * (d2-1.5)*0.289529654602168 + 0.176091259055681 ); We want k to be
       * too large rather than too small. The error in the first-order Taylor
       * series approximation is in our favor, so we just round up the
       * constant enough to compensate for any error in the multiplication of
       * (i - BIAS) by 0.301029995663981; since |i - BIAS| <= 1077, and
       * 1077 * 0.30103 * 2^-52 ~=~ 7.2e-14, adding 1e-13 to the constant
       * term more than suffices. Hence we adjust the constant term to
       * 0.1760912590558. (We could get a more accurate k by invoking log10,
       * but this is probably not worthwhile.)
       */

      i     -= BIAS;
      denorm = 0;
    }
  else
    {
      /* d is denormalized */

      i          = bbits + be + (BIAS + (P - 1) - 1);
      x          = i > 32 ? WORD0(d) << (64 - i) | WORD1(d) >> (i - 32) :
                   WORD1(d) << (32 - i);
      d2         = x;
      WORD0(d2) -= 31 * EXP_MSK1;       /* Adjust exponent */
      i         -= (BIAS + (P - 1) - 1) + 1;
      denorm     = 1;
    }

  ds = (d2 - 1.5) * 0.289529654602168 + 0.1760912590558 +
       i * 0.301029995663981;
  k  = (int)ds;

  if (ds < 0. && ds != k)
    {
      k--;  /* Want k = floor(ds) */
    }

  k_check = 1;

  if (k >= 0 && k <= TEN_PMAX)
    {
      if (d < tens[k])
        {
          k--;
        }

      k_check = 0;
    }

  j = bbits - i - 1;
  if (j >= 0)
    {
      b2 = 0;
      s2 = j;
    }
  else
    {
      b2 = -j;
      s2 = 0;
    }

  if (k >= 0)
    {
      b5  = 0;
      s5  = k;
      s2 += k;
    }
  else
    {
      b2 -= k;
      b5  = -k;
      s5  = 0;
    }

  if (mode < 0 || mode > 9)
    {
      mode = 0;
    }

  try_quick = 1;
  if (mode > 5)
    {
      mode     -= 4;
      try_quick = 0;
    }

  leftright = 1;
  switch (mode)
    {
    case 0:
    case 1:
      ilim = ilim1 = -1;
      i = 18;
      ndigits = 0;
      break;

    case 2:
      leftright = 0;

      /* FALLTHROUGH */

    case 4:
      if (ndigits <= 0)
        {
          ndigits = 1;
        }

      i     = ndigits;
      ilim1 = i;
      ilim  = i;
      break;

    case 3:
      leftright = 0;

      /* FALLTHROUGH */

    case 5:
      i     = ndigits + k + 1;
      ilim  = i;
      ilim1 = i - 1;

      if (i <= 0)
        {
          i = 1;
        }
    }

  j = sizeof(unsigned long);
  for (result_k = 0;
       (signed)(sizeof(bigint_t) - sizeof(unsigned long) + j) <= i;
       j <<= 1)
    {
      result_k++;
    }

  result = balloc(result_k);
  st0    = (FAR char *)result;
  st     = st0;

  if (ilim >= 0 && ilim <= QUICK_MAX && try_quick)
    {
      /* Try to get by with floating-point arithmetic. */

      i      = 0;
      d2     = d;
      k0     = k;
      ilim0  = ilim;
      ieps   = 2;               /* Conservative */

      if (k > 0)
        {
          ds = tens[k & 0xf];
          j   = k >> 4;

          if ((j & BLETCH) != 0)
            {
              /* Prevent overflows */

              j &= BLETCH - 1;
              d /= g_bigtens[n_bigtens - 1];
              ieps++;
            }

          for (; j; j >>= 1, i++)
            {
              if (j & 1)
                {
                  ieps++;
                  ds *= g_bigtens[i];
                }
            }

          d /= ds;
        }
      else if ((j_1 = -k))
        {
          d *= tens[j_1 & 0xf];
          for (j = j_1 >> 4; j; j >>= 1, i++)
            {
              if ((j & 1) != 0)
                {
                  ieps++;
                  d *= g_bigtens[i];
                }
            }
        }

      if (k_check && d < 1. && ilim > 0)
        {
          if (ilim1 <= 0)
            {
              goto fast_failed;
            }

          ilim = ilim1;
          k--;
          d   *= 10.;
          ieps++;
        }

      eps         = ieps * d + 7.;
      WORD0(eps) -= (P - 1) * EXP_MSK1;

      if (ilim == 0)
        {
          mhi = 0;
          s   = 0;
          d  -= 5.;

          if (d > eps)
            {
              goto one_digit;
            }

          if (d < -eps)
            {
              goto no_digits;
            }

          goto fast_failed;
        }

#ifndef CONFIG_DTOA_NO_LEFTRIGHT
      if (leftright)
        {
          /* Use Steele & White method of only generating digits needed. */

          eps = 0.5 / tens[ilim - 1] - eps;
          for (i = 0; ; )
            {
              l     = (int)d;
              d    -= l;
              *st++ = '0' + (int)l;

              if (d < eps)
                {
                  goto ret1;
                }

              if (1. - d < eps)
                {
                  goto bump_up;
                }

              if (++i >= ilim)
                {
                  break;
                }

              eps *= 10.;
              d   *= 10.;
            }
        }
      else
        {
#endif
          /* Generate ilim digits, then fix them up. */

          eps *= tens[ilim - 1];
          for (i = 1; ; i++, d *= 10.)
            {
              l     = (int)d;
              d    -= l;
              *st++ = '0' + (int)l;

              if (i == ilim)
                {
                  if (d > 0.5 + eps)
                    {
                      goto bump_up;
                    }
                  else if (d < 0.5 - eps)
                    {
                      while (*--st == '0')
                        {
                        }

                      st++;
                      goto ret1;
                    }

                  break;
                }
            }
#ifndef CONFIG_DTOA_NO_LEFTRIGHT
        }
#endif

fast_failed:
      st   = st0;
      d    = d2;
      k    = k0;
      ilim = ilim0;
    }

  /* Do we have a "small" integer? */

  if (be >= 0 && k <= SMALL_MAX)
    {
      /* Yes. */

      ds = tens[k];
      if (ndigits < 0 && ilim <= 0)
        {
          s = mhi = 0;
          if (ilim < 0 || d <= 5 * ds)
            {
              goto no_digits;
            }

          goto one_digit;
        }

      for (i = 1; ; i++)
        {
          l  = (int)(d / ds);
          d -= l * ds;

#ifdef Check_FLT_ROUNDS
          /* If FLT_ROUNDS == 2, l will usually be high by 1 */

          if (d < 0)
            {
              l--;
              d += ds;
            }
#endif

          *st++ = '0' + (int)l;
          if (i == ilim)
            {
              d += d;
              if (d > ds || (d == ds && (l & 1)))
                {
bump_up:
                  while (*--st == '9')
                    {
                      if (st == st0)
                        {
                          k++;
                          *st = '0';
                          break;
                        }
                    }

                  ++*st++;
                }

              break;
            }

          if ((d *= 10.) == 0)
            {
              break;
            }
        }

      goto ret1;
    }

  m2  = b2;
  m5  = b5;
  mhi = mlo = 0;

  if (leftright)
    {
      if (mode < 2)
        {
          i = denorm ? be + (BIAS + (P - 1) - 1 + 1) : 1 + P - bbits;
        }
      else
        {
          j = ilim - 1;
          if (m5 >= j)
            {
              m5 -= j;
            }
          else
            {
              s5 += j -= m5;
              b5 += j;
              m5  = 0;
            }

          if ((i = ilim) < 0)
            {
              m2 -= i;
              i   = 0;
            }
        }

      b2 += i;
      s2 += i;
      mhi = i2b(1);
    }

  if (m2 > 0 && s2 > 0)
    {
      i   = m2 < s2 ? m2 : s2;
      b2 -= i;
      m2 -= i;
      s2 -= i;
    }

  if (b5 > 0)
    {
      if (leftright)
        {
          if (m5 > 0)
            {
              mhi = pow5mult(mhi, m5);
              b1  = mult(mhi, b);
              bfree(b);
              b   = b1;
            }

          if ((j = b5 - m5) != 0)
            {
              b = pow5mult(b, j);
            }
        }
      else
        {
          b = pow5mult(b, b5);
        }
    }

  s = i2b(1);
  if (s5 > 0)
    {
      s = pow5mult(s, s5);
    }

  /* Check for special case that d is a normalized power of 2. */

  if (mode < 2)
    {
      if (WORD1(d) == 0 && (WORD0(d) & BNDRY_MASK) == 0 &&
          (WORD0(d) & EXP_MASK) != 0)
        {
          /* The special case */

          b2       += LOG2P;
          s2       += LOG2P;
          spec_case = 1;
        }
      else
        {
          spec_case = 0;
        }
    }

  /* Arrange for convenient computation of quotients: shift left if
   * necessary so divisor has 4 leading 0 bits.
   *
   * Perhaps we should just compute leading 28 bits of s once and for all
   * and pass them and a shift to quorem, so it can do shifts and ors
   * to compute the numerator for q.
   */

#ifdef CONFIG_DTOA_PACK32
  i = ((s5 ? 32 - hi0bits(s->x[s->wds - 1]) : 1) + s2) & 0x1f;
  if (i != 0)
    {
      i = 32 - i;
    }
#else
  i = ((s5 ? 32 - hi0bits(s->x[s->wds - 1]) : 1) + s2) & 0xf;
  if (i != 0)
    {
      i = 16 - i;
    }
#endif

  if (i > 4)
    {
      i  -= 4;
      b2 += i;
      m2 += i;
      s2 += i;
    }
  else if (i < 4)
    {
      i  += 28;
      b2 += i;
      m2 += i;
      s2 += i;
    }

  if (b2 > 0)
    {
      b = lshift(b, b2);
    }

  if (s2 > 0)
    {
      s = lshift(s, s2);
    }

  if (k_check)
    {
      if (cmp(b, s) < 0)
        {
          k--;
          b = multadd(b, 10, 0);        /* we botched the k estimate */
          if (leftright)
            {
              mhi = multadd(mhi, 10, 0);
            }

          ilim = ilim1;
        }
    }

  if (ilim <= 0 && mode > 2)
    {
      if (ilim < 0 || cmp(b, s = multadd(s, 5, 0)) <= 0)
        {
          /* no digits, fcvt style */

no_digits:
          k = -1 - ndigits;
          goto ret;
        }

one_digit:
      *st++ = '1';
      k++;
      goto ret;
    }

  if (leftright)
    {
      if (m2 > 0)
        {
          mhi = lshift(mhi, m2);
        }

      /* Compute mlo -- check for special case that d is a normalized power
       * of 2.
       */

      mlo = mhi;
      if (spec_case)
        {
          mhi = balloc(mhi->k);
          BCOPY(mhi, mlo);
          mhi = lshift(mhi, LOG2P);
        }

      for (i = 1; ; i++)
        {
          dig = quorem(b, s) + '0';

          /* Have we yet the shortest decimal string that will round to d? */

          j     = cmp(b, mlo);
          delta = diff(s, mhi);
          j_1   = delta->sign ? 1 : cmp(b, delta);
          bfree(delta);
#ifndef CONFIG_DTOA_ROUND_BIASED
          if (j_1 == 0 && !mode && !(WORD1(d) & 1))
            {
              if (dig == '9')
                {
                 goto round_9_up;
                }

              if (j > 0)
                {
                  dig++;
                }

              *st++ = dig;
              goto ret;
            }
#endif

          if (j < 0 || (j == 0 && !mode
#ifndef CONFIG_DTOA_ROUND_BIASED
              && ((WORD1(d) & 1) == 0)
#endif
            ))
            {
              if ((j_1 > 0))
                {
                  b   = lshift(b, 1);
                  j_1 = cmp(b, s);

                  if ((j_1 > 0 || (j_1 == 0 && (dig & 1))) && dig++ == '9')
                    {
                      goto round_9_up;
                    }
                }

              *st++ = dig;
              goto ret;
            }

          if (j_1 > 0)
            {
              if (dig == '9')
                {
                  /* Possible if i == 1 */

                  round_9_up:
                  *st++ = '9';
                  goto roundoff;
                }

              *st++ = dig + 1;
              goto ret;
            }

          *st++ = dig;
          if (i == ilim)
            {
              break;
            }

          b = multadd(b, 10, 0);
          if (mlo == mhi)
            {
              mhi = multadd(mhi, 10, 0);
              mlo = mhi;
            }
          else
            {
              mlo = multadd(mlo, 10, 0);
              mhi = multadd(mhi, 10, 0);
            }
        }
    }
  else
    {
      for (i = 1; ; i++)
        {
          *st++ = dig = quorem(b, s) + '0';
          if (i >= ilim)
            {
              break;
            }

          b = multadd(b, 10, 0);
        }
    }

  /* Round off last digit */

  b = lshift(b, 1);
  j = cmp(b, s);

  if (j > 0 || (j == 0 && (dig & 1)))
    {
roundoff:
      while (*--st == '9')
        {
          if (st == st0)
            {
              k++;
              *st++ = '1';
              goto ret;
            }
        }

      ++*st++;
    }
  else
    {
      while (*--st == '0')
        {
        }

      st++;
    }

ret:
  bfree(s);
  if (mhi)
    {
      if (mlo && mlo != mhi)
        {
          bfree(mlo);
        }

      bfree(mhi);
    }

ret1:
  bfree(b);
  if (st == st0)
    {
      /* Don't return empty string */

      *st++ = '0';
      k     = 0;
    }

  *st = 0;
  *decpt = k + 1;
  if (rve != NULL)
    {
      *rve = st;
    }

  return st0;
}
