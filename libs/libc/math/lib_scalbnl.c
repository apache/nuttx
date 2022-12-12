/****************************************************************************
 * libs/libc/math/lib_scalbnl.c
 * get a long double number of x*2^n
 *
 * This file is a part of NuttX:
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Ported by: Darcy Gong
 *
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <math.h>

#ifdef CONFIG_HAVE_LONG_DOUBLE

#define LONG_DOUBLE_MIN (0x1p-16381l)
union ldshape
{
  long double f;
  struct
  {
    uint16_t se;
    uint16_t pad;
    uint64_t m;
  } i;
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: scalbnl
 *
 * Description:
 *   get a long double number of x*2^n
 *
 ****************************************************************************/

long double scalbnl(long double x, int n)
{
  union ldshape u;

  if (n > 16383)
    {
      x *= 0x1p16383l;
      n -= 16383;
      if (n > 16383)
        {
          x *= 0x1p16383l;
          n -= 16383;
          if (n > 16383)
            {
              n = 16383;
            }
        }
    }
  else if (n < -16382)
    {
      x *= LONG_DOUBLE_MIN * 0x1p112l;
      n += 16381 - 112;
      if (n < -16382)
        {
          x *= LONG_DOUBLE_MIN * 0x1p112l;
          n += 16381 - 112;
          if (n < -16382)
            {
              n = -16382;
            }
        }
    }

  u.f = 1.0;
  u.i.se = 0x3fff + n;
  return x * u.f;
}
#endif  /* CONFIG_HAVE_LONG_DOUBLE */
