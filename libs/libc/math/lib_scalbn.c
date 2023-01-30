/****************************************************************************
 * libs/libc/math/lib_scalbn.c
 * get a double number of x*2^n
 *
 * This file is copy from musl libc
 * musl is an implementation of the C standard library built on top of the
 * Linux system call API, including interfaces defined in the base language
 * standard, POSIX, and widely agreed-upon extensions.
 * musl is lightweight, fast, simple, free, and strives to be correct in
 * the sense of standards-conformance and safety.
 *
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <math.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#ifdef CONFIG_HAVE_DOUBLE
#define DOUBLE_MIN (0x1p-1022)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: scalbn
 *
 * Description:
 *   get a double number of x*2^n
 *
 ****************************************************************************/

double scalbn(double x, int n)
{
  union
    {
      double f;
      uint64_t i;
    }u;

  double_t y = x;

  if (n > 1023)
    {
      y *= 0x1p1023;
      n -= 1023;
      if (n > 1023)
        {
          y *= 0x1p1023;
          n -= 1023;
          if (n > 1023)
            {
              n = 1023;
            }
        }
    }
  else if (n < -1022)
    {
      /* make sure final n < -53 to avoid double
       * rounding in the subnormal range
       */

      y *= DOUBLE_MIN * 0x1p53;
      n += 1022 - 53;
      if (n < -1022)
        {
          y *= DOUBLE_MIN * 0x1p53;
          n += 1022 - 53;
          if (n < -1022)
            {
              n = -1022;
            }
        }
    }

  u.i = (uint64_t)(0x3ff + n) << 52;
  x = y * u.f;
  return x;
}
#endif /* CONFIG_HAVE_DOUBLE */
