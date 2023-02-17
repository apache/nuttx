/****************************************************************************
 * libs/libm/libm/lib_scalbnl.c
 * get a long double number of x*2^n
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
#include <endian.h>
#include <float.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#ifdef CONFIG_HAVE_LONG_DOUBLE

#define LDOUBLE_MIN (0x1p-16382l)

#if LDBL_MANT_DIG == 53 && LDBL_MAX_EXP == 1024
#elif LDBL_MANT_DIG == 64 && LDBL_MAX_EXP == 16384 && __BYTE_ORDER == __LITTLE_ENDIAN
union ldshape
{
  long double f;
  struct
    {
      uint64_t m;
      uint16_t se;
    }i;
};
#elif LDBL_MANT_DIG == 64 && LDBL_MAX_EXP == 16384 && __BYTE_ORDER == __BIG_ENDIAN

/* This is the m68k variant of 80-bit long double,
 * and this definition only works on archs where
 * the alignment requirement of uint64_t is <= 4.
 */

union ldshape
  {
    long double f;
    struct
      {
        uint16_t se;
        uint16_t pad;
        uint64_t m;
      }i;
};
#elif LDBL_MANT_DIG == 113 && LDBL_MAX_EXP == 16384 && __BYTE_ORDER == __LITTLE_ENDIAN
union ldshape
  {
    long double f;
    struct
      {
        uint64_t lo;
        uint32_t mid;
        uint16_t top;
        uint16_t se;
      }i;
};
#elif LDBL_MANT_DIG == 113 && LDBL_MAX_EXP == 16384 && __BYTE_ORDER == __BIG_ENDIAN
union ldshape
  {
    long double f;
    struct
      {
        uint16_t se;
        uint16_t top;
        uint32_t mid;
        uint64_t lo;
      }i;
};
#else
#error Unsupported long double representation
#endif

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
#if LDBL_MANT_DIG == 53 && LDBL_MAX_EXP == 1024
long double scalbnl(long double x, int n)
{
  return scalbn(x, n);
}
#elif (LDBL_MANT_DIG == 64 || LDBL_MANT_DIG == 113) && LDBL_MAX_EXP == 16384
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
      x *= LDOUBLE_MIN * 0x1p113l;
      n += 16382 - 113;
      if (n < -16382)
        {
          x *= LDOUBLE_MIN * 0x1p113l;
          n += 16382 - 113;
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
#endif  /* LDBL_MANT_DIG == 53 && LDBL_MAX_EXP == 1024 */
#endif  /* CONFIG_HAVE_LONG_DOUBLE */

