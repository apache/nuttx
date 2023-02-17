/****************************************************************************
 * libs/libm/libm/lib_scalbnf.c
 * get a float number of x*2^n
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

#define FLT_MIN (0x1p-126f)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: scalbnf
 *
 * Description:
 *   get a float number of x*2^n
 *
 ****************************************************************************/

float scalbnf(float x, int n)
{
  union
    {
      float f;
      uint32_t i;
    }u;

  float_t y = x;

  if (n > 127)
    {
      y *= 0x1p127f;
      n -= 127;
      if (n > 127)
        {
          y *= 0x1p127f;
          n -= 127;
          if (n > 127)
            {
              n = 127;
            }
        }
    }
  else if (n < -126)
    {
      y *= FLT_MIN * 0x1p24f;
      n += 126 - 24;
      if (n < -126)
        {
          y *= FLT_MIN * 0x1p24f;
          n += 126 - 24;
          if (n < -126)
            {
              n = -126;
            }
        }
    }

  u.i = (uint32_t)(0x7f + n) << 23;
  x = y * u.f;
  return x;
}

