/****************************************************************************
 * libs/libm/libm/lib_gamma.c
 *
 * Ported to NuttX from FreeBSD by Alan Carvalho de Assis:
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/* "A Precision Approximation of the Gamma Function"
 *   - Cornelius Lanczos (1964)
 * "Lanczos Implementation of the Gamma Function"
 *   - Paul Godfrey (2001)
 * "An Analysis of the Lanczos Gamma Approximation"
 *   - Glendon Ralph Pugh (2004)
 *
 * Approximation method:
 *
 *                         (x - 0.5)         S(x)
 * Gamma(x) = (x + g - 0.5)         *  ----------------
 *                                     exp(x + g - 0.5)
 *
 * with
 *                  a1      a2      a3            aN
 * S(x) ~= [ a0 + ----- + ----- + ----- + ... + ----- ]
 *                x + 1   x + 2   x + 3         x + N
 *
 * with a0, a1, a2, a3,.. aN constants which depend on g.
 *
 * for x < 0 the following reflection formula is used:
 *
 * Gamma(x)*Gamma(-x) = -pi/(x sin(pi x))
 *
 * most ideas and constants are from boost and python
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <sys/types.h>
#include <math.h>

#ifdef CONFIG_HAVE_DOUBLE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FORCE_EVAL(x)                       \
  do                                        \
    {                                       \
      if (sizeof(x) == sizeof(float))       \
        {                                   \
          volatile float __x;               \
          UNUSED(__x);                      \
          __x = (x);                        \
        }                                   \
      else if (sizeof(x) == sizeof(double)) \
        {                                   \
          volatile double __x;              \
          UNUSED(__x);                      \
          __x = (x);                        \
        }                                   \
      else                                  \
        {                                   \
          volatile long double __x;         \
          UNUSED(__x);                      \
          __x = (x);                        \
        }                                   \
    } \
  while (0)

#define N 12

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const double pi = 3.141592653589793238462643383279502884;

static const double g_gmhalf = 5.524680040776729583740234375;
static const double g_snum[N + 1] =
{
  23531376880.410759688572007674451636754734846804940,
  42919803642.649098768957899047001988850926355848959,
  35711959237.355668049440185451547166705960488635843,
  17921034426.037209699919755754458931112671403265390,
  6039542586.3520280050642916443072979210699388420708,
  1439720407.3117216736632230727949123939715485786772,
  248874557.86205415651146038641322942321632125127801,
  31426415.585400194380614231628318205362874684987640,
  2876370.6289353724412254090516208496135991145378768,
  186056.26539522349504029498971604569928220784236328,
  8071.6720023658162106380029022722506138218516325024,
  210.82427775157934587250973392071336271166969580291,
  2.5066282746310002701649081771338373386264310793408,
};

static const double g_sden[N + 1] =
{
  0, 39916800, 120543840, 150917976, 105258076, 45995730, 13339535,
  2637558, 357423, 32670, 1925, 66, 1,
};

/* n! for small integer n */

static const double g_fact[] =
{
  1, 1, 2, 6, 24, 120, 720, 5040.0, 40320.0, 362880.0, 3628800.0, 39916800.0,
  479001600.0, 6227020800.0, 87178291200.0, 1307674368000.0,
  20922789888000.0, 355687428096000.0, 6402373705728000.0,
  121645100408832000.0, 2432902008176640000.0, 51090942171709440000.0,
  1124000727777607680000.0,
};

/* S(x) rational function for positive x */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* sin(pi x) with x > 0x1p-100, if sin(pi*x)==0 the sign is arbitrary */

static double sinpi(double x)
{
  int n;

  /* argument reduction: x = |x| mod 2 */

  /* spurious inexact when x is odd int */

  x = x * 0.5;
  x = 2 * (x - floor(x));

  /* reduce x into [-.25,.25] */

  n = 4 * x;
  n = (n + 1) / 2;
  x -= n * 0.5;

  x *= pi;
  switch (n)
    {
    default:                   /* case 4 */
    case 0:
      return __sin(x, 0, 0);

    case 1:
      return __cos(x, 0);

    case 2:
      return __sin(-x, 0, 0);

    case 3:
      return -__cos(x, 0);
    }
}

static double s(double x)
{
  double num = 0;
  double den = 0;
  int i;

  /* to avoid overflow handle large x differently */

  if (x < 8)
    {
      for (i = N; i >= 0; i--)
        {
          num = num * x + g_snum[i];
          den = den * x + g_sden[i];
        }
    }
  else
    {
      for (i = 0; i <= N; i++)
        {
          num = num / x + g_snum[i];
          den = den / x + g_sden[i];
        }
    }

  return num / den;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

double tgamma(double x)
{
  union
    {
      double f;
      uint64_t i;
    } u;

  u.f = x;

  double absx;
  double y;
  double dy;
  double z;
  double r;
  uint32_t ix = u.i >> 32 & 0x7fffffff;
  int sign = u.i >> 63;

  /* special cases */

  if (ix >= 0x7ff00000)
    {
      /* tgamma(nan)=nan, tgamma(inf)=inf, tgamma(-inf)=nan with invalid */

      return x + INFINITY;
    }

  if (ix < (0x3ff - 54) << 20)
    {
      /* |x| < 2^-54: tgamma(x) ~ 1/x, +-0 raises div-by-zero */

      return 1 / x;
    }

  /* integer arguments */

  /* raise inexact when non-integer */

  if (x == floor(x))
    {
      if (sign)
        {
          return NAN;
        }

      if (x <= sizeof g_fact / sizeof *g_fact)
        {
          return g_fact[(int)x - 1];
        }
    }

  /* x >= 172: tgamma(x)=inf with overflow */

  /* x =< -184: tgamma(x)=+-0 with underflow */

  if (ix >= 0x40670000)
    {
      /* |x| >= 184 */

      if (sign)
        {
          FORCE_EVAL((float)(ldexp(1.0, -126) / x));

          if (floor(x) * 0.5 == floor(x * 0.5))
            {
              return 0;
            }

          return -0.0;
        }

      x *= 0x1p1023;
      return x;
    }

  absx = sign ? -x : x;

  /* handle the error of x + g - 0.5 */

  y = absx + g_gmhalf;
  if (absx > g_gmhalf)
    {
      dy = y - absx;
      dy -= g_gmhalf;
    }
  else
    {
      dy = y - g_gmhalf;
      dy -= absx;
    }

  z = absx - 0.5;
  r = s(absx) * exp(-y);
  if (x < 0)
    {
      /* reflection formula for negative x */

      /* sinpi(absx) is not 0, integers are already handled */

      r = -pi / (sinpi(absx) * absx * r);
      dy = -dy;
      z = -z;
    }

  r += dy * (g_gmhalf + 0.5) * r / y;
  z = pow(y, 0.5 * z);
  y = r * z * z;

  return y;
}

double gamma(double x)
{
  return tgamma(x);
}
#endif
