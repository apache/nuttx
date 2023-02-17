/****************************************************************************
 * libs/libm/libm/lib_lgamma.c
 *
 *   Copyright (C) 1993 by Sun Microsystems, Inc. All rights reserved.
 *
 *   Developed at SunSoft, a Sun Microsystems, Inc. business.
 *   Permission to use, copy, modify, and distribute this
 *   software is freely granted, provided that this notice
 *   is preserved.
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

/* lgamma_r(x, signgamp)
 *
 * Reentrant version of the logarithm of the Gamma function
 * with user provide pointer for the sign of Gamma(x).
 *
 * Method:
 *   1. Argument Reduction for 0 < x <= 8
 *      Since gamma(1+s)=s*gamma(s), for x in [0,8], we may
 *      reduce x to a number in [1.5,2.5] by
 *              lgamma(1+s) = log(s) + lgamma(s)
 *      for example,
 *              lgamma(7.3) = log(6.3) + lgamma(6.3)
 *                          = log(6.3*5.3) + lgamma(5.3)
 *                          = log(6.3*5.3*4.3*3.3*2.3) + lgamma(2.3)
 *   2. Polynomial approximation of lgamma around its
 *      minimun ymin=1.461632144968362245 to maintain monotonicity.
 *      On [ymin-0.23, ymin+0.27] (i.e., [1.23164,1.73163]), use
 *              Let z = x-ymin;
 *              lgamma(x) = -1.214862905358496078218 + z^2*poly(z)
 *      where
 *              poly(z) is a 14 degree polynomial.
 *   2. Rational approximation in the primary interval [2,3]
 *      We use the following approximation:
 *              s = x-2.0;
 *              lgamma(x) = 0.5*s + s*P(s)/Q(s)
 *      with accuracy
 *              |P/Q - (lgamma(x)-0.5s)| < 2**-61.71
 *      Our algorithms are based on the following observation
 *
 *                             zeta(2)-1    2    zeta(3)-1    3
 * lgamma(2+s) = s*(1-Euler) + --------- * s  -  --------- * s  + ...
 *                                 2                 3
 *
 *      where Euler = 0.5771... is the Euler constant, which is very
 *      close to 0.5.
 *
 *   3. For x>=8, we have
 *      lgamma(x)~(x-0.5)log(x)-x+0.5*log(2pi)+1/(12x)-1/(360x**3)+....
 *      (better formula:
 *         lgamma(x)~(x-0.5)*(log(x)-1)-.5*(log(2pi)-1) + ...)
 *      Let z = 1/x, then we approximation
 *              f(z) = lgamma(x) - (x-0.5)(log(x)-1)
 *      by
 *                                  3       5             11
 *              w = w0 + w1*z + w2*z  + w3*z  + ... + w6*z
 *      where
 *              |w - f(z)| < 2**-58.74
 *
 *   4. For negative x, since (G is gamma function)
 *              -x*G(-x)*G(x) = pi/sin(pi*x),
 *      we have
 *              G(x) = pi/(sin(pi*x)*(-x)*G(-x))
 *      since G(-x) is positive, sign(G(x)) = sign(sin(pi*x)) for x<0
 *      Hence, for x<0, signgam = sign(sin(pi*x)) and
 *              lgamma(x) = log(|Gamma(x)|)
 *                        = log(pi/(|x*sin(pi*x)|)) - lgamma(-x);
 *      Note: one should avoid compute pi*(-x) directly in the
 *            computation of sin(pi*(-x)).
 *
 *   5. Special Cases
 *             lgamma(2+s) ~ s*(1-Euler) for tiny s
 *             lgamma(1) = lgamma(2) = 0
 *             lgamma(x) ~ -log(|x|) for tiny x
 *             lgamma(0) = lgamma(neg.integer) = inf and raise divide-by-zero
 *             lgamma(inf) = inf
 *             lgamma(-inf) = inf (bug for bug compatible with C99!?)
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
 * Private Data
 ****************************************************************************/

static int g_signgam = 0;

static const double g_pi  =  3.14159265358979311600e+00; /* 0x400921FB, 0x54442D18 */
static const double g_a0  =  7.72156649015328655494e-02; /* 0x3FB3C467, 0xE37DB0C8 */
static const double g_a1  =  3.22467033424113591611e-01; /* 0x3FD4A34C, 0xC4A60FAD */
static const double g_a2  =  6.73523010531292681824e-02; /* 0x3FB13E00, 0x1A5562A7 */
static const double g_a3  =  2.05808084325167332806e-02; /* 0x3F951322, 0xAC92547B */
static const double g_a4  =  7.38555086081402883957e-03; /* 0x3F7E404F, 0xB68FEFE8 */
static const double g_a5  =  2.89051383673415629091e-03; /* 0x3F67ADD8, 0xCCB7926B */
static const double g_a6  =  1.19270763183362067845e-03; /* 0x3F538A94, 0x116F3F5D */
static const double g_a7  =  5.10069792153511336608e-04; /* 0x3F40B6C6, 0x89B99C00 */
static const double g_a8  =  2.20862790713908385557e-04; /* 0x3F2CF2EC, 0xED10E54D */
static const double g_a9  =  1.08011567247583939954e-04; /* 0x3F1C5088, 0x987DFB07 */
static const double g_a10 =  2.52144565451257326939e-05; /* 0x3EFA7074, 0x428CFA52 */
static const double g_a11 =  4.48640949618915160150e-05; /* 0x3F07858E, 0x90A45837 */
static const double g_tc  =  1.46163214496836224576e+00; /* 0x3FF762D8, 0x6356BE3F */
static const double g_tf  = -1.21486290535849611461e-01; /* 0xBFBF19B9, 0xBCC38A42 */

/* tt = -(tail of tf) */

static const double g_tt  = -3.63867699703950536541e-18; /* 0xBC50C7CA, 0xA48A971F */
static const double g_t0  =  4.83836122723810047042e-01; /* 0x3FDEF72B, 0xC8EE38A2 */
static const double g_t1  = -1.47587722994593911752e-01; /* 0xBFC2E427, 0x8DC6C509 */
static const double g_t2  =  6.46249402391333854778e-02; /* 0x3FB08B42, 0x94D5419B */
static const double g_t3  = -3.27885410759859649565e-02; /* 0xBFA0C9A8, 0xDF35B713 */
static const double g_t4  =  1.79706750811820387126e-02; /* 0x3F9266E7, 0x970AF9EC */
static const double g_t5  = -1.03142241298341437450e-02; /* 0xBF851F9F, 0xBA91EC6A */
static const double g_t6  =  6.10053870246291332635e-03; /* 0x3F78FCE0, 0xE370E344 */
static const double g_t7  = -3.68452016781138256760e-03; /* 0xBF6E2EFF, 0xB3E914D7 */
static const double g_t8  =  2.25964780900612472250e-03; /* 0x3F6282D3, 0x2E15C915 */
static const double g_t9  = -1.40346469989232843813e-03; /* 0xBF56FE8E, 0xBF2D1AF1 */
static const double g_t10 =  8.81081882437654011382e-04; /* 0x3F4CDF0C, 0xEF61A8E9 */
static const double g_t11 = -5.38595305356740546715e-04; /* 0xBF41A610, 0x9C73E0EC */
static const double g_t12 =  3.15632070903625950361e-04; /* 0x3F34AF6D, 0x6C0EBBF7 */
static const double g_t13 = -3.12754168375120860518e-04; /* 0xBF347F24, 0xECC38C38 */
static const double g_t14 =  3.35529192635519073543e-04; /* 0x3F35FD3E, 0xE8C2D3F4 */
static const double g_u0  = -7.72156649015328655494e-02; /* 0xBFB3C467, 0xE37DB0C8 */
static const double g_u1  =  6.32827064025093366517e-01; /* 0x3FE4401E, 0x8B005DFF */
static const double g_u2  =  1.45492250137234768737e+00; /* 0x3FF7475C, 0xD119BD6F */
static const double g_u3  =  9.77717527963372745603e-01; /* 0x3FEF4976, 0x44EA8450 */
static const double g_u4  =  2.28963728064692451092e-01; /* 0x3FCD4EAE, 0xF6010924 */
static const double g_u5  =  1.33810918536787660377e-02; /* 0x3F8B678B, 0xBF2BAB09 */
static const double g_v1  =  2.45597793713041134822e+00; /* 0x4003A5D7, 0xC2BD619C */
static const double g_v2  =  2.12848976379893395361e+00; /* 0x40010725, 0xA42B18F5 */
static const double g_v3  =  7.69285150456672783825e-01; /* 0x3FE89DFB, 0xE45050AF */
static const double g_v4  =  1.04222645593369134254e-01; /* 0x3FBAAE55, 0xD6537C88 */
static const double g_v5  =  3.21709242282423911810e-03; /* 0x3F6A5ABB, 0x57D0CF61 */
static const double g_s0  = -7.72156649015328655494e-02; /* 0xBFB3C467, 0xE37DB0C8 */
static const double g_s1  =  2.14982415960608852501e-01; /* 0x3FCB848B, 0x36E20878 */
static const double g_s2  =  3.25778796408930981787e-01; /* 0x3FD4D98F, 0x4F139F59 */
static const double g_s3  =  1.46350472652464452805e-01; /* 0x3FC2BB9C, 0xBEE5F2F7 */
static const double g_s4  =  2.66422703033638609560e-02; /* 0x3F9B481C, 0x7E939961 */
static const double g_s5  =  1.84028451407337715652e-03; /* 0x3F5E26B6, 0x7368F239 */
static const double g_s6  =  3.19475326584100867617e-05; /* 0x3F00BFEC, 0xDD17E945 */
static const double g_r1  =  1.39200533467621045958e+00; /* 0x3FF645A7, 0x62C4AB74 */
static const double g_r2  =  7.21935547567138069525e-01; /* 0x3FE71A18, 0x93D3DCDC */
static const double g_r3  =  1.71933865632803078993e-01; /* 0x3FC601ED, 0xCCFBDF27 */
static const double g_r4  =  1.86459191715652901344e-02; /* 0x3F9317EA, 0x742ED475 */
static const double g_r5  =  7.77942496381893596434e-04; /* 0x3F497DDA, 0xCA41A95B */
static const double g_r6  =  7.32668430744625636189e-06; /* 0x3EDEBAF7, 0xA5B38140 */
static const double g_w0  =  4.18938533204672725052e-01; /* 0x3FDACFE3, 0x90C97D69 */
static const double g_w1  =  8.33333333333329678849e-02; /* 0x3FB55555, 0x5555553B */
static const double g_w2  = -2.77777777728775536470e-03; /* 0xBF66C16C, 0x16B02E5C */
static const double g_w3  =  7.93650558643019558500e-04; /* 0x3F4A019F, 0x98CF38B6 */
static const double g_w4  = -5.95187557450339963135e-04; /* 0xBF4380CB, 0x8C0FE741 */
static const double g_w5  =  8.36339918996282139126e-04; /* 0x3F4B67BA, 0x4CDAD5D1 */
static const double g_w6  = -1.63092934096575273989e-03; /* 0xBF5AB89D, 0x0B9E43E4 */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* sin(pi*x) assuming x > 2^-100, if sin(pi*x)==0 the sign is arbitrary */

static double sin_pi(double x)
{
  int n;

  /* spurious inexact if odd int */

  x = 2.0 * (x * 0.5 - floor(x * 0.5)); /* x mod 2.0 */

  n = (int)(x * 4.0);
  n = (n + 1) / 2;
  x -= n * 0.5f;
  x *= g_pi;

  switch (n)
    {
    default:                   /* case 4: */
    case 0:
      return __sin(x, 0.0, 0);

    case 1:
      return __cos(x, 0.0);

    case 2:
      return __sin(-x, 0.0, 0);

    case 3:
      return -__cos(x, 0.0);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

double lgamma_r(double x, int *signgamp)
{
  union
    {
      double f;
      uint64_t i;
    } u;

  u.f = x;

  double t;
  double y;
  double z;
  double nadj = 0.0;
  double p;
  double p1;
  double p2;
  double p3;
  double q;
  double r;
  double w;
  uint32_t ix;
  int sign;
  int i;

  /* purge off +-inf, NaN, +-0, tiny and negative arguments */

  *signgamp = 1;
  sign = u.i >> 63;

  ix = u.i >> 32 & 0x7fffffff;
  if (ix >= 0x7ff00000)
    {
      return x * x;
    }

  /* |x|<2**-70, return -log(|x|) */

  if (ix < (0x3ff - 70) << 20)
    {
      if (sign)
        {
          x = -x;
          *signgamp = -1;
        }

      return -log(x);
    }

  if (sign)
    {
      x = -x;
      t = sin_pi(x);

      if (t == 0.0)
        {
          /* -integer */

          return 1.0 / (x - x);
        }

      if (t > 0.0)
        {
          *signgamp = -1;
        }
      else
        {
          t = -t;
        }

      nadj = log(g_pi / (t * x));
    }

  /* purge off 1 and 2 */

  if ((ix == 0x3ff00000 || ix == 0x40000000) && (uint32_t) u.i == 0)
    {
      r = 0;
    }
  else                          /* for x < 2.0 */
    {
      if (ix < 0x40000000)
        {
          if (ix <= 0x3feccccc)
            {
              /* lgamma(x) = lgamma(x+1)-log(x) */

              r = -log(x);

              if (ix >= 0x3fe76944)
                {
                  y = 1.0 - x;
                  i = 0;
                }
              else
                {
                  if (ix >= 0x3fcda661)
                    {
                      y = x - (g_tc - 1.0);
                      i = 1;
                    }
                  else
                    {
                      y = x;
                      i = 2;
                    }
                }
            }
          else
            {
              r = 0.0;

              if (ix >= 0x3ffbb4c3)
                {
                  /* [1.7316,2] */

                  y = 2.0 - x;
                  i = 0;
                }
              else
                {
                  if (ix >= 0x3ff3b4c4)
                    {
                      /* [1.23,1.73] */

                      y = x - g_tc;
                      i = 1;
                    }
                  else
                    {
                      y = x - 1.0;
                      i = 2;
                    }
                }
            }

          switch (i)
            {
            case 0:
              z = y * y;
              p1 = g_a0 + z * (g_a2 + z * (g_a4 +
                   z * (g_a6 + z * (g_a8 + z * g_a10))));
              p2 = z * (g_a1 + z * (g_a3 + z * (g_a5 +
                   z * (g_a7 + z * (g_a9 + z * g_a11)))));
              p = y * p1 + p2;
              r += (p - 0.5 * y);
              break;

            case 1:
              z = y*y;
              w = z*y;
              p1 = g_t0 + w * (g_t3 + w *(g_t6 + w * (g_t9 + w * g_t12))); /* parallel comp */
              p2 = g_t1 + w * (g_t4 + w *(g_t7 + w * (g_t10 + w * g_t13)));
              p3 = g_t2 + w * (g_t5 + w *(g_t8 + w * (g_t11 + w * g_t14)));
              p = z * p1 - (g_tt - w * (p2 + y * p3));
              r += g_tf + p;
              break;

            case 2:
              p1 = y * (g_u0 + y * (g_u1 + y * (g_u2 +
                   y * (g_u3 + y * (g_u4 + y * g_u5)))));
              p2 = 1.0 + y * (g_v1 + y * (g_v2 +
                   y * (g_v3 + y * (g_v4 + y * g_v5))));
              r += -0.5 * y + p1 / p2;
            }
        }
      else
        {
          if (ix < 0x40200000)
            {
              /* x < 8.0 */

              i = (int)x;
              y = x - (double)i;
              p = y * (g_s0 + y * (g_s1 + y * (g_s2 +
                  y * (g_s3 + y * (g_s4 + y * (g_s5 + y * g_s6))))));
              q = 1.0 + y * (g_r1 + y * (g_r2 +
                  y * (g_r3 + y * (g_r4 + y * (g_r5 + y * g_r6)))));
              r = 0.5 * y + p / q;
              z = 1.0;

              /* lgamma(1+s) = log(s) + lgamma(s) */

              switch (i)
                {
                case 7:
                  z *= y + 6.0; /* FALLTHRU */
                case 6:
                  z *= y + 5.0; /* FALLTHRU */
                case 5:
                  z *= y + 4.0; /* FALLTHRU */
                case 4:
                  z *= y + 3.0; /* FALLTHRU */
                case 3:
                  z *= y + 2.0; /* FALLTHRU */
                  r += log(z);
                  break;
                }
            }
          else
            {
              if (ix < 0x43900000)
                {
                  /* 8.0 <= x < 2**58 */

                  t = log(x);
                  z = 1.0 / x;
                  y = z * z;
                  w = g_w0 + z * (g_w1 + y * (g_w2 +
                      y * (g_w3 + y * (g_w4 + y * (g_w5 + y * g_w6)))));
                  r = (x - 0.5) * (t - 1.0) + w;
                }
              else
                {
                  /* 2**58 <= x <= inf */

                  r = x * (log(x) - 1.0);
                }
            }
        }
    }

  if (sign)
    {
      r = nadj - r;
    }

  return r;
}

double lgamma(double x)
{
  return lgamma_r(x, &g_signgam);
}
#endif
