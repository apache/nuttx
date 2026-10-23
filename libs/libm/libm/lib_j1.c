/****************************************************************************
 * libs/libm/libm/lib_j1.c
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Copyright (C) 1993 by Sun Microsystems, Inc.
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

/* j1(x), y1(x)
 *
 * Bessel function of the first and second kinds of order one.
 *
 * Method -- j1(x):
 *   1. For tiny x, we use j1(x) = x/2 - x^3/16 + x^5/384 - ...
 *   2. Reduce x to |x| since j1(x)=-j1(-x),  and
 *      for x in (0,2)
 *              j1(x) = x/2 + x*z*R0/S0,  where z = x*x;
 *      (precision:  |j1/x - 1/2 - R0/S0 |<2**-61.51 )
 *      for x in (2,inf)
 *              j1(x) = sqrt(2/(pi*x))*(p1(x)*cos(x1)-q1(x)*sin(x1))
 *              y1(x) = sqrt(2/(pi*x))*(p1(x)*sin(x1)+q1(x)*cos(x1))
 *      where x1 = x-3*pi/4. It is better to compute sin(x1),cos(x1)
 *      as follow:
 *              cos(x1) =  cos(x)cos(3pi/4)+sin(x)sin(3pi/4)
 *                      =  1/sqrt(2) * (sin(x) - cos(x))
 *              sin(x1) =  sin(x)cos(3pi/4)-cos(x)sin(3pi/4)
 *                      = -1/sqrt(2) * (sin(x) + cos(x))
 *      (To avoid cancellation, use
 *              sin(x) +- cos(x) = -cos(2x)/(sin(x) -+ cos(x))
 *      to compute the worse one.)
 *
 *   3. Special cases
 *              j1(nan)= nan
 *              j1(0) = 0
 *              j1(inf) = 0
 *
 * Method -- y1(x):
 *   1. screen out x<=0 cases: y1(0)=-inf, y1(x<0)=NaN
 *   2. For x<2.
 *      Since
 *              y1(x) = 2/pi*(j1(x)*(ln(x/2)+Euler)-1/x-x/2+5/64*x^3-...)
 *      therefore y1(x)-2/pi*j1(x)*ln(x)-1/x is an odd function.
 *      We use the following function to approximate y1,
 *              y1(x) = x*U(z)/V(z) + (2/pi)*(j1(x)*ln(x)-1/x), z= x^2
 *      where for x in [0,2] (abs err less than 2**-65.89)
 *              U(z) = U0[0] + U0[1]*z + ... + U0[4]*z^4
 *              V(z) = 1  + v0[0]*z + ... + v0[4]*z^5
 *      Note: For tiny x, 1/x dominate y1 and hence
 *              y1(tiny) = -2/pi/tiny, (choose tiny<2**-54)
 *   3. For x>=2.
 *              y1(x) = sqrt(2/(pi*x))*(p1(x)*sin(x1)+q1(x)*cos(x1))
 *      where x1 = x-3*pi/4. It is better to compute sin(x1),cos(x1)
 *      by method mentioned above.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>
#include <math.h>

#ifdef CONFIG_HAVE_DOUBLE

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const double g_invsqrtpi = 5.64189583547756279280e-01; /* 0x3FE20DD7, 0x50429B6D */
static const double g_tpi       = 6.36619772367581382433e-01; /* 0x3FE45F30, 0x6DC9C883 */

/* R0/S0 on [0,2] */

static const double g_r00 = -6.25000000000000000000e-02; /* 0xBFB00000, 0x00000000 */
static const double g_r01 =  1.40705666955189706048e-03; /* 0x3F570D9F, 0x98472C61 */
static const double g_r02 = -1.59955631084035597520e-05; /* 0xBEF0C5C6, 0xBA169668 */
static const double g_r03 =  4.96727999609584448412e-08; /* 0x3E6AAAFA, 0x46CA0BD9 */
static const double g_s01 =  1.91537599538363460805e-02; /* 0x3F939D0B, 0x12637E53 */
static const double g_s02 =  1.85946785588630915560e-04; /* 0x3F285F56, 0xB9CDF664 */
static const double g_s03 =  1.17718464042623683263e-06; /* 0x3EB3BFF8, 0x333F8498 */
static const double g_s04 =  5.04636257076217042715e-09; /* 0x3E35AC88, 0xC97DFF2C */
static const double g_s05 =  1.23542274426137913908e-11; /* 0x3DAB2ACF, 0xCFB97ED8 */

/* U(z)/V(z) of y1() on [0,2] */

static const double g_u0[5] =
{
  -1.96057090646238940668e-01, /* 0xBFC91866, 0x143CBC8A */
  5.04438716639811282616e-02,  /* 0x3FA9D3C7, 0x76292CD1 */
  -1.91256895875763547298e-03, /* 0xBF5F55E5, 0x4844F50F */
  2.35252600561610495928e-05,  /* 0x3EF8AB03, 0x8FA6B88E */
  -9.19099158039878874504e-08, /* 0xBE78AC00, 0x569105B8 */
};

static const double g_v0[5] =
{
  1.99167318236649903973e-02, /* 0x3F94650D, 0x3F4DA9F0 */
  2.02552581025135171496e-04, /* 0x3F2A8C89, 0x6C257764 */
  1.35608801097516229404e-06, /* 0x3EB6C05A, 0x894E8CA6 */
  6.22741452364621501295e-09, /* 0x3E3ABF1D, 0x5BA69A86 */
  1.66559246207992079114e-11, /* 0x3DB25039, 0xDACA772A */
};

/* For x >= 8, the asymptotic expansions of pone is
 *      1 + 15/128 s^2 - 4725/2^15 s^4 - ...,   where s = 1/x.
 * We approximate pone by
 *      pone(x) = 1 + (R/S)
 * where  R = pr0 + pr1*s^2 + pr2*s^4 + ... + pr5*s^10
 *        S = 1 + ps0*s^2 + ... + ps4*s^10
 * and
 *      | pone(x)-1-R/S | <= 2  ** ( -60.06)
 */

/* For x in [inf, 8]=1/[0,0.125] */

static const double g_pr8[6] =
{
  0.00000000000000000000e00,  /* 0x00000000, 0x00000000 */
  1.17187499999988647970e-01, /* 0x3FBDFFFF, 0xFFFFFCCE */
  1.32394806593073575129e01,  /* 0x402A7A9D, 0x357F7FCE */
  4.12051854307378562225e02,  /* 0x4079C0D4, 0x652EA590 */
  3.87474538913960532227e03,  /* 0x40AE457D, 0xA3A532CC */
  7.91447954031891731574e03,  /* 0x40BEEA7A, 0xC32782DD */
};

static const double g_ps8[5] =
{
  1.14207370375678408436e02, /* 0x405C8D45, 0x8E656CAC */
  3.65093083420853463394e03, /* 0x40AC85DC, 0x964D274F */
  3.69562060269033463555e04, /* 0x40E20B86, 0x97C5BB7F */
  9.76027935934950801311e04, /* 0x40F7D42C, 0xB28F17BB */
  3.08042720627888811578e04, /* 0x40DE1511, 0x697A0B2D */
};

/* For x in [8,4.5454]=1/[0.125,0.22001] */

static const double g_pr5[6] =
{
  1.31990519556243522749e-11, /* 0x3DAD0667, 0xDAE1CA7D */
  1.17187493190614097638e-01, /* 0x3FBDFFFF, 0xE2C10043 */
  6.80275127868432871736e00,  /* 0x401B3604, 0x6E6315E3 */
  1.08308182990189109773e02,  /* 0x405B13B9, 0x452602ED */
  5.17636139533199752805e02,  /* 0x40802D16, 0xD052D649 */
  5.28715201363337541807e02,  /* 0x408085B8, 0xBB7E0CB7 */
};

static const double g_ps5[5] =
{
  5.92805987221131331921e01, /* 0x404DA3EA, 0xA8AF633D */
  9.91401418733614377743e02, /* 0x408EFB36, 0x1B066701 */
  5.35326695291487976647e03, /* 0x40B4E944, 0x5706B6FB */
  7.84469031749551231769e03, /* 0x40BEA4B0, 0xB8A5BB15 */
  1.50404688810361062679e03, /* 0x40978030, 0x036F5E51 */
};

/* For x in [4.547,2.8571]=1/[0.2199,0.35001] */

static const double g_pr3[6] =
{
  3.02503916137373618024e-09, /* 0x3E29FC21, 0xA7AD9EDD */
  1.17186865567253592491e-01, /* 0x3FBDFFF5, 0x5B21D17B */
  3.93297750033315640650e00,  /* 0x400F76BC, 0xE85EAD8A */
  3.51194035591636932736e01,  /* 0x40418F48, 0x9DA6D129 */
  9.10550110750781271918e01,  /* 0x4056C385, 0x4D2C1837 */
  4.85590685197364919645e01,  /* 0x4048478F, 0x8EA83EE5 */
};

static const double g_ps3[5] =
{
  3.47913095001251519989e01, /* 0x40416549, 0xA134069C */
  3.36762458747825746741e02, /* 0x40750C33, 0x07F1A75F */
  1.04687139975775130551e03, /* 0x40905B7C, 0x5037D523 */
  8.90811346398256432622e02, /* 0x408BD67D, 0xA32E31E9 */
  1.03787932439639277504e02, /* 0x4059F26D, 0x7C2EED53 */
};

/* For x in [2.8570,2]=1/[0.3499,0.5] */

static const double g_pr2[6] =
{
  1.07710830106873743082e-07, /* 0x3E7CE9D4, 0xF65544F4 */
  1.17176219462683348094e-01, /* 0x3FBDFF42, 0xBE760D83 */
  2.36851496667608785174e00,  /* 0x4002F2B7, 0xF98FAEC0 */
  1.22426109148261232917e01,  /* 0x40287C37, 0x7F71A964 */
  1.76939711271687727390e01,  /* 0x4031B1A8, 0x177F8EE2 */
  5.07352312588818499250e00,  /* 0x40144B49, 0xA574C1FE */
};

static const double g_ps2[5] =
{
  2.14364859363821409488e01, /* 0x40356FBD, 0x8AD5ECDC */
  1.25290227168402751090e02, /* 0x405F5293, 0x14F92CD5 */
  2.32276469057162813669e02, /* 0x406D08D8, 0xD5A2DBD9 */
  1.17679373287147100768e02, /* 0x405D6B7A, 0xDA1884A9 */
  8.36463893371618283368e00, /* 0x4020BAB1, 0xF44E5192 */
};

/* For x >= 8, the asymptotic expansions of qone is
 *      3/8 s - 105/1024 s^3 - ..., where s = 1/x.
 * We approximate qone by
 *      qone(x) = s*(0.375 + (R/S))
 * where  R = qr1*s^2 + qr2*s^4 + ... + qr5*s^10
 *        S = 1 + qs1*s^2 + ... + qs6*s^12
 * and
 *      | qone(x)/s -0.375-R/S | <= 2  ** ( -61.13)
 */

/* For x in [inf, 8]=1/[0,0.125] */

static const double g_qr8[6] =
{
  0.00000000000000000000e00,   /* 0x00000000, 0x00000000 */
  -1.02539062499992714161e-01, /* 0xBFBA3FFF, 0xFFFFFDF3 */
  -1.62717534544589987888e01,  /* 0xC0304591, 0xA26779F7 */
  -7.59601722513950107896e02,  /* 0xC087BCD0, 0x53E4B576 */
  -1.18498066702429587167e04,  /* 0xC0C724E7, 0x40F87415 */
  -4.84385124285750353010e04,  /* 0xC0E7A6D0, 0x65D09C6A */
};

static const double g_qs8[6] =
{
  1.61395369700722909556e02,  /* 0x40642CA6, 0xDE5BCDE5 */
  7.82538599923348465381e03,  /* 0x40BE9162, 0xD0D88419 */
  1.33875336287249578163e05,  /* 0x4100579A, 0xB0B75E98 */
  7.19657723683240939863e05,  /* 0x4125F653, 0x72869C19 */
  6.66601232617776375264e05,  /* 0x412457D2, 0x7719AD5C */
  -2.94490264303834643215e05, /* 0xC111F969, 0x0EA5AA18 */
};

/* For x in [8,4.5454]=1/[0.125,0.22001] */

static const double g_qr5[6] =
{
  -2.08979931141764104297e-11, /* 0xBDB6FA43, 0x1AA1A098 */
  -1.02539050241375426231e-01, /* 0xBFBA3FFF, 0xCB597FEF */
  -8.05644828123936029840e00,  /* 0xC0201CE6, 0xCA03AD4B */
  -1.83669607474888380239e02,  /* 0xC066F56D, 0x6CA7B9B0 */
  -1.37319376065508163265e03,  /* 0xC09574C6, 0x6931734F */
  -2.61244440453215656817e03,  /* 0xC0A468E3, 0x88FDA79D */
};

static const double g_qs5[6] =
{
  8.12765501384335777857e01,  /* 0x405451B2, 0xFF5A11B2 */
  1.99179873460485964642e03,  /* 0x409F1F31, 0xE77BF839 */
  1.74684851924908907677e04,  /* 0x40D10F1F, 0x0D64CE29 */
  4.98514270910352279316e04,  /* 0x40E8576D, 0xAABAD197 */
  2.79480751638918118260e04,  /* 0x40DB4B04, 0xCF7C364B */
  -4.71918354795128470869e03, /* 0xC0B26F2E, 0xFCFFA004 */
};

/* For x in [4.547,2.8571]=1/[0.2199,0.35001] */

static const double g_qr3[6] =
{
  -5.07831226461766561369e-09, /* 0xBE35CFA9, 0xD38FC84F */
  -1.02537829820837089745e-01, /* 0xBFBA3FEB, 0x51AEED54 */
  -4.61011581139473403113e00,  /* 0xC01270C2, 0x3302D9FF */
  -5.78472216562783643212e01,  /* 0xC04CEC71, 0xC25D16DA */
  -2.28244540737631695038e02,  /* 0xC06C87D3, 0x4718D55F */
  -2.19210128478909325622e02,  /* 0xC06B66B9, 0x5F5C1BF6 */
};

static const double g_qs3[6] =
{
  4.76651550323729509273e01,  /* 0x4047D523, 0xCCD367E4 */
  6.73865112676699709482e02,  /* 0x40850EEB, 0xC031EE3E */
  3.38015286679526343505e03,  /* 0x40AA684E, 0x448E7C9A */
  5.54772909720722782367e03,  /* 0x40B5ABBA, 0xA61D54A6 */
  1.90311919338810798763e03,  /* 0x409DBC7A, 0x0DD4DF4B */
  -1.35201191444307340817e02, /* 0xC060E670, 0x290A311F */
};

/* For x in [2.8570,2]=1/[0.3499,0.5] */

static const double g_qr2[6] =
{
  -1.78381727510958865572e-07, /* 0xBE87F126, 0x44C626D2 */
  -1.02517042607985553460e-01, /* 0xBFBA3E8E, 0x9148B010 */
  -2.75220568278187460720e00,  /* 0xC0060484, 0x69BB4EDA */
  -1.96636162643703720221e01,  /* 0xC033A9E2, 0xC168907F */
  -4.23253133372830490089e01,  /* 0xC04529A3, 0xDE104AAA */
  -2.13719211703704061733e01,  /* 0xC0355F36, 0x39CF6E52 */
};

static const double g_qs2[6] =
{
  2.95333629060523854548e01,  /* 0x403D888A, 0x78AE64FF */
  2.52981549982190529136e02,  /* 0x406F9F68, 0xDB821CBA */
  7.57502834868645436472e02,  /* 0x4087AC05, 0xCE49A0F7 */
  7.39393205320467245656e02,  /* 0x40871B25, 0x48D4C029 */
  1.55949003336666123687e02,  /* 0x40637E5E, 0x3C3ED8D4 */
  -4.95949898822628210127e00, /* 0xC013D686, 0xE71BE86B */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* The high 32 bits (sign, exponent and the top of the fraction) and the
 * low 32 bits of a double.
 */

static uint32_t high_word(double x)
{
  union
    {
      double f;
      uint64_t i;
    } u;

  u.f = x;
  return (uint32_t)(u.i >> 32);
}

static uint32_t low_word(double x)
{
  union
    {
      double f;
      uint64_t i;
    } u;

  u.f = x;
  return (uint32_t)u.i;
}

static double pone(double x)
{
  FAR const double *p;
  FAR const double *q;
  double z;
  double r;
  double s;
  uint32_t ix;

  ix = high_word(x) & 0x7fffffff;
  if (ix >= 0x40200000)
    {
      p = g_pr8;
      q = g_ps8;
    }
  else if (ix >= 0x40122e8b)
    {
      p = g_pr5;
      q = g_ps5;
    }
  else if (ix >= 0x4006db6d)
    {
      p = g_pr3;
      q = g_ps3;
    }
  else
    {
      /* ix >= 0x40000000 */

      p = g_pr2;
      q = g_ps2;
    }

  z = 1.0 / (x * x);
  r = p[0] + z * (p[1] + z * (p[2] + z * (p[3] + z * (p[4] + z * p[5]))));
  s = 1.0 + z * (q[0] + z * (q[1] + z * (q[2] + z * (q[3] + z * q[4]))));
  return 1.0 + r / s;
}

static double qone(double x)
{
  FAR const double *p;
  FAR const double *q;
  double z;
  double r;
  double s;
  uint32_t ix;

  ix = high_word(x) & 0x7fffffff;
  if (ix >= 0x40200000)
    {
      p = g_qr8;
      q = g_qs8;
    }
  else if (ix >= 0x40122e8b)
    {
      p = g_qr5;
      q = g_qs5;
    }
  else if (ix >= 0x4006db6d)
    {
      p = g_qr3;
      q = g_qs3;
    }
  else
    {
      /* ix >= 0x40000000 */

      p = g_qr2;
      q = g_qs2;
    }

  z = 1.0 / (x * x);
  r = p[0] + z * (p[1] + z * (p[2] + z * (p[3] + z * (p[4] + z * p[5]))));
  s = 1.0 + z * (q[0] + z * (q[1] + z * (q[2] + z * (q[3] +
            z * (q[4] + z * q[5])))));
  return (0.375 + r / s) / x;
}

/* The common method of j1() and y1() for |x| >= 2:
 *
 *   j1(x) = sqrt(2/(pi*x))*(p1(x)*cos(x-3pi/4)-q1(x)*sin(x-3pi/4))
 *   y1(x) = sqrt(2/(pi*x))*(p1(x)*sin(x-3pi/4)+q1(x)*cos(x-3pi/4))
 *
 *   sin(x-3pi/4) = -(sin(x) + cos(x))/sqrt(2)
 *   cos(x-3pi/4) = (sin(x) - cos(x))/sqrt(2)
 *   sin(x) +- cos(x) = -cos(2x)/(sin(x) -+ cos(x))
 */

static double common(uint32_t ix, double x, int yflag, int sign)
{
  double z;
  double s;
  double c;
  double ss;
  double cc;

  s = sin(x);
  if (yflag)
    {
      s = -s;
    }

  c = cos(x);
  cc = s - c;

  /* Avoid overflow in 2*x */

  if (ix < 0x7fe00000)
    {
      ss = -s - c;
      z = cos(2 * x);
      if (s * c > 0)
        {
          cc = z / ss;
        }
      else
        {
          ss = z / cc;
        }

      if (ix < 0x48000000)
        {
          if (yflag)
            {
              ss = -ss;
            }

          cc = pone(x) * cc - qone(x) * ss;
        }
    }

  if (sign)
    {
      cc = -cc;
    }

  return g_invsqrtpi * cc / sqrt(x);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

double j1(double x)
{
  double z;
  double r;
  double s;
  uint32_t ix;
  int sign;

  ix = high_word(x);
  sign = ix >> 31;
  ix &= 0x7fffffff;

  if (ix >= 0x7ff00000)
    {
      return 1.0 / (x * x);
    }

  if (ix >= 0x40000000)
    {
      /* |x| >= 2 */

      return common(ix, fabs(x), 0, sign);
    }

  if (ix >= 0x38000000)
    {
      /* |x| >= 2**-127 */

      z = x * x;
      r = z * (g_r00 + z * (g_r01 + z * (g_r02 + z * g_r03)));
      s = 1.0 + z * (g_s01 + z * (g_s02 + z * (g_s03 + z * (g_s04 +
          z * g_s05))));
      z = r / s;
    }
  else
    {
      /* Avoid underflow, raise inexact if x != 0 */

      z = x;
    }

  return (0.5 + z) * x;
}

double y1(double x)
{
  double z;
  double u;
  double v;
  uint32_t ix;
  uint32_t lx;

  ix = high_word(x);
  lx = low_word(x);

  /* y1(nan) = nan, y1(<0) = nan, y1(0) = -inf, y1(inf) = 0 */

  if ((ix << 1 | lx) == 0)
    {
      return -INFINITY;
    }

  if (ix >> 31)
    {
      return NAN;
    }

  if (ix >= 0x7ff00000)
    {
      return 1.0 / x;
    }

  if (ix >= 0x40000000)
    {
      /* x >= 2 */

      return common(ix, x, 1, 0);
    }

  if (ix < 0x3c900000)
    {
      /* x < 2**-54 */

      return -g_tpi / x;
    }

  z = x * x;
  u = g_u0[0] + z * (g_u0[1] + z * (g_u0[2] + z * (g_u0[3] +
      z * g_u0[4])));
  v = 1.0 + z * (g_v0[0] + z * (g_v0[1] + z * (g_v0[2] + z * (g_v0[3] +
      z * g_v0[4]))));
  return x * (u / v) + g_tpi * (j1(x) * log(x) - 1.0 / x);
}
#endif
