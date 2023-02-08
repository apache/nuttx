/****************************************************************************
 * include/nuttx/lib/math.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_LIB_MATH_H
#define __INCLUDE_NUTTX_LIB_MATH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/* If CONFIG_ARCH_MATH_H is defined, then the top-level Makefile will copy
 * this header file to include/math.h where it will become the system math.h
 * header file.  In this case, the architecture specific code must provide
 * an arch/<architecture>/include/math.h file which will be included below:
 */

#ifdef CONFIG_ARCH_MATH_H
#  include <arch/math.h>

/* If CONFIG_LIBM is enabled, then the math library at lib/math will be
 * built.  This library was taken from the math library developed for the
 * Rhombus OS by Nick Johnson (https://github.com/nickbjohnson4224/rhombus).
 * The port or the Rhombus math library was contributed by Darcy Gong.
 */

#elif defined(CONFIG_LIBM)

/****************************************************************************
 * Copyright (C) 2009-2011 Nick Johnson <nickbjohnson4224 at gmail.com>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* General Constants ********************************************************/

#ifndef _HUGE_ENUF
#  define _HUGE_ENUF (1e+300)  /* _HUGE_ENUF*_HUGE_ENUF must overflow */
#endif

#define INFINITY   ((double)(_HUGE_ENUF * _HUGE_ENUF))
#define NAN        ((double)(INFINITY * 0.0F))
#define HUGE_VAL   INFINITY

#define INFINITY_F ((float)INFINITY)
#define NAN_F      ((float)(INFINITY * 0.0F))

#define INFINITY_L ((long double)INFINITY)
#define NAN_L      ((long double)(INFINITY * 0.0F))

#define isnan(x)   ((x) != (x))
#define isnanf(x)  ((x) != (x))
#define isnanl(x)  ((x) != (x))
#define isinf(x)   (((x) == INFINITY) || ((x) == -INFINITY))
#define isinff(x)  (((x) == INFINITY_F) || ((x) == -INFINITY_F))
#define isinfl(x)  (((x) == INFINITY_L) || ((x) == -INFINITY_L))

#define finite(x)  (!(isinf(x) || isnan(x)))
#define finitef(x) (!(isinff(x) || isnanf(x)))
#define finitel(x) (!(isinfl(x) || isnanl(x)))

#define isfinite(x) \
  (sizeof(x) == sizeof(float) ? finitef(x) : \
   sizeof(x) == sizeof(double) ? finite(x) : finitel(x))

/* Exponential and Logarithmic constants ************************************/

#define M_E        2.7182818284590452353602874713526625
#define M_SQRT2    1.4142135623730950488016887242096981
#define M_SQRT1_2  0.7071067811865475244008443621048490
#define M_LOG2E    1.4426950408889634073599246810018921
#define M_LOG10E   0.4342944819032518276511289189166051
#define M_LN2      0.6931471805599453094172321214581765
#define M_LN10     2.3025850929940456840179914546843642

/* Trigonometric Constants **************************************************/

#define M_PI       3.1415926535897932384626433832795029
#define M_PI_2     1.5707963267948966192313216916397514
#define M_PI_4     0.7853981633974483096156608458198757
#define M_1_PI     0.3183098861837906715377675267450287
#define M_2_PI     0.6366197723675813430755350534900574
#define M_2_SQRTPI 1.1283791670955125738961589031215452

#define M_PI_F     ((float)M_PI)
#define M_PI_2_F   ((float)M_PI_2)

/****************************************************************************
 * Type Declarations
 ****************************************************************************/

/* Floating point types */

typedef float        float_t;
#ifndef CONFIG_HAVE_DOUBLE
typedef float        double_t;
#else
typedef double       double_t;
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#if defined(__cplusplus)
extern "C"
{
#endif

/* General Functions ********************************************************/

float       ceilf (float x);
#ifdef CONFIG_HAVE_DOUBLE
double      ceil  (double x);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double ceill (long double x);
#endif

float       floorf(float x);
#ifdef CONFIG_HAVE_DOUBLE
double      floor (double x);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double floorl(long double x);
#endif

float       roundf(float x);
#ifdef CONFIG_HAVE_DOUBLE
double      round (double x);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double roundl(long double x);
#endif

long int    lroundf(float x);
#ifdef CONFIG_HAVE_DOUBLE
long int    lround(double x);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long int    lroundl(long double x);
#endif

#ifdef CONFIG_HAVE_LONG_LONG
long long int llroundf(float x);
#ifdef CONFIG_HAVE_DOUBLE
long long int llround (double x);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long long int llroundl(long double x);
#endif
#endif

float       rintf(float x);      /* Not implemented */
#ifdef CONFIG_HAVE_DOUBLE
double      rint(double x);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double rintl(long double x); /* Not implemented */
#endif

long int    lrintf(float x);
#ifdef CONFIG_HAVE_DOUBLE
long int    lrint(double x);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long int    lrintl(long double x);
#endif

#ifdef CONFIG_HAVE_LONG_LONG
long long int llrintf(float x);
#ifdef CONFIG_HAVE_DOUBLE
long long int llrint(double x);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long long int llrintl(long double x);
#endif
#endif

float       fabsf (float x);
#ifdef CONFIG_HAVE_DOUBLE
double      fabs  (double x);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double fabsl (long double x);
#endif

float       modff (float x, float *iptr);
#ifdef CONFIG_HAVE_DOUBLE
double      modf  (double x, double *iptr);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double modfl (long double x, long double *iptr);
#endif

float       fmodf (float x, float div);
#ifdef CONFIG_HAVE_DOUBLE
double      fmod  (double x, double div);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double fmodl (long double x, long double div);
#endif

/* Exponential and Logarithmic Functions ************************************/

float       powf  (float b, float e);
#ifdef CONFIG_HAVE_DOUBLE
double      pow   (double b, double e);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double powl  (long double b, long double e);
#endif

float       expf  (float x);
float       exp2f (float x);
float       expm1f(float x);
#ifdef CONFIG_HAVE_DOUBLE
double      exp   (double x);
double      exp2  (double x);
double      expm1 (double x);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double expl  (long double x);
long double exp2l (long double x);
long double expm1l(long double x);
#endif

float       fdimf(float x, float y);
#ifdef CONFIG_HAVE_DOUBLE
double      fdim(double x, double y);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double fdiml(long double x, long double y);
#endif

float       fmaf(float x, float y, float z);
#ifdef CONFIG_HAVE_DOUBLE
double      fma(double x, double y, double z);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double fmal(long double x, long double y, long double z);
#endif

float       fmaxf(float x, float y);
#ifdef CONFIG_HAVE_DOUBLE
double      fmax(double x, double y);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double fmaxl(long double x, long double y);
#endif

float       fminf(float x, float y);
#ifdef CONFIG_HAVE_DOUBLE
double      fmin(double x, double y);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double fminl(long double x, long double y);
#endif

float       hypotf(float x, float y);
#ifdef CONFIG_HAVE_DOUBLE
double      hypot(double x, double y);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double hypotl(long double x, long double y);
#endif

float       lgammaf(float x);
#ifdef CONFIG_HAVE_DOUBLE
double      __cos(double x, double y);
double      __sin(double x, double y, int iy);
double      gamma(double x);
double      lgamma(double x);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double lgammal(long double x);
#endif

float       tgammaf(float x);
#ifdef CONFIG_HAVE_DOUBLE
double      tgamma(double x);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double tgammal(long double x);
#endif

float       logf  (float x);
#ifdef CONFIG_HAVE_DOUBLE
double      log   (double x);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double logl  (long double x);
#endif

float       log10f(float x);
#ifdef CONFIG_HAVE_DOUBLE
double      log10 (double x);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double log10l(long double x);
#endif

float       log1pf(float x);
#ifdef CONFIG_HAVE_DOUBLE
double      log1p (double x);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double log1pl(long double x);
#endif

float       log2f (float x);
#ifdef CONFIG_HAVE_DOUBLE
double      log2  (double x);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double log2l (long double x);
#endif

float       logbf (float x);
#ifdef CONFIG_HAVE_DOUBLE
double      logb  (double x);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double logbl (long double x);
#endif

int         ilogbf (float x);
#ifdef CONFIG_HAVE_DOUBLE
int         ilogb  (double x);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
int         ilogbl (long double x);
#endif

float       sqrtf (float x);
#ifdef CONFIG_HAVE_DOUBLE
double      sqrt  (double x);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double sqrtl (long double x);
#endif

float       ldexpf(float x, int n);
#ifdef CONFIG_HAVE_DOUBLE
double      ldexp (double x, int n);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double ldexpl(long double x, int n);
#endif

float       frexpf(float x, int *exp);
#ifdef CONFIG_HAVE_DOUBLE
double      frexp (double x, int *exp);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double frexpl(long double x, int *exp);
#endif

/* Trigonometric Functions **************************************************/

void        sincosf(float, float *, float *);
#ifdef CONFIG_HAVE_DOUBLE
void        sincos(double, double *, double *);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
void        sincosl(long double, long double *, long double *);
#endif

float       sinf  (float x);
#ifdef CONFIG_HAVE_DOUBLE
double      sin   (double x);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double sinl  (long double x);
#endif

float       cosf  (float x);
#ifdef CONFIG_HAVE_DOUBLE
double      cos   (double x);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double cosl  (long double x);
#endif

float       tanf  (float x);
#ifdef CONFIG_HAVE_DOUBLE
double      tan   (double x);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double tanl  (long double x);
#endif

float       asinf (float x);
#ifdef CONFIG_HAVE_DOUBLE
double      asin  (double x);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double asinl (long double x);
#endif

float       acosf (float x);
#ifdef CONFIG_HAVE_DOUBLE
double      acos  (double x);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double acosl (long double x);
#endif

float       atanf (float x);
#ifdef CONFIG_HAVE_DOUBLE
double      atan  (double x);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double atanl (long double x);
#endif

float       atan2f(float y, float x);
#ifdef CONFIG_HAVE_DOUBLE
double      atan2 (double y, double x);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double atan2l(long double y, long double x);
#endif

float       sinhf (float x);
#ifdef CONFIG_HAVE_DOUBLE
double      sinh  (double x);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double sinhl (long double x);
#endif

float       coshf (float x);
#ifdef CONFIG_HAVE_DOUBLE
double      cosh  (double x);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double coshl (long double x);
#endif

float       cbrtf (float x);
#ifdef CONFIG_HAVE_DOUBLE
double      cbrt  (double x);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double cbrtl (long double x);
#endif

float       tanhf (float x);
#ifdef CONFIG_HAVE_DOUBLE
double      tanh  (double x);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double tanhl (long double x);
#endif

float       asinhf (float x);
#ifdef CONFIG_HAVE_DOUBLE
double      asinh  (double x);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double asinhl (long double x);
#endif

float       acoshf (float x);
#ifdef CONFIG_HAVE_DOUBLE
double      acosh  (double x);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double acoshl (long double x);
#endif

float       atanhf (float x);
#ifdef CONFIG_HAVE_DOUBLE
double      atanh  (double x);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double atanhl (long double x);
#endif

float       erff (float x);
float       erfcf(float x);
#ifdef CONFIG_HAVE_DOUBLE
double      erf  (double x);
double      erfc(double x);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double erfl (long double x);
long double erfcl(long double x);
#endif

float       copysignf (float x, float y);
#ifdef CONFIG_HAVE_DOUBLE
double      copysign  (double x, double y);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double copysignl (long double x, long double y);
#endif

float       truncf (float x);
#ifdef CONFIG_HAVE_DOUBLE
double      trunc (double x);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double truncl (long double x);
#endif

float       nanf(const char *tagp);
#ifdef CONFIG_HAVE_DOUBLE
double      nan(const char *tagp);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double nanl(const char *tagp);
#endif

float       nearbyintf(float x);
#ifdef CONFIG_HAVE_DOUBLE
double      nearbyint(double x);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double nearbyintl(long double x);
#endif

float       nextafterf(float x, float y);
#ifdef CONFIG_HAVE_DOUBLE
double      nextafter(double x, double y);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double nextafterl(long double x, long double y);
#endif

float       nexttowardf(float x, long double y);
#ifdef CONFIG_HAVE_DOUBLE
double      nexttoward(double x, long double y);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double nexttowardl(long double x, long double y);
#endif

float       remainderf(float x, float y);
#ifdef CONFIG_HAVE_DOUBLE
double      remainder(double x, double y);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double remainderl(long double x, long double y);
#endif

float       remquof(float x, float y, int *quo);
#ifdef CONFIG_HAVE_DOUBLE
double      remquo(double x, double y, int *quo);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double remquol(long double x, long double y, int *quo);
#endif

float       scalblnf(float x, long int n);
#ifdef CONFIG_HAVE_DOUBLE
double      scalbln(double x, long int n);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double scalblnl(long double x, long int n);
#endif

float       scalbnf(float x, int n);
#ifdef CONFIG_HAVE_DOUBLE
double      scalbn(double x, int n);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double scalbnl(long double x, int n);
#endif

#define FP_INFINITE     0
#define FP_NAN          1
#define FP_NORMAL       2
#define FP_SUBNORMAL    3
#define FP_ZERO         4
#define fpclassify(x) \
    __builtin_fpclassify(FP_NAN, FP_INFINITE, FP_NORMAL, FP_SUBNORMAL, \
                         FP_ZERO, x)

#define isunordered(x, y)    __builtin_isunordered(x, y)
#define isgreater(x, y)      __builtin_isgreater(x, y)
#define isgreaterequal(x, y) __builtin_isgreaterequal(x, y)
#define isless(x, y)         __builtin_isless(x, y)
#define islessequal(x, y)    __builtin_islessequal(x, y)
#define islessgreater(x, y)  __builtin_islessgreater(x, y)
#define isnormal(x)          __builtin_isnormal(x)
#define signbit(x)           __builtin_signbit(x)

#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_LIBM */
#endif /* __INCLUDE_NUTTX_LIB_MATH_H */
