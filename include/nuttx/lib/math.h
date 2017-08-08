/****************************************************************************
 * include/nuttx/lib/math.h
 *
 *   Copyright (C) 2009, 2012, 2014-2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#define INFINITY    (1.0/0.0)
#define NAN         (0.0/0.0)
#define HUGE_VAL    INFINITY

#define INFINITY_F  (1.0F/0.0F)
#define NAN_F       (0.0F/0.0F)

#define isnan(x)    ((x) != (x))
#define isinf(x)    (((x) == INFINITY) || ((x) == -INFINITY))
#define isfinite(x) (!(isinf(x) || isnan(x)))

#define isinf_f(x)  (((x) == INFINITY_F) || ((x) == -INFINITY_F))

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

#define lroundf(x) ((long)roundf(x))
#ifdef CONFIG_HAVE_DOUBLE
#define lround(x)  ((long)round(x))
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
#define lroundl(x) ((long)roundl(x))
#endif

#ifdef CONFIG_HAVE_LONG_LONG
#define llroundf(x) ((long long)roundf(x))
#ifdef CONFIG_HAVE_DOUBLE
#define llround(x)  ((long long)round(x))
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
#define llroundl(x) ((long long)roundl(x))
#endif
#endif

float       rintf(float x);      /* Not implemented */
#ifdef CONFIG_HAVE_DOUBLE
double      rint(double x);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double rintl(long double x); /* Not implemented */
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
#define expm1f(x) (expf(x) - 1.0)
#ifdef CONFIG_HAVE_DOUBLE
double      exp   (double x);
#define expm1(x) (exp(x) - 1.0)
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double expl  (long double x);
#define expm1l(x) (expl(x) - 1.0)
#endif

#ifdef CONFIG_HAVE_DOUBLE
double      __cos(double x, double y);
double      __sin(double x, double y, int iy);
double      gamma(double x);
double      lgamma(double x);
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

float       log2f (float x);
#ifdef CONFIG_HAVE_DOUBLE
double      log2  (double x);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double log2l (long double x);
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
#define     erfcf(x) (1 - erff(x))
#ifdef CONFIG_HAVE_DOUBLE
double      erf  (double x);
#define     erfc(x) (1 - erf(x))
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double erfl (long double x);
#define     erfcl(x) (1 - erfl(x))
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

#define nanf(x) ((float)(NAN))
#ifdef CONFIG_HAVE_DOUBLE
#define nan(x) ((double)(NAN))
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
#define nanl(x) ((long double)(NAN))
#endif

#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_LIBM */
#endif /* __INCLUDE_NUTTX_LIB_MATH_H */
