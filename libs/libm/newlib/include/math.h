/****************************************************************************
 * libs/libm/newlib/include/math.h
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

#ifndef  _MATH_H_

#define  _MATH_H_

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <machine/ieeefp.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* Macro to test version of GCC.  Returns 0 for non-GCC or too old GCC. */
#ifndef __GNUC_PREREQ
# if defined __GNUC__ && defined __GNUC_MINOR__
#  define __GNUC_PREREQ(maj, min) \
        ((__GNUC__ << 16) + __GNUC_MINOR__ >= ((maj) << 16) + (min))
# else
#  define __GNUC_PREREQ(maj, min) 0
# endif
#endif /* __GNUC_PREREQ */

/* Newlib doesn't fully support long double math functions so far.
 * On platforms where long double equals double the long double functions
 * let's define _LDBL_EQ_DBL to simply call the double functions.
 */

#define _LDBL_EQ_DBL 1

extern _Thread_local int _tls_gamma_signgam;
#define _REENT_SIGNGAM(ptr) _tls_gamma_signgam

/* Natural log of 2 */
#define _M_LN2        0.693147180559945309417

#if __GNUC_PREREQ (3, 3)
/* gcc >= 3.3 implicitly defines builtins for HUGE_VALx values.  */

# ifndef HUGE_VAL
#  define HUGE_VAL (__builtin_huge_val())
# endif

# ifndef HUGE_VALF
#  define HUGE_VALF (__builtin_huge_valf())
# endif

# ifndef HUGE_VALL
#  define HUGE_VALL (__builtin_huge_vall())
# endif

# ifndef INFINITY
#  define INFINITY (__builtin_inff())
# endif

# ifndef NAN
#  define NAN (__builtin_nanf(""))
# endif

#else /* !gcc >= 3.3  */

/* No builtins.  Use fixed defines instead.  (All 3 HUGE plus the INFINITY
 * and NAN macros are required to be constant expressions. Using a variable--
 * even a static const--does not meet this requirement, as it cannot be
 * evaluated at translation time.)
 *      The infinities are done using numbers that are far in excess of
 * something that would be expected to be encountered in a floating-point
 * implementation.  (A more certain way uses values from float.h, but that is
 * avoided because system includes are not supposed to include each other.)
 *      This method might produce warnings from some compilers.  (It does in
 * newer GCCs, but not for ones that would hit this #else.)  If this happens,
 * please report details to the Newlib mailing list.
 */

 #ifndef HUGE_VAL
  #define HUGE_VAL (1.0e999999999)
 #endif

 #ifndef HUGE_VALF
  #define HUGE_VALF (1.0e999999999F)
 #endif

 #if !defined(HUGE_VALL)  &&  defined(_HAVE_LONG_DOUBLE)
  #define HUGE_VALL (1.0e999999999L)
 #endif

 #if !defined(INFINITY)
  #define INFINITY (HUGE_VALF)
 #endif

 #if !defined(NAN)
  #define NAN (0.0F/0.0F)
 #endif

#endif /* !gcc >= 3.3  */

/* Reentrant ANSI C functions.  */

#ifndef __math_68881
extern double atan (double);
extern double cos (double);
extern double sin (double);
extern double tan (double);
extern double tanh (double);
extern double frexp (double, int *);
extern double modf (double, double *);
extern double ceil (double);
extern double fabs (double);
extern double floor (double);
#endif /* ! defined (__math_68881) */

/* Non reentrant ANSI C functions.  */

#ifndef __math_68881
extern double acos (double);
extern double asin (double);
extern double atan2 (double, double);
extern double cosh (double);
extern double sinh (double);
extern double exp (double);
extern double ldexp (double, int);
extern double log (double);
extern double log10 (double);
extern double pow (double, double);
extern double sqrt (double);
extern double fmod (double, double);
#endif /* ! defined (__math_68881) */

extern int finite (double);
extern int finitef (float);
extern int finitel (long double);
extern int isinff (float);
extern int isnanf (float);
#ifdef __CYGWIN__ /* not implemented in newlib yet */
extern int isinfl (long double);
extern int isnanl (long double);
#endif
#if !defined(__cplusplus) || __cplusplus < 201103L
extern int isinf (double);
#endif
#if !defined(__cplusplus) || __cplusplus < 201103L
extern int isnan (double);
#endif

/* ISO C99 types and macros. */

/* FIXME:  FLT_EVAL_METHOD should somehow be gotten from float.h (which is
 * hard, considering that the standard says the includes it defines should
 * not include other includes that it defines) and that value used.  (This
 * can be solved, but autoconf has a bug which makes the solution more
 * difficult, so it has been skipped for now.)
 */
#if !defined(FLT_EVAL_METHOD) && defined(__FLT_EVAL_METHOD__)
  #define FLT_EVAL_METHOD __FLT_EVAL_METHOD__
  #define __TMP_FLT_EVAL_METHOD
#endif /* FLT_EVAL_METHOD */
#if defined FLT_EVAL_METHOD
/* FLT_EVAL_METHOD == 16 has meaning as defined in ISO/IEC TS 18661-3,
 * which provides non-compliant extensions to C and POSIX (by adding
 * additional positive values for FLT_EVAL_METHOD).  It effectively has
 * same meaning as the C99 and C11 definitions for value 0, while also
 * serving as a flag that the _Float16 (float16_t) type exists.
 *
 * FLT_EVAL_METHOD could be any number of bits of supported floating point
 * format (e.g. 32, 64, 128), but currently only AArch64 and few other
 * targets might define that as 16.
 */
  #if (FLT_EVAL_METHOD == 0) \
      || (FLT_EVAL_METHOD == 16)
    typedef float  float_t;
    typedef double double_t;
   #elif FLT_EVAL_METHOD == 1
    typedef double float_t;
    typedef double double_t;
   #elif FLT_EVAL_METHOD == 2
    typedef long double float_t;
    typedef long double double_t;
   #else
  /* Implementation-defined.  Assume float_t and double_t have been
   * defined previously for this configuration (e.g. config.h).
   */

  /* If __DOUBLE_TYPE is defined (__FLOAT_TYPE is then supposed to be
   * defined as well) float_t and double_t definition is suggested by
   * an arch specific header.
   */
   #ifdef __DOUBLE_TYPE
    typedef __DOUBLE_TYPE double_t;
    typedef __FLOAT_TYPE float_t;
   #endif
  /* Assume config.h has provided these types.  */
  #endif
#else
    /* Assume basic definitions.  */

    typedef float  float_t;
    typedef double double_t;
#endif
#if defined(__TMP_FLT_EVAL_METHOD)
  #undef FLT_EVAL_METHOD
#endif

#define FP_NAN         0
#define FP_INFINITE    1
#define FP_ZERO        2
#define FP_SUBNORMAL   3
#define FP_NORMAL      4

#ifndef FP_ILOGB0
#  define FP_ILOGB0 (-__INT_MAX__)
#endif
#ifndef FP_ILOGBNAN
#  define FP_ILOGBNAN __INT_MAX__
#endif

#ifndef MATH_ERRNO
#  define MATH_ERRNO 1
#endif
#ifndef MATH_ERREXCEPT
#  define MATH_ERREXCEPT 2
#endif
#ifndef math_errhandling
# ifdef _IEEE_LIBM
#  define _MATH_ERRHANDLING_ERRNO 0
# else
#  define _MATH_ERRHANDLING_ERRNO MATH_ERRNO
# endif
# ifdef _SUPPORTS_ERREXCEPT
#  define _MATH_ERRHANDLING_ERREXCEPT MATH_ERREXCEPT
# else
#  define _MATH_ERRHANDLING_ERREXCEPT 0
# endif
#  define math_errhandling (_MATH_ERRHANDLING_ERRNO | _MATH_ERRHANDLING_ERREXCEPT)
#endif

extern int __isinff (float);
extern int __isinfd (double);
extern int __isnanf (float);
extern int __isnand (double);
extern int __fpclassifyf (float);
extern int __fpclassifyd (double);
extern int __signbitf (float);
extern int __signbitd (double);

/* Note: isinf and isnan were once functions in newlib that took double
 *       arguments.  C99 specifies that these names are reserved for macros
 *       supporting multiple floating point types.  Thus, they are
 *       now defined as macros.  Implementations of the old functions
 *       taking double arguments still exist for compatibility purposes
 *       (prototypes for them are earlier in this header).
 */

#if __GNUC_PREREQ (4, 4)
  #define fpclassify(__x) (__builtin_fpclassify (FP_NAN, FP_INFINITE, \
                                                 FP_NORMAL, FP_SUBNORMAL, \
                                                 FP_ZERO, __x))
  #ifndef isfinite
    #define isfinite(__x) (__builtin_isfinite (__x))
  #endif
  #ifndef isinf
    #define isinf(__x) (__builtin_isinf_sign (__x))
  #endif
  #ifndef isnan
    #define isnan(__x) (__builtin_isnan (__x))
  #endif
  #define isnormal(__x) (__builtin_isnormal (__x))
#else
  #define fpclassify(__x) \
          ((sizeof(__x) == sizeof(float))  ? __fpclassifyf(__x) : \
          __fpclassifyd(__x))
  #ifndef isfinite
    #define isfinite(__y) \
            (__extension__ ({int __cy = fpclassify(__y); \
                             __cy != FP_INFINITE && __cy != FP_NAN;}))
  #endif
  #ifndef isinf
    #define isinf(__x) (fpclassify(__x) == FP_INFINITE)
  #endif
  #ifndef isnan
    #define isnan(__x) (fpclassify(__x) == FP_NAN)
  #endif
  #define isnormal(__x) (fpclassify(__x) == FP_NORMAL)
#endif

#if __GNUC_PREREQ (4, 0)
  #if defined(_HAVE_LONG_DOUBLE)
    #define signbit(__x) \
            ((sizeof(__x) == sizeof(float))  ? __builtin_signbitf(__x) : \
             (sizeof(__x) == sizeof(double)) ? __builtin_signbit (__x) : \
                                               __builtin_signbitl(__x))
  #else
    #define signbit(__x) \
            ((sizeof(__x) == sizeof(float))  ? __builtin_signbitf(__x) : \
                                               __builtin_signbit (__x))
  #endif
#else
  #define signbit(__x) \
          ((sizeof(__x) == sizeof(float))  ?  __signbitf(__x) : \
                  __signbitd(__x))
#endif

#if __GNUC_PREREQ (2, 97)
#define isgreater(__x,__y) (__builtin_isgreater (__x, __y))
#define isgreaterequal(__x,__y) (__builtin_isgreaterequal (__x, __y))
#define isless(__x,__y) (__builtin_isless (__x, __y))
#define islessequal(__x,__y) (__builtin_islessequal (__x, __y))
#define islessgreater(__x,__y) (__builtin_islessgreater (__x, __y))
#define isunordered(__x,__y) (__builtin_isunordered (__x, __y))
#else
#define isgreater(x,y) \
          (__extension__ ({__typeof__(x) __x = (x); __typeof__(y) __y = (y); \
                           !isunordered(__x,__y) && (__x > __y);}))
#define isgreaterequal(x,y) \
          (__extension__ ({__typeof__(x) __x = (x); __typeof__(y) __y = (y); \
                           !isunordered(__x,__y) && (__x >= __y);}))
#define isless(x,y) \
          (__extension__ ({__typeof__(x) __x = (x); __typeof__(y) __y = (y); \
                           !isunordered(__x,__y) && (__x < __y);}))
#define islessequal(x,y) \
          (__extension__ ({__typeof__(x) __x = (x); __typeof__(y) __y = (y); \
                           !isunordered(__x,__y) && (__x <= __y);}))
#define islessgreater(x,y) \
          (__extension__ ({__typeof__(x) __x = (x); __typeof__(y) __y = (y); \
                           !isunordered(__x,__y) && (__x < __y || __x > __y);}))

#define isunordered(a,b) \
          (__extension__ ({__typeof__(a) __a = (a); __typeof__(b) __b = (b); \
                           fpclassify(__a) == FP_NAN || fpclassify(__b) == FP_NAN;}))
#endif

/* Non ANSI double precision functions.  */

extern double infinity (void);
extern double nan (const char *);
extern double copysign (double, double);
extern double logb (double);
extern int ilogb (double);

extern double asinh (double);
extern double cbrt (double);
extern double nextafter (double, double);
extern double rint (double);
extern double scalbn (double, int);

extern double exp2 (double);
extern double scalbln (double, long int);
extern double tgamma (double);
extern double nearbyint (double);
extern long int lrint (double);
extern long long int llrint (double);
extern double round (double);
extern long int lround (double);
extern long long int llround (double);
extern double trunc (double);
extern double remquo (double, double, int *);
extern double fdim (double, double);
extern double fmax (double, double);
extern double fmin (double, double);
extern double fma (double, double, double);

#ifndef __math_68881
extern double log1p (double);
extern double expm1 (double);
#endif /* ! defined (__math_68881) */

extern double acosh (double);
extern double atanh (double);
extern double remainder (double, double);
extern double gamma (double);
extern double lgamma (double);
extern double erf (double);
extern double erfc (double);
extern double log2 (double);
#if !defined(__cplusplus)
#define log2(x) (log (x) / _M_LN2)
#endif

#ifndef __math_68881
extern double hypot (double, double);
#endif

/* Single precision versions of ANSI functions.  */

extern float atanf (float);
extern float cosf (float);
extern float sinf (float);
extern float tanf (float);
extern float tanhf (float);
extern float frexpf (float, int *);
extern float modff (float, float *);
extern float ceilf (float);
extern float fabsf (float);
extern float floorf (float);

extern float acosf (float);
extern float asinf (float);
extern float atan2f (float, float);
extern float coshf (float);
extern float sinhf (float);
extern float expf (float);
extern float ldexpf (float, int);
extern float logf (float);
extern float log10f (float);
extern float powf (float, float);
extern float sqrtf (float);
extern float fmodf (float, float);

/* Other single precision functions.  */

extern float exp2f (float);
extern float scalblnf (float, long int);
extern float tgammaf (float);
extern float nearbyintf (float);
extern long int lrintf (float);
extern long long int llrintf (float);
extern float roundf (float);
extern long int lroundf (float);
extern long long int llroundf (float);
extern float truncf (float);
extern float remquof (float, float, int *);
extern float fdimf (float, float);
extern float fmaxf (float, float);
extern float fminf (float, float);
extern float fmaf (float, float, float);

extern float infinityf (void);
extern float nanf (const char *);
extern float copysignf (float, float);
extern float logbf (float);
extern int ilogbf (float);

extern float asinhf (float);
extern float cbrtf (float);
extern float nextafterf (float, float);
extern float rintf (float);
extern float scalbnf (float, int);
extern float log1pf (float);
extern float expm1f (float);

extern float acoshf (float);
extern float atanhf (float);
extern float remainderf (float, float);
extern float gammaf (float);
extern float lgammaf (float);
extern float erff (float);
extern float erfcf (float);
extern float log2f (float);
extern float hypotf (float, float);

/* Newlib doesn't fully support long double math functions so far.
 * On platforms where long double equals double the long double functions
 * simply call the double functions.  On Cygwin the long double functions
 * are implemented independently from newlib to be able to use optimized
 * assembler functions despite using the Microsoft x86_64 ABI.
 */

#if defined (_LDBL_EQ_DBL) || defined (__CYGWIN__)
/* Reentrant ANSI C functions.  */
#ifndef __math_68881
extern long double atanl (long double);
extern long double cosl (long double);
extern long double sinl (long double);
extern long double tanl (long double);
extern long double tanhl (long double);
extern long double frexpl (long double, int *);
extern long double modfl (long double, long double *);
extern long double ceill (long double);
extern long double fabsl (long double);
extern long double floorl (long double);
extern long double log1pl (long double);
extern long double expm1l (long double);
#endif /* ! defined (__math_68881) */
/* Non reentrant ANSI C functions.  */
#ifndef __math_68881
extern long double acosl (long double);
extern long double asinl (long double);
extern long double atan2l (long double, long double);
extern long double coshl (long double);
extern long double sinhl (long double);
extern long double expl (long double);
extern long double ldexpl (long double, int);
extern long double logl (long double);
extern long double log10l (long double);
extern long double powl (long double, long double);
extern long double sqrtl (long double);
extern long double fmodl (long double, long double);
extern long double hypotl (long double, long double);
#endif /* ! defined (__math_68881) */
extern long double copysignl (long double, long double);
extern long double nanl (const char *);
extern int ilogbl (long double);
extern long double asinhl (long double);
extern long double cbrtl (long double);
extern long double nextafterl (long double, long double);
extern float nexttowardf (float, long double);
extern double nexttoward (double, long double);
extern long double nexttowardl (long double, long double);
extern long double logbl (long double);
extern long double log2l (long double);
extern long double rintl (long double);
extern long double scalbnl (long double, int);
extern long double exp2l (long double);
extern long double scalblnl (long double, long);
extern long double tgammal (long double);
extern long double nearbyintl (long double);
extern long int lrintl (long double);
extern long long int llrintl (long double);
extern long double roundl (long double);
extern long lroundl (long double);
extern long long int llroundl (long double);
extern long double truncl (long double);
extern long double remquol (long double, long double, int *);
extern long double fdiml (long double, long double);
extern long double fmaxl (long double, long double);
extern long double fminl (long double, long double);
extern long double fmal (long double, long double, long double);
extern long double acoshl (long double);
extern long double atanhl (long double);
extern long double remainderl (long double, long double);
extern long double lgammal (long double);
extern long double erfl (long double);
extern long double erfcl (long double);
#else /* !_LDBL_EQ_DBL && !__CYGWIN__ */
extern long double hypotl (long double, long double);
extern long double sqrtl (long double);
extern long double frexpl (long double, int *);
#ifdef __i386__
/* Other long double precision functions.  */

extern _LONG_DOUBLE rintl (_LONG_DOUBLE);
extern long int lrintl (_LONG_DOUBLE);
extern long long int llrintl (_LONG_DOUBLE);
#endif /* __i386__ */
#endif /* !_LDBL_EQ_DBL && !__CYGWIN__ */

extern double drem (double, double);
extern float dremf (float, float);
#ifdef __CYGWIN__
extern float dreml (long double, long double);
#endif /* __CYGWIN__ */
extern double gamma_r (double, int *);
extern double lgamma_r (double, int *);
extern float gammaf_r (float, int *);
extern float lgammaf_r (float, int *);

extern double y0 (double);
extern double y1 (double);
extern double yn (int, double);
extern double j0 (double);
extern double j1 (double);
extern double jn (int, double);

extern float y0f (float);
extern float y1f (float);
extern float ynf (int, float);
extern float j0f (float);
extern float j1f (float);
extern float jnf (int, float);

/* GNU extensions */

extern void sincos (double, double *, double *);
extern void sincosf (float, float *, float *);
#ifdef __CYGWIN__
extern void sincosl (long double, long double *, long double *);
#endif /* __CYGWIN__ */
# ifndef exp10
extern double exp10 (double);
# endif
# ifndef pow10
extern double pow10 (double);
# endif
# ifndef exp10f
extern float exp10f (float);
# endif
# ifndef pow10f
extern float pow10f (float);
# endif
#ifdef __CYGWIN__
# ifndef exp10l
extern float exp10l (float);
# endif
# ifndef pow10l
extern float pow10l (float);
# endif
#endif /* __CYGWIN__ */

/* The gamma functions use a global variable, signgam.  */
#define signgam (*__signgam())
extern int *__signgam (void);

#define __signgam_r(ptr) _REENT_SIGNGAM(ptr)

/* Useful constants.  */

#define MAXFLOAT        3.40282347e+38F

#define M_E             2.7182818284590452354
#define M_LOG2E         1.4426950408889634074
#define M_LOG10E        0.43429448190325182765
#define M_LN2           _M_LN2
#define M_LN10          2.30258509299404568402
#define M_PI            3.14159265358979323846
#define M_PI_2          1.57079632679489661923
#define M_PI_4          0.78539816339744830962
#define M_1_PI          0.31830988618379067154
#define M_2_PI          0.63661977236758134308
#define M_2_SQRTPI      1.12837916709551257390
#define M_SQRT2         1.41421356237309504880
#define M_SQRT1_2       0.70710678118654752440

#define M_TWOPI         (M_PI * 2.0)
#define M_3PI_4         2.3561944901923448370E0
#define M_SQRTPI        1.77245385090551602792981
#define M_LN2LO         1.9082149292705877000E-10
#define M_LN2HI         6.9314718036912381649E-1
#define M_SQRT3         1.73205080756887719000
#define M_IVLN10        0.43429448190325182765 /* 1 / log(10) */
#define M_LOG2_E        _M_LN2
#define M_INVLN2        1.4426950408889633870E0  /* 1 / log(2) */

#ifdef __cplusplus
}
#endif

#endif /* _MATH_H_ */
