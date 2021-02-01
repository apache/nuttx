/****************************************************************************
 * include/nuttx/lib/float.h
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

#ifndef __INCLUDE_NUTTX_LIB_FLOAT_H
#define __INCLUDE_NUTTX_LIB_FLOAT_H

/* TODO:  These values could vary with architectures toolchains.  This
 * logic should be move at least to the include/arch directory.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Radix of exponent representation, b. */

#define FLT_RADIX 2

/* Number of base-FLT_RADIX digits in the floating-point significand, p. */

#ifndef FLT_MANT_DIG  /* May be defined in a toolchain header */
#  define FLT_MANT_DIG 24
#endif

#ifndef DBL_MANT_DIG  /* May be defined in a toolchain header */
#  ifdef CONFIG_HAVE_DOUBLE
#    define DBL_MANT_DIG 53
#  else
#    define DBL_MANT_DIG FLT_MANT_DIG
#  endif
#endif

#ifndef LDBL_MANT_DIG  /* May be defined in a toolchain header */
#  ifdef CONFIG_HAVE_LONG_DOUBLE
#    define LDBL_MANT_DIG DBL_MANT_DIG /* FIX ME */
#  else
#    define LDBL_MANT_DIG DBL_MANT_DIG
#  endif
#endif

/* Number of decimal digits, n, such that any floating-point number in the
 * widest supported floating type with pmax radix b digits can be rounded
 * to a floating-point number with n decimal digits and back again without
 * change to the value.
 */

#define DECIMAL_DIG 10

/* Number of decimal digits, q, such that any floating-point number with q
 * decimal digits can be rounded into a floating-point number with p radix
 * b digits and back again without change to the q decimal digits.
 */

#ifndef FLT_DIG  /* May be defined in a toolchain header */
#  define FLT_DIG 6
#endif

#ifndef DBL_DIG  /* May be defined in a toolchain header */
#  ifdef CONFIG_HAVE_DOUBLE
#    define DBL_DIG 15  /* 10 */
#  else
#    define DBL_DIG FLT_DIG
#  endif
#endif

#ifndef LDBL_DIG  /* May be defined in a toolchain header */
# ifdef CONFIG_HAVE_LONG_DOUBLE
#    define LDBL_DIG DBL_DIG  /* FIX ME */
#  else
#    define LDBL_DIG DBL_DIG
#  endif
#endif

/* Minimum negative integer such that FLT_RADIX raised to that power minus
 * 1 is a normalized floating-point number, emin.
 */

#ifndef FLT_MIN_EXP  /* May be defined in a toolchain header */
#  define FLT_MIN_EXP (-125)
#endif

#ifndef DBL_MIN_EXP  /* May be defined in a toolchain header */
#  ifdef CONFIG_HAVE_DOUBLE
#    define DBL_MIN_EXP (-1021)
#  else
#    define DBL_MIN_EXP FLT_MIN_EXP
#  endif
#endif

#ifndef LDBL_MIN_EXP  /* May be defined in a toolchain header */
#  ifdef CONFIG_HAVE_LONG_DOUBLE
#    define LDBL_MIN_EXP DBL_MIN_EXP /* FIX ME */
#  else
#    define LDBL_MIN_EXP DBL_MIN_EXP
#  endif
#endif

/* Minimum negative integer such that 10 raised to that power is in the range
 * of normalized floating-point numbers.
 */

#ifndef FLT_MIN_10_EXP  /* May be defined in a toolchain header */
#  define FLT_MIN_10_EXP (-37)
#endif

#ifndef DBL_MIN_10_EXP  /* May be defined in a toolchain header */
#  ifdef CONFIG_HAVE_DOUBLE
#    define DBL_MIN_10_EXP (-307)  /* -37 */
#  else
#    define DBL_MIN_10_EXP FLT_MIN_10_EXP
#  endif
#endif

#ifndef LDBL_MIN_10_EXP  /* May be defined in a toolchain header */
#  ifdef CONFIG_HAVE_LONG_DOUBLE
#    define LDBL_MIN_10_EXP DBL_MIN_10_EXP  /* FIX ME */
#  else
#    define LDBL_MIN_10_EXP DBL_MIN_10_EXP
#  endif
#endif

/* Maximum integer such that FLT_RADIX raised to that power minus 1 is a
 * representable finite floating-point number, emax.
 */

#ifndef FLT_MAX_EXP  /* May be defined in a toolchain header */
#  define FLT_MAX_EXP 128
#endif

#ifndef DBL_MAX_EXP  /* May be defined in a toolchain header */
#  ifdef CONFIG_HAVE_DOUBLE
#    define DBL_MAX_EXP 1024
#  else
#    define DBL_MAX_EXP FLT_MAX_EXP
#  endif
#endif

#ifndef LDBL_MAX_EXP  /* May be defined in a toolchain header */
#  ifdef CONFIG_HAVE_LONG_DOUBLE
#    define LDBL_MAX_EXP DBL_MAX_EXP /* FIX ME */
#  else
#    define LDBL_MAX_EXP DBL_MAX_EXP
#  endif
#endif

/* Maximum integer such that 10 raised to that power is in the range of
 * representable finite floating-point numbers.
 */

#ifndef FLT_MAX_10_EXP       /* May be defined in toolchain header */
#  define FLT_MAX_10_EXP 38  /* 37 */
#endif

#ifndef DBL_MAX_10_EXP          /* May be defined in toolchain header */
#  ifdef CONFIG_HAVE_DOUBLE
#    define DBL_MAX_10_EXP 308  /* 37 */
#  else
#    define DBL_MAX_10_EXP FLT_MAX_10_EXP
#  endif
#endif

#ifndef LDBL_MAX_10_EXP      /* May be defined in toolchain header */
#  ifdef CONFIG_HAVE_LONG_DOUBLE
#    define LDBL_MAX_10_EXP DBL_MAX_10_EXP  /* FIX ME */
#  else
#    define LDBL_MAX_10_EXP DBL_MAX_10_EXP
#  endif
#endif

/* Maximum representable finite floating-point number. */

#ifndef FLT_MAX                    /* May be defined in toolchain header */
#  define FLT_MAX 3.40282347e+38F  /* 1E+37 */
#endif

#ifndef DBL_MAX                    /* May be defined in toolchain header */
#  ifdef CONFIG_HAVE_DOUBLE
#    define DBL_MAX 1.7976931348623157e+308  /* 1E+37 */
#  else
#    define DBL_MAX FLT_MAX
#  endif
#endif

#ifndef LDBL_MAX                   /* May be defined in toolchain header */
#  ifdef CONFIG_HAVE_LONG_DOUBLE
#    define LDBL_MAX DBL_MAX       /* FIX ME */
#  else
#    define LDBL_MAX DBL_MAX
#  endif
#endif

/* The difference between 1 and the least value greater than 1 that is
 * representable in the given floating-point type, b1-p.
 */

#ifndef FLT_EPSILON                  /* May be defined in toolchain header */
#  define FLT_EPSILON 1.1920929e-07F /* 1E-5 */
#endif

#ifndef DBL_EPSILON                  /* May be defined in toolchain header */
#  ifdef CONFIG_HAVE_DOUBLE
#    define DBL_EPSILON 2.2204460492503131e-16  /* 1E-9 */
#  else
#    define DBL_EPSILON FLT_EPSILON
#  endif
#endif

#ifndef LDBL_EPSILON                 /* May be defined in toolchain header */
#  ifdef CONFIG_HAVE_LONG_DOUBLE
#    define LDBL_EPSILON DBL_EPSILON /* FIX ME */
#  else
#    define LDBL_EPSILON DBL_EPSILON
#  endif
#endif

/* Minimum normalized positive floating-point number, bemin -1. */

#ifndef FLT_MIN                    /* May be defined in toolchain header */
#  define FLT_MIN 1.17549435e-38F  /* 1E-37 */
#endif

#ifndef DBL_MIN                    /* May be defined in toolchain header */
#  ifdef CONFIG_HAVE_DOUBLE
#    define DBL_MIN 2.2250738585072014e-308  /* 1E-37 */
#  else
#    define DBL_MIN FLT_MIN
#  endif
#endif

#ifndef LDBL_MIN                    /* May be defined in toolchain header */
#  ifdef CONFIG_HAVE_LONG_DOUBLE
#    define LDBL_MIN DBL_MIN        /* FIX ME */
#  else
#    define LDBL_MIN DBL_MIN
#  endif
#endif

#endif /* __INCLUDE_NUTTX_LIB_FLOAT_H */
