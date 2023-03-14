/****************************************************************************
 * libs/libm/newlib/include/ieeefp.h
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

#ifndef __LIBS_LIBM_NEWLIB_INCLUDE_IEEEFP_H
#define __LIBS_LIBM_NEWLIB_INCLUDE_IEEEFP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "_ansi.h"

#include <machine/ieeefp.h>
#include <float.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN  extern "C"
extern "C"
{
#else
#define EXTERN  extern
#endif

#ifdef __IEEE_BIG_ENDIAN

typedef union
{
  double value;
  struct _number
  {
    unsigned int sign : 1;
    unsigned int exponent : 11;
    unsigned int fraction0 : 4;
    unsigned int fraction1 : 16;
    unsigned int fraction2 : 16;
    unsigned int fraction3 : 16;
  };
  struct _number number;
  struct _nan
  {
    unsigned int sign : 1;
    unsigned int exponent : 11;
    unsigned int quiet : 1;
    unsigned int function0 : 3;
    unsigned int function1 : 16;
    unsigned int function2 : 16;
    unsigned int function3 : 16;
  };
  struct _nan nan;
  struct
  {
    unsigned long msw;
    unsigned long lsw;
  } parts;
  long aslong[2];
} __ieee_double_shape_type;

#elif defined __IEEE_LITTLE_ENDIAN

typedef union
{
  double value;
  struct
  {
#ifdef __SMALL_BITFIELDS
    unsigned int fraction3 : 16;
    unsigned int fraction2 : 16;
    unsigned int fraction1 : 16;
    unsigned int fraction0 : 4;
#else
    unsigned int fraction1 : 32;
    unsigned int fraction0 : 20;
#endif
    unsigned int exponent : 11;
    unsigned int sign : 1;
  } number;
  struct
  {
#ifdef __SMALL_BITFIELDS
    unsigned int function3 : 16;
    unsigned int function2 : 16;
    unsigned int function1 : 16;
    unsigned int function0 : 3;
#else
    unsigned int function1 : 32;
    unsigned int function0 : 19;
#endif
    unsigned int quiet : 1;
    unsigned int exponent : 11;
    unsigned int sign : 1;
  } nan;
  struct
  {
    unsigned long lsw;
    unsigned long msw;
  } parts;

  long aslong[2];
} __ieee_double_shape_type;

#endif /* __IEEE_LITTLE_ENDIAN */

#ifdef __IEEE_BIG_ENDIAN

typedef union
{
  float value;
  struct
  {
    unsigned int sign : 1;
    unsigned int exponent : 8;
    unsigned int fraction0 : 7;
    unsigned int fraction1 : 16;
  } number;
  struct
  {
    unsigned int sign : 1;
    unsigned int exponent : 8;
    unsigned int quiet : 1;
    unsigned int function0 : 6;
    unsigned int function1 : 16;
  } nan;
  long p1;
} __ieee_float_shape_type;

#elif defined __IEEE_LITTLE_ENDIAN

typedef union
{
  float value;
  struct
  {
    unsigned int fraction0 : 7;
    unsigned int fraction1 : 16;
    unsigned int exponent : 8;
    unsigned int sign : 1;
  } number;
  struct
  {
    unsigned int function1 : 16;
    unsigned int function0 : 6;
    unsigned int quiet : 1;
    unsigned int exponent : 8;
    unsigned int sign : 1;
  } nan;
  long p1;
} __ieee_float_shape_type;

#endif /* __IEEE_LITTLE_ENDIAN */

#ifndef _LDBL_EQ_DBL

#ifndef LDBL_MANT_DIG
#error "LDBL_MANT_DIG not defined - should be found in float.h"

#elif LDBL_MANT_DIG == DBL_MANT_DIG
#error \
  "double and long double are the same size but LDBL_EQ_DBL is not defined"

#elif LDBL_MANT_DIG == 53

/* This happens when doubles are 32-bits and long doubles are 64-bits.  */
#define EXT_EXPBITS             11
#define EXT_FRACHBITS           20
#define EXT_FRACLBITS           32
#define __ieee_ext_field_type   unsigned long

#elif LDBL_MANT_DIG == 64
#define EXT_EXPBITS             15
#define EXT_FRACHBITS           32
#define EXT_FRACLBITS           32
#define __ieee_ext_field_type   unsigned int

#elif LDBL_MANT_DIG == 65
#define EXT_EXPBITS             15
#define EXT_FRACHBITS           32
#define EXT_FRACLBITS           32
#define __ieee_ext_field_type   unsigned int

#elif LDBL_MANT_DIG == 112
#define EXT_EXPBITS             15
#define EXT_FRACHBITS           48
#define EXT_FRACLBITS           64
#define __ieee_ext_field_type   unsigned long long

#elif LDBL_MANT_DIG == 113
#define EXT_EXPBITS             15
#define EXT_FRACHBITS           48
#define EXT_FRACLBITS           64
#define __ieee_ext_field_type   unsigned long long

#else
#error Unsupported value for LDBL_MANT_DIG
#endif

#define EXT_EXP_INFNAN          ((1 << EXT_EXPBITS) - 1)        /* 32767 */
#define EXT_EXP_BIAS            ((1 << (EXT_EXPBITS - 1)) - 1)  /* 16383 */
#define EXT_FRACBITS            (EXT_FRACLBITS + EXT_FRACHBITS)

typedef struct ieee_ext
{
  __ieee_ext_field_type ext_fracl : EXT_FRACLBITS;
  __ieee_ext_field_type ext_frach : EXT_FRACHBITS;
  __ieee_ext_field_type ext_exp : EXT_EXPBITS;
  __ieee_ext_field_type ext_sign : 1;
} ieee_ext;

typedef union ieee_ext_u
{
  long double extu_ld;
  struct ieee_ext extu_ext;
} ieee_ext_u;

#endif /* ! _LDBL_EQ_DBL */

/* FLOATING ROUNDING */

typedef int fp_rnd;
#define FP_RN   0   /* Round to nearest         */
#define FP_RM   1   /* Round down           */
#define FP_RP   2   /* Round up             */
#define FP_RZ   3   /* Round to zero (trunate)  */

fp_rnd fpgetround(void);

fp_rnd fpsetround(fp_rnd);

/* EXCEPTIONS */

typedef int fp_except;
#define FP_X_INV    0x10    /* Invalid operation        */
#define FP_X_DX     0x80    /* Divide by zero		*/
#define FP_X_OFL    0x04    /* Overflow exception		*/
#define FP_X_UFL    0x02    /* Underflow exception		*/
#define FP_X_IMP    0x01    /* imprecise exception		*/

fp_except fpgetmask(void);

fp_except fpsetmask(fp_except);
fp_except fpgetsticky(void);

fp_except fpsetsticky(fp_except);

/* INTEGER ROUNDING */

typedef int fp_rdi;
#define FP_RDI_TOZ  0   /* Round to Zero        */
#define FP_RDI_RD   1   /* Follow float mode		*/

fp_rdi fpgetroundtoi(void);

fp_rdi fpsetroundtoi(fp_rdi);

#ifdef _DOUBLE_IS_32BITS

#undef __ieee_double_shape_type
#define __ieee_double_shape_type  __ieee_float_shape_type

#endif /* _DOUBLE_IS_32BITS */

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __LIBS_LIBM_NEWLIB_INCLUDE_IEEEFP_H */
