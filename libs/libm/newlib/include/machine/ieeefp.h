/****************************************************************************
 * libs/libm/newlib/include/machine/ieeefp.h
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

#ifndef __IEEE_BIG_ENDIAN
#ifndef __IEEE_LITTLE_ENDIAN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if (defined(__arm__) || defined(__thumb__)) && !defined(__MAVERICK__)

/* ARM traditionally used big-endian words; and within those words the
 *  byte ordering was big or little endian depending upon the target.
 *  Modern floating-point formats are naturally ordered; in this case
 *  __VFP_FP__ will be defined, even if soft-float.
 */

#ifdef __VFP_FP__
# ifdef __ARMEL__
#  define __IEEE_LITTLE_ENDIAN
# else
#  define __IEEE_BIG_ENDIAN
# endif
# if __ARM_FP & 0x8
#  define __OBSOLETE_MATH_DEFAULT  0
# endif
#else
# define __IEEE_BIG_ENDIAN
# ifdef __ARMEL__
#  define __IEEE_BYTES_LITTLE_ENDIAN
# endif
#endif
#ifndef __SOFTFP__
# define _SUPPORTS_ERREXCEPT
#endif

/* As per ISO/IEC TS 18661 '__FLT_EVAL_METHOD__' will be defined to 16
 *  (if compiling with +fp16 support) so it can't be used by math.h to
 *  define float_t and double_t.  For values of '__FLT_EVAL_METHOD__'
 *  other than 0, 1, 2 the definition of float_t and double_t is
 *  implementation-defined.
 */

#define __DOUBLE_TYPE   double
#define __FLOAT_TYPE    float
#endif

#if defined (__aarch64__)
#if defined (__AARCH64EL__)
#define __IEEE_LITTLE_ENDIAN
#else
#define __IEEE_BIG_ENDIAN
#endif
#define __OBSOLETE_MATH_DEFAULT     0
#ifdef __ARM_FP
# define _SUPPORTS_ERREXCEPT
#endif

/* As per ISO/IEC TS 18661 '__FLT_EVAL_METHOD__' will be defined to 16
 *  (if compiling with +fp16 support) so it can't be used by math.h to
 *  define float_t and double_t.  For values of '__FLT_EVAL_METHOD__'
 *  other than 0, 1, 2 the definition of float_t and double_t is
 *  implementation-defined.
 */

#define __DOUBLE_TYPE               double
#define __FLOAT_TYPE                float
#endif

#ifdef __epiphany__
#define __IEEE_LITTLE_ENDIAN
#define Sudden_Underflow            1
#endif

#ifdef __hppa__
#define __IEEE_BIG_ENDIAN
#endif

#ifdef __nds32__
#ifdef __big_endian__
#define __IEEE_BIG_ENDIAN
#else
#define __IEEE_LITTLE_ENDIAN
#endif
#endif

#ifdef __SPU__
#define __IEEE_BIG_ENDIAN

#define isfinite(__y)                                                         \
  (__extension__({ int __cy;                                                  \
                   (sizeof(__y) == sizeof(float))  ? (1) :                    \
                   (__cy = fpclassify(__y)) != FP_INFINITE && __cy != FP_NAN; \
                 }))

#define isinf(__x)              ((sizeof(__x) == \
                                  sizeof(float))  ?  (0) : __isinfd(__x))
#define isnan(__x)              ((sizeof(__x) == \
                                  sizeof(float))  ?  (0) : __isnand(__x))

/* Macros for use in ieeefp.h. We can't just define the real ones here
 * (like those above) as we have name space issues when this is *not*
 * included via generic the ieeefp.h.
 */

#define __ieeefp_isnanf(x)      0
#define __ieeefp_isinff(x)      0
#define __ieeefp_finitef(x)     1
#endif

#ifdef __sparc__
#ifdef __LITTLE_ENDIAN_DATA__
#define __IEEE_LITTLE_ENDIAN
#else
#define __IEEE_BIG_ENDIAN
#endif
#endif

#if defined(__m68k__) || defined(__mc68000__)
#define __IEEE_BIG_ENDIAN
#endif

#if defined(__mc68hc11__) || defined(__mc68hc12__) || defined(__mc68hc1x__)
#define __IEEE_BIG_ENDIAN
#ifdef __HAVE_SHORT_DOUBLE__
# define _DOUBLE_IS_32BITS
#endif
#endif

#if defined (__H8300__) || defined (__H8300H__) || defined (__H8300S__) || \
  defined (__H8500__) || defined (__H8300SX__)
#define __IEEE_BIG_ENDIAN
#define _FLOAT_ARG  float
#define _DOUBLE_IS_32BITS
#endif

#if defined (__xc16x__) || defined (__xc16xL__) || defined (__xc16xS__)
#define __IEEE_LITTLE_ENDIAN
#define _FLOAT_ARG  float
#define _DOUBLE_IS_32BITS
#endif

#ifdef __sh__
#ifdef __LITTLE_ENDIAN__
#define __IEEE_LITTLE_ENDIAN
#else
#define __IEEE_BIG_ENDIAN
#endif
#if defined(__SH2E__) || defined(__SH3E__) || defined(__SH4_SINGLE_ONLY__) || \
  defined(__SH2A_SINGLE_ONLY__)
#define _DOUBLE_IS_32BITS
#endif
#endif

#ifdef _AM29K
#define __IEEE_BIG_ENDIAN
#endif

#ifdef _WIN32
#define __IEEE_LITTLE_ENDIAN
#endif

#ifdef __i386__
#define __IEEE_LITTLE_ENDIAN
# define _SUPPORTS_ERREXCEPT
#endif

#ifdef __riscv
#if defined(__BYTE_ORDER__) && (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
#define __IEEE_BIG_ENDIAN
#else
#define __IEEE_LITTLE_ENDIAN
#endif
#ifdef __riscv_flen
# define _SUPPORTS_ERREXCEPT
#endif
#endif

#ifdef __i960__
#define __IEEE_LITTLE_ENDIAN
#endif

#ifdef __lm32__
#define __IEEE_BIG_ENDIAN
#endif

#ifdef __M32R__
#define __IEEE_BIG_ENDIAN
#endif

#ifdef __nvptx__
#define __IEEE_LITTLE_ENDIAN
#endif

#if defined(_C4x) || defined(_C3x)
#define __IEEE_BIG_ENDIAN
#define _DOUBLE_IS_32BITS
#endif

#ifdef __TMS320C6X__
#ifdef _BIG_ENDIAN
#define __IEEE_BIG_ENDIAN
#else
#define __IEEE_LITTLE_ENDIAN
#endif
#endif

#ifdef __TIC80__
#define __IEEE_LITTLE_ENDIAN
#endif

#ifdef __MIPSEL__
#define __IEEE_LITTLE_ENDIAN
#endif
#ifdef __MIPSEB__
#define __IEEE_BIG_ENDIAN
#endif

#ifdef __MMIX__
#define __IEEE_BIG_ENDIAN
#endif

#ifdef __D30V__
#define __IEEE_BIG_ENDIAN
#endif

/* necv70 was __IEEE_LITTLE_ENDIAN. */

#ifdef __W65__
#define __IEEE_LITTLE_ENDIAN
#define _DOUBLE_IS_32BITS
#endif

#if defined(__Z8001__) || defined(__Z8002__)
#define __IEEE_BIG_ENDIAN
#endif

#ifdef __m88k__
#define __IEEE_BIG_ENDIAN
#endif

#ifdef __mn10300__
#define __IEEE_LITTLE_ENDIAN
#endif

#ifdef __mn10200__
#define __IEEE_LITTLE_ENDIAN
#define _DOUBLE_IS_32BITS
#endif

#ifdef __v800
#define __IEEE_LITTLE_ENDIAN
#endif

#ifdef __v850
#define __IEEE_LITTLE_ENDIAN
#endif

#ifdef __D10V__
#define __IEEE_BIG_ENDIAN
#if __DOUBLE__ == 32
#define _DOUBLE_IS_32BITS
#endif
#endif

#ifdef __PPC__
#if (defined(_BIG_ENDIAN) && _BIG_ENDIAN) || (defined(_AIX) && _AIX)
#define __IEEE_BIG_ENDIAN
#else
#if (defined(_LITTLE_ENDIAN) && _LITTLE_ENDIAN) || (defined(__sun__) && \
  __sun__) || (defined(_WIN32) && _WIN32)
#define __IEEE_LITTLE_ENDIAN
#endif
#endif
#endif

#ifdef __xstormy16__
#define __IEEE_LITTLE_ENDIAN
#endif

#ifdef __arc__
#ifdef __big_endian__
#define __IEEE_BIG_ENDIAN
#else
#define __IEEE_LITTLE_ENDIAN
#endif
#endif

#ifdef __CRX__
#define __IEEE_LITTLE_ENDIAN
#endif

#ifdef __CSKY__
#ifdef __CSKYBE__
#define __IEEE_BIG_ENDIAN
#else
#define __IEEE_LITTLE_ENDIAN
#endif
#endif

#ifdef __fr30__
#define __IEEE_BIG_ENDIAN
#endif

#ifdef __FT32__
#define __IEEE_LITTLE_ENDIAN
#endif

#ifdef __mcore__
#define __IEEE_BIG_ENDIAN
#endif

#ifdef __mt__
#define __IEEE_BIG_ENDIAN
#endif

#ifdef __frv__
#define __IEEE_BIG_ENDIAN
#endif

#ifdef __moxie__
#ifdef __MOXIE_BIG_ENDIAN__
#define __IEEE_BIG_ENDIAN
#else
#define __IEEE_LITTLE_ENDIAN
#endif
#endif

#ifdef __ia64__
#ifdef __BIG_ENDIAN__
#define __IEEE_BIG_ENDIAN
#else
#define __IEEE_LITTLE_ENDIAN
#endif
#endif

#ifdef __AVR__
#define __IEEE_LITTLE_ENDIAN
#define _DOUBLE_IS_32BITS
#endif

#if defined(__or1k__) || defined(__OR1K__) || defined(__OR1KND__)
#define __IEEE_BIG_ENDIAN
#endif

#ifdef __IP2K__
#define __IEEE_BIG_ENDIAN
#define __SMALL_BITFIELDS
#define _DOUBLE_IS_32BITS
#endif

#ifdef __iq2000__
#define __IEEE_BIG_ENDIAN
#endif

#ifdef __MAVERICK__
#ifdef __ARMEL__
#  define __IEEE_LITTLE_ENDIAN
#else  /* must be __ARMEB__ */
#  define __IEEE_BIG_ENDIAN
#endif /* __ARMEL__ */
#endif /* __MAVERICK__ */

#ifdef __m32c__
#define __IEEE_LITTLE_ENDIAN
#define __SMALL_BITFIELDS
#endif

#ifdef __CRIS__
#define __IEEE_LITTLE_ENDIAN
#endif

#ifdef __BFIN__
#define __IEEE_LITTLE_ENDIAN
#endif

#ifdef __x86_64__
#define __IEEE_LITTLE_ENDIAN
# define _SUPPORTS_ERREXCEPT
#endif

#ifdef __mep__
#ifdef __LITTLE_ENDIAN__
#define __IEEE_LITTLE_ENDIAN
#else
#define __IEEE_BIG_ENDIAN
#endif
#endif

#ifdef __MICROBLAZE__
#ifndef __MICROBLAZEEL__
#define __IEEE_BIG_ENDIAN
#else
#define __IEEE_LITTLE_ENDIAN
#endif
#endif

#ifdef __MSP430__
#define __IEEE_LITTLE_ENDIAN
#define __SMALL_BITFIELDS  /* 16 Bit INT */
#endif

#ifdef __PRU__
#define __IEEE_LITTLE_ENDIAN
#endif

#ifdef __RL78__
#define __IEEE_LITTLE_ENDIAN
#define __SMALL_BITFIELDS  /* 16 Bit INT */
#ifndef __RL78_64BIT_DOUBLES__
#define _DOUBLE_IS_32BITS
#endif
#endif

#ifdef __RX__

#ifdef __RX_BIG_ENDIAN__
#define __IEEE_BIG_ENDIAN
#else
#define __IEEE_LITTLE_ENDIAN
#endif

#ifndef __RX_64BIT_DOUBLES__
#define _DOUBLE_IS_32BITS
#endif

#ifdef __RX_16BIT_INTS__
#define __SMALL_BITFIELDS
#endif

#endif

#if (defined(__CR16__) || defined(__CR16C__) || defined(__CR16CP__))
#define __IEEE_LITTLE_ENDIAN
#define __SMALL_BITFIELDS  /* 16 Bit INT */
#endif

#ifdef __NIOS2__
# ifdef __nios2_big_endian__
#  define __IEEE_BIG_ENDIAN
# else
#  define __IEEE_LITTLE_ENDIAN
# endif
#endif

#ifdef __VISIUM__
#define __IEEE_BIG_ENDIAN
#endif

#ifdef __AMDGCN__
#define __IEEE_LITTLE_ENDIAN
#endif

#ifdef __XTENSA_EL__
#define __IEEE_LITTLE_ENDIAN
#endif

#ifdef __CYGWIN__
#define __OBSOLETE_MATH_DEFAULT     0
#endif

#ifndef __OBSOLETE_MATH_DEFAULT

/* Use old math code by default.  */
#define __OBSOLETE_MATH_DEFAULT     1
#endif
#ifndef __OBSOLETE_MATH
#define __OBSOLETE_MATH             __OBSOLETE_MATH_DEFAULT
#endif

#ifndef __IEEE_BIG_ENDIAN
#ifndef __IEEE_LITTLE_ENDIAN
#error Endianess not declared!!
#endif  /* not __IEEE_LITTLE_ENDIAN */
#endif  /* not __IEEE_BIG_ENDIAN */

#endif  /* not __IEEE_LITTLE_ENDIAN */
#endif  /* not __IEEE_BIG_ENDIAN */
