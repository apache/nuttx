/****************************************************************************
 * include/nuttx/compiler.h
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

#ifndef __INCLUDE_NUTTX_COMPILER_H
#define __INCLUDE_NUTTX_COMPILER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GCC-specific definitions *************************************************/

#ifdef __GNUC__

/* Pre-processor */

#  define CONFIG_CPP_HAVE_VARARGS 1 /* Supports variable argument macros */
#  define CONFIG_CPP_HAVE_WARNING 1 /* Supports #warning */

/* Intriniscs.  GCC supports __func__ but provides __FUNCTION__ for backward
 * compatibility with older versions of GCC.
 */

#  define CONFIG_HAVE_FUNCTIONNAME 1 /* Has __FUNCTION__ */
#  define CONFIG_HAVE_FILENAME     1 /* Has __FILE__ */

/* Indicate that a local variable is not used */

#  define UNUSED(a) ((void)(a))

/* Built-in functions */

/* GCC 4.x have __builtin_ctz(|l|ll) and __builtin_clz(|l|ll). These count
 * trailing/leading zeros of input number and typically will generate few
 * fast bit-counting instructions. Inputting zero to these functions is
 * undefined and needs to be taken care of by the caller.
 */

#if __GNUC__ >= 4
#  define CONFIG_HAVE_BUILTIN_CTZ      1
#  define CONFIG_HAVE_BUILTIN_CLZ      1
#  define CONFIG_HAVE_BUILTIN_POPCOUNT 1
#endif

/* C++ support */

#if defined(__cplusplus) && __cplusplus >= 201402L
#  define CONFIG_HAVE_CXX14 1
#else
#  undef CONFIG_HAVE_CXX14
#endif

/* Attributes
 *
 * GCC supports weak symbols which can be used to reduce code size because
 * unnecessary "weak" functions can be excluded from the link.
 */

# if !defined(__CYGWIN__) && !defined(CONFIG_ARCH_GNU_NO_WEAKFUNCTIONS)
#  define CONFIG_HAVE_WEAKFUNCTIONS 1
#  define weak_alias(name, aliasname) \
   extern __typeof (name) aliasname __attribute__ ((weak, alias (#name)));
#  define weak_data __attribute__ ((weak))
#  define weak_function __attribute__ ((weak))
#  define weak_const_function __attribute__ ((weak, __const__))
# else
#  undef  CONFIG_HAVE_WEAKFUNCTIONS
#  define weak_alias(name, aliasname)
#  define weak_data
#  define weak_function
#  define weak_const_function
# endif

/* The noreturn attribute informs GCC that the function will not return.
 * C11 adds _Noreturn keyword (see stdnoreturn.h)
 */

#  define noreturn_function __attribute__ ((noreturn))

/* The farcall_function attribute informs GCC that is should use long calls
 * (even though -mlong-calls does not appear in the compilation options)
 */

#  define farcall_function __attribute__ ((long_call))

/* Code locate */

#  define locate_code(n) __attribute__ ((section(n)))

/* Data alignment */

#  define aligned_data(n) __attribute__ ((aligned(n)))

/* Data location */

#  define locate_data(n) __attribute__ ((section(n)))

/* The packed attribute informs GCC that the structure elements are packed,
 * ignoring other alignment rules.
 */

#  define begin_packed_struct
#  define end_packed_struct __attribute__ ((packed))

/* GCC does not support the reentrant attribute */

#  define reentrant_function

/* The naked attribute informs GCC that the programmer will take care of
 * the function prolog and epilog.
 */

#  define naked_function __attribute__ ((naked,no_instrument_function))

/* The inline_function attribute informs GCC that the function should always
 * be inlined, regardless of the level of optimization.  The
 * noinline_function indicates that the function should never be inlined.
 */

#  define inline_function __attribute__ ((always_inline,no_instrument_function))
#  define noinline_function __attribute__ ((noinline))

/* GCC does not use storage classes to qualify addressing */

#  define FAR
#  define NEAR
#  define DSEG
#  define CODE

/* Handle cases where sizeof(int) is 16-bits, sizeof(long) is 32-bits, and
 * pointers are 16-bits.
 */

#if defined(__m32c__)
/* No I-space access qualifiers */

#  define IOBJ
#  define IPTR

/* Select the small, 16-bit addressing model */

#  define CONFIG_SMALL_MEMORY 1

/* Long and int are not the same size */

#  define CONFIG_LONG_IS_NOT_INT 1

/* Pointers and int are the same size */

#  undef  CONFIG_PTR_IS_NOT_INT

#elif defined(__AVR__)
# if defined(CONFIG_AVR_HAS_MEMX_PTR)
  /* I-space access qualifiers needed by Harvard architecture */

#  define IOBJ __flash
#  define IPTR __memx

# else
/* No I-space access qualifiers */

#  define IOBJ
#  define IPTR
# endif

/* Select the small, 16-bit addressing model (for D-Space) */

#  define CONFIG_SMALL_MEMORY 1

/* Long and int are not the same size */

#  define CONFIG_LONG_IS_NOT_INT 1

/* Pointers and int are the same size */

#  undef  CONFIG_PTR_IS_NOT_INT

/* Uses a 32-bit FAR pointer only from accessing data outside of the 16-bit
 * data space.
 */

#  define CONFIG_HAVE_FARPOINTER 1

#elif defined(__mc68hc1x__)

/* No I-space access qualifiers */

#  define IOBJ
#  define IPTR

/* Select the small, 16-bit addressing model */

#  define CONFIG_SMALL_MEMORY 1

/* Normally, mc68hc1x code is compiled with the -mshort option
 * which results in a 16-bit integer.  If -mnoshort is defined
 * then an integer is 32-bits.  GCC will defined __INT__ accordingly:
 */

# if __INT__ == 16
/* int is 16-bits, long is 32-bits */

#   define CONFIG_LONG_IS_NOT_INT 1

/* Pointers and int are the same size (16-bits) */

#   undef  CONFIG_PTR_IS_NOT_INT
# else
/* int and long are both 32-bits */

#   undef  CONFIG_LONG_IS_NOT_INT

/* Pointers and int are NOT the same size */

#   define CONFIG_PTR_IS_NOT_INT 1
# endif

#else

/* No I-space access qualifiers */

#  define IOBJ
#  define IPTR

/* Select the large, 32-bit addressing model */

#  undef  CONFIG_SMALL_MEMORY

/* Long and int are (probably) the same size (32-bits) */

#  undef  CONFIG_LONG_IS_NOT_INT

/* Pointers and int are the same size (32-bits) */

#  undef  CONFIG_PTR_IS_NOT_INT
#endif

/* ISO C11 supports anonymous (unnamed) structures and unions, added in
 * GCC 4.6 (but might be suppressed with -std= option).  ISO C++11 also
 * adds un-named unions, but NOT unnamed structures (although compilers
 * may support them).
 *
 * CAREFUL: This can cause issues for shared data structures shared between
 * C and C++ if the two versions do not support the same features.
 * Structures and unions can lose binary compatibility!
 *
 * NOTE: The NuttX coding standard forbids the use of unnamed structures and
 * unions within the OS.
 */

#  undef CONFIG_HAVE_ANONYMOUS_STRUCT
#  undef CONFIG_HAVE_ANONYMOUS_UNION

#  if (defined(__cplusplus) && __cplusplus >= 201103L) || \
      (defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L)
#    define CONFIG_HAVE_ANONYMOUS_STRUCT 1
#    define CONFIG_HAVE_ANONYMOUS_UNION 1
#  endif

#  define CONFIG_HAVE_LONG_LONG 1
#  define CONFIG_HAVE_FLOAT 1
#  define CONFIG_HAVE_DOUBLE 1
#  define CONFIG_HAVE_LONG_DOUBLE 1

/* Indicate that a local variable is not used */

#  define UNUSED(a) ((void)(a))

/* SDCC-specific definitions ************************************************/

#elif defined(SDCC) || defined(__SDCC)

/* No I-space access qualifiers */

#  define IOBJ
#  define IPTR

/* Pre-processor */

#  define CONFIG_CPP_HAVE_VARARGS 1 /* Supports variable argument macros */
#  define CONFIG_CPP_HAVE_WARNING 1 /* Supports #warning */

/* Intriniscs */

#  define CONFIG_HAVE_FUNCTIONNAME 1 /* Has __FUNCTION__ */
#  define CONFIG_HAVE_FILENAME     1 /* Has __FILE__ */
#  define __FUNCTION__ __func__      /* SDCC supports on __func__ */

/* Pragmas
 *
 * Disable warnings for unused function arguments
 */

# pragma disable_warning 85

/* C++ support */

#  undef CONFIG_HAVE_CXX14

/* Attributes
 *
 * SDCC does not support weak symbols
 */

#  undef  CONFIG_HAVE_WEAKFUNCTIONS
#  define weak_alias(name, aliasname)
#  define weak_data
#  define weak_function
#  define weak_const_function
#  define restrict /* REVISIT */

/* SDCC does not support the noreturn or packed attributes */

/* Current SDCC supports noreturn via C11 _Noreturn keyword (see
 * stdnoreturn.h).
 */

#  define noreturn_function
#  define locate_code(n)
#  define aligned_data(n)
#  define locate_data(n)
#  define begin_packed_struct
#  define end_packed_struct

/* REVISIT: */

#  define farcall_function

/* SDCC does support "naked" functions */

#  define naked_function __naked

/* SDCC does not support forced inlining. */

#  define inline_function
#  define noinline_function

/* The reentrant attribute informs SDCC that the function
 * must be reentrant.  In this case, SDCC will store input
 * arguments on the stack to support reentrancy.
 *
 * SDCC functions are always reentrant (except for the mcs51,
 * ds390, hc08 and s08 backends)
 */

#  define reentrant_function __reentrant

/* ISO C11 supports anonymous (unnamed) structures and unions.  Does SDCC? */

#  undef CONFIG_HAVE_ANONYMOUS_STRUCT
#  undef  CONFIG_HAVE_ANONYMOUS_UNION

/* Indicate that a local variable is not used */

#  define UNUSED(a) ((void)(a))

/* It is assumed that the system is build using the small
 * data model with storage defaulting to internal RAM.
 * The NEAR storage class can also be used to address data
 * in internal RAM; FAR can be used to address data in
 * external RAM.
 */

#if defined(__SDCC_z80) || defined(__SDCC_z180) || defined(__SDCC_gbz80)
#  define FAR
#  define NEAR
#  define CODE
#  define DSEG
#else
#  define FAR    __xdata
#  define NEAR   __data
#  define CODE   __code
#  if defined(SDCC_MODEL_SMALL)
#    define DSEG __data
#  else
#    define DSEG __xdata
#  endif
#endif

/* Select small, 16-bit address model */

#  define CONFIG_SMALL_MEMORY 1

/* Long and int are not the same size */

#  define CONFIG_LONG_IS_NOT_INT 1

/* The generic pointer and int are not the same size (for some SDCC
 * architectures).  REVISIT: SDCC now has more backends where pointers are
 * the same size as int than just z80 and z180.
 */

#if !defined(__z80) && !defined(__gbz80)
#  define CONFIG_PTR_IS_NOT_INT 1
#endif

/* SDCC does types long long and float, but not types double and long
 * double.
 */

#  define CONFIG_HAVE_LONG_LONG 1
#  define CONFIG_HAVE_FLOAT 1
#  undef  CONFIG_HAVE_DOUBLE
#  undef  CONFIG_HAVE_LONG_DOUBLE

/* Indicate that a local variable is not used */

#  define UNUSED(a) ((void)(a))

/* Zilog-specific definitions ***********************************************/

#elif defined(__ZILOG__)

/* At present, only the following Zilog compilers are recognized */

#  if !defined(__ZNEO__) && !defined(__EZ8__) && !defined(__EZ80__)
#    warning "Unrecognized Zilog compiler"
#  endif

/* Pre-processor */

#  undef CONFIG_CPP_HAVE_VARARGS /* No variable argument macros */
#  undef CONFIG_CPP_HAVE_WARNING /* Does not support #warning */

/* Intrinsics */

#  define CONFIG_HAVE_FUNCTIONNAME 1 /* Has __FUNCTION__ */
#  define CONFIG_HAVE_FILENAME     1 /* Has __FILE__ */

/* No I-space access qualifiers */

#  define IOBJ
#  define IPTR

/* C++ support */

#  undef CONFIG_HAVE_CXX14

/* Attributes
 *
 * The Zilog compiler does not support weak symbols
 */

#  undef  CONFIG_HAVE_WEAKFUNCTIONS
#  define weak_alias(name, aliasname)
#  define weak_data
#  define weak_function
#  define weak_const_function
#  define restrict

/* The Zilog compiler does not support the noreturn, packed, naked
 * attributes.
 */

#  define noreturn_function
#  define aligned_data(n)
#  define locate_code(n)
#  define locate_data(n)
#  define begin_packed_struct
#  define end_packed_struct
#  define naked_function
#  define inline_function
#  define noinline_function

/* REVISIT: */

#  define farcall_function

/* The Zilog compiler does not support the reentrant attribute */

#  define reentrant_function

/* Addressing.
 *
 * Z16F ZNEO:  Far is 24-bits; near is 16-bits of address.
 *             The supported model is (1) all code on ROM, and (2) all data
 *             and stacks in external (far) RAM.
 * Z8Encore!:  Far is 16-bits; near is 8-bits of address.
 *             The supported model is (1) all code on ROM, and (2) all data
 *             and stacks in internal (far) RAM.
 * Z8Acclaim:  In Z80 mode, all pointers are 16-bits.  In ADL mode, all
 *             pointers are 24 bits.
 */

#  if defined(__ZNEO__)
#    define FAR   _Far
#    define NEAR  _Near
#    define DSEG  _Far
#    define CODE  _Erom
#    undef  CONFIG_SMALL_MEMORY       /* Select the large, 32-bit addressing model */
#    undef  CONFIG_LONG_IS_NOT_INT    /* Long and int are the same size */
#    undef  CONFIG_PTR_IS_NOT_INT     /* FAR pointers and int are the same size */
#  elif defined(__EZ8__)
#    define FAR   far
#    define NEAR  near
#    define DSEG  far
#    define CODE  rom
#    define CONFIG_SMALL_MEMORY 1     /* Select small, 16-bit address model */
#    define CONFIG_LONG_IS_NOT_INT 1  /* Long and int are not the same size */
#    undef  CONFIG_PTR_IS_NOT_INT     /* FAR pointers and int are the same size */
#  elif defined(__EZ80__)
#    define FAR
#    define NEAR
#    define DSEG
#    define CODE
#    undef  CONFIG_SMALL_MEMORY       /* Select the large, 32-bit addressing model */
#    define CONFIG_LONG_IS_NOT_INT 1  /* Long and int are not the same size */
#    ifdef CONFIG_EZ80_Z80MODE
#      define CONFIG_PTR_IS_NOT_INT 1 /* Pointers and int are not the same size */
#    else
#      undef  CONFIG_PTR_IS_NOT_INT   /* Pointers and int are the same size */
#    endif
#  endif

/* ISO C11 supports anonymous (unnamed) structures and unions.  Zilog does
 * not support C11
 */

#  undef CONFIG_HAVE_ANONYMOUS_STRUCT
#  undef  CONFIG_HAVE_ANONYMOUS_UNION

/* Older Zilog compilers support both types double and long long, but the
 * size is 32-bits (same as long and single precision) so it is safer to say
 * that they are not supported.  Later versions are more ANSII compliant and
 * simply do not support long long or double.
 */

#  undef  CONFIG_HAVE_LONG_LONG
#  define CONFIG_HAVE_FLOAT 1
#  undef  CONFIG_HAVE_DOUBLE
#  undef  CONFIG_HAVE_LONG_DOUBLE

/* Indicate that a local variable is not used */

#  define UNUSED(a) ((void)(a))

/* ICCARM-specific definitions **********************************************/

#elif defined(__ICCARM__)

#  define CONFIG_CPP_HAVE_VARARGS 1 /* Supports variable argument macros */
#  define CONFIG_HAVE_FILENAME 1    /* Has __FILE__ */
#  define CONFIG_HAVE_FLOAT 1

/* Indicate that a local variable is not used */

#  define UNUSED(a) ((void)(a))

#  define weak_alias(name, aliasname)
#  define weak_data            __weak
#  define weak_function        __weak
#  define weak_const_function
#  define noreturn_function
#  define farcall_function
#  define locate_code(n)
#  define aligned_data(n)
#  define locate_data(n)
#  define begin_packed_struct  __packed
#  define end_packed_struct
#  define reentrant_function
#  define naked_function
#  define inline_function
#  define noinline_function

#  define FAR
#  define NEAR
#  define DSEG
#  define CODE
#  define IOBJ
#  define IPTR

#  define __asm__       asm
#  define __volatile__  volatile

/* For operatots __sfb() and __sfe() */

#  pragma section = ".bss"
#  pragma section = ".data"
#  pragma section = ".data_init"
#  pragma section = ".text"

/* C++ support */

#  undef CONFIG_HAVE_CXX14

/* ISO C11 supports anonymous (unnamed) structures and unions.  Does
 * ICCARM?
 */

#  undef CONFIG_HAVE_ANONYMOUS_STRUCT
#  undef  CONFIG_HAVE_ANONYMOUS_UNION

/* Unknown compiler *********************************************************/

#else

#  undef  CONFIG_CPP_HAVE_VARARGS
#  undef  CONFIG_CPP_HAVE_WARNING
#  undef  CONFIG_HAVE_FUNCTIONNAME
#  undef  CONFIG_HAVE_FILENAME
#  undef  CONFIG_HAVE_WEAKFUNCTIONS
#  undef CONFIG_HAVE_CXX14
#  define weak_alias(name, aliasname)
#  define weak_data
#  define weak_function
#  define weak_const_function
#  define restrict
#  define noreturn_function
#  define farcall_function
#  define aligned_data(n)
#  define locate_code(n)
#  define locate_data(n)
#  define begin_packed_struct
#  define end_packed_struct
#  define reentrant_function
#  define naked_function
#  define inline_function
#  define noinline_function

#  define FAR
#  define NEAR
#  define DSEG
#  define CODE

#  undef  CONFIG_SMALL_MEMORY
#  undef  CONFIG_LONG_IS_NOT_INT
#  undef  CONFIG_PTR_IS_NOT_INT
#  undef  CONFIG_HAVE_LONG_LONG
#  define CONFIG_HAVE_FLOAT 1
#  undef  CONFIG_HAVE_DOUBLE
#  undef  CONFIG_HAVE_LONG_DOUBLE
#  undef  CONFIG_HAVE_ANONYMOUS_STRUCT
#  undef  CONFIG_HAVE_ANONYMOUS_UNION

#  define UNUSED(a) ((void)(a))

#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_COMPILER_H */
