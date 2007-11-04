/************************************************************
 * compiler.h
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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
 ************************************************************/

#ifndef __COMPILER_H
#define __COMPILER_H

/************************************************************
 * Included Files
 ************************************************************/

/************************************************************
 * Definitions
 ************************************************************/

/* GCC-specific definitions *********************************/

#ifdef __GNUC__

/* GCC supports weak symbols which can be used to reduce
 * code size because unnecessary "weak" functions can be
 * excluded from the link.
 */

# ifndef __CYGWIN__
#  define CONFIG_HAVE_WEAKFUNCTIONS 1
#  define weak_alias(name, aliasname) \
   extern __typeof (name) aliasname __attribute__ ((weak, alias (#name)));
#  define weak_function __attribute__ ((weak))
#  define weak_const_function __attribute__ ((weak, __const__))
# else
#  undef  CONFIG_HAVE_WEAKFUNCTIONS
#  define weak_alias(name, aliasname)
#  define weak_function
#  define weak_const_function
#endif

/* The noreturn attribute informs GCC that the function will
 * not return.
 */

# define noreturn_function __attribute__ ((noreturn))

/* The packed attribute informs GCC that the stucture elements
 * are packed, ignoring other alignment rules.
 */

# define packed_struct __attribute__ ((packed))

/* GCC does not support the reentrant attribute */

# define reentrant_function

/* GCC has does not use storage classes to qualify addressing */

# define FAR
# define NEAR
# define DSEG
# define CODE

/* Select the large, 32-bit addressing model */

# undef  CONFIG_SMALL_MEMORY

/* Long and int are (probably) the same size */

# undef  CONFIG_LONG_IS_NOT_INT

/* Pointers and int are the same size */

# undef  CONFIG_PTR_IS_NOT_INT

/* GCC supports inlined functions */

# define CONFIG_HAVE_INLINE 1

/* GCC supports both types double and long long */

# define CONFIG_HAVE_DOUBLE 1
# define CONFIG_HAVE_LONG_LONG 1

/* Structures and unions can be assigned and passed as values */

# define CONFIG_CAN_PASS_STRUCTS 1

/* SDCC-specific definitions ********************************/

#elif defined(SDCC)

/* Disable warnings for unused function arguments */

# pragma disable_warning 85

/* SDCC does not support weak symbols */

# undef  CONFIG_HAVE_WEAKFUNCTIONS
# define weak_alias(name, aliasname)
# define weak_function
# define weak_const_function

/* SDCC does not support the noreturn or packed attributes */

# define noreturn_function
# define packed_struct

/* The reentrant attribute informs SDCC that the function
 * must be reentrant.  In this case, SDCC will store input
 * arguments on the stack to support reentrancy.
 */

# define reentrant_function __reentrant

/* It is assumed that the system is build using the small
 * data model with storage defaulting to internal RAM.
 * The NEAR storage class can also be used to address data
 * in internal RAM; FAR can be used to address data in
 * external RAM.
 */

#define FAR  __xdata
#define NEAR __data
#define CODE __code

#if defined(SDCC_MODEL_SMALL)
# define DSEG __data
#else
# define DSEG __xdata
#endif

/* Select small, 16-bit address model */

# define CONFIG_SMALL_MEMORY 1

/* Long and int are not the same size */

# define CONFIG_LONG_IS_NOT_INT 1

/* The generic pointer and int are not the same size
 * (for some SDCC architectures)
 */
 
#if !defined(__z80) && defined(__gbz80)
# define CONFIG_PTR_IS_NOT_INT 1
#endif

/* SDCC does not support inline functions */

# undef  CONFIG_HAVE_INLINE
# define inline

/* SDCC does not support type long long or type double */

# undef  CONFIG_HAVE_LONG_LONG
# undef  CONFIG_HAVE_DOUBLE

/* Structures and unions cannot be passed as values or used
 * in assignments.
 */

# undef  CONFIG_CAN_PASS_STRUCTS

/* Unknown compiler *****************************************/

#else

# undef  CONFIG_HAVE_WEAKFUNCTIONS
# define weak_alias(name, aliasname)
# define weak_function
# define weak_const_function
# define noreturn_function
# define packed_struct
# define reentrant_function

# define FAR
# define NEAR
# define DSEG
# define CODE

# undef  CONFIG_SMALL_MEMORY
# undef  CONFIG_LONG_IS_NOT_INT
# undef  CONFIG_PTR_IS_NOT_INT
# undef  CONFIG_HAVE_INLINE
# define inline
# undef  CONFIG_HAVE_LONG_LONG
# undef  CONFIG_HAVE_DOUBLE
# undef  CONFIG_CAN_PASS_STRUCTS

#endif

/************************************************************
 * Global Function Prototypes
 ************************************************************/

/************************************************************
 * Global Function Prototypes
 ************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif


#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __COMPILER_H */
