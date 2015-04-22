/****************************************************************************
 * include/assert.h
 *
 *   Copyright (C) 2007-2009, 2011-2013, 2015 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_ASSERT_H
#define __INCLUDE_ASSERT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Macro Name: ASSERT, VERIFY, et al. */

#undef ASSERT       /* Assert if the condition is not true */
#undef VERIFY       /* Assert if a function returns a negative value */
#undef DEBUGASSERT  /* Like ASSERT, but only if CONFIG_DEBUG is defined */
#undef DEBUGVERIFY  /* Like VERIFY, but only if CONFIG_DEBUG is defined */
#undef PANIC        /* Unconditional abort */

#ifdef CONFIG_HAVE_FILENAME

#  define ASSERT(f) \
     do \
       { \
         if (!(f)) \
           { \
             up_assert((const uint8_t *)__FILE__, (int)__LINE__); \
           } \
       } \
     while (0)

#  define VERIFY(f) \
     do \
       { \
         if ((f) < 0) \
           { \
             up_assert((const uint8_t *)__FILE__, (int)__LINE__); \
           } \
       } \
     while (0)

#  define PANIC() \
     up_assert((const uint8_t *)__FILE__, (int)__LINE__)

#  ifdef CONFIG_DEBUG

#    define DEBUGASSERT(f) \
       do \
         { \
           if (!(f)) \
             { \
               up_assert((const uint8_t *)__FILE__, (int)__LINE__); \
             } \
         } \
       while (0)

#    define DEBUGVERIFY(f) \
       do \
         { \
           if ((f) < 0) \
             { \
               up_assert((const uint8_t *)__FILE__, (int)__LINE__); \
             } \
         } \
       while (0)

#    define DEBUGPANIC() \
       up_assert((const uint8_t *)__FILE__, (int)__LINE__)

#  else

#    define DEBUGASSERT(f)
#    define DEBUGVERIFY(f) ((void)(f))
#    define DEBUGPANIC()

#  endif /* CONFIG_DEBUG */

#else

#  define ASSERT(f)        { if (!(f)) up_assert(); }
#  define VERIFY(f)        { if ((f) < 0) up_assert(); }
#  define PANIC()          up_assert()

#  ifdef CONFIG_DEBUG

#    define DEBUGASSERT(f) { if (!(f)) up_assert(); }
#    define DEBUGVERIFY(f) { if ((f) < 0) up_assert(); }
#    define DEBUGPANIC()   up_assert()

#  else

#    define DEBUGASSERT(f)
#    define DEBUGVERIFY(f) ((void)(f))
#    define DEBUGPANIC()

#  endif /* CONFIG_DEBUG */
#endif

#ifndef assert
#  define assert ASSERT
#endif

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_HAVE_FILENAME
void up_assert(FAR const uint8_t *filename, int linenum) noreturn_function;
#else
void up_assert(void) noreturn_function;
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_ASSERT_H */
