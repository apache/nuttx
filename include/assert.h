/****************************************************************************
 * include/assert.h
 *
 *   Copyright (C) 2007-2009, 2011-2013, 2015-2016 Gregory Nutt.
 *   All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 *   Copyright (C) 2016 Omni Hoverboards Inc. All rights reserved.
 *   Author: Paul Alexander Patience <paul-a.patience@polymtl.ca>
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

/* Macro Name: PANIC, ASSERT, VERIFY, et al. */

#undef PANIC        /* Unconditional abort */
#undef ASSERT       /* Assert if the condition is not true */
#undef VERIFY       /* Assert if a function returns a negative value */
#undef DEBUGPANIC   /* Like PANIC, but only if CONFIG_DEBUG_ASSERTIONS is defined */
#undef DEBUGASSERT  /* Like ASSERT, but only if CONFIG_DEBUG_ASSERTIONS is defined */
#undef DEBUGVERIFY  /* Like VERIFY, but only if CONFIG_DEBUG_ASSERTIONS is defined */

#ifdef CONFIG_HAVE_FILENAME
#  define PANIC()        _assert(__FILE__, __LINE__)
#else
#  define PANIC()        _assert("unknown", 0)
#endif

#define ASSERT(f)        do { if (!(f)) PANIC(); } while (0)
#define VERIFY(f)        do { if ((f) < 0) PANIC(); } while (0)

#ifdef CONFIG_DEBUG_ASSERTIONS
#  define DEBUGPANIC()   PANIC()
#  define DEBUGASSERT(f) ASSERT(f)
#  define DEBUGVERIFY(f) VERIFY(f)
#else
#  define DEBUGPANIC()
#  define DEBUGASSERT(f)
#  define DEBUGVERIFY(f) ((void)(f))
#endif

/* The C standard states that if NDEBUG is defined, assert will do nothing.
 * Users can define and undefine NDEBUG as they see fit to choose when assert
 * does something or does not do anything.
 */

#ifdef NDEBUG
#  define assert(f)
#else
#  define assert(f) ASSERT(f)
#endif

/* Definition required for C11 compile-time assertion checking.  The
 * static_assert macro simply expands to the _Static_assert keyword.
 */

#ifndef __cplusplus
#  define static_assert _Static_assert
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

void _assert(FAR const char *filename, int linenum) noreturn_function;

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_ASSERT_H */
