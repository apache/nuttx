/****************************************************************************
 * include/assert.h
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

#ifndef __INCLUDE_ASSERT_H
#define __INCLUDE_ASSERT_H

#ifndef __ASSEMBLY__

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>

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

#ifndef CONFIG_HAVE_FILENAME
#  define __FILE__       NULL
#  define __LINE__       0
#endif

#define PANIC()          __assert(__FILE__, __LINE__, "panic")

#ifdef CONFIG_DEBUG_ASSERTIONS_EXPRESSION
#define ASSERT(f)        do { if (predict_false(!(f))) __assert(__FILE__, __LINE__, #f); } while (0)
#define VERIFY(f)        do { if (predict_false((f) < 0)) __assert(__FILE__, __LINE__, #f); } while (0)
#else
#define ASSERT(f)        do { if (predict_false(!(f))) __assert(__FILE__, __LINE__, NULL); } while (0)
#define VERIFY(f)        do { if (predict_false((f) < 0)) __assert(__FILE__, __LINE__, NULL); } while (0)
#endif

#ifdef CONFIG_DEBUG_ASSERTIONS
#  define DEBUGPANIC()   PANIC()
#  define DEBUGASSERT(f) ASSERT(f)
#  define DEBUGVERIFY(f) VERIFY(f)
#else
#  define DEBUGPANIC()
#  define DEBUGASSERT(f) ((void)(1 || (f)))
#  define DEBUGVERIFY(f) ((void)(f))
#endif

/* The C standard states that if NDEBUG is defined, assert will do nothing.
 * Users can define and undefine NDEBUG as they see fit to choose when assert
 * does something or does not do anything.
 */

#ifdef NDEBUG
#  define assert(f) ((void)(1 || (f)))
#else
#  define assert(f) ASSERT(f)
#endif

/* Suppress 3rd party library redefine _assert/__assert */

#define _assert _assert
#define __assert __assert

/* Definition required for C11 compile-time assertion checking.  The
 * static_assert macro simply expands to the _Static_assert keyword.
 */

#ifndef __cplusplus
#  if defined(__STDC_VERSION__) && __STDC_VERSION__  > 199901L
#    define static_assert _Static_assert
#  else
#    define static_assert(cond, msg) \
       extern int (*__static_assert_function (void)) \
       [!!sizeof (struct { int __error_if_negative: (cond) ? 2 : -1; })]
#  endif
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

/****************************************************************************
 * Name: _assert
 *
 * Description:
 *   This is the assert system call that performs the core dump etc. Function
 *   might not return if it is not safe to do so (in IRQ or in IDLE task).
 *
 ****************************************************************************/

void _assert(FAR const char *filename, int linenum, FAR const char *msg);

/****************************************************************************
 * Name: __assert
 *
 * Description:
 *   This is the user space assert procedure.
 *
 ****************************************************************************/

void __assert(FAR const char *filename, int linenum,
              FAR const char *msg) noreturn_function;

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __INCLUDE_ASSERT_H */
