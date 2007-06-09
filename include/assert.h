/************************************************************
 * assert.h
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

#ifndef __ASSERT_H
#define __ASSERT_H

/************************************************************
 * Included Files
 ************************************************************/

#include <sys/types.h>

/************************************************************
 * Definitions
 ************************************************************/

/* Macro Name: ASSERT, ASSERTCODE, et al. */

#undef ASSERT
#undef ASSERTFILE
#undef ASSERTCODE
#undef DEBUGASSERT

#if defined(__GNUC__) || defined(SDCC)

#  define ASSERT(f) \
     { if (!(f)) up_assert((const ubyte *)__FILE__, (int)__LINE__); }

#  define ASSERTCODE(f, errCode) \
     { if (!(f)) up_assert_code((const ubyte *)__FILE__, (int)__LINE__, errCode); }

#  ifdef CONFIG_DEBUG
#    define DEBUGASSERT(f) \
       { if (!(f)) up_assert((const ubyte *)__FILE__, (int)__LINE__); }
#  else
#    define DEBUGASSERT(f)
#  endif /* CONFIG_DEBUG */

#  define PANIC(errCode) \
      up_assert_code((const ubyte *)__FILE__, (int)__LINE__, (errCode)|0x8000)

#else
#  define ASSERT(f) \
     { if (!(f)) up_assert(); }

#  define ASSERTCODE(f, errCode) \
     { if (!(f)) up_assert_code(errCode); }

#  ifdef CONFIG_DEBUG
#    define DEBUGASSERT(f) \
       { if (!(f)) up_assert(); }
#  else
#    define DEBUGASSERT(f)
#  endif /* CONFIG_DEBUG */

#  define PANIC(errCode) \
      up_assert_code((errCode)|0x8000)

#endif

/************************************************************
 * Included Files
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

#if defined(__GNUC__) || defined(SDCC)
EXTERN void   up_assert(const ubyte *filename, int linenum);
EXTERN void   up_assert_code(const ubyte *filename, int linenum,
                             int error_code);
#else
EXTERN void   up_assert(void);
EXTERN void   up_assert_code(int error_code);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSERT_H */
