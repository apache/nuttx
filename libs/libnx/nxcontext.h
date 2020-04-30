/****************************************************************************
 * libs/libnx/nxcontext.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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

#ifndef _LIBNX_NXCONTEXT_H
#define _LIBNX_NXCONTEXT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>

#include <nuttx/kmalloc.h>
#include <nuttx/streams.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The NuttX NX library can be build in two modes: (1) as a standard, C-
 * library that can be used by normal, user-space applications, or (2) as
 * a special, kernel-mode NX-library only used within the OS.  If NuttX is
 * not being built as separated kernel- and user-space modules, then only
 * the first mode is supported.
 */

#if !defined(CONFIG_BUILD_FLAT) && defined(__KERNEL__)

  /* Domain-specific allocations */

#  define lib_malloc(s)     kmm_malloc(s)
#  define lib_zalloc(s)     kmm_zalloc(s)
#  define lib_realloc(p,s)  kmm_realloc(p,s)
#  define lib_memalign(p,s) kmm_memalign(p,s)
#  define lib_free(p)       kmm_free(p)

  /* User-accessible allocations */

#  define lib_umalloc(s)    kumm_malloc(s)
#  define lib_uzalloc(s)    kumm_zalloc(s)
#  define lib_urealloc(p,s) kumm_realloc(p,s)
#  define lib_ufree(p)      kumm_free(p)

#else

  /* Domain-specific allocations */

#  define lib_malloc(s)     malloc(s)
#  define lib_zalloc(s)     zalloc(s)
#  define lib_realloc(p,s)  realloc(p,s)
#  define lib_free(p)       free(p)

  /* User-accessible allocations */

#  define lib_umalloc(s)    malloc(s)
#  define lib_uzalloc(s)    zalloc(s)
#  define lib_urealloc(p,s) realloc(p,s)
#  define lib_ufree(p)      free(p)

#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* _LIBNX_NXCONTEXT_H */
