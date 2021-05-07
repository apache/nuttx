/****************************************************************************
 * libs/libnx/nxcontext.h
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
