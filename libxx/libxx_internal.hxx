//***************************************************************************
// lib/libxx_internal.h
//
//   Copyright (C) 2012-2013 Gregory Nutt. All rights reserved.
//   Author: Gregory Nutt <gnutt@nuttx.org>
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in
//    the documentation and/or other materials provided with the
//    distribution.
// 3. Neither the name NuttX nor the names of its contributors may be
//    used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
// OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
// AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
//***************************************************************************

#ifndef __LIBXX_LIBXX_INTERNAL_HXX
#define __LIBXX_LIBXX_INTERNAL_HXX

//***************************************************************************
// Included Files
//***************************************************************************

#include <nuttx/config.h>

//***************************************************************************
// Definitions
//***************************************************************************

// The NuttX C library an be build in two modes: (1) as a standard, C-libary
// that can be used by normal, user-space applications, or (2) as a special,
// kernel-mode C-library only used within the OS.  If NuttX is not being
// built as separated kernel- and user-space modules, then only the first
// mode is supported.

#if defined(CONFIG_NUTTX_KERNEL) && defined(__KERNEL__)
#  include <nuttx/kmalloc.h>
#  define lib_malloc(s)    kmalloc(s)
#  define lib_zalloc(s)    kzalloc(s)
#  define lib_realloc(p,s) krealloc(p,s)
#  define lib_free(p)      kfree(p)
#else
#  include <cstdlib>
#  define lib_malloc(s)    malloc(s)
#  define lib_zalloc(s)    zalloc(s)
#  define lib_realloc(p,s) realloc(p,s)
#  define lib_free(p)      free(p)
#endif

//***************************************************************************
// Public Types
//***************************************************************************/

typedef CODE void (*__cxa_exitfunc_t)(void *arg);

//***************************************************************************
// Public Variables
//***************************************************************************

extern "C" FAR void *__dso_handle;

//***************************************************************************
// Public Function Prototypes
//***************************************************************************

extern "C" int __cxa_atexit(__cxa_exitfunc_t func, void *arg, void *dso_handle);

#endif // __LIBXX_LIBXX_INTERNAL_HXX
