/************************************************************************
 * include/nuttx/errno.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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
 ************************************************************************/

#ifndef __INCLUDE_NUTTX_ERRNO_H
#define __INCLUDE_NUTTX_ERRNO_H

/************************************************************************
 * Included Files
 ************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_LIB_SYSCALL

/************************************************************************
 * Pre-processor Definitions
 ************************************************************************/

/************************************************************************
 * Type Declarations
 ************************************************************************/

/************************************************************************
 * Public Data
 ************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************
 * Public Function Prototypes
 ************************************************************************/
/* These get_errno() set_errno() are always exported as functions from
 * the kernel always when the system call interface is built.  errno.h
 * provides the context-specific usage interface for both the kernel- and
 * user-spaces which may be different:  These are macros within the
 * kernel and system call proxy signatures from user space.
 *
 * This header file always exists to disambuiguate the context.  The
 * prototypes here duplicate thos of errno.h and must exactly match.
 */

void set_errno(int errcode);
int  get_errno(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_LIB_SYSCALL */
#endif /* __INCLUDE_NUTTX_ERRNO_H */
