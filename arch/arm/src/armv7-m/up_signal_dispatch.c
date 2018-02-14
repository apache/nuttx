/****************************************************************************
 * arch/arm/src/armv7-m/up_signal_dispatch.c
 *
 *   Copyright (C) 2013-2014 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>

#include "svcall.h"
#include "up_internal.h"

#if ((defined(CONFIG_BUILD_PROTECTED) && defined(__KERNEL__)) || \
      defined(CONFIG_BUILD_KERNEL)) && !defined(CONFIG_DISABLE_SIGNALS)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_signal_dispatch
 *
 * Description:
 *   In this kernel mode build, this function will be called to execute a
 *   a signal handler in user-space.  When the signal is delivered, a
 *   kernel-mode stub will first run to perform some housekeeping functions.
 *   This kernel-mode stub will then be called transfer control to the user
 *   mode signal handler by calling this function.
 *
 *   Normally the a user-mode signaling handling stub will also execute
 *   before the ultimate signal handler is called.  See
 *   arch/arm/src/armv[6\7]/up_signal_handler.  This function is the
 *   user-space, signal handler trampoline function.  It is called from
 *   up_signal_dispatch() in user-mode.
 *
 * Input Parameters:
 *   sighand - The address user-space signal handling function
 *   signo, info, and ucontext - Standard arguments to be passed to the
 *     signal handling function.
 *
 * Returned Value:
 *   None.  This function does not return in the normal sense.  It returns
 *   via an architecture specific system call made by up_signal_handler().
 *   However, this will look like a normal return by the caller of
 *   up_signal_dispatch.
 *
 ****************************************************************************/

void up_signal_dispatch(_sa_sigaction_t sighand, int signo,
                        FAR siginfo_t *info, FAR void *ucontext)
{
  /* Let sys_call4() do all of the work */

  (void)sys_call4(SYS_signal_handler, (uintptr_t)sighand, (uintptr_t)signo,
                  (uintptr_t)info, (uintptr_t)ucontext);
}

#endif /* (CONFIG_BUILD_PROTECTED || CONFIG_BUILD_KERNEL) && !CONFIG_DISABLE_PTHREAD */
