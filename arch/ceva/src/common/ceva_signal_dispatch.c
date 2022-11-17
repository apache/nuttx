/****************************************************************************
 * arch/ceva/src/common/ceva_signal_dispatch.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>

#include <arch/syscall.h>

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
 *   Normally the a user-mode signalling handling stub will also execute
 *   before the ultimate signal handler is called.  See
 *   arch/ceva/src/[xc5/xm6]/ceva_signal_handler.  This function is the
 *   user-space, signal handler trampoline function.  It is called from
 *   up_signal_dispatch() in user-mode.
 *
 * Inputs:
 *   sighand - The address user-space signal handling function
 *   signo, info, and ucontext - Standard arguments to be passed to the
 *     signal handling function.
 *
 * Return:
 *   None.  This function does not return in the normal sense.  It returns
 *   via an architecture specific system call made by up_signal_handler().
 *   However, this will look like a normal return by the caller of
 *   up_signal_dispatch.
 *
 ****************************************************************************/

void up_signal_dispatch(_sa_sigaction_t sighand, int signo,
                        siginfo_t *info, void *ucontext)
{
  /* Let sys_call4() do all of the work */

  sys_call4(SYS_signal_handler, (uintptr_t)sighand, (uintptr_t)signo,
                  (uintptr_t)info, (uintptr_t)ucontext);
}

#endif /* (CONFIG_BUILD_PROTECTED || CONFIG_BUILD_KERNEL) && !CONFIG_DISABLE_PTHREAD */
