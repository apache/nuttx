/************************************************************************************
 * arch/arm/src/armv7-r/svcall.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_ARMV7_R_SVCALL_H
#define __ARCH_ARM_SRC_ARMV7_R_SVCALL_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_LIB_SYSCALL
#  include <syscall.h>
#endif

#ifdef CONFIG_LIB_SYSCALL

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Configuration ********************************************************************/
/* This logic uses one system call for the syscall return.  So a minimum of one
 * syscall values must be reserved.  If CONFIG_BUILD_PROTECTED is defined, then four
 * more syscall values must be reserved.
 */

#ifdef CONFIG_BUILD_PROTECTED
#  ifndef CONFIG_SYS_RESERVED
#    error "CONFIG_SYS_RESERVED must be defined to have the value 6"
#  elif CONFIG_SYS_RESERVED != 6
#    error "CONFIG_SYS_RESERVED must have the value 6"
#  endif
#else
#  ifndef CONFIG_SYS_RESERVED
#    error "CONFIG_SYS_RESERVED must be defined to have the value 1"
#  elif CONFIG_SYS_RESERVED != 1
#    error "CONFIG_SYS_RESERVED must have the value 1"
#  endif
#endif

/* Cortex-R system calls ************************************************************/

/* SYS call 0:
 *
 * void up_syscall_return(void);
 */

#define SYS_syscall_return        (0)

#ifdef CONFIG_BUILD_PROTECTED
/* SYS call 1:
 *
 * void up_fullcontextrestore(uint32_t *restoreregs) noreturn_function;
 */

#define SYS_context_restore       (1)

/* SYS call 2:
 *
 * void up_task_start(main_t taskentry, int argc, FAR char *argv[])
 *        noreturn_function;
 */

#define SYS_task_start            (2)

/* SYS call 3:
 *
 * void up_pthread_start(pthread_startroutine_t entrypt, pthread_addr_t arg)
 *        noreturn_function
 */

#define SYS_pthread_start         (3)

/* SYS call 4:
 *
 * void signal_handler(_sa_sigaction_t sighand, int signo, FAR siginfo_t *info,
 *                     FAR void *ucontext);
 */

#define SYS_signal_handler        (4)

/* SYS call 5:
 *
 * void signal_handler_return(void);
 */

#define SYS_signal_handler_return (5)

#endif /* CONFIG_BUILD_PROTECTED */

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

#endif /* CONFIG_LIB_SYSCALL */
#endif  /* __ARCH_ARM_SRC_ARMV7_R_SVCALL_H */
