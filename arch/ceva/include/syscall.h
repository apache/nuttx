/****************************************************************************
 * arch/ceva/include/syscall.h
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

/* This file should never be included directed but, rather, only indirectly
 * through include/syscall.h or include/sys/sycall.h
 */

#ifndef __ARCH_CEVA_INCLUDE_SYSCALL_H
#define __ARCH_CEVA_INCLUDE_SYSCALL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* This logic uses three system calls {0,1,2} for context switching and one
 * for the syscall return.
 * So a minimum of four syscall values must be reserved.
 * If CONFIG_BUILD_FLAT isn't defined, then four more syscall values
 * must be reserved.
 */

#ifndef CONFIG_BUILD_FLAT
#  define CONFIG_SYS_RESERVED 8
#else
#  define CONFIG_SYS_RESERVED 4
#endif

/* CEVA system calls ********************************************************/

/* SYS call 0:
 *
 * int up_saveusercontext(void *saveregs);
 */

#define SYS_save_context          0x00

/* SYS call 1:
 *
 * void up_fullcontextrestore(uint32_t *restoreregs) noreturn_function;
 */

#define SYS_restore_context       0x01

/* SYS call 2:
 *
 * void up_switchcontext(uint32_t **saveregs, uint32_t *restoreregs);
 */

#define SYS_switch_context        0x02

#ifdef CONFIG_LIB_SYSCALL
/* SYS call 3:
 *
 * void up_syscall_return(void);
 */

#define SYS_syscall_return        0x03
#endif /* CONFIG_LIB_SYSCALL */

#ifndef CONFIG_BUILD_FLAT
/* SYS call 4:
 *
 * void up_task_start(main_t taskentry, int argc, char *argv[])
 *        noreturn_function;
 */

#define SYS_task_start            0x04

/* SYS call 5:
 *
 * void up_pthread_start(pthread_startroutine_t entrypt, pthread_addr_t arg)
 *        noreturn_function
 */

#define SYS_pthread_start         0x05

/* SYS call 6:
 *
 * void signal_handler(_sa_sigaction_t sighand, int signo,
 *                     siginfo_t *info, void *ucontext);
 */

#define SYS_signal_handler        0x06

/* SYS call 7:
 *
 * void signal_handler_return(void);
 */

#define SYS_signal_handler_return 0x07
#endif /* !CONFIG_BUILD_FLAT */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* TRAP call with SYS_ call number and no parameters */

uintptr_t sys_call0(unsigned int nbr);

/* TRAP call with SYS_ call number and one parameter */

uintptr_t sys_call1(unsigned int nbr, uintptr_t parm1);

/* TRAP call with SYS_ call number and two parameters */

uintptr_t sys_call2(unsigned int nbr, uintptr_t parm1,
                    uintptr_t parm2);

/* TRAP call with SYS_ call number and three parameters */

uintptr_t sys_call3(unsigned int nbr, uintptr_t parm1,
                    uintptr_t parm2, uintptr_t parm3);

/* TRAP call with SYS_ call number and four parameters */

uintptr_t sys_call4(unsigned int nbr, uintptr_t parm1,
                    uintptr_t parm2, uintptr_t parm3,
                    uintptr_t parm4);

/* TRAP call with SYS_ call number and five parameters */

uintptr_t sys_call5(unsigned int nbr, uintptr_t parm1,
                    uintptr_t parm2, uintptr_t parm3,
                    uintptr_t parm4, uintptr_t parm5);

/* TRAP call with SYS_ call number and six parameters */

uintptr_t sys_call6(unsigned int nbr, uintptr_t parm1,
                    uintptr_t parm2, uintptr_t parm3,
                    uintptr_t parm4, uintptr_t parm5,
                    uintptr_t parm6);

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_CEVA_INCLUDE_SYSCALL_H */
