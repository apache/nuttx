/****************************************************************************
 * arch/arm/include/syscall.h
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

/* This file should never be included directly but, rather, only indirectly
 * through include/syscall.h or include/sys/sycall.h
 */

#ifndef __ARCH_ARM_INCLUDE_SYSCALL_H
#define __ARCH_ARM_INCLUDE_SYSCALL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

#define SYS_syscall 0x00

#if defined(__thumb__) || defined(__thumb2__)
#  define SYS_smhcall 0xab
#else
#  define SYS_smhcall 0x123456
#endif

/* Configuration ************************************************************/

/* This logic uses three system calls {0,1,2} for context switching and one
 * for the syscall return.
 * So a minimum of four syscall values must be reserved.
 * If CONFIG_BUILD_PROTECTED is defined, then four more syscall values must
 * be reserved.
 */

#ifndef CONFIG_BUILD_FLAT
#  define CONFIG_SYS_RESERVED 8
#else
#  define CONFIG_SYS_RESERVED 4
#endif

/* Cortex-M system calls ****************************************************/

/* SYS call 0:
 *
 * int arm_saveusercontext(uint32_t *saveregs);
 */

#define SYS_save_context          (0)

/* SYS call 1:
 *
 * void arm_fullcontextrestore(uint32_t *restoreregs) noreturn_function;
 */

#define SYS_restore_context       (1)

/* SYS call 2:
 *
 * void arm_switchcontext(uint32_t *saveregs, uint32_t *restoreregs);
 */

#define SYS_switch_context        (2)

#ifdef CONFIG_LIB_SYSCALL
/* SYS call 3:
 *
 * void arm_syscall_return(void);
 */

#define SYS_syscall_return        (3)
#endif /* CONFIG_LIB_SYSCALL */

#ifndef CONFIG_BUILD_FLAT
/* SYS call 4:
 *
 * void up_task_start(main_t taskentry, int argc, FAR char *argv[])
 *        noreturn_function;
 */

#define SYS_task_start            (4)

/* SYS call 5:
 *
 * void up_pthread_start((pthread_startroutine_t startup,
 *                        pthread_startroutine_t entrypt, pthread_addr_t arg)
 *        noreturn_function
 */

#define SYS_pthread_start         (5)

/* SYS call 6:
 *
 * void signal_handler(_sa_sigaction_t sighand,
 *                     int signo, FAR siginfo_t *info,
 *                     FAR void *ucontext);
 */

#define SYS_signal_handler        (6)

/* SYS call 7:
 *
 * void signal_handler_return(void);
 */

#define SYS_signal_handler_return (7)
#endif /* !CONFIG_BUILD_FLAT */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* SVC with SYS_ call number and no parameters */

static inline uintptr_t sys_call0(unsigned int nbr)
{
  register long reg0 __asm__("r0") = (long)(nbr);

  __asm__ __volatile__
  (
    "svc %1"
    : "=r"(reg0)
    : "i"(SYS_syscall), "r"(reg0)
    : "memory", "r14"
  );

  return reg0;
}

/* SVC with SYS_ call number and one parameter */

static inline uintptr_t sys_call1(unsigned int nbr, uintptr_t parm1)
{
  register long reg0 __asm__("r0") = (long)(nbr);
  register long reg1 __asm__("r1") = (long)(parm1);

  __asm__ __volatile__
  (
    "svc %1"
    : "=r"(reg0)
    : "i"(SYS_syscall), "r"(reg0), "r"(reg1)
    : "memory", "r14"
  );

  return reg0;
}

/* SVC with SYS_ call number and two parameters */

static inline uintptr_t sys_call2(unsigned int nbr, uintptr_t parm1,
                                  uintptr_t parm2)
{
  register long reg0 __asm__("r0") = (long)(nbr);
  register long reg2 __asm__("r2") = (long)(parm2);
  register long reg1 __asm__("r1") = (long)(parm1);

  __asm__ __volatile__
  (
    "svc %1"
    : "=r"(reg0)
    : "i"(SYS_syscall), "r"(reg0), "r"(reg1), "r"(reg2)
    : "memory", "r14"
  );

  return reg0;
}

/* SVC with SYS_ call number and three parameters */

static inline uintptr_t sys_call3(unsigned int nbr, uintptr_t parm1,
                                  uintptr_t parm2, uintptr_t parm3)
{
  register long reg0 __asm__("r0") = (long)(nbr);
  register long reg3 __asm__("r3") = (long)(parm3);
  register long reg2 __asm__("r2") = (long)(parm2);
  register long reg1 __asm__("r1") = (long)(parm1);

  __asm__ __volatile__
  (
    "svc %1"
    : "=r"(reg0)
    : "i"(SYS_syscall), "r"(reg0), "r"(reg1), "r"(reg2), "r"(reg3)
    : "memory", "r14"
  );

  return reg0;
}

/* SVC with SYS_ call number and four parameters */

static inline uintptr_t sys_call4(unsigned int nbr, uintptr_t parm1,
                                  uintptr_t parm2, uintptr_t parm3,
                                  uintptr_t parm4)
{
  register long reg0 __asm__("r0") = (long)(nbr);
  register long reg4 __asm__("r4") = (long)(parm4);
  register long reg3 __asm__("r3") = (long)(parm3);
  register long reg2 __asm__("r2") = (long)(parm2);
  register long reg1 __asm__("r1") = (long)(parm1);

  __asm__ __volatile__
  (
    "svc %1"
    : "=r"(reg0)
    : "i"(SYS_syscall), "r"(reg0), "r"(reg1), "r"(reg2),
      "r"(reg3), "r"(reg4)
    : "memory", "r14"
  );

  return reg0;
}

/* SVC with SYS_ call number and five parameters */

static inline uintptr_t sys_call5(unsigned int nbr, uintptr_t parm1,
                                  uintptr_t parm2, uintptr_t parm3,
                                  uintptr_t parm4, uintptr_t parm5)
{
  register long reg0 __asm__("r0") = (long)(nbr);
  register long reg5 __asm__("r5") = (long)(parm5);
  register long reg4 __asm__("r4") = (long)(parm4);
  register long reg3 __asm__("r3") = (long)(parm3);
  register long reg2 __asm__("r2") = (long)(parm2);
  register long reg1 __asm__("r1") = (long)(parm1);

  __asm__ __volatile__
  (
    "svc %1"
    : "=r"(reg0)
    : "i"(SYS_syscall), "r"(reg0), "r"(reg1), "r"(reg2),
      "r"(reg3), "r"(reg4), "r"(reg5)
    : "memory", "r14"
  );

  return reg0;
}

/* SVC with SYS_ call number and six parameters */

static inline uintptr_t sys_call6(unsigned int nbr, uintptr_t parm1,
                                  uintptr_t parm2, uintptr_t parm3,
                                  uintptr_t parm4, uintptr_t parm5,
                                  uintptr_t parm6)
{
  register long reg0 __asm__("r0") = (long)(nbr);
  register long reg6 __asm__("r6") = (long)(parm6);
  register long reg5 __asm__("r5") = (long)(parm5);
  register long reg4 __asm__("r4") = (long)(parm4);
  register long reg3 __asm__("r3") = (long)(parm3);
  register long reg2 __asm__("r2") = (long)(parm2);
  register long reg1 __asm__("r1") = (long)(parm1);

  __asm__ __volatile__
  (
    "svc %1"
    : "=r"(reg0)
    : "i"(SYS_syscall), "r"(reg0), "r"(reg1), "r"(reg2),
      "r"(reg3), "r"(reg4), "r"(reg5), "r"(reg6)
    : "memory", "r14"
  );

  return reg0;
}

/* semihosting(SMH) call with call number and one parameter */

static inline long smh_call(unsigned int nbr, void *parm)
{
  register long reg0 __asm__("r0") = (long)(nbr);
  register long reg1 __asm__("r1") = (long)(parm);

  __asm__ __volatile__
  (
#if defined(CONFIG_ARCH_ARMV6M) || \
    defined(CONFIG_ARCH_ARMV7M) || \
    defined(CONFIG_ARCH_ARMV8M)
  "bkpt %1"
#else
  "svc %1"
#endif
    : "=r"(reg0)
    : "i"(SYS_smhcall), "r"(reg0), "r"(reg1)
    : "memory", "r14"
  );

  return reg0;
}

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_INCLUDE_SYSCALL_H */
