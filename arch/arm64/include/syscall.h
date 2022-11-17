/****************************************************************************
 * arch/arm64/include/syscall.h
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

#ifndef __ARCH_ARM64_INCLUDE_SYSCALL_H
#define __ARCH_ARM64_INCLUDE_SYSCALL_H

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
#define SYS_smhcall 0xf000

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

/* system calls */

/* SYS call 0:
 *
 * int up_saveusercontext(void *saveregs);
 */

#define SYS_save_context          (0)

/* SYS call 1:
 *
 * void arm64_fullcontextrestore(void *restoreregs) noreturn_function;
 */

#define SYS_restore_context       (1)

/* SYS call 2:
 *
 * void arm64_switchcontext(void **saveregs, void *restoreregs);
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
 * void up_task_start(main_t taskentry, int argc, char *argv[])
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
 *                     int signo, siginfo_t *info,
 *                     void *ucontext);
 */

#define SYS_signal_handler        (6)

/* SYS call 7:
 *
 * void signal_handler_return(void);
 */

#define SYS_signal_handler_return (7)
#endif /* !CONFIG_BUILD_FLAT */

#define ARM_SMCC_RES_A0       (0)
#define ARM_SMCC_RES_A1       (1)
#define ARM_SMCC_RES_A2       (2)
#define ARM_SMCC_RES_A3       (3)
#define ARM_SMCC_RES_A4       (4)
#define ARM_SMCC_RES_A5       (5)
#define ARM_SMCC_RES_A6       (6)
#define ARM_SMCC_RES_A7       (7)

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Result from SMC/HVC call
 * a0-a7 result values from registers 0 to 7
 */

struct arm64_smccc_res
{
  unsigned long a0;
  unsigned long a1;
  unsigned long a2;
  unsigned long a3;
  unsigned long a4;
  unsigned long a5;
  unsigned long a6;
  unsigned long a7;
};

typedef struct arm64_smccc_res arm64_smccc_res_t;

enum arm64_smccc_conduit
{
  SMCCC_CONDUIT_NONE,
  SMCCC_CONDUIT_SMC,
  SMCCC_CONDUIT_HVC,
};

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/* SVC with SYS_ call number and no parameters */

static inline uintptr_t sys_call0(unsigned int nbr)
{
  register uint64_t reg0 __asm__("x0") = (uint64_t)(nbr);

  __asm__ __volatile__
  (
    "svc %1"
    : "=r"(reg0)
    : "i"(SYS_syscall), "r"(reg0)
    : "memory", "x30"
  );

  return reg0;
}

/* SVC with SYS_ call number and one parameter */

static inline uintptr_t sys_call1(unsigned int nbr, uintptr_t parm1)
{
  register uint64_t reg0 __asm__("x0") = (uint64_t)(nbr);
  register uint64_t reg1 __asm__("x1") = (uint64_t)(parm1);

  __asm__ __volatile__
  (
    "svc %1"
    : "=r"(reg0)
    : "i"(SYS_syscall), "r"(reg0), "r"(reg1)
    : "memory", "x30"
  );

  return reg0;
}

/* SVC with SYS_ call number and two parameters */

static inline uintptr_t sys_call2(unsigned int nbr, uintptr_t parm1,
                                  uintptr_t parm2)
{
  register uint64_t reg0 __asm__("x0") = (uint64_t)(nbr);
  register uint64_t reg2 __asm__("x2") = (uint64_t)(parm2);
  register uint64_t reg1 __asm__("x1") = (uint64_t)(parm1);

  __asm__ __volatile__
  (
    "svc %1"
    : "=r"(reg0)
    : "i"(SYS_syscall), "r"(reg0), "r"(reg1), "r"(reg2)
    : "memory", "x30"
  );

  return reg0;
}

/* SVC with SYS_ call number and three parameters */

static inline uintptr_t sys_call3(unsigned int nbr, uintptr_t parm1,
                                  uintptr_t parm2, uintptr_t parm3)
{
  register uint64_t reg0 __asm__("x0") = (uint64_t)(nbr);
  register uint64_t reg3 __asm__("x3") = (uint64_t)(parm3);
  register uint64_t reg2 __asm__("x2") = (uint64_t)(parm2);
  register uint64_t reg1 __asm__("x1") = (uint64_t)(parm1);

  __asm__ __volatile__
  (
    "svc %1"
    : "=r"(reg0)
    : "i"(SYS_syscall), "r"(reg0), "r"(reg1), "r"(reg2),
      "r"(reg3)
    : "memory", "x30"
  );

  return reg0;
}

/* SVC with SYS_ call number and four parameters */

static inline uintptr_t sys_call4(unsigned int nbr, uintptr_t parm1,
                                  uintptr_t parm2, uintptr_t parm3,
                                  uintptr_t parm4)
{
  register uint64_t reg0 __asm__("x0") = (uint64_t)(nbr);
  register uint64_t reg4 __asm__("x4") = (uint64_t)(parm4);
  register uint64_t reg3 __asm__("x3") = (uint64_t)(parm3);
  register uint64_t reg2 __asm__("x2") = (uint64_t)(parm2);
  register uint64_t reg1 __asm__("x1") = (uint64_t)(parm1);

  __asm__ __volatile__
  (
    "svc %1"
    : "=r"(reg0)
    : "i"(SYS_syscall), "r"(reg0), "r"(reg1), "r"(reg2),
      "r"(reg3), "r"(reg4)
    : "memory", "x30"
  );

  return reg0;
}

/* SVC with SYS_ call number and five parameters */

static inline uintptr_t sys_call5(unsigned int nbr, uintptr_t parm1,
                                  uintptr_t parm2, uintptr_t parm3,
                                  uintptr_t parm4, uintptr_t parm5)
{
  register uint64_t reg0 __asm__("x0") = (uint64_t)(nbr);
  register uint64_t reg5 __asm__("x5") = (uint64_t)(parm5);
  register uint64_t reg4 __asm__("x4") = (uint64_t)(parm4);
  register uint64_t reg3 __asm__("x3") = (uint64_t)(parm3);
  register uint64_t reg2 __asm__("x2") = (uint64_t)(parm2);
  register uint64_t reg1 __asm__("x1") = (uint64_t)(parm1);

  __asm__ __volatile__
  (
    "svc %1"
    : "=r"(reg0)
    : "i"(SYS_syscall), "r"(reg0), "r"(reg1), "r"(reg2),
      "r"(reg3), "r"(reg4), "r"(reg5)
    : "memory", "x30"
  );

  return reg0;
}

/* SVC with SYS_ call number and six parameters */

static inline uintptr_t sys_call6(unsigned int nbr, uintptr_t parm1,
                                  uintptr_t parm2, uintptr_t parm3,
                                  uintptr_t parm4, uintptr_t parm5,
                                  uintptr_t parm6)
{
  register uint64_t reg0 __asm__("x0") = (uint64_t)(nbr);
  register uint64_t reg6 __asm__("x6") = (uint64_t)(parm6);
  register uint64_t reg5 __asm__("x5") = (uint64_t)(parm5);
  register uint64_t reg4 __asm__("x4") = (uint64_t)(parm4);
  register uint64_t reg3 __asm__("x3") = (uint64_t)(parm3);
  register uint64_t reg2 __asm__("x2") = (uint64_t)(parm2);
  register uint64_t reg1 __asm__("x1") = (uint64_t)(parm1);

  __asm__ __volatile__
  (
    "svc %1"
    : "=r"(reg0)
    : "i"(SYS_syscall), "r"(reg0), "r"(reg1), "r"(reg2),
      "r"(reg3), "r"(reg4), "r"(reg5), "r"(reg6)
    : "memory", "x30"
  );

  return reg0;
}

/* semihosting(SMH) call with call number and one parameter */

static inline long smh_call(unsigned int nbr, void *parm)
{
  register uint64_t reg0 __asm__("x0") = (uint64_t)(nbr);
  register uint64_t reg1 __asm__("x1") = (uint64_t)(parm);

  __asm__ __volatile__
  (
  "hlt %1"
    : "=r"(reg0)
    : "i"(SYS_smhcall), "r"(reg0), "r"(reg1)
    : "memory", "x30"
  );

  return reg0;
}

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Make HVC calls
 *
 * param a0 function identifier
 * param a1-a7 parameters registers
 * param res results
 */

void arm64_smccc_hvc(unsigned long a0, unsigned long a1,
                     unsigned long a2, unsigned long a3,
                     unsigned long a4, unsigned long a5,
                     unsigned long a6, unsigned long a7,
                     struct arm64_smccc_res *res);

/* Make SMC calls
 *
 * param a0 function identifier
 * param a1-a7 parameters registers
 * param res results
 */

void arm64_smccc_smc(unsigned long a0, unsigned long a1,
                     unsigned long a2, unsigned long a3,
                     unsigned long a4, unsigned long a5,
                     unsigned long a6, unsigned long a7,
                     struct arm64_smccc_res *res);

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

#endif /* __ARCH_ARM64_INCLUDE_SYSCALL_H */
