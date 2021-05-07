/****************************************************************************
 * arch/arm/include/armv7-a/syscall.h
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

#ifndef __ARCH_ARM_INCLUDE_ARMV7_A_SYSCALL_H
#define __ARCH_ARM_INCLUDE_ARMV7_A_SYSCALL_H

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
  "svc %1"
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
#endif /* __ARCH_ARM_INCLUDE_ARMV7_A_SYSCALL_H */
