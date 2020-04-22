/****************************************************************************
 * arch/arm/include/armv8-m/syscall.h
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

#ifndef __ARCH_ARM_INCLUDE_ARMV8_M_SYSCALL_H
#define __ARCH_ARM_INCLUDE_ARMV8_M_SYSCALL_H

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

/* This is the value used as the argument to the SVC instruction.  It is not
 * used.
 */

#define SYS_syscall 0x00
#define SYS_smhcall 0xab

/* The SYS_signal_handler_return is executed here... its value is not always
 * available in this context and so is assumed to be 7.
 */

#ifndef SYS_signal_handler_return
#  define SYS_signal_handler_return (7)
#elif SYS_signal_handler_return != 7
#  error "SYS_signal_handler_return was assumed to be 7"
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* SVC call with SYS_ call number and no parameters */

static inline uintptr_t sys_call0(unsigned int nbr)
{
  register long reg0 __asm__("r0") = (long)(nbr);

  __asm__ __volatile__
  (
    "svc %1"
    : "=r"(reg0)
    : "i"(SYS_syscall), "r"(reg0)
    : "memory"
  );

  return reg0;
}

/* SVC call with SYS_ call number and one parameter */

static inline uintptr_t sys_call1(unsigned int nbr, uintptr_t parm1)
{
  register long reg0 __asm__("r0") = (long)(nbr);
  register long reg1 __asm__("r1") = (long)(parm1);

  __asm__ __volatile__
  (
    "svc %1"
    : "=r"(reg0)
    : "i"(SYS_syscall), "r"(reg0), "r"(reg1)
    : "memory"
  );

  return reg0;
}

/* SVC call with SYS_ call number and two parameters */

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
    : "memory"
  );

  return reg0;
}

/* SVC call with SYS_ call number and three parameters */

static inline uintptr_t sys_call4(unsigned int nbr, uintptr_t parm1,
                                  uintptr_t parm2, uintptr_t parm3,
                                  uintptr_t parm4);
static inline uintptr_t sys_call3(unsigned int nbr, uintptr_t parm1,
                                  uintptr_t parm2, uintptr_t parm3)
{
  return sys_call4(nbr, parm1, parm2, parm3, 0);
}

/* SVC call with SYS_ call number and four parameters.
 *
 * NOTE the nonstandard parameter passing:  parm4 is in R4
 */

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
    : "memory"
  );

  return reg0;
}

/* SVC call with SYS_ call number and five parameters.
 *
 * NOTE the nonstandard parameter passing:  parm4 and parm5 are in R4 and R5
 */

static inline uintptr_t sys_call6(unsigned int nbr, uintptr_t parm1,
                                  uintptr_t parm2, uintptr_t parm3,
                                  uintptr_t parm4, uintptr_t parm5,
                                  uintptr_t parm6);
static inline uintptr_t sys_call5(unsigned int nbr, uintptr_t parm1,
                                  uintptr_t parm2, uintptr_t parm3,
                                  uintptr_t parm4, uintptr_t parm5)
{
  return sys_call6(nbr, parm1, parm2, parm3, parm4, parm5, 0);
}

/* SVC call with SYS_ call number and six parameters.
 *
 * NOTE the nonstandard parameter passing:  parm4-parm6 are in R4-R6
 */

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
    : "memory"
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
  "bkpt %1"
    : "=r"(reg0)
    : "i"(SYS_smhcall), "r"(reg0), "r"(reg1)
    : "memory"
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
#endif /* __ARCH_ARM_INCLUDE_ARMV8_M_SYSCALL_H */
