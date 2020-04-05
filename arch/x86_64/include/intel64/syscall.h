/****************************************************************************
 * arch/x86_64/include/intel64/syscall.h
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

#ifndef __ARCH_X86_64_INCLUDE_INTEL64_SYSCALL_H
#define __ARCH_X86_64_INCLUDE_INTEL64_SYSCALL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
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

void enable_syscall(void);
void syscall_entry(void);
uint64_t syscall_handler(unsigned long nbr, uintptr_t parm1, uintptr_t parm2,
                         uintptr_t parm3, uintptr_t parm4, uintptr_t parm5,
                         uintptr_t parm6);
uint64_t linux_interface(unsigned long nbr, uintptr_t parm1, uintptr_t parm2,
                         uintptr_t parm3, uintptr_t parm4, uintptr_t parm5,
                         uintptr_t parm6);

/* SWI with SYS_ call number and six parameters */

static inline uintptr_t sys_call6(unsigned int nbr, uintptr_t parm1,
                                  uintptr_t parm2, uintptr_t parm3,
                                  uintptr_t parm4, uintptr_t parm5,
                                  uintptr_t parm6);

/* SWI with SYS_ call number and no parameters */

static inline uintptr_t sys_call0(unsigned int nbr)
{
  return sys_call6(nbr, 0, 0, 0, 0, 0, 0);
}

/* SWI with SYS_ call number and one parameter */

static inline uintptr_t sys_call1(unsigned int nbr, uintptr_t parm1)
{
  return sys_call6(nbr, parm1, 0, 0, 0, 0, 0);
}

/* SWI with SYS_ call number and two parameters */

static inline uintptr_t sys_call2(unsigned int nbr, uintptr_t parm1,
                                  uintptr_t parm2)
{
  return sys_call6(nbr, parm1, parm2, 0, 0, 0, 0);
}

/* SWI with SYS_ call number and three parameters */

static inline uintptr_t sys_call3(unsigned int nbr, uintptr_t parm1,
                                  uintptr_t parm2, uintptr_t parm3)
{
  return sys_call6(nbr, parm1, parm2, parm3, 0, 0, 0);
}

/* SWI with SYS_ call number and four parameters */

static inline uintptr_t sys_call4(unsigned int nbr, uintptr_t parm1,
                                  uintptr_t parm2, uintptr_t parm3,
                                  uintptr_t parm4)
{
  return sys_call6(nbr, parm1, parm2, parm3, parm4, 0, 0);
}

/* SWI with SYS_ call number and five parameters */

static inline uintptr_t sys_call5(unsigned int nbr, uintptr_t parm1,
                                  uintptr_t parm2, uintptr_t parm3,
                                  uintptr_t parm4, uintptr_t parm5)
{
  return sys_call6(nbr, parm1, parm2, parm3, parm4, parm5, 0);
}

static inline uintptr_t sys_call6(unsigned int nbr, uintptr_t parm1,
                                  uintptr_t parm2, uintptr_t parm3,
                                  uintptr_t parm4, uintptr_t parm5,
                                  uintptr_t parm6)
{
  register uint64_t reg0 __asm__("rax") = (uint64_t)(nbr);
  register uint64_t reg1 __asm__("rdi") = (uint64_t)(parm1);
  register uint64_t reg2 __asm__("rsi") = (uint64_t)(parm2);
  register uint64_t reg3 __asm__("rdx") = (uint64_t)(parm3);
  register uint64_t reg4 __asm__("r10") = (uint64_t)(parm4);
  register uint64_t reg5 __asm__("r8") = (uint64_t)(parm5);
  register uint64_t reg6 __asm__("r9") = (uint64_t)(parm6);

  __asm__ __volatile__
  (
    "syscall"
    : "=r"(reg0)
    : "r"(reg0), "r"(reg1), "r"(reg2),
      "r"(reg3), "r"(reg4), "r"(reg5), "r"(reg6)
    : "memory"
  );

  return reg0;
}

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_X86_64_INCLUDE_INTEL64_SYSCALL_H */

