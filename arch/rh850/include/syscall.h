/****************************************************************************
 * arch/rh850/include/syscall.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_RH850_INCLUDE_SYSCALL_H
#define __ARCH_RH850_INCLUDE_SYSCALL_H

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

/* Reserved system call numbers, passed in r6 to TRAP 0 */

#define SYS_restore_context       (1)
#define SYS_switch_context        (2)

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
extern "C"
{
#endif

/* TRAP 0x0: r6 = call number, r7-r9 then r10-r12 = arguments, r10 =
 * return value.
 */

static inline uintptr_t sys_call0(unsigned int nbr)
{
  register uintptr_t reg0 __asm__("r6") = (uintptr_t)nbr;
  register uintptr_t ret __asm__("r10");

  __asm__ __volatile__("trap 0x0" : "=r"(ret) : "r"(reg0) : "memory");
  return ret;
}

static inline uintptr_t sys_call1(unsigned int nbr, uintptr_t parm1)
{
  register uintptr_t reg0 __asm__("r6") = (uintptr_t)nbr;
  register uintptr_t reg1 __asm__("r7") = parm1;
  register uintptr_t ret __asm__("r10");

  __asm__ __volatile__("trap 0x0" : "=r"(ret) : "r"(reg0), "r"(reg1)
                       : "memory");
  return ret;
}

static inline uintptr_t sys_call2(unsigned int nbr, uintptr_t parm1,
                                  uintptr_t parm2)
{
  register uintptr_t reg0 __asm__("r6") = (uintptr_t)nbr;
  register uintptr_t reg1 __asm__("r7") = parm1;
  register uintptr_t reg2 __asm__("r8") = parm2;
  register uintptr_t ret __asm__("r10");

  __asm__ __volatile__("trap 0x0" : "=r"(ret)
                       : "r"(reg0), "r"(reg1), "r"(reg2) : "memory");
  return ret;
}

static inline uintptr_t sys_call3(unsigned int nbr, uintptr_t parm1,
                                  uintptr_t parm2, uintptr_t parm3)
{
  register uintptr_t reg0 __asm__("r6") = (uintptr_t)nbr;
  register uintptr_t reg1 __asm__("r7") = parm1;
  register uintptr_t reg2 __asm__("r8") = parm2;
  register uintptr_t reg3 __asm__("r9") = parm3;
  register uintptr_t ret __asm__("r10");

  __asm__ __volatile__("trap 0x0" : "=r"(ret)
                       : "r"(reg0), "r"(reg1), "r"(reg2), "r"(reg3)
                       : "memory");
  return ret;
}

static inline uintptr_t sys_call4(unsigned int nbr, uintptr_t parm1,
                                  uintptr_t parm2, uintptr_t parm3,
                                  uintptr_t parm4)
{
  register uintptr_t reg0 __asm__("r6") = (uintptr_t)nbr;
  register uintptr_t reg1 __asm__("r7") = parm1;
  register uintptr_t reg2 __asm__("r8") = parm2;
  register uintptr_t reg3 __asm__("r9") = parm3;
  register uintptr_t reg4 __asm__("r10") = parm4;

  __asm__ __volatile__("trap 0x0" : "+r"(reg4)
                       : "r"(reg0), "r"(reg1), "r"(reg2), "r"(reg3)
                       : "memory");
  return reg4;
}

static inline uintptr_t sys_call5(unsigned int nbr, uintptr_t parm1,
                                  uintptr_t parm2, uintptr_t parm3,
                                  uintptr_t parm4, uintptr_t parm5)
{
  register uintptr_t reg0 __asm__("r6") = (uintptr_t)nbr;
  register uintptr_t reg1 __asm__("r7") = parm1;
  register uintptr_t reg2 __asm__("r8") = parm2;
  register uintptr_t reg3 __asm__("r9") = parm3;
  register uintptr_t reg4 __asm__("r10") = parm4;
  register uintptr_t reg5 __asm__("r11") = parm5;

  __asm__ __volatile__("trap 0x0" : "+r"(reg4)
                       : "r"(reg0), "r"(reg1), "r"(reg2), "r"(reg3),
                         "r"(reg5)
                       : "memory");
  return reg4;
}

static inline uintptr_t sys_call6(unsigned int nbr, uintptr_t parm1,
                                  uintptr_t parm2, uintptr_t parm3,
                                  uintptr_t parm4, uintptr_t parm5,
                                  uintptr_t parm6)
{
  register uintptr_t reg0 __asm__("r6") = (uintptr_t)nbr;
  register uintptr_t reg1 __asm__("r7") = parm1;
  register uintptr_t reg2 __asm__("r8") = parm2;
  register uintptr_t reg3 __asm__("r9") = parm3;
  register uintptr_t reg4 __asm__("r10") = parm4;
  register uintptr_t reg5 __asm__("r11") = parm5;
  register uintptr_t reg6 __asm__("r12") = parm6;

  __asm__ __volatile__("trap 0x0" : "+r"(reg4)
                       : "r"(reg0), "r"(reg1), "r"(reg2), "r"(reg3),
                         "r"(reg5), "r"(reg6)
                       : "memory");
  return reg4;
}

#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RH850_INCLUDE_SYSCALL_H */
