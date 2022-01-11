/****************************************************************************
 * arch/xtensa/include/syscall.h
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

#ifndef __ARCH_XTENSA_INCLUDE_SYSCALL_H
#define __ARCH_XTENSA_INCLUDE_SYSCALL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <debug.h>
#include <arch/irq.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* This is the value used as the argument to the SYSCALL instruction.
 */

#define SYS_syscall 0x00

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

/* Software interrupt with SYS_ call number and no parameters */

static inline uintptr_t sys_call0(unsigned int nbr)
{
  register long reg0 __asm__("a2") = (long)(nbr);

  __asm__ __volatile__
  (
    "movi a3, %1\n"
    "wsr a3, intset\n"
    "isync\n"
    "dsync\n"
    "esync\n"
    "memw\n"
    : "=r"(reg0)
    : "i"(XCHAL_SWINT_CALL), "r"(reg0)
    : "a3", "memory"
  );

  return reg0;
}

/* Software interrupt with SYS_ call number and one parameter */

static inline uintptr_t sys_call1(unsigned int nbr, uintptr_t parm1)
{
  register long reg0 __asm__("a2") = (long)(nbr);
  register long reg1 __asm__("a3") = (long)(parm1);

  __asm__ __volatile__
  (
    "movi a4, %1\n"
    "wsr a4, intset\n"
    "isync\n"
    "dsync\n"
    "esync\n"
    "memw\n"
    : "=r"(reg0)
    : "i"(XCHAL_SWINT_CALL), "r"(reg0), "r"(reg1)
    : "a4", "memory"
  );

  return reg0;
}

/* Software interrupt with SYS_ call number and two parameters */

static inline uintptr_t sys_call2(unsigned int nbr, uintptr_t parm1,
                                  uintptr_t parm2)
{
  register long reg0 __asm__("a2") = (long)(nbr);
  register long reg2 __asm__("a4") = (long)(parm2);
  register long reg1 __asm__("a3") = (long)(parm1);

  __asm__ __volatile__
  (
    "movi a5, %1\n"
    "wsr a5, intset\n"
    "isync\n"
    "dsync\n"
    "esync\n"
    "memw\n"
    : "=r"(reg0)
    : "i"(XCHAL_SWINT_CALL), "r"(reg0), "r"(reg1), "r"(reg2)
    : "a5", "memory"
  );

  return reg0;
}

/* Software interrupt with SYS_ call number and three parameters */

static inline uintptr_t sys_call3(unsigned int nbr, uintptr_t parm1,
                                  uintptr_t parm2, uintptr_t parm3)
{
  register long reg0 __asm__("a2") = (long)(nbr);
  register long reg3 __asm__("a5") = (long)(parm3);
  register long reg2 __asm__("a4") = (long)(parm2);
  register long reg1 __asm__("a3") = (long)(parm1);

  __asm__ __volatile__
  (
    "movi a6, %1\n"
    "wsr a6, intset\n"
    "isync\n"
    "dsync\n"
    "esync\n"
    "memw\n"
    : "=r"(reg0)
    : "i"(XCHAL_SWINT_CALL), "r"(reg0), "r"(reg1), "r"(reg2),
      "r"(reg3)
    : "a6", "memory"
  );

  return reg0;
}

/* Software interrupt with SYS_ call number and four parameters.
 *
 */

static inline uintptr_t sys_call4(unsigned int nbr, uintptr_t parm1,
                                  uintptr_t parm2, uintptr_t parm3,
                                  uintptr_t parm4)
{
  register long reg0 __asm__("a2") = (long)(nbr);
  register long reg4 __asm__("a6") = (long)(parm4);
  register long reg3 __asm__("a5") = (long)(parm3);
  register long reg2 __asm__("a4") = (long)(parm2);
  register long reg1 __asm__("a3") = (long)(parm1);

  __asm__ __volatile__
  (
    "movi a7, %1\n"
    "wsr a7, intset\n"
    "isync\n"
    "dsync\n"
    "esync\n"
    "memw\n"
    : "=r"(reg0)
    : "i"(XCHAL_SWINT_CALL), "r"(reg0), "r"(reg1), "r"(reg2),
      "r"(reg3), "r"(reg4)
    : "a7", "memory"
  );

  return reg0;
}

/* Software interrupt with SYS_ call number and five parameters.
 *
 */

static inline uintptr_t sys_call5(unsigned int nbr, uintptr_t parm1,
                                  uintptr_t parm2, uintptr_t parm3,
                                  uintptr_t parm4, uintptr_t parm5)
{
  register long reg0 __asm__("a2") = (long)(nbr);
  register long reg5 __asm__("a7") = (long)(parm4);
  register long reg4 __asm__("a6") = (long)(parm4);
  register long reg3 __asm__("a5") = (long)(parm3);
  register long reg2 __asm__("a4") = (long)(parm2);
  register long reg1 __asm__("a3") = (long)(parm1);

  __asm__ __volatile__
  (
    "movi a8, %1\n"
    "wsr a8, intset\n"
    "isync\n"
    "dsync\n"
    "esync\n"
    "memw\n"
    : "=r"(reg0)
    : "i"(XCHAL_SWINT_CALL), "r"(reg0), "r"(reg1), "r"(reg2),
      "r"(reg3), "r"(reg4), "r"(reg5)
    : "a8", "memory"
  );

  return reg0;
}

/* Software interrupt with SYS_ call number and six parameters.
 *
 */

static inline uintptr_t sys_call6(unsigned int nbr, uintptr_t parm1,
                                  uintptr_t parm2, uintptr_t parm3,
                                  uintptr_t parm4, uintptr_t parm5,
                                  uintptr_t parm6)
{
  register long reg0 __asm__("a2") = (long)(nbr);
  register long reg6 __asm__("a8") = (long)(parm4);
  register long reg5 __asm__("a7") = (long)(parm4);
  register long reg4 __asm__("a6") = (long)(parm4);
  register long reg3 __asm__("a5") = (long)(parm3);
  register long reg2 __asm__("a4") = (long)(parm2);
  register long reg1 __asm__("a3") = (long)(parm1);

  __asm__ __volatile__
  (
    "movi a9, %1\n"
    "wsr a9, intset\n"
    "isync\n"
    "dsync\n"
    "esync\n"
    "memw\n"
    : "=r"(reg0)
    : "i"(XCHAL_SWINT_CALL), "r"(reg0), "r"(reg1), "r"(reg2),
      "r"(reg3), "r"(reg4), "r"(reg5)
    : "a9", "memory"
  );

  return reg0;
}

#endif

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

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_XTENSA_INCLUDE_SYSCALL_H */
