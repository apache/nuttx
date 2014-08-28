/****************************************************************************
 * arch/arm/include/armv6-m/syscall.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

/* This file should never be included directed but, rather, only indirectly
 * through include/syscall.h or include/sys/sycall.h
 */

#ifndef __ARCH_ARM_INCLUDE_ARMV6_M_SYSCALL_H
#define __ARCH_ARM_INCLUDE_ARMV6_M_SYSCALL_H

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
    : "memory"
  );

  return reg0;
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
    : "memory"
  );

  return reg0;
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

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_INCLUDE_ARMV6_M_SYSCALL_H */

