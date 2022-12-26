/****************************************************************************
 * arch/mips/include/syscall.h
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

#ifndef __ARCH_MIPS_INCLUDE_SYSCALL_H
#define __ARCH_MIPS_INCLUDE_SYSCALL_H

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

#define SYS_syscall 0x00

/* Configuration ************************************************************/

/* SYS call 1 and 2 are defined for internal use by the PIC32MX port (see
 * arch/mips/include/mips32/syscall.h).  In addition, SYS call 3 is the
 * return from a SYS call in kernel mode.  The first four syscall values
 * must, therefore, be reserved (0 is not used).
 */

#ifndef CONFIG_BUILD_FLAT
#  define CONFIG_SYS_RESERVED 4
#else
#  define CONFIG_SYS_RESERVED 3
#endif

/* sys_call macros **********************************************************/

/* System calls with 3 parameters and fewer are handled by sys_call0
 * (sys_call1, sys_call2, and sys_call3 are aliases for sys_call0).
 * This is because the parameters are passed in a0-a3.  a0 is reserved for
 * the syscall number leaving up to three additional parameters that can be
 * passed in registers.  The remainder would have to be pushed onto the
 * stack.
 *
 * Instead, these macros are provided which handle parameters four, five and
 * six in a non-standard way:  The use s0 ($7), s1 ($8), and s2 ($9) to pass
 * the additional parameters.
 */

#ifndef __ASSEMBLY__

/* System call SYS_ argument and four additional parameters. */

#define sys_call4(nbr,parm1,parm2,parm3,parm4) __extension__({ \
  uintptr_t __result; \
  __asm__ __volatile__ (\
    "\tmove	$4, %0\n" \
    "\tmove	$5, %1\n" \
    "\tmove	$6, %2\n" \
    "\tmove	$7, %3\n" \
    "\tmove	$8, %4\n" \
    "\la	$12, sys_call3\n" \
    "\jalr	$12, $31\n" \
    "\tmove	%5, $r2\n" \
    : "=r" (nbr) "=r" (parm1) "=r" (parm2) "=r" (parm3) "=r" (parm4) \
    : " "r"(__result)\
    : "memory"\
  ); \
  __result; \
})

/* System call SYS_ argument and five additional parameters. */

#define sys_call5(nbr,parm1,parm2,parm3,parm4,parm5) __extension__({ \
  uintptr_t __result; \
  __asm__ __volatile__ (\
    "\tmove	$4, %0\n" \
    "\tmove	$5, %1\n" \
    "\tmove	$6, %2\n" \
    "\tmove	$7, %3\n" \
    "\tmove	$8, %4\n" \
    "\tmove	$9, %5\n" \
    "\la	$12, sys_call3\n" \
    "\jalr	$12, $31\n" \
    "\tmove	%6, $r2\n" \
    : "=r" (nbr) "=r" (parm1) "=r" (parm2) "=r" (parm3) "=r" (parm4) "=r" (parm5) \
    : " "r"(__result)\
    : "memory"\
  ); \
  __result; \
})

/* System call SYS_ argument and six additional parameters. */

#define sys_call6(nbr,parm1,parm2,parm3,parm4,parm5,parm6) __extension__({ \
  uintptr_t __result; \
  __asm__ __volatile__ (\
    "\tmove	$4, %0\n" \
    "\tmove	$5, %1\n" \
    "\tmove	$6, %2\n" \
    "\tmove	$7, %3\n" \
    "\tmove	$8, %4\n" \
    "\tmove	$9, %5\n" \
    "\tmove	$10, %5\n" \
    "\la	$12, sys_call3\n" \
    "\jalr	$12, $31\n" \
    "\tmove	%6, $r2\n" \
    : "=r" (nbr) "=r" (parm1) "=r" (parm2) "=r" (parm3) "=r" (parm4) "=r" (parm5) \
    : " "r"(__result)\
    : "memory"\
  ); \
  __result; \
})

/* Context switching system calls *******************************************/

/* SYS call 0:
 *
 * int up_saveusercontext(void *saveregs);
 */

#define SYS_save_context (0)

/* SYS call 1:
 *
 * void up_fullcontextrestore(uint32_t *restoreregs) noreturn_function;
 */

#define SYS_restore_context (1)
#define up_fullcontextrestore(restoreregs) \
  sys_call1(SYS_restore_context, (uintptr_t)restoreregs)

/* SYS call 2:
 *
 * void mips_switchcontext(uint32_t *saveregs, uint32_t *restoreregs);
 */

#define SYS_switch_context (2)
#define mips_switchcontext(saveregs, restoreregs) \
  sys_call2(SYS_switch_context, (uintptr_t)saveregs, (uintptr_t)restoreregs)

#ifdef CONFIG_BUILD_KERNEL
/* SYS call 3:
 *
 * void up_syscall_return(void);
 */

#define SYS_syscall_return (3)
#define up_syscall_return() sys_call0(SYS_syscall_return)

#endif
#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

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

/****************************************************************************
 * Name: up_syscall0
 *
 * Description:
 *   System call SYS_ argument and no additional parameters.
 *
 ****************************************************************************/

uintptr_t sys_call0(unsigned int nbr);

/****************************************************************************
 * Name: up_syscall1
 *
 * Description:
 *   System call SYS_ argument and one additional parameter.
 *
 ****************************************************************************/

uintptr_t sys_call1(unsigned int nbr, uintptr_t parm1);

/****************************************************************************
 * Name: up_syscall2
 *
 * Description:
 *   System call SYS_ argument and two additional parameters.
 *
 ****************************************************************************/

uintptr_t sys_call2(unsigned int nbr, uintptr_t parm1, uintptr_t parm2);

/****************************************************************************
 * Name: up_syscall3
 *
 * Description:
 *   System call SYS_ argument and three additional parameters.
 *
 ****************************************************************************/

uintptr_t sys_call3(unsigned int nbr, uintptr_t parm1, uintptr_t parm2,
                    uintptr_t parm3);

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_MIPS_INCLUDE_SYSCALL_H */
