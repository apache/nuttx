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
 * If CONFIG_BUILD_FLAT isn't defined, then four more syscall values must
 * be reserved.
 */

#ifndef CONFIG_BUILD_FLAT
#  define CONFIG_SYS_RESERVED 8
#else
#  define CONFIG_SYS_RESERVED 4
#endif

/* Cortex-M system calls ****************************************************/

/* SYS call 1:
 *
 * void arm_fullcontextrestore(uint32_t *restoreregs) noreturn_function;
 */

#define SYS_restore_context       (1)

/* SYS call 2:
 *
 * void arm_switchcontext(uint32_t **saveregs, uint32_t *restoreregs);
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

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* SVC with SYS_ call number and no parameters */

#ifdef __ghs__
#  define sys_call0(nbr)                    \
({                                          \
  uintptr_t ret;                            \
  __asm__ __volatile__                      \
  (                                         \
    "mov r0, %2\n\t"                        \
    "svc %1\n\t"                            \
    "mov %0, r0"                            \
    : "=r"(ret)                             \
    : "i"(SYS_syscall), "r"((uintptr_t)nbr) \
    : "memory", "r0"                        \
  );                                        \
  ret;                                      \
})
#else
#  define sys_call0(nbr)                                    \
({                                                          \
  register uintptr_t reg0 __asm__("r0") = (uintptr_t)(nbr); \
  __asm__ __volatile__                                      \
  (                                                         \
    "svc %1"                                                \
    : "=r"(reg0)                                            \
    : "i"(SYS_syscall), "r"(reg0)                           \
    : "memory"                                              \
  );                                                        \
  reg0;                                                     \
})
#endif

/* SVC with SYS_ call number and one parameter */

#ifdef __ghs__
#  define sys_call1(nbr, parm1)              \
({                                           \
  uintptr_t ret;                             \
  __asm__ __volatile__                       \
  (                                          \
    "mov r0, %2\n\t"                         \
    "mov r1, %3\n\t"                         \
    "svc %1\n\t"                             \
    "mov %0, r0"                             \
    : "=r"(ret)                              \
    : "i"(SYS_syscall), "r"((uintptr_t)nbr), \
      "r"((uintptr_t)parm1)                  \
    : "memory", "r0", "r1"                   \
  );                                         \
  ret;                                       \
})
#else
#  define sys_call1(nbr, parm1)                               \
({                                                            \
  register uintptr_t reg1 __asm__("r1") = (uintptr_t)(parm1); \
  register uintptr_t reg0 __asm__("r0") = (uintptr_t)(nbr);   \
  __asm__ __volatile__                                        \
  (                                                           \
    "svc %1"                                                  \
    : "=r"(reg0)                                              \
    : "i"(SYS_syscall), "r"(reg0), "r"(reg1)                  \
    : "memory"                                                \
  );                                                          \
  reg0;                                                       \
})
#endif

/* SVC with SYS_ call number and two parameters */

#ifdef __ghs__
#  define sys_call2(nbr, parm1, parm2)             \
({                                                 \
  uintptr_t ret;                                   \
  __asm__ __volatile__                             \
  (                                                \
    "mov r0, %2\n\t"                               \
    "mov r1, %3\n\t"                               \
    "mov r2, %4\n\t"                               \
    "svc %1\n\t"                                   \
    "mov %0, r0"                                   \
    : "=r"(ret)                                    \
    : "i"(SYS_syscall), "r"((uintptr_t)nbr),       \
      "r"((uintptr_t)parm1), "r"((uintptr_t)parm2) \
    : "memory", "r0", "r1", "r2"                   \
  );                                               \
  ret;                                             \
})
#else
#  define sys_call2(nbr, parm1, parm2)                        \
({                                                            \
  register uintptr_t reg2 __asm__("r2") = (uintptr_t)(parm2); \
  register uintptr_t reg1 __asm__("r1") = (uintptr_t)(parm1); \
  register uintptr_t reg0 __asm__("r0") = (uintptr_t)(nbr);   \
  __asm__ __volatile__                                        \
  (                                                           \
    "svc %1"                                                  \
    : "=r"(reg0)                                              \
    : "i"(SYS_syscall), "r"(reg0), "r"(reg1), "r"(reg2)       \
    : "memory"                                                \
  );                                                          \
  reg0;                                                       \
})
#endif

/* SVC with SYS_ call number and three parameters */

#ifdef __ghs__
#  define sys_call3(nbr, parm1, parm2, parm3)                       \
({                                                                  \
  uintptr_t ret;                                                    \
  __asm__ __volatile__                                              \
  (                                                                 \
    "mov r0, %2\n\t"                                                \
    "mov r1, %3\n\t"                                                \
    "mov r2, %4\n\t"                                                \
    "mov r3, %5\n\t"                                                \
    "svc %1\n\t"                                                    \
    "mov %0, r0"                                                    \
    : "=r"(ret)                                                     \
    : "i"(SYS_syscall), "r"((uintptr_t)nbr), "r"((uintptr_t)parm1), \
      "r"((uintptr_t)parm2), "r"((uintptr_t)parm3)                  \
    : "memory", "r0", "r1", "r2", "r3"                              \
  );                                                                \
  ret;                                                              \
})
#else
#  define sys_call3(nbr, parm1, parm2, parm3)                      \
({                                                                 \
  register uintptr_t reg3 __asm__("r3") = (uintptr_t)(parm3);      \
  register uintptr_t reg2 __asm__("r2") = (uintptr_t)(parm2);      \
  register uintptr_t reg1 __asm__("r1") = (uintptr_t)(parm1);      \
  register uintptr_t reg0 __asm__("r0") = (uintptr_t)(nbr);        \
  __asm__ __volatile__                                             \
  (                                                                \
    "svc %1"                                                       \
    : "=r"(reg0)                                                   \
    : "i"(SYS_syscall), "r"(reg0), "r"(reg1), "r"(reg2), "r"(reg3) \
    : "memory"                                                     \
  );                                                               \
  reg0;                                                            \
})
#endif

/* SVC with SYS_ call number and four parameters */

#ifdef __ghs__
#  define sys_call4(nbr, parm1, parm2, parm3, parm4)                      \
({                                                                        \
  uintptr_t ret;                                                          \
  __asm__ __volatile__                                                    \
  (                                                                       \
    "push {r4}\n\t"                                                       \
    "mov r0, %2\n\t"                                                      \
    "mov r1, %3\n\t"                                                      \
    "mov r2, %4\n\t"                                                      \
    "mov r3, %5\n\t"                                                      \
    "mov r4, %6\n\t"                                                      \
    "svc %1\n\t"                                                          \
    "pop {r4}\n\t"                                                        \
    "mov %0, r0"                                                          \
    : "=r"(ret)                                                           \
    : "i"(SYS_syscall), "r"((uintptr_t)nbr), "r"((uintptr_t)parm1),       \
      "r"((uintptr_t)parm2), "r"((uintptr_t)parm3), "r"((uintptr_t)parm4) \
    : "memory", "r0", "r1", "r2", "r3"                                    \
  );                                                                      \
  ret;                                                                    \
})
#else
#  define sys_call4(nbr, parm1, parm2, parm3, parm4)          \
({                                                            \
  register uintptr_t reg4 __asm__("r4") = (uintptr_t)(parm4); \
  register uintptr_t reg3 __asm__("r3") = (uintptr_t)(parm3); \
  register uintptr_t reg2 __asm__("r2") = (uintptr_t)(parm2); \
  register uintptr_t reg1 __asm__("r1") = (uintptr_t)(parm1); \
  register uintptr_t reg0 __asm__("r0") = (uintptr_t)(nbr);   \
  __asm__ __volatile__                                        \
  (                                                           \
    "svc %1"                                                  \
    : "=r"(reg0)                                              \
    : "i"(SYS_syscall), "r"(reg0), "r"(reg1), "r"(reg2),      \
      "r"(reg3), "r"(reg4)                                    \
    : "memory"                                                \
  );                                                          \
  reg0;                                                       \
})
#endif

/* SVC with SYS_ call number and five parameters */

#ifdef __ghs__
#  define sys_call5(nbr, parm1, parm2, parm3, parm4, parm5)                \
({                                                                         \
  uintptr_t ret;                                                           \
  __asm__ __volatile__                                                     \
  (                                                                        \
    "push {r3-r5}\n\t"                                                     \
    "push {%7}\n\t"                                                        \
    "push {%6}\n\t"                                                        \
    "mov r0, %2\n\t"                                                       \
    "mov r1, %3\n\t"                                                       \
    "mov r2, %4\n\t"                                                       \
    "mov r3, %5\n\t"                                                       \
    "pop {r4-r5}\n\t"                                                      \
    "svc %1\n\t"                                                           \
    "pop {r3-r5}\n\t"                                                      \
    "mov %0, r0"                                                           \
    : "=r"(ret)                                                            \
    : "i"(SYS_syscall), "r"((uintptr_t)nbr), "r"((uintptr_t)parm1),        \
      "r"((uintptr_t)parm2), "r"((uintptr_t)parm3), "r"((uintptr_t)parm4), \
      "r"((uintptr_t)parm5)                                                \
    : "memory", "r0", "r1", "r2"                                           \
  );                                                                       \
  ret;                                                                     \
})
#else
#  define sys_call5(nbr, parm1, parm2, parm3, parm4, parm5)   \
({                                                            \
  register uintptr_t reg5 __asm__("r5") = (uintptr_t)(parm5); \
  register uintptr_t reg4 __asm__("r4") = (uintptr_t)(parm4); \
  register uintptr_t reg3 __asm__("r3") = (uintptr_t)(parm3); \
  register uintptr_t reg2 __asm__("r2") = (uintptr_t)(parm2); \
  register uintptr_t reg1 __asm__("r1") = (uintptr_t)(parm1); \
  register uintptr_t reg0 __asm__("r0") = (uintptr_t)(nbr);   \
  __asm__ __volatile__                                        \
  (                                                           \
    "svc %1"                                                  \
    : "=r"(reg0)                                              \
    : "i"(SYS_syscall), "r"(reg0), "r"(reg1), "r"(reg2),      \
      "r"(reg3), "r"(reg4), "r"(reg5)                         \
    : "memory"                                                \
  );                                                          \
  reg0;                                                       \
})
#endif

/* SVC with SYS_ call number and six parameters */

#ifdef __ghs__
#  define sys_call6(nbr, parm1, parm2, parm3, parm4, parm5, parm6)         \
({                                                                         \
  uintptr_t ret;                                                           \
  __asm__ __volatile__                                                     \
  (                                                                        \
    "push {r2-r6}\n\t"                                                     \
    "push {%8}\n\t"                                                        \
    "push {%7}\n\t"                                                        \
    "push {%6}\n\t"                                                        \
    "push {%5}\n\t"                                                        \
    "mov r0, %2\n\t"                                                       \
    "mov r1, %3\n\t"                                                       \
    "mov r2, %4\n\t"                                                       \
    "pop {r3-r6}\n\t"                                                      \
    "svc %1\n\t"                                                           \
    "pop {r2-r6}\n\t"                                                      \
    "mov %0, r0"                                                           \
    : "=r"(ret)                                                            \
    : "i"(SYS_syscall), "r"((uintptr_t)nbr), "r"((uintptr_t)parm1),        \
      "r"((uintptr_t)parm2), "r"((uintptr_t)parm3), "r"((uintptr_t)parm4), \
      "r"((uintptr_t)parm5), "r"((uintptr_t)parm6)                         \
    : "memory", "r0", "r1"                                                 \
  );                                                                       \
  ret;                                                                     \
})
#else
#  define sys_call6(nbr, parm1, parm2, parm3, parm4, parm5, parm6) \
({                                                                 \
  register uintptr_t reg6 __asm__("r6") = (uintptr_t)(parm6);      \
  register uintptr_t reg5 __asm__("r5") = (uintptr_t)(parm5);      \
  register uintptr_t reg4 __asm__("r4") = (uintptr_t)(parm4);      \
  register uintptr_t reg3 __asm__("r3") = (uintptr_t)(parm3);      \
  register uintptr_t reg2 __asm__("r2") = (uintptr_t)(parm2);      \
  register uintptr_t reg1 __asm__("r1") = (uintptr_t)(parm1);      \
  register uintptr_t reg0 __asm__("r0") = (uintptr_t)(nbr);        \
  __asm__ __volatile__                                             \
  (                                                                \
    "svc %1"                                                       \
    : "=r"(reg0)                                                   \
    : "i"(SYS_syscall), "r"(reg0), "r"(reg1), "r"(reg2),           \
      "r"(reg3), "r"(reg4), "r"(reg5), "r"(reg6)                   \
    : "memory"                                                     \
  );                                                               \
  reg0;                                                            \
})
#endif

/* semihosting(SMH) call with call number and one parameter */

#ifdef __ghs__
#  if defined(CONFIG_ARCH_ARMV6M) || \
      defined(CONFIG_ARCH_ARMV7M) || \
      defined(CONFIG_ARCH_ARMV8M)

#    define smh_call(nbr, parm1)                                   \
({                                                                 \
  uintptr_t ret;                                                   \
  __asm__ __volatile__                                             \
  (                                                                \
    "mov r0, %2\n\t"                                               \
    "mov r1, %3\n\t"                                               \
    "bkpt %1\n\t"                                                  \
    "mov %0, r0"                                                   \
    : "=r"(ret)                                                    \
    : "i"(SYS_smhcall), "r"((uintptr_t)nbr), "r"((uintptr_t)parm1) \
    : "memory", "r0", "r1"                                         \
  );                                                               \
  ret;                                                             \
})
#  else
#    define smh_call(nbr, parm1)                                   \
({                                                                 \
  uintptr_t ret;                                                   \
  __asm__ __volatile__                                             \
  (                                                                \
    "mov r0, %2\n\t"                                               \
    "mov r1, %3\n\t"                                               \
    "svc %1\n\t"                                                   \
    "mov %0, r0"                                                   \
    : "=r"(ret)                                                    \
    : "i"(SYS_smhcall), "r"((uintptr_t)nbr), "r"((uintptr_t)parm1) \
    : "memory", "r0", "r1"                                         \
  );                                                               \
  ret;                                                             \
})
#  endif
#else
#  if defined(CONFIG_ARCH_ARMV6M) || \
      defined(CONFIG_ARCH_ARMV7M) || \
      defined(CONFIG_ARCH_ARMV8M)

#    define smh_call(nbr, parm1)                              \
({                                                            \
  register uintptr_t reg1 __asm__("r1") = (uintptr_t)(parm1); \
  register uintptr_t reg0 __asm__("r0") = (uintptr_t)(nbr);   \
  __asm__ __volatile__                                        \
  (                                                           \
    "bkpt %1"                                                 \
    : "=r"(reg0)                                              \
    : "i"(SYS_smhcall), "r"(reg0), "r"(reg1)                  \
    : "memory"                                                \
  );                                                          \
  reg0;                                                       \
})
#  else
#    define smh_call(nbr, parm1)                              \
({                                                            \
  register uintptr_t reg1 __asm__("r1") = (uintptr_t)(parm1); \
  register uintptr_t reg0 __asm__("r0") = (uintptr_t)(nbr);   \
  __asm__ __volatile__                                        \
  (                                                           \
    "svc %1"                                                  \
    : "=r"(reg0)                                              \
    : "i"(SYS_smhcall), "r"(reg0), "r"(reg1)                  \
    : "memory"                                                \
  );                                                          \
  reg0;                                                       \
})
#  endif
#endif

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
