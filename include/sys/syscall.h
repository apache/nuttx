/****************************************************************************
 * include/sys/syscall.h
 * This file contains the system call numbers.
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

#ifndef __INCLUDE_SYS_SYSCALL_H
#define __INCLUDE_SYS_SYSCALL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

#include <arch/syscall.h>

#ifdef CONFIG_LIB_SYSCALL

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Note that the reported number of system calls does *NOT* include the
 * architecture-specific system calls.  If the "real" total is required,
 * use SYS_maxsyscall.
 */

#define SYS_nsyscalls                  (SYS_maxsyscall - CONFIG_SYS_RESERVED)

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

enum
{
#  define SYSCALL_LOOKUP_(f)   SYS_##f
#  define SYSCALL_LOOKUP1(f,n) SYSCALL_LOOKUP_(f) = CONFIG_SYS_RESERVED
#  define SYSCALL_LOOKUP(f,n)  , SYSCALL_LOOKUP_(f)
#  include "syscall_lookup.h"
  , SYS_maxsyscall
#  undef SYSCALL_LOOKUP_
#  undef SYSCALL_LOOKUP1
#  undef SYSCALL_LOOKUP
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#ifdef CONFIG_LIB_SYSCALL

/* Given the system call number, the corresponding entry in this table
 * provides the address of the stub function.
 *
 * This table is only available during the kernel phase of a kernel build.
 */

EXTERN const uintptr_t g_stublookup[SYS_nsyscalls];

#endif

/* Given the system call number, the corresponding entry in this table
 * provides the name of the function.
 */

EXTERN const char *g_funcnames[SYS_nsyscalls];

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_LIB_SYSCALL */
#endif /* __INCLUDE_SYS_SYSCALL_H */
