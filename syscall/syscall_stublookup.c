/****************************************************************************
 * syscall/syscall_stublookup.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <syscall.h>

/* The content of this file is only meaningful during the kernel phase of
 * a kernel build.
 */

#if defined(CONFIG_LIB_SYSCALL)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CAT(p,f)  CAT_(p, f)
#define CAT_(p,f) p##f

#define STUB_NAME(f) CAT(STUB_, f)

#define STUB_PROT0(f) \
  uintptr_t STUB_NAME(f)(int nbr)
#define STUB_PROT1(f) \
  uintptr_t STUB_NAME(f)(int nbr, uintptr_t parm1)
#define STUB_PROT2(f) \
  uintptr_t STUB_NAME(f)(int nbr, uintptr_t parm1, uintptr_t parm2)
#define STUB_PROT3(f) \
  uintptr_t STUB_NAME(f)(int nbr, uintptr_t parm1, uintptr_t parm2, \
                 uintptr_t parm3)
#define STUB_PROT4(f) \
  uintptr_t STUB_NAME(f)(int nbr, uintptr_t parm1, uintptr_t parm2, \
                 uintptr_t parm3, uintptr_t parm4)
#define STUB_PROT5(f) \
  uintptr_t STUB_NAME(f)(int nbr, uintptr_t parm1, uintptr_t parm2, \
                 uintptr_t parm3, uintptr_t parm4, uintptr_t parm5)
#define STUB_PROT6(f) \
  uintptr_t STUB_NAME(f)(int nbr, uintptr_t parm1, uintptr_t parm2, \
                 uintptr_t parm3, uintptr_t parm4, uintptr_t parm5, \
                 uintptr_t parm6)

#define STUB_PROT(f, n) CAT(STUB_PROT, n)(f)

/****************************************************************************
 * Stub Function Prototypes
 ****************************************************************************/

#define SYSCALL_LOOKUP1(f,n) STUB_PROT(f, n);
#define SYSCALL_LOOKUP(f,n)  STUB_PROT(f, n);
#include <sys/syscall_lookup.h>
#undef SYSCALL_LOOKUP1
#undef SYSCALL_LOOKUP

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Stub lookup tables.  This table is indexed by the system call number.
 * Given the system call number, the corresponding entry in this table
 * provides the address of the stub function.
 */

const uintptr_t g_stublookup[SYS_nsyscalls] =
{
#  define SYSCALL_LOOKUP1(f,n) (uintptr_t)STUB_NAME(f)
#  define SYSCALL_LOOKUP(f,n)  , (uintptr_t)STUB_NAME(f)
#  include <sys/syscall_lookup.h>
#  undef SYSCALL_LOOKUP1
#  undef SYSCALL_LOOKUP
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#endif /* CONFIG_LIB_SYSCALL */
