/****************************************************************************
 * arch/x86/src/common/x86_fork.h
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

#ifndef __ARCH_X86_SRC_COMMON_X86_FORK_H
#define __ARCH_X86_SRC_COMMON_X86_FORK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdbool.h>

#ifndef __ASSEMBLY__
#  include <sys/types.h>
#  include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FORK_SIZEOF (sizeof(struct fork_s))

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* The caller snapshot fork.S builds on the caller's own stack, in the order
 * the pushes leave it in memory:  lowest address first.  Only the registers
 * the i386 System V ABI makes callee-saved are here -- EBX, ESI, EDI and EBP
 * -- plus what is needed to resume the caller:  its stack pointer, its
 * segment selectors, its flags and the address it would have returned to.
 *
 * EAX, ECX and EDX are absent deliberately.  They are caller-saved, so the
 * caller has no expectation of them surviving the call, and the child gets
 * EAX = 0 as its return value.
 */

struct fork_s
{
  uint32_t edi;    /* 0  */
  uint32_t esi;    /* 4  */
  uint32_t ebx;    /* 8  */
  uint32_t ebp;    /* 12 */
  uint32_t esp;    /* 16 */
  uint32_t ds;     /* 20 */
  uint32_t cs;     /* 24 */
  uint32_t ss;     /* 28 */
  uint32_t eflags; /* 32 */
  uint32_t eip;    /* 36 */

  /* The x87/FPU state is not saved.  This architecture does not save it on a
   * context switch either -- see i486_savestate.c -- so there is nothing for
   * the child to inherit that the parent itself preserves across a call.
   */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* The C half of the primitives, called from fork.S with the snapshot above
 * and the vfork flag.
 */

pid_t x86_fork(const struct fork_s *context, bool vfork);

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_X86_SRC_COMMON_X86_FORK_H */
