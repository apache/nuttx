/****************************************************************************
 * arch/arm64/include/ucontext.h
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

#ifndef __ARCH_ARM64_INCLUDE_UCONTEXT_H
#define __ARCH_ARM64_INCLUDE_UCONTEXT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The machine context of getcontext() and makecontext().  The assembly code
 * in libs/libc/machine/arm64/arch_ucontext.S knows this layout.
 */

typedef struct mcontext_s
{
  uint64_t regs[31];   /* x0-x30 */
  uint64_t sp;
  uint64_t pc;
#ifdef CONFIG_ARCH_FPU
  uint64_t fpregs[8];  /* d8-d15, the callee-saved FP registers */
#endif
} mcontext_t;

#endif /* __ARCH_ARM64_INCLUDE_UCONTEXT_H */
