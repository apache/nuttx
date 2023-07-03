/****************************************************************************
 * arch/risc-v/src/nuttsbi/sbi_mscratch.c
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
#include <nuttx/irq.h>

#include <arch/barriers.h>

#include <stdint.h>

#include "riscv_internal.h"

#include "sbi_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define STACK_SIZE  (STACK_ALIGN_DOWN(MMODE_IRQSTACK))

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern uintptr_t g_mintstacktop;
extern uintptr_t g_mintstackalloc;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sbi_mscratch_assign
 *
 * Description:
 *   Assign the mscratch register for hartid. Sets the M-mode interrupt stack
 *   which is a must because M-mode deals with flat addressing and cannot
 *   share the user stack for exception handling.
 *
 * Input Parameters:
 *   hartid - Hartid
 *
 ****************************************************************************/

void sbi_mscratch_assign(uintptr_t hartid)
{
  uintptr_t stack_top;
#if MMODE_HART_CNT > 1
  /* Calculate offset first (stack size * hartid) */

  /* REVISIT: This requires that hartID is sequential, and starts from 0 */

  stack_top = (uintptr_t)&g_mintstacktop - hartid * STACK_SIZE;
#else
  UNUSED(hartid);
  stack_top = (uintptr_t)&g_mintstacktop;
#endif

  WRITE_CSR(mscratch, stack_top);

  /* Make sure mscratch is updated before continuing */

  __MB();
}
