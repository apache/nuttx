/****************************************************************************
 * arch/risc-v/src/rv64gc/riscv_copystate.c
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

#include <stdint.h>
#include <arch/irq.h>

#include "riscv_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: riscv_copystate
 ****************************************************************************/

/* A little faster than most memcpy's */

void riscv_copystate(uint64_t *dest, uint64_t *src)
{
  int i;

#ifdef CONFIG_ARCH_FPU
  uint64_t *regs = dest;
#endif

  /* In the RISC-V model, the state is copied from the stack to the TCB,
   * but only a reference is passed to get the state from the TCB.  So the
   * following check avoids copying the TCB save area onto itself:
   */

  if (src != dest)
    {
      /* save integer registers first */

      for (i = 0; i < INT_XCPT_REGS; i++)
        {
          *dest++ = *src++;
        }

      /* Save the floating point registers: This will initialize the floating
       * registers at indices INT_XCPT_REGS through (XCPTCONTEXT_REGS-1).
       * Do this after saving REG_INT_CTX with the ORIGINAL context pointer.
       */

#ifdef CONFIG_ARCH_FPU
      riscv_savefpu(regs);
#endif
    }
}
