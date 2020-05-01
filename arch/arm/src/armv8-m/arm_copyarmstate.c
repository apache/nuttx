/****************************************************************************
 * arch/arm/src/armv8-m/arm_copyarmstate.c
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

#include "arm_internal.h"

#if defined(CONFIG_ARCH_FPU) && defined(CONFIG_ARMV8M_LAZYFPU)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_copyarmstate
 *
 * Description:
 *    Copy the ARM portion of the register save area (omitting the floating
 *    point registers) and save the floating pointer register directly.
 *
 ****************************************************************************/

void arm_copyarmstate(uint32_t *dest, uint32_t *src)
{
  int i;

  /* In the Cortex-M3 model, the state is copied from the stack to the TCB,
   * but only a reference is passed to get the state from the TCB.  So the
   * following check avoids copying the TCB save area onto itself:
   */

  if (src != dest)
    {
      /* Save the floating point registers: This will initialize the floating
       * registers at indices SW_INT_REGS through (SW_INT_REGS+SW_FPU_REGS-1)
       */

      arm_savefpu(dest);

      /* Save the block of ARM registers that were saved by the interrupt
       * handling logic.  Indices: 0 through (SW_INT_REGS-1).
       */

      for (i = 0; i < SW_INT_REGS; i++)
        {
          *dest++ = *src++;
        }

      /* Skip over the floating point registers and save the block of ARM
       * registers that were saved by the hardware when the interrupt was
       * taken.  Indices: (SW_INT_REGS+SW_FPU_REGS) through
       * (XCPTCONTEXT_REGS-1)
       */

      src  += SW_FPU_REGS;
      dest += SW_FPU_REGS;

      for (i = 0; i < HW_XCPT_REGS; i++)
        {
          *dest++ = *src++;
        }
    }
}

#endif /* CONFIG_ARCH_FPU && CONFIG_ARMV8M_LAZYFPU */
