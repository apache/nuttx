/****************************************************************************
 * arch/arm/src/armv7-a/arm_copyarmstate.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <arch/irq.h>

#include "arm_internal.h"

#ifdef CONFIG_ARCH_FPU

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

  /* In the Cortex-A model, the state is copied from the stack to the TCB,
   * but only a reference is passed to get the state from the TCB.  So the
   * following check avoids copying the TCB save area onto itself:
   */

  if (src != dest)
    {
      /* Save the floating point registers: This will initialize the floating
       * registers at indices ARM_CONTEXT_REGS through (XCPTCONTEXT_REGS-1)
       */

      arm_savefpu(dest);

      /* Then copy all of the ARM registers (omitting the floating point
       * registers).  Indices: 0 through (ARM_CONTEXT_REGS-1).
       */

      for (i = 0; i < ARM_CONTEXT_REGS; i++)
        {
          *dest++ = *src++;
        }
    }
}

#endif /* CONFIG_ARCH_FPU */
