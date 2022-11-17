/****************************************************************************
 * arch/x86/src/i486/i486_savestate.c
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

#include <assert.h>
#include <debug.h>

#include <arch/arch.h>
#include <arch/irq.h>

#include "x86_internal.h"

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
 * Name: x86_savestate
 *
 * Description:
 *   This function saves the interrupt level context information in the
 *   TCB.  This would just be a x86_copystate but we have to handle one
 *   special case.  In the case where the privilege level changes, the
 *   value of sp and ss will not be saved on stack by the interrupt handler.
 *   So, in that case, we will have to fudge those values here.
 *
 ****************************************************************************/

void x86_savestate(uint32_t *regs)
{
  uint8_t cpl;
  uint8_t rpl;

  /* First, just copy all of the registers */

  x86_copystate(regs, (uint32_t *)g_current_regs);

  /* The RES_SP and REG_SS values will not be saved by the interrupt handling
   * logic if there is no change in privilege level.  In that case, we will
   * have to "fudge" those values here.  For now, just overwrite the REG_SP
   * and REG_SS values with what we believe to be correct.  Obviously, this
   * will have to change in the future to support multi-segment operation.
   *
   * Check for a change in privilege level.
   */

  rpl = regs[REG_CS] & 3;
  cpl = up_getcs() & 3;
  DEBUGASSERT(rpl >= cpl);

  if (rpl == cpl)
    {
      /* No priority change, SP and SS are not present in the stack frame.
       *
       * The value saved in the REG_ESP will be the stackpointer value prior
       * to the execution of the PUSHA.  It will point at REG_IRQNO.
       */

      regs[REG_SP] = g_current_regs[REG_ESP] + 4*BOTTOM_NOPRIO;
      regs[REG_SS] = up_getss();
    }
  else
    {
      DEBUGASSERT(regs[REG_SP] == g_current_regs[REG_ESP] + 4*BOTTOM_PRIO);
    }
}
