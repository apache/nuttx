/****************************************************************************
 * arch/arm/src/armv8-m/arm_gen_nonsecfault.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/syslog/syslog.h>

#include <stdint.h>
#include <arch/irq.h>

#include "nvic.h"
#include "sau.h"
#include "arm_internal.h"
#include "exc_return.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define OFFSET_R0              (0 * 4) /* R0 */
#define OFFSET_R1              (1 * 4) /* R1 */
#define OFFSET_R2              (2 * 4) /* R2 */
#define OFFSET_R3              (3 * 4) /* R3 */
#define OFFSET_R12             (4 * 4) /* R12 */
#define OFFSET_R14             (5 * 4) /* R14 = LR */
#define OFFSET_R15             (6 * 4) /* R15 = PC */
#define OFFSET_XPSR            (7 * 4) /* xPSR */

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint32_t g_psp_ns;
static uint32_t g_msp_ns;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_should_gen_nonsecurefault
 *
 * Description:
 *   Check whether should generate non-secure IRQ from securefault
 *
 ****************************************************************************/

bool weak_function arm_should_gen_nonsecurefault(void)
{
  return true;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_gen_nonsecurefault
 *
 * Description:
 *   For TEE & REE, securefault & busfault are not banked, so the faults can
 *   only forword to TEE/REE.
 *   But how to crash dump the other core which not handled faults ?
 *
 *   Here we provide a way to resolve this problem:
 *   1. Set the securefault & busfault to TEE
 *   2. busfault happend from TEE, then directly dump TEE
 *   3. busfault happend from REE, then generate nonsecurefault
 *   4. Back to REE, and dump
 *
 * Return values:
 *   1 means generated done
 *   0 means don't need generated
 *
 ****************************************************************************/

int arm_gen_nonsecurefault(int irq, uint32_t *regs)
{
  uint32_t sp_ns;

  if (!arm_should_gen_nonsecurefault())
    {
      return 0;
    }

  /* Check whether come from REE */

  if (regs[REG_EXC_RETURN] & EXC_RETURN_SECURE_STACK)
    {
      return 0;
    }

  if (getreg32(SAU_SFSR) == 0)
    {
      /* busfault are forward to REE ? */

      if (getreg32(NVIC_AIRCR) & NVIC_AIRCR_BFHFNMINS)
        {
          return 0;
        }

      /* Redict busfault to REE */

      up_secure_irq(NVIC_IRQ_BUSFAULT, false);
    }

  /* Get non-secure SP */

  if (regs[REG_EXC_RETURN] & EXC_RETURN_THREAD_MODE)
    {
      __asm__ __volatile__ ("mrs %0, psp_ns" : "=r" (g_psp_ns));
      sp_ns = g_psp_ns;
    }
  else
    {
      __asm__ __volatile__ ("mrs %0, msp_ns" : "=r" (g_msp_ns));
      sp_ns = g_msp_ns;
    }

  _alert("Dump REE registers:\n");
  _alert("R0: %08" PRIx32 " R1: %08" PRIx32
         " R2: %08" PRIx32 "  R3: %08" PRIx32 "\n",
         getreg32(sp_ns + OFFSET_R0), getreg32(sp_ns + OFFSET_R1),
         getreg32(sp_ns + OFFSET_R2), getreg32(sp_ns + OFFSET_R3));
  _alert("IP: %08" PRIx32 " SP: %08" PRIx32
          " LR: %08" PRIx32 "  PC: %08" PRIx32 "\n",
         getreg32(sp_ns + OFFSET_R12), sp_ns,
         getreg32(sp_ns + OFFSET_R14), getreg32(sp_ns + OFFSET_R15));

  syslog_flush();

  /* Force set return ReturnAddress to 0, then non-secure cpu will crash.
   * Also, the ReturnAddress is very important, so move it to R12.
   */

  putreg32(getreg32(sp_ns + OFFSET_R15), sp_ns + OFFSET_R12);
  putreg32(0, sp_ns + OFFSET_R15);

  return 1;
}
