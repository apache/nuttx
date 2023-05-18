/****************************************************************************
 * arch/arm/src/armv8-m/arm_gen_nonsecfault.c
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
  uint32_t nsp;

  if (!arm_should_gen_nonsecurefault())
    {
      return 0;
    }

  /* Check whether come from REE */

  if (regs[REG_EXC_RETURN] & EXC_RETURN_SECURE_STACK)
    {
      return 0;
    }

  /* busfault are forward to REE ? */

  if (getreg32(NVIC_AIRCR) & NVIC_AIRCR_BFHFNMINS)
    {
      return 0;
    }

  /* Redict busfault to REE */

  up_secure_irq(NVIC_IRQ_BUSFAULT, false);

  /* Get non-secure SP */

  __asm__ __volatile__ ("mrs %0, msp_ns" : "=r" (nsp));

  _alert("Dump REE registers:\n");
  _alert("R0: %08" PRIx32 " R1: %08" PRIx32
         " R2: %08" PRIx32 "  R3: %08" PRIx32 "\n",
         getreg32(nsp + OFFSET_R0), getreg32(nsp + OFFSET_R1),
         getreg32(nsp + OFFSET_R2), getreg32(nsp + OFFSET_R3));
  _alert("IP: %08" PRIx32 " SP: %08" PRIx32
          " LR: %08" PRIx32 "  PC: %08" PRIx32 "\n",
         getreg32(nsp + OFFSET_R12), nsp,
         getreg32(nsp + OFFSET_R14), getreg32(nsp + OFFSET_R15));
  syslog_flush();

  /* Force set return ReturnAddress to 0, then non-secure cpu will crash.
   * Also, the ReturnAddress is very important, so move it to R12.
   */

  putreg32(getreg32(nsp + OFFSET_R15), nsp + OFFSET_R12);
  putreg32(0, nsp + OFFSET_R15);

  return 1;
}
