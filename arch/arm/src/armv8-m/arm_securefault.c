/****************************************************************************
 * arch/arm/src/armv8-m/arm_securefault.c
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
#include <string.h>
#include <assert.h>
#include <debug.h>

#include <arch/irq.h>

#include "nvic.h"
#include "sau.h"
#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_SECUREFAULT
#  define sfalert(format, ...)  _alert(format, ##__VA_ARGS__)

#  define OFFSET_R0              (0 * 4) /* R0 */
#  define OFFSET_R1              (1 * 4) /* R1 */
#  define OFFSET_R2              (2 * 4) /* R2 */
#  define OFFSET_R3              (3 * 4) /* R3 */
#  define OFFSET_R12             (4 * 4) /* R12 */
#  define OFFSET_R14             (5 * 4) /* R14 = LR */
#  define OFFSET_R15             (6 * 4) /* R15 = PC */
#  define OFFSET_XPSR            (7 * 4) /* xPSR */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void generate_nonsecure_busfault(void)
{
  uint32_t nsp;

  /* Get non-secure SP */

  __asm__ __volatile__ ("mrs %0, msp_ns" : "=r" (nsp));

  sfalert("Non-sec sp %08" PRIx32 "\n", nsp);
  syslog_flush();

  /* Force set return ReturnAddress to 0, then non-secure cpu will crash.
   * Also, the ReturnAddress is very important, so move it to R12.
   */

  putreg32(getreg32(nsp + OFFSET_R15), nsp + OFFSET_R12);
  putreg32(0, nsp + OFFSET_R15);
}
#else
#  define sfalert(...)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_securefault_should_generate
 *
 * Description:
 *   Check whether should generate non-secure IRQ from securefault
 *
 ****************************************************************************/

bool weak_function arm_should_generate_nonsecure_busfault(void)
{
  return true;
}

/****************************************************************************
 * Name: arm_securefault
 *
 * Description:
 *   This is Secure Fault exception handler.  It also catches SVC call
 *   exceptions that are performed in bad contexts.
 *
 ****************************************************************************/

int arm_securefault(int irq, void *context, void *arg)
{
  uint32_t sfsr = getreg32(SAU_SFSR);

  sfalert("PANIC!!! Secure Fault:\n");
  sfalert("\tIRQ: %d regs: %p\n", irq, context);
  sfalert("\tBASEPRI: %08x PRIMASK: %08x IPSR: %08"
          PRIx32 " CONTROL: %08" PRIx32 "\n",
          getbasepri(), getprimask(), getipsr(), getcontrol());
  sfalert("\tCFSR: %08x HFSR: %08x DFSR: %08x\n", getreg32(NVIC_CFAULTS),
          getreg32(NVIC_HFAULTS), getreg32(NVIC_DFAULTS));
  sfalert("\tBFAR: %08x AFSR: %08x SFAR: %08x\n",
          getreg32(NVIC_BFAULT_ADDR), getreg32(NVIC_AFAULTS),
          getreg32(SAU_SFAR));

  sfalert("Secure Fault Reason:\n");
  if (sfsr & SAU_SFSR_INVEP)
    {
      sfalert("\tInvalid entry point\n");
    }

  if (sfsr & SAU_SFSR_INVIS)
    {
      sfalert("\tInvalid integrity signature\n");
    }

  if (sfsr & SAU_SFSR_INVER)
    {
      sfalert("\tInvalid exception return\n");
    }

  if (sfsr & SAU_SFSR_AUVIOL)
    {
      sfalert("\tAttribution unit violation\n");
    }

  if (sfsr & SAU_SFSR_INVTRAN)
    {
      sfalert("\tInvalid transition\n");
    }

  if (sfsr & SAU_SFSR_LSPERR)
    {
      sfalert("\tLazy state preservation\n");
    }

  if (sfsr & SAU_SFSR_LSERR)
    {
      sfalert("\tLazy state error\n");
    }

  /* clear SFSR sticky bits */

  putreg32(0xff, SAU_SFSR);

#ifdef CONFIG_DEBUG_SECUREFAULT
  if (arm_should_generate_nonsecure_busfault())
    {
      generate_nonsecure_busfault();
      return OK;
    }
#endif

  up_irq_save();
  PANIC();

  return OK;
}
