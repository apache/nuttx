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
#else
#  define sfalert(...)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_securefault
 *
 * Description:
 *   This is Secure Fault exception handler.  It also catches SVC call
 *   exceptions that are performed in bad contexts.
 *
 ****************************************************************************/

int arm_securefault(int irq, FAR void *context, FAR void *arg)
{
  uint32_t sfsr = getreg32(SAU_SFSR);

  sfalert("PANIC!!! Secure Fault:\n");
  sfalert("\tIRQ: %d regs: %p\n", irq, context);
  sfalert("\tBASEPRI: %08x PRIMASK: %08x IPSR: %08x CONTROL: %08x\n",
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

  up_irq_save();
  PANIC();
  return OK;
}
