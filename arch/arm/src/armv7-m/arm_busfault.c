/****************************************************************************
 * arch/arm/src/armv7-m/arm_busfault.c
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
#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_BUSFAULT
# define bfalert(format, ...)  _alert(format, ##__VA_ARGS__)
#else
# define bfalert(x...)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_busfault
 *
 * Description:
 *   This is Bus Fault exception handler.  It also catches SVC call
 *   exceptions that are performed in bad contexts.
 *
 ****************************************************************************/

int arm_busfault(int irq, void *context, void *arg)
{
  uint32_t cfsr = getreg32(NVIC_CFAULTS);

  bfalert("PANIC!!! Bus Fault:\n");
  bfalert("\tIRQ: %d regs: %p\n", irq, context);
  bfalert("\tBASEPRI: %08x PRIMASK: %08x IPSR: %08x CONTROL: %08x\n",
          getbasepri(), getprimask(), getipsr(), getcontrol());
  bfalert("\tCFSR: %08x HFSR: %08x DFSR: %08x BFAR: %08x AFSR: %08x\n",
          cfsr, getreg32(NVIC_HFAULTS), getreg32(NVIC_DFAULTS),
          getreg32(NVIC_BFAULT_ADDR), getreg32(NVIC_AFAULTS));

  bfalert("Bus Fault Reason:\n");
  if (cfsr & NVIC_CFAULTS_IBUSERR)
    {
      bfalert("\tInstruction bus error\n");
    }

  if (cfsr & NVIC_CFAULTS_PRECISERR)
    {
      bfalert("\tPrecise data bus error\n");
    }

  if (cfsr & NVIC_CFAULTS_IMPRECISERR)
    {
      bfalert("\tImprecise data bus error\n");
    }

  if (cfsr & NVIC_CFAULTS_UNSTKERR)
    {
      bfalert("\tBusFault on unstacking for a return from exception\n");
    }

  if (cfsr & NVIC_CFAULTS_STKERR)
    {
      bfalert("\tBusFault on stacking for exception entry\n");
    }

  if (cfsr & NVIC_CFAULTS_LSPERR)
    {
      bfalert("\tFloating-point lazy state preservation error\n");
    }

  up_irq_save();
  PANIC();
  return OK;
}
