/****************************************************************************
 * arch/arm/src/armv8-m/arm_usagefault.c
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

#ifdef CONFIG_DEBUG_USAGEFAULT
# define ufalert(format, ...)  _alert(format, ##__VA_ARGS__)
#else
# define ufalert(x...)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_usagefault
 *
 * Description:
 *   This is Usage Fault exception handler.  It also catches SVC call
 *   exceptions that are performed in bad contexts.
 *
 ****************************************************************************/

int arm_usagefault(int irq, void *context, void *arg)
{
  uint32_t cfsr = getreg32(NVIC_CFAULTS);

  /* Dump some usage fault info */

  ufalert("PANIC!!! Usage Fault:\n");
  ufalert("\tIRQ: %d regs: %p\n", irq, context);
  ufalert("\tBASEPRI: %08x PRIMASK: %08x IPSR: %08"
          PRIx32 " CONTROL: %08" PRIx32 "\n",
          getbasepri(), getprimask(), getipsr(), getcontrol());
  ufalert("\tCFSR: %08" PRIx32 " HFSR: %08" PRIx32 " DFSR: %08"
          PRIx32 " BFAR: %08" PRIx32 " AFSR: %08" PRIx32 "\n",
          cfsr, getreg32(NVIC_HFAULTS), getreg32(NVIC_DFAULTS),
          getreg32(NVIC_BFAULT_ADDR), getreg32(NVIC_AFAULTS));

  ufalert("Usage Fault Reason:\n");
  if (cfsr & NVIC_CFAULTS_UNDEFINSTR)
    {
      ufalert("\tUndefined instruction\n");
    }

  if (cfsr & NVIC_CFAULTS_INVSTATE)
    {
      ufalert("\tInvalid state\n");
    }

  if (cfsr & NVIC_CFAULTS_INVPC)
    {
      ufalert("\tInvalid PC load, "
              "caused by an invalid PC load by EXC_RETURN\n");
    }

  if (cfsr & NVIC_CFAULTS_NOCP)
    {
      ufalert("\tNo Coprocessor\n");
    }

  if (cfsr & NVIC_CFAULTS_STKOF)
    {
      ufalert("\tStack Overflow\n");
    }

  if (cfsr & NVIC_CFAULTS_UNALIGNED)
    {
      ufalert("\tUnaligned access\n");
    }

  if (cfsr & NVIC_CFAULTS_DIVBYZERO)
    {
      ufalert("\tDivide by zero\n");
    }

  up_irq_save();
  PANIC_WITH_REGS("panic", context);
  return OK;
}
