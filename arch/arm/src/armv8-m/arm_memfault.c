/****************************************************************************
 * arch/arm/src/armv8-m/arm_memfault.c
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
#include <inttypes.h>

#include <arch/irq.h>

#include "nvic.h"
#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_MEMFAULT
# define mfalert(format, ...)  _alert(format, ##__VA_ARGS__)
#else
# define mfalert(x...)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_memfault
 *
 * Description:
 *   This is Memory Management Fault exception handler.  Normally we get
 *   here when the Cortex M3 MPU is enabled and an MPU fault is detected.
 *   However, I understand that there are other error conditions that can
 *   also generate memory management faults.
 *
 ****************************************************************************/

int arm_memfault(int irq, void *context, void *arg)
{
  uint32_t cfsr = getreg32(NVIC_CFAULTS);

  /* Dump some memory management fault info */

  mfalert("PANIC!!! Memory Management Fault:\n");
  mfalert("\tIRQ: %d context: %p\n", irq, context);
  mfalert("\tCFSR: %08x MMFAR: %08x\n",
          getreg32(NVIC_CFAULTS), getreg32(NVIC_MEMMANAGE_ADDR));
  mfalert("\tBASEPRI: %08x PRIMASK: %08x IPSR: %08x CONTROL: %08x\n",
          getbasepri(), getprimask(), getipsr(), getcontrol());

  mfalert("Memory Management Fault Reason:\n");
  if (cfsr & NVIC_CFAULTS_IACCVIOL)
    {
      mfalert("\tInstruction access violation\n");
    }

  if (cfsr & NVIC_CFAULTS_DACCVIOL)
    {
      mfalert("\tData access violation\n");
    }

  if (cfsr & NVIC_CFAULTS_MUNSTKERR)
    {
      mfalert("\tMemManage fault on unstacking\n");
    }

  if (cfsr & NVIC_CFAULTS_MSTKERR)
    {
      mfalert("\tMemManage fault on stacking\n");
    }

  if (cfsr & NVIC_CFAULTS_MLSPERR)
    {
      mfalert("\tFloating-point lazy state preservation error\n");
    }

  up_irq_save();
  PANIC();
  return OK; /* Won't get here */
}
