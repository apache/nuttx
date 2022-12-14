/****************************************************************************
 * arch/arm/src/common/arm_registerdump.c
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

#include <stdio.h>
#include <stdint.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "arm_internal.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_getusrsp
 ****************************************************************************/

uintptr_t up_getusrsp(void)
{
  return CURRENT_REGS[REG_SP];
}

/****************************************************************************
 * Name: arm_registerdump
 ****************************************************************************/

void arm_registerdump(volatile uint32_t *regs)
{
  /* Dump the interrupt registers */

  _alert("R0: %08" PRIx32 " R1: %08" PRIx32
         " R2: %08" PRIx32 "  R3: %08" PRIx32 "\n",
         regs[REG_R0], regs[REG_R1], regs[REG_R2], regs[REG_R3]);
#ifdef CONFIG_ARM_THUMB
  _alert("R4: %08" PRIx32 " R5: %08" PRIx32
         " R6: %08" PRIx32 "  FP: %08" PRIx32 "\n",
         regs[REG_R4], regs[REG_R5], regs[REG_R6], regs[REG_R7]);
  _alert("R8: %08" PRIx32 " SB: %08" PRIx32
         " SL: %08" PRIx32 " R11: %08" PRIx32 "\n",
         regs[REG_R8], regs[REG_R9], regs[REG_R10], regs[REG_R11]);
#else
  _alert("R4: %08" PRIx32 " R5: %08" PRIx32
         " R6: %08" PRIx32 "  R7: %08" PRIx32 "\n",
         regs[REG_R4], regs[REG_R5], regs[REG_R6], regs[REG_R7]);
  _alert("R8: %08" PRIx32 " SB: %08" PRIx32
         " SL: %08" PRIx32 "  FP: %08" PRIx32 "\n",
         regs[REG_R8], regs[REG_R9], regs[REG_R10], regs[REG_R11]);
#endif
  _alert("IP: %08" PRIx32 " SP: %08" PRIx32
         " LR: %08" PRIx32 "  PC: %08" PRIx32 "\n",
         regs[REG_R12], regs[REG_R13], regs[REG_R14], regs[REG_R15]);

#if defined(REG_BASEPRI)
  _alert("xPSR: %08" PRIx32 " BASEPRI: %08" PRIx32
         " CONTROL: %08" PRIx32 "\n",
         regs[REG_XPSR], regs[REG_BASEPRI], getcontrol());
#elif defined(REG_PRIMASK)
  _alert("xPSR: %08" PRIx32 " PRIMASK: %08" PRIx32
         " CONTROL: %08" PRIx32 "\n",
         regs[REG_XPSR], regs[REG_PRIMASK], getcontrol());
#elif defined(REG_CPSR)
  _alert("CPSR: %08" PRIx32 "\n", regs[REG_CPSR]);
#endif

#ifdef REG_EXC_RETURN
  _alert("EXC_RETURN: %08" PRIx32 "\n", regs[REG_EXC_RETURN]);
#endif
}
