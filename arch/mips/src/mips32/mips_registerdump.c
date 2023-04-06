/****************************************************************************
 * arch/mips/src/mips32/mips_registerdump.c
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

#include <inttypes.h>
#include <stdint.h>
#include <stdlib.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "mips_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_getusrsp
 ****************************************************************************/

uintptr_t up_getusrsp(void *regs)
{
  uint32_t *ptr = regs;
  return ptr[REG_SP];
}

/****************************************************************************
 * Name: up_dump_register
 ****************************************************************************/

void up_dump_register(void *dumpregs)
{
  volatile uint32_t *regs = dumpregs ? dumpregs : CURRENT_REGS;

  /* Are user registers available from interrupt processing? */

  if (regs)
    {
      _alert("MFLO:%08" PRIx32 " MFHI:%08" PRIx32
             " EPC:%08" PRIx32 " STATUS:%08" PRIx32 "\n",
             regs[REG_MFLO], regs[REG_MFHI],
             regs[REG_EPC], regs[REG_STATUS]);
      _alert("AT:%08" PRIx32 " V0:%08" PRIx32 " V1:%08" PRIx32
             " A0:%08" PRIx32 " A1:%08" PRIx32 " A2:%08" PRIx32
             " A3:%08" PRIx32 "\n",
             regs[REG_AT], regs[REG_V0],
             regs[REG_V1], regs[REG_A0],
             regs[REG_A1], regs[REG_A2],
             regs[REG_A3]);
      _alert("T0:%08" PRIx32 " T1:%08" PRIx32 " T2:%08" PRIx32
             " T3:%08" PRIx32 " T4:%08" PRIx32 " T5:%08" PRIx32
             " T6:%08" PRIx32 " T7:%08" PRIx32 "\n",
             regs[REG_T0], regs[REG_T1],
             regs[REG_T2], regs[REG_T3],
             regs[REG_T4], regs[REG_T5],
             regs[REG_T6], regs[REG_T7]);
      _alert("S0:%08" PRIx32 " S1:%08" PRIx32 " S2:%08" PRIx32
             " S3:%08" PRIx32 " S4:%08" PRIx32 " S5:%08" PRIx32
             " S6:%08" PRIx32 " S7:%08" PRIx32 "\n",
             regs[REG_S0], regs[REG_S1],
             regs[REG_S2], regs[REG_S3],
             regs[REG_S4], regs[REG_S5],
             regs[REG_S6], regs[REG_S7]);
#ifdef MIPS32_SAVE_GP
      _alert("T8:%08" PRIx32 " T9:%08" PRIx32 " GP:%08" PRIx32
             " SP:%08" PRIx32 " FP:%08" PRIx32 " RA:%08" PRIx32 "\n",
             regs[REG_T8], regs[REG_T9],
             regs[REG_GP], regs[REG_SP],
             regs[REG_FP], regs[REG_RA]);
#else
      _alert("T8:%08" PRIx32 " T9:%08" PRIx32 " SP:%08" PRIx32
             " FP:%08" PRIx32 " RA:%08" PRIx32 "\n",
             regs[REG_T8], regs[REG_T9],
             regs[REG_SP], regs[REG_FP],
             regs[REG_RA]);
#endif
    }
}
