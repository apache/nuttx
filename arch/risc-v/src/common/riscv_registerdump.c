/****************************************************************************
 * arch/risc-v/src/common/riscv_registerdump.c
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

#include "riscv_internal.h"

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
 * Name: riscv_registerdump
 ****************************************************************************/

void riscv_registerdump(volatile uintptr_t *regs)
{
  /* Are user registers available from interrupt processing? */

  _alert("EPC: %" PRIxREG "\n", regs[REG_EPC]);
  _alert("A0: %" PRIxREG " A1: %" PRIxREG " A2: %" PRIxREG
         " A3: %" PRIxREG "\n",
         regs[REG_A0], regs[REG_A1], regs[REG_A2], regs[REG_A3]);
  _alert("A4: %" PRIxREG " A5: %" PRIxREG " A6: %" PRIxREG
         " A7: %" PRIxREG "\n",
         regs[REG_A4], regs[REG_A5], regs[REG_A6], regs[REG_A7]);
  _alert("T0: %" PRIxREG " T1: %" PRIxREG " T2: %" PRIxREG
         " T3: %" PRIxREG "\n",
         regs[REG_T0], regs[REG_T1], regs[REG_T2], regs[REG_T3]);
  _alert("T4: %" PRIxREG " T5: %" PRIxREG " T6: %" PRIxREG "\n",
         regs[REG_T4], regs[REG_T5], regs[REG_T6]);
  _alert("S0: %" PRIxREG " S1: %" PRIxREG " S2: %" PRIxREG
         " S3: %" PRIxREG "\n",
         regs[REG_S0], regs[REG_S1], regs[REG_S2], regs[REG_S3]);
  _alert("S4: %" PRIxREG " S5: %" PRIxREG " S6: %" PRIxREG
         " S7: %" PRIxREG "\n",
         regs[REG_S4], regs[REG_S5], regs[REG_S6], regs[REG_S7]);
  _alert("S8: %" PRIxREG " S9: %" PRIxREG " S10: %" PRIxREG
         " S11: %" PRIxREG "\n",
         regs[REG_S8], regs[REG_S9], regs[REG_S10], regs[REG_S11]);
#ifdef RISCV_SAVE_GP
  _alert("GP: %" PRIxREG " SP: %" PRIxREG " FP: %" PRIxREG
         " TP: %" PRIxREG " RA: %" PRIxREG "\n",
         regs[REG_GP], regs[REG_SP], regs[REG_FP], regs[REG_TP],
         regs[REG_RA]);
#else
  _alert("SP: %" PRIxREG " FP: %" PRIxREG " TP: %" PRIxREG
         " RA: %" PRIxREG "\n",
         regs[REG_SP], regs[REG_FP], regs[REG_TP], regs[REG_RA]);
#endif
}

