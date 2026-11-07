/****************************************************************************
 * arch/rh850/src/common/rh850_registerdump.c
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

#include <inttypes.h>
#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/debug.h>
#include <nuttx/irq.h>
#include <nuttx/sched.h>

#include "rh850_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_dump_register
 ****************************************************************************/

void up_dump_register(FAR void *dumpregs)
{
  volatile uintptr_t *regs = dumpregs ? dumpregs : running_regs();

  _alert("R1: %08" PRIxPTR " R2: %08" PRIxPTR " SP: %08" PRIxPTR
         " R4: %08" PRIxPTR " R5: %08" PRIxPTR "\n",
         regs[REG_R1], regs[REG_R2], regs[REG_SP], regs[REG_R4],
         regs[REG_R5]);
  _alert("R6: %08" PRIxPTR " R7: %08" PRIxPTR " R8: %08" PRIxPTR
         " R9: %08" PRIxPTR "\n",
         regs[REG_R6], regs[REG_R7], regs[REG_R8], regs[REG_R9]);
  _alert("R10: %08" PRIxPTR " R11: %08" PRIxPTR " R12: %08" PRIxPTR
         " R13: %08" PRIxPTR "\n",
         regs[REG_R10], regs[REG_R11], regs[REG_R12], regs[REG_R13]);
  _alert("R14: %08" PRIxPTR " R15: %08" PRIxPTR " R16: %08" PRIxPTR
         " R17: %08" PRIxPTR "\n",
         regs[REG_R14], regs[REG_R15], regs[REG_R16], regs[REG_R17]);
  _alert("R18: %08" PRIxPTR " R19: %08" PRIxPTR " R20: %08" PRIxPTR
         " R21: %08" PRIxPTR "\n",
         regs[REG_R18], regs[REG_R19], regs[REG_R20], regs[REG_R21]);
  _alert("R22: %08" PRIxPTR " R23: %08" PRIxPTR " R24: %08" PRIxPTR
         " R25: %08" PRIxPTR "\n",
         regs[REG_R22], regs[REG_R23], regs[REG_R24], regs[REG_R25]);
  _alert("R26: %08" PRIxPTR " R27: %08" PRIxPTR " R28: %08" PRIxPTR
         " R29: %08" PRIxPTR "\n",
         regs[REG_R26], regs[REG_R27], regs[REG_R28], regs[REG_R29]);
  _alert("EP: %08" PRIxPTR " LP: %08" PRIxPTR "\n",
         regs[REG_EP], regs[REG_LP]);
  _alert("PC: %08" PRIxPTR " PSW: %08" PRIxPTR " FPSR: %08" PRIxPTR
         " FPEPC: %08" PRIxPTR "\n",
         regs[REG_EIPC], regs[REG_EIPSW], regs[REG_FPSR], regs[REG_FPEPC]);
}
