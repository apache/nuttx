/****************************************************************************
 * arch/renesas/src/sh1/sh1_registerdump.c
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
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "renesas_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_getusrsp
 ****************************************************************************/

uintptr_t up_getusrsp(void)
{
  return g_current_regs[REG_SP];
}

/****************************************************************************
 * Name: renesas_registerdump
 ****************************************************************************/

void renesas_registerdump(volatile uint32_t *regs)
{
  /* Dump the interrupt registers */

  _alert("PC: %08x SR=%08x\n",
         regs[REG_PC], regs[REG_SR]);

  _alert("PR: %08x GBR: %08x MACH: %08x MACL: %08x\n",
         regs[REG_PR], regs[REG_GBR], regs[REG_MACH], regs[REG_MACL]);

  _alert("R%d: %08x %08x %08x %08x %08x %08x %08x %08x\n", 0,
         regs[REG_R0], regs[REG_R1], regs[REG_R2], regs[REG_R3],
         regs[REG_R4], regs[REG_R5], regs[REG_R6], regs[REG_R7]);

  _alert("R%d: %08x %08x %08x %08x %08x %08x %08x %08x\n", 8,
         regs[REG_R8], regs[REG_R9], regs[REG_R10], regs[REG_R11],
         regs[REG_R12], regs[REG_R13], regs[REG_R14], regs[REG_R15]);
}

