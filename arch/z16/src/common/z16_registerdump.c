/****************************************************************************
 * arch/z16/src/common/z16_registerdump.c
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

#include "z16_internal.h"

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
 * Name: up_dump_register
 ****************************************************************************/

void up_dump_register(FAR void *dumpregs)
{
#ifdef CONFIG_DEBUG_INFO
  FAR volatile uint32_t *regs = dumpregs ? dumpregs : g_current_regs;

  _alert("R0 :%08x R1 :%08x R2 :%08x R3 :%08x "
         "R4 :%08x R5 :%08x R6 :%08x R7 :%08x\n"
         regs[REG_R0 / 2],  regs[REG_R1 / 2],  regs[REG_R2 / 2],
         regs[REG_R3 / 2],  regs[REG_R4 / 2],  regs[REG_R5 / 2],
         regs[REG_R6 / 2],  regs[REG_R7 / 2]);
  _alert("R8 :%08x R9 :%08x R10:%08x R11:%08x R12:%08x R13:%08x\n"
         regs[REG_R8 / 2],  regs[REG_R9 / 2],  regs[REG_R10 / 2],
         regs[REG_R11 / 2],  regs[REG_R12 / 2], regs[REG_R13 / 2]);
  _alert("FP :%08x SP :%08x FLG:%04x\n"
         regs[REG_R14 / 2], regs[REG_R15 / 2], regs[REG_FLAGS]);
#endif
}
