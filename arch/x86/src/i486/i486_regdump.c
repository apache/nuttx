/****************************************************************************
 * arch/x86/src/i486/i486_regdump.c
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

#include <debug.h>
#include <nuttx/irq.h>

#include "x86_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_getusrsp
 ****************************************************************************/

uintptr_t up_getusrsp(void)
{
  return g_current_regs[REG_ESP];
}

/****************************************************************************
 * Name: x86_registerdump
 ****************************************************************************/

void x86_registerdump(uint32_t *regs)
{
  _alert(" ds:%08x irq:%08x err:%08x\n",
         regs[REG_DS], regs[REG_IRQNO], regs[REG_ERRCODE]);
  _alert("edi:%08x esi:%08x ebp:%08x esp:%08x\n",
         regs[REG_EDI], regs[REG_ESI], regs[REG_EBP], regs[REG_ESP]);
  _alert("ebx:%08x edx:%08x ecx:%08x eax:%08x\n",
         regs[REG_EBX], regs[REG_EDX], regs[REG_ECX], regs[REG_EAX]);
  _alert("eip:%08x  cs:%08x flg:%08x  sp:%08x ss:%08x\n",
         regs[REG_EIP], regs[REG_CS], regs[REG_EFLAGS], regs[REG_SP],
         regs[REG_SS]);
}
