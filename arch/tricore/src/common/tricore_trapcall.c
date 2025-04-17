/****************************************************************************
 * arch/tricore/src/common/tricore_trapcall.c
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

#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <syscall.h>

#include <arch/irq.h>
#include <sched/sched.h>
#include <nuttx/coredump.h>
#include <nuttx/sched.h>

#include "tricore_internal.h"

#include "IfxCpu_Trap.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static IfxCpu_Trap g_trapinfo;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void tricore_trapinfo(volatile void *trap)
{
  IfxCpu_Trap *ctrap = (IfxCpu_Trap *)trap;

  g_trapinfo.tCpu   = ctrap->tCpu;
  g_trapinfo.tClass = ctrap->tClass;
  g_trapinfo.tId    = ctrap->tId;
  g_trapinfo.tAddr  = ctrap->tAddr;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int tricore_mmutrap(uint32_t tid, void *context, void *arg)
{
  _alert("PANIC!!! MMU Trap:\n");
  _alert("\tClass %d TID: %" PRId32 " regs: %p\n",
         IfxCpu_Trap_Class_memoryManagement,
         tid, context);

  _alert("MMU Trap Reason:\n");
  if (tid == IfxCpu_Trap_MemoryManagement_Id_virtualAddressFill)
    {
      _alert("\tVirtual Address Fill\n");
    }

  if (tid == IfxCpu_Trap_MemoryManagement_Id_virtualAddressProtection)
    {
      _alert("\tVirtual Address Protection\n");
    }

  up_irq_save();
  PANIC_WITH_REGS("panic", context);
  return OK;
}

int tricore_internalprotrape(uint32_t tid, void *context, void *arg)
{
  _alert("PANIC!!! Internal Protection Trap:\n");
  _alert("\tClass %d TID: %" PRId32 " regs: %p\n",
        IfxCpu_Trap_Class_internalProtection, tid, context);

  _alert("Internal Protection Reason:\n");
  if (tid == IfxCpu_Trap_InternalProtection_Id_privilegeViolation)
    {
      _alert("\tPrivileged Instruction\n");
    }

  if (tid == IfxCpu_Trap_InternalProtection_Id_memoryProtectionRead)
    {
      _alert("\tMemory Protection Read\n");
    }

  if (tid == IfxCpu_Trap_InternalProtection_Id_memoryProtectionWrite)
    {
      _alert("\tMemory Proteciton Write\n");
    }

  if (tid == IfxCpu_Trap_InternalProtection_Id_memoryProtectionExecute)
    {
      _alert("\tMemory Protection Execution\n");
    }

  if (tid ==
      IfxCpu_Trap_InternalProtection_Id_memoryProtectionPeripheralAccess)
    {
      _alert("\tMemory Protection Peripheral Access\n");
    }

  if (tid == IfxCpu_Trap_InternalProtection_Id_memoryProtectionNullAddress)
    {
      _alert("\tMemory Protection Null Address\n");
    }

  if (tid == IfxCpu_Trap_InternalProtection_Id_globalRegisterWriteProtection)
    {
      _alert("\tGlobal Register Write Protection\n");
    }

  up_irq_save();
  PANIC_WITH_REGS("panic", context);
  return OK;
}

int tricore_insterrorstrap(uint32_t tid, void *context, void *arg)
{
  _alert("PANIC!!! Instruction Errors Trap:\n");
  _alert("\tClass %d TID: %" PRId32 " regs: %p\n",
        IfxCpu_Trap_Class_instructionErrors, tid, context);

  _alert("Instruction Errors Trap Reason:\n");
  if (tid == IfxCpu_Trap_InstructionErrors_Id_illegalOpcode)
    {
      _alert("\tIllegal Opcode\n");
    }

  if (tid == IfxCpu_Trap_InstructionErrors_Id_unimplementedOpcode)
    {
      _alert("\tUnimplemented Opcode\n");
    }

  if (tid == IfxCpu_Trap_InstructionErrors_Id_invalidOperand)
    {
      _alert("\tInvalid Operand Specification\n");
    }

  if (tid == IfxCpu_Trap_InstructionErrors_Id_dataAddressAlignment)
    {
      _alert("\tData Address Alignment\n");
    }

  if (tid == IfxCpu_Trap_InstructionErrors_Id_invalidMemoryAddress)
    {
      _alert("\tInvalid Local Memory Address\n");
    }

  up_irq_save();
  PANIC_WITH_REGS("panic", context);
  return OK;
}

int tricore_contexmnttrap(uint32_t tid, void *context, void *arg)
{
  _alert("PANIC!!! Context Management Trap:\n");
  _alert("\tClass %d TID: %" PRId32 " regs: %p\n",
        IfxCpu_Trap_Class_contextManagement, tid, context);

  _alert("Context Management Reason:\n");
  if (tid == IfxCpu_Trap_ContextManagement_Id_freeContextListDepletion)
    {
      _alert("\tFree Context List Depletion\n");
    }

  if (tid == IfxCpu_Trap_ContextManagement_Id_callDepthOverflow)
    {
      _alert("\tCall Depth Overflow\n");
    }

  if (tid == IfxCpu_Trap_ContextManagement_Id_callDepthUnderflow)
    {
      _alert("\tCall Depth Underflow\n");
    }

  if (tid == IfxCpu_Trap_ContextManagement_Id_freeContextListUnderflow)
    {
      _alert("\tFree Context List Underflow\n");
    }

  if (tid == IfxCpu_Trap_ContextManagement_Id_callStackUnderflow)
    {
      _alert("\tCall Stack Underflow\n");
    }

  if (tid == IfxCpu_Trap_ContextManagement_Id_contextType)
    {
      _alert("\tContext Type\n");
    }

  if (tid == IfxCpu_Trap_ContextManagement_Id_nestingError)
    {
      _alert("\tNesting Error:RFE with non-zero call depth\n");
    }

  up_irq_save();
  PANIC_WITH_REGS("panic", context);
  return OK;
}

int tricore_bustrap(uint32_t tid, void *context, void *arg)
{
  _alert("PANIC!!! System Bus Trap:\n");
  _alert("\tClass %d TID: %" PRId32 " regs: %p\n", IfxCpu_Trap_Class_bus,
         tid, context);

  _alert("System Bus Reason:\n");
  if (tid == IfxCpu_Trap_Bus_Id_programFetchSynchronousError)
    {
      _alert("\tProgram Fetch Synchronous Error\n");
    }

  if (tid == IfxCpu_Trap_Bus_Id_dataAccessSynchronousError)
    {
      _alert("\tData Access Synchronous Error\n");
    }

  if (tid == IfxCpu_Trap_Bus_Id_dataAccessAsynchronousError)
    {
      _alert("\tData Access Asysnchronous Error\n");
    }

  if (tid == IfxCpu_Trap_Bus_Id_CoprocessorTrapAsynchronousError)
    {
      _alert("\tCoprocessor Trap Asynchronous Error\n");
    }

  if (tid == IfxCpu_Trap_Bus_Id_programMemoryIntegrityError)
    {
      _alert("\tProgram Memory Integrity Error\n");
    }

  if (tid == IfxCpu_Trap_Bus_Id_dataMemoryIntegrityError)
    {
      _alert("\tData Memory Integrity Error\n");
    }

  if (tid == IfxCpu_Trap_Bus_Id_temporalAsynchronousError)
    {
      _alert("\tTemporal Asynchronous Error\n");
    }

  up_irq_save();
  PANIC_WITH_REGS("panic", context);
  return OK;
}

int tricore_assertiontrap(uint32_t tid, void *context, void *arg)
{
  _alert("PANIC!!! Assertion Trap:\n");
  _alert("\tClass %d TID: %" PRId32 " regs: %p\n",
         IfxCpu_Trap_Class_assertion,
         tid, context);

  _alert("System Bus Reason:\n");
  if (tid == IfxCpu_Trap_Assertion_Id_arithmeticOverflow)
    {
      _alert("\tArithmetic Overflow\n");
    }

  if (tid == IfxCpu_Trap_Assertion_Id_stickyArithmeticOverflow)
    {
      _alert("\tSticky Arithmetic Overflow\n");
    }

  up_irq_save();
  PANIC_WITH_REGS("panic", context);
  return OK;
}

/****************************************************************************
 * Name: tricore_trapcall
 *
 * Description:
 *   This is Trap exception handler
 *
 ****************************************************************************/

void tricore_trapcall(volatile void *trap)
{
  uintptr_t *regs;
  uintptr_t pcxi;

  IfxCpu_Trap *ctrap = (IfxCpu_Trap *)trap;
  IfxCpu_Trap_Class tclass = (IfxCpu_Trap_Class)ctrap->tClass;
  unsigned int tid = ctrap->tId;

  tricore_trapinfo(trap);

  regs = tricore_csa2addr(__mfcr(CPU_PCXI));
  pcxi = regs[REG_UPCXI];
  regs = tricore_csa2addr(pcxi);

  if (!up_interrupt_context())
    {
      /* Update the current task's regs */

      g_running_tasks[this_cpu()]->xcp.regs = regs;
    }

  up_set_interrupt_context(true);

  if (tclass == IfxCpu_Trap_Class_memoryManagement)
    {
      tricore_mmutrap(tid, regs, NULL);
      return;
    }

  if (tclass == IfxCpu_Trap_Class_internalProtection)
    {
      tricore_internalprotrape(tid, regs, NULL);
      return;
    }

  if (tclass == IfxCpu_Trap_Class_instructionErrors)
    {
      tricore_insterrorstrap(tid, regs, NULL);
      return;
    }

  if (tclass == IfxCpu_Trap_Class_contextManagement)
    {
      tricore_contexmnttrap(tid, regs, NULL);
      return;
    }

  if (tclass == IfxCpu_Trap_Class_bus)
    {
      tricore_bustrap(tid, regs, NULL);
      return;
    }

  if (tclass == IfxCpu_Trap_Class_assertion)
    {
      tricore_assertiontrap(tid, regs, NULL);
      return;
    }

  up_irq_save();
  PANIC_WITH_REGS("Trap", regs);
}

/****************************************************************************
 * Function:  tricore_trapinit
 *
 * Description:
 *   Trap init for tricore arch.
 *
 ****************************************************************************/

void tricore_trapinit(void)
{
#ifdef CONFIG_COREDUMP
  coredump_add_memory_region(&g_trapinfo, sizeof(g_trapinfo),
                             PF_REGISTER);
#endif
}
