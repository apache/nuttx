/****************************************************************************
 * arch/risc-v/src/common/riscv_exception.c
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
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "riscv_internal.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const char *g_reasons_str[RISCV_MAX_EXCEPTION + 1] =
{
  "Instruction address misaligned",
  "Instruction access fault",
  "Illegal instruction",
  "Breakpoint",
  "Load address misaligned",
  "Load access fault",
  "Store/AMO address misaligned",
  "Store/AMO access fault",
  "Environment call from U-mode",
  "Environment call from S-mode",
  "Environment call from H-mode",
  "Environment call from M-mode",
  "Instruction page fault",
  "Load page fault",
  "Reserved",
  "Store/AMO page fault"
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: riscv_exception
 *
 * Description:
 *   This is the exception handler.
 *
 ****************************************************************************/

int riscv_exception(int mcause, void *regs, void *args)
{
  uintptr_t cause = mcause & RISCV_IRQ_MASK;

  if (mcause > RISCV_MAX_EXCEPTION)
    {
      _alert("EXCEPTION: Unknown.  MCAUSE: %" PRIxREG "\n", cause);
    }
  else
    {
      _alert("EXCEPTION: %s. MCAUSE: %" PRIxREG "\n",
             g_reasons_str[cause], cause);
    }

  _alert("PANIC!!! Exception = %" PRIxREG "\n", cause);
  up_irq_save();
  CURRENT_REGS = regs;
  PANIC();

  return 0;
}

/****************************************************************************
 * Name: riscv_exception_attach
 *
 * Description:
 *   Attach standard exception with suitable handler
 *
 ****************************************************************************/

void riscv_exception_attach(void)
{
  irq_attach(RISCV_IRQ_IAMISALIGNED, riscv_exception, NULL);
  irq_attach(RISCV_IRQ_IAFAULT, riscv_exception, NULL);
  irq_attach(RISCV_IRQ_IINSTRUCTION, riscv_exception, NULL);
  irq_attach(RISCV_IRQ_BPOINT, riscv_exception, NULL);
  irq_attach(RISCV_IRQ_LAFAULT, riscv_exception, NULL);
  irq_attach(RISCV_IRQ_SAFAULT, riscv_exception, NULL);

#ifdef CONFIG_RISCV_MISALIGNED_HANDLER
  irq_attach(RISCV_IRQ_LAMISALIGNED, riscv_misaligned, NULL);
  irq_attach(RISCV_IRQ_SAMISALIGNED, riscv_misaligned, NULL);
#else
  irq_attach(RISCV_IRQ_LAMISALIGNED, riscv_exception, NULL);
  irq_attach(RISCV_IRQ_SAMISALIGNED, riscv_exception, NULL);
#endif

  /* Attach the ecall interrupt handler */

#ifndef CONFIG_BUILD_FLAT
  irq_attach(RISCV_IRQ_ECALLU, riscv_swint, NULL);
#else
  irq_attach(RISCV_IRQ_ECALLU, riscv_exception, NULL);
#endif

  irq_attach(RISCV_IRQ_ECALLS, riscv_exception, NULL);
  irq_attach(RISCV_IRQ_ECALLH, riscv_exception, NULL);

#ifndef CONFIG_ARCH_USE_S_MODE
  irq_attach(RISCV_IRQ_ECALLM, riscv_swint, NULL);
#else
  irq_attach(RISCV_IRQ_ECALLM, riscv_exception, NULL);
#endif

  irq_attach(RISCV_IRQ_INSTRUCTIONPF, riscv_exception, NULL);
  irq_attach(RISCV_IRQ_LOADPF, riscv_exception, NULL);
  irq_attach(RISCV_IRQ_RESERVED, riscv_exception, NULL);
  irq_attach(RISCV_IRQ_STOREPF, riscv_exception, NULL);

#ifdef CONFIG_SMP
  irq_attach(RISCV_IRQ_MSOFT, riscv_pause_handler, NULL);
#else
  irq_attach(RISCV_IRQ_MSOFT, riscv_exception, NULL);
#endif
}
