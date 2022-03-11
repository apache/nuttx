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
#include <stdlib.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/syslog/syslog.h>
#include <arch/irq.h>

#include "riscv_internal.h"

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
  "Reserved",
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

void riscv_exception(uintptr_t mcause, uintptr_t *regs)
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
}

