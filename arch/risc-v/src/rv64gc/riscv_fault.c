/****************************************************************************
 * arch/risc-v/src/rv64gc/up_fault.c
 *
 *   Copyright (C) 2020 Masayuki Ishikawa. All rights reserved.
 *   Author: Masayuki Ishikawa <masayuki.ishikawa@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

#include <arch/board/board.h>

#include "sched/sched.h"
#include "irq/irq.h"

#include "riscv_arch.h"
#include "riscv_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_fault
 *
 * Description:
 *   This is Fault exception handler.
 *
 ****************************************************************************/

void up_fault(int irq, uint64_t *regs)
{
  CURRENT_REGS = regs;

  _alert("EPC:%016x\n",
         CURRENT_REGS[REG_EPC]);

  _alert("Fault IRQ=%d \n", irq);

  /* Dump register info */

  _alert("A0:%016x A1:%016x A2:%016x A3:%016x \n",
         CURRENT_REGS[REG_A0], CURRENT_REGS[REG_A1],
         CURRENT_REGS[REG_A2], CURRENT_REGS[REG_A3]);

  _alert("A4:%016x A5:%016x A6:%016x A7:%016x \n",
         CURRENT_REGS[REG_A4], CURRENT_REGS[REG_A5],
         CURRENT_REGS[REG_A6], CURRENT_REGS[REG_A7]);

  _alert("T0:%016x T1:%016x T2:%016x T3:%016x \n",
         CURRENT_REGS[REG_T0], CURRENT_REGS[REG_T1],
         CURRENT_REGS[REG_T2], CURRENT_REGS[REG_T3]);

  _alert("T4:%016x T5:%016x T6:%016x \n",
         CURRENT_REGS[REG_T4], CURRENT_REGS[REG_T5],
         CURRENT_REGS[REG_T6]);

  _alert("S0:%016x S1:%016x S2:%016x S3:%016x \n",
         CURRENT_REGS[REG_S0], CURRENT_REGS[REG_S1],
         CURRENT_REGS[REG_S2], CURRENT_REGS[REG_S3]);

  _alert("S4:%016x S5:%016x S6:%016x S7:%016x \n",
         CURRENT_REGS[REG_S4], CURRENT_REGS[REG_S5],
         CURRENT_REGS[REG_S6], CURRENT_REGS[REG_S7]);

  _alert("S8:%016x S9:%016x S10:%016x S11:%016x \n",
         CURRENT_REGS[REG_S8], CURRENT_REGS[REG_S9],
         CURRENT_REGS[REG_S10], CURRENT_REGS[REG_S11]);

#ifdef RISCV_SAVE_GP
  _alert("GP:%016x SP:%016x FP:%016x TP:%016x RA:%016x \n",
         CURRENT_REGS[REG_GP], CURRENT_REGS[REG_SP],
         CURRENT_REGS[REG_FP], CURRENT_REGS[REG_TP],
         CURRENT_REGS[REG_RA]);
#else
  _alert("SP:%016x FP:%016x TP:%016x RA:%016x \n",
         CURRENT_REGS[REG_SP], CURRENT_REGS[REG_FP],
         CURRENT_REGS[REG_TP], CURRENT_REGS[REG_RA]);
#endif

  up_irq_save();
}
