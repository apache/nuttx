/****************************************************************************
 * arch/arm64/src/common/arm64_fatal.c
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

#include <sys/types.h>
#include <stdint.h>

#include <arch/irq.h>
#include <debug.h>
#include <assert.h>
#include <sched.h>
#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/tls.h>
#include <nuttx/board.h>
#include <arch/chip/chip.h>
#include <nuttx/syslog/syslog.h>
#include "sched/sched.h"
#include "irq/irq.h"
#include "arm64_arch.h"
#include "arm64_internal.h"
#include "arm64_fatal.h"
#include "arm64_mmu.h"
#include "arm64_fatal.h"
#include "arm64_arch_timer.h"

#ifdef CONFIG_ARCH_FPU
#include "arm64_fpu.h"
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: print_ec_cause
 ****************************************************************************/

static void print_ec_cause(uint64_t esr)
{
  uint32_t ec = (uint32_t)esr >> 26;

  switch (ec)
    {
      case 0b000000:
        {
          sinfo("Unknown reason\n");
          break;
        }

      case 0b000001:
        {
          sinfo("Trapped WFI or WFE instruction execution\n");
          break;
        }

      case 0b000011:
        {
          sinfo(
             "Trapped MCR or MRC access with (coproc==0b1111) that "
             "is not reported using EC 0b000000\n");
          break;
        }

      case 0b000100:
        {
          sinfo(
             "Trapped MCRR or MRRC access with (coproc==0b1111) "
             "that is not reported using EC 0b000000\n");
          break;
        }

      case 0b000101:
        {
          sinfo("Trapped MCR or MRC access with (coproc==0b1110)\n");
          break;
        }

      case 0b000110:
        {
          sinfo("Trapped LDC or STC access\n");
          break;
        }

      case 0b000111:
        {
          sinfo(
             "Trapped access to SVE, Advanced SIMD, or "
             "floating-point functionality\n");
          break;
        }

      case 0b001100:
        {
          sinfo("Trapped MRRC access with (coproc==0b1110)\n");
          break;
        }

      case 0b001101:
        {
          sinfo("Branch Target Exception\n");
          break;
        }

      case 0b001110:
        {
          sinfo("Illegal Execution state\n");
          break;
        }

      case 0b010001:
        {
          sinfo("SVC instruction execution in AArch32 state\n");
          break;
        }

      case 0b011000:
        {
          sinfo(
             "Trapped MSR, MRS or System instruction execution in "
             "AArch64 state, that is not reported using EC "
             "0b000000, 0b000001 or 0b000111\n");
          break;
        }

      case 0b011001:
        {
          sinfo("Trapped access to SVE functionality\n");
          break;
        }

      case 0b100000:
        {
          sinfo(
             "Instruction Abort from a lower Exception level, that "
             "might be using AArch32 or AArch64\n");
          break;
        }

      case 0b100001:
        {
          sinfo(
             "Instruction Abort taken without a change "
             "in Exception level.\n");
          break;
        }

      case 0b100010:
        {
          sinfo("PC alignment fault exception.\n");
          break;
        }

      case 0b100100:
        {
          sinfo(
             "Data Abort from a lower Exception level, that might "
             "be using AArch32 or AArch64\n");
          break;
        }

      case 0b100101:
        {
          sinfo("Data Abort taken without a change in Exception level\n");
          break;
        }

      case 0b100110:
        {
          sinfo("SP alignment fault exception\n");
          break;
        }

      case 0b101000:
        {
          sinfo(
             "Trapped floating-point exception "
             "taken from AArch32 state\n");
          break;
        }

      case 0b101100:
        {
          sinfo(
             "Trapped floating-point exception "
             "taken from AArch64 state.\n");
          break;
        }

      case 0b101111:
        {
          sinfo("SError interrupt\n");
          break;
        }

      case 0b110000:
        {
          sinfo(
             "Breakpoint exception from a lower Exception level, "
             "that might be using AArch32 or AArch64\n");
          break;
        }

      case 0b110001:
        {
          sinfo(
             "Breakpoint exception taken without a change in "
             "Exception level\n");
          break;
        }

      case 0b110010:
        {
          sinfo(
             "Software Step exception from a lower Exception level, "
             "that might be using AArch32 or AArch64\n");
          break;
        }

      case 0b110011:
        {
          sinfo(
             "Software Step exception taken without a change in "
             "Exception level\n");
          break;
        }

      case 0b110100:
        {
          sinfo(
             "Watchpoint exception from a lower Exception level, "
             "that might be using AArch32 or AArch64\n");
          break;
        }

      case 0b110101:
        {
          sinfo(
             "Watchpoint exception taken without a change in "
             "Exception level.\n");
          break;
        }

      case 0b111000:
        {
          sinfo("BKPT instruction execution in AArch32 state\n");
          break;
        }

      case 0b111100:
        {
          sinfo("BRK instruction execution in AArch64 state.\n");
          break;
        }

      default:
        break;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_mdelay
 ****************************************************************************/

void up_mdelay(unsigned int milliseconds)
{
  volatile unsigned int i;
  volatile unsigned int j;

  for (i = 0; i < milliseconds; i++)
    {
      for (j = 0; j < CONFIG_BOARD_LOOPSPERMSEC; j++)
        {
        }
    }
}

/****************************************************************************
 * Name: arm64_fatal_error
 *
 * Description:
 *
 ****************************************************************************/

void arm64_fatal_error(unsigned int reason, struct regs_context * reg)
{
  uint64_t el, esr, elr, far;

  sinfo("reason = %d\n", reason);

  CURRENT_REGS = (uint64_t *)reg;

  if (reason != K_ERR_SPURIOUS_IRQ)
    {
      __asm__ volatile ("mrs %0, CurrentEL" : "=r" (el));

      switch (GET_EL(el))
        {
          case MODE_EL1:
            {
              sinfo("CurrentEL: MODE_EL1\n");
              __asm__ volatile ("mrs %0, esr_el1" : "=r" (esr));
              __asm__ volatile ("mrs %0, far_el1" : "=r" (far));
              __asm__ volatile ("mrs %0, elr_el1" : "=r" (elr));
              break;
            }

          case MODE_EL2:
            {
              sinfo("CurrentEL: MODE_EL2\n");
              __asm__ volatile ("mrs %0, esr_el2" : "=r" (esr));
              __asm__ volatile ("mrs %0, far_el2" : "=r" (far));
              __asm__ volatile ("mrs %0, elr_el2" : "=r" (elr));
              break;
            }

#ifdef CONFIG_ARCH_HAVE_EL3
          case MODE_EL3:
            {
              sinfo("CurrentEL: MODE_EL3\n");
              __asm__ volatile ("mrs %0, esr_el3" : "=r" (esr));
              __asm__ volatile ("mrs %0, far_el3" : "=r" (far));
              __asm__ volatile ("mrs %0, elr_el3" : "=r" (elr));
              break;
            }
#endif

          default:
            {
              sinfo("CurrentEL: unknown\n");

              /* Just to keep the compiler happy */

              esr = elr = far = 0;
              break;
            }
        }

      if (GET_EL(el) != MODE_EL0)
        {
          sinfo("ESR_ELn: 0x%"PRIx64"\n", esr);
          sinfo("FAR_ELn: 0x%"PRIx64"\n", far);
          sinfo("ELR_ELn: 0x%"PRIx64"\n", elr);

          print_ec_cause(esr);
        }
    }

  PANIC_WITH_REGS("panic", reg);
}
