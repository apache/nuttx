/****************************************************************************
 * arch/mips/src/jz4780/jz4780_exception.c
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
#include <syscall.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>

#include <arch/board/board.h>
#include <arch/jz4780/cp0.h>

#include "mips_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: jz4780_exception
 *
 * Description:
 *   Called from assembly language logic on all other exceptions.
 *
 ****************************************************************************/

uint32_t *jz4780_exception(uint32_t *regs)
{
  uint32_t cause;
  uint32_t epc;

  /* If the board supports LEDs, turn on an LED now to indicate that we are
   * processing an interrupt.
   */

  board_autoled_on(LED_INIRQ);

  /* Get the cause of the exception from the CAUSE register */

  asm volatile("\tmfc0 %0,$13,0\n" : "=r"(cause));
  asm volatile("\tmfc0 %0,$14,0\n" : "=r"(epc));

  if ((cause & CP0_CAUSE_EXCCODE_MASK) == CP0_CAUSE_EXCCODE_SYS)
    {
      if (cause & CP0_CAUSE_BD)
        {
          /* Syscall in delay slot!
           * EPC points to the Branch. We need to skip the Branch (4)
           * AND the Syscall in the delay slot (4).
           */

          regs[REG_EPC] += 8;
        }
      else
        {
          /* Standard syscall. Just skip the syscall instruction itself. */

          regs[REG_EPC] += 4;
        }

      switch (regs[REG_A0])
        {
          /* A0=SYS_save_context:  This is a save context command:
           *
           *   int up_saveusercontext(void *saveregs);
           *
           * At this point, the following values are saved in context:
           *
           *   A0 = SYS_save_context
           *   A1 = saveregs
           *
           * In this case, we simply need to copy the current registers to
           * the save register space references in the saved R1 and return.
           */

          case SYS_save_context:
            {
              DEBUGASSERT(regs[REG_A1] != 0);
              mips_copystate((uint32_t *)regs[REG_A1], regs);
            }
            break;

          /* A0=SYS_restore_context: This a restore context command:
           *
           * void
           * up_fullcontextrestore(uint32_t *restoreregs) noreturn_function;
           *
           * At this point, the following values are saved in context:
           *
           *   A0 = SYS_restore_context
           *   A1 = restoreregs
           *
           * In this case, we simply need to set g_current_regs to restore
           * the register area referenced in the saved R1.
           * context == g_current_regs is the normal exception return.  By
           * setting g_current_regs equals to context[R1], we force the
           * return to the saved context referenced in R1.
           */

          case SYS_restore_context:
            {
              DEBUGASSERT(regs[REG_A1] != 0);
              up_set_current_regs((uint32_t *)regs[REG_A1]);
            }
            break;

          /* A0=SYS_switch_context: This a switch context command:
           *
           *   void mips_switchcontext(uint32_t *saveregs,
           *                           uint32_t *restoreregs);
           *
           * At this point, the following values are saved in context:
           *
           *   A0 = SYS_switch_context
           *   A1 = saveregs
           *   A2 = restoreregs
           *
           * In this case, we save the context registers to the save register
           * area referenced by the saved contents of A1 and then set
           * g_current_regs to the save register area referenced by the saved
           * contents of A2.
           */

          case SYS_switch_context:
            {
              DEBUGASSERT(regs[REG_A1] != 0 && regs[REG_A2] != 0);
              mips_copystate((uint32_t *)regs[REG_A1], regs);
              up_set_current_regs((uint32_t *)regs[REG_A2]);
            }
            break;

          default:
            {
              svcerr("ERROR: Bad SYS call: %" PRId32 "\n", regs[REG_A0]);
            }
            break;
        }

      regs = up_current_regs();
      up_set_current_regs(NULL);
      board_autoled_off(LED_INIRQ);
      return regs;
    }

#ifdef CONFIG_DEBUG_INFO
  switch (cause & CP0_CAUSE_EXCCODE_MASK)
    {
    case CP0_CAUSE_EXCCODE_INT:      /* Interrupt */
      _alert("EXCEPTION: Interrupt"
            " CAUSE: %x EPC: %x\n", cause, epc);
      break;
    case CP0_CAUSE_EXCCODE_TLBL:     /* TLB exception (load or instruction fetch) */
      _alert("EXCEPTION: TLB exception (load or instruction fetch)"
            " CAUSE: %x EPC:%x\n", cause, epc);
      break;
    case CP0_CAUSE_EXCCODE_TLBS:     /* TLB exception (store) */
      _alert("EXCEPTION: TLB exception (store)"
            " CAUSE: %x EPC: %x\n", cause, epc);
      break;
    case CP0_CAUSE_EXCCODE_ADEL:     /* Address error exception (load or instruction fetch) */
      _alert("EXCEPTION: Address error exception (load or instruction fetch)"
            " CAUSE: %x EPC: %x\n", cause, epc);
      break;
    case CP0_CAUSE_EXCCODE_ADES:     /* Address error exception (store) */
      _alert("EXCEPTION: Address error exception (store)"
            " CAUSE: %x EPC: %x\n", cause, epc);
      break;
    case CP0_CAUSE_EXCCODE_IBE:      /* Bus error exception (instruction fetch) */
      _alert("EXCEPTION: Bus error exception (instruction fetch)"
            " CAUSE: %x EPC: %x\n", cause, epc);
      break;
    case CP0_CAUSE_EXCCODE_DBE:      /* Bus error exception (data reference: load or store) */
      _alert("EXCEPTION: Bus error exception (data reference: load or store)"
            " CAUSE: %x EPC: %x\n", cause, epc);
      break;
    case CP0_CAUSE_EXCCODE_SYS:      /* Syscall exception */
      _alert("EXCEPTION: Syscall exception"
            " CAUSE: %x EPC: %x\n", cause, epc);
      break;
    case CP0_CAUSE_EXCCODE_BP:       /* Breakpoint exception */
      _alert("EXCEPTION: Breakpoint exception"
            " CAUSE: %x EPC: %x\n", cause, epc);
      break;
    case CP0_CAUSE_EXCCODE_RI:       /* Reserved instruction exception */
      _alert("EXCEPTION: Reserved instruction exception"
            " CAUSE: %x EPC: %x\n", cause, epc);
      break;
    case CP0_CAUSE_EXCCODE_CPU:      /* Coprocessor Unusable exception */
      _alert("EXCEPTION: Coprocessor Unusable exception"
            " CAUSE: %x EPC: %x\n", cause, epc);
      break;
    case CP0_CAUSE_EXCCODE_OV:       /* Arithmetic Overflow exception */
      _alert("EXCEPTION: Arithmetic Overflow exception"
            " CAUSE: %x EPC: %x\n", cause, epc);
      break;
    case CP0_CAUSE_EXCCODE_TR:       /* Trap exception */
      _alert("EXCEPTION: Trap exception"
            " CAUSE: %x EPC: %x\n", cause, epc);
      break;
    case CP0_CAUSE_EXCCODE_FPE:      /* Floating point exception */
      _alert("EXCEPTION: Floating point exception"
            " CAUSE: %x EPC: %x\n", cause, epc);
      break;
    case CP0_CAUSE_EXCCODE_C2E:      /* Precise Coprocessor 2 exceptions */
      _alert("EXCEPTION: Precise Coprocessor 2 exceptions"
            " CAUSE: %x EPC: %x\n", cause, epc);
      break;
    case CP0_CAUSE_EXCCODE_MDMX:     /* MDMX Unusable (MIPS64) */
      _alert("EXCEPTION: MDMX Unusable (MIPS64)"
            " CAUSE: %x EPC: %x\n", cause, epc);
      break;
    case CP0_CAUSE_EXCCODE_WATCH:    /* WatchHi/WatchLo address */
      _alert("EXCEPTION: WatchHi/WatchLo address"
            " CAUSE: %x EPC: %x\n", cause, epc);
      break;
    case CP0_CAUSE_EXCCODE_MCHECK:   /* Machine check */
      _alert("EXCEPTION: Machine check"
            " CAUSE: %x EPC: %x\n", cause, epc);
      break;
    case CP0_CAUSE_EXCCODE_CACHEERR: /* Cache error */
      _alert("EXCEPTION: Cache error"
            " CAUSE: %x EPC: %x\n", cause, epc);
      break;
    default:
      _alert("EXCEPTION: Unknown"
            " CAUSE: %x EPC: %x\n", cause, epc);
      break;
    }
#else
  _alert("EXCEPTION: CAUSE: %x EPC: %x\n", cause, epc);
#endif
  up_dump_register(regs);

  /* Crash with currents_regs set so that we can dump the register
   * contents.
   */

  up_set_current_regs(regs);
  PANIC_WITH_REGS("panic", regs);
  return regs; /* Won't get here */
}
