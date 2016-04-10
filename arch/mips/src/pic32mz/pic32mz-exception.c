/****************************************************************************
 * arch/mips/src/pic32mz/pic32mz-exception.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>

#include <arch/board/board.h>
#include <arch/pic32mz/cp0.h>

#include "up_arch.h"
#include "up_internal.h"

#include "chip/pic32mz-int.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/************************************************************************************
 * Name: pic32mz_exception
 *
 * Description:
 *   Called from assembly language logic on all other exceptions.
 *
 ************************************************************************************/

uint32_t *pic32mz_exception(uint32_t *regs)
{
#ifdef CONFIG_DEBUG
  uint32_t cause;
  uint32_t epc;
#endif

  /* If the board supports LEDs, turn on an LED now to indicate that we are
   * processing an interrupt.
   */

  board_autoled_on(LED_INIRQ);

#ifdef CONFIG_DEBUG
  /* Get the cause of the exception from the CAUSE register */

  asm volatile("\tmfc0 %0,$13,0\n" : "=r"(cause));
  asm volatile("\tmfc0 %0,$14,0\n" : "=r"(epc));

#ifdef CONFIG_DEBUG_VERBOSE
  switch (cause & CP0_CAUSE_EXCCODE_MASK)
    {
    case CP0_CAUSE_EXCCODE_INT:      /* Interrupt */
      llvdbg("EXCEPTION: Interrupt"
             " CAUSE: %08x EPC: %08x\n", cause, epc);
      break;
    case CP0_CAUSE_EXCCODE_TLBL:     /* TLB exception (load or instruction fetch) */
      llvdbg("EXCEPTION: TLB exception (load or instruction fetch)"
             " CAUSE: %08x EPC:%08x\n", cause, epc);
      break;
    case CP0_CAUSE_EXCCODE_TLBS:     /* TLB exception (store) */
      llvdbg("EXCEPTION: TLB exception (store)"
             " CAUSE: %08x EPC: %08x\n", cause, epc);
      break;
    case CP0_CAUSE_EXCCODE_ADEL:     /* Address error exception (load or instruction fetch) */
      llvdbg("EXCEPTION: Address error exception (load or instruction fetch)"
             " CAUSE: %08x EPC: %08x\n", cause, epc);
      break;
    case CP0_CAUSE_EXCCODE_ADES:     /* Address error exception (store) */
      llvdbg("EXCEPTION: Address error exception (store)"
             " CAUSE: %08x EPC: %08x\n", cause, epc);
      break;
    case CP0_CAUSE_EXCCODE_IBE:      /* Bus error exception (instruction fetch) */
      llvdbg("EXCEPTION: Bus error exception (instruction fetch)"
             " CAUSE: %08x EPC: %08x\n", cause, epc);
      break;
    case CP0_CAUSE_EXCCODE_DBE:      /* Bus error exception (data reference: load or store) */
      llvdbg("EXCEPTION: Bus error exception (data reference: load or store)"
             " CAUSE: %08x EPC: %08x\n", cause, epc);
      break;
    case CP0_CAUSE_EXCCODE_SYS:      /* Syscall exception */
      llvdbg("EXCEPTION: Syscall exception"
             " CAUSE: %08x EPC: %08x\n", cause, epc);
      break;
    case CP0_CAUSE_EXCCODE_BP:       /* Breakpoint exception */
      llvdbg("EXCEPTION: Breakpoint exception"
             " CAUSE: %08x EPC: %08x\n", cause, epc);
      break;
    case CP0_CAUSE_EXCCODE_RI:       /* Reserved instruction exception */
      llvdbg("EXCEPTION: Reserved instruction exception"
             " CAUSE: %08x EPC: %08x\n", cause, epc);
      break;
    case CP0_CAUSE_EXCCODE_CPU:      /* Coprocessor Unusable exception */
      llvdbg("EXCEPTION: Coprocessor Unusable exception"
             " CAUSE: %08x EPC: %08x\n", cause, epc);
      break;
    case CP0_CAUSE_EXCCODE_OV:       /* Arithmetic Overflow exception */
      llvdbg("EXCEPTION: Arithmetic Overflow exception"
             " CAUSE: %08x EPC: %08x\n", cause, epc);
      break;
    case CP0_CAUSE_EXCCODE_TR:       /* Trap exception */
      llvdbg("EXCEPTION: Trap exception"
             " CAUSE: %08x EPC: %08x\n", cause, epc);
      break;
    case CP0_CAUSE_EXCCODE_FPE:      /* Floating point exception */
      llvdbg("EXCEPTION: Floating point exception"
             " CAUSE: %08x EPC: %08x\n", cause, epc);
      break;
    case CP0_CAUSE_EXCCODE_C2E:      /* Precise Coprocessor 2 exceptions */
      llvdbg("EXCEPTION: Precise Coprocessor 2 exceptions"
             " CAUSE: %08x EPC: %08x\n", cause, epc);
      break;
    case CP0_CAUSE_EXCCODE_MDMX:     /* MDMX Unusable (MIPS64) */
      llvdbg("EXCEPTION: MDMX Unusable (MIPS64)"
             " CAUSE: %08x EPC: %08x\n", cause, epc);
      break;
    case CP0_CAUSE_EXCCODE_WATCH:    /* WatchHi/WatchLo address */
      llvdbg("EXCEPTION: WatchHi/WatchLo address"
             " CAUSE: %08x EPC: %08x\n", cause, epc);
      break;
    case CP0_CAUSE_EXCCODE_MCHECK:   /* Machine check */
      llvdbg("EXCEPTION: Machine check"
             " CAUSE: %08x EPC: %08x\n", cause, epc);
      break;
    case CP0_CAUSE_EXCCODE_CACHEERR: /* Cache error */
      llvdbg("EXCEPTION: Cache error"
             " CAUSE: %08x EPC: %08x\n", cause, epc);
      break;
    default:
      llvdbg("EXCEPTION: Unknown"
             " CAUSE: %08x EPC: %08x\n", cause, epc);
      break;
    }
#else
  lldbg("EXCEPTION: CAUSE: %08x EPC: %08x\n", cause, epc);
#endif
#endif

  /* Crash with currents_regs set so that we can dump the register contents. */

  g_current_regs = regs;
  PANIC();
  return regs; /* Won't get here */
}
