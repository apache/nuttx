/****************************************************************************
 * arch/mips/src/pic32mx/pic32mx_exception.c
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
#include <nuttx/board.h>

#include <arch/board/board.h>
#include <arch/pic32mx/cp0.h>

#include "mips_internal.h"
#include "pic32mx_int.h"
#include "pic32mx.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mx_exception
 *
 * Description:
 *   Called from assembly language logic on all other exceptions.
 *
 ****************************************************************************/

uint32_t *pic32mx_exception(uint32_t *regs)
{
#ifdef CONFIG_DEBUG_FEATURES
  uint32_t cause;
  uint32_t epc;
#endif

  /* If the board supports LEDs, turn on an LED now to indicate that we are
   * processing an interrupt.
   */

  board_autoled_on(LED_INIRQ);

#ifdef CONFIG_DEBUG_FEATURES
  /* Get the cause of the exception from the CAUSE register */

  asm volatile("\tmfc0 %0,$13,0\n" : "=r"(cause));
  asm volatile("\tmfc0 %0,$14,0\n" : "=r"(epc));

#ifdef CONFIG_DEBUG_INFO
  switch (cause & CP0_CAUSE_EXCCODE_MASK)
    {
    case CP0_CAUSE_EXCCODE_INT:      /* Interrupt */
      _alert("EXCEPTION: Interrupt"
            " CAUSE: %08x EPC: %08x\n", cause, epc);
      break;
    case CP0_CAUSE_EXCCODE_TLBL:     /* TLB exception (load or instruction fetch) */
      _alert("EXCEPTION: TLB exception (load or instruction fetch)"
            " CAUSE: %08x EPC:%08x\n", cause, epc);
      break;
    case CP0_CAUSE_EXCCODE_TLBS:     /* TLB exception (store) */
      _alert("EXCEPTION: TLB exception (store)"
            " CAUSE: %08x EPC: %08x\n", cause, epc);
      break;
    case CP0_CAUSE_EXCCODE_ADEL:     /* Address error exception (load or instruction fetch) */
      _alert("EXCEPTION: Address error exception (load or instruction fetch)"
            " CAUSE: %08x EPC: %08x\n", cause, epc);
      break;
    case CP0_CAUSE_EXCCODE_ADES:     /* Address error exception (store) */
      _alert("EXCEPTION: Address error exception (store)"
            " CAUSE: %08x EPC: %08x\n", cause, epc);
      break;
    case CP0_CAUSE_EXCCODE_IBE:      /* Bus error exception (instruction fetch) */
      _alert("EXCEPTION: Bus error exception (instruction fetch)"
            " CAUSE: %08x EPC: %08x\n", cause, epc);
      break;
    case CP0_CAUSE_EXCCODE_DBE:      /* Bus error exception (data reference: load or store) */
      _alert("EXCEPTION: Bus error exception (data reference: load or store)"
            " CAUSE: %08x EPC: %08x\n", cause, epc);
      break;
    case CP0_CAUSE_EXCCODE_SYS:      /* Syscall exception */
      _alert("EXCEPTION: Syscall exception"
            " CAUSE: %08x EPC: %08x\n", cause, epc);
      break;
    case CP0_CAUSE_EXCCODE_BP:       /* Breakpoint exception */
      _alert("EXCEPTION: Breakpoint exception"
            " CAUSE: %08x EPC: %08x\n", cause, epc);
      break;
    case CP0_CAUSE_EXCCODE_RI:       /* Reserved instruction exception */
      _alert("EXCEPTION: Reserved instruction exception"
            " CAUSE: %08x EPC: %08x\n", cause, epc);
      break;
    case CP0_CAUSE_EXCCODE_CPU:      /* Coprocessor Unusable exception */
      _alert("EXCEPTION: Coprocessor Unusable exception"
            " CAUSE: %08x EPC: %08x\n", cause, epc);
      break;
    case CP0_CAUSE_EXCCODE_OV:       /* Arithmetic Overflow exception */
      _alert("EXCEPTION: Arithmetic Overflow exception"
            " CAUSE: %08x EPC: %08x\n", cause, epc);
      break;
    case CP0_CAUSE_EXCCODE_TR:       /* Trap exception */
      _alert("EXCEPTION: Trap exception"
            " CAUSE: %08x EPC: %08x\n", cause, epc);
      break;
    case CP0_CAUSE_EXCCODE_FPE:      /* Floating point exception */
      _alert("EXCEPTION: Floating point exception"
            " CAUSE: %08x EPC: %08x\n", cause, epc);
      break;
    case CP0_CAUSE_EXCCODE_C2E:      /* Precise Coprocessor 2 exceptions */
      _alert("EXCEPTION: Precise Coprocessor 2 exceptions"
            " CAUSE: %08x EPC: %08x\n", cause, epc);
      break;
    case CP0_CAUSE_EXCCODE_MDMX:     /* MDMX Unusable (MIPS64) */
      _alert("EXCEPTION: MDMX Unusable (MIPS64)"
            " CAUSE: %08x EPC: %08x\n", cause, epc);
      break;
    case CP0_CAUSE_EXCCODE_WATCH:    /* WatchHi/WatchLo address */
      _alert("EXCEPTION: WatchHi/WatchLo address"
            " CAUSE: %08x EPC: %08x\n", cause, epc);
      break;
    case CP0_CAUSE_EXCCODE_MCHECK:   /* Machine check */
      _alert("EXCEPTION: Machine check"
            " CAUSE: %08x EPC: %08x\n", cause, epc);
      break;
    case CP0_CAUSE_EXCCODE_CACHEERR: /* Cache error */
      _alert("EXCEPTION: Cache error"
            " CAUSE: %08x EPC: %08x\n", cause, epc);
      break;
    default:
      _alert("EXCEPTION: Unknown"
            " CAUSE: %08x EPC: %08x\n", cause, epc);
      break;
    }
#else
  _alert("EXCEPTION: CAUSE: %08x EPC: %08x\n", cause, epc);
#endif
#endif

  /* Crash with currents_regs set so that we can dump the register
   * contents.
   */

  CURRENT_REGS = regs;
  PANIC();
  return regs; /* Won't get here */
}
