/****************************************************************************
 * arch/z80/src/common/z80_doirq.c
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

#include <nuttx/addrenv.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>

#include <arch/irq.h>

#include "chip/switch.h"
#include "z80_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR chipreg_t *z80_doirq(uint8_t irq, FAR chipreg_t *regs)
{
  board_autoled_on(LED_INIRQ);

  DECL_SAVESTATE();

#ifdef CONFIG_SUPPRESS_INTERRUPTS

  IRQ_ENTER(regs);
  err("ERROR: Unexpected IRQ\n");
  PANIC();
  return NULL; /* Won't get here */

#else
#ifdef CONFIG_ARCH_ADDRENV
  FAR chipreg_t *newregs;
#endif

  if (irq < NR_IRQS)
    {
      /* Indicate that we have entered IRQ processing logic */

      IRQ_ENTER(irq, regs);

      /* Deliver the IRQ */

      irq_dispatch(irq, regs);

#ifdef CONFIG_ARCH_ADDRENV
      /* If a context switch occurred, 'newregs' will hold the new context */

      newregs = IRQ_STATE();

      if (newregs != regs)
        {
          /* Make sure that the address environment for the previously
           * running task is closed down gracefully and set up the
           * address environment for the new thread at the head of the
           * ready-to-run list.
           */

          addrenv_switch(NULL);
        }

      regs = newregs;

#else
      /* If a context switch occurred, 'regs' will hold the new context */

      regs = IRQ_STATE();
#endif

      /* Indicate that we are no longer in interrupt processing logic */

      IRQ_LEAVE(irq);
    }

  board_autoled_off(LED_INIRQ);
  return regs;
#endif
}
