/****************************************************************************
 * arch/z80/src/common/z80_doirq.c
 *
 *   Copyright (C) 2007-2009, 2014-2015 Gregory Nutt. All rights reserved.
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
#include "up_arch.h"

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>

#include <arch/irq.h>

#include "chip/switch.h"
#include "z80_internal.h"

#include "group/group.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR chipreg_t *z80_doirq(uint8_t irq, FAR chipreg_t *regs)
{
  board_autoled_on(LED_INIRQ);

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
      DECL_SAVESTATE();

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

          group_addrenv(NULL);
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
