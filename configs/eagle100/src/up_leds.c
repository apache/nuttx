/****************************************************************************
 * configs/eagle100/src/up_leds.c
 * arch/arm/src/board/up_leds.c
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
#include <sys/types.h>

#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"
#include "lm3s_internal.h"

/****************************************************************************
 * Definitions
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

/****************************************************************************
 * Name: up_ledinit
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void up_ledinit(void)
{
  /* Make sure that the GPIOE peripheral is enabled */

  modifyreg32(LM3S_SYSCON_RCGC2_OFFSET, 0, SYSCON_RCGC2_GPIOE);

  /* Configure Port E, Bit 1 as an output, initial value=OFF */

  lm3s_configgpio(GPIO_FUNC_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORTE | 1);
}

/****************************************************************************
 * Name: up_ledon
 ****************************************************************************/

void up_ledon(int led)
{
  switch (led)
    {
      case LED_STARTED:
      case LED_HEAPALLOCATE:
      default:
        break;
      case LED_IRQSENABLED:
      case LED_STACKCREATED:
      case LED_INIRQ:
      case LED_SIGNAL:
      case LED_ASSERTION:
      case LED_PANIC:
        modifyreg32(LM3S_GPIOE_DATA, 0, (1 << 1));
        break;
    }
}

/****************************************************************************
 * Name: up_ledoff
 ****************************************************************************/

void up_ledoff(int led)
{
  switch (led)
    {
      case LED_IRQSENABLED:
      case LED_STACKCREATED:
      default:
        break;

      case LED_STARTED:
      case LED_HEAPALLOCATE:
      case LED_INIRQ:
      case LED_SIGNAL:
      case LED_ASSERTION:
      case LED_PANIC:
        modifyreg32(LM3S_GPIOE_DATA, (1 << 1), 0);
        break;
    }
}

#endif /* CONFIG_ARCH_LEDS */
