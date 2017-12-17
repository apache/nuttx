/****************************************************************************
 * configs/zkit-arm-1769/src/lpc17_buttons.c
 *
 *   Copyright (C)  2011 Zilogic Systems. All rights reserved.
 *   Author: Kannan <code@zilogic.com>
 *
 * Based on configs/stm3210e-eval/src/board_buttons.c
 *
 *   Copyright (C) 2009, 2011, 2017 Gregory Nutt. All rights reserved.
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

#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "up_internal.h"
#include "lpc17_gpio.h"
#include "zkit-arm-1769.h"

#ifdef CONFIG_ARCH_BUTTONS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* Pin configuration for each zkit-arm-1769 button.  This array is indexed by
 * NUM_BUTTONS in board.h
 */

static const uint16_t g_buttons[NUM_BUTTONS] =
{
  ZKITARM_KEY1, ZKITARM_KEY2, ZKITARM_KEY3, ZKITARM_KEY4, ZKITARM_KEY5
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_button_initialize
 *
 * Description:
 *   board_button_initialize() must be called to initialize button resources.  After
 *   that, board_buttons() may be called to collect the current state of all
 *   buttons or board_button_irq() may be called to register button interrupt
 *   handlers.
 *
 ****************************************************************************/

void board_button_initialize(void)
{
  int i;

  /* Configure the GPIO pins as inputs.  NOTE that EXTI interrupts are
   * configured for some pins but NOT used in this file
   */

  for (i = 0; i < NUM_BUTTONS; i++)
    {
      lpc17_configgpio(g_buttons[i]);
    }
}

/****************************************************************************
 * Name: board_buttons
 ****************************************************************************/

uint32_t board_buttons(void)
{
  uint32_t ret = 0;
  bool released;
  int i;

  /* Check that state of each key */

  for (i = 0; i < NUM_BUTTONS; i++)
    {
      released = lpc17_gpioread(g_buttons[i]);

       /* Accumulate set of depressed keys */

      if (!released)
        {
          ret |= (1 << i);
        }
    }

  return ret;
}

/************************************************************************************
 * Button support.
 *
 * Description:
 *   board_button_initialize() must be called to initialize button resources.  After
 *   that, board_buttons() may be called to collect the current state of all
 *   buttons or board_button_irq() may be called to register button interrupt
 *   handlers.
 *
 *   After board_button_initialize() has been called, board_buttons() may be called to
 *   collect the state of all buttons.  board_buttons() returns an 32-bit bit set
 *   with each bit associated with a button.  See the BUTTON_*_BIT and JOYSTICK_*_BIT
 *   definitions in board.h for the meaning of each bit.
 *
 *   board_button_irq() may be called to register an interrupt handler that will
 *   be called when a button is depressed or released.  The ID value is a
 *   button enumeration value that uniquely identifies a button resource. See the
 *   BUTTON_* and JOYSTICK_* definitions in board.h for the meaning of enumeration
 *   value.
 *
 ************************************************************************************/

#if defined CONFIG_ARCH_IRQBUTTONS && CONFIG_LPC17_GPIOIRQ
int board_button_irq(int id, xcpt_t irqhandler, FAR void *arg)
{
  irqstate_t flags;
  int ret = -EINVAL;

  /* Interrupts are supported on KEY5 only */

  if (id == BOARD_BUTTON_5)
    {
      flags = enter_critical_section();

      /* Attach or detach the interrupt handler for KEY5. */

      if (irqhandler)
        {
          /* Configure KEY5 as an interrupting input */

          lpc17_configgpio(ZKITARM_INT_KEY5);

          /* Attach the new interrupt handler and enable the interrupt */

          ret = irq_attach(ZKITARM_KEY5_IRQ, irqhandler, arg);
          if (ret == OK)
            {
              up_enable_irq(ZKITARM_KEY5_IRQ);
            }
        }
      else
        {
          /* Disable the interrupt and detach the handler */

          up_disable_irq(ZKITARM_KEY5_IRQ);
          (void)irq_detach(ZKITARM_KEY5_IRQ);

          /* Configure KEY5 as a non-interrupting input */

          lpc17_configgpio(ZKITARM_KEY5);
          ret = OK;
        }

      leave_critical_section(flags);
    }

  return ret;
}

#endif
#endif /* CONFIG_ARCH_BUTTONS */
