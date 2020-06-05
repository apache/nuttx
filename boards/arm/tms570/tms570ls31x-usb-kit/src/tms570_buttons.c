/****************************************************************************
 * boards/arm/tms570/tms570ls31x-usb-kit/src/tms570_buttons.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/irq.h>

#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "arm_arch.h"
#include "tms570_gio.h"
#include "tms570ls31x_usb_kit.h"

#ifdef CONFIG_ARCH_BUTTONS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#undef HAVE_IRQBUTTONS
#if defined(CONFIG_ARCH_IRQBUTTONS) && defined(CONFIG_TMS570_GIO_IRQ)
#  define HAVE_IRQBUTTONS 1
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef HAVE_IRQBUTTONS
static xcpt_t g_irq_button;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_button_irqx
 *
 * Description:
 *   This function implements the core of the board_button_irq() logic.
 *
 ****************************************************************************/

#ifdef HAVE_IRQBUTTONS
static int board_button_irqx(gio_pinset_t pinset, int irq,
                             xcpt_t irqhandler, xcpt_t *store, void *arg)
{
  irqstate_t flags;

  /* Disable interrupts until we are done.  This guarantees that the
   * following operations are atomic.
   */

  flags = enter_critical_section();

  /* Get the old button interrupt handler and save the new one */

  *store = irqhandler;

  /* Are we attaching or detaching? */

  if (irqhandler != NULL)
    {
      /* Configure the interrupt */

      tms570_gioirq(pinset);
      irq_attach(irq, irqhandler, arg);
      tms570_gioirqenable(irq);
    }
  else
    {
      /* Detach and disable the interrupt */

      irq_detach(irq);
      tms570_gioirqdisable(irq);
    }

  leave_critical_section(flags);

  /* Return the old button handler (so that it can be restored) */

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_button_initialize
 *
 * Description:
 *   board_button_initialize() must be called to initialize button resources.
 *   After that, board_buttons() may be called to collect the current state
 *   of all buttons or board_button_irq() may be called to register button
 *   interrupt handlers.
 *
 ****************************************************************************/

uint32_t board_button_initialize(void)
{
  /* Configure button GIOs */

  tms570_configgio(GIO_BUTTON);
  return NUM_BUTTONS;
}

/****************************************************************************
 * Name: board_buttons
 *
 * Description:
 *   After board_button_initialize() has been called, board_buttons() may be
 *   called to collect the state of all buttons.  board_buttons() returns an
 *   32-bit bit set with each bit associated with a button.  See the BUTTON*
 *   definitions above for the meaning of each bit in the returned value.
 *
 ****************************************************************************/

uint32_t board_buttons(void)
{
  return tms570_gioread(GIO_BUTTON) ? BUTTON_GIOA7_BIT : 0;
}

/****************************************************************************
 * Name: board_button_irq
 *
 * Description:
 *   This function may be called to register an interrupt handler that will
 *   be called when a button is depressed or released.  The ID value is one
 *   of the BUTTON* definitions provided above.
 *
 * Configuration Notes:
 *   Configuration CONFIG_AVR32_GIOIRQ must be selected to enable the
 *   overall GIO IRQ feature and CONFIG_AVR32_GIOIRQSETA and/or
 *   CONFIG_AVR32_GIOIRQSETB must be enabled to select GIOs to support
 *   interrupts on.  For button support, bits 2 and 3 must be set in
 *   CONFIG_AVR32_GIOIRQSETB (PB2 and PB3).
 *
 ****************************************************************************/

int board_button_irq(int id, xcpt_t irqhandler, FAR void *arg)
{
#ifdef HAVE_IRQBUTTONS
  if (id == BUTTON_GIOA7)
    {
      return board_button_irqx(GIO_BUTTON, IRQ_BUTTON, irqhandler,
                               &g_irq_button, arg);
    }
#endif

  return -EINVAL;
}

#endif /* CONFIG_ARCH_BUTTONS */
