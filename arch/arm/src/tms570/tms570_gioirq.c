/****************************************************************************
 * arch/arm/src/tms570/tms570_gioirq.c
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
#include <errno.h>
#include <debug.h>

#include <nuttx/init.h>
#include <nuttx/arch.h>

#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "up_internal.h"

#include "tms570_gio.h"
#include "chip/tms570_gio.h"

#ifdef CONFIG_TMS570_GIO_IRQ

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tms570_gio_interrupt
 *
 * Description:
 *   Receive GIO interrupts
 *
 ****************************************************************************/

static int tms3570_gio_interrupt(int irq, void *context, FAR void *arg)
{
  uint32_t off1;
  int irq2;

  /* Loop until all pending GIO interrupts have been processed */

  while ((off1 = getreg32(TMS570_GIO_OFF1)) != GIO_OFF_NONE)
    {
      /* Convert the offset value to the second-level IRQ number */

      irq2 = off1 + TMS570_IRQ_GIOA0 - 1;

      /* And dispatch the second-level GIO IRQ */

      irq_dispatch(irq2, context);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tms570_gioirqinitialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for
 *   GIO pins.
 *
 ****************************************************************************/

void tms570_gioirq_initialize(void)
{
  /* Disable all pin interrupts on the pin.  Make sure they are all level 0. */

  putreg32(0xffffffff, TMS570_GIO_ENACLR);
  putreg32(0xffffffff, TMS570_GIO_LVLCLR);

  /* Attach and enable the GIO level 0 interrupt */

  DEBUGVERIFY(irq_attach(TMS570_REQ_GIO_0, tms3570_gio_interrupt, NULL));
  up_enable_irq(TMS570_REQ_GIO_0);
}

/************************************************************************************
 * Name: tms570_gioirq
 *
 * Description:
 *   Configure an interrupt for the specified GIO pin.
 *
 ************************************************************************************/

void tms570_gioirq(gio_pinset_t pinset)
{
  uint32_t port = tms570_gio_port(pinset);
  uint32_t pin = tms570_gio_pin(pinset);
  irqstate_t flags;
  uint32_t regval;

  /* Start with the pin interrupts disabled.  Make sure that level 0 is selected. */

  putreg32(GIO_ENACLR_PORT_PIN(port, pin), TMS570_GIO_ENACLR);
  putreg32(GIO_LVLCLR_PORT_PIN(port, pin), TMS570_GIO_LVLCLR);

  /* Make sure that the pin is configured as an input and that interrupts can e
   * supported on this port.
   */

  if ((pinset & GIO_MODE_MASK) == GIO_INPUT && port < TMS570_NIRQPORTS)
    {
      flags = enter_critical_section();
      switch (pinset & GIO_INT_MASK)
      {
        case GIO_INT_NONE:
        default:
          break;

        case GIO_INT_RISING:
          {
             /* Enable rising edge detectioni */

             regval  = getreg32(TMS570_GIO_POL);
             regval |= GIO_POL_PORT_PIN(port, pin);
             putreg32(regval, TMS570_GIO_POL);

             /* Disable both rising and falling edge detection */

             regval  = getreg32(TMS570_GIO_INTDET);
             regval &= ~GIO_INTDET_PORT_PIN(port, pin);
             putreg32(regval, TMS570_GIO_INTDET);
          }
          break;

        case GIO_INT_FALLING:
          {
             /* Enable falling edge detectioni */

             regval  = getreg32(TMS570_GIO_POL);
             regval &= ~GIO_POL_PORT_PIN(port, pin);
             putreg32(regval, TMS570_GIO_POL);

             /* Disable both rising and falling edge detection */

             regval  = getreg32(TMS570_GIO_INTDET);
             regval &= ~GIO_INTDET_PORT_PIN(port, pin);
             putreg32(regval, TMS570_GIO_INTDET);
          }
          break;

        case GIO_INT_BOTHEDGES:
          {
             /* Enable both rising and falling edge detection */

             regval = getreg32(TMS570_GIO_INTDET);
             regval |=  GIO_INTDET_PORT_PIN(port, pin);
             putreg32(regval, TMS570_GIO_INTDET);
          }
          break;
        }

      leave_critical_section(flags);
    }
}

/************************************************************************************
 * Name: tms570_gioirqenable
 *
 * Description:
 *   Enable the interrupt for specified GIO IRQ
 *
 ************************************************************************************/

void tms570_gioirqenable(int irq)
{
  int offset;
  int port;
  int pin;

  offset = irq - TMS570_IRQ_GIOA0;
  if (offset < TMS570_NGIO_IRQS)
    {
      /* Convert the offset IRQ number to a port and pin number */

      pin  = offset & 7;
      port = offset >> 3;

      /* Enable this pin interrupt */

      putreg32(GIO_ENACLR_PORT_PIN(port, pin), TMS570_GIO_ENASET);
    }
}

/************************************************************************************
 * Name: tms570_gioirqdisable
 *
 * Description:
 *   Disable the interrupt for specified GIO IRQ
 *
 ************************************************************************************/

void tms570_gioirqdisable(int irq)
{
  int offset;
  int port;
  int pin;

  offset = irq - TMS570_IRQ_GIOA0;
  if (offset < TMS570_NGIO_IRQS)
    {
      /* Convert the offset IRQ number to a port and pin number */

      pin  = offset & 7;
      port = offset >> 3;

      /* Enable this pin interrupt */

      putreg32(GIO_ENACLR_PORT_PIN(port, pin), TMS570_GIO_ENACLR);
    }
}

#endif /* CONFIG_TMS570_GIO_IRQ */
