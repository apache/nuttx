/****************************************************************************
 * arch/arm/src/rm57/rm57_gioirq.c
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

/* Adapted from tms570_gioirq.c, using RM57's 2-port GIO register layout
 * (see rm57_gio.c).
 *
 * The GIO module drives two separate VIM request lines, and per SPNU562A
 * section 25.5.5 the level registers pick which one a pin uses:
 *
 *   LVLSET -> level A "high level" -> reported in GIOOFF1 -> VIM channel 9
 *             (RM57_REQ_GIOHIGH)
 *   LVLCLR -> level B "low level"  -> reported in GIOOFF2 -> VIM channel 23
 *             (RM57_REQ_GIOLOW)
 *
 * Reset leaves every pin on level B, so a pin must be explicitly promoted
 * with LVLSET before it can reach RM57_REQ_GIOHIGH.  This driver wires up
 * only the level A line, so rm57_gioirq() promotes each pin it arms and
 * RM57_REQ_GIOLOW stays unused.
 *
 * NOTE: the upstream tms570_gioirq.c this was derived from writes LVLCLR
 * while attaching to the level A channel, which silently strands every pin
 * on the unhandled level B line.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "hardware/rm57_gio.h"
#include "rm57_gio.h"

#ifdef CONFIG_RM57_GIO_IRQ

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rm57_gio_interrupt
 *
 * Description:
 *   Receive GIO interrupts
 *
 ****************************************************************************/

static int rm57_gio_interrupt(int irq, void *context, void *arg)
{
  uint32_t off1;
  int irq2;

  /* Loop until all pending GIO interrupts have been processed.
   * RM57_GIO_OFF1 is a hardware priority encoder that only ever presents the
   * highest-priority pending pin per read, so a single read here could
   * silently strand a second pin that became pending while the first was
   * being dispatched.  The loop terminates because reading the register
   * clears both it and the corresponding GIOFLG bit, so it eventually reads
   * back zero (SPNU562A section 25.5.7).
   */

  while ((off1 = getreg32(RM57_GIO_OFF1)) != GIO_OFF_NONE)
    {
      /* Convert the offset value to the second-level IRQ number */

      irq2 = off1 + RM57_IRQ_GIOA0 - 1;

      /* And dispatch the second-level GIO IRQ */

      irq_dispatch(irq2, context);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rm57_gioirq_initialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for
 *   GIO pins.
 *
 ****************************************************************************/

void rm57_gioirq_initialize(void)
{
  /* Disable every pin interrupt and return every pin to level B, the reset
   * default.  rm57_gioirq() promotes the pins it arms to level A.
   */

  putreg32(0xffffffff, RM57_GIO_ENACLR);
  putreg32(0xffffffff, RM57_GIO_LVLCLR);

  /* Discard any transition latched before we got here.  Per SPNU562A
   * section 25.5.6 a GIOFLG bit is set by the selected transition whether
   * or not interrupt generation is enabled, so a stale flag would fire the
   * instant a pin is armed.
   */

  putreg32(0xffffffff, RM57_GIO_FLG);

  /* Attach and enable the GIO level A (high level) interrupt */

  DEBUGVERIFY(irq_attach(RM57_REQ_GIOHIGH, rm57_gio_interrupt, NULL));
  up_enable_irq(RM57_REQ_GIOHIGH);
}

/****************************************************************************
 * Name: rm57_gioirq
 *
 * Description:
 *   Configure an interrupt for the specified GIO pin.
 *
 ****************************************************************************/

void rm57_gioirq(gio_pinset_t pinset)
{
  uint32_t port = rm57_gio_port(pinset);
  uint32_t pin = rm57_gio_pin(pinset);
  irqstate_t flags;
  uint32_t regval;

  /* Start with the pin interrupt disabled while it is reconfigured */

  putreg32(GIO_ENACLR_PORT_PIN(port, pin), RM57_GIO_ENACLR);

  /* Make sure that the pin is configured as an input and that interrupts
   * can be supported on this port.
   */

  if ((pinset & GIO_MODE_MASK) == GIO_INPUT && port < RM57_NPORTS)
    {
      flags = enter_critical_section();
      switch (pinset & GIO_INT_MASK)
        {
          case GIO_INT_NONE:
          default:
            break;

          case GIO_INT_RISING:
            {
              /* Enable rising edge detection */

              regval  = getreg32(RM57_GIO_POL);
              regval |= GIO_POL_PORT_PIN(port, pin);
              putreg32(regval, RM57_GIO_POL);

              /* Select single edge detection so that GIOPOL decides which
               * edge is used
               */

              regval  = getreg32(RM57_GIO_INTDET);
              regval &= ~GIO_INTDET_PORT_PIN(port, pin);
              putreg32(regval, RM57_GIO_INTDET);
            }
            break;

          case GIO_INT_FALLING:
            {
              /* Enable falling edge detection */

              regval  = getreg32(RM57_GIO_POL);
              regval &= ~GIO_POL_PORT_PIN(port, pin);
              putreg32(regval, RM57_GIO_POL);

              /* Select single edge detection so that GIOPOL decides which
               * edge is used
               */

              regval  = getreg32(RM57_GIO_INTDET);
              regval &= ~GIO_INTDET_PORT_PIN(port, pin);
              putreg32(regval, RM57_GIO_INTDET);
            }
            break;

          case GIO_INT_BOTHEDGES:
            {
              /* Enable both rising and falling edge detection */

              regval  = getreg32(RM57_GIO_INTDET);
              regval |= GIO_INTDET_PORT_PIN(port, pin);
              putreg32(regval, RM57_GIO_INTDET);
            }
            break;
        }

      if ((pinset & GIO_INT_MASK) != GIO_INT_NONE)
        {
          /* Promote the pin to a level A (high level) interrupt so that it
           * is reported in GIOOFF1 and raised on RM57_REQ_GIOHIGH, the only
           * GIO request line this driver handles.  Without this the pin
           * stays on level B and its interrupt is never delivered.
           */

          putreg32(GIO_LVLSET_PORT_PIN(port, pin), RM57_GIO_LVLSET);

          /* Drop any transition latched while the pin was being configured
           * so that arming it does not fire immediately.
           */

          putreg32(GIO_FLG_PORT_PIN(port, pin), RM57_GIO_FLG);
        }

      leave_critical_section(flags);
    }
}

/****************************************************************************
 * Name: rm57_gioirqenable
 *
 * Description:
 *   Enable the interrupt for specified GIO IRQ
 *
 ****************************************************************************/

void rm57_gioirqenable(int irq)
{
  int offset;
  int port;
  int pin;

  offset = irq - RM57_IRQ_GIOA0;
  if (offset >= 0 && offset < RM57_NGIO_IRQS)
    {
      /* Convert the offset IRQ number to a port and pin number */

      pin  = offset & 7;
      port = offset >> 3;

      /* Clear any stale flag, then enable this pin interrupt */

      putreg32(GIO_FLG_PORT_PIN(port, pin), RM57_GIO_FLG);
      putreg32(GIO_ENASET_PORT_PIN(port, pin), RM57_GIO_ENASET);
    }
}

/****************************************************************************
 * Name: rm57_gioirqdisable
 *
 * Description:
 *   Disable the interrupt for specified GIO IRQ
 *
 ****************************************************************************/

void rm57_gioirqdisable(int irq)
{
  int offset;
  int port;
  int pin;

  offset = irq - RM57_IRQ_GIOA0;
  if (offset >= 0 && offset < RM57_NGIO_IRQS)
    {
      /* Convert the offset IRQ number to a port and pin number */

      pin  = offset & 7;
      port = offset >> 3;

      /* Disable this pin interrupt */

      putreg32(GIO_ENACLR_PORT_PIN(port, pin), RM57_GIO_ENACLR);
    }
}

#endif /* CONFIG_RM57_GIO_IRQ */
