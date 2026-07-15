/****************************************************************************
 * arch/risc-v/src/gd32vw55x/gd32vw55x_exti.c
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
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <arch/irq.h>

#include "riscv_internal.h"
#include "chip.h"
#include "gd32vw55x_exti.h"
#include "gd32vw55x_gpio.h"
#include "hardware/gd32vw55x_exti.h"
#include "hardware/gd32vw55x_gpio.h"
#include "hardware/gd32vw55x_rcu.h"
#include "hardware/gd32vw55x_syscfg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Number of EXTI lines that are connected to a GPIO pin */

#define GD32VW55X_NEXTI_GPIO   16

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The interrupt handler attached to one GPIO EXTI line */

struct gd32_gpio_irq_s
{
  xcpt_t irqhandler;
  void  *arg;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The ECLIC interrupt of each GPIO EXTI line.  Lines 5-9 and lines 10-15
 * share one interrupt each.
 */

static const int g_exti_gpio_irqs[GD32VW55X_NEXTI_GPIO] =
{
  GD32VW55X_IRQ_EXTI0,    GD32VW55X_IRQ_EXTI1,
  GD32VW55X_IRQ_EXTI2,    GD32VW55X_IRQ_EXTI3,
  GD32VW55X_IRQ_EXTI4,    GD32VW55X_IRQ_EXTI5_9,
  GD32VW55X_IRQ_EXTI5_9,  GD32VW55X_IRQ_EXTI5_9,
  GD32VW55X_IRQ_EXTI5_9,  GD32VW55X_IRQ_EXTI5_9,
  GD32VW55X_IRQ_EXTI10_15, GD32VW55X_IRQ_EXTI10_15,
  GD32VW55X_IRQ_EXTI10_15, GD32VW55X_IRQ_EXTI10_15,
  GD32VW55X_IRQ_EXTI10_15, GD32VW55X_IRQ_EXTI10_15
};

/* The handler attached to each GPIO EXTI line */

static struct gd32_gpio_irq_s g_gpio_irq[GD32VW55X_NEXTI_GPIO];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_syscfg_clock_enable
 *
 * Description:
 *   Enable the SYSCFG clock.  The EXTI source selection registers live in
 *   the SYSCFG block, so its clock must be running before an EXTI line can
 *   be assigned to a GPIO port.
 *
 ****************************************************************************/

static void gd32_syscfg_clock_enable(void)
{
  uint32_t regval = getreg32(GD32VW55X_RCU_APB2EN);

  if ((regval & RCU_APB2EN_SYSCFGEN) == 0)
    {
      putreg32(regval | RCU_APB2EN_SYSCFGEN, GD32VW55X_RCU_APB2EN);
    }
}

/****************************************************************************
 * Name: gd32_syscfg_exti_line_config
 *
 * Description:
 *   Connect an EXTI line to the pin of the given GPIO port.
 *
 * Input Parameters:
 *   port - The GPIO port number (0=GPIOA, 1=GPIOB, 2=GPIOC)
 *   pin  - The GPIO pin number, 0-15
 *
 ****************************************************************************/

static void gd32_syscfg_exti_line_config(uint8_t port, uint8_t pin)
{
  uintptr_t regaddr;
  uint32_t regval;

  regaddr = GD32VW55X_SYSCFG_EXTISS(pin);

  regval  = getreg32(regaddr);
  regval &= ~SYSCFG_EXTISS_MASK(pin);
  regval |= ((uint32_t)port << SYSCFG_EXTISS_SHIFT(pin));
  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: gd32_exti_dispatch
 *
 * Description:
 *   Clear the pending flag of one EXTI line and call the handler which is
 *   attached to it, if any.
 *
 ****************************************************************************/

static int gd32_exti_dispatch(int irq, void *context, int line)
{
  xcpt_t irqhandler;
  void *arg;
  int ret = OK;

  /* Clear the pending flag of the line before the handler runs so that an
   * edge which arrives while the handler is running is not lost.
   */

  putreg32(EXTI_LINE(line), GD32VW55X_EXTI_PD);

  irqhandler = g_gpio_irq[line].irqhandler;
  arg        = g_gpio_irq[line].arg;

  if (irqhandler != NULL)
    {
      ret = irqhandler(irq, context, arg);
    }

  return ret;
}

/****************************************************************************
 * Name: gd32_exti_multi_dispatch
 *
 * Description:
 *   Handle the EXTI interrupts which are shared by several lines (lines
 *   5-9 and lines 10-15).
 *
 ****************************************************************************/

static int gd32_exti_multi_dispatch(int irq, void *context, int first,
                                    int last)
{
  uint32_t pending;
  int line;
  int ret = OK;
  int tmp;

  pending = getreg32(GD32VW55X_EXTI_PD) & getreg32(GD32VW55X_EXTI_INTEN);

  for (line = first; line <= last; line++)
    {
      if ((pending & EXTI_LINE(line)) != 0)
        {
          tmp = gd32_exti_dispatch(irq, context, line);
          if (tmp < 0)
            {
              ret = tmp;
            }
        }
    }

  return ret;
}

/****************************************************************************
 * Name: gd32_exti0_irqhandler and friends
 *
 * Description:
 *   The interrupt handlers attached to the ECLIC.  Each one of the EXTI
 *   lines 0-4 has a dedicated interrupt; lines 5-9 and 10-15 share one.
 *
 ****************************************************************************/

static int gd32_exti0_irqhandler(int irq, void *context, void *arg)
{
  return gd32_exti_dispatch(irq, context, 0);
}

static int gd32_exti1_irqhandler(int irq, void *context, void *arg)
{
  return gd32_exti_dispatch(irq, context, 1);
}

static int gd32_exti2_irqhandler(int irq, void *context, void *arg)
{
  return gd32_exti_dispatch(irq, context, 2);
}

static int gd32_exti3_irqhandler(int irq, void *context, void *arg)
{
  return gd32_exti_dispatch(irq, context, 3);
}

static int gd32_exti4_irqhandler(int irq, void *context, void *arg)
{
  return gd32_exti_dispatch(irq, context, 4);
}

static int gd32_exti5_9_irqhandler(int irq, void *context, void *arg)
{
  return gd32_exti_multi_dispatch(irq, context, 5, 9);
}

static int gd32_exti10_15_irqhandler(int irq, void *context, void *arg)
{
  return gd32_exti_multi_dispatch(irq, context, 10, 15);
}

/****************************************************************************
 * Name: gd32_exti_irqhandler
 *
 * Description:
 *   Return the interrupt handler which must be attached to the ECLIC for
 *   the given GPIO EXTI line.
 *
 ****************************************************************************/

static xcpt_t gd32_exti_irqhandler(int line)
{
  switch (line)
    {
      case 0:
        return gd32_exti0_irqhandler;

      case 1:
        return gd32_exti1_irqhandler;

      case 2:
        return gd32_exti2_irqhandler;

      case 3:
        return gd32_exti3_irqhandler;

      case 4:
        return gd32_exti4_irqhandler;

      case 5:
      case 6:
      case 7:
      case 8:
      case 9:
        return gd32_exti5_9_irqhandler;

      default:
        return gd32_exti10_15_irqhandler;
    }
}

/****************************************************************************
 * Name: gd32_exti_irq_inuse
 *
 * Description:
 *   Check whether any GPIO EXTI line which shares the given ECLIC
 *   interrupt still has a handler attached to it.
 *
 ****************************************************************************/

static bool gd32_exti_irq_inuse(int irq)
{
  int line;

  for (line = 0; line < GD32VW55X_NEXTI_GPIO; line++)
    {
      if (g_exti_gpio_irqs[line] == irq &&
          g_gpio_irq[line].irqhandler != NULL)
        {
          return true;
        }
    }

  return false;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_exti_init
 *
 * Description:
 *   Initialize the mode and the trigger type of the given EXTI lines.
 *
 ****************************************************************************/

void gd32_exti_init(uint32_t linex, uint8_t exti_mode, uint8_t trig_type)
{
  /* Reset the mode and the trigger type of the lines */

  modifyreg32(GD32VW55X_EXTI_INTEN, linex, 0);
  modifyreg32(GD32VW55X_EXTI_EVEN, linex, 0);
  modifyreg32(GD32VW55X_EXTI_RTEN, linex, 0);
  modifyreg32(GD32VW55X_EXTI_FTEN, linex, 0);

  /* Set the trigger type before the lines are enabled */

  switch (trig_type)
    {
      case EXTI_TRIG_RISING:
        modifyreg32(GD32VW55X_EXTI_RTEN, 0, linex);
        break;

      case EXTI_TRIG_FALLING:
        modifyreg32(GD32VW55X_EXTI_FTEN, 0, linex);
        break;

      case EXTI_TRIG_BOTH:
        modifyreg32(GD32VW55X_EXTI_RTEN, 0, linex);
        modifyreg32(GD32VW55X_EXTI_FTEN, 0, linex);
        break;

      case EXTI_TRIG_NONE:
      default:
        break;
    }

  /* Enable the interrupt or the event of the lines */

  switch (exti_mode)
    {
      case EXTI_MODE_INTERRUPT:
        modifyreg32(GD32VW55X_EXTI_INTEN, 0, linex);
        break;

      case EXTI_MODE_EVENT:
        modifyreg32(GD32VW55X_EXTI_EVEN, 0, linex);
        break;

      case EXTI_MODE_NONE:
      default:
        break;
    }
}

/****************************************************************************
 * Name: gd32_exti_interrupt_enable
 *
 * Description:
 *   Enable the interrupts from the given EXTI lines.
 *
 ****************************************************************************/

void gd32_exti_interrupt_enable(uint32_t linex)
{
  modifyreg32(GD32VW55X_EXTI_INTEN, 0, linex);
}

/****************************************************************************
 * Name: gd32_exti_interrupt_disable
 *
 * Description:
 *   Disable the interrupts from the given EXTI lines.
 *
 ****************************************************************************/

void gd32_exti_interrupt_disable(uint32_t linex)
{
  modifyreg32(GD32VW55X_EXTI_INTEN, linex, 0);
}

/****************************************************************************
 * Name: gd32_exti_event_enable
 *
 * Description:
 *   Enable the events from the given EXTI lines.
 *
 ****************************************************************************/

void gd32_exti_event_enable(uint32_t linex)
{
  modifyreg32(GD32VW55X_EXTI_EVEN, 0, linex);
}

/****************************************************************************
 * Name: gd32_exti_event_disable
 *
 * Description:
 *   Disable the events from the given EXTI lines.
 *
 ****************************************************************************/

void gd32_exti_event_disable(uint32_t linex)
{
  modifyreg32(GD32VW55X_EXTI_EVEN, linex, 0);
}

/****************************************************************************
 * Name: gd32_exti_software_interrupt_enable
 *
 * Description:
 *   Raise a software interrupt/event on the given EXTI lines.
 *
 ****************************************************************************/

void gd32_exti_software_interrupt_enable(uint32_t linex)
{
  modifyreg32(GD32VW55X_EXTI_SWIEV, 0, linex);
}

/****************************************************************************
 * Name: gd32_exti_software_interrupt_disable
 *
 * Description:
 *   Clear the software interrupt/event of the given EXTI lines.
 *
 ****************************************************************************/

void gd32_exti_software_interrupt_disable(uint32_t linex)
{
  modifyreg32(GD32VW55X_EXTI_SWIEV, linex, 0);
}

/****************************************************************************
 * Name: gd32_exti_interrupt_flag_get
 *
 * Description:
 *   Check whether any of the given EXTI lines has a pending interrupt.
 *
 ****************************************************************************/

bool gd32_exti_interrupt_flag_get(uint32_t linex)
{
  uint32_t pending = getreg32(GD32VW55X_EXTI_PD) &
                     getreg32(GD32VW55X_EXTI_INTEN);

  return ((pending & linex) != 0);
}

/****************************************************************************
 * Name: gd32_exti_interrupt_flag_clear
 *
 * Description:
 *   Clear the pending flag of the given EXTI lines.
 *
 ****************************************************************************/

void gd32_exti_interrupt_flag_clear(uint32_t linex)
{
  putreg32(linex, GD32VW55X_EXTI_PD);
}

/****************************************************************************
 * Name: gd32_gpio_setevent
 *
 * Description:
 *   Sets/clears GPIO based event and interrupt triggers.
 *
 * Input Parameters:
 *   pinset      - GPIO pin configuration, as used by gd32_gpio_config().
 *                 Only the port and the pin number are used.
 *   risingedge  - Enables interrupt on rising edges
 *   fallingedge - Enables interrupt on falling edges
 *   event       - Generate an event instead of an interrupt
 *   func        - Interrupt handler.  A NULL value disables the interrupt
 *                 and detaches any previously installed handler.
 *   arg         - Argument passed to the interrupt handler
 *
 * Returned Value:
 *   OK on success
 *   A negated errno value on invalid port or pin.
 *
 ****************************************************************************/

int gd32_gpio_setevent(uint32_t pinset, bool risingedge, bool fallingedge,
                       bool event, xcpt_t func, void *arg)
{
  irqstate_t flags;
  uint32_t linex;
  uint8_t trig_type;
  uint8_t exti_mode;
  uint8_t port;
  uint8_t pin;
  int irq;

  /* Verify that this hardware supports the selected GPIO port */

  port = (pinset & GPIO_CFG_PORT_MASK) >> GPIO_CFG_PORT_SHIFT;
  if (port >= GD32VW55X_NGPIO_PORTS)
    {
      return -EINVAL;
    }

  /* Get the pin number.  It is also the EXTI line number */

  pin   = (pinset & GPIO_CFG_PIN_MASK) >> GPIO_CFG_PIN_SHIFT;
  linex = EXTI_LINE(pin);
  irq   = g_exti_gpio_irqs[pin];

  flags = enter_critical_section();

  if (func == NULL && !event)
    {
      /* Disable the line and detach the handler */

      gd32_exti_init(linex, EXTI_MODE_NONE, EXTI_TRIG_NONE);
      gd32_exti_interrupt_flag_clear(linex);

      g_gpio_irq[pin].irqhandler = NULL;
      g_gpio_irq[pin].arg        = NULL;

      /* The interrupts of the lines 5-9 and 10-15 are shared.  Only detach
       * the ECLIC interrupt when no other line is using it.
       */

      if (!gd32_exti_irq_inuse(irq))
        {
          up_disable_irq(irq);
          irq_detach(irq);
        }

      leave_critical_section(flags);
      return OK;
    }

  /* Select the trigger type */

  if (risingedge && fallingedge)
    {
      trig_type = EXTI_TRIG_BOTH;
    }
  else if (risingedge)
    {
      trig_type = EXTI_TRIG_RISING;
    }
  else if (fallingedge)
    {
      trig_type = EXTI_TRIG_FALLING;
    }
  else
    {
      trig_type = EXTI_TRIG_NONE;
    }

  exti_mode = event ? EXTI_MODE_EVENT : EXTI_MODE_INTERRUPT;

  /* Connect the EXTI line to the pin of the selected GPIO port */

  gd32_syscfg_clock_enable();
  gd32_syscfg_exti_line_config(port, pin);

  /* Install the handler before the line is enabled */

  g_gpio_irq[pin].irqhandler = func;
  g_gpio_irq[pin].arg        = arg;

  /* Configure the EXTI line and clear any interrupt which is left over
   * from a previous configuration.
   */

  gd32_exti_init(linex, exti_mode, trig_type);
  gd32_exti_interrupt_flag_clear(linex);

  /* Attach and enable the ECLIC interrupt of the line.  Events do not
   * reach the CPU, so no interrupt is needed for them.
   */

  if (exti_mode == EXTI_MODE_INTERRUPT)
    {
      irq_attach(irq, gd32_exti_irqhandler(pin), NULL);
      up_enable_irq(irq);
    }

  leave_critical_section(flags);
  return OK;
}
