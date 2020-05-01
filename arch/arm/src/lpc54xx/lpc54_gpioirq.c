/****************************************************************************
 * arch/arm/src/lpc54/lpc54_gpioirq.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "arm_arch.h"

#include "hardware/lpc54_syscon.h"
#include "hardware/lpc54_inputmux.h"
#include "hardware/lpc54_pint.h"
#include "lpc54_gpio.h"

#ifdef CONFIG_LPC54_GPIOIRQ

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The maximum number of pin interrupts */

#define MAX_PININT  8

/* A mask for both the port and pin number */

#define GPIO_PORTPIN_MASK (GPIO_PORT_MASK | GPIO_PIN_MASK)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is the set of all pin interrupts that have been allocated.  Any pin
 * in P0 or P1 may be configured as an interrupts source via the input
 * multiplexor.  Up to eight pin interrupts are supported.
 */

static uint8_t g_pinints;

/* Maps a pin interrupt number to an IRQ number (they are not contiguous) */

static const uint8_t g_pinirq[MAX_PININT] =
{
 LPC54_IRQ_PININT0, LPC54_IRQ_PININT1, LPC54_IRQ_PININT2, LPC54_IRQ_PININT3,
 LPC54_IRQ_PININT4, LPC54_IRQ_PININT5, LPC54_IRQ_PININT6, LPC54_IRQ_PININT7
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc54_alloc_pinint
 *
 * Description:
 *   Initialize logic to support interrupting GPIO pins.  This function is
 *   called by the OS inialization logic and is not a user interface.
 *
 ****************************************************************************/

static int lpc54_alloc_pinint(lpc54_pinset_t pinset)
{
  irqstate_t flags = enter_critical_section();
  int pin;

  /* REVISIT:  This is overlying complex in the current design.  There is
   * not yet any mechanism to de-configure a pin.  At present, a simple
   * counter would be sufficient to assign a pin.  This bit-mapped allocator
   * is used in the anticipation that such pin-deconfiguration will be
   * supported in the future.
   */

  for (pin = 0; pin < MAX_PININT; pin++)
    {
      uint8_t mask = (1 << pin);
      if ((g_pinints & mask) == 0)
        {
          g_pinints |= mask;
          leave_critical_section(flags);
          return pin;
        }
    }

  leave_critical_section(flags);
  return -ENOSPC;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc54_gpio_irqinitialize
 *
 * Description:
 *   Initialize logic to support interrupting GPIO pins.  This function is
 *   called by the OS inialization logic and is not a user interface.
 *
 ****************************************************************************/

void lpc54_gpio_irqinitialize(void)
{
  /* NOTE: "Once set up, no clocks are needed for the input multiplexer to
   * function. The system clock is needed only to write to or read from the
   * INPUT MUX registers. Once the input multiplexer is configured, disable
   * the clock to the INPUT MUX block in the AHBCLKCTRL register."
   *
   * REVISIT: Future power optimization.
   */

#ifdef CONFIG_LPC54_GPIOIRQ_GROUPS
  /* Enable the Input Mux, PINT, and GINT modules */

  putreg32(SYSCON_AHBCLKCTRL0_INPUTMUX | SYSCON_AHBCLKCTRL0_PINT |
           SYSCON_AHBCLKCTRL0_GINT, LPC54_SYSCON_AHBCLKCTRLSET0);
#else
  /* Enable the Input Mux and PINT modules */

  putreg32(SYSCON_AHBCLKCTRL0_INPUTMUX | SYSCON_AHBCLKCTRL0_PINT,
           LPC54_SYSCON_AHBCLKCTRLSET0);
#endif
}

/************************************************************************************
 * Name: lpc54_gpio_interrupt
 *
 * Description:
 *   Configure a GPIO interrupt pin based on bit-encoded description of the pin.
 *   This function is called by lpc54_gpio_config to setup interrupting pins.  It is
 *   not a user interface.
 *
 ************************************************************************************/

int lpc54_gpio_interrupt(lpc54_pinset_t pinset)
{
  uintptr_t regaddr;
  uint32_t mask;
  unsigned int port;
  int pinint;

  /* Is this pin configured as an interrupting pin */

  if (!GPIO_IS_INTR(pinset))
    {
      return -EPERM;
    }

  /* Pin interrupts are supported only on P0 and P1 */

  port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  if (port > 1)
    {
      return -EINVAL;
    }

  /* Assign a pin interrupt */

  pinint = lpc54_alloc_pinint(pinset);
  if (pinint < 0)
    {
      return pinint;
    }

  /* Make sure that the pin interrupt is disabled at the NVIC. */

  up_disable_irq(g_pinirq[pinint]);

  /* Select the pin interrupt input:
   *
   *   For PIOm.n: pin = (m * 32) + n.
   *   PIO0.0 to PIO1.31 correspond to numbers 0 to 63.
   */

  regaddr = LPC54_MUX_PINTSEL(pinint);
  putreg32((uint32_t)pinset & GPIO_PORTPIN_MASK, regaddr);

  /* Enable the pin interrupt triggers */

  mask = (1 << pinint);
  if (GPIO_IS_INTLEVEL(pinset))
    {
      /* Set the pinint bit to select level sensitive trigger */

      modifyreg32(LPC54_PINT_ISEL, 0, mask);
    }
  else
    {
      /* Clear the pinint bit to select edge sensitive trigger */

      modifyreg32(LPC54_PINT_ISEL, mask, 0);
    }

  switch (pinset & GPIO_FUNC_MASK)
    {
      /* Write to SIENR to enable rising-edge or level interrupts */

      case GPIO_INTRE:    /* GPIO interrupt rising edge */
      case GPIO_INTBOTH:  /* GPIO interrupt both edges */
      case GPIO_INTLOW:   /* GPIO interrupt low level */
      case GPIO_INTHIGH:  /* GPIO interrupt high level */
        putreg32(mask, LPC54_PINT_SIENR);
        break;

      /* Write to CIENR to disable rising-edge or level interrupts */

      case GPIO_INTFE:    /* GPIO interrupt falling edge */
        putreg32(mask, LPC54_PINT_CIENR);
        break;

      default:
        DEBUGPANIC();
        return -EINVAL;
    }

  switch (pinset & GPIO_FUNC_MASK)
    {
      /* Write to SIENF to enable falling-edge or active-high level
       * interrupts.
       */

      case GPIO_INTFE:    /* GPIO interrupt falling edge */
      case GPIO_INTBOTH:  /* GPIO interrupt both edges */
      case GPIO_INTHIGH:  /* GPIO interrupt high level */
        putreg32(mask, LPC54_PINT_SIENF);
        break;

      /* Write to CIENF to disable falling-edge or enable active-low level
       * interrupts.
       */

      case GPIO_INTRE:    /* GPIO interrupt rising edge */
      case GPIO_INTLOW:   /* GPIO interrupt low level */
        putreg32(mask, LPC54_PINT_CIENF);
        break;

      default:
        DEBUGPANIC();
        return -EINVAL;
    }

  return OK;
}

/************************************************************************************
 * Name: lpc54_gpio_irqno
 *
 * Description:
 *   Returns the IRQ number that was associated with an interrupt pin after it was
 *   configured.
 *
 ************************************************************************************/

int lpc54_gpio_irqno(lpc54_pinset_t pinset)
{
  irqstate_t flags;
  uintptr_t regaddr;
  uint32_t regval;
  int portpin = pinset & GPIO_PORTPIN_MASK;
  int i;

  flags = enter_critical_section();

  /* Find the PININT index that as the assignment to the this port and pin */

  for (i = 0, regaddr = LPC54_MUX_PINTSEL0;
       i < MAX_PININT;
       i++, regaddr += 4)
    {
      regval = getreg32(regaddr) & GPIO_PORTPIN_MASK;
      if (regval == portpin)
        {
          leave_critical_section(flags);
          return (int)g_pinirq[i];
        }
    }

  leave_critical_section(flags);
  return -ENOENT;
}

/************************************************************************************
 * Name: lpc54_gpio_ackedge
 *
 * Description:
 *   Acknowledge edge interrupts by clearing the associated bits in the rising and
 *   falling registers.  This acknowledgemment is, of course, not needed for level
 *   interrupts.
 *
 ************************************************************************************/

int lpc54_gpio_ackedge(int irq)
{
  uint32_t regval;
  uint32_t mask;
  unsigned int pinint;

  /* Map the IRQ number to a pin interrupt number */

  if (irq >= LPC54_IRQ_PININT0 && irq <= LPC54_IRQ_PININT3)
    {
      pinint = irq - LPC54_IRQ_PININT0;
    }
  else if (irq >= LPC54_IRQ_PININT4 && irq <= LPC54_IRQ_PININT7)
    {
      pinint = irq - LPC54_IRQ_PININT4 + 4;
    }
  else
    {
      return -EINVAL;
    }

  /* Acknowledge the pin interrupt */

  mask   = (1 << pinint);
  regval = getreg32(LPC54_PINT_RISE) & mask;
  putreg32(regval, LPC54_PINT_RISE);

  regval = getreg32(LPC54_PINT_FALL) & mask;
  putreg32(regval, LPC54_PINT_FALL);
  return OK;
}

#endif /* CONFIG_LPC54_GPIOIRQ */
