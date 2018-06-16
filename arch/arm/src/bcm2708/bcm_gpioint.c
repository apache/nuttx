/****************************************************************************
 * arch/arm/src/bcm2708/bcm_gpioint.c
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

#include "bcm_config.h"
#include "bcm_gpio.h"

#ifdef CONFIG_BCM2708_GPIO_IRQ

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bcm_gpio_risingedge
 *
 * Description:
 *   Set/clear rising edge detection.
 *
 ****************************************************************************/

static void bcm_gpio_risingedge(unsigned int pin, gpio_pinset_t pinset)
{
  uintptr_t regaddr = BCM_GPIO_GPREN(pin);
  uint32_t mask = BCM_GPIO_GPREN_REN(pin;

  if ((pinset & GPIO_INT_MASK) == GPIO_INT_RISING)
    {
      modifiyreg32(regaddr, 0, mask);
    }
  else
    {
      modifiyreg32(regaddr, mask, 0);
    }
}

/****************************************************************************
 * Name: bcm_gpio_fallingedge
 *
 * Description:
 *   Set/clear falling edge detection.
 *
 ****************************************************************************/

static void bcm_gpio_fallingedge(unsigned int pin, gpio_pinset_t pinset)
{
  uintptr_t regaddr = BCM_GPIO_GPFEN(pin);
  uint32_t mask = BCM_GPIO_GPFEN_FEN(pin;

  if ((pinset & GPIO_INT_MASK) == GPIO_INT_FALLING)
    {
      modifiyreg32(regaddr, 0, mask);
    }
  else
    {
      modifiyreg32(regaddr, mask, 0);
    }
}

/****************************************************************************
 * Name: bcm_gpio_highlevel
 *
 * Description:
 *   Set/clear high level detection.
 *
 ****************************************************************************/

static void bcm_gpio_highlevel(unsigned int pin, gpio_pinset_t pinset)
{
  uintptr_t regaddr = BCM_GPIO_GPHEN(pin);
  uint32_t mask = BCM_GPIO_GPHEN_HEN(pin;

  if ((pinset & GPIO_INT_MASK) == GPIO_INT_HIGHLEVEL)
    {
      modifiyreg32(regaddr, 0, mask);
    }
  else
    {
      modifiyreg32(regaddr, mask, 0);
    }
}

/****************************************************************************
 * Name: bcm_gpio_lowlevel
 *
 * Description:
 *   Set/clear low level detection.
 *
 ****************************************************************************/

static void bcm_gpio_lowlevel(unsigned int pin, gpio_pinset_t pinset)
{
  uintptr_t regaddr = BCM_GPIO_GPLEN(pin);
  uint32_t mask = BCM_GPIO_GPHEN_HEN(pin;

  if ((pinset & GPIO_INT_MASK) == GPIO_INT_LOWLEVEL)
    {
      modifiyreg32(regaddr, 0, mask);
    }
  else
    {
      modifiyreg32(regaddr, mask, 0);
    }
}

/****************************************************************************
 * Name: bcm_gpio_async_risingedge
 *
 * Description:
 *   Set/clear asynchronous rising edge detection.
 *
 ****************************************************************************/

static void bcm_gpio_async_risingedge(unsigned int pin, gpio_pinset_t pinset)
{
  uintptr_t regaddr = BCM_GPIO_GPAREN(pin);
  uint32_t mask = BCM_GPIO_GPAREN_AREN(pin;

  if ((pinset & GPIO_INT_MASK) == GPIO_INT_ASYNCHRISING)
    {
      modifiyreg32(regaddr, 0, mask);
    }
  else
    {
      modifiyreg32(regaddr, mask, 0);
    }
}

/****************************************************************************
 * Name: bcm_gpio_async_fallingedge
 *
 * Description:
 *   Set/clear asynchronous falling edge detection.
 *
 ****************************************************************************/

static void bcm_gpio_async_fallingedge(unsigned int pin, gpio_pinset_t pinset)
{
  uintptr_t regaddr = BCM_GPIO_GPAFEN(pin);
  uint32_t mask = BCM_GPIO_GPAFEN_AFEN(pin;

  if ((pinset & GPIO_INT_MASK) == GPIO_INT_ASYNCHFALLING)
    {
      modifiyreg32(regaddr, 0, mask);
    }
  else
    {
      modifiyreg32(regaddr, mask, 0);
    }
}

/****************************************************************************
 * Name: bcm_gpio_async_fallingedge
 *
 * Description:
 *   Set/clear falling edge detection.
 *
 ****************************************************************************/

static void bcm_gpio_async_fallingedge(unsigned int pin, gpio_pinset_t pinset)
{
  uintptr_t regaddr = BCM_GPIO_GPAFEN(pin);
  uint32_t mask = BCM_GPIO_GPAFEN_AFEN(pin;

  if ((pinset & GPIO_INT_MASK) == GPIO_INT_ASYNCHFALLING)
    {
      modifiyreg32(regaddr, 0, mask);
    }
  else
    {
      modifiyreg32(regaddr, mask, 0);
    }
}

/****************************************************************************
 * Name: bcm_gpio0_interrupt
 *
 * Description:
 *   GPIO0 interrupt handler
 *
 ****************************************************************************/

int bcm_gpio0_interrupt(int irq, FAR void *context, FAR void *arg)
{
  uint32_t eds;
  uint32_t mask;
  int i;

  /* Clear all pending interrpts */

  eds = getreg32(BCM_GPIO_GPEDS0);
  putreg32(eds, BCM_GPIO_GPEDS0);

  /* Then process each pending GPIO interrupt */

  for (i = 0; i < 32 && eds != NULL; i++)
    {
      mask = (uint32_t)1 << i;
      if ((eds & mask) != 0)
        {
          /* Remove the pending interrupt bit from the mask */

          eds &= ~mask;

          /* And disptach the GPIO interrupt to the register handler */

          irq_dispatch(BCM_IRQ_GPIO0_FIRST + i, context);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: bcm_gpio1_interrupt
 *
 * Description:
 *   GPIO1 interrupt handler
 *
 ****************************************************************************/

int bcm_gpio1_interrupt(int irq, FAR void *context, FAR void *arg)
{
  uint32_t eds;
  uint32_t mask;
  int i;

  /* Clear all pending interrpts */

  eds = getreg32(BCM_GPIO_GPEDS1);
  putreg32(eds, BCM_GPIO_GPEDS1);

  /* Then process each pending GPIO interrupt */

  for (i = 0; i < 32 && eds != NULL; i++)
    {
      mask = (uint32_t)1 << i;
      if ((eds & mask) != 0)
        {
          /* Remove the pending interrupt bit from the mask */

          eds &= ~mask;

          /* And disptach the GPIO interrupt to the register handler */

          irq_dispatch(BCM_IRQ_GPIO1_FIRST + i, context);
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bcm_gpio_irqinitialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for
 *   GPIO pins.
 *
 ****************************************************************************/

void bcm_gpio_irqinitialize(void)
{
  /* Disabled all event detections */

  putreg32(0, BCM_GPIO_GPREN0);
  putreg32(0, BCM_GPIO_GPREN1);
  putreg32(0, BCM_GPIO_GPFEN0);
  putreg32(0, BCM_GPIO_GPFEN1);
  putreg32(0, BCM_GPIO_GPHEN0);
  putreg32(0, BCM_GPIO_GPHEN1);
  putreg32(0, BCM_GPIO_GPLEN0);
  putreg32(0, BCM_GPIO_GPLEN1);
  putreg32(0, BCM_GPIO_GPAREN0);
  putreg32(0, BCM_GPIO_GPAREN1);
  putreg32(0, BCM_GPIO_GPAFEN0);
  putreg32(0, BCM_GPIO_GPAFEN1);

  /* Attach and enable the GPIO interrupt handlers */

  (void) irq_attach(BCM_IRQ_GPIO0, bcm_gpio0_interrupt);
  (void) irq_attach(BCM_IRQ_GPIO1, bcm_gpio1_interrupt);

  up_enable_irq(BCM_IRQ_GPIO0);
  up_enable_irq(BCM_IRQ_GPIO1);
}

/****************************************************************************
 * Name: bcm_gpio_irqenable
 *
 * Description:
 *   Configure interrupt event detection for the specified GPIO pin.  This
 *   effective enables the pin interrupts.
 *
 ****************************************************************************/

void bcm_gpio_irqenable(gpio_pinset_t pinset)
{
  unsigned int pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

  /* Configure pin detection settings */

  bcm_gpio_risingedge(pin, pinset);
  bcm_gpio_fallingedge(pin, pinset);
  bcm_gpio_highlevel(pin, pinset);
  bcm_gpio_lowlevel(pin, pinset);
  bcm_gpio_async_risingedge(pin, pinset);
  bcm_gpio_async_fallingedge(pin, pinset);
}

/************************************************************************************
 * Name: bcm_gpio_irqdisable
 *
 * Description:
 *   Reset interrupt event detection for the specified GPIO pin.  This
 *   effective disables the pin interrupts.
 *
 ************************************************************************************/

void bcm_gpio_irqdisable(gpio_pinset_t pinset);
{
  unsigned int pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

  /* Reset pin detection settings */

  bcm_gpio_risingedge(pin, 0);
  bcm_gpio_fallingedge(pin, 0);
  bcm_gpio_highlevel(pin, 0);
  bcm_gpio_lowlevel(pin, 0);
  bcm_gpio_async_risingedge(pin, 0);
  bcm_gpio_async_fallingedge(pin, 0);
}

#endif /* CONFIG_BCM2708_GPIO_IRQ */
