/************************************************************************************
 * arch/arm/src/efm32/efm32_gpioirq.c
 *
 *   Copyright (C) 2014-2015 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "arm_arch.h"
#include "hardware/efm32_gpio.h"
#include "efm32_gpio.h"
#include "efm32_bitband.h"

#ifdef CONFIG_EFM32_GPIO_IRQ

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: efm32_getport
 *
 * Description:
 *   Extract the encoded port number
 *
 ************************************************************************************/

static inline uint8_t efm32_getport(gpio_pinset_t cfgset)
{
  return (uint8_t)((cfgset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT);
}

/************************************************************************************
 * Name: efm32_getpin
 *
 * Description:
 *   Extract the encoded pin number
 *
 ************************************************************************************/

static inline uint8_t efm32_getpin(gpio_pinset_t cfgset)
{
  return (uint8_t)((cfgset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT);
}

/************************************************************************************
 * Name: efm32_gpio_interrupt
 *
 * Description:
 *   Common GPIO interrupt handling logic
 *
 ************************************************************************************/

static int  efm32_gpio_interrupt(uint32_t mask, void *context)
{
  uint32_t pending;
  uint32_t bit;
  int      irq;

  /* Get the set of even/odd, pending, enabled interrupts */

  pending = getreg32(EFM32_GPIO_IF) & getreg32(EFM32_GPIO_IEN) & mask;
  putreg32(pending, EFM32_GPIO_IFC);

  /* Then dispatch each interrupt */

  for (bit = 1, irq = EFM32_IRQ_EXTI0; pending != 0; bit <<= 1, irq++)
    {
      if ((pending & bit) != 0)
        {
          /* Re-deliver the IRQ (recurses! We got here from irq_dispatch!) */

          irq_dispatch(irq, context);

          /* Remove this from the set of pending interrupts */

          pending &= ~bit;
        }
    }

  return OK;
}

/************************************************************************************
 * Name: efm32_even_interrupt
 *
 * Description:
 *   Even GPIO interrupt handling logic
 *
 ************************************************************************************/

static int efm32_even_interrupt(int irq, void *context, FAR void *arg)
{
  return efm32_gpio_interrupt(0x00005555, context);
}

/************************************************************************************
 * Name: efm32_even_interrupt
 *
 * Description:
 *   Even GPIO interrupt handling logic
 *
 ************************************************************************************/

static int efm32_odd_interrupt(int irq, void *context, FAR void *arg)
{
  return efm32_gpio_interrupt(0x0000aaaa, context);
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: efm32_gpioirqinitialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for PIO pins.
 *
 ************************************************************************************/

void efm32_gpioirqinitialize(void)
{
  /* Initialize GPIO interrupt registers, disabling GPIO interrupts at the source */

  putreg32(0, EFM32_GPIO_EXTIRISE);
  putreg32(0, EFM32_GPIO_EXTIFALL);
  putreg32(0, EFM32_GPIO_IEN);

  /* Attach the even and odd interrupt handlers */

  DEBUGVERIFY(irq_attach(EFM32_IRQ_GPIO_EVEN, efm32_even_interrupt, NULL));
  DEBUGVERIFY(irq_attach(EFM32_IRQ_GPIO_ODD, efm32_odd_interrupt, NULL));

  /* Enable GPIO even and odd interrupts at the NVIC */

  up_enable_irq(EFM32_IRQ_GPIO_ODD);
  up_enable_irq(EFM32_IRQ_GPIO_EVEN);
}

/************************************************************************************
 * Name: efm32_gpioirq
 *
 * Description:
 *   Configure an interrupt for the specified PIO pin.
 *
 ************************************************************************************/

void efm32_gpioirq(gpio_pinset_t pinset)
{
  irqstate_t flags;
  unsigned int shift;
  uintptr_t regaddr;
  uint32_t regval;
  uint32_t bit;
  uint8_t port;
  uint8_t pin;

  /* Get basic pin configuration information */

  port = efm32_getport(pinset);
  pin  = efm32_getpin(pinset);
  bit  = ((uint32_t)1 << pin);

  /* Make sure that the pin interrupt is disabled */

  flags   = enter_critical_section();
  regval  = getreg32(EFM32_GPIO_IEN);
  regval &= ~bit;
  putreg32(regval, EFM32_GPIO_IEN);

  /* Set the interrupt port */

  if (pin < 8)
    {
      regaddr = EFM32_GPIO_EXTIPSELL;
      shift   = (unsigned int)pin << 2;
    }
  else
    {
      regaddr = EFM32_GPIO_EXTIPSELH;
      shift   = (unsigned int)(pin - 8) << 2;
    }

  regval = getreg32(regaddr);
  regval &= ~(7 << shift);
  regval |= ((uint32_t)port << shift);
  putreg32(regval, regaddr);

  /* Set/clear rising edge interrupt detection */

  regval = getreg32(EFM32_GPIO_EXTIRISE);
  if ((pinset & GPIO_INT_RISING) != 0)
    {
      regval |= bit;
    }
  else
    {
      regval &= ~bit;
    }

  putreg32(regval, EFM32_GPIO_EXTIRISE);

  /* Set/clear rising edge interrupt detection */

  regval  = getreg32(EFM32_GPIO_EXTIFALL);
  if ((pinset & GPIO_INT_FALLING) != 0)
    {
      regval |= bit;
    }
  else
    {
      regval &= ~bit;
    }

  putreg32(regval, EFM32_GPIO_EXTIFALL);
  leave_critical_section(flags);
}

/************************************************************************************
 * Name: efm32_gpioirqenable
 *
 * Description:
 *   Enable the interrupt for specified PIO IRQ
 *
 ************************************************************************************/

void efm32_gpioirqenable(int irq)
{

  if (irq >= EFM32_IRQ_EXTI0 && irq <= EFM32_IRQ_EXTI15)
    {
      /* Enable the interrupt associated with the pin */

#ifndef CONFIG_EFM32_BITBAND
      irqstate_t flags;
      uint32_t regval;
      uint32_t bit;
      bit     = ((uint32_t)1 << (irq - EFM32_IRQ_EXTI0));
      flags   = enter_critical_section();
      regval  = getreg32(EFM32_GPIO_IEN);
      regval |= bit;
      putreg32(regval, EFM32_GPIO_IEN);
      leave_critical_section(flags);
#else
      bitband_set_peripheral(EFM32_GPIO_IEN, (irq - EFM32_IRQ_EXTI0), 1);
#endif
    }
}

/************************************************************************************
 * Name: efm32_gpioirqdisable
 *
 * Description:
 *   Disable the interrupt for specified PIO IRQ
 *
 ************************************************************************************/

void efm32_gpioirqdisable(int irq)
{
  if (irq >= EFM32_IRQ_EXTI0 && irq <= EFM32_IRQ_EXTI15)
    {
      /* Enable the interrupt associated with the pin */

#ifndef CONFIG_EFM32_BITBAND
      irqstate_t flags;
      uint32_t regval;
      uint32_t bit;

      bit     = ((uint32_t)1 << (irq - EFM32_IRQ_EXTI0));
      flags   = enter_critical_section();
      regval  = getreg32(EFM32_GPIO_IEN);
      regval &= ~bit;
      putreg32(regval, EFM32_GPIO_IEN);
      leave_critical_section(flags);
#else
      bitband_set_peripheral(EFM32_GPIO_IEN, (irq - EFM32_IRQ_EXTI0), 0);
#endif
    }
}

/************************************************************************************
 * Name: efm32_gpioirqclear
 *
 * Description:
 *   Disable the interrupt for specified PIO IRQ
 *
 ************************************************************************************/

void efm32_gpioirqclear(int irq)
{
  if (irq >= EFM32_IRQ_EXTI0 && irq <= EFM32_IRQ_EXTI15)
    {
      /* Enable the interrupt associated with the pin */

#ifndef CONFIG_EFM32_BITBAND
      irqstate_t flags;
      uint32_t regval;
      uint32_t bit;

      bit     = ((uint32_t)1 << (irq - EFM32_IRQ_EXTI0));
      flags   = enter_critical_section();
      regval  = getreg32(EFM32_GPIO_IFC);
      regval |= bit;
      putreg32(regval, EFM32_GPIO_IFC);
      leave_critical_section(flags);
#else
      bitband_set_peripheral(EFM32_GPIO_IFC, (irq - EFM32_IRQ_EXTI0), 1);
#endif
    }
}

#endif /* CONFIG_EFM32_GPIO_IRQ */
