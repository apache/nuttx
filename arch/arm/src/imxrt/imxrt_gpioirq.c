/****************************************************************************
 * arch/arm/src/imxrt/imxrt_gpioirq.c
 *
 *   Copyright (C) 2018-2019 Gregory Nutt. All rights reserved.
 *   Author:  Gregory Nutt <gnutt@nuttx.org>
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

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "arm_arch.h"

#include "imxrt_config.h"
#include "imxrt_irq.h"
#include "imxrt_gpio.h"

#ifdef CONFIG_IMXRT_GPIO_IRQ

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(_IMXRT_GPIO6_0_15_BASE)
#  define _IMXRT_FOLLOWS_GPIO6_16_31 _IMXRT_GPIO6_0_15_BASE
#else
#  define _IMXRT_FOLLOWS_GPIO6_16_31 IMXRT_GPIO_IRQ_LAST
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_gpio_info
 *
 * Description:
 *   Given an IRQ number, provide the register and bit setting to enable or
 *   disable the irq.
 *
 ****************************************************************************/

static int imxrt_gpio_info(int irq, uintptr_t *regaddr, unsigned int *pin)
{
  DEBUGASSERT(irq >= IMXRT_GPIO_IRQ_FIRST && irq < IMXRT_GPIO_IRQ_LAST);

#ifdef CONFIG_IMXRT_GPIO1_0_15_IRQ
  if (irq < _IMXRT_GPIO1_16_31_BASE)
    {
      *regaddr = IMXRT_GPIO1_IMR;
      *pin     = irq - _IMXRT_GPIO1_0_15_BASE;
    }
  else
#endif
#ifdef CONFIG_IMXRT_GPIO1_16_31_IRQ
  if (irq < _IMXRT_GPIO2_0_15_BASE)
    {
      *regaddr = IMXRT_GPIO1_IMR;
      *pin     = irq - _IMXRT_GPIO1_16_31_BASE + 16;
    }
  else
#endif
#ifdef CONFIG_IMXRT_GPIO2_0_15_IRQ
  if (irq < _IMXRT_GPIO2_16_31_BASE)
    {
      *regaddr = IMXRT_GPIO2_IMR;
      *pin     = irq - _IMXRT_GPIO2_0_15_BASE;
    }
  else
#endif
#ifdef CONFIG_IMXRT_GPIO2_16_31_IRQ
  if (irq < _IMXRT_GPIO3_0_15_BASE)
    {
      *regaddr = IMXRT_GPIO2_IMR;
      *pin     = irq - _IMXRT_GPIO2_16_31_BASE + 16;
    }
  else
#endif
#ifdef CONFIG_IMXRT_GPIO3_0_15_IRQ
  if (irq < _IMXRT_GPIO3_16_31_BASE)
    {
      *regaddr = IMXRT_GPIO3_IMR;
      *pin     = irq - _IMXRT_GPIO3_0_15_BASE;
    }
  else
#endif
#ifdef CONFIG_IMXRT_GPIO3_16_31_IRQ
#ifdef IMXRT_GPIO4_IMR
  if (irq < _IMXRT_GPIO4_0_15_BASE)
#else
  if (irq < _IMXRT_GPIO5_0_15_BASE)
#endif
    {
      *regaddr = IMXRT_GPIO3_IMR;
      *pin     = irq - _IMXRT_GPIO3_16_31_BASE + 16;
    }
  else
#endif
#ifdef IMXRT_GPIO4_IMR
#ifdef CONFIG_IMXRT_GPIO4_0_15_IRQ
  if (irq < _IMXRT_GPIO4_16_31_BASE)
    {
      *regaddr = IMXRT_GPIO4_IMR;
      *pin     = irq - _IMXRT_GPIO4_0_15_BASE;
    }
  else
#endif
#ifdef CONFIG_IMXRT_GPIO4_16_31_IRQ
  if (irq < _IMXRT_GPIO5_0_15_BASE)
    {
      *regaddr = IMXRT_GPIO4_IMR;
      *pin     = irq - _IMXRT_GPIO4_16_31_BASE + 16;
    }
  else
#endif
#endif
#ifdef CONFIG_IMXRT_GPIO5_0_15_IRQ
  if (irq < _IMXRT_GPIO5_16_31_BASE)
    {
      *regaddr = IMXRT_GPIO5_IMR;
      *pin     = irq - _IMXRT_GPIO5_0_15_BASE;
    }
  else
#endif
#ifdef CONFIG_IMXRT_GPIO5_16_31_IRQ
  if (irq < _IMXRT_FOLLOWS_GPIO6_16_31)
    {
      *regaddr = IMXRT_GPIO5_IMR;
      *pin     = irq - _IMXRT_GPIO5_16_31_BASE + 16;
    }
  else
#endif
#ifdef CONFIG_IMXRT_GPIO6_0_15_IRQ
  if (irq < _IMXRT_GPIO6_16_31_BASE)
    {
      *regaddr = IMXRT_GPIO6_IMR;
      *pin     = irq - _IMXRT_GPIO6_0_15_BASE;
    }
  else
#endif
#ifdef CONFIG_IMXRT_GPIO6_16_31_IRQ
  if (irq < _IMXRT_GPIO7_0_15_BASE)
    {
      *regaddr = IMXRT_GPIO6_IMR;
      *pin     = irq - _IMXRT_GPIO6_16_31_BASE + 16;
    }
  else
#endif
#ifdef CONFIG_IMXRT_GPIO7_0_15_IRQ
  if (irq < _IMXRT_GPIO7_16_31_BASE)
    {
      *regaddr = IMXRT_GPIO7_IMR;
      *pin     = irq - _IMXRT_GPIO7_0_15_BASE;
    }
  else
#endif
#ifdef CONFIG_IMXRT_GPIO7_16_31_IRQ
  if (irq < _IMXRT_GPIO8_0_15_BASE)
    {
      *regaddr = IMXRT_GPIO7_IMR;
      *pin     = irq - _IMXRT_GPIO7_16_31_BASE + 16;
    }
  else
#endif
#ifdef CONFIG_IMXRT_GPIO8_0_15_IRQ
  if (irq < _IMXRT_GPIO8_16_31_BASE)
    {
      *regaddr = IMXRT_GPIO8_IMR;
      *pin     = irq - _IMXRT_GPIO8_0_15_BASE;
    }
  else
#endif
#ifdef CONFIG_IMXRT_GPIO8_16_31_IRQ
  if (irq < _IMXRT_GPIO9_0_15_BASE)
    {
      *regaddr = IMXRT_GPIO8_IMR;
      *pin     = irq - _IMXRT_GPIO8_16_31_BASE + 16;
    }
  else
#endif
#ifdef CONFIG_IMXRT_GPIO9_0_15_IRQ
  if (irq < _IMXRT_GPIO9_16_31_BASE)
    {
      *regaddr = IMXRT_GPIO9_IMR;
      *pin     = irq - _IMXRT_GPIO9_0_15_BASE;
    }
  else
#endif
#ifdef CONFIG_IMXRT_GPIO9_16_31_IRQ
  if (irq < IMXRT_GPIO_IRQ_LAST)
    {
      *regaddr = IMXRT_GPIO9_IMR;
      *pin     = irq - _IMXRT_GPIO9_16_31_BASE + 16;
    }
  else
#endif
    {
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: imxrt_gpioN_A_B_interrupt
 *
 * Description:
 *   GPIO interrupt handlers.
 *
 ****************************************************************************/

#ifdef CONFIG_IMXRT_GPIO1_0_15_IRQ
static int imxrt_gpio1_0_15_interrupt(int irq, FAR void *context,
                                      FAR void *arg)
{
  uint32_t status;
  int gpioirq;
  int bit;

  /* Get the pending interrupt indications */

  status = getreg32(IMXRT_GPIO1_ISR) & getreg32(IMXRT_GPIO1_IMR) &
           0x0000fffff;

  /* Decode the pending interrupts */

  for (bit = 0, gpioirq = _IMXRT_GPIO1_0_15_BASE;
       bit < 16 && status != 0;
       bit++, gpioirq++)
    {
      /* Is the IRQ associate with this pin pending? */

      uint32_t mask = (1 << bit);
      if ((status & mask) != 0)
        {
          /* Yes, clear the status bit and dispatch the interrupt */

          putreg32(mask, IMXRT_GPIO1_ISR);
          status &= ~mask;

          irq_dispatch(gpioirq, context);
        }
    }

  return OK;
}
#endif

#ifdef CONFIG_IMXRT_GPIO1_16_31_IRQ
static int imxrt_gpio1_16_31_interrupt(int irq, FAR void *context,
                                       FAR void *arg)
{
  uint32_t status;
  int gpioirq;
  int bit;

  /* Get the pending interrupt indications */

  status = getreg32(IMXRT_GPIO1_ISR) & getreg32(IMXRT_GPIO1_IMR) &
           0xffff0000;

  /* Decode the pending interrupts */

  for (bit = 16, gpioirq = _IMXRT_GPIO1_16_31_BASE;
       bit < 32 && status != 0;
       bit++, gpioirq++)
    {
      /* Is the IRQ associate with this pin pending? */

      uint32_t mask = (1 << bit);
      if ((status & mask) != 0)
        {
          /* Yes, clear the status bit and dispatch the interrupt */

          putreg32(mask, IMXRT_GPIO1_ISR);
          status &= ~mask;

          irq_dispatch(gpioirq, context);
        }
    }

  return OK;
}
#endif

#ifdef CONFIG_IMXRT_GPIO2_0_15_IRQ
static int imxrt_gpio2_0_15_interrupt(int irq, FAR void *context,
                                      FAR void *arg)
{
  uint32_t status;
  int gpioirq;
  int bit;

  /* Get the pending interrupt indications */

  status = getreg32(IMXRT_GPIO2_ISR) & getreg32(IMXRT_GPIO2_IMR) &
           0x0000fffff;

  /* Decode the pending interrupts */

  for (bit = 0, gpioirq = _IMXRT_GPIO2_0_15_BASE;
       bit < 16 && status != 0;
       bit++, gpioirq++)
    {
      /* Is the IRQ associate with this pin pending? */

      uint32_t mask = (1 << bit);
      if ((status & mask) != 0)
        {
          /* Yes, clear the status bit and dispatch the interrupt */

          putreg32(mask, IMXRT_GPIO2_ISR);
          status &= ~mask;

          irq_dispatch(gpioirq, context);
        }
    }

  return OK;
}
#endif

#ifdef CONFIG_IMXRT_GPIO2_16_31_IRQ
static int imxrt_gpio2_16_31_interrupt(int irq, FAR void *context,
                                       FAR void *arg)
{
  uint32_t status;
  int gpioirq;
  int bit;

  /* Get the pending interrupt indications */

  status = getreg32(IMXRT_GPIO2_ISR) & getreg32(IMXRT_GPIO2_IMR) &
           0xffff0000;

  /* Decode the pending interrupts */

  for (bit = 16, gpioirq = _IMXRT_GPIO2_16_31_BASE;
       bit < 32 && status != 0;
       bit++, gpioirq++)
    {
      /* Is the IRQ associate with this pin pending? */

      uint32_t mask = (1 << bit);
      if ((status & mask) != 0)
        {
          /* Yes, clear the status bit and dispatch the interrupt */

          putreg32(mask, IMXRT_GPIO2_ISR);
          status &= ~mask;

          irq_dispatch(gpioirq, context);
        }
    }

  return OK;
}
#endif

#ifdef CONFIG_IMXRT_GPIO3_0_15_IRQ
static int imxrt_gpio3_0_15_interrupt(int irq, FAR void *context,
                                      FAR void *arg)
{
  uint32_t status;
  int gpioirq;
  int bit;

  /* Get the pending interrupt indications */

  status = getreg32(IMXRT_GPIO3_ISR) & getreg32(IMXRT_GPIO3_IMR) &
           0x0000fffff;

  /* Decode the pending interrupts */

  for (bit = 0, gpioirq = _IMXRT_GPIO3_0_15_BASE;
       bit < 16 && status != 0;
       bit++, gpioirq++)
    {
      /* Is the IRQ associate with this pin pending? */

      uint32_t mask = (1 << bit);
      if ((status & mask) != 0)
        {
          /* Yes, clear the status bit and dispatch the interrupt */

          putreg32(mask, IMXRT_GPIO3_ISR);
          status &= ~mask;

          irq_dispatch(gpioirq, context);
        }
    }

  return OK;
}
#endif

#ifdef CONFIG_IMXRT_GPIO3_16_31_IRQ
static int imxrt_gpio3_16_31_interrupt(int irq, FAR void *context,
                                       FAR void *arg)
{
  uint32_t status;
  int gpioirq;
  int bit;

  /* Get the pending interrupt indications */

  status = getreg32(IMXRT_GPIO3_ISR) & getreg32(IMXRT_GPIO3_IMR) &
           0xffff0000;

  /* Decode the pending interrupts */

  for (bit = 16, gpioirq = _IMXRT_GPIO3_16_31_BASE;
       bit < 32 && status != 0;
       bit++, gpioirq++)
    {
      /* Is the IRQ associate with this pin pending? */

      uint32_t mask = (1 << bit);
      if ((status & mask) != 0)
        {
          /* Yes, clear the status bit and dispatch the interrupt */

          putreg32(mask, IMXRT_GPIO3_ISR);
          status &= ~mask;

          irq_dispatch(gpioirq, context);
        }
    }

  return OK;
}
#endif

#ifdef IMXRT_GPIO4_IMR
#ifdef CONFIG_IMXRT_GPIO4_0_15_IRQ
static int imxrt_gpio4_0_15_interrupt(int irq, FAR void *context,
                                      FAR void *arg)
{
  uint32_t status;
  int gpioirq;
  int bit;

  /* Get the pending interrupt indications */

  status = getreg32(IMXRT_GPIO4_ISR) & getreg32(IMXRT_GPIO4_IMR) &
           0x0000fffff;

  /* Decode the pending interrupts */

  for (bit = 0, gpioirq = _IMXRT_GPIO4_0_15_BASE;
       bit < 16 && status != 0;
       bit++, gpioirq++)
    {
      /* Is the IRQ associate with this pin pending? */

      uint32_t mask = (1 << bit);
      if ((status & mask) != 0)
        {
          /* Yes, clear the status bit and dispatch the interrupt */

          putreg32(mask, IMXRT_GPIO4_ISR);
          status &= ~mask;

          irq_dispatch(gpioirq, context);
        }
    }

  return OK;
}
#endif

#ifdef CONFIG_IMXRT_GPIO4_16_31_IRQ
static int imxrt_gpio4_16_31_interrupt(int irq, FAR void *context,
                                       FAR void *arg)
{
  uint32_t status;
  int gpioirq;
  int bit;

  /* Get the pending interrupt indications */

  status = getreg32(IMXRT_GPIO4_ISR) & getreg32(IMXRT_GPIO4_IMR) &
           0xffff0000;

  /* Decode the pending interrupts */

  for (bit = 16, gpioirq = _IMXRT_GPIO4_16_31_BASE;
       bit < 32 && status != 0;
       bit++, gpioirq++)
    {
      /* Is the IRQ associate with this pin pending? */

      uint32_t mask = (1 << bit);
      if ((status & mask) != 0)
        {
          /* Yes, clear the status bit and dispatch the interrupt */

          putreg32(mask, IMXRT_GPIO4_ISR);
          status &= ~mask;

          irq_dispatch(gpioirq, context);
        }
    }

  return OK;
}
#endif
#endif

#ifdef CONFIG_IMXRT_GPIO5_0_15_IRQ
static int imxrt_gpio5_0_15_interrupt(int irq, FAR void *context,
                                      FAR void *arg)
{
  uint32_t status;
  int gpioirq;
  int bit;

  /* Get the pending interrupt indications */

  status = getreg32(IMXRT_GPIO5_ISR) & getreg32(IMXRT_GPIO5_IMR) &
           0x0000fffff;

  /* Decode the pending interrupts */

  for (bit = 0, gpioirq = _IMXRT_GPIO5_0_15_BASE;
       bit < 16 && status != 0;
       bit++, gpioirq++)
    {
      /* Is the IRQ associate with this pin pending? */

      uint32_t mask = (1 << bit);
      if ((status & mask) != 0)
        {
          /* Yes, clear the status bit and dispatch the interrupt */

          putreg32(mask, IMXRT_GPIO5_ISR);
          status &= ~mask;

          irq_dispatch(gpioirq, context);
        }
    }

  return OK;
}
#endif

#ifdef CONFIG_IMXRT_GPIO5_16_31_IRQ
static int imxrt_gpio5_16_31_interrupt(int irq, FAR void *context,
                                       FAR void *arg)
{
  uint32_t status;
  int gpioirq;
  int bit;

  /* Get the pending interrupt indications */

  status = getreg32(IMXRT_GPIO5_ISR) & getreg32(IMXRT_GPIO5_IMR) &
           0xffff0000;

  /* Decode the pending interrupts */

  for (bit = 16, gpioirq = _IMXRT_GPIO5_16_31_BASE;
       bit < 32 && status != 0;
       bit++, gpioirq++)
    {
      /* Is the IRQ associate with this pin pending? */

      uint32_t mask = (1 << bit);
      if ((status & mask) != 0)
        {
          /* Yes, clear the status bit and dispatch the interrupt */

          putreg32(mask, IMXRT_GPIO5_ISR);
          status &= ~mask;

          irq_dispatch(gpioirq, context);
        }
    }

  return OK;
}

#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_gpioirq_initialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for
 *   GPIO pins.
 *
 ****************************************************************************/

void imxrt_gpioirq_initialize(void)
{
  /* Disable all GPIO interrupts at the source */

  putreg32(0, IMXRT_GPIO1_IMR);
  putreg32(0, IMXRT_GPIO2_IMR);
  putreg32(0, IMXRT_GPIO3_IMR);
#if defined(IMXRT_GPIO4_IMR)
  putreg32(0, IMXRT_GPIO4_IMR);
#endif
  putreg32(0, IMXRT_GPIO5_IMR);

  /* Disable all unconfigured GPIO interrupts at the NVIC */

#ifndef CONFIG_IMXRT_GPIO1_0_15_IRQ
  up_disable_irq(IMXRT_IRQ_GPIO1_0_15);
#endif
#ifndef CONFIG_IMXRT_GPIO1_16_31_IRQ
  up_disable_irq(IMXRT_IRQ_GPIO1_16_31);
#endif
#ifndef CONFIG_IMXRT_GPIO2_0_15_IRQ
  up_disable_irq(IMXRT_IRQ_GPIO2_0_15);
#endif
#ifndef CONFIG_IMXRT_GPIO2_16_31_IRQ
  up_disable_irq(IMXRT_IRQ_GPIO2_16_31);
#endif
#ifndef CONFIG_IMXRT_GPIO3_0_15_IRQ
  up_disable_irq(IMXRT_IRQ_GPIO3_0_15);
#endif
#ifndef CONFIG_IMXRT_GPIO3_16_31_IRQ
  up_disable_irq(IMXRT_IRQ_GPIO3_16_31);
#endif
#ifdef IMXRT_GPIO4_IMR
#ifndef CONFIG_IMXRT_GPIO4_0_15_IRQ
  up_disable_irq(IMXRT_IRQ_GPIO4_0_15);
#endif
#ifndef CONFIG_IMXRT_GPIO4_16_31_IRQ
  up_disable_irq(IMXRT_IRQ_GPIO4_16_31);
#endif
#endif
#ifndef CONFIG_IMXRT_GPIO5_0_15_IRQ
  up_disable_irq(IMXRT_IRQ_GPIO5_0_15);
#endif
#ifndef CONFIG_IMXRT_GPIO5_16_31_IRQ
  up_disable_irq(IMXRT_IRQ_GPIO5_16_31);
#endif

  /* Attach all configured GPIO interrupts and enable the interrupt at the
   * NVIC
   */

#ifdef CONFIG_IMXRT_GPIO1_0_15_IRQ
  DEBUGVERIFY(irq_attach(IMXRT_IRQ_GPIO1_0_15,
                         imxrt_gpio1_0_15_interrupt, NULL));
  up_enable_irq(IMXRT_IRQ_GPIO1_0_15);
#endif

#ifdef CONFIG_IMXRT_GPIO1_16_31_IRQ
  DEBUGVERIFY(irq_attach(IMXRT_IRQ_GPIO1_16_31,
                         imxrt_gpio1_16_31_interrupt, NULL));
  up_enable_irq(IMXRT_IRQ_GPIO1_16_31);
#endif

#ifdef CONFIG_IMXRT_GPIO2_0_15_IRQ
  DEBUGVERIFY(irq_attach(IMXRT_IRQ_GPIO2_0_15,
                         imxrt_gpio2_0_15_interrupt, NULL));
  up_enable_irq(IMXRT_IRQ_GPIO2_0_15);
#endif

#ifdef CONFIG_IMXRT_GPIO2_16_31_IRQ
  DEBUGVERIFY(irq_attach(IMXRT_IRQ_GPIO2_16_31,
                         imxrt_gpio2_16_31_interrupt, NULL));
  up_enable_irq(IMXRT_IRQ_GPIO2_16_31);
#endif

#ifdef CONFIG_IMXRT_GPIO3_0_15_IRQ
  DEBUGVERIFY(irq_attach(IMXRT_IRQ_GPIO3_0_15,
                         imxrt_gpio3_0_15_interrupt, NULL));
  up_enable_irq(IMXRT_IRQ_GPIO3_0_15);
#endif

#ifdef CONFIG_IMXRT_GPIO3_16_31_IRQ
  DEBUGVERIFY(irq_attach(IMXRT_IRQ_GPIO3_16_31,
                         imxrt_gpio3_16_31_interrupt, NULL));
  up_enable_irq(IMXRT_IRQ_GPIO3_16_31);
#endif

#ifdef IMXRT_GPIO4_IMR
#ifdef CONFIG_IMXRT_GPIO4_0_15_IRQ
  DEBUGVERIFY(irq_attach(IMXRT_IRQ_GPIO4_0_15,
                         imxrt_gpio4_0_15_interrupt, NULL));
  up_enable_irq(IMXRT_IRQ_GPIO4_0_15);
#endif

#ifdef CONFIG_IMXRT_GPIO4_16_31_IRQ
  DEBUGVERIFY(irq_attach(IMXRT_IRQ_GPIO4_16_31,
                         imxrt_gpio4_16_31_interrupt, NULL));
  up_enable_irq(IMXRT_IRQ_GPIO4_16_31);
#endif
#endif

#ifdef CONFIG_IMXRT_GPIO5_0_15_IRQ
  DEBUGVERIFY(irq_attach(IMXRT_IRQ_GPIO5_0_15,
                         imxrt_gpio5_0_15_interrupt, NULL));
  up_enable_irq(IMXRT_IRQ_GPIO5_0_15);
#endif

#ifdef CONFIG_IMXRT_GPIO5_16_31_IRQ
  DEBUGVERIFY(irq_attach(IMXRT_IRQ_GPIO5_16_31,
                         imxrt_gpio5_16_31_interrupt, NULL));
  up_enable_irq(IMXRT_IRQ_GPIO5_16_31);
#endif
}

/****************************************************************************
 * Name: imxrt_gpioirq_configure
 *
 * Description:
 *   Configure an interrupt for the specified GPIO pin.
 *
 ****************************************************************************/

int imxrt_gpioirq_configure(gpio_pinset_t pinset)
{
  unsigned int port;
  unsigned int pin;
  uintptr_t regaddr;
  uint32_t regval;
  uint32_t icr;
  uint32_t bothedge;

  /* Decode information in the pin configuration */

  port     = ((unsigned int)pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  pin      = ((unsigned int)pinset & GPIO_PIN_MASK)  >> GPIO_PIN_SHIFT;
  icr      = ((uint32_t)pinset & GPIO_INTCFG_MASK)   >> GPIO_INTCFG_SHIFT;
  bothedge = ((uint32_t)pinset & GPIO_INTBOTHCFG_MASK) >>
             GPIO_INTBOTHCFG_SHIFT;

  /* Set the right field in the right ICR register */

  regaddr = pin < 16 ? IMXRT_GPIO_ICR1(port) : IMXRT_GPIO_ICR2(port);
  regval  = getreg32(regaddr);
  regval &= ~GPIO_ICR_MASK(pin);
  regval |= GPIO_ICR(icr, pin);
  putreg32(regval, regaddr);

  /* Add any both-edge setup (overrides above see User Manual 12.5.9) */

  regaddr = IMXRT_GPIO_EDGE(port);
  regval = getreg32(regaddr);
  regval &= ~GPIO_EDGE_MASK(pin);
  regval |= GPIO_EDGE(bothedge, pin);
  putreg32(regval, regaddr);

  return OK;
}

/****************************************************************************
 * Name: imxrt_gpioirq_enable
 *
 * Description:
 *   Enable the interrupt for specified GPIO IRQ
 *
 ****************************************************************************/

int imxrt_gpioirq_enable(int irq)
{
  uintptr_t regaddr;
  unsigned int pin;
  int ret;

  ret = imxrt_gpio_info(irq, &regaddr, &pin);
  if (ret >= 0)
    {
      modifyreg32(regaddr, 0, 1 << pin);
    }

  return ret;
}

/****************************************************************************
 * Name: imxrt_gpioirq_disable
 *
 * Description:
 *   Disable the interrupt for specified GPIO IRQ
 *
 ****************************************************************************/

int imxrt_gpioirq_disable(int irq)
{
  uintptr_t regaddr;
  unsigned int pin;
  int ret;

  ret = imxrt_gpio_info(irq, &regaddr, &pin);
  if (ret >= 0)
    {
      modifyreg32(regaddr, 1 << pin, 0);
    }

  return ret;
}

#endif /* CONFIG_IMXRT_GPIO_IRQ */
