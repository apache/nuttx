/****************************************************************************
 * arch/arm/src/tiva/tiva_gpioirq.c
 *
 *   Copyright (C) 2009-2010, 2012, 2014-2016 Gregory Nutt. All rights reserved.
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
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <debug.h>

#include <arch/board/board.h>

#include "chip.h"

#include "up_internal.h"
#include "up_arch.h"
#include "irq/irq.h"

#include "tiva_gpio.h"

#ifdef CONFIG_TIVA_GPIO_IRQS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TIVA_NPINS 8
#define TIVA_NIRQ_PINS (TIVA_NPORTS * TIVA_NPINS)
#define TIVA_GPIO_IRQ_IDX(port,pin) ((port*TIVA_NPINS)+(pin))

/****************************************************************************
 * Private types
 ****************************************************************************/

struct gpio_handler_s
{
  xcpt_t isr;    /* Interrupt service routine entry point */
  void  *arg;    /* The argument that accompanies the interrupt */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A table of handlers for each GPIO port interrupt */

static struct gpio_handler_s g_gpioportirqvector[TIVA_NIRQ_PINS];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gpioport2irq
 *
 * Description:
 *   Translates from GPIO port to GPIO IRQ.
 *
 ****************************************************************************/

static int gpioport2irq(uint8_t port)
{
  int irq = -1;

  switch (port)
    {
#ifdef CONFIG_TIVA_GPIOA_IRQS
      case (GPIO_PORTA >> GPIO_PORT_SHIFT):
        {
          irq = TIVA_IRQ_GPIOA;
        }
        break;
#endif

#ifdef CONFIG_TIVA_GPIOB_IRQS
      case (GPIO_PORTB >> GPIO_PORT_SHIFT):
        {
          irq = TIVA_IRQ_GPIOB;
        }
        break;
#endif

#ifdef CONFIG_TIVA_GPIOC_IRQS
      case (GPIO_PORTC >> GPIO_PORT_SHIFT):
        {
          irq = TIVA_IRQ_GPIOC;
        }
        break;
#endif

#ifdef CONFIG_TIVA_GPIOD_IRQS
      case (GPIO_PORTD >> GPIO_PORT_SHIFT):
        {
          irq = TIVA_IRQ_GPIOD;
        }
        break;
#endif

#ifdef CONFIG_TIVA_GPIOE_IRQS
      case (GPIO_PORTE >> GPIO_PORT_SHIFT):
        {
          irq = TIVA_IRQ_GPIOE;
        }
        break;
#endif

#ifdef CONFIG_TIVA_GPIOF_IRQS
      case (GPIO_PORTF >> GPIO_PORT_SHIFT):
        {
          irq = TIVA_IRQ_GPIOF;
        }
        break;
#endif

#ifdef CONFIG_TIVA_GPIOG_IRQS
      case (GPIO_PORTG >> GPIO_PORT_SHIFT):
        {
          irq = TIVA_IRQ_GPIOG;
        }
        break;
#endif

#ifdef CONFIG_TIVA_GPIOH_IRQS
      case (GPIO_PORTH >> GPIO_PORT_SHIFT):
        {
          irq = TIVA_IRQ_GPIOH;
        }
        break;
#endif

#ifdef CONFIG_TIVA_GPIOJ_IRQS
      case (GPIO_PORTJ >> GPIO_PORT_SHIFT):
        {
          irq = TIVA_IRQ_GPIOJ;
        }
        break;
#endif

#ifdef CONFIG_TIVA_GPIOK_IRQS
      case (GPIO_PORTK >> GPIO_PORT_SHIFT):
        {
          irq = TIVA_IRQ_GPIOK;
        }
        break;
#endif

#ifdef CONFIG_TIVA_GPIOL_IRQS
      case (GPIO_PORTL >> GPIO_PORT_SHIFT):
        {
          irq = TIVA_IRQ_GPIOL;
        }
        break;
#endif

#ifdef CONFIG_TIVA_GPIOM_IRQS
      case (GPIO_PORTM >> GPIO_PORT_SHIFT):
        {
          irq = TIVA_IRQ_GPIOM;
        }
        break;
#endif

#ifdef CONFIG_TIVA_GPION_IRQS
      case (GPIO_PORTN >> GPIO_PORT_SHIFT):
        {
          irq = TIVA_IRQ_GPION;
        }
        break;
#endif
#ifdef CONFIG_TIVA_GPIOP_IRQS
      case (GPIO_PORTP >> GPIO_PORT_SHIFT):
        {
          irq = TIVA_IRQ_GPIOP;
        }
        break;
#endif

#ifdef CONFIG_TIVA_GPIOQ_IRQS
      case (GPIO_PORTQ >> GPIO_PORT_SHIFT):
        {
          irq = TIVA_IRQ_GPIOQ;
        }
        break;
#endif

#ifdef CONFIG_TIVA_GPIOR_IRQS
      case (GPIO_PORTR >> GPIO_PORT_SHIFT):
        {
          irq = TIVA_IRQ_GPIOR;
        }
        break;
#endif

#ifdef CONFIG_TIVA_GPIOS_IRQS
      case (GPIO_PORTS >> GPIO_PORT_SHIFT):
        {
          irq = TIVA_IRQ_GPIOS;
        }
        break;
#endif
    }

  return irq;
}

/****************************************************************************
 * Name: tiva_gpioirqstatus
 *
 * Description:
 *   Returns raw or masked interrupt status.
 *
 ****************************************************************************/

uint32_t tiva_gpioirqstatus(uint8_t port, bool masked)
{
  uintptr_t base = tiva_gpiobaseaddress(port);

  if (masked)
    {
      return getreg32(base + TIVA_GPIO_MIS_OFFSET);
    }
  else
    {
      return getreg32(base + TIVA_GPIO_RIS_OFFSET);
    }
}

/****************************************************************************
 * Name: tiva_gpioirqclear
 *
 * Description:
 *   Clears the interrupt status of the input base
 *
 ****************************************************************************/

void tiva_gpioirqclear(uint8_t port, uint32_t pinmask)
{
  uintptr_t base = tiva_gpiobaseaddress(port);

  /* "The GPIOICR register is the interrupt clear register. Writing a 1 to a bit
   * in this register clears the corresponding interrupt edge detection logic
   * register. Writing a 0 has no effect."
   */

  modifyreg32(base + TIVA_GPIO_ICR_OFFSET, 0, pinmask);
}

/****************************************************************************
 * Name: tiva_gpioporthandler
 *
 * Description:
 *   Handle interrupts on each enabled GPIO port
 *
 ****************************************************************************/

static int tiva_gpioporthandler(uint8_t port, void *context)
{

  int       irq  = gpioport2irq(port);    /* GPIO port interrupt vector */
  uint32_t  mis  = tiva_gpioirqstatus(port, true); /* Masked Interrupt Status */
  uint8_t   pin;                                   /* Pin number */

  tiva_gpioirqclear(port, 0xff);
  gpioinfo("mis=0b%08b\n", mis & 0xff);

  /* Now process each IRQ pending in the MIS */

  if (mis != 0)
    {
      for (pin = 0; pin < TIVA_NPINS; ++pin)
        {
          if (((mis >> pin) & 1) != 0)
            {
              int index = TIVA_GPIO_IRQ_IDX(port, pin);
              FAR struct gpio_handler_s *handler = &g_gpioportirqvector[index];

              gpioinfo("port=%d pin=%d isr=%p arg=%p index=%d\n",
                       port, pin, handler->isr, handler->arg, index);

              handler->isr(irq, context, handler->arg);
            }
        }
    }

  return OK;
}

#ifdef CONFIG_TIVA_GPIOA_IRQS
static int tiva_gpioahandler(int irq, FAR void *context, FAR void *arg)
{
  irqstate_t flags;
  flags = enter_critical_section();
  up_disable_irq(irq);
  int ret = tiva_gpioporthandler((GPIO_PORTA >> GPIO_PORT_SHIFT), context);
  up_enable_irq(irq);
  leave_critical_section(flags);
  return ret;
}
#endif

#ifdef CONFIG_TIVA_GPIOB_IRQS
static int tiva_gpiobhandler(int irq, FAR void *context, FAR void *arg)
{
  irqstate_t flags;
  flags = enter_critical_section();
  up_disable_irq(irq);
  int ret = tiva_gpioporthandler((GPIO_PORTB >> GPIO_PORT_SHIFT), context);
  up_enable_irq(irq);
  leave_critical_section(flags);
  return ret;
}
#endif

#ifdef CONFIG_TIVA_GPIOC_IRQS
static int tiva_gpiochandler(int irq, FAR void *context, FAR void *arg)
{
  irqstate_t flags;
  flags = enter_critical_section();
  up_disable_irq(irq);
  int ret = tiva_gpioporthandler((GPIO_PORTC >> GPIO_PORT_SHIFT), context);
  up_enable_irq(irq);
  leave_critical_section(flags);
  return ret;
}
#endif

#ifdef CONFIG_TIVA_GPIOD_IRQS
static int tiva_gpiodhandler(int irq, FAR void *context, FAR void *arg)
{
  irqstate_t flags;
  flags = enter_critical_section();
  up_disable_irq(irq);
  int ret = tiva_gpioporthandler((GPIO_PORTD >> GPIO_PORT_SHIFT), context);
  up_enable_irq(irq);
  leave_critical_section(flags);
  return ret;
}
#endif

#ifdef CONFIG_TIVA_GPIOE_IRQS
static int tiva_gpioehandler(int irq, FAR void *context, FAR void *arg)
{
  irqstate_t flags;
  flags = enter_critical_section();
  up_disable_irq(irq);
  int ret = tiva_gpioporthandler((GPIO_PORTE >> GPIO_PORT_SHIFT), context);
  up_enable_irq(irq);
  leave_critical_section(flags);
  return ret;
}
#endif

#ifdef CONFIG_TIVA_GPIOF_IRQS
static int tiva_gpiofhandler(int irq, FAR void *context, FAR void *arg)
{
  irqstate_t flags;
  flags = enter_critical_section();
  up_disable_irq(irq);
  int ret = tiva_gpioporthandler((GPIO_PORTF >> GPIO_PORT_SHIFT), context);
  up_enable_irq(irq);
  leave_critical_section(flags);
  return ret;
}
#endif

#ifdef CONFIG_TIVA_GPIOG_IRQS
static int tiva_gpioghandler(int irq, FAR void *context, FAR void *arg)
{
  irqstate_t flags;
  flags = enter_critical_section();
  up_disable_irq(irq);
  int ret = tiva_gpioporthandler((GPIO_PORTG >> GPIO_PORT_SHIFT), context);
  up_enable_irq(irq);
  leave_critical_section(flags);
  return ret;
}
#endif

#ifdef CONFIG_TIVA_GPIOH_IRQS
static int tiva_gpiohhandler(int irq, FAR void *context, FAR void *arg)
{
  irqstate_t flags;
  flags = enter_critical_section();
  up_disable_irq(irq);
  int ret = tiva_gpioporthandler((GPIO_PORTH >> GPIO_PORT_SHIFT), context);
  up_enable_irq(irq);
  leave_critical_section(flags);
  return ret;
}
#endif

#ifdef CONFIG_TIVA_GPIOJ_IRQS
static int tiva_gpiojhandler(int irq, FAR void *context, FAR void *arg)
{
  irqstate_t flags;
  flags = enter_critical_section();
  up_disable_irq(irq);
  int ret = tiva_gpioporthandler((GPIO_PORTJ >> GPIO_PORT_SHIFT), context);
  up_enable_irq(irq);
  leave_critical_section(flags);
  return ret;
}
#endif

#ifdef CONFIG_TIVA_GPIOK_IRQS
static int tiva_gpiokhandler(int irq, FAR void *context, FAR void *arg)
{
  irqstate_t flags;
  flags = enter_critical_section();
  up_disable_irq(irq);
  int ret = tiva_gpioporthandler((GPIO_PORTK >> GPIO_PORT_SHIFT), context);
  up_enable_irq(irq);
  leave_critical_section(flags);
  return ret;
}
#endif

#ifdef CONFIG_TIVA_GPIOL_IRQS
static int tiva_gpiolhandler(int irq, FAR void *context, FAR void *arg)
{
  irqstate_t flags;
  flags = enter_critical_section();
  up_disable_irq(irq);
  int ret = tiva_gpioporthandler((GPIO_PORTL >> GPIO_PORT_SHIFT), context);
  up_enable_irq(irq);
  leave_critical_section(flags);
  return ret;
}
#endif

#ifdef CONFIG_TIVA_GPIOM_IRQS
static int tiva_gpiomhandler(int irq, FAR void *context, FAR void *arg)
{
  irqstate_t flags;
  flags = enter_critical_section();
  up_disable_irq(irq);
  int ret = tiva_gpioporthandler((GPIO_PORTM >> GPIO_PORT_SHIFT), context);
  up_enable_irq(irq);
  leave_critical_section(flags);
  return ret;
}
#endif

#ifdef CONFIG_TIVA_GPION_IRQS
static int tiva_gpionhandler(int irq, FAR void *context, FAR void *arg)
{
  irqstate_t flags;
  flags = enter_critical_section();
  up_disable_irq(irq);
  int ret = tiva_gpioporthandler((GPIO_PORTN >> GPIO_PORT_SHIFT), context);
  up_enable_irq(irq);
  leave_critical_section(flags);
  return ret;
}
#endif

#ifdef CONFIG_TIVA_GPIOP_IRQS
static int tiva_gpiophandler(int irq, FAR void *context, FAR void *arg)
{
  irqstate_t flags;
  flags = enter_critical_section();
  up_disable_irq(irq);
  int ret = tiva_gpioporthandler((GPIO_PORTP >> GPIO_PORT_SHIFT), context);
  up_enable_irq(irq);
  leave_critical_section(flags);
  return ret;
}
#endif

#ifdef CONFIG_TIVA_GPIOQ_IRQS
static int tiva_gpioqhandler(int irq, FAR void *context, FAR void *arg)
{
  irqstate_t flags;
  flags = enter_critical_section();
  up_disable_irq(irq);
  int ret = tiva_gpioporthandler((GPIO_PORTQ >> GPIO_PORT_SHIFT), context);
  up_enable_irq(irq);
  leave_critical_section(flags);
  return ret;
}
#endif

#ifdef CONFIG_TIVA_GPIOR_IRQS
static int tiva_gpiorhandler(int irq, FAR void *context, FAR void *arg)
{
  irqstate_t flags;
  flags = enter_critical_section();
  up_disable_irq(irq);
  int ret = tiva_gpioporthandler((GPIO_PORTR >> GPIO_PORT_SHIFT), context);
  up_enable_irq(irq);
  leave_critical_section(flags);
  return ret;
}
#endif

#ifdef CONFIG_TIVA_GPIOS_IRQS
static int tiva_gpioshandler(int irq, FAR void *context, FAR void *arg)
{
  irqstate_t flags;
  flags = enter_critical_section();
  up_disable_irq(irq);
  int ret = tiva_gpioporthandler((GPIO_PORTS >> GPIO_PORT_SHIFT), context);
  up_enable_irq(irq);
  leave_critical_section(flags);
  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tiva_gpioirqinitialize
 *
 * Description:
 *   Initialize all vectors to the unexpected interrupt handler
 *
 ****************************************************************************/

int tiva_gpioirqinitialize(void)
{
  int i;

  /* Point all interrupt vectors to the unexpected interrupt */

  for (i = 0; i < TIVA_NIRQ_PINS; ++i)
    {
      g_gpioportirqvector[i].isr = irq_unexpected_isr;
      g_gpioportirqvector[i].arg = NULL;
    }

  gpioinfo("tiva_gpioirqinitialize isr=%d/%d irq_unexpected_isr=%p\n",
           i, TIVA_NIRQ_PINS, irq_unexpected_isr);

  /* Then attach each GPIO interrupt handlers and enable corresponding GPIO
   * interrupts
   */

#ifdef CONFIG_TIVA_GPIOA_IRQS
  irq_attach(TIVA_IRQ_GPIOA, tiva_gpioahandler, NULL);
  up_enable_irq(TIVA_IRQ_GPIOA);
#endif

#ifdef CONFIG_TIVA_GPIOB_IRQS
  irq_attach(TIVA_IRQ_GPIOB, tiva_gpiobhandler, NULL);
  up_enable_irq(TIVA_IRQ_GPIOB);
#endif

#ifdef CONFIG_TIVA_GPIOC_IRQS
  irq_attach(TIVA_IRQ_GPIOC, tiva_gpiochandler, NULL);
  up_enable_irq(TIVA_IRQ_GPIOC);
#endif

#ifdef CONFIG_TIVA_GPIOD_IRQS
  irq_attach(TIVA_IRQ_GPIOD, tiva_gpiodhandler, NULL);
  up_enable_irq(TIVA_IRQ_GPIOD);
#endif

#ifdef CONFIG_TIVA_GPIOE_IRQS
  irq_attach(TIVA_IRQ_GPIOE, tiva_gpioehandler, NULL);
  up_enable_irq(TIVA_IRQ_GPIOE);
#endif

#ifdef CONFIG_TIVA_GPIOF_IRQS
  irq_attach(TIVA_IRQ_GPIOF, tiva_gpiofhandler, NULL);
  up_enable_irq(TIVA_IRQ_GPIOF);
#endif

#ifdef CONFIG_TIVA_GPIOG_IRQS
  irq_attach(TIVA_IRQ_GPIOG, tiva_gpioghandler, NULL);
  up_enable_irq(TIVA_IRQ_GPIOG);
#endif

#ifdef CONFIG_TIVA_GPIOH_IRQS
  irq_attach(TIVA_IRQ_GPIOH, tiva_gpiohhandler, NULL);
  up_enable_irq(TIVA_IRQ_GPIOH);
#endif

#ifdef CONFIG_TIVA_GPIOJ_IRQS
  irq_attach(TIVA_IRQ_GPIOJ, tiva_gpiojhandler, NULL);
  up_enable_irq(TIVA_IRQ_GPIOJ);
#endif

#ifdef CONFIG_TIVA_GPIOK_IRQS
  irq_attach(TIVA_IRQ_GPIOK, tiva_gpiokhandler, NULL);
  up_enable_irq(TIVA_IRQ_GPIOK);
#endif

#ifdef CONFIG_TIVA_GPIOL_IRQS
  irq_attach(TIVA_IRQ_GPIOL, tiva_gpiolhandler, NULL);
  up_enable_irq(TIVA_IRQ_GPIOL);
#endif

#ifdef CONFIG_TIVA_GPIOM_IRQS
  irq_attach(TIVA_IRQ_GPIOM, tiva_gpiomhandler, NULL);
  up_enable_irq(TIVA_IRQ_GPIOM);
#endif

#ifdef CONFIG_TIVA_GPION_IRQS
  irq_attach(TIVA_IRQ_GPION, tiva_gpionhandler, NULL);
  up_enable_irq(TIVA_IRQ_GPION);
#endif

#ifdef CONFIG_TIVA_GPIOP_IRQS
  irq_attach(TIVA_IRQ_GPIOP, tiva_gpiophandler, NULL);
  up_enable_irq(TIVA_IRQ_GPIOP);
#endif

#ifdef CONFIG_TIVA_GPIOQ_IRQS
  irq_attach(TIVA_IRQ_GPIOQ, tiva_gpioqhandler, NULL);
  up_enable_irq(TIVA_IRQ_GPIOQ);
#endif

#ifdef CONFIG_TIVA_GPIOR_IRQS
  irq_attach(TIVA_IRQ_GPIOR, tiva_gpiorhandler, NULL);
  up_enable_irq(TIVA_IRQ_GPIOR);
#endif

#ifdef CONFIG_TIVA_GPIOS_IRQS
  irq_attach(TIVA_IRQ_GPIOS, tiva_gpioshandler, NULL);
  up_enable_irq(TIVA_IRQ_GPIOS);
#endif

  return OK;
}

/****************************************************************************
 * Name: tiva_gpioirqattach
 *
 * Description:
 *   Attach in GPIO interrupt to the provided 'isr'. If isr==NULL, then the
 *   irq_unexpected_isr handler is assigned and the pin's interrupt mask is
 *   disabled to stop further interrupts. Otherwise, the new isr is linked
 *   and the pin's interrupt mask is set.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise a negated errno value is
 *   return to indicate the nature of the failure.
 *
 ****************************************************************************/

int tiva_gpioirqattach(uint32_t pinset, xcpt_t isr, void *arg)
{
  FAR struct gpio_handler_s *handler;
  irqstate_t flags;
  uint8_t    port  = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  uint8_t    pinno = (pinset & GPIO_PIN_MASK);
  uint8_t    pin   = 1 << pinno;

  /* Assign per-pin interrupt handlers */

  if (port < TIVA_NPORTS)
    {
      flags = enter_critical_section();

      /* If the new ISR is NULL, then the ISR is being detached.
       * In this case, disable the ISR and direct any interrupts
       * to the unexpected interrupt handler.
       */

      gpioinfo("assign port=%d pin=%d function=%p to idx=%d\n",
               port, pinno, isr, TIVA_GPIO_IRQ_IDX(port, pinno));

      handler = &g_gpioportirqvector[TIVA_GPIO_IRQ_IDX(port, pinno)];
      if (isr == NULL)
        {
          tiva_gpioirqdisable(port, pin);
          handler->isr = irq_unexpected_isr;
          handler->arg = NULL;
        }
      else
        {
          handler->isr = isr;
          handler->arg = arg;
          tiva_gpioirqenable(port, pin);
        }

      leave_critical_section(flags);
    }

  return OK;
}

/****************************************************************************
 * Name: tiva_gpioportirqattach
 *
 * Description:
 *   Attach 'isr' to the GPIO port. Only use this if you want to handle
 *   the entire ports interrupts explicitly.
 *
 ****************************************************************************/

void tiva_gpioportirqattach(uint8_t port, xcpt_t isr)
{
  irqstate_t flags;
  int irq = gpioport2irq(port);

  /* assign port interrupt handler */

  if (port < TIVA_NPORTS)
    {
      flags = enter_critical_section();

      /* If the new ISR is NULL, then the ISR is being detached.
       * In this case, disable the ISR and direct any interrupts
       * to the unexpected interrupt handler.
       */

      gpioinfo("assign function=%p to port=%d\n", isr, port);

      if (isr == NULL)
        {
          tiva_gpioirqdisable(port, 0xff);
          irq_attach(irq, irq_unexpected_isr, NULL);
        }
      else
        {
          irq_attach(irq, isr, NULL);
          tiva_gpioirqenable(port, 0xff);
        }

      leave_critical_section(flags);
    }
}

/****************************************************************************
 * Name: tiva_gpioirqenable
 *
 * Description:
 *   Enable the GPIO port IRQ
 *
 ****************************************************************************/

void tiva_gpioirqenable(uint8_t port, uint8_t pin)
{
  uintptr_t base = tiva_gpiobaseaddress(port);

  /* Enable the GPIO interrupt. "The GPIO IM register is the interrupt
   * mask register. Bits set to High in GPIO IM allow the corresponding
   * pins to trigger their individual interrupts and the combined GPIO INTR
   * line. Clearing a bit disables interrupt triggering on that pin. All
   * bits are cleared by a reset.
   */

  modifyreg32(base + TIVA_GPIO_IM_OFFSET, 0, pin);
}

/****************************************************************************
 * Name: tiva_gpioirqdisable
 *
 * Description:
 *   Disable the GPIO port IRQ
 *
 ****************************************************************************/

void tiva_gpioirqdisable(uint8_t port, uint8_t pin)
{
  uintptr_t base = tiva_gpiobaseaddress(port);

  /* Disable the GPIO interrupt. "The GPIO IM register is the interrupt
   * mask register. Bits set to High in GPIO IM allow the corresponding
   * pins to trigger their individual interrupts and the combined GPIO INTR
   * line. Clearing a bit disables interrupt triggering on that pin. All
   * bits are cleared by a reset.
   */

  modifyreg32(base + TIVA_GPIO_IM_OFFSET, pin, 0);
}

#endif /* CONFIG_TIVA_GPIO_IRQS */
