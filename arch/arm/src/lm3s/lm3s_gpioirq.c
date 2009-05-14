/****************************************************************************
 * arch/arm/src/lm3s/lm3s_gpioirq.c
 * arch/arm/src/chip/lm3s_gpioirq.c
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
#include <sys/types.h>

#include <string.h>
#include <assert.h>
#include <debug.h>

#include <arch/irq.h>

#include "up_arch.h"
#include "os_internal.h"
#include "irq_internal.h"
#include "lm3s_internal.h"

/****************************************************************************
 * Private Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A table of handlers for each GPIO interrupt */

static FAR xcpt_t g_gpioirqvector[NR_GPIO_IRQS];

/* A table that maps a GPIO group to a GPIO base address.  Overly complicated
 * be we support disabling interrupt support for arbitrary ports
 */

static const uint32 g_gpiobase[] =
{
#ifndef CONFIG_LM3S_DISABLE_GPIOA_IRQS
   LM3S_GPIOA_BASE,
#endif
#ifndef CONFIG_LM3S_DISABLE_GPIOB_IRQS
  LM3S_GPIOB_BASE,
#endif
#ifndef CONFIG_LM3S_DISABLE_GPIOC_IRQS
  LM3S_GPIOC_BASE,
#endif
#ifndef CONFIG_LM3S_DISABLE_GPIOD_IRQS
  LM3S_GPIOD_BASE,
#endif
#ifndef CONFIG_LM3S_DISABLE_GPIOE_IRQS
  LM3S_GPIOE_BASE,
#endif
#ifndef CONFIG_LM3S_DISABLE_GPIOF_IRQS
  LM3S_GPIOF_BASE,
#endif
#ifndef CONFIG_LM3S_DISABLE_GPIOG_IRQS
  LM3S_GPIOG_BASE,
#endif
#ifndef CONFIG_LM3S_DISABLE_GPIOH_IRQS
  LM3S_GPIOH_BASE,
#endif
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lm3s_gpiobaseaddress
 *
 * Description:
 *   Given a GPIO enumeration value, return the base address of the
 *   associated GPIO registers.
 *
 ****************************************************************************/

static inline uint32 lm3s_gpiobaseaddress(unsigned int port)
{
  return g_gpiobase[port >> 3];
}

/****************************************************************************
 * Name: lm3s_gpio*handler
 *
 * Description:
 *   Handle interrupts on each enabled GPIO port
 *
 ****************************************************************************/

static int lm3s_gpiohandler(uint32 regbase, int irqbase, void *context)
{
  uint32 mis;
  int irq;
  int pin;

  /* Handle each pending GPIO interrupt.  "The GPIO MIS register is the masked
   * interrupt status register. Bits read High in GPIO MIS reflect the status
   * of input lines triggering an interrupt. Bits read as Low indicate that
   * either no interrupt has been generated, or the interrupt is masked."
   */

  mis = getreg32(regbase + LM3S_GPIO_MIS_OFFSET) & 0xff;

  /* Clear all GPIO interrupts that we are going to process.  "The GPIO ICR
   * register is the interrupt clear register. Writing a 1 to a bit in this
   * register clears the corresponding interrupt edge detection logic register.
   * Writing a 0 has no effect."
   */

  putreg32(mis, regbase + LM3S_GPIO_ICR_OFFSET);

  /* Now process each IRQ pending in the MIS */

  for (pin = 0; pin < 8 && mis != 0; pin++, mis >>= 1)
    {
      if ((mis & 1) != 0)
        {
          irq = irqbase + pin;
          g_gpioirqvector[irq - NR_IRQS](irq, context);
        }
    }
  return OK;
}

#ifndef CONFIG_LM3S_DISABLE_GPIOA_IRQS
static int lm3s_gpioahandler(int irq, FAR void *context)
{
  return lm3s_gpiohandler(LM3S_GPIOA_BASE, LM3S_IRQ_GPIOA_0, context);
}
#endif

#ifndef CONFIG_LM3S_DISABLE_GPIOB_IRQS
static int lm3s_gpiobhandler(int irq, FAR void *context)
{
  return lm3s_gpiohandler(LM3S_GPIOB_BASE, LM3S_IRQ_GPIOB_0, context);
}
#endif

#ifndef CONFIG_LM3S_DISABLE_GPIOC_IRQS
static int lm3s_gpiochandler(int irq, FAR void *context)
{
  return lm3s_gpiohandler(LM3S_GPIOC_BASE, LM3S_IRQ_GPIOC_0, context);
}
#endif

#ifndef CONFIG_LM3S_DISABLE_GPIOD_IRQS
static int lm3s_gpiodhandler(int irq, FAR void *context)
{
  return lm3s_gpiohandler(LM3S_GPIOD_BASE, LM3S_IRQ_GPIOD_0, context);
}
#endif

#ifndef CONFIG_LM3S_DISABLE_GPIOE_IRQS
static int lm3s_gpioehandler(int irq, FAR void *context)
{
  return lm3s_gpiohandler(LM3S_GPIOE_BASE, LM3S_IRQ_GPIOE_0, context);
}
#endif

#ifndef CONFIG_LM3S_DISABLE_GPIOF_IRQS
static int lm3s_gpiofhandler(int irq, FAR void *context)
{
  return lm3s_gpiohandler(LM3S_GPIOF_BASE, LM3S_IRQ_GPIOF_0, context);
}
#endif

#ifndef CONFIG_LM3S_DISABLE_GPIOG_IRQS
static int lm3s_gpioghandler(int irq, FAR void *context)
{
  return lm3s_gpiohandler(LM3S_GPIOG_BASE, LM3S_IRQ_GPIOG_0, context);
}
#endif

#ifndef CONFIG_LM3S_DISABLE_GPIOH_IRQS
static int lm3s_gpiohhandler(int irq, FAR void *context)
{
  return lm3s_gpiohandler(LM3S_GPIOH_BASE, LM3S_IRQ_GPIOH_0, context);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gpio_irqinitialize
 *
 * Description:
 *   Initialize all vectors to the unexpected interrupt handler
 *
 ****************************************************************************/

int gpio_irqinitialize(void)
{
  int i;

  /* Point all interrupt vectors to the unexpected interrupt */

  for (i = 0; i < NR_GPIO_IRQS; i++)
    {
      g_gpioirqvector[i] = irq_unexpected_isr;
    }

  /* Then attach all GPIO interrupt handlers */

#ifndef CONFIG_LM3S_DISABLE_GPIOA_IRQS
  irq_attach(LM3S_IRQ_GPIOA, lm3s_gpioahandler);
#endif
#ifndef CONFIG_LM3S_DISABLE_GPIOB_IRQS
  irq_attach(LM3S_IRQ_GPIOB, lm3s_gpiobhandler);
#endif
#ifndef CONFIG_LM3S_DISABLE_GPIOC_IRQS
  irq_attach(LM3S_IRQ_GPIOC, lm3s_gpiochandler);
#endif
#ifndef CONFIG_LM3S_DISABLE_GPIOD_IRQS
  irq_attach(LM3S_IRQ_GPIOD, lm3s_gpiodhandler);
#endif
#ifndef CONFIG_LM3S_DISABLE_GPIOE_IRQS
  irq_attach(LM3S_IRQ_GPIOE, lm3s_gpioehandler);
#endif
#ifndef CONFIG_LM3S_DISABLE_GPIOF_IRQS
  irq_attach(LM3S_IRQ_GPIOF, lm3s_gpiofhandler);
#endif
#ifndef CONFIG_LM3S_DISABLE_GPIOG_IRQS
  irq_attach(LM3S_IRQ_GPIOG, lm3s_gpioghandler);
#endif
#ifndef CONFIG_LM3S_DISABLE_GPIOH_IRQS
  irq_attach(LM3S_IRQ_GPIOH, lm3s_gpiohhandler);
#endif

  return OK;
}

/****************************************************************************
 * Name: gpio_irqattach
 *
 * Description:
 *   Attach in GPIO interrupt to the provide 'isr'
 *
 ****************************************************************************/

int gpio_irqattach(int irq, xcpt_t isr)
{
  irqstate_t flags;
  int        gpioirq = irq - NR_IRQS;
  int        ret     = ERROR;

  if ((unsigned)gpioirq < NR_GPIO_IRQS)
    {
      flags = irqsave();

      /* If the new ISR is NULL, then the ISR is being detached.
       * In this case, disable the ISR and direct any interrupts
       * to the unexpected interrupt handler.
       */

      if (isr == NULL)
        {
#ifndef CONFIG_ARCH_NOINTC
           gpio_irqdisable(gpioirq);
#endif
           isr = irq_unexpected_isr;
        }

      /* Save the new ISR in the table. */

      g_irqvector[gpioirq] = isr;
      irqrestore(flags);
      ret = OK;
    }
  return ret;
}

/****************************************************************************
 * Name: gpio_irqenable
 *
 * Description:
 *   Enable the GPIO IRQ specified by 'irq'
 *
 ****************************************************************************/

void gpio_irqenable(int irq)
{
  irqstate_t flags;
  int        gpioirq = irq - NR_IRQS;
  uint32     base;
  uint32     regval;
  int        pin;
  int        ret     = ERROR;

  if ((unsigned)gpioirq < NR_GPIO_IRQS)
    {
      /* Get the base address of the GPIO module associated with this IRQ */

      base = lm3s_gpiobaseaddress(gpioirq);
      pin  = (1 << (gpioirq & 7));

      /* Disable the GPIO interrupt. "The GPIO IM register is the interrupt
       * mask register. Bits set to High in GPIO IM allow the corresponding
       * pins to trigger their individual interrupts and the combined GPIO INTR
       * line. Clearing a bit disables interrupt triggering on that pin. All
       * bits are cleared by a reset.
       */

      flags   = irqsave();
      regval  = getreg32(base + LM3S_GPIO_IM_OFFSET);
      regval |= pin;
      putreg32(regval, base + LM3S_GPIO_IM_OFFSET);
      irqrestore(flags);
      ret = OK;
    }
  return ret;
}

/****************************************************************************
 * Name: gpio_irqdisable
 *
 * Description:
 *   Disable the GPIO IRQ specified by 'irq'
 *
 ****************************************************************************/

void gpio_irqdisable(int irq)
{
  irqstate_t flags;
  int        gpioirq = irq - NR_IRQS;
  uint32     base;
  uint32     regval;
  int        pin;
  int        ret     = ERROR;

  if ((unsigned)gpioirq < NR_GPIO_IRQS)
    {
      /* Get the base address of the GPIO module associated with this IRQ */

      base = lm3s_gpiobaseaddress(gpioirq);
      pin  = (1 << (gpioirq & 7));

      /* Disable the GPIO interrupt. "The GPIO IM register is the interrupt
       * mask register. Bits set to High in GPIO IM allow the corresponding
       * pins to trigger their individual interrupts and the combined GPIO INTR
       * line. Clearing a bit disables interrupt triggering on that pin. All
       * bits are cleared by a reset.
       */

      flags   = irqsave();
      regval  = getreg32(base + LM3S_GPIO_IM_OFFSET);
      regval &= ~pin;
      putreg32(regval, base + LM3S_GPIO_IM_OFFSET);
      irqrestore(flags);
      ret = OK;
    }
  return ret;
}

