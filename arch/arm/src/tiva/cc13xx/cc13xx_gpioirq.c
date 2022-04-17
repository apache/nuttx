/****************************************************************************
 * arch/arm/src/tiva/cc13xx/cc13xx_gpioirq.c
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
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <debug.h>

#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "irq/irq.h"

#include "tiva_gpio.h"

#ifdef CONFIG_TIVA_GPIO_IRQS

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

static struct gpio_handler_s g_gpio_inthandler[TIVA_NIRQ_PINS];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cc13xx_gpio_interrupt
 *
 * Description:
 *   Handle interrupts on each enabled GPIO port
 *
 ****************************************************************************/

static int cc13xx_gpio_interrupt(int irq, void *context, void *arg)
{
  uint32_t evflags;
  uint32_t regval;
  unsigned int dio;
  int irq;

  /* Get pending events */

  evflags = getreg32(TIVA_GPIO_EVFLAGS);

  /* Clear pending events that will be processing here */
#warning Missing logic

  /* Now process each pending DIO edge event */

  for (dio = 0, irq = TIVA_IRQ_DIO_0;
       dio < TIVA_NDIO && evflags != 0;
       dio++, irq++)
    {
      uint32_t diomask = (1 << dio);

      /* Is an event pending on this DIO? */

      if ((evflags & diomask) != 0)
        {
          /* Call any handler registered for each pending DIO interrupt */

          struct gpio_handler_s *handler = &g_gpio_inthandler[dio];

          gpioinfo("dio=%d isr=%p arg=%p\n", dio, handler->isr,
                   handler->arg);
          handler->isr(irq, context, handler->arg);

          evflags &= ~diomask;
        }
    }

  return OK;
}

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
  unsigned int dio;

  /* Point all interrupt vectors to the unexpected interrupt */

  for (dio = 0; dio < TIVA_NDIO; dio++)
    {
      g_gpio_inthandler[dio].isr = irq_unexpected_isr;
      g_gpio_inthandler[dio].arg = NULL;
    }

  /* Then attach the GPIO interrupt handler and enable corresponding GPIO
   * interrupts at the NVIC.
   */

  irq_attach(TIVA_IRQ_AON_GPIO_EDGE, cc13xx_gpio_interrupt, NULL);
  up_enable_irq(TIVA_IRQ_AON_GPIO_EDGE);

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

int tiva_gpioirqattach(pinconfig_t pinconfig, xcpt_t isr, void *arg)
{
  struct gpio_handler_s *handler;
  unsigned int dio;
  irqstate_t flags;

  /* Assign per-pin interrupt handlers */

  dio = (pinconfig->gpio & GPIO_DIO_MASK) >> GPIO_DIO_SHIFT;
  DEBUGASSERT(dio < TIVA_NDIO);

  if (dio < TIVA_NDIO)
    {
      flags = enter_critical_section();

      /* If the new ISR is NULL, then the ISR is being detached.
       * In this case, disable the ISR and direct any interrupts
       * to the unexpected interrupt handler.
       */

      gpioinfo("Attach dio=%d isr=%p arg=%p\n", dio, isr, arg);

      handler = &g_gpio_inthandler[dio];
      if (isr == NULL)
        {
          tiva_gpioirqdisable(pinconfig);
          handler->isr = irq_unexpected_isr;
          handler->arg = NULL;
        }
      else
        {
          handler->isr = isr;
          handler->arg = arg;
          tiva_gpioirqenable(pinconfig);
        }

      leave_critical_section(flags);
    }

  return OK;
}

/****************************************************************************
 * Name: tiva_gpioirqenable
 *
 * Description:
 *   Enable the GPIO port IRQ
 *
 ****************************************************************************/

void tiva_gpioirqenable(pinconfig_t pinconfig)
{
  uintptr_t regaddr;
  unsigned int dio;

  /* Enable edge interrupt generation */

  dio = (pinconfig->gpio & GPIO_DIO_MASK) >> GPIO_DIO_SHIFT;
  DEBUGASSERT(dio < TIVA_NDIO);

  regaddr = TIVA_IOC_IOCFG_OFFSET(dio);
  modifyreg32(regaddr, 0, IOC_IOCFG_EDGE_IRQEN);
}

/****************************************************************************
 * Name: tiva_gpioirqdisable
 *
 * Description:
 *   Disable the GPIO port IRQ
 *
 ****************************************************************************/

void tiva_gpioirqdisable(pinconfig_t pinconfig)
{
  uintptr_t regaddr;
  unsigned int dio;

  /* Disable edge interrupt generation */

  dio = (pinconfig->gpio & GPIO_DIO_MASK) >> GPIO_DIO_SHIFT;
  DEBUGASSERT(dio < TIVA_NDIO);

  regaddr = TIVA_IOC_IOCFG_OFFSET(dio);
  modifyreg32(regaddr, IOC_IOCFG_EDGE_IRQEN, 0);
}

/****************************************************************************
 * Name: tiva_gpioirqclear
 *
 * Description:
 *   Clears the interrupt status of the input base
 *
 ****************************************************************************/

void tiva_gpioirqclear(pinconfig_t pinconfig)
{
  unsigned int dio;

  /* Clear pending edge events */

  dio = (pinconfig->gpio & GPIO_DIO_MASK) >> GPIO_DIO_SHIFT;
  DEBUGASSERT(dio < TIVA_NDIO);

  modifyreg32(TIVA_GPIO_EVFLAGS, (1 << dio), 0);
}

#endif /* CONFIG_TIVA_GPIO_IRQS */
