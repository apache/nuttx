/****************************************************************************
 * arch/mips/src/pic32mx/pic32mx_gpioirq.c
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
#include <assert.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "mips_internal.h"
#include "pic32mx_gpio.h"
#include "pic32mx.h"

#ifdef CONFIG_PIC32MX_GPIOIRQ

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct g_cnisrs_s
{
  xcpt_t handler;   /* Interrupt handler entry point */
  void  *arg;       /* The argument that accompanies the interrupt handler */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct g_cnisrs_s g_cnisrs[IOPORT_NUMCN];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: Inline PIN set field extractors
 ****************************************************************************/

static inline bool pic32mx_input(uint16_t pinset)
{
  return ((pinset & GPIO_MODE_MASK) != GPIO_INPUT);
}

static inline bool pic32mx_interrupt(uint16_t pinset)
{
  return ((pinset & GPIO_INTERRUPT) != 0);
}

static inline bool pic32mx_pullup(uint16_t pinset)
{
  return ((pinset & GPIO_INT_MASK) == GPIO_PUINT);
}

/****************************************************************************
 * Name: pic32mx_cninterrupt
 *
 * Description:
 *  Change notification interrupt handler.
 *
 ****************************************************************************/

static int pic32mx_cninterrupt(int irq, void *context)
{
  int status;
  int ret = OK;
  int i;

  /* Call all attached handlers */

  for (i = 0; i < IOPORT_NUMCN; i++)
    {
      /* Is this one attached */

      if (g_cnisrs[i].handler != NULL)
        {
          xcpt_t handler = irstab[i].handler;
          void  *arg     = irstab[i].arg;

          /* Call the attached handler */

          status = handler(irq, context, arg);

          /* Keep track of the status of the last handler that failed */

          if (status < 0)
            {
              ret = status;
            }
        }
    }

  /* Clear the pending interrupt */

  up_clrpend_irq(PIC32MX_IRQ_CN);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mx_gpioirqinitialize
 *
 * Description:
 *   Initialize logic to support a GPIO change notification interrupts.
 *   This function is called internally by the system on power up and should
 *   not be called again.
 *
 ****************************************************************************/

void pic32mx_gpioirqinitialize(void)
{
  int ret;

  /* Attach the change notice interrupt handler */

  ret = irqattach(PIC32MX_IRQ_CN, pic32mx_cninterrupt);
  DEBUGASSERT(ret == OK);

  /* Set the interrupt priority */

#ifdef CONFIG_ARCH_IRQPRIO
  ret = up_prioritize_irq(PIC32MX_IRQ_CN, CONFIG_PIC32MX_CNPRIO);
  DEBUGASSERT(ret == OK);
#endif

  /* Reset all registers and enable the CN module */

  putreg32(IOPORT_CN_ALL, PIC32MX_IOPORT_CNENCLR);
  putreg32(IOPORT_CN_ALL, PIC32MX_IOPORT_CNPUECLR);
  putreg32(IOPORT_CNCON_ON, PIC32MX_IOPORT_CNCON);

  /* And enable the GPIO interrupt */

  ret = up_enable_irq(PIC32MX_IRQSRC_CN);
  DEBUGASSERT(ret == OK);
}

/****************************************************************************
 * Name: pic32mx_gpioattach
 *
 * Description:
 *   Attach an interrupt service routine to a GPIO interrupt.  This will
 *   also reconfigure the pin as an interrupting input.  The change
 *   notification number is associated with all interrupt-capabile GPIO pins.
 *   The association could, however, differ from part to part and must be
 *   provided by the caller.
 *
 *   When an interrupt occurs, it is due to a change on the GPIO input pin.
 *   In that case, all attached handlers will be called.  Each handler must
 *   maintain state and determine if the unlying GPIO input value changed.
 *
 * Input Parameters:
 *   pinset  - GPIO pin configuration
 *   cn      - The change notification number associated with the pin.
 *   handler - Interrupt handler (may be NULL to detach)
 *   arg     - The argument that accompanies the interrupt
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated error value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

int pic32mx_gpioattach(uint32_t pinset, unsigned int cn, xcpt_t handler,
                       void *arg)
{
  irqstate_t flags;

  DEBUGASSERT(cn < IOPORT_NUMCN);

  /* First verify that the pinset is configured as an interrupting input */

  if (pic32mx_input(pinset) && pic32mx_interrupt(pinset))
    {
      /* Get the previously attached handler as the return value */

      flags = enter_critical_section();

      /* Are we attaching or detaching? */

      if (handler != NULL)
        {
          /* Attaching... Make sure that the GPIO is properly configured as
           * an input
           */

          pic32mx_configgpio(pinset);

          /* Pull-up requested? */

          if (pic32mx_pullup(pinset))
            {
              putreg32(1 << cn, PIC32MX_IOPORT_CNPUESET);
            }
          else
            {
              putreg32(1 << cn, PIC32MX_IOPORT_CNPUECLR);
            }
        }
      else
        {
          /* Make sure that any further interrupts are disabled.
           * (disable the pull-up as well).
           */

          putreg32(1 << cn, PIC32MX_IOPORT_CNENCLR);
          putreg32(1 << cn, PIC32MX_IOPORT_CNPUECLR);
        }

      /* Set the new handler (perhaps NULLifying the current handler) */

      g_cnisrs[cn].handler = handler;
      g_cnisrs[cn].arg     = arg;
      leave_critical_section(flags);
    }

  return OK;
}

/****************************************************************************
 * Name: pic32mx_gpioirqenable
 *
 * Description:
 *   Enable the interrupt for specified GPIO IRQ
 *
 ****************************************************************************/

void pic32mx_gpioirqenable(unsigned int cn)
{
  DEBUGASSERT(cn < IOPORT_NUMCN);
  putreg32(1 << cn, PIC32MX_IOPORT_CNENSET);
}

/****************************************************************************
 * Name: pic32mx_gpioirqdisable
 *
 * Description:
 *   Disable the interrupt for specified GPIO IRQ
 *
 ****************************************************************************/

void pic32mx_gpioirqdisable(unsigned int cn)
{
  DEBUGASSERT(cn < IOPORT_NUMCN);
  putreg32(1 << cn, PIC32MX_IOPORT_CNENCLR);
}

#endif /* CONFIG_PIC32MX_GPIOIRQ */
