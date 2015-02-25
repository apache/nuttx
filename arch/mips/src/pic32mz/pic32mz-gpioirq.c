/****************************************************************************
 * arch/mips/src/pic32mz/pic32mz-gpio.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <assert.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "chip/pic32mz-ioport.h"
#include "pic32mz-gpio.h"

#ifdef CONFIG_PIC32MZ_GPIOIRQ

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static xcpt_t g_cnisrs[IOPORT_NUMCN];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: Inline PIN set field extractors
 ****************************************************************************/

static inline bool pic32mz_input(uint16_t pinset)
{
  return ((pinset & GPIO_MODE_MASK) != GPIO_INPUT);
}

static inline bool pic32mz_interrupt(uint16_t pinset)
{
  return ((pinset & GPIO_INTERRUPT) != 0);
}

static inline bool pic32mz_pullup(uint16_t pinset)
{
  return ((pinset & GPIO_INT_MASK) == GPIO_PUINT);
}

/****************************************************************************
 * Name: pic32mz_cninterrupt
 *
 * Description:
 *  Change notification interrupt handler.
 *
 ****************************************************************************/

static int pic32mz_cninterrupt(int irq, FAR void *context)
{
  int status;
  int ret = OK;
  int i;

  /* Call all attached handlers */

  for (i = 0; i < IOPORT_NUMCN; i++)
    {
      /* Is this one attached */

      if (g_cnisrs[i])
        {
          /* Call the attached handler */

          status = g_cnisrs[i](irq, context);

          /* Keep track of the status of the last handler that failed */

          if (status < 0)
            {
              ret = status;
            }
    }

  /* Clear the pending interrupt */

  up_clrpend_irq(PIC32MZ_IRQ_CN);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mz_gpioirqinitialize
 *
 * Description:
 *   Initialize logic to support a GPIO change notification interrupts.
 *   This function is called internally by the system on power up and should
 *   not be called again.
 *
 ****************************************************************************/

void pic32mz_gpioirqinitialize(void)
{
  int ret;

  /* Attach the change notice interrupt handler */

  ret = irqattach(PIC32MZ_IRQ_CN, pic32mz_cninterrupt);
  DEBUGASSERT(ret == OK);

  /* Set the interrupt priority */

#ifdef CONFIG_ARCH_IRQPRIO
  ret = up_prioritize_irq(PIC32MZ_IRQ_CN, CONFIG_PIC32MZ_CNPRIO);
  DEBUGASSERT(ret == OK);
#endif

  /* Reset all registers and enable the CN module */

  putreg32(IOPORT_CN_ALL, PIC32MZ_IOPORT_CNENCLR);
  putreg32(IOPORT_CN_ALL, PIC32MZ_IOPORT_CNPUECLR);
  putreg32(IOPORT_CNCON_ON, PIC32MZ_IOPORT_CNCON);

  /* And enable the GPIO interrupt */

  ret = up_enable_irq(PIC32MZ_IRQSRC_CN);
  DEBUGASSERT(ret == OK);
}

/****************************************************************************
 * Name: pic32mz_gpioattach
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
 * Parameters:
 *  - pinset:  GPIO pin configuration
 *  - cn:      The change notification number associated with the pin.
 *  - handler: Interrupt handler (may be NULL to detach)
 *
 * Returns:
 *  The previous value of the interrupt handler function pointer.  This
 *  value may, for example, be used to restore the previous handler when
 *  multiple handlers are used.
 *
 ****************************************************************************/

xcpt_t pic32mz_gpioattach(uint32_t pinset, unsigned int cn, xcpt_t handler)
{
  xcpt_t oldhandler = NULL;
  irqstate_t flags;

  DEBUGASSERT(cn < IOPORT_NUMCN);

  /* First verify that the pinset is configured as an interrupting input */

  if (pic32mz_input(pinset) && pic32mz_interrupt(pinset))
    {
      /* Get the previously attached handler as the return value */

      flags = irqsave();
      oldhandler = g_cnisrs[cn];

      /* Are we attaching or detaching? */

      if (handler != NULL)
        {
          /* Attaching... Make sure that the GPIO is properly configured as
           * an input
           */

          pic32mz_configgpio(pinset);

          /* Pull-up requested? */

          if (pic32mz_pullup(pinset))
            {
              putreg32(1 << cn, PIC32MZ_IOPORT_CNPUESET);
            }
          else
            {
              putreg32(1 << cn, PIC32MZ_IOPORT_CNPUECLR);
            }
        }
      else
        {
           /* Make sure that any further interrupts are disabled.
            * (disable the pull-up as well).
            */

           putreg32(1 << cn, PIC32MZ_IOPORT_CNENCLR);
           putreg32(1 << cn, PIC32MZ_IOPORT_CNPUECLR);
        }

      /* Set the new handler (perhaps NULLifying the current handler) */

      g_cnisrs[cn] = handler;
      irqrestore(flags);
    }

  return oldhandler;
}

/****************************************************************************
 * Name: pic32mz_gpioirqenable
 *
 * Description:
 *   Enable the interrupt for specified GPIO IRQ
 *
 ****************************************************************************/

void pic32mz_gpioirqenable(unsigned int cn)
{
  DEBUGASSERT(cn < IOPORT_NUMCN);
  putreg32(1 << cn, PIC32MZ_IOPORT_CNENSET);
}

/****************************************************************************
 * Name: pic32mz_gpioirqdisable
 *
 * Description:
 *   Disable the interrupt for specified GPIO IRQ
 *
 ****************************************************************************/

void pic32mz_gpioirqdisable(unsigned int cn)
{
  DEBUGASSERT(cn < IOPORT_NUMCN);
  putreg32(1 << cn, PIC32MZ_IOPORT_CNENCLR);
}

#endif /* CONFIG_PIC32MZ_GPIOIRQ */
