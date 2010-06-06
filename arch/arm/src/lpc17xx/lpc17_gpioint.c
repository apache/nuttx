/****************************************************************************
 * arch/arm/src/lpc17xx/lpc17_gpioint.c
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <arch/irq.h>

#include "up_arch.h"
#include "chip.h"
#include "lpc17_gpio.h"
#include "lpc17_pinconn.h"
#include "lpc17_internal.h"

#ifdef CONFIG_GPIO_IRQ

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* We have to remember the configured interrupt setting.. PINs are not
 * actually set up to interrupt until the interrupt is enabled.
 */

#ifdef CONFIG_GPIO_IRQ
uint64_t g_intedge0;
atic uint64_t g_intedge2;
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/
/* These tables have global scope because they are also used in
 * lpc17_gpiodbg.c
 */

/* We have to remember the configured interrupt setting.. PINs are not
 * actually set up to interrupt until the interrupt is enabled.
 */

#ifdef CONFIG_GPIO_IRQ
uint64_t g_intedge0;
uint64_t g_intedge2;
#endif

/* FIO register base addresses */

const uint32_t g_fiobase[GPIO_NPORTS] =
{
  LPC17_FIO0_BASE,
  LPC17_FIO1_BASE,
  LPC17_FIO2_BASE,
  LPC17_FIO3_BASE,
  LPC17_FIO4_BASE
};

/* Port 0 and Port 2 can provide a single interrupt for any combination of
 * port pins 
 */

const uint32_t g_intbase[GPIO_NPORTS] = 
{
  LPC17_GPIOINT0_OFFSET,
  0,
  LPC17_GPIOINT2_OFFSET,
  0,
  0
};

const uint32_t g_lopinsel[GPIO_NPORTS] = 
{
  LPC17_PINCONN_PINSEL0,
  LPC17_PINCONN_PINSEL2,
  LPC17_PINCONN_PINSEL4,
  0,
  0
};

const uint32_t g_hipinsel[GPIO_NPORTS] = 
{
  LPC17_PINCONN_PINSEL1,
  LPC17_PINCONN_PINSEL3,
  0,
  LPC17_PINCONN_PINSEL7,
  LPC17_PINCONN_PINSEL9
};

const uint32_t g_lopinmode[GPIO_NPORTS] =
{
  LPC17_PINCONN_PINMODE0,
  LPC17_PINCONN_PINMODE2,
  LPC17_PINCONN_PINMODE4,
  0,
  0
};

const uint32_t g_hipinmode[GPIO_NPORTS] =
{
  LPC17_PINCONN_PINMODE1,
  LPC17_PINCONN_PINMODE3,
  0,
  LPC17_PINCONN_PINMODE7,
  LPC17_PINCONN_PINMODE9
};

const uint32_t g_odmode[GPIO_NPORTS] =
{
  LPC17_PINCONN_ODMODE0,
  LPC17_PINCONN_ODMODE1,
  LPC17_PINCONN_ODMODE2,
  LPC17_PINCONN_ODMODE3,
  LPC17_PINCONN_ODMODE4
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc17_getintedge
 *
 * Description:
 *  Get the stored interrupt edge configuration.
 *
 ****************************************************************************/

static unsigned int lpc17_getintedge(unsigned int port, unsigned int pin)
{
  const uint64_t *intedge;

  /* Which word to we use? */

  if (port == 0)
    {
      intedge = g_intedge0;
    }
  else if (port == 2)
    {
      intedge  = g_intedge2;
    }
  else
    {
      return 0;
    }

  /* Return the value for the PINSEL */

  return (unsigned int)((*intedge & (3 << (pin << 1))) >> shift);
}

/****************************************************************************
 * Name: lpc17_setintedge
 *
 * Description:
 *  Set the edge interrupt enabled bits for this pin.
 *
 ****************************************************************************/

static void lpc17_setintedge(uint32_t intbase, unsigned int pin, unsigned int edges)
{
  int regval;

  /* Set/clear the rising edge enable bit */

  regval = getreg32(intbase + LPC17_GPIOINT_INTENR_OFFSET);
  if ((edges & 2) != 0)
    {
      regval |= GPIOINT(pin);
    }
  else
    {
      regval &= ~GPIOINT(pin);
    }
  endif
  putreg32(regval, intbase + LPC17_GPIOINT_INTENR_OFFSET);

  /* Set/clear the rising edge enable bit */

  regval = getreg32(intbase + LPC17_GPIOINT_INTENF_OFFSET);
  if ((edges & 1) != 0)
    {
      regval |= GPIOINT(pin);
    }
  else
    {
      regval &= ~GPIOINT(pin);
    }
  endif
  putreg32(regval, intbase + LPC17_GPIOINT_INTENF_OFFSET);
}

/****************************************************************************
 * Name: lpc17_irq2port
 *
 * Description:
 *  Given an IRQ number, return the GPIO port number (0 or 2) of the interrupt.
 *
 ****************************************************************************/

static int lpc17_irq2port(int irq)
{
 /* Set 1: 12 interrupts p0.0-p0.11 */

  if (irq >= LPC17_VALID_FIRST0L && irq < (LPC17_VALID_FIRST0L+LPC17_VALID_NIRQS0L)
    {
      return 0;
    }

  /* Set 2: 16 interrupts p0.15-p0.30 */

  else if (irq >= LPC17_VALID_FIRST0H && irq < (LPC17_VALID_FIRST0H+LPC17_VALID_NIRQS0H)
    {
      return 0;
    }

  /* Set 3: 14 interrupts p2.0-p2.13 */

  else if (irq >= LPC17_VALID_NIRQS2 && irq < (LPC17_VALID_FIRST2+LPC17_VALID_NIRQS2)
    {
      return 2;
    }
  return -EINVAL;
}

/****************************************************************************
 * Name: lpc17_irq2pin
 *
 * Description:
 *  Given an IRQ number, return the GPIO pin number (0..31) of the interrupt.
 *
 ****************************************************************************/

static int lpc17_irq2port(int irq)
{
 /* Set 1: 12 interrupts p0.0-p0.11 */

  if (irq >= LPC17_VALID_FIRST0L && irq < (LPC17_VALID_FIRST0L+LPC17_VALID_NIRQS0L)
    {
      return irq - LPC17_VALID_FIRST0L + LPC17_VALID_SHIFT0L;
    }

  /* Set 2: 16 interrupts p0.15-p0.30 */

  else if (irq >= LPC17_VALID_FIRST0H && irq < (LPC17_VALID_FIRST0H+LPC17_VALID_NIRQS0H)
    {
      return irq - LPC17_VALID_FIRST0H + LPC17_VALID_SHIFT0H;
      
      12
    }

  /* Set 3: 14 interrupts p2.0-p2.13 */

  else if (irq >= LPC17_VALID_NIRQS2 && irq < (LPC17_VALID_FIRST2+LPC17_VALID_NIRQS2)
    {
      return irq - LPC17_VALID_FIRST0H + LPC17_VALID_SHIFT2;
    }
  return -EINVAL;
}


/****************************************************************************
 * Global Functions
 ****************************************************************************/

/************************************************************************************
 * Name: lpc17_gpioirqenable
 *
 * Description:
 *   Enable the interrupt for specified GPIO IRQ
 *
 ************************************************************************************/

void lpc17_gpioirqenable(int irq)
{
  /* Map the IRQ number to a port number */

  int port = lpc17_irq2port(irq);
  if (port >= 0)
    {
      /* The IRQ number does correspond to an interrupt port.  Now get the base
       * address of the GPIOINT registers for the port.
       */

      uint32_t intbase = g_intbase[GPIO_NPORTS];
      if (intabase != 0)
        {
          /* And get the pin number associated with the port */

          unsigned int pin   = g_irq2pin(irq);
          unsigned int edges = lpc17_getintedge(port, pin);
          lpc17_setintedge(intbase, pin, edges);
        }
    }
}

/************************************************************************************
 * Name: lpc17_gpioirqdisable
 *
 * Description:
 *   Disable the interrupt for specified GPIO IRQ
 *
 ************************************************************************************/

void lpc17_gpioirqdisable(int irq)
{
  /* Map the IRQ number to a port number */

  int port = lpc17_irq2port(irq);
  if (port >= 0)
    {
      /* The IRQ number does correspond to an interrupt port.  Now get the base
       * address of the GPIOINT registers for the port.
       */

      uint32_t intbase = g_intbase[GPIO_NPORTS];
      if (intabase != 0)
        {
          /* And get the pin number associated with the port */

          unsigned int pin   = g_irq2pin(irq);
          lpc17_setintedge(intbase, pin, 0);
        }
    }
}

#warning "Still needs initialization, interrupt handling and decoding logic"

#endif /* CONFIG_GPIO_IRQ */

