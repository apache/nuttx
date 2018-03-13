/****************************************************************************
 *  arch/arm/src/kl/kl_gpioirq.c
 *
 *   Copyright (C) 2014, 2017 Gregory Nutt. All rights reserved.
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

#include <arch/board/board.h>
#include <nuttx/config.h>

#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "up_arch.h"

#include "chip/kl_port.h"
#include "kl_gpio.h"

#ifdef CONFIG_KL_GPIOIRQ

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* The Kinetis port interrupt logic is very flexible and will program
 * interrupts on most all pin events.  In order to keep the memory usage to
 * a minimum, the NuttX port supports enabling interrupts on a per-port
 * basis.
 */

#if defined(CONFIG_KL_PORTAINTS) || defined(CONFIG_KL_PORTDINTS)
#  define HAVE_PORTINTS 1
#endif

#if defined(CONFIG_KL_PORTBINTS) || defined(CONFIG_KL_PORTCINTS) || \
    defined(CONFIG_KL_PORTEINTS)
#  error Kinetis KL25 only supports interrupt on PORTA or PORTD
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct g_portisrs_s
{
  xcpt_t handler;   /* Interrupt handler entry point */
  void  *arg;       /* The argument that accompanies the interrupt handler */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* Per pin port interrupt vectors.  NOTE:  Not all pins in each port
 * correspond to externally available GPIOs.  However, I believe that the
 * Kinetis will support interrupts even if the pin is not available as
 * a GPIO. Hence, we need to support all 32 pins for each port.  To keep the
 * memory usage at a minimum, the logic may be configure per port.
 */

#ifdef CONFIG_KL_PORTAINTS
static struct g_portisrs_s g_portaisrs[32];
#endif

#ifdef CONFIG_KL_PORTDINTS
static struct g_portisrs_s g_portdisrs[32];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kl_portinterrupt
 *
 * Description:
 *   Common port interrupt handling.
 *
 ****************************************************************************/

#ifdef HAVE_PORTINTS
static int kl_portinterrupt(int irq, FAR void *context,
                                uintptr_t addr, xcpt_t *isrtab)
{
  uint32_t isfr = getreg32(addr);
  int i;

  /* Examine each pin in the port */

  for (i = 0; i < 32 && isfr != 0; i++)
    {
      /* A bit set in the ISR means that an interrupt is pending for this
       * pin.  If the pin is programmed for level sensitive inputs, then
       * the interrupt handling logic MUST disable the interrupt (or cause
       * the level to change) to prevent infinite interrupts.
       */

      uint32_t bit = (1 << i);
      if ((isfr & bit) != 0)
        {
          /* I think that bits may be set in the ISFR for DMA activities
           * well.  So, no error is declared if there is no registered
           * interrupt handler for the pin.
           */

          if (isrtab[i].handler != NULL)
            {
              xcpt_t handler = irstab[i].handler;
              void  *arg     = irstab[i].arg;

              /* There is a registered interrupt handler... invoke it */

              (void)handler(irq, context, arg);
            }

          /* Writing a one to the ISFR register will clear the pending
           * interrupt.  If pin is configured to generate a DMA request
           * then the ISFR bit will be cleared automatically at the
           * completion of the requested DMA transfer. If configured for
           * a level sensitive interrupt and the pin remains asserted and
           * the bit will set again immediately after it is cleared.
           */

          isfr &= ~bit;
          putreg32(bit, addr);
        }
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: kl_portXinterrupt
 *
 * Description:
 *   Handle interrupts arriving on individual ports
 *
 ****************************************************************************/

#ifdef CONFIG_KL_PORTAINTS
static int kl_portainterrupt(int irq, FAR void *context, FAR void *arg)
{
  return kl_portinterrupt(irq, context, KL_PORTA_ISFR, g_portaisrs);
}
#endif

#ifdef CONFIG_KL_PORTDINTS
static int kl_portdinterrupt(int irq, FAR void *context, FAR void *arg)
{
  return kl_portinterrupt(irq, context, KL_PORTD_ISFR, g_portdisrs);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kl_gpioirqinitialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for
 *   GPIO pins.
 *
 ****************************************************************************/

void kl_gpioirqinitialize(void)
{
#ifdef CONFIG_KL_PORTAINTS
  (void)irq_attach(KL_IRQ_PORTA, kl_portainterrupt, NULL);
  putreg32(0xffffffff, KL_PORTA_ISFR);
  up_enable_irq(KL_IRQ_PORTA);
#endif

#ifdef CONFIG_KL_PORTDINTS
  (void)irq_attach(KL_IRQ_PORTD, kl_portdinterrupt, NULL);
  putreg32(0xffffffff, KL_PORTD_ISFR);
  up_enable_irq(KL_IRQ_PORTD);
#endif
}

/****************************************************************************
 * Name: kl_gpioirqattach
 *
 * Description:
 *   Attach a pin interrupt handler.  The normal initialization sequence is:
 *
 *   1. Call kl_gpioconfig() to configure the interrupting pin (pin interrupts
 *      will be disabled.
 *   2. Call kl_gpioirqattach() to attach the pin interrupt handling function.
 *   3. Call kl_gpioirqenable() to enable interrupts on the pin.
 *
 * Input Parameters:
 *  - pinset:  Pin configuration
 *  - pinisr:  Pin interrupt service routine
 *  - pinarg:  The argument that will accompany the pin interrupt
 *
 * Returned Value:
 * Returned Value:
 *   Zero (OK) is returned on success; On any failure, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ************************************************************************************/

int kl_gpioirqattach(uint32_t pinset, xcpt_t pinisr, void *pinarg)
{
#ifdef HAVE_PORTINTS
  struct g_portisrs_s *isrtab;
  irqstate_t flags;
  unsigned int port;
  unsigned int pin;

  /* It only makes sense to call this function for input pins that are
   * configured as interrupts.
   */

  DEBUGASSERT((pinset & _PIN_INTDMA_MASK) == _PIN_INTERRUPT);
  DEBUGASSERT((pinset & _PIN_IO_MASK) == _PIN_INPUT);

  /* Get the port number and pin number */

  port = (pinset & _PIN_PORT_MASK) >> _PIN_PORT_SHIFT;
  pin  = (pinset & _PIN_MASK)      >> _PIN_SHIFT;

  /* Get the table associated with this port */

  DEBUGASSERT(port < KL_NPORTS);
  flags = enter_critical_section();
  switch (port)
    {
#ifdef CONFIG_KL_PORTAINTS
      case KL_PORTA :
        isrtab = g_portaisrs;
        break;
#endif
#ifdef CONFIG_KL_PORTDINTS
      case KL_PORTD :
        isrtab = g_portdisrs;
        break;
#endif
      default:
        leave_critical_section(flags);
        return NULL;
    }

   /* Get the old PIN ISR and set the new PIN ISR */

   isrtab[pin].handler = pinisr;
   isrtab[pin].arg     = pinarg;

   /* And return the old PIN isr address */

   leave_critical_section(flags);
   return OK;

#else
   return -ENOSYS;
#endif /* HAVE_PORTINTS */
}

/************************************************************************************
 * Name: kl_gpioirqenable
 *
 * Description:
 *   Enable the interrupt for specified pin IRQ
 *
 ************************************************************************************/

void kl_gpioirqenable(uint32_t pinset)
{
#ifdef HAVE_PORTINTS
  uintptr_t    base;
  uint32_t     regval;
  unsigned int port;
  unsigned int pin;

  /* Get the port number and pin number */

  port = (pinset & _PIN_PORT_MASK) >> _PIN_PORT_SHIFT;
  pin  = (pinset & _PIN_MASK)      >> _PIN_SHIFT;

  DEBUGASSERT(port < KL_NPORTS);
  if (port < KL_NPORTS)
    {
      /* Get the base address of PORT block for this port */

      base =  KL_PORT_BASE(port);

      /* Modify the IRQC field of the port PCR register in order to enable
       * the interrupt.
       */

      regval = getreg32(base + KL_PORT_PCR_OFFSET(pin));
      regval &= ~PORT_PCR_IRQC_MASK;

      switch (pinset & _PIN_INT_MASK)
        {
          case PIN_INT_ZERO : /* Interrupt when logic zero */
            regval |= PORT_PCR_IRQC_ZERO;
            break;

          case PIN_INT_RISING : /* Interrupt on rising edge */
            regval |= PORT_PCR_IRQC_RISING;
            break;

          case PIN_INT_FALLING : /* Interrupt on falling edge */
            regval |= PORT_PCR_IRQC_FALLING;
            break;

          case PIN_INT_BOTH : /* Interrupt on either edge */
            regval |= PORT_PCR_IRQC_BOTH;
            break;

          case PIN_DMA_RISING : /* Interrupt on DMA rising */
            regval |= PORT_PCR_IRQC_DMARISING;
            break;

          case PIN_DMA_FALLING : /* Interrupt on DMA falling */
            regval |= PORT_PCR_IRQC_DMAFALLING;
            break;

          case PIN_INT_ONE : /* IInterrupt when logic one */
            regval |= PORT_PCR_IRQC_ONE;
            break;

          default:
            return;
        }

      putreg32(regval, base + KL_PORT_PCR_OFFSET(pin));
    }
#endif /* HAVE_PORTINTS */
}

/************************************************************************************
 * Name: kl_gpioirqdisable
 *
 * Description:
 *   Disable the interrupt for specified pin
 *
 ************************************************************************************/

void kl_gpioirqdisable(uint32_t pinset)
{
#ifdef HAVE_PORTINTS
  uintptr_t    base;
  uint32_t     regval;
  unsigned int port;
  unsigned int pin;

  /* Get the port number and pin number */

  port = (pinset & _PIN_PORT_MASK) >> _PIN_PORT_SHIFT;
  pin  = (pinset & _PIN_MASK)      >> _PIN_SHIFT;

  DEBUGASSERT(port < KL_NPORTS);
  if (port < KL_NPORTS)
    {
      /* Get the base address of PORT block for this port */

      base =  KL_PORT_BASE(port);

      /* Clear the IRQC field of the port PCR register in order to disable
       * the interrupt.
       */

      regval = getreg32(base + KL_PORT_PCR_OFFSET(pin));
      regval &= ~PORT_PCR_IRQC_MASK;
      putreg32(regval, base + KL_PORT_PCR_OFFSET(pin));
    }
#endif /* HAVE_PORTINTS */
}
#endif /* CONFIG_KL_GPIOIRQ */
