/****************************************************************************
 * arch/mips/src/pic32mz/pic32mz_gpioirq.c
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
#include <arch/pic32mz/irq.h>

#include "mips_internal.h"
#include "hardware/pic32mz_ioport.h"
#include "pic32mz_gpio.h"

#ifdef CONFIG_PIC32MZ_GPIOIRQ

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline unsigned int pic32mz_ioport(pinset_t pinset);
static inline unsigned int pic32mz_pin(pinset_t pinset);
static inline bool pic32mz_input(pinset_t pinset);
static inline bool pic32mz_interrupt(pinset_t pinset);
static inline bool pic32mz_edgedetect(pinset_t pinset);
static inline unsigned int pic32mz_edgemode(pinset_t pinset);
static inline bool pic32mz_pullup(pinset_t pinset);
static inline bool pic32mz_pulldown(pinset_t pinset);
static int pic32mz_cninterrupt(int irq, void *context, void *arg);

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct ioport_handler_s
{
  xcpt_t entry;     /* Interrupt handler entry point */
  void  *arg;       /* The argument that accompanies the interrupt handler */
};

struct ioport_level2_s
{
  struct ioport_handler_s handler[16];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Arrays of second level interrupt handlers for each pin of each enabled
 * I/O port.
 */

#ifdef CONFIG_PIC32MZ_GPIOIRQ_PORTA
static struct ioport_level2_s g_ioporta_cnisrs;
#endif
#ifdef CONFIG_PIC32MZ_GPIOIRQ_PORTB
static struct ioport_level2_s g_ioportb_cnisrs;
#endif
#ifdef CONFIG_PIC32MZ_GPIOIRQ_PORTC
static struct ioport_level2_s g_ioportc_cnisrs;
#endif
#ifdef CONFIG_PIC32MZ_GPIOIRQ_PORTD
static struct ioport_level2_s g_ioportd_cnisrs;
#endif
#ifdef CONFIG_PIC32MZ_GPIOIRQ_PORTE
static struct ioport_level2_s g_ioporte_cnisrs;
#endif
#ifdef CONFIG_PIC32MZ_GPIOIRQ_PORTF
static struct ioport_level2_s g_ioportf_cnisrs;
#endif
#ifdef CONFIG_PIC32MZ_GPIOIRQ_PORTG
static struct ioport_level2_s g_ioportg_cnisrs;
#endif
#ifdef CONFIG_PIC32MZ_GPIOIRQ_PORTH
static struct ioport_level2_s g_ioporth_cnisrs;
#endif
#ifdef CONFIG_PIC32MZ_GPIOIRQ_PORTJ
static struct ioport_level2_s g_ioportj_cnisrs;
#endif
#ifdef CONFIG_PIC32MZ_GPIOIRQ_PORTK
static struct ioport_level2_s g_ioportk_cnisrs;
#endif

/* Look-up of port to interrupt handler array */

static struct ioport_level2_s * const g_level2_handlers[CHIP_NPORTS] =
{
#ifdef CONFIG_PIC32MZ_GPIOIRQ_PORTA
  [PIC32MZ_IOPORTA] = &g_ioporta_cnisrs,
#endif
#ifdef CONFIG_PIC32MZ_GPIOIRQ_PORTB
  [PIC32MZ_IOPORTB] = &g_ioportb_cnisrs,
#endif
#ifdef CONFIG_PIC32MZ_GPIOIRQ_PORTC
  [PIC32MZ_IOPORTC] = &g_ioportc_cnisrs,
#endif
#ifdef CONFIG_PIC32MZ_GPIOIRQ_PORTD
  [PIC32MZ_IOPORTD] = &g_ioportd_cnisrs,
#endif
#ifdef CONFIG_PIC32MZ_GPIOIRQ_PORTE
  [PIC32MZ_IOPORTE] = &g_ioporte_cnisrs,
#endif
#ifdef CONFIG_PIC32MZ_GPIOIRQ_PORTF
  [PIC32MZ_IOPORTF] = &g_ioportf_cnisrs,
#endif
#ifdef CONFIG_PIC32MZ_GPIOIRQ_PORTG
  [PIC32MZ_IOPORTG] = &g_ioportg_cnisrs,
#endif
#ifdef CONFIG_PIC32MZ_GPIOIRQ_PORTH
  [PIC32MZ_IOPORTH] = &g_ioporth_cnisrs,
#endif
#ifdef CONFIG_PIC32MZ_GPIOIRQ_PORTJ
  [PIC32MZ_IOPORTJ] = &g_ioportj_cnisrs,
#endif
#ifdef CONFIG_PIC32MZ_GPIOIRQ_PORTK
  [PIC32MZ_IOPORTK] = &g_ioportk_cnisrs,
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: Inline PIN set field extractors
 ****************************************************************************/

static inline bool pic32mz_input(pinset_t pinset)
{
  return (((pinset & GPIO_MODE_MASK) >> GPIO_MODE_SHIFT) == GPIO_INPUT);
}

static inline bool pic32mz_interrupt(pinset_t pinset)
{
  return ((pinset & GPIO_INTERRUPT) != 0);
}

static inline bool pic32mz_edgedetect(pinset_t pinset)
{
  return ((pinset & GPIO_EDGE_DETECT) != 0);
}

static inline unsigned int pic32mz_edgemode(pinset_t pinset)
{
  return ((pinset & GPIO_EDGE_RISING));
}

static inline bool pic32mz_pullup(pinset_t pinset)
{
  return ((pinset & GPIO_PULLUP) != 0);
}

static inline bool pic32mz_pulldown(pinset_t pinset)
{
  return ((pinset & GPIO_PULLDOWN) != 0);
}

static inline unsigned int pic32mz_ioport(pinset_t pinset)
{
  return ((pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT);
}

static inline unsigned int pic32mz_pin(pinset_t pinset)
{
  return ((pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT);
}

/****************************************************************************
 * Name: pic32mz_cninterrupt
 *
 * Description:
 *  Common change notification interrupt handler.
 *
 ****************************************************************************/

static int pic32mz_cninterrupt(int irq, void *context, void *arg)
{
  struct ioport_level2_s *handlers;
  xcpt_t handler;
  uintptr_t base;
  uint32_t cncon;
  uint32_t cnen;
  uint32_t cnne;
  uint32_t cnstat;
  uint32_t cnf;
  uint32_t pending;
  uint32_t regval;
  int ioport;
  int status;
  int ret = OK;
  int i;

  /* Get the IO port index from the IRQ number.  This, of course,
   * assumes that the irq numbers are consecutive beginning with
   * IOPORTA.
   */

  ioport   = irq - PIC32MZ_IRQ_PORTA;
  DEBUGASSERT(ioport >= 0 && ioport < CHIP_NPORTS);

  /* If we got this interrupt, then there must also be an array
   * of second level handlers (and a base address) for the IOPORT.
   */

  handlers = g_level2_handlers[ioport];
  base     = g_gpiobase[ioport];
  DEBUGASSERT(handlers && base);

  /* Get the control registers. It will be used to get the interrupt type. */

  cncon = getreg32(base + PIC32MZ_IOPORT_CNCON_OFFSET);

  if (handlers && base)
    {
      /* Mismatch mode selected? */

      if ((cncon & IOPORT_CNCON_EDGEDETECT) == 0)
        {
          /* Get the interrupt status associated with this interrupt */

          cnstat  = getreg32(base + PIC32MZ_IOPORT_CNSTAT_OFFSET);
          cnen    = getreg32(base + PIC32MZ_IOPORT_CNEN_OFFSET);
          pending = cnstat & cnen;

          /* Call all attached handlers for each pending interrupt */

          for (i = 0; i < 16; i++)
            {
              /* Is this interrupt pending */

              if ((pending & (IOPORT_CNSTAT(i))) != 0)
                {
                  /* Yes.. Has the user attached a handler? */

                  handler = handlers->handler[i].entry;
                  if (handler)
                    {
                      /* Yes.. call the attached handler */

                      status = handler(irq, context,
                                       handlers->handler[i].arg);

                      /* Keep track of the status of the last handler that
                       * failed.
                       */

                      if (status < 0)
                        {
                          ret = status;
                        }
                    }
                }
            }

          /* Clear pending status. */

          regval = getreg32(base + PIC32MZ_IOPORT_PORT_OFFSET);
          UNUSED(regval);
        }

      /* Edge detect mode selected. */

      else
        {
          /* Get the interrupt status associated with this interrupt.
           * The interrupt is controlled by either CNEN (positive edge)
           * or CNNE (negative edge).
           */

          cnen    = getreg32(base + PIC32MZ_IOPORT_CNEN_OFFSET);
          cnne    = getreg32(base + PIC32MZ_IOPORT_CNNE_OFFSET);
          cnf     = getreg32(base + PIC32MZ_IOPORT_CNF_OFFSET);
          pending = cnf & (cnen | cnne);

          /* Call all attached handlers for each pending interrupt */

          for (i = 0; i < 16; i++)
            {
              /* Is this interrupt pending */

              if ((pending & (IOPORT_CNF(i))) != 0)
                {
                  /* Yes.. Has the user attached a handler? */

                  handler = handlers->handler[i].entry;
                  if (handler)
                    {
                      /* Yes.. call the attached handler */

                      status = handler(irq, context,
                                       handlers->handler[i].arg);

                      /* Keep track of the status of the last handler that
                       * failed.
                       */

                      if (status < 0)
                        {
                          ret = status;
                        }
                    }
                }
            }

          /* Clear pending status. */

          putreg32(pending, base + PIC32MZ_IOPORT_CNFCLR_OFFSET);
        }
    }

  /* Clear the pending interrupt */

  up_clrpend_irq(irq);
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
  uintptr_t base;
  uint32_t regval;
  int ret;
  int i;

  /* Perform initialization for each IO port that has interrupt support
   * enabled.  We can tell this because there will be an array of second
   * level handlers for each enabled IO port.
   */

  for (i = 0; i < CHIP_NPORTS; i++)
    {
      /* Get the base address of this IO port peripheral */

      base = g_gpiobase[i];
      DEBUGASSERT(base);

      /* Reset all registers and disable the CN module */

      putreg32(IOPORT_CNEN_ALL, base + PIC32MZ_IOPORT_CNENCLR_OFFSET);
      putreg32(IOPORT_CNNE_ALL, base + PIC32MZ_IOPORT_CNNECLR_OFFSET);
      putreg32(IOPORT_CNPU_ALL, base + PIC32MZ_IOPORT_CNPUCLR_OFFSET);
      putreg32(IOPORT_CNPD_ALL, base + PIC32MZ_IOPORT_CNPDCLR_OFFSET);
      putreg32(0,               base + PIC32MZ_IOPORT_CNCON_OFFSET);

      /* Is interrupt support selected for this IO port? */

      if (g_level2_handlers[i] != NULL)
        {
          /* Yes.. Attach the common change notice interrupt handler
           * to the IO port interrupt.  Notice that this assumes that
           * each IRQ number is consecutive beginning with IOPORTA.
           */

          ret = irq_attach(PIC32MZ_IRQ_PORTA + i, pic32mz_cninterrupt, NULL);
          DEBUGASSERT(ret == OK);
          UNUSED(ret);

          /* Enable the CN module.  NOTE that the CN module is active when
           * in sleep mode.
           */

          putreg32(IOPORT_CNCON_ON, base + PIC32MZ_IOPORT_CNCON_OFFSET);

          /* Read the port to clear the interrupt. */

          regval = getreg32(base + PIC32MZ_IOPORT_PORT_OFFSET);
          UNUSED(regval);

          /* Clear the CN interrupt flag. Same assumption as above. */

          up_clrpend_irq(PIC32MZ_IRQ_PORTA + i);
        }
    }
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
 *   maintain state and determine if the underlying GPIO input value changed.
 *
 * Input Parameters:
 *   pinset  - GPIO pin configuration
 *   handler - Interrupt handler (may be NULL to detach)
 *   arg     - The argument that accompanies the interrupt
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated error value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

int pic32mz_gpioattach(pinset_t pinset, xcpt_t handler, void *arg)
{
  struct ioport_level2_s *handlers;
  irqstate_t flags;
  uintptr_t base;
  int ioport;
  int pin;

  /* First verify that the pinset is configured as an interrupting input */

  if (pic32mz_input(pinset) && pic32mz_interrupt(pinset))
    {
      /* Get the ioport index and pin number from the pinset */

      ioport = pic32mz_ioport(pinset);
      pin    = pic32mz_pin(pinset);
      DEBUGASSERT(ioport >= 0 && ioport < CHIP_NPORTS);

      /* Get the register base address for this port */

      base = g_gpiobase[ioport];
      DEBUGASSERT(base);

      /* If this IO port has been properly configured for interrupts, then
       * there should be an array of second level interrupt handlers
       * allocated for it.
       */

      handlers = g_level2_handlers[ioport];
      DEBUGASSERT(handlers);
      if (handlers)
        {
          /* Get the previously attached handler as the return value */

          flags = enter_critical_section();

          /* Are we attaching or detaching? */

          if (handler != NULL)
            {
              /* Attaching... Make sure that the GPIO is properly configured
               * as an input
               */

              pic32mz_configgpio(pinset);

              /* Pull-up requested? */

              if (pic32mz_pullup(pinset))
                {
                  putreg32(1 << pin, base + PIC32MZ_IOPORT_CNPUSET_OFFSET);
                }
              else
                {
                  putreg32(1 << pin, base + PIC32MZ_IOPORT_CNPUCLR_OFFSET);
                }

              /* Pull-down requested? */

              if (pic32mz_pulldown(pinset))
                {
                  putreg32(1 << pin, base + PIC32MZ_IOPORT_CNPDSET_OFFSET);
                }
              else
                {
                  putreg32(1 << pin, base + PIC32MZ_IOPORT_CNPDCLR_OFFSET);
                }
            }
          else
            {
              /* Disable the pull-up/downs (just so things look better in
               * the debugger).
               */

               putreg32(1 << pin, base + PIC32MZ_IOPORT_CNPUCLR_OFFSET);
               putreg32(1 << pin, base + PIC32MZ_IOPORT_CNPDCLR_OFFSET);
            }

          /* Whether attaching or detaching, the next state of the interrupt
           * should be disabled.
           */

          putreg32(1 << pin, base + PIC32MZ_IOPORT_CNENCLR_OFFSET);
          putreg32(1 << pin, base + PIC32MZ_IOPORT_CNNECLR_OFFSET);

          /* Set the new handler (perhaps NULLifying the current handler) */

          handlers->handler[pin].entry = handler;
          handlers->handler[pin].arg   = arg;
          leave_critical_section(flags);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: pic32mz_gpioirqenable
 *
 * Description:
 *   Enable the interrupt for specified GPIO IRQ
 *
 ****************************************************************************/

void pic32mz_gpioirqenable(pinset_t pinset)
{
  uintptr_t base;
  int ioport;
  int pin;

  /* Get the ioport index and pin number from the pinset */

  ioport = pic32mz_ioport(pinset);
  pin    = pic32mz_pin(pinset);
  DEBUGASSERT(ioport >= 0 && ioport < CHIP_NPORTS);

  /* Get the register base address for this port */

  base = g_gpiobase[ioport];
  DEBUGASSERT(base);
  if (base)
    {
      /* Enable the correct CN interrupt.
       * NOTE that we don't actually check if
       * interrupts are configured for this IO port.  If not, this operation
       * should do nothing.
       */

      if (!pic32mz_edgedetect(pinset))
        {
          /* If Edge detect is not selected, then CNEN
           * controls the interrupt.
           */

          putreg32(1 << pin, base + PIC32MZ_IOPORT_CNENSET_OFFSET);
        }
      else
        {
          /* Enable edge detect. */

          putreg32(IOPORT_CNCON_EDGEDETECT,
                   base + PIC32MZ_IOPORT_CNCONSET_OFFSET);

          if (pic32mz_edgemode(pinset) == GPIO_EDGE_RISING)
            {
              /* Rising edge selected. CNEN controls the interrupt. */

              putreg32(1 << pin, base + PIC32MZ_IOPORT_CNENSET_OFFSET);
            }
          else
            {
              /* Falling edge selected. CNNE controls the interrupt. */

              putreg32(1 << pin, base + PIC32MZ_IOPORT_CNNESET_OFFSET);
            }
        }

      /* And enable the interrupt. */

      up_enable_irq(ioport + PIC32MZ_IRQ_PORTA);
    }
}

/****************************************************************************
 * Name: pic32mz_gpioirqdisable
 *
 * Description:
 *   Disable the interrupt for specified GPIO IRQ
 *
 ****************************************************************************/

void pic32mz_gpioirqdisable(pinset_t pinset)
{
  uintptr_t base;
  int ioport;
  int pin;

  /* Get the ioport index and pin number from the pinset */

  ioport = pic32mz_ioport(pinset);
  pin    = pic32mz_pin(pinset);
  DEBUGASSERT(ioport >= 0 && ioport < CHIP_NPORTS);

  /* Get the register base address for this port */

  base = g_gpiobase[ioport];
  DEBUGASSERT(base);
  if (base)
    {
      /* And disable the correct interrupt.
       * NOTE that we don't actually check if
       * interrupts are configured for this IO port.  If not, this operation
       * should do nothing.
       */

      if (!pic32mz_edgedetect(pinset))
        {
          /* If Edge detect is not selected, then CNEN
           * controls the interrupt.
           */

          putreg32(1 << pin, base + PIC32MZ_IOPORT_CNENCLR_OFFSET);
        }
      else
        {
          if (pic32mz_edgemode(pinset) == GPIO_EDGE_RISING)
            {
              /* Rising edge selected.
               * CNEN controls the interrupt.
               */

              putreg32(1 << pin, base + PIC32MZ_IOPORT_CNENCLR_OFFSET);
            }
          else
            {
              /* Falling edge selected.
               * CNNE controls the interrupt.
               */

              putreg32(1 << pin, base + PIC32MZ_IOPORT_CNNECLR_OFFSET);
            }
        }
    }
}

#endif /* CONFIG_PIC32MZ_GPIOIRQ */
