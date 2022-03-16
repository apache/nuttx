/****************************************************************************
 * arch/risc-v/src/rv32m1/rv32m1_gpio.c
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
#include <nuttx/nuttx.h>

#include <stdint.h>
#include <assert.h>
#include <debug.h>
#include <queue.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include <nuttx/kmalloc.h>

#include "riscv_internal.h"
#include "chip.h"
#include "hardware/rv32m1_port.h"
#include "hardware/rv32m1_gpio.h"
#include "rv32m1_pcc.h"
#include "rv32m1_gpio.h"

/****************************************************************************
 * Pre-Processor Declarations
 ****************************************************************************/

#define RV32M1_NGPIO_PORTS   5

/****************************************************************************
 * Data Type Definition
 ****************************************************************************/

struct rv32m1_ctrlbase_s
{
  const uint32_t portbase;  /* Port Configure Base */
  const uint32_t gpiobase;  /* GPIO Configure Base */
  const uint32_t portgate;  /* Port Clock Control Gate */
  const uint32_t gpiogate;  /* GPIO Clock Control Gate */
  const uint32_t irq;       /* IRQ Number */
  sq_queue_t *isrchain;     /* Interrupt Service Routine Chain */
};

struct rv32m1_isr_s
{
  sq_entry_t link;
  xcpt_t     isr;
  void   *arg;
  uint32_t   pin;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if RV32M1_NGPIO_PORTS > 0
static sq_queue_t g_porta_isrchain =
{
  .head = NULL,
  .tail = NULL
};
#endif
#if RV32M1_NGPIO_PORTS > 1
static sq_queue_t g_portb_isrchain =
{
  .head = NULL,
  .tail = NULL
};
#endif
#if RV32M1_NGPIO_PORTS > 2
static sq_queue_t g_portc_isrchain =
{
  .head = NULL,
  .tail = NULL
};
#endif
#if RV32M1_NGPIO_PORTS > 3
static sq_queue_t g_portd_isrchain =
{
  .head = NULL,
  .tail = NULL
};
#endif
#if RV32M1_NGPIO_PORTS > 4
static sq_queue_t g_porte_isrchain =
{
  .head = NULL,
  .tail = NULL
};
#endif

/* Base addresses for each GPIO block */

static struct rv32m1_ctrlbase_s g_ctrlbase[RV32M1_NGPIO_PORTS] =
{
#if RV32M1_NGPIO_PORTS > 0
  {
    .portbase = RV32M1_PORTA_BASE,
    .gpiobase = RV32M1_GPIOA_BASE,
    .portgate = RV32M1_PCC_PORTA,
    .gpiogate = 0,
    .irq      = RV32M1_IRQ_PORTA,
    .isrchain = &g_porta_isrchain,
  },
#endif
#if RV32M1_NGPIO_PORTS > 1
  {
    .portbase = RV32M1_PORTB_BASE,
    .gpiobase = RV32M1_GPIOB_BASE,
    .portgate = RV32M1_PCC_PORTB,
    .gpiogate = 0,
    .irq      = RV32M1_IRQ_PORTB,
    .isrchain = &g_portb_isrchain,
  },
#endif
#if RV32M1_NGPIO_PORTS > 2
  {
    .portbase = RV32M1_PORTC_BASE,
    .gpiobase = RV32M1_GPIOC_BASE,
    .portgate = RV32M1_PCC_PORTC,
    .gpiogate = 0,
    .irq      = RV32M1_IRQ_PORTC,
    .isrchain = &g_portc_isrchain,
  },
#endif
#if RV32M1_NGPIO_PORTS > 3
  {
    .portbase = RV32M1_PORTD_BASE,
    .gpiobase = RV32M1_GPIOD_BASE,
    .portgate = RV32M1_PCC_PORTD,
    .gpiogate = 0,
    .irq      = RV32M1_IRQ_PORTD,
    .isrchain = &g_portd_isrchain,
  },
#endif
#if RV32M1_NGPIO_PORTS > 4
  {
    .portbase = RV32M1_PORTE_BASE,
    .gpiobase = RV32M1_GPIOE_BASE,
    .portgate = RV32M1_PCC_PORTE,
    .gpiogate = RV32M1_PCC_GPIOE,
    .irq      = RV32M1_IRQ_PORTE,
    .isrchain = &g_porte_isrchain,
  },
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rv32m1_modifyreg32
 ****************************************************************************/

static inline void rv32m1_modifyreg32(uint32_t addr, uint32_t bitclr,
                                      uint32_t bitset)
{
  uint32_t regval = getreg32(addr);
  regval &= ~bitclr;
  regval |= bitset;
  putreg32(regval, addr);
}

/****************************************************************************
 * Name: rv32m1_gpio_irqconfig
 ****************************************************************************/

static void rv32m1_gpio_irqconfig(uint32_t cfgset)
{
  uint32_t portbase;

  unsigned int port;
  unsigned int pin;

  uint32_t irqc;

  /* Verify that this hardware supports the selected GPIO port */

  port = (cfgset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;

  /* Get the port base address */

  portbase = g_ctrlbase[port].portbase;

  pin = (cfgset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  irqc = (cfgset & GPIO_INT_MASK) >> GPIO_INT_SHIFT;

  /* Write '1' to clear Interrupt flag */

  putreg32(pin << 1, portbase + RV32M1_PORT_ISFR_OFFSET);

  rv32m1_modifyreg32(portbase + (pin << 2), PORT_PCR_IRQC_MASK,
                     (irqc << PORT_PCR_IRQC_SHIFT) & PORT_PCR_IRQC_MASK);
}

/****************************************************************************
 * Name: rv32m1_gpio_portconfig
 ****************************************************************************/

static void rv32m1_gpio_portconfig(uint32_t cfgset)
{
  uint32_t portbase;

  unsigned int port;
  unsigned int pin;

  uint32_t regval;
  uint32_t cfg;

  /* No Sanity check of port */

  port = (cfgset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;

  /* Get the port base address */

  portbase = g_ctrlbase[port].portbase;

  pin = (cfgset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

  regval = getreg32(portbase + (pin << 2));

  cfg = cfgset & GPIO_PUPD_MASK;

  if (cfg == GPIO_FLOAT)
    {
      /* Float */

      regval &= ~PORT_PCR_PE;
    }
  else
    {
      /* Pull Enable */

      regval |= PORT_PCR_PE;

      if (cfg == GPIO_PULLUP)
        {
          /* Pull Up */

          regval |= PORT_PCR_PS;
        }
      else
        {
          /* Pull Down */

          regval &= ~PORT_PCR_PS;
        }
    }

  if (cfgset & GPIO_OPENDRAIN)
    {
      /* Open Drain Enable */

      regval |= PORT_PCR_ODE;
    }
  else
    {
      /* Open Drain Disable */

      regval &= ~PORT_PCR_ODE;
    }

  if (cfgset & GPIO_FILTER)
    {
      /* Passive Filter Enable */

      regval |= PORT_PCR_PFE;
    }
  else
    {
      /* Passive Filter Disable */

      regval &= ~PORT_PCR_PFE;
    }

  if (cfgset & GPIO_SSR)
    {
      /* Slow Slew Rate Enable */

      regval |= PORT_PCR_SRE;
    }
  else
    {
      /* Fast Slew Rate Enable */

      regval &= ~PORT_PCR_SRE;
    }

  putreg32(regval, portbase + (pin << 2));
}

/****************************************************************************
 * Name: rv32m1_gpio_interrupt
 ****************************************************************************/

LOCATE_ITCM
static int rv32m1_gpio_interrupt(int irq, void *context, void *arg)
{
  const struct rv32m1_ctrlbase_s *ctrl;
  const sq_queue_t *isrchain;
  const sq_entry_t *e;
  const struct rv32m1_isr_s *priv;

  uint32_t portbase;

  uint32_t risf; /* the Read([red]) Interrupt status Flag */
  uint32_t wisf; /* The Interrupt status Flag to write back */

  ctrl = (const struct rv32m1_ctrlbase_s *)arg;
  portbase = ctrl->portbase;
  isrchain = ctrl->isrchain;

  risf = getreg32(portbase + RV32M1_PORT_ISFR_OFFSET);
  wisf = 0;

  /* Enumerate gpio isr chain to dispatch services */

  e = sq_peek(isrchain);
  while (e)
    {
      priv = container_of(e, const struct rv32m1_isr_s, link);

      /* Dispatch services to whom has subcribed(registered) the
       * corresponding pin.
       */

      if (risf & (1 << priv->pin))
        {
          wisf |= 1 << priv->pin;

          /* Double check isr to avoid System Crash */

          if (priv->isr)
            {
              priv->isr(irq, context, priv->arg);
            }
        }

      e = sq_next(e);
    }

  /* Write '1' to clear corresponding Interrupt Status bit Flag */

  putreg32(wisf, portbase + RV32M1_PORT_ISFR_OFFSET);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rv32m1_gpio_config
 ****************************************************************************/

int rv32m1_gpio_config(uint32_t cfgset)
{
  uint32_t gpiobase;
  uint32_t portbase;
  unsigned int port;
  unsigned int pin;
  unsigned int mode;

  irqstate_t flags;

  /* Verify that this hardware supports the selected GPIO port */

  port = (cfgset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  if (port >= RV32M1_NGPIO_PORTS)
    {
      return -EINVAL;
    }

  /* Get the gpio and port base address */

  gpiobase = g_ctrlbase[port].gpiobase;
  portbase = g_ctrlbase[port].portbase;

  /* Get the pin number and select the port configuration register for that
   * pin
   */

  pin = (cfgset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

  /* Interrupts must be disabled from here on out so that we have mutually
   * exclusive access to all of the GPIO configuration registers.
   */

  flags = enter_critical_section();

  rv32m1_pcc_clock_enable(g_ctrlbase[port].portgate);

  /* Open the GPIO GATE if required */

  if (g_ctrlbase[port].gpiogate)
    {
      rv32m1_pcc_clock_enable(g_ctrlbase[port].gpiogate);
    }

  /* Cofigure Open Drain, Pull Up/Down, Filter and Slew Rate abilities */

  rv32m1_gpio_portconfig(cfgset);

  mode = cfgset & GPIO_MODE_MASK;

  if (mode == GPIO_INPUT)
    {
      rv32m1_modifyreg32(portbase + (pin << 2), PORT_PCR_MUX_MASK,
                         PORT_PCR_MUX_GPIO);
      rv32m1_modifyreg32(gpiobase + RV32M1_GPIO_PDDR_OFFSET, 1 << pin, 0);

      /* Always configure the irq on this pin, even if the irq configure
       * could be None.
       */

      rv32m1_gpio_irqconfig(cfgset);
    }
  else if (mode == GPIO_OUTPUT)
    {
      rv32m1_modifyreg32(portbase + (pin << 2), PORT_PCR_MUX_MASK,
                         PORT_PCR_MUX_GPIO);
      rv32m1_modifyreg32(gpiobase + RV32M1_GPIO_PDDR_OFFSET, 0, 1 << pin);

      /* Initialize the output value */

      if (cfgset & GPIO_OUTPUT_SET)
        {
          putreg32(1 << pin, gpiobase + RV32M1_GPIO_PSOR_OFFSET);
        }
      else
        {
          putreg32(1 << pin, gpiobase + RV32M1_GPIO_PCOR_OFFSET);
        }
    }
  else
    {
      uint32_t alt = (cfgset & GPIO_AF_MASK) >> GPIO_AF_SHIFT;

      rv32m1_modifyreg32(portbase + (pin << 2), PORT_PCR_MUX_MASK,
                         (alt << PORT_PCR_MUX_SHIFT) & PORT_PCR_MUX_MASK);
    }

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: rv32m1_gpio_write
 ****************************************************************************/

void rv32m1_gpio_write(uint32_t cfgset, bool value)
{
  uint32_t gpiobase;
  unsigned int port;
  unsigned int pin;

  /* Verify that this hardware supports the selected GPIO port */

  port = (cfgset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  if (port >= RV32M1_NGPIO_PORTS)
    {
      return ;
    }

  /* Get the gpio base address */

  gpiobase = g_ctrlbase[port].gpiobase;

  pin = (cfgset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

  if (value)
    {
      putreg32(1 << pin, gpiobase + RV32M1_GPIO_PSOR_OFFSET);
    }
  else
    {
      putreg32(1 << pin, gpiobase + RV32M1_GPIO_PCOR_OFFSET);
    }
}

/****************************************************************************
 * Name: rv32m1_gpio_toggle
 ****************************************************************************/

void rv32m1_gpio_toggle(uint32_t cfgset)
{
  uint32_t gpiobase;
  unsigned int port;
  unsigned int pin;

  /* Verify that this hardware supports the selected GPIO port */

  port = (cfgset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  if (port >= RV32M1_NGPIO_PORTS)
    {
      return ;
    }

  /* Get the gpio base address */

  gpiobase = g_ctrlbase[port].gpiobase;

  pin = (cfgset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  putreg32(1 << pin, gpiobase + RV32M1_GPIO_PTOR_OFFSET);
}

/****************************************************************************
 * Name: rv32m1_gpio_read
 ****************************************************************************/

bool rv32m1_gpio_read(uint32_t cfgset)
{
  uint32_t gpiobase;
  unsigned int port;
  unsigned int pin;

  /* Verify that this hardware supports the selected GPIO port */

  port = (cfgset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  if (port >= RV32M1_NGPIO_PORTS)
    {
      return false;
    }

  /* Get the gpio base address */

  gpiobase = g_ctrlbase[port].gpiobase;

  pin = (cfgset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  return (getreg32(gpiobase + RV32M1_GPIO_PDIR_OFFSET) & (1 << pin)) != 0;
}

/****************************************************************************
 * Name: rv32m1_gpio_irqenable
 ****************************************************************************/

void rv32m1_gpio_irqenable(uint32_t cfgset)
{
  uint32_t irq;
  unsigned int port;

  /* Verify that this hardware supports the selected GPIO port */

  port = (cfgset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  if (port >= RV32M1_NGPIO_PORTS)
    {
      return ;
    }

  /* Get the irq */

  irq = g_ctrlbase[port].irq;

  /* Configure the irq on the pin */

  rv32m1_gpio_irqconfig(cfgset);

  up_enable_irq(irq);
}

/****************************************************************************
 * Name: rv32m1_gpio_irqdisable
 ****************************************************************************/

void rv32m1_gpio_irqdisable(uint32_t cfgset)
{
  uint32_t portbase;
  unsigned int port;
  unsigned int pin;

  /* Verify that this hardware supports the selected GPIO port */

  port = (cfgset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  if (port >= RV32M1_NGPIO_PORTS)
    {
      return ;
    }

  /* Get the port base address */

  portbase = g_ctrlbase[port].portbase;

  pin = (cfgset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  rv32m1_modifyreg32(portbase + (pin << 2), PORT_PCR_IRQC_MASK, 0);
}

/****************************************************************************
 * Name: rv32m1_gpio_irqattach
 ****************************************************************************/

int rv32m1_gpio_irqattach(uint32_t cfgset, xcpt_t isr, void *arg)
{
  unsigned int port;
  unsigned int pin;
  uint32_t irq;

  int ret;

  irqstate_t flags;

  sq_queue_t *isrchain;
  sq_entry_t *e;

  struct rv32m1_isr_s *priv;

  if (!isr)
    {
      return -EINVAL;
    }

  /* Verify that this hardware supports the selected GPIO port */

  port = (cfgset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  if (port >= RV32M1_NGPIO_PORTS)
    {
      return -EINVAL;
    }

  pin = (cfgset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

  isrchain = g_ctrlbase[port].isrchain;
  irq      = g_ctrlbase[port].irq;

  flags = enter_critical_section();

  e = sq_peek(isrchain);
  while (e)
    {
      priv = container_of(e, struct rv32m1_isr_s, link);
      if (priv->isr == isr && priv->pin == pin && priv->arg == arg)
        {
          /* The isr has been one 'link' of Chain */

          ret = 0;
          goto done;
        }

      e = sq_next(e);
    }

  priv = (struct rv32m1_isr_s *)kmm_malloc(sizeof(*priv));
  if (priv)
    {
      /* If it is the first time to attach an isr, the generic gpio
       * isr has to be attached.
       */

      if (!sq_peek(isrchain))
        {
          irq_attach(irq, rv32m1_gpio_interrupt, &g_ctrlbase[port]);
        }

      priv->isr = isr;
      priv->arg = arg;
      priv->pin = pin;

      /* Append the new isr to Chain */

      sq_addlast(&priv->link, isrchain);

      ret = 0;
    }
  else
    {
      ret = -ENOMEM;
    }

done:
  leave_critical_section(flags);

  return ret;
}

/****************************************************************************
 * Name: rv32m1_gpio_irqdetach
 ****************************************************************************/

int rv32m1_gpio_irqdetach(uint32_t cfgset, xcpt_t isr, void *arg)
{
  uint32_t port;
  uint32_t pin;

  int ret = -1;

  irqstate_t flags;

  sq_queue_t *isrchain;
  sq_entry_t *cur;
  sq_entry_t *pre;

  struct rv32m1_isr_s *priv;

  if (!isr)
    {
      return -EINVAL;
    }

  /* Verify that this hardware supports the selected GPIO port */

  port = (cfgset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  if (port >= RV32M1_NGPIO_PORTS)
    {
      return -EINVAL;
    }

  pin = (cfgset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

  /* Get the isr chain */

  isrchain = g_ctrlbase[port].isrchain;

  flags = enter_critical_section();

  pre = NULL;
  cur = sq_peek(isrchain);
  while (cur)
    {
      priv = container_of(cur, struct rv32m1_isr_s, link);
      if (priv->isr == isr && priv->pin == pin && priv->arg == arg)
        {
          if (pre)
            {
              sq_remafter(pre, isrchain);
              cur = pre;
            }
          else
            {
              sq_remfirst(isrchain);
              cur = NULL;
            }

          /* Return back resources */

          kmm_free(priv);

          ret = 0;
        }

      pre = cur;
      cur = cur ? sq_next(cur) : sq_peek(isrchain);
    }

  leave_critical_section(flags);

  return ret;
}

/****************************************************************************
 * Name: rv32m1_gpio_clearpending
 ****************************************************************************/

void rv32m1_gpio_clearpending(uint32_t cfgset)
{
  uint32_t portbase;
  unsigned int port;
  unsigned int pin;

  /* Verify that this hardware supports the selected GPIO port */

  port = (cfgset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  if (port >= RV32M1_NGPIO_PORTS)
    {
      return;
    }

  /* Get the port base address */

  portbase = g_ctrlbase[port].portbase;

  pin = (cfgset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

  /* Write '1' to clear corresponding Interrupt Status bit Flag */

  putreg32(1 << pin, portbase + RV32M1_PORT_ISFR_OFFSET);
}
