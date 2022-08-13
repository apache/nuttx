/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_gpioint.c
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

#include <arch/board/board.h>
#include <nuttx/config.h>

#include <assert.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/spinlock.h>

#include "arm_internal.h"
#include "chip.h"

#include "cxd56_pinconfig.h"
#include "cxd56_gpio.h"
#include "cxd56_gpioint.h"
#include "hardware/cxd5602_topreg.h"

#ifdef CONFIG_CXD56_GPIO_IRQ

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GPIO Interrupt Polarity Definitions */

#define GPIOINT_POLARITY_SHIFT      (0)
#define GPIOINT_POLARITY_MASK       (7)
#define GPIOINT_GET_POLARITY(v)     (((v) & GPIOINT_POLARITY_MASK) >> GPIOINT_POLARITY_SHIFT)
#define GPIOINT_SET_POLARITY(v)     (((v) << GPIOINT_POLARITY_SHIFT) & GPIOINT_POLARITY_MASK)
#define GPIOINT_IS_LEVEL(v)         (GPIOINT_GET_POLARITY(v) <= GPIOINT_LEVEL_LOW)
#define GPIOINT_IS_EDGE(v)          (GPIOINT_EDGE_RISE <= GPIOINT_GET_POLARITY(v))
#define GPIOINT_IS_HIGH(v)          ((GPIOINT_LEVEL_HIGH == GPIOINT_GET_POLARITY(v)) || \
                                     (GPIOINT_EDGE_RISE == GPIOINT_GET_POLARITY(v)))
#define GPIOINT_IS_LOW(v)           ((GPIOINT_LEVEL_LOW == GPIOINT_GET_POLARITY(v)) || \
                                     (GPIOINT_EDGE_FALL == GPIOINT_GET_POLARITY(v)))

/* GPIO Interrupt Noise Filter Definitions */

#define GPIOINT_NOISE_FILTER_SHIFT      (3)
#define GPIOINT_NOISE_FILTER_MASK       (1u << GPIOINT_NOISE_FILTER_SHIFT)
#define GPIOINT_NOISE_FILTER_ENABLED(v) (((v) & GPIOINT_NOISE_FILTER_MASK) \
                                         == GPIOINT_NOISE_FILTER_ENABLE)

/* Use Pseudo Edge Interrupt */

#define GPIOINT_TOGGLE_MODE_SHIFT       (16)

/* GPIO Interrupt Index Number Definitions */

#define MAX_SLOT                (12)
#define MAX_SYS_SLOT            (6)
#define INTSEL_DEFAULT_VAL      (63)

#define GET_SLOT2IRQ(slot)      (CXD56_IRQ_EXDEVICE_0 + (slot))
#define GET_IRQ2SLOT(irq)       ((irq) - CXD56_IRQ_EXDEVICE_0)

/* PMU_WAKE_TRIG_CPUINTSELx */

#define INT_ROUTE_THROUGH       (0)
#define INT_ROUTE_INVERTER      (1)
#define INT_ROUTE_PMU           (2)
#define INT_ROUTE_PMU_LATCH     (3)

#define CXD56_INTC_ENABLE       (CXD56_INTC_BASE + 0x10)
#define CXD56_INTC_INVERT       (CXD56_INTC_BASE + 0x20)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static xcpt_t g_isr[MAX_SLOT];
static uint32_t g_bothedge = 0;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* allocate/deallocate/get slot number (SYS: 0~5, APP: 6~11) */

static int alloc_slot(int pin, bool isalloc)
{
  irqstate_t flags;
  int alloc = -1;
  int slot;
  uint8_t val;
  uint32_t base = (pin < PIN_IS_CLK) ? CXD56_TOPREG_IOCSYS_INTSEL0
                                     : CXD56_TOPREG_IOCAPP_INTSEL0;
  int offset = (pin < PIN_IS_CLK) ? 1 : 56;

  flags = spin_lock_irqsave(NULL);

  for (slot = 0; slot < MAX_SYS_SLOT; slot++)
    {
      val = getreg8(base + slot);
      if ((pin - offset) == val)
        {
          if (isalloc == false)
            {
              putreg8(INTSEL_DEFAULT_VAL, base + slot);
            }

          break; /* already used */
        }

      if ((-1 == alloc) && (INTSEL_DEFAULT_VAL == val))
        {
          alloc = slot;
        }
    }

  if (slot == MAX_SYS_SLOT)
    {
      if (isalloc && (-1 != alloc))
        {
          slot = alloc;
          putreg8(pin - offset, base + slot);
        }
      else
        {
          spin_unlock_irqrestore(NULL, flags);
          return -ENXIO; /* no space */
        }
    }

  spin_unlock_irqrestore(NULL, flags);

  if (PIN_IS_CLK <= pin)
    {
      slot += MAX_SYS_SLOT;
    }

  return slot;
}

/* convert from slot to pin */

static int get_slot2pin(int slot)
{
  uint32_t base = (slot < MAX_SYS_SLOT) ? CXD56_TOPREG_IOCSYS_INTSEL0
                                        : CXD56_TOPREG_IOCAPP_INTSEL0;
  int offset = 1;

  if (MAX_SYS_SLOT <= slot)
    {
      slot -= MAX_SYS_SLOT;
      offset = 56;
    }

  return (int)getreg8(base + slot) + offset;
}

/* convert from pin to slot number (SYS: 0~5, APP: 6~11) */

static int get_pin2slot(int pin)
{
  int slot;
  uint32_t base = (pin < PIN_IS_CLK) ? CXD56_TOPREG_IOCSYS_INTSEL0
                                     : CXD56_TOPREG_IOCAPP_INTSEL0;
  int offset = (pin < PIN_IS_CLK) ? 1 : 56;

  for (slot = 0; slot < MAX_SYS_SLOT; slot++)
    {
      if ((pin - offset) == getreg8(base + slot)) /* byte access */
        {
          break;
        }
    }

  if (slot == MAX_SYS_SLOT)
    {
      return -1;
    }

  if (PIN_IS_CLK <= pin)
    {
      slot += MAX_SYS_SLOT;
    }

  return slot;
}

/* convert from pin to irq number */

static int get_pin2irq(int pin)
{
  int slot = get_pin2slot(pin);

  if ((0 <= slot) && (slot < MAX_SLOT))
    {
      return GET_SLOT2IRQ(slot);
    }
  else
    {
      return -1;
    }
}

/* set GPIO interrupt configuration registers */

static int set_gpioint_config(int slot, uint32_t gpiocfg)
{
  uint32_t val;
  uint32_t shift;
  uint32_t polreg = CXD56_TOPREG_PMU_WAKE_TRIG_INTDET0;
  uint32_t selreg = CXD56_TOPREG_PMU_WAKE_TRIG_CPUINTSEL0;

  /* Configure the noise filter */

  val = getreg32(CXD56_TOPREG_PMU_WAKE_TRIG_NOISECUTEN0);
  if (GPIOINT_NOISE_FILTER_ENABLED(gpiocfg))
    {
      val |= (1 << (slot + 16));
    }
  else
    {
      val &= ~(1 << (slot + 16));
    }

  putreg32(val, CXD56_TOPREG_PMU_WAKE_TRIG_NOISECUTEN0);

  /* Configure the polarity */

  shift = 16 + (slot * 4);
  if (32 <= shift)
    {
      polreg = CXD56_TOPREG_PMU_WAKE_TRIG_INTDET1;
      selreg = CXD56_TOPREG_PMU_WAKE_TRIG_CPUINTSEL1;
      shift -= 32;
    }

  val = getreg32(polreg);
  val &= ~(0x7 << shift);
  val |= (GPIOINT_GET_POLARITY(gpiocfg) << shift);
  putreg32(val, polreg);

  /* Configure the interrupt route */

  val = getreg32(selreg);
  val &= ~(0x7 << shift);

  switch (GPIOINT_GET_POLARITY(gpiocfg))
    {
    case GPIOINT_LEVEL_HIGH:
      if (GPIOINT_NOISE_FILTER_ENABLED(gpiocfg))
        {
          val |= (INT_ROUTE_PMU << shift);
        }
      else
        {
          val |= (INT_ROUTE_THROUGH << shift);
        }
      break;
    case GPIOINT_LEVEL_LOW:
      if (GPIOINT_NOISE_FILTER_ENABLED(gpiocfg))
        {
          val |= (INT_ROUTE_PMU << shift);
        }
      else
        {
          val |= (INT_ROUTE_INVERTER << shift);
        }
      break;
    case GPIOINT_EDGE_RISE:
    case GPIOINT_EDGE_FALL:
    case GPIOINT_EDGE_BOTH:
      val |= (INT_ROUTE_PMU_LATCH << shift);
      break;
    default:
      DEBUGPANIC();
      break;
    }

  putreg32(val, selreg);

  return 0;
}

/* Invert interrupt polarity in INTC */

static void invert_irq(int irq)
{
  irqstate_t flags;
  uint32_t val;

  flags = spin_lock_irqsave(NULL);

  val = getreg32(CXD56_INTC_INVERT);
  val ^= (1 << (irq - CXD56_IRQ_EXTINT));
  putreg32(val, CXD56_INTC_INVERT);

  spin_unlock_irqrestore(NULL, flags);
}

static bool inverted_irq(int irq)
{
  uint32_t val;

  val = getreg32(CXD56_INTC_INVERT);
  return ((val & (1 << (irq - CXD56_IRQ_EXTINT))) != 0);
}

static bool enabled_irq(int irq)
{
  uint32_t val;

  val = getreg32(CXD56_INTC_ENABLE);
  return ((val & (1 << (irq - CXD56_IRQ_EXTINT))) != 0);
}

static int gpioint_handler(int irq, void *context, void *arg)
{
  uint32_t val;
  uint32_t shift;
  uint32_t polreg = CXD56_TOPREG_PMU_WAKE_TRIG_INTDET0;
  int slot = GET_IRQ2SLOT(irq);

  /* Invert mask of interrupt to be disable temporarily */

  invert_irq(irq);

  if (g_bothedge & (1 << slot))
    {
      g_isr[slot](irq, context, arg);
      return 0;
    }

  /* Get the polarity */

  shift = 16 + (slot * 4);
  if (32 <= shift)
    {
      polreg = CXD56_TOPREG_PMU_WAKE_TRIG_INTDET1;
      shift -= 32;
    }

  val = getreg32(polreg);
  val = (val >> shift) & 0x7;

  if (inverted_irq(irq))
    {
      /* Clear edge interrupt */

      if (GPIOINT_IS_EDGE(val))
        {
          /* TBD: ignore access protection */

          putreg32(1 << (slot + 16), CXD56_TOPREG_PMU_WAKE_TRIG0_CLR);
        }

      g_isr[slot](irq, context, arg);
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cxd56_gpioint_config
 *
 * Description:
 *   Configure a GPIO pin as an GPIO pin interrupt source
 *
 * Input Parameters:
 *   pin - Pin number defined in cxd56_pinconfig.h
 *   gpiocfg - GPIO Interrupt Polarity and Noise Filter Configuration Value
 *   isr - Interrupt handler. If isr is NULL, then free an allocated handler.
 *   arg - Argument for the interrupt handler
 *
 * Returned Value:
 *   IRQ number on success; a negated errno value on failure.
 *
 * Assumptions:
 *   The interrupt are disabled so that read-modify-write operations
 *   are safe.
 *
 ****************************************************************************/

int cxd56_gpioint_config(uint32_t pin, uint32_t gpiocfg, xcpt_t isr,
                         void *arg)
{
  int slot;
  int irq;
  irqstate_t flags;

  slot = alloc_slot(pin, (isr != NULL));
  if (slot < 0)
    {
      return -ENXIO;
    }

  irq = GET_SLOT2IRQ(slot);

  if (isr == NULL)
    {
      /* disable GPIO input */

      cxd56_gpio_config(pin, false);

      /* disable interrupt */

      irq_attach(irq, NULL, NULL);
      g_isr[slot] = NULL;

      flags = spin_lock_irqsave(NULL);
      g_bothedge &= ~(1 << slot);
      spin_unlock_irqrestore(NULL, flags);
      return irq;
    }

  /* enable GPIO input */

  cxd56_gpio_config(pin, true);

  /* set GPIO interrupt configuration */

  if (gpiocfg & GPIOINT_TOGGLE_BOTH_MASK)
    {
      /* set GPIO pseudo both edge interrupt */

      flags = spin_lock_irqsave(NULL);
      g_bothedge |= (1 << slot);
      spin_unlock_irqrestore(NULL, flags);

      /* detect the change from the current signal */

      if (true == cxd56_gpio_read(pin))
        {
          gpiocfg |= GPIOINT_SET_POLARITY(GPIOINT_LEVEL_LOW);
        }
      else
        {
          gpiocfg |= GPIOINT_SET_POLARITY(GPIOINT_LEVEL_HIGH);
        }
    }

  set_gpioint_config(slot, gpiocfg);

  if ((gpiocfg & GPIOINT_TOGGLE_MODE_MASK) || GPIOINT_IS_EDGE(gpiocfg))
    {
      irq_attach(irq, gpioint_handler, arg); /* call intermediate handler */
      g_isr[slot] = isr;
    }
  else
    {
      irq_attach(irq, isr, arg); /* call user handler directly */
      g_isr[slot] = NULL;
    }

  return irq;
}

/****************************************************************************
 * Name: cxd56_gpioint_irq
 *
 * Description:
 *   Get a GPIO interrupt number for specified pin number
 *
 * Returned Value:
 *   IRQ number on success; a negated errno value on failure.
 *
 ****************************************************************************/

int cxd56_gpioint_irq(uint32_t pin)
{
  return get_pin2irq(pin);
}

/****************************************************************************
 * Name: cxd56_gpioint_pin
 *
 * Description:
 *   Get a pin number for specified IRQ number
 *
 * Returned Value:
 *   Pin number on success; a negated errno value on failure.
 *
 ****************************************************************************/

int cxd56_gpioint_pin(int irq)
{
  int slot;

  if ((irq < CXD56_IRQ_EXDEVICE_0) || (CXD56_IRQ_EXDEVICE_11 < irq))
    {
      return -1;
    }

  slot = GET_IRQ2SLOT(irq);
  return get_slot2pin(slot);
}

/****************************************************************************
 * Name: cxd56_gpioint_enable
 *
 * Description:
 *   Enable a GPIO interrupt for specified pin number
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void cxd56_gpioint_enable(uint32_t pin)
{
  int irq = get_pin2irq(pin);

  if (irq > 0)
    {
      up_enable_irq(irq);
    }
}

/****************************************************************************
 * Name: cxd56_gpioint_disable
 *
 * Description:
 *   Disable a GPIO interrupt for specified pin number
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void cxd56_gpioint_disable(uint32_t pin)
{
  int irq = get_pin2irq(pin);

  if (irq > 0)
    {
      up_disable_irq(irq);
    }
}

/****************************************************************************
 * Name: cxd56_gpioint_invert
 *
 * Description:
 *   Invert polarity of a GPIO interrupt for specified pin number
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void cxd56_gpioint_invert(uint32_t pin)
{
  int irq = get_pin2irq(pin);

  if (irq > 0)
    {
      invert_irq(irq);
    }
}

/****************************************************************************
 * Name: cxd56_gpioint_status
 *
 * Description:
 *   Get a gpio interrupt status
 *
 * Returned Value:
 *   OK on success; A negated errno value on failure.
 *
 ****************************************************************************/

int cxd56_gpioint_status(uint32_t pin, cxd56_gpioint_status_t *stat)
{
  uint32_t val;
  uint32_t shift;
  uint32_t polreg = CXD56_TOPREG_PMU_WAKE_TRIG_INTDET0;
  int slot;

  DEBUGASSERT(stat);

  /* Get IRQ number */

  stat->irq = cxd56_gpioint_irq(pin);

  if (stat->irq < 0)
    {
      return -EINVAL;
    }

  /* Get polarity */

  slot = GET_IRQ2SLOT(stat->irq);
  shift = 16 + (slot * 4);
  if (32 <= shift)
    {
      polreg = CXD56_TOPREG_PMU_WAKE_TRIG_INTDET1;
      shift -= 32;
    }

  val = getreg32(polreg);
  stat->polarity = GPIOINT_GET_POLARITY(val >> shift);

  /* Replace for pseudo edge */

  if ((g_isr[slot]) && (stat->polarity == GPIOINT_LEVEL_HIGH))
    {
      stat->polarity = GPIOINT_EDGE_RISE;
    }

  if ((g_isr[slot]) && (stat->polarity == GPIOINT_LEVEL_LOW))
    {
      stat->polarity = GPIOINT_EDGE_FALL;
    }

  if ((g_isr[slot]) && (g_bothedge & (1 << slot)))
    {
      stat->polarity = GPIOINT_EDGE_BOTH;
    }

  /* Get noise filter enabled or not */

  val = getreg32(CXD56_TOPREG_PMU_WAKE_TRIG_NOISECUTEN0);
  stat->filter = ((val >> (slot + 16)) & 1) ? true : false;

  /* Get interrupt enabled or not */

  stat->enable = enabled_irq(stat->irq);

  return OK;
}

#endif /* CONFIG_CXD56_GPIO_IRQ */
