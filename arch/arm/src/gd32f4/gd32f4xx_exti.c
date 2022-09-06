/****************************************************************************
 * arch/arm/src/gd32f4/gd32f4xx_exti.c
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

#include <nuttx/irq.h>

#include "chip.h"
#include "gd32f4xx_exti.h"
#include "gd32f4xx.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct gd32_gpio_irq_s
{
  xcpt_t irqhandler;
  void  *arg;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint8_t g_exti_gpio_irqs[16] =
{
  GD32_IRQ_EXTI0, GD32_IRQ_EXTI1, GD32_IRQ_EXTI2, GD32_IRQ_EXTI3,
  GD32_IRQ_EXTI4, GD32_IRQ_EXTI5_9, GD32_IRQ_EXTI5_9, GD32_IRQ_EXTI5_9,
  GD32_IRQ_EXTI5_9, GD32_IRQ_EXTI5_9, GD32_IRQ_EXTI10_15, GD32_IRQ_EXTI10_15,
  GD32_IRQ_EXTI10_15, GD32_IRQ_EXTI10_15, GD32_IRQ_EXTI10_15,
  GD32_IRQ_EXTI10_15
};

/* Interrupt handlers attached to each EXTI */

static struct gd32_gpio_irq_s g_gpio_irq_s[16];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * GPIO EXTI linex interrupt handler
 ****************************************************************************/

static int gd32_exti0_irqhandler(int irq, void *context, void *arg)
{
  int ret = OK;

  /* Clear EXTI line0 pend flag */

  gd32_exti_interrupt_flag_clear(EXTI_0);

  if (g_gpio_irq_s[0].irqhandler != NULL)
    {
      xcpt_t irqhandler = g_gpio_irq_s[0].irqhandler;
      void  *cbarg    = g_gpio_irq_s[0].arg;

      ret = irqhandler(irq, context, cbarg);
    }

  return ret;
}

static int gd32_exti1_irqhandler(int irq, void *context, void *arg)
{
  int ret = OK;

  /* Clear EXTI line1 pend flag */

  gd32_exti_interrupt_flag_clear(EXTI_1);

  if (g_gpio_irq_s[1].irqhandler != NULL)
    {
      xcpt_t irqhandler = g_gpio_irq_s[1].irqhandler;
      void  *cbarg    = g_gpio_irq_s[1].arg;

      ret = irqhandler(irq, context, cbarg);
    }

  return ret;
}

static int gd32_exti2_irqhandler(int irq, void *context, void *arg)
{
  int ret = OK;

  /* Clear EXTI line2 pend flag */

  gd32_exti_interrupt_flag_clear(EXTI_2);

  if (g_gpio_irq_s[2].irqhandler != NULL)
    {
      xcpt_t irqhandler = g_gpio_irq_s[2].irqhandler;
      void  *cbarg    = g_gpio_irq_s[2].arg;

      ret = irqhandler(irq, context, cbarg);
    }

  return ret;
}

static int gd32_exti3_irqhandler(int irq, void *context, void *arg)
{
  int ret = OK;

  /* Clear EXTI line0 pend flag */

  gd32_exti_interrupt_flag_clear(EXTI_3);

  if (g_gpio_irq_s[3].irqhandler != NULL)
    {
      xcpt_t irqhandler = g_gpio_irq_s[3].irqhandler;
      void  *cbarg    = g_gpio_irq_s[3].arg;

      ret = irqhandler(irq, context, cbarg);
    }

  return ret;
}

static int gd32_exti4_irqhandler(int irq, void *context, void *arg)
{
  int ret = OK;

  /* Clear EXTI line0 pend flag */

  gd32_exti_interrupt_flag_clear(EXTI_4);

  if (g_gpio_irq_s[4].irqhandler != NULL)
    {
      xcpt_t irqhandler = g_gpio_irq_s[4].irqhandler;
      void  *cbarg    = g_gpio_irq_s[4].arg;

      ret = irqhandler(irq, context, cbarg);
    }

  return ret;
}

static int gd32_exti5_9_irqhandler(int irq, void *context, void *arg)
{
  int ret = OK;
  uint32_t pd_flag;
  uint8_t irq_pin;

  pd_flag = getreg32(GD32_EXTI_PD);

  for (irq_pin = 5; irq_pin <= 9; irq_pin++)
    {
      /* Check interrupt pend bit */

      uint32_t pd_bit = (1 << irq_pin);

      if ((pd_flag & pd_bit) != 0)
        {
          /* Clear interrupt pend bit */

          gd32_exti_interrupt_flag_clear(pd_bit);

          /* Handle the interrupt */

          if (g_gpio_irq_s[irq_pin].irqhandler != NULL)
            {
              xcpt_t irqhandler = g_gpio_irq_s[irq_pin].irqhandler;
              void  *cbarg    = g_gpio_irq_s[irq_pin].arg;

              ret = irqhandler(irq, context, cbarg);
            }
        }
    }

  return ret;
}

static int gd32_exti10_15_irqhandler(int irq, void *context, void *arg)
{
  int ret = OK;
  uint32_t pd_flag = getreg32(GD32_EXTI_PD);
  uint8_t irq_pin;

  for (irq_pin = 10; irq_pin <= 15; irq_pin++)
    {
      /* Check interrupt pend bit */

      uint32_t pd_bit = (1 << irq_pin);

      if ((pd_flag & pd_bit) != 0)
        {
          /* Clear interrupt pend bit */

          gd32_exti_interrupt_flag_clear(pd_bit);

          /* Handle the interrupt */

          if (g_gpio_irq_s[irq_pin].irqhandler != NULL)
            {
              xcpt_t irqhandler = g_gpio_irq_s[irq_pin].irqhandler;
              void  *cbarg    = g_gpio_irq_s[irq_pin].arg;

              ret = irqhandler(irq, context, cbarg);
            }
        }
    }

  return ret;
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_exti_gpioirq_init
 *
 * Description:
 *   Initialize the EXTI gpio irq.
 *
 * Input Parameters:
 *  - cfgset: GPIO pin
 *  - exti_mode: interrupt or event mode
 *  - trig_type: interrupt trigger type
 *  - irqnum: pointer to GPIO pin irq
 *
 ****************************************************************************/

int gd32_exti_gpioirq_init(uint32_t cfgset, uint8_t exti_mode,
                           uint8_t trig_type, uint8_t *irqnum)
{
  uint32_t exti_linex;
  uint8_t port;
  uint8_t pin;

  /* Verify that this hardware supports the select GPIO port */

  port = (cfgset & GPIO_CFG_PORT_MASK) >> GPIO_CFG_PORT_SHIFT;
  if (port >= GD32_NGPIO_PORTS)
    {
      return -EINVAL;
    }

  /* Get the exti line number */

  pin = (cfgset & GPIO_CFG_PIN_MASK) >> GPIO_CFG_PIN_SHIFT;

  if (irqnum != NULL)
    {
      *irqnum = pin;
    }

  exti_linex = (1 << pin);

  gd32_syscfg_clock_enable();

  /* Connect key EXTI line to key GPIO pin */

  gd32_syscfg_exti_line_config(port, pin);

  /* Configure the interrupt */

  gd32_exti_init(exti_linex, exti_mode, trig_type);

  gd32_exti_interrupt_flag_clear(exti_linex);

  return OK;
}

/****************************************************************************
 * Name: gd32_exti_gpio_irq_attach
 *
 * Description:
 *   Attach the EXTI gpio irq handler.
 *
 * Input Parameters:
 *  - irqpin: GPIO irq pin
 *  - irqhandler: irq handler
 *  - arg: Argument passed to the interrupt callback
 *
 ****************************************************************************/

int gd32_exti_gpio_irq_attach(uint8_t irqpin, xcpt_t irqhandler,
                              void *arg)
{
  switch (g_exti_gpio_irqs[irqpin])
    {
      case GD32_IRQ_EXTI0:
        irq_attach(GD32_IRQ_EXTI0, gd32_exti0_irqhandler, NULL);
        break;
      case GD32_IRQ_EXTI1:
        irq_attach(GD32_IRQ_EXTI1, gd32_exti1_irqhandler, NULL);
        break;
      case GD32_IRQ_EXTI2:
        irq_attach(GD32_IRQ_EXTI2, gd32_exti2_irqhandler, NULL);
        break;
      case GD32_IRQ_EXTI3:
        irq_attach(GD32_IRQ_EXTI3, gd32_exti3_irqhandler, NULL);
        break;
      case GD32_IRQ_EXTI4:
        irq_attach(GD32_IRQ_EXTI4, gd32_exti4_irqhandler, NULL);
        break;
      case GD32_IRQ_EXTI5_9:
        irq_attach(GD32_IRQ_EXTI5_9, gd32_exti5_9_irqhandler, NULL);
        break;
      case GD32_IRQ_EXTI10_15:
        irq_attach(GD32_IRQ_EXTI10_15, gd32_exti10_15_irqhandler, NULL);
        break;
      default:
        break;
    }

  /* Get the old GPIO pin irq handler and set the new GPIO pin irq handler */

  g_gpio_irq_s[irqpin].irqhandler = irqhandler;
  g_gpio_irq_s[irqpin].arg     = arg;

  return OK;
}

/****************************************************************************
 * Name: gd32_exti_init
 *
 * Description:
 *   Initialize the EXTI.
 *
 * Input Parameters:
 *  - linex: EXTI line number
 *  - exti_mode: interrupt or event mode
 *  - trig_type: interrupt trigger type
 *
 ****************************************************************************/

void gd32_exti_init(uint32_t linex, uint8_t exti_mode, uint8_t trig_type)
{
  modifyreg32(GD32_EXTI_INTEN, linex, 0);
  modifyreg32(GD32_EXTI_EVEN, linex, 0);
  modifyreg32(GD32_EXTI_RTEN, linex, 0);
  modifyreg32(GD32_EXTI_FTEN, linex, 0);

  /* set the EXTI mode and enable the interrupts or events from
   * EXTI line x
   */

  switch (exti_mode)
    {
      case EXTI_INTERRUPT:
        modifyreg32(GD32_EXTI_INTEN, 0, linex);
        break;
      case EXTI_EVENT:
        modifyreg32(GD32_EXTI_EVEN, 0, linex);
        break;
      default:
        break;
  }

  /* set the EXTI trigger type */

  switch (trig_type)
    {
      case EXTI_TRIG_RISING:
        modifyreg32(GD32_EXTI_RTEN, 0, linex);
        break;
      case EXTI_TRIG_FALLING:
        modifyreg32(GD32_EXTI_FTEN, 0, linex);
        break;
      case EXTI_TRIG_BOTH:
        modifyreg32(GD32_EXTI_RTEN, 0, linex);
        modifyreg32(GD32_EXTI_FTEN, 0, linex);
        break;
      case EXTI_TRIG_NONE:
      default:
        break;
    }
}

/****************************************************************************
 * Name: gd32_gpio_exti_linex_get
 *
 * Description:
 *   Get EXTI GPIO port and linex from GPIO pin.
 *
 ****************************************************************************/

int gd32_gpio_exti_linex_get(uint32_t cfgset, uint32_t *linex)
{
  uint8_t port;
  uint8_t pin;

  /* Verify that this hardware supports the select GPIO port */

  port = (cfgset & GPIO_CFG_PORT_MASK) >> GPIO_CFG_PORT_SHIFT;
  if (port >= GD32_NGPIO_PORTS)
    {
      return -EINVAL;
    }

  /* Get the pin number */

  pin = (cfgset & GPIO_CFG_PIN_MASK) >> GPIO_CFG_PIN_SHIFT;

  *linex = (1 << pin);

  return OK;
}

/****************************************************************************
 * Name: gd32_gpio_exti_irqnum_get
 *
 * Description:
 *   Get EXTI GPIO irq number from GPIO pin.
 *
 ****************************************************************************/

int gd32_gpio_exti_irqnum_get(uint32_t cfgset, uint8_t *irqnum)
{
  uint8_t port;
  uint8_t pin;

  /* Verify that this hardware supports the select GPIO port */

  port = (cfgset & GPIO_CFG_PORT_MASK) >> GPIO_CFG_PORT_SHIFT;
  if (port >= GD32_NGPIO_PORTS)
    {
      return -EINVAL;
    }

  /* Get the pin number */

  pin = (cfgset & GPIO_CFG_PIN_MASK) >> GPIO_CFG_PIN_SHIFT;

  *irqnum = g_exti_gpio_irqs[pin];

  return OK;
}

/****************************************************************************
 * Name: gd32_exti_interrupt_enable
 *
 * Description:
 *   Enable the interrupts from EXTI line x.
 *
 * Input Parameters:
 *  - linex: EXTI line number
 *
 ****************************************************************************/

void gd32_exti_interrupt_enable(uint32_t linex)
{
  modifyreg32(GD32_EXTI_INTEN, 0, linex);
}

/****************************************************************************
 * Name: gd32_exti_interrupt_disable
 *
 * Description:
 *   Disable the interrupts from EXTI line x.
 *
 * Input Parameters:
 *  - linex: EXTI line number
 *
 ****************************************************************************/

void gd32_exti_interrupt_disable(uint32_t linex)
{
  modifyreg32(GD32_EXTI_INTEN, linex, 0);
}

/****************************************************************************
 * Name: gd32_exti_event_enable
 *
 * Description:
 *   Enable the events from EXTI line x.
 *
 * Input Parameters:
 *  - linex: EXTI line number
 *
 ****************************************************************************/

void gd32_exti_event_enable(uint32_t linex)
{
  modifyreg32(GD32_EXTI_EVEN, 0, linex);
}

/****************************************************************************
 * Name: gd32_exti_event_disable
 *
 * Description:
 *   Disable the events from EXTI line x.
 *
 * Input Parameters:
 *  - linex: EXTI line number
 *
 ****************************************************************************/

void gd32_exti_event_disable(uint32_t linex)
{
  modifyreg32(GD32_EXTI_EVEN, linex, 0);
}

/****************************************************************************
 * Name: gd32_exti_software_interrupt_enable
 *
 * Description:
 *   Enable EXTI software interrupt event.
 *
 * Input Parameters:
 *  - linex: EXTI line number
 *
 ****************************************************************************/

void gd32_exti_software_interrupt_enable(uint32_t linex)
{
  modifyreg32(GD32_EXTI_SWIEV, 0, linex);
}

/****************************************************************************
 * Name: gd32_exti_software_interrupt_disable
 *
 * Description:
 *   Disable EXTI software interrupt event.
 *
 * Input Parameters:
 *  - linex: EXTI line number
 *
 ****************************************************************************/

void gd32_exti_software_interrupt_disable(uint32_t linex)
{
  modifyreg32(GD32_EXTI_SWIEV, linex, 0);
}

/****************************************************************************
 * Name: gd32_exti_interrupt_flag_get
 *
 * Description:
 *   Get EXTI lines flag when the interrupt flag is set.
 *
 * Input Parameters:
 *  - linex: EXTI line number
 *
 * Returned Value:
 *   status of flag (false or true)
 *
 ****************************************************************************/

bool gd32_exti_interrupt_flag_get(uint32_t linex)
{
  uint32_t regval0;
  uint32_t regval1;

  regval0 = (getreg32(GD32_EXTI_PD) & linex);
  regval1 = (getreg32(GD32_EXTI_INTEN) & linex);

  if ((regval0 & regval1))
    {
      return true;
    }
  else
    {
      return false;
    }
}

/****************************************************************************
 * Name: gd32_exti_interrupt_flag_clear
 *
 * Description:
 *   Clear EXTI lines pending flag.
 *
 * Input Parameters:
 *  - linex: EXTI line number
 *
 ****************************************************************************/

void gd32_exti_interrupt_flag_clear(uint32_t linex)
{
  putreg32(linex, GD32_EXTI_PD);
}

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
