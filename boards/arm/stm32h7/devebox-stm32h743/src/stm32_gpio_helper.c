/****************************************************************************
 * boards/arm/stm32h7/devebox-stm32h743/src/stm32_gpio_helper.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <nuttx/ioexpander/gpio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "stm32_gpio.h"
#include "stm32_gpio_helper.h"
#include "arm_internal.h"
#include "hardware/stm32_exti.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct gpio_helper_dev_s
{
  struct gpio_dev_s dev;
  uint32_t pin;
  pin_interrupt_t callback;
  uint8_t exti_line;
  bool irq_enabled;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct gpio_helper_dev_s *g_gpio_helper_exti_dev[16];

/****************************************************************************
 * Private Functions - GPIO Operations
 ****************************************************************************/

static int gpio_helper_read(FAR struct gpio_dev_s *dev, FAR bool *value)
{
  FAR struct gpio_helper_dev_s *priv = (FAR struct gpio_helper_dev_s *)dev;
  *value = stm32_gpioread(priv->pin);
  return OK;
}

static int gpio_helper_write(FAR struct gpio_dev_s *dev, bool value)
{
  FAR struct gpio_helper_dev_s *priv = (FAR struct gpio_helper_dev_s *)dev;
  stm32_gpiowrite(priv->pin, value);
  return OK;
}

static int gpio_helper_attach(FAR struct gpio_dev_s *dev,
                              pin_interrupt_t callback)
{
  FAR struct gpio_helper_dev_s *priv = (FAR struct gpio_helper_dev_s *)dev;
  priv->callback = callback;
  return OK;
}

static int gpio_helper_enable(FAR struct gpio_dev_s *dev, bool enable)
{
  FAR struct gpio_helper_dev_s *priv = (FAR struct gpio_helper_dev_s *)dev;
  int irq = STM32_IRQ_EXTI0 + priv->exti_line;

  if (enable)
    {
      up_enable_irq(irq);
      priv->irq_enabled = true;
    }
  else
    {
      up_disable_irq(irq);
      priv->irq_enabled = false;
    }
  return OK;
}

static int gpio_helper_setpintype(FAR struct gpio_dev_s *dev,
                                  enum gpio_pintype_e pintype)
{
  return -ENOSYS;
}

static int gpio_helper_setdebounce(FAR struct gpio_dev_s *dev,
                                   unsigned long duration)
{
  return -ENOSYS;
}

static int gpio_helper_setmask(FAR struct gpio_dev_s *dev, bool enable)
{
  FAR struct gpio_helper_dev_s *priv = (FAR struct gpio_helper_dev_s *)dev;
  if (priv->exti_line >= 16)
    return -EINVAL;

  uint32_t bit = 1 << priv->exti_line;
  if (enable)
    modifyreg32(STM32_EXTI_CPUIMR1, 0, bit);   /* mask interrupt */
  else
    modifyreg32(STM32_EXTI_CPUIMR1, bit, 0);   /* unmask interrupt */
  return OK;
}

/****************************************************************************
 * Private Data - Operations Table
 ****************************************************************************/

static const struct gpio_operations_s g_gpio_helper_ops =
{
  .go_read        = gpio_helper_read,
  .go_write       = gpio_helper_write,
  .go_attach      = gpio_helper_attach,
  .go_enable      = gpio_helper_enable,
  .go_setpintype  = gpio_helper_setpintype,
  .go_setdebounce = gpio_helper_setdebounce,
  .go_setmask     = gpio_helper_setmask,
};

/****************************************************************************
 * Private Functions - Interrupt Handler
 ****************************************************************************/

static int gpio_helper_exti_handler(int irq, void *context, FAR void *arg)
{
  int line = irq - STM32_IRQ_EXTI0;
  if (line < 0 || line >= 16)
    return OK;

  /* Clear pending bit */

  putreg32(1 << line, STM32_EXTI_CPUPR1);

  struct gpio_helper_dev_s *priv = g_gpio_helper_exti_dev[line];
  if (priv && priv->callback)
    priv->callback(&priv->dev, 0);

  return OK;
}

/****************************************************************************
 * Private Functions - Common Registration
 ****************************************************************************/

static int gpio_helper_reg_common(uint32_t pin, const char *devpath,
                                       enum gpio_pintype_e pintype,
                                       bool with_irq)
{
  struct gpio_helper_dev_s *priv;
  int ret;
  unsigned int pin_num = (pin & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

  if (with_irq)
    {
      /* Enable EXTI for this pin (stm32_configgpio will set SYSCFG) */

      pin |= GPIO_EXTI;
    }

  stm32_configgpio(pin);

  priv = malloc(sizeof(*priv));
  if (!priv)
    return -ENOMEM;

  memset(priv, 0, sizeof(*priv));
  priv->dev.gp_pintype = pintype;
  priv->dev.gp_ops     = &g_gpio_helper_ops;
  priv->pin            = pin;
  priv->exti_line      = pin_num;
  priv->irq_enabled    = false;

  ret = gpio_pin_register_byname(&priv->dev, devpath);
  if (ret < 0)
    {
      free(priv);
      return ret;
    }

  if (with_irq)
    {
      int irq = STM32_IRQ_EXTI0 + pin_num;
      g_gpio_helper_exti_dev[pin_num] = priv;

      ret = irq_attach(irq, gpio_helper_exti_handler, NULL);
      if (ret < 0)
        {
          gpio_pin_unregister_byname(&priv->dev, devpath);
          free(priv);
          return ret;
        }

      /* Disable interrupt by default – user enables via ioctl */

      up_disable_irq(irq);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int gpio_helper_reg_output(uint32_t pin, const char *devpath)
{
  return gpio_helper_reg_common(pin, devpath, GPIO_OUTPUT_PIN, false);
}

int gpio_helper_reg_input(uint32_t pin, const char *devpath,
                               enum gpio_helper_pull_e pull)
{
  uint32_t cfg = GPIO_INPUT;
  if (pull == GPIO_HELPER_PULL_UP)
    cfg |= GPIO_PULLUP;
  else if (pull == GPIO_HELPER_PULL_DOWN)
    cfg |= GPIO_PULLDOWN;

  pin = (pin & ~(GPIO_MODE_MASK | GPIO_PUPD_MASK)) | cfg;
  return gpio_helper_reg_common(pin, devpath, GPIO_INPUT_PIN, false);
}

int gpio_helper_reg_input_interrupt(uint32_t pin, const char *devpath,
                                   enum gpio_helper_pull_e pull,
                                   enum gpio_helper_mode_e mode)
{
  uint32_t cfg = GPIO_INPUT;
  if (pull == GPIO_HELPER_PULL_UP)
    cfg |= GPIO_PULLUP;
  else if (pull == GPIO_HELPER_PULL_DOWN)
    cfg |= GPIO_PULLDOWN;

  /* Configure pin as input with EXTI (trigger type set later) */

  pin = (pin & ~(GPIO_MODE_MASK | GPIO_PUPD_MASK)) | cfg | GPIO_EXTI;

  int ret = gpio_helper_reg_common(pin, devpath, GPIO_INTERRUPT_PIN, true);
  if (ret < 0)
    {
      return ret;
    }

  /* Now configure EXTI trigger via registers */

  unsigned int line = (pin & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  uint32_t bit = 1 << line;

  /* Clear previous trigger settings */

  modifyreg32(STM32_EXTI_RTSR1, bit, 0);
  modifyreg32(STM32_EXTI_FTSR1, bit, 0);

  switch (mode)
    {
      case GPIO_HELPER_INTERRUPT_RISING:
        modifyreg32(STM32_EXTI_RTSR1, 0, bit);
        break;
      case GPIO_HELPER_INTERRUPT_FALLING:
        modifyreg32(STM32_EXTI_FTSR1, 0, bit);
        break;
      case GPIO_HELPER_INTERRUPT_BOTH:
        modifyreg32(STM32_EXTI_RTSR1, 0, bit);
        modifyreg32(STM32_EXTI_FTSR1, 0, bit);
        break;
      case GPIO_HELPER_INTERRUPT_HIGH:

        /* For high-level trigger, set RTSR */

        modifyreg32(STM32_EXTI_RTSR1, 0, bit);
        break;
      case GPIO_HELPER_INTERRUPT_LOW:

        /* For low-level trigger, set FTSR */

        modifyreg32(STM32_EXTI_FTSR1, 0, bit);
        break;
      default:
        return -EINVAL;
    }

  /* Unmask in EXTI (but IRQ still disabled; enable via ioctl) */

  modifyreg32(STM32_EXTI_CPUIMR1, 0, bit);

  return OK;
}
