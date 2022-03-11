/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_gpio.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include "chip.h"
#include "arm_internal.h"
#include "cxd56_pinconfig.h"
#include "cxd56_gpio.h"
#include "hardware/cxd5602_topreg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GPIO register Definitions */

#define GPIO_OUTPUT_EN_SHIFT    (16)
#define GPIO_OUTPUT_EN_MASK     (1u << GPIO_OUTPUT_EN_SHIFT)
#define GPIO_OUTPUT_ENABLE      (0u << GPIO_OUTPUT_EN_SHIFT)
#define GPIO_OUTPUT_DISABLE     (1u << GPIO_OUTPUT_EN_SHIFT)
#define GPIO_OUTPUT_ENABLED(v)  (((v) & GPIO_OUTPUT_EN_MASK) == GPIO_OUTPUT_ENABLE)
#define GPIO_OUTPUT_SHIFT       (8)
#define GPIO_OUTPUT_MASK        (1u << GPIO_OUTPUT_SHIFT)
#define GPIO_OUTPUT_HIGH        (1u << GPIO_OUTPUT_SHIFT)
#define GPIO_OUTPUT_LOW         (0u << GPIO_OUTPUT_SHIFT)
#define GPIO_INPUT_SHIFT        (0)
#define GPIO_INPUT_MASK         (1u << GPIO_INPUT_SHIFT)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint32_t get_gpio_regaddr(uint32_t pin)
{
  uint32_t base;

  base = (pin < PIN_IS_CLK) ? 1 : 7;

  return CXD56_TOPREG_GP_I2C4_BCK + ((pin - base) * 4);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cxd56_gpio_config
 *
 * Description:
 *   Configure a GPIO which input is enabled or not.
 *   Output is enabled when cxd56_gpio_write() is called.
 *
 * Returned Value:
 *   OK on success; A negated errno value on failure.
 *
 ****************************************************************************/

int cxd56_gpio_config(uint32_t pin, bool input_enable)
{
  int ret = 0;
  uint32_t pinconf;
  uint32_t regaddr;
  uint32_t ioreg;
  uint32_t ioval;

  DEBUGASSERT((PIN_I2C4_BCK <= pin) && (pin <= PIN_USB_VBUSINT));
  DEBUGASSERT((pin <= PIN_GNSS_1PPS_OUT) || (PIN_SPI0_CS_X <= pin));
  DEBUGASSERT((pin <= PIN_HIF_GPIO0) || (PIN_SEN_IRQ_IN <= pin));
  DEBUGASSERT((pin <= PIN_PWM3) || (PIN_IS_CLK <= pin));

  ioreg = CXD56_TOPREG_IO_RTC_CLK_IN + (pin * 4);
  ioval = getreg32(ioreg);

  if (input_enable)
    {
      pinconf = PINCONF_SET(pin, PINCONF_MODE0, PINCONF_INPUT_ENABLE,
                            ioval & PINCONF_DRIVE_MASK,
                            ioval & PINCONF_PULL_MASK);
    }
  else
    {
      pinconf = PINCONF_SET(pin, PINCONF_MODE0, PINCONF_INPUT_DISABLE,
                            ioval & PINCONF_DRIVE_MASK,
                            ioval & PINCONF_PULL_MASK);
    }

  ret = cxd56_pin_config(pinconf);

  /* output disabled */

  regaddr = get_gpio_regaddr(pin);
  putreg32(GPIO_OUTPUT_DISABLE, regaddr);

  return ret;
}

/****************************************************************************
 * Name: cxd56_gpio_write
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void cxd56_gpio_write(uint32_t pin, bool value)
{
  uint32_t regaddr;
  uint32_t regval;

  DEBUGASSERT((PIN_I2C4_BCK <= pin) && (pin <= PIN_USB_VBUSINT));
  DEBUGASSERT((pin <= PIN_GNSS_1PPS_OUT) || (PIN_SPI0_CS_X <= pin));
  DEBUGASSERT((pin <= PIN_HIF_GPIO0) || (PIN_SEN_IRQ_IN <= pin));
  DEBUGASSERT((pin <= PIN_PWM3) || (PIN_IS_CLK <= pin));

  regaddr = get_gpio_regaddr(pin);

  if (value)
    {
      regval = GPIO_OUTPUT_ENABLE | GPIO_OUTPUT_HIGH;
    }
  else
    {
      regval = GPIO_OUTPUT_ENABLE | GPIO_OUTPUT_LOW;
    }

  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: cxd56_gpio_write_hiz
 *
 * Description:
 *   Write HiZ to the selected opendrain GPIO pin
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void cxd56_gpio_write_hiz(uint32_t pin)
{
  uint32_t regaddr;
  uint32_t regval;

  DEBUGASSERT((PIN_I2C4_BCK <= pin) && (pin <= PIN_USB_VBUSINT));
  DEBUGASSERT((pin <= PIN_GNSS_1PPS_OUT) || (PIN_SPI0_CS_X <= pin));
  DEBUGASSERT((pin <= PIN_HIF_GPIO0) || (PIN_SEN_IRQ_IN <= pin));
  DEBUGASSERT((pin <= PIN_PWM3) || (PIN_IS_CLK <= pin));

  regaddr = get_gpio_regaddr(pin);

  regval = GPIO_OUTPUT_DISABLE;

  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: cxd56_gpio_read
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 * Returned Value:
 *   The boolean state of the input pin
 *
 ****************************************************************************/

bool cxd56_gpio_read(uint32_t pin)
{
  uint32_t regaddr;
  uint32_t regval;
  uint32_t shift;
  uint32_t ioreg;
  uint32_t ioval;

  DEBUGASSERT((PIN_I2C4_BCK <= pin) && (pin <= PIN_USB_VBUSINT));
  DEBUGASSERT((pin <= PIN_GNSS_1PPS_OUT) || (PIN_SPI0_CS_X <= pin));
  DEBUGASSERT((pin <= PIN_HIF_GPIO0) || (PIN_SEN_IRQ_IN <= pin));
  DEBUGASSERT((pin <= PIN_PWM3) || (PIN_IS_CLK <= pin));

  regaddr = get_gpio_regaddr(pin);
  regval = getreg32(regaddr);

  ioreg = CXD56_TOPREG_IO_RTC_CLK_IN + (pin * 4);
  ioval = getreg32(ioreg);

  if (PINCONF_INPUT_ENABLED(ioval))
    {
      shift = GPIO_INPUT_SHIFT;
    }
  else if (GPIO_OUTPUT_ENABLED(regval))
    {
      shift = GPIO_OUTPUT_SHIFT;
    }
  else
    {
      shift = GPIO_INPUT_SHIFT;
    }

  return ((regval & (1 << shift)) != 0);
}

/****************************************************************************
 * Name: cxd56_gpio_status
 *
 * Description:
 *   Get a gpio status which input/output is enabled or not.
 *
 * Returned Value:
 *   OK on success; A negated errno value on failure.
 *
 ****************************************************************************/

int cxd56_gpio_status(uint32_t pin, cxd56_gpio_status_t *stat)
{
  uint32_t regaddr;
  uint32_t regval;
  uint32_t ioreg;
  uint32_t ioval;

  if ((pin < PIN_I2C4_BCK) ||
      ((PIN_GNSS_1PPS_OUT < pin) && (pin < PIN_SPI0_CS_X)) ||
      ((PIN_HIF_GPIO0 < pin) && (pin < PIN_SEN_IRQ_IN)) ||
      ((PIN_PWM3 < pin) && (pin < PIN_IS_CLK)) ||
      (PIN_USB_VBUSINT < pin))
    {
      return -EINVAL;
    }

  ioreg = CXD56_TOPREG_IO_RTC_CLK_IN + (pin * 4);
  ioval = getreg32(ioreg);

  regaddr = get_gpio_regaddr(pin);
  regval = getreg32(regaddr);

  stat->input_en  = PINCONF_INPUT_ENABLED(ioval);
  stat->output_en = GPIO_OUTPUT_ENABLED(regval);

  return 0;
}
