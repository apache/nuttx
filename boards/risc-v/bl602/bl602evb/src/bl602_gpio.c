/****************************************************************************
 * boards/risc-v/bl602/bl602evb/src/bl602_gpio.c
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
#include <stdio.h>
#include <stdbool.h>
#include <assert.h>
#include <debug.h>
#include <nuttx/arch.h>
#include <nuttx/ioexpander/gpio.h>
#include <arch/board/board.h>
#include "riscv_internal.h"
#include "bl602_gpio.h"

#if defined(CONFIG_DEV_GPIO) && !defined(CONFIG_GPIO_LOWER_HALF)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BOARD_GPIO_PIN(mode, pupd, func, pin) (mode | pupd | func | pin)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct bl602_gpio_dev_s
{
  struct gpio_dev_s gpio;
  uint8_t           id;
};

struct bl602_gpint_dev_s
{
  struct bl602_gpio_dev_s bl602gpio;
  pin_interrupt_t         callback;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int gpin_read(struct gpio_dev_s *dev, bool *value);
static int gpout_read(struct gpio_dev_s *dev, bool *value);
static int gpout_write(struct gpio_dev_s *dev, bool value);
static int gpint_read(struct gpio_dev_s *dev, bool *value);
static int gpint_attach(struct gpio_dev_s *dev,
                        pin_interrupt_t callback);
static int gpint_enable(struct gpio_dev_s *dev, bool enable);
static int gpio_setpintype(struct gpio_dev_s *dev,
                           enum gpio_pintype_e    gp_pintype);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct gpio_operations_s gpin_ops =
{
  .go_read       = gpin_read,
  .go_write      = NULL,
  .go_attach     = NULL,
  .go_enable     = NULL,
  .go_setpintype = gpio_setpintype,
};

static const struct gpio_operations_s gpout_ops =
{
  .go_read       = gpout_read,
  .go_write      = gpout_write,
  .go_attach     = NULL,
  .go_enable     = NULL,
  .go_setpintype = gpio_setpintype,
};

static const struct gpio_operations_s gpint_ops =
{
  .go_read       = gpint_read,
  .go_write      = NULL,
  .go_attach     = gpint_attach,
  .go_enable     = gpint_enable,
  .go_setpintype = gpio_setpintype,
};

#if BOARD_NGPIOIN > 0
/* This array maps the GPIO pins used as INPUT */

static const uint32_t g_gpioinputs[BOARD_NGPIOIN] =
{
  BOARD_GPIO_IN1,
};

static struct bl602_gpio_dev_s g_gpin[BOARD_NGPIOIN];
#endif

#if BOARD_NGPIOOUT
/* This array maps the GPIO pins used as OUTPUT */

static const uint32_t g_gpiooutputs[BOARD_NGPIOOUT] =
{
  BOARD_GPIO_OUT1,
};

static struct bl602_gpio_dev_s g_gpout[BOARD_NGPIOOUT];
#endif

#if BOARD_NGPIOINT > 0
/* This array maps the GPIO pins used as INTERRUPT INPUTS */

static const uint32_t g_gpiointinputs[BOARD_NGPIOINT] =
{
  BOARD_GPIO_INT1,
};

static struct bl602_gpint_dev_s g_gpint[BOARD_NGPIOINT];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bl602_gpio_intmask
 *
 * Description:
 *   intmask a gpio pin.
 *
 ****************************************************************************/

static void bl602_gpio_intmask(int pin, int intmask)
{
  uint32_t tmp_val;

  if (pin < 28)
    {
      tmp_val = getreg32(BL602_GPIO_INT_MASK1);
      if (intmask == 1)
        {
          tmp_val |= (1 << pin);
        }
      else
        {
          tmp_val &= ~(1 << pin);
        }

      putreg32(tmp_val, BL602_GPIO_INT_MASK1);
    }
}

/****************************************************************************
 * Name: bl602_gpio_set_intmod
 *
 * Description:
 *   set gpio intmod.
 *
 ****************************************************************************/

static void bl602_gpio_set_intmod(uint8_t gpio_pin,
              uint8_t int_ctlmod, uint8_t int_trgmod)
{
  uint32_t tmp_val;

  if (gpio_pin < GPIO_PIN10)
    {
      /* GPIO0 ~ GPIO9 */

      tmp_val = gpio_pin;
      modifyreg32(BL602_GPIO_INT_MODE_SET1,
                  0x7 << (3 * tmp_val),
                  ((int_ctlmod << 2) | int_trgmod) << (3 * tmp_val));
    }
  else if (gpio_pin < GPIO_PIN20)
    {
      /* GPIO10 ~ GPIO19 */

      tmp_val = gpio_pin - GPIO_PIN10;
      modifyreg32(BL602_GPIO_INT_MODE_SET2,
                  0x7 << (3 * tmp_val),
                  ((int_ctlmod << 2) | int_trgmod) << (3 * tmp_val));
    }
  else
    {
      /* GPIO20 ~ GPIO29 */

      tmp_val = gpio_pin - GPIO_PIN20;
      modifyreg32(BL602_GPIO_INT_MODE_SET3,
                  0x7 << (3 * tmp_val),
                  ((int_ctlmod << 2) | int_trgmod) << (3 * tmp_val));
    }
}

/****************************************************************************
 * Name: bl602_gpio_get_intstatus
 *
 * Description:
 *   get gpio intstatus.
 *
 ****************************************************************************/

static int bl602_gpio_get_intstatus(uint8_t gpio_pin)
{
  uint32_t tmp_val = 0;

  if (gpio_pin < 28)
    {
      /* GPIO0 ~ GPIO27 */

      tmp_val = getreg32(BL602_GPIO_INT_STAT1);
    }

  return (tmp_val & (1 << gpio_pin)) ? 1 : 0;
}

/****************************************************************************
 * Name: bl602_gpio_intclear
 *
 * Description:
 *   clear gpio int.
 *
 ****************************************************************************/

static void bl602_gpio_intclear(uint8_t gpio_pin, uint8_t int_clear)
{
  if (gpio_pin < 28)
    {
      /* GPIO0 ~ GPIO27 */

      modifyreg32(BL602_GPIO_INT_CLR1,
                  int_clear ? 0 : (1 << gpio_pin),
                  int_clear ? (1 << gpio_pin) : 0);
    }
}

/****************************************************************************
 * Name: bl602_gpio_interrupt
 *
 * Description:
 *   gpio interrupt.
 *
 ****************************************************************************/

static int bl602_gpio_interrupt(int irq, void *context, void *arg)
{
  struct bl602_gpint_dev_s *bl602xgpint =
    (struct bl602_gpint_dev_s *)arg;

  uint32_t time_out = 0;
  uint8_t gpio_pin;

  DEBUGASSERT(bl602xgpint != NULL && bl602xgpint->callback != NULL);
  gpioinfo("Interrupt! callback=%p\n", bl602xgpint->callback);

  gpio_pin = (g_gpiointinputs[bl602xgpint->bl602gpio.id] & GPIO_PIN_MASK) >>
    GPIO_PIN_SHIFT;

  if (1 == bl602_gpio_get_intstatus(gpio_pin))
    {
      bl602_gpio_intclear(gpio_pin, 1);

      /* timeout check */

      time_out = 32;
      do
        {
          time_out--;
        }
      while ((1 == bl602_gpio_get_intstatus(gpio_pin)) && time_out);
      if (!time_out)
        {
          gpiowarn("WARNING: Clear GPIO interrupt status fail.\n");
        }

      /* if time_out==0, GPIO interrupt status not cleared */

      bl602_gpio_intclear(gpio_pin, 0);
    }

  bl602xgpint->callback(&bl602xgpint->bl602gpio.gpio,
                        gpio_pin);

  return OK;
}

/****************************************************************************
 * Name: gpio_setpintype
 *
 * Description:
 *   set gpio pintype.
 *
 ****************************************************************************/

static int gpio_setpintype(struct gpio_dev_s *dev,
                           enum gpio_pintype_e    gp_pintype)
{
  struct bl602_gpint_dev_s *bl602xgpint =
    (struct bl602_gpint_dev_s *)dev;
  uint8_t gpio_pin;
  uint8_t pintype = bl602xgpint->bl602gpio.gpio.gp_pintype;

  DEBUGASSERT(bl602xgpint != NULL);
  gpioinfo("setpintype...\n");

  if (pintype <= GPIO_INPUT_PIN_PULLDOWN)
    {
      gpio_pin =
        (g_gpioinputs[bl602xgpint->bl602gpio.id] & GPIO_PIN_MASK) >>
        GPIO_PIN_SHIFT;
    }
  else if (pintype <= GPIO_OUTPUT_PIN_OPENDRAIN)
    {
      gpio_pin =
        (g_gpiooutputs[bl602xgpint->bl602gpio.id] & GPIO_PIN_MASK) >>
        GPIO_PIN_SHIFT;
    }
  else if (pintype < GPIO_NPINTYPES)
    {
      gpio_pin =
        (g_gpiointinputs[bl602xgpint->bl602gpio.id] & GPIO_PIN_MASK) >>
        GPIO_PIN_SHIFT;
    }
  else
    {
      gpioerr("pintype error\n");
      return -1;
    }

  switch (gp_pintype)
    {
    case GPIO_INPUT_PIN:
      bl602_configgpio(
      BOARD_GPIO_PIN(GPIO_INPUT, GPIO_FLOAT, GPIO_FUNC_SWGPIO, gpio_pin));
      break;
    case GPIO_INPUT_PIN_PULLUP:
      bl602_configgpio(
      BOARD_GPIO_PIN(GPIO_INPUT, GPIO_PULLUP, GPIO_FUNC_SWGPIO, gpio_pin));
      break;
    case GPIO_INPUT_PIN_PULLDOWN:
      bl602_configgpio(
      BOARD_GPIO_PIN(GPIO_INPUT, GPIO_PULLDOWN, GPIO_FUNC_SWGPIO, gpio_pin));
      break;
    case GPIO_OUTPUT_PIN:
      bl602_configgpio(
      BOARD_GPIO_PIN(GPIO_OUTPUT, GPIO_PULLUP, GPIO_FUNC_SWGPIO, gpio_pin));
      break;
    case GPIO_OUTPUT_PIN_OPENDRAIN:
      bl602_configgpio(
      BOARD_GPIO_PIN(GPIO_OUTPUT, GPIO_FLOAT, GPIO_FUNC_SWGPIO, gpio_pin));
      break;
    case GPIO_INTERRUPT_RISING_PIN:
      bl602_gpio_set_intmod(gpio_pin, 1, GLB_GPIO_INT_TRIG_POS_PULSE);
      bl602_configgpio(
      BOARD_GPIO_PIN(GPIO_INPUT, GPIO_PULLUP, GPIO_FUNC_SWGPIO, gpio_pin));
      break;
    case GPIO_INTERRUPT_FALLING_PIN:
      bl602_gpio_set_intmod(gpio_pin, 1, GLB_GPIO_INT_TRIG_NEG_PULSE);
      bl602_configgpio(
      BOARD_GPIO_PIN(GPIO_INPUT, GPIO_PULLUP, GPIO_FUNC_SWGPIO, gpio_pin));
      break;
    default:
      break;
    }

  return 0;
}

/****************************************************************************
 * Name: gpin_read
 *
 * Description:
 *   read gpio input.
 *
 ****************************************************************************/

static int gpin_read(struct gpio_dev_s *dev, bool *value)
{
  struct bl602_gpio_dev_s *bl602xgpio =
    (struct bl602_gpio_dev_s *)dev;

  DEBUGASSERT(bl602xgpio != NULL && value != NULL);
  gpioinfo("Reading...\n");
  *value            = bl602_gpioread(g_gpioinputs[bl602xgpio->id]);

  return OK;
}

/****************************************************************************
 * Name: gpout_read
 *
 * Description:
 *   read gpio output.
 *
 ****************************************************************************/

static int gpout_read(struct gpio_dev_s *dev, bool *value)
{
  struct bl602_gpio_dev_s *bl602xgpio =
    (struct bl602_gpio_dev_s *)dev;

  DEBUGASSERT(bl602xgpio != NULL && value != NULL);
  DEBUGASSERT(bl602xgpio->id < BOARD_NGPIOOUT);
  gpioinfo("Reading...\n");

  uint8_t gpio_pin = (g_gpiooutputs[bl602xgpio->id] & GPIO_PIN_MASK) >>
    GPIO_PIN_SHIFT;

  *value = (getreg32(BL602_GPIO_CFGCTL32) & (1 << gpio_pin) ? 1 : 0);

  return OK;
}

/****************************************************************************
 * Name: gpout_write
 *
 * Description:
 *   write gpio.
 *
 ****************************************************************************/

static int gpout_write(struct gpio_dev_s *dev, bool value)
{
  struct bl602_gpio_dev_s *bl602xgpio =
    (struct bl602_gpio_dev_s *)dev;

  DEBUGASSERT(bl602xgpio != NULL);
  DEBUGASSERT(bl602xgpio->id < BOARD_NGPIOOUT);
  gpioinfo("Writing %d\n", (int)value);

  bl602_gpiowrite(g_gpiooutputs[bl602xgpio->id], value);

  return OK;
}

/****************************************************************************
 * Name: gpint_read
 *
 * Description:
 *   read gpio.
 *
 ****************************************************************************/

static int gpint_read(struct gpio_dev_s *dev, bool *value)
{
  struct bl602_gpint_dev_s *bl602xgpint =
    (struct bl602_gpint_dev_s *)dev;

  DEBUGASSERT(bl602xgpint != NULL && value != NULL);
  DEBUGASSERT(bl602xgpint->bl602gpio.id < BOARD_NGPIOINT);
  gpioinfo("Reading int pin...\n");

  *value = bl602_gpioread(g_gpiointinputs[bl602xgpint->bl602gpio.id]);

  return OK;
}

/****************************************************************************
 * Name: gpint_attach
 *
 * Description:
 *   gpio attach.
 *
 ****************************************************************************/

static int gpint_attach(struct gpio_dev_s *dev, pin_interrupt_t callback)
{
  struct bl602_gpint_dev_s *bl602xgpint =
    (struct bl602_gpint_dev_s *)dev;

  uint8_t gpio_pin =
    (g_gpiointinputs[bl602xgpint->bl602gpio.id] & GPIO_PIN_MASK) >>
    GPIO_PIN_SHIFT;
  gpioinfo("Attaching the callback\n");

  /* Make sure the interrupt is disabled */

  bl602xgpint->callback = callback;
  bl602_gpio_intmask(gpio_pin, 1);

  irq_attach(BL602_IRQ_GPIO_INT0, bl602_gpio_interrupt, dev);
  bl602_gpio_intmask(gpio_pin, 0);

  gpioinfo("Attach %p\n", callback);
  return OK;
}

/****************************************************************************
 * Name: gpint_enable
 *
 * Description:
 *   gpint enable.
 *
 ****************************************************************************/

static int gpint_enable(struct gpio_dev_s *dev, bool enable)
{
  struct bl602_gpint_dev_s *bl602xgpint =
    (struct bl602_gpint_dev_s *)dev;

  if (enable)
    {
      if (bl602xgpint->callback != NULL)
        {
          gpioinfo("Enabling the interrupt\n");
          up_enable_irq(BL602_IRQ_GPIO_INT0);
        }
    }
  else
    {
      gpioinfo("Disable the interrupt\n");
      up_disable_irq(BL602_IRQ_GPIO_INT0);
    }

  return OK;
}

/****************************************************************************
 * Name: bl602_gpio_initialize
 *
 * Description:
 *   Initialize GPIO drivers for use with /apps/examples/gpio
 *
 ****************************************************************************/

int bl602_gpio_initialize(void)
{
  int i;
  int pincount = 0;

#if BOARD_NGPIOIN > 0
  for (i = 0; i < BOARD_NGPIOIN; i++)
    {
      /* Setup and register the GPIO pin */

      g_gpin[i].gpio.gp_pintype = GPIO_INPUT_PIN;
      g_gpin[i].gpio.gp_ops     = &gpin_ops;
      g_gpin[i].id              = i;
      gpio_pin_register(&g_gpin[i].gpio, pincount);

      /* Configure the pin that will be used as input */

      bl602_configgpio(g_gpioinputs[i]);

      pincount++;
    }
#endif

#if BOARD_NGPIOOUT > 0
  for (i = 0; i < BOARD_NGPIOOUT; i++)
    {
      /* Setup and register the GPIO pin */

      g_gpout[i].gpio.gp_pintype = GPIO_OUTPUT_PIN;
      g_gpout[i].gpio.gp_ops     = &gpout_ops;
      g_gpout[i].id              = i;
      gpio_pin_register(&g_gpout[i].gpio, pincount);

      /* Configure the pin that will be used as output */

      bl602_configgpio(g_gpiooutputs[i]);

      pincount++;
    }
#endif

#if BOARD_NGPIOINT > 0
  for (i = 0; i < BOARD_NGPIOINT; i++)
    {
      /* Setup and register the GPIO pin */

      g_gpint[i].bl602gpio.gpio.gp_pintype = GPIO_INTERRUPT_PIN;
      g_gpint[i].bl602gpio.gpio.gp_ops     = &gpint_ops;
      g_gpint[i].bl602gpio.id              = i;
      gpio_pin_register(&g_gpint[i].bl602gpio.gpio, pincount);

      /* Configure the pin that will be used as interrupt input */

      bl602_gpio_set_intmod(
        g_gpiointinputs[i], 1, GLB_GPIO_INT_TRIG_NEG_PULSE);
      bl602_configgpio(g_gpiointinputs[i]);

      pincount++;
    }
#endif

  return 0;
}
#endif /* CONFIG_DEV_GPIO && !CONFIG_GPIO_LOWER_HALF */
