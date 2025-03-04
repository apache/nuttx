/****************************************************************************
 * boards/risc-v/esp32h2/esp32h2-devkit/src/esp32h2_gpio.c
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

/* Config */

#include <nuttx/config.h>

/* Libc */

#include <sys/types.h>
#include <syslog.h>
#include <assert.h>
#include <debug.h>

/* NuttX */

#include <arch/irq.h>
#include <nuttx/irq.h>
#include <nuttx/ioexpander/gpio.h>

/* Arch */

#include "espressif/esp_gpio.h"

/* Board */

#include "esp32h2-devkit.h"
#include <arch/board/board.h>

/* HAL */

#include <arch/chip/gpio_sig_map.h>

#if defined(CONFIG_DEV_GPIO) && !defined(CONFIG_GPIO_LOWER_HALF)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Pin 1 and 2 are used for this example as GPIO outputs. */

#define GPIO_OUT1  1
#define GPIO_OUT2  2

#if !defined(CONFIG_ESPRESSIF_GPIO_IRQ) && BOARD_NGPIOINT > 0
#  error "NGPIOINT is > 0 and GPIO interrupts aren't enabled"
#endif

/* Interrupt pins. GPIO9 is used as an example, any other inputs could be
 * used.
 */

#define GPIO_IRQPIN  9

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct espgpio_dev_s
{
  struct gpio_dev_s gpio;
  uint8_t id;
};

struct espgpint_dev_s
{
  struct espgpio_dev_s espgpio;
  pin_interrupt_t callback;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#if BOARD_NGPIOOUT > 0
static int gpout_read(struct gpio_dev_s *dev, bool *value);
static int gpout_write(struct gpio_dev_s *dev, bool value);
static int gpout_setpintype(struct gpio_dev_s *dev,
                            enum gpio_pintype_e pintype);
#endif

#if BOARD_NGPIOINT > 0
static int gpint_read(struct gpio_dev_s *dev, bool *value);
static int gpint_attach(struct gpio_dev_s *dev,
                        pin_interrupt_t callback);
static int gpint_enable(struct gpio_dev_s *dev, bool enable);
static int gpint_setpintype(struct gpio_dev_s *dev,
                            enum gpio_pintype_e pintype);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if BOARD_NGPIOOUT > 0
static const struct gpio_operations_s gpout_ops =
{
  .go_read   = gpout_read,
  .go_write  = gpout_write,
  .go_attach = NULL,
  .go_enable = NULL,
  .go_setpintype = gpout_setpintype,
};

/* This array maps the GPIO pins used as OUTPUT */

static const uint32_t g_gpiooutputs[BOARD_NGPIOOUT] =
{
  GPIO_OUT1, GPIO_OUT2
};

static struct espgpio_dev_s g_gpout[BOARD_NGPIOOUT];
#endif

#if BOARD_NGPIOINT > 0
static const struct gpio_operations_s gpint_ops =
{
  .go_read   = gpint_read,
  .go_write  = NULL,
  .go_attach = gpint_attach,
  .go_enable = gpint_enable,
  .go_setpintype = gpint_setpintype,
};

/* This array maps the GPIO pins used as INTERRUPT INPUTS */

static const uint32_t g_gpiointinputs[BOARD_NGPIOINT] =
{
  GPIO_IRQPIN,
};

static struct espgpint_dev_s g_gpint[BOARD_NGPIOINT];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gpout_read
 *
 * Description:
 *   Read a digital output pin.
 *
 * Parameters:
 *   dev - A pointer to the gpio driver struct.
 *   value - A pointer to store the state of the pin.
 *
 * Returned Value:
 *   Zero (OK).
 *
 ****************************************************************************/

#if BOARD_NGPIOOUT > 0
static int gpout_read(struct gpio_dev_s *dev, bool *value)
{
  struct espgpio_dev_s *espgpio = (struct espgpio_dev_s *)dev;

  DEBUGASSERT(espgpio != NULL && value != NULL);
  DEBUGASSERT(espgpio->id < BOARD_NGPIOOUT);
  gpioinfo("Reading...\n");

  *value = esp_gpioread(g_gpiooutputs[espgpio->id]);
  return OK;
}

/****************************************************************************
 * Name: gpout_write
 *
 * Description:
 *   Write to a digital output pin.
 *
 * Parameters:
 *   dev - A pointer to the gpio driver struct.
 *   value - The value to be written.
 *
 * Returned Value:
 *   Zero (OK).
 *
 ****************************************************************************/

static int gpout_write(struct gpio_dev_s *dev, bool value)
{
  struct espgpio_dev_s *espgpio = (struct espgpio_dev_s *)dev;

  DEBUGASSERT(espgpio != NULL);
  DEBUGASSERT(espgpio->id < BOARD_NGPIOOUT);
  gpioinfo("Writing %d\n", (int)value);

  esp_gpiowrite(g_gpiooutputs[espgpio->id], value);
  return OK;
}

/****************************************************************************
 * Name: gpout_setpintype
 *
 * Description:
 *   Set digital output pin type.
 *
 * Parameters:
 *   dev - A pointer to the gpio driver struct.
 *   pintype - The pin type. See nuttx/ioexpander/gpio.h.
 *
 * Returned Value:
 *   Zero (OK) on success; -1 (ERROR) otherwise.
 *
 ****************************************************************************/

static int gpout_setpintype(struct gpio_dev_s *dev,
                            enum gpio_pintype_e pintype)
{
  struct espgpio_dev_s *espgpio = (struct espgpio_dev_s *)dev;

  DEBUGASSERT(espgpio != NULL);
  DEBUGASSERT(espgpio->id < BOARD_NGPIOOUT);
  gpioinfo("Setting pintype: %d\n", (int)pintype);

  esp_gpio_matrix_out(g_gpiooutputs[espgpio->id],
                      SIG_GPIO_OUT_IDX, 0, 0);

  switch (pintype)
    {
      case GPIO_INPUT_PIN:
        esp_configgpio(g_gpiooutputs[espgpio->id], INPUT);
        break;
      case GPIO_INPUT_PIN_PULLUP:
        esp_configgpio(g_gpiooutputs[espgpio->id], INPUT_PULLUP);
        break;
      case GPIO_INPUT_PIN_PULLDOWN:
        esp_configgpio(g_gpiooutputs[espgpio->id], INPUT_PULLDOWN);
        break;
      case GPIO_OUTPUT_PIN:
        esp_configgpio(g_gpiooutputs[espgpio->id], INPUT | OUTPUT);
        break;
      case GPIO_OUTPUT_PIN_OPENDRAIN:
        esp_configgpio(g_gpiooutputs[espgpio->id],
                       INPUT | OUTPUT_OPEN_DRAIN);
        break;
      default:
        return ERROR;
        break;
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: espgpio_interrupt
 *
 * Description:
 *   Digital input interrupt handler.
 *
 * Input Parameters:
 *   irq           - Identifier of the interrupt request.
 *   context       - Context data from the ISR.
 *   arg           - Opaque pointer to the internal driver state structure.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

#if BOARD_NGPIOINT > 0
static int espgpio_interrupt(int irq, void *context, void *arg)
{
  struct espgpint_dev_s *espgpint = (struct espgpint_dev_s *)arg;

  DEBUGASSERT(espgpint != NULL && espgpint->callback != NULL);
  gpioinfo("Interrupt! callback=%p\n", espgpint->callback);

  espgpint->callback(&espgpint->espgpio.gpio, espgpint->espgpio.id);
  return OK;
}

/****************************************************************************
 * Name: gpint_read
 *
 * Description:
 *   Read a digital input pin.
 *
 * Parameters:
 *   dev - A pointer to the gpio driver struct.
 *   value - A pointer to store the state of the pin.
 *
 * Returned Value:
 *   Zero (OK).
 *
 ****************************************************************************/

static int gpint_read(struct gpio_dev_s *dev, bool *value)
{
  struct espgpint_dev_s *espgpint =
    (struct espgpint_dev_s *)dev;

  DEBUGASSERT(espgpint != NULL && value != NULL);
  DEBUGASSERT(espgpint->espgpio.id < BOARD_NGPIOINT);
  gpioinfo("Reading int pin...\n");

  *value = esp_gpioread(g_gpiointinputs[espgpint->espgpio.id]);
  return OK;
}

/****************************************************************************
 * Name: gpint_attach
 *
 * Description:
 *   Attach the ISR to IRQ and register the callback. But it still doesn't
 *   enable interrupt yet.
 *
 * Parameters:
 *   dev - A pointer to the gpio driver struct.
 *   callback - User callback function.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

static int gpint_attach(struct gpio_dev_s *dev,
                        pin_interrupt_t callback)
{
  struct espgpint_dev_s *espgpint =
    (struct espgpint_dev_s *)dev;
  int irq = ESP_PIN2IRQ(g_gpiointinputs[espgpint->espgpio.id]);
  int ret;

  gpioinfo("Attaching the callback\n");

  /* Make sure the interrupt is disabled */

  esp_gpioirqdisable(irq);
  ret = irq_attach(irq,
                   espgpio_interrupt,
                   &g_gpint[espgpint->espgpio.id]);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: gpint_attach() failed: %d\n", ret);
      return ret;
    }

  gpioinfo("Attach %p\n", callback);
  espgpint->callback = callback;
  return OK;
}

/****************************************************************************
 * Name: gpint_enable
 *
 * Description:
 *   Enable/Disable interrupt.
 *
 * Parameters:
 *   dev - A pointer to the gpio driver struct.
 *   enable - True to enable, false to disable.
 *
 * Returned Value:
 *   Zero (OK).
 *
 ****************************************************************************/

static int gpint_enable(struct gpio_dev_s *dev, bool enable)
{
  struct espgpint_dev_s *espgpint = (struct espgpint_dev_s *)dev;
  int irq = ESP_PIN2IRQ(g_gpiointinputs[espgpint->espgpio.id]);

  if (enable)
    {
      if (espgpint->callback != NULL)
        {
          gpioinfo("Enabling the interrupt\n");

          /* Configure the interrupt for rising edge */

          esp_gpioirqenable(irq, RISING);
        }
    }
  else
    {
      gpioinfo("Disable the interrupt\n");
      esp_gpioirqdisable(irq);
    }

  return OK;
}

/****************************************************************************
 * Name: gpint_setpintype
 *
 * Description:
 *   Set digital interrupt pin type.
 *
 * Parameters:
 *   dev - A pointer to the gpio driver struct.
 *   pintype - The pin type. See nuttx/ioexpander/gpio.h.
 *
 * Returned Value:
 *   Zero (OK) on success; -1 (ERROR) otherwise.
 *
 ****************************************************************************/

static int gpint_setpintype(struct gpio_dev_s *dev,
                            enum gpio_pintype_e pintype)
{
  struct espgpint_dev_s *espgpint = (struct espgpint_dev_s *)dev;

  DEBUGASSERT(espgpint != NULL);
  DEBUGASSERT(espgpint->espgpio.id < BOARD_NGPIOINT);
  gpioinfo("Setting pintype: %d\n", (int)pintype);
  switch (pintype)
    {
      case GPIO_INTERRUPT_HIGH_PIN:
        esp_configgpio(g_gpiointinputs[espgpint->espgpio.id],
                       INPUT_PULLUP);
        break;
      case GPIO_INTERRUPT_LOW_PIN:
        esp_configgpio(g_gpiointinputs[espgpint->espgpio.id],
                       INPUT_PULLDOWN);
        break;
      default:
        return ERROR;
        break;
    }

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_gpio_init
 *
 * Description:
 *   Configure the GPIO driver.
 *
 * Returned Value:
 *   Zero (OK).
 *
 ****************************************************************************/

int esp_gpio_init(void)
{
  int pincount = 0;
  int i;

#if BOARD_NGPIOOUT > 0
  for (i = 0; i < BOARD_NGPIOOUT; i++)
    {
      /* Setup and register the GPIO pin */

      g_gpout[i].gpio.gp_pintype = GPIO_OUTPUT_PIN;
      g_gpout[i].gpio.gp_ops     = &gpout_ops;
      g_gpout[i].id              = i;
      gpio_pin_register(&g_gpout[i].gpio, pincount);

      /* Configure the pins that will be used as output */

      esp_gpio_matrix_out(g_gpiooutputs[i], SIG_GPIO_OUT_IDX, 0, 0);
      esp_configgpio(g_gpiooutputs[i], OUTPUT_FUNCTION_2 | INPUT_FUNCTION_2);
      esp_gpiowrite(g_gpiooutputs[i], 0);

      pincount++;
    }
#endif

#if BOARD_NGPIOINT > 0
  for (i = 0; i < BOARD_NGPIOINT; i++)
    {
      /* Setup and register the GPIO pin */

      g_gpint[i].espgpio.gpio.gp_pintype = GPIO_INTERRUPT_PIN;
      g_gpint[i].espgpio.gpio.gp_ops     = &gpint_ops;
      g_gpint[i].espgpio.id              = i;
      gpio_pin_register(&g_gpint[i].espgpio.gpio, pincount);

      /* Configure the pins that will be used as interrupt input */

      esp_configgpio(g_gpiointinputs[i], INPUT_FUNCTION_2 | PULLDOWN);

      pincount++;
    }
#endif

  return OK;
}
#endif /* CONFIG_DEV_GPIO && !CONFIG_GPIO_LOWER_HALF */
