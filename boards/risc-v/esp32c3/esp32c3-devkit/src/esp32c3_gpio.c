/****************************************************************************
 * boards/risc-v/esp32c3/esp32c3-devkit/src/esp32c3_gpio.c
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
#include <syslog.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <arch/irq.h>

#include <nuttx/ioexpander/gpio.h>

#include <arch/board/board.h>

#include "esp32c3-devkit.h"
#include "esp32c3_gpio.h"
#include "hardware/esp32c3_gpio_sigmap.h"

#if defined(CONFIG_DEV_GPIO) && !defined(CONFIG_GPIO_LOWER_HALF)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Pin 1 and 2 are used for this example as GPIO outputs. */

#define GPIO_OUT1  1
#define GPIO_OUT2  2

#if !defined(CONFIG_ESP32C3_GPIO_IRQ) && BOARD_NGPIOINT > 0
#  error "NGPIOINT is > 0 and GPIO interrupts aren't enabled"
#endif

/* Interrupt pins.  GPIO9 is used as an example, any other inputs could be
 * used.
 */

#define GPIO_IRQPIN  9

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp32c3gpio_dev_s
{
  struct gpio_dev_s gpio;
  uint8_t id;
};

struct esp32c3gpint_dev_s
{
  struct esp32c3gpio_dev_s esp32c3gpio;
  pin_interrupt_t callback;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#if BOARD_NGPIOOUT > 0
static int gpout_read(FAR struct gpio_dev_s *dev, FAR bool *value);
static int gpout_write(FAR struct gpio_dev_s *dev, bool value);
#endif

#if BOARD_NGPIOINT > 0
static int gpint_read(FAR struct gpio_dev_s *dev, FAR bool *value);
static int gpint_attach(FAR struct gpio_dev_s *dev,
                        pin_interrupt_t callback);
static int gpint_enable(FAR struct gpio_dev_s *dev, bool enable);
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
};

/* This array maps the GPIO pins used as OUTPUT */

static const uint32_t g_gpiooutputs[BOARD_NGPIOOUT] =
{
  GPIO_OUT1, GPIO_OUT2
};

static struct esp32c3gpio_dev_s g_gpout[BOARD_NGPIOOUT];
#endif

#if BOARD_NGPIOINT > 0
static const struct gpio_operations_s gpint_ops =
{
  .go_read   = gpint_read,
  .go_write  = NULL,
  .go_attach = gpint_attach,
  .go_enable = gpint_enable,
};

/* This array maps the GPIO pins used as INTERRUPT INPUTS */

static const uint32_t g_gpiointinputs[BOARD_NGPIOINT] =
{
  GPIO_IRQPIN,
};

static struct esp32c3gpint_dev_s g_gpint[BOARD_NGPIOINT];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gpout_read
 ****************************************************************************/

#if BOARD_NGPIOOUT > 0
static int gpout_read(FAR struct gpio_dev_s *dev, FAR bool *value)
{
  FAR struct esp32c3gpio_dev_s *esp32c3gpio =
    (FAR struct esp32c3gpio_dev_s *)dev;

  DEBUGASSERT(esp32c3gpio != NULL && value != NULL);
  DEBUGASSERT(esp32c3gpio->id < BOARD_NGPIOOUT);
  gpioinfo("Reading...\n");

  *value = esp32c3_gpioread(g_gpiooutputs[esp32c3gpio->id]);
  return OK;
}

/****************************************************************************
 * Name: gpout_write
 ****************************************************************************/

static int gpout_write(FAR struct gpio_dev_s *dev, bool value)
{
  FAR struct esp32c3gpio_dev_s *esp32c3gpio =
    (FAR struct esp32c3gpio_dev_s *)dev;

  DEBUGASSERT(esp32c3gpio != NULL);
  DEBUGASSERT(esp32c3gpio->id < BOARD_NGPIOOUT);
  gpioinfo("Writing %d\n", (int)value);

  esp32c3_gpiowrite(g_gpiooutputs[esp32c3gpio->id], value);
  return OK;
}
#endif

/****************************************************************************
 * Name: esp32c3gpio_interrupt
 ****************************************************************************/

#if BOARD_NGPIOINT > 0
static int esp32c3gpio_interrupt(int irq, void *context, void *arg)
{
  FAR struct esp32c3gpint_dev_s *esp32c3gpint =
    (FAR struct esp32c3gpint_dev_s *)arg;

  DEBUGASSERT(esp32c3gpint != NULL && esp32c3gpint->callback != NULL);
  gpioinfo("Interrupt! callback=%p\n", esp32c3gpint->callback);

  esp32c3gpint->callback(&esp32c3gpint->esp32c3gpio.gpio,
                         esp32c3gpint->esp32c3gpio.id);
  return OK;
}

/****************************************************************************
 * Name: gpint_read
 ****************************************************************************/

static int gpint_read(FAR struct gpio_dev_s *dev, FAR bool *value)
{
  FAR struct esp32c3gpint_dev_s *esp32c3gpint =
    (FAR struct esp32c3gpint_dev_s *)dev;

  DEBUGASSERT(esp32c3gpint != NULL && value != NULL);
  DEBUGASSERT(esp32c3gpint->esp32c3gpio.id < BOARD_NGPIOINT);
  gpioinfo("Reading int pin...\n");

  *value = esp32c3_gpioread(g_gpiointinputs[esp32c3gpint->esp32c3gpio.id]);
  return OK;
}

/****************************************************************************
 * Name: gpint_attach
 ****************************************************************************/

static int gpint_attach(FAR struct gpio_dev_s *dev,
                        pin_interrupt_t callback)
{
  FAR struct esp32c3gpint_dev_s *esp32c3gpint =
    (FAR struct esp32c3gpint_dev_s *)dev;
  int irq = ESP32C3_PIN2IRQ(g_gpiointinputs[esp32c3gpint->esp32c3gpio.id]);
  int ret;

  gpioinfo("Attaching the callback\n");

  /* Make sure the interrupt is disabled */

  esp32c3_gpioirqdisable(irq);
  ret = irq_attach(irq,
                   esp32c3gpio_interrupt,
                   &g_gpint[esp32c3gpint->esp32c3gpio.id]);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: gpint_attach() failed: %d\n", ret);
      return ret;
    }

  gpioinfo("Attach %p\n", callback);
  esp32c3gpint->callback = callback;
  return OK;
}

/****************************************************************************
 * Name: gpint_enable
 ****************************************************************************/

static int gpint_enable(FAR struct gpio_dev_s *dev, bool enable)
{
  FAR struct esp32c3gpint_dev_s *esp32c3gpint =
    (FAR struct esp32c3gpint_dev_s *)dev;
  int irq = ESP32C3_PIN2IRQ(g_gpiointinputs[esp32c3gpint->esp32c3gpio.id]);

  if (enable)
    {
      if (esp32c3gpint->callback != NULL)
        {
          gpioinfo("Enabling the interrupt\n");

          /* Configure the interrupt for rising edge */

          esp32c3_gpioirqenable(irq, RISING);
        }
    }
  else
    {
      gpioinfo("Disable the interrupt\n");
      esp32c3_gpioirqdisable(irq);
    }

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32c3_gpio_init
 ****************************************************************************/

int esp32c3_gpio_init(void)
{
  int i;

#if BOARD_NGPIOOUT > 0
  for (i = 0; i < BOARD_NGPIOOUT; i++)
    {
      /* Setup and register the GPIO pin */

      g_gpout[i].gpio.gp_pintype = GPIO_OUTPUT_PIN;
      g_gpout[i].gpio.gp_ops     = &gpout_ops;
      g_gpout[i].id              = i;
      gpio_pin_register(&g_gpout[i].gpio, i);

      /* Configure the pins that will be used as output */

      esp32c3_gpio_matrix_out(g_gpiooutputs[i], SIG_GPIO_OUT_IDX, 0, 0);
      esp32c3_configgpio(g_gpiooutputs[i], OUTPUT_FUNCTION_1 |
                         INPUT_FUNCTION_1);
      esp32c3_gpiowrite(g_gpiooutputs[i], 0);
    }
#endif

#if BOARD_NGPIOINT > 0
  for (i = 0; i < BOARD_NGPIOINT; i++)
    {
      /* Setup and register the GPIO pin */

      g_gpint[i].esp32c3gpio.gpio.gp_pintype = GPIO_INTERRUPT_PIN;
      g_gpint[i].esp32c3gpio.gpio.gp_ops     = &gpint_ops;
      g_gpint[i].esp32c3gpio.id              = i;
      gpio_pin_register(&g_gpint[i].esp32c3gpio.gpio, i);

      /* Configure the pins that will be used as interrupt input */

      esp32c3_configgpio(g_gpiointinputs[i], INPUT_FUNCTION_1 | PULLDOWN);
    }
#endif

  return OK;
}
#endif /* CONFIG_DEV_GPIO && !CONFIG_GPIO_LOWER_HALF */
