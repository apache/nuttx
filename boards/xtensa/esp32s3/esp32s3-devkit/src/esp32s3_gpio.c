/****************************************************************************
 * boards/xtensa/esp32s3/esp32s3-devkit/src/esp32s3_gpio.c
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

#include <sys/types.h>
#include <nuttx/irq.h>
#include <arch/irq.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/ioexpander/gpio.h>

#include <arch/board/board.h>

#include "esp32s3-devkit.h"
#include "esp32s3_gpio.h"
#include "hardware/esp32s3_gpio_sigmap.h"

#if defined(CONFIG_DEV_GPIO) && !defined(CONFIG_GPIO_LOWER_HALF)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if !defined(CONFIG_ESP32S3_GPIO_IRQ) && BOARD_NGPIOINT > 0
#  error "NGPIOINT is > 0 and GPIO interrupts aren't enabled"
#endif

/* Output pins. GPIO15 is used as an example, any other outputs could be
 * used.
 */

#define GPIO_OUT1    15

/* Input pins. GPIO18 is used as an example, any other inputs could be
 * used.
 */

#define GPIO_IN1     18

/* Interrupt pins. GPIO21 is used as an example, any other inputs could be
 * used.
 */

#define GPIO_IRQPIN1 21

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp32s3gpio_dev_s
{
  struct gpio_dev_s gpio;
  uint8_t id;
};

struct esp32s3gpint_dev_s
{
  struct esp32s3gpio_dev_s esp32s3gpio;
  pin_interrupt_t callback;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#if BOARD_NGPIOOUT > 0
static int gpout_read(struct gpio_dev_s *dev, bool *value);
static int gpout_write(struct gpio_dev_s *dev, bool value);
#endif

#if BOARD_NGPIOIN > 0
static int gpin_read(struct gpio_dev_s *dev, bool *value);
#endif

#if BOARD_NGPIOINT > 0
static int gpint_read(struct gpio_dev_s *dev, bool *value);
static int gpint_attach(struct gpio_dev_s *dev,
                        pin_interrupt_t callback);
static int gpint_enable(struct gpio_dev_s *dev, bool enable);
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
  GPIO_OUT1
};

static struct esp32s3gpio_dev_s g_gpout[BOARD_NGPIOOUT];
#endif

#if BOARD_NGPIOIN > 0
static const struct gpio_operations_s gpin_ops =
{
  .go_read   = gpin_read,
  .go_write  = NULL,
  .go_attach = NULL,
  .go_enable = NULL,
};

/* This array maps the GPIO pins used as INTERRUPT INPUTS */

static const uint32_t g_gpioinputs[BOARD_NGPIOIN] =
{
  GPIO_IN1
};

static struct esp32s3gpio_dev_s g_gpin[BOARD_NGPIOIN];
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
  GPIO_IRQPIN1,
};

static struct esp32s3gpint_dev_s g_gpint[BOARD_NGPIOINT];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gpout_read
 ****************************************************************************/

#if BOARD_NGPIOOUT > 0
static int gpout_read(struct gpio_dev_s *dev, bool *value)
{
  struct esp32s3gpio_dev_s *esp32s3gpio = (struct esp32s3gpio_dev_s *)dev;

  DEBUGASSERT(esp32s3gpio != NULL && value != NULL);
  DEBUGASSERT(esp32s3gpio->id < BOARD_NGPIOOUT);
  gpioinfo("Reading...\n");

  *value = esp32s3_gpioread(g_gpiooutputs[esp32s3gpio->id]);
  return OK;
}

/****************************************************************************
 * Name: gpout_write
 ****************************************************************************/

static int gpout_write(struct gpio_dev_s *dev, bool value)
{
  struct esp32s3gpio_dev_s *esp32s3gpio = (struct esp32s3gpio_dev_s *)dev;

  DEBUGASSERT(esp32s3gpio != NULL);
  DEBUGASSERT(esp32s3gpio->id < BOARD_NGPIOOUT);
  gpioinfo("Writing %d\n", (int)value);

  esp32s3_gpiowrite(g_gpiooutputs[esp32s3gpio->id], value);
  return OK;
}
#endif

/****************************************************************************
 * Name: gpin_read
 ****************************************************************************/

#if BOARD_NGPIOIN > 0
static int gpin_read(struct gpio_dev_s *dev, bool *value)
{
  struct esp32s3gpio_dev_s *esp32s3gpio = (struct esp32s3gpio_dev_s *)dev;

  DEBUGASSERT(esp32s3gpio != NULL && value != NULL);
  DEBUGASSERT(esp32s3gpio->id < BOARD_NGPIOIN);
  gpioinfo("Reading... pin %d\n", g_gpioinputs[esp32s3gpio->id]);

  *value = esp32s3_gpioread(g_gpioinputs[esp32s3gpio->id]);
  return OK;
}
#endif

/****************************************************************************
 * Name: esp32s3gpio_interrupt
 ****************************************************************************/

#if BOARD_NGPIOINT > 0
static int esp32s3gpio_interrupt(int irq, void *context, void *arg)
{
  struct esp32s3gpint_dev_s *esp32s3gpint = (struct esp32s3gpint_dev_s *)arg;

  DEBUGASSERT(esp32s3gpint != NULL && esp32s3gpint->callback != NULL);
  gpioinfo("Interrupt! callback=%p\n", esp32s3gpint->callback);

  esp32s3gpint->callback(&esp32s3gpint->esp32s3gpio.gpio,
                         esp32s3gpint->esp32s3gpio.id);
  return OK;
}

/****************************************************************************
 * Name: gpint_read
 ****************************************************************************/

static int gpint_read(struct gpio_dev_s *dev, bool *value)
{
  struct esp32s3gpint_dev_s *esp32s3gpint = (struct esp32s3gpint_dev_s *)dev;

  DEBUGASSERT(esp32s3gpint != NULL && value != NULL);
  DEBUGASSERT(esp32s3gpint->esp32s3gpio.id < BOARD_NGPIOINT);
  gpioinfo("Reading int pin...\n");

  *value = esp32s3_gpioread(g_gpiointinputs[esp32s3gpint->esp32s3gpio.id]);
  return OK;
}

/****************************************************************************
 * Name: gpint_attach
 ****************************************************************************/

static int gpint_attach(struct gpio_dev_s *dev,
                        pin_interrupt_t callback)
{
  struct esp32s3gpint_dev_s *esp32s3gpint =
    (struct esp32s3gpint_dev_s *)dev;
  int irq = ESP32S3_PIN2IRQ(g_gpiointinputs[esp32s3gpint->esp32s3gpio.id]);
  int ret;

  gpioinfo("Attaching the callback\n");

  /* Make sure the interrupt is disabled */

  esp32s3_gpioirqdisable(irq);
  ret = irq_attach(irq,
                   esp32s3gpio_interrupt,
                   &g_gpint[esp32s3gpint->esp32s3gpio.id]);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: gpint_attach() failed: %d\n", ret);
      return ret;
    }

  gpioinfo("Attach %p\n", callback);
  esp32s3gpint->callback = callback;
  return OK;
}

/****************************************************************************
 * Name: gpint_enable
 ****************************************************************************/

static int gpint_enable(struct gpio_dev_s *dev, bool enable)
{
  struct esp32s3gpint_dev_s *esp32s3gpint = (struct esp32s3gpint_dev_s *)dev;
  int irq = ESP32S3_PIN2IRQ(g_gpiointinputs[esp32s3gpint->esp32s3gpio.id]);

  if (enable)
    {
      if (esp32s3gpint->callback != NULL)
        {
          gpioinfo("Enabling the interrupt\n");

          /* Configure the interrupt for rising edge */

          esp32s3_gpioirqenable(irq, RISING);
        }
    }
  else
    {
      gpioinfo("Disable the interrupt\n");
      esp32s3_gpioirqdisable(irq);
    }

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_gpio_init
 ****************************************************************************/

int esp32s3_gpio_init(void)
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

      esp32s3_gpio_matrix_out(g_gpiooutputs[i], SIG_GPIO_OUT_IDX, 0, 0);
      esp32s3_configgpio(g_gpiooutputs[i], OUTPUT_FUNCTION_2 |
                         INPUT_FUNCTION_2);
      esp32s3_gpiowrite(g_gpiooutputs[i], 0);

      pincount++;
    }
#endif

#if BOARD_NGPIOIN > 0
  for (i = 0; i < BOARD_NGPIOIN; i++)
    {
      /* Setup and register the GPIO pin */

      g_gpin[i].gpio.gp_pintype = GPIO_INPUT_PIN;
      g_gpin[i].gpio.gp_ops     = &gpin_ops;
      g_gpin[i].id              = i;
      gpio_pin_register(&g_gpin[i].gpio, pincount);

      /* Configure the pins that will be used as INPUT */

      esp32s3_configgpio(g_gpioinputs[i], INPUT_FUNCTION_2);

      pincount++;
    }
#endif

#if BOARD_NGPIOINT > 0
  for (i = 0; i < BOARD_NGPIOINT; i++)
    {
      /* Setup and register the GPIO pin */

      g_gpint[i].esp32s3gpio.gpio.gp_pintype = GPIO_INTERRUPT_PIN;
      g_gpint[i].esp32s3gpio.gpio.gp_ops     = &gpint_ops;
      g_gpint[i].esp32s3gpio.id              = i;
      gpio_pin_register(&g_gpint[i].esp32s3gpio.gpio, pincount);

      /* Configure the pins that will be used as interrupt input */

      esp32s3_configgpio(g_gpiointinputs[i], INPUT_FUNCTION_2 | PULLDOWN);

      pincount++;
    }
#endif

  return OK;
}
#endif /* CONFIG_DEV_GPIO && !CONFIG_GPIO_LOWER_HALF */
