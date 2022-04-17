/****************************************************************************
 * boards/xtensa/esp32s2/esp32s2-saola-1/src/esp32s2_gpio.c
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

#include "esp32s2-saola-1.h"
#include "esp32s2_gpio.h"
#include "hardware/esp32s2_gpio_sigmap.h"

#if defined(CONFIG_DEV_GPIO) && !defined(CONFIG_GPIO_LOWER_HALF)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Pin 1 and 2 are used for this example as GPIO outputs. */

#define GPIO_OUT1  1
#define GPIO_OUT2  2
#define GPIO_IN1   4

#if !defined(CONFIG_ESP32S2_GPIO_IRQ) && BOARD_NGPIOINT > 0
#  error "NGPIOINT is > 0 and GPIO interrupts aren't enabled"
#endif

/* Interrupt pins.  GPIO9 is used as an example, any other inputs could be
 * used.
 */

#define GPIO_IRQPIN  9

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp32s2gpio_dev_s
{
  struct gpio_dev_s gpio;
  uint8_t id;
};

struct esp32s2gpint_dev_s
{
  struct esp32s2gpio_dev_s esp32s2gpio;
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
  GPIO_OUT1, GPIO_OUT2
};

static struct esp32s2gpio_dev_s g_gpout[BOARD_NGPIOOUT];
#endif

#if BOARD_NGPIOIN > 0
static const struct gpio_operations_s gpin_ops =
{
  .go_read   = gpin_read,
  .go_write  = NULL,
  .go_attach = NULL,
  .go_enable = NULL,
};

/* This array maps the GPIO pins used as OUTPUT */

static const uint32_t g_gpioinputs[BOARD_NGPIOIN] =
{
  GPIO_IN1
};

static struct esp32s2gpio_dev_s g_gpin[BOARD_NGPIOIN];
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

static struct esp32s2gpint_dev_s g_gpint[BOARD_NGPIOINT];
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
  struct esp32s2gpio_dev_s *esp32s2gpio =
    (struct esp32s2gpio_dev_s *)dev;

  DEBUGASSERT(esp32s2gpio != NULL && value != NULL);
  DEBUGASSERT(esp32s2gpio->id < BOARD_NGPIOOUT);
  gpioinfo("Reading...\n");

  *value = esp32s2_gpioread(g_gpiooutputs[esp32s2gpio->id]);
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
  struct esp32s2gpio_dev_s *esp32s2gpio =
    (struct esp32s2gpio_dev_s *)dev;

  DEBUGASSERT(esp32s2gpio != NULL);
  DEBUGASSERT(esp32s2gpio->id < BOARD_NGPIOOUT);
  gpioinfo("Writing %d\n", (int)value);

  esp32s2_gpiowrite(g_gpiooutputs[esp32s2gpio->id], value);
  return OK;
}
#endif

/****************************************************************************
 * Name: gpin_read
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

#if BOARD_NGPIOIN > 0
static int gpin_read(struct gpio_dev_s *dev, bool *value)
{
  struct esp32s2gpio_dev_s *esp32s2gpio =
    (struct esp32s2gpio_dev_s *)dev;

  DEBUGASSERT(esp32s2gpio != NULL && value != NULL);
  DEBUGASSERT(esp32s2gpio->id < BOARD_NGPIOIN);
  gpioinfo("Reading...\n");

  *value = esp32s2_gpioread(g_gpioinputs[esp32s2gpio->id]);
  return OK;
}
#endif

/****************************************************************************
 * Name: esp32s2gpio_interrupt
 *
 * Description:
 *   Digital Input ISR.
 *
 ****************************************************************************/

#if BOARD_NGPIOINT > 0
static int esp32s2gpio_interrupt(int irq, void *context, void *arg)
{
  struct esp32s2gpint_dev_s *esp32s2gpint =
    (struct esp32s2gpint_dev_s *)arg;

  DEBUGASSERT(esp32s2gpint != NULL && esp32s2gpint->callback != NULL);
  gpioinfo("Interrupt! callback=%p\n", esp32s2gpint->callback);

  esp32s2gpint->callback(&esp32s2gpint->esp32s2gpio.gpio,
                         esp32s2gpint->esp32s2gpio.id);
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
  struct esp32s2gpint_dev_s *esp32s2gpint =
    (struct esp32s2gpint_dev_s *)dev;

  DEBUGASSERT(esp32s2gpint != NULL && value != NULL);
  DEBUGASSERT(esp32s2gpint->esp32s2gpio.id < BOARD_NGPIOINT);
  gpioinfo("Reading int pin...\n");

  *value = esp32s2_gpioread(g_gpiointinputs[esp32s2gpint->esp32s2gpio.id]);
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
  struct esp32s2gpint_dev_s *esp32s2gpint =
    (struct esp32s2gpint_dev_s *)dev;
  int irq = ESP32S2_PIN2IRQ(g_gpiointinputs[esp32s2gpint->esp32s2gpio.id]);
  int ret;

  gpioinfo("Attaching the callback\n");

  /* Make sure the interrupt is disabled */

  esp32s2_gpioirqdisable(irq);
  ret = irq_attach(irq,
                   esp32s2gpio_interrupt,
                   &g_gpint[esp32s2gpint->esp32s2gpio.id]);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: gpint_attach() failed: %d\n", ret);
      return ret;
    }

  gpioinfo("Attach %p\n", callback);
  esp32s2gpint->callback = callback;
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
  struct esp32s2gpint_dev_s *esp32s2gpint =
    (struct esp32s2gpint_dev_s *)dev;
  int irq = ESP32S2_PIN2IRQ(g_gpiointinputs[esp32s2gpint->esp32s2gpio.id]);

  if (enable)
    {
      if (esp32s2gpint->callback != NULL)
        {
          gpioinfo("Enabling the interrupt\n");

          /* Configure the interrupt for rising edge */

          esp32s2_gpioirqenable(irq, RISING);
        }
    }
  else
    {
      gpioinfo("Disable the interrupt\n");
      esp32s2_gpioirqdisable(irq);
    }

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s2_gpio_init
 *
 * Description:
 *   Initialize all pins. Function should be called from the bringup.
 *
 * Returned Value:
 *   Zero (OK).
 *
 ****************************************************************************/

int esp32s2_gpio_init(void)
{
  int pincount = 0;

#if BOARD_NGPIOOUT > 0
  for (int i = 0; i < BOARD_NGPIOOUT; i++)
    {
      /* Setup and register the GPIO pin */

      g_gpout[i].gpio.gp_pintype = GPIO_OUTPUT_PIN;
      g_gpout[i].gpio.gp_ops     = &gpout_ops;
      g_gpout[i].id              = i;
      gpio_pin_register(&g_gpout[i].gpio, pincount);

      /* Configure the pins that will be used as output */

      esp32s2_gpio_matrix_out(g_gpiooutputs[i], SIG_GPIO_OUT_IDX, 0, 0);
      esp32s2_configgpio(g_gpiooutputs[i], OUTPUT_FUNCTION_1 |
                         INPUT_FUNCTION_1);
      esp32s2_gpiowrite(g_gpiooutputs[i], 0);

      pincount++;
    }
#endif

#if BOARD_NGPIOIN > 0
  for (int i = 0; i < BOARD_NGPIOIN; i++)
    {
      /* Setup and register the GPIO pin */

      g_gpin[i].gpio.gp_pintype = GPIO_INPUT_PIN_PULLDOWN;
      g_gpin[i].gpio.gp_ops     = &gpin_ops;
      g_gpin[i].id              = i;
      gpio_pin_register(&g_gpin[i].gpio, pincount);

      /* Configure the pins that will be used as interrupt input */

      esp32s2_configgpio(g_gpioinputs[i], INPUT_FUNCTION_1 | PULLDOWN);

      pincount++;
    }
#endif

#if BOARD_NGPIOINT > 0
  for (int i = 0; i < BOARD_NGPIOINT; i++)
    {
      /* Setup and register the GPIO pin */

      g_gpint[i].esp32s2gpio.gpio.gp_pintype = GPIO_INTERRUPT_PIN;
      g_gpint[i].esp32s2gpio.gpio.gp_ops     = &gpint_ops;
      g_gpint[i].esp32s2gpio.id              = i;
      gpio_pin_register(&g_gpint[i].esp32s2gpio.gpio, pincount);

      /* Configure the pins that will be used as interrupt input */

      esp32s2_configgpio(g_gpiointinputs[i], INPUT_FUNCTION_1 | PULLDOWN);

      pincount++;
    }
#endif

  return OK;
}
#endif /* CONFIG_DEV_GPIO && !CONFIG_GPIO_LOWER_HALF */
