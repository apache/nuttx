/****************************************************************************
 * boards/arm/stm32/stm32f411-minimum/src/stm32_gpio.c
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

#include <stdbool.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/clock.h>
#include <nuttx/wdog.h>
#include <nuttx/ioexpander/gpio.h>

#include <arch/board/board.h>

#include "chip.h"
#include "stm32.h"
#include "stm32f411-minimum.h"

#if defined(CONFIG_DEV_GPIO) && !defined(CONFIG_GPIO_LOWER_HALF)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32gpio_dev_s
{
  struct gpio_dev_s gpio;
  uint8_t id;
};

struct stm32gpint_dev_s
{
  struct stm32gpio_dev_s stm32gpio;
  pin_interrupt_t callback;
};

struct stm32gpio_info_s
{
  uint32_t pin;
  const char *pinname;  /* Holds pin name like gpio_a0, gpio_custom_name */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#if BOARD_NGPIO_IN > 0
static int gpin_read(struct gpio_dev_s *dev, bool *value);
#endif

#if BOARD_NGPIO_OUT > 0
static int gpout_read(struct gpio_dev_s *dev, bool *value);
static int gpout_write(struct gpio_dev_s *dev, bool value);
#endif

#if BOARD_NGPIO_INT > 0
static int gpint_read(struct gpio_dev_s *dev, bool *value);
static int gpint_enable(struct gpio_dev_s *dev, bool enable);
static int gpint_attach(struct gpio_dev_s *dev, pin_interrupt_t callback);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if BOARD_NGPIO_IN > 0
static const struct gpio_operations_s gpin_ops =
{
  .go_read   = gpin_read,
  .go_write  = NULL,
  .go_attach = NULL,
  .go_enable = NULL,
};

static struct stm32gpio_dev_s g_gpin[BOARD_NGPIO_IN];
static const struct stm32gpio_info_s g_gpio_inputs[BOARD_NGPIO_IN] =
{
#ifdef CONFIG_STM32F411MINIMUM_GPIO_A0_IN
  { .pin = GPIO_A0_IN, .pinname = CONFIG_STM32F411MINIMUM_GPIO_A0_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_A1_IN
  { .pin = GPIO_A1_IN, .pinname = CONFIG_STM32F411MINIMUM_GPIO_A1_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_A2_IN
  { .pin = GPIO_A2_IN, .pinname = CONFIG_STM32F411MINIMUM_GPIO_A2_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_A3_IN
  { .pin = GPIO_A3_IN, .pinname = CONFIG_STM32F411MINIMUM_GPIO_A3_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_A4_IN
  { .pin = GPIO_A4_IN, .pinname = CONFIG_STM32F411MINIMUM_GPIO_A4_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_A5_IN
  { .pin = GPIO_A5_IN, .pinname = CONFIG_STM32F411MINIMUM_GPIO_A5_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_A6_IN
  { .pin = GPIO_A6_IN, .pinname = CONFIG_STM32F411MINIMUM_GPIO_A6_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_A7_IN
  { .pin = GPIO_A7_IN, .pinname = CONFIG_STM32F411MINIMUM_GPIO_A7_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_A8_IN
  { .pin = GPIO_A8_IN, .pinname = CONFIG_STM32F411MINIMUM_GPIO_A8_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_A9_IN
  { .pin = GPIO_A9_IN, .pinname = CONFIG_STM32F411MINIMUM_GPIO_A9_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_A10_IN
  { .pin = GPIO_A10_IN, .pinname = CONFIG_STM32F411MINIMUM_GPIO_A10_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_A11_IN
  { .pin = GPIO_A11_IN, .pinname = CONFIG_STM32F411MINIMUM_GPIO_A11_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_A12_IN
  { .pin = GPIO_A12_IN, .pinname = CONFIG_STM32F411MINIMUM_GPIO_A12_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_A15_IN
  { .pin = GPIO_A15_IN, .pinname = CONFIG_STM32F411MINIMUM_GPIO_A15_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_B0_IN
  { .pin = GPIO_B0_IN, .pinname = CONFIG_STM32F411MINIMUM_GPIO_B0_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_B1_IN
  { .pin = GPIO_B1_IN, .pinname = CONFIG_STM32F411MINIMUM_GPIO_B1_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_B2_IN
  { .pin = GPIO_B2_IN, .pinname = CONFIG_STM32F411MINIMUM_GPIO_B2_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_B3_IN
  { .pin = GPIO_B3_IN, .pinname = CONFIG_STM32F411MINIMUM_GPIO_B3_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_B4_IN
  { .pin = GPIO_B4_IN, .pinname = CONFIG_STM32F411MINIMUM_GPIO_B4_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_B5_IN
  { .pin = GPIO_B5_IN, .pinname = CONFIG_STM32F411MINIMUM_GPIO_B5_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_B6_IN
  { .pin = GPIO_B6_IN, .pinname = CONFIG_STM32F411MINIMUM_GPIO_B6_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_B7_IN
  { .pin = GPIO_B7_IN, .pinname = CONFIG_STM32F411MINIMUM_GPIO_B7_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_B8_IN
  { .pin = GPIO_B8_IN, .pinname = CONFIG_STM32F411MINIMUM_GPIO_B8_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_B9_IN
  { .pin = GPIO_B9_IN, .pinname = CONFIG_STM32F411MINIMUM_GPIO_B9_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_B10_IN
  { .pin = GPIO_B10_IN, .pinname = CONFIG_STM32F411MINIMUM_GPIO_B10_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_B12_IN
  { .pin = GPIO_B12_IN, .pinname = CONFIG_STM32F411MINIMUM_GPIO_B12_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_B13_IN
  { .pin = GPIO_B13_IN, .pinname = CONFIG_STM32F411MINIMUM_GPIO_B13_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_B14_IN
  { .pin = GPIO_B14_IN, .pinname = CONFIG_STM32F411MINIMUM_GPIO_B14_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_B15_IN
  { .pin = GPIO_B15_IN, .pinname = CONFIG_STM32F411MINIMUM_GPIO_B15_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_C13_IN
  { .pin = GPIO_C13_IN, .pinname = CONFIG_STM32F411MINIMUM_GPIO_C13_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_C14_IN
  { .pin = GPIO_C14_IN, .pinname = CONFIG_STM32F411MINIMUM_GPIO_C14_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_C15_IN
  { .pin = GPIO_C15_IN, .pinname = CONFIG_STM32F411MINIMUM_GPIO_C15_NAME },
#endif
};
#endif /* BOARD_NGPIO_IN > 0 */

#if BOARD_NGPIO_OUT > 0
static const struct gpio_operations_s gpout_ops =
{
  .go_read   = gpout_read,
  .go_write  = gpout_write,
  .go_attach = NULL,
  .go_enable = NULL,
};

static struct stm32gpio_dev_s g_gpout[BOARD_NGPIO_OUT];
static const struct stm32gpio_info_s g_gpio_outputs[BOARD_NGPIO_OUT] =
{
#ifdef CONFIG_STM32F411MINIMUM_GPIO_A0_OUT
  { .pin = GPIO_A0_OUT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_A0_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_A1_OUT
  { .pin = GPIO_A1_OUT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_A1_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_A2_OUT
  { .pin = GPIO_A2_OUT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_A2_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_A3_OUT
  { .pin = GPIO_A3_OUT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_A3_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_A4_OUT
  { .pin = GPIO_A4_OUT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_A4_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_A5_OUT
  { .pin = GPIO_A5_OUT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_A5_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_A6_OUT
  { .pin = GPIO_A6_OUT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_A6_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_A7_OUT
  { .pin = GPIO_A7_OUT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_A7_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_A8_OUT
  { .pin = GPIO_A8_OUT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_A8_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_A9_OUT
  { .pin = GPIO_A9_OUT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_A9_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_A10_OUT
  { .pin = GPIO_A10_OUT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_A10_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_A11_OUT
  { .pin = GPIO_A11_OUT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_A11_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_A12_OUT
  { .pin = GPIO_A12_OUT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_A12_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_A15_OUT
  { .pin = GPIO_A15_OUT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_A15_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_B0_OUT
  { .pin = GPIO_B0_OUT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_B0_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_B1_OUT
  { .pin = GPIO_B1_OUT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_B1_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_B2_OUT
  { .pin = GPIO_B2_OUT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_B2_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_B3_OUT
  { .pin = GPIO_B3_OUT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_B3_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_B4_OUT
  { .pin = GPIO_B4_OUT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_B4_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_B5_OUT
  { .pin = GPIO_B5_OUT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_B5_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_B6_OUT
  { .pin = GPIO_B6_OUT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_B6_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_B7_OUT
  { .pin = GPIO_B7_OUT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_B7_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_B8_OUT
  { .pin = GPIO_B8_OUT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_B8_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_B9_OUT
  { .pin = GPIO_B9_OUT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_B9_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_B10_OUT
  { .pin = GPIO_B10_OUT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_B10_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_B12_OUT
  { .pin = GPIO_B12_OUT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_B12_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_B13_OUT
  { .pin = GPIO_B13_OUT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_B13_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_B14_OUT
  { .pin = GPIO_B14_OUT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_B14_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_B15_OUT
  { .pin = GPIO_B15_OUT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_B15_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_C13_OUT
  { .pin = GPIO_C13_OUT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_C13_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_C14_OUT
  { .pin = GPIO_C14_OUT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_C14_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_C15_OUT
  { .pin = GPIO_C15_OUT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_C15_NAME },
#endif
};
#endif /* BOARD_NGPIO_OUT > 0 */

#if BOARD_NGPIO_INT > 0
static const struct gpio_operations_s gpint_ops =
{
  .go_read   = gpint_read,
  .go_write  = NULL,
  .go_attach = gpint_attach,
  .go_enable = gpint_enable,
};

static struct stm32gpint_dev_s g_gpint[BOARD_NGPIO_INT];
static const struct stm32gpio_info_s g_gpio_int_inputs[BOARD_NGPIO_INT] =
{
#ifdef CONFIG_STM32F411MINIMUM_GPIO_A0_INT
  { .pin = GPIO_A0_INT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_A0_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_A1_INT
  { .pin = GPIO_A1_INT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_A1_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_A2_INT
  { .pin = GPIO_A2_INT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_A2_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_A3_INT
  { .pin = GPIO_A3_INT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_A3_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_A4_INT
  { .pin = GPIO_A4_INT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_A4_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_A5_INT
  { .pin = GPIO_A5_INT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_A5_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_A6_INT
  { .pin = GPIO_A6_INT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_A6_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_A7_INT
  { .pin = GPIO_A7_INT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_A7_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_A8_INT
  { .pin = GPIO_A8_INT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_A8_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_A9_INT
  { .pin = GPIO_A9_INT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_A9_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_A10_INT
  { .pin = GPIO_A10_INT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_A10_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_A11_INT
  { .pin = GPIO_A11_INT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_A11_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_A12_INT
  { .pin = GPIO_A12_INT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_A12_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_A15_INT
  { .pin = GPIO_A15_INT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_A15_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_B0_INT
  { .pin = GPIO_B0_INT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_B0_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_B1_INT
  { .pin = GPIO_B1_INT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_B1_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_B2_INT
  { .pin = GPIO_B2_INT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_B2_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_B3_INT
  { .pin = GPIO_B3_INT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_B3_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_B4_INT
  { .pin = GPIO_B4_INT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_B4_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_B5_INT
  { .pin = GPIO_B5_INT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_B5_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_B6_INT
  { .pin = GPIO_B6_INT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_B6_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_B7_INT
  { .pin = GPIO_B7_INT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_B7_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_B8_INT
  { .pin = GPIO_B8_INT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_B8_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_B9_INT
  { .pin = GPIO_B9_INT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_B9_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_B10_INT
  { .pin = GPIO_B10_INT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_B10_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_B12_INT
  { .pin = GPIO_B12_INT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_B12_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_B13_INT
  { .pin = GPIO_B13_INT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_B13_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_B14_INT
  { .pin = GPIO_B14_INT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_B14_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_B15_INT
  { .pin = GPIO_B15_INT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_B15_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_C13_INT
  { .pin = GPIO_C13_INT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_C13_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_C14_INT
  { .pin = GPIO_C14_INT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_C14_NAME },
#endif

#ifdef CONFIG_STM32F411MINIMUM_GPIO_C15_INT
  { .pin = GPIO_C15_INT, .pinname = CONFIG_STM32F411MINIMUM_GPIO_C15_NAME },
#endif
};
#endif /* BOARD_NGPIO_INT > 0 */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gpin_read
 ****************************************************************************/

#if BOARD_NGPIO_IN > 0
static int gpin_read(struct gpio_dev_s *dev, bool *value)
{
  struct stm32gpio_dev_s *stm32gpio =
                        (struct stm32gpio_dev_s *)dev;

  DEBUGASSERT(stm32gpio != NULL && value != NULL);
  DEBUGASSERT(stm32gpio->id < BOARD_NGPIO_IN);
  gpioinfo("Reading...\n");

  *value = stm32_gpioread(g_gpio_inputs[stm32gpio->id].pin);
  return OK;
}
#endif

/****************************************************************************
 * Name: gpout_read
 ****************************************************************************/

#if BOARD_NGPIO_OUT > 0
static int gpout_read(struct gpio_dev_s *dev, bool *value)
{
  struct stm32gpio_dev_s *stm32gpio =
                        (struct stm32gpio_dev_s *)dev;

  DEBUGASSERT(stm32gpio != NULL && value != NULL);
  DEBUGASSERT(stm32gpio->id < BOARD_NGPIO_OUT);
  gpioinfo("Reading...\n");

  *value = stm32_gpioread(g_gpio_outputs[stm32gpio->id].pin);
  return OK;
}
#endif

/****************************************************************************
 * Name: gpout_write
 ****************************************************************************/

#if BOARD_NGPIO_OUT > 0
static int gpout_write(struct gpio_dev_s *dev, bool value)
{
  struct stm32gpio_dev_s *stm32gpio =
                             (struct stm32gpio_dev_s *)dev;

  DEBUGASSERT(stm32gpio != NULL);
  DEBUGASSERT(stm32gpio->id < BOARD_NGPIO_OUT);
  gpioinfo("Writing %d\n", (int)value);

  stm32_gpiowrite(g_gpio_outputs[stm32gpio->id].pin, value);
  return OK;
}
#endif

/****************************************************************************
 * Name: gpint_read
 ****************************************************************************/

#if BOARD_NGPIO_INT > 0
static int gpint_read(struct gpio_dev_s *dev, bool *value)
{
  struct stm32gpint_dev_s *stm32gpint =
                              (struct stm32gpint_dev_s *)dev;

  DEBUGASSERT(stm32gpint != NULL && value != NULL);
  DEBUGASSERT(stm32gpint->stm32gpio.id < BOARD_NGPIO_INT);
  gpioinfo("Reading int pin...\n");

  *value = stm32_gpioread(g_gpio_int_inputs[stm32gpint->stm32gpio.id].pin);
  return OK;
}
#endif

/****************************************************************************
 * Name: stm32gpio_interrupt
 ****************************************************************************/

#if BOARD_NGPIO_INT > 0
static int stm32gpio_interrupt(int irq, void *context, void *arg)
{
  struct stm32gpint_dev_s *stm32gpint =
                        (struct stm32gpint_dev_s *)arg;

  DEBUGASSERT(stm32gpint != NULL && stm32gpint->callback != NULL);
  gpioinfo("Interrupt! callback=%p\n", stm32gpint->callback);

  stm32gpint->callback(&stm32gpint->stm32gpio.gpio,
                       stm32gpint->stm32gpio.id);
  return OK;
}
#endif

/****************************************************************************
 * Name: gpint_attach
 ****************************************************************************/

#if BOARD_NGPIO_INT > 0
static int gpint_attach(struct gpio_dev_s *dev,
                        pin_interrupt_t callback)
{
  struct stm32gpint_dev_s *stm32gpint =
                             (struct stm32gpint_dev_s *)dev;

  gpioinfo("Attaching the callback\n");

  /* Make sure the interrupt is disabled */

  stm32_gpiosetevent(g_gpio_int_inputs[stm32gpint->stm32gpio.id].pin, false,
                     false, false, NULL, NULL);

  gpioinfo("Attach %p\n", callback);
  stm32gpint->callback = callback;
  return OK;
}
#endif

/****************************************************************************
 * Name: gpint_enable
 ****************************************************************************/

#if BOARD_NGPIO_INT > 0
static int gpint_enable(struct gpio_dev_s *dev, bool enable)
{
  struct stm32gpint_dev_s *stm32gpint =
                              (struct stm32gpint_dev_s *)dev;

  if (enable)
    {
      if (stm32gpint->callback != NULL)
        {
          gpioinfo("Enabling the interrupt\n");

          /* Configure the interrupt for rising edge */

          stm32_gpiosetevent(g_gpio_int_inputs[stm32gpint->stm32gpio.id].pin,
                             true, false, false, stm32gpio_interrupt,
                             &g_gpint[stm32gpint->stm32gpio.id]);
        }
    }
  else
    {
      gpioinfo("Disable the interrupt\n");
      stm32_gpiosetevent(g_gpio_int_inputs[stm32gpint->stm32gpio.id].pin,
                         false, false, false, NULL, NULL);
    }

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_gpio_initialize
 *
 * Description:
 *   Initialize GPIO drivers
 *
 ****************************************************************************/

int stm32_gpio_initialize(void)
{
  int i;

#if BOARD_NGPIO_IN > 0
  for (i = 0; i < BOARD_NGPIO_IN; i++)
    {
      /* Setup and register the GPIO pin */

      g_gpin[i].gpio.gp_pintype = GPIO_INPUT_PIN;
      g_gpin[i].gpio.gp_ops     = &gpin_ops;
      g_gpin[i].id              = i;
      gpio_pin_register_byname(&g_gpin[i].gpio, g_gpio_inputs[i].pinname);

      /* Configure the pin that will be used as input */

      stm32_configgpio(g_gpio_inputs[i].pin);
    }
#endif

#if BOARD_NGPIO_OUT > 0
  for (i = 0; i < BOARD_NGPIO_OUT; i++)
    {
      /* Setup and register the GPIO pin */

      g_gpout[i].gpio.gp_pintype = GPIO_OUTPUT_PIN;
      g_gpout[i].gpio.gp_ops     = &gpout_ops;
      g_gpout[i].id              = i;
      gpio_pin_register_byname(&g_gpout[i].gpio, g_gpio_outputs[i].pinname);

      /* Configure the pin that will be used as output */

      stm32_gpiowrite(g_gpio_outputs[i].pin, 0);
      stm32_configgpio(g_gpio_outputs[i].pin);
    }
#endif

#if BOARD_NGPIO_INT > 0
  for (i = 0; i < BOARD_NGPIO_INT; i++)
    {
      /* Setup and register the GPIO pin */

      g_gpint[i].stm32gpio.gpio.gp_pintype = GPIO_INTERRUPT_PIN;
      g_gpint[i].stm32gpio.gpio.gp_ops     = &gpint_ops;
      g_gpint[i].stm32gpio.id              = i;
      gpio_pin_register_byname(&g_gpint[i].stm32gpio.gpio,
                               g_gpio_int_inputs[i].pinname);

      /* Configure the pin that will be used as interrupt input */

      stm32_configgpio(g_gpio_int_inputs[i].pin);
    }
#endif

  return 0;
}
#endif /* CONFIG_DEV_GPIO && !CONFIG_GPIO_LOWER_HALF */
