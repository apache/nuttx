/****************************************************************************
 * boards/arm/stm32h7/weact-stm32h743/src/stm32_morse.c
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

#include <debug.h>
#include <errno.h>
#include <stdbool.h>

#include <nuttx/arch.h>
#include <nuttx/spi/spi.h>
#include <nuttx/wireless/morse.h>

#include "stm32_gpio.h"
#include "stm32_spi.h"
#include "weact-stm32h743.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The HaLow module is wired to SPI1 */

#define HALOW_SPI_BUS     1

/* EXTI line interrupts of the IRQ (PA1) and BUSY (PA0) pins */

#define HALOW_IRQ_IRQ     STM32_IRQ_EXTI1
#define HALOW_BUSY_IRQ    STM32_IRQ_EXTI0

/* With jumper wires the SPI clock ringing limits the bus to ~20 MHz */

#define HALOW_SPI_FREQ    20000000

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void halow_reset(const struct morse_lower_s *lower, bool assert);
#ifdef CONFIG_WEACT_STM32H743_HALOW_POWERSAVE
static void halow_wake(const struct morse_lower_s *lower, bool assert);
static bool halow_busy(const struct morse_lower_s *lower);
static int  halow_busy_attach(const struct morse_lower_s *lower,
                              xcpt_t handler, void *arg);
static void halow_busy_enable(const struct morse_lower_s *lower,
                              bool enable);
#endif
static bool halow_irq_asserted(const struct morse_lower_s *lower);
static int  halow_irq_attach(const struct morse_lower_s *lower,
                             xcpt_t handler, void *arg);
static void halow_irq_enable(const struct morse_lower_s *lower,
                             bool enable);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct morse_lower_s g_halow_lower =
{
  .devid        = SPIDEV_WIRELESS(0),
  .frequency    = HALOW_SPI_FREQ,
  .reset        = halow_reset,
#ifdef CONFIG_WEACT_STM32H743_HALOW_POWERSAVE
  .wake         = halow_wake,
  .busy         = halow_busy,
  .busy_attach  = halow_busy_attach,
  .busy_enable  = halow_busy_enable,
#endif
  .irq_asserted = halow_irq_asserted,
  .irq_attach   = halow_irq_attach,
  .irq_enable   = halow_irq_enable,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void halow_reset(const struct morse_lower_s *lower, bool assert)
{
  stm32_gpiowrite(GPIO_HALOW_RESET, !assert);
}

static bool halow_irq_asserted(const struct morse_lower_s *lower)
{
  return !stm32_gpioread(GPIO_HALOW_IRQ);
}

static int halow_irq_attach(const struct morse_lower_s *lower,
                            xcpt_t handler, void *arg)
{
  /* The IRQ line is active low: interrupt on the falling edge */

  return stm32_gpiosetevent(GPIO_HALOW_IRQ, false, true, true,
                            handler, arg);
}

static void halow_irq_enable(const struct morse_lower_s *lower,
                             bool enable)
{
  /* EXTI1 has a dedicated NVIC vector: this is safe from an ISR */

  if (enable)
    {
      up_enable_irq(HALOW_IRQ_IRQ);
    }
  else
    {
      up_disable_irq(HALOW_IRQ_IRQ);
    }
}

#ifdef CONFIG_WEACT_STM32H743_HALOW_POWERSAVE
static void halow_wake(const struct morse_lower_s *lower, bool assert)
{
  stm32_gpiowrite(GPIO_HALOW_WAKE, assert);
}

static bool halow_busy(const struct morse_lower_s *lower)
{
  return stm32_gpioread(GPIO_HALOW_BUSY);
}

static int halow_busy_attach(const struct morse_lower_s *lower,
                             xcpt_t handler, void *arg)
{
  return stm32_gpiosetevent(GPIO_HALOW_BUSY, true, false, true,
                            handler, arg);
}

static void halow_busy_enable(const struct morse_lower_s *lower,
                              bool enable)
{
  if (enable)
    {
      up_enable_irq(HALOW_BUSY_IRQ);
    }
  else
    {
      up_disable_irq(HALOW_BUSY_IRQ);
    }
}
#endif /* CONFIG_WEACT_STM32H743_HALOW_POWERSAVE */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_morse_initialize
 *
 * Description:
 *   Configure the HaLow module control lines and register the Morse Micro
 *   HaLow network driver.
 *
 ****************************************************************************/

int stm32_morse_initialize(void)
{
  struct spi_dev_s *spi;

  /* RESET_N is held low until the driver boots the chip */

  stm32_configgpio(GPIO_HALOW_RESET);
  stm32_configgpio(GPIO_HALOW_IRQ);
  stm32_configgpio(GPIO_HALOW_WAKE);

#ifdef CONFIG_WEACT_STM32H743_HALOW_POWERSAVE
  stm32_configgpio(GPIO_HALOW_BUSY);
#else
  /* Keep WAKE asserted in case the line is connected anyway */

  stm32_gpiowrite(GPIO_HALOW_WAKE, true);
#endif

  spi = stm32_spibus_initialize(HALOW_SPI_BUS);
  if (spi == NULL)
    {
      wlerr("ERROR: failed to initialize SPI%d\n", HALOW_SPI_BUS);
      return -ENODEV;
    }

  return morse_register(spi, &g_halow_lower);
}
