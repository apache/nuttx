/****************************************************************************
 * boards/arm/stm32/shenzhou/src/stm32_touchscreen.c
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
#include <stdio.h>
#include <debug.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/board.h>
#include <nuttx/irq.h>
#include <nuttx/spi/spi.h>
#include <nuttx/input/touchscreen.h>
#include <nuttx/input/ads7843e.h>

#include "stm32.h"
#include "shenzhou.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifdef CONFIG_INPUT_ADS7843E
#ifndef CONFIG_INPUT
#  error "Touchscreen support requires CONFIG_INPUT"
#endif

#ifndef CONFIG_STM32_SPI3
#  error "Touchscreen support requires CONFIG_STM32_SPI3"
#endif

#ifndef CONFIG_ADS7843E_FREQUENCY
#  define CONFIG_ADS7843E_FREQUENCY 500000
#endif

#ifndef CONFIG_ADS7843E_SPIDEV
#  define CONFIG_ADS7843E_SPIDEV 3
#endif

#if CONFIG_ADS7843E_SPIDEV != 3
#  error "CONFIG_ADS7843E_SPIDEV must be three"
#endif

#ifndef CONFIG_ADS7843E_DEVMINOR
#  define CONFIG_ADS7843E_DEVMINOR 0
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_config_s
{
  struct ads7843e_config_s dev;
  xcpt_t                   handler;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* IRQ/GPIO access callbacks.  These operations all hidden behind
 * callbacks to isolate the ADS7843E driver from differences in GPIO
 * interrupt handling by varying boards and MCUs.  If possible,
 * interrupts should be configured on both rising and falling edges
 * so that contact and loss-of-contact events can be detected.
 *
 * attach  - Attach the ADS7843E interrupt handler to the GPIO interrupt
 * enable  - Enable or disable the GPIO interrupt
 * clear   - Acknowledge/clear any pending GPIO interrupt
 * pendown - Return the state of the pen down GPIO input
 */

static int  tsc_attach(struct ads7843e_config_s *state, xcpt_t isr);
static void tsc_enable(struct ads7843e_config_s *state, bool enable);
static void tsc_clear(struct ads7843e_config_s *state);
static bool tsc_busy(struct ads7843e_config_s *state);
static bool tsc_pendown(struct ads7843e_config_s *state);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the ADS7843E
 * driver.  This structure provides information about the configuration
 * of the ADS7843E and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active. The
 * memory must be writable because, under certain circumstances, the driver
 * may modify frequency or X plate resistance values.
 */

static struct stm32_config_s g_tscinfo =
{
  {
    .frequency = CONFIG_ADS7843E_FREQUENCY,
    .attach    = tsc_attach,
    .enable    = tsc_enable,
    .clear     = tsc_clear,
    .busy      = tsc_busy,
    .pendown   = tsc_pendown,
  },
  .handler     = NULL,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* IRQ/GPIO access callbacks.  These operations all hidden behind
 * callbacks to isolate the ADS7843E driver from differences in GPIO
 * interrupt handling by varying boards and MCUs.  If possible,
 * interrupts should be configured on both rising and falling edges
 * so that contact and loss-of-contact events can be detected.
 *
 * attach  - Attach the ADS7843E interrupt handler to the GPIO interrupt
 * enable  - Enable or disable the GPIO interrupt
 * clear   - Acknowledge/clear any pending GPIO interrupt
 * pendown - Return the state of the pen down GPIO input
 */

static int tsc_attach(struct ads7843e_config_s *state, xcpt_t handler)
{
  struct stm32_config_s *priv = (struct stm32_config_s *)state;

  /* Just save the handler for use when the interrupt is enabled */

  priv->handler = handler;
  return OK;
}

static void tsc_enable(struct ads7843e_config_s *state, bool enable)
{
  struct stm32_config_s *priv = (struct stm32_config_s *)state;

  /* The caller should not attempt to enable interrupts if the handler
   * has not yet been 'attached'
   */

  DEBUGASSERT(priv->handler || !enable);

  /* Attach and enable, or detach and disable */

  iinfo("enable:%d\n", enable);
  if (enable)
    {
      stm32_gpiosetevent(GPIO_TP_INT, true, true, false,
                         priv->handler, NULL);
    }
  else
    {
      stm32_gpiosetevent(GPIO_TP_INT, false, false, false,
                         NULL, NULL);
    }
}

static void tsc_clear(struct ads7843e_config_s *state)
{
  /* Does nothing */
}

static bool tsc_busy(struct ads7843e_config_s *state)
{
  /* Hmmm... The ADS7843E BUSY pin is not brought out on the Shenzhou board.
   * We will most certainly have to revisit this.  There is this cryptic
   * statement in the XPT2046 spec:  "No DCLK delay required with dedicated
   * serial port."
   *
   * The busy state is used by the ADS7843E driver to control the delay
   * between sending the command, then reading the returned data.
   */

  return false;
}

static bool tsc_pendown(struct ads7843e_config_s *state)
{
  /* XPT2046 uses an an internal pullup resistor.  The PENIRQ output goes low
   * due to the current path through the touch screen to ground, which
   * initiates an interrupt to the processor via TP_INT.
   */

  bool pendown = !stm32_gpioread(GPIO_TP_INT);
  iinfo("pendown:%d\n", pendown);
  return pendown;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_tsc_setup
 *
 * Description:
 *   This function is called by board-bringup logic to configure the
 *   touchscreen device.  This function will register the driver as
 *   /dev/inputN where N is the minor device number.
 *
 * Input Parameters:
 *   minor   - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int stm32_tsc_setup(int minor)
{
  struct spi_dev_s *dev;
  int ret;

  iinfo("minor %d\n", minor);
  DEBUGASSERT(minor == 0);

  /* Configure and enable the ADS7843E interrupt pin as an input. */

  stm32_configgpio(GPIO_TP_INT);

  /* Get an instance of the SPI interface */

  dev = stm32_spibus_initialize(CONFIG_ADS7843E_SPIDEV);
  if (!dev)
    {
      ierr("ERROR: Failed to initialize SPI bus %d\n",
           CONFIG_ADS7843E_SPIDEV);
      return -ENODEV;
    }

  /* Initialize and register the SPI touschscreen device */

  ret = ads7843e_register(dev, &g_tscinfo.dev,
                          CONFIG_ADS7843E_DEVMINOR);
  if (ret < 0)
    {
      ierr("ERROR: Failed to initialize SPI bus %d\n",
           CONFIG_ADS7843E_SPIDEV);

      /* up_spiuninitialize(dev); */

      return -ENODEV;
    }

  return OK;
}

#endif /* CONFIG_INPUT_ADS7843E */
