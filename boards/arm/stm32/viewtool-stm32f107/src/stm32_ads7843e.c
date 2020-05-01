/****************************************************************************
 * boards/arm/stm32/viewtools-stm32f107/src/stm32_ads7843e.c
 *
 *   Copyright (C) 2014, 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

#include <nuttx/irq.h>
#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/input/touchscreen.h>
#include <nuttx/input/ads7843e.h>

#include "arm_arch.h"
#include "stm32_gpio.h"
#include "stm32_spi.h"

#include "viewtool_stm32f107.h"

#ifdef CONFIG_INPUT_ADS7843E

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifndef CONFIG_STM32_SPI2
#  error CONFIG_STM32_SPI2 is required for touchscreen support
#endif

#ifndef CONFIG_ADS7843E_FREQUENCY
#  define CONFIG_ADS7843E_FREQUENCY 500000
#endif

#ifndef CONFIG_ADS7843E_DEVMINOR
#  define CONFIG_ADS7843E_DEVMINOR 0
#endif

/* The touchscreen communicates on SPI2 */

#define TSC_DEVNUM 2

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct viewtool_tscinfo_s
{
  /* Standard ADS7843/XTP2046 interface */

  struct ads7843e_config_s config;

  /* Extensions for the viewtool board */

  xcpt_t handler;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* IRQ/GPIO access callbacks.  These operations all hidden behind
 * callbacks to isolate the XPT2046 driver from differences in GPIO
 * interrupt handling by varying boards and MCUs.  If possible,
 * interrupts should be configured on both rising and falling edges
 * so that contact and loss-of-contact events can be detected.
 *
 *   attach  - Attach the XPT2046 interrupt handler to the GPIO interrupt
 *   enable  - Enable or disable the GPIO interrupt
 *   clear   - Acknowledge/clear any pending GPIO interrupt
 *   pendown - Return the state of the pen down GPIO input
 */

static int  tsc_attach(FAR struct ads7843e_config_s *state, xcpt_t isr);
static void tsc_enable(FAR struct ads7843e_config_s *state, bool enable);
static void tsc_clear(FAR struct ads7843e_config_s *state);
static bool tsc_busy(FAR struct ads7843e_config_s *state);
static bool tsc_pendown(FAR struct ads7843e_config_s *state);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the XPT2046
 * driver.  This structure provides information about the configuration
 * of the XPT2046 and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active. The
 * memory must be writable because, under certain circumstances, the driver
 * may modify certain values.
 */

static struct viewtool_tscinfo_s g_tscinfo =
{
  .config =
  {
    .frequency = CONFIG_ADS7843E_FREQUENCY,

    .attach    = tsc_attach,
    .enable    = tsc_enable,
    .clear     = tsc_clear,
    .busy      = tsc_busy,
    .pendown   = tsc_pendown,
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * IRQ/GPIO access callbacks.  These operations all hidden behind
 * callbacks to isolate the XPT2046 driver from differences in GPIO
 * interrupt handling by varying boards and MCUs.  If possible,
 * interrupts should be configured on both rising and falling edges
 * so that contact and loss-of-contact events can be detected.
 *
 *   attach  - Attach the XPT2046 interrupt handler to the GPIO interrupt
 *   enable  - Enable or disable the GPIO interrupt
 *   clear   - Acknowledge/clear any pending GPIO interrupt
 *   pendown - Return the state of the pen down GPIO input
 *
 ****************************************************************************/

static int tsc_attach(FAR struct ads7843e_config_s *state, xcpt_t isr)
{
  FAR struct viewtool_tscinfo_s *priv = (FAR struct viewtool_tscinfo_s *)state;

  if (isr)
    {
      /* Just save the address of the handler for now.  The new handler will
       * be attached when the interrupt is next enabled.
       */

      iinfo("Attaching %p\n", isr);
      priv->handler = isr;
    }
  else
    {
      iinfo("Detaching %p\n", priv->handler);
      tsc_enable(state, false);
      priv->handler = NULL;
    }

  return OK;
}

static void tsc_enable(FAR struct ads7843e_config_s *state, bool enable)
{
  FAR struct viewtool_tscinfo_s *priv = (FAR struct viewtool_tscinfo_s *)state;
  irqstate_t flags;

  /* Attach and enable, or detach and disable.  Enabling and disabling GPIO
   * interrupts is a multi-step process so the safest thing is to keep
   * interrupts disabled during the reconfiguration.
   */

  flags = enter_critical_section();
  if (enable && priv->handler)
    {
      /* Configure the EXTI interrupt using the saved handler */

      stm32_gpiosetevent(GPIO_LCDTP_IRQ, true, true, true,
                         priv->handler, NULL);
    }
  else
    {
      /* Configure the EXTI interrupt with a NULL handler to disable it.
       *
       * REVISIT:  There is a problem here... interrupts received while
       * the EXTI is de-configured will not pend but will be lost.
       */

     stm32_gpiosetevent(GPIO_LCDTP_IRQ, false, false, false,
                        NULL, NULL);
    }

  leave_critical_section(flags);
}

static void tsc_clear(FAR struct ads7843e_config_s *state)
{
  /* Does nothing */
}

static bool tsc_busy(FAR struct ads7843e_config_s *state)
{
  return false; /* The BUSY signal is not connected */
}

static bool tsc_pendown(FAR struct ads7843e_config_s *state)
{
  /* The /PENIRQ value is active low */

  bool pendown = !stm32_gpioread(GPIO_LCDTP_IRQ);
  iinfo("pendown=%d\n", pendown);
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
  FAR struct spi_dev_s *dev;
  int ret;

  iinfo("minor %d\n", minor);
  DEBUGASSERT(minor == 0);

  /* Configure the XPT2046 interrupt pin as an input */

  stm32_configgpio(GPIO_LCDTP_IRQ);

  /* Get an instance of the SPI interface for the touchscreen chip select */

  dev = stm32_spibus_initialize(TSC_DEVNUM);
  if (!dev)
    {
      ierr("ERROR: Failed to initialize SPI%d\n", TSC_DEVNUM);
      return -ENODEV;
    }

  /* Initialize and register the SPI touchscreen device */

  ret = ads7843e_register(dev, &g_tscinfo.config, CONFIG_ADS7843E_DEVMINOR);
  if (ret < 0)
    {
      ierr("ERROR: Failed to register touchscreen device\n");
      /* up_spiuninitialize(dev); */
      return -ENODEV;
    }

  return OK;
}

#endif /* CONFIG_INPUT_ADS7843E */
