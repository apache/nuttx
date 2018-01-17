/************************************************************************************
 * configs/open1788/src/lpc17_touchscreen.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/spi/spi.h>
#include <nuttx/input/touchscreen.h>
#include <nuttx/input/ads7843e.h>

#include "lpc17_gpio.h"
#include "lpc17_ssp.h"
#include "open1788.h"

#ifdef CONFIG_INPUT_ADS7843E

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_INPUT
#  error "Touchscreen support requires CONFIG_INPUT"
#endif

#ifndef CONFIG_LPC17_SSP1
#  error "Touchscreen support requires CONFIG_LPC17_SSP1"
#endif

#ifndef CONFIG_LPC17_GPIOIRQ
#  error "Touchscreen support requires CONFIG_LPC17_GPIOIRQ"
#endif

#ifndef CONFIG_ADS7843E_FREQUENCY
#  define CONFIG_ADS7843E_FREQUENCY 500000
#endif

#ifndef CONFIG_ADS7843E_SPIDEV
#  define CONFIG_ADS7843E_SPIDEV 1
#endif

#if CONFIG_ADS7843E_SPIDEV != 1
#  error "CONFIG_ADS7843E_SPIDEV must be one"
#endif

#ifndef CONFIG_ADS7843E_DEVMINOR
#  define CONFIG_ADS7843E_DEVMINOR 0
#endif

/* REVISIT:  Currently, XPT2046 reports BUSY all of the time.  This is
 * probably GPIO setting issues.  But there is this cryptic statement in
 * the XPT2046 spec:  "No DCLK delay required with dedicated serial port."
 *
 * The busy state is used by the XPT2046 driver to control the delay
 * between sending the command, then reading the returned data.
 */

#define XPT2046_NO_BUSY 1

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* IRQ/GPIO access callbacks.  These operations all hidden behind
 * callbacks to isolate the XPT2046 driver from differences in GPIO
 * interrupt handling by varying boards and MCUs.  If possible,
 * interrupts should be configured on both rising and falling edges
 * so that contact and loss-of-contact events can be detected.
 *
 * attach  - Attach the XPT2046 interrupt handler to the GPIO interrupt
 * enable  - Enable or disable the GPIO interrupt
 * clear   - Acknowledge/clear any pending GPIO interrupt
 * pendown - Return the state of the pen down GPIO input
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
 * by the driver and is presumed to persist while the driver is active.
 */

static struct ads7843e_config_s g_tscinfo =
{
  .frequency = CONFIG_ADS7843E_FREQUENCY,
  .attach    = tsc_attach,
  .enable    = tsc_enable,
  .clear     = tsc_clear,
  .busy      = tsc_busy,
  .pendown   = tsc_pendown,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* IRQ/GPIO access callbacks.  These operations all hidden behind
 * callbacks to isolate the XPT2046 driver from differences in GPIO
 * interrupt handling by varying boards and MCUs.  If possible,
 * interrupts should be configured on both rising and falling edges
 * so that contact and loss-of-contact events can be detected.
 *
 * attach  - Attach the XPT2046 interrupt handler to the GPIO interrupt
 * enable  - Enable or disable the GPIO interrupt
 * clear   - Acknowledge/clear any pending GPIO interrupt
 * pendown - Return the state of the pen down GPIO input
 */

static int tsc_attach(FAR struct ads7843e_config_s *state, xcpt_t handler)
{
  /* Attach then enable the touchscreen interrupt handler */

  (void)irq_attach(LPC17_IRQ_PENIRQ, handler, NULL);
  return OK;
}

static void tsc_enable(FAR struct ads7843e_config_s *state, bool enable)
{
  iinfo("enable:%d\n", enable);
  if (enable)
    {
      /* Enable PENIRQ interrupts.  NOTE: The pin interrupt is enabled from worker thread
       * logic after completion of processing of the touchscreen interrupt.
       */

      up_enable_irq(LPC17_IRQ_PENIRQ);
    }
  else
    {
      /* Disable PENIRQ interrupts.  NOTE: The PENIRQ interrupt will be disabled from
       * interrupt handling logic.
       */

      up_disable_irq(LPC17_IRQ_PENIRQ);
    }
}

static void tsc_clear(FAR struct ads7843e_config_s *state)
{
  /* Does nothing.  The interrupt is cleared automatically in the GPIO
   * logic for the LPC17xx family.
   */
}

static bool tsc_busy(FAR struct ads7843e_config_s *state)
{
/* The busy state is used by the XPT2046 driver to control the delay
 * between sending the command, then reading the returned data.
 */

#ifdef XPT2046_NO_BUSY
/* REVISIT:  Currently, XPT2046 reports BUSY all of the time.  This is
 * probably GPIO setting issues.  But there is this cryptic statement in
 * the XPT2046 spec:  "No DCLK delay required with dedicated serial port."
 */

  return false;

#else /* XPT2046_NO_BUSY */

#if defined(CONFIG_DEBUG_INPUT) && defined(CONFIG_DEBUG_INFO)
  static bool last = (bool)-1;
#endif

  /* REVISIT:  This might need to be inverted */

  bool busy = lpc17_gpioread(GPIO_TC_BUSY);
#if defined(CONFIG_DEBUG_INPUT) && defined(CONFIG_DEBUG_INFO)
  if (busy != last)
    {
      iinfo("busy:%d\n", busy);
      last = busy;
    }
#endif

  return busy;

#endif /* XPT2046_NO_BUSY */
}

static bool tsc_pendown(FAR struct ads7843e_config_s *state)
{
  /* XPT2046 uses an an internal pullup resistor.  The PENIRQ output goes low
   * due to the current path through the touch screen to ground, which
   * initiates an interrupt to the processor via TP_INT.
   */

  bool pendown = !lpc17_gpioread(GPIO_TC_PENIRQ);
  iinfo("pendown:%d\n", pendown);
  return pendown;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: open1788_tsc_setup
 *
 * Description:
 *   This function is called by board-bringup logic to configure the
 *   touchscreen device.  This function will register the driver as
 *   /dev/inputN where N is the minor device number.
 *
 * Input Parameters:
 *   minor - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int open1788_tsc_setup(int minor)
{
  FAR struct spi_dev_s *dev;
  int ret;

  iinfo("minor:%d\n", minor);
  DEBUGASSERT(minor == 0);

  /* Configure and enable the XPT2046 PENIRQ pin as an interrupting input. */

  (void)lpc17_configgpio(GPIO_TC_PENIRQ);

  /* Configure the XPT2046 BUSY pin as a normal input. */

#ifndef XPT2046_NO_BUSY
  (void)lpc17_configgpio(GPIO_TC_BUSY);
#endif

  /* Get an instance of the SPI interface */

  dev = lpc17_sspbus_initialize(CONFIG_ADS7843E_SPIDEV);
  if (!dev)
    {
      ierr("ERROR: Failed to initialize SPI bus %d\n", CONFIG_ADS7843E_SPIDEV);
      return -ENODEV;
    }

  /* Initialize and register the SPI touchscreen device */

  ret = ads7843e_register(dev, &g_tscinfo, CONFIG_ADS7843E_DEVMINOR);
  if (ret < 0)
    {
      ierr("ERROR: Failed to register touchscreen device minor=%d\n",
           CONFIG_ADS7843E_DEVMINOR);
      /* up_spiuninitialize(dev); */
      return -ENODEV;
    }

  return OK;
}

#endif /* CONFIG_INPUT_ADS7843E */
