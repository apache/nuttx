/************************************************************************************
 * configs/sam3u-ek/src/up_touchscreen.c
 * arch/arm/src/board/up_touchscreen.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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
#include <errno.h>

#include <nuttx/sdio.h>
#include <nuttx/input/touchscreen.h>
#include <nuttx/input/ads7843e.h>

#include "sam3u_internal.h"
#include "sam3uek_internal.h.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Static Function Prototypes
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

static int  tsc_attach(FAR struct ads7843e_config_s *state, xcpt_t isr);
static void tsc_enable(FAR struct ads7843e_config_s *state, bool enable);
static void tsc_clear(FAR struct ads7843e_config_s *state);
static bool tsc_pendown(FAR struct ads7843e_config_s *state);

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

static struct ads7843e_config_s g_tscinfo =
{
  .calib     = CONFIG_INPUT_TSCCALIB,
  .frequency = CONFIG_INPUT_TSCFREQUENCY,

  .attach    = tsc_attach,
  .enable    = tsc_enable,
  .clear     = tsc_clear,
  .pendown   = tsc_pendown,
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

static int tsc_attach(FAR struct ads7843e_config_s *state, xcpt_t isr)
{
#warning "Missing logic"
  return OK;
}

static void tsc_enable(FAR struct ads7843e_config_s *state, bool enable)
{
  /* Attach and enable, or detach and disable */

  if (enable && g_tschandler)
    {
      /* Configure and enable the ADS7843E interrupt */
#warning "Missing logic"
    }
  else
    {
#warning "Missing logic"
    }
}

static void tsc_clear(FAR struct ads7843e_config_s *state)
{
  /* Does nothing */
}

static bool tsc_pendown(FAR struct ads7843e_config_s *state)
{
  /* REVISIT:  This might need to be inverted */

  return sam3u_gpioread(GPIO_ADS7843E);
}
#endif /* HAVE_TOUCHSCREEN */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_tcinitialize
 *
 * Description:
 *   Initialize the touchscreen device
 *
 ****************************************************************************/

int up_tcinitialize(void)
{
  FAR struct spi_dev_s *dev;
  int ret;

  /* Configure and enable the ADS7843E interrupt pin as an input */

  (void)sam3u_configgpio(GPIO_ADS7843E_BUY);
  (void)sam3u_configgpio(GPIO_ADS7843E_IRQ);

  /* Get an instance of the SPI interface */

  dev = up_spiinitialize(CONFIG_INPUT_TSCSPIDEV);
  if (!dev)
    {
      dbg("Failed to initialize SPI bus %d\n", CONFIG_INPUT_TSCSPIDEV);
      return -ENODEV;
    }

  /* Initialize and register the SPI touschscreen device */

  ret = ads7843e_register(dev, &g_tscinfo, CONFIG_INPUT_TSCMINOR);
  if (ret < 0)
    {
      dbg("Failed to initialize SPI bus %d\n", CONFIG_INPUT_TSCSPIDEV);
      /* up_spiuninitialize(dev); */
      return -ENODEV;
    }

  return OK;
}
