/****************************************************************************
 * boards/arm/stm32/viewtools-stm32f107/src/stm32_ft80x.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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
#include <nuttx/lcd/ft80x.h>

#include "arm_arch.h"
#include "stm32_gpio.h"
#include "stm32_spi.h"

#include "viewtool_stm32f107.h"

#if defined(CONFIG_VIEWTOOL_FT80X_SPI1) || defined(CONFIG_VIEWTOOL_FT80X_SPI2)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct viewtool_ft80xlower_s
{
  /* Standard FT80x interface */

  struct ft80x_config_s config;

  /* Extensions for the viewtool board */

  xcpt_t handler;
  FAR void *arg;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* IRQ/GPIO access callbacks.  These operations all hidden behind callbacks
 * to isolate the FT80X driver from differences in GPIO interrupt handling
 * by varying boards and MCUs.
 *
 * Interrupts should be configured on the falling edge of nINT.
 *
 *   attach  - Attach the ADS7843E interrupt handler to the GPIO interrupt
 *   enable  - Enable or disable the GPIO interrupt
 *   clear   - Acknowledge/clear any pending GPIO interrupt as necessary.
 *   pwrdown - Power the FT80X up or down.
 *   audio   - Enable audio (i.e., set the external audio amplifier shutdown
 *             pin to the appropriate level to enable or disable the
 *             external audio amplifier)
 *   destroy - The driver has been unlinked. Cleanup as necessary.
 */

static int  ft80x_attach(FAR const struct ft80x_config_s *lower, xcpt_t isr,
              FAR void *arg);
static void ft80x_enable(FAR const struct ft80x_config_s *lower, bool enable);
static void ft80x_clear(FAR const struct ft80x_config_s *lower);

static void ft80x_pwrdown(FAR const struct ft80x_config_s *lower,
                          bool pwrdown);
#ifdef CONFIG_LCD_FT80X_AUDIO_MCUSHUTDOWN
static void ft80x_audio(FAR const struct ft80x_config_s *lower, bool enable);
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static void ft80x_destroy(FAR const struct ft80x_config_s *lower);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the FT80x
 * driver.  This structure provides information about the configuration
 * of the FT80x and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active. The
 * memory must be writable because, under certain circumstances, the driver
 * may modify certain values.
 */

static struct viewtool_ft80xlower_s g_ft80xlower =
{
  .config =
  {
    .init_frequency = CONFIG_VIEWTOOL_FT80X_INITFREQUENCY,
    .op_frequency   = CONFIG_VIEWTOOL_FT80X_OPFREQUENCY,

    .attach         = ft80x_attach,
    .enable         = ft80x_enable,
    .clear          = ft80x_clear,
    .pwrdown        = ft80x_pwrdown,
#ifdef CONFIG_LCD_FT80X_AUDIO_MCUSHUTDOWN
    .audio          = ft80x_audio,
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
    .destroy        = ft80x_destroy,
#endif
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * IRQ/GPIO access callbacks.  These operations all hidden behind
 * callbacks to isolate the FT80x driver from differences in GPIO
 * interrupt handling by varying boards and MCUs.  If possible,
 * interrupts should be configured on both rising and falling edges
 * so that contact and loss-of-contact events can be detected.
 *
 *   attach  - Attach the ADS7843E interrupt handler to the GPIO interrupt
 *   enable  - Enable or disable the GPIO interrupt
 *   clear   - Acknowledge/clear any pending GPIO interrupt as necessary.
 *   pwrdown - Power the FT80X up or down.
 *   audio   - Enable audio (i.e., set the external audio amplifier shutdown
 *             pin to the appropriate level to enable or disable the
 *             external audio amplifier)
 *   destroy - The driver has been unlinked. Cleanup as necessary.
 *
 ****************************************************************************/

static int ft80x_attach(FAR const struct ft80x_config_s *lower, xcpt_t isr,
                        FAR void *arg)
{
  FAR struct viewtool_ft80xlower_s *priv =
    (FAR struct viewtool_ft80xlower_s *)lower;

  if (isr)
    {
      /* Just save the address of the handler for now.  The new handler will
       * be attached when the interrupt is next enabled.
       */

      iinfo("Attaching %p\n", isr);
      priv->handler = isr;
      priv->arg     = arg;
    }
  else
    {
      iinfo("Detaching %p\n", priv->handler);
      ft80x_enable(lower, false);
      priv->handler = NULL;
      priv->arg     = NULL;
    }

  return OK;
}

static void ft80x_enable(FAR const struct ft80x_config_s *lower,
                         bool enable)
{
  FAR struct viewtool_ft80xlower_s *priv =
    (FAR struct viewtool_ft80xlower_s *)lower;
  irqstate_t flags;

  /* Attach and enable, or detach and disable.  Enabling and disabling GPIO
   * interrupts is a multi-step process so the safest thing is to keep
   * interrupts disabled during the reconfiguration.
   */

  flags = enter_critical_section();
  if (enable && priv->handler)
    {
      /* Configure the EXTI interrupt using the saved handler */

      stm32_gpiosetevent(GPIO_FT80X_INT, true, true, true,
                         priv->handler, priv->arg);
    }
  else
    {
      /* Configure the EXTI interrupt with a NULL handler to disable it.
       *
       * REVISIT:  There is a problem here... interrupts received while
       * the EXTI is de-configured will not pend but will be lost.
       */

     stm32_gpiosetevent(GPIO_FT80X_INT, false, false, false,
                        NULL, NULL);
    }

  leave_critical_section(flags);
}

static void ft80x_clear(FAR const struct ft80x_config_s *lower)
{
  /* Does nothing */
}

static void ft80x_pwrdown(FAR const struct ft80x_config_s *lower,
                          bool pwrdown)
{
  /* Powerdown pin is active low.  Hence, it is really a power up pin. */

  stm32_gpiowrite(GPIO_FT80_PD, !pwrdown);
}

#ifdef CONFIG_LCD_FT80X_AUDIO_MCUSHUTDOWN
static void ft80x_audio(FAR const struct ft80x_config_s *lower, bool enable)
{
  /* Does nothing */
}
#endif

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static void ft80x_destroy(FAR const struct ft80x_config_s *lower)
{
  /* Does nothing */
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_ft80x_setup
 *
 * Description:
 *   This function is called by board-bringup logic to configure the
 *   FT80x GUI device.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int stm32_ft80x_setup(void)
{
  FAR struct spi_dev_s *spi;
  int ret;

  /* Configure the FT80x interrupt pin as an input and powerdown pin as an
   * output.  Device is initially powered down.
   */

  stm32_configgpio(GPIO_FT80X_INT);
  stm32_configgpio(GPIO_FT80_PD);

  /* Get an instance of the SPI interface for the touchscreen chip select */

  spi = stm32_spibus_initialize(FT80X_SPIBUS);
  if (!spi)
    {
      lcderr("ERROR: Failed to initialize SPI%d\n", FT80X_SPIBUS);
      return -ENODEV;
    }

  /* Initialize and register the SPI touchscreen device */

  ret = ft80x_register(spi, &g_ft80xlower.config);
  if (ret < 0)
    {
      lcderr("ERROR: Failed to register touchscreen device\n");
      /* up_spiuninitialize(spi); */
      return -ENODEV;
    }

  return OK;
}

#endif /* CONFIG_VIEWTOOL_FT80X_SPI1 || CONFIG_VIEWTOOL_FT80X_SPI2 */
