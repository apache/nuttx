/****************************************************************************
 * boards/arm/kl/freedom-kl25z/src/kl_adxl345.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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
#include <errno.h>

#include <nuttx/spi/spi.h>
#include <nuttx/sensors/adxl345.h>

#include <nuttx/irq.h>

#include "freedom-kl25z.h"
#include "kl_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifdef CONFIG_SENSORS_ADXL345
#ifndef CONFIG_KL_SPI0
#  error "ADXL345 support requires CONFIG_KL_SPI0"
#endif

#ifndef CONFIG_ADXL345_SPI
#  error "Only the ADXL345 SPI interface is supported"
#endif

#ifdef CONFIG_ADXL345_I2C
#  error "Only the ADXL345 SPI interface is supported"
#endif

#ifndef CONFIG_ADXL345_FREQUENCY
#  define CONFIG_ADXL345_FREQUENCY 500000
#endif

#ifndef CONFIG_ADXL345_SPIDEV
#  define CONFIG_ADXL345_SPIDEV 0
#endif

#if CONFIG_ADXL345_SPIDEV != 0
#  error "CONFIG_ADXL345_SPIDEV must be zero"
#endif

#ifndef CONFIG_ADXL345_DEVMINOR
#  define CONFIG_ADXL345_DEVMINOR 0
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct kl_adxl345config_s
{
  /* Configuration structure as seen by the ADXL345 driver */

  struct adxl345_config_s config;

  /* Additional private definitions only known to this driver */

  ADXL345_HANDLE handle;      /* The ADXL345 driver handle */
  adxl345_handler_t handler;  /* The ADXL345 interrupt handler */
  FAR void *arg;              /* Argument to pass to the interrupt handler */
};

/****************************************************************************
 * Static Function Prototypes
 ****************************************************************************/

/* IRQ/GPIO access callbacks.  These operations all hidden behind callbacks
 * to isolate the ADXL345 driver from differences in GPIO interrupt handling
 * by varying boards and MCUs.
 *
 *   attach  - Attach the ADXL345 interrupt handler to the GPIO interrupt
 *   enable  - Enable or disable the GPIO interrupt
 *   clear   - Acknowledge/clear any pending GPIO interrupt
 */

static int  adxl345_attach(FAR struct adxl345_config_s *state,
                           adxl345_handler_t handler, FAR void *arg);
static void adxl345_enable(FAR struct adxl345_config_s *state, bool enable);
static void adxl345_clear(FAR struct adxl345_config_s *state);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the ADXL345
 * driver.  This structure provides information about the configuration
 * of the ADXL345 and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active. The
 * memory must be writable because, under certain circumstances, the driver
 * may modify frequency or X plate resistance values.
 */

static struct kl_adxl345config_s g_adxl345config =
{
  .config =
  {
#ifdef CONFIG_ADXL345_I2C
    .address   = ADXL345_ADDR1,
#endif
    .frequency = CONFIG_ADXL345_FREQUENCY,

    .attach    = adxl345_attach,
    .enable    = adxl345_enable,
    .clear     = adxl345_clear,
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* This is the ADXL345 Interrupt handler */

int adxl345_interrupt(int irq, FAR void *context)
{
  /* Verify that we have a handler attached */

  if (g_adxl345config.handler)
    {
      /* Yes.. forward with interrupt along with its argument */

      g_adxl345config.handler(&g_adxl345config.config, g_adxl345config.arg);
    }

  return OK;
}

/* IRQ/GPIO access callbacks.  These operations all hidden behind
 * callbacks to isolate the ADXL345 driver from differences in GPIO
 * interrupt handling by varying boards and MCUs.
 *
 *   attach  - Attach the ADXL345 interrupt handler to the GPIO interrupt
 *   enable  - Enable or disable the GPIO interrupt
 *   clear   - Acknowledge/clear any pending GPIO interrupt
 */

static int adxl345_attach(FAR struct adxl345_config_s *state,
                           adxl345_handler_t handler, FAR void *arg)
{
  FAR struct kl_adxl345config_s *priv =
      (FAR struct kl_adxl345config_s *)state;

  sninfo("Saving handler %p\n", handler);
  DEBUGASSERT(priv);

  /* Just save the handler and its argument.  We will use it when interrupts
   * are enabled
   */

  priv->handler = handler;
  priv->arg = arg;
  return OK;
}

static void adxl345_enable(FAR struct adxl345_config_s *state, bool enable)
{
  FAR struct kl_adxl345config_s *priv =
     (FAR struct kl_adxl345config_s *)state;
  irqstate_t flags;

  /* Attach and enable, or detach and disable.  Enabling and disabling GPIO
   * interrupts is a multi-step process so the safest thing is to keep
   * interrupts disabled during the reconfiguration.
   */

  flags = enter_critical_section();
  if (enable)
    {
      /* Configure the interrupt using the SAVED handler */

      kl_configgpio(GPIO_ADXL345_INT1);
      kl_gpioirqattach(GPIO_ADXL345_INT1, adxl345_interrupt, NULL);
      kl_gpioirqenable(GPIO_ADXL345_INT1);
    }
  else
    {
      /* Configure the interrupt with a NULL handler to disable it */

      kl_gpioirqattach(GPIO_ADXL345_INT1, NULL, NULL);
      kl_gpioirqdisable(GPIO_ADXL345_INT1);
    }

  leave_critical_section(flags);
}

static void adxl345_clear(FAR struct adxl345_config_s *state)
{
  /* Does nothing */
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: adxl345_archinitialize
 *
 * Description:
 *   Each board that supports an adxl345 device must provide this function.
 *   This function is called by application-specific, setup logic to
 *   configure the accelerometer device.  This function will register the
 *   driver as /dev/accelN where N is the minor device number.
 *
 * Input Parameters:
 *   minor   - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int adxl345_archinitialize(int minor)
{
  FAR struct spi_dev_s *dev;
  int ret;

  sninfo("minor %d\n", minor);
  DEBUGASSERT(minor == 0);

  /* Check if we are already initialized */

  if (!g_adxl345config.handle)
    {
      sninfo("Initializing\n");

      /* Configure the ADXL345 interrupt pin as an input */

      kl_configgpio(GPIO_ADXL345_INT1);

      /* Get an instance of the I2C interface */

      dev = kl_spibus_initialize(CONFIG_ADXL345_SPIDEV);
      if (!dev)
        {
          snerr("ERROR: Failed to initialize SPI bus %d\n",
                CONFIG_ADXL345_SPIDEV);
          return -ENODEV;
        }

      /* Instantiate the ADXL345 driver */

      g_adxl345config.handle =
        adxl345_instantiate(dev, (FAR struct adxl345_config_s *)&g_adxl345config);
      if (!g_adxl345config.handle)
        {
          snerr("ERROR: Failed to instantiate the ADXL345 driver\n");
          return -ENODEV;
        }

      /* Initialize and register the ADXL345 driver */

      ret = adxl345_register(g_adxl345config.handle, CONFIG_ADXL345_DEVMINOR);
      if (ret < 0)
        {
          snerr("ERROR: Failed to register ADXL345 driver: %d\n", ret);
          return ret;
        }
    }

  return OK;
}

#endif /* CONFIG_SENSORS_ADXL345 */
