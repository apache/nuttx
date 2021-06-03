/****************************************************************************
 * boards/arm/stm32/common/src/stm32_xen1210.c
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
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/spi/spi.h>
#include <nuttx/sensors/xen1210.h>

#include <nuttx/timers/pwm.h>
#include <arch/board/board.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/fs/ioctl.h>

#include "stm32_gpio.h"
#include "stm32_spi.h"
#include "stm32_pwm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_xen1210config_s
{
  /* Configuration structure as seen by the XEN1210 driver */

  struct xen1210_config_s config;

  /* Additional private definitions only known to this driver */

  XEN1210_HANDLE handle;      /* The XEN1210 driver handle */
  xen1210_handler_t handler;  /* The XEN1210 interrupt handler */
  FAR void *arg;              /* Argument to pass to the interrupt handler */
};

/****************************************************************************
 * Static Function Prototypes
 ****************************************************************************/

/* IRQ/GPIO access callbacks.  These operations all hidden behind callbacks
 * to isolate the XEN1210 driver from differences in GPIO interrupt handling
 * by varying boards and MCUs.
 *
 *   attach  - Attach the XEN1210 interrupt handler to the GPIO interrupt
 *   enable  - Enable or disable the GPIO interrupt
 *   clear   - Acknowledge/clear any pending GPIO interrupt
 */

static int  xen1210_attach(FAR struct xen1210_config_s *state,
                           xen1210_handler_t handler, FAR void *arg);
static void xen1210_enable(FAR struct xen1210_config_s *state, bool enable);
static void xen1210_clear(FAR struct xen1210_config_s *state);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the XEN1210
 * driver.  This structure provides information about the configuration
 * of the XEN1210 and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active. The
 * memory must be writable because, under certain circumstances, the driver
 * may modify frequency or X plate resistance values.
 */

static struct stm32_xen1210config_s g_xen1210config =
{
  .config =
  {
    .frequency = XEN1210_SPI_MAXFREQUENCY,
    .attach    = xen1210_attach,
    .enable    = xen1210_enable,
    .clear     = xen1210_clear,
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* This is the XEN1210 Interrupt handler */

static int xen1210_interrupt(int irq, FAR void *context, FAR void *arg)
{
  /* Verify that we have a handler attached */

  if (g_xen1210config.handler)
    {
      /* Yes.. forward with interrupt along with its argument */

      g_xen1210config.handler(&g_xen1210config.config, g_xen1210config.arg);
    }

  return OK;
}

/* IRQ/GPIO access callbacks.  These operations all hidden behind
 * callbacks to isolate the XEN1210 driver from differences in GPIO
 * interrupt handling by varying boards and MCUs.
 *
 *   attach  - Attach the XEN1210 interrupt handler to the GPIO interrupt
 *   enable  - Enable or disable the GPIO interrupt
 *   clear   - Acknowledge/clear any pending GPIO interrupt
 */

static int xen1210_attach(FAR struct xen1210_config_s *state,
                           xen1210_handler_t handler, FAR void *arg)
{
  FAR struct stm32_xen1210config_s *priv =
    (FAR struct stm32_xen1210config_s *)state;

  sninfo("Saving handler %p\n", handler);
  DEBUGASSERT(priv);

  /* Just save the handler and its argument.  We will use it when interrupts
   * are enabled
   */

  priv->handler = handler;
  priv->arg = arg;
  return OK;
}

static void xen1210_enable(FAR struct xen1210_config_s *state, bool enable)
{
  irqstate_t flags;

  /* Attach and enable, or detach and disable.  Enabling and disabling GPIO
   * interrupts is a multi-step process so the safest thing is to keep
   * interrupts disabled during the reconfiguration.
   */

  flags = enter_critical_section();
  if (enable)
    {
      /* Configure the interrupt using the SAVED handler */

      stm32_configgpio(BOARD_XEN1210_GPIO_INT);
      stm32_gpiosetevent(BOARD_XEN1210_GPIO_INT, false, true,
                         true, xen1210_interrupt, NULL);
    }
  else
    {
      /* Configure the interrupt with a NULL handler to disable it */

      stm32_gpiosetevent(BOARD_XEN1210_GPIO_INT, false, false, false,
                         NULL, NULL);
    }

  leave_critical_section(flags);
}

static void xen1210_clear(FAR struct xen1210_config_s *state)
{
  /* Does nothing */
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xen1210_pwm_setup
 *
 * Description:
 *   All STM32 architectures must provide the following interface to
 *   work with examples/pwm.
 *
 ****************************************************************************/

static int xen1210_pwm_setup(void)
{
  static bool initialized = false;
  struct pwm_lowerhalf_s *pwm;
  struct pwm_info_s info;

  /* Have we already initialized? */

  if (!initialized)
    {
      /* Call stm32_pwminitialize() to get an instance of the PWM interface */

      pwm = stm32_pwminitialize(BOARD_XEN1210_PWMTIMER);
      if (!pwm)
        {
          _err("ERROR: Failed to get the STM32 PWM lower half\n");
          return -ENODEV;
        }

      /* Define frequency and duty cycle: 2MHz @ 50% */

      info.frequency = 2000000; /* 2MHz */
      info.duty = 32768;        /* This value means 50% */

      /* Initialize PWM */

      pwm->ops->setup(pwm);
      pwm->ops->start(pwm, &info);

      /* Now we are initialized */

      initialized = true;
    }

  return 0;
}

/****************************************************************************
 * Name: xen1210_archinitialize
 *
 * Description:
 *   Initialize the XEN1210 device driver
 *
 * Input Parameters:
 *   devno - The device number, used to build the device path as /dev/accelN
 *   busno - The SPI bus number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int board_xen1210_initialize(int devno, int busno)
{
  FAR struct spi_dev_s *dev;
  int ret;

  /* Check if we are already initialized */

  if (!g_xen1210config.handle)
    {
      sninfo("Initializing\n");

      /* XEN1210 needs an external clock 1-4MHz
       * We are using PWM on TIM1 CH1 to do it!
       */

      xen1210_pwm_setup();

      /* Configure the XEN1210 interrupt pin as an input */

      stm32_configgpio(BOARD_XEN1210_GPIO_INT);

      /* Get an instance of the I2C interface */

      dev = stm32_spibus_initialize(busno);
      if (!dev)
        {
          snerr("ERROR: Failed to initialize SPI bus %d\n", busno);
          return -ENODEV;
        }

      /* Instantiate the XEN1210 driver */

      g_xen1210config.handle =
          xen1210_instantiate(dev,
                              (FAR struct xen1210_config_s *)
                              &g_xen1210config);
      if (!g_xen1210config.handle)
        {
          snerr("ERROR: Failed to instantiate the XEN1210 driver\n");
          return -ENODEV;
        }

      /* Initialize and register the XEN1210 driver */

      ret = xen1210_register(g_xen1210config.handle, devno);
      if (ret < 0)
        {
          snerr("ERROR: Failed to register XEN1210 driver: %d\n", ret);
          return ret;
        }
    }

  return OK;
}
