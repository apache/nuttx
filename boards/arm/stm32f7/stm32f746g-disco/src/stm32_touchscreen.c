/****************************************************************************
 * boards/arm/stm32f7/stm32f746g-disco/src/stm32_touchscreen.c
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

#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/input/ft5x06.h>

#include "stm32_gpio.h"
#include "stm32_i2c.h"

#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FT5X06_FREQUENCY 100000  /* For now, will boost later */

#ifdef CONFIG_INPUT_FT5X06
#ifndef CONFIG_INPUT
#  error "FT5x06 support requires CONFIG_INPUT"
#endif

#ifndef CONFIG_STM32F7_I2C3
#  error "FT5x06 support requires CONFIG_STM32F7_I2C3"
#endif

#ifndef CONFIG_FT5X06_I2CDEV
#  define CONFIG_FT5X06_I2CDEV 3
#endif

#if CONFIG_FT5X06_I2CDEV != 3
#  error "CONFIG_FT5X06_I2CDEV must be three"
#endif

#ifndef CONFIG_FT5X06_DEVMINOR
#  define CONFIG_FT5X06_DEVMINOR 0
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_ft5x06_config_s
{
  xcpt_t  handler;  /* The FT5x06 interrupt handler */
  void   *arg;      /* Interrupt handler argument */
};

/****************************************************************************
 * Private Function Ptototypes
 ****************************************************************************/

#ifndef CONFIG_FT5X06_POLLMODE
static int  stm32_ft5x06_attach(const struct ft5x06_config_s *config,
                                xcpt_t isr, void *arg);
static void stm32_ft5x06_enable(const struct ft5x06_config_s *config,
                                bool enable);
static void stm32_ft5x06_clear(const struct ft5x06_config_s *config);
#endif
static void stm32_ft5x06_wakeup(const struct ft5x06_config_s *config);
static void stm32_ft5x06_nreset(const struct ft5x06_config_s *config,
                                bool state);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct ft5x06_config_s g_ft5x06_config =
{
  .address   = FT5X06_I2C_ADDRESS,
  .frequency = FT5X06_FREQUENCY,
#ifndef CONFIG_FT5X06_POLLMODE
  .attach    = stm32_ft5x06_attach,
  .enable    = stm32_ft5x06_enable,
  .clear     = stm32_ft5x06_clear,
#endif
  .wakeup    = stm32_ft5x06_wakeup,
  .nreset    = stm32_ft5x06_nreset
};

static struct stm32_ft5x06_config_s g_priv_config =
{
  .handler     = NULL,
  .arg         = NULL,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_ft5x06_attach
 *
 * Description:
 *   Attach an FT5x06 interrupt handler to a GPIO interrupt
 *
 ****************************************************************************/

#ifndef CONFIG_FT5X06_POLLMODE
static int  stm32_ft5x06_attach(const struct ft5x06_config_s *config,
                                xcpt_t isr, void *arg)
{
  iinfo("Saving handler %p\n", isr);

  /* Just save the handler.  We will use it when EXTI interruptsare enabled */

  if (isr)
    {
      /* Just save the address of the handler for now.  The new handler will
       * be attached when the interrupt is next enabled.
       */

      iinfo("Attaching %p\n", isr);
      g_priv_config.handler = isr;
      g_priv_config.arg     = arg;
    }
  else
    {
      iinfo("Detaching %p\n", g_priv_config.handler);
      stm32_ft5x06_enable(config, false);
      g_priv_config.handler = NULL;
      g_priv_config.arg     = NULL;
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: stm32_ft5x06_enable
 *
 * Description:
 *   Enable or disable a GPIO interrupt
 *
 ****************************************************************************/

#ifndef CONFIG_FT5X06_POLLMODE
static void stm32_ft5x06_enable(const struct ft5x06_config_s *config,
                                bool enable)
{
  irqstate_t flags;

  /* Attach and enable, or detach and disable.  Enabling and disabling GPIO
   * interrupts is a multi-step process so the safest thing is to keep
   * interrupts disabled during the reconfiguration.
   */

  flags = enter_critical_section();
  if (enable)
    {
      /* Configure the EXTI interrupt using the SAVED handler */

      stm32_gpiosetevent(GPIO_TP_INT, true, false, true,
                         g_priv_config.handler, g_priv_config.arg);
    }
  else
    {
      /* Configure the EXTI interrupt with a NULL handler to disable it */

     stm32_gpiosetevent(GPIO_TP_INT, false, false, false,
                        NULL, NULL);
    }

  leave_critical_section(flags);
}
#endif

/****************************************************************************
 * Name: stm32_ft5x06_clear
 *
 * Description:
 *   Acknowledge/clear any pending GPIO interrupt
 *
 ****************************************************************************/

#ifndef CONFIG_FT5X06_POLLMODE
static void stm32_ft5x06_clear(const struct ft5x06_config_s *config)
{
  /* Does nothing */
}
#endif

/****************************************************************************
 * Name: stm32_ft5x06_wakeup
 *
 * Description:
 *   Issue WAKE interrupt to FT5x06 to change the FT5x06 from Hibernate to
 *   Active mode.
 *
 ****************************************************************************/

static void stm32_ft5x06_wakeup(const struct ft5x06_config_s *config)
{
  /* We do not have access to the WAKE pin in the implementation */
}

/****************************************************************************
 * Name: stm32_ft5x06_nreset
 *
 * Description:
 *   Control the chip reset pin
 *
 ****************************************************************************/

static void stm32_ft5x06_nreset(const struct ft5x06_config_s *config,
                                bool nstate)
{
  /* We do not have access to the RESET pin in the implementation */
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
  struct i2c_master_s *dev;
  int ret;

  iinfo("minor %d\n", minor);
  DEBUGASSERT(minor == CONFIG_FT5X06_DEVMINOR);

  /* Check if we are already initialized */

  iinfo("Initializing\n");

  /* Configure the FT5X06 interrupt pin as an input */

  stm32_configgpio(GPIO_TP_INT);

  /* Get an instance of the I2C interface */

  dev = stm32_i2cbus_initialize(CONFIG_FT5X06_I2CDEV);
  if (!dev)
    {
      ierr("ERROR: Failed to initialize I2C bus %d\n", CONFIG_FT5X06_I2CDEV);
      return -ENODEV;
    }

  /* Initialize and register the I2C touchscreen device */

  ret = ft5x06_register(dev, &g_ft5x06_config, CONFIG_FT5X06_DEVMINOR);
  if (ret < 0)
    {
      ierr("ERROR: Failed to register FT5x06 driver: %d\n", ret);
      stm32_i2cbus_uninitialize(dev);
      return ret;
    }

  return OK;
}

#endif /* CONFIG_INPUT_FT5X06 */
