/****************************************************************************
 * boards/arm/lpc54xx/lpcxpresso-lpc54628/src/lpc54_ft5x06.c
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

#include <syslog.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/input/ft5x06.h>

#include "lpc54_config.h"
#include "lpc54_gpio.h"
#include "lpcxpresso-lpc54628.h"

#ifdef HAVE_FT5x06

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FT5X06_FREQUENCY 400000  /* For now, will boost later */

/****************************************************************************
 * Private Function Ptototypes
 ****************************************************************************/

#ifndef CONFIG_FT5X06_POLLMODE
static int  lpc54_ft5x06_attach(const struct ft5x06_config_s *config,
              xcpt_t isr, void *arg);
static void lpc54_ft5x06_enable(const struct ft5x06_config_s *config,
              bool enable);
static void lpc54_ft5x06_clear(const struct ft5x06_config_s *config);
#endif

static void lpc54_ft5x06_wakeup(const struct ft5x06_config_s *config);
static void lpc54_ft5x06_nreset(const struct ft5x06_config_s *config,
              bool state);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct ft5x06_config_s g_ft5x06_config =
{
  .address   = FT5X06_I2C_ADDRESS,
  .frequency = FT5X06_FREQUENCY,
#ifndef CONFIG_FT5X06_POLLMODE
  .attach    = lpc54_ft5x06_attach,
  .enable    = lpc54_ft5x06_enable,
  .clear     = lpc54_ft5x06_clear,
#endif
  .wakeup    = lpc54_ft5x06_wakeup,
  .nreset    = lpc54_ft5x06_nreset
};

#ifndef CONFIG_FT5X06_POLLMODE
static uint8_t g_ft5x06_irq;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc54_ft5x06_attach
 *
 * Description:
 *   Attach an FT5x06 interrupt handler to a GPIO interrupt
 *
 ****************************************************************************/

#ifndef CONFIG_FT5X06_POLLMODE
static int lpc54_ft5x06_attach(const struct ft5x06_config_s *config,
                               xcpt_t isr, void *arg)
{
  return irq_attach(g_ft5x06_irq, isr, arg);
}
#endif

/****************************************************************************
 * Name: lpc54_ft5x06_enable
 *
 * Description:
 *   Enable or disable a GPIO interrupt
 *
 ****************************************************************************/

#ifndef CONFIG_FT5X06_POLLMODE
static void lpc54_ft5x06_enable(const struct ft5x06_config_s *config,
                                bool enable)
{
  if (enable)
    {
      up_enable_irq(g_ft5x06_irq);
    }
  else
    {
      up_disable_irq(g_ft5x06_irq);
    }
}
#endif

/****************************************************************************
 * Name: lpc54_ft5x06_clear
 *
 * Description:
 *   Acknowledge/clear any pending GPIO interrupt
 *
 ****************************************************************************/

#ifndef CONFIG_FT5X06_POLLMODE
static void lpc54_ft5x06_clear(const struct ft5x06_config_s *config)
{
  lpc54_gpio_ackedge(g_ft5x06_irq);
}
#endif

/****************************************************************************
 * Name: lpc54_ft5x06_wakeup
 *
 * Description:
 *   Issue WAKE interrupt to FT5x06 to change the FT5x06 from Hibernate to
 *   Active mode.
 *
 ****************************************************************************/

static void lpc54_ft5x06_wakeup(const struct ft5x06_config_s *config)
{
  /* We do not have access to the WAKE pin in the implementation */
}

/****************************************************************************
 * Name: lpc54_ft5x06_nreset
 *
 * Description:
 *   Control the chip reset pin (active low)
 *
 ****************************************************************************/

static void lpc54_ft5x06_nreset(const struct ft5x06_config_s *config,
                                bool nstate)
{
  lpc54_gpio_write(GPIO_FT5X06_CTRSTN, nstate);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc54_ft5x06_register
 *
 * Description:
 *   Register the FT5x06 touch panel driver
 *
 ****************************************************************************/

int lpc54_ft5x06_register(void)
{
  struct i2c_master_s *i2c;
  int ret;

#ifndef CONFIG_FT5X06_POLLMODE
  int irq;

  /* Initialize GPIO pins.  NOTE:  The nRST pin was already configured during
   * early LCD initialization.  The Part is in reset now.
   */

  lpc54_gpio_config(GPIO_FT5X06_INTR);
  irq = lpc54_gpio_irqno(GPIO_FT5X06_INTR);
  DEBUGASSERT(irq > 0 && irq < UINT8_MAX);
  g_ft5x06_irq = (uint8_t)irq;

  /* Make sure that the interrupt is disabled at the NVIC */

  lpc54_gpio_ackedge(irq);
  up_disable_irq(irq);
#endif

  /* Take the FT5x06 out of reset */

  lpc54_gpio_write(GPIO_FT5X06_CTRSTN, true);

  /* The FT5x06 is on I2C2.  Get the handle and register the F5x06 device */

  i2c = lpc54_i2c_handle(2, I2C2NDX);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to get I2C2 interface\n");
      return -ENODEV;
    }
  else
    {
      ret = ft5x06_register(i2c, &g_ft5x06_config, 0);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to register FT5x06 driver: %d\n",
                 ret);

          lpc54_gpio_write(GPIO_FT5X06_CTRSTN, false);
          lpc54_i2c_free(I2C2NDX);
          return ret;
        }
    }

  return OK;
}

#endif /* HAVE_FT5x06*/
