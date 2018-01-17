/****************************************************************************
 * configs/lpcxpresso-lpc54628/src/lpc54_ft5x06.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

#include <syslog.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/input/ft5x06.h>

#include "lpc54_config.h"
#include "lpc54_gpio.h"
#include "lpcxpresso-lpc54628.h"

#ifdef HAVE_FT5x06

/****************************************************************************
 * Pre-processor Defintions
 ****************************************************************************/

#define FT5x06_FREQUENCY 400000  /* For now, will boost later */

/****************************************************************************
 * Private Function Ptototypes
 ****************************************************************************/

#ifndef CONFIG_FT5X06_POLLMODE
static int  lpc54_ft5x06_attach(FAR const struct ft5x06_config_s *config,
              xcpt_t isr, FAR void *arg);
static void lpc54_ft5x06_enable(FAR const struct ft5x06_config_s *config,
              bool enable);
static void lpc54_ft5x06_clear(FAR const struct ft5x06_config_s *config);
#endif

static void lpc54_ft5x06_wakeup(FAR const struct ft5x06_config_s *config);
static void lpc54_ft5x06_nreset(FAR const struct ft5x06_config_s *config,
              bool state);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct ft5x06_config_s g_ft5x06_config =
{
  .address   = FT5x06_I2C_ADDRESS,
  .frequency = FT5x06_FREQUENCY,
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
static int lpc54_ft5x06_attach(FAR const struct ft5x06_config_s *config,
                               xcpt_t isr, FAR void *arg)
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
static void lpc54_ft5x06_enable(FAR const struct ft5x06_config_s *config,
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
static void lpc54_ft5x06_clear(FAR const struct ft5x06_config_s *config)
{
  (void)lpc54_gpio_ackedge(g_ft5x06_irq);
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

static void lpc54_ft5x06_wakeup(FAR const struct ft5x06_config_s *config)
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

static void lpc54_ft5x06_nreset(FAR const struct ft5x06_config_s *config,
                                bool nstate)
{
  lpc54_gpio_write(GPIO_FT5x06_CTRSTn, nstate);
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
  FAR struct i2c_master_s *i2c;
  int ret;

#ifndef CONFIG_FT5X06_POLLMODE
  int irq;

  /* Initialize GPIO pins.  NOTE:  The nRST pin was already configured during
   * early LCD initialization.  The Part is in reset now.
   */

  lpc54_gpio_config(GPIO_FT5x06_INTR);
  irq = lpc54_gpio_irqno(GPIO_FT5x06_INTR);
  DEBUGASSERT(irq > 0 && irq < UINT8_MAX);
  g_ft5x06_irq = (uint8_t)irq;

  /* Make sure that the interrupt is disabled at the NVIC */

  lpc54_gpio_ackedge(irq);
  up_disable_irq(irq);
#endif

  /* Take the FT5x06 out of reset */

  lpc54_gpio_write(GPIO_FT5x06_CTRSTn, true);

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

          lpc54_gpio_write(GPIO_FT5x06_CTRSTn, false);
          lpc54_i2c_free(I2C2NDX);
          return ret;
        }
    }

  return OK;
}

#endif /* HAVE_FT5x06*/
