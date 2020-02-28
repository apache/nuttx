/****************************************************************************
 * boards/arm/imxrt/imxrt1060-evk/src/imxrt_ft5x06.c
 *
 *   Copyright 2019 ElFaro LAB S.L. All rights reserved.
 *   Author: Fabio Balzano <fabio@elfarolab.com>
 *
 * Based on boards/arm/lpc54xx/lpcxpresso-lpc54628/src/lpc54_ft5x06.c
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

#include "imxrt_config.h"
#include "imxrt_gpio.h"
#include "imxrt_lpi2c.h"

#include "imxrt1060-evk.h"

#include <arch/board/board.h>

#ifdef CONFIG_INPUT_FT5X06

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FT5X06_FREQUENCY 400000

/****************************************************************************
 * Private Function Ptototypes
 ****************************************************************************/

#ifndef CONFIG_FT5X06_POLLMODE
static int  imxrt_ft5x06_attach(FAR const struct ft5x06_config_s *config,
              xcpt_t isr, FAR void *arg);
static void imxrt_ft5x06_enable(FAR const struct ft5x06_config_s *config,
              bool enable);
static void imxrt_ft5x06_clear(FAR const struct ft5x06_config_s *config);
#endif

static void imxrt_ft5x06_wakeup(FAR const struct ft5x06_config_s *config);
static void imxrt_ft5x06_nreset(FAR const struct ft5x06_config_s *config,
              bool state);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct ft5x06_config_s g_ft5x06_config =
{
  .address   = FT5X06_I2C_ADDRESS,
  .frequency = FT5X06_FREQUENCY,
#ifndef CONFIG_FT5X06_POLLMODE
  .attach    = imxrt_ft5x06_attach,
  .enable    = imxrt_ft5x06_enable,
  .clear     = imxrt_ft5x06_clear,
#endif
  .wakeup    = imxrt_ft5x06_wakeup,
  .nreset    = imxrt_ft5x06_nreset
};

#ifndef CONFIG_FT5X06_POLLMODE
static uint8_t g_ft5x06_irq;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_ft5x06_attach
 *
 * Description:
 *   Attach an FT5X06 interrupt handler to a GPIO interrupt
 *
 ****************************************************************************/

#ifndef CONFIG_FT5X06_POLLMODE
static int imxrt_ft5x06_attach(FAR const struct ft5x06_config_s *config,
                               xcpt_t isr, FAR void *arg)
{
  return irq_attach(g_ft5x06_irq, isr, arg);
}
#endif

/****************************************************************************
 * Name: imxrt_ft5x06_enable
 *
 * Description:
 *   Enable or disable a GPIO interrupt
 *
 ****************************************************************************/

#ifndef CONFIG_FT5X06_POLLMODE
static void imxrt_ft5x06_enable(FAR const struct ft5x06_config_s *config,
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
 * Name: imxrt_ft5x06_clear
 *
 * Description:
 *   Acknowledge/clear any pending GPIO interrupt
 *
 ****************************************************************************/

#ifndef CONFIG_FT5X06_POLLMODE
static void imxrt_ft5x06_clear(FAR const struct ft5x06_config_s *config)
{
  imxrt_gpio_ackedge(g_ft5x06_irq);
}
#endif

/****************************************************************************
 * Name: imxrt_ft5x06_wakeup
 *
 * Description:
 *   Issue WAKE interrupt to FT5X06 to change the FT5X06 from Hibernate to
 *   Active mode.
 *
 ****************************************************************************/

static void imxrt_ft5x06_wakeup(FAR const struct ft5x06_config_s *config)
{
  /* We do not have access to the WAKE pin in the implementation */
}

/****************************************************************************
 * Name: imxrt_ft5x06_nreset
 *
 * Description:
 *   Control the chip reset pin (active low)
 *
 ****************************************************************************/

static void imxrt_ft5x06_nreset(FAR const struct ft5x06_config_s *config,
                                bool nstate)
{
  imxrt_gpio_write(GPIO_FT5X06_CTRSTn, nstate);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_ft5x06_register
 *
 * Description:
 *   Register the FT5X06 touch panel driver
 *
 ****************************************************************************/

int imxrt_ft5x06_register(void)
{
  FAR struct i2c_master_s *i2c;
  int ret;

  /* Initialize CTRSTn pin */

  imxrt_config_gpio(GPIO_FT5X06_CTRSTn);

#ifndef CONFIG_FT5X06_POLLMODE
  int irq;

  /* Initialize GPIO interrupt pin. */

  imxrt_gpio_config(GPIO_FT5X06_INTR);
  irq = imxrt_gpio_irqno(GPIO_FT5X06_INTR);
  DEBUGASSERT(irq > 0 && irq < UINT8_MAX);
  g_ft5x06_irq = (uint8_t)irq;

  /* Make sure that the interrupt is disabled at the NVIC */

  imxrt_gpio_ackedge(irq);
  up_disable_irq(irq);
#endif

  /* Take the FT5X06 out of reset */

  imxrt_gpio_write(GPIO_FT5X06_CTRSTn, true);

  /* The FT5X06 is on LPI2C1.  Get the handle and register the F5x06 device */

  i2c = imxrt_i2cbus_initialize(1);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to get LPI2C1 interface\n");
      return -ENODEV;
    }
  else
    {
      ret = ft5x06_register(i2c, &g_ft5x06_config, 0);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to register FT5X06 driver: %d\n",
                 ret);

          imxrt_gpio_write(GPIO_FT5X06_CTRSTn, false);
          imxrt_i2cbus_uninitialize(i2c);
          return ret;
        }
    }

  return OK;
}

#endif /* CONFIG_INPUT_FT5X06*/
