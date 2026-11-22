/****************************************************************************
 * boards/arm/common/stm32/src/stm32_led1642gw.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <errno.h>
#include <nuttx/debug.h>
#include <stdio.h>
#include <string.h>

#include <nuttx/kmalloc.h>
#include <nuttx/leds/led1642gw.h>
#include <nuttx/leds/rgbled.h>
#include <nuttx/timers/pwm.h>
#include <arch/board/board.h>

#include "stm32.h"
#include "stm32_pwm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PWCLK is common for all LED1642GW devices on the board.  The board.h
 * header file must provide:
 *
 *   BOARD_LED1642GW_PWCLK_TIM     - Timer that generates PWCLK
 *   BOARD_LED1642GW_PWCLK_CHANNEL - Timer channel connected to PWCLK
 *   BOARD_LED1642GW_PWCLK_FREQ    - PWCLK frequency
 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* One instance for each LED1642GW device */

struct stm32_led1642gw_s
{
  struct led1642gw_config_s config;   /* Must be first */
  uint32_t                  clk;      /* CLK pin */
  uint32_t                  sdi;      /* SDI pin */
  uint32_t                  le;       /* LE pin */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void led1642gw_clk(const struct led1642gw_config_s *config,
                          bool state);
static void led1642gw_sdi(const struct led1642gw_config_s *config,
                          bool state);
static void led1642gw_le(const struct led1642gw_config_s *config,
                         bool state);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_led1642gw_pwclk;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: led1642gw_clk/sdi/le
 *
 * Description:
 *   Set the state of the CLK, SDI and LE lines.
 *
 ****************************************************************************/

static void led1642gw_clk(const struct led1642gw_config_s *config,
                          bool state)
{
  const struct stm32_led1642gw_s *priv =
    (const struct stm32_led1642gw_s *)config;

  stm32_gpiowrite(priv->clk, state);
}

static void led1642gw_sdi(const struct led1642gw_config_s *config,
                          bool state)
{
  const struct stm32_led1642gw_s *priv =
    (const struct stm32_led1642gw_s *)config;

  stm32_gpiowrite(priv->sdi, state);
}

static void led1642gw_le(const struct led1642gw_config_s *config,
                         bool state)
{
  const struct stm32_led1642gw_s *priv =
    (const struct stm32_led1642gw_s *)config;

  stm32_gpiowrite(priv->le, state);
}

/****************************************************************************
 * Name: led1642gw_pwclk_start
 *
 * Description:
 *   Start the free-running PWCLK clock.
 *
 ****************************************************************************/

static int led1642gw_pwclk_start(void)
{
  struct pwm_lowerhalf_s *pwm;
  struct pwm_info_s info;
  int ret;

  if (g_led1642gw_pwclk)
    {
      return OK;
    }

  pwm = stm32_pwminitialize(BOARD_LED1642GW_PWCLK_TIM);
  if (pwm == NULL)
    {
      return -ENODEV;
    }

  memset(&info, 0, sizeof(info));
  info.frequency           = BOARD_LED1642GW_PWCLK_FREQ;
  info.channels[0].duty    = b16HALF;
  info.channels[0].channel = BOARD_LED1642GW_PWCLK_CHANNEL;
#if CONFIG_PWM_NCHANNELS > 1
  info.channels[1].channel = -1;
#endif

  ret = pwm->ops->setup(pwm);
  if (ret < 0)
    {
      return ret;
    }

  ret = pwm->ops->start(pwm, &info);
  if (ret < 0)
    {
      return ret;
    }

  g_led1642gw_pwclk = true;
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_led1642gw_initialize
 *
 * Description:
 *   Initialize one LED1642GW LED driver and register the RGB LEDs connected
 *   to its outputs.  RGB LED n is connected to outputs 3n (R), 3n + 1 (G)
 *   and 3n + 2 (B).  Can be called for each LED1642GW device connected to
 *   the board.
 *
 * Input Parameters:
 *   devno - The number of the first RGB LED device (/dev/rgbledN)
 *   nleds - The number of RGB LEDs connected to the device
 *   clk   - The GPIO connected to the CLK line
 *   sdi   - The GPIO connected to the SDI line
 *   le    - The GPIO connected to the LE line
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_led1642gw_initialize(int devno, int nleds, uint32_t clk,
                               uint32_t sdi, uint32_t le)
{
  struct stm32_led1642gw_s *priv;
  struct pwm_lowerhalf_s *pwm;
  char devpath[14];
  int ret;
  int i;

  if (nleds < 1 || nleds > LED1642GW_NCHANNELS / 3)
    {
      return -EINVAL;
    }

  ret = led1642gw_pwclk_start();
  if (ret < 0)
    {
      lederr("ERROR: Failed to start PWCLK: %d\n", ret);
      return ret;
    }

  priv = kmm_zalloc(sizeof(struct stm32_led1642gw_s));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  priv->config.clk   = led1642gw_clk;
  priv->config.sdi   = led1642gw_sdi;
  priv->config.le    = led1642gw_le;
  priv->config.gain  = LED1642GW_GAIN_MAX;
  priv->config.range = false;
  priv->config.pwm16 = false;
  priv->clk          = clk;
  priv->sdi          = sdi;
  priv->le           = le;

  stm32_configgpio(clk);
  stm32_configgpio(sdi);
  stm32_configgpio(le);

  pwm = led1642gw_initialize(&priv->config);
  if (pwm == NULL)
    {
      lederr("ERROR: led1642gw_initialize failed\n");
      kmm_free(priv);
      return -ENOMEM;
    }

  /* Register the RGB LEDs.  The PWM channel n is the output n - 1. */

  for (i = 0; i < nleds; i++)
    {
      snprintf(devpath, sizeof(devpath), "/dev/rgbled%d", devno + i);

      ret = rgbled_register(devpath, pwm, pwm, pwm,
                            3 * i + 1, 3 * i + 2, 3 * i + 3);
      if (ret < 0)
        {
          lederr("ERROR: rgbled_register(%s) failed: %d\n",
                 devpath, ret);
          return ret;
        }
    }

  return OK;
}
