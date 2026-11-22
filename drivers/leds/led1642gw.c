/****************************************************************************
 * drivers/leds/led1642gw.c
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

#include <assert.h>
#include <errno.h>
#include <string.h>
#include <nuttx/debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/leds/led1642gw.h>
#include <nuttx/timers/pwm.h>

#ifdef CONFIG_LEDS_LED1642GW

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Digital keys: the number of CLK rising edges with LE high that ends
 * a 16-bit word and selects the command.
 */

#define LED1642GW_KEY_SWITCH   2    /* Write switch */
#define LED1642GW_KEY_DATA     4    /* Brightness data latch */
#define LED1642GW_KEY_GLOBAL   6    /* Brightness global latch */
#define LED1642GW_KEY_WRCFG    7    /* Write configuration register */

/* Configuration register */

#define LED1642GW_CFG_GAIN_MASK 0x003f    /* Bits 0-5: Current gain */
#define LED1642GW_CFG_RANGE     (1 << 6)  /* Bit 6: Current range */
#define LED1642GW_CFG_PWM12     (1 << 15) /* Bit 15: 12-bit brightness */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct led1642gw_dev_s
{
  FAR const struct pwm_ops_s *ops;    /* Must be first */
  FAR const struct led1642gw_config_s *config;
  mutex_t  lock;
  uint16_t brightness[LED1642GW_NCHANNELS];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void led1642gw_write(FAR struct led1642gw_dev_s *priv, uint16_t data,
                            int key);
static void led1642gw_update(FAR struct led1642gw_dev_s *priv);

static int led1642gw_clear(FAR struct led1642gw_dev_s *priv);

static int led1642gw_setup(FAR struct pwm_lowerhalf_s *dev);
static int led1642gw_shutdown(FAR struct pwm_lowerhalf_s *dev);
static int led1642gw_start(FAR struct pwm_lowerhalf_s *dev,
                           FAR const struct pwm_info_s *info);
static int led1642gw_stop(FAR struct pwm_lowerhalf_s *dev);
static int led1642gw_ioctl(FAR struct pwm_lowerhalf_s *dev, int cmd,
                           unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct pwm_ops_s g_led1642gw_ops =
{
  led1642gw_setup,              /* setup */
  led1642gw_shutdown,           /* shutdown */
  led1642gw_start,              /* start */
  led1642gw_stop,               /* stop */
  led1642gw_ioctl,              /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: led1642gw_write
 *
 * Description:
 *   Shift out one 16-bit word, MSB first.  LE is high for the last 'key'
 *   CLK rising edges.
 *
 ****************************************************************************/

static void led1642gw_write(FAR struct led1642gw_dev_s *priv, uint16_t data,
                            int key)
{
  FAR const struct led1642gw_config_s *config = priv->config;
  int i;

  for (i = 15; i >= 0; i--)
    {
      config->sdi(config, (data & (1 << i)) != 0);

      if (i == key - 1)
        {
          config->le(config, true);
        }

      config->clk(config, true);
      config->clk(config, false);
    }

  config->le(config, false);
}

/****************************************************************************
 * Name: led1642gw_update
 *
 * Description:
 *   Load the brightness of all channels and enable the channels with
 *   non-zero brightness.
 *
 ****************************************************************************/

static void led1642gw_update(FAR struct led1642gw_dev_s *priv)
{
  uint16_t mask = 0;
  uint16_t data;
  int i;

  /* Channel 15 goes first, the last word latches all channels at once */

  for (i = LED1642GW_NCHANNELS - 1; i >= 0; i--)
    {
      data = priv->brightness[i];
      if (!priv->config->pwm16)
        {
          data >>= 4;
        }

      led1642gw_write(priv, data,
                      i > 0 ? LED1642GW_KEY_DATA : LED1642GW_KEY_GLOBAL);

      if (data > 0)
        {
          mask |= 1 << i;
        }
    }

  led1642gw_write(priv, mask, LED1642GW_KEY_SWITCH);
}

/****************************************************************************
 * Name: led1642gw_clear
 *
 * Description:
 *   Turn all outputs off.
 *
 ****************************************************************************/

static int led1642gw_clear(FAR struct led1642gw_dev_s *priv)
{
  int ret;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  memset(priv->brightness, 0, sizeof(priv->brightness));
  led1642gw_update(priv);
  nxmutex_unlock(&priv->lock);

  return OK;
}

/****************************************************************************
 * Name: led1642gw_setup
 *
 * Description:
 *   This method is called when the driver is opened.  The device is
 *   configured in led1642gw_initialize(), nothing to do here.
 *
 ****************************************************************************/

static int led1642gw_setup(FAR struct pwm_lowerhalf_s *dev)
{
  return OK;
}

/****************************************************************************
 * Name: led1642gw_shutdown
 *
 * Description:
 *   This method is called when the driver is closed.  Turn all outputs off.
 *
 ****************************************************************************/

static int led1642gw_shutdown(FAR struct pwm_lowerhalf_s *dev)
{
  return led1642gw_clear((FAR struct led1642gw_dev_s *)dev);
}

/****************************************************************************
 * Name: led1642gw_start
 *
 * Description:
 *   Set the brightness of the outputs given in the channel list.  Channel n
 *   is the output n - 1.  The other outputs are not changed.  The frequency
 *   is ignored: it is set by the PWCLK clock.
 *
 ****************************************************************************/

static int led1642gw_start(FAR struct pwm_lowerhalf_s *dev,
                           FAR const struct pwm_info_s *info)
{
  FAR struct led1642gw_dev_s *priv = (FAR struct led1642gw_dev_s *)dev;
  uint32_t duty;
  int channel;
  int ret;
  int i;

  for (i = 0; i < CONFIG_PWM_NCHANNELS; i++)
    {
      channel = info->channels[i].channel;
      if (channel < 0)
        {
          break;
        }

      if (channel > LED1642GW_NCHANNELS)
        {
          lederr("ERROR: Invalid channel: %d\n", channel);
          return -EINVAL;
        }
    }

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  for (i = 0; i < CONFIG_PWM_NCHANNELS; i++)
    {
      channel = info->channels[i].channel;
      if (channel < 0)
        {
          break;
        }

      if (channel == 0)
        {
          continue;
        }

      duty = info->channels[i].duty;
      if (duty > UINT16_MAX)
        {
          duty = UINT16_MAX;
        }

      priv->brightness[channel - 1] = duty;
    }

  led1642gw_update(priv);
  nxmutex_unlock(&priv->lock);

  return OK;
}

/****************************************************************************
 * Name: led1642gw_stop
 *
 * Description:
 *   Turn all outputs off.
 *
 ****************************************************************************/

static int led1642gw_stop(FAR struct pwm_lowerhalf_s *dev)
{
  return led1642gw_clear((FAR struct led1642gw_dev_s *)dev);
}

/****************************************************************************
 * Name: led1642gw_ioctl
 ****************************************************************************/

static int led1642gw_ioctl(FAR struct pwm_lowerhalf_s *dev, int cmd,
                           unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: led1642gw_initialize
 *
 * Description:
 *   Initialize one LED1642GW device: write the configuration register and
 *   turn all outputs off.  The returned PWM lower half can be passed to
 *   pwm_register() or rgbled_register().
 *
 * Input Parameters:
 *   config - The board configuration.  It must persist as long as the
 *            driver persists.
 *
 * Returned Value:
 *   Valid PWM lower half structure reference on success; NULL on failure.
 *
 ****************************************************************************/

FAR struct pwm_lowerhalf_s *
led1642gw_initialize(FAR const struct led1642gw_config_s *config)
{
  FAR struct led1642gw_dev_s *priv;
  uint16_t regval;

  /* Sanity check */

  DEBUGASSERT(config != NULL && config->clk != NULL &&
              config->sdi != NULL && config->le != NULL);

  /* Initialize the LED1642GW device structure */

  priv = kmm_zalloc(sizeof(struct led1642gw_dev_s));
  if (priv == NULL)
    {
      lederr("ERROR: Failed to allocate instance of led1642gw_dev_s\n");
      return NULL;
    }

  priv->ops    = &g_led1642gw_ops;
  priv->config = config;
  nxmutex_init(&priv->lock);

  /* Write the configuration register and turn all outputs off */

  regval = config->gain & LED1642GW_CFG_GAIN_MASK;

  if (config->range)
    {
      regval |= LED1642GW_CFG_RANGE;
    }

  if (!config->pwm16)
    {
      regval |= LED1642GW_CFG_PWM12;
    }

  led1642gw_write(priv, regval, LED1642GW_KEY_WRCFG);
  led1642gw_update(priv);

  return (FAR struct pwm_lowerhalf_s *)priv;
}

#endif /* CONFIG_LEDS_LED1642GW */
