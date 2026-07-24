/****************************************************************************
 * drivers/ioexpander/pi4ioe5v6408.c
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
#include <stdbool.h>
#include <stdint.h>

#include <nuttx/debug.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/ioexpander/ioexpander.h>
#include <nuttx/ioexpander/pi4ioe5v6408.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/signal.h>
#ifdef CONFIG_PI4IOE5V6408_INT_ENABLE
#  include <nuttx/wqueue.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PI4IOE5V6408_REG_CHIP_ID_CTRL   0x01
#define PI4IOE5V6408_REG_DIRECTION      0x03
#define PI4IOE5V6408_REG_OUTPUT         0x05
#define PI4IOE5V6408_REG_OUTPUT_HIGH_Z  0x07
#define PI4IOE5V6408_REG_INPUT_DEFAULT  0x09
#define PI4IOE5V6408_REG_PULL_ENABLE    0x0b
#define PI4IOE5V6408_REG_PULL_SELECT    0x0d
#define PI4IOE5V6408_REG_INPUT          0x0f
#define PI4IOE5V6408_REG_INT_MASK       0x11
#define PI4IOE5V6408_REG_INT_STATUS     0x13

#define PI4IOE5V6408_SOFT_RESET         0xff
#define PI4IOE5V6408_RESET_DELAY_US     1000
#define PI4IOE5V6408_INT_MASK_DEFAULT   0xff
#define PI4IOE5V6408_IN_DEF_DEFAULT     0x00

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_PI4IOE5V6408_INT_ENABLE
struct pi4ioe5v6408_callback_s
{
  ioe_pinset_t pinset;
  ioe_callback_t callback;
  FAR void *arg;
};
#endif

struct pi4ioe5v6408_dev_s
{
  struct ioexpander_dev_s dev;
  FAR struct i2c_master_s *i2c;
  FAR struct pi4ioe5v6408_config_s *config;
  mutex_t lock;
  uint8_t direction;
  uint8_t output;
  uint8_t high_impedance;
  uint8_t pull_enable;
  uint8_t pull_select;
  uint8_t input_default;
  uint8_t interrupt_mask;

#ifdef CONFIG_PI4IOE5V6408_INT_ENABLE
  struct work_s work;
  struct pi4ioe5v6408_callback_s
    callbacks[CONFIG_PI4IOE5V6408_INT_NCALLBACKS];
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int pi4ioe5v6408_direction(FAR struct ioexpander_dev_s *dev,
                                  uint8_t pin, int direction);
static int pi4ioe5v6408_option(FAR struct ioexpander_dev_s *dev,
                               uint8_t pin, int option, FAR void *value);
static int pi4ioe5v6408_writepin(FAR struct ioexpander_dev_s *dev,
                                 uint8_t pin, bool value);
static int pi4ioe5v6408_readpin(FAR struct ioexpander_dev_s *dev,
                                uint8_t pin, FAR bool *value);
static int pi4ioe5v6408_readbuf(FAR struct ioexpander_dev_s *dev,
                                uint8_t pin, FAR bool *value);
#ifdef CONFIG_IOEXPANDER_MULTIPIN
static int pi4ioe5v6408_multiwritepin(FAR struct ioexpander_dev_s *dev,
                                      FAR const uint8_t *pins,
                                      FAR const bool *values, int count);
static int pi4ioe5v6408_multireadpin(FAR struct ioexpander_dev_s *dev,
                                     FAR const uint8_t *pins,
                                     FAR bool *values, int count);
static int pi4ioe5v6408_multireadbuf(FAR struct ioexpander_dev_s *dev,
                                     FAR const uint8_t *pins,
                                     FAR bool *values, int count);
#endif
#ifdef CONFIG_PI4IOE5V6408_INT_ENABLE
static FAR void *pi4ioe5v6408_attach(FAR struct ioexpander_dev_s *dev,
                                     ioe_pinset_t pinset,
                                     ioe_callback_t callback, FAR void *arg);
static int pi4ioe5v6408_detach(FAR struct ioexpander_dev_s *dev,
                               FAR void *handle);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct ioexpander_ops_s g_pi4ioe5v6408_ops =
{
  pi4ioe5v6408_direction,
  pi4ioe5v6408_option,
  pi4ioe5v6408_writepin,
  pi4ioe5v6408_readpin,
  pi4ioe5v6408_readbuf
#ifdef CONFIG_IOEXPANDER_MULTIPIN
  , pi4ioe5v6408_multiwritepin
  , pi4ioe5v6408_multireadpin
  , pi4ioe5v6408_multireadbuf
#endif
#ifdef CONFIG_IOEXPANDER_INT_ENABLE
#  ifdef CONFIG_PI4IOE5V6408_INT_ENABLE
  , pi4ioe5v6408_attach
  , pi4ioe5v6408_detach
#  else
  , NULL
  , NULL
#  endif
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int pi4ioe5v6408_write(FAR struct pi4ioe5v6408_dev_s *priv,
                              uint8_t reg, uint8_t value)
{
  struct i2c_config_s config;
  uint8_t buffer[2];
  int ret;

  config.frequency = priv->config->frequency;
  config.address   = priv->config->address;
  config.addrlen   = 7;

  buffer[0] = reg;
  buffer[1] = value;

  ret = i2c_write(priv->i2c, &config, buffer, sizeof(buffer));
  if (ret < 0)
    {
      gpioerr("ERROR: PI4IOE5V6408 write reg 0x%02x val 0x%02x failed: %d\n",
              reg, value, ret);
    }

  return ret;
}

static int pi4ioe5v6408_read(FAR struct pi4ioe5v6408_dev_s *priv,
                             uint8_t reg, FAR uint8_t *value)
{
  struct i2c_config_s config;
  int ret;

  config.frequency = priv->config->frequency;
  config.address   = priv->config->address;
  config.addrlen   = 7;

  ret = i2c_writeread(priv->i2c, &config, &reg, 1, value, 1);
  if (ret < 0)
    {
      gpioerr("ERROR: PI4IOE5V6408 read reg 0x%02x failed: %d\n",
              reg, ret);
    }

  return ret;
}

static int pi4ioe5v6408_update(FAR struct pi4ioe5v6408_dev_s *priv,
                               uint8_t reg, FAR uint8_t *shadow,
                               uint8_t mask, bool set)
{
  uint8_t value;
  int ret;

  value = set ? (*shadow | mask) : (*shadow & ~mask);

  /* Skip the I2C write when the register is unchanged. */

  if (value == *shadow)
    {
      return OK;
    }

  ret = pi4ioe5v6408_write(priv, reg, value);
  if (ret >= 0)
    {
      *shadow = value;
    }

  return ret;
}

static int pi4ioe5v6408_checkpin(uint8_t pin)
{
  return pin < PI4IOE5V6408_NPINS ? OK : -ENXIO;
}

static int pi4ioe5v6408_setpull(FAR struct pi4ioe5v6408_dev_s *priv,
                                uint8_t pin, int direction)
{
  uint8_t mask = 1 << pin;
  bool enable;
  bool pullup;
  int ret;

  enable = direction == IOEXPANDER_DIRECTION_IN_PULLUP ||
           direction == IOEXPANDER_DIRECTION_IN_PULLDOWN;
  pullup = direction == IOEXPANDER_DIRECTION_IN_PULLUP;

  if (enable)
    {
      ret = pi4ioe5v6408_update(priv, PI4IOE5V6408_REG_PULL_SELECT,
                                &priv->pull_select, mask, pullup);
      if (ret < 0)
        {
          return ret;
        }
    }

  return pi4ioe5v6408_update(priv, PI4IOE5V6408_REG_PULL_ENABLE,
                             &priv->pull_enable, mask, enable);
}

static int pi4ioe5v6408_direction(FAR struct ioexpander_dev_s *dev,
                                  uint8_t pin, int direction)
{
  FAR struct pi4ioe5v6408_dev_s *priv =
    (FAR struct pi4ioe5v6408_dev_s *)dev;
  uint8_t mask;
  bool output;
  int ret;

  ret = pi4ioe5v6408_checkpin(pin);
  if (ret < 0)
    {
      return ret;
    }

  if (direction != IOEXPANDER_DIRECTION_IN &&
      direction != IOEXPANDER_DIRECTION_IN_PULLUP &&
      direction != IOEXPANDER_DIRECTION_IN_PULLDOWN &&
      direction != IOEXPANDER_DIRECTION_OUT)
    {
      return -ENOTSUP;
    }

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  mask = 1 << pin;
  output = direction == IOEXPANDER_DIRECTION_OUT;

  /* Leave pull registers at reset defaults when configuring
   * an output.  Only input modes with explicit pull request change them.
   */

  if (!output)
    {
      ret = pi4ioe5v6408_setpull(priv, pin, direction);
    }
  else
    {
      ret = pi4ioe5v6408_update(priv, PI4IOE5V6408_REG_OUTPUT_HIGH_Z,
                                &priv->high_impedance, mask, false);
    }

  if (ret >= 0)
    {
      ret = pi4ioe5v6408_update(priv, PI4IOE5V6408_REG_DIRECTION,
                                &priv->direction, mask, output);
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

static int pi4ioe5v6408_option(FAR struct ioexpander_dev_s *dev,
                               uint8_t pin, int option, FAR void *value)
{
#ifdef CONFIG_PI4IOE5V6408_INT_ENABLE
  FAR struct pi4ioe5v6408_dev_s *priv =
    (FAR struct pi4ioe5v6408_dev_s *)dev;
  uintptr_t setting = (uintptr_t)value;
  uint8_t mask;
#endif
  int ret;

  ret = pi4ioe5v6408_checkpin(pin);
  if (ret < 0)
    {
      return ret;
    }

#ifndef CONFIG_PI4IOE5V6408_INT_ENABLE
  UNUSED(dev);
  UNUSED(option);
  UNUSED(value);
  return -ENOTSUP;
#else
  if (option != IOEXPANDER_OPTION_INTCFG &&
      option != IOEXPANDER_OPTION_SETMASK)
    {
      return -ENOTSUP;
    }

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  mask = 1 << pin;

  if (option == IOEXPANDER_OPTION_SETMASK)
    {
      if (setting != IOEXPANDER_MASK_DISABLE &&
          setting != IOEXPANDER_MASK_ENABLE)
        {
          ret = -EINVAL;
        }
      else
        {
          ret = pi4ioe5v6408_update(priv, PI4IOE5V6408_REG_INT_MASK,
                                    &priv->interrupt_mask, mask,
                                    setting == IOEXPANDER_MASK_ENABLE);
        }
    }
  else if (setting == IOEXPANDER_VAL_DISABLE)
    {
      ret = pi4ioe5v6408_update(priv, PI4IOE5V6408_REG_INT_MASK,
                                &priv->interrupt_mask, mask, true);
    }
  else if (setting == IOEXPANDER_VAL_HIGH ||
           setting == IOEXPANDER_VAL_RISING ||
           setting == IOEXPANDER_VAL_LOW ||
           setting == IOEXPANDER_VAL_FALLING)
    {
      /* The chip compares the input against INPUT_DEFAULT.  A default low
       * detects high/rising and a default high detects low/falling.
       */

      bool default_high = setting == IOEXPANDER_VAL_LOW ||
                          setting == IOEXPANDER_VAL_FALLING;

      ret = pi4ioe5v6408_update(priv, PI4IOE5V6408_REG_INPUT_DEFAULT,
                                &priv->input_default, mask, default_high);
      if (ret >= 0)
        {
          ret = pi4ioe5v6408_update(priv, PI4IOE5V6408_REG_INT_MASK,
                                    &priv->interrupt_mask, mask, false);
        }
    }
  else
    {
      ret = -ENOTSUP;
    }

  nxmutex_unlock(&priv->lock);
  return ret;
#endif
}

static int pi4ioe5v6408_writepin(FAR struct ioexpander_dev_s *dev,
                                 uint8_t pin, bool value)
{
  FAR struct pi4ioe5v6408_dev_s *priv =
    (FAR struct pi4ioe5v6408_dev_s *)dev;
  int ret;

  ret = pi4ioe5v6408_checkpin(pin);
  if (ret < 0)
    {
      return ret;
    }

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = pi4ioe5v6408_update(priv, PI4IOE5V6408_REG_OUTPUT,
                            &priv->output, 1 << pin, value);
  nxmutex_unlock(&priv->lock);
  return ret;
}

static int pi4ioe5v6408_readpin(FAR struct ioexpander_dev_s *dev,
                                uint8_t pin, FAR bool *value)
{
  FAR struct pi4ioe5v6408_dev_s *priv =
    (FAR struct pi4ioe5v6408_dev_s *)dev;
  uint8_t input;
  int ret;

  if (value == NULL)
    {
      return -EINVAL;
    }

  ret = pi4ioe5v6408_checkpin(pin);
  if (ret < 0)
    {
      return ret;
    }

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = pi4ioe5v6408_read(priv, PI4IOE5V6408_REG_INPUT, &input);
  if (ret >= 0)
    {
      *value = (input & (1 << pin)) != 0;
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

static int pi4ioe5v6408_readbuf(FAR struct ioexpander_dev_s *dev,
                                uint8_t pin, FAR bool *value)
{
  FAR struct pi4ioe5v6408_dev_s *priv =
    (FAR struct pi4ioe5v6408_dev_s *)dev;
  int ret;

  if (value == NULL)
    {
      return -EINVAL;
    }

  ret = pi4ioe5v6408_checkpin(pin);
  if (ret < 0)
    {
      return ret;
    }

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  *value = (priv->output & (1 << pin)) != 0;
  nxmutex_unlock(&priv->lock);
  return OK;
}

#ifdef CONFIG_IOEXPANDER_MULTIPIN
static int pi4ioe5v6408_multiwritepin(FAR struct ioexpander_dev_s *dev,
                                      FAR const uint8_t *pins,
                                      FAR const bool *values, int count)
{
  FAR struct pi4ioe5v6408_dev_s *priv =
    (FAR struct pi4ioe5v6408_dev_s *)dev;
  uint8_t output;
  int ret;
  int i;

  if (pins == NULL || values == NULL || count < 0)
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  output = priv->output;
  for (i = 0; i < count; i++)
    {
      ret = pi4ioe5v6408_checkpin(pins[i]);
      if (ret < 0)
        {
          nxmutex_unlock(&priv->lock);
          return ret;
        }

      if (values[i])
        {
          output |= 1 << pins[i];
        }
      else
        {
          output &= ~(1 << pins[i]);
        }
    }

  ret = pi4ioe5v6408_write(priv, PI4IOE5V6408_REG_OUTPUT, output);
  if (ret >= 0)
    {
      priv->output = output;
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

static int pi4ioe5v6408_multiread(FAR struct ioexpander_dev_s *dev,
                                  FAR const uint8_t *pins,
                                  FAR bool *values, int count, bool buffered)
{
  FAR struct pi4ioe5v6408_dev_s *priv =
    (FAR struct pi4ioe5v6408_dev_s *)dev;
  uint8_t data;
  int ret;
  int i;

  if (pins == NULL || values == NULL || count < 0)
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (buffered)
    {
      data = priv->output;
    }
  else
    {
      ret = pi4ioe5v6408_read(priv, PI4IOE5V6408_REG_INPUT, &data);
      if (ret < 0)
        {
          nxmutex_unlock(&priv->lock);
          return ret;
        }
    }

  for (i = 0; i < count; i++)
    {
      ret = pi4ioe5v6408_checkpin(pins[i]);
      if (ret < 0)
        {
          nxmutex_unlock(&priv->lock);
          return ret;
        }

      values[i] = (data & (1 << pins[i])) != 0;
    }

  nxmutex_unlock(&priv->lock);
  return OK;
}

static int pi4ioe5v6408_multireadpin(FAR struct ioexpander_dev_s *dev,
                                     FAR const uint8_t *pins,
                                     FAR bool *values, int count)
{
  return pi4ioe5v6408_multiread(dev, pins, values, count, false);
}

static int pi4ioe5v6408_multireadbuf(FAR struct ioexpander_dev_s *dev,
                                     FAR const uint8_t *pins,
                                     FAR bool *values, int count)
{
  return pi4ioe5v6408_multiread(dev, pins, values, count, true);
}
#endif

#ifdef CONFIG_PI4IOE5V6408_INT_ENABLE
static void pi4ioe5v6408_irqworker(FAR void *arg)
{
  FAR struct pi4ioe5v6408_dev_s *priv =
    (FAR struct pi4ioe5v6408_dev_s *)arg;
  uint8_t status = 0;
  int ret;
  int i;

  ret = nxmutex_lock(&priv->lock);
  if (ret >= 0)
    {
      ret = pi4ioe5v6408_read(priv, PI4IOE5V6408_REG_INT_STATUS, &status);
      nxmutex_unlock(&priv->lock);
    }

  if (ret >= 0)
    {
      for (i = 0; i < CONFIG_PI4IOE5V6408_INT_NCALLBACKS; i++)
        {
          ioe_pinset_t match = priv->callbacks[i].pinset & status;

          if (match != 0 && priv->callbacks[i].callback != NULL)
            {
              priv->callbacks[i].callback(&priv->dev, match,
                                          priv->callbacks[i].arg);
            }
        }
    }

  priv->config->enable(priv->config, true);
}

static int pi4ioe5v6408_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct pi4ioe5v6408_dev_s *priv =
    (FAR struct pi4ioe5v6408_dev_s *)arg;
  int ret;

  if (work_available(&priv->work))
    {
      priv->config->enable(priv->config, false);
      ret = work_queue(HPWORK, &priv->work, pi4ioe5v6408_irqworker,
                       priv, 0);
      if (ret < 0)
        {
          priv->config->enable(priv->config, true);
          return ret;
        }
    }

  return OK;
}

static FAR void *pi4ioe5v6408_attach(FAR struct ioexpander_dev_s *dev,
                                     ioe_pinset_t pinset,
                                     ioe_callback_t callback, FAR void *arg)
{
  FAR struct pi4ioe5v6408_dev_s *priv =
    (FAR struct pi4ioe5v6408_dev_s *)dev;
  FAR void *handle = NULL;
  int ret;
  int i;

  if (callback == NULL || pinset == 0)
    {
      return NULL;
    }

  pinset &= (1 << PI4IOE5V6408_NPINS) - 1;
  if (pinset == 0)
    {
      return NULL;
    }

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return NULL;
    }

  for (i = 0; i < CONFIG_PI4IOE5V6408_INT_NCALLBACKS; i++)
    {
      if (priv->callbacks[i].callback == NULL)
        {
          priv->callbacks[i].pinset = pinset;
          priv->callbacks[i].callback = callback;
          priv->callbacks[i].arg = arg;
          handle = &priv->callbacks[i];
          break;
        }
    }

  nxmutex_unlock(&priv->lock);
  return handle;
}

static int pi4ioe5v6408_detach(FAR struct ioexpander_dev_s *dev,
                               FAR void *handle)
{
  FAR struct pi4ioe5v6408_dev_s *priv =
    (FAR struct pi4ioe5v6408_dev_s *)dev;
  FAR struct pi4ioe5v6408_callback_s *callback =
    (FAR struct pi4ioe5v6408_callback_s *)handle;
  int ret;

  if ((uintptr_t)callback < (uintptr_t)&priv->callbacks[0] ||
      (uintptr_t)callback >=
      (uintptr_t)&priv->callbacks[CONFIG_PI4IOE5V6408_INT_NCALLBACKS])
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  callback->pinset = 0;
  callback->callback = NULL;
  callback->arg = NULL;
  nxmutex_unlock(&priv->lock);
  return OK;
}
#endif

static int pi4ioe5v6408_reset(FAR struct pi4ioe5v6408_dev_s *priv)
{
  uint8_t chip_id;
  int ret;

  ret = pi4ioe5v6408_write(priv, PI4IOE5V6408_REG_CHIP_ID_CTRL,
                           PI4IOE5V6408_SOFT_RESET);
  if (ret < 0)
    {
      return ret;
    }

  /* Give the chip time to leave reset before the follow-up register
   * burst (avoids intermittent -EIO).
   */

  nxsig_usleep(PI4IOE5V6408_RESET_DELAY_US);

  /* Reading CHIP_ID_CTRL clears the reset interrupt indication. */

  ret = pi4ioe5v6408_read(priv, PI4IOE5V6408_REG_CHIP_ID_CTRL, &chip_id);
  if (ret < 0)
    {
      return ret;
    }

  gpioinfo("PI4IOE5V6408 chip ID 0x%02x\n", chip_id);

  ret = pi4ioe5v6408_write(priv, PI4IOE5V6408_REG_DIRECTION,
                           PI4IOE5V6408_DIR_DEFAULT);
  if (ret < 0)
    {
      return ret;
    }

  ret = pi4ioe5v6408_write(priv, PI4IOE5V6408_REG_OUTPUT_HIGH_Z,
                           PI4IOE5V6408_HIGHZ_DEFAULT);
  if (ret < 0)
    {
      return ret;
    }

  ret = pi4ioe5v6408_write(priv, PI4IOE5V6408_REG_PULL_SELECT,
                           PI4IOE5V6408_PULLSEL_DEFAULT);
  if (ret < 0)
    {
      return ret;
    }

  ret = pi4ioe5v6408_write(priv, PI4IOE5V6408_REG_PULL_ENABLE,
                           PI4IOE5V6408_PULLEN_DEFAULT);
  if (ret < 0)
    {
      return ret;
    }

  ret = pi4ioe5v6408_write(priv, PI4IOE5V6408_REG_OUTPUT,
                           PI4IOE5V6408_OUT_DEFAULT);
  if (ret < 0)
    {
      return ret;
    }

  priv->direction      = PI4IOE5V6408_DIR_DEFAULT;
  priv->high_impedance = PI4IOE5V6408_HIGHZ_DEFAULT;
  priv->pull_select    = PI4IOE5V6408_PULLSEL_DEFAULT;
  priv->pull_enable    = PI4IOE5V6408_PULLEN_DEFAULT;
  priv->output         = PI4IOE5V6408_OUT_DEFAULT;
  priv->input_default  = PI4IOE5V6408_IN_DEF_DEFAULT;
  priv->interrupt_mask = PI4IOE5V6408_INT_MASK_DEFAULT;

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct ioexpander_dev_s *
pi4ioe5v6408_initialize(FAR struct i2c_master_s *i2c,
                        FAR struct pi4ioe5v6408_config_s *config)
{
  FAR struct pi4ioe5v6408_dev_s *priv;
  int ret;

  DEBUGASSERT(i2c != NULL && config != NULL);

#ifdef CONFIG_PI4IOE5V6408_INT_ENABLE
  DEBUGASSERT(config->attach != NULL && config->enable != NULL);
#endif

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      gpioerr("ERROR: Failed to allocate PI4IOE5V6408 instance\n");
      return NULL;
    }

  priv->dev.ops = &g_pi4ioe5v6408_ops;
  priv->i2c     = i2c;
  priv->config  = config;

  nxmutex_init(&priv->lock);

  ret = pi4ioe5v6408_reset(priv);
  if (ret < 0)
    {
      nxmutex_destroy(&priv->lock);
      kmm_free(priv);
      return NULL;
    }

#ifdef CONFIG_PI4IOE5V6408_INT_ENABLE
  ret = config->attach(config, pi4ioe5v6408_interrupt, priv);
  if (ret < 0)
    {
      nxmutex_destroy(&priv->lock);
      kmm_free(priv);
      return NULL;
    }

  config->enable(config, true);
#endif

  return &priv->dev;
}
