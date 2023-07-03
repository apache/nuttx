/****************************************************************************
 * drivers/ioexpander/sx1509.c
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

#include <inttypes.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/nuttx.h>
#include <nuttx/irq.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>
#include <nuttx/ioexpander/ioexpander.h>

#include "sx1509.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#if (!defined CONFIG_SCHED_HPWORK) && (defined CONFIG_SX1509_INT_ENABLE)
#  error High-Priority Work support is required (CONFIG_SCHED_HPWORK)
#endif

#ifdef CONFIG_IOEXPANDER_MULTIPIN
#  error Multipin operation not supported for SX1509 for now
#endif

#ifdef CONFIG_SX1509_INT_ENABLE
#  error Interrupt support not testen on HW
#endif

/* LED driver register index */

#define SX1509_LEDREGS_TON  (0)
#define SX1509_LEDREGS_ION  (1)
#define SX1509_LEDREGS_OFF  (2)
#define SX1509_LEDREGS_RISE (3)
#define SX1509_LEDREGS_FALL (4)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline int sx1509_write(FAR struct sx1509_dev_s *priv,
             FAR const uint8_t *wbuffer, int wbuflen);
static inline int sx1509_writeread(FAR struct sx1509_dev_s *priv,
             FAR const uint8_t *wbuffer, int wbuflen, FAR uint8_t *rbuffer,
             int rbuflen);
static int sx1509_direction(FAR struct ioexpander_dev_s *dev, uint8_t pin,
             int dir);
static int sx1509_option(FAR struct ioexpander_dev_s *dev, uint8_t pin,
             int opt, void *val);
static int sx1509_writepin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
             bool value);
static int sx1509_readpin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
             FAR bool *value);
static int sx1509_readbuf(FAR struct ioexpander_dev_s *dev, uint8_t pin,
             FAR bool *value);
#ifdef CONFIG_IOEXPANDER_INT_ENABLE
static FAR void *sx1509_attach(FAR struct ioexpander_dev_s *dev,
             ioe_pinset_t pinset, ioe_callback_t callback, FAR void *arg);
static int sx1509_detach(FAR struct ioexpander_dev_s *dev,
             FAR void *handle);
#endif
#ifdef CONFIG_SX1509_LED_ENABLE
static int sx1509_osc_config(FAR struct sx1509_dev_s *priv);
static int sx1509_led2pin(FAR struct sx1509_dev_s *priv, uint8_t led);
static userled_set_t
sx1509_ll_supported(FAR const struct userled_lowerhalf_s *lower);
static void sx1509_ll_setled(FAR const struct userled_lowerhalf_s *lower,
                             int led, bool ledon);
static void sx1509_ll_setall(FAR const struct userled_lowerhalf_s *lower,
                             userled_set_t ledset);
#  ifdef CONFIG_USERLED_LOWER_READSTATE
static void sx1509_ll_getall(FAR const struct userled_lowerhalf_s *lower,
                             userled_set_t *ledset);
#  endif
#  ifdef CONFIG_USERLED_EFFECTS
static void sx1509_ll_effect_sup(FAR const struct userled_lowerhalf_s *lower,
                                 FAR struct userled_effect_sup_s *sup);
static int sx1509_ll_effect_set(FAR const struct userled_lowerhalf_s *lower,
                                FAR struct userled_effect_set_s *effect);
#  endif
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifndef CONFIG_SX1509_MULTIPLE
/* If only a single SX1509 device is supported, then the driver state
 * structure may as well be pre-allocated.
 */

static struct sx1509_dev_s g_sx1509;

/* Otherwise, we will need to maintain allocated driver instances in a list */

#else
static struct sx1509_dev_s *g_sx1509list;
#endif

/* I/O expander vtable */

static const struct ioexpander_ops_s g_sx1509_ops =
{
  sx1509_direction,
  sx1509_option,
  sx1509_writepin,
  sx1509_readpin,
  sx1509_readbuf
#ifdef CONFIG_IOEXPANDER_MULTIPIN
  , NULL
  , NULL
  , NULL
#endif
#ifdef CONFIG_IOEXPANDER_INT_ENABLE
  , sx1509_attach
  , sx1509_detach
#endif
};

#ifdef CONFIG_USERLED_EFFECTS
/* LED driver registers:
 *   TON, ION, OFF, TRISE, TFALL
 */

static FAR uint8_t g_sx1509_led_regs[SX1509_NR_GPIO_MAX][5] =
{
  {
    SX1509_REGTON_0, SX1509_REGION_0, SX1509_REGOFF_0,
    0, 0
  },

  {
    SX1509_REGTON_1, SX1509_REGION_1, SX1509_REGOFF_1,
    0, 0
  },
  {
    SX1509_REGTON_2, SX1509_REGION_2, SX1509_REGOFF_2,
    0, 0
  },
  {
    SX1509_REGTON_3, SX1509_REGION_3, SX1509_REGOFF_3,
    0, 0
  },
  {
    SX1509_REGTON_4, SX1509_REGION_4, SX1509_REGOFF_4,
    SX1509_TRISE_4, SX1509_TFALL_4
  },
  {
    SX1509_REGTON_5, SX1509_REGION_5, SX1509_REGOFF_5,
    SX1509_TRISE_5, SX1509_TFALL_5
  },
  {
    SX1509_REGTON_6, SX1509_REGION_6, SX1509_REGOFF_6,
    SX1509_TRISE_6, SX1509_TFALL_6
  },
  {
    SX1509_REGTON_7, SX1509_REGION_7, SX1509_REGOFF_7,
    SX1509_TRISE_7, SX1509_TFALL_7
  },
  {
    SX1509_REGTON_8, SX1509_REGION_8, SX1509_REGOFF_8,
    0, 0
  },
  {
    SX1509_REGTON_9, SX1509_REGION_9, SX1509_REGOFF_9,
    0, 0
  },
  {
    SX1509_REGTON_10, SX1509_REGION_10, SX1509_REGOFF_10,
    0, 0
  },
  {
    SX1509_REGTON_11, SX1509_REGION_11, SX1509_REGOFF_11,
    0, 0
  },
  {
    SX1509_REGTON_12, SX1509_REGION_12, SX1509_REGOFF_12,
    SX1509_TRISE_12, SX1509_TFALL_12
  },
  {
    SX1509_REGTON_13, SX1509_REGION_13, SX1509_REGOFF_13,
    SX1509_TRISE_13, SX1509_TFALL_13
  },
  {
    SX1509_REGTON_14, SX1509_REGION_14, SX1509_REGOFF_14,
    SX1509_TRISE_14, SX1509_TFALL_14
  },
  {
    SX1509_REGTON_15, SX1509_REGION_15, SX1509_REGOFF_15,
    SX1509_TRISE_15, SX1509_TFALL_15
  }
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sx1509_write
 *
 * Description:
 *   Write to the I2C device.
 *
 ****************************************************************************/

static inline int sx1509_write(FAR struct sx1509_dev_s *priv,
                               FAR const uint8_t *wbuffer, int wbuflen)
{
  struct i2c_msg_s msg;
  int ret;

  /* Setup for the transfer */

  msg.frequency = priv->config->frequency;
  msg.addr      = priv->config->address;
  msg.flags     = 0;
  msg.buffer    = (FAR uint8_t *)wbuffer;  /* Override const */
  msg.length    = wbuflen;

  /* Then perform the transfer. */

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  return (ret >= 0) ? OK : ret;
}

/****************************************************************************
 * Name: sx1509_writeread
 *
 * Description:
 *   Write to then read from the I2C device.
 *
 ****************************************************************************/

static inline int sx1509_writeread(FAR struct sx1509_dev_s *priv,
                                   FAR const uint8_t *wbuffer, int wbuflen,
                                   FAR uint8_t *rbuffer, int rbuflen)
{
  struct i2c_config_s config;

  /* Set up the configuration and perform the write-read operation */

  config.frequency = priv->config->frequency;
  config.address   = priv->config->address;
  config.addrlen   = 7;

  return i2c_writeread(priv->i2c, &config, wbuffer, wbuflen,
                       rbuffer, rbuflen);
}

/****************************************************************************
 * Name: sx1509_setbit
 *
 * Description:
 *  Write a bit in a register pair
 *
 ****************************************************************************/

static int sx1509_setbit(FAR struct sx1509_dev_s *priv, uint8_t addr,
                         uint8_t pin, bool bitval)
{
  uint8_t buf[2];
  int ret;

  if (pin > 15)
    {
      return -ENXIO;
    }
  else if (pin > 7)
    {
      addr -= 1;
      pin  -= 8;
    }

  buf[0] = addr;

  /* Get the register value from the IO-Expander */

  ret = sx1509_writeread(priv, &buf[0], 1, &buf[1], 1);
  if (ret < 0)
    {
      return ret;
    }

  if (bitval)
    {
      buf[1] |= (1 << pin);
    }
  else
    {
      buf[1] &= ~(1 << pin);
    }

  ret = sx1509_write(priv, buf, 2);
#ifdef CONFIG_SX1509_RETRY
  if (ret != OK)
    {
      /* Try again (only once) */

      ret = sx1509_write(priv, buf, 2);
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: sx1509_getbit
 *
 * Description:
 *  Get a bit from a register pair
 *
 ****************************************************************************/

static int sx1509_getbit(FAR struct sx1509_dev_s *priv, uint8_t addr,
                         uint8_t pin, FAR bool *val)
{
  uint8_t buf;
  int ret;

  if (pin > 15)
    {
      return -ENXIO;
    }
  else if (pin > 7)
    {
      addr -= 1;
      pin  -= 8;
    }

  ret = sx1509_writeread(priv, &addr, 1, &buf, 1);
  if (ret < 0)
    {
      return ret;
    }

  *val = (buf >> pin) & 1;
  return OK;
}

/****************************************************************************
 * Name: sx1509_direction
 *
 * Description:
 *   Set the direction of an ioexpander pin. Required.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   pin - The index of the pin to alter in this call
 *   dir - One of the IOEXPANDER_DIRECTION_ macros
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int sx1509_direction(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                            int direction)
{
  FAR struct sx1509_dev_s *priv = (FAR struct sx1509_dev_s *)dev;
  int ret;

  if (direction != IOEXPANDER_DIRECTION_IN &&
      direction != IOEXPANDER_DIRECTION_IN_PULLUP &&
      direction != IOEXPANDER_DIRECTION_IN_PULLDOWN &&
      direction != IOEXPANDER_DIRECTION_OUT &&
      direction != IOEXPANDER_DIRECTION_OUT_OPENDRAIN &&
      direction != IOEXPANDER_DIRECTION_OUT_LED)
    {
      return -EINVAL;
    }

  /* Get exclusive access to the SX1509 */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = sx1509_setbit(priv, SX1509_REGLEDDRIVERENABLE_A, pin,
                      (direction == IOEXPANDER_DIRECTION_OUT_LED));
  if (ret < 0)
    {
      goto errout_unlock;
    }

  ret = sx1509_setbit(priv, SX1509_REGDIR_A, pin,
                      (direction == IOEXPANDER_DIRECTION_IN) ||
                      (direction == IOEXPANDER_DIRECTION_IN_PULLUP) ||
                      (direction == IOEXPANDER_DIRECTION_IN_PULLDOWN));
  if (ret < 0)
    {
      goto errout_unlock;
    }

  ret = sx1509_setbit(priv, SX1509_REGPULLUP_A, pin,
                      (direction == IOEXPANDER_DIRECTION_IN_PULLUP));
  if (ret < 0)
    {
      goto errout_unlock;
    }

  ret = sx1509_setbit(priv, SX1509_REGPULLUP_A, pin,
                      (direction == IOEXPANDER_DIRECTION_IN_PULLDOWN));
  if (ret < 0)
    {
      goto errout_unlock;
    }

  ret = sx1509_setbit(priv, SX1509_REGOPENDRAIN_A, pin,
                      (direction == IOEXPANDER_DIRECTION_OUT_OPENDRAIN));
  if (ret < 0)
    {
      goto errout_unlock;
    }

errout_unlock:
  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: sx1509_option
 *
 * Description:
 *   Set pin options. Required.
 *   Since all IO expanders have various pin options, this API allows setting
 *     pin options in a flexible way.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   pin - The index of the pin to alter in this call
 *   opt - One of the IOEXPANDER_OPTION_ macros
 *   val - The option's value
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int sx1509_option(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                         int opt, FAR void *value)
{
  FAR struct sx1509_dev_s *priv = (FAR struct sx1509_dev_s *)dev;
  int ret = -EINVAL;

  /* Get exclusive access to the SX1509 */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (opt == IOEXPANDER_OPTION_INVERT)
    {
      ret = sx1509_setbit(priv, SX1509_REGPOLARITY_A, pin,
                            ((uintptr_t)value == IOEXPANDER_VAL_INVERT));
    }
#ifdef CONFIG_SX1509_INT_ENABLE
  else if (opt == IOEXPANDER_OPTION_INTCFG)
    {
      uint8_t addr  = SX1509_REGSENSELOW_A - (pin / 4);
      uint8_t shift = (pin % 4) * 2;
      uint8_t buf[2];

      buf[0] = addr;
      ret = sx1509_writeread(priv, &buf[0], 1, &buf[1], 1);
      if (ret < 0)
        {
          goto errout;
        }

      buf[1] &= ~(SX1509_SENSE_MASK << shift);

      if ((uintptr_t)value & IOEXPANDER_VAL_DISABLE)
        {
          buf[1] |= SX1509_SENSE_NONE;
        }
      else if ((uintptr_t)value & IOEXPANDER_VAL_RISING)
        {
          buf[1] |= SX1509_SENSE_RISING;
        }
      else if ((uintptr_t)value & IOEXPANDER_VAL_FALLING)
        {
          buf[1] |= SX1509_SENSE_FALLING;
        }
      else if ((uintptr_t)value & IOEXPANDER_VAL_BOTH)
        {
          buf[1] |= SX1509_SENSE_BOTH;
        }

      ret = sx1509_write(priv, buf, 2);
#  ifdef CONFIG_SX1509_RETRY
      if (ret != OK)
        {
          /* Try again (only once) */

          ret = sx1509_write(priv, buf, 2);
        }
#  endif
    }
#endif
#ifdef CONFIG_SX1509_LED_ENABLE
  else if (opt == IOEXPANDER_OPTION_LEDCFG)
    {
      if ((intptr_t)value < (sizeof(userled_set_t) * 8))
        {
          priv->pin2led[pin] = (intptr_t)value;
        }
      else
        {
          ret = -EINVAL;
        }
    }
#endif

#ifdef CONFIG_SX1509_INT_ENABLE
errout:
#endif
  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: sx1509_writepin
 *
 * Description:
 *   Set the pin level. Required.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   pin - The index of the pin to alter in this call
 *   val - The pin level. Usually true will set the pin high,
 *         except if OPTION_INVERT has been set on this pin.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int sx1509_writepin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                           bool value)
{
  FAR struct sx1509_dev_s *priv = (FAR struct sx1509_dev_s *)dev;
  int ret;

  /* Get exclusive access to the SX1509 */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = sx1509_setbit(priv, SX1509_REGDATA_A, pin, value);
  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: sx1509_readpin
 *
 * Description:
 *   Read the actual PIN level. This can be different from the last value
 *      written to this pin. Required.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   pin    - The index of the pin
 *   valptr - Pointer to a buffer where the pin level is stored. Usually true
 *            if the pin is high, except if OPTION_INVERT has been set on
 *            this pin.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int sx1509_readpin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                          FAR bool *value)
{
  FAR struct sx1509_dev_s *priv = (FAR struct sx1509_dev_s *)dev;
  int ret;

  /* Get exclusive access to the SX1509 */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = sx1509_getbit(priv, SX1509_REGDATA_A, pin, value);
  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: sx1509_readbuf
 *
 * Description:
 *   Read the buffered pin level.
 *   This can be different from the actual pin state. Required.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   pin    - The index of the pin
 *   valptr - Pointer to a buffer where the level is stored.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int sx1509_readbuf(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                          FAR bool *value)
{
  FAR struct sx1509_dev_s *priv = (FAR struct sx1509_dev_s *)dev;
  int ret;

  /* Get exclusive access to the SX1509 */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = sx1509_getbit(priv, SX1509_REGDATA_A, pin, value);
  nxmutex_unlock(&priv->lock);
  return ret;
}

#ifdef CONFIG_SX1509_INT_ENABLE
/****************************************************************************
 * Name: sx1509_attach
 *
 * Description:
 *   Attach and enable a pin interrupt callback function.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   pinset   - The set of pin events that will generate the callback
 *   callback - The pointer to callback function.  NULL will detach the
 *              callback.
 *   arg      - User-provided callback argument
 *
 * Returned Value:
 *   A non-NULL handle value is returned on success.  This handle may be
 *   used later to detach and disable the pin interrupt.
 *
 ****************************************************************************/

static FAR void *sx1509_attach(FAR struct ioexpander_dev_s *dev,
                               ioe_pinset_t pinset,
                               ioe_callback_t callback,
                               FAR void *arg)
{
  FAR struct sx1509_dev_s *priv = (FAR struct sx1509_dev_s *)dev;
  FAR void *handle = NULL;
  uint8_t addr = SX1509_REGINTERRUPTMASK_B;
  uint8_t buf[3];
  int i;
  int ret;

  /* Get exclusive access to the SX1509 */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return NULL;
    }

  ret = sx1509_writeread(priv, &addr, 1, &buf[1], 2);
  if (ret < 0)
    {
      nxmutex_unlock(&priv->lock);
      return NULL;
    }

  buf[0] = addr;
  buf[1] &= (uint8_t)(~(pinset & 0xff00));
  buf[2] &= (uint8_t)(~((pinset & 0x00ff) >> 8));

  ret = sx1509_write(priv, buf, 3);
  if (ret < 0)
    {
      nxmutex_unlock(&priv->lock);
      return NULL;
    }

  /* Find and available in entry in the callback table */

  for (i = 0; i < CONFIG_SX1509_INT_NCALLBACKS; i++)
    {
      /* Is this entry available (i.e., no callback attached) */

      if (priv->cb[i].cbfunc == NULL)
        {
          /* Yes.. use this entry */

          priv->cb[i].pinset = pinset;
          priv->cb[i].cbfunc = callback;
          priv->cb[i].cbarg  = arg;
          handle             = &priv->cb[i];
          break;
        }
    }

  /* Add this callback to the table */

  nxmutex_unlock(&priv->lock);
  return handle;
}

/****************************************************************************
 * Name: sx1509_detach
 *
 * Description:
 *   Detach and disable a pin interrupt callback function.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   handle   - The non-NULL opaque value return by sx1509_attch()
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int sx1509_detach(FAR struct ioexpander_dev_s *dev, FAR void *handle)
{
  FAR struct sx1509_dev_s *priv = (FAR struct sx1509_dev_s *)dev;
  FAR struct sx1509_callback_s *cb =
    (FAR struct sx1509_callback_s *)handle;

  DEBUGASSERT(priv != NULL && cb != NULL);
  DEBUGASSERT((uintptr_t)cb >= (uintptr_t)&priv->cb[0] &&
              (uintptr_t)cb <=
              (uintptr_t)&priv->cb[CONFIG_SX1509_INT_NCALLBACKS - 1]);
  UNUSED(priv);

  cb->pinset = 0;
  cb->cbfunc = NULL;
  cb->cbarg  = NULL;
  return OK;
}

/****************************************************************************
 * Name: sx1509_irqworker
 *
 * Description:
 *   Handle GPIO interrupt events (this function actually executes in the
 *   context of the worker thread).
 *
 ****************************************************************************/

static void sx1509_irqworker(void *arg)
{
  FAR struct sx1509_dev_s *priv = (FAR struct sx1509_dev_s *)arg;
  uint8_t addr = SX1509_REGINTERRUPTSOURCE_B;
  uint8_t buf[2];
  ioe_pinset_t pinset;
  int ret;
  int i;

  /* Read interrupt flags */

  ret = sx1509_writeread(priv, &addr, 1, buf, 2);
  if (ret == OK)
    {
      /* Create a 16-bit pinset */

      pinset = ((unsigned int)buf[0] << 8) | buf[1];

      /* Perform pin interrupt callbacks */

      for (i = 0; i < CONFIG_SX1509_INT_NCALLBACKS; i++)
        {
          /* Is this entry valid (i.e., callback attached)?  If so, did
           * any of the requested pin interrupts occur?
           */

          if (priv->cb[i].cbfunc != NULL)
            {
              /* Did any of the requested pin interrupts occur? */

              ioe_pinset_t match = pinset & priv->cb[i].pinset;
              if (match != 0)
                {
                  /* Yes.. perform the callback */

                  priv->cb[i].cbfunc(&priv->dev, match,
                                    priv->cb[i].cbarg);
                }
            }
        }

      /* Read GPIOs to clear interrupt condition */

      addr = SX1509_REGDATA_B;
      sx1509_writeread(priv, &addr, 1, buf, 2);
    }
}

/****************************************************************************
 * Name: sx1509_interrupt
 *
 * Description:
 *   Handle GPIO interrupt events (this function executes in the
 *   context of the interrupt).
 *
 ****************************************************************************/

static int sx1509_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct sx1509_dev_s *priv = (FAR struct sx1509_dev_s *)arg;

  /* In complex environments, we cannot do I2C transfers from the interrupt
   * handler because semaphores are probably used to lock the I2C bus.  In
   * this case, we will defer processing to the worker thread.  This is also
   * much kinder in the use of system resources and is, therefore, probably
   * a good thing to do in any event.
   */

  /* Notice that further GPIO interrupts are disabled until the work is
   * actually performed.  This is to prevent overrun of the worker thread.
   * Interrupts are re-enabled in sx1509_irqworker() when the work is
   * completed.
   */

  if (work_available(&priv->work))
    {
      work_queue(HPWORK, &priv->work, sx1509_irqworker,
                 (FAR void *)priv, 0);
    }

  return OK;
}
#endif

#ifdef CONFIG_SX1509_LED_ENABLE
/****************************************************************************
 * Name: sx1509_osc_config
 *
 * Description:
 *   Configure oscilator required for LED driver and keypad engine.
 *
 ****************************************************************************/

static int sx1509_osc_config(FAR struct sx1509_dev_s *priv)
{
  int ret = OK;
  uint8_t buf[2];

  if (priv->config->led_pre <= 0 || priv->config->led_pre > 7)
    {
      gpioerr("Invalid oscillator prescaler value %d\n",
              priv->config->led_pre);
      return ret;
    }

  /* Configure oscilator source */

  buf[0] = SX1509_REGCLOCK;
  buf[1] = SX1509_OSC_INT;

  ret = sx1509_write(priv, buf, 2);
  if (ret < 0)
    {
      return ret;
    }

  /* Configure frequency of the LED driver clock */

  buf[0] = SX1509_REGMISC;
  buf[1] = priv->config->led_pre << SX1509_LEDCLK_SHIFT;

  ret = sx1509_write(priv, buf, 2);
  if (ret == OK)
    {
      priv->led_freq = (SX1509_INTOSC_FREQ /
                        (2 << (priv->config->led_pre - 1)));

      priv->t_on_1_ms = 64 * 15 * 255 * 1000 / priv->led_freq;
      priv->t_on_2_ms = 512 * 15 * 255 * 1000 / priv->led_freq;

      priv->t_off_1_ms = 64 * 15 * 255 * 1000 / priv->led_freq;
      priv->t_off_2_ms = 512 * 15 * 255 * 1000 / priv->led_freq;
    }

  return ret;
}

/****************************************************************************
 * Name: sx1509_led2pin
 *
 * Description:
 *   Return the pin number assigned to a given LED
 *
 ****************************************************************************/

static int sx1509_led2pin(FAR struct sx1509_dev_s *priv, uint8_t led)
{
  int pin = -1;
  int i   = 0;

  for (i = 0; i < SX1509_NR_GPIO_MAX; i++)
    {
      if (priv->pin2led[i] == led)
        {
          pin = i;
          break;
        }
    }

  return pin;
}

/****************************************************************************
 * Name: sx1509_ll_supported
 *
 * Description:
 *   Return the set of LEDs supported by the device
 *
 ****************************************************************************/

static userled_set_t
sx1509_ll_supported(FAR const struct userled_lowerhalf_s *lower)
{
  FAR struct sx1509_dev_s *priv =
    container_of(lower, struct sx1509_dev_s, userleds);
  userled_set_t ledset          = 0;
  int           i               = 0;

  for (i = 0; i < SX1509_NR_GPIO_MAX; i++)
    {
      if (priv->pin2led[i] >= 0)
        {
          ledset |= 1 << priv->pin2led[i];
        }
    }

  return ledset;
}

/****************************************************************************
 * Name: sx1509_ll_setled
 *
 * Description:
 *   Set the current state of one LED
 *
 ****************************************************************************/

static void sx1509_ll_setled(FAR const struct userled_lowerhalf_s *lower,
                             int led, bool ledon)
{
  FAR struct sx1509_dev_s *priv =
    container_of(lower, struct sx1509_dev_s, userleds);
  int pin;

  /* Find corresponding pin */

  pin = sx1509_led2pin(priv, led);

  if (pin != -1)
    {
      sx1509_writepin((FAR struct ioexpander_dev_s *)priv, pin, ledon);
    }
}

/****************************************************************************
 * Name: sx1509_ll_setall
 *
 * Description:
 *   Set the state of all LEDs
 *
 ****************************************************************************/

static void sx1509_ll_setall(FAR const struct userled_lowerhalf_s *lower,
                             userled_set_t ledset)
{
  FAR struct sx1509_dev_s *priv =
    container_of(lower, struct sx1509_dev_s, userleds);
  int pin                       = 0;

  for (pin = 0; pin < SX1509_NR_GPIO_MAX; pin++)
    {
      if ((priv->pin2led[pin] >= 0))
        {
          sx1509_ll_setled(lower, priv->pin2led[pin],
                           ledset & (1 << priv->pin2led[pin]));
        }
    }
}

#  ifdef CONFIG_USERLED_LOWER_READSTATE
/****************************************************************************
 * Name: sx1509_ll_getall
 *
 * Description:
 *   Get the state of all LEDs
 *
 ****************************************************************************/

static void sx1509_ll_getall(FAR const struct userled_lowerhalf_s *lower,
                             userled_set_t *ledset)
{
  FAR struct sx1509_dev_s *priv =
    container_of(lower, struct sx1509_dev_s, userleds);
  int val                       = 0;

  for (pin = 0; pin < SX1509_NR_GPIO_MAX; pin++)
    {
      if ((priv->pin2led[pin] >= 0))
        {
          ret = sx1509_readpin((FAR struct ioexpander_dev_s *)priv,
                               pin, &val);
          if (ret < 0)
            {
              break;
            }

          *ledset |= (val << pin);
        }
    }
}
#  endif

#  ifdef CONFIG_USERLED_EFFECTS
/****************************************************************************
 * Name: sx1509_ll_effect_sup
 *
 * Description:
 *   Get supported effects for a given LED pin.
 *
 ****************************************************************************/

static void sx1509_ll_effect_sup(FAR const struct userled_lowerhalf_s *lower,
                                 FAR struct userled_effect_sup_s *sup)
{
  FAR struct sx1509_dev_s *priv =
    container_of(lower, struct sx1509_dev_s, userleds);
  int pin;

  /* Find corresponding pin */

  pin = sx1509_led2pin(priv, sup->led);

  if (pin == -1)
    {
      /* Pin not found */

      sup->int_on  = false;
      sup->int_off = false;
      sup->t_on    = false;
      sup->t_off   = false;
      sup->t_fade  = false;
      sup->t_fall  = false;
    }
  else
    {
      sup->int_on  = true;
      sup->int_off = true;
      sup->t_on    = true;
      sup->t_off   = true;

      if (g_sx1509_led_regs[pin][SX1509_LEDREGS_RISE] != 0)
        {
          sup->t_fade = true;
          sup->t_fall = true;
        }
    }
}

/****************************************************************************
 * Name: sx1509_ll_effect_set
 *
 * Description:
 *   Set effects for a given LED pin.
 *
 ****************************************************************************/

static int sx1509_ll_effect_set(FAR const struct userled_lowerhalf_s *lower,
                                 FAR struct userled_effect_set_s *set)
{
  FAR struct sx1509_dev_s *priv =
    container_of(lower, struct sx1509_dev_s, userleds);
  uint8_t buf[2];
  uint8_t tmp;
  int pin;

  pin = sx1509_led2pin(priv, set->led);

  /* ON intensity [0% - 100%] */

  if (set->int_on > 100)
    {
      gpioerr("int_on value out of supported range %" PRId8 "\n",
              set->int_on);
      return -EINVAL;
    }
  else
    {
      buf[0] = g_sx1509_led_regs[pin][SX1509_LEDREGS_ION];
      buf[1] = 255 * set->int_on / 100;

      sx1509_write(priv, buf, 2);
    }

  /* OFF intensity - not supported for now */

  if (set->int_off > 0)
    {
      gpioerr("int_off value out of supported range %" PRId8 "\n",
              set->int_off);
      return -EINVAL;
    }

  /* ON time [ms] */

  if (set->t_on <= priv->t_on_1_ms)
    {
      tmp = (SX1509_TON_1_MAX * set->t_on / priv->t_on_1_ms);
      if (tmp == 0)
        {
          tmp += 1;
        }
    }

  else if (set->t_on <= priv->t_on_2_ms)
    {
      tmp = ((SX1509_TON_2_MAX * set->t_on / priv->t_on_2_ms) +
             SX1509_TON_1_MAX);
      if (tmp == SX1509_TON_1_MAX)
        {
          tmp += 1;
        }
    }

  else
    {
      gpioerr("t_on value out of supported range %" PRId32 "\n",
              set->t_on);
      return -EINVAL;
    }

  buf[0] = g_sx1509_led_regs[pin][SX1509_LEDREGS_TON];
  buf[1] = (tmp << SX1509_TON_SHIFT);
  sx1509_write(priv, buf, 2);

  /* OFF time [ms] */

  if (set->t_off <= priv->t_off_1_ms)
    {
      tmp = (SX1509_TOFF_1_MAX * set->t_off / priv->t_off_1_ms);
      if (tmp == 0)
        {
          tmp += 1;
        }
    }

  else if (set->t_off <= priv->t_off_2_ms)
    {
      tmp = ((SX1509_TOFF_2_MAX * set->t_off / priv->t_off_2_ms) +
             SX1509_TOFF_1_MAX);
      if (tmp == SX1509_TOFF_1_MAX)
        {
          tmp += 1;
        }
    }

  else
    {
      gpioerr("t_off value out of supported range %" PRId32 "\n",
              set->t_off);
      return -EINVAL;
    }

  buf[0] = g_sx1509_led_regs[pin][SX1509_LEDREGS_OFF];
  buf[1] = (tmp << SX1509_TOFF_SHIFT);
  sx1509_write(priv, buf, 2);

  /* Fade in */

  if (g_sx1509_led_regs[pin][SX1509_LEDREGS_RISE] > 0)
    {
      buf[0] = g_sx1509_led_regs[pin][SX1509_LEDREGS_RISE];
      buf[1] = set->t_fade;
      sx1509_write(priv, buf, 2);
    }

  /* Fade out */

  if (g_sx1509_led_regs[pin][SX1509_LEDREGS_FALL] > 0)
    {
      buf[0] = g_sx1509_led_regs[pin][SX1509_LEDREGS_FALL];
      buf[1] = set->t_fall;
      sx1509_write(priv, buf, 2);
    }

  /* Force ON */

  sx1509_writepin((FAR struct ioexpander_dev_s *)priv, pin, true);

  return OK;
}
#  endif
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sx1509_initialize
 *
 * Description:
 *   Initialize a SX1509 I2C device.
 *
 * Input Parameters:
 *   i2c     - An I2C driver instance
 *   config  - Persistent board configuration data
 *
 ****************************************************************************/

FAR struct ioexpander_dev_s *
sx1509_initialize(FAR struct i2c_master_s *i2cdev,
                  FAR struct sx1509_config_s *config)
{
  FAR struct sx1509_dev_s *priv = NULL;

  DEBUGASSERT(i2cdev != NULL);
  DEBUGASSERT(config != NULL);
  DEBUGASSERT(config->set_nreset != NULL);

  /* Reset device */

  config->set_nreset(false);
  up_mdelay(20);
  config->set_nreset(true);
  up_mdelay(20);

#ifdef CONFIG_SX1509_MULTIPLE
  /* Allocate the device state structure */

  priv = (FAR struct sx1509_dev_s *)
    kmm_zalloc(sizeof(struct sx1509_dev_s));
  if (!priv)
    {
      return NULL;
    }

  /* And save the device structure in the list of SX1509 so that we can
   * find it later.
   */

  priv->flink = g_sx1509list;
  g_sx1509list = priv;
#else
  /* Use the global SX1509 driver instance */

  priv = &g_sx1509;
#endif

  /* Initialize the device state structure */

  priv->i2c     = i2cdev;
  priv->dev.ops = &g_sx1509_ops;
  priv->config  = config;

#ifdef CONFIG_SX1509_INT_ENABLE
  priv->config->attach(priv->config, sx1509_interrupt, priv);
#endif

#ifdef CONFIG_SX1509_LED_ENABLE
  /* Reset pin to LED mapping */

  memset(priv->pin2led, -1, SX1509_NR_GPIO_MAX);
#endif

  nxmutex_init(&priv->lock);

  return &priv->dev;
}

#ifdef CONFIG_SX1509_LED_ENABLE
/****************************************************************************
 * Name: sx1509_leds_initialize
 *
 * Description:
 *   Initialize the LED driver for a given SX1509 device.
 *
 ****************************************************************************/

int sx1509_leds_initialize(FAR struct ioexpander_dev_s *ioe,
                           FAR const char *devname)
{
  FAR struct sx1509_dev_s *priv = (FAR struct sx1509_dev_s *)ioe;
  int ret;

  /* Configure oscilator */

  ret = sx1509_osc_config(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Connect LED lower-half */

  priv->userleds.ll_supported = sx1509_ll_supported;
  priv->userleds.ll_setled    = sx1509_ll_setled;
  priv->userleds.ll_setall    = sx1509_ll_setall;
#ifdef CONFIG_USERLED_LOWER_READSTATE
  priv->userleds.ll_getall    = sx1509_ll_getall;
#endif
#ifdef CONFIG_USERLED_EFFECTS
  priv->userleds.ll_effect_sup = sx1509_ll_effect_sup;
  priv->userleds.ll_effect_set = sx1509_ll_effect_set;
#endif

  return userled_register(devname, &priv->userleds);
}
#endif
