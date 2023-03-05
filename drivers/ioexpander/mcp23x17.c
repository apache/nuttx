/****************************************************************************
 * drivers/ioexpander/mcp23x17.c
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
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>
#include <nuttx/ioexpander/ioexpander.h>

#include "mcp23x17.h"

#if defined(CONFIG_IOEXPANDER_MCP23X17)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_I2C
#  warning I2C support is required (CONFIG_I2C)
#endif

#if (!defined CONFIG_SCHED_WORKQUEUE) && (defined CONFIG_MCP23X17_INT_ENABLE)
#  error Work queue support is required (CONFIG_SCHED_WORKQUEUE)
#endif

#if (!defined CONFIG_SCHED_HPWORK) && (defined CONFIG_MCP23X17_INT_ENABLE)
#  error High-Priority Work support is required (CONFIG_SCHED_HPWORK)
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline int mcp23x17_write(FAR struct mcp23x17_dev_s *priv,
             FAR const uint8_t *wbuffer, int wbuflen);
static inline int mcp23x17_writeread(FAR struct mcp23x17_dev_s *priv,
             FAR const uint8_t *wbuffer, int wbuflen, FAR uint8_t *rbuffer,
             int rbuflen);
static int mcp23x17_direction(FAR struct ioexpander_dev_s *dev, uint8_t pin,
             int dir);
static int mcp23x17_option(FAR struct ioexpander_dev_s *dev, uint8_t pin,
             int opt, void *val);
static int mcp23x17_writepin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
             bool value);
static int mcp23x17_readpin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
             FAR bool *value);
static int mcp23x17_readbuf(FAR struct ioexpander_dev_s *dev, uint8_t pin,
             FAR bool *value);
#ifdef CONFIG_IOEXPANDER_MULTIPIN
static int mcp23x17_multiwritepin(FAR struct ioexpander_dev_s *dev,
             FAR uint8_t *pins, FAR bool *values, int count);
static int mcp23x17_multireadpin(FAR struct ioexpander_dev_s *dev,
             FAR uint8_t *pins, FAR bool *values, int count);
static int mcp23x17_multireadbuf(FAR struct ioexpander_dev_s *dev,
             FAR uint8_t *pins, FAR bool *values, int count);
#endif
#ifdef CONFIG_IOEXPANDER_INT_ENABLE
static FAR void *mcp23x17_attach(FAR struct ioexpander_dev_s *dev,
             ioe_pinset_t pinset, ioe_callback_t callback, FAR void *arg);
static int mcp23x17_detach(FAR struct ioexpander_dev_s *dev,
             FAR void *handle);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifndef CONFIG_MCP23X17_MULTIPLE
/* If only a single MCP23X17 device is supported, then the driver state
 * structure may as well be pre-allocated.
 */

static struct mcp23x17_dev_s g_mcp23x17;

/* Otherwise, we will need to maintain allocated driver instances in a list */

#else
static struct mcp23x17_dev_s *g_mcp23x17list;
#endif

/* I/O expander vtable */

static const struct ioexpander_ops_s g_mcp23x17_ops =
{
  mcp23x17_direction,
  mcp23x17_option,
  mcp23x17_writepin,
  mcp23x17_readpin,
  mcp23x17_readbuf
#ifdef CONFIG_IOEXPANDER_MULTIPIN
  , mcp23x17_multiwritepin
  , mcp23x17_multireadpin
  , mcp23x17_multireadbuf
#endif
#ifdef CONFIG_IOEXPANDER_INT_ENABLE
  , mcp23x17_attach
  , mcp23x17_detach
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mcp23x17_write
 *
 * Description:
 *   Write to the I2C device.
 *
 ****************************************************************************/

static inline int mcp23x17_write(FAR struct mcp23x17_dev_s *priv,
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
 * Name: mcp23x17_writeread
 *
 * Description:
 *   Write to then read from the I2C device.
 *
 ****************************************************************************/

static inline int mcp23x17_writeread(FAR struct mcp23x17_dev_s *priv,
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
 * Name: mcp23x17_setbit
 *
 * Description:
 *  Write a bit in a register pair
 *
 ****************************************************************************/

static int mcp23x17_setbit(FAR struct mcp23x17_dev_s *priv, uint8_t addr,
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
      addr += 1;
      pin  -= 8;
    }

  buf[0] = addr;

#ifdef CONFIG_MCP23X17_SHADOW_MODE
  /* Get the shadowed register value */

  buf[1] = priv->sreg[addr];

#else
  /* Get the register value from the IO-Expander */

  ret = mcp23x17_writeread(priv, &buf[0], 1, &buf[1], 1);
  if (ret < 0)
    {
      return ret;
    }
#endif

  if (bitval)
    {
      buf[1] |= (1 << pin);
    }
  else
    {
      buf[1] &= ~(1 << pin);
    }

#ifdef CONFIG_MCP23X17_SHADOW_MODE
  /* Save the new register value in the shadow register */

  priv->sreg[addr] = buf[1];
#endif

  ret = mcp23x17_write(priv, buf, 2);
#ifdef CONFIG_MCP23X17_RETRY
  if (ret != OK)
    {
      /* Try again (only once) */

      ret = mcp23x17_write(priv, buf, 2);
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: mcp23x17_getbit
 *
 * Description:
 *  Get a bit from a register pair
 *
 ****************************************************************************/

static int mcp23x17_getbit(FAR struct mcp23x17_dev_s *priv, uint8_t addr,
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
      addr += 1;
      pin  -= 8;
    }

  ret = mcp23x17_writeread(priv, &addr, 1, &buf, 1);
  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_MCP23X17_SHADOW_MODE
  /* Save the new register value in the shadow register */

  priv->sreg[addr] = buf;
#endif

  *val = (buf >> pin) & 1;
  return OK;
}

/****************************************************************************
 * Name: mcp23x17_direction
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

static int mcp23x17_direction(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                             int direction)
{
  FAR struct mcp23x17_dev_s *priv = (FAR struct mcp23x17_dev_s *)dev;
  int ret;

  if (direction != IOEXPANDER_DIRECTION_IN &&
      direction != IOEXPANDER_DIRECTION_IN_PULLUP &&
      direction != IOEXPANDER_DIRECTION_OUT)
    {
      return -EINVAL;
    }

  /* Get exclusive access to the MCP23X17 */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = mcp23x17_setbit(priv, MCP23X17_IODIRA, pin,
                        (direction == IOEXPANDER_DIRECTION_IN) ||
                        (direction == IOEXPANDER_DIRECTION_IN_PULLUP));
  if (ret < 0)
    {
      nxmutex_unlock(&priv->lock);
      return ret;
    }

  ret = mcp23x17_setbit(priv, MCP23X17_GPPUA, pin,
                        (direction == IOEXPANDER_DIRECTION_IN_PULLUP));

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: mcp23x17_option
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

static int mcp23x17_option(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                           int opt, FAR void *value)
{
  FAR struct mcp23x17_dev_s *priv = (FAR struct mcp23x17_dev_s *)dev;
  int ret = -EINVAL;

  if (opt == IOEXPANDER_OPTION_INVERT)
    {
      /* Get exclusive access to the MCP23X17 */

      ret = nxmutex_lock(&priv->lock);
      if (ret < 0)
        {
          return ret;
        }

      ret = mcp23x17_setbit(priv, MCP23X17_IPOLA, pin,
                            ((uintptr_t)value == IOEXPANDER_VAL_INVERT));
      nxmutex_unlock(&priv->lock);
    }

  return ret;
}

/****************************************************************************
 * Name: mcp23x17_writepin
 *
 * Description:
 *   Set the pin level. Required.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   pin - The index of the pin to alter in this call
 *   val - The pin level. Usually TRUE will set the pin high,
 *         except if OPTION_INVERT has been set on this pin.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int mcp23x17_writepin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                             bool value)
{
  FAR struct mcp23x17_dev_s *priv = (FAR struct mcp23x17_dev_s *)dev;
  int ret;

  /* Get exclusive access to the MCP23X17 */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = mcp23x17_setbit(priv, MCP23X17_GPIOA, pin, value);
  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: mcp23x17_readpin
 *
 * Description:
 *   Read the actual PIN level. This can be different from the last value
 *      written to this pin. Required.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   pin    - The index of the pin
 *   valptr - Pointer to a buffer where the pin level is stored. Usually TRUE
 *            if the pin is high, except if OPTION_INVERT has been set on
 *            this pin.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int mcp23x17_readpin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                            FAR bool *value)
{
  FAR struct mcp23x17_dev_s *priv = (FAR struct mcp23x17_dev_s *)dev;
  int ret;

  /* Get exclusive access to the MCP23X17 */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = mcp23x17_getbit(priv, MCP23X17_GPIOA, pin, value);

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: mcp23x17_readbuf
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

static int mcp23x17_readbuf(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                            FAR bool *value)
{
  FAR struct mcp23x17_dev_s *priv = (FAR struct mcp23x17_dev_s *)dev;
  int ret;

  /* Get exclusive access to the MCP23X17 */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = mcp23x17_getbit(priv, MCP23X17_GPIOA, pin, value);
  nxmutex_unlock(&priv->lock);
  return ret;
}

#ifdef CONFIG_IOEXPANDER_MULTIPIN

/****************************************************************************
 * Name: mcp23x17_getmultibits
 *
 * Description:
 *  Read multiple bits from MCP23X17 registers.
 *
 ****************************************************************************/

static int mcp23x17_getmultibits(FAR struct mcp23x17_dev_s *priv,
                                 uint8_t addr,
                                 FAR uint8_t *pins,
                                 FAR bool *values,
                                 int count)
{
  uint8_t buf[2];
  int ret = OK;
  int i;
  int index;
  int pin;

  ret = mcp23x17_writeread(priv, &addr, 1, buf, 2);
  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_MCP23X17_SHADOW_MODE
  /* Save the new register value in the shadow register */

  priv->sreg[addr]     = buf[0];
  priv->sreg[addr + 1] = buf[1];
#endif

  /* Read the requested bits */

  for (i = 0; i < count; i++)
    {
      index = 0;
      pin   = pins[i];
      if (pin > 15)
        {
          return -ENXIO;
        }
      else if (pin > 7)
        {
          index = 1;
          pin  -= 8;
        }

      values[i] = (buf[index] >> pin) & 1;
    }

  return OK;
}

/****************************************************************************
 * Name: mcp23x17_multiwritepin
 *
 * Description:
 *   Set the pin level for multiple pins. This routine may be faster than
 *   individual pin accesses. Optional.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   pins - The list of pin indexes to alter in this call
 *   val - The list of pin levels.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int mcp23x17_multiwritepin(FAR struct ioexpander_dev_s *dev,
                                  FAR uint8_t *pins, FAR bool *values,
                                  int count)
{
  FAR struct mcp23x17_dev_s *priv = (FAR struct mcp23x17_dev_s *)dev;
  uint8_t addr = MCP23X17_GPIOA;
  uint8_t buf[3];
  int ret;
  int i;
  int index;
  int pin;

  /* Get exclusive access to the MCP23X17 */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Start by reading both registers, whatever the pins to change. We could
   * attempt to read one port only if all pins were on the same port, but
   * this would not save much.
   */

#ifndef CONFIG_MCP23X17_SHADOW_MODE
  ret = mcp23x17_writeread(priv, &addr, 1, &buf[1], 2);
  if (ret < 0)
    {
      nxmutex_unlock(&priv->lock);
      return ret;
    }
#else
  /* In Shadow-Mode we "read" the pin status from the shadow registers */

  buf[1] = priv->sreg[addr];
  buf[2] = priv->sreg[addr + 1];
#endif

  /* Apply the user defined changes */

  for (i = 0; i < count; i++)
    {
      index = 1;
      pin = pins[i];
      if (pin > 15)
        {
          nxmutex_unlock(&priv->lock);
          return -ENXIO;
        }
      else if (pin > 7)
        {
          index = 2;
          pin  -= 8;
        }

      if (values[i])
        {
          buf[index] |= (1 << pin);
        }
      else
        {
          buf[index] &= ~(1 << pin);
        }
    }

  /* Now write back the new pins states */

  buf[0] = addr;
#ifdef CONFIG_MCP23X17_SHADOW_MODE
  /* Save the new register values in the shadow register */

  priv->sreg[addr]     = buf[1];
  priv->sreg[addr + 1] = buf[2];
#endif
  ret = mcp23x17_write(priv, buf, 3);

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: mcp23x17_multireadpin
 *
 * Description:
 *   Read the actual level for multiple pins. This routine may be faster than
 *   individual pin accesses. Optional.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   pin    - The list of pin indexes to read
 *   valptr - Pointer to a buffer where the pin levels are stored.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int mcp23x17_multireadpin(FAR struct ioexpander_dev_s *dev,
                                 FAR uint8_t *pins, FAR bool *values,
                                 int count)
{
  FAR struct mcp23x17_dev_s *priv = (FAR struct mcp23x17_dev_s *)dev;
  int ret;

  /* Get exclusive access to the MCP23X17 */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = mcp23x17_getmultibits(priv, MCP23X17_GPIOA,
                              pins, values, count);
  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: mcp23x17_multireadbuf
 *
 * Description:
 *   Read the buffered level of multiple pins. This routine may be faster
 *   than individual pin accesses. Optional.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   pin    - The index of the pin
 *   valptr - Pointer to a buffer where the buffered levels are stored.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int mcp23x17_multireadbuf(FAR struct ioexpander_dev_s *dev,
                                 FAR uint8_t *pins, FAR bool *values,
                                 int count)
{
  FAR struct mcp23x17_dev_s *priv = (FAR struct mcp23x17_dev_s *)dev;
  int ret;

  /* Get exclusive access to the MCP23X17 */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = mcp23x17_getmultibits(priv, MCP23X17_GPIOA,
                              pins, values, count);
  nxmutex_unlock(&priv->lock);
  return ret;
}

#endif

#ifdef CONFIG_MCP23X17_INT_ENABLE

/****************************************************************************
 * Name: mcp23x17_attach
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

static FAR void *mcp23x17_attach(FAR struct ioexpander_dev_s *dev,
                                 ioe_pinset_t pinset,
                                 ioe_callback_t callback,
                                 FAR void *arg)
{
  FAR struct mcp23x17_dev_s *priv = (FAR struct mcp23x17_dev_s *)dev;
  FAR void *handle = NULL;
  uint8_t addr = MCP23X17_GPINTENA;
  uint8_t buf[3];
  int i;
  int ret;

  /* Get exclusive access to the MCP23X17 */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return NULL;
    }

  ret = mcp23x17_writeread(priv, &addr, 1, &buf[1], 2);
  if (ret < 0)
    {
      nxmutex_unlock(&priv->lock);
      return NULL;
    }

  buf[0] = addr;
  buf[1] |= (uint8_t)(pinset & 0x00ff);
  buf[2] |= (uint8_t)((pinset & 0xff00) >> 8);

  ret = mcp23x17_write(priv, buf, 3);
  if (ret < 0)
    {
      nxmutex_unlock(&priv->lock);
      return NULL;
    }

  /* Find and available in entry in the callback table */

  for (i = 0; i < CONFIG_MCP23X17_INT_NCALLBACKS; i++)
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
 * Name: mcp23x17_detach
 *
 * Description:
 *   Detach and disable a pin interrupt callback function.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   handle   - The non-NULL opaque value return by mcp23x17_attch()
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int mcp23x17_detach(FAR struct ioexpander_dev_s *dev,
                           FAR void *handle)
{
  FAR struct mcp23x17_dev_s *priv = (FAR struct mcp23x17_dev_s *)dev;
  FAR struct mcp23x17_callback_s *cb =
    (FAR struct mcp23x17_callback_s *)handle;

  DEBUGASSERT(priv != NULL && cb != NULL);
  DEBUGASSERT((uintptr_t)cb >= (uintptr_t)&priv->cb[0] &&
              (uintptr_t)cb <=
              (uintptr_t)&priv->cb[CONFIG_MCP23X17_INT_NCALLBACKS - 1]);
  UNUSED(priv);

  cb->pinset = 0;
  cb->cbfunc = NULL;
  cb->cbarg  = NULL;
  return OK;
}

/****************************************************************************
 * Name: mcp23x17_irqworker
 *
 * Description:
 *   Handle GPIO interrupt events (this function actually executes in the
 *   context of the worker thread).
 *
 ****************************************************************************/

static void mcp23x17_irqworker(void *arg)
{
  FAR struct mcp23x17_dev_s *priv = (FAR struct mcp23x17_dev_s *)arg;
  uint8_t addr = MCP23X17_INTFA;
  uint8_t buf[2];
  ioe_pinset_t pinset;
  int ret;
  int i;

  /* Read interrupt flags */

  ret = mcp23x17_writeread(priv, &addr, 1, buf, 2);
  if (ret == OK)
    {
      /* Create a 16-bit pinset */

      pinset = ((unsigned int)buf[1] << 8) | buf[0];

      /* Perform pin interrupt callbacks */

      for (i = 0; i < CONFIG_MCP23X17_INT_NCALLBACKS; i++)
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

      addr = MCP23X17_INTCAPA;

      mcp23x17_writeread(priv, &addr, 1, buf, 2);

#ifdef CONFIG_MCP23X17_SHADOW_MODE
      /* Don't forget to update the shadow registers at this point */

      priv->sreg[addr]     = buf[0];
      priv->sreg[addr + 1] = buf[1];
#endif
    }

  /* Re-enable interrupts */

  priv->config->enable(priv->config, TRUE);
}

/****************************************************************************
 * Name: mcp23x17_interrupt
 *
 * Description:
 *   Handle GPIO interrupt events (this function executes in the
 *   context of the interrupt).
 *
 ****************************************************************************/

static int mcp23x17_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct mcp23x17_dev_s *priv = (FAR struct mcp23x17_dev_s *)arg;

  /* In complex environments, we cannot do I2C transfers from the interrupt
   * handler because semaphores are probably used to lock the I2C bus.  In
   * this case, we will defer processing to the worker thread.  This is also
   * much kinder in the use of system resources and is, therefore, probably
   * a good thing to do in any event.
   */

  /* Notice that further GPIO interrupts are disabled until the work is
   * actually performed.  This is to prevent overrun of the worker thread.
   * Interrupts are re-enabled in mcp23x17_irqworker() when the work is
   * completed.
   */

  if (work_available(&priv->work))
    {
      priv->config->enable(priv->config, FALSE);
      work_queue(HPWORK, &priv->work, mcp23x17_irqworker,
                 (FAR void *)priv, 0);
    }

  return OK;
}

#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mcp23x17_initialize
 *
 * Description:
 *   Initialize a MCP23X17 I2C device.
 *
 * TODO: Add support for more than one device.
 *
 ****************************************************************************/

FAR struct ioexpander_dev_s *mcp23x17_initialize(
                              FAR struct i2c_master_s *i2cdev,
                              FAR struct mcp23x17_config_s *config)
{
  FAR struct mcp23x17_dev_s *priv;
#ifdef CONFIG_MCP23X17_INT_MIRROR
  uint8_t buf[3];
#endif

  DEBUGASSERT(i2cdev != NULL && config != NULL);

#ifdef CONFIG_MCP23X17_MULTIPLE
  /* Allocate the device state structure */

  priv = (FAR struct mcp23x17_dev_s *)
    kmm_zalloc(sizeof(struct mcp23x17_dev_s));
  if (!priv)
    {
      return NULL;
    }

  /* And save the device structure in the list of MCP23X17 so that we can
   * find it later.
   */

  priv->flink = g_mcp23x17list;
  g_mcp23x17list = priv;

#else
  /* Use the global MCP23X17 driver instance */

  priv = &g_mcp23x17;
#endif

  /* Initialize the device state structure */

  priv->i2c     = i2cdev;
  priv->dev.ops = &g_mcp23x17_ops;
  priv->config  = config;

#ifdef CONFIG_MCP23X17_INT_ENABLE

#ifdef CONFIG_MCP23X17_INT_MIRROR
  buf[0] = MCP23X17_IOCON;
  buf[1] = 0x40;
  buf[2] = 0x40;
  mcp23x17_write(priv, buf, 3);
#endif

  priv->config->attach(priv->config, mcp23x17_interrupt, priv);
  priv->config->enable(priv->config, TRUE);
#endif

  nxmutex_init(&priv->lock);
  return &priv->dev;
}

#endif /* CONFIG_IOEXPANDER_MCP23X17 */
