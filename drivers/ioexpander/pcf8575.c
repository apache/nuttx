/****************************************************************************
 * drivers/ioexpander/pcf8575.c
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
#include <debug.h>
#include <sys/param.h>

#include <nuttx/kmalloc.h>
#include <nuttx/ioexpander/ioexpander.h>
#include <nuttx/ioexpander/pcf8575.h>

#include "pcf8575.h"

#ifdef CONFIG_IOEXPANDER_PCF8575

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
#  error PCF8575 cannot work with CONFIG_IOEXPANDER_INT_ENABLE
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* PCF8575xx Helpers */

static int pcf8575_read(FAR struct pcf8575_dev_s *priv,
                        FAR uint16_t *portval);
static int pcf8575_write(struct pcf8575_dev_s *priv, uint16_t portval);

/* I/O Expander Methods */

static int pcf8575_direction(FAR struct ioexpander_dev_s *dev, uint8_t pin,
             int dir);
static int pcf8575_option(FAR struct ioexpander_dev_s *dev, uint8_t pin,
             int opt, void *regval);
static int pcf8575_writepin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
             bool value);
static int pcf8575_readpin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
             FAR bool *value);
#ifdef CONFIG_IOEXPANDER_MULTIPIN
static int pcf8575_multiwritepin(FAR struct ioexpander_dev_s *dev,
             FAR const uint8_t *pins, FAR const bool *values, int count);
static int pcf8575_multireadpin(FAR struct ioexpander_dev_s *dev,
             FAR const uint8_t *pins, FAR bool *values, int count);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifndef CONFIG_PCF8575_MULTIPLE
/* If only a single device is supported, then the driver state structure may
 * as well be pre-allocated.
 */

static struct pcf8575_dev_s g_pcf8575;
#endif

/* I/O expander vtable */

static const struct ioexpander_ops_s g_pcf8575_ops =
{
  pcf8575_direction,
  pcf8575_option,
  pcf8575_writepin,
  pcf8575_readpin,
  pcf8575_readpin
#ifdef CONFIG_IOEXPANDER_MULTIPIN
  , pcf8575_multiwritepin
  , pcf8575_multireadpin
  , pcf8575_multireadpin
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pcf8575_read
 *
 * Description:
 *  Read the PCF8575 16-bit value from a PCF8575xx port
 *
 *  Primitive I2C read operation for the PCF8575. The PCF8575 is
 *  'interesting' in that it doesn't really have a data direction register,
 *  but instead the outputs are current-limited when high, so by setting an
 *  IO line high, you are also making it an input.  Consequently, before
 *  using this method, you'll need to perform a pca8574_write() setting the
 *  bits you are interested in reading to 1's, then call this method.
 *
 ****************************************************************************/

static int pcf8575_read(FAR struct pcf8575_dev_s *priv,
                        FAR uint16_t *portval)
{
  struct i2c_msg_s msg;
  uint8_t buffer[2];
  int ret;

  DEBUGASSERT(priv != NULL && priv->i2c != NULL && priv->config != NULL);

  /* Setup for the transfer */

  msg.frequency = priv->config->frequency,
  msg.addr      = priv->config->address,
  msg.flags     = I2C_M_READ;
  msg.buffer    = buffer;
  msg.length    = 2;

  /* Then perform the transfer. */

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  *portval = buffer[0] & 0xff;
  *portval |= (buffer[1] << 8) & 0xff00;

  return (ret >= 0) ? OK : ret;
}

/****************************************************************************
 * Name: pcf8575_write
 *
 * Description:
 *  Write a 16-bit value to a PCF8575xx port
 *
 *  Primitive I2C write operation for the PCA8574.  The I2C interface
 *  simply sets the state of the 8 IO lines in the PCA8574 port.
 *
 ****************************************************************************/

static int pcf8575_write(struct pcf8575_dev_s *priv, uint16_t portval)
{
  struct i2c_msg_s msg;
  uint8_t buffer[2];
  int ret;

  DEBUGASSERT(priv != NULL && priv->i2c != NULL && priv->config != NULL);

  /* Setup for the transfer */

  buffer[0] = (uint8_t)(portval & 0xff);
  buffer[1] = (uint8_t)((portval & 0xff00) >> 8);

  msg.frequency = priv->config->frequency,
  msg.addr      = priv->config->address;
  msg.flags     = 0;
  msg.buffer    = buffer;
  msg.length    = 2;

  /* Then perform the transfer. */

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  return (ret >= 0) ? OK : ret;
}

/****************************************************************************
 * Name: pcf8575_direction
 *
 * Description:
 *   Set the direction of an ioexpander pin. Required.
 *
 *   The PCF8575 is 'interesting' in that it doesn't really have a data
 *   direction register, but instead the outputs are current-limited when
 *   high, so by setting an IO line high, you are also making it an input.
 *   Consequently, before using this method, you'll need to perform a
 *   pcf8575_write() setting the bits you are interested in reading to 1's,
 *   before calling pcf8575_read().
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

static int pcf8575_direction(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                          int direction)
{
  FAR struct pcf8575_dev_s *priv = (FAR struct pcf8575_dev_s *)dev;
  int ret;

  if (direction != IOEXPANDER_DIRECTION_IN &&
      direction != IOEXPANDER_DIRECTION_OUT)
    {
      return -EINVAL;
    }

  if (pin > 15)
    {
      return -ENXIO;
    }

  DEBUGASSERT(priv != NULL && priv->config != NULL);

  gpioinfo("I2C addr=%02x pin=%u direction=%s\n",
           priv->config->address, pin,
           (direction == IOEXPANDER_DIRECTION_IN) ? "IN" : "OUT");

  /* Get exclusive access to the I/O Expander */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Set a bit in inpins if the pin is an input.  Clear the bit in
   * inpins if the pin is an output.
   */

  if (direction == IOEXPANDER_DIRECTION_IN)
    {
      priv->inpins   |= (1 << pin);
      priv->outstate &= ~(1 << pin);
    }
  else
    {
      priv->inpins &= ~(1 << pin);
    }

  /* Write the OR of the set of input pins and the set of output pins.
   * In order to read input pins, we have to write a '1' to put the
   * pin in the current limiting state.
   */

  ret = pcf8575_write(priv, priv->inpins | priv->outstate);

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: pcf8575_option
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

static int pcf8575_option(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                          int opt, FAR void *value)
{
  FAR struct pcf8575_dev_s *priv = (FAR struct pcf8575_dev_s *)dev;
  int ret = OK;

  DEBUGASSERT(priv != NULL && priv->config != NULL);

  gpioinfo("I2C addr=%02x pin=%u option=%u\n",
           priv->config->address, pin, opt);

  return ret;
}

/****************************************************************************
 * Name: pcf8575_writepin
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

static int pcf8575_writepin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                            bool value)
{
  FAR struct pcf8575_dev_s *priv = (FAR struct pcf8575_dev_s *)dev;
  int ret;

  if (pin > 15)
    {
      return -ENXIO;
    }

  DEBUGASSERT(priv != NULL && priv->config != NULL);

  gpioinfo("I2C addr=%02x pin=%u value=%u\n",
           priv->config->address, pin, value);

  /* Get exclusive access to the I/O Expander */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Make sure that this is an output pin */

  if ((priv->inpins & (1 << pin)) != 0)
    {
      gpioerr("ERROR: pin%u is an input\n", pin);
      nxmutex_unlock(&priv->lock);
      return -EINVAL;
    }

  /* Set/clear a bit in outstate. */

  if (value)
    {
      priv->outstate |= (1 << pin);
    }
  else
    {
      priv->outstate &= ~(1 << pin);
    }

  /* Write the OR of the set of input pins and the set of output pins.
   * In order to set the new output value.
   */

  ret = pcf8575_write(priv, priv->inpins | priv->outstate);

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: pcf8575_readpin
 *
 * Description:
 *   Read the actual PIN level. This can be different from the last value
 *   written to this pin. Required.
 *
 *   The PCF8575 is 'interesting' in that it doesn't really have a data
 *   direction register, but instead the outputs are current-limited when
 *   high, so by setting an IO line high, you are also making it an input.
 *   Consequently, before using this method, you'll need to perform a
 *   pca8574_write() setting the bits you are interested in reading to 1's,
 *   before calling pca8574_read().
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

static int pcf8575_readpin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                           FAR bool *value)
{
  FAR struct pcf8575_dev_s *priv = (FAR struct pcf8575_dev_s *)dev;
  uint16_t regval;
  int ret;

  if (pin > 15)
    {
      return -ENXIO;
    }

  DEBUGASSERT(priv != NULL && priv->config != NULL && value != NULL);

  gpioinfo("I2C addr=%02x, pin=%u\n", priv->config->address, pin);

  /* Get exclusive access to the I/O Expander */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Is the pin an output? */

  if ((priv->inpins & (1 << pin)) == 0)
    {
      /* We cannot read the value on pin directly.  Just Return the last
       * value that we wrote to the pin.
       */

      *value = ((priv->outstate & (1 << pin)) != 0);
      nxmutex_unlock(&priv->lock);
      return OK;
    }

  /* It is an input pin. Read the input register for this pin
   *
   * The Input Port Register reflects the incoming logic levels of the pins,
   * regardless of whether the pin is defined as an input or an output by
   * the Configuration Register. They act only on read operation.
   */

  ret = pcf8575_read(priv, &regval);
  if (ret < 0)
    {
      gpioerr("ERROR: Failed to read port register: %d\n", ret);

      goto errout_with_lock;
    }

  /* Return 0 or 1 to indicate the state of pin */

  *value = (bool)((regval >> (pin & 0xf)) & 1);
  ret = OK;

errout_with_lock:
  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: pcf8575_multiwritepin
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

#ifdef CONFIG_IOEXPANDER_MULTIPIN
static int pcf8575_multiwritepin(FAR struct ioexpander_dev_s *dev,
                                 FAR const uint8_t *pins,
                                 FAR const bool *values, int count)
{
  FAR struct pcf8575_dev_s *priv = (FAR struct pcf8575_dev_s *)dev;
  uint8_t pin;
  int ret;
  int i;

  DEBUGASSERT(priv != NULL && priv->config != NULL &&
              pins != NULL && values != NULL);

  gpioinfo("I2C addr=%02x count=%d\n", priv->config->address, count);

  /* Get exclusive access to the I/O Expander */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Process each pin setting */

  for (i = 0; i < count; i++)
    {
      /* Make sure that this is an output pin */

      pin = pins[i];
      if (pin > 15)
        {
          nxmutex_unlock(&priv->lock);
          return -ENXIO;
        }

      gpioinfo("%d. pin=%u value=%u\n", i, pin, values[i]);

      if ((priv->inpins & (1 << pin)) != 0)
        {
          gpioerr("ERROR: pin%u is an input\n", pin);
          continue;
        }

      /* Set/clear a bit in outstate. */

      if (values[i])
        {
          priv->outstate |= (1 << pin);
        }
      else
        {
          priv->outstate &= ~(1 << pin);
        }
    }

  /* Write the OR of the set of input pins and the set of output pins.
   * In order to set the new output value.
   */

  ret = pcf8575_write(priv, priv->inpins | priv->outstate);

  nxmutex_unlock(&priv->lock);
  return ret;
}
#endif

/****************************************************************************
 * Name: pcf8575_multireadpin
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

#ifdef CONFIG_IOEXPANDER_MULTIPIN
static int pcf8575_multireadpin(FAR struct ioexpander_dev_s *dev,
                                FAR const uint8_t *pins, FAR bool *values,
                                int count)
{
  FAR struct pcf8575_dev_s *priv = (FAR struct pcf8575_dev_s *)dev;
  uint16_t regval;
  uint8_t pin;
  int ret;
  int i;

  DEBUGASSERT(priv != NULL && priv->config != NULL &&
              pins != NULL && values != NULL);

  gpioinfo("I2C addr=%02x, count=%d\n", priv->config->address, count);

  /* Get exclusive access to the I/O Expander */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Read the input register for this pin
   *
   * The Input Port Register reflects the incoming logic levels of the pins,
   * regardless of whether the pin is defined as an input or an output by
   * the Configuration Register. They act only on read operation.
   */

  ret = pcf8575_read(priv, &regval);
  if (ret < 0)
    {
      gpioerr("ERROR: Failed to read port register: %d\n", ret);
      goto errout_with_lock;
    }

  /* Return the requested pin values */

  for (i = 0; i < count; i++)
    {
      /* Make sure that this is an output pin */

      pin = pins[i];
      if (pin > 15)
        {
          nxmutex_unlock(&priv->lock);
          return -ENXIO;
        }

      /* Is the pin an output? */

      if ((priv->inpins & (1 << pin)) == 0)
        {
          /* We cannot read the value on pin directly.  Just Return the last
           * value that we wrote to the pin.
           */

          values[i] = ((priv->outstate & (1 << pin)) != 0);
        }
      else
        {
          values[i] = ((regval & (1 << pin)) != 0);
        }

      gpioinfo("%d. pin=%u value=%u\n", i, pin, values[i]);
    }

  ret = OK;

errout_with_lock:
  nxmutex_unlock(&priv->lock);
  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pcf8575_initialize
 *
 * Description:
 *   Instantiate and configure the PCF8575xx device driver to use the
 *   provided I2C device instance.
 *
 * Input Parameters:
 *   i2c     - An I2C driver instance
 *   minor   - The device i2c address
 *   config  - Persistent board configuration data
 *
 * Returned Value:
 *   an ioexpander_dev_s instance on success, NULL on failure.
 *
 ****************************************************************************/

FAR struct ioexpander_dev_s *pcf8575_initialize(FAR struct i2c_master_s *i2c,
                              FAR struct pcf8575_config_s *config)
{
  FAR struct pcf8575_dev_s *priv;

#ifdef CONFIG_PCF8575_MULTIPLE
  /* Allocate the device state structure */

  priv = (FAR struct pcf8575_dev_s *)
    kmm_zalloc(sizeof(struct pcf8575_dev_s));
  if (!priv)
    {
      gpioerr("ERROR: Failed to allocate driver instance\n");
      return NULL;
    }
#else
  /* Use the one-and-only I/O Expander driver instance */

  priv = &g_pcf8575;
#endif

  /* Initialize the device state structure */

  priv->dev.ops  = &g_pcf8575_ops;
  priv->i2c      = i2c;
  priv->config   = config;

  nxmutex_init(&priv->lock);
  return &priv->dev;
}

#endif /* CONFIG_IOEXPANDER_PCF8575 */
