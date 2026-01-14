/****************************************************************************
 * drivers/ioexpander/pca9557.c
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

#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>
#include <nuttx/ioexpander/ioexpander.h>

#include "pca9557.h"

#if defined(CONFIG_IOEXPANDER_PCA9557)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline int pca9557_i2c_write(FAR struct pca9557_dev_s *pca,
                                    FAR const uint8_t *wbuffer, int wbuflen);
static inline int pca9557_i2c_writeread(FAR struct pca9557_dev_s *pca,
                                        FAR const uint8_t *wbuffer,
                                        int wbuflen, FAR uint8_t *rbuffer,
                                        int rbuflen);
static int pca9557_setbit(FAR struct pca9557_dev_s *pca, uint8_t reg,
                          uint8_t pin, bool bitval);
static int pca9557_getbit(FAR struct pca9557_dev_s *pca, uint8_t reg,
                          uint8_t pin, FAR bool *val);
static int pca9557_direction(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                             int direction);
static int pca9557_option(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                          int opt, FAR void *value);
static int pca9557_writepin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                            bool value);
static int pca9557_readpin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                           FAR bool *value);
static int pca9557_readbuf(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                           FAR bool *value);
#ifdef CONFIG_IOEXPANDER_MULTIPIN
static int pca9557_getmultibits(FAR struct pca9557_dev_s *pca, uint8_t reg,
                                FAR uint8_t *pins, FAR bool *values,
                                int count);
static int pca9557_multiwritepin(FAR struct ioexpander_dev_s *dev,
                                 FAR const uint8_t *pins,
                                 FAR const bool *values, int count);
static int pca9557_multireadpin(FAR struct ioexpander_dev_s *dev,
                                FAR const uint8_t *pins,
                                FAR bool *values, int count);
static int pca9557_multireadbuf(FAR struct ioexpander_dev_s *dev,
                                FAR uint8_t *pins, FAR bool *values,
                                int count);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* I/O expander vtable */

static const struct ioexpander_ops_s g_pca9557_ops =
{
  .ioe_direction     = pca9557_direction,
  .ioe_option        = pca9557_option,
  .ioe_writepin      = pca9557_writepin,
  .ioe_readpin       = pca9557_readpin,
  .ioe_readbuf       = pca9557_readbuf,
#ifdef CONFIG_IOEXPANDER_MULTIPIN
  .ioe_multiwritepin = pca9557_multiwritepin,
  .ioe_multireadpin  = pca9557_multireadpin,
  .ioe_multireadbuf  = pca9557_multireadbuf,
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pca9557_i2c_write
 *
 * Description:
 *   Write to the I2C device.
 *
 ****************************************************************************/

static inline int pca9557_i2c_write(FAR struct pca9557_dev_s *pca,
                                    FAR const uint8_t *wbuffer, int wbuflen)
{
  struct i2c_msg_s msg;
  int ret;

  /* Setup for the transfer */

  msg.frequency = pca->config->frequency;
  msg.addr      = pca->config->address;
  msg.flags     = 0;
  msg.buffer    = (FAR uint8_t *)wbuffer;  /* Override const */
  msg.length    = wbuflen;

  /* Then perform the transfer. */

  ret = I2C_TRANSFER(pca->i2c, &msg, 1);
  return (ret >= 0) ? OK : ret;
}

/****************************************************************************
 * Name: pca9557_i2c_writeread
 *
 * Description:
 *   Write to then read from the I2C device.
 *
 ****************************************************************************/

static inline int pca9557_i2c_writeread(FAR struct pca9557_dev_s *pca,
                                        FAR const uint8_t *wbuffer,
                                        int wbuflen, FAR uint8_t *rbuffer,
                                        int rbuflen)
{
  struct i2c_config_s config;

  /* Set up the configuration and perform the write-read operation */

  config.frequency = pca->config->frequency;
  config.address   = pca->config->address;
  config.addrlen   = 7;

  return i2c_writeread(pca->i2c, &config, wbuffer,
                       wbuflen, rbuffer, rbuflen);
}

/****************************************************************************
 * Name: pca9557_setbit
 *
 * Description:
 *  Write a bit in a register pair
 *
 ****************************************************************************/

static int pca9557_setbit(FAR struct pca9557_dev_s *pca, uint8_t reg,
                          uint8_t pin, bool bitval)
{
  uint8_t buf[2];
  int ret;

  if (pin >= PCA9557_GPIO_NPINS)
    {
      return -ENXIO;
    }

  ret = nxrmutex_lock(&pca->lock);
  if (ret < 0)
    {
      return ret;
    }

  buf[0] = reg;

#ifdef CONFIG_PCA9557_SHADOW_MODE
  /* Get the shadowed register value */

  buf[1] = pca->sreg[reg];

#else
  /* Get the register value from the IO-Expander */

  ret = pca9557_i2c_writeread(pca, &buf[0], 1, &buf[1], 1);
  if (ret < 0)
    {
      nxrmutex_unlock(&pca->lock);
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

  ret = pca9557_i2c_write(pca, buf, 2);
#ifdef CONFIG_PCA9557_RETRY
  if (ret != OK)
    {
      /* Try again (only once) */

      ret = pca9557_i2c_write(pca, buf, 2);
    }
#endif

#ifdef CONFIG_PCA9557_SHADOW_MODE
  if (ret == OK)
    {
      /* Save the new register value in the shadow register */

      pca->sreg[reg] = buf[1];
    }
#endif

  nxrmutex_unlock(&pca->lock);
  return ret;
}

/****************************************************************************
 * Name: pca9557_getbit
 *
 * Description:
 *  Get a bit from a register pair
 *
 ****************************************************************************/

static int pca9557_getbit(FAR struct pca9557_dev_s *pca, uint8_t reg,
                          uint8_t pin, FAR bool *val)
{
  uint8_t buf;
  int ret;

  if (pin >= PCA9557_GPIO_NPINS)
    {
      return -ENXIO;
    }

  ret = nxrmutex_lock(&pca->lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = pca9557_i2c_writeread(pca, &reg, 1, &buf, 1);
  if (ret < 0)
    {
      nxrmutex_unlock(&pca->lock);
      return ret;
    }

#ifdef CONFIG_PCA9557_SHADOW_MODE
  /* Save the new register value in the shadow register */

  pca->sreg[reg] = buf;
#endif

  *val = (buf >> pin) & 1;
  nxrmutex_unlock(&pca->lock);
  return OK;
}

/****************************************************************************
 * Name: pca9557_direction
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

static int pca9557_direction(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                             int direction)
{
  if (direction != IOEXPANDER_DIRECTION_IN &&
      direction != IOEXPANDER_DIRECTION_OUT)
    {
      return -EINVAL;
    }

  return pca9557_setbit((FAR struct pca9557_dev_s *)dev, PCA9557_REG_CONFIG,
                        pin, (direction == IOEXPANDER_DIRECTION_IN));
}

/****************************************************************************
 * Name: pca9557_option
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

static int pca9557_option(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                          int opt, FAR void *value)
{
  int ret = -EINVAL;

  if (opt == IOEXPANDER_OPTION_INVERT)
    {
      ret = pca9557_setbit((FAR struct pca9557_dev_s *)dev,
                           PCA9557_REG_POLINV, pin,
                           ((uintptr_t)value == IOEXPANDER_VAL_INVERT));
    }

  return ret;
}

/****************************************************************************
 * Name: pca9557_writepin
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

static int pca9557_writepin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                            bool value)
{
  return pca9557_setbit((FAR struct pca9557_dev_s *)dev, PCA9557_REG_OUTPUT,
                        pin, value);
}

/****************************************************************************
 * Name: pca9557_readpin
 *
 * Description:
 *   Read the actual PIN level. This can be different from the last value
 *   written to this pin. Required.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   pin    - The index of the pin
 *   valptr - Pointer to a buffer where the pin level is stored.
 *            Usually TRUE if the pin is high, except if OPTION_INVERT
 *            has been set on this pin.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int pca9557_readpin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                           FAR bool *value)
{
  return pca9557_getbit((FAR struct pca9557_dev_s *)dev, PCA9557_REG_INPUT,
                        pin, value);
}

/****************************************************************************
 * Name: pca9557_readbuf
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

static int pca9557_readbuf(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                           FAR bool *value)
{
  return pca9557_getbit((FAR struct pca9557_dev_s *)dev, PCA9557_REG_OUTPUT,
                        pin, value);
}

#ifdef CONFIG_IOEXPANDER_MULTIPIN

/****************************************************************************
 * Name: pca9557_getmultibits
 *
 * Description:
 *  Read multiple bits from PCA9557 registers.
 *
 ****************************************************************************/

static int pca9557_getmultibits(FAR struct pca9557_dev_s *pca, uint8_t reg,
                                FAR uint8_t *pins, FAR bool *values,
                                int count)
{
  uint8_t val;
  int ret = OK;
  int i;
  int pin;

  ret = nxrmutex_lock(&pca->lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = pca9557_i2c_writeread(pca, &reg, 1, &val, 1);
  if (ret < 0)
    {
      nxrmutex_unlock(&pca->lock);
      return ret;
    }

#ifdef CONFIG_PCA9557_SHADOW_MODE
  /* Save the new register value in the shadow register */

  pca->sreg[reg]   = val;
#endif

  /* Read the requested bits */

  for (i = 0; i < count; i++)
    {
      pin   = pins[i];
      if (pin >= PCA9557_GPIO_NPINS)
        {
          nxrmutex_unlock(&pca->lock);
          return -ENXIO;
        }

      values[i] = (val >> pin) & 1;
    }

  nxrmutex_unlock(&pca->lock);
  return OK;
}

/****************************************************************************
 * Name: pca9557_multiwritepin
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

static int pca9557_multiwritepin(FAR struct ioexpander_dev_s *dev,
                                 FAR const uint8_t *pins,
                                 FAR const bool *values, int count)
{
  FAR struct pca9557_dev_s *pca = (FAR struct pca9557_dev_s *)dev;
  uint8_t reg = PCA9557_REG_OUTPUT;
  uint8_t buf[2];
  int ret;
  int i;
  int pin;

  /* Get exclusive access to the PCA9557 */

  ret = nxrmutex_lock(&pca->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Start by reading both registers, whatever the pins to change. We could
   * attempt to read one port only if all pins were on the same port, but
   * this would not save much.
   */

#ifndef CONFIG_PCA9557_SHADOW_MODE
  ret = pca9557_i2c_writeread(pca, &reg, 1, &buf[1], 1);
  if (ret < 0)
    {
      nxrmutex_unlock(&pca->lock);
      return ret;
    }
#else
  /* In Shadow-Mode we "read" the pin status from the shadow registers */

  buf[1] = pca->sreg[reg];
#endif

  /* Apply the user defined changes */

  for (i = 0; i < count; i++)
    {
      pin = pins[i];
      if (pin >= PCA9557_GPIO_NPINS)
        {
          nxrmutex_unlock(&pca->lock);
          return -ENXIO;
        }

      if (values[i])
        {
          buf[1] |= (1 << pin);
        }
      else
        {
          buf[1] &= ~(1 << pin);
        }
    }

  /* Now write back the new pins states */

  buf[0] = reg;
#ifdef CONFIG_PCA9557_SHADOW_MODE
  /* Save the new register values in the shadow register */

  pca->sreg[reg] = buf[1];
#endif
  ret = pca9557_i2c_write(pca, buf, 2);

  nxrmutex_unlock(&pca->lock);
  return ret;
}

/****************************************************************************
 * Name: pca9557_multireadpin
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

static int pca9557_multireadpin(FAR struct ioexpander_dev_s *dev,
                                FAR const uint8_t *pins,
                                FAR bool *values, int count)
{
  return pca9557_getmultibits((FAR struct pca9557_dev_s *)dev,
                              PCA9557_REG_INPUT, pins, values, count);
}

/****************************************************************************
 * Name: pca9557_multireadbuf
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

static int pca9557_multireadbuf(FAR struct ioexpander_dev_s *dev,
                                FAR uint8_t *pins, FAR bool *values,
                                int count)
{
  return pca9557_getmultibits((FAR struct pca9557_dev_s *)dev,
                              PCA9557_REG_OUTPUT, pins, values, count);
}

#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pca9557_initialize
 *
 * Description:
 *   Initialize a PCA9557 I2C device.
 *
 ****************************************************************************/

FAR struct ioexpander_dev_s *
pca9557_initialize(FAR struct i2c_master_s *i2cdev,
                   FAR struct pca9557_config_s *config)
{
  FAR struct pca9557_dev_s *pcadev;

  DEBUGASSERT(i2cdev != NULL && config != NULL);

  if (config->set_nreset_pin)
    {
      config->set_nreset_pin(true);
    }

  /* Allocate the device state structure */

  pcadev = kmm_zalloc(sizeof(struct pca9557_dev_s));
  if (!pcadev)
    {
      return NULL;
    }

  /* Initialize the device state structure */

  pcadev->i2c     = i2cdev;
  pcadev->dev.ops = &g_pca9557_ops;
  pcadev->config  = config;

  nxrmutex_init(&pcadev->lock);
  return &pcadev->dev;
}

#endif /* CONFIG_IOEXPANDER_PCA9557 */
