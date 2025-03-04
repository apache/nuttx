/****************************************************************************
 * drivers/ioexpander/iso1i813t.c
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
#include <nuttx/arch.h>
#include <nuttx/spi/spi.h>
#include <nuttx/ioexpander/ioexpander.h>
#include <nuttx/ioexpander/iso1i813t.h>

#include "iso1i813t.h"

#ifdef CONFIG_IOEXPANDER_ISO1I813T

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct iso1i813t_dev_s
{
  struct ioexpander_dev_s dev;           /* Nested structure to allow casting as public gpio
                                          * expander. */
  FAR struct iso1i813t_config_s *config; /* Board configuration data */
  FAR struct spi_dev_s *spi;             /* Saved I2C driver instance */
  mutex_t lock;
};

/****************************************************************************
 * Private Function Protototypes
 ****************************************************************************/

/* SPI helpers */

static void iso1i813t_select(FAR struct spi_dev_s *spi,
                             FAR struct iso1i813t_config_s *config,
                             int bits);
static void iso1i813t_deselect(FAR struct spi_dev_s *spi,
                               FAR struct iso1i813t_config_s *config);

/* I/O Expander Methods */

static int iso1i813t_direction(FAR struct ioexpander_dev_s *dev, uint8_t pin,
             int dir);
static int iso1i813t_option(FAR struct ioexpander_dev_s *dev, uint8_t pin,
             int opt, void *regval);
static int iso1i813t_writepin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
             bool value);
static int iso1i813t_readpin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
             FAR bool *value);
#ifdef CONFIG_IOEXPANDER_MULTIPIN
static int iso1i813t_multiwritepin(FAR struct ioexpander_dev_s *dev,
             FAR const uint8_t *pins, FAR const bool *values, int count);
static int iso1i813t_multireadpin(FAR struct ioexpander_dev_s *dev,
             FAR const uint8_t *pins, FAR bool *values, int count);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifndef CONFIG_ISO1I813T_MULTIPLE
/* If only a single device is supported, then the driver state structure may
 * as well be pre-allocated.
 */

static struct iso1i813t_dev_s g_iso1i813t;
#endif

/* I/O expander vtable */

static const struct ioexpander_ops_s g_iso1i813t_ops =
{
  iso1i813t_direction,
  iso1i813t_option,
  iso1i813t_writepin,
  iso1i813t_readpin,
  iso1i813t_readpin
#ifdef CONFIG_IOEXPANDER_MULTIPIN
  , iso1i813t_multiwritepin
  , iso1i813t_multireadpin
  , iso1i813t_multireadpin
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: iso1i813t_select
 *
 * Description:
 *   Select the SPI, locking and  re-configuring if necessary
 *
 * Input Parameters:
 *   spi   - Reference to the SPI driver structure
 *   bits  - Number of SPI bits
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void iso1i813t_select(FAR struct spi_dev_s *spi,
                             FAR struct iso1i813t_config_s *config, int bits)
{
  /* Select ISO1I813T chip (locking the SPI bus in case there are multiple
   * devices competing for the SPI bus
   */

  SPI_LOCK(spi, true);
  SPI_SELECT(spi, SPIDEV_EXPANDER(config->id), true);

  /* Now make sure that the SPI bus is configured for the ISO1I813T (it
   * might have gotten configured for a different device while unlocked)
   */

  SPI_SETMODE(spi, config->mode);
  SPI_SETBITS(spi, bits);
  SPI_SETFREQUENCY(spi, config->frequency);
}

/****************************************************************************
 * Name: iso1i813t_deselect
 *
 * Description:
 *   De-select the SPI
 *
 * Input Parameters:
 *   spi  - Reference to the SPI driver structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void iso1i813t_deselect(FAR struct spi_dev_s *spi,
                               FAR struct iso1i813t_config_s *config)
{
  /* De-select ISO1I813T chip and relinquish the SPI bus. */

  SPI_SELECT(spi, SPIDEV_EXPANDER(config->id), false);
  SPI_LOCK(spi, false);
}

/****************************************************************************
 * Name: iso1i813t_direction
 *
 * Description:
 *   ISO1I813T is only input pin. However interface is provided in order
 *   to avoid system falls if called.
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

static int iso1i813t_direction(FAR struct ioexpander_dev_s *dev,
                               uint8_t pin, int direction)
{
  FAR struct iso1i813t_dev_s *priv = (FAR struct iso1i813t_dev_s *)dev;

  DEBUGASSERT(priv != NULL && priv->config != NULL);

  gpiowarn("WARNING: ISO1I813T is only input expander!\n");

  return OK;
}

/****************************************************************************
 * Name: iso1i813t_option
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

static int iso1i813t_option(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                            int opt, FAR void *value)
{
  FAR struct iso1i813t_dev_s *priv = (FAR struct iso1i813t_dev_s *)dev;
  uint16_t regval;
  int ret = OK;

  DEBUGASSERT(priv != NULL && priv->config != NULL);

  gpioinfo("Expander id=%02x pin=%u option=%u\n",
           priv->config->id, pin, opt);

  if (opt == IOEXPANDER_OPTION_NONGENERIC)
    {
      /* This means we want to modify context of COEFIL register */

      regval = ((ISO1I813T_COEFIL + (2 * pin)) | ISO1I813T_WRITE_OPS) << 8;
      regval |= (uint16_t)((uintptr_t)value) & 0x1f;
      iso1i813t_select(priv->spi, priv->config, 16);
      SPI_SEND(priv->spi, regval);
      iso1i813t_deselect(priv->spi, priv->config);
    }
  else
    {
      return -ENOTTY;
    }

  return ret;
}

/****************************************************************************
 * Name: iso1i813t_writepin
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

static int iso1i813t_writepin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                            bool value)
{
  gpiowarn("WARNING: ISO1I813T is only input expander!\n");

  return OK;
}

/****************************************************************************
 * Name: iso1i813t_readpin
 *
 * Description:
 *   Read the actual PIN level. This can be different from the last value
 *   written to this pin. Required.
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

static int iso1i813t_readpin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                             FAR bool *value)
{
  FAR struct iso1i813t_dev_s *priv = (FAR struct iso1i813t_dev_s *)dev;
  uint8_t buff;
  int ret;

  if (pin > 7)
    {
      return -ENXIO;
    }

  DEBUGASSERT(priv != NULL && priv->config != NULL && value != NULL);

  gpioinfo("Expander id=%02x, pin=%u\n", priv->config->id, pin);

  /* Get exclusive access to the I/O Expander */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  iso1i813t_select(priv->spi, priv->config, 8);
  SPI_RECVBLOCK(priv->spi, &buff, 1);
  iso1i813t_deselect(priv->spi, priv->config);

  nxmutex_unlock(&priv->lock);

  *value = (bool)((buff >> (pin & 0x7)) & 1);
  return ret;
}

/****************************************************************************
 * Name: iso1i813t_multiwritepin
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
static int iso1i813t_multiwritepin(FAR struct ioexpander_dev_s *dev,
                                   FAR const uint8_t *pins,
                                   FAR const bool *values, int count)
{
  gpiowarn("WARNING: ISO1I813T is only input expander!\n");

  return OK;
}

/****************************************************************************
 * Name: iso1i813t_multireadpin
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

static int iso1i813t_multireadpin(FAR struct ioexpander_dev_s *dev,
                                  FAR const uint8_t *pins, FAR bool *values,
                                  int count)
{
  FAR struct iso1i813t_dev_s *priv = (FAR struct iso1i813t_dev_s *)dev;
  uint8_t buff;
  int pin;
  int i;
  int ret;

  /* Get exclusive access to the ISO1I813T */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  iso1i813t_select(priv->spi, priv->config, 8);
  SPI_RECVBLOCK(priv->spi, &buff, 1);
  iso1i813t_deselect(priv->spi, priv->config);

  for (i = 0; i < count; i++)
    {
      pin = pins[i];
      if (pin > 7)
        {
          nxmutex_unlock(&priv->lock);
          return -ENXIO;
        }

      values[i] = (bool)((buff >> (pin & 0x7)) & 1);
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}
#endif /* CONFIG_IOEXPANDER_MULTIPIN */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: iso1i813t_initialize
 *
 * Description:
 *   Instantiate and configure the ISO1I813Txx device driver to use the
 *   provided SPI device instance.
 *
 * Input Parameters:
 *   spi     - A SPI driver instance
 *   config  - Persistent board configuration data
 *
 * Returned Value:
 *   an ioexpander_dev_s instance on success, NULL on failure.
 *
 ****************************************************************************/

FAR struct ioexpander_dev_s *iso1i813t_initialize(FAR struct spi_dev_s *spi,
                                      FAR struct iso1i813t_config_s *config)
{
  FAR struct iso1i813t_dev_s *priv;

#ifdef CONFIG_ISO1I813T_MULTIPLE
  /* Allocate the device state structure */

  priv = kmm_zalloc(sizeof(struct iso1i813t_dev_s));
  if (!priv)
    {
      gpioerr("ERROR: Failed to allocate driver instance\n");
      return NULL;
    }
#else
  /* Use the one-and-only I/O Expander driver instance */

  priv = &g_iso1i813t;
#endif

  /* Initialize the device state structure */

  priv->dev.ops  = &g_iso1i813t_ops;
  priv->spi      = spi;
  priv->config   = config;

  nxmutex_init(&priv->lock);

  return &priv->dev;
}

#endif /* CONFIG_IOEXPANDER_ISO1I813T */
