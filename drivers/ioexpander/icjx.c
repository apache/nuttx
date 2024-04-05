/****************************************************************************
 * drivers/ioexpander/icjx.c
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
#include <nuttx/ioexpander/icjx.h>

#include "icjx.h"

#ifdef CONFIG_IOEXPANDER_ICJX

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ICJX_NOP     0x00
#define ICJX_RNW     0x01
#define ICJX_NOB1    0x0f
#define ICJX_NOB2    0x1e
#define ICJX_CONTROL 0x59

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct icjx_callback_s
{
  ioe_pinset_t pinset;
  ioe_callback_t function;
  FAR void *arg;
};

struct icjx_dev_s
{
  struct ioexpander_dev_s dev;         /* Nested structure to allow casting
                                        * as public gpio expander. */
  FAR struct icjx_config_s *config;    /* Board configuration data */
  FAR struct spi_dev_s *spi;           /* Saved SPI driver instance */

  uint16_t outpins;
  uint16_t outstate;
  uint16_t irqpins;
  mutex_t lock;

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
  struct work_s work;
  struct icjx_callback_s callback;
#endif
};

/****************************************************************************
 * Private Function Protototypes
 ****************************************************************************/

/* SPI helpers */

static void icjx_select(FAR struct spi_dev_s *spi,
                        FAR struct icjx_config_s *config, int bits);
static void icjx_deselect(FAR struct spi_dev_s *spi,
                          FAR struct icjx_config_s *config);

/* Read/Write helpers */

static int icjx_read(FAR struct icjx_dev_s *priv, uint8_t reg,
                     uint16_t *data, int nob);
static int icjx_write(FAR struct icjx_dev_s *priv, uint8_t reg,
                      uint16_t data, int nob);

/* I/O Expander Methods */

static int icjx_direction(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                          int dir);
static int icjx_option(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                       int opt, void *regval);
static int icjx_writepin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                         bool value);
static int icjx_readpin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                        FAR bool *value);
#ifdef CONFIG_IOEXPANDER_MULTIPIN
static int icjx_multiwritepin(FAR struct ioexpander_dev_s *dev,
                              FAR const uint8_t *pins,
                              FAR const bool *values, int count);
static int icjx_multireadpin(FAR struct ioexpander_dev_s *dev,
                             FAR const uint8_t *pins, FAR bool *values,
                             int count);
#endif
#ifdef CONFIG_IOEXPANDER_INT_ENABLE
static FAR void *icjx_attach(FAR struct ioexpander_dev_s *dev,
                             ioe_pinset_t pinset, ioe_callback_t callback,
                             FAR void *arg);
static int icjx_detach(FAR struct ioexpander_dev_s *dev, FAR void *handle);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifndef CONFIG_ICJX_MULTIPLE
/* If only a single device is supported, then the driver state structure may
 * as well be pre-allocated.
 */

static struct icjx_dev_s g_icjx;
#endif

/* I/O expander vtable */

static const struct ioexpander_ops_s g_icjx_ops =
{
  icjx_direction,
  icjx_option,
  icjx_writepin,
  icjx_readpin,
  icjx_readpin
#ifdef CONFIG_IOEXPANDER_MULTIPIN
  , icjx_multiwritepin
  , icjx_multireadpin
  , icjx_multireadpin
#endif
#ifdef CONFIG_IOEXPANDER_INT_ENABLE
  , icjx_attach
  , icjx_detach
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icjx_select
 *
 * Description:
 *   Select the SPI, locking and  re-configuring if necessary
 *
 * Input Parameters:
 *   spi    - Reference to the SPI driver structure
 *   config - Reference to iC-JX configuration structure
 *   bits   - Number of SPI bits
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void icjx_select(FAR struct spi_dev_s *spi,
                        FAR struct icjx_config_s *config, int bits)
{
  /* Select iC-JX chip (locking the SPI bus in case there are multiple
   * devices competing for the SPI bus
   */

  SPI_LOCK(spi, true);
  SPI_SELECT(spi, SPIDEV_EXPANDER(config->id), true);

  /* Now make sure that the SPI bus is configured for the iC-JX (it
   * might have gotten configured for a different device while unlocked)
   */

  SPI_SETMODE(spi, config->mode);
  SPI_SETBITS(spi, bits);
  SPI_SETFREQUENCY(spi, config->frequency);
}

/****************************************************************************
 * Name: icjx_deselect
 *
 * Description:
 *   De-select the SPI
 *
 * Input Parameters:
 *   spi    - Reference to the SPI driver structure
 *   config - Reference to iC-JX configuration structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void icjx_deselect(FAR struct spi_dev_s *spi,
                          FAR struct icjx_config_s *config)
{
  /* De-select iC-JX chip and relinquish the SPI bus. */

  SPI_SELECT(spi, SPIDEV_EXPANDER(config->id), false);
  SPI_LOCK(spi, false);
}

/****************************************************************************
 * Name: icjx_read
 *
 * Description:
 *   Helper function for register read operation.
 *
 * Input Parameters:
 *   priv  - Pointer to icjx_dev_s structure
 *   reg   - Register offset (see icjx.h)
 *   data  - Pointer to read data
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int icjx_read(FAR struct icjx_dev_s *priv, uint8_t reg,
                     uint16_t *data, int nob)
{
  uint8_t startaddr;
  uint8_t tx_buffer[6];
  uint8_t rx_buffer[6];

  startaddr =  (priv->config->addr << 6) | (reg << 1) | ICJX_RNW;
  tx_buffer[0] = startaddr;
  tx_buffer[1] = ICJX_NOP;
  tx_buffer[2] = nob;

  icjx_select(priv->spi, priv->config, 8);
  SPI_EXCHANGE(priv->spi, tx_buffer, rx_buffer, 3);

  *data = rx_buffer[2];

  if (priv->config->verification)
    {
      if (nob == ICJX_NOB2)
        {
          tx_buffer[0] = rx_buffer[2];
          SPI_EXCHANGE(priv->spi, tx_buffer, rx_buffer, 1);
          *data |= rx_buffer[0] << 8;
          tx_buffer[0] = rx_buffer[0];
          startaddr =  (priv->config->addr << 6) |
                       ((reg + 1) << 1) | ICJX_RNW;
        }
      else
        {
          tx_buffer[0] = rx_buffer[2];
        }

      tx_buffer[1] = ICJX_CONTROL;
      SPI_EXCHANGE(priv->spi, tx_buffer, rx_buffer, 2);
    }

  icjx_deselect(priv->spi, priv->config);

  if (priv->config->verification)
    {
      if (rx_buffer[0] != startaddr)
        {
          gpioerr("ERROR: Data verification error for register 0x%x!\n",
                  reg);
          return -EIO;
        }

      if (rx_buffer[1] != ICJX_CONTROL)
        {
          gpioerr("ERROR: Control byte verification error for register"
                  "0x%x!\n", reg);
          return -EIO;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: icjx_write
 *
 * Description:
 *   Helper function for register write operation.
 *
 * Input Parameters:
 *   priv  - Pointer to icjx_dev_s structure
 *   reg   - Register offset (see icjx.h)
 *   data  - Data to be written
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int icjx_write(FAR struct icjx_dev_s *priv, uint8_t reg,
                      uint16_t data, int nob)
{
  uint8_t startaddr;
  uint8_t bytes_to_exchange;
  uint8_t tx_buffer[6];
  uint8_t rx_buffer[6];
  int ver_idx;
  int data_len;

  data_len = 1;
  ver_idx = 2;
  bytes_to_exchange = 0;
  startaddr =  (priv->config->addr << 6) | (reg << 1);
  tx_buffer[bytes_to_exchange++] = startaddr;
  tx_buffer[bytes_to_exchange++] = nob;
  tx_buffer[bytes_to_exchange++] = data & 0xff;
  if (nob == ICJX_NOB2)
    {
      data_len = 2;
      tx_buffer[bytes_to_exchange++] = (data >> 8) & 0xff;
    }

  if (priv->config->verification)
    {
      tx_buffer[bytes_to_exchange++] = (priv->config->addr << 6) |
                                       ((reg + (data_len - 1)) << 1);
      tx_buffer[bytes_to_exchange++] = ICJX_CONTROL;
    }

  icjx_select(priv->spi, priv->config, 8);
  SPI_EXCHANGE(priv->spi, tx_buffer, rx_buffer, bytes_to_exchange);
  icjx_deselect(priv->spi, priv->config);

  if (priv->config->verification)
    {
      if (rx_buffer[ver_idx++] != nob)
        {
          gpioerr("ERROR: Start address verification error for register"
                  "0x%x!\n", reg);
          return -EIO;
        }

      for (int i = 0; i < data_len; i++)
        {
          if (rx_buffer[ver_idx++] != ((data >> (i * 8)) & 0xff))
            {
              gpioerr("ERROR: Data verification error for register 0x%x!\n",
                      reg);
              return -EIO;
            }
        }

      if (rx_buffer[ver_idx++] != ICJX_CONTROL)
        {
          gpioerr("ERROR: Control verification error for register 0x%x!\n",
                  reg);
          return -EIO;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: icjx_direction
 *
 * Description:
 *   iC-JX is only input pin. However interface is provided in order
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

static int icjx_direction(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                          int dir)
{
  FAR struct icjx_dev_s *priv = (FAR struct icjx_dev_s *)dev;
  uint8_t outpins;
  uint16_t out_set;
  uint8_t regaddr;
  int ret;

  if (dir != IOEXPANDER_DIRECTION_IN &&
      dir != IOEXPANDER_DIRECTION_OUT)
    {
      return -EINVAL;
    }

  DEBUGASSERT(priv != NULL && priv->config != NULL && pin < 16);

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (dir == IOEXPANDER_DIRECTION_OUT)
    {
      priv->outpins |= (1 << pin);
    }
  else
    {
      priv->outpins &= ~(1 << pin);
    }

  if (pin < 8)
    {
      regaddr = ICJX_CTRL_WORD_2_A;
      outpins = priv->outpins & 0xff;
    }
  else
    {
      regaddr = ICJX_CTRL_WORD_2_B;
      outpins = (priv->outpins >> 8) & 0xff;
    }

  ret = icjx_read(priv, regaddr, &out_set, ICJX_NOB1);
  if (ret < 0)
    {
      nxmutex_unlock(&priv->lock);
      return ret;
    }

  /* Enable output generation */

  if ((outpins & 0xf) != 0)
    {
      out_set |= ICJX_CTRL_WORD_2_NIOL;
    }
  else
    {
      out_set &= ~ICJX_CTRL_WORD_2_NIOL;
    }

  if ((outpins & 0xf0) != 0)
    {
      out_set |= ICJX_CTRL_WORD_2_NIOH;
    }
  else
    {
      out_set &= ~ICJX_CTRL_WORD_2_NIOH;
    }

  ret = icjx_write(priv, regaddr, out_set, ICJX_NOB1);
  nxmutex_unlock(&priv->lock);

  return ret;
}

/****************************************************************************
 * Name: icjx_option
 *
 * Description:
 *   Set pin options. Required.
 *   Since all IO expanders have various pin options, this API allows setting
 *     pin options in a flexible way.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   pin   - The index of the pin to alter in this call
 *   opt   - One of the IOEXPANDER_OPTION_ macros
 *   value - The option's value
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int icjx_option(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                       int opt, FAR void *value)
{
  /* TODO: Implementation of iC-JX options should be here. This includes
   * setup of filters, ADC etc. The right way to implement
   * this would probably be to introduce config structure to
   * include/nuttx/ioexpanders/icjx.h that the user could use for the
   * nibbles configuration.
   *
   * Currently only interrupts are implemented.
   */

  FAR struct icjx_dev_s *priv = (FAR struct icjx_dev_s *)dev;
  uint8_t data;
  uint8_t reg;
  int ret;

  DEBUGASSERT(priv != NULL && priv->config != NULL);

  gpioinfo("Expander id=%02x pin=%u option=%u\n",
           priv->config->id, pin, opt);

  if (opt == IOEXPANDER_OPTION_INTCFG)
    {
      unsigned int ival = (unsigned int)((uintptr_t)value);
      ret = nxmutex_lock(&priv->lock);
      if (ret < 0)
        {
          return ret;
        }

      switch (ival)
        {
          case IOEXPANDER_VAL_HIGH:
          case IOEXPANDER_VAL_LOW:
          case IOEXPANDER_VAL_RISING:
          case IOEXPANDER_VAL_FALLING:
          case IOEXPANDER_VAL_BOTH:
            priv->irqpins |= 1 << pin;
            break;
          case IOEXPANDER_VAL_DISABLE:
            priv->irqpins &= ~(1 << pin);
            break;
          default:
            nxmutex_unlock(&priv->lock);
            return -EINVAL;
        }

      /* We have to modify ICJX_CHNG_INT_EN_A or ICJX_CHNG_INT_EN_B
       * register.
       */

      if (pin < 8)
        {
          reg = ICJX_CHNG_INT_EN_A;
          data = priv->irqpins & 0xff;
        }
      else
        {
          reg = ICJX_CHNG_INT_EN_B;
          data = (priv->irqpins >> 8) & 0xff;
        }

      ret = icjx_write(priv, reg, data, ICJX_NOB1);
      if (ret < 0)
        {
          gpioerr("Cannot write to %s register\n.",
            reg == ICJX_CHNG_INT_EN_A ?
            "ICJX_CHNG_INT_EN_A" : "ICJX_CHNG_INT_EN_B");
        }

      nxmutex_unlock(&priv->lock);
    }
  else
    {
      return -ENOTTY;
    }

  return ret;
}

/****************************************************************************
 * Name: icjx_writepin
 *
 * Description:
 *   Set the pin level. Required.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   pin   - The index of the pin to alter in this call
 *   value - The pin level. Usually TRUE will set the pin high,
 *           except if OPTION_INVERT has been set on this pin.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int icjx_writepin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                         bool value)
{
  FAR struct icjx_dev_s *priv = (FAR struct icjx_dev_s *)dev;
  uint8_t outstate;
  uint8_t regaddr;
  int ret;

  if (pin > 16)
    {
      return -ENXIO;
    }

  DEBUGASSERT(priv != NULL && priv->config != NULL);

  gpioinfo("Expander id=%02x, pin=%u\n", priv->config->id, pin);

  /* Get exclusive access to the I/O Expander */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  if ((priv->outpins & (1 << pin)) == 0)
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

  if (pin < 8)
    {
      regaddr = ICJX_OUTPUT_A;
      outstate = priv->outstate & 0xff;
    }
  else
    {
      regaddr = ICJX_OUTPUT_B;
      outstate = (priv->outstate >> 8) & 0xff;
    }

  ret = icjx_write(priv, regaddr, outstate, ICJX_NOB1);
  nxmutex_unlock(&priv->lock);

  return ret;
}

/****************************************************************************
 * Name: icjx_readpin
 *
 * Description:
 *   Read the actual PIN level. This can be different from the last value
 *   written to this pin. Required.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   pin    - The index of the pin
 *   value  - Pointer to a buffer where the pin level is stored. Usually
 *            TRUE if the pin is high, except if OPTION_INVERT has been
 *            set this pin.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int icjx_readpin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                        FAR bool *value)
{
  FAR struct icjx_dev_s *priv = (FAR struct icjx_dev_s *)dev;
  uint8_t regaddr;
  uint16_t data;
  int ret;

  if (pin > 16)
    {
      return -ENXIO;
    }

  DEBUGASSERT(priv != NULL && priv->config != NULL && value != NULL);

  gpioinfo("Expander addr=%02x, pin=%u\n", priv->config->addr, pin);

  /* Get exclusive access to the I/O Expander */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Make sure that this is an output pin */

  if ((priv->outpins & (1 << pin)) != 0)
    {
      *value = ((priv->outstate & (1 << pin)) != 0);
      nxmutex_unlock(&priv->lock);
      return OK;
    }

  regaddr = pin < 8 ? ICJX_INPUT_A : ICJX_INPUT_B;

  ret = icjx_read(priv, regaddr, &data, ICJX_NOB1);
  nxmutex_unlock(&priv->lock);
  if (ret != OK)
    {
      return ret;
    }

  *value = (bool)((data >> (pin & 0xf)) & 1);
  return OK;
}

/****************************************************************************
 * Name: icjx_multiwritepin
 *
 * Description:
 *   Set the pin level for multiple pins. This routine may be faster than
 *   individual pin accesses. Optional.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   pins   - The list of pin indexes to alter in this call
 *   values - The list of pin levels.
 *   count  - Number of pins to be written
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

#ifdef CONFIG_IOEXPANDER_MULTIPIN
static int icjx_multiwritepin(FAR struct ioexpander_dev_s *dev,
                              FAR const uint8_t *pins,
                              FAR const bool *values, int count)
{
  FAR struct icjx_dev_s *priv = (FAR struct icjx_dev_s *)dev;
  int ret;
  int pin;
  int value;

  if (count >= 16)
    {
      return -ENXIO;
    }

  DEBUGASSERT(priv != NULL && priv->config != NULL);

  /* Get exclusive access to the I/O Expander */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  for (int i = 0; i < count; i++)
    {
      pin = pins[i];
      value = values[i];

      gpioinfo("Expander id=%02x, pin=%u\n", priv->config->id, pin);

      if ((priv->outpins & (1 << pin)) == 0)
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
    }

  ret = icjx_write(priv, ICJX_OUTPUT_A, priv->outstate, ICJX_NOB2);
  nxmutex_unlock(&priv->lock);

  return ret;
}

/****************************************************************************
 * Name: icjx_multireadpin
 *
 * Description:
 *   Read the actual level for multiple pins. This routine may be faster than
 *   individual pin accesses. Optional.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   pin    - The list of pin indexes to read
 *   values - Pointer to a buffer where the pin levels are stored.
 *   count  - Number of pins to be read
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int icjx_multireadpin(FAR struct ioexpander_dev_s *dev,
                             FAR const uint8_t *pins, FAR bool *values,
                             int count)
{
  FAR struct icjx_dev_s *priv = (FAR struct icjx_dev_s *)dev;
  uint16_t data;
  int pin;
  int ret;

  if (count > 16)
    {
      return -ENXIO;
    }

  DEBUGASSERT(priv != NULL && priv->config != NULL && values != NULL);

  /* Get exclusive access to the I/O Expander */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  for (int i = 0; i < count; i++)
    {
      pin = pins[i];

      gpioinfo("Expander id=%02x, pin=%u\n", priv->config->id, pin);

      if ((priv->outpins & (1 << pin)) != 0)
        {
           values[i] = (bool)((priv->outstate & (1 << pin)) != 0);
        }
    }

  ret = icjx_read(priv, ICJX_INPUT_A, &data, ICJX_NOB2);
  nxmutex_unlock(&priv->lock);
  if (ret != OK)
    {
      return ret;
    }

  for (int i = 0; i < count; i++)
    {
      values[i] = (bool)((data >> (pins[i] & 0xf)) & 1);
    }

  return OK;
}
#endif /* CONFIG_IOEXPANDER_MULTIPIN */

/****************************************************************************
 * Name: icjx_attach
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

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
static FAR void *icjx_attach(FAR struct ioexpander_dev_s *dev,
                             ioe_pinset_t pinset, ioe_callback_t callback,
                             FAR void *arg)
{
  FAR struct icjx_dev_s *priv = (FAR struct icjx_dev_s *)dev;
  FAR void *handle = NULL;
  int ret;

  /* Get exclusive access */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return NULL;
    }

  if (priv->callback.function == NULL)
    {
      /* Yes.. use this entry */

      priv->callback.pinset   = pinset;
      priv->callback.function = callback;
      priv->callback.arg      = arg;
      handle                  = &priv->callback;
    }

  /* Add this callback to the table */

  nxmutex_unlock(&priv->lock);
  return handle;
}

/****************************************************************************
 * Name: icjx_detach
 *
 * Description:
 *   Detach and disable a pin interrupt callback function.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   handle   - The non-NULL opaque value return by pca9555_attch()
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int icjx_detach(FAR struct ioexpander_dev_s *dev, FAR void *handle)
{
  FAR struct icjx_callback_s *cb =
    (FAR struct icjx_callback_s *)handle;

  cb->pinset = 0;
  cb->function = NULL;
  cb->arg  = NULL;
  return OK;
}

/****************************************************************************
 * Name: icjx_interrupt_worker
 *
 * Description:
 *   Handle GPIO interrupt events (this function actually executes in the
 *   context of the worker thread).
 *
 ****************************************************************************/

static void icjx_interrupt_worker(void *arg)
{
  FAR struct icjx_dev_s *priv = (FAR struct icjx_dev_s *)arg;
  uint8_t regaddr;
  uint16_t change_of_input;
  uint16_t isr;
  ioe_pinset_t irq_match;
  int ret;

  /* Read interrupt status register */

  icjx_read(priv, ICJX_INT_STATUS_A, &isr, ICJX_NOB2);
  while (isr != 0)
    {
      if ((isr & ICJX_ISR_A_DCHI) != 0)
        {
          ret = icjx_read(priv, ICJX_CHNG_MSG_A, &change_of_input,
                          ICJX_NOB2);
          if (ret == OK)
            {
              irq_match = change_of_input & priv->callback.pinset;
              if (irq_match != 0)
                {
                  /* Change of input.. perform the callback */

                  priv->callback.function(&priv->dev, irq_match,
                                          priv->callback.arg);
                }
            }
        }

      /* Clear interrupt and check ISR again */

      icjx_write(priv, ICJX_CTRL_WORD_4, ICJX_CTRL_WORD_4_EOI, ICJX_NOB1);
      icjx_read(priv, ICJX_INT_STATUS_A, &isr, ICJX_NOB2);
    }
}

/****************************************************************************
 * Name: icjx_interrupt
 *
 * Description:
 *   Handle GPIO interrupt events (this function executes in the
 *   context of the interrupt).
 *
 ****************************************************************************/

static int icjx_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct icjx_dev_s *priv = (FAR struct icjx_dev_s *)arg;

  /* Create HP work to handle the interrupt. We do not want to do
   * this in the interrupt handler because SPI communication speed.
   */

  DEBUGASSERT(work_available(&priv->work));
  DEBUGVERIFY(work_queue(HPWORK, &priv->work, icjx_interrupt_worker,
                        (void *)priv, 0));

  return OK;
}

#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icjx_initialize
 *
 * Description:
 *   Instantiate and configure the ICJXxx device driver to use the
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

FAR struct ioexpander_dev_s *icjx_initialize(FAR struct spi_dev_s *spi,
                                      FAR struct icjx_config_s *config)
{
  FAR struct icjx_dev_s *priv;
  uint8_t regval;
  int ret;

#ifdef CONFIG_ICJX_MULTIPLE
  /* Allocate the device state structure */

  priv = kmm_zalloc(sizeof(struct icjx_dev_s));
  if (!priv)
    {
      gpioerr("ERROR: Failed to allocate driver instance\n");
      return NULL;
    }
#else
  /* Use the one-and-only I/O Expander driver instance */

  priv = &g_icjx;
#endif

  /* Initialize the device state structure */

  priv->dev.ops  = &g_icjx_ops;
  priv->spi      = spi;
  priv->config   = config;
  priv->outpins  = 0;
  priv->outstate = 0;
  priv->irqpins  = 0;

#ifdef CONFIG_ICJX_INT_ENABLE
  config->attach(config, icjx_interrupt, priv);
  config->enable(config, true);
#endif

  nxmutex_init(&priv->lock);

  /* Set pull up/down based on expander configuration. This is the first
   * time we are accessing the expander therefore we do not have to worry
   * about reading the register content first.
   */

  regval = (config->current_src << 4) | config->current_src;

  ret = icjx_write(priv, ICJX_CTRL_WORD_2_A, regval, ICJX_NOB1);
  if (ret < 0)
    {
      gpioerr("ERROR: Could write to ICJX_CTRL_WORD_2_A: %d!\n", ret);
      goto err;
    }

  ret = icjx_write(priv, ICJX_CTRL_WORD_2_B, regval, ICJX_NOB1);
  if (ret < 0)
    {
      gpioerr("ERROR: Could write to ICJX_CTRL_WORD_2_B: %d!\n", ret);
      goto err;
    }

  /* Bypass filters as those are not yet supported. */

  regval = ICJX_CTRL_WORD_1_BYP0 | ICJX_CTRL_WORD_1_BYP1;
  ret = icjx_write(priv, ICJX_CTRL_WORD_1_A, regval, ICJX_NOB1);
  if (ret < 0)
    {
      gpioerr("ERROR: Could write to ICJX_CTRL_WORD_1_A: %d!\n", ret);
      goto err;
    }

  ret = icjx_write(priv, ICJX_CTRL_WORD_1_B, regval, ICJX_NOB1);
  if (ret < 0)
    {
      gpioerr("ERROR: Could write to ICJX_CTRL_WORD_1_B: %d!\n", ret);
      goto err;
    }

  /* Clear initial interrupts if any */

  icjx_write(priv, ICJX_CTRL_WORD_4, ICJX_CTRL_WORD_4_EOI, ICJX_NOB1);

  return &priv->dev;

err:
  nxmutex_destroy(&priv->lock);
#ifdef CONFIG_ICJX_MULTIPLE
  kmm_free(priv);
#endif
  return NULL;
}

#endif /* CONFIG_IOEXPANDER_ICJX */
