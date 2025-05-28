/****************************************************************************
 * drivers/analog/ads1115.c
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
#include <nuttx/nuttx.h>
#include <nuttx/signal.h>

#include <assert.h>
#include <debug.h>
#include <endian.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/types.h>

#include <nuttx/analog/adc.h>
#include <nuttx/analog/ads1115.h>
#include <nuttx/analog/ioctl.h>
#include <nuttx/arch.h>
#include <nuttx/i2c/i2c_master.h>

/****************************************************************************
 * Preprocessor definitions
 ****************************************************************************/

#if defined(CONFIG_ADC_ADS1115)

#ifndef CONFIG_ADC_ADS1115_I2C_FREQUENCY
#define CONFIG_ADC_ADS1115_I2C_FREQUENCY 100000
#endif

#define ADS1115_NUM_CHANNELS 8

#define ADS1115_OS_SHIFT (1 << 15)
#define ADS1115_MUX_SHIFT 12
#define ADS1115_MUX_MASK (7 << ADS1115_MUX_SHIFT)
#define ADS1115_PGA_SHIFT 9
#define ADS1115_PGA_MASK (7 << ADS1115_PGA_SHIFT)
#define ADS1115_MODE_MASK (1 << 8)
#define ADS1115_DR_SHIFT 5
#define ADS1115_DR_MASK (7 << ADS1115_DR_SHIFT)
#define ADS1115_COMP_MODE_MASK (1 << 4)
#define ADS1115_COMP_POL_MASK (1 << 3)
#define ADS1115_COMP_LAT_MASK (1 << 2)
#define ADS1115_COMP_QUE_SHIFT 0
#define ADS1115_COMP_QUE_MASK (3 << ADS1115_COMP_QUE_SHIFT)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ads1115_dev_s
{
  FAR struct i2c_master_s *i2c;
  FAR const struct adc_callback_s *cb;
  uint8_t addr;

  /* Current configuration of ADC. */

  uint16_t cmdbyte;
};

enum ads1115_registers_e
{
  ADS1115_CONVERSION_REGISTER = 0x00,
  ADS1115_CONFIG_REGISTER = 0x01,
  ADS1115_LO_THRESH_REGISTER = 0x02,
  ADS1115_HI_THRESH_REGISTER = 0x03,
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int ads1115_bind(FAR struct adc_dev_s *dev,
                        FAR const struct adc_callback_s *callback);
static void ads1115_reset(FAR struct adc_dev_s *dev);
static int ads1115_setup(FAR struct adc_dev_s *dev);
static void ads1115_shutdown(FAR struct adc_dev_s *dev);
static void ads1115_rxint(FAR struct adc_dev_s *dev, bool enable);
static int ads1115_ioctl(FAR struct adc_dev_s *dev, int cmd,
                         unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct adc_ops_s g_ads1115ops =
{
    .ao_bind = ads1115_bind,
    .ao_reset = ads1115_reset,
    .ao_setup = ads1115_setup,
    .ao_shutdown = ads1115_shutdown,
    .ao_rxint = ads1115_rxint,
    .ao_ioctl = ads1115_ioctl,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ads1115_write_register
 *
 * Description:
 *   Writes a value to a register in the ADS1115.
 *
 * Input Parameters:
 *   priv - An ADS1115 device structure
 *   reg - The register to write to.
 *   value - The value to write to the register, in big endian.
 *
 ****************************************************************************/

static int ads1115_write_register(FAR struct ads1115_dev_s *priv,
                                  enum ads1115_registers_e reg,
                                  uint16_t value)
{
  int ret = OK;

  if (priv == NULL)
    {
      ret = -EINVAL;
    }
  else
    {
      struct i2c_msg_s i2cmsg[2];

      /* Write the register into the address pointer register. */

      i2cmsg[0].frequency = CONFIG_ADC_ADS1115_I2C_FREQUENCY;
      i2cmsg[0].addr = priv->addr;
      i2cmsg[0].flags = 0;
      i2cmsg[0].buffer = &reg;
      i2cmsg[0].length = sizeof(reg);

      /* Write the value into the register. */

      i2cmsg[1].frequency = CONFIG_ADC_ADS1115_I2C_FREQUENCY;
      i2cmsg[1].addr = priv->addr;
      i2cmsg[1].flags = I2C_M_NOSTART;
      i2cmsg[1].buffer = (FAR uint8_t *)&value;
      i2cmsg[1].length = sizeof(value);

      ret = I2C_TRANSFER(priv->i2c, i2cmsg, 2);
      if (ret < 0)
        {
          aerr("ADS1115 I2C transfer failed: %d\n", ret);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: ads1115_read_register
 *
 * Description:
 *  Reads a value from a register in the ADS1115 in big endian.
 *
 * Input Parameters:
 *  priv - An ADS1115 device structure
 *  reg - The register to read from.
 *  buf - A pointer to a buffer to store the value read from the register.
 *
 ****************************************************************************/

static int ads1115_read_register(FAR struct ads1115_dev_s *priv, uint8_t reg,
                                 uint16_t *buf)
{
  int ret = OK;

  if (priv == NULL || buf == NULL)
    {
      ret = -EINVAL;
    }
  else
    {
      struct i2c_msg_s i2cmsg[2];

      /* Write the register into the address pointer register. */

      i2cmsg[0].frequency = CONFIG_ADC_ADS1115_I2C_FREQUENCY;
      i2cmsg[0].addr = priv->addr;
      i2cmsg[0].flags = 0;
      i2cmsg[0].buffer = (FAR uint8_t *)(&reg);
      i2cmsg[0].length = sizeof(reg);

      /* Read from the register. */

      i2cmsg[1].frequency = CONFIG_ADC_ADS1115_I2C_FREQUENCY;
      i2cmsg[1].addr = priv->addr;
      i2cmsg[1].flags = I2C_M_READ;
      i2cmsg[1].buffer = (FAR uint8_t *)(buf);
      i2cmsg[1].length = sizeof(*buf);

      ret = I2C_TRANSFER(priv->i2c, i2cmsg, 2);
      if (ret < 0)
        {
          aerr("ADS1115 I2C transfer failed: %d\n", ret);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: ads1115_read_current_register
 *
 * Description:
 *  Reads from the current register in the ADS1115 in big endian.
 *
 * Input Parameters:
 *  priv - An ADS1115 device structure
 *  buf - A pointer to a buffer to store the value read from the register.
 *
 ****************************************************************************/

static int ads1115_read_current_register(FAR struct ads1115_dev_s *priv,
                                         uint16_t *buf)
{
  int ret = OK;

  if (priv == NULL || buf == NULL)
    {
      ret = -EINVAL;
    }
  else
    {
      struct i2c_msg_s i2cmsg[1];

      i2cmsg[0].frequency = CONFIG_ADC_ADS1115_I2C_FREQUENCY;
      i2cmsg[0].addr = priv->addr;
      i2cmsg[0].flags = I2C_M_READ;
      i2cmsg[0].buffer = (FAR uint8_t *)(buf);
      i2cmsg[0].length = sizeof(*buf);

      ret = I2C_TRANSFER(priv->i2c, i2cmsg, 1);
      if (ret < 0)
        {
          aerr("ADS1115 I2C transfer failed: %d\n", ret);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: cmdbyte_init
 *
 * Description:
 *  Initializes the command byte with the default settings, and writes to the
 *  high and low threshold registers.
 *
 * Input Parameters:
 *  priv - An ADS1115 device structure
 *
 ****************************************************************************/

static int cmdbyte_init(FAR struct ads1115_dev_s *priv)
{
  int ret = OK;

  /* Set the default channel. */

  priv->cmdbyte = CONFIG_ADC_ADS1115_CHANNEL << ADS1115_MUX_SHIFT;

  /* Set the default pga setting. */

  priv->cmdbyte |= CONFIG_ADC_ADS1115_PGA << ADS1115_PGA_SHIFT;

  /* Set the default mode. */

#ifndef CONFIG_ADC_ADS1115_CONTINOUS
  priv->cmdbyte |= ADS1115_MODE_MASK;
#endif

  /* Set the default data rate. */

  priv->cmdbyte |= CONFIG_ADC_ADS1115_DR << ADS1115_DR_SHIFT;

  /* Set the default comparator mode. */

#ifdef CONFIG_ADC_ADS1115_COMP_MODE
  priv->cmdbyte |= ADS1115_COMP_MODE_MASK;
#endif

  /* Set the default comparator polarity. */

#ifdef CONFIG_ADC_ADS1115_COMP_POL
  priv->cmdbyte |= ADS1115_COMP_POL_MASK;
#endif

  /* Set the default comparator latching. */

#ifdef CONFIG_ADC_ADS1115_COMP_LAT
  priv->cmdbyte |= ADS1115_COMP_LAT_MASK;
#endif

  /* Set the comparator queue. */

  priv->cmdbyte |= CONFIG_ADC_ADS1115_COMP_QUE << ADS1115_COMP_QUE_SHIFT;

  ret = ads1115_write_register(priv, ADS1115_HI_THRESH_REGISTER,
                               htobe16(CONFIG_ADC_ADS1115_HI_THRESH));
  if (ret != OK)
    {
      aerr("Failed to write high threshold register\n");
      return ret;
    }

  ret = ads1115_write_register(priv, ADS1115_LO_THRESH_REGISTER,
                               htobe16(CONFIG_ADC_ADS1115_LO_THRESH));

  return ret;
}

/****************************************************************************
 * Name: ads1115_readchannel
 *
 * Description:
 *   Reads a conversion from the ADC.
 *
 * Input Parameters:
 *   priv - An ADS1115 device structure
 *   msg - An ADC message struct where the am_channel member contains the
 *   channel number to be read, and where the am_data member is where the
 *   reading is stored.
 *
 * NOTE:
 *   When switching between channels in continuous mode, old data may be
 *   read.  Using the ALERT/RDY pin can avoid this.
 *
 *   Each channel corresponds to a differing mux configuration as defined in
 *   the datasheet.
 *
 *   msg->am_channel  MUX Configuration
 *   0               AINP = AIN0, AINN = AIN1
 *   1               AINP = AIN0, AINN = AIN3
 *   2               AINP = AIN1, AINN = AIN3
 *   3               AINP = AIN2, AINN = AIN3
 *   4               AINP = AIN0, AINN = GND
 *   5               AINP = AIN0, AINN = GND
 *   6               AINP = AIN1, AINN = GND
 *   7               AINP = AIN2, AINN = GND
 *   8               AINP = AIN3, AINN = GND
 ****************************************************************************/

static int ads1115_readchannel(FAR struct ads1115_dev_s *priv,
                               FAR struct adc_msg_s *msg)
{
  int ret = OK;

  if (priv == NULL || msg == NULL || msg->am_channel >= ADS1115_NUM_CHANNELS)
    {
      ret = -EINVAL;
    }
  else
    {
      uint16_t buf;
      uint16_t channel_bits = msg->am_channel << ADS1115_MUX_SHIFT;

      bool one_shot =
          ((priv->cmdbyte & ADS1115_MODE_MASK) == ADS1115_MODE_MASK);
      bool new_channel =
          ((priv->cmdbyte & ADS1115_MUX_MASK) != channel_bits);

      /* check if our current channel is the same as the one that
       * is set or if in one shot mode.
       */

      if (new_channel || one_shot)
        {
          /* Clear the current mux bits. */

          priv->cmdbyte &= ~(ADS1115_MUX_MASK);

          /* Set the new mux bits. */

          priv->cmdbyte |= channel_bits;

          /* Set the OS bit to start conversion. */

          priv->cmdbyte |= ADS1115_OS_SHIFT;

          /* Write to the configuration register. */

          ainfo("cmdbyte: %x\n", priv->cmdbyte);
          ret = ads1115_write_register(priv, ADS1115_CONFIG_REGISTER,
                                       htobe16(priv->cmdbyte));
        }

      if (one_shot)
        {
          /* Read the configuration register until OS has been set to 1. */

          int count = 0;
          do
            {
              /* ADS1115 takes ~25 usec to wake up */

              up_udelay(4000);
              ret = ads1115_read_current_register(priv, &buf);
              count++;
            }
          while ((be16toh(buf) & ADS1115_OS_SHIFT) == 0 && ret == OK &&
                  count < 3);
          ainfo("config register: %x\n", (be16toh(buf)));

          if (count >= 3)
            {
              aerr("ADS1115 timed out waiting for conversion to finish\n");
              return -ETIMEDOUT;
            }
        }

      /* Read the conversion register. */

      ret = ads1115_read_register(priv, ADS1115_CONVERSION_REGISTER, &buf);
      ainfo("output: %d\n", (int16_t)betoh16(buf));
      msg->am_data = (uint32_t)betoh16(buf);

      priv->cmdbyte &= ~(ADS1115_OS_SHIFT); /* Clear the OS bit. */
    }

  return ret;
}

/****************************************************************************
 * Name: ads1115_bind
 *
 * Description:
 *   Bind the upper-half driver callbacks to the lower-half implementation.
 *   This must be called early in order to receive ADC event notifications.
 *
 ****************************************************************************/

static int ads1115_bind(FAR struct adc_dev_s *dev,
                        FAR const struct adc_callback_s *callback)
{
  FAR struct ads1115_dev_s *priv = (FAR struct ads1115_dev_s *)dev->ad_priv;

  DEBUGASSERT(priv != NULL);
  priv->cb = callback;
  return 0;
}

/****************************************************************************
 * Name: ads1115_reset
 *
 * Description:
 *   Reset the ADC device. Called early to initialize the hardware. This
 *   is called, before ao_setup() and on error conditions.
 *
 ****************************************************************************/

static void ads1115_reset(FAR struct adc_dev_s *dev)
{
  FAR struct ads1115_dev_s *priv = (FAR struct ads1115_dev_s *)dev->ad_priv;

  cmdbyte_init(priv);
}

/****************************************************************************
 * Name: ads1115_setup
 *
 * Description:
 *   Configure the ADC. This method is called the first time that the ADC
 *   device is opened. This method will write the default configuration into
 *   the configuration register.
 *
 ****************************************************************************/

static int ads1115_setup(FAR struct adc_dev_s *dev)
{
  FAR struct ads1115_dev_s *priv = (FAR struct ads1115_dev_s *)dev->ad_priv;

  int ret = ads1115_write_register(priv, ADS1115_CONFIG_REGISTER,
                                   htobe16(priv->cmdbyte));
  return ret;
}

/****************************************************************************
 * Name: ads1115_shutdown
 *
 * Description:
 *   Disable the ADC. This method is called when the ADC device is closed.
 *   This method should reverse the operation of the setup method. The
 *   ADS1115 can be put into a power-down state by setting the mode bit to
 *   single-shot. This will stop the ADC from converting and reduce power
 *   consumption.
 *
 ****************************************************************************/

static void ads1115_shutdown(FAR struct adc_dev_s *dev)
{
  FAR struct ads1115_dev_s *priv = (FAR struct ads1115_dev_s *)dev->ad_priv;

  priv->cmdbyte |= ADS1115_MODE_MASK;
  ads1115_write_register(priv, ADS1115_CONFIG_REGISTER,
                         htobe16(priv->cmdbyte));
}

/****************************************************************************
 * Name: ads1115_rxint
 *
 * Description:
 *   Needed for ADC upper-half compatibility but conversion interrupts
 *   are not supported by the ADS1115.
 *
 ****************************************************************************/

static void ads1115_rxint(FAR struct adc_dev_s *dev, bool enable)
{
}

/****************************************************************************
 * Name: ads1115_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method.
 *
 ****************************************************************************/

static int ads1115_ioctl(FAR struct adc_dev_s *dev, int cmd,
                         unsigned long arg)
{
  FAR struct ads1115_dev_s *priv = (FAR struct ads1115_dev_s *)dev->ad_priv;
  int ret = OK;

  switch (cmd)
    {
    case ANIOC_TRIGGER:
      {
        struct adc_msg_s msg;

        for (uint8_t i = 0; i < ADS1115_NUM_CHANNELS && (ret == 0); i++)
          {
            msg.am_channel = i;
            ainfo("Channel: %d\n", i);
            ret = ads1115_readchannel(priv, &msg);
            if (ret == 0)
              {
                priv->cb->au_receive(dev, i, msg.am_data);
              }
            else
              {
                aerr("Error reading channel %d with error %d \n", i, ret);
              }
          }
      }
      break;

    case ANIOC_ADS1115_SET_PGA:
      {
        if (arg >= ADS1115_PGA1 && arg <= ADS1115_PGA8)
          {
            priv->cmdbyte &= ~ADS1115_PGA_MASK;
            priv->cmdbyte |= arg << ADS1115_PGA_SHIFT;
          }
        else
          {
            ret = -EINVAL;
            break;
          }

        ret = ads1115_write_register(priv, ADS1115_CONFIG_REGISTER,
                                     htobe16(priv->cmdbyte));
      }
      break;

    case ANIOC_ADS1115_SET_MODE:
      {
        if (arg == ADS1115_MODE1)
          {
            priv->cmdbyte &= ~ADS1115_MODE_MASK;
          }
        else if (arg == ADS1115_MODE2)
          {
            priv->cmdbyte |= ADS1115_MODE_MASK;
          }
        else
          {
            ret = -EINVAL;
            break;
          }

        ret = ads1115_write_register(priv, ADS1115_CONFIG_REGISTER,
                                     htobe16(priv->cmdbyte));
      }
      break;

    case ANIOC_ADS1115_SET_DR:
      {
        if (arg >= ADS1115_DR1 && arg <= ADS1115_DR8)
          {
            priv->cmdbyte &= ~ADS1115_DR_MASK;
            priv->cmdbyte |= arg << ADS1115_DR_SHIFT;
          }
        else
          {
            ret = -EINVAL;
            break;
          }

        ret = ads1115_write_register(priv, ADS1115_CONFIG_REGISTER,
                                     htobe16(priv->cmdbyte));
      }
      break;

    case ANIOC_ADS1115_SET_COMP_POL:
      {
        if (arg == ADS1115_COMP_POL1)
          {
            priv->cmdbyte &= ~ADS1115_COMP_POL_MASK;
          }
        else if (arg == ADS1115_COMP_POL2)
          {
            priv->cmdbyte |= ADS1115_COMP_POL_MASK;
          }
        else
          {
            ret = -EINVAL;
            break;
          }

        ret = ads1115_write_register(priv, ADS1115_CONFIG_REGISTER,
                                     htobe16(priv->cmdbyte));
      }
      break;

    case ANIOC_ADS1115_SET_COMP_MODE:
      {
        if (arg == ADS1115_COMP_MODE1)
          {
            priv->cmdbyte &= ~ADS1115_COMP_MODE_MASK;
          }
        else if (arg == ADS1115_COMP_MODE2)
          {
            priv->cmdbyte |= ADS1115_COMP_MODE_MASK;
          }
        else
          {
            ret = -EINVAL;
            break;
          }

        ret = ads1115_write_register(priv, ADS1115_CONFIG_REGISTER,
                                     htobe16(priv->cmdbyte));
      }
      break;

    case ANIOC_ADS1115_SET_COMP_LAT:
      {
        if (arg == ADS1115_COMP_LAT1)
          {
            priv->cmdbyte &= ~ADS1115_COMP_LAT_MASK;
          }
        else if (arg == ADS1115_COMP_LAT2)
          {
            priv->cmdbyte |= ADS1115_COMP_LAT_MASK;
          }
        else
          {
            ret = -EINVAL;
            break;
          }

        ret = ads1115_write_register(priv, ADS1115_CONFIG_REGISTER,
                                     htobe16(priv->cmdbyte));
      }
      break;

    case ANIOC_ADS1115_SET_COMP_QUEUE:
      {
        if (arg >= ADS1115_COMP_QUEUE1 && arg <= ADS1115_COMP_QUEUE4)
          {
            priv->cmdbyte &= ~ADS1115_COMP_QUE_MASK;
            priv->cmdbyte |= arg << ADS1115_COMP_QUE_SHIFT;
          }
        else
          {
            ret = -EINVAL;
            break;
          }

        ret = ads1115_write_register(priv, ADS1115_CONFIG_REGISTER,
                                     htobe16(priv->cmdbyte));
      }
      break;

    case ANIOC_ADS1115_READ_CHANNEL:
      {
        FAR struct adc_msg_s *msg = (FAR struct adc_msg_s *)arg;
        ret = ads1115_readchannel(priv, msg);
      }
      break;

    case ANIOC_ADS1115_SET_HI_THRESH:
      {
        if (arg >= 0 && arg <= UINT16_MAX)
          {
            ret = ads1115_write_register(priv, ADS1115_HI_THRESH_REGISTER,
                                         htobe16(arg));
          }
        else
          {
            ret = -EINVAL;
          }
      }
      break;

    case ANIOC_ADS1115_SET_LO_THRESH:
      {
        if (arg >= 0 && arg <= UINT16_MAX)
          {
            ret = ads1115_write_register(priv, ADS1115_LO_THRESH_REGISTER,
                                         htobe16(arg));
          }
        else
          {
            ret = -EINVAL;
          }
      }
      break;

    case ANIOC_GET_NCHANNELS:
      {
        ret = ADS1115_NUM_CHANNELS;
      }
      break;

    default:
      ret = -ENOTTY;
      break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ads1115_initialize
 *
 * Description:
 *  Initialize the selected ADC
 *
 * Input Parameters:
 *  i2c - Pointer to a valid I2C master struct.
 *  addr - I2C device address.
 *
 * Returned Value:
 *   Valid ADC device structure reference on success; a NULL on failure
 *
 * **************************************************************************/

FAR struct adc_dev_s *ads1115_initialize(FAR struct i2c_master_s *i2c,
                                         uint8_t addr)
{
  FAR struct ads1115_dev_s *priv;
  FAR struct adc_dev_s *adcdev;

  DEBUGASSERT(i2c != NULL);

  priv = kmm_malloc(sizeof(struct ads1115_dev_s));
  if (priv == NULL)
    {
      aerr("ERROR: Failed to allocate ads1115_dev_s instance\n");
      free(priv);
      return NULL;
    }

  priv->cb = NULL;
  priv->i2c = i2c;
  priv->addr = addr;
  priv->cmdbyte = 0;

  int ret = cmdbyte_init(priv);
  if (ret != OK)
    {
      aerr("Failed to initialize ADS1115\n");
      free(priv);
      return NULL;
    }

  adcdev = kmm_malloc(sizeof(struct adc_dev_s));
  if (adcdev == NULL)
    {
      aerr("ERROR: Failed to allocate adc_dev_s instance\n");
      return NULL;
    }

  memset(adcdev, 0, sizeof(struct adc_dev_s));
  adcdev->ad_ops = &g_ads1115ops;
  adcdev->ad_priv = priv;

  return adcdev;
}

#endif /* CONFIG_ADC_ADS1115 */
