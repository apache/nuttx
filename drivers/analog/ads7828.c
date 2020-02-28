/****************************************************************************
 * drivers/analog/ads7828.c
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

#include <stdio.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>
#include <endian.h>

#include <nuttx/arch.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>
#include <nuttx/analog/ads7828.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_ADC_ADS7828)

#define ADS7828_NUM_CHANNELS            8

/* All conversions are started by writing a command byte to the ADS7828.
 * Below are the bit/ mask definitions for this byte.
 */

#define ADS7828_CMD_BYTE_MODE           (1 << 7)
#define ADS7828_CMD_BYTE_CHANNEL_SHIFT  4
#define ADS7828_CMD_BYTE_CHANNEL_MASK   (7 << ADS7828_CMD_BYTE_CHANNEL_SHIFT)
#define ADS7828_CMD_BYTE_REF            (1 << 3)
#define ADS7828_CMD_BYTE_PWR_DOWN       (1 << 2)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ads7828_dev_s
{
  FAR struct i2c_master_s           *i2c;
  FAR const struct adc_callback_s   *cb;
  uint8_t                           addr;

  /* List of channels to read on every convert trigger.
   * Bit position corresponds to channel. i.e. bit0 = 1 to read channel 0.
   */

  uint8_t                           chanstrobed;

  /* Current configuration of the ADC. Encoded in its position in the
   * command byte that is sent to the ADS7828 to trigger a conversion
   */

  uint8_t                           cmdbyte;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* ads7828 helpers */

static int ads7828_manage_strobe(FAR struct ads7828_dev_s *priv,
                                 uint8_t channel, bool add_nremove);
static int ads7828_readchannel(FAR struct ads7828_dev_s *priv,
                               FAR struct adc_msg_s *msg);

/* ADC methods */

static int  ads7828_bind(FAR struct adc_dev_s *dev,
                     FAR const struct adc_callback_s *callback);
static void ads7828_reset(FAR struct adc_dev_s *dev);
static int  ads7828_setup(FAR struct adc_dev_s *dev);
static void ads7828_shutdown(FAR struct adc_dev_s *dev);
static void ads7828_rxint(FAR struct adc_dev_s *dev, bool enable);
static int  ads7828_ioctl(FAR struct adc_dev_s *dev, int cmd,
                          unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct adc_ops_s g_adcops =
{
  .ao_bind     = ads7828_bind,      /* ao_bind */
  .ao_reset    = ads7828_reset,     /* ao_reset */
  .ao_setup    = ads7828_setup,     /* ao_setup */
  .ao_shutdown = ads7828_shutdown,  /* ao_shutdown */
  .ao_rxint    = ads7828_rxint,     /* ao_rxint */
  .ao_ioctl    = ads7828_ioctl      /* ao_read */
};

static struct ads7828_dev_s g_adcpriv;

static struct adc_dev_s g_adcdev =
{
  .ad_ops  = &g_adcops,
  .ad_priv = &g_adcpriv,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ads7828_manage_strobe
 *
 * Description:
 *   Controls which channels are read on ANIOC_TRIGGER. By default all
 *   channels are read.
 *
 * Returned Value:
 *   0 on success. Negated errno on failure.
 *
 ****************************************************************************/

static int ads7828_manage_strobe(FAR struct ads7828_dev_s *priv,
                                 uint8_t channel, bool add_nremove)
{
  int ret = OK;

  if (priv == NULL || channel >= ADS7828_NUM_CHANNELS)
    {
      ret = -EINVAL;
    }
  else
    {
      uint8_t flag = 1U << channel;

      if (add_nremove)
        {
          priv->chanstrobed |= flag;
        }
       else
        {
          priv->chanstrobed &= ~flag;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: ads7828_readchannel
 *
 * Description:
 *   Reads a conversion from the ADC.
 *
 * Input Parameters:
 *   msg - msg->am_channel should be set to the channel to be read.
 *         msg->am_data will store the result of the read.
 *
 * Returned Value:
 *   0 on success. Negated errno on failure.
 *
 * Assumptions/Limitations:
 *   NOTE: When used in single-ended mode, msg->am_channel will be converted
 *         to the corresponding channel selection bits in the command byte.
 *         In differential mode, msg->am_channel is used as the channel
 *         selection bits. The corresponding "channels" are as follows:
 *
 *         msg->am_channel  Analog Source
 *         0                +CH0, -CH1
 *         1                +CH2, -CH3
 *         2                +CH4, -CH5
 *         3                +CH6, -CH7
 *         4                +CH1, -CH0
 *         5                +CH3, -CH2
 *         6                +CH5, -CH4
 *         7                +CH7, -CH6
 *
 ****************************************************************************/

static int ads7828_readchannel(FAR struct ads7828_dev_s *priv,
                               FAR struct adc_msg_s *msg)
{
  int ret = OK;
  if (priv == NULL || msg == NULL)
    {
      ret = -EINVAL;
    }
  else
    {
      struct i2c_msg_s i2cmsg[2];
      uint8_t channel = msg->am_channel;

      /* Convert single-ended channels to write encoding. */

      if (priv->cmdbyte & ADS7828_CMD_BYTE_MODE)
        {
          channel = channel / 2;

          if (msg->am_channel % 2 != 0)
            {
              channel += 4;
            }
        }

      uint8_t cmdbyte = channel << ADS7828_CMD_BYTE_CHANNEL_SHIFT;
      cmdbyte |= priv->cmdbyte;

      i2cmsg[0].frequency = CONFIG_ADS7828_FREQUENCY;
      i2cmsg[0].addr = priv->addr;
      i2cmsg[0].flags = I2C_M_NOSTOP;
      i2cmsg[0].buffer = &cmdbyte;
      i2cmsg[0].length = sizeof(cmdbyte);

      i2cmsg[1].frequency = CONFIG_ADS7828_FREQUENCY;
      i2cmsg[1].addr = priv->addr;
      i2cmsg[1].flags = I2C_M_READ;

      uint16_t buf;
      i2cmsg[1].buffer = (uint8_t *)(&buf);
      i2cmsg[1].length = sizeof(buf);
      ret = I2C_TRANSFER(priv->i2c, i2cmsg, 2);
      if (ret < 0)
        {
            aerr("ADS7828 I2C transfer failed: %d", ret);
        }

      msg->am_data = be16toh(buf);
    }

  return ret;
}

/****************************************************************************
 * Name: ads7828_bind
 *
 * Description:
 *   Bind the upper-half driver callbacks to the lower-half implementation.
 *   This must be called early in order to receive ADC event notifications.
 *
 ****************************************************************************/

static int ads7828_bind(FAR struct adc_dev_s *dev,
                        FAR const struct adc_callback_s *callback)
{
  FAR struct ads7828_dev_s *priv = (FAR struct ads7828_dev_s *)dev->ad_priv;

  DEBUGASSERT(priv != NULL);
  priv->cb = callback;
  return OK;
}

/****************************************************************************
 * Name: ads7828_reset
 *
 * Description:
 *   Reset the ADC device.  Called early to initialize the hardware. This
 *   is called, before ao_setup() and on error conditions.
 *
 ****************************************************************************/

static void ads7828_reset(FAR struct adc_dev_s *dev)
{
    FAR struct ads7828_dev_s *priv =
                                (FAR struct ads7828_dev_s *)dev->ad_priv;

    priv->cmdbyte = 0;
    priv->chanstrobed = 0xffu;
}

/****************************************************************************
 * Name: ads7828_setup
 *
 * Description:
 *   Configure the ADC. This method is called the first time that the ADC
 *   device is opened.  The ADS7828 is quite simple and nothing special is
 *   needed to be done.
 *
 ****************************************************************************/

static int ads7828_setup(FAR struct adc_dev_s *dev)
{
  return OK;
}

/****************************************************************************
 * Name: ads7828_shutdown
 *
 * Description:
 *   Disable the ADC.  This method is called when the ADC device is closed.
 *   This method should reverse the operation of the setup method, but as
 *   the ADS7828 is quite simple does not need to do anything.
 *
 ****************************************************************************/

static void ads7828_shutdown(FAR struct adc_dev_s *dev)
{
}

/****************************************************************************
 * Name: ads7828_rxint
 *
 * Description:
 *   Needed for ADC upper-half compatibility but conversion interrupts
 *   are not supported by the ADC7828.
 *
 ****************************************************************************/

static void ads7828_rxint(FAR struct adc_dev_s *dev, bool enable)
{
}

/****************************************************************************
 * Name: ads7828_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int ads7828_ioctl(FAR struct adc_dev_s *dev, int cmd,
                         unsigned long arg)
{
  FAR struct ads7828_dev_s *priv = (FAR struct ads7828_dev_s *)dev->ad_priv;
  int ret = OK;

  switch (cmd)
    {
      case ANIOC_TRIGGER:
        {
          struct adc_msg_s msg;
          int i;

          for (i = 0; (i < ADS7828_NUM_CHANNELS) && (ret == OK); i++)
            {
              if ((priv->chanstrobed >> i) & 1u)
                {
                  msg.am_channel = i;
                  ret = ads7828_readchannel(priv, &msg);
                  if (ret == OK)
                    {
                      priv->cb->au_receive(&g_adcdev, i, msg.am_data);
                    }
                }
            }
        }
        break;

      /* Add a channel to list of channels read on ANIOC_TRIGGER */

      case ANIOC_ADS7828_ADD_CHAN:
        {
          ret = ads7828_manage_strobe(priv, (uint8_t)arg, true);
        }
        break;

      /* Remove a channel from list of channels read on ANIOC_TRIGGER */

      case ANIOC_ADS7828_REMOVE_CHAN:
        {
          ret = ads7828_manage_strobe(priv, (uint8_t)arg, false);
        }
        break;

      /* Read a single channel from the ADC */

      case ANIOC_ADS7828_READ_CHANNEL:
        {
          FAR struct adc_msg_s *msg = (FAR struct adc_msg_s *)arg;
          ret = ads7828_readchannel(priv, msg);
        }
        break;

      /* Set the ADC reference voltage */

      case ANIOC_ADS7828_SET_REF:
        {
          if (arg == ADS7828_REF_INTERNAL)
            {
              priv->cmdbyte |= ADS7828_CMD_BYTE_REF;
            }
          else if (arg == ADS7828_REF_EXTERNAL)
            {
              priv->cmdbyte &= ~ADS7828_CMD_BYTE_REF;
            }
          else
            {
              ret = -EINVAL;
            }
        }
        break;

      /* Set mode to single-ended or differential-mode */

      case ANIOC_ADS7828_MODE:
        {
          if (arg == ADS7828_SINGLE_ENDED)
            {
              priv->cmdbyte |= ADS7828_CMD_BYTE_MODE;
            }
          else if (arg == ADS7828_DIFFERENTIAL)
            {
              priv->cmdbyte &= ~ADS7828_CMD_BYTE_MODE;
            }
          else
            {
              ret = -EINVAL;
            }
        }
        break;

      /* Set whether to power down ADC between conversions or not */

      case ANIOC_ADS7828_POWER_SAVE:
        {
          if ((bool)arg)
            {
              priv->cmdbyte &= ~ADS7828_CMD_BYTE_PWR_DOWN;
            }
          else
            {
              priv->cmdbyte |= ADS7828_CMD_BYTE_PWR_DOWN;
            }
        }
        break;

      /* Command was not recognized */

      default:
      ret = -ENOTTY;
      aerr("ADS7828 ERROR: Unrecognized cmd: %d\n", cmd);
      break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ads7828_initialize
 *
 * Description:
 *   Initialize the selected adc
 *
 * Input Parameters:
 *   i2c - Pointer to a I2C master struct for the bus the ADC resides on.
 *   addr - I2C address of the ADC
 *
 * Returned Value:
 *   Valid ADC device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

FAR struct adc_dev_s *ads7828_initialize(FAR struct i2c_master_s *i2c,
                                               uint8_t addr)
{
  DEBUGASSERT(i2c != NULL);

  /* Driver state data */

  FAR struct ads7828_dev_s *priv;
  priv = (FAR struct ads7828_dev_s *)g_adcdev.ad_priv;

  priv->cb   = NULL;
  priv->i2c  = i2c;
  priv->addr = addr;
  priv->cmdbyte = 0;
  priv->chanstrobed = 0xffu;

  return &g_adcdev;
}

#endif
