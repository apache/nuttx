/****************************************************************************
 * drivers/analog/max1161x.c
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
 * This driver is for a whole family of I2C ADC chips:
 * +--------------+-----+----+--------+---------+
 * | Type         | VCC | In | Pkg    | I2C Addr|
 * +--------------+-----+----+--------+---------+
 * | MAX11612EUA+ | 5V0 |  4 | 8μMAX  | 0110100 |
 * | MAX11613EUA+ | 3V3 |  4 | 8μMAX  | 0110100 |
 * | MAX11613EWC+ | 3V3 |  4 | 12WLP  | 0110100 |
 * | MAX11614EEE+ | 5V0 |  8 | 16QSOP | 0110011 |
 * | MAX11615EEE+ | 3V3 |  8 | 16QSOP | 0110011 |
 * | MAX11615EWE+ | 3V3 |  8 | 16WLP  | 0110011 |
 * | MAX11616EEE+ | 5V0 | 12 | 16QSOP | 0110101 |
 * | MAX11617EEE+ | 3V3 | 12 | 16QSOP | 0110101 |
 * | MAX11617EWE+ | 3V3 | 12 | 16WLP  | 0110101 |
 * +--------------+-----+----+--------+---------+
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
#include <nuttx/analog/max1161x.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_ADC_MAX1161X)

#if defined(CONFIG_MAX1161X_4CHAN)
#define MAX1161X_NUM_CHANNELS 4
#define MAX1161X_CHANNELSTROBED 0x000f
#define MAX1161X_I2C_ADDR 0x34u
#elif defined(CONFIG_MAX1161X_8CHAN)
#define MAX1161X_NUM_CHANNELS 8
#define MAX1161X_CHANNELSTROBED 0x00ff
#define MAX1161X_I2C_ADDR 0x33u
#elif defined(CONFIG_MAX1161X_12CHAN)
#define MAX1161X_NUM_CHANNELS 12
#define MAX1161X_CHANNELSTROBED 0x0fff
#define MAX1161X_I2C_ADDR 0x35u
#else
#error "MAX1161X Chip Type not defined"
#endif

#define MAX1161X_SETUP_MARKER           (1 << 7)
#define MAX1161X_SETUP_REF_SHIFT        4
#define MAX1161X_SETUP_REF_MASK         (7 << MAX1161X_SETUP_REF_SHIFT)
#define MAX1161X_SETUP_CLK              (1 << 3)
#define MAX1161X_SETUP_UNIBIP           (1 << 2)
#define MAX1161X_SETUP_RESET            (1 << 1)

#define MAX1161X_CMD_MARKER             (0 << 7)
#define MAX1161X_CMD_SCAN_SHIFT         5
#define MAX1161X_CMD_SCAN_MASK          (3 << MAX1161X_CMD_SCAN_SHIFT)
#define MAX1161X_CMD_CHANNEL_SHIFT      1
#define MAX1161X_CMD_CHANNEL_MASK       (15 << MAX1161X_CMD_CHANNEL_SHIFT)
#define MAX1161X_CMD_SNGDIF             (1 << 0)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct max1161x_dev_s
{
  FAR struct i2c_master_s           *i2c;
  FAR const struct adc_callback_s   *cb;
  uint8_t                           addr;

  /* List of channels to read on every convert trigger.
   * Bit position corresponds to channel. i.e. bit0 = 1 to read channel 0.
   */

  uint16_t                          chanstrobed;

  /* Current configuration of the ADC. There are two bytes holding
   * the complete configuration - the Setup Byte has Bit 7 always set,
   * while the command byte has bit 7 always reset
   */

  uint8_t                           setupbyte;
  uint8_t                           cmdbyte;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* max1161x helpers */

static int max1161x_readchannel(FAR struct max1161x_dev_s *priv,
                               FAR struct adc_msg_s *msg);

/* ADC methods */

static int  max1161x_bind(FAR struct adc_dev_s *dev,
                     FAR const struct adc_callback_s *callback);
static void max1161x_reset(FAR struct adc_dev_s *dev);
static int  max1161x_setup(FAR struct adc_dev_s *dev);
static void max1161x_shutdown(FAR struct adc_dev_s *dev);
static void max1161x_rxint(FAR struct adc_dev_s *dev, bool enable);
static int  max1161x_ioctl(FAR struct adc_dev_s *dev, int cmd,
                          unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct adc_ops_s g_adcops =
{
  max1161x_bind,      /* ao_bind */
  max1161x_reset,     /* ao_reset */
  max1161x_setup,     /* ao_setup */
  max1161x_shutdown,  /* ao_shutdown */
  max1161x_rxint,     /* ao_rxint */
  max1161x_ioctl      /* ao_read */
};

static struct max1161x_dev_s g_adcpriv;

static struct adc_dev_s g_adcdev =
{
  &g_adcops,    /* ad_ops */
  &g_adcpriv    /* ad_priv */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: max1161x_manage_strobe
 *
 * Description:
 *   Controls which channels are read on ANIOC_TRIGGER. By default all
 *   channels are read.
 *
 * Returned Value:
 *   0 on success. Negated errno on failure.
 *
 ****************************************************************************/

static int max1161x_manage_strobe(FAR struct max1161x_dev_s *priv,
                                 uint8_t channel, bool add_nremove)
{
  int ret = OK;

  if (priv == NULL || channel >= MAX1161X_NUM_CHANNELS)
    {
      ret = -EINVAL;
    }
  else
    {
      uint16_t flag = 1U << channel;

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
 * Name: max1161x_readchannel
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
 *         1                +CH1, -CH0
 *         2                +CH2, -CH3
 *         3                +CH3, -CH2
 *         4                +CH4, -CH5
 *         5                +CH5, -CH4
 *         6                +CH6, -CH7
 *         7                +CH7, -CH6
 *
 ****************************************************************************/

static int max1161x_readchannel(FAR struct max1161x_dev_s *priv,
                               FAR struct adc_msg_s *msg)
{
  int ret = OK;
  if (priv == NULL || msg == NULL)
    {
      ret = -EINVAL;
    }
  else
    {
      struct i2c_msg_s i2cmsg[3];
      uint8_t channel = msg->am_channel & ~(MAX1161X_CMD_CHANNEL_MASK);

      uint8_t setupbyte = priv->setupbyte;
      uint8_t cmdbyte = priv->cmdbyte & ~(MAX1161X_CMD_CHANNEL_MASK);
      cmdbyte |= channel << MAX1161X_CMD_CHANNEL_SHIFT;

      i2cmsg[0].frequency = CONFIG_MAX1161X_FREQUENCY;
      i2cmsg[0].addr = priv->addr;
      i2cmsg[0].flags = I2C_M_NOSTOP;
      i2cmsg[0].buffer = &setupbyte;
      i2cmsg[0].length = sizeof(setupbyte);

      i2cmsg[1].frequency = CONFIG_MAX1161X_FREQUENCY;
      i2cmsg[1].addr = priv->addr;
      i2cmsg[1].flags = I2C_M_NOSTOP;
      i2cmsg[1].buffer = &cmdbyte;
      i2cmsg[1].length = sizeof(cmdbyte);

      i2cmsg[2].frequency = CONFIG_MAX1161X_FREQUENCY;
      i2cmsg[2].addr = priv->addr;
      i2cmsg[2].flags = I2C_M_READ;

      uint16_t buf;
      i2cmsg[2].buffer = (uint8_t *)(&buf);
      i2cmsg[2].length = sizeof(buf);
      ret = I2C_TRANSFER(priv->i2c, i2cmsg, 3);
      if (ret < 0)
        {
            aerr("MAX1161X I2C transfer failed: %d", ret);
        }

      msg->am_data = be16toh(buf) & 0x0fffu;
    }

  return ret;
}

/****************************************************************************
 * Name: max1161x_bind
 *
 * Description:
 *   Bind the upper-half driver callbacks to the lower-half implementation.
 *   This must be called early in order to receive ADC event notifications.
 *
 ****************************************************************************/

static int max1161x_bind(FAR struct adc_dev_s *dev,
                        FAR const struct adc_callback_s *callback)
{
  FAR struct max1161x_dev_s *priv =
                        (FAR struct max1161x_dev_s *)dev->ad_priv;

  DEBUGASSERT(priv != NULL);
  priv->cb = callback;
  return OK;
}

/****************************************************************************
 * Name: max1161x_reset
 *
 * Description:
 *   Reset the ADC device.  Called early to initialize the hardware. This
 *   is called, before ao_setup() and on error conditions.
 *
 ****************************************************************************/

static void max1161x_reset(FAR struct adc_dev_s *dev)
{
  FAR struct max1161x_dev_s *priv =
                              (FAR struct max1161x_dev_s *)dev->ad_priv;

  priv->setupbyte   = MAX1161X_SETUP_MARKER;
  priv->cmdbyte     = MAX1161X_CMD_MARKER;
  priv->chanstrobed = MAX1161X_CHANNELSTROBED;

#ifndef MAX1161X_ENABLE_SCAN
  priv->cmdbyte &= ~(MAX1161X_CMD_SCAN_MASK);
  priv->cmdbyte |= (MAX1161X_SCAN_NONE << MAX1161X_CMD_SCAN_SHIFT);
#endif
}

/****************************************************************************
 * Name: max1161x_setup
 *
 * Description:
 *   Configure the ADC. This method is called the first time that the ADC
 *   device is opened.  The MAX1161X is quite simple and nothing special is
 *   needed to be done.
 *
 ****************************************************************************/

static int max1161x_setup(FAR struct adc_dev_s *dev)
{
  return OK;
}

/****************************************************************************
 * Name: max1161x_shutdown
 *
 * Description:
 *   Disable the ADC.  This method is called when the ADC device is closed.
 *   This method should reverse the operation of the setup method, but as
 *   the MAX1161X is quite simple does not need to do anything.
 *
 ****************************************************************************/

static void max1161x_shutdown(FAR struct adc_dev_s *dev)
{
}

/****************************************************************************
 * Name: max1161x_rxint
 *
 * Description:
 *   Needed for ADC upper-half compatibility but conversion interrupts
 *   are not supported by the MAX1161X.
 *
 ****************************************************************************/

static void max1161x_rxint(FAR struct adc_dev_s *dev, bool enable)
{
}

/****************************************************************************
 * Name: max1161x_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int max1161x_ioctl(FAR struct adc_dev_s *dev, int cmd,
                         unsigned long arg)
{
  FAR struct max1161x_dev_s *priv =
                              (FAR struct max1161x_dev_s *)dev->ad_priv;
  int ret = OK;

  switch (cmd)
    {
      case ANIOC_TRIGGER:
        {
          struct adc_msg_s msg;
          int i;

          for (i = 0; (i < MAX1161X_NUM_CHANNELS) && (ret == OK); i++)
            {
              if ((priv->chanstrobed >> i) & 1u)
                {
                  msg.am_channel = i;
                  ret = max1161x_readchannel(priv, &msg);
                  if (ret == OK)
                    {
                      priv->cb->au_receive(&g_adcdev, i, msg.am_data);
                    }
                }
            }
        }
        break;

      /* Add a channel to list of channels read on ANIOC_TRIGGER */

      case ANIOC_MAX1161X_ADD_CHAN:
        {
          ret = max1161x_manage_strobe(priv, (uint8_t)arg, true);
        }
        break;

      /* Remove a channel from list of channels read on ANIOC_TRIGGER */

      case ANIOC_MAX1161X_REMOVE_CHAN:
        {
          ret = max1161x_manage_strobe(priv, (uint8_t)arg, false);
        }
        break;

      /* Read a single channel from the ADC */

      case ANIOC_MAX1161X_READ_CHANNEL:
        {
          FAR struct adc_msg_s *msg = (FAR struct adc_msg_s *)arg;
          ret = max1161x_readchannel(priv, msg);
        }
        break;

      /* Set the ADC reference source and pin function */

      case ANIOC_MAX1161X_SET_REF:
        {
          switch (arg)
          {
            case MAX1161X_REF_VDD_AIN_NC_OFF:
            case MAX1161X_REF_EXT_RIN_IN_OFF:
            case MAX1161X_REF_INT_AIN_NC_OFF:
            case MAX1161X_REF_INT_AIN_NC_ON:
            case MAX1161X_REF_INT_ROUT_OUT_OFF:
            case MAX1161X_REF_INT_ROUT_OUT_ON:
              {
              priv->setupbyte &= ~(MAX1161X_SETUP_REF_MASK);
              priv->setupbyte |= (arg << MAX1161X_SETUP_REF_SHIFT);
              }
              break;

            default:
              {
                ret = -EINVAL;
              }
              break;
          }
        }
        break;

      /* Set clock source */

      case ANIOC_MAX1161X_SET_CLOCK:
        {
          if (arg == MAX1161X_CLOCK_EXT)
            {
              priv->setupbyte |= MAX1161X_SETUP_CLK;
            }
          else if (arg == MAX1161X_CLOCK_INT)
            {
              priv->setupbyte &= ~MAX1161X_SETUP_CLK;
            }
          else
            {
              ret = -EINVAL;
            }
        }
        break;

      /* Set the ADC Unipolar/Bipolar Mode */

      case ANIOC_MAX1161X_SET_UNIBIP:
        {
          if (arg == MAX1161X_BIPOLAR)
            {
              priv->setupbyte |= MAX1161X_SETUP_UNIBIP;
            }
          else if (arg == MAX1161X_UNIPOLAR)
            {
              priv->setupbyte &= ~MAX1161X_SETUP_UNIBIP;
            }
          else
            {
              ret = -EINVAL;
            }
        }
        break;

      /* Set the ADC Scan Mode */

      case ANIOC_MAX1161X_SET_SCAN:
        {
#ifdef MAX1161X_ENABLE_SCAN
          switch (arg)
          {
            case MAX1161X_SCAN_FROM_ZERO :
            case MAX1161X_SCAN_EIGHT_TIMES:
            case MAX1161X_SCAN_UPPER:
            case MAX1161X_SCAN_NONE:
              {
              priv->cmdbyte &= ~(MAX1161X_CMD_SCAN_MASK);
              priv->cmdbyte |= (arg << MAX1161X_CMD_SCAN_SHIFT);
              }
              break;

            default:
              {
                ret = -EINVAL;
              }
              break;
          }
#endif
        }
        break;

      /* Set the ADC Single Ended/Differential Mode */

      case ANIOC_MAX1161X_SET_SNGDIF:
        {
          if (arg == MAX1161X_SINGLE_ENDED)
            {
              priv->cmdbyte |= MAX1161X_CMD_SNGDIF;
            }
          else if (arg == MAX1161X_DIFFERENTIAL)
            {
              priv->cmdbyte &= ~MAX1161X_CMD_SNGDIF;
            }
          else
            {
              ret = -EINVAL;
            }
        }
        break;

      /* Command was not recognized */

      default:
        ret = -ENOTTY;
        aerr("MAX1161X ERROR: Unrecognized cmd: %d\n", cmd);
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: max1161x_initialize
 *
 * Description:
 *   Initialize the selected adc
 *
 * Input Parameters:
 *   i2c - Pointer to a I2C master struct for the bus the ADC resides on.
 *
 * Returned Value:
 *   Valid ADC device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

FAR struct adc_dev_s *max1161x_initialize(FAR struct i2c_master_s *i2c)
{
  DEBUGASSERT(i2c != NULL);

  /* Driver state data */

  FAR struct max1161x_dev_s *priv;
  priv = (FAR struct max1161x_dev_s *)g_adcdev.ad_priv;

  priv->cb   = NULL;
  priv->i2c  = i2c;
  priv->addr        = MAX1161X_I2C_ADDR;
  priv->setupbyte   = MAX1161X_SETUP_MARKER;
  priv->cmdbyte     = MAX1161X_CMD_MARKER;
  priv->chanstrobed = MAX1161X_CHANNELSTROBED;

#ifndef MAX1161X_ENABLE_SCAN
  priv->cmdbyte &= ~(MAX1161X_CMD_SCAN_MASK);
  priv->cmdbyte |= (MAX1161X_SCAN_NONE << MAX1161X_CMD_SCAN_SHIFT);
#endif

  return &g_adcdev;
}

#endif
