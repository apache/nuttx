/****************************************************************************
 * drivers/analog/mcp3008.c
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
#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/types.h>

#include <nuttx/analog/adc.h>
#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/spi/spi.h>

#include <nuttx/analog/mcp3008.h>

/****************************************************************************
 * Preprocessor definitions
 ****************************************************************************/

#if !defined(CONFIG_SPI)
#error "SPI Support Required."
#endif

#define MCP3008_NUM_CHANNELS 8

/* 3.6MHz is recommended for VDD >= 4V
 * 2.34MHz is recommended for VDD >= 3.3V
 * 1.35MHz is recommended for VDD = 2.7V
 */

#ifndef CONFIG_ADC_MCP3008_SPI_FREQUENCY
#define CONFIG_ADC_MCP3008_SPI_FREQUENCY 2340000
#endif /* CONFIG_ADC_MCP3008_SPI_FREQUENCY */

/* Single-ended or differential modes */

#ifndef CONFIG_ADC_MCP3008_DIFFERENTIAL
#define CONFIG_ADC_MCP3008_DIFFERENTIAL 0
#endif /* CONFIG_ADC_MCP3008_DIFFERENTIAL */

#define MCP3008_SPI_MODE (SPIDEV_MODE0)

#if defined(CONFIG_ADC_MCP3008)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mcp3008_dev_s
{
  FAR struct spi_dev_s *spi; /* SPI interface */
  FAR const struct adc_callback_s *cb;
  bool diff; /* True if the ADC is in differential mode, false otherwise. */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* ADC methods */

static int mcp3008_bind(FAR struct adc_dev_s *dev,
                        FAR const struct adc_callback_s *callback);
static void mcp3008_reset(FAR struct adc_dev_s *dev);
static int mcp3008_setup(FAR struct adc_dev_s *dev);
static void mcp3008_shutdown(FAR struct adc_dev_s *dev);
static void mcp3008_rxint(FAR struct adc_dev_s *dev, bool enable);
static int mcp3008_ioctl(FAR struct adc_dev_s *dev, int cmd,
                         unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct adc_ops_s g_mcp3008ops =
{
  .ao_bind = mcp3008_bind,
  .ao_reset = mcp3008_reset,
  .ao_setup = mcp3008_setup,
  .ao_shutdown = mcp3008_shutdown,
  .ao_rxint = mcp3008_rxint,
  .ao_ioctl = mcp3008_ioctl,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mcp3008_configspi
 *
 * Description:
 *   Configure the SPI interface for the MCP3008.
 *
 ****************************************************************************/

static inline void mcp3008_configspi(FAR struct spi_dev_s *spi)
{
  SPI_SETMODE(spi, MCP3008_SPI_MODE);
  SPI_SETBITS(spi, 8);
  SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, CONFIG_ADC_MCP3008_SPI_FREQUENCY);
}

/****************************************************************************
 * Name: mcp3008_readchannel
 *
 * Description:
 *   Read the corresponding channel of the MCP3008 ADC.
 *
 * Input Parameters:
 *   priv - An MCP3008 device structure.
 *   msg - An ADC message struct where the am_channel member contains the
 *   channel number to be read, and where the am_data member is where the
 *   reading is stored.
 *
 * NOTE:
 *  When single-ended mode is enabled, the channel number will correspond
 *  directly to the channel number (0-7) on the ADC.
 *  When differential mode is enabled, the channel numbers will correspond
 *  to the following differential pairs:
 *
 *  msg->am_channel  Source
 *  0                CH0+, CH1-
 *  1                CH0-, CH1+
 *  2                CH2+, CH3-
 *  3                CH2-, CH3+
 *  4                CH4+, CH5-
 *  5                CH4-, CH5+
 *  6                CH6+, CH7-
 *  7                CH6-, CH7+
 *
 ****************************************************************************/

static int mcp3008_readchannel(FAR struct mcp3008_dev_s *priv,
                               FAR struct adc_msg_s *msg)
{
  /* First byte is 0s followed by start byte.
   * Second byte is control bits, set later.
   * Third byte contents do not matter, but are 0 here.
   *
   * When data is received into this buffer:
   * First byte is garbage.
   * Second and third byte contain b9-b0.
   */

  uint8_t data[3] =
    {
      1, 0, 0
    };

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(msg != NULL);

  if (priv->diff)
    {
      data[1] = 0x0; /* Differential control bits start with 0 in MSB. */
    }
  else
    {
      data[1] = 0x8; /* Single-ended control bits start with 1 in MSB. */
    }

  /* Only get last three bits of the channel since there are only 8 channels.
   */

  data[1] |= (msg->am_channel & 0x7);
  data[1] <<= 4;

  /* Send control message */

  SPI_LOCK(priv->spi, true);

  mcp3008_configspi(priv->spi);
  SPI_SELECT(priv->spi, SPIDEV_ADC(0), true);

  SPI_EXCHANGE(priv->spi, data, data, sizeof(data));

  SPI_SELECT(priv->spi, SPIDEV_ADC(0), false);
  SPI_LOCK(priv->spi, false);

  /* Mask out anything not part of the 10 measurement bits */

  msg->am_data = ((data[1] & 3) << 8) + (data[2]);
  return 0;
}

/****************************************************************************
 * Name: mcp3008_bind
 *
 * Description:
 *   Bind the upper-half driver callbacks to the lower-half implementation.
 *   This must be called early in order to receive ADC event notifications.
 *
 ****************************************************************************/

static int mcp3008_bind(FAR struct adc_dev_s *dev,
                        FAR const struct adc_callback_s *callback)
{
  FAR struct mcp3008_dev_s *priv = (FAR struct mcp3008_dev_s *)dev->ad_priv;

  DEBUGASSERT(priv != NULL);
  priv->cb = callback;
  return 0;
}

/****************************************************************************
 * Name: mcp3008_reset
 *
 * Description:
 *   Reset the ADC device.  Called early to initialize the hardware. This
 *   is called, before ao_setup() and on error conditions.
 *   The MCP3008 can't be reset, nothing needs to be done here.
 *
 ****************************************************************************/

static void mcp3008_reset(FAR struct adc_dev_s *dev)
{
}

/****************************************************************************
 * Name: mcp3008_setup
 *
 * Description:
 *   Configure the ADC. This method is called the first time that the ADC
 *   device is opened.
 *   MCP3008 runs as soon as it is powered. There is no setup required.
 *
 ****************************************************************************/

static int mcp3008_setup(FAR struct adc_dev_s *dev)
{
  return OK;
}

/****************************************************************************
 * Name: mcp3008_shutdown
 *
 * Description:
 *   Disable the ADC. This method is called when the ADC device is closed.
 *   This method should reverse the operation of the setup method.
 *   The MCP3008 cannot be shutdown unless powered off, so nothing is
 *   required.
 *
 ****************************************************************************/

static void mcp3008_shutdown(FAR struct adc_dev_s *dev)
{
}

/****************************************************************************
 * Name: mcp3008_rxint
 *
 * Description:
 *   Needed for ADC upper-half compatibility but conversion interrupts
 *   are not supported by the MCP3008.
 *
 ****************************************************************************/

static void mcp3008_rxint(FAR struct adc_dev_s *dev, bool enable)
{
}

/****************************************************************************
 * Name: mcp3008_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method.
 *
 ****************************************************************************/

static int mcp3008_ioctl(FAR struct adc_dev_s *dev, int cmd,
                         unsigned long arg)
{
  FAR struct mcp3008_dev_s *priv = (FAR struct mcp3008_dev_s *)dev->ad_priv;
  int ret = 0;

  switch (cmd)
  {
    /* Trigger a measurement */

    case ANIOC_TRIGGER:
      {
        struct adc_msg_s msg;

        for (uint8_t i = 0; i < MCP3008_NUM_CHANNELS && (ret == 0); i++)
        {
          msg.am_channel = i;
          ret = mcp3008_readchannel(priv, &msg);
          if (ret == 0)
            {
              priv->cb->au_receive(dev, i, msg.am_data);
            }
        }
      } break;

    /* Change differential mode */

    case ANIOC_MCP3008_DIFF:
      DEBUGASSERT(arg == 0 || arg == 1);
      priv->diff = arg;
      break;

    /* Get the number of channels */

    case ANIOC_GET_NCHANNELS:
      ret = 8;
      break;

    /* Command was not recognized */

    default:
      ret = -EINVAL;
      aerr("MCP3008 ioctl: Unrecognized cmd: %d\n", cmd);
      break;
  }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mcp3008_initialize
 *
 * Description:
 *   Initialize ADC
 *
 * Input Parameters:
 *    spi - SPI driver instance
 *    spidev - SPI chip select number
 *
 * Returned Value:
 *   Valid MCP3008 ADC device structure reference on success; a NULL on
 *   failure
 *
 ****************************************************************************/

FAR struct adc_dev_s *mcp3008_initialize(FAR struct spi_dev_s *spi)
{
  FAR struct mcp3008_dev_s *priv;
  FAR struct adc_dev_s *adcdev;

  DEBUGASSERT(spi != NULL);

  /* Initialize the ADC device structure */

  priv = kmm_malloc(sizeof(struct mcp3008_dev_s));
  if (priv == NULL)
    {
      aerr("ERROR: Failed to allocate mcp3008_dev_s instance\n");
      free(priv);
      return NULL;
    }

  /* Initialize the MCP3008 device structure */

  priv->cb = NULL;
  priv->spi = spi;
  priv->diff = CONFIG_ADC_MCP3008_DIFFERENTIAL;

  adcdev = kmm_malloc(sizeof(struct adc_dev_s));
  if (adcdev == NULL)
    {
      aerr("ERROR: Failed to allocate adc_dev_s instance\n");
      return NULL;
    }

  memset(adcdev, 0, sizeof(struct adc_dev_s));
  adcdev->ad_ops = &g_mcp3008ops;
  adcdev->ad_priv = priv;

  return adcdev;
}

#endif /* CONFIG_ADC_MCP3008 */
