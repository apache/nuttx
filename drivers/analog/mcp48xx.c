/****************************************************************************
 * drivers/analog/mcp48xx.c
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
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/arch.h>
#include <nuttx/analog/dac.h>
#include <nuttx/spi/spi.h>

#include <nuttx/analog/mcp48xx.h>

/****************************************************************************
 * Preprocessor definitions
 ****************************************************************************/

#if !defined(CONFIG_SPI)
#  error SPI Support Required.
#endif

#if defined(CONFIG_MCP48XX)

#if defined(CONFIG_MCP4802)
#  define MCP48XX_DATA_BITS  8u
#elif defined(CONFIG_MCP4812)
#  define MCP48XX_DATA_BITS  10u
#elif defined(CONFIG_MCP4822)
#  define MCP48XX_DATA_BITS  12u
#else
#  error MCP48XX variant selection required
#endif

#ifndef CONFIG_MCP48XX_SPI_FREQUENCY
#  define CONFIG_MCP48XX_SPI_FREQUENCY 4000000
#endif

#define MCP48XX_SPI_MODE     (SPIDEV_MODE0)  /* SPI Mode 0: CPOL=0 CPHA=0 */

#define MCP48XX_MAX_CHANNELS 2u

#define MCP48XX_SHDN         (1u << 12)  /* Output Shutdown Control bit */
#define MCP48XX_GA           (1u << 13)  /* Output Gain Selection bit */

#define MCP48XX_MAX_BITS     12u

#define MCP48XX_DATA_MASK    ((1u << MCP48XX_DATA_BITS) - 1u)
#define MCP48XX_DATA_SHIFT   (MCP48XX_MAX_BITS - MCP48XX_DATA_BITS)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mcp48xx_dev_s
{
  FAR struct spi_dev_s *spi;           /* SPI interface */
  uint32_t spidev;                     /* SPI Chip Select number */
  uint16_t cmd[MCP48XX_MAX_CHANNELS];  /* Output Gain Selection and Output
                                        * Shutdown Control bits */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* DAC methods */

static void mcp48xx_reset(FAR struct dac_dev_s *dev);
static int  mcp48xx_setup(FAR struct dac_dev_s *dev);
static void mcp48xx_shutdown(FAR struct dac_dev_s *dev);
static void mcp48xx_txint(FAR struct dac_dev_s *dev, bool enable);
static int  mcp48xx_send(FAR struct dac_dev_s *dev,
                         FAR struct dac_msg_s *msg);
static int  mcp48xx_ioctl(FAR struct dac_dev_s *dev, int cmd,
                          unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct mcp48xx_dev_s g_devpriv;

static const struct dac_ops_s g_dacops =
{
  mcp48xx_reset,        /* ao_reset */
  mcp48xx_setup,        /* ao_setup */
  mcp48xx_shutdown,     /* ao_shutdown */
  mcp48xx_txint,        /* ao_txint */
  mcp48xx_send,         /* ao_send */
  mcp48xx_ioctl         /* ao_ioctl */
};

static struct dac_dev_s g_dacdev =
{
  &g_dacops,    /* ad_ops */
  &g_devpriv    /* ad_priv */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mcp48xx_configspi
 *
 * Description:
 *   Configure the SPI interface
 *
 ****************************************************************************/

static inline void mcp48xx_configspi(FAR struct spi_dev_s *spi)
{
  SPI_SETMODE(spi, MCP48XX_SPI_MODE);
  SPI_SETBITS(spi, 8);
  SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, CONFIG_MCP48XX_SPI_FREQUENCY);
}

/****************************************************************************
 * Name: mcp48xx_reset
 *
 * Description:
 *   Reset the DAC device.  Called early to initialize the hardware.  This
 *   is called, before ao_setup() and on error conditions.
 *
 ****************************************************************************/

static void mcp48xx_reset(FAR struct dac_dev_s *dev)
{
}

/****************************************************************************
 * Name: mcp48xx_setup
 *
 * Description:
 *   Configure the DAC.  This method is called the first time that the DAC
 *   device is opened.  This will occur when the port is first opened.  This
 *   setup includes configuring and attaching DAC interrupts.  Interrupts are
 *   all disabled upon return.
 *
 ****************************************************************************/

static int mcp48xx_setup(FAR struct dac_dev_s *dev)
{
  return OK;
}

/****************************************************************************
 * Name: mcp48xx_shutdown
 *
 * Description:
 *   Disable the DAC. This method is called when the DAC device is closed.
 *   This method reverses the operation the setup method.
 *
 ****************************************************************************/

static void mcp48xx_shutdown(FAR struct dac_dev_s *dev)
{
}

/****************************************************************************
 * Name: mcp48xx_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void mcp48xx_txint(FAR struct dac_dev_s *dev, bool enable)
{
}

/****************************************************************************
 * Name: mcp48xx_send
 ****************************************************************************/

static int mcp48xx_send(FAR struct dac_dev_s *dev, FAR struct dac_msg_s *msg)
{
  FAR struct mcp48xx_dev_s *priv = (FAR struct mcp48xx_dev_s *)dev->ad_priv;
  uint16_t data;

  /* Sanity check */

  DEBUGASSERT(priv->spi != NULL);

  /* Set up message to send */

  ainfo("value: %08"PRIx32"\n", msg->am_data);

  SPI_LOCK(priv->spi, true);

  mcp48xx_configspi(priv->spi);

  data = ((msg->am_data & MCP48XX_DATA_MASK) << MCP48XX_DATA_SHIFT)
       | ((msg->am_channel & 0x01) << 15)
       | priv->cmd[msg->am_channel & 0x01];

  SPI_SELECT(priv->spi, priv->spidev, true);

  SPI_SEND(priv->spi, (data >> 8));
  SPI_SEND(priv->spi, (data & 0xff));

  SPI_SELECT(priv->spi, priv->spidev, false);

  SPI_LOCK(priv->spi, false);

  dac_txdone(dev);
  return 0;
}

/****************************************************************************
 * Name: mcp48xx_ioctl
 ****************************************************************************/

static int mcp48xx_ioctl(FAR struct dac_dev_s *dev, int cmd,
                         unsigned long arg)
{
  FAR struct mcp48xx_dev_s *priv = (FAR struct mcp48xx_dev_s *)dev->ad_priv;
  int ret = OK;

  switch (cmd)
    {
      case ANIOC_MCP48XX_DAC_SET_GAIN:
        {
          int i;

          if ((arg & MCP48XX_GAIN_1X) != 0)
            {
              for (i = 0; i < MCP48XX_MAX_CHANNELS; i++)
                {
                  if ((arg & (1u << i)) != 0)
                    {
                      priv->cmd[i] |= MCP48XX_GA;
                    }
                }
            }
          else
            {
              for (i = 0; i < MCP48XX_MAX_CHANNELS; i++)
                {
                  if ((arg & (1u << i)) != 0)
                    {
                      priv->cmd[i] &= ~MCP48XX_GA;
                    }
                }
            }
        }
        break;

      case ANIOC_MCP48XX_DAC_ENABLE:
        {
          int i;

          for (i = 0; i < MCP48XX_MAX_CHANNELS; i++)
            {
              if ((arg & (1u << i)) != 0)
                {
                  priv->cmd[i] |= MCP48XX_SHDN;
                }
            }
        }
        break;

      case ANIOC_MCP48XX_DAC_DISABLE:
        {
          int i;

          for (i = 0; i < MCP48XX_MAX_CHANNELS; i++)
            {
              if ((arg & (1u << i)) != 0)
                {
                  priv->cmd[i] &= ~MCP48XX_SHDN;
                }
            }
        }
        break;

    /* Command was not recognized */

      default:
        aerr("MCP48XX ERROR: Unrecognized cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mcp48xx_initialize
 *
 * Description:
 *   Initialize DAC
 *
 * Input Parameters:
 *    spi - SPI driver instance
 *    spidev - SPI Chip Select number
 *
 * Returned Value:
 *   Valid MCP48XX device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

FAR struct dac_dev_s *mcp48xx_initialize(FAR struct spi_dev_s *spi,
                                         uint32_t spidev)
{
  FAR struct mcp48xx_dev_s *priv;
  int i;

  /* Sanity check */

  DEBUGASSERT(spi != NULL);

  /* Initialize the MCP48XX device structure */

  priv = (FAR struct mcp48xx_dev_s *)g_dacdev.ad_priv;
  priv->spi = spi;
  priv->spidev = spidev;

  /* Enable both channels with 1x gain by default */

  for (i = 0; i < MCP48XX_MAX_CHANNELS; i++)
    {
      priv->cmd[i] = MCP48XX_SHDN | MCP48XX_GA;
    }

  return &g_dacdev;
}

#endif /* CONFIG_MCP48XX */
