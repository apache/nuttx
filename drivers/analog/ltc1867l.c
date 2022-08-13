/****************************************************************************
 * drivers/analog/ltc1867l.c
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

#include <sys/types.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>
#include <nuttx/analog/ltc1867l.h>
#include <nuttx/semaphore.h>

#if defined(CONFIG_ADC_LTC1867L)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Device uses SPI Mode 1: CKPOL = 0, CKPHA = 0 */

#define LTC1867L_SPI_MODE (SPIDEV_MODE0)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ltc1867l_dev_s
{
  FAR const struct adc_callback_s *cb;
  FAR struct spi_dev_s *spi;
  unsigned int devno;
  FAR struct ltc1867l_channel_config_s *channel_config;
  int channel_config_count;
  sem_t sem;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void adc_lock(FAR struct spi_dev_s *spi);
static void adc_unlock(FAR struct spi_dev_s *spi);

/* ADC methods */

static int  adc_bind(FAR struct adc_dev_s *dev,
                     FAR const struct adc_callback_s *callback);
static void adc_reset(FAR struct adc_dev_s *dev);
static int  adc_setup(FAR struct adc_dev_s *dev);
static void adc_shutdown(FAR struct adc_dev_s *dev);
static void adc_rxint(FAR struct adc_dev_s *dev, bool enable);
static int  adc_ioctl(FAR struct adc_dev_s *dev, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct adc_ops_s g_adcops =
{
  adc_bind,      /* ao_bind */
  adc_reset,     /* ao_reset */
  adc_setup,     /* ao_setup */
  adc_shutdown,  /* ao_shutdown */
  adc_rxint,     /* ao_rxint */
  adc_ioctl      /* ao_read */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: adc_lock
 *
 * Description:
 *   Lock and configure the SPI bus.
 *
 ****************************************************************************/

static void adc_lock(FAR struct spi_dev_s *spi)
{
  SPI_LOCK(spi, true);
  SPI_SETMODE(spi, LTC1867L_SPI_MODE);
  SPI_SETBITS(spi, 16);
  SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, CONFIG_LTC1867L_FREQUENCY);
}

/****************************************************************************
 * Name: adc_unlock
 *
 * Description:
 *   Unlock the SPI bus.
 *
 ****************************************************************************/

static void adc_unlock(FAR struct spi_dev_s *spi)
{
  SPI_LOCK(spi, false);
}

/****************************************************************************
 * Name: adc_bind
 *
 * Description:
 *   Bind the upper-half driver callbacks to the lower-half implementation.
 *   This must be called early in order to receive ADC event notifications.
 *
 ****************************************************************************/

static int adc_bind(FAR struct adc_dev_s *dev,
                    FAR const struct adc_callback_s *callback)
{
  FAR struct ltc1867l_dev_s *priv =
    (FAR struct ltc1867l_dev_s *)dev->ad_priv;
  priv->cb = callback;
  return OK;
}

/****************************************************************************
 * Name: adc_reset
 *
 * Description:
 *   Reset the ADC device.  Called early to initialize the hardware. This
 *   is called, before ao_setup() and on error conditions.
 *
 ****************************************************************************/

static void adc_reset(FAR struct adc_dev_s *dev)
{
}

/****************************************************************************
 * Name: adc_setup
 *
 * Description:
 *   Configure the ADC. This method is called the first time that the ADC
 *   device is opened.  This will occur when the port is first opened.
 *   This setup includes configuring and attaching ADC interrupts. Interrupts
 *   are all disabled upon return.
 *
 ****************************************************************************/

static int adc_setup(FAR struct adc_dev_s *dev)
{
  return OK;
}

/****************************************************************************
 * Name: adc_shutdown
 *
 * Description:
 *   Disable the ADC.  This method is called when the ADC device is closed.
 *   This method reverses the operation the setup method.
 *
 ****************************************************************************/

static void adc_shutdown(FAR struct adc_dev_s *dev)
{
}

/****************************************************************************
 * Name: adc_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void adc_rxint(FAR struct adc_dev_s *dev, bool enable)
{
}

/****************************************************************************
 * Name: adc_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int adc_ioctl(FAR struct adc_dev_s *dev, int cmd, unsigned long arg)
{
  FAR struct ltc1867l_dev_s *priv =
    (FAR struct ltc1867l_dev_s *)dev->ad_priv;
  FAR struct spi_dev_s *spi = priv->spi;
  int i;
  uint16_t command;
  uint16_t data;
  int32_t postdata;
  int ret = OK;

  if (cmd == ANIOC_TRIGGER)
    {
      while (nxsem_wait(&priv->sem) < 0);

      adc_lock(spi);

      for (i = 0; i <= priv->channel_config_count; i++)
        {
          SPI_SELECT(spi, priv->devno, true);

          if (i < priv->channel_config_count)
            {
              command = priv->channel_config[i].analog_multiplexer_config |
                        priv->channel_config[i].analog_inputmode;
              command = command << 8;
            }
          else
            {
            command = 0;
            }

          data = SPI_SEND(spi, command);
          up_udelay(2);
          SPI_SELECT(spi, priv->devno, false);
          up_udelay(4);

          if (i > 0)
            {
              if (priv->channel_config[i - 1].analog_inputmode ==
                  LTC1867L_UNIPOLAR ||
                  (priv->channel_config[i - 1].analog_inputmode ==
                   LTC1867L_BIPOLAR &&
                   data >= 0 && data <= 0x7fff))
                {
                  postdata = data;
                }
              else
                {
                  postdata = -(0xffff - data) - 1;
                }

              priv->cb->au_receive(dev, priv->channel_config[i - 1].channel,
                                   postdata);
            }
        }

      adc_unlock(spi);
      nxsem_post(&priv->sem);
    }
  else
    {
      aerr("ERROR: Unrecognized cmd: %d\n", cmd);
      ret = -ENOTTY;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ltc1867l_register
 *
 * Description:
 *   Register the LTC1867L character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/adc0"
 *   spi - An instance of the SPI interface to use to communicate with
 *     LTC1867L
 *   devno - SPI device number
 *   channel_config - A pointer to an array which holds the configuration
 *     for each sampled channel.
 *   channel_config_count - Number of channels to sample
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ltc1867l_register(FAR const char *devpath, FAR struct spi_dev_s *spi,
                      unsigned int devno,
                      FAR struct ltc1867l_channel_config_s *channel_config,
                      int channel_config_count)
{
  FAR struct ltc1867l_dev_s *adcpriv;
  FAR struct adc_dev_s *adcdev;
  int ret;

  /* Sanity check */

  DEBUGASSERT(spi != NULL);
  DEBUGASSERT(channel_config != NULL);

  /* Initialize the LTC1867L device structure */

  adcpriv =
    (FAR struct ltc1867l_dev_s *)kmm_malloc(sizeof(struct ltc1867l_dev_s));
  if (adcpriv == NULL)
    {
      aerr("ERROR: Failed to allocate ltc1867l_dev_s instance\n");
      return -ENOMEM;
    }

  adcpriv->spi = spi;
  adcpriv->devno = devno;
  adcpriv->channel_config = channel_config;
  adcpriv->channel_config_count = channel_config_count;

  ret = nxsem_init(&adcpriv->sem, 1, 1);
  if (ret < 0)
    {
      kmm_free(adcpriv);
      return ret;
    }

  adcdev = (FAR struct adc_dev_s *)kmm_malloc(sizeof(struct adc_dev_s));
  if (adcdev == NULL)
    {
      aerr("ERROR: Failed to allocate adc_dev_s instance\n");
      nxsem_destroy(&adcpriv->sem);
      kmm_free(adcpriv);
      return -ENOMEM;
    }

  memset(adcdev, 0, sizeof(struct adc_dev_s));
  adcdev->ad_ops = &g_adcops;
  adcdev->ad_priv = adcpriv;

  /* Register the character driver */

  ret = adc_register(devpath, adcdev);
  if (ret < 0)
    {
      aerr("ERROR: Failed to register adc driver: %d\n", ret);
      kmm_free(adcdev);
      nxsem_destroy(&adcpriv->sem);
      kmm_free(adcpriv);
    }

  return ret;
}
#endif
