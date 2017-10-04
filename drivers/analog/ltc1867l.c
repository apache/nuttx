/****************************************************************************
 * arch/drivers/analog/ltc1867l.c
 *
 *   Copyright (C) 2017 DS-Automotion GmbH. All rights reserved.
 *   Author: Martin Lederhilger <m.lederhilger@ds-automotion.com>
 *
 * This file is a part of NuttX:
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
#include <semaphore.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>
#include <nuttx/analog/ltc1867l.h>

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
  .ao_bind     = adc_bind,      /* ao_bind */
  .ao_reset    = adc_reset,     /* ao_reset */
  .ao_setup    = adc_setup,     /* ao_setup */
  .ao_shutdown = adc_shutdown,  /* ao_shutdown */
  .ao_rxint    = adc_rxint,     /* ao_rxint */
  .ao_ioctl    = adc_ioctl      /* ao_read */
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
  (void)SPI_LOCK(spi, true);
  SPI_SETMODE(spi, LTC1867L_SPI_MODE);
  SPI_SETBITS(spi, 16);
  (void)SPI_HWFEATURES(spi, 0);
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
  (void)SPI_LOCK(spi, false);
}

/****************************************************************************
 * Name: adc_bind
 *
 * Description:
 *   Bind the upper-half driver callbacks to the lower-half implementation.  This
 *   must be called early in order to receive ADC event notifications.
 *
 ****************************************************************************/

static int adc_bind(FAR struct adc_dev_s *dev,
                    FAR const struct adc_callback_s *callback)
{
  FAR struct ltc1867l_dev_s *priv = (FAR struct ltc1867l_dev_s *)dev->ad_priv;
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
 *   This setup includes configuring and attaching ADC interrupts.  Interrupts
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
  FAR struct ltc1867l_dev_s *priv = (FAR struct ltc1867l_dev_s *)dev->ad_priv;
  FAR struct spi_dev_s *spi = priv->spi;
  int i;
  uint16_t command;
  uint16_t data;
  int32_t dataToPost;
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
                        priv->channel_config[i].analog_inputMode;
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
              if (priv->channel_config[i-1].analog_inputMode == LTC1867L_UNIPOLAR ||
                  (priv->channel_config[i-1].analog_inputMode == LTC1867L_BIPOLAR &&
                   data >= 0 && data <= 0x7fff))
                {
                  dataToPost = data;
                }
              else
                {
                  dataToPost = -(0xffff - data) - 1;
                }

              priv->cb->au_receive(dev, priv->channel_config[i-1].channel, dataToPost);
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
                      FAR struct ltc1867l_channel_config_s* channel_config,
                      int channel_config_count)
{
  FAR struct ltc1867l_dev_s *adcpriv;
  FAR struct adc_dev_s *adcdev;
  int ret;

  /* Sanity check */

  DEBUGASSERT(spi != NULL);
  DEBUGASSERT(channel_config != NULL);

  /* Initialize the LTC1867L device structure */

  adcpriv = (FAR struct ltc1867l_dev_s *)kmm_malloc(sizeof(struct ltc1867l_dev_s));
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
