/****************************************************************************
 * drivers/spi/spi_dummy.c
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
#include <stdbool.h>
#include <stdint.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/spi/spi_dummy.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct spi_dummy_dev_s
{
  struct spi_dev_s spi;
  mutex_t lock;
  FAR struct spi_dummy_device_s * const *devices;
  int ndevices;
  FAR struct spi_dummy_device_s *selected_dev;
  uint32_t frequency;
  enum spi_mode_e mode;
  int nbits;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int spi_dummy_lock(FAR struct spi_dev_s *dev, bool lock);
static void spi_dummy_select(FAR struct spi_dev_s *dev, uint32_t devid,
                             bool selected);
static uint32_t spi_dummy_setfrequency(FAR struct spi_dev_s *dev,
                                       uint32_t frequency);
static void spi_dummy_setmode(FAR struct spi_dev_s *dev,
                              enum spi_mode_e mode);
static void spi_dummy_setbits(FAR struct spi_dev_s *dev, int nbits);
#ifdef CONFIG_SPI_HWFEATURES
static int spi_dummy_hwfeatures(FAR struct spi_dev_s *dev,
                                spi_hwfeatures_t features);
#endif
static uint8_t spi_dummy_status(FAR struct spi_dev_s *dev, uint32_t devid);
static FAR struct spi_dummy_device_s *
  spi_dummy_find(FAR struct spi_dummy_dev_s *priv, uint32_t devid);
static int spi_dummy_validate(FAR struct spi_dummy_dev_s *priv);
static uint32_t spi_dummy_send(FAR struct spi_dev_s *dev, uint32_t wd);
#ifdef CONFIG_SPI_EXCHANGE
static int spi_dummy_exchange(FAR struct spi_dev_s *dev,
                              FAR const void *txbuffer, FAR void *rxbuffer,
                              size_t nwords);
#endif
static int spi_dummy_registercallback(FAR struct spi_dev_s *dev,
                                      spi_mediachange_t callback,
                                      FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct spi_ops_s g_spi_dummy_ops =
{
  spi_dummy_lock,
  spi_dummy_select,
  spi_dummy_setfrequency,
#ifdef CONFIG_SPI_DELAY_CONTROL
  NULL,
#endif
  spi_dummy_setmode,
  spi_dummy_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  spi_dummy_hwfeatures,
#endif
  spi_dummy_status,
#ifdef CONFIG_SPI_CMDDATA
  NULL,
#endif
  spi_dummy_send,
#ifdef CONFIG_SPI_EXCHANGE
  spi_dummy_exchange,
  NULL,
  NULL,
#else
  NULL,
  NULL,
#endif
#ifdef CONFIG_SPI_TRIGGER
  NULL,
#endif
  spi_dummy_registercallback
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int spi_dummy_lock(FAR struct spi_dev_s *dev, bool lock)
{
  FAR struct spi_dummy_dev_s *priv = (FAR struct spi_dummy_dev_s *)dev;

  if (lock)
    {
      return nxmutex_lock(&priv->lock);
    }

  return nxmutex_unlock(&priv->lock);
}

static void spi_dummy_select(FAR struct spi_dev_s *dev, uint32_t devid,
                            bool selected)
{
  FAR struct spi_dummy_dev_s *priv = (FAR struct spi_dummy_dev_s *)dev;
  FAR struct spi_dummy_device_s *dummy;

  if (selected)
    {
      dummy = spi_dummy_find(priv, devid);
      if (dummy != NULL)
        {
          priv->selected_dev = dummy;
          if (dummy->select != NULL)
            {
              dummy->select(dummy, true);
            }
        }
    }
  else if (priv->selected_dev != NULL && priv->selected_dev->devid == devid)
    {
      if (priv->selected_dev->select != NULL)
        {
          priv->selected_dev->select(priv->selected_dev, false);
        }

      priv->selected_dev = NULL;
    }
}

static uint32_t spi_dummy_setfrequency(FAR struct spi_dev_s *dev,
                                      uint32_t frequency)
{
  FAR struct spi_dummy_dev_s *priv = (FAR struct spi_dummy_dev_s *)dev;

  priv->frequency = frequency;
  return frequency;
}

static void spi_dummy_setmode(FAR struct spi_dev_s *dev,
                             enum spi_mode_e mode)
{
  FAR struct spi_dummy_dev_s *priv = (FAR struct spi_dummy_dev_s *)dev;

  priv->mode = mode;
}

static void spi_dummy_setbits(FAR struct spi_dev_s *dev, int nbits)
{
  FAR struct spi_dummy_dev_s *priv = (FAR struct spi_dummy_dev_s *)dev;

  priv->nbits = nbits;
}

#ifdef CONFIG_SPI_HWFEATURES
static int spi_dummy_hwfeatures(FAR struct spi_dev_s *dev,
                                spi_hwfeatures_t features)
{
  spi_hwfeatures_t supported = HWFEAT_FORCE_CS_INACTIVE_AFTER_TRANSFER |
                               HWFEAT_FORCE_CS_ACTIVE_AFTER_TRANSFER;

  UNUSED(dev);

  if (features & ~supported)
    {
      return -ENOSYS;
    }

  return OK;
}
#endif

static uint8_t spi_dummy_status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  FAR struct spi_dummy_dev_s *priv = (FAR struct spi_dummy_dev_s *)dev;
  FAR struct spi_dummy_device_s *dummy;

  dummy = spi_dummy_find(priv, devid);
  if (dummy == NULL)
    {
      return 0;
    }

  return dummy->status != NULL ? dummy->status(dummy) : SPI_STATUS_PRESENT;
}

static FAR struct spi_dummy_device_s *
spi_dummy_find(FAR struct spi_dummy_dev_s *priv, uint32_t devid)
{
  FAR struct spi_dummy_device_s *dummy;
  int i;

  for (i = 0; i < priv->ndevices; i++)
    {
      dummy = priv->devices[i];
      if (dummy->devid == devid)
        {
          return dummy;
        }
    }

  return NULL;
}

static int spi_dummy_validate(FAR struct spi_dummy_dev_s *priv)
{
  if (priv->selected_dev == NULL)
    {
      return -ENODEV;
    }

  if (priv->frequency == 0 || priv->mode > SPIDEV_MODE3 ||
      priv->nbits <= 0 || priv->nbits > 32)
    {
      return -EINVAL;
    }

  return OK;
}

static uint32_t spi_dummy_send(FAR struct spi_dev_s *dev, uint32_t wd)
{
  FAR struct spi_dummy_dev_s *priv = (FAR struct spi_dummy_dev_s *)dev;

  if (spi_dummy_validate(priv) < 0)
    {
      return 0;
    }

  if (priv->selected_dev->send == NULL)
    {
      return 0;
    }

  return priv->selected_dev->send(priv->selected_dev, wd, priv->nbits);
}

#ifdef CONFIG_SPI_EXCHANGE
static int spi_dummy_exchange(FAR struct spi_dev_s *dev,
                             FAR const void *txbuffer, FAR void *rxbuffer,
                             size_t nwords)
{
  FAR struct spi_dummy_dev_s *priv = (FAR struct spi_dummy_dev_s *)dev;
  int ret;

  ret = spi_dummy_validate(priv);
  if (ret < 0)
    {
      return ret;
    }

  if (priv->selected_dev->exchange == NULL)
    {
      return -ENOSYS;
    }

  return priv->selected_dev->exchange(priv->selected_dev, txbuffer,
                                      rxbuffer, nwords, priv->nbits);
}
#endif

static int spi_dummy_registercallback(FAR struct spi_dev_s *dev,
                                     spi_mediachange_t callback,
                                     FAR void *arg)
{
  return -ENOSYS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct spi_dev_s *
spi_dummy_initialize(FAR struct spi_dummy_device_s * const *devices,
                     int ndevices)
{
  FAR struct spi_dummy_dev_s *priv;

  if (ndevices < 0 || (devices == NULL && ndevices > 0))
    {
      return NULL;
    }

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      return NULL;
    }

  priv->spi.ops = &g_spi_dummy_ops;
  priv->devices = devices;
  priv->ndevices = ndevices;
  priv->frequency = 400000;
  priv->mode = SPIDEV_MODE0;
  priv->nbits = 8;
  nxmutex_init(&priv->lock);

  return &priv->spi;
}
