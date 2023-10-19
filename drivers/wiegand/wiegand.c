/****************************************************************************
 * drivers/wiegand/wiegand.c
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
#include <assert.h>
#include <debug.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/clock.h>
#include <nuttx/signal.h>

#include <nuttx/wiegand/wiegand.h>

#if defined(CONFIG_DRIVERS_WIEGAND)

#define WIEGAND_TIMEOUT_MS 50

#ifdef CONFIG_WIEGAND_FORMAT_34_BIT
#define WIEGAND_FORMAT_BIT 34
#define WIEGAND_FACILITY_CODE_BIT 16
#define WIEGAND_ID_CARD_BIT 16
#else
#define WIEGAND_FORMAT_BIT 26
#define WIEGAND_FACILITY_CODE_BIT 8
#define WIEGAND_ID_CARD_BIT 16
#endif

struct wiegand_dev_s
{
  FAR struct wiegand_config_s *config;
  uint8_t raw_data[WIEGAND_FORMAT_BIT];
  mutex_t devlock;
  int bit_index;
  bool wait_bit;
  int odd;
  int even;
};

static int wiegand_open(FAR struct file *filep);
static int wiegand_close(FAR struct file *filep);
static ssize_t wiegand_read(FAR struct file *filep,
                            FAR char *buffer, size_t buflen);
static const struct file_operations g_wiegandops =
{
  wiegand_open,  /* open */
  wiegand_close, /* close */
  wiegand_read,  /* read */
  NULL,          /* write */
};

static int wiegand_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct wiegand_dev_s *priv = inode->i_private;
  int ret;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  nxmutex_unlock(&priv->devlock);
  return OK;
}

static int wiegand_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct wiegand_dev_s *priv = inode->i_private;
  int ret;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  nxmutex_unlock(&priv->devlock);
  return OK;
}

static int wiegand_interrupt(int irq, FAR void *context, FAR void *arg)
{
    FAR struct wiegand_dev_s *priv =
        (FAR struct wiegand_dev_s *)arg;

  DEBUGASSERT(priv != NULL);

  bool data0 = priv->config->get_data(priv->config, 0);
  bool data1 = priv->config->get_data(priv->config, 1);

  if (data0 == data1)
    {
      return OK;
    }

    if (priv->bit_index >= WIEGAND_FORMAT_BIT)
    {
      return OK;
    }

  priv->raw_data[priv->bit_index] = (data0 ? 0x01 : 0x00);
  priv->bit_index++;
  priv->wait_bit = true;

  return OK;
}

static void wiegand_wait_frame(FAR struct wiegand_dev_s *priv)
{
  clock_t timeout;
  clock_t start;
  clock_t elapsed;

  timeout = MSEC2TICK(WIEGAND_TIMEOUT_MS);
  start = clock_systime_ticks();

  do
  {
    if (priv->wait_bit)
      {
        start = clock_systime_ticks();
        priv->wait_bit = false;
      }
    elapsed = clock_systime_ticks() - start;
  }
  while (elapsed < timeout);
}

static int wiegand_check_parity(FAR struct wiegand_dev_s *priv)
{
  int ret = OK;

  if ((priv->even % 2) && priv->raw_data[0] == 0)
    {
      return ERROR;
    }

  if ((!(priv->odd % 2) && priv->raw_data[WIEGAND_FORMAT_BIT - 1]) == 0)
    {
      return ERROR;
    }

  return ret;
}

static int wiegand_start_read_frame(FAR struct wiegand_dev_s *priv,
                                FAR struct wiegand_data_s *data)
{
  int i;
  int ret = OK;
  int facility_index = WIEGAND_FACILITY_CODE_BIT + 1;
  int id_index = WIEGAND_FACILITY_CODE_BIT + WIEGAND_FACILITY_CODE_BIT + 1;

  data->id = data->aba_code = data->facility_code = 0;

  if (priv->bit_index <= 0 || priv->bit_index < WIEGAND_FORMAT_BIT)
    {
      return ERROR;
    }

  for (i = 1; i < facility_index; i++)
    {
      data->facility_code <<= 1;
      data->facility_code |= priv->raw_data[i];

      if (priv->raw_data[i])
        {
          priv->even++;
        }
    }

  for (i = facility_index; i < id_index; i++)
    {
      data->id <<= 1;
      data->id |= priv->raw_data[i];

      if (priv->raw_data[i])
        {
          priv->odd++;
        }
    }

  data->aba_code = (data->facility_code << WIEGAND_FACILITY_CODE_BIT);
  data->aba_code |= data->id;

  return ret;
}

static ssize_t wiegand_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen)
{
  int ret = OK;
  FAR struct inode *inode = filep->f_inode;
  FAR struct wiegand_dev_s *priv = inode->i_private;
  FAR struct wiegand_data_s *data = (FAR struct wiegand_data_s *)buffer;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return (ssize_t)ret;
    }

  if (buflen < sizeof(FAR struct wiegand_data_s))
    {
      snerr("ERROR: Not enough memory to read data sample.\n");
      return -ENOSYS;
    }

  wiegand_wait_frame(priv);

  ret = wiegand_start_read_frame(priv, data);
  if (ret < 0)
    {
      data->status = WIEGAND_READ_UNCOMPLETED;
    }

  else
    {
      data->status = WIEGAND_SUCCESS;

      ret = (wiegand_check_parity(priv) < 0);
      if (ret < 0)
        {
          data->status = WIEGAND_PARITY_ERROR;
        }
    }

  priv->bit_index = priv->even = priv->odd = 0;
  nxmutex_unlock(&priv->devlock);

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int wiegand_register(FAR const char *devpath,
                     FAR struct wiegand_config_s *config)
{
  int ret = 0;
  FAR struct wiegand_dev_s *priv;

  priv = (FAR struct wiegand_dev_s *)
        kmm_malloc(sizeof(struct wiegand_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->config = config;
  memset(&priv->raw_data, 0x00, sizeof(priv->raw_data));
  priv->bit_index = 0;

  nxmutex_init(&priv->devlock);

  ret = register_driver(devpath, &g_wiegandops, 0666, priv);
  if (ret < 0)
    {
      nxmutex_destroy(&priv->devlock);
      kmm_free(priv);
      snerr("ERROR: Failed to register driver: %d\n", ret);
      return ret;
    }

  priv->config->irq_attach(priv->config, wiegand_interrupt, priv);
  priv->config->irq_enable(priv->config, true);

  return ret;
}
#endif /* CONFIG_DRIVERS_WIEGAND */