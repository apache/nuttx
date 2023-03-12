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

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

struct wiegand_upperhalf_s
{
  uint8_t index; /**/
  bool wait;     /* */
  uint8_t buffer[WIEGAND_FORMAT_BIT];
  mutex_t lock;                          /**/
  FAR struct wiegand_lowerhalf_s *lower; /**/
};

static int wiegand_open(FAR struct file *filep);
static int wiegand_close(FAR struct file *filep);
static ssize_t wiegand_read(FAR struct file *filep,
                            FAR char *buffer, size_t buflen);
static int wiegand_ioctl(FAR struct file *filep, int cmd,
                         unsigned long arg);

static const struct file_operations g_wiegandops =
    {
        wiegand_open,  /* open */
        wiegand_close, /* close */
        wiegand_read,  /* read */
        NULL,          /* write */
        NULL,          /* seek */
        wiegand_ioctl, /* ioctl */
};

static int wiegand_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct wiegand_upperhalf_s *priv = (struct wiegand_upperhalf_s *)arg;

  DEBUGASSERT(priv != NULL);

  bool data0 = priv->lower->ops->getdata(priv->lower, 0);
  bool data1 = priv->lower->ops->getdata(priv->lower, 1);

  if (data0 == data1)
  {
    return OK;
  }

  if (priv->index >= WIEGAND_FORMAT_BIT)
  {
    return OK;
  }

  priv->buffer[priv->index++] = (data0 ? 0x01 : 0x00);
  priv->wait = true;

  return OK;
}

static int wiegand_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct wiegand_upperhalf_s *upper = inode->i_private;
  int ret;

  ret = nxmutex_lock(&upper->lock);
  if (ret < 0)
  {
    return ret;
  }

  FAR struct wiegand_lowerhalf_s *lower = upper->lower;

  ret = lower->ops->interrupt(lower, wiegand_interrupt, upper);
  if (ret < 0)
  {
    return ret;
  }

  ret = lower->ops->setup(lower);
  if (ret < 0)
  {
    return ret;
  }

  memset(&upper->buffer, 0, sizeof(upper->buffer));
  upper->index = 0;

  nxmutex_unlock(&upper->lock);

  return OK;
}

static int wiegand_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  return 0;
}

static int wiegand_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct wiegand_upperhalf_s *priv = inode->i_private;
  int ret;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
  {
    return ret;
  }

  nxmutex_unlock(&priv->lock);
  return OK;
}

static void wiegand_wait_frame(FAR struct wiegand_upperhalf_s *priv)
{
  clock_t timeout;
  clock_t start;
  clock_t elapsed;

  timeout = MSEC2TICK(WIEGAND_TIMEOUT_MS);
  start = clock_systime_ticks();

  do
  {
    if (priv->wait)
    {
      start = clock_systime_ticks();
      priv->wait = false;
    }
    elapsed = clock_systime_ticks() - start;
  } while (elapsed < timeout);
}

static int wiegand_start_read_frame(FAR struct wiegand_upperhalf_s *priv,
                                    FAR struct wiegand_data_s *data)
{
  int i;

  // int facility_index = WIEGAND_FACILITY_CODE_BIT + 1;
  // int id_index = WIEGAND_FACILITY_CODE_BIT + WIEGAND_FACILITY_CODE_BIT + 1;

  data->id = data->aba_code = data->facility_code = 0;

  if (priv->index <= 0 || priv->index < WIEGAND_FORMAT_BIT)
  {
    return ERROR;
  }

  for (i = 1; i < WIEGAND_FORMAT_BIT; i++)
  {
    if (i <= WIEGAND_FACILITY_CODE_BIT)
    {
      data->facility_code <<= 1;
      data->facility_code |= priv->buffer[i];
    }
    else
    {
      data->id <<= 1;
      data->id |= priv->buffer[i];
    }
  }
  data->aba_code = (data->facility_code << WIEGAND_FACILITY_CODE_BIT);
  data->aba_code |= data->id;

  return OK;
}

static ssize_t wiegand_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen)
{
  int ret = 0;
  FAR struct inode *inode = filep->f_inode;
  FAR struct wiegand_upperhalf_s *upper;
  FAR struct wiegand_lowerhalf_s *lower;

  FAR struct wiegand_data_s *data = (FAR struct wiegand_data_s *)buffer;

  upper = inode->i_private;
  DEBUGASSERT(upper != NULL);
  lower = upper->lower;
  DEBUGASSERT(lower != NULL);

  ret = nxmutex_lock(&upper->lock);
  if (ret < 0)
  {
    return (ssize_t)ret;
  }

  if (buflen < sizeof(FAR struct wiegand_data_s))
  {
    snerr("ERROR: Not enough memory to read data sample.\n");
    return -ENOSYS;
  }

  wiegand_wait_frame(upper);

  ret = wiegand_start_read_frame(upper, data);
  if (ret < 0)
    {
      data->status = WIEGAND_READ_UNCOMPLETED;
    }

  else
    {
      data->status = WIEGAND_SUCCESS;
     }

  upper->index = 0;
  nxmutex_unlock(&upper->lock);

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int wiegand_register(FAR const char *devpath,
                     FAR struct wiegand_lowerhalf_s *lower)
{
  // FAR struct wiegand_dev_s *priv;
  FAR struct wiegand_upperhalf_s *upper;

  upper = (FAR struct wiegand_upperhalf_s *)
      kmm_malloc(sizeof(struct wiegand_upperhalf_s));
  if (upper == NULL)
  {
    snerr("ERROR: Failed to allocate instance\n");
    return -ENOMEM;
  }

  nxmutex_init(&upper->lock);
  upper->lower = lower;

  int ret = register_driver(devpath, &g_wiegandops, 0666, upper);
  if (ret < 0)
  {
    nxmutex_destroy(&upper->lock);
    kmm_free(upper);
    snerr("ERROR: Failed to register driver: %d\n", ret);
    return ret;
  }

  return ret;
}
#endif /* CONFIG_DRIVERS_WIEGAND */