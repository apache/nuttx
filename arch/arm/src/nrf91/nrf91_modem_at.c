/****************************************************************************
 * arch/arm/src/nrf91/nrf91_modem_at.c
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

#include <nuttx/fs/fs.h>
#include <nuttx/mutex.h>

#include <debug.h>
#include <fcntl.h>
#include <semaphore.h>
#include <string.h>

#include "nrf_modem_at.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NRF91_MODEM_AT_RX 255

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct nrf91_modem_at_s
{
  char    rxbuf[NRF91_MODEM_AT_RX];
  size_t  rx_i;
  sem_t   rx_sem;
  mutex_t lock;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void nrf91_modem_at_notify_handler(const char *notif);
static void nrf91_modem_at_resp_handler(const char *resp);
static ssize_t nrf91_modem_at_read(struct file *filep, char *buffer,
                                   size_t buflen);
static ssize_t nrf91_modem_at_write(struct file *filep, const char *buffer,
                                    size_t buflen);
static int nrf91_modem_at_ioctl(struct file *filep, int cmd,
                                unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_nrf91_modem_at_fops =
{
  NULL,                 /* open */
  NULL,                 /* close */
  nrf91_modem_at_read,  /* read */
  nrf91_modem_at_write, /* write */
  NULL,                 /* seek */
  nrf91_modem_at_ioctl, /* ioctl */
  NULL,                 /* mmap */
  NULL,                 /* truncate */
  NULL                  /* poll */
};

static struct nrf91_modem_at_s g_nrf91_modem_at;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf91_modem_at_notify_handler
 ****************************************************************************/

static void nrf91_modem_at_notify_handler(const char *notif)
{
  struct nrf91_modem_at_s *dev = &g_nrf91_modem_at;

  /* Copy notify */

  strncpy(&dev->rxbuf[dev->rx_i], notif, NRF91_MODEM_AT_RX - dev->rx_i);
  dev->rx_i += strlen(notif);

  /* Wake-up any thread waiting in recv */

  nxsem_post(&dev->rx_sem);
}

/****************************************************************************
 * Name: nrf91_modem_at_resp_handler
 ****************************************************************************/

static void nrf91_modem_at_resp_handler(const char *resp)
{
  struct nrf91_modem_at_s *dev = &g_nrf91_modem_at;

  /* Copy response */

  strncpy(&dev->rxbuf[dev->rx_i], resp, NRF91_MODEM_AT_RX - dev->rx_i);
  dev->rx_i += strlen(resp);

  /* Wake-up any thread waiting in recv */

  nxsem_post(&dev->rx_sem);
}

/****************************************************************************
 * Name: nrf91_modem_at_read
 ****************************************************************************/

static ssize_t nrf91_modem_at_read(struct file *filep, char *buffer,
                                   size_t len)
{
  struct nrf91_modem_at_s *dev   = NULL;
  struct inode            *inode = NULL;
  int                      ret   = 0;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  dev = (struct nrf91_modem_at_s *)inode->i_private;

  ret = nxmutex_lock(&dev->lock);
  if (ret < 0)
    {
      return ret;
    }

  if ((filep->f_oflags & O_NONBLOCK) != 0)
    {
      nxsem_trywait(&dev->rx_sem);
      ret = 0;
    }
  else
    {
      ret = nxsem_wait(&dev->rx_sem);
    }

  if (ret < 0)
    {
      return ret;
    }

  /* Get response data */

  if (len > dev->rx_i)
    {
      len = dev->rx_i;
    }

  strncpy(buffer, dev->rxbuf, len);
  dev->rx_i = 0;
  ret = len;

  nxmutex_unlock(&dev->lock);
  return ret;
}

/****************************************************************************
 * Name: nrf91_modem_at_write
 ****************************************************************************/

static ssize_t nrf91_modem_at_write(struct file *filep, const char *buffer,
                                    size_t len)
{
  struct nrf91_modem_at_s *dev   = NULL;
  struct inode            *inode = NULL;
  int                      ret   = 0;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  dev = (struct nrf91_modem_at_s *)inode->i_private;

  ret = nxmutex_lock(&dev->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Send AT command */

  ret = nrf_modem_at_cmd_async(nrf91_modem_at_resp_handler, buffer);

  nxmutex_unlock(&dev->lock);
  return ret;
}

/****************************************************************************
 * Name: nrf91_modem_at_ioct
 ****************************************************************************/

static int nrf91_modem_at_ioctl(struct file *filep, int cmd,
                                unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf91_at_register
 ****************************************************************************/

int nrf91_at_register(const char *path)
{
  struct nrf91_modem_at_s *dev = &g_nrf91_modem_at;
  int                      ret = OK;

  /* Initialize mutex & sem */

  memset(&g_nrf91_modem_at, 0, sizeof(struct nrf91_modem_at_s));

  nxmutex_init(&dev->lock);
  nxsem_init(&dev->rx_sem, 0, 0);

  /* Initialize AT modem */

  nrf_modem_at_notif_handler_set(nrf91_modem_at_notify_handler);
  nrf_modem_at_cmd_custom_set(NULL, 0);

  /* Register driver */

  ret = register_driver(path, &g_nrf91_modem_at_fops, 0666, dev);
  if (ret < 0)
    {
      nerr("ERROR: register_driver failed: %d\n", ret);
    }

  return ret;
}
