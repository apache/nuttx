/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_hostif.c
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
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/irq.h>

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <semaphore.h>
#include <fcntl.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <arch/chip/hostif.h>

#include "chip.h"
#include "arm_internal.h"
#include "cxd56_clock.h"
#include "cxd56_pinconfig.h"
#include "cxd56_icc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Debug */

#ifdef CONFIG_CXD56_HOSTIF_DEBUG_ERROR
#define hiferr(format, ...)   _err(format, ##__VA_ARGS__)
#else
#define hiferr(x, ...)
#endif
#ifdef CONFIG_CXD56_HOSTIF_DEBUG_WARN
#define hifwarn(format, ...)  _warn(format, ##__VA_ARGS__)
#else
#define hifwarn(x, ...)
#endif
#ifdef CONFIG_CXD56_HOSTIF_DEBUG_INFO
#define hifinfo(format, ...)  _info(format, ##__VA_ARGS__)
#else
#define hifinfo(x, ...)
#endif

/* Message id definitions */

#define HIF_I2C_INIT      1
#define HIF_SPI_INIT      2
#define HIF_READ_DEVICE   3
#define HIF_WRITE_DEVICE  4

/* Message timeout definition in units of msec */

#define HIF_TIMEOUT 5000

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Host interface device structure for each buffer */

struct cxd56_hifdev_s
{
  int           id;
  uint32_t      flags;
  const void    *buffer;
  size_t        len;
  mutex_t       lock;
  int           crefs;
};

/* Host interface driver structure */

struct cxd56_hifdrv_s
{
  struct cxd56_hifdev_s *dev;
  int           ndev;
  sem_t         sync;
  int           errcode;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Character driver methods */

static int     hif_open(struct file *filep);
static int     hif_close(struct file *filep);
static off_t   hif_seek(struct file *filep, off_t offset,
                        int whence);
static ssize_t hif_read(struct file *filep, char *buffer,
                        size_t len);
static ssize_t hif_write(struct file *filep,
                         const char *buffer, size_t len);
static int     hif_ioctl(struct file *filep, int cmd,
                         unsigned long arg);
static int     hif_poll(struct file *filep, struct pollfd *fds,
                        bool setup);
static int     hif_unlink(struct inode *inode);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Host interface driver */

static struct cxd56_hifdrv_s g_hifdrv =
{
  .sync = SEM_INITIALIZER(0),
};

/* Host interface operations */

static const struct file_operations g_hif_fops =
{
  hif_open,    /* open */
  hif_close,   /* close */
  hif_read,    /* read */
  hif_write,   /* write */
  hif_seek,    /* seek */
  hif_ioctl,   /* ioctl */
  hif_poll     /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , hif_unlink /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int hif_sendmsg(uint8_t id, void *arg)
{
  struct cxd56_hifdrv_s *drv = &g_hifdrv;
  iccmsg_t msg;
  int ret;

  /* Check parameters */

  DEBUGASSERT((HIF_I2C_INIT <= id) && (id <= HIF_WRITE_DEVICE));
  DEBUGASSERT(arg);

  /* Send any message to system CPU */

  msg.cpuid = 0;
  msg.msgid = id;
  msg.protodata = id;
  msg.data  = (uint32_t)arg;

  ret = cxd56_iccsend(CXD56_PROTO_HOSTIF, &msg, HIF_TIMEOUT);
  if (ret < 0)
    {
      hiferr("ERROR: Send message (%d)\n", ret);
      return ret;
    }

  /* Wait for reply message from system CPU */

  nxsem_wait_uninterruptible(&drv->sync);

  /* Get the error code returned from system cpu */

  ret = drv->errcode;

  return ret;
}

static int hif_open(struct file *filep)
{
  struct inode *inode;
  struct cxd56_hifdev_s *priv;

  DEBUGASSERT(filep && filep->f_inode);
  inode = filep->f_inode;

  priv = (struct cxd56_hifdev_s *)inode->i_private;
  DEBUGASSERT(priv);

  /* Check parameters */

  if ((filep->f_oflags & O_WRONLY) != 0 &&
      (filep->f_oflags & O_RDONLY) != 0)
    {
      return -EACCES;
    }

  if ((filep->f_oflags & O_RDONLY) &&
      ((priv->flags & HOSTIF_BUFF_ATTR_READ) == 0))
    {
      return -EINVAL;
    }

  if ((filep->f_oflags & O_WRONLY) &&
      ((priv->flags & HOSTIF_BUFF_ATTR_READ) != 0))
    {
      return -EINVAL;
    }

  /* Increment reference counter */

  nxmutex_lock(&priv->lock);

  priv->crefs++;
  DEBUGASSERT(priv->crefs > 0);

  if (priv->crefs > 1)
    {
      nxmutex_unlock(&priv->lock);
      return OK;
    }

  /* Check if non-blocking mode */

  if (filep->f_oflags & O_NONBLOCK)
    {
      priv->flags |= O_NONBLOCK;
    }

  nxmutex_unlock(&priv->lock);
  return OK;
}

static int hif_close(struct file *filep)
{
  struct inode *inode;
  struct cxd56_hifdev_s *priv;

  DEBUGASSERT(filep && filep->f_inode);
  inode = filep->f_inode;

  priv = (struct cxd56_hifdev_s *)inode->i_private;
  DEBUGASSERT(priv);

  /* Decrement reference counter */

  nxmutex_lock(&priv->lock);

  DEBUGASSERT(priv->crefs > 0);
  priv->crefs--;

  nxmutex_unlock(&priv->lock);
  return OK;
}

static off_t hif_seek(struct file *filep, off_t offset, int whence)
{
  return OK;
}

static ssize_t hif_read(struct file *filep, char *buffer, size_t len)
{
  struct inode *inode;
  struct cxd56_hifdev_s *priv;
  int ret;

  DEBUGASSERT(filep && filep->f_inode);
  inode = filep->f_inode;

  priv = (struct cxd56_hifdev_s *)inode->i_private;
  DEBUGASSERT(priv);

  /* Check parameters */

  DEBUGASSERT(buffer);

  if ((filep->f_oflags & O_RDONLY) == 0)
    {
      return -EACCES;
    }

  /* Receive data from host */

  priv->buffer = buffer;
  priv->len = len;

  ret = hif_sendmsg(HIF_READ_DEVICE, priv);

  return ret;
}

static ssize_t hif_write(struct file *filep,
                         const char *buffer, size_t len)
{
  struct inode *inode;
  struct cxd56_hifdev_s *priv;
  int ret;

  DEBUGASSERT(filep && filep->f_inode);
  inode = filep->f_inode;

  priv = (struct cxd56_hifdev_s *)inode->i_private;
  DEBUGASSERT(priv);

  /* Check parameters */

  DEBUGASSERT(buffer);

  if ((filep->f_oflags & O_WRONLY) == 0)
    {
      return -EACCES;
    }

  /* Send data to host */

  priv->buffer = buffer;
  priv->len = len;

  ret = hif_sendmsg(HIF_WRITE_DEVICE, priv);

  return ret;
}

static int hif_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  return OK;
}

static int hif_poll(struct file *filep,
                    struct pollfd *fds, bool setup)
{
  return OK;
}

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int hif_unlink(struct inode *inode)
{
  return OK;
}
#endif

static int hif_rxhandler(int cpuid, int protoid,
                         uint32_t pdata, uint32_t data,
                         void *userdata)
{
  struct cxd56_hifdrv_s *drv = &g_hifdrv;

  DEBUGASSERT(cpuid == 0);
  DEBUGASSERT(protoid == CXD56_PROTO_HOSTIF);

  drv->errcode = (int)data;

  nxsem_post(&drv->sync);

  return OK;
}

static int hif_initialize(struct hostif_buff_s *buffer)
{
  struct cxd56_hifdrv_s *drv = &g_hifdrv;
  struct cxd56_hifdev_s *priv;
  char devpath[16];
  int num;
  int ret;

  /* Check parameters */

  DEBUGASSERT(buffer);

  /* Get the number of devices */

  for (num = 0; num < MAX_BUFFER_NUM; num++)
    {
      if (buffer[num].size == 0)
        {
          break;
        }
    }

  /* Setup driver structure */

  drv->dev =
    (struct cxd56_hifdev_s *)kmm_malloc(sizeof(struct cxd56_hifdev_s) * num);
  if (drv->dev == NULL)
    {
      hiferr("ERROR: hostif allocation failed\n");

      return -ENOMEM;
    }

  drv->ndev = num;

  /* Setup each device structure */

  for (num = 0; num < drv->ndev; num++)
    {
      priv = &drv->dev[num];

      priv->id = num;
      priv->flags = buffer[num].flag;
      snprintf(devpath, sizeof(devpath), "/dev/hostif%c%d",
               (priv->flags & HOSTIF_BUFF_ATTR_READ) ? 'r' : 'w', num);

      ret = register_driver(devpath, &g_hif_fops, 0666, priv);
      if (ret < 0)
        {
          hiferr("ERROR: Failed to register %s (%d)\n", devpath, ret);
          kmm_free(drv->dev);
          return ret;
        }

      nxmutex_init(&priv->lock);
      priv->crefs = 0;
    }

  /* Enable hostif clock */

  ret = cxd56_hostif_clock_enable();
  if (ret < 0)
    {
      hiferr("ERROR: Enable clock (%d)\n", ret);
      kmm_free(drv->dev);
      return ret;
    }

  /* Initialize communication with system CPU */

  cxd56_iccinit(CXD56_PROTO_HOSTIF);

  return cxd56_iccregisterhandler(CXD56_PROTO_HOSTIF, hif_rxhandler, NULL);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hostif_i2cinitialize
 *
 * Description:
 *   Initialize the host interface for I2C slave
 *
 * Input Parameter:
 *   config - pointer to I2C buffer configuration
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int hostif_i2cinitialize(struct hostif_i2cconf_s *config)
{
  int ret;

  DEBUGASSERT(config);

  /* Initialize common driver */

  ret = hif_initialize(config->buff);
  if (ret < 0)
    {
      hiferr("ERROR: Failed to initialize (%d)\n", ret);
      return ret;
    }

  /* Initialize I2C driver */

  ret = hif_sendmsg(HIF_I2C_INIT, config);
  if (ret < 0)
    {
      hiferr("ERROR: Initialize I2C (%d)\n", ret);
      return ret;
    }

  /* Enable hostif sequencer clock */

  ret = cxd56_hostseq_clock_enable();
  if (ret < 0)
    {
      hiferr("ERROR: Enable sequencer clock (%d)\n", ret);
      return ret;
    }

  /* Pin setting */

  CXD56_PIN_CONFIGS(PINCONFS_SPI2A_I2C3);

  return OK;
}

/****************************************************************************
 * Name: hostif_spiinitialize
 *
 * Description:
 *   Initialize the host interface for SPI slave
 *
 * Input Parameter:
 *   config - pointer to SPI buffer configuration
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int hostif_spiinitialize(struct hostif_spiconf_s *config)
{
  int ret;

  DEBUGASSERT(config);

  /* Initialize common driver */

  ret = hif_initialize(config->buff);
  if (ret < 0)
    {
      hiferr("ERROR: Failed to initialize (%d)\n", ret);
      return ret;
    }

  /* Initialize SPI driver */

  ret = hif_sendmsg(HIF_SPI_INIT, config);
  if (ret < 0)
    {
      hiferr("ERROR: Initialize SPI (%d)\n", ret);
      return ret;
    }

  /* Enable hostif sequencer clock */

  ret = cxd56_hostseq_clock_enable();
  if (ret < 0)
    {
      hiferr("ERROR: Enable sequencer clock (%d)\n", ret);
      return ret;
    }

  /* Pin setting */

  CXD56_PIN_CONFIGS(PINCONFS_SPI2);

  return OK;
}

/****************************************************************************
 * Name: hostif_uninitialize
 *
 * Description:
 *   Uninitialize the host interface
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int hostif_uninitialize(void)
{
  struct cxd56_hifdrv_s *drv = &g_hifdrv;
  struct cxd56_hifdev_s *priv;
  char devpath[16];
  int num;

  for (num = 0; num < drv->ndev; num++)
    {
      priv = &drv->dev[num];

      snprintf(devpath, sizeof(devpath), "/dev/hostif%c%d",
               (priv->flags & HOSTIF_BUFF_ATTR_READ) ? 'r' : 'w', num);
      unregister_driver(devpath);
    }

  if (drv->dev)
    {
      kmm_free(drv->dev);
    }

  return OK;
}
