/****************************************************************************
 * drivers/sensors/qmi8658.c
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
#include <debug.h>
#include <stdlib.h>
#include <string.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/qmi8658.h>
#include <nuttx/fs/ioctl.h>

#include "qmi8658_base.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Use the common device structure from base header */

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Character driver methods */

static int qmi8658_open(FAR struct file *filep);
static int qmi8658_close(FAR struct file *filep);
static ssize_t qmi8658_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen);
static ssize_t qmi8658_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen);
static int qmi8658_ioctl(FAR struct file *filep, int cmd,
                         unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_qmi8658fops =
{
  qmi8658_open,     /* open */
  qmi8658_close,    /* close */
  qmi8658_read,     /* read */
  qmi8658_write,    /* write */
  NULL,             /* seek */
  qmi8658_ioctl,    /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: qmi8658_open
 *
 * Description:
 *   This function is called whenever the QMI8658 device is opened.
 *
 ****************************************************************************/

static int qmi8658_open(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: qmi8658_close
 *
 * Description:
 *   This function is called when the QMI8658 device is closed.
 *
 ****************************************************************************/

static int qmi8658_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: qmi8658_read
 *
 * Description:
 *   This function is called when the QMI8658 device is read.
 *
 ****************************************************************************/

static ssize_t qmi8658_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct qmi8658_dev_s *priv = inode->i_private;
  struct qmi8658_data_s data;
  int ret;

  if (buflen < sizeof(struct qmi8658_data_s))
    {
      return -EINVAL;
    }

  ret = qmi8658_read_all(priv, &data);
  if (ret < 0)
    {
      return ret;
    }

  memcpy(buffer, &data, sizeof(struct qmi8658_data_s));

  return sizeof(struct qmi8658_data_s);
}

/****************************************************************************
 * Name: qmi8658_write
 *
 * Description:
 *   This function is called when the QMI8658 device is written.
 *
 ****************************************************************************/

static ssize_t qmi8658_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen)
{
  /* QMI8658 is a read-only sensor, use ioctl for configuration */

  return -ENOSYS;
}

/****************************************************************************
 * Name: qmi8658_ioctl
 *
 * Description:
 *   This function is called when the QMI8658 device is ioctl'ed.
 *
 ****************************************************************************/

static int qmi8658_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct qmi8658_dev_s *priv = inode->i_private;
  int ret = -EINVAL;

  switch (cmd)
    {
    case SNIOC_SET_ACC_RANGE:
      ret = qmi8658_set_acc_range(priv, (uint8_t)arg);

      break;

    case SNIOC_SET_GYRO_RANGE:
      ret = qmi8658_set_gyro_range(priv, (uint8_t)arg);

      break;

    case SNIOC_ENABLE_SENSOR:

      /* Enable both accelerometer and gyroscope */

      ret = qmi8658_set_accelerometer(priv, true);

      if (ret == OK)
        {
          ret = qmi8658_set_gyroscope(priv, true);
        }

      break;

    case SNIOC_DISABLE_SENSOR:

      /* Disable both accelerometer and gyroscope */

      ret = qmi8658_set_accelerometer(priv, false);

      if (ret == OK)
        {
          ret = qmi8658_set_gyroscope(priv, false);
        }

      break;

    default:
      snerr("Unsupported ioctl command: 0x%x\n", cmd);
      ret = -ENOTTY;
      break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: qmi8658_register
 *
 * Description:
 *   Register the QMI8658 character device
 *
 ****************************************************************************/

int qmi8658_register(FAR const char *devpath,
                     FAR struct i2c_master_s *i2c, uint8_t addr)
{
  FAR struct qmi8658_dev_s *priv;
  int ret;

  DEBUGASSERT(devpath != NULL && i2c != NULL);

  priv = (FAR struct qmi8658_dev_s *)
          kmm_malloc(sizeof(struct qmi8658_dev_s));
  if (priv == NULL)
    {
      snerr("Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c = i2c;
  priv->addr = addr;
  priv->freq = CONFIG_QMI8658_I2C_FREQUENCY;

  ret = register_driver(devpath, &g_qmi8658fops, 0666, priv);
  if (ret < 0)
    {
      snerr("Failed to register driver: %d\n", ret);
      kmm_free(priv);
      return ret;
    }

  ret = qmi8658_initialize(priv);
  if (ret < 0)
    {
      snerr("Failed to initialize QMI8658: %d\n", ret);
      unregister_driver(devpath);
      kmm_free(priv);
      return ret;
    }

  sninfo("QMI8658 registered at %s\n", devpath);
  return OK;
}
