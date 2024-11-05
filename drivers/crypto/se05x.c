/****************************************************************************
 * drivers/crypto/se05x.c
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

/* Copyright 2023 NXP */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "pnt/pnt_se05x_api.h"
#include "se05x_internal.h"
#include <debug.h>
#include <nuttx/config.h>
#include <nuttx/crypto/se05x.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_CRYPTO_CONTROLSE
#warning Controlse is not available; This is probably not what you want.
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Character driver methods */

static int se05x_open(FAR struct file *filep);
static int se05x_close(FAR struct file *filep);
static ssize_t se05x_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen);
static ssize_t se05x_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen);
static int se05x_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const FAR struct file_operations g_fops =
{
    se05x_open, se05x_close, se05x_read, se05x_write,
    NULL,       se05x_ioctl, NULL
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int se05x_open(FAR struct file *filep)
{
  /* create se05x session */

  FAR struct inode *inode = filep->f_inode;
  FAR struct se05x_dev_s *priv = inode->i_private;
  nxmutex_lock(&priv->mutex);
  int res = pnt_se05x_open(priv) == 0 ? OK : ERROR;
  if (res == ERROR)
    {
      nxmutex_unlock(&priv->mutex);
    }

  return res;
}

static int se05x_close(FAR struct file *filep)
{
  /* stop se05x session */

  FAR struct inode *inode = filep->f_inode;
  FAR struct se05x_dev_s *priv = inode->i_private;
  pnt_se05x_close(priv);
  nxmutex_unlock(&priv->mutex);
  return OK;
}

static ssize_t se05x_read(FAR struct file *filep, char *buffer,
                           size_t buflen)
{
  return -ENOSYS;
}

static ssize_t se05x_write(FAR struct file *filep, const char *buffer,
                           size_t buflen)
{
  return -ENOSYS;
}

static int se05x_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct se05x_dev_s *priv = inode->i_private;
  int ret = -ENOTTY;

  switch (cmd)
    {
    case SEIOC_GET_INFO:
      {
        FAR struct se05x_info_s *info = (FAR struct se05x_info_s *)arg;
        ret = pnt_se05x_get_info(priv, info);
      }
      break;

    case SEIOC_GET_UID:
      {
        FAR struct se05x_uid_s *uid = (FAR struct se05x_uid_s *)arg;
        ret = pnt_se05x_get_uid(priv, uid);
      }
      break;

    case SEIOC_GENERATE_KEYPAIR:
      {
        FAR struct se05x_generate_keypair_s *generate_keypair_args =
            (FAR struct se05x_generate_keypair_s *)arg;
        ret = pnt_se05x_generate_keypair(priv, generate_keypair_args);
      }
      break;

    case SEIOC_SET_KEY:
      {
        FAR struct se05x_key_transmission_s *set_key_args =
            (FAR struct se05x_key_transmission_s *)arg;
        ret = pnt_se05x_set_public_key(priv, set_key_args);
      }
      break;

    case SEIOC_SET_DATA:
      {
        FAR struct se05x_key_transmission_s *set_key_args =
            (FAR struct se05x_key_transmission_s *)arg;
        ret = pnt_se05x_set_data(priv, set_key_args);
      }
      break;

    case SEIOC_GET_KEY:
      {
        FAR struct se05x_key_transmission_s *get_key_args =
            (FAR struct se05x_key_transmission_s *)arg;
        ret = pnt_se05x_get_key(priv, get_key_args);
      }
      break;

    case SEIOC_GET_DATA:
      {
        FAR struct se05x_key_transmission_s *get_data_args =
            (FAR struct se05x_key_transmission_s *)arg;
        ret = pnt_se05x_get_data(priv, get_data_args);
      }
      break;

    case SEIOC_DELETE_KEY:
      {
        ret = pnt_se05x_delete_key(priv, arg);
      }
      break;

    case SEIOC_DERIVE_SYMM_KEY:
      {
        FAR struct se05x_derive_key_s *derive_key_args =
            (FAR struct se05x_derive_key_s *)arg;
        ret = pnt_se05x_derive_key(priv, derive_key_args);
      }
      break;

    case SEIOC_CREATE_SIGNATURE:
      {
        FAR struct se05x_signature_s *create_signature_args =
            (FAR struct se05x_signature_s *)arg;
        ret = pnt_se05x_create_signature(priv, create_signature_args);
      }
      break;

    case SEIOC_VERIFY_SIGNATURE:
      {
        FAR struct se05x_signature_s *verify_signature_args =
            (FAR struct se05x_signature_s *)arg;
        ret = pnt_se05x_verify_signature(priv, verify_signature_args);
      }
      break;

    default:
      crypterr("ERROR: Unrecognized cmd: %d\n", cmd);
      ret = -ENOTTY;
      break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int se05x_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                   FAR struct se05x_config_s *config)
{
  int ret;

  FAR struct se05x_dev_s *priv;

  /* Sanity check */

  DEBUGASSERT(devpath != NULL);
  DEBUGASSERT(i2c != NULL);

  /* Initialize the device's structure */

  priv = (FAR struct se05x_dev_s *)kmm_malloc(sizeof(*priv));
  if (priv == NULL)
    {
      crypterr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->config = config;
  priv->i2c = i2c;

  /* Check se05x availability */

  pnt_se05x_open(priv);
  struct se05x_uid_s uid;
  ret = pnt_se05x_get_uid(priv, &uid);
  if (ret < 0)
    {
      crypterr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
      return -ENODEV;
    }

  pnt_se05x_close(priv);

  /* Register driver */

  ret = register_driver(devpath, &g_fops, 0666, priv);
  if (ret < 0)
    {
      crypterr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  nxmutex_init(&priv->mutex);

  return ret;
}
