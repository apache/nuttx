/****************************************************************************
 * drivers/crypto/pnt/sm_i2c.c
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

#include "sm_i2c.h"
#include "../se05x_internal.h"
#include <nuttx/crypto/se05x.h>
#include <nuttx/i2c/i2c_master.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int se05x_i2c_write(FAR struct se05x_dev_s *priv,
                           FAR const uint8_t *buffer, ssize_t buflen)
{
  struct i2c_msg_s msg;
  int ret;

  /* Setup for the transfer */

  msg.frequency = priv->config->frequency;
  msg.addr = priv->config->address;
  msg.flags = 0;
  msg.buffer = (FAR uint8_t *)buffer; /* Override const */
  msg.length = buflen;

  /* Then perform the transfer. */

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  return (ret >= 0) ? OK : ret;
}

static int se05x_i2c_read(FAR struct se05x_dev_s *priv, FAR uint8_t *buffer,
                          ssize_t buflen)
{
  struct i2c_msg_s msg;
  int ret;

  /* Setup for the transfer */

  msg.frequency = priv->config->frequency;
  msg.addr = priv->config->address;
  msg.flags = I2C_M_READ;
  msg.buffer = buffer;
  msg.length = buflen;

  /* Then perform the transfer. */

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  return (ret >= 0) ? OK : ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: axI2CInit
 *
 * Description:
 *   Initialize i2c (plug and trust mw hook)
 *   pDevName should contain the private device struct
 *
 ****************************************************************************/

unsigned int sm_i2c_init(FAR void **conn_ctx, FAR const char *dev_name)
{
  *conn_ctx = (FAR void *)dev_name;
  return I2C_OK;
}

void sm_i2c_term(FAR void *conn_ctx, int mode)
{
  (void)conn_ctx;
  (void)mode;
}

unsigned int sm_i2c_write(FAR void *conn_ctx, unsigned char bus,
                          unsigned char addr, FAR unsigned char *tx,
                          unsigned short tx_len)
{
  (void)bus;
  int result = se05x_i2c_write(conn_ctx, tx, (ssize_t)tx_len);
  return result == OK ? I2C_OK : I2C_FAILED;
}

unsigned int sm_i2c_read(FAR void *conn_ctx, uint8_t bus, uint8_t addr,
                         FAR uint8_t *rx, unsigned short rx_len)
{
  (void)bus;
  int result = se05x_i2c_read(conn_ctx, rx, (ssize_t)rx_len);
  return result == OK ? I2C_OK : I2C_FAILED;
}
