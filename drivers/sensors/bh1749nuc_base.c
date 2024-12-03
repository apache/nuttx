/****************************************************************************
 * drivers/sensors/bh1749nuc_base.c
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

#include "bh1749nuc_base.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bh1749nuc_getreg8
 *
 * Description:
 *   Read from an 8-bit BH1749NUC register
 *
 ****************************************************************************/

uint8_t bh1749nuc_getreg8(FAR struct bh1749nuc_dev_s *priv, uint8_t regaddr)
{
  struct i2c_msg_s msg[2];
  uint8_t regval = 0;
  int ret;

  msg[0].frequency = priv->freq;
  msg[0].addr      = priv->addr;
  msg[0].flags     = I2C_M_NOSTOP;
  msg[0].buffer    = &regaddr;
  msg[0].length    = 1;

  msg[1].frequency = priv->freq;
  msg[1].addr      = priv->addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = &regval;
  msg[1].length    = 1;

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  if (ret < 0)
    {
      snerr("I2C_TRANSFER failed: %d\n", ret);
    }

  return regval;
}

/****************************************************************************
 * Name: bh1749nuc_read16
 *
 * Description:
 *   Read 16-bit register
 *
 ****************************************************************************/

uint16_t bh1749nuc_read16(FAR struct bh1749nuc_dev_s *priv, uint8_t regaddr)
{
  struct i2c_msg_s msg[2];
  uint8_t regval[2];
  int ret;

  msg[0].frequency = priv->freq;
  msg[0].addr      = priv->addr;
  msg[0].flags     = I2C_M_NOSTOP;
  msg[0].buffer    = &regaddr;
  msg[0].length    = 1;

  msg[1].frequency = priv->freq;
  msg[1].addr      = priv->addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = (FAR uint8_t *)&regval;
  msg[1].length    = 2;

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  if (ret < 0)
    {
      snerr("I2C_TRANSFER failed: %d\n", ret);
    }

  return regval[1] << 8 | regval[0] << 0;
}

/****************************************************************************
 * Name: bh1749nuc_putreg8
 *
 * Description:
 *   Write to an 8-bit BH1749NUC register
 *
 ****************************************************************************/

void bh1749nuc_putreg8(FAR struct bh1749nuc_dev_s *priv,
                       uint8_t regaddr, uint8_t regval)
{
  struct i2c_msg_s msg[2];
  uint8_t txbuffer[2];
  int ret;

  txbuffer[0] = regaddr;
  txbuffer[1] = regval;

  msg[0].frequency = priv->freq;
  msg[0].addr      = priv->addr;
  msg[0].flags     = 0;
  msg[0].buffer    = txbuffer;
  msg[0].length    = 2;

  ret = I2C_TRANSFER(priv->i2c, msg, 1);
  if (ret < 0)
    {
      snerr("I2C_TRANSFER failed: %d\n", ret);
    }
}

/****************************************************************************
 * Name: bh1749nuc_checkid
 *
 * Description:
 *   Read and verify the BH1749NUC chip ID
 *
 ****************************************************************************/

int bh1749nuc_checkid(FAR struct bh1749nuc_dev_s *priv)
{
  uint8_t id;

  /* Read Manufact ID */

  id = bh1749nuc_getreg8(priv, BH1749NUC_MANUFACTURER_ID);
  if (id != BH1749NUC_MANUFACTID)
    {
      /* Manufact ID is not Correct */

      snerr("Wrong Manufact ID! %02x\n", id);
      return -ENODEV;
    }

  /* Read Part ID */

  id = bh1749nuc_getreg8(priv, BH1749NUC_SYSTEM_CONTROL);
  if ((id & 0x3f) != BH1749NUC_PARTID)
    {
      /* Part ID is not Correct */

      snerr("Wrong Part ID! %02x\n", id);
      return -ENODEV;
    }

  return OK;
}
