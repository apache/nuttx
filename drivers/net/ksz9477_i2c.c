/****************************************************************************
 * drivers/net/ksz9477_i2c.c
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
#include <nuttx/net/ksz9477.h>
#include <stdbool.h>
#include <errno.h>
#include "ksz9477_reg.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct i2c_master_s *g_ksz9477_i2c_bus;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void setup_i2c_transfer(struct i2c_msg_s *msg, uint8_t *data,
                               size_t len, bool read)
{
  msg[0].frequency = KSZ9477_I2C_SPEED;
  msg[0].addr      = KSZ9477_I2C_ADDRESS;
  msg[0].flags     = read ? I2C_M_READ : 0;
  msg[0].buffer    = data;
  msg[0].length    = len;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int ksz9477_read(struct ksz9477_transfer_s *read_msg)
{
  struct i2c_msg_s msg[2];

  /* Set up to write the address */

  setup_i2c_transfer(&msg[0], (uint8_t *)&read_msg->reg,
                     sizeof(read_msg->reg), false);

  /* Followed by the read data */

  setup_i2c_transfer(&msg[1], (uint8_t *)&read_msg->data,
                     read_msg->len - sizeof(read_msg->reg), true);

  return I2C_TRANSFER(g_ksz9477_i2c_bus, msg, 2);
}

int ksz9477_write(struct ksz9477_transfer_s *write_msg)
{
  struct i2c_msg_s msg;

  /* Set up to write the address and data */

  setup_i2c_transfer(&msg, (uint8_t *)&write_msg->reg,
                     write_msg->len, false);

  return I2C_TRANSFER(g_ksz9477_i2c_bus, &msg, 1);
}

/****************************************************************************
 * Name: ksz9477_i2c_init
 *
 * Description:
 *   Stores the configured i2c_master and calls the main init function
 *
 * Input Parameters:
 *   i2c_bus:     The i2c master used as a control interface
 *   master_port: The switch port connected to the host MAC
 * Returned Value:
 *   OK or negative error number
 *
 ****************************************************************************/

int ksz9477_i2c_init(struct i2c_master_s *i2c_bus,
                     ksz9477_port_t master_port)
{
  if (!i2c_bus)
    {
      return -EINVAL;
    }

  g_ksz9477_i2c_bus = i2c_bus;

  /* Call the main init function */

  return ksz9477_init(master_port);
}
