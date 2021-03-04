/****************************************************************************
 * drivers/i2c/i2c_read.c
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

#include <nuttx/i2c/i2c_master.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: i2c_read
 *
 * Description:
 *   Receive a block of data from I2C. Each read operation will be an
 *   'atomic' operation in the sense that any other I2C actions will be
 *   serialized and pend until this read completes.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   buffer - A pointer to a buffer of data to receive the data from the
 *            device
 *   buflen - The requested number of bytes to be read
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

int i2c_read(FAR struct i2c_master_s *dev,
             FAR const struct i2c_config_s *config,
             FAR uint8_t *buffer, int buflen)
{
  struct i2c_msg_s msg;
  unsigned int flags;
  int ret;

  /* 7- or 10-bit? */

  flags = (config->addrlen == 10) ? I2C_M_TEN : 0;

  /* Setup for the transfer */

  msg.frequency = config->frequency,
  msg.addr      = config->address,
  msg.flags     = (flags | I2C_M_READ);
  msg.buffer    = buffer;
  msg.length    = buflen;

  /* Then perform the transfer. */

  ret = I2C_TRANSFER(dev, &msg, 1);
  return (ret >= 0) ? OK : ret;
}
