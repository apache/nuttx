/****************************************************************************
 * drivers/i2c/i2c_writeread.c
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
 * Name: i2c_writeread
 *
 * Description:
 *   Send a block of data on I2C followed by restarted read access.  This
 *   provides a convenient wrapper to the transfer function.
 *
 * Input Parameters:
 *   dev     - Device-specific state data
 *   config  - Described the I2C configuration
 *   wbuffer - A pointer to the read-only buffer of data to be written to
 *             device
 *   wbuflen - The number of bytes to send from the buffer
 *   rbuffer - A pointer to a buffer of data to receive the data from the
 *             device
 *   rbuflen - The requested number of bytes to be read
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

int i2c_writeread(FAR struct i2c_master_s *dev,
                  FAR const struct i2c_config_s *config,
                  FAR const uint8_t *wbuffer, int wbuflen,
                  FAR uint8_t *rbuffer, int rbuflen)
{
  struct i2c_msg_s msg[2];
  unsigned int flags;
  int ret;

  /* 7- or 10-bit address? */

  DEBUGASSERT(config->addrlen == 10 || config->addrlen == 7);
  flags = (config->addrlen == 10) ? I2C_M_TEN : 0;

  /* Format two messages: The first is a write which is never terminated
   * with STOP condition.
   */

  msg[0].frequency  = config->frequency,
  msg[0].addr       = config->address;
  msg[0].flags      = flags | I2C_M_NOSTOP;
  msg[0].buffer     = (FAR uint8_t *)wbuffer;  /* Override const */
  msg[0].length     = wbuflen;

  /* The second is either a read (rbuflen > 0) with a repeated start or a
   * write (rbuflen < 0) with no restart.
   */

  if (rbuflen > 0)
    {
      msg[1].flags  = (flags | I2C_M_READ);
    }
  else
    {
      msg[1].flags  = (flags | I2C_M_NOSTART);
      rbuflen       = -rbuflen;
    }

  msg[1].frequency  = config->frequency,
  msg[1].addr       = config->address;
  msg[1].buffer     = rbuffer;
  msg[1].length     = rbuflen;

  /* Then perform the transfer. */

  ret = I2C_TRANSFER(dev, msg, 2);
  return (ret >= 0) ? OK : ret;
}
