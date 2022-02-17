/****************************************************************************
 * boards/xtensa/esp32/common/src/esp32_tca9548a.c
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

#include <stdio.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/i2c/tca9548a.h>

#include "esp32_board_i2c.h"
#include "esp32_i2c.h"
#include "esp32_tca9548a.h"

struct tca9548a_dev_s *g_tca9448a_devs[8] =
{
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_tca9548a_initialize
 *
 * Description:
 *   Initialize and register the TCA9548A Multiplexer.
 *
 * Input Parameters:
 *   devno - The device number associated to I2C address, 0=0x70 ... 7=0x77
 *   busno - The I2C bus number
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_tca9548a_initialize(int devno, int busno)
{
  struct i2c_master_s *i2c;
  uint8_t addr;

  i2cinfo("Initializing TCA9548A!\n");

  /* TCA9548A valid addresses are 0:0x70 - 7:0x77 */

  addr = TCA9548A_BASEADDR0 + devno;

  if (addr < TCA9548A_BASEADDR0 || addr > TCA9548A_BASEADDR7)
    {
      i2cerr("Invalid devno: %d! Valid range: 0 to 7!\n", devno);
      return -EINVAL;
    }

  /* Initialize TCA9548A */

  i2c = esp32_i2cbus_initialize(busno);
  if (i2c != NULL)
    {
      /* Check if this position is not used */

      if (g_tca9448a_devs[devno] != NULL)
        {
          i2cerr("ERROR: This tca9548a_devs[%d] is in use!\n", devno);
          return -EFAULT;
        }

      /* Then try to initialize the TCA9548A */

      g_tca9448a_devs[devno] = tca9548a_initialize(i2c, addr);

      if (g_tca9448a_devs[devno] == NULL)
        {
          i2cerr("ERROR: Error registering TCA9548A in I2C%d\n", busno);
          return -ENODEV;
        }
    }
  else
    {
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: esp32_i2cmux_getmaster
 *
 * Description:
 *   Returns an I2C Master for TCA9548A multiplexer channel.
 *
 *   NOTE: esp32_i2cmux_getmaster() is generic name, it can be used to return
 *   an I2C Master for others I2C Master, not only TCA9548A.
 *
 * Input Parameters:
 *   devno   - The device number, it is the TCA9548A I2C_Addr minus 0x70.
 *   channel - The TCA9548A's channel where the device will be added.
 *
 * Returned Value:
 *   Common i2c multiplexer device instance; NULL on failure.
 *
 ****************************************************************************/

struct i2c_master_s *esp32_i2cmux_getmaster(int devno, uint8_t channel)
{
  int addr = devno + TCA9548A_BASEADDR0;

  /* Check if we have a valid device number */

  if (addr < TCA9548A_BASEADDR0 || addr > TCA9548A_BASEADDR7)
    {
      i2cerr("Invalid devno: %d! Valid range: 0 to 7!\n", devno);
      return NULL;
    }

  /* Check if we have a valid channel */

  if (channel > TCA9548A_SEL_CH7)
    {
      snerr("Invalid channel: %d! Valid range: 0 to 7!\n", channel);
      return NULL;
    }

  return tca9548a_lower_half(g_tca9448a_devs[devno], channel);
}
