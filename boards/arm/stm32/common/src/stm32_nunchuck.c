/****************************************************************************
 * boards/arm/stm32/common/src/stm32_nunchuck.c
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

#include <stdint.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>
#include <nuttx/input/nunchuck.h>

#include "stm32_i2c.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_nunchuk_initialize
 *
 * Description:
 *   Initialize and register the Nunchuck joystick driver driver.
 *
 * Input Parameters:
 *   devno - The device number, used to build the device path as
 *           /dev/nunchuckN
 *   busno - The I2C bus number
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_nunchuck_initialize(int devno, int busno)
{
  FAR struct i2c_master_s *i2c;
  char devpath[15];
  int ret;

  iinfo("Initializing Wii Nunchuck!\n");

  /* Initialize I2C */

  i2c = stm32_i2cbus_initialize(busno);
  if (i2c == NULL)
    {
      return -ENODEV;
    }

  /* Register the joystick device as /dev/nunchuck0 */

  snprintf(devpath, 15, "/dev/nunchuck%d", devno);
  iinfo("Initialize joystick driver: %s\n", devpath);

  ret = nunchuck_register(devpath, i2c);
  if (ret < 0)
    {
      ierr("ERROR: nunchuck_register failed: %d\n", ret);
    }

  return ret;
}
