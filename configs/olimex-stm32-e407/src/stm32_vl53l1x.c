/****************************************************************************
 * configs/olimex-stm32-e407/src/stm32_vl53l1x.c
 *
 *   Copyright (C) 2019 Acutronics Robotics. All rights reserved.
 *   Author: Acutronics Robotics (Juan Flores Muñoz) <juan@erlerobotics.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <nuttx/sensors/vl53l1x.h>

#include "stm32.h"
#include "stm32_i2c.h"
#include "olimex-stm32-e407.h"

#ifdef HAVE_VL53L1

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VL53L1X_I2C_PORTNO 1   /* On I2C1 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_vl53l1xinitialize
 *
 * Description:
 *   Initialize and register the VL53L1X distance/light Sensor driver.
 *
 * Input parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/tof0"
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int stm32_vl53l1xinitialize(FAR const char *devpath)
{
  FAR struct i2c_master_s *i2c;
  int ret;

  sninfo("Initializing VL53L1X!\n");

  /* Initialize I2C */

  i2c = stm32_i2cbus_initialize(1);

  if (!i2c)
    {
      return -ENODEV;
    }

  /* Then register the barometer sensor */

  ret = vl53l1x_register(devpath, i2c);
  if (ret < 0)
    {
      snerr("ERROR: Error registering VL53L1X\n");
    }

  return ret;
}

#endif /* HAVE_VL53L1 */
