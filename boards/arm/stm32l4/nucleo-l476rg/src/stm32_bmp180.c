/****************************************************************************
 * boards/arm/stm32l4/nucleo-l476rg/src/stm32_bmp180.c
 *
 *   Copyright (C) 2015 Alan Carvalho de Assis. All rights reserved.
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
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
#include <nuttx/sensors/bmp180.h>

#include "stm32l4.h"
#include "stm32l4_i2c.h"
#include "nucleo-l476rg.h"

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_BMP180)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bmp180initialize
 *
 * Description:
 *   Initialize and register the MPL115A Pressure Sensor driver.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/press0"
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int stm32_bmp180initialize(FAR const char *devpath)
{
  FAR struct i2c_master_s *i2c;
  int ret;

  sninfo("Initializing BMP180!\n");

  /* Initialize I2C */

  i2c = stm32l4_i2cbus_initialize(BMP180_I2C_PORTNO);

  if (!i2c)
    {
      return -ENODEV;
    }

  /* Then register the barometer sensor */

  ret = bmp180_register(devpath, i2c);
  if (ret < 0)
    {
      snerr("ERROR: Error registering BM180\n");
    }

  return ret;
}

#endif /* CONFIG_I2C && CONFIG_SENSORS_BMP180 */
