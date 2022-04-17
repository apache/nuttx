/****************************************************************************
 * boards/arm/stm32l4/nucleo-l476rg/src/stm32_as726x.c
 *
 *   Copyright (C) 2019 Fabian Justi. All rights reserved.
 *   Author: Fabian Justi <Fabian.Justi@gmx.de>
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
#include <nuttx/sensors/as726x.h>

#include "stm32l4.h"
#include "stm32l4_i2c.h"
#include "nucleo-l476rg.h"

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_AS726X)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_as726xinitialize
 *
 * Description:
 *   Initialize and register the AS726X Spectral sensor.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/spectr0"
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int stm32_as726xinitialize(const char *devpath)
{
  struct i2c_master_s *i2c;
  int ret;

  sninfo("Initializing AS726X!\n");

  /* Initialize I2C */

  i2c = stm32l4_i2cbus_initialize(AS726X_I2C_PORTNO);

  if (!i2c)
    {
      return -ENODEV;
    }

  /* Then register the light sensor */

  ret = as726x_register(devpath, i2c);
  if (ret < 0)
    {
      snerr("ERROR: Error registering AS726X\n");
    }

  return ret;
}

#endif /* CONFIG_I2C && CONFIG_SENSORS_AS726X && CONFIG_STM32_I2C1 */
