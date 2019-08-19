/*****************************************************************************
 * boards/arm/stm32l4/nucleo-l476rg/src/stm32_lsm303agr.c
 *
 *   Copyright (C) 2018 Greg Nutt. All rights reserved.
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
 ****************************************************************************/

/*****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/board.h>
#include "stm32l4.h"
#include <nucleo-l476rg.h>
#include <nuttx/sensors/lsm303agr.h>

/*****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_STM32L4_I2C1
#  error "LSM303AGR driver requires CONFIG_STM32L4_I2C1 to be enabled"
#endif

/*****************************************************************************
 * Public Functions
 ****************************************************************************/

/*****************************************************************************
 * Name: stm32l4_lsm303agr_initialize
 *
 * Description:
 *   Initialize I2C-based LSM303AGR.
 ****************************************************************************/

int stm32l4_lsm303agr_initialize(char *devpath)
{
  FAR struct i2c_master_s *i2c;
  int ret = OK;

  sninfo("INFO: Initializing LMS303AGR sensor over I2C\n");

#if defined(CONFIG_STM32L4_I2C1)
  i2c = stm32l4_i2cbus_initialize(1);
  if (i2c == NULL)
    {
      return -ENODEV;
    }

  ret = lsm303agr_sensor_register("/dev/lsm303mag0", i2c,
                                  LSM303AGRMAGNETO_ADDR);
  if (ret < 0)
    {
      snerr("ERROR: Failed to initialize LMS303AGR magneto driver %s\n",
            devpath);
      return -ENODEV;
    }

  sninfo("INFO: LMS303AGR sensor has been initialized successfully\n");
#endif

  return ret;
}
