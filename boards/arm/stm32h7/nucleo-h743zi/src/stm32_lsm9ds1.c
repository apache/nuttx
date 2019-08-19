/*****************************************************************************
 * boards/arm/stm32h7/nucleo-h743zi/src/stm32_lsm9ds1.c
 *
 *   Copyright (C) 2019 Greg Nutt. All rights reserved.
 *   Author: Mateusz Szafoni <raiden00@railab.me>
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
#include "stm32.h"
#include <nucleo-h743zi.h>
#include <nuttx/sensors/lsm9ds1.h>

/*****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_STM32H7_I2C1
#  error "LSM9DS1 driver requires CONFIG_STM32H7_I2C1 to be enabled"
#endif

#define LSM9DS1MAG_DEVPATH "/dev/lsm9ds1mag0"
#define LSM9DS1ACC_DEVPATH "/dev/lsm9ds1acc0"
#define LSM9DS1GYR_DEVPATH "/dev/lsm9ds1gyr0"

/*****************************************************************************
 * Public Functions
 ****************************************************************************/

/*****************************************************************************
 * Name: stm32_lsm9ds1_initialize
 *
 * Description:
 *   Initialize I2C-based LSM9DS1.
 ****************************************************************************/

int stm32_lsm9ds1_initialize(void)
{
  FAR struct i2c_master_s *i2c;
  int ret = OK;

  sninfo("Initializing LMS9DS1!\n");

#if defined(CONFIG_STM32H7_I2C1)
  i2c = stm32_i2cbus_initialize(LMS9DS1_I2CBUS);
  if (i2c == NULL)
    {
      return -ENODEV;
    }

  sninfo("INFO: Initializing LMS9DS1 9DoF sensor over I2C%d\n", LMS9DS1_I2CBUS);

  ret = lsm9ds1mag_register(LSM9DS1MAG_DEVPATH, i2c, LSM9DS1MAG_ADDR1);
  if (ret < 0)
    {
      snerr("ERROR: Failed to initialize LMS9DS1 mag driver %s\n", LSM9DS1MAG_DEVPATH);
      return -ENODEV;
    }

  ret = lsm9ds1gyro_register(LSM9DS1GYR_DEVPATH, i2c, LSM9DS1GYRO_ADDR1);
  if (ret < 0)
    {
      snerr("ERROR: Failed to initialize LMS9DS1 gyro driver %s\n", LSM9DS1MAG_DEVPATH);
      return -ENODEV;
    }

  ret = lsm9ds1accel_register(LSM9DS1ACC_DEVPATH, i2c, LSM9DS1ACCEL_ADDR1);
  if (ret < 0)
    {
      snerr("ERROR: Failed to initialize LMS9DS1 accel driver %s\n", LSM9DS1MAG_DEVPATH);
      return -ENODEV;
    }

  sninfo("INFO: LMS9DS1 sensor has been initialized successfully\n");
#endif

  return ret;
}
