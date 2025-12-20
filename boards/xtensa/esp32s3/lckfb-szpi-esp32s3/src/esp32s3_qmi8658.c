/****************************************************************************
 * boards/xtensa/esp32s3/lckfb-szpi-esp32s3/src/esp32s3_qmi8658.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <unistd.h>
#include <stdlib.h>
#include <debug.h>
#include <assert.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/sensors/qmi8658.h>

#include "esp32s3_i2c.h"
#include "esp32s3-szpi.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_qmi8658_initialize
 *
 * Description:
 *   Initialize and register the QMI8658 6-axis IMU sensor driver.
 *
 *   This function registers the uORB interface which creates:
 *   - "/dev/uorb/sensor_accel0" for accelerometer data
 *   - "/dev/uorb/sensor_gyro0" for gyroscope data
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int esp32s3_qmi8658_initialize(void)
{
  struct i2c_master_s *i2c;
  int ret;

  i2c = esp32s3_i2cbus_initialize(QMI8658_I2C_PORT);
  if (!i2c)
    {
      i2cerr("Initialize I2C bus failed!\n");
      return -EINVAL;
    }

  /* Register QMI8658 uORB sensor device */

  ret = qmi8658_uorb_register(0, i2c, QMI8658_I2C_ADDR);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to register QMI8658 uORB driver: %d\n",
              ret);
    }

  return ret;
}
