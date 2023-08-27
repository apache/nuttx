/****************************************************************************
 * boards/arm/stm32f7/stm32f777zit6-meadow/src/board_projlab.c
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

#include <sys/types.h>
#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/fs/fs.h>

#include "stm32_bh1750.h"
#include "stm32_bmi270.h"
#include "stm32f777zit6-meadow.h"

/****************************************************************************
 * Preprocessor definitions
 ****************************************************************************/

#if !defined(CONFIG_SENSORS_BH1750FVI)
#  error ProjectLab boards needs Light Sensor BH1750FVI enabled.
#endif

#if !defined(CONFIG_SENSORS_BMI270_I2C)
#  error ProjectLab boards needs Inertial Sensor BMI270 enabled.
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: init_projectlab
 *
 * Description:
 *   Initialize the fixed devices from ProjectLab board
 *
 ****************************************************************************/

int init_projectlab(void)
{
  int ret = OK;

  /* Initialize Light Sensor BH1750 */

  ret = board_bh1750_initialize(0, 1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: init BH1750 failed: %d\n", ret);
    }

  /* Initialize Accelerometer BMI270 */

  ret = board_bmi270_initialize(0, 1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: init BMI270 failed: %d\n", ret);
    }

  return ret;
}
