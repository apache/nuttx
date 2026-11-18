/****************************************************************************
 * boards/arm/stm32n6/stm32n6570-dk/src/stm32_camera.c
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

#include <stdbool.h>

#include <nuttx/signal.h>

#include "stm32_gpio.h"
#include "stm32n6570-dk.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Module supply settling time after enable, with the sensor in reset */

#define CAMERA_POWER_DELAY_US  10000

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_camera_powered;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_camera_power
 *
 * Description:
 *   Power the camera module with the IMX335 held in reset.  Only the first
 *   call powers the module.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void stm32_camera_power(void)
{
  if (!g_camera_powered)
    {
      stm32_configgpio(GPIO_CAM_NRST);
      stm32_configgpio(GPIO_CAM_EN);
      stm32_gpiowrite(GPIO_CAM_EN, true);
      nxsig_usleep(CAMERA_POWER_DELAY_US);
      g_camera_powered = true;
    }
}
