/****************************************************************************
 * boards/arm/stm32n6/stm32n6570-dk/src/stm32_tof.c
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

#include <errno.h>

#include <nuttx/signal.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/video/vl53l5cx.h>

#include "stm32_exti.h"
#include "stm32_gpio.h"
#include "stm32_i2c.h"
#include "stm32n6570-dk.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2C interface start-up after LPn goes high */

#define TOF_LPN_DELAY_US  10000

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int stm32_tof_attach(FAR const struct vl53l5cx_config_s *config,
                            xcpt_t isr, FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct vl53l5cx_config_s g_tof_config =
{
  .addr   = TOF_I2C_ADDR,
  .attach = stm32_tof_attach,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_tof_attach
 *
 * Description:
 *   Attach and enable the INT handler, or disable it with isr NULL.  INT
 *   falls when a frame is ready.
 *
 * Input Parameters:
 *   config - The sensor configuration
 *   isr    - The INT handler, or NULL to disable INT
 *   arg    - The handler argument
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int stm32_tof_attach(FAR const struct vl53l5cx_config_s *config,
                            xcpt_t isr, FAR void *arg)
{
  return stm32_gpiosetevent(GPIO_TOF_INT, false, true, false, isr, arg);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_tof_initialize
 *
 * Description:
 *   Power the camera module, enable the VL53L5CX I2C interface (LPn) and
 *   register the sensor as a V4L2 depth camera.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int stm32_tof_initialize(void)
{
  FAR struct i2c_master_s *i2c;
  int ret;

  /* Power the module with LPn low, then enable the sensor I2C interface */

  stm32_configgpio(GPIO_TOF_INT);
  stm32_configgpio(GPIO_TOF_LPN);
  stm32_camera_power();
  stm32_gpiowrite(GPIO_TOF_LPN, true);
  nxsig_usleep(TOF_LPN_DELAY_US);

  ret = -ENODEV;
  i2c = stm32_i2cbus_initialize(TOF_I2C_BUS);
  if (i2c != NULL)
    {
      ret = vl53l5cx_register(TOF_DEVPATH, i2c, &g_tof_config);
    }

  if (ret < 0)
    {
      stm32_gpiowrite(GPIO_TOF_LPN, false);
    }

  return ret;
}
