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
#include <errno.h>

#include <nuttx/signal.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/video/imgsensor.h>
#include <nuttx/video/imx335.h>

#if defined(CONFIG_STM32_DCMIPP) && defined(CONFIG_VIDEO_IMX335)
#  include <nuttx/video/v4l2_cap.h>
#endif

#ifdef CONFIG_SENSORS_LSM6DS3TRC
#  include <nuttx/sensors/lsm6ds3trc.h>
#endif

#include "stm32_gpio.h"
#include "stm32_i2c.h"
#include "stm32_dcmipp.h"
#include "stm32n6570-dk.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Module supply settling time after enable, with the sensor in reset */

#define CAMERA_POWER_DELAY_US  10000

#if defined(CONFIG_STM32_DCMIPP) && defined(CONFIG_VIDEO_IMX335)
#  define HAVE_IMX335 1
#endif

/* IMX335 reset release to the first I2C access */

#define CAMERA_RESET_DELAY_US  600

/* The MB1854 module has its own 24 MHz oscillator for the IMX335 */

#define CAMERA_INCK            24000000

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_camera_powered;

#ifdef HAVE_IMX335

/* IMX335: 2592x1944 RAW10 RGGB on two CSI-2 lanes.  The D-PHY receives the
 * sensor without errors when set to 800 to 1600 Mbit/s per lane (measured on
 * the DK); 1200 is the middle of that range.  The black level of 12 (8-bit
 * scale) is the dark level measured with the lens covered at the minimum
 * gain and exposure.  Unity white balance gains and an identity colour
 * matrix.  The sensor gain has 0.3 dB steps: 20 steps double it.
 */

static const struct stm32_dcmipp_config_s g_camera_config =
{
  .width       = 2592,
  .height      = 1944,
  .pixelformat = IMGSENSOR_PIX_FMT_SRGGB10P,
  .lanes       = 2,
  .bitrate     = 1200,
  .bpp         = 10,
  .bayer       = STM32_DCMIPP_BAYER_RGGB,
  .blacklevel  = 12,
  .gain        =
    {
      128, 128, 128
    },
  .ccm         =
    {
      {
        256, 0, 0
      },
      {
        0, 256, 0
      },
      {
        0, 0, 256
      },
    },
  .gamma       = true,
  .gainsteps   = 20,
};

#endif

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

#ifdef HAVE_IMX335

/****************************************************************************
 * Name: stm32_camera_initialize
 *
 * Description:
 *   Power the camera module, release the IMX335 reset and register the
 *   sensor, the DCMIPP and /dev/video.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int stm32_camera_initialize(void)
{
  FAR struct i2c_master_s *i2c;
  FAR struct imgsensor_s *sensor;
  int ret;

  /* Power the module, then release the sensor reset */

  stm32_camera_power();
  stm32_gpiowrite(GPIO_CAM_NRST, true);
  nxsig_usleep(CAMERA_RESET_DELAY_US);

  i2c = stm32_i2cbus_initialize(CAMERA_I2C_BUS);
  if (i2c == NULL)
    {
      return -ENODEV;
    }

  sensor = imx335_initialize(i2c, CAMERA_INCK);
  if (sensor == NULL)
    {
      return -ENODEV;
    }

  /* The DCMIPP registers itself in front of the raw sensor */

  ret = stm32_dcmipp_initialize(&g_camera_config, sensor);
  if (ret < 0)
    {
      return ret;
    }

  return capture_initialize("/dev/video");
}

#endif /* HAVE_IMX335 */

#ifdef CONFIG_SENSORS_LSM6DS3TRC

/****************************************************************************
 * Name: stm32_imu_initialize
 *
 * Description:
 *   Power the camera module and register its IMU, polled.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int stm32_imu_initialize(void)
{
  static struct lsm6ds3trc_config_s config;
  FAR struct i2c_master_s *i2c;

  stm32_camera_power();

  i2c = stm32_i2cbus_initialize(CAMERA_I2C_BUS);
  if (i2c == NULL)
    {
      return -ENODEV;
    }

  return lsm6ds3trc_register(i2c, IMU_I2C_ADDR, 0, &config);
}

#endif /* CONFIG_SENSORS_LSM6DS3TRC */
