/****************************************************************************
 * boards/xtensa/esp32s3/lckfb-szpi-esp32s3/src/esp32s3_board_camera.c
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

#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <nuttx/debug.h>
#include <sys/ioctl.h>

#include <nuttx/arch.h>

#include <nuttx/ioexpander/gpio.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/video/imgsensor.h>
#include <nuttx/video/imgdata.h>
#include <nuttx/video/v4l2_cap.h>
#include <nuttx/video/gc0308.h>

#include "esp32s3_i2c.h"
#include "esp32s3_cam.h"
#include "esp32s3-szpi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PCA9557 DVP_PWDN is registered as /dev/gpio2 */

#define DVP_PWDN_PATH  "/dev/gpio2"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_camera_initialize
 *
 * Description:
 *   Initialize the camera subsystem:
 *   1. Power on camera via PCA9557 DVP_PWDN
 *   2. Register GC0308 imgsensor
 *   3. Register ESP32-S3 CAM imgdata
 *   4. Register V4L2 capture device
 *
 ****************************************************************************/

int esp32s3_camera_initialize(void)
{
  struct imgdata_s *imgdata;
  struct imgsensor_s *imgsensor;
  struct i2c_master_s *i2c;
  int fd;
  int ret;

  /* Step 1: Power on camera by driving DVP_PWDN low via PCA9557 GPIO */

  fd = open(DVP_PWDN_PATH, O_RDWR);
  if (fd >= 0)
    {
      /* Write 0 to deassert PWDN (active high) */

      ioctl(fd, GPIOC_WRITE, 0);
      close(fd);
      up_mdelay(10);  /* Wait for sensor power-up */
    }
  else
    {
      snerr("ERROR: Failed to open %s: %d\n", DVP_PWDN_PATH, errno);
      return -errno;
    }

  /* Step 2: Register ESP32-S3 CAM imgdata driver.
   *   This also starts XCLK output — GC0308 needs clock before I2C.
   */

  imgdata = esp32s3_cam_initialize();
  if (!imgdata)
    {
      snerr("ERROR: Failed to register CAM imgdata\n");
      return -ENODEV;
    }

  up_mdelay(10);  /* Wait for XCLK to stabilize */

  /* Step 3: Initialize GC0308 sensor via I2C */

  i2c = esp32s3_i2cbus_initialize(0);
  if (!i2c)
    {
      snerr("ERROR: Failed to initialize I2C bus\n");
      return -ENODEV;
    }

  imgsensor = gc0308_initialize(i2c, 320, 240);
  if (!imgsensor)
    {
      snerr("ERROR: Failed to initialize GC0308\n");
      return -ENODEV;
    }

  /* Step 4: Register imgdata and imgsensor globally.
   *   capture_initialize() in the camera app will create /dev/video.
   */

  imgdata_register(imgdata);
  ret = imgsensor_register(imgsensor);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register imgsensor: %d\n", ret);
      return ret;
    }

  sninfo("Camera drivers registered\n");
  return OK;
}
