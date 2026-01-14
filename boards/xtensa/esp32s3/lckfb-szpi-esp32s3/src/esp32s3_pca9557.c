/****************************************************************************
 * boards/xtensa/esp32s3/lckfb-szpi-esp32s3/src/esp32s3_pca9557.c
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
#include <nuttx/ioexpander/pca9557.h>
#include <nuttx/ioexpander/gpio.h>

#include "esp32s3_i2c.h"
#include "esp32s3-szpi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* | PCA9557 | ESP32S3 |
 * |---------|---------|
 * | SCL     | IO2     |
 * | SDA     | IO1     |
 * | A0      | 3V3     |
 * | A1      | GND     |
 * | A2      | GND     |
 */

#define ESP32S3_PCA9557_I2C_PORT       (0)
#define ESP32S3_PCA9557_I2C_ADDR       (0x19)
#define ESP32S3_PCA9557_I2C_CLOCK      (100 * 1000)

/* | PCA9557 | Peripherals |
 * |---------|-------------|
 * | IO0     | LCD_CS      |
 * | IO1     | PA_EN       |
 * | IO2     | DVP_PWDW    |
 */

#define ESP32S3_PCA9557_LCD_CS         (0)
#define ESP32S3_PCA9557_PA_EN          (1)
#define ESP32S3_PCA9557_DVP_PWDN       (2)

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct pca9557_config_s g_pca9557_config =
{
  ESP32S3_PCA9557_I2C_ADDR,
  ESP32S3_PCA9557_I2C_CLOCK,
  NULL,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int esp32s3_pca9557_initialize(void)
{
  struct ioexpander_dev_s *ioe;
  struct i2c_master_s *i2c;
  int ret;

  i2c = esp32s3_i2cbus_initialize(ESP32S3_PCA9557_I2C_PORT);
  if (!i2c)
    {
      i2cerr("Initialize I2C bus failed!\n");
      return -EINVAL;
    }

  ioe = pca9557_initialize(i2c, &g_pca9557_config);
  if (!ioe)
    {
      i2cerr("Initialize PCA9557 failed!\n");
      return -EINVAL;
    }

  ret = gpio_lower_half(ioe, ESP32S3_PCA9557_LCD_CS, GPIO_OUTPUT_PIN, 0);
  if (ret)
    {
      gpioerr("Create GPIO LCD_CS pin device failed!\n");
    }

  ret = gpio_lower_half(ioe, ESP32S3_PCA9557_PA_EN, GPIO_OUTPUT_PIN, 1);
  if (ret)
    {
      gpioerr("Create GPIO PA_EN pin device failed!\n");
    }

  ret = gpio_lower_half(ioe, ESP32S3_PCA9557_DVP_PWDN, GPIO_OUTPUT_PIN, 2);
  if (ret)
    {
      gpioerr("Create GPIO DVP_PWDW pin device failed!\n");
    }

  return 0;
}
