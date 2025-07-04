/****************************************************************************
 * boards/xtensa/esp32s3/lckfb-szpi-esp32s3/src/esp32s3_ft5x06.c
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
#include <nuttx/input/ft5x06.h>
#include <nuttx/input/touchscreen.h>

#include "esp32s3_i2c.h"
#include "esp32s3-szpi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_FT5X06_POLLMODE
#error "Only support poll mode currently!"
#endif

#define ESP32S3_FT5X06_I2C_PORT (0)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct ft5x06_config_s g_ft5x06_config =
{
  .address   = FT5X06_I2C_ADDRESS,
  .frequency = FT5X06_FREQUENCY,
  .lower =
    {
      .xres = CONFIG_ARCH_BOARD_ESP32S3_LCKFB_SZPI_DISPLAY_XRES,
      .yres = CONFIG_ARCH_BOARD_ESP32S3_LCKFB_SZPI_DISPLAY_YRES,
      .flags = 0
#ifdef CONFIG_ARCH_BOARD_ESP32S3_LCKFB_SZPI_TOUCHSCREEN_SWAPXY
               | TOUCH_FLAG_SWAPXY
#endif
#ifdef CONFIG_ARCH_BOARD_ESP32S3_LCKFB_SZPI_TOUCHSCREEN_MIRRORY
               | TOUCH_FLAG_MIRRORY
#endif
               ,
    },
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int esp32s3_ft5x06_initialize(void)
{
  struct i2c_master_s *i2c;
  int ret;

  i2c = esp32s3_i2cbus_initialize(ESP32S3_FT5X06_I2C_PORT);
  if (!i2c)
    {
      i2cerr("Initialize I2C bus failed!\n");
      return -EINVAL;
    }

  ret = ft5x06_register(i2c, &g_ft5x06_config, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register FT5X06 driver: %d\n", ret);
    }

  return 0;
}
