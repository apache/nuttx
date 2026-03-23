/****************************************************************************
 * boards/xtensa/esp32s3/lckfb-szpi-esp32s3/src/esp32s3_board_es7210.c
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
#include <stdio.h>
#include <debug.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/audio/i2s.h>
#include <nuttx/audio/es7210.h>
#include <nuttx/audio/audio.h>

#include <arch/board/board.h>

#include "esp32s3_i2c.h"
#include "espressif/esp_i2s.h"
#include "esp32s3-szpi.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_es7210_initialize
 *
 * Description:
 *   This function is called by board-specific setup logic to configure
 *   and register the ES7210 audio ADC device.
 *
 * Input Parameters:
 *   i2c_port - The I2C port number for the ES7210 control interface
 *   i2s_port - The I2S port number for the ES7210 audio data interface
 *
 * Returned Value:
 *   Zero is returned on success. Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int esp32s3_es7210_initialize(int i2c_port, int i2s_port)
{
  FAR struct i2c_master_s *i2c;
  FAR struct i2s_dev_s *i2s;
  FAR struct audio_lowerhalf_s *es7210;
  struct es7210_lower_s lower;
  static bool initialized = false;
  char devname[] = "pcm_in0";
  int ret;

  if (initialized)
    {
      return OK;
    }

  /* Get I2C bus instance */

  i2c = esp32s3_i2cbus_initialize(i2c_port);
  if (i2c == NULL)
    {
      auderr("ERROR: Failed to initialize I2C%d\n", i2c_port);
      return -ENODEV;
    }

  /* Get I2S bus instance (RX only) */

  i2s = esp_i2sbus_initialize(i2s_port);
  if (i2s == NULL)
    {
      auderr("ERROR: Failed to initialize I2S%d\n", i2s_port);
      return -ENODEV;
    }

  /* Configure ES7210 lower half */

  lower.frequency = ES7210_I2C_FREQ;
  lower.address   = ES7210_I2C_ADDR;

  /* Initialize the ES7210 codec */

  es7210 = es7210_initialize(i2c, i2s, &lower);
  if (es7210 == NULL)
    {
      auderr("ERROR: Failed to initialize ES7210\n");
      return -ENODEV;
    }

  /* Register the audio device as /dev/audio/pcm_in0 */

  ret = audio_register(devname, es7210);
  if (ret < 0)
    {
      auderr("ERROR: Failed to register %s: %d\n", devname, ret);
      return ret;
    }

  initialized = true;
  audinfo("ES7210 registered as %s\n", devname);
  return OK;
}
