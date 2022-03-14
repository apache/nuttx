/****************************************************************************
 * boards/arm/rp2040/common/src/rp2040_i2sdev.c
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

#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/audio/audio.h>
#include <nuttx/audio/audio_i2s.h>
#include <nuttx/audio/i2s.h>
#include <nuttx/audio/pcm.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "rp2040_i2s.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_i2sdev_initialize
 *
 * Description:
 *   Initialize i2s driver and register the /dev/audio/pcm0 device.
 *
 ****************************************************************************/

int board_i2sdev_initialize(int port)
{
  FAR struct audio_lowerhalf_s *audio_i2s;
  FAR struct audio_lowerhalf_s *pcm;
  FAR struct i2s_dev_s *i2s;
  char devname[12];
  int ret;

  ainfo("Initializing I2S\n");

  i2s = rp2040_i2sbus_initialize(port);

#ifdef CONFIG_AUDIO_I2SCHAR
  i2schar_register(i2s, 0);
#endif

  audio_i2s = audio_i2s_initialize(i2s, true);

  if (!audio_i2s)
    {
      auderr("ERROR: Failed to initialize I2S\n");
      return -ENODEV;
    }

  pcm = pcm_decode_initialize(audio_i2s);

  if (!pcm)
    {
      auderr("ERROR: Failed create the PCM decoder\n");
      return  -ENODEV;
    }

  snprintf(devname, 12, "pcm%d", port);

  ret = audio_register(devname, pcm);

  if (ret < 0)
    {
      auderr("ERROR: Failed to register /dev/%s device: %d\n", devname, ret);
    }

  return 0;
}
