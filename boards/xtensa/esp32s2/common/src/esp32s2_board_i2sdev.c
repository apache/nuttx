/****************************************************************************
 * boards/xtensa/esp32s2/common/src/esp32s2_board_i2sdev.c
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

#include "esp32s2_i2s.h"

#if defined(CONFIG_ESP32S2_I2S) && !defined(CONFIG_AUDIO_CS4344) && \
    !defined(CONFIG_AUDIO_ES8311)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_i2sdev_initialize
 *
 * Description:
 *   This function is called by platform-specific, setup logic to configure
 *   and register the generic I2S audio driver.  This function will register
 *   the driver as /dev/audio/pcm[x] where x is determined by the I2S port
 *   number.
 *
 * Input Parameters:
 *   enable_tx  - Register device as TX if true
 *   enable_rx  - Register device as RX if true
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int board_i2sdev_initialize(bool enable_tx, bool enable_rx)
{
  struct audio_lowerhalf_s *audio_i2s;
  struct i2s_dev_s *i2s;
  char devname[8];
  int ret;

  ainfo("Initializing I2S\n");

  i2s = esp32s2_i2sbus_initialize();

#ifdef CONFIG_AUDIO_I2SCHAR
  ret = i2schar_register(i2s, 0);
  if (ret < 0)
    {
      aerr("ERROR: i2schar_register failed: %d\n", ret);
      return ret;
    }
#endif

  if (enable_tx)
    {
      /* Initialize audio output */

      audio_i2s = audio_i2s_initialize(i2s, true);

      if (audio_i2s == NULL)
        {
          auderr("ERROR: Failed to initialize I2S audio output\n");
          return -ENODEV;
        }

      snprintf(devname, sizeof(devname), "pcm0");

      /* If nxlooper is selected, the playback buffer is not rendered as
       * a WAV file. Therefore, PCM decode will fail while processing such
       * output buffer. In such a case, we bypass the PCM decode.
       */

#ifdef CONFIG_SYSTEM_NXLOOPER
      ret = audio_register(devname, audio_i2s);
#else
      struct audio_lowerhalf_s *pcm;

      pcm = pcm_decode_initialize(audio_i2s);

      if (pcm == NULL)
        {
          auderr("ERROR: Failed create the PCM decoder\n");
          return -ENODEV;
        }

      ret = audio_register(devname, pcm);
#endif /* CONFIG_SYSTEM_NXLOOPER */

      if (ret < 0)
        {
          auderr("ERROR: Failed to register /dev/%s device: %d\n",
                 devname, ret);
          return ret;
        }
    }

  if (enable_rx)
    {
      /* Initialize audio input */

      audio_i2s = audio_i2s_initialize(i2s, false);

      if (audio_i2s == NULL)
        {
          auderr("ERROR: Failed to initialize I2S audio input\n");
          return -ENODEV;
        }

      snprintf(devname, sizeof(devname), "pcm_in0");

      ret = audio_register(devname, audio_i2s);

      if (ret < 0)
        {
          auderr("ERROR: Failed to register /dev/%s device: %d\n",
                 devname, ret);
          return ret;
        }
    }

  return ret;
}

#endif /* (CONFIG_ESP32S2_I2S) && !(CONFIG_AUDIO_CS4344)  *
        * !(CONFIG_AUDIO_ES8311)                          */
