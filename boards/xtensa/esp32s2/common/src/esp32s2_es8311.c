/****************************************************************************
 * boards/xtensa/esp32s2/common/src/esp32s2_es8311.c
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

#include <nuttx/irq.h>
#include <nuttx/audio/i2s.h>
#include <nuttx/audio/pcm.h>
#include <nuttx/audio/es8311.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>

#include <arch/board/board.h>

#include "esp32s2_i2c.h"
#include "esp32s2_i2s.h"

#if defined(CONFIG_ESP32S2_I2S) && defined(CONFIG_AUDIO_ES8311)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct es8311_lower_s g_es8311_lower[2];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s2_es8311_initialize
 *
 * Description:
 *   This function is called by platform-specific, setup logic to configure
 *   and register the ES8311 device.  This function will register the driver
 *   as /dev/audio/pcm[x] where x is determined by the I2S port number.
 *
 * Input Parameters:
 *   i2c_port  - The I2C port used for the device;
 *   i2c_addr  - The I2C address used by the device;
 *   i2c_freq  - The I2C frequency used for the device.
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int esp32s2_es8311_initialize(int i2c_port, uint8_t i2c_addr, int i2c_freq)
{
  struct audio_lowerhalf_s *es8311;
  struct i2s_dev_s *i2s;
  struct i2c_master_s *i2c;
  static bool initialized = false;
  int ret;

  audinfo("i2c_port %d, i2c_addr %d, i2c_freq %d\n",
          i2c_port, i2c_addr, i2c_freq);

  /* Have we already initialized? Since we never uninitialize we must
   * prevent multiple initializations. This is necessary, for example,
   * when the touchscreen example is used as a built-in application in
   * NSH and can be called numerous time. It will attempt to initialize
   * each time.
   */

  if (!initialized)
    {
      /* Get an instance of the I2S interface for the ES8311 data channel */

      i2s = esp32s2_i2sbus_initialize();
      if (i2s == NULL)
        {
          auderr("ERROR: Failed to initialize I2S\n");
          ret = -ENODEV;
          goto errout;
        }

      i2c = esp32s2_i2cbus_initialize(i2c_port);
      if (i2c == NULL)
        {
          auderr("ERROR: Failed to initialize I2C%d\n", i2c_port);
          ret = -ENODEV;
          goto errout;
        }

      /* Check wheter to enable a simple character driver that supports I2S
       * transfers via a read() and write().  The intent of this driver is to
       * support I2S testing.  It is not an audio driver but does conform to
       * some of the buffer management heuristics of an audio driver.  It is
       * not suitable for use in any real driver application in its current
       * form. The i2schar driver will be initialized at /dev/i2schar0
       */

#ifdef CONFIG_AUDIO_I2SCHAR
      ret = i2schar_register(i2s, 0);
      if (ret < 0)
        {
          auderr("ERROR: i2schar_register failed: %d\n", ret);
          goto errout;
        }
#endif

      /* Now we can use this I2S interface to initialize the ES8311 output
       * which will return an audio interface.
       */

      g_es8311_lower[0].address   = i2c_addr;
      g_es8311_lower[0].frequency = i2c_freq;

      es8311 = es8311_initialize(i2c, i2s, &g_es8311_lower[0]);
      if (es8311 == NULL)
        {
          auderr("ERROR: Failed to initialize the ES8311\n");
          ret = -ENODEV;
          goto errout;
        }

#ifdef CONFIG_SYSTEM_NXLOOPER
      /* If nxlooper is selected, the playback buffer is not rendered as
       * a WAV file. Therefore, PCM decode will fail while processing such
       * output buffer. In such a case, we bypass the PCM decode.
       */

      ret = audio_register("pcm0", es8311);
#else
      /* Now we can embed the ES8311/I2S conglomerate into a PCM decoder
       * instance so that we will have a PCM front end for the the ES8311
       * driver.
       */

      struct audio_lowerhalf_s *pcm = pcm_decode_initialize(es8311);
      if (pcm == NULL)
        {
          auderr("ERROR: Failed create the PCM decoder\n");
          ret = -ENODEV;
          goto errout;
        }

      /* Finally, we can register the PCM/ES8311/I2S audio device. */

      ret = audio_register("pcm0", pcm);
#endif
      if (ret < 0)
        {
          auderr("ERROR: Failed to register /dev/pcm0 device: %d\n", ret);
          goto errout;
        }

      /* Now we can use this I2S interface to initialize the ES8311 input
       * which will return an audio interface.
       */

      g_es8311_lower[1].address   = i2c_addr;
      g_es8311_lower[1].frequency = i2c_freq;

      es8311 = es8311_initialize(i2c, i2s, &g_es8311_lower[1]);
      if (es8311 == NULL)
        {
          auderr("ERROR: Failed to initialize the ES8311\n");
          ret = -ENODEV;
          goto errout;
        }

      /* Finally, we can register the PCM/ES8311/I2S audio device. */

      ret = audio_register("pcm_in0", es8311);
      if (ret < 0)
        {
          auderr("ERROR: Failed to register /dev/pcm_in0 device: %d\n", ret);
          goto errout;
        }

      /* Now we are initialized */

      initialized = true;
    }

  return OK;

errout:
  return ret;
}

#endif /* CONFIG_ESP32S2_I2S && CONFIG_AUDIO_ES8311 */
