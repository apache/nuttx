/****************************************************************************
 * boards/xtensa/esp32/common/src/esp32_es8388.c
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
#include <nuttx/audio/es8388.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>

#include <arch/board/board.h>

#include "esp32_board_i2c.h"
#include "esp32_i2c.h"
#include "esp32_i2s.h"

#if defined(CONFIG_ESP32_I2S) && defined(CONFIG_AUDIO_ES8388)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_es8388_initialize
 *
 * Description:
 *   This function is called by platform-specific, setup logic to configure
 *   and register the ES8388 device.  This function will register the driver
 *   as /dev/audio/pcm[x] where x is determined by the I2S port number.
 *
 * Input Parameters:
 *   i2c_port  - The I2C port used for the device
 *   i2c_addr  - The I2C address used by the device
 *   i2c_freq  - The I2C frequency used for the device
 *   i2s_port  - The I2S port used for the device
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int esp32_es8388_initialize(int i2c_port, uint8_t i2c_addr, int i2c_freq,
                            int i2s_port)
{
  struct audio_lowerhalf_s *es8388;
  struct audio_lowerhalf_s *pcm;
  struct es8388_lower_s *lower;
  struct i2s_dev_s *i2s;
  struct i2c_master_s *i2c;
  static bool initialized = false;
  char devname[12];
  int ret;

  audinfo("i2s_port %d\ni2c_port %d, i2c_addr %d, i2c_freq %d\n",
          i2s_port, i2c_port, i2c_addr, i2c_freq);
  DEBUGASSERT(i2s_port >= 0 && i2s_port <= 1 && \
              i2c_port >= 0 && i2c_port <= 1);

  /* Have we already initialized?  Since we never uninitialize we must
   * prevent multiple initializations.  This is necessary, for example,
   * when the touchscreen example is used as a built-in application in
   * NSH and can be called numerous time.  It will attempt to initialize
   * each time.
   */

  if (!initialized)
    {
      /* Get an instance of the I2S interface for the ES8388 data channel */

      i2s = esp32_i2sbus_initialize(i2s_port);
      if (i2s == NULL)
        {
          auderr("ERROR: Failed to initialize I2S%d\n", i2s_port);
          ret = -ENODEV;
          goto errout;
        }

      i2c = esp32_i2cbus_initialize(i2c_port);
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

      /* Now we can use this I2S interface to initialize the ES8388 which
       * will return an audio interface.
       */

      lower = (FAR struct es8388_lower_s *)
        kmm_zalloc(sizeof(struct es8388_lower_s));

      lower->address   = i2c_addr,
      lower->frequency = i2c_freq,

      es8388 = es8388_initialize(i2c, i2s, lower);
      if (es8388 == NULL)
        {
          auderr("ERROR: Failed to initialize the ES8388\n");
          ret = -ENODEV;
          goto errout;
        }

      /* Now we can embed the ES8388/I2S conglomerate into a PCM decoder
       * instance so that we will have a PCM front end for the the ES8388
       * driver.
       */

      pcm = pcm_decode_initialize(es8388);
      if (pcm == NULL)
        {
          auderr("ERROR: Failed create the PCM decoder\n");
          ret = -ENODEV;
          goto errout;
        }

      /* Create a device name */

      snprintf(devname, 12, "pcm%d",  i2s_port);

      /* Finally, we can register the PCM/ES8388/I2S audio device. */

      ret = audio_register(devname, pcm);
      if (ret < 0)
        {
          auderr("ERROR: Failed to register /dev/%s device: %d\n",
                 devname, ret);
          goto errout;
        }

      /* Now we are initialized */

      initialized = true;
    }

  return OK;

errout:
  return ret;
}

#endif /* CONFIG_ESP32_I2S && CONFIG_AUDIO_ES8388 */
