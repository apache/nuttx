/****************************************************************************
 * boards/xtensa/esp32s2/common/src/esp32s2_cs4344.c
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
#include <nuttx/audio/cs4344.h>

#include <arch/board/board.h>

#include "esp32s2_i2s.h"

#if defined(CONFIG_ESP32S2_I2S) && defined(CONFIG_AUDIO_CS4344)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s2_cs4344_initialize
 *
 * Description:
 *   This function is called by platform-specific, setup logic to configure
 *   and register the CS4344 device.  This function will register the driver
 *   as /dev/audio/pcm[x] where x is determined by the I2S port number.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int esp32s2_cs4344_initialize(void)
{
  struct audio_lowerhalf_s *cs4344;
  struct audio_lowerhalf_s *pcm;
  struct i2s_dev_s *i2s;
  static bool initialized = false;
  char devname[5];
  int ret;

  /* Have we already initialized?  Since we are already initialized we must
   * prevent multiple initializations.
   */

  if (!initialized)
    {
      /* Get an instance of the I2S interface for the CS4344 data channel */

      i2s = esp32s2_i2sbus_initialize();
      if (!i2s)
        {
          auderr("ERROR: Failed to initialize I2S0\n");
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

      /* Now we can use this I2S interface to initialize the CS4344 which
       * will return an audio interface.
       */

      cs4344 = cs4344_initialize(i2s);
      if (!cs4344)
        {
          auderr("ERROR: Failed to initialize the CS4344\n");
          ret = -ENODEV;
          goto errout;
        }

      /* No we can embed the CS4344/I2S conglomerate into a PCM decoder
       * instance so that we will have a PCM front end for the the CS4344
       * driver.
       */

      pcm = pcm_decode_initialize(cs4344);
      if (!pcm)
        {
          auderr("ERROR: Failed create the PCM decoder\n");
          ret = -ENODEV;
          goto errout;
        }

      /* Create a device name */

      snprintf(devname, 12, "pcm0");

      /* Finally, we can register the PCM/CS4344/I2S audio device.
       *
       * Is anyone young enough to remember Rube Goldberg?
       */

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

#endif /* CONFIG_ESP32S2_I2S && CONFIG_AUDIO_CS4344 */
