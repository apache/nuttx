/****************************************************************************
 * boards/arm/stm32f7/common/src/stm32_cs4344.c
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

#include "stm32_i2s.h"
#include "stm32_pwr.h"
#include "stm32_rcc.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_cs4344_initialize
 *
 * Description:
 *   This function is called by platform-specific, setup logic to configure
 *   and register the CS4344 device.  This function will register the driver
 *   as /dev/audio/pcm[x] where x is determined by the minor device number.
 *
 * Input Parameters:
 *   minor - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int board_cs4344_initialize(int devno, int port)
{
  struct audio_lowerhalf_s *cs4344;
  struct audio_lowerhalf_s *pcm;
  struct i2s_dev_s *i2s;
  static bool initialized = false;
  char devname[12];
  int ret;

  audinfo("minor %d\n", devno);
  DEBUGASSERT(devno >= 0 && devno <= 25);

  /* Have we already initialized?  Since we never uninitialize we must
   * prevent multiple initializations.  This is necessary, for example,
   * when the touchscreen example is used as a built-in application in
   * NSH and can be called numerous time.  It will attempt to initialize
   * each time.
   */

  if (!initialized)
    {
      /* Get an instance of the I2S interface for the CS4344 data channel */

      i2s = stm32_i2sbus_initialize(port);
      if (!i2s)
        {
          auderr("ERROR: Failed to initialize I2S%d\n", port);
          ret = -ENODEV;
          goto errout;
        }

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

      snprintf(devname, 12, "pcm%d",  devno);

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

