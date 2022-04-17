/****************************************************************************
 * boards/arm/sama5/sama5d4-ek/src/sam_audio_null.c
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

#include <nuttx/audio/audio.h>
#include <nuttx/audio/pcm.h>
#include <nuttx/audio/audio_null.h>

#include "sama5d4-ek.h"

#ifdef HAVE_AUDIO_NULL

/****************************************************************************
 * Pre-processor Definitions
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
 * Name: sam_audio_null_initialize
 *
 * Description:
 *   Set up to use the NULL audio device for PCM unit-level testing.
 *
 * Input Parameters:
 *   minor - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int sam_audio_null_initialize(int minor)
{
  struct audio_lowerhalf_s *nullaudio;
  struct audio_lowerhalf_s *pcm;
  static bool initialized = false;
  char devname[12];
  int ret;

  audinfo("minor %d\n", minor);
  DEBUGASSERT(minor >= 0 && minor <= 25);

  /* Have we already initialized?
   * Since we never uninitialize we must prevent multiple initializations.
   * This is necessary, for example, when thetouchscreen example is used as
   * a built-in application in NSH and can be called numerous times.
   * It will attempt to initialize each time.
   */

  if (!initialized)
    {
      /* Get a null audio interface
       */

      nullaudio = audio_null_initialize();
      if (!nullaudio)
        {
          auderr("ERROR: Failed to get the NULL audio interface\n");
          ret = -ENODEV;
          goto errout;
        }

      /* No we can embed the null audio interface into a PCM decoder
       * instance so that we will have a PCM front end for the NULL
       * audio driver.
       */

      pcm = pcm_decode_initialize(nullaudio);
      if (!pcm)
        {
          auderr("ERROR: Failed create the PCM decoder\n");
          ret = -ENODEV;
          goto errout_with_nullaudio;
        }

      /* Create a device name */

      snprintf(devname, 12, "pcm%d",  minor);

      /* Finally, we can register the PCM/NULL audio device. */

      ret = audio_register(devname, pcm);
      if (ret < 0)
        {
          auderr("ERROR: Failed to register /dev/%s device: %d\n",
                 devname, ret);
          goto errout_with_pcm;
        }

      /* Now we are initialized */

      initialized = true;
    }

  return OK;

  /* Error exits.  Unfortunately there is no mechanism in place now to
   * recover resources from most errors on initialization failures.
   */

errout_with_nullaudio:
errout_with_pcm:
errout:
  return ret;
}

#endif /* HAVE_AUDIO_NULL */
