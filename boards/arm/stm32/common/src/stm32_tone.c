/****************************************************************************
 * boards/arm/stm32/common/src/stm32_tone.c
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
#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>
#include <stdio.h>

#include <nuttx/timers/pwm.h>
#include <nuttx/timers/oneshot.h>
#include <nuttx/audio/tone.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "stm32_pwm.h"
#include "stm32_tone.h"

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
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_tone_initialize
 *
 * Input Parameters:
 *   devno - The device number, used to build the device path as /dev/toneN
 *
 * Description:
 *   Configure and initialize the tone generator.
 *
 ****************************************************************************/

int board_tone_initialize(int devno)
{
  static bool                initialized = false;
  struct pwm_lowerhalf_s     *tone;
  struct oneshot_lowerhalf_s *oneshot = NULL;
  int                        ret;
  char                       devpath[12];

  /* Have we already initialized? */

  if (!initialized)
    {
      /* Call stm32_pwminitialize() to get an instance of the PWM interface */

      tone = stm32_pwminitialize(BOARD_TONE_PWM_TIM);
      if (!tone)
        {
          auderr("Failed to get the STM32 PWM lower half to AUDIO TONE\n");
          return -ENODEV;
        }

      /* Initialize TONE PWM */

      tone->ops->setup(tone);

      /* Initialize ONESHOT Timer */

      oneshot = oneshot_initialize(BOARD_TONE_ONESHOT_TIM,
                                   BOARD_TONE_ONESHOT_TIM_RES);
      if (!oneshot)
        {
          auderr("Failed to initialize ONESHOT Timer!\n");
          return -ENODEV;
        }

      /* Register the Audio Tone driver at "/dev/tone0" */

      snprintf(devpath, 12, "/dev/tone%d", devno);
      ret = tone_register(devpath, tone, oneshot);
      if (ret < 0)
        {
          auderr("ERROR: tone_register failed: %d\n", ret);
          return ret;
        }

      /* Now we are initialized */

      initialized = true;
    }

  return OK;
}
