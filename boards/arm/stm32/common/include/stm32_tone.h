/****************************************************************************
 * boards/arm/stm32/common/include/stm32_tone.h
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

#ifndef __STM32_TONE_H
#define __STM32_TONE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct board_tone_config_s
{
  int pwm_timer;                  /* PWM timer number for tone generation */
  int oneshot_timer;              /* Oneshot timer for note intervals */
  int oneshot_timer_resolution;   /* Oneshot timer resolution in us */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: board_tone_initialize
 *
 * Input Parameters:
 *   cfg   - Configuration for the driver
 *   devno - The device number, used to build the device path as /dev/toneN
 *
 * Description:
 *   Configure and initialize the tone generator.
 *
 ****************************************************************************/

int board_tone_initialize(FAR struct board_tone_config_s *cfg, int devno);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif // __STM32_TONE_H
