/****************************************************************************
 * boards/xtensa/esp32/common/src/esp32_backlight.c
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

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "esp32_gpio.h"
#include "esp32_backlight.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define HAVE_BACKLIGHT    1

#if !defined(DISPLAY_BCKL)
 #undef HAVE_BACKLIGHT
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_set_backlight
 *
 * Description:
 *    Configure the backlight gpio and set the brightness level.
 *
 * Input Parameters:
 *    level - select the brightness level
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int esp32_set_backlight(uint8_t level)
{
  #ifdef HAVE_BACKLIGHT
  esp32_configgpio(DISPLAY_BCKL, OUTPUT);

  /* TODO: use PWM to set the display brightness */

  if (level == 0)
    {
      esp32_gpiowrite(DISPLAY_BCKL, false);
    }
  else
    {
      /* Set full brightness */

      esp32_gpiowrite(DISPLAY_BCKL, true);
    }
  #endif

  return OK;
}
