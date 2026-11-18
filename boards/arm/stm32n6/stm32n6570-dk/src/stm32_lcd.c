/****************************************************************************
 * boards/arm/stm32n6/stm32n6570-dk/src/stm32_lcd.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/signal.h>
#include <nuttx/video/fb.h>

#include "stm32_gpio.h"
#include "stm32_ltdc.h"
#include "stm32n6570-dk.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LCD_NRST low time, as for the GT911 that shares the line */

#define LCD_RESET_DELAY 1000

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_fbinitialize
 *
 * Description:
 *   Power the RK050HR18 panel and initialize the LTDC.
 *
 ****************************************************************************/

int up_fbinitialize(int display)
{
  int ret;

#ifndef CONFIG_INPUT_GT9XX
  /* Reset the panel.  With the touch controller, its set-up did it. */

  stm32_configgpio(GPIO_LCD_NRST);
  nxsig_usleep(LCD_RESET_DELAY);
  stm32_gpiowrite(GPIO_LCD_NRST, true);
#endif

  /* Turn the display on */

  stm32_configgpio(GPIO_LCD_DISP);
  stm32_configgpio(GPIO_LCD_BL);
  stm32_gpiowrite(GPIO_LCD_DISP, true);

  ret = stm32_ltdcinitialize();
  if (ret < 0)
    {
      return ret;
    }

  stm32_gpiowrite(GPIO_LCD_BL, true);
  return OK;
}

/****************************************************************************
 * Name: up_fbgetvplane
 *
 * Description:
 *   Return a reference to the framebuffer object for the specified video
 *   plane.
 *
 ****************************************************************************/

struct fb_vtable_s *up_fbgetvplane(int display, int vplane)
{
  return stm32_ltdcgetvplane(vplane);
}

/****************************************************************************
 * Name: up_fbuninitialize
 *
 * Description:
 *   Turn the panel off and uninitialize the LTDC.
 *
 ****************************************************************************/

void up_fbuninitialize(int display)
{
  stm32_gpiowrite(GPIO_LCD_BL, false);
  stm32_gpiowrite(GPIO_LCD_DISP, false);

  stm32_ltdcuninitialize();
}
