/****************************************************************************
 * boards/arm/am335x/beaglebone-black/src/am335x_lcd.c
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

#include <stdint.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/lcd/tda19988.h>
#include <nuttx/video/fb.h>
#include <nuttx/video/edid.h>
#include <nuttx/video/videomode.h>

#include "am335x_lcdc.h"
#include "beaglebone-black.h"

#ifdef HAVE_LCD

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef HAVE_TDA19988
static int am335x_attach(const struct tda19988_lower_s *lower,
                         xcpt_t handler, void *arg);
static int am335x_enable(const struct tda19988_lower_s *lower, bool enable);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef HAVE_TDA19988
static const strurct tda19988_lower_s g_lower;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am335x_attach
 *
 * Description:
 *   Attach or detach the TDA19988 interrupt handler
 *
 ****************************************************************************/

#ifdef HAVE_TDA19988
static int am335x_attach(const struct tda19988_lower_s *lower,
                         xcpt_t handler, void *arg)
{
#warning Missing logic
  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: am335x_enable
 *
 * Description:
 *   Enable or disable the TDA19988 interrupt
 *
 ****************************************************************************/

#ifdef HAVE_TDA19988
static int am335x_enable(const struct tda19988_lower_s *lower, bool enable)
{
#warning Missing logic
  return -ENOSYS;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_fbinitialize
 *
 * Description:
 *   Initialize the LCD.  If support for the TDA19988 HDMI controller is
 *   enabled, then this involves:
 *
 *   1. Initializing the TDA19988 HDMI controller driver
 *   2. Reading EDID data from the connected monitor
 *   3. Selecting a compatible video mode from the EDID data
 *   4. Initializing the LCD controller using this video mode
 *   5. Initializing the HDMI controller using the video mode
 *
 *   Otherwise, a default video mode is used to initialize the LCD
 *   controller.
 *
 ****************************************************************************/

int up_fbinitialize(int display)
{
  const struct videomode_s *videomode;
  struct am335x_panel_info_s panel;
  int ret;

#ifdef HAVE_TDA19988
  TDA19988_HANDLE handle;

  /* Initialize the TDA19988 GPIO interrupt input
   * Initialize the TDA19988 lower half state instance
   * Initialize the TDA19988 HDMI controller driver
   * Allocate a buffer to hold the EDID data
   * Read raw EDID data from the connected monitor
   * Select a compatible video mode from the EDID data
   * Free the allocated EDID buffer
   */

#warning Missing logic
#else
  /* Lookup the video mode corresponding to the default video mode */

  videomode = videomode_lookup_by_name(CONFIG_BEAGLEBONE_VIDEOMODE);
  if (videomode == NULL)
    {
      lcderr("ERROR: Videomode \"%s\" is not supported.\n",
             CONFIG_BEAGLEBONE_VIDEOMODE);
      return -ENOTSUP;
    }

#endif
  /* Convert the selected video mode to a AM335X LCD panel configuration */

  am335x_lcd_videomode(videomode, &panel);

  /* Initialize the LCD controller using this panel configuration */

  ret = am335x_lcd_initialize(&panel);
  if (ret < 0)
    {
      lcderr("ERROR: am335x_lcd_initialize() failed: %d\n", ret);
      return ret;
    }

#ifdef HAVE_TDA19988
  /* Initialize the HDMI controller using the selected video mode */

  ret = tda19988_videomode(handle, videomode);
  if (ret < 0)
    {
      lcderr("ERROR: tda19988_videomode() failed: %d\n", ret);
      return ret;
    }
#endif

  return OK;
}

#endif /* HAVE_LCD */
