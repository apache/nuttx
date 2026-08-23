/****************************************************************************
 * boards/xtensa/esp32s3/esp32s3-touch-lcd7/src/esp32s3_lcd.c
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

/* The 7" 800x480 panel is driven by the LCD_CAM peripheral over a 16-bit
 * parallel RGB565 bus, which arch/xtensa/src/esp32s3/esp32s3_lcd.c owns.
 * Everything that is specific to this board is here: the panel reset and
 * the display enable are not wired to GPIOs but to the CH422G I/O expander,
 * because the RGB bus has taken most of the usable pins.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <syslog.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/ioexpander/ioexpander.h>
#include <nuttx/ioexpander/ch422g.h>
#include <nuttx/video/fb.h>

#include "esp32s3-touch-lcd7.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The EK9716 wants its reset held for at least 10us and needs time to come
 * up before it is clocked.  These are comfortably longer than that.
 */

#define LCD_RESET_HOLD_US   (20 * 1000)
#define LCD_RESET_WAIT_US  (120 * 1000)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_lcd_initialize
 *
 * Description:
 *   Bring the RGB panel out of reset, switch the backlight on and register
 *   the framebuffer character driver.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_lcd_initialize(void)
{
  FAR struct ioexpander_dev_s *ioe;
  int ret;

  ioe = board_ioexpander_initialize();
  if (ioe == NULL)
    {
      return -ENODEV;
    }

  /* The panel reset and the display enable are outputs of the expander.
   * Drive the panel into reset and keep the display off while it settles.
   */

  ret = IOEXP_SETDIRECTION(ioe, BOARD_EXIO_LCD_RST,
                           IOEXPANDER_DIRECTION_OUT);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to configure LCD reset: %d\n", ret);
      return ret;
    }

  ret = IOEXP_SETDIRECTION(ioe, BOARD_EXIO_DISP, IOEXPANDER_DIRECTION_OUT);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to configure display enable: %d\n",
             ret);
      return ret;
    }

  IOEXP_WRITEPIN(ioe, BOARD_EXIO_DISP, false);
  IOEXP_WRITEPIN(ioe, BOARD_EXIO_LCD_RST, false);
  up_udelay(LCD_RESET_HOLD_US);

  ret = IOEXP_WRITEPIN(ioe, BOARD_EXIO_LCD_RST, true);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to release LCD reset: %d\n", ret);
      return ret;
    }

  up_udelay(LCD_RESET_WAIT_US);

  /* The framebuffer has to exist, and so be scanning out a defined buffer,
   * before the display is enabled.  Enabling it first shows whatever the
   * panel happens to latch until the first frame arrives.
   */

  ret = fb_register(0, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: fb_register() failed: %d\n", ret);
      return ret;
    }

  ret = IOEXP_WRITEPIN(ioe, BOARD_EXIO_DISP, true);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to enable the display: %d\n", ret);
      return ret;
    }

  return OK;
}
