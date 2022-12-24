/****************************************************************************
 * boards/arm64/a64/pinephone/src/pinephone_lcd.h
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

#ifndef __BOARDS_ARM64_A64_PINEPHONE_SRC_PINEPHONE_LCD_H
#define __BOARDS_ARM64_A64_PINEPHONE_SRC_PINEPHONE_LCD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Xingbangda XBD599 LCD Panel Width and Height (pixels) */

#define PINEPHONE_LCD_PANEL_WIDTH  720
#define PINEPHONE_LCD_PANEL_HEIGHT 1440

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: pinephone_lcd_backlight_enable
 *
 * Description:
 *   Turn on the Backlight in Xingbangda XBD599 LCD Panel.
 *
 * Input Parameters:
 *   percent - Percent Brightness
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

int pinephone_lcd_backlight_enable(uint32_t percent);

/****************************************************************************
 * Name: pinephone_lcd_panel_reset
 *
 * Description:
 *   Reset the Xingbangda XBD599 LCD Panel.
 *
 * Input Parameters:
 *   val - True if Reset should be set to High; False if Reset should be
 *         set to Low
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

int pinephone_lcd_panel_reset(bool val);

/****************************************************************************
 * Name: pinephone_lcd_panel_init
 *
 * Description:
 *   Initialize the Sitronix ST7703 LCD Controller in Xingbangda XBD599
 *   LCD Panel.  Send 20 Initialization Commands to ST7703 over MIPI DSI.
 *   Assumes that the LCD Panel has been powered on (via AXP803 PMIC),
 *   MIPI DSI and D-PHY have been enabled on the SoC, and LCD Panel has
 *   been Reset (to Low then High).  After the LCD Panel has been
 *   initialized, we may start MIPI DSI (in HSC and HSD modes) and
 *   Display Engine.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

int pinephone_lcd_panel_init(void);

#endif /* __BOARDS_ARM64_A64_PINEPHONE_SRC_PINEPHONE_LCD_H */
