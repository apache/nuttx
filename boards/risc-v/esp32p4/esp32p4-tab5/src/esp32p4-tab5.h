/****************************************************************************
 * boards/risc-v/esp32p4/esp32p4-tab5/src/esp32p4-tab5.h
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

#ifndef __BOARDS_RISCV_ESP32P4_ESP32P4_TAB5_SRC_ESP32P4_TAB5_H
#define __BOARDS_RISCV_ESP32P4_ESP32P4_TAB5_SRC_ESP32P4_TAB5_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdbool.h>

#ifdef CONFIG_ESPRESSIF_MIPI_DSI
#  include "espressif/esp_mipi_dsi.h"
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp_bringup
 *
 * Description:
 *   Perform architecture-specific initialization.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int esp_bringup(void);

#ifdef CONFIG_ESP32P4_TAB5_LCD_POWER

/****************************************************************************
 * Name: tab5_mipi_phy_power
 *
 * Description:
 *   Enable or disable LDO channel 3 (VDD_MIPI_DPHY @ 2500 mV).
 *
 * Input Parameters:
 *   on - True to enable the LDO channel, false to disable it.
 *
 * Returned Value:
 *   Zero on success, -1 on failure.
 *
 ****************************************************************************/

int tab5_mipi_phy_power(bool on);

/****************************************************************************
 * Name: tab5_lcd_enable
 *
 * Description:
 *   Drive PI4IOE P4 (BSP_LCD_EN).  true = enable (high).
 *
 * Input Parameters:
 *   enable - True to enable the LCD rail, false to disable it.
 *
 * Returned Value:
 *   Zero on success, -1 on failure.
 *
 ****************************************************************************/

int tab5_lcd_enable(bool enable);

/****************************************************************************
 * Name: tab5_lcd_backlight
 *
 * Description:
 *   Drive backlight enable GPIO22 (ME2212 boost EN).
 *
 * Input Parameters:
 *   on - True to enable the backlight, false to disable it.
 *
 * Returned Value:
 *   Zero on success, -1 on failure.
 *
 ****************************************************************************/

int tab5_lcd_backlight(bool on);

/****************************************************************************
 * Name: tab5_lcd_power_init
 *
 * Description:
 *   Power MIPI PHY LDO, init PI4IOE, and enable LCD (P4 high).
 *
 * Returned Value:
 *   Zero on success, -1 on failure.
 *
 ****************************************************************************/

int tab5_lcd_power_init(void);

#endif /* CONFIG_ESP32P4_TAB5_LCD_POWER */

#ifdef CONFIG_ESP32P4_TAB5_LCD

/****************************************************************************
 * Name: tab5_fb_reload_test_pattern
 *
 * Description:
 *   Re-fill the plane with the bring-up test color and cache write-back.
 *   Call after fb_register(): the generic FB driver memset()s the plane
 *   and would otherwise leave a black DMA buffer.
 *
 * Returned Value:
 *   Zero on success, -1 on failure.
 *
 ****************************************************************************/

int tab5_fb_reload_test_pattern(void);

/****************************************************************************
 * Name: tab5_mipi_dsi_dpi_config
 *
 * Description:
 *   Fill DPI timing for the selected Tab5 ST712x panel.  Arch
 *   host has no panel defaults — board code supplies this to
 *   esp_mipi_dsi_configure_dpi().
 *
 * Input Parameters:
 *   cfg - Pointer to the DPI configuration structure.
 *
 * Returned Value:
 *   Zero on success, -1 on failure.
 *
 ****************************************************************************/

void tab5_mipi_dsi_dpi_config(FAR struct esp_mipi_dsi_dpi_config_s *cfg);

#endif /* CONFIG_ESP32P4_TAB5_LCD */

#ifdef CONFIG_ESP32P4_TAB5_IOEXPANDER
/****************************************************************************
 * Name: tab5_pi4ioe_init
 *
 * Description:
 *   Initialize the IO expanders.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero on success, -1 on failure.
 *
 ****************************************************************************/

int tab5_pi4ioe_init(void);

/****************************************************************************
 * Name: tab5_pi4ioe_high_write_pin
 *
 * Description:
 *   Write a pin on the IO expander (high).
 *
 * Input Parameters:
 *   pin - The pin to write on the IO expander (high).
 *   enable - True to set the pin high, false to set the pin low.
 *
 * Returned Value:
 *   Zero on success, -1 on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32P4_TAB5_IOEXPANDER_HIGH
int tab5_pi4ioe_high_write_pin(uint8_t pin, bool enable);
#endif

/****************************************************************************
 * Name: tab5_pi4ioe_low_write_pin
 *
 * Description:
 *   Write a pin on the IO expander (low).
 *
 * Input Parameters:
 *   pin - The pin to write on the IO expander (low).
 *   enable - True to set the pin high, false to set the pin low.
 *
 * Returned Value:
 *   Zero on success, -1 on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32P4_TAB5_IOEXPANDER_LOW
int tab5_pi4ioe_low_write_pin(uint8_t pin, bool enable);
#endif
#endif /* CONFIG_ESP32P4_TAB5_IOEXPANDER */

#ifdef CONFIG_ESP32P4_TAB5_TOUCHSCREEN
/****************************************************************************
 * Name: tab5_touchscreen_power_init
 *
 * Description:
 *   Enable or disable the Touch Screen Controller rail.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero on success, -1 on failure.
 *
 ****************************************************************************/

int tab5_touchscreen_power_init(void);

/****************************************************************************
 * Name: tab5_touchscreen_init
 *
 * Description:
 *   Initialize the touch screen controller.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero on success, -1 on failure.
 *
 ****************************************************************************/

int tab5_touchscreen_init(void);
#endif /* CONFIG_ESP32P4_TAB5_TOUCHSCREEN */

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_RISCV_ESP32P4_ESP32P4_TAB5_SRC_ESP32P4_TAB5_H */
