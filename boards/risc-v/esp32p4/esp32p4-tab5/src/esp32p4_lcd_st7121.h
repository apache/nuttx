/****************************************************************************
 * boards/risc-v/esp32p4/esp32p4-tab5/src/esp32p4_lcd_st7121.h
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

#ifndef __BOARDS_RISCV_ESP32P4_ESP32P4_TAB5_SRC_ESP32P4_LCD_ST7121_H
#define __BOARDS_RISCV_ESP32P4_ESP32P4_TAB5_SRC_ESP32P4_LCD_ST7121_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/video/mipi_dsi.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: tab5_st7121_initialize
 *
 * Description:
 *   Register the ST7121 as a mipi_dsi_device, attach to the Espressif host,
 *   power-cycle LCD_EN (BSP_LCD_RST is NC), issue DCS soft-reset, and send
 *   the M5Stack Tab5 ST7121 vendor DCS initialization table.  Does not start
 *   video; board code calls display_on after video_start.
 *
 * Input Parameters:
 *   host - Registered MIPI-DSI host (esp_mipi_dsi_host_get())
 *
 * Returned Value:
 *   Pointer to the registered device on success; NULL on failure.
 *
 ****************************************************************************/
#ifdef CONFIG_ESP32P4_TAB5_LCD_ST7121
FAR struct mipi_dsi_device *tab5_st7121_initialize(
      FAR struct mipi_dsi_host *host);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_RISCV_ESP32P4_ESP32P4_TAB5_SRC_ESP32P4_LCD_ST7121_H */
