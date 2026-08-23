/****************************************************************************
 * boards/xtensa/esp32s3/esp32s3-touch-lcd7/src/esp32s3-touch-lcd7.h
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

#ifndef __BOARDS_XTENSA_ESP32S3_ESP32S3_TOUCH_LCD7_SRC_ESP32S3_TOUCH_LCD7_H
#define __BOARDS_XTENSA_ESP32S3_ESP32S3_TOUCH_LCD7_SRC_ESP32S3_TOUCH_LCD7_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#ifdef CONFIG_IOEXPANDER_CH422G
#  include <nuttx/ioexpander/ioexpander.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2C bus ******************************************************************/

/* One I2C bus, GPIO8 (SDA) and GPIO9 (SCL), is shared by the CH422G I/O
 * expander, the GT911 touch controller and the external I2C header.
 */

#define BOARD_I2C_BUS      0

/* CH422G I/O expander ******************************************************/

/* The board labels the expander pins EXIO0-EXIO7, and they are the CH422G
 * pins IO0-IO7 in order, so the ioexpander pin number is the EXIO number.
 * EXIO0, EXIO6 and EXIO7 are not connected to anything on this board.
 */

#define BOARD_EXIO_TP_RST  CH422G_IO1  /* GT911 touch controller reset */
#define BOARD_EXIO_DISP    CH422G_IO2  /* LCD display and backlight enable */
#define BOARD_EXIO_LCD_RST CH422G_IO3  /* LCD panel reset */
#define BOARD_EXIO_SD_CS   CH422G_IO4  /* TF card chip select */
#define BOARD_EXIO_USB_SEL CH422G_IO5  /* USB / CAN transceiver select */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 ****************************************************************************/

int esp32s3_bringup(void);

#ifdef CONFIG_IOEXPANDER_CH422G
/****************************************************************************
 * Name: board_ioexpander_initialize
 *
 * Description:
 *   Bring up the CH422G I/O expander, if it has not been brought up
 *   already, and return it.  Several parts of the board hang off the
 *   expander, so the first caller initialises it and the rest share it.
 *
 * Returned Value:
 *   The expander instance on success, NULL on failure.
 *
 ****************************************************************************/

FAR struct ioexpander_dev_s *board_ioexpander_initialize(void);
#endif

#ifdef CONFIG_ESP32S3_BOARD_LCD
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

int board_lcd_initialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_XTENSA_ESP32S3_ESP32S3_TOUCH_LCD7_SRC_ESP32S3_TOUCH_LCD7_H */
