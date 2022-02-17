/****************************************************************************
 * boards/xtensa/esp32/common/include/esp32_lcd_backpack.h
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

#ifndef __BOARDS_XTENSA_ESP32_COMMON_INCLUDE_ESP32_LCD_BACKPACK_H
#define __BOARDS_XTENSA_ESP32_COMMON_INCLUDE_ESP32_LCD_BACKPACK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

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
 * Name: board_lcd_backpack_init
 *
 * Description:
 *   Initialize the LCD1602 display controlled by Backpack with PCF8574
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/slcd0"
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_lcd_backpack_init(int devno, int busno, int rows, int cols);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __BOARDS_XTENSA_ESP32_COMMON_INCLUDE_ESP32_LCD_BACKPACK_H */
