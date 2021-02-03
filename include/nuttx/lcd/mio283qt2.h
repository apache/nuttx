/****************************************************************************
 * include/nuttx/lcd/mio283qt2.h
 *
 * Interface definition for the
 * MI0283QT-2 LCD from Multi-Inno Technology Co., Ltd.
 * This LCD is based on the Himax HX8347-D LCD controller.
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

#ifndef __INCLUDE_NUTTX_LCD_MIO283QT2_H
#define __INCLUDE_NUTTX_LCD_MIO283QT2_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_LCD_MIO283QT2

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* CONFIG_LCD_MIO283QT2 - Enables support for the MIO283QT2-based LCD.
 * CONFIG_LCD_NOGETRUN
 *   NX components need to know if it can read from the LCD or not.
 *   If reading from the LCD is supported then some graphic operations
 *   can be improved. Default: Supported
 * CONFIG_LCD_LANDSCAPE - Define for 320x240 display "landscape" support.
 *   Default is this 320x240 "landscape" orientation.
 * CONFIG_LCD_RLANDSCAPE - Define for 320x240 display "reverse landscape"
 *   support.
 *   Default is this 320x240 "landscape" orientation
 * CONFIG_LCD_PORTRAIT - Define for 240x320 display "portrait" orientation
 *   support.
 *   Default is this 320x240 "landscape" orientation
 * CONFIG_LCD_RPORTRAIT - Define for 240x320 display "reverse portrait"
 *   orientation support.  Default is this 320x240 "landscape" orientation
 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure defines the interface to the LCD provided by the platform.
 *  The nature of this interface is hidden from the MIO283QT2 driver.
 */

struct mio283qt2_lcd_s
{
  /* Interface to write to a MIO283QT2 register.
   *
   *  - select      Select the device (as necessary).
   *                The meaning of selecting (and  and deselecting) is not
   *                defined.  select() will be called before starting any
   *                sequence of operations.  deselect() when that sequence
   *                of operations is complete.
   *  - deselect    Deselect the device (as necessary)
   *  - index       Set register index
   *  - read        Read data from the LCD (auto-incrementing)
   *  - write       Write data to the LCD (auto-incrementing)
   *  - backlight   Set the backlight power level
   *                (0=OFF; CONFIG_LCD_MAXPOWER=MAX)
   */

  void (*select)(FAR struct mio283qt2_lcd_s *dev);
  void (*deselect)(FAR struct mio283qt2_lcd_s *dev);
  void (*index)(FAR struct mio283qt2_lcd_s *dev, uint8_t index);
#ifndef CONFIG_LCD_NOGETRUN
  uint16_t (*read)(FAR struct mio283qt2_lcd_s *dev);
#endif
  void (*write)(FAR struct mio283qt2_lcd_s *dev, uint16_t value);
  void (*backlight)(FAR struct mio283qt2_lcd_s *dev, int power);

  /* platform-specific data may follow */
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
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name:  mio283qt2_lcdinitialize
 *
 * Description:
 *   Initialize the LCD video hardware. The initial state of the LCD is fully
 *   initialized, display memory cleared, and the LCD ready to use, but with
 *   the power setting at 0 (full off).
 *
 ****************************************************************************/

FAR struct lcd_dev_s *mio283qt2_lcdinitialize(
                                         FAR struct mio283qt2_lcd_s *lcd);

/****************************************************************************
 * Name:  mio283qt2_clear
 *
 * Description:
 *   This is a non-standard LCD interface.  Because of the various rotations,
 *   clearing the display in the normal way by writing a sequences of runs
 *   that covers the entire display can be very slow.  Here the display is
 *   cleared by simply setting all GRAM memory to the specified color.
 *
 ****************************************************************************/

void mio283qt2_clear(FAR struct lcd_dev_s *dev, uint16_t color);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_LCD_MIO283QT2 */
#endif /* __INCLUDE_NUTTX_LCD_MIO283QT2_H */
