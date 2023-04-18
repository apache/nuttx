/****************************************************************************
 * include/nuttx/lcd/lcd_dev.h
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

#ifndef __INCLUDE_NUTTX_LCD_LCD_DEV_H
#define __INCLUDE_NUTTX_LCD_LCD_DEV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/input/ioctl.h>
#include <nuttx/lcd/lcd.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

#define LCDDEVIO_PUTRUN       _LCDIOC(0)  /* Arg: const struct lcddev_run_s* */
#define LCDDEVIO_PUTAREA      _LCDIOC(1)  /* Arg: const struct lcddev_area_s* */
#define LCDDEVIO_GETRUN       _LCDIOC(2)  /* Arg: struct lcddev_run_s* */
#define LCDDEVIO_GETAREA      _LCDIOC(3)  /* Arg: struct lcddev_area_s* */
#define LCDDEVIO_GETPOWER     _LCDIOC(4)  /* Arg: int* */
#define LCDDEVIO_SETPOWER     _LCDIOC(5)  /* Arg: int */
#define LCDDEVIO_GETCONTRAST  _LCDIOC(6)  /* Arg: int* */
#define LCDDEVIO_SETCONTRAST  _LCDIOC(7)  /* Arg: unsigned int */
#define LCDDEVIO_GETPLANEINFO _LCDIOC(8)  /* Arg: struct lcd_planeinfo_s* */
#define LCDDEVIO_GETVIDEOINFO _LCDIOC(9)  /* Arg: struct fb_videoinfo_s* */
#define LCDDEVIO_SETPLANENO   _LCDIOC(10) /* Arg: int */

#ifdef CONFIG_FB_CMAP
#define LCDDEVIO_GETCMAP      _LCDIOC(11) /* Arg: struct fb_cmap_s* */
#define LCDDEVIO_PUTCMAP      _LCDIOC(12) /* Arg: const struct fb_cmap_s* */
#endif

#ifdef CONFIG_FB_HWCURSOR
#define LCDDEVIO_GETCURSOR    _LCDIOC(13) /* Arg: struct fb_cursorattrib_s* */
#define LCDDEVIO_SETCURSOR    _LCDIOC(14) /* Arg: struct fb_setcursor_s* */
#endif

#define LCDDEVIO_SETFRAMERATE _LCDIOC(15) /* Arg: int */
#define LCDDEVIO_GETFRAMERATE _LCDIOC(16) /* Arg: int* */

#define LCDDEVIO_GETAREAALIGN _LCDIOC(17) /* Arg: struct lcddev_area_align_s* */

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

struct lcddev_run_s
{
  fb_coord_t row, col;
  FAR uint8_t *data;
  size_t npixels;
};

struct lcddev_area_s
{
  fb_coord_t row_start, row_end;
  fb_coord_t col_start, col_end;
  FAR uint8_t *data;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: lcddev_register
 *
 * Description:
 *   Register the lcd_dev character driver as the specified device.
 *
 * Input Parameters:
 *   devno - The LCD device number.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int lcddev_register(int devno);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_LCD_LCD_DEV_H */
