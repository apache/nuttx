/****************************************************************************
 * include/nuttx/lcd/lcd.h
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

#ifndef __INCLUDE_NUTTX_LCD_LCD_H
#define __INCLUDE_NUTTX_LCD_LCD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#include <stdint.h>
#include <nuttx/video/fb.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Friendlier names */

#define LCD_FULL_OFF     (0)
#define LCD_FULL_ON      CONFIG_LCD_MAXPOWER

#define LCD_MIN_CONTRAST (0)
#define LCD_MAX_CONTRAST CONFIG_LCD_MAXCONTRAST

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

struct lcd_dev_s;

/* Some special LCD drivers require input data to be aligned.
 * Such as starting row and column, width, height, data address, etc.
 */

struct lcddev_area_align_s
{
  uint16_t row_start_align; /* Start row index alignment */
  uint16_t height_align;    /* Height alignment */
  uint16_t col_start_align; /* Start column index alignment */
  uint16_t width_align;     /* Width alignment */
  uint16_t buf_align;       /* Buffer addr alignment */
};

/* This structure describes one color plane.  Some YUV formats may support
 * up to 4 planes (although they probably wouldn't be used on LCD hardware).
 * The framebuffer driver provides the video memory address in its
 * corresponding fb_planeinfo_s structure.  The LCD driver, instead, provides
 * methods to transfer data to/from the LCD color plane.
 */

struct lcd_planeinfo_s
{
  /* LCD Data Transfer ******************************************************/

  /* This method can be used to write a partial raster line to the LCD:
   *
   *  dev     - LCD interface to write to
   *  row     - Starting row to write to (range: 0 <= row < yres)
   *  col     - Starting column to write to (range: 0 <= col <= xres-npixels)
   *  buffer  - The buffer containing the run to be written to the LCD
   *  npixels - The number of pixels to write to the LCD
   *            (range: 0 < npixels <= xres-col)
   */

  int (*putrun)(FAR struct lcd_dev_s *dev, fb_coord_t row, fb_coord_t col,
                FAR const uint8_t *buffer, size_t npixels);

  /* This method can be used to write a rectangular area to the LCD:
   *
   *  dev       - LCD interface to write to
   *  row_start - Starting row to write to (range: 0 <= row < yres)
   *  row_end   - Ending row to write to (range: row_start <= row < yres)
   *  col_start - Starting column to write to (range: 0 <= col <= xres)
   *  col_end   - Ending column to write to
   *              (range: col_start <= col_end < xres)
   *  buffer    - The buffer containing the area to be written to the LCD
   *  stride    - Length of a line in bytes. This parameter may be necessary
   *              to allow the LCD driver to calculate the offset for partial
   *              writes when the buffer needs to be splited for row-by-row
   *              writing.
   *
   * NOTE: this operation may not be supported by the device, in which case
   * the callback pointer will be NULL. In that case, putrun() should be
   * used.
   */

  int (*putarea)(FAR struct lcd_dev_s *dev, fb_coord_t row_start,
                 fb_coord_t row_end, fb_coord_t col_start,
                 fb_coord_t col_end, FAR const uint8_t *buffer,
                 fb_coord_t stride);

  /* This method can be used to read a partial raster line from the LCD:
   *
   *  dev     - LCD interface to read from
   *  row     - Starting row to read from (range: 0 <= row < yres)
   *  col     - Starting column to read read
   *            (range: 0 <= col <= xres-npixels)
   *  buffer  - The buffer in which to return the run read from the LCD
   *  npixels - The number of pixels to read from the LCD
   *            (range: 0 < npixels <= xres-col)
   */

  int (*getrun)(FAR struct lcd_dev_s *dev, fb_coord_t row,
                fb_coord_t col, FAR uint8_t *buffer, size_t npixels);

  /* This method can be used to read a rectangular area from the LCD:
   *
   *  dev       - LCD interface to read from
   *  row_start - Starting row to read from (range: 0 <= row < yres)
   *  row_end   - Ending row to read from (range: row_start <= row < yres)
   *  col_start - Starting column to read from (range: 0 <= col <= xres)
   *  col_end   - Ending column to read from
   *              (range: col_start <= col_end < xres)
   *  buffer    - The buffer where the data will be written
   *  stride    - Length of a line in bytes.
   *
   * NOTE: this operation may not be supported by the device, in which case
   * the callback pointer will be NULL. In that case, getrun() should be
   * used.
   */

  int (*getarea)(FAR struct lcd_dev_s *dev, fb_coord_t row_start,
                 fb_coord_t row_end, fb_coord_t col_start,
                 fb_coord_t col_end, FAR uint8_t *buffer,
                 fb_coord_t stride);

  /* This method can be used to redraw display's content.
   *
   *  dev       - LCD interface to redraw its memory content
   *
   * NOTE: In case of non e-ink dispalys redrawing is cheap and can be done
   * after each memory modification. Redrawing e-ink display is time and
   * energy consuming.
   * In order to avoid such operation (time and energy consumption) we can
   * implement callback function putrun without redrawing the screen.
   * Function putrun is called many times unless the function putarea is
   * implemented.
   */

  int (*redraw)(FAR struct lcd_dev_s *dev);

  /* Plane color characteristics ********************************************/

  /* This is working memory allocated by the LCD driver for each LCD device
   * and for each color plane.  This memory will hold one raster line of
   * data. The size of the allocated run buffer must therefore be at least
   * (bpp * xres / 8).  Actual alignment of the buffer must conform to the
   * bitwidth of the underlying pixel type.
   *
   * If there are multiple planes, they may share the same working buffer
   * because different planes will not be operate on concurrently.  However,
   * if there are multiple LCD devices, they must each have unique run
   * buffers.
   */

  FAR uint8_t *buffer;

  /* This is the number of bits in one pixel.  This may be one of {1, 2, 4,
   * 8, 16, 24, or 32} unless support for one or more of those resolutions
   * has been disabled.
   */

  uint8_t  bpp;

  /* This is the LCD interface corresponding to which this color plane
   * belongs.
   */

  FAR struct lcd_dev_s *dev;
};

/* This structure defines an LCD interface */

struct lcd_dev_s
{
  /* LCD Configuration ******************************************************/

  /* Get information about the LCD video controller configuration and the
   * configuration of each LCD color plane.
   */

  int (*getvideoinfo)(FAR struct lcd_dev_s *dev,
         FAR struct fb_videoinfo_s *vinfo);
  int (*getplaneinfo)(FAR struct lcd_dev_s *dev, unsigned int planeno,
         FAR struct lcd_planeinfo_s *pinfo);

  /* LCD RGB Mapping ********************************************************/

  /* The following are provided only if the video hardware supports RGB color
   * mapping
   */

#ifdef CONFIG_FB_CMAP
  int (*getcmap)(FAR struct lcd_dev_s *dev, FAR struct fb_cmap_s *cmap);
  int (*putcmap)(FAR struct lcd_dev_s *dev,
         FAR const struct fb_cmap_s *cmap);
#endif

  /* Cursor Controls ********************************************************/

  /* The following are provided only if the video hardware supports a
   * hardware cursor
   */

#ifdef CONFIG_FB_HWCURSOR
  int (*getcursor)(FAR struct lcd_dev_s *dev,
        FAR struct fb_cursorattrib_s *attrib);
  int (*setcursor)(FAR struct lcd_dev_s *dev,
        FAR struct fb_setcursor_s *settings);
#endif

  /* LCD Specific Controls **************************************************/

  /* Get the LCD panel power status (0: full off - CONFIG_LCD_MAXPOWER: full
   * on).  On backlit LCDs, this setting may correspond to the backlight
   * setting.
   */

  int (*getpower)(struct lcd_dev_s *dev);

  /* Enable/disable LCD panel power (0: full off - CONFIG_LCD_MAXPOWER: full
   * on).  On backlit LCDs, this setting may correspond to the backlight
   * setting.
   */

  int (*setpower)(struct lcd_dev_s *dev, int power);

  /* Get the current contrast setting (0-CONFIG_LCD_MAXCONTRAST) */

  int (*getcontrast)(struct lcd_dev_s *dev);

  /* Set LCD panel contrast (0-CONFIG_LCD_MAXCONTRAST) */

  int (*setcontrast)(struct lcd_dev_s *dev, unsigned int contrast);

  /* Set LCD panel frame rate (0: disable refresh) */

  int (*setframerate)(struct lcd_dev_s *dev, int rate);

  /* Get LCD panel frame rate (0: disable refresh) */

  int (*getframerate)(struct lcd_dev_s *dev);

  /* Get LCD panel area alignment */

  int (*getareaalign)(FAR struct lcd_dev_s *dev,
                      FAR struct lcddev_area_align_s *align);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_LCD_LCD_H */
