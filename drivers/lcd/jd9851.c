/****************************************************************************
 * drivers/lcd/jd9851.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/spi/spi.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/jd9851.h>

#include "jd9851.h"

#ifdef CONFIG_LCD_JD9851

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Verify that all configuration requirements have been met */

#ifndef CONFIG_LCD_JD9851_SPIMODE
#  define CONFIG_LCD_JD9851_SPIMODE SPIDEV_MODE0
#endif

/* SPI frequency */

#ifndef CONFIG_LCD_JD9851_FREQUENCY
#  define CONFIG_LCD_JD9851_FREQUENCY 1000000
#endif

/* Check contrast selection */

#if !defined(CONFIG_LCD_MAXCONTRAST)
#  define CONFIG_LCD_MAXCONTRAST 1
#endif

/* Check power setting */

#if !defined(CONFIG_LCD_MAXPOWER) || CONFIG_LCD_MAXPOWER < 1
#  define CONFIG_LCD_MAXPOWER 1
#endif

#if CONFIG_LCD_MAXPOWER > 255
#  error "CONFIG_LCD_MAXPOWER must be less than 256 to fit in uint8_t"
#endif

/* Check orientation */

#if defined(CONFIG_LCD_PORTRAIT)
#  if defined(CONFIG_LCD_LANDSCAPE) || defined(CONFIG_LCD_RLANDSCAPE) ||\
      defined(CONFIG_LCD_RPORTRAIT)
#    error "Cannot define both portrait and any other orientations"
#  endif
#elif defined(CONFIG_LCD_RPORTRAIT)
#  if defined(CONFIG_LCD_LANDSCAPE) || defined(CONFIG_LCD_RLANDSCAPE)
#    error "Cannot define both rportrait and any other orientations"
#  endif
#elif defined(CONFIG_LCD_LANDSCAPE)
#  ifdef CONFIG_LCD_RLANDSCAPE
#    error "Cannot define both landscape and any other orientations"
#  endif
#elif !defined(CONFIG_LCD_RLANDSCAPE)
#  define CONFIG_LCD_LANDSCAPE 1
#endif

/* Display Resolution */

#if !defined(CONFIG_LCD_JD9851_XRES)
#  define CONFIG_LCD_JD9851_XRES 240
#endif

#if !defined(CONFIG_LCD_JD9851_YRES)
#  define CONFIG_LCD_JD9851_YRES 320
#endif

#if !defined(CONFIG_LCD_JD9851_BPP)
#  define CONFIG_LCD_JD9851_BPP 16
#endif

#if !defined(CONFIG_LCD_JD9851_XOFFSET)
#  define CONFIG_LCD_JD9851_XOFFSET 0
#endif

#if !defined(CONFIG_LCD_JD9851_YOFFSET)
#  define CONFIG_LCD_JD9851_YOFFSET 0
#endif

#define JD9851_LUT_SIZE    CONFIG_LCD_JD9851_YRES

#if defined(CONFIG_LCD_LANDSCAPE) || defined(CONFIG_LCD_RLANDSCAPE)
#  define JD9851_XRES       CONFIG_LCD_JD9851_YRES
#  define JD9851_YRES       CONFIG_LCD_JD9851_XRES
#  define JD9851_XOFFSET    CONFIG_LCD_JD9851_YOFFSET
#  define JD9851_YOFFSET    CONFIG_LCD_JD9851_XOFFSET
#else
#  define JD9851_XRES       CONFIG_LCD_JD9851_XRES
#  define JD9851_YRES       CONFIG_LCD_JD9851_YRES
#  define JD9851_XOFFSET    CONFIG_LCD_JD9851_XOFFSET
#  define JD9851_YOFFSET    CONFIG_LCD_JD9851_YOFFSET
#endif

/* Color depth and format */

#ifdef CONFIG_LCD_JD9851_BPP
#  if (CONFIG_LCD_JD9851_BPP == 16)
#    define JD9851_BPP           16
#    define JD9851_COLORFMT      FB_FMT_RGB16_565
#    define JD9851_BYTESPP       2
#  else
#    define JD9851_BPP           16
#    define JD9851_COLORFMT      FB_FMT_RGB16_565
#    define JD9851_BYTESPP       2
#    warning "Invalid color depth.  Falling back to 16bpp"
#  endif
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the state of this driver */

struct jd9851_dev_s
{
  /* Publicly visible device structure */

  struct lcd_dev_s dev;

  /* Private LCD-specific information follows */

  FAR struct spi_dev_s *spi;  /* SPI device */
  uint8_t bpp;                /* Selected color depth */
  uint8_t power;              /* Current power setting */

  /* This is working memory allocated by the LCD driver for each LCD device
   * and for each color plane. This memory will hold one raster line of data.
   * The size of the allocated run buffer must therefore be at least
   * (bpp * xres / 8).  Actual alignment of the buffer must conform to the
   * bitwidth of the underlying pixel type.
   *
   * If there are multiple planes, they may share the same working buffer
   * because different planes will not be operate on concurrently.  However,
   * if there are multiple LCD devices, they must each have unique run
   * buffers.
   */

  uint16_t runbuffer[JD9851_LUT_SIZE];
};

/****************************************************************************
 * Private Function Protototypes
 ****************************************************************************/

/* Misc. Helpers */

static void jd9851_select(FAR struct spi_dev_s *spi, int bits);
static void jd9851_deselect(FAR struct spi_dev_s *spi);

static inline void jd9851_sendcmd(FAR struct jd9851_dev_s *dev, uint8_t cmd);
static void jd9851_cmddata(FAR struct jd9851_dev_s *dev, uint8_t cmd,
                               const uint8_t *data, int len);
static void jd9851_init(FAR struct jd9851_dev_s *dev);
static void jd9851_sleep(FAR struct jd9851_dev_s *dev, bool sleep);
static void jd9851_setorientation(FAR struct jd9851_dev_s *dev);
static void jd9851_display(FAR struct jd9851_dev_s *dev, bool on);
static void jd9851_setcursor(FAR struct jd9851_dev_s *dev,
                           uint16_t x0, uint16_t y0,
                           uint16_t x1, uint16_t y1);
static void jd9851_bpp(FAR struct jd9851_dev_s *dev, int bpp);
static void jd9851_wrram(FAR struct jd9851_dev_s *dev,
                         FAR const uint8_t *buff, size_t size , size_t skip,
                         size_t count);
#ifndef CONFIG_LCD_NOGETRUN
static void jd9851_rdram(FAR struct jd9851_dev_s *dev,
                         FAR uint16_t *buff, size_t size);
#endif
static void jd9851_fill(FAR struct jd9851_dev_s *dev, uint16_t color);

/* LCD Data Transfer Methods */

static int jd9851_putrun(FAR struct lcd_dev_s *dev,
                         fb_coord_t row, fb_coord_t col,
                         FAR const uint8_t *buffer, size_t npixels);
static int jd9851_putarea(FAR struct lcd_dev_s *dev,
                          fb_coord_t row_start, fb_coord_t row_end,
                          fb_coord_t col_start, fb_coord_t col_end,
                          FAR const uint8_t *buffer, fb_coord_t stride);
#ifndef CONFIG_LCD_NOGETRUN
static int jd9851_getrun(FAR struct lcd_dev_s *dev,
                         fb_coord_t row, fb_coord_t col,
                         FAR uint8_t *buffer, size_t npixels);
#endif

/* LCD Configuration */

static int jd9851_getvideoinfo(FAR struct lcd_dev_s *dev,
                               FAR struct fb_videoinfo_s *vinfo);
static int jd9851_getplaneinfo(FAR struct lcd_dev_s *dev,
                               unsigned int planeno,
                               FAR struct lcd_planeinfo_s *pinfo);

/* LCD Specific Controls */

static int jd9851_getpower(FAR struct lcd_dev_s *dev);
static int jd9851_setpower(FAR struct lcd_dev_s *dev, int power);
static int jd9851_getcontrast(FAR struct lcd_dev_s *dev);
static int jd9851_setcontrast(FAR struct lcd_dev_s *dev,
                              unsigned int contrast);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct jd9851_dev_s g_lcddev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: jd9851_select
 *
 * Description:
 *   Select the SPI, locking and re-configuring if necessary
 *
 * Input Parameters:
 *   spi   - Reference to the SPI driver structure
 *   bits  - Number of SPI bits
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void jd9851_select(FAR FAR struct spi_dev_s *spi, int bits)
{
  /* Select JD9851 chip (locking the SPI bus in case there are multiple
   * devices competing for the SPI bus
   */

  SPI_LOCK(spi, true);
  SPI_SELECT(spi, SPIDEV_DISPLAY(0), true);

  /* Now make sure that the SPI bus is configured for the JD9851 (it
   * might have gotten configured for a different device while unlocked)
   */

  SPI_SETMODE(spi, CONFIG_LCD_JD9851_SPIMODE);
  SPI_SETBITS(spi, bits);
  SPI_SETFREQUENCY(spi, CONFIG_LCD_JD9851_FREQUENCY);
}

/****************************************************************************
 * Name: jd9851_deselect
 *
 * Description:
 *   De-select the SPI
 *
 * Input Parameters:
 *   spi  - Reference to the SPI driver structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void jd9851_deselect(FAR FAR struct spi_dev_s *spi)
{
  /* De-select JD9851 chip and relinquish the SPI bus. */

  SPI_SELECT(spi, SPIDEV_DISPLAY(0), false);
  SPI_LOCK(spi, false);
}

/****************************************************************************
 * Name: jd9851_sendcmd
 *
 * Description:
 *   Send a command to the driver.
 *
 ****************************************************************************/

static inline void jd9851_sendcmd(FAR struct jd9851_dev_s *dev, uint8_t cmd)
{
  jd9851_select(dev->spi, 8);
  SPI_CMDDATA(dev->spi, SPIDEV_DISPLAY(0), true);
  SPI_SEND(dev->spi, cmd);
  SPI_CMDDATA(dev->spi, SPIDEV_DISPLAY(0), false);
  jd9851_deselect(dev->spi);
}

/****************************************************************************
 * Name: jd9851_cmddata
 *
 * Description:
 *   Send a command and a series of data to the driver.
 *
 ****************************************************************************/

static void jd9851_cmddata(FAR struct jd9851_dev_s *dev, uint8_t cmd,
                                      const uint8_t *data, int len)
{
  jd9851_select(dev->spi, 8);
  SPI_CMDDATA(dev->spi, SPIDEV_DISPLAY(0), true);
  SPI_SEND(dev->spi, cmd);
  SPI_CMDDATA(dev->spi, SPIDEV_DISPLAY(0), false);
  SPI_SNDBLOCK(dev->spi, data, len);
  jd9851_deselect(dev->spi);
}

/****************************************************************************
 * Name: jd9851_init
 *
 * Description:
 *   Send jd9851 internal init commands.
 *
 * Assumption/Limitations:
 *   Initialization is hardware-specific and may need to be rewritten for
 *   different modules.
 *
 ****************************************************************************/

static void jd9851_init(FAR struct jd9851_dev_s *dev)
{
  jd9851_cmddata(dev, JD9851_PASSWORD, (const uint8_t *) "\x98\x51\xe9", 3);

  jd9851_cmddata(dev, JD9851_SET_PAGE_CMD, (const uint8_t *) "\x00", 1);
  jd9851_cmddata(dev, JD9851_GAMMA_SET_CMD, (const uint8_t *)
                 "\x1b\x7a\x17\x32", 4);
  jd9851_cmddata(dev, JD9851_R_GAMMA_SET_CMD, (const uint8_t *)
                 "\x3c\x32\x2b\x2c\x2f\x32\x2c\x2b\x28\x27\x22\x16\x10\x0b\
                  \x05\x0e\x3c\x32\x2b\x2c\x2f\x32\x2c\x2b\x28\x27\x22\x16\
                  \x10\x0b\x05\x0e", 32);
  jd9851_cmddata(dev, JD9851_POWER_CTRL, (const uint8_t *)
                 "\x33\x28\xcc", 3);
  jd9851_cmddata(dev, JD9851_DCDC_SET, (const uint8_t *)
                 "\x47\x7a\x30\x30\x6c\x60\x50\x70", 8);
  jd9851_cmddata(dev, JD9851_VDDD_CTRL, (const uint8_t *) "\x38\x3c", 2);
  jd9851_cmddata(dev, JD9851_SETSTBA, (const uint8_t *) "\x31\x20", 2);
  jd9851_cmddata(dev, JD9851_SETPANEL, (const uint8_t *) "\x16", 1);
  jd9851_cmddata(dev, JD9851_SETRGBCYC, (const uint8_t *)
                 "\x08\x00\x0a\x10\x08\x54\x45\x71\x2c", 9);
  jd9851_cmddata(dev, JD9851_SETTCON, (const uint8_t *)
                 "\x00\xa0\x79\x0e\x0a\x16\x79\x0e\x0a\x16\x79\x0e\x0a\x16\
                  \x82\x00\x03", 17);
  jd9851_cmddata(dev, JD9851_SETGD, (const uint8_t *)
                 "\x04\x0c\x6b\x0f\x07\x03", 6);
  jd9851_cmddata(dev, JD9851_SETRGBIF, (const uint8_t *) "\x22\x20", 2);
  jd9851_cmddata(dev, JD9851_RAM_CTRL, (const uint8_t *) "\x00\x00", 2);

  jd9851_cmddata(dev, JD9851_SET_PAGE_CMD, (const uint8_t *) "\x02", 1);
  jd9851_cmddata(dev, JD9851_DCDC_SET2, (const uint8_t *)
                 "\x19\xa0\x2f\x04\x33", 5);
  jd9851_cmddata(dev, JD9851_SETPANEL, (const uint8_t *)
                 "\x10\x66\x66\x01", 4);
  jd9851_cmddata(dev, JD9851_OSCM_SET, (const uint8_t *) "\x01\x00\x00", 3);
  jd9851_cmddata(dev, JD9851_SETMIPI_2, (const uint8_t *) "\x10\x20\xf4", 3);

  jd9851_sendcmd(dev, JD9851_SLEEP_OUT);
  up_mdelay(120);
}

/****************************************************************************
 * Name: jd9851_sleep
 *
 * Description:
 *   Sleep or wake up the driver.
 *
 ****************************************************************************/

static void jd9851_sleep(FAR struct jd9851_dev_s *dev, bool sleep)
{
  if (sleep)
    {
      jd9851_sendcmd(dev, JD9851_SLEEP_IN);
    }
  else
    {
      jd9851_sendcmd(dev, JD9851_SLEEP_OUT);
    }

  up_mdelay(120);
}

/****************************************************************************
 * Name: jd9851_display
 *
 * Description:
 *   Turn on or off the display.
 *
 ****************************************************************************/

static void jd9851_display(FAR struct jd9851_dev_s *dev, bool on)
{
  uint8_t reg;

#if defined(CONFIG_LCD_JD9851_TE)
  reg = JD9851_TE_MODE0;
  jd9851_cmddata(dev, JD9851_TEON, &reg, 1);
#endif

#if defined(CONFIG_LCD_JD9851_INVERT)
  jd9851_sendcmd(dev, JD9851_INVERSION_ON);
#endif

  if (on)
    {
      jd9851_sendcmd(dev, JD9851_DISON);
    }
  else
    {
      jd9851_sendcmd(dev, JD9851_DISOFF);
    }
}

/****************************************************************************
 * Name: jd9851_setorientation
 *
 * Description:
 *   Set screen orientation.
 *
 ****************************************************************************/

static void jd9851_setorientation(FAR struct jd9851_dev_s *dev)
{
  uint8_t reg = 0x00;

#if !defined(CONFIG_LCD_PORTRAIT) || defined(CONFIG_LCD_JD9851_BGR)

#  if defined(CONFIG_LCD_RLANDSCAPE)

  reg = JD9851_MADCTL_MY | JD9851_MADCTL_MV;

#  elif defined(CONFIG_LCD_LANDSCAPE)

  reg = JD9851_MADCTL_MV | JD9851_MADCTL_MX;

#  elif defined(CONFIG_LCD_RPORTRAIT)

  reg = JD9851_MADCTL_MY | JD9851_MADCTL_MX;

#  endif

#  if defined(CONFIG_LCD_JD9851_BGR)

  reg |= JD9851_MADCTL_BGR;

#  endif

#endif

  jd9851_cmddata(dev, JD9851_MADCTL, &reg, 1);
}

/****************************************************************************
 * Name: jd9851_setcursor
 *
 * Description:
 *   Set the rectangular area for an upcoming read or write from RAM.
 *
 ****************************************************************************/

static void jd9851_setcursor(FAR struct jd9851_dev_s *dev,
                           uint16_t x0, uint16_t y0,
                           uint16_t x1, uint16_t y1)
{
  /* Set row address */

  jd9851_sendcmd(dev, JD9851_PASET);
  jd9851_select(dev->spi, 8);
  SPI_SEND(dev->spi, (y0 + JD9851_YOFFSET) >> 8);
  SPI_SEND(dev->spi, (y0 + JD9851_YOFFSET) & 0xff);
  SPI_SEND(dev->spi, (y1 + JD9851_YOFFSET) >> 8);
  SPI_SEND(dev->spi, (y1 + JD9851_YOFFSET) & 0xff);
  jd9851_deselect(dev->spi);

  /* Set column address */

  jd9851_sendcmd(dev, JD9851_CASET);
  jd9851_select(dev->spi, 8);
  SPI_SEND(dev->spi, (x0 + JD9851_XOFFSET) >> 8);
  SPI_SEND(dev->spi, (x0 + JD9851_XOFFSET) & 0xff);
  SPI_SEND(dev->spi, (x1 + JD9851_XOFFSET) >> 8);
  SPI_SEND(dev->spi, (x1 + JD9851_XOFFSET) & 0xff);
  jd9851_deselect(dev->spi);
}

/****************************************************************************
 * Name: jd9851_bpp
 *
 * Description:
 *   Set the color depth of the device.
 *
 ****************************************************************************/

static void jd9851_bpp(FAR struct jd9851_dev_s *dev, int bpp)
{
  uint8_t depth;

  /* Don't send any command if the depth hasn't changed. */

  if (dev->bpp != bpp)
    {
      depth = bpp >> 2 | 1;
      depth = (depth & 0x0f) << 4 | depth;
      jd9851_cmddata(dev, JD9851_COLMOD, &depth, 1);

      /* Cache the new BPP */

      dev->bpp = bpp;
    }
}

/****************************************************************************
 * Name: jd9851_wrram
 *
 * Description:
 *   Write to the driver's RAM. It is possible to write multiples of size
 *   while skipping some values.
 *
 ****************************************************************************/

static void jd9851_wrram(FAR struct jd9851_dev_s *dev,
                         FAR const uint8_t *buff, size_t size, size_t skip,
                         size_t count)
{
  size_t i;

  jd9851_sendcmd(dev, JD9851_RAMWR);

  jd9851_select(dev->spi, 8);

  for (i = 0; i < count; i++)
    {
      SPI_SNDBLOCK(dev->spi, buff + (i * (size + skip)), size);
    }

  jd9851_deselect(dev->spi);
}

/****************************************************************************
 * Name: jd9851_rdram
 *
 * Description:
 *   Read from the driver's RAM.
 *
 ****************************************************************************/

#ifndef CONFIG_LCD_NOGETRUN
static void jd9851_rdram(FAR struct jd9851_dev_s *dev,
                         FAR uint16_t *buff, size_t size)
{
  jd9851_sendcmd(dev, JD9851_RAMRD);

  jd9851_select(dev->spi, JD9851_BYTESPP * 8);
  SPI_RECVBLOCK(dev->spi, buff, size);
  jd9851_deselect(dev->spi);
}
#endif

/****************************************************************************
 * Name: jd9851_fill
 *
 * Description:
 *   Fill the display with the specified color.
 *
 ****************************************************************************/

static void jd9851_fill(FAR struct jd9851_dev_s *dev, uint16_t color)
{
  int i;

  jd9851_setcursor(dev, 0, 0, JD9851_XRES - 1, JD9851_YRES - 1);

  jd9851_sendcmd(dev, JD9851_RAMWR);
  jd9851_select(dev->spi, JD9851_BYTESPP *8);

  for (i = 0; i < JD9851_XRES * JD9851_YRES; i++)
    {
      SPI_SEND(dev->spi, color);
    }

  jd9851_deselect(dev->spi);
}

/****************************************************************************
 * Name:  jd9851_putrun
 *
 * Description:
 *   This method can be used to write a partial raster line to the LCD:
 *
 *   dev     - The lcd device
 *   row     - Starting row to write to (range: 0 <= row < yres)
 *   col     - Starting column to write to (range: 0 <= col <= xres-npixels)
 *   buffer  - The buffer containing the run to be written to the LCD
 *   npixels - The number of pixels to write to the LCD
 *             (range: 0 < npixels <= xres-col)
 *
 ****************************************************************************/

static int jd9851_putrun(FAR struct lcd_dev_s *dev,
                         fb_coord_t row, fb_coord_t col,
                         FAR const uint8_t *buffer, size_t npixels)
{
  FAR struct jd9851_dev_s *priv = (FAR struct jd9851_dev_s *)dev;

  ginfo("row: %d col: %d npixels: %d\n", row, col, npixels);
  DEBUGASSERT(buffer && ((uintptr_t)buffer & 1) == 0);

  jd9851_setcursor(priv, col, row, col + npixels - 1, row);
  jd9851_wrram(priv, buffer, npixels * (priv->bpp >> 3), 0, 1);

  return OK;
}

/****************************************************************************
 * Name:  jd9851_putarea
 *
 * Description:
 *   This method can be used to write a partial area to the LCD:
 *
 *   dev       - The lcd device
 *   row_start - Starting row to write to (range: 0 <= row < yres)
 *   row_end   - Ending row to write to (range: row_start <= row < yres)
 *   col_start - Starting column to write to (range: 0 <= col <= xres)
 *   col_end   - Ending column to write to
 *               (range: col_start <= col_end < xres)
 *   buffer    - The buffer containing the area to be written to the LCD
 *   stride    - Length of a line in bytes. This parameter may be necessary
 *               to allow the LCD driver to calculate the offset for partial
 *               writes when the buffer needs to be splited for row-by-row
 *               writing.
 *
 ****************************************************************************/

static int jd9851_putarea(FAR struct lcd_dev_s *dev,
                          fb_coord_t row_start, fb_coord_t row_end,
                          fb_coord_t col_start, fb_coord_t col_end,
                          FAR const uint8_t *buffer, fb_coord_t stride)
{
  FAR struct jd9851_dev_s *priv = (FAR struct jd9851_dev_s *)dev;
  size_t cols = col_end - col_start + 1;
  size_t rows = row_end - row_start + 1;
  size_t row_size = cols * (priv->bpp >> 3);

  ginfo("row_start: %d row_end: %d col_start: %d col_end: %d\n",
         row_start, row_end, col_start, col_end);

  DEBUGASSERT(buffer && ((uintptr_t)buffer & 1) == 0);

  jd9851_setcursor(priv, col_start, row_start, col_end, row_end);

  /* If the stride is the same of the row, a single SPI transfer is enough.
   * That is always true for lcddev. For framebuffer, that indicates a full
   * screen or full row update.
   */

  if (stride == row_size)
    {
      /* simpler case, we can just send the whole buffer */

      ginfo("Using full screen/full row mode\n");
      jd9851_wrram(priv, buffer, rows * row_size, 0, 1);
    }
  else
    {
      /* We have to go row by row */

      ginfo("Falling-back to row by row mode\n");
      jd9851_wrram(priv, buffer, row_size, stride - row_size, rows);
    }

  return OK;
}

/****************************************************************************
 * Name:  jd9851_getrun
 *
 * Description:
 *   This method can be used to read a partial raster line from the LCD:
 *
 *  dev     - The lcd device
 *  row     - Starting row to read from (range: 0 <= row < yres)
 *  col     - Starting column to read read (range: 0 <= col <= xres-npixels)
 *  buffer  - The buffer in which to return the run read from the LCD
 *  npixels - The number of pixels to read from the LCD
 *            (range: 0 < npixels <= xres-col)
 *
 ****************************************************************************/

#ifndef CONFIG_LCD_NOGETRUN
static int jd9851_getrun(FAR struct lcd_dev_s *dev,
                         fb_coord_t row, fb_coord_t col,
                         FAR uint8_t *buffer, size_t npixels)
{
  FAR struct jd9851_dev_s *priv = (FAR struct jd9851_dev_s *)dev;
  FAR uint16_t *dest = (FAR uint16_t *)buffer;

  ginfo("row: %d col: %d npixels: %d\n", row, col, npixels);
  DEBUGASSERT(buffer && ((uintptr_t)buffer & 1) == 0);

  jd9851_setcursor(priv, col, row, col + npixels - 1, row);
  jd9851_rdram(priv, dest, npixels);

  return OK;
}
#endif

/****************************************************************************
 * Name:  jd9851_getvideoinfo
 *
 * Description:
 *   Get information about the LCD video controller configuration.
 *
 ****************************************************************************/

static int jd9851_getvideoinfo(FAR struct lcd_dev_s *dev,
                               FAR struct fb_videoinfo_s *vinfo)
{
  DEBUGASSERT(dev && vinfo);
  lcdinfo("fmt: %d xres: %d yres: %d nplanes: 1\n",
          JD9851_COLORFMT, JD9851_XRES, JD9851_YRES);

  vinfo->fmt     = JD9851_COLORFMT;    /* Color format: RGB16-565: RRRR RGGG GGGB BBBB */
  vinfo->xres    = JD9851_XRES;        /* Horizontal resolution in pixel columns */
  vinfo->yres    = JD9851_YRES;        /* Vertical resolution in pixel rows */
  vinfo->nplanes = 1;                  /* Number of color planes supported */
  return OK;
}

/****************************************************************************
 * Name:  jd9851_getplaneinfo
 *
 * Description:
 *   Get information about the configuration of each LCD color plane.
 *
 ****************************************************************************/

static int jd9851_getplaneinfo(FAR struct lcd_dev_s *dev,
                               unsigned int planeno,
                               FAR struct lcd_planeinfo_s *pinfo)
{
  FAR struct jd9851_dev_s *priv = (FAR struct jd9851_dev_s *)dev;

  DEBUGASSERT(dev && pinfo && planeno == 0);
  lcdinfo("planeno: %d bpp: %d\n", planeno, JD9851_BPP);

  pinfo->putrun = jd9851_putrun;                  /* Put a run into LCD memory */
  pinfo->putarea = jd9851_putarea;                /* Put an area into LCD */
#ifndef CONFIG_LCD_NOGETRUN
  pinfo->getrun = jd9851_getrun;                  /* Get a run from LCD memory */
#endif
  pinfo->buffer = (FAR uint8_t *)priv->runbuffer; /* Run scratch buffer */
  pinfo->bpp    = priv->bpp;                      /* Bits-per-pixel */
  pinfo->dev    = dev;                            /* The lcd device */
  return OK;
}

/****************************************************************************
 * Name:  jd9851_getpower
 ****************************************************************************/

static int jd9851_getpower(FAR struct lcd_dev_s *dev)
{
  FAR struct jd9851_dev_s *priv = (FAR struct jd9851_dev_s *)dev;

  lcdinfo("power: %d\n", priv->power);
  return priv->power;
}

/****************************************************************************
 * Name:  jd9851_setpower
 ****************************************************************************/

static int jd9851_setpower(FAR struct lcd_dev_s *dev, int power)
{
  FAR struct jd9851_dev_s *priv = (FAR struct jd9851_dev_s *)dev;

  lcdinfo("power: %d\n", power);
  DEBUGASSERT((unsigned)power <= CONFIG_LCD_MAXPOWER);

  /* Set new power level */

  if (power > 0)
    {
      /* Turn on the display */

      jd9851_display(priv, true);

      /* Save the power */

      priv->power = power;
    }
  else
    {
      /* Turn off the display */

      jd9851_display(priv, false);

      /* Save the power */

      priv->power = 0;
    }

  return OK;
}

/****************************************************************************
 * Name:  jd9851_getcontrast
 *
 * Description:
 *   Get the current contrast setting (0-CONFIG_LCD_MAXCONTRAST).
 *
 ****************************************************************************/

static int jd9851_getcontrast(FAR struct lcd_dev_s *dev)
{
  lcdinfo("Not implemented\n");
  return -ENOSYS;
}

/****************************************************************************
 * Name:  jd9851_setcontrast
 *
 * Description:
 *   Set LCD panel contrast (0-CONFIG_LCD_MAXCONTRAST).
 *
 ****************************************************************************/

static int jd9851_setcontrast(FAR struct lcd_dev_s *dev,
                              unsigned int contrast)
{
  lcdinfo("contrast: %d\n", contrast);
  return -ENOSYS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  jd9851_initialize
 *
 * Description:
 *   Initialize the JD9851 video hardware.  The initial state of the
 *   LCD is fully initialized, display memory cleared, and the LCD ready
 *   to use, but with the power setting at 0 (full off == sleep mode).
 *
 * Returned Value:
 *
 *   On success, this function returns a reference to the LCD object for
 *   the specified LCD.  NULL is returned on any failure.
 *
 ****************************************************************************/

FAR struct lcd_dev_s *jd9851_lcdinitialize(FAR FAR struct spi_dev_s *spi)
{
  FAR struct jd9851_dev_s *priv = &g_lcddev;

  /* Initialize the driver data structure */

  priv->dev.getvideoinfo = jd9851_getvideoinfo;
  priv->dev.getplaneinfo = jd9851_getplaneinfo;
  priv->dev.getpower     = jd9851_getpower;
  priv->dev.setpower     = jd9851_setpower;
  priv->dev.getcontrast  = jd9851_getcontrast;
  priv->dev.setcontrast  = jd9851_setcontrast;
  priv->spi              = spi;

  /* Init the hardware and clear the display */

  jd9851_init(priv);
  jd9851_display(priv, false);
  jd9851_sleep(priv, false);
  jd9851_bpp(priv, JD9851_BPP);
  jd9851_setorientation(priv);
  jd9851_fill(priv, 0x0000);
  jd9851_display(priv, true);

  return &priv->dev;
}

#endif /* CONFIG_LCD_JD9851 */
