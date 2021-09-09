/****************************************************************************
 * drivers/lcd/st77xx.c
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
#include <nuttx/lcd/st77xx.h>

#include "st77xx.h"

#ifdef CONFIG_LCD_ST77XX

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Verify that all configuration requirements have been met */

#ifndef CONFIG_LCD_ST77XX_SPIMODE
#  define CONFIG_LCD_ST77XX_SPIMODE SPIDEV_MODE0
#endif

/* SPI frequency */

#ifndef CONFIG_LCD_ST77XX_FREQUENCY
#  define CONFIG_LCD_ST77XX_FREQUENCY 1000000
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

#if defined(CONFIG_LCD_ST7735_GM00)
#  define CONFIG_LCD_ST77XX_XRES 132
#  define CONFIG_LCD_ST77XX_YRES 162
#  define ST77XX_LUT_SIZE    162
#elif defined(CONFIG_LCD_ST7735) & !defined(CONFIG_LCD_ST7735_GM00)
#  define CONFIG_LCD_ST77XX_XRES 128
#  define CONFIG_LCD_ST77XX_YRES 160
#  define ST77XX_LUT_SIZE    160
#elif defined(CONFIG_LCD_ST7789)
#  define CONFIG_LCD_ST77XX_XRES CONFIG_LCD_ST7789_XRES
#  define CONFIG_LCD_ST77XX_YRES CONFIG_LCD_ST7789_YRES
#  define ST77XX_LUT_SIZE    CONFIG_LCD_ST7789_YRES
#else
#  error "No display resolutin defined"
#endif

#if defined(CONFIG_LCD_LANDSCAPE) || defined(CONFIG_LCD_RLANDSCAPE)
#  define ST77XX_XRES       CONFIG_LCD_ST77XX_YRES
#  define ST77XX_YRES       CONFIG_LCD_ST77XX_XRES
#  ifdef CONFIG_LCD_ST7789
#    define ST77XX_XOFFSET    CONFIG_LCD_ST7789_YOFFSET
#    define ST77XX_YOFFSET    CONFIG_LCD_ST7789_XOFFSET
#  else
#    define ST77XX_XOFFSET    0
#    define ST77XX_YOFFSET    0
#  endif
#else
#  define ST77XX_XRES       CONFIG_LCD_ST77XX_XRES
#  define ST77XX_YRES       CONFIG_LCD_ST77XX_YRES
#  ifdef CONFIG_LCD_ST7789
#    define ST77XX_XOFFSET    CONFIG_LCD_ST7789_XOFFSET
#    define ST77XX_YOFFSET    CONFIG_LCD_ST7789_YOFFSET
#  else
#    define ST77XX_XOFFSET    0
#    define ST77XX_YOFFSET    0
#  endif
#endif

/* Color depth and format */

#ifdef CONFIG_LCD_ST77XX_BPP
#  if (CONFIG_LCD_ST77XX_BPP == 12)
#    define ST77XX_BPP           12
#    define ST77XX_COLORFMT      FB_FMT_RGB12_444
#    define ST77XX_BYTESPP       2
#  elif (CONFIG_LCD_ST77XX_BPP == 16)
#    define ST77XX_BPP           16
#    define ST77XX_COLORFMT      FB_FMT_RGB16_565
#    define ST77XX_BYTESPP       2
#  else
#    define ST77XX_BPP           16
#    define ST77XX_COLORFMT      FB_FMT_RGB16_565
#    define ST77XX_BYTESPP       2
#    warning "Invalid color depth.  Falling back to 16bpp"
#  endif
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the state of this driver */

struct st77xx_dev_s
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

  uint16_t runbuffer[ST77XX_LUT_SIZE];
};

/****************************************************************************
 * Private Function Protototypes
 ****************************************************************************/

/* Misc. Helpers */

static void st77xx_select(FAR struct spi_dev_s *spi, int bits);
static void st77xx_deselect(FAR struct spi_dev_s *spi);

static inline void st77xx_sendcmd(FAR struct st77xx_dev_s *dev, uint8_t cmd);
static void st77xx_sleep(FAR struct st77xx_dev_s *dev, bool sleep);
static void st77xx_setorientation(FAR struct st77xx_dev_s *dev);
static void st77xx_display(FAR struct st77xx_dev_s *dev, bool on);
static void st77xx_setarea(FAR struct st77xx_dev_s *dev,
                           uint16_t x0, uint16_t y0,
                           uint16_t x1, uint16_t y1);
static void st77xx_bpp(FAR struct st77xx_dev_s *dev, int bpp);
static void st77xx_wrram(FAR struct st77xx_dev_s *dev,
                         FAR const uint16_t *buff, size_t size);
#ifndef CONFIG_LCD_NOGETRUN
static void st77xx_rdram(FAR struct st77xx_dev_s *dev,
                         FAR uint16_t *buff, size_t size);
#endif
static void st77xx_fill(FAR struct st77xx_dev_s *dev, uint16_t color);

/* LCD Data Transfer Methods */

static int st77xx_putrun(fb_coord_t row, fb_coord_t col,
                         FAR const uint8_t *buffer, size_t npixels);
static int st77xx_putarea(fb_coord_t row_start, fb_coord_t row_end,
                          fb_coord_t col_start, fb_coord_t col_end,
                          FAR const uint8_t *buffer);
#ifndef CONFIG_LCD_NOGETRUN
static int st77xx_getrun(fb_coord_t row, fb_coord_t col,
                         FAR uint8_t *buffer, size_t npixels);
#endif

/* LCD Configuration */

static int st77xx_getvideoinfo(FAR struct lcd_dev_s *dev,
                               FAR struct fb_videoinfo_s *vinfo);
static int st77xx_getplaneinfo(FAR struct lcd_dev_s *dev,
                               unsigned int planeno,
                               FAR struct lcd_planeinfo_s *pinfo);

/* LCD Specific Controls */

static int st77xx_getpower(FAR struct lcd_dev_s *dev);
static int st77xx_setpower(FAR struct lcd_dev_s *dev, int power);
static int st77xx_getcontrast(FAR struct lcd_dev_s *dev);
static int st77xx_setcontrast(FAR struct lcd_dev_s *dev,
                              unsigned int contrast);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct st77xx_dev_s g_lcddev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: st77xx_select
 *
 * Description:
 *   Select the SPI, locking and  re-configuring if necessary
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

static void st77xx_select(FAR struct spi_dev_s *spi, int bits)
{
  /* Select ST77XX chip (locking the SPI bus in case there are multiple
   * devices competing for the SPI bus
   */

  SPI_LOCK(spi, true);
  SPI_SELECT(spi, SPIDEV_DISPLAY(0), true);

  /* Now make sure that the SPI bus is configured for the ST77XX (it
   * might have gotten configured for a different device while unlocked)
   */

  SPI_SETMODE(spi, CONFIG_LCD_ST77XX_SPIMODE);
  SPI_SETBITS(spi, bits);
  SPI_SETFREQUENCY(spi, CONFIG_LCD_ST77XX_FREQUENCY);
}

/****************************************************************************
 * Name: st77xx_deselect
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

static void st77xx_deselect(FAR struct spi_dev_s *spi)
{
  /* De-select ST77XX chip and relinquish the SPI bus. */

  SPI_SELECT(spi, SPIDEV_DISPLAY(0), false);
  SPI_LOCK(spi, false);
}

/****************************************************************************
 * Name: st77xx_sendcmd
 *
 * Description:
 *   Send a command to the driver.
 *
 ****************************************************************************/

static inline void st77xx_sendcmd(FAR struct st77xx_dev_s *dev, uint8_t cmd)
{
  st77xx_select(dev->spi, ST77XX_BYTESPP * 8);
  SPI_CMDDATA(dev->spi, SPIDEV_DISPLAY(0), true);
  SPI_SEND(dev->spi, cmd);
  SPI_CMDDATA(dev->spi, SPIDEV_DISPLAY(0), false);
  st77xx_deselect(dev->spi);
}

/****************************************************************************
 * Name: st77xx_sleep
 *
 * Description:
 *   Sleep or wake up the driver.
 *
 ****************************************************************************/

static void st77xx_sleep(FAR struct st77xx_dev_s *dev, bool sleep)
{
  st77xx_sendcmd(dev, sleep ? ST77XX_SLPIN : ST77XX_SLPOUT);
  up_mdelay(120);
}

/****************************************************************************
 * Name: st77xx_display
 *
 * Description:
 *   Turn on or off the display.
 *
 ****************************************************************************/

static void st77xx_display(FAR struct st77xx_dev_s *dev, bool on)
{
  st77xx_sendcmd(dev, on ? ST77XX_DISPON : ST77XX_DISPOFF);

#ifdef CONFIG_LCD_ST7789
  st77xx_sendcmd(dev, ST77XX_INVON);
#endif
}

/****************************************************************************
 * Name: st77xx_setorientation
 *
 * Description:
 *   Set screen orientation.
 *
 ****************************************************************************/

static void st77xx_setorientation(FAR struct st77xx_dev_s *dev)
{
  /* No need to change the orientation in PORTRAIT mode */

#if !defined(CONFIG_LCD_PORTRAIT)
  st77xx_sendcmd(dev, ST77XX_MADCTL);
  st77xx_select(dev->spi, 8);

#  if defined(CONFIG_LCD_RLANDSCAPE)
  /* RLANDSCAPE : MY=1 MV=1 */

  SPI_SEND(dev->spi, 0xa0);

#  elif defined(CONFIG_LCD_LANDSCAPE)
  /* LANDSCAPE : MX=1 MV=1 */

  SPI_SEND(dev->spi, 0x70);

#  elif defined(CONFIG_LCD_RPORTRAIT)
  /* RPORTRAIT : MX=1 MY=1 */

  SPI_SEND(dev->spi, 0xc0);
#  endif

  st77xx_deselect(dev->spi);
#endif
}

/****************************************************************************
 * Name: st77xx_setarea
 *
 * Description:
 *   Set the rectangular area for an upcoming read or write from RAM.
 *
 ****************************************************************************/

static void st77xx_setarea(FAR struct st77xx_dev_s *dev,
                           uint16_t x0, uint16_t y0,
                           uint16_t x1, uint16_t y1)
{
  /* Set row address */

  st77xx_sendcmd(dev, ST77XX_RASET);
  st77xx_select(dev->spi, 16);
  SPI_SEND(dev->spi, y0 + ST77XX_YOFFSET);
  SPI_SEND(dev->spi, y1 + ST77XX_YOFFSET);
  st77xx_deselect(dev->spi);

  /* Set column address */

  st77xx_sendcmd(dev, ST77XX_CASET);
  st77xx_select(dev->spi, 16);
  SPI_SEND(dev->spi, x0 + ST77XX_XOFFSET);
  SPI_SEND(dev->spi, x1 + ST77XX_XOFFSET);
  st77xx_deselect(dev->spi);
}

/****************************************************************************
 * Name: st77xx_bpp
 *
 * Description:
 *   Set the color depth of the device.
 *
 ****************************************************************************/

static void st77xx_bpp(FAR struct st77xx_dev_s *dev, int bpp)
{
  uint8_t depth;

  /* Don't send any command if the depth hasn't changed. */

  if (dev->bpp != bpp)
    {
      /* REVISIT: Works only for 12 and 16 bpp! */

      depth = bpp >> 2 | 1;
      st77xx_sendcmd(dev, ST77XX_COLMOD);
      st77xx_select(dev->spi, 8);
      SPI_SEND(dev->spi, depth);
      st77xx_deselect(dev->spi);

      /* Cache the new BPP */

      dev->bpp = bpp;
    }
}

/****************************************************************************
 * Name: st77xx_wrram
 *
 * Description:
 *   Write to the driver's RAM.
 *
 ****************************************************************************/

static void st77xx_wrram(FAR struct st77xx_dev_s *dev,
                         FAR const uint16_t *buff, size_t size)
{
  st77xx_sendcmd(dev, ST77XX_RAMWR);

  st77xx_select(dev->spi, ST77XX_BYTESPP * 8);
  SPI_SNDBLOCK(dev->spi, buff, size);
  st77xx_deselect(dev->spi);
}

/****************************************************************************
 * Name: st77xx_rdram
 *
 * Description:
 *   Read from the driver's RAM.
 *
 ****************************************************************************/

#ifndef CONFIG_LCD_NOGETRUN
static void st77xx_rdram(FAR struct st77xx_dev_s *dev,
                         FAR uint16_t *buff, size_t size)
{
  st77xx_sendcmd(dev, ST77XX_RAMRD);

  st77xx_select(dev->spi, ST77XX_BYTESPP * 8);
  SPI_RECVBLOCK(dev->spi, buff, size);
  st77xx_deselect(dev->spi);
}
#endif

/****************************************************************************
 * Name: st77xx_fill
 *
 * Description:
 *   Fill the display with the specified color.
 *
 ****************************************************************************/

static void st77xx_fill(FAR struct st77xx_dev_s *dev, uint16_t color)
{
  int i;

  st77xx_setarea(dev, 0, 0, ST77XX_XRES - 1, ST77XX_YRES - 1);

  st77xx_sendcmd(dev, ST77XX_RAMWR);
  st77xx_select(dev->spi, ST77XX_BYTESPP *8);

  for (i = 0; i < ST77XX_XRES * ST77XX_YRES; i++)
    {
      SPI_SEND(dev->spi, color);
    }

  st77xx_deselect(dev->spi);
}

/****************************************************************************
 * Name:  st77xx_putrun
 *
 * Description:
 *   This method can be used to write a partial raster line to the LCD:
 *
 *   row     - Starting row to write to (range: 0 <= row < yres)
 *   col     - Starting column to write to (range: 0 <= col <= xres-npixels)
 *   buffer  - The buffer containing the run to be written to the LCD
 *   npixels - The number of pixels to write to the LCD
 *             (range: 0 < npixels <= xres-col)
 *
 ****************************************************************************/

static int st77xx_putrun(fb_coord_t row, fb_coord_t col,
                         FAR const uint8_t *buffer, size_t npixels)
{
  FAR struct st77xx_dev_s *priv = &g_lcddev;
  FAR const uint16_t *src = (FAR const uint16_t *)buffer;

  ginfo("row: %d col: %d npixels: %d\n", row, col, npixels);
  DEBUGASSERT(buffer && ((uintptr_t)buffer & 1) == 0);

  st77xx_setarea(priv, col, row, col + npixels - 1, row);
  st77xx_wrram(priv, src, npixels);

  return OK;
}

/****************************************************************************
 * Name:  st77xx_putarea
 *
 * Description:
 *   This method can be used to write a partial area to the LCD:
 *
 *   row_start - Starting row to write to (range: 0 <= row < yres)
 *   row_end   - Ending row to write to (range: row_start <= row < yres)
 *   col_start - Starting column to write to (range: 0 <= col <= xres)
 *   col_end   - Ending column to write to
 *               (range: col_start <= col_end < xres)
 *   buffer    - The buffer containing the area to be written to the LCD
 *
 ****************************************************************************/

static int st77xx_putarea(fb_coord_t row_start, fb_coord_t row_end,
                          fb_coord_t col_start, fb_coord_t col_end,
                          FAR const uint8_t *buffer)
{
  FAR struct st77xx_dev_s *priv = &g_lcddev;
  FAR const uint16_t *src = (FAR const uint16_t *)buffer;

  ginfo("row_start: %d row_end: %d col_start: %d col_end: %d\n",
         row_start, row_end, col_start, col_end);

  DEBUGASSERT(buffer && ((uintptr_t)buffer & 1) == 0);

  st77xx_setarea(priv, col_start, row_start, col_end, row_end);
  st77xx_wrram(priv, src,
               (row_end - row_start + 1) * (col_end - col_start + 1));

  return OK;
}

/****************************************************************************
 * Name:  st77xx_getrun
 *
 * Description:
 *   This method can be used to read a partial raster line from the LCD:
 *
 *  row     - Starting row to read from (range: 0 <= row < yres)
 *  col     - Starting column to read read (range: 0 <= col <= xres-npixels)
 *  buffer  - The buffer in which to return the run read from the LCD
 *  npixels - The number of pixels to read from the LCD
 *            (range: 0 < npixels <= xres-col)
 *
 ****************************************************************************/

#ifndef CONFIG_LCD_NOGETRUN
static int st77xx_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t *buffer,
                         size_t npixels)
{
  FAR struct st77xx_dev_s *priv = &g_lcddev;
  FAR uint16_t *dest = (FAR uint16_t *)buffer;

  ginfo("row: %d col: %d npixels: %d\n", row, col, npixels);
  DEBUGASSERT(buffer && ((uintptr_t)buffer & 1) == 0);

  st77xx_setarea(priv, col, row, col + npixels - 1, row);
  st77xx_rdram(priv, dest, npixels);

  return OK;
}
#endif

/****************************************************************************
 * Name:  st77xx_getvideoinfo
 *
 * Description:
 *   Get information about the LCD video controller configuration.
 *
 ****************************************************************************/

static int st77xx_getvideoinfo(FAR struct lcd_dev_s *dev,
                               FAR struct fb_videoinfo_s *vinfo)
{
  DEBUGASSERT(dev && vinfo);
  lcdinfo("fmt: %d xres: %d yres: %d nplanes: 1\n",
          ST77XX_COLORFMT, ST77XX_XRES, ST77XX_YRES);

  vinfo->fmt     = ST77XX_COLORFMT;    /* Color format: RGB16-565: RRRR RGGG GGGB BBBB */
  vinfo->xres    = ST77XX_XRES;        /* Horizontal resolution in pixel columns */
  vinfo->yres    = ST77XX_YRES;        /* Vertical resolution in pixel rows */
  vinfo->nplanes = 1;                  /* Number of color planes supported */
  return OK;
}

/****************************************************************************
 * Name:  st77xx_getplaneinfo
 *
 * Description:
 *   Get information about the configuration of each LCD color plane.
 *
 ****************************************************************************/

static int st77xx_getplaneinfo(FAR struct lcd_dev_s *dev,
                               unsigned int planeno,
                               FAR struct lcd_planeinfo_s *pinfo)
{
  FAR struct st77xx_dev_s *priv = (FAR struct st77xx_dev_s *)dev;

  DEBUGASSERT(dev && pinfo && planeno == 0);
  lcdinfo("planeno: %d bpp: %d\n", planeno, ST77XX_BPP);

  pinfo->putrun = st77xx_putrun;                  /* Put a run into LCD memory */
  pinfo->putarea = st77xx_putarea;                /* Put an area into LCD */
#ifndef CONFIG_LCD_NOGETRUN
  pinfo->getrun = st77xx_getrun;                  /* Get a run from LCD memory */
#endif
  pinfo->buffer = (FAR uint8_t *)priv->runbuffer; /* Run scratch buffer */
  pinfo->bpp    = priv->bpp;                      /* Bits-per-pixel */
  return OK;
}

/****************************************************************************
 * Name:  st77xx_getpower
 ****************************************************************************/

static int st77xx_getpower(FAR struct lcd_dev_s *dev)
{
  FAR struct st77xx_dev_s *priv = (FAR struct st77xx_dev_s *)dev;

  lcdinfo("power: %d\n", priv->power);
  return priv->power;
}

/****************************************************************************
 * Name:  st77xx_setpower
 ****************************************************************************/

static int st77xx_setpower(FAR struct lcd_dev_s *dev, int power)
{
  FAR struct st77xx_dev_s *priv = (FAR struct st77xx_dev_s *)dev;

  lcdinfo("power: %d\n", power);
  DEBUGASSERT((unsigned)power <= CONFIG_LCD_MAXPOWER);

  /* Set new power level */

  if (power > 0)
    {
      /* Turn on the display */

      st77xx_display(priv, true);

      /* Save the power */

      priv->power = power;
    }
  else
    {
      /* Turn off the display */

      st77xx_display(priv, false);

      /* Save the power */

      priv->power = 0;
    }

  return OK;
}

/****************************************************************************
 * Name:  st77xx_getcontrast
 *
 * Description:
 *   Get the current contrast setting (0-CONFIG_LCD_MAXCONTRAST).
 *
 ****************************************************************************/

static int st77xx_getcontrast(FAR struct lcd_dev_s *dev)
{
  lcdinfo("Not implemented\n");
  return -ENOSYS;
}

/****************************************************************************
 * Name:  st77xx_setcontrast
 *
 * Description:
 *   Set LCD panel contrast (0-CONFIG_LCD_MAXCONTRAST).
 *
 ****************************************************************************/

static int st77xx_setcontrast(FAR struct lcd_dev_s *dev,
                              unsigned int contrast)
{
  lcdinfo("contrast: %d\n", contrast);
  return -ENOSYS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  st77xx_initialize
 *
 * Description:
 *   Initialize the ST77XX video hardware.  The initial state of the
 *   LCD is fully initialized, display memory cleared, and the LCD ready
 *   to use, but with the power setting at 0 (full off == sleep mode).
 *
 * Returned Value:
 *
 *   On success, this function returns a reference to the LCD object for
 *   the specified LCD.  NULL is returned on any failure.
 *
 ****************************************************************************/

FAR struct lcd_dev_s *st77xx_lcdinitialize(FAR struct spi_dev_s *spi)
{
  FAR struct st77xx_dev_s *priv = &g_lcddev;

  /* Initialize the driver data structure */

  priv->dev.getvideoinfo = st77xx_getvideoinfo;
  priv->dev.getplaneinfo = st77xx_getplaneinfo;
  priv->dev.getpower     = st77xx_getpower;
  priv->dev.setpower     = st77xx_setpower;
  priv->dev.getcontrast  = st77xx_getcontrast;
  priv->dev.setcontrast  = st77xx_setcontrast;
  priv->spi              = spi;

  /* Init the hardware and clear the display */

  st77xx_sleep(priv, false);
  st77xx_bpp(priv, ST77XX_BPP);
  st77xx_setorientation(priv);
  st77xx_display(priv, true);
  st77xx_fill(priv, 0xffff);

  return &priv->dev;
}

#endif /* CONFIG_LCD_ST77XX */
