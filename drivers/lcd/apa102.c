/****************************************************************************
 * drivers/lcd/apa102.c
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

/* Driver to create a display using RBG LEDs APA102 chained together */

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
#include <nuttx/lcd/apa102.h>
#include <nuttx/leds/apa102.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef MAX
#  define MAX(a,b)    ((a) > (b) ? (a) : (b))
#endif

/* Configuration ************************************************************/

/* APA102 Configuration Settings:
 *
 * CONFIG_APA102_XRES - Specifies the number of physical
 *   APA102 devices that are connected together horizontally.
 *
 * CONFIG_APA102_YRES - Specifies the number of physical
 *   APA102 devices that are connected together vertically.
 *
 * CONFIG_LCD_INTENSITY - Defines the default bright of LEDs.
 *
 * Required LCD driver settings:
 * CONFIG_LCD_APA102 - Enable APA102 support
 *
 */

/* Verify that all configuration requirements have been met */

/* SPI frequency */

#ifndef CONFIG_APA102_FREQUENCY
#  define CONFIG_APA102_FREQUENCY 10000000
#endif

/* APA102_COLUMNS determines the number of physical LEDs
 * matrices that are used connected horizontally.
 */

#ifndef CONFIG_APA102_XRES
#  define CONFIG_APA102_XRES 16
#endif

/* APA102_LINES determines the number of physical LEDs
 * matrices that are used connected vertically.
 */

#ifndef CONFIG_APA102_YRES
#  define CONFIG_APA102_YRES 16
#endif

/* Check contrast selection */

#if !defined(CONFIG_LCD_MAXCONTRAST)
#  define CONFIG_LCD_MAXCONTRAST 1
#endif

/* Color Properties *********************************************************/

/* Display Resolution */

#define APA102_XRES         CONFIG_APA102_XRES
#define APA102_YRES         CONFIG_APA102_YRES

/* Color depth and format */

#define APA102_BPP          16
#define APA102_COLORFMT     FB_FMT_RGB16_565

#define APA102_LUT_SIZE     MAX(APA102_XRES, APA102_YRES)

/* The size of the shadow frame buffer (4 bytes per LED) */

#define APA102_FBSIZE       (APA102_XRES * APA102_YRES)

/* LCD RGB Mapping */

#ifdef CONFIG_FB_CMAP
#  error "RGB color mapping not supported by this driver"
#endif

/* Cursor Controls */

#ifdef CONFIG_FB_HWCURSOR
#  error "Cursor control not supported by this driver"
#endif

/****************************************************************************
 * Private Type Definition
 ****************************************************************************/

/* This structure describes the state of this driver */

struct apa102_dev_s
{
  /* Publicly visible device structure */

  struct lcd_dev_s dev;

  /* Private LCD-specific information follows */

  FAR struct spi_dev_s *spi;
  uint8_t contrast;
  uint8_t powered;

  /* We need to send all the APA102 matrix screen once.
   * So, create a framebuffer to save it in memory
   */

  struct apa102_ledstrip_s fb[APA102_FBSIZE];
};

/****************************************************************************
 * Private Function Protototypes
 ****************************************************************************/

/* LCD Data Transfer Methods */

static int apa102_putrun(FAR struct lcd_dev_s *dev, fb_coord_t row,
                         fb_coord_t col, FAR const uint8_t *buffer,
                         size_t npixels);
static int apa102_putarea(FAR struct lcd_dev_s *dev,
                          fb_coord_t row_start, fb_coord_t row_end,
                          fb_coord_t col_start, fb_coord_t col_end,
                          FAR const uint8_t *buffer, fb_coord_t stride);
static int apa102_getrun(FAR struct lcd_dev_s *dev, fb_coord_t row,
                         fb_coord_t col, FAR uint8_t *buffer,
                         size_t npixels);

/* LCD Configuration */

static int apa102_getvideoinfo(FAR struct lcd_dev_s *dev,
                               FAR struct fb_videoinfo_s *vinfo);
static int apa102_getplaneinfo(FAR struct lcd_dev_s *dev,
                               unsigned int planeno,
                               FAR struct lcd_planeinfo_s *pinfo);

/* LCD Specific Controls */

static int apa102_getpower(FAR struct lcd_dev_s *dev);
static int apa102_setpower(FAR struct lcd_dev_s *dev, int power);
static int apa102_getcontrast(FAR struct lcd_dev_s *dev);
static int apa102_setcontrast(FAR struct lcd_dev_s *dev,
                              unsigned int contrast);

/* Initialization */

static inline void up_clear(FAR struct apa102_dev_s  *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is working memory allocated by the LCD driver for each LCD device
 * and for each color plane.  This memory will hold one raster line of data.
 * The size of the allocated run buffer must therefore be at least
 * (bpp * xres / 8).  Actual alignment of the buffer must conform to the
 * bitwidth of the underlying pixel type.
 *
 * If there are multiple planes, they may share the same working buffer
 * because different planes will not be operate on concurrently.  However,
 * if there are multiple LCD devices, they must each have unique run buffers.
 */

static uint8_t g_runbuffer[APA102_LUT_SIZE];

/* This structure describes the overall LCD video controller */

static const struct fb_videoinfo_s g_videoinfo =
{
  APA102_COLORFMT,    /* Color format: RGB16-565: RRRR RGGG GGGB BBBB */
  APA102_XRES,        /* Horizontal resolution in pixel columns */
  APA102_YRES,        /* Vertical resolution in pixel rows */
  1,                  /* Number of color planes supported */
};

/* This is the standard, NuttX Plane information object */

static const struct lcd_planeinfo_s g_planeinfo =
{
  apa102_putrun,              /* Put a run into LCD memory */
  apa102_putarea,             /* Put a run into LCD memory */
  apa102_getrun,              /* Get a run from LCD memory */
  NULL,                       /* No getarea function */
  NULL,                       /* No redraw function */
  (FAR uint8_t *)g_runbuffer, /* Run scratch buffer */
  APA102_BPP,                 /* Bits-per-pixel */
  NULL,                       /* Put lcd_dev_s later */
};

/* This is the standard, NuttX LCD driver object */

static struct apa102_dev_s g_apa102dev =
{
  /* struct lcd_dev_s */

  {
    /* LCD Configuration */

    apa102_getvideoinfo,
    apa102_getplaneinfo,

#ifdef CONFIG_FB_CMAP
    /* LCD RGB Mapping -- Not supported */

    NULL,
    NULL,
#endif

#ifdef CONFIG_FB_HWCURSOR
    /* Cursor Controls -- Not supported */

    NULL,
    NULL,
#endif

    /* LCD Specific Controls */

    apa102_getpower,
    apa102_setpower,
    apa102_getcontrast,
    apa102_setcontrast,
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rgb565_apa102
 *
 * Description:
 *   Convert RGB565 to APA102 RGB888 format
 *
 ****************************************************************************/

static struct apa102_ledstrip_s rgb565_apa102(uint16_t rgb565)
{
  struct apa102_ledstrip_s led;

  led.red    = (rgb565 & 0b1111100000000000) >> 8;
  led.green  = (rgb565 & 0b11111100000) >> 3;
  led.blue   = (rgb565 & 0b11111) << 3;
  led.bright = 0;
  return led;
}

/****************************************************************************
 * Name: apa102_rgb565
 *
 * Description:
 *   Convert APA102 RGB888 to RGB565 format
 *
 ****************************************************************************/

static uint16_t apa102_rgb565(struct apa102_ledstrip_s led)
{
  uint16_t pixel;

  pixel  = (led.red & 0b11111000) << 8;
  pixel |= (led.green & 0b11111100) << 3;
  pixel |= (led.blue & 0b11111000) >> 3;
  return pixel;
}

/****************************************************************************
 * Name: apa102_configspi
 *
 * Description:
 *   Set the SPI bus configuration
 *
 ****************************************************************************/

static inline void apa102_configspi(FAR struct spi_dev_s *spi)
{
  /* Configure SPI for the APA102 */

  SPI_SETMODE(spi, SPIDEV_MODE0);
  SPI_SETBITS(spi, 8);
  SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, APA102_SPI_MAXFREQUENCY);
}

/****************************************************************************
 * Name: apa102_write32
 *
 * Description:
 *   Write 32-bit to APA102
 *
 ****************************************************************************/

static inline void apa102_write32(FAR struct apa102_dev_s *priv,
                                  uint32_t value)
{
  /* If SPI bus is shared then lock and configure it */

  SPI_LOCK(priv->spi, true);

  /* Note: APA102 doesn't use chip select */

  /* Send 32 bits (4 bytes) */

  SPI_SEND(priv->spi, (value & 0xff));
  SPI_SEND(priv->spi, ((value & 0xff00) >> 8));
  SPI_SEND(priv->spi, ((value & 0xff0000) >> 16));
  SPI_SEND(priv->spi, ((value & 0xff000000) >> 24));

  /* Unlock bus */

  SPI_LOCK(priv->spi, false);
}

/****************************************************************************
 * Name: apa102_refresh
 *
 * Description:
 *   Send converted framebuffer to the APA102 LED Matrix
 *
 ****************************************************************************/

static inline void apa102_refresh(FAR struct apa102_dev_s *priv)
{
  int i;

  /* Confirm that SPI is configured correctly */

  apa102_configspi(priv->spi);

  /* Send a start of frame */

  apa102_write32(priv, APA102_START_FRAME);

  /* Send all LEDs values in the matrix */

  for (i = 0; i < (APA102_XRES * APA102_YRES); i++)
    {
      uint32_t *led = (uint32_t *) &priv->fb[i];

      /* Then transfer 4 bytes per LED */

      apa102_write32(priv, (uint32_t) (*led | APA102_HEADER_FRAME));
    }

  /* Send an end of frame */

  apa102_write32(priv, APA102_END_FRAME);

  for (i = 0; i < (1 + APA102_XRES * APA102_YRES / 32); i++)
    {
      apa102_write32(priv, 0);
    }
}

/****************************************************************************
 * Name:  apa102_putrun
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

static int apa102_putrun(FAR struct lcd_dev_s *dev, fb_coord_t row,
                         fb_coord_t col, FAR const uint8_t *buffer,
                         size_t npixels)
{
  /* Because of this line of code, we will only be able to support a single
   * APA102 device .
   */

  FAR struct apa102_dev_s *priv = (FAR struct apa102_dev_s *)dev;
  int i;
  int pixlen;

  ginfo("row: %d col: %d npixels: %d\n", row, col, npixels);
  DEBUGASSERT(buffer && ((uintptr_t)buffer & 1) == 0);

  /* Clip the run to the display */

  pixlen = npixels;
  if ((unsigned int)col + (unsigned int)pixlen > (unsigned int)APA102_XRES)
    {
      pixlen = (int)APA102_XRES - (int)col;
    }

  /* Verify that some portion of the run remains on the display */

  if (pixlen <= 0 || row > APA102_YRES)
    {
      return OK;
    }

  /* Convert rgb565 to APA102 LED values */

  for (i = 0; i < pixlen; i++)
    {
      uint16_t *ptr = (uint16_t *)buffer;

      if (row % 2 == 0)
        {
          priv->fb[(row * APA102_XRES) + col + i] = rgb565_apa102(*ptr++);
        }
      else
        {
          priv->fb[(row * APA102_XRES) + APA102_XRES - col - i - 1] =
            rgb565_apa102(*ptr++);
        }
    }

  /* Update the display with converted data */

  apa102_refresh(priv);

  return OK;
}

/****************************************************************************
 * Name:  apa102_putarea
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

static int apa102_putarea(FAR struct lcd_dev_s *dev,
                          fb_coord_t row_start, fb_coord_t row_end,
                          fb_coord_t col_start, fb_coord_t col_end,
                          FAR const uint8_t *buffer, fb_coord_t stride)
{
  FAR struct apa102_dev_s *priv = (FAR struct apa102_dev_s *)dev;
  FAR uint16_t *src = (FAR uint16_t *)buffer;
  int i;
  int j;

  ginfo("row_start: %d row_end: %d col_start: %d col_end: %d\n",
         row_start, row_end, col_start, col_end);

  DEBUGASSERT(buffer && ((uintptr_t)buffer & 1) == 0);

  /* Convert RGB565 to APA102 LED values */

  for (i = row_start; i <= row_end; i++)
    {
      for (j = col_start; j <= col_end; j++)
        {
          if (i % 2 == 0)
            {
              priv->fb[(i * APA102_XRES) + j] =
                rgb565_apa102(*(src + (i * APA102_XRES) + j));
            }
          else
            {
              priv->fb[(i * APA102_XRES) + APA102_XRES - j - 1] =
                rgb565_apa102(*(src + (i * APA102_XRES) + j));
            }
        }
    }

  /* Update the display with converted data */

  apa102_refresh(priv);

  return OK;
}

/****************************************************************************
 * Name:  apa102_getrun
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

static int apa102_getrun(FAR struct lcd_dev_s *dev, fb_coord_t row,
                         fb_coord_t col, FAR uint8_t *buffer,
                         size_t npixels)
{
  /* Because of this line of code, we will only be able to support a single
   * APA102 device.
   */

  FAR struct apa102_dev_s *priv = (FAR struct apa102_dev_s *)dev;
  int i;
  int pixlen;

  ginfo("row: %d col: %d npixels: %d\n", row, col, npixels);
  DEBUGASSERT(buffer);

  /* Clip the run to the display */

  pixlen = npixels;
  if ((unsigned int)col + (unsigned int)pixlen > (unsigned int)APA102_XRES)
    {
      pixlen = (int)APA102_XRES - (int)col;
    }

  /* Verify that some portion of the run is actually the display */

  if (pixlen <= 0 || row > APA102_YRES)
    {
      return -EINVAL;
    }

  for (i = 0; i < pixlen; i++)
    {
      uint16_t *ptr = (uint16_t *) buffer;

      *ptr++ = apa102_rgb565(priv->fb[(row * APA102_XRES) + col + i]);
    }

  return OK;
}

/****************************************************************************
 * Name:  apa102_getvideoinfo
 *
 * Description:
 *   Get information about the LCD video controller configuration.
 *
 ****************************************************************************/

static int apa102_getvideoinfo(FAR struct lcd_dev_s *dev,
                               FAR struct fb_videoinfo_s *vinfo)
{
  DEBUGASSERT(dev && vinfo);

  ginfo("fmt: %d xres: %d yres: %d nplanes: %d\n",
        g_videoinfo.fmt, g_videoinfo.xres, g_videoinfo.yres,
        g_videoinfo.nplanes);

  memcpy(vinfo, &g_videoinfo, sizeof(struct fb_videoinfo_s));
  return OK;
}

/****************************************************************************
 * Name:  apa102_getplaneinfo
 *
 * Description:
 *   Get information about the configuration of each LCD color plane.
 *
 ****************************************************************************/

static int apa102_getplaneinfo(FAR struct lcd_dev_s *dev,
                               unsigned int planeno,
                               FAR struct lcd_planeinfo_s *pinfo)
{
  DEBUGASSERT(dev && pinfo && planeno == 0);

  ginfo("planeno: %d bpp: %d\n", planeno, g_planeinfo.bpp);

  memcpy(pinfo, &g_planeinfo, sizeof(struct lcd_planeinfo_s));
  pinfo->dev = dev;

  return OK;
}

/****************************************************************************
 * Name:  apa102_getpower
 *
 * Description:
 *   Get the LCD panel power status (0: full off - CONFIG_LCD_MAXPOWER: full
 *   on). On backlit LCDs, this setting may correspond to the backlight
 *   setting.
 *
 ****************************************************************************/

static int apa102_getpower(FAR struct lcd_dev_s *dev)
{
  struct apa102_dev_s *priv = (FAR struct apa102_dev_s *)dev;

  DEBUGASSERT(priv);
  ginfo("powered: %d\n", priv->powered);

  return priv->powered;
}

/****************************************************************************
 * Name:  apa102_setpower
 *
 * Description:
 *   Enable/disable LCD panel power (0: full off - CONFIG_LCD_MAXPOWER: full
 *   on). On backlit LCDs, this setting may correspond to the backlight
 *   setting.
 *
 ****************************************************************************/

static int apa102_setpower(FAR struct lcd_dev_s *dev, int power)
{
  return OK;
}

/****************************************************************************
 * Name:  apa102_getcontrast
 *
 * Description:
 *   Get the current contrast setting (0-CONFIG_LCD_MAXCONTRAST).
 *
 ****************************************************************************/

static int apa102_getcontrast(FAR struct lcd_dev_s *dev)
{
  struct apa102_dev_s *priv = (FAR struct apa102_dev_s *)dev;

  DEBUGASSERT(priv);
  return (int)priv->contrast;
}

/****************************************************************************
 * Name:  apa102_setcontrast
 *
 * Description:
 *   Set LCD panel contrast (0-CONFIG_LCD_MAXCONTRAST).
 *
 ****************************************************************************/

static int apa102_setcontrast(FAR struct lcd_dev_s *dev,
                              unsigned int contrast)
{
  return OK;
}

/****************************************************************************
 * Name:  up_clear
 *
 * Description:
 *   Clear the display.
 *
 ****************************************************************************/

static inline void up_clear(FAR struct apa102_dev_s  *priv)
{
  int i;

  /* Clear the framebuffer */

  memset(priv->fb, APA102_BLACK, 4 * APA102_FBSIZE);

  /* Update the display with framebuffer data */

  apa102_refresh(priv);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: apa102_initialize
 *
 * Description:
 *   Initialize the APA102 device as a LCD interface.
 *
 * Input Parameters:
 *   spi   - An instance of the SPI interface to use to communicate
 *           with the APA102.
 *   devno - Device number to identify current display.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

FAR struct lcd_dev_s *apa102_initialize(FAR struct spi_dev_s *spi,
                                        unsigned int devno)
{
  /* Configure and enable LCD */

  FAR struct apa102_dev_s *priv = &g_apa102dev;

  ginfo("Initializing\n");
  DEBUGASSERT(spi && devno == 0);

  /* Save the reference to the SPI device */

  priv->spi = spi;

  /* Clear the framebuffer */

  up_clear(priv);
  return &priv->dev;
}
