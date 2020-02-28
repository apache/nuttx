/****************************************************************************
 * drivers/lcd/max7219.c
 * Driver for the Maxim MAX7219 used for driver 8x8 LED display chains.
 *
 *   Copyright (C) 2017 Alan Carvalho de Assis. All rights reserved.
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/spi/spi.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/max7219.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/
/* MAX7219 Configuration Settings:
 *
 * CONFIG_MAX7219_NHORIZONTALBLKS - Specifies the number of physical
 *   MAX7219 devices that are connected together horizontally.
 *
 * CONFIG_MAX7219_NVERTICALBLKS - Specifies the number of physical
 *   MAX7219 devices that are connected together vertically.
 *
 * CONFIG_LCD_INTENSITY - Defines the default bright of LEDs.
 *
 * Required LCD driver settings:
 * CONFIG_LCD_MAX7219 - Enable MAX7219 support
 *
 */

/* Verify that all configuration requirements have been met */

/* SPI frequency */

#ifndef CONFIG_MAX7219_FREQUENCY
#  define CONFIG_MAX7219_FREQUENCY 10000000
#endif

/* MAX7219_NHORIZONTALBLKS determines the number of physical 8x8 LEDs
 * matrices that are used connected horizontally.
 */

#ifndef CONFIG_MAX7219_NHORIZONTALBLKS
#  define CONFIG_MAX7219_NHORIZONTALBLKS 1
#endif

/* MAX7219_NVERTICALBLKS determines the number of physical 8x8 LEDs
 * matrices that are used connected vertically.
 */

#ifndef CONFIG_MAX7219_NVERTICALBLKS
#  define CONFIG_MAX7219_NVERTICALBLKS 1
#endif

#if CONFIG_LCD_MAXCONTRAST > 15
#  undef CONFIG_LCD_MAXCONTRAST
#  define CONFIG_LCD_MAXCONTRAST 15
#endif

/* Color Properties *********************************************************/
/* The MAX7219 chip can handle resolution of 8x8, 16x8, 8x16, 16x16, 24x8,
 * etc.
 */

/* Display Resolution */

#define MAX7219_XRES         (8 * CONFIG_MAX7219_NHORIZONTALBLKS)
#define MAX7219_YRES         (8 * CONFIG_MAX7219_NVERTICALBLKS)

/* Color depth and format */

#define MAX7219_BPP          1
#define MAX7219_COLORFMT     FB_FMT_Y1

/* Bytes per logical row and actual device row */

#define MAX7219_XSTRIDE      (MAX7219_XRES >> 3) /* Pixels arrange "horizontally for user" */
#define MAX7219_YSTRIDE      (MAX7219_YRES >> 3) /* Pixels arrange "vertically for user" */

/* The size of the shadow frame buffer */

#define MAX7219_FBSIZE       (MAX7219_XRES * MAX7219_YSTRIDE) + 1

/* Bit helpers */

#define LS_BIT               (1 << 0)
#define MS_BIT               (1 << 7)

#define BIT(nr)              (1 << (nr))
#define BITS_PER_BYTE         8
#define BIT_MASK(nr)         (1 << ((nr) % BITS_PER_BYTE))
#define BIT_BYTE(nr)         ((nr) / BITS_PER_BYTE)

/****************************************************************************
 * Private Type Definition
 ****************************************************************************/

/* This structure describes the state of this driver */

struct max7219_dev_s
{
  /* Publicly visible device structure */

  struct lcd_dev_s dev;

  /* Private LCD-specific information follows */

  FAR struct spi_dev_s *spi;
  uint8_t contrast;
  uint8_t powered;

  /* The MAX7219 does not support reading from the display memory in SPI mode.
   * Since there is 1 BPP and access is byte-by-byte, it is necessary to keep
   * a shadow copy of the framebuffer memory.
   */

  uint8_t fb[MAX7219_FBSIZE];
};

/****************************************************************************
 * Private Function Protototypes
 ****************************************************************************/

/* SPI helpers */

static void max7219_select(FAR struct spi_dev_s *spi);
static void max7219_deselect(FAR struct spi_dev_s *spi);

/* LCD Data Transfer Methods */

static int max7219_putrun(fb_coord_t row, fb_coord_t col,
             FAR const uint8_t *buffer, size_t npixels);
static int max7219_getrun(fb_coord_t row, fb_coord_t col,
             FAR uint8_t *buffer, size_t npixels);

/* LCD Configuration */

static int max7219_getvideoinfo(FAR struct lcd_dev_s *dev,
             FAR struct fb_videoinfo_s *vinfo);
static int max7219_getplaneinfo(FAR struct lcd_dev_s *dev,
             unsigned int planeno, FAR struct lcd_planeinfo_s *pinfo);

/* LCD RGB Mapping */

#ifdef CONFIG_FB_CMAP
#  error "RGB color mapping not supported by this driver"
#endif

/* Cursor Controls */

#ifdef CONFIG_FB_HWCURSOR
#  error "Cursor control not supported by this driver"
#endif

/* LCD Specific Controls */

static int max7219_getpower(FAR struct lcd_dev_s *dev);
static int max7219_setpower(FAR struct lcd_dev_s *dev, int power);
static int max7219_getcontrast(FAR struct lcd_dev_s *dev);
static int max7219_setcontrast(FAR struct lcd_dev_s *dev,
                               unsigned int contrast);

/* Initialization */

static inline void up_clear(FAR struct max7219_dev_s  *priv);

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

static uint8_t g_runbuffer[MAX7219_XSTRIDE + 1];

/* This structure describes the overall LCD video controller */

static const struct fb_videoinfo_s g_videoinfo =
{
  MAX7219_COLORFMT,    /* Color format: RGB16-565: RRRR RGGG GGGB BBBB */
  MAX7219_XRES,        /* Horizontal resolution in pixel columns */
  MAX7219_YRES,        /* Vertical resolution in pixel rows */
  1,                   /* Number of color planes supported */
};

/* This is the standard, NuttX Plane information object */

static const struct lcd_planeinfo_s g_planeinfo =
{
  max7219_putrun,              /* Put a run into LCD memory */
  max7219_getrun,              /* Get a run from LCD memory */
  (FAR uint8_t *)g_runbuffer,  /* Run scratch buffer */
  MAX7219_BPP,                 /* Bits-per-pixel */
};

/* This is the standard, NuttX LCD driver object */

static struct max7219_dev_s g_max7219dev =
{
  /* struct lcd_dev_s */
  {
    /* LCD Configuration */

    max7219_getvideoinfo,
    max7219_getplaneinfo,

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

    max7219_getpower,
    max7219_setpower,
    max7219_getcontrast,
    max7219_setcontrast,
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * __set_bit - Set a bit in memory
 *
 *   nr   - The bit to set
 *   addr - The address to start counting from
 *
 * Unlike set_bit(), this function is non-atomic and may be reordered.
 * If it's called on the same region of memory simultaneously, the effect
 * may be that only one operation succeeds.
 *
 ****************************************************************************/

static inline void __set_bit(int nr, uint8_t * addr)
{
  uint8_t mask = BIT_MASK(nr);
  uint8_t *p = ((uint8_t *) addr) + BIT_BYTE(nr);
  *p |= mask;
}

static inline void __clear_bit(int nr, uint8_t * addr)
{
  uint8_t mask = BIT_MASK(nr);
  uint8_t *p = ((uint8_t *) addr) + BIT_BYTE(nr);
  *p &= ~mask;
}

static inline int __test_bit(int nr, const volatile uint8_t * addr)
{
  return 1 & (addr[BIT_BYTE(nr)] >> (nr & (BITS_PER_BYTE - 1)));
}

/****************************************************************************
 * Name:  max7219_powerstring
 *
 * Description:
 *   Convert the power setting to a string.
 *
 ****************************************************************************/

static inline FAR const char *max7219_powerstring(uint8_t power)
{
  if (power == MAX7219_POWER_OFF)
    {
      return "OFF";
    }
  else if (power == MAX7219_POWER_ON)
    {
      return "ON";
    }
  else
    {
      return "ERROR";
    }
}

/****************************************************************************
 * Name: max7219_select
 *
 * Description:
 *   Select the SPI, locking and  re-configuring if necessary
 *
 * Input Parameters:
 *   spi - Reference to the SPI driver structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void max7219_select(FAR struct spi_dev_s *spi)
{
  /* Select MAX7219 chip (locking the SPI bus in case there are multiple
   * devices competing for the SPI bus
   */

  SPI_LOCK(spi, true);
  SPI_SELECT(spi, SPIDEV_DISPLAY(0), true);

  /* Now make sure that the SPI bus is configured for the MAX7219 (it
   * might have gotten configured for a different device while unlocked)
   */

  SPI_SETMODE(spi, SPIDEV_MODE0);
  SPI_SETBITS(spi, 8);
  SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, CONFIG_MAX7219_FREQUENCY);
}

/****************************************************************************
 * Name: max7219_deselect
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
 ****************************************************************************/

static void max7219_deselect(FAR struct spi_dev_s *spi)
{
  /* De-select MAX7219 chip and relinquish the SPI bus. */

  SPI_SELECT(spi, SPIDEV_DISPLAY(0), false);
  SPI_LOCK(spi, false);
}

/****************************************************************************
 * Name:  max7219_putrun
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

static int max7219_putrun(fb_coord_t row, fb_coord_t col,
                          FAR const uint8_t *buffer, size_t npixels)
{
  /* Because of this line of code, we will only be able to support a single
   * MAX7219 device .
   */

  FAR struct max7219_dev_s *priv = &g_max7219dev;
  FAR uint8_t *fbptr;
  FAR uint8_t *ptr;
  uint16_t data;
  uint8_t usrmask;
  int i;
  int pixlen;
  int newrow;

  ginfo("row: %d col: %d npixels: %d\n", row, col, npixels);
  DEBUGASSERT(buffer);

  /* Clip the run to the display */

  pixlen = npixels;
  if ((unsigned int)col + (unsigned int)pixlen > (unsigned int)MAX7219_XRES)
    {
      pixlen = (int)MAX7219_XRES - (int)col;
    }

  /* Verify that some portion of the run remains on the display */

  if (pixlen <= 0 || row > MAX7219_YRES)
    {
      return OK;
    }

  /* Get real row position in the strip */

  newrow = (int) (row % 8);

  /* Divide row by 8 */

  row = (row >> 3);

  col = col + (row * MAX7219_XRES);

  row = newrow;

#ifdef CONFIG_LCD_PACKEDMSFIRST
  usrmask = MS_BIT;
#else
  usrmask = LS_BIT;
#endif

#ifdef CONFIG_LCD_PACKEDMSFIRST
  usrmask = MS_BIT;
#else
  usrmask = LS_BIT;
#endif

  fbptr = &priv->fb[row * MAX7219_XSTRIDE * MAX7219_YSTRIDE];
  ptr = fbptr + (col >> 3);

  for (i = 0; i < pixlen; i++)
    {
      if ((*buffer & usrmask) != 0)
        {
          __set_bit(col % 8 + i, ptr);
        }
      else
        {
          __clear_bit(col % 8 + i, ptr);
        }

#ifdef CONFIG_LCD_PACKEDMSFIRST
      if (usrmask == LS_BIT)
        {
          buffer++;
          usrmask = MS_BIT;
        }
      else
        {
          usrmask >>= 1;
        }
#else
      if (usrmask == MS_BIT)
        {
          buffer++;
          usrmask = LS_BIT;
        }
      else
        {
          usrmask <<= 1;
        }
#endif
    }

  /* Lock and select the device */

  max7219_select(priv->spi);

  /* We need to send last row/column first to avoid mirror image */

  for (i = (MAX7219_XSTRIDE * MAX7219_YSTRIDE) - 1; i >= 0; i--)
    {
      /* Setup the row data */

      data = (8 - row) | (*(fbptr + i) << 8);

      /* Then transfer all 8 columns of data */

      SPI_SNDBLOCK(priv->spi, &data, 2);
    }

  /* Unlock */

  max7219_deselect(priv->spi);

  return OK;
}

/****************************************************************************
 * Name:  max7219_getrun
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

static int max7219_getrun(fb_coord_t row, fb_coord_t col,
                          FAR uint8_t *buffer, size_t npixels)
{
  /* Because of this line of code, we will only be able to support a single
   * MAX7219 device.
   */

  FAR struct max7219_dev_s *priv = &g_max7219dev;
  FAR uint8_t *fbptr;
  FAR uint8_t *ptr;
  uint8_t usrmask;
  int     i;
  int     pixlen;

  ginfo("row: %d col: %d npixels: %d\n", row, col, npixels);
  DEBUGASSERT(buffer);

  /* Clip the run to the display */

  pixlen = npixels;
  if ((unsigned int)col + (unsigned int)pixlen > (unsigned int)MAX7219_XRES)
    {
      pixlen = (int)MAX7219_XRES - (int)col;
    }

  /* Verify that some portion of the run is actually the display */

  if (pixlen <= 0 || row > MAX7219_YRES)
    {
      return -EINVAL;
    }

#ifdef CONFIG_LCD_PACKEDMSFIRST
  usrmask = MS_BIT;
#else
  usrmask = LS_BIT;
#endif

  fbptr = &priv->fb[row * MAX7219_XSTRIDE];
  ptr = fbptr + (col >> 3);

  for (i = 0; i < pixlen; i++)
    {
      if (__test_bit(col % 8 + i, ptr))
        {
          *buffer |= usrmask;
        }
      else
        {
          *buffer &= ~usrmask;
        }

#ifdef CONFIG_LCD_PACKEDMSFIRST
      if (usrmask == LS_BIT)
        {
          buffer++;
          usrmask = MS_BIT;
        }
      else
        {
          usrmask >>= 1;
        }
#else
      if (usrmask == MS_BIT)
        {
          buffer++;
          usrmask = LS_BIT;
        }
      else
        {
          usrmask <<= 1;
        }
#endif
    }

  return OK;
}

/****************************************************************************
 * Name:  max7219_getvideoinfo
 *
 * Description:
 *   Get information about the LCD video controller configuration.
 *
 ****************************************************************************/

static int max7219_getvideoinfo(FAR struct lcd_dev_s *dev,
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
 * Name:  max7219_getplaneinfo
 *
 * Description:
 *   Get information about the configuration of each LCD color plane.
 *
 ****************************************************************************/

static int max7219_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int planeno,
                              FAR struct lcd_planeinfo_s *pinfo)
{
  DEBUGASSERT(dev && pinfo && planeno == 0);

  ginfo("planeno: %d bpp: %d\n", planeno, g_planeinfo.bpp);

  memcpy(pinfo, &g_planeinfo, sizeof(struct lcd_planeinfo_s));
  return OK;
}

/****************************************************************************
 * Name:  max7219_getpower
 *
 * Description:
 *   Get the LCD panel power status (0: full off - CONFIG_LCD_MAXPOWER: full
 *   on). On backlit LCDs, this setting may correspond to the backlight
 *   setting.
 *
 ****************************************************************************/

static int max7219_getpower(struct lcd_dev_s *dev)
{
  struct max7219_dev_s *priv = (struct max7219_dev_s *)dev;

  DEBUGASSERT(priv);
  ginfo("powered: %s\n", max7219_powerstring(priv->powered));

  return priv->powered;
}

/****************************************************************************
 * Name:  max7219_setpower
 *
 * Description:
 *   Enable/disable LCD panel power (0: full off - CONFIG_LCD_MAXPOWER: full on). On
 *   backlit LCDs, this setting may correspond to the backlight setting.
 *
 ****************************************************************************/

static int max7219_setpower(struct lcd_dev_s *dev, int power)
{
  struct max7219_dev_s *priv = (struct max7219_dev_s *)dev;
  uint16_t data;

  DEBUGASSERT(priv && (unsigned)power <= CONFIG_LCD_MAXPOWER);
  ginfo("power: %s powered: %s\n",
        max7219_powerstring(power), max7219_powerstring(priv->powered));

  /* Select and lock the device */

  max7219_select(priv->spi);

  if (power <= MAX7219_POWER_OFF)
    {
      data = (MAX7219_SHUTDOWN) | (MAX7219_POWER_OFF << 8);

      /* Turn the display off (power-down) */

      SPI_SNDBLOCK(priv->spi, &data, 2);

      priv->powered = MAX7219_POWER_OFF;
    }
  else
    {
      data = (MAX7219_SHUTDOWN) | (MAX7219_POWER_ON << 8);

      /* Leave the power-down */

      SPI_SNDBLOCK(priv->spi, &data, 2);

      priv->powered = MAX7219_POWER_ON;
    }

  /* Let go of the SPI lock and de-select the device */

  max7219_deselect(priv->spi);
  return OK;
}

/****************************************************************************
 * Name:  max7219_getcontrast
 *
 * Description:
 *   Get the current contrast setting (0-CONFIG_LCD_MAXCONTRAST).
 *
 ****************************************************************************/

static int max7219_getcontrast(struct lcd_dev_s *dev)
{
  struct max7219_dev_s *priv = (struct max7219_dev_s *)dev;

  DEBUGASSERT(priv);
  return (int)priv->contrast;
}

/****************************************************************************
 * Name:  max7219_setcontrast
 *
 * Description:
 *   Set LCD panel contrast (0-CONFIG_LCD_MAXCONTRAST).
 *
 ****************************************************************************/

static int max7219_setcontrast(struct lcd_dev_s *dev, unsigned int contrast)
{
  struct max7219_dev_s *priv = (struct max7219_dev_s *)dev;
  uint16_t data;

  ginfo("contrast: %d\n", contrast);
  DEBUGASSERT(priv);

  if (contrast > 15)
    {
      return -EINVAL;
    }

  /* Save the contrast */

  priv->contrast = contrast;

  /* Select and lock the device */

  max7219_select(priv->spi);

  /* Configure the contrast/intensity */

  data = (MAX7219_INTENSITY) | (DISPLAY_INTENSITY(contrast) << 8);

  /* Set the contrast */

  SPI_SNDBLOCK(priv->spi, &data, 2);

  /* Let go of the SPI lock and de-select the device */

  max7219_deselect(priv->spi);
  return OK;
}

/****************************************************************************
 * Name:  up_clear
 *
 * Description:
 *   Clear the display.
 *
 ****************************************************************************/

static inline void up_clear(FAR struct max7219_dev_s  *priv)
{
  FAR struct spi_dev_s *spi  = priv->spi;
  uint16_t data;
  int row;
  int i;

  /* Clear the framebuffer */

  memset(priv->fb, MAX7219_BLACK, MAX7219_FBSIZE);

  /* Go throw max7219 all 8 rows */

  for (row = 0, i = 0; i < 8; i++)
    {
      /* Select and lock the device */

      max7219_select(priv->spi);

      /* Setup the row data */

      data = (row + 1) | (priv->fb[row] << 8);

      /* Then transfer all 8 columns of data */

      SPI_SNDBLOCK(priv->spi, &data, 2);

      /* Unlock and de-select the device */

      max7219_deselect(spi);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: max7219_initialize
 *
 * Description:
 *   Initialize the MAX7219 device as a LCD interface.
 *
 * Input Parameters:
 *   spi   - An instance of the SPI interface to use to communicate
 *           with the MAX7219.
 *   devno - Device number to identify current display.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

FAR struct lcd_dev_s *max7219_initialize(FAR struct spi_dev_s *spi,
                                         unsigned int devno)
{
  /* Configure and enable LCD */

  FAR struct max7219_dev_s  *priv = &g_max7219dev;
  uint16_t data;

  ginfo("Initializing\n");
  DEBUGASSERT(spi && devno == 0);

  /* Save the reference to the SPI device */

  priv->spi = spi;

  /* Select and lock the device */

  max7219_select(spi);

  /* Leave the shutdown mode */

  data = (MAX7219_SHUTDOWN) | (MAX7219_POWER_ON << 8);

  SPI_SNDBLOCK(priv->spi, &data, 2);

  max7219_deselect(spi);

  max7219_select(spi);

  /* Disable 7 segment decoding */

  data = (MAX7219_DECODE_MODE) | (DISABLE_DECODE << 8);

  SPI_SNDBLOCK(priv->spi, &data, 2);

  max7219_deselect(spi);

  max7219_select(spi);

  /* Set scan limit for all digits */

  data = (MAX7219_SCAN_LIMIT) | (DEFAULT_SCAN_LIMIT << 8);

  SPI_SNDBLOCK(priv->spi, &data, 2);

  max7219_deselect(spi);

  max7219_select(spi);

  /* Set intensity level configured by the user */

  data = (MAX7219_INTENSITY) | (DISPLAY_INTENSITY(CONFIG_LCD_MAXCONTRAST) << 8);

  SPI_SNDBLOCK(priv->spi, &data, 2);

  /* Let go of the SPI lock and de-select the device */

  max7219_deselect(spi);

  /* Clear the framebuffer */

  up_clear(priv);
  return &priv->dev;
}
