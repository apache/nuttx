/**************************************************************************************
 * drivers/lcd/ssd1306.c
 * Driver for Univision UG-2864HSWEG01 OLED display or UG-2832HSWEG04 both with the
 * Univision SSD1306 controller in SPI mode
 *
 *   Copyright (C) 2012-2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *   1. Product Specification (Preliminary), Part Name: OEL Display Module, Part ID:
 *      UG-2864HSWEG01, Doc No: SAS1-9046-B, Univision Technology Inc.
 *   2. Product Specification, Part Name: OEL Display Module, Part ID: UG-2832HSWEG04,
 *      Doc No.: SAS1-B020-B, Univision Technology Inc.
 *   3. SSD1306, 128 X 64 Dot Matrix OLED/PLED, Preliminary Segment/Common Driver with
 *      Controller,  Solomon Systech
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
 **************************************************************************************/
/**************************************************************************************
 * Device memory organization:
 *
 *          +----------------------------+
 *          |           Column           |
 *  --------+----+---+---+---+-...-+-----+
 *  Page    | 0  | 1 | 2 | 3 | ... | 127 |
 *  --------+----+---+---+---+-...-+-----+
 *  Page 0  | D0 | X |   |   |     |     |
 *          | D1 | X |   |   |     |     |
 *          | D2 | X |   |   |     |     |
 *          | D3 | X |   |   |     |     |
 *          | D4 | X |   |   |     |     |
 *          | D5 | X |   |   |     |     |
 *          | D6 | X |   |   |     |     |
 *          | D7 | X |   |   |     |     |
 *  --------+----+---+---+---+-...-+-----+
 *  Page 1  |    |   |   |   |     |     |
 *  --------+----+---+---+---+-...-+-----+
 *  Page 2  |    |   |   |   |     |     |
 *  --------+----+---+---+---+-...-+-----+
 *  Page 3  |    |   |   |   |     |     |
 *  --------+----+---+---+---+-...-+-----+
 *  Page 4  |    |   |   |   |     |     |
 *  --------+----+---+---+---+-...-+-----+
 *  Page 5  |    |   |   |   |     |     |
 *  --------+----+---+---+---+-...-+-----+
 *  Page 6  |    |   |   |   |     |     |
 *  --------+----+---+---+---+-...-+-----+
 *  Page 7  |    |   |   |   |     |     |
 *  --------+----+---+---+---+-...-+-----+
 *
 *  -----------------------------------+---------------------------------------
 *  Landscape Display:                 | Reverse Landscape Display:
 *  --------+-----------------------+  |  --------+---------------------------+
 *          |       Column          |  |          |         Column            |
 *  --------+---+---+---+-...-+-----+  |  --------+-----+-----+-----+-...-+---+
 *  Page 0  | 0 | 1 | 2 |     | 127 |  |  Page 7  | 127 | 126 | 125 |     | 0 |
 *  --------+---+---+---+-...-+-----+  |  --------+-----+-----+-----+-...-+---+
 *  Page 1  | V                     |  |  Page 6  |                         ^ |
 *  --------+---+---+---+-...-+-----+  |  --------+-----+-----+-----+-...-+---+
 *  Page 2  | V                     |  |  Page 5  |                         ^ |
 *  --------+---+---+---+-...-+-----+  |  --------+-----+-----+-----+-...-+---+
 *  Page 3  | V                     |  |  Page 4  |                         ^ |
 *  --------+---+---+---+-...-+-----+  |  --------+-----+-----+-----+-...-+---+
 *  Page 4  | V                     |  |  Page 3  |                         ^ |
 *  --------+---+---+---+-...-+-----+  |  --------+-----+-----+-----+-...-+---+
 *  Page 5  | V                     |  |  Page 2  |                         ^ |
 *  --------+---+---+---+-...-+-----+  |  --------+-----+-----+-----+-...-+---+
 *  Page 6  | V                     |  |  Page 1  |                         ^ |
 *  --------+---+---+---+-...-+-----+  |  --------+-----+-----+-----+-...-+---+
 *  Page 7  | V                     |  |  Page 0  |                         ^ |
 *  --------+---+---+---+-...-+-----+  |  --------+-----+-----+-----+-...-+---+
 *  -----------------------------------+---------------------------------------
 *
 *  -----------------------------------+---------------------------------------
 *  Portrait Display:                  | Reverse Portrait Display:
 *  -----------+---------------------+ |  -----------+---------------------+
 *             |         Page        | |             |       Page          |
 *  -----------+---+---+---+-...-+---+ |  -----------+---+---+---+-...-+---+
 *  Column 0   | 0 | 1 | 2 |     | 7 | |  Column 127 | 7 | 6 | 5 |     | 0 |
 *  -----------+---+---+---+-...-+---+ |  -----------+---+---+---+-...-+---+
 *  Column 1   | >   >   >    >    > | |  Column 126 |                     |
 *  -----------+---+---+---+-...-+---+ |  -----------+---+---+---+-...-+---+
 *  Column 2   |                     | |  Column 125 |                     |
 *  -----------+---+---+---+-...-+---+ |  -----------+---+---+---+-...-+---+
 *  ...        |                     | |  ...        |                     |
 *  -----------+---+---+---+-...-+---+ |  -----------+---+---+---+-...-+---+
 *  Column 127 |                     | |  Column 0   | <   <   <    <    < |
 *  -----------+---+---+---+-...-+---+ |  -----------+---+---+---+-...-+---+
 *  -----------------------------------+----------------------------------------
 **************************************************************************************/

/**************************************************************************************
 * Included Files
 **************************************************************************************/

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
#include <nuttx/lcd/ssd1306.h>

#include <arch/irq.h>

#ifdef CONFIG_LCD_SSD1306

/**************************************************************************************
 * Pre-processor Definitions
 **************************************************************************************/
/* Configuration **********************************************************************/
/* Limitations of the current configuration that I hope to fix someday */

#ifndef CONFIG_SSD1306_NINTERFACES
#  define CONFIG_SSD1306_NINTERFACES 1
#endif

#if CONFIG_SSD1306_NINTERFACES != 1
#  warning "This implementation supports only a single SSD1306 device"
#  undef CONFIG_SSD1306_NINTERFACES
#  define CONFIG_SSD1306_NINTERFACES 1
#endif

#if !defined(CONFIG_LCD_UG2864HSWEG01) && !defined(CONFIG_LCD_UG2832HSWEG04)
#  error "Unknown and unsupported SSD1306 LCD"
#endif

#if defined(CONFIG_LCD_PORTRAIT) || defined(CONFIG_LCD_RPORTRAIT)
#  warning "No support for portrait modes"
#  undef CONFIG_LCD_LANDSCAPE
#  define CONFIG_LCD_LANDSCAPE 1
#  undef CONFIG_LCD_PORTRAIT
#  undef CONFIG_LCD_RLANDSCAPE
#  undef CONFIG_LCD_RPORTRAIT
#endif

/* SSD1306 Commands *******************************************************************/

#define SSD1306_SETCOLL(ad)      (0x00 | ((ad) & 0x0f)) /* Set Lower Column Address: (00h - 0fh) */
#define SSD1306_SETCOLH(ad)      (0x10 | ((ad) & 0x0f)) /* Set Higher Column Address: (10h - 1fh) */
#define SSD1306_STARTLINE(ln)    (0x40 | ((ln) & 0x3f)) /* Set Display Start Line: (40h - 7fh) */
#define SSD1306_CONTRAST_MODE    (0x81)                 /* Set Contrast Control Register: (Double Bytes Command) */
#  define SSD1306_CONTRAST(c)    (c)
#define SSD1306_SEGREMAP(m)      (0xa0 | ((m) & 0x01))  /* Set Segment Re-map: (a0h - a1h) */
#  define SSD1306_REMAPRIGHT     SSD1306_SEGREMAP(0)    /*   Right rotation */
#  define SSD1306_REMAPPLEFT     SSD1306_SEGREMAP(1)    /*   Left rotation */
#define SSD1306_EDISPOFFON(s)    (0xa4 | ((s) & 0x01))  /* Set Entire Display OFF/ON: (a4h - a5h) */
#  define SSD1306_EDISPOFF       SSD1306_EDISPOFFON(0)  /*   Display off */
#  define SSD1306_EDISPON        SSD1306_EDISPOFFON(1)  /*   Display on */
#define SSD1306_NORMREV(s)       (0xa6 | ((s) & 0x01))  /* Set Normal/Reverse Display: (a6h -a7h) */
#  define SSD1306_NORMAL         SSD1306_NORMREV(0)     /*   Normal display */
#  define SSD1306_REVERSE        SSD1306_NORMREV(1)     /*   Reverse display */
#define SSD1306_MRATIO_MODE      (0xa8)                 /* Set Multiplex Ration: (Double Bytes Command) */
#  define SSD1306_MRATIO(d)      ((d) & 0x3f)
#define SSD1306_DCDC_MODE        (0xad)                 /* Set DC-DC OFF/ON: (Double Bytes Command) */
#  define SSD1306_DCDC_OFF       (0x8a)
#  define SSD1306_DCDC_ON        (0x8b)

#define SSD1306_DISPOFFON(s)     (0xae | ((s) & 0x01))  /* Display OFF/ON: (aeh - afh) */
#  define SSD1306_DISPOFF        SSD1306_DISPOFFON(0)   /*   Display off */
#  define SSD1306_DISPON         SSD1306_DISPOFFON(1)   /*   Display on */
#define SSD1306_PAGEADDR(a)      (0xb0 | ((a) & 0x0f))  /* Set Page Address: (b0h - b7h) */
#define SSD1306_SCANDIR(d)       (0xc0 | ((d) & 0x08))  /* Set Common Output Scan Direction: (c0h - c8h) */
#  define SSD1306_SCANFROMCOM0   SSD1306_SCANDIR(0x00)  /*   Scan from COM[0] to COM[n-1]*/
#  define SSD1306_SCANTOCOM0     SSD1306_SCANDIR(0x08)  /*   Scan from COM[n-1] to COM[0] */
#define SSD1306_DISPOFFS_MODE    (0xd3)                 /* Set Display Offset: (Double Bytes Command) */
#  define SSD1306_DISPOFFS(o)    ((o) & 0x3f)
#define SSD1306_CLKDIV_SET       (0xd5)                 /* Set Display Clock Divide Ratio/Oscillator Frequency: (Double Bytes Command) */
#  define SSD1306_CLKDIV(f,d)    ((((f) & 0x0f) << 4) | ((d) & 0x0f))
#define SSD1306_CHRGPER_SET      (0xd9)                 /* Set Dis-charge/Pre-charge Period: (Double Bytes Command) */
#  define SSD1306_CHRGPER(d,p)   ((((d) & 0x0f) << 4) | ((p) & 0x0f))
#define SSD1306_CMNPAD_CONFIG    (0xda)                 /* Set Common pads hardware configuration: (Double Bytes Command) */
#  define SSD1306_CMNPAD(c)      ((0x02) | ((c) & 0x10))
#define SSD1306_VCOM_SET         (0xdb)                 /* Set VCOM Deselect Level: (Double Byte Command) */
#  define SSD1306_VCOM(v)        (v)

#define SSD1306_CHRPUMP_SET      (0x8d)                 /* Charge Pump Setting */
#  define SSD1306_CHRPUMP_ON     (0x14)
#  define SSD1306_CHRPUMP_OFF    (0x10)

#define SSD1306_RMWSTART         (0xe0)                 /* Read-Modify-Write: (e0h) */
#define SSD1306_NOP              (0xe3)                 /* NOP: (e3h) */
#define SSD1306_END              (0xee)                 /* End: (eeh) */

#define SSD1306_WRDATA(d)        (d)                    /* Write Display Data */
#define SSD1306_STATUS_BUSY      (0x80)                 /* Read Status */
#define SSD1306_STATUS_ONOFF     (0x40)
#define SSD1306_RDDATA(d)        (d)                    /* Read Display Data */

/* Color Properties *******************************************************************/
/* Display Resolution
 *
 * The SSD1306 display controller can handle a resolution of 132x64. The UG-2864HSWEG01
 * on the base board is 128x64; the UG-2832HSWEG04 is 128x32.
 */

#if defined(CONFIG_LCD_UG2864HSWEG01)
#  define SSD1306_DEV_NATIVE_XRES 128  /* Only 128 of 131 columns used */
#  define SSD1306_DEV_NATIVE_YRES 64   /* 8 pages each 8 rows */
#  define SSD1306_DEV_XOFFSET     2    /* Offset to logical column 0 */
#  define SSD1306_DEV_PAGES       8    /* 8 pages */
#  define SSD1306_DEV_CMNPAD      0x12 /* COM configuration */
#elif defined(CONFIG_LCD_UG2832HSWEG04)
#  define SSD1306_DEV_NATIVE_XRES 128  /* Only 128 of 131 columns used */
#  define SSD1306_DEV_NATIVE_YRES 32   /* 4 pages each 8 rows */
#  define SSD1306_DEV_XOFFSET     2    /* Offset to logical column 0 */
#  define SSD1306_DEV_PAGES       4    /* 4 pages */
#  define SSD1306_DEV_CMNPAD      0x02 /* COM configuration */
#endif

#if defined(CONFIG_LCD_LANDSCAPE) || defined(CONFIG_LCD_RLANDSCAPE)
#  define SSD1306_DEV_XRES        SSD1306_DEV_NATIVE_XRES
#  define SSD1306_DEV_YRES        SSD1306_DEV_NATIVE_YRES
#else
#  define SSD1306_DEV_XRES        SSD1306_DEV_NATIVE_YRES
#  define SSD1306_DEV_YRES        SSD1306_DEV_NATIVE_XRES
#endif

#define SSD1306_DEV_DUTY          (SSD1306_DEV_NATIVE_YRES-1)

/* Bytes per logical row and actual device row */

#define SSD1306_DEV_XSTRIDE       (SSD1306_DEV_XRES >> 3)
#define SSD1306_DEV_YSTRIDE       (SSD1306_DEV_YRES >> 3)

/* Color depth and format */

#define SSD1306_DEV_BPP           1
#define SSD1306_DEV_COLORFMT      FB_FMT_Y1

/* Default contrast */

#define SSD1306_DEV_CONTRAST      (128)

/* The size of the shadow frame buffer or one row buffer.
 *
 * Frame buffer size: 128 columns x 64 rows / 8 bits-per-pixel
 * Row size:          128 columns x 8 rows-per-page / 8 bits-per-pixel
 */

#define SSD1306_DEV_FBSIZE        (SSD1306_DEV_XSTRIDE * SSD1306_DEV_YRES)
#define SSD1306_DEV_ROWSIZE       (SSD1306_DEV_XSTRIDE)

/* Orientation.  There seem to be device differences. */

#if defined(CONFIG_LCD_UG2864HSWEG01)
#  if defined(CONFIG_LCD_LANDSCAPE)
#    undef  SSD1306_DEV_REVERSEX
#    undef  SSD1306_DEV_REVERSEY
#  elif defined(CONFIG_LCD_RLANDSCAPE)
#    define SSD1306_DEV_REVERSEX  1
#    define SSD1306_DEV_REVERSEY  1
#  endif
#elif defined(CONFIG_LCD_UG2832HSWEG04)
#  if defined(CONFIG_LCD_LANDSCAPE)
#    define SSD1306_DEV_REVERSEX  1
#    undef  SSD1306_DEV_REVERSEY
#  elif defined(CONFIG_LCD_RLANDSCAPE)
#    undef  SSD1306_DEV_REVERSEX
#    define SSD1306_DEV_REVERSEY  1
#  endif
#endif

/* Bit helpers */

#define LS_BIT                    (1 << 0)
#define MS_BIT                    (1 << 7)

/* Debug ******************************************************************************/

#ifdef CONFIG_DEBUG_LCD
#  define lcddbg(format, ...)     dbg(format, ##__VA_ARGS__)
#  define lcdvdbg(format, ...)    vdbg(format, ##__VA_ARGS__)
#else
#  define lcddbg(x...)
#  define lcdvdbg(x...)
#endif

/**************************************************************************************
 * Private Type Definition
 **************************************************************************************/

/* This structure describes the state of this driver */

struct ssd1306_dev_s
{
  struct lcd_dev_s       dev;      /* Publically visible device structure */

  /* Private LCD-specific information follows */

  FAR struct spi_dev_s  *spi;      /* Cached SPI device reference */
  uint8_t                contrast; /* Current contrast setting */
  bool                   on;       /* true: display is on */


 /* The SSD1306 does not support reading from the display memory in SPI mode.
  * Since there is 1 BPP and access is byte-by-byte, it is necessary to keep
  * a shadow copy of the framebuffer memory. At 128x64, this amounts to 1KB.
  */

  uint8_t fb[SSD1306_DEV_FBSIZE];
};

/**************************************************************************************
 * Private Function Protototypes
 **************************************************************************************/

/* Low-level SPI helpers */

#ifdef CONFIG_SPI_OWNBUS
static inline void ssd1306_configspi(FAR struct spi_dev_s *spi);
#  define ssd1306_lock(spi)
#  define ssd1306_unlock(spi)
#else
#  define ssd1306_configspi(spi)
static void ssd1306_lock(FAR struct spi_dev_s *spi);
static void ssd1306_unlock(FAR struct spi_dev_s *spi);
#endif

/* LCD Data Transfer Methods */

static int ssd1306_putrun(fb_coord_t row, fb_coord_t col,
                          FAR const uint8_t *buffer, size_t npixels);
static int ssd1306_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t *buffer,
                          size_t npixels);

/* LCD Configuration */

static int ssd1306_getvideoinfo(FAR struct lcd_dev_s *dev,
                                FAR struct fb_videoinfo_s *vinfo);
static int ssd1306_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int planeno,
                                FAR struct lcd_planeinfo_s *pinfo);

/* LCD RGB Mapping */

#ifdef CONFIG_FB_CMAP
#  error "RGB color mapping not supported by this driver"
#endif

/* Cursor Controls */

#ifdef CONFIG_FB_HWCURSOR
#  error "Cursor control not supported by this driver"
#endif

/* LCD Specific Controls */

static int ssd1306_getpower(struct lcd_dev_s *dev);
static int ssd1306_setpower(struct lcd_dev_s *dev, int power);
static int ssd1306_getcontrast(struct lcd_dev_s *dev);
static int ssd1306_setcontrast(struct lcd_dev_s *dev, unsigned int contrast);

/**************************************************************************************
 * Private Data
 **************************************************************************************/

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

static uint8_t g_runbuffer[SSD1306_DEV_ROWSIZE];

/* This structure describes the overall LCD video controller */

static const struct fb_videoinfo_s g_videoinfo =
{
  .fmt     = SSD1306_DEV_COLORFMT,  /* Color format: RGB16-565: RRRR RGGG GGGB BBBB */
  .xres    = SSD1306_DEV_XRES,      /* Horizontal resolution in pixel columns */
  .yres    = SSD1306_DEV_YRES,      /* Vertical resolution in pixel rows */
  .nplanes = 1,                     /* Number of color planes supported */
};

/* This is the standard, NuttX Plane information object */

static const struct lcd_planeinfo_s g_planeinfo =
{
  .putrun = ssd1306_putrun,          /* Put a run into LCD memory */
  .getrun = ssd1306_getrun,          /* Get a run from LCD memory */
  .buffer = (uint8_t*)g_runbuffer,   /* Run scratch buffer */
  .bpp    = SSD1306_DEV_BPP,         /* Bits-per-pixel */
};

/* This is the OLED driver instance (only a single device is supported for now) */

static struct ssd1306_dev_s g_oleddev =
{
  .dev =
  {
    /* LCD Configuration */

    .getvideoinfo = ssd1306_getvideoinfo,
    .getplaneinfo = ssd1306_getplaneinfo,

    /* LCD RGB Mapping -- Not supported */
    /* Cursor Controls -- Not supported */

    /* LCD Specific Controls */

    .getpower     = ssd1306_getpower,
    .setpower     = ssd1306_setpower,
    .getcontrast  = ssd1306_getcontrast,
    .setcontrast  = ssd1306_setcontrast,
  },
};

/**************************************************************************************
 * Private Functions
 **************************************************************************************/

/**************************************************************************************
 * Name: ssd1306_configspi
 *
 * Description:
 *   Configure the SPI for use with the UG-2864HSWEG01
 *
 * Input Parameters:
 *   spi  - Reference to the SPI driver structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 **************************************************************************************/

#ifdef CONFIG_SPI_OWNBUS
static inline void ssd1306_configspi(FAR struct spi_dev_s *spi)
{
  lcdvdbg("Mode: %d Bits: 8 Frequency: %d\n",
          CONFIG_SSD1306_SPIMODE, CONFIG_SSD1306_FREQUENCY);

  /* Configure SPI for the UG-2864HSWEG01.  But only if we own the SPI bus.  Otherwise,
   * don't bother because it might change.
   */

  SPI_SETMODE(spi, CONFIG_SSD1306_SPIMODE);
  SPI_SETBITS(spi, 8);
  SPI_SETFREQUENCY(spi, CONFIG_SSD1306_FREQUENCY);
}
#endif

/**************************************************************************************
 * Name: ssd1306_lock
 *
 * Description:
 *   Select the SPI, locking and  re-configuring if necessary
 *
 * Input Parameters:
 *   spi  - Reference to the SPI driver structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 **************************************************************************************/

#ifndef CONFIG_SPI_OWNBUS
static inline void ssd1306_lock(FAR struct spi_dev_s *spi)
{
  /* Lock the SPI bus if there are multiple devices competing for the SPI bus. */

  SPI_LOCK(spi, true);

  /* Now make sure that the SPI bus is configured for the UG-2864HSWEG01 (it
   * might have gotten configured for a different device while unlocked)
   */

  SPI_SETMODE(spi, CONFIG_SSD1306_SPIMODE);
  SPI_SETBITS(spi, 8);
  SPI_SETFREQUENCY(spi, CONFIG_SSD1306_FREQUENCY);
}
#endif

/**************************************************************************************
 * Name: ssd1306_unlock
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
 **************************************************************************************/

#ifndef CONFIG_SPI_OWNBUS
static inline void ssd1306_unlock(FAR struct spi_dev_s *spi)
{
  /* De-select UG-2864HSWEG01 chip and relinquish the SPI bus. */

  SPI_LOCK(spi, false);
}
#endif

/**************************************************************************************
 * Name:  ssd1306_putrun
 *
 * Description:
 *   This method can be used to write a partial raster line to the LCD.
 *
 * Input Parameters:
 *   row     - Starting row to write to (range: 0 <= row < yres)
 *   col     - Starting column to write to (range: 0 <= col <= xres-npixels)
 *   buffer  - The buffer containing the run to be written to the LCD
 *   npixels - The number of pixels to write to the LCD
 *             (range: 0 < npixels <= xres-col)
 *
 **************************************************************************************/

#if defined(CONFIG_LCD_LANDSCAPE) || defined(CONFIG_LCD_RLANDSCAPE)
static int ssd1306_putrun(fb_coord_t row, fb_coord_t col, FAR const uint8_t *buffer,
                          size_t npixels)
{
  /* Because of this line of code, we will only be able to support a single UG device */

  FAR struct ssd1306_dev_s *priv = (FAR struct ssd1306_dev_s *)&g_oleddev;
  FAR uint8_t *fbptr;
  FAR uint8_t *ptr;
  uint8_t devcol;
  uint8_t fbmask;
  uint8_t page;
  uint8_t usrmask;
  int pixlen;
  uint8_t i;

  lcdvdbg("row: %d col: %d npixels: %d\n", row, col, npixels);
  DEBUGASSERT(buffer);

  /* Clip the run to the display */

  pixlen = npixels;
  if ((unsigned int)col + (unsigned int)pixlen > (unsigned int)SSD1306_DEV_XRES)
    {
      pixlen = (int)SSD1306_DEV_XRES - (int)col;
    }

  /* Verify that some portion of the run remains on the display */

  if (pixlen <= 0 || row > SSD1306_DEV_YRES)
    {
      return OK;
    }

  /* Perform coordinate conversion for reverse landscape mode.
   * If the rows are reversed then rows are are a mirror reflection of
   * top to bottom.
   */

#ifdef SSD1306_DEV_REVERSEY
  row = (SSD1306_DEV_YRES-1) - row;
#endif

  /* If the column is switched then the start of the run is the mirror of
   * the end of the run.
   *
   *            col+pixlen-1
   *     col    |
   *  0  |      |                    XRES
   *  .  S>>>>>>E                    .
   *  .                    E<<<<<<S  .
   *                       |      |
   *                       |      `-(XRES-1)-col
   *                       ` (XRES-1)-col-(pixlen-1)
   */

#ifdef SSD1306_DEV_REVERSEX
  col  = (SSD1306_DEV_XRES-1) - col;
  col -= (pixlen - 1);
#endif

  /* Get the page number.  The range of 64 lines is divided up into eight
   * pages of 8 lines each.
   */

  page = row >> 3;

  /* Update the shadow frame buffer memory. First determine the pixel
   * position in the frame buffer memory.  Pixels are organized like
   * this:
   *
   *  --------+---+---+---+---+-...-+-----+
   *  Segment | 0 | 1 | 2 | 3 | ... | 131 |
   *  --------+---+---+---+---+-...-+-----+
   *     D0   |   | X |   |   |     |     |
   *     D1   |   | X |   |   |     |     |
   *     D2   |   | X |   |   |     |     |
   *     D3   |   | X |   |   |     |     |
   *     D4   |   | X |   |   |     |     |
   *     D5   |   | X |   |   |     |     |
   *     D6   |   | X |   |   |     |     |
   *     D7   |   | X |   |   |     |     |
   *  --------+---+---+---+---+-...-+-----+
   *
   * So, in order to draw a white, horizontal line, at row 45. we
   * would have to modify all of the bytes in page 45/8 = 5.  We
   * would have to set bit 45%8 = 5 in every byte in the page.
   */

  fbmask  = 1 << (row & 7);
  fbptr   = &priv->fb[page * SSD1306_DEV_XRES + col];
#ifdef SSD1306_DEV_REVERSEX
  ptr     = fbptr + (pixlen - 1);
#else
  ptr     = fbptr;
#endif

#ifdef CONFIG_NX_PACKEDMSFIRST
  usrmask = MS_BIT;
#else
  usrmask = LS_BIT;
#endif

  for (i = 0; i < pixlen; i++)
    {
      /* Set or clear the corresponding bit */

#ifdef SSD1306_DEV_REVERSEX
      if ((*buffer & usrmask) != 0)
        {
          *ptr-- |= fbmask;
        }
      else
        {
          *ptr-- &= ~fbmask;
        }
#else
      if ((*buffer & usrmask) != 0)
        {
          *ptr++ |= fbmask;
        }
      else
        {
          *ptr++ &= ~fbmask;
        }
#endif

      /* Inc/Decrement to the next source pixel */

#ifdef CONFIG_NX_PACKEDMSFIRST
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

  /* Offset the column position to account for smaller horizontal
   * display range.
   */

  devcol = col + SSD1306_DEV_XOFFSET;

  /* Lock and select device */

  ssd1306_lock(priv->spi);
  SPI_SELECT(priv->spi, SPIDEV_DISPLAY, true);

  /* Select command transfer */

  SPI_CMDDATA(priv->spi, SPIDEV_DISPLAY, true);

  /* Set the starting position for the run */
  /* Set the column address to the XOFFSET value */

  SPI_SEND(priv->spi, SSD1306_SETCOLL(devcol & 0x0f));
  SPI_SEND(priv->spi, SSD1306_SETCOLH(devcol >> 4));

  /* Set the page address */

  SPI_SEND(priv->spi, SSD1306_PAGEADDR(page));

  /* Select data transfer */

  SPI_CMDDATA(priv->spi, SPIDEV_DISPLAY, false);

  /* Then transfer all of the data */

  (void)SPI_SNDBLOCK(priv->spi, fbptr, pixlen);

  /* De-select and unlock the device */

  SPI_SELECT(priv->spi, SPIDEV_DISPLAY, false);
  ssd1306_unlock(priv->spi);
  return OK;
}
#else
#  error "Configuration not implemented"
#endif

/**************************************************************************************
 * Name:  ssd1306_getrun
 *
 * Description:
 *   This method can be used to read a partial raster line from the LCD.
 *
 * Description:
 *   This method can be used to write a partial raster line to the LCD.
 *
 *  row     - Starting row to read from (range: 0 <= row < yres)
 *  col     - Starting column to read read (range: 0 <= col <= xres-npixels)
 *  buffer  - The buffer in which to return the run read from the LCD
 *  npixels - The number of pixels to read from the LCD
 *            (range: 0 < npixels <= xres-col)
 *
 **************************************************************************************/

#if defined(CONFIG_LCD_LANDSCAPE) || defined(CONFIG_LCD_RLANDSCAPE)
static int ssd1306_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t *buffer,
                      size_t npixels)
{
  /* Because of this line of code, we will only be able to support a single UG device */

  FAR struct ssd1306_dev_s *priv = &g_oleddev;
  FAR uint8_t *fbptr;
  uint8_t page;
  uint8_t fbmask;
  uint8_t usrmask;
  int pixlen;
  uint8_t i;

  lcdvdbg("row: %d col: %d npixels: %d\n", row, col, npixels);
  DEBUGASSERT(buffer);

  /* Clip the run to the display */

  pixlen = npixels;
  if ((unsigned int)col + (unsigned int)pixlen > (unsigned int)SSD1306_DEV_XRES)
    {
      pixlen = (int)SSD1306_DEV_XRES - (int)col;
    }

  /* Verify that some portion of the run is actually the display */

  if (pixlen <= 0 || row > SSD1306_DEV_YRES)
    {
      return -EINVAL;
    }

  /* Perform coordinate conversion for reverse landscape mode.
   * If the rows are reversed then rows are are a mirror reflection of
   * top to bottom.
   */

#ifdef SSD1306_DEV_REVERSEY
  row = (SSD1306_DEV_YRES-1) - row;
#endif

  /* If the column is switched then the start of the run is the mirror of
   * the end of the run.
   *
   *            col+pixlen-1
   *     col    |
   *  0  |      |                    XRES
   *  .  S>>>>>>E                    .
   *  .                    E<<<<<<S  .
   *                       |      |
   *                       |      `-(XRES-1)-col
   *                       ` (XRES-1)-col-(pixlen-1)
   */

#ifdef SSD1306_DEV_REVERSEX
  col  = (SSD1306_DEV_XRES-1) - col;
#endif

  /* Then transfer the display data from the shadow frame buffer memory */
  /* Get the page number.  The range of 64 lines is divided up into eight
   * pages of 8 lines each.
   */

  page = row >> 3;

  /* Update the shadow frame buffer memory. First determine the pixel
   * position in the frame buffer memory.  Pixels are organized like
   * this:
   *
   *  --------+---+---+---+---+-...-+-----+
   *  Segment | 0 | 1 | 2 | 3 | ... | 131 |
   *  --------+---+---+---+---+-...-+-----+
   *     D0   |   | X |   |   |     |     |
   *     D1   |   | X |   |   |     |     |
   *     D2   |   | X |   |   |     |     |
   *     D3   |   | X |   |   |     |     |
   *     D4   |   | X |   |   |     |     |
   *     D5   |   | X |   |   |     |     |
   *     D6   |   | X |   |   |     |     |
   *     D7   |   | X |   |   |     |     |
   *  --------+---+---+---+---+-...-+-----+
   *
   * So, in order to draw a white, horizontal line, at row 45. we
   * would have to modify all of the bytes in page 45/8 = 5.  We
   * would have to set bit 45%8 = 5 in every byte in the page.
   */

  fbmask  = 1 << (row & 7);
  fbptr   = &priv->fb[page * SSD1306_DEV_XRES + col];

#ifdef CONFIG_NX_PACKEDMSFIRST
  usrmask = MS_BIT;
#else
  usrmask = LS_BIT;
#endif

  *buffer = 0;
  for (i = 0; i < pixlen; i++)
    {
      /* Set or clear the corresponding bit */

#ifdef SSD1306_DEV_REVERSEX
      uint8_t byte = *fbptr--;
#else
      uint8_t byte = *fbptr++;
#endif
      if ((byte & fbmask) != 0)
        {
          *buffer |= usrmask;
        }

      /* Inc/Decrement to the next destination pixel. Hmmmm. It looks like
       * this logic could write past the end of the user buffer.  Revisit
       * this!
       */

#ifdef CONFIG_NX_PACKEDMSFIRST
      if (usrmask == LS_BIT)
        {
          buffer++;
         *buffer = 0;
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
         *buffer = 0;
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
#else
#  error "Configuration not implemented"
#endif

/**************************************************************************************
 * Name:  ssd1306_getvideoinfo
 *
 * Description:
 *   Get information about the LCD video controller configuration.
 *
 **************************************************************************************/

static int ssd1306_getvideoinfo(FAR struct lcd_dev_s *dev,
                                FAR struct fb_videoinfo_s *vinfo)
{
  DEBUGASSERT(dev && vinfo);
  lcdvdbg("fmt: %d xres: %d yres: %d nplanes: %d\n",
          g_videoinfo.fmt, g_videoinfo.xres, g_videoinfo.yres, g_videoinfo.nplanes);
  memcpy(vinfo, &g_videoinfo, sizeof(struct fb_videoinfo_s));
  return OK;
}

/**************************************************************************************
 * Name:  ssd1306_getplaneinfo
 *
 * Description:
 *   Get information about the configuration of each LCD color plane.
 *
 **************************************************************************************/

static int ssd1306_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int planeno,
                                FAR struct lcd_planeinfo_s *pinfo)
{
  DEBUGASSERT(pinfo && planeno == 0);
  lcdvdbg("planeno: %d bpp: %d\n", planeno, g_planeinfo.bpp);
  memcpy(pinfo, &g_planeinfo, sizeof(struct lcd_planeinfo_s));
  return OK;
}

/**************************************************************************************
 * Name:  ssd1306_getpower
 *
 * Description:
 *   Get the LCD panel power status (0: full off - CONFIG_LCD_MAXPOWER: full on. On
 *   backlit LCDs, this setting may correspond to the backlight setting.
 *
 **************************************************************************************/

static int ssd1306_getpower(FAR struct lcd_dev_s *dev)
{
  FAR struct ssd1306_dev_s *priv = (FAR struct ssd1306_dev_s *)dev;
  DEBUGASSERT(priv);

  lcdvdbg("power: %s\n", priv->on ? "ON" : "OFF");
  return priv->on ? CONFIG_LCD_MAXPOWER : 0;
}

/**************************************************************************************
 * Name:  ssd1306_setpower
 *
 * Description:
 *   Enable/disable LCD panel power (0: full off - CONFIG_LCD_MAXPOWER: full on). On
 *   backlit LCDs, this setting may correspond to the backlight setting.
 *
 **************************************************************************************/

static int ssd1306_setpower(FAR struct lcd_dev_s *dev, int power)
{
  struct ssd1306_dev_s *priv = (struct ssd1306_dev_s *)dev;
  DEBUGASSERT(priv && (unsigned)power <= CONFIG_LCD_MAXPOWER && priv->spi);

  lcdvdbg("power: %d [%d]\n", power, priv->on ? CONFIG_LCD_MAXPOWER : 0);

  /* Lock and select device */

  ssd1306_lock(priv->spi);
  SPI_SELECT(priv->spi, SPIDEV_DISPLAY, true);

  if (power <= 0)
    {
      /* Turn the display off */

      (void)SPI_SEND(priv->spi, SSD1306_DISPOFF);
      priv->on = false;
    }
  else
    {
      /* Turn the display on */

      (void)SPI_SEND(priv->spi, SSD1306_DISPON);
      priv->on = true;
    }

  /* De-select and unlock the device */

  SPI_SELECT(priv->spi, SPIDEV_DISPLAY, false);
  ssd1306_unlock(priv->spi);
  return OK;
}

/**************************************************************************************
 * Name:  ssd1306_getcontrast
 *
 * Description:
 *   Get the current contrast setting (0-CONFIG_LCD_MAXCONTRAST).
 *
 **************************************************************************************/

static int ssd1306_getcontrast(struct lcd_dev_s *dev)
{
  struct ssd1306_dev_s *priv = (struct ssd1306_dev_s *)dev;
  DEBUGASSERT(priv);

  lcdvdbg("contrast: %d\n", priv->contrast);
  return priv->contrast;
}

/**************************************************************************************
 * Name:  ssd1306_setcontrast
 *
 * Description:
 *   Set LCD panel contrast (0-CONFIG_LCD_MAXCONTRAST).
 *
 **************************************************************************************/

static int ssd1306_setcontrast(struct lcd_dev_s *dev, unsigned int contrast)
{
  struct ssd1306_dev_s *priv = (struct ssd1306_dev_s *)dev;
  unsigned int scaled;

  lcdvdbg("contrast: %d\n", contrast);
  DEBUGASSERT(priv);

  /* Verify the contrast value */

#ifdef CONFIG_DEBUG
  if (contrast > CONFIG_LCD_MAXCONTRAST)
    {
      return -EINVAL;
    }
#endif

  /* Scale contrast:  newcontrast = 255 * contrast / CONFIG_LCD_MAXCONTRAST
   * Where contrast is in the range {1,255}
   */

#if CONFIG_LCD_MAXCONTRAST != 255
  scaled = ((contrast << 8) - 1) / CONFIG_LCD_MAXCONTRAST;
#else
  scaled = contrast;
#endif

  /* Lock and select device */

  ssd1306_lock(priv->spi);
  SPI_SELECT(priv->spi, SPIDEV_DISPLAY, true);

  /* Select command transfer */

  SPI_CMDDATA(priv->spi, SPIDEV_DISPLAY, true);

  /* Set the contrast */

  (void)SPI_SEND(priv->spi, SSD1306_CONTRAST_MODE);    /* Set contrast control register */
  (void)SPI_SEND(priv->spi, SSD1306_CONTRAST(scaled)); /* Data 1: Set 1 of 256 contrast steps */
  priv->contrast = contrast;

  /* De-select and unlock the device */

  SPI_SELECT(priv->spi, SPIDEV_DISPLAY, false);
  ssd1306_unlock(priv->spi);
  return OK;
}

/**************************************************************************************
 * Public Functions
 **************************************************************************************/

/**************************************************************************************
 * Name:  ssd1306_initialize
 *
 * Description:
 *   Initialize the UG-2864HSWEG01 video hardware.  The initial state of the
 *   OLED is fully initialized, display memory cleared, and the OLED ready
 *   to use, but with the power setting at 0 (full off == sleep mode).
 *
 * Input Parameters:
 *
 *   spi - A reference to the SPI driver instance.
 *   devno - A value in the range of 0 through CONFIG_SSD1306_NINTERFACES-1.
 *     This allows support for multiple OLED devices.
 *
 * Returned Value:
 *
 *   On success, this function returns a reference to the LCD object for
 *   the specified OLED.  NULL is returned on any failure.
 *
 **************************************************************************************/

FAR struct lcd_dev_s *ssd1306_initialize(FAR struct spi_dev_s *spi, unsigned int devno)
{
  FAR struct ssd1306_dev_s  *priv = &g_oleddev;

  lcdvdbg("Initializing\n");
  DEBUGASSERT(spi && devno == 0);

  /* Save the reference to the SPI device */

  priv->spi = spi;

  /* Configure the SPI */

  ssd1306_configspi(spi);

  /* Lock and select device */

  ssd1306_lock(priv->spi);
  SPI_SELECT(spi, SPIDEV_DISPLAY, true);

  /* Select command transfer */

  SPI_CMDDATA(spi, SPIDEV_DISPLAY, true);

  /* Configure OLED SPI or I/O, must be delayed 1-10ms */

  up_mdelay(5);

  /* Configure the device */

  SPI_SEND(spi, SSD1306_DISPOFF);         /* Display off 0xae */
  SPI_SEND(spi, SSD1306_SETCOLL(0));      /* Set lower column address 0x00 */
  SPI_SEND(spi, SSD1306_SETCOLH(0));      /* Set higher column address 0x10 */
  SPI_SEND(spi, SSD1306_STARTLINE(0));    /* Set display start line 0x40 */
  /* SPI_SEND(spi, SSD1306_PAGEADDR(0));*//* Set page address  (Can ignore)*/
  SPI_SEND(spi, SSD1306_CONTRAST_MODE);   /* Contrast control 0x81 */
  SPI_SEND(spi ,SSD1306_CONTRAST(SSD1306_DEV_CONTRAST));  /* Default contrast 0xCF */
  SPI_SEND(spi, SSD1306_REMAPPLEFT);      /* Set segment remap left 95 to 0 | 0xa1 */
  /* SPI_SEND(spi, SSD1306_EDISPOFF); */  /* Normal display off 0xa4 (Can ignore)*/
  SPI_SEND(spi, SSD1306_NORMAL);          /* Normal (un-reversed) display mode 0xa6 */
  SPI_SEND(spi, SSD1306_MRATIO_MODE);     /* Multiplex ratio 0xa8 */
  SPI_SEND(spi, SSD1306_MRATIO(SSD1306_DEV_DUTY));  /* Duty = 1/64 or 1/32 */
  /* SPI_SEND(spi, SSD1306_SCANTOCOM0);*/ /* Com scan direction: Scan from COM[n-1] to COM[0] (Can ignore)*/
  SPI_SEND(spi, SSD1306_DISPOFFS_MODE);   /* Set display offset 0xd3 */
  SPI_SEND(spi, SSD1306_DISPOFFS(0));
  SPI_SEND(spi, SSD1306_CLKDIV_SET);      /* Set clock divider 0xd5*/
  SPI_SEND(spi, SSD1306_CLKDIV(8,0));     /* 0x80*/

  SPI_SEND(spi, SSD1306_CHRGPER_SET);     /* Set pre-charge period 0xd9 */
  SPI_SEND(spi, SSD1306_CHRGPER(0x0f,1)); /* 0xf1 or 0x22 Enhanced mode */

  SPI_SEND(spi, SSD1306_CMNPAD_CONFIG);   /* Set common pads / set com pins hardware configuration 0xda */
  SPI_SEND(spi, SSD1306_CMNPAD(SSD1306_DEV_CMNPAD)); /* 0x12 or 0x02 */

  SPI_SEND(spi, SSD1306_VCOM_SET);        /* set vcomh 0xdb*/
  SPI_SEND(spi, SSD1306_VCOM(0x40));

  SPI_SEND(spi, SSD1306_CHRPUMP_SET);     /* Set Charge Pump enable/disable 0x8d ssd1306 */
  SPI_SEND(spi, SSD1306_CHRPUMP_ON);      /* 0x14 close 0x10 */

  /* SPI_SEND(spi, SSD1306_DCDC_MODE); */ /* DC/DC control mode: on (SSD1306 Not supported) */
  /* SPI_SEND(spi, SSD1306_DCDC_ON); */

  SPI_SEND(spi, SSD1306_DISPON);          /* Display ON 0xaf */

  /* De-select and unlock the device */

  SPI_SELECT(spi, SPIDEV_DISPLAY, false);
  ssd1306_unlock(priv->spi);

  /* Clear the display */

  up_mdelay(100);
  ssd1306_fill(&priv->dev, SSD1306_Y1_BLACK);
  return &priv->dev;
}

/**************************************************************************************
 * Name:  ssd1306_fill
 *
 * Description:
 *   This non-standard method can be used to clear the entire display by writing one
 *   color to the display.  This is much faster than writing a series of runs.
 *
 * Input Parameters:
 *   priv   - Reference to private driver structure
 *
 * Assumptions:
 *   Caller has selected the OLED section.
 *
 **************************************************************************************/

void ssd1306_fill(FAR struct lcd_dev_s *dev, uint8_t color)
{
  FAR struct ssd1306_dev_s  *priv = &g_oleddev;
  unsigned int page;

  /* Make an 8-bit version of the selected color */

  if (color & 1)
    {
      color = 0xff;
    }
  else
    {
      color = 0;
    }

  /* Initialize the framebuffer */

  memset(priv->fb, color, SSD1306_DEV_FBSIZE);

  /* Lock and select device */

  ssd1306_lock(priv->spi);
  SPI_SELECT(priv->spi, SPIDEV_DISPLAY, true);

  /* Visit each page */

  for (page = 0; page < SSD1306_DEV_PAGES; page++)
    {
      /* Select command transfer */

      SPI_CMDDATA(priv->spi, SPIDEV_DISPLAY, true);

      /* Set the column address to the XOFFSET value */

      SPI_SEND(priv->spi, SSD1306_SETCOLL(SSD1306_DEV_XOFFSET));
      SPI_SEND(priv->spi, SSD1306_SETCOLH(0));

      /* Set the page address */

      SPI_SEND(priv->spi, SSD1306_PAGEADDR(page));

      /* Select data transfer */

      SPI_CMDDATA(priv->spi, SPIDEV_DISPLAY, false);

      /* Transfer one page of the selected color */

      (void)SPI_SNDBLOCK(priv->spi, &priv->fb[page * SSD1306_DEV_XRES],
                         SSD1306_DEV_XRES);
    }

  /* De-select and unlock the device */

  SPI_SELECT(priv->spi, SPIDEV_DISPLAY, false);
  ssd1306_unlock(priv->spi);
}

#endif /* CONFIG_LCD_SSD1306 */
