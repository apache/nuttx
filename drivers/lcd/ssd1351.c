/****************************************************************************
 * drivers/lcd/ssd1351.c
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
#include <nuttx/lcd/ssd1351.h>

#ifdef CONFIG_LCD_SSD1351

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* SSD1351 configuration settings:
 * CONFIG_SSD1351_PARALLEL8BIT - 8-bit parallel interface
 * CONFIG_SSD1351_SPI3WIRE     - 3-wire SPI interface
 * CONFIG_SSD1351_SPI4WIRE     - 4-wire SPI interface
 * CONFIG_SSD1351_SPIMODE      - SPI mode
 * CONFIG_SSD1351_SPIFREQ      - SPI frequency
 * CONFIG_SSD1351_NINTERFACES  - number of physical devices supported
 * CONFIG_SSD1351_XRES         - X resolution
 * CONFIG_SSD1351_YRES         - Y resolution
 * CONFIG_SSD1351_MIRRORX      - mirror along the X axis
 * CONFIG_SSD1351_MIRRORY      - mirror along the Y axis
 * CONFIG_SSD1351_INVERT       - invert the display
 * CONFIG_SSD1351_VDDEXT       - external VDD
 * CONFIG_SSD1351_TRST         - reset period
 * CONFIG_SSD1351_TPRECHG1     - first pre-charge period
 * CONFIG_SSD1351_PERFENHANCE  - enhance display performance
 * CONFIG_SSD1351_CLKDIV       - clock divider
 * CONFIG_SSD1351_OSCFREQ      - oscillator frequency
 * CONFIG_SSD1351_TPRECHG2     - second pre-charge period
 * CONFIG_SSD1351_VPRECHG      - pre-charge voltage level
 * CONFIG_SSD1351_VCOMH        - COM deselect voltage level
 * CONFIG_SSD1351_CONTRASTA    - color A contrast
 * CONFIG_SSD1351_CONTRASTB    - color B contrast
 * CONFIG_SSD1351_CONTRASTC    - color C contrast
 * CONFIG_SSD1351_MSTRCONTRAST - master contrast ratio
 *
 * Required LCD driver settings:
 * CONFIG_LCD_SSD1351          - enables SSD1351 support
 * CONFIG_LCD_MAXPOWER         - maximum power, must be 1
 *
 * Additional LCD driver settings:
 * CONFIG_LCD_LANDSCAPE        - landscape
 * CONFIG_LCD_RLANDSCAPE       - reverse landscape
 * CONFIG_LCD_PORTRAIT         - portrait
 * CONFIG_LCD_RPORTRAIT        - reverse portrait
 *
 * Required SPI driver settings:
 * CONFIG_SPI                  - enables support for SPI
 * CONFIG_SPI_CMDDATA          - enables support for cmd/data selection
 *                               (if using 4-wire SPI)
 */

/* Max power */

#if CONFIG_LCD_MAXPOWER != 1
#  error "CONFIG_LCD_MAXPOWER should be 1"
#endif

/* 9-bit SPI */

#ifdef CONFIG_SSD1351_SPI3WIRE
#  define SSD1351_SPICMD  0
#  define SSD1351_SPIDATA (1 << 8)
#  define SSD1351_SPIBITS 9
#else
#  define SSD1351_SPIBITS 8
#endif

/* Macro Helpers ************************************************************/

#define SSD1351_MAX(a, b)        ((a) > (b) ? (a) : (b))
#define SSD1351_MIN(a, b)        ((a) < (b) ? (a) : (b))
#define SSD1351_CLAMP(n, a, b)   SSD1351_MIN(SSD1351_MAX(n, a), b)

/* Fundamental Commands *****************************************************/

/* Set column address.  Two data bytes.
 *   Data 1: start address (0-127)
 *   Data 2: end address (0-127)
 */

#define SSD1351_CMD_COLADDR      0x15

/* Set row address.  Two data bytes.
 *   Data 1: start address (0-127)
 *   Data 2: end address (0-127)
 */

#define SSD1351_CMD_ROWADDR      0x75

/* Write data bytes to RAM. */

#define SSD1351_CMD_RAMWRITE     0x5c

/* Read data bytes from RAM. */

#define SSD1351_CMD_RAMREAD      0x5d

/* Set address increment, column address mapping, color sequence,
 * scan direction, COM split, and color depth.
 * One data byte.
 */

#define SSD1351_CMD_ORIENTATION  0xa0
#define SSD1351_ADDRINCHORIZ     0x00 /* Horizontal address increment */
#define SSD1351_ADDRINCVERT      0x01 /* Vertical address increment */
#define SSD1351_REMAPCOL0        0x00 /* Column address 0 mapped to SEG0 */
#define SSD1351_REMAPCOL127      0x02 /* Column address 127 mapped to SEG0 */
#define SSD1351_COLORABC         0x00 /* Color sequence ABC */
#define SSD1351_COLORCBA         0x04 /* Color sequence CBA */
#define SSD1351_SCANFROMCOM0     0x00 /* Scan from COM0 */
#define SSD1351_SCANTOCOM0       0x10 /* Scan to COM0 */
#define SSD1351_SPLITDIS         0x00 /* Disable COM split odd even */
#define SSD1351_SPLITEN          0x20 /* Enable COM split odd even */
#define SSD1351_DEPTH65K         0x00 /* 65k color depth */
#define SSD1351_DEPTH262K1       0x80 /* 262k color depth format 1 */
#define SSD1351_DEPTH262K2       0xc0 /* 262k color depth format 2 */

/* Set vertical scroll by RAM (0-128).  One data byte. */

#define SSD1351_CMD_STARTLINE    0xa1
#define SSD1351_STARTLINE(n)     SSD1351_CLAMP(n, 0, 128)

/* Set vertical scroll by row (0-128).  One data byte. */

#define SSD1351_CMD_OFFSET       0xa2
#define SSD1351_OFFSET(n)        SSD1351_CLAMP(n, 0, 128)

/* Set all pixels off.  No data bytes. */

#define SSD1351_CMD_ALLOFF       0xa4

/* Set all pixels on.  No data bytes. */

#define SSD1351_CMD_ALLON        0xa5

/* Set normal display.  No data bytes. */

#define SSD1351_CMD_NORMAL       0xa6

/* Set inverse display.  No data bytes. */

#define SSD1351_CMD_INVERSE      0xa7

/* Set VDD and interface.  One data byte. */

#define SSD1351_CMD_VDDIFACE     0xab
#define SSD1351_VDDEXT           0x00 /* external VDD */
#define SSD1351_VDDINT           0x01 /* internal VDD */
#define SSD1351_IFACE8BIT        0x00 /* 8-bit parallel */
#define SSD1351_IFACE16BIT       0x40 /* 16-bit parallel */
#define SSD1351_IFACE18BIT       0xc0 /* 18-bit parallel */

/* Set display off (sleep mode on).  No data bytes. */

#define SSD1351_CMD_DISPOFF      0xae

/* Set display on (sleep mode off).  No data bytes. */

#define SSD1351_CMD_DISPON       0xaf

/* Set reset period in DCLKs (5-31) and first pre-charge period
 * in DCLKs (3-15).  One data byte.
 */

#define SSD1351_CMD_TRSTTPRECHG1 0xb1
#define SSD1351_TRST(n)          (SSD1351_CLAMP(n, 5, 31) / 2)
#define SSD1351_TPRECHG1(n)      (SSD1351_CLAMP(n, 3, 15) << 4)

/* Set display performance.  Three data bytes. */

#define SSD1351_CMD_PERF         0xb2
#define SSD1351_PERFNORMAL       0x00 /* Data 1: normal performance */
#define SSD1351_PERFENHANCED     0xa4 /* Data 1: enhanced performance */
#define SSD1351_PERFDATA2        0x00
#define SSD1351_PERFDATA3        0x00

/* Set clock divider (0-10) and oscillator frequency (0-15).
 * One data byte.
 */

#define SSD1351_CMD_DIVFREQ      0xb3
#define SSD1351_CLKDIV(r)        (SSD1351_CLAMP(r, 0, 10) << 0)
#define SSD1351_OSCFREQ(r)       (SSD1351_CLAMP(r, 0, 15) << 4)

/* Set segment low voltage.  Three data bytes. */

#define SSD1351_CMD_VSL          0xb4
#define SSD1351_VSLEXT           0xa0 /* Data 1: external VSL */
#define SSD1351_VSLDATA2         0xb5
#define SSD1351_VSLDATA3         0x55

/* Set GPIO pins.  One data byte. */

#define SSD1351_CMD_GPIO         0xb5
#define SSD1351_GPIODIS          0x00 /* High impedance, disabled */
#define SSD1351_GPIOEN           0x01 /* High impedance, enabled */
#define SSD1351_GPIOLOW          0x02 /* Output low */
#define SSD1351_GPIOHIGH         0x03 /* Output high */
#define SSD1351_GPIO0(n)         (((n) & 3) << 0)
#define SSD1351_GPIO1(n)         (((n) & 3) << 2)

/* Set second pre-charge period in DCLKs (1-15).  One data byte. */

#define SSD1351_CMD_TPRECHG2     0xb6
#define SSD1351_TPRECHG2(n)      SSD1351_CLAMP(n, 1, 15)

/* Set lookup table for grayscale pulse width.  63 data bytes. */

#define SSD1351_CMD_LUT              0xb8

/* Use built-in linear lookup table.  No data bytes. */

#define SSD1351_CMD_LINEARLUT        0xb9

/* Set pre-charge voltage level as a percentage of VCC (20-60).
 * One data byte.
 */

#define SSD1351_CMD_VPRECHG      0xbb
#define SSD1351_VPRECHG(n)       (100 * (SSD1351_CLAMP(n, 20, 60) - 20) / \
                                  (100 * (60 - 20) / 31))

/* Set COM deselect voltage level as a percentage of VCC (72-86).
 * One data byte.
 */

#define SSD1351_CMD_VCOMH        0xbe
#define SSD1351_VCOMH(n)         ((SSD1351_CLAMP(n, 72, 86) - 72) / 2)

/* Set contrast for colors A (0-255), B (0-255), and C (0-255).
 * Three data bytes.
 *   Data 1: color A
 *   Data 2: color B
 *   Data 3: color C
 */

#define SSD1351_CMD_CONTRAST     0xc1
#define SSD1351_CONTRAST(n)      SSD1351_CLAMP(n, 0, 255)

/* Set master contrast ratio in sixteenths (1-16).  One data byte. */

#define SSD1351_CMD_MSTRCONTRAST 0xc7
#define SSD1351_MSTRCONTRAST(n)  (SSD1351_CLAMP(n, 1, 16) - 1)

/* Set multiplex ratio (16-128).  One data byte. */

#define SSD1351_CMD_MUXRATIO     0xca
#define SSD1351_MUXRATIO(n)      (SSD1351_CLAMP(n, 16, 128) - 1)

/* Set command lock.  One data byte. */

#define SSD1351_CMD_LOCK         0xfd
#define SSD1351_UNLOCK           0x12 /* Unlock commands */
#define SSD1351_LOCK             0x16 /* Lock commands */
#define SSD1351_INACCESSIBLE     0xb0 /* Make some commands inaccessible */
#define SSD1351_ACCESSIBLE       0xb1 /* Make some commands accessible */

/* Graphic Acceleration Commands ********************************************/

/* Set horizontal scroll.  Five data bytes.
 *   Data 1: 0x00:      no scrolling
 *           0x01-0x3f: scroll towards SEG127 with 1 column offset
 *           0x40-0xff: scroll towards SEG0 with 1 column offset
 *   Data 2: start row address
 *   Data 3: number of rows to scroll
 */

#define SSD1351_CMD_HSCROLL      0x96
#define SSD1351_HSCROLLDATA4     0x00 /* Data 4 */
#define SSD1351_HSCROLLTEST      0x00 /* Data 5: test mode */
#define SSD1351_HSCROLLNORMAL    0x01 /* Data 5: normal */
#define SSD1351_HSCROLLSLOW      0x02 /* Data 5: slow */
#define SSD1351_HSCROLLSLOWEST   0x03 /* Data 5: slowest */

/* Start horizontal scroll.  No data bytes. */

#define SSD1351_CMD_STARTHSCROLL 0x9e

/* Stop horizontal scroll.  No data bytes. */

#define SSD1351_CMD_STOPHSCROLL  0x9f

/* Color Properties *********************************************************/

/* Display resolution */

#if defined(CONFIG_LCD_LANDSCAPE) || defined(CONFIG_LCD_RLANDSCAPE)
#define SSD1351_XRES         CONFIG_SSD1351_XRES
#define SSD1351_YRES         CONFIG_SSD1351_YRES
#else
#define SSD1351_XRES         CONFIG_SSD1351_YRES
#define SSD1351_YRES         CONFIG_SSD1351_XRES
#endif

/* Color depth and format */

#define SSD1351_BPP          16
#define SSD1351_COLORFMT     FB_FMT_RGB16_565
#define SSD1351_STRIDE       (2 * SSD1351_XRES)
#define SSD1351_PIX2BYTES(p) (2 * (p))

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the state of this driver */

struct ssd1351_dev_s
{
  /* Publicly visible device structure */

  struct lcd_dev_s          dev;

  /* Private LCD-specific information follows */

#ifdef CONFIG_SSD1351_PARALLEL8BIT
  FAR struct ssd1351_lcd_s *lcd;   /* Contained platform-specific interface */
#elif defined(CONFIG_SSD1351_SPI3WIRE) || defined(CONFIG_SSD1351_SPI4WIRE)
  FAR struct spi_dev_s     *spi;   /* Contained SPI driver instance */
#endif
  uint8_t                   power; /* Current power (backlight) setting */

  /* This is working memory allocated by the LCD driver for each LCD device
   * and for each color plane.  This memory will hold one raster line of
   * data.  The size of the allocated run buffer must therefore be at least
   * (bpp * xres / 8).  Actual alignment of the buffer must conform to the
   * bitwidth of the underlying pixel type.
   *
   * If there are multiple planes, they may share the same working buffer
   * because different planes will not be operate on concurrently.  However,
   * if there are multiple LCD devices, they must each have unique run
   * buffers.
   */

  uint16_t                  runbuffer[SSD1351_XRES];

  /* This is another buffer, but used internally by the LCD driver in order
   * to expand the pixel data into 9-bit data needed by the LCD.  There are
   * some customizations that would eliminate the need for this extra buffer
   * and for the extra expansion/copy, but those customizations would require
   * a special, non-standard SPI driver that could expand 8- to 9-bit data on
   * the fly.
   */

#ifdef CONFIG_SSD1351_SPI3WIRE
  uint16_t                  rowbuffer[SSD1351_STRIDE + 1];
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helpers */

#ifdef CONFIG_SSD1351_PARALLEL8BIT
#define ssd1351_select(priv)
#define ssd1351_deselect(priv)
#elif defined(CONFIG_SSD1351_SPI3WIRE) || defined(CONFIG_SSD1351_SPI4WIRE)
static void ssd1351_select(FAR struct ssd1351_dev_s *priv);
static void ssd1351_deselect(FAR struct ssd1351_dev_s *priv);
#endif

#if defined(CONFIG_SSD1351_PARALLEL8BIT) && !defined(CONFIG_LCD_NOGETRUN)
static void ssd1351_read(FAR struct ssd1351_dev_s *priv, uint8_t cmd,
                         FAR uint8_t *data, size_t datlen);
#endif
static void ssd1351_write(FAR struct ssd1351_dev_s *priv, uint8_t cmd,
                          FAR const uint8_t *data, size_t datlen);

/* LCD Data Transfer Methods */

static int ssd1351_putrun(fb_coord_t row, fb_coord_t col,
                          FAR const uint8_t *buffer, size_t npixels);
static int ssd1351_getrun(fb_coord_t row, fb_coord_t col,
                          FAR uint8_t *buffer, size_t npixels);

/* LCD Configuration */

static int ssd1351_getvideoinfo(FAR struct lcd_dev_s *dev,
                                FAR struct fb_videoinfo_s *vinfo);
static int ssd1351_getplaneinfo(FAR struct lcd_dev_s *dev,
                                unsigned int planeno,
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

static int ssd1351_getpower(struct lcd_dev_s *dev);
static int ssd1351_setpower(struct lcd_dev_s *dev, int power);
static int ssd1351_getcontrast(struct lcd_dev_s *dev);
static int ssd1351_setcontrast(struct lcd_dev_s *dev, unsigned int contrast);

/* Initialization */

static inline void ssd1351_hwinitialize(FAR struct ssd1351_dev_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is the standard, NuttX LCD driver object */

static struct ssd1351_dev_s g_lcddev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ssd1351_select
 *
 * Description:
 *   Select the SPI, locking and re-configuring if necessary.
 *
 ****************************************************************************/

#if defined(CONFIG_SSD1351_SPI3WIRE) || defined(CONFIG_SSD1351_SPI4WIRE)
static void ssd1351_select(FAR struct ssd1351_dev_s *priv)
{
  FAR struct spi_dev_s *spi = priv->spi;

  /* Select the chip, locking the SPI bus in case there are multiple devices
   * competing for the SPI bus
   */

  ginfo("SELECTED\n");

  SPI_LOCK(spi, true);
  SPI_SELECT(spi, SPIDEV_DISPLAY(0), true);

  /* Now make sure that the SPI bus is configured for this device (it might
   * have gotten configured for a different device while unlocked)
   */

  SPI_SETMODE(spi, CONFIG_SSD1351_SPIMODE);
  SPI_SETBITS(spi, SSD1351_SPIBITS);
  SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, CONFIG_SSD1351_SPIFREQ);
}
#endif

/****************************************************************************
 * Name: ssd1351_deselect
 *
 * Description:
 *   De-select the SPI.
 *
 ****************************************************************************/

#if defined(CONFIG_SSD1351_SPI3WIRE) || defined(CONFIG_SSD1351_SPI4WIRE)
static void ssd1351_deselect(FAR struct ssd1351_dev_s *priv)
{
  FAR struct spi_dev_s *spi = priv->spi;

  /* De-select the chip and relinquish the SPI bus */

  ginfo("DE-SELECTED\n");

  SPI_SELECT(spi, SPIDEV_DISPLAY(0), false);
  SPI_LOCK(spi, false);
}
#endif

/****************************************************************************
 * Name: ssd1351_read
 *
 * Description:
 *   Send a 1-byte command and read datlen data bytes.
 *
 ****************************************************************************/

#if defined(CONFIG_SSD1351_PARALLEL8BIT) && !defined(CONFIG_LCD_NOGETRUN)
static void ssd1351_read(FAR struct ssd1351_dev_s *priv, uint8_t cmd,
                         FAR uint8_t *data, size_t datlen)
{
  FAR struct ssd1351_lcd_s *lcd = priv->lcd;
  size_t i;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT((data == NULL && datlen == 0) || (data != NULL && datlen > 0));

  /* Send the command */

  lcd->cmd(lcd, cmd);

  /* Discard the first data read if reading from the display */

  if (cmd == SSD1351_CMD_RAMREAD)
    {
      lcd->read(lcd);
    }

  /* Read all of the data */

  for (i = 0; i < datlen; i++)
    {
      data[i] = lcd->read(lcd);
    }
}
#endif

/****************************************************************************
 * Name: ssd1351_write
 *
 * Description:
 *   Send a 1-byte command followed by datlen data bytes.
 *
 ****************************************************************************/

#ifdef CONFIG_SSD1351_PARALLEL8BIT
static void ssd1351_write(FAR struct ssd1351_dev_s *priv, uint8_t cmd,
                          FAR const uint8_t *data, size_t datlen)
{
  FAR struct ssd1351_lcd_s *lcd = priv->lcd;
  size_t i;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT((data == NULL && datlen == 0) || (data != NULL && datlen > 0));

  /* Send the command */

  lcd->cmd(lcd, cmd);

  /* Write all of the data */

  for (i = 0; i < datlen; i++)
    {
      lcd->write(lcd, data[i]);
    }
}
#elif defined(CONFIG_SSD1351_SPI3WIRE)
static void ssd1351_write(FAR struct ssd1351_dev_s *priv, uint8_t cmd,
                          FAR const uint8_t *data, size_t datlen)
{
  size_t i;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT((data == NULL && datlen == 0) || (data != NULL && datlen > 0));
  DEBUGASSERT(datlen <= SSD1351_STRIDE);

  /* Copy the command into the line buffer */

  priv->rowbuffer[0] = (uint16_t)cmd | SSD1351_SPICMD;

  /* Copy any data after the command into the line buffer */

  for (i = 0; i < datlen; i++)
    {
      priv->rowbuffer[i + 1] = (uint16_t)data[i] | SSD1351_SPIDATA;
    }

  /* Send the line buffer */

  SPI_SNDBLOCK(priv->spi, priv->rowbuffer, datlen + 1);
}
#elif defined(CONFIG_SSD1351_SPI4WIRE)
static void ssd1351_write(FAR struct ssd1351_dev_s *priv, uint8_t cmd,
                          FAR const uint8_t *data, size_t datlen)
{
  FAR struct spi_dev_s *spi = priv->spi;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT((data == NULL && datlen == 0) || (data != NULL && datlen > 0));

  /* Select command transfer */

  SPI_CMDDATA(spi, SPIDEV_DISPLAY(0), true);

  /* Send the command */

  SPI_SEND(spi, cmd);

  /* Do we have any data to send? */

  if (datlen > 0)
    {
      /* Yes, select data transfer */

      SPI_CMDDATA(spi, SPIDEV_DISPLAY(0), false);

      /* Transfer all of the data */

      SPI_SNDBLOCK(spi, data, datlen);
    }
}
#endif

/****************************************************************************
 * Name: ssd1351_setcursor
 *
 * Description:
 *   Set the cursor position.
 *
 ****************************************************************************/

static void ssd1351_setcursor(FAR struct ssd1351_dev_s *priv, uint8_t col,
                              uint8_t row)
{
  uint8_t buf[2];

#if defined(CONFIG_LCD_LANDSCAPE) || defined(CONFIG_LCD_RLANDSCAPE)
  /* Set the column address to the column */

  buf[0] = col;
  buf[1] = SSD1351_XRES - 1;
  ssd1351_write(priv, SSD1351_CMD_COLADDR, buf, 2);

  /* Set the row address to the row */

  buf[0] = row;
  buf[1] = SSD1351_YRES - 1;
  ssd1351_write(priv, SSD1351_CMD_ROWADDR, buf, 2);
#elif defined(CONFIG_LCD_PORTRAIT) || defined(CONFIG_LCD_RPORTRAIT)
  /* Set the column address to the row */

  buf[0] = row;
  buf[1] = SSD1351_YRES - 1;
  ssd1351_write(priv, SSD1351_CMD_COLADDR, buf, 2);

  /* Set the row address to the column */

  buf[0] = col;
  buf[1] = SSD1351_XRES - 1;
  ssd1351_write(priv, SSD1351_CMD_ROWADDR, buf, 2);
#endif
}

/****************************************************************************
 * Name: ssd1351_putrun
 *
 * Description:
 *   This method can be used to write a partial raster line to the LCD:
 *
 * Input Parameters:
 *   row     - Starting row to write to (range: 0 <= row < yres)
 *   col     - Starting column to write to (range: 0 <= col <= xres-npixels
 *   buffer  - The buffer containing the run to be written to the LCD
 *   npixels - The number of pixels to write to the LCD
 *             (range: 0 < npixels <= xres-col)
 *
 ****************************************************************************/

static int ssd1351_putrun(fb_coord_t row, fb_coord_t col,
                          FAR const uint8_t *buffer, size_t npixels)
{
  FAR struct ssd1351_dev_s *priv = &g_lcddev;

  /* Sanity check */

  DEBUGASSERT(buffer != NULL && ((uintptr_t)buffer & 1) == 0 &&
              col >= 0 && col + npixels <= SSD1351_XRES &&
              row >= 0 && row < SSD1351_YRES);

  /* Select and lock the device */

  ssd1351_select(priv);

  /* Set the starting position for the run */

  ssd1351_setcursor(priv, col, row);

  /* Write all of the data */

  ssd1351_write(priv, SSD1351_CMD_RAMWRITE, buffer,
                SSD1351_PIX2BYTES(npixels));

  /* Unlock and de-select the device */

  ssd1351_deselect(priv);

  return OK;
}

/****************************************************************************
 * Name: ssd1351_getrun
 *
 * Description:
 *   This method can be used to read a partial raster line from the LCD.
 *
 * Input Parameters:
 *   row     - Starting row to read from (range: 0 <= row < yres)
 *   col     - Starting column to read from (range: 0 <= col <= xres-npixels)
 *   buffer  - The buffer in which to return the run read from the LCD
 *   npixels - The number of pixels to read from the LCD
 *             (range: 0 < npixels <= xres-col)
 *
 ****************************************************************************/

static int ssd1351_getrun(fb_coord_t row, fb_coord_t col,
                          FAR uint8_t *buffer, size_t npixels)
{
#if defined(CONFIG_SSD1351_PARALLEL8BIT) && !defined(CONFIG_LCD_NOGETRUN)
  FAR struct ssd1351_dev_s *priv = &g_lcddev;

  /* Sanity check */

  DEBUGASSERT(buffer != NULL && ((uintptr_t)buffer & 1) == 0 &&
              col >= 0 && col + npixels <= SSD1351_XRES &&
              row >= 0 && row < SSD1351_YRES);

  /* Select and lock the device */

  ssd1351_select(priv);

  /* Set the starting position for the run */

  ssd1351_setcursor(priv, col, row);

  /* Read all of the data */

  ssd1351_read(priv, SSD1351_CMD_RAMREAD, buffer,
               SSD1351_PIX2BYTES(npixels));

  /* Unlock and de-select the device */

  ssd1351_deselect(priv);

  return OK;
#else
  return -ENOSYS;
#endif
}

/****************************************************************************
 * Name: ssd1351_getvideoinfo
 *
 * Description:
 *   Get information about the LCD video controller configuration.
 *
 ****************************************************************************/

static int ssd1351_getvideoinfo(FAR struct lcd_dev_s *dev,
                                FAR struct fb_videoinfo_s *vinfo)
{
  DEBUGASSERT(dev != NULL && vinfo != NULL);

  vinfo->fmt     = SSD1351_COLORFMT;
  vinfo->xres    = SSD1351_XRES;
  vinfo->yres    = SSD1351_YRES;
  vinfo->nplanes = 1;

  ginfo("fmt: %u xres: %u yres: %u nplanes: %u\n",
        vinfo->fmt, vinfo->xres, vinfo->yres, vinfo->nplanes);
  return OK;
}

/****************************************************************************
 * Name: ssd1351_getplaneinfo
 *
 * Description:
 *   Get information about the configuration of each LCD color plane.
 *
 ****************************************************************************/

static int ssd1351_getplaneinfo(FAR struct lcd_dev_s *dev,
                                unsigned int planeno,
                                FAR struct lcd_planeinfo_s *pinfo)
{
  FAR struct ssd1351_dev_s *priv = (FAR struct ssd1351_dev_s *)dev;

  DEBUGASSERT(dev != NULL && pinfo != NULL && planeno == 0);

  pinfo->putrun = ssd1351_putrun;
  pinfo->getrun = ssd1351_getrun;
  pinfo->buffer = (uint8_t *)priv->runbuffer;
  pinfo->bpp    = SSD1351_BPP;

  ginfo("planeno: %u bpp: %u\n", planeno, pinfo->bpp);
  return OK;
}

/****************************************************************************
 * Name: ssd1351_getpower
 *
 * Description:
 *   Get the LCD panel power status
 *   (0: full off - CONFIG_LCD_MAXPOWER: full on).
 *   On backlit LCDs, this setting may correspond to the backlight setting.
 *
 ****************************************************************************/

static int ssd1351_getpower(FAR struct lcd_dev_s *dev)
{
  FAR struct ssd1351_dev_s *priv = (FAR struct ssd1351_dev_s *)dev;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);
  ginfo("power: %d\n", priv->power);

  return priv->power;
}

/****************************************************************************
 * Name: ssd1351_setpower
 *
 * Description:
 *   Enable/disable LCD panel power
 *   (0: full off - CONFIG_LCD_MAXPOWER: full on).
 *   On backlit LCDs, this setting may correspond to the backlight setting.
 *
 ****************************************************************************/

static int ssd1351_setpower(FAR struct lcd_dev_s *dev, int power)
{
  FAR struct ssd1351_dev_s *priv = (FAR struct ssd1351_dev_s *)dev;

  /* Sanity check */

  DEBUGASSERT(priv != NULL && (unsigned int)power <= LCD_FULL_ON);
  ginfo("power: %d\n", power);

  /* Select and lock the device */

  ssd1351_select(priv);

  if (power > LCD_FULL_OFF)
    {
      /* Turn the display on */

      ssd1351_write(priv, SSD1351_CMD_DISPON, NULL, 0);
      priv->power = LCD_FULL_ON;
    }
  else
    {
      /* Turn the display off */

      ssd1351_write(priv, SSD1351_CMD_DISPOFF, NULL, 0);
      priv->power = LCD_FULL_OFF;
    }

  /* Unlock and de-select the device */

  ssd1351_deselect(priv);

  return OK;
}

/****************************************************************************
 * Name: ssd1351_getcontrast
 *
 * Description:
 *   Get the current contrast setting (0-CONFIG_LCD_MAXCONTRAST).
 *
 ****************************************************************************/

static int ssd1351_getcontrast(FAR struct lcd_dev_s *dev)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: ssd1351_setcontrast
 *
 * Description:
 *   Set LCD panel contrast (0-CONFIG_LCD_MAXCONTRAST).
 *
 ****************************************************************************/

static int ssd1351_setcontrast(FAR struct lcd_dev_s *dev,
                               unsigned int contrast)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: ssd1351_hwinitialize
 *
 * Description:
 *   Initialize the video hardware.
 *
 ****************************************************************************/

static inline void ssd1351_hwinitialize(FAR struct ssd1351_dev_s *priv)
{
  size_t i;
  uint8_t buf[3];

  /* Select and lock the device */

  ssd1351_select(priv);

  /* Unlock most commands */

  buf[0] = SSD1351_UNLOCK;
  ssd1351_write(priv, SSD1351_CMD_LOCK, buf, 1);

  /* Unlock the rest of the commands */

  buf[0] = SSD1351_ACCESSIBLE;
  ssd1351_write(priv, SSD1351_CMD_LOCK, buf, 1);

  /* Turn the display off */

  ssd1351_write(priv, SSD1351_CMD_DISPOFF, NULL, 0);

  /* Set the address increment, the column address mapping, the color
   * sequence, the scan direction, the COM split, and the color depth
   */

  buf[0] = SSD1351_COLORABC | SSD1351_SPLITEN | SSD1351_DEPTH65K;
#if defined(CONFIG_LCD_LANDSCAPE) || defined(CONFIG_LCD_RLANDSCAPE)
  buf[0] |= SSD1351_ADDRINCHORIZ;
#else
  buf[0] |= SSD1351_ADDRINCVERT;
#endif
#if (defined(CONFIG_LCD_LANDSCAPE)  && !defined(CONFIG_SSD1351_MIRRORX)) || \
    (defined(CONFIG_LCD_RLANDSCAPE) &&  defined(CONFIG_SSD1351_MIRRORX)) || \
    (defined(CONFIG_LCD_PORTRAIT)   && !defined(CONFIG_SSD1351_MIRRORY)) || \
    (defined(CONFIG_LCD_RPORTRAIT)  &&  defined(CONFIG_SSD1351_MIRRORY))
  buf[0] |= SSD1351_REMAPCOL0;
#else
  buf[0] |= SSD1351_REMAPCOL127;
#endif
#if (defined(CONFIG_LCD_LANDSCAPE)  && !defined(CONFIG_SSD1351_MIRRORY)) || \
    (defined(CONFIG_LCD_RLANDSCAPE) &&  defined(CONFIG_SSD1351_MIRRORY)) || \
    (defined(CONFIG_LCD_PORTRAIT)   &&  defined(CONFIG_SSD1351_MIRRORX)) || \
    (defined(CONFIG_LCD_RPORTRAIT)  && !defined(CONFIG_SSD1351_MIRRORX))
  buf[0] |= SSD1351_SCANTOCOM0;
#else
  buf[0] |= SSD1351_SCANFROMCOM0;
#endif
  ssd1351_write(priv, SSD1351_CMD_ORIENTATION, buf, 1);

  /* Set the vertical scroll by RAM */

#if (defined(CONFIG_LCD_LANDSCAPE)  && !defined(CONFIG_SSD1351_MIRRORY)) || \
    (defined(CONFIG_LCD_RLANDSCAPE) &&  defined(CONFIG_SSD1351_MIRRORY)) || \
    (defined(CONFIG_LCD_PORTRAIT)   &&  defined(CONFIG_SSD1351_MIRRORX)) || \
    (defined(CONFIG_LCD_RPORTRAIT)  && !defined(CONFIG_SSD1351_MIRRORX))
  buf[0] = SSD1351_STARTLINE(CONFIG_SSD1351_YRES);
#else
  buf[0] = SSD1351_STARTLINE(0);
#endif
  ssd1351_write(priv, SSD1351_CMD_STARTLINE, buf, 1);

  /* Set the vertical scroll by row */

  buf[0] = SSD1351_OFFSET(0);
  ssd1351_write(priv, SSD1351_CMD_OFFSET, buf, 1);

  /* Set the display to normal or inverse */

#ifdef CONFIG_SSD1351_INVERT
  ssd1351_write(priv, SSD1351_CMD_INVERSE, NULL, 0);
#else
  ssd1351_write(priv, SSD1351_CMD_NORMAL, NULL, 0);
#endif

  /* Set the VDD and the interface */

#ifdef CONFIG_SSD1351_VDDEXT
  buf[0] = SSD1351_VDDEXT;
#else
  buf[0] = SSD1351_VDDINT;
#endif
  buf[0] |= SSD1351_IFACE8BIT;
  ssd1351_write(priv, SSD1351_CMD_VDDIFACE, buf, 1);

  /* Set the reset period and the first pre-charge period */

  buf[0] = SSD1351_TRST(CONFIG_SSD1351_TRST) |
           SSD1351_TPRECHG1(CONFIG_SSD1351_TPRECHG1);
  ssd1351_write(priv, SSD1351_CMD_TRSTTPRECHG1, buf, 1);

  /* Set the display performance */

#ifdef CONFIG_SSD1351_PERFENHANCE
  buf[0] = SSD1351_PERFENHANCE;
#else
  buf[0] = SSD1351_PERFNORMAL;
#endif
  buf[1] = SSD1351_PERFDATA2;
  buf[2] = SSD1351_PERFDATA3;
  ssd1351_write(priv, SSD1351_CMD_PERF, buf, 3);

  /* Set the clock divider and the oscillator frequency */

  buf[0] = SSD1351_CLKDIV(CONFIG_SSD1351_CLKDIV) |
           SSD1351_OSCFREQ(CONFIG_SSD1351_OSCFREQ);
  ssd1351_write(priv, SSD1351_CMD_DIVFREQ, buf, 1);

  /* Set the segment low voltage */

  buf[0] = SSD1351_VSLEXT;
  buf[1] = SSD1351_VSLDATA2;
  buf[2] = SSD1351_VSLDATA3;
  ssd1351_write(priv, SSD1351_CMD_VSL, buf, 3);

  /* Set the GPIO pins */

  buf[0] = SSD1351_GPIO0(SSD1351_GPIOLOW) | SSD1351_GPIO1(SSD1351_GPIOLOW);
  ssd1351_write(priv, SSD1351_CMD_GPIO, buf, 1);

  /* Set the second pre-charge period */

  buf[0] = SSD1351_TPRECHG2(CONFIG_SSD1351_TPRECHG2);
  ssd1351_write(priv, SSD1351_CMD_TPRECHG2, buf, 1);

  /* Use the built-in linear lookup table */

  ssd1351_write(priv, SSD1351_CMD_LINEARLUT, NULL, 0);

  /* Set the pre-charge voltage level */

  buf[0] = SSD1351_VPRECHG(CONFIG_SSD1351_VPRECHG);
  ssd1351_write(priv, SSD1351_CMD_VPRECHG, buf, 1);

  /* Set the COM deselect voltage level */

  buf[0] = SSD1351_VCOMH(CONFIG_SSD1351_VCOMH);
  ssd1351_write(priv, SSD1351_CMD_VCOMH, buf, 1);

  /* Set the contrast */

  buf[0] = SSD1351_CONTRAST(CONFIG_SSD1351_CONTRASTA);
  buf[1] = SSD1351_CONTRAST(CONFIG_SSD1351_CONTRASTB);
  buf[2] = SSD1351_CONTRAST(CONFIG_SSD1351_CONTRASTC);
  ssd1351_write(priv, SSD1351_CMD_CONTRAST, buf, 3);

  /* Set the master contrast ratio */

  buf[0] = SSD1351_MSTRCONTRAST(CONFIG_SSD1351_MSTRCONTRAST);
  ssd1351_write(priv, SSD1351_CMD_MSTRCONTRAST, buf, 1);

  /* Set the multiplex ratio */

  buf[0] = SSD1351_MUXRATIO(128);
  ssd1351_write(priv, SSD1351_CMD_MUXRATIO, buf, 1);

  /* Lock some of the commands */

  buf[0] = SSD1351_INACCESSIBLE;
  ssd1351_write(priv, SSD1351_CMD_LOCK, buf, 1);

  /* Set the cursor position */

  ssd1351_setcursor(priv, 0, 0);

  /* Clear the display memory */

  buf[0] = 0;
  buf[1] = 0;
  for (i = 0; i < SSD1351_XRES * SSD1351_YRES; i++)
    {
      ssd1351_write(priv, SSD1351_CMD_RAMWRITE, buf, 2);
    }

  /* Unlock and de-select the device */

  ssd1351_deselect(priv);
}

/****************************************************************************
 * Name: ssd1351_initialize
 *
 * Description:
 *   Initialize the video hardware.  The initial state of the device
 *   is fully initialized, display memory cleared, and ready to use,
 *   but with the power setting at 0 (full off == sleep mode).
 *
 * Input Parameters:
 *   lcd   - A reference to the platform-specific interface.
 *   spi   - A reference to the SPI driver instance.
 *   devno - A value in the range of 0 through CONFIG_SSD1351_NINTERFACES-1.
 *           This allows support for multiple devices.
 *
 * Returned Value:
 *   On success, this function returns a reference to the LCD object for the
 *   specified device.  NULL is returned on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SSD1351_PARALLEL8BIT
FAR struct lcd_dev_s *ssd1351_initialize(FAR struct ssd1351_lcd_s *lcd,
                                         unsigned int devno)
#elif defined(CONFIG_SSD1351_SPI3WIRE) || defined(CONFIG_SSD1351_SPI4WIRE)
FAR struct lcd_dev_s *ssd1351_initialize(FAR struct spi_dev_s *spi,
                                         unsigned int devno)
#endif
{
  FAR struct ssd1351_dev_s *priv = &g_lcddev;

  /* Sanity check */

#ifdef CONFIG_SSD1351_PARALLEL8BIT
  DEBUGASSERT(lcd != NULL);
#elif defined(CONFIG_SSD1351_SPI3WIRE) || defined(CONFIG_SSD1351_SPI4WIRE)
  DEBUGASSERT(spi != NULL);
#endif
  DEBUGASSERT(devno == 0);

  /* Initialize the driver data structure */

  priv->dev.getvideoinfo = ssd1351_getvideoinfo;
  priv->dev.getplaneinfo = ssd1351_getplaneinfo;
  priv->dev.getpower     = ssd1351_getpower;
  priv->dev.setpower     = ssd1351_setpower;
  priv->dev.getcontrast  = ssd1351_getcontrast;
  priv->dev.setcontrast  = ssd1351_setcontrast;
#ifdef CONFIG_SSD1351_PARALLEL8BIT
  priv->lcd              = lcd;
#elif defined(CONFIG_SSD1351_SPI3WIRE) || defined(CONFIG_SSD1351_SPI4WIRE)
  priv->spi              = spi;
#endif
  priv->power            = LCD_FULL_OFF;

  /* Configure the device */

  ssd1351_hwinitialize(priv);

  return &priv->dev;
}

#endif /* CONFIG_LCD_SSD1351 */
