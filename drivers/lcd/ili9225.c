/****************************************************************************
 * drivers/lcd/ili9225.c
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
#include <nuttx/lcd/ili9225.h>

#ifdef CONFIG_LCD_ILI9225

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Verify that all configuration requirements have been met */

#ifndef CONFIG_LCD_ILI9225_SPIMODE
#  define CONFIG_LCD_ILI9225_SPIMODE SPIDEV_MODE0
#endif

/* SPI frequency */

#ifndef CONFIG_LCD_ILI9225_FREQUENCY
#  define CONFIG_LCD_ILI9225_FREQUENCY 1000000
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

#if !defined(CONFIG_LCD_ILI9225_XRES)
#  define CONFIG_LCD_ILI9225_XRES 176
#endif

#if !defined(CONFIG_LCD_ILI9225_YRES)
#  define CONFIG_LCD_ILI9225_YRES 220
#endif

#define ILI9225_LUT_SIZE    CONFIG_LCD_ILI9225_YRES

#if defined(CONFIG_LCD_LANDSCAPE) || defined(CONFIG_LCD_RLANDSCAPE)
#  define ILI9225_XRES       CONFIG_LCD_ILI9225_YRES
#  define ILI9225_YRES       CONFIG_LCD_ILI9225_XRES
#else
#  define ILI9225_XRES       CONFIG_LCD_ILI9225_XRES
#  define ILI9225_YRES       CONFIG_LCD_ILI9225_YRES
#endif

/* Color depth and format */

#ifdef CONFIG_LCD_ILI9225_BPP
#  if (CONFIG_LCD_ILI9225_BPP == 16)
#    define ILI9225_BPP           16
#    define ILI9225_COLORFMT      FB_FMT_RGB16_565
#    define ILI9225_BYTESPP       2
#  else
#    define ILI9225_BPP           16
#    define ILI9225_COLORFMT      FB_FMT_RGB16_565
#    define ILI9225_BYTESPP       2
#    warning "Invalid color depth.  Falling back to 16bpp"
#  endif
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the state of this driver */

struct ili9225_dev_s
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

  uint16_t runbuffer[ILI9225_LUT_SIZE];
};

/****************************************************************************
 * Private Function Protototypes
 ****************************************************************************/

/* Misc. Helpers */

static void ili9225_select(FAR struct spi_dev_s *spi, int bits);
static void ili9225_deselect(FAR struct spi_dev_s *spi);

static inline void ili9225_sendcmd(FAR struct ili9225_dev_s *dev,
                                    uint16_t cmd);
static inline void ili9225_writereg(FAR struct ili9225_dev_s *dev,
                                    uint16_t reg, uint16_t data);
static void ili9225_display(FAR struct ili9225_dev_s *dev, bool on);
static void ili9225_setarea(FAR struct ili9225_dev_s *dev,
                           uint16_t x0, uint16_t y0,
                           uint16_t x1, uint16_t y1);
static void ili9225_bpp(FAR struct ili9225_dev_s *dev, int bpp);
static void ili9225_wrram(FAR struct ili9225_dev_s *dev,
                         FAR const uint16_t *buff, size_t size);
#ifndef CONFIG_LCD_NOGETRUN
static void ili9225_rdram(FAR struct ili9225_dev_s *dev,
                         FAR uint16_t *buff, size_t size);
#endif
static void ili9225_fill(FAR struct ili9225_dev_s *dev, uint16_t color);

/* LCD Data Transfer Methods */

static int ili9225_putrun(FAR struct lcd_dev_s *dev,
                          fb_coord_t row, fb_coord_t col,
                          FAR const uint8_t *buffer, size_t npixels);
#ifndef CONFIG_LCD_NOGETRUN
static int ili9225_getrun(FAR struct lcd_dev_s *dev,
                          fb_coord_t row, fb_coord_t col,
                          FAR uint8_t *buffer, size_t npixels);
#endif

/* LCD Configuration */

static int ili9225_getvideoinfo(FAR struct lcd_dev_s *dev,
                               FAR struct fb_videoinfo_s *vinfo);
static int ili9225_getplaneinfo(FAR struct lcd_dev_s *dev,
                               unsigned int planeno,
                               FAR struct lcd_planeinfo_s *pinfo);

/* LCD Specific Controls */

static int ili9225_getpower(FAR struct lcd_dev_s *dev);
static int ili9225_setpower(FAR struct lcd_dev_s *dev, int power);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct ili9225_dev_s g_lcddev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ili9225_select
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

static void ili9225_select(FAR struct spi_dev_s *spi, int bits)
{
  /* Select ILI9225 chip (locking the SPI bus in case there are multiple
   * devices competing for the SPI bus
   */

  SPI_LOCK(spi, true);
  SPI_SELECT(spi, SPIDEV_DISPLAY(0), true);

  /* Now make sure that the SPI bus is configured for the ILI9225 (it
   * might have gotten configured for a different device while unlocked)
   */

  SPI_SETMODE(spi, CONFIG_LCD_ILI9225_SPIMODE);
  SPI_SETBITS(spi, bits);
  SPI_SETFREQUENCY(spi, CONFIG_LCD_ILI9225_FREQUENCY);
}

/****************************************************************************
 * Name: ili9225_deselect
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

static void ili9225_deselect(FAR struct spi_dev_s *spi)
{
  /* De-select ILI9225 chip and relinquish the SPI bus. */

  SPI_SELECT(spi, SPIDEV_DISPLAY(0), false);
  SPI_LOCK(spi, false);
}

/****************************************************************************
 * Name: ili9225_sendcmd
 *
 * Description:
 *   Write data into register.
 *
 ****************************************************************************/

static inline void ili9225_writereg(FAR struct ili9225_dev_s *dev,
                                    uint16_t reg, uint16_t data)
{
  ili9225_select(dev->spi, ILI9225_BYTESPP * 8);
  SPI_CMDDATA(dev->spi, SPIDEV_DISPLAY(0), true);
  SPI_SEND(dev->spi, reg);
  SPI_CMDDATA(dev->spi, SPIDEV_DISPLAY(0), false);
  SPI_SEND(dev->spi, data);
  ili9225_deselect(dev->spi);
}

/****************************************************************************
 * Name: ili9225_sendcmd
 *
 * Description:
 *   Send a data to the driver.
 *
 ****************************************************************************/

static inline void ili9225_sendcmd(FAR struct ili9225_dev_s *dev,
                                   uint16_t cmd)
{
  ili9225_select(dev->spi, ILI9225_BYTESPP * 8);
  SPI_CMDDATA(dev->spi, SPIDEV_DISPLAY(0), true);
  SPI_SEND(dev->spi, cmd);
  SPI_CMDDATA(dev->spi, SPIDEV_DISPLAY(0), false);
  ili9225_deselect(dev->spi);
}

/****************************************************************************
 * Name: ili9225_display
 *
 * Description:
 *   Turn on or off the display.
 *
 ****************************************************************************/

static void ili9225_display(FAR struct ili9225_dev_s *dev, bool on)
{
  if (!on)
    {
      /* Start turn off sequence */

      ili9225_writereg(dev, ILI9225_DISP_CTRL1, ILI9225_DISP_CTRL1_GON |
                                                ILI9225_DISP_CTRL1_D(2));
      up_mdelay(20);
      ili9225_writereg(dev, ILI9225_DISP_CTRL1, 0);
      up_mdelay(20);
      ili9225_writereg(dev, ILI9225_POWER_CTRL1, ILI9225_POWER_CTRL1_SAP(0));
      ili9225_writereg(dev, ILI9225_POWER_CTRL2, ~ILI9225_POWER_CTRL2_PON);
      ili9225_writereg(dev, ILI9225_POWER_CTRL5, 0);
    }
  else
    {
      /* Turn on sequence and settings */

      ili9225_writereg(dev, ILI9225_POWER_CTRL2, ILI9225_POWER_CTRL2_VC(8) |
                                                 ILI9225_POWER_CTRL2_VC_EN
                                                 );
      ili9225_writereg(dev, ILI9225_POWER_CTRL3, ILI9225_POWER_CTRL3_DC3(1) |
                                                 ILI9225_POWER_CTRL3_DC2(2) |
                                                 ILI9225_POWER_CTRL3_DC1(1) |
                                                 ILI9225_POWER_CTRL3_BT(6)
                                                 );
      ili9225_writereg(dev, ILI9225_POWER_CTRL4,
                            ILI9225_POWER_CTRL4_GVD(111)
                            );
      ili9225_writereg(dev, ILI9225_POWER_CTRL5,
                            ILI9225_POWER_CTRL5_VML(95) |
                            ILI9225_POWER_CTRL5_VCM(9)  |
                            ILI9225_POWER_CTRL5_VCOMG
                            );
      ili9225_writereg(dev, ILI9225_POWER_CTRL1, ILI9225_POWER_CTRL1_SAP(8));
      ili9225_writereg(dev, ILI9225_POWER_CTRL2, ILI9225_POWER_CTRL2_VC(11) |
                                                 ILI9225_POWER_CTRL2_VC_EN  |
                                                 ILI9225_POWER_CTRL2_AON    |
                                                 ILI9225_POWER_CTRL2_APON
                                                 );
      ili9225_writereg(dev,  ILI9225_DRIVER_OUTPUT_CTRL,
                             ILI9225_DRIVER_OUTPUT_CTRL_NL(28) |
                             ILI9225_DRIVER_OUTPUT_CTRL_SS
                             );
      ili9225_writereg(dev, ILI9225_LCD_DRIVING_CTRL,
                            ILI9225_LCD_DRIVING_CTRL_INV0);
      ili9225_writereg(dev, ILI9225_ENTRY_MODE, ILI9225_ENTRY_MODE_AM |
                                                ILI9225_ENTRY_MODE_ID(3) |
                                                ILI9225_ENTRY_MODE_BGR
                                                );
      ili9225_writereg(dev, ILI9225_DISP_CTRL2, ILI9225_DISP_CTRL2_BP(8) |
                                                ILI9225_DISP_CTRL2_FP(8)
                                                );
      ili9225_writereg(dev, ILI9225_FRAME_CYCLE_CTRL,
                            ILI9225_FRAME_CYCLE_CTRL_STD(1) |
                            ILI9225_FRAME_CYCLE_CTRL_NO(1)
                            );
      ili9225_writereg(dev, ILI9225_RGB_DISP_INT_CTRL1,
                            ILI9225_RGB_DISP_INT_CTRL1_RIM(1));
      ili9225_writereg(dev, ILI9225_OSC_CTRL, ILI9225_OSC_CTRL_EN |
                                              ILI9225_OSC_CTRL_FOSC(13)
                                              );
      ili9225_writereg(dev, ILI9225_VCI_REC, ILI9225_VCI_REC_VCIR(2));

      ili9225_writereg(dev, ILI9225_VER_SCROLL_CTRL1,
                            ILI9225_VER_SCROLL_CTRL1_SEA(219)
                            );
      ili9225_writereg(dev, ILI9225_PART_SCR_DRIV_POS1,
                            ILI9225_PART_SCR_DRIV_POS1_SE(219)
                            );
      ili9225_writereg(dev, ILI9225_HORIZONTAL_ADDR_END,
                            ILI9225_HORIZONTAL_ADDR_END_HEA(175)
                            );
      ili9225_writereg(dev, ILI9225_VERTICAL_ADDR_END,
                            ILI9225_VERTICAL_ADDR_END_VEA(219)
                            );

      ili9225_writereg(dev, ILI9225_GAMMA_CTRL2, ILI9225_GAMMA_CTRL2_KP2(8) |
                                                 ILI9225_GAMMA_CTRL2_KP3(8)
                                                 );
      ili9225_writereg(dev, ILI9225_GAMMA_CTRL3,
                            ILI9225_GAMMA_CTRL3_KP4(10) |
                            ILI9225_GAMMA_CTRL3_KP5(8)
                            );
      ili9225_writereg(dev, ILI9225_GAMMA_CTRL4,
                            ILI9225_GAMMA_CTRL4_RP0(10)
                            );
      ili9225_writereg(dev, ILI9225_GAMMA_CTRL5, ILI9225_GAMMA_CTRL5_KN0(8) |
                                                 ILI9225_GAMMA_CTRL5_KN1(10)
                                                 );
      ili9225_writereg(dev, ILI9225_GAMMA_CTRL6, ILI9225_GAMMA_CTRL6_KN2(8) |
                                                 ILI9225_GAMMA_CTRL6_KN3(8)
                                                 );
      ili9225_writereg(dev, ILI9225_GAMMA_CTRL8,
                            ILI9225_GAMMA_CTRL8_RN1(10)
                            );
      ili9225_writereg(dev, ILI9225_GAMMA_CTRL9,
                            ILI9225_GAMMA_CTRL9_VRP0(16) |
                            ILI9225_GAMMA_CTRL9_VRP1(7)
                            );
      ili9225_writereg(dev, ILI9225_GAMMA_CTRL10,
                            ILI9225_GAMMA_CTRL10_VRN0(10) |
                            ILI9225_GAMMA_CTRL10_VRN1(7)
                            );
      ili9225_writereg(dev, ILI9225_DISP_CTRL1, ILI9225_DISP_CTRL1_D(2) |
                                                ILI9225_DISP_CTRL1_GON
                                                );

      ili9225_writereg(dev, ILI9225_DISP_CTRL1, ILI9225_DISP_CTRL1_D(3) |
                                                ILI9225_DISP_CTRL1_REV  |
                                                ILI9225_DISP_CTRL1_GON  |
                                                ILI9225_DISP_CTRL1_TEMON
                                                );
    }
}

/****************************************************************************
 * Name: ili9225_setarea
 *
 * Description:
 *   Set the rectangular area for an upcoming read or write from RAM.
 *
 ****************************************************************************/

static void ili9225_setarea(FAR struct ili9225_dev_s *dev,
                            uint16_t x0, uint16_t y0,
                            uint16_t x1, uint16_t y1)
{
  /* Set row address */

  ili9225_writereg(dev, ILI9225_HORIZONTAL_ADDR_START,
                   ILI9225_HORIZONTAL_ADDR_START_HSA(x0));
  ili9225_writereg(dev, ILI9225_HORIZONTAL_ADDR_END,
                   ILI9225_HORIZONTAL_ADDR_END_HEA(x1));

  /* Set column address */

  ili9225_writereg(dev, ILI9225_VERTICAL_ADDR_START,
                   ILI9225_VERTICAL_ADDR_START_VSA(y0));
  ili9225_writereg(dev, ILI9225_VERTICAL_ADDR_END,
                   ILI9225_VERTICAL_ADDR_END_VEA(y1));
}

/****************************************************************************
 * Name: ili9225_bpp
 *
 * Description:
 *   Set the color depth of the device.
 *
 ****************************************************************************/

static void ili9225_bpp(FAR struct ili9225_dev_s *dev, int bpp)
{
  if (dev->bpp != bpp)
    {
      dev->bpp = bpp;
    }
}

/****************************************************************************
 * Name: ili9225_wrram
 *
 * Description:
 *   Write to the driver's RAM.
 *
 ****************************************************************************/

static void ili9225_wrram(FAR struct ili9225_dev_s *dev,
                          FAR const uint16_t *buff, size_t size)
{
  ili9225_sendcmd(dev, ILI9225_GRAM_DATA_REG);

  ili9225_select(dev->spi, ILI9225_BYTESPP * 8);
  SPI_SNDBLOCK(dev->spi, buff, size);
  ili9225_deselect(dev->spi);
}

/****************************************************************************
 * Name: ili9225_rdram
 *
 * Description:
 *   Read from the driver's RAM.
 *
 ****************************************************************************/

#ifndef CONFIG_LCD_NOGETRUN
static void ili9225_rdram(FAR struct ili9225_dev_s *dev,
                          FAR uint16_t *buff, size_t size)
{
  ili9225_sendcmd(dev, ILI9225_GRAM_DATA_REG);

  ili9225_select(dev->spi, ILI9225_BYTESPP * 8);
  SPI_RECVBLOCK(dev->spi, buff, size);
  ili9225_deselect(dev->spi);
}
#endif

/****************************************************************************
 * Name: ili9225_fill
 *
 * Description:
 *   Fill the display with the specified color.
 *
 ****************************************************************************/

static void ili9225_fill(FAR struct ili9225_dev_s *dev, uint16_t color)
{
  int i;

  ili9225_setarea(dev, 0, 0, ILI9225_XRES - 1, ILI9225_YRES - 1);
  ili9225_sendcmd(dev, ILI9225_GRAM_DATA_REG);
  ili9225_select(dev->spi, ILI9225_BYTESPP * 8);
  for (i = 0; i < ILI9225_XRES * ILI9225_YRES; i++)
    {
      SPI_SEND(dev->spi, color);
    }

  ili9225_deselect(dev->spi);
}

/****************************************************************************
 * Name:  ili9225_putrun
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

static int ili9225_putrun(FAR struct lcd_dev_s *dev,
                          fb_coord_t row, fb_coord_t col,
                          FAR const uint8_t *buffer, size_t npixels)
{
  FAR struct ili9225_dev_s *priv = (FAR struct ili9225_dev_s *)dev;
  FAR const uint16_t *src = (FAR const uint16_t *)buffer;

  ginfo("row: %d col: %d npixels: %d\n", row, col, npixels);
  DEBUGASSERT(buffer && ((uintptr_t)buffer & 1) == 0);

  ili9225_setarea(priv, col, row, col + npixels - 1, row);
  ili9225_wrram(priv, src, npixels);

  return OK;
}

/****************************************************************************
 * Name:  ili9225_getrun
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
static int ili9225_getrun(FAR struct lcd_dev_s *dev,
                          fb_coord_t row, fb_coord_t col,
                          FAR uint8_t *buffer, size_t npixels)
{
  FAR struct ili9225_dev_s *priv = (FAR struct ili9225_dev_s *)dev;
  FAR uint16_t *dest = (FAR uint16_t *)buffer;

  ginfo("row: %d col: %d npixels: %d\n", row, col, npixels);
  DEBUGASSERT(buffer && ((uintptr_t)buffer & 1) == 0);

  ili9225_setarea(priv, col, row, col + npixels - 1, row);
  ili9225_rdram(priv, dest, npixels);

  return OK;
}
#endif

/****************************************************************************
 * Name:  ili9225_getvideoinfo
 *
 * Description:
 *   Get information about the LCD video controller configuration.
 *
 ****************************************************************************/

static int ili9225_getvideoinfo(FAR struct lcd_dev_s *dev,
                               FAR struct fb_videoinfo_s *vinfo)
{
  DEBUGASSERT(dev && vinfo);
  lcdinfo("fmt: %d xres: %d yres: %d nplanes: 1\n",
          ILI9225_COLORFMT, ILI9225_XRES, ILI9225_YRES);

  vinfo->fmt     = ILI9225_COLORFMT;    /* Color format: RGB16-565: RRRR RGGG GGGB BBBB */
  vinfo->xres    = ILI9225_XRES;        /* Horizontal resolution in pixel columns */
  vinfo->yres    = ILI9225_YRES;        /* Vertical resolution in pixel rows */
  vinfo->nplanes = 1;                   /* Number of color planes supported */
  return OK;
}

/****************************************************************************
 * Name:  ili9225_getplaneinfo
 *
 * Description:
 *   Get information about the configuration of each LCD color plane.
 *
 ****************************************************************************/

static int ili9225_getplaneinfo(FAR struct lcd_dev_s *dev,
                               unsigned int planeno,
                               FAR struct lcd_planeinfo_s *pinfo)
{
  FAR struct ili9225_dev_s *priv = (FAR struct ili9225_dev_s *)dev;

  DEBUGASSERT(dev && pinfo && planeno == 0);
  lcdinfo("planeno: %d bpp: %d\n", planeno, ILI9225_BPP);

  pinfo->putrun = ili9225_putrun;                  /* Put a run into LCD memory */
#ifndef CONFIG_LCD_NOGETRUN
  pinfo->getrun = ili9225_getrun;                  /* Get a run from LCD memory */
#endif
  pinfo->buffer = (FAR uint8_t *)priv->runbuffer; /* Run scratch buffer */
  pinfo->bpp    = priv->bpp;                      /* Bits-per-pixel */
  pinfo->dev    = dev;                            /* The lcd device */
  return OK;
}

/****************************************************************************
 * Name:  ili9225_getpower
 ****************************************************************************/

static int ili9225_getpower(FAR struct lcd_dev_s *dev)
{
  FAR struct ili9225_dev_s *priv = (FAR struct ili9225_dev_s *)dev;

  lcdinfo("power: %d\n", priv->power);
  return priv->power;
}

/****************************************************************************
 * Name:  ili9225_setpower
 ****************************************************************************/

static int ili9225_setpower(FAR struct lcd_dev_s *dev, int power)
{
  FAR struct ili9225_dev_s *priv = (FAR struct ili9225_dev_s *)dev;

  lcdinfo("power: %d\n", power);
  DEBUGASSERT((unsigned)power <= CONFIG_LCD_MAXPOWER);

  /* Set new power level */

  if (power > 0)
    {
      /* Turn on the display */

      ili9225_display(priv, true);

      /* Save the power */

      priv->power = power;
    }
  else
    {
      /* Turn off the display */

      ili9225_display(priv, false);

      /* Save the power */

      priv->power = 0;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  ili9225_initialize
 *
 * Description:
 *   Initialize the ILI9225 video hardware.  The initial state of the
 *   LCD is fully initialized, display memory cleared, and the LCD ready
 *   to use, but with the power setting at 0 (full off == sleep mode).
 *
 * Returned Value:
 *
 *   On success, this function returns a reference to the LCD object for
 *   the specified LCD.  NULL is returned on any failure.
 *
 ****************************************************************************/

FAR struct lcd_dev_s *ili9225_lcdinitialize(FAR struct spi_dev_s *spi)
{
  FAR struct ili9225_dev_s *priv = &g_lcddev;

  /* Initialize the driver data structure */

  priv->dev.getvideoinfo = ili9225_getvideoinfo;
  priv->dev.getplaneinfo = ili9225_getplaneinfo;
  priv->dev.getpower     = ili9225_getpower;
  priv->dev.setpower     = ili9225_setpower;
  priv->spi              = spi;

  /* Init the hardware and clear the display */

  ili9225_display(priv, true);

  ili9225_bpp(priv, ILI9225_BPP);

  /* Set background color */

  ili9225_fill(priv, 0x07ff);

  return &priv->dev;
}

#endif /* CONFIG_LCD_ILI9225 */
