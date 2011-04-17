/**************************************************************************************
 * drivers/lcd/ug-9664hswag01.c
 * Driver for the Univision UG-9664HSWAG01 Display with the Solomon Systech SSD1305 LCD
 * controller.
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * Reference: "Product Specification, OEL Display Module, UG-9664HSWAG01", Univision
 *            Technology Inc., SAS1-6020-B, January 3, 2008.
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
#include <nuttx/spi.h>
#include <nuttx/lcd/lcd.h>

#include "up_arch.h"
#inclu;de "ssd1305.h"

/**************************************************************************************
 * Pre-processor Definitions
 **************************************************************************************/

/* Configuration **********************************************************************/
/* Verify that all configuration requirements have been met */

/* Define the following to enable register-level debug output */

#undef CONFIG_LCD_UGDEBUG

/* Verbose debug must also be enabled */

#ifndef CONFIG_DEBUG
#  undef CONFIG_DEBUG_VERBOSE
#  undef CONFIG_DEBUG_GRAPHICS
#endif

#ifndef CONFIG_DEBUG_VERBOSE
#  undef CONFIG_LCD_UGDEBUG
#endif

/* Color Properties *******************************************************************/
/* The SSD1305 display controller can handle a resolution of 132x64. The OLED
 * on the base board is 96x64.
 */

#define UG_DEV_XRES     132
#define UG_XOFFSET      18

/* Display Resolution */

#define UG_XRES         96
#define UG_YRES         64

/* Color depth and format */

#define UG_BPP          1
#define UG_COLORFMT     FB_FMT_Y1

/* Bytes per visible and actual device row */

#define UG_STRIDE       (UG_XRES >> 3)
#define UG_DEV_STRIDE   (UG_DEV_XRES >> 3)

/* The size of the shadow frame buffer */

#define UG_FBSIZE       (UG_XRES * (UG_XRES >> 3))

/* Debug ******************************************************************************/

#ifdef CONFIG_LCD_UGDEBUG
# define ugdbg(format, arg...)  vdbg(format, ##arg)
#else
# define ugdbg(x...)
#endif

/**************************************************************************************
 * Private Type Definition
 **************************************************************************************/

/* This structure describes the state of this driver */

struct ug_dev_s
{
  /* Publically visible device structure */

  struct lcd_dev_s dev;

  /* Private LCD-specific information follows */

  FAR struct spi_dev_s *spi;
  uint8_t contrast;
};

/**************************************************************************************
 * Private Function Protototypes
 **************************************************************************************/

/* SPI helpers */

#ifdef CONFIG_SPI_OWNBUS
static inline void ug_select(FAR struct spi_dev_s *spi);
static inline void ug_deselect(FAR struct spi_dev_s *spi);
#else
static void ug_select(FAR struct spi_dev_s *spi);
static void ug_deselect(FAR struct spi_dev_s *spi);
#endif

/* LCD Data Transfer Methods */

static int ug_putrun(fb_coord_t row, fb_coord_t col, FAR const uint8_t *buffer,
                     size_t npixels);
static int ug_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t *buffer,
                     size_t npixels);

/* LCD Configuration */

static int ug_getvideoinfo(FAR struct lcd_dev_s *dev,
                           FAR struct fb_videoinfo_s *vinfo);
static int ug_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int planeno,
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

static int ug_getpower(struct lcd_dev_s *dev);
static int ug_setpower(struct lcd_dev_s *dev, int power);
static int ug_getcontrast(struct lcd_dev_s *dev);
static int ug_setcontrast(struct lcd_dev_s *dev, unsigned int contrast);

/* Initialization */

static inline void up_clear(FAR struct ug_dev_s  *priv);

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

static uint8_t g_runbuffer[UG_XRES >> 8];

/* The SSD1305 does not support reading from the display memory in SPI mode.
 * Since there is 1 BPP and access is byte-by-byte, it is necessary to kee
 * a shadow copy of the framebuffer memory.
 */

static uint8_t g_ugfb[UG_XRES >> 3][UG_YRES];

/* This structure describes the overall LCD video controller */

static const struct fb_videoinfo_s g_videoinfo =
{
  .fmt     = UG_COLORFMT,    /* Color format: RGB16-565: RRRR RGGG GGGB BBBB */
  .xres    = UG_XRES,        /* Horizontal resolution in pixel columns */
  .yres    = UG_YRES,        /* Vertical resolution in pixel rows */
  .nplanes = 1,              /* Number of color planes supported */
};

/* This is the standard, NuttX Plane information object */

static const struct lcd_planeinfo_s g_planeinfo = 
{
  .putrun = ug_putrun,             /* Put a run into LCD memory */
  .getrun = ug_getrun,             /* Get a run from LCD memory */
  .buffer = (uint8_t*)g_runbuffer, /* Run scratch buffer */
  .bpp    = UG_BPP,                /* Bits-per-pixel */
};

/* This is the standard, NuttX LCD driver object */

static struct ug_dev_s g_ugdev = 
{
  .dev =
  {
    /* LCD Configuration */
 
    .getvideoinfo = ug_getvideoinfo,
    .getplaneinfo = ug_getplaneinfo,

    /* LCD RGB Mapping -- Not supported */
    /* Cursor Controls -- Not supported */

    /* LCD Specific Controls */

    .getpower     = ug_getpower,
    .setpower     = ug_setpower,
    .getcontrast  = ug_getcontrast,
    .setcontrast  = ug_setcontrast,
  },
};

/**************************************************************************************
 * Private Functions
 **************************************************************************************/

/**************************************************************************************
 * Function: ug_select
 *
 * Description:
 *   Select the SPI, locking and  re-configuring if necessary
 *
 * Parameters:
 *   spi  - Reference to the SPI driver structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 **************************************************************************************/

#ifdef CONFIG_SPI_OWNBUS
static inline void ug_select(FAR struct spi_dev_s *spi)
{
  /* We own the SPI bus, so just select the chip */

  SPI_SELECT(spi, SPIDEV_DISPLAY, true);
}
#else
static void ug_select(FAR struct spi_dev_s *spi)
{
  /* Select P14201 chip (locking the SPI bus in case there are multiple
   * devices competing for the SPI bus
   */

  SPI_LOCK(spi, true);
  SPI_SELECT(spi, SPIDEV_DISPLAY, true);

  /* Now make sure that the SPI bus is configured for the P14201 (it
   * might have gotten configured for a different device while unlocked)
   */

  SPI_SETMODE(spi, CONFIG_UG9664HSWAG01_SPIMODE);
  SPI_SETBITS(spi, 8);
#ifdef CONFIG_UG9664HSWAG01_FREQUENCY
  SPI_SETFREQUENCY(spi, CONFIG_UG9664HSWAG01_FREQUENCY);
#endif
}
#endif

/**************************************************************************************
 * Function: ug_deselect
 *
 * Description:
 *   De-select the SPI
 *
 * Parameters:
 *   spi  - Reference to the SPI driver structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 **************************************************************************************/

#ifdef CONFIG_SPI_OWNBUS
static inline void ug_deselect(FAR struct spi_dev_s *spi)
{
  /* We own the SPI bus, so just de-select the chip */

  SPI_SELECT(spi, SPIDEV_DISPLAY, false);
}
#else
static void ug_deselect(FAR struct spi_dev_s *spi)
{
  /* De-select P14201 chip and relinquish the SPI bus. */

  SPI_SELECT(spi, SPIDEV_DISPLAY, false);
  SPI_LOCK(spi, false);
}
#endif

/**************************************************************************************
 * Name:  ug_putrun
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
 **************************************************************************************/

static int ug_putrun(fb_coord_t row, fb_coord_t col, FAR const uint8_t *buffer,
                       size_t npixels)
{
  /* Because of this line of code, we will only be able to support a single UG device */

  FAR struct ug_dev_s *priv = &g_ugdev;
  uint8_t *rowptr;
  uint8_t devcol;
  uint8_t startcol;
  uint8_t endcol;
  uint8_t addrlo;
  uint8_t addrhi;
  uint8_t page;
  uint8_t i;
  int     pixlen;

  gvdbg("row: %d col: %d npixels: %d\n", row, col, npixels);
  DEBUGASSERT(buffer);

  /* Clip the run to the display */

  pixlen = npixels;
  if ((unsigned int)col + (unsigned int)pixlen > (unsigned int)UG_XRES)
    {
      pixlen = (int)UG_XRES - (int)col;
    }

  /* Verify that some portion of the run remains on the display */

  if (pixlen <= 0 || row >  UG_YRES)
    {
      return;
    }

  /* Update the shadow frame buffer memory */

  rowptr = &g_ugfb[0][row];
  endcol = col + pixlen;

  for (i = col; i < endcol; i++)
    {
      /* Point to the byte to be modified */

      uint8_t *ptr = &rowptr[i >> 8];

      /* Set or clear the corresponding bit */

      uint8_t mask = (1 << (i & 7));
      if (color > 0)
        {
          *ptr |= mask;
        }
      else
        {
          *ptr &= ~mask;
        }
    }

  /* Get the page number.  The range of 64 lines is divided up into eight
   * pages or 8 lines each.
   */

  page = row >> 3;

  /* Offset the column position to account for smaller horizontal
   * display range.
   */

  devcol = col + UG_XOFFSET;

  /* Select and lock the device */

  ug_select(priv->spi);

  /* Select command transfer */

  SPI_CMDDATA(spi, SPIDEV_DISPLAY, true);

  /* Set the starting position for the run */

  (void)SPI_SEND(priv->spi, SSD1305_SETPAGESTART+page);        /* Set the page start */
  (void)SPI_SEND(priv->spi, SSD1305_SETCOLL + (devcol & 0x0f); /* Set the low column */
  (void)SPI_SEND(priv->spi, SSD1305_SETCOLH + (devcol >> 4));  /* Set the high column */

  /* Select data transfer */

  SPI_CMDDATA(spi, SPIDEV_DISPLAY, false);

  /* Then transfer all of the data */

  startcol = col >> 3;
  endcol   = (col + pixlen + 7) >> 3;
  (void)SPI_SNDBLOCK(priv->spi, &rowptr[startcol], endcol - startcol + 1);

  /* Unlock and de-select the device */

  ug_deselect(spi);
  return OK;
}

/**************************************************************************************
 * Name:  ug_getrun
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
 **************************************************************************************/

static int ug_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t *buffer,
                       size_t npixels)
{
  /* Because of this line of code, we will only be able to support a single UG device */

  FAR struct ug_dev_s *priv = &g_ugdev;
  uint8_t *rowptr;
  uint8_t i;
  int     pixlen;

  gvdbg("row: %d col: %d npixels: %d\n", row, col, npixels);
  DEBUGASSERT(buffer);

  /* Clip the run to the display */

  pixlen = npixels;
  if ((unsigned int)col + (unsigned int)pixlen > (unsigned int)UG_XRES)
    {
      pixlen = (int)UG_XRES - (int)col;
    }

   /* Fetch the data from the shadow frame buffer memory */

  rowptr = &g_ugfb[0][row];

  /* Then transfer all of the data */

  startcol = col >> 3;
  endcol   = (col + pixlen + 7) >> 3;
  memcpy(buffer, &rowptr[startcol], endcol - startcol + 1);
  return OK;
}

/**************************************************************************************
 * Name:  ug_getvideoinfo
 *
 * Description:
 *   Get information about the LCD video controller configuration.
 *
 **************************************************************************************/

static int ug_getvideoinfo(FAR struct lcd_dev_s *dev,
                              FAR struct fb_videoinfo_s *vinfo)
{
  DEBUGASSERT(dev && vinfo);
  gvdbg("fmt: %d xres: %d yres: %d nplanes: %d\n",
         g_videoinfo.fmt, g_videoinfo.xres, g_videoinfo.yres, g_videoinfo.nplanes);
  memcpy(vinfo, &g_videoinfo, sizeof(struct fb_videoinfo_s));
  return OK;
}

/**************************************************************************************
 * Name:  ug_getplaneinfo
 *
 * Description:
 *   Get information about the configuration of each LCD color plane.
 *
 **************************************************************************************/

static int ug_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int planeno,
                              FAR struct lcd_planeinfo_s *pinfo)
{
  DEBUGASSERT(dev && pinfo && planeno == 0);
  gvdbg("planeno: %d bpp: %d\n", planeno, g_planeinfo.bpp);
  memcpy(pinfo, &g_planeinfo, sizeof(struct lcd_planeinfo_s));
  return OK;
}

/**************************************************************************************
 * Name:  ug_getpower
 *
 * Description:
 *   Get the LCD panel power status (0: full off - CONFIG_LCD_MAXPOWER: full on. On
 *   backlit LCDs, this setting may correspond to the backlight setting.
 *
 **************************************************************************************/

static int ug_getpower(struct lcd_dev_s *dev)
{
  struct ug_dev_s *priv = (struct ug_dev_s *)dev;
  gvdbg("power: %d\n", 0);
  return 0;
}

/**************************************************************************************
 * Name:  ug_setpower
 *
 * Description:
 *   Enable/disable LCD panel power (0: full off - CONFIG_LCD_MAXPOWER: full on). On
 *   backlit LCDs, this setting may correspond to the backlight setting.
 *
 **************************************************************************************/

static int ug_setpower(struct lcd_dev_s *dev, int power)
{
  struct ug_dev_s *priv = (struct ug_dev_s *)dev;

  gvdbg("power: %d\n", power);
  DEBUGASSERT(power <= CONFIG_LCD_MAXPOWER);

  /* Set new power level */

  return OK;
}

/**************************************************************************************
 * Name:  ug_getcontrast
 *
 * Description:
 *   Get the current contrast setting (0-CONFIG_LCD_MAXCONTRAST).
 *
 **************************************************************************************/

static int ug_getcontrast(struct lcd_dev_s *dev)
{
  struct ug_dev_s *priv = (struct ug_dev_s *)dev;
  DEBUGASSERT(priv);
  return return (int)priv->contrast;
}

/**************************************************************************************
 * Name:  ug_setcontrast
 *
 * Description:
 *   Set LCD panel contrast (0-CONFIG_LCD_MAXCONTRAST).
 *
 **************************************************************************************/

static int ug_setcontrast(struct lcd_dev_s *dev, unsigned int contrast)
{
  struct ug_dev_s *priv = (struct ug_dev_s *)dev;

  gvdbg("contrast: %d\n", contrast);
  DEBUGASSERT(priv);

  if (contrast > 255)
    {
      return -EINVAL;
    }

  /* Select and lock the device */

  ug_select(priv->spi);

  /* Select command transfer */

  SPI_CMDDATA(spi, SPIDEV_DISPLAY, true);

  /* Set the contrast */

  (void)SPI_SEND(priv->spi, SSD1305_SETCONTRAST);  /* Set contrast control register */
  (void)SPI_SEND(priv->spi, contrast);             /* Data 1: Set 1 of 256 contrast steps */
  priv->contrast = contrast;
  
  /* Unlock and de-select the device */

  ug_deselect(spi);
  return OK;
}

/**************************************************************************************
 * Name:  up_clear
 *
 * Description:
 *   Clear the display.
 *
 **************************************************************************************/

static inline void up_clear(FAR struct ug_dev_s  *priv)
{
  FAR struct spi_dev_s *spi  = priv->spi;
  int row;
  int i;
  int j;

  /* Clear the framebuffer */

  memset(g_ugfb, 0, UG_FBSIZE);

  /* Select and lock the device */

  ug_select(priv->spi);

  /* Go through all 8 pages */

  for (row = 0, i = 0; i < 8; i++)
    {
      /* Select command transfer */

      SPI_CMDDATA(spi, SPIDEV_DISPLAY, true);

      /* Set the starting position for the run */

      (void)SPI_SEND(priv->spi, SSD1305_SETPAGESTART+i); /* Set the page start */
      (void)SPI_SEND(priv->spi, SSD1305_SETCOLL);        /* Set the low column=0 */
      (void)SPI_SEND(priv->spi, SSD1305_SETCOLH);        /* Set the high column=0 */

      /* Select data transfer */

      SPI_CMDDATA(spi, SPIDEV_DISPLAY, false);

       /* Then transfer all of the data */

       for (j = 0; j < 8; j++, row++)
         {
           (void)SPI_SNDBLOCK(priv->spi, &g_ugfb[0][row], UG_XRES >> 3);
         }
    }

  /* Unlock and de-select the device */

  ug_deselect(spi);
}

/**************************************************************************************
 * Public Functions
 **************************************************************************************/

/**************************************************************************************
 * Name:  up_oledinitialize
 *
 * Description:
 *   Initialize the LCD video hardware.  The initial state of the LCD is fully
 *   initialized, display memory cleared, and the LCD ready to use, but with the power
 *   setting at 0 (full off).
 *
 **************************************************************************************/

FAR struct lcd_dev_s *up_oledinitialize(FAR struct spi_dev_s *spi)
{
  gvdbg("Initializing\n");

  /* Configure and enable LCD */
 
  FAR struct ug_dev_s  *priv = &g_ugdev;
  FAR struct spi_dev_s *spi  = priv->spi;

  /* Select and lock the device */

  ug_select(spi);

  /* Select command transfer */

  SPI_CMDDATA(spi, SPIDEV_DISPLAY, true);

  /* Set the starting position for the run */

  (void)SPI_SEND(spi, SSD1305_SETCOLL + 2);       /* Set low column address */
  (void)SPI_SEND(spi, SSD1305_SETCOLH + 2);       /* Set high column address */
  (void)SPI_SEND(spi, SSD1305_SETSTARTLINE+0);    /* Display start set */
  (void)SPI_SEND(spi, SSD1305_SCROLL_STOP);       /* Stop horzontal scroll */
  (void)SPI_SEND(spi, SSD1305_SETCONTRAST);       /* Set contrast control register */
  (void)SPI_SEND(spi, 0x32);                      /* Data 1: Set 1 of 256 contrast steps */
  (void)SPI_SEND(spi, SSD1305_SETBRIGHTNESS);     /* Brightness for color bank */
  (void)SPI_SEND(spi, 0x80);                      /* Data 1: Set 1 of 256 contrast steps */
  (void)SPI_SEND(spi, SSD1305_MAPCOL131);         /* Set segment re-map */
  (void)SPI_SEND(spi, SSD1305_DISPNORMAL);        /* Set normal display */
/*(void)SPI_SEND(spi, SSD1305_DISPINVERTED);         Set inverse display */
  (void)SPI_SEND(spi, SSD1305_SETMUX);            /* Set multiplex ratio */
  (void)SPI_SEND(spi, 0x3f);                      /* Data 1: MUX ratio -1: 15-63 */
  (void)SPI_SEND(spi, SSD1305_SETOFFSET);         /* Set display offset */
  (void)SPI_SEND(spi, 0x40);                      /* Data 1: Vertical shift by COM: 0-63 */
  (void)SPI_SEND(spi, SSD1305_MSTRCONFIG);        /* Set dc-dc on/off */
  (void)SPI_SEND(spi, SSD1305_MSTRCONFIG_EXTVCC); /* Data 1: Select external Vcc */
  (void)SPI_SEND(spi, SSD1305_SETCOMREMAPPED);    /* Set com output scan direction */
  (void)SPI_SEND(spi, SSD1305_SETDCLK);           /* Set display clock divide
                                                   * ratio/oscillator/frequency */
  (void)SPI_SEND(spi, 15 << SSD1305_DCLKFREQ_SHIFT | 0 << SSD1305_DCLKDIV_SHIFT);
  (void)SPI_SEND(spi, SSD1305_SETCOLORMODE);      /* Set area color mode on/off & low power
                                                   * display mode */
  (void)SPI_SEND(spi, SSD1305_COLORMODE_MONO | SSD1305_POWERMODE_LOW);
  (void)SPI_SEND(spi, SSD1305_SETPRECHARGE);      /* Set pre-charge period */
  (void)SPI_SEND(spi, 15 << SSD1305_PHASE2_SHIFT | 1 << SSD1305_PHASE1_SHIFT);
  (void)SPI_SEND(spi, SSD1305_COMCONFIG_ALT);     /* Data 1, Bit 4: 1=Alternative COM pin configuration */
  (void)SPI_SEND(spi, SSD1305_SETVCOMHDESEL);     /* Set VCOMH deselect level */
  (void)SPI_SEND(spi, SSD1305_VCOMH_x7p7);        /* Data 1: ~0.77 x Vcc  */
  (void)SPI_SEND(spi, SSD1305_SETLUT);            /* Set look up table for area color */
  (void)SPI_SEND(spi, 0x3f);                      /* Data 1: Pulse width: 31-63 */
  (void)SPI_SEND(spi, 0x3f);                      /* Data 2: Color A: 31-63 */
  (void)SPI_SEND(spi, 0x3f);                      /* Data 3: Color B: 31-63 */
  (void)SPI_SEND(spi, 0x3f);                      /* Data 4: Color C: 31-63 */
  (void)SPI_SEND(spi, SSD1305_DISPON);            /* Display on, normal mode */
  (void)SPI_SEND(spi, SSD1305_DISPRAM);           /* Resume to RAM content display */

  /* Let go of the SPI lock and de-select the device */

  ug_deselect(spi);

  /* Clear the framebuffer */

  up_mdelay(100);
  up_clear(priv);
  return &priv->dev;
}
