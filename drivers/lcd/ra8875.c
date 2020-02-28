/**************************************************************************************
 * drivers/lcd/ra8875.c
 *
 * Driver for the RAiO Technologies RA8875 LCD controller
 *
 *   Copyright (C) 2015 Intuitive Aerial AB. All rights reserved.
 *   Author: Marten Svanfeldt <marten@intuitiveaerial.com>
 *
 * References: RA8875, Rev 1.6, Apr 2013, RAiO Technologies Inc
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
#include <nuttx/spi/spi.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/ra8875.h>

#include "ra8875.h"

#ifdef CONFIG_LCD_RA8875

/**************************************************************************************
 * Pre-processor Definitions
 **************************************************************************************/
/* Configuration **********************************************************************/

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
#  if defined(CONFIG_LCD_LANDSCAPE) || defined(CONFIG_LCD_RLANDSCAPE) || \
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

/* Display/Color Properties ***********************************************************/
/* Display Resolution */

#if defined(CONFIG_RA8875_XRES)
#  define RA8875_XRES       CONFIG_RA8875_XRES
#else
#  if defined(CONFIG_LCD_LANDSCAPE) || defined(CONFIG_LCD_RLANDSCAPE)
#    define RA8875_XRES       800
#  else
#    define RA8875_XRES       480
#  endif
#endif

#if defined(CONFIG_RA8875_YRES)
#  define RA8875_YRES       CONFIG_RA8875_YRES
#else
#  if defined(CONFIG_LCD_LANDSCAPE) || defined(CONFIG_LCD_RLANDSCAPE)
#    define RA8875_YRES       480
#  else
#    define RA8875_YRES       800
#  endif
#endif

#if defined(CONFIG_LCD_LANDSCAPE) || defined(CONFIG_LCD_RLANDSCAPE)
#  define RA8875_HW_XRES RA8875_XRES
#  define RA8875_HW_YRES RA8875_YRES
#else
#  define RA8875_HW_XRES RA8875_YRES
#  define RA8875_HW_YRES RA8875_XRES
#endif

/* Color depth and format */

#if defined(CONFIG_LCD_RA8875_65K)
#  define RA8875_BPP           16
#  define RA8875_COLORFMT      FB_FMT_RGB16_565

#  define RA8875_UNPACK_RED(p)    (((p)>>11) & 0x1f)
#  define RA8875_UNPACK_GREEN(p)  (((p)>>5) & 0x3f)
#  define RA8875_UNPACK_BLUE(p)   ((p) & 0x1f)

#  define RA8875_PACK_RGB(r,g,b) ((r) << 11 | (g) << 5 | (b))
#else /* LCD_RA8875_256 */
#  define RA8875_BPP           8
#  define RA8875_COLORFMT      FB_FMT_RGB8_332

#  define RA8875_UNPACK_RED(p)    (((p)>>5) & 0x7)
#  define RA8875_UNPACK_GREEN(p)  (((p)>>2) & 0x7)
#  define RA8875_UNPACK_BLUE(p)   ((p) & 0x3)

#  define RA8875_PACK_RGB(r,g,b) (((r)&0x7) << 5 | ((g)&0x7) << 2 | ((b)&0x3))
#endif

#if RA8875_HW_XRES >= 800 && RA8875_HW_YRES >= 480 && RA8875_BPP > 8
#  undef RA8875_2LAYER_POSSIBLE
#else
#  define RA8875_2LAYER_POSSIBLE 1
#endif

/**************************************************************************************
 * Private Type Definition
 **************************************************************************************/

/* This structure describes the state of this driver */

struct ra8875_dev_s
{
  /* Publicly visible device structure */

  struct lcd_dev_s dev;

  /* Private LCD-specific information follows */

  FAR struct ra8875_lcd_s *lcd;   /* The contained platform-specific, LCD interface */
  uint8_t power;                  /* Current power setting */

#if defined(RA8875_2LAYER_POSSIBLE)
  uint8_t current_layer;          /* Current drawing layer, 0=disabled */
#endif

  /* Shadow these registers to speed up rendering */
  uint8_t shadow_mwcr0;
  uint16_t shadow_w_curh, shadow_w_curv;

  /* These fields simplify and reduce debug output */

#ifdef CONFIG_DEBUG_LCD
  bool put;                       /* Last raster operation was a putrun */
  fb_coord_t firstrow;            /* First row of the run */
  fb_coord_t lastrow;             /* Last row of the run */
  fb_coord_t col;                 /* Column of the run */
  size_t npixels;                 /* Length of the run */
#endif

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

  uint16_t runbuffer[RA8875_XRES];
};

/**************************************************************************************
 * Private Function Prototypes
 **************************************************************************************/
/* Low Level LCD access */

static inline void ra8875_putreg(FAR struct ra8875_lcd_s *lcd, uint8_t regaddr,
                           uint8_t regval);
static inline void ra8875_putreg16(FAR struct ra8875_lcd_s *lcd, uint8_t regaddr,
                            uint16_t regval);
#ifndef CONFIG_LCD_NOGETRUN
static inline uint8_t ra8875_readreg(FAR struct ra8875_lcd_s *lcd, uint8_t regaddr);
static inline void ra8875_waitreg(FAR struct ra8875_lcd_s *lcd, uint8_t regaddr,
                                  uint8_t mask);
#endif
static void ra8875_set_writecursor(FAR struct ra8875_dev_s *dev, uint16_t column,
                                    uint16_t row);
static void ra8875_set_readcursor(FAR struct ra8875_lcd_s *lcd, uint16_t column,
                                    uint16_t row);
static void ra8875_set_mwcr0(FAR struct ra8875_dev_s *dev, uint8_t value);
static void ra8875_setwindow(FAR struct ra8875_lcd_s *lcd, uint16_t x, uint16_t y,
                              uint16_t width, uint16_t height);
static inline void ra8875_setbackground(FAR struct ra8875_lcd_s *lcd, uint16_t color);
static inline void ra8875_setforeground(FAR struct ra8875_lcd_s *lcd, uint16_t color);
static void ra8875_clearmem(FAR struct ra8875_lcd_s *lcd);

/* LCD Data Transfer Methods */

#if 0 /* Sometimes useful */
static void ra8875_dumprun(FAR const char *msg, FAR uint16_t *run, size_t npixels);
#else
#  define ra8875_dumprun(m,r,n)
#endif

#ifdef CONFIG_DEBUG_LCD
static void ra8875_showrun(FAR struct ra8875_dev_s *priv, fb_coord_t row,
                            fb_coord_t col, size_t npixels, bool put);
#else
#  define ra8875_showrun(p,r,c,n,b)
#endif

static int ra8875_putrun(fb_coord_t row, fb_coord_t col, FAR const uint8_t *buffer,
             size_t npixels);
static int ra8875_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t *buffer,
             size_t npixels);

/* LCD Configuration */

static int ra8875_getvideoinfo(FAR struct lcd_dev_s *dev,
             FAR struct fb_videoinfo_s *vinfo);
static int ra8875_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int planeno,
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

static int ra8875_getpower(FAR struct lcd_dev_s *dev);
static int ra8875_setpower(FAR struct lcd_dev_s *dev, int power);
static int ra8875_getcontrast(FAR struct lcd_dev_s *dev);
static int ra8875_setcontrast(FAR struct lcd_dev_s *dev, unsigned int contrast);

/* Initialization */

static inline int ra8875_hwinitialize(FAR struct ra8875_dev_s *priv);

/**************************************************************************************
 * Private Data
 **************************************************************************************/

/* This driver can support only a signal RA8875 device.  This is due to an
 * unfortunate decision made when the getrun and putrun methods were designed.  The
 * following is the single RA8875 driver state instance:
 */

static struct ra8875_dev_s g_lcddev;

/**************************************************************************************
 * Private Functions
 **************************************************************************************/

/**************************************************************************************
 * Name:  ra8875_putreg(lcd,
 *
 * Description:
 *   Write to an LCD register
 *
 **************************************************************************************/

static void ra8875_putreg(FAR struct ra8875_lcd_s *lcd, uint8_t regaddr,
                          uint8_t regval)
{
  /* Set the index register to the register address and write the register contents */

  lcdinfo("putreg 0x%02x = 0x%02x\n", regaddr, regval);

  lcd->write_reg(lcd, regaddr, regval);
}

/**************************************************************************************
 * Name:  ra8875_putreg16(lcd,
 *
 * Description:
 *   Write to an LCD register
 *
 **************************************************************************************/

static void ra8875_putreg16(FAR struct ra8875_lcd_s *lcd, uint8_t regaddr,
                            uint16_t regval)
{
  /* Set the index register to the register address and write the register contents */

  lcdinfo("putreg 0x%02x = 0x%04x\n", regaddr, regval);

  lcd->write_reg16(lcd, regaddr, regval);
}

/**************************************************************************************
 * Name:  ra8875_readreg
 *
 * Description:
 *   Read from an LCD register
 *
 **************************************************************************************/

#ifndef CONFIG_LCD_NOGETRUN
static uint8_t ra8875_readreg(FAR struct ra8875_lcd_s *lcd, uint8_t regaddr)
{
  uint8_t regval;
  /* Set the index register to the register address and read the register contents */

  regval = lcd->read_reg(lcd, regaddr);

  lcdinfo("readreg 0x%02x = 0x%02x\n", regaddr, regval);
  return regval;
}
#endif

/**************************************************************************************
 * Name:  ra8875_waitreg
 *
 * Description:
 *   Wait while an LCD register match set mask
 *
 **************************************************************************************/

#ifndef CONFIG_LCD_NOGETRUN
static void ra8875_waitreg(FAR struct ra8875_lcd_s *lcd, uint8_t regaddr, uint8_t mask)
{
  int i = 20000/100;

  while (i-- && ra8875_readreg(lcd, regaddr) & mask)
    {
      up_udelay(100);
    }
}
#endif

/**************************************************************************************
 * Name:  ra8875_set_writecursor
 *
 * Description:
 *   Set the position to use for the write cursor
 *
 **************************************************************************************/

static void ra8875_set_writecursor(FAR struct ra8875_dev_s *dev, uint16_t column,
                                   uint16_t row)
{
  FAR struct ra8875_lcd_s *lcd = dev->lcd;

#if defined(CONFIG_LCD_PORTRAIT) || defined(CONFIG_LCD_RPORTRAIT)
  if (dev->shadow_w_curh != row)
    {
      ra8875_putreg16(lcd, RA8875_CURH0, row);
      dev->shadow_w_curh = row;
    }

  if (dev->shadow_w_curv != column)
    {
      ra8875_putreg16(lcd, RA8875_CURV0, column);
      dev->shadow_w_curv = column;
    }
#elif defined(CONFIG_LCD_LANDSCAPE) || defined(CONFIG_LCD_RLANDSCAPE)
  if (dev->shadow_w_curh != column)
    {
      ra8875_putreg16(lcd, RA8875_CURH0, column);
      dev->shadow_w_curh = column;
    }

  if (dev->shadow_w_curv != row)
    {
      ra8875_putreg16(lcd, RA8875_CURV0, row);
      dev->shadow_w_curv = row;
    }
#endif
}

/**************************************************************************************
 * Name:  ra8875_set_readcursor
 *
 * Description:
 *   Set the position to use for the read cursor
 *
 **************************************************************************************/

static void ra8875_set_readcursor(FAR struct ra8875_lcd_s *lcd, uint16_t column,
                                  uint16_t row)
{
#if defined(CONFIG_LCD_PORTRAIT) || defined(CONFIG_LCD_RPORTRAIT)
  ra8875_putreg16(lcd, RA8875_RCURH0, row);
  ra8875_putreg16(lcd, RA8875_RCURV0, column);
#elif defined(CONFIG_LCD_LANDSCAPE) || defined(CONFIG_LCD_RLANDSCAPE)
  ra8875_putreg16(lcd, RA8875_RCURH0, column);
  ra8875_putreg16(lcd, RA8875_RCURV0, row);
#endif
}

static void ra8875_set_mwcr0(FAR struct ra8875_dev_s *dev, uint8_t value)
{
  FAR struct ra8875_lcd_s *lcd = dev->lcd;

  if (dev->shadow_mwcr0 != value)
    {
      ra8875_putreg(lcd, RA8875_MWCR0, value);
      dev->shadow_mwcr0 = value;
    }
}

/**************************************************************************************
 * Name:  ra8875_setwindow
 *
 * Description:
 *   Set hardware clipping window
 *
 **************************************************************************************/

static void ra8875_setwindow(FAR struct ra8875_lcd_s *lcd, uint16_t x, uint16_t y,
                             uint16_t width, uint16_t height)
{
#if defined(CONFIG_LCD_PORTRAIT) || defined(CONFIG_LCD_RPORTRAIT)
  ra8875_putreg16(lcd, RA8875_HSAW0, y);
  ra8875_putreg16(lcd, RA8875_VSAW0, x);
  ra8875_putreg16(lcd, RA8875_HEAW0, (y+height-1));
  ra8875_putreg16(lcd, RA8875_VEAW0, (x+width-1));
#elif defined(CONFIG_LCD_LANDSCAPE) || defined(CONFIG_LCD_RLANDSCAPE)
  ra8875_putreg16(lcd, RA8875_HSAW0, x);
  ra8875_putreg16(lcd, RA8875_VSAW0, y);
  ra8875_putreg16(lcd, RA8875_HEAW0, (x+width-1));
  ra8875_putreg16(lcd, RA8875_VEAW0, (y+height-1));
#endif
}

/**************************************************************************************
 * Name:  ra8875_setbackground
 *
 * Description:
 *   Set the background color to use for the BTE engine
 *
 **************************************************************************************/

static inline void ra8875_setbackground(FAR struct ra8875_lcd_s *lcd, uint16_t color)
{
  ra8875_putreg(lcd, RA8875_BGCR0, RA8875_UNPACK_RED(color));
  ra8875_putreg(lcd, RA8875_BGCR1, RA8875_UNPACK_GREEN(color));
  ra8875_putreg(lcd, RA8875_BGCR2, RA8875_UNPACK_BLUE(color));
}

/**************************************************************************************
 * Name:  ra8875_setforeground
 *
 * Description:
 *   Set the foreground color to use for the BTE engine
 *
 **************************************************************************************/

static inline void ra8875_setforeground(FAR struct ra8875_lcd_s *lcd, uint16_t color)
{
  ra8875_putreg(lcd, RA8875_FGCR0, RA8875_UNPACK_RED(color));
  ra8875_putreg(lcd, RA8875_FGCR1, RA8875_UNPACK_GREEN(color));
  ra8875_putreg(lcd, RA8875_FGCR2, RA8875_UNPACK_BLUE(color));
}

/**************************************************************************************
 * Name:  ra8875_clearmem
 *
 * Description:
 *
 *
 **************************************************************************************/

static void ra8875_clearmem(FAR struct ra8875_lcd_s *lcd)
{
  lcdinfo("clearmem start\n");
  ra8875_putreg(lcd, RA8875_MCLR, RA8875_MCLR_CLEAR | RA8875_MCLR_FULL);

  /* Wait for operation to finish */

  ra8875_waitreg(lcd, RA8875_MCLR, RA8875_MCLR_CLEAR);
  lcdinfo("clearmem done\n");
}

/**************************************************************************************
 * Name:  ra8875_showrun
 *
 * Description:
 *   When LCD debug is enabled, try to reduce then amount of output data generated by
 *   ra8875_putrun and ra8875_getrun
 *
 **************************************************************************************/

#ifdef CONFIG_DEBUG_LCD
static void ra8875_showrun(FAR struct ra8875_dev_s *priv, fb_coord_t row,
                            fb_coord_t col, size_t npixels, bool put)
{
  fb_coord_t nextrow = priv->lastrow + 1;

  /* Has anything changed (other than the row is the next row in the sequence)? */

  if (put == priv->put && row == nextrow && col == priv->col &&
      npixels == priv->npixels)
    {
      /* No, just update the last row */

      priv->lastrow = nextrow;
    }
  else
    {
      /* Yes... then this is the end of the preceding sequence.  Output the last run
       * (if there were more than one run in the sequence).
       */

      if (priv->firstrow != priv->lastrow)
        {
          lcdinfo("...\n");
          lcdinfo("%s row: %d col: %d npixels: %d\n",
                  priv->put ? "PUT" : "GET",
                  priv->lastrow, priv->col, priv->npixels);
        }

      /* And we are starting a new sequence.  Output the first run of the
       * new sequence
       */

      lcdinfo("%s row: %d col: %d npixels: %d\n",
              put ? "PUT" : "GET", row, col, npixels);

      /* And save information about the run so that we can detect continuations
       * of the sequence.
       */

      priv->put      = put;
      priv->firstrow = row;
      priv->lastrow  = row;
      priv->col      = col;
      priv->npixels  = npixels;
    }
}
#endif

/**************************************************************************************
 * Name:  ra8875_putrun
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

static int ra8875_putrun(fb_coord_t row, fb_coord_t col, FAR const uint8_t *buffer,
                         size_t npixels)
{
  FAR struct ra8875_dev_s *priv = &g_lcddev;
  FAR struct ra8875_lcd_s *lcd = priv->lcd;
  int16_t curhinc = 0;
  int16_t curvinc = 0;

#if RA8875_BPP == 16
  DEBUGASSERT(buffer && ((uintptr_t)buffer & 1) == 0);

  FAR const uint16_t *src = (FAR const uint16_t *)buffer;
#else
  FAR const uint8_t *src = (FAR const uint8_t *)buffer;
#endif
  int i;

  /* Buffer must be provided and aligned to a 16-bit address boundary */

  ra8875_showrun(priv, row, col, npixels, true);

#ifdef CONFIG_LCD_LANDSCAPE

  /* Set the cursor position */

  ra8875_set_mwcr0(priv, RA8875_MWCR0_MODE_GRAPHICS | RA8875_MWCR0_MEMDIR_LEFTRIGHT |
                         RA8875_MWCR0_WINC_ENABLE);
  ra8875_set_writecursor(priv, col, row);

  curhinc = npixels;
  curvinc = 0;

#elif defined(CONFIG_LCD_RLANDSCAPE)

  /* Retransform coordinates, write right to left */

  col = (RA8875_XRES-1) - col;
  row = (RA8875_YRES-1) - row;

  /* Set the cursor position */

  ra8875_set_mwcr0(priv, RA8875_MWCR0_MODE_GRAPHICS | RA8875_MWCR0_MEMDIR_RIGHTLEFT | RA8875_MWCR0_WINC_ENABLE);
  ra8875_set_writecursor(priv, col, row);

  curhinc = -npixels;
  curvinc = 0;

#elif defined(CONFIG_LCD_PORTRAIT)

  row = (RA8875_YRES-1) - row;

  /* Set the cursor position */

  ra8875_set_mwcr0(priv, RA8875_MWCR0_MODE_GRAPHICS | RA8875_MWCR0_MEMDIR_TOPDOWN |
                         RA8875_MWCR0_WINC_ENABLE);
  ra8875_set_writecursor(priv, col, row);

  curhinc = 0;
  curvinc = npixels;

#else /* CONFIG_LCD_RPORTRAIT */

  col = (RA8875_XRES-1) - col;

  /* Set the cursor position */

  ra8875_set_mwcr0(priv, RA8875_MWCR0_MODE_GRAPHICS | RA8875_MWCR0_MEMDIR_DOWNTOP |
                         RA8875_MWCR0_WINC_ENABLE);
  ra8875_set_writecursor(priv, col, row);

  curhinc = 0;
  curvinc = -npixels;

#endif

  /* Write data, handle order automatically */

  lcd->pwrite_prepare(lcd, RA8875_MRWC);

  for (i = 0; i < npixels; i++)
    {
      /* Write the next pixel to this position */

#if RA8875_BPP == 8
      lcd->pwrite_data8(lcd, *src++);
#else
      lcd->pwrite_data16(lcd, *src++);
#endif
    }

  lcd->pwrite_finish(lcd);

  /* Update shadowed cursor */

  priv->shadow_w_curh += curhinc;
  priv->shadow_w_curv += curvinc;

  return OK;
}

/**************************************************************************************
 * Name:  ra8875_getrun
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

static int ra8875_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t *buffer,
                         size_t npixels)
{
#ifndef CONFIG_LCD_NOGETRUN
  FAR struct ra8875_dev_s *priv = &g_lcddev;
  FAR struct ra8875_lcd_s *lcd = priv->lcd;
  FAR uint16_t *dest = (FAR uint16_t *)buffer;
  int i;

  /* Buffer must be provided and aligned to a 16-bit address boundary */

  ra8875_showrun(priv, row, col, npixels, false);
  DEBUGASSERT(buffer && ((uintptr_t)buffer & 1) == 0);


#ifdef CONFIG_LCD_LANDSCAPE
  /* Set the cursor position */

  ra8875_putreg(lcd, RA8875_MWCR0, RA8875_MWCR0_MODE_GRAPHICS |
                                   RA8875_MWCR0_MEMDIR_LEFTRIGHT |
                                   RA8875_MWCR0_RINC_ENABLE);
  ra8875_putreg(lcd, RA8875_MRDC, RA8875_MRCD_MEMDIR_LEFTRIGHT);

  ra8875_set_readcursor(lcd, col, row);

#elif defined(CONFIG_LCD_RLANDSCAPE)
  /* Retransform coordinates, write right to left */

  col = (RA8875_XRES-1) - col;
  row = (RA8875_YRES-1) - row;

  /* Set the cursor position */

  ra8875_putreg(lcd, RA8875_MWCR0, RA8875_MWCR0_MODE_GRAPHICS |
                                   RA8875_MWCR0_MEMDIR_RIGHTLEFT |
                                   RA8875_MWCR0_RINC_ENABLE);
  ra8875_putreg(lcd, RA8875_MRDC, RA8875_MRCD_MEMDIR_RIGHTLEFT);

  ra8875_set_readcursor(lcd, col, row);

#elif defined(CONFIG_LCD_PORTRAIT)
  /* Retransform coordinates, write right to left */

  row = (RA8875_YRES-1) - row;

  /* Set the cursor position */

  ra8875_putreg(lcd, RA8875_MWCR0, RA8875_MWCR0_MODE_GRAPHICS |
                                   RA8875_MWCR0_MEMDIR_TPDOWN |
                                   RA8875_MWCR0_RINC_ENABLE);
  ra8875_putreg(lcd, RA8875_MRDC, RA8875_MRCD_MEMDIR_TOPDOWN);

  ra8875_set_readcursor(lcd, col, row);

#else /* CONFIG_LCD_RPORTRAIT */
  /* Retransform coordinates, write right to left */

  col = (RA8875_XRES-1) - col;

  /* Set the cursor position */

  ra8875_putreg(lcd, RA8875_MWCR0, RA8875_MWCR0_MODE_GRAPHICS |
                                   RA8875_MWCR0_MEMDIR_DOWNTOP |
                                   RA8875_MWCR0_RINC_ENABLE);
  ra8875_putreg(lcd, RA8875_MRDC, RA8875_MRCD_MEMDIR_DOWNTOP);

  ra8875_set_readcursor(lcd, col, row);

#endif

  /* Read data, handle order automatically */

  lcd->pread_prepare(lcd, RA8875_MRWC);

  for (i = 0; i < npixels; i++)
    {
      /* Read the next pixel from this position */

      *dest++ = lcd->pread_data16(lcd);
    }

  lcd->pread_finish(lcd);

  return OK;
#else
  return -ENOSYS;
#endif
}

/**************************************************************************************
 * Name:  ra8875_getvideoinfo
 *
 * Description:
 *   Get information about the LCD video controller configuration.
 *
 **************************************************************************************/

static int ra8875_getvideoinfo(FAR struct lcd_dev_s *dev,
                               FAR struct fb_videoinfo_s *vinfo)
{
  DEBUGASSERT(dev && vinfo);
  lcdinfo("fmt: %d xres: %d yres: %d nplanes: 1\n",
          RA8875_COLORFMT, RA8875_XRES, RA8875_YRES);

  vinfo->fmt     = RA8875_COLORFMT;    /* Color format: RGB16-565: RRRR RGGG GGGB BBBB */
  vinfo->xres    = RA8875_XRES;        /* Horizontal resolution in pixel columns */
  vinfo->yres    = RA8875_YRES;        /* Vertical resolution in pixel rows */
  vinfo->nplanes = 1;                  /* Number of color planes supported */
  return OK;
}

/**************************************************************************************
 * Name:  ra8875_getplaneinfo
 *
 * Description:
 *   Get information about the configuration of each LCD color plane.
 *
 **************************************************************************************/

static int ra8875_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int planeno,
                                FAR struct lcd_planeinfo_s *pinfo)
{
  FAR struct ra8875_dev_s *priv = (FAR struct ra8875_dev_s *)dev;

  DEBUGASSERT(dev && pinfo && planeno == 0);
  lcdinfo("planeno: %d bpp: %d\n", planeno, RA8875_BPP);

  pinfo->putrun = ra8875_putrun;                  /* Put a run into LCD memory */
  pinfo->getrun = ra8875_getrun;                  /* Get a run from LCD memory */
  pinfo->buffer = (FAR uint8_t *)priv->runbuffer; /* Run scratch buffer */
  pinfo->bpp    = RA8875_BPP;                     /* Bits-per-pixel */
  return OK;
}

/**************************************************************************************
 * Name:  ra8875_getpower
 *
 * Description:
 *   Get the LCD panel power status (0: full off - CONFIG_LCD_MAXPOWER: full on). On
 *   backlit LCDs, this setting may correspond to the backlight setting.
 *
 **************************************************************************************/

static int ra8875_getpower(FAR struct lcd_dev_s *dev)
{
  lcdinfo("power: %d\n", 0);
  return g_lcddev.power;
}

/**************************************************************************************
 * Name:  ra8875_poweroff
 *
 * Description:
 *   Enable/disable LCD panel power (0: full off - CONFIG_LCD_MAXPOWER: full on). On
 *   backlit LCDs, this setting may correspond to the backlight setting.
 *
 **************************************************************************************/

static int ra8875_poweroff(FAR struct ra8875_lcd_s *lcd)
{
  /* Set the backlight off */

  ra8875_putreg(lcd, RA8875_P1CR, RA8875_P1CR_PWM_DISABLE);
  ra8875_putreg(lcd, RA8875_P1DCR, 0);

  /* Turn the display off */

  ra8875_putreg(lcd, RA8875_PWRR, RA8875_PWRR_DISPLAY_OFF);

  /* Remember the power off state */

  g_lcddev.power = 0;
  return OK;
}

/**************************************************************************************
 * Name:  ra8875_setpower
 *
 * Description:
 *   Enable/disable LCD panel power (0: full off - CONFIG_LCD_MAXPOWER: full on). On
 *   backlit LCDs, this setting may correspond to the backlight setting.
 *
 **************************************************************************************/

static int ra8875_setpower(FAR struct lcd_dev_s *dev, int power)
{
  FAR struct ra8875_dev_s *priv = (FAR struct ra8875_dev_s *)dev;
  FAR struct ra8875_lcd_s *lcd  = priv->lcd;

  lcdinfo("power: %d\n", power);
  DEBUGASSERT((unsigned)power <= CONFIG_LCD_MAXPOWER);

  /* Set new power level */

  if (power > 0)
    {
      if (g_lcddev.power == 0)
        {
          /* Set the backlight level */

          ra8875_putreg(lcd, RA8875_P1CR, RA8875_P1CR_PWM_ENABLE);
          ra8875_putreg(lcd, RA8875_P1CR, RA8875_P1CR_PWM_ENABLE | RA8875_P1CR_CSDIV(1));
        }

      ra8875_putreg(lcd, RA8875_P1DCR, power);

      /* Turn on display */

      ra8875_putreg(lcd, RA8875_PWRR, RA8875_PWRR_DISPLAY_ON);

      g_lcddev.power = power;
    }
  else
    {
      /* Turn the display off */

      ra8875_poweroff(lcd);
    }

  return OK;
}

/**************************************************************************************
 * Name:  ra8875_getcontrast
 *
 * Description:
 *   Get the current contrast setting (0-CONFIG_LCD_MAXCONTRAST).
 *
 **************************************************************************************/

static int ra8875_getcontrast(FAR struct lcd_dev_s *dev)
{
  lcdinfo("Not implemented\n");
  return -ENOSYS;
}

/**************************************************************************************
 * Name:  ra8875_setcontrast
 *
 * Description:
 *   Set LCD panel contrast (0-CONFIG_LCD_MAXCONTRAST).
 *
 **************************************************************************************/

static int ra8875_setcontrast(FAR struct lcd_dev_s *dev, unsigned int contrast)
{
  lcdinfo("contrast: %d\n", contrast);
  return -ENOSYS;
}

/**************************************************************************************
 * Name:  ra8875_hwinitialize
 *
 * Description:
 *   Initialize the LCD hardware.
 *
 **************************************************************************************/

static inline int ra8875_hwinitialize(FAR struct ra8875_dev_s *priv)
{
  uint8_t rv;
  FAR struct ra8875_lcd_s *lcd  = priv->lcd;

  /* REVISIT: Maybe some of these values needs to be configurable?? */

  lcdinfo("hwinitialize\n");

  /* Reset */

  ra8875_putreg(lcd, RA8875_PWRR, RA8875_PWRR_SWRESET);
  up_mdelay(100);
  ra8875_putreg(lcd, RA8875_PWRR, 0);

  /* Setup the PLL config */

  ra8875_putreg(lcd, RA8875_PLLC1, RA8875_PLLC1_PLLDIVN(11));
  up_mdelay(10);
  ra8875_putreg(lcd, RA8875_PLLC2, RA8875_PLLC2_PLLDIVK(2));
  up_mdelay(10);

  /* Interface and color depth */

#if RA8875_BPP == 16
  rv = RA8875_SYSR_COLOR_65K;
#else
  rv = RA8875_SYSR_COLOR_256;
#endif
#if defined(CONFIG_LCD_RA8875_8BIT)
  rv |= RA8875_SYSR_MCUIF_8BIT;
#elif defined(CONFIG_LCD_RA8875_16BIT)
  rv |= RA8875_SYSR_MCUIF_16BIT;
#endif
  ra8875_putreg(lcd, RA8875_SYSR, rv);

  /* Pixel clock, invert + 4*SYS */

  ra8875_putreg(lcd, RA8875_PCSR, RA8875_PCSR_PCLK_INV | RA8875_PCSR_PERIOD_4SYS);
  up_mdelay(1);

  /* Horizontal Settings */

  ra8875_putreg(lcd, RA8875_HDWR, RA8875_HDWR_WIDTH(RA8875_HW_XRES));
  ra8875_putreg(lcd, RA8875_HNDFTR, 0x02);
  ra8875_putreg(lcd, RA8875_HNDR, 0x03);
  ra8875_putreg(lcd, RA8875_HSTR, 0x01);
  ra8875_putreg(lcd, RA8875_HPWR, 0x03);

  /* Vertical Settings */

  ra8875_putreg(lcd, RA8875_VDHR0, RA8875_VDHR0_HEIGHT(RA8875_HW_YRES));
  ra8875_putreg(lcd, RA8875_VDHR1, RA8875_VDHR1_HEIGHT(RA8875_HW_YRES));
  ra8875_putreg(lcd, RA8875_VNDR0, 0x0f);
  ra8875_putreg(lcd, RA8875_VNDR1, 0x00);
  ra8875_putreg(lcd, RA8875_VSTR0, 0x0e);
  ra8875_putreg(lcd, RA8875_VSTR1, 0x06);
  ra8875_putreg(lcd, RA8875_VPWR, 0x01);

#if !defined(RA8875_2LAYER_POSSIBLE)
  /* Too high, only one layer possible */

  ra8875_putreg(lcd, RA8875_DPCR, RA8875_DPCR_LAYERS_ONE);
#else
  /* Two layers */

  ra8875_putreg(lcd, RA8875_DPCR, RA8875_DPCR_LAYERS_TWO);
#endif

  /* Setup window to be full screen */

  ra8875_setwindow(lcd, 0, 0, RA8875_XRES, RA8875_YRES);

  /* Set background and foreground colors to black and white (respectively) */

  ra8875_setbackground(lcd, 0x0000);
  ra8875_setforeground(lcd, 0xffff);

  /* Clear the memory */

  ra8875_clearmem(lcd);

  ra8875_setpower(&priv->dev, 0x0);

  /* Initialize the shadow registers */

  priv->shadow_mwcr0 = 0;
  priv->shadow_w_curh = 0;
  priv->shadow_w_curv = 0;

  lcdinfo("hwinitialize done\n");
  return OK;
}

/**************************************************************************************
 * Public Functions
 **************************************************************************************/

/**************************************************************************************
 * Name:  ra8875_lcdinitialize
 *
 * Description:
 *   Initialize the LCD video hardware.  The initial state of the LCD is fully
 *   initialized, display memory cleared, and the LCD ready to use, but with the power
 *   setting at 0 (full off).
 *
 **************************************************************************************/

FAR struct lcd_dev_s *ra8875_lcdinitialize(FAR struct ra8875_lcd_s *lcd)
{
  int ret;

  lcdinfo("Initializing\n");

  /* If we could support multiple RA8875 devices, this is where we would allocate
   * a new driver data structure... but we can't.  Why not?  Because of a bad should
   * the form of the getrun() and putrun methods.
   */

  FAR struct ra8875_dev_s *priv = &g_lcddev;

  /* Initialize the driver data structure */

  priv->dev.getvideoinfo = ra8875_getvideoinfo;
  priv->dev.getplaneinfo = ra8875_getplaneinfo;
  priv->dev.getpower     = ra8875_getpower;
  priv->dev.setpower     = ra8875_setpower;
  priv->dev.getcontrast  = ra8875_getcontrast;
  priv->dev.setcontrast  = ra8875_setcontrast;
  priv->lcd              = lcd;

  /* Configure and enable LCD */

  ret = ra8875_hwinitialize(priv);
  if (ret == OK)
    {
      /* Clear the display (setting it to the color 0=black) */

      ra8875_clear(&priv->dev, 0);

      /* Turn the display off */

      ra8875_poweroff(lcd);

      lcdinfo("Initialized\n");

      return &g_lcddev.dev;
    }

  return NULL;
}

/**************************************************************************************
 * Name:  ra8875_clear
 *
 * Description:
 *   This is a non-standard LCD interface just for the RA8875.  Because
 *   of the various rotations, clearing the display in the normal way by writing a
 *   sequences of runs that covers the entire display can be very slow.  Here the
 *   display is cleared by simply setting all video memory to the specified color.
 *
 *   NOTE: This function is not available to applications in the protected or kernel
 *   build modes.
 *
 **************************************************************************************/

void ra8875_clear(FAR struct lcd_dev_s *dev, uint16_t color)
{
#if 0
  FAR struct ra8875_dev_s *priv = (FAR struct ra8875_dev_s *)dev;
  FAR struct ra8875_lcd_s *lcd  = priv->lcd;

  /* Set the color to use for filling */

  ra8875_setforeground(lcd, color);

  /* Draw a rectangle over entire screen */

  ra8875_putreg16(lcd, RA8875_DLHSR0, 0);
  ra8875_putreg16(lcd, RA8875_DLVSR0, 0);
  ra8875_putreg16(lcd, RA8875_DLHER0, RA8875_XRES);
  ra8875_putreg16(lcd, RA8875_DLVER0, RA8875_YRES);

  ra8875_putreg(lcd, RA8875_DCR, RA8875_DCR_FILL | RA8875_DCR_SQUARE);
  ra8875_putreg(lcd, RA8875_DCR, RA8875_DCR_FILL | RA8875_DCR_SQUARE |
                                 RA8875_DCR_LINE_START);

  ra8875_waitreg(lcd, RA8875_DCR, RA8875_DCR_LINE_START);
#endif

  /* Draw a rectangle over entire screen */

  ra8875_drawrectangle(dev, 0, 0, RA8875_XRES, RA8875_YRES, color, true);
}

/**************************************************************************************
 * Name:  ra8875_drawrectangle
 *
 *   This is a non-standard function to draw a rectangle on the LCD.  This function is
 *   also used internally as part of the ra8875_clear implementation
 *
 *   NOTE: This non-standard function is not available to applications in the
 *   protected or kernel build modes.
 *
 **************************************************************************************/

void ra8875_drawrectangle(FAR struct lcd_dev_s *dev, uint16_t x, uint16_t y,
                          uint16_t width, uint16_t height, uint16_t color, bool fill)
{
  FAR struct ra8875_dev_s *priv = (FAR struct ra8875_dev_s *)dev;
  FAR struct ra8875_lcd_s *lcd  = priv->lcd;

  uint8_t draw_cmd = RA8875_DCR_SQUARE;
  uint16_t sx, sy, ex, ey;

  /* Set the color to use for filling */

  ra8875_setforeground(lcd, color);

  /* Handle degenerate cases */

  if (width == 1)
    {
      ra8875_drawline(dev, x, y, x, y+height, color);
    }
  else if (height == 1)
    {
      ra8875_drawline(dev, x, y, x+width, y, color);
    }

  if (fill)
    {
      draw_cmd |= RA8875_DCR_FILL;
    }

  /* Setup coordinates */

#ifdef CONFIG_LCD_LANDSCAPE

  sx = x;
  sy = y;
  ex = x + width - 1;
  ey = y + height - 1;

#elif defined(CONFIG_LCD_RLANDSCAPE)

  sx = (RA8875_XRES - 1) - x - width + 1;
  sy = (RA8875_YRES - 1) - y - height + 1;
  ex = (RA8875_XRES - 1) - x;
  ey = (RA8875_YRES - 1) - y;

#elif defined(CONFIG_LCD_PORTRAIT)

  sx = (RA8875_YRES - 1) - y - height + 1;
  sy = x;
  ex = (RA8875_YRES - 1) - y;
  ey = x + width - 1;

#else /* CONFIG_LCD_RPORTRAIT */

  sx = y;
  sy = (RA8875_XRES - 1) - x - width + 1;
  ex = y + height - 1;
  ey = (RA8875_XRES - 1) - x;

#endif

  ra8875_putreg16(lcd, RA8875_DLHSR0, sx);
  ra8875_putreg16(lcd, RA8875_DLVSR0, sy);
  ra8875_putreg16(lcd, RA8875_DLHER0, ex);
  ra8875_putreg16(lcd, RA8875_DLVER0, ey);

  /* Run drawing */

  ra8875_putreg(lcd, RA8875_DCR, draw_cmd);
  ra8875_putreg(lcd, RA8875_DCR, draw_cmd | RA8875_DCR_LINE_START);

  ra8875_waitreg(lcd, RA8875_DCR, RA8875_DCR_LINE_START);
}

/**************************************************************************************
 * Name:  ra8875_drawline
 *
 * Description:
 *   This is a non-standard function to draw a line on the LCD.  This function is
 *   also used internally as part of the ra8875_rectandle implementation.
 *
 *   NOTE: This non-standard function is not available to applications in the
 *   protected or kernel build modes.
 *
 **************************************************************************************/

void ra8875_drawline(FAR struct lcd_dev_s *dev, uint16_t x1, uint16_t y1, uint16_t x2,
                     uint16_t y2, uint16_t color)
{
  FAR struct ra8875_dev_s *priv = (FAR struct ra8875_dev_s *)dev;
  FAR struct ra8875_lcd_s *lcd  = priv->lcd;

  uint8_t draw_cmd = RA8875_DCR_LINE;
  uint16_t sx, sy, ex, ey;

  /* Set the color to use for filling */

  ra8875_setforeground(lcd, color);

  /* Handle degenerate cases */
  /* Setup coordinates */

#if defined(CONFIG_LCD_LANDSCAPE) || defined(CONFIG_LCD_RLANDSCAPE)

  sx = x1;
  sy = y1;
  ex = x2;
  ey = y2;

#else /* CONFIG_LCD_PORTRAIT or CONFIG_LCD_RPORTRAIT */

  sx = y1;
  sy = x1;
  ex = y2;
  ey = x2;

#endif

  ra8875_putreg16(lcd, RA8875_DLHSR0, sx);
  ra8875_putreg16(lcd, RA8875_DLVSR0, sy);
  ra8875_putreg16(lcd, RA8875_DLHER0, ex);
  ra8875_putreg16(lcd, RA8875_DLVER0, ey);

  /* Run drawing */

  ra8875_putreg(lcd, RA8875_DCR, draw_cmd);
  ra8875_putreg(lcd, RA8875_DCR, draw_cmd | RA8875_DCR_LINE_START);

  ra8875_waitreg(lcd, RA8875_DCR, RA8875_DCR_LINE_START);
}

/**************************************************************************************
 * Name:  ra8875_drawtriangle
 *
 * Description:
 *   This is a non-standard function to draw a triangle on the LCD.  This function is
 *   also used internally as part of the ra8875_rectandle implementation.
 *
 *   NOTE: This non-standard function is not available to applications in the
 *   protected or kernel build modes.
 *
 **************************************************************************************/

#ifdef CONFIG_LCD_RA8875_EXTENDED
void ra8875_drawtriangle(FAR struct lcd_dev_s *dev, uint16_t x0, uint16_t y0, uint16_t x1,
                         uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color, bool fill)
{
  FAR struct ra8875_dev_s *priv = (FAR struct ra8875_dev_s *)dev;
  FAR struct ra8875_lcd_s *lcd  = priv->lcd;

  uint8_t draw_cmd = RA8875_DCR_TRIANGLE;
  uint16_t _x0, _x1, _x2;
  uint16_t _y0, _y1, _y2;

  /* Set the color to use for filling */

  ra8875_setforeground(lcd, color);

  if (fill)
    {
      draw_cmd |= RA8875_DCR_FILL;
    }

  /* Setup coordinates */

#if defined(CONFIG_LCD_LANDSCAPE) || defined(CONFIG_LCD_RLANDSCAPE)

  _x0 = x0;
  _x1 = x1;
  _x2 = x2;

  _y0 = y0;
  _y1 = y1;
  _y2 = y2;

#else /* CONFIG_LCD_PORTRAIT or CONFIG_LCD_RPORTRAIT */

  _x0 = y0;
  _x1 = y1;
  _x2 = y2;

  _y0 = x0;
  _y1 = x1;
  _y2 = x2;

#endif

  ra8875_putreg16(lcd, RA8875_DLHSR0, _x0);
  ra8875_putreg16(lcd, RA8875_DLVSR0, _y0);
  ra8875_putreg16(lcd, RA8875_DLHER0, _x1);
  ra8875_putreg16(lcd, RA8875_DLVER0, _y1);
  ra8875_putreg16(lcd, RA8875_DTPH0, _x2);
  ra8875_putreg16(lcd, RA8875_DTPV0, _y2);

  /* Run drawing */

  ra8875_putreg(lcd, RA8875_DCR, draw_cmd);
  ra8875_putreg(lcd, RA8875_DCR, draw_cmd | RA8875_DCR_LINE_START);

  ra8875_waitreg(lcd, RA8875_DCR, RA8875_DCR_LINE_START);
}
#endif

/**************************************************************************************
 * Name:  ra8875_drawcircle
 *
 * Description:
 *   This is a non-standard function to draw a circle on the LCD.  This function is
 *   also used internally as part of the ra8875_rectandle implementation.
 *
 *   NOTE: This non-standard function is not available to applications in the
 *   protected or kernel build modes.
 *
 **************************************************************************************/

#ifdef CONFIG_LCD_RA8875_EXTENDED
void ra8875_drawcircle(FAR struct lcd_dev_s *dev, uint16_t x, uint16_t y, uint8_t radius,
                       uint16_t color, bool fill)
{
  FAR struct ra8875_dev_s *priv = (FAR struct ra8875_dev_s *)dev;
  FAR struct ra8875_lcd_s *lcd  = priv->lcd;

  uint8_t draw_cmd = 0;
  uint16_t _x, _y;

  /* Set the color to use for filling */

  ra8875_setforeground(lcd, color);

  if (fill)
    {
      draw_cmd |= RA8875_DCR_FILL;
    }

  /* Setup coordinates */

#if defined(CONFIG_LCD_LANDSCAPE) || defined(CONFIG_LCD_RLANDSCAPE)

  _x = x;
  _y = y;

#else /* CONFIG_LCD_PORTRAIT or CONFIG_LCD_RPORTRAIT */

  _x = y;
  _y = x;

#endif

  ra8875_putreg16(lcd, RA8875_DCHR0, _x);
  ra8875_putreg16(lcd, RA8875_DCVR0, _y);
  ra8875_putreg(lcd, RA8875_DCRR, radius);

    /* Run drawing */

  ra8875_putreg(lcd, RA8875_DCR, draw_cmd);
  ra8875_putreg(lcd, RA8875_DCR, draw_cmd | RA8875_DCR_CIRCLE_START);

  ra8875_waitreg(lcd, RA8875_DCR, RA8875_DCR_CIRCLE_START);
}
#endif

#endif /* CONFIG_LCD_RA8875 */
