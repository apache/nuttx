/****************************************************************************
 * drivers/lcd/mio283qt9a.c
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
#include <nuttx/lcd/mio283qt9a.h>

#ifdef CONFIG_LCD_MIO283QT9A

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

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
#  if defined(CONFIG_LCD_LANDSCAPE) || defined(CONFIG_LCD_RLANDSCAPE) || defined(CONFIG_LCD_RPORTRAIT)
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

/* Display/Color Properties *************************************************/

/* Display Resolution */

#if defined(CONFIG_LCD_LANDSCAPE) || defined(CONFIG_LCD_RLANDSCAPE)
#  define MIO283QT9A_XRES       320
#  define MIO283QT9A_YRES       240
#else
#  define MIO283QT9A_XRES       240
#  define MIO283QT9A_YRES       320
#endif

/* Color depth and format */

#define MIO283QT9A_BPP           16
#define MIO283QT9A_COLORFMT      FB_FMT_RGB16_565

/* Hardware LCD/LCD controller definitions **********************************/

/* In this driver, I chose to use all literal constants for register address
 * and values. Some recent experiences have shown me that during LCD bringup,
 * it is more important to know the binary values rather than nice, people
 * friendly names.  Sad, but true.
 */

#define ILI9341_ID_1 0x93
#define ILI9341_ID_2 0x41

/****************************************************************************
 * Private Type Definition
 ****************************************************************************/

/* This structure describes the state of this driver */

struct mio283qt9a_dev_s
{
  /* Publicly visible device structure */

  struct lcd_dev_s dev;

  /* Private LCD-specific information follows */

  FAR struct mio283qt9a_lcd_s *lcd;  /* The contained platform-specific, LCD interface */
  uint8_t power;                     /* Current power setting */

  /* This is working memory allocated by the LCD driver for each LCD device
   * and for each color plane.
   * This memory will hold one raster line of data.
   * The size of the allocated run buffer must therefore be at least
   * (bpp * xres / 8).  Actual alignment of the buffer must conform to the
   * bitwidth of the underlying pixel type.
   *
   * If there are multiple planes, they may share the same working buffer
   * because different planes will not be operate on concurrently.
   * However, if there are multiple LCD devices, they must each have unique
   * run buffers.
   */

  uint16_t runbuffer[MIO283QT9A_XRES];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Low Level LCD access */

static void mio283qt9a_putreg(FAR struct mio283qt9a_lcd_s *lcd,
                              uint8_t regaddr,
                              uint16_t regval);
#ifndef CONFIG_LCD_NOGETRUN
static uint16_t mio283qt9a_readreg(FAR struct mio283qt9a_lcd_s *lcd,
                                   uint8_t regaddr);
#endif
static inline void mio283qt9a_gramwrite(FAR struct mio283qt9a_lcd_s *lcd,
             uint16_t rgbcolor);
#ifndef CONFIG_LCD_NOGETRUN
static inline void mio283qt9a_readsetup(FAR struct mio283qt9a_lcd_s *lcd,
             FAR uint16_t *accum);
static inline uint16_t mio283qt9a_gramread(FAR struct mio283qt9a_lcd_s *lcd,
             FAR uint16_t *accum);
#endif
static void mio283qt9a_setarea(FAR struct mio283qt9a_lcd_s *lcd,
             uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);

/* LCD Data Transfer Methods */

static int mio283qt9a_putrun(FAR struct lcd_dev_s *dev,
                             fb_coord_t row, fb_coord_t col,
                             FAR const uint8_t *buffer,
                             size_t npixels);
static int mio283qt9a_getrun(FAR struct lcd_dev_s *dev,
                             fb_coord_t row, fb_coord_t col,
                             FAR uint8_t *buffer,
                             size_t npixels);

/* LCD Configuration */

static int mio283qt9a_getvideoinfo(FAR struct lcd_dev_s *dev,
             FAR struct fb_videoinfo_s *vinfo);
static int mio283qt9a_getplaneinfo(FAR struct lcd_dev_s *dev,
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

static int mio283qt9a_getpower(FAR struct lcd_dev_s *dev);
static int mio283qt9a_setpower(FAR struct lcd_dev_s *dev, int power);
static int mio283qt9a_getcontrast(FAR struct lcd_dev_s *dev);
static int mio283qt9a_setcontrast(FAR struct lcd_dev_s *dev,
                                  unsigned int contrast);

/* Initialization */

static inline int mio283qt9a_hwinitialize(
                                  FAR struct mio283qt9a_dev_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This driver can support only a signal MIO283QT9A device.
 * The following is the single MIO283QT9A driver state instance:
 */

static struct mio283qt9a_dev_s g_lcddev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  mio283qt9a_putreg
 *
 * Description:
 *   Write to an LCD register
 *
 ****************************************************************************/

static void mio283qt9a_putreg(FAR struct mio283qt9a_lcd_s *lcd,
                             uint8_t regaddr, uint16_t regval)
{
  /* Set the index register to the register address and write the register
   * contents
   */

  lcd->index(lcd, regaddr);
  lcd->write(lcd, regval);
}

/****************************************************************************
 * Name:  mio283qt9a_readreg
 *
 * Description:
 *   Read from an LCD register
 *
 ****************************************************************************/

#ifndef CONFIG_LCD_NOGETRUN
static uint16_t mio283qt9a_readreg(FAR struct mio283qt9a_lcd_s *lcd,
                                   uint8_t regaddr)
{
  /* Set the index register to the register address and read the register
   * contents.
   */

  lcd->index(lcd, regaddr);
  return lcd->read(lcd);
}
#endif

/****************************************************************************
 * Name:  mio283qt9a_gramselect_write
 *
 * Description:
 *   Setup to write multiple pixels to the GRAM memory
 *
 ****************************************************************************/

static inline void mio283qt9a_gramselect_write(
                                    FAR struct mio283qt9a_lcd_s *lcd)
{
  lcd->index(lcd, 0x2c);
}

/****************************************************************************
 * Name:  mio283qt9a_gramselect_read
 *
 * Description:
 *   Setup to read multiple pixels to the GRAM memory
 *
 ****************************************************************************/

static inline void mio283qt9a_gramselect_read(
                                   FAR struct mio283qt9a_lcd_s *lcd)
{
  lcd->index(lcd, 0x2e);
  lcd->readgram(lcd);
}

/****************************************************************************
 * Name:  mio283qt9a_gramwrite
 *
 * Description:
 *   Setup to read or write multiple pixels to the GRAM memory
 *
 ****************************************************************************/

static inline void mio283qt9a_gramwrite(
                          FAR struct mio283qt9a_lcd_s *lcd, uint16_t data)
{
  lcd->write(lcd, data);
}

/****************************************************************************
 * Name:  mio283qt9a_readsetup
 *
 * Description:
 *   Prime the operation by reading one pixel from the GRAM memory if
 *   necessary for this LCD type.
 *  When reading 16-bit gram data, there may be some shifts in the
 *   returned data:
 *
 *   - ILI932x: Discard first dummy read; no shift in the return data
 *
 ****************************************************************************/

#ifndef CONFIG_LCD_NOGETRUN
static inline void mio283qt9a_readsetup(FAR struct mio283qt9a_lcd_s *lcd,
                                        FAR uint16_t *accum)
{
#if 0 /* Probably not necessary... untested */
  /* Read-ahead one pixel */

  *accum = lcd->read(lcd);
#endif
}
#endif

/****************************************************************************
 * Name:  mio283qt9a_gramread
 *
 * Description:
 *   Read one correctly aligned pixel from the GRAM memory.
 *    Possibly shifting the data and possibly swapping red and green
 *    components.
 *
 *   - ILI932x: Unknown -- assuming colors are in the color order
 *
 ****************************************************************************/

#ifndef CONFIG_LCD_NOGETRUN
static inline uint16_t mio283qt9a_gramread(FAR struct mio283qt9a_lcd_s *lcd,
                                           FAR uint16_t *accum)
{
  /* Read the value (GRAM register already selected) */

  return lcd->readgram(lcd);
}
#endif

/****************************************************************************
 * Name:  mio283qt9a_setarea
 *
 * Description:
 *   Set the cursor position.
 *   In landscape mode, the "column" is actually the physical
 *   Y position and the "row" is the physical X position.
 *
 ****************************************************************************/

static void mio283qt9a_setarea(FAR struct mio283qt9a_lcd_s *lcd,
                          uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
  mio283qt9a_putreg(lcd, 0x2a, (x0 >> 8)); /* Set column address x0 */
  lcd->write(lcd, (x0 & 0xff));            /* Set x0 */
  lcd->write(lcd, (x1 >> 8));              /* Set x1 */
  lcd->write(lcd, (x1 & 0xff));            /* Set x1 */

  mio283qt9a_putreg(lcd, 0x2b, (y0 >> 8)); /* Set page address y0 */
  lcd->write(lcd, (y0 & 0xff));            /* Set y0 */
  lcd->write(lcd, (y1 >> 8));              /* Set y1 */
  lcd->write(lcd, (y1 & 0xff));            /* Set y1 */
}

/****************************************************************************
 * Name:  mio283qt9a_dumprun
 *
 * Description:
 *   Dump the contexts of the run buffer:
 *
 *  run     - The buffer in containing the run read to be dumped
 *  npixels - The number of pixels to dump
 *
 ****************************************************************************/

#if 0 /* Sometimes useful */
static void mio283qt9a_dumprun(FAR const char *msg,
                               FAR uint16_t *run,
                               size_t npixels)
{
  int i;
  int j;

  syslog(LOG_INFO, "\n%s:\n", msg);
  for (i = 0; i < npixels; i += 16)
    {
      up_putc(' ');
      syslog(LOG_INFO, " ");
      for (j = 0; j < 16; j++)
        {
          syslog(LOG_INFO, " %04x", *run++);
        }

      up_putc('\n');
    }
}
#endif

/****************************************************************************
 * Name:  mio283qt9a_putrun
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

static int mio283qt9a_putrun(FAR struct lcd_dev_s *dev,
                             fb_coord_t row, fb_coord_t col,
                             FAR const uint8_t *buffer,
                             size_t npixels)
{
  FAR struct mio283qt9a_dev_s *priv = (FAR struct mio283qt9a_dev_s *)dev;
  FAR struct mio283qt9a_lcd_s *lcd = priv->lcd;
  FAR const uint16_t *src = (FAR const uint16_t *)buffer;
  int i;

  /* Buffer must be provided and aligned to a 16-bit address boundary */

  DEBUGASSERT(buffer && ((uintptr_t)buffer & 1) == 0);

  /* Select the LCD */

  lcd->select(lcd);

  /* Write the run to GRAM. */

  mio283qt9a_setarea(lcd, col, row, col + npixels - 1, row);
  mio283qt9a_gramselect_write(lcd);

  for (i = 0; i < npixels; i++)
    {
      mio283qt9a_gramwrite(lcd, *src);
      src++;
    }

  /* De-select the LCD */

  lcd->deselect(lcd);
  return OK;
}

/****************************************************************************
 * Name:  mio283qt9a_getrun
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

static int mio283qt9a_getrun(FAR struct lcd_dev_s *dev,
                             fb_coord_t row, fb_coord_t col,
                             FAR uint8_t *buffer,
                             size_t npixels)
{
#ifndef CONFIG_LCD_NOGETRUN
  FAR struct mio283qt9a_dev_s *priv = (FAR struct mio283qt9a_dev_s *)dev;
  FAR struct mio283qt9a_lcd_s *lcd = priv->lcd;
  FAR uint16_t *dest = (FAR uint16_t *)buffer;
  uint16_t accum;
  uint16_t test;
  int i;

  /* Buffer must be provided and aligned to a 16-bit address boundary */

  lcdinfo("mio283qt9a_getrun row: %d col: %d npixels: %d\n",
           row, col, npixels);
  DEBUGASSERT(buffer && ((uintptr_t)buffer & 1) == 0);

  /* Read the run from GRAM. */

  /* Select the LCD */

  lcd->select(lcd);

  /* Red the run fram GRAM. */

  mio283qt9a_setarea(lcd, col, row, col + npixels - 1, row);
  mio283qt9a_gramselect_read(lcd);

  /* Prime the pump for unaligned read data */

  mio283qt9a_readsetup(lcd, &accum);

  for (i = 0; i < npixels; i++)
    {
      test = mio283qt9a_gramread(lcd, &accum);
      *dest++ = test;
    }

  /* De-select the LCD */

  lcd->deselect(lcd);
  return OK;
#else
  return -ENOSYS;
#endif
}

/****************************************************************************
 * Name:  mio283qt9a_getvideoinfo
 *
 * Description:
 *   Get information about the LCD video controller configuration.
 *
 ****************************************************************************/

static int mio283qt9a_getvideoinfo(FAR struct lcd_dev_s *dev,
                                   FAR struct fb_videoinfo_s *vinfo)
{
  DEBUGASSERT(dev && vinfo);
  lcdinfo("fmt: %d xres: %d yres: %d nplanes: 1\n",
          MIO283QT9A_COLORFMT, MIO283QT9A_XRES, MIO283QT9A_YRES);

  vinfo->fmt     = MIO283QT9A_COLORFMT;  /* Color format: RGB16-565: RRRR RGGG GGGB BBBB */
  vinfo->xres    = MIO283QT9A_XRES;      /* Horizontal resolution in pixel columns */
  vinfo->yres    = MIO283QT9A_YRES;      /* Vertical resolution in pixel rows */
  vinfo->nplanes = 1;                    /* Number of color planes supported */
  return OK;
}

/****************************************************************************
 * Name:  mio283qt9a_getplaneinfo
 *
 * Description:
 *   Get information about the configuration of each LCD color plane.
 *
 ****************************************************************************/

static int mio283qt9a_getplaneinfo(FAR struct lcd_dev_s *dev,
                                   unsigned int planeno,
                                   FAR struct lcd_planeinfo_s *pinfo)
{
  FAR struct mio283qt9a_dev_s *priv = (FAR struct mio283qt9a_dev_s *)dev;

  DEBUGASSERT(dev && pinfo && planeno == 0);
  lcdinfo("planeno: %d bpp: %d\n", planeno, MIO283QT9A_BPP);

  pinfo->putrun = mio283qt9a_putrun;               /* Put a run into LCD memory */
  pinfo->getrun = mio283qt9a_getrun;               /* Get a run from LCD memory */
  pinfo->buffer = (FAR uint8_t *)priv->runbuffer;  /* Run scratch buffer */
  pinfo->bpp    = MIO283QT9A_BPP;                  /* Bits-per-pixel */
  pinfo->dev    = dev;                             /* The lcd device */

  return OK;
}

/****************************************************************************
 * Name:  mio283qt9a_getpower
 *
 * Description:
 *   Get the LCD panel power status
 *  (0: full off - CONFIG_LCD_MAXPOWER: full on).
 *   On backlit LCDs, this setting may correspond to the backlight setting.
 *
 ****************************************************************************/

static int mio283qt9a_getpower(FAR struct lcd_dev_s *dev)
{
  lcdinfo("getpower: %d\n", 0);
  return g_lcddev.power;
}

/****************************************************************************
 * Name:  mio283qt9a_poweroff
 *
 * Description:
 *   Enable/disable LCD panel power
 *  (0: full off - CONFIG_LCD_MAXPOWER: full on).
 *   On backlit LCDs, this setting may correspond to the backlight setting.
 *
 ****************************************************************************/

static int mio283qt9a_poweroff(FAR struct mio283qt9a_lcd_s *lcd)
{
  /* Select the LCD */

  lcdinfo("mio283qt9a_poweroff\n");

  lcd->select(lcd);

  /* Set the backlight off */

  lcd->backlight(lcd, 0);

  /* Turn the display off */

  mio283qt9a_putreg(lcd, 0x28, 0x0000); /* GON=0, DTE=0, D=0 */

  /* Deselect the LCD */

  lcd->deselect(lcd);

  /* Remember the power off state */

  g_lcddev.power = 0;
  return OK;
}

/****************************************************************************
 * Name:  mio283qt9a_setpower
 *
 * Description:
 *   Enable/disable LCD panel power
 *  (0: full off - CONFIG_LCD_MAXPOWER: full on).
 *   On backlit LCDs, this setting may correspond to the backlight setting.
 *
 ****************************************************************************/

static int mio283qt9a_setpower(FAR struct lcd_dev_s *dev, int power)
{
  FAR struct mio283qt9a_dev_s *priv = (FAR struct mio283qt9a_dev_s *)dev;
  FAR struct mio283qt9a_lcd_s *lcd  = priv->lcd;

  lcdinfo("setpower: %d\n", power);
  DEBUGASSERT((unsigned)power <= CONFIG_LCD_MAXPOWER);

  /* Set new power level */

  if (power > 0)
    {
      /* Select the LCD */

      lcd->select(lcd);

      /* Set the backlight level */

      lcd->backlight(lcd, power);

      /* Then turn the display on: */

      mio283qt9a_putreg(lcd, 0x29, 0x00); /* GON=1, DTE=1, D=2 */

      /* Deselect the LCD */

      lcd->deselect(lcd);

      /* Remember the power on state */

      g_lcddev.power = power;
    }
  else
    {
      /* Turn the display off */

      mio283qt9a_poweroff(lcd);
    }

  return OK;
}

/****************************************************************************
 * Name:  mio283qt9a_getcontrast
 *
 * Description:
 *   Get the current contrast setting (0-CONFIG_LCD_MAXCONTRAST).
 *
 ****************************************************************************/

static int mio283qt9a_getcontrast(FAR struct lcd_dev_s *dev)
{
  lcdinfo("Not implemented\n");
  return -ENOSYS;
}

/****************************************************************************
 * Name:  mio283qt9a_setcontrast
 *
 * Description:
 *   Set LCD panel contrast (0-CONFIG_LCD_MAXCONTRAST).
 *
 ****************************************************************************/

static int mio283qt9a_setcontrast(FAR struct lcd_dev_s *dev,
                                  unsigned int contrast)
{
  lcdinfo("contrast: %d\n", contrast);
  return -ENOSYS;
}

/****************************************************************************
 * Name:  mio283qt9a_hwinitialize
 *
 * Description:
 *   Initialize the LCD hardware.
 *
 ****************************************************************************/

static inline int mio283qt9a_hwinitialize(FAR struct mio283qt9a_dev_s *priv)
{
  FAR struct mio283qt9a_lcd_s *lcd  = priv->lcd;
#if !defined(CONFIG_LCD_NOGETRUN) || defined(CONFIG_DEBUG_LCD)
  uint16_t id_a;
  uint16_t id_b;
  uint16_t id_c;
  uint16_t id_d;
#endif
#ifdef CONFIG_DEBUG_LCD
  uint16_t id_e;
#endif
  int ret;

  /* Select the LCD */

  lcd->select(lcd);

  /* Read the HIMAX ID registger (0x00) */

#ifndef CONFIG_LCD_NOGETRUN
  id_a = mio283qt9a_readreg(lcd, 0xd3);
  id_b = lcd->read(lcd);
  id_c = lcd->read(lcd);
  id_d = lcd->read(lcd);

  lcdinfo("LCD ID: %04x %04x %04x %04x\n", id_a, id_b, id_c, id_d);
  UNUSED(id_a);
  UNUSED(id_b);

  /* Check if the ID is for the ILI9341 */

  if (id_c == ILI9341_ID_1 && id_d == ILI9341_ID_2)
#endif
    {
      mio283qt9a_putreg(lcd, 0x29, 0);      /* Power on */

      mio283qt9a_putreg(lcd, 0x13, 0);      /* Normal display on */
      mio283qt9a_putreg(lcd, 0xc0, 0x28);   /* Power control C1 */
      mio283qt9a_putreg(lcd, 0xc1, 0x01);   /* Power control C2 BT = 1 */
      mio283qt9a_putreg(lcd, 0xc5, 0x24);   /* VCOM control, VMH = 3.6v */
      lcd->write(lcd, 0x25);                /* VML = -1.575v */
      mio283qt9a_putreg(lcd, 0x26, 0x01);   /* Gamma 2.2 */
      mio283qt9a_putreg(lcd, 0x3a, 0x55);   /* Pixel format set command */

#if defined(CONFIG_LCD_LANDSCAPE)
      mio283qt9a_putreg(lcd, 0x36, 0x0028); /* MY=1, MX=0, MV=1, ML=0, BGR=1 */
#elif defined(CONFIG_LCD_PORTRAIT)
      mio283qt9a_putreg(lcd, 0x36, 0x0008); /* MY=0, MX=0, MV=0, ML=0, BGR=1 */
#elif defined(CONFIG_LCD_RLANDSCAPE)
      mio283qt9a_putreg(lcd, 0x36, 0x0068); /* MY=0, MX=1, MV=1, ML=0, BGR=1 */
#elif defined(CONFIG_LCD_RPORTRAIT)
      mio283qt9a_putreg(lcd, 0x36, 0x00c8); /* MY=1, MX=0, MV=1, ML=0, BGR=1 */
#endif

      /* Window setting */

      mio283qt9a_setarea(lcd, 0, 0, (MIO283QT9A_XRES - 1),
                        (MIO283QT9A_YRES - 1));
      mio283qt9a_putreg(lcd, 0x11, 0);      /* Sleep out mode */
      up_mdelay(25);

#ifdef CONFIG_DEBUG_LCD
      /* Read back some info from the panel */

      id_a = mio283qt9a_readreg(lcd, 0x04); /* Read display information */
      id_b = lcd->read(lcd);
      id_c = lcd->read(lcd);
      id_d = lcd->read(lcd);
      lcdinfo("LCD man ID: %02x, version: %02x, driver ID: %02x\n",
               id_b, id_c, id_d);

      id_a = mio283qt9a_readreg(lcd, 0x09); /* Read display status */
      id_b = lcd->read(lcd);
      id_c = lcd->read(lcd);
      id_d = lcd->read(lcd);
      id_e = lcd->read(lcd);
      lcdinfo("Display status %02x, %02x, %02x, %02x, %02x\n",
               id_a, id_b, id_c, id_d, id_e);

      id_a = mio283qt9a_readreg(lcd, 0x0a); /* Read power status */
      id_b = lcd->read(lcd);
      lcdinfo("Power status %02x, %02x\n", id_a, id_b);

      id_a = mio283qt9a_readreg(lcd, 0x0b); /* Read MADCTL */
      id_b = lcd->read(lcd);
      lcdinfo("MADCTL %02x, %02x\n", id_a, id_b);

      id_a = mio283qt9a_readreg(lcd, 0x0c); /* Read pixel format */
      id_b = lcd->read(lcd);
      lcdinfo("Pixel format %02x, %02x\n", id_a, id_b);

      id_a = mio283qt9a_readreg(lcd, 0x0d); /* Read image format */
      id_b = lcd->read(lcd);
      lcdinfo("Image format %02x, %02x\n", id_a, id_b);

      id_a = mio283qt9a_readreg(lcd, 0x0e);  /* read signal mode */
      id_b = lcd->read(lcd);
      lcdinfo("Signal mode %02x, %02x\n", id_a, id_b);

      id_a = mio283qt9a_readreg(lcd, 0x0f);  /* read self diag */
      id_b = lcd->read(lcd);
      lcdinfo("Self diag %02x, %02x\n", id_a, id_b);
#endif
      ret = OK;
    }
#ifndef CONFIG_LCD_NOGETRUN
  else
    {
      lcderr("ERROR: Unsupported LCD type\n");
      ret = -ENODEV;
    }
#endif

  /* De-select the LCD */

  lcd->deselect(lcd);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  mio283qt9a_lcdinitialize
 *
 * Description:
 *   Initialize the LCD video hardware.
 *   The initial state of the LCD is fully initialized, display memory
 *   cleared, and the LCD ready to use, but with the power setting at 0
 *   (full off).
 *
 ****************************************************************************/

FAR struct lcd_dev_s *mio283qt9a_lcdinitialize(
                                        FAR struct mio283qt9a_lcd_s *lcd)
{
  FAR struct mio283qt9a_dev_s *priv;
  int ret;

  lcdinfo("Initializing\n");

  /* If we could support multiple MIO283QT9A devices, this is where we would
   * allocate a new driver data structure.
   */

  priv = &g_lcddev;

  /* Initialize the driver data structure */

  priv->dev.getvideoinfo = mio283qt9a_getvideoinfo;
  priv->dev.getplaneinfo = mio283qt9a_getplaneinfo;
  priv->dev.getpower     = mio283qt9a_getpower;
  priv->dev.setpower     = mio283qt9a_setpower;
  priv->dev.getcontrast  = mio283qt9a_getcontrast;
  priv->dev.setcontrast  = mio283qt9a_setcontrast;
  priv->lcd              = lcd;

  /* Configure and enable LCD */

  ret = mio283qt9a_hwinitialize(priv);
  if (ret == OK)
    {
      /* Clear the display (setting it to the color 0=black) */

      mio283qt9a_clear(&priv->dev, 0x0000);

      /* Turn the display off */

      mio283qt9a_poweroff(lcd);
      return &g_lcddev.dev;
    }

  return NULL;
}

/****************************************************************************
 * Name:  mio283qt9a_clear
 *
 * Description:
 *   This is a non-standard LCD interface just for the stm3240g-EVAL board.
 *   Because of the various rotations, clearing the display in the normal
 *   way by writing a sequences of runs that covers the entire display can
 *   be very slow.  Here the display is cleared by simply setting all GRAM
 *   memory to the specified color.
 *
 ****************************************************************************/

void mio283qt9a_clear(FAR struct lcd_dev_s *dev, uint16_t color)
{
  FAR struct mio283qt9a_dev_s *priv = (FAR struct mio283qt9a_dev_s *)dev;
  FAR struct mio283qt9a_lcd_s *lcd  = priv->lcd;
  uint32_t i;

  /* Select the LCD and set the drawring area */

  lcd->select(lcd);
  mio283qt9a_setarea(lcd, 0, 0, (MIO283QT9A_XRES - 1),
                    (MIO283QT9A_YRES - 1));

  /* Prepare to write GRAM data */

  mio283qt9a_gramselect_write(lcd);

  /* Copy color into all of GRAM.
   *  Orientation does not matter in this case.
   */

  for (i = 0; i < MIO283QT9A_XRES * MIO283QT9A_YRES; i++)
    {
      mio283qt9a_gramwrite(lcd, color);
    }

  /* De-select the LCD */

  lcd->deselect(lcd);
}

#endif /* CONFIG_LCD_MIO283QT9A */
