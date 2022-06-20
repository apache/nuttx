/****************************************************************************
 * drivers/lcd/ssd1680.c
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

/* Driver for e-paper displays with SSD1680 controller.
 *
 * References:
 *   1. https://www.crystalfontz.com/controllers/SolomonSystech/SSD1680/497/
 *   2. https://www.adafruit.com/product/4947
 *   3. https://github.com/adamkaliszan/TTGO-Electronic-Badge

 * TTGO-Electronic-Badge sequence that display picture:
 *      cmd:      dta:
 * Hardware reset and busy wait
 *   1)   0x01      27 01 00
 *   2)   0x0C      D7 D6 9D         boost soft start
 *   3)   0x2c      A8               write VCom
 *   4)   0x3A      1A           ??? Can't find it in in SSD documentation
 *   5)   0x3B      08           ??? Can't find it in in SSD documentation
 *   6)   0x11      01               Data Mode
 *   7)   0x44      00 0F
 *   8)   0x45      27 01 00 00
 *   9)   0x4E      00
 *   10)  0x4F      27 01
 *   11)  0x32      50 AA 55 AA 11 00 00 00 00 00 00 00 00 00 00 00
 *                  00 00 00 00 FF FF 1F 00 00 00 00 00 00 00
 *   12)  0x22      C0
 *   13)  0x20
 * Busy Wait
 *   14)  0x24      0xFF ... 4736 bytes with bitmap
 *   15)  0x22      C4
 *   16)  0x20
 * Busy Wait
 *   17)  0xFF
 *   18)  0x22      C3
 *   19)  0x20
 * Busy Wait
 */

/****************************************************************************
 * Device memory organization:
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
#include <nuttx/lcd/ssd1680.h>
#include <nuttx/signal.h>

#include <arch/irq.h>

#include "ssd1680.h"

#ifdef CONFIG_LCD_SSD1680
/* This structure describes the state of the SSD1680 driver */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ssd1680_dev_s
{
  struct lcd_dev_s       dev;      /* Publicly visible device structure */

  /* Private LCD-specific information follows */

  FAR struct spi_dev_s  *spi;      /* Cached SPI device reference */
  uint8_t                contrast; /* Current contrast setting */
  bool                   on;       /* true: display is on */
  bool                   is_conf;  /* true: display had been configured */

  FAR const struct ssd1680_priv_s *board_priv; /* Board specific structure */

  /* The SSD1680 does not support reading from the display memory in SPI
   * mode. Since there is 1 BPP and access is byte-by-byte.
   * Also shared line (MISO/MOSI) could be problematic. Now implementation
   * uses shadow buffer.
   * Its size depends on resolution and is between 4kB and 32 kB.
   */

  uint8_t shadow_fb[SSD1680_DEV_FBSIZE];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Libc extension */

FAR void *bitscpy_ds(FAR void *dest, int dest_offset, FAR const void *src,
    size_t nbits);

FAR void *bitscpy_ss(FAR void *dest, FAR const void *src, int src_offset,
    size_t nbits);

/* LCD Data Transfer Methods */

static void ssd1680_busy_wait(FAR struct ssd1680_dev_s *priv);

static void ssd1680_snd_cmd_with_data0(FAR struct ssd1680_dev_s *priv,
    uint8_t cmd);

static void ssd1680_snd_cmd_with_data1(FAR struct ssd1680_dev_s *priv,
    uint8_t cmd, uint8_t dta1);

static void ssd1680_snd_cmd_with_data2(FAR struct ssd1680_dev_s *priv,
    uint8_t cmd, uint8_t dta1, uint8_t dta2);

static void ssd1680_snd_cmd_with_data3(FAR struct ssd1680_dev_s *priv,
    uint8_t cmd, uint8_t dta1, uint8_t dta2, uint8_t dta3);

static void ssd1680_snd_cmd_with_data4(FAR struct ssd1680_dev_s *priv,
    uint8_t cmd, uint8_t dta1, uint8_t dta2, uint8_t dta3, uint8_t dta4);

static void ssd1680_snd_cmd_with_data(FAR struct ssd1680_dev_s *priv,
    uint8_t cmd, const uint8_t *dta, int dta_len);

#if !defined(CONFIG_LCD_PORTRAIT) && !defined(CONFIG_LCD_RPORTRAIT)
#  if SSD1680_DEV_BPP == 1
static void ssd1680_snd_cmd_with_data_bitstrip(
    FAR struct ssd1680_dev_s *priv, uint8_t cmd, const uint8_t *dta,
    int dta_len, int strip_len);
#  else
/* Special functions for sending to RAM1 and RAM2 should be implemented
 * ssd1680_snd_cmd_with_data_bitstrip works fine with 1 bit per pixel
 */
#    error "SSD1680 driver has no implementation for 3 color with landscape"
#  endif
#endif

static void ssd1680_configspi(FAR struct spi_dev_s *spi);
static void ssd1680_select(FAR struct ssd1680_dev_s *priv, bool cs);
static void ssd1680_cmddata(FAR struct ssd1680_dev_s *priv, bool cmd);

static int ssd1680_putrun(FAR struct lcd_dev_s *dev, fb_coord_t row,
                          fb_coord_t col, FAR const uint8_t *buffer,
                          size_t npixels);

static int ssd1680_getrun(FAR struct lcd_dev_s *dev, fb_coord_t row,
                          fb_coord_t col, FAR uint8_t *buffer,
                          size_t npixels);

/* LCD Configuration */

static int ssd1680_getvideoinfo(FAR struct lcd_dev_s *dev,
                                FAR struct fb_videoinfo_s *vinfo);
static int ssd1680_getplaneinfo(FAR struct lcd_dev_s *dev,
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

static int ssd1680_getpower(struct lcd_dev_s *dev);
static int ssd1680_setpower(struct lcd_dev_s *dev, int power);
static int ssd1680_getcontrast(struct lcd_dev_s *dev);
static int ssd1680_setcontrast(struct lcd_dev_s *dev,
                               unsigned int contrast);

static int ssd1680_configuredisplay(struct ssd1680_dev_s *priv);
static int ssd1680_redraw_display(struct ssd1680_dev_s *priv);
static int ssd1680_redrawfb_row(struct ssd1680_dev_s *priv, int row);
static int ssd1680_redrawfb(struct ssd1680_dev_s *priv);

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

static uint8_t g_runbuffer[SSD1680_DEV_ROWSIZE];

/* This structure describes the overall LCD video controller */

static const struct fb_videoinfo_s g_videoinfo =
{
  .fmt     = SSD1680_DEV_COLORFMT,    /* Color format: B&W */
  .xres    = SSD1680_DEV_FB_XRES,     /* Horizontal resolution in pixel columns */
  .yres    = SSD1680_DEV_FB_YRES,     /* Vertical resolution in pixel rows */
  .nplanes = SSD1680_NO_OF_PLANES,    /* Number of color planes supported */
};

/* This is the standard, NuttX Plane information object */

static const struct lcd_planeinfo_s g_planeinfo =
{
  .putrun  = ssd1680_putrun,             /* Put a run into LCD memory */
  .putarea = NULL,                       /* Not need to implement */
  .getrun  = ssd1680_getrun,             /* Read content of shadow memory */
  .getarea = NULL,                       /* Not need to implement */
  .buffer  = (FAR uint8_t *)g_runbuffer, /* Run scratch buffer */
  .bpp     = SSD1680_DEV_BPP,            /* Bits-per-pixel */
};

/* This is the outside visible interface for the OLED driver */

static const struct lcd_dev_s g_lcd_epaper_dev =
{
  /* LCD Configuration */

  .getvideoinfo = ssd1680_getvideoinfo,
  .getplaneinfo = ssd1680_getplaneinfo,

  /* LCD RGB Mapping -- Not supported */

  /* Cursor Controls -- Not supported */

  /* LCD Specific Controls */

  .getpower     = ssd1680_getpower,
  .setpower     = ssd1680_setpower,
  .getcontrast  = ssd1680_getcontrast,
  .setcontrast  = ssd1680_setcontrast,
};

/* This is the OLED driver instance. Only a single device is supported
 * for now.
 */

static struct ssd1680_dev_s g_epaperdev;

static const uint8_t ssd1680_lut[] =
{
  0x50, 0xaa, 0x55, 0xaa, 0x11, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x1f, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  ssd1680_putrun
 *
 * Description:
 *   This method can be used to write a partial raster line to the LCD.
 *
 * Input Parameters:
 *   dev     - The lcd device
 *   row     - Starting row to write to (range: 0 <= row < yres)
 *   col     - Starting column to write to (range: 0 <= col <= xres-npixels)
 *   buffer  - The buffer containing the run to be written to the LCD
 *   npixels - The number of pixels to write to the LCD
 *             (range: 0 < npixels <= xres-col)
 *
 ****************************************************************************/

static int ssd1680_putrun(FAR struct lcd_dev_s *dev, fb_coord_t row,
                          fb_coord_t col, FAR const uint8_t *buffer,
                          size_t npixels)
{
  FAR struct ssd1680_dev_s *priv = (FAR struct ssd1680_dev_s *)dev;

  uint8_t *dst = priv->shadow_fb +
      row * SSD1680_DEV_ROWSIZE + (col >> SSD1680_PDF);

  int dst_start_bitshift = col % (SSD1680_PDV);

  /* Write data to shadow memory */

  bitscpy_ds(dst,  dst_start_bitshift, buffer, npixels);

  /* Redraw Screen */

  ssd1680_redrawfb_row(priv, row);

  return OK;
}

/****************************************************************************
 * Name:  ssd1680_getrun
 *
 * Description:
 *   This method can be used to read a partial raster line from the LCD:
 *
 * Input Parameters:
 *
 *  dev     - The lcd device
 *  row     - Starting row to read from (range: 0 <= row < yres)
 *  col     - Starting column to read read
 *            (range: 0 <= col <= xres-npixels)
 *  buffer  - The buffer in which to return the run read from the LCD
 *  npixels - The number of pixels to read from the LCD
 *            (range: 0 < npixels <= xres-col)
 */

static int ssd1680_getrun(FAR struct lcd_dev_s *dev, fb_coord_t row,
                          fb_coord_t col, FAR uint8_t *buffer,
                          size_t npixels)
{
  lcdinfo("(%d, %d, %d)\n", row, col, npixels);
  FAR struct ssd1680_dev_s *priv = (FAR struct ssd1680_dev_s *)dev;

  bitscpy_ss(buffer,
      priv->shadow_fb + row * SSD1680_DEV_FBSIZE + (col >> SSD1680_PDF),
      col % SSD1680_PDV,
      npixels);

  return OK;
}

/****************************************************************************
 * Name:  ssd1680_getvideoinfo
 *
 * Description:
 *   Get information about the LCD video controller configuration.
 *
 ****************************************************************************/

static int ssd1680_getvideoinfo(FAR struct lcd_dev_s *dev,
                                FAR struct fb_videoinfo_s *vinfo)
{
  DEBUGASSERT(dev && vinfo);
  lcdinfo("fmt: %d xres: %d yres: %d nplanes: %d\n",
          g_videoinfo.fmt, g_videoinfo.xres, g_videoinfo.yres,
          g_videoinfo.nplanes);
  memcpy(vinfo, &g_videoinfo, sizeof(struct fb_videoinfo_s));
  return OK;
}

/****************************************************************************
 * Name:  ssd1680_getplaneinfo
 *
 * Description:
 *   Get information about the configuration of each LCD color plane.
 *
 ****************************************************************************/

static int ssd1680_getplaneinfo(FAR struct lcd_dev_s *dev,
                                unsigned int planeno,
                                FAR struct lcd_planeinfo_s *pinfo)
{
  DEBUGASSERT(pinfo && planeno == 0);

  lcdinfo("planeno: %d bpp: %d\n", planeno, g_planeinfo.bpp);
  memcpy(pinfo, &g_planeinfo, sizeof(struct lcd_planeinfo_s));
  pinfo->dev = dev;
  return OK;
}

/****************************************************************************
 * Name:  ssd1680_getpower
 *
 * Description:
 *   Get the LCD panel power status:
 *     0: full off
 *     CONFIG_LCD_MAXPOWER: full on
 *   On backlit LCDs, this setting may correspond to the backlight setting.
 *
 ****************************************************************************/

static int ssd1680_getpower(FAR struct lcd_dev_s *dev)
{
  struct ssd1680_dev_s *priv = (struct ssd1680_dev_s *) dev;
  DEBUGASSERT(priv);

  lcdinfo("power: %s\n", priv->on ? "ON" : "OFF");
  return priv->on ? CONFIG_LCD_MAXPOWER : 0;
}

static void ssd1680_reset(struct ssd1680_dev_s *priv)
{
  if (priv->board_priv && priv->board_priv->set_rst)
    {
      lcdinfo("Hardware reset\n");
      priv->board_priv->set_rst(false);
      nxsig_usleep(10);
      priv->board_priv->set_rst(true);
    }
  else
    {
      lcdinfo("Hardware reset is not available. Operation skipped.\n");
    }
}

/****************************************************************************
 * Name:  ssd1680_setpower
 *
 * Description:
 *   Enable/disable LCD panel power:
 *     0: full off
 *     CONFIG_LCD_MAXPOWER: full on
 *   On backlit LCDs, this setting may correspond to the backlight setting.
 *
 ****************************************************************************/

static int ssd1680_setpower(FAR struct lcd_dev_s *dev, int power)
{
  int ret = OK;
  struct ssd1680_dev_s *priv = (struct ssd1680_dev_s *) dev;
  DEBUGASSERT(dev);
  lcdinfo("power: %d -> %d\n", priv->on ? CONFIG_LCD_MAXPOWER : 0, power);
  DEBUGASSERT((unsigned)power <= CONFIG_LCD_MAXPOWER);

  if (power <= 0)
    {
      priv->on = false;

      /* Try turn off power completely */

      if (priv->board_priv && priv->board_priv->set_vcc)
        {
          /* Do power off. */

          if (priv->board_priv->set_vcc(false))
            {
              /* Display is completely powered off, not configured anymore. */

            priv->is_conf = false;
            }
        }
    }
  else
    {
      if (priv->board_priv && priv->board_priv->set_vcc)
        {
          /* Do power on. */

          lcdinfo("Set Pwr Ctrl Linepower ON\n");
          priv->board_priv->set_vcc(true);
          nxsig_usleep(10000);
        }
      else
        {
          lcdinfo("No line for controlling PWR, operation skipped\n");
        }

      if (!priv->is_conf)
        {
          ssd1680_reset(priv);

          ret = ssd1680_configuredisplay(priv);
          if (ret < 0)
            {
              return ret;
            }

          /* Draw the framebuffer */

          ret = ssd1680_redrawfb(priv);
        }

      if (ret < 0)
        {
          return ret;
        }

      priv->on = true;
    }

  return ret;
}

/****************************************************************************
 * Name:  ssd1680_getcontrast
 *
 * Description:
 *   Get the current contrast setting (0-CONFIG_LCD_MAXCONTRAST).
 *
 ****************************************************************************/

static int ssd1680_getcontrast(struct lcd_dev_s *dev)
{
  struct ssd1680_dev_s *priv = (struct ssd1680_dev_s *)dev;
  DEBUGASSERT(priv);

  lcdinfo("contrast: %d\n", priv->contrast);
  return priv->contrast;
}

/****************************************************************************
 * Name:  ssd1680_setcontrast
 *
 * Description:
 *   Set LCD panel contrast (0-CONFIG_LCD_MAXCONTRAST).
 *
 ****************************************************************************/

static int ssd1680_setcontrast(struct lcd_dev_s *dev, unsigned int contrast)
{
  struct ssd1680_dev_s *priv = (struct ssd1680_dev_s *)dev;

  lcdinfo("ignoring set constrast(%d)\n", contrast);
  DEBUGASSERT(priv);

  /* Verify the contrast value */

#ifdef CONFIG_DEBUG_FEATURES
  if (contrast > CONFIG_LCD_MAXCONTRAST)
    {
      return -EINVAL;
    }
#endif

  return OK;
}

/****************************************************************************
 * Name:  ssd1680_configuredisplay
 *
 * Description:
 *   Setup LCD display.
 *
 ****************************************************************************/

static int ssd1680_configuredisplay(struct ssd1680_dev_s *priv)
{
  lcdinfo("Start\n");

  /* Software Reset */

  lcdinfo("Software reset: (0x%02x)\n", SSD1680_SW_RESET);
  ssd1680_snd_cmd_with_data0(priv, SSD1680_SW_RESET);

  /* Busy wait */

  ssd1680_busy_wait(priv);
  lcdinfo("SSD1680 is ready\n");

  /* Step 1: Driver Output Control 3 bytes of data:
   * - A[8:0] MUX Gate lines
   * - B[2:0] sequence
   * last data byte depends on connection between display and controller
   */

  lcdinfo("Set the driver output controll (0x%02x): %d 0x%02x\n",
      SSD1680_DRIVER_CONTROL, SSD1680_DEV_NATIVE_YRES - 1,
      SSD1680_DEV_GATE_LAYOUT);
  ssd1680_snd_cmd_with_data3(priv, SSD1680_DRIVER_CONTROL,
     (uint8_t)((SSD1680_DEV_NATIVE_YRES - 1) & 0xff),
     (SSD1680_DEV_NATIVE_YRES - 1) >> 8,
     SSD1680_DEV_GATE_LAYOUT);

  /* Step 2: SSD1680_BOOST_SOFTSTART 0x0C D7 D6 9D */

  lcdinfo("Set boost soft start\n");
  ssd1680_snd_cmd_with_data3(priv, SSD1680_BOOST_SOFTSTART,
      0xd7, 0xd6, 0x9d);

  /* Step 3: Vcom Voltage SSD1680_WRITE_VCOM, 0x36 */

  lcdinfo("Set Vcom voltage (0x%02x): 0x%2x\n", SSD1680_WRITE_VCOM, 0xa8);
  ssd1680_snd_cmd_with_data1(priv, SSD1680_WRITE_VCOM, 0xa8);

  /* Step 4: Sending undocumented command: 0x3a with data 1A */

  lcdinfo("Set (0x%02x): 0x%2x\n", 0x3a, 0x1a);
  ssd1680_snd_cmd_with_data1(priv, 0x3a, 0x1a);

  /* Step 5: Sending undocumented command: 0x3b with data 08 */

  lcdinfo("Set (0x%02x): 0x%2x\n", 0x3b, 0x08);
  ssd1680_snd_cmd_with_data1(priv, 0x3b, 0x08);

  /* Step 6: Data entry mode SSD1680_DATA_MODE, 0x03
   * TODO w arduino była wartość 0x01
   */

  lcdinfo("Set data entry mode (0x%02x): 0x%2x\n",
      SSD1680_DATA_MODE, SSD1680_VAL_DATA_MODE);
  ssd1680_snd_cmd_with_data1(priv,
      SSD1680_DATA_MODE, SSD1680_VAL_DATA_MODE);

  /* Step 7: Set ram X start/end postion 00 FF */

  lcdinfo("Set ram X start/end position (0x%02x): 0, %d\n",
      SSD1680_SET_RAMXPOS, (SSD1680_DEV_X_ROUND_UP >> 3)-1);
  ssd1680_snd_cmd_with_data2(priv, SSD1680_SET_RAMXPOS,
      0x00, (SSD1680_DEV_X_ROUND_UP >> 3)-1);

  /* Step 8: Set ram Y start/end postion
   * TODO w adruino zamieniona start ze stopem
   */

  lcdinfo("Set ram Y start/end position (%x): 0, %d\n",
      SSD1680_SET_RAMYPOS, SSD1680_DEV_NATIVE_YRES - 1);
  ssd1680_snd_cmd_with_data4(priv, SSD1680_SET_RAMYPOS,
      0x00, 0x00,
      (uint8_t)((SSD1680_DEV_NATIVE_YRES - 1) & 0xff),
      (SSD1680_DEV_NATIVE_YRES - 1) >> 8);

  /* Step 9: SSD1680_SET_RAMXCOUNT, 0 */

  ssd1680_snd_cmd_with_data1(priv, SSD1680_SET_RAMXCOUNT, 0x00);

  /* Step 10: SSD1680_SET_RAMYCOUNT, 0, 0 */

  ssd1680_snd_cmd_with_data2(priv, SSD1680_SET_RAMYCOUNT, 0x00, 0x00);

  /* Step 11: Lookup table */

  lcdinfo("Write lookup table (%d bytes)\n", sizeof (ssd1680_lut));
  ssd1680_snd_cmd_with_data(priv, SSD1680_WRITE_LUT, ssd1680_lut,
      sizeof (ssd1680_lut));

  /* Step 12: Write sequence */

  lcdinfo("Write controll sequence 0x%02x\n", 0xc0);
  ssd1680_snd_cmd_with_data1(priv, SSD1680_DISP_CTRL2, 0xc0);

  /* Step 13: Master Activate and busy wait */

  ssd1680_snd_cmd_with_data0(priv, SSD1680_MASTER_ACTIVATE);
  ssd1680_busy_wait(priv);

  lcdinfo("Configuration ready\n");
  priv->is_conf = true;
  return OK;
}

/****************************************************************************
 * Name:  ssd1680_redrawfb
 *
 * Description:
 *   Redraw full framebuffer to display
 *
 * Input Parameters:
 *   priv   - Reference to private driver structure
 *
 * Assumptions:
 *   Caller has selected the OLED section.
 *
 ****************************************************************************/

static int ssd1680_redrawfb(struct ssd1680_dev_s *priv)
{
  /* Step 9: SSD1680_SET_RAMXCOUNT, 0 */

  ssd1680_snd_cmd_with_data1(priv, SSD1680_SET_RAMXCOUNT, 0x00);

  /* Step 10: SSD1680_SET_RAMYCOUNT, 0, 0 */

#if defined(CONFIG_LCD_PORTRAIT) | defined(CONFIG_LCD_RLANDSCAPE)
  ssd1680_snd_cmd_with_data2(priv, SSD1680_SET_RAMYCOUNT, 0x00, 0x00);
#elif defined (CONFIG_LCD_LANDSCAPE)
  ssd1680_snd_cmd_with_data2(priv, SSD1680_SET_RAMYCOUNT, 0, 0);
#elif defined (CONFIG_LCD_RORTRAIT)
#error "Not implemented LSC orientation"
#endif

  /* Step 14: */

#if SSD1680_DEV_BPP == 1
#if defined(CONFIG_LCD_PORTRAIT) || defined(CONFIG_LCD_RPORTRAIT)
  ssd1680_snd_cmd_with_data(priv,
    SSD1680_WRITE_RAM1, priv->shadow_fb, SSD1680_DEV_FBSIZE);
#else
  int line;
#  if SSD1680_DEV_BPP == 1
  for (line = 0; line < SSD1680_DEV_FB_YRES; line += 8)
    {
      ssd1680_snd_cmd_with_data_bitstrip(priv, SSD1680_WRITE_RAM1,
          priv->shadow_fb + line * SSD1680_DEV_NATIVE_YRES,
          SSD1680_DEV_NATIVE_YRES, SSD1680_DEV_ROWSIZE);
    }
#  else
#    error "3 color mode not implemented yet"
#  endif
#endif

#else
  ssd1680_snd_cmd_with_data_even_bits(priv,
    SSD1680_WRITE_RAM1, priv->shadow_fb, SSD1680_DEV_FBSIZE);

  /* Step 9: SSD1680_SET_RAMXCOUNT, 0 */

  ssd1680_snd_cmd_with_data1(priv, SSD1680_SET_RAMXCOUNT, 0x00);

  /* Step 10: SSD1680_SET_RAMYCOUNT, 0, 0 */

  ssd1680_snd_cmd_with_data2(priv, SSD1680_SET_RAMYCOUNT, 0x00, 0x00);

  /* Step 14b: */

  ssd1680_snd_cmd_with_data_odd_bits(priv,
    SSD1680_WRITE_RAM2, priv->shadow_fb, SSD1680_DEV_FBSIZE);
#endif
  ssd1680_redraw_display(priv);
  return OK;
}

static int ssd1680_redraw_display(struct ssd1680_dev_s *priv)
{
  /* Step 15:
   * Set control register 2.
   * 1 byte of data with following bits:
   *   0x80 - enable clock signal
   *   0x40 - enable analog
   *   0x20 - load temperature value
   *   0x10 - Load LUT
   *   0x08 - Display Mode 2
   *   0x04 - disable OSC
   *   0x02 - disable analog
   *   0x01 - disable clock signal
   */

  ssd1680_snd_cmd_with_data1(priv, SSD1680_DISP_CTRL2, 0xc4);

  /* Step 16: */

  ssd1680_snd_cmd_with_data0(priv, SSD1680_MASTER_ACTIVATE);
  ssd1680_busy_wait(priv);

  /* Step 18: */

  ssd1680_snd_cmd_with_data1(priv, SSD1680_DISP_CTRL2, 0xc3);

  /* Step 19: */

  ssd1680_snd_cmd_with_data0(priv, SSD1680_MASTER_ACTIVATE);
  ssd1680_busy_wait(priv);

  return OK;
}

static int ssd1680_redrawfb_row(struct ssd1680_dev_s *priv, int row)
{
#if defined(CONFIG_LCD_PORTRAIT) || defined(CONFIG_LCD_RPORTRAIT)
  uint8_t *src = priv->shadow_fb + row * SSD1680_DEV_ROWSIZE;

  ssd1680_snd_cmd_with_data1(priv, SSD1680_SET_RAMXCOUNT, 0);
  ssd1680_snd_cmd_with_data2(priv, SSD1680_SET_RAMYCOUNT, row, row >> 8);

#if SSD1680_DEV_BPP == 1
  ssd1680_snd_cmd_with_data(priv, SSD1680_WRITE_RAM1, src,
      SSD1680_DEV_ROWSIZE);
#else
  ssd1680_snd_cmd_with_data_even_bits(priv, SSD1680_WRITE_RAM1, src,
      SSD1680_DEV_ROWSIZE);

  ssd1680_snd_cmd_with_data1(priv, SSD1680_SET_RAMXCOUNT, 1);
  ssd1680_snd_cmd_with_data2(priv, SSD1680_SET_RAMYCOUNT,
      row, row >> 8);
  ssd1680_snd_cmd_with_data_odd_bits(priv,
    SSD1680_WRITE_RAM2, src, SSD1680_DEV_FBSIZE);
#endif

  if (row == SSD1680_DEV_FB_YRES - 1)
    {
      ssd1680_redraw_display(priv);
    }
#else
  int row_group = (row >> 3) << 3;
  uint8_t *src = priv->shadow_fb + row_group * SSD1680_DEV_ROWSIZE;

  ssd1680_snd_cmd_with_data1(priv, SSD1680_SET_RAMXCOUNT, row >> 3);
  ssd1680_snd_cmd_with_data2(priv, SSD1680_SET_RAMYCOUNT, 0, 0);

#if SSD1680_DEV_BPP == 1
  ssd1680_snd_cmd_with_data_bitstrip(priv, SSD1680_WRITE_RAM1, src,
      SSD1680_DEV_NATIVE_YRES, SSD1680_DEV_ROWSIZE);
#else
#error "Landscape mode with 3 colors is not implemented"
  /* TODO send ssd1680_snd_cmd_with_data_even_bits_bitstrip
   * (priv, SSD1680_WRITE_RAM1, src, SSD1680_DEV_NATIVE_YRES,
   *  SSD1680_DEV_ROWSIZE);
   */

  ssd1680_snd_cmd_with_data1(priv, SSD1680_SET_RAMXCOUNT, row >> 3);
  ssd1680_snd_cmd_with_data2(priv, SSD1680_SET_RAMYCOUNT, 0, 0);

  /*  TODO send ssd1680_snd_cmd_with_data_odd_bits_bitstrip(priv,
   *  SSD1680_WRITE_RAM2, src, SSD1680_DEV_NATIVE_YRESSSD1680_DEV_ROWSIZE, );
   */

#endif

  if (row == SSD1680_DEV_FB_YRES - 1)
    {
      ssd1680_redraw_display(priv);
    }
#endif

  return OK;
}

/****************************************************************************
 * Name: ssd1680_configspi
 *
 * Description:
 *
 ****************************************************************************/

static void ssd1680_configspi(FAR struct spi_dev_s *spi)
{
  /* Configure SPI for the SSD1680 */

  SPI_SETMODE(spi, CONFIG_SSD1680_SPIMODE);
  SPI_SETBITS(spi, 8);
  SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, CONFIG_SSD1680_FREQUENCY);
}

/****************************************************************************
 * Name: ssd1680_select
 *
 * Description:
 *   Enable/Disable SSD1680 SPI CS
 *
 ****************************************************************************/

static void ssd1680_select(FAR struct ssd1680_dev_s *priv, bool cs)
{
  /* If we are selecting the device */

  if (cs == true)
    {
      /* If SPI bus is shared then lock and configure it */

      SPI_LOCK(priv->spi, true);
      ssd1680_configspi(priv->spi);
    }

  /* Select/deselect SPI device */

  SPI_SELECT(priv->spi, SPIDEV_DISPLAY(0), cs);

  /* If we are deselecting the device */

  if (cs == false)
    {
      /* Unlock bus */

      SPI_LOCK(priv->spi, false);
    }
}

static void ssd1680_busy_wait(FAR struct ssd1680_dev_s *priv)
{
  if ((priv->board_priv != NULL) && (priv->board_priv->check_busy != NULL))
    {
      while (priv->board_priv->check_busy())
        {
          nxsig_usleep(1);
        }
    }
  else
    {
      nxsig_usleep(2000000);
    }
}

static void ssd1680_snd_cmd_with_data0(FAR struct ssd1680_dev_s *priv,
  uint8_t cmd)
{
  ssd1680_select(priv, true);
  ssd1680_cmddata(priv, true);
  SPI_SEND(priv->spi, cmd);
  ssd1680_select(priv, false);
}

static void ssd1680_snd_cmd_with_data1(FAR struct ssd1680_dev_s *priv,
    uint8_t cmd, uint8_t dta1)
{
  ssd1680_select(priv, true);
  ssd1680_cmddata(priv, true);
  SPI_SEND(priv->spi, cmd);
  ssd1680_cmddata(priv, false);
  SPI_SEND(priv->spi, dta1);
  ssd1680_select(priv, false);
}

static void ssd1680_snd_cmd_with_data2(FAR struct ssd1680_dev_s *priv,
    uint8_t cmd, uint8_t dta1, uint8_t dta2)
{
  ssd1680_select(priv, true);
  ssd1680_cmddata(priv, true);
  SPI_SEND(priv->spi, cmd);
  ssd1680_cmddata(priv, false);
  SPI_SEND(priv->spi, dta1);
  SPI_SEND(priv->spi, dta2);
  ssd1680_select(priv, false);
}

static void ssd1680_snd_cmd_with_data3(FAR struct ssd1680_dev_s *priv,
    uint8_t cmd, uint8_t dta1, uint8_t dta2, uint8_t dta3)
{
  ssd1680_select(priv, true);
  ssd1680_cmddata(priv, true);
  SPI_SEND(priv->spi, cmd);
  ssd1680_cmddata(priv, false);
  SPI_SEND(priv->spi, dta1);
  SPI_SEND(priv->spi, dta2);
  SPI_SEND(priv->spi, dta3);
  ssd1680_select(priv, false);
}

static void ssd1680_snd_cmd_with_data4(FAR struct ssd1680_dev_s *priv,
    uint8_t cmd, uint8_t dta1, uint8_t dta2, uint8_t dta3, uint8_t dta4)
{
  ssd1680_select(priv, true);
  ssd1680_cmddata(priv, true);
  SPI_SEND(priv->spi, cmd);
  ssd1680_cmddata(priv, false);
  SPI_SEND(priv->spi, dta1);
  SPI_SEND(priv->spi, dta2);
  SPI_SEND(priv->spi, dta3);
  SPI_SEND(priv->spi, dta4);
  ssd1680_select(priv, false);
}

static void ssd1680_snd_cmd_with_data(FAR struct ssd1680_dev_s *priv,
    uint8_t cmd, const uint8_t *dta, int dta_len)
{
  ssd1680_select(priv, true);
  ssd1680_cmddata(priv, true);
  SPI_SEND(priv->spi, cmd);
  ssd1680_cmddata(priv, false);
  SPI_SNDBLOCK(priv->spi, dta, dta_len);
  ssd1680_select(priv, false);
}

#if !defined(CONFIG_LCD_PORTRAIT) && !defined(CONFIG_LCD_RPORTRAIT)

#if SSD1680_DEV_BPP == 1
static void ssd1680_snd_cmd_with_data_bitstrip(
    FAR struct ssd1680_dev_s *priv, uint8_t cmd, const uint8_t *dta,
    int nopix, int strip_len)
{
  int i;
  int j;
  uint8_t bytes[8];
  uint8_t val;

#if defined(CONFIG_LCD_LANDSCAPE)
  dta += (strip_len - 1);
#endif

  ssd1680_select(priv, true);
  ssd1680_cmddata(priv, true);
  SPI_SEND(priv->spi, cmd);
  ssd1680_cmddata(priv, false);
  while (nopix > 0)
    {
      for (j = 0; j < 8; j++)
        {
          bytes[j] = *(dta + j * strip_len);
        }

      for (i = 0; i < 8; i++)
        {
          val = 0;
          for (j = 0; j < 8; j++)
            {
#if defined(CONFIG_LCD_LANDSCAPE)
              val |= ((bytes[j] << (7 - j)) & (1 << (7 - j)));
              bytes[j] = bytes[j] >> 1;
#elif
              val |= ((bytes[j] >> j) & (1 << (7 - j)));
              bytes[j] = bytes[j] << 1;
#endif
            }

          SPI_SEND(priv->spi, val);
          nopix--;
          if (nopix == 0)
            {
              break;
            }
        }

#if defined(CONFIG_LCD_LANDSCAPE)
      dta--;
#elif
      dta++;
#endif
    }

  ssd1680_select(priv, false);
}
#else
static void ssd1680_snd_cmd_with_data_even_bits_bitstrip(
    FAR struct ssd1680_dev_s *priv, uint8_t cmd, const uint8_t *dta,
    int nopix, int strip_len)
{
  int i;
  int j;
  uint8_t rows[8];
  uint16_t tmp[8];
  uint8_t val;

#if defined(CONFIG_LCD_LANDSCAPE)
  dta += (strip_len - 1);
#endif

  ssd1680_select(priv, true);
  ssd1680_cmddata(priv, true);
  SPI_SEND(priv->spi, cmd);
  ssd1680_cmddata(priv, false);
  while (nopix > 0)
    {
      for (j = 0; j < 8; j++)
        {
#if defined(CONFIG_LCD_LANDSCAPE)
          tmp = *(dta + j * strip_len)
              + ((*(dta + j * strip_len + 1)) << 8);
#else
          tmp = *(dta + j * strip_len + 1)
              + ((*(dta + j * strip_len)) << 8);
#endif
          rows[j] =
                 (tmp & 0x01)       | ((tmp >> 1) & 0x02)
              | ((tmp >> 2) & 0x04) | ((tmp >> 3) & 0x08)
              | ((tmp >> 4) & 0x10) | ((tmp >> 5) & 0x20)
              | ((tmp >> 6) & 0x40) | ((tmp >> 7) & 0x80);
        }

      for (i = 0; i < 8; i++)
        {
          val = 0;
          for (j = 0; j < 8; j++)
            {
#if defined(CONFIG_LCD_LANDSCAPE)
              val |= ((rows[j] << (7 - j)) & (1 << (7 - j)));
              rows[j] = rows[j] >> 1;
#elif
              val |= ((rows[j] >> (j)) & (1 << (7 - j)));
              rows[j] = rows[j] << 1;
#endif
            }

          SPI_SEND(priv->spi, val);
          nopix--;
          if (nopix == 0)
            {
              break;
            }
        }

#if defined(CONFIG_LCD_LANDSCAPE)
      dta--;
#elif
      dta++;
#endif
    }

  ssd1680_select(priv, false);
}

static void ssd1680_snd_cmd_with_data_odd_bits_bitstrip(
    FAR struct ssd1680_dev_s *priv, uint8_t cmd, const uint8_t *dta,
    int nopix, int strip_len)
{
  int i;
  int j;
  uint16_t rows[8];
  uint8_t val;

#if defined(CONFIG_LCD_LANDSCAPE)
  dta += (strip_len - 1);
#endif

  ssd1680_select(priv, true);
  ssd1680_cmddata(priv, true);
  SPI_SEND(priv->spi, cmd);
  ssd1680_cmddata(priv, false);
  while (nopix > 0)
    {
      for (j = 0; j < 8; j++)
        {
#if defined(CONFIG_LCD_LANDSCAPE)
          rows[j] = *(dta + j * strip_len)
              + ((*(dta + j * strip_len + 1)) << 8);
#else
          rows[j] = *(dta + j * strip_len + 1)
              + ((*(dta + j * strip_len)) << 8);
#endif
        }

      for (i = 0; i < 8; i++)
        {
          val = 0;
          for (j = 0; j < 8; j++)
            {
#if defined(CONFIG_LCD_LANDSCAPE)
              val |= ((rows[j] << (7 - j)) & (1 << (7 - j)));
              rows[j] = rows[j] >> 2;
#elif
              val |= ((rows[j] >> (j)) & (1 << (7 - j)));
              rows[j] = rows[j] << 2;
#endif
            }

          SPI_SEND(priv->spi, val);
          nopix--;
          if (nopix == 0)
            {
              break;
            }
        }

#if defined(CONFIG_LCD_LANDSCAPE)
      dta--;
#elif
      dta++;
#endif
    }

  ssd1680_select(priv, false);
}
#endif
#endif

#if SSD1680_DEV_BPP == 2
static void ssd1680_snd_cmd_with_data_even_bits(
    FAR struct ssd1680_dev_s *priv, uint8_t cmd, const uint8_t *dta,
    int dta_len)
{
  int i;
  uint8_t dta_byte;
  uint8_t src1;
  uint8_t src2;
  ssd1680_select(priv, true);
  ssd1680_cmddata(priv, true);
  SPI_SEND(priv->spi, cmd);
  ssd1680_cmddata(priv, false);

  for (i = 0; i < dta_len; i++)
    {
      src1 = *(dta++);
      src2 = *(dta++);
      dta_byte = (src1 & 0x01) | ((src1 >> 1) & 0x02)
               | ((src1 >> 2) & 0x04) | ((src1 >> 3) & 0x08)
               | ((src2 << 4) & 0x10) | ((src2 << 3) & 0x20)
               | ((src2 << 2) & 0x40) | ((src2 << 1) & 0x80);
      SPI_SEND(priv->spi, dta_byte);
    }

  ssd1680_select(priv, false);
}

static void ssd1680_snd_cmd_with_data_odd_bits(
    FAR struct ssd1680_dev_s *priv, uint8_t cmd, const uint8_t *dta,
    int dta_len)
{
  int i;
  uint8_t dta_byte;
  uint8_t src1;
  uint8_t src2;
  ssd1680_select(priv, true);
  ssd1680_cmddata(priv, true);
  SPI_SEND(priv->spi, cmd);
  ssd1680_cmddata(priv, false);

  for (i = 0; i < dta_len; i++)
    {
      src1 = *(dta++);
      src2 = *(dta++);
      dta_byte = ((src1 >> 1 & 0x01)) | ((src1 >> 2) & 0x02)
               | ((src1 >> 3) & 0x04) | ((src1 >> 4) & 0x08)
               | ((src2 << 3) & 0x10) | ((src2 << 2) & 0x20)
               | ((src2 << 1) & 0x40) | (src2 & 0x80);
      SPI_SEND(priv->spi, dta_byte);
    }

  ssd1680_select(priv, false);
}
#endif

/****************************************************************************
 * Name: ssd1680_cmddata
 *
 * Description:
 *   Select Command/Data mode for SSD1680
 *
 ****************************************************************************/

inline static void ssd1680_cmddata(FAR struct ssd1680_dev_s *priv, bool cmd)
{
  SPI_CMDDATA(priv->spi, SPIDEV_DISPLAY(0), cmd);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  ssd1680_initialize
 *
 * Description:
 *   Initialize the video hardware.  The initial state of the OLED is
 *   fully initialized, display memory cleared, and the OLED ready
 *   to use, but with the power setting at 0 (full off == sleep mode).
 *
 * Input Parameters:
 *
 *   dev - A reference to the SPI driver instance.
 *   board_priv - Board specific structure.
 *
 * Returned Value:
 *
 *   On success, this function returns a reference to the LCD object for
 *   the specified OLED.  NULL is returned on any failure.
 *
 ****************************************************************************/

FAR struct lcd_dev_s *ssd1680_initialize(FAR struct spi_dev_s *dev,
                          FAR const struct ssd1680_priv_s *board_priv)
{
  FAR struct ssd1680_dev_s *priv = &g_epaperdev;
  DEBUGASSERT(dev);

  priv->dev = g_lcd_epaper_dev;
  priv->on = false;
  priv->is_conf = false;

  /* Register board specific functions */

  priv->board_priv = board_priv;
  priv->spi = dev;

  /* Configure the SPI */

  ssd1680_configspi(priv->spi);

  /* Initialize the framebuffer */

  memset(priv->shadow_fb, SSD1680_Y1_BLACK & 1 ?
    0xff : 0x00, SSD1680_DEV_FBSIZE);

  /* Power on and configure display */

  ssd1680_setpower(&priv->dev, true);

  return &priv->dev;
}

FAR void *bitscpy_ds(FAR void *dest, int dest_offset, FAR const void *src,
    size_t nbits)
{
  FAR unsigned char *pout = (FAR unsigned char *)dest;
  FAR unsigned char *pin  = (FAR unsigned char *)src;
  uint8_t val;

  /* Copy block of bytes */

  while (nbits >= 8)
    {
      /* Read. MSB is first */

      val = *pin++;

      /* Write */

      if (dest_offset == 0)
        {
          *pout++ = val;
        }
      else
        {
          *pout &= (~(0xff >> dest_offset));
          *pout |= (val >> dest_offset);
          pout++;
          *pout &= ~(0xff << (8 - dest_offset));
          *pout |= (val << (8 - dest_offset));
        }

      nbits -= 8;
    }

  /* Copy last bits 1-7 */

  if (nbits > 0)
    {
      val = *pin;

      if (nbits + dest_offset <= 8)
        {
          *pout &= (~((0xff << (8 - nbits)) >> dest_offset));
          *pout |= (val & (0xff << (8 - nbits)) >> dest_offset);
        }
      else
        {
          *pout &= (~(0xff >> dest_offset));
          *pout |= (val >> dest_offset);
          pout++;

          nbits -= (8 - dest_offset);
          *pout &= ~(0xff << (8 - nbits));
          *pout |= ((val << (8 - dest_offset)) & (0xff << (8 - nbits)));
        }
    }

  return dest;
}

FAR void *bitscpy_ss(FAR void *dest, FAR const void *src, int src_offset,
    size_t nbits)
{
  FAR unsigned char *pout = (FAR unsigned char *)dest;
  FAR unsigned char *pin  = (FAR unsigned char *)src;
  uint8_t val;

  /* Copy block of bytes */

  while (nbits >= 8)
    {
      /* Read. MSB is first */

      if (src_offset == 0)
        {
          val = *pin++;
        }
      else
        {
          val = ((*pin) << src_offset);
          pin++;
          val |= ((*pin) >> (8 - src_offset));
        }

      /* Write */

      *pout++ = val;
      nbits -= 8;
    }

  /* Copy last bits 1-7 */

  if (nbits > 0)
    {
      val = ((*pin) << src_offset);
      if (nbits + src_offset > 8)
        {
          pin++;
          val |= ((*pin & (~(0xff >> (nbits + src_offset - 8))
              >> (8 - src_offset))));
        }

      *pout &= (~((0xff << (8 - nbits))));
      *pout |= (val & (0xff << (8 - nbits)));
    }

  return dest;
}

#endif /* CONFIG_LCD_SSD1680 */
