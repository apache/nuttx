/****************************************************************************
 * drivers/lcd/co5300.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <sys/param.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <endian.h>
#include <assert.h>
#include <errno.h>
#include <nuttx/debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/spi/qspi.h>
#include <nuttx/video/fb.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/co5300.h>

#ifdef CONFIG_LCD_CO5300

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_LCD_CO5300_FREQUENCY
#  define CONFIG_LCD_CO5300_FREQUENCY 40000000
#endif

#ifndef CONFIG_LCD_CO5300_BUFSIZE
#  define CONFIG_LCD_CO5300_BUFSIZE 2048
#endif

#if CONFIG_LCD_CO5300_BUFSIZE < 2 || (CONFIG_LCD_CO5300_BUFSIZE & 1) != 0
#  error "CONFIG_LCD_CO5300_BUFSIZE must be even and non-zero"
#endif

#ifndef CONFIG_LCD_CO5300_BRIGHTNESS
#  define CONFIG_LCD_CO5300_BRIGHTNESS 255
#endif

#if !defined(CONFIG_LCD_MAXPOWER) || CONFIG_LCD_MAXPOWER < 1
#  undef CONFIG_LCD_MAXPOWER
#  define CONFIG_LCD_MAXPOWER 1
#endif

#if !defined(CONFIG_LCD_MAXCONTRAST) || CONFIG_LCD_MAXCONTRAST < 1
#  undef CONFIG_LCD_MAXCONTRAST
#  define CONFIG_LCD_MAXCONTRAST 255
#endif

#define CO5300_XRES          CONFIG_LCD_CO5300_XRES
#define CO5300_YRES          CONFIG_LCD_CO5300_YRES
#define CO5300_XOFFSET       CONFIG_LCD_CO5300_XOFFSET
#define CO5300_YOFFSET       CONFIG_LCD_CO5300_YOFFSET

/* Only RGB565 is supported */

#define CO5300_BPP           16
#define CO5300_COLORFMT      FB_FMT_RGB16_565
#define CO5300_STRIDE        (CO5300_XRES * 2)

/* Pixels per QSPI memory transfer */

#define CO5300_TXPIXELS      (CONFIG_LCD_CO5300_BUFSIZE / 2)

/* Window start and size must be multiples of this */

#define CO5300_ALIGN         2

/* QSPI opcodes: the 24-bit address field carries the DCS command in
 * bits 15-8.
 */

#define CO5300_OP_WRREG      0x02  /* 1-1-1 register write */
#define CO5300_OP_WRPIXEL    0x32  /* 1-1-4 pixel write */

/* DCS and CO5300 commands */

#define CO5300_SLPIN         0x10  /* Sleep in */
#define CO5300_SLPOUT        0x11  /* Sleep out */
#define CO5300_INVOFF        0x20  /* Display inversion off */
#define CO5300_DISPOFF       0x28  /* Display off */
#define CO5300_DISPON        0x29  /* Display on */
#define CO5300_CASET         0x2a  /* Column address set */
#define CO5300_RASET         0x2b  /* Row address set */
#define CO5300_RAMWR         0x2c  /* Memory write */
#define CO5300_MADCTL        0x36  /* Memory data access control */
#define CO5300_COLMOD        0x3a  /* Interface pixel format */
#define CO5300_RAMWRC        0x3c  /* Memory write continue */
#define CO5300_WRDISBV       0x51  /* Display brightness */
#define CO5300_WRCTRLD       0x53  /* Control display */
#define CO5300_WRCE          0x58  /* Contrast enhancement */
#define CO5300_WRHBMDISBV    0x63  /* HBM display brightness */
#define CO5300_SPIMODECTL    0xc4  /* SPI mode control */
#define CO5300_CMDPAGE       0xfe  /* Command page select */

#define CO5300_COLMOD_RGB565 0x55
#define CO5300_WRCTRLD_BCTRL 0x20  /* Brightness control block on */
#define CO5300_SPIMODE_QSPI  0x80
#define CO5300_CMDPAGE_USER  0x00

#define CO5300_MADCTL_MX     0x02  /* Mirror X */
#define CO5300_MADCTL_MY     0x05  /* Mirror Y */
#define CO5300_MADCTL_BGR    0x08  /* BGR color order */

#ifdef CONFIG_LCD_CO5300_MIRRORX
#  define CO5300_MADCTL_X    CO5300_MADCTL_MX
#else
#  define CO5300_MADCTL_X    0
#endif

#ifdef CONFIG_LCD_CO5300_MIRRORY
#  define CO5300_MADCTL_Y    CO5300_MADCTL_MY
#else
#  define CO5300_MADCTL_Y    0
#endif

#ifdef CONFIG_LCD_CO5300_BGR
#  define CO5300_MADCTL_RGB  CO5300_MADCTL_BGR
#else
#  define CO5300_MADCTL_RGB  0
#endif

#define CO5300_MADCTL_VALUE  (CO5300_MADCTL_X | CO5300_MADCTL_Y | \
                              CO5300_MADCTL_RGB)

#define CO5300_SLEEP_DELAY   120000  /* us after SLPIN/SLPOUT */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct co5300_dev_s
{
  struct lcd_dev_s dev;           /* Publicly visible device structure */
  FAR struct qspi_dev_s *qspi;    /* QSPI bus */
  FAR uint16_t *txbuf;            /* DMA-capable transfer buffer */
#ifdef CONFIG_LCD_CO5300_SHADOWBUF
  FAR uint16_t *shadow;           /* Copy of display RAM, wire byte order */
#endif
  uint8_t power;                  /* Current power setting */
  uint8_t brightness;             /* Current DCS brightness (0-255) */
  uint16_t runbuffer[CO5300_XRES];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* LCD Data Transfer Methods */

static int co5300_putrun(FAR struct lcd_dev_s *dev, fb_coord_t row,
                         fb_coord_t col, FAR const uint8_t *buffer,
                         size_t npixels);
static int co5300_putarea(FAR struct lcd_dev_s *dev, fb_coord_t row_start,
                          fb_coord_t row_end, fb_coord_t col_start,
                          fb_coord_t col_end, FAR const uint8_t *buffer,
                          fb_coord_t stride);
static int co5300_getrun(FAR struct lcd_dev_s *dev, fb_coord_t row,
                         fb_coord_t col, FAR uint8_t *buffer,
                         size_t npixels);

/* LCD Configuration */

static int co5300_getvideoinfo(FAR struct lcd_dev_s *dev,
                               FAR struct fb_videoinfo_s *vinfo);
static int co5300_getplaneinfo(FAR struct lcd_dev_s *dev,
                               unsigned int planeno,
                               FAR struct lcd_planeinfo_s *pinfo);

/* LCD Specific Controls */

static int co5300_getpower(FAR struct lcd_dev_s *dev);
static int co5300_setpower(FAR struct lcd_dev_s *dev, int power);
static int co5300_getcontrast(FAR struct lcd_dev_s *dev);
static int co5300_setcontrast(FAR struct lcd_dev_s *dev,
                              unsigned int contrast);
static int co5300_getareaalign(FAR struct lcd_dev_s *dev,
                               FAR struct lcddev_area_align_s *align);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Register writes (command, parameter) sent after sleep out */

static const uint8_t g_co5300_initcmds[][2] =
{
  { CO5300_CMDPAGE,    CO5300_CMDPAGE_USER  },
  { CO5300_SPIMODECTL, CO5300_SPIMODE_QSPI  },
  { CO5300_COLMOD,     CO5300_COLMOD_RGB565 },
  { CO5300_WRCTRLD,    CO5300_WRCTRLD_BCTRL },
  { CO5300_WRHBMDISBV, 0xff                 },
  { CO5300_MADCTL,     CO5300_MADCTL_VALUE  },
  { CO5300_WRCE,       0x00                 },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: co5300_lock
 *
 * Description:
 *   Lock and configure the QSPI bus.
 *
 ****************************************************************************/

static void co5300_lock(FAR struct co5300_dev_s *priv)
{
  QSPI_LOCK(priv->qspi, true);
  QSPI_SETMODE(priv->qspi, QSPIDEV_MODE0);
  QSPI_SETBITS(priv->qspi, 8);
  QSPI_SETFREQUENCY(priv->qspi, CONFIG_LCD_CO5300_FREQUENCY);
}

/****************************************************************************
 * Name: co5300_unlock
 ****************************************************************************/

static void co5300_unlock(FAR struct co5300_dev_s *priv)
{
  QSPI_LOCK(priv->qspi, false);
}

/****************************************************************************
 * Name: co5300_wrreg
 *
 * Description:
 *   Send a command with up to 4 parameter bytes on a single line.
 *
 ****************************************************************************/

static int co5300_wrreg(FAR struct co5300_dev_s *priv, uint8_t cmd,
                        FAR const uint8_t *param, uint8_t nparam)
{
  struct qspi_cmdinfo_s cmdinfo;
  uint8_t buf[4];

  DEBUGASSERT(nparam <= sizeof(buf));

  /* Copy into a word-sized buffer: some controllers read whole words */

  memset(buf, 0, sizeof(buf));
  if (nparam > 0)
    {
      memcpy(buf, param, nparam);
    }

  memset(&cmdinfo, 0, sizeof(cmdinfo));
  cmdinfo.flags   = QSPICMD_ADDRESS;
  cmdinfo.addrlen = 3;
  cmdinfo.cmd     = CO5300_OP_WRREG;
  cmdinfo.addr    = (uint32_t)cmd << 8;

  if (nparam > 0)
    {
      cmdinfo.flags  |= QSPICMD_WRITEDATA;
      cmdinfo.buflen  = nparam;
      cmdinfo.buffer  = buf;
    }

  return QSPI_COMMAND(priv->qspi, &cmdinfo);
}

/****************************************************************************
 * Name: co5300_wrreg8
 ****************************************************************************/

static int co5300_wrreg8(FAR struct co5300_dev_s *priv, uint8_t cmd,
                         uint8_t value)
{
  return co5300_wrreg(priv, cmd, &value, 1);
}

/****************************************************************************
 * Name: co5300_setwindow
 *
 * Description:
 *   Set the display RAM window (inclusive, panel coordinates).
 *
 ****************************************************************************/

static int co5300_setwindow(FAR struct co5300_dev_s *priv,
                            fb_coord_t row_start, fb_coord_t row_end,
                            fb_coord_t col_start, fb_coord_t col_end)
{
  uint8_t param[4];
  uint16_t start;
  uint16_t end;
  int ret;

  start    = col_start + CO5300_XOFFSET;
  end      = col_end + CO5300_XOFFSET;
  param[0] = start >> 8;
  param[1] = start & 0xff;
  param[2] = end >> 8;
  param[3] = end & 0xff;

  ret = co5300_wrreg(priv, CO5300_CASET, param, 4);
  if (ret < 0)
    {
      return ret;
    }

  start    = row_start + CO5300_YOFFSET;
  end      = row_end + CO5300_YOFFSET;
  param[0] = start >> 8;
  param[1] = start & 0xff;
  param[2] = end >> 8;
  param[3] = end & 0xff;

  return co5300_wrreg(priv, CO5300_RASET, param, 4);
}

/****************************************************************************
 * Name: co5300_wrpixels
 *
 * Description:
 *   Send npixels from the transfer buffer on four data lines.  The first
 *   transfer of a window uses RAMWR, the following ones RAMWRC, so that
 *   CS may be released between transfers.
 *
 ****************************************************************************/

static int co5300_wrpixels(FAR struct co5300_dev_s *priv, size_t npixels,
                           bool first)
{
  struct qspi_meminfo_s meminfo;

  memset(&meminfo, 0, sizeof(meminfo));
  meminfo.flags   = QSPIMEM_WRITE | QSPIMEM_QUADDATA;
  meminfo.addrlen = 3;
  meminfo.cmd     = CO5300_OP_WRPIXEL;
  meminfo.addr    = (uint32_t)(first ? CO5300_RAMWR : CO5300_RAMWRC) << 8;
  meminfo.buflen  = npixels * 2;
  meminfo.buffer  = priv->txbuf;

  return QSPI_MEMORY(priv->qspi, &meminfo);
}

/****************************************************************************
 * Name: co5300_wrarea
 *
 * Description:
 *   Set the window and stream the pixels of an area through the transfer
 *   buffer.  If swap is true, source pixels are in CPU byte order and are
 *   converted to the big-endian wire order.  Bus must be locked.
 *
 ****************************************************************************/

static int co5300_wrarea(FAR struct co5300_dev_s *priv,
                         fb_coord_t row_start, fb_coord_t row_end,
                         fb_coord_t col_start, fb_coord_t col_end,
                         FAR const uint8_t *buffer, size_t stride,
                         bool swap)
{
  FAR const uint16_t *src;
  size_t width = col_end - col_start + 1;
  size_t fill = 0;
  bool first = true;
  fb_coord_t row;
  size_t i;
  int ret;

  ret = co5300_setwindow(priv, row_start, row_end, col_start, col_end);
  if (ret < 0)
    {
      return ret;
    }

  for (row = row_start; row <= row_end; row++)
    {
      src = (FAR const uint16_t *)buffer;

      for (i = 0; i < width; i++)
        {
          priv->txbuf[fill++] = swap ? htobe16(src[i]) : src[i];
          if (fill == CO5300_TXPIXELS)
            {
              ret = co5300_wrpixels(priv, fill, first);
              if (ret < 0)
                {
                  return ret;
                }

              first = false;
              fill  = 0;
            }
        }

      buffer += stride;
    }

  if (fill > 0)
    {
      ret = co5300_wrpixels(priv, fill, first);
    }

  return ret;
}

/****************************************************************************
 * Name: co5300_fill
 *
 * Description:
 *   Fill the whole display RAM with one color.  Bus must be locked.
 *
 ****************************************************************************/

static int co5300_fill(FAR struct co5300_dev_s *priv, uint16_t color)
{
  size_t remaining = CO5300_XRES * CO5300_YRES;
  size_t npixels;
  bool first = true;
  size_t i;
  int ret;

  ret = co5300_setwindow(priv, 0, CO5300_YRES - 1, 0, CO5300_XRES - 1);
  if (ret < 0)
    {
      return ret;
    }

  for (i = 0; i < CO5300_TXPIXELS; i++)
    {
      priv->txbuf[i] = htobe16(color);
    }

  while (remaining > 0)
    {
      npixels = remaining < CO5300_TXPIXELS ? remaining : CO5300_TXPIXELS;
      ret = co5300_wrpixels(priv, npixels, first);
      if (ret < 0)
        {
          return ret;
        }

      first      = false;
      remaining -= npixels;
    }

  return OK;
}

#ifdef CONFIG_LCD_CO5300_SHADOWBUF
/****************************************************************************
 * Name: co5300_shadow_update
 *
 * Description:
 *   Copy an area into the shadow buffer in wire byte order.
 *
 ****************************************************************************/

static void co5300_shadow_update(FAR struct co5300_dev_s *priv,
                                 fb_coord_t row_start, fb_coord_t row_end,
                                 fb_coord_t col_start, size_t width,
                                 FAR const uint8_t *buffer, size_t stride)
{
  FAR const uint16_t *src;
  FAR uint16_t *dest;
  fb_coord_t row;
  size_t i;

  for (row = row_start; row <= row_end; row++)
    {
      src  = (FAR const uint16_t *)buffer;
      dest = &priv->shadow[row * CO5300_XRES + col_start];

      for (i = 0; i < width; i++)
        {
          dest[i] = htobe16(src[i]);
        }

      buffer += stride;
    }
}
#endif

/****************************************************************************
 * Name: co5300_putarea
 *
 * Description:
 *   Write a rectangular area to the display.
 *
 *   The panel requires even window start and size.  Unaligned areas are
 *   widened from the shadow buffer when CONFIG_LCD_CO5300_SHADOWBUF is
 *   enabled; otherwise they are sent unmodified.
 *
 ****************************************************************************/

static int co5300_putarea(FAR struct lcd_dev_s *dev, fb_coord_t row_start,
                          fb_coord_t row_end, fb_coord_t col_start,
                          fb_coord_t col_end, FAR const uint8_t *buffer,
                          fb_coord_t stride)
{
  FAR struct co5300_dev_s *priv = (FAR struct co5300_dev_s *)dev;
  size_t width = col_end - col_start + 1;
  bool aligned;
  int ret;

  DEBUGASSERT(buffer != NULL && ((uintptr_t)buffer & 1) == 0);

  if (row_start > row_end || col_start > col_end ||
      row_end >= CO5300_YRES || col_end >= CO5300_XRES)
    {
      return -EINVAL;
    }

  if (stride == 0)
    {
      stride = width * 2;
    }

  lcdinfo("rows: %d-%d cols: %d-%d stride: %d\n",
          row_start, row_end, col_start, col_end, stride);

  aligned = (row_start % CO5300_ALIGN) == 0 &&
            (col_start % CO5300_ALIGN) == 0 &&
            ((row_end + 1) % CO5300_ALIGN) == 0 &&
            ((col_end + 1) % CO5300_ALIGN) == 0;

  co5300_lock(priv);

#ifdef CONFIG_LCD_CO5300_SHADOWBUF
  co5300_shadow_update(priv, row_start, row_end, col_start, width,
                       buffer, stride);

  if (!aligned)
    {
      /* Widen to the alignment and send from the shadow copy */

      row_start &= ~(CO5300_ALIGN - 1);
      col_start &= ~(CO5300_ALIGN - 1);
      row_end   |= CO5300_ALIGN - 1;
      col_end   |= CO5300_ALIGN - 1;

      if (row_end >= CO5300_YRES)
        {
          row_end = CO5300_YRES - 1;
        }

      if (col_end >= CO5300_XRES)
        {
          col_end = CO5300_XRES - 1;
        }

      ret = co5300_wrarea(priv, row_start, row_end, col_start, col_end,
                          (FAR const uint8_t *)
                          &priv->shadow[row_start * CO5300_XRES +
                                        col_start],
                          CO5300_STRIDE, false);
      co5300_unlock(priv);
      return ret;
    }
#else
  if (!aligned)
    {
      lcdinfo("Unaligned area, panel may not display it correctly\n");
    }
#endif

  ret = co5300_wrarea(priv, row_start, row_end, col_start, col_end,
                      buffer, stride, true);
  co5300_unlock(priv);
  return ret;
}

/****************************************************************************
 * Name: co5300_putrun
 *
 * Description:
 *   Write a partial raster line to the display.
 *
 ****************************************************************************/

static int co5300_putrun(FAR struct lcd_dev_s *dev, fb_coord_t row,
                         fb_coord_t col, FAR const uint8_t *buffer,
                         size_t npixels)
{
  if (npixels == 0)
    {
      return OK;
    }

  return co5300_putarea(dev, row, row, col, col + npixels - 1, buffer,
                        npixels * 2);
}

/****************************************************************************
 * Name: co5300_getrun
 *
 * Description:
 *   Read a partial raster line.  The panel cannot be read over this
 *   interface, so this is only supported with the shadow buffer.
 *
 ****************************************************************************/

static int co5300_getrun(FAR struct lcd_dev_s *dev, fb_coord_t row,
                         fb_coord_t col, FAR uint8_t *buffer,
                         size_t npixels)
{
#ifdef CONFIG_LCD_CO5300_SHADOWBUF
  FAR struct co5300_dev_s *priv = (FAR struct co5300_dev_s *)dev;
  FAR uint16_t *dest = (FAR uint16_t *)buffer;
  FAR const uint16_t *src;
  size_t i;

  if (row >= CO5300_YRES || col + npixels > CO5300_XRES)
    {
      return -EINVAL;
    }

  co5300_lock(priv);
  src = &priv->shadow[row * CO5300_XRES + col];
  for (i = 0; i < npixels; i++)
    {
      dest[i] = be16toh(src[i]);
    }

  co5300_unlock(priv);
  return OK;
#else
  return -ENOSYS;
#endif
}

/****************************************************************************
 * Name: co5300_getvideoinfo
 ****************************************************************************/

static int co5300_getvideoinfo(FAR struct lcd_dev_s *dev,
                               FAR struct fb_videoinfo_s *vinfo)
{
  DEBUGASSERT(dev != NULL && vinfo != NULL);

  vinfo->fmt     = CO5300_COLORFMT;
  vinfo->xres    = CO5300_XRES;
  vinfo->yres    = CO5300_YRES;
  vinfo->nplanes = 1;
  return OK;
}

/****************************************************************************
 * Name: co5300_getplaneinfo
 ****************************************************************************/

static int co5300_getplaneinfo(FAR struct lcd_dev_s *dev,
                               unsigned int planeno,
                               FAR struct lcd_planeinfo_s *pinfo)
{
  FAR struct co5300_dev_s *priv = (FAR struct co5300_dev_s *)dev;

  DEBUGASSERT(dev != NULL && pinfo != NULL && planeno == 0);

  pinfo->putrun  = co5300_putrun;
  pinfo->putarea = co5300_putarea;
  pinfo->getrun  = co5300_getrun;
  pinfo->buffer  = (FAR uint8_t *)priv->runbuffer;
  pinfo->bpp     = CO5300_BPP;
  pinfo->dev     = dev;
  return OK;
}

/****************************************************************************
 * Name: co5300_getpower
 ****************************************************************************/

static int co5300_getpower(FAR struct lcd_dev_s *dev)
{
  FAR struct co5300_dev_s *priv = (FAR struct co5300_dev_s *)dev;

  return priv->power;
}

/****************************************************************************
 * Name: co5300_setpower
 *
 * Description:
 *   0 puts the panel to sleep.  Any other value wakes it up and, if
 *   CONFIG_LCD_MAXPOWER > 1, also scales the brightness.
 *
 ****************************************************************************/

static int co5300_setpower(FAR struct lcd_dev_s *dev, int power)
{
  FAR struct co5300_dev_s *priv = (FAR struct co5300_dev_s *)dev;
  int ret = OK;

  if (power < 0 || power > CONFIG_LCD_MAXPOWER)
    {
      return -EINVAL;
    }

  co5300_lock(priv);

  if (power == 0)
    {
      if (priv->power > 0)
        {
          ret = co5300_wrreg(priv, CO5300_DISPOFF, NULL, 0);
          if (ret >= 0)
            {
              ret = co5300_wrreg(priv, CO5300_SLPIN, NULL, 0);
              nxsig_usleep(CO5300_SLEEP_DELAY);
            }
        }
    }
  else
    {
      if (priv->power == 0)
        {
          ret = co5300_wrreg(priv, CO5300_SLPOUT, NULL, 0);
          nxsig_usleep(CO5300_SLEEP_DELAY);
          if (ret >= 0)
            {
              ret = co5300_wrreg(priv, CO5300_DISPON, NULL, 0);
            }
        }

#if CONFIG_LCD_MAXPOWER > 1
      if (ret >= 0)
        {
          priv->brightness = power * 255 / CONFIG_LCD_MAXPOWER;
          ret = co5300_wrreg8(priv, CO5300_WRDISBV, priv->brightness);
        }
#endif
    }

  if (ret >= 0)
    {
      priv->power = power;
    }

  co5300_unlock(priv);
  return ret;
}

/****************************************************************************
 * Name: co5300_getcontrast
 *
 * Description:
 *   Return the brightness scaled to 0-CONFIG_LCD_MAXCONTRAST.
 *
 ****************************************************************************/

static int co5300_getcontrast(FAR struct lcd_dev_s *dev)
{
  FAR struct co5300_dev_s *priv = (FAR struct co5300_dev_s *)dev;

  return (priv->brightness * CONFIG_LCD_MAXCONTRAST + 127) / 255;
}

/****************************************************************************
 * Name: co5300_setcontrast
 *
 * Description:
 *   Set the brightness (DCS 0x51), contrast 0-CONFIG_LCD_MAXCONTRAST.
 *
 ****************************************************************************/

static int co5300_setcontrast(FAR struct lcd_dev_s *dev,
                              unsigned int contrast)
{
  FAR struct co5300_dev_s *priv = (FAR struct co5300_dev_s *)dev;
  int ret;

  if (contrast > CONFIG_LCD_MAXCONTRAST)
    {
      return -EINVAL;
    }

  co5300_lock(priv);
  priv->brightness = contrast * 255 / CONFIG_LCD_MAXCONTRAST;
  ret = co5300_wrreg8(priv, CO5300_WRDISBV, priv->brightness);
  co5300_unlock(priv);
  return ret;
}

/****************************************************************************
 * Name: co5300_getareaalign
 ****************************************************************************/

static int co5300_getareaalign(FAR struct lcd_dev_s *dev,
                               FAR struct lcddev_area_align_s *align)
{
  align->row_start_align = CO5300_ALIGN;
  align->height_align    = CO5300_ALIGN;
  align->col_start_align = CO5300_ALIGN;
  align->width_align     = CO5300_ALIGN;
  align->buf_align       = sizeof(uint16_t);
  return OK;
}

/****************************************************************************
 * Name: co5300_hwinit
 *
 * Description:
 *   Send the initialization sequence.  Bus must be locked.
 *
 ****************************************************************************/

static int co5300_hwinit(FAR struct co5300_dev_s *priv)
{
  size_t i;
  int ret;

  ret = co5300_wrreg(priv, CO5300_SLPOUT, NULL, 0);
  if (ret < 0)
    {
      return ret;
    }

  nxsig_usleep(CO5300_SLEEP_DELAY);

  for (i = 0; i < nitems(g_co5300_initcmds) && ret >= 0; i++)
    {
      ret = co5300_wrreg8(priv, g_co5300_initcmds[i][0],
                          g_co5300_initcmds[i][1]);
    }

  if (ret >= 0)
    {
      ret = co5300_wrreg(priv, CO5300_INVOFF, NULL, 0);
    }

  /* Clear the display RAM before turning the panel on */

  if (ret >= 0)
    {
      ret = co5300_fill(priv, 0);
    }

  if (ret >= 0)
    {
      ret = co5300_wrreg(priv, CO5300_DISPON, NULL, 0);
    }

  if (ret >= 0)
    {
      ret = co5300_wrreg8(priv, CO5300_WRDISBV, priv->brightness);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: co5300_lcdinitialize
 *
 * Description:
 *   Initialize the CO5300 AMOLED controller attached to a QSPI bus.
 *
 ****************************************************************************/

FAR struct lcd_dev_s *co5300_lcdinitialize(FAR struct qspi_dev_s *qspi)
{
  FAR struct co5300_dev_s *priv;
  int ret;

  DEBUGASSERT(qspi != NULL);

  priv = kmm_zalloc(sizeof(struct co5300_dev_s));
  if (priv == NULL)
    {
      return NULL;
    }

  priv->qspi  = qspi;
  priv->txbuf = QSPI_ALLOC(qspi, CO5300_TXPIXELS * 2);
  if (priv->txbuf == NULL)
    {
      lcderr("ERROR: Failed to allocate transfer buffer\n");
      goto errout_with_priv;
    }

#ifdef CONFIG_LCD_CO5300_SHADOWBUF
  priv->shadow = kmm_zalloc(CO5300_STRIDE * CO5300_YRES);
  if (priv->shadow == NULL)
    {
      lcderr("ERROR: Failed to allocate shadow buffer\n");
      goto errout_with_txbuf;
    }
#endif

  priv->dev.getvideoinfo = co5300_getvideoinfo;
  priv->dev.getplaneinfo = co5300_getplaneinfo;
  priv->dev.getpower     = co5300_getpower;
  priv->dev.setpower     = co5300_setpower;
  priv->dev.getcontrast  = co5300_getcontrast;
  priv->dev.setcontrast  = co5300_setcontrast;
  priv->dev.getareaalign = co5300_getareaalign;
  priv->brightness       = CONFIG_LCD_CO5300_BRIGHTNESS;

  co5300_lock(priv);
  ret = co5300_hwinit(priv);
  co5300_unlock(priv);

  if (ret < 0)
    {
      lcderr("ERROR: Panel initialization failed: %d\n", ret);
      goto errout_with_shadow;
    }

#if CONFIG_LCD_MAXPOWER > 1
  priv->power = priv->brightness * CONFIG_LCD_MAXPOWER / 255;
  if (priv->power == 0)
    {
      priv->power = 1;
    }
#else
  priv->power = 1;
#endif

  return &priv->dev;

errout_with_shadow:
#ifdef CONFIG_LCD_CO5300_SHADOWBUF
  kmm_free(priv->shadow);
errout_with_txbuf:
#endif
  QSPI_FREE(qspi, priv->txbuf);
errout_with_priv:
  kmm_free(priv);
  return NULL;
}

#endif /* CONFIG_LCD_CO5300 */
