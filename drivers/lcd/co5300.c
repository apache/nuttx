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
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <endian.h>
#include <assert.h>
#include <errno.h>
#include <nuttx/debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#ifdef CONFIG_LCD_CO5300_TXTHREAD
#  include <stdio.h>
#  include <stdlib.h>
#  include <nuttx/kthread.h>
#  include <nuttx/mutex.h>
#  include <nuttx/semaphore.h>
#endif
#include <nuttx/spi/qspi.h>
#include <nuttx/video/fb.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/co5300.h>

#ifdef CONFIG_LCD_CO5300

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if (CONFIG_LCD_CO5300_BUFSIZE & 1) != 0
#  error "CONFIG_LCD_CO5300_BUFSIZE must be even"
#endif

#if defined(CONFIG_LCD_CO5300_TXTHREAD) && \
    (CONFIG_LCD_CO5300_BUFSIZE & 3) != 0
#  error "CONFIG_LCD_CO5300_BUFSIZE must be a multiple of 4 (two halves)"
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

/* Pixels per QSPI memory transfer.  With the transmit thread, the
 * transfer buffer is split in two halves that are filled and sent in turn.
 */

#ifdef CONFIG_LCD_CO5300_TXTHREAD
#  define CO5300_NTXBUF      2
#else
#  define CO5300_NTXBUF      1
#endif

#define CO5300_TXPIXELS      (CONFIG_LCD_CO5300_BUFSIZE / 2 / CO5300_NTXBUF)

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

/* MADCTL uses the standard MIPI DCS bit layout */

#define CO5300_MADCTL_MY     0x80  /* Row address order (mirror Y) */
#define CO5300_MADCTL_MX     0x40  /* Column address order (mirror X) */
#define CO5300_MADCTL_MV     0x20  /* Row/column exchange */
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

#ifdef CONFIG_LCD_CO5300_MIRRORV
#  define CO5300_MADCTL_V    CO5300_MADCTL_MV
#else
#  define CO5300_MADCTL_V    0
#endif

#ifdef CONFIG_LCD_CO5300_BGR
#  define CO5300_MADCTL_RGB  CO5300_MADCTL_BGR
#else
#  define CO5300_MADCTL_RGB  0
#endif

#define CO5300_MADCTL_VALUE  (CO5300_MADCTL_X | CO5300_MADCTL_Y | \
                              CO5300_MADCTL_V | CO5300_MADCTL_RGB)

#define CO5300_SLEEP_DELAY   120000  /* us after SLPIN/SLPOUT */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct co5300_dev_s
{
  struct lcd_dev_s dev;           /* Publicly visible device structure */
  FAR struct qspi_dev_s *qspi;    /* QSPI bus */
  FAR uint16_t *txbuf;            /* DMA-capable transfer buffer(s) */
#ifdef CONFIG_LCD_CO5300_TXTHREAD
  mutex_t lock;                   /* Serializes the driver methods */
  sem_t txready;                  /* Number of buffers queued to send */
  sem_t txfree;                   /* Number of buffers free to fill */
  size_t txlen[CO5300_NTXBUF];    /* Pixels queued in each buffer */
  bool txfirst[CO5300_NTXBUF];    /* Buffer starts a new window */
  uint8_t txhead;                 /* Next buffer to fill */
  uint8_t txtail;                 /* Next buffer to send */
  bool busheld;                   /* The QSPI bus is locked by a method */
  bool threaded;                  /* The transmit thread is running */
  int txerr;                      /* First error of the transmit thread */
#endif
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
 * Name: co5300_buslock
 *
 * Description:
 *   Lock and configure the QSPI bus.
 *
 ****************************************************************************/

static void co5300_buslock(FAR struct co5300_dev_s *priv)
{
  QSPI_LOCK(priv->qspi, true);
  QSPI_SETMODE(priv->qspi, QSPIDEV_MODE0);
  QSPI_SETBITS(priv->qspi, 8);
  QSPI_SETFREQUENCY(priv->qspi, CONFIG_LCD_CO5300_FREQUENCY);
}

/****************************************************************************
 * Name: co5300_drain
 *
 * Description:
 *   Wait until the transmit thread has sent all queued pixels and take the
 *   bus back, so that commands are never interleaved with pixel data.
 *
 ****************************************************************************/

#ifdef CONFIG_LCD_CO5300_TXTHREAD
static void co5300_drain(FAR struct co5300_dev_s *priv)
{
  int i;

  if (priv->busheld)
    {
      return;
    }

  for (i = 0; i < CO5300_NTXBUF; i++)
    {
      nxsem_wait_uninterruptible(&priv->txfree);
    }

  for (i = 0; i < CO5300_NTXBUF; i++)
    {
      nxsem_post(&priv->txfree);
    }

  co5300_buslock(priv);
  priv->busheld = true;
}
#endif

/****************************************************************************
 * Name: co5300_lock
 *
 * Description:
 *   Serialize the driver methods and lock the QSPI bus.
 *
 ****************************************************************************/

static int co5300_lock(FAR struct co5300_dev_s *priv)
{
#ifdef CONFIG_LCD_CO5300_TXTHREAD
  int ret;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  co5300_drain(priv);
#else
  co5300_buslock(priv);
#endif

  return OK;
}

/****************************************************************************
 * Name: co5300_unlock
 ****************************************************************************/

static void co5300_unlock(FAR struct co5300_dev_s *priv)
{
#ifdef CONFIG_LCD_CO5300_TXTHREAD
  if (priv->busheld)
    {
      QSPI_LOCK(priv->qspi, false);
      priv->busheld = false;
    }

  nxmutex_unlock(&priv->lock);
#else
  QSPI_LOCK(priv->qspi, false);
#endif
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

#ifdef CONFIG_LCD_CO5300_TXTHREAD
  co5300_drain(priv);
#endif

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

static int co5300_wrpixels(FAR struct co5300_dev_s *priv,
                           FAR uint16_t *buffer, size_t npixels, bool first)
{
  struct qspi_meminfo_s meminfo;

  memset(&meminfo, 0, sizeof(meminfo));
  meminfo.flags   = QSPIMEM_WRITE | QSPIMEM_QUADDATA;
  meminfo.addrlen = 3;
  meminfo.cmd     = CO5300_OP_WRPIXEL;
  meminfo.addr    = (uint32_t)(first ? CO5300_RAMWR : CO5300_RAMWRC) << 8;
  meminfo.buflen  = npixels * 2;
  meminfo.buffer  = buffer;

  return QSPI_MEMORY(priv->qspi, &meminfo);
}

/****************************************************************************
 * Name: co5300_getbuf
 *
 * Description:
 *   Return a transfer buffer to fill with CO5300_TXPIXELS pixels.  With
 *   the transmit thread, wait until one of the buffers is free and release
 *   the bus so that the thread can send the queued ones.
 *
 ****************************************************************************/

static FAR uint16_t *co5300_getbuf(FAR struct co5300_dev_s *priv)
{
#ifdef CONFIG_LCD_CO5300_TXTHREAD
  if (!priv->threaded)
    {
      return priv->txbuf;
    }

  if (priv->busheld)
    {
      QSPI_LOCK(priv->qspi, false);
      priv->busheld = false;
    }

  nxsem_wait_uninterruptible(&priv->txfree);
  return priv->txbuf + priv->txhead * CO5300_TXPIXELS;
#else
  return priv->txbuf;
#endif
}

/****************************************************************************
 * Name: co5300_sendbuf
 *
 * Description:
 *   Send the buffer returned by co5300_getbuf().  With the transmit thread
 *   the buffer is only queued; an error is then reported by a later call.
 *
 ****************************************************************************/

static int co5300_sendbuf(FAR struct co5300_dev_s *priv,
                          FAR uint16_t *buffer, size_t npixels, bool first)
{
#ifdef CONFIG_LCD_CO5300_TXTHREAD
  irqstate_t flags;
  int ret;

  if (!priv->threaded)
    {
      return co5300_wrpixels(priv, buffer, npixels, first);
    }

  /* Fetch and clear an error of an earlier transfer */

  flags       = enter_critical_section();
  ret         = priv->txerr;
  priv->txerr = OK;
  leave_critical_section(flags);

  priv->txlen[priv->txhead]   = npixels;
  priv->txfirst[priv->txhead] = first;
  priv->txhead                = (priv->txhead + 1) % CO5300_NTXBUF;
  nxsem_post(&priv->txready);
  return ret;
#else
  return co5300_wrpixels(priv, buffer, npixels, first);
#endif
}

#ifdef CONFIG_LCD_CO5300_TXTHREAD
/****************************************************************************
 * Name: co5300_txthread
 *
 * Description:
 *   Send the queued transfer buffers to the panel, so that the caller can
 *   prepare the next buffer (and LVGL render the next area) meanwhile.
 *
 ****************************************************************************/

static int co5300_txthread(int argc, FAR char *argv[])
{
  FAR struct co5300_dev_s *priv =
    (FAR struct co5300_dev_s *)((uintptr_t)strtoul(argv[1], NULL, 16));
  int ret;

  for (; ; )
    {
      nxsem_wait_uninterruptible(&priv->txready);

      co5300_buslock(priv);
      ret = co5300_wrpixels(priv,
                            priv->txbuf + priv->txtail * CO5300_TXPIXELS,
                            priv->txlen[priv->txtail],
                            priv->txfirst[priv->txtail]);
      QSPI_LOCK(priv->qspi, false);

      if (ret < 0)
        {
          irqstate_t flags = enter_critical_section();

          lcderr("ERROR: Pixel transfer failed: %d\n", ret);
          if (priv->txerr == OK)
            {
              priv->txerr = ret;
            }

          leave_critical_section(flags);
        }

      priv->txtail = (priv->txtail + 1) % CO5300_NTXBUF;
      nxsem_post(&priv->txfree);
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: co5300_copypixels
 *
 * Description:
 *   Copy n pixels into the transfer buffer, converting them from CPU byte
 *   order to the big-endian wire order if swap is true.  Two pixels are
 *   converted per 32-bit word when both buffers are word aligned.
 *
 ****************************************************************************/

static void co5300_copypixels(FAR uint16_t *dst, FAR const uint16_t *src,
                              size_t n, bool swap)
{
  if (!swap)
    {
      memcpy(dst, src, n * sizeof(uint16_t));
      return;
    }

  if ((((uintptr_t)dst | (uintptr_t)src) & 3) == 0)
    {
      FAR uint32_t *d = (FAR uint32_t *)dst;
      FAR const uint32_t *s = (FAR const uint32_t *)src;
      size_t words = n / 2;
      size_t i;

      for (i = 0; i < words; i++)
        {
          uint32_t v = s[i];

          d[i] = ((v & 0x00ff00ff) << 8) | ((v >> 8) & 0x00ff00ff);
        }

      dst += words * 2;
      src += words * 2;
      n   -= words * 2;
    }

  while (n-- > 0)
    {
      *dst++ = htobe16(*src++);
    }
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
  FAR uint16_t *txbuf = NULL;
  size_t width = col_end - col_start + 1;
  size_t remaining;
  size_t fill = 0;
  size_t n;
  bool first = true;
  fb_coord_t row;
  int ret;

  ret = co5300_setwindow(priv, row_start, row_end, col_start, col_end);
  if (ret < 0)
    {
      return ret;
    }

  for (row = row_start; row <= row_end; row++)
    {
      src       = (FAR const uint16_t *)buffer;
      remaining = width;

      while (remaining > 0)
        {
          if (fill == 0)
            {
              txbuf = co5300_getbuf(priv);
            }

          n = CO5300_TXPIXELS - fill;
          if (n > remaining)
            {
              n = remaining;
            }

          co5300_copypixels(&txbuf[fill], src, n, swap);
          fill      += n;
          src       += n;
          remaining -= n;

          if (fill == CO5300_TXPIXELS)
            {
              ret = co5300_sendbuf(priv, txbuf, fill, first);
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
      ret = co5300_sendbuf(priv, txbuf, fill, first);
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
  FAR uint16_t *txbuf;
  size_t npixels;
  bool first = true;
  int nfilled = 0;
  size_t i;
  int ret;

  ret = co5300_setwindow(priv, 0, CO5300_YRES - 1, 0, CO5300_XRES - 1);
  if (ret < 0)
    {
      return ret;
    }

  while (remaining > 0)
    {
      /* Each of the transfer buffers needs to be filled only once */

      txbuf = co5300_getbuf(priv);
      if (nfilled < CO5300_NTXBUF)
        {
          for (i = 0; i < CO5300_TXPIXELS; i++)
            {
              txbuf[i] = htobe16(color);
            }

          nfilled++;
        }

      npixels = remaining < CO5300_TXPIXELS ? remaining : CO5300_TXPIXELS;
      ret = co5300_sendbuf(priv, txbuf, npixels, first);
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
  fb_coord_t row;

  for (row = row_start; row <= row_end; row++)
    {
      co5300_copypixels(&priv->shadow[row * CO5300_XRES + col_start],
                        (FAR const uint16_t *)buffer, width, true);
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

  ret = co5300_lock(priv);
  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_LCD_CO5300_SHADOWBUF
  /* Update the shadow copy and send the area from it, widened to the
   * alignment if needed.  The shadow is in wire byte order already.
   */

  co5300_shadow_update(priv, row_start, row_end, col_start, width,
                       buffer, stride);

  if (!aligned)
    {
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
    }

  ret = co5300_wrarea(priv, row_start, row_end, col_start, col_end,
                      (FAR const uint8_t *)
                      &priv->shadow[row_start * CO5300_XRES + col_start],
                      CO5300_STRIDE, false);
#else
  if (!aligned)
    {
      lcdinfo("Unaligned area, panel may not display it correctly\n");
    }

  ret = co5300_wrarea(priv, row_start, row_end, col_start, col_end,
                      buffer, stride, true);
#endif

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

  if (col >= CO5300_XRES || npixels > CO5300_XRES - col)
    {
      return -EINVAL;
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
  int ret;

  if (row >= CO5300_YRES || col + npixels > CO5300_XRES)
    {
      return -EINVAL;
    }

  /* Only serialize with the writers, the bus is not needed */

#ifdef CONFIG_LCD_CO5300_TXTHREAD
  ret = nxmutex_lock(&priv->lock);
#else
  ret = QSPI_LOCK(priv->qspi, true);
#endif
  if (ret < 0)
    {
      return ret;
    }

  src = &priv->shadow[row * CO5300_XRES + col];
  for (i = 0; i < npixels; i++)
    {
      dest[i] = be16toh(src[i]);
    }

#ifdef CONFIG_LCD_CO5300_TXTHREAD
  nxmutex_unlock(&priv->lock);
#else
  QSPI_LOCK(priv->qspi, false);
#endif
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

  ret = co5300_lock(priv);
  if (ret < 0)
    {
      return ret;
    }

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

  ret = co5300_lock(priv);
  if (ret < 0)
    {
      return ret;
    }

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
  priv->txbuf = QSPI_ALLOC(qspi, CO5300_TXPIXELS * 2 * CO5300_NTXBUF);
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

#ifdef CONFIG_LCD_CO5300_TXTHREAD
  nxmutex_init(&priv->lock);
  nxsem_init(&priv->txready, 0, 0);
  nxsem_init(&priv->txfree, 0, CO5300_NTXBUF);
#endif

  /* Initialize the panel synchronously, before the transmit thread runs */

  co5300_lock(priv);
  ret = co5300_hwinit(priv);
  co5300_unlock(priv);

  if (ret < 0)
    {
      lcderr("ERROR: Panel initialization failed: %d\n", ret);
      goto errout_with_sync;
    }

#ifdef CONFIG_LCD_CO5300_TXTHREAD
  {
    FAR char *argv[2];
    char arg1[32];

    snprintf(arg1, sizeof(arg1), "%" PRIxPTR, (uintptr_t)priv);
    argv[0] = arg1;
    argv[1] = NULL;

    priv->threaded = true;
    ret = kthread_create("co5300", CONFIG_LCD_CO5300_TXTHREAD_PRIORITY,
                         CONFIG_LCD_CO5300_TXTHREAD_STACKSIZE,
                         co5300_txthread, argv);
    if (ret < 0)
      {
        /* Still usable: the pixels are then sent synchronously */

        lcdwarn("WARNING: No transmit thread: %d\n", ret);
        priv->threaded = false;
      }
  }
#endif

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

errout_with_sync:
#ifdef CONFIG_LCD_CO5300_TXTHREAD
  nxsem_destroy(&priv->txfree);
  nxsem_destroy(&priv->txready);
  nxmutex_destroy(&priv->lock);
#endif

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
