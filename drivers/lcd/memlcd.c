/******************************************************************************
 * drivers/lcd/memlcd.c
 * Driver for Sharp Memory LCD.
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Librae <librae8226@gmail.com>
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
 *    without specific prior writen permission.
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
 ******************************************************************************/

/******************************************************************************
 * Included Files
 ******************************************************************************/

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
#include <nuttx/lcd/memlcd.h>

#include <arch/irq.h>

/******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

/* Configuration */

/* Cisplay resolution */

#if defined CONFIG_MEMLCD_LS013B7DH01
#  define MEMLCD_XRES        144
#  define MEMLCD_YRES        168
#elif defined CONFIG_MEMLCD_LS013B7DH03
#  define MEMLCD_XRES        128
#  define MEMLCD_YRES        128
#else
#  error "This Memory LCD model is not supported yet."
#endif

/* lcd command */

#define MEMLCD_CMD_UPDATE    (0x01)
#define MEMLCD_CMD_ALL_CLEAR (0x04)
#define MEMLCD_CONTROL_BYTES (0)

/* Dolor depth and format */

#define MEMLCD_BPP           1
#define MEMLCD_COLORFMT      FB_FMT_Y1

/* Bytes per logical row and column */

#define MEMLCD_XSTRIDE       (MEMLCD_XRES >> 3)
#define MEMLCD_YSTRIDE       (MEMLCD_YRES >> 3)

/* display memory allocation */
#define MEMLCD_FBSIZE        (MEMLCD_XSTRIDE*MEMLCD_YRES)

/* contrast setting, related to VCOM toggle frequency
 * higher frequency gives better contrast, instead, saves power
 */

#define MEMLCD_CONTRAST      24
#define MEMLCD_MAXCONTRAST   60
#define MEMLCD_MINCONTRAST   1

/* Other misc settings */

#define MEMLCD_SPI_FREQUENCY 2250000
#define MEMLCD_SPI_BITS      (-8)
#define MEMLCD_SPI_MODE      SPIDEV_MODE0

#define LS_BIT               (1 << 0)
#define MS_BIT               (1 << 7)

/* Debug */

#ifdef CONFIG_DEBUG_LCD
#  define lcddbg(format, arg...)  dbg(format, ##arg)
#  define lcdvdbg(format, arg...) vdbg(format, ##arg)
#else
#  define lcddbg(x...)
#  define lcdvdbg(x...)
#endif

/******************************************************************************
 * Private Type Definition
 ******************************************************************************/

struct memlcd_dev_s
{
  /* Publically visible device structure */

  struct lcd_dev_s dev;

  /* Private lcd-specific information follows */

  FAR struct spi_dev_s *spi;      /* Cached SPI device reference */
  FAR struct memlcd_priv_s *priv; /* Board specific structure */
  uint8_t contrast;               /* Current contrast setting */
  uint8_t power;                  /* Current power setting */

  /* The memlcds does not support reading the display memory in SPI mode.
   * Since there is 1 BPP and is byte access, it is necessary to keep a
   * shadow copy of the framebuffer. At 128x128, it amounts to 2KB.
   */

  uint8_t fb[MEMLCD_FBSIZE];
};

/******************************************************************************
 * Private Function Protototypes
 ******************************************************************************/

/* Low-level spi helpers */

static inline void memlcd_configspi(FAR struct spi_dev_s *spi);
#ifdef CONFIG_SPI_OWNBUS
static inline void memlcd_select(FAR struct spi_dev_s *spi);
static inline void memlcd_deselect(FAR struct spi_dev_s *spi);
#else
static void memlcd_select(FAR struct spi_dev_s *spi);
static void memlcd_deselect(FAR struct spi_dev_s *spi);
#endif

/* lcd data transfer methods */

static int memlcd_putrun(fb_coord_t row, fb_coord_t col,
                         FAR const uint8_t * buffer, size_t npixels);
static int memlcd_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t * buffer,
                         size_t npixels);

/* lcd configuration */

static int memlcd_getvideoinfo(FAR struct lcd_dev_s *dev,
                               FAR struct fb_videoinfo_s *vinfo);
static int memlcd_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int planeno,
                               FAR struct lcd_planeinfo_s *pinfo);

/* lcd specific controls */

static int memlcd_getpower(struct lcd_dev_s *dev);
static int memlcd_setpower(struct lcd_dev_s *dev, int power);
static int memlcd_getcontrast(struct lcd_dev_s *dev);
static int memlcd_setcontrast(struct lcd_dev_s *dev, unsigned int contrast);

/******************************************************************************
 * Private Data
 ******************************************************************************/

static uint8_t g_runbuffer[MEMLCD_BPP * MEMLCD_XRES / 8];

/* This structure describes the overall lcd video controller */

static const struct fb_videoinfo_s g_videoinfo =
{
  .fmt = MEMLCD_COLORFMT,            /* Color format: rgb16-565: rrrr rggg gggb bbbb */
  .xres = MEMLCD_XRES,               /* Horizontal resolution in pixel columns */
  .yres = MEMLCD_YRES,               /* Vertical resolution in pixel rows */
  .nplanes = 1,                      /* Number of color planes supported */
};

/* This is the standard, nuttx plane information object */

static const struct lcd_planeinfo_s g_planeinfo =
{
  .putrun = memlcd_putrun,           /* Put a run into lcd memory */
  .getrun = memlcd_getrun,           /* Get a run from lcd memory */
  .buffer = (uint8_t *) g_runbuffer, /* Run scratch buffer */
  .bpp = MEMLCD_BPP,                 /* Bits-per-pixel */
};

/* This is the oled driver instance (only a single device is supported for now) */

static struct memlcd_dev_s g_memlcddev =
{
  .dev =
  {
    /* lcd configuration */

    .getvideoinfo = memlcd_getvideoinfo,
    .getplaneinfo = memlcd_getplaneinfo,

    /* lcd specific controls */

    .getpower = memlcd_getpower,
    .setpower = memlcd_setpower,
    .getcontrast = memlcd_getcontrast,
    .setcontrast = memlcd_setcontrast,
  },
};

/******************************************************************************
 * Private Functions
 ******************************************************************************/

/******************************************************************************
 * __set_bit - Set a bit in memory
 *
 * @nr: the bit to set
 * @addr: the address to start counting from
 *
 * Unlike set_bit(), this function is non-atomic and may be reordered.
 * If it's called on the same region of memory simultaneously, the effect
 * may be that only one operation succeeds.
 *
 ******************************************************************************/

#define BIT(nr)            (1 << (nr))
#define BITS_PER_BYTE        8
#define BIT_MASK(nr)        (1 << ((nr) % BITS_PER_BYTE))
#define BIT_BYTE(nr)        ((nr) / BITS_PER_BYTE)

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

/******************************************************************************
 * Name: memlcd_configspi
 *
 * Description:
 *   Configure the SPI for use with the Sharp Memory LCD
 *
 * Input Parameters:
 *   spi  - Reference to the SPI driver structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ******************************************************************************/

static inline void memlcd_configspi(FAR struct spi_dev_s *spi)
{
#ifdef CONFIG_MEMLCD_SPI_FREQUENCY
  lcddbg("Mode: %d Bits: %d Frequency: %d\n",
         MEMLCD_SPI_MODE, MEMLCD_SPI_BITS, CONFIG_MEMLCD_SPI_FREQUENCY);
#else
  lcddbg("Mode: %d Bits: %d Frequency: %d\n",
         MEMLCD_SPI_MODE, MEMLCD_SPI_BITS, MEMLCD_SPI_FREQUENCY);
#endif

  /* Configure SPI for the Memory LCD.  But only if we own the SPI bus.
   * Otherwise, don't bother because it might change.
   */

#ifdef CONFIG_SPI_OWNBUS
  SPI_SETMODE(spi, MEMLCD_SPI_MODE);
  SPI_SETBITS(spi, MEMLCD_SPI_BITS);
#  ifdef CONFIG_MEMLCD_SPI_FREQUENCY
  SPI_SETFREQUENCY(spi, CONFIG_MEMLCD_SPI_FREQUENCY);
#  else
  SPI_SETFREQUENCY(spi, MEMLCD_SPI_FREQUENCY);
#  endif
#endif
}

/*******************************************************************************
 * Name: memlcd_select
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
 ******************************************************************************/

#ifdef CONFIG_SPI_OWNBUS
static inline void memlcd_select(FAR struct spi_dev_s *spi)
{
  /* we own the spi bus, so just select the chip */
  SPI_SELECT(spi, SPIDEV_DISPLAY, true);
}

#else
static void memlcd_select(FAR struct spi_dev_s *spi)
{
  /*
   * Select memlcd (locking the SPI bus in case there are multiple
   * devices competing for the SPI bus
   */
  SPI_LOCK(spi, true);
  SPI_SELECT(spi, SPIDEV_DISPLAY, true);

  /*
   * Now make sure that the SPI bus is configured for the memlcd (it
   * might have gotten configured for a different device while unlocked)
   */
  SPI_SETMODE(spi, MEMLCD_SPI_MODE);
  SPI_SETBITS(spi, MEMLCD_SPI_BITS);
#  ifdef CONFIG_MEMLCD_SPI_FREQUENCY
  SPI_SETFREQUENCY(spi, CONFIG_MEMLCD_SPI_FREQUENCY);
#  else
  SPI_SETFREQUENCY(spi, MEMLCD_SPI_FREQUENCY);
#  endif
}
#endif

/*******************************************************************************
 * Name: memlcd_deselect
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
 ******************************************************************************/

#ifdef CONFIG_SPI_OWNBUS
static inline void memlcd_deselect(FAR struct spi_dev_s *spi)
{
  /* we own the spi bus, so just de-select the chip */
  SPI_SELECT(spi, SPIDEV_DISPLAY, false);
}
#else
static void memlcd_deselect(FAR struct spi_dev_s *spi)
{
  /* de-select memlcd and relinquish the spi bus. */
  SPI_SELECT(spi, SPIDEV_DISPLAY, false);
  SPI_LOCK(spi, false);
}
#endif

/*******************************************************************************
 * Name:  memlcd_clear
 *
 * Description:
 *   This method can be used to clear the entire display.
 *
 * Input Parameters:
 *   mlcd   - Reference to private driver structure
 *
 * Assumptions:
 *
 ******************************************************************************/

static inline void memlcd_clear(FAR struct memlcd_dev_s *mlcd)
{
  uint16_t cmd = MEMLCD_CMD_ALL_CLEAR;
  lcddbg("Clear display\n");
  memlcd_select(mlcd->spi);
  /* XXX Ensure 2us here */
  SPI_SNDBLOCK(mlcd->spi, &cmd, 2);
  /* XXX Ensure 6us here */
  memlcd_deselect(mlcd->spi);
}

/*******************************************************************************
 * Name:  memlcd_extcominisr
 *
 * Description:
 *   This method enables/disables the polarity (VCOM) toggling behavior for
 *   the Memory LCD. Which is always used within setpower() call.
 *   Basically, the frequency shall be 1Hz~60Hz.
 *   If use hardware mode to toggle VCOM, we need to send specific command at a
 *   constant frequency to trigger the LCD intenal hardware logic.
 *   While use software mode, we set up a timer to toggle EXTCOMIN connected IO,
 *   basically, it is a hardware timer to ensure a constant frequency.
 *
 * Input Parameters:
 *   mlcd   - Reference to private driver structure
 *
 * Assumptions:
 *   Board specific logic needs to be provided to support it.
 *
 ******************************************************************************/

static int memlcd_extcominisr(int irq, FAR void *context)
{
  static bool pol = 0;
  struct memlcd_dev_s *mlcd = &g_memlcddev;
#ifdef CONFIG_MEMLCD_EXTCOMIN_MODE_HW
#  error "CONFIG_MEMLCD_EXTCOMIN_MODE_HW unsupported yet!"
  /*
   * start a worker thread, do it in bottom half?
   */
#else
  pol = !pol;
  mlcd->priv->setpolarity(pol);
#endif
  return OK;
}

/*******************************************************************************
 * Name:  memlcd_putrun
 *
 * Description:
 *   This method can be used to write a partial raster line to the LCD.
 *
 * Input Parameters:
 *   row     - Starting row to write to (range: 0 <= row < yres)
 *   col     - Starting column to write to (range: 0 <= col <= xres-npixels)
 *   buffer  - The buffer containing the run to be writen to the LCD
 *   npixels - The number of pixels to write to the LCD
 *             (range: 0 < npixels <= xres-col)
 *
 ******************************************************************************/

static int memlcd_putrun(fb_coord_t row, fb_coord_t col,
                         FAR const uint8_t * buffer, size_t npixels)
{
  FAR struct memlcd_dev_s *mlcd = (FAR struct memlcd_dev_s *)&g_memlcddev;
  uint16_t cmd;
  uint8_t *p;
  uint8_t *pfb;
  uint8_t usrmask;
  int i;

  DEBUGASSERT(buffer);
  lcdvdbg("row: %d col: %d npixels: %d\n", row, col, npixels);

#ifdef CONFIG_NX_PACKEDMSFIRST
  usrmask = MS_BIT;
#else
  usrmask = LS_BIT;
#endif

  pfb = &mlcd->fb[row * MEMLCD_XSTRIDE];
  p = pfb + (col >> 3);
  for (i = 0; i < npixels; i++)
    {
      if ((*buffer & usrmask) != 0)
        {
          __set_bit(col % 8 + i, p);
        }
      else
        {
          __clear_bit(col % 8 + i, p);
        }

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

  /* Need to adjust start row by one because Memory LCD starts counting
   * lines from 1, while the display interface starts from 0.
   */

  row++;

  memlcd_select(mlcd->spi);

  /* XXX Ensure 6us here */

  cmd = MEMLCD_CMD_UPDATE | row << 8;
  SPI_SNDBLOCK(mlcd->spi, &cmd, 2);
  SPI_SNDBLOCK(mlcd->spi, pfb, MEMLCD_YRES / 8 + MEMLCD_CONTROL_BYTES);
  cmd = 0xffff;
  SPI_SNDBLOCK(mlcd->spi, &cmd, 2);

  /* XXX Ensure 2us here */

  memlcd_deselect(mlcd->spi);

  return OK;
}

/*******************************************************************************
 * Name:  memlcd_getrun
 *
 * Description:
 *   This method can be used to read a partial raster line from the LCD.
 *
 *  row     - Starting row to read from (range: 0 <= row < yres)
 *  col     - Starting column to read read (range: 0 <= col <= xres-npixels)
 *  buffer  - The buffer in which to return the run read from the LCD
 *  npixels - The number of pixels to read from the LCD
 *            (range: 0 < npixels <= xres-col)
 *
 ******************************************************************************/

static int memlcd_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t * buffer,
                         size_t npixels)
{
  FAR struct memlcd_dev_s *mlcd = (FAR struct memlcd_dev_s *)&g_memlcddev;
  uint8_t *p;
  uint8_t *pfb;
  uint8_t usrmask;
  int i;

  DEBUGASSERT(buffer);
  lcdvdbg("row: %d col: %d npixels: %d\n", row, col, npixels);

#ifdef CONFIG_NX_PACKEDMSFIRST
  usrmask = MS_BIT;
#else
  usrmask = LS_BIT;
#endif

  pfb = &mlcd->fb[row * MEMLCD_XSTRIDE];
  p = pfb + (col >> 3);
  for (i = 0; i < npixels; i++)
    {
      if (__test_bit(col % 8 + i, p))
        {
          *buffer |= usrmask;
        }
      else
        {
          *buffer &= ~usrmask;
        }

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

  return OK;
}

/*******************************************************************************
 * Name:  memlcd_getvideoinfo
 *
 * Description:
 *   Get information about the LCD video controller configuration.
 *
 ******************************************************************************/
static int memlcd_getvideoinfo(FAR struct lcd_dev_s *dev,
                               FAR struct fb_videoinfo_s *vinfo)
{
  DEBUGASSERT(dev && vinfo);
  lcdvdbg("fmt: %d xres: %d yres: %d nplanes: %d\n",
          g_videoinfo.fmt, g_videoinfo.xres, g_videoinfo.yres,
          g_videoinfo.nplanes);
  memcpy(vinfo, &g_videoinfo, sizeof(struct fb_videoinfo_s));
  return OK;
}

/*******************************************************************************
 * Name:  memlcd_getplaneinfo
 *
 * Description:
 *   Get information about the configuration of each LCD color plane.
 *
 ******************************************************************************/

static int memlcd_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int planeno,
                               FAR struct lcd_planeinfo_s *pinfo)
{
  DEBUGASSERT(pinfo && planeno == 0);
  lcdvdbg("planeno: %d bpp: %d\n", planeno, g_planeinfo.bpp);
  memcpy(pinfo, &g_planeinfo, sizeof(struct lcd_planeinfo_s));
  return OK;
}

/*******************************************************************************
 * Name:  memlcd_getpower
 *
 * Description:
 *   Get the LCD panel power status (0: full off - CONFIG_LCD_MAXPOWER: full on.
 *   On backlit LCDs, this setting may correspond to the backlight setting.
 *
 ******************************************************************************/

static int memlcd_getpower(FAR struct lcd_dev_s *dev)
{
  FAR struct memlcd_dev_s *mlcd = (FAR struct memlcd_dev_s *)dev;
  DEBUGASSERT(mlcd);
  lcddbg("%d\n", mlcd->power);
  return mlcd->power;
}

/*******************************************************************************
 * Name:  memlcd_setpower
 *
 * Description:
 *   Enable/disable LCD panel power (0: full off - CONFIG_LCD_MAXPOWER: full on).
 *   On backlit LCDs, this setting may correspond to the backlight setting.
 *
 ******************************************************************************/

static int memlcd_setpower(FAR struct lcd_dev_s *dev, int power)
{
  struct memlcd_dev_s *mlcd = (struct memlcd_dev_s *)dev;
  DEBUGASSERT(mlcd && (unsigned)power <= CONFIG_LCD_MAXPOWER && mlcd->spi);
  lcddbg("%d\n", power);
  mlcd->power = power;

  if (power > 0)
    {
      mlcd->priv->dispcontrol(1);
      memlcd_clear(mlcd);
    }
  else
    {
      mlcd->priv->dispcontrol(0);
    }

  return OK;
}

/*******************************************************************************
 * Name:  memlcd_getcontrast
 *
 * Description:
 *   Get the current contrast setting (0-CONFIG_LCD_MAXCONTRAST).
 *
 ******************************************************************************/

static int memlcd_getcontrast(struct lcd_dev_s *dev)
{
  struct memlcd_dev_s *mlcd = (struct memlcd_dev_s *)dev;
  DEBUGASSERT(mlcd);
  lcddbg("contrast: %d\n", mlcd->contrast);
  return mlcd->contrast;
}

/*******************************************************************************
 * Name:  memlcd_setcontrast
 *
 * Description:
 *   Set LCD panel contrast (0-CONFIG_LCD_MAXCONTRAST).
 *
 ******************************************************************************/

static int memlcd_setcontrast(struct lcd_dev_s *dev, unsigned int contrast)
{
  struct memlcd_dev_s *mlcd = (struct memlcd_dev_s *)dev;
  DEBUGASSERT(mlcd);
  lcddbg("contrast: %d\n", contrast);
  if (contrast > MEMLCD_MAXCONTRAST)
    {
      contrast = MEMLCD_MAXCONTRAST;
    }

  if (contrast < MEMLCD_MINCONTRAST)
    {
      contrast = MEMLCD_MINCONTRAST;
    }

  mlcd->contrast = contrast;
  mlcd->priv->setvcomfreq(contrast);
  return OK;
}

/*******************************************************************************
 * Public Functions
 ******************************************************************************/

/*******************************************************************************
 * Name:  memlcd_initialize
 *
 * Description:
 *   Initialize the Sharp Memory LCD hardware.  The initial state of the
 *   OLED is fully initialized, display memory cleared, and the OLED ready
 *   to use, but with the power setting at 0 (full off == sleep mode).
 *
 * Input Parameters:
 *
 *   spi - A reference to the SPI driver instance.
 *   devno - A value in the range of 0 through CONFIG_memlcd_NINTERFACES-1.
 *     This allows support for multiple OLED devices.
 *
 * Returned Value:
 *
 *   On success, this function returns a reference to the LCD object for
 *   the specified LCD.  NULL is returned on any failure.
 *
 ******************************************************************************/
FAR struct lcd_dev_s *memlcd_initialize(FAR struct spi_dev_s *spi,
                                        FAR struct memlcd_priv_s *priv,
                                        unsigned int devno)
{
  FAR struct memlcd_dev_s *mlcd = &g_memlcddev;

  DEBUGASSERT(spi && priv && devno == 0);

  /* register board specific functions */
  mlcd->priv = priv;

  mlcd->spi = spi;
  memlcd_configspi(spi);

  mlcd->priv->attachirq(memlcd_extcominisr);

  lcddbg("done\n");
  return &mlcd->dev;
}
