/**************************************************************************************
 * drivers/lcd/ssd1306.h
 *
 *   Copyright (C) 2015 Alan Carvalho de Assis
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
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

#ifndef __DRIVERS_LCD_SSD1306_H
#define __DRIVERS_LCD_SSD1306_H 1

/**************************************************************************************
 * Included Files
 **************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/ssd1306.h>

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

#if !defined(CONFIG_LCD_SH1106_OLED_132) && !defined(CONFIG_LCD_UG2864HSWEG01) && \
    !defined(CONFIG_LCD_UG2832HSWEG04) && !defined(CONFIG_LCD_DD12864WO4A) && \
    !defined(CONFIG_LCD_HILETGO)
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

/**************************************************************************************
 * Pre-processor Definitions
 **************************************************************************************/

#ifndef CONFIG_NX_BGCOLOR
#  define CONFIG_NX_BGCOLOR SSD1306_Y1_BLACK
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

#define SSD1309_PROTOFF          (0xfd)
#define SSD1309_SETMEMORY        (0x20)
#  define SSD1309_MEMADDR(ma)    ((ma) & 0x03)

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
#  undef IS_SSD1309
#elif defined(CONFIG_LCD_UG2832HSWEG04)
#  define SSD1306_DEV_NATIVE_XRES 128  /* Only 128 of 131 columns used */
#  define SSD1306_DEV_NATIVE_YRES 32   /* 4 pages each 8 rows */
#  define SSD1306_DEV_XOFFSET     2    /* Offset to logical column 0 */
#  define SSD1306_DEV_PAGES       4    /* 4 pages */
#  define SSD1306_DEV_CMNPAD      0x02 /* COM configuration */
#  undef IS_SSD1309
#elif defined(CONFIG_LCD_SH1106_OLED_132)
#  define SSD1306_DEV_NATIVE_XRES 128  /* Only 128 columns used, supporting 132 is a bit difficult */
#  define SSD1306_DEV_NATIVE_YRES 64   /* 8 pages each 8 rows */
#  define SSD1306_DEV_XOFFSET     0    /* Offset to logical column 0 */
#  define SSD1306_DEV_PAGES       8    /* 8 pages */
#  define SSD1306_DEV_CMNPAD      0x12 /* COM configuration */
#  undef IS_SSD1309
#elif defined(CONFIG_LCD_HILETGO)
#  define SSD1306_DEV_NATIVE_XRES 128  /* Only 128 of 132 columns supported */
#  define SSD1306_DEV_NATIVE_YRES 64   /* 8 pages each 8 rows */
#  define SSD1306_DEV_XOFFSET     0    /* Offset to logical column 0 */
#  define SSD1306_DEV_PAGES       8    /* 8 pages */
#  define SSD1306_DEV_CMNPAD      0x12 /* COM configuration */
#  undef IS_SSD1309
#elif defined(CONFIG_LCD_DD12864WO4A)
#  define SSD1306_DEV_NATIVE_XRES 128  /* 128 of 128 columns used */
#  define SSD1306_DEV_NATIVE_YRES 64   /* 8 pages each 8 rows */
#  define SSD1306_DEV_XOFFSET     0    /* Offset to logical column 0 */
#  define SSD1306_DEV_PAGES       8    /* 8 pages */
#  define SSD1306_DEV_CMNPAD      0x10 /* COM configuration */
#  define IS_SSD1309              1    /* SSD1309 based LCD. */
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

#if defined(CONFIG_LCD_SH1106_OLED_132)
#  define SSD1306_DEV_XSTRIDE     (SSD1306_DEV_XRES >> 3)
#else
#  define SSD1306_DEV_XSTRIDE     (SSD1306_DEV_XRES >> 3)
#endif

#define SSD1306_DEV_YSTRIDE       (SSD1306_DEV_YRES >> 3)

/* Color depth and format */

#define SSD1306_DEV_BPP           1
#define SSD1306_DEV_COLORFMT      FB_FMT_Y1

/* Default contrast */

#define SSD1306_DEV_CONTRAST      (128)
#define SSD1309_DEV_CONTRAST      (255)

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
#elif defined(CONFIG_LCD_SH1106_OLED_132)
#  if defined(CONFIG_LCD_LANDSCAPE)
#    undef SSD1306_DEV_REVERSEX
#    define SSD1306_DEV_REVERSEY  1
#  elif defined(CONFIG_LCD_RLANDSCAPE)
#    define SSD1306_DEV_REVERSEX  1
#    undef  SSD1306_DEV_REVERSEY
#  endif
#elif defined(CONFIG_LCD_DD12864WO4A)
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

/**************************************************************************************
 * Public Type Definition
 **************************************************************************************/

/* This structure describes the state of the SSD1306 driver */

struct ssd1306_dev_s
{
  struct lcd_dev_s       dev;      /* Publicly visible device structure */

  /* Private LCD-specific information follows */

#ifdef CONFIG_LCD_SSD1306_SPI
  FAR struct spi_dev_s  *spi;      /* Cached SPI device reference */
#else
  FAR struct i2c_master_s  *i2c;   /* Cached I2C device reference */
  uint8_t                addr;     /* 7-bit I2C address */
#endif
  uint8_t                contrast; /* Current contrast setting */
  bool                   on;       /* true: display is on */
  bool                   is_conf;  /* true: display had been configured */

  FAR const struct ssd1306_priv_s *board_priv; /* Board specific structure */

 /* The SSD1306 does not support reading from the display memory in SPI mode.
  * Since there is 1 BPP and access is byte-by-byte, it is necessary to keep
  * a shadow copy of the framebuffer memory. At 128x64, this amounts to 1KB.
  */

  uint8_t fb[SSD1306_DEV_FBSIZE];
};

/**************************************************************************************
 * Public Function Prototypes
 **************************************************************************************/

int ssd1306_sendbyte(FAR struct ssd1306_dev_s *priv, uint8_t regval);
int ssd1306_sendblk(FAR struct ssd1306_dev_s *priv, uint8_t *data, uint8_t len);

#ifdef CONFIG_LCD_SSD1306_SPI
void ssd1306_select(FAR struct ssd1306_dev_s *priv, bool cs);
void ssd1306_cmddata(FAR struct ssd1306_dev_s *priv, bool cmd);
void ssd1306_configspi(FAR struct spi_dev_s *spi);

#else
#  define ssd1306_select(priv, cs)
#  define ssd1306_cmddata(priv, cmd)
#  define ssd1306_configspi(spi)
#endif

#endif /* __DRIVERS_LCD_SSD1306_H */

