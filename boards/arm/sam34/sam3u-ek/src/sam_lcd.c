/****************************************************************************
 * boards/arm/sam34/sam3u-ek/src/sam_lcd.c
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
 * The SAM3U-EK development board features a TFT/Transmissive color LCD
 * module with touch-screen, FTM280C12D, with integratd driver IC HX8346.
 * The LCD display size is 2.8 inches, with a native resolution of 240 x 320
 * pixels.
 *
 *   LCD Module Pin Out:                   SAM3U PIO:
 *  ---------------------------------- ------------------------------------
 *   Pin Symbol Function                  LCD        PeriphA  PeriphB Extra
 *  ---- ------ ---------------------- ------------ -------- ------- ------
 *   1   GND    Ground                 N/A          ---      ---     ---
 *   2   CS     Chip Select            PC16         NCS2     PWML3   AD12BAD5
 *   3   RS     Register select signal PB8 (see A1) CTS0     A1      AD3
 *   4   WR     Write operation signal PB23 (NWE)   NWR0/NEW PCK1    ---
 *   5   RD     Read operation signal  PB19 (NRD)   NRD      PWML2   ---
 *   6   DB0    Data bus               PB9          D0       DTR0    ---
 *   7   DB1    Data bus               PB10         D1       DSR0    ---
 *   8   DB2    Data bus               PB11         D2       DCD0    ---
 *   9   DB3    Data bus               PB12         D3       RI0     ---
 *   10  DB4    Data bus               PB13         D4       PWMH0   ---
 *   11  DB5    Data bus               PB14         D5       PWMH1   ---
 *   12  DB6    Data bus               PB15         D6       PWMH2   ---
 *   13  DB7    Data bus               PB16         D7       PMWH3   ---
 *   14  DB8    Data bus               PB25         D8       PWML0   ---
 *   15  DB9    Data bus               PB26         D9       PWML1   ---
 *   16  DB10   Data bus               PB27         D10      PWML2   ---
 *   17  DB11   Data bus               PB28         D11      PWML3   ---
 *   18  DB12   Data bus               PB29         D12      ---     ---
 *   19  DB13   Data bus               PB30         D13      ---     ---
 *   20  DB14   Data bus               PB31         D14      ---     ---
 *   21  DB15   Data bus               PB6          TIOA1    D15     AD1
 *   22  NC     No connection          N/A          ---      ---     ---
 *   23  NC     No connection          N/A          ---      ---     ---
 *   24  RESET  Reset signal           N/A          ---      ---     ---
 *   25  GND    Ground                 N/A          ---      ---     ---
 *   26  X+     Touch panel X_RIGHT    PA15         SPCK     PWMH2   ---
 *   27  Y+     Touch panel Y_UP       PA14         MOSI     ---     ---
 *   28  X-     Touch panel X_LEFT     PA13         MISO     ---     ---
 *   29  Y-     Touch panel Y_DOWN     PC14         A3       NPCS2   ---
 *   30  GND    Ground                 N/A          ---      ---     ---
 *   31  VDD1   Power supply for
 *              digital IO Pad         N/A          ---      ---     ---
 *   32  VDD2   Power supply for
 *              analog circuit         N/A          ---      ---     ---
 *   33  A1     Power supply for
 *              backlight              PB8 (see RS) CTS0     A1      AD3
 *   34  A2     Power supply for
 *              backlight              N/A          ---      ---     ---
 *   35  A3     Power supply for
 *              backlight              N/A          ---      ---     ---
 *   36  A4     Power supply for
 *              backlight              N/A          ---      ---     ---
 *   37  NC     No connection          N/A          ---      ---     ---
 *   38  NC     No connection          N/A          ---      ---     ---
 *   39  K      Backlight ground       N/A          ---      ---     ---
 *
 * The LCD module gets its reset from NRST. As explained previously, this
 * NRST is shared with the JTAG port and the push button BP1. The LCD chip
 * select signal is connected to NCS2 (a dedicated jumper can disable it,
 * making NCS2 available for other custom usage).
 *
 * The SAM3U4E communicates with the LCD through PIOB where a 16-bit parallel
 * 8080-like protocol data bus has to be implemented by software.
 *
 * LCD backlight is made of 4 white chip LEDs in parallel, driven by an
 * AAT3194 charge pump, MN4. The AAT3194 is controlled by the SAM3U4E through
 * a single line Simple Serial Control (S2Cwire) interface, which permits to
 * enable, disable, and set the LED drive current (LED brightness control)
 * from a 32-level logarithmic scale.
 * Four resistors R93/R94/R95/R96 are implemented for optional current
 * limitation.
 *
 * The LCD module integrates a 4-wire touch screen panel controlled by
 * MN5, ADS7843, which is a slave device on the SAM3U4E SPI bus.
 * The ADS7843 touch ADC auxiliary inputs IN3/IN4 are connected to test
 * points for optional function extension.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/video/rgbcolors.h>

#include <arch/irq.h>

#include "arm_internal.h"
#include "sam_gpio.h"
#include "hardware/sam_pmc.h"
#include "hardware/sam_smc.h"
#include "sam3u-ek.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Define the following to enable register-level debug output */

#undef CONFIG_LCD_REGDEBUG

/* Verbose debug must also be enabled */

#ifndef CONFIG_DEBUG_FEATURES
#  undef CONFIG_DEBUG_INFO
#  undef CONFIG_DEBUG_LED
#endif

#ifndef CONFIG_DEBUG_INFO
#  undef CONFIG_LCD_REGDEBUG
#endif

/* CONFIG_LCD_MAXCONTRAST -- must be defined and less than 32 */

#if !defined(CONFIG_LCD_MAXCONTRAST) || CONFIG_LCD_MAXCONTRAST < 1 || CONFIG_LCD_MAXCONTRAST > 31
#  error "CONFIG_LCD_MAXCONTRAST must be defined in the range 1 to 31"
#endif

/* Graphics Capbilities *****************************************************/

/* LCD resolution: 320 (columns) by 240 (rows).
 * The physical dimensions of the device are really 240 (columns) by 320
 * (rows), but unless CONFIG_LCD_PORTRAIT is defined, we swap rows and
 * columns in setcursor to make things behave nicer (there IS a performance
 * hit for this swap!).
 */

#ifdef CONFIG_LCD_PORTRAIT
#  define SAM3UEK_XRES         240
#  define SAM3UEK_YRES         320
#else
#  define SAM3UEK_XRES         320
#  define SAM3UEK_YRES         240
#endif

/* Color depth and format. BPP=16 R=6, G=6, B=5: RRRR RBBB BBBG GGGG */

#define SAM3UEK_BPP          16
#define SAM3UEK_RGBFMT       FB_FMT_RGB16_565

/* HX834x Definitions  ******************************************************/

/* HX834x register select */

#define HX843X_LCD_RS        (1 << 1)

/* HX8347 ID code */

#define HX8347_CHIPID        0x47

/* HX8347 LCD Registers */

#define HX8347_R00H          0x00
#define HX8347_R01H          0x01
#define HX8347_R02H          0x02
#define HX8347_R03H          0x03
#define HX8347_R04H          0x04
#define HX8347_R05H          0x05
#define HX8347_R06H          0x06
#define HX8347_R07H          0x07
#define HX8347_R08H          0x08
#define HX8347_R09H          0x09
#define HX8347_R0AH          0x0a
#define HX8347_R0CH          0x0c
#define HX8347_R0DH          0x0d
#define HX8347_R0EH          0x0e
#define HX8347_R0FH          0x0f
#define HX8347_R10H          0x10
#define HX8347_R11H          0x11
#define HX8347_R12H          0x12
#define HX8347_R13H          0x13
#define HX8347_R14H          0x14
#define HX8347_R15H          0x15
#define HX8347_R16H          0x16
#define HX8347_R18H          0x18
#define HX8347_R19H          0x19
#define HX8347_R1AH          0x1a
#define HX8347_R1BH          0x1b
#define HX8347_R1CH          0x1c
#define HX8347_R1DH          0x1d
#define HX8347_R1EH          0x1e
#define HX8347_R1FH          0x1f
#define HX8347_R20H          0x20
#define HX8347_R21H          0x21
#define HX8347_R22H          0x22
#define HX8347_R23H          0x23
#define HX8347_R24H          0x24
#define HX8347_R25H          0x25
#define HX8347_R26H          0x26
#define HX8347_R27H          0x27
#define HX8347_R28H          0x28
#define HX8347_R29H          0x29
#define HX8347_R2AH          0x2a
#define HX8347_R2BH          0x2b
#define HX8347_R2CH          0x2c
#define HX8347_R2DH          0x2d
#define HX8347_R35H          0x35
#define HX8347_R36H          0x36
#define HX8347_R37H          0x37
#define HX8347_R38H          0x38
#define HX8347_R39H          0x39
#define HX8347_R3AH          0x3a
#define HX8347_R3BH          0x3b
#define HX8347_R3CH          0x3c
#define HX8347_R3DH          0x3d
#define HX8347_R3EH          0x3e
#define HX8347_R40H          0x40
#define HX8347_R41H          0x41
#define HX8347_R42H          0x42
#define HX8347_R43H          0x43
#define HX8347_R44H          0x44
#define HX8347_R45H          0x45
#define HX8347_R46H          0x46
#define HX8347_R47H          0x47
#define HX8347_R48H          0x48
#define HX8347_R49H          0x49
#define HX8347_R4AH          0x4a
#define HX8347_R4BH          0x4b
#define HX8347_R4CH          0x4c
#define HX8347_R4DH          0x4d
#define HX8347_R4EH          0x4e
#define HX8347_R4FH          0x4f
#define HX8347_R50H          0x50
#define HX8347_R51H          0x51
#define HX8347_R64H          0x64
#define HX8347_R65H          0x65
#define HX8347_R66H          0x66
#define HX8347_R67H          0x67
#define HX8347_R70H          0x70
#define HX8347_R72H          0x72
#define HX8347_R90H          0x90
#define HX8347_R91H          0x91
#define HX8347_R93H          0x93
#define HX8347_R94H          0x94
#define HX8347_R95H          0x95

/****************************************************************************
 * Private Type Definition
 ****************************************************************************/

/* This structure describes the state of this driver */

struct sam_dev_s
{
  /* Publicly visible device structure */

  struct lcd_dev_s dev;

  /* Private device state */

  uint8_t power;    /* The current power setting */
};

/****************************************************************************
 * Private Function Protototypes
 ****************************************************************************/

/* Low-level HX834x Register access */

static void sam_putreg(uint16_t reg,  uint16_t data);
#ifdef CONFIG_DEBUG_LCD_INFO
static uint16_t sam_getreg(uint16_t reg);
#endif

/* Misc. LCD Helper Functions */

static void sam_setcursor(fb_coord_t row, fb_coord_t col);
static inline void sam_wrsetup(void);
static inline void sam_wrram(uint16_t color);
static inline uint16_t sam_rdram(void);
static void sam_lcdon(void);
static void sam_lcdoff(void);

#if 0 /* CONFIG_DEBUG_LCD_INFO */
static void sam_dumpreg(uint8_t startreg, uint8_t endreg);
#endif

/* LCD Data Transfer Methods */

static int sam_putrun(fb_coord_t row, fb_coord_t col,
                      FAR const uint8_t *buffer,
                      size_t npixels);
static int sam_getrun(fb_coord_t row, fb_coord_t col,
                      FAR uint8_t *buffer,
                      size_t npixels);

/* LCD Configuration */

static int sam_getvideoinfo(FAR struct lcd_dev_s *dev,
             FAR struct fb_videoinfo_s *vinfo);
static int sam_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int planeno,
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

static int sam_getpower(struct lcd_dev_s *dev);
static int sam_setpower(struct lcd_dev_s *dev, int power);
static int sam_getcontrast(struct lcd_dev_s *dev);
static int sam_setcontrast(struct lcd_dev_s *dev, unsigned int contrast);

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

static uint16_t g_runbuffer[SAM3UEK_XRES];

/* This structure describes the overall LCD video controller */

static const struct fb_videoinfo_s g_videoinfo =
{
  .fmt     = SAM3UEK_RGBFMT,      /* Color format: RGB16-565: RRRR RGGG GGGB BBBB */
  .xres    = SAM3UEK_XRES,        /* Horizontal resolution in pixel columns */
  .yres    = SAM3UEK_YRES,        /* Vertical resolution in pixel rows */
  .nplanes = 1,                   /* Number of color planes supported */
};

/* This is the standard, NuttX Plane information object */

static const struct lcd_planeinfo_s g_planeinfo =
{
  .putrun = sam_putrun,             /* Put a run into LCD memory */
  .getrun = sam_getrun,             /* Get a run from LCD memory */
  .buffer = (uint8_t *)g_runbuffer, /* Run scratch buffer */
  .bpp    = SAM3UEK_BPP,            /* Bits-per-pixel */
};

/* This is the standard, NuttX LCD driver object */

static struct sam_dev_s g_lcddev_s =
{
  .dev =
  {
    /* LCD Configuration */

    .getvideoinfo = sam_getvideoinfo,
    .getplaneinfo = sam_getplaneinfo,

    /* LCD RGB Mapping -- Not supported */

    /* Cursor Controls -- Not supported */

    /* LCD Specific Controls */

    .getpower     = sam_getpower,
    .setpower     = sam_setpower,
    .getcontrast  = sam_getcontrast,
    .setcontrast  = sam_setcontrast,
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  sam_putreg
 *
 * Description:
 *   Write to a HX834x register
 *
 ****************************************************************************/

static void sam_putreg(uint16_t reg,  uint16_t data)
{
  lcdinfo("base: %08x RS: %04x data: %04x\n",
          LCD_BASE, LCD_BASE + HX843X_LCD_RS, data);
  putreg16(reg, LCD_BASE);
  putreg16(data, LCD_BASE + HX843X_LCD_RS);
}

/****************************************************************************
 * Name:  sam_getreg
 *
 * Description:
 *   Read from a HX834x register
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_LCD_INFO
static uint16_t sam_getreg(uint16_t reg)
{
  uint16_t data;

  putreg16(reg, LCD_BASE);
  data = getreg16(LCD_BASE + HX843X_LCD_RS);
  lcdinfo("base: %08x RS: %04x data: %04x\n",
          LCD_BASE, LCD_BASE + HX843X_LCD_RS, data);
  return data;
}
#endif

/****************************************************************************
 * Name:  sam_setcursor
 *
 * Description:
 *   Set the LCD cursor position.
 *
 ****************************************************************************/

static void sam_setcursor(fb_coord_t row, fb_coord_t col)
{
  uint8_t  x1;
  uint8_t  x2;
  uint8_t  y1;
  uint8_t  y2;

  /* Get the upper and lower x and y positions */

  x1  = (uint8_t)col;
  x2  = (uint8_t)((uint16_t)col >> 8);

  y1  = (uint8_t)row;
  y2  = (uint8_t)((uint16_t)row >> 8);

  /* Then set the cursor position */

  sam_putreg(HX8347_R02H, x2);        /* column high */
  sam_putreg(HX8347_R03H, x1);        /* column low */
  sam_putreg(HX8347_R06H, y2);        /* row high */
  sam_putreg(HX8347_R07H, y1);        /* row low */
}

/****************************************************************************
 * Name:  sam_wrsetup
 *
 * Description:
 *   Set up for a GRAM write operation.
 *
 ****************************************************************************/

static inline void sam_wrsetup(void)
{
  putreg16(HX8347_R22H, LCD_BASE);
}

/****************************************************************************
 * Name: sam_wrram
 *
 * Description:
 *   Write to the 16-bit GRAM register
 *
 ****************************************************************************/

static inline void sam_wrram(uint16_t color)
{
  putreg16(color, LCD_BASE + HX843X_LCD_RS);
}

/****************************************************************************
 * Name: sam_rdram
 *
 * Description:
 *   Read from the 16-bit GRAM register
 *
 ****************************************************************************/

static inline uint16_t sam_rdram(void)
{
  return getreg16(LCD_BASE + HX843X_LCD_RS);
}

/****************************************************************************
 * Name:  sam_lcdon
 *
 * Description:
 *   Turn the LCD on
 *
 ****************************************************************************/

static void sam_lcdon(void)
{
  /* Display ON Setting */

  lcdinfo("ON\n");
  sam_putreg(HX8347_R90H, 0x7f);      /* SAP=0111 1111 */
  sam_putreg(HX8347_R26H, 0x04);      /* GON=0 DTE=0 D=01 */
  up_mdelay(100);
  sam_putreg(HX8347_R26H, 0x24);      /* GON=1 DTE=0 D=01 */
  sam_putreg(HX8347_R26H, 0x2c);      /* GON=1 DTE=0 D=11 */
  up_mdelay(100);
  sam_putreg(HX8347_R26H, 0x3c);      /* GON=1 DTE=1 D=11 */
}

/****************************************************************************
 * Name:  sam_lcdoff
 *
 * Description:
 *   Turn the LCD off
 *
 ****************************************************************************/

static void sam_lcdoff(void)
{
  lcdinfo("OFF\n");
  sam_putreg(HX8347_R90H, 0x00);      /* SAP=0000 0000 */
  sam_putreg(HX8347_R26H, 0x00);      /* GON=0 DTE=0 D=00 */
}

/****************************************************************************
 * Name:  sam_dumpreg
 *
 * Description:
 *   Dump a range of LCD registers.
 *
 ****************************************************************************/

#if 0 /* CONFIG_DEBUG_LCD_INFO */
static void sam_dumpreg(uint8_t startreg, uint8_t endreg)
{
  uint16_t value;
  uint8_t  addr;

  for (addr = startreg; addr <= endreg; addr++)
    {
      value = sam_getreg(addr);
      lcdinfo(" %02x: %04x\n", addr, value);
    }
}
#endif

/****************************************************************************
 * Name:  sam_putrun
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
 ****************************************************************************/

static int sam_putrun(fb_coord_t row, fb_coord_t col,
                      FAR const uint8_t *buffer,
                      size_t npixels)
{
  uint16_t *run = (uint16_t *)buffer;
  unsigned int i;

  /* Buffer must be provided and aligned to a 16-bit address boundary */

  lcdinfo("row: %d col: %d npixels: %d\n", row, col, npixels);
  DEBUGASSERT(buffer && ((uintptr_t)buffer & 1) == 0);

#ifdef CONFIG_LCD_PORTRAIT
  /* Set up to write the run. */

  sam_setcursor(row, col);
  sam_wrsetup();

  /* Write the run to GRAM. */

  for (i = 0; i < npixels; i++)
    {
      /* Write the pixel pixel to GRAM */

      sam_wrram(*run++);
    }
#else
  /* Write the run to GRAM.
   * Because rows and columns are swapped, we need to reset
   * the cursor position for every pixel.  We could do this much faster if we
   * adapted to the strange device aspect ratio.
   */

  col = 319 - col;
  for (i = 0; i < npixels; i++)
    {
      /* Set up to write the next pixel.
       * Swapping x and y orientations so that the image
       * comes out with the 320x240 aspect ratio (not the native 240x320).
       * That is:
       *
       *   row: 0-239 maps to x: 0-239
       *   col: 0-319 maps to y: 319-0
       */

      sam_setcursor(col--, row);
      sam_wrsetup();

      /* Write the pixel pixel to GRAM */

      sam_wrram(*run++);
    }
#endif

  return OK;
}

/****************************************************************************
 * Name:  sam_getrun
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
 ****************************************************************************/

static int sam_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t *buffer,
                      size_t npixels)
{
  uint16_t *run = (uint16_t *)buffer;
  unsigned int i;

  /* Buffer must be provided and aligned to a 16-bit address boundary */

  lcdinfo("row: %d col: %d npixels: %d\n", row, col, npixels);
  DEBUGASSERT(buffer && ((uintptr_t)buffer & 1) == 0);

#ifdef CONFIG_LCD_PORTRAIT
  /* Set up to read the run */

  sam_setcursor(row, col);

  /* Read the run from GRAM. */

  for (i = 0; i < npixels; i++)
    {
      /* Read the next pixel */

      *run++ = sam_rdram();
    }
#else
  /* Read the run from GRAM  Because rows and columns are swapped, we need to
   * reset the cursor position for every pixel.
   * We could do this much faster if we adapted to the strange device aspect
   * ratio.
   */

  col = 319 - col;
  for (i = 0; i < npixels; i++)
    {
      /* Read the next pixel..
       * Swapping x and y orientations so that the image comes out with the
       * 320x240 aspect ratio (not the native 240x320).
       * That is:
       *
       *   row: 0-239 maps to x: 0-239
       *   col: 0-319 maps to y: 319-0
       */

      sam_setcursor(col--, row);
      *run++ = sam_rdram();
    }
#endif

  return OK;
}

/****************************************************************************
 * Name:  sam_getvideoinfo
 *
 * Description:
 *   Get information about the LCD video controller configuration.
 *
 ****************************************************************************/

static int sam_getvideoinfo(FAR struct lcd_dev_s *dev,
                            FAR struct fb_videoinfo_s *vinfo)
{
  DEBUGASSERT(dev && vinfo);
  lcdinfo("fmt: %d xres: %d yres: %d nplanes: %d\n",
          g_videoinfo.fmt, g_videoinfo.xres,
          g_videoinfo.yres, g_videoinfo.nplanes);
  memcpy(vinfo, &g_videoinfo, sizeof(struct fb_videoinfo_s));
  return OK;
}

/****************************************************************************
 * Name:  sam_getplaneinfo
 *
 * Description:
 *   Get information about the configuration of each LCD color plane.
 *
 ****************************************************************************/

static int sam_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int planeno,
                            FAR struct lcd_planeinfo_s *pinfo)
{
  DEBUGASSERT(dev && pinfo && planeno == 0);
  lcdinfo("planeno: %d bpp: %d\n", planeno, g_planeinfo.bpp);
  memcpy(pinfo, &g_planeinfo, sizeof(struct lcd_planeinfo_s));
  return OK;
}

/****************************************************************************
 * Name:  sam_getpower
 *
 * Description:
 *   Get the LCD panel power status (0: full off - CONFIG_LCD_MAXPOWER:
 *   full on.
 *   On backlit LCDs, this setting may correspond to the backlight setting.
 *
 ****************************************************************************/

static int sam_getpower(struct lcd_dev_s *dev)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;
  DEBUGASSERT(dev);
  lcdinfo("power: %d\n", priv->power);
  return priv->power;
}

/****************************************************************************
 * Name:  sam_setpower
 *
 * Description:
 *   Enable/disable LCD panel power (0: full off - CONFIG_LCD_MAXPOWER:
 *   full on).
 *   On backlit LCDs, this setting may correspond to the backlight setting.
 *
 *   LCD backlight is made of 4 white chip LEDs in parallel, driven by an
 *   AAT3194 charge pump, MN4. The AAT3194 is controlled by the SAM3U4E
 *   through a single line. Simple Serial Control (S2Cwire) interface,
 *   which permits to enable, disable, and set the LED drive current
 *  (LED brightness control) from a 32-level logarithmic scale. Four
 *   resistors R93/R94/R95/R96 are implemented for optional current
 *   limitation.
 *
 ****************************************************************************/

static int sam_setpower(struct lcd_dev_s *dev, int power)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;
  unsigned int i;

  lcdinfo("power: %d\n", power);
  DEBUGASSERT(power <= CONFIG_LCD_MAXPOWER);

  /* Switch off backlight */

  sam_gpiowrite(GPIO_LCD_BKL, false);

  /* For for at least 500uS to drain the charge pump */

  up_udelay(500);

  /* Set new backlight level by pumping "level" times */

  for (i = 0; i < power; i++)
    {
      sam_gpiowrite(GPIO_LCD_BKL, false);
      sam_gpiowrite(GPIO_LCD_BKL, false);
      sam_gpiowrite(GPIO_LCD_BKL, false);
      sam_gpiowrite(GPIO_LCD_BKL, true);
      sam_gpiowrite(GPIO_LCD_BKL, true);
      sam_gpiowrite(GPIO_LCD_BKL, true);
    }

  /* This delay seems to be required...
   * perhaps because of the big current jump?
   */

  if (power != LCD_FULL_OFF)
    {
      up_mdelay(100);
    }

  priv->power = power;
  return OK;
}

/****************************************************************************
 * Name:  sam_getcontrast
 *
 * Description:
 *   Get the current contrast setting (0-CONFIG_LCD_MAXCONTRAST).
 *
 ****************************************************************************/

static int sam_getcontrast(struct lcd_dev_s *dev)
{
  lcdinfo("Not implemented\n");
  return -ENOSYS;
}

/****************************************************************************
 * Name:  sam_getcontrast
 *
 * Description:
 *   Set LCD panel contrast (0-CONFIG_LCD_MAXCONTRAST).
 *
 ****************************************************************************/

static int sam_setcontrast(struct lcd_dev_s *dev, unsigned int contrast)
{
  lcdinfo("contrast: %d\n", contrast);
  return -ENOSYS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  board_lcd_initialize
 *
 * Description:
 *   Initialize the LCD video hardware.  The initial state of the LCD is
 *   fully initialized, display memory cleared, and the LCD ready to use,
 *   but with the power setting at 0 (full off).
 *
 ****************************************************************************/

int board_lcd_initialize(void)
{
#ifdef CONFIG_DEBUG_LCD_INFO
  uint16_t hxregval;
#endif
  uint32_t regval;
  unsigned int i;

  lcdinfo("Initializing\n");

  /* Enable LCD EXTCS2 pins */

  sam_configgpio(GPIO_LCD_NCS2);
  sam_configgpio(GPIO_LCD_RS);
  sam_configgpio(GPIO_LCD_NWE);
  sam_configgpio(GPIO_LCD_NRD);

  sam_configgpio(GPIO_LCD_D0);
  sam_configgpio(GPIO_LCD_D1);
  sam_configgpio(GPIO_LCD_D2);
  sam_configgpio(GPIO_LCD_D3);
  sam_configgpio(GPIO_LCD_D4);
  sam_configgpio(GPIO_LCD_D5);
  sam_configgpio(GPIO_LCD_D6);
  sam_configgpio(GPIO_LCD_D7);
  sam_configgpio(GPIO_LCD_D8);
  sam_configgpio(GPIO_LCD_D9);
  sam_configgpio(GPIO_LCD_D10);
  sam_configgpio(GPIO_LCD_D11);
  sam_configgpio(GPIO_LCD_D12);
  sam_configgpio(GPIO_LCD_D13);
  sam_configgpio(GPIO_LCD_D14);
  sam_configgpio(GPIO_LCD_D15);

#ifdef CONFIG_LCD_REGDEBUG
  sam_dumpgpio(GPIO_PORT_PIOB, "PORTB");
  sam_dumpgpio(GPIO_PORT_PIOC, "PORTC");
#endif

  /* Configure LCD Backlight Pin */

  sam_configgpio(GPIO_LCD_BKL);

  /* Enable SMC peripheral clock */

  putreg32((1 << SAM_PID_SMC), SAM_PMC_PCER);
  lcdinfo("PMC PCSR: %08" PRIx32 " SMC: %08x\n",
          getreg32(SAM_PMC_PCSR), (1 << SAM_PID_SMC));

  /* Configure SMC CS2 */

  regval = (4 << SMCCS_SETUP_NWESETUP_SHIFT) |
           (2 << SMCCS_SETUP_NCSWRSETUP_SHIFT) |
           (4 << SMCCS_SETUP_NRDSETUP_SHIFT) |
           (2 << SMCCS_SETUP_NCSRDSETUP_SHIFT);
  putreg32(regval, SAM_SMCCS_SETUP(2));

  regval = (5 << SMCCS_PULSE_NWEPULSE_SHIFT) |
           (18 << SMCCS_PULSE_NCSWRPULSE_SHIFT) |
           (5 << SMCCS_PULSE_NRDPULSE_SHIFT)  |
           (18 << SMCCS_PULSE_NCSRDPULSE_SHIFT);
  putreg32(regval, SAM_SMCCS_PULSE(2));

  regval = (22 << SMCCS_CYCLE_NWECYCLE_SHIFT) |
           (22 << SMCCS_CYCLE_NRDCYCLE_SHIFT);
  putreg32(regval, SAM_SMCCS_CYCLE(2));

  regval  = getreg32(SAM_SMCCS_MODE(2));
  regval &= ~(SMCCS_MODE_DBW_MASK | SMCCS_MODE_PMEN);
  regval |= (SMCCS_MODE_READMODE)  |
            (SMCCS_MODE_WRITEMODE) |
            (SMCCS_MODE_DBW_16BITS);
  putreg32(regval, SAM_SMCCS_MODE(2));

  lcdinfo("SMC SETUP[%08x]: %08" PRIx32 " PULSE[%08x]: %08" PRIx32 "\n",
          SAM_SMCCS_SETUP(2), getreg32(SAM_SMCCS_SETUP(2)),
          SAM_SMCCS_PULSE(2), getreg32(SAM_SMCCS_PULSE(2)));
  lcdinfo("    CYCLE[%08x]: %08" PRIx32 " MODE[%08x]:  %08" PRIx32 "\n",
          SAM_SMCCS_CYCLE(2), getreg32(SAM_SMCCS_CYCLE(2)),
          SAM_SMCCS_MODE(2),  getreg32(SAM_SMCCS_MODE(2)));

#ifdef CONFIG_DEBUG_LCD_INFO
  /* Check HX8347 Chip ID */

  hxregval = sam_getreg(HX8347_R67H);
  lcdinfo("Chip ID: %04x\n", hxregval);
  if (hxregval != HX8347_CHIPID)
    {
      lcderr("ERROR: Bad chip ID: %04x Expected: %04x\n",
             hxregval, HX8347_CHIPID);
      return -ENODEV;
    }
#endif

  /* Initialize LCD controller (HX8347) --
   * Magic code from Atmel LCD example
   */

  /* Start internal OSC */

  sam_putreg(HX8347_R19H, 0x49);      /* OSCADJ=10 0000 OSD_EN=1 60Hz */
  sam_putreg(HX8347_R93H, 0x0c);      /* RADJ=1100 */

  /* Power on flow */

  sam_putreg(HX8347_R44H, 0x4d);      /* VCM=100 1101 */
  sam_putreg(HX8347_R45H, 0x11);      /* VDV=1 0001 */
  sam_putreg(HX8347_R20H, 0x40);      /* BT=0100 */
  sam_putreg(HX8347_R1DH, 0x07);      /* VC1=111 */
  sam_putreg(HX8347_R1EH, 0x00);      /* VC3=000 */
  sam_putreg(HX8347_R1FH, 0x04);      /* VRH=0100 */
  sam_putreg(HX8347_R1CH, 0x04);      /* AP=100 */
  sam_putreg(HX8347_R1BH, 0x10);      /* GASENB=0 PON=1 DK=0 XDK=0 DDVDH_TRI=0 STB=0 */
  up_mdelay(50);
  sam_putreg(HX8347_R43H, 0x80);      /* Set VCOMG=1 */
  up_mdelay(50);

  /* Gamma for CMO 2.8 */

  sam_putreg(HX8347_R46H, 0x95);
  sam_putreg(HX8347_R47H, 0x51);
  sam_putreg(HX8347_R48H, 0x00);
  sam_putreg(HX8347_R49H, 0x36);
  sam_putreg(HX8347_R4AH, 0x11);
  sam_putreg(HX8347_R4BH, 0x66);
  sam_putreg(HX8347_R4CH, 0x14);
  sam_putreg(HX8347_R4DH, 0x77);
  sam_putreg(HX8347_R4EH, 0x13);
  sam_putreg(HX8347_R4FH, 0x4c);
  sam_putreg(HX8347_R50H, 0x46);
  sam_putreg(HX8347_R51H, 0x46);

  /* 240x320 window setting */

  sam_putreg(HX8347_R02H, 0x00);      /* Column address start2 */
  sam_putreg(HX8347_R03H, 0x00);      /* Column address start1 */
  sam_putreg(HX8347_R04H, 0x00);      /* Column address end2 */
  sam_putreg(HX8347_R05H, 0xef);      /* Column address end1 */
  sam_putreg(HX8347_R06H, 0x00);      /* Row address start2 */
  sam_putreg(HX8347_R07H, 0x00);      /* Row address start1 */
  sam_putreg(HX8347_R08H, 0x01);      /* Row address end2 */
  sam_putreg(HX8347_R09H, 0x3f);      /* Row address end1 */

  /* Display Setting */

  sam_putreg(HX8347_R01H, 0x06);      /* IDMON=0 INVON=1 NORON=1 PTLON=0 */
  sam_putreg(HX8347_R16H, 0xc8);      /* MY=1 MX=1 MV=0 BGR=1 */
  sam_putreg(HX8347_R23H, 0x95);      /* N_DC=1001 0101 */
  sam_putreg(HX8347_R24H, 0x95);      /* P_DC=1001 0101 */
  sam_putreg(HX8347_R25H, 0xff);      /* I_DC=1111 1111 */
  sam_putreg(HX8347_R27H, 0x06);      /* N_BP=0000 0110 */
  sam_putreg(HX8347_R28H, 0x06);      /* N_FP=0000 0110 */
  sam_putreg(HX8347_R29H, 0x06);      /* P_BP=0000 0110 */
  sam_putreg(HX8347_R2AH, 0x06);      /* P_FP=0000 0110 */
  sam_putreg(HX8347_R2CH, 0x06);      /* I_BP=0000 0110 */
  sam_putreg(HX8347_R2DH, 0x06);      /* I_FP=0000 0110 */
  sam_putreg(HX8347_R3AH, 0x01);      /* N_RTN=0000 N_NW=001 */
  sam_putreg(HX8347_R3BH, 0x01);      /* P_RTN=0000 P_NW=001 */
  sam_putreg(HX8347_R3CH, 0xf0);      /* I_RTN=1111 I_NW=000 */
  sam_putreg(HX8347_R3DH, 0x00);      /* DIV=00 */
  sam_putreg(HX8347_R3EH, 0x38);      /* SON=38h */
  sam_putreg(HX8347_R40H, 0x0f);      /* GDON=0Fh */
  sam_putreg(HX8347_R41H, 0xf0);      /* GDOF=F0h */

  /* Set LCD backlight to FULL off */

  sam_setpower(&g_lcddev_s.dev, LCD_FULL_OFF);

  /* Fill the display memory with the color BLACK */

  sam_setcursor(0, 0);
  sam_wrsetup();
  for (i = 0; i < (SAM3UEK_XRES * SAM3UEK_YRES); i++)
    {
      sam_wrram(RGB16_BLACK);
    }

  /* Turn the LCD on (but with the backlight off) */

  sam_lcdon();
  return OK;
}

/****************************************************************************
 * Name:  board_lcd_getdev
 *
 * Description:
 *   Return a a reference to the LCD object for the specified LCD.
 *   This allows support for multiple LCD devices.
 *
 ****************************************************************************/

FAR struct lcd_dev_s *board_lcd_getdev(int lcddev)
{
  lcdinfo("lcddev: %d\n", lcddev);
  return lcddev == 0 ? &g_lcddev_s.dev : NULL;
}

/****************************************************************************
 * Name:  board_lcd_uninitialize
 *
 * Description:
 *   Uninitialize the framebuffer support.
 *
 ****************************************************************************/

void board_lcd_uninitialize(void)
{
  /* Turn the LCD off */

  sam_lcdoff();

  /* Set LCD backlight to FULL off */

  sam_setpower(&g_lcddev_s.dev, LCD_FULL_OFF);

  /* Disable SMC peripheral clock */

  putreg32((1 << SAM_PID_SMC), SAM_PMC_PCDR);
}
