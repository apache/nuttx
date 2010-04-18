/**************************************************************************************
 * configs/sam3u-ek/src/up_lcd.c
 * arch/arm/src/board/up_lcd.c
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 * The SAM3U-EK developement board features a TFT/Transmissive color LCD module with
 * touch-screen, FTM280C12D, with integratd driver IC HX8346. The LCD display size
 * is 2.8 inches, with a native resolution of 240 x 320 pixels.
 *
 *   LCD Module Pin Out:                         AT91SAM3U PIO:
 *  -------------------------------------------- --------------------------------------
 *   Pin Symbol Function                         LCD            PeriphA  PeriphB Extra
 *  ---- ------ -------------------------------- -------------- -------- ------- ------
 *   1   GND    Ground                           N/A            ---      ---     ---
 *   2   CS     Chip Select                      PC16           NCS2     PWML3   AD12BAD5
 *   3   RS     Register select signal           PB8 (see A1)   CTS0     A1      AD3
 *   4   WR     Write operation signal           PB23 (NWE)     NWR0/NEW PCK1    ---
 *   5   RD     Read operation signal            PB19 (NRD)     NRD      PWML2   ---
 *   6   DB0    Data bus                         PB9            D0       DTR0    ---
 *   7   DB1    Data bus                         PB10           D1       DSR0    ---
 *   8   DB2    Data bus                         PB11           D2       DCD0    ---
 *   9   DB3    Data bus                         PB12           D3       RI0     ---
 *   10  DB4    Data bus                         PB13           D4       PWMH0   ---
 *   11  DB5    Data bus                         PB14           D5       PWMH1   ---
 *   12  DB6    Data bus                         PB15           D6       PWMH2   ---
 *   13  DB7    Data bus                         PB16           D7       PMWH3   ---
 *   14  DB8    Data bus                         PB25           D8       PWML0   ---
 *   15  DB9    Data bus                         PB26           D9       PWML1   ---
 *   16  DB10   Data bus                         PB27           D10      PWML2   ---
 *   17  DB11   Data bus                         PB28           D11      PWML3   ---
 *   18  DB12   Data bus                         PB29           D12      ---     ---
 *   19  DB13   Data bus                         PB30           D13      ---     ---
 *   20  DB14   Data bus                         PB31           D14      ---     ---
 *   21  DB15   Data bus                         PB6            TIOA1    D15     AD1
 *   22  NC     No connection                    N/A            ---      ---     ---
 *   23  NC     No connection                    N/A            ---      ---     ---
 *   24  RESET  Reset signal                     N/A            ---      ---     ---
 *   25  GND    Ground                           N/A            ---      ---     ---
 *   26  X+     Touch panel X_RIGHT              PA15           SPCK     PWMH2   ---
 *   27  Y+     Touch panel Y_UP                 PA14           MOSI     ---     ---
 *   28  X-     Touch panel X_LEFT               PA13           MISO     ---     ---
 *   29  Y-     Touch panel Y_DOWN               PC14           A3       NPCS2   ---
 *   30  GND    Ground                           N/A            ---      ---     ---
 *   31  VDD1   Power supply for digital IO Pad  N/A            ---      ---     ---
 *   32  VDD2   Power supply for analog circuit  N/A            ---      ---     ---
 *   33  A1     Power supply for backlight       PB8 (see RS)   CTS0     A1      AD3
 *   34  A2     Power supply for backlight       N/A            ---      ---     ---
 *   35  A3     Power supply for backlight       N/A            ---      ---     ---
 *   36  A4     Power supply for backlight       N/A            ---      ---     ---
 *   37  NC     No connection                    N/A            ---      ---     ---
 *   38  NC     No connection                    N/A            ---      ---     ---
 *   39  K      Backlight ground                 N/A            ---      ---     ---
 *
 * The LCD module gets its reset from NRST. As explained previously, this NRST is
 * shared with the JTAG port and the push button BP1. The LCD chip select signal is
 * connected to NCS2 (a dedicated jumper can disable it, making NCS2 available for
 * other custom usage).
 *
 * The SAM3U4E communicates with the LCD through PIOB where a 16-bit parallel
 * “8080-like” protocol data bus has to be implemented by software.
 *
 * LCD backlight is made of 4 white chip LEDs in parallel, driven by an AAT3194
 * charge pump, MN4. The AAT3194 is controlled by the SAM3U4E through a single line
 * Simple Serial Control (S2Cwire) interface, which permits to enable, disable, and
 * set the LED drive current (LED brightness control) from a 32-level logarithmic
 * scale. Four resistors R93/R94/R95/R96 are implemented for optional current
 * limitation.
 *
 * The LCD module integrates a 4-wire touch screen panel controlled by
 * MN5, ADS7843, which is a slave device on the SAM3U4E SPI bus. The ADS7843 touch
 * ADC auxiliary inputs IN3/IN4 are connected to test points for optional function
 * extension.
 *
 **************************************************************************************/

/**************************************************************************************
 * Included Files
 **************************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/lcd.h>

#include "up_arch.h"
#include "sam3u_internal.h"
#include "sam3uek_internal.h"

/**************************************************************************************
 * Pre-processor Definitions
 **************************************************************************************/

/* LCD resolution */

#define SAM3UEK_XRES         320
#define SAM3UEK_YRES         240

/* Color depth and format. BPP=16 R=6, G=6, B=5: RRRR RBBB BBBG GGGG */

#define SAM3UEK_BPP          16
#define SAM3UEK_RGBFMT       FB_FMT_RGB16_565

/**************************************************************************************
 * Private Function Protototypes
 **************************************************************************************/

/* LCD Data Transfer Methods */

static int sam3u_putrun(fb_coord_t row, fb_coord_t col, FAR const uint8_t *buffer,
             size_t npixels);
static int sam3u_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t *buffer,
             size_t npixels);

/* LCD Configuration */

static int sam3u_getvideoinfo(FAR struct lcd_dev_s *dev,
             FAR struct fb_videoinfo_s *vinfo);
static int sam3u_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int planeno,
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

static int sam3u_getpower(struct lcd_dev_s *dev);
static int sam3u_setpower(struct lcd_dev_s *dev, int power);
static int sam3u_getcontrast(struct lcd_dev_s *dev);
static int sam3u_setcontrast(struct lcd_dev_s *dev, unsigned int contrast);

/**************************************************************************************
 * Private Function Protototypes
 **************************************************************************************/

/* This is working memory allocated by the LCD driver for each LCD device
 * and for each color plane.  This memory will hold one raster line of data.
 * The size of the allocated run buffer must therefor be at least
 * (bpp * xres / 8).  Actual alignment of the buffer must conform to the
 * bitwidth of the underlying pixel type.
 *
 * If there are multiple planes, they may share the same working buffer
 * because different planes will not be operate on concurrently.  However,
 * if there are multiple LCD devices, they must each have unique run buffers.
 */

static uint16_t g_runbuffer[SAM3UEK_XRES];

/* This structure describes the overall LCD video controller */

static struct fb_videoinfo_s g_videoinfo =
{
  .fmt     = SAM3UEK_RGBFMT,      /* Color format: RGB16-565: RRRR RGGG GGGB BBBB */
  .xres    = SAM3UEK_XRES,        /* Horizontal resolution in pixel columns */
  .yres    = SAM3UEK_YRES,        /* Vertical resolution in pixel rows */
  .nplanes = 1,                   /* Number of color planes supported */
};

/* This is the standard, NuttX Plane information object */

static struct lcd_planeinfo_s g_planeinfo = 
{
  .putrun = sam3u_putrun,          /* Put a run into LCD memory */
  .getrun = sam3u_getrun,          /* Get a run from LCD memory */
  .buffer = (uint8_t*)g_runbuffer, /* Run scratch buffer */
  .bpp    = SAM3UEK_BPP,           /* Bits-per-pixel */
};

/* This is the standard, NuttX LCD driver object */

static struct lcd_dev_s g_lcddev_s = 
{
  /* LCD Configuration */
 
  .getvideoinfo = sam3u_getvideoinfo,
  .getplaneinfo = sam3u_getplaneinfo,

  /* LCD RGB Mapping -- Not supported */
  /* Cursor Controls -- Not supported */

  /* LCD Specific Controls */

  .getpower     = sam3u_getpower,
  .setpower     = sam3u_setpower,
  .getcontrast  = sam3u_getcontrast,
  .setcontrast  = sam3u_setcontrast,
};

/**************************************************************************************
 * Private Functions
 **************************************************************************************/

/**************************************************************************************
 * Name:  sam3u_putrun
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

static int sam3u_putrun(fb_coord_t row, fb_coord_t col, FAR const uint8_t *buffer,
                        size_t npixels)
{
  return -ENOSYS;
}

/**************************************************************************************
 * Name:  sam3u_getrun
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

static int sam3u_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t *buffer,
                        size_t npixels)
{
  return -ENOSYS;
}

/**************************************************************************************
 * Name:  sam3u_getvideoinfo
 *
 * Description:
 *   Get information about the LCD video controller configuration.
 *
 **************************************************************************************/

static int sam3u_getvideoinfo(FAR struct lcd_dev_s *dev,
                              FAR struct fb_videoinfo_s *vinfo)
{
  return -ENOSYS;
}

/**************************************************************************************
 * Name:  sam3u_getplaneinfo
 *
 * Description:
 *   Get information about the configuration of each LCD color plane.
 *
 **************************************************************************************/

static int sam3u_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int planeno,
                              FAR struct lcd_planeinfo_s *pinfo)
{
  return -ENOSYS;
}

/**************************************************************************************
 * Name:  sam3u_getpower
 *
 * Description:
 *   Get the LCD panel power status (0: full off - CONFIG_LCD_MAXPOWER:
 *   full on.
 *
 **************************************************************************************/

static int sam3u_getpower(struct lcd_dev_s *dev)
{
  return -ENOSYS;
}

/**************************************************************************************
 * Name:  sam3u_setpower
 *
 * Description:
 *   Enable/disable LCD panel power (0: full off - CONFIG_LCD_MAXPOWERL:
 *   full on).
 *
 **************************************************************************************/

static int sam3u_setpower(struct lcd_dev_s *dev, int power)
{
  return -ENOSYS;
}

/**************************************************************************************
 * Name:  sam3u_getcontrast
 *
 * Description:
 *   Get the current contrast setting (0-CONFIG_LCD_MAXCONTRAST).
 *
 **************************************************************************************/

static int sam3u_getcontrast(struct lcd_dev_s *dev)
{
  return -ENOSYS;
}

/**************************************************************************************
 * Name:  sam3u_getcontrast
 *
 * Description:
 *   Set LCD panel contrast (0-CONFIG_LCD_MAXCONTRAST).
 *
 **************************************************************************************/

static int sam3u_setcontrast(struct lcd_dev_s *dev, unsigned int contrast)
{
  return -ENOSYS;
}

/**************************************************************************************
 * Public Functions
 **************************************************************************************/

/**************************************************************************************
 * Name:  up_lcdinitialize
 *
 * Description:
 *   Initialize the LCD video hardware.
 *
 **************************************************************************************/

int up_lcdinitialize(void)
{
  /* Enable LCD EXTCS2 pins */

  sam3u_configgpio(GPIO_LCD_NCS2);
  sam3u_configgpio(GPIO_LCD_RS);
  sam3u_configgpio(GPIO_LCD_NWE);
  sam3u_configgpio(GPIO_LCD_NRD);

  sam3u_configgpio(GPIO_LCD_D0);
  sam3u_configgpio(GPIO_LCD_D1);
  sam3u_configgpio(GPIO_LCD_D2);
  sam3u_configgpio(GPIO_LCD_D3);
  sam3u_configgpio(GPIO_LCD_D4);
  sam3u_configgpio(GPIO_LCD_D5);
  sam3u_configgpio(GPIO_LCD_D6);
  sam3u_configgpio(GPIO_LCD_D7);
  sam3u_configgpio(GPIO_LCD_D8);
  sam3u_configgpio(GPIO_LCD_D9);
  sam3u_configgpio(GPIO_LCD_D10);
  sam3u_configgpio(GPIO_LCD_D11);
  sam3u_configgpio(GPIO_LCD_D12);
  sam3u_configgpio(GPIO_LCD_D13);
  sam3u_configgpio(GPIO_LCD_D14);
  sam3u_configgpio(GPIO_LCD_D15);

  /* Configure LCD Backlight Pin */

  sam3u_configgpio(GPIO_LCD_D15);
  return -ENOSYS;
}

/**************************************************************************************
 * Name:  up_lcdgetdev
 *
 * Description:
 *   Return a a reference to the LCD object for the specified LCD.  This allows
 *   support for multiple LCD devices.
 *
 **************************************************************************************/

FAR struct lcd_dev_s *up_lcdgetdev(int lcdddev)
{
  return NULL;
}

/**************************************************************************************
 * Name:  up_lcduninitialize
 *
 * Description:
 *   Unitialize the framebuffer support.
 *
 **************************************************************************************/

void up_lcduninitialize(void)
{
}


