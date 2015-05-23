/************************************************************************************
 * configs/sam4e-ek/src/sam_ili9341.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 * - This driver is a modification of the SAMA4E ILI9325 LCD driver.
 * - ILI9341 Datasheet, Version: V1.11, ILI9341_DS_V1.11.pdf, ILI TECHNOLOGY CORP.,
 * - SAM4Ex Datasheet, Atmel
 * - Atmel ILI93241 Sample code for the SAM4E
 *
 * Some the LCD and SMC initialization logic comes from Atmel sample code for the
 * SAM4E.  The Atmel sample code has a BSD-like license with an additional
 * requirement that restricts the code from being used on anything but Atmel
 * microprocessors.  I do not believe that this file "derives" from the Atmel
 * sample code nor do I believe that it contains anything but generally available
 * ILI9341 and SAM4x logic.  Credit, however, needs to go where it is due.
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
 ************************************************************************************/

/**************************************************************************************
 *
 * The SAM4E-EK carries a TFT transmissive LCD module with touch panel, FTM280C34D.
 * Its integrated driver IC is ILI9341. The LCD display area is 2.8 inches diagonally
 * measured, with a native resolution of 240 x 320 dots.
 *
 * The SAM4E16 communicates with the LCD through PIOC where an 8-bit parallel "8080-
 * like" protocol data bus has to be implemented in software.
 *
 *  ---- ----- --------- --------------------------------
 *  PIN  PIO   SIGNAL    NOTES
 *  ---- ----- --------- --------------------------------
 *    1                  VDD
 *    2  PC7   DB17
 *    3  PC6   DB16
 *    4  PC5   DB15
 *    5  PC4   DB14
 *    6  PC3   DB13
 *    7  PC2   DB12
 *    8  PC1   DB11
 *    9  PC0   DB10
 *   10        DB9       Pulled low
 *   11        DB8       Pulled low
 *   12        DB7       Pulled low
 *   13        DB6       Pulled low
 *   14        DB5       Pulled low
 *   15        DB4       Pulled low
 *   16        DB3       Pulled low
 *   17        DB2       Pulled low
 *   18        DB1       Pulled low
 *   19        DB0       Pulled low
 *  ---- ----- --------- --------------------------------
 *   20                  VDD
 *   21  PC11  RD
 *   22  PC8   WR
 *   23  PC19  RS
 *   24  PD18  CS        Via J8, pulled high.
 *   25        RESET     Connects to NSRST
 *   26        IM0       Pulled high
 *   27        IM1       Grounded
 *   28        GND
 *  ---- ----- --------- --------------------------------
 *   29 [PC13] LED-A     Backlight controls:  PC13 enables
 *   30 [PC13] LEDK1       AAT3155 charge pump that drives
 *   31 [PC13] LEDK2       the backlight LEDs
 *   32 [PC13] LEDK3
 *   33 [PC13] LEDK4
 *   34 [PC13] LEDK1
 *  ---- ----- --------- --------------------------------
 *   35        Y+        These go to the ADS7843
 *   36        Y-          touchscreen controller.
 *   37        X+
 *   38        X-
 *   39        NC
 *  ---- ----- --------- --------------------------------
 *
 * LCD backlight is made of 4 white chip LEDs in parallel, driven by an AAT3155
 * charge pump, MN4. The AAT3155 is controlled by the SAM3U4E through a single line
 * Simple Serial Control (S2Cwire) interface, which permits to enable, disable, and
 * set the LED drive current (LED brightness control) from a 32-level logarithmic
 * scale. Four resistors R93/R94/R95/R96 are implemented for optional current
 * limitation.
 *
 **************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/ili9341.h>
#include <nuttx/video/rgbcolors.h>

#include <arch/irq.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "sam_gpio.h"
#include "sam_periphclks.h"
#include "chip/sam_pmc.h"
#include "chip/sam_smc.h"
#include "sam4e-ek.h"

#ifdef CONFIG_LCD

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* SMC must be selected */

#if !defined(CONFIG_SAM34_SMC)
#  error "CONFIG_SAM34_SMC is required"
#endif

/* Check contrast selection */

#if !defined(CONFIG_LCD_MAXCONTRAST)
#  define CONFIG_LCD_MAXCONTRAST 1
#endif

/* Check power setting */

#if !defined(CONFIG_LCD_MAXPOWER)
#  define CONFIG_LCD_MAXPOWER 16
#endif

#if CONFIG_LCD_MAXPOWER < 16
#  error CONFIG_LCD_MAXPOWER should be >= 16
#  undef CONFIG_LCD_MAXPOWER
#  define CONFIG_LCD_MAXPOWER 16
#endif

#if CONFIG_LCD_MAXPOWER > 255
#  error "CONFIG_LCD_MAXPOWER must be less than 256 to fit in uint8_t"
#endif

/* Check orientation */

#if defined(CONFIG_LCD_LANDSCAPE)
#  if defined(CONFIG_LCD_PORTRAIT) || defined(CONFIG_LCD_RPORTRAIT) || \
      defined(CONFIG_LCD_RLANDSCAPE)
#    error "Cannot define both portrait and any other orientations"
#  endif
#elif defined(CONFIG_LCD_RLANDSCAPE)
#  if defined(CONFIG_LCD_PORTRAIT) || defined(CONFIG_LCD_RPORTRAIT)
#    error "Cannot define both rportrait and any other orientations"
#  endif
#elif defined(CONFIG_LCD_PORTRAIT)
#  ifdef CONFIG_LCD_RPORTRAIT
#    error "Cannot define both landscape and any other orientations"
#  endif
#elif !defined(CONFIG_LCD_RPORTRAIT)
#  define CONFIG_LCD_LANDSCAPE 1
#endif

/* Background color */

#if !defined(CONFIG_SAM4EEK_LCD_BGCOLOR)
#  define CONFIG_SAM4EEK_LCD_BGCOLOR 0
#endif

/* Define CONFIG_DEBUG_LCD to enable detailed LCD debug output. Verbose debug must
 * also be enabled.
 */

#ifndef CONFIG_DEBUG
#  undef CONFIG_DEBUG_VERBOSE
#  undef CONFIG_DEBUG_GRAPHICS
#  undef CONFIG_DEBUG_LCD
#  undef CONFIG_LCD_REGDEBUG
#endif

#ifndef CONFIG_DEBUG_VERBOSE
#  undef CONFIG_DEBUG_LCD
#endif

/* Display/Color Properties ***********************************************************/
/* Display Resolution */

#if defined(CONFIG_LCD_LANDSCAPE) || defined(CONFIG_LCD_RLANDSCAPE)
#  define SAM_XRES            320
#  define SAM_YRES            240
#else
#  define SAM_XRES            240
#  define SAM_YRES            320
#endif

/* Color depth and format */

#if defined(CONFIG_SAM4EEK_LCD_RGB565)
#  define SAM_BPP             16
#  define SAM_COLORFMT        FB_FMT_RGB16_565
#elif defined(CONFIG_SAM4EEK_LCD_RGB24)
#  define SAM_BPP             24
#  define SAM_COLORFMT        FB_FMT_RGB24
#else /* if defined(CONFIG_SAM4EEK_LCD_RGB32) -- without ALPHA */
#  define SAM_BPP             32
#  define SAM_COLORFMT        FB_FMT_RGB32
#endif

/* Color decoding macros */

#ifdef CONFIG_SAM4EEK_LCD_RGB565
#  define RGB_RED(rgb)        RGB16RED(rgb)
#  define RGB_GREEN(rgb)      RGB16GREEN(rgb)
#  define RGB_BLUE(rgb)       RGB16BLUE(rgb)
#  define RGB_COLOR(r,g,b)    RGBTO16(r,g,b)
#else /* RGB888 or RGB32 without ALPHA */
#  define RGB_RED(rgb)        RGB24RED(rgb)
#  define RGB_GREEN(rgb)      RGB24GREEN(rgb)
#  define RGB_BLUE(rgb)       RGB24BLUE(rgb)
#  define RGB_COLOR(r,g,b)    RGBTO24(r,g,b)
#endif

/* SAM4E-EK LCD Hardware Definitions **************************************************/
/* LCD /CS is CE4,  Bank 3 of NOR/SRAM Bank 1~4 */

#define SAM_LCD_BASE          ((uintptr_t)SAM_EXTCS1_BASE)
#define LCD_INDEX             (*(volatile uint8_t *)(SAM_LCD_BASE))
#define LCD_DATA              (*(volatile uint8_t *)(SAM_LCD_BASE + 2))

/* LCD SMC chip select number to be set */

#define SAM_LCD_CS            1

/* Backlight */

#define BKL_LEVELS            16
#define BKL_PULSE_DURATION    24
#define BKL_ENABLE_DURATION   (128*1024)
#define BKL_DISABLE_DURATION  (128*1024)

/* Debug ******************************************************************************/

#ifdef CONFIG_DEBUG_LCD
#  define lcddbg              dbg
#  define lcdvdbg             vdbg
#else
#  define lcddbg(x...)
#  define lcdvdbg(x...)
#endif

/************************************************************************************
 * Private Type Definition
 ************************************************************************************/

/* Type definition for the correct size of one pixel (from the application standpoint). */

#ifdef CONFIG_SAM4EEK_LCD_RGB565
typedef uint16_t sam_color_t;
#else /* RGB888 or RGB32 (without ALPHA) */
typedef uint32_t sam_color_t;
#endif

/* This structure describes the LCD registers */

struct lcd_regs_s
{
  volatile uint16_t index;
  volatile uint16_t value;
};

/* This structure describes the state of this driver */

struct sam_dev_s
{
  /* Publicly visible device structure */

  struct lcd_dev_s dev;

  /* Private LCD-specific information follows */

  uint8_t  power;       /* Current power setting */
  bool     output;      /* True: Configured for output */
};

/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/
/* Low Level LCD access */

static void sam_putreg(uint8_t regaddr, FAR const uint8_t *buffer, unsigned int buflen);
static void sam_getreg(uint8_t regaddr, FAR uint8_t *buffer, unsigned int buflen);
static void sam_setwindow(sam_color_t row, sam_color_t col,
                          sam_color_t width, sam_color_t height);
static inline void sam_gram_wrprepare(void);
static inline void sam_gram_rdprepare(void);
static inline void sam_gram_write(sam_color_t color);
static inline sam_color_t sam_gram_read(void);

/* Backlight/power controls */

static void sam_disable_backlight(void);
static void sam_set_backlight(unsigned int power);
static int  sam_poweroff(FAR struct sam_dev_s *priv);

/* LCD Data Transfer Methods */

static int  sam_putrun(fb_coord_t row, fb_coord_t col, FAR const uint8_t *buffer,
              size_t npixels);
static int  sam_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t *buffer,
              size_t npixels);

/* LCD Configuration */

static int  sam_getvideoinfo(FAR struct lcd_dev_s *dev,
              FAR struct fb_videoinfo_s *vinfo);
static int  sam_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int planeno,
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

static int  sam_getpower(struct lcd_dev_s *dev);
static int  sam_setpower(struct lcd_dev_s *dev, int power);
static int  sam_getcontrast(struct lcd_dev_s *dev);
static int  sam_setcontrast(struct lcd_dev_s *dev, unsigned int contrast);

/* Initialization */

static void sam_gpio_initialize(void);
static inline void sam_smc_initialize(void);
static void sam_lcd9341_initialize(void);
static inline int sam_lcd_initialize(void);

/************************************************************************************
 * Private Data
 ************************************************************************************/
/* LCD GPIO configurations */

static const uint32_t g_lcdpin[] =
{
  GPIO_SMC_D0,  GPIO_SMC_D1,  GPIO_SMC_D2,  GPIO_SMC_D3,      /* D0-D3 */
  GPIO_SMC_D4,  GPIO_SMC_D5,  GPIO_SMC_D6,  GPIO_SMC_D7,      /* D4-D7 */
  GPIO_SMC_NRD, GPIO_SMC_NWE, GPIO_SMC_A1,  GPIO_SMC_NCS1_2,  /* RD, WR, RS, CS */
  GPIO_LCD_BKL                                                /* Backlight control */
};

#define LCD_NPINS (sizeof(g_lcdpin) / sizeof(uint32_t))

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

static uint16_t g_runbuffer[SAM_XRES];

/* This structure describes the overall LCD video controller */

static const struct fb_videoinfo_s g_videoinfo =
{
  .fmt     = SAM_COLORFMT,         /* Color format: RGB16-565: RRRR RGGG GGGB BBBB */
  .xres    = SAM_XRES,             /* Horizontal resolution in pixel columns */
  .yres    = SAM_YRES,             /* Vertical resolution in pixel rows */
  .nplanes = 1,                    /* Number of color planes supported */
};

/* This is the standard, NuttX Plane information object */

static const struct lcd_planeinfo_s g_planeinfo =
{
  .putrun = sam_putrun,            /* Put a run into LCD memory */
  .getrun = sam_getrun,            /* Get a run from LCD memory */
  .buffer = (uint8_t*)g_runbuffer, /* Run scratch buffer */
  .bpp    = SAM_BPP,               /* Bits-per-pixel */
};

/* This is the standard, NuttX LCD driver object */

static struct sam_dev_s g_lcddev =
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

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name:  sam_putreg
 *
 * Description:
 *   Write to a multi-byte ILI9341 register
 *
 ************************************************************************************/

static void sam_putreg(uint8_t regaddr, FAR const uint8_t *buffer, unsigned int buflen)
{
  LCD_INDEX = 0;
  LCD_INDEX = regaddr;

  /* Write the multi-byte register value */

  for (; buflen > 0; buflen--) 
    {
      LCD_DATA = *buffer++;
    }
}

/************************************************************************************
 * Name:  sam_getreg
 *
 * Description:
 *   Read from a multi-byte ILI9341 register
 *
 ************************************************************************************/

static void sam_getreg(uint8_t regaddr, FAR uint8_t *buffer, unsigned int buflen)
{
  LCD_INDEX = 0;
  LCD_INDEX = regaddr;

  /* Read the multi-byte register value */

  for (; buflen > 0; buflen--) 
    {
      *buffer++ = LCD_DATA;
    }
}

/************************************************************************************
 * Name:  sam_setwindow
 *
 * Description:
 *   Setup drawing window
 *
 ************************************************************************************/

static void sam_setwindow(sam_color_t row, sam_color_t col,
                          sam_color_t width, sam_color_t height)
{
  uint8_t buffer[4];

  lcdvdbg("row=%d col=%d width=%d height=%d\n", row, col, width, height);

  /* Set Column Address Position */

  buffer[0] = (col >> 8) & 0xff;
  buffer[1] = col & 0xff;
  buffer[2] = ((col + width - 1) >> 8) & 0xff;
  buffer[3] = (col + width - 1) & 0xff;
  sam_putreg(ILI9341_COLUMN_ADDRESS_SET, buffer, 4);

  /* Set Page Address Position */

  buffer[0] = (row >> 8) & 0xff;
  buffer[1] = row & 0xff;
  buffer[2] = ((row + height - 1) >> 8) & 0xff;
  buffer[3] = (row + height - 1) & 0xff;
  sam_putreg(ILI9341_PAGE_ADDRESS_SET, buffer, 4);
}

/************************************************************************************
 * Name:  sam_gram_wrprepare
 *
 * Description:
 *   Setup to write multiple pixels to the GRAM memory
 *
 ************************************************************************************/

static inline void sam_gram_wrprepare(void)
{
  /* Memory write command */

  LCD_INDEX = ILI9341_MEMORY_WRITE;
  LCD_INDEX = 0;
  LCD_INDEX = ILI9341_WRITE_MEMORY_CONTINUE;
}

/************************************************************************************
 * Name:  sam_gram_rdprepare
 *
 * Description:
 *   Setup to read multiple pixels from the GRAM memory
 *
 ************************************************************************************/

static inline void sam_gram_rdprepare(void)
{
  /* Write Data to GRAM */

  LCD_INDEX = 0;
  LCD_INDEX = ILI9341_MEMORY_READ;
}

/************************************************************************************
 * Name:  sam_gram_write
 *
 * Description:
 *   Write one pixel to the GRAM memory
 *
 ************************************************************************************/

static inline void sam_gram_write(sam_color_t color)
{
  LCD_DATA = RGB_RED(color);
  LCD_DATA = RGB_GREEN(color);
  LCD_DATA = RGB_BLUE(color);
}

/************************************************************************************
 * Name:  sam_gram_read
 *
 * Description:
 *   Read one 16-bit pixel to the GRAM memory
 *
 ************************************************************************************/

static inline sam_color_t sam_gram_read(void)
{
  uint8_t buffer[3];

  buffer[0] = LCD_DATA;       /* Dummy read */
  buffer[0] = LCD_DATA;       /* R */
  buffer[1] = LCD_DATA;       /* G */
  buffer[2] = LCD_DATA;       /* B */

  /* Return the converted color */

  return RGB_COLOR((sam_color_t)buffer[0], (sam_color_t)buffer[1],
                   (sam_color_t)buffer[2]);
}

/************************************************************************************
 * Name:  sam_dumprun
 *
 * Description:
 *   Dump the contexts of the run buffer:
 *
 *  run     - The buffer in containing the run read to be dumped
 *  npixels - The number of pixels to dump
 *
 ************************************************************************************/

#if 0 /* Sometimes useful */
static void sam_dumprun(FAR const char *msg, FAR uint16_t *run, size_t npixels)
{
  int i, j;

  syslog(LOG_DEBUG, "\n%s:\n", msg);
  for (i = 0; i < npixels; i += 16)
    {
      up_putc(' ');
      syslog(LOG_DEBUG, " ");
      for (j = 0; j < 16; j++)
        {
          syslog(LOG_DEBUG, " %04x", *run++);
        }

      up_putc('\n');
    }
}
#endif

/************************************************************************************
 * Name:  sam_disable_backlight
 *
 * Description:
 *   Turn the backlight off.
 *
 ************************************************************************************/

static void sam_disable_backlight(void)
{
  volatile int delay;

  sam_gpiowrite(GPIO_LCD_BKL, false);
  for (delay = 0; delay < BKL_DISABLE_DURATION; delay++);
}

/************************************************************************************
 * Name:  sam_set_backlight
 *
 * Description:
 *   The the backlight to the level associated with the specified power value.
 *
 ************************************************************************************/

static void sam_set_backlight(unsigned int power)
{
  volatile int delay;
  unsigned int level;
  int i;

  lcdvdbg("power=%d\n", power);

  /* Scale the power setting to the range 1...BKL_LEVELS */

  DEBUGASSERT(power > 0 && power <= CONFIG_LCD_MAXPOWER);
  level = (power * BKL_LEVELS) / CONFIG_LCD_MAXPOWER;
  if (level < 1)
    {
      level = 1;
    }

  level = BKL_LEVELS - level + 1;

  /* Set the new backlight level */

  for (i = 0; i < level; i++)
    {
      /* Generate a pulse to the charge pump */

      sam_gpiowrite(GPIO_LCD_BKL, false);
      for (delay = 0; delay < BKL_PULSE_DURATION; delay++);

      sam_gpiowrite(GPIO_LCD_BKL, true);
      for (delay = 0; delay < BKL_PULSE_DURATION; delay++);
    }

  /* Lock in this level */

  for (delay = 0; delay < BKL_ENABLE_DURATION; delay++);
}

/************************************************************************************
 * Name:  sam_poweroff
 *
 * Description:
 *   Enable/disable LCD panel power (0: full off - CONFIG_LCD_MAXPOWER: full on). On
 *   backlit LCDs, this setting may correspond to the backlight setting.
 *
 ************************************************************************************/

static int sam_poweroff(FAR struct sam_dev_s *priv)
{
  lcdvdbg("OFF\n");

  /* Turn the display off */

   sam_putreg(ILI9341_DISPLAY_OFF, NULL, 0);

  /* Disable the backlight */

  sam_disable_backlight();

  /* Remember the power off state */

  priv->power = 0;
  return OK;
}

/************************************************************************************
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
 ************************************************************************************/

static int sam_putrun(fb_coord_t row, fb_coord_t col, FAR const uint8_t *buffer,
                       size_t npixels)
{
#if defined(CONFIG_SAM4EEK_LCD_RGB565)
  FAR const uint16_t *src = (FAR const uint16_t*)buffer;
#elif defined(CONFIG_SAM4EEK_LCD_RGB24)
  FAR const uint8_t  *src = (FAR const uint8_t*)buffer;
#elif defined(CONFIG_SAM4EEK_LCD_RGB32)
  FAR const uint32_t *src = (FAR const uint32_t*)buffer;
#endif

  /* Buffer must be provided and aligned to a 16-bit address boundary */

  lcdvdbg("row: %d col: %d npixels: %d\n", row, col, npixels);

#if defined(CONFIG_SAM4EEK_LCD_RGB565)
  DEBUGASSERT(src && ((uintptr_t)src & 1) == 0);
#elif defined(CONFIG_SAM4EEK_LCD_RGB24)
  DEBUGASSERT(src);
#elif defined(CONFIG_SAM4EEK_LCD_RGB32)
  DEBUGASSERT(src && ((uintptr_t)src & 3) == 0);
#endif

  /* Determine the refresh window area */

  sam_setwindow(row, col, npixels, 1);

  /* Prepare to write in GRAM */

  sam_gram_wrprepare();

  /* Write the run into GRAM memory */

  while (npixels-- > 0)
    {
      sam_gram_write(*src++);
    }

  /* Reset the refresh window area */

  sam_setwindow(0, 0, SAM_XRES, SAM_YRES);
  return OK;
}

/************************************************************************************
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
 ************************************************************************************/

static int sam_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t *buffer,
                      size_t npixels)
{
#if defined(CONFIG_SAM4EEK_LCD_RGB565)
  FAR uint16_t *dest = (FAR uint16_t*)buffer;
#elif defined(CONFIG_SAM4EEK_LCD_RGB24)
  FAR uint8_t  *dest = (FAR uint8_t*)buffer;
#elif defined(dest)
  FAR uint32_t *dest = (FAR uint32_t*)buffer;
#endif

  /* Buffer must be provided and aligned to a 16-bit address boundary */

  lcdvdbg("row: %d col: %d npixels: %d\n", row, col, npixels);

#if defined(CONFIG_SAM4EEK_LCD_RGB565)
  DEBUGASSERT(dest && ((uintptr_t)dest & 1) == 0);
#elif defined(CONFIG_SAM4EEK_LCD_RGB24)
  DEBUGASSERT(dest);
#elif defined(CONFIG_SAM4EEK_LCD_RGB32)
  DEBUGASSERT(dest && ((uintptr_t)dest & 3) == 0);
#endif

  /* Determine the refresh window area */

  sam_setwindow(row, col, npixels, 1);

  /* Prepare to read GRAM data */

  sam_gram_rdprepare();

  /* Write the run into GRAM memory */

  while (npixels-- > 0)
    {
      *dest++ = sam_gram_read();
    }

  /* Reset the refresh window area */

  sam_setwindow(0, 0, SAM_XRES, SAM_YRES);
  return OK;
}

/************************************************************************************
 * Name:  sam_getvideoinfo
 *
 * Description:
 *   Get information about the LCD video controller configuration.
 *
 ************************************************************************************/

static int sam_getvideoinfo(FAR struct lcd_dev_s *dev,
                            FAR struct fb_videoinfo_s *vinfo)
{
  DEBUGASSERT(dev && vinfo);
  lcdvdbg("fmt: %d xres: %d yres: %d nplanes: %d\n",
          g_videoinfo.fmt, g_videoinfo.xres, g_videoinfo.yres, g_videoinfo.nplanes);
  memcpy(vinfo, &g_videoinfo, sizeof(struct fb_videoinfo_s));
  return OK;
}

/************************************************************************************
 * Name:  sam_getplaneinfo
 *
 * Description:
 *   Get information about the configuration of each LCD color plane.
 *
 ************************************************************************************/

static int sam_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int planeno,
                              FAR struct lcd_planeinfo_s *pinfo)
{
  DEBUGASSERT(dev && pinfo && planeno == 0);
  lcdvdbg("planeno: %d bpp: %d\n", planeno, g_planeinfo.bpp);
  memcpy(pinfo, &g_planeinfo, sizeof(struct lcd_planeinfo_s));
  return OK;
}

/************************************************************************************
 * Name:  sam_getpower
 *
 * Description:
 *   Get the LCD panel power status (0: full off - CONFIG_LCD_MAXPOWER: full on). On
 *   backlit LCDs, this setting may correspond to the backlight setting.
 *
 ************************************************************************************/

static int sam_getpower(struct lcd_dev_s *dev)
{
  FAR struct sam_dev_s *priv = (FAR struct sam_dev_s *)dev;

  lcdvdbg("power: %d\n", 0);
  return priv->power;
}

/************************************************************************************
 * Name:  sam_setpower
 *
 * Description:
 *   Enable/disable LCD panel power (0: full off - CONFIG_LCD_MAXPOWER: full on). On
 *   backlit LCDs, this setting may correspond to the backlight setting.
 *
 ************************************************************************************/

static int sam_setpower(struct lcd_dev_s *dev, int power)
{
  FAR struct sam_dev_s *priv = (FAR struct sam_dev_s *)dev;

  lcdvdbg("power: %d\n", power);
  DEBUGASSERT((unsigned)power <= CONFIG_LCD_MAXPOWER);

  /* Set new power level */

  if (power > 0)
    {
      /* Then turn the display on */

      sam_putreg(ILI9341_DISPLAY_ON, NULL, 0);

      /* Set the backlight level */

      sam_set_backlight((unsigned int)power);
      up_mdelay(50);
      priv->power = power;
    }
  else
    {
      /* Turn the display off */

      sam_poweroff(priv);
    }

  return OK;
}

/************************************************************************************
 * Name:  sam_getcontrast
 *
 * Description:
 *   Get the current contrast setting (0-CONFIG_LCD_MAXCONTRAST).
 *
 ************************************************************************************/

static int sam_getcontrast(struct lcd_dev_s *dev)
{
  lcdvdbg("Not implemented\n");
  return -ENOSYS;
}

/************************************************************************************
 * Name:  sam_setcontrast
 *
 * Description:
 *   Set LCD panel contrast (0-CONFIG_LCD_MAXCONTRAST).
 *
 ************************************************************************************/

static int sam_setcontrast(struct lcd_dev_s *dev, unsigned int contrast)
{
  lcdvdbg("contrast: %d\n", contrast);
  return -ENOSYS;
}

/************************************************************************************
 * Name:  sam_gpio_initialize
 *
 * Description:
 *   Configure LCD GPIO pins
 *
 ************************************************************************************/

static inline void sam_gpio_initialize(void)
{
  int i;

  /* Configure all LCD pins pins (backlight is initially off) */

  for (i = 0; i < LCD_NPINS; i++)
    {
      sam_configgpio(g_lcdpin[i]);
    }
}

/************************************************************************************
 * Name:  sam_smc_initialize
 *
 * Description:
 *   Configure LCD SMC interface
 *
 ************************************************************************************/

static inline void sam_smc_initialize(void)
{
  uintptr_t smcbase =  SAM_SMCCS_BASE(SAM_LCD_CS);
  uint32_t regval;

  /* Configure SMC interface for the LCD */

  regval = SMCCS_SETUP_NWESETUP(2) | SMCCS_SETUP_NCSWRSETUP(2) |
           SMCCS_SETUP_NRDSETUP(2) | SMCCS_SETUP_NCSRDSETUP(2);
  putreg32(regval, smcbase + SAM_SMCCS_SETUP_OFFSET);

  regval = SMCCS_PULSE_NWEPULSE(4)  | SMCCS_PULSE_NCSWRPULSE(4) |
           SMCCS_PULSE_NRDPULSE(10) | SMCCS_PULSE_NCSRDPULSE(10);
  putreg32(regval, smcbase + SAM_SMCCS_PULSE_OFFSET);

  regval = SMCCS_CYCLE_NWECYCLE(10) | SMCCS_CYCLE_NRDCYCLE(22);
  putreg32(regval, smcbase + SAM_SMCCS_CYCLE_OFFSET);

#ifdef SMCCS_MODE_DBW_8BITS /* SAM3U, SAM3X, SAM3A */
  regval = SMCCS_MODE_READMODE | SMCCS_MODE_WRITEMODE | SMCCS_MODE_DBW_8BITS;
#else
  regval = SMCCS_MODE_READMODE | SMCCS_MODE_WRITEMODE;
#endif
  putreg32(regval, smcbase + SAM_SMCCS_MODE_OFFSET);
}

/************************************************************************************
 * Name:  sam_lcd9341_initialize
 *
 * Description:
 *   Initialize the ILI9341 LCD.
 *
 ************************************************************************************/

static void sam_lcd9341_initialize(void)
{
  uint8_t buffer[5];

  /* Power control A configuration*/

  buffer[0] = 0x39;
  buffer[1] = 0x2C;
  buffer[2] = 0x00;
  buffer[3] = 0x34;
  buffer[4] = 0x02;
  sam_putreg(ILI9341_POWER_CONTROL_A, buffer, 5);

  /* Power control B configuration */

  buffer[0] = 0x00;
  buffer[1] = 0xaa;
  buffer[2] = 0xb0;
  sam_putreg(ILI9341_POWER_CONTROL_B, buffer, 3);

  /* Pump Ratio Control configuration */

  buffer[0] = 0x30;
  sam_putreg(ILI9341_PUMP_RATIO_CONTROL, buffer, 1);

  /* Power Control 1 configuration */

  buffer[0] = 0x25;
  sam_putreg(ILI9341_POWER_CONTROL_1, buffer, 1);

  /* Power Control 2 configuration */

  buffer[0] = 0x11;
  sam_putreg(ILI9341_POWER_CONTROL_2, buffer, 1);

  /* VOM Control 1 configuration */

  buffer[0] = 0x5C;
  buffer[1] = 0x4C;
  sam_putreg(ILI9341_VCOM_CONTROL_1, buffer, 2);

  /* VOM control 2 configuration */

  buffer[0] = 0x94;
  sam_putreg(ILI9341_VCOM_CONTROL_2, buffer, 1);

  /* Driver Timing Control A configuration */

  buffer[0] = 0x85;
  buffer[1] = 0x01;
  buffer[2] = 0x78;
  sam_putreg(ILI9341_DRIVER_TIMING_CTL_A, buffer, 3);

  /* Driver Timing Control B configuration */

  buffer[0] = 0x00;
  buffer[1] = 0x00;
  sam_putreg(ILI9341_DRIVER_TIMING_CTL_B, buffer, 2);

  /* Memory Access Control configuration */

#if defined(CONFIG_LCD_LANDSCAPE)
  /* Horizontal refresh order (MH): 0
   * RGB/BGR order (BGR)          : 1
   * Vertical refresh order (ML)  : 0
   * Row/column exchange (MV)     : 1
   * Column address order (MX)    : 0
   * Row address order (MY)       : 1
   */

  buffer[0] = ILI9341_MEMORY_ACCESS_CONTROL_BGR |
              ILI9341_MEMORY_ACCESS_CONTROL_MV;

#elif defined(CONFIG_LCD_PORTRAIT)
  /* Horizontal refresh order (MH): 0
   * RGB/BGR order (BGR)          : 1
   * Vertical refresh order (ML)  : 0
   * Row/column exchange (MV)     : 0
   * Column address order (MX)    : 0
   * Row address order (MY)       : 0
   */

  buffer[0] = ILI9341_MEMORY_ACCESS_CONTROL_BGR;

#elif defined(CONFIG_LCD_RLANDSCAPE)
  /* Horizontal refresh order (MH): 0
   * RGB/BGR order (BGR)          : 1
   * Vertical refresh order (ML)  : 0
   * Row/column exchange (MV)     : 1
   * Column address order (MX)    : 1
   * Row address order (MY)       : 0
   */

  buffer[0] = ILI9341_MEMORY_ACCESS_CONTROL_BGR |
              ILI9341_MEMORY_ACCESS_CONTROL_MV |
              ILI9341_MEMORY_ACCESS_CONTROL_MX;

#elif defined(CONFIG_LCD_RPORTRAIT)
  /* Horizontal refresh order (MH): 0
   * RGB/BGR order (BGR)          : 1
   * Vertical refresh order (ML)  : 0
   * Row/column exchange (MV)     : 1
   * Column address order (MX)    : 0
   * Row address order (MY)       : 1
   */

  buffer[0] = ILI9341_MEMORY_ACCESS_CONTROL_BGR |
              ILI9341_MEMORY_ACCESS_CONTROL_MX |
              ILI9341_MEMORY_ACCESS_CONTROL_MY;

#endif

  sam_putreg(ILI9341_MEMORY_ACCESS_CONTROL, buffer, 1);

  /* Colmod Pixel Format Set configuration */

  buffer[0] = 0x06;
  sam_putreg(ILI9341_PIXEL_FORMAT_SET, buffer, 1);

  /* Display Function Control */

  buffer[0] = 0x02;
  buffer[1] = 0x82;
  buffer[2] = 0x27;
  buffer[3] = 0x00;
  sam_putreg(ILI9341_DISPLAY_FUNCTION_CTL, buffer, 4);
    
  /* Set window area*/

  sam_setwindow(0, 0, SAM_XRES, SAM_YRES);

  /* Leave sleep mode*/

  sam_putreg(ILI9341_SLEEP_OUT, buffer, 0);
  up_mdelay(10);

  /* Initial state: LCD off */

  sam_putreg(ILI9341_DISPLAY_OFF, buffer, 0);
}

/************************************************************************************
 * Name:  sam_lcd_initialize
 *
 * Description:
 *   Initialize the LCD panel
 *
 ************************************************************************************/

static inline int sam_lcd_initialize(void)
{
  uint8_t buffer[4];
  uint16_t id;

  /* Check the LCD ID */

  sam_getreg(ILI9341_READ_ID4, buffer, 4);

  id = ((uint16_t)buffer[2] << 8) | (uint16_t)buffer[3];
  if (id != ILI9341_DEVICE_CODE)
    {
      lcddbg("ERROR: Unsupported LCD: %04x\n", id);
      return -ENODEV;
    }

  sam_lcd9341_initialize();
  return OK;
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name:  board_lcd_initialize
 *
 * Description:
 *   Initialize the LCD video hardware.  The initial state of the LCD is fully
 *   initialized, display memory cleared, and the LCD ready to use, but with the
 *   power setting at 0 (full off).
 *
 ************************************************************************************/

int board_lcd_initialize(void)
{
  FAR struct sam_dev_s *priv = &g_lcddev;
  int ret;

  lcdvdbg("Initializing\n");

  /* Configure all LCD pins pins (backlight is initially off) */

  sam_gpio_initialize();

  /* Enable peripheral clock */

  sam_smc_enableclk();

  /* Configure SMC interface for the LCD */

  sam_smc_initialize();

  /* Identify and configure the LCD */

  up_mdelay(50);
  ret = sam_lcd_initialize();
  if (ret == OK)
    {
      /* Clear the display (setting it to the color 0=black) */

      sam_lcdclear(CONFIG_SAM4EEK_LCD_BGCOLOR);

      /* Turn the display off */

      sam_poweroff(priv);
    }

  return ret;
}

/************************************************************************************
 * Name: board_lcd_getdev
 *
 * Description:
 *   Return a a reference to the LCD object for the specified LCD.  This allows
 *   support for multiple LCD devices.
 *
 ************************************************************************************/

FAR struct lcd_dev_s *board_lcd_getdev(int lcddev)
{
  DEBUGASSERT(lcddev == 0);
  return &g_lcddev.dev;
}

/************************************************************************************
 * Name:  board_lcd_uninitialize
 *
 * Description:
 *   Uninitialize the LCD support
 *
 ************************************************************************************/

void board_lcd_uninitialize(void)
{
  FAR struct sam_dev_s *priv = &g_lcddev;

  /* Put the LCD in the lowest possible power state */

  sam_poweroff(priv);
}

/************************************************************************************
 * Name:  sam_lcdclear
 *
 * Description:
 *   This is a non-standard LCD interface just for the SAM4E-EK board.  Because
 *   of the various rotations, clearing the display in the normal way by writing a
 *   sequences of runs that covers the entire display can be very slow.  Here the
 *   display is cleared by simply setting all GRAM memory to the specified color.
 *
 ************************************************************************************/

#if defined(CONFIG_SAM4EEK_LCD_RGB565)
void sam_lcdclear(uint16_t color)
#else /* if defined(CONFIG_SAM4EEK_LCD_RGB24) defined(CONFIG_SAM4EEK_LCD_RGB32) */
void sam_lcdclear(uint32_t color)
#endif
{
  unsigned long i;

  sam_setwindow(0, 0, SAM_XRES, SAM_YRES);
  sam_gram_wrprepare();

  for (i = SAM_XRES * SAM_YRES; i > 0; i--)
    {
      sam_gram_write(color);
    }
}

#endif /* CONFIG_LCD */
