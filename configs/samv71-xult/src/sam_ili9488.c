/************************************************************************************
 * configs/samv71-xult/src/sam_ili9488.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 * - This driver is a modification of the SAMA4E ILI9341 LCD driver.
 * - Atmel ILI93241 Sample code for the SAM4E
 * - Atmel ILI9488 Sample code for the SAMV71
 *
 * Some the LCD and SMC initialization logic comes from Atmel sample code for the
 * SAMV7.  The Atmel sample code has two-clause BSD-like license which does not
 * require this copyright statement, but here it is anyway:
 *
 *   Copyright (c) 2014, Atmel Corporation
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
 * 3. Neither the name NuttX, Atmel, nor the names of contributors may be
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

/* maXTouch Xplained Pro Xplained Pro LCD Connector *********************************
 *
 * Only the RGB is supported by this BSP (via SMC/EBI).  The switch mode
 * selector on the back of the maXtouch should be set in the OFF-ON-OFF
 * positions to select 16-bit color mode.
 *
 * ----------------- ------------- --------------------------------------------------
 *        LCD            SAMV71    Description
 * Pin  Function     Pin  Function
 * ---- ------------ ---- -------- --------------------------------------------------
 *  1   ID            -    -       Chip ID communication line
 *  2   GND           -   GND      Ground
 *  3   D0           PC0  D0       Data line
 *  4   D1           PC1  D1       Data line
 *  5   D2           PC2  D2       Data line
 *  6   D3           PC3  D3       Data line
 *  7   GND           -   GND      Ground
 *  8   D4           PC4  D4       Data line
 *  9   D5           PC5  D5       Data line
 * 10   D6           PC6  D6       Data line
 * 11   D7           PC7  D7       Data line
 * 12   GND           -   GND      Ground
 * 13   D8           PE0  D8       Data line
 * 14   D9           PE1  D9       Data line
 * 15   D10          PE2  D10      Data line
 * 16   D11          PE3  D11      Data line
 * 17   GND           -   GND      Ground
 * 18   D12          PE4  D12      Data line
 * 19   D13          PE5  D13      Data line
 * 20   D14          PA15 D14      Data line
 * 21   D15          PA16 D15      Data line
 * 22   GND           -   GND      Ground
 * 23   D16           -    -       Data line
 * 24   D17           -    -       Data line
 * 25   N/C           -    -
 * 26   N/C           -    -
 * 27   GND           -   GND      Ground
 * 28   N/C           -    -
 * 29   N/C           -    -
 * 30   N/C           -    -
 * 31   N/C           -    -
 * 32   GND           -   GND      Ground
 * 33   PCLK/        PC30 GPIO     RGB: Pixel clock Display RAM select.
 *      CMD_DATA_SEL               MCU: One address line of the MCU for displays where it
 *                                      is possible to select either the register or the
 *                                      data interface
 * 34   VSYNC/CS     PD19 NCS3     RGB: Vertical synchronization.
 *                                 MCU: Chip select
 * 35   HSYNC/WE     PC8  NWE      RGB: Horizontal synchronization
 *                                 MCU: Write enable signal
 * 36   DATA ENABLE/ PC11 NRD      RGB: Data enable signal
 *      RE                         MCU: Read enable signal
 * 37   SPI SCK       -    -       MCU: Clock for SPI
 * 38   SPI MOSI      -    -       MCU: Master out slave in line of SPI
 * 39   SPI MISO      -    -       MCU: Master in slave out line of SPI
 * 40   SPI SS        -    -       MCU: Slave select for SPI
 * 41   N/C           -    -
 * 42   TWI SDA      PA3  TWD0     I2C data line (maXTouch®)
 * 43   TWI SCL      PA4  TWCK0    I2C clock line (maXTouch)
 * 44   IRQ1         PD28 WKUP5    maXTouch interrupt line
 * 45   N/C          PA2  WKUP2
 * 46   PWM          PC9  TIOB7    Backlight control
 * 47   RESET        PC13 GPIO     Reset for both display and maxTouch
 * 48   VCC           -    -       3.3V power supply for extension board
 * 49   VCC           -    -       3.3V power supply for extension board
 * 50   GND           -    -       Ground
 * ---- ------------ ---- -------- --------------------------------------------------
 *
 ************************************************************************************/

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
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/ili9488.h>
#include <nuttx/video/rgbcolors.h>

#include <arch/irq.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "sam_gpio.h"
#include "sam_periphclks.h"
#include "chip/sam_pmc.h"
#include "chip/sam_smc.h"
#include "samv7-xult.h"

#ifdef CONFIG_LCD

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* SMC must be selected */

#if !defined(CONFIG_SAMV7_SMC)
#  error "CONFIG_SAMV7_SMC is required"
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
#  if defined(CONFIG_LCD_PORTAIT) || defined(CONFIG_LCD_RPORTAIT) || \
      defined(CONFIG_LCD_RLANDSCAPE)
#    error "Cannot define both portrait and any other orientations"
#  endif
#elif defined(CONFIG_LCD_RLANDSCAPE)
#  if defined(CONFIG_LCD_PORTAIT) || defined(CONFIG_LCD_RPORTAIT)
#    error "Cannot define both rportrait and any other orientations"
#  endif
#elif defined(CONFIG_LCD_PORTAIT)
#  ifdef CONFIG_LCD_RPORTAIT
#    error "Cannot define both landscape and any other orientations"
#  endif
#elif !defined(CONFIG_LCD_RPORTAIT)
#  define CONFIG_LCD_LANDSCAPE 1
#endif

/* Background color */

#if !defined(CONFIG_SAMV71XULT_LCD_BGCOLOR)
#  define CONFIG_SAMV71XULT_LCD_BGCOLOR 0
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
#  define SAM_XRES            480
#  define SAM_YRES            320
#else
#  define SAM_XRES            320
#  define SAM_YRES            480
#endif

/* Color depth and format */

#if defined(CONFIG_SAMV71XULT_LCD_RGB565)
#  define SAM_BPP             16
#  define SAM_COLORFMT        FB_FMT_RGB16_565
#elif defined(CONFIG_SAMV71XULT_LCD_RGB24)
#  define SAM_BPP             24
#  define SAM_COLORFMT        FB_FMT_RGB24
#else /* if defined(CONFIG_SAMV71XULT_LCD_RGB32) -- without ALPHA */
#  define SAM_BPP             32
#  define SAM_COLORFMT        FB_FMT_RGB32
#endif

/* Color decoding macros */

#ifdef CONFIG_SAMV71XULT_LCD_RGB565
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

#define SAM_LCD_BASE          ((uintptr_t)SAM_EXTCS3_BASE)
#define LCD_INDEX             (*(volatile uint8_t *)(SAM_LCD_BASE))
#define LCD_DATA              (*(volatile uint8_t *)(SAM_LCD_BASE + 2))

/* LCD SMC chip select number to be set */

#define SAM_LCD_CS            3

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

#ifdef CONFIG_SAMV71XULT_LCD_RGB565
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
static void sam_lcd9488_initialize(void);
static inline int sam_lcd_initialize(void);

/************************************************************************************
 * Private Data
 ************************************************************************************/
/* LCD GPIO configurations */

static const uint32_t g_lcdpin[] =
{
  GPIO_SMC_D0,      GPIO_SMC_D1,      GPIO_SMC_D2,      GPIO_SMC_D3,  /* D0-D3 */
  GPIO_SMC_D4,      GPIO_SMC_D5,      GPIO_SMC_D6,      GPIO_SMC_D7,  /* D4-D7 */
  GPIO_SMC_D8,      GPIO_SMC_D9,      GPIO_SMC_D10,     GPIO_SMC_D11, /* D8-D11 */
  GPIO_SMC_D12,     GPIO_SMC_D13,     GPIO_SMC_D14,     GPIO_SMC_D15, /* D12-15 */
  GPIO_SMC_NWE,     GPIO_SMC_NRD,     GPIO_SMC_NCS3,                  /* RD, WR, CS */
  GPIO_ILI9488_CDS, GPIO_ILI9488_RST, GPIO_ILI9488_BKL                /* PIO outputs */
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
 *   Write to a multi-byte ILI9488 register
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
 *   Read from a multi-byte ILI9488 register
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
  sam_putreg(ILI9488_COLUMN_ADDRESS_SET, buffer, 4);

  /* Set Page Address Position */

  buffer[0] = (row >> 8) & 0xff;
  buffer[1] = row & 0xff;
  buffer[2] = ((row + height - 1) >> 8) & 0xff;
  buffer[3] = (row + height - 1) & 0xff;
  sam_putreg(ILI9488_PAGE_ADDRESS_SET, buffer, 4);
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

  LCD_INDEX = ILI9488_MEMORY_WRITE;
  LCD_INDEX = 0;
  LCD_INDEX = ILI9488_WRITE_MEMORY_CONTINUE;
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
  LCD_INDEX = ILI9488_MEMORY_READ;
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

  sam_gpiowrite(GPIO_ILI9488_BKL, false);
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

      sam_gpiowrite(GPIO_ILI9488_BKL, false);
      for (delay = 0; delay < BKL_PULSE_DURATION; delay++);

      sam_gpiowrite(GPIO_ILI9488_BKL, true);
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

   sam_putreg(ILI9488_DISPLAY_OFF, NULL, 0);

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
#if defined(CONFIG_SAMV71XULT_LCD_RGB565)
  FAR const uint16_t *src = (FAR const uint16_t*)buffer;
#elif defined(CONFIG_SAMV71XULT_LCD_RGB24)
  FAR const uint8_t  *src = (FAR const uint8_t*)buffer;
#elif defined(CONFIG_SAMV71XULT_LCD_RGB32)
  FAR const uint32_t *src = (FAR const uint32_t*)buffer;
#endif

  /* Buffer must be provided and aligned to a 16-bit address boundary */

  lcdvdbg("row: %d col: %d npixels: %d\n", row, col, npixels);

#if defined(CONFIG_SAMV71XULT_LCD_RGB565)
  DEBUGASSERT(src && ((uintptr_t)src & 1) == 0);
#elif defined(CONFIG_SAMV71XULT_LCD_RGB24)
  DEBUGASSERT(src);
#elif defined(CONFIG_SAMV71XULT_LCD_RGB32)
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
#if defined(CONFIG_SAMV71XULT_LCD_RGB565)
  FAR uint16_t *dest = (FAR uint16_t*)buffer;
#elif defined(CONFIG_SAMV71XULT_LCD_RGB24)
  FAR uint8_t  *dest = (FAR uint8_t*)buffer;
#elif defined(dest)
  FAR uint32_t *dest = (FAR uint32_t*)buffer;
#endif

  /* Buffer must be provided and aligned to a 16-bit address boundary */

  lcdvdbg("row: %d col: %d npixels: %d\n", row, col, npixels);

#if defined(CONFIG_SAMV71XULT_LCD_RGB565)
  DEBUGASSERT(dest && ((uintptr_t)dest & 1) == 0);
#elif defined(CONFIG_SAMV71XULT_LCD_RGB24)
  DEBUGASSERT(dest);
#elif defined(CONFIG_SAMV71XULT_LCD_RGB32)
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

      sam_putreg(ILI9488_DISPLAY_ON, NULL, 0);

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

  /* Backlight off */

  sam_gpiowrite(GPIO_ILI9488_BKL, true);
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

  /* Enable the SMC peripheral clock */

  sam_smc_enableclk();

  /* Configure SMC interface for the LCD on NCS3 */

  regval = SMCCS_SETUP_NWESETUP(2) | SMCCS_SETUP_NCSWRSETUP(0) |
           SMCCS_SETUP_NRDSETUP(0) | SMCCS_SETUP_NCSRDSETUP(0);
  putreg32(regval, smcbase + SAM_SMCCS_SETUP_OFFSET);

  regval = SMCCS_PULSE_NWEPULSE(6)  | SMCCS_PULSE_NCSWRPULSE(10) |
           SMCCS_PULSE_NRDPULSE(10) | SMCCS_PULSE_NCSRDPULSE(10);
  putreg32(regval, smcbase + SAM_SMCCS_PULSE_OFFSET);

  regval = SMCCS_CYCLE_NWECYCLE(10) | SMCCS_CYCLE_NRDCYCLE(10);
  putreg32(regval, smcbase + SAM_SMCCS_CYCLE_OFFSET);

  regval = SMCCS_MODE_READMODE | SMCCS_MODE_WRITEMODE | SMCCS_EXNWMODE_DISABLED |
           SMCCS_MODE_DBW_16BITS | SMCCS_MODE_TDFCYCLES(15);
  putreg32(regval, smcbase + SAM_SMCCS_MODE_OFFSET);
}

/************************************************************************************
 * Name:  sam_lcd9488_initialize
 *
 * Description:
 *   Initialize the ILI9488 LCD.
 *
 ************************************************************************************/

static void sam_lcd9488_initialize(void)
{
  uint8_t buffer[5];

  /* Power control A configuration*/

  buffer[0] = 0x39;
  buffer[1] = 0x2C;
  buffer[2] = 0x00;
  buffer[3] = 0x34;
  buffer[4] = 0x02;
  sam_putreg(ILI9488_POWER_CONTROL_A, buffer, 5);

  /* Power control B configuration */

  buffer[0] = 0x00;
  buffer[1] = 0xaa;
  buffer[2] = 0xb0;
  sam_putreg(ILI9488_POWER_CONTROL_B, buffer, 3);

  /* Pump Ratio Control configuration */

  buffer[0] = 0x30;
  sam_putreg(ILI9488_PUMP_RATIO_CONTROL, buffer, 1);

  /* Power Control 1 configuration */

  buffer[0] = 0x25;
  sam_putreg(ILI9488_POWER_CONTROL_1, buffer, 1);

  /* Power Control 2 configuration */

  buffer[0] = 0x11;
  sam_putreg(ILI9488_POWER_CONTROL_2, buffer, 1);

  /* VOM Control 1 configuration */

  buffer[0] = 0x5C;
  buffer[1] = 0x4C;
  sam_putreg(ILI9488_VCOM_CONTROL_1, buffer, 2);

  /* VOM control 2 configuration */

  buffer[0] = 0x94;
  sam_putreg(ILI9488_VCOM_CONTROL_2, buffer, 1);

  /* Driver Timing Control A configuration */

  buffer[0] = 0x85;
  buffer[1] = 0x01;
  buffer[2] = 0x78;
  sam_putreg(ILI9488_DRIVER_TIMING_CTL_A, buffer, 3);

  /* Driver Timing Control B configuration */

  buffer[0] = 0x00;
  buffer[1] = 0x00;
  sam_putreg(ILI9488_DRIVER_TIMING_CTL_B, buffer, 2);

  /* Memory Access Control configuration */

#if defined(CONFIG_LCD_LANDSCAPE)
  /* Horizontal refresh order (MH): 0
   * RGB/BGR order (BGR)          : 1
   * Vertical refresh order (ML)  : 0
   * Row/column exchange (MV)     : 1
   * Column address order (MX)    : 0
   * Row address order (MY)       : 1
   */

  buffer[0] = ILI9488_MEMORY_ACCESS_CONTROL_BGR |
              ILI9488_MEMORY_ACCESS_CONTROL_MV;

#elif defined(CONFIG_LCD_PORTRAIT)
  /* Horizontal refresh order (MH): 0
   * RGB/BGR order (BGR)          : 1
   * Vertical refresh order (ML)  : 0
   * Row/column exchange (MV)     : 0
   * Column address order (MX)    : 0
   * Row address order (MY)       : 0
   */

  buffer[0] = ILI9488_MEMORY_ACCESS_CONTROL_BGR;

#elif defined(CONFIG_LCD_RLANDSCAPE)
  /* Horizontal refresh order (MH): 0
   * RGB/BGR order (BGR)          : 1
   * Vertical refresh order (ML)  : 0
   * Row/column exchange (MV)     : 1
   * Column address order (MX)    : 1
   * Row address order (MY)       : 0
   */

  buffer[0] = ILI9488_MEMORY_ACCESS_CONTROL_BGR |
              ILI9488_MEMORY_ACCESS_CONTROL_MV |
              ILI9488_MEMORY_ACCESS_CONTROL_MX;

#elif defined(CONFIG_LCD_RPORTRAIT)
  /* Horizontal refresh order (MH): 0
   * RGB/BGR order (BGR)          : 1
   * Vertical refresh order (ML)  : 0
   * Row/column exchange (MV)     : 1
   * Column address order (MX)    : 0
   * Row address order (MY)       : 1
   */

  buffer[0] = ILI9488_MEMORY_ACCESS_CONTROL_BGR |
              ILI9488_MEMORY_ACCESS_CONTROL_MX |
              ILI9488_MEMORY_ACCESS_CONTROL_MY;

#endif

  sam_putreg(ILI9488_MEMORY_ACCESS_CONTROL, buffer, 1);

  /* Colmod Pixel Format Set configuration */

  buffer[0] = 0x06;
  sam_putreg(ILI9488_PIXEL_FORMAT_SET, buffer, 1);

  /* Display Function Control */

  buffer[0] = 0x02;
  buffer[1] = 0x82;
  buffer[2] = 0x27;
  buffer[3] = 0x00;
  sam_putreg(ILI9488_DISPLAY_FUNCTION_CTL, buffer, 4);
    
  /* Set window area*/

  sam_setwindow(0, 0, SAM_XRES, SAM_YRES);

  /* Leave sleep mode*/

  sam_putreg(ILI9488_SLEEP_OUT, buffer, 0);
  up_mdelay(10);

  /* Initial state: LCD off */

  sam_putreg(ILI9488_DISPLAY_OFF, buffer, 0);
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

  sam_getreg(ILI9488_READ_ID4, buffer, 4);

  id = ((uint16_t)buffer[2] << 8) | (uint16_t)buffer[3];
  if (id != ILI9488_DEVICE_CODE)
    {
      lcddbg("ERROR: Unsupported LCD: %04x\n", id);
      return -ENODEV;
    }

  sam_lcd9488_initialize();
  return OK;
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name:  up_lcdinitialize
 *
 * Description:
 *   Initialize the LCD video hardware.  The initial state of the LCD is fully
 *   initialized, display memory cleared, and the LCD ready to use, but with the
 *   power setting at 0 (full off).
 *
 ************************************************************************************/

int up_lcdinitialize(void)
{
  FAR struct sam_dev_s *priv = &g_lcddev;
  int ret;

  lcdvdbg("Initializing\n");

  /* Configure all LCD pins pins (backlight is initially off) */

  sam_gpio_initialize();

  /* Configure SMC interface for the LCD */

  sam_smc_initialize();

  /* Configure DMA */
#warning Missing logic

  /* Identify and configure the LCD */

  up_mdelay(50);
  ret = sam_lcd_initialize();
  if (ret == OK)
    {
      /* Clear the display (setting it to the color 0=black) */

      sam_lcdclear(CONFIG_SAMV71XULT_LCD_BGCOLOR);

      /* Turn the display off */

      sam_poweroff(priv);
    }

  return ret;
}

/************************************************************************************
 * Name: up_lcdgetdev
 *
 * Description:
 *   Return a a reference to the LCD object for the specified LCD.  This allows
 *   support for multiple LCD devices.
 *
 ************************************************************************************/

FAR struct lcd_dev_s *up_lcdgetdev(int lcddev)
{
  DEBUGASSERT(lcddev == 0);
  return &g_lcddev.dev;
}

/************************************************************************************
 * Name:  up_lcduninitialize
 *
 * Description:
 *   Uninitialize the LCD support
 *
 ************************************************************************************/

void up_lcduninitialize(void)
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

#if defined(CONFIG_SAMV71XULT_LCD_RGB565)
void sam_lcdclear(uint16_t color)
#else /* if defined(CONFIG_SAMV71XULT_LCD_RGB24) defined(CONFIG_SAMV71XULT_LCD_RGB32) */
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
