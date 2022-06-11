/****************************************************************************
 * boards/arm/sam34/sam4e-ek/src/sam_ili9325.c
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

/* References:
 * - This driver is a modification of the Shenzhou ILI9325 LCD driver.
 * - ILI9325 Datasheet, Version: V0.43, ILI9325DS_V0.43.pdf,
 *                  ILI TECHNOLOGY CORP.,
 * - SAM4Ex Datasheet, Atmel
 * - Atmel ILI9325 Sample code for the SAM4S
 */

/****************************************************************************
 *
 * The SAM4E-EK carries a TFT transmissive LCD module with touch panel,
 * FTM280C34D.
 * Its integrated driver IC is ILI9325. The LCD display area is 2.8 inches
 * diagonally measured, with a native resolution of 240 x 320 dots.
 *
 * The SAM4E16 communicates with the LCD through PIOC where an 8-bit parallel
 * "8080-like" protocol data bus has to be implemented in software.
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
 * LCD backlight is made of 4 white chip LEDs in parallel, driven by an
 * AAT3155 charge pump, MN4.
 * The AAT3155 is controlled by the SAM3U4E through a single line Simple
 * Serial Control (S2Cwire) interface, which permits to enable, disable, and
 * set the LED drive current (LED brightness control) from a 32-level
 * logarithmic scale.
 * Four resistors R93/R94/R95/R96 are implemented for optional current
 * limitation.
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
#include <nuttx/board.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/ili9325.h>
#include <nuttx/video/rgbcolors.h>

#include <arch/irq.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "sam_gpio.h"
#include "sam_periphclks.h"
#include "hardware/sam_pmc.h"
#include "hardware/sam_smc.h"
#include "sam4e-ek.h"

#ifdef CONFIG_LCD

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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

/* Background color */

#if !defined(CONFIG_SAM4EEK_LCD_BGCOLOR)
#  define CONFIG_SAM4EEK_LCD_BGCOLOR 0
#endif

/* Display/Color Properties *************************************************/

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
#  define RGB_RED(rgb)        (((rgb) >> 8)  & 0xf8)
#  define RGB_GREEN(rgb)      (((rgb) >> 3)  & 0xfc)
#  define RGB_BLUE(rgb)       (((rgb) << 3)  & 0xf8)
#else /* RGB888 or RGB32 without ALPHA */
#  define RGB_RED(rgb)        (((rgb) >> 16) & 0xff)
#  define RGB_GREEN(rgb)      (((rgb) >> 8)  & 0xff)
#  define RGB_BLUE(rgb)       ( (rgb)        & 0xff)
#endif

/* SAM4E-EK LCD Hardware Definitions ****************************************/

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

/****************************************************************************
 * Private Type Definition
 ****************************************************************************/

/* Type definition for the correct size of one pixel
 * (from the application standpoint).
 */

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

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Low Level LCD access */

static void sam_write_reg(uint8_t regaddr, uint16_t regval);
static uint16_t sam_read_reg(uint8_t regaddr);
static inline void sam_gram_prepare(void);
static inline void sam_gram_write(sam_color_t color);
static inline sam_color_t sam_gram_read(void);
static void sam_set_cursor(uint16_t col, uint16_t row);

/* Backlight/power controls */

static void sam_disable_backlight(void);
static void sam_set_backlight(unsigned int power);
static int sam_poweroff(struct sam_dev_s *priv);

/* LCD Data Transfer Methods */

static int sam_putrun(fb_coord_t row, fb_coord_t col,
                      const uint8_t *buffer,
                      size_t npixels);
static int sam_getrun(fb_coord_t row, fb_coord_t col,
                      uint8_t *buffer,
                      size_t npixels);

/* LCD Configuration */

static int sam_getvideoinfo(struct lcd_dev_s *dev,
             struct fb_videoinfo_s *vinfo);
static int sam_getplaneinfo(struct lcd_dev_s *dev,
             unsigned int planeno,
             struct lcd_planeinfo_s *pinfo);

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

/* Initialization */

static void sam_gpio_initialize(void);
static inline void sam_smc_initialize(void);
static void sam_lcd9325_initialize(void);
static inline int sam_lcd_initialize(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

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
  .putrun = sam_putrun,             /* Put a run into LCD memory */
  .getrun = sam_getrun,             /* Get a run from LCD memory */
  .buffer = (uint8_t *)g_runbuffer, /* Run scratch buffer */
  .bpp    = SAM_BPP,                /* Bits-per-pixel */
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

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  sam_write_reg
 *
 * Description:
 *   Write to an LCD register
 *
 ****************************************************************************/

static void sam_write_reg(uint8_t regaddr, uint16_t regval)
{
  LCD_INDEX = 0;
  LCD_INDEX = regaddr;

  /* Write the 16-bit register value */

  LCD_DATA  = (uint8_t)(regval >> 8);
  LCD_DATA  = (uint8_t)(regval & 0xff);
}

/****************************************************************************
 * Name:  sam_read_reg
 *
 * Description:
 *   Read from an LCD register
 *
 ****************************************************************************/

static uint16_t sam_read_reg(uint8_t regaddr)
{
  uint16_t regval;

  LCD_INDEX = 0;
  LCD_INDEX = regaddr;

  /* Read and return the 16-bit register contents */

  regval = (uint16_t)LCD_DATA;
  regval = (regval << 8) | (uint16_t)LCD_DATA;

  return regval;
}

/****************************************************************************
 * Name:  sam_gram_prepare
 *
 * Description:
 *   Setup to read or write multiple pixels to the GRAM memory
 *
 ****************************************************************************/

static inline void sam_gram_prepare(void)
{
  LCD_INDEX = 0;
  LCD_INDEX = ILI9325_GRAM_DATA_REG;
}

/****************************************************************************
 * Name:  sam_gram_write
 *
 * Description:
 *   Write one pixel to the GRAM memory
 *
 ****************************************************************************/

static inline void sam_gram_write(sam_color_t color)
{
  LCD_DATA = RGB_RED(color);
  LCD_DATA = RGB_GREEN(color);
  LCD_DATA = RGB_BLUE(color);
}

/****************************************************************************
 * Name:  sam_gram_read
 *
 * Description:
 *   Read one 16-bit pixel to the GRAM memory
 *
 ****************************************************************************/

static inline sam_color_t sam_gram_read(void)
{
  uint8_t value[2];

  value[0] = LCD_DATA;       /* Dummy read */
  value[1] = LCD_DATA;       /* Dummy read */
  value[0] = LCD_DATA;       /* RGB565 data upper byte */
  value[1] = LCD_DATA;       /* RGB565 data lower byte */

  /* Convert and transfer the color to the user buffer */

#if defined(CONFIG_SAM4EEK_LCD_RGB565)
  /* Return the raw RGB565 color */

  return (sam_color_t)value[0] << 8 | (sam_color_t)value[1];

#else /* if defined(CONFIG_SAM4EEK_LCD_RGB24) || defined(CONFIG_SAM4EEK_LCD_RGB32) */
      /* RRRR RGGG GGGB BBBB -> 0000 0000 RRRR R000 GGGG GG00 BBBB B000 */

  return ((value[0] & 0xf8)) |
         ((value[0] & 0x07) << 13) | ((value[1] & 0xe0) << 5) |
         ((value[1] & 0x1f) << 19);
#endif
}

/****************************************************************************
 * Name:  sam_set_cursor
 *
 * Description:
 *   Set the cursor position.
 *   In landscape mode, the "column" is actually the physical
 *   Y position and the "row" is the physical X position.
 *
 ****************************************************************************/

static void sam_set_cursor(uint16_t col, uint16_t row)
{
  sam_write_reg(ILI9325_HORIZONTAL_GRAM_ADDR_SET, row);
  sam_write_reg(ILI9325_VERTICAL_GRAM_ADDR_SET, col);
}

/****************************************************************************
 * Name:  sam_dumprun
 *
 * Description:
 *   Dump the contexts of the run buffer:
 *
 *  run     - The buffer in containing the run read to be dumped
 *  npixels - The number of pixels to dump
 *
 ****************************************************************************/

#if 0 /* Sometimes useful */
static void sam_dumprun(const char *msg, uint16_t *run,
                        size_t npixels)
{
  int i;
  int j;

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

/****************************************************************************
 * Name:  sam_disable_backlight
 *
 * Description:
 *   Turn the backlight off.
 *
 ****************************************************************************/

static void sam_disable_backlight(void)
{
  volatile int delay;

  sam_gpiowrite(GPIO_LCD_BKL, false);
  for (delay = 0; delay < BKL_DISABLE_DURATION; delay++);
}

/****************************************************************************
 * Name:  sam_set_backlight
 *
 * Description:
 *   The the backlight to the level associated with the specified power
 *   value.
 *
 ****************************************************************************/

static void sam_set_backlight(unsigned int power)
{
  volatile int delay;
  unsigned int level;
  int i;

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

/****************************************************************************
 * Name:  sam_poweroff
 *
 * Description:
 *   Enable/disable LCD panel power
 *  (0: full off - CONFIG_LCD_MAXPOWER: full on). On backlit LCDs,
 *   this setting may correspond to the backlight setting.
 *
 ****************************************************************************/

static int sam_poweroff(struct sam_dev_s *priv)
{
  /* Turn the display off */

  sam_write_reg(ILI9325_DISP_CTRL1, 0);

  /* Disable the backlight */

  sam_disable_backlight();

  /* Remember the power off state */

  priv->power = 0;
  return OK;
}

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
                      const uint8_t *buffer,
                      size_t npixels)
{
#if defined(CONFIG_SAM4EEK_LCD_RGB565)
  const uint16_t *src = (const uint16_t *)buffer;
#elif defined(CONFIG_SAM4EEK_LCD_RGB24)
  const uint8_t  *src = (const uint8_t *)buffer;
#elif defined(CONFIG_SAM4EEK_LCD_RGB32)
  const uint32_t *src = (const uint32_t *)buffer;
#endif

  /* Buffer must be provided and aligned to a 16-bit address boundary */

  lcdinfo("row: %d col: %d npixels: %d\n", row, col, npixels);
  DEBUGASSERT(buffer && ((uintptr_t)buffer & 1) == 0);

  /* Set the cursor position */

  sam_set_cursor(col, row);

  /* Prepare to write GRAM data */

  sam_gram_prepare();

  /* Then transfer the pixels as 3 8-bit transfers, each providing 6 bits of
   * the color component in the MS bits.
   */

#if defined(CONFIG_SAM4EEK_LCD_RGB565) || defined(CONFIG_SAM4EEK_LCD_RGB32)
  while (npixels--)
    {
      sam_color_t color = *src++;
      LCD_DATA = RGB_RED(color);
      LCD_DATA = RGB_GREEN(color);
      LCD_DATA = RGB_BLUE(color);
    }
#else /* defined(CONFIG_SAM4EEK_LCD_RGB24) */
  while (npixels--)
    {
      LCD_DATA      = *src++;
      LCD_DATA      = *src++;
      LCD_DATA      = *src++;
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

static int sam_getrun(fb_coord_t row, fb_coord_t col, uint8_t *buffer,
                      size_t npixels)
{
  uint8_t value[2];
#if defined(CONFIG_SAM4EEK_LCD_RGB24)
  uint8_t *ptr = (uint8_t *)buffer;
#endif
  /* Set the cursor position */

  sam_set_cursor(col, row);

  /* Prepare to write GRAM data */

  sam_gram_prepare();

  /* Then transfer the pixels, reading RGB565 and converting this to the
   * format expected by the caller.
   */

  while (npixels--)
    {
      /* Read the RGB565 colot */

      value[0] = LCD_DATA;       /* Dummy read */
      value[1] = LCD_DATA;       /* Dummy read */
      value[0] = LCD_DATA;       /* Data upper byte */
      value[1] = LCD_DATA;       /* Data lower byte */

      /* Convert and transfer the color to the user buffer */

#if defined(CONFIG_SAM4EEK_LCD_RGB565)

      /* Return the raw RGB565 color */

      *buffer++ = (sam_color_t)value[0] << 8 | (sam_color_t)value[1];

#elif defined(CONFIG_SAM4EEK_LCD_RGB24)

      /* RRRR RGGG GGGB BBBB -> RRRR R000, GGGG GG00, BBBB B000 */

      *ptr++ = (value[0] & 0xf8);
      *ptr++ = ((value[0] & 0x07) << 5) | ((value[1] & 0xe0) >> 3);
      *ptr++ = (value[1] & 0x1f) << 3;

#else /* if defined(CONFIG_SAM4EEK_LCD_RGB32) */

      /* RRRR RGGG GGGB BBBB -> 0000 0000 RRRR R000 GGGG GG00 BBBB B000 */

      *buffer++ = ((value[0] & 0xf8)) |
                  ((value[0] & 0x07) << 13) | ((value[1] & 0xe0) << 5) |
                  ((value[1] & 0x1f) << 19);
#endif
    }

  return OK;
}

/****************************************************************************
 * Name:  sam_getvideoinfo
 *
 * Description:
 *   Get information about the LCD video controller configuration.
 *
 ****************************************************************************/

static int sam_getvideoinfo(struct lcd_dev_s *dev,
                            struct fb_videoinfo_s *vinfo)
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

static int sam_getplaneinfo(struct lcd_dev_s *dev, unsigned int planeno,
                              struct lcd_planeinfo_s *pinfo)
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
 *   Get the LCD panel power status
 *   (0: full off - CONFIG_LCD_MAXPOWER: full on). On backlit LCDs,
 *   this setting may correspond to the backlight setting.
 *
 ****************************************************************************/

static int sam_getpower(struct lcd_dev_s *dev)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;

  lcdinfo("power: %d\n", 0);
  return priv->power;
}

/****************************************************************************
 * Name:  sam_setpower
 *
 * Description:
 *   Enable/disable LCD panel power
 *   (0: full off - CONFIG_LCD_MAXPOWER: full on). On backlit LCDs,
 *   this setting may correspond to the backlight setting.
 *
 ****************************************************************************/

static int sam_setpower(struct lcd_dev_s *dev, int power)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;

  lcdinfo("power: %d\n", power);
  DEBUGASSERT((unsigned)power <= CONFIG_LCD_MAXPOWER);

  /* Set new power level */

  if (power > 0)
    {
      /* Then turn the display on */

      sam_write_reg(ILI9325_DISP_CTRL1,
                    ILI9325_DISP_CTRL1_BASEE | ILI9325_DISP_CTRL1_GON |
                    ILI9325_DISP_CTRL1_DTE | ILI9325_DISP_CTRL1_D(3));

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
 * Name:  sam_setcontrast
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
 * Name:  sam_gpio_initialize
 *
 * Description:
 *   Configure LCD GPIO pins
 *
 ****************************************************************************/

static inline void sam_gpio_initialize(void)
{
  int i;

  /* Configure all LCD pins pins (backlight is initially off) */

  for (i = 0; i < LCD_NPINS; i++)
    {
      sam_configgpio(g_lcdpin[i]);
    }
}

/****************************************************************************
 * Name:  sam_smc_initialize
 *
 * Description:
 *   Configure LCD SMC interface
 *
 ****************************************************************************/

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

/****************************************************************************
 * Name:  sam_lcd9325_initialize
 *
 * Description:
 *   Initialize the ILI9325 LCD.
 *
 ****************************************************************************/

static void sam_lcd9325_initialize(void)
{
  uint16_t regval;

  /* Turn off the LCD *******************************************************/

  sam_write_reg(ILI9325_DISP_CTRL1,
                ILI9325_DISP_CTRL1_GON | ILI9325_DISP_CTRL1_DTE |
                ILI9325_DISP_CTRL1_D(3));

  /* Initial sequence *******************************************************/

  /* Disable sleep and standby mode */

  sam_write_reg(ILI9325_POWER_CTRL1, 0);

  /* Start internal OSC */

  sam_write_reg(ILI9325_START_OSC_CTRL, ILI9325_START_OSC_CTRL_EN);

  /* Set SS bit and direction output from S720 to S1 */

  sam_write_reg(ILI9325_DRIVER_OUTPUT_CTRL1, ILI9325_DRIVER_OUTPUT_CTRL1_SS);

  /* Set 1 line inversion */

  sam_write_reg(ILI9325_LCD_DRIVING_CTRL,
                ILI9325_LCD_DRIVING_CTRL_BIT10 |
                ILI9325_LCD_DRIVING_CTRL_EOR |
                ILI9325_LCD_DRIVING_CTRL_BC0);

  /* Disable resizing feature */

  sam_write_reg(ILI9325_RESIZE_CTRL, 0);

  /* Set the back porch and front porch */

  sam_write_reg(ILI9325_DISP_CTRL2,
                ILI9325_DISP_CTRL2_BP(7) | ILI9325_DISP_CTRL2_FP(2));

  /* Set non-display area refresh cycle ISC[3:0] */

  sam_write_reg(ILI9325_DISP_CTRL3, 0);

  /* Disable FMARK function */

  sam_write_reg(ILI9325_DISP_CTRL4, 0);

  /* 18-bit RGB interface and writing display data by the system interface */

  sam_write_reg(ILI9325_RGB_DISP_INTERFACE_CTRL1, 0);

  /* Set the output position of frame cycle */

  sam_write_reg(ILI9325_FRAME_MAKER_SHIFT, 0);

  /* RGB interface polarity */

  sam_write_reg(ILI9325_RGB_DISP_INTERFACE_CTRL2, 0);

  /* Power on sequence ******************************************************/

  /* Disable sleep and standby mode */

  sam_write_reg(ILI9325_POWER_CTRL1, 0);

  /* Select the operating frequency of the step-up circuit 1,2 and set the
   * ratio factor of Vci
   */

  sam_write_reg(ILI9325_POWER_CTRL2, 0);

  /* Set VREG1OUT voltage */

  sam_write_reg(ILI9325_POWER_CTRL3, 0);

  /* Set VCOM amplitude */

  sam_write_reg(ILI9325_POWER_CTRL4, 0);
  up_mdelay(200);

  /* Enable power supply and source driver **********************************/

  /* Adjust the constant current and set the factor used in the step-up
   * circuits.
   */

  sam_write_reg(ILI9325_POWER_CTRL1,
                ILI9325_POWER_CTRL1_SAP | ILI9325_POWER_CTRL1_BT(2) |
                ILI9325_POWER_CTRL1_APE | ILI9325_POWER_CTRL1_AP(1));

  /* Select the operating frequency of the step-up circuit 1,2 and set the
   * ratio factor of Vci
   */

  sam_write_reg(ILI9325_POWER_CTRL2,
                ILI9325_POWER_CTRL2_DC1(2) | ILI9325_POWER_CTRL2_DC0(2) |
                ILI9325_POWER_CTRL2_VC(7));
  up_mdelay(50);

  /* Internal reference voltage= Vci */

  sam_write_reg(ILI9325_POWER_CTRL3,
                ILI9325_POWER_CTRL3_PON | ILI9325_POWER_CTRL3_VRH(11));
  up_mdelay(50);

  /* Set VDV[4:0] for VCOM amplitude */

  sam_write_reg(ILI9325_POWER_CTRL4, ILI9325_POWER_CTRL4_VDV(17));

  /* Set VCM[5:0] for VCOMH */

  sam_write_reg(ILI9325_POWER_CTRL7, ILI9325_POWER_CTRL7_VCM(25));

  /* Set Frame Rate */

  sam_write_reg(ILI9325_FRAME_RATE_AND_COLOR_CTRL,
                ILI9325_FRAME_RATE_AND_COLOR_CTRL_FRS(13));
  up_mdelay(50);

  /* Adjust the Gamma Curve */

  sam_write_reg(ILI9325_GAMMA_CTRL1, 9);
  sam_write_reg(ILI9325_GAMMA_CTRL2,
                ILI9325_GAMMA_CTRL2_KP3(2) | ILI9325_GAMMA_CTRL2_KP2(4));
  sam_write_reg(ILI9325_GAMMA_CTRL3,
                ILI9325_GAMMA_CTRL3_KP5(2) | ILI9325_GAMMA_CTRL3_KP4(0));
  sam_write_reg(ILI9325_GAMMA_CTRL4,
                ILI9325_GAMMA_CTRL4_RP1(0) | ILI9325_GAMMA_CTRL4_RP0(7));
  sam_write_reg(ILI9325_GAMMA_CTRL5,
                ILI9325_GAMMA_CTRL5_VRP1(20) | ILI9325_GAMMA_CTRL5_VRP0(4));
  sam_write_reg(ILI9325_GAMMA_CTRL6,
                ILI9325_GAMMA_CTRL6_KN1(7) | ILI9325_GAMMA_CTRL6_KN0(5));
  sam_write_reg(ILI9325_GAMMA_CTRL7,
                ILI9325_GAMMA_CTRL7_KN3(3) | ILI9325_GAMMA_CTRL7_KN2(5));
  sam_write_reg(ILI9325_GAMMA_CTRL8,
                ILI9325_GAMMA_CTRL8_KN5(7) | ILI9325_GAMMA_CTRL8_KN4(7));
  sam_write_reg(ILI9325_GAMMA_CTRL9,
                ILI9325_GAMMA_CTRL9_RN1(7) | ILI9325_GAMMA_CTRL9_RN0(1));
  sam_write_reg(ILI9325_GAMMA_CTRL10,
                ILI9325_GAMMA_CTRL10_VRN1(0) |
                ILI9325_GAMMA_CTRL10_VRN0(14));

  /* Set the Entry Mode:
   *
   * AM controls the GRAM update direction.
   *   0 = The address is updated in horizontal writing direction.
   *   1 = The address is updated in vertical writing direction.
   *
   * I/D[1:0] controls the address counter (AC) to automatically increase or
   *   decrease by 1 when update one pixel display data.
   *
   *  00 = Horizontal decrement, Vertical decrement
   *  01 = Horizontal increment, Vertical decrement
   *  10 = Horizontal decrement, Vertical increment
   *  11 = Horizontal increment, Vertical increment
   *
   * ORG moves the origin address according to the ID setting when a window
   * address area is made.
   *   0 = The origin address is not moved.
   *   1 = The original address moves according to the I/D[1:0] setting.
   *
   * BGR swaps the R and B order of written data.
   *   0 = Follow the RGB order to write the pixel data.
   *   1 = Swap the RGB data to BGR in writing into GRAM.
   *
   * TRI = 1: Data are transferred to the internal RAM in 8-bit x 3 transfers
   *   mode via the 8-bit interface.
   *
   * DFM + TRI: Data is transferred as 3 byte transfers
   *
   * Use the high speed write mode (HWM=1).
   * When TRI = 1, data are transferred to the internal RAM in 8-bit x 3
   * transfers mode via the 8-bit interface.
   * DFM set the mode of transferring data to the internal RAM when TRI = 1.
   * I/D[1:0] = 11 Horizontal : increment
   * Vertical : increment, AM=0:Horizontal
   */

#if defined(CONFIG_LCD_LANDSCAPE)
  /* Landscape: Horizontal increment/ Vertical decrement, address is update
   * in horizontal direction
   */

  regval = ILI9325_ENTRY_MODE_ID(1) | ILI9325_ENTRY_MODE_ORG |
           ILI9325_ENTRY_MODE_HWM | ILI9325_ENTRY_MODE_BGR |
           ILI9325_ENTRY_MODE_TRI | ILI9325_ENTRY_MODE_DFM;

#elif defined(CONFIG_LCD_RLANDSCAPE)
  /* Landscape: Horizontal decrement/ Vertical increment, address is update
   * in horizontal direction
   */

  regval = ILI9325_ENTRY_MODE_ID(2) | ILI9325_ENTRY_MODE_ORG |
           ILI9325_ENTRY_MODE_HWM | ILI9325_ENTRY_MODE_BGR |
           ILI9325_ENTRY_MODE_TRI | ILI9325_ENTRY_MODE_DFM;

#elif defined(CONFIG_LCD_PORTRAIT)
  /* Landscape: Horizontal decrement/ Vertical decrement, address is update
   * in vertical direction
   */

  regval = ILI9325_ENTRY_MODE_AM | ILI9325_ENTRY_MODE_ID(0) |
           ILI9325_ENTRY_MODE_ORG | ILI9325_ENTRY_MODE_HWM |
           ILI9325_ENTRY_MODE_BGR | ILI9325_ENTRY_MODE_TRI |
           ILI9325_ENTRY_MODE_DFM;

#else /* if defined(CONFIG_LCD_RPORTRAIT) */
  /* Landscape: Horizontal increment/ Vertical increment, address is update
   * in vertical direction
   */

  regval = ILI9325_ENTRY_MODE_AM | ILI9325_ENTRY_MODE_ID(3) |
           ILI9325_ENTRY_MODE_ORG | ILI9325_ENTRY_MODE_HWM |
           ILI9325_ENTRY_MODE_BGR | ILI9325_ENTRY_MODE_TRI |
           ILI9325_ENTRY_MODE_DFM;
#endif

  sam_write_reg(ILI9325_ENTRY_MODE, regval);

  /* Driver Output Control
   *
   * SS: Select the shift direction of outputs from the source driver
   *   0 = The shift direction of outputs is from S1 to S720
   *   1 = The shift direction of outputs is from S720 to S1
   * SM: Sets the gate driver pin arrangement in combination with the GS bit
   */

#if defined(CONFIG_LCD_LANDSCAPE) || defined(CONFIG_LCD_RPORTRAIT)
  /* Horizontal increment */

  regval = 0;

#else /* defined(CONFIG_LCD_RLANDSCAPE) || defined(CONFIG_LCD_PORTRAIT) */
  /* Horizontal decrement */

  regval = ILI9325_DRIVER_OUTPUT_CTRL1_SS;
#endif

  sam_write_reg(ILI9325_DRIVER_OUTPUT_CTRL1, regval);

  /* Set the number of lines to drive the LCD at an interval of 8 lines.
   * The scan direction is from G320 to G1
   */

  regval = ILI9325_DRIVER_OUTPUT_CTRL2_NL((SAM_XRES / 8) - 1);

#if  defined(CONFIG_LCD_LANDSCAPE) || defined(CONFIG_LCD_PORTRAIT)
  /* Vertical decrement */

  regval |= ILI9325_DRIVER_OUTPUT_CTRL2_GS;
#endif

  sam_write_reg(ILI9325_DRIVER_OUTPUT_CTRL2, regval);

  /* Vertical Scrolling *****************************************************/

  /* Disable scrolling and enable the grayscale inversion */

  sam_write_reg(ILI9325_BASE_IMG_DISP_CTRL, ILI9325_BASE_IMG_DISP_CTRL_REV);
  sam_write_reg(ILI9325_VERTICAL_SCROLL_CTRL, 0);

  /* Disable Partial Display */

  sam_write_reg(ILI9325_PARTIAL_IMG1_DISP_SHIFT, 0);
  sam_write_reg(ILI9325_PARTIAL_IMG1_AREA_START_LINE, 0);
  sam_write_reg(ILI9325_PARTIAL_IMG1_AREA_END_LINE, 0);
  sam_write_reg(ILI9325_PARTIAL_IMG2_DISP_SHIFT, 0);
  sam_write_reg(ILI9325_PARTIAL_IMG2_AREA_START_LINE, 0);
  sam_write_reg(ILI9325_PARTIAL_IMG2_AREA_END_LINE, 0);

  /* Panel Control */

  sam_write_reg(ILI9325_PANEL_INTERFACE_CTRL1,
                ILI9325_PANEL_INTERFACE_CTRL1_RTNI(16));
  sam_write_reg(ILI9325_PANEL_INTERFACE_CTRL2,
               ILI9325_PANEL_INTERFACE_CTRL2_NOWI(6));
  sam_write_reg(ILI9325_PANEL_INTERFACE_CTRL4,
                ILI9325_PANEL_INTERFACE_CTRL4_DIVE(1) |
                ILI9325_PANEL_INTERFACE_CTRL4_RTNE(16));

  /* Set the drawing window */

  sam_write_reg(ILI9325_HORIZONTAL_ADDR_START, 0);
  sam_write_reg(ILI9325_HORIZONTAL_ADDR_END, SAM_XRES - 1);
  sam_write_reg(ILI9325_VERTICAL_ADDR_START, 0);
  sam_write_reg(ILI9325_VERTICAL_ADDR_END, SAM_YRES - 1);

  /* Home the cursor */

  sam_set_cursor(0, 0);
}

/****************************************************************************
 * Name:  sam_lcd_initialize
 *
 * Description:
 *   Initialize the LCD panel
 *
 ****************************************************************************/

static inline int sam_lcd_initialize(void)
{
  uint16_t id;

  /* Check the LCD ID */

  id = sam_read_reg(ILI9325_DEVICE_CODE_REG);
  lcdinfo("LCD ID: %04x\n", id);

  /* Initialize the LCD hardware */

  if (id != ILI9325_DEVICE_CODE)
    {
      lcderr("ERROR: Unsupported LCD: %04x\n", id);
      return -ENODEV;
    }

  sam_lcd9325_initialize();
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  board_lcd_initialize
 *
 * Description:
 *   Initialize the LCD video hardware.
 *   The initial state of the LCD is fully initialized, display memory
 *   cleared, and the LCD ready to use, but with the  power setting at 0
 *   (full off).
 *
 ****************************************************************************/

int board_lcd_initialize(void)
{
  struct sam_dev_s *priv = &g_lcddev;
  int ret;

  lcdinfo("Initializing\n");

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

/****************************************************************************
 * Name: board_lcd_getdev
 *
 * Description:
 *   Return a a reference to the LCD object for the specified LCD.
 *   This allows support for multiple LCD devices.
 *
 ****************************************************************************/

struct lcd_dev_s *board_lcd_getdev(int lcddev)
{
  DEBUGASSERT(lcddev == 0);
  return &g_lcddev.dev;
}

/****************************************************************************
 * Name:  board_lcd_uninitialize
 *
 * Description:
 *   Uninitialize the LCD support
 *
 ****************************************************************************/

void board_lcd_uninitialize(void)
{
  struct sam_dev_s *priv = &g_lcddev;

  /* Put the LCD in the lowest possible power state */

  sam_poweroff(priv);
}

/****************************************************************************
 * Name:  sam_lcdclear
 *
 * Description:
 *   This is a non-standard LCD interface just for the SAM4E-EK board.
 *   Because of the various rotations, clearing the display in the normal
 *   way by writing a sequences of runs that covers the entire display can
 *   be very slow.  Here the display is cleared by simply setting all GRAM
 *   memory to the specified color.
 *
 ****************************************************************************/

#if defined(CONFIG_SAM4EEK_LCD_RGB565)
void sam_lcdclear(uint16_t color)
#else /* if defined(CONFIG_SAM4EEK_LCD_RGB24) defined(CONFIG_SAM4EEK_LCD_RGB32) */
void sam_lcdclear(uint32_t color)
#endif
{
  uint8_t r = RGB_RED(color);
  uint8_t g = RGB_GREEN(color);
  uint8_t b = RGB_BLUE(color);
  uint32_t i;

  sam_set_cursor(0, 0);
  sam_gram_prepare();

  for (i = SAM_XRES * SAM_YRES; i > 0; i--)
    {
      LCD_DATA = r;
      LCD_DATA = g;
      LCD_DATA = b;
    }
}

#endif /* CONFIG_LCD */
