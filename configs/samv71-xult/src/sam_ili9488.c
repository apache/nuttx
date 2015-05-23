/****************************************************************************
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
 * Some the LCD and SMC initialization logic comes from Atmel sample code
 * for the SAMV7.  The Atmel sample code has two-clause BSD-like license
 * which does not require this copyright statement, but here it is anyway:
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
 ****************************************************************************/

/* maXTouch Xplained Pro Xplained Pro LCD Connector *************************
 *
 * Only the RGB is supported by this BSP (via SMC/EBI).  The switch mode
 * selector on the back of the maXtouch should be set in the OFF-ON-OFF
 * positions to select 16-bit color mode.
 *
 * ----------------- ------------- ------------------------------------------
 *        LCD            SAMV71    Description
 * Pin  Function     Pin  Function
 * ---- ------------ ---- -------- ------------------------------------------
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
 * ---- ------------ ---- -------- ------------------------------------------
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/wdog.h>
#include <nuttx/clock.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/ili9488.h>
#include <nuttx/video/rgbcolors.h>

#include <arch/irq.h>
#include <arch/board/board.h>

#include "cache.h"
#include "up_arch.h"
#include "sam_gpio.h"
#include "sam_periphclks.h"
#include "sam_xdmac.h"
#include "chip/sam_pmc.h"
#include "chip/sam_smc.h"
#include "chip/sam_pinmap.h"

#include "samv71-xult.h"
#include "atmxt-xpro.h"

#ifdef HAVE_ILI9488_SMC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
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

/* Display/Color Properties **************************************************/
/* Display Resolution */

#if defined(CONFIG_LCD_LANDSCAPE) || defined(CONFIG_LCD_RLANDSCAPE)
#  define SAM_XRES            480
#  define SAM_YRES            320
#else
#  define SAM_XRES            320
#  define SAM_YRES            480
#endif

/* Color depth and format */

#define SAM_BPP               16
#define SAM_COLORFMT          FB_FMT_RGB16_565

/* Color decoding macros */

#define RGB_RED(rgb)          RGB16RED(rgb)
#define RGB_GREEN(rgb)        RGB16GREEN(rgb)
#define RGB_BLUE(rgb)         RGB16BLUE(rgb)
#define RGB_COLOR(r,g,b)      RGBTO16(r,g,b)

/* SAMV7-XULT LCD Hardware Definitions ***************************************/
/* LCD /CS is NCS3 */

#define SAM_LCD_BASE          ((uintptr_t)SAM_EXTCS3_BASE)
#define SAM_LCD_CS            3

/* Backlight */

#define BKL_LEVELS            16
#define BKL_PULSE_DURATION    24
#define BKL_ENABLE_DURATION   (128*1024)
#define BKL_DISABLE_DURATION  (128*1024)

/* DMA */

#define DMA_FLAGS \
  DMACH_FLAG_PERIPHPID(0) | DMACH_FLAG_PERIPHAHB_AHB_IF1 | \
  DMACH_FLAG_PERIPHWIDTH_16BITS | DMACH_FLAG_PERIPHCHUNKSIZE_1 | \
  DMACH_FLAG_MEMPID(0) | DMACH_FLAG_MEMAHB_AHB_IF0 | \
  DMACH_FLAG_MEMWIDTH_16BITS | DMACH_FLAG_MEMINCREMENT | \
  DMACH_FLAG_MEMCHUNKSIZE_1 | DMACH_FLAG_MEMBURST_1

/* DMA timeout.  The value is not critical; we just don't want the system to
 * hang in the event that a DMA does not finish.  This is set to
 */

#define DMA_TIMEOUT_MS        (800)
#define DMA_TIMEOUT_TICKS     MSEC2TICK(DMA_TIMEOUT_MS)

/* Buffer Alignment.
 *
 * If the data cache is enabled the a higher level of alignment is required.  That is
 * because the data will need to be invalidated and that cache invalidation will occur
 * in multiples of full change lines.
 */

#ifdef CONFIG_ARMV7M_DCACHE
/* Align to the cache line size which we assume is >= 8 */

#  define LCD_ALIGN            ARMV7M_DCACHE_LINESIZE
#  define LCD_ALIGN_MASK       (LCD_ALIGN-1)
#  define LCD_ALIGN_UP(n)      (((n) + LCD_ALIGN_MASK) & ~LCD_ALIGN_MASK)

#  define LCD_RUNBUFFER_BYTES  LCD_ALIGN_UP(SAM_XRES * sizeof(uint16_t))
#  define LCD_RUNBUFFER_PIXELS (LCD_RUNBUFFER_BYTES / sizeof(uint16_t))

#else
/* Align to 2-byte boundaries */

#  define LCD_ALIGN            2
#  define LCD_ALIGN_MASK       1
#  define LCD_ALIGN_UP(n)      (((n) + 1) & ~1)

#  define LCD_RUNBUFFER_BYTES  SAM_XRES * sizeof(uint16_t)
#  define LCD_RUNBUFFER_PIXELS SAM_XRES
#endif

/* Debug *********************************************************************/

#ifdef CONFIG_DEBUG_LCD
#  define lcddbg              dbg
#  define lcdvdbg             vdbg
#else
#  define lcddbg(x...)
#  define lcdvdbg(x...)
#endif

#ifdef CONFIG_DEBUG_DMA
#  define SAMPLENDX_BEFORE_SETUP  0
#  define SAMPLENDX_AFTER_SETUP   1
#  define SAMPLENDX_DMA_CALLBACK  2
#  define SAMPLENDX_TIMEOUT       3
#  define DEBUG_NDMASAMPLES       4
#endif

/****************************************************************************
 * Private Type Definition
 ****************************************************************************/

/* Type definition for the correct size of one pixel (from the application
 * standpoint).
 */

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

  /* Allocated DMA channel */

  DMA_HANDLE dmach;
  WDOG_ID dmadog;         /* For DMA timeout detection */
  volatile int result;    /* Result of the DMA transfer */
  sem_t waitsem;          /* Used to way for DMA completion */
  volatile bool dmabusy;  /* True: DMA is in progress */
  volatile bool cmd;      /* True: Command transfer */

  /* Private LCD-specific information follows */

  uint8_t power;          /* Current power setting */
  bool output;            /* True: Configured for output */
#ifdef CONFIG_DEBUG_DMA
  uint8_t smplset;
  struct sam_dmaregs_s samples[DEBUG_NDMASAMPLES];
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* Low Level LCD access */

static int  sam_sendcmd(FAR struct sam_dev_s *priv, uint16_t cmd);
static int  sam_lcd_put(FAR struct sam_dev_s *priv, uint16_t cmd,
              FAR const uint16_t *buffer, unsigned int buflen);
static int  sam_lcd_get(FAR struct sam_dev_s *priv, uint8_t cmd,
              FAR uint16_t *buffer, unsigned int buflen);
static int  sam_lcd_getreg(FAR struct sam_dev_s *priv, uint8_t cmd,
              FAR uint8_t *buffer, unsigned int nbytes);
static int  sam_setwindow(FAR struct sam_dev_s *priv, sam_color_t row,
              sam_color_t col, sam_color_t width, sam_color_t height);

/* Backlight/power controls */

static void sam_disable_backlight(void);
static int  sam_set_backlight(unsigned int power);
static int  sam_poweroff(FAR struct sam_dev_s *priv);

/* DMA Helpers */

#ifdef CONFIG_DMA_DEBUG
static void sam_lcd_sample(FAR struct sam_dev_s *priv, int index);
static void sam_lcd_sampleinit(FAR struct sam_dev_s *priv);
static void sam_lcd_dumpone(FAR struct sam_dev_s *priv, int index,
              FAR const char *msg);
static void sam_lcd_dump(struct sam_dev_s *priv);
#else
#  define sam_lcd_sample(priv, index)
#  define sam_lcd_sampleinit(priv)
#  define sam_lcd_dump(priv)
#endif

static void sam_lcd_endwait(struct sam_dev_s *priv, int result);
static void sam_lcd_dmatimeout(int argc, uint32_t arg);
static int  sam_lcd_dmawait(FAR struct sam_dev_s *priv, uint32_t timeout);
static void sam_lcd_dmacallback(DMA_HANDLE handle, void *arg, int result);
static int  sam_lcd_txtransfer(FAR struct sam_dev_s *priv,
              FAR const uint16_t *buffer, unsigned int buflen);
static int  sam_lcd_rxtransfer(FAR struct sam_dev_s *priv,
              FAR const uint16_t *buffer, unsigned int buflen);

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

static int  sam_getpower(FAR struct lcd_dev_s *dev);
static int  sam_setpower(FAR struct lcd_dev_s *dev, int power);
static int  sam_getcontrast(FAR struct lcd_dev_s *dev);
static int  sam_setcontrast(FAR struct lcd_dev_s *dev, unsigned int contrast);

/* Initialization */

static void sam_gpio_initialize(void);
static inline void sam_smc_initialize(void);
static inline int sam_lcd_initialize(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/
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

static uint16_t g_runbuffer[LCD_RUNBUFFER_BYTES] __attribute__((aligned(LCD_ALIGN)));

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

/* This is the ILI9488 LCD driver object */

static struct sam_dev_s g_lcddev =
{
  /* This is the contained standard, NuttX LCD driver object */

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
 * Name: sam_sendcmd
 *
 * Description:
 *  Send an ILI9488 command byte
 *
 ****************************************************************************/

static int sam_sendcmd(FAR struct sam_dev_s *priv, uint16_t cmd)
{
  volatile int i;
  int ret;

  /* Set the CDS GPIO output low (command) */

  sam_gpiowrite(GPIO_ILI9488_CDS, false);

  /* Send the command via TX DMA */

  ret = sam_lcd_txtransfer(priv, &cmd, sizeof(uint16_t));
  if (ret < 0)
    {
      lcddbg("ERROR: Failed to send command %02x: %d\n", cmd, ret);
    }

  /* Make sure that the CMD/DATA GPIO is reset for commands.  I don't understand
   * the delay loop... it comes from the SAMV7 sample code and is, apparently, a
   * work-around for some issue.
   */

  for (i = 0; i < 15; i++);

  /* Set the CDS OUTPUT to high (data) */

  sam_gpiowrite(GPIO_ILI9488_CDS, true);
  return ret;
}

/****************************************************************************
 * Name:  sam_lcd_put
 *
 * Description:
 *   Write to a multi-byte ILI9488 register
 *
 ****************************************************************************/

static int sam_lcd_put(FAR struct sam_dev_s *priv, uint16_t cmd,
                       FAR const uint16_t *buffer, unsigned int buflen)
{
  int ret;

  /* Send the command */

  ret = sam_sendcmd(priv, cmd);
  if (ret < 0)
    {
      return ret;
    }

  /* If the command was sent successfully, then send any accompanying data */

  if (buflen > 0)
    {
      ret = sam_lcd_txtransfer(priv, buffer, buflen);
      if (ret < 0)
        {
          lcddbg("ERROR: Failed to send command %02x data: %d\n", cmd, ret);
       }
    }

  return ret;
}

/****************************************************************************
 * Name:  sam_lcd_get
 *
 * Description:
 *   Send a command and read 16-bit data from the ILI9488
 *
 ****************************************************************************/

static int sam_lcd_get(FAR struct sam_dev_s *priv, uint8_t cmd,
                       FAR uint16_t *buffer, unsigned int buflen)
{
  int ret;

  /* Send the command */

  ret = sam_sendcmd(priv, cmd);

  /* If the command was sent successfully, then receive any accompanying data */

  if (ret == OK && buflen > 0)
    {
      ret = sam_lcd_rxtransfer(priv, buffer, buflen);
    }

  return ret;
}

/****************************************************************************
 * Name:  sam_lcd_getreg
 *
 * Description:
 *   Read from a multi-byte ILI9488 register
 *
 ****************************************************************************/

static int sam_lcd_getreg(FAR struct sam_dev_s *priv, uint8_t cmd,
                          FAR uint8_t *buffer, unsigned int nbytes)
{
  uint32_t tmp[4];
  int ret;
  int i;

  DEBUGASSERT(nbytes <= 4);

  /* Read the request number of byes (as 16-bit values) plus a leading
   * dummy read.
   */

  ret = sam_lcd_get(priv, cmd, (FAR uint16_t *)tmp, nbytes << 2);
  if (ret == OK)
    {
      for (i = 0; i < nbytes; i++)
        {
          buffer[i] = tmp[i] & 0xff;
        }
    }

  return ret;
}

/****************************************************************************
 * Name:  sam_setwindow
 *
 * Description:
 *   Setup drawing window
 *
 ****************************************************************************/

static int sam_setwindow(FAR struct sam_dev_s *priv, sam_color_t row,
                         sam_color_t col, sam_color_t width,
                         sam_color_t height)
{
  uint16_t buffer[4];
  int ret;

  lcdvdbg("row=%d col=%d width=%d height=%d\n", row, col, width, height);

  /* Set Column Address Position */

  buffer[0] = (col >> 8) & 0xff;
  buffer[1] = col & 0xff;
  buffer[2] = ((col + width - 1) >> 8) & 0xff;
  buffer[3] = (col + width - 1) & 0xff;
  ret = sam_lcd_put(priv, ILI9488_CMD_COLUMN_ADDRESS_SET, buffer,
                    4 * sizeof(uint16_t));
  if (ret < 0)
    {
      return ret;
    }

  ret = sam_sendcmd(priv, ILI9488_CMD_NOP);
  if (ret < 0)
    {
      return ret;
    }

  /* Set Page Address Position */

  buffer[0] = (row >> 8) & 0xff;
  buffer[1] = row & 0xff;
  buffer[2] = ((row + height - 1) >> 8) & 0xff;
  buffer[3] = (row + height - 1) & 0xff;
  ret = sam_lcd_put(priv, ILI9488_CMD_PAGE_ADDRESS_SET, buffer,
                    4 * sizeof(uint16_t));
  if (ret < 0)
    {
      return ret;
    }

  return sam_sendcmd(priv, ILI9488_CMD_NOP);
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

/****************************************************************************
 * Name:  sam_disable_backlight
 *
 * Description:
 *   Turn the backlight off.
 *
 ****************************************************************************/

static void sam_disable_backlight(void)
{
  /* PWM support is not yet available.  Backlight is currently just configured as a
   * GPIO output.
   */
#warning Missing logic

  sam_gpiowrite(GPIO_ILI9488_BKL, false);
}

/****************************************************************************
 * Name:  sam_set_backlight
 *
 * Description:
 *   The the backlight to the level associated with the specified power value.
 *
 ****************************************************************************/

static int sam_set_backlight(unsigned int power)
{
  lcdvdbg("power=%d\n", power);

  /* PWM support is not yet available.  Backlight is currently just
   * configured as a GPIO output.
   */
#warning Missing logic

  if (power > 0)
    {
      sam_gpiowrite(GPIO_ILI9488_BKL, true);
    }
  else
    {
      sam_gpiowrite(GPIO_ILI9488_BKL, false);
    }

  return OK;
}

/****************************************************************************
 * Name:  sam_poweroff
 *
 * Description:
 *   Enable/disable LCD panel power (0: full off - CONFIG_LCD_MAXPOWER:
  *  full on). On backlit LCDs, this setting may correspond to the backlight
  *  setting.
 *
 ****************************************************************************/

static int sam_poweroff(FAR struct sam_dev_s *priv)
{
  int ret;

  lcdvdbg("OFF\n");

  /* Turn the display off */

  ret = sam_lcd_put(priv, ILI9488_CMD_DISPLAY_OFF, NULL, 0);

  /* Disable the backlight */

  sam_disable_backlight();

  /* Remember the power off state */

  priv->power = 0;
  return ret;
}

/****************************************************************************
 * Name: sam_lcd_sample
 *
 * Description:
 *   Sample HSMCI/DMA registers
 *
 ****************************************************************************/

#ifdef CONFIG_DMA_DEBUG
static void sam_lcd_sample(struct sam_dev_s *priv, int index)
{
  /* On a multiple block transfer, only sample on the first block */

  if ((priv->smplset & (1 << index)) == 0)
    {
      sam_dmasample(priv->dmach, &priv->samples[index]);
      priv->smplset |= (1 << index);
    }
}
#endif

/****************************************************************************
 * Name: sam_lcd_sampleinit
 *
 * Description:
 *   Setup prior to collecting DMA samples
 *
 ****************************************************************************/

#ifdef CONFIG_DMA_DEBUG
static void sam_lcd_sampleinit(struct sam_dev_s *priv)
{
  priv->smplset = 0;
  memset(priv->samples, 0xff, DEBUG_NDMASAMPLES * sizeof(struct sam_dmaregs_s));
}
#endif

/****************************************************************************
 * Name: sam_lcd_dumpone
 *
 * Description:
 *   Dump one transfer register sample
 *
 ****************************************************************************/

#ifdef CONFIG_DMA_DEBUG
static void sam_lcd_dumpone(struct sam_dev_s *priv, int index,
                                const char *msg)
{
  if ((priv->smplset & (1 << index)) != 0)
    {
      sam_dmadump(priv->dmach, &priv->samples[index], msg);
    }
  else
    {
      fdbg("%s: Not collected\n", msg);
    }
}
#endif

/****************************************************************************
 * Name: sam_lcd_dump
 *
 * Description:
 *   Dump all transfer-related, sampled register data
 *
 ****************************************************************************/

#ifdef CONFIG_DMA_DEBUG
static void  sam_lcd_dump(struct sam_dev_s *priv)
{
  sam_lcd_dumpone(priv, SAMPLENDX_BEFORE_SETUP, "Before setup");
  sam_lcd_dumpone(priv, SAMPLENDX_AFTER_SETUP, "After setup");
  sam_lcd_dumpone(priv, SAMPLENDX_END_TRANSFER, "End of transfer");
  sam_lcd_dumpone(priv, SAMPLENDX_DMA_CALLBACK, "DMA Callback");
  sam_lcd_dumpone(priv, SAMPLENDX_TIMEOUT, "Timeout");
  priv->smplset = 0;
}
#endif

/****************************************************************************
 * Name: sam_lcd_endwait
 *
 * Description:
 *   Wake up a waiting thread if the waited-for event has occurred.
 *
 * Input Parameters:
 *   priv      - An instance of the HSMCI device interface
 *   wkupevent - The event that caused the wait to end
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Always called from the interrupt level with interrupts disabled.
 *
 ****************************************************************************/

static void sam_lcd_endwait(struct sam_dev_s *priv, int result)
{
  /* Save the result and cancel the watchdog timeout */

  (void)wd_cancel(priv->dmadog);
  priv->result = result;

  /* Wake up the waiting thread */

  sem_post(&priv->waitsem);
}

/****************************************************************************
 * Name: sam_lcd_dmatimeout
 *
 * Description:
 *   The watchdog timeout setup when the DMA was startd.  Indicates a DMA
 *   timeout failure.
 *
 * Input Parameters:
 *   argc   - The number of arguments (should be 1)
 *   arg    - The argument (state structure reference cast to uint32_t)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Always called from the interrupt level with interrupts disabled.
 *
 ****************************************************************************/

static void sam_lcd_dmatimeout(int argc, uint32_t arg)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)arg;

  DEBUGASSERT(argc == 1 && priv != NULL);
  sam_lcd_sample((struct sam_dev_s *)arg, SAMPLENDX_TIMEOUT);

  /* Make sure that any hung DMA is stopped.  dmabusy == false is the cue
   * so the DMA callback is ignored.
   */

  priv->dmabusy = false;
  sam_dmastop(priv->dmach);

  /* The wake up the waiting client a timeout error */

  sam_lcd_endwait(priv, -ETIMEDOUT);
}

/****************************************************************************
 * Name: sam_lcd_dmawait
 *
 * Description:
 *   Wait for one either (1) the DMA to complete, or (2) a DMA timeout to
 *   occur.
 *
 * Input Parameters:
 *   dev     - An instance of the SDIO device interface
 *   timeout - Maximum time in milliseconds to wait.  Zero means immediate
 *             timeout with no wait.
 *
 * Returned Value:
 *   The result of the DMA transfer
 *
 ****************************************************************************/

static int sam_lcd_dmawait(FAR struct sam_dev_s *priv, uint32_t timeout)
{
  int ret;

  /* Started ... setup the timeout */

  ret = wd_start(priv->dmadog, timeout, (wdentry_t)sam_lcd_dmatimeout,
                 1, (uint32_t)priv);
  if (ret < 0)
    {
      lcddbg("ERROR: wd_start failed: %d\n", errno);
    }

  /* Loop until the event (or the timeout occurs). */

  while (priv->result == -EBUSY)
    {
      /* Wait for an event in event set to occur.  If this the event has
       * already occurred, then the semaphore will already have been
       * incremented and there will be no wait.
       */

      ret = sem_wait(&priv->waitsem);

      /* The only expected failure is EINTR */

      DEBUGASSERT(ret == OK || errno == EINTR);
    }

  /* Dump the collect DMA sample data */

  sam_lcd_dump(priv);
  return priv->result;
}

/****************************************************************************
 * Name: sam_lcd_dmacallback
 *
 * Description:
 *   Called when HSMCI DMA completes
 *
 ****************************************************************************/

static void sam_lcd_dmacallback(DMA_HANDLE handle, void *arg, int result)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)arg;

  /* Is DMA still active?  We can get this callback when sam_dmastop() is
   * called too.
   */

  if (priv->dmabusy)
    {
      /* Sample DMA registers */

      priv->dmabusy = false;
      sam_lcd_sample((struct sam_dev_s *)arg, SAMPLENDX_DMA_CALLBACK);

      /* Wake-up the waiting client */

      sam_lcd_endwait(priv, result);
    }
}

/****************************************************************************
 * Name: sam_lcd_txtransfer
 *
 * Description:
 *   Perform a TX DMA transfer (memory-to-LCD)
 *
 ****************************************************************************/

static int sam_lcd_txtransfer(FAR struct sam_dev_s *priv,
                              FAR const uint16_t *buffer, unsigned int buflen)
{
  irqstate_t flags;
  int ret;

  priv->dmabusy = true;
  priv->result  = -EBUSY;

  /* Set up to transfer to the LCD */

  ret = sam_dmatxsetup(priv->dmach, (uint32_t)SAM_LCD_BASE, (uint32_t)buffer, buflen);
  if (ret == OK)
    {
      flags = irqsave();

      /* The setup was successful, start the DMA */

      ret = sam_dmastart(priv->dmach, sam_lcd_dmacallback, priv);
      if (ret == OK)
        {
          /* Started ... Wait for the DMA (or timeout) to complete */

          ret = sam_lcd_dmawait(priv, DMA_TIMEOUT_TICKS);
        }

      irqrestore(flags);
    }

  priv->dmabusy = false;
  return ret;
}

/****************************************************************************
 * Name: sam_lcd_rxtransfer
 *
 * Description:
 *   Perform a RX DMA transfer (LCD-to-memory)
 *
 ****************************************************************************/

static int sam_lcd_rxtransfer(FAR struct sam_dev_s *priv,
                              FAR const uint16_t *buffer, unsigned int buflen)
{
  irqstate_t flags;
  int ret;

  priv->dmabusy = true;
  priv->result  = -EBUSY;

  /* Set up to transfer to the LCD */

  ret = sam_dmarxsetup(priv->dmach, (uint32_t)SAM_LCD_BASE, (uint32_t)buffer, buflen);
  if (ret == OK)
    {
      flags = irqsave();

      /* The setup was successful, start the DMA */

      ret = sam_dmastart(priv->dmach, sam_lcd_dmacallback, priv);
      if (ret == OK)
        {
          /* Started ... Wait for the DMA (or timeout) to complete */

          ret = sam_lcd_dmawait(priv, DMA_TIMEOUT_TICKS);
        }

      irqrestore(flags);
    }

  priv->dmabusy = false;
  return ret;
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
                      FAR const uint8_t *buffer, size_t npixels)
{
  FAR struct sam_dev_s *priv = &g_lcddev;
  int ret;

  /* Buffer must be provided and aligned to a 16-bit address boundary */

  lcdvdbg("row: %d col: %d npixels: %d\n", row, col, npixels);
  DEBUGASSERT(buffer && ((uintptr_t)buffer & 1) == 0);

  /* Determine the refresh window area */

  ret = sam_setwindow(priv, row, col, npixels, 1);
  if (ret < 0)
    {
      lcddbg("ERROR: sam_setwindow failed: %d\n",  ret);
      return ret;
    }

  /* Write the run into the LCD */

  return sam_lcd_put(priv, ILI9488_CMD_MEMORY_WRITE,
                    (FAR const uint16_t *)buffer,
                     npixels * sizeof(uint16_t));
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
  FAR struct sam_dev_s *priv = &g_lcddev;
  int ret;

  /* Buffer must be provided and aligned to a 16-bit address boundary */

  lcdvdbg("row: %d col: %d npixels: %d\n", row, col, npixels);
  DEBUGASSERT(buffer && ((uintptr_t)buffer & 1) == 0);

  /* Determine the refresh window area */

  ret = sam_setwindow(priv, row, col, npixels, 1);
  if (ret < 0)
    {
      lcddbg("ERROR: sam_setwindow failed: %d\n",  ret);
      return ret;
    }

  /* Write the run into the LCD */

  return sam_lcd_get(priv, ILI9488_CMD_MEMORY_READ, (FAR uint16_t *)buffer,
                     npixels * sizeof(uint16_t));
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
  lcdvdbg("fmt: %d xres: %d yres: %d nplanes: %d\n",
          g_videoinfo.fmt, g_videoinfo.xres, g_videoinfo.yres,
          g_videoinfo.nplanes);

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
  lcdvdbg("planeno: %d bpp: %d\n", planeno, g_planeinfo.bpp);
  memcpy(pinfo, &g_planeinfo, sizeof(struct lcd_planeinfo_s));
  return OK;
}

/****************************************************************************
 * Name:  sam_getpower
 *
 * Description:
 *   Get the LCD panel power status (0: full off - CONFIG_LCD_MAXPOWER: full
 *   on). On backlit LCDs, this setting may correspond to the backlight
 *   setting.
 *
 ****************************************************************************/

static int sam_getpower(struct lcd_dev_s *dev)
{
  FAR struct sam_dev_s *priv = (FAR struct sam_dev_s *)dev;

  lcdvdbg("power: %d\n", 0);
  return priv->power;
}

/****************************************************************************
 * Name:  sam_setpower
 *
 * Description:
 *   Enable/disable LCD panel power (0: full off - CONFIG_LCD_MAXPOWER: full
 *   on). On backlit LCDs, this setting may correspond to the backlight
 *   setting.
 *
 ****************************************************************************/

static int sam_setpower(struct lcd_dev_s *dev, int power)
{
  FAR struct sam_dev_s *priv = (FAR struct sam_dev_s *)dev;
  int ret;

  lcdvdbg("power: %d\n", power);
  DEBUGASSERT((unsigned)power <= CONFIG_LCD_MAXPOWER);

  /* Set new power level */

  if (power > 0)
    {
      /* If the display was off, then turn the display on */

      if (priv->power == 0)
        {
          ret = sam_sendcmd(priv, ILI9488_CMD_PIXEL_OFF);
          if (ret < 0)
            {
              return ret;
            }

          ret = sam_sendcmd(priv, ILI9488_CMD_DISPLAY_ON);
          if (ret < 0)
            {
              return ret;
            }

          ret = sam_sendcmd(priv, ILI9488_CMD_NORMAL_DISP_MODE_ON);
          if (ret < 0)
            {
              return ret;
            }
        }

      /* Set the backlight level */

      ret = sam_set_backlight((unsigned int)power);
      up_mdelay(50);

      /* Remember the power setting */

      priv->power = power;
    }
  else
    {
      /* Turn the display off */

      ret = sam_poweroff(priv);
    }

  return ret;
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
  lcdvdbg("Not implemented\n");
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
  lcdvdbg("contrast: %d\n", contrast);
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

  /* Backlight off */

  sam_gpiowrite(GPIO_ILI9488_BKL, true);
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
  uintptr_t smcbase = SAM_SMCCS_BASE(SAM_LCD_CS);
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
           SMCCS_MODE_DBW_16BIT | SMCCS_MODE_TDFCYCLES(15);
  putreg32(regval, smcbase + SAM_SMCCS_MODE_OFFSET);
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
  FAR struct sam_dev_s *priv = &g_lcddev;
  uint8_t buffer[4] = {0, 0, 0, 0};
  uint16_t id;
  uint16_t param;
  int ret;

  /* Reset the LCD and bring it out of sleep mode */

  ret = sam_lcd_put(priv, ILI9488_CMD_SOFTWARE_RESET, &param, 0);
  if (ret < 0)
    {
      return ret;
    }

  up_mdelay(200);

  sam_lcd_put(priv, ILI9488_CMD_SLEEP_OUT, &param, 0);
  if (ret < 0)
    {
      return ret;
    }

  up_mdelay(200);

  /* Configure for tRGB and reverse the column order */

  param = 0x48;
  ret = sam_lcd_put(priv, ILI9488_CMD_MEMORY_ACCESS_CONTROL, &param,
                   sizeof(uint16_t));
  if (ret < 0)
    {
      return ret;
    }

  up_mdelay(100);

  param = 0x04;
  ret = sam_lcd_put(priv, ILI9488_CMD_CABC_CONTROL_9, &param,
                   sizeof(uint16_t));
  if (ret < 0)
    {
      return ret;
    }

  /* Check the LCD ID */

  ret = sam_lcd_getreg(priv, ILI9488_CMD_READ_ID4, buffer, 4);
  if (ret < 0)
    {
      return ret;
    }

  id = ((uint16_t)buffer[2] << 8) | ((uint16_t)buffer[3] & 0xff);
  lcdvdbg("ID: %04x\n", id);

  if (id != ILI9488_DEVICE_CODE)
    {
      lcddbg("ERROR: Unsupported LCD ID: %04x (vs. %04x)\n",
             id, ILI9488_DEVICE_CODE);
      return -ENODEV;
    }

  /* Set the RGB565 format */

  param = 5;
  ret = sam_lcd_put(priv, ILI9488_CMD_COLMOD_PIXEL_FORMAT_SET, &param,
                   sizeof(uint16_t));
  if (ret < 0)
    {
      return ret;
    }

  ret = sam_lcd_put(priv, ILI9488_CMD_NORMAL_DISP_MODE_ON, &param, 0);
  if (ret < 0)
    {
      return ret;
    }

  ret = sam_lcd_put(priv, ILI9488_CMD_DISPLAY_ON, &param, 0);
  if (ret < 0)
    {
      return ret;
    }

  /* Landscape/portrait mode */

#if defined(CONFIG_LCD_LANDSCAPE)
  param = 0xe8;
#elif defined(CONFIG_LCD_PORTRAIT)
  param = 0x48;
#else
#  error Unsupported LCD orientation
#endif

  ret = sam_lcd_put(priv, ILI9488_CMD_MEMORY_ACCESS_CONTROL, &param,
                    sizeof(uint16_t));
  if (ret < 0)
    {
      return ret;
    }

  /* Disable the backlight */

  sam_disable_backlight();

  /* Reset the refresh window area */

  return sam_setwindow(priv, 0, 0, SAM_XRES, SAM_YRES);
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
  FAR struct sam_dev_s *priv = &g_lcddev;
  int ret;

  lcdvdbg("Initializing\n");

  /* Configure all LCD pins pins (backlight is initially off) */

  sam_gpio_initialize();

  /* Configure SMC interface for the LCD */

  sam_smc_initialize();

  /* Initialize the LCD state structure */

  sem_init(&priv->waitsem, 0, 0);

  /* Allocate a DMA channel */

  priv->dmach = sam_dmachannel(0, DMA_FLAGS);
  if (!priv->dmach)
    {
      lcddbg("ERROR: Failed to allocate a DMA channel\n");
      ret =  -EAGAIN;
      goto errout_with_waitsem;
    }

  /* Allocate a watchdog timer to catch DMA timeouts */

  priv->dmadog = wd_create();
  if (!priv->dmadog)
    {
      lcddbg("ERROR: Failed to allocate a timer\n");
      ret = -EAGAIN;
      goto errout_with_dmach;
    }

  /* Identify and configure the LCD */

  up_mdelay(50);
  ret = sam_lcd_initialize();
  if (ret < 0)
    {
      lcddbg("ERROR: sam_lcd_initialize failed: %d\n", ret);
      goto errout_with_dmadog;
    }

  /* Clear the display (setting it to the color 0=black) */

  sam_lcdclear(CONFIG_SAMV71XULT_LCD_BGCOLOR);

  /* Turn the display off */

  ret = sam_poweroff(priv);
  if (ret < 0)
    {
      lcddbg("ERROR: sam_poweroff failed: %d\n", ret);
      goto errout_with_dmadog;
    }

  return OK;

errout_with_dmadog:
  wd_delete(priv->dmadog);
  priv->dmadog = NULL;

errout_with_dmach:
  sam_dmafree(priv->dmach);
  priv->dmach = NULL;

errout_with_waitsem:
  sem_destroy(&priv->waitsem);
  return ret;
}

/****************************************************************************
 * Name: board_lcd_getdev
 *
 * Description:
 *   Return a a reference to the LCD object for the specified LCD.  This
 *   allows support for multiple LCD devices.
 *
 ****************************************************************************/

FAR struct lcd_dev_s *board_lcd_getdev(int lcddev)
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
  FAR struct sam_dev_s *priv = &g_lcddev;

  /* Free the DMA channel */

  if (priv->dmach)
    {
      sam_dmafree(priv->dmach);
      priv->dmach = NULL;
    }

  /* Free other resources */

  wd_delete(priv->dmadog);
  priv->dmadog = NULL;

  sem_destroy(&priv->waitsem);

  /* Put the LCD in the lowest possible power state */

  sam_poweroff(priv);
}

/****************************************************************************
 * Name:  sam_lcdclear
 *
 * Description:
 *   This is a non-standard LCD interface just for the SAMV7-XULT board.
 *   Because of the various rotations, clearing the display in the normal
 *   way by writing a sequences of runs that covers the entire display can
 *   be very slow.
 *
 ****************************************************************************/

void sam_lcdclear(uint16_t color)
{
  FAR struct sam_dev_s *priv = &g_lcddev;
  unsigned int row;
  unsigned int col;
  int ret;

  /* Create a full width run of the requested color */

  for (col = 0; col < SAM_XRES; col++)
    {
      g_runbuffer[col] = color;
    }

  /* Then write the run into the LCD for each line */

  ret = sam_setwindow(priv, 0, 0, SAM_XRES, SAM_YRES);
  if (ret < 0)
    {
      lcddbg("ERROR: sam_setwindow failed: %d\n",  ret);
      return;
    }

  for (row = 0; row < SAM_YRES; row++)
    {
      ret = sam_putrun(row, 0, (FAR const uint8_t *)g_runbuffer, SAM_XRES);
      if (ret < 0)
        {
          lcddbg("ERROR: sam_putrun failed on row %d: %d\n", row, ret);
          return;
        }
    }
}

#endif /* HAVE_ILI9488_SMC */
