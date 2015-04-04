/**************************************************************************************
 * configs/mikroe-stm32f4/src/stm32_mio283qt9a.c
 *
 * Interface definition for the MI0283QT-9A LCD from Multi-Inno Technology Co., Ltd.
 * LCD is based on the Ilitek ILI9341 LCD controller.
 *
 *   Copyright (C) 2012-2014 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *
 *   Modified: 2013-2014 by Ken Pettit to support Mikroe-STM32F4 board.
 *   Adapted by Tobias Duckworth <toby@orogenic.net> for the MI0283QT-9A
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
 * Included Files
 **************************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/mio283qt9a.h>

#include <arch/board/board.h>

#include "up_arch.h"
#include "stm32.h"
#include "stm32_gpio.h"
#include "mikroe-stm32f4-internal.h"

#ifdef CONFIG_LCD_MIO283QT9A

/**************************************************************************************
 * Pre-processor Definitions
 **************************************************************************************/
/* Configuration **********************************************************************/

/* Define CONFIG_DEBUG_LCD to enable detailed LCD debug output. Verbose debug must
 * also be enabled.
 */

#ifndef CONFIG_DEBUG
#  undef CONFIG_DEBUG_VERBOSE
#  undef CONFIG_DEBUG_GRAPHICS
#  undef CONFIG_DEBUG_LCD
#endif

#ifndef CONFIG_DEBUG_VERBOSE
#  undef CONFIG_DEBUG_LCD
#endif

/* Mikroe-STM32F4 Hardware Definitions ************************************************/
/* --- ---------------------------------- -------------------- ------------------------
 * PIN CONFIGURATIONS                     SIGNAL NAME          ON-BOARD CONNECTIONS
 *     (Family Data Sheet Table 1-1)     (PIC32MX7 Schematic)
 * --- ---------------------------------- -------------------- ------------------------
 *  39 PE8                                LCD_RST              TFT display
 *  46 PE15                               LCD-CS#              TFT display
 *  40 PE9                                LCD_BLED             LCD backlight LED
 *  43 PE12                               LCD-RS               TFT display
 *
 *
 *  97 RE0                                T_D0                 TFT display
 *  98 RE1                                T_D1                 TFT display
 *   1 RE2                                T_D2                 TFT display
 *   2 RE3                                T_D3                 TFT display
 *   3 RE4                                T_D4                 TFT display
 *   4 RE5                                T_D5                 TFT display
 *   5 RE6                                T_D6                 TFT display
 *  38 RE7                                T_D7                 TFT display
 *
 *  41 PE10                               PMPRD
 *  42 RE11                               PMPWR
 *
 * TOUCHSCREEN PIN CONFIGURATIONS
 * PIN CONFIGURATIONS                     SIGNAL NAME          ON-BOARD CONNECTIONS
 * --- ---------------------------------- -------------------- ------------------------
 *  35 PB0                                LCD-YD               TFT display
 *   ?                                    LCD-XR               TFT display
 *   ?                                    LCD-YU               TFT display
 *  36 PB1                                LCD-XL               TFT display
 *  95 PB8                                DRIVEA               TFT display
 *  96 PB9                                DRIVEB               TFT display
 */

/* Debug ******************************************************************************/

#ifdef CONFIG_DEBUG_LCD
#  define lcddbg       dbg
#  define lcdvdbg      vdbg
#else
#  define lcddbg(x...)
#  define lcdvdbg(x...)
#endif

/**************************************************************************************
 * Private Type Definition
 **************************************************************************************/

struct stm32f4_dev_s
{
  struct mio283qt9a_lcd_s dev;      /* The externally visible part of the driver */
  bool                   grammode;  /* true=Writing to GRAM (16-bit write vs 8-bit) */
  bool                   firstread; /* First GRAM read? */
  FAR struct lcd_dev_s  *drvr;      /* The saved instance of the LCD driver */
};

/**************************************************************************************
 * Private Function Protototypes
 **************************************************************************************/
/* Low Level LCD access */

static void stm32_select(FAR struct mio283qt9a_lcd_s *dev);
static void stm32_deselect(FAR struct mio283qt9a_lcd_s *dev);
static void stm32_index(FAR struct mio283qt9a_lcd_s *dev, uint8_t index);
#if !defined(CONFIG_MIO283QT2_WRONLY) && CONFIG_LCD_NOGETRUN != 1
static uint16_t stm32_read(FAR struct mio283qt9a_lcd_s *dev);
#endif
static void stm32_write(FAR struct mio283qt9a_lcd_s *dev, uint16_t data);
static void stm32_backlight(FAR struct mio283qt9a_lcd_s *dev, int power);

/**************************************************************************************
 * Private Data
 **************************************************************************************/

/* This is the driver state structure (there is no retained state information) */

static struct stm32f4_dev_s g_stm32f4_lcd =
{
  {
    .select    = stm32_select,
    .deselect  = stm32_deselect,
    .index     = stm32_index,
#if !defined(CONFIG_MIO283QT2_WRONLY) && CONFIG_LCD_NOGETRUN != 1
    .read      = stm32_read,
#endif
    .write     = stm32_write,
    .backlight = stm32_backlight
  }
};

static uint32_t * volatile g_portsetreset = (uint32_t *) STM32_GPIOE_BSRR;

/**************************************************************************************
 * Private Functions
 **************************************************************************************/

/**************************************************************************************
 * Name: stm32_tinydelay
 *
 * Description:
 *   Delay for a few hundred NS.
 *
 **************************************************************************************/

static void stm32_tinydelay(void)
{
  volatile uint8_t x = 0;

  for (x = 1; x > 0; x--)
    ;
}

/**************************************************************************************
 * Name: stm32_command
 *
 * Description:
 *   Configure to write an LCD command
 *
 **************************************************************************************/

static inline void stm32_command(void)
{
  uint32_t  * volatile portsetreset = (uint32_t *) STM32_GPIOE_BSRR;

  /* Low selects command */

  *portsetreset = (1 << LCD_RS_PIN) << 16;
}

/**************************************************************************************
 * Name: stm32_data
 *
 * Description:
 *   Configure to read or write LCD data
 *
 **************************************************************************************/

static inline void stm32_data(void)
{
  /* Hi selects data */

  *g_portsetreset = 1 << LCD_RS_PIN;
}

/**************************************************************************************
 * Name: stm32_select
 *
 * Description:
 *   Select the LCD device
 *
 **************************************************************************************/

static void stm32_select(FAR struct mio283qt9a_lcd_s *dev)
{
  /* CS low selects */

  *g_portsetreset = (1 << LCD_CS_PIN) << 16;
}

/**************************************************************************************
 * Name: stm32_deselect
 *
 * Description:
 *   De-select the LCD device
 *
 **************************************************************************************/

static void stm32_deselect(FAR struct mio283qt9a_lcd_s *dev)
{
  /* CS high de-selects */

  *g_portsetreset = 1 << LCD_CS_PIN;
}

/**************************************************************************************
 * Name: stm32_index
 *
 * Description:
 *   Set the index register
 *
 **************************************************************************************/

static void stm32_index(FAR struct mio283qt9a_lcd_s *dev, uint8_t index)
{
  FAR struct stm32f4_dev_s *priv = (FAR struct stm32f4_dev_s *)dev;

  /* Setup to write in command mode (vs data mode) */

  stm32_command();

  /* Write the index register to the 8-bit GPIO pin bus.  We are violating the
   * datasheet here a little by driving the WR pin low at the same time as
   * the data, but the fact is that all ASIC logic will latch on the rising
   * edge of WR anyway, not the falling edge.  We are just shaving off a few
   * cycles every time this routine is called, which will be fairly often.
   */

  *g_portsetreset = index | ((uint8_t) (~index) << 16) |
      ((1 << LCD_PMPWR_PIN) << 16);

  /* Record if we are accessing GRAM or not (16 vs 8 bit accesses)
   * NOTE.  This also serves as a delay between WR low to WR high
   * transition.
   */

  priv->grammode = index == 0x2c;
  priv->firstread = true;

  /* Now raise the WR line */

  *g_portsetreset = (1 << LCD_PMPWR_PIN);

  /* Back to data mode to read/write the data */

  stm32_data();
}

/**************************************************************************************
 * Name: stm32_read
 *
 * Description:
 *   Read LCD data (GRAM data or register contents)
 *
 **************************************************************************************/

#if !defined(CONFIG_MIO283QT2_WRONLY) && CONFIG_LCD_NOGETRUN != 1
static uint16_t stm32_read(FAR struct mio283qt9a_lcd_s *dev)
{
  FAR struct stm32f4_dev_s *priv = (FAR struct stm32f4_dev_s *)dev;
  uint32_t  * volatile portsetreset = (uint32_t *) STM32_GPIOE_BSRR;
  uint32_t  * volatile portmode = (uint32_t *) STM32_GPIOE_MODER;
  uint32_t  * volatile portinput = (uint32_t *) STM32_GPIOE_IDR;
  uint16_t data;

  /* Set the I/O Port to input mode.  Ugly, but fast. */

  *portmode &= 0xFFFF0000;

  /* Read the data */

  *portsetreset = (1 << LCD_PMPRD_PIN) << 16;
  stm32_tinydelay();
  data = *portinput & 0x00FF;
  *portsetreset = (1 << LCD_PMPRD_PIN);

  /* Test if a 16-bit read is needed (GRAM mode) */

  if (priv->grammode)
    {
      /* If this is the 1st GRAM read, then discard the dummy byte */

      if (priv->firstread)
        {
          priv->firstread = false;
          *portsetreset = (1 << LCD_PMPRD_PIN) << 16;
          stm32_tinydelay();
          data = *portinput;
          *portsetreset = (1 << LCD_PMPRD_PIN);
        }

      /* Okay, a 16-bit read is actually a 24-bit read from the LCD.
       * this is because the read color format of the MIO283QT-2 is a bit
       * different than the 16-bit write color format.  During a read,
       * the R,G and B samples are read on subsequent bytes, and the
       * data is MSB aligned.  We must re-construct the 16-bit 565 data.
       */

      /* Clip RED sample to 5-bits and shit to MSB */

      data = (data & 0xF8) << 8;

      /* Now read Green sample */

      *portsetreset = (1 << LCD_PMPRD_PIN) << 16;
      stm32_tinydelay();
      data |= (*portinput & 0x00FC) << 3;
      *portsetreset = (1 << LCD_PMPRD_PIN);

      /* Now read Blue sample */

      *portsetreset = (1 << LCD_PMPRD_PIN) << 16;
      stm32_tinydelay();
      data |= (*portinput & 0x00F8) >> 3;
      *portsetreset = (1 << LCD_PMPRD_PIN);
    }

  /* Put the port back in output mode.  Ugly, but fast. */

  *portmode |= 0x00005555;

  return data;
}
#endif

/**************************************************************************************
 * Name: stm32_write
 *
 * Description:
 *   Write LCD data (GRAM data or register contents)
 *
 **************************************************************************************/

static void stm32_write(FAR struct mio283qt9a_lcd_s *dev, uint16_t data)
{
  FAR struct stm32f4_dev_s *priv = (FAR struct stm32f4_dev_s *)dev;

  /* Write the data register to the 8-bit GPIO pin bus.  We are violating the
   * datasheet here a little by driving the WR pin low at the same time as
   * the data, but the fact is that all ASIC logic will latch on the rising
   * edge of WR anyway, not the falling edge.  We are just shaving off a few
   * cycles every time this routine is called, which will be fairly often.
   */

  if (priv->grammode)
    {
      /* Need to write 16-bit pixel data (16 BPP).  Write the upper pixel data first */

      *g_portsetreset = ((data>>8) & 0xFF) | (((~data>>8) & 0xFF) << 16) |
        ((1 << LCD_PMPWR_PIN) << 16);
      stm32_tinydelay();
      *g_portsetreset = (1 << LCD_PMPWR_PIN);
    }

  /* Now write the lower 8-bit of data */

  *g_portsetreset = (data & 0xFF) | ((~data & 0xFF) << 16) |
    ((1 << LCD_PMPWR_PIN) << 16);
  stm32_tinydelay();
  *g_portsetreset = (1 << LCD_PMPWR_PIN);
}

/**************************************************************************************
 * Name: stm32_backlight
 *
 * Description:
 *   Set the backlight power level.
 *
 **************************************************************************************/

static void stm32_backlight(FAR struct mio283qt9a_lcd_s *dev, int power)
{
  /* For now, we just control the backlight as a discrete.  Pulse width modulation
   * would be required to vary the backlight level.  A low value turns the backlight
   * off.
   */

  stm32_gpiowrite(GPIO_LCD_BLED, power > 0);
}

/**************************************************************************************
 * Public Functions
 **************************************************************************************/

/**************************************************************************************
 * Name: stm32_lcdinitialize
 *
 * Description:
 *   Initialize the LCD.  This function should be called early in the boot
 *   sequendce -- Even if the LCD is not enabled.  In that case we should
 *   at a minimum at least disable the LCD backlight.
 *
 **************************************************************************************/

void stm32_lcdinitialize(void)
{
  /* Configure all LCD discrete controls.  LCD will be left in this state:
   * 1. Held in reset,
   * 2. Not selected,
   * 3. Backlight off,
   * 4. Command selected.
   */

#ifdef CONFIG_LCD_MIO283QT9A
   stm32_configgpio(GPIO_LCD_RST);
   stm32_configgpio(GPIO_LCD_CS);
   stm32_configgpio(GPIO_LCD_BLED);
   stm32_gpiowrite(GPIO_LCD_BLED, false);
   stm32_configgpio(GPIO_LCD_RS);
   stm32_configgpio(GPIO_LCD_PMPWR);
   stm32_configgpio(GPIO_LCD_PMPRD);

   /* Configure PE0-7 for output */

   stm32_configgpio(GPIO_LCD_T_D0);
   stm32_configgpio(GPIO_LCD_T_D1);
   stm32_configgpio(GPIO_LCD_T_D2);
   stm32_configgpio(GPIO_LCD_T_D3);
   stm32_configgpio(GPIO_LCD_T_D4);
   stm32_configgpio(GPIO_LCD_T_D5);
   stm32_configgpio(GPIO_LCD_T_D6);
   stm32_configgpio(GPIO_LCD_T_D7);

#else
  /* Just configure the backlight control as an output and turn off the
   * backlight for now.
   */

   stm32_configgpio(GPIO_LCD_BLED);
#endif
}

/**************************************************************************************
 * Name:  board_lcd_initialize
 *
 * Description:
 *   Initialize the LCD video hardware.  The initial state of the LCD is fully
 *   initialized, display memory cleared, and the LCD ready to use, but with the power
 *   setting at 0 (full off).
 *
 **************************************************************************************/

int board_lcd_initialize(void)
{
  /* Only initialize the driver once.  NOTE: The LCD GPIOs were already configured
   * by stm32_lcdinitialize.
   */

  if (!g_stm32f4_lcd.drvr)
    {
      lcdvdbg("Initializing\n");

      /* Hold the LCD in reset (active low)  */

      stm32_gpiowrite(GPIO_LCD_RST, false);

      /* Bring the LCD out of reset */

      up_mdelay(5);
      stm32_gpiowrite(GPIO_LCD_RST, true);

      /* Configure and enable the LCD */

      up_mdelay(50);
      g_stm32f4_lcd.drvr = mio283qt9a_lcdinitialize(&g_stm32f4_lcd.dev);
      if (!g_stm32f4_lcd.drvr)
        {
          lcddbg("ERROR: mio283qt9a_lcdinitialize failed\n");
          return -ENODEV;
        }
    }

  /* Turn the display off */

  g_stm32f4_lcd.drvr->setpower(g_stm32f4_lcd.drvr, 0);
  return OK;
}

/**************************************************************************************
 * Name:  board_lcd_getdev
 *
 * Description:
 *   Return a a reference to the LCD object for the specified LCD.  This allows support
 *   for multiple LCD devices.
 *
 **************************************************************************************/

FAR struct lcd_dev_s *board_lcd_getdev(int lcddev)
{
  DEBUGASSERT(lcddev == 0);
  return g_stm32f4_lcd.drvr;
}

/**************************************************************************************
 * Name:  board_lcd_uninitialize
 *
 * Description:
 *   Uninitialize the LCD support
 *
 **************************************************************************************/

void board_lcd_uninitialize(void)
{
  /* Turn the display off */

  g_stm32f4_lcd.drvr->setpower(g_stm32f4_lcd.drvr, 0);
}

#endif /* CONFIG_LCD_MIO283QT9A */
