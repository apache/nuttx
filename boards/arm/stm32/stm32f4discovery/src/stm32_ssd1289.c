/****************************************************************************
 * boards/arm/stm32/stm32f4discovery/src/stm32_ssd1289.c
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
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/ssd1289.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "stm32.h"
#include "stm32f4discovery.h"

#ifdef CONFIG_LCD_SSD1289

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_STM32_FSMC
#  error "CONFIG_STM32_FSMC is required to use the LCD"
#endif

/* STM32F4Discovery LCD Hardware Definitions ********************************/

/* LCD /CS is CE1 ==  NOR/SRAM Bank 1
 *
 * Bank 1 = 0x60000000 | 0x00000000
 * Bank 2 = 0x60000000 | 0x04000000
 * Bank 3 = 0x60000000 | 0x08000000
 * Bank 4 = 0x60000000 | 0x0c000000
 *
 * FSMC address bit 16 is used to distinguish command and data.
 * FSMC address bits 0-24 correspond to ARM address bits 1-25.
 */

#define STM32_LCDBASE ((uintptr_t)(0x60000000 | 0x00000000))
#define LCD_INDEX     (STM32_LCDBASE)
#define LCD_DATA      (STM32_LCDBASE + 0x00020000)

/* SRAM pin definitions */

#define LCD_NADDRLINES   1   /* A16 */
#define LCD_NDATALINES   16  /* D0-15 */

/****************************************************************************
 * Private Function Protototypes
 ****************************************************************************/

/* Low Level LCD access */

static void stm32_select(FAR struct ssd1289_lcd_s *dev);
static void stm32_deselect(FAR struct ssd1289_lcd_s *dev);
static void stm32_index(FAR struct ssd1289_lcd_s *dev, uint8_t index);
#ifndef CONFIG_SSD1289_WRONLY
static uint16_t stm32_read(FAR struct ssd1289_lcd_s *dev);
#endif
static void stm32_write(FAR struct ssd1289_lcd_s *dev, uint16_t data);
static void stm32_backlight(FAR struct ssd1289_lcd_s *dev, int power);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* LCD pin mapping (see boards/arm/stm32/stm324discovery/README.txt
 * MAPPING TO STM32 F4:
 *
 *  ---------------- ------------- ----------------------------------
 *   STM32 FUNCTION  LCD PIN       STM32F4Discovery PIN
 *  ---------------- ------------- ----------------------------------
 *   FSMC_D0          D0    pin 4   PD14 P1 pin 46 Conflict (Note 1)
 *   FSMC_D1          D1    pin 3   PD15 P1 pin 47 Conflict (Note 2)
 *   FSMC_D2          D2    pin 6   PD0  P2 pin 36 Free I/O
 *   FSMC_D3          D3    pin 5   PD1  P2 pin 33 Free I/O
 *   FSMC_D4          D4    pin 8   PE7  P1 pin 25 Free I/O
 *   FSMC_D5          D5    pin 7   PE8  P1 pin 26 Free I/O
 *   FSMC_D6          D6    pin 10  PE9  P1 pin 27 Free I/O
 *   FSMC_D7          D7    pin 9   PE10 P1 pin 28 Free I/O
 *   FSMC_D8          D8    pin 12  PE11 P1 pin 29 Free I/O
 *   FSMC_D9          D9    pin 11  PE12 P1 pin 30 Free I/O
 *   FSMC_D10         D10   pin 14  PE13 P1 pin 31 Free I/O
 *   FSMC_D11         D11   pin 13  PE14 P1 pin 32 Free I/O
 *   FSMC_D12         D12   pin 16  PE15 P1 pin 33 Free I/O
 *   FSMC_D13         D13   pin 15  PD8  P1 pin 40 Free I/O
 *   FSMC_D14         D14   pin 18  PD9  P1 pin 41 Free I/O
 *   FSMC_D15         D15   pin 17  PD10 P1 pin 42 Free I/O
 *   FSMC_A16         RS    pin 19  PD11 P1 pin 27 Free I/O
 *   FSMC_NE1         ~CS   pin 10  PD7  P2 pin 27 Free I/O
 *   FSMC_NWE         ~WR   pin 22  PD5  P2 pin 29 Conflict (Note 3)
 *   FSMC_NOE         ~RD   pin 21  PD4  P2 pin 32 Conflict (Note 4)
 *   PC6              RESET pin 24  PC6  P2 pin 47 Free I/O
 *  ---------------- ------------- ----------------------------------
 *
 *   1 Used for the RED LED
 *   2 Used for the BLUE LED
 *   3 Used for the RED LED and for OTG FS Overcurrent.  It may be okay to
 *     use for the parallel interface if PC0 is held high (or floating).
 *     PC0 enables the STMPS2141STR IC power switch that drives the OTG FS
 *     host VBUS.
 *   4 Also the reset pin for the CS43L22 audio Codec.
 */

#define GPIO_LCD_RESET (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                        GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN6)

/* GPIO configurations unique to the LCD  */

static const uint32_t g_lcdconfig[] =
{
  /* PC6(RESET), FSMC_A16, FSMC_NOE, FSMC_NWE, and FSMC_NE1  */

  GPIO_LCD_RESET, GPIO_FSMC_A16, GPIO_FSMC_NOE, GPIO_FSMC_NWE, GPIO_FSMC_NE1
};
#define NLCD_CONFIG (sizeof(g_lcdconfig)/sizeof(uint32_t))

/* This is the driver state structure
 * (there is no retained state information)
 */

static struct ssd1289_lcd_s g_ssd1289 =
{
  .select    = stm32_select,
  .deselect  = stm32_deselect,
  .index     = stm32_index,
#ifndef CONFIG_SSD1289_WRONLY
  .read      = stm32_read,
#endif
  .write     = stm32_write,
  .backlight = stm32_backlight
};

/* The saved instance of the LCD driver */

static FAR struct lcd_dev_s *g_ssd1289drvr;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_select
 *
 * Description:
 *   Select the LCD device
 *
 ****************************************************************************/

static void stm32_select(FAR struct ssd1289_lcd_s *dev)
{
  /* Does not apply to this hardware */
}

/****************************************************************************
 * Name: stm32_deselect
 *
 * Description:
 *   De-select the LCD device
 *
 ****************************************************************************/

static void stm32_deselect(FAR struct ssd1289_lcd_s *dev)
{
  /* Does not apply to this hardware */
}

/****************************************************************************
 * Name: stm32_deselect
 *
 * Description:
 *   Set the index register
 *
 ****************************************************************************/

static void stm32_index(FAR struct ssd1289_lcd_s *dev, uint8_t index)
{
  putreg16((uint16_t)index, LCD_INDEX);
}

/****************************************************************************
 * Name: stm32_read
 *
 * Description:
 *   Read LCD data (GRAM data or register contents)
 *
 ****************************************************************************/

#ifndef CONFIG_SSD1289_WRONLY
static uint16_t stm32_read(FAR struct ssd1289_lcd_s *dev)
{
  return getreg16(LCD_DATA);
}
#endif

/****************************************************************************
 * Name: stm32_write
 *
 * Description:
 *   Write LCD data (GRAM data or register contents)
 *
 ****************************************************************************/

static void stm32_write(FAR struct ssd1289_lcd_s *dev, uint16_t data)
{
  putreg16((uint16_t)data, LCD_DATA);
}

/****************************************************************************
 * Name: stm32_write
 *
 * Description:
 *   Write LCD data (GRAM data or register contents)
 *
 ****************************************************************************/

static void stm32_backlight(FAR struct ssd1289_lcd_s *dev, int power)
{
#warning "Missing logic"
}

/****************************************************************************
 * Name: stm32_selectlcd
 *
 * Description:
 *   Initialize to the LCD
 *
 ****************************************************************************/

void stm32_selectlcd(void)
{
  /* Configure GPIO pins */

  stm32_extmemdata(LCD_NDATALINES);             /* Common data lines: D0-D15 */
  stm32_extmemgpios(g_lcdconfig, NLCD_CONFIG);  /* LCD-specific control lines */

  /* Enable AHB clocking to the FSMC */

  stm32_fsmc_enable();

  /* Color LCD configuration (LCD configured as follow):
   *
   *   - Data/Address MUX  = Disable   "FSMC_BCR_MUXEN" just not enable it.
   *   - Extended Mode     = Disable   "FSMC_BCR_EXTMOD"
   *   - Memory Type       = SRAM      "FSMC_BCR_SRAM"
   *   - Data Width        = 16bit     "FSMC_BCR_MWID16"
   *   - Write Operation   = Enable    "FSMC_BCR_WREN"
   *   - Asynchronous Wait = Disable
   */

  /* Bank1 NOR/SRAM control register configuration */

  putreg32(FSMC_BCR_SRAM | FSMC_BCR_MWID16 | FSMC_BCR_WREN, STM32_FSMC_BCR1);

  /* Bank1 NOR/SRAM timing register configuration */

  putreg32(FSMC_BTR_ADDSET(5) | FSMC_BTR_ADDHLD(0) |
           FSMC_BTR_DATAST(9) | FSMC_BTR_BUSTURN(0) |
           FSMC_BTR_CLKDIV(0) | FSMC_BTR_DATLAT(0) |
           FSMC_BTR_ACCMODA, STM32_FSMC_BTR1);

  putreg32(0xffffffff, STM32_FSMC_BWTR1);

  /* Enable the bank by setting the MBKEN bit */

  putreg32(FSMC_BCR_MBKEN | FSMC_BCR_SRAM |
           FSMC_BCR_MWID16 | FSMC_BCR_WREN, STM32_FSMC_BCR1);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  board_lcd_initialize
 *
 * Description:
 *   Initialize the LCD video hardware. The initial state of the LCD is fully
 *   initialized, display memory cleared, and the LCD ready to use, but with
 *   the power setting at 0 (full off).
 *
 ****************************************************************************/

int board_lcd_initialize(void)
{
  /* Only initialize the driver once */

  if (!g_ssd1289drvr)
    {
      lcdinfo("Initializing\n");

      /* Configure GPIO pins and configure the FSMC to support the LCD */

      stm32_selectlcd();

      /* Reset the LCD (active low) */

      stm32_gpiowrite(GPIO_LCD_RESET, false);
      up_mdelay(5);
      stm32_gpiowrite(GPIO_LCD_RESET, true);

      /* Configure and enable the LCD */

      up_mdelay(50);
      g_ssd1289drvr = ssd1289_lcdinitialize(&g_ssd1289);
      if (!g_ssd1289drvr)
        {
          lcderr("ERROR: ssd1289_lcdinitialize failed\n");
          return -ENODEV;
        }
    }

  /* Clear the display (setting it to the color 0=black) */

#if 0 /* Already done in the driver */
  ssd1289_clear(g_ssd1289drvr, 0);
#endif

  /* Turn the display off */

  g_ssd1289drvr->setpower(g_ssd1289drvr, 0);
  return OK;
}

/****************************************************************************
 * Name:  board_lcd_getdev
 *
 * Description:
 *   Return a a reference to the LCD object for the specified LCD.
 *    This allows support for multiple LCD devices.
 *
 ****************************************************************************/

FAR struct lcd_dev_s *board_lcd_getdev(int lcddev)
{
  DEBUGASSERT(lcddev == 0);
  return g_ssd1289drvr;
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
  /* Turn the display off */

  g_ssd1289drvr->setpower(g_ssd1289drvr, 0);
}

#endif /* CONFIG_LCD_SSD1289 */
