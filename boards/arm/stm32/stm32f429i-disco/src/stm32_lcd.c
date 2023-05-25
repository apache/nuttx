/****************************************************************************
 * boards/arm/stm32/stm32f429i-disco/src/stm32_lcd.c
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

#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/lcd/ili9341.h>
#include <nuttx/video/fb.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "stm32f429i-disco.h"
#include "stm32_ltdc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_STM32F429I_DISCO_ILI9341_LCDDEVICE
#  define ILI9341_LCD_DEVICE CONFIG_STM32F429I_DISCO_ILI9341_LCDDEVICE
#else
#  define ILI9341_LCD_DEVICE 0
#endif

#ifdef CONFIG_STM32F429I_DISCO_ILI9341_FBIFACE

/* Display settings */

/* Pixel Format Set (COLMOD)
 *
 * Note! RGB interface settings (DPI) is unimportant for the MCU interface
 * mode but set the register to the defined state equal to the MCU interface
 * pixel format.
 *
 * 16 Bit MCU:  01100101 / h65
 *
 * DPI:         6 (RGB18-666 RGB interface)
 * DBI:         5 (RGB16-565 MCU interface, not used set to default)
 */

#define STM32_ILI9341_PIXSET_PARAM    (ILI9341_PIXEL_FORMAT_SET_DPI(6) | \
                                      ILI9341_PIXEL_FORMAT_SET_DBI(5))

/* DE Mode RCM = 2, Sync Mode RCM = 3
 * Interface Mode Control
 *
 * EPL:         0 High enable for RGB interface
 * DPL:         1 data fetched at the falling time
 * HSPL:        0 Low level sync clock
 * VSPL:        0 Low level sync clock
 * RCM:         2 (DE Mode)
 * ByPass_Mode: 1 (Memory)
 */

#define STM32_ILI9341_IFMODE_PARAM    (ILI9341_INTERFACE_CONTROL_DPL |   \
                                      ILI9341_INTERFACE_CONTROL_RCM(2) | \
                                      ILI9341_INTERFACE_CONTROL_BPASS)

/* Interface control (IFCTL)
 *
 * Parameter 1: 0x0001
 * MY_EOR:  0
 * MX_EOR:  0
 * MV_EOR:  0
 * BGR_EOR: 0
 * WEMODE:  1   Reset column and page if data transfer exceeds
 */

#define STM32_ILI9341_IFCTL_PARAM1    (ILI9341_INTERFACE_CONTROL_WEMODE)

/* Parameter 2: 0x0000
 *
 * EPF:     0   65k color format for RGB interface
 * MDT:     0   Display data transfer mode
 *
 */
#define STM32_ILI9341_IFCTL_PARAM2    (ILI9341_INTERFACE_CONTROL_MDT(0) | \
                                      ILI9341_INTERFACE_CONTROL_EPF(0))

/* Parameter 3: 0x0000/0x0020
 *
 * ENDIAN:  0   Big endian
 * DM:      1   RGB Interface Mode
 * RM:      1   RGB interface
 * RIM:     0   18-bit 1 transfer/pixel RGB interface mode
 *
 */
#define STM32_ILI9341_IFCTL_PARAM3    (ILI9341_INTERFACE_CONTROL_RM | \
                                      ILI9341_INTERFACE_CONTROL_DM(1))

/* Memory access control (MADCTL) */

/* Landscape:   00100000 / 00101000 / h28
 *
 * MY:          0
 * MX:          0
 * MV:          1
 * ML:          0
 * BGR:         0/1 Depending on endian mode of the mcu?
 * MH:          0
 */

#define ILI9341_MADCTL_LANDSCAPE_MY     0
#define ILI9341_MADCTL_LANDSCAPE_MX     0
#define ILI9341_MADCTL_LANDSCAPE_MV     ILI9341_MEMORY_ACCESS_CONTROL_MV
#define ILI9341_MADCTL_LANDSCAPE_ML     0
#ifdef CONFIG_ENDIAN_BIG
#  define ILI9341_MADCTL_LANDSCAPE_BGR  0
#else
#  define ILI9341_MADCTL_LANDSCAPE_BGR  ILI9341_MEMORY_ACCESS_CONTROL_BGR
#endif
#define ILI9341_MADCTL_LANDSCAPE_MH     0

#define ILI9341_MADCTL_LANDSCAPE_PARAM1 (ILI9341_MADCTL_LANDSCAPE_MY | \
                                        ILI9341_MADCTL_LANDSCAPE_MX | \
                                        ILI9341_MADCTL_LANDSCAPE_MV | \
                                        ILI9341_MADCTL_LANDSCAPE_ML | \
                                        ILI9341_MADCTL_LANDSCAPE_BGR | \
                                        ILI9341_MADCTL_LANDSCAPE_MH)

/* Portrait:    00000000 / 00001000 / h08
 *
 * MY:          0
 * MX:          0
 * MV:          0
 * ML:          0
 * BGR:         0/1 Depending on endian mode of the mcu?
 * MH:          0
 */

#define ILI9341_MADCTL_PORTRAIT_MY      0
#define ILI9341_MADCTL_PORTRAIT_MX      ILI9341_MEMORY_ACCESS_CONTROL_MX
#define ILI9341_MADCTL_PORTRAIT_MV      0
#define ILI9341_MADCTL_PORTRAIT_ML      0
#ifdef CONFIG_ENDIAN_BIG
#  define ILI9341_MADCTL_PORTRAIT_BGR   0
#else
#  define ILI9341_MADCTL_PORTRAIT_BGR   ILI9341_MEMORY_ACCESS_CONTROL_BGR
#endif
#define ILI9341_MADCTL_PORTRAIT_MH      0

#define ILI9341_MADCTL_PORTRAIT_PARAM1  (ILI9341_MADCTL_PORTRAIT_MY | \
                                        ILI9341_MADCTL_PORTRAIT_MX | \
                                        ILI9341_MADCTL_PORTRAIT_MV | \
                                        ILI9341_MADCTL_PORTRAIT_ML | \
                                        ILI9341_MADCTL_PORTRAIT_BGR | \
                                        ILI9341_MADCTL_PORTRAIT_MH)

/* RLandscape:  01100000 / 01101000 / h68
 *
 * MY:          0
 * MX:          1
 * MV:          1
 * ML:          0
 * BGR:         0/1 Depending on endian mode of the mcu?
 * MH:          0
 */

#define ILI9341_MADCTL_RLANDSCAPE_MY    0
#define ILI9341_MADCTL_RLANDSCAPE_MX    ILI9341_MEMORY_ACCESS_CONTROL_MX
#define ILI9341_MADCTL_RLANDSCAPE_MV    ILI9341_MEMORY_ACCESS_CONTROL_MV
#define ILI9341_MADCTL_RLANDSCAPE_ML    0
#ifdef CONFIG_ENDIAN_BIG
#  define ILI9341_MADCTL_RLANDSCAPE_BGR 0
#else
#  define ILI9341_MADCTL_RLANDSCAPE_BGR ILI9341_MEMORY_ACCESS_CONTROL_BGR
#endif
#define ILI9341_MADCTL_RLANDSCAPE_MH    0

#define ILI9341_MADCTL_RLANDSCAPE_PARAM1 \
                                        (ILI9341_MADCTL_RLANDSCAPE_MY | \
                                        ILI9341_MADCTL_RLANDSCAPE_MX | \
                                        ILI9341_MADCTL_RLANDSCAPE_MV | \
                                        ILI9341_MADCTL_RLANDSCAPE_ML | \
                                        ILI9341_MADCTL_RLANDSCAPE_BGR | \
                                        ILI9341_MADCTL_RLANDSCAPE_MH)

/* RPortrait:   11000000 / 11001000 / hc8
 *
 * MY:          1
 * MX:          1
 * MV:          0
 * ML:          0
 * BGR:         0/1 Depending on endian mode of the mcu?
 * MH:          0
 *
 */

#define ILI9341_MADCTL_RPORTRAIT_MY     ILI9341_MEMORY_ACCESS_CONTROL_MY
#define ILI9341_MADCTL_RPORTRAIT_MX     0
#define ILI9341_MADCTL_RPORTRAIT_MV     0
#define ILI9341_MADCTL_RPORTRAIT_ML     0
#ifdef CONFIG_ENDIAN_BIG
#  define ILI9341_MADCTL_RPORTRAIT_BGR  0
#else
#  define ILI9341_MADCTL_RPORTRAIT_BGR  ILI9341_MEMORY_ACCESS_CONTROL_BGR
#endif
#define ILI9341_MADCTL_RPORTRAIT_MH     0

#define ILI9341_MADCTL_RPORTRAIT_PARAM1 (ILI9341_MADCTL_RPORTRAIT_MY | \
                                        ILI9341_MADCTL_RPORTRAIT_MX | \
                                        ILI9341_MADCTL_RPORTRAIT_MV | \
                                        ILI9341_MADCTL_RPORTRAIT_ML | \
                                        ILI9341_MADCTL_RPORTRAIT_BGR | \
                                        ILI9341_MADCTL_RPORTRAIT_MH)

/* Set the display orientation */

#if defined(CONFIG_STM32F429I_DISCO_ILI9341_FBIFACE_LANDSCAPE)
#  define STM32_ILI9341_MADCTL_PARAM  ILI9341_MADCTL_LANDSCAPE_PARAM1
# warning "ILI9341 doesn't support full landscape with RGB interface"
#elif defined(CONFIG_STM32F429I_DISCO_ILI9341_FBIFACE_PORTRAIT)
#  define STM32_ILI9341_MADCTL_PARAM  ILI9341_MADCTL_PORTRAIT_PARAM1
#elif defined(CONFIG_STM32F429I_DISCO_ILI9341_FBIFACE_RLANDSCAPE)
#  define STM32_ILI9341_MADCTL_PARAM  ILI9341_MADCTL_RLANDSCAPE_PARAM1
# warning "ILI9341 doesn't support full landscape with RGB interface"
#elif defined(CONFIG_STM32F429I_DISCO_ILI9341_FBIFACE_RPORTRAIT)
#  define STM32_ILI9341_MADCTL_PARAM  ILI9341_MADCTL_RPORTRAIT_PARAM1
#else
# error "display orientation not defined"
#endif

#define ILI9341_XRES BOARD_LTDC_WIDTH
#define ILI9341_YRES BOARD_LTDC_HEIGHT
#endif /* CONFIG_STM32F429I_DISCO_ILI9341_FBIFACE */

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_STM32F429I_DISCO_ILI9341_LCDIFACE
struct lcd_dev_s *g_lcd = NULL;
#endif

#ifdef CONFIG_STM32F429I_DISCO_ILI9341_FBIFACE
struct ili9341_lcd_s *g_ltdc = NULL;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_STM32F429I_DISCO_ILI9341_FBIFACE
/****************************************************************************
 * Name: stm32_ili9341_initialize
 *
 * Description:
 *   Initialize the ili9341 LCD controller to the RGB interface mode.
 *
 ****************************************************************************/

static int stm32_ili9341_initialize(void)
{
  struct ili9341_lcd_s *lcd = g_ltdc;

  lcd = stm32_ili93414ws_initialize();

  if (lcd == NULL)
    {
      return ENODEV;
    }

  /* Select spi device */

  lcdinfo("Initialize ili9341 lcd driver\n");
  lcd->select(lcd);

#ifdef CONFIG_DEBUG_LCD_INFO
  /* Read display identification */

  uint8_t param;
  lcd->sendcmd(lcd, ILI9341_READ_ID1);
  lcd->recvparam(lcd, &param);
  lcdinfo("ili9341 LCD driver: LCD modules manufacturer ID: %d\n", param);

  lcd->sendcmd(lcd, ILI9341_READ_ID2);
  lcd->recvparam(lcd, &param);
  lcdinfo("ili9341 LCD driver: LCD modules driver version ID: %d\n", param);

  lcd->sendcmd(lcd, ILI9341_READ_ID3);
  lcd->recvparam(lcd, &param);
  lcdinfo("ili9341 LCD driver: LCD modules driver ID: %d\n", param);
#endif

  /* Reset the lcd display to the default state */

  lcdinfo("ili9341 LCD driver: Software Reset\n");
  lcd->sendcmd(lcd, ILI9341_SOFTWARE_RESET);
  up_mdelay(5);

  lcdinfo("ili9341 LCD driver: set Memory Access Control %08x\n",
        STM32_ILI9341_MADCTL_PARAM);
  lcd->sendcmd(lcd, ILI9341_MEMORY_ACCESS_CONTROL);
  lcd->sendparam(lcd, STM32_ILI9341_MADCTL_PARAM);

  /* Pixel Format */

  lcdinfo("ili9341 LCD driver: Set Pixel Format: %02x\n",
          STM32_ILI9341_PIXSET_PARAM);
  lcd->sendcmd(lcd, ILI9341_PIXEL_FORMAT_SET);
  lcd->sendparam(lcd, STM32_ILI9341_PIXSET_PARAM);

  /* Select column */

  lcdinfo("ili9341 LCD driver: Set Column Address\n");
  lcd->sendcmd(lcd, ILI9341_COLUMN_ADDRESS_SET);
  lcd->sendparam(lcd, 0);
  lcd->sendparam(lcd, 0);
  lcd->sendparam(lcd, (ILI9341_XRES >> 8));
  lcd->sendparam(lcd, (ILI9341_XRES & 0xff));

  /* Select page */

  lcdinfo("ili9341 LCD driver: Set Page Address\n");
  lcd->sendcmd(lcd, ILI9341_PAGE_ADDRESS_SET);
  lcd->sendparam(lcd, 0);
  lcd->sendparam(lcd, 0);
  lcd->sendparam(lcd, (ILI9341_YRES >> 8));
  lcd->sendparam(lcd, (ILI9341_YRES & 0xff));

  /* RGB Interface signal control */

  lcdinfo("ili9341 LCD driver: Set RGB Interface signal control: %02x\n",
          STM32_ILI9341_IFMODE_PARAM);
  lcd->sendcmd(lcd, ILI9341_RGB_SIGNAL_CONTROL);
  lcd->sendparam(lcd, STM32_ILI9341_IFMODE_PARAM);

  /* Interface control */

  lcdinfo("ili9341 LCD driver: Set Interface control: %d:%d:%d\n",
          STM32_ILI9341_IFCTL_PARAM1,
          STM32_ILI9341_IFCTL_PARAM2,
          STM32_ILI9341_IFCTL_PARAM3);

  lcd->sendcmd(lcd, ILI9341_INTERFACE_CONTROL);
  lcd->sendparam(lcd, STM32_ILI9341_IFCTL_PARAM1);
  lcd->sendparam(lcd, STM32_ILI9341_IFCTL_PARAM2);
  lcd->sendparam(lcd, STM32_ILI9341_IFCTL_PARAM3);

  /* Sleep out set to the end */

  lcdinfo("ili9341 LCD driver: Sleep Out\n");
  lcd->sendcmd(lcd, ILI9341_SLEEP_OUT);
  up_mdelay(5); /* 120? */

  /* Display on */

  lcdinfo("ili9341 LCD driver: Display On\n");
  lcd->sendcmd(lcd, ILI9341_DISPLAY_ON);

  /* Deselect spi device */

  lcd->deselect(lcd);

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_STM32F429I_DISCO_ILI9341_LCDIFACE
/****************************************************************************
 * Name: board_lcd_uninitialize
 *
 * Description:
 *   Uninitialize the LCD Device.
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

void board_lcd_uninitialize(void)
{
  /* Set display off */

  g_lcd->setpower(g_lcd, 0);

  g_lcd = NULL;
}

/****************************************************************************
 * Name: board_lcd_getdev
 *
 * Description:
 *   Return a reference to the LCD object for the specified LCD Device.
 *   This allows support for multiple LCD devices.
 *
 * Input Parameters:
 *   lcddev - Number of the LDC Device.
 *
 * Returned Value:
 *   Reference to the LCD object if exist otherwise NULL
 *
 ****************************************************************************/

struct lcd_dev_s *board_lcd_getdev(int lcddev)
{
  if (lcddev == ILI9341_LCD_DEVICE)
    {
      return g_lcd;
    }

  return NULL;
}

/****************************************************************************
 * Name: board_lcd_initialize
 *
 * Description:
 *   Initialize the LCD video hardware. The initial state of the LCD is
 *   fully initialized, display memory cleared, and the LCD ready to use, but
 *   with the power setting at 0 (full off).
 *
 * Input Parameters:
 *
 * Returned Value:
 *   On success - Ok
 *   On error   - Error Code
 *
 ****************************************************************************/

int board_lcd_initialize(void)
{
  /* check if always initialized */

  if (!g_lcd)
    {
      /* Initialize the sub driver structure */

      struct ili9341_lcd_s *dev = stm32_ili93414ws_initialize();

      /* Initialize public lcd driver structure */

      if (dev)
        {
          /* Get a reference to valid lcd driver structure to avoid repeated
           * initialization of the LCD Device. Also enables uninitializing of
           * the LCD Device.
           */

          g_lcd = ili9341_initialize(dev, ILI9341_LCD_DEVICE);
          if (g_lcd)
            {
              return OK;
            }
        }

      return -ENODEV;
    }

  return OK;
}
#endif /* CONFIG_STM32F429I_DISCO_ILI9341_LCDIFACE */

#ifdef CONFIG_STM32_LTDC
/****************************************************************************
 * Name: up_fbinitialize
 *
 * Description:
 *   Initialize the framebuffer video hardware associated with the display.
 *
 * Input Parameters:
 *   display - In the case of hardware with multiple displays, this
 *     specifies the display.  Normally this is zero.
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure.
 *
 ****************************************************************************/

int up_fbinitialize(int display)
{
  static bool initialized = false;
  int ret = OK;

  if (!initialized)
    {
#ifdef CONFIG_STM32F429I_DISCO_ILI9341_FBIFACE
      /* Initialize the ili9341 LCD controller */

      ret = stm32_ili9341_initialize();
      if (ret >= OK)
        {
          ret = stm32_ltdcinitialize();
        }

#else
      /* Custom LCD display with RGB interface */

      ret = stm32_ltdcinitialize();
#endif

      initialized = (ret >= OK);
    }

  return ret;
}

/****************************************************************************
 * Name: up_fbgetvplane
 *
 * Description:
 *   Return a a reference to the framebuffer object for the specified video
 *   plane of the specified plane.
 *   Many OSDs support multiple planes of video.
 *
 * Input Parameters:
 *   display - In the case of hardware with multiple displays, this
 *     specifies the display.  Normally this is zero.
 *   vplane - Identifies the plane being queried.
 *
 * Returned Value:
 *   A non-NULL pointer to the frame buffer access structure is returned on
 *   success; NULL is returned on any failure.
 *
 ****************************************************************************/

struct fb_vtable_s *up_fbgetvplane(int display, int vplane)
{
  return stm32_ltdcgetvplane(vplane);
}

/****************************************************************************
 * Name: up_fbuninitialize
 *
 * Description:
 *   Uninitialize the framebuffer support for the specified display.
 *
 * Input Parameters:
 *   display - In the case of hardware with multiple displays, this
 *     specifies the display.  Normally this is zero.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void up_fbuninitialize(int display)
{
  stm32_ltdcuninitialize();
}
#endif /* CONFIG_STM32_LTDC */
