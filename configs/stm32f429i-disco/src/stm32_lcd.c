/************************************************************************************
 * configs/stm32f429i-disco/src/stm32_lcd.c
 *
 *   Copyright (C) 2014-2015 Marco Krahl. All rights reserved.
 *   Author: Marco Krahl <ocram.lhark@gmail.com>
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

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/lcd/ili9341.h>
#include <nuttx/video/fb.h>

#include <arch/chip/ltdc.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "stm32f429i-disco.h"
#include "stm32_ltdc.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
#ifdef CONFIG_STM32F429I_DISCO_ILI9341_LCDDEVICE
# define ILI9341_LCD_DEVICE CONFIG_STM32F429I_DISCO_ILI9341_LCDDEVICE
#else
# define ILI9341_LCD_DEVICE  0
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
 * Interface Mode Controle
 *
 * EPL:         0 High enable for RGB interface
 * DPL:         1 data fetched at the falling time
 * HSPL:        0 Low level sync clock
 * VSPL:        0 Low level sync clock
 * RCM:         2 (DE Mode)
 * ByPass_Mode: 1 (Memory)
 */

#define STM32_ILI9341_IFMODE_PARAM    ((!ILI9341_INTERFACE_CONTROL_EPL) | \
                                      ILI9341_INTERFACE_CONTROL_DPL | \
                                      (!ILI9341_INTERFACE_CONTROL_HSPL) | \
                                      (!ILI9341_INTERFACE_CONTROL_VSPL) | \
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

#define STM32_ILI9341_IFCTL_PARAM1    (ILI9341_INTERFACE_CONTROL_WEMODE | \
                                      !ILI9341_INTERFACE_CONTROL_BGREOR | \
                                      !ILI9341_INTERFACE_CONTROL_MVEOR | \
                                      !ILI9341_INTERFACE_CONTROL_MXEOR | \
                                      !ILI9341_INTERFACE_CONTROL_MYEOR)

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
#define STM32_ILI9341_IFCTL_PARAM3    ((!ILI9341_INTERFACE_CONTROL_RIM) | \
                                      ILI9341_INTERFACE_CONTROL_RM | \
                                      ILI9341_INTERFACE_CONTROL_DM(1) | \
                                      (!ILI9341_INTERFACE_CONTROL_ENDIAN))

/* Memory access control (MADCTL) */

/*
 * Landscape:   00100000 / 00101000 / h28
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
#ifdef CONFIG_BIG_ENDIAN
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

/*
 * Portrait:    00000000 / 00001000 / h08
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
#ifdef CONFIG_BIG_ENDIAN
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

/*
 * RLandscape:  01100000 / 01101000 / h68
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
#ifdef CONFIG_BIG_ENDIAN
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

/*
 * RPortrait:   11000000 / 11001000 / hc8
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
#ifdef CONFIG_BIG_ENDIAN
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
# define STM32_ILI9341_MADCTL_PARAM  ILI9341_MADCTL_LANDSCAPE_PARAM1
# warning "ILI9341 doesn't support full landscape with RGB interface"
#elif defined(CONFIG_STM32F429I_DISCO_ILI9341_FBIFACE_PORTRAIT)
# define STM32_ILI9341_MADCTL_PARAM  ILI9341_MADCTL_PORTRAIT_PARAM1
#elif defined(CONFIG_STM32F429I_DISCO_ILI9341_FBIFACE_RLANDSCAPE)
# define STM32_ILI9341_MADCTL_PARAM  ILI9341_MADCTL_RLANDSCAPE_PARAM1
# warning "ILI9341 doesn't support full landscape with RGB interface"
#elif defined(CONFIG_STM32F429I_DISCO_ILI9341_FBIFACE_RPORTRAIT)
# define STM32_ILI9341_MADCTL_PARAM  ILI9341_MADCTL_RPORTRAIT_PARAM1
#else
# error "display orientation not defined"
#endif

#define ILI9341_XRES BOARD_LTDC_WIDTH
#define ILI9341_YRES BOARD_LTDC_HEIGHT
#endif /* CONFIG_STM32F429I_DISCO_ILI9341_FBIFACE */

/******************************************************************************
 * Private Data
 ******************************************************************************/

#ifdef CONFIG_STM32F429I_DISCO_ILI9341_LCDIFACE
FAR struct lcd_dev_s *g_lcd = NULL;
#endif

#ifdef CONFIG_STM32F429I_DISCO_ILI9341_FBIFACE
FAR struct ili9341_lcd_s *g_ltdc = NULL;
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/

#ifdef CONFIG_STM32F429I_DISCO_ILI9341_FBIFACE
/*******************************************************************************
 * Name: stm32_ili9341_initialize
 *
 * Description:
 *   Initialize the ili9341 LCD controller to the RGB interface mode.
 *
 ******************************************************************************/

static int stm32_ili9341_initialize(void)
{
  FAR struct ili9341_lcd_s *lcd = g_ltdc;

  lcd = stm32_ili93414ws_initialize();

  if (lcd == NULL)
    {
      return ENODEV;
    }

  /* Select spi device */

  dbg("Initialize ili9341 lcd driver\n");
  lcd->select(lcd);

#ifdef CONFIG_DEBUG_LCD
  /* Read display identification */

  lcd->sendcmd(lcd, ILI9341_READ_ID1);
  lcd->recvparam(lcd, &param);
  dbg("ili9341 LCD driver: LCD modules manufacturer ID: %d\n", param);
  lcd->sendcmd(lcd, ILI9341_READ_ID2);
  lcd->recvparam(lcd, &param);
  dbg("ili9341 LCD driver: LCD modules driver version ID: %d\n", param);
  lcd->sendcmd(lcd, ILI9341_READ_ID3);
  lcd->recvparam(lcd, &param);
  dbg("ili9341 LCD driver: LCD modules driver ID: %d\n", param);
#endif

  /* Reset the lcd display to the default state */

  vdbg("ili9341 LCD driver: Software Reset\n");
  lcd->sendcmd(lcd, ILI9341_SOFTWARE_RESET);
  up_mdelay(5);

  vdbg("ili9341 LCD driver: set Memory Access Control %08x\n",
        STM32_ILI9341_MADCTL_PARAM);
  lcd->sendcmd(lcd, ILI9341_MEMORY_ACCESS_CONTROL);
  lcd->sendparam(lcd, STM32_ILI9341_MADCTL_PARAM);

  /* Pixel Format */

  vdbg("ili9341 LCD driver: Set Pixel Format: %02x\n",
          STM32_ILI9341_PIXSET_PARAM);
  lcd->sendcmd(lcd, ILI9341_PIXEL_FORMAT_SET);
  lcd->sendparam(lcd, STM32_ILI9341_PIXSET_PARAM);

  /* Select column */

  vdbg("ili9341 LCD driver: Set Column Address\n");
  lcd->sendcmd(lcd, ILI9341_COLUMN_ADDRESS_SET);
  lcd->sendparam(lcd, 0);
  lcd->sendparam(lcd, 0);
  lcd->sendparam(lcd, (ILI9341_XRES >> 8));
  lcd->sendparam(lcd, (ILI9341_XRES & 0xff));

  /* Select page */

  vdbg("ili9341 LCD driver: Set Page Address\n");
  lcd->sendcmd(lcd, ILI9341_PAGE_ADDRESS_SET);
  lcd->sendparam(lcd, 0);
  lcd->sendparam(lcd, 0);
  lcd->sendparam(lcd, (ILI9341_YRES >> 8));
  lcd->sendparam(lcd, (ILI9341_YRES & 0xff));

  /* RGB Interface signal control */

  vdbg("ili9341 LCD driver: Set RGB Interface signal control: %02x\n",
          STM32_ILI9341_IFMODE_PARAM);
  lcd->sendcmd(lcd, ILI9341_RGB_SIGNAL_CONTROL);
  lcd->sendparam(lcd, STM32_ILI9341_IFMODE_PARAM);

  /* Interface control */

  vdbg("ili9341 LCD driver: Set Interface control: %d:%d:%d\n",
          STM32_ILI9341_IFCTL_PARAM1,
          STM32_ILI9341_IFCTL_PARAM2,
          STM32_ILI9341_IFCTL_PARAM3);

  lcd->sendcmd(lcd, ILI9341_INTERFACE_CONTROL);
  lcd->sendparam(lcd, STM32_ILI9341_IFCTL_PARAM1);
  lcd->sendparam(lcd, STM32_ILI9341_IFCTL_PARAM2);
  lcd->sendparam(lcd, STM32_ILI9341_IFCTL_PARAM3);

  /* Sleep out set to the end */

  vdbg("ili9341 LCD driver: Sleep Out\n");
  lcd->sendcmd(lcd, ILI9341_SLEEP_OUT);
  up_mdelay(5); /* 120? */

  /* Display on */

  vdbg("ili9341 LCD driver: Display On\n");
  lcd->sendcmd(lcd, ILI9341_DISPLAY_ON);

  /* Deselect spi device */

  lcd->deselect(lcd);

  return OK;
}
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#ifdef CONFIG_STM32F429I_DISCO_ILI9341_LCDIFACE
/****************************************************************************
 * Name: board_lcd_uninitialize
 *
 * Description:
 *   Unitialize the LCD Device.
 *
 * Parameter:
 *
 * Return:
 *
 ***************************************************************************/

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
 * Parameter:
 *   lcddev - Number of the LDC Device.
 *
 * Return:
 *   Reference to the LCD object if exist otherwise NULL
 *
 ***************************************************************************/

FAR struct lcd_dev_s *board_lcd_getdev(int lcddev)
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
 * Parameter:
 *
 * Return:
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

      FAR struct ili9341_lcd_s *dev = stm32_ili93414ws_initialize();

      /* Initialize public lcd driver structure */

      if (dev)
        {
          /*
           * Get a reference to valid lcd driver structure to avoid repeated
           * initialization of the LCD Device. Also enables uninitializing of
           * the LCD Device.
           */

          g_lcd = ili9341_initialize(dev, ILI9341_LCD_DEVICE);
          if (g_lcd)
            {
              return OK;
            }
        }

      return -errno;
    }

  return OK;
}
#endif /* CONFIG_STM32F429I_DISCO_ILI9341_LCDIFACE */

#ifdef CONFIG_STM32_LTDC
/*******************************************************************************
 * Name: up_fbinitialize
 *
 * Description:
 *   The generic method to initialize the framebuffer device
 *
 * Return:
 *   OK - On succes
 *
 ******************************************************************************/

int up_fbinitialize(void)
{
#ifdef CONFIG_STM32F429I_DISCO_ILI9341_FBIFACE
  int ret;

  /* Initialize the ili9341 LCD controller */

  ret = stm32_ili9341_initialize();

  if (ret == OK)
    {
      ret = stm32_ltdcinitialize();
    }

  return ret;

#else
  /* Custom LCD display with RGB interface */

  return stm32_ltdcinitialize();
#endif
}

/*******************************************************************************
 * Name: up_fbgetvplane
 *
 * Description:
 *   The generic method to get the videoplane.
 *
 * Paramater:
 *   vplane - Number othe video plane
 *
 * Return:
 *   Reference to the fb_vtable_s on success otherwise NULL.
 *
 ******************************************************************************/

FAR struct fb_vtable_s *up_fbgetvplane(int vplane)
{
  return stm32_ltdcgetvplane(vplane);
}

/******************************************************************************
 * Name: up_uninitialize
 *
 * Description:
 *   The generic method to uninitialize the framebuffer device
 *
 ******************************************************************************/

void fb_uninitialize(void)
{
  stm32_ltdcuninitialize();
}

/******************************************************************************
 * Name: up_ltdcgetlayer
 *
 * Description:
 *   The application function name to get a reference to the hardware layer of
 *   the ltdc device.
 *
 * Parameter:
 *   lid - The specific layer identifier
 *
 ******************************************************************************/

#ifdef CONFIG_STM32_LTDC_INTERFACE
FAR struct ltdc_layer_s *up_ltdcgetlayer(int lid)
{
  return stm32_ltdcgetlayer(lid);
}
#endif

#endif /* CONFIG_STM32_LTDC */
