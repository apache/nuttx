/****************************************************************************
 * boards/xtensa/esp32s3/esp32s3-box/src/esp32s3_board_lcd_ili9342c.c
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

#include <stdio.h>
#include <stdbool.h>
#include <debug.h>
#include <errno.h>
#include <sys/param.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/signal.h>
#include <nuttx/spi/spi.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/ili9341.h>

#include <arch/board/board.h>

#include "esp32s3_gpio.h"
#include "esp32s3_spi.h"
#include "hardware/esp32s3_gpio_sigmap.h"

#include "esp32s3-box.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ILI9342C_FREQUENCY   (40 * 1000 * 1000)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ili9342c_config_data
{
  uint8_t cmd;
  uint8_t data_bytes;
  uint8_t data[16];
};

struct ili9342c_lcd_dev
{
  struct ili9341_lcd_s ili9342c_lcd;
  struct spi_dev_s *spi_dev;
  struct lcd_dev_s *lcd;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* ILI9342C Initialization Configuration */

static const struct ili9342c_config_data g_lcd_config[] =
{
  {
    /* SW reset */

    ILI9341_SOFTWARE_RESET, 0
  },
  {
    /* 200 ms delay */

    0x80, 1,
    {
      0xfa
    }
  },
  {
    /* Power contorl B, power control = 0, DC_ENA = 1 */

    ILI9341_POWER_CONTROL_B, 3,
    {
      0x00, 0xaa, 0xe0
    }
  },
  {
    /* Power on sequence control */

    ILI9341_POWER_ON_SEQUENCE_CONTROL, 4,
    {
      0x67, 0x03, 0x12, 0x81
    }
  },
  {
    /* Driver timing control A */

    ILI9341_DRIVER_TIMING_CTL_A, 3,
    {
      0x8a, 0x01, 0x78
    }
  },
  {
    /* Power control A, Vcore=1.6V, DDVDH=5.6V */

    ILI9341_POWER_CONTROL_A, 5,
    {
      0x39, 0x2c, 0x00, 0x34, 0x02
    }
  },
  {
    /* Pump ratio control, DDVDH=2xVCl */

    ILI9341_PUMP_RATIO_CONTROL, 1,
    {
      0x20
    }
  },
  {
    /* Driver timing control, all=0 unit */

    ILI9341_DRIVER_TIMING_CTL_B, 2,
    {
      0x00, 0x00
    }
  },
  {
    /* Power control 1, GVDD=4.75V */

    ILI9341_POWER_CONTROL_1, 1,
    {
      0x23
    }
  },
  {
    /* Power control 2, DDVDH=VCl*2, VGH=VCl*7, VGL=-VCl*3 */

    ILI9341_POWER_CONTROL_2, 1,
    {
      0x11
    }
  },
  {
    /* VCOM control 1, VCOMH=4.025V, VCOML=-0.950V */

    ILI9341_VCOM_CONTROL_1, 2,
    {
      0x43, 0x4c
    }
  },
  {
    /* VCOM control 2, VCOMH=VMH-2, VCOML=VML-2 */

    ILI9341_VCOM_CONTROL_2, 1,
    {
      0xa0
    }
  },
  {
    /* Memory access contorl, MX=MY=0, MV=1, ML=0, BGR=1, MH=0 */

    ILI9341_MEMORY_ACCESS_CONTROL, 1,
    {
      0x28
    }
  },
  {
    /* Pixel format, 16bits/pixel for RGB/MCU interface */

    ILI9341_PIXEL_FORMAT_SET, 1,
    {
      0x55
    }
  },
  {
    /* Frame rate control, f=fosc, 70Hz fps */

    ILI9341_FRAME_RATE_CONTROL_NORMAL, 2,
    {
      0x00, 0x1b
    }
  },
  {
    /* Disable 3G gamma */

    ILI9341_ENABLE_3_GAMMA_CONTROL, 1,
    {
      0x0
    }
  },
  {
    /* Gamma set, curve 1 */

    ILI9341_GAMMA_SET, 1,
    {
      0x01
    }
  },
  {
    /* Positive gamma correction */

    ILI9341_POSITIVE_GAMMA_CORRECTION, 15,
    {
      0x1f, 0x36, 0x36, 0x3a, 0x0c, 0x05, 0x4f, 0x87,
      0x3c, 0x08, 0x11, 0x35, 0x19, 0x13, 0x00
    }
  },
  {
    /* Negative gamma correction */

    ILI9341_NEGATIVE_GAMMA_CORRECTION, 15,
    {
      0x00, 0x09, 0x09, 0x05, 0x13, 0x0a, 0x30, 0x78,
      0x43, 0x07, 0x0e, 0x0a, 0x26, 0x2c, 0x1f
    }
  },
  {
    /* Column address set, SC=0, EC=0xEF */

    ILI9341_COLUMN_ADDRESS_SET, 4,
    {
      0x00, 0x00, 0x00, 0xef
    }
  },
  {
    /* Page address set, SP=0, EP=0x013F */

    ILI9341_PAGE_ADDRESS_SET, 4,
    {
      0x00, 0x00, 0x01, 0x3f
    }
  },
  {
    /* Memory write */

    ILI9341_MEMORY_WRITE, 0
  },
  {
    /* Entry mode set, Low vol detect disabled, normal display */

    ILI9341_ENTRY_MODE_SET, 1,
    {
      0x07
    }
  },
  {
    /* Display function control */

    ILI9341_DISPLAY_FUNCTION_CTL, 3,
    {
      0x08, 0x82, 0x27
    }
  },
  {
    /* Sleep out */

    ILI9341_SLEEP_OUT, 0
  },
  {
    /* Display on */

    ILI9341_DISPLAY_ON, 0
  },
  {
    /* Invert colors */

    ILI9341_DISP_INVERSION_OFF, 0,
  },
  {
    ILI9341_MEMORY_ACCESS_CONTROL, 1,
    {
      0xc8
    }
  }
};

static struct ili9342c_lcd_dev g_lcddev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ili9342c_select
 *
 * Description:
 *   Select the SPI, locking and re-configuring if necessary
 *
 * Input Parameters:
 *   spi  - Reference to the public driver structure
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void ili9342c_select(struct ili9341_lcd_s *lcd)
{
  struct ili9342c_lcd_dev *priv = (struct ili9342c_lcd_dev *)lcd;

  SPI_LOCK(priv->spi_dev, true);
  SPI_SELECT(priv->spi_dev, SPIDEV_DISPLAY(0), true);
}

/****************************************************************************
 * Name: ili9342c_deselect
 *
 * Description:
 *   De-select the SPI
 *
 * Input Parameters:
 *   spi  - Reference to the public driver structure
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void ili9342c_deselect(struct ili9341_lcd_s *lcd)
{
  struct ili9342c_lcd_dev *priv = (struct ili9342c_lcd_dev *)lcd;

  SPI_SELECT(priv->spi_dev, SPIDEV_DISPLAY(0), false);
  SPI_LOCK(priv->spi_dev, false);
}

/****************************************************************************
 * Name: ili9342c_sendcmd
 *
 * Description:
 *   Send a command to the lcd driver.
 *
 * Input Parameters:
 *   lcd  - Reference to the ili9341_lcd_s driver structure
 *   cmd  - command to send
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int ili9342c_sendcmd(struct ili9341_lcd_s *lcd, const uint8_t cmd)
{
  struct ili9342c_lcd_dev *priv = (struct ili9342c_lcd_dev *)lcd;

  lcdinfo("cmd=%02x\n", cmd);

  SPI_SETBITS(priv->spi_dev, 8);

  SPI_CMDDATA(priv->spi_dev, SPIDEV_DISPLAY(0), true);
  SPI_SEND(priv->spi_dev, cmd);

  return 0;
}

/****************************************************************************
 * Name: stm32_ili93414ws_sendparam
 *
 * Description:
 *   Send a parameter to the lcd driver.
 *
 * Input Parameters:
 *   lcd    - Reference to the ili9341_lcd_s driver structure
 *   param  - parameter to send
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int ili9342c_sendparam(struct ili9341_lcd_s *lcd, const uint8_t param)
{
  struct ili9342c_lcd_dev *priv = (struct ili9342c_lcd_dev *)lcd;

  lcdinfo("param=%02x\n", param);

  SPI_SETBITS(priv->spi_dev, 8);

  SPI_CMDDATA(priv->spi_dev, SPIDEV_DISPLAY(0), false);
  SPI_SEND(priv->spi_dev, param);

  return 0;
}

/****************************************************************************
 * Name: stm32_ili93414ws_sendgram
 *
 * Description:
 *   Send a number of pixel words to the lcd driver gram.
 *
 * Input Parameters:
 *   lcd    - Reference to the ili9341_lcd_s driver structure
 *   wd     - Reference to the words to send
 *   nwords - number of words to send
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int ili9342c_sendgram(struct ili9341_lcd_s *lcd,
                             const uint16_t *wd,
                             uint32_t nwords)
{
  struct ili9342c_lcd_dev *priv = (struct ili9342c_lcd_dev *)lcd;

  lcdinfo("lcd:%p, wd=%p, nwords=%" PRIu32 "\n", lcd, wd, nwords);

  SPI_SETBITS(priv->spi_dev, 16);

  SPI_CMDDATA(priv->spi_dev, SPIDEV_DISPLAY(0), false);
  SPI_SNDBLOCK(priv->spi_dev, wd, nwords);

  return 0;
}

/****************************************************************************
 * Name: stm32_ili93414ws_backlight
 *
 * Description:
 *   Set the backlight level of the connected display.
 *
 * Input Parameters:
 *   spi   - Reference to the public driver structure
 *   level - backligth level
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int ili9342c_backlight(struct ili9341_lcd_s *lcd, int level)
{
  if (level > 0)
    {
      lcd->sendcmd(lcd, ILI9341_WRITE_CTRL_DISPLAY);
      lcd->sendparam(lcd, 0x24);
    }
  else
    {
      lcd->sendcmd(lcd, ILI9341_WRITE_CTRL_DISPLAY);
      lcd->sendparam(lcd, 0x0);
    }

  return 0;
}

/****************************************************************************
 * Name:  esp32s3_initializa_ili9342c
 *
 * Description:
 *   Initialize the device structure to control the LCD Single chip driver.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   On success, this function returns a reference to the LCD control object
 *   for the specified ILI9341 LCD Single chip driver connected as 4 wire
 *   serial (spi). NULL is returned on any failure.
 *
 ****************************************************************************/

static struct ili9341_lcd_s *esp32s3_initializa_ili9342c(int spi_port)
{
  struct ili9341_lcd_s *ili9342c_lcd = &g_lcddev.ili9342c_lcd;

  /* Initialize non-SPI GPIOs */

  esp32s3_configgpio(DISPLAY_DC, OUTPUT);
  esp32s3_configgpio(DISPLAY_RST, OUTPUT);
  esp32s3_configgpio(DISPLAY_BCKL, OUTPUT);

  /* Reset LCD */

  nxsig_usleep(10 * 1000);
  esp32s3_gpiowrite(DISPLAY_RST, true);
  nxsig_usleep(10 * 1000);
  esp32s3_gpiowrite(DISPLAY_RST, false);
  nxsig_usleep(50 * 1000);

  /* Turn on LCD backlight */

  esp32s3_gpiowrite(DISPLAY_BCKL, true);

  g_lcddev.spi_dev = esp32s3_spibus_initialize(spi_port);
  if (!g_lcddev.spi_dev)
    {
      lcderr("ERROR: Failed to initialize SPI port %d\n", spi_port);
      return NULL;
    }

  SPI_SETMODE(g_lcddev.spi_dev, SPIDEV_MODE0);
  SPI_SETBITS(g_lcddev.spi_dev, 8);
  SPI_SETFREQUENCY(g_lcddev.spi_dev, ILI9342C_FREQUENCY);

  ili9342c_lcd->select    = ili9342c_select;
  ili9342c_lcd->deselect  = ili9342c_deselect;
  ili9342c_lcd->sendcmd   = ili9342c_sendcmd;
  ili9342c_lcd->sendparam = ili9342c_sendparam;
  ili9342c_lcd->recvparam = NULL;
  ili9342c_lcd->sendgram  = ili9342c_sendgram;
  ili9342c_lcd->recvgram  = NULL;
  ili9342c_lcd->backlight = ili9342c_backlight;

  return ili9342c_lcd;
}

/****************************************************************************
 * Name: ili9342c_configure
 *
 * Description:
 *   Configuration ILI9342C with given command and parameters.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void ili9342c_configure(struct ili9341_lcd_s *lcd,
                               uint8_t cmd,
                               uint8_t data_bytes,
                               const uint8_t *data)
{
  lcd->select(lcd);
  lcd->sendcmd(lcd, cmd);
  if (data_bytes)
    {
      for (int i = 0; i < data_bytes; i++)
        {
          lcd->sendparam(lcd, data[i]);
        }
    }

  lcd->deselect(lcd);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  board_lcd_initialize
 *
 * Description:
 *   Initialize the LCD video hardware.  The initial state of the LCD is
 *   fully initialized, display memory cleared, and the LCD ready to use, but
 *   with the power setting at 0 (full off).
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_lcd_initialize(void)
{
  struct ili9341_lcd_s *ili9342c_lcd;

  ili9342c_lcd = esp32s3_initializa_ili9342c(DISPLAY_SPI);
  if (!ili9342c_lcd)
    {
      lcderr("ERROR: Failed to initialize ILI9341\n");
      return -ENODEV;
    }

  g_lcddev.lcd = ili9341_initialize(ili9342c_lcd, 0);
  if (!g_lcddev.lcd)
    {
      lcderr("ERROR: st7789_lcdinitialize() failed\n");
      return -ENODEV;
    }

  for (int i = 0; i < nitems(g_lcd_config); i++)
    {
      ili9342c_configure(ili9342c_lcd,
                         g_lcd_config[i].cmd,
                         g_lcd_config[i].data_bytes,
                         g_lcd_config[i].data);
    }

  return OK;
}

/****************************************************************************
 * Name:  board_lcd_getdev
 *
 * Description:
 *   Return a a reference to the LCD object for the specified LCD.  This
 *   allows support for multiple LCD devices.
 *
 * Input Parameters:
 *   devno - LCD device nmber
 *
 * Returned Value:
 *   LCD device pointer if success or NULL if failed.
 *
 ****************************************************************************/

struct lcd_dev_s *board_lcd_getdev(int devno)
{
  if (!g_lcddev.lcd)
    {
      lcderr("ERROR: Failed to bind SPI port 4 to LCD %d\n", devno);
    }
  else
    {
      lcdinfo("SPI port %d bound to LCD %d\n", DISPLAY_SPI, devno);
      return g_lcddev.lcd;
    }

  return NULL;
}

/****************************************************************************
 * Name:  board_lcd_uninitialize
 *
 * Description:
 *   Uninitialize the LCD support
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void board_lcd_uninitialize(void)
{
  /* Turn the display off */

  g_lcddev.lcd->setpower(g_lcddev.lcd, 0);
}
