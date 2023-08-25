/****************************************************************************
 * boards/xtensa/esp32s3/esp32s3-lcd-ev/src/esp32s3_lcd.c
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

#include <unistd.h>
#include <stdlib.h>
#include <debug.h>
#include <assert.h>
#include <sys/param.h>
#include <unistd.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/video/fb.h>
#include <nuttx/signal.h>

#include "esp32s3_gpio.h"
#include "esp32s3-lcd-ev.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CS_PIN      1
#define SCK_PIN     2
#define SDO_PIN     3

#define DELAY(us)   esp_rom_delay_us(us)
#define CS(v)       DEBUGASSERT(board_ioexpander_output(CS_PIN, v ? 1 : 0) == 0)
#define SCK(v)      DEBUGASSERT(board_ioexpander_output(SCK_PIN, v ? 1 : 0) == 0)
#define SDO(v)      DEBUGASSERT(board_ioexpander_output(SDO_PIN, v ? 1 : 0) == 0)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct lcd_config_data
{
  uint8_t cmd;
  uint8_t data_bytes;
  uint8_t data[52];
};

/****************************************************************************
 * External Functions
 ****************************************************************************/

extern void esp_rom_delay_us(uint32_t us);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* LCD GC9503CV configuration parameters */

static const struct lcd_config_data g_lcd_config[] =
{
  {
    /* Enable manufacture command set in page 0 */

    0xf0, 5,
    {
      0x55, 0xaa, 0x52, 0x08, 0x00
    }
  },
  {
    /* Unknown */

    0xf6, 2,
    {
      0x5a, 0x87
    }
  },
  {
    /* Unknown */

    0xc1, 1,
    {
      0x3f
    }
  },
  {
    /* Unknown */

    0xc2, 1,
    {
      0x0e
    }
  },
  {
    /* Unknown */

    0xc6, 1,
    {
      0xf8
    }
  },
  {
    /* Unknown */

    0xc9, 1,
    {
      0x10
    }
  },
  {
    /* Unknown */

    0xcd, 1,
    {
      0x25
    }
  },
  {
    /* Unknown */

    0xf8, 1,
    {
      0x8a
    }
  },
  {
    /* Unknown */

    0xac, 1,
    {
      0x45
    }
  },
  {
    /* Set VGH to be 13(15.00) */

    0xa0, 1,
    {
      0xdd
    }
  },
  {
    /* Unknown */

    0xa7, 1,
    {
      0x47
    }
  },
  {
    /* Unknown */

    0xfa, 4,
    {
      0x00, 0x00, 0x00, 0x04
    }
  },
  {
    /* Set VGL to be 5(-10.5) */

    0x86, 4,
    {
      0x99, 0xa3, 0xa3, 0x51
    }
  },
  {
    /* Unknown */

    0xa3, 1,
    {
      0xee
    }
  },
  {
    /* Unknown */

    0xfd, 3,
    {
      0x3c, 0x3c, 0x00
    }
  },
  {
    /* Unknown */

    0x71, 1,
    {
      0x48
    }
  },
  {
    /* Unknown */

    0x72, 1,
    {
      0x48
    }
  },
  {
    /* Unknown */

    0x73, 2,
    {
      0x00, 0x44
    }
  },
  {
    /* Unknown */

    0x97, 1,
    {
      0xee
    }
  },
  {
    /* Unknown */

    0x83, 1,
    {
      0x93
    }
  },
  {
    /* Set adjustment of VGMP */

    0x9a, 1,
    {
      0x72
    }
  },
  {
    /* Set adjustment of VGMN */

    0x9b, 1,
    {
      0x5a
    }
  },
  {
    /* Set adjustment of VGSPN */

    0x82, 2,
    {
      0x2c, 0x2c
    }
  },
  {
    /* Unknown */

    0x6d, 32,
    {
      0x00, 0x1f, 0x19, 0x1a, 0x10, 0x0e, 0x0c, 0x0a,
      0x02, 0x07, 0x1e, 0x1e, 0x1e, 0x1e, 0x1e, 0x1e,
      0x1e, 0x1e, 0x1e, 0x1e, 0x1e, 0x1e, 0x08, 0x01,
      0x09, 0x0b, 0x0d, 0x0f, 0x1a, 0x19, 0x1f, 0x00
    }
  },
  {
    /* Unknown */

    0x64, 16,
    {
      0x38, 0x05, 0x01, 0xdb, 0x03, 0x03, 0x38, 0x04,
      0x01, 0xdc, 0x03, 0x03, 0x7a, 0x7a, 0x7a, 0x7a
    }
  },
  {
    /* Unknown */

    0x65, 16,
    {
      0x38, 0x03, 0x01, 0xdd, 0x03, 0x03, 0x38, 0x02,
      0x01, 0xde, 0x03, 0x03, 0x7a, 0x7a, 0x7a, 0x7a
    }
  },
  {
    /* Unknown */

    0x66, 16,
    {
      0x38, 0x01, 0x01, 0xdf, 0x03, 0x03, 0x38, 0x00,
      0x01, 0xe0, 0x03, 0x03, 0x7a, 0x7a, 0x7a, 0x7a
    }
  },
  {
    /* Unknown */

    0x67, 16,
    {
      0x30, 0x01, 0x01, 0xe1, 0x03, 0x03, 0x30, 0x02,
      0x01, 0xe2, 0x03, 0x03, 0x7a, 0x7a, 0x7a, 0x7a
    }
  },
  {
    /* Unknown */

    0x68, 13,
    {
      0x00, 0x08, 0x15, 0x08, 0x15, 0x7a, 0x7a, 0x08,
      0x15, 0x08, 0x15, 0x7a, 0x7a
    }
  },
  {
    /* Unknown */

    0x60, 8,
    {
      0x38, 0x08, 0x7a, 0x7a, 0x38, 0x09, 0x7a, 0x7a
    }
  },
  {
    /* Unknown */

    0x63, 8,
    {
      0x31, 0xe4, 0x7a, 0x7a, 0x31, 0xe5, 0x7a, 0x7a
    }
  },
  {
    /* Unknown */

    0x69, 7,
    {
      0x04, 0x22, 0x14, 0x22, 0x14, 0x22, 0x08
    }
  },
  {
    /* Unknown */

    0x6b, 1,
    {
      0x07
    }
  },
  {
    /* Unknown */

    0x7a, 2,
    {
      0x08, 0x13
    }
  },
  {
    /* Unknown */

    0x7b, 2,
    {
      0x08, 0x13
    }
  },
  {
    /* Unknown */

    0xd1, 52,
    {
      0x00, 0x00, 0x00, 0x04, 0x00, 0x12, 0x00, 0x18,
      0x00, 0x21, 0x00, 0x2a, 0x00, 0x35, 0x00, 0x47,
      0x00, 0x56, 0x00, 0x90, 0x00, 0xe5, 0x01, 0x68,
      0x01, 0xd5, 0x01, 0xd7, 0x02, 0x36, 0x02, 0xa6,
      0x02, 0xee, 0x03, 0x48, 0x03, 0xa0, 0x03, 0xba,
      0x03, 0xc5, 0x03, 0xd0, 0x03, 0xe0, 0x03, 0xea,
      0x03, 0xfa, 0x03, 0xff
    }
  },
  {
    /* Unknown */

    0xd2, 52,
    {
      0x00, 0x00, 0x00, 0x04, 0x00, 0x12, 0x00, 0x18,
      0x00, 0x21, 0x00, 0x2a, 0x00, 0x35, 0x00, 0x47,
      0x00, 0x56, 0x00, 0x90, 0x00, 0xe5, 0x01, 0x68,
      0x01, 0xd5, 0x01, 0xd7, 0x02, 0x36, 0x02, 0xa6,
      0x02, 0xee, 0x03, 0x48, 0x03, 0xa0, 0x03, 0xba,
      0x03, 0xc5, 0x03, 0xd0, 0x03, 0xe0, 0x03, 0xea,
      0x03, 0xfa, 0x03, 0xff
    }
  },
  {
    /* Unknown */

    0xd3, 52,
    {
      0x00, 0x00, 0x00, 0x04, 0x00, 0x12, 0x00, 0x18,
      0x00, 0x21, 0x00, 0x2a, 0x00, 0x35, 0x00, 0x47,
      0x00, 0x56, 0x00, 0x90, 0x00, 0xe5, 0x01, 0x68,
      0x01, 0xd5, 0x01, 0xd7, 0x02, 0x36, 0x02, 0xa6,
      0x02, 0xee, 0x03, 0x48, 0x03, 0xa0, 0x03, 0xba,
      0x03, 0xc5, 0x03, 0xd0, 0x03, 0xe0, 0x03, 0xea,
      0x03, 0xfa, 0x03, 0xff
    }
  },
  {
    /* Unknown */

    0xd4, 52,
    {
      0x00, 0x00, 0x00, 0x04, 0x00, 0x12, 0x00, 0x18,
      0x00, 0x21, 0x00, 0x2a, 0x00, 0x35, 0x00, 0x47,
      0x00, 0x56, 0x00, 0x90, 0x00, 0xe5, 0x01, 0x68,
      0x01, 0xd5, 0x01, 0xd7, 0x02, 0x36, 0x02, 0xa6,
      0x02, 0xee, 0x03, 0x48, 0x03, 0xa0, 0x03, 0xba,
      0x03, 0xc5, 0x03, 0xd0, 0x03, 0xe0, 0x03, 0xea,
      0x03, 0xfa, 0x03, 0xff
    }
  },
  {
    /* Unknown */

    0xd5, 52,
    {
      0x00, 0x00, 0x00, 0x04, 0x00, 0x12, 0x00, 0x18,
      0x00, 0x21, 0x00, 0x2a, 0x00, 0x35, 0x00, 0x47,
      0x00, 0x56, 0x00, 0x90, 0x00, 0xe5, 0x01, 0x68,
      0x01, 0xd5, 0x01, 0xd7, 0x02, 0x36, 0x02, 0xa6,
      0x02, 0xee, 0x03, 0x48, 0x03, 0xa0, 0x03, 0xba,
      0x03, 0xc5, 0x03, 0xd0, 0x03, 0xe0, 0x03, 0xea,
      0x03, 0xfa, 0x03, 0xff
    },
  },
  {
    /* Unknown */

    0xd6, 52,
    {
      0x00, 0x00, 0x00, 0x04, 0x00, 0x12, 0x00, 0x18,
      0x00, 0x21, 0x00, 0x2a, 0x00, 0x35, 0x00, 0x47,
      0x00, 0x56, 0x00, 0x90, 0x00, 0xe5, 0x01, 0x68,
      0x01, 0xd5, 0x01, 0xd7, 0x02, 0x36, 0x02, 0xa6,
      0x02, 0xee, 0x03, 0x48, 0x03, 0xa0, 0x03, 0xba,
      0x03, 0xc5, 0x03, 0xd0, 0x03, 0xe0, 0x03, 0xea,
      0x03, 0xfa, 0x03, 0xff
    },
  },
  {
    /* 18-bit Pixel */

    0x3a, 1,
    {
      0x60
    }
  },
  {
    /* Enter sleep out mode, no parameters */

    0x11, 0
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * 9-bit SPI Rising Mode
 ****************************************************************************/

/****************************************************************************
 * Name: send_byte
 *
 * Description:
 *   Send 9-bit data to LCD.
 *
 * Input Parameters:
 *   data - TX data
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void send_byte(uint16_t data)
{
  for (int n = 0; n < 9; n++)
    {
      if (data & 0x0100)
        {
          SDO(1);
        }
      else
        {
          SDO(0);
        }

      data = data << 1;

      SCK(0);
      DELAY(10);
      SCK(1);
      DELAY(10);
    }
}

/****************************************************************************
 * Name: send_cmd
 *
 * Description:
 *   Send command to LCD.
 *
 * Input Parameters:
 *   cmd - TX command
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void send_cmd(uint8_t cmd)
{
  uint16_t spi_data = cmd;

  CS(0);
  DELAY(10);

  send_byte(spi_data);

  DELAY(10);
  CS(1);
  SCK(0);
  SDO(0);
  DELAY(10);
}

/****************************************************************************
 * Name: send_data
 *
 * Description:
 *   Send data to LCD.
 *
 * Input Parameters:
 *   cmd - TX command
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void send_data(uint8_t data)
{
  uint16_t spi_data = data;

  CS(0);
  DELAY(10);

  spi_data &= 0x00ff;
  spi_data |= 0x0100;
  send_byte(spi_data);

  DELAY(10);
  CS(1);
  SCK(0);
  SDO(0);
  DELAY(10);
}

/****************************************************************************
 * Name: lcd_initialize_spi
 *
 * Description:
 *   Initialize write only SPI interface by IO expander.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void lcd_initialize_spi(void)
{
  uint8_t pin_mask = (1 << CS_PIN) |
                     (1 << SCK_PIN) |
                     (1 << SDO_PIN);

  DEBUGASSERT(board_ioexpander_initialize() == 0);
  DEBUGASSERT(board_ioexpander_set_pin(0, pin_mask) == 0);

  CS(1);
  SCK(1);
  SDO(1);
}

/****************************************************************************
 * Name: lcd_configure_display
 *
 * Description:
 *   Configure LCD with global configuration parameters.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void lcd_configure_display(void)
{
  /* Pull-up V-SYNC pin to start configurating LCD */

  esp32s3_configgpio(CONFIG_ESP32S3_LCD_VSYNC_PIN, OUTPUT | PULLUP);
  esp32s3_gpiowrite(CONFIG_ESP32S3_LCD_VSYNC_PIN, 1);

  for (int i = 0; i < nitems(g_lcd_config); i++)
    {
      send_cmd(g_lcd_config[i].cmd);

      for (int j = 0; j < g_lcd_config[i].data_bytes; j++)
        {
          send_data(g_lcd_config[i].data[j]);
        }
    }

  /* Wait until LCD is ready */

  nxsig_usleep(120 * 1000);

  /* Display on */

  send_cmd(0x29);

  /* Wait until LCD is on */

  nxsig_usleep(20 * 1000);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_lcd_initialize
 *
 * Description:
 *   Initialize LCD.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_lcd_initialize(void)
{
  int ret;

  lcd_initialize_spi();
  lcd_configure_display();

#ifdef CONFIG_VIDEO_FB
  /* Initialize and register the framebuffer driver */

  ret = fb_register(0, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: fb_register() failed: %d\n", ret);
      return ret;
    }
#else
  UNUSED(ret);
#endif

  return 0;
}
