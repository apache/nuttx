/****************************************************************************
 * boards/arm/stm32/nucleo-l152re/src/stm32_ili93418b.c
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
#include <inttypes.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "nucleo-l152re.h"
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/ili9341.h>
#include <nuttx/video/rgbcolors.h>

#include "stm32_gpio.h"

#include <arch/board/board.h>

#ifdef CONFIG_LCD_ILI9341

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * References: ILI9341_DS_V1.10.pdf (Rev: 1.10), "a-Si TFT LCD Single Chip
 *             Driver 240RGBx320 Resolution and 262K color", ILI TECHNOLOGY
 *             CORP., http://www.ilitek.com.
 *             ILI TECHNOLOGY CORP., http://www.ilitek.com.
 ****************************************************************************/

/****************************************************************************
 * Private Function Protototypes
 ****************************************************************************/

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

#define STM32_ILI9341_IFMODE_PARAM ((!ILI9341_INTERFACE_CONTROL_EPL) |  \
                                    ILI9341_INTERFACE_CONTROL_DPL |     \
                                    (!ILI9341_INTERFACE_CONTROL_HSPL) | \
                                    (!ILI9341_INTERFACE_CONTROL_VSPL) | \
                                    ILI9341_INTERFACE_CONTROL_RCM(2) |  \
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

#define STM32_ILI9341_IFCTL_PARAM1 (ILI9341_INTERFACE_CONTROL_WEMODE |  \
                                    !ILI9341_INTERFACE_CONTROL_BGREOR | \
                                    !ILI9341_INTERFACE_CONTROL_MVEOR |  \
                                    !ILI9341_INTERFACE_CONTROL_MXEOR |  \
                                    !ILI9341_INTERFACE_CONTROL_MYEOR)

/* Parameter 2: 0x0000
 *
 * EPF:     0   65k color format for RGB interface
 * MDT:     0   Display data transfer mode
 *
 */
#define STM32_ILI9341_IFCTL_PARAM2 (ILI9341_INTERFACE_CONTROL_MDT(0) | \
                                    ILI9341_INTERFACE_CONTROL_EPF(0))

/* Parameter 3: 0x0000/0x0020
 *
 * ENDIAN:  0   Big endian
 * DM:      1   RGB Interface Mode
 * RM:      1   RGB interface
 * RIM:     0   18-bit 1 transfer/pixel RGB interface mode
 *
 */
#define STM32_ILI9341_IFCTL_PARAM3 ((!ILI9341_INTERFACE_CONTROL_RIM) | \
                                    ILI9341_INTERFACE_CONTROL_RM |     \
                                    ILI9341_INTERFACE_CONTROL_DM(1) |  \
                                    (!ILI9341_INTERFACE_CONTROL_ENDIAN))

/* LCD CONTROL */

#define LCD_CS_CLR (stm32_gpiowrite(GPIO_LCD_CS, 0))
#define LCD_CS_SET (stm32_gpiowrite(GPIO_LCD_CS, 1))

#define LCD_WR_CLR (stm32_gpiowrite(GPIO_LCD_WR, 0))
#define LCD_WR_SET (stm32_gpiowrite(GPIO_LCD_WR, 1))

#define LCD_RD_CLR (stm32_gpiowrite(GPIO_LCD_RD, 0))
#define LCD_RD_SET (stm32_gpiowrite(GPIO_LCD_RD, 1))

#define LCD_RS_DATA (stm32_gpiowrite(GPIO_LCD_RS, 1)) /* write data */
#define LCD_RS_CMD (stm32_gpiowrite(GPIO_LCD_RS, 0))  /* write cmd */

#define BIT_SHIFT (0)
#define BIT0 (0 << BIT_SHIFT)
#define BIT1 (1 << BIT_SHIFT)
#define BIT2 (2 << BIT_SHIFT)
#define BIT3 (3 << BIT_SHIFT)
#define BIT4 (4 << BIT_SHIFT)
#define BIT5 (5 << BIT_SHIFT)
#define BIT6 (6 << BIT_SHIFT)
#define BIT7 (7 << BIT_SHIFT)

#define GET_BIT(data, bit) (((data) & (1 << (bit))) >> (bit))
#define WRITE_BIT(b, pin) ((b) == 1 ? GPIO_BSRR_SET(pin) : GPIO_BSRR_RESET(pin))

static const uint32_t g_lcdpin[] =
    {
      GPIO_LCD_D0, GPIO_LCD_D1, GPIO_LCD_D2, GPIO_LCD_D3, /* D0-D3 */
      GPIO_LCD_D4, GPIO_LCD_D5, GPIO_LCD_D6, GPIO_LCD_D7, /* D4-D7 */
      GPIO_LCD_RD, GPIO_LCD_WR, GPIO_LCD_RS, GPIO_LCD_RST,
      GPIO_LCD_CS
    };

#define LCD_NPINS (sizeof(g_lcdpin) / sizeof(uint32_t))

/* Command and data transmission control */

static int stm32_ili93418b_recvblock(struct ili9341_lcd_s *lcd,
                                     uint16_t *wd, uint16_t nwords);
static void stm32_ili93418b_deselect(struct ili9341_lcd_s *lcd);
static void stm32_ili93418b_select(struct ili9341_lcd_s *lcd);
static int stm32_ili93418b_sendcmd(struct ili9341_lcd_s *lcd,
                                   const uint8_t cmd);
static int stm32_ili93418b_sendparam(struct ili9341_lcd_s *lcd,
                                     const uint8_t param);
static int stm32_ili93418b_recvparam(struct ili9341_lcd_s *lcd,
                                     uint8_t *param);
static int stm32_ili93418b_backlight(struct ili9341_lcd_s *lcd,
                                     int level);
static int stm32_ili93418b_sendgram(struct ili9341_lcd_s *lcd,
                                    const uint16_t *wd, uint32_t nwords);
static int stm32_ili93418b_recvgram(struct ili9341_lcd_s *lcd,
                                    uint16_t *wd, uint32_t nwords);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* This is the standard, NuttX LCD driver object */

static struct ili9341_lcd_s g_ili9341_lcddev =
{
  /* Initialize structure */

  .select    = stm32_ili93418b_select,
  .deselect  = stm32_ili93418b_deselect,
  .sendcmd   = stm32_ili93418b_sendcmd,
  .sendparam = stm32_ili93418b_sendparam,
  .recvparam = stm32_ili93418b_recvparam,
  .sendgram  = stm32_ili93418b_sendgram,
  .recvgram  = stm32_ili93418b_recvgram,
  .backlight = stm32_ili93418b_backlight,
};

/****************************************************************************
 * Name: write_byte
 *
 * Description:
 *   Send a byte to the lcd driver.
 *
 * Input Parameters:
 *   data - a byte
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void write_byte(uint8_t data)
{
  LCD_RD_SET;

  /**
   * This is simple to understand
   *
   * stm32_gpiowrite(GPIO_LCD_D0, GET_BIT(data, BIT0));
   * stm32_gpiowrite(GPIO_LCD_D1, GET_BIT(data, BIT1));
   * stm32_gpiowrite(GPIO_LCD_D2, GET_BIT(data, BIT2));
   * stm32_gpiowrite(GPIO_LCD_D3, GET_BIT(data, BIT3));
   * stm32_gpiowrite(GPIO_LCD_D4, GET_BIT(data, BIT4));
   * stm32_gpiowrite(GPIO_LCD_D5, GET_BIT(data, BIT5));
   * stm32_gpiowrite(GPIO_LCD_D6, GET_BIT(data, BIT6));
   * stm32_gpiowrite(GPIO_LCD_D7, GET_BIT(data, BIT7));
   *
   **/

  putreg32(WRITE_BIT(GET_BIT(data, BIT0), GPIO_PIN9) |
               WRITE_BIT(GET_BIT(data, BIT2), GPIO_PIN10) |
               WRITE_BIT(GET_BIT(data, BIT7), GPIO_PIN8),
               STM32_GPIOA_BSRR);
  putreg32(WRITE_BIT(GET_BIT(data, BIT3), GPIO_PIN3) |
               WRITE_BIT(GET_BIT(data, BIT4), GPIO_PIN5) |
               WRITE_BIT(GET_BIT(data, BIT5), GPIO_PIN4) |
               WRITE_BIT(GET_BIT(data, BIT6), GPIO_PIN10),
               STM32_GPIOB_BSRR);
  putreg32(WRITE_BIT(GET_BIT(data, BIT1), GPIO_PIN7), STM32_GPIOC_BSRR);

  LCD_WR_CLR;
  LCD_WR_SET;
}

/** references https://controllerstech.com/interface-tft-display-with-stm32/
 * #define READ() (((getreg32(STM32_GPIOA_IDR) & (1 << GPIO_PIN9)) >> 9) |  \
 *                 ((getreg32(STM32_GPIOC_IDR) & (1 << GPIO_PIN7)) >> 6) |  \
 *                 ((getreg32(STM32_GPIOA_IDR) & (1 << GPIO_PIN10)) >> 8) | \
 *                 ((getreg32(STM32_GPIOB_IDR) & (1 << GPIO_PIN3)) >> 0) |  \
 *                 ((getreg32(STM32_GPIOB_IDR) & (1 << GPIO_PIN5)) >> 1) |  \
 *                 ((getreg32(STM32_GPIOB_IDR) & (1 << GPIO_PIN4)) << 1) |  \
 *                 ((getreg32(STM32_GPIOB_IDR) & (1 << GPIO_PIN10)) >> 4) | \
 *                 ((getreg32(STM32_GPIOA_IDR) & (1 << GPIO_PIN8)) >> 1))
 **/

/****************************************************************************
 * Name: read_byte
 *
 * Description:
 *   Read a byte from IOs
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success - The received byte from the LCD Single Chip Driver.
 *   On error   - 0 (If timeout during receiving)
 *
 ****************************************************************************/

static inline uint8_t read_byte(void)
{
  uint32_t a_idr;
  uint32_t b_idr;
  uint8_t data = 0;
  LCD_RD_CLR;
  up_udelay(1);
  a_idr = getreg32(STM32_GPIOA_IDR);
  b_idr = getreg32(STM32_GPIOB_IDR);

  data |= (stm32_gpioread(GPIO_LCD_D1) << 1);
  data |= (GET_BIT(a_idr, GPIO_PIN9) << BIT0 |
           GET_BIT(a_idr, GPIO_PIN10) << BIT2 |
           GET_BIT(a_idr, GPIO_PIN8) << BIT7);
  data |= (GET_BIT(b_idr, BIT3) << BIT3 |
           GET_BIT(b_idr, GPIO_PIN5) << BIT4 |
           GET_BIT(b_idr, GPIO_PIN4) << BIT5 |
           GET_BIT(b_idr, GPIO_PIN10) << BIT6);

  /* data = READ(); */

  LCD_RD_SET;
  return data;
}

/****************************************************************************
 * Name: stm32_ili93414ws_recvblock
 *
 * Description:
 *   Receive a number of words from to the lcd driver.
 *   Note: The first received word is the dummy word and discarded!
 *
 * Input Parameters:
 *   lcd    - Reference to the private device structure
 *   wd    -  Reference to where the words receive
 *   nwords - number of words to receive
 *
 * Returned Value:
 *  OK - On Success
 *
 ****************************************************************************/

static int stm32_ili93418b_recvblock(struct ili9341_lcd_s *lcd,
                                     uint16_t *wd, uint16_t nwords)
{
  /** ili9341 uses a 18-bit pixel format packed in a 24-bit stream per pixel.
   *  The following format is transmitted: RRRRRR00 GGGGGG00 BBBBBB00
   *  Convert it to:                       RRRRRGGG GGGBBBBB
   */

  /**  8-bit parallel mode is enabled for pixel data operations.
   *  Each pixel must be received by three read operations.
   */

  uint16_t *dest = (uint16_t *)wd;
  LCD_RS_DATA;
  while (nwords--)
    {
      uint8_t r;
      uint8_t g;
      uint8_t b;

      /* read dummy */

      r = read_byte();

      /* read red */

      r = read_byte();
      r = r >> 3;

      /* read green  */

      g = read_byte();
      g = g >> 2;

      /* read blue */

      b = read_byte();
      b = b >> 3;
      *dest++ = ((r << 11) | (g << 5) | b);
    }

  return OK;
}

/****************************************************************************
 * Name: stm32_ili93418b_select
 *
 * Description:
 *   Select the LCD
 *
 * Input Parameters:
 *   lcd  - Reference to the public driver structure
 *
 * Returned Value:
 *
 ****************************************************************************/

static void stm32_ili93418b_select(struct ili9341_lcd_s *lcd)
{
  LCD_CS_CLR;
}

/****************************************************************************
 * Name: stm32_ili93418b_deselect
 *
 * Description:
 *   De-select the LCD
 *
 * Input Parameters:
 *   lcd  - Reference to the public driver structure
 *
 * Returned Value:
 *
 ****************************************************************************/

static void stm32_ili93418b_deselect(struct ili9341_lcd_s *lcd)
{
  LCD_CS_SET;
}

/****************************************************************************
 * Name: stm32_ili93418b_sendparam
 *
 * Description:
 *   Send a parameter to the lcd driver.
 *
 * Input Parameters:
 *   lcd    - Reference to the ili9341_lcd_s driver structure
 *   param  - parameter to send
 *
 * Returned Value:
 *   OK - On Success
 *
 ****************************************************************************/

static int stm32_ili93418b_sendparam(struct ili9341_lcd_s *lcd,
                                     const uint8_t param)
{
  lcdinfo("param=%04x\n", param);
  LCD_RS_DATA;
  write_byte(param);
  return OK;
}

/****************************************************************************
 * Name: stm32_ili93418b_sendgram
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
 *   OK - On Success
 *
 ****************************************************************************/

static int stm32_ili93418b_sendgram(struct ili9341_lcd_s *lcd,
                                    const uint16_t *wd, uint32_t nwords)
{
  lcdinfo("wd=%p , wd=0x%x, nwords=%" PRId32 "\n", wd, *wd, nwords);

  /* 8-bit parallel mode is enabled for pixel data operations.
   * Each pixel must be transmitted by two write operations.
   */

  const uint16_t *src = wd;
  uint16_t word;
  LCD_RS_DATA;
  while (nwords-- > 0)
    {
      word = *src++;
      write_byte((word & 0xff));
      write_byte((word >> 8));
    }

  return OK;
};

/****************************************************************************
 * Name: stm32_ili93418b_recvparam
 *
 * Description:
 *   Receive a parameter from the lcd driver.
 *
 * Input Parameters:
 *   lcd    - Reference to the ili9341_lcd_s driver structure
 *   param  - Reference to where parameter receive
 *
 * Returned Value:
 *   OK - On Success
 *
 ****************************************************************************/

static int stm32_ili93418b_recvparam(struct ili9341_lcd_s *lcd,
                                     uint8_t *param)
{
  LCD_RS_DATA;
  *param = read_byte();
  lcdinfo("param=%p\n", param);
  return OK;
}

/****************************************************************************
 * Name: stm32_ili93418b_recvgram
 *
 * Description:
 *   Receive pixel words from the lcd driver gram.
 *
 * Input Parameters:
 *   lcd    - Reference to the public driver structure
 *   wd     - Reference to where the pixel words receive
 *   nwords - number of pixel words to receive
 *
 * Returned Value:
 *   OK - On Success
 *
 ****************************************************************************/

static int stm32_ili93418b_recvgram(struct ili9341_lcd_s *lcd,
                                    uint16_t *wd, uint32_t nwords)
{
  lcdinfo("wd=%p, nwords=%" PRId32 "\n", wd, nwords);
  return stm32_ili93418b_recvblock(lcd, wd, nwords);
}

/****************************************************************************
 * Name: stm32_ili93418b_sndcmd
 *
 * Description:
 *   Send a command to the lcd driver.
 *
 * Input Parameters:
 *   lcd  - Reference to the ili9341_lcd_s driver structure
 *   cmd  - command to send
 *
 * Returned Value:
 *   On success - OK
 *
 ****************************************************************************/

static int stm32_ili93418b_sendcmd(
    struct ili9341_lcd_s *lcd, const uint8_t cmd)
{
  lcdinfo("cmd=%04x\n", cmd);
  LCD_RS_CMD;
  write_byte(cmd);
  return OK;
}

/****************************************************************************
 * Name: stm32_ili93418b_backlight
 *
 * Description:
 *   Set the backlight level of the connected display.
 *
 * Input Parameters:
 *   lcd   - Reference to the public driver structure
 *   level - backligth level
 *
 * Returned Value:
 *   OK - On Success
 *
 ****************************************************************************/

static int stm32_ili93418b_backlight(struct ili9341_lcd_s *lcd,
                                     int level)
{
  return OK;
}

/****************************************************************************
 * Name:  sam_gpio_initialize
 *
 * Description:
 *   Configure LCD GPIO pins
 *
 ****************************************************************************/

static inline void stm32_gpio_initialize(void)
{
  int i;

  /* Configure all LCD pins pins (backlight is initially off) */

  for (i = 0; i < LCD_NPINS; i++)
    {
      stm32_configgpio(g_lcdpin[i]);
    }
}

/****************************************************************************
 * Name:  stm32_ili93414ws_initialize
 *
 * Description:
 *   Initialize the device structure to control the LCD Single chip driver.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   On success, this function returns a reference to the LCD control object
 *   for the specified ILI9341 LCD Single chip driver connected as 8-bit
 *   series parallel. NULL is returned on any failure.
 *
 ****************************************************************************/

struct lcd_dev_s *g_lcddev;
struct lcd_dev_s *stm32_ili93418b_initialize(void)
{
  struct ili9341_lcd_s *lcd = &g_ili9341_lcddev;

  lcdinfo("initialize ili9341 8bit parallel subdriver\n");

  /* Configure gpios */

  stm32_gpio_initialize();
  up_mdelay(50);

  /* reset LCD   */

  stm32_gpiowrite(GPIO_LCD_RST, 0);
  up_mdelay(100);
  stm32_gpiowrite(GPIO_LCD_RST, 1);
  up_mdelay(50);

  /* initialize LCD */

  g_lcddev = ili9341_initialize(lcd, 0);

  /* Select LCD device */

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

  lcdinfo("ili9341 set Frame control\n");
  lcd->sendcmd(lcd, ILI9341_FRAME_RATE_CONTROL_NORMAL);
  lcd->sendparam(lcd, 0x00);
  lcd->sendparam(lcd, 0x13); /* 0x18 79Hz, 0x1B default 70Hz, 0x13 100Hz */

  /* Sleep out set to the end */

  lcdinfo("ili9341 LCD driver: Sleep Out\n");
  lcd->sendcmd(lcd, ILI9341_SLEEP_OUT);
  up_mdelay(5);

  /* Display on */

  lcdinfo("ili9341 LCD driver: Display On\n");
  lcd->sendcmd(lcd, ILI9341_DISPLAY_ON);

  /* Deselect LCD device */

  lcd->deselect(lcd);
  return g_lcddev;
}

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
  g_lcddev = stm32_ili93418b_initialize();
  if (g_lcddev == NULL)
    {
      lcdinfo("Initialize ili9341 lcd driver NULL\n");
      return ENODEV;
    }

  ili9341_clear(g_lcddev, 0);
  return OK;
}

/****************************************************************************
 * Name:  board_lcd_getdev
 *
 * Description:
 *   Return a a reference to the LCD object for the specified LCD.
 *   This allows support
 *   for multiple LCD devices.
 *
 ****************************************************************************/

struct lcd_dev_s *board_lcd_getdev(int lcddev)
{
  if (lcddev == 0)
    {
      return g_lcddev;
    }

  return NULL;
}

#endif /* CONFIG_LCD_ILI9341 */
