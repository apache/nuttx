/****************************************************************************
 * boards/arm/stm32f1/nvmpg-stm32f1/src/stm32_ili9225.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

/* ILI9225 176x220 TFT on a GPIO bit-banged 8080 16-bit bus.
 *
 * The upstream drivers/lcd/ili9225.c only supports SPI, so this board
 * provides its own lcd_dev_s.  The panel is used in landscape (220x176):
 *
 *   GRAM vertical address (0..219)   = screen column (x)
 *   GRAM horizontal address (0..175) = 175 - screen row (y)
 *
 * which is the same orientation as the original NVMPG firmware.  Entry mode
 * AM=1 makes the address counter move along the vertical (x) direction
 * first, so a screen raster line is one contiguous GRAM write.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/irq.h>
#include <sys/param.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/video/fb.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "stm32.h"
#include "nvmpg-stm32f1.h"

#ifdef CONFIG_LCD

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LCD_XRES        NVMPG_LCD_XRES
#define LCD_YRES        NVMPG_LCD_YRES
#define LCD_BPP         16

/* Control pin masks (GPIOA/GPIOC BSRR/BRR) */

#define LCD_WR_BIT      (1 << 10)  /* PC10 */
#define LCD_RS_BIT      (1 << 11)  /* PC11 */
#define LCD_CS_BIT      (1 << 12)  /* PC12 */
#define LCD_RD_BIT      (1 << 15)  /* PA15 */

/* ILI9225 registers used here */

#define ILI9225_DRIVER_OUTPUT   0x01
#define ILI9225_LCD_AC_DRIVING  0x02
#define ILI9225_ENTRY_MODE      0x03
#define ILI9225_DISP_CTRL1      0x07
#define ILI9225_BLANK_PERIOD    0x08
#define ILI9225_FRAME_CYCLE     0x0b
#define ILI9225_INTERFACE_CTRL  0x0c
#define ILI9225_OSC_CTRL        0x0f
#define ILI9225_POWER_CTRL1     0x10
#define ILI9225_POWER_CTRL2     0x11
#define ILI9225_POWER_CTRL3     0x12
#define ILI9225_POWER_CTRL4     0x13
#define ILI9225_POWER_CTRL5     0x14
#define ILI9225_VCI_RECYCLING   0x15
#define ILI9225_RAM_ADDR_H      0x20
#define ILI9225_RAM_ADDR_V      0x21
#define ILI9225_GRAM_DATA       0x22
#define ILI9225_HWIN_END        0x36
#define ILI9225_HWIN_START      0x37
#define ILI9225_VWIN_END        0x38
#define ILI9225_VWIN_START      0x39

/* Entry mode: BGR=1, I/D=10 (V increment, H decrement), AM=1 */

#define ILI9225_ENTRY_LANDSCAPE 0x1028

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct nvmpg_lcd_s
{
  struct lcd_dev_s dev;   /* Publicly visible device structure */
  uint8_t power;          /* Current power setting */
  bool initialized;
  uint16_t runbuffer[LCD_XRES];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int lcd_putrun(FAR struct lcd_dev_s *dev, fb_coord_t row,
                      fb_coord_t col, FAR const uint8_t *buffer,
                      size_t npixels);
static int lcd_putarea(FAR struct lcd_dev_s *dev, fb_coord_t row_start,
                       fb_coord_t row_end, fb_coord_t col_start,
                       fb_coord_t col_end, FAR const uint8_t *buffer,
                       fb_coord_t stride);
static int lcd_getvideoinfo(FAR struct lcd_dev_s *dev,
                            FAR struct fb_videoinfo_s *vinfo);
static int lcd_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int planeno,
                            FAR struct lcd_planeinfo_s *pinfo);
static int lcd_getpower(FAR struct lcd_dev_s *dev);
static int lcd_setpower(FAR struct lcd_dev_s *dev, int power);
static int lcd_getcontrast(FAR struct lcd_dev_s *dev);
static int lcd_setcontrast(FAR struct lcd_dev_s *dev,
                           unsigned int contrast);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct nvmpg_lcd_s g_lcd =
{
  .dev =
  {
    .getvideoinfo = lcd_getvideoinfo,
    .getplaneinfo = lcd_getplaneinfo,
    .getpower     = lcd_getpower,
    .setpower     = lcd_setpower,
    .getcontrast  = lcd_getcontrast,
    .setcontrast  = lcd_setcontrast,
  },
};

/* Power-on sequence taken from the original NVMPG firmware */

static const uint16_t g_init_power[][2] =
{
  { ILI9225_POWER_CTRL2, 0x001a },
  { ILI9225_POWER_CTRL3, 0x3121 },
  { ILI9225_POWER_CTRL4, 0x006c },
  { ILI9225_POWER_CTRL5, 0x4249 },
  { ILI9225_POWER_CTRL1, 0x0800 },
};

static const uint16_t g_init_ramp[] =
{
  0x011a, 0x031a, 0x071a, 0x0f1a, 0x0f3a
};

static const uint16_t g_init_regs[][2] =
{
  { ILI9225_DRIVER_OUTPUT,  0x011c },
  { ILI9225_LCD_AC_DRIVING, 0x0100 },
  { ILI9225_ENTRY_MODE,     ILI9225_ENTRY_LANDSCAPE },
  { ILI9225_DISP_CTRL1,     0x0000 },
  { ILI9225_BLANK_PERIOD,   0x0808 },
  { ILI9225_FRAME_CYCLE,    0x1100 },
  { ILI9225_INTERFACE_CTRL, 0x0000 },
  { ILI9225_OSC_CTRL,       0x1401 },
  { ILI9225_VCI_RECYCLING,  0x0000 },
  { ILI9225_RAM_ADDR_H,     0x0000 },
  { ILI9225_RAM_ADDR_V,     0x0000 },
  { 0x50, 0x0001 },                 /* Gamma 0x50..0x59 */
  { 0x51, 0x020b },
  { 0x52, 0x0805 },
  { 0x53, 0x0404 },
  { 0x54, 0x0c0c },
  { 0x55, 0x000c },
  { 0x56, 0x0101 },
  { 0x57, 0x0400 },
  { 0x58, 0x1108 },
  { 0x59, 0x050c },
  { ILI9225_HWIN_END,       LCD_YRES - 1 },  /* Full window */
  { ILI9225_HWIN_START,     0x0000 },
  { ILI9225_VWIN_END,       LCD_XRES - 1 },
  { ILI9225_VWIN_START,     0x0000 },
  { ILI9225_OSC_CTRL,       0x0b01 },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Bus primitives.  These are written against the registers directly: going
 * through stm32_gpiowrite() for every edge would make a full screen redraw
 * several times slower.
 */

static inline void lcd_wrbus(uint16_t value)
{
  putreg32(value, STM32_GPIOB_ODR);
  putreg32(LCD_WR_BIT, STM32_GPIOC_BRR);
  putreg32(LCD_WR_BIT, STM32_GPIOC_BSRR);
}

static inline void lcd_select(void)
{
  putreg32(LCD_CS_BIT, STM32_GPIOC_BRR);
}

static inline void lcd_deselect(void)
{
  putreg32(LCD_CS_BIT, STM32_GPIOC_BSRR);
}

static inline void lcd_index(uint16_t index)
{
  putreg32(LCD_RS_BIT, STM32_GPIOC_BRR);
  lcd_wrbus(index);
  putreg32(LCD_RS_BIT, STM32_GPIOC_BSRR);
}

static void lcd_writereg(uint16_t reg, uint16_t value)
{
  lcd_select();
  lcd_index(reg);
  lcd_wrbus(value);
  lcd_deselect();
}

/* Reads only the ID register, so the slow path through stm32_configgpio()
 * to turn PB0..15 around is fine here.
 */

static uint16_t lcd_readreg(uint16_t reg)
{
  uint16_t value;
  int i;

  lcd_select();
  lcd_index(reg);

  for (i = 0; i < 16; i++)
    {
      stm32_configgpio(GPIO_INPUT | GPIO_CNF_INFLOAT | GPIO_MODE_INPUT |
                       GPIO_PORTB | (i << GPIO_PIN_SHIFT));
    }

  putreg32(LCD_RD_BIT, STM32_GPIOA_BRR);
  up_udelay(5);
  value = (uint16_t)getreg32(STM32_GPIOB_IDR);
  putreg32(LCD_RD_BIT, STM32_GPIOA_BSRR);

  for (i = 0; i < 16; i++)
    {
      stm32_configgpio(GPIO_LCD_DATA | (i << GPIO_PIN_SHIFT));
    }

  lcd_deselect();
  return value;
}

/* Select the screen rectangle [x0..x1] x [y0..y1] and leave the controller
 * ready to receive pixels in raster order (left to right, top to bottom).
 */

static void lcd_setarea(fb_coord_t x0, fb_coord_t y0,
                        fb_coord_t x1, fb_coord_t y1)
{
  lcd_writereg(ILI9225_HWIN_END,   LCD_YRES - 1 - y0);
  lcd_writereg(ILI9225_HWIN_START, LCD_YRES - 1 - y1);
  lcd_writereg(ILI9225_VWIN_END,   x1);
  lcd_writereg(ILI9225_VWIN_START, x0);
  lcd_writereg(ILI9225_RAM_ADDR_H, LCD_YRES - 1 - y0);
  lcd_writereg(ILI9225_RAM_ADDR_V, x0);
}

static void lcd_fill(uint16_t color)
{
  int i;

  lcd_setarea(0, 0, LCD_XRES - 1, LCD_YRES - 1);
  lcd_select();
  lcd_index(ILI9225_GRAM_DATA);
  putreg32(color, STM32_GPIOB_ODR);

  for (i = 0; i < LCD_XRES * LCD_YRES; i++)
    {
      putreg32(LCD_WR_BIT, STM32_GPIOC_BRR);
      putreg32(LCD_WR_BIT, STM32_GPIOC_BSRR);
    }

  lcd_deselect();
}

/****************************************************************************
 * Name: lcd_putrun
 ****************************************************************************/

static int lcd_putrun(FAR struct lcd_dev_s *dev, fb_coord_t row,
                      fb_coord_t col, FAR const uint8_t *buffer,
                      size_t npixels)
{
  FAR const uint16_t *src = (FAR const uint16_t *)buffer;

  DEBUGASSERT(buffer != NULL && ((uintptr_t)buffer & 1) == 0);
  DEBUGASSERT(row < LCD_YRES && col + npixels <= LCD_XRES);

  lcd_setarea(col, row, col + npixels - 1, row);
  lcd_select();
  lcd_index(ILI9225_GRAM_DATA);

  while (npixels-- > 0)
    {
      lcd_wrbus(*src++);
    }

  lcd_deselect();
  return OK;
}

/****************************************************************************
 * Name: lcd_putarea
 ****************************************************************************/

static int lcd_putarea(FAR struct lcd_dev_s *dev, fb_coord_t row_start,
                       fb_coord_t row_end, fb_coord_t col_start,
                       fb_coord_t col_end, FAR const uint8_t *buffer,
                       fb_coord_t stride)
{
  size_t width = col_end - col_start + 1;
  fb_coord_t row;
  size_t i;

  DEBUGASSERT(buffer != NULL && ((uintptr_t)buffer & 1) == 0);
  DEBUGASSERT(row_end < LCD_YRES && col_end < LCD_XRES);

  if (stride == 0)
    {
      stride = width * sizeof(uint16_t);
    }

  lcd_setarea(col_start, row_start, col_end, row_end);
  lcd_select();
  lcd_index(ILI9225_GRAM_DATA);

  for (row = row_start; row <= row_end; row++)
    {
      FAR const uint16_t *src = (FAR const uint16_t *)buffer;

      for (i = 0; i < width; i++)
        {
          lcd_wrbus(src[i]);
        }

      buffer += stride;
    }

  lcd_deselect();
  return OK;
}

/****************************************************************************
 * Name: lcd_getvideoinfo
 ****************************************************************************/

static int lcd_getvideoinfo(FAR struct lcd_dev_s *dev,
                            FAR struct fb_videoinfo_s *vinfo)
{
  DEBUGASSERT(vinfo != NULL);

  vinfo->fmt     = FB_FMT_RGB16_565;
  vinfo->xres    = LCD_XRES;
  vinfo->yres    = LCD_YRES;
  vinfo->nplanes = 1;
  return OK;
}

/****************************************************************************
 * Name: lcd_getplaneinfo
 ****************************************************************************/

static int lcd_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int planeno,
                            FAR struct lcd_planeinfo_s *pinfo)
{
  FAR struct nvmpg_lcd_s *priv = (FAR struct nvmpg_lcd_s *)dev;

  DEBUGASSERT(pinfo != NULL && planeno == 0);

  pinfo->putrun  = lcd_putrun;
  pinfo->putarea = lcd_putarea;
  pinfo->buffer  = (FAR uint8_t *)priv->runbuffer;
  pinfo->bpp     = LCD_BPP;
  pinfo->dev     = dev;
  return OK;
}

/****************************************************************************
 * Name: lcd_getpower / lcd_setpower
 ****************************************************************************/

static int lcd_getpower(FAR struct lcd_dev_s *dev)
{
  FAR struct nvmpg_lcd_s *priv = (FAR struct nvmpg_lcd_s *)dev;

  return priv->power;
}

static int lcd_setpower(FAR struct lcd_dev_s *dev, int power)
{
  FAR struct nvmpg_lcd_s *priv = (FAR struct nvmpg_lcd_s *)dev;

  if (power > 0)
    {
      lcd_writereg(ILI9225_DISP_CTRL1, 0x0016);
      up_mdelay(2);
      lcd_writereg(ILI9225_DISP_CTRL1, 0x0017);
      priv->power = 1;
    }
  else
    {
      lcd_writereg(ILI9225_DISP_CTRL1, 0x0000);
      priv->power = 0;
    }

  return OK;
}

/****************************************************************************
 * Name: lcd_getcontrast / lcd_setcontrast
 ****************************************************************************/

static int lcd_getcontrast(FAR struct lcd_dev_s *dev)
{
  return -ENOSYS;
}

static int lcd_setcontrast(FAR struct lcd_dev_s *dev, unsigned int contrast)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: lcd_hwinit
 ****************************************************************************/

static void lcd_hwinit(void)
{
  uint16_t id;
  int i;

  /* PB3/PB4 are only usable as GPIO with JTAG-DP disabled
   * (CONFIG_STM32_JTAG_SW_ENABLE keeps SWD available for debugging).
   */

  for (i = 0; i < 16; i++)
    {
      stm32_configgpio(GPIO_LCD_DATA | (i << GPIO_PIN_SHIFT));
    }

  stm32_configgpio(GPIO_LCD_WR);
  stm32_configgpio(GPIO_LCD_RS);
  stm32_configgpio(GPIO_LCD_CS);
  stm32_configgpio(GPIO_LCD_RD);

  up_mdelay(50);
  lcd_writereg(0x00, 0x0001);   /* Start oscillator */
  up_mdelay(50);

  id = lcd_readreg(0x00);
  lcdinfo("LCD ID: %04x\n", id);
  UNUSED(id);

  for (i = 0; i < nitems(g_init_power); i++)
    {
      lcd_writereg(g_init_power[i][0], g_init_power[i][1]);
    }

  up_mdelay(10);

  for (i = 0; i < nitems(g_init_ramp); i++)
    {
      lcd_writereg(ILI9225_POWER_CTRL2, g_init_ramp[i]);
      up_mdelay(i < 3 ? 10 : 20);
    }

  up_mdelay(30);

  for (i = 0; i < nitems(g_init_regs); i++)
    {
      lcd_writereg(g_init_regs[i][0], g_init_regs[i][1]);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_lcd_initialize
 ****************************************************************************/

int board_lcd_initialize(void)
{
  FAR struct nvmpg_lcd_s *priv = &g_lcd;

  if (!priv->initialized)
    {
      lcd_hwinit();
      lcd_fill(0x0000);
      lcd_setpower(&priv->dev, CONFIG_LCD_MAXPOWER);
      priv->initialized = true;
    }

  return OK;
}

/****************************************************************************
 * Name: board_lcd_getdev
 ****************************************************************************/

FAR struct lcd_dev_s *board_lcd_getdev(int lcddev)
{
  if (lcddev != 0)
    {
      return NULL;
    }

  return &g_lcd.dev;
}

/****************************************************************************
 * Name: board_lcd_uninitialize
 ****************************************************************************/

void board_lcd_uninitialize(void)
{
  lcd_setpower(&g_lcd.dev, 0);
}

#endif /* CONFIG_LCD */
