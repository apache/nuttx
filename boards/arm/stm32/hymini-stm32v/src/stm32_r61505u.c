/****************************************************************************
 * boards/arm/stm32/hymini-stm32v/src/stm32_r61505u.c
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

#include "arm_internal.h"
#include "stm32.h"
#include "hymini-stm32v.h"

#include <arch/board/board.h>  /* Should always be included last due to dependencies */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_STM32_FSMC
#  error CONFIG_STM32_FSMC is required for LCD support
#endif

/* Color depth and format */

#define LCD_BPP          16
#define LCD_COLORFMT     FB_FMT_RGB16_565

/* Display Resolution */

#if defined(CONFIG_LCD_LANDSCAPE)
#  define LCD_XRES       320
#  define LCD_YRES       240
#else
#  define LCD_XRES       240
#  define LCD_YRES       320
#endif

#define LCD_BL_TIMER_PERIOD 8999

/* LCD is connected to the FSMC_Bank1_NOR/SRAM1 and NE1 is used as ship
 * select signal
 */

/* RS <==> A16 */

#define LCD_REG          (*((volatile unsigned short *) 0x60000000)) /* RS = 0 */
#define LCD_RAM          (*((volatile unsigned short *) 0x60020000)) /* RS = 1 */

/* LCD IDs */

#define LCD_ID           0x1505

/* This should be put elsewhere (possibly include/nuttx/compiler.h) */

#ifdef __CC_ARM               /* ARM Compiler        */
#  define lcd_inline          static __inline
#elif defined (__ICCARM__)    /* for IAR Compiler */
#  define lcd_inline          inline
#elif defined (__GNUC__)      /* GNU GCC Compiler */
#  define lcd_inline          static __inline
#else
#  define lcd_inline          static
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mylcd_dev_s
{
  /* Publicly visible device structure */

  struct lcd_dev_s dev;

  /* Private LCD-specific information follows */

  uint8_t power; /* Current power setting */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Low Level methods */

static void lcd_clear(uint16_t color);

/* LCD Data Transfer Methods */

static int lcd_putrun(fb_coord_t row, fb_coord_t col,
                      const uint8_t *buffer, size_t npixels);
static int lcd_getrun(fb_coord_t row, fb_coord_t col,
                      uint8_t *buffer, size_t npixels);

/* LCD Configuration */

static int lcd_getvideoinfo(struct lcd_dev_s *dev,
                            struct fb_videoinfo_s *vinfo);
static int lcd_getplaneinfo(struct lcd_dev_s *dev,
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

static int lcd_getpower(struct lcd_dev_s *dev);
static int lcd_setpower(struct lcd_dev_s *dev, int power);
static int lcd_getcontrast(struct lcd_dev_s *dev);
static int lcd_setcontrast(struct lcd_dev_s *dev,
                           unsigned int contrast);

/* Initialization (LCD ctrl / backlight) */

static inline void lcd_initialize(void);

#ifdef CONFIG_LCD_BACKLIGHT
static void lcd_backlight(void);
#else
#  define lcd_backlight()
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint16_t fsmc_gpios[] =
{
  /* A16... A24 */

  GPIO_NPS_A16, GPIO_NPS_A17, GPIO_NPS_A18, GPIO_NPS_A19, GPIO_NPS_A20,
  GPIO_NPS_A21, GPIO_NPS_A22, GPIO_NPS_A23,

  /* D0... D15 */

  GPIO_NPS_D0, GPIO_NPS_D1, GPIO_NPS_D2, GPIO_NPS_D3,
  GPIO_NPS_D4, GPIO_NPS_D5, GPIO_NPS_D6, GPIO_NPS_D7,
  GPIO_NPS_D8, GPIO_NPS_D9, GPIO_NPS_D10, GPIO_NPS_D11,
  GPIO_NPS_D12, GPIO_NPS_D13, GPIO_NPS_D14, GPIO_NPS_D15,

  /* NOE, NWE  */

  GPIO_NPS_NOE, GPIO_NPS_NWE,

  /* NE1  */

  GPIO_NPS_NE1
};

#define NGPIOS (sizeof(fsmc_gpios)/sizeof(uint16_t))

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

static uint16_t g_runbuffer[LCD_XRES];

/* This structure describes the overall LCD video controller */

static const struct fb_videoinfo_s g_videoinfo =
{
  .fmt     = LCD_COLORFMT, /* Color format: RGB16-565: RRRR RGGG GGGB BBBB */
  .xres    = LCD_XRES,     /* Horizontal resolution in pixel columns */
  .yres    = LCD_YRES,     /* Vertical resolution in pixel rows */
  .nplanes = 1,            /* Number of color planes supported */
};

/* This is the standard, NuttX Plane information object */

static const struct lcd_planeinfo_s g_planeinfo =
{
  .putrun = lcd_putrun,              /* Put a run into LCD memory */
  .getrun = lcd_getrun,              /* Get a run from LCD memory */
  .buffer = (uint8_t *) g_runbuffer, /* Run scratch buffer */
  .bpp    = LCD_BPP,                 /* Bits-per-pixel */
};

/* This is the standard, NuttX LCD driver object */

static struct mylcd_dev_s g_lcddev =
{ .dev =
  {
    /* LCD Configuration */

    .getvideoinfo = lcd_getvideoinfo,
    .getplaneinfo = lcd_getplaneinfo,

    /* LCD RGB Mapping -- Not supported */

    /* Cursor Controls -- Not supported */

    /* LCD Specific Controls */

    .getpower     = lcd_getpower,
    .setpower     = lcd_setpower,
    .getcontrast  = lcd_getcontrast,
    .setcontrast  = lcd_setcontrast,
  },
  .power = 0
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_extmemgpios
 *
 * Description:
 *   Initialize GPIOs for NOR or SRAM
 *
 ****************************************************************************/

static inline void stm32_extmemgpios(const uint16_t *gpios, int ngpios)
{
  int i;

  /* Configure GPIOs */

  for (i = 0; i < ngpios; i++)
    {
      stm32_configgpio(gpios[i]);
    }
}

/****************************************************************************
 * Name: stm32_selectlcd
 *
 * Description:
 *   Initialize to the LCD
 *
 ****************************************************************************/

static void stm32_selectlcd(void)
{
  /* Configure new GPIO state */

  stm32_extmemgpios(fsmc_gpios, NGPIOS);

  /* Enable AHB clocking to the FSMC */

  stm32_fsmc_enable();

  /* Bank1 NOR/SRAM control register configuration */

  putreg32(FSMC_BCR_SRAM | FSMC_BCR_MWID16 | FSMC_BCR_WREN, STM32_FSMC_BCR1);

  /* Bank1 NOR/SRAM timing register configuration */

  putreg32(FSMC_BTR_ADDSET(2)  | FSMC_BTR_ADDHLD(0) | FSMC_BTR_DATAST(2) |
           FSMC_BTR_BUSTURN(0) | FSMC_BTR_CLKDIV(0) | FSMC_BTR_DATLAT(0) |
           FSMC_BTR_ACCMODA,
           STM32_FSMC_BTR1);

  /* As ext mode is not active the write timing is ignored!! */

  putreg32(0xffffffff, STM32_FSMC_BWTR1);

  /* Enable the bank by setting the MBKEN bit */

  putreg32(FSMC_BCR_MBKEN | FSMC_BCR_SRAM | FSMC_BCR_MWID16 | FSMC_BCR_WREN,
           STM32_FSMC_BCR1);
}

/****************************************************************************
 * Name: stm32_deselectlcd
 *
 * Description:
 *   Disable the LCD
 *
 ****************************************************************************/

static void stm32_deselectlcd(void)
{
  /* Restore registers to their power up settings */

  putreg32(0xffffffff, STM32_FSMC_BCR4);

  /* Bank1 NOR/SRAM timing register configuration */

  putreg32(0x0fffffff, STM32_FSMC_BTR4);

  /* Disable AHB clocking to the FSMC */

  stm32_fsmc_disable();
}

/****************************************************************************
 * Name:  write_cmd
 ****************************************************************************/

lcd_inline void write_cmd(unsigned short cmd)
{
  LCD_REG = cmd;
}

/****************************************************************************
 * Name:  read_data
 ****************************************************************************/

lcd_inline unsigned short read_data(void)
{
  return LCD_RAM;
}

/****************************************************************************
 * Name:  write_data
 ****************************************************************************/

lcd_inline void write_data(unsigned short data_code)
{
  LCD_RAM = data_code;
}

/****************************************************************************
 * Name:  write_reg
 ****************************************************************************/

static void write_reg(unsigned char reg_addr, unsigned short reg_val)
{
  write_cmd(reg_addr);
  write_data(reg_val);
}

/****************************************************************************
 * Name:  read_reg
 ****************************************************************************/

static unsigned short read_reg(unsigned char reg_addr)
{
  unsigned short val;
  write_cmd(reg_addr);
  val = read_data();
  return (val);
}

/****************************************************************************
 * Name:  lcd_gramselect
 ****************************************************************************/

static inline void lcd_gramselect(void)
{
  write_cmd(0x22);
}

/****************************************************************************
 * Name:  lcd_setcursor
 ****************************************************************************/

static void lcd_setcursor(unsigned int x, unsigned int y)
{
#if defined(CONFIG_LCD_PORTRAIT) || defined (CONFIG_LCD_RPORTRAIT)
# if defined (CONFIG_LCD_RPORTRAIT)
  x = (LCD_XRES - 1) - x;
  y = (LCD_YRES - 1) - y;
# endif
  write_reg(0x20, x); /* Row */
  write_reg(0x21, y); /* Line */
#endif

#if defined(CONFIG_LCD_LANDSCAPE)
  y = (LCD_YRES - 1) - y;

  write_reg(0x20, x); /* Row */
  write_reg(0x21, y); /* Line */
#endif
}

/****************************************************************************
 * Name:  lcd_putrun
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

static int lcd_putrun(fb_coord_t row, fb_coord_t col,
                      const uint8_t *buffer,
                      size_t npixels)
{
  int i;
  const uint16_t *src = (const uint16_t *) buffer;

  /* Buffer must be provided and aligned to a 16-bit address boundary */

  DEBUGASSERT(buffer && ((uintptr_t)buffer & 1) == 0);

  /* Write the run to GRAM. */

  lcd_setcursor(col, row);
  lcd_gramselect();

  for (i = 0; i < npixels; i++)
    {
      write_data(*src++);
    }

  return OK;
}

/****************************************************************************
 * Name:  lcd_getrun
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

static int lcd_getrun(fb_coord_t row, fb_coord_t col, uint8_t *buffer,
                      size_t npixels)
{
  uint16_t *dest = (uint16_t *) buffer;
  int i;

  /* Buffer must be provided and aligned to a 16-bit address boundary */

  DEBUGASSERT(buffer && ((uintptr_t)buffer & 1) == 0);

  /* Read the run from GRAM. */

  lcd_setcursor(col, row);
  lcd_gramselect();

  /* dummy read */

  read_data();

  for (i = 0; i < npixels; i++)
    {
      *dest++ = read_data();
    }

  return OK;
}

/****************************************************************************
 * Name:  lcd_getvideoinfo
 *
 * Description:
 *   Get information about the LCD video controller configuration.
 *
 ****************************************************************************/

static int lcd_getvideoinfo(struct lcd_dev_s *dev,
                            struct fb_videoinfo_s *vinfo)
{
  DEBUGASSERT(dev && vinfo);
  ginfo("fmt: %d xres: %d yres: %d nplanes: %d\n",
        g_videoinfo.fmt, g_videoinfo.xres,
        g_videoinfo.yres, g_videoinfo.nplanes);

  memcpy(vinfo, &g_videoinfo, sizeof(struct fb_videoinfo_s));
  return OK;
}

/****************************************************************************
 * Name:  lcd_getplaneinfo
 *
 * Description:
 *   Get information about the configuration of each LCD color plane.
 *
 ****************************************************************************/

static int lcd_getplaneinfo(struct lcd_dev_s *dev, unsigned int planeno,
                            struct lcd_planeinfo_s *pinfo)
{
  DEBUGASSERT(dev && pinfo && planeno == 0);
  ginfo("planeno: %d bpp: %d\n", planeno, g_planeinfo.bpp);

  memcpy(pinfo, &g_planeinfo, sizeof(struct lcd_planeinfo_s));
  return OK;
}

/****************************************************************************
 * Name:  lcd_getpower
 *
 * Description:
 *   Get the LCD panel power status
 *   (0: full off - CONFIG_LCD_MAXPOWER: full on). On
 *   backlit LCDs, this setting may correspond to the backlight setting.
 *
 ****************************************************************************/

static int lcd_getpower(struct lcd_dev_s *dev)
{
  ginfo("power: %d\n", 0);
  return g_lcddev.power;
}

/****************************************************************************
 * Name:  lcd_setpower
 *
 * Description:
 *   Enable/disable LCD panel power
 *  (0: full off - CONFIG_LCD_MAXPOWER: full on).
 *   Used here to set pwm duty on timer used for backlight.
 *
 ****************************************************************************/

static int lcd_setpower(struct lcd_dev_s *dev, int power)
{
  if (g_lcddev.power == power)
    {
      return OK;
    }

  ginfo("power: %d\n", power);
  DEBUGASSERT(power <= CONFIG_LCD_MAXPOWER);

  /* Set new power level */

  if (power > 0)
    {
#ifdef CONFIG_LCD_BACKLIGHT
      uint32_t duty;

      /* Calculate the new backlight duty.  It is a fraction of the timer
       * period based on the ration of the current power setting to the
       * maximum power setting.
       */

      duty = ((uint32_t)LCD_BL_TIMER_PERIOD * (uint32_t)power) /
              CONFIG_LCD_MAXPOWER;
      if (duty >= LCD_BL_TIMER_PERIOD)
        {
          duty = LCD_BL_TIMER_PERIOD - 1;
        }

      ginfo("PWM duty: %d\n", duty);
      putreg16((uint16_t)duty, STM32_TIM3_CCR2);
#endif
      /* TODO turn the display on */
    }
  else
    {
      /* FIXME: Turn display off ? */

      ginfo("Force PWM to 0\n");
      putreg16((uint16_t)0, STM32_TIM3_CCR2);
    }

  g_lcddev.power = power;
  return OK;
}

/****************************************************************************
 * Name:  lcd_getcontrast
 *
 * Description:
 *   Get the current contrast setting (0-CONFIG_LCD_MAXCONTRAST).
 *
 ****************************************************************************/

static int lcd_getcontrast(struct lcd_dev_s *dev)
{
  ginfo("Not implemented\n");
  return -ENOSYS;
}

/****************************************************************************
 * Name:  lcd_setcontrast
 *
 * Description:
 *   Set LCD panel contrast (0-CONFIG_LCD_MAXCONTRAST).
 *
 ****************************************************************************/

static int lcd_setcontrast(struct lcd_dev_s *dev, unsigned int contrast)
{
  ginfo("Not implemented\n");
  return -ENOSYS;
}

/****************************************************************************
 * Name:  lcd_lcdinitialize
 *
 * Description:
 *   Set LCD panel contrast (0-CONFIG_LCD_MAXCONTRAST).
 *
 ****************************************************************************/

static inline void lcd_initialize(void)
{
  /* Second release on 3/5  ,luminance is acceptable,water wave appear
   * during camera preview
   */

  write_reg(0x07, 0x0000);
  up_mdelay(5);               /* Delay 50 ms */
  write_reg(0x12, 0x011c);    /* Why need to set several times? */
  write_reg(0xa4, 0x0001);    /* NVM */
  write_reg(0x08, 0x000f);
  write_reg(0x0a, 0x0008);
  write_reg(0x0d, 0x0008);

  /* GAMMA CONTROL */

  write_reg(0x30, 0x0707);
  write_reg(0x31, 0x0007);
  write_reg(0x32, 0x0603);
  write_reg(0x33, 0x0700);
  write_reg(0x34, 0x0202);
  write_reg(0x35, 0x0002);
  write_reg(0x36, 0x1f0f);
  write_reg(0x37, 0x0707);
  write_reg(0x38, 0x0000);
  write_reg(0x39, 0x0000);
  write_reg(0x3a, 0x0707);
  write_reg(0x3b, 0x0000);
  write_reg(0x3c, 0x0007);
  write_reg(0x3d, 0x0000);
  up_mdelay(5);               /* Delay 50 ms */
  write_reg(0x07, 0x0001);
  write_reg(0x17, 0x0001);    /* Power supply startup enable */
  up_mdelay(5);               /* Delay 50 ms */

  /* Power control */

  write_reg(0x10, 0x17a0);
  write_reg(0x11, 0x0217);    /* Feference voltage VC[2:0]   Vciout = 1.00*Vcivl */
  write_reg(0x12, 0x011e);    /* Vreg1out = Vcilvl*1.80   is it the same as Vgama1out ?  */
  write_reg(0x13, 0x0f00);    /* VDV[4:0]-->VCOM Amplitude VcomL = VcomH - Vcom Ampl */
  write_reg(0x2a, 0x0000);
  write_reg(0x29, 0x000a);    /* Vcomh = VCM1[4:0]*Vreg1out    gate source voltage?? */
  write_reg(0x12, 0x013e);    /* Power supply on */

  /* Coordinates Control */

  write_reg(0x50, 0x0000);
  write_reg(0x51, 0x00ef);
  write_reg(0x52, 0x0000);
  write_reg(0x53, 0x013f);

  /* Panel Image Control */

  write_reg(0x60, 0x2700);
  write_reg(0x61, 0x0001);
  write_reg(0x6a, 0x0000);
  write_reg(0x80, 0x0000);

  /* Partial Image Control */

  write_reg(0x81, 0x0000);
  write_reg(0x82, 0x0000);
  write_reg(0x83, 0x0000);
  write_reg(0x84, 0x0000);
  write_reg(0x85, 0x0000);

  /* Panel Interface Control */

  write_reg(0x90, 0x0013);    /* Frequency */
  write_reg(0x92, 0x0300);
  write_reg(0x93, 0x0005);
  write_reg(0x95, 0x0000);
  write_reg(0x97, 0x0000);
  write_reg(0x98, 0x0000);

  write_reg(0x01, 0x0100);
  write_reg(0x02, 0x0700);
  write_reg(0x03, 0x1030);
  write_reg(0x04, 0x0000);
  write_reg(0x0c, 0x0000);
  write_reg(0x0f, 0x0000);
  write_reg(0x20, 0x0000);
  write_reg(0x21, 0x0000);
  write_reg(0x07, 0x0021);
  up_mdelay(20);             /* Delay 200 ms */
  write_reg(0x07, 0x0061);
  up_mdelay(20);             /* Delay 200 ms */
  write_reg(0x07, 0x0173);
  up_mdelay(20);             /* Delay 200 ms */
}

/****************************************************************************
 * Name:  lcd_backlight
 *
 * Description:
 *   The LCD backlight is driven from PB.5 which must be configured as TIM3
 *   CH2.  TIM3 must then be configured to pwm output on PB.5; the duty
 *   of the clock determines the backlight level.
 *
 ****************************************************************************/

#ifdef CONFIG_LCD_BACKLIGHT

#ifndef CONFIG_STM32_TIM3_PARTIAL_REMAP
#  error CONFIG_STM32_TIM3_PARTIAL_REMAP must be set (to have TIM3 CH2 on pin B.5)
#endif

static void lcd_backlight(void)
{
  uint16_t ccmr;
  uint16_t ccer;
  uint16_t cr2;

  /* Configure PB5 as TIM3 CH2 output */

  stm32_configgpio(GPIO_TIM3_CH2OUT);

  /* Enabled timer 3 clocking */

  modifyreg32(STM32_RCC_APB1ENR, 0, RCC_APB1ENR_TIM3EN);

  /* Reset timer 3 */

  modifyreg32(STM32_RCC_APB1RSTR, 0, RCC_APB1RSTR_TIM3RST);
  modifyreg32(STM32_RCC_APB1RSTR, RCC_APB1RSTR_TIM3RST, 0);

  /* Reset the Counter Mode and set the clock division */

  putreg16(0, STM32_TIM3_CR1);

  /* Set the Autoreload value */

  putreg16(LCD_BL_TIMER_PERIOD, STM32_TIM3_ARR);

  /* Set the Prescaler value */

  putreg16(0, STM32_TIM3_PSC);

  /* Generate an update event to reload the Prescaler value immediately */

  putreg16(ATIM_EGR_UG, STM32_TIM3_EGR);

  /* Disable the Channel 2 */

  ccer  = getreg16(STM32_TIM3_CCER);
  ccer &= ~ATIM_CCER_CC2E;
  putreg16(ccer, STM32_TIM3_CCER);

  /* Get the TIM3 CR2 register value */

  cr2  = getreg16(STM32_TIM3_CR2);

  /* Select the Output Compare Mode Bits */

  ccmr  = getreg16(STM32_TIM3_CCMR1);
  ccmr &= ATIM_CCMR1_OC2M_MASK;
  ccmr |= (ATIM_CCMR_MODE_PWM1 << ATIM_CCMR1_OC2M_SHIFT);

  /* Set the capture compare register value (50% duty) */

  /* FIXME should be set to 0
   *  (appl needs to call setpower to change it)
   */

  g_lcddev.power = (CONFIG_LCD_MAXPOWER + 1) / 2;
  putreg16((LCD_BL_TIMER_PERIOD + 1) / 2, STM32_TIM3_CCR2);

  /* Select the output polarity level == HIGH */

  ccer &= !ATIM_CCER_CC2P;

  /* Enable channel 2 */

  ccer |= ATIM_CCER_CC2E;

  /* Write the timer configuration */

  putreg16(ccmr, STM32_TIM3_CCMR1);
  putreg16(ccer, STM32_TIM3_CCER);

  /* Set the auto preload enable bit */

  modifyreg16(STM32_TIM3_CR1, 0, ATIM_CR1_ARPE);

  /* Enable Backlight Timer !!!! */

  modifyreg16(STM32_TIM3_CR1, 0, ATIM_CR1_CEN);

  /* Dump timer3 registers */

  lcdinfo("APB1ENR: %08x\n", getreg32(STM32_RCC_APB1ENR));
  lcdinfo("CR1:     %04x\n", getreg32(STM32_TIM3_CR1));
  lcdinfo("CR2:     %04x\n", getreg32(STM32_TIM3_CR2));
  lcdinfo("SMCR:    %04x\n", getreg32(STM32_TIM3_SMCR));
  lcdinfo("DIER:    %04x\n", getreg32(STM32_TIM3_DIER));
  lcdinfo("SR:      %04x\n", getreg32(STM32_TIM3_SR));
  lcdinfo("EGR:     %04x\n", getreg32(STM32_TIM3_EGR));
  lcdinfo("CCMR1:   %04x\n", getreg32(STM32_TIM3_CCMR1));
  lcdinfo("CCMR2:   %04x\n", getreg32(STM32_TIM3_CCMR2));
  lcdinfo("CCER:    %04x\n", getreg32(STM32_TIM3_CCER));
  lcdinfo("CNT:     %04x\n", getreg32(STM32_TIM3_CNT));
  lcdinfo("PSC:     %04x\n", getreg32(STM32_TIM3_PSC));
  lcdinfo("ARR:     %04x\n", getreg32(STM32_TIM3_ARR));
  lcdinfo("CCR1:    %04x\n", getreg32(STM32_TIM3_CCR1));
  lcdinfo("CCR2:    %04x\n", getreg32(STM32_TIM3_CCR2));
  lcdinfo("CCR3:    %04x\n", getreg32(STM32_TIM3_CCR3));
  lcdinfo("CCR4:    %04x\n", getreg32(STM32_TIM3_CCR4));
  lcdinfo("CCR4:    %04x\n", getreg32(STM32_TIM3_CCR4));
  lcdinfo("CCR4:    %04x\n", getreg32(STM32_TIM3_CCR4));
  lcdinfo("DMAR:    %04x\n", getreg32(STM32_TIM3_DMAR));
}
#endif

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
  unsigned short id;

  ginfo("Initializing\n");

  /* Configure GPIO pins and configure the FSMC to support the LCD */

  stm32_selectlcd();

  /* Delay required here */

  up_mdelay(50);

  /* Check model id */

  id = read_reg(0x0);
  if (id != LCD_ID)
    {
      /* Not a R61505U ? */

      lcderr("ERROR: board_lcd_initialize: LCD ctrl is not a R61505U");
      return -ENXIO;
    }

  /* Configure and enable LCD */

  lcd_initialize();

  /* Clear the display (setting it to the color 0=black) */

  lcd_clear(0);

  /* Configure the backlight */

  lcd_backlight();
  return OK;
}

/****************************************************************************
 * Name:  board_lcd_getdev
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
 *   Un-initialize the LCD support
 *
 ****************************************************************************/

void board_lcd_uninitialize(void)
{
  lcd_setpower(&g_lcddev.dev, 0);
  stm32_deselectlcd();
}

/****************************************************************************
 * Name:  lcd_clear
 *
 * Description:
 *   Fill the LCD ctrl memory with given color
 *
 ****************************************************************************/

void lcd_clear(uint16_t color)
{
  uint32_t index;

  lcd_setcursor(0, 0);
  lcd_gramselect(); /* Prepare to write GRAM */
  for (index = 0; index < LCD_XRES * LCD_YRES; index++)
    {
      write_data(color);
    }
}
