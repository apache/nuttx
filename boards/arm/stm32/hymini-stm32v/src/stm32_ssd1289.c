/****************************************************************************
 * boards/arm/stm32/hymini-stm32v/src/stm32_ssd1289.c
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
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/ssd1289.h>

#include "arm_internal.h"
#include "stm32.h"
#include "hymini-stm32v.h"

#include <arch/board/board.h>  /* Should always be included last due to dependencies */

#ifdef CONFIG_LCD_SSD1289

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_STM32_FSMC
#  error "CONFIG_STM32_FSMC is required to use the LCD"
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

#define LCD_INDEX        0x60000000  /* RS = 0 */
#define LCD_DATA         0x60020000  /* RS = 1 */

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Low Level LCD access */

static void stm32_select(struct ssd1289_lcd_s *dev);
static void stm32_deselect(struct ssd1289_lcd_s *dev);
static void stm32_index(struct ssd1289_lcd_s *dev, uint8_t index);
#ifndef CONFIG_SSD1289_WRONLY
static uint16_t stm32_read(struct ssd1289_lcd_s *dev);
#endif
static void stm32_write(struct ssd1289_lcd_s *dev, uint16_t data);
static void stm32_backlight(struct ssd1289_lcd_s *dev, int power);

static void stm32_extmemgpios(const uint16_t *gpios, int ngpios);

/****************************************************************************
 * Private Data
 ****************************************************************************/

const uint16_t fsmc_gpios[] =
{
  /* A16... A24 */

  GPIO_NPS_A16, GPIO_NPS_A17, GPIO_NPS_A18, GPIO_NPS_A19, GPIO_NPS_A20,
  GPIO_NPS_A21, GPIO_NPS_A22, GPIO_NPS_A23,

  /* D0... D15 */

  GPIO_NPS_D0, GPIO_NPS_D1, GPIO_NPS_D2, GPIO_NPS_D3, GPIO_NPS_D4,
  GPIO_NPS_D5, GPIO_NPS_D6, GPIO_NPS_D7, GPIO_NPS_D8, GPIO_NPS_D9,
  GPIO_NPS_D10, GPIO_NPS_D11, GPIO_NPS_D12, GPIO_NPS_D13, GPIO_NPS_D14,
  GPIO_NPS_D15,

  /* NOE, NWE */

  GPIO_NPS_NOE, GPIO_NPS_NWE,

  /* NE1 */

  GPIO_NPS_NE1
};

#define NGPIOS (sizeof(fsmc_gpios)/sizeof(uint16_t))

/* This is the driver state structure */

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

static struct lcd_dev_s *g_ssd1289drvr;

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

static void stm32_select(struct ssd1289_lcd_s *dev)
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

static void stm32_deselect(struct ssd1289_lcd_s *dev)
{
  /* Does not apply to this hardware */
}

/****************************************************************************
 * Name: stm32_index
 *
 * Description:
 *   Set the index register
 *
 ****************************************************************************/

static void stm32_index(struct ssd1289_lcd_s *dev, uint8_t index)
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
static uint16_t stm32_read(struct ssd1289_lcd_s *dev)
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

static void stm32_write(struct ssd1289_lcd_s *dev, uint16_t data)
{
  putreg16((uint16_t)data, LCD_DATA);
}

/****************************************************************************
 * Name: stm32_backlight
 *
 * Description:
 *   Enable/disable LCD panel power (0: full off - CONFIG_LCD_MAXPOWER:
 *   full on).
 *   Used here to set pwm duty on timer used for backlight.
 *
 ****************************************************************************/

static void stm32_backlight(struct ssd1289_lcd_s *dev, int power)
{
  DEBUGASSERT(power <= CONFIG_LCD_MAXPOWER);

  /* Set new power level */

  if (power > 0)
    {
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

      putreg16((uint16_t)duty, STM32_TIM3_CCR2);
    }
  else
    {
      putreg16((uint16_t)0, STM32_TIM3_CCR2);
    }
}

static void init_lcd_backlight(void)
{
  uint16_t ccmr;
  uint16_t ccer;

  /* Configure PB5 as TIM3 CH2 output */

  stm32_configgpio(GPIO_TIM3_CH2OUT);

  /* Enable timer 3 clocking */

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

  /* Select the Output Compare Mode Bits */

  ccmr  = getreg16(STM32_TIM3_CCMR1);
  ccmr &= ATIM_CCMR1_OC2M_MASK;
  ccmr |= (ATIM_CCMR_MODE_PWM1 << ATIM_CCMR1_OC2M_SHIFT);

  putreg16(0, STM32_TIM3_CCR2);

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

  lcdinfo("APB1ENR: %08" PRIx32 "\n", getreg32(STM32_RCC_APB1ENR));
  lcdinfo("CR1:     %04" PRIx32 "\n", getreg32(STM32_TIM3_CR1));
  lcdinfo("CR2:     %04" PRIx32 "\n", getreg32(STM32_TIM3_CR2));
  lcdinfo("SMCR:    %04" PRIx32 "\n", getreg32(STM32_TIM3_SMCR));
  lcdinfo("DIER:    %04" PRIx32 "\n", getreg32(STM32_TIM3_DIER));
  lcdinfo("SR:      %04" PRIx32 "\n", getreg32(STM32_TIM3_SR));
  lcdinfo("EGR:     %04" PRIx32 "\n", getreg32(STM32_TIM3_EGR));
  lcdinfo("CCMR1:   %04" PRIx32 "\n", getreg32(STM32_TIM3_CCMR1));
  lcdinfo("CCMR2:   %04" PRIx32 "\n", getreg32(STM32_TIM3_CCMR2));
  lcdinfo("CCER:    %04" PRIx32 "\n", getreg32(STM32_TIM3_CCER));
  lcdinfo("CNT:     %04" PRIx32 "\n", getreg32(STM32_TIM3_CNT));
  lcdinfo("PSC:     %04" PRIx32 "\n", getreg32(STM32_TIM3_PSC));
  lcdinfo("ARR:     %04" PRIx32 "\n", getreg32(STM32_TIM3_ARR));
  lcdinfo("CCR1:    %04" PRIx32 "\n", getreg32(STM32_TIM3_CCR1));
  lcdinfo("CCR2:    %04" PRIx32 "\n", getreg32(STM32_TIM3_CCR2));
  lcdinfo("CCR3:    %04" PRIx32 "\n", getreg32(STM32_TIM3_CCR3));
  lcdinfo("CCR4:    %04" PRIx32 "\n", getreg32(STM32_TIM3_CCR4));
  lcdinfo("CCR4:    %04" PRIx32 "\n", getreg32(STM32_TIM3_CCR4));
  lcdinfo("CCR4:    %04" PRIx32 "\n", getreg32(STM32_TIM3_CCR4));
  lcdinfo("DMAR:    %04" PRIx32 "\n", getreg32(STM32_TIM3_DMAR));
}

/****************************************************************************
 * Name: stm32_selectlcd
 *
 * Description:
 *   Initialize the memory controller  (FSMC)
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

  putreg32(FSMC_BTR_ADDSET(1)  | FSMC_BTR_ADDHLD(0) | FSMC_BTR_DATAST(2) |
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
 * Name: stm32_extmemgpios
 *
 * Description:
 *   Initialize GPIOs for NOR or SRAM
 *
 ****************************************************************************/

static void stm32_extmemgpios(const uint16_t *gpios, int ngpios)
{
  int i;

  /* Configure GPIOs */

  for (i = 0; i < ngpios; i++)
    {
      stm32_configgpio(gpios[i]);
    }
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
  /* Only initialize the driver once */

  if (!g_ssd1289drvr)
    {
      lcdinfo("Initializing\n");

      init_lcd_backlight();

      /* Configure GPIO pins and configure the FSMC to support the LCD */

      stm32_selectlcd();

      /* Configure and enable the LCD */

      up_mdelay(50);
      g_ssd1289drvr = ssd1289_lcdinitialize(&g_ssd1289);
      if (!g_ssd1289drvr)
        {
          lcderr("ERROR: ssd1289_lcdinitialize failed\n");
          return -ENODEV;
        }
    }

  /* Turn the display off */

  g_ssd1289drvr->setpower(g_ssd1289drvr, 0);
  return OK;
}

/****************************************************************************
 * Name:  board_lcd_getdev
 *
 * Description:
 *   Return a a reference to the LCD object for the specified LCD.  This
 *   allows support for multiple LCD devices.
 *
 ****************************************************************************/

struct lcd_dev_s *board_lcd_getdev(int lcddev)
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
