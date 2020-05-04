/****************************************************************************
 * boards/arm/stm32/viewtool-stm32f107/src/ssd1289.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/ssd1289.h>

#include <arch/board/board.h>

#include "arm_arch.h"
#include "stm32.h"
#include "viewtool_stm32f107.h"

#ifdef CONFIG_LCD_SSD1289

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration **********************************************************************/

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

/* LCD is connected to the FSMC_Bank1_NOR/SRAM1 and NE1 is used as chip select signal */
/* RS <==> A16 */

#define LCD_INDEX        0x60000000  /* RS = 0 */
#define LCD_DATA         0x60020000  /* RS = 1 */

/****************************************************************************
 * Private Function Prototypes
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

static void stm32_extmemgpios(const uint16_t *gpios, int ngpios);

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* LCD
 *
 * An LCD may be connected via J11.  Only the STM32F103 supports the FSMC signals
 * needed to drive the LCD.
 *
 * The LCD features an (1) HY32D module with built-in SSD1289 LCD controller, and (a)
 * a XPT2046 touch screen controller.
 *
 * LCD Connector
 * -------------
 *
 *   ----------------------------- ------------------------ --------------------------------
 *          Connector J11           GPIO CONFIGURATION(s)
 *   PIN SIGNAL        LEGEND          (F103 only)                   LCD Module
 *   --- ------------- ----------- ------------------------ --------------------------------
 *   1   VDD_5         NC          N/A                      5V      ---
 *   2   GND           GND         N/A                      GND     ---
 *   3   PD14          DATA0       GPIO_NPS_D0              D0      HY32D
 *   4   PD15          DATA1       GPIO_NPS_D1              D1      HY32D
 *   5   PD0           DATA2       GPIO_NPS_D2              D2      HY32D
 *   6   PD1           DATA3       GPIO_NPS_D3              D3      HY32D
 *   7   PE7           DATA4       GPIO_NPS_D4              D4      HY32D
 *   8   PE8           DATA5       GPIO_NPS_D5              D5      HY32D
 *   9   PE9           DATA6       GPIO_NPS_D6              D6      HY32D
 *   10  PE10          DATA7       GPIO_NPS_D7              D7      HY32D
 *   11  PE11          DATA8       GPIO_NPS_D8              D8      HY32D
 *   12  PE12          DATA9       GPIO_NPS_D9              D9      HY32D
 *   13  PE13          DATA10      GPIO_NPS_D10             D10     HY32D
 *   14  PE14          DATA11      GPIO_NPS_D11             D11     HY32D
 *   15  PE15          DATA12      GPIO_NPS_D12             D12     HY32D
 *   16  PD8           DATA13      GPIO_NPS_D13             D13     HY32D
 *   17  PD9           DATA14      GPIO_NPS_D14             D14     HY32D
 *   18  PD10          DATA15      GPIO_NPS_D15             D15     HY32D
 *   19  (3)           LCD_CS      GPIO_NPS_NE1             CS      HY32D
 *   20  PD11          LCD_RS      GPIO_NPS_A16             RS      HY32D
 *   21  PD5           LCD_R/W     GPIO_NPS_NWE             WR      HY32D
 *   22  PD4           LCD_RD      GPIO_NPS_NOE             RD      HY32D
 *   23  PB1           LCD_RESET   (GPIO)                   RESET   HY32D
 *   24  N/C           NC          N/A                      TE      (unused?)
 *   25  VDD_3.3       BL_VCC      N/A                      BLVDD   CA6219 (Drives LCD backlight)
 *   26  GND           BL_GND      N/A                      BLGND   CA6219
 *   27  PB0           BL_PWM      GPIO_TIM3_CH3OUT(2)      BL_CNT  CA6219
 *   28  PC5           LCDTP_IRQ   (GPIO)                   TP_IRQ  XPT2046
 *   29  PC4           LCDTP_CS    (GPIO)                   TP_CS   XPT2046
 *   30  PB13          LCDTP_CLK   GPIO_SPI2_SCK            TP_SCK  XPT2046
 *   31  PB15          LCDTP_DIN   GPIO_SPI2_MOSI           TP_SI   XPT2046
 *   32  PB14          LCDTP_DOUT  GPIO_SPI2_MISO           TP_SO   XPT2046
 *   33  VDD_3.3       VDD_3.3     N/A                      3.3V    ---
 *   34  GND           GND         N/A                      GND     ---
 *   --- ------------- ----------- ------------------------ --------------------------------
 *
 *   NOTES:
 *   1) Only the F103 version of the board supports the FSMC
 *   2) No remap
 *   3) LCD_CS is controlled by J13 JUMPER4 (under the LCD unfortunately):
 *
 *      1->2 : PD7 (GPIO_NPS_NE1) enables the multiplexor  : 1E\ enable input (active LOW)
 *      3->4 : PD13 provides 1A0 input (1A1 is grounded).  : 1A0 address input
 *             So will chip enable to either LCD_CS or
 *             Flash_CS.
 *      5->6 : 1Y0 output to LCD_CS                        : 1Y0 address output
 *      7->8 : 1Y1 output to Flash_CE                      : 1Y1 address output
 *
 *      Truth Table:
 *      1E\ 1A0 1A1 1Y0 1Y1
 *      --- --- --- --- ---
 *      HI  N/A N/A HI  HI
 *      LO  LO  LO  LO  HI
 *      LO  HI  LO  HI  LO
 */

const uint16_t fsmc_gpios[] =
{
  /* A16... A23.  REVIST: only A16 is used by the LCD */

  GPIO_NPS_A16, GPIO_NPS_A17, GPIO_NPS_A18, GPIO_NPS_A19, GPIO_NPS_A20,
  GPIO_NPS_A21, GPIO_NPS_A22, GPIO_NPS_A23,

  /* D0... D15 */

  GPIO_NPS_D0,  GPIO_NPS_D1,  GPIO_NPS_D2,  GPIO_NPS_D3,  GPIO_NPS_D4,
  GPIO_NPS_D5,  GPIO_NPS_D6,  GPIO_NPS_D7,  GPIO_NPS_D8,  GPIO_NPS_D9,
  GPIO_NPS_D10, GPIO_NPS_D11, GPIO_NPS_D12, GPIO_NPS_D13, GPIO_NPS_D14,
  GPIO_NPS_D15,

  /* NOE, NWE, and NE1 */

  GPIO_NPS_NOE, GPIO_NPS_NWE, GPIO_NPS_NE1
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
 * Name: stm32_index
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
 * Name: stm32_backlight
 *
 * Description:
 *   Enable/disable LCD panel power (0: full off - CONFIG_LCD_MAXPOWER: full on).
 *   Used here to set pwm duty on timer used for backlight.
 *
 ****************************************************************************/

static void stm32_backlight(FAR struct ssd1289_lcd_s *dev, int power)
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

      duty = ((uint32_t)LCD_BL_TIMER_PERIOD * (uint32_t)power) / CONFIG_LCD_MAXPOWER;
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

  /* Enable channel 2*/

  ccer |= ATIM_CCER_CC2E;

  /* Write the timer configuration */

  putreg16(ccmr, STM32_TIM3_CCMR1);
  putreg16(ccer, STM32_TIM3_CCER);

  /* Set the auto preload enable bit */

  modifyreg16(STM32_TIM3_CR1, 0, ATIM_CR1_ARPE);

  /* Enable Backlight Timer !!!!*/

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

  putreg32(
      FSMC_BTR_ADDSET(1)|FSMC_BTR_ADDHLD(0)|FSMC_BTR_DATAST(2)|FSMC_BTR_BUSTURN(0)| FSMC_BTR_CLKDIV(0)|FSMC_BTR_DATLAT(0)|FSMC_BTR_ACCMODA,
      STM32_FSMC_BTR1);

  /* As ext mode is not active the write timing is ignored!! */

  putreg32(0xffffffff, STM32_FSMC_BWTR1);

  /* Enable the bank by setting the MBKEN bit */

  putreg32(FSMC_BCR_MBKEN | FSMC_BCR_SRAM | FSMC_BCR_MWID16 | FSMC_BCR_WREN,
      STM32_FSMC_BCR1);

  /* Configure the LCD RESET\ pin.  Initial value will take the LCD out of reset */

  stm32_configgpio(GPIO_LCD_RESET);
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
 *   Initialize the LCD video hardware.  The initial state of the LCD is fully
 *   initialized, display memory cleared, and the LCD ready to use, but with the power
 *   setting at 0 (full off).
 *
 ****************************************************************************/

int board_lcd_initialize(void)
{
  /* Only initialize the driver once */

  if (!g_ssd1289drvr)
    {
      lcdinfo("Initializing\n");

      /* Initialize the backlight */

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
 *   Return a a reference to the LCD object for the specified LCD.  This allows support
 *   for multiple LCD devices.
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
