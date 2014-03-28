/**************************************************************************************
 * configs/sam4e-ek/src/sam_ili9325.c
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
 **************************************************************************************/

/**************************************************************************************
 *
 * The SAM4E-EK carries a TFT transmissive LCD module with touch panel, FTM280C34D.
 * Its integrated driver IC is ILI9325. The LCD display area is 2.8 inches diagonally
 * measured, with a native resolution of 240 x 320 dots.
 *
 * The SAM4E16 communicates with the LCD through PIOC where an 8-bit parallel "8080-
 * like" protocol data bus has to be implemented in software.
 *
 *  ---- ----- --------- --------------------------------
 *  PIN  PIO   SIGNAL    NOTES
 *  ---- ----- --------- --------------------------------
 *    1                  VDD
 *    2  PC7   DB17
 *    3  PC6   DB16
 *    4  PC5   DB15
 *    5  PC4   DB14
 *    6  PC3   DB13
 *    7  PC2   DB12
 *    8  PC1   DB11
 *    9  PC0   DB10
 *   10        DB9       Pulled low
 *   11        DB8       Pulled low
 *   12        DB7       Pulled low
 *   13        DB6       Pulled low
 *   14        DB5       Pulled low
 *   15        DB4       Pulled low
 *   16        DB3       Pulled low
 *   17        DB2       Pulled low
 *   18        DB1       Pulled low
 *   19        DB0       Pulled low
 *  ---- ----- --------- --------------------------------
 *   20                  VDD
 *   21  PC11  RD
 *   22  PC8   WR
 *   23  PC19  RS
 *   24  PD18  CS        Via J8, pulled high.  Connects to NRST.
 *   25        RESET     Connects to NSRST
 *   26        IM0       Pulled high
 *   27        IM1       Grounded
 *   28        GND
 *  ---- ----- --------- --------------------------------
 *   29 [PC13] LED-A     Backlight controls:  PC13 enables
 *   30 [PC13] LEDK1       AAT3155 charge pump that drives
 *   31 [PC13] LEDK2       the backlight LEDs
 *   32 [PC13] LEDK3
 *   33 [PC13] LEDK4
 *   34 [PC13] LEDK1
 *  ---- ----- --------- --------------------------------
 *   35        Y+        These go to the ADS7843
 *   36        Y-          touchscreen controller.
 *   37        X+
 *   38        X-
 *   39        NC
 *  ---- ----- --------- --------------------------------
 *
 * LCD backlight is made of 4 white chip LEDs in parallel, driven by an AAT3155
 * charge pump, MN4. The AAT3155 is controlled by the SAM3U4E through a single line
 * Simple Serial Control (S2Cwire) interface, which permits to enable, disable, and
 * set the LED drive current (LED brightness control) from a 32-level logarithmic
 * scale. Four resistors R93/R94/R95/R96 are implemented for optional current
 * limitation.
 *
 **************************************************************************************/

/**************************************************************************************
 * Included Files
 **************************************************************************************/

/**************************************************************************************
 * Pre-processor Definitions
 **************************************************************************************/

/* Configuration **********************************************************************/

/* Define the following to enable register-level debug output */

#undef CONFIG_LCD_REGDEBUG

/* Verbose debug must also be enabled */

#ifndef CONFIG_DEBUG
#  undef CONFIG_DEBUG_VERBOSE
#  undef CONFIG_DEBUG_LED
#endif

#ifndef CONFIG_DEBUG_VERBOSE
#  undef CONFIG_LCD_REGDEBUG
#endif

/* Debug ******************************************************************************/

#ifdef CONFIG_LCD_REGDEBUG
#  define regdbg(format, arg...)  vdbg(format, ##arg)
#else
#  define regdbg(x...)
#endif

#ifdef CONFIG_DEBUG_LCD
#  define lcddbg(format, arg...)  dbg(format, ##arg)
#  define lcdvdbg(format, arg...) vdbg(format, ##arg)
#else
#  define lcddbg(x...)
#  define lcdvdbg(x...)
#endif

/**************************************************************************************
 * Private Type Definition
 **************************************************************************************/

/**************************************************************************************
 * Private Function Prototypes
 **************************************************************************************/

/**************************************************************************************
 * Private Data
 **************************************************************************************/

/**************************************************************************************
 * Private Functions
 **************************************************************************************/

/**************************************************************************************
 * Public Functions
 **************************************************************************************/

/**************************************************************************************
 * Name:  up_lcdinitialize
 *
 * Description:
 *   Initialize the LCD video hardware.  The initial state of the LCD is fully
 *   initialized, display memory cleared, and the LCD ready to use, but with the power
 *   setting at 0 (full off).
 *
 **************************************************************************************/

int up_lcdinitialize(void)
{
  /* Not implemented
   *
   * If you want to implement LCD support, here are some references:
   *
   * 1. Atmel Sample Code (ASF).  There is no example for the SAM4E-EK, but there is for
   *    the SAM4S-EK.  The LCD and its processor connectivity appear to be equivalent
   *    to the SAM4E-EK so this sample code should be a good place to begin.  NOTE that
   *    the clock frequencies may be different and pin usage may be different.  So it
   *    may be necessary to adjust the SAM configuration to use this example.
   * 2. There is an example of an LCD driver for the SAM3U at configs/sam4u-ek/src/up_lcd.c.
   *    That LCD driver is for an LCD with a different LCD controller but should provide
   *    the NuttX SAM framework for an LCD driver.
   * 3. There are other LCD drivers for different MCUs that do support the ILI9325
   *    LCD.  Look at configs/shenzhou/src/up_ili93xx.c, configs/stm3220g-eval/src/up_lcd.c,
   *    and configs/stm3240g-eval/src/up_lcd.c.  I believe that the Shenzhou driver is
   *    the most recent.
   */

  return -ENOSYS;
}

/**************************************************************************************
 * Name:  up_lcdgetdev
 *
 * Description:
 *   Return a a reference to the LCD object for the specified LCD.  This allows
 *   support for multiple LCD devices.
 *
 **************************************************************************************/

FAR struct lcd_dev_s *up_lcdgetdev(int lcddev)
{
  /* Not implemented */

  return NULL;
}

/**************************************************************************************
 * Name:  up_lcduninitialize
 *
 * Description:
 *   Unitialize the framebuffer support.
 *
 **************************************************************************************/

void up_lcduninitialize(void)
{
  /* Not implemented */
}
