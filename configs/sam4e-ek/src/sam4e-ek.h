/************************************************************************************
 * configs/sam4e-ek/src/sam4e-ek.h
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
 ************************************************************************************/

#ifndef __CONFIGS_SAM4E_EK_SRC_SAM4E_EK_H
#define __CONFIGS_SAM4E_EK_SRC_SAM4E_EK_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>

#include <arch/irq.h>
#include <nuttx/irq.h>

#include "chip/sam_pinmap.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* External Memory Usage ************************************************************/
/* LCD on CS2 */

#define LCD_BASE              SAM_EXTCS2_BASE

/* Touchscreen controller (TSC) */

#define CONFIG_TSC_ADS7843    1   /* ADS7843 present on board */
#define CONFIG_TSC_SPI        0   /* On SPI0 */

/* SAM4E-EK GPIO Pin Definitions ****************************************************/

/* LCD:
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
 */

#define GPIO_LCD_NCS2 (GPIO_PERIPHA | GPIO_CFG_PULLUP | GPIO_PORT_PIOC | GPIO_PIN16)
#define GPIO_LCD_RS   (GPIO_PERIPHB | GPIO_CFG_PULLUP | GPIO_PORT_PIOB | GPIO_PIN8)
#define GPIO_LCD_NWE  (GPIO_PERIPHA | GPIO_CFG_PULLUP | GPIO_PORT_PIOB | GPIO_PIN23)
#define GPIO_LCD_NRD  (GPIO_PERIPHA | GPIO_CFG_PULLUP | GPIO_PORT_PIOB | GPIO_PIN19)

#define GPIO_LCD_D0   (GPIO_PERIPHA | GPIO_CFG_PULLUP | GPIO_PORT_PIOB | GPIO_PIN9)
#define GPIO_LCD_D1   (GPIO_PERIPHA | GPIO_CFG_PULLUP | GPIO_PORT_PIOB | GPIO_PIN10)
#define GPIO_LCD_D2   (GPIO_PERIPHA | GPIO_CFG_PULLUP | GPIO_PORT_PIOB | GPIO_PIN11)
#define GPIO_LCD_D3   (GPIO_PERIPHA | GPIO_CFG_PULLUP | GPIO_PORT_PIOB | GPIO_PIN12)
#define GPIO_LCD_D4   (GPIO_PERIPHA | GPIO_CFG_PULLUP | GPIO_PORT_PIOB | GPIO_PIN13)
#define GPIO_LCD_D5   (GPIO_PERIPHA | GPIO_CFG_PULLUP | GPIO_PORT_PIOB | GPIO_PIN14)
#define GPIO_LCD_D6   (GPIO_PERIPHA | GPIO_CFG_PULLUP | GPIO_PORT_PIOB | GPIO_PIN15)
#define GPIO_LCD_D7   (GPIO_PERIPHA | GPIO_CFG_PULLUP | GPIO_PORT_PIOB | GPIO_PIN16)
#define GPIO_LCD_D8   (GPIO_PERIPHA | GPIO_CFG_PULLUP | GPIO_PORT_PIOB | GPIO_PIN25)
#define GPIO_LCD_D9   (GPIO_PERIPHA | GPIO_CFG_PULLUP | GPIO_PORT_PIOB | GPIO_PIN26)
#define GPIO_LCD_D10  (GPIO_PERIPHA | GPIO_CFG_PULLUP | GPIO_PORT_PIOB | GPIO_PIN27)
#define GPIO_LCD_D11  (GPIO_PERIPHA | GPIO_CFG_PULLUP | GPIO_PORT_PIOB | GPIO_PIN28)
#define GPIO_LCD_D12  (GPIO_PERIPHA | GPIO_CFG_PULLUP | GPIO_PORT_PIOB | GPIO_PIN29)
#define GPIO_LCD_D13  (GPIO_PERIPHA | GPIO_CFG_PULLUP | GPIO_PORT_PIOB | GPIO_PIN30)
#define GPIO_LCD_D14  (GPIO_PERIPHA | GPIO_CFG_PULLUP | GPIO_PORT_PIOB | GPIO_PIN31)
#define GPIO_LCD_D15  (GPIO_PERIPHB | GPIO_CFG_PULLUP | GPIO_PORT_PIOB | GPIO_PIN6)

/* LCD Backlight pin definition. */

#define GPIO_LCD_BKL  (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_CLEAR | \
                       GPIO_PORT_PIOC | GPIO_PIN19)

/* ADS7843 Touchscreen controller (TSC)
 *
 * The LCD module integrates a 4-wire touch screen panel controlled by
 * MN5, ADS7843, which is a slave device on the SAM3U4E SPI bus. The ADS7843 touch
 * ADC auxiliary inputs IN3/IN4 are connected to test points for optional function
 * extension.
 *
 *   ------ -------
 *   GPIO   PIN
 *   ------ -------
 *   PA11   /CS
 *   PA12   DOUT
 *   PA13   DIN
 *   PA14   DCLK
 *   PA16   /PENIRQ
 *   PA17   BUSY
 *   ------ -------
 *
 * The IRQ is active low and pulled up.
 *
 * Pen Interrupt. Open anode output, requires 10kO to 100kO pull-up resistor
 * externally.  There is a 100KO pull-up on the SAM4E-EK board so no additional
 * pull-up should be required.
 *
 * BUSY is high impedance when CS is high (not selected).  When CS is
 * is low, BUSY is active high.  Since the pin is pulled up, it will appear
 * busy if CS is not selected (there is no pull-up onboard).
 */

#define GPIO_TCS_IRQ  (GPIO_INPUT | GPIO_CFG_PULLUP | GPIO_INT_BOTHEDGES | \
                       GPIO_PORT_PIOA | GPIO_PIN16)
#define GPIO_TCS_BUSY (GPIO_INPUT | GPIO_CFG_PULLUP | GPIO_PORT_PIOA | \
                       GPIO_PIN17)

#define SAM_TCS_IRQ   SAM_IRQ_PA16

/* Ethernet MAC.  The PHY interrupt is available on pin PD28 */

#define GPIO_PHY_IRQ  (GPIO_INPUT | GPIO_CFG_PULLUP | GPIO_INT_BOTHEDGES | \
                       GPIO_PORT_PIOD | GPIO_PIN28)
#define SAM_PHY_IRQ   SAM_IRQ_PD28

/* LEDs
 *
 *  D2 PA0  Blue   Pulled high
 *  D3 PD20 Amber  Pulled high
 *  D4 PD21 Green  Pulled high
 */

#define GPIO_D3       (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_PORT_PIOD | \
                       GPIO_OUTPUT_CLEAR | GPIO_PIN20)
#define GPIO_D2       (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_PORT_PIOA | \
                       GPIO_OUTPUT_SET | GPIO_PIN0)
#define GPIO_D4       (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_PORT_PIOD | \
                       GPIO_OUTPUT_SET | GPIO_PIN21)

/* Buttons
 *
 * Four buttons for software inputs:
 *
 *   PA1  BUTTON_SCROLL-UP    Grounded
 *   PA2  BUTTON_SCROLL-DOWN  Grounded
 *   PA19 BUTTON_WAKU         Grounded
 *   PA20 BUTTON_TAMP         Grounded
 */

#define GPIO_SCROLLUP  (GPIO_INPUT | GPIO_CFG_PULLUP | GPIO_CFG_DEGLITCH | \
                        GPIO_INT_BOTHEDGES | GPIO_PORT_PIOA | GPIO_PIN1)
#define GPIO_SCROLLDWN (GPIO_INPUT | GPIO_CFG_PULLUP | GPIO_CFG_DEGLITCH | \
                        GPIO_INT_BOTHEDGES | GPIO_PORT_PIOA | GPIO_PIN2)
#define GPIO_WAKU      (GPIO_INPUT | GPIO_CFG_PULLUP | GPIO_CFG_DEGLITCH | \
                        GPIO_INT_BOTHEDGES | GPIO_PORT_PIOA | GPIO_PIN19)
#define GPIO_TAMP      (GPIO_INPUT | GPIO_CFG_PULLUP | GPIO_CFG_DEGLITCH | \
                        GPIO_INT_BOTHEDGES | GPIO_PORT_PIOA | GPIO_PIN20)

#define IRQ_SCROLLUP   SAM_IRQ_PA1
#define IRQ_SCROLLDWN  SAM_IRQ_PA2
#define IRQ_WAKU       SAM_IRQ_PA19
#define IRQ_TAMP       SAM_IRQ_PA20
#define IRQ_SCROLLUP   SAM_IRQ_PA1
#define IRQ_SCROLLDWN  SAM_IRQ_PA2
#define IRQ_WAKU       SAM_IRQ_PA19
#define IRQ_TAMP       SAM_IRQ_PA20

/* USART1: To avoid any electrical conflict, the RS232 and RS485 transceiver
 * are isolated from the receiving line PA21.
 *
 * - Chose RS485 channel: Close 1-2 pins on JP11 and set PA23 to high level
 * - Chose RS232 channel: Close 2-3 pins on JP11 and set PA23 to low level
 */

#define GPIO_RS232_ENABLE (GPIO_OUTPUT | GPIO_CFG_DEFAULT | \
                           GPIO_OUTPUT_CLEAR | GPIO_PORT_PIOA | GPIO_PIN21)
#define GPIO_RS485_ENABLE (GPIO_OUTPUT | GPIO_CFG_DEFAULT | \
                           GPIO_OUTPUT_SET | GPIO_PORT_PIOA | GPIO_PIN21)

/* SD Card Detect */

#define GPIO_MCI_CD   (GPIO_INPUT | GPIO_CFG_PULLUP | GPIO_PORT_PIOA | GPIO_PIN25)

/* SPI Chip Selects */

/* Chip select pin connected to the touchscreen controller and to the ZigBee module
 * connector.  Notice that the touchscreen chip select is implemented as a GPIO
 * OUTPUT that must be controlled by board-specific.  This is because the ADS7843E
 * driver must be able to sample the device BUSY GPIO input between SPI transfers.
 * However, the AD7843E will tri-state the BUSY input whenever the chip select is
 * de-asserted.  So the only option is to control the chip select manually and hold
 * it low throughout the SPI transfer.
 */

#define GPIO_TSC_NPCS2 (GPIO_OUTPUT | GPIO_CFG_PULLUP | GPIO_OUTPUT_SET | \
                        GPIO_PORT_PIOA | GPIO_PIN11)
#define TSC_CSNUM      0

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public data
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: sam_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the SAM4E-EK board.
 *
 ************************************************************************************/

void weak_function sam_spiinitialize(void);

/************************************************************************************
 * Name: sam_usbinitialize
 *
 * Description:
 *   Called to setup USB-related GPIO pins for the SAM4E-EK board.
 *
 ************************************************************************************/

void weak_function sam_usbinitialize(void);

/************************************************************************************
 * Name: sam_hsmciinit
 *
 * Description:
 *   Initialize HSMCI support
 *
 ************************************************************************************/

#ifdef CONFIG_SAM34_HSMCI
int weak_function sam_hsmciinit(void);
#else
# define sam_hsmciinit()
#endif

/************************************************************************************
 * Name: board_led_initialize
 ************************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void board_led_initialize(void);
#endif

/************************************************************************************
 * Name: sam_cardinserted
 *
 * Description:
 *   Check if a card is inserted into the selected HSMCI slot
 *
 ************************************************************************************/

#ifdef CONFIG_SAM34_HSMCI
bool sam_cardinserted(unsigned char slot);
#else
#  define sam_cardinserted(slot) (false)
#endif

/************************************************************************************
 * Name: sam_writeprotected
 *
 * Description:
 *   Check if a card is inserted into the selected HSMCI slot
 *
 ************************************************************************************/

#ifdef CONFIG_SAM34_HSMCI
bool sam_writeprotected(unsigned char slot);
#else
#  define sam_writeprotected(slot) (false)
#endif

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_SAM4E_EK_SRC_SAM4E_EK_H */
