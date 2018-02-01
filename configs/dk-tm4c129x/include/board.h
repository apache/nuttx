/************************************************************************************
 * configs/dk-tm4c129x/include/board.h
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

#ifndef __CONFIGS_DK_TM4C129X_INCLUDE_BOARD_H
#define __CONFIGS_DK_TM4C129X_INCLUDE_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#ifndef __ASSEMBLY__
#  include <stdbool.h>
#endif

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/

/* Crystals on-board the DK-TM4C129X include:
 *
 * 1. 25.0MHz (Y2) is connected to OSC0/1 pins and is used as the run mode input to
 *    the PLL.
 * 2. 32.768kHz (Y3) connected to XOSC0/1 and clocks the hibernation module.
 */

#define SYSCON_RCC_XTAL      SYSCON_RCC_XTAL16000KHZ /* On-board crystal is 25 MHz */
#define XTAL_FREQUENCY       25000000

/* Frequencies of other clock sources */

#define PIOSC_FREQUENCY      16000000 /* Precision internal oscillator */
#define RTCOSC_FREQUENCY     32768    /* Hibernation Module RTC Oscillator */
#define LFIOSC_FREQUENCY     33000    /* Low frequency internal oscillator */

/* The PLL generates Fvco according to the following formulae.  The input clock to
 * the PLL may be either the external crystal (Fxtal) or PIOSC (Fpiosc).  This
 * logic supports only the external crystal as the PLL source clock.
 *
 *   Fin  = Fxtal / (Q + 1 )(N + 1) -OR- Fpiosc / (Q + 1)(N + 1)
 *   Mdiv = Mint + (MFrac / 1024)
 *   Fvco = Fin * Mdiv
 *
 * Where the register fields Q and N actually hold (Q-1) and (N-1).   The following
 * setup then generates Fvco = 480MHz:
 *
 *   Fin  = 25 MHz / 1 / 5 = 5 MHz
 *   Mdiv = 96
 *   Fvco = 480
 */

#define BOARD_PLL_MINT       96        /* Integer part of PLL M value */
#define BOARD_PLL_MFRAC      0         /* Fractional part of PLL M value */
#define BOARD_PLL_N          5         /* PLL N value */
#define BOARD_PLL_Q          1         /* PLL Q value */

#define BOARD_FVCO_FREQUENCY 480000000 /* Resulting Fvco */

/* When the PLL is active, the system clock frequency (SysClk) is calculated using
 * the following equation:
 *
 *   SysClk = Fvco/ (sysdiv + 1)
 *
 * The following setup generates Sysclk = 120MHz:
 */

#define BOARD_PLL_SYSDIV     4         /* Sysclk = Fvco / 4 = 120MHz */
#define SYSCLK_FREQUENCY     120000000 /* Resulting SysClk frequency */

/* Alternate Clock (ALTCLK)
 *
 * The ALTCLK provides a clock source of numerous frequencies to the general-purpose
 * timer, SSI, and UART modules.  The default source for the ALTCLK is the Precision
 * Internal Oscillator (PIOSC).  The Hibernation Real-time Clock (RTCOSC) and Low
 * Frequency Internal Oscillator (LFIOSC) are alternatives.  If the RTCOSC Output is
 * selected, the clock source must also be enabled in the Hibernation module.
 */

#define BOARD_ALTCLKCFG      SYSCON_ALTCLKCFG_ALTCLK_PIOSC
#define ALTCLK_FREQUENCY     PIOSC_FREQUENCY

/* LED definitions ******************************************************************/
/* The DK-TM4C129X has a single RGB LED.  There is only one visible LED which
 * will vary in color.  But, from the standpoint of the firmware, this appears as
 * three LEDs:
 *
 *   --- ------------ -----------------
 *   Pin Pin Function Jumper
 *   --- ------------ -----------------
 *   PN5 Red LED      J36 pins 1 and 2
 *   PQ4 Blue LED     J36 pins 3 and 4
 *   PQ7 Green LED    J36 pins 5 and 6
 *   --- ------------ -----------------
 */

/* LED index values for use with board_userled() */

#define BOARD_LED_R               0
#define BOARD_LED_G               1
#define BOARD_LED_B               2
#define BOARD_NLEDS               3

/* LED bits for use with board_userled_all() */

#define BOARD_LED_R_BIT           (1 << BOARD_LED_R)
#define BOARD_LED_G_BIT           (1 << BOARD_LED_G)
#define BOARD_LED_B_BIT           (1 << BOARD_LED_B)

/* If CONFIG_ARCH_LEDS is defined, then automated support for the DK-TM4C129X LED
 * will be included in the build:
 */
                                /* RED  GREEN BLUE */
#define LED_STARTED       0     /* OFF  OFF   ON   */
#define LED_HEAPALLOCATE  1     /* NC   NC    NC   */
#define LED_IRQSENABLED   1     /* NC   NC    NC   */
#define LED_STACKCREATED  2     /* OFF  ON    OFF  */
#define LED_INIRQ         1     /* NC   NC    NC   */
#define LED_SIGNAL        1     /* NC   NC    NC   */
#define LED_ASSERTION     1     /* NC   NC    NC   */
#define LED_PANIC         3     /* ON   OFF   OFF (flashing 2Hz) */

/* Button definitions ***************************************************************/
/* There are three push buttons on the board.
 *
 *   --- ------------ -----------------
 *   Pin Pin Function Jumper
 *   --- ------------ -----------------
 *   PP1 Select SW4   J37 pins 1 and 2
 *   PN3 Up SW2       J37 pins 3 and 4
 *   PE5 Down SW3     J37 pins 5 and 6
 *   --- ------------ -----------------
 */

#define BUTTON_SW2        0
#define BUTTON_SW3        1
#define BUTTON_SW4        2
#define NUM_BUTTONS       3

#define BUTTON_SW2_BIT    (1 << BUTTON_SW2)
#define BUTTON_SW3_BIT    (1 << BUTTON_SW3)
#define BUTTON_SW4_BIT    (1 << BUTTON_SW4)

/* Pin Multiplexing Disambiguation **************************************************/
/* UARTs
 *
 *   UART0: PA0-1 (fixed configuration)
 *   UART3: PJ0-1 to EM_TX/EM_RX or BOOSTER_PACK2_RX/BOOSTER_PACK2_TX(Depending
 *          on J12/J13)
 *   UART5: PH6-7 to BOOSTER_PACK1_RX/BOOSTER_PACK1_TX
 */

#define GPIO_UART3_RX     GPIO_UART3_RX_2
#define GPIO_UART3_TX     GPIO_UART3_TX_2

#define GPIO_UART5_RX     GPIO_UART5_RX_2
#define GPIO_UART5_TX     GPIO_UART5_TX_2

/* SSI:
 *
 *   SSI0: PA2-5 are used for SSI0 to the second booster pack (fixed configuration)
 *   SSI3: PF0/PF4-5/PH4/PQ0-2 are used for the SPI flash (on-board and SD card).
 *         PH4 selects the SD card and PQ1 selects the on-board SPI flash.
 */

#define GPIO_SSI3_CLK     GPIO_SSI3_CLK_2
#define GPIO_SSI3_FSS     GPIO_SSI3_FSS_2
#define GPIO_SSI3_XDAT0   GPIO_SSI3_XDAT0_2
#define GPIO_SSI3_XDAT1   GPIO_SSI3_XDAT1_1
#define GPIO_SSI3_XDAT2   GPIO_SSI3_XDAT2_1
#define GPIO_SSI3_XDAT3   GPIO_SSI3_XDAT3_1

/* I2C:
 *
 *   I2C3: PG4-5 are provide to the BoostPack 1 interface
 *   I2C7: PA4-5 are provide to the BoostPack 2 interface
 *   I2C6: PB6-7 are used for I2C to the TMP100 and the EM connector.
 *         J18 and J20 must be closed to connect the TMP100.
 *         I2C address is 0x4A
 */

#define GPIO_I2C3_SCL     GPIO_I2C3_SCL_1
#define GPIO_I2C3_SDA     GPIO_I2C3_SDA_1
#define GPIO_I2C7_SCL     GPIO_I2C7_SCL_1
#define GPIO_I2C7_SDA     GPIO_I2C7_SDA_1
#define GPIO_I2C6_SCL     GPIO_I2C6_SCL_2
#define GPIO_I2C6_SDA     GPIO_I2C6_SDA_2

/* USB:
 *
 *   PB0-1/PD6-7/PL6-7 are used for USB (Only PD6-7 are not fixed configuration)
 */

#define GPIO_USB0_EPEN   GPIO_USB0_EPEN_3
#define GPIO_USB0_PFLT   GPIO_USB0_PFLT_2

/* Ethernet LEDs
 *
 *    PF1/PK4/PK6 are used for Ethernet LEDs.
 *      PK4/EN0RXD3/LED0
 *      PK6/EN0TXD2/LED1
 *      PF1/LED2
 */

#  define GPIO_EN0_LED0   GPIO_EN0_LED0_2
#  define GPIO_EN0_LED1   GPIO_EN0_LED1_2
#  define GPIO_EN0_LED2   GPIO_EN0_LED2_1

/* LCD
 *
 *   PF6-7/PJ6/PS4-5/PR0-7 are used for the LCD (fixed configuration).
 */

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Name: tiva_tmp100_initialize
 *
 * Description:
 *   Initialize and register the TMP-100 Temperature Sensor driver.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/temp0"
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ************************************************************************************/

#if defined(CONFIG_I2C) && defined(CONFIG_LM75_I2C) && defined(CONFIG_TIVA_I2C6)
int tiva_tmp100_initialize(FAR const char *devpath);
#endif

#endif /* __ASSEMBLY__ */
#endif  /* __CONFIGS_DK_TM4C129X_INCLUDE_BOARD_H */
