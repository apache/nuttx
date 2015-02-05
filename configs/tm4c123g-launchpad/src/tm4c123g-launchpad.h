/************************************************************************************
 * configs/tm4c123g-launchpad/src/tm4c123g-launchpad.h
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

#ifndef __CONFIGS_TM4C123G_LAUNCHPAD_TM4C123G_LAUNCHPAD_H
#define __CONFIGS_TM4C123G_LAUNCHPAD_TM4C123G_LAUNCHPAD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include "chip.h"
#include "tiva_gpio.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/

#define HAVE_AT24 1

/* How many SSI modules does this chip support? */

#if TIVA_NSSI < 1
#  undef CONFIG_TIVA_SSI0
#  undef CONFIG_TIVA_SSI1
#elif TIVA_NSSI < 2
#  undef CONFIG_TIVA_SSI1
#endif

/* AT24 Serial EEPROM
 *
 * A AT24C512 Serial EEPPROM was used for tested I2C.  There are no I2C
 * devices on-board the Launchpad, but an external serial EEPROM module
 * module was used.
 *
 * The Serial EEPROM was mounted on an external adaptor board and connected
 * to the LaunchPad thusly:
 *
 *   - VCC -- VCC
 *   - GND -- GND
 *   - PB2 -- SCL
 *   - PB3 -- SDA
 */

#define AT24_BUS   0
#define AT24_MINOR 0

#if !defined(CONFIG_MTD_AT24XX) || !defined(CONFIG_TIVA_I2C0)
#  undef HAVE_AT24
#endif

/* Can't support AT25 features if mountpoints are disabled or if we were not
 * asked to mount the AT25 part
 */

#if defined(CONFIG_DISABLE_MOUNTPOINT) || \
   !defined(CONFIG_TM4C123G_LAUNCHPAD_AT24_BLOCKMOUNT)
#  undef HAVE_AT24
#endif

/* If we are going to mount the AT25, then they user must also have told
 * us what to do with it by setting one of these.
 */

#ifndef CONFIG_FS_NXFFS
#  undef CONFIG_TM4C123G_LAUNCHPAD_AT24_NXFFS
#endif

#if !defined(CONFIG_TM4C123G_LAUNCHPAD_AT24_FTL) && \
    !defined(CONFIG_TM4C123G_LAUNCHPAD_AT24_NXFFS)
#  undef HAVE_AT24
#endif

#if defined(CONFIG_TM4C123G_LAUNCHPAD_AT24_FTL) && \
   defined(CONFIG_TM4C123G_LAUNCHPAD_AT24_NXFFS)
#  warning Both CONFIG_TM4C123G_LAUNCHPAD_AT24_FTL and CONFIG_TM4C123G_LAUNCHPAD_AT24_NXFFS are set
#  warning Ignoring CONFIG_TM4C123G_LAUNCHPAD_AT24_NXFFS
#  undef CONFIG_TM4C123G_LAUNCHPAD_AT24_NXFFS
#endif

/* TM4C123G LaunchPad ***************************************************************/
/* The TM4C123G LaunchPad has a single RGB LED.  There is only one visible LED which
 * will vary in color.  But, from the standpoint of the firmware, this appears as
 * three LEDs:
 *
 *   BOARD_LED_R    -- Connected to PF1
 *   BOARD_LED_G    -- Connected to PF3
 *   BOARD_LED_B    -- Connected to PF2
 *
 * If CONFIG_ARCH_LEDS is defined, then automated support for the LaunchPad LEDs
 * will be included in the build:
 *
 * OFF:
 * - OFF means that the OS is still initializing. Initialization is very fast so
 *   if you see this at all, it probably means that the system is hanging up
 *   somewhere in the initialization phases.
 *
 * GREEN or GREEN-ish
 * - This means that the OS completed initialization.
 *
 * Bluish:
 * - Whenever and interrupt or signal handler is entered, the BLUE LED is
 *   illuminated and extinguished when the interrupt or signal handler exits.
 *   This will add a BLUE-ish tinge to the LED.
 *
 * Redish:
 * - If a recovered assertion occurs, the RED component will be illuminated
 *   briefly while the assertion is handled.  You will probably never see this.
 *
 * Flashing RED:
 * - In the event of a fatal crash, the BLUE and GREEN components will be
 *   extinguished and the RED component will FLASH at a 2Hz rate.
 *
 *                          RED  GREEN BLUE
 * LED_STARTED       0      OFF  OFF   OFF
 * LED_HEAPALLOCATE  0      OFF  OFF   OFF
 * LED_IRQSENABLED   0      OFF  OFF   OFF
 * LED_STACKCREATED  1      OFF  ON    OFF
 * LED_INIRQ         2      NC   NC    ON  (momentary)
 * LED_SIGNAL        2      NC   NC    ON  (momentary)
 * LED_ASSERTION     3      ON   NC    NC  (momentary)
 * LED_PANIC         3      ON   OFF   OFF (flashing 2Hz)
 */

#define GPIO_LED_R   (GPIO_FUNC_OUTPUT | GPIO_VALUE_ONE | GPIO_PORTF | GPIO_PIN_1)
#define GPIO_LED_G   (GPIO_FUNC_OUTPUT | GPIO_VALUE_ONE | GPIO_PORTF | GPIO_PIN_3)
#define GPIO_LED_B   (GPIO_FUNC_OUTPUT | GPIO_VALUE_ONE | GPIO_PORTF | GPIO_PIN_2)

/* The TM4C123G LaunchPad has a two buttons:
 *
 *   BOARD_SW1    -- Connected to PF4
 *   BOARD_SW2    -- Connected to PF0
 */

#define GPIO_SW1     (GPIO_FUNC_INTERRUPT | GPIO_INT_BOTHEDGES | \
                      GPIO_STRENGTH_2MA | GPIO_PADTYPE_STDWPU | GPIO_PORTF | \
                      GPIO_PIN_4)
#define GPIO_SW2     (GPIO_FUNC_INTERRUPT | GPIO_INT_BOTHEDGES | \
                      GPIO_STRENGTH_2MA | GPIO_PADTYPE_STDWPU | GPIO_PORTF | \
                      GPIO_PIN_0)

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Name: tm4c_ssiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the TM4C123G LaunchPad.
 *
 ************************************************************************************/

void weak_function tm4c_ssiinitialize(void);

/****************************************************************************
 * Name: tm4c_ledinit
 *
 * Description:
 *   Called to initialize the on-board LEDs.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void tm4c_ledinit(void);
#endif

/****************************************************************************
 * Name: tm4c_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

int tm4c_bringup(void);

/****************************************************************************
 * Name: tm4c_at24_automount
 *
 * Description:
 *   Initialize and configure the AT24 serial EEPROM
 *
 ****************************************************************************/

#ifdef HAVE_AT24
int tm4c_at24_automount(int minor);
#endif

/****************************************************************************
 * Name: tiva_timer_initialize
 *
 * Description:
 *   Configure the timer driver
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_TIMER
int tiva_timer_initialize(void);
#endif

/************************************************************************************
 * Name: board_adc_initialize
 *
 * Description:
 *   Initialize and register the ADC driver
 *
 ************************************************************************************/

#ifdef CONFIG_TIVA_ADC
int board_adc_initialize(void);
#endif

/************************************************************************************
 * Name: adc_devinit
 *
 * Description:
 *   All Tiva architectures must provide the following interface to work with
 *   examples/adc.
 *
 ************************************************************************************/

#if defined(CONFIG_TIVA_ADC) && defined(CONFIG_EXAMPLES_ADC)
int adc_devinit(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_TM4C123G_LAUNCHPAD_TM4C123G_LAUNCHPAD_H */

