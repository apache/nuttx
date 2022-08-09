/****************************************************************************
 * boards/arm/tiva/tm4c123g-launchpad/src/tm4c123g-launchpad.h
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

#ifndef __BOARDS_ARM_TIVA_TM4C123G_LAUNCHPAD_TM4C123G_LAUNCHPAD_H
#define __BOARDS_ARM_TIVA_TM4C123G_LAUNCHPAD_TM4C123G_LAUNCHPAD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <nuttx/irq.h>

#include "chip.h"
#include "tiva_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

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
 * The Serial EEPROM was mounted on an external adapter board and connected
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

/* Can't support AT25 features if mount points are disabled or if we were not
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

/* TM4C123G LaunchPad *******************************************************/

/* The TM4C123G LaunchPad has a single RGB LED.
 * There is only one visible LED which will vary in color.
 * But, from the standpoint of the firmware, this appears as three LEDs:
 *
 *   BOARD_LED_R    -- Connected to PF1
 *   BOARD_LED_G    -- Connected to PF3
 *   BOARD_LED_B    -- Connected to PF2
 *
 * If CONFIG_ARCH_LEDS is defined, then automated support for the LaunchPad
 * LEDs will be included in the build:
 *
 * OFF:
 * - OFF means that the OS is still initializing. Initialization is very fast
 *   so if you see this at all, it probably means that the system is hanging
 *   up somewhere in the initialization phases.
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
 *   briefly while the assertion is handled.
 *   You will probably never see this.
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
 *   BOARD_SW1    -- Connected to PF0
 *   BOARD_SW2    -- Connected to PF4
 */

#define GPIO_SW1     (GPIO_FUNC_INTERRUPT | GPIO_INT_BOTHEDGES | \
                      GPIO_STRENGTH_2MA | GPIO_PADTYPE_STDWPU | \
                      GPIO_PORTF | GPIO_PIN_4)
#define GPIO_SW2     (GPIO_FUNC_INTERRUPT | GPIO_INT_BOTHEDGES | \
                      GPIO_STRENGTH_2MA | GPIO_PADTYPE_STDWPU | \
                      GPIO_PORTF | GPIO_PIN_0)

#define SDCCS_GPIO (GPIO_FUNC_OUTPUT | GPIO_PADTYPE_STDWPU | GPIO_STRENGTH_4MA | \
                    GPIO_VALUE_ONE | GPIO_PORTG | 1)

/* MCP2551: Did not test the CS settings. GPIO_STRENGTH could be different. */

#ifdef CONFIG_CAN_MCP2515
#  define GPIO_MCP2515_IRQ (GPIO_FUNC_INTERRUPT | GPIO_INT_FALLINGEDGE | \
                            GPIO_PORTB | GPIO_PIN_0)
#  define GPIO_SSI2_CLK_1  (GPIO_FUNC_PFIO     | GPIO_ALT_2  | GPIO_PORTB | \
                            GPIO_PIN_4)
#  define GPIO_SSI2_RX_1   (GPIO_FUNC_PFIO     | GPIO_ALT_2  | GPIO_PORTB | \
                            GPIO_PIN_6)
#  define GPIO_SSI2_TX_1   (GPIO_FUNC_PFIO     | GPIO_ALT_2  | GPIO_PORTB | \
                            GPIO_PIN_7)
#  define GPIO_MCP2515_CS  (GPIO_FUNC_OUTPUT | GPIO_PADTYPE_STDWPU | \
                            GPIO_STRENGTH_4MA | GPIO_VALUE_ONE | GPIO_PORTB | \
                            GPIO_PIN_5)
#endif

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Name: tm4c_ssidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the TM4C123G
 *   LaunchPad.
 *
 ****************************************************************************/

void weak_function tm4c_ssidev_initialize(void);

/****************************************************************************
 * Name: tm4c_led_initialize
 *
 * Description:
 *   Called to initialize the on-board LEDs.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void tm4c_led_initialize(void);
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
 * Name: tm4c_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC driver.
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_ADC
int tm4c_adc_setup(void);
#endif

/****************************************************************************
 * Name: tm4c_can_setup
 *
 * Description:
 *   Initialize CAN modules and register the CAN driver.
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_CAN
int tm4c_can_setup(void);
#endif

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
 * Name: tiva_timer_configure
 *
 * Description:
 *   Configure the timer driver
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_TIMER
int tiva_timer_configure(void);
#endif

/****************************************************************************
 * Name: tiva_mcp2515initialize
 *
 * Description:
 *   Initialize and register the MCP2515 CAN driver.
 *
 ****************************************************************************/

#ifdef CONFIG_CAN_MCP2515
int tiva_mcp2515initialize(const char *devpath);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_TIVA_TM4C123G_LAUNCHPAD_TM4C123G_LAUNCHPAD_H */
