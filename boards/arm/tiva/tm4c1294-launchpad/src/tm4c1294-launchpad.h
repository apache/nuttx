/****************************************************************************
 * boards/arm/tiva/tm4c1294-launchpad/src/tm4c1294-launchpad.h
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

#ifndef __BOARDS_ARM_TIVA_TM4C1294_LAUNCHPAD_TM4C1294_LAUNCHPAD_H
#define __BOARDS_ARM_TIVA_TM4C1294_LAUNCHPAD_TM4C1294_LAUNCHPAD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include "chip.h"
#include "tiva_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#define HAVE_HCIUART    1

#if !defined(CONFIG_TIVA_HCIUART) || !defined(CONFIG_BLUETOOTH_UART)
#  undef HAVE_HCIUART
#elif defined(CONFIG_TIVA_UART0_HCIUART) && !defined(TM4C1294_LAUNCHPAD_JUMPERS_CAN)
#  define HCIUART_SERDEV HCIUART0
#elif defined(CONFIG_TIVA_UART1_HCIUART)
#  define HCIUART_SERDEV HCIUART1
#elif defined(CONFIG_TIVA_UART2_HCIUART)
#  define HCIUART_SERDEV HCIUART2
#elif defined(CONFIG_TIVA_UART3_HCIUART)
#  define HCIUART_SERDEV HCIUART3
#elif defined(CONFIG_TIVA_UART4_HCIUART)
#  define HCIUART_SERDEV HCIUART4
#elif defined(CONFIG_TIVA_UART5_HCIUART)
#  define HCIUART_SERDEV HCIUART5
#elif defined(CONFIG_TIVA_UART6_HCIUART)
#  define HCIUART_SERDEV HCIUART6
#elif defined(CONFIG_TIVA_UART7_HCIUART)
#  define HCIUART_SERDEV HCIUART7
#else
#  error No HCI UART specifified
#endif

/* How many SSI modules does this chip support? */

#if TIVA_NSSI < 1
#  undef CONFIG_TIVA_SSI0
#  undef CONFIG_TIVA_SSI0
#elif TIVA_NSSI < 2
#  undef CONFIG_TIVA_SSI0
#endif

/* Do we need to register I2C drivers on behalf of the I2C tool? */

#define HAVE_I2CTOOL 1
#if !defined(CONFIG_SYSTEM_I2CTOOL) || !defined(CONFIG_I2C_DRIVER)
#  undef HAVE_I2CTOOL
#endif

/* Define the procfs mounting point */

#define TIVA_PROCFS_MOUNTPOINT "/proc"

/* LED definitions **********************************************************/

/* The EK-TM4C1294XL has a four green LEDs.
 *
 *   --- ------------
 *   Pin Pin Function
 *   --- ------------
 *   PN1 Green LED D1
 *   PN0 Green LED D2
 *   PF4 Green LED D3
 *   PF0 Green LED D4
 *   --- ------------
 *
 * A high output illuminates the LED.
 */

#define GPIO_LED_D1   (GPIO_FUNC_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORTN | GPIO_PIN_1)
#define GPIO_LED_D2   (GPIO_FUNC_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORTN | GPIO_PIN_0)
#define GPIO_LED_D3   (GPIO_FUNC_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORTF | GPIO_PIN_4)
#define GPIO_LED_D4   (GPIO_FUNC_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORTF | GPIO_PIN_0)

/* Check if we have the LED driver */

#define HAVE_USERLED_DRIVER   1
#if !defined(CONFIG_USERLED) || !defined(CONFIG_USERLED_LOWER)
#  undef HAVE_USERLED_DRIVER
#endif

/* Button definitions *******************************************************/

/* There are four push buttons on the board.
 * Two of them are user controllable.
 * The others are RESET and WAKE
 *
 *   --- ------------
 *   Pin Pin Function
 *   --- ------------
 *   PJ0 USR_SW1
 *   PJ1 USR_SW2
 *   --- ------------
 */

#ifdef CONFIG_ARCH_IRQBUTTONS
#  define GPIO_SW1    (GPIO_FUNC_INTERRUPT | GPIO_INT_BOTHEDGES | \
                       GPIO_STRENGTH_2MA | GPIO_PADTYPE_STDWPU | \
                       GPIO_PORTJ | GPIO_PIN_0)
#  define GPIO_SW2    (GPIO_FUNC_INTERRUPT | GPIO_INT_BOTHEDGES | \
                       GPIO_STRENGTH_2MA | GPIO_PADTYPE_STDWPU | \
                       GPIO_PORTJ | GPIO_PIN_1)
#else
#  define GPIO_SW1    (GPIO_FUNC_INPUT | GPIO_PORTJ | GPIO_PIN_0)
#  define GPIO_SW2    (GPIO_FUNC_INPUT | GPIO_PORTJ | GPIO_PIN_1)
#endif

/* SPI Chip selects *********************************************************/

/*   SSI0: PA3 is used for SSI0 chip select to the second booster pack
 *         (No pull- ups)
 *   SSI3: PH4 selects the SD card and PQ1 selects the on-board SPI flash.
 *          Both pulled up on board.
 */

#define GPIO_BSTR2_CS (GPIO_FUNC_OUTPUT | GPIO_PADTYPE_STDWPU | GPIO_STRENGTH_4MA | \
                       GPIO_VALUE_ONE | GPIO_PORTA | GPIO_PIN_3)
#define GPIO_FLASH_CS (GPIO_FUNC_OUTPUT | GPIO_PADTYPE_STD | GPIO_STRENGTH_4MA | \
                       GPIO_VALUE_ONE | GPIO_PORTH | GPIO_PIN_4)
#define GPIO_SD_CS    (GPIO_FUNC_OUTPUT | GPIO_PADTYPE_STD | GPIO_STRENGTH_4MA | \
                       GPIO_VALUE_ONE | GPIO_PORTH | GPIO_PIN_4)

/* I2C **********************************************************************/

/*   I2C3: PG4-5 are provide to the BoostPack 1 interface
 *   I2C7: PA4-5 are provide to the BoostPack 2 interface
 *   I2C6: PB6-7 are used for I2C to the connector.
 */

/* Speaker outputs **********************************************************/

/* PB2/PD4 are used for the speaker output */

/* Touchscreen **************************************************************/

/* PE7/PP7/PT2-3 are used for the touch screen */

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Name: tm4c_ssidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the EK-TM4C1294XL.
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
 * Name: tiva_timer_configure
 *
 * Description:
 *   Configure the timer driver
 *
 ****************************************************************************/

#ifdef CONFIG_TM4C1294_LAUNCHPAD_TIMER
int tiva_timer_configure(void);
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
 * Name: hciuart_dev_initialize
 *
 * Description:
 *   This function is called by board initialization logic to configure the
 *   Bluetooth HCI UART driver
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

#ifdef HAVE_HCIUART
int hciuart_dev_initialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_TIVA_TM4C1294_LAUNCHPAD_TM4C1294_LAUNCHPAD_H */
