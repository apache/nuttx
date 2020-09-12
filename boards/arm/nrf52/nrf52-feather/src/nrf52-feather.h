/****************************************************************************
 * boards/arm/nrf52/nrf52-feather/src/nrf52-feather.h
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

#ifndef __BOARDS_ARM_NRF52_NRF52_FEATHER_SRC_NRF52_FEATHER_H
#define __BOARDS_ARM_NRF52_NRF52_FEATHER_SRC_NRF52_FEATHER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include "nrf52_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LED definitions **********************************************************/

/* Definitions to configure LED GPIO as outputs */

#define GPIO_LED1  (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT0 | GPIO_PIN(17))
#define GPIO_LED2  (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT0 | GPIO_PIN(19))

/* Button definitions *******************************************************/

/* No buttons on board */

/* Keyboard FeatherWing definitions *****************************************/

#ifdef CONFIG_NRF52_FEATHER_KB_FEATHERWING

#define ILI9341_DISPLAY_CS \
    (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT0 | GPIO_PIN(31))
#define ILI9341_DISPLAY_DC \
    (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORT0 | GPIO_PIN(11))

#define KB_FW_CARD_CS \
    (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT0 | GPIO_PIN(27))
#define KB_FW_STMP_CS \
    (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT0 | GPIO_PIN(30))

#define KB_FW_CARD_DET_PIN      0
#define KB_FW_KBD_INT_PIN       2
#define KB_FW_LCD_BL_PIN        3

#define KB_FW_CARD_DET (STMPE811_GPIO_INPUT | \
                        STMPE811_GPIO_FALLING | STMPE811_GPIO_RISING | \
                        (KB_FW_CARD_DET_PIN << STMPE811_GPIO_PIN_SHIFT))
#define KB_FW_KBD_INT  (STMPE811_GPIO_INPUT | STMPE811_GPIO_FALLING | \
                        (KB_FW_KBD_INT_PIN << STMPE811_GPIO_PIN_SHIFT))
#define KB_FW_LCD_BL   (STMPE811_GPIO_OUTPUT | \
                        (KB_FW_LCD_BL_PIN << STMPE811_GPIO_PIN_SHIFT))

#define KB_FW_SPI_DEV            0  /* This is the primary shared bus */
#define KB_FW_NEOPIXEL_SPI_DEV   2  /* Neopixel hogs the bus */
#define KB_FW_I2C_DEV            1  /* This is the primary shared bus */

#define ILI9341_DISPLAY_SPI_DEV  KB_FW_SPI_DEV
#define KB_FW_SDCARD_SPI_DEV     KB_FW_SPI_DEV
#define KB_FW_STMP_SPI_DEV       KB_FW_SPI_DEV
#define KB_FW_KEYBOARD_I2C_DEV   KB_FW_I2C_DEV

#endif /* CONFIG_NRF52_FEATHER_KB_FEATHERWING */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_LIB_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int nrf52_bringup(void);

/****************************************************************************
 * Name: nrf52_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the
 *   nrf52840-dk board.
 *
 ****************************************************************************/

#ifdef CONFIG_NRF52_SPI_MASTER
void nrf52_spidev_initialize(void);
#endif

#ifdef CONFIG_NRF52_FEATHER_KB_FEATHERWING

/****************************************************************************
 * Name: fw_stmpe811_setup
 *
 * Description:
 *   This function is called by board-bringup logic to configure the
 *   touchscreen device.  This function will register the driver as
 *   /dev/inputN where N is the minor device number.
 *
 * Input Parameters:
 *   minor - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int fw_stmpe811_setup(int minor);

/****************************************************************************
 * Name: kb_int
 *
 * Description:
 *   This function is called a key press is detected on the keyboard
 *
 * Input Parameters:
 *   pin - Interrupt GPIO pin (not used)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void kb_int(int pin);

/****************************************************************************
 * Name: kb_feather_carddet
 *
 * Description:
 *   This function is called when a change in the sd card is detected
 *
 * Input Parameters:
 *   pin - Interrupt GPIO pin (not used)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void kb_feather_carddet(int pin);

/****************************************************************************
 * Name: kb_card_state
 *
 * Description:
 *   This function returns the state of the sdcard
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   True if card is inserted
 *
 ****************************************************************************/

bool kb_card_state(void);

/****************************************************************************
 * Name: kb_backlightctl
 *
 * Description:
 *   This function is called to control the keyboard backlight attached
 *   to stmpe811
 *
 * Input Parameters:
 *   state - true or false to drive backlight
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void kb_backlightctl(bool state);

/****************************************************************************
 * Name: q10kbd_setup
 *
 * Description:
 *   This function is called by board-bringup logic to configure the
 *   Q10 BlackBerry Keyboard
 *
 * Input Parameters:
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_KB_FEATHERWING_KEYBOARD
int q10kbd_setup(int bus);
#endif
#endif

/****************************************************************************
 * Name: nrf52_i2c_register
 *
 * Description:
 *   Register one I2C drivers for the I2C tool.
 *
 ****************************************************************************/

#if defined(CONFIG_I2C) && defined(CONFIG_I2C_DRIVER)
int nrf52_i2c_register(int bus);
#endif

/****************************************************************************
 * Name: nrf52_i2ctool
 *
 * Description:
 *   Register I2C drivers for the I2C tool.
 *
 ****************************************************************************/

#ifdef CONFIG_SYSTEM_I2CTOOL
int nrf52_i2ctool(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_NRF52_NRF52_FEATHER_SRC_NRF52_FEATHER_H */
