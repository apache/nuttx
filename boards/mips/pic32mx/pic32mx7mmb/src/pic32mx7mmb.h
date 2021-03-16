/****************************************************************************
 * boards/mips/pic32mx/pic32mx7mmb/src/pic32mx7mmb.h
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

#ifndef __BOARDS_MIPS_PIC32MX_PIC32MX7MMB_SRC_PIC32MX7MMB_H
#define __BOARDS_MIPS_PIC32MX_PIC32MX7MMB_SRC_PIC32MX7MMB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* The Mikroelektronika PIC32MX7 MMB has 3 user LEDs labeled LED0-2 in the
 * schematics:
 *
 * ---  ----- --------------------------------------------------------------
 * PIN  Board Notes
 * ---  ----- --------------------------------------------------------------
 * RA0  LED0  Pulled-up, low value illuminates
 * RA1  LED1  Pulled-up, low value illuminates
 * RD9  LED2  Pulled-up, low value illuminates
 * RA9  LED4  Not available for general use*, indicates MMC/SD activity
 * ---  LED5  Not controllable by software, indicates power-on
 *
 * * RA9 is also the SD chip select.  It will illuminate whenever the SD card
 *   is selected.  If SD is not used, then LED4 could also be used as a user-
 *   controlled LED.
 */

/* The Mikroelektronika PIC32MX7 MMB has a joystick:
 *
 * ------ -------- ------------------------- --------------------------------
 *  GPIO   SIGNAL  BOARD CONNECTION           NOTES
 * ------ -------- ------------------------- --------------------------------
 *   RB0   JOY-A   Joystick A, HDR1 pin 24   Pulled up, low value when closed
 *   RB2   JOY-C   Joystick C, HDR1 pin 22   Pulled up, low value when closed
 *   RB1   JOY-B   Joystick B, HDR1 pin 23   Pulled up, low value when closed
 *   RB3   JOY-D   Joystick D, HDR1 pin 21   Pulled up, low value when closed
 *   RA10  JOY-CP  Joystick CP, HDR1 pin 25  Pulled up, low value when closed
 */

/* LCD
 *
 * ------ -------- ------------------------- --------------------------------
 *  GPIO   SIGNAL  BOARD CONNECTION           NOTES
 * ------ -------- ------------------------- --------------------------------
 *   RD2   LCD-BLED Backlight Light          Low value turns off
 */

/* SPI1 and SD Card
 *
 * ------ -------- ------------------------- --------------------------------
 *  GPIO   SIGNAL  BOARD CONNECTION           NOTES
 * ------ -------- ------------------------- --------------------------------
 *   RC4   SPI1    SD card slot              SPI1 data IN
 *   RD0   SPO1    SD card slot              SPI1 data OUT
 *   RD10  SCK1    SD card slot              SD card, SPI clock
 *
 *   RA9   SD_CS#  SD card slot              SD card, SPI chip select
 *                                           (active low)
 *   RG6   SD_WP   SD card slot              SD card, write protect
 *   RG7   SD_CD#  SD card slot              SD card, card detect (not)
 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: pic32mx_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the Mikroelektronika
 *   PIC32MX7 MMB board.
 *
 ****************************************************************************/

#if defined(CONFIG_PIC32MX_SPI1) || defined(CONFIG_PIC32MX_SPI2) || \
    defined(CONFIG_PIC32MX_SPI3) || defined(CONFIG_PIC32MX_SPI4)
void weak_function pic32mx_spidev_initialize(void);
#endif

/****************************************************************************
 * Name: pic32mx_led_initialize
 *
 * Description:
 *   Configure on-board LEDs if LED support has been selected.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void pic32mx_led_initialize(void);
#endif

/****************************************************************************
 * Name: pic32mx_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y && CONFIG_LIB_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int pic32mx_bringup(void);

/****************************************************************************
 * Name: pic32mx_lcdinitialize
 *
 * Description:
 *   Initialize the LCD.  This function should be called early in the boot
 *   sequence -- even if the LCD is not enabled.  In that case we should
 *   at a minimum at least disable the LCD backlight.
 *
 ****************************************************************************/

void pic32mx_lcdinitialize(void);

/****************************************************************************
 * Name: pic32mx_tsc_setup
 *
 * Description:
 *   This function is called by board-bringup logic to configure the
 *   touchscreen device.  This function will register the driver as
 *   /dev/inputN where N is the minor device number.
 *
 * Input Parameters:
 *   minor   - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_INPUT
int pic32mx_tsc_setup(int minor);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_MIPS_PIC32MX_PIC32MX7MMB_SRC_PIC32MX7MMB_H */
