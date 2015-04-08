/****************************************************************************
 * configs/sure-pic32mx/src/sure-pic32mx.h
 *
 *   Copyright (C) 2011, 2013 Gregory Nutt. All rights reserved.
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

#ifndef __CONFIGS_SURE_PIC32MX_SRC_SURE_PIC32MXL_H
#define __CONFIGS_SURE_PIC32MX_SRC_SURE_PIC32MXL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

/* GPIO Pin Configurations **************************************************/
/* The Sure PIC32MX board has five LEDs.  One (D4, lablel "Power") is not
 * controllable by software.  Four are controllable by software:
 *
 * D7  "USB"    Yellow  RD7 Low illuminates
 * D8  "SD"     Yellow  RD6 Low illuminates
 * D9  "Flash"  Yellow  RF0 Low illuminates
 * D10 "Error"  Red     RF1 Low illuminates
 *
 */

#define GPIO_USB_LED   (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORTD | GPIO_PIN7)
#define GPIO_SD_LED    (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORTD | GPIO_PIN6)
#define GPIO_FLASH_LED (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORTF | GPIO_PIN0)
#define GPIO_ERROR_LED (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORTF | GPIO_PIN1)

 /* LCD pin mapping (see configs/sure-pic32mx/README.txt)
 *
 *  --------------------- ---------- ----------------------------------
 *  PIC32                  Sure JP1   Sure Signal Description
 *  PIN  SIGNAL NAME      PIN NAME(s)
 *  --------------------- ---------- ----------------------------------
 *   34  Vbus             1.  +5V    +5V VBUS device mode
 *                                    To GND via capacitor
 *                        2.  GND    GND
 *   49  RD1              3.  Vo     Transistor circuit driven by PWM2
 *   44  PMA0/AN15/RB15   4.  RS     PMA0, Selects registers
 *   53  PMRD/RD5         5.  RW     PMRD/PMWR, Selects read or write
 *   45  PMPCS1/RD11      6.  E      Starts data read/write
 *   60  PMD0/RE0         7.  DB0    PMD0
 *   61  PMD1/RE1         8.  DB1    PMD1
 *   62  PMD2/RE2         9.  DB2    PMD2
 *   63  PMD3/RE3         10. DB3    PMD3
 *   64  PMD4/RE4         11. DB4    PMD4
 *    1  PMD5/RE5         12. DB5    PMD5
 *    2  PMD6/RE6         13. DB6    PMD6
 *    3  PMD7/RE7         14. DB7    PMD7
 *                        15. A      +5V_DUSB
 *   46 INT0/RD0          16. K      Transistor circuit driven by PWM1
 *  --------------------- ---------- ----------------------------------
 *
 *  Vbus power also requires Vbuson/AN5/RB5
 */

#define GPIO_LCD_RS    (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORTB | GPIO_PIN15)
#define GPIO_LCD_RW    (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORTD | GPIO_PIN5)
#define GPIO_LCD_E     (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORTD | GPIO_PIN11)

/* This pin drives the +5V needed by the LCD */

#define GPIO_LCD_PWR   (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORTB | GPIO_PIN5)

/* These pins are label PWM1 and PWM2 and so are obviously intended to
 * support modulated outputs.  However, here for simplicity, they are
 * just treated as on/off discretes outputs.
 */

#define GPIO_LCD_LIGHT (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORTD | GPIO_PIN0)
#define GPIO_LCD_COMP  (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORTD | GPIO_PIN1)

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
extern "C" {
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Name: pic32mx_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the Sure PIC32MX Logic board.
 *
 ************************************************************************************/

#if defined(CONFIG_PIC32MX_SPI2)
void weak_function pic32mx_spiinitialize(void);
#endif

/************************************************************************************
 * Name: pic32mx_usbdevinitialize
 *
 * Description:
 *   Called to configure the mini-B PHY on the Sure PIC32MX board for the USB device
 *
 ************************************************************************************/

#if defined(CONFIG_PIC32MX_USBDEV)
void weak_function pic32mx_usbdevinitialize(void);
#endif

/************************************************************************************
 * Name: pic32mx_ledinit
 *
 * Description:
 *   Configure on-board LEDs if LED support has been selected.
 *
 ************************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void pic32mx_ledinit(void);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_SURE_PIC32MX_SRC_SURE_PIC32MXL_H */
