/****************************************************************************
 * boards/arm/stm32/nucleo-l152re/src/nucleo-l152re.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2015 Omni Hoverboards Inc. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Paul Alexander Patience <paul-a.patience@polymtl.ca>
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

#ifndef __BOARDS_ARM_STM32_NUCLEO_L152RE_SRC_NUCLEO_L152RE_H
#define __BOARDS_ARM_STM32_NUCLEO_L152RE_SRC_NUCLEO_L152RE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LED definitions **********************************************************/

/* The Nucleo L152RE board has three LEDs.  Two of these are controlled by
 * logic on the board and are not available for software control:
 *
 * LD1 COM:  LD1 default status is red.  LD1 turns to green to indicate that
 *           communications are in progress between the PC and the
 *           ST-LINK/V2-1.
 * LD3 PWR:  red LED indicates that the board is powered.
 *
 * And one can be controlled by software:
 *
 * User LD2: green LED is a user LED connected to the I/O PA5 of the
 *           STM32L152RET6.
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LED in
 * any way.  The following definition is used to access the LED.
 */

#define GPIO_LED1      (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_10MHz|\
                        GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN5)

#define LED_DRIVER_PATH "/dev/userleds"

/* Button definitions *******************************************************/

/* The Nucleo L152RE supports two buttons; only one button is controllable
 * by software:
 *
 *   B1 USER:  user button connected to the I/O PC13 of the STM32L152RET6.
 *   B2 RESET: push button connected to NRST is used to RESET the
 *             STM32L152RET6.
 *
 * NOTE that EXTI interrupts are configured.
 */

#define MIN_IRQBUTTON  BUTTON_USER
#define MAX_IRQBUTTON  BUTTON_USER
#define NUM_IRQBUTTONS 1

#define GPIO_BTN_USER  (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTC|GPIO_PIN13)

/* ILI9341 pins  */
#define GPIO_OUT (GPIO_OUTPUT | GPIO_PUSHPULL  | GPIO_PULLUP | GPIO_SPEED_40MHz | GPIO_OUTPUT_CLEAR)

#define GPIO_LCD_RST (GPIO_OUT | GPIO_PORTC | GPIO_PIN1) /* PC1 or PB9   A4 */
#define GPIO_LCD_CS (GPIO_OUT | GPIO_PORTB | GPIO_PIN0)  /* PB0          A3 */
#define GPIO_LCD_RS (GPIO_OUT | GPIO_PORTA | GPIO_PIN4)  /* PA4          A2 */
#define GPIO_LCD_WR (GPIO_OUT | GPIO_PORTA | GPIO_PIN1)  /* PA1          A1 */
#define GPIO_LCD_RD (GPIO_OUT | GPIO_PORTA | GPIO_PIN0)  /* PA0          A0 */

/* -------LCD pin------------- mcu ping ------------------------CN9 ---GPIO */

#define GPIO_LCD_D0 ( GPIO_OUT | GPIO_PORTA | GPIO_PIN9)   /*  D8     PA9   */
#define GPIO_LCD_D1 ( GPIO_OUT | GPIO_PORTC | GPIO_PIN7)   /*  D9     PC7   */
#define GPIO_LCD_D2 ( GPIO_OUT | GPIO_PORTA | GPIO_PIN10)  /*  D2     PA10  */
#define GPIO_LCD_D3 ( GPIO_OUT | GPIO_PORTB | GPIO_PIN3)   /*  D3     PB3   */
#define GPIO_LCD_D4 ( GPIO_OUT | GPIO_PORTB | GPIO_PIN5)   /*  D4     PB5   */
#define GPIO_LCD_D5 ( GPIO_OUT | GPIO_PORTB | GPIO_PIN4)   /*  D5     PB4   */
#define GPIO_LCD_D6 ( GPIO_OUT | GPIO_PORTB | GPIO_PIN10)  /*  D6     PB10  */
#define GPIO_LCD_D7 ( GPIO_OUT | GPIO_PORTA | GPIO_PIN8)   /*  D7     PA8   */

#ifdef CONFIG_LCD_ILI9341
FAR struct lcd_dev_s *stm32_ili93418b_initialize(void);
#endif

/* SPI sd card */

#define GPIO_SPI1_CS     (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_40MHz | \
                            GPIO_OUTPUT_SET | GPIO_PORTB | GPIO_PIN6)

#ifdef CONFIG_MMCSD_SPI
int stm32_spisd_initialize(int port, int minor);
#endif

/* procfs File System */

#ifdef CONFIG_FS_PROCFS
    #ifdef CONFIG_NSH_PROC_MOUNTPOINT
    #define STM32_PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT
    #else
    #define STM32_PROCFS_MOUNTPOINT "/proc"
    #endif
#endif /* CONFIG_FS_PROCFS */
#endif /* __BOARDS_ARM_STM32_NUCLEO_L152RE_SRC_NUCLEO_L152RE_H */
