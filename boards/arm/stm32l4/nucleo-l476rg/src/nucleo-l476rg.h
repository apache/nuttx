/****************************************************************************
 * boards/arm/stm32l4/nucleo-l476rg/src/nucleo-l476rg.h
 *
 *   Copyright (C) 2014, 2016, 2018-2019 Gregory Nutt. All rights reserved.
 *   Authors: Frank Bennett
 *            Gregory Nutt <gnutt@nuttx.org>
 *            Sebastien Lorquet <sebastien@lorquet.fr>
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

#ifndef __BOARDS_ARM_STM32L4_NUCLEO_L476RG_SRC_NUCLEO_L476RG_H
#define __BOARDS_ARM_STM32L4_NUCLEO_L476RG_SRC_NUCLEO_L476RG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>

#include "stm32l4_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#define HAVE_PROC           1
#define HAVE_RTC_DRIVER     1

#define HAVE_MMCSD_SPI      1
#define HAVE_MMCSD_SDIO     1

#if !defined(CONFIG_FS_PROCFS)
#  undef HAVE_PROC
#endif

#if defined(HAVE_PROC) && defined(CONFIG_DISABLE_MOUNTPOINT)
#  warning Mountpoints disabled.  No procfs support
#  undef HAVE_PROC
#endif

/* Check if we can support the RTC driver */

#if !defined(CONFIG_RTC) || !defined(CONFIG_RTC_DRIVER)
#  undef HAVE_RTC_DRIVER
#endif

#if !defined(CONFIG_STM32L4_SPI1) || !defined(CONFIG_MMCSD) || \
    !defined(CONFIG_MMCSD_SPI)
#  undef  HAVE_MMCSD_SPI
#endif

#if !defined(CONFIG_STM32L4_SDIO) || !defined(CONFIG_MMCSD) || \
    !defined(CONFIG_MMCSD_SDIO)
#  undef HAVE_MMCSD_SDIO
#endif

/* LED.  User LD2: the green LED is a user LED connected to Arduino signal D13
 * corresponding to MCU I/O PA5 (pin 21) or PB13 (pin 34) depending on the STM32
 * target.
 *
 * - When the I/O is HIGH value, the LED is on.
 * - When the I/O is LOW, the LED is off.
 */

#define GPIO_LD2 \
  (GPIO_PORTA | GPIO_PIN5 | GPIO_OUTPUT_CLEAR | GPIO_OUTPUT | GPIO_PULLUP | \
   GPIO_SPEED_50MHz)

/* Buttons
 *
 * B1 USER: the user button is connected to the I/O PC13 (pin 2) of the STM32
 * microcontroller.
 */

#define MIN_IRQBUTTON   BUTTON_USER
#define MAX_IRQBUTTON   BUTTON_USER
#define NUM_IRQBUTTONS  1

#define GPIO_BTN_USER \
  (GPIO_INPUT |GPIO_FLOAT |GPIO_EXTI | GPIO_PORTC | GPIO_PIN13)

/* The shield uses the following pins:
 *
 *   +5V
 *   GND
 *  SERIAL_TX=PA_2    USER_BUTTON=PC_13
 *  SERIAL_RX=PA_3            LD2=PA_5
 *
 * Analog                         Digital
 *  A0=PA_0    USART2RX D0=PA_3              D8 =PA_9
 *  A1=PA_1    USART2TX D1=PA_2              D9 =PC_7
 *  A2=PA_4             D2=PA_10     WIFI_CS=D10=PB_6 SPI_CS
 *  A3=PB_0    WIFI_INT=D3=PB_3              D11=PA_7 SPI_MOSI
 *  A4=PC_1       SD_CS=D4=PB_5              D12=PA_6 SPI_MISO
 *  A5=PC_0     WIFI_EN=D5=PB_4          LD2=D13=PA_5 SPI_SCK
 *                 LED2=D6=PB_10    I2C1_SDA=D14=PB_9 WIFI Probe
 *                      D7=PA_8     I2C1_SCL=D15=PB_8 WIFI Probe
 *
 *  mostly from: https://mbed.org/platforms/ST-Nucleo-F401RE/
 */

#ifdef CONFIG_WL_CC1101
#  define GPIO_CC1101_PWR  (GPIO_PORTC | GPIO_PIN6 | GPIO_OUTPUT_SET | \
                            GPIO_OUTPUT | GPIO_PULLUP | GPIO_SPEED_50MHz)
#  define GPIO_CC1101_CS   (GPIO_PORTB | GPIO_PIN12 | GPIO_OUTPUT_SET | \
                            GPIO_OUTPUT | GPIO_PULLUP | GPIO_SPEED_50MHz)
#  define GPIO_CC1101_MISO (GPIO_PORTB | GPIO_PIN14)
#  define GPIO_CC1101_GDO2 (GPIO_PORTC | GPIO_PIN10 | \
                            GPIO_EXTI | GPIO_SPEED_50MHz)
#endif

/* SPI chip selects */

#ifdef CONFIG_MMCSD_SPI
#  define GPIO_SPI_CS_SD_CARD \
                          (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                           GPIO_OUTPUT_SET | GPIO_PORTB | GPIO_PIN10)
#endif

#ifdef CONFIG_LCD_PCD8544

#define STM32_LCD_CS      (GPIO_OUTPUT|GPIO_PULLUP|GPIO_SPEED_2MHz|\
                           GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN4)

#define STM32_LCD_CD      (GPIO_OUTPUT|GPIO_PULLUP|GPIO_SPEED_2MHz|\
                           GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN10)

#define STM32_LCD_RST     (GPIO_OUTPUT|GPIO_PULLUP|GPIO_SPEED_2MHz|\
                           GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN13)
#endif

/* Devices on the onboard bus.
 *
 * Note that these are unshifted addresses.
 */

#define NUCLEO_I2C_OBDEV_LED       0x55
#define NUCLEO_I2C_OBDEV_HMC5883   0x1e

/* User GPIOs
 *
 * GPIO0-1 are for probing WIFI status
 */

/* Itead Joystick Shield
 *
 * See http://imall.iteadstudio.com/im120417014.html for more information
 * about this joystick.
 *
 *   --------- ----------------- ---------------------------------
 *   ARDUINO   ITEAD             NUCLEO-F4x1
 *   PIN NAME  SIGNAL            SIGNAL
 *   --------- ----------------- ---------------------------------
 *    D3       Button E Output   PB3
 *    D4       Button D Output   PB5
 *    D5       Button C Output   PB4
 *    D6       Button B Output   PB10
 *    D7       Button A Output   PA8
 *    D8       Button F Output   PA9
 *    D9       Button G Output   PC7
 *    A0       Joystick Y Output PA0  ADC1_0
 *    A1       Joystick X Output PA1  ADC1_1
 *   --------- ----------------- ---------------------------------
 *
 *   All buttons are pulled on the shield.  A sensed low value indicates
 *   when the button is pressed.
 *
 *   NOTE: Button F cannot be used with the default USART1 configuration
 *   because PA9 is configured for USART1_RX by default.  Use select
 *   different USART1 pins in the board.h file or select a different
 *   USART or select CONFIG_NUCLEO_L476RG_AJOY_MINBUTTONS which will
 *   eliminate all but buttons A, B, and C.
 */

#define ADC_XOUPUT   1 /* X output is on ADC channel 1 */
#define ADC_YOUPUT   0 /* Y output is on ADC channel 0 */

#define GPIO_BUTTON_A \
  (GPIO_INPUT | GPIO_PULLUP |GPIO_EXTI | GPIO_PORTA | GPIO_PIN8)
#define GPIO_BUTTON_B \
  (GPIO_INPUT | GPIO_PULLUP |GPIO_EXTI | GPIO_PORTB | GPIO_PIN10)
#define GPIO_BUTTON_C \
  (GPIO_INPUT | GPIO_PULLUP |GPIO_EXTI | GPIO_PORTB | GPIO_PIN4)
#define GPIO_BUTTON_D \
  (GPIO_INPUT | GPIO_PULLUP |GPIO_EXTI | GPIO_PORTB | GPIO_PIN5)
#define GPIO_BUTTON_E \
  (GPIO_INPUT | GPIO_PULLUP |GPIO_EXTI | GPIO_PORTB | GPIO_PIN3)
#define GPIO_BUTTON_F \
  (GPIO_INPUT | GPIO_PULLUP |GPIO_EXTI | GPIO_PORTA | GPIO_PIN9)
#define GPIO_BUTTON_G \
  (GPIO_INPUT | GPIO_PULLUP |GPIO_EXTI | GPIO_PORTC | GPIO_PIN7)

/* Itead Joystick Signal interpretation:
 *
 *   --------- ----------------------- ---------------------------
 *   BUTTON     TYPE                    NUTTX ALIAS
 *   --------- ----------------------- ---------------------------
 *   Button A  Large button A          JUMP/BUTTON 3
 *   Button B  Large button B          FIRE/BUTTON 2
 *   Button C  Joystick select button  SELECT/BUTTON 1
 *   Button D  Tiny Button D           BUTTON 6
 *   Button E  Tiny Button E           BUTTON 7
 *   Button F  Large Button F          BUTTON 4
 *   Button G  Large Button G          BUTTON 5
 *   --------- ----------------------- ---------------------------
 */

#define GPIO_BUTTON_1 GPIO_BUTTON_C
#define GPIO_BUTTON_2 GPIO_BUTTON_B
#define GPIO_BUTTON_3 GPIO_BUTTON_A
#define GPIO_BUTTON_4 GPIO_BUTTON_F
#define GPIO_BUTTON_5 GPIO_BUTTON_G
#define GPIO_BUTTON_6 GPIO_BUTTON_D
#define GPIO_BUTTON_7 GPIO_BUTTON_E

#define GPIO_SELECT   GPIO_BUTTON_1
#define GPIO_FIRE     GPIO_BUTTON_2
#define GPIO_JUMP     GPIO_BUTTON_3

/* GPIO pins used by the GPIO Subsystem
 * Added by: Ben vd Veen (DisruptiveNL) -- www.nuttx.nl
 */

#define BOARD_NGPIOIN     1 /* Amount of GPIO Input pins */
#define BOARD_NGPIOOUT    1 /* Amount of GPIO Output pins */
#define BOARD_NGPIOINT    1 /* Amount of GPIO Input w/ Interruption pins */

/* This was really something to find out! But the GPIOA Port was causing
 * conflicts with the nsh console.  So that's why I have chosen GPIOB
 *
 * Author: Ben vd Veen (DisruptiveNL) -- www.nuttx.nl
 * https://www.youtube.com/watch?v=VXsTLzI6idA -- Original video GPIO
 */

#define GPIO_IN1          (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN0)
#define GPIO_OUT1         (GPIO_OUTPUT | GPIO_OUTPUT | GPIO_SPEED_50MHz | \
                           GPIO_OUTPUT_SET | GPIO_PORTB | GPIO_PIN1)
#define GPIO_INT1         (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN2)

#define GPIO_HTS221_INT   (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN10)
#define GPIO_LSM6DSL_INT1 (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN4)
#define GPIO_LSM6DSL_INT2 (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN5)

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Global driver instances */

#ifdef CONFIG_STM32L4_SPI1
extern struct spi_dev_s *g_spi1;
#endif
#ifdef CONFIG_STM32L4_SPI2
extern struct spi_dev_s *g_spi2;
#endif
#ifdef HAVE_MMCSD_SDIO
extern struct sdio_dev_s *g_sdio;
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32l4_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins.
 *
 ****************************************************************************/

void stm32l4_spiinitialize(void);

/****************************************************************************
 * Name: stm32l4_usbinitialize
 *
 * Description:
 *   Called to setup USB-related GPIO pins.
 *
 ****************************************************************************/

void stm32l4_usbinitialize(void);

/****************************************************************************
 * Name: stm32l4_gpio_initialize
 *
 * Description:
 *   Initialize GPIO drivers for use with /apps/examples/gpio
 *   Added by: Ben vd Veen (DisruptiveNL) -- www.nuttx.nl
 *
 ****************************************************************************/

#ifdef CONFIG_DEV_GPIO
int stm32l4_gpio_initialize(void);
#endif

/****************************************************************************
 * Name: stm32l4_can_setup -- DisruptiveNL
 *
 * Description:
 *   Initialize CAN and register the CAN device.
 *   Added by: Ben vd Veen (DisruptiveNL) -- www.nuttx.nl
 *
 ****************************************************************************/

#ifdef CONFIG_CAN
int stm32l4_can_setup(void);
#endif

/****************************************************************************
 * Name: stm32l4_pwm_setup
 *
 * Description:
 *   Initialize PWM and register the PWM device.
 *
 ****************************************************************************/

#ifdef CONFIG_PWM
int stm32l4_pwm_setup(void);
#endif

/****************************************************************************
 * Name: stm32l4_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC driver.
 *
 ****************************************************************************/

#ifdef CONFIG_ADC
int stm32l4_adc_setup(void);
#endif

/****************************************************************************
 * Name: board_ajoy_initialize
 *
 * Description:
 *   Initialize and register the button joystick driver
 *
 ****************************************************************************/

#ifdef CONFIG_AJOYSTICK
int board_ajoy_initialize(void);
#endif

/****************************************************************************
 * Name: stm32l4_mmcsd_initialize
 *
 * Description:
 *   Initializes SPI-based SD card
 *
 ****************************************************************************/

#ifdef CONFIG_MMCSD_SPI
int stm32l4_mmcsd_initialize(int minor);
#endif

/****************************************************************************
 * Name: board_timer_driver_initialize
 *
 * Description:
 *   Initialize and register a timer
 *
 ****************************************************************************/

#ifdef CONFIG_TIMER
int board_timer_driver_initialize(FAR const char *devpath, int timer);
#endif

/****************************************************************************
 * Name: stm32l4_qencoder_initialize
 *
 * Description:
 *   Initialize and register a qencoder
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_QENCODER
int stm32l4_qencoder_initialize(FAR const char *devpath, int timer);
#endif

/****************************************************************************
 * Name: stm32l4_cc1101_initialize
 *
 * Description:
 *   Initialize and register the cc1101 radio driver
 *
 ****************************************************************************/

#ifdef CONFIG_WL_CC1101
int stm32l4_cc1101_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_as726xinitialize
 *
 * Description:
 * Called to configure an I2C and to register AS726X.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_AS726X
int stm32_as726xinitialize(FAR const char *devpath);
#endif

/****************************************************************************
 * Name: stm32_bmp180initialize
 *
 * Description:
 * Called to configure an I2C and to register BMP180.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_BMP180
int stm32_bmp180initialize(FAR const char *devpath);
#endif

#endif /* __BOARDS_ARM_STM32L4_NUCLEO_L476RG_SRC_NUCLEO_L476RG_H */
