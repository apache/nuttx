/****************************************************************************
 * boards/arm/stm32/nucleo-f4x1re/src/nucleo-f4x1re.h
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

#ifndef __BOARDS_ARM_STM32_NUCLEO_F401RE_SRC_NUCLEO_F401RE_H
#define __BOARDS_ARM_STM32_NUCLEO_F401RE_SRC_NUCLEO_F401RE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#define HAVE_MMCSD 1
#if !defined(CONFIG_STM32_SDIO) || !defined(CONFIG_MMCSD) || \
    !defined(CONFIG_MMCSD_SDIO)
#  undef HAVE_MMCSD
#endif

/* LED.  User LD2: the green LED is a user LED connected to Arduino signal
 * D13 corresponding to MCU I/O PA5 (pin 21) or PB13 (pin 34) depending on
 * the STM32 target.
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
 *
 */

/* SPI1 off */

#define GPIO_SPI1_MOSI_OFF (GPIO_INPUT | GPIO_PULLDOWN | \
                            GPIO_PORTA | GPIO_PIN7)
#define GPIO_SPI1_MISO_OFF (GPIO_INPUT | GPIO_PULLDOWN | \
                            GPIO_PORTA | GPIO_PIN6)
#define GPIO_SPI1_SCK_OFF  (GPIO_INPUT | GPIO_PULLDOWN | \
                            GPIO_PORTA | GPIO_PIN5)

/* SSD1306 */

#define GPIO_SSD1306_CS    (GPIO_OUTPUT|GPIO_OTYPER_PP(0)|GPIO_SPEED_2MHz|\
                            GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN6)

#define GPIO_SSD1306_CMD   (GPIO_OUTPUT|GPIO_OTYPER_PP(0)|GPIO_OSPEED_2MHz|\
                            GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN7)

#define GPIO_SSD1306_RST   (GPIO_OUTPUT|GPIO_OTYPER_PP(0)|GPIO_SPEED_2MHz|\
                            GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN9)

/* MCP2551 */

#define GPIO_MCP2515_CS   (GPIO_OUTPUT|GPIO_OTYPER_PP(0)|GPIO_SPEED_2MHz|\
                           GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN4)

#define GPIO_MCP2515_IRQ  (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTA|GPIO_PIN1)

#ifdef HAVE_MMCSD
#  define GPIO_SPI_CS_SD_CARD_OFF \
    (GPIO_INPUT | GPIO_PULLDOWN | GPIO_SPEED_2MHz | \
     GPIO_PORTB | GPIO_PIN5)
#endif

#ifdef HAVE_MMCSD
#  define GPIO_SPI_CS_SD_CARD \
    (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | \
     GPIO_OUTPUT_SET | GPIO_PORTB | GPIO_PIN5)
#endif

/* Devices on the onboard bus.
 *
 * Note that these are unshifted addresses.
 */

#define NUCLEO_I2C_OBDEV_LED       0x55
#define NUCLEO_I2C_OBDEV_HMC5883   0x1e

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
 *   USART or select CONFIG_NUCLEO_F401RE_AJOY_MINBUTTONS which will
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

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Global driver instances */

#ifdef CONFIG_STM32_SPI1
extern struct spi_dev_s *g_spi1;
#endif
#ifdef CONFIG_STM32_SPI2
extern struct spi_dev_s *g_spi2;
#endif
#ifdef HAVE_MMCSD
extern struct sdio_dev_s *g_sdio;
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture specific initialization
 *
 *   CONFIG_BOARDCTL=y:
 *     If CONFIG_NSH_ARCHINITIALIZE=y:
 *       Called from the NSH library (or other application)
 *     Otherwise, assumed to be called from some other application.
 *
 *   Otherwise CONFIG_BOARD_LATE_INITIALIZE=y:
 *     Called from board_late_initialize().
 *
 *   Otherwise, bad news:  Never called
 *
 ****************************************************************************/

int stm32_bringup(void);

/****************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins.
 *
 ****************************************************************************/

void stm32_spidev_initialize(void);

/****************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called to setup USB-related GPIO pins.
 *
 ****************************************************************************/

void stm32_usbinitialize(void);

/****************************************************************************
 * Name: stm32_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC driver.
 *
 ****************************************************************************/

#ifdef CONFIG_ADC
int stm32_adc_setup(void);
#endif

/****************************************************************************
 * Name: board_ajoy_initialize
 *
 * Description:
 *   Initialize and register the button joystick driver
 *
 ****************************************************************************/

#ifdef CONFIG_INPUT_AJOYSTICK
int board_ajoy_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_mcp2515initialize
 *
 * Description:
 *   Initialize and register the MCP2515 CAN driver.
 *
 ****************************************************************************/

#ifdef CONFIG_CAN_MCP2515
int stm32_mcp2515initialize(const char *devpath);
#endif

#endif /* __BOARDS_ARM_STM32_NUCLEO_F401RE_SRC_NUCLEO_F401RE_H */
