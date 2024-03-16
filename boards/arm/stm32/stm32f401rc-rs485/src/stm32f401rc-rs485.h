/****************************************************************************
 * boards/arm/stm32/stm32f401rc-rs485/src/stm32f401rc-rs485.h
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

#ifndef __BOARDS_ARM_STM32_STM32F401RC_RS485_SRC_STM32F401RC_RS485_H
#define __BOARDS_ARM_STM32_STM32F401RC_RS485_SRC_STM32F401RC_RS485_H

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

#define HAVE_SDIO  1
#if !defined(CONFIG_STM32_SDIO) || !defined(CONFIG_MMCSD) || \
    !defined(CONFIG_MMCSD_SDIO)
#  undef HAVE_SDIO
#endif

#define SDIO_MINOR  0  /* Any minor number, default 0 */
#define SDIO_SLOTNO 0  /* Only one slot */

#define GPIO_SDIO_NCD    (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|\
                          GPIO_PORTA|GPIO_PIN8)

/* The STM32F401RC-RS485 has 4 blue LEDs connected as below:
 * - LED_1 is connected to the GPIO PC0.
 * - LED_2 is connected to the GPIO PC1.
 * - LED_3 is connected to the GPIO PC2.
 * - LED_4 is connected to the GPIO PC3.
 *
 * - When the I/O is HIGH value, the LED is on.
 * - When the I/O is LOW, the LED is off.
 */

#define GPIO_LED1 \
  (GPIO_PORTC | GPIO_PIN0 | GPIO_OUTPUT_CLEAR | GPIO_OUTPUT | \
   GPIO_SPEED_50MHz)

#define GPIO_LED2 \
  (GPIO_PORTC | GPIO_PIN1 | GPIO_OUTPUT_CLEAR | GPIO_OUTPUT | \
   GPIO_SPEED_50MHz)

#define GPIO_LED3 \
  (GPIO_PORTC | GPIO_PIN2 | GPIO_OUTPUT_CLEAR | GPIO_OUTPUT | \
   GPIO_SPEED_50MHz)

#define GPIO_LED4 \
  (GPIO_PORTC | GPIO_PIN3 | GPIO_OUTPUT_CLEAR | GPIO_OUTPUT | \
   GPIO_SPEED_50MHz)

/* Buttons
 * The STM32F401RC-RS485 has 3 user buttons.
 * - SW3 is connected to the GPIO PB13.
 * - SW4 is connected to the GPIO PB14.
 * - SW5 is connected to the GPIO PB15.
 */

#define MIN_IRQBUTTON   BUTTON_SW3
#define MAX_IRQBUTTON   BUTTON_SW5
#define NUM_IRQBUTTONS  3

#define GPIO_BTN_SW3 \
  (GPIO_INPUT |GPIO_FLOAT |GPIO_EXTI | GPIO_PORTB | GPIO_PIN13)
#define GPIO_BTN_SW4 \
  (GPIO_INPUT |GPIO_FLOAT |GPIO_EXTI | GPIO_PORTB | GPIO_PIN14)
#define GPIO_BTN_SW5 \
  (GPIO_INPUT |GPIO_FLOAT |GPIO_EXTI | GPIO_PORTB | GPIO_PIN15)

/* SPI1 off */

#define GPIO_SPI1_MOSI_OFF (GPIO_INPUT | GPIO_PULLDOWN | \
                            GPIO_PORTA | GPIO_PIN7)
#define GPIO_SPI1_MISO_OFF (GPIO_INPUT | GPIO_PULLDOWN | \
                            GPIO_PORTA | GPIO_PIN6)
#define GPIO_SPI1_SCK_OFF  (GPIO_INPUT | GPIO_PULLDOWN | \
                            GPIO_PORTA | GPIO_PIN5)

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

/* PWM
 *
 * The STM32F401RC-RS485 has no real on-board PWM devices, but the board can
 * be configured to output a pulse train using TIM3 CH1 on PA6.
 */

#define STM32F401RCRS485_PWMTIMER   3
#define STM32F401RCRS485_PWMCHANNEL 1

/* Quadrature Encoder
 *
 * Use Timer 3 (TIM3) on channels 1 and 2 for QEncoder, using PB4 and PA7.
 */

#define STM32F401RCRS485_QETIMER 3

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
 * Name: stm32_pwm_setup
 *
 * Description:
 *   Initialize PWM and register the PWM device.
 *
 * Return Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_PWM
int stm32_pwm_setup(void);
#endif

/****************************************************************************
 * Name: stm32_sdio_initialize
 *
 * Description:
 *   Initialize SDIO-based MMC/SD card support
 *
 ****************************************************************************/

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && defined(CONFIG_STM32_SDIO)
int stm32_sdio_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_i2cbus_initialize
 *
 * Description:
 *   Initialize one I2C bus
 *
 ****************************************************************************/

struct i2c_master_s *stm32_i2cbus_initialize(int port);

/****************************************************************************
 * Name: stm32_at24_init
 *
 * Description:
 *   Initialize and register the EEPROM for 24XX  driver.
 *
 ****************************************************************************/

int stm32_at24_init(char *path);

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

#endif /* __BOARDS_ARM_STM32_STM32F401RC_RS485_SRC_STM32F401RC_RS485_H */
