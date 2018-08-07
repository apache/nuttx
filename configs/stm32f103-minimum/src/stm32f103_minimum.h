/************************************************************************************
 * configs/stm32f103-minimum/src/stm32f103_minimum.h
 *
 *   Copyright (C) 2016, 2018 Gregory Nutt. All rights reserved.
 *   Author: Laurent Latil <laurent@latil.nom.fr>
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

#ifndef __CONFIGS_STM32F103_MINIMUM_SRC_STM32F103_MINIMUM_H
#define __CONFIGS_STM32F103_MINIMUM_SRC_STM32F103_MINIMUM_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#include <arch/chip/chip.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#define HAVE_AT24 1

/* AT24 Serial EEPROM */

#define AT24_I2C_BUS   1 /* AT24C256 connected to I2C1 */
#define AT24_MINOR     0

#if !defined(CONFIG_MTD_AT24XX) || !defined(CONFIG_STM32_I2C1)
#  undef HAVE_AT24
#endif

/* Can't support AT24 features if mountpoints are disabled or if we were not
 * asked to mount the AT25 part
 */

#if defined(CONFIG_DISABLE_MOUNTPOINT) || \
   !defined(CONFIG_STM32F103MINIMUM_AT24_BLOCKMOUNT)
#  undef HAVE_AT24
#endif

/* If we are going to mount the AT24, then they user must also have told
 * us what to do with it by setting one of these.
 */

#ifndef CONFIG_FS_NXFFS
#  undef CONFIG_STM32F103MINIMUM_AT24_NXFFS
#endif

#if !defined(CONFIG_STM32F103MINIMUM_AT24_FTL) && \
    !defined(CONFIG_STM32F103MINIMUM_AT24_NXFFS)
#  undef HAVE_AT24
#endif

/* How many SPI modules does this chip support? The LM3S6918 supports 2 SPI
 * modules (others may support more -- in such case, the following must be
 * expanded).
 */

#if STM32_NSPI < 1
#  undef CONFIG_STM32_SPI1
#  undef CONFIG_STM32_SPI2
#elif STM32_NSPI < 2
#  undef CONFIG_STM32_SPI2
#endif

/* GPIOs **************************************************************/
/* LEDs */

/* The Blue/Red pills have a different pinout to the Black pill,
 * which includes the board's user LED.
 */

#ifdef CONFIG_STM32F103MINIMUM_BLACKPILL
#  define GPIO_LED1         (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                            GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN12)
#else
#  define GPIO_LED1         (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                             GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN13)
#endif

/* BUTTONs */

#define GPIO_BTN_USER1    (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|\
                           GPIO_EXTI|GPIO_PORTA|GPIO_PIN0)

#define GPIO_BTN_USER2    (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|\
                           GPIO_EXTI|GPIO_PORTA|GPIO_PIN1)

#define MIN_IRQBUTTON     BUTTON_USER1
#define MAX_IRQBUTTON     BUTTON_USER2
#define NUM_IRQBUTTONS    (BUTTON_USER1 - BUTTON_USER2 + 1)

/* ZERO CROSS pin definition */

#define GPIO_ZEROCROSS    (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_PORTA|GPIO_PIN0)

/* Pins config to use with HC-SR04 sensor */

#define GPIO_HCSR04_INT   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_PORTA|GPIO_PIN0)
#define GPIO_HCSR04_TRIG  (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                           GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN1)

/* Pin for APDS-9960 sensor */

#define GPIO_APDS9960_INT (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_PORTA|GPIO_PIN0)

/* SPI chip selects */

#define FLASH_SPI1_CS     (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                           GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN4)

#define GPIO_CS_MFRC522   (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                           GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN4)

#define STM32_LCD_CS      (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                           GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN4)

#define GPIO_MAX6675_CS   (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                           GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN4)

#define GPIO_MCP2515_CS   (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                           GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN4)

#define GPIO_NRF24L01_CS  (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                           GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN4)

#define GPIO_SDCARD_CS    (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                           GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN4)

#define STM32_LCD_RST     (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                           GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN3)

#define STM32_LCD_RS      (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                           GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN2)

#define STM32_LCD_CD      (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                           GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN2)

/* PWN Configuration */

#define STM32F103MINIMUM_PWMTIMER   3
#define STM32F103MINIMUM_PWMCHANNEL 3

/* LM-75 Temperature Sensor: PA.0 */

#define GPIO_LM75_OSINT (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_PORTA|GPIO_PIN0)

/* nRF24 Configuration */

/* NRF24L01 chip enable:  PB.1 */

#define GPIO_NRF24L01_CE  (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                           GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN1)

/* NRF24L01 IRQ line:  PA.0 */

#define GPIO_NRF24L01_IRQ (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_PORTA|GPIO_PIN0)

/* MCP2515 IRQ line: PB.0 */

#define GPIO_MCP2515_IRQ (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_PORTB|GPIO_PIN0)

/* USB Soft Connect Pullup: PC.13 */

#define GPIO_USB_PULLUP   (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                           GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN13)

/* GPIO pins used by the GPIO Subsystem */

#define BOARD_NGPIOIN     1 /* Amount of GPIO Input pins */
#define BOARD_NGPIOOUT    1 /* Amount of GPIO Output pins */
#define BOARD_NGPIOINT    1 /* Amount of GPIO Input w/ Interruption pins */

#define GPIO_IN1          (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_PORTA|GPIO_PIN0)
#define GPIO_OUT1         (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                           GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN1)

#define GPIO_INT1         (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_PORTA|GPIO_PIN2)

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture specific initialization
 *
 *   CONFIG_LIB_BOARDCTL=y:
 *     If CONFIG_NSH_ARCHINITIALIZE=y:
 *       Called from the NSH library (or other application)
 *     Otherwise, assumed to be called from some other application.
 *
 *   Otherwise CONFIG_BOARD_INITIALIZE=y:
 *     Called from board_initialize().
 *
 *   Otherise, bad news:  Never called
 *
 ****************************************************************************/

int stm32_bringup(void);

/****************************************************************************
 * Name: stm32_gpio_initialize
 *
 * Description:
 *   Initialize GPIO drivers for use with /apps/examples/gpio
 *
 ****************************************************************************/

#ifdef CONFIG_DEV_GPIO
int stm32_gpio_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_zerocross_initialize
 *
 * Description:
 *   Initialize and register the zero cross driver
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_ZEROCROSS
int stm32_zerocross_initialize(void);
#endif

/************************************************************************************
 * Name: stm32_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC driver.
 *
 ************************************************************************************/

#ifdef CONFIG_ADC
int stm32_adc_setup(void);
#endif

/************************************************************************************
 * Name: stm32_apds9960initialize
 *
 * Description:
 *   Initialize APDS-9960 gesture sensor
 *
 ************************************************************************************/

#ifdef CONFIG_SENSORS_APDS9960
int stm32_apds9960initialize(FAR const char *devpath);
#endif

/****************************************************************************
 * Name: stm32_bmp180initialize
 *
 * Description:
 *   Called to configure an I2C and to register BMP180 for the stm32f4discovery
 *   board.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_BMP180
int stm32_bmp180initialize(FAR const char *devpath);
#endif

/************************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the Hy-Mini STM32v board.
 *
 ************************************************************************************/

void stm32_spidev_initialize(void);

/************************************************************************************
 * Name: stm32_mmcsd_initialize
 *
 * Description:
 *   Initializes SPI-based SD card
 *
 ************************************************************************************/

#ifdef CONFIG_MMCSD
int stm32_mmcsd_initialize(int minor);
#endif

/****************************************************************************
 * Name: nunchuck_initialize
 *
 * Description:
 *   Initialize and register the button joystick driver
 *
 ****************************************************************************/

#ifdef CONFIG_INPUT_NUNCHUCK
int nunchuck_initialize(FAR char *devname);
#endif

/************************************************************************************
 * Name: stm32_hcsr04_initialize
 *
 * Description:
 *   Called to initialize the HC-SR04 sensor
 *
 ************************************************************************************/

int stm32_hcsr04_initialize(FAR const char *devname);

/************************************************************************************
 * Name: stm32_lm75initialize
 *
 * Description:
 *   Called to initialize LM75 temperature sensor
 *
 ************************************************************************************/

#ifdef CONFIG_LM75_I2C
int stm32_lm75initialize(FAR const char *devpath);
#endif

/************************************************************************************
 * Name: stm32_max6675initialize
 *
 * Description:
 *   Called to initialize MAX6675 temperature sensor
 *
 ************************************************************************************/

#ifdef CONFIG_SENSORS_MAX6675
int stm32_max6675initialize(FAR const char *devpath);
#endif

/************************************************************************************
 * Name: stm32_w25initialize
 *
 * Description:
 *   Called to initialize Winbond W25 memory
 *
 ************************************************************************************/

int stm32_w25initialize(int minor);

/****************************************************************************
 * Name: stm32_qencoder_initialize
 *
 * Description:
 *   Initialize and register a qencoder
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_QENCODER
int stm32_qencoder_initialize(FAR const char *devpath, int timer);
#endif

/****************************************************************************
 * Name: stm32_rgbled_setup
 *
 * Description:
 *   This function is called by board initialization logic to configure the
 *   RGB LED driver.  This function will register the driver as /dev/rgbled0.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_RGBLED
int stm32_rgbled_setup(void);
#endif

/************************************************************************************
 * Name: stm32_apa102init
 *
 * Description:
 *   Initialize and register the APA102 LED Strip driver
 *
 ************************************************************************************/

#ifdef CONFIG_LEDS_APA102
int stm32_apa102init(FAR const char *devpath);
#endif

/************************************************************************************
 * Name: stm32_mcp2515initialize
 *
 * Description:
 *   Initialize and register the MCP2515 CAN driver.
 *
 ************************************************************************************/

#ifdef CONFIG_CAN_MCP2515
int stm32_mcp2515initialize(FAR const char *devpath);
#endif

/************************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called to setup USB-related GPIO pins for the Hy-Mini STM32v board.
 *
 ************************************************************************************/

void stm32_usbinitialize(void);

/************************************************************************************
 * Name: stm32_pwm_setup
 *
 * Description:
 *   Initialize PWM and register the PWM device.
 *
 ************************************************************************************/

#ifdef CONFIG_PWM
int stm32_pwm_setup(void);
#endif

/************************************************************************************
 * Name: stm32_wlinitialize
 *
 * Description:
 *   Initialize the NRF24L01 wireless module
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

#ifdef CONFIG_WL_NRF24L01
void stm32_wlinitialize(void);
#endif

/************************************************************************************
 * Name: stm32_mfrc522initialize
 *
 * Description:
 *   Function used to initialize the MFRC522 RFID Transceiver
 *
 ************************************************************************************/

#ifdef CONFIG_CL_MFRC522
int stm32_mfrc522initialize(FAR const char *devpath);
#endif

/************************************************************************************
 * Name: stm32_tone_setup
 *
 * Description:
 *   Function used to initialize a PWM and Oneshot timers to Audio Tone Generator.
 *
 ************************************************************************************/

#ifdef CONFIG_AUDIO_TONE
int stm32_tone_setup(void);
#endif

/***********************************************************************************
 * Name: stm32_veml6070initialize
 *
 * Description:
 *   Called to configure an I2C and to register VEML6070 for the stm32f103-minimum
 *   board.
 *
 ***********************************************************************************/

#ifdef CONFIG_SENSORS_VEML6070
int stm32_veml6070initialize(FAR const char *devpath);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_STM32F103_MINIMUM_SRC_STM32F103_MINIMUM_H */
