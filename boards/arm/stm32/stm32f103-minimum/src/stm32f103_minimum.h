/****************************************************************************
 * boards/arm/stm32/stm32f103-minimum/src/stm32f103_minimum.h
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

#ifndef __BOARDS_ARM_STM32_STM32F103_MINIMUM_SRC_STM32F103_MINIMUM_H
#define __BOARDS_ARM_STM32_STM32F103_MINIMUM_SRC_STM32F103_MINIMUM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#include <arch/chip/chip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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

/* procfs File System */

#ifdef CONFIG_FS_PROCFS
#  ifdef CONFIG_NSH_PROC_MOUNTPOINT
#    define STM32_PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT
#  else
#    define STM32_PROCFS_MOUNTPOINT "/proc"
#  endif
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

/* GPIOs ********************************************************************/

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

/* PWM Configuration */

#define STM32F103MINIMUM_PWMTIMER   3
#define STM32F103MINIMUM_PWMCHANNEL 3

/* LM-75 Temperature Sensor: PA.0 */

#define GPIO_LM75_OSINT (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_PORTA|GPIO_PIN0)

/* nRF24 Configuration */

/* MCP2515 IRQ line: PB.0 */

#define GPIO_MCP2515_IRQ (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_PORTB|GPIO_PIN0)

/* USB Soft Connect Pullup: PC.13 */

#define GPIO_USB_PULLUP   (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                           GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN12)

/* GPIO pins used by the GPIO Subsystem */

#define BOARD_NGPIOIN     1 /* Amount of GPIO Input pins */
#define BOARD_NGPIOOUT    1 /* Amount of GPIO Output pins */
#define BOARD_NGPIOINT    1 /* Amount of GPIO Input w/ Interruption pins */

#define GPIO_IN1          (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_PORTA|GPIO_PIN0)
#define GPIO_OUT1         (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                           GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN1)

#define GPIO_INT1         (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_PORTA|GPIO_PIN2)

/* WS2812 LEDs */

#define WS2812_NLEDS 2
#define WS2812_SPI 1

/* Sensor */

#define BOARD_HYT271_NBUS      2 /* Bus number of connected HYT271 */

#define BOARD_HYT271_POWOUT    (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz| \
                                GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN0)
#define BOARD_HYT271_POWIN     (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_PORTA| \
                                GPIO_PIN1)

#define BOARD_DS18B20_NBUS     2 /* Bus number of connected DS18B20 */
#define BOARD_DS18B20_NSLAVES  2 /* Number of expected DS18B20 slaves */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

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
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the Hy-Mini STM32v
 *   board.
 *
 ****************************************************************************/

void stm32_spidev_initialize(void);

/****************************************************************************
 * Name: stm32_mmcsd_initialize
 *
 * Description:
 *   Initializes SPI-based SD card
 *
 ****************************************************************************/

#ifdef CONFIG_MMCSD
int stm32_mmcsd_initialize(int minor);
#endif

/****************************************************************************
 * Name: stm32_w25initialize
 *
 * Description:
 *   Called to initialize Winbond W25 memory
 *
 ****************************************************************************/

int stm32_w25initialize(int minor);

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

/****************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called to setup USB-related GPIO pins for the Hy-Mini STM32v board.
 *
 ****************************************************************************/

void stm32_usbinitialize(void);

/****************************************************************************
 * Name: stm32_pwm_setup
 *
 * Description:
 *   Initialize PWM and register the PWM device.
 *
 ****************************************************************************/

#ifdef CONFIG_PWM
int stm32_pwm_setup(void);
#endif

/****************************************************************************
 * Name: stm32_hyt271initialize
 *
 * Description:
 *   Function used to initialize HYT271 snesors on a i2c bus
 *
 * Parameter:
 *   devno   - First character device number
 *
 * Return
 *   Error or number of device that have been successfully registered.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_HYT271
int stm32_hyt271initialize(int devno);
#endif

/****************************************************************************
 * Name: stm32_ds18b20initialize
 *
 * Description:
 *   Function used to initialize DS18B20 snesors on a 1wire bus
 *
 * Parameter:
 *   devno   - First character device number
 *
 * Return
 *   Error or number of device that have been successfully registered.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_DS18B20
int stm32_ds18b20initialize(int devno);
#endif

/****************************************************************************
 * Name: stm32_mfrc522initialize
 *
 * Description:
 *   Function used to initialize the MFRC522 RFID Transceiver
 *
 ****************************************************************************/

#ifdef CONFIG_CL_MFRC522
int stm32_mfrc522initialize(const char *devpath);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32_STM32F103_MINIMUM_SRC_STM32F103_MINIMUM_H */
