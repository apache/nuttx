/****************************************************************************
 * boards/arm/stm32/stm32f429i-disco/src/stm32f429i-disco.h
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

#ifndef __BOARDS_ARM_STM32_STM32F429I_DISCO_SRC_STM32F429I_DISCO_H
#define __BOARDS_ARM_STM32_STM32F429I_DISCO_SRC_STM32F429I_DISCO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#ifdef CONFIG_STM32F429I_DISCO_ILI9341
#include <nuttx/lcd/ili9341.h>
#endif

#include <arch/stm32/chip.h>

#include "stm32_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#define HAVE_PROC       1
#define HAVE_USBDEV     1
#define HAVE_USBHOST    1
#define HAVE_USBMONITOR 1

/* Can't support USB host or device features if USB OTG HS is not enabled */

#ifndef CONFIG_STM32_OTGHS
#  undef HAVE_USBDEV
#  undef HAVE_USBHOST
#endif

/* Can't support USB device monitor if USB device is not enabled */

#ifndef CONFIG_USBDEV
#  undef HAVE_USBDEV
#endif

/* Can't support USB host is USB host is not enabled */

#ifndef CONFIG_USBHOST
#  undef HAVE_USBHOST
#endif

/* Check if we should enable the USB monitor before starting NSH */

#ifndef CONFIG_USBMONITOR
#  undef HAVE_USBMONITOR
#endif

#ifndef HAVE_USBDEV
#  undef CONFIG_USBDEV_TRACE
#endif

#ifndef HAVE_USBHOST
#  undef CONFIG_USBHOST_TRACE
#endif

#if !defined(CONFIG_USBDEV_TRACE) && !defined(CONFIG_USBHOST_TRACE)
#  undef HAVE_USBMONITOR
#endif

/* Check if we have the procfs file system */

#if !defined(CONFIG_FS_PROCFS)
#  undef HAVE_PROC
#endif

#if defined(HAVE_PROC) && defined(CONFIG_DISABLE_MOUNTPOINT)
#  warning Mountpoints disabled.  No procfs support
#  undef HAVE_PROC
#endif

/* How many SPI modules does this chip support? */

#if STM32_NSPI < 1
#  undef CONFIG_STM32_SPI1
#  undef CONFIG_STM32_SPI2
#  undef CONFIG_STM32_SPI3
#elif STM32_NSPI < 2
#  undef CONFIG_STM32_SPI2
#  undef CONFIG_STM32_SPI3
#elif STM32_NSPI < 3
#  undef CONFIG_STM32_SPI3
#endif

/* STMPE811 on I2C3 */

#define STMPE811_ADDR1    0x41
#define STMPE811_ADDR2    0x44

#define GPIO_IO_EXPANDER (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTA|GPIO_PIN15)

/* STM32F429 Discovery GPIOs ************************************************/

/* LEDs */

#define GPIO_LED1       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_CLEAR|GPIO_PORTG|GPIO_PIN13)
#define GPIO_LED2       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_CLEAR|GPIO_PORTG|GPIO_PIN14)

/* BUTTONS -- NOTE that all have EXTI interrupts configured */

#define MIN_IRQBUTTON   BUTTON_USER
#define MAX_IRQBUTTON   BUTTON_USER
#define NUM_IRQBUTTONS  1

#define GPIO_BTN_USER   (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTA|GPIO_PIN0)

/* PWM
 *
 * The STM32F429 Discovery has no real on-board PWM devices, but the board
 * can be configured to output a pulse train using TIM4 CH2 on PD13.
 */

#define STM32F429I_DISCO_PWMTIMER   4
#define STM32F429I_DISCO_PWMCHANNEL 2

/* SPI chip selects */

#define GPIO_CS_MEMS    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN1)
#define GPIO_CS_LCD     (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN2)
#define GPIO_LCD_DC     (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN13)
#define GPIO_LCD_ENABLE (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_CLEAR|GPIO_PORTF|GPIO_PIN10)
#define GPIO_CS_SST25   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN4)

/* USB OTG HS
 *
 * PA9  OTG_HS_VBUS VBUS sensing (also connected to the green LED)
 * PC0  OTG_HS_PowerSwitchOn
 * PD5  OTG_HS_Overcurrent
 */

#define GPIO_OTGHS_VBUS (GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_OPENDRAIN|GPIO_PORTB|GPIO_PIN13)
#define GPIO_OTGHS_PWRON (GPIO_OUTPUT|GPIO_OUTPUT_SET|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_PUSHPULL|GPIO_PORTC|GPIO_PIN4)

#ifdef CONFIG_USBHOST
#  define GPIO_OTGHS_OVER  (GPIO_INPUT|GPIO_EXTI|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_PUSHPULL|GPIO_PORTD|GPIO_PIN5)

#else
#  define GPIO_OTGHS_OVER  (GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_PUSHPULL|GPIO_PORTC|GPIO_PIN5)
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************

 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void);

/****************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the stm32f429i-disco
 *   board.
 *
 ****************************************************************************/

void weak_function stm32_spidev_initialize(void);

/****************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called from stm32_usbinitialize very early in inialization to setup USB-
 *   related GPIO pins for the STM32F429Discovery board.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_OTGHS
void weak_function stm32_usbinitialize(void);
#endif

/****************************************************************************
 * Name: stm32_usbhost_initialize
 *
 * Description:
 *   Called at application startup time to initialize the USB host
 *   functionality. This function will start a thread that will monitor for
 *   device connection/disconnection events.
 *
 ****************************************************************************/

#if defined(CONFIG_STM32_OTGHS) && defined(CONFIG_USBHOST)
int stm32_usbhost_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_tsc_setup
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

#ifdef CONFIG_INPUT_STMPE811
int stm32_tsc_setup(int minor);
#endif

/****************************************************************************
 * Name: stm32_sdram_initialize
 *
 * Description:
 *   Called from stm32_bringup to initialize external SDRAM access.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_FMC
void stm32_sdram_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_ledpminitialize
 *
 * Description:
 *   Enable logic to use the LEDs on the STM32F429Discovery to support
 *   power management testing
 *
 ****************************************************************************/

#ifdef CONFIG_PM
void stm32_ledpminitialize(void);
#endif

/****************************************************************************
 * Name: stm32_pmbuttons
 *
 * Description:
 *   Configure the user button of the STM32F429I-DISCO board as EXTI,
 *   so it is able to wakeup the MCU from the PM_STANDBY mode
 *
 ****************************************************************************/

#if defined(CONFIG_PM) && defined(CONFIG_ARCH_IDLE_CUSTOM) && \
    defined(CONFIG_PM_BUTTONS)
void stm32_pmbuttons(void);
#endif

/****************************************************************************
 * Name:  stm32_ili93414ws_initialize
 *
 * Description:
 *   Initialize the device structure to control the LCD Single chip driver.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   On success, this function returns a reference to the LCD control object
 *   for the specified ILI9341 LCD Single chip driver connected as 4 wire
 *   serial (spi). NULL is returned on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32F429I_DISCO_ILI9341
struct ili9341_lcd_s *stm32_ili93414ws_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_spi5initialize
 *
 * Description:
 *   Initialize the selected SPI port.
 *   As long as the method stm32_spibus_initialize() recognized the
 *   initialized state of the spi device by the spi enable flag of the cr1
 *   register, it isn't safe to disable the spi device outside of the nuttx
 *   spi interface structure. But this has to be done as long as the nuttx
 *   spi interface doesn't support bidirectional data transfer for multiple
 *   devices share one spi bus. This wrapper does nothing else than store
 *   the initialized state of the spi device after the first initializing
 *   and should be used by each driver who shares the spi5 bus.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_SPI5
struct spi_dev_s *stm32_spi5initialize(void);
#endif

/****************************************************************************
 * Name: stm32_l3gd20initialize()
 *
 * Description:
 *   Initialize and register the L3GD20 3 axis gyroscope sensor driver.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/gyro0"
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#if defined(CONFIG_SPI) & defined(CONFIG_SENSORS_L3GD20)
int stm32_l3gd20initialize(const char *devpath);
#endif

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
 * Name: stm32_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC device.
 *
 ****************************************************************************/

#ifdef CONFIG_ADC
int stm32_adc_setup(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32_STM32F429I_DISCO_SRC_STM32F429I_DISCO_H */
