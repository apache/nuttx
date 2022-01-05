/****************************************************************************
 * boards/arm/stm32/olimexino-stm32/src/olimexino-stm32.h
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

#ifndef __BOARDS_ARM_STM32_OLIMEXINO_STM32_SRC_OLIMEXINO_STM32_H
#define __BOARDS_ARM_STM32_OLIMEXINO_STM32_SRC_OLIMEXINO_STM32_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if STM32_NSPI < 1
#  undef CONFIG_STM32_SPI1
#  undef CONFIG_STM32_SPI2
#elif STM32_NSPI < 2
#  undef CONFIG_STM32_SPI2
#endif

/* LEDs *********************************************************************
 *
 *   GPIO      Function                               MPU        Board
 *                                                    Pin #      Name
 * -- ----- --------------------------------     ----------------------------
 *
 *  PA[05] PA5/SPI1_SCK/ADC5                          21       D13(SCK1/LED1)
 *  PA[01] PA1/USART2_RTS/ADC1/TIM2_CH2               15       D3(LED2)
 */

#define GPIO_LED1       (GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_50MHz | \
                         GPIO_PORTA | GPIO_PIN5 | GPIO_OUTPUT_CLEAR)
#define GPIO_LED_GREEN  GPIO_LED1
#define GPIO_LED2       (GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_50MHz | \
                         GPIO_PORTA | GPIO_PIN1 | GPIO_OUTPUT_CLEAR)
#define GPIO_LED_YELLOW GPIO_LED2

/* BUTTON *******************************************************************
 *
 *   GPIO      Function                               MPU        Board
 *                                                    Pin #      Name
 * -- ----- --------------------------------     ----------------------------
 *
 *  PC[09] PC9/TIM3_CH4                               40       BOOT0
 *
 */

#define BUTTON_BOOT0N   (GPIO_INPUT | GPIO_CNF_INFLOAT | GPIO_PORTC | GPIO_PIN9 | \
                         GPIO_EXTI)
#define IRQBUTTON       BUTTON_BOOT0_BIT

/* USBs *********************************************************************
 *
 *   GPIO      Function                               MPU        Board
 *                                                    Pin #      Name
 * -- ----- --------------------------------     ----------------------------
 *
 *  PC[11] PC11/USART3_RX                             52       USB_P
 *  PC[12] PC12/USART3_CK                             53       DISC
 *
 */

#define GPIO_USB_VBUS    (GPIO_INPUT | GPIO_CNF_INFLOAT | GPIO_PORTC | GPIO_PIN11)
#define GPIO_USB_PULLUPN (GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_50MHz | \
                          GPIO_PORTC | GPIO_PIN12 | GPIO_OUTPUT_SET)

/* SPI **********************************************************************
 *
 *   GPIO      Function                               MPU        Board
 *                                                    Pin #      Name
 * -- ----- --------------------------------     ----------------------------
 *
 *  PC[09] PA4/SPI1_NSS/USART2_CK/ADC4                20       D10(#SS1)
 *  PD[02] PD2/TIM3_ETR                               54       D25(MMC_CS)
 */

#define GPIO_SPI1_SSn (GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_50MHz | \
                       GPIO_PORTC | GPIO_PIN9 | GPIO_OUTPUT_SET)
#define USER_CSN      GPIO_SPI1_SSn

#define GPIO_SPI2_SSn (GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_50MHz | \
                       GPIO_PORTD | GPIO_PIN2 | GPIO_OUTPUT_SET)
#define MMCSD_CSN     GPIO_SPI2_SSn

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins.
 *
 ****************************************************************************/

#if defined(CONFIG_STM32_SPI1) || defined(CONFIG_STM32_SPI2) || \
    defined(CONFIG_STM32_SPI3)
void weak_function stm32_spidev_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called to setup USB-related GPIO pins.
 *
 ****************************************************************************/

void stm32_usbinitialize(void);

/****************************************************************************
 * Name: stm32_usb_set_pwr_callback()
 *
 * Description:
 *   Called to setup set a call back for USB power state changes.
 *
 * Input Parameters:
 *   pwr_changed_handler: An interrupt handler that will be called on VBUS
 *   power state changes.
 *
 ****************************************************************************/

void stm32_usb_set_pwr_callback(xcpt_t pwr_changed_handler);

/****************************************************************************
 * Name: stm32_led_initialize
 *
 * Description:
 *   This functions is called very early in initialization to perform board-
 *   specific initialization of LED-related resources.  This includes such
 *   things as, for example, configure GPIO pins to drive the LEDs and also
 *   putting the LEDs in their correct initial state.
 *
 *   NOTE: In most architectures, LED initialization() is called from
 *   board-specific initialization and should, therefore, have the name
 *   <arch>_led_initialize().  But there are a few architectures where the
 *   LED initialization function is still called from common chip
 *   architecture logic.  This interface is not, however, a common board
 *   interface in any event and the name board_led_initialization is
 *   deprecated.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
uint32_t stm32_led_initialize(void);
#endif

/****************************************************************************
 * Name: board_usbmsc_initialize
 *
 * Description:
 *   Called from the application system/usbmc or the boards_nsh if the
 *   application is not included.
 *   Perform architecture specific initialization.  This function must
 *   configure the block device to export via USB.  This function must be
 *   provided by architecture-specific logic in order to use this add-on.
 *
 ****************************************************************************/

#ifndef CONFIG_BOARDCTL_USBDEVCTRL
int board_usbmsc_initialize(int port);
#endif

/****************************************************************************
 * Name: stm32_can_setup
 *
 * Description:
 *  Initialize CAN and register the CAN device
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_CAN_CHARDRIVER
int stm32_can_setup(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32_OLIMEXINO_STM32_SRC_OLIMEXINO_STM32_H */
