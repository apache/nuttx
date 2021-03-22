/****************************************************************************
 * boards/arm/stm32/olimexino-stm32/src/olimexino-stm32.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           David_s5 <david_s5@nscdg.com>
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

#ifdef CONFIG_CAN
int stm32_can_setup(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32_OLIMEXINO_STM32_SRC_OLIMEXINO_STM32_H */
