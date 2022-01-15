/****************************************************************************
 * boards/arm/stm32/hymini-stm32v/src/hymini-stm32v.h
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

#ifndef __BOARDS_ARM_STM32_HYMINI_STM32V_SRC_HYMINI_STM32V_H
#define __BOARDS_ARM_STM32_HYMINI_STM32V_SRC_HYMINI_STM32V_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>

#include <arch/stm32/chip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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

#define GPIO_LED1       (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN0)
#define GPIO_LED2       (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN1)

/* BUTTONS -- NOTE that some have EXTI interrupts configured */

#define MIN_IRQBUTTON   BUTTON_KEYA
#define MAX_IRQBUTTON   BUTTON_KEYB
#define NUM_IRQBUTTONS  NUM_BUTTONS

/* Button A is externally pulled up */

#define GPIO_BTN_KEYA (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|\
                         GPIO_PORTC|GPIO_PIN13)

/* Button B is externally pulled dw */

#define GPIO_BTN_KEYB (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|\
                         GPIO_PORTB|GPIO_PIN2)

/* SPI touch screen (ADS7843) chip select:  PA.4 */

#define GPIO_TS_CS   (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN4)

/* Touch screen (ADS7843) IRQ pin:  PB.6 */

#define GPIO_TS_IRQ  (GPIO_INPUT|GPIO_CNF_INPULLUP|GPIO_MODE_INPUT|\
                         GPIO_PORTB|GPIO_PIN6)

/* USB Soft Connect Pullup: PB.7 */

#define GPIO_USB_PULLUP (GPIO_OUTPUT|GPIO_CNF_OUTOD|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN7)

/* SD card detect pin: PD.3   (line is pulled up on board) */

#define GPIO_SD_CD (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|\
                         GPIO_PORTD|GPIO_PIN3)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the Hy-Mini STM32v
 *   board.
 *
 ****************************************************************************/

void weak_function stm32_spidev_initialize(void);

/****************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called to setup USB-related GPIO pins for the Hy-Mini STM32v board.
 *
 ****************************************************************************/

void weak_function stm32_usbinitialize(void);

/****************************************************************************
 * Name: stm32_tsc_setup
 *
 * Description:
 *   This function is called by board-bringup logic to configure the
 *   touchscreen device.
 *  This function will register the driver as /dev/inputN where N is the
 *   minor device number.
 *
 * Input Parameters:
 *   minor   - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_INPUT
int stm32_tsc_setup(int minor);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32_HYMINI_STM32V_SRC_HYMINI_STM32V_H */
