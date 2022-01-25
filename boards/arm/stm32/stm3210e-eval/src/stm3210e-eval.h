/****************************************************************************
 * boards/arm/stm32/stm3210e-eval/src/stm3210e-eval.h
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

#ifndef __BOARDS_ARM_STM32_STM3210E_EVAL_SRC_STM3210E_EVAL_H
#define __BOARDS_ARM_STM32_STM3210E_EVAL_SRC_STM3210E_EVAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>

#include <arch/stm32/chip.h>

#include "stm32_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* How many SPI modules does this chip support?
 * Most support 2 SPI modules (others may support more -- in such case,
 * the following must be expanded).
 */

#if STM32_NSPI < 1
#  undef CONFIG_STM32_SPI1
#  undef CONFIG_STM32_SPI2
#elif STM32_NSPI < 2
#  undef CONFIG_STM32_SPI2
#endif

/* There is only CAN1 on the STM3210E-EVAL board */

#if defined(CONFIG_STM32_CAN2)
#  warning "The STM3210E-EVAL only supports CAN1"
#endif

/* Do we need to register I2C drivers on behalf of the I2C tool? */

#define HAVE_I2CTOOL 1
#if !defined(CONFIG_SYSTEM_I2CTOOL) || !defined(CONFIG_I2C_DRIVER)
#  undef HAVE_I2CTOOL
#endif

/* STM3210E-EVAL GPIOs ******************************************************/

/* LEDs */

#define GPIO_LED1        (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                          GPIO_OUTPUT_CLEAR|GPIO_PORTF|GPIO_PIN6)
#define GPIO_LED2        (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                          GPIO_OUTPUT_CLEAR|GPIO_PORTF|GPIO_PIN7)
#define GPIO_LED3        (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                          GPIO_OUTPUT_CLEAR|GPIO_PORTF|GPIO_PIN8)
#define GPIO_LED4        (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                          GPIO_OUTPUT_CLEAR|GPIO_PORTF|GPIO_PIN9)

/* BUTTONS -- NOTE that some have EXTI interrupts configured */

#ifdef CONFIG_INPUT_DJOYSTICK
#  define MIN_IRQBUTTON  BUTTON_KEY
#  define MAX_IRQBUTTON  BUTTON_KEY
#  define NUM_IRQBUTTONS (1)
#else
#  define MIN_IRQBUTTON  BUTTON_KEY
#  define MAX_IRQBUTTON  JOYSTICK_UP
#  define NUM_IRQBUTTONS (JOYSTICK_UP - BUTTON_KEY + 1)
#endif

#define GPIO_BTN_WAKEUP  (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|\
                          GPIO_PORTA|GPIO_PIN0)
#define GPIO_BTN_TAMPER  (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|\
                          GPIO_PORTC|GPIO_PIN13)
#define GPIO_BTN_KEY     (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|\
                          GPIO_EXTI|GPIO_PORTG|GPIO_PIN8)

#define GPIO_JOY_SEL     (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|\
                          GPIO_EXTI|GPIO_PORTG|GPIO_PIN7)
#define GPIO_JOY_DOWN    (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|\
                          GPIO_EXTI|GPIO_PORTD|GPIO_PIN3)
#define GPIO_JOY_LEFT    (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|\
                          GPIO_EXTI|GPIO_PORTG|GPIO_PIN14)
#define GPIO_JOY_RIGHT   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|\
                          GPIO_EXTI|GPIO_PORTG|GPIO_PIN13)
#define GPIO_JOY_UP      (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|\
                          GPIO_EXTI|GPIO_PORTG|GPIO_PIN15)

/* SPI FLASH chip select:  PA.4 */

#define GPIO_FLASH_CS    (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                          GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN2)

/* Backlight control: PA.8
 *
 * If CONFIG_LCD_TIM1 (and CONFIG_STM32_TIM1) is defined, PA.8 will be
 * configured as CH1OUT for variable backlight control.  Otherwise, the
 * following definition will be used to support a discrete backlight control.
 */

#define GPIO_LCD_BACKLIGHT (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                             GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN8)

/* USB Soft Connect Pullup: PB.14 */

#define GPIO_USB_PULLUP (GPIO_OUTPUT|GPIO_CNF_OUTOD|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN14)

/* LM-75 Temperature Sensor: PB.5 */

#define GPIO_LM75_OSINT (GPIO_INPUT|GPIO_CNF_INPULLUP|GPIO_MODE_10MHz|\
                         GPIO_EXTI|GPIO_PORTB|GPIO_PIN5)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* GPIO settings that will be altered when external memory is selected */

struct extmem_save_s
{
  uint32_t gpiod_crl;
  uint32_t gpiod_crh;
  uint32_t gpioe_crl;
  uint32_t gpioe_crh;
  uint32_t gpiof_crl;
  uint32_t gpiof_crh;
  uint32_t gpiog_crl;
  uint32_t gpiog_crh;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* GPIO configurations common to SRAM and NOR Flash */

#define NCOMMON_CONFIG 37
extern const uint16_t g_commonconfig[NCOMMON_CONFIG];

/****************************************************************************
 * Public Functions Definitions
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
 *   Called to configure SPI chip select GPIO pins for the STM3210E-EVAL
 *   board.
 *
 ****************************************************************************/

void weak_function stm32_spidev_initialize(void);

/****************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called to setup USB-related GPIO pins for the STM3210E-EVAL board.
 *
 ****************************************************************************/

void weak_function stm32_usbinitialize(void);

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
 * Name: stm32_can_setup
 *
 * Description:
 *  Initialize CAN and register the CAN device
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_CAN_CHARDRIVER
int stm32_can_setup(void);
#endif

/****************************************************************************
 * Name: stm32_extcontextsave
 *
 * Description:
 *  Save current GPIOs that will used by external memory configurations
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_FSMC
void stm32_extcontextsave(struct extmem_save_s *save);

/****************************************************************************
 * Name: stm32_extcontextrestore
 *
 * Description:
 *  Restore GPIOs that were used by external memory configurations
 *
 ****************************************************************************/

void stm32_extcontextrestore(struct extmem_save_s *restore);

/****************************************************************************
 * Name: stm32_extmemgpios
 *
 * Description:
 *   Initialize GPIOs for NOR or SRAM
 *
 ****************************************************************************/

void stm32_extmemgpios(const uint16_t *gpios, int ngpios);

/****************************************************************************
 * Name: stm32_selectnor
 *
 * Description:
 *   Initialize to access NOR flash
 *
 ****************************************************************************/

void stm32_selectnor(void);

/****************************************************************************
 * Name: stm32_deselectnor
 *
 * Description:
 *   Disable NOR FLASH
 *
 ****************************************************************************/

void stm32_deselectnor(void);

/****************************************************************************
 * Name: stm32_selectsram
 *
 * Description:
 *   Initialize to access external SRAM
 *
 ****************************************************************************/

void stm32_selectsram(void);

/****************************************************************************
 * Name: stm32_deselectsram
 *
 * Description:
 *   Disable external SRAM
 *
 ****************************************************************************/

void stm32_deselectsram(void);

/****************************************************************************
 * Name: stm32_selectlcd
 *
 * Description:
 *   Initialize to the LCD
 *
 ****************************************************************************/

void stm32_selectlcd(void);

/****************************************************************************
 * Name: stm32_deselectlcd
 *
 * Description:
 *   Disable the LCD
 *
 ****************************************************************************/

void stm32_deselectlcd(void);

#endif /* CONFIG_STM32_FSMC */

/****************************************************************************
 * Name: stm32_ledpminitialize
 *
 * Description:
 *   Register the LEDs to receive power management event callbacks
 *
 ****************************************************************************/

#if defined(CONFIG_PM) && defined(CONFIG_ARCH_LEDS)
void stm32_ledpminitialize(void);
#endif

/****************************************************************************
 * Name: stm32_pmbuttons
 *
 * Description:
 *   Configure all the buttons of the STM3210e-eval board as EXTI, so any
 *   button is able to wakeup the MCU from the PM_STANDBY mode
 *
 ****************************************************************************/

#if defined(CONFIG_PM) && defined(CONFIG_ARCH_IDLE_CUSTOM) && defined(CONFIG_PM_BUTTONS)
void stm32_pmbuttons(void);
#endif

/****************************************************************************
 * Name: stm32_djoy_initialization
 *
 * Description:
 *   Initialize and register the discrete joystick driver
 *
 ****************************************************************************/

#ifdef CONFIG_INPUT_DJOYSTICK
int stm32_djoy_initialization(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32_STM3210E_EVAL_SRC_STM3210E_EVAL_H */
