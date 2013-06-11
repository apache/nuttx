/****************************************************************************************************
 * configs/mikroe-stm32f4/src/mikroe-stm32f4-internal.h
 * arch/arm/src/board/mikroe-stm32f4-internal.n
 *
 *   Copyright (C) 2011-2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ****************************************************************************************************/

#ifndef __CONFIGS_MIKROE_STM32F4_SRC_MIKROE_STM32F4_INTERNAL_H
#define __CONFIGS_MIKROE_STM32F4_SRC_MIKROE_STM32F4_INTERNAL_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/
/* Configuration ************************************************************************************/
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

/* Mikroe STM32F4 GPIOs **************************************************************************/
/* LEDs - There are no user LEDs on this board unless you add some manually. */

#define GPIO_LED1       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN12)

/* BUTTONS -- NOTE that all have EXTI interrupts configured */
/* There are no user buttons on this board unless you add some externally. */

#define MIN_IRQBUTTON   BUTTON_USER
#define MAX_IRQBUTTON   BUTTON_USER
#define NUM_IRQBUTTONS  1

#define GPIO_BTN_USER   (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTA|GPIO_PIN0)

/* PWM
 *
 * The Mikroe-STM32F4 has no real on-board PWM devices, but the board can be
 * configured to output a pulse train using TIM4 CH2 on PD13.
 */

#define STM32F4DISCOVERY_PWMTIMER   4
#define STM32F4DISCOVERY_PWMCHANNEL 2

/* SPI chip selects */

#define GPIO_CS_MMCSD   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN3)
#define GPIO_CS_FLASH   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN7)
#define GPIO_CS_MP3_DATA (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN9)
#define GPIO_CS_MP3_CMD (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN8)
#define GPIO_CS_EXP_SPI3 (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN0)
#define GPIO_SD_CD      (GPIO_INPUT|GPIO_PORTD|GPIO_PIN15)

/* USB OTG FS
 *
 * PA9  OTG_FS_VBUS VBUS sensing (also connected to the green LED)
 * PC0  OTG_FS_PowerSwitchOn
 * PD5  OTG_FS_Overcurrent
 */

#define GPIO_OTGFS_VBUS (GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_OPENDRAIN|GPIO_PORTA|GPIO_PIN9)
#define GPIO_OTGFS_PWRON (GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_PUSHPULL|GPIO_PORTC|GPIO_PIN0)

#ifdef CONFIG_USBHOST
#  define GPIO_OTGFS_OVER  (GPIO_INPUT|GPIO_EXTI|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_PUSHPULL|GPIO_PORTD|GPIO_PIN5)

#else
#  define GPIO_OTGFS_OVER  (GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_PUSHPULL|GPIO_PORTD|GPIO_PIN5)
#endif

/* TFT LCD Controller GPIOs
 *
 * PE8,  LCD_RST -- Low value holds in reset
 * PE15, LCD_CS  -- Low value selects LCD
 * PE9,  LCD_BLED -- Backlight -- Low value turns off
 * PE12, RS -- High values selects data
 *
 * PE10, PMPRD -- Low to read from the LCD
 * PE11, PMPWR -- Low to write to the LCD
 */

#define GPIO_LCD_RST    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_PORTE|GPIO_PIN8)

#define GPIO_LCD_CS     (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                           GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN15)
#define LCD_CS_PIN      GPIO_PIN15

#define GPIO_LCD_BLED   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_PORTE|GPIO_PIN9)

#define GPIO_LCD_RS     (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_PORTE|GPIO_PIN12)
#define LCD_RS_PIN      GPIO_PIN12

#define GPIO_LCD_PMPRD  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN10)
#define LCD_PMPRD_PIN   GPIO_PIN10

#define GPIO_LCD_PMPWR  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN11)
#define LCD_PMPWR_PIN   GPIO_PIN11

#define GPIO_LCD_T_D0   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_PORTE|GPIO_PIN0)

#define GPIO_LCD_T_D1   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_PORTE|GPIO_PIN1)

#define GPIO_LCD_T_D2   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_PORTE|GPIO_PIN2)

#define GPIO_LCD_T_D3   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_PORTE|GPIO_PIN3)

#define GPIO_LCD_T_D4   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_PORTE|GPIO_PIN4)

#define GPIO_LCD_T_D5   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_PORTE|GPIO_PIN5)

#define GPIO_LCD_T_D6   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_PORTE|GPIO_PIN6)

#define GPIO_LCD_T_D7   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_PORTE|GPIO_PIN7)

#define GPIO_TP_DRIVEA  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN8)

#define GPIO_TP_DRIVEB  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_PORTB|GPIO_PIN9)

#define GPIO_TP_YD      (GPIO_ANALOG|GPIO_PORTB|GPIO_PIN0)

#define GPIO_TP_XL      (GPIO_ANALOG|GPIO_PORTB|GPIO_PIN1)

/* MP3 Codec control pins */

#define GPIO_VS1053_RST (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN7)
#define GPIO_VS1053_DREQ (GPIO_INPUT|GPIO_SPEED_50MHz|GPIO_PORTC|GPIO_PIN6)

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

/****************************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the mikroe-stm32f4 board.
 *
 ****************************************************************************************************/

void weak_function stm32_spiinitialize(void);

/****************************************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called from stm32_usbinitialize very early in inialization to setup USB-related
 *   GPIO pins for the Mikroe-stm32f4 board.
 *
 ****************************************************************************************************/

#ifdef CONFIG_STM32_OTGFS
void weak_function stm32_usbinitialize(void);
#endif

/****************************************************************************************************
 * Name: stm32_usbhost_initialize
 *
 * Description:
 *   Called at application startup time to initialize the USB host functionality. This function will
 *   start a thread that will monitor for device connection/disconnection events.
 *
 ****************************************************************************************************/

#if defined(CONFIG_STM32_OTGFS) && defined(CONFIG_USBHOST)
#  error "The Mikroe-STM32F4 board does not support HOST OTG, only device!"
#endif

/****************************************************************************************************
 * Name: stm32_lcdinitialize
 *
 * Description:
 *   Initialize the LCD.  This function should be called early in the boot sequendce -- Even if the
 *   LCD is not enabled.  In that case we should at a minimum at least disable the LCD backlight.
 *
 ****************************************************************************************************/

#ifdef CONFIG_LCD_MIO283QT2
void stm32_lcdinitialize(void);
#endif

/****************************************************************************************************
 * Name:  up_lcdinitialize
 *
 * Description:
 *   Initialize the LCD video hardware.  The initial state of the LCD is fully initialized, display
 *   memory cleared, and the LCD ready to use, but with the power setting at 0 (full off).
 *
 ****************************************************************************************************/

#ifdef CONFIG_LCD_MIO283QT2
int up_lcdinitialize(void);
#endif

/****************************************************************************************************
 * Name:  up_vs1053initialize
 *
 * Description:
 *   Initialize the VS1053 Audio CODEC hardware.
 *
 ****************************************************************************************************/

#ifdef CONFIG_VS1053
void up_vs1053initialize(FAR struct spi_dev_s *spi);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_MIKROE_STM32F4_SRC_MIKROE_STM32F4_INTERNAL_H */
