/************************************************************************************
 * configs/nucleo-f401re/src/nucleo-f401re.h
 *
 ************************************************************************************/

#ifndef __CONFIGS_NUCLEO_F401RE_SRC_NUCLEO_F401RE_H
#define __CONFIGS_NUCLEO_F401RE_SRC_NUCLEO_F401RE_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/* The shield uses the following pins:
 *
 *   +5V
 *   GND
 *   Digital pin  3: IRQ for WiFi
 *   Digital pin  4: Card Select for SD card
 *   Digital pin  5: WiFi enable
 *   Digital pin 10: Chip Select for WiFi
 *   Digital pins 11, 12, 13 for SPI communication (both WiFi and SD). 
 *   Digital pin 11:
 *   Digital pin 12:
 *   Digital pin 13:
 *   Even if optional 6-pin SPI header is used, these pins are unavailable for other use.
 *
 * LED.  User LD2: the green LED is a user LED connected to Arduino signal D13
 * corresponding to MCU I/O PA5 (pin 21) or PB13 (pin 34) depending on the STM32
 * target.
 *
 * - When the I/O is HIGH value, the LED is on.
 * - When the I/O is LOW, the LED is off.
 */

#define GPIO_WIFI_INT (GPIO_PORTB | GPIO_PIN3  | GPIO_INPUT        | GPIO_PULLUP | GPIO_EXTI)
#define GPIO_WIFI_EN  (GPIO_PORTB | GPIO_PIN4  | GPIO_OUTPUT_CLEAR | GPIO_OUTPUT | GPIO_PULLUP | GPIO_SPEED_50MHz)
#define GPIO_WIFI_CS  (GPIO_PORTB | GPIO_PIN6  | GPIO_OUTPUT_SET   | GPIO_OUTPUT | GPIO_PULLUP | GPIO_SPEED_50MHz)

#define GPIO_LD2      (GPIO_PORTA | GPIO_PIN13 | GPIO_OUTPUT_CLEAR | GPIO_OUTPUT | GPIO_PULLUP | GPIO_SPEED_50MHz)

#if defined(CONFIG_CC3000_PROBES)
#  define GPIO_D0     (GPIO_PORTB | GPIO_PIN7  | GPIO_OUTPUT_CLEAR | GPIO_OUTPUT | GPIO_PULLUP | GPIO_SPEED_50MHz)
#  define GPIO_D1     (GPIO_PORTB | GPIO_PIN6  | GPIO_OUTPUT_CLEAR | GPIO_OUTPUT | GPIO_PULLUP | GPIO_SPEED_50MHz)
#else
#  define GPIO_D0     (GPIO_PORTA | GPIO_PIN3  | GPIO_INPUT        | GPIO_PULLUP | GPIO_EXTI)
#  define GPIO_D1     (GPIO_PORTA | GPIO_PIN2  | GPIO_INPUT        | GPIO_PULLUP | GPIO_EXTI)
#  define GPIO_D2     (GPIO_PORTA | GPIO_PIN10 | GPIO_OUTPUT_CLEAR | GPIO_OUTPUT | GPIO_PULLUP | GPIO_SPEED_50MHz)

#  define GPIO_D8     (GPIO_PORTA | GPIO_PIN9  | GPIO_OUTPUT_CLEAR | GPIO_OUTPUT | GPIO_PULLUP | GPIO_SPEED_50MHz)

#  define GPIO_A0     (GPIO_PORTA | GPIO_PIN0  | GPIO_OUTPUT_SET   | GPIO_OUTPUT | GPIO_PULLUP | GPIO_SPEED_50MHz)
#  define GPIO_A1     (GPIO_PORTA | GPIO_PIN1  | GPIO_OUTPUT_SET   | GPIO_OUTPUT | GPIO_PULLUP | GPIO_SPEED_50MHz)
#  define GPIO_A2     (GPIO_PORTA | GPIO_PIN4  | GPIO_INPUT        | GPIO_PULLUP )
#  define GPIO_A3     (GPIO_PORTB | GPIO_PIN0  | GPIO_INPUT        | GPIO_PULLUP )
#endif

#ifndef __ASSEMBLY__

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins.
 *
 ************************************************************************************/

void stm32_spiinitialize(void);

/************************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called to setup USB-related GPIO pins.
 *
 ************************************************************************************/

void stm32_usbinitialize(void);

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_NUCLEO_F401RE_SRC_NUCLEO_F401RE_H */
