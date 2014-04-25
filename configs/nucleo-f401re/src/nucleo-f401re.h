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

/* LED.  User LD2: the green LED is a user LED connected to Arduino signal D13
 * corresponding to MCU I/O PA5 (pin 21) or PB13 (pin 34) depending on the STM32
 * target.
 *
 * - When the I/O is HIGH value, the LED is on.
 * - When the I/O is LOW, the LED is off.
 */

#define GPIO_LD2      (GPIO_PORTA | GPIO_PIN13 | GPIO_OUTPUT_CLEAR | GPIO_OUTPUT | GPIO_PULLUP | GPIO_SPEED_50MHz)

/* Buttons
 *
 * B1 USER: the user button is connected to the I/O PC13 (pin 2) of the STM32
 * microcontroller.
 */

#define MIN_IRQBUTTON   BUTTON_USER
#define MAX_IRQBUTTON   BUTTON_USER
#define NUM_IRQBUTTONS  1

#define GPIO_BTN_USER   (GPIO_INPUT |GPIO_FLOAT |GPIO_EXTI | GPIO_PORTC | GPIO_PIN13)

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
 */

#define GPIO_WIFI_INT (GPIO_PORTB | GPIO_PIN3  | GPIO_INPUT        | GPIO_PULLUP | GPIO_EXTI)
#define GPIO_WIFI_EN  (GPIO_PORTB | GPIO_PIN4  | GPIO_OUTPUT_CLEAR | GPIO_OUTPUT | GPIO_PULLUP | GPIO_SPEED_50MHz)
#define GPIO_WIFI_CS  (GPIO_PORTB | GPIO_PIN6  | GPIO_OUTPUT_SET   | GPIO_OUTPUT | GPIO_PULLUP | GPIO_SPEED_50MHz)

#if defined(CONFIG_CC3000_PROBES)
#  define GPIO_D14    (GPIO_PORTB | GPIO_PIN9  | GPIO_OUTPUT_CLEAR | GPIO_OUTPUT | GPIO_PULLUP | GPIO_SPEED_50MHz)
#  define GPIO_D15    (GPIO_PORTB | GPIO_PIN8  | GPIO_OUTPUT_CLEAR | GPIO_OUTPUT | GPIO_PULLUP | GPIO_SPEED_50MHz)
#else
#  define GPIO_D0     (GPIO_PORTA | GPIO_PIN3  | GPIO_INPUT        | GPIO_PULLUP | GPIO_EXTI)
#  define GPIO_D1     (GPIO_PORTA | GPIO_PIN2  | GPIO_INPUT        | GPIO_PULLUP | GPIO_EXTI)
#  define GPIO_D2     (GPIO_PORTA | GPIO_PIN10 | GPIO_OUTPUT_CLEAR | GPIO_OUTPUT | GPIO_PULLUP | GPIO_SPEED_50MHz)

#  define GPIO_A0     (GPIO_PORTA | GPIO_PIN0  | GPIO_OUTPUT_SET   | GPIO_OUTPUT | GPIO_PULLUP | GPIO_SPEED_50MHz)
#  define GPIO_A1     (GPIO_PORTA | GPIO_PIN1  | GPIO_OUTPUT_SET   | GPIO_OUTPUT | GPIO_PULLUP | GPIO_SPEED_50MHz)
#  define GPIO_A2     (GPIO_PORTA | GPIO_PIN4  | GPIO_INPUT        | GPIO_PULLUP )
#  define GPIO_A3     (GPIO_PORTB | GPIO_PIN0  | GPIO_INPUT        | GPIO_PULLUP )
#endif

/* SPI1 off */

#define GPIO_SPI1_MOSI_OFF      (GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTA|GPIO_PIN7)
#define GPIO_SPI1_MISO_OFF      (GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTA|GPIO_PIN6)
#define GPIO_SPI1_SCK_OFF       (GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTA|GPIO_PIN5)

/* SPI1 chip selects off */

#define GPIO_SPI_CS_WIFI_OFF \
  (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_2MHz|GPIO_PORTB|GPIO_PIN6)
#define GPIO_SPI_CS_SD_CARD_OFF \
  (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_2MHz|GPIO_PORTB|GPIO_PIN5)
#define GPIO_SPI_CS_FRAM \
  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN10)

/* SPI chip selects */

#define GPIO_SPI_CS_WIFI \
  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN13)
#define GPIO_SPI_CS_SD_CARD \
  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN15)

/* Devices on the onboard bus.
 *
 * Note that these are unshifted addresses.
 */

#define NUCLEO_I2C_OBDEV_LED       0x55
#define NUCLEO_I2C_OBDEV_HMC5883   0x1e

/* User GPIOs
 *
 * GPIO0-1 are for probing WIFI status
 */

#define GPIO_GPIO0_INPUT        (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN8)
#define GPIO_GPIO1_INPUT        (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN9)
#define GPIO_GPIO0_OUTPUT \
  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN8)
#define GPIO_GPIO1_OUTPUT \
  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN9)

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

/****************************************************************************
 * Name: board_led_initialize
 *
 * Description:
 *   Initialize LED GPIO outputs
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void board_led_initialize(void);
#endif

#endif /* __CONFIGS_NUCLEO_F401RE_SRC_NUCLEO_F401RE_H */
