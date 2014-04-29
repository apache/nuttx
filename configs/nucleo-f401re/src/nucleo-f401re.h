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

#define GPIO_LD2      (GPIO_PORTA | GPIO_PIN5 | GPIO_OUTPUT_CLEAR | GPIO_OUTPUT | GPIO_PULLUP | GPIO_SPEED_50MHz)

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
 *  SERIAL_TX=PA_2    USER_BUTTON=PC_13
 *  SERIAL_RX=PA_3            LD2=PA_5
 *
 * Analog                         Digital
 *  A0=PA_0    USART2RX D0=PA_3              D8 =PA_9
 *  A1=PA_1    USART2TX D1=PA_2              D9 =PC_7
 *  A2=PA_4             D2=PA_10     WIFI_CS=D10=PB_6 SPI_CS
 *  A3=PB_0    WIFI_INT=D3=PB_3              D11=PA_7 SPI_MOSI
 *  A4=PC_1       SD_CS=D4=PB_5              D12=PA_6 SPI_MISO
 *  A5=PC_0     WIFI_EN=D5=PB_4          LD2=D13=PA_5 SPI_SCK
 *                 LED2=D6=PB_10    I2C1_SDA=D14=PB_9 WIFI Probe
 *                      D7=PA_8     I2C1_SCL=D15=PB_8 WIFI Probe
 *
 *  mostly from: https://mbed.org/platforms/ST-Nucleo-F401RE/
 *
 */

#define GPIO_WIFI_INT (GPIO_PORTB | GPIO_PIN3  | GPIO_INPUT        | GPIO_PULLUP | GPIO_EXTI)
#define GPIO_WIFI_EN  (GPIO_PORTB | GPIO_PIN4  | GPIO_OUTPUT_CLEAR | GPIO_OUTPUT | GPIO_PULLUP | GPIO_SPEED_50MHz)
#define GPIO_WIFI_CS  (GPIO_PORTB | GPIO_PIN6  | GPIO_OUTPUT_SET   | GPIO_OUTPUT | GPIO_PULLUP | GPIO_SPEED_50MHz)

#  define GPIO_D14    (GPIO_PORTB | GPIO_PIN9  | GPIO_OUTPUT_CLEAR | GPIO_OUTPUT | GPIO_PULLUP | GPIO_SPEED_50MHz)
#  define GPIO_D15    (GPIO_PORTB | GPIO_PIN8  | GPIO_OUTPUT_CLEAR | GPIO_OUTPUT | GPIO_PULLUP | GPIO_SPEED_50MHz)
#  define GPIO_D0     (GPIO_PORTA | GPIO_PIN3  | GPIO_INPUT        | GPIO_PULLUP )
#  define GPIO_D1     (GPIO_PORTA | GPIO_PIN2  | GPIO_OUTPUT_CLEAR | GPIO_PULLUP )
#  define GPIO_D2     (GPIO_PORTA | GPIO_PIN10 | GPIO_OUTPUT_CLEAR | GPIO_OUTPUT | GPIO_PULLUP | GPIO_SPEED_50MHz)

#  define GPIO_A0     (GPIO_PORTA | GPIO_PIN0  | GPIO_OUTPUT_SET   | GPIO_OUTPUT | GPIO_PULLUP | GPIO_SPEED_50MHz)
#  define GPIO_A1     (GPIO_PORTA | GPIO_PIN1  | GPIO_OUTPUT_SET   | GPIO_OUTPUT | GPIO_PULLUP | GPIO_SPEED_50MHz)
#  define GPIO_A2     (GPIO_PORTA | GPIO_PIN4  | GPIO_INPUT        | GPIO_PULLUP )
#  define GPIO_A3     (GPIO_PORTB | GPIO_PIN0  | GPIO_INPUT        | GPIO_PULLUP )

/* SPI1 off */

#define GPIO_SPI1_MOSI_OFF      (GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTA|GPIO_PIN7)
#define GPIO_SPI1_MISO_OFF      (GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTA|GPIO_PIN6)
#define GPIO_SPI1_SCK_OFF       (GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTA|GPIO_PIN5)

/* SPI1 chip selects off */

#define GPIO_SPI_CS_WIFI_OFF \
  (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_2MHz|GPIO_PORTB|GPIO_PIN6)
#define GPIO_SPI_CS_SD_CARD_OFF \
  (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_2MHz|GPIO_PORTB|GPIO_PIN5)

/* SPI chip selects */

#define GPIO_SPI_CS_WIFI \
  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN6)
#define GPIO_SPI_CS_SD_CARD \
  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN5)

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
