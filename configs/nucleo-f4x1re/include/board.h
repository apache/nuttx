/************************************************************************************
 * configs/nucleo-f4x1re/include/board.h
 *
 *   Copyright (C) 2014, 2016 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __CONFIGS_NUCLEO_F401RE_INCLUDE_BOARD_H
#define __CONFIGS_NUCLEO_F401RE_INCLUDE_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

#include <stm32.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/

#if defined(CONFIG_ARCH_CHIP_STM32F401RE)
#  include <arch/board/nucleo-f401re.h>
#elif defined(CONFIG_ARCH_CHIP_STM32F411RE)
#  include <arch/board/nucleo-f411re.h>
#endif

/* DMA Channel/Stream Selections ****************************************************/
/* Stream selections are arbitrary for now but might become important in the future
 * is we set aside more DMA channels/streams.
 *
 * SDIO DMA
 *   DMAMAP_SDIO_1 = Channel 4, Stream 3 <- may later be used by SPI DMA
 *   DMAMAP_SDIO_2 = Channel 4, Stream 6
 */

#define DMAMAP_SDIO DMAMAP_SDIO_1

/* Need to VERIFY fwb */

#define DMACHAN_SPI1_RX DMAMAP_SPI1_RX_1
#define DMACHAN_SPI1_TX DMAMAP_SPI1_TX_1
#define DMACHAN_SPI2_RX DMAMAP_SPI2_RX
#define DMACHAN_SPI2_TX DMAMAP_SPI2_TX

/* Alternate function pin selections ************************************************/

/* USART1:
 *   RXD: PA10  CN9 pin 3, CN10 pin 33
 *        PB7   CN7 pin 21
 *   TXD: PA9   CN5 pin 1, CN10 pin 21
 *        PB6   CN5 pin 3, CN10 pin 17
 */

#if 1
#  define GPIO_USART1_RX GPIO_USART1_RX_1    /* PA10 */
#  define GPIO_USART1_TX GPIO_USART1_TX_1    /* PA9  */
#else
#  define GPIO_USART1_RX GPIO_USART1_RX_2    /* PB7 */
#  define GPIO_USART1_TX GPIO_USART1_TX_2    /* PB6  */
#endif

/* USART2:
 *   RXD: PA3   CN9 pin 1 (See SB13, 14, 62, 63). CN10 pin 37
 *        PD6
 *   TXD: PA2   CN9 pin 2(See SB13, 14, 62, 63). CN10 pin 35
 *        PD5
 */

#define GPIO_USART2_RX   GPIO_USART2_RX_1    /* PA3 */
#define GPIO_USART2_TX   GPIO_USART2_TX_1    /* PA2 */
#define GPIO_USART2_RTS  GPIO_USART2_RTS_2
#define GPIO_USART2_CTS  GPIO_USART2_CTS_2

/* USART6:
 *  RXD: PC7    CN5 pin2, CN10 pin 19
 *       PA12   CN10, pin 12
 *  TXD: PC6    CN10, pin 4
 *       PA11   CN10, pin 14
 */

#define GPIO_USART6_RX   GPIO_USART6_RX_1    /* PC7 */
#define GPIO_USART6_TX   GPIO_USART6_TX_1    /* PC6 */

/* UART RX DMA configurations */

#define DMAMAP_USART1_RX DMAMAP_USART1_RX_2
#define DMAMAP_USART6_RX DMAMAP_USART6_RX_2

/* I2C
 *
 * The optional _GPIO configurations allow the I2C driver to manually
 * reset the bus to clear stuck slaves.  They match the pin configuration,
 * but are normally-high GPIOs.
 */

#define GPIO_I2C1_SCL    GPIO_I2C1_SCL_2
#define GPIO_I2C1_SDA    GPIO_I2C1_SDA_2
#define GPIO_I2C1_SCL_GPIO \
   (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN8)
#define GPIO_I2C1_SDA_GPIO \
   (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN9)

#define GPIO_I2C2_SCL    GPIO_I2C2_SCL_1
#define GPIO_I2C2_SDA    GPIO_I2C2_SDA_1
#define GPIO_I2C2_SCL_GPIO \
   (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN10)
#define GPIO_I2C2_SDA_GPIO \
   (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN11)

/* SPI
 *
 * There are sensors on SPI1, and SPI2 is connected to the FRAM.
 */

#define GPIO_SPI1_MISO   GPIO_SPI1_MISO_1
#define GPIO_SPI1_MOSI   GPIO_SPI1_MOSI_1
#define GPIO_SPI1_SCK    GPIO_SPI1_SCK_1

#define GPIO_SPI2_MISO   GPIO_SPI2_MISO_1
#define GPIO_SPI2_MOSI   GPIO_SPI2_MOSI_1
#define GPIO_SPI2_SCK    GPIO_SPI2_SCK_2

/* LEDs
 *
 * The Nucleo F401RE and F411RE boards provide a single user LED, LD2.  LD2
 * is the green LED connected to Arduino signal D13 corresponding to MCU I/O
 * PA5 (pin 21) or PB13 (pin 34) depending on the STM32 target.
 *
 *   - When the I/O is HIGH value, the LED is on.
 *   - When the I/O is LOW, the LED is off.
 */

/* LED index values for use with board_userled() */

#define BOARD_LD2         0
#define BOARD_NLEDS       1

/* LED bits for use with board_userled_all() */

#define BOARD_LD2_BIT     (1 << BOARD_LD2)

/* These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/sam_leds.c. The LEDs are used to encode OS-related
 * events as follows when the red LED (PE24) is available:
 *
 *   SYMBOL                Meaning                   LD2
 *   -------------------  -----------------------  -----------
 *   LED_STARTED          NuttX has been started     OFF
 *   LED_HEAPALLOCATE     Heap has been allocated    OFF
 *   LED_IRQSENABLED      Interrupts enabled         OFF
 *   LED_STACKCREATED     Idle stack created         ON
 *   LED_INIRQ            In an interrupt            No change
 *   LED_SIGNAL           In a signal handler        No change
 *   LED_ASSERTION        An assertion failed        No change
 *   LED_PANIC            The system has crashed     Blinking
 *   LED_IDLE             MCU is is sleep mode       Not used
 *
 * Thus if LD2, NuttX has successfully booted and is, apparently, running
 * normally.  If LD2 is flashing at approximately 2Hz, then a fatal error
 * has been detected and the system has halted.
 */

#define LED_STARTED      0
#define LED_HEAPALLOCATE 0
#define LED_IRQSENABLED  0
#define LED_STACKCREATED 1
#define LED_INIRQ        2
#define LED_SIGNAL       2
#define LED_ASSERTION    2
#define LED_PANIC        1

/* Buttons
 *
 *   B1 USER: the user button is connected to the I/O PC13 (pin 2) of the STM32
 *   microcontroller.
 */

#define BUTTON_USER        0
#define NUM_BUTTONS        1

#define BUTTON_USER_BIT    (1 << BUTTON_USER)

#define GPIO_TIM2_CH1IN (GPIO_TIM2_CH1IN_1 | GPIO_PULLUP)
#define GPIO_TIM2_CH2IN (GPIO_TIM2_CH2IN_1 | GPIO_PULLUP)

#endif  /* __CONFIGS_NUCLEO_F401RE_INCLUDE_BOARD_H */
