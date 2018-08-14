/************************************************************************************
 * configs/stm32l4r9ai-disco/include/board.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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

#ifndef __CONFIGS_STM32L4R9AI_DISCO_INCLUDE_BOARD_H
#define __CONFIGS_STM32L4R9AI_DISCO_INCLUDE_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

#include <stm32l4.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/

#include <arch/board/stm32l4r9ai-disco-clocking.h>

/* DMA Channel/Stream Selections ****************************************************/
/* Stream selections are arbitrary for now but might become important in the future
 * is we set aside more DMA channels/streams.
 */

/* Values defined in arch/arm/src/stm32l4/chip/stm32l4x6xx_dma.h */

/* XXX are these used on disco? */

#if 0

#define DMACHAN_SDMMC DMACHAN_SDMMC_1      /* 2 choices * /

#define DMACHAN_SPI1_RX DMACHAN_SPI1_RX_1 /* 2 choices * /
#define DMACHAN_SPI1_TX DMACHAN_SPI1_TX_1 /* 2 choices * /

/* UART RX DMA configurations */

#define DMACHAN_USART1_RX DMACHAN_USART1_RX_2

#endif

/* ADC measurements
 * Default is ADC1_IN9 (PA4) connected to STMod+ connector CN1 pin 13.
 */

#define ADC1_MEASURE_CHANNEL     9
#define GPIO_MEASURE_ADC         (GPIO_ADC1_IN9)

/* Alternate function pin selections ************************************************/

/* USART1:
 *   RXD: PB7
 *   TXD: PA6
 *
 * these pins are shared with the I2C1, which is used for some onboard
 * devices, so I am disabling USART1, however I'm leaving the declarations
 * here for documentation for anyone who might want to use it instead of
 * those onboard devices.
 */

#if 0
#define GPIO_USART1_RX GPIO_USART1_RX_2    / * PB7  * /
#define GPIO_USART1_TX GPIO_USART1_TX_2    / * PB6  * /
#endif

/* USART2: Connected to STLink Debug via PD5, PD6
 *   RXD: PD6
 *   TXD: PD5
 */

#define GPIO_USART2_RX   GPIO_USART2_RX_2    /* PD6 */
#define GPIO_USART2_TX   GPIO_USART2_TX_2    /* PD5 */

/* UART4: Connected to PA0, PA1
 *   RXD: PA1 -> CN11 D5
 *   TXD: PA0 -> CN17 A4
 */

#define GPIO_UART4_RX   GPIO_UART4_RX_1    /* PA1 */
#define GPIO_UART4_TX   GPIO_UART4_TX_1    /* PA0 */


/* I2C
 *
 * The optional _GPIO configurations allow the I2C driver to manually
 * reset the bus to clear stuck slaves.  They match the pin configuration,
 * but are normally-high GPIOs.
 *
 * I2C1 connects to devices on the Discovery board, including the
 * CS43L22 (addr 0x94)  stereo DAC and amplifier.  It shares the
 * pins with USART1, so that port is disabled so we can access the
 * onboard device.
 *
 */

#define GPIO_I2C1_SCL    (GPIO_I2C1_SCL_1|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET)
#define GPIO_I2C1_SDA    (GPIO_I2C1_SDA_1|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET)
#define GPIO_I2C1_SCL_GPIO \
   (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN6)
#define GPIO_I2C1_SDA_GPIO \
   (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN7)

/* XXX Is I2C2 used on Disco? */

#if 0

#define GPIO_I2C2_SCL    (GPIO_I2C2_SCL_1|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET)
#define GPIO_I2C2_SDA    (GPIO_I2C2_SDA_1|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET)
#define GPIO_I2C2_SCL_GPIO \
   (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN10)
#define GPIO_I2C2_SDA_GPIO \
   (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN11)

#endif

/* SPI */

/* XXX is SPI1 used on Disco? */

#define GPIO_SPI1_MISO   GPIO_SPI1_MISO_1
#define GPIO_SPI1_MOSI   GPIO_SPI1_MOSI_1
#define GPIO_SPI1_SCK    GPIO_SPI1_SCK_1

/* XXX is SPI2 used on Disco? */

#define GPIO_SPI2_MISO   GPIO_SPI2_MISO_3
#define GPIO_SPI2_MOSI   GPIO_SPI2_MOSI_3
#define GPIO_SPI2_SCK    GPIO_SPI2_SCK_3

/* LEDs
 *
 * The STM32L4R9AI-DISCO board provides two user LEDs, LD1 (orange) and LD2 (green).
 *
 * PB0 is LD1 (orange)
 * PH4 is LD2 (green)
 *   - When the I/O is HIGH value, the LED is on.
 *   - When the I/O is LOW, the LED is off.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED_RED     0
#define BOARD_LED_GRN     1
#define BOARD_NLEDS       2

/* LED bits for use with board_userled_all() */

#define BOARD_LED_RED_BIT (1 << BOARD_LED_RED)
#define BOARD_LED_GRN_BIT (1 << BOARD_LED_GRN)

/* These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/stm32_autoleds.c. The LEDs are used to encode
 * OS-related events as follows when the red and green LEDs are available:
 *
 *   SYMBOL                Meaning                  BOARD_LED_GRN  BOARD_LED_RED
 *   -------------------  -----------------------  -----------    ------------
 *   LED_STARTED          NuttX has been started
 *   LED_HEAPALLOCATE     Heap has been allocated
 *   LED_IRQSENABLED      Interrupts enabled
 *   LED_STACKCREATED     Idle stack created
 *   LED_INIRQ            In an interrupt
 *   LED_SIGNAL           In a signal handler
 *   LED_ASSERTION        An assertion failed
 *   LED_PANIC            The system has crashed                  Blinking
 *   LED_IDLE             MCU is is sleep mode       ON
 *
 * Thus if BOARD_LED_GRN, NuttX has successfully booted and is, apparently, running
 * normally.  If BOARD_LED_RED is flashing at approximately 2Hz, then a fatal error
 * has been detected and the system has halted.
 */

#define LED_STARTED      0
#define LED_HEAPALLOCATE 1
#define LED_IRQSENABLED  2
#define LED_STACKCREATED 3
#define LED_INIRQ        4
#define LED_SIGNAL       5
#define LED_ASSERTION    6
#define LED_PANIC        7
#define LED_IDLE         8

/* Buttons
 *
 *  There is a 4 way d-pad 'joystick' with center button
 *  connected to PA0,1,5,2,3
 *                 C L D R U
 */

#define BUTTON_CENTER      0
#define BUTTON_LEFT        1
#define BUTTON_DOWN        2
#define BUTTON_RIGHT       3
#define BUTTON_UP          4
#define NUM_BUTTONS        5

#define BUTTON_CENTER_BIT  (1 << BUTTON_CENTER)
#define BUTTON_LEFT_BIT    (1 << BUTTON_LEFT)
#define BUTTON_DOWN_BIT    (1 << BUTTON_DOWN)
#define BUTTON_RIGHT_BIT   (1 << BUTTON_RIGHT)
#define BUTTON_UP_BIT      (1 << BUTTON_UP)

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/
/************************************************************************************
 * Name: stm32l4_board_initialize
 *
 * Description:
 *   All STM32L4 architectures must provide the following entry point.  This entry point
 *   is called early in the initialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

void stm32l4_board_initialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif  /* __CONFIGS_STM32L4R9AI_DISCO_INCLUDE_BOARD_H */
