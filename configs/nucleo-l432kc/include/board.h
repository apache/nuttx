/************************************************************************************
 * configs/nucleo-l432kc/include/board.h
 *
 *   Copyright (C) 2016, 2018 Gregory Nutt. All rights reserved.
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

#ifndef __CONFIGS_NUCLEO_L432KC_INCLUDE_BOARD_H
#define __CONFIGS_NUCLEO_L432KC_INCLUDE_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

#ifdef __KERNEL__
#include <stm32l4.h>
#endif

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/

#if defined(CONFIG_ARCH_CHIP_STM32L432KC)
#  include <arch/board/nucleo-l432kc.h>
#endif

/* DMA Channel/Stream Selections ****************************************************/
/* Stream selections are arbitrary for now but might become important in the future
 * is we set aside more DMA channels/streams.
 */

/* Values defined in arch/arm/src/stm32l4/chip/stm32l4x3xx_dma.h */

#define DMACHAN_SPI1_RX DMACHAN_SPI1_RX_1 /* 2 choices */
#define DMACHAN_SPI1_TX DMACHAN_SPI1_TX_1 /* 2 choices */

/* UART RX DMA configurations */

#define DMACHAN_USART1_RX DMACHAN_USART1_RX_2

/* ADC */

#define ADC1_DMA_CHAN DMACHAN_ADC1_1

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

/* USART2: Connected to STLInk Debug via PA2(TX), PA15(RX) */

#define GPIO_USART2_RX   GPIO_USART2_RX_2    /* PA15 */
#define GPIO_USART2_TX   GPIO_USART2_TX_1    /* PA2 */
#define GPIO_USART2_RTS  GPIO_USART2_RTS_2
#define GPIO_USART2_CTS  GPIO_USART2_CTS_2

/* I2C
 *
 * On Arduino the I2C bus is available at positions A4 and A5. On the
 * nucleo-1432kc board the I2C bus pins (PB6 and PB7) are connected to
 * pins A4 and A5 through the SB16 and SB18 solder bridges.
 *
 * If these solder bridges are open, then Arduino D4(PA5) and D5(PA6)
 * pins are not supported.  PA5 and PA6 must be configured as input
 * floating.
 *
 * The optional _GPIO configurations allow the I2C driver to manually
 * reset the bus to clear stuck slaves.  They match the pin configuration,
 * but are normally-high GPIOs.
 */

#define GPIO_I2C1_D4 \
   (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN5)
#define GPIO_I2C1_D5 \
   (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN6)
#define GPIO_I2C1_SCL \
   (GPIO_I2C1_SCL_1 | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET)
#define GPIO_I2C1_SDA \
   (GPIO_I2C1_SDA_1 | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET)
#define GPIO_I2C1_SCL_GPIO \
   (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | \
    GPIO_PORTB | GPIO_PIN6)
#define GPIO_I2C1_SDA_GPIO \
   (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | \
    GPIO_PORTB | GPIO_PIN7)

/* SPI */

/* SPI1 is available on the Arduino protocol at positions D11/12/13 */

#define GPIO_SPI1_MISO   GPIO_SPI1_MISO_1 /*PA6*/
#define GPIO_SPI1_MOSI   GPIO_SPI1_MOSI_1 /*PA7*/
#define GPIO_SPI1_SCK    GPIO_SPI1_SCK_1  /*PA5*/

#define GPIO_SPI2_MISO   GPIO_SPI2_MISO_1 /*PB14*/
#define GPIO_SPI2_MOSI   GPIO_SPI2_MOSI_1 /*PB15*/
#define GPIO_SPI2_SCK    GPIO_SPI2_SCK_2  /*PB13*/

/* LEDs
 *
 * The Nucleo l432kc board provides a single user LED, LD3.  LD3
 * is the green LED connected to Arduino signal D13 corresponding to
 * MCU I/O PB3 (pin 26).
 *
 *   - When the I/O is HIGH value, the LED is on.
 *   - When the I/O is LOW, the LED is off.
 */

/* LED index values for use with board_userled() */

#define BOARD_LD3         0
#define BOARD_NLEDS       1

/* LED bits for use with board_userled_all() */

#define BOARD_LD3_BIT     (1 << BOARD_LD3)

/* These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/sam_leds.c. The LEDs are used to encode OS-related
 * events as follows when the red LED (PE24) is available:
 *
 *   SYMBOL                Meaning                   LD3
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
 * Thus if LD3 NuttX has successfully booted and is, apparently, running
 * normally.  If LD3 is flashing at approximately 2Hz, then a fatal error
 * has been detected and the system has halted.
 */

#define LED_STARTED      0
#define LED_HEAPALLOCATE 0
#define LED_IRQSENABLED  0
#define LED_STACKCREATED 1
#define LED_INIRQ        1
#define LED_SIGNAL       2
#define LED_ASSERTION    2
#define LED_PANIC        1

/* Quadrature encoder
 * Default is to use timer 5 (32-bit) and encoder on PA0/PA1
 */

#define GPIO_TIM2_CH1IN GPIO_TIM2_CH1IN_1
#define GPIO_TIM2_CH2IN GPIO_TIM2_CH2IN_1

#define GPIO_TIM3_CH1IN GPIO_TIM3_CH1IN_3
#define GPIO_TIM3_CH2IN GPIO_TIM3_CH2IN_3

#define GPIO_TIM5_CH1IN GPIO_TIM5_CH1IN_1
#define GPIO_TIM5_CH2IN GPIO_TIM5_CH2IN_1

/* PWM output for full bridge, uses config 1, because port E is N/A on QFP64
 * CH1     | 1(A8) 2(E9)
 * CH2     | 1(A9) 2(E11)
 * CHN1    | 1(A7) 2(B13) 3(E8)
 * CHN2    | 1(B0) 2(B14) 3(E10)
 */

#define GPIO_TIM1_CH1OUT  GPIO_TIM1_CH1OUT_1
#define GPIO_TIM1_CH1NOUT GPIO_TIM1_CH1N_1
#define GPIO_TIM1_CH2OUT  GPIO_TIM1_CH2OUT_1
#define GPIO_TIM1_CH2NOUT GPIO_TIM1_CH2N_1

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
#endif  /* __CONFIGS_NUCLEO_L432KC_INCLUDE_BOARD_H */
