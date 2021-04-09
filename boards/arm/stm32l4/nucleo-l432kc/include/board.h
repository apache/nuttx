/****************************************************************************
 * boards/arm/stm32l4/nucleo-l432kc/include/board.h
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

#ifndef __BOARDS_ARM_STM32L4_NUCLEO_L432KC_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32L4_NUCLEO_L432KC_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

/* Clocking *****************************************************************/

#if defined(CONFIG_ARCH_CHIP_STM32L432KC)
#  include <arch/board/nucleo-l432kc.h>
#endif

/* Do not include STM32L4 header files here */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DMA Channel/Stream Selections ********************************************/

/* Stream selections are arbitrary for now but might become important in the
 * future is we set aside more DMA channels/streams.
 */

/* Values defined in arch/arm/src/stm32l4/hardware/stm32l4x3xx_dma.h */

#define DMACHAN_SPI1_RX DMACHAN_SPI1_RX_1 /* 2 choices */
#define DMACHAN_SPI1_TX DMACHAN_SPI1_TX_1 /* 2 choices */

/* UART RX DMA configurations */

#define DMACHAN_USART1_RX DMACHAN_USART1_RX_2

/* ADC */

#define ADC1_DMA_CHAN DMACHAN_ADC1_1

/* Alternate function pin selections ****************************************/

/* USART1:
 *   RXD: PA10  CN9 pin 3, CN10 pin 33
 *        PB7   CN7 pin 21
 *   TXD: PA9   CN5 pin 1, CN10 pin 21
 *        PB6   CN5 pin 3, CN10 pin 17
 */

#if defined(CONFIG_ARCH_BOARD_USART1_RX_PA10)
#  define GPIO_USART1_RX GPIO_USART1_RX_1    /* PA10 */
#elif defined(CONFIG_ARCH_BOARD_USART1_RX_PB7)
#  define GPIO_USART1_RX GPIO_USART1_RX_2    /* PB7 */
#endif

#if defined(CONFIG_ARCH_BOARD_USART1_TX_PA9)
#  define GPIO_USART1_TX GPIO_USART1_TX_1    /* PA9  */
#elif defined(CONFIG_ARCH_BOARD_USART1_TX_PB6)
#  define GPIO_USART1_TX GPIO_USART1_TX_2    /* PB6  */
#endif

/* USART2: Connected to STLInk Debug via PA2(TX), PA15(RX) */

#if defined(CONFIG_ARCH_BOARD_USART2_RX_PA3)
#  define GPIO_USART2_RX   GPIO_USART2_RX_1  /* PA3 */
#elif defined(CONFIG_ARCH_BOARD_USART2_RX_PA15)
#  define GPIO_USART2_RX   GPIO_USART2_RX_2  /* PA15 */
#endif
#define GPIO_USART2_TX   GPIO_USART2_TX_1    /* PA2 */
#define GPIO_USART2_RTS  GPIO_USART2_RTS_2
#define GPIO_USART2_CTS  GPIO_USART2_CTS_2

/* LPUART1 */

#if defined(CONFIG_ARCH_BOARD_LPUART1_RX_PA3)
#  define GPIO_LPUART1_RX GPIO_LPUART1_RX_1   /* PA3  */
#endif

#if defined(CONFIG_ARCH_BOARD_LPUART1_TX_PA2)
#  define GPIO_LPUART1_TX GPIO_LPUART1_TX_1   /* PA2 */
#endif

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
   (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN7)
#define GPIO_I2C1_D5 \
   (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN6)
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

#define GPIO_I2C3_SCL \
   (GPIO_I2C3_SCL_1 | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET)
#define GPIO_I2C3_SDA \
   (GPIO_I2C3_SDA_1 | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET)

/* SPI */

#if 1
/* On Arduino the SPI bus is available at positions D10 (SPI_CS), D11
 * (SPI_MOSI), D12 (SPI_MISO), D13 (SPI_SCK). On the nucleo-1432kc board
 * the SPI bus is available at PA11 (SPI_CS, made by GPIO), PB5 (SPI1_MOSI),
 * PB4 (SPI1_MISO), PB3 (SPI1_SCK).
 */

#  define GPIO_SPI1_MISO  GPIO_SPI1_MISO_3  /* PB4 */
#  define GPIO_SPI1_MOSI  GPIO_SPI1_MOSI_3  /* PB5 */
#  define GPIO_SPI1_SCK   GPIO_SPI1_SCK_3   /* PB3 */

#else
/* Optionally it is possible to use SPI bus on pins PA5, PA6 and PA7 but this
 * is incompatible with Arduino nano specification.
 */

#  define GPIO_SPI1_MISO  GPIO_SPI1_MISO_1  /* PA6 */
#  define GPIO_SPI1_MOSI  GPIO_SPI1_MOSI_1  /* PA7 */
#  define GPIO_SPI1_SCK   GPIO_SPI1_SCK_1   /* PA5 */
#endif

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
 * CH3     | 1(A10) 2(E10)
 * CH4     | 1(A11) 2(E14)
 * CHN1    | 1(A7) 2(B13) 3(E8)
 * CHN2    | 1(B0) 2(B14) 3(E10)
 * CHN3    | 1(B1) 2(B15) 3(E12)
 */

#define GPIO_TIM1_CH1OUT  GPIO_TIM1_CH1OUT_1
#define GPIO_TIM1_CH1NOUT GPIO_TIM1_CH1N_1
#define GPIO_TIM1_CH2OUT  GPIO_TIM1_CH2OUT_1
#define GPIO_TIM1_CH2NOUT GPIO_TIM1_CH2N_1
#define GPIO_TIM1_CH3OUT GPIO_TIM1_CH3OUT_1
#define GPIO_TIM1_CH3NOUT GPIO_TIM1_CH3OUT_1
#define GPIO_TIM1_CH4OUT GPIO_TIM1_CH4OUT_1

/* LPTIM2 PWM output
 * REVISIT : Add support for the other clock sources, LSE, LSI and HSI
 *
 * CH1     | 1(A4) 2(A8)
 */

#if defined(CONFIG_STM32L4_LPTIM2_CLK_APB1)
#  define STM32L4_LPTIM2_FREQUENCY STM32L4_APB1_LPTIM2_CLKIN
#endif

#if 1
#  define GPIO_LPTIM2_CH1OUT GPIO_LPTIM2_OUT_1
#else
#  define GPIO_LPTIM2_CH1OUT GPIO_LPTIM2_OUT_2
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32l4_board_initialize
 *
 * Description:
 *   All STM32L4 architectures must provide the following entry point.
 *   This entry point is called early in the initialization -- after all
 *   memory has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/

void stm32l4_board_initialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32L4_NUCLEO_L432KC_INCLUDE_BOARD_H */
