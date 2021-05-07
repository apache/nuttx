/****************************************************************************
 * boards/arm/stm32l4/nucleo-l476rg/include/board.h
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

#ifndef __BOARDS_ARM_STM32L4_NUCLEO_L476RG_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32L4_NUCLEO_L476RG_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

/* Clocking *****************************************************************/

#if defined(CONFIG_ARCH_CHIP_STM32L476RG)
#  include <arch/board/nucleo-l476rg.h>
#endif

/* Do not include STM32 L4 header files here */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DMA Channel/Stream Selections ********************************************/

/* Stream selections are arbitrary for now but might become important in the
 * future is we set aside more DMA channels/streams.
 */

/* Values defined in arch/arm/src/stm32l4/hardware/stm32l4x6xx_dma.h */

#define DMACHAN_SDMMC DMACHAN_SDMMC_1     /* 2 choices */

#define DMACHAN_SPI1_RX DMACHAN_SPI1_RX_1 /* 2 choices */
#define DMACHAN_SPI1_TX DMACHAN_SPI1_TX_1 /* 2 choices */

/* UART RX DMA configurations */

#define DMACHAN_USART1_RX DMACHAN_USART1_RX_2

/* ADC */

#define ADC1_DMA_CHAN DMACHAN_ADC1_1
#define ADC2_DMA_CHAN DMACHAN_ADC2_2
#define ADC3_DMA_CHAN DMACHAN_ADC3_2

/* Alternate function pin selections ****************************************/

/* CAN1: (added 31-03 -- Ben vd Veen (DisruptiveNL)
 *   RXD: PA11
 *        PB8
 *        PD0
 *   TXD: PA12
 *        PB9
 *        PD1
 */

#define GPIO_CAN1_RX   GPIO_CAN1_RX_2        /* PA11 - AF9 */
#define GPIO_CAN1_TX   GPIO_CAN1_TX_2        /* PA12 - AF9 */

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

/* USART2: Connected to STLInk Debug via PA2, PA3
 *   RXD: PA3   CN9 pin 1 (See SB13, 14, 62, 63). CN10 pin 37
 *        PD6
 *   TXD: PA2   CN9 pin 2 (See SB13, 14, 62, 63). CN10 pin 35
 *        PD5
 */

#define GPIO_USART2_RX   GPIO_USART2_RX_1    /* PA3 */
#define GPIO_USART2_TX   GPIO_USART2_TX_1    /* PA2 */
#define GPIO_USART2_RTS  GPIO_USART2_RTS_2
#define GPIO_USART2_CTS  GPIO_USART2_CTS_2

/* USART3:
 *   RXD: PA10  CN9 pin 3, CN10 pin 33
 *        PC5   CN5 pin 9, CN10 pin 6
 *        PC11  CN7 pin 2
 *   TXD: PA9   CN5 pin 1, CN10 pin 21
 *        PC4   CN9 pin 3, CN10 pin 34
 *        PC10  CN7 pin 1
 */

#define GPIO_USART3_RX   GPIO_USART3_RX_3    /* PC11 */
#define GPIO_USART3_TX   GPIO_USART3_TX_3    /* PC10 */

#define GPIO_UART4_RX    GPIO_UART4_RX_1     /* PA1 */
#define GPIO_UART4_TX    GPIO_UART4_TX_1     /* PA0 */

/* I2C
 *
 * The optional _GPIO configurations allow the I2C driver to manually
 * reset the bus to clear stuck slaves.  They match the pin configuration,
 * but are normally-high GPIOs.
 */

#define GPIO_I2C1_SCL \
   (GPIO_I2C1_SCL_2 | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET)
#define GPIO_I2C1_SDA \
   (GPIO_I2C1_SDA_2 | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET)
#define GPIO_I2C1_SCL_GPIO \
   (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | \
    GPIO_PORTB | GPIO_PIN8)
#define GPIO_I2C1_SDA_GPIO \
   (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | \
    GPIO_PORTB | GPIO_PIN9)

#define GPIO_I2C2_SCL \
   (GPIO_I2C2_SCL_1 | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET)
#define GPIO_I2C2_SDA \
   (GPIO_I2C2_SDA_1 | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET)
#define GPIO_I2C2_SCL_GPIO \
   (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | \
    GPIO_PORTB | GPIO_PIN10)
#define GPIO_I2C2_SDA_GPIO \
   (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | \
    GPIO_PORTB | GPIO_PIN11)

/* SPI */

#define GPIO_SPI1_MISO   GPIO_SPI1_MISO_2     /* PB4 */
#define GPIO_SPI1_MOSI   GPIO_SPI1_MOSI_2     /* PB5 */
#define GPIO_SPI1_SCK    GPIO_SPI1_SCK_2      /* PB3 */

#define GPIO_SPI2_MISO   GPIO_SPI2_MISO_1     /* PB14 */
#define GPIO_SPI2_MOSI   GPIO_SPI2_MOSI_1     /* PB15 */
#define GPIO_SPI2_SCK    GPIO_SPI2_SCK_2      /* PB13 */

/* LEDs
 *
 * The Nucleo l476RG board provides a single user LED, LD2.  LD2
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
#define LED_INIRQ        1
#define LED_SIGNAL       2
#define LED_ASSERTION    2
#define LED_PANIC        1

/* Buttons
 *
 *   B1 USER:
 *   the user button is connected to the I/O PC13 (pin 2) of the STM32
 *   microcontroller.
 */

#define BUTTON_USER        0
#define NUM_BUTTONS        1

#define BUTTON_USER_BIT    (1 << BUTTON_USER)

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

#define GPIO_LPTIM1_CH1OUT  GPIO_LPTIM1_OUT_1
#define GPIO_LPTIM2_CH1OUT  GPIO_LPTIM2_OUT_2

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

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32L4_NUCLEO_F476RG_INCLUDE_BOARD_H */
