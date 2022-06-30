/****************************************************************************
 * boards/arm/stm32wb/nucleo-wb55rg/include/board.h
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

#ifndef __BOARDS_ARM_STM32WB_NUCLEO_WB55RG_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32WB_NUCLEO_WB55RG_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

/* Clocking *****************************************************************/

#include "nucleo-wb55rg.h"

/* Do not include STM32WB header files here */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DMA Channel/Stream Selections ********************************************/

/* Stream selections are arbitrary for now but might become important in the
 * future is we set aside more DMA channels/streams.
 */

/* Values defined in arch/arm/src/stm32wb/hardware/stm32wb_dmamux.h */

#define DMAMAP_SPI1_RX DMAMAP_SPI1_RX_0     /* DMA1 */
#define DMAMAP_SPI1_TX DMAMAP_SPI1_TX_0     /* DMA1 */

/* UART RX DMA configurations */

#define DMAMAP_USART1_RX DMAMAP_USART1_RX_0 /* DMA1 */

/* ADC */

#define DMAMAP_ADC1 DMAMAP_ADC1_0           /* DMA1 */

/* Alternate function pin selections ****************************************/

/* USART */

#define GPIO_USART1_RX GPIO_USART1_RX_2    /* PB7 */
#define GPIO_USART1_TX GPIO_USART1_TX_2    /* PB6 */

/* LPUART */

#define GPIO_LPUART1_RX GPIO_LPUART1_RX_1  /* PA3 */
#define GPIO_LPUART1_TX GPIO_LPUART1_TX_1  /* PA2 */

/* I2C
 *
 * The optional _GPIO configurations allow the I2C driver to manually
 * reset the bus to clear stuck slaves.  They match the pin configuration,
 * but are normally-high GPIOs.
 */

#define GPIO_I2C1_SCL (GPIO_I2C1_SCL_2 | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET)
#define GPIO_I2C1_SDA (GPIO_I2C1_SDA_2 | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET)
#define GPIO_I2C1_SCL_GPIO \
   (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | \
    GPIO_PORTB | GPIO_PIN8)
#define GPIO_I2C1_SDA_GPIO \
   (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | \
    GPIO_PORTB | GPIO_PIN9)

/* SPI */

#define GPIO_SPI1_NSS    GPIO_SPI1_NSS_2      /* PB2 */
#define GPIO_SPI1_SCK    GPIO_SPI1_SCK_2      /* PA5 */
#define GPIO_SPI1_MISO   GPIO_SPI1_MISO_1     /* PA6 */
#define GPIO_SPI1_MOSI   GPIO_SPI1_MOSI_1     /* PA7 */

/* LEDs
 *
 * The Nucleo WB55RG board provides three user LEDs.
 *
 *   - When the I/O is HIGH value, the LED is on.
 *   - When the I/O is LOW, the LED is off.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED1        0    /* PB5 */
#define BOARD_LED2        1    /* PB0 */
#define BOARD_LED3        2    /* PB1 */
#define BOARD_NLEDS       3

#define BOARD_LED_BLUE    BOARD_LED1
#define BOARD_LED_GREEN   BOARD_LED2
#define BOARD_LED_RED     BOARD_LED3

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT    (1 << BOARD_LED1)
#define BOARD_LED2_BIT    (1 << BOARD_LED2)
#define BOARD_LED3_BIT    (1 << BOARD_LED3)

/* These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/stm32_autoleds.c.
 */

#define LED_STARTED      0
#define LED_HEAPALLOCATE 0
#define LED_IRQSENABLED  0
#define LED_STACKCREATED 1
#define LED_INIRQ        1
#define LED_SIGNAL       2
#define LED_ASSERTION    2
#define LED_PANIC        1

/* Buttons */

#define BUTTON_SW1        0  /* PC4, needs SB47 close */
#define BUTTON_SW2        1  /* PD0 */
#define BUTTON_SW3        2  /* PD1 */
#define NUM_BUTTONS       3

#define BUTTON_SW1_BIT    (1 << BUTTON_SW1)
#define BUTTON_SW2_BIT    (1 << BUTTON_SW2)
#define BUTTON_SW3_BIT    (1 << BUTTON_SW3)

/* Quadrature Encoder */

#define GPIO_TIM2_CH1IN GPIO_TIM2_CH1IN_1      /* PA0 */
#define GPIO_TIM2_CH2IN GPIO_TIM2_CH2IN_1      /* PA1 */

/* PWM output for full bridge */

#define GPIO_TIM1_CH1OUT  GPIO_TIM1_CH1OUT_1   /* PA8 */
#define GPIO_TIM1_CH1NOUT GPIO_TIM1_CH1N_2     /* PB13 */
#define GPIO_TIM1_CH2OUT  GPIO_TIM1_CH2OUT_1   /* PA9 */
#define GPIO_TIM1_CH2NOUT GPIO_TIM1_CH2N_2     /* PB14 */

/* LPTIM PWM */

#define GPIO_LPTIM1_CH1OUT  GPIO_LPTIM1_OUT_3  /* PC1 */
#define GPIO_LPTIM2_CH1OUT  GPIO_LPTIM2_OUT_1  /* PA4 */

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
 * Name: stm32wb_board_initialize
 *
 * Description:
 *   All STM32WB architectures must provide the following entry point.
 *   This entry point is called early in the initialization -- after all
 *   memory has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/

void stm32wb_board_initialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32WB_NUCLEO_WB55RG_INCLUDE_BOARD_H */
