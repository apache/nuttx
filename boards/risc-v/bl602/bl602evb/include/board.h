/****************************************************************************
 * boards/risc-v/bl602/bl602evb/include/board.h
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

#ifndef __BOARDS_RISCV_BL602_BL602EVB_INCLUDE_BOARD_H
#define __BOARDS_RISCV_BL602_BL602EVB_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/* Do not include BL602 header files here. */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GPIO Configuration */

#define BOARD_NGPIOIN     1 /* Amount of GPIO Input pins */
#define BOARD_NGPIOOUT    1 /* Amount of GPIO Output pins */
#define BOARD_NGPIOINT    1 /* Amount of GPIO Input w/ Interruption pins */

#define BOARD_GPIO_IN1    (GPIO_INPUT | GPIO_PULLUP | \
                            GPIO_FUNC_SWGPIO | GPIO_PIN0)
#define BOARD_GPIO_OUT1   (GPIO_OUTPUT | GPIO_PULLDOWN | \
                            GPIO_FUNC_SWGPIO | GPIO_PIN1)
#define BOARD_GPIO_INT1   (GPIO_INPUT | GPIO_PULLUP | \
                            GPIO_FUNC_SWGPIO | GPIO_PIN2)

/* UART Configuration */

#define BOARD_UART_0_RX_PIN (GPIO_INPUT | GPIO_PULLUP | \
                              GPIO_FUNC_UART | GPIO_PIN7)
#define BOARD_UART_0_TX_PIN (GPIO_INPUT | GPIO_PULLUP | \
                              GPIO_FUNC_UART | GPIO_PIN16)
#define BOARD_UART_1_RX_PIN (GPIO_INPUT | GPIO_PULLUP | \
                              GPIO_FUNC_UART | GPIO_PIN3)
#define BOARD_UART_1_TX_PIN (GPIO_INPUT | GPIO_PULLUP | \
                              GPIO_FUNC_UART | GPIO_PIN4)

/* PWM Configuration */

#define BOARD_PWM_CH0_PIN (GPIO_OUTPUT | GPIO_PULLDOWN | \
                            GPIO_FUNC_PWM | GPIO_PIN0)
#define BOARD_PWM_CH1_PIN (GPIO_OUTPUT | GPIO_PULLDOWN | \
                            GPIO_FUNC_PWM | GPIO_PIN1)
#define BOARD_PWM_CH2_PIN (GPIO_OUTPUT | GPIO_PULLDOWN | \
                            GPIO_FUNC_PWM | GPIO_PIN2)
#define BOARD_PWM_CH3_PIN (GPIO_OUTPUT | GPIO_PULLDOWN | \
                            GPIO_FUNC_PWM | GPIO_PIN3)
#define BOARD_PWM_CH4_PIN (GPIO_OUTPUT | GPIO_PULLDOWN | \
                            GPIO_FUNC_PWM | GPIO_PIN4)

/* I2C Configuration */

#define BOARD_I2C_SCL (GPIO_INPUT | GPIO_PULLUP | GPIO_FUNC_I2C | GPIO_PIN4)
#define BOARD_I2C_SDA (GPIO_INPUT | GPIO_PULLUP | GPIO_FUNC_I2C | GPIO_PIN3)

/* SPI Configuration */

#define BOARD_SPI_CS   (GPIO_INPUT | GPIO_PULLUP | GPIO_FUNC_SPI | GPIO_PIN2)
#define BOARD_SPI_MOSI (GPIO_INPUT | GPIO_PULLUP | GPIO_FUNC_SPI | GPIO_PIN1)
#define BOARD_SPI_MISO (GPIO_INPUT | GPIO_PULLUP | GPIO_FUNC_SPI | GPIO_PIN0)
#define BOARD_SPI_CLK  (GPIO_INPUT | GPIO_PULLUP | GPIO_FUNC_SPI | GPIO_PIN3)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

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
 * Name: litex_boardinitialize
 ****************************************************************************/

void bl602_boardinitialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_RISC_V_BL602_BL602EVB_INCLUDE_BOARD_H */
