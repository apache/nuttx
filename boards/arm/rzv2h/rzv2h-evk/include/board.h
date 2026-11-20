/****************************************************************************
 * boards/arm/rzv2h/rzv2h-evk/include/board.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __BOARDS_ARM_RZV2H_RZV2H_EVK_INCLUDE_BOARD_H
#define __BOARDS_ARM_RZV2H_RZV2H_EVK_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GPIO definitions *********************************************************/

#define BOARD_NGPIOOUT          2
#define BOARD_NGPIOIN           1

/* LED index values for use with board_userled() */

#define BOARD_LED_0             0
#define BOARD_LED_1             1
#define BOARD_NLEDS             2

/* LED bits for use with board_userled_all() */

#define BOARD_LED_0_BIT         (1 << BOARD_LED_0)
#define BOARD_LED_1_BIT         (1 << BOARD_LED_1)

/* LED pins - aliases to existing NuttX LED macros */

#define BOARD_P0_0_GPIO         GPIO_P0_0_OUTPUT_LOW
#define BOARD_P0_1_GPIO         GPIO_P0_1_OUTPUT_LOW

/* Button pins - using Pmod at CN1 */

#define BOARD_P8_2_BUTTON  GPIO_P8_2_INPUT_PULLDOWN

/* GPIO Configuration */

#define GPIO_P0_0_OUTPUT_LOW (gpio_pinset_t){BSP_IO_PORT_00_PIN_00, (IOPORT_CFG_DRIVE_B01 | IOPORT_CFG_PORT_DIRECTION_OUTPUT | IOPORT_CFG_PORT_OUTPUT_LOW | IOPORT_CFG_SLEW_RATE_FAST)}
#define GPIO_P0_1_OUTPUT_LOW (gpio_pinset_t){BSP_IO_PORT_00_PIN_01, (IOPORT_CFG_DRIVE_B01 | IOPORT_CFG_PORT_DIRECTION_OUTPUT | IOPORT_CFG_PORT_OUTPUT_LOW | IOPORT_CFG_SLEW_RATE_FAST)}
#define GPIO_P8_2_INPUT_PULLDOWN (gpio_pinset_t){BSP_IO_PORT_08_PIN_02, (IOPORT_CFG_PORT_DIRECTION_INPUT | IOPORT_CFG_SPECIAL_PURPOSE_PORT_INPUT_ENABLE | IOPORT_CFG_PULLDOWN_ENABLE)}

/* In this stage, autoled is not supported */

#define LED_STARTED         0
#define LED_HEAPALLOCATE    0
#define LED_IRQSENABLED     0
#define LED_STACKCREATED    0
#define LED_INIRQ           0
#define LED_SIGNAL          0
#define LED_ASSERTION       0
#define LED_PANIC           0

/* Peripheral definitions ***************************************************/

#ifdef CONFIG_RZV2H_SPI_B

/* Hardware SSL routing is board-specific.  To select another valid SSL
 * signal, change only the BOARD_SPIx_SSL_ROUTE definition.  The matching
 * hardware SSL selector and pin configuration are derived below.  A master
 * uses the peripheral output configuration and a slave uses the input one.
 */

#  ifdef CONFIG_RZV2H_SPI_CHANNEL_0
#    define BOARD_SPI0_MOSI_GPIO  GPIO_MOSI0_P9_0_M1
#    define BOARD_SPI0_MISO_GPIO  GPIO_MISO0_P9_1_M1
#    define BOARD_SPI0_SCK_GPIO   GPIO_RSPCK0_P9_2_M1

#    define BOARD_SPI0_SSL_ROUTE_SSLA0_P04  1
#    define BOARD_SPI0_SSL_ROUTE_SSLA0_P34  2
#    define BOARD_SPI0_SSL_ROUTE_SSLA0_P93  3
#    define BOARD_SPI0_SSL_ROUTE_SSLA1_P05  4
#    define BOARD_SPI0_SSL_ROUTE_SSLA1_P35  5
#    define BOARD_SPI0_SSL_ROUTE_SSLA1_P94  6
#    define BOARD_SPI0_SSL_ROUTE_SSLA2_P14  7
#    define BOARD_SPI0_SSL_ROUTE_SSLA2_P36  8
#    define BOARD_SPI0_SSL_ROUTE_SSLA2_P95  9
#    define BOARD_SPI0_SSL_ROUTE_SSLA3_P15  10
#    define BOARD_SPI0_SSL_ROUTE_SSLA3_P37  11
#    define BOARD_SPI0_SSL_ROUTE_SSLA3_P96  12

#    define BOARD_SPI0_SSL_ROUTE  BOARD_SPI0_SSL_ROUTE_SSLA0_P93

#    if BOARD_SPI0_SSL_ROUTE == BOARD_SPI0_SSL_ROUTE_SSLA0_P04
#      define BOARD_SPI0_SSL_SELECT  0
#      define BOARD_SPI0_SSL_INPUT   GPIO_SSLA0_P0_4_M2_INPUT
#      define BOARD_SPI0_SSL_OUTPUT  GPIO_SSLA0_P0_4_M2_OUTPUT
#    elif BOARD_SPI0_SSL_ROUTE == BOARD_SPI0_SSL_ROUTE_SSLA0_P34
#      define BOARD_SPI0_SSL_SELECT  0
#      define BOARD_SPI0_SSL_INPUT   GPIO_SSLA0_P3_4_M5_INPUT
#      define BOARD_SPI0_SSL_OUTPUT  GPIO_SSLA0_P3_4_M5_OUTPUT
#    elif BOARD_SPI0_SSL_ROUTE == BOARD_SPI0_SSL_ROUTE_SSLA0_P93
#      define BOARD_SPI0_SSL_SELECT  0
#      define BOARD_SPI0_SSL_INPUT   GPIO_SSLA0_P9_3_M1_INPUT
#      define BOARD_SPI0_SSL_OUTPUT  GPIO_SSLA0_P9_3_M1_OUTPUT
#    elif BOARD_SPI0_SSL_ROUTE == BOARD_SPI0_SSL_ROUTE_SSLA1_P05
#      define BOARD_SPI0_SSL_SELECT  1
#      define BOARD_SPI0_SSL_INPUT   GPIO_SSLA1_P0_5_M2_INPUT
#      define BOARD_SPI0_SSL_OUTPUT  GPIO_SSLA1_P0_5_M2_OUTPUT
#    elif BOARD_SPI0_SSL_ROUTE == BOARD_SPI0_SSL_ROUTE_SSLA1_P35
#      define BOARD_SPI0_SSL_SELECT  1
#      define BOARD_SPI0_SSL_INPUT   GPIO_SSLA1_P3_5_M5_INPUT
#      define BOARD_SPI0_SSL_OUTPUT  GPIO_SSLA1_P3_5_M5_OUTPUT
#    elif BOARD_SPI0_SSL_ROUTE == BOARD_SPI0_SSL_ROUTE_SSLA1_P94
#      define BOARD_SPI0_SSL_SELECT  1
#      define BOARD_SPI0_SSL_INPUT   GPIO_SSLA1_P9_4_M1_INPUT
#      define BOARD_SPI0_SSL_OUTPUT  GPIO_SSLA1_P9_4_M1_OUTPUT
#    elif BOARD_SPI0_SSL_ROUTE == BOARD_SPI0_SSL_ROUTE_SSLA2_P14
#      define BOARD_SPI0_SSL_SELECT  2
#      define BOARD_SPI0_SSL_INPUT   GPIO_SSLA2_P1_4_M2_INPUT
#      define BOARD_SPI0_SSL_OUTPUT  GPIO_SSLA2_P1_4_M2_OUTPUT
#    elif BOARD_SPI0_SSL_ROUTE == BOARD_SPI0_SSL_ROUTE_SSLA2_P36
#      define BOARD_SPI0_SSL_SELECT  2
#      define BOARD_SPI0_SSL_INPUT   GPIO_SSLA2_P3_6_M5_INPUT
#      define BOARD_SPI0_SSL_OUTPUT  GPIO_SSLA2_P3_6_M5_OUTPUT
#    elif BOARD_SPI0_SSL_ROUTE == BOARD_SPI0_SSL_ROUTE_SSLA2_P95
#      define BOARD_SPI0_SSL_SELECT  2
#      define BOARD_SPI0_SSL_INPUT   GPIO_SSLA2_P9_5_M1_INPUT
#      define BOARD_SPI0_SSL_OUTPUT  GPIO_SSLA2_P9_5_M1_OUTPUT
#    elif BOARD_SPI0_SSL_ROUTE == BOARD_SPI0_SSL_ROUTE_SSLA3_P15
#      define BOARD_SPI0_SSL_SELECT  3
#      define BOARD_SPI0_SSL_INPUT   GPIO_SSLA3_P1_5_M2_INPUT
#      define BOARD_SPI0_SSL_OUTPUT  GPIO_SSLA3_P1_5_M2_OUTPUT
#    elif BOARD_SPI0_SSL_ROUTE == BOARD_SPI0_SSL_ROUTE_SSLA3_P37
#      define BOARD_SPI0_SSL_SELECT  3
#      define BOARD_SPI0_SSL_INPUT   GPIO_SSLA3_P3_7_M5_INPUT
#      define BOARD_SPI0_SSL_OUTPUT  GPIO_SSLA3_P3_7_M5_OUTPUT
#    elif BOARD_SPI0_SSL_ROUTE == BOARD_SPI0_SSL_ROUTE_SSLA3_P96
#      define BOARD_SPI0_SSL_SELECT  3
#      define BOARD_SPI0_SSL_INPUT   GPIO_SSLA3_P9_6_M1_INPUT
#      define BOARD_SPI0_SSL_OUTPUT  GPIO_SSLA3_P9_6_M1_OUTPUT
#    else
#      error "Invalid SPI_B channel 0 SSL route"
#    endif

#    ifdef CONFIG_RZV2H_SPI_CHANNEL_0_SLAVE
#      define BOARD_SPI0_SSL_PIN  BOARD_SPI0_SSL_INPUT
#    else
#      define BOARD_SPI0_SSL_PIN  BOARD_SPI0_SSL_OUTPUT
#    endif

#    ifdef CONFIG_RZV2H_SPI_CHANNEL_0_4WIRE
#      define BOARD_SPI0_SSL_USED         1
#    else
#      define BOARD_SPI0_SSL_USED         0
#    endif
#  endif

#  ifdef CONFIG_RZV2H_SPI_CHANNEL_1
#    define BOARD_SPI1_MOSI_GPIO  GPIO_MOSI0_PB_1_M4
#    define BOARD_SPI1_MISO_GPIO  GPIO_MISO0_PB_2_M4
#    define BOARD_SPI1_SCK_GPIO   GPIO_RSPCK0_PB_0_M4

#    define BOARD_SPI1_SSL_ROUTE_SSLB0_P34  1
#    define BOARD_SPI1_SSL_ROUTE_SSLB0_PA4  2
#    define BOARD_SPI1_SSL_ROUTE_SSLB1_P36  3
#    define BOARD_SPI1_SSL_ROUTE_SSLB1_PA5  4
#    define BOARD_SPI1_SSL_ROUTE_SSLB2_P04  5
#    define BOARD_SPI1_SSL_ROUTE_SSLB2_PA6  6
#    define BOARD_SPI1_SSL_ROUTE_SSLB3_P14  7
#    define BOARD_SPI1_SSL_ROUTE_SSLB3_PA7  8

#    define BOARD_SPI1_SSL_ROUTE  BOARD_SPI1_SSL_ROUTE_SSLB0_PA4

#    if BOARD_SPI1_SSL_ROUTE == BOARD_SPI1_SSL_ROUTE_SSLB0_P34
#      define BOARD_SPI1_SSL_SELECT  0
#      define BOARD_SPI1_SSL_INPUT   GPIO_SSLB0_P3_4_M6_INPUT
#      define BOARD_SPI1_SSL_OUTPUT  GPIO_SSLB0_P3_4_M6_OUTPUT
#    elif BOARD_SPI1_SSL_ROUTE == BOARD_SPI1_SSL_ROUTE_SSLB0_PA4
#      define BOARD_SPI1_SSL_SELECT  0
#      define BOARD_SPI1_SSL_INPUT   GPIO_SSLB0_PA_4_M4_INPUT
#      define BOARD_SPI1_SSL_OUTPUT  GPIO_SSLB0_PA_4_M4_OUTPUT
#    elif BOARD_SPI1_SSL_ROUTE == BOARD_SPI1_SSL_ROUTE_SSLB1_P36
#      define BOARD_SPI1_SSL_SELECT  1
#      define BOARD_SPI1_SSL_INPUT   GPIO_SSLB1_P3_6_M6_INPUT
#      define BOARD_SPI1_SSL_OUTPUT  GPIO_SSLB1_P3_6_M6_OUTPUT
#    elif BOARD_SPI1_SSL_ROUTE == BOARD_SPI1_SSL_ROUTE_SSLB1_PA5
#      define BOARD_SPI1_SSL_SELECT  1
#      define BOARD_SPI1_SSL_INPUT   GPIO_SSLB1_PA_5_M4_INPUT
#      define BOARD_SPI1_SSL_OUTPUT  GPIO_SSLB1_PA_5_M4_OUTPUT
#    elif BOARD_SPI1_SSL_ROUTE == BOARD_SPI1_SSL_ROUTE_SSLB2_P04
#      define BOARD_SPI1_SSL_SELECT  2
#      define BOARD_SPI1_SSL_INPUT   GPIO_SSLB2_P0_4_M3_INPUT
#      define BOARD_SPI1_SSL_OUTPUT  GPIO_SSLB2_P0_4_M3_OUTPUT
#    elif BOARD_SPI1_SSL_ROUTE == BOARD_SPI1_SSL_ROUTE_SSLB2_PA6
#      define BOARD_SPI1_SSL_SELECT  2
#      define BOARD_SPI1_SSL_INPUT   GPIO_SSLB2_PA_6_M4_INPUT
#      define BOARD_SPI1_SSL_OUTPUT  GPIO_SSLB2_PA_6_M4_OUTPUT
#    elif BOARD_SPI1_SSL_ROUTE == BOARD_SPI1_SSL_ROUTE_SSLB3_P14
#      define BOARD_SPI1_SSL_SELECT  3
#      define BOARD_SPI1_SSL_INPUT   GPIO_SSLB3_P1_4_M3_INPUT
#      define BOARD_SPI1_SSL_OUTPUT  GPIO_SSLB3_P1_4_M3_OUTPUT
#    elif BOARD_SPI1_SSL_ROUTE == BOARD_SPI1_SSL_ROUTE_SSLB3_PA7
#      define BOARD_SPI1_SSL_SELECT  3
#      define BOARD_SPI1_SSL_INPUT   GPIO_SSLB3_PA_7_M4_INPUT
#      define BOARD_SPI1_SSL_OUTPUT  GPIO_SSLB3_PA_7_M4_OUTPUT
#    else
#      error "Invalid SPI_B channel 1 SSL route"
#    endif

#    ifdef CONFIG_RZV2H_SPI_CHANNEL_1_SLAVE
#      define BOARD_SPI1_SSL_PIN  BOARD_SPI1_SSL_INPUT
#    else
#      define BOARD_SPI1_SSL_PIN  BOARD_SPI1_SSL_OUTPUT
#    endif

#    ifdef CONFIG_RZV2H_SPI_CHANNEL_1_4WIRE
#      define BOARD_SPI1_SSL_USED         1
#    else
#      define BOARD_SPI1_SSL_USED         0
#    endif
#  endif

#  ifdef CONFIG_RZV2H_SPI_CHANNEL_2
#    define BOARD_SPI2_MOSI_GPIO  GPIO_MOSI0_PB_4_M5
#    define BOARD_SPI2_MISO_GPIO  GPIO_MISO0_PB_3_M5
#    define BOARD_SPI2_SCK_GPIO   GPIO_RSPCK0_PB_5_M5

#    define BOARD_SPI2_SSL_ROUTE_SSLC0_P35  1
#    define BOARD_SPI2_SSL_ROUTE_SSLC0_PA7  2
#    define BOARD_SPI2_SSL_ROUTE_SSLC1_P37  3
#    define BOARD_SPI2_SSL_ROUTE_SSLC1_PA6  4
#    define BOARD_SPI2_SSL_ROUTE_SSLC2_P05  5
#    define BOARD_SPI2_SSL_ROUTE_SSLC2_PA5  6
#    define BOARD_SPI2_SSL_ROUTE_SSLC3_P15  7
#    define BOARD_SPI2_SSL_ROUTE_SSLC3_PA4  8

#    define BOARD_SPI2_SSL_ROUTE  BOARD_SPI2_SSL_ROUTE_SSLC0_PA7

#    if BOARD_SPI2_SSL_ROUTE == BOARD_SPI2_SSL_ROUTE_SSLC0_P35
#      define BOARD_SPI2_SSL_SELECT  0
#      define BOARD_SPI2_SSL_INPUT   GPIO_SSLC0_P3_5_M6_INPUT
#      define BOARD_SPI2_SSL_OUTPUT  GPIO_SSLC0_P3_5_M6_OUTPUT
#    elif BOARD_SPI2_SSL_ROUTE == BOARD_SPI2_SSL_ROUTE_SSLC0_PA7
#      define BOARD_SPI2_SSL_SELECT  0
#      define BOARD_SPI2_SSL_INPUT   GPIO_SSLC0_PA_7_M5_INPUT
#      define BOARD_SPI2_SSL_OUTPUT  GPIO_SSLC0_PA_7_M5_OUTPUT
#    elif BOARD_SPI2_SSL_ROUTE == BOARD_SPI2_SSL_ROUTE_SSLC1_P37
#      define BOARD_SPI2_SSL_SELECT  1
#      define BOARD_SPI2_SSL_INPUT   GPIO_SSLC1_P3_7_M6_INPUT
#      define BOARD_SPI2_SSL_OUTPUT  GPIO_SSLC1_P3_7_M6_OUTPUT
#    elif BOARD_SPI2_SSL_ROUTE == BOARD_SPI2_SSL_ROUTE_SSLC1_PA6
#      define BOARD_SPI2_SSL_SELECT  1
#      define BOARD_SPI2_SSL_INPUT   GPIO_SSLC1_PA_6_M5_INPUT
#      define BOARD_SPI2_SSL_OUTPUT  GPIO_SSLC1_PA_6_M5_OUTPUT
#    elif BOARD_SPI2_SSL_ROUTE == BOARD_SPI2_SSL_ROUTE_SSLC2_P05
#      define BOARD_SPI2_SSL_SELECT  2
#      define BOARD_SPI2_SSL_INPUT   GPIO_SSLC2_P0_5_M3_INPUT
#      define BOARD_SPI2_SSL_OUTPUT  GPIO_SSLC2_P0_5_M3_OUTPUT
#    elif BOARD_SPI2_SSL_ROUTE == BOARD_SPI2_SSL_ROUTE_SSLC2_PA5
#      define BOARD_SPI2_SSL_SELECT  2
#      define BOARD_SPI2_SSL_INPUT   GPIO_SSLC2_PA_5_M5_INPUT
#      define BOARD_SPI2_SSL_OUTPUT  GPIO_SSLC2_PA_5_M5_OUTPUT
#    elif BOARD_SPI2_SSL_ROUTE == BOARD_SPI2_SSL_ROUTE_SSLC3_P15
#      define BOARD_SPI2_SSL_SELECT  3
#      define BOARD_SPI2_SSL_INPUT   GPIO_SSLC3_P1_5_M3_INPUT
#      define BOARD_SPI2_SSL_OUTPUT  GPIO_SSLC3_P1_5_M3_OUTPUT
#    elif BOARD_SPI2_SSL_ROUTE == BOARD_SPI2_SSL_ROUTE_SSLC3_PA4
#      define BOARD_SPI2_SSL_SELECT  3
#      define BOARD_SPI2_SSL_INPUT   GPIO_SSLC3_PA_4_M5_INPUT
#      define BOARD_SPI2_SSL_OUTPUT  GPIO_SSLC3_PA_4_M5_OUTPUT
#    else
#      error "Invalid SPI_B channel 2 SSL route"
#    endif

#    ifdef CONFIG_RZV2H_SPI_CHANNEL_2_SLAVE
#      define BOARD_SPI2_SSL_PIN  BOARD_SPI2_SSL_INPUT
#    else
#      define BOARD_SPI2_SSL_PIN  BOARD_SPI2_SSL_OUTPUT
#    endif

#    ifdef CONFIG_RZV2H_SPI_CHANNEL_2_4WIRE
#      define BOARD_SPI2_SSL_USED         1
#    else
#      define BOARD_SPI2_SSL_USED         0
#    endif
#  endif
#endif

/* SCI0 is used as the default NSH console interface */

#define BOARD_CONSOLE_UART        0       /* Use SCI0 for console */
#define BOARD_CONSOLE_BAUD        115200
#define BOARD_CONSOLE_BITS        8
#define BOARD_CONSOLE_PARITY      0
#define BOARD_CONSOLE_2STOP       0

/* SCI-B UART pins */

#define BOARD_SCI0_TXD_GPIO   GPIO_TXD0_MOSI0_DA0_P5_0_M1
#define BOARD_SCI0_RXD_GPIO   GPIO_RXD0_MISO0_SCL0_P5_1_M1
#define BOARD_SCI1_TXD_GPIO   GPIO_TXD1_MOSI1_SDA1_P5_2_M1
#define BOARD_SCI1_RXD_GPIO   GPIO_RXD1_MISO1_SCL1_P5_3_M1
#define BOARD_SCI2_TXD_GPIO   GPIO_TXD2_MOSI2_SDA2_P5_4_M1
#define BOARD_SCI2_RXD_GPIO   GPIO_RXD2_MISO2_SCL2_P5_5_M1
#define BOARD_SCI3_TXD_GPIO   GPIO_TXD3_MOSI3_SDA3_P5_6_M1
#define BOARD_SCI3_RXD_GPIO   GPIO_RXD3_MISO3_SCL3_P5_7_M1

#define BOARD_SCI4_TXD_GPIO   GPIO_TXD4_MOSI4_SDA4_P8_4_M6
#define BOARD_SCI4_RXD_GPIO   GPIO_RXD4_MISO4_SCL4_P8_5_M6
#define BOARD_SCI5_TXD_GPIO   GPIO_TXD5_MOSI5_SDA5_P7_2_M1
#define BOARD_SCI5_RXD_GPIO   GPIO_RXD5_MISO5_SCL5_P7_3_M1
#define BOARD_SCI6_TXD_GPIO   GPIO_TXD6_MOSI6_SDA6_P7_4_M1
#define BOARD_SCI6_RXD_GPIO   GPIO_RXD6_MISO6_SCL6_P7_5_M1
#define BOARD_SCI7_TXD_GPIO   GPIO_TXD7_MOSI7_SDA7_P7_6_M1
#define BOARD_SCI7_RXD_GPIO   GPIO_RXD7_MISO7_SCL7_P7_7_M1
#define BOARD_SCI8_TXD_GPIO   GPIO_TXD8_MOSI8_SDA6_P9_0_M2
#define BOARD_SCI8_RXD_GPIO   GPIO_RXD8_MISO8_SCL6_P9_1_M2
#define BOARD_SCI9_TXD_GPIO   GPIO_TXD9_MOSI9_SDA9_P8_2_M6
#define BOARD_SCI9_RXD_GPIO   GPIO_RXD9_MISO9_SCL9_P8_3_M6

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: rzv2h_board_initialize
 *
 * Description:
 *   All RZV2H architectures must provide the following entry point. This
 *   entry point is called early in the initialization -- after clocking
 *   and memory have been configured but before caches have been enabled
 *   and before any devices have been initialized.
 *
 ****************************************************************************/

void rzv2h_board_initialize(void);

/****************************************************************************
 * Name: rzv2h_serial_setup
 *
 * Description:
 *   Configure the board used by the enabled SCI UART channels.
 *
 ****************************************************************************/

void rzv2h_serial_setup(void);

#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_RZV2H_RZV2H_EVK_INCLUDE_BOARD_H */
