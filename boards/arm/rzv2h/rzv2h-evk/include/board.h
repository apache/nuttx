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
#define GPIO_P5_7_OUTPUT  RZV2H_GPIO_OUTPUT(5, 7)
#define GPIO_P7_3_OUTPUT  RZV2H_GPIO_OUTPUT(7, 3)
#define GPIO_P7_4_OUTPUT  RZV2H_GPIO_OUTPUT(7, 4)
#define GPIO_P7_5_OUTPUT  RZV2H_GPIO_OUTPUT(7, 5)
#define GPIO_P7_6_OUTPUT  RZV2H_GPIO_OUTPUT(7, 6)
#define GPIO_P7_7_OUTPUT  RZV2H_GPIO_OUTPUT(7, 7)

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

/* RIIC I2C pin configurations. */

#define BOARD_RIIC0_SCL_GPIO  GPIO_SCL0_P3_1_M1
#define BOARD_RIIC0_SDA_GPIO  GPIO_SDA0_P3_0_M1
#define BOARD_RIIC1_SCL_GPIO  GPIO_SCL1_P3_3_M1
#define BOARD_RIIC1_SDA_GPIO  GPIO_SDA1_P3_2_M1
#define BOARD_RIIC2_SCL_GPIO  GPIO_SCL2_P2_1_M4
#define BOARD_RIIC2_SDA_GPIO  GPIO_SDA2_P2_0_M4
#define BOARD_RIIC3_SCL_GPIO  GPIO_SCL3_P3_7_M1
#define BOARD_RIIC3_SDA_GPIO  GPIO_SDA3_P3_6_M1
#define BOARD_RIIC4_SCL_GPIO  GPIO_SCL4_P4_1_M1
#define BOARD_RIIC4_SDA_GPIO  GPIO_SDA4_P4_0_M1
#define BOARD_RIIC5_SCL_GPIO  GPIO_SCL5_P4_3_M1
#define BOARD_RIIC5_SDA_GPIO  GPIO_SDA5_P4_2_M1
#define BOARD_RIIC6_SCL_GPIO  GPIO_SCL6_P4_5_M1
#define BOARD_RIIC6_SDA_GPIO  GPIO_SDA6_P4_4_M1
#define BOARD_RIIC7_SCL_GPIO  GPIO_SCL7_P4_7_M1
#define BOARD_RIIC7_SDA_GPIO  GPIO_SDA7_P4_6_M1
#define BOARD_RIIC8_SCL_GPIO  GPIO_SCL8_P0_7_M1
#define BOARD_RIIC8_SDA_GPIO  GPIO_SDA8_P0_6_M1

/* SCI I2C pin configurations (SDAn = TXDn, SCLn = RXDn).
 * A channel can only be used for I2C when its SCI UART is disabled.
 * Both lines need external pull-up resistors.
 */

#define BOARD_SCI_I2C0_SDA_GPIO  GPIO_SCI_SDA0_P5_0_M1
#define BOARD_SCI_I2C0_SCL_GPIO  GPIO_SCI_SCL0_P5_1_M1
#define BOARD_SCI_I2C1_SDA_GPIO  GPIO_SCI_SDA1_P5_2_M1
#define BOARD_SCI_I2C1_SCL_GPIO  GPIO_SCI_SCL1_P5_3_M1
#define BOARD_SCI_I2C2_SDA_GPIO  GPIO_SCI_SDA2_P5_4_M1
#define BOARD_SCI_I2C2_SCL_GPIO  GPIO_SCI_SCL2_P5_5_M1
#define BOARD_SCI_I2C4_SDA_GPIO  GPIO_SCI_SDA4_P8_4_M6
#define BOARD_SCI_I2C4_SCL_GPIO  GPIO_SCI_SCL4_P8_5_M6
#define BOARD_SCI_I2C5_SDA_GPIO  GPIO_SCI_SDA5_P7_2_M1
#define BOARD_SCI_I2C5_SCL_GPIO  GPIO_SCI_SCL5_P7_3_M1
#define BOARD_SCI_I2C6_SDA_GPIO  GPIO_SCI_SDA6_P7_4_M1
#define BOARD_SCI_I2C6_SCL_GPIO  GPIO_SCI_SCL6_P7_5_M1
#define BOARD_SCI_I2C7_SDA_GPIO  GPIO_SCI_SDA7_P7_6_M1
#define BOARD_SCI_I2C7_SCL_GPIO  GPIO_SCI_SCL7_P7_7_M1
#define BOARD_SCI_I2C8_SDA_GPIO  GPIO_SCI_SDA8_P8_0_M6
#define BOARD_SCI_I2C8_SCL_GPIO  GPIO_SCI_SCL8_P8_1_M6
#define BOARD_SCI_I2C9_SDA_GPIO  GPIO_SCI_SDA9_P8_2_M6
#define BOARD_SCI_I2C9_SCL_GPIO  GPIO_SCI_SCL9_P8_3_M6

#ifdef CONFIG_RZV2H_SCI_I2C3
#  error "RZV2H-EVK does not route the SCI3 I2C pin P5_6 to a connector"
#endif

/* GPT PWM pin configurations. */

#define BOARD_GPT0_GTIOC_GPIO   GPIO_GTIOC0A_P7_0_M9
#define BOARD_GPT0_USE_GTIOCA   1
#define BOARD_GPT1_GTIOC_GPIO   GPIO_GTIOC1A_P8_0_M9
#define BOARD_GPT1_USE_GTIOCA   1
#define BOARD_GPT2_GTIOC_GPIO   GPIO_GTIOC2A_P4_4_M9
#define BOARD_GPT2_USE_GTIOCA   1
#define BOARD_GPT3_GTIOC_GPIO   GPIO_GTIOC3A_P4_6_M9
#define BOARD_GPT3_USE_GTIOCA   1
#define BOARD_GPT4_GTIOC_GPIO   GPIO_GTIOC4A_P9_4_M11
#define BOARD_GPT4_USE_GTIOCA   1
#define BOARD_GPT5_GTIOC_GPIO   GPIO_GTIOC5A_P9_6_M11
#define BOARD_GPT5_USE_GTIOCA   1
#define BOARD_GPT6_GTIOC_GPIO   GPIO_GTIOC6A_P3_4_M9
#define BOARD_GPT6_USE_GTIOCA   1
#define BOARD_GPT7_GTIOC_GPIO   GPIO_GTIOC7A_P8_6_M9
#define BOARD_GPT7_USE_GTIOCA   1
#define BOARD_GPT8_GTIOC_GPIO   GPIO_GTIOC8A_P9_4_M9
#define BOARD_GPT8_USE_GTIOCA   1
#define BOARD_GPT9_GTIOC_GPIO   GPIO_GTIOC9A_P9_6_M9
#define BOARD_GPT9_USE_GTIOCA   1
#define BOARD_GPT10_GTIOC_GPIO  GPIO_GTIOC10A_PA_4_M9
#define BOARD_GPT10_USE_GTIOCA  1
#define BOARD_GPT11_GTIOC_GPIO  GPIO_GTIOC11A_P6_2_M11
#define BOARD_GPT11_USE_GTIOCA  1
#define BOARD_GPT12_GTIOC_GPIO  GPIO_GTIOC12A_P3_0_M11
#define BOARD_GPT12_USE_GTIOCA  1
#define BOARD_GPT13_GTIOC_GPIO  GPIO_GTIOC13A_P6_4_M11
#define BOARD_GPT13_USE_GTIOCA  1
#define BOARD_GPT14_GTIOC_GPIO  GPIO_GTIOC14A_P3_5_M11
#define BOARD_GPT14_USE_GTIOCA  1
#define BOARD_GPT15_GTIOC_GPIO  GPIO_GTIOC15A_P6_6_M11
#define BOARD_GPT15_USE_GTIOCA  1

/* CANFD pin configurations. */

#define BOARD_CANFD0_TX_GPIO  GPIO_CANFD0_TX_P8_0_M5
#define BOARD_CANFD0_RX_GPIO  GPIO_CANFD0_RX_P8_1_M5
#define BOARD_CANFD0_STB_GPIO GPIO_P5_7_OUTPUT
#define BOARD_CANFD1_TX_GPIO  GPIO_CANFD1_TX_P8_2_M5
#define BOARD_CANFD1_RX_GPIO  GPIO_CANFD1_RX_P8_3_M5
#define BOARD_CANFD1_STB_GPIO GPIO_P7_3_OUTPUT
#define BOARD_CANFD2_TX_GPIO  GPIO_CANFD2_TX_P8_4_M5
#define BOARD_CANFD2_RX_GPIO  GPIO_CANFD2_RX_P8_5_M5
#define BOARD_CANFD2_STB_GPIO GPIO_P7_6_OUTPUT
#define BOARD_CANFD3_TX_GPIO  GPIO_CANFD3_TX_P8_6_M5
#define BOARD_CANFD3_RX_GPIO  GPIO_CANFD3_RX_P8_7_M5
#define BOARD_CANFD3_STB_GPIO GPIO_P7_7_OUTPUT
#define BOARD_CANFD4_TX_GPIO  GPIO_CANFD4_TX_P4_2_M5
#define BOARD_CANFD4_RX_GPIO  GPIO_CANFD4_RX_P4_3_M5
#define BOARD_CANFD4_STB_GPIO GPIO_P7_4_OUTPUT
#define BOARD_CANFD5_TX_GPIO  GPIO_CANFD5_TX_P4_6_M5
#define BOARD_CANFD5_RX_GPIO  GPIO_CANFD5_RX_P4_7_M5
#define BOARD_CANFD5_STB_GPIO GPIO_P7_5_OUTPUT

/* xSPI0 (QSPI NOR flash) pins.
 *
 */

#define BOARD_QSPI0_CKN_GPIO     GPIO_XSPI0_CKN
#define BOARD_QSPI0_CKP_GPIO     GPIO_XSPI0_CKP
#define BOARD_QSPI0_CS0N_GPIO    GPIO_XSPI0_CS0N
#define BOARD_QSPI0_CS1N_GPIO    GPIO_XSPI0_CS1N
#define BOARD_QSPI0_RESET0N_GPIO GPIO_XSPI0_RESET0N
#define BOARD_QSPI0_IO0_GPIO     GPIO_XSPI0_IO0
#define BOARD_QSPI0_IO1_GPIO     GPIO_XSPI0_IO1
#define BOARD_QSPI0_IO2_GPIO     GPIO_XSPI0_IO2
#define BOARD_QSPI0_IO3_GPIO     GPIO_XSPI0_IO3
#define BOARD_QSPI0_DS_GPIO      GPIO_XSPI0_DS

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

/****************************************************************************
 * Name: rzv2h_qspi_setup
 *
 * Description:
 *   Configure the xSPI0 pins (SPCLK/IO0-3/CS0) for QSPI operation.
 *
 ****************************************************************************/

#ifdef CONFIG_RZV2H_QSPI
void rzv2h_qspi_setup(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_RZV2H_RZV2H_EVK_INCLUDE_BOARD_H */
