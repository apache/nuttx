/****************************************************************************
 * boards/arm/cxd56xx/spresense/include/board_pinconfig.h
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

#ifndef __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_BOARD_PINCONFIG_H
#define __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_BOARD_PINCONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Customize from default to the board specific pin configuration
 * The default pin configurations are defined in
 * boards/arm/cxd56xx/spresense/include/board_pinconfig.h.
 *
 *   Mode: shared pin function mode
 *   ENZI: 1=Input Enable, 0=Input Disable
 *   4mA : Drive Current 1=4mA, 0=2mA
 *   Pull: 0=HiZ floating, PINCONF_PULLUP, PINCONF_PULLDOWN
 *
 *                                                            M  E     P
 *                                        P                   o  N  4  u
 *                                        i                   d  Z  m  l
 *                                        n                   e  I  A  l
 */

#undef PINCONF_UART2_TXD
#undef PINCONF_UART2_RXD
#undef PINCONF_UART2_CTS
#undef PINCONF_UART2_RTS
#define PINCONF_UART2_TXD          PINCONF(PIN_UART2_TXD,      1, 0, 1, 0)
#define PINCONF_UART2_RXD          PINCONF(PIN_UART2_RXD,      1, 1, 1, 0)
#define PINCONF_UART2_CTS          PINCONF(PIN_UART2_CTS,      1, 1, 1, PINCONF_PULLDOWN)
#define PINCONF_UART2_RTS          PINCONF(PIN_UART2_RTS,      1, 0, 1, 0)

#undef PINCONF_SPI4_CS_X
#undef PINCONF_SPI4_SCK
#undef PINCONF_SPI4_MOSI
#define PINCONF_SPI4_CS_X          PINCONF(PIN_SPI4_CS_X,      1, 0, 1, 0)
#define PINCONF_SPI4_SCK           PINCONF(PIN_SPI4_SCK,       1, 0, 1, 0)
#define PINCONF_SPI4_MOSI          PINCONF(PIN_SPI4_MOSI,      1, 0, 1, 0)

#undef PINCONF_SPI3_CS0_X
#undef PINCONF_SPI3_CS1_X
#undef PINCONF_SPI3_CS2_X
#undef PINCONF_SPI3_SCK
#undef PINCONF_SPI3_MOSI
#undef PINCONF_SPI3_MISO
#define PINCONF_SPI3_CS0_X         PINCONF(PIN_SPI3_CS0_X,     1, 0, 1, 0)
#define PINCONF_SPI3_CS1_X         PINCONF(PIN_SPI3_CS1_X,     1, 0, 1, 0)
#define PINCONF_SPI3_CS2_X         PINCONF(PIN_SPI3_CS2_X,     1, 0, 1, 0)
#define PINCONF_SPI3_SCK           PINCONF(PIN_SPI3_SCK,       1, 0, 1, 0)
#define PINCONF_SPI3_MOSI          PINCONF(PIN_SPI3_MOSI,      1, 0, 1, 0)
#define PINCONF_SPI3_MISO          PINCONF(PIN_SPI3_MISO,      1, 1, 1, 0)

#undef PINCONF_PWM0
#undef PINCONF_PWM1
#undef PINCONF_PWM2
#undef PINCONF_PWM3
#define PINCONF_PWM0               PINCONF(PIN_PWM0,           1, 0, 1, 0)
#define PINCONF_PWM1               PINCONF(PIN_PWM1,           1, 0, 1, 0)
#define PINCONF_PWM2               PINCONF(PIN_PWM2,           1, 0, 1, 0)
#define PINCONF_PWM3               PINCONF(PIN_PWM3,           1, 0, 1, 0)

#undef PINCONF_I2C0_BCK
#undef PINCONF_I2C0_BDT
#define PINCONF_I2C0_BCK           PINCONF(PIN_I2C0_BCK,       1, 1, 1, 0)
#define PINCONF_I2C0_BDT           PINCONF(PIN_I2C0_BDT,       1, 1, 1, 0)

#undef PINCONF_SDIO_CLK_GPIO
#undef PINCONF_SDIO_CMD_GPIO
#undef PINCONF_SDIO_DATA0_GPIO
#undef PINCONF_SDIO_DATA1_GPIO
#undef PINCONF_SDIO_DATA2_GPIO
#undef PINCONF_SDIO_DATA3_GPIO
#define PINCONF_SDIO_CLK_GPIO      PINCONF(PIN_SDIO_CLK,       0, 0, 0, PINCONF_PULLDOWN)
#define PINCONF_SDIO_CMD_GPIO      PINCONF(PIN_SDIO_CMD,       0, 0, 0, PINCONF_PULLDOWN)
#define PINCONF_SDIO_DATA0_GPIO    PINCONF(PIN_SDIO_DATA0,     0, 0, 0, PINCONF_PULLDOWN)
#define PINCONF_SDIO_DATA1_GPIO    PINCONF(PIN_SDIO_DATA1,     0, 0, 0, PINCONF_PULLDOWN)
#define PINCONF_SDIO_DATA2_GPIO    PINCONF(PIN_SDIO_DATA2,     0, 0, 0, PINCONF_PULLDOWN)
#define PINCONF_SDIO_DATA3_GPIO    PINCONF(PIN_SDIO_DATA3,     0, 0, 0, PINCONF_PULLDOWN)

#undef PINCONF_SDIO_CD_GPIO
#define PINCONF_SDIO_CD_GPIO       PINCONF(PIN_SDIO_CD,        0, 0, 0, PINCONF_PULLUP)

#endif /* __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_BOARD_PINCONFIG_H */
