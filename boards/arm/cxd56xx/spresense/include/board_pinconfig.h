/****************************************************************************
 * boards/arm/cxd56xx/spresense/include/board_pinconfig.h
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
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
