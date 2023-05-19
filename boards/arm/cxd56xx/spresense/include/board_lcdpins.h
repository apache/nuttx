/****************************************************************************
 * boards/arm/cxd56xx/spresense/include/board_lcdpins.h
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

#ifndef __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_BOARD_LCDPINS_H
#define __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_BOARD_LCDPINS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ILI934X RST pin definition */

#if defined(CONFIG_LCD_RSTPIN_UART2_TX) \
    || defined(CONFIG_LCD_RSTPIN_UART2_TX_E)
#  define ILI934X_RST_PIN PIN_UART2_TXD
#elif defined(CONFIG_LCD_RSTPIN_UART2_RX) \
    || defined(CONFIG_LCD_RSTPIN_UART2_RX_E)
#  define ILI934X_RST_PIN PIN_UART2_RXD
#elif defined(CONFIG_LCD_RSTPIN_UART2_RTS)
#  define ILI934X_RST_PIN PIN_UART2_RTS
#elif defined(CONFIG_LCD_RSTPIN_UART2_CTS)
#  define ILI934X_RST_PIN PIN_UART2_CTS
#elif defined(CONFIG_LCD_RSTPIN_I2S0_BCK)
#  define ILI934X_RST_PIN PIN_I2S0_BCK
#elif defined(CONFIG_LCD_RSTPIN_I2S0_LRCK)
#  define ILI934X_RST_PIN PIN_I2S0_LRCK
#elif defined(CONFIG_LCD_RSTPIN_SEN_IRQ_IN)
#  define ILI934X_RST_PIN PIN_SEN_IRQ_IN
#elif defined(CONFIG_LCD_RSTPIN_EMMC_DATA3)
#  define ILI934X_RST_PIN PIN_EMMC_DATA3
#elif defined(CONFIG_LCD_RSTPIN_EMMC_DATA2)
#  define ILI934X_RST_PIN PIN_EMMC_DATA2
#elif defined(CONFIG_LCD_RSTPIN_I2S0_DATA_IN)
#  define ILI934X_RST_PIN PIN_I2S0_DATA_IN
#elif defined(CONFIG_LCD_RSTPIN_I2S0_DATA_OUT)
#  define ILI934X_RST_PIN PIN_I2S0_DATA_OUT
#elif defined(CONFIG_LCD_RSTPIN_I2C0_SCL) \
    || defined(CONFIG_LCD_RSTPIN_I2C0_SCL_E)
#  define ILI934X_RST_PIN PIN_I2C0_BCK
#elif defined(CONFIG_LCD_RSTPIN_I2C0_SDA) \
    || defined(CONFIG_LCD_RSTPIN_I2C0_SDA_E)
#  define ILI934X_RST_PIN PIN_I2C0_BDT
#elif defined(CONFIG_LCD_RSTPIN_PWM2)
#  define ILI934X_RST_PIN PIN_PWM2
#elif defined(CONFIG_LCD_RSTPIN_SPI2_MISO)
#  define ILI934X_RST_PIN PIN_SPI2_MISO
#elif defined(CONFIG_LCD_RSTPIN_SPI3_CS1_X)
#  define ILI934X_RST_PIN PIN_SPI3_CS1_X
#elif defined(CONFIG_LCD_RSTPIN_PWM0)
#  define ILI934X_RST_PIN PIN_PWM0
#elif defined(CONFIG_LCD_RSTPIN_PWM1)
#  define ILI934X_RST_PIN PIN_PWM1
#elif defined(CONFIG_LCD_RSTPIN_SPI2_MOSI)
#  define ILI934X_RST_PIN PIN_SPI2_MOSI
#elif defined(CONFIG_LCD_RSTPIN_PWM3)
#  define ILI934X_RST_PIN PIN_PWM3
#elif defined(CONFIG_LCD_RSTPIN_HIF_IRQ_OUT)
#  define ILI934X_RST_PIN PIN_HIF_IRQ_OUT
#endif

/* ILI934X DC pin definition */

#if defined(CONFIG_LCD_DCPIN_UART2_TX) \
    || defined(CONFIG_LCD_DCPIN_UART2_TX_E)
#  define ILI934X_DC_PIN PIN_UART2_TXD
#elif defined(CONFIG_LCD_DCPIN_UART2_RX) \
    || defined(CONFIG_LCD_DCPIN_UART2_RX_E)
#  define ILI934X_DC_PIN PIN_UART2_RXD
#elif defined(CONFIG_LCD_DCPIN_UART2_RTS)
#  define ILI934X_DC_PIN PIN_UART2_RTS
#elif defined(CONFIG_LCD_DCPIN_UART2_CTS)
#  define ILI934X_DC_PIN PIN_UART2_CTS
#elif defined(CONFIG_LCD_DCPIN_I2S0_BCK)
#  define ILI934X_DC_PIN PIN_I2S0_BCK
#elif defined(CONFIG_LCD_DCPIN_I2S0_LRCK)
#  define ILI934X_DC_PIN PIN_I2S0_LRCK
#elif defined(CONFIG_LCD_DCPIN_SEN_IRQ_IN)
#  define ILI934X_DC_PIN PIN_SEN_IRQ_IN
#elif defined(CONFIG_LCD_DCPIN_EMMC_DATA3)
#  define ILI934X_DC_PIN PIN_EMMC_DATA3
#elif defined(CONFIG_LCD_DCPIN_EMMC_DATA2)
#  define ILI934X_DC_PIN PIN_EMMC_DATA2
#elif defined(CONFIG_LCD_DCPIN_I2S0_DATA_IN)
#  define ILI934X_DC_PIN PIN_I2S0_DATA_IN
#elif defined(CONFIG_LCD_DCPIN_I2S0_DATA_OUT)
#  define ILI934X_DC_PIN PIN_I2S0_DATA_OUT
#elif defined(CONFIG_LCD_DCPIN_I2C0_SCL) \
    || defined(CONFIG_LCD_DCPIN_I2C0_SCL_E)
#  define ILI934X_DC_PIN PIN_I2C0_BCK
#elif defined(CONFIG_LCD_DCPIN_I2C0_SDA) \
    || defined(CONFIG_LCD_DCPIN_I2C0_SDA_E)
#  define ILI934X_DC_PIN PIN_I2C0_BDT
#elif defined(CONFIG_LCD_DCPIN_PWM2)
#  define ILI934X_DC_PIN PIN_PWM2
#elif defined(CONFIG_LCD_DCPIN_SPI2_MISO)
#  define ILI934X_DC_PIN PIN_SPI2_MISO
#elif defined(CONFIG_LCD_DCPIN_SPI3_CS1_X)
#  define ILI934X_DC_PIN PIN_SPI3_CS1_X
#elif defined(CONFIG_LCD_DCPIN_PWM0)
#  define ILI934X_DC_PIN PIN_PWM0
#elif defined(CONFIG_LCD_DCPIN_PWM1)
#  define ILI934X_DC_PIN PIN_PWM1
#elif defined(CONFIG_LCD_DCPIN_SPI2_MOSI)
#  define ILI934X_DC_PIN PIN_SPI2_MOSI
#elif defined(CONFIG_LCD_DCPIN_PWM3)
#  define ILI934X_DC_PIN PIN_PWM3
#elif defined(CONFIG_LCD_DCPIN_HIF_IRQ_OUT)
#  define ILI934X_DC_PIN PIN_HIF_IRQ_OUT
#endif

#if !defined(CONFIG_LCD)

#  if !defined(ILI934X_RST_PIN)
#    define ILI934X_RST_PIN 0
#  endif

#  if !defined(ILI934X_DC_PIN)
#    define ILI934X_DC_PIN 0
#  endif

#endif

#endif  /* __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_BOARD_LCDPINS_H */
