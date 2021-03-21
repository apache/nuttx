/****************************************************************************
 * arch/arm/include/cxd56xx/pin.h
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

#ifndef __ARCH_ARM_INCLUDE_CXD56XX_PIN_H
#define __ARCH_ARM_INCLUDE_CXD56XX_PIN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* Pin number Prototypes */

#define PIN_RTC_CLK_IN          (0)

/* SYS GPIO: system power domain GPIOs */

#define PIN_I2C4_BCK            (1)
#define PIN_I2C4_BDT            (2)
#define PIN_PMIC_INT            (3)
#define PIN_RTC_IRQ_OUT         (4)
#define PIN_AP_CLK              (5)
#define PIN_GNSS_1PPS_OUT       (6)
#define PIN_SPI0_CS_X           (17)
#define PIN_SPI0_SCK            (18)
#define PIN_SPI0_MOSI           (19)
#define PIN_SPI0_MISO           (20)
#define PIN_SPI1_CS_X           (21)
#define PIN_SPI1_SCK            (22)
#define PIN_SPI1_IO0            (23)
#define PIN_SPI1_IO1            (24)
#define PIN_SPI1_IO2            (25)
#define PIN_SPI1_IO3            (26)
#define PIN_SPI2_CS_X           (27)
#define PIN_SPI2_SCK            (28)
#define PIN_SPI2_MOSI           (29)
#define PIN_SPI2_MISO           (30)
#define PIN_HIF_IRQ_OUT         (31)
#define PIN_HIF_GPIO0           (32)
#define PIN_SEN_IRQ_IN          (37)
#define PIN_SPI3_CS0_X          (38)
#define PIN_SPI3_CS1_X          (39)
#define PIN_SPI3_CS2_X          (40)
#define PIN_SPI3_SCK            (41)
#define PIN_SPI3_MOSI           (42)
#define PIN_SPI3_MISO           (43)
#define PIN_I2C0_BCK            (44)
#define PIN_I2C0_BDT            (45)
#define PIN_PWM0                (46)
#define PIN_PWM1                (47)
#define PIN_PWM2                (48)
#define PIN_PWM3                (49)

/* APP GPIO: application power domain GPIOs */

#define PIN_IS_CLK              (56)
#define PIN_IS_VSYNC            (57)
#define PIN_IS_HSYNC            (58)
#define PIN_IS_DATA0            (59)
#define PIN_IS_DATA1            (60)
#define PIN_IS_DATA2            (61)
#define PIN_IS_DATA3            (62)
#define PIN_IS_DATA4            (63)
#define PIN_IS_DATA5            (64)
#define PIN_IS_DATA6            (65)
#define PIN_IS_DATA7            (66)
#define PIN_UART2_TXD           (67)
#define PIN_UART2_RXD           (68)
#define PIN_UART2_CTS           (69)
#define PIN_UART2_RTS           (70)
#define PIN_SPI4_CS_X           (71)
#define PIN_SPI4_SCK            (72)
#define PIN_SPI4_MOSI           (73)
#define PIN_SPI4_MISO           (74)
#define PIN_EMMC_CLK            (75)
#define PIN_SPI5_SCK            (PIN_EMMC_CLK)
#define PIN_EMMC_CMD            (76)
#define PIN_SPI5_CS_X           (PIN_EMMC_CMD)
#define PIN_EMMC_DATA0          (77)
#define PIN_SPI5_MOSI           (PIN_EMMC_DATA0)
#define PIN_EMMC_DATA1          (78)
#define PIN_SPI5_MISO           (PIN_EMMC_DATA1)
#define PIN_EMMC_DATA2          (79)
#define PIN_EMMC_DATA3          (80)
#define PIN_SDIO_CLK            (81)
#define PIN_SDIO_CMD            (82)
#define PIN_SDIO_DATA0          (83)
#define PIN_SDIO_DATA1          (84)
#define PIN_SDIO_DATA2          (85)
#define PIN_SDIO_DATA3          (86)
#define PIN_SDIO_CD             (87)
#define PIN_SDIO_WP             (88)
#define PIN_SDIO_CMDDIR         (89)
#define PIN_SDIO_DIR0           (90)
#define PIN_SDIO_DIR1_3         (91)
#define PIN_SDIO_CLKI           (92)
#define PIN_I2S0_BCK            (93)
#define PIN_I2S0_LRCK           (94)
#define PIN_I2S0_DATA_IN        (95)
#define PIN_I2S0_DATA_OUT       (96)
#define PIN_I2S1_BCK            (97)
#define PIN_I2S1_LRCK           (98)
#define PIN_I2S1_DATA_IN        (99)
#define PIN_I2S1_DATA_OUT       (100)
#define PIN_MCLK                (101)
#define PIN_PDM_CLK             (102)
#define PIN_PDM_IN              (103)
#define PIN_PDM_OUT             (104)
#define PIN_USB_VBUSINT         (105)

#endif /* __ARCH_ARM_INCLUDE_CXD56XX_PIN_H */
