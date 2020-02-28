/****************************************************************************
 * arch/arm/src/cxd56xx/hardware/cxd5602_pinconfig.h
 *
 *   Copyright (C) 2008-2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD5602_PINCONFIG_H
#define __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD5602_PINCONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Set the standard pinconf macro Definitions
 *  - If it's used as input pin, then set 1. Otherwise set 0 (default).
 *  - If it's drived in 4mA, then set 1. Otherwise set 0 (default 2mA).
 *  - If it's used as weak pull-up/down, then set PINCONF_PULLUP/PINCONF_PULLDOWN.
 *    Otherwise set 0 (default).
 *
 */
#define PINCONF(pin, mode, input, drive, pull) \
  ( \
    (PINCONF_SET_PIN(pin)) | \
    (PINCONF_SET_MODE(mode)) | \
    ((input) ? PINCONF_INPUT_ENABLE : PINCONF_INPUT_DISABLE) | \
    ((drive) ?  PINCONF_DRIVE_HIGH : PINCONF_DRIVE_NORMAL) | \
    ((pull) ? (pull) : PINCONF_FLOAT) \
    )

/* CXD5602 Pin Configuration Table
 *
 * Group            Pin             100 185 Mode0   Mode1           Mode2           Mode3
 * ================ =============== === === ======= =============== =============== ===============
 * I2C4             I2C4_BCK        o   o   GPIO    I2C4(PMIC)      -               -
 *                  I2C4_BDT        o   o   GPIO    I2C4(PMIC)      -               -
 * ---------------- --------------- --- --- ------- --------------- --------------- ---------------
 * PMIC_INT         PMIC_INT        o   o   GPIO    PMIC_INT        PMIC_INT(OD)    -
 * ---------------- --------------- --- --- ------- --------------- --------------- ---------------
 * RTC_IRQ_OUT      RTC_IRQ_OUT     -   o   GPIO    RTC_IRQ_OUT     RTC_IRQ_OUT(OD) -
 * ---------------- --------------- --- --- ------- --------------- --------------- ---------------
 * AP_CLK           AP_CLK          o   o   GPIO    AP_CLK          PMU_WDT         PMU_WDT(OD)
 * ---------------- --------------- --- --- ------- --------------- --------------- ---------------
 * GNSS_1PPS_OUT    GNSS_1PPS_OUT   -   o   GPIO    GNSS_1PPS_OUT   CPU_WDT         CPU_WDT(OD)
 * ---------------- --------------- --- --- --------------- ------- --------------- ---------------
 * SPI0A            SPI0_CS_X       o   o   GPIO    UART1(DBG)      SPI0(CFG)       SYS_MONOUT0
 *                  SPI0_SCK        o   o   GPIO    UART1(DBG)      SPI0(CFG)       SYS_MONOUT1
 * ---------------- --------------- --- --- ------- ---------------                 ---------------
 * SPI0B            SPI0_MOSI       -   o   GPIO    I2C2(CFG)       SPI0(CFG)       SYS_MONOUT2
 *                  SPI0_MISO       -   o   GPIO    I2C2(CFG)       SPI0(CFG)       SYS_MONOUT3
 * ---------------- --------------- --- --- ------- --------------- --------------- ---------------
 * SPI1A            SPI1_CS_X       o   o   GPIO    SPI1(Flash)     SPI0(CFG)       SYS_MONOUT4
 *                  SPI1_SCK        o   o   GPIO    SPI1(Flash)     SPI0(CFG)       SYS_MONOUT5
 *                  SPI1_IO0        o   o   GPIO    SPI1(Flash)     SPI0(CFG)       SYS_MONOUT6
 *                  SPI1_IO1        o   o   GPIO    SPI1(Flash)     SPI0(CFG)       SYS_MONOUT7
 * ---------------- --------------- --- --- -------                 --------------- ---------------
 * SPI1B            SPI1_IO2        o   o   GPIO    SPI1(Flash)     -               SYS_MONOUT8
 *                  SPI1_IO3        o   o   GPIO    SPI1(Flash)     -               SYS_MONOUT9
 * ---------------- --------------- --- --- ------- --------------- --------------- ---------------
 * SPI2A            SPI2_CS_X       o   o   GPIO    SPI2(HostIF)    UART0(HostIF)   I2C3(HostIF)
 *                  SPI2_SCK        o   o   GPIO    SPI2(HostIF)    UART0(HostIF)   I2C3(HostIF)
 * ---------------- --------------- --- --- -------                 --------------- ---------------
 * SPI2B            SPI2_MOSI       o   o   GPIO    SPI2(HostIF)    UART0(HostIF)   -
 *                  SPI2_MISO       o   o   GPIO    SPI2(HostIF)    UART0(HostIF)   -
 * ---------------- --------------- --- --- ------- --------------- --------------- ---------------
 * HIFIRQ           HIF_IRQ_OUT     o   o   GPIO    HIF_IRQ_OUT     HIF_IRQ_OUT(OD) GNSS_1PPS_OUT
 * ---------------- --------------- --- --- ------- --------------- --------------- ---------------
 * HIFEXT           HIF_GPIO0       -   o   GPIO    -               -               GPS_EXTLD
 * ---------------- --------------- --- --- ------- --------------- --------------- ---------------
 * SEN_IRQ_IN       SEN_IRQ_IN      o   o   GPIO    SEN_IRQ_IN      SYS_MONOUT0     DBG_LOGGER
 * ---------------- --------------- --- --- ------- --------------- --------------- ---------------
 * SPI3_CS0_X       SPI3_CS0_X      o   o   GPIO    SPI3_CS0_X      SYS_MONOUT1     DBG_LOGGER
 * ---------------- --------------- --- --- ------- --------------- --------------- ---------------
 * SPI3_CS1_X       SPI3_CS1_X      o   o   GPIO    SPI3_CS1_X      SYS_MONOUT2     DBG_LOGGER
 * ---------------- --------------- --- --- ------- --------------- --------------- ---------------
 * SPI3_CS2_X       SPI3_CS2_X      o   o   GPIO    SPI3_CS2_X      SYS_MONOUT3     DBG_LOGGER
 * ---------------- --------------- --- --- ------- --------------- --------------- ---------------
 * SPI3             SPI3_SCK        o   o   GPIO    SPI3(Sensor)    SYS_MONOUT4     DBG_LOGGER
 *                  SPI3_MOSI       o   o   GPIO    SPI3(Sensor)    SYS_MONOUT5     DBG_LOGGER
 *                  SPI3_MISO       o   o   GPIO    SPI3(Sensor)    SYS_MONOUT6     DBG_LOGGER
 * ---------------- --------------- --- --- ------- --------------- --------------- ---------------
 * I2C0             I2C0_BCK        o   o   GPIO    I2C0(Sensor)    SYS_MONOUT7     DBG_LOGGER
 *                  I2C0_BDT        o   o   GPIO    I2C0(Sensor)    SYS_MONOUT8     DBG_LOGGER
 * ---------------- --------------- --- --- ------- --------------- --------------- ---------------
 * PWMA             PWM0            o   o   GPIO    PWMA            SYS_MONOUT9     DBG_LOGGER
 *                  PWM1            o   o   GPIO    PWMA            GPIO            DBG_LOGGER
 * ---------------- --------------- --- --- ------- --------------- --------------- ---------------
 * PWMB             PWM2            o   o   GPIO    PWMB            I2C1(Sensor)    DBG_LOGGER
 *                  PWM3            o   o   GPIO    PWMB            I2C1(Sensor)    DBG_LOGGER
 * ---------------- --------------- --- --- ------- --------------- --------------- ---------------
 * IS               IS_CLK          -   o   GPIO    IS              -               -
 *                  IS_VSYNC        -   o   GPIO    IS              -               -
 *                  IS_HSYNC        -   o   GPIO    IS              -               -
 *                  IS_DATA0        -   o   GPIO    IS              -               -
 *                  IS_DATA1        -   o   GPIO    IS              -               -
 *                  IS_DATA2        -   o   GPIO    IS              -               -
 *                  IS_DATA3        -   o   GPIO    IS              -               -
 *                  IS_DATA4        -   o   GPIO    IS              -               -
 *                  IS_DATA5        -   o   GPIO    IS              -               -
 *                  IS_DATA6        -   o   GPIO    IS              -               -
 *                  IS_DATA7        -   o   GPIO    IS              -               -
 * ---------------- --------------- --- --- ------- --------------- --------------- ---------------
 * UART2            UART2_TXD       o   o   GPIO    UART2(APP)      APP_MONOUT0     -
 *                  UART2_RXD       o   o   GPIO    UART2(APP)      APP_MONOUT1     -
 *                  UART2_CTS       o   o   GPIO    UART2(APP)      APP_MONOUT2     -
 *                  UART2_RTS       o   o   GPIO    UART2(APP)      APP_MONOUT3     -
 * ---------------- --------------- --- --- ------- --------------- --------------- ---------------
 * SPI4             SPI4_CS_X       o   o   GPIO    SPI4(APP)       APP_MONOUT4     -
 *                  SPI4_SCK        o   o   GPIO    SPI4(APP)       APP_MONOUT5     -
 *                  SPI4_MOSI       o   o   GPIO    SPI4(APP)       APP_MONOUT6     -
 *                  SPI4_MISO       o   o   GPIO    SPI4(APP)       APP_MONOUT7     -
 * ---------------- --------------- --- --- ------- --------------- --------------- ---------------
 * EMMCA            EMMC_CLK        o   o   GPIO    EMMC            SPI5(APP)       -
 *                  EMMC_CMD        o   o   GPIO    EMMC            SPI5(APP)       -
 *                  EMMC_DATA0      o   o   GPIO    EMMC            SPI5(APP)       -
 *                  EMMC_DATA1      o   o   GPIO    EMMC            SPI5(APP)       -
 * ---------------- --------------- --- --- -------                 --------------- ---------------
 * EMMCB            EMMC_DATA2      o   o   GPIO    EMMC            APP_MONOUT8     -
 *                  EMMC_DATA3      o   o   GPIO    EMMC            APP_MONOUT9     -
 * ---------------- --------------- --- --- ------- --------------- --------------- ---------------
 * SDIOA            SDIO_CLK        -   o   GPIO    SDIO            SPI5(APP)       -
 *                  SDIO_CMD        -   o   GPIO    SDIO            SPI5(APP)       -
 *                  SDIO_DATA0      -   o   GPIO    SDIO            SPI5(APP)       -
 *                  SDIO_DATA1      -   o   GPIO    SDIO            SPI5(APP)       -
 *                  SDIO_DATA2      -   o   GPIO    SDIO            GPIO            -
 *                  SDIO_DATA3      -   o   GPIO    SDIO            GPIO            -
 * ---------------- --------------- --- --- ------- --------------- --------------- ---------------
 * SDIOB            SDIO_CD         -   o   GPIO    SDIO            -               -
 *                  SDIO_WP         -   o   GPIO    SDIO            -               -
 * ---------------- --------------- --- --- ------- --------------- --------------- ---------------
 * SDIOC            SDIO_CMDDIR     -   o   GPIO    SDIO            -               -
 *                  SDIO_DIR0       -   o   GPIO    SDIO            -               -
 *                  SDIO_DIR1_3     -   o   GPIO    SDIO            -               -
 * ---------------- --------------- --- --- ------- --------------- --------------- ---------------
 * SDIOD            SDIO_CLKI       -   o   GPIO    SDIO            -               -
 * ---------------- --------------- --- --- ------- --------------- --------------- ---------------
 * I2S0             I2S0_BCK        o   o   GPIO    I2S0            APP_MONOUT0     -
 *                  I2S0_LRCK       o   o   GPIO    I2S0            APP_MONOUT1     -
 *                  I2S0_DATA_IN    o   o   GPIO    I2S0            APP_MONOUT2     -
 *                  I2S0_DATA_OUT   o   o   GPIO    I2S0            APP_MONOUT3     -
 * ---------------- --------------- --- --- ------- --------------- --------------- ---------------
 * I2S1             I2S1_BCK        -   o   GPIO    I2S1            APP_MONOUT4     -
 *                  I2S1_LRCK       -   o   GPIO    I2S1            APP_MONOUT5     -
 *                  I2S1_DATA_IN    -   o   GPIO    I2S1            APP_MONOUT6     -
 *                  I2S1_DATA_OUT   -   o   GPIO    I2S1            APP_MONOUT7     -
 * ---------------- --------------- --- --- ------- --------------- --------------- ---------------
 * MCLK             MCLK            o   o   GPIO    MCLK            APP_MONOUT8     -
 * ---------------- --------------- --- --- ------- --------------- --------------- ---------------
 * PDM              PDM_CLK         o   o   GPIO    PDM             APP_MONOUT9     -
 *                  PDM_IN          o   o   GPIO    PDM             GPIO            -
 *                  PDM_OUT         o   o   GPIO    PDM             GPIO            -
 * ---------------- --------------- --- --- ------- --------------- --------------- ---------------
 * USBVBUS          USB_VBUSINT     o   o   GPIO    USB_VBUSINT     -               DBG_LOGGER
 * ---------------- --------------- --- --- ------- --------------- --------------- ---------------
 */

/* Default pin configurations
 *  Mode: shared pin function mode
 *  ENZI: 1=Input Enable, 0=Input Disable
 *  4mA : Drive Current 1=4mA, 0=2mA
 *  Pull: 0=HiZ floating, PINCONF_PULLUP, PINCONF_PULLDOWN
 *                                                                      M  E     P
 *                                                  P                   o  N  4  u
 *                                                  i                   d  Z  m  l
 *                                                  n                   e  I  A  l
 */

#define PINCONF_I2C4_BCK_GPIO               PINCONF(PIN_I2C4_BCK,       0, 0, 0, 0)
#define PINCONF_I2C4_BCK                    PINCONF(PIN_I2C4_BCK,       1, 1, 0, 0)
#define PINCONF_I2C4_BDT_GPIO               PINCONF(PIN_I2C4_BDT,       0, 0, 0, 0)
#define PINCONF_I2C4_BDT                    PINCONF(PIN_I2C4_BDT,       1, 1, 0, 0)
#define PINCONF_PMIC_INT_GPIO               PINCONF(PIN_PMIC_INT,       0, 0, 0, 0)
#define PINCONF_PMIC_INT                    PINCONF(PIN_PMIC_INT,       1, 1, 0, 0)
#define PINCONF_PMIC_INT_OD                 PINCONF(PIN_PMIC_INT,       2, 1, 0, 0)
#ifdef CONFIG_CXD56_FCBGA
#define PINCONF_RTC_IRQ_OUT_GPIO            PINCONF(PIN_RTC_IRQ_OUT,    0, 0, 0, 0)
#define PINCONF_RTC_IRQ_OUT                 PINCONF(PIN_RTC_IRQ_OUT,    1, 0, 0, 0)
#define PINCONF_RTC_IRQ_OUT_OD              PINCONF(PIN_RTC_IRQ_OUT,    2, 0, 0, 0)
#endif /* CONFIG_CXD56_FCBGA */
#define PINCONF_AP_CLK_GPIO                 PINCONF(PIN_AP_CLK,         0, 0, 0, 0)
#define PINCONF_AP_CLK                      PINCONF(PIN_AP_CLK,         1, 1, 0, 0)
#define PINCONF_AP_CLK_PMU_WDT              PINCONF(PIN_AP_CLK,         2, 0, 0, 0)
#define PINCONF_AP_CLK_PMU_WDT_OD           PINCONF(PIN_AP_CLK,         3, 0, 0, 0)
#ifdef CONFIG_CXD56_FCBGA
#define PINCONF_GNSS_1PPS_OUT_GPIO          PINCONF(PIN_GNSS_1PPS_OUT,  0, 0, 0, 0)
#define PINCONF_GNSS_1PPS_OUT               PINCONF(PIN_GNSS_1PPS_OUT,  1, 0, 0, 0)
#define PINCONF_GNSS_1PPS_OUT_CPU_WDT       PINCONF(PIN_GNSS_1PPS_OUT,  2, 0, 0, 0)
#define PINCONF_GNSS_1PPS_OUT_CPU_WDT_OD    PINCONF(PIN_GNSS_1PPS_OUT,  3, 0, 0, 0)
#endif /* CONFIG_CXD56_FCBGA */
#define PINCONF_SPI0_CS_X_GPIO              PINCONF(PIN_SPI0_CS_X,      0, 0, 0, 0)
#define PINCONF_SPI0_CS_X_UART1_TXD         PINCONF(PIN_SPI0_CS_X,      1, 0, 0, 0)
#define PINCONF_SPI0_CS_X                   PINCONF(PIN_SPI0_CS_X,      2, 0, 0, 0)
#define PINCONF_SPI0_CS_X_SYS_MONOUT0       PINCONF(PIN_SPI0_CS_X,      3, 0, 0, 0)
#define PINCONF_SPI0_SCK_GPIO               PINCONF(PIN_SPI0_SCK,       0, 0, 0, 0)
#define PINCONF_SPI0_SCK_UART1_RXD          PINCONF(PIN_SPI0_SCK,       1, 1, 0, 0)
#define PINCONF_SPI0_SCK                    PINCONF(PIN_SPI0_SCK,       2, 0, 0, 0)
#define PINCONF_SPI0_SCK_SYS_MONOUT1        PINCONF(PIN_SPI0_SCK,       3, 0, 0, 0)
#ifdef CONFIG_CXD56_FCBGA
#define PINCONF_SPI0_MOSI_GPIO              PINCONF(PIN_SPI0_MOSI,      0, 0, 0, 0)
#define PINCONF_SPI0_MOSI_I2C2_BCK          PINCONF(PIN_SPI0_MOSI,      1, 1, 0, 0)
#define PINCONF_SPI0_MOSI                   PINCONF(PIN_SPI0_MOSI,      2, 0, 0, 0)
#define PINCONF_SPI0_MOSI_SYS_MONOUT2       PINCONF(PIN_SPI0_MOSI,      3, 0, 0, 0)
#define PINCONF_SPI0_MISO_GPIO              PINCONF(PIN_SPI0_MISO,      0, 0, 0, 0)
#define PINCONF_SPI0_MISO_I2C2_BDT          PINCONF(PIN_SPI0_MISO,      1, 1, 0, 0)
#define PINCONF_SPI0_MISO                   PINCONF(PIN_SPI0_MISO,      2, 1, 0, 0)
#define PINCONF_SPI0_MISO_SYS_MONOUT3       PINCONF(PIN_SPI0_MISO,      3, 0, 0, 0)
#endif /* CONFIG_CXD56_FCBGA */
#define PINCONF_SPI1_CS_X_GPIO              PINCONF(PIN_SPI1_CS_X,      0, 0, 0, 0)
#define PINCONF_SPI1_CS_X                   PINCONF(PIN_SPI1_CS_X,      1, 0, 1, 0)
#define PINCONF_SPI1_CS_X_SPI0_CS_X         PINCONF(PIN_SPI1_CS_X,      2, 0, 0, 0)
#define PINCONF_SPI1_CS_X_SYS_MONOUT4       PINCONF(PIN_SPI1_CS_X,      3, 0, 0, 0)
#define PINCONF_SPI1_SCK_GPIO               PINCONF(PIN_SPI1_SCK,       0, 0, 0, 0)
#define PINCONF_SPI1_SCK                    PINCONF(PIN_SPI1_SCK,       1, 0, 1, 0)
#define PINCONF_SPI1_SCK_SPI0_SCK           PINCONF(PIN_SPI1_SCK,       2, 0, 0, 0)
#define PINCONF_SPI1_SCK_SYS_MONOUT5        PINCONF(PIN_SPI1_SCK,       3, 0, 0, 0)
#define PINCONF_SPI1_IO0_GPIO               PINCONF(PIN_SPI1_IO0,       0, 0, 0, 0)
#define PINCONF_SPI1_IO0                    PINCONF(PIN_SPI1_IO0,       1, 1, 1, 0)
#define PINCONF_SPI1_IO0_SPI0_MOSI          PINCONF(PIN_SPI1_IO0,       2, 0, 0, 0)
#define PINCONF_SPI1_IO0_SYS_MONOUT6        PINCONF(PIN_SPI1_IO0,       3, 0, 0, 0)
#define PINCONF_SPI1_IO1_GPIO               PINCONF(PIN_SPI1_IO1,       0, 0, 0, 0)
#define PINCONF_SPI1_IO1                    PINCONF(PIN_SPI1_IO1,       1, 1, 1, 0)
#define PINCONF_SPI1_IO1_SPI0_MISO          PINCONF(PIN_SPI1_IO1,       2, 1, 0, 0)
#define PINCONF_SPI1_IO1_SYS_MONOUT7        PINCONF(PIN_SPI1_IO1,       3, 0, 0, 0)
#define PINCONF_SPI1_IO2_GPIO               PINCONF(PIN_SPI1_IO2,       0, 0, 0, 0)
#define PINCONF_SPI1_IO2                    PINCONF(PIN_SPI1_IO2,       1, 1, 1, 0)
#define PINCONF_SPI1_IO2_SYS_MONOUT8        PINCONF(PIN_SPI1_IO2,       3, 0, 0, 0)
#define PINCONF_SPI1_IO3_GPIO               PINCONF(PIN_SPI1_IO3,       0, 0, 0, 0)
#define PINCONF_SPI1_IO3                    PINCONF(PIN_SPI1_IO3,       1, 1, 1, 0)
#define PINCONF_SPI1_IO3_SYS_MONOUT9        PINCONF(PIN_SPI1_IO3,       3, 0, 0, 0)
#define PINCONF_SPI2_CS_X_GPIO              PINCONF(PIN_SPI2_CS_X,      0, 0, 0, 0)
#define PINCONF_SPI2_CS_X                   PINCONF(PIN_SPI2_CS_X,      1, 1, 0, 0)
#define PINCONF_SPI2_CS_X_UART0_TXD         PINCONF(PIN_SPI2_CS_X,      2, 0, 0, 0)
#define PINCONF_SPI2_CS_X_I2C3_BCK          PINCONF(PIN_SPI2_CS_X,      3, 1, 0, 0)
#define PINCONF_SPI2_SCK_GPIO               PINCONF(PIN_SPI2_SCK,       0, 0, 0, 0)
#define PINCONF_SPI2_SCK                    PINCONF(PIN_SPI2_SCK,       1, 1, 0, 0)
#define PINCONF_SPI2_SCK_UART0_RXD          PINCONF(PIN_SPI2_SCK,       2, 1, 0, 0)
#define PINCONF_SPI2_SCK_I2C3_BDT           PINCONF(PIN_SPI2_SCK,       3, 1, 0, 0)
#define PINCONF_SPI2_MOSI_GPIO              PINCONF(PIN_SPI2_MOSI,      0, 0, 0, 0)
#define PINCONF_SPI2_MOSI                   PINCONF(PIN_SPI2_MOSI,      1, 1, 0, 0)
#define PINCONF_SPI2_MOSI_UART0_CTS         PINCONF(PIN_SPI2_MOSI,      2, 1, 0, 0)
#define PINCONF_SPI2_MISO_GPIO              PINCONF(PIN_SPI2_MISO,      0, 0, 0, 0)
#define PINCONF_SPI2_MISO                   PINCONF(PIN_SPI2_MISO,      1, 0, 0, 0)
#define PINCONF_SPI2_MISO_UART0_RTS         PINCONF(PIN_SPI2_MISO,      2, 0, 0, 0)
#define PINCONF_HIF_IRQ_OUT_GPIO            PINCONF(PIN_HIF_IRQ_OUT,    0, 0, 0, 0)
#define PINCONF_HIF_IRQ_OUT                 PINCONF(PIN_HIF_IRQ_OUT,    1, 0, 0, 0)
#define PINCONF_HIF_IRQ_OUT_OD              PINCONF(PIN_HIF_IRQ_OUT,    2, 0, 0, 0)
#define PINCONF_HIF_IRQ_OUT_GNSS_1PPS_OUT   PINCONF(PIN_HIF_IRQ_OUT,    3, 0, 0, 0)
#ifdef CONFIG_CXD56_FCBGA
#define PINCONF_HIF_GPIO0_GPIO              PINCONF(PIN_HIF_GPIO0,      0, 0, 0, 0)
#define PINCONF_HIF_GPIO0_GPS_EXTLD         PINCONF(PIN_HIF_GPIO0,      3, 0, 0, 0)
#endif /* CONFIG_CXD56_FCBGA */
#define PINCONF_SEN_IRQ_IN_GPIO             PINCONF(PIN_SEN_IRQ_IN,     0, 0, 0, 0)
#define PINCONF_SEN_IRQ_IN                  PINCONF(PIN_SEN_IRQ_IN,     1, 1, 0, 0)
#define PINCONF_SEN_IRQ_IN_SYS_MONOUT0      PINCONF(PIN_SEN_IRQ_IN,     2, 0, 0, 0)
#define PINCONF_SEN_IRQ_IN_DBG_LOGGERI3     PINCONF(PIN_SEN_IRQ_IN,     3, 0, 0, 0)
#define PINCONF_SPI3_CS0_X_GPIO             PINCONF(PIN_SPI3_CS0_X,     0, 0, 0, 0)
#define PINCONF_SPI3_CS0_X                  PINCONF(PIN_SPI3_CS0_X,     1, 0, 0, 0)
#define PINCONF_SPI3_CS0_X_SYS_MONOUT1      PINCONF(PIN_SPI3_CS0_X,     2, 0, 0, 0)
#define PINCONF_SPI3_CS0_X_DBG_LOGGERI2     PINCONF(PIN_SPI3_CS0_X,     3, 0, 0, 0)
#define PINCONF_SPI3_CS1_X_GPIO             PINCONF(PIN_SPI3_CS1_X,     0, 0, 0, 0)
#define PINCONF_SPI3_CS1_X                  PINCONF(PIN_SPI3_CS1_X,     1, 0, 0, 0)
#define PINCONF_SPI3_CS1_X_SYS_MONOUT2      PINCONF(PIN_SPI3_CS1_X,     2, 0, 0, 0)
#define PINCONF_SPI3_CS1_X_DBG_LOGGERI1     PINCONF(PIN_SPI3_CS1_X,     3, 0, 0, 0)
#define PINCONF_SPI3_CS2_X_GPIO             PINCONF(PIN_SPI3_CS2_X,     0, 0, 0, 0)
#define PINCONF_SPI3_CS2_X                  PINCONF(PIN_SPI3_CS2_X,     1, 0, 0, 0)
#define PINCONF_SPI3_CS2_X_SYS_MONOUT3      PINCONF(PIN_SPI3_CS2_X,     2, 0, 0, 0)
#define PINCONF_SPI3_CS2_X_DBG_LOGGERI0     PINCONF(PIN_SPI3_CS2_X,     3, 0, 0, 0)
#define PINCONF_SPI3_SCK_GPIO               PINCONF(PIN_SPI3_SCK,       0, 0, 0, 0)
#define PINCONF_SPI3_SCK                    PINCONF(PIN_SPI3_SCK,       1, 0, 0, 0)
#define PINCONF_SPI3_SCK_SYS_MONOUT4        PINCONF(PIN_SPI3_SCK,       2, 0, 0, 0)
#define PINCONF_SPI3_SCK_DBG_LOGGERQ5       PINCONF(PIN_SPI3_SCK,       3, 0, 0, 0)
#define PINCONF_SPI3_MOSI_GPIO              PINCONF(PIN_SPI3_MOSI,      0, 0, 0, 0)
#define PINCONF_SPI3_MOSI                   PINCONF(PIN_SPI3_MOSI,      1, 0, 0, 0)
#define PINCONF_SPI3_MOSI_SYS_MONOUT5       PINCONF(PIN_SPI3_MOSI,      2, 0, 0, 0)
#define PINCONF_SPI3_MOSI_DBG_LOGGERQ4      PINCONF(PIN_SPI3_MOSI,      3, 0, 0, 0)
#define PINCONF_SPI3_MISO_GPIO              PINCONF(PIN_SPI3_MISO,      0, 0, 0, 0)
#define PINCONF_SPI3_MISO                   PINCONF(PIN_SPI3_MISO,      1, 1, 0, 0)
#define PINCONF_SPI3_MISO_SYS_MONOUT6       PINCONF(PIN_SPI3_MISO,      2, 0, 0, 0)
#define PINCONF_SPI3_MISO_DBG_LOGGERQ3      PINCONF(PIN_SPI3_MISO,      3, 0, 0, 0)
#define PINCONF_I2C0_BCK_GPIO               PINCONF(PIN_I2C0_BCK,       0, 0, 0, 0)
#define PINCONF_I2C0_BCK                    PINCONF(PIN_I2C0_BCK,       1, 1, 0, 0)
#define PINCONF_I2C0_BCK_SYS_MONOUT7        PINCONF(PIN_I2C0_BCK,       2, 0, 0, 0)
#define PINCONF_I2C0_BCK_DBG_LOGGERQ2       PINCONF(PIN_I2C0_BCK,       3, 0, 0, 0)
#define PINCONF_I2C0_BDT_GPIO               PINCONF(PIN_I2C0_BDT,       0, 0, 0, 0)
#define PINCONF_I2C0_BDT                    PINCONF(PIN_I2C0_BDT,       1, 1, 0, 0)
#define PINCONF_I2C0_BDT_SYS_MONOUT8        PINCONF(PIN_I2C0_BDT,       2, 0, 0, 0)
#define PINCONF_I2C0_BDT_DBG_LOGGERQ1       PINCONF(PIN_I2C0_BDT,       3, 0, 0, 0)
#define PINCONF_PWM0_GPIO                   PINCONF(PIN_PWM0,           0, 0, 0, 0)
#define PINCONF_PWM0                        PINCONF(PIN_PWM0,           1, 0, 0, 0)
#define PINCONF_PWM0_SYS_MONOUT9            PINCONF(PIN_PWM0,           2, 0, 0, 0)
#define PINCONF_PWM0_DBG_LOGGERQ0           PINCONF(PIN_PWM0,           3, 0, 0, 0)
#define PINCONF_PWM1_GPIO                   PINCONF(PIN_PWM1,           0, 0, 0, 0)
#define PINCONF_PWM1                        PINCONF(PIN_PWM1,           1, 0, 0, 0)
#define PINCONF_PWM1_SYS_MONOUT_GPIO        PINCONF(PIN_PWM1,           2, 0, 0, 0)
#define PINCONF_PWM1_DBG_LOGGERSEL          PINCONF(PIN_PWM1,           3, 0, 0, 0)
#define PINCONF_PWM2_GPIO                   PINCONF(PIN_PWM2,           0, 0, 0, 0)
#define PINCONF_PWM2                        PINCONF(PIN_PWM2,           1, 0, 0, 0)
#define PINCONF_PWM2_I2C1_BCK               PINCONF(PIN_PWM2,           2, 1, 0, 0)
#define PINCONF_PWM2_DBG_LOGGERI5           PINCONF(PIN_PWM2,           3, 0, 0, 0)
#define PINCONF_PWM3_GPIO                   PINCONF(PIN_PWM3,           0, 0, 0, 0)
#define PINCONF_PWM3                        PINCONF(PIN_PWM3,           1, 0, 0, 0)
#define PINCONF_PWM3_I2C1_BDT               PINCONF(PIN_PWM3,           2, 1, 0, 0)
#define PINCONF_PWM3_DBG_LOGGERI4           PINCONF(PIN_PWM3,           3, 0, 0, 0)
#ifdef CONFIG_CXD56_FCBGA
#define PINCONF_IS_CLK_GPIO                 PINCONF(PIN_IS_CLK,         0, 0, 0, 0)
#define PINCONF_IS_CLK                      PINCONF(PIN_IS_CLK,         1, 1, 0, 0)
#define PINCONF_IS_VSYNC_GPIO               PINCONF(PIN_IS_VSYNC,       0, 0, 0, 0)
#define PINCONF_IS_VSYNC                    PINCONF(PIN_IS_VSYNC,       1, 1, 0, 0)
#define PINCONF_IS_HSYNC_GPIO               PINCONF(PIN_IS_HSYNC,       0, 0, 0, 0)
#define PINCONF_IS_HSYNC                    PINCONF(PIN_IS_HSYNC,       1, 1, 0, 0)
#define PINCONF_IS_DATA0_GPIO               PINCONF(PIN_IS_DATA0,       0, 0, 0, 0)
#define PINCONF_IS_DATA0                    PINCONF(PIN_IS_DATA0,       1, 1, 0, 0)
#define PINCONF_IS_DATA1_GPIO               PINCONF(PIN_IS_DATA1,       0, 0, 0, 0)
#define PINCONF_IS_DATA1                    PINCONF(PIN_IS_DATA1,       1, 1, 0, 0)
#define PINCONF_IS_DATA2_GPIO               PINCONF(PIN_IS_DATA2,       0, 0, 0, 0)
#define PINCONF_IS_DATA2                    PINCONF(PIN_IS_DATA2,       1, 1, 0, 0)
#define PINCONF_IS_DATA3_GPIO               PINCONF(PIN_IS_DATA3,       0, 0, 0, 0)
#define PINCONF_IS_DATA3                    PINCONF(PIN_IS_DATA3,       1, 1, 0, 0)
#define PINCONF_IS_DATA4_GPIO               PINCONF(PIN_IS_DATA4,       0, 0, 0, 0)
#define PINCONF_IS_DATA4                    PINCONF(PIN_IS_DATA4,       1, 1, 0, 0)
#define PINCONF_IS_DATA5_GPIO               PINCONF(PIN_IS_DATA5,       0, 0, 0, 0)
#define PINCONF_IS_DATA5                    PINCONF(PIN_IS_DATA5,       1, 1, 0, 0)
#define PINCONF_IS_DATA6_GPIO               PINCONF(PIN_IS_DATA6,       0, 0, 0, 0)
#define PINCONF_IS_DATA6                    PINCONF(PIN_IS_DATA6,       1, 1, 0, 0)
#define PINCONF_IS_DATA7_GPIO               PINCONF(PIN_IS_DATA7,       0, 0, 0, 0)
#define PINCONF_IS_DATA7                    PINCONF(PIN_IS_DATA7,       1, 1, 0, 0)
#endif /* CONFIG_CXD56_FCBGA */
#define PINCONF_UART2_TXD_GPIO              PINCONF(PIN_UART2_TXD,      0, 0, 0, 0)
#define PINCONF_UART2_TXD                   PINCONF(PIN_UART2_TXD,      1, 0, 0, 0)
#define PINCONF_UART2_TXD_APP_MONOUT0       PINCONF(PIN_UART2_TXD,      2, 0, 0, 0)
#define PINCONF_UART2_RXD_GPIO              PINCONF(PIN_UART2_RXD,      0, 0, 0, 0)
#define PINCONF_UART2_RXD                   PINCONF(PIN_UART2_RXD,      1, 1, 0, 0)
#define PINCONF_UART2_RXD_APP_MONOUT1       PINCONF(PIN_UART2_RXD,      2, 0, 0, 0)
#define PINCONF_UART2_CTS_GPIO              PINCONF(PIN_UART2_CTS,      0, 0, 0, 0)
#define PINCONF_UART2_CTS                   PINCONF(PIN_UART2_CTS,      1, 1, 0, 0)
#define PINCONF_UART2_CTS_APP_MONOUT2       PINCONF(PIN_UART2_CTS,      2, 0, 0, 0)
#define PINCONF_UART2_RTS_GPIO              PINCONF(PIN_UART2_RTS,      0, 0, 0, 0)
#define PINCONF_UART2_RTS                   PINCONF(PIN_UART2_RTS,      1, 0, 0, 0)
#define PINCONF_UART2_RTS_APP_MONOUT3       PINCONF(PIN_UART2_RTS,      2, 0, 0, 0)
#define PINCONF_SPI4_CS_X_GPIO              PINCONF(PIN_SPI4_CS_X,      0, 0, 0, 0)
#define PINCONF_SPI4_CS_X                   PINCONF(PIN_SPI4_CS_X,      1, 0, 0, 0)
#define PINCONF_SPI4_CS_X_APP_MONOUT4       PINCONF(PIN_SPI4_CS_X,      2, 0, 0, 0)
#define PINCONF_SPI4_SCK_GPIO               PINCONF(PIN_SPI4_SCK,       0, 0, 0, 0)
#define PINCONF_SPI4_SCK                    PINCONF(PIN_SPI4_SCK,       1, 0, 0, 0)
#define PINCONF_SPI4_SCK_APP_MONOUT5        PINCONF(PIN_SPI4_SCK,       2, 0, 0, 0)
#define PINCONF_SPI4_MOSI_GPIO              PINCONF(PIN_SPI4_MOSI,      0, 0, 0, 0)
#define PINCONF_SPI4_MOSI                   PINCONF(PIN_SPI4_MOSI,      1, 0, 0, 0)
#define PINCONF_SPI4_MOSI_APP_MONOUT6       PINCONF(PIN_SPI4_MOSI,      2, 0, 0, 0)
#define PINCONF_SPI4_MISO_GPIO              PINCONF(PIN_SPI4_MISO,      0, 0, 0, 0)
#define PINCONF_SPI4_MISO                   PINCONF(PIN_SPI4_MISO,      1, 1, 0, 0)
#define PINCONF_SPI4_MISO_APP_MONOUT7       PINCONF(PIN_SPI4_MISO,      2, 0, 0, 0)
#define PINCONF_EMMC_CLK_GPIO               PINCONF(PIN_EMMC_CLK,       0, 0, 0, 0)
#define PINCONF_EMMC_CLK                    PINCONF(PIN_EMMC_CLK,       1, 0, 0, 0)
#define PINCONF_EMMC_CLK_SPI5_SCK           PINCONF(PIN_EMMC_CLK,       2, 0, 0, 0)
#define PINCONF_EMMC_CMD_GPIO               PINCONF(PIN_EMMC_CMD,       0, 0, 0, 0)
#define PINCONF_EMMC_CMD                    PINCONF(PIN_EMMC_CMD,       1, 1, 0, 0)
#define PINCONF_EMMC_CMD_SPI5_CS_X          PINCONF(PIN_EMMC_CMD,       2, 0, 0, 0)
#define PINCONF_EMMC_DATA0_GPIO             PINCONF(PIN_EMMC_DATA0,     0, 0, 0, 0)
#define PINCONF_EMMC_DATA0                  PINCONF(PIN_EMMC_DATA0,     1, 1, 0, 0)
#define PINCONF_EMMC_DATA0_SPI5_MOSI        PINCONF(PIN_EMMC_DATA0,     2, 0, 0, 0)
#define PINCONF_EMMC_DATA1_GPIO             PINCONF(PIN_EMMC_DATA1,     0, 0, 0, 0)
#define PINCONF_EMMC_DATA1                  PINCONF(PIN_EMMC_DATA1,     1, 1, 0, 0)
#define PINCONF_EMMC_DATA1_SPI5_MISO        PINCONF(PIN_EMMC_DATA1,     2, 1, 0, 0)
#define PINCONF_EMMC_DATA2_GPIO             PINCONF(PIN_EMMC_DATA2,     0, 0, 0, 0)
#define PINCONF_EMMC_DATA2                  PINCONF(PIN_EMMC_DATA2,     1, 1, 0, 0)
#define PINCONF_EMMC_DATA2_APP_MONOUT8      PINCONF(PIN_EMMC_DATA2,     2, 0, 0, 0)
#define PINCONF_EMMC_DATA3_GPIO             PINCONF(PIN_EMMC_DATA3,     0, 0, 0, 0)
#define PINCONF_EMMC_DATA3                  PINCONF(PIN_EMMC_DATA3,     1, 1, 0, 0)
#define PINCONF_EMMC_DATA3_APP_MONOUT9      PINCONF(PIN_EMMC_DATA3,     2, 0, 0, 0)
#ifdef CONFIG_CXD56_FCBGA
#define PINCONF_SDIO_CLK_GPIO               PINCONF(PIN_SDIO_CLK,       0, 0, 0, 0)
#define PINCONF_SDIO_CLK                    PINCONF(PIN_SDIO_CLK,       1, 1, 1, 0)
#define PINCONF_SDIO_CLK_SDCARD             PINCONF(PIN_SDIO_CLK,       1, 0, 1, 0)
#define PINCONF_SDIO_CLK_SPI5_SCK           PINCONF(PIN_SDIO_CLK,       2, 0, 0, 0)
#define PINCONF_SDIO_CMD_GPIO               PINCONF(PIN_SDIO_CMD,       0, 0, 0, 0)
#define PINCONF_SDIO_CMD                    PINCONF(PIN_SDIO_CMD,       1, 1, 1, 0)
#define PINCONF_SDIO_CMD_SPI5_CS_X          PINCONF(PIN_SDIO_CMD,       2, 0, 0, 0)
#define PINCONF_SDIO_DATA0_GPIO             PINCONF(PIN_SDIO_DATA0,     0, 0, 0, 0)
#define PINCONF_SDIO_DATA0                  PINCONF(PIN_SDIO_DATA0,     1, 1, 1, 0)
#define PINCONF_SDIO_DATA0_SPI5_MOSI        PINCONF(PIN_SDIO_DATA0,     2, 0, 0, 0)
#define PINCONF_SDIO_DATA1_GPIO             PINCONF(PIN_SDIO_DATA1,     0, 0, 0, 0)
#define PINCONF_SDIO_DATA1                  PINCONF(PIN_SDIO_DATA1,     1, 1, 1, 0)
#define PINCONF_SDIO_DATA1_SPI5_MISO        PINCONF(PIN_SDIO_DATA1,     2, 1, 0, 0)
#define PINCONF_SDIO_DATA2_GPIO             PINCONF(PIN_SDIO_DATA2,     0, 0, 0, 0)
#define PINCONF_SDIO_DATA2                  PINCONF(PIN_SDIO_DATA2,     1, 1, 1, 0)
#define PINCONF_SDIO_DATA2_SPI5_GPIO        PINCONF(PIN_SDIO_DATA2,     2, 0, 0, 0)
#define PINCONF_SDIO_DATA3_GPIO             PINCONF(PIN_SDIO_DATA3,     0, 0, 0, 0)
#define PINCONF_SDIO_DATA3                  PINCONF(PIN_SDIO_DATA3,     1, 1, 1, 0)
#define PINCONF_SDIO_DATA3_SPI5_GPIO        PINCONF(PIN_SDIO_DATA3,     2, 0, 0, 0)
#define PINCONF_SDIO_CD_GPIO                PINCONF(PIN_SDIO_CD,        0, 0, 0, 0)
#define PINCONF_SDIO_CD                     PINCONF(PIN_SDIO_CD,        1, 1, 0, 0)
#define PINCONF_SDIO_WP_GPIO                PINCONF(PIN_SDIO_WP,        0, 0, 0, 0)
#define PINCONF_SDIO_WP                     PINCONF(PIN_SDIO_WP,        1, 1, 0, 0)
#define PINCONF_SDIO_CMDDIR_GPIO            PINCONF(PIN_SDIO_CMDDIR,    0, 0, 0, 0)
#define PINCONF_SDIO_CMDDIR                 PINCONF(PIN_SDIO_CMDDIR,    1, 0, 1, 0)
#define PINCONF_SDIO_DIR0_GPIO              PINCONF(PIN_SDIO_DIR0,      0, 0, 0, 0)
#define PINCONF_SDIO_DIR0                   PINCONF(PIN_SDIO_DIR0,      1, 0, 1, 0)
#define PINCONF_SDIO_DIR1_3_GPIO            PINCONF(PIN_SDIO_DIR1_3,    0, 0, 0, 0)
#define PINCONF_SDIO_DIR1_3                 PINCONF(PIN_SDIO_DIR1_3,    1, 0, 1, 0)
#define PINCONF_SDIO_CLKI_GPIO              PINCONF(PIN_SDIO_CLKI,      0, 0, 0, 0)
#define PINCONF_SDIO_CLKI                   PINCONF(PIN_SDIO_CLKI,      1, 1, 0, 0) /* only for SD Card */
#endif /* CONFIG_CXD56_FCBGA */
#define PINCONF_I2S0_BCK_GPIO               PINCONF(PIN_I2S0_BCK,       0, 0, 0, 0)
#define PINCONF_I2S0_BCK_M_HIGH             PINCONF(PIN_I2S0_BCK,       1, 0, 1, 0)
#define PINCONF_I2S0_BCK_M_NORM             PINCONF(PIN_I2S0_BCK,       1, 0, 0, 0)
#define PINCONF_I2S0_BCK_S                  PINCONF(PIN_I2S0_BCK,       1, 1, 0, 0)
#define PINCONF_I2S0_BCK_APP_MONOUT0        PINCONF(PIN_I2S0_BCK,       2, 0, 0, 0)
#define PINCONF_I2S0_LRCK_GPIO              PINCONF(PIN_I2S0_LRCK,      0, 0, 0, 0)
#define PINCONF_I2S0_LRCK_M_HIGH            PINCONF(PIN_I2S0_LRCK,      1, 0, 1, 0)
#define PINCONF_I2S0_LRCK_M_NORM            PINCONF(PIN_I2S0_LRCK,      1, 0, 0, 0)
#define PINCONF_I2S0_LRCK_S                 PINCONF(PIN_I2S0_LRCK,      1, 1, 0, 0)
#define PINCONF_I2S0_LRCK_APP_MONOUT1       PINCONF(PIN_I2S0_LRCK,      2, 0, 0, 0)
#define PINCONF_I2S0_DATA_IN_GPIO           PINCONF(PIN_I2S0_DATA_IN,   0, 0, 0, 0)
#define PINCONF_I2S0_DATA_IN                PINCONF(PIN_I2S0_DATA_IN,   1, 1, 0, 0)
#define PINCONF_I2S0_DATA_IN_APP_MONOUT2    PINCONF(PIN_I2S0_DATA_IN,   2, 0, 0, 0)
#define PINCONF_I2S0_DATA_OUT_GPIO          PINCONF(PIN_I2S0_DATA_OUT,  0, 0, 0, 0)
#define PINCONF_I2S0_DATA_OUT_HIGH          PINCONF(PIN_I2S0_DATA_OUT,  1, 0, 1, 0)
#define PINCONF_I2S0_DATA_OUT_NORM          PINCONF(PIN_I2S0_DATA_OUT,  1, 0, 0, 0)
#define PINCONF_I2S0_DATA_OUT_APP_MONOUT3   PINCONF(PIN_I2S0_DATA_OUT,  2, 0, 0, 0)
#ifdef CONFIG_CXD56_FCBGA
#define PINCONF_I2S1_BCK_GPIO               PINCONF(PIN_I2S1_BCK,       0, 0, 0, 0)
#define PINCONF_I2S1_BCK_M_HIGH             PINCONF(PIN_I2S1_BCK,       1, 0, 1, 0)
#define PINCONF_I2S1_BCK_M_NORM             PINCONF(PIN_I2S1_BCK,       1, 0, 0, 0)
#define PINCONF_I2S1_BCK_S                  PINCONF(PIN_I2S1_BCK,       1, 1, 0, 0)
#define PINCONF_I2S1_BCK_APP_MONOUT4        PINCONF(PIN_I2S1_BCK,       2, 0, 0, 0)
#define PINCONF_I2S1_LRCK_GPIO              PINCONF(PIN_I2S1_LRCK,      0, 0, 0, 0)
#define PINCONF_I2S1_LRCK_M_HIGH            PINCONF(PIN_I2S1_LRCK,      1, 0, 1, 0)
#define PINCONF_I2S1_LRCK_M_NORM            PINCONF(PIN_I2S1_LRCK,      1, 0, 0, 0)
#define PINCONF_I2S1_LRCK_S                 PINCONF(PIN_I2S1_LRCK,      1, 1, 0, 0)
#define PINCONF_I2S1_LRCK_APP_MONOUT5       PINCONF(PIN_I2S1_LRCK,      2, 0, 0, 0)
#define PINCONF_I2S1_DATA_IN_GPIO           PINCONF(PIN_I2S1_DATA_IN,   0, 0, 0, 0)
#define PINCONF_I2S1_DATA_IN                PINCONF(PIN_I2S1_DATA_IN,   1, 1, 0, 0)
#define PINCONF_I2S1_DATA_IN_APP_MONOUT6    PINCONF(PIN_I2S1_DATA_IN,   2, 0, 0, 0)
#define PINCONF_I2S1_DATA_OUT_GPIO          PINCONF(PIN_I2S1_DATA_OUT,  0, 0, 0, 0)
#define PINCONF_I2S1_DATA_OUT_HIGH          PINCONF(PIN_I2S1_DATA_OUT,  1, 0, 1, 0)
#define PINCONF_I2S1_DATA_OUT_NORM          PINCONF(PIN_I2S1_DATA_OUT,  1, 0, 0, 0)
#define PINCONF_I2S1_DATA_OUT_APP_MONOUT7   PINCONF(PIN_I2S1_DATA_OUT,  2, 0, 0, 0)
#endif /* CONFIG_CXD56_FCBGA */
#define PINCONF_MCLK_GPIO                   PINCONF(PIN_MCLK,           0, 0, 0, 0)
#define PINCONF_MCLK                        PINCONF(PIN_MCLK,           1, 1, 0, 0)
#define PINCONF_MCLK_APP_MONOUT8            PINCONF(PIN_MCLK,           2, 0, 0, 0)
#define PINCONF_PDM_CLK_GPIO                PINCONF(PIN_PDM_CLK,        0, 0, 0, 0)
#define PINCONF_PDM_CLK_HIGH                PINCONF(PIN_PDM_CLK,        1, 0, 1, 0)
#define PINCONF_PDM_CLK_NORM                PINCONF(PIN_PDM_CLK,        1, 0, 0, 0)
#define PINCONF_PDM_CLK_APP_MONOUT9         PINCONF(PIN_PDM_CLK,        2, 0, 0, 0)
#define PINCONF_PDM_IN_GPIO                 PINCONF(PIN_PDM_IN,         0, 0, 0, 0)
#define PINCONF_PDM_IN                      PINCONF(PIN_PDM_IN,         1, 1, 0, 0)
#define PINCONF_PDM_OUT_GPIO                PINCONF(PIN_PDM_OUT,        0, 0, 0, 0)
#define PINCONF_PDM_OUT_HIGH                PINCONF(PIN_PDM_OUT,        1, 0, 1, 0)
#define PINCONF_PDM_OUT_NORM                PINCONF(PIN_PDM_OUT,        1, 0, 0, 0)
#define PINCONF_USB_VBUSINT_GPIO            PINCONF(PIN_USB_VBUSINT,    0, 0, 0, 0)
#define PINCONF_USB_VBUSINT                 PINCONF(PIN_USB_VBUSINT,    1, 1, 0, 0)
#define PINCONF_USB_VBUSINT_DBG_LOGGERCLK   PINCONF(PIN_USB_VBUSINT,    3, 0, 0, 0)

/* Reference set of multiple pinconfigs
 *
 */

#define PINCONFS_I2C4_GPIO                  { PINCONF_I2C4_BCK_GPIO, PINCONF_I2C4_BDT_GPIO }
#define PINCONFS_I2C4                       { PINCONF_I2C4_BCK,      PINCONF_I2C4_BDT      }
#define PINCONFS_PMIC_INT_GPIO              { PINCONF_PMIC_INT_GPIO }
#define PINCONFS_PMIC_INT                   { PINCONF_PMIC_INT }
#define PINCONFS_PMIC_INT_OD                { PINCONF_PMIC_INT_OD }
#ifdef CONFIG_CXD56_FCBGA
#define PINCONFS_RTC_IRQ_OUT_GPIO           { PINCONF_RTC_IRQ_OUT_GPIO }
#define PINCONFS_RTC_IRQ_OUT                { PINCONF_RTC_IRQ_OUT }
#define PINCONFS_RTC_IRQ_OUT_OD             { PINCONF_RTC_IRQ_OUT_OD   }
#endif /* CONFIG_CXD56_FCBGA */
#define PINCONFS_AP_CLK_GPIO                { PINCONF_AP_CLK_GPIO }
#define PINCONFS_AP_CLK                     { PINCONF_AP_CLK }
#define PINCONFS_AP_CLK_PMU_WDT             { PINCONF_AP_CLK_PMU_WDT }
#define PINCONFS_AP_CLK_PMU_WDT_OD          { PINCONF_AP_CLK_PMU_WDT_OD }
#ifdef CONFIG_CXD56_FCBGA
#define PINCONFS_GNSS_1PPS_OUT_GPIO         { PINCONF_GNSS_1PPS_OUT_GPIO }
#define PINCONFS_GNSS_1PPS_OUT              { PINCONF_GNSS_1PPS_OUT }
#define PINCONFS_GNSS_1PPS_OUT_CPU_WDT      { PINCONF_GNSS_1PPS_OUT_CPU_WDT }
#define PINCONFS_GNSS_1PPS_OUT_CPU_WDT_OD   { PINCONF_GNSS_1PPS_OUT_CPU_WDT_OD }
#endif /* CONFIG_CXD56_FCBGA */
#define PINCONFS_SPI0A_GPIO                 { PINCONF_SPI0_CS_X_GPIO, PINCONF_SPI0_SCK_GPIO }
#ifdef CONFIG_CXD56_FCBGA
#define PINCONFS_SPI0B_GPIO                 { PINCONF_SPI0_MOSI_GPIO, PINCONF_SPI0_MISO_GPIO }
#endif /* CONFIG_CXD56_FCBGA */
#define PINCONFS_SPI0A_UART1                { PINCONF_SPI0_CS_X_UART1_TXD, PINCONF_SPI0_SCK_UART1_RXD }
#ifdef CONFIG_CXD56_FCBGA
#define PINCONFS_SPI0B_I2C2                 { PINCONF_SPI0_MOSI_I2C2_BCK,  PINCONF_SPI0_MISO_I2C2_BDT }
#define PINCONFS_SPI0_GPIO                  { PINCONF_SPI0_CS_X_GPIO, PINCONF_SPI0_SCK_GPIO, \
                                               PINCONF_SPI0_MOSI_GPIO, PINCONF_SPI0_MISO_GPIO }
#define PINCONFS_SPI0                       { PINCONF_SPI0_CS_X, PINCONF_SPI0_SCK, \
                                               PINCONF_SPI0_MOSI, PINCONF_SPI0_MISO }
#endif /* CONFIG_CXD56_FCBGA */
#define PINCONFS_SPI1A_GPIO                 { PINCONF_SPI1_CS_X_GPIO, PINCONF_SPI1_SCK_GPIO, \
                                               PINCONF_SPI1_IO0_GPIO, PINCONF_SPI1_IO1_GPIO }
#define PINCONFS_SPI1B_GPIO                 { PINCONF_SPI1_IO2_GPIO, PINCONF_SPI1_IO3_GPIO }
#define PINCONFS_SPI1A_SPI0                 { PINCONF_SPI1_CS_X_SPI0_CS_X, PINCONF_SPI1_SCK_SPI0_SCK, \
                                               PINCONF_SPI1_IO0_SPI0_MOSI, PINCONF_SPI1_IO1_SPI0_MISO }
#define PINCONFS_SPI1_GPIO                  { PINCONF_SPI1_CS_X_GPIO, PINCONF_SPI1_SCK_GPIO, PINCONF_SPI1_IO0_GPIO, \
                                               PINCONF_SPI1_IO1_GPIO, PINCONF_SPI1_IO2_GPIO, PINCONF_SPI1_IO3_GPIO }
#define PINCONFS_SPI1                       { PINCONF_SPI1_CS_X, PINCONF_SPI1_SCK, PINCONF_SPI1_IO0, \
                                               PINCONF_SPI1_IO1, PINCONF_SPI1_IO2, PINCONF_SPI1_IO3 }
#define PINCONFS_SPI2A_GPIO                 { PINCONF_SPI2_CS_X_GPIO, PINCONF_SPI2_SCK_GPIO }
#define PINCONFS_SPI2B_GPIO                 { PINCONF_SPI2_MOSI_GPIO, PINCONF_SPI2_MISO_GPIO }
#define PINCONFS_SPI2A_UART0                { PINCONF_SPI2_CS_X_UART0_TXD, PINCONF_SPI2_SCK_UART0_RXD }
#define PINCONFS_SPI2B_UART0                { PINCONF_SPI2_MOSI_UART0_CTS, PINCONF_SPI2_MISO_UART0_RTS }
#define PINCONFS_SPI2A_I2C3                 { PINCONF_SPI2_CS_X_I2C3_BCK, PINCONF_SPI2_SCK_I2C3_BDT }
#define PINCONFS_SPI2_GPIO                  { PINCONF_SPI2_CS_X_GPIO, PINCONF_SPI2_SCK_GPIO, \
                                               PINCONF_SPI2_MOSI_GPIO, PINCONF_SPI2_MISO_GPIO }
#define PINCONFS_SPI2                       { PINCONF_SPI2_CS_X, PINCONF_SPI2_SCK, \
                                               PINCONF_SPI2_MOSI, PINCONF_SPI2_MISO }
#define PINCONFS_SPI2_UART0                 { PINCONF_SPI2_CS_X_UART0_TXD, PINCONF_SPI2_SCK_UART0_RXD, \
                                               PINCONF_SPI2_MOSI_UART0_CTS, PINCONF_SPI2_MISO_UART0_RTS }
#define PINCONFS_HIF_IRQ_OUT_GPIO           { PINCONF_HIF_IRQ_OUT_GPIO }
#define PINCONFS_HIF_IRQ_OUT                { PINCONF_HIF_IRQ_OUT }
#define PINCONFS_HIF_IRQ_OUT_OD             { PINCONF_HIF_IRQ_OUT_OD }
#define PINCONFS_HIF_IRQ_OUT_GNSS_1PPS_OUT  { PINCONF_HIF_IRQ_OUT_GNSS_1PPS_OUT }
#ifdef CONFIG_CXD56_FCBGA
#define PINCONFS_HIF_GPIO0_GPIO             { PINCONF_HIF_GPIO0_GPIO }
#define PINCONFS_HIF_GPIO0_GPS_EXTLD        { PINCONF_HIF_GPIO0_GPS_EXTLD }
#endif /* CONFIG_CXD56_FCBGA */
#define PINCONFS_SEN_IRQ_IN_GPIO            { PINCONF_SEN_IRQ_IN_GPIO }
#define PINCONFS_SEN_IRQ_IN                 { PINCONF_SEN_IRQ_IN }
#define PINCONFS_SPI3_CS0_X_GPIO            { PINCONF_SPI3_CS0_X_GPIO }
#define PINCONFS_SPI3_CS0_X                 { PINCONF_SPI3_CS0_X }
#define PINCONFS_SPI3_CS1_X_GPIO            { PINCONF_SPI3_CS1_X_GPIO }
#define PINCONFS_SPI3_CS1_X                 { PINCONF_SPI3_CS1_X }
#define PINCONFS_SPI3_CS2_X_GPIO            { PINCONF_SPI3_CS2_X_GPIO }
#define PINCONFS_SPI3_CS2_X                 { PINCONF_SPI3_CS2_X }
#define PINCONFS_SPI3_GPIO                  { PINCONF_SPI3_SCK_GPIO, \
                                               PINCONF_SPI3_MOSI_GPIO, PINCONF_SPI3_MISO_GPIO }
#define PINCONFS_SPI3                       { PINCONF_SPI3_SCK, \
                                               PINCONF_SPI3_MOSI, PINCONF_SPI3_MISO }
#define PINCONFS_I2C0_GPIO                  { PINCONF_I2C0_BCK_GPIO, PINCONF_I2C0_BDT_GPIO }
#define PINCONFS_I2C0                       { PINCONF_I2C0_BCK, PINCONF_I2C0_BDT }
#define PINCONFS_PWMA_GPIO                  { PINCONF_PWM0_GPIO, PINCONF_PWM1_GPIO }
#define PINCONFS_PWMA                       { PINCONF_PWM0, PINCONF_PWM1 }
#define PINCONFS_PWMB_GPIO                  { PINCONF_PWM2_GPIO, PINCONF_PWM3_GPIO }
#define PINCONFS_PWMB                       { PINCONF_PWM2, PINCONF_PWM3 }
#define PINCONFS_PWMB_I2C1                  { PINCONF_PWM2_I2C1_BCK, PINCONF_PWM3_I2C1_BDT }
#ifdef CONFIG_CXD56_FCBGA
#define PINCONFS_IS_GPIO                    { PINCONF_IS_CLK_GPIO, PINCONF_IS_VSYNC_GPIO, PINCONF_IS_HSYNC_GPIO, \
                                               PINCONF_IS_DATA0_GPIO, PINCONF_IS_DATA1_GPIO, PINCONF_IS_DATA2_GPIO, \
                                               PINCONF_IS_DATA3_GPIO, PINCONF_IS_DATA4_GPIO, PINCONF_IS_DATA5_GPIO, \
                                               PINCONF_IS_DATA6_GPIO, PINCONF_IS_DATA7_GPIO }
#define PINCONFS_IS                         { PINCONF_IS_CLK, PINCONF_IS_VSYNC, PINCONF_IS_HSYNC, \
                                               PINCONF_IS_DATA0, PINCONF_IS_DATA1, PINCONF_IS_DATA2, \
                                               PINCONF_IS_DATA3, PINCONF_IS_DATA4, PINCONF_IS_DATA5, \
                                               PINCONF_IS_DATA6, PINCONF_IS_DATA7 }
#endif /* CONFIG_CXD56_FCBGA */
#define PINCONFS_UART2_GPIO                 { PINCONF_UART2_TXD_GPIO, PINCONF_UART2_RXD_GPIO, \
                                               PINCONF_UART2_CTS_GPIO, PINCONF_UART2_RTS_GPIO }
#define PINCONFS_UART2                      { PINCONF_UART2_TXD, PINCONF_UART2_RXD, \
                                               PINCONF_UART2_CTS, PINCONF_UART2_RTS }
#define PINCONFS_SPI4_GPIO                  { PINCONF_SPI4_CS_X_GPIO, PINCONF_SPI4_SCK_GPIO, \
                                               PINCONF_SPI4_MOSI_GPIO, PINCONF_SPI4_MISO_GPIO }
#define PINCONFS_SPI4                       { PINCONF_SPI4_CS_X, PINCONF_SPI4_SCK, \
                                               PINCONF_SPI4_MOSI, PINCONF_SPI4_MISO }
#define PINCONFS_EMMCA_GPIO                 { PINCONF_EMMC_CLK_GPIO, PINCONF_EMMC_CMD_GPIO, \
                                               PINCONF_EMMC_DATA0_GPIO, PINCONF_EMMC_DATA1_GPIO }
#define PINCONFS_EMMCB_GPIO                 { PINCONF_EMMC_DATA2_GPIO, PINCONF_EMMC_DATA3_GPIO }
#define PINCONFS_EMMCA_SPI5                 { PINCONF_EMMC_CLK_SPI5_SCK, PINCONF_EMMC_CMD_SPI5_CS_X, \
                                               PINCONF_EMMC_DATA0_SPI5_MOSI, PINCONF_EMMC_DATA1_SPI5_MISO }
#define PINCONFS_EMMC_GPIO                  { PINCONF_EMMC_CLK_GPIO, PINCONF_EMMC_CMD_GPIO, \
                                               PINCONF_EMMC_DATA0_GPIO, PINCONF_EMMC_DATA1_GPIO, \
                                               PINCONF_EMMC_DATA2_GPIO, PINCONF_EMMC_DATA3_GPIO }
#define PINCONFS_EMMC                       { PINCONF_EMMC_CLK, PINCONF_EMMC_CMD, \
                                               PINCONF_EMMC_DATA0, PINCONF_EMMC_DATA1, \
                                               PINCONF_EMMC_DATA2, PINCONF_EMMC_DATA3 }
#ifdef CONFIG_CXD56_FCBGA
#define PINCONFS_SDIOA_GPIO                 { PINCONF_SDIO_CLK_GPIO, PINCONF_SDIO_CMD_GPIO, \
                                               PINCONF_SDIO_DATA0_GPIO, PINCONF_SDIO_DATA1_GPIO, \
                                               PINCONF_SDIO_DATA2_GPIO, PINCONF_SDIO_DATA3_GPIO }
#define PINCONFS_SDIOA_SDIO                 { PINCONF_SDIO_CLK, PINCONF_SDIO_CMD, \
                                               PINCONF_SDIO_DATA0, PINCONF_SDIO_DATA1, \
                                               PINCONF_SDIO_DATA2, PINCONF_SDIO_DATA3 }
#define PINCONFS_SDIOA_SDCARD               { PINCONF_SDIO_CLK_SDCARD, PINCONF_SDIO_CMD, \
                                               PINCONF_SDIO_DATA0, PINCONF_SDIO_DATA1, \
                                               PINCONF_SDIO_DATA2, PINCONF_SDIO_DATA3 }
#define PINCONFS_SDIOA_SPI5                 { PINCONF_SDIO_CLK_SPI5_SCK, PINCONF_SDIO_CMD_SPI5_CS_X, \
                                               PINCONF_SDIO_DATA0_SPI5_MOSI, PINCONF_SDIO_DATA1_SPI5_MISO, \
                                               PINCONF_SDIO_DATA2_SPI5_GPIO, PINCONF_SDIO_DATA3_SPI5_GPIO }
#define PINCONFS_SDIOB_GPIO                 { PINCONF_SDIO_CD_GPIO, PINCONF_SDIO_WP_GPIO }
#define PINCONFS_SDIOB_SDCARD               { PINCONF_SDIO_CD, PINCONF_SDIO_WP }
#define PINCONFS_SDIOC_GPIO                 { PINCONF_SDIO_CMDDIR_GPIO, PINCONF_SDIO_DIR0_GPIO, PINCONF_SDIO_DIR1_3_GPIO }
#define PINCONFS_SDIOC_SDIO                 { PINCONF_SDIO_CMDDIR, PINCONF_SDIO_DIR0, PINCONF_SDIO_DIR1_3 }
#define PINCONFS_SDIOD_GPIO                 { PINCONF_SDIO_CLKI_GPIO }
#define PINCONFS_SDIOD_SDIO                 { PINCONF_SDIO_CLKI }
#endif /* CONFIG_CXD56_FCBGA */
#define PINCONFS_I2S0_GPIO                  { PINCONF_I2S0_BCK_GPIO, PINCONF_I2S0_LRCK_GPIO, \
                                               PINCONF_I2S0_DATA_IN_GPIO, PINCONF_I2S0_DATA_OUT_GPIO }
#define PINCONFS_I2S0_M_HIGH                { PINCONF_I2S0_BCK_M_HIGH, PINCONF_I2S0_LRCK_M_HIGH, \
                                               PINCONF_I2S0_DATA_IN, PINCONF_I2S0_DATA_OUT_HIGH }
#define PINCONFS_I2S0_M_NORM                { PINCONF_I2S0_BCK_M_NORM, PINCONF_I2S0_LRCK_M_NORM, \
                                               PINCONF_I2S0_DATA_IN, PINCONF_I2S0_DATA_OUT_NORM }
#define PINCONFS_I2S0_S_HIGH                { PINCONF_I2S0_BCK_S, PINCONF_I2S0_LRCK_S, \
                                               PINCONF_I2S0_DATA_IN, PINCONF_I2S0_DATA_OUT_HIGH }
#define PINCONFS_I2S0_S_NORM                { PINCONF_I2S0_BCK_S, PINCONF_I2S0_LRCK_S, \
                                               PINCONF_I2S0_DATA_IN, PINCONF_I2S0_DATA_OUT_NORM }
#ifdef CONFIG_CXD56_FCBGA
#define PINCONFS_I2S1_GPIO                  { PINCONF_I2S1_BCK_GPIO, PINCONF_I2S1_LRCK_GPIO, \
                                               PINCONF_I2S1_DATA_IN_GPIO, PINCONF_I2S1_DATA_OUT_GPIO }
#define PINCONFS_I2S1_M_HIGH                { PINCONF_I2S1_BCK_M_HIGH, PINCONF_I2S1_LRCK_M_HIGH, \
                                               PINCONF_I2S1_DATA_IN, PINCONF_I2S1_DATA_OUT_HIGH }
#define PINCONFS_I2S1_M_NORM                { PINCONF_I2S1_BCK_M_NORM, PINCONF_I2S1_LRCK_M_NORM, \
                                               PINCONF_I2S1_DATA_IN, PINCONF_I2S1_DATA_OUT_NORM }
#define PINCONFS_I2S1_S_HIGH                { PINCONF_I2S1_BCK_S, PINCONF_I2S1_LRCK_S, \
                                               PINCONF_I2S1_DATA_IN, PINCONF_I2S1_DATA_OUT_HIGH }
#define PINCONFS_I2S1_S_NORM                { PINCONF_I2S1_BCK_S, PINCONF_I2S1_LRCK_S, \
                                               PINCONF_I2S1_DATA_IN, PINCONF_I2S1_DATA_OUT_NORM }
#endif /* CONFIG_CXD56_FCBGA */
#define PINCONFS_MCLK_GPIO                  { PINCONF_MCLK_GPIO }
#define PINCONFS_MCLK                       { PINCONF_MCLK }
#define PINCONFS_PDM_GPIO                   { PINCONF_PDM_CLK_GPIO, PINCONF_PDM_IN_GPIO, PINCONF_PDM_OUT_GPIO }
#define PINCONFS_PDM_HIGH                   { PINCONF_PDM_CLK_HIGH, PINCONF_PDM_IN, PINCONF_PDM_OUT_HIGH }
#define PINCONFS_PDM_NORM                   { PINCONF_PDM_CLK_NORM, PINCONF_PDM_IN, PINCONF_PDM_OUT_NORM }
#define PINCONFS_USB_VBUSINT_GPIO           { PINCONF_USB_VBUSINT_GPIO }
#define PINCONFS_USB_VBUSINT                { PINCONF_USB_VBUSINT }

#ifdef CONFIG_CXD56_CUSTOM_PINCONFIG
/* Change the pin configuration depending on each board */

#  include <arch/board/board_pinconfig.h>

#endif /* CONFIG_CXD56_CUSTOM_PINCONFIG */

#endif /* __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD5602_PINCONFIG_H */
