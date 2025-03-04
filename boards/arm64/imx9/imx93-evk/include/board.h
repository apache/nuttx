/****************************************************************************
 * boards/arm64/imx9/imx93-evk/include/board.h
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

#ifndef __BOARDS_ARM64_IMX9_IMX93_EVK_INCLUDE_BOARD_H
#define __BOARDS_ARM64_IMX9_IMX93_EVK_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/net/mii.h>
#include <nuttx/net/gmii.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Default PAD configurations */

#define IOMUX_LPI2C_DEFAULT  (IOMUXC_PAD_OD_ENABLE | IOMUXC_PAD_FSEL_SFAST | IOMUXC_PAD_DSE_X6)
#define IOMUX_LPSPI_DEFAULT  (IOMUXC_PAD_PU_ON     | IOMUXC_PAD_FSEL_FAST  | IOMUXC_PAD_DSE_X6)
#define IOMUX_GPIO_DEFAULT   (IOMUXC_PAD_FSEL_SLOW | IOMUXC_PAD_DSE_X6)

/* UART pin muxings */

#define MUX_LPUART1_RX       IOMUX_CFG(IOMUXC_PAD_UART1_RXD_LPUART1_RX, 0, IOMUXC_MUX_SION_ON)
#define MUX_LPUART1_TX       IOMUX_CFG(IOMUXC_PAD_UART1_TXD_LPUART1_TX, IOMUXC_PAD_FSEL_SLOW | IOMUXC_PAD_DSE_X4, 0)

/* FLEXIO to PWM pin muxings */

/* EVK signals
 * GPIO_IO04 -> FLEXIO1_04
 * GPIO_IO05 -> FLEXIO1_05
 * GPIO_IO06 -> FLEXIO1_06
 * GPIO_IO07 -> FLEXIO1_07
 */

#define FLEXIO1_PWM0_MUX IOMUX_CFG(IOMUXC_PAD_GPIO_IO04_FLEXIO1_FLEXIO04, IOMUXC_PAD_FSEL_SFAST | IOMUXC_PAD_DSE_X6, 0)
#define FLEXIO1_PWM1_MUX IOMUX_CFG(IOMUXC_PAD_GPIO_IO05_FLEXIO1_FLEXIO05, IOMUXC_PAD_FSEL_SFAST | IOMUXC_PAD_DSE_X6, 0)
#define FLEXIO1_PWM2_MUX IOMUX_CFG(IOMUXC_PAD_GPIO_IO06_FLEXIO1_FLEXIO06, IOMUXC_PAD_FSEL_SFAST | IOMUXC_PAD_DSE_X6, 0)
#define FLEXIO1_PWM3_MUX IOMUX_CFG(IOMUXC_PAD_GPIO_IO07_FLEXIO1_FLEXIO07, IOMUXC_PAD_FSEL_SFAST | IOMUXC_PAD_DSE_X6, 0)

/* LPI2Cs */

/* TPM3 ch3 to PWM pin GPIO_IO24 muxing */

#define TPM3_PWM3_MUX IOMUX_CFG(IOMUXC_PAD_GPIO_IO24_TPM3_CH3, IOMUXC_PAD_FSEL_SFAST | IOMUXC_PAD_DSE_X6, 0)

/* LPI2Cs */

#define MUX_LPI2C1_SCL       IOMUX_CFG(IOMUXC_PAD_I2C1_SCL_LPI2C1_SCL, IOMUX_LPI2C_DEFAULT, IOMUXC_MUX_SION_ON)
#define MUX_LPI2C1_SDA       IOMUX_CFG(IOMUXC_PAD_I2C1_SDA_LPI2C1_SDA, IOMUX_LPI2C_DEFAULT, IOMUXC_MUX_SION_ON)

/* I2C reset functionality */

#define GPIO_LPI2C1_SCL_RESET  (GPIO_PORT1 | GPIO_PIN0 | GPIO_OUTPUT | GPIO_OUTPUT_ONE)
#define GPIO_LPI2C1_SDA_RESET  (GPIO_PORT1 | GPIO_PIN1 | GPIO_OUTPUT | GPIO_OUTPUT_ONE)

/* LPSPIs */

#define MUX_LPSPI3_SCK       IOMUX_CFG(IOMUXC_PAD_GPIO_IO11_LPSPI3_SCK,  IOMUX_LPSPI_DEFAULT, IOMUXC_MUX_SION_ON)
#define MUX_LPSPI3_MOSI      IOMUX_CFG(IOMUXC_PAD_GPIO_IO10_LPSPI3_SOUT, IOMUX_LPSPI_DEFAULT, IOMUXC_MUX_SION_ON)
#define MUX_LPSPI3_MISO      IOMUX_CFG(IOMUXC_PAD_GPIO_IO09_LPSPI3_SIN,  IOMUX_LPSPI_DEFAULT, IOMUXC_MUX_SION_ON)
#define MUX_LPSPI6_SCK       IOMUX_CFG(IOMUXC_PAD_GPIO_IO03_LPSPI6_SCK,  IOMUX_LPSPI_DEFAULT, IOMUXC_MUX_SION_ON)
#define MUX_LPSPI6_MOSI      IOMUX_CFG(IOMUXC_PAD_GPIO_IO02_LPSPI6_SOUT, IOMUX_LPSPI_DEFAULT, IOMUXC_MUX_SION_ON)
#define MUX_LPSPI6_MISO      IOMUX_CFG(IOMUXC_PAD_GPIO_IO01_LPSPI6_SIN,  IOMUX_LPSPI_DEFAULT, IOMUXC_MUX_SION_ON)

/* SPI CS */

#define MUX_LPSPI3_CS        IOMUX_CFG(IOMUXC_PAD_GPIO_IO08_GPIO2_IO08,  IOMUX_GPIO_DEFAULT, IOMUXC_MUX_SION_ON)
#define GPIO_LPSPI3_CS       (GPIO_PORT2 | GPIO_PIN8 | GPIO_OUTPUT | GPIO_OUTPUT_ONE)
#define MUX_LPSPI6_CS        IOMUX_CFG(IOMUXC_PAD_GPIO_IO00_GPIO2_IO00,  IOMUX_GPIO_DEFAULT, IOMUXC_MUX_SION_ON)
#define GPIO_LPSPI6_CS       (GPIO_PORT2 | GPIO_PIN0 | GPIO_OUTPUT | GPIO_OUTPUT_ONE)

/* USDHC */

/* Note: Need to set the SION for cmd and data pads (ERR052021) */

#define PIN_USDHC2_D0_MUX IOMUX_CFG(IOMUXC_PAD_SD2_DATA0_USDHC2_DATA0, IOMUXC_PAD_DSE_X1 | IOMUXC_PAD_FSEL_FAST | IOMUXC_PAD_PU_ON | IOMUXC_PAD_HYS_ST_ON, IOMUXC_MUX_SION_ON)
#define PIN_USDHC2_D1_MUX IOMUX_CFG(IOMUXC_PAD_SD2_DATA1_USDHC2_DATA1, IOMUXC_PAD_DSE_X1 | IOMUXC_PAD_FSEL_FAST | IOMUXC_PAD_PU_ON | IOMUXC_PAD_HYS_ST_ON, IOMUXC_MUX_SION_ON)
#define PIN_USDHC2_D2_MUX IOMUX_CFG(IOMUXC_PAD_SD2_DATA2_USDHC2_DATA2, IOMUXC_PAD_DSE_X1 | IOMUXC_PAD_FSEL_FAST | IOMUXC_PAD_PU_ON | IOMUXC_PAD_HYS_ST_ON, IOMUXC_MUX_SION_ON)
#define PIN_USDHC2_D3_MUX IOMUX_CFG(IOMUXC_PAD_SD2_DATA3_USDHC2_DATA3, IOMUXC_PAD_DSE_X1 | IOMUXC_PAD_FSEL_FAST | IOMUXC_PAD_PU_ON | IOMUXC_PAD_HYS_ST_ON, IOMUXC_MUX_SION_ON)
#define PIN_USDHC2_DCLK_MUX IOMUX_CFG(IOMUXC_PAD_SD2_DATA1_USDHC2_DATA1, IOMUXC_PAD_DSE_X6 | IOMUXC_PAD_FSEL_FAST | IOMUXC_PAD_PD_ON | IOMUXC_PAD_HYS_ST_ON, 0)
#define PIN_USDHC2_CMD_MUX IOMUX_CFG(IOMUXC_PAD_SD2_CMD_USDHC2_CMD, IOMUXC_PAD_DSE_X1 | IOMUXC_PAD_FSEL_FAST | IOMUXC_PAD_PU_ON | IOMUXC_PAD_HYS_ST_ON, IOMUXC_MUX_SION_ON)
#define PIN_USDHC2_CD_MUX IOMUX_CFG(IOMUXC_PAD_SD2_DATA3_USDHC2_DATA3, IOMUXC_PAD_DSE_X4 | IOMUXC_PAD_FSEL_FAST, 0)
#define PIN_USDHC2_VSELECT_MUX IOMUX_CFG(IOMUXC_PAD_SD2_VSELECT_USDHC2_VSELECT, IOMUXC_PAD_DSE_X5 | IOMUXC_PAD_FSEL_SFAST | IOMUXC_PAD_PD_ON, 0)

/* 390 KHz for initial inquiry stuff */

#define BOARD_USDHC_IDMODE_PRESCALER    USDHC_SYSCTL_SDCLKFS_DIV256
#define BOARD_USDHC_IDMODE_DIVISOR      USDHC_SYSCTL_DVS_DIV(2)

/* 25MHz for 1-bit wide bus */

#define BOARD_USDHC_MMCMODE_PRESCALER   USDHC_SYSCTL_SDCLKFS_DIV8
#define BOARD_USDHC_MMCMODE_DIVISOR     USDHC_SYSCTL_DVS_DIV(1)

#define BOARD_USDHC_SD1MODE_PRESCALER   USDHC_SYSCTL_SDCLKFS_DIV8
#define BOARD_USDHC_SD1MODE_DIVISOR     USDHC_SYSCTL_DVS_DIV(1)

/* 50MHz for 4-bit wide bus */

#define BOARD_USDHC_SD4MODE_PRESCALER   USDHC_SYSCTL_SDCLKFS_DIV4
#define BOARD_USDHC_SD4MODE_DIVISOR     USDHC_SYSCTL_DVS_DIV(1)

/* Set the PLL clocks as follows:
 *
 * - OSC24M       : 24   MHz
 * - ARMPLL_OUT   : 1692 MHz
 * - DRAMPLL      : 933  MHz
 * - SYSPLL1      : 4000 MHz
 * - SYSPLL_PFD0  : 1000 MHz
 * - SYSPLL_PFD1  : 800  MHz
 * - SYSPLL_PFD2  : 625  MHz
 * - AUDIOPLL_OUT : OFF
 * - VIDEOPLL_OUT : OFF
 *
 * After reset all clock sources (OSCPLL) and root clocks (CLOCK_ROOT) are
 * running, but gated (LPCG).
 *
 * By default, all peripheral root clocks are set to the 24 MHz oscillator.
 */

#define ARMPLL_CFG  PLL_CFG(IMX9_ARMPLL_BASE, false, PLL_PARMS(1, 2, 141, 0, 0))
#define DRAMPLL_CFG PLL_CFG(IMX9_DRAMPLL_BASE, true, PLL_PARMS(1, 4, 155, 1, 2))

#define PLL_CFGS \
  { \
    PLL_CFG(IMX9_SYSPLL_BASE,  true, PLL_PARMS(1, 4, 166, 2, 3)), \
  }

#define PFD_CFGS \
  { \
    PFD_CFG(IMX9_SYSPLL_BASE, 0, PFD_PARMS(4, 0, true)), \
    PFD_CFG(IMX9_SYSPLL_BASE, 1, PFD_PARMS(5, 0, true)), \
    PFD_CFG(IMX9_SYSPLL_BASE, 2, PFD_PARMS(6, 2, true)), \
  }

/* Ethernet configuration */

#define BOARD_ENET1_PHY_LIST                             \
{                                                        \
  {                                                      \
    .name = GMII_RTL8211F_NAME,                           \
    .id1 = GMII_PHYID1_RTL8211F,                                \
    .id2 = GMII_PHYID2_RTL8211F,                                \
    .status = GMII_RTL8211F_PHYSR_A43,                             \
    .address_lo = 2,                                                   \
    .address_high = 0xffff,                                              \
    .mbps10 = GMII_RTL8211F_PHYSR_10MBPS,                          \
    .mbps100 = GMII_RTL8211F_PHYSR_100MBPS,                         \
    .duplex = GMII_RTL8211F_PHYSR_DUPLEX,                          \
    .clause = 22,                                                  \
    .mbps1000 = GMII_RTL8211F_PHYSR_1000MBPS,                       \
    .speed_mask = GMII_RTL8211F_PHYSR_SPEED_MASK,                    \
  },                                                     \
}

#endif /* CONFIG_IMX9_ENET1 */

#ifdef CONFIG_IMX9_ENET1

#define MUX_ENET1_MDIO        IOMUX_CFG(IOMUXC_PAD_ENET2_MDIO_ENET1_MDIO, IOMUXC_PAD_FSEL_FAST | IOMUXC_PAD_DSE_X6, IOMUXC_MUX_SION_ON)
#define MUX_ENET1_MDC         IOMUX_CFG(IOMUXC_PAD_ENET2_MDC_ENET1_MDC, IOMUXC_PAD_FSEL_FAST | IOMUXC_PAD_DSE_X6, 0)

#define MUX_ENET1_RX_DATA00   IOMUX_CFG(IOMUXC_PAD_ENET2_RD0_ENET1_RGMII_RD0, 0, 0)
#define MUX_ENET1_RX_DATA01   IOMUX_CFG(IOMUXC_PAD_ENET2_RD1_ENET1_RGMII_RD1, 0, 0)

#define MUX_ENET1_TX_DATA00   IOMUX_CFG(IOMUXC_PAD_ENET2_TD0_ENET1_RGMII_TD0, IOMUXC_PAD_FSEL_FAST | IOMUXC_PAD_DSE_X6, 0)
#define MUX_ENET1_TX_DATA01   IOMUX_CFG(IOMUXC_PAD_ENET2_TD1_ENET1_RGMII_TD1, IOMUXC_PAD_FSEL_FAST | IOMUXC_PAD_DSE_X6, 0)

#if defined(CONFIG_IMX9_ENET1_RGMII)

#  define MUX_ENET1_RX_DATA02 IOMUX_CFG(IOMUXC_PAD_ENET2_RD2_ENET1_RGMII_RD2, 0, 0)
#  define MUX_ENET1_RX_DATA03 IOMUX_CFG(IOMUXC_PAD_ENET2_RD3_ENET1_RGMII_RD3, 0, 0)
#  define MUX_ENET1_TX_DATA02 IOMUX_CFG(IOMUXC_PAD_ENET2_TD2_ENET1_RGMII_TD2, IOMUXC_PAD_FSEL_FAST | IOMUXC_PAD_DSE_X6, 0)
#  define MUX_ENET1_TX_DATA03 IOMUX_CFG(IOMUXC_PAD_ENET2_TD3_ENET1_RGMII_TD3, IOMUXC_PAD_FSEL_FAST | IOMUXC_PAD_DSE_X6, 0)
#  define MUX_ENET1_RXC       IOMUX_CFG(IOMUXC_PAD_ENET2_RXC_ENET1_RGMII_RXC, 0, 0)
#  define MUX_ENET1_TX_CTL    IOMUX_CFG(IOMUXC_PAD_ENET2_TX_CTL_ENET1_RGMII_TX_CTL, IOMUXC_PAD_FSEL_FAST | IOMUXC_PAD_DSE_X6, 0)
#  define MUX_ENET1_RX_CTL    IOMUX_CFG(IOMUXC_PAD_ENET2_RX_CTL_ENET1_RGMII_RX_CTL, 0, 0)

#elif defined(CONFIG_IMX9_ENET1_RMII)

/* Same pin as TX_CTL for RGMII */

#  define MUX_ENET1_TX_EN     IOMUX_CFG(IOMUXC_PAD_ENET2_TX_CTL_ENET1_RGMII_TX_CTL, IOMUXC_PAD_FSEL_FAST | IOMUXC_PAD_DSE_X6, 0)

/* Same pin as TX_DATA02 for RGMII */

#  define MUX_ENET1_REF_CLK   IOMUX_CFG(IOMUXC_PAD_ENET2_TD2_ENET1_RGMII_TD2, IOMUXC_PAD_FSEL_FAST | IOMUXC_PAD_DSE_X6, 0)

/* Same pin as RX_CTL for RGMII */

#  define MUX_ENET1_CRS_DV    IOMUX_CFG(IOMUXC_PAD_ENET2_RX_CTL_ENET1_RGMII_RX_CTL, 0, 0)

#else
#error ENET1 supports only RMII and RGMII
#endif

#define BOARD_ENET1_PHY_LIST                             \
{                                                        \
  {                                                      \
    GMII_RTL8211F_NAME,                                  \
    GMII_PHYID1_RTL8211F,                                \
    GMII_PHYID2_RTL8211F,                                \
    GMII_RTL8211F_PHYSR_A43,                             \
    2,                                                   \
    0xffff,                                              \
    GMII_RTL8211F_PHYSR_10MBPS,                          \
    GMII_RTL8211F_PHYSR_100MBPS,                         \
    GMII_RTL8211F_PHYSR_DUPLEX,                          \
    22,                                                  \
    GMII_RTL8211F_PHYSR_1000MBPS,                        \
    GMII_RTL8211F_PHYSR_SPEED_MASK,                      \
  },                                                     \
}

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
#endif /* __BOARDS_ARM64_IMX9_IMX93_EVK_INCLUDE_BOARD_H */
