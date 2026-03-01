/****************************************************************************
 * arch/arm/src/am67/am67_pinmux.h
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

#ifndef __ARCH_ARM_SRC_AM67_AM67_PINMUX_H
#define __ARCH_ARM_SRC_AM67_AM67_PINMUX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CSL_PADCFG_CTRL0_CFG0_BASE            (0xf0000ul)
#define CSL_PADCFG_CTRL0_CFG0_SIZE            (0x8000ul)

#define CSL_MCU_PADCFG_CTRL0_CFG0_BASE        (0x4080000ul)
#define CSL_MCU_PADCFG_CTRL0_CFG0_SIZE        (0x8000ul)

#define PADCFG_PMUX_OFFSET                    (0x4000u)

#define CSL_MAIN_PADCONFIG_LOCK0_KICK0_OFFSET (0x1008)
#define CSL_MAIN_PADCONFIG_LOCK1_KICK0_OFFSET (0x5008)
#define CSL_MCU_PADCONFIG_LOCK0_KICK0_OFFSET  (0x1008)
#define CSL_MCU_PADCONFIG_LOCK1_KICK0_OFFSET  (0x5008)

#define KICK_LOCK_VAL                         (0x00000000u)
#define KICK0_UNLOCK_VAL                      (0x68ef3490u)
#define KICK1_UNLOCK_VAL                      (0xd172bc5au)

#define PINMUX_END                            (-1)

#define PIN_MODE(mode)                        ((uint32_t) mode)
#define PIN_PULL_DISABLE                      (((uint32_t) 0x1u) << 16u)
#define PIN_PULL_DIRECTION                    (((uint32_t) 0x1u) << 17u)
#define PIN_INPUT_ENABLE                      (((uint32_t) 0x1u) << 18u)
#define PIN_OUTPUT_DISABLE                    (((uint32_t) 0x1u) << 21u)
#define PIN_WAKEUP_ENABLE                     (((uint32_t) 0x1u) << 29u)

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum pinmux_main_offsets_e
{
  PIN_MMC1_DAT1      = 0x022c,
  PIN_MMC1_DAT0      = 0x0230,
  PIN_EXT_REFCLK1    = 0x01f0,
  PIN_MMC1_DAT3      = 0x0224,
  PIN_MMC1_DAT2      = 0x0228,
  PIN_VOUT0_VSYNC    = 0x0100,
  PIN_VOUT0_HSYNC    = 0x00f8,
  PIN_VOUT0_PCLK     = 0x0104,
  PIN_VOUT0_DE       = 0x00fc,
  PIN_VOUT0_DATA0    = 0x00b8,
  PIN_VOUT0_DATA1    = 0x00bc,
  PIN_VOUT0_DATA2    = 0x00c0,
  PIN_VOUT0_DATA3    = 0x00c4,
  PIN_VOUT0_DATA4    = 0x00c8,
  PIN_VOUT0_DATA5    = 0x00cc,
  PIN_VOUT0_DATA6    = 0x00d0,
  PIN_VOUT0_DATA7    = 0x00d4,
  PIN_VOUT0_DATA8    = 0x00d8,
  PIN_VOUT0_DATA9    = 0x00dc,
  PIN_VOUT0_DATA10   = 0x00e0,
  PIN_VOUT0_DATA11   = 0x00e4,
  PIN_VOUT0_DATA12   = 0x00e8,
  PIN_VOUT0_DATA13   = 0x00ec,
  PIN_VOUT0_DATA14   = 0x00f0,
  PIN_VOUT0_DATA15   = 0x00f4,
  PIN_GPMC0_AD8      = 0x005c,
  PIN_GPMC0_AD9      = 0x0060,
  PIN_GPMC0_AD10     = 0x0064,
  PIN_GPMC0_AD11     = 0x0068,
  PIN_GPMC0_AD12     = 0x006c,
  PIN_GPMC0_AD13     = 0x0070,
  PIN_GPMC0_AD14     = 0x0074,
  PIN_GPMC0_AD15     = 0x0078,
  PIN_GPMC0_WAIT1    = 0x009c,
  PIN_SPI0_CS1       = 0x01bb,
  PIN_UART0_TXD      = 0x01cc,
  PIN_UART0_RXD      = 0x01c8,
  PIN_SPI0_CS0       = 0x01b4,
  PIN_MMC0_DAT3      = 0x0208,
  PIN_I2C1_SCL       = 0x01e8,
  PIN_I2C1_SDA       = 0x01ec,
  PIN_MMC0_CLK       = 0x0218,
  PIN_MMC0_CMD       = 0x0220,
  PIN_SPI0_CLK       = 0x01bc,
  PIN_SPI0_D0        = 0x01c0,
  PIN_I2C0_SCL       = 0x01e0,
  PIN_I2C0_SDA       = 0x01e4,
  PIN_SPI0_D1        = 0x01c4,
  PIN_MMC0_DAT0      = 0x0214,
  PIN_MMC0_DAT5      = 0x0200,
  PIN_MCAN0_TX       = 0x01d8,
  PIN_MCAN0_RX       = 0x01dc,
  PIN_MCASP0_ACLKX   = 0x01a4,
  PIN_MCASP0_AFSX    = 0x01a8,
  PIN_MCASP0_ACLKR   = 0x01b0,
  PIN_MCASP0_AFSR    = 0x01ac,
  PIN_RGMII2_RD2     = 0x018c,
  PIN_RGMII2_RD3     = 0x0190,
  PIN_RGMII2_TD2     = 0x0174,
  PIN_GPMC0_DIR      = 0x00a4,
  PIN_MCASP0_AXR3    = 0x0194,
  PIN_MCASP0_AXR2    = 0x0198,
  PIN_MCASP0_AXR0    = 0x01a0,
  PIN_MCASP0_AXR1    = 0x019c,
  PIN_GPMC0_CSN3     = 0x00b4,
  PIN_GPMC0_WPN      = 0x00a0,
  PIN_GPMC0_AD0      = 0x003c,
  PIN_GPMC0_AD1      = 0x0040,
  PIN_GPMC0_AD2      = 0x0044,
  PIN_GPMC0_AD3      = 0x0048,
  PIN_GPMC0_AD4      = 0x004c,
  PIN_GPMC0_AD5      = 0x0050,
  PIN_GPMC0_AD6      = 0x0054,
  PIN_GPMC0_AD7      = 0x0058,
  PIN_GPMC0_WAIT0    = 0x0098,
  PIN_GPMC0_BE1N     = 0x0094,
  PIN_GPMC0_CSN0     = 0x00a8,
  PIN_GPMC0_CLK      = 0x007c,
  PIN_GPMC0_ADVN_ALE = 0x0084,
  PIN_GPMC0_OEN_REN  = 0x0088,
  PIN_GPMC0_WEN      = 0x008c,
  PIN_GPMC0_BE0N_CLE = 0x0090,
  PIN_UART0_CTSN     = 0x01d0,
  PIN_UART0_RTSN     = 0x01d4,
  PIN_GPMC0_CSN2     = 0x00b0,
  PIN_MMC0_DAT6      = 0x01fc,
  PIN_MMC0_DAT7      = 0x01f8,
  PIN_OSPI0_D6       = 0x0024,
  PIN_OSPI0_D7       = 0x0028,
  PIN_OSPI0_D5       = 0x0020,
  PIN_RGMII2_TD3     = 0x0178,
  PIN_RGMII2_TX_CTL  = 0x0164,
  PIN_MDIO0_MDC      = 0x0160,
  PIN_MDIO0_MDIO     = 0x015c,
  PIN_MMC0_DAT1      = 0x0210,
  PIN_MMC0_DAT2      = 0x020c,
  PIN_MMC0_DAT4      = 0x0204,
  PIN_MMC1_CMD       = 0x023c,
  PIN_MMC1_CLK       = 0x0234,
  PIN_MMC1_SDCD      = 0x0240,
  PIN_MMC1_SDWP      = 0x0244,
  PIN_MMC2_CMD       = 0x0120,
  PIN_MMC2_CLK       = 0x0118,
  PIN_MMC2_DAT0      = 0x0114,
  PIN_MMC2_DAT1      = 0x0110,
  PIN_MMC2_DAT2      = 0x010c,
  PIN_MMC2_DAT3      = 0x0108,
  PIN_MMC2_SDCD      = 0x0124,
  PIN_MMC2_SDWP      = 0x0128,
  PIN_OLDI0_A0N      = 0x0260,
  PIN_OLDI0_A0P      = 0x025c,
  PIN_OLDI0_A1N      = 0x0268,
  PIN_OLDI0_A1P      = 0x0264,
  PIN_OLDI0_A2N      = 0x0270,
  PIN_OLDI0_A2P      = 0x026c,
  PIN_OLDI0_A3N      = 0x0278,
  PIN_OLDI0_A3P      = 0x0274,
  PIN_OLDI0_A4N      = 0x0280,
  PIN_OLDI0_A4P      = 0x027c,
  PIN_OLDI0_A5N      = 0x0288,
  PIN_OLDI0_A5P      = 0x0284,
  PIN_OLDI0_A6N      = 0x0290,
  PIN_OLDI0_A6P      = 0x028c,
  PIN_OLDI0_A7N      = 0x0298,
  PIN_OLDI0_A7P      = 0x0294,
  PIN_OLDI0_CLK0N    = 0x02a0,
  PIN_OLDI0_CLK0P    = 0x029c,
  PIN_OLDI0_CLK1N    = 0x02a8,
  PIN_OLDI0_CLK1P    = 0x02a4,
  PIN_OSPI0_CLK      = 0x0000,
  PIN_OSPI0_CSN0     = 0x002c,
  PIN_OSPI0_CSN1     = 0x0030,
  PIN_OSPI0_CSN2     = 0x0034,
  PIN_OSPI0_CSN3     = 0x0038,
  PIN_OSPI0_D0       = 0x000c,
  PIN_OSPI0_D1       = 0x0010,
  PIN_OSPI0_D2       = 0x0014,
  PIN_OSPI0_D3       = 0x0018,
  PIN_OSPI0_D4       = 0x001c,
  PIN_OSPI0_DQS      = 0x0008,
  PIN_RGMII1_RD0     = 0x014c,
  PIN_RGMII1_RD1     = 0x0150,
  PIN_RGMII1_RD2     = 0x0154,
  PIN_RGMII1_RD3     = 0x0158,
  PIN_RGMII1_RXC     = 0x0148,
  PIN_RGMII1_RX_CTL  = 0x0144,
  PIN_RGMII1_TD0     = 0x0134,
  PIN_RGMII1_TD1     = 0x0138,
  PIN_RGMII1_TD2     = 0x013c,
  PIN_RGMII1_TD3     = 0x0140,
  PIN_RGMII1_TXC     = 0x0130,
  PIN_RGMII1_TX_CTL  = 0x012c,
  PIN_RGMII2_RD0     = 0x0184,
  PIN_RGMII2_RD1     = 0x0188,
  PIN_RGMII2_RXC     = 0x0180,
  PIN_RGMII2_RX_CTL  = 0x017c,
  PIN_RGMII2_TD0     = 0x016c,
  PIN_RGMII2_TD1     = 0x0170,
  PIN_RGMII2_TXC     = 0x0168,
  PIN_EXTINTN        = 0x01f4,
  PIN_PORZ_OUT       = 0x0250,
  PIN_RESETSTATZ     = 0x024c,
  PIN_RESET_REQZ     = 0x0248,
  PIN_GPMC0_CSN1     = 0x00ac,
  PIN_OSPI0_LBCLKO   = 0x0004,
  PIN_USB1_DRVVBUS   = 0x0258,
  PIN_USB0_DRVVBUS   = 0x0254,
  PIN_PCIE0_CLKREQN  = 0x02ac,
};

enum pinmux_mcu_offsets_e
{
  PIN_EMU0            = 0x0078,
  PIN_EMU1            = 0x007c,
  PIN_TCK             = 0x0064,
  PIN_TDI             = 0x006c,
  PIN_TDO             = 0x0070,
  PIN_TMS             = 0x0074,
  PIN_TRSTN           = 0x0068,
  PIN_MCU_I2C0_SCL    = 0x0044,
  PIN_MCU_I2C0_SDA    = 0x0048,
  PIN_MCU_MCAN1_RX    = 0x0040,
  PIN_MCU_MCAN1_TX    = 0x003c,
  PIN_MCU_MCAN0_RX    = 0x0038,
  PIN_MCU_MCAN0_TX    = 0x0034,
  PIN_MCU_SPI0_CLK    = 0x0008,
  PIN_MCU_SPI0_CS0    = 0x0000,
  PIN_MCU_SPI0_D0     = 0x000c,
  PIN_MCU_SPI0_D1     = 0x0010,
  PIN_WKUP_UART0_RTSN = 0x0030,
  PIN_WKUP_UART0_CTSN = 0x002c,
  PIN_MCU_UART0_CTSN  = 0x001c,
  PIN_MCU_UART0_RTSN  = 0x0020,
  PIN_MCU_ERRORN      = 0x0060,
  PIN_MCU_SPI0_CS1    = 0x0004,
  PIN_MCU_PORZ        = 0x0058,
  PIN_MCU_RESETSTATZ  = 0x005c,
  PIN_MCU_RESETZ      = 0x0054,
  PIN_MCU_UART0_RXD   = 0x0014,
  PIN_MCU_UART0_TXD   = 0x0018,
  PIN_WKUP_I2C0_SCL   = 0x004c,
  PIN_WKUP_I2C0_SDA   = 0x0050,
  PIN_WKUP_CLKOUT0    = 0x0084,
  PIN_PMIC_LPM_EN0    = 0x0080,
  PIN_WKUP_UART0_RXD  = 0x0024,
  PIN_WKUP_UART0_TXD  = 0x0028,
};

struct pinmux_conf_s
{
  int16_t offset;
  uint32_t setting;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: am67_pinmux_config
 *
 * Description:
 *   Configure pin multiplexing settings by writing configuration values to
 *   pad configuration registers after unlocking the pinmux registers.
 *
 ****************************************************************************/

void am67_pinmux_config(const struct pinmux_conf_s *pinmux_conf);

/****************************************************************************
 * Name: am67_pinmux_init
 *
 * Description:
 *   Initialize pin multiplexing using the global pinmux configuration array.
 *
 ****************************************************************************/

void am67_pinmux_init(void);

#endif /* __ARCH_ARM_SRC_AM67_AM67_PINMUX_H */
