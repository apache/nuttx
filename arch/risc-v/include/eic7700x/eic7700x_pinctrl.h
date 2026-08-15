/****************************************************************************
 * arch/risc-v/include/eic7700x/eic7700x_pinctrl.h
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

#ifndef __ARCH_RISCV_INCLUDE_EIC7700X_EIC7700X_PINCTRL_H
#define __ARCH_RISCV_INCLUDE_EIC7700X_EIC7700X_PINCTRL_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Every pad of the Chip Level Mode Mux, in register order.
 *
 * The pad registers are contiguous, so a pad id is simply the register
 * index and the address falls out of it.  The names are the manual's own,
 * taken from the register detail description in TRM part 4 section 12.1.2.2
 * pages 371 to 410, which means a name describes the pad's default function
 * rather than whatever it has been muxed to.  EIC7700X_PAD_UART0_TX is the
 * pad that comes up as UART0 TX, not a pad that is necessarily still UART.
 *
 * Eight registers the manual leaves as RESERVED are numbered in offset
 * order.  They are real registers with real reset values, so they are
 * carried here rather than left as holes; the pad table records that their
 * function select cannot be driven.
 *
 * Two balls in part 1 table 2-4 have no register in this range at all,
 * GPIO5 and GPIO11, so they have no pad id.  See the note on the reverse
 * map in eic7700x_pinctrl.c.
 */

#define EIC7700X_PAD_NPADS         (166)
#define EIC7700X_PAD_OFFSET(id)    (0x0080 + ((id) << 2))

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum eic7700x_pad_e
{
  /* Chip mode, straps and the oscillator pads */

  EIC7700X_PAD_CHIP_MODE             =   0,  /* 0x080 */
  EIC7700X_PAD_MODE_SET0             =   1,  /* 0x084 */
  EIC7700X_PAD_MODE_SET1             =   2,  /* 0x088 */
  EIC7700X_PAD_MODE_SET2             =   3,  /* 0x08c */
  EIC7700X_PAD_MODE_SET3             =   4,  /* 0x090 */
  EIC7700X_PAD_XIN                   =   5,  /* 0x094 */
  EIC7700X_PAD_RESERVED0             =   6,  /* 0x098 */
  EIC7700X_PAD_RST_OUT_N             =   7,  /* 0x09c */
  EIC7700X_PAD_KEY_RESET_N           =   8,  /* 0x0a0 */
  EIC7700X_PAD_RESERVED1             =   9,  /* 0x0a4 */
  EIC7700X_PAD_RESERVED2             =  10,  /* 0x0a8 */
  EIC7700X_PAD_RESERVED3             =  11,  /* 0x0ac */
  EIC7700X_PAD_GPIO0                 =  12,  /* 0x0b0 */
  EIC7700X_PAD_POR_SEL               =  13,  /* 0x0b4 */

  /* JTAG, SPI2 chip selects, PCIe control and HDMI */

  EIC7700X_PAD_JTAG0_TCK             =  14,  /* 0x0b8 */
  EIC7700X_PAD_JTAG0_TMS             =  15,  /* 0x0bc */
  EIC7700X_PAD_JTAG0_TDI             =  16,  /* 0x0c0 */
  EIC7700X_PAD_JTAG0_TDO             =  17,  /* 0x0c4 */
  EIC7700X_PAD_JTAG0_TRST            =  18,  /* 0x0c8 */
  EIC7700X_PAD_SPI2_CS0_N            =  19,  /* 0x0cc */
  EIC7700X_PAD_JTAG1_TCK             =  20,  /* 0x0d0 */
  EIC7700X_PAD_JTAG1_TMS             =  21,  /* 0x0d4 */
  EIC7700X_PAD_JTAG1_TDI             =  22,  /* 0x0d8 */
  EIC7700X_PAD_JTAG1_TDO             =  23,  /* 0x0dc */
  EIC7700X_PAD_JTAG1_TRST            =  24,  /* 0x0e0 */
  EIC7700X_PAD_SPI2_CS1_N            =  25,  /* 0x0e4 */
  EIC7700X_PAD_PCIE_CLKREQ_N         =  26,  /* 0x0e8 */
  EIC7700X_PAD_PCIE_WAKE_N           =  27,  /* 0x0ec */
  EIC7700X_PAD_PCIE_PERST_N          =  28,  /* 0x0f0 */
  EIC7700X_PAD_HDMI_SCL              =  29,  /* 0x0f4 */
  EIC7700X_PAD_HDMI_SDA              =  30,  /* 0x0f8 */
  EIC7700X_PAD_HDMI_CEC              =  31,  /* 0x0fc */
  EIC7700X_PAD_JTAG2_TRST            =  32,  /* 0x100 */

  /* Gigabit Ethernet, I2S, SPI1 and the dedicated GPIO pads */

  EIC7700X_PAD_RGMII0_CLK_125        =  33,  /* 0x104 */
  EIC7700X_PAD_RGMII0_TXEN           =  34,  /* 0x108 */
  EIC7700X_PAD_RGMII0_TXCLK          =  35,  /* 0x10c */
  EIC7700X_PAD_RGMII0_TXD0           =  36,  /* 0x110 */
  EIC7700X_PAD_RGMII0_TXD1           =  37,  /* 0x114 */
  EIC7700X_PAD_RGMII0_TXD2           =  38,  /* 0x118 */
  EIC7700X_PAD_RGMII0_TXD3           =  39,  /* 0x11c */
  EIC7700X_PAD_I2S0_BCLK             =  40,  /* 0x120 */
  EIC7700X_PAD_I2S0_WCLK             =  41,  /* 0x124 */
  EIC7700X_PAD_I2S0_SDI              =  42,  /* 0x128 */
  EIC7700X_PAD_I2S0_SDO              =  43,  /* 0x12c */
  EIC7700X_PAD_I2S_MCLK              =  44,  /* 0x130 */
  EIC7700X_PAD_RGMII0_RXCLK          =  45,  /* 0x134 */
  EIC7700X_PAD_RGMII0_RXDV           =  46,  /* 0x138 */
  EIC7700X_PAD_RGMII0_RXD0           =  47,  /* 0x13c */
  EIC7700X_PAD_RGMII0_RXD1           =  48,  /* 0x140 */
  EIC7700X_PAD_RGMII0_RXD2           =  49,  /* 0x144 */
  EIC7700X_PAD_RGMII0_RXD3           =  50,  /* 0x148 */
  EIC7700X_PAD_I2S2_BCLK             =  51,  /* 0x14c */
  EIC7700X_PAD_I2S2_WCLK             =  52,  /* 0x150 */
  EIC7700X_PAD_I2S2_SDI              =  53,  /* 0x154 */
  EIC7700X_PAD_I2S2_SDO              =  54,  /* 0x158 */
  EIC7700X_PAD_GPIO27                =  55,  /* 0x15c */
  EIC7700X_PAD_GPIO28                =  56,  /* 0x160 */
  EIC7700X_PAD_GPIO29                =  57,  /* 0x164 */
  EIC7700X_PAD_RGMII0_MDC            =  58,  /* 0x168 */
  EIC7700X_PAD_RGMII0_MDIO           =  59,  /* 0x16c */
  EIC7700X_PAD_RGMII0_INTB           =  60,  /* 0x170 */
  EIC7700X_PAD_RGMII1_CLK_125        =  61,  /* 0x174 */
  EIC7700X_PAD_RGMII1_TXEN           =  62,  /* 0x178 */
  EIC7700X_PAD_RGMII1_TXCLK          =  63,  /* 0x17c */
  EIC7700X_PAD_RGMII1_TXD0           =  64,  /* 0x180 */
  EIC7700X_PAD_RGMII1_TXD1           =  65,  /* 0x184 */
  EIC7700X_PAD_RGMII1_TXD2           =  66,  /* 0x188 */
  EIC7700X_PAD_RGMII1_TXD3           =  67,  /* 0x18c */
  EIC7700X_PAD_I2S1_BCLK             =  68,  /* 0x190 */
  EIC7700X_PAD_I2S1_WCLK             =  69,  /* 0x194 */
  EIC7700X_PAD_I2S1_SDI              =  70,  /* 0x198 */
  EIC7700X_PAD_I2S1_SDO              =  71,  /* 0x19c */
  EIC7700X_PAD_GPIO34                =  72,  /* 0x1a0 */
  EIC7700X_PAD_RGMII1_RXCLK          =  73,  /* 0x1a4 */
  EIC7700X_PAD_RGMII1_RXDV           =  74,  /* 0x1a8 */
  EIC7700X_PAD_RGMII1_RXD0           =  75,  /* 0x1ac */
  EIC7700X_PAD_RGMII1_RXD1           =  76,  /* 0x1b0 */
  EIC7700X_PAD_RGMII1_RXD2           =  77,  /* 0x1b4 */
  EIC7700X_PAD_RGMII1_RXD3           =  78,  /* 0x1b8 */
  EIC7700X_PAD_SPI1_CS0_N            =  79,  /* 0x1bc */
  EIC7700X_PAD_SPI1_CLK              =  80,  /* 0x1c0 */
  EIC7700X_PAD_SPI1_D0               =  81,  /* 0x1c4 */
  EIC7700X_PAD_SPI1_D1               =  82,  /* 0x1c8 */
  EIC7700X_PAD_SPI1_D2               =  83,  /* 0x1cc */
  EIC7700X_PAD_SPI1_D3               =  84,  /* 0x1d0 */
  EIC7700X_PAD_SPI1_CS1_N            =  85,  /* 0x1d4 */
  EIC7700X_PAD_RGMII1_MDC            =  86,  /* 0x1d8 */
  EIC7700X_PAD_RGMII1_MDIO           =  87,  /* 0x1dc */
  EIC7700X_PAD_RGMII1_INTB           =  88,  /* 0x1e0 */

  /* USB power, I2C 0-5, UART 0-2, JTAG2 and the fan */

  EIC7700X_PAD_USB0_PWREN            =  89,  /* 0x1e4 */
  EIC7700X_PAD_USB1_PWREN            =  90,  /* 0x1e8 */
  EIC7700X_PAD_I2C0_SCL              =  91,  /* 0x1ec */
  EIC7700X_PAD_I2C0_SDA              =  92,  /* 0x1f0 */
  EIC7700X_PAD_I2C1_SCL              =  93,  /* 0x1f4 */
  EIC7700X_PAD_I2C1_SDA              =  94,  /* 0x1f8 */
  EIC7700X_PAD_I2C2_SCL              =  95,  /* 0x1fc */
  EIC7700X_PAD_I2C2_SDA              =  96,  /* 0x200 */
  EIC7700X_PAD_I2C3_SCL              =  97,  /* 0x204 */
  EIC7700X_PAD_I2C3_SDA              =  98,  /* 0x208 */
  EIC7700X_PAD_I2C4_SCL              =  99,  /* 0x20c */
  EIC7700X_PAD_I2C4_SDA              = 100,  /* 0x210 */
  EIC7700X_PAD_I2C5_SCL              = 101,  /* 0x214 */
  EIC7700X_PAD_I2C5_SDA              = 102,  /* 0x218 */
  EIC7700X_PAD_UART0_TX              = 103,  /* 0x21c */
  EIC7700X_PAD_UART0_RX              = 104,  /* 0x220 */
  EIC7700X_PAD_UART1_TX              = 105,  /* 0x224 */
  EIC7700X_PAD_UART1_RX              = 106,  /* 0x228 */
  EIC7700X_PAD_UART1_CTS             = 107,  /* 0x22c */
  EIC7700X_PAD_UART1_RTS             = 108,  /* 0x230 */
  EIC7700X_PAD_UART2_TX              = 109,  /* 0x234 */
  EIC7700X_PAD_UART2_RX              = 110,  /* 0x238 */
  EIC7700X_PAD_JTAG2_TCK             = 111,  /* 0x23c */
  EIC7700X_PAD_JTAG2_TMS             = 112,  /* 0x240 */
  EIC7700X_PAD_JTAG2_TDI             = 113,  /* 0x244 */
  EIC7700X_PAD_JTAG2_TDO             = 114,  /* 0x248 */
  EIC7700X_PAD_FAN_PWM               = 115,  /* 0x24c */
  EIC7700X_PAD_FAN_TACH              = 116,  /* 0x250 */

  /* MIPI CSI camera sync and clock */

  EIC7700X_PAD_MIPI_CSI0_XVS         = 117,  /* 0x254 */
  EIC7700X_PAD_MIPI_CSI0_XHS         = 118,  /* 0x258 */
  EIC7700X_PAD_MIPI_CSI0_MCLK        = 119,  /* 0x25c */
  EIC7700X_PAD_MIPI_CSI1_XVS         = 120,  /* 0x260 */
  EIC7700X_PAD_MIPI_CSI1_XHS         = 121,  /* 0x264 */
  EIC7700X_PAD_MIPI_CSI1_MCLK        = 122,  /* 0x268 */
  EIC7700X_PAD_MIPI_CSI2_XVS         = 123,  /* 0x26c */
  EIC7700X_PAD_MIPI_CSI2_XHS         = 124,  /* 0x270 */
  EIC7700X_PAD_MIPI_CSI2_MCLK        = 125,  /* 0x274 */
  EIC7700X_PAD_MIPI_CSI3_XVS         = 126,  /* 0x278 */
  EIC7700X_PAD_MIPI_CSI3_XHS         = 127,  /* 0x27c */
  EIC7700X_PAD_MIPI_CSI3_MCLK        = 128,  /* 0x280 */
  EIC7700X_PAD_MIPI_CSI4_XVS         = 129,  /* 0x284 */
  EIC7700X_PAD_MIPI_CSI4_XHS         = 130,  /* 0x288 */
  EIC7700X_PAD_MIPI_CSI4_MCLK        = 131,  /* 0x28c */
  EIC7700X_PAD_MIPI_CSI5_XVS         = 132,  /* 0x290 */
  EIC7700X_PAD_MIPI_CSI5_XHS         = 133,  /* 0x294 */
  EIC7700X_PAD_MIPI_CSI5_MCLK        = 134,  /* 0x298 */

  /* SPI3, SPI0, I2C 10-11 and more dedicated GPIO */

  EIC7700X_PAD_SPI3_CS_N             = 135,  /* 0x29c */
  EIC7700X_PAD_SPI3_CLK              = 136,  /* 0x2a0 */
  EIC7700X_PAD_SPI3_DI               = 137,  /* 0x2a4 */
  EIC7700X_PAD_SPI3_DO               = 138,  /* 0x2a8 */
  EIC7700X_PAD_GPIO92                = 139,  /* 0x2ac */
  EIC7700X_PAD_GPIO93                = 140,  /* 0x2b0 */
  EIC7700X_PAD_S_MODE                = 141,  /* 0x2b4 */
  EIC7700X_PAD_GPIO95                = 142,  /* 0x2b8 */
  EIC7700X_PAD_SPI0_CS_N             = 143,  /* 0x2bc */
  EIC7700X_PAD_SPI0_CLK              = 144,  /* 0x2c0 */
  EIC7700X_PAD_SPI0_D0               = 145,  /* 0x2c4 */
  EIC7700X_PAD_SPI0_D1               = 146,  /* 0x2c8 */
  EIC7700X_PAD_SPI0_D2               = 147,  /* 0x2cc */
  EIC7700X_PAD_SPI0_D3               = 148,  /* 0x2d0 */
  EIC7700X_PAD_I2C10_SCL             = 149,  /* 0x2d4 */
  EIC7700X_PAD_I2C10_SDA             = 150,  /* 0x2d8 */
  EIC7700X_PAD_I2C11_SCL             = 151,  /* 0x2dc */
  EIC7700X_PAD_I2C11_SDA             = 152,  /* 0x2e0 */
  EIC7700X_PAD_GPIO106               = 153,  /* 0x2e4 */

  /* Boot select, reserved pads and the RGMII voltage mode */

  EIC7700X_PAD_BOOT_SEL0             = 154,  /* 0x2e8 */
  EIC7700X_PAD_BOOT_SEL1             = 155,  /* 0x2ec */
  EIC7700X_PAD_BOOT_SEL2             = 156,  /* 0x2f0 */
  EIC7700X_PAD_BOOT_SEL3             = 157,  /* 0x2f4 */
  EIC7700X_PAD_GPIO111               = 158,  /* 0x2f8 */
  EIC7700X_PAD_RESERVED4             = 159,  /* 0x2fc */
  EIC7700X_PAD_RESERVED5             = 160,  /* 0x300 */
  EIC7700X_PAD_RESERVED6             = 161,  /* 0x304 */
  EIC7700X_PAD_RESERVED7             = 162,  /* 0x308 */
  EIC7700X_PAD_LPDDR_REF_CLK         = 163,  /* 0x30c */
  EIC7700X_PAD_ADDR_RGMII0_SEL_MODE  = 164,  /* 0x310 */
  EIC7700X_PAD_ADDR_RGMII1_SEL_MODE  = 165,  /* 0x314 */
};

/* What each pad's function select selects, from part 1 table 2-4.
 *
 * A pad only accepts the values named here; the write path refuses any
 * other, even where the hardware would take it.  The names are the
 * manual's, so a constant reads as the signal that appears on the ball.
 */

/* Straps, oscillator, JTAG0 and JTAG1, SPI2 selects, PCIe, HDMI */

#define EIC7700X_PAD_CHIP_MODE_FUNC_CHIP_MODE                    0

#define EIC7700X_PAD_MODE_SET0_FUNC_SDIO0_DETECT                 0
#define EIC7700X_PAD_MODE_SET0_FUNC_GPIO13                       2

#define EIC7700X_PAD_MODE_SET1_FUNC_SDIO0_WRITE_PROT             0
#define EIC7700X_PAD_MODE_SET1_FUNC_GPIO14                       2

#define EIC7700X_PAD_MODE_SET2_FUNC_SDIO1_DETECT                 0
#define EIC7700X_PAD_MODE_SET2_FUNC_GPIO15                       2

#define EIC7700X_PAD_MODE_SET3_FUNC_SDIO1_WRITE_PROT             0
#define EIC7700X_PAD_MODE_SET3_FUNC_GPIO16                       2

#define EIC7700X_PAD_XIN_FUNC_XIN_24M                            0
#define EIC7700X_PAD_RESERVED0_FUNC_XOUT_24M                     0
#define EIC7700X_PAD_RST_OUT_N_FUNC_RST_OUT_N                    0
#define EIC7700X_PAD_KEY_RESET_N_FUNC_KEY_RESET_N                0
#define EIC7700X_PAD_GPIO0_FUNC_GPIO0                            0
#define EIC7700X_PAD_POR_SEL_FUNC_POR_SEL                        0

#define EIC7700X_PAD_JTAG0_TCK_FUNC_JTAG0_TCK                    0
#define EIC7700X_PAD_JTAG0_TCK_FUNC_SPI2_CLK                     1
#define EIC7700X_PAD_JTAG0_TCK_FUNC_GPIO1                        2

#define EIC7700X_PAD_JTAG0_TMS_FUNC_JTAG0_TMS                    0
#define EIC7700X_PAD_JTAG0_TMS_FUNC_SPI2_D0                      1
#define EIC7700X_PAD_JTAG0_TMS_FUNC_GPIO2                        2

#define EIC7700X_PAD_JTAG0_TDI_FUNC_JTAG0_TDI                    0
#define EIC7700X_PAD_JTAG0_TDI_FUNC_SPI2_D1                      1
#define EIC7700X_PAD_JTAG0_TDI_FUNC_GPIO3                        2

#define EIC7700X_PAD_JTAG0_TDO_FUNC_JTAG0_TDO                    0
#define EIC7700X_PAD_JTAG0_TDO_FUNC_SPI2_D2                      1
#define EIC7700X_PAD_JTAG0_TDO_FUNC_GPIO4                        2

#define EIC7700X_PAD_SPI2_CS0_N_FUNC_SPI2_CS0_N                  0
#define EIC7700X_PAD_SPI2_CS0_N_FUNC_GPIO6                       2

#define EIC7700X_PAD_JTAG1_TCK_FUNC_JTAG1_TCK                    0
#define EIC7700X_PAD_JTAG1_TCK_FUNC_GPIO7                        2

#define EIC7700X_PAD_JTAG1_TMS_FUNC_JTAG1_TMS                    0
#define EIC7700X_PAD_JTAG1_TMS_FUNC_GPIO8                        2

#define EIC7700X_PAD_JTAG1_TDI_FUNC_JTAG1_TDI                    0
#define EIC7700X_PAD_JTAG1_TDI_FUNC_GPIO9                        2

#define EIC7700X_PAD_JTAG1_TDO_FUNC_JTAG1_TDO                    0
#define EIC7700X_PAD_JTAG1_TDO_FUNC_GPIO10                       2

#define EIC7700X_PAD_SPI2_CS1_N_FUNC_SPI2_CS1_N                  0
#define EIC7700X_PAD_SPI2_CS1_N_FUNC_GPIO12                      2

#define EIC7700X_PAD_PCIE_CLKREQ_N_FUNC_PCIE_CLKREQ_N            0
#define EIC7700X_PAD_PCIE_WAKE_N_FUNC_PCIE_WAKE_N                0
#define EIC7700X_PAD_PCIE_PERST_N_FUNC_PCIE_PERST_N              0
#define EIC7700X_PAD_HDMI_SCL_FUNC_HDMI_SCL                      0
#define EIC7700X_PAD_HDMI_SDA_FUNC_HDMI_SDA                      0
#define EIC7700X_PAD_HDMI_CEC_FUNC_HDMI_CEC                      0

#define EIC7700X_PAD_JTAG2_TRST_FUNC_JTAG2_TRST                  0
#define EIC7700X_PAD_JTAG2_TRST_FUNC_GPIO17                      2

/* Gigabit Ethernet, I2S, SPI1 and the dedicated GPIO pads */

#define EIC7700X_PAD_RGMII0_CLK_125_FUNC_RGMII0_CLK_125          0
#define EIC7700X_PAD_RGMII0_TXEN_FUNC_RGMII0_TXEN                0
#define EIC7700X_PAD_RGMII0_TXCLK_FUNC_RGMII0_TXCLK              0
#define EIC7700X_PAD_RGMII0_TXD0_FUNC_RGMII0_TXD0                0
#define EIC7700X_PAD_RGMII0_TXD1_FUNC_RGMII0_TXD1                0
#define EIC7700X_PAD_RGMII0_TXD2_FUNC_RGMII0_TXD2                0
#define EIC7700X_PAD_RGMII0_TXD3_FUNC_RGMII0_TXD3                0

#define EIC7700X_PAD_I2S0_BCLK_FUNC_I2S0_BCLK                    0
#define EIC7700X_PAD_I2S0_BCLK_FUNC_GPIO18                       2

#define EIC7700X_PAD_I2S0_WCLK_FUNC_I2S0_WCLK                    0
#define EIC7700X_PAD_I2S0_WCLK_FUNC_GPIO19                       2

#define EIC7700X_PAD_I2S0_SDI_FUNC_I2S0_SDI                      0
#define EIC7700X_PAD_I2S0_SDI_FUNC_GPIO20                        2

#define EIC7700X_PAD_I2S0_SDO_FUNC_I2S0_SDO                      0
#define EIC7700X_PAD_I2S0_SDO_FUNC_GPIO21                        2

#define EIC7700X_PAD_I2S_MCLK_FUNC_I2S_MCLK                      0
#define EIC7700X_PAD_I2S_MCLK_FUNC_GPIO22                        2

#define EIC7700X_PAD_RGMII0_RXCLK_FUNC_RGMII0_RXCLK              0
#define EIC7700X_PAD_RGMII0_RXDV_FUNC_RGMII0_RXDV                0
#define EIC7700X_PAD_RGMII0_RXD0_FUNC_RGMII0_RXD0                0
#define EIC7700X_PAD_RGMII0_RXD1_FUNC_RGMII0_RXD1                0
#define EIC7700X_PAD_RGMII0_RXD2_FUNC_RGMII0_RXD2                0
#define EIC7700X_PAD_RGMII0_RXD3_FUNC_RGMII0_RXD3                0

#define EIC7700X_PAD_I2S2_BCLK_FUNC_I2S2_BCLK                    0
#define EIC7700X_PAD_I2S2_BCLK_FUNC_GPIO23                       2

#define EIC7700X_PAD_I2S2_WCLK_FUNC_I2S2_WCLK                    0
#define EIC7700X_PAD_I2S2_WCLK_FUNC_GPIO24                       2

#define EIC7700X_PAD_I2S2_SDI_FUNC_I2S2_SDI                      0
#define EIC7700X_PAD_I2S2_SDI_FUNC_GPIO25                        2

#define EIC7700X_PAD_I2S2_SDO_FUNC_I2S2_SDO                      0
#define EIC7700X_PAD_I2S2_SDO_FUNC_GPIO26                        2

#define EIC7700X_PAD_GPIO27_FUNC_GPIO27                          0
#define EIC7700X_PAD_GPIO27_FUNC_SATA_ACT_LED                    1

#define EIC7700X_PAD_GPIO28_FUNC_GPIO28                          0

#define EIC7700X_PAD_GPIO29_FUNC_POR_TIME_SEL0                   0
#define EIC7700X_PAD_GPIO29_FUNC_EMMC_LED_CONTROL                1
#define EIC7700X_PAD_GPIO29_FUNC_GPIO29                          2

#define EIC7700X_PAD_RGMII0_MDC_FUNC_RGMII0_MDC                  0
#define EIC7700X_PAD_RGMII0_MDIO_FUNC_RGMII0_MDIO                0
#define EIC7700X_PAD_RGMII0_INTB_FUNC_RGMII0_INTB                0
#define EIC7700X_PAD_RGMII1_CLK_125_FUNC_RGMII1_CLK_125          0
#define EIC7700X_PAD_RGMII1_TXEN_FUNC_RGMII1_TXEN                0
#define EIC7700X_PAD_RGMII1_TXCLK_FUNC_RGMII1_TXCLK              0
#define EIC7700X_PAD_RGMII1_TXD0_FUNC_RGMII1_TXD0                0
#define EIC7700X_PAD_RGMII1_TXD1_FUNC_RGMII1_TXD1                0
#define EIC7700X_PAD_RGMII1_TXD2_FUNC_RGMII1_TXD2                0
#define EIC7700X_PAD_RGMII1_TXD3_FUNC_RGMII1_TXD3                0

#define EIC7700X_PAD_I2S1_BCLK_FUNC_I2S1_BCLK                    0
#define EIC7700X_PAD_I2S1_BCLK_FUNC_GPIO30                       2

#define EIC7700X_PAD_I2S1_WCLK_FUNC_I2S1_WCLK                    0
#define EIC7700X_PAD_I2S1_WCLK_FUNC_GPIO31                       2

#define EIC7700X_PAD_I2S1_SDI_FUNC_I2S1_SDI                      0
#define EIC7700X_PAD_I2S1_SDI_FUNC_GPIO32                        2

#define EIC7700X_PAD_I2S1_SDO_FUNC_I2S1_SDO                      0
#define EIC7700X_PAD_I2S1_SDO_FUNC_GPIO33                        2

#define EIC7700X_PAD_GPIO34_FUNC_POR_TIME_SEL1                   0
#define EIC7700X_PAD_GPIO34_FUNC_SD0_LED_CONTROL                 1
#define EIC7700X_PAD_GPIO34_FUNC_GPIO34                          2

#define EIC7700X_PAD_RGMII1_RXCLK_FUNC_RGMII1_RXCLK              0
#define EIC7700X_PAD_RGMII1_RXDV_FUNC_RGMII1_RXDV                0
#define EIC7700X_PAD_RGMII1_RXD0_FUNC_RGMII1_RXD0                0
#define EIC7700X_PAD_RGMII1_RXD1_FUNC_RGMII1_RXD1                0
#define EIC7700X_PAD_RGMII1_RXD2_FUNC_RGMII1_RXD2                0
#define EIC7700X_PAD_RGMII1_RXD3_FUNC_RGMII1_RXD3                0

#define EIC7700X_PAD_SPI1_CS0_N_FUNC_SPI1_CS0_N                  0
#define EIC7700X_PAD_SPI1_CS0_N_FUNC_GPIO35                      2

#define EIC7700X_PAD_SPI1_CLK_FUNC_SPI1_CLK                      0
#define EIC7700X_PAD_SPI1_CLK_FUNC_GPIO36                        2

#define EIC7700X_PAD_SPI1_D0_FUNC_SPI1_D0                        0
#define EIC7700X_PAD_SPI1_D0_FUNC_I2C9_SCL                       1
#define EIC7700X_PAD_SPI1_D0_FUNC_GPIO37                         2
#define EIC7700X_PAD_SPI1_D0_FUNC_UART4_TX                       3

#define EIC7700X_PAD_SPI1_D1_FUNC_SPI1_D1                        0
#define EIC7700X_PAD_SPI1_D1_FUNC_I2C9_SDA                       1
#define EIC7700X_PAD_SPI1_D1_FUNC_GPIO38                         2
#define EIC7700X_PAD_SPI1_D1_FUNC_UART4_RX                       3

#define EIC7700X_PAD_SPI1_D2_FUNC_SPI1_D2                        0
#define EIC7700X_PAD_SPI1_D2_FUNC_SD1_LED_CONTROL                1
#define EIC7700X_PAD_SPI1_D2_FUNC_GPIO39                         2

#define EIC7700X_PAD_SPI1_D3_FUNC_SPI1_D3                        0
#define EIC7700X_PAD_SPI1_D3_FUNC_PWM1                           1
#define EIC7700X_PAD_SPI1_D3_FUNC_GPIO40                         2

#define EIC7700X_PAD_SPI1_CS1_N_FUNC_SPI1_CS1_N                  0
#define EIC7700X_PAD_SPI1_CS1_N_FUNC_PWM2                        1
#define EIC7700X_PAD_SPI1_CS1_N_FUNC_GPIO41                      2

#define EIC7700X_PAD_RGMII1_MDC_FUNC_RGMII1_MDC                  0
#define EIC7700X_PAD_RGMII1_MDIO_FUNC_RGMII1_MDIO                0
#define EIC7700X_PAD_RGMII1_INTB_FUNC_RGMII1_INTB                0

/* USB power, I2C 0 to 5, UART 0 to 2, JTAG2 and the fan */

#define EIC7700X_PAD_USB0_PWREN_FUNC_USB0_PWREN                  0
#define EIC7700X_PAD_USB0_PWREN_FUNC_GPIO42                      2

#define EIC7700X_PAD_USB1_PWREN_FUNC_USB1_PWREN                  0
#define EIC7700X_PAD_USB1_PWREN_FUNC_GPIO43                      2

#define EIC7700X_PAD_I2C0_SCL_FUNC_I2C0_SCL                      0
#define EIC7700X_PAD_I2C0_SCL_FUNC_GPIO44                        2

#define EIC7700X_PAD_I2C0_SDA_FUNC_I2C0_SDA                      0
#define EIC7700X_PAD_I2C0_SDA_FUNC_GPIO45                        2

#define EIC7700X_PAD_I2C1_SCL_FUNC_I2C1_SCL                      0
#define EIC7700X_PAD_I2C1_SCL_FUNC_GPIO46                        2

#define EIC7700X_PAD_I2C1_SDA_FUNC_I2C1_SDA                      0
#define EIC7700X_PAD_I2C1_SDA_FUNC_GPIO47                        2

#define EIC7700X_PAD_I2C2_SCL_FUNC_I2C2_SCL                      0
#define EIC7700X_PAD_I2C2_SCL_FUNC_GPIO48                        2

#define EIC7700X_PAD_I2C2_SDA_FUNC_I2C2_SDA                      0
#define EIC7700X_PAD_I2C2_SDA_FUNC_GPIO49                        2

#define EIC7700X_PAD_I2C3_SCL_FUNC_I2C3_SCL                      0
#define EIC7700X_PAD_I2C3_SCL_FUNC_GPIO50                        2

#define EIC7700X_PAD_I2C3_SDA_FUNC_I2C3_SDA                      0
#define EIC7700X_PAD_I2C3_SDA_FUNC_GPIO51                        2

#define EIC7700X_PAD_I2C4_SCL_FUNC_I2C4_SCL                      0
#define EIC7700X_PAD_I2C4_SCL_FUNC_GPIO52                        2

#define EIC7700X_PAD_I2C4_SDA_FUNC_I2C4_SDA                      0
#define EIC7700X_PAD_I2C4_SDA_FUNC_GPIO53                        2

#define EIC7700X_PAD_I2C5_SCL_FUNC_I2C5_SCL                      0
#define EIC7700X_PAD_I2C5_SCL_FUNC_GPIO54                        2

#define EIC7700X_PAD_I2C5_SDA_FUNC_I2C5_SDA                      0
#define EIC7700X_PAD_I2C5_SDA_FUNC_GPIO55                        2

#define EIC7700X_PAD_UART0_TX_FUNC_UART0_TX                      0
#define EIC7700X_PAD_UART0_TX_FUNC_GPIO56                        2

#define EIC7700X_PAD_UART0_RX_FUNC_UART0_RX                      0
#define EIC7700X_PAD_UART0_RX_FUNC_GPIO57                        2

#define EIC7700X_PAD_UART1_TX_FUNC_UART1_TX                      0
#define EIC7700X_PAD_UART1_TX_FUNC_GPIO58                        2

#define EIC7700X_PAD_UART1_RX_FUNC_UART1_RX                      0
#define EIC7700X_PAD_UART1_RX_FUNC_GPIO59                        2

#define EIC7700X_PAD_UART1_CTS_FUNC_UART1_CTS                    0
#define EIC7700X_PAD_UART1_CTS_FUNC_I2C6_SCL                     1
#define EIC7700X_PAD_UART1_CTS_FUNC_GPIO60                       2

#define EIC7700X_PAD_UART1_RTS_FUNC_UART1_RTS                    0
#define EIC7700X_PAD_UART1_RTS_FUNC_I2C6_SDA                     1
#define EIC7700X_PAD_UART1_RTS_FUNC_GPIO61                       2

#define EIC7700X_PAD_UART2_TX_FUNC_UART2_TX                      0
#define EIC7700X_PAD_UART2_TX_FUNC_I2C7_SCL                      1
#define EIC7700X_PAD_UART2_TX_FUNC_GPIO62                        2

#define EIC7700X_PAD_UART2_RX_FUNC_UART2_RX                      0
#define EIC7700X_PAD_UART2_RX_FUNC_I2C7_SDA                      1
#define EIC7700X_PAD_UART2_RX_FUNC_GPIO63                        2

#define EIC7700X_PAD_JTAG2_TCK_FUNC_JTAG2_TCK                    0
#define EIC7700X_PAD_JTAG2_TCK_FUNC_GPIO64                       2

#define EIC7700X_PAD_JTAG2_TMS_FUNC_JTAG2_TMS                    0
#define EIC7700X_PAD_JTAG2_TMS_FUNC_GPIO65                       2

#define EIC7700X_PAD_JTAG2_TDI_FUNC_JTAG2_TDI                    0
#define EIC7700X_PAD_JTAG2_TDI_FUNC_GPIO66                       2

#define EIC7700X_PAD_JTAG2_TDO_FUNC_JTAG2_TDO                    0
#define EIC7700X_PAD_JTAG2_TDO_FUNC_GPIO67                       2

#define EIC7700X_PAD_FAN_PWM_FUNC_FAN_PWM                        0
#define EIC7700X_PAD_FAN_PWM_FUNC_GPIO68                         2

#define EIC7700X_PAD_FAN_TACH_FUNC_FAN_TACH                      0
#define EIC7700X_PAD_FAN_TACH_FUNC_GPIO69                        2

/* MIPI CSI camera sync and clock */

#define EIC7700X_PAD_MIPI_CSI0_XVS_FUNC_MIPI_CSI0_XVS            0
#define EIC7700X_PAD_MIPI_CSI0_XVS_FUNC_GPIO70                   2

#define EIC7700X_PAD_MIPI_CSI0_XHS_FUNC_MIPI_CSI0_XHS            0
#define EIC7700X_PAD_MIPI_CSI0_XHS_FUNC_GPIO71                   2

#define EIC7700X_PAD_MIPI_CSI0_MCLK_FUNC_MIPI_CSI0_MCLK          0
#define EIC7700X_PAD_MIPI_CSI0_MCLK_FUNC_GPIO72                  2

#define EIC7700X_PAD_MIPI_CSI1_XVS_FUNC_MIPI_CSI1_XVS            0
#define EIC7700X_PAD_MIPI_CSI1_XVS_FUNC_GPIO73                   2

#define EIC7700X_PAD_MIPI_CSI1_XHS_FUNC_MIPI_CSI1_XHS            0
#define EIC7700X_PAD_MIPI_CSI1_XHS_FUNC_GPIO74                   2

#define EIC7700X_PAD_MIPI_CSI1_MCLK_FUNC_MIPI_CSI1_MCLK          0
#define EIC7700X_PAD_MIPI_CSI1_MCLK_FUNC_GPIO75                  2

#define EIC7700X_PAD_MIPI_CSI2_XVS_FUNC_MIPI_CSI2_XVS            0
#define EIC7700X_PAD_MIPI_CSI2_XVS_FUNC_GPIO76                   2

#define EIC7700X_PAD_MIPI_CSI2_XHS_FUNC_MIPI_CSI2_XHS            0
#define EIC7700X_PAD_MIPI_CSI2_XHS_FUNC_GPIO77                   2

#define EIC7700X_PAD_MIPI_CSI2_MCLK_FUNC_MIPI_CSI2_MCLK          0
#define EIC7700X_PAD_MIPI_CSI2_MCLK_FUNC_GPIO78                  2

#define EIC7700X_PAD_MIPI_CSI3_XVS_FUNC_MIPI_CSI3_XVS            0
#define EIC7700X_PAD_MIPI_CSI3_XVS_FUNC_GPIO79                   2

#define EIC7700X_PAD_MIPI_CSI3_XHS_FUNC_MIPI_CSI3_XHS            0
#define EIC7700X_PAD_MIPI_CSI3_XHS_FUNC_GPIO80                   2

#define EIC7700X_PAD_MIPI_CSI3_MCLK_FUNC_MIPI_CSI3_MCLK          0
#define EIC7700X_PAD_MIPI_CSI3_MCLK_FUNC_GPIO81                  2

#define EIC7700X_PAD_MIPI_CSI4_XVS_FUNC_MIPI_CSI4_XVS            0
#define EIC7700X_PAD_MIPI_CSI4_XVS_FUNC_GPIO82                   2

#define EIC7700X_PAD_MIPI_CSI4_XHS_FUNC_MIPI_CSI4_XHS            0
#define EIC7700X_PAD_MIPI_CSI4_XHS_FUNC_GPIO83                   2

#define EIC7700X_PAD_MIPI_CSI4_MCLK_FUNC_MIPI_CSI4_MCLK          0
#define EIC7700X_PAD_MIPI_CSI4_MCLK_FUNC_GPIO84                  2

#define EIC7700X_PAD_MIPI_CSI5_XVS_FUNC_MIPI_CSI5_XVS            0
#define EIC7700X_PAD_MIPI_CSI5_XVS_FUNC_GPIO85                   2

#define EIC7700X_PAD_MIPI_CSI5_XHS_FUNC_MIPI_CSI5_XHS            0
#define EIC7700X_PAD_MIPI_CSI5_XHS_FUNC_GPIO86                   2

#define EIC7700X_PAD_MIPI_CSI5_MCLK_FUNC_MIPI_CSI5_MCLK          0
#define EIC7700X_PAD_MIPI_CSI5_MCLK_FUNC_GPIO87                  2

/* SPI3, SPI0, I2C 10 and 11, boot select and the last GPIO */

#define EIC7700X_PAD_SPI3_CS_N_FUNC_SPI3_CS_N                    0
#define EIC7700X_PAD_SPI3_CS_N_FUNC_GPIO88                       2

#define EIC7700X_PAD_SPI3_CLK_FUNC_SPI3_CLK                      0
#define EIC7700X_PAD_SPI3_CLK_FUNC_GPIO89                        2

#define EIC7700X_PAD_SPI3_DI_FUNC_SPI3_DI                        0
#define EIC7700X_PAD_SPI3_DI_FUNC_GPIO90                         2

#define EIC7700X_PAD_SPI3_DO_FUNC_SPI3_DO                        0
#define EIC7700X_PAD_SPI3_DO_FUNC_GPIO91                         2

#define EIC7700X_PAD_GPIO92_FUNC_I2C8_SCL                        0
#define EIC7700X_PAD_GPIO92_FUNC_MIPI_CSI_XTRIG0                 1
#define EIC7700X_PAD_GPIO92_FUNC_GPIO92                          2
#define EIC7700X_PAD_GPIO92_FUNC_UART3_TX                        3

#define EIC7700X_PAD_GPIO93_FUNC_I2C8_SDA                        0
#define EIC7700X_PAD_GPIO93_FUNC_MIPI_CSI_XTRIG1                 1
#define EIC7700X_PAD_GPIO93_FUNC_GPIO93                          2
#define EIC7700X_PAD_GPIO93_FUNC_UART3_RX                        3

#define EIC7700X_PAD_S_MODE_FUNC_S_MODE                          0
#define EIC7700X_PAD_S_MODE_FUNC_GPIO94                          2

#define EIC7700X_PAD_GPIO95_FUNC_LPDDR_REFCLK_SEL                0
#define EIC7700X_PAD_GPIO95_FUNC_GPIO95                          2

#define EIC7700X_PAD_SPI0_CS_N_FUNC_SPI0_CS_N                    0
#define EIC7700X_PAD_SPI0_CS_N_FUNC_GPIO96                       2

#define EIC7700X_PAD_SPI0_CLK_FUNC_SPI0_CLK                      0
#define EIC7700X_PAD_SPI0_CLK_FUNC_GPIO97                        2

#define EIC7700X_PAD_SPI0_D0_FUNC_SPI0_D0                        0
#define EIC7700X_PAD_SPI0_D0_FUNC_GPIO98                         2

#define EIC7700X_PAD_SPI0_D1_FUNC_SPI0_D1                        0
#define EIC7700X_PAD_SPI0_D1_FUNC_GPIO99                         2

#define EIC7700X_PAD_SPI0_D2_FUNC_SPI0_D2                        0
#define EIC7700X_PAD_SPI0_D2_FUNC_GPIO100                        2

#define EIC7700X_PAD_SPI0_D3_FUNC_SPI0_D3                        0
#define EIC7700X_PAD_SPI0_D3_FUNC_GPIO101                        2

#define EIC7700X_PAD_I2C10_SCL_FUNC_I2C10_SCL                    0
#define EIC7700X_PAD_I2C10_SCL_FUNC_GPIO102                      2

#define EIC7700X_PAD_I2C10_SDA_FUNC_I2C10_SDA                    0
#define EIC7700X_PAD_I2C10_SDA_FUNC_GPIO103                      2

#define EIC7700X_PAD_I2C11_SCL_FUNC_I2C11_SCL                    0
#define EIC7700X_PAD_I2C11_SCL_FUNC_GPIO104                      2

#define EIC7700X_PAD_I2C11_SDA_FUNC_I2C11_SDA                    0
#define EIC7700X_PAD_I2C11_SDA_FUNC_GPIO105                      2

#define EIC7700X_PAD_GPIO106_FUNC_GPIO106                        0

#define EIC7700X_PAD_BOOT_SEL0_FUNC_BOOT_SEL0                    0
#define EIC7700X_PAD_BOOT_SEL0_FUNC_GPIO107                      2

#define EIC7700X_PAD_BOOT_SEL1_FUNC_BOOT_SEL1                    0
#define EIC7700X_PAD_BOOT_SEL1_FUNC_GPIO108                      2

#define EIC7700X_PAD_BOOT_SEL2_FUNC_BOOT_SEL2                    0
#define EIC7700X_PAD_BOOT_SEL2_FUNC_GPIO109                      2

#define EIC7700X_PAD_BOOT_SEL3_FUNC_BOOT_SEL3                    0
#define EIC7700X_PAD_BOOT_SEL3_FUNC_GPIO110                      2

#define EIC7700X_PAD_GPIO111_FUNC_GPIO111                        0
#define EIC7700X_PAD_LPDDR_REF_CLK_FUNC_LPDDR_REF_CLK            0

#endif /* __ARCH_RISCV_INCLUDE_EIC7700X_EIC7700X_PINCTRL_H */
