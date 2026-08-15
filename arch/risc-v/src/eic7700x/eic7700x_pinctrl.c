/****************************************************************************
 * arch/risc-v/src/eic7700x/eic7700x_pinctrl.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdio.h>
#include <inttypes.h>
#include <assert.h>
#include <sys/param.h>

#include <nuttx/debug.h>

#include "riscv_internal.h"
#include "eic7700x_pinctrl.h"
#include "hardware/eic7700x_pinctrl.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* One table row per pad, in register order, so a row's position in the
 * array is the pad id and the offset in the trailing comment is redundant
 * by construction.  There is a macro per field layout rather than a shape
 * argument, so that a row cannot silently claim a layout its fields do not
 * match.
 */

#define PADGEN(w, d, fm, gf, fl) \
  { \
    .wmask = (w), .shape = EIC7700X_PADSHAPE_GENERAL, .dflt = (d), \
    .funcmask = (fm), .gpiofunc = (gf), .flags = (fl) \
  }

#define PADRGM(w, d, fm, gf, fl) \
  { \
    .wmask = (w), .shape = EIC7700X_PADSHAPE_RGMII, .dflt = (d), \
    .funcmask = (fm), .gpiofunc = (gf), .flags = (fl) \
  }

#define PADOSC(w, d, fm, gf, fl) \
  { \
    .wmask = (w), .shape = EIC7700X_PADSHAPE_OSC, .dflt = (d), \
    .funcmask = (fm), .gpiofunc = (gf), .flags = (fl) \
  }

#define PADMSEL(w, d, fm, gf, fl) \
  { \
    .wmask = (w), .shape = EIC7700X_PADSHAPE_MODESEL, .dflt = (d), \
    .funcmask = (fm), .gpiofunc = (gf), .flags = (fl) \
  }

/* Shorthands so a table row fits a line. */

#define NOGP  EIC7700X_PAD_NOGPIO
#define LOCK  EIC7700X_PAD_LOCKED

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* The reset values the manual gives, one per field, reassembled into
 * words.  Fifteen cover all 166 pads, so the table above stores an index.
 */

const uint32_t g_eic7700x_pad_defaults[EIC7700X_PAD_NDEFAULTS] =
{
  0x00000000,  /*  0 */
  0x00000001,  /*  1 */
  0x00000008,  /*  2 */
  0x00000009,  /*  3 */
  0x0000000a,  /*  4 */
  0x0000000b,  /*  5 */
  0x0000000d,  /*  6 */
  0x00000010,  /*  7 */
  0x00000011,  /*  8 */
  0x00000083,  /*  9 */
  0x00000085,  /* 10 */
  0x00000089,  /* 11 */
  0x0000008b,  /* 12 */
  0x000000c8,  /* 13 */
  0x00000301,  /* 14 */
};

/* Every pad of the block, from TRM part 4 section 12.1.2.2 pages 371 to
 * 410.  The write mask is the manual's access column: a bit is set here
 * only where the manual says RW.
 *
 * Five pads have a zero write mask.  They are not omissions: CHIP_MODE and
 * POR_SEL report how the chip was strapped, KEY_RESET_N reports a button,
 * and two reserved pads report a state nothing can change.  Reading them
 * is useful and writing them is not possible, which is a distinction the
 * write path has to keep.
 */

const struct eic7700x_pad_s g_eic7700x_pads[EIC7700X_PAD_NPADS] =
{
  PADGEN(0x00000000, 10, 0x01, NOGP, 0),     /* 0x080 CHIP_MODE */
  PADGEN(0x000700ff, 11, 0x05, 2   , 0),     /* 0x084 MODE_SET0 */
  PADGEN(0x000700ff, 11, 0x05, 2   , 0),     /* 0x088 MODE_SET1 */
  PADGEN(0x000700ff, 11, 0x05, 2   , 0),     /* 0x08c MODE_SET2 */
  PADGEN(0x000700ff, 11, 0x05, 2   , 0),     /* 0x090 MODE_SET3 */
  PADOSC(0x000000ff, 13, 0x01, NOGP, 0),     /* 0x094 XIN */
  PADOSC(0x000000ff, 13, 0x01, NOGP, 0),     /* 0x098 RESERVED0 */
  PADGEN(0x000000fe,  4, 0x01, NOGP, 0),     /* 0x09c RST_OUT_N */
  PADGEN(0x00000000,  9, 0x01, NOGP, 0),     /* 0x0a0 KEY_RESET_N */
  PADGEN(0x00000000,  9, 0x01, NOGP, 0),     /* 0x0a4 RESERVED1 */
  PADGEN(0x00000000,  9, 0x01, NOGP, 0),     /* 0x0a8 RESERVED2 */
  PADGEN(0x00000078,  4, 0x01, NOGP, 0),     /* 0x0ac RESERVED3 */
  PADGEN(0x000000ff,  2, 0x01, 0   , 0),     /* 0x0b0 GPIO0 */
  PADGEN(0x00000000,  1, 0x01, NOGP, 0),     /* 0x0b4 POR_SEL */
  PADGEN(0x000700ff,  6, 0x07, 2   , 0),     /* 0x0b8 JTAG0_TCK */
  PADGEN(0x000700ff,  5, 0x07, 2   , 0),     /* 0x0bc JTAG0_TMS */
  PADGEN(0x000700ff,  5, 0x07, 2   , 0),     /* 0x0c0 JTAG0_TDI */
  PADGEN(0x000700ff,  2, 0x07, 2   , 0),     /* 0x0c4 JTAG0_TDO */
  PADGEN(0x000700ff,  2, 0x01, NOGP, 0),     /* 0x0c8 JTAG0_TRST */
  PADGEN(0x000700ff,  2, 0x05, 2   , 0),     /* 0x0cc SPI2_CS0_N */
  PADGEN(0x000700ff,  6, 0x05, 2   , 0),     /* 0x0d0 JTAG1_TCK */
  PADGEN(0x000700ff,  5, 0x05, 2   , 0),     /* 0x0d4 JTAG1_TMS */
  PADGEN(0x000700ff,  5, 0x05, 2   , 0),     /* 0x0d8 JTAG1_TDI */
  PADGEN(0x000700ff,  2, 0x05, 2   , 0),     /* 0x0dc JTAG1_TDO */
  PADGEN(0x000000ff,  2, 0x01, NOGP, 0),     /* 0x0e0 JTAG1_TRST */
  PADGEN(0x000700ff,  2, 0x05, 2   , 0),     /* 0x0e4 SPI2_CS1_N */
  PADGEN(0x000000ff,  3, 0x01, NOGP, 0),     /* 0x0e8 PCIE_CLKREQ_N */
  PADGEN(0x000000ff,  5, 0x01, NOGP, 0),     /* 0x0ec PCIE_WAKE_N */
  PADGEN(0x000000ff,  2, 0x01, NOGP, 0),     /* 0x0f0 PCIE_PERST_N */
  PADGEN(0x000000ff,  3, 0x01, NOGP, 0),     /* 0x0f4 HDMI_SCL */
  PADGEN(0x000000ff,  3, 0x01, NOGP, 0),     /* 0x0f8 HDMI_SDA */
  PADGEN(0x000000ff,  4, 0x01, NOGP, 0),     /* 0x0fc HDMI_CEC */
  PADGEN(0x000700ff, 12, 0x05, 2   , 0),     /* 0x100 JTAG2_TRST */
  PADGEN(0x000700ff,  8, 0x01, NOGP, 0),     /* 0x104 RGMII0_CLK_125 */
  PADGEN(0x000700ff,  7, 0x01, NOGP, 0),     /* 0x108 RGMII0_TXEN */
  PADGEN(0x000700ff,  7, 0x01, NOGP, 0),     /* 0x10c RGMII0_TXCLK */
  PADGEN(0x000700ff,  7, 0x01, NOGP, 0),     /* 0x110 RGMII0_TXD0 */
  PADGEN(0x000700ff,  7, 0x01, NOGP, 0),     /* 0x114 RGMII0_TXD1 */
  PADGEN(0x000700ff,  7, 0x01, NOGP, 0),     /* 0x118 RGMII0_TXD2 */
  PADGEN(0x000700ff,  7, 0x01, NOGP, 0),     /* 0x11c RGMII0_TXD3 */
  PADGEN(0x000700ff,  2, 0x05, 2   , 0),     /* 0x120 I2S0_BCLK */
  PADGEN(0x000700ff,  2, 0x05, 2   , 0),     /* 0x124 I2S0_WCLK */
  PADGEN(0x000700ff,  3, 0x05, 2   , 0),     /* 0x128 I2S0_SDI */
  PADGEN(0x000700ff,  2, 0x05, 2   , 0),     /* 0x12c I2S0_SDO */
  PADGEN(0x000700ff,  2, 0x05, 2   , 0),     /* 0x130 I2S_MCLK */
  PADGEN(0x000700ff,  1, 0x01, NOGP, 0),     /* 0x134 RGMII0_RXCLK */
  PADGEN(0x000700ff,  1, 0x01, NOGP, 0),     /* 0x138 RGMII0_RXDV */
  PADGEN(0x000700ff,  1, 0x01, NOGP, 0),     /* 0x13c RGMII0_RXD0 */
  PADGEN(0x000700ff,  1, 0x01, NOGP, 0),     /* 0x140 RGMII0_RXD1 */
  PADGEN(0x000700ff,  1, 0x01, NOGP, 0),     /* 0x144 RGMII0_RXD2 */
  PADGEN(0x000700ff,  1, 0x01, NOGP, 0),     /* 0x148 RGMII0_RXD3 */
  PADGEN(0x000700ff,  2, 0x05, 2   , 0),     /* 0x14c I2S2_BCLK */
  PADGEN(0x000700ff,  2, 0x05, 2   , 0),     /* 0x150 I2S2_WCLK */
  PADGEN(0x000700ff,  3, 0x05, 2   , 0),     /* 0x154 I2S2_SDI */
  PADGEN(0x000700ff,  2, 0x05, 2   , 0),     /* 0x158 I2S2_SDO */
  PADGEN(0x000700ff,  2, 0x03, 0   , 0),     /* 0x15c GPIO27 */
  PADGEN(0x000000ff,  2, 0x01, 0   , 0),     /* 0x160 GPIO28 */
  PADGEN(0x000700ff,  3, 0x07, 2   , 0),     /* 0x164 GPIO29 */
  PADGEN(0x000000ff,  7, 0x01, NOGP, 0),     /* 0x168 RGMII0_MDC */
  PADGEN(0x000000ff,  8, 0x01, NOGP, 0),     /* 0x16c RGMII0_MDIO */
  PADGEN(0x000000ff,  8, 0x01, NOGP, 0),     /* 0x170 RGMII0_INTB */
  PADGEN(0x000000ff,  8, 0x01, NOGP, 0),     /* 0x174 RGMII1_CLK_125 */
  PADGEN(0x000000ff,  7, 0x01, NOGP, 0),     /* 0x178 RGMII1_TXEN */
  PADGEN(0x000000ff,  7, 0x01, NOGP, 0),     /* 0x17c RGMII1_TXCLK */
  PADGEN(0x000000ff,  7, 0x01, NOGP, 0),     /* 0x180 RGMII1_TXD0 */
  PADGEN(0x000000ff,  7, 0x01, NOGP, 0),     /* 0x184 RGMII1_TXD1 */
  PADGEN(0x000000ff,  7, 0x01, NOGP, 0),     /* 0x188 RGMII1_TXD2 */
  PADGEN(0x000000ff,  7, 0x01, NOGP, 0),     /* 0x18c RGMII1_TXD3 */
  PADGEN(0x000700ff,  2, 0x05, 2   , 0),     /* 0x190 I2S1_BCLK */
  PADGEN(0x000700ff,  2, 0x05, 2   , 0),     /* 0x194 I2S1_WCLK */
  PADGEN(0x000700ff,  3, 0x05, 2   , 0),     /* 0x198 I2S1_SDI */
  PADGEN(0x000700ff,  2, 0x05, 2   , 0),     /* 0x19c I2S1_SDO */
  PADGEN(0x000700ff,  3, 0x07, 2   , 0),     /* 0x1a0 GPIO34 */
  PADGEN(0x000000ff,  1, 0x01, NOGP, 0),     /* 0x1a4 RGMII1_RXCLK */
  PADGEN(0x000000ff,  1, 0x01, NOGP, 0),     /* 0x1a8 RGMII1_RXDV */
  PADGEN(0x000000ff,  1, 0x01, NOGP, 0),     /* 0x1ac RGMII1_RXD0 */
  PADGEN(0x000000ff,  1, 0x01, NOGP, 0),     /* 0x1b0 RGMII1_RXD1 */
  PADGEN(0x000000ff,  1, 0x01, NOGP, 0),     /* 0x1b4 RGMII1_RXD2 */
  PADGEN(0x000000ff,  1, 0x01, NOGP, 0),     /* 0x1b8 RGMII1_RXD3 */
  PADGEN(0x000700ff,  2, 0x05, 2   , 0),     /* 0x1bc SPI1_CS0_N */
  PADGEN(0x000700ff,  2, 0x05, 2   , 0),     /* 0x1c0 SPI1_CLK */
  PADGEN(0x000700ff,  2, 0x0f, 2   , 0),     /* 0x1c4 SPI1_D0 */
  PADGEN(0x000700ff,  3, 0x0f, 2   , 0),     /* 0x1c8 SPI1_D1 */
  PADGEN(0x000700ff,  3, 0x07, 2   , 0),     /* 0x1cc SPI1_D2 */
  PADGEN(0x000700ff,  3, 0x07, 2   , 0),     /* 0x1d0 SPI1_D3 */
  PADGEN(0x000700ff,  2, 0x07, 2   , 0),     /* 0x1d4 SPI1_CS1_N */
  PADGEN(0x000000ff,  7, 0x01, NOGP, 0),     /* 0x1d8 RGMII1_MDC */
  PADGEN(0x000000ff,  8, 0x01, NOGP, 0),     /* 0x1dc RGMII1_MDIO */
  PADGEN(0x000000ff,  8, 0x01, NOGP, 0),     /* 0x1e0 RGMII1_INTB */
  PADGEN(0x000700ff,  2, 0x05, 2   , 0),     /* 0x1e4 USB0_PWREN */
  PADGEN(0x000700ff,  2, 0x05, 2   , 0),     /* 0x1e8 USB1_PWREN */
  PADGEN(0x000700ff,  3, 0x05, 2   , 0),     /* 0x1ec I2C0_SCL */
  PADGEN(0x000700ff,  3, 0x05, 2   , 0),     /* 0x1f0 I2C0_SDA */
  PADGEN(0x000700ff,  3, 0x05, 2   , 0),     /* 0x1f4 I2C1_SCL */
  PADGEN(0x000700ff,  3, 0x05, 2   , 0),     /* 0x1f8 I2C1_SDA */
  PADGEN(0x000700ff,  3, 0x05, 2   , 0),     /* 0x1fc I2C2_SCL */
  PADGEN(0x000700ff,  3, 0x05, 2   , 0),     /* 0x200 I2C2_SDA */
  PADGEN(0x000700ff,  3, 0x05, 2   , 0),     /* 0x204 I2C3_SCL */
  PADGEN(0x000700ff,  3, 0x05, 2   , 0),     /* 0x208 I2C3_SDA */
  PADGEN(0x000700ff,  3, 0x05, 2   , 0),     /* 0x20c I2C4_SCL */
  PADGEN(0x000700ff,  3, 0x05, 2   , 0),     /* 0x210 I2C4_SDA */
  PADGEN(0x000700ff,  3, 0x05, 2   , 0),     /* 0x214 I2C5_SCL */
  PADGEN(0x000700ff,  3, 0x05, 2   , 0),     /* 0x218 I2C5_SDA */
  PADGEN(0x000700ff,  2, 0x05, 2   , 0),     /* 0x21c UART0_TX */
  PADGEN(0x000700ff,  3, 0x05, 2   , 0),     /* 0x220 UART0_RX */
  PADGEN(0x000700ff,  2, 0x05, 2   , 0),     /* 0x224 UART1_TX */
  PADGEN(0x000700ff,  3, 0x05, 2   , 0),     /* 0x228 UART1_RX */
  PADGEN(0x000700ff,  3, 0x07, 2   , 0),     /* 0x22c UART1_CTS */
  PADGEN(0x000700ff,  2, 0x07, 2   , 0),     /* 0x230 UART1_RTS */
  PADGEN(0x000700ff,  2, 0x07, 2   , 0),     /* 0x234 UART2_TX */
  PADGEN(0x000700ff,  3, 0x07, 2   , 0),     /* 0x238 UART2_RX */
  PADGEN(0x000700ff,  6, 0x05, 2   , 0),     /* 0x23c JTAG2_TCK */
  PADGEN(0x000700ff,  5, 0x05, 2   , 0),     /* 0x240 JTAG2_TMS */
  PADGEN(0x000700ff,  5, 0x05, 2   , 0),     /* 0x244 JTAG2_TDI */
  PADGEN(0x000700ff,  2, 0x05, 2   , 0),     /* 0x248 JTAG2_TDO */
  PADGEN(0x000700ff,  2, 0x05, 2   , 0),     /* 0x24c FAN_PWM */
  PADGEN(0x000700ff,  3, 0x05, 2   , 0),     /* 0x250 FAN_TACH */
  PADGEN(0x000700ff,  2, 0x05, 2   , 0),     /* 0x254 MIPI_CSI0_XVS */
  PADGEN(0x000700ff,  2, 0x05, 2   , 0),     /* 0x258 MIPI_CSI0_XHS */
  PADGEN(0x000700ff,  2, 0x05, 2   , 0),     /* 0x25c MIPI_CSI0_MCLK */
  PADGEN(0x000700ff,  2, 0x05, 2   , 0),     /* 0x260 MIPI_CSI1_XVS */
  PADGEN(0x000700ff,  2, 0x05, 2   , 0),     /* 0x264 MIPI_CSI1_XHS */
  PADGEN(0x000700ff,  2, 0x05, 2   , 0),     /* 0x268 MIPI_CSI1_MCLK */
  PADGEN(0x000700ff,  2, 0x05, 2   , 0),     /* 0x26c MIPI_CSI2_XVS */
  PADGEN(0x000700ff,  2, 0x05, 2   , 0),     /* 0x270 MIPI_CSI2_XHS */
  PADGEN(0x000700ff,  2, 0x05, 2   , 0),     /* 0x274 MIPI_CSI2_MCLK */
  PADGEN(0x000700ff,  2, 0x05, 2   , 0),     /* 0x278 MIPI_CSI3_XVS */
  PADGEN(0x000700ff,  2, 0x05, 2   , 0),     /* 0x27c MIPI_CSI3_XHS */
  PADGEN(0x000700ff,  2, 0x05, 2   , 0),     /* 0x280 MIPI_CSI3_MCLK */
  PADGEN(0x000700ff,  2, 0x05, 2   , 0),     /* 0x284 MIPI_CSI4_XVS */
  PADGEN(0x000700ff,  2, 0x05, 2   , 0),     /* 0x288 MIPI_CSI4_XHS */
  PADGEN(0x000700ff,  2, 0x05, 2   , 0),     /* 0x28c MIPI_CSI4_MCLK */
  PADGEN(0x000700ff,  2, 0x05, 2   , 0),     /* 0x290 MIPI_CSI5_XVS */
  PADGEN(0x000700ff,  2, 0x05, 2   , 0),     /* 0x294 MIPI_CSI5_XHS */
  PADGEN(0x000700ff,  2, 0x05, 2   , 0),     /* 0x298 MIPI_CSI5_MCLK */
  PADGEN(0x000700ff,  3, 0x05, 2   , 0),     /* 0x29c SPI3_CS_N */
  PADGEN(0x000700ff,  3, 0x05, 2   , 0),     /* 0x2a0 SPI3_CLK */
  PADGEN(0x000700ff,  3, 0x05, 2   , 0),     /* 0x2a4 SPI3_DI */
  PADGEN(0x000700ff,  2, 0x05, 2   , 0),     /* 0x2a8 SPI3_DO */
  PADGEN(0x000700ff,  3, 0x0f, 2   , 0),     /* 0x2ac GPIO92 */
  PADGEN(0x000700ff,  3, 0x0f, 2   , 0),     /* 0x2b0 GPIO93 */
  PADGEN(0x000700ff,  3, 0x05, 2   , 0),     /* 0x2b4 S_MODE */
  PADGEN(0x000700ff,  3, 0x05, 2   , 0),     /* 0x2b8 GPIO95 */
  PADGEN(0x000700ff,  4, 0x05, 2   , 0),     /* 0x2bc SPI0_CS_N */
  PADGEN(0x000700ff,  2, 0x05, 2   , 0),     /* 0x2c0 SPI0_CLK */
  PADGEN(0x000700ff,  2, 0x05, 2   , 0),     /* 0x2c4 SPI0_D0 */
  PADGEN(0x000700ff,  3, 0x05, 2   , 0),     /* 0x2c8 SPI0_D1 */
  PADGEN(0x000700ff,  3, 0x05, 2   , 0),     /* 0x2cc SPI0_D2 */
  PADGEN(0x000700ff,  3, 0x05, 2   , 0),     /* 0x2d0 SPI0_D3 */
  PADGEN(0x000700ff,  3, 0x05, 2   , 0),     /* 0x2d4 I2C10_SCL */
  PADGEN(0x000700ff,  3, 0x05, 2   , 0),     /* 0x2d8 I2C10_SDA */
  PADGEN(0x000700ff,  3, 0x05, 2   , 0),     /* 0x2dc I2C11_SCL */
  PADGEN(0x000700ff,  3, 0x05, 2   , 0),     /* 0x2e0 I2C11_SDA */
  PADGEN(0x000000ff,  2, 0x01, 0   , 0),     /* 0x2e4 GPIO106 */
  PADGEN(0x000700ff,  3, 0x05, 2   , 0),     /* 0x2e8 BOOT_SEL0 */
  PADGEN(0x000700ff,  3, 0x05, 2   , 0),     /* 0x2ec BOOT_SEL1 */
  PADGEN(0x000700ff,  3, 0x05, 2   , 0),     /* 0x2f0 BOOT_SEL2 */
  PADGEN(0x000700ff,  3, 0x05, 2   , 0),     /* 0x2f4 BOOT_SEL3 */
  PADGEN(0x000000ff,  2, 0x01, 0   , 0),     /* 0x2f8 GPIO111 */
  PADGEN(0x000000ff,  1, 0x01, NOGP, 0),     /* 0x2fc RESERVED4 */
  PADGEN(0x000000ff,  2, 0x01, NOGP, 0),     /* 0x300 RESERVED5 */
  PADGEN(0x000000ff,  9, 0x01, NOGP, 0),     /* 0x304 RESERVED6 */
  PADGEN(0x000000ff,  4, 0x01, NOGP, 0),     /* 0x308 RESERVED7 */
  PADRGM(0x000003ff, 14, 0x01, NOGP, LOCK),  /* 0x30c LPDDR_REF_CLK */
  PADMSEL(0x00000003,  0, 0x01, NOGP, LOCK), /* 0x310 ADDR_RGMII0_SEL_MODE */
  PADMSEL(0x00000003,  0, 0x01, NOGP, LOCK), /* 0x314 ADDR_RGMII1_SEL_MODE */
};

static_assert(nitems(g_eic7700x_pads) == EIC7700X_PAD_NPADS,
              "pad table length does not match the pad count");
static_assert(nitems(g_eic7700x_pad_defaults) == EIC7700X_PAD_NDEFAULTS,
              "reset default table length does not match the count");
static_assert(EIC7700X_PAD_OFFSET(EIC7700X_PAD_NPADS - 1) == 0x0314,
              "last pad does not land on the last register of the block");

/* Which pad carries each GPIO line, for the GPIO driver to come.
 *
 * GPIO 5 and 11 are balls that part 1 table 2-4 documents but that the
 * register detail description gives no pad register for, and none of the
 * eight reserved registers can be attributed to them without guessing.
 * They are marked NOPAD rather than guessed at.  The consequence is real:
 * both lines exist in port A and are interruptible, but their input
 * enable cannot be reached, so neither can be configured as an input.
 */

#define NOPAD EIC7700X_PAD_NOPAD

const uint8_t g_eic7700x_pad_bygpio[EIC7700X_PAD_NGPIOS] =
{
  12,    14,    15,    16,    17,    NOPAD, 19,    20,    /*   0 -   7 */
  21,    22,    23,    NOPAD, 25,    1,     2,     3,     /*   8 -  15 */
  4,     32,    40,    41,    42,    43,    44,    51,    /*  16 -  23 */
  52,    53,    54,    55,    56,    57,    68,    69,    /*  24 -  31 */
  70,    71,    72,    79,    80,    81,    82,    83,    /*  32 -  39 */
  84,    85,    89,    90,    91,    92,    93,    94,    /*  40 -  47 */
  95,    96,    97,    98,    99,    100,   101,   102,   /*  48 -  55 */
  103,   104,   105,   106,   107,   108,   109,   110,   /*  56 -  63 */
  111,   112,   113,   114,   115,   116,   117,   118,   /*  64 -  71 */
  119,   120,   121,   122,   123,   124,   125,   126,   /*  72 -  79 */
  127,   128,   129,   130,   131,   132,   133,   134,   /*  80 -  87 */
  135,   136,   137,   138,   139,   140,   141,   142,   /*  88 -  95 */
  143,   144,   145,   146,   147,   148,   149,   150,   /*  96 - 103 */
  151,   152,   153,   154,   155,   156,   157,   158,   /* 104 - 111 */
};

static_assert(nitems(g_eic7700x_pad_bygpio) == EIC7700X_PAD_NGPIOS,
              "GPIO reverse map length does not match the GPIO count");

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The device handed to the pinctrl framework.  struct pinctrl_dev_s holds
 * an operations pointer and nothing else, and pinctrl_register() stores
 * the address rather than copying, so this has to outlive registration.
 */

static struct pinctrl_dev_s g_eic7700x_pinctrl_dev =
{
  .ops = &g_eic7700x_pinctrl_ops
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: eic7700x_pinctrl_initialize
 *
 * Description:
 *   Bring up the pad multiplexing block.  This writes nothing to the
 *   hardware: a pad only moves when a driver asks it to.
 *
 * Returned Value:
 *   OK on success, or a negated errno on failure.
 *
 ****************************************************************************/

int eic7700x_pinctrl_initialize(void)
{
#ifdef CONFIG_DEBUG_ASSERTIONS
  unsigned int i;

  for (i = 0; i < nitems(g_eic7700x_pads); i++)
    {
      FAR const struct eic7700x_pad_s *pad = &g_eic7700x_pads[i];

      DEBUGASSERT(pad->dflt < EIC7700X_PAD_NDEFAULTS);
      DEBUGASSERT(pad->shape <= EIC7700X_PADSHAPE_MODESEL);

      /* A reset value with bits outside the register is a transcription
       * error, not a property of the hardware.  The general layout uses
       * bits 0 to 18, and the other three use bits 0 to 9.
       */

      DEBUGASSERT((g_eic7700x_pad_defaults[pad->dflt] &
                   ~(pad->shape == EIC7700X_PADSHAPE_GENERAL ?
                     0x0007ffff : 0x000003ff)) == 0);

      /* A pad that offers GPIO must offer it as one of its documented
       * functions, or select_gpio would hand out a value the write path
       * is about to refuse.
       */

      DEBUGASSERT(pad->gpiofunc == EIC7700X_PAD_NOGPIO ||
                  (pad->funcmask & (1 << pad->gpiofunc)) != 0);

      /* Locking a pad the hardware already refuses to drive would say
       * nothing.  The three that are locked are all writable, which is
       * exactly why the lock has to exist.
       */

      DEBUGASSERT((pad->flags & EIC7700X_PAD_LOCKED) == 0 ||
                  pad->wmask != 0);
    }

  for (i = 0; i < nitems(g_eic7700x_pad_bygpio); i++)
    {
      DEBUGASSERT(g_eic7700x_pad_bygpio[i] == EIC7700X_PAD_NOPAD ||
                  g_eic7700x_pad_bygpio[i] < EIC7700X_PAD_NPADS);
    }
#endif

  /* Publish the block as /dev/pinctrl0.  The framework allocates nothing
   * and keeps no state of its own, so the device below is all there is and
   * a file static one is enough: there is one mux block on this SoC and it
   * has no per instance state.
   */

  return pinctrl_register(&g_eic7700x_pinctrl_dev, 0);
}

/****************************************************************************
 * Name: eic7700x_pinctrl_count
 *
 * Description:
 *   How many pads the block has, and through changed how many currently
 *   differ from their reset defaults, which after boot is the set of pads
 *   the boot loader and the drivers have configured.
 *
 ****************************************************************************/

unsigned int eic7700x_pinctrl_count(FAR unsigned int *changed)
{
  unsigned int ndiff = 0;
  unsigned int i;

  for (i = 0; i < nitems(g_eic7700x_pads); i++)
    {
      FAR const struct eic7700x_pad_s *pad = &g_eic7700x_pads[i];

      if (getreg32(EIC7700X_PINCTRL_PAD(i)) !=
          g_eic7700x_pad_defaults[pad->dflt])
        {
          ndiff++;
        }
    }

  if (changed != NULL)
    {
      *changed = ndiff;
    }

  return EIC7700X_PAD_NPADS;
}
