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
  .ops   = &g_eic7700x_pinctrl_ops,
  .npins = EIC7700X_PAD_NPADS
};

#ifdef HAVE_PINCTRL_TEXT

/* Every pad's name and the name of each function select the manual
 * documents, in select order, for /proc/pinctrl and PINCTRLC_GETPAD.
 * The names are the manual's own; a pad's name describes its default
 * function, not its current one.
 */

static const struct pinctrl_padname_s
g_eic7700x_pad_names[EIC7700X_PAD_NPADS] =
{
  [EIC7700X_PAD_CHIP_MODE] =
    PINCTRL_PADNAME("CHIP_MODE", "CHIP_MODE"),
  [EIC7700X_PAD_MODE_SET0] =
    PINCTRL_PADNAME("MODE_SET0", "SDIO0_DETECT", NULL, "GPIO13"),
  [EIC7700X_PAD_MODE_SET1] =
    PINCTRL_PADNAME("MODE_SET1", "SDIO0_WRITE_PROT", NULL, "GPIO14"),
  [EIC7700X_PAD_MODE_SET2] =
    PINCTRL_PADNAME("MODE_SET2", "SDIO1_DETECT", NULL, "GPIO15"),
  [EIC7700X_PAD_MODE_SET3] =
    PINCTRL_PADNAME("MODE_SET3", "SDIO1_WRITE_PROT", NULL, "GPIO16"),
  [EIC7700X_PAD_XIN] =
    PINCTRL_PADNAME("XIN", "XIN_24M"),
  [EIC7700X_PAD_RESERVED0] =
    PINCTRL_PADNAME("RESERVED0", "XOUT_24M"),
  [EIC7700X_PAD_RST_OUT_N] =
    PINCTRL_PADNAME("RST_OUT_N", "RST_OUT_N"),
  [EIC7700X_PAD_KEY_RESET_N] =
    PINCTRL_PADNAME("KEY_RESET_N", "KEY_RESET_N"),
  [EIC7700X_PAD_RESERVED1] =
    PINCTRL_PADNAME("RESERVED1", NULL),
  [EIC7700X_PAD_RESERVED2] =
    PINCTRL_PADNAME("RESERVED2", NULL),
  [EIC7700X_PAD_RESERVED3] =
    PINCTRL_PADNAME("RESERVED3", NULL),
  [EIC7700X_PAD_GPIO0] =
    PINCTRL_PADNAME("GPIO0", "GPIO0"),
  [EIC7700X_PAD_POR_SEL] =
    PINCTRL_PADNAME("POR_SEL", "POR_SEL"),
  [EIC7700X_PAD_JTAG0_TCK] =
    PINCTRL_PADNAME("JTAG0_TCK", "JTAG0_TCK", "SPI2_CLK", "GPIO1"),
  [EIC7700X_PAD_JTAG0_TMS] =
    PINCTRL_PADNAME("JTAG0_TMS", "JTAG0_TMS", "SPI2_D0", "GPIO2"),
  [EIC7700X_PAD_JTAG0_TDI] =
    PINCTRL_PADNAME("JTAG0_TDI", "JTAG0_TDI", "SPI2_D1", "GPIO3"),
  [EIC7700X_PAD_JTAG0_TDO] =
    PINCTRL_PADNAME("JTAG0_TDO", "JTAG0_TDO", "SPI2_D2", "GPIO4"),
  [EIC7700X_PAD_JTAG0_TRST] =
    PINCTRL_PADNAME("JTAG0_TRST", NULL),
  [EIC7700X_PAD_SPI2_CS0_N] =
    PINCTRL_PADNAME("SPI2_CS0_N", "SPI2_CS0_N", NULL, "GPIO6"),
  [EIC7700X_PAD_JTAG1_TCK] =
    PINCTRL_PADNAME("JTAG1_TCK", "JTAG1_TCK", NULL, "GPIO7"),
  [EIC7700X_PAD_JTAG1_TMS] =
    PINCTRL_PADNAME("JTAG1_TMS", "JTAG1_TMS", NULL, "GPIO8"),
  [EIC7700X_PAD_JTAG1_TDI] =
    PINCTRL_PADNAME("JTAG1_TDI", "JTAG1_TDI", NULL, "GPIO9"),
  [EIC7700X_PAD_JTAG1_TDO] =
    PINCTRL_PADNAME("JTAG1_TDO", "JTAG1_TDO", NULL, "GPIO10"),
  [EIC7700X_PAD_JTAG1_TRST] =
    PINCTRL_PADNAME("JTAG1_TRST", NULL),
  [EIC7700X_PAD_SPI2_CS1_N] =
    PINCTRL_PADNAME("SPI2_CS1_N", "SPI2_CS1_N", NULL, "GPIO12"),
  [EIC7700X_PAD_PCIE_CLKREQ_N] =
    PINCTRL_PADNAME("PCIE_CLKREQ_N", "PCIE_CLKREQ_N"),
  [EIC7700X_PAD_PCIE_WAKE_N] =
    PINCTRL_PADNAME("PCIE_WAKE_N", "PCIE_WAKE_N"),
  [EIC7700X_PAD_PCIE_PERST_N] =
    PINCTRL_PADNAME("PCIE_PERST_N", "PCIE_PERST_N"),
  [EIC7700X_PAD_HDMI_SCL] =
    PINCTRL_PADNAME("HDMI_SCL", "HDMI_SCL"),
  [EIC7700X_PAD_HDMI_SDA] =
    PINCTRL_PADNAME("HDMI_SDA", "HDMI_SDA"),
  [EIC7700X_PAD_HDMI_CEC] =
    PINCTRL_PADNAME("HDMI_CEC", "HDMI_CEC"),
  [EIC7700X_PAD_JTAG2_TRST] =
    PINCTRL_PADNAME("JTAG2_TRST", "JTAG2_TRST", NULL, "GPIO17"),
  [EIC7700X_PAD_RGMII0_CLK_125] =
    PINCTRL_PADNAME("RGMII0_CLK_125", "RGMII0_CLK_125"),
  [EIC7700X_PAD_RGMII0_TXEN] =
    PINCTRL_PADNAME("RGMII0_TXEN", "RGMII0_TXEN"),
  [EIC7700X_PAD_RGMII0_TXCLK] =
    PINCTRL_PADNAME("RGMII0_TXCLK", "RGMII0_TXCLK"),
  [EIC7700X_PAD_RGMII0_TXD0] =
    PINCTRL_PADNAME("RGMII0_TXD0", "RGMII0_TXD0"),
  [EIC7700X_PAD_RGMII0_TXD1] =
    PINCTRL_PADNAME("RGMII0_TXD1", "RGMII0_TXD1"),
  [EIC7700X_PAD_RGMII0_TXD2] =
    PINCTRL_PADNAME("RGMII0_TXD2", "RGMII0_TXD2"),
  [EIC7700X_PAD_RGMII0_TXD3] =
    PINCTRL_PADNAME("RGMII0_TXD3", "RGMII0_TXD3"),
  [EIC7700X_PAD_I2S0_BCLK] =
    PINCTRL_PADNAME("I2S0_BCLK", "I2S0_BCLK", NULL, "GPIO18"),
  [EIC7700X_PAD_I2S0_WCLK] =
    PINCTRL_PADNAME("I2S0_WCLK", "I2S0_WCLK", NULL, "GPIO19"),
  [EIC7700X_PAD_I2S0_SDI] =
    PINCTRL_PADNAME("I2S0_SDI", "I2S0_SDI", NULL, "GPIO20"),
  [EIC7700X_PAD_I2S0_SDO] =
    PINCTRL_PADNAME("I2S0_SDO", "I2S0_SDO", NULL, "GPIO21"),
  [EIC7700X_PAD_I2S_MCLK] =
    PINCTRL_PADNAME("I2S_MCLK", "I2S_MCLK", NULL, "GPIO22"),
  [EIC7700X_PAD_RGMII0_RXCLK] =
    PINCTRL_PADNAME("RGMII0_RXCLK", "RGMII0_RXCLK"),
  [EIC7700X_PAD_RGMII0_RXDV] =
    PINCTRL_PADNAME("RGMII0_RXDV", "RGMII0_RXDV"),
  [EIC7700X_PAD_RGMII0_RXD0] =
    PINCTRL_PADNAME("RGMII0_RXD0", "RGMII0_RXD0"),
  [EIC7700X_PAD_RGMII0_RXD1] =
    PINCTRL_PADNAME("RGMII0_RXD1", "RGMII0_RXD1"),
  [EIC7700X_PAD_RGMII0_RXD2] =
    PINCTRL_PADNAME("RGMII0_RXD2", "RGMII0_RXD2"),
  [EIC7700X_PAD_RGMII0_RXD3] =
    PINCTRL_PADNAME("RGMII0_RXD3", "RGMII0_RXD3"),
  [EIC7700X_PAD_I2S2_BCLK] =
    PINCTRL_PADNAME("I2S2_BCLK", "I2S2_BCLK", NULL, "GPIO23"),
  [EIC7700X_PAD_I2S2_WCLK] =
    PINCTRL_PADNAME("I2S2_WCLK", "I2S2_WCLK", NULL, "GPIO24"),
  [EIC7700X_PAD_I2S2_SDI] =
    PINCTRL_PADNAME("I2S2_SDI", "I2S2_SDI", NULL, "GPIO25"),
  [EIC7700X_PAD_I2S2_SDO] =
    PINCTRL_PADNAME("I2S2_SDO", "I2S2_SDO", NULL, "GPIO26"),
  [EIC7700X_PAD_GPIO27] =
    PINCTRL_PADNAME("GPIO27", "GPIO27", "SATA_ACT_LED"),
  [EIC7700X_PAD_GPIO28] =
    PINCTRL_PADNAME("GPIO28", "GPIO28"),
  [EIC7700X_PAD_GPIO29] =
    PINCTRL_PADNAME("GPIO29", "POR_TIME_SEL0", "EMMC_LED_CONTROL", "GPIO29"),
  [EIC7700X_PAD_RGMII0_MDC] =
    PINCTRL_PADNAME("RGMII0_MDC", "RGMII0_MDC"),
  [EIC7700X_PAD_RGMII0_MDIO] =
    PINCTRL_PADNAME("RGMII0_MDIO", "RGMII0_MDIO"),
  [EIC7700X_PAD_RGMII0_INTB] =
    PINCTRL_PADNAME("RGMII0_INTB", "RGMII0_INTB"),
  [EIC7700X_PAD_RGMII1_CLK_125] =
    PINCTRL_PADNAME("RGMII1_CLK_125", "RGMII1_CLK_125"),
  [EIC7700X_PAD_RGMII1_TXEN] =
    PINCTRL_PADNAME("RGMII1_TXEN", "RGMII1_TXEN"),
  [EIC7700X_PAD_RGMII1_TXCLK] =
    PINCTRL_PADNAME("RGMII1_TXCLK", "RGMII1_TXCLK"),
  [EIC7700X_PAD_RGMII1_TXD0] =
    PINCTRL_PADNAME("RGMII1_TXD0", "RGMII1_TXD0"),
  [EIC7700X_PAD_RGMII1_TXD1] =
    PINCTRL_PADNAME("RGMII1_TXD1", "RGMII1_TXD1"),
  [EIC7700X_PAD_RGMII1_TXD2] =
    PINCTRL_PADNAME("RGMII1_TXD2", "RGMII1_TXD2"),
  [EIC7700X_PAD_RGMII1_TXD3] =
    PINCTRL_PADNAME("RGMII1_TXD3", "RGMII1_TXD3"),
  [EIC7700X_PAD_I2S1_BCLK] =
    PINCTRL_PADNAME("I2S1_BCLK", "I2S1_BCLK", NULL, "GPIO30"),
  [EIC7700X_PAD_I2S1_WCLK] =
    PINCTRL_PADNAME("I2S1_WCLK", "I2S1_WCLK", NULL, "GPIO31"),
  [EIC7700X_PAD_I2S1_SDI] =
    PINCTRL_PADNAME("I2S1_SDI", "I2S1_SDI", NULL, "GPIO32"),
  [EIC7700X_PAD_I2S1_SDO] =
    PINCTRL_PADNAME("I2S1_SDO", "I2S1_SDO", NULL, "GPIO33"),
  [EIC7700X_PAD_GPIO34] =
    PINCTRL_PADNAME("GPIO34", "POR_TIME_SEL1", "SD0_LED_CONTROL", "GPIO34"),
  [EIC7700X_PAD_RGMII1_RXCLK] =
    PINCTRL_PADNAME("RGMII1_RXCLK", "RGMII1_RXCLK"),
  [EIC7700X_PAD_RGMII1_RXDV] =
    PINCTRL_PADNAME("RGMII1_RXDV", "RGMII1_RXDV"),
  [EIC7700X_PAD_RGMII1_RXD0] =
    PINCTRL_PADNAME("RGMII1_RXD0", "RGMII1_RXD0"),
  [EIC7700X_PAD_RGMII1_RXD1] =
    PINCTRL_PADNAME("RGMII1_RXD1", "RGMII1_RXD1"),
  [EIC7700X_PAD_RGMII1_RXD2] =
    PINCTRL_PADNAME("RGMII1_RXD2", "RGMII1_RXD2"),
  [EIC7700X_PAD_RGMII1_RXD3] =
    PINCTRL_PADNAME("RGMII1_RXD3", "RGMII1_RXD3"),
  [EIC7700X_PAD_SPI1_CS0_N] =
    PINCTRL_PADNAME("SPI1_CS0_N", "SPI1_CS0_N", NULL, "GPIO35"),
  [EIC7700X_PAD_SPI1_CLK] =
    PINCTRL_PADNAME("SPI1_CLK", "SPI1_CLK", NULL, "GPIO36"),
  [EIC7700X_PAD_SPI1_D0] =
    PINCTRL_PADNAME("SPI1_D0", "SPI1_D0", "I2C9_SCL", "GPIO37", "UART4_TX"),
  [EIC7700X_PAD_SPI1_D1] =
    PINCTRL_PADNAME("SPI1_D1", "SPI1_D1", "I2C9_SDA", "GPIO38", "UART4_RX"),
  [EIC7700X_PAD_SPI1_D2] =
    PINCTRL_PADNAME("SPI1_D2", "SPI1_D2", "SD1_LED_CONTROL", "GPIO39"),
  [EIC7700X_PAD_SPI1_D3] =
    PINCTRL_PADNAME("SPI1_D3", "SPI1_D3", "PWM1", "GPIO40"),
  [EIC7700X_PAD_SPI1_CS1_N] =
    PINCTRL_PADNAME("SPI1_CS1_N", "SPI1_CS1_N", "PWM2", "GPIO41"),
  [EIC7700X_PAD_RGMII1_MDC] =
    PINCTRL_PADNAME("RGMII1_MDC", "RGMII1_MDC"),
  [EIC7700X_PAD_RGMII1_MDIO] =
    PINCTRL_PADNAME("RGMII1_MDIO", "RGMII1_MDIO"),
  [EIC7700X_PAD_RGMII1_INTB] =
    PINCTRL_PADNAME("RGMII1_INTB", "RGMII1_INTB"),
  [EIC7700X_PAD_USB0_PWREN] =
    PINCTRL_PADNAME("USB0_PWREN", "USB0_PWREN", NULL, "GPIO42"),
  [EIC7700X_PAD_USB1_PWREN] =
    PINCTRL_PADNAME("USB1_PWREN", "USB1_PWREN", NULL, "GPIO43"),
  [EIC7700X_PAD_I2C0_SCL] =
    PINCTRL_PADNAME("I2C0_SCL", "I2C0_SCL", NULL, "GPIO44"),
  [EIC7700X_PAD_I2C0_SDA] =
    PINCTRL_PADNAME("I2C0_SDA", "I2C0_SDA", NULL, "GPIO45"),
  [EIC7700X_PAD_I2C1_SCL] =
    PINCTRL_PADNAME("I2C1_SCL", "I2C1_SCL", NULL, "GPIO46"),
  [EIC7700X_PAD_I2C1_SDA] =
    PINCTRL_PADNAME("I2C1_SDA", "I2C1_SDA", NULL, "GPIO47"),
  [EIC7700X_PAD_I2C2_SCL] =
    PINCTRL_PADNAME("I2C2_SCL", "I2C2_SCL", NULL, "GPIO48"),
  [EIC7700X_PAD_I2C2_SDA] =
    PINCTRL_PADNAME("I2C2_SDA", "I2C2_SDA", NULL, "GPIO49"),
  [EIC7700X_PAD_I2C3_SCL] =
    PINCTRL_PADNAME("I2C3_SCL", "I2C3_SCL", NULL, "GPIO50"),
  [EIC7700X_PAD_I2C3_SDA] =
    PINCTRL_PADNAME("I2C3_SDA", "I2C3_SDA", NULL, "GPIO51"),
  [EIC7700X_PAD_I2C4_SCL] =
    PINCTRL_PADNAME("I2C4_SCL", "I2C4_SCL", NULL, "GPIO52"),
  [EIC7700X_PAD_I2C4_SDA] =
    PINCTRL_PADNAME("I2C4_SDA", "I2C4_SDA", NULL, "GPIO53"),
  [EIC7700X_PAD_I2C5_SCL] =
    PINCTRL_PADNAME("I2C5_SCL", "I2C5_SCL", NULL, "GPIO54"),
  [EIC7700X_PAD_I2C5_SDA] =
    PINCTRL_PADNAME("I2C5_SDA", "I2C5_SDA", NULL, "GPIO55"),
  [EIC7700X_PAD_UART0_TX] =
    PINCTRL_PADNAME("UART0_TX", "UART0_TX", NULL, "GPIO56"),
  [EIC7700X_PAD_UART0_RX] =
    PINCTRL_PADNAME("UART0_RX", "UART0_RX", NULL, "GPIO57"),
  [EIC7700X_PAD_UART1_TX] =
    PINCTRL_PADNAME("UART1_TX", "UART1_TX", NULL, "GPIO58"),
  [EIC7700X_PAD_UART1_RX] =
    PINCTRL_PADNAME("UART1_RX", "UART1_RX", NULL, "GPIO59"),
  [EIC7700X_PAD_UART1_CTS] =
    PINCTRL_PADNAME("UART1_CTS", "UART1_CTS", "I2C6_SCL", "GPIO60"),
  [EIC7700X_PAD_UART1_RTS] =
    PINCTRL_PADNAME("UART1_RTS", "UART1_RTS", "I2C6_SDA", "GPIO61"),
  [EIC7700X_PAD_UART2_TX] =
    PINCTRL_PADNAME("UART2_TX", "UART2_TX", "I2C7_SCL", "GPIO62"),
  [EIC7700X_PAD_UART2_RX] =
    PINCTRL_PADNAME("UART2_RX", "UART2_RX", "I2C7_SDA", "GPIO63"),
  [EIC7700X_PAD_JTAG2_TCK] =
    PINCTRL_PADNAME("JTAG2_TCK", "JTAG2_TCK", NULL, "GPIO64"),
  [EIC7700X_PAD_JTAG2_TMS] =
    PINCTRL_PADNAME("JTAG2_TMS", "JTAG2_TMS", NULL, "GPIO65"),
  [EIC7700X_PAD_JTAG2_TDI] =
    PINCTRL_PADNAME("JTAG2_TDI", "JTAG2_TDI", NULL, "GPIO66"),
  [EIC7700X_PAD_JTAG2_TDO] =
    PINCTRL_PADNAME("JTAG2_TDO", "JTAG2_TDO", NULL, "GPIO67"),
  [EIC7700X_PAD_FAN_PWM] =
    PINCTRL_PADNAME("FAN_PWM", "FAN_PWM", NULL, "GPIO68"),
  [EIC7700X_PAD_FAN_TACH] =
    PINCTRL_PADNAME("FAN_TACH", "FAN_TACH", NULL, "GPIO69"),
  [EIC7700X_PAD_MIPI_CSI0_XVS] =
    PINCTRL_PADNAME("MIPI_CSI0_XVS", "MIPI_CSI0_XVS", NULL, "GPIO70"),
  [EIC7700X_PAD_MIPI_CSI0_XHS] =
    PINCTRL_PADNAME("MIPI_CSI0_XHS", "MIPI_CSI0_XHS", NULL, "GPIO71"),
  [EIC7700X_PAD_MIPI_CSI0_MCLK] =
    PINCTRL_PADNAME("MIPI_CSI0_MCLK", "MIPI_CSI0_MCLK", NULL, "GPIO72"),
  [EIC7700X_PAD_MIPI_CSI1_XVS] =
    PINCTRL_PADNAME("MIPI_CSI1_XVS", "MIPI_CSI1_XVS", NULL, "GPIO73"),
  [EIC7700X_PAD_MIPI_CSI1_XHS] =
    PINCTRL_PADNAME("MIPI_CSI1_XHS", "MIPI_CSI1_XHS", NULL, "GPIO74"),
  [EIC7700X_PAD_MIPI_CSI1_MCLK] =
    PINCTRL_PADNAME("MIPI_CSI1_MCLK", "MIPI_CSI1_MCLK", NULL, "GPIO75"),
  [EIC7700X_PAD_MIPI_CSI2_XVS] =
    PINCTRL_PADNAME("MIPI_CSI2_XVS", "MIPI_CSI2_XVS", NULL, "GPIO76"),
  [EIC7700X_PAD_MIPI_CSI2_XHS] =
    PINCTRL_PADNAME("MIPI_CSI2_XHS", "MIPI_CSI2_XHS", NULL, "GPIO77"),
  [EIC7700X_PAD_MIPI_CSI2_MCLK] =
    PINCTRL_PADNAME("MIPI_CSI2_MCLK", "MIPI_CSI2_MCLK", NULL, "GPIO78"),
  [EIC7700X_PAD_MIPI_CSI3_XVS] =
    PINCTRL_PADNAME("MIPI_CSI3_XVS", "MIPI_CSI3_XVS", NULL, "GPIO79"),
  [EIC7700X_PAD_MIPI_CSI3_XHS] =
    PINCTRL_PADNAME("MIPI_CSI3_XHS", "MIPI_CSI3_XHS", NULL, "GPIO80"),
  [EIC7700X_PAD_MIPI_CSI3_MCLK] =
    PINCTRL_PADNAME("MIPI_CSI3_MCLK", "MIPI_CSI3_MCLK", NULL, "GPIO81"),
  [EIC7700X_PAD_MIPI_CSI4_XVS] =
    PINCTRL_PADNAME("MIPI_CSI4_XVS", "MIPI_CSI4_XVS", NULL, "GPIO82"),
  [EIC7700X_PAD_MIPI_CSI4_XHS] =
    PINCTRL_PADNAME("MIPI_CSI4_XHS", "MIPI_CSI4_XHS", NULL, "GPIO83"),
  [EIC7700X_PAD_MIPI_CSI4_MCLK] =
    PINCTRL_PADNAME("MIPI_CSI4_MCLK", "MIPI_CSI4_MCLK", NULL, "GPIO84"),
  [EIC7700X_PAD_MIPI_CSI5_XVS] =
    PINCTRL_PADNAME("MIPI_CSI5_XVS", "MIPI_CSI5_XVS", NULL, "GPIO85"),
  [EIC7700X_PAD_MIPI_CSI5_XHS] =
    PINCTRL_PADNAME("MIPI_CSI5_XHS", "MIPI_CSI5_XHS", NULL, "GPIO86"),
  [EIC7700X_PAD_MIPI_CSI5_MCLK] =
    PINCTRL_PADNAME("MIPI_CSI5_MCLK", "MIPI_CSI5_MCLK", NULL, "GPIO87"),
  [EIC7700X_PAD_SPI3_CS_N] =
    PINCTRL_PADNAME("SPI3_CS_N", "SPI3_CS_N", NULL, "GPIO88"),
  [EIC7700X_PAD_SPI3_CLK] =
    PINCTRL_PADNAME("SPI3_CLK", "SPI3_CLK", NULL, "GPIO89"),
  [EIC7700X_PAD_SPI3_DI] =
    PINCTRL_PADNAME("SPI3_DI", "SPI3_DI", NULL, "GPIO90"),
  [EIC7700X_PAD_SPI3_DO] =
    PINCTRL_PADNAME("SPI3_DO", "SPI3_DO", NULL, "GPIO91"),
  [EIC7700X_PAD_GPIO92] =
    PINCTRL_PADNAME("GPIO92",
                    "I2C8_SCL", "MIPI_CSI_XTRIG0", "GPIO92", "UART3_TX"),
  [EIC7700X_PAD_GPIO93] =
    PINCTRL_PADNAME("GPIO93",
                    "I2C8_SDA", "MIPI_CSI_XTRIG1", "GPIO93", "UART3_RX"),
  [EIC7700X_PAD_S_MODE] =
    PINCTRL_PADNAME("S_MODE", "S_MODE", NULL, "GPIO94"),
  [EIC7700X_PAD_GPIO95] =
    PINCTRL_PADNAME("GPIO95", "LPDDR_REFCLK_SEL", NULL, "GPIO95"),
  [EIC7700X_PAD_SPI0_CS_N] =
    PINCTRL_PADNAME("SPI0_CS_N", "SPI0_CS_N", NULL, "GPIO96"),
  [EIC7700X_PAD_SPI0_CLK] =
    PINCTRL_PADNAME("SPI0_CLK", "SPI0_CLK", NULL, "GPIO97"),
  [EIC7700X_PAD_SPI0_D0] =
    PINCTRL_PADNAME("SPI0_D0", "SPI0_D0", NULL, "GPIO98"),
  [EIC7700X_PAD_SPI0_D1] =
    PINCTRL_PADNAME("SPI0_D1", "SPI0_D1", NULL, "GPIO99"),
  [EIC7700X_PAD_SPI0_D2] =
    PINCTRL_PADNAME("SPI0_D2", "SPI0_D2", NULL, "GPIO100"),
  [EIC7700X_PAD_SPI0_D3] =
    PINCTRL_PADNAME("SPI0_D3", "SPI0_D3", NULL, "GPIO101"),
  [EIC7700X_PAD_I2C10_SCL] =
    PINCTRL_PADNAME("I2C10_SCL", "I2C10_SCL", NULL, "GPIO102"),
  [EIC7700X_PAD_I2C10_SDA] =
    PINCTRL_PADNAME("I2C10_SDA", "I2C10_SDA", NULL, "GPIO103"),
  [EIC7700X_PAD_I2C11_SCL] =
    PINCTRL_PADNAME("I2C11_SCL", "I2C11_SCL", NULL, "GPIO104"),
  [EIC7700X_PAD_I2C11_SDA] =
    PINCTRL_PADNAME("I2C11_SDA", "I2C11_SDA", NULL, "GPIO105"),
  [EIC7700X_PAD_GPIO106] =
    PINCTRL_PADNAME("GPIO106", "GPIO106"),
  [EIC7700X_PAD_BOOT_SEL0] =
    PINCTRL_PADNAME("BOOT_SEL0", "BOOT_SEL0", NULL, "GPIO107"),
  [EIC7700X_PAD_BOOT_SEL1] =
    PINCTRL_PADNAME("BOOT_SEL1", "BOOT_SEL1", NULL, "GPIO108"),
  [EIC7700X_PAD_BOOT_SEL2] =
    PINCTRL_PADNAME("BOOT_SEL2", "BOOT_SEL2", NULL, "GPIO109"),
  [EIC7700X_PAD_BOOT_SEL3] =
    PINCTRL_PADNAME("BOOT_SEL3", "BOOT_SEL3", NULL, "GPIO110"),
  [EIC7700X_PAD_GPIO111] =
    PINCTRL_PADNAME("GPIO111", "GPIO111"),
  [EIC7700X_PAD_RESERVED4] =
    PINCTRL_PADNAME("RESERVED4", NULL),
  [EIC7700X_PAD_RESERVED5] =
    PINCTRL_PADNAME("RESERVED5", NULL),
  [EIC7700X_PAD_RESERVED6] =
    PINCTRL_PADNAME("RESERVED6", NULL),
  [EIC7700X_PAD_RESERVED7] =
    PINCTRL_PADNAME("RESERVED7", NULL),
  [EIC7700X_PAD_LPDDR_REF_CLK] =
    PINCTRL_PADNAME("LPDDR_REF_CLK", "LPDDR_REF_CLK"),
  [EIC7700X_PAD_ADDR_RGMII0_SEL_MODE] =
    PINCTRL_PADNAME("ADDR_RGMII0_SEL_MODE", NULL),
  [EIC7700X_PAD_ADDR_RGMII1_SEL_MODE] =
    PINCTRL_PADNAME("ADDR_RGMII1_SEL_MODE", NULL),
};

#endif /* HAVE_PINCTRL_TEXT */

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

/****************************************************************************
 * Name: eic7700x_pad_name
 *
 * Description:
 *   The manual's name for a pad, or NULL if the pad id is out of range or
 *   the name tables are not built.
 *
 ****************************************************************************/

FAR const char *eic7700x_pad_name(unsigned int pad)
{
#ifdef HAVE_PINCTRL_TEXT
  return pinctrl_padname(g_eic7700x_pad_names, EIC7700X_PAD_NPADS, pad);
#else
  return NULL;
#endif
}

/****************************************************************************
 * Name: eic7700x_pad_funcname
 *
 * Description:
 *   The name of function select func on pad, or NULL if the manual does
 *   not document it or the name tables are not built.
 *
 ****************************************************************************/

FAR const char *eic7700x_pad_funcname(unsigned int pad, unsigned int func)
{
#ifdef HAVE_PINCTRL_TEXT
  return pinctrl_funcname(g_eic7700x_pad_names, EIC7700X_PAD_NPADS,
                          pad, func);
#else
  return NULL;
#endif
}
