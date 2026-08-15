/****************************************************************************
 * arch/risc-v/src/eic7700x/eic7700x_clk.c
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
#include <inttypes.h>
#include <errno.h>
#include <syslog.h>
#include <sys/param.h>

#include <nuttx/clk/clk.h>
#include <nuttx/clk/clk_provider.h>
#include <nuttx/debug.h>

#include "riscv_internal.h"
#include "eic7700x_clk.h"
#include "hardware/eic7700x_clk.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clock names live in read only memory for the lifetime of the system, so
 * the framework is told to reference them rather than copy them.
 */

#define CLK_STATIC (CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC)

/* Table entry shorthands.  Keeping one clock per source line makes the
 * table reviewable against the register descriptions in the TRM.
 */

#define CLK_FIXED(_name, _rate) \
  { \
    .name = _name, .type = EIC7700X_CLKTYPE_FIXED, \
    .cflags = CLK_STATIC, .arg = _rate \
  }

#define CLK_PLL(_name, _parent, _base, _out, _lock) \
  { \
    .name = _name, .parent = _parent, .type = EIC7700X_CLKTYPE_PLL, \
    .cflags = CLK_STATIC, .reg = _base, .bit = _out, .arg = _lock \
  }

#define CLK_FACTOR(_name, _parent, _mult, _div) \
  { \
    .name = _name, .parent = _parent, .type = EIC7700X_CLKTYPE_FACTOR, \
    .cflags = CLK_STATIC, .arg = ((_mult) << 8) | (_div) \
  }

#define CLK_DIV(_name, _parent, _off, _shift, _width, _lowdiv) \
  { \
    .name = _name, .parent = _parent, .type = EIC7700X_CLKTYPE_DIV, \
    .cflags = CLK_STATIC, .reg = EIC7700X_CLK_BASE + (_off), \
    .shift = _shift, .width = _width, .arg = _lowdiv \
  }

#define CLK_MUX(_name, _srcs, _tbl, _off, _shift, _width) \
  { \
    .name = _name, .parents = _srcs, .table = _tbl, \
    .nparents = nitems(_srcs), .type = EIC7700X_CLKTYPE_MUX, \
    .cflags = CLK_STATIC, .reg = EIC7700X_CLK_BASE + (_off), \
    .shift = _shift, .width = _width \
  }

#define CLK_GATE(_name, _parent, _off, _bit, _extra) \
  { \
    .name = _name, .parent = _parent, .type = EIC7700X_CLKTYPE_GATE, \
    .cflags = CLK_STATIC | (_extra), \
    .reg = EIC7700X_CLK_BASE + (_off), .bit = _bit \
  }

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum eic7700x_clktype_e
{
  EIC7700X_CLKTYPE_FIXED = 0,   /* Root oscillator or board reference     */
  EIC7700X_CLKTYPE_PLL,         /* One output of one PLL                  */
  EIC7700X_CLKTYPE_FACTOR,      /* Fixed ratio tap                        */
  EIC7700X_CLKTYPE_MUX,         /* Selector field                         */
  EIC7700X_CLKTYPE_DIV,         /* Divisor field                          */
  EIC7700X_CLKTYPE_GATE,        /* Single enable bit, set means running   */
};

/* One row per clock.  Rows must appear after every clock they name as a
 * parent.  Registering in topological order keeps the framework orphan
 * list empty, which matters because the orphan rescan in clk_register()
 * subscripts parent_names[] with a value read straight out of a mux
 * register.
 */

struct eic7700x_clkdesc_s
{
  FAR const char        *name;      /* Unique, under 40 characters, no /  */
  FAR const char        *parent;    /* Single parent, NULL when a root    */
  FAR const char * const *parents;  /* Parent list, mux only              */
  FAR const uint8_t     *table;     /* Selector values, mux only          */
  uintptr_t              reg;       /* Absolute register address          */
  uint32_t               arg;       /* Rate, ratio, lowdiv or lock bit    */
  uint8_t                type;
  uint8_t                nparents;
  uint8_t                cflags;
  uint8_t                bit;       /* Gate bit or PLL output number      */
  uint8_t                shift;
  uint8_t                width;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* System configuration root selector (TRM p136).  Value 0 routes the
 * divided spll0_fout3, value 1 routes the crystal.  The reset value is 1;
 * the boot loader is what moves it onto the PLL.
 */

static const char * const g_syscfg_parents[] =
{
  "div_sys_cfg",
  "xtal_24m",
};

static const uint8_t g_syscfg_table[] =
{
  0, 1
};

/* U84 core selector (TRM p135).  Resets to the crystal, like the rest of
 * the root selectors on this SoC.
 */

static const char * const g_u84_core_parents[] =
{
  "cpupll_fout2",
  "clk_u84_core_lp",
  "xtal_24m",
};

static const uint8_t g_u84_core_table[] =
{
  0, 1, 2
};

/* U84 bus ratio selector (TRM p135).  Unlike the scpu and lpcpu clusters,
 * whose ratio bits are documented as always two to one, the U84 bus can
 * run at either half the core clock or at the core clock.
 */

static const char * const g_u84_bus_parents[] =
{
  "clk_u84_core_div2",
  "clk_u84_core",
};

static const uint8_t g_u84_bus_table[] =
{
  0, 1
};

/* The boot SPI, SCPU and LPCPU core selectors all choose between their
 * own divided PLL output and the crystal, and all three reset to the
 * crystal (TRM p100 to p102).
 */

static const char * const g_bootspi_parents[] =
{
  "div_bootspi",
  "xtal_24m",
};

static const char * const g_scpu_core_parents[] =
{
  "div_scpu_core",
  "xtal_24m",
};

static const char * const g_lpcpu_core_parents[] =
{
  "div_lpcpu_core",
  "xtal_24m",
};

/* SCPU bus ratio: value 0 is two to one, value 1 is one to one */

static const char * const g_scpu_bus_parents[] =
{
  "clk_scpu_core_div2",
  "clk_scpu_core",
};

/* DDR AXI source (TRM p103).  Note the parent order is spll2 first here,
 * which is the opposite of the video domain selectors.
 */

static const char * const g_ddr_aclk_parents[] =
{
  "spll2_fout1",
  "spll0_fout1",
};

static const uint8_t g_sel2_table[] =
{
  0, 1
};

/* High speed peripheral backup selectors (TRM p110).  Value 0 keeps the
 * derived clock, value 1 switches to the board supplied LPDDR reference.
 */

static const char * const g_sata_phy_parents[] =
{
  "div_sata_phy_ref",
  "lpddr_ref_bak",
};

static const char * const g_eth_core_parents[] =
{
  "div_eth0_core",
  "lpddr_ref_bak",
};

static const char * const g_rmii_ref_parents[] =
{
  "div_rmii_ref",
  "lpddr_ref_bak",
};

/* SD and eMMC core selectors (TRM p112) */

static const char * const g_mshc_parents[] =
{
  "spll0_fout3",
  "spll2_fout3",
};

/* Always on DMA selector (TRM p130), which also resets to the crystal */

static const char * const g_aondma_parents[] =
{
  "div_aondma_axi",
  "xtal_24m",
};

/* DSP and die to die AXI sources (TRM p106, p108).  spll2 first, as with
 * the DDR selector.
 */

static const char * const g_spll2_spll0_parents[] =
{
  "spll2_fout1",
  "spll0_fout1",
};

/* Neural processor selectors (TRM p115, p116).  Both fields are two bits
 * wide but only three encodings are defined, so an unexpected value has
 * somewhere safe to land.
 */

static const char * const g_npu_llc_parents[] =
{
  "div_npu_llc_src0",
  "div_npu_llc_src1",
  "vpll_fout1",
};

static const char * const g_npu_core_parents[] =
{
  "spll1_fout1",
  "vpll_fout1",
  "spll2_fout2",
};

static const uint8_t g_sel3_table[] =
{
  0, 1, 2
};

/* Video AXI sources (TRM p118, p122).  Note the order is the reverse of
 * the DDR, DSP and die to die selectors: spll0 is encoding zero here.
 */

static const char * const g_spll0_spll2_parents[] =
{
  "spll0_fout1",
  "spll2_fout1",
};

/* Video input pipeline sources (TRM p117 to p119) */

static const char * const g_vpll_spll0_parents[] =
{
  "vpll_fout1",
  "spll0_fout1",
};

/* Video output pixel and audio master sources (TRM p124, p125) */

static const char * const g_vo_pixel_parents[] =
{
  "vpll_fout1",
  "spll2_fout2",
};

static const char * const g_vo_mclk_parents[] =
{
  "clk_vo_mclk_st1",
  "ext_mclk",
};

/* The clock tree, in topological order.
 *
 * Rates are never hard coded except for the crystals and the board
 * supplied references: everything else is computed from the registers.
 * That is not a stylistic choice.  Six root selectors on this SoC reset
 * to the 24MHz crystal and are switched onto their PLLs by the boot
 * loader, so the frequencies quoted in the TRM clock tree describe the
 * post boot state rather than what a given board is actually running.
 */

static const struct eic7700x_clkdesc_s g_eic7700x_clks[] =
{
  /* Roots (TRM p72) */

  CLK_FIXED("xtal_24m", 24000000),
  CLK_FIXED("xtal_32k", 32768),

  /* Backup reference arriving on the LPDDR reference pad.  The clock
   * tree table leaves its default blank and gives 125MHz as the maximum,
   * while the registers that select it call it the 50MHz backup clock.
   * It is a board level input, so 50MHz is the sensible assumption and
   * boards that drive it differently will need to say so.
   */

  CLK_FIXED("lpddr_ref_bak", 50000000),

  /* External audio master clock arriving on a pad.  The manual gives it
   * no frequency at all, so it is registered at zero and a board that
   * drives it has to say what it drives it with.
   */

  CLK_FIXED("ext_mclk", 0),

  /* PLL outputs (TRM p72, p80 to p99).  Documented rates are noted for
   * review; the driver computes them rather than trusting them.
   */

  CLK_PLL("spll0_fout1", "xtal_24m", EIC7700X_SPLL0_BASE, 1,
          PLL_STATUS_SPLL0_LOCK),                          /* 1600 MHz   */
  CLK_PLL("spll0_fout2", "xtal_24m", EIC7700X_SPLL0_BASE, 2,
          PLL_STATUS_SPLL0_LOCK),                          /*  800 MHz   */
  CLK_PLL("spll0_fout3", "xtal_24m", EIC7700X_SPLL0_BASE, 3,
          PLL_STATUS_SPLL0_LOCK),                          /*  400 MHz   */

  CLK_PLL("spll1_fout1", "xtal_24m", EIC7700X_SPLL1_BASE, 1,
          PLL_STATUS_SPLL1_LOCK),                          /* 1500 MHz   */
  CLK_PLL("spll1_fout2", "xtal_24m", EIC7700X_SPLL1_BASE, 2,
          PLL_STATUS_SPLL1_LOCK),                          /*  300 MHz   */
  CLK_PLL("spll1_fout3", "xtal_24m", EIC7700X_SPLL1_BASE, 3,
          PLL_STATUS_SPLL1_LOCK),                          /*  250 MHz   */

  CLK_PLL("spll2_fout1", "xtal_24m", EIC7700X_SPLL2_BASE, 1,
          PLL_STATUS_SPLL2_LOCK),                          /* 2080 MHz   */
  CLK_PLL("spll2_fout2", "xtal_24m", EIC7700X_SPLL2_BASE, 2,
          PLL_STATUS_SPLL2_LOCK),                          /* 1040 MHz   */
  CLK_PLL("spll2_fout3", "xtal_24m", EIC7700X_SPLL2_BASE, 3,
          PLL_STATUS_SPLL2_LOCK),                          /*  416 MHz   */

  CLK_PLL("vpll_fout1", "xtal_24m", EIC7700X_VPLL_BASE, 1,
          PLL_STATUS_VPLL_LOCK),                           /* 1188 MHz   */
  CLK_PLL("vpll_fout2", "xtal_24m", EIC7700X_VPLL_BASE, 2,
          PLL_STATUS_VPLL_LOCK),                           /*  594 MHz   */
  CLK_PLL("vpll_fout3", "xtal_24m", EIC7700X_VPLL_BASE, 3,
          PLL_STATUS_VPLL_LOCK),                           /* 49.5 MHz   */

  CLK_PLL("apll_fout1", "xtal_24m", EIC7700X_APLL_BASE, 1,
          PLL_STATUS_APLL_LOCK),                           /* 983.04 MHz */

  /* All three CPU PLL outputs are registered even though the output
   * enable field only defaults to outputs 1 and 3, because the U84 core
   * selector documents its first choice as fout2 (TRM p135).  The manual
   * does not reconcile the two, so the tree carries all of them and
   * leaves it to the enable state to say which are actually running.
   */

  CLK_PLL("cpupll_fout1", "xtal_24m", EIC7700X_CPUPLL_BASE, 1,
          PLL_STATUS_CPUPLL_LOCK),
  CLK_PLL("cpupll_fout2", "xtal_24m", EIC7700X_CPUPLL_BASE, 2,
          PLL_STATUS_CPUPLL_LOCK),
  CLK_PLL("cpupll_fout3", "xtal_24m", EIC7700X_CPUPLL_BASE, 3,
          PLL_STATUS_CPUPLL_LOCK),

  CLK_PLL("ddrpll_fout1", "xtal_24m", EIC7700X_DDRPLL_BASE, 1,
          PLL_STATUS_DDRPLL_LOCK),
  CLK_PLL("d2dpll_fout1", "xtal_24m", EIC7700X_D2DPLL_BASE, 1,
          PLL_STATUS_D2DPLL_LOCK),

  /* Configuration domain root (TRM p136).  clk_sys_cfg is the parent of
   * every peripheral pclk on the SoC, including the console UART, so it
   * is marked critical.
   */

  CLK_DIV("div_sys_cfg", "spll0_fout3", EIC7700X_SYSCFG_CLK_CTRL,
          SYSCFG_DIVSOR_SHIFT, SYSCFG_DIVSOR_WIDTH, 2),

  {
    .name     = "clk_sys_cfg",
    .parents  = g_syscfg_parents,
    .table    = g_syscfg_table,
    .nparents = nitems(g_syscfg_parents),
    .type     = EIC7700X_CLKTYPE_MUX,
    .cflags   = CLK_STATIC | CLK_IS_CRITICAL,
    .reg      = EIC7700X_CLK_BASE + EIC7700X_SYSCFG_CLK_CTRL,
    .shift    = SYSCFG_CLK_SEL_SHIFT,
    .width    = SYSCFG_CLK_SEL_WIDTH,
  },

  /* Low speed peripheral pclk gates, bank 0 (TRM p134).  Every one of
   * these hangs off the configuration domain.  uart0 is the NuttX console
   * on the boards supported so far, so it is marked critical to keep a
   * stray disable from taking the console down.
   */

  CLK_GATE("lsp_fan_pclk", "clk_sys_cfg", EIC7700X_LSP_CLK_EN0,
           LSP_CLK_EN0_FAN_PCLK, 0),
  CLK_GATE("lsp_pvt_pclk", "clk_sys_cfg", EIC7700X_LSP_CLK_EN0,
           LSP_CLK_EN0_PVT_PCLK, 0),

  CLK_GATE("lsp_i2c0_pclk", "clk_sys_cfg", EIC7700X_LSP_CLK_EN0,
           LSP_CLK_EN0_I2C_PCLK(0), 0),
  CLK_GATE("lsp_i2c1_pclk", "clk_sys_cfg", EIC7700X_LSP_CLK_EN0,
           LSP_CLK_EN0_I2C_PCLK(1), 0),
  CLK_GATE("lsp_i2c2_pclk", "clk_sys_cfg", EIC7700X_LSP_CLK_EN0,
           LSP_CLK_EN0_I2C_PCLK(2), 0),
  CLK_GATE("lsp_i2c3_pclk", "clk_sys_cfg", EIC7700X_LSP_CLK_EN0,
           LSP_CLK_EN0_I2C_PCLK(3), 0),
  CLK_GATE("lsp_i2c4_pclk", "clk_sys_cfg", EIC7700X_LSP_CLK_EN0,
           LSP_CLK_EN0_I2C_PCLK(4), 0),
  CLK_GATE("lsp_i2c5_pclk", "clk_sys_cfg", EIC7700X_LSP_CLK_EN0,
           LSP_CLK_EN0_I2C_PCLK(5), 0),
  CLK_GATE("lsp_i2c6_pclk", "clk_sys_cfg", EIC7700X_LSP_CLK_EN0,
           LSP_CLK_EN0_I2C_PCLK(6), 0),
  CLK_GATE("lsp_i2c7_pclk", "clk_sys_cfg", EIC7700X_LSP_CLK_EN0,
           LSP_CLK_EN0_I2C_PCLK(7), 0),
  CLK_GATE("lsp_i2c8_pclk", "clk_sys_cfg", EIC7700X_LSP_CLK_EN0,
           LSP_CLK_EN0_I2C_PCLK(8), 0),
  CLK_GATE("lsp_i2c9_pclk", "clk_sys_cfg", EIC7700X_LSP_CLK_EN0,
           LSP_CLK_EN0_I2C_PCLK(9), 0),

  /* The other two I2C controllers are in the always on domain and take a
   * register each rather than a bit in the LSP bank.  They are named for
   * the controllers they gate rather than for the registers, which count
   * from zero again: i2c10 and i2c11 to the rest of the world.
   */

  CLK_GATE("aon_i2c0_pclk", "clk_sys_cfg", EIC7700X_I2C0_CLK_CTRL,
           I2C_CLK_CTRL_PCLKEN, 0),
  CLK_GATE("aon_i2c1_pclk", "clk_sys_cfg", EIC7700X_I2C1_CLK_CTRL,
           I2C_CLK_CTRL_PCLKEN, 0),

  CLK_GATE("lsp_uart0_pclk", "clk_sys_cfg", EIC7700X_LSP_CLK_EN0,
           LSP_CLK_EN0_UART_PCLK(0), CLK_IS_CRITICAL),
  CLK_GATE("lsp_uart1_pclk", "clk_sys_cfg", EIC7700X_LSP_CLK_EN0,
           LSP_CLK_EN0_UART_PCLK(1), 0),
  CLK_GATE("lsp_uart2_pclk", "clk_sys_cfg", EIC7700X_LSP_CLK_EN0,
           LSP_CLK_EN0_UART_PCLK(2), 0),
  CLK_GATE("lsp_uart3_pclk", "clk_sys_cfg", EIC7700X_LSP_CLK_EN0,
           LSP_CLK_EN0_UART_PCLK(3), 0),
  CLK_GATE("lsp_uart4_pclk", "clk_sys_cfg", EIC7700X_LSP_CLK_EN0,
           LSP_CLK_EN0_UART_PCLK(4), 0),

  CLK_GATE("lsp_timer_pclk", "clk_sys_cfg", EIC7700X_LSP_CLK_EN0,
           LSP_CLK_EN0_TIMER_PCLK, 0),

  CLK_GATE("lsp_ssi0_pclk", "clk_sys_cfg", EIC7700X_LSP_CLK_EN0,
           LSP_CLK_EN0_SSI_PCLK(0), 0),
  CLK_GATE("lsp_ssi1_pclk", "clk_sys_cfg", EIC7700X_LSP_CLK_EN0,
           LSP_CLK_EN0_SSI_PCLK(1), 0),

  CLK_GATE("lsp_wdt0_pclk", "clk_sys_cfg", EIC7700X_LSP_CLK_EN0,
           LSP_CLK_EN0_WDT_PCLK(0), 0),
  CLK_GATE("lsp_wdt1_pclk", "clk_sys_cfg", EIC7700X_LSP_CLK_EN0,
           LSP_CLK_EN0_WDT_PCLK(1), 0),
  CLK_GATE("lsp_wdt2_pclk", "clk_sys_cfg", EIC7700X_LSP_CLK_EN0,
           LSP_CLK_EN0_WDT_PCLK(2), 0),
  CLK_GATE("lsp_wdt3_pclk", "clk_sys_cfg", EIC7700X_LSP_CLK_EN0,
           LSP_CLK_EN0_WDT_PCLK(3), 0),

  /* Low speed peripheral gates, bank 1 (TRM p135) */

  CLK_GATE("lsp_mbox0_pclk", "clk_sys_cfg", EIC7700X_LSP_CLK_EN1,
           LSP_CLK_EN1_MBOX_PCLK(0), 0),
  CLK_GATE("lsp_mbox1_pclk", "clk_sys_cfg", EIC7700X_LSP_CLK_EN1,
           LSP_CLK_EN1_MBOX_PCLK(1), 0),
  CLK_GATE("lsp_mbox2_pclk", "clk_sys_cfg", EIC7700X_LSP_CLK_EN1,
           LSP_CLK_EN1_MBOX_PCLK(2), 0),
  CLK_GATE("lsp_mbox3_pclk", "clk_sys_cfg", EIC7700X_LSP_CLK_EN1,
           LSP_CLK_EN1_MBOX_PCLK(3), 0),
  CLK_GATE("lsp_mbox4_pclk", "clk_sys_cfg", EIC7700X_LSP_CLK_EN1,
           LSP_CLK_EN1_MBOX_PCLK(4), 0),
  CLK_GATE("lsp_mbox5_pclk", "clk_sys_cfg", EIC7700X_LSP_CLK_EN1,
           LSP_CLK_EN1_MBOX_PCLK(5), 0),
  CLK_GATE("lsp_mbox6_pclk", "clk_sys_cfg", EIC7700X_LSP_CLK_EN1,
           LSP_CLK_EN1_MBOX_PCLK(6), 0),
  CLK_GATE("lsp_mbox7_pclk", "clk_sys_cfg", EIC7700X_LSP_CLK_EN1,
           LSP_CLK_EN1_MBOX_PCLK(7), 0),
  CLK_GATE("lsp_mbox8_pclk", "clk_sys_cfg", EIC7700X_LSP_CLK_EN1,
           LSP_CLK_EN1_MBOX_PCLK(8), 0),
  CLK_GATE("lsp_mbox9_pclk", "clk_sys_cfg", EIC7700X_LSP_CLK_EN1,
           LSP_CLK_EN1_MBOX_PCLK(9), 0),
  CLK_GATE("lsp_mbox10_pclk", "clk_sys_cfg", EIC7700X_LSP_CLK_EN1,
           LSP_CLK_EN1_MBOX_PCLK(10), 0),
  CLK_GATE("lsp_mbox11_pclk", "clk_sys_cfg", EIC7700X_LSP_CLK_EN1,
           LSP_CLK_EN1_MBOX_PCLK(11), 0),
  CLK_GATE("lsp_mbox12_pclk", "clk_sys_cfg", EIC7700X_LSP_CLK_EN1,
           LSP_CLK_EN1_MBOX_PCLK(12), 0),
  CLK_GATE("lsp_mbox13_pclk", "clk_sys_cfg", EIC7700X_LSP_CLK_EN1,
           LSP_CLK_EN1_MBOX_PCLK(13), 0),
  CLK_GATE("lsp_mbox14_pclk", "clk_sys_cfg", EIC7700X_LSP_CLK_EN1,
           LSP_CLK_EN1_MBOX_PCLK(14), 0),
  CLK_GATE("lsp_mbox15_pclk", "clk_sys_cfg", EIC7700X_LSP_CLK_EN1,
           LSP_CLK_EN1_MBOX_PCLK(15), 0),

  CLK_GATE("lsp_pvt_clk", "clk_sys_cfg", EIC7700X_LSP_CLK_EN1,
           LSP_CLK_EN1_PVT_CLK(0), 0),
  CLK_GATE("ddr_pvt_clk", "clk_sys_cfg", EIC7700X_LSP_CLK_EN1,
           LSP_CLK_EN1_PVT_CLK(1), 0),

  /* U84 application cluster (TRM p73, p135).  These are the harts NuttX
   * runs on, so the cluster clocks are critical.
   */

  CLK_FACTOR("clk_u84_core_lp", "spll0_fout2", 1, 2),       /* 400 MHz    */

  {
    .name     = "clk_u84_core",
    .parents  = g_u84_core_parents,
    .table    = g_u84_core_table,
    .nparents = nitems(g_u84_core_parents),
    .type     = EIC7700X_CLKTYPE_MUX,
    .cflags   = CLK_STATIC | CLK_IS_CRITICAL,
    .reg      = EIC7700X_CLK_BASE + EIC7700X_U84_CLK_CTRL,
    .shift    = U84_CORE_CLK_SEL_SHIFT,
    .width    = U84_CORE_CLK_SEL_WIDTH,
  },

  CLK_FACTOR("clk_u84_core_div2", "clk_u84_core", 1, 2),

  {
    .name     = "clk_u84_bus",
    .parents  = g_u84_bus_parents,
    .table    = g_u84_bus_table,
    .nparents = nitems(g_u84_bus_parents),
    .type     = EIC7700X_CLKTYPE_MUX,
    .cflags   = CLK_STATIC | CLK_IS_CRITICAL,
    .reg      = EIC7700X_CLK_BASE + EIC7700X_U84_CLK_CTRL,
    .shift    = U84_BUSCLK_RATIO_SEL_SHIFT,
    .width    = U84_BUSCLK_RATIO_SEL_WIDTH,
  },

  CLK_GATE("u84_core0_clk", "clk_u84_core", EIC7700X_U84_CLK_CTRL,
           U84_CORE_CLKEN(0), CLK_IS_CRITICAL),
  CLK_GATE("u84_core1_clk", "clk_u84_core", EIC7700X_U84_CLK_CTRL,
           U84_CORE_CLKEN(1), CLK_IS_CRITICAL),
  CLK_GATE("u84_core2_clk", "clk_u84_core", EIC7700X_U84_CLK_CTRL,
           U84_CORE_CLKEN(2), CLK_IS_CRITICAL),
  CLK_GATE("u84_core3_clk", "clk_u84_core", EIC7700X_U84_CLK_CTRL,
           U84_CORE_CLKEN(3), CLK_IS_CRITICAL),

  CLK_GATE("u84_trace0_clk", "clk_u84_bus", EIC7700X_U84_CLK_CTRL,
           U84_TRACE_CLKEN(0), 0),
  CLK_GATE("u84_trace1_clk", "clk_u84_bus", EIC7700X_U84_CLK_CTRL,
           U84_TRACE_CLKEN(1), 0),
  CLK_GATE("u84_trace2_clk", "clk_u84_bus", EIC7700X_U84_CLK_CTRL,
           U84_TRACE_CLKEN(2), 0),
  CLK_GATE("u84_trace3_clk", "clk_u84_bus", EIC7700X_U84_CLK_CTRL,
           U84_TRACE_CLKEN(3), 0),
  CLK_GATE("u84_trace_com_clk", "clk_u84_bus", EIC7700X_U84_CLK_CTRL,
           U84_TRACE_COM_CLKEN, 0),

  /* Real time clock chain (TRM p132).  clk_1m is the cpu rtc toggle
   * signal the CLINT mtime counter increments from (TRM p251), which
   * makes it the source of the NuttX system tick.  It is critical for
   * that reason.  Note the two divisors in this register do not share an
   * encoding: rtc_toggle_divsor treats 0 and 1 as bypass while
   * aon_rtc_divsor uses the usual divide by two aliases.
   */

  {
    .name   = "clk_1m",
    .parent = "xtal_24m",
    .type   = EIC7700X_CLKTYPE_DIV,
    .cflags = CLK_STATIC | CLK_IS_CRITICAL,
    .reg    = EIC7700X_CLK_BASE + EIC7700X_RTC_CLK_CTRL,
    .shift  = RTC_TOGGLE_DIVSOR_SHIFT,
    .width  = RTC_TOGGLE_DIVSOR_WIDTH,
    .arg    = 1,
  },

  CLK_DIV("div_clk_rtc", "clk_1m", EIC7700X_RTC_CLK_CTRL,
          RTC_AON_DIVSOR_SHIFT, RTC_AON_DIVSOR_WIDTH, 2),
  CLK_GATE("clk_rtc", "div_clk_rtc", EIC7700X_RTC_CLK_CTRL,
           RTC_CLKEN, 0),
  CLK_GATE("clk_rtc_cfg", "clk_sys_cfg", EIC7700X_RTC_CLK_CTRL,
           RTC_CFG_CLKEN, 0),

  /* Low speed timers (TRM p131).  The counters run straight off the
   * crystal; only the register interface uses the configuration domain.
   */

  CLK_GATE("timer0_clk", "xtal_24m", EIC7700X_TIMER_CLK_CTRL,
           TIMER_CLK_CLKEN(0), 0),
  CLK_GATE("timer1_clk", "xtal_24m", EIC7700X_TIMER_CLK_CTRL,
           TIMER_CLK_CLKEN(1), 0),
  CLK_GATE("timer2_clk", "xtal_24m", EIC7700X_TIMER_CLK_CTRL,
           TIMER_CLK_CLKEN(2), 0),
  CLK_GATE("timer3_clk", "xtal_24m", EIC7700X_TIMER_CLK_CTRL,
           TIMER_CLK_CLKEN(3), 0),

  CLK_GATE("timer0_pclk", "clk_sys_cfg", EIC7700X_TIMER_CLK_CTRL,
           TIMER_PCLK_CLKEN(0), 0),
  CLK_GATE("timer1_pclk", "clk_sys_cfg", EIC7700X_TIMER_CLK_CTRL,
           TIMER_PCLK_CLKEN(1), 0),
  CLK_GATE("timer2_pclk", "clk_sys_cfg", EIC7700X_TIMER_CLK_CTRL,
           TIMER_PCLK_CLKEN(2), 0),
  CLK_GATE("timer3_pclk", "clk_sys_cfg", EIC7700X_TIMER_CLK_CTRL,
           TIMER_PCLK_CLKEN(3), 0),

  CLK_GATE("timer3_clk8", "vpll_fout3", EIC7700X_TIMER_CLK_CTRL,
           TIMER3_CLK8_CLKEN, 0),

  /* Network on chip (TRM p99).  The watchdog reference comes off the
   * crystal: the clock tree table names the configuration domain as its
   * source, but that cannot be right because the divisor resets to one
   * and the tree itself gives the result as 24MHz.
   */

  CLK_DIV("div_noc_nsp", "spll2_fout1", EIC7700X_NOC_CLK_CTRL,
          NOC_NSP_DIVSOR_SHIFT, NOC_NSP_DIVSOR_WIDTH, 2),
  CLK_GATE("noc_nsp_clk", "div_noc_nsp", EIC7700X_NOC_CLK_CTRL,
           NOC_NSP_CLKEN, CLK_IS_CRITICAL),
  CLK_GATE("rnoc_nsp_clk", "div_noc_nsp", EIC7700X_NOC_CLK_CTRL,
           NOC_RNOC_NSP_CLKEN, CLK_IS_CRITICAL),

  CLK_DIV("div_noc_wdref", "xtal_24m", EIC7700X_NOC_CLK_CTRL,
          NOC_WDREF_DIVSOR_SHIFT, NOC_WDREF_DIVSOR_WIDTH, 1),
  CLK_GATE("noc_wd_refclk", "div_noc_wdref", EIC7700X_NOC_CLK_CTRL,
           NOC_WDREF_CLKEN, 0),

  /* Boot SPI (TRM p100).  The divisor resets to twenty, which against
   * the 800MHz parent gives 40MHz rather than the 50MHz the selector
   * description quotes.  The two statements cannot both hold; the rate
   * reported here is whatever the registers actually say.
   */

  CLK_DIV("div_bootspi", "spll0_fout2", EIC7700X_BOOTSPI_CLK_CTRL,
          BOOTSPI_DIVSOR_SHIFT, BOOTSPI_DIVSOR_WIDTH, 2),
  CLK_MUX("clk_bootspi_st3", g_bootspi_parents, g_sel2_table,
          EIC7700X_BOOTSPI_CLK_CTRL, CLKCTRL_SRC_SEL_SHIFT,
          CLKCTRL_SRC_SEL_WIDTH),
  CLK_GATE("clk_bootspi", "clk_bootspi_st3", EIC7700X_BOOTSPI_CLK_CTRL,
           CLKCTRL_CLKEN, 0),
  CLK_GATE("clk_bootspi_cfg", "clk_sys_cfg", EIC7700X_BOOTSPI_CFG_CTRL,
           CLKCTRL_CLKEN, 0),

  /* Secure and low power clusters (TRM p101, p102).  Neither runs NuttX,
   * so unlike the U84 cluster these are not marked critical.
   */

  CLK_DIV("div_scpu_core", "spll0_fout1", EIC7700X_SCPU_CORE_CTRL,
          CLKCTRL_DIVSOR_SHIFT, CLKCTRL_DIVSOR_WIDTH, 2),
  CLK_MUX("clk_scpu_core_src", g_scpu_core_parents, g_sel2_table,
          EIC7700X_SCPU_CORE_CTRL, CLKCTRL_SRC_SEL_SHIFT,
          CLKCTRL_SRC_SEL_WIDTH),
  CLK_GATE("clk_scpu_core", "clk_scpu_core_src", EIC7700X_SCPU_CORE_CTRL,
           CLKCTRL_CLKEN, 0),
  CLK_FACTOR("clk_scpu_core_div2", "clk_scpu_core", 1, 2),
  CLK_MUX("clk_scpu_bus_src", g_scpu_bus_parents, g_sel2_table,
          EIC7700X_SCPU_BUS_CTRL, CPU_BUSCLK_SEL_SHIFT,
          CPU_BUSCLK_SEL_WIDTH),
  CLK_GATE("clk_scpu_bus", "clk_scpu_bus_src", EIC7700X_SCPU_BUS_CTRL,
           CLKCTRL_CLKEN, 0),

  CLK_DIV("div_lpcpu_core", "spll0_fout1", EIC7700X_LPCPU_CORE_CTRL,
          CLKCTRL_DIVSOR_SHIFT, CLKCTRL_DIVSOR_WIDTH, 2),
  CLK_MUX("clk_lpcpu_core_src", g_lpcpu_core_parents, g_sel2_table,
          EIC7700X_LPCPU_CORE_CTRL, CLKCTRL_SRC_SEL_SHIFT,
          CLKCTRL_SRC_SEL_WIDTH),
  CLK_GATE("clk_lpcpu_core", "clk_lpcpu_core_src",
           EIC7700X_LPCPU_CORE_CTRL, CLKCTRL_CLKEN, 0),

  /* The LPCPU bus ratio selector is documented as reserved, so that bus
   * is always half the core clock (TRM p102).
   */

  CLK_FACTOR("clk_lpcpu_bus_div", "clk_lpcpu_core", 1, 2),
  CLK_GATE("clk_lpcpu_bus", "clk_lpcpu_bus_div", EIC7700X_LPCPU_BUS_CTRL,
           CLKCTRL_CLKEN, 0),

  /* DDR controllers (TRM p103).  Everything downstream of the AXI clock
   * is critical: gating any of it stops main memory.
   */

  {
    .name     = "clk_ddr_aclk_st0",
    .parents  = g_ddr_aclk_parents,
    .table    = g_sel2_table,
    .nparents = nitems(g_ddr_aclk_parents),
    .type     = EIC7700X_CLKTYPE_MUX,
    .cflags   = CLK_STATIC | CLK_IS_CRITICAL,
    .reg      = EIC7700X_CLK_BASE + EIC7700X_DDR0_CLK_CTRL,
    .shift    = DDR_ACLKSRC_SEL_SHIFT,
    .width    = DDR_ACLKSRC_SEL_WIDTH,
  },

  {
    .name   = "clk_ddr_aclk",
    .parent = "clk_ddr_aclk_st0",
    .type   = EIC7700X_CLKTYPE_DIV,
    .cflags = CLK_STATIC | CLK_IS_CRITICAL,
    .reg    = EIC7700X_CLK_BASE + EIC7700X_DDR0_CLK_CTRL,
    .shift  = DDR_ACLK_DIVSOR_SHIFT,
    .width  = DDR_ACLK_DIVSOR_WIDTH,
    .arg    = 2,
  },

  CLK_GATE("ddrt_cfg_clk", "clk_sys_cfg", EIC7700X_DDR0_CLK_CTRL,
           DDR_CFG_CLKEN, CLK_IS_CRITICAL),

  CLK_GATE("ddrt0_p0_aclk", "clk_ddr_aclk", EIC7700X_DDR0_CLK_CTRL,
           DDR_PORT_ACLK_CLKEN(0), CLK_IS_CRITICAL),
  CLK_GATE("ddrt0_p1_aclk", "clk_ddr_aclk", EIC7700X_DDR0_CLK_CTRL,
           DDR_PORT_ACLK_CLKEN(1), CLK_IS_CRITICAL),
  CLK_GATE("ddrt0_p2_aclk", "clk_ddr_aclk", EIC7700X_DDR0_CLK_CTRL,
           DDR_PORT_ACLK_CLKEN(2), CLK_IS_CRITICAL),
  CLK_GATE("ddrt0_p3_aclk", "clk_ddr_aclk", EIC7700X_DDR0_CLK_CTRL,
           DDR_PORT_ACLK_CLKEN(3), CLK_IS_CRITICAL),
  CLK_GATE("ddrt0_p4_aclk", "clk_ddr_aclk", EIC7700X_DDR0_CLK_CTRL,
           DDR_PORT_ACLK_CLKEN(4), CLK_IS_CRITICAL),
  CLK_GATE("ddrt0_trace_aclk", "clk_ddr_aclk", EIC7700X_DDR0_CLK_CTRL,
           DDR_TRACE_CLKEN, 0),

  CLK_GATE("ddrt1_p0_aclk", "clk_ddr_aclk", EIC7700X_DDR1_CLK_CTRL,
           DDR_PORT_ACLK_CLKEN(0), CLK_IS_CRITICAL),
  CLK_GATE("ddrt1_p1_aclk", "clk_ddr_aclk", EIC7700X_DDR1_CLK_CTRL,
           DDR_PORT_ACLK_CLKEN(1), CLK_IS_CRITICAL),
  CLK_GATE("ddrt1_p2_aclk", "clk_ddr_aclk", EIC7700X_DDR1_CLK_CTRL,
           DDR_PORT_ACLK_CLKEN(2), CLK_IS_CRITICAL),
  CLK_GATE("ddrt1_p3_aclk", "clk_ddr_aclk", EIC7700X_DDR1_CLK_CTRL,
           DDR_PORT_ACLK_CLKEN(3), CLK_IS_CRITICAL),
  CLK_GATE("ddrt1_p4_aclk", "clk_ddr_aclk", EIC7700X_DDR1_CLK_CTRL,
           DDR_PORT_ACLK_CLKEN(4), CLK_IS_CRITICAL),
  CLK_GATE("ddrt1_trace_aclk", "clk_ddr_aclk", EIC7700X_DDR1_CLK_CTRL,
           DDR_TRACE_CLKEN, 0),

  /* Translation control unit (TRM p103).  Its AXI clock is a tap off the
   * DDR AXI clock rather than a domain of its own.
   */

  CLK_GATE("tcu_aclk", "clk_ddr_aclk", EIC7700X_TCU_ACLK_CTRL,
           CLKCTRL_CLKEN, 0),
  CLK_GATE("tcu_cfg_clk", "clk_sys_cfg", EIC7700X_TCU_CFG_CTRL,
           CLKCTRL_CLKEN, 0),

  /* High speed peripheral fabric (TRM p109).  The clock tree names the
   * parent of hsp_aclk as hsp_aclk_free, which has no row of its own;
   * the divisor here is annotated with that name and its reset value
   * against spll0_fout1 gives the documented 800MHz.
   */

  CLK_DIV("div_hsp_aclk", "spll0_fout1", EIC7700X_HSP_ACLK_CTRL,
          CLKCTRL_DIVSOR_SHIFT, CLKCTRL_DIVSOR_WIDTH, 2),
  CLK_GATE("hsp_aclk", "div_hsp_aclk", EIC7700X_HSP_ACLK_CTRL,
           CLKCTRL_CLKEN, 0),
  CLK_GATE("hsp_dma0_clk", "hsp_aclk", EIC7700X_HSP_ACLK_CTRL,
           HSP_DMA0_CLKEN, 0),
  CLK_GATE("hsp_cfg_clk", "clk_sys_cfg", EIC7700X_HSP_CFG_CTRL,
           CLKCTRL_CLKEN, 0),
  CLK_GATE("hsp_pclk", "clk_sys_cfg", EIC7700X_HSP_CFG_CTRL,
           HSP_PCLK_EN, 0),

  /* SATA reference (TRM p110) */

  CLK_DIV("div_sata_phy_ref", "spll1_fout2", EIC7700X_SATA_OOB_CTRL,
          SATA_PHY_REF_DIVSOR_SHIFT, SATA_PHY_REF_DIVSOR_WIDTH, 2),
  CLK_MUX("sata_phy_ref_gated", g_sata_phy_parents, g_sel2_table,
          EIC7700X_SATA_OOB_CTRL, SATA_PHY_BAK_SEL_SHIFT, 1),
  CLK_GATE("hsp_sata_phy_ref_clk", "sata_phy_ref_gated",
           EIC7700X_SATA_OOB_CTRL, SATA_OOB_CLKEN, 0),

  /* Ethernet (TRM p110).  The reduced media independent interface
   * reference is a fixed sixth of spll1_fout2 rather than a register
   * divisor; the clock tree gives its minimum, default and maximum
   * divisor all as six.
   *
   * Only the first controller carries backup selectors.  Their field
   * names are not indexed, so whether they also steer the second
   * controller is unclear; the second is modelled without them.
   */

  CLK_DIV("div_eth0_core", "spll1_fout3", EIC7700X_ETH0_CTRL,
          ETH_DIVSOR_SHIFT, ETH_DIVSOR_WIDTH, 2),
  CLK_MUX("eth0_core_src", g_eth_core_parents, g_sel2_table,
          EIC7700X_ETH0_CTRL, ETH_CORE_BAK_SEL_SHIFT, 1),
  CLK_GATE("hsp_eth0_core_clk", "eth0_core_src", EIC7700X_ETH0_CTRL,
           ETH_CLK_EN, 0),

  CLK_FACTOR("div_rmii_ref", "spll1_fout2", 1, 6),
  CLK_MUX("rmii_ref_clk_mux", g_rmii_ref_parents, g_sel2_table,
          EIC7700X_ETH0_CTRL, ETH_RMII_BAK_SEL_SHIFT, 1),
  CLK_GATE("rmii0_ref_clk", "rmii_ref_clk_mux", EIC7700X_ETH0_CTRL,
           ETH_RMII_REF_CLKEN, 0),

  CLK_DIV("div_eth1_core", "spll1_fout3", EIC7700X_ETH1_CTRL,
          ETH_DIVSOR_SHIFT, ETH_DIVSOR_WIDTH, 2),
  CLK_GATE("hsp_eth1_core_clk", "div_eth1_core", EIC7700X_ETH1_CTRL,
           ETH_CLK_EN, 0),
  CLK_GATE("rmii1_ref_clk", "rmii_ref_clk_mux", EIC7700X_ETH1_CTRL,
           ETH_RMII_REF_CLKEN, 0),

  /* SD and eMMC controllers (TRM p112) */

  CLK_MUX("mshc0_core_src", g_mshc_parents, g_sel2_table,
          EIC7700X_MSHC0_CORE_CTRL, MSHC_CORE_SEL_SHIFT, 1),
  CLK_DIV("div_mshc0_core", "mshc0_core_src", EIC7700X_MSHC0_CORE_CTRL,
          MSHC_DIVSOR_SHIFT, MSHC_DIVSOR_WIDTH, 2),
  CLK_GATE("hsp_mshc0_core_clk", "div_mshc0_core",
           EIC7700X_MSHC0_CORE_CTRL, MSHC_CORE_CLKEN, 0),

  CLK_MUX("mshc1_core_src", g_mshc_parents, g_sel2_table,
          EIC7700X_MSHC1_CORE_CTRL, MSHC_CORE_SEL_SHIFT, 1),
  CLK_DIV("div_mshc1_core", "mshc1_core_src", EIC7700X_MSHC1_CORE_CTRL,
          MSHC_DIVSOR_SHIFT, MSHC_DIVSOR_WIDTH, 2),
  CLK_GATE("hsp_mshc1_core_clk", "div_mshc1_core",
           EIC7700X_MSHC1_CORE_CTRL, MSHC_CORE_CLKEN, 0),

  CLK_MUX("mshc2_core_src", g_mshc_parents, g_sel2_table,
          EIC7700X_MSHC2_CORE_CTRL, MSHC_CORE_SEL_SHIFT, 1),
  CLK_DIV("div_mshc2_core", "mshc2_core_src", EIC7700X_MSHC2_CORE_CTRL,
          MSHC_DIVSOR_SHIFT, MSHC_DIVSOR_WIDTH, 2),
  CLK_GATE("hsp_mshc2_core_clk", "div_mshc2_core",
           EIC7700X_MSHC2_CORE_CTRL, MSHC_CORE_CLKEN, 0),

  /* PCI Express (TRM p114) */

  CLK_DIV("div_pcie_aclk", "spll2_fout2", EIC7700X_PCIE_ACLK_CTRL,
          CLKCTRL_DIVSOR_SHIFT, CLKCTRL_DIVSOR_WIDTH, 2),
  CLK_GATE("pciet_aclk", "div_pcie_aclk", EIC7700X_PCIE_ACLK_CTRL,
           CLKCTRL_CLKEN, 0),
  CLK_GATE("pciet_cfg_clk", "clk_sys_cfg", EIC7700X_PCIE_CFG_CTRL,
           CLKCTRL_CLKEN, 0),
  CLK_GATE("pciet_aux_clk", "clk_sys_cfg", EIC7700X_PCIE_CFG_CTRL,
           PCIE_AUX_CLKEN, 0),
  CLK_FACTOR("div_pciet_cr", "clk_sys_cfg", 1, 2),
  CLK_GATE("pciet_cr_clk", "div_pciet_cr", EIC7700X_PCIE_CFG_CTRL,
           PCIE_CR_CLKEN, 0),

  /* Always on DMA (TRM p130) */

  CLK_DIV("div_aondma_axi", "spll0_fout1", EIC7700X_AON_DMA_CLK_CTRL,
          CLKCTRL_DIVSOR_SHIFT, CLKCTRL_DIVSOR_WIDTH, 2),
  CLK_MUX("clk_aondma_axi_st3", g_aondma_parents, g_sel2_table,
          EIC7700X_AON_DMA_CLK_CTRL, CLKCTRL_SRC_SEL_SHIFT,
          CLKCTRL_SRC_SEL_WIDTH),
  CLK_GATE("aondma_axi_clk", "clk_aondma_axi_st3",
           EIC7700X_AON_DMA_CLK_CTRL, AONDMA_AXI_CLKEN, 0),
  CLK_GATE("aondma_cfg_clk", "clk_sys_cfg", EIC7700X_AON_DMA_CLK_CTRL,
           AONDMA_CFG_CLKEN, 0),
  CLK_GATE("aon_aclk", "clk_aondma_axi_st3", EIC7700X_AON_DMA_CLK_CTRL,
           AON_ACLK_CLKEN, 0),

  /* Secure blocks (TRM p133) */

  CLK_GATE("clk_pka_cfg", "clk_sys_cfg", EIC7700X_PKA_CLK_CTRL,
           CLKCTRL_CLKEN, 0),
  CLK_GATE("clk_spacc_cfg", "clk_sys_cfg", EIC7700X_SPACC_CLK_CTRL,
           CLKCTRL_CLKEN, 0),
  CLK_DIV("div_crypto", "spll0_fout1", EIC7700X_SPACC_CLK_CTRL,
          CRYPTO_DIVSOR_SHIFT, CRYPTO_DIVSOR_WIDTH, 2),
  CLK_GATE("clk_crypto", "div_crypto", EIC7700X_SPACC_CLK_CTRL,
           SPACC_CRYPTO_CLKEN, 0),
  CLK_GATE("clk_trng_cfg", "clk_sys_cfg", EIC7700X_TRNG_CLK_CTRL,
           CLKCTRL_CLKEN, 0),
  CLK_GATE("clk_otp_cfg", "clk_sys_cfg", EIC7700X_OTP_CLK_CTRL,
           CLKCTRL_CLKEN, 0),

  /* 3D graphics (TRM p105).  The gray clock comes off the crystal rather
   * than off the graphics AXI clock.
   */

  CLK_DIV("div_gpu_aclk", "spll0_fout1", EIC7700X_GPU_ACLK_CTRL,
          CLKCTRL_DIVSOR_SHIFT, CLKCTRL_DIVSOR_WIDTH, 2),
  CLK_GATE("gpu_aclk", "div_gpu_aclk", EIC7700X_GPU_ACLK_CTRL,
           CLKCTRL_CLKEN, 0),
  CLK_GATE("gpu_cfg_clk", "clk_sys_cfg", EIC7700X_GPU_CFG_CTRL,
           CLKCTRL_CLKEN, 0),
  CLK_GATE("gpu_gray_clk", "xtal_24m", EIC7700X_GPU_GRAY_CTRL,
           CLKCTRL_CLKEN, 0),

  /* Digital signal processor (TRM p106).  clk_dsp_root is the selector
   * output ahead of the divisor; the 2D graphics block taps it there
   * rather than after the divisor, which is why it is a clock in its own
   * right.
   */

  CLK_MUX("clk_dsp_root", g_spll2_spll0_parents, g_sel2_table,
          EIC7700X_DSP_ACLK_CTRL, CLKCTRL_SRC_SEL_SHIFT,
          CLKCTRL_SRC_SEL_WIDTH),
  CLK_DIV("clk_dsp_aclk_st1", "clk_dsp_root", EIC7700X_DSP_ACLK_CTRL,
          CLKCTRL_DIVSOR_SHIFT, CLKCTRL_DIVSOR_WIDTH, 2),
  CLK_GATE("dspt_aclk", "clk_dsp_aclk_st1", EIC7700X_DSP_ACLK_CTRL,
           CLKCTRL_CLKEN, 0),
  CLK_GATE("dspt_cfg_clk", "clk_sys_cfg", EIC7700X_DSP_CFG_CTRL,
           CLKCTRL_CLKEN, 0),

  /* Die to die interface (TRM p108).  Its configuration gate is at bit 0
   * rather than the usual bit 31.
   */

  CLK_MUX("clk_d2d_root", g_spll2_spll0_parents, g_sel2_table,
          EIC7700X_D2D_ACLK_CTRL, CLKCTRL_SRC_SEL_SHIFT,
          CLKCTRL_SRC_SEL_WIDTH),
  CLK_DIV("clk_d2ddr_aclk", "clk_d2d_root", EIC7700X_D2D_ACLK_CTRL,
          CLKCTRL_DIVSOR_SHIFT, CLKCTRL_DIVSOR_WIDTH, 2),
  CLK_GATE("d2d_aclk", "clk_d2ddr_aclk", EIC7700X_D2D_ACLK_CTRL,
           CLKCTRL_CLKEN, 0),
  CLK_GATE("d2d_cfg_clk", "clk_sys_cfg", EIC7700X_D2D_CFG_CTRL,
           D2D_CFG_CLKEN, 0),

  /* Neural processor (TRM p115) */

  CLK_DIV("div_npu_aclk", "spll0_fout1", EIC7700X_NPU_ACLK_CTRL,
          CLKCTRL_DIVSOR_SHIFT, CLKCTRL_DIVSOR_WIDTH, 2),
  CLK_GATE("npu_aclk", "div_npu_aclk", EIC7700X_NPU_ACLK_CTRL,
           CLKCTRL_CLKEN, 0),
  CLK_GATE("npu_cfg_clk", "clk_sys_cfg", EIC7700X_NPU_ACLK_CTRL,
           NPU_CFG_CLKEN, 0),

  CLK_DIV("div_npu_llc_src0", "spll0_fout1", EIC7700X_NPU_LLC_CTRL,
          NPU_LLC_SRC0_DIVSOR_SHIFT, NPU_LLC_DIVSOR_WIDTH, 2),
  CLK_DIV("div_npu_llc_src1", "spll2_fout1", EIC7700X_NPU_LLC_CTRL,
          NPU_LLC_SRC1_DIVSOR_SHIFT, NPU_LLC_DIVSOR_WIDTH, 2),
  CLK_MUX("npu_llclk_src", g_npu_llc_parents, g_sel3_table,
          EIC7700X_NPU_LLC_CTRL, NPU_LLCLK_SEL_SHIFT, NPU_SEL_WIDTH),
  CLK_GATE("npu_llc_aclk", "npu_llclk_src", EIC7700X_NPU_LLC_CTRL,
           CLKCTRL_CLKEN, 0),

  CLK_MUX("npu_core_src", g_npu_core_parents, g_sel3_table,
          EIC7700X_NPU_CORE_CTRL, NPU_CORECLK_SEL_SHIFT, NPU_SEL_WIDTH),
  CLK_DIV("div_npu_core", "npu_core_src", EIC7700X_NPU_CORE_CTRL,
          NPU_CORECLK_DIVSOR_SHIFT, NPU_DIVSOR_WIDTH, 2),
  CLK_GATE("npu_clk", "div_npu_core", EIC7700X_NPU_CORE_CTRL,
           CLKCTRL_CLKEN, 0),

  CLK_MUX("npu_e31_src", g_npu_core_parents, g_sel3_table,
          EIC7700X_NPU_CORE_CTRL, NPU_E31CLK_SEL_SHIFT, NPU_SEL_WIDTH),
  CLK_DIV("div_npu_e31", "npu_e31_src", EIC7700X_NPU_CORE_CTRL,
          NPU_E31CLK_DIVSOR_SHIFT, NPU_DIVSOR_WIDTH, 2),
  CLK_GATE("npu_e31_clk", "div_npu_e31", EIC7700X_NPU_CORE_CTRL,
           NPU_E31_CORE_CLKEN, 0),

  /* MIPI escape clock (TRM p73).  A fixed tenth of the configuration
   * domain with no register of its own, feeding both display phys.
   */

  CLK_FACTOR("clk_mipi_txesc", "clk_sys_cfg", 1, 10),

  /* Video input (TRM p117).  The dewarp, image signal and digital video
   * pipelines share a selector layout between the video PLL and spll0.
   */

  CLK_MUX("clk_vi_aclk_root", g_spll0_spll2_parents, g_sel2_table,
          EIC7700X_VI_ACLK_CTRL, CLKCTRL_SRC_SEL_SHIFT,
          CLKCTRL_SRC_SEL_WIDTH),
  CLK_DIV("clk_vi_aclk_st1", "clk_vi_aclk_root", EIC7700X_VI_ACLK_CTRL,
          CLKCTRL_DIVSOR_SHIFT, CLKCTRL_DIVSOR_WIDTH, 2),
  CLK_GATE("vi_aclk", "clk_vi_aclk_st1", EIC7700X_VI_ACLK_CTRL,
           CLKCTRL_CLKEN, 0),
  CLK_GATE("vi_cfg_clk", "clk_sys_cfg", EIC7700X_VI_ACLK_CTRL,
           VI_CFG_CLKEN, 0),

  CLK_MUX("vi_dw_root", g_vpll_spll0_parents, g_sel2_table,
          EIC7700X_VI_DW_CLK_CTRL, CLKCTRL_SRC_SEL_SHIFT,
          CLKCTRL_SRC_SEL_WIDTH),
  CLK_DIV("div_vi_dw", "vi_dw_root", EIC7700X_VI_DW_CLK_CTRL,
          CLKCTRL_DIVSOR_SHIFT, CLKCTRL_DIVSOR_WIDTH, 2),
  CLK_GATE("vi_dig_dw_clk", "div_vi_dw", EIC7700X_VI_DW_CLK_CTRL,
           CLKCTRL_CLKEN, 0),

  CLK_MUX("vi_dig_isp_root", g_vpll_spll0_parents, g_sel2_table,
          EIC7700X_VI_DIG_ISP_CTRL, CLKCTRL_SRC_SEL_SHIFT,
          CLKCTRL_SRC_SEL_WIDTH),
  CLK_DIV("div_vi_dig_isp", "vi_dig_isp_root", EIC7700X_VI_DIG_ISP_CTRL,
          CLKCTRL_DIVSOR_SHIFT, CLKCTRL_DIVSOR_WIDTH, 2),
  CLK_GATE("vi_dig_isp_clk", "div_vi_dig_isp", EIC7700X_VI_DIG_ISP_CTRL,
           CLKCTRL_CLKEN, 0),

  CLK_MUX("vi_dvp_root", g_vpll_spll0_parents, g_sel2_table,
          EIC7700X_VI_DVP_CLK_CTRL, CLKCTRL_SRC_SEL_SHIFT,
          CLKCTRL_SRC_SEL_WIDTH),
  CLK_DIV("div_vi_dvp", "vi_dvp_root", EIC7700X_VI_DVP_CLK_CTRL,
          CLKCTRL_DIVSOR_SHIFT, CLKCTRL_DIVSOR_WIDTH, 2),
  CLK_GATE("vi_dvp_clk", "div_vi_dvp", EIC7700X_VI_DVP_CLK_CTRL,
           CLKCTRL_CLKEN, 0),

  /* Camera shutter references, six identical blocks on a four byte
   * stride (TRM p120).
   */

  CLK_DIV("div_vi_ref0", "vpll_fout2", EIC7700X_VI_SHUTTER_CTRL(0),
          VI_SHUTTER_DIVSOR_SHIFT, VI_SHUTTER_DIVSOR_WIDTH, 2),
  CLK_GATE("vi_ref_clk0", "div_vi_ref0", EIC7700X_VI_SHUTTER_CTRL(0),
           CLKCTRL_CLKEN, 0),
  CLK_DIV("div_vi_ref1", "vpll_fout2", EIC7700X_VI_SHUTTER_CTRL(1),
          VI_SHUTTER_DIVSOR_SHIFT, VI_SHUTTER_DIVSOR_WIDTH, 2),
  CLK_GATE("vi_ref_clk1", "div_vi_ref1", EIC7700X_VI_SHUTTER_CTRL(1),
           CLKCTRL_CLKEN, 0),
  CLK_DIV("div_vi_ref2", "vpll_fout2", EIC7700X_VI_SHUTTER_CTRL(2),
          VI_SHUTTER_DIVSOR_SHIFT, VI_SHUTTER_DIVSOR_WIDTH, 2),
  CLK_GATE("vi_ref_clk2", "div_vi_ref2", EIC7700X_VI_SHUTTER_CTRL(2),
           CLKCTRL_CLKEN, 0),
  CLK_DIV("div_vi_ref3", "vpll_fout2", EIC7700X_VI_SHUTTER_CTRL(3),
          VI_SHUTTER_DIVSOR_SHIFT, VI_SHUTTER_DIVSOR_WIDTH, 2),
  CLK_GATE("vi_ref_clk3", "div_vi_ref3", EIC7700X_VI_SHUTTER_CTRL(3),
           CLKCTRL_CLKEN, 0),
  CLK_DIV("div_vi_ref4", "vpll_fout2", EIC7700X_VI_SHUTTER_CTRL(4),
          VI_SHUTTER_DIVSOR_SHIFT, VI_SHUTTER_DIVSOR_WIDTH, 2),
  CLK_GATE("vi_ref_clk4", "div_vi_ref4", EIC7700X_VI_SHUTTER_CTRL(4),
           CLKCTRL_CLKEN, 0),
  CLK_DIV("div_vi_ref5", "vpll_fout2", EIC7700X_VI_SHUTTER_CTRL(5),
          VI_SHUTTER_DIVSOR_SHIFT, VI_SHUTTER_DIVSOR_WIDTH, 2),
  CLK_GATE("vi_ref_clk5", "div_vi_ref5", EIC7700X_VI_SHUTTER_CTRL(5),
           CLKCTRL_CLKEN, 0),

  CLK_GATE("vi_phy_cfg_clk", "clk_sys_cfg", EIC7700X_VI_PHY_CLK_CTRL,
           VI_PHYCFG_CLKEN, 0),
  CLK_GATE("vi_phy_txclkesc", "clk_mipi_txesc", EIC7700X_VI_PHY_CLK_CTRL,
           VI_TXESC_CLKEN, 0),

  /* Video output (TRM p123).  The clock tree gives the video input AXI
   * clock as the parent of vo_aclk, but this register carries its own
   * selector and divisor annotated with vo_aclk, so the tree row looks
   * like a copy of the video input one.  The registers are used here.
   */

  CLK_MUX("vo_aclk_root", g_spll0_spll2_parents, g_sel2_table,
          EIC7700X_VO_ACLK_CTRL, CLKCTRL_SRC_SEL_SHIFT,
          CLKCTRL_SRC_SEL_WIDTH),
  CLK_DIV("div_vo_aclk", "vo_aclk_root", EIC7700X_VO_ACLK_CTRL,
          CLKCTRL_DIVSOR_SHIFT, CLKCTRL_DIVSOR_WIDTH, 2),
  CLK_GATE("vo_aclk", "div_vo_aclk", EIC7700X_VO_ACLK_CTRL,
           CLKCTRL_CLKEN, 0),
  CLK_GATE("vo_cfg_clk", "clk_sys_cfg", EIC7700X_VO_ACLK_CTRL,
           VO_CFG_CLKEN, 0),

  CLK_DIV("div_vo_iesm", "spll0_fout3", EIC7700X_VO_IESM_CLK_CTRL,
          CLKCTRL_DIVSOR_SHIFT, CLKCTRL_DIVSOR_WIDTH, 2),
  CLK_GATE("vo_hdmi_iesmclk", "div_vo_iesm", EIC7700X_VO_IESM_CLK_CTRL,
           CLKCTRL_CLKEN, 0),

  CLK_MUX("vo_pixel_root", g_vo_pixel_parents, g_sel2_table,
          EIC7700X_VO_PIXEL_CTRL, CLKCTRL_SRC_SEL_SHIFT,
          CLKCTRL_SRC_SEL_WIDTH),
  CLK_DIV("div_vo_pixel", "vo_pixel_root", EIC7700X_VO_PIXEL_CTRL,
          VO_PIXEL_DIVSOR_SHIFT, VO_PIXEL_DIVSOR_WIDTH, 2),
  CLK_GATE("vo_pixel_clk", "div_vo_pixel", EIC7700X_VO_PIXEL_CTRL,
           CLKCTRL_CLKEN, 0),

  CLK_DIV("clk_vo_mclk_st1", "apll_fout1", EIC7700X_VO_MCLK_CTRL,
          VO_MCLK_DIVSOR_SHIFT, VO_MCLK_DIVSOR_WIDTH, 2),
  CLK_MUX("vo_mclk_src", g_vo_mclk_parents, g_sel2_table,
          EIC7700X_VO_MCLK_CTRL, CLKCTRL_SRC_SEL_SHIFT,
          CLKCTRL_SRC_SEL_WIDTH),
  CLK_GATE("vo_i2s_mclk", "vo_mclk_src", EIC7700X_VO_MCLK_CTRL,
           CLKCTRL_CLKEN, 0),

  CLK_DIV("vo_cec_clk", "vpll_fout2", EIC7700X_VO_PHY_CLK_CTRL,
          VO_CEC_DIVSOR_SHIFT, VO_CEC_DIVSOR_WIDTH, 2),
  CLK_GATE("vo_cr_clk", "clk_mipi_txesc", EIC7700X_VO_PHY_CLK_CTRL,
           VO_CR_CLKEN, 0),
  CLK_GATE("vo_phy_txclkesc", "clk_mipi_txesc",
           EIC7700X_VO_PHY_CLK_CTRL, VO_TXESC_CLKEN, 0),

  /* Video codec (TRM p126).  The codec root selector names spll2_fout1
   * in the register description while the clock tree names spll2_fout2;
   * the register is followed here, which is also what makes the maximum
   * frequency the tree quotes come out right.
   */

  CLK_MUX("vc_aclk_root", g_spll0_spll2_parents, g_sel2_table,
          EIC7700X_VC_ACLK_CTRL, CLKCTRL_SRC_SEL_SHIFT,
          CLKCTRL_SRC_SEL_WIDTH),
  CLK_DIV("div_vc_aclk", "vc_aclk_root", EIC7700X_VC_ACLK_CTRL,
          CLKCTRL_DIVSOR_SHIFT, CLKCTRL_DIVSOR_WIDTH, 2),
  CLK_GATE("vc_aclk", "div_vc_aclk", EIC7700X_VC_ACLK_CTRL,
           CLKCTRL_CLKEN, 0),

  CLK_MUX("clk_vc_cdec_root", g_spll0_spll2_parents, g_sel2_table,
          EIC7700X_VCDEC_ROOT_CTRL, CLKCTRL_SRC_SEL_SHIFT,
          CLKCTRL_SRC_SEL_WIDTH),

  CLK_DIV("div_vc_je", "clk_vc_cdec_root", EIC7700X_JE_CLK_CTRL,
          CLKCTRL_DIVSOR_SHIFT, CLKCTRL_DIVSOR_WIDTH, 2),
  CLK_GATE("vc_je_clk", "div_vc_je", EIC7700X_JE_CLK_CTRL,
           CLKCTRL_CLKEN, 0),
  CLK_DIV("div_vc_jd", "clk_vc_cdec_root", EIC7700X_JD_CLK_CTRL,
          CLKCTRL_DIVSOR_SHIFT, CLKCTRL_DIVSOR_WIDTH, 2),
  CLK_GATE("vc_jd_clk", "div_vc_jd", EIC7700X_JD_CLK_CTRL,
           CLKCTRL_CLKEN, 0),
  CLK_DIV("div_vc_vd", "clk_vc_cdec_root", EIC7700X_VD_CLK_CTRL,
          CLKCTRL_DIVSOR_SHIFT, CLKCTRL_DIVSOR_WIDTH, 2),
  CLK_GATE("vc_vd_clk", "div_vc_vd", EIC7700X_VD_CLK_CTRL,
           CLKCTRL_CLKEN, 0),
  CLK_DIV("div_vc_ve", "clk_vc_cdec_root", EIC7700X_VE_CLK_CTRL,
          CLKCTRL_DIVSOR_SHIFT, CLKCTRL_DIVSOR_WIDTH, 2),
  CLK_GATE("vc_ve_clk", "div_vc_ve", EIC7700X_VE_CLK_CTRL,
           CLKCTRL_CLKEN, 0),

  CLK_GATE("vc_cfg_clk", "clk_sys_cfg", EIC7700X_VC_CLKEN_CTRL,
           VC_CFG_CLKEN, 0),
  CLK_GATE("vc_jd_pclk", "clk_sys_cfg", EIC7700X_VC_CLKEN_CTRL,
           VC_JD_PCLK_EN, 0),
  CLK_GATE("vc_je_pclk", "clk_sys_cfg", EIC7700X_VC_CLKEN_CTRL,
           VC_JE_PCLK_EN, 0),
  CLK_GATE("vc_mon_pclk", "clk_sys_cfg", EIC7700X_VC_CLKEN_CTRL,
           VC_MON_PCLK_EN, 0),
  CLK_GATE("vc_vd_pclk", "clk_sys_cfg", EIC7700X_VC_CLKEN_CTRL,
           VC_VD_PCLK_EN, 0),
  CLK_GATE("vc_ve_pclk", "clk_sys_cfg", EIC7700X_VC_CLKEN_CTRL,
           VC_VE_PCLK_EN, 0),

  /* 2D graphics (TRM p127), which taps the DSP selector output */

  CLK_DIV("clk_g2d_st1", "clk_dsp_root", EIC7700X_G2D_CTRL,
          CLKCTRL_DIVSOR_SHIFT, CLKCTRL_DIVSOR_WIDTH, 2),
  CLK_GATE("g2d_aclk", "clk_g2d_st1", EIC7700X_G2D_CTRL, G2D_ACLKEN, 0),
  CLK_GATE("g2d_clk", "clk_g2d_st1", EIC7700X_G2D_CTRL, G2D_CLKEN, 0),
  CLK_GATE("g2d_pclk", "clk_sys_cfg", EIC7700X_G2D_CTRL, G2D_PCLK_EN, 0),
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: eic7700x_clk_register_one
 *
 * Description:
 *   Register a single clock described by one table row.
 *
 * Input Parameters:
 *   desc - The table row to register
 *
 * Returned Value:
 *   OK on success, or a negated errno on failure.
 *
 ****************************************************************************/

static int eic7700x_clk_register_one(
              FAR const struct eic7700x_clkdesc_s *desc)
{
  struct eic7700x_clk_pll_s pll;
  struct eic7700x_clk_div_s divider;
  struct eic7700x_clk_mux_s mux;
  FAR struct clk_s *clk = NULL;

  switch (desc->type)
    {
      case EIC7700X_CLKTYPE_FIXED:
        clk = clk_register_fixed_rate(desc->name, NULL, desc->cflags,
                                      desc->arg);
        break;

      case EIC7700X_CLKTYPE_PLL:
        pll.reg     = desc->reg;
        pll.out     = desc->bit;
        pll.lockbit = desc->arg;
        clk = clk_register(desc->name, &desc->parent, 1, desc->cflags,
                           &g_eic7700x_clk_pll_ops, &pll, sizeof(pll));
        break;

      case EIC7700X_CLKTYPE_FACTOR:
        clk = clk_register_fixed_factor(desc->name, desc->parent,
                                        desc->cflags,
                                        (uint8_t)(desc->arg >> 8),
                                        (uint8_t)(desc->arg & 0xff));
        break;

      case EIC7700X_CLKTYPE_MUX:
        mux.reg      = desc->reg;
        mux.table    = desc->table;
        mux.shift    = desc->shift;
        mux.width    = desc->width;
        mux.nparents = desc->nparents;
        clk = clk_register(desc->name, desc->parents, desc->nparents,
                           desc->cflags, &g_eic7700x_clk_mux_ops,
                           &mux, sizeof(mux));
        break;

      case EIC7700X_CLKTYPE_DIV:
        divider.reg    = desc->reg;
        divider.shift  = desc->shift;
        divider.width  = desc->width;
        divider.lowdiv = desc->arg;
        clk = clk_register(desc->name, &desc->parent, 1, desc->cflags,
                           &g_eic7700x_clk_div_ops, &divider,
                           sizeof(divider));
        break;

      case EIC7700X_CLKTYPE_GATE:
        clk = clk_register_gate(desc->name, desc->parent, desc->cflags,
                                desc->reg, desc->bit, 0);
        break;

      default:
        return -EINVAL;
    }

  /* A NULL return also covers a duplicate name, which the framework
   * rejects without any other diagnostic.
   */

  if (clk == NULL)
    {
      syslog(LOG_ERR, "clk: %s: registration failed\n", desc->name);
      return -ENOMEM;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: eic7700x_clk_initialize
 *
 * Description:
 *   Register the EIC7700X Clock and Reset Generator clock tree with the
 *   clock framework.  See eic7700x_clk.h.
 *
 ****************************************************************************/

/* What eic7700x_clk_initialize() registered, for the board to report */

static unsigned int g_eic7700x_clk_count;
static unsigned int g_eic7700x_clk_failed;

int eic7700x_clk_initialize(void)
{
  size_t i;
  int nfail = 0;

  for (i = 0; i < nitems(g_eic7700x_clks); i++)
    {
      if (eic7700x_clk_register_one(&g_eic7700x_clks[i]) < 0)
        {
          nfail++;
        }
    }

#ifdef CONFIG_DEBUG_ASSERTIONS
  /* A row naming a parent that never registered leaves a permanent
   * orphan whose rate silently reads back as zero, so a mistyped parent
   * name is worth catching loudly rather than debugging on hardware.
   */

  for (i = 0; i < nitems(g_eic7700x_clks); i++)
    {
      FAR const struct eic7700x_clkdesc_s *desc = &g_eic7700x_clks[i];
      FAR struct clk_s *clk;

      if (desc->parent == NULL && desc->parents == NULL)
        {
          continue;
        }

      clk = clk_get(desc->name);
      if (clk == NULL || clk_get_parent(clk) == NULL)
        {
          syslog(LOG_ERR, "clk: %s: orphaned, check the parent name\n",
                 desc->name);
          DEBUGPANIC();
        }
    }
#endif

  g_eic7700x_clk_count  = nitems(g_eic7700x_clks) - nfail;
  g_eic7700x_clk_failed = nfail;

  return nfail == 0 ? OK : -EIO;
}

/****************************************************************************
 * Name: eic7700x_clk_count
 *
 * Description:
 *   What eic7700x_clk_initialize() registered.  See eic7700x_clk.h.
 *
 ****************************************************************************/

unsigned int eic7700x_clk_count(FAR unsigned int *failed)
{
  if (failed != NULL)
    {
      *failed = g_eic7700x_clk_failed;
    }

  return g_eic7700x_clk_count;
}
