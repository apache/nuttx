/****************************************************************************
 * arch/risc-v/src/eic7700x/eic7700x_reset.c
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
#include <stdbool.h>
#include <inttypes.h>
#include <assert.h>
#include <sys/param.h>

#include <nuttx/debug.h>
#include <nuttx/reset/reset-controller.h>
#ifdef CONFIG_RESET_PROCFS
#  include <stdio.h>
#  include <nuttx/fs/procfs.h>
#endif

#include "riscv_internal.h"
#include "eic7700x_reset.h"
#include "hardware/eic7700x_reset.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* One table row.  Writing the register index into the designated
 * initialiser keeps a row's position in the array visibly equal to the
 * register index that a reset line id carries in its upper bits.
 */

#define RSTREG(i, v, r, c) \
  [i] = \
  { \
    .valid = (v), .rdonly = (r), .critical = (c) \
  }

/* System critical means a line whose assertion takes down the system that
 * is asserting it: the fabric that carries instruction fetch, the memory
 * the kernel runs from, the cluster it runs on, the configuration path
 * back to this very register block, and the pipeline and translation
 * buffer cells inside those paths.  Those lines are registered so that a
 * driver can still release them and read their state, but assert and
 * reset return -EPERM.
 *
 * The line is drawn at "will hang", not "might upset something".  Section
 * 3.1 of the manual warns that resetting any block that is not idle can
 * hang the bus, which is true of every line in the table and so cannot be
 * the criterion here.  Defining each critical mask in terms of the
 * matching valid mask keeps the two in step by construction.
 */

#define SNOC_RST_CRIT              SNOC_RST_VALID
#define DDR_RST_CRIT               DDR_RST_VALID
#define TCU_RST_CRIT               TCU_RST_VALID
#define LSPCFG_RST_CRIT            LSPCFG_RST_VALID
#define U84_RST_CRIT               U84_RST_VALID
#define MNOC_RST_CRIT              MNOC_RST_VALID
#define RNOC_RST_CRIT              RNOC_RST_VALID
#define CNOC_RST_CRIT              CNOC_RST_VALID
#define LNOC_RST_CRIT              LNOC_RST_VALID
#define PIPE_RST_CRIT              PIPE_RST_VALID
#define TBU_RST_CRIT               TBU_RST_VALID

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The reset controller.  The framework does not allocate this and there is
 * no per instance state to carry: the register base is a compile time
 * constant and everything else falls out of the line id, so a file scope
 * object is all that is needed and no container_of is involved.
 */

static struct reset_controller_dev g_eic7700x_rcdev =
{
  .name = EIC7700X_RESET_CONTROLLER,
  .ops  = &g_eic7700x_reset_ops,
#ifdef CONFIG_RESET_PROCFS
  /* The id space, not the line count: an id is a register index times
   * thirty two plus a bit, and most of it names nothing.
   */

  .nlines = EIC7700X_RESET_ID(EIC7700X_RESET_NREGS, 0),
#endif
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Which bits of each control register name a reset line, which of those
 * the hardware will not let software drive, and which of them refuse to be
 * asserted.  Rows not listed are all zero, so every id in them is
 * rejected, which is what the registers this SoC does not have resolve to.
 */

const struct eic7700x_reset_reg_s
g_eic7700x_reset_regs[EIC7700X_RESET_NREGS] =
{
  RSTREG(0x00, SNOC_RST_VALID,   0,                SNOC_RST_CRIT),
  RSTREG(0x01, GPU_RST_VALID,    0,                0),
  RSTREG(0x02, DSP_RST_VALID,    0,                0),
  RSTREG(0x03, D2D_RST_VALID,    0,                0),
  RSTREG(0x04, DDR_RST_VALID,    0,                DDR_RST_CRIT),
  RSTREG(0x05, TCU_RST_VALID,    0,                TCU_RST_CRIT),
  RSTREG(0x06, NPU_RST_VALID,    0,                0),
  RSTREG(0x07, HSPDMA_RST_VALID, 0,                0),
  RSTREG(0x08, PCIE_RST_VALID,   0,                0),
  RSTREG(0x09, I2C_RST_VALID,    0,                0),
  RSTREG(0x0a, FAN_RST_VALID,    0,                0),
  RSTREG(0x0b, PVT_RST_VALID,    0,                0),
  RSTREG(0x0c, MBOX_RST_VALID,   0,                0),
  RSTREG(0x0d, UART_RST_VALID,   0,                0),
  RSTREG(0x0e, GPIO_RST_VALID,   0,                0),
  RSTREG(0x0f, TIMER_RST_VALID,  0,                0),
  RSTREG(0x10, SSI_RST_VALID,    0,                0),
  RSTREG(0x11, WDT_RST_VALID,    0,                0),
  RSTREG(0x12, LSPCFG_RST_VALID, 0,                LSPCFG_RST_CRIT),
  RSTREG(0x13, U84_RST_VALID,    0,                U84_RST_CRIT),
  RSTREG(0x14, SCPU_RST_VALID,   0,                0),
  RSTREG(0x15, LPCPU_RST_VALID,  0,                0),
  RSTREG(0x16, VC_RST_VALID,     0,                0),
  RSTREG(0x17, JD_RST_VALID,     0,                0),
  RSTREG(0x18, JE_RST_VALID,     0,                0),
  RSTREG(0x19, VD_RST_VALID,     0,                0),
  RSTREG(0x1a, VE_RST_VALID,     0,                0),
  RSTREG(0x1b, G2D_RST_VALID,    0,                0),
  RSTREG(0x1c, VI_RST_VALID,     0,                0),
  RSTREG(0x1d, DVP_RST_VALID,    0,                0),
  RSTREG(0x1e, ISP0_RST_VALID,   0,                0),
  RSTREG(0x1f, ISP1_RST_VALID,   0,                0),
  RSTREG(0x20, SHUTTER_RST_VALID, 0,               0),
  RSTREG(0x21, VOPHY_RST_VALID,  0,                0),
  RSTREG(0x22, VOI2S_RST_VALID,  0,                0),
  RSTREG(0x23, VO_RST_VALID,     0,                0),
  RSTREG(0x24, BOOTSPI_RST_VALID, 0,               0),
  RSTREG(0x25, I2C1_RST_VALID,   0,                0),
  RSTREG(0x26, I2C0_RST_VALID,   0,                0),
  RSTREG(0x27, DMA1_RST_VALID,   0,                0),
  RSTREG(0x28, FPRT_RST_VALID,   0,                0),
  RSTREG(0x29, HBLOCK_RST_VALID, 0,                0),
  RSTREG(0x2a, SECSR_RST_VALID,  0,                0),
  RSTREG(0x2b, OTP_RST_VALID,    0,                0),
  RSTREG(0x2c, PKA_RST_VALID,    0,                0),
  RSTREG(0x2d, SPACC_RST_VALID,  0,                0),
  RSTREG(0x2e, TRNG_RST_VALID,   0,                0),

  /* 0x4bc is reserved.  The manual documents nothing at this offset
   * and the vendor Linux binding names the index RESERVED, so the row
   * is written out explicitly rather than left to the zero fill: any
   * id landing in it is rejected, and the gap is visible in the table.
   */

  RSTREG(0x2f, 0,                0,                0),

  RSTREG(0x30, TIMER0_RST_VALID, 0,                0),
  RSTREG(0x31, TIMER1_RST_VALID, 0,                0),
  RSTREG(0x32, TIMER2_RST_VALID, 0,                0),
  RSTREG(0x33, TIMER3_RST_VALID, 0,                0),
  RSTREG(0x34, RTC_RST_VALID,    0,                0),
  RSTREG(0x35, MNOC_RST_VALID,   0,                MNOC_RST_CRIT),
  RSTREG(0x36, RNOC_RST_VALID,   0,                RNOC_RST_CRIT),
  RSTREG(0x37, CNOC_RST_VALID,   0,                CNOC_RST_CRIT),
  RSTREG(0x38, LNOC_RST_VALID,   0,                LNOC_RST_CRIT),
  RSTREG(0x39, PIPE_RST_VALID,   0,                PIPE_RST_CRIT),
  RSTREG(0x3a, TBU_RST_VALID,    0,                TBU_RST_CRIT),
  RSTREG(0x3b, STATUS_RST_VALID, STATUS_RST_VALID, 0),
  RSTREG(0x3c, TEST_RST_VALID,   0,                0),
};

/* The table has to describe exactly the id space the public header
 * promises, and the last row has to land on the last control register.
 */

static_assert(nitems(g_eic7700x_reset_regs) == EIC7700X_RESET_NREGS,
              "reset table length must match the id space bound");
static_assert(EIC7700X_RESET_CTRL(EIC7700X_RESET_NREGS - 1) ==
              EIC7700X_RESET_BASE + 0x00f0,
              "the last reset control register is at offset 0x4f0");

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_RESET_PROCFS

/* One reset line and the manual's name for it.  Only built when the procfs
 * entry is, since nothing else needs a line to have a name at run time: the
 * enumeration carries the names everywhere else and does not survive
 * compilation.
 */

struct eic7700x_reset_name_s
{
  FAR const char *name;
  uint16_t id;
};

#define RSTNAME(i, n) \
  { \
    .name = (n), .id = (i) \
  }

static const struct eic7700x_reset_name_s g_eic7700x_reset_names[] =
{
  RSTNAME(EIC7700X_RESET_NOC_NSP,    "noc_nsp"),
  RSTNAME(EIC7700X_RESET_NOC_CFG,    "noc_cfg"),
  RSTNAME(EIC7700X_RESET_RNOC_NSP,   "rnoc_nsp"),
  RSTNAME(EIC7700X_RESET_SNOC_TCU_A, "snoc_tcu_a"),
  RSTNAME(EIC7700X_RESET_SNOC_U84_A, "snoc_u84_a"),
  RSTNAME(EIC7700X_RESET_SNOC_PCIET_XS, "snoc_pciet_xs"),
  RSTNAME(EIC7700X_RESET_SNOC_PCIET_XM, "snoc_pciet_xm"),
  RSTNAME(EIC7700X_RESET_SNOC_PCIET_P, "snoc_pciet_p"),
  RSTNAME(EIC7700X_RESET_SNOC_NPU_A, "snoc_npu_a"),
  RSTNAME(EIC7700X_RESET_SNOC_JTAG_P, "snoc_jtag_p"),
  RSTNAME(EIC7700X_RESET_SNOC_DSPT_A, "snoc_dspt_a"),
  RSTNAME(EIC7700X_RESET_SNOC_DDRC1_P2_A, "snoc_ddrc1_p2_a"),
  RSTNAME(EIC7700X_RESET_SNOC_DDRC1_P1_A, "snoc_ddrc1_p1_a"),
  RSTNAME(EIC7700X_RESET_SNOC_DDRC0_P2_A, "snoc_ddrc0_p2_a"),
  RSTNAME(EIC7700X_RESET_SNOC_DDRC0_P1_A, "snoc_ddrc0_p1_a"),
  RSTNAME(EIC7700X_RESET_SNOC_D2D_A, "snoc_d2d_a"),
  RSTNAME(EIC7700X_RESET_SNOC_AON_A, "snoc_aon_a"),
  RSTNAME(EIC7700X_RESET_GPU_AXI,    "gpu_axi"),
  RSTNAME(EIC7700X_RESET_GPU_CFG,    "gpu_cfg"),
  RSTNAME(EIC7700X_RESET_GPU_GRAY,   "gpu_gray"),
  RSTNAME(EIC7700X_RESET_GPU_JONES,  "gpu_jones"),
  RSTNAME(EIC7700X_RESET_GPU_SPU,    "gpu_spu"),
  RSTNAME(EIC7700X_RESET_DSP_AXI,    "dsp_axi"),
  RSTNAME(EIC7700X_RESET_DSP_CFG,    "dsp_cfg"),
  RSTNAME(EIC7700X_RESET_DSP_DIV4,   "dsp_div4"),
  RSTNAME(EIC7700X_RESET_DSP_DIV0,   "dsp_div0"),
  RSTNAME(EIC7700X_RESET_DSP_DIV1,   "dsp_div1"),
  RSTNAME(EIC7700X_RESET_DSP_DIV2,   "dsp_div2"),
  RSTNAME(EIC7700X_RESET_DSP_DIV3,   "dsp_div3"),
  RSTNAME(EIC7700X_RESET_D2D_AXI,    "d2d_axi"),
  RSTNAME(EIC7700X_RESET_D2D_CFG,    "d2d_cfg"),
  RSTNAME(EIC7700X_RESET_D2D_P,      "d2d_p"),
  RSTNAME(EIC7700X_RESET_D2D_RAW_PCS, "d2d_raw_pcs"),
  RSTNAME(EIC7700X_RESET_D2D_RX,     "d2d_rx"),
  RSTNAME(EIC7700X_RESET_D2D_TX,     "d2d_tx"),
  RSTNAME(EIC7700X_RESET_D2D_CORE,   "d2d_core"),
  RSTNAME(EIC7700X_RESET_D2D_SUBSYS, "d2d_subsys"),
  RSTNAME(EIC7700X_RESET_DDR1_P0_A,  "ddr1_p0_a"),
  RSTNAME(EIC7700X_RESET_DDR1_P1_A,  "ddr1_p1_a"),
  RSTNAME(EIC7700X_RESET_DDR1_P2_A,  "ddr1_p2_a"),
  RSTNAME(EIC7700X_RESET_DDR1_P3_A,  "ddr1_p3_a"),
  RSTNAME(EIC7700X_RESET_DDR1_P4_A,  "ddr1_p4_a"),
  RSTNAME(EIC7700X_RESET_DDR1_TRACE_A, "ddr1_trace_a"),
  RSTNAME(EIC7700X_RESET_DDR1_PMP_SOFT, "ddr1_pmp_soft"),
  RSTNAME(EIC7700X_RESET_DDR0_P0_A,  "ddr0_p0_a"),
  RSTNAME(EIC7700X_RESET_DDR0_P1_A,  "ddr0_p1_a"),
  RSTNAME(EIC7700X_RESET_DDR0_P2_A,  "ddr0_p2_a"),
  RSTNAME(EIC7700X_RESET_DDR0_P3_A,  "ddr0_p3_a"),
  RSTNAME(EIC7700X_RESET_DDR0_P4_A,  "ddr0_p4_a"),
  RSTNAME(EIC7700X_RESET_DDR_CFG,    "ddr_cfg"),
  RSTNAME(EIC7700X_RESET_DDR0_TRACE_A, "ddr0_trace_a"),
  RSTNAME(EIC7700X_RESET_DDR_CORE,   "ddr_core"),
  RSTNAME(EIC7700X_RESET_DDR0_PMP_SOFT, "ddr0_pmp_soft"),
  RSTNAME(EIC7700X_RESET_DDR_P,      "ddr_p"),
  RSTNAME(EIC7700X_RESET_TCU_AXI,    "tcu_axi"),
  RSTNAME(EIC7700X_RESET_TCU_CFG,    "tcu_cfg"),
  RSTNAME(EIC7700X_RESET_TCU_TBU0,   "tcu_tbu0"),
  RSTNAME(EIC7700X_RESET_TCU_TBU1,   "tcu_tbu1"),
  RSTNAME(EIC7700X_RESET_TCU_TBU2,   "tcu_tbu2"),
  RSTNAME(EIC7700X_RESET_TCU_TBU3,   "tcu_tbu3"),
  RSTNAME(EIC7700X_RESET_TCU_TBU4,   "tcu_tbu4"),
  RSTNAME(EIC7700X_RESET_TCU_TBU5,   "tcu_tbu5"),
  RSTNAME(EIC7700X_RESET_TCU_TBU6,   "tcu_tbu6"),
  RSTNAME(EIC7700X_RESET_TCU_TBU7,   "tcu_tbu7"),
  RSTNAME(EIC7700X_RESET_TCU_TBU8,   "tcu_tbu8"),
  RSTNAME(EIC7700X_RESET_TCU_TBU9,   "tcu_tbu9"),
  RSTNAME(EIC7700X_RESET_TCU_TBU10,  "tcu_tbu10"),
  RSTNAME(EIC7700X_RESET_TCU_TBU11,  "tcu_tbu11"),
  RSTNAME(EIC7700X_RESET_TCU_TBU12,  "tcu_tbu12"),
  RSTNAME(EIC7700X_RESET_TCU_TBU13,  "tcu_tbu13"),
  RSTNAME(EIC7700X_RESET_TCU_TBU14,  "tcu_tbu14"),
  RSTNAME(EIC7700X_RESET_TCU_TBU15,  "tcu_tbu15"),
  RSTNAME(EIC7700X_RESET_TCU_TBU16,  "tcu_tbu16"),
  RSTNAME(EIC7700X_RESET_NPU_AXI,    "npu_axi"),
  RSTNAME(EIC7700X_RESET_NPU_CFG,    "npu_cfg"),
  RSTNAME(EIC7700X_RESET_NPU_CORE,   "npu_core"),
  RSTNAME(EIC7700X_RESET_NPU_E31CORE, "npu_e31core"),
  RSTNAME(EIC7700X_RESET_NPU_E31BUS, "npu_e31bus"),
  RSTNAME(EIC7700X_RESET_NPU_E31DBG, "npu_e31dbg"),
  RSTNAME(EIC7700X_RESET_NPU_LLC,    "npu_llc"),
  RSTNAME(EIC7700X_RESET_HSP_AXI,    "hsp_axi"),
  RSTNAME(EIC7700X_RESET_HSP_CFG,    "hsp_cfg"),
  RSTNAME(EIC7700X_RESET_HSP_POR,    "hsp_por"),
  RSTNAME(EIC7700X_RESET_MSHC0_PHY,  "mshc0_phy"),
  RSTNAME(EIC7700X_RESET_MSHC1_PHY,  "mshc1_phy"),
  RSTNAME(EIC7700X_RESET_MSHC2_PHY,  "mshc2_phy"),
  RSTNAME(EIC7700X_RESET_MSHC0_TXRX, "mshc0_txrx"),
  RSTNAME(EIC7700X_RESET_MSHC1_TXRX, "mshc1_txrx"),
  RSTNAME(EIC7700X_RESET_MSHC2_TXRX, "mshc2_txrx"),
  RSTNAME(EIC7700X_RESET_SATA_ASIC0, "sata_asic0"),
  RSTNAME(EIC7700X_RESET_SATA_OOB,   "sata_oob"),
  RSTNAME(EIC7700X_RESET_SATA_PMALIVE, "sata_pmalive"),
  RSTNAME(EIC7700X_RESET_SATA_RBC,   "sata_rbc"),
  RSTNAME(EIC7700X_RESET_DMA0,       "dma0"),
  RSTNAME(EIC7700X_RESET_HSP_DMA0,   "hsp_dma0"),
  RSTNAME(EIC7700X_RESET_USB0_VAUX,  "usb0_vaux"),
  RSTNAME(EIC7700X_RESET_USB1_VAUX,  "usb1_vaux"),
  RSTNAME(EIC7700X_RESET_HSP_SD1_P,  "hsp_sd1_p"),
  RSTNAME(EIC7700X_RESET_HSP_SD0_P,  "hsp_sd0_p"),
  RSTNAME(EIC7700X_RESET_HSP_EMMC_P, "hsp_emmc_p"),
  RSTNAME(EIC7700X_RESET_HSP_DMA_P,  "hsp_dma_p"),
  RSTNAME(EIC7700X_RESET_HSP_SD1_A,  "hsp_sd1_a"),
  RSTNAME(EIC7700X_RESET_HSP_SD0_A,  "hsp_sd0_a"),
  RSTNAME(EIC7700X_RESET_HSP_EMMC_A, "hsp_emmc_a"),
  RSTNAME(EIC7700X_RESET_HSP_DMA_A,  "hsp_dma_a"),
  RSTNAME(EIC7700X_RESET_HSP_ETH1_A, "hsp_eth1_a"),
  RSTNAME(EIC7700X_RESET_HSP_ETH0_A, "hsp_eth0_a"),
  RSTNAME(EIC7700X_RESET_HSP_SATA_A, "hsp_sata_a"),
  RSTNAME(EIC7700X_RESET_PCIE_CFG,   "pcie_cfg"),
  RSTNAME(EIC7700X_RESET_PCIE_POWERUP, "pcie_powerup"),
  RSTNAME(EIC7700X_RESET_PCIE_PERST, "pcie_perst"),
  RSTNAME(EIC7700X_RESET_I2C0,       "i2c0"),
  RSTNAME(EIC7700X_RESET_I2C1,       "i2c1"),
  RSTNAME(EIC7700X_RESET_I2C2,       "i2c2"),
  RSTNAME(EIC7700X_RESET_I2C3,       "i2c3"),
  RSTNAME(EIC7700X_RESET_I2C4,       "i2c4"),
  RSTNAME(EIC7700X_RESET_I2C5,       "i2c5"),
  RSTNAME(EIC7700X_RESET_I2C6,       "i2c6"),
  RSTNAME(EIC7700X_RESET_I2C7,       "i2c7"),
  RSTNAME(EIC7700X_RESET_I2C8,       "i2c8"),
  RSTNAME(EIC7700X_RESET_I2C9,       "i2c9"),
  RSTNAME(EIC7700X_RESET_FAN,        "fan"),
  RSTNAME(EIC7700X_RESET_PVT_LSP,    "pvt_lsp"),
  RSTNAME(EIC7700X_RESET_PVT_DDR,    "pvt_ddr"),
  RSTNAME(EIC7700X_RESET_MBOX0,      "mbox0"),
  RSTNAME(EIC7700X_RESET_MBOX1,      "mbox1"),
  RSTNAME(EIC7700X_RESET_MBOX2,      "mbox2"),
  RSTNAME(EIC7700X_RESET_MBOX3,      "mbox3"),
  RSTNAME(EIC7700X_RESET_MBOX4,      "mbox4"),
  RSTNAME(EIC7700X_RESET_MBOX5,      "mbox5"),
  RSTNAME(EIC7700X_RESET_MBOX6,      "mbox6"),
  RSTNAME(EIC7700X_RESET_MBOX7,      "mbox7"),
  RSTNAME(EIC7700X_RESET_MBOX8,      "mbox8"),
  RSTNAME(EIC7700X_RESET_MBOX9,      "mbox9"),
  RSTNAME(EIC7700X_RESET_MBOX10,     "mbox10"),
  RSTNAME(EIC7700X_RESET_MBOX11,     "mbox11"),
  RSTNAME(EIC7700X_RESET_MBOX12,     "mbox12"),
  RSTNAME(EIC7700X_RESET_MBOX13,     "mbox13"),
  RSTNAME(EIC7700X_RESET_MBOX14,     "mbox14"),
  RSTNAME(EIC7700X_RESET_MBOX15,     "mbox15"),
  RSTNAME(EIC7700X_RESET_UART0,      "uart0"),
  RSTNAME(EIC7700X_RESET_UART1,      "uart1"),
  RSTNAME(EIC7700X_RESET_UART2,      "uart2"),
  RSTNAME(EIC7700X_RESET_UART3,      "uart3"),
  RSTNAME(EIC7700X_RESET_UART4,      "uart4"),
  RSTNAME(EIC7700X_RESET_GPIO0,      "gpio0"),
  RSTNAME(EIC7700X_RESET_GPIO1,      "gpio1"),
  RSTNAME(EIC7700X_RESET_LSP_TIMER,  "lsp_timer"),
  RSTNAME(EIC7700X_RESET_SSI0,       "ssi0"),
  RSTNAME(EIC7700X_RESET_SSI1,       "ssi1"),
  RSTNAME(EIC7700X_RESET_WDT0,       "wdt0"),
  RSTNAME(EIC7700X_RESET_WDT1,       "wdt1"),
  RSTNAME(EIC7700X_RESET_WDT2,       "wdt2"),
  RSTNAME(EIC7700X_RESET_WDT3,       "wdt3"),
  RSTNAME(EIC7700X_RESET_LSP_CFG,    "lsp_cfg"),
  RSTNAME(EIC7700X_RESET_U84_CORE0,  "u84_core0"),
  RSTNAME(EIC7700X_RESET_U84_CORE1,  "u84_core1"),
  RSTNAME(EIC7700X_RESET_U84_CORE2,  "u84_core2"),
  RSTNAME(EIC7700X_RESET_U84_CORE3,  "u84_core3"),
  RSTNAME(EIC7700X_RESET_U84_BUS,    "u84_bus"),
  RSTNAME(EIC7700X_RESET_U84_DBG,    "u84_dbg"),
  RSTNAME(EIC7700X_RESET_U84_TRACECOM, "u84_tracecom"),
  RSTNAME(EIC7700X_RESET_U84_TRACE0, "u84_trace0"),
  RSTNAME(EIC7700X_RESET_U84_TRACE1, "u84_trace1"),
  RSTNAME(EIC7700X_RESET_U84_TRACE2, "u84_trace2"),
  RSTNAME(EIC7700X_RESET_U84_TRACE3, "u84_trace3"),
  RSTNAME(EIC7700X_RESET_SCPU_CORE,  "scpu_core"),
  RSTNAME(EIC7700X_RESET_SCPU_BUS,   "scpu_bus"),
  RSTNAME(EIC7700X_RESET_SCPU_DBG,   "scpu_dbg"),
  RSTNAME(EIC7700X_RESET_LPCPU_CORE, "lpcpu_core"),
  RSTNAME(EIC7700X_RESET_LPCPU_BUS,  "lpcpu_bus"),
  RSTNAME(EIC7700X_RESET_LPCPU_DBG,  "lpcpu_dbg"),
  RSTNAME(EIC7700X_RESET_VC_CFG,     "vc_cfg"),
  RSTNAME(EIC7700X_RESET_VC_AXI,     "vc_axi"),
  RSTNAME(EIC7700X_RESET_VC_MONCFG,  "vc_moncfg"),
  RSTNAME(EIC7700X_RESET_JD_CFG,     "jd_cfg"),
  RSTNAME(EIC7700X_RESET_JD_AXI,     "jd_axi"),
  RSTNAME(EIC7700X_RESET_JE_CFG,     "je_cfg"),
  RSTNAME(EIC7700X_RESET_JE_AXI,     "je_axi"),
  RSTNAME(EIC7700X_RESET_VD_CFG,     "vd_cfg"),
  RSTNAME(EIC7700X_RESET_VD_AXI,     "vd_axi"),
  RSTNAME(EIC7700X_RESET_VE_CFG,     "ve_cfg"),
  RSTNAME(EIC7700X_RESET_VE_AXI,     "ve_axi"),
  RSTNAME(EIC7700X_RESET_G2D_CORE,   "g2d_core"),
  RSTNAME(EIC7700X_RESET_G2D_CFG,    "g2d_cfg"),
  RSTNAME(EIC7700X_RESET_G2D_AXI,    "g2d_axi"),
  RSTNAME(EIC7700X_RESET_VI_AXI,     "vi_axi"),
  RSTNAME(EIC7700X_RESET_VI_CFG,     "vi_cfg"),
  RSTNAME(EIC7700X_RESET_VI_DWE,     "vi_dwe"),
  RSTNAME(EIC7700X_RESET_VI_DVP,     "vi_dvp"),
  RSTNAME(EIC7700X_RESET_VI_ISP0,    "vi_isp0"),
  RSTNAME(EIC7700X_RESET_VI_ISP1,    "vi_isp1"),
  RSTNAME(EIC7700X_RESET_VI_SHUTTER0, "vi_shutter0"),
  RSTNAME(EIC7700X_RESET_VI_SHUTTER1, "vi_shutter1"),
  RSTNAME(EIC7700X_RESET_VI_SHUTTER2, "vi_shutter2"),
  RSTNAME(EIC7700X_RESET_VI_SHUTTER3, "vi_shutter3"),
  RSTNAME(EIC7700X_RESET_VI_SHUTTER4, "vi_shutter4"),
  RSTNAME(EIC7700X_RESET_VI_SHUTTER5, "vi_shutter5"),
  RSTNAME(EIC7700X_RESET_VO_MIPI_P,  "vo_mipi_p"),
  RSTNAME(EIC7700X_RESET_VO_P,       "vo_p"),
  RSTNAME(EIC7700X_RESET_VO_HDMI_P,  "vo_hdmi_p"),
  RSTNAME(EIC7700X_RESET_HDMI_PHYCTRL, "hdmi_phyctrl"),
  RSTNAME(EIC7700X_RESET_VO_HDMI,    "vo_hdmi"),
  RSTNAME(EIC7700X_RESET_VO_I2S,     "vo_i2s"),
  RSTNAME(EIC7700X_RESET_VO_I2S_P,   "vo_i2s_p"),
  RSTNAME(EIC7700X_RESET_VO_AXI,     "vo_axi"),
  RSTNAME(EIC7700X_RESET_VO_CFG,     "vo_cfg"),
  RSTNAME(EIC7700X_RESET_VO_DC,      "vo_dc"),
  RSTNAME(EIC7700X_RESET_VO_DC_P,    "vo_dc_p"),
  RSTNAME(EIC7700X_RESET_BOOTSPI_H,  "bootspi_h"),
  RSTNAME(EIC7700X_RESET_BOOTSPI,    "bootspi"),
  RSTNAME(EIC7700X_RESET_I2C1_P,     "i2c1_p"),
  RSTNAME(EIC7700X_RESET_I2C0_P,     "i2c0_p"),
  RSTNAME(EIC7700X_RESET_DMA1_A,     "dma1_a"),
  RSTNAME(EIC7700X_RESET_DMA1_H,     "dma1_h"),
  RSTNAME(EIC7700X_RESET_FPRT_H,     "fprt_h"),
  RSTNAME(EIC7700X_RESET_HBLOCK_H,   "hblock_h"),
  RSTNAME(EIC7700X_RESET_SECSR_H,    "secsr_h"),
  RSTNAME(EIC7700X_RESET_OTP_P,      "otp_p"),
  RSTNAME(EIC7700X_RESET_PKA_H,      "pka_h"),
  RSTNAME(EIC7700X_RESET_SPACC,      "spacc"),
  RSTNAME(EIC7700X_RESET_TRNG_H,     "trng_h"),
  RSTNAME(EIC7700X_RESET_TIMER0_CNT0, "timer0_cnt0"),
  RSTNAME(EIC7700X_RESET_TIMER0_CNT1, "timer0_cnt1"),
  RSTNAME(EIC7700X_RESET_TIMER0_CNT2, "timer0_cnt2"),
  RSTNAME(EIC7700X_RESET_TIMER0_CNT3, "timer0_cnt3"),
  RSTNAME(EIC7700X_RESET_TIMER0_CNT4, "timer0_cnt4"),
  RSTNAME(EIC7700X_RESET_TIMER0_CNT5, "timer0_cnt5"),
  RSTNAME(EIC7700X_RESET_TIMER0_CNT6, "timer0_cnt6"),
  RSTNAME(EIC7700X_RESET_TIMER0_CNT7, "timer0_cnt7"),
  RSTNAME(EIC7700X_RESET_TIMER0_P,   "timer0_p"),
  RSTNAME(EIC7700X_RESET_TIMER1_CNT0, "timer1_cnt0"),
  RSTNAME(EIC7700X_RESET_TIMER1_CNT1, "timer1_cnt1"),
  RSTNAME(EIC7700X_RESET_TIMER1_CNT2, "timer1_cnt2"),
  RSTNAME(EIC7700X_RESET_TIMER1_CNT3, "timer1_cnt3"),
  RSTNAME(EIC7700X_RESET_TIMER1_CNT4, "timer1_cnt4"),
  RSTNAME(EIC7700X_RESET_TIMER1_CNT5, "timer1_cnt5"),
  RSTNAME(EIC7700X_RESET_TIMER1_CNT6, "timer1_cnt6"),
  RSTNAME(EIC7700X_RESET_TIMER1_CNT7, "timer1_cnt7"),
  RSTNAME(EIC7700X_RESET_TIMER1_P,   "timer1_p"),
  RSTNAME(EIC7700X_RESET_TIMER2_CNT0, "timer2_cnt0"),
  RSTNAME(EIC7700X_RESET_TIMER2_CNT1, "timer2_cnt1"),
  RSTNAME(EIC7700X_RESET_TIMER2_CNT2, "timer2_cnt2"),
  RSTNAME(EIC7700X_RESET_TIMER2_CNT3, "timer2_cnt3"),
  RSTNAME(EIC7700X_RESET_TIMER2_CNT4, "timer2_cnt4"),
  RSTNAME(EIC7700X_RESET_TIMER2_CNT5, "timer2_cnt5"),
  RSTNAME(EIC7700X_RESET_TIMER2_CNT6, "timer2_cnt6"),
  RSTNAME(EIC7700X_RESET_TIMER2_CNT7, "timer2_cnt7"),
  RSTNAME(EIC7700X_RESET_TIMER2_P,   "timer2_p"),
  RSTNAME(EIC7700X_RESET_TIMER3_CNT0, "timer3_cnt0"),
  RSTNAME(EIC7700X_RESET_TIMER3_CNT1, "timer3_cnt1"),
  RSTNAME(EIC7700X_RESET_TIMER3_CNT2, "timer3_cnt2"),
  RSTNAME(EIC7700X_RESET_TIMER3_CNT3, "timer3_cnt3"),
  RSTNAME(EIC7700X_RESET_TIMER3_CNT4, "timer3_cnt4"),
  RSTNAME(EIC7700X_RESET_TIMER3_CNT5, "timer3_cnt5"),
  RSTNAME(EIC7700X_RESET_TIMER3_CNT6, "timer3_cnt6"),
  RSTNAME(EIC7700X_RESET_TIMER3_CNT7, "timer3_cnt7"),
  RSTNAME(EIC7700X_RESET_TIMER3_P,   "timer3_p"),
  RSTNAME(EIC7700X_RESET_RTC,        "rtc"),
  RSTNAME(EIC7700X_RESET_MNOC_SNOC_NSP, "mnoc_snoc_nsp"),
  RSTNAME(EIC7700X_RESET_MNOC_VC_A,  "mnoc_vc_a"),
  RSTNAME(EIC7700X_RESET_MNOC_CFG,   "mnoc_cfg"),
  RSTNAME(EIC7700X_RESET_MNOC_HSP_A, "mnoc_hsp_a"),
  RSTNAME(EIC7700X_RESET_MNOC_GPU_A, "mnoc_gpu_a"),
  RSTNAME(EIC7700X_RESET_MNOC_DDRC1_P3_A, "mnoc_ddrc1_p3_a"),
  RSTNAME(EIC7700X_RESET_MNOC_DDRC0_P3_A, "mnoc_ddrc0_p3_a"),
  RSTNAME(EIC7700X_RESET_RNOC_VO_A,  "rnoc_vo_a"),
  RSTNAME(EIC7700X_RESET_RNOC_VI_A,  "rnoc_vi_a"),
  RSTNAME(EIC7700X_RESET_RNOC_SNOC_NSP, "rnoc_snoc_nsp"),
  RSTNAME(EIC7700X_RESET_RNOC_CFG,   "rnoc_cfg"),
  RSTNAME(EIC7700X_RESET_RNOC_DDRC1_P4_A, "rnoc_ddrc1_p4_a"),
  RSTNAME(EIC7700X_RESET_RNOC_DDRC0_P4_A, "rnoc_ddrc0_p4_a"),
  RSTNAME(EIC7700X_RESET_CNOC_VO_CFG, "cnoc_vo_cfg"),
  RSTNAME(EIC7700X_RESET_CNOC_VI_CFG, "cnoc_vi_cfg"),
  RSTNAME(EIC7700X_RESET_CNOC_VC_CFG, "cnoc_vc_cfg"),
  RSTNAME(EIC7700X_RESET_CNOC_TCU_CFG, "cnoc_tcu_cfg"),
  RSTNAME(EIC7700X_RESET_CNOC_PCIET_CFG, "cnoc_pciet_cfg"),
  RSTNAME(EIC7700X_RESET_CNOC_NPU_CFG, "cnoc_npu_cfg"),
  RSTNAME(EIC7700X_RESET_CNOC_LSP_CFG, "cnoc_lsp_cfg"),
  RSTNAME(EIC7700X_RESET_CNOC_HSP_CFG, "cnoc_hsp_cfg"),
  RSTNAME(EIC7700X_RESET_CNOC_GPU_CFG, "cnoc_gpu_cfg"),
  RSTNAME(EIC7700X_RESET_CNOC_DSPT_CFG, "cnoc_dspt_cfg"),
  RSTNAME(EIC7700X_RESET_CNOC_DDRT1_CFG, "cnoc_ddrt1_cfg"),
  RSTNAME(EIC7700X_RESET_CNOC_DDRT0_CFG, "cnoc_ddrt0_cfg"),
  RSTNAME(EIC7700X_RESET_CNOC_D2D_CFG, "cnoc_d2d_cfg"),
  RSTNAME(EIC7700X_RESET_CNOC_CFG,   "cnoc_cfg"),
  RSTNAME(EIC7700X_RESET_CNOC_CLMM_CFG, "cnoc_clmm_cfg"),
  RSTNAME(EIC7700X_RESET_CNOC_AON_CFG, "cnoc_aon_cfg"),
  RSTNAME(EIC7700X_RESET_LNOC_CFG,   "lnoc_cfg"),
  RSTNAME(EIC7700X_RESET_LNOC_NPU_LLC_A, "lnoc_npu_llc_a"),
  RSTNAME(EIC7700X_RESET_LNOC_DDRC1_P0_A, "lnoc_ddrc1_p0_a"),
  RSTNAME(EIC7700X_RESET_LNOC_DDRC0_P0_A, "lnoc_ddrc0_p0_a"),
  RSTNAME(EIC7700X_RESET_PIPE_TBU2SNOC, "pipe_tbu2snoc"),
  RSTNAME(EIC7700X_RESET_PIPE_TBU2MNOC, "pipe_tbu2mnoc"),
  RSTNAME(EIC7700X_RESET_PIPE_SNOC2PCIE, "pipe_snoc2pcie"),
  RSTNAME(EIC7700X_RESET_PIPE_SNOC2MNOC, "pipe_snoc2mnoc"),
  RSTNAME(EIC7700X_RESET_PIPE_SNOC2DDR1P1, "pipe_snoc2ddr1p1"),
  RSTNAME(EIC7700X_RESET_PIPE_SNOC2DDR0P1, "pipe_snoc2ddr0p1"),
  RSTNAME(EIC7700X_RESET_PIPE_SNOC2DDR1P2, "pipe_snoc2ddr1p2"),
  RSTNAME(EIC7700X_RESET_PIPE_SNOC2DDR0P2, "pipe_snoc2ddr0p2"),
  RSTNAME(EIC7700X_RESET_PIPE_SNOC2D2D, "pipe_snoc2d2d"),
  RSTNAME(EIC7700X_RESET_PIPE_MCPU2SNOC, "pipe_mcpu2snoc"),
  RSTNAME(EIC7700X_RESET_PIPE_MCPU2SNOCSP, "pipe_mcpu2snocsp"),
  RSTNAME(EIC7700X_RESET_PIPE_MNOC2SNOC, "pipe_mnoc2snoc"),
  RSTNAME(EIC7700X_RESET_PIPE_LNOC2DDR1, "pipe_lnoc2ddr1"),
  RSTNAME(EIC7700X_RESET_PIPE_LNOC2DDR0, "pipe_lnoc2ddr0"),
  RSTNAME(EIC7700X_RESET_PIPE_RNOC2DDR1, "pipe_rnoc2ddr1"),
  RSTNAME(EIC7700X_RESET_PIPE_RNOC2DDR0, "pipe_rnoc2ddr0"),
  RSTNAME(EIC7700X_RESET_PIPE_MNOC2DDR0, "pipe_mnoc2ddr0"),
  RSTNAME(EIC7700X_RESET_PIPE_SNOC2DSP, "pipe_snoc2dsp"),
  RSTNAME(EIC7700X_RESET_PIPE_DSP2SNOC, "pipe_dsp2snoc"),
  RSTNAME(EIC7700X_RESET_PIPE_DSP2TCU, "pipe_dsp2tcu"),
  RSTNAME(EIC7700X_RESET_PIPE_PCIE2TCU, "pipe_pcie2tcu"),
  RSTNAME(EIC7700X_RESET_PIPE_VI2RNOC, "pipe_vi2rnoc"),
  RSTNAME(EIC7700X_RESET_PIPE_VO2RNOC, "pipe_vo2rnoc"),
  RSTNAME(EIC7700X_RESET_PIPE_GPU2MNOC, "pipe_gpu2mnoc"),
  RSTNAME(EIC7700X_RESET_PIPE_VC2MNOC, "pipe_vc2mnoc"),
  RSTNAME(EIC7700X_RESET_TBU_AON_A,  "tbu_aon_a"),
  RSTNAME(EIC7700X_RESET_TBU_PCIET_TBU3, "tbu_pciet_tbu3"),
  RSTNAME(EIC7700X_RESET_TBU_HSP_A,  "tbu_hsp_a"),
  RSTNAME(EIC7700X_RESET_TBU_VI_AXI, "tbu_vi_axi"),
  RSTNAME(EIC7700X_RESET_EXT_D2D_STATUS, "ext_d2d_status"),
  RSTNAME(EIC7700X_RESET_TESTMUX,    "testmux"),
  RSTNAME(EIC7700X_RESET_SPI_SLV,    "spi_slv"),
};

static_assert(nitems(g_eic7700x_reset_names) == 324,
              "reset name table does not cover every line");

/* eic7700x_reset_getline() binary searches this table, so it has to be in
 * id order.  Checking every pair at compile time is not expressible here;
 * the ends catch a table rebuilt in the wrong order.
 */

static_assert(EIC7700X_RESET_NOC_NSP < EIC7700X_RESET_SPI_SLV,
              "reset name table must be sorted by id");

/****************************************************************************
 * Name: eic7700x_reset_getline
 *
 * Description:
 *   Describe one reset line for /proc/reset: its name, and the register
 *   and bit it lives in.  The state is not reported here; the framework
 *   asks status() for it.
 *
 *   The ids are a register index times thirty two plus a bit, so most of
 *   the space names nothing.  The table is sorted by id, which makes the
 *   lookup a binary search and reports a gap as -ENODEV.
 *
 *   Reads only.
 *
 * Input Parameters:
 *   rcdev - The reset controller
 *   id    - The reset line id
 *   info  - Receives the name, and the register and bit as extra text
 *
 * Returned Value:
 *   OK, or -ENODEV if the id names no line.
 *
 ****************************************************************************/

int eic7700x_reset_getline(FAR struct reset_controller_dev *rcdev,
                           unsigned int id,
                           FAR struct reset_lineinfo_s *info)
{
  unsigned int low = 0;
  unsigned int high = nitems(g_eic7700x_reset_names);

  while (low < high)
    {
      unsigned int mid = low + (high - low) / 2;
      unsigned int found = g_eic7700x_reset_names[mid].id;

      if (found == id)
        {
          strlcpy(info->name, g_eic7700x_reset_names[mid].name,
                  sizeof(info->name));
          snprintf(info->extra, sizeof(info->extra), "reg:0x%03x bit:%u",
                   (unsigned int)(0x400 + EIC7700X_RESET_REGOF(id) * 4),
                   EIC7700X_RESET_BITOF(id));
          return OK;
        }

      if (found < id)
        {
          low = mid + 1;
        }
      else
        {
          high = mid;
        }
    }

  return -ENODEV;
}

#endif /* CONFIG_RESET_PROCFS */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: eic7700x_reset_initialize
 *
 * Description:
 *   Register the reset lines of the Clock and Reset Generator with the
 *   reset framework.  See eic7700x_reset.h.
 *
 ****************************************************************************/

/* What eic7700x_reset_initialize() found, for the board to report */

static unsigned int g_eic7700x_reset_count;
static unsigned int g_eic7700x_reset_held;

int eic7700x_reset_initialize(void)
{
  size_t reg;
  int ret;

#ifdef CONFIG_DEBUG_ASSERTIONS
  size_t i;

  /* A critical or read only bit that names no line, or a line marked as
   * both, means a mistyped mask.  Either would show up on hardware as a
   * reset that silently does nothing, which is a poor way to find out.
   */

  for (i = 0; i < nitems(g_eic7700x_reset_regs); i++)
    {
      FAR const struct eic7700x_reset_reg_s *desc =
        &g_eic7700x_reset_regs[i];

      DEBUGASSERT((desc->critical & ~desc->valid) == 0);
      DEBUGASSERT((desc->rdonly & ~desc->valid) == 0);
      DEBUGASSERT((desc->critical & desc->rdonly) == 0);
    }
#endif

  ret = reset_controller_register(&g_eic7700x_rcdev);
  if (ret < 0)
    {
      rsterr("failed to register the reset controller: %d\n", ret);
      return ret;
    }

  rstinfo("registered %s\n", g_eic7700x_rcdev.name);

  /* Count what the boot loader left, one register read per register
   * rather than one per line.  The lines are active low, so a valid bit
   * reading zero is a line still held.
   */

  for (reg = 0; reg < nitems(g_eic7700x_reset_regs); reg++)
    {
      uint32_t valid = g_eic7700x_reset_regs[reg].valid;
      uint32_t raw = getreg32(EIC7700X_RESET_CTRL(reg));

      g_eic7700x_reset_count += __builtin_popcount(valid);
      g_eic7700x_reset_held += __builtin_popcount(valid & ~raw);
    }

  return OK;
}

/****************************************************************************
 * Name: eic7700x_reset_count
 *
 * Description:
 *   What eic7700x_reset_initialize() found.  See eic7700x_reset.h.
 *
 ****************************************************************************/

unsigned int eic7700x_reset_count(FAR unsigned int *held)
{
  if (held != NULL)
    {
      *held = g_eic7700x_reset_held;
    }

  return g_eic7700x_reset_count;
}
