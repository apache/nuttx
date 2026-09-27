/****************************************************************************
 * arch/risc-v/include/eic7700x/eic7700x_reset.h
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

#ifndef __ARCH_RISCV_INCLUDE_EIC7700X_EIC7700X_RESET_H
#define __ARCH_RISCV_INCLUDE_EIC7700X_EIC7700X_RESET_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Name this provider registers under.  Pass it to reset_control_get() and
 * the wrappers around it, together with one of the line ids below.
 */

#define EIC7700X_RESET_CONTROLLER  "eic7700x-crg"

/* A reset line is named by the control register that holds it and by the
 * bit within that register:
 *
 *   id = register index * 32 + bit
 *
 * The register index is the word offset from the first reset control
 * register, so decoding an id is arithmetic and needs no table.  Register
 * index 0 is offset 0x400 and register index 60 is offset 0x4f0.
 */

#define EIC7700X_RESET_NREGS       61
#define EIC7700X_RESET_ID(r, b)    ((r) * 32 + (b))
#define EIC7700X_RESET_REGOF(id)   ((id) >> 5)
#define EIC7700X_RESET_BITOF(id)   ((id) & 31)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Every reset line the Clock and Reset Generator carries.
 *
 * The lines are active low: a line is held in reset while its bit reads
 * zero and released while it reads one.  The manual never says so.  It is
 * inferred from the field names, which all end in _rstn, _arstn, _prstn or
 * _hrstn, from the reset defaults, which read one for every block that has
 * to be running before software starts and zero for every block software
 * has to bring up, and from both vendor Linux drivers, which assert by
 * clearing the bit and deassert by setting it.  Before trusting a write,
 * read the console UART line back: it must report deasserted on a board
 * that is printing.
 *
 * Section 3.1 of the manual asks that the clock be running before a reset
 * is released, and that a block's configuration interface be released
 * before its bus interface and core logic.  It also warns that resetting a
 * block that is not idle can hang the bus.  None of that is something this
 * provider can enforce; all of it is the caller's to get right.
 *
 * Lines that carry the fabric, main memory, the cluster NuttX runs on or
 * the configuration path back to this register block are registered but
 * refuse to be asserted, because asserting one of them takes down the
 * system doing the asserting.  They can still be released and read.
 */

enum eic7700x_reset_e
{
  /* 0x400 snoc_rst_ctrl, system network on chip (TRM p141).  Every line
   * here carries instruction fetch, data, or the path to the generator's
   * own registers, so none of them may be asserted.
   */

  EIC7700X_RESET_NOC_NSP          = EIC7700X_RESET_ID(0x00,  0),
  EIC7700X_RESET_NOC_CFG          = EIC7700X_RESET_ID(0x00,  1),
  EIC7700X_RESET_RNOC_NSP         = EIC7700X_RESET_ID(0x00,  2),
  EIC7700X_RESET_SNOC_TCU_A       = EIC7700X_RESET_ID(0x00,  3),
  EIC7700X_RESET_SNOC_U84_A       = EIC7700X_RESET_ID(0x00,  4),
  EIC7700X_RESET_SNOC_PCIET_XS    = EIC7700X_RESET_ID(0x00,  5),
  EIC7700X_RESET_SNOC_PCIET_XM    = EIC7700X_RESET_ID(0x00,  6),
  EIC7700X_RESET_SNOC_PCIET_P     = EIC7700X_RESET_ID(0x00,  7),
  EIC7700X_RESET_SNOC_NPU_A       = EIC7700X_RESET_ID(0x00,  8),
  EIC7700X_RESET_SNOC_JTAG_P      = EIC7700X_RESET_ID(0x00,  9),
  EIC7700X_RESET_SNOC_DSPT_A      = EIC7700X_RESET_ID(0x00, 10),
  EIC7700X_RESET_SNOC_DDRC1_P2_A  = EIC7700X_RESET_ID(0x00, 11),
  EIC7700X_RESET_SNOC_DDRC1_P1_A  = EIC7700X_RESET_ID(0x00, 12),
  EIC7700X_RESET_SNOC_DDRC0_P2_A  = EIC7700X_RESET_ID(0x00, 13),
  EIC7700X_RESET_SNOC_DDRC0_P1_A  = EIC7700X_RESET_ID(0x00, 14),
  EIC7700X_RESET_SNOC_D2D_A       = EIC7700X_RESET_ID(0x00, 15),
  EIC7700X_RESET_SNOC_AON_A       = EIC7700X_RESET_ID(0x00, 16),

  /* 0x404 gpu_rst_ctrl, 3D graphics (TRM p142) */

  EIC7700X_RESET_GPU_AXI          = EIC7700X_RESET_ID(0x01,  0),
  EIC7700X_RESET_GPU_CFG          = EIC7700X_RESET_ID(0x01,  1),
  EIC7700X_RESET_GPU_GRAY         = EIC7700X_RESET_ID(0x01,  2),
  EIC7700X_RESET_GPU_JONES        = EIC7700X_RESET_ID(0x01,  3),
  EIC7700X_RESET_GPU_SPU          = EIC7700X_RESET_ID(0x01,  4),

  /* 0x408 dsp_rst_ctrl, digital signal processors (TRM p143).  The four
   * divider resets in sw_dsp_div_rstn[7:4] carry no instance mapping;
   * ascending order is assumed.  Bit 3 is reserved.
   */

  EIC7700X_RESET_DSP_AXI          = EIC7700X_RESET_ID(0x02,  0),
  EIC7700X_RESET_DSP_CFG          = EIC7700X_RESET_ID(0x02,  1),
  EIC7700X_RESET_DSP_DIV4         = EIC7700X_RESET_ID(0x02,  2),
  EIC7700X_RESET_DSP_DIV0         = EIC7700X_RESET_ID(0x02,  4),
  EIC7700X_RESET_DSP_DIV1         = EIC7700X_RESET_ID(0x02,  5),
  EIC7700X_RESET_DSP_DIV2         = EIC7700X_RESET_ID(0x02,  6),
  EIC7700X_RESET_DSP_DIV3         = EIC7700X_RESET_ID(0x02,  7),

  /* 0x40c d2d_rst_ctrl, die to die interface (TRM p143).  Only the
   * subsystem line resets released; the rest come up held.
   */

  EIC7700X_RESET_D2D_AXI          = EIC7700X_RESET_ID(0x03,  0),
  EIC7700X_RESET_D2D_CFG          = EIC7700X_RESET_ID(0x03,  1),
  EIC7700X_RESET_D2D_P            = EIC7700X_RESET_ID(0x03,  2),
  EIC7700X_RESET_D2D_RAW_PCS      = EIC7700X_RESET_ID(0x03,  3),
  EIC7700X_RESET_D2D_RX           = EIC7700X_RESET_ID(0x03,  4),
  EIC7700X_RESET_D2D_TX           = EIC7700X_RESET_ID(0x03,  5),
  EIC7700X_RESET_D2D_CORE         = EIC7700X_RESET_ID(0x03,  6),
  EIC7700X_RESET_D2D_SUBSYS       = EIC7700X_RESET_ID(0x03,  7),

  /* 0x410 ddr_rst_ctrl, both memory controllers (TRM p143).  This is the
   * memory NuttX runs from, so none of it may be asserted.  The manual
   * gives no mapping from bit to port for sw_ddr0_p_arstn[20:16] or
   * sw_ddr1_p_arstn[4:0]; ascending order is assumed.  Note the two
   * controllers are not laid out symmetrically.
   */

  EIC7700X_RESET_DDR1_P0_A        = EIC7700X_RESET_ID(0x04,  0),
  EIC7700X_RESET_DDR1_P1_A        = EIC7700X_RESET_ID(0x04,  1),
  EIC7700X_RESET_DDR1_P2_A        = EIC7700X_RESET_ID(0x04,  2),
  EIC7700X_RESET_DDR1_P3_A        = EIC7700X_RESET_ID(0x04,  3),
  EIC7700X_RESET_DDR1_P4_A        = EIC7700X_RESET_ID(0x04,  4),
  EIC7700X_RESET_DDR1_TRACE_A     = EIC7700X_RESET_ID(0x04,  6),
  EIC7700X_RESET_DDR1_PMP_SOFT    = EIC7700X_RESET_ID(0x04,  8),
  EIC7700X_RESET_DDR0_P0_A        = EIC7700X_RESET_ID(0x04, 16),
  EIC7700X_RESET_DDR0_P1_A        = EIC7700X_RESET_ID(0x04, 17),
  EIC7700X_RESET_DDR0_P2_A        = EIC7700X_RESET_ID(0x04, 18),
  EIC7700X_RESET_DDR0_P3_A        = EIC7700X_RESET_ID(0x04, 19),
  EIC7700X_RESET_DDR0_P4_A        = EIC7700X_RESET_ID(0x04, 20),
  EIC7700X_RESET_DDR_CFG          = EIC7700X_RESET_ID(0x04, 21),
  EIC7700X_RESET_DDR0_TRACE_A     = EIC7700X_RESET_ID(0x04, 22),
  EIC7700X_RESET_DDR_CORE         = EIC7700X_RESET_ID(0x04, 23),
  EIC7700X_RESET_DDR0_PMP_SOFT    = EIC7700X_RESET_ID(0x04, 24),
  EIC7700X_RESET_DDR_P            = EIC7700X_RESET_ID(0x04, 26),

  /* 0x414 tcu_rst_ctrl, translation control unit (TRM p144).  The manual
   * describes tbu_rstn[20:4] only as "Reserved.", giving no mapping from
   * bit to translation buffer unit; the seventeen lines below assume the
   * ascending order the vendor Linux binding uses.
   */

  EIC7700X_RESET_TCU_AXI          = EIC7700X_RESET_ID(0x05,  0),
  EIC7700X_RESET_TCU_CFG          = EIC7700X_RESET_ID(0x05,  1),
  EIC7700X_RESET_TCU_TBU0         = EIC7700X_RESET_ID(0x05,  4),
  EIC7700X_RESET_TCU_TBU1         = EIC7700X_RESET_ID(0x05,  5),
  EIC7700X_RESET_TCU_TBU2         = EIC7700X_RESET_ID(0x05,  6),
  EIC7700X_RESET_TCU_TBU3         = EIC7700X_RESET_ID(0x05,  7),
  EIC7700X_RESET_TCU_TBU4         = EIC7700X_RESET_ID(0x05,  8),
  EIC7700X_RESET_TCU_TBU5         = EIC7700X_RESET_ID(0x05,  9),
  EIC7700X_RESET_TCU_TBU6         = EIC7700X_RESET_ID(0x05, 10),
  EIC7700X_RESET_TCU_TBU7         = EIC7700X_RESET_ID(0x05, 11),
  EIC7700X_RESET_TCU_TBU8         = EIC7700X_RESET_ID(0x05, 12),
  EIC7700X_RESET_TCU_TBU9         = EIC7700X_RESET_ID(0x05, 13),
  EIC7700X_RESET_TCU_TBU10        = EIC7700X_RESET_ID(0x05, 14),
  EIC7700X_RESET_TCU_TBU11        = EIC7700X_RESET_ID(0x05, 15),
  EIC7700X_RESET_TCU_TBU12        = EIC7700X_RESET_ID(0x05, 16),
  EIC7700X_RESET_TCU_TBU13        = EIC7700X_RESET_ID(0x05, 17),
  EIC7700X_RESET_TCU_TBU14        = EIC7700X_RESET_ID(0x05, 18),
  EIC7700X_RESET_TCU_TBU15        = EIC7700X_RESET_ID(0x05, 19),
  EIC7700X_RESET_TCU_TBU16        = EIC7700X_RESET_ID(0x05, 20),

  /* 0x418 npu_rst_ctrl, the neural processor (TRM p144).  The manual
   * marks the two E31 debug and bus lines "reserved" but still names
   * them, so they are exposed under their own names.
   */

  EIC7700X_RESET_NPU_AXI          = EIC7700X_RESET_ID(0x06,  0),
  EIC7700X_RESET_NPU_CFG          = EIC7700X_RESET_ID(0x06,  1),
  EIC7700X_RESET_NPU_CORE         = EIC7700X_RESET_ID(0x06,  2),
  EIC7700X_RESET_NPU_E31CORE      = EIC7700X_RESET_ID(0x06,  3),
  EIC7700X_RESET_NPU_E31BUS       = EIC7700X_RESET_ID(0x06,  4),
  EIC7700X_RESET_NPU_E31DBG       = EIC7700X_RESET_ID(0x06,  5),
  EIC7700X_RESET_NPU_LLC          = EIC7700X_RESET_ID(0x06,  6),

  /* 0x41c hspdma_rst_ctrl, the high speed peripherals (TRM p145).  There
   * is no separate register for SD, eMMC or ethernet: everything lives
   * here, and every bit resets HELD, so any of these blocks needs its
   * lines released before it will answer.
   *
   * Section 3.1 of the manual asks for the configuration interface first,
   * so release HSP_CFG before HSP_AXI and before any of the per device
   * lines.
   *
   * sw_mshc_phy_rstn[5:3] and sw_mshc_txrx_rstn[8:6] are three bit fields
   * with no instance mapping in the manual.  Controller n is taken to be
   * the n'th bit of each field, matching the vendor Linux binding.  Take
   * care here: instance 0 is bit 3 and bit 6 rather than bit 0, and the
   * neighbouring per device lines run eMMC, SD0, SD1 descending while
   * this assumption runs ascending, so the two orders do not agree and
   * only silicon can say which is right.
   */

  EIC7700X_RESET_HSP_AXI          = EIC7700X_RESET_ID(0x07,  0),
  EIC7700X_RESET_HSP_CFG          = EIC7700X_RESET_ID(0x07,  1),
  EIC7700X_RESET_HSP_POR          = EIC7700X_RESET_ID(0x07,  2),
  EIC7700X_RESET_MSHC0_PHY        = EIC7700X_RESET_ID(0x07,  3),
  EIC7700X_RESET_MSHC1_PHY        = EIC7700X_RESET_ID(0x07,  4),
  EIC7700X_RESET_MSHC2_PHY        = EIC7700X_RESET_ID(0x07,  5),
  EIC7700X_RESET_MSHC0_TXRX       = EIC7700X_RESET_ID(0x07,  6),
  EIC7700X_RESET_MSHC1_TXRX       = EIC7700X_RESET_ID(0x07,  7),
  EIC7700X_RESET_MSHC2_TXRX       = EIC7700X_RESET_ID(0x07,  8),
  EIC7700X_RESET_SATA_ASIC0       = EIC7700X_RESET_ID(0x07,  9),
  EIC7700X_RESET_SATA_OOB         = EIC7700X_RESET_ID(0x07, 10),
  EIC7700X_RESET_SATA_PMALIVE     = EIC7700X_RESET_ID(0x07, 11),
  EIC7700X_RESET_SATA_RBC         = EIC7700X_RESET_ID(0x07, 12),
  EIC7700X_RESET_DMA0             = EIC7700X_RESET_ID(0x07, 13),
  EIC7700X_RESET_HSP_DMA0         = EIC7700X_RESET_ID(0x07, 14),
  EIC7700X_RESET_USB0_VAUX        = EIC7700X_RESET_ID(0x07, 15),
  EIC7700X_RESET_USB1_VAUX        = EIC7700X_RESET_ID(0x07, 16),
  EIC7700X_RESET_HSP_SD1_P        = EIC7700X_RESET_ID(0x07, 17),
  EIC7700X_RESET_HSP_SD0_P        = EIC7700X_RESET_ID(0x07, 18),
  EIC7700X_RESET_HSP_EMMC_P       = EIC7700X_RESET_ID(0x07, 19),
  EIC7700X_RESET_HSP_DMA_P        = EIC7700X_RESET_ID(0x07, 20),
  EIC7700X_RESET_HSP_SD1_A        = EIC7700X_RESET_ID(0x07, 21),
  EIC7700X_RESET_HSP_SD0_A        = EIC7700X_RESET_ID(0x07, 22),
  EIC7700X_RESET_HSP_EMMC_A       = EIC7700X_RESET_ID(0x07, 23),
  EIC7700X_RESET_HSP_DMA_A        = EIC7700X_RESET_ID(0x07, 24),
  EIC7700X_RESET_HSP_ETH1_A       = EIC7700X_RESET_ID(0x07, 25),
  EIC7700X_RESET_HSP_ETH0_A       = EIC7700X_RESET_ID(0x07, 26),
  EIC7700X_RESET_HSP_SATA_A       = EIC7700X_RESET_ID(0x07, 27),

  /* 0x420 pcie_rst_ctrl (TRM p146) */

  EIC7700X_RESET_PCIE_CFG         = EIC7700X_RESET_ID(0x08,  0),
  EIC7700X_RESET_PCIE_POWERUP     = EIC7700X_RESET_ID(0x08,  1),
  EIC7700X_RESET_PCIE_PERST       = EIC7700X_RESET_ID(0x08,  2),

  /* 0x424 i2c_rst_ctrl (TRM p146).  All ten controllers reset HELD, so a
   * driver must release its line before touching the peripheral.  The
   * manual gives sw_i2c_rst_n[9:0] as one field with no instance mapping;
   * bit n is taken to be controller n, which is what the vendor Linux
   * binding assumes.
   *
   * The two single bit registers at 0x494 and 0x498 further down are a
   * separate thing that the manual does not relate to this field.
   */

  EIC7700X_RESET_I2C0             = EIC7700X_RESET_ID(0x09,  0),
  EIC7700X_RESET_I2C1             = EIC7700X_RESET_ID(0x09,  1),
  EIC7700X_RESET_I2C2             = EIC7700X_RESET_ID(0x09,  2),
  EIC7700X_RESET_I2C3             = EIC7700X_RESET_ID(0x09,  3),
  EIC7700X_RESET_I2C4             = EIC7700X_RESET_ID(0x09,  4),
  EIC7700X_RESET_I2C5             = EIC7700X_RESET_ID(0x09,  5),
  EIC7700X_RESET_I2C6             = EIC7700X_RESET_ID(0x09,  6),
  EIC7700X_RESET_I2C7             = EIC7700X_RESET_ID(0x09,  7),
  EIC7700X_RESET_I2C8             = EIC7700X_RESET_ID(0x09,  8),
  EIC7700X_RESET_I2C9             = EIC7700X_RESET_ID(0x09,  9),

  /* 0x428 fan_rst_ctrl, 0x42c pvt_rst_ctrl (TRM p146) */

  EIC7700X_RESET_FAN              = EIC7700X_RESET_ID(0x0a,  0),
  EIC7700X_RESET_PVT_LSP          = EIC7700X_RESET_ID(0x0b,  0),
  EIC7700X_RESET_PVT_DDR          = EIC7700X_RESET_ID(0x0b,  1),

  /* 0x430 mbox_rst_ctrl (TRM p146).  Sixteen mailboxes in one field with
   * no instance mapping given; ascending order is assumed.
   */

  EIC7700X_RESET_MBOX0            = EIC7700X_RESET_ID(0x0c,  0),
  EIC7700X_RESET_MBOX1            = EIC7700X_RESET_ID(0x0c,  1),
  EIC7700X_RESET_MBOX2            = EIC7700X_RESET_ID(0x0c,  2),
  EIC7700X_RESET_MBOX3            = EIC7700X_RESET_ID(0x0c,  3),
  EIC7700X_RESET_MBOX4            = EIC7700X_RESET_ID(0x0c,  4),
  EIC7700X_RESET_MBOX5            = EIC7700X_RESET_ID(0x0c,  5),
  EIC7700X_RESET_MBOX6            = EIC7700X_RESET_ID(0x0c,  6),
  EIC7700X_RESET_MBOX7            = EIC7700X_RESET_ID(0x0c,  7),
  EIC7700X_RESET_MBOX8            = EIC7700X_RESET_ID(0x0c,  8),
  EIC7700X_RESET_MBOX9            = EIC7700X_RESET_ID(0x0c,  9),
  EIC7700X_RESET_MBOX10           = EIC7700X_RESET_ID(0x0c, 10),
  EIC7700X_RESET_MBOX11           = EIC7700X_RESET_ID(0x0c, 11),
  EIC7700X_RESET_MBOX12           = EIC7700X_RESET_ID(0x0c, 12),
  EIC7700X_RESET_MBOX13           = EIC7700X_RESET_ID(0x0c, 13),
  EIC7700X_RESET_MBOX14           = EIC7700X_RESET_ID(0x0c, 14),
  EIC7700X_RESET_MBOX15           = EIC7700X_RESET_ID(0x0c, 15),

  /* 0x434 uart_rst_ctrl (TRM p147).  sw_uart_rst_n[4:0], same lack of an
   * instance mapping as the I2C field.  All five reset released.
   *
   * UART0 is the console on the boards supported so far and is not
   * locked: losing the console is recoverable, and a serial driver
   * resetting its own port is the ordinary case.
   */

  EIC7700X_RESET_UART0            = EIC7700X_RESET_ID(0x0d,  0),
  EIC7700X_RESET_UART1            = EIC7700X_RESET_ID(0x0d,  1),
  EIC7700X_RESET_UART2            = EIC7700X_RESET_ID(0x0d,  2),
  EIC7700X_RESET_UART3            = EIC7700X_RESET_ID(0x0d,  3),
  EIC7700X_RESET_UART4            = EIC7700X_RESET_ID(0x0d,  4),

  /* 0x438, the two GPIO controllers.  This register is absent from the
   * manual, whose table goes straight from 0x434 to 0x43c, but the
   * vendor Linux binding names the gap and the blocks would otherwise
   * have no reset at all.  The reset default is unknown; nothing reads
   * or writes this register at startup, so these lines only reach the
   * hardware when a driver asks for them.
   */

  EIC7700X_RESET_GPIO0            = EIC7700X_RESET_ID(0x0e,  0),
  EIC7700X_RESET_GPIO1            = EIC7700X_RESET_ID(0x0e,  1),

  /* 0x43c timer_rst_ctrl, the low speed timer block (TRM p147) */

  EIC7700X_RESET_LSP_TIMER        = EIC7700X_RESET_ID(0x0f,  0),

  /* 0x440 ssi_rst_ctrl, the two SPI controllers (TRM p147) */

  EIC7700X_RESET_SSI0             = EIC7700X_RESET_ID(0x10,  0),
  EIC7700X_RESET_SSI1             = EIC7700X_RESET_ID(0x10,  1),

  /* 0x444 wdt_rst_ctrl (TRM p147), four watchdogs, order assumed */

  EIC7700X_RESET_WDT0             = EIC7700X_RESET_ID(0x11,  0),
  EIC7700X_RESET_WDT1             = EIC7700X_RESET_ID(0x11,  1),
  EIC7700X_RESET_WDT2             = EIC7700X_RESET_ID(0x11,  2),
  EIC7700X_RESET_WDT3             = EIC7700X_RESET_ID(0x11,  3),

  /* 0x448 lsp_cfgrst_ctrl (TRM p147).  This is the configuration
   * interface reset that section 3.1 of the manual says to release
   * before the bus and core logic of anything on the low speed bus.
   */

  EIC7700X_RESET_LSP_CFG          = EIC7700X_RESET_ID(0x12,  0),

  /* 0x44c u84_rst_ctrl, the application cluster (TRM p147).  These are
   * the harts NuttX runs on, so the cluster lines may not be asserted.
   * The manual gives no mapping from bit to hart for sw_u84_core_rstn
   * [3:0] or sw_u84_trace_rstn[11:8]; ascending order is assumed.
   *
   * Note also that this provider never writes clr_boot_info at 0x30c
   * (TRM p140), the register the vendor Linux driver pokes at probe so
   * that cluster resets take effect at all.  On a board where the boot
   * loader left that flag set, these lines are inert.
   */

  EIC7700X_RESET_U84_CORE0        = EIC7700X_RESET_ID(0x13,  0),
  EIC7700X_RESET_U84_CORE1        = EIC7700X_RESET_ID(0x13,  1),
  EIC7700X_RESET_U84_CORE2        = EIC7700X_RESET_ID(0x13,  2),
  EIC7700X_RESET_U84_CORE3        = EIC7700X_RESET_ID(0x13,  3),
  EIC7700X_RESET_U84_BUS          = EIC7700X_RESET_ID(0x13,  4),
  EIC7700X_RESET_U84_DBG          = EIC7700X_RESET_ID(0x13,  5),
  EIC7700X_RESET_U84_TRACECOM     = EIC7700X_RESET_ID(0x13,  6),
  EIC7700X_RESET_U84_TRACE0       = EIC7700X_RESET_ID(0x13,  8),
  EIC7700X_RESET_U84_TRACE1       = EIC7700X_RESET_ID(0x13,  9),
  EIC7700X_RESET_U84_TRACE2       = EIC7700X_RESET_ID(0x13, 10),
  EIC7700X_RESET_U84_TRACE3       = EIC7700X_RESET_ID(0x13, 11),

  /* 0x450 scpu_rst_ctrl, the secure processor (TRM p147).  NuttX does not
   * run on it and cannot restart the firmware it runs, so these are
   * registered but not locked; a caller that asserts one is on its own.
   */

  EIC7700X_RESET_SCPU_CORE        = EIC7700X_RESET_ID(0x14,  0),
  EIC7700X_RESET_SCPU_BUS         = EIC7700X_RESET_ID(0x14,  1),
  EIC7700X_RESET_SCPU_DBG         = EIC7700X_RESET_ID(0x14,  2),

  /* 0x454 lpcpu_rst_ctrl, the low power processor (TRM p147) */

  EIC7700X_RESET_LPCPU_CORE       = EIC7700X_RESET_ID(0x15,  0),
  EIC7700X_RESET_LPCPU_BUS        = EIC7700X_RESET_ID(0x15,  1),
  EIC7700X_RESET_LPCPU_DBG        = EIC7700X_RESET_ID(0x15,  2),

  /* 0x458 to 0x46c, the video codec blocks (TRM p148) */

  EIC7700X_RESET_VC_CFG           = EIC7700X_RESET_ID(0x16,  0),
  EIC7700X_RESET_VC_AXI           = EIC7700X_RESET_ID(0x16,  1),
  EIC7700X_RESET_VC_MONCFG        = EIC7700X_RESET_ID(0x16,  2),
  EIC7700X_RESET_JD_CFG           = EIC7700X_RESET_ID(0x17,  0),
  EIC7700X_RESET_JD_AXI           = EIC7700X_RESET_ID(0x17,  1),
  EIC7700X_RESET_JE_CFG           = EIC7700X_RESET_ID(0x18,  0),
  EIC7700X_RESET_JE_AXI           = EIC7700X_RESET_ID(0x18,  1),
  EIC7700X_RESET_VD_CFG           = EIC7700X_RESET_ID(0x19,  0),
  EIC7700X_RESET_VD_AXI           = EIC7700X_RESET_ID(0x19,  1),
  EIC7700X_RESET_VE_CFG           = EIC7700X_RESET_ID(0x1a,  0),
  EIC7700X_RESET_VE_AXI           = EIC7700X_RESET_ID(0x1a,  1),
  EIC7700X_RESET_G2D_CORE         = EIC7700X_RESET_ID(0x1b,  0),
  EIC7700X_RESET_G2D_CFG          = EIC7700X_RESET_ID(0x1b,  1),
  EIC7700X_RESET_G2D_AXI          = EIC7700X_RESET_ID(0x1b,  2),

  /* 0x470 to 0x480, video input (TRM p149).  The six shutter lines are
   * one field with no instance mapping; ascending order is assumed.
   */

  EIC7700X_RESET_VI_AXI           = EIC7700X_RESET_ID(0x1c,  0),
  EIC7700X_RESET_VI_CFG           = EIC7700X_RESET_ID(0x1c,  1),
  EIC7700X_RESET_VI_DWE           = EIC7700X_RESET_ID(0x1c,  2),
  EIC7700X_RESET_VI_DVP           = EIC7700X_RESET_ID(0x1d,  0),
  EIC7700X_RESET_VI_ISP0          = EIC7700X_RESET_ID(0x1e,  0),
  EIC7700X_RESET_VI_ISP1          = EIC7700X_RESET_ID(0x1f,  0),
  EIC7700X_RESET_VI_SHUTTER0      = EIC7700X_RESET_ID(0x20,  0),
  EIC7700X_RESET_VI_SHUTTER1      = EIC7700X_RESET_ID(0x20,  1),
  EIC7700X_RESET_VI_SHUTTER2      = EIC7700X_RESET_ID(0x20,  2),
  EIC7700X_RESET_VI_SHUTTER3      = EIC7700X_RESET_ID(0x20,  3),
  EIC7700X_RESET_VI_SHUTTER4      = EIC7700X_RESET_ID(0x20,  4),
  EIC7700X_RESET_VI_SHUTTER5      = EIC7700X_RESET_ID(0x20,  5),

  /* 0x484 to 0x48c, video output (TRM p149).  Bit 2 of the phy register
   * is reserved.
   */

  EIC7700X_RESET_VO_MIPI_P        = EIC7700X_RESET_ID(0x21,  0),
  EIC7700X_RESET_VO_P             = EIC7700X_RESET_ID(0x21,  1),
  EIC7700X_RESET_VO_HDMI_P        = EIC7700X_RESET_ID(0x21,  3),
  EIC7700X_RESET_HDMI_PHYCTRL     = EIC7700X_RESET_ID(0x21,  4),
  EIC7700X_RESET_VO_HDMI          = EIC7700X_RESET_ID(0x21,  5),
  EIC7700X_RESET_VO_I2S           = EIC7700X_RESET_ID(0x22,  0),
  EIC7700X_RESET_VO_I2S_P         = EIC7700X_RESET_ID(0x22,  1),
  EIC7700X_RESET_VO_AXI           = EIC7700X_RESET_ID(0x23,  0),
  EIC7700X_RESET_VO_CFG           = EIC7700X_RESET_ID(0x23,  1),
  EIC7700X_RESET_VO_DC            = EIC7700X_RESET_ID(0x23,  2),
  EIC7700X_RESET_VO_DC_P          = EIC7700X_RESET_ID(0x23,  3),

  /* 0x490 bootspi_rst_ctrl (TRM p150) */

  EIC7700X_RESET_BOOTSPI_H        = EIC7700X_RESET_ID(0x24,  0),
  EIC7700X_RESET_BOOTSPI          = EIC7700X_RESET_ID(0x24,  1),

  /* 0x494 i2c1_rst_ctrl, 0x498 i2c0_rst_ctrl (TRM p150).  Two separate
   * single bit APB resets, presumably for the two always on side I2C
   * instances.  The manual does not say how they relate to the ten bit
   * field at 0x424, so both are exposed and neither is assumed to be a
   * duplicate of the other.
   */

  EIC7700X_RESET_I2C1_P           = EIC7700X_RESET_ID(0x25,  0),
  EIC7700X_RESET_I2C0_P           = EIC7700X_RESET_ID(0x26,  0),

  /* 0x49c dma1_rst_ctrl (TRM p150) */

  EIC7700X_RESET_DMA1_A           = EIC7700X_RESET_ID(0x27,  0),
  EIC7700X_RESET_DMA1_H           = EIC7700X_RESET_ID(0x27,  1),

  /* 0x4a0 to 0x4b8, the security and one time programmable blocks
   * (TRM p150, p151).  All reset released.
   */

  EIC7700X_RESET_FPRT_H           = EIC7700X_RESET_ID(0x28,  0),
  EIC7700X_RESET_HBLOCK_H         = EIC7700X_RESET_ID(0x29,  0),
  EIC7700X_RESET_SECSR_H          = EIC7700X_RESET_ID(0x2a,  0),
  EIC7700X_RESET_OTP_P            = EIC7700X_RESET_ID(0x2b,  0),
  EIC7700X_RESET_PKA_H            = EIC7700X_RESET_ID(0x2c,  0),
  EIC7700X_RESET_SPACC            = EIC7700X_RESET_ID(0x2d,  0),
  EIC7700X_RESET_TRNG_H           = EIC7700X_RESET_ID(0x2e,  0),

  /* 0x4c0 to 0x4cc, the four timer blocks (TRM p151).  Each has one APB
   * reset and eight counter resets with no mapping given; ascending
   * order is assumed.  Timer 0 resets released, timers 1 to 3 held.
   */

  EIC7700X_RESET_TIMER0_CNT0      = EIC7700X_RESET_ID(0x30,  0),
  EIC7700X_RESET_TIMER0_CNT1      = EIC7700X_RESET_ID(0x30,  1),
  EIC7700X_RESET_TIMER0_CNT2      = EIC7700X_RESET_ID(0x30,  2),
  EIC7700X_RESET_TIMER0_CNT3      = EIC7700X_RESET_ID(0x30,  3),
  EIC7700X_RESET_TIMER0_CNT4      = EIC7700X_RESET_ID(0x30,  4),
  EIC7700X_RESET_TIMER0_CNT5      = EIC7700X_RESET_ID(0x30,  5),
  EIC7700X_RESET_TIMER0_CNT6      = EIC7700X_RESET_ID(0x30,  6),
  EIC7700X_RESET_TIMER0_CNT7      = EIC7700X_RESET_ID(0x30,  7),
  EIC7700X_RESET_TIMER0_P         = EIC7700X_RESET_ID(0x30,  8),

  EIC7700X_RESET_TIMER1_CNT0      = EIC7700X_RESET_ID(0x31,  0),
  EIC7700X_RESET_TIMER1_CNT1      = EIC7700X_RESET_ID(0x31,  1),
  EIC7700X_RESET_TIMER1_CNT2      = EIC7700X_RESET_ID(0x31,  2),
  EIC7700X_RESET_TIMER1_CNT3      = EIC7700X_RESET_ID(0x31,  3),
  EIC7700X_RESET_TIMER1_CNT4      = EIC7700X_RESET_ID(0x31,  4),
  EIC7700X_RESET_TIMER1_CNT5      = EIC7700X_RESET_ID(0x31,  5),
  EIC7700X_RESET_TIMER1_CNT6      = EIC7700X_RESET_ID(0x31,  6),
  EIC7700X_RESET_TIMER1_CNT7      = EIC7700X_RESET_ID(0x31,  7),
  EIC7700X_RESET_TIMER1_P         = EIC7700X_RESET_ID(0x31,  8),

  EIC7700X_RESET_TIMER2_CNT0      = EIC7700X_RESET_ID(0x32,  0),
  EIC7700X_RESET_TIMER2_CNT1      = EIC7700X_RESET_ID(0x32,  1),
  EIC7700X_RESET_TIMER2_CNT2      = EIC7700X_RESET_ID(0x32,  2),
  EIC7700X_RESET_TIMER2_CNT3      = EIC7700X_RESET_ID(0x32,  3),
  EIC7700X_RESET_TIMER2_CNT4      = EIC7700X_RESET_ID(0x32,  4),
  EIC7700X_RESET_TIMER2_CNT5      = EIC7700X_RESET_ID(0x32,  5),
  EIC7700X_RESET_TIMER2_CNT6      = EIC7700X_RESET_ID(0x32,  6),
  EIC7700X_RESET_TIMER2_CNT7      = EIC7700X_RESET_ID(0x32,  7),
  EIC7700X_RESET_TIMER2_P         = EIC7700X_RESET_ID(0x32,  8),

  EIC7700X_RESET_TIMER3_CNT0      = EIC7700X_RESET_ID(0x33,  0),
  EIC7700X_RESET_TIMER3_CNT1      = EIC7700X_RESET_ID(0x33,  1),
  EIC7700X_RESET_TIMER3_CNT2      = EIC7700X_RESET_ID(0x33,  2),
  EIC7700X_RESET_TIMER3_CNT3      = EIC7700X_RESET_ID(0x33,  3),
  EIC7700X_RESET_TIMER3_CNT4      = EIC7700X_RESET_ID(0x33,  4),
  EIC7700X_RESET_TIMER3_CNT5      = EIC7700X_RESET_ID(0x33,  5),
  EIC7700X_RESET_TIMER3_CNT6      = EIC7700X_RESET_ID(0x33,  6),
  EIC7700X_RESET_TIMER3_CNT7      = EIC7700X_RESET_ID(0x33,  7),
  EIC7700X_RESET_TIMER3_P         = EIC7700X_RESET_ID(0x33,  8),

  /* 0x4d0 rtc_rst_ctrl (TRM p151) */

  EIC7700X_RESET_RTC              = EIC7700X_RESET_ID(0x34,  0),

  /* 0x4d4 mnoc_rst_ctrl, memory network on chip (TRM p151) */

  EIC7700X_RESET_MNOC_SNOC_NSP    = EIC7700X_RESET_ID(0x35,  0),
  EIC7700X_RESET_MNOC_VC_A        = EIC7700X_RESET_ID(0x35,  1),
  EIC7700X_RESET_MNOC_CFG         = EIC7700X_RESET_ID(0x35,  2),
  EIC7700X_RESET_MNOC_HSP_A       = EIC7700X_RESET_ID(0x35,  3),
  EIC7700X_RESET_MNOC_GPU_A       = EIC7700X_RESET_ID(0x35,  4),
  EIC7700X_RESET_MNOC_DDRC1_P3_A  = EIC7700X_RESET_ID(0x35,  5),
  EIC7700X_RESET_MNOC_DDRC0_P3_A  = EIC7700X_RESET_ID(0x35,  6),

  /* 0x4d8 rnoc_rst_ctrl, real time network on chip (TRM p152) */

  EIC7700X_RESET_RNOC_VO_A        = EIC7700X_RESET_ID(0x36,  0),
  EIC7700X_RESET_RNOC_VI_A        = EIC7700X_RESET_ID(0x36,  1),
  EIC7700X_RESET_RNOC_SNOC_NSP    = EIC7700X_RESET_ID(0x36,  2),
  EIC7700X_RESET_RNOC_CFG         = EIC7700X_RESET_ID(0x36,  3),
  EIC7700X_RESET_RNOC_DDRC1_P4_A  = EIC7700X_RESET_ID(0x36,  4),
  EIC7700X_RESET_RNOC_DDRC0_P4_A  = EIC7700X_RESET_ID(0x36,  5),

  /* 0x4dc cnoc_rst_ctrl, configuration network on chip (TRM p152).  Bit
   * 13 is the worst single line on the SoC: asserting it removes access
   * to every peripheral's configuration space, this register block
   * included, so it cannot be undone.
   */

  EIC7700X_RESET_CNOC_VO_CFG      = EIC7700X_RESET_ID(0x37,  0),
  EIC7700X_RESET_CNOC_VI_CFG      = EIC7700X_RESET_ID(0x37,  1),
  EIC7700X_RESET_CNOC_VC_CFG      = EIC7700X_RESET_ID(0x37,  2),
  EIC7700X_RESET_CNOC_TCU_CFG     = EIC7700X_RESET_ID(0x37,  3),
  EIC7700X_RESET_CNOC_PCIET_CFG   = EIC7700X_RESET_ID(0x37,  4),
  EIC7700X_RESET_CNOC_NPU_CFG     = EIC7700X_RESET_ID(0x37,  5),
  EIC7700X_RESET_CNOC_LSP_CFG     = EIC7700X_RESET_ID(0x37,  6),
  EIC7700X_RESET_CNOC_HSP_CFG     = EIC7700X_RESET_ID(0x37,  7),
  EIC7700X_RESET_CNOC_GPU_CFG     = EIC7700X_RESET_ID(0x37,  8),
  EIC7700X_RESET_CNOC_DSPT_CFG    = EIC7700X_RESET_ID(0x37,  9),
  EIC7700X_RESET_CNOC_DDRT1_CFG   = EIC7700X_RESET_ID(0x37, 10),
  EIC7700X_RESET_CNOC_DDRT0_CFG   = EIC7700X_RESET_ID(0x37, 11),
  EIC7700X_RESET_CNOC_D2D_CFG     = EIC7700X_RESET_ID(0x37, 12),
  EIC7700X_RESET_CNOC_CFG         = EIC7700X_RESET_ID(0x37, 13),
  EIC7700X_RESET_CNOC_CLMM_CFG    = EIC7700X_RESET_ID(0x37, 14),
  EIC7700X_RESET_CNOC_AON_CFG     = EIC7700X_RESET_ID(0x37, 15),

  /* 0x4e0 lnoc_rst_ctrl, low latency network on chip (TRM p153) */

  EIC7700X_RESET_LNOC_CFG         = EIC7700X_RESET_ID(0x38,  0),
  EIC7700X_RESET_LNOC_NPU_LLC_A   = EIC7700X_RESET_ID(0x38,  1),
  EIC7700X_RESET_LNOC_DDRC1_P0_A  = EIC7700X_RESET_ID(0x38,  2),
  EIC7700X_RESET_LNOC_DDRC0_P0_A  = EIC7700X_RESET_ID(0x38,  3),

  /* 0x4e4 pipe_rst_ctrl, pipeline cells inside the fabric (TRM p153).
   * Neither Linux driver models this register.
   */

  EIC7700X_RESET_PIPE_TBU2SNOC    = EIC7700X_RESET_ID(0x39,  0),
  EIC7700X_RESET_PIPE_TBU2MNOC    = EIC7700X_RESET_ID(0x39,  1),
  EIC7700X_RESET_PIPE_SNOC2PCIE   = EIC7700X_RESET_ID(0x39,  2),
  EIC7700X_RESET_PIPE_SNOC2MNOC   = EIC7700X_RESET_ID(0x39,  3),
  EIC7700X_RESET_PIPE_SNOC2DDR1P1 = EIC7700X_RESET_ID(0x39,  4),
  EIC7700X_RESET_PIPE_SNOC2DDR0P1 = EIC7700X_RESET_ID(0x39,  5),
  EIC7700X_RESET_PIPE_SNOC2DDR1P2 = EIC7700X_RESET_ID(0x39,  6),
  EIC7700X_RESET_PIPE_SNOC2DDR0P2 = EIC7700X_RESET_ID(0x39,  7),
  EIC7700X_RESET_PIPE_SNOC2D2D    = EIC7700X_RESET_ID(0x39,  8),
  EIC7700X_RESET_PIPE_MCPU2SNOC   = EIC7700X_RESET_ID(0x39,  9),
  EIC7700X_RESET_PIPE_MCPU2SNOCSP = EIC7700X_RESET_ID(0x39, 10),
  EIC7700X_RESET_PIPE_MNOC2SNOC   = EIC7700X_RESET_ID(0x39, 11),
  EIC7700X_RESET_PIPE_LNOC2DDR1   = EIC7700X_RESET_ID(0x39, 12),
  EIC7700X_RESET_PIPE_LNOC2DDR0   = EIC7700X_RESET_ID(0x39, 13),
  EIC7700X_RESET_PIPE_RNOC2DDR1   = EIC7700X_RESET_ID(0x39, 14),
  EIC7700X_RESET_PIPE_RNOC2DDR0   = EIC7700X_RESET_ID(0x39, 15),
  EIC7700X_RESET_PIPE_MNOC2DDR0   = EIC7700X_RESET_ID(0x39, 16),
  EIC7700X_RESET_PIPE_SNOC2DSP    = EIC7700X_RESET_ID(0x39, 17),
  EIC7700X_RESET_PIPE_DSP2SNOC    = EIC7700X_RESET_ID(0x39, 18),
  EIC7700X_RESET_PIPE_DSP2TCU     = EIC7700X_RESET_ID(0x39, 19),
  EIC7700X_RESET_PIPE_PCIE2TCU    = EIC7700X_RESET_ID(0x39, 20),
  EIC7700X_RESET_PIPE_VI2RNOC     = EIC7700X_RESET_ID(0x39, 21),
  EIC7700X_RESET_PIPE_VO2RNOC     = EIC7700X_RESET_ID(0x39, 22),
  EIC7700X_RESET_PIPE_GPU2MNOC    = EIC7700X_RESET_ID(0x39, 23),
  EIC7700X_RESET_PIPE_VC2MNOC     = EIC7700X_RESET_ID(0x39, 24),

  /* 0x4e8 tbu_rst_ctrl, translation buffer units (TRM p155).  Neither
   * Linux driver models this register.
   */

  EIC7700X_RESET_TBU_AON_A        = EIC7700X_RESET_ID(0x3a,  0),
  EIC7700X_RESET_TBU_PCIET_TBU3   = EIC7700X_RESET_ID(0x3a,  1),
  EIC7700X_RESET_TBU_HSP_A        = EIC7700X_RESET_ID(0x3a,  2),
  EIC7700X_RESET_TBU_VI_AXI       = EIC7700X_RESET_ID(0x3a,  3),

  /* 0x4ec reset_status (TRM p155).  The one read only line in the block:
   * it reports the reset state of the far die on a dual die part.  Only
   * reset_control_status() works on it; the write operations return
   * -EROFS.
   */

  EIC7700X_RESET_EXT_D2D_STATUS   = EIC7700X_RESET_ID(0x3b,  0),

  /* 0x4f0 test_rst_ctrl (TRM p155).  Neither Linux driver models this
   * register.
   */

  EIC7700X_RESET_TESTMUX          = EIC7700X_RESET_ID(0x3c,  0),
  EIC7700X_RESET_SPI_SLV          = EIC7700X_RESET_ID(0x3c,  1),
};

#endif /* __ARCH_RISCV_INCLUDE_EIC7700X_EIC7700X_RESET_H */
