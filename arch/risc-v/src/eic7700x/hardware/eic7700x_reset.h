/****************************************************************************
 * arch/risc-v/src/eic7700x/hardware/eic7700x_reset.h
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

#ifndef __ARCH_RISCV_SRC_EIC7700X_HARDWARE_EIC7700X_RESET_H
#define __ARCH_RISCV_SRC_EIC7700X_HARDWARE_EIC7700X_RESET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/eic7700x_clk.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register definitions transcribed from the EIC7700X SoC Technical
 * Reference Manual v1.0.0, Part 1 section 3.2.  Page numbers in the
 * comments below are PDF page numbers of Part 1; the printed page number
 * is the PDF page number minus 16.
 *
 * The reset controls are the second half of the Clock and Reset Generator
 * whose base is defined alongside the clock registers.  Offsets 0x400 to
 * 0x4f0 hold one control register per peripheral group (TRM p141 to p155).
 * The system control and boot configuration registers at 0x300 to 0x340
 * are deliberately omitted: none of them is a per peripheral reset.
 */

#define EIC7700X_RESET_BASE        (EIC7700X_CLK_BASE + 0x0400)
#define EIC7700X_RESET_CTRL(n)     (EIC7700X_RESET_BASE + 4 * (n))

/* Bits that name a reset line, one mask per control register.  Everything
 * else in each word is marked reserved and read only by the manual.
 *
 * Note that the Default column of the manual disagrees with the MSB and
 * LSB columns for 0x49c, 0x4d8 and 0x4ec, stating a width that does not
 * match the bit range.  These masks follow the bit range, which is the
 * self consistent one.
 */

#define SNOC_RST_VALID             0x0001fffful /* 0x400 p141, 17 lines   */
#define GPU_RST_VALID              0x0000001ful /* 0x404 p142,  5 lines   */
#define DSP_RST_VALID              0x000000f7ul /* 0x408 p143,  7 lines   */
#define D2D_RST_VALID              0x000000fful /* 0x40c p143,  8 lines   */
#define DDR_RST_VALID              0x05ff015ful /* 0x410 p143, 17 lines   */
#define TCU_RST_VALID              0x001ffff3ul /* 0x414 p144, 19 lines   */
#define NPU_RST_VALID              0x0000007ful /* 0x418 p144,  7 lines   */
#define HSPDMA_RST_VALID           0x0ffffffful /* 0x41c p145, 28 lines   */
#define PCIE_RST_VALID             0x00000007ul /* 0x420 p146,  3 lines   */
#define I2C_RST_VALID              0x000003fful /* 0x424 p146, 10 lines   */
#define FAN_RST_VALID              0x00000001ul /* 0x428 p146,  1 line    */
#define PVT_RST_VALID              0x00000003ul /* 0x42c p146,  2 lines   */
#define MBOX_RST_VALID             0x0000fffful /* 0x430 p146, 16 lines   */
#define UART_RST_VALID             0x0000001ful /* 0x434 p147,  5 lines   */

/* 0x438 is absent from the manual, whose table goes straight from 0x434
 * to 0x43c.  The vendor Linux binding names the gap gpio_rst_ctrl with
 * one bit per controller, and the SoC has two GPIO blocks that would
 * otherwise have no reset at all.  The reset default is unknown, so the
 * page column below reads as a gap rather than a citation.
 */

#define GPIO_RST_VALID             0x00000003ul /* 0x438:  ,  2 lines   */
#define TIMER_RST_VALID            0x00000001ul /* 0x43c p147,  1 line    */
#define SSI_RST_VALID              0x00000003ul /* 0x440 p147,  2 lines   */
#define WDT_RST_VALID              0x0000000ful /* 0x444 p147,  4 lines   */
#define LSPCFG_RST_VALID           0x00000001ul /* 0x448 p147,  1 line    */
#define U84_RST_VALID              0x00000f7ful /* 0x44c p147, 11 lines   */
#define SCPU_RST_VALID             0x00000007ul /* 0x450 p147,  3 lines   */
#define LPCPU_RST_VALID            0x00000007ul /* 0x454 p147,  3 lines   */
#define DMA1_RST_VALID             0x00000003ul /* 0x49c p150,  2 lines   */
#define VC_RST_VALID               0x00000007ul /* 0x458 p148,  3 lines   */
#define JD_RST_VALID               0x00000003ul /* 0x45c p148,  2 lines   */
#define JE_RST_VALID               0x00000003ul /* 0x460 p148,  2 lines   */
#define VD_RST_VALID               0x00000003ul /* 0x464 p148,  2 lines   */
#define VE_RST_VALID               0x00000003ul /* 0x468 p148,  2 lines   */
#define G2D_RST_VALID              0x00000007ul /* 0x46c p148,  3 lines   */
#define VI_RST_VALID               0x00000007ul /* 0x470 p149,  3 lines   */
#define DVP_RST_VALID              0x00000001ul /* 0x474 p149,  1 line    */
#define ISP0_RST_VALID             0x00000001ul /* 0x478 p149,  1 line    */
#define ISP1_RST_VALID             0x00000001ul /* 0x47c p149,  1 line    */
#define SHUTTER_RST_VALID          0x0000003ful /* 0x480 p149,  6 lines   */
#define VOPHY_RST_VALID            0x0000003bul /* 0x484 p149,  5 lines   */
#define VOI2S_RST_VALID            0x00000003ul /* 0x488 p149,  2 lines   */
#define VO_RST_VALID               0x0000000ful /* 0x48c p150,  4 lines   */
#define BOOTSPI_RST_VALID          0x00000003ul /* 0x490 p150,  2 lines   */
#define I2C1_RST_VALID             0x00000001ul /* 0x494 p150,  1 line    */
#define I2C0_RST_VALID             0x00000001ul /* 0x498 p150,  1 line    */
#define FPRT_RST_VALID             0x00000001ul /* 0x4a0 p150,  1 line    */
#define HBLOCK_RST_VALID           0x00000001ul /* 0x4a4 p150,  1 line    */
#define SECSR_RST_VALID            0x00000001ul /* 0x4a8 p150,  1 line    */
#define OTP_RST_VALID              0x00000001ul /* 0x4ac p150,  1 line    */
#define PKA_RST_VALID              0x00000001ul /* 0x4b0 p151,  1 line    */
#define SPACC_RST_VALID            0x00000001ul /* 0x4b4 p151,  1 line    */
#define TRNG_RST_VALID             0x00000001ul /* 0x4b8 p151,  1 line    */
#define TIMER0_RST_VALID           0x000001fful /* 0x4c0 p151,  9 lines   */
#define TIMER1_RST_VALID           0x000001fful /* 0x4c4 p151,  9 lines   */
#define TIMER2_RST_VALID           0x000001fful /* 0x4c8 p151,  9 lines   */
#define TIMER3_RST_VALID           0x000001fful /* 0x4cc p151,  9 lines   */
#define RTC_RST_VALID              0x00000001ul /* 0x4d0 p151,  1 line    */
#define MNOC_RST_VALID             0x0000007ful /* 0x4d4 p151,  7 lines   */
#define RNOC_RST_VALID             0x0000003ful /* 0x4d8 p152,  6 lines   */
#define CNOC_RST_VALID             0x0000fffful /* 0x4dc p152, 16 lines   */
#define LNOC_RST_VALID             0x0000000ful /* 0x4e0 p153,  4 lines   */
#define PIPE_RST_VALID             0x01fffffful /* 0x4e4 p153, 25 lines   */
#define TBU_RST_VALID              0x0000000ful /* 0x4e8 p155,  4 lines   */
#define STATUS_RST_VALID           0x00000001ul /* 0x4ec p155,  1 line    */
#define TEST_RST_VALID             0x00000003ul /* 0x4f0 p155,  2 lines   */

#endif /* __ARCH_RISCV_SRC_EIC7700X_HARDWARE_EIC7700X_RESET_H */
