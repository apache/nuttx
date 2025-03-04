/****************************************************************************
 * arch/risc-v/src/mpfs/hardware/mpfs_mpucfg.h
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

#ifndef __ARCH_RISC_V_SRC_MPFS_HARDWARE_MPFS_MPUCFG_H
#define __ARCH_RISC_V_SRC_MPFS_HARDWARE_MPFS_MPUCFG_H

/* FIC0 (FPGA) PMP configurations - for fabric memory transfers */

#define MPFS_PMPCFG_FIC0_0            (MPFS_MPUCFG_BASE + 0x00)
#define MPFS_PMPCFG_FIC0_1            (MPFS_MPUCFG_BASE + 0x08)
#define MPFS_PMPCFG_FIC0_2            (MPFS_MPUCFG_BASE + 0x10)
#define MPFS_PMPCFG_FIC0_3            (MPFS_MPUCFG_BASE + 0x18)
#define MPFS_PMPCFG_FIC0_4            (MPFS_MPUCFG_BASE + 0x20)
#define MPFS_PMPCFG_FIC0_5            (MPFS_MPUCFG_BASE + 0x28)
#define MPFS_PMPCFG_FIC0_6            (MPFS_MPUCFG_BASE + 0x30)
#define MPFS_PMPCFG_FIC0_7            (MPFS_MPUCFG_BASE + 0x38)
#define MPFS_PMPCFG_FIC0_8            (MPFS_MPUCFG_BASE + 0x40)
#define MPFS_PMPCFG_FIC0_9            (MPFS_MPUCFG_BASE + 0x48)
#define MPFS_PMPCFG_FIC0_10           (MPFS_MPUCFG_BASE + 0x50)
#define MPFS_PMPCFG_FIC0_11           (MPFS_MPUCFG_BASE + 0x58)
#define MPFS_PMPCFG_FIC0_12           (MPFS_MPUCFG_BASE + 0x60)
#define MPFS_PMPCFG_FIC0_13           (MPFS_MPUCFG_BASE + 0x68)
#define MPFS_PMPCFG_FIC0_14           (MPFS_MPUCFG_BASE + 0x70)
#define MPFS_PMPCFG_FIC0_15           (MPFS_MPUCFG_BASE + 0x78)

/* FIC1 (FPGA) PMP configurations - for fabric memory transfers */

#define MPFS_PMPCFG_FIC1_0            (MPFS_MPUCFG_BASE + 0x100)
#define MPFS_PMPCFG_FIC1_1            (MPFS_MPUCFG_BASE + 0x108)
#define MPFS_PMPCFG_FIC1_2            (MPFS_MPUCFG_BASE + 0x110)
#define MPFS_PMPCFG_FIC1_3            (MPFS_MPUCFG_BASE + 0x118)
#define MPFS_PMPCFG_FIC1_4            (MPFS_MPUCFG_BASE + 0x120)
#define MPFS_PMPCFG_FIC1_5            (MPFS_MPUCFG_BASE + 0x128)
#define MPFS_PMPCFG_FIC1_6            (MPFS_MPUCFG_BASE + 0x130)
#define MPFS_PMPCFG_FIC1_7            (MPFS_MPUCFG_BASE + 0x138)
#define MPFS_PMPCFG_FIC1_8            (MPFS_MPUCFG_BASE + 0x140)
#define MPFS_PMPCFG_FIC1_9            (MPFS_MPUCFG_BASE + 0x148)
#define MPFS_PMPCFG_FIC1_10           (MPFS_MPUCFG_BASE + 0x150)
#define MPFS_PMPCFG_FIC1_11           (MPFS_MPUCFG_BASE + 0x158)
#define MPFS_PMPCFG_FIC1_12           (MPFS_MPUCFG_BASE + 0x160)
#define MPFS_PMPCFG_FIC1_13           (MPFS_MPUCFG_BASE + 0x168)
#define MPFS_PMPCFG_FIC1_14           (MPFS_MPUCFG_BASE + 0x170)
#define MPFS_PMPCFG_FIC1_15           (MPFS_MPUCFG_BASE + 0x178)

/* FIC2 (FPGA) PMP configurations - for fabric memory transfers */

#define MPFS_PMPCFG_FIC2_0            (MPFS_MPUCFG_BASE + 0x200)
#define MPFS_PMPCFG_FIC2_1            (MPFS_MPUCFG_BASE + 0x208)
#define MPFS_PMPCFG_FIC2_2            (MPFS_MPUCFG_BASE + 0x210)
#define MPFS_PMPCFG_FIC2_3            (MPFS_MPUCFG_BASE + 0x218)
#define MPFS_PMPCFG_FIC2_4            (MPFS_MPUCFG_BASE + 0x220)
#define MPFS_PMPCFG_FIC2_5            (MPFS_MPUCFG_BASE + 0x228)
#define MPFS_PMPCFG_FIC2_6            (MPFS_MPUCFG_BASE + 0x230)
#define MPFS_PMPCFG_FIC2_7            (MPFS_MPUCFG_BASE + 0x238)

/* Crpyto PMP configurations - for DMA transfers */

#define MPFS_PMPCFG_CRYPTO_0          (MPFS_MPUCFG_BASE + 0x300)
#define MPFS_PMPCFG_CRYPTO_1          (MPFS_MPUCFG_BASE + 0x308)
#define MPFS_PMPCFG_CRYPTO_2          (MPFS_MPUCFG_BASE + 0x310)
#define MPFS_PMPCFG_CRYPTO_3          (MPFS_MPUCFG_BASE + 0x318)

/* Ethernet PMP configurations - for DMA transfers */

#define MPFS_PMPCFG_ETH0_0            (MPFS_MPUCFG_BASE + 0x400)
#define MPFS_PMPCFG_ETH0_1            (MPFS_MPUCFG_BASE + 0x408)
#define MPFS_PMPCFG_ETH0_2            (MPFS_MPUCFG_BASE + 0x410)
#define MPFS_PMPCFG_ETH0_3            (MPFS_MPUCFG_BASE + 0x418)
#define MPFS_PMPCFG_ETH0_4            (MPFS_MPUCFG_BASE + 0x420)
#define MPFS_PMPCFG_ETH0_5            (MPFS_MPUCFG_BASE + 0x428)
#define MPFS_PMPCFG_ETH0_6            (MPFS_MPUCFG_BASE + 0x430)
#define MPFS_PMPCFG_ETH0_7            (MPFS_MPUCFG_BASE + 0x438)
#define MPFS_PMPCFG_ETH1_0            (MPFS_MPUCFG_BASE + 0x500)
#define MPFS_PMPCFG_ETH1_1            (MPFS_MPUCFG_BASE + 0x508)
#define MPFS_PMPCFG_ETH1_2            (MPFS_MPUCFG_BASE + 0x510)
#define MPFS_PMPCFG_ETH1_3            (MPFS_MPUCFG_BASE + 0x518)
#define MPFS_PMPCFG_ETH1_4            (MPFS_MPUCFG_BASE + 0x520)
#define MPFS_PMPCFG_ETH1_5            (MPFS_MPUCFG_BASE + 0x528)
#define MPFS_PMPCFG_ETH1_6            (MPFS_MPUCFG_BASE + 0x530)
#define MPFS_PMPCFG_ETH1_7            (MPFS_MPUCFG_BASE + 0x528)

/* USB PMP configurations - for DMA transfers */

#define MPFS_PMPCFG_USB_0             (MPFS_MPUCFG_BASE + 0x600)
#define MPFS_PMPCFG_USB_1             (MPFS_MPUCFG_BASE + 0x608)
#define MPFS_PMPCFG_USB_2             (MPFS_MPUCFG_BASE + 0x610)
#define MPFS_PMPCFG_USB_3             (MPFS_MPUCFG_BASE + 0x618)

/* MMC PMP configurations - for DMA transfers */

#define MPFS_PMPCFG_MMC_0             (MPFS_MPUCFG_BASE + 0x700)
#define MPFS_PMPCFG_MMC_1             (MPFS_MPUCFG_BASE + 0x708)
#define MPFS_PMPCFG_MMC_2             (MPFS_MPUCFG_BASE + 0x710)
#define MPFS_PMPCFG_MMC_3             (MPFS_MPUCFG_BASE + 0x718)

/* SCB PMP configurations - for DMA transfers */

#define MPFS_PMPCFG_SCB_0             (MPFS_MPUCFG_BASE + 0x800)
#define MPFS_PMPCFG_SCB_1             (MPFS_MPUCFG_BASE + 0x808)
#define MPFS_PMPCFG_SCB_2             (MPFS_MPUCFG_BASE + 0x810)
#define MPFS_PMPCFG_SCB_3             (MPFS_MPUCFG_BASE + 0x818)
#define MPFS_PMPCFG_SCB_4             (MPFS_MPUCFG_BASE + 0x820)
#define MPFS_PMPCFG_SCB_5             (MPFS_MPUCFG_BASE + 0x828)
#define MPFS_PMPCFG_SCB_6             (MPFS_MPUCFG_BASE + 0x830)
#define MPFS_PMPCFG_SCB_7             (MPFS_MPUCFG_BASE + 0x838)

/* TRACE PMP configurations - for DMA transfers */

#define MPFS_PMPCFG_TRACE_0           (MPFS_MPUCFG_BASE + 0x900)
#define MPFS_PMPCFG_TRACE_1           (MPFS_MPUCFG_BASE + 0x908)

/* DDR segments - set up by mpfs_ddr.c */

#define MPFS_MPUCFG_SEG0_REG0         (MPFS_MPUCFG_BASE + 0xd00)
#define MPFS_MPUCFG_SEG0_REG1         (MPFS_MPUCFG_BASE + 0xd08)
#define MPFS_MPUCFG_SEG0_REG2         (MPFS_MPUCFG_BASE + 0xd10)
#define MPFS_MPUCFG_SEG0_REG3         (MPFS_MPUCFG_BASE + 0xd18)
#define MPFS_MPUCFG_SEG0_REG4         (MPFS_MPUCFG_BASE + 0xd20)
#define MPFS_MPUCFG_SEG0_REG5         (MPFS_MPUCFG_BASE + 0xd28)
#define MPFS_MPUCFG_SEG0_REG6         (MPFS_MPUCFG_BASE + 0xd30)

#define MPFS_MPUCFG_SEG1_REG0         (MPFS_MPUCFG_BASE + 0xe00)
#define MPFS_MPUCFG_SEG1_REG1         (MPFS_MPUCFG_BASE + 0xe08)
#define MPFS_MPUCFG_SEG1_REG2         (MPFS_MPUCFG_BASE + 0xe10)
#define MPFS_MPUCFG_SEG1_REG3         (MPFS_MPUCFG_BASE + 0xe18)
#define MPFS_MPUCFG_SEG1_REG4         (MPFS_MPUCFG_BASE + 0xe20)
#define MPFS_MPUCFG_SEG1_REG5         (MPFS_MPUCFG_BASE + 0xe28)
#define MPFS_MPUCFG_SEG1_REG6         (MPFS_MPUCFG_BASE + 0xe30)

/* Size of the register area, which is 4K */

#define MPFS_MPUCFG_SIZE              (0x1000)
#define MPFS_MPUCFG_END               (MPFS_MPUCFG_BASE + MPFS_MPUCFG_SIZE)

#endif /* __NUTTX_ARCH_RISC_V_SRC_MPFS_HARDWARE_MPFS_MPUCFG_H */
