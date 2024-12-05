/*********************************************************************************
 * arch/arm/src/imxrt/hardware/rt117x/imxrt117x_ocotp.h
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
 *********************************************************************************/

/* Copyright 2022 NXP */

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_RT117X_IMXRT117X_OCOTP_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_RT117X_IMXRT117X_OCOTP_H

/* OCOTP Register Offsets ********************************************************/
#define IMXRT_OCOTP_CTRL_OFFSET             (0x0000)
#define IMXRT_OCOTP_CTRL_SET_OFFSET         (0x0004)
#define IMXRT_OCOTP_CTRL_CLR_OFFSET         (0x0008)
#define IMXRT_OCOTP_CTRL_TOG_OFFSET         (0x000c)
#define IMXRT_OCOTP_PDN_OFFSET              (0x0010)
#define IMXRT_OCOTP_DATA_OFFSET             (0x0020)
#define IMXRT_OCOTP_READ_CTRL_OFFSET        (0x0030)
#define IMXRT_OCOTP_OUT_STATUS_OFFSET       (0x0090)
#define IMXRT_OCOTP_OUT_STATUS_SET_OFFSET   (0x0094)
#define IMXRT_OCOTP_OUT_STATUS_CLR_OFFSET   (0x0098)
#define IMXRT_OCOTP_OUT_STATUS_TOG_OFFSET   (0x009c)
#define IMXRT_OCOTP_VERSION_OFFSET          (0x00b0)
#define IMXRT_OCOTP_READ_FUSE_DATA0_OFFSET  (0x0100)
#define IMXRT_OCOTP_READ_FUSE_DATA1_OFFSET  (0x0110)
#define IMXRT_OCOTP_READ_FUSE_DATA2_OFFSET  (0x0120)
#define IMXRT_OCOTP_READ_FUSE_DATA3_OFFSET  (0x0130)
#define IMXRT_OCOTP_SW_LOCK_OFFSET          (0x0140)
#define IMXRT_OCOTP_BIT_LOCK_OFFSET         (0x0150)
#define IMXRT_OCOTP_LOCKED0_OFFSET          (0x0600)
#define IMXRT_OCOTP_LOCKED1_OFFSET          (0x0610)
#define IMXRT_OCOTP_LOCKED2_OFFSET          (0x0620)
#define IMXRT_OCOTP_LOCKED3_OFFSET          (0x0630)
#define IMXRT_OCOTP_LOCKED4_OFFSET          (0x0640)
#define IMXRT_OCOTP_FUSE_OFFSET             (0x0800)

/* OCOTP Register Addresses ******************************************************/
#define IMXRT_OCOTP_CTRL             (IMXRT_OCOTP_BASE + IMXRT_OCOTP_CTRL_OFFSET)
#define IMXRT_OCOTP_CTRL_SET         (IMXRT_OCOTP_BASE + IMXRT_OCOTP_CTRL_SET_OFFSET)
#define IMXRT_OCOTP_CTRL_CLR         (IMXRT_OCOTP_BASE + IMXRT_OCOTP_CTRL_CLR_OFFSET)
#define IMXRT_OCOTP_CTRL_TOG         (IMXRT_OCOTP_BASE + IMXRT_OCOTP_CTRL_TOG_OFFSET)
#define IMXRT_OCOTP_PDN              (IMXRT_OCOTP_BASE + IMXRT_OCOTP_PDN_OFFSET)
#define IMXRT_OCOTP_DATA             (IMXRT_OCOTP_BASE + IMXRT_OCOTP_DATA_OFFSET)
#define IMXRT_OCOTP_READ_CTRL        (IMXRT_OCOTP_BASE + IMXRT_OCOTP_READ_CTRL_OFFSET)
#define IMXRT_OCOTP_OUT_STATUS       (IMXRT_OCOTP_BASE + IMXRT_OCOTP_OUT_STATUS_OFFSET)
#define IMXRT_OCOTP_OUT_STATUS_SET   (IMXRT_OCOTP_BASE + IMXRT_OCOTP_OUT_STATUS_SET_OFFSET)
#define IMXRT_OCOTP_OUT_STATUS_CLR   (IMXRT_OCOTP_BASE + IMXRT_OCOTP_OUT_STATUS_CLR_OFFSET)
#define IMXRT_OCOTP_OUT_STATUS_TOG   (IMXRT_OCOTP_BASE + IMXRT_OCOTP_OUT_STATUS_TOG_OFFSET)
#define IMXRT_OCOTP_VERSION          (IMXRT_OCOTP_BASE + IMXRT_OCOTP_VERSION_OFFSET)
#define IMXRT_OCOTP_READ_FUSE_DATA0  (IMXRT_OCOTP_BASE + IMXRT_OCOTP_READ_FUSE_DATA0_OFFSET)
#define IMXRT_OCOTP_READ_FUSE_DATA1  (IMXRT_OCOTP_BASE + IMXRT_OCOTP_READ_FUSE_DATA1_OFFSET)
#define IMXRT_OCOTP_READ_FUSE_DATA2  (IMXRT_OCOTP_BASE + IMXRT_OCOTP_READ_FUSE_DATA2_OFFSET)
#define IMXRT_OCOTP_READ_FUSE_DATA3  (IMXRT_OCOTP_BASE + IMXRT_OCOTP_READ_FUSE_DATA3_OFFSET)
#define IMXRT_OCOTP_SW_LOCK          (IMXRT_OCOTP_BASE + IMXRT_OCOTP_SW_LOCK_OFFSET)
#define IMXRT_OCOTP_BIT_LOCK         (IMXRT_OCOTP_BASE + IMXRT_OCOTP_BIT_LOCK_OFFSET)
#define IMXRT_OCOTP_LOCKED0          (IMXRT_OCOTP_BASE + IMXRT_OCOTP_LOCKED0_OFFSET)
#define IMXRT_OCOTP_LOCKED1          (IMXRT_OCOTP_BASE + IMXRT_OCOTP_LOCKED1_OFFSET)
#define IMXRT_OCOTP_LOCKED2          (IMXRT_OCOTP_BASE + IMXRT_OCOTP_LOCKED2_OFFSET)
#define IMXRT_OCOTP_LOCKED3          (IMXRT_OCOTP_BASE + IMXRT_OCOTP_LOCKED3_OFFSET)
#define IMXRT_OCOTP_LOCKED4          (IMXRT_OCOTP_BASE + IMXRT_OCOTP_LOCKED4_OFFSET)
#define IMXRT_OCOTP_FUSE_BASE        (IMXRT_OCOTP_BASE + IMXRT_OCOTP_FUSE_OFFSET)
#define IMXRT_OCOTP_FUSE(n)          (IMXRT_OCOTP_FUSE_BASE + (n * 0x10))

/* 64 Bit unique id consisting of:
 * LOT_NO_ENC[42:0] 42 bits LOT ID
 *     IMXRT_OCOTP_UNIQUE_ID_MSB[31:0] IMXRT_OCOTP_UNIQUE_ID_LSB[10:0]
 * WAFER_NO[4:0] 5 bits The wafer number of the wafer on which the device
 * was fabricated IMXRT_OCOTP_UNIQUE_ID_LSB[15:11]
 * DIE-YCORDINATE[7:0] 8 bits The Y-coordinate of the die location on the wafer
 *     IMXRT_OCOTP_UNIQUE_ID_LSB[23:16]
 * DIE-XCORDINATE[7:0] 8 bits The X-coordinate of the die location on the wafer
 *     IMXRT_OCOTP_UNIQUE_ID_LSB[31:24]
 */
#define IMXRT_OCOTP_UNIQUE_ID_MSB    (IMXRT_OCOTP_FUSE(0x10)) /* Most Significant Bytes of 64 bit UUID */
#define IMXRT_OCOTP_UNIQUE_ID_LSB    (IMXRT_OCOTP_FUSE(0x11)) /* Least Significant Bytes of 64 bit UUID */

/* OTP Controller Control and Status Register (CTRL) */
#define OCOTP_CTRL_ADDR_SHIFT       (0)        /* Bits 0-10: OTP write and read access address register */
#define OCOTP_CTRL_ADDR_MASK        (0x3FF << OCOTP_CTRL_ADDR_SHIFT)
#define OCOTP_CTRL_ADDR(n)          (((n) << OCOTP_CTRL_ADDR_SHIFT) & OCOTP_CTRL_ADDR_MASK)
#define OCOTP_CTRL_BUSY             (1 << 10)  /* Bit 10: OTP controller status bit */
#define OCOTP_CTRL_ERROR            (1 << 11)  /* Bit 11: Locked Region Access Error */
#define OCOTP_CTRL_RELOAD_SHADOWS   (1 << 12)  /* Bit 12: Reload Shadow Registers */
#define OCOTP_CTRL_WORDLOCK         (1 << 15)  /* Bit 15: Lock fuse word */
#define OCOTP_CTRL_WR_UNLOCK_SHIFT  (16)       /* Bits 16-32: Write unlock */
#define OCOTP_CTRL_WR_UNLOCK_MASK   (0xFFFF << OCOTP_CTRL_WR_UNLOCK_SHIFT)
#define OCOTP_CTRL_WR_UNLOCK(n)     (((n) << OCOTP_CTRL_WR_UNLOCK_SHIFT) & OCOTP_CTRL_WR_UNLOCK_MASK)

/* OTP Controller Control and Status Register (CTRL_SET) */
#define OCOTP_CTRL_SET_ADDR_SHIFT       (0)        /* Bits 0-10: OTP write and read access address register */
#define OCOTP_CTRL_SET_ADDR_MASK        (0x3FF << OCOTP_CTRL_SET_ADDR_SHIFT)
#define OCOTP_CTRL_SET_ADDR(n)          (((n) << OCOTP_CTRL_SET_ADDR_SHIFT) & OCOTP_CTRL_SET_ADDR_MASK)
#define OCOTP_CTRL_SET_BUSY             (1 << 10)  /* Bit 10: OTP controller status bit */
#define OCOTP_CTRL_SET_ERROR            (1 << 11)  /* Bit 11: Locked Region Access Error */
#define OCOTP_CTRL_SET_RELOAD_SHADOWS   (1 << 12)  /* Bit 12: Reload Shadow Registers */
#define OCOTP_CTRL_SET_WORDLOCK         (1 << 15)  /* Bit 15: Lock fuse word */
#define OCOTP_CTRL_SET_WR_UNLOCK_SHIFT  (16)       /* Bits 16-32: Write unlock */
#define OCOTP_CTRL_SET_WR_UNLOCK_MASK   (0xFFFF << OCOTP_CTRL_SET_WR_UNLOCK_SHIFT)
#define OCOTP_CTRL_SET_WR_UNLOCK(n)     (((n) << OCOTP_CTRL_SET_WR_UNLOCK_SHIFT) & OCOTP_CTRL_SET_WR_UNLOCK_MASK)

/* OTP Controller Control and Status Register (CTRL_CLR) */
#define OCOTP_CTRL_CLR_ADDR_SHIFT       (0)        /* Bits 0-10: OTP write and read access address register */
#define OCOTP_CTRL_CLR_ADDR_MASK        (0x3FF << OCOTP_CTRL_CLR_ADDR_SHIFT)
#define OCOTP_CTRL_CLR_ADDR(n)          (((n) << OCOTP_CTRL_CLR_ADDR_SHIFT) & OCOTP_CTRL_CLR_ADDR_MASK)
#define OCOTP_CTRL_CLR_BUSY             (1 << 10)  /* Bit 10: OTP controller status bit */
#define OCOTP_CTRL_CLR_ERROR            (1 << 11)  /* Bit 11: Locked Region Access Error */
#define OCOTP_CTRL_CLR_RELOAD_SHADOWS   (1 << 12)  /* Bit 12: Reload Shadow Registers */
#define OCOTP_CTRL_CLR_WORDLOCK         (1 << 15)  /* Bit 15: Lock fuse word */
#define OCOTP_CTRL_CLR_WR_UNLOCK_SHIFT  (16)       /* Bits 16-32: Write unlock */
#define OCOTP_CTRL_CLR_WR_UNLOCK_MASK   (0xFFFF << OCOTP_CTRL_CLR_WR_UNLOCK_SHIFT)
#define OCOTP_CTRL_CLR_WR_UNLOCK(n)     (((n) << OCOTP_CTRL_CLR_WR_UNLOCK_SHIFT) & OCOTP_CTRL_CLR_WR_UNLOCK_MASK)

/* OTP Controller Control and Status Register (CTRL_TOG) */
#define OCOTP_CTRL_TOG_ADDR_SHIFT       (0)        /* Bits 0-10: OTP write and read access address register */
#define OCOTP_CTRL_TOG_ADDR_MASK        (0x3FF << OCOTP_CTRL_TOG_ADDR_SHIFT)
#define OCOTP_CTRL_TOG_ADDR(n)          (((n) << OCOTP_CTRL_TOG_ADDR_SHIFT) & OCOTP_CTRL_TOG_ADDR_MASK)
#define OCOTP_CTRL_TOG_BUSY             (1 << 10)  /* Bit 10: OTP controller status bit */
#define OCOTP_CTRL_TOG_ERROR            (1 << 11)  /* Bit 11: Locked Region Access Error */
#define OCOTP_CTRL_TOG_RELOAD_SHADOWS   (1 << 12)  /* Bit 12: Reload Shadow Registers */
#define OCOTP_CTRL_TOG_WORDLOCK         (1 << 15)  /* Bit 15: Lock fuse word */
#define OCOTP_CTRL_TOG_WR_UNLOCK_SHIFT  (16)       /* Bits 16-32: Write unlock */
#define OCOTP_CTRL_TOG_WR_UNLOCK_MASK   (0xFFFF << OCOTP_CTRL_TOG_WR_UNLOCK_SHIFT)
#define OCOTP_CTRL_TOG_WR_UNLOCK(n)     (((n) << OCOTP_CTRL_TOG_WR_UNLOCK_SHIFT) & OCOTP_CTRL_TOG_WR_UNLOCK_MASK)

/* OTP Controller PDN Register (PDN) */
#define OCOTP_PDN_PDN  (1 << 0)  /* Bit 0: PDN value */

/* OTP Controller Write Data Register (DATA) */
#define OCOTP_DATA_DATA_SHIFT  (0)  /* Bits 0-32: Data */
#define OCOTP_DATA_DATA_MASK   (0xFFFFFFFF << OCOTP_DATA_DATA_SHIFT)
#define OCOTP_DATA_DATA(n)     (((n) << OCOTP_DATA_DATA_SHIFT) & OCOTP_DATA_DATA_MASK)

/* OTP Controller Read Control Register (READ_CTRL) */
#define OCOTP_READ_CTRL_READ_FUSE                 (1 << 0)  /* Bit 0: Read Fuse */
#define OCOTP_READ_CTRL_READ_FUSE_CNTR_SHIFT      (1)       /* Bits 1-3: Number of words to read. */
#define OCOTP_READ_CTRL_READ_FUSE_CNTR_MASK       (0x3 << OCOTP_READ_CTRL_READ_FUSE_CNTR_SHIFT)
#define OCOTP_READ_CTRL_READ_FUSE_CNTR(n)         (((n) << OCOTP_READ_CTRL_READ_FUSE_CNTR_SHIFT) & OCOTP_READ_CTRL_READ_FUSE_CNTR_MASK)
#define OCOTP_READ_CTRL_READ_FUSE_DONE_INTR_ENA   (1 << 3)  /* Bit 3: Enable read-done interrupt */
#define OCOTP_READ_CTRL_READ_FUSE_ERROR_INTR_ENA  (1 << 4)  /* Bit 4: Enable read-error interrupt */

/* 8K OTP Memory STATUS Register (OUT_STATUS) */
#define OCOTP_OUT_STATUS_SEC              (1 << 9)   /* Bit 9: Single Error Correct */
#define OCOTP_OUT_STATUS_DED              (1 << 10)  /* Bit 10: Double error detect */
#define OCOTP_OUT_STATUS_LOCKED           (1 << 11)  /* Bit 11: Word Locked */
#define OCOTP_OUT_STATUS_PROGFAIL         (1 << 12)  /* Bit 12: Programming failed */
#define OCOTP_OUT_STATUS_ACK              (1 << 13)  /* Bit 13: Acknowledge */
#define OCOTP_OUT_STATUS_PWOK             (1 << 14)  /* Bit 14: Power OK */
#define OCOTP_OUT_STATUS_FLAGSTATE_SHIFT  (15)       /* Bits 15-19: Flag state */
#define OCOTP_OUT_STATUS_FLAGSTATE_MASK   (0xF << OCOTP_OUT_STATUS_FLAGSTATE_SHIFT)
#define OCOTP_OUT_STATUS_FLAGSTATE(n)     (((n) << OCOTP_OUT_STATUS_FLAGSTATE_SHIFT) & OCOTP_OUT_STATUS_FLAGSTATE_MASK)
#define OCOTP_OUT_STATUS_SEC_RELOAD       (1 << 19)  /* Bit 19: Indicates single error correction occured on reload */
#define OCOTP_OUT_STATUS_DED_RELOAD       (1 << 20)  /* Bit 20: Indicates double error detection occured on reload */
#define OCOTP_OUT_STATUS_CALIBRATED       (1 << 21)  /* Bit 21: Calibrated status */
#define OCOTP_OUT_STATUS_READ_DONE_INTR   (1 << 22)  /* Bit 22: Read fuse done */
#define OCOTP_OUT_STATUS_READ_ERROR_INTR  (1 << 23)  /* Bit 23: Fuse read error */
#define OCOTP_OUT_STATUS_DED0             (1 << 24)  /* Bit 24: Double error detect */
#define OCOTP_OUT_STATUS_DED1             (1 << 25)  /* Bit 25: Double error detect */
#define OCOTP_OUT_STATUS_DED2             (1 << 26)  /* Bit 26: Double error detect */
#define OCOTP_OUT_STATUS_DED3             (1 << 27)  /* Bit 27: Double error detect */

/* 8K OTP Memory STATUS Register (OUT_STATUS_SET) */
#define OCOTP_OUT_STATUS_SET_SEC              (1 << 9)   /* Bit 9: Single Error Correct */
#define OCOTP_OUT_STATUS_SET_DED              (1 << 10)  /* Bit 10: Double error detect */
#define OCOTP_OUT_STATUS_SET_LOCKED           (1 << 11)  /* Bit 11: Word Locked */
#define OCOTP_OUT_STATUS_SET_PROGFAIL         (1 << 12)  /* Bit 12: Programming failed */
#define OCOTP_OUT_STATUS_SET_ACK              (1 << 13)  /* Bit 13: Acknowledge */
#define OCOTP_OUT_STATUS_SET_PWOK             (1 << 14)  /* Bit 14: Power OK */
#define OCOTP_OUT_STATUS_SET_FLAGSTATE_SHIFT  (15)       /* Bits 15-19: Flag state */
#define OCOTP_OUT_STATUS_SET_FLAGSTATE_MASK   (0xF << OCOTP_OUT_STATUS_SET_FLAGSTATE_SHIFT)
#define OCOTP_OUT_STATUS_SET_FLAGSTATE(n)     (((n) << OCOTP_OUT_STATUS_SET_FLAGSTATE_SHIFT) & OCOTP_OUT_STATUS_SET_FLAGSTATE_MASK)
#define OCOTP_OUT_STATUS_SET_SEC_RELOAD       (1 << 19)  /* Bit 19: Indicates single error correction occured on reload */
#define OCOTP_OUT_STATUS_SET_DED_RELOAD       (1 << 20)  /* Bit 20: Indicates double error detection occured on reload */
#define OCOTP_OUT_STATUS_SET_CALIBRATED       (1 << 21)  /* Bit 21: Calibrated status */
#define OCOTP_OUT_STATUS_SET_READ_DONE_INTR   (1 << 22)  /* Bit 22: Read fuse done */
#define OCOTP_OUT_STATUS_SET_READ_ERROR_INTR  (1 << 23)  /* Bit 23: Fuse read error */
#define OCOTP_OUT_STATUS_SET_DED0             (1 << 24)  /* Bit 24: Double error detect */
#define OCOTP_OUT_STATUS_SET_DED1             (1 << 25)  /* Bit 25: Double error detect */
#define OCOTP_OUT_STATUS_SET_DED2             (1 << 26)  /* Bit 26: Double error detect */
#define OCOTP_OUT_STATUS_SET_DED3             (1 << 27)  /* Bit 27: Double error detect */

/* 8K OTP Memory STATUS Register (OUT_STATUS_CLR) */
#define OCOTP_OUT_STATUS_CLR_SEC              (1 << 9)   /* Bit 9: Single Error Correct */
#define OCOTP_OUT_STATUS_CLR_DED              (1 << 10)  /* Bit 10: Double error detect */
#define OCOTP_OUT_STATUS_CLR_LOCKED           (1 << 11)  /* Bit 11: Word Locked */
#define OCOTP_OUT_STATUS_CLR_PROGFAIL         (1 << 12)  /* Bit 12: Programming failed */
#define OCOTP_OUT_STATUS_CLR_ACK              (1 << 13)  /* Bit 13: Acknowledge */
#define OCOTP_OUT_STATUS_CLR_PWOK             (1 << 14)  /* Bit 14: Power OK */
#define OCOTP_OUT_STATUS_CLR_FLAGSTATE_SHIFT  (15)       /* Bits 15-19: Flag state */
#define OCOTP_OUT_STATUS_CLR_FLAGSTATE_MASK   (0xF << OCOTP_OUT_STATUS_CLR_FLAGSTATE_SHIFT)
#define OCOTP_OUT_STATUS_CLR_FLAGSTATE(n)     (((n) << OCOTP_OUT_STATUS_CLR_FLAGSTATE_SHIFT) & OCOTP_OUT_STATUS_CLR_FLAGSTATE_MASK)
#define OCOTP_OUT_STATUS_CLR_SEC_RELOAD       (1 << 19)  /* Bit 19: Indicates single error correction occured on reload */
#define OCOTP_OUT_STATUS_CLR_DED_RELOAD       (1 << 20)  /* Bit 20: Indicates double error detection occured on reload */
#define OCOTP_OUT_STATUS_CLR_CALIBRATED       (1 << 21)  /* Bit 21: Calibrated status */
#define OCOTP_OUT_STATUS_CLR_READ_DONE_INTR   (1 << 22)  /* Bit 22: Read fuse done */
#define OCOTP_OUT_STATUS_CLR_READ_ERROR_INTR  (1 << 23)  /* Bit 23: Fuse read error */
#define OCOTP_OUT_STATUS_CLR_DED0             (1 << 24)  /* Bit 24: Double error detect */
#define OCOTP_OUT_STATUS_CLR_DED1             (1 << 25)  /* Bit 25: Double error detect */
#define OCOTP_OUT_STATUS_CLR_DED2             (1 << 26)  /* Bit 26: Double error detect */
#define OCOTP_OUT_STATUS_CLR_DED3             (1 << 27)  /* Bit 27: Double error detect */

/* 8K OTP Memory STATUS Register (OUT_STATUS_TOG) */
#define OCOTP_OUT_STATUS_TOG_SEC              (1 << 9)   /* Bit 9: Single Error Correct */
#define OCOTP_OUT_STATUS_TOG_DED              (1 << 10)  /* Bit 10: Double error detect */
#define OCOTP_OUT_STATUS_TOG_LOCKED           (1 << 11)  /* Bit 11: Word Locked */
#define OCOTP_OUT_STATUS_TOG_PROGFAIL         (1 << 12)  /* Bit 12: Programming failed */
#define OCOTP_OUT_STATUS_TOG_ACK              (1 << 13)  /* Bit 13: Acknowledge */
#define OCOTP_OUT_STATUS_TOG_PWOK             (1 << 14)  /* Bit 14: Power OK */
#define OCOTP_OUT_STATUS_TOG_FLAGSTATE_SHIFT  (15)       /* Bits 15-19: Flag state */
#define OCOTP_OUT_STATUS_TOG_FLAGSTATE_MASK   (0xF << OCOTP_OUT_STATUS_TOG_FLAGSTATE_SHIFT)
#define OCOTP_OUT_STATUS_TOG_FLAGSTATE(n)     (((n) << OCOTP_OUT_STATUS_TOG_FLAGSTATE_SHIFT) & OCOTP_OUT_STATUS_TOG_FLAGSTATE_MASK)
#define OCOTP_OUT_STATUS_TOG_SEC_RELOAD       (1 << 19)  /* Bit 19: Indicates single error correction occured on reload */
#define OCOTP_OUT_STATUS_TOG_DED_RELOAD       (1 << 20)  /* Bit 20: Indicates double error detection occured on reload */
#define OCOTP_OUT_STATUS_TOG_CALIBRATED       (1 << 21)  /* Bit 21: Calibrated status */
#define OCOTP_OUT_STATUS_TOG_READ_DONE_INTR   (1 << 22)  /* Bit 22: Read fuse done */
#define OCOTP_OUT_STATUS_TOG_READ_ERROR_INTR  (1 << 23)  /* Bit 23: Fuse read error */
#define OCOTP_OUT_STATUS_TOG_DED0             (1 << 24)  /* Bit 24: Double error detect */
#define OCOTP_OUT_STATUS_TOG_DED1             (1 << 25)  /* Bit 25: Double error detect */
#define OCOTP_OUT_STATUS_TOG_DED2             (1 << 26)  /* Bit 26: Double error detect */
#define OCOTP_OUT_STATUS_TOG_DED3             (1 << 27)  /* Bit 27: Double error detect */

/* OTP Controller Version Register (VERSION) */
#define OCOTP_VERSION_STEP_SHIFT   (0)   /* Bits 0-16: RTL Version Stepping */
#define OCOTP_VERSION_STEP_MASK    (0xFFFF << OCOTP_VERSION_STEP_SHIFT)
#define OCOTP_VERSION_STEP(n)      (((n) << OCOTP_VERSION_STEP_SHIFT) & OCOTP_VERSION_STEP_MASK)
#define OCOTP_VERSION_MINOR_SHIFT  (16)  /* Bits 16-24: Minor RTL Version */
#define OCOTP_VERSION_MINOR_MASK   (0xFF << OCOTP_VERSION_MINOR_SHIFT)
#define OCOTP_VERSION_MINOR(n)     (((n) << OCOTP_VERSION_MINOR_SHIFT) & OCOTP_VERSION_MINOR_MASK)
#define OCOTP_VERSION_MAJOR_SHIFT  (24)  /* Bits 24-32: Major RTL Version */
#define OCOTP_VERSION_MAJOR_MASK   (0xFF << OCOTP_VERSION_MAJOR_SHIFT)
#define OCOTP_VERSION_MAJOR(n)     (((n) << OCOTP_VERSION_MAJOR_SHIFT) & OCOTP_VERSION_MAJOR_MASK)

/* OTP Controller Read Data 0 Register (READ_FUSE_DATA0) */
#define OCOTP_READ_FUSE_DATA0_DATA_SHIFT  (0)  /* Bits 0-32: Data */
#define OCOTP_READ_FUSE_DATA0_DATA_MASK   (0xFFFFFFFF << OCOTP_READ_FUSE_DATA0_DATA_SHIFT)
#define OCOTP_READ_FUSE_DATA0_DATA(n)     (((n) << OCOTP_READ_FUSE_DATA0_DATA_SHIFT) & OCOTP_READ_FUSE_DATA0_DATA_MASK)

/* OTP Controller Read Data 1 Register (READ_FUSE_DATA1) */
#define OCOTP_READ_FUSE_DATA1_DATA_SHIFT  (0)  /* Bits 0-32: Data */
#define OCOTP_READ_FUSE_DATA1_DATA_MASK   (0xFFFFFFFF << OCOTP_READ_FUSE_DATA1_DATA_SHIFT)
#define OCOTP_READ_FUSE_DATA1_DATA(n)     (((n) << OCOTP_READ_FUSE_DATA1_DATA_SHIFT) & OCOTP_READ_FUSE_DATA1_DATA_MASK)

/* OTP Controller Read Data 2 Register (READ_FUSE_DATA2) */
#define OCOTP_READ_FUSE_DATA2_DATA_SHIFT  (0)  /* Bits 0-32: Data */
#define OCOTP_READ_FUSE_DATA2_DATA_MASK   (0xFFFFFFFF << OCOTP_READ_FUSE_DATA2_DATA_SHIFT)
#define OCOTP_READ_FUSE_DATA2_DATA(n)     (((n) << OCOTP_READ_FUSE_DATA2_DATA_SHIFT) & OCOTP_READ_FUSE_DATA2_DATA_MASK)

/* OTP Controller Read Data 3 Register (READ_FUSE_DATA3) */
#define OCOTP_READ_FUSE_DATA3_DATA_SHIFT  (0)  /* Bits 0-32: Data */
#define OCOTP_READ_FUSE_DATA3_DATA_MASK   (0xFFFFFFFF << OCOTP_READ_FUSE_DATA3_DATA_SHIFT)
#define OCOTP_READ_FUSE_DATA3_DATA(n)     (((n) << OCOTP_READ_FUSE_DATA3_DATA_SHIFT) & OCOTP_READ_FUSE_DATA3_DATA_MASK)

/* SW_LOCK Register (SW_LOCK) */
#define OCOTP_SW_LOCK_SW_LOCK_SHIFT  (0)  /* Bits 0-32: This register contains lock information, which has the same function as the RLOCK fuse words (supplementary fuse words 8 (0x880) and 9 (0x890)) in fuse memory */
#define OCOTP_SW_LOCK_SW_LOCK_MASK   (0xFFFFFFFF << OCOTP_SW_LOCK_SW_LOCK_SHIFT)
#define OCOTP_SW_LOCK_SW_LOCK(n)     (((n) << OCOTP_SW_LOCK_SW_LOCK_SHIFT) & OCOTP_SW_LOCK_SW_LOCK_MASK)

/* BIT_LOCK Register (BIT_LOCK) */
#define OCOTP_BIT_LOCK_BIT_LOCK_SHIFT  (0)  /* Bits 0-32: Each bit controls the corresponding bit in supplementary fuse word 13 and its shadow register */
#define OCOTP_BIT_LOCK_BIT_LOCK_MASK   (0xFFFFFFFF << OCOTP_BIT_LOCK_BIT_LOCK_SHIFT)
#define OCOTP_BIT_LOCK_BIT_LOCK(n)     (((n) << OCOTP_BIT_LOCK_BIT_LOCK_SHIFT) & OCOTP_BIT_LOCK_BIT_LOCK_MASK)

/* OTP Controller Program Locked Status 0 Register (LOCKED0) */
#define OCOTP_LOCKED0_LOCKED_SHIFT  (0)  /* Bits 0-16: Stores program locked status for fuse words 0-15. */
#define OCOTP_LOCKED0_LOCKED_MASK   (0xFFFF << OCOTP_LOCKED0_LOCKED_SHIFT)
#define OCOTP_LOCKED0_LOCKED(n)     (((n) << OCOTP_LOCKED0_LOCKED_SHIFT) & OCOTP_LOCKED0_LOCKED_MASK)

/* OTP Controller Program Locked Status 1 Register (LOCKED1) */
#define OCOTP_LOCKED1_LOCKED_SHIFT  (0)  /* Bits 0-32: Stores program locked status for fuse words 16-47 */
#define OCOTP_LOCKED1_LOCKED_MASK   (0xFFFFFFFF << OCOTP_LOCKED1_LOCKED_SHIFT)
#define OCOTP_LOCKED1_LOCKED(n)     (((n) << OCOTP_LOCKED1_LOCKED_SHIFT) & OCOTP_LOCKED1_LOCKED_MASK)

/* OTP Controller Program Locked Status 2 Register (LOCKED2) */
#define OCOTP_LOCKED2_LOCKED_SHIFT  (0)  /* Bits 0-32: Stores program locked status for fuse words 48-79 */
#define OCOTP_LOCKED2_LOCKED_MASK   (0xFFFFFFFF << OCOTP_LOCKED2_LOCKED_SHIFT)
#define OCOTP_LOCKED2_LOCKED(n)     (((n) << OCOTP_LOCKED2_LOCKED_SHIFT) & OCOTP_LOCKED2_LOCKED_MASK)

/* OTP Controller Program Locked Status 3 Register (LOCKED3) */
#define OCOTP_LOCKED3_LOCKED_SHIFT  (0)  /* Bits 0-32: Stores program locked status for fuse words 80-111 */
#define OCOTP_LOCKED3_LOCKED_MASK   (0xFFFFFFFF << OCOTP_LOCKED3_LOCKED_SHIFT)
#define OCOTP_LOCKED3_LOCKED(n)     (((n) << OCOTP_LOCKED3_LOCKED_SHIFT) & OCOTP_LOCKED3_LOCKED_MASK)

/* OTP Controller Program Locked Status 4 Register (LOCKED4) */
#define OCOTP_LOCKED4_LOCKED_SHIFT  (0)  /* Bits 0-32: Stores program locked status for fuse words 112-143 */
#define OCOTP_LOCKED4_LOCKED_MASK   (0xFFFFFFFF << OCOTP_LOCKED4_LOCKED_SHIFT)
#define OCOTP_LOCKED4_LOCKED(n)     (((n) << OCOTP_LOCKED4_LOCKED_SHIFT) & OCOTP_LOCKED4_LOCKED_MASK)

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_RT117X_IMXRT117X_OCOTP_H */
