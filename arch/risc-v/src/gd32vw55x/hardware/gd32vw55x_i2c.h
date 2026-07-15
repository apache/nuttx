/****************************************************************************
 * arch/risc-v/src/gd32vw55x/hardware/gd32vw55x_i2c.h
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

#ifndef __ARCH_RISCV_SRC_GD32VW55X_HARDWARE_GD32VW55X_I2C_H
#define __ARCH_RISCV_SRC_GD32VW55X_HARDWARE_GD32VW55X_I2C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "gd32vw55x_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The GD32VW55x I2C controller is NOT the same IP as the GD32F4 I2C.  It is
 * the newer GigaDevice/ARM-style I2C block (as found on the GD32E5/L23 and
 * equivalent to the STM32 "I2C v2"): the bit timing is programmed through a
 * single TIMING register, the transfer length is loaded in CTL1.BYTENUM and
 * START/STOP/NACK generation is handled by the hardware state machine.
 *
 * There are two instances: I2C0 and I2C1, both on APB1.
 */

/* Register offsets *********************************************************/

#define GD32VW55X_I2C_CTL0_OFFSET     0x0000  /* Control register 0 */
#define GD32VW55X_I2C_CTL1_OFFSET     0x0004  /* Control register 1 */
#define GD32VW55X_I2C_SADDR0_OFFSET   0x0008  /* Slave address register 0 */
#define GD32VW55X_I2C_SADDR1_OFFSET   0x000c  /* Slave address register 1 */
#define GD32VW55X_I2C_TIMING_OFFSET   0x0010  /* Timing register */
#define GD32VW55X_I2C_TIMEOUT_OFFSET  0x0014  /* Timeout register */
#define GD32VW55X_I2C_STAT_OFFSET     0x0018  /* Status register */
#define GD32VW55X_I2C_STATC_OFFSET    0x001c  /* Status clear register */
#define GD32VW55X_I2C_PEC_OFFSET      0x0020  /* PEC register */
#define GD32VW55X_I2C_RDATA_OFFSET    0x0024  /* Receive data register */
#define GD32VW55X_I2C_TDATA_OFFSET    0x0028  /* Transmit data register */
#define GD32VW55X_I2C_CTL2_OFFSET     0x0090  /* Control register 2 */

/* Register addresses *******************************************************/

#define GD32VW55X_I2C_CTL0(b)         ((b) + GD32VW55X_I2C_CTL0_OFFSET)
#define GD32VW55X_I2C_CTL1(b)         ((b) + GD32VW55X_I2C_CTL1_OFFSET)
#define GD32VW55X_I2C_SADDR0(b)       ((b) + GD32VW55X_I2C_SADDR0_OFFSET)
#define GD32VW55X_I2C_SADDR1(b)       ((b) + GD32VW55X_I2C_SADDR1_OFFSET)
#define GD32VW55X_I2C_TIMING(b)       ((b) + GD32VW55X_I2C_TIMING_OFFSET)
#define GD32VW55X_I2C_TIMEOUT(b)      ((b) + GD32VW55X_I2C_TIMEOUT_OFFSET)
#define GD32VW55X_I2C_STAT(b)         ((b) + GD32VW55X_I2C_STAT_OFFSET)
#define GD32VW55X_I2C_STATC(b)        ((b) + GD32VW55X_I2C_STATC_OFFSET)
#define GD32VW55X_I2C_PEC(b)          ((b) + GD32VW55X_I2C_PEC_OFFSET)
#define GD32VW55X_I2C_RDATA(b)        ((b) + GD32VW55X_I2C_RDATA_OFFSET)
#define GD32VW55X_I2C_TDATA(b)        ((b) + GD32VW55X_I2C_TDATA_OFFSET)
#define GD32VW55X_I2C_CTL2(b)         ((b) + GD32VW55X_I2C_CTL2_OFFSET)

/* Register bit definitions *************************************************/

/* Control register 0 */

#define I2C_CTL0_I2CEN                (1 << 0)   /* Peripheral enable */
#define I2C_CTL0_TIE                  (1 << 1)   /* Transmit interrupt en */
#define I2C_CTL0_RBNEIE               (1 << 2)   /* Receive interrupt en */
#define I2C_CTL0_ADDMIE               (1 << 3)   /* Address match int en */
#define I2C_CTL0_NACKIE               (1 << 4)   /* NACK received int en */
#define I2C_CTL0_STPDETIE             (1 << 5)   /* Stop detection int en */
#define I2C_CTL0_TCIE                 (1 << 6)   /* Transfer complete int */
#define I2C_CTL0_ERRIE                (1 << 7)   /* Error interrupt enable */
#define I2C_CTL0_DNF_SHIFT            (8)        /* Bits 8-11: Digital NF */
#define I2C_CTL0_DNF_MASK             (15 << I2C_CTL0_DNF_SHIFT)
#  define I2C_CTL0_DNF(n)             ((n) << I2C_CTL0_DNF_SHIFT)
#define I2C_CTL0_ANOFF                (1 << 12)  /* Analog filter disable */
#define I2C_CTL0_DENT                 (1 << 14)  /* DMA enable transmit */
#define I2C_CTL0_DENR                 (1 << 15)  /* DMA enable receive */
#define I2C_CTL0_SBCTL                (1 << 16)  /* Slave byte control */
#define I2C_CTL0_SS                   (1 << 17)  /* SCL stretching disable */
#define I2C_CTL0_WUEN                 (1 << 18)  /* Wakeup from deep-sleep */
#define I2C_CTL0_GCEN                 (1 << 19)  /* General call enable */
#define I2C_CTL0_SMBHAEN              (1 << 20)  /* SMBus host address en */
#define I2C_CTL0_SMBDAEN              (1 << 21)  /* SMBus device address en */
#define I2C_CTL0_SMBALTEN             (1 << 22)  /* SMBus alert enable */
#define I2C_CTL0_PECEN                (1 << 23)  /* PEC calculation enable */

/* Control register 1 */

#define I2C_CTL1_SADDRESS_SHIFT       (0)        /* Bits 0-9: Slave address */
#define I2C_CTL1_SADDRESS_MASK        (0x3ff << I2C_CTL1_SADDRESS_SHIFT)
#  define I2C_CTL1_SADDRESS(n)        ((n) << I2C_CTL1_SADDRESS_SHIFT)
#define I2C_CTL1_TRDIR                (1 << 10)  /* Transfer direction */
#  define I2C_CTL1_TRDIR_WRITE        (0)        /* Master transmitter */
#  define I2C_CTL1_TRDIR_READ         I2C_CTL1_TRDIR
#define I2C_CTL1_ADD10EN              (1 << 11)  /* 10-bit addressing mode */
#define I2C_CTL1_HEAD10R              (1 << 12)  /* 10-bit read header only */
#define I2C_CTL1_START                (1 << 13)  /* Generate START */
#define I2C_CTL1_STOP                 (1 << 14)  /* Generate STOP */
#define I2C_CTL1_NACKEN               (1 << 15)  /* Generate NACK (slave) */
#define I2C_CTL1_BYTENUM_SHIFT        (16)       /* Bits 16-23: Byte count */
#define I2C_CTL1_BYTENUM_MASK         (0xff << I2C_CTL1_BYTENUM_SHIFT)
#  define I2C_CTL1_BYTENUM(n)         ((n) << I2C_CTL1_BYTENUM_SHIFT)
#define I2C_CTL1_RELOAD               (1 << 24)  /* Reload mode enable */
#define I2C_CTL1_AUTOEND              (1 << 25)  /* Automatic end mode */
#define I2C_CTL1_PECTRANS             (1 << 26)  /* PEC transfer */

/* Slave address register 0 */

#define I2C_SADDR0_ADDRESS0           (1 << 0)   /* Bit 0 of 10-bit address */
#define I2C_SADDR0_ADDRESS_SHIFT      (1)        /* Bits 1-7: 7-bit address */
#define I2C_SADDR0_ADDRESS_MASK       (0x7f << I2C_SADDR0_ADDRESS_SHIFT)
#define I2C_SADDR0_ADDRESS_H_SHIFT    (8)        /* Bits 8-9: 10-bit high */
#define I2C_SADDR0_ADDRESS_H_MASK     (3 << I2C_SADDR0_ADDRESS_H_SHIFT)
#define I2C_SADDR0_ADDFORMAT          (1 << 10)  /* 10-bit address format */
#define I2C_SADDR0_ADDRESSEN          (1 << 15)  /* Address enable */

/* Slave address register 1 */

#define I2C_SADDR1_ADDRESS2_SHIFT     (1)        /* Bits 1-7: Address 2 */
#define I2C_SADDR1_ADDRESS2_MASK      (0x7f << I2C_SADDR1_ADDRESS2_SHIFT)
#define I2C_SADDR1_ADDMSK2_SHIFT      (8)        /* Bits 8-10: Address mask */
#define I2C_SADDR1_ADDMSK2_MASK       (7 << I2C_SADDR1_ADDMSK2_SHIFT)
#define I2C_SADDR1_ADDRESS2EN         (1 << 15)  /* Address 2 enable */

/* Timing register */

#define I2C_TIMING_SCLL_SHIFT         (0)        /* Bits 0-7: SCL low */
#define I2C_TIMING_SCLL_MASK          (0xff << I2C_TIMING_SCLL_SHIFT)
#  define I2C_TIMING_SCLL(n)          ((n) << I2C_TIMING_SCLL_SHIFT)
#define I2C_TIMING_SCLH_SHIFT         (8)        /* Bits 8-15: SCL high */
#define I2C_TIMING_SCLH_MASK          (0xff << I2C_TIMING_SCLH_SHIFT)
#  define I2C_TIMING_SCLH(n)          ((n) << I2C_TIMING_SCLH_SHIFT)
#define I2C_TIMING_SDADELY_SHIFT      (16)       /* Bits 16-19: Data hold */
#define I2C_TIMING_SDADELY_MASK       (15 << I2C_TIMING_SDADELY_SHIFT)
#  define I2C_TIMING_SDADELY(n)       ((n) << I2C_TIMING_SDADELY_SHIFT)
#define I2C_TIMING_SCLDELY_SHIFT      (20)       /* Bits 20-23: Data setup */
#define I2C_TIMING_SCLDELY_MASK       (15 << I2C_TIMING_SCLDELY_SHIFT)
#  define I2C_TIMING_SCLDELY(n)       ((n) << I2C_TIMING_SCLDELY_SHIFT)
#define I2C_TIMING_PSC_SHIFT          (28)       /* Bits 28-31: Prescaler */
#define I2C_TIMING_PSC_MASK           (15 << I2C_TIMING_PSC_SHIFT)
#  define I2C_TIMING_PSC(n)           ((n) << I2C_TIMING_PSC_SHIFT)

/* Timeout register */

#define I2C_TIMEOUT_BUSTOA_SHIFT      (0)        /* Bits 0-11: Bus timeout A */
#define I2C_TIMEOUT_BUSTOA_MASK       (0xfff << I2C_TIMEOUT_BUSTOA_SHIFT)
#define I2C_TIMEOUT_TOIDLE            (1 << 12)  /* Idle clock timeout mode */
#define I2C_TIMEOUT_TOEN              (1 << 15)  /* Clock timeout enable */
#define I2C_TIMEOUT_BUSTOB_SHIFT      (16)       /* Bits 16-27: Timeout B */
#define I2C_TIMEOUT_BUSTOB_MASK       (0xfff << I2C_TIMEOUT_BUSTOB_SHIFT)
#define I2C_TIMEOUT_EXTOEN            (1 << 31)  /* Extended timeout enable */

/* Status register */

#define I2C_STAT_TBE                  (1 << 0)   /* TDATA empty */
#define I2C_STAT_TI                   (1 << 1)   /* Transmit interrupt */
#define I2C_STAT_RBNE                 (1 << 2)   /* RDATA not empty */
#define I2C_STAT_ADDSEND              (1 << 3)   /* Address matched (slave) */
#define I2C_STAT_NACK                 (1 << 4)   /* NACK received */
#define I2C_STAT_STPDET               (1 << 5)   /* STOP detected */
#define I2C_STAT_TC                   (1 << 6)   /* Transfer complete */
#define I2C_STAT_TCR                  (1 << 7)   /* Transfer compl. reload */
#define I2C_STAT_BERR                 (1 << 8)   /* Bus error */
#define I2C_STAT_LOSTARB              (1 << 9)   /* Arbitration lost */
#define I2C_STAT_OUERR                (1 << 10)  /* Overrun/underrun error */
#define I2C_STAT_PECERR               (1 << 11)  /* PEC error */
#define I2C_STAT_TIMEOUT              (1 << 12)  /* Timeout */
#define I2C_STAT_SMBALT               (1 << 13)  /* SMBus alert */
#define I2C_STAT_I2CBSY               (1 << 15)  /* Bus busy */
#define I2C_STAT_TR                   (1 << 16)  /* Transmitter (slave) */
#define I2C_STAT_READDR_SHIFT         (17)       /* Bits 17-23: Matched addr */
#define I2C_STAT_READDR_MASK          (0x7f << I2C_STAT_READDR_SHIFT)

/* Status clear register */

#define I2C_STATC_ADDSENDC            (1 << 3)   /* Clear ADDSEND */
#define I2C_STATC_NACKC               (1 << 4)   /* Clear NACK */
#define I2C_STATC_STPDETC             (1 << 5)   /* Clear STPDET */
#define I2C_STATC_BERRC               (1 << 8)   /* Clear BERR */
#define I2C_STATC_LOSTARBC            (1 << 9)   /* Clear LOSTARB */
#define I2C_STATC_OUERRC              (1 << 10)  /* Clear OUERR */
#define I2C_STATC_PECERRC             (1 << 11)  /* Clear PECERR */
#define I2C_STATC_TIMEOUTC            (1 << 12)  /* Clear TIMEOUT */
#define I2C_STATC_SMBALTC             (1 << 13)  /* Clear SMBALT */

/* All of the error flags (and the matching clear bits) */

#define I2C_STAT_ERRORS               (I2C_STAT_BERR | I2C_STAT_LOSTARB | \
                                       I2C_STAT_OUERR | I2C_STAT_PECERR | \
                                       I2C_STAT_TIMEOUT | I2C_STAT_SMBALT)

#define I2C_STATC_ERRORS              (I2C_STATC_BERRC | I2C_STATC_LOSTARBC | \
                                       I2C_STATC_OUERRC | I2C_STATC_PECERRC | \
                                       I2C_STATC_TIMEOUTC | I2C_STATC_SMBALTC)

/* Data registers */

#define I2C_RDATA_MASK                (0xff)     /* Receive data value */
#define I2C_TDATA_MASK                (0xff)     /* Transmit data value */

/* Control register 2 */

#define I2C_CTL2_ADDM_SHIFT           (9)        /* Bits 9-15: Address mask */
#define I2C_CTL2_ADDM_MASK            (0x7f << I2C_CTL2_ADDM_SHIFT)

/* The maximum number of bytes that can be transferred with a single
 * BYTENUM setting.  Longer messages must use the RELOAD mechanism.
 */

#define I2C_BYTENUM_MAX               (255)

#endif /* __ARCH_RISCV_SRC_GD32VW55X_HARDWARE_GD32VW55X_I2C_H */
