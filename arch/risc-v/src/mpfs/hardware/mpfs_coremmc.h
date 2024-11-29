/****************************************************************************
 * arch/risc-v/src/mpfs/hardware/mpfs_coremmc.h
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

#ifndef __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_COREMMC_H
#define __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_COREMMC_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MPFS_COREMMC_SR_OFFSET        0x00
#define MPFS_COREMMC_VR_OFFSET        0x01
#define MPFS_COREMMC_MJVR_OFFSET      0x02
#define MPFS_COREMMC_MIVR_OFFSET      0x03
#define MPFS_COREMMC_CMDX_OFFSET      0x04
#define MPFS_COREMMC_ARG1_OFFSET      0x08
#define MPFS_COREMMC_ARG2_OFFSET      0x09
#define MPFS_COREMMC_ARG3_OFFSET      0x0a
#define MPFS_COREMMC_ARG4_OFFSET      0x0b
#define MPFS_COREMMC_RR0_OFFSET       0x10
#define MPFS_COREMMC_RR1_OFFSET       0x14
#define MPFS_COREMMC_RR2_OFFSET       0x15
#define MPFS_COREMMC_RR3_OFFSET       0x16
#define MPFS_COREMMC_RR4_OFFSET       0x17
#define MPFS_COREMMC_RR5_OFFSET       0x18
#define MPFS_COREMMC_RR6_OFFSET       0x19
#define MPFS_COREMMC_RR7_OFFSET       0x1a
#define MPFS_COREMMC_RR8_OFFSET       0x1b
#define MPFS_COREMMC_RR9_OFFSET       0x1c
#define MPFS_COREMMC_RR10_OFFSET      0x1d
#define MPFS_COREMMC_RR11_OFFSET      0x1e
#define MPFS_COREMMC_RR12_OFFSET      0x1f
#define MPFS_COREMMC_RR13_OFFSET      0x20
#define MPFS_COREMMC_RR14_OFFSET      0x21
#define MPFS_COREMMC_RR15_OFFSET      0x22
#define MPFS_COREMMC_WDR_OFFSET       0x24
#define MPFS_COREMMC_RDR_OFFSET       0x28
#define MPFS_COREMMC_IMR_OFFSET       0x2c
#define MPFS_COREMMC_SBIMR_OFFSET     0x2d
#define MPFS_COREMMC_MBIMR_OFFSET     0x2e
#define MPFS_COREMMC_ISR_OFFSET       0x30
#define MPFS_COREMMC_SBISR_OFFSET     0x31
#define MPFS_COREMMC_MBISR_OFFSET     0x32
#define MPFS_COREMMC_ICR_OFFSET       0x34
#define MPFS_COREMMC_SBICR_OFFSET     0x35
#define MPFS_COREMMC_MBICR_OFFSET     0x36
#define MPFS_COREMMC_CTRL_OFFSET      0x38
#define MPFS_COREMMC_SBCSR_OFFSET     0x39
#define MPFS_COREMMC_MBCSR_OFFSET     0x3a
#define MPFS_COREMMC_RSPTO_OFFSET     0x3c
#define MPFS_COREMMC_DATATO_OFFSET    0x40
#define MPFS_COREMMC_BLR_OFFSET       0x44
#define MPFS_COREMMC_DCTRL_OFFSET     0x48
#define MPFS_COREMMC_CLKR_OFFSET      0x4c
#define MPFS_COREMMC_BCR_OFFSET       0x50
#define MPFS_COREMMC_MAX_OFFSET       (MPFS_COREMMC_BCR_OFFSET + 0x4)

/* Status register */

#define MPFS_COREMMC_SR_RDRE          (1 << 7)  /* Response Data Ready */
#define MPFS_COREMMC_SR_SWFF          (1 << 6)  /* Write FIFO Full     */
#define MPFS_COREMMC_SR_SRFF          (1 << 5)  /* Read FIFO Full      */
#define MPFS_COREMMC_SR_SWFE          (1 << 4)  /* Write FIFO Empty    */
#define MPFS_COREMMC_SR_SRFE          (1 << 3)  /* Read FIFO Empty     */
#define MPFS_COREMMC_SR_EBOD          (1 << 2)  /* Buffer Overflow     */
#define MPFS_COREMMC_SR_EBUD          (1 << 1)  /* Buffer Underrun     */
#define MPFS_COREMMC_SR_ECRD          (1 << 0)  /* CRC Error Detected  */

/* Version register */

#define COREMMC_VR_FIFODEPTH          (0x3 << 4)
#define COREMMC_VR_FIFODEPTH_512      0x00
#define COREMMC_VR_FIFODEPTH_4K       0x01
#define COREMMC_VR_FIFODEPTH_16K      0x02
#define COREMMC_VR_FIFODEPTH_32K      0x03

/* Interrupt Mask Register */

#define COREMMC_IMR_UER               (1 << 7)
#define COREMMC_IMR_SBI               (1 << 6)
#define COREMMC_IMR_TBI               (1 << 5)
#define COREMMC_IMR_TXI               (1 << 4)
#define COREMMC_IMR_RRI               (1 << 3)
#define COREMMC_IMR_CSI               (1 << 2)
#define COREMMC_IMR_BOI               (1 << 1)
#define COREMMC_IMR_BUI               (1 << 0)

#define COREMMC_IMR_ERROR (COREMMC_IMR_UER | COREMMC_IMR_SBI | \
                           COREMMC_IMR_TBI | COREMMC_IMR_TXI | \
                           COREMMC_IMR_TXI | COREMMC_IMR_BOI | \
                           COREMMC_IMR_BUI)

/* Single Block Interrupt Mask Register */

#define COREMMC_SBIMR_WDATAINFIFOTO   (1 << 7)
#define COREMMC_SBIMR_WBUSYTO         (1 << 6)
#define COREMMC_SBIMR_WCRCSTAERR      (1 << 5)
#define COREMMC_SBIMR_RSTPERR         (1 << 4)
#define COREMMC_SBIMR_RSTTO           (1 << 3)
#define COREMMC_SBIMR_CRCERR          (1 << 2)
#define COREMMC_SBIMR_RDONE           (1 << 1)
#define COREMMC_SBIMR_WDONE           (1 << 0)

#define COREMMC_SBIMR_ERROR (COREMMC_SBIMR_WDATAINFIFOTO | \
                             COREMMC_SBIMR_WBUSYTO       | \
                             COREMMC_SBIMR_WCRCSTAERR    | \
                             COREMMC_SBIMR_RSTPERR       | \
                             COREMMC_SBIMR_RSTTO         | \
                             COREMMC_SBIMR_CRCERR)

/* Multiple Block Interrupt Mask Register */

#define COREMMC_MBIMR_WDATAINFIFOTO   (1 << 7)
#define COREMMC_MBIMR_WBUSYTO         (1 << 6)
#define COREMMC_MBIMR_WCRCSTAERR      (1 << 5)
#define COREMMC_MBIMR_RSTPERR         (1 << 4)
#define COREMMC_MBIMR_RSTTO           (1 << 3)
#define COREMMC_MBIMR_CRCERR          (1 << 2)
#define COREMMC_MBIMR_RDONE           (1 << 1)
#define COREMMC_MBIMR_WDONE           (1 << 0)

#define COREMMC_MBIMR_ERROR (COREMMC_MBIMR_WDATAINFIFOTO | \
                             COREMMC_MBIMR_WBUSYTO       | \
                             COREMMC_MBIMR_WCRCSTAERR    | \
                             COREMMC_MBIMR_RSTPERR       | \
                             COREMMC_MBIMR_RSTTO         | \
                             COREMMC_MBIMR_CRCERR)

/* Interrupt Status Register */

#define COREMMC_ISR_UER               (1 << 7) /* User error is detected. Write FIFO overrun or Read FIFO underrun error. */
#define COREMMC_ISR_SBI               (1 << 6) /* Response start bit error detected or time-out error while waiting for a response. */
#define COREMMC_ISR_TBI               (1 << 5) /* Stop bit error is detected on response to command. */
#define COREMMC_ISR_TXI               (1 << 4) /* Transmit bit error is detected on response to command. */
#define COREMMC_ISR_RRI               (1 << 3) /* Response to command received. */
#define COREMMC_ISR_CSI               (1 << 2) /* Command sent. */
#define COREMMC_ISR_BOI               (1 << 1) /* Buffer overflow occurred. Read FIFO was full. */
#define COREMMC_ISR_BUI               (1 << 0) /* Underrun occurred. Write FIFO was empty. */

#define COREMMC_ISR_ERROR (COREMMC_ISR_UER | COREMMC_ISR_SBI | \
                           COREMMC_ISR_TBI | COREMMC_ISR_TXI | \
                           COREMMC_ISR_BOI | COREMMC_ISR_BUI)

/* Single and Multiple Block Interrupt Status Registers */

/* Single / multiple block Write FIFO timeout.  Asserted when less than block
 * length amount of data in the Write FIFO after the period defined in the
 * DATOTO register at the start of a block within a single / multiple block
 * write transfer.  Prevents Write FIFO underruns during Multiple block write
 * transfers.
 */

#define COREMMC_XBISR_WDATAINFIFOTO   (1 << 7)

/* Single / multiple block write busy timeout.  Set when eMMC slave device
 * holds DAT0 low for longer than the period defined in DATATO register at
 * the start of a block within a single / multiple block write transfer.
 * Indicates that the slave device is not ready to receive data.
 */

#define COREMMC_XBISR_WBUSYTO         (1 << 6)

/* Single / multiple block write CRC response error.  Set when start bit of
 * CRC Status frame not received within period defined in DATATO register or
 * when no valid stop bit detected for CRC status frame for a block within a
 * single / multiple block write transfer.
 */

#define COREMMC_XBISR_WCRCSTAERR      (1 << 5)

/* Single / multiple Block Read Stop Error. Set when valid stop bit not
 * detected on all active DATI lines for a block within a single / multiple
 * block read transfer.
 */

#define COREMMC_XBISR_RSTPERR         (1 << 4)

/* Single / multiple Block Read start time-out.  Set when no incoming
 * start-bit found on DAT for period defined in DATATO register for a block
 * within a single / multiple block read transfer.
 */

#define COREMMC_XBISR_RSTTO           (1 << 3)

/* Single / multiple block read or write encountered CRC error. */

#define COREMMC_XBISR_CRCERR          (1 << 2)

/* Single / multiple block read done. */

#define COREMMC_XBISR_RDONE           (1 << 1)

/* Single / multiple block write done. */

#define COREMMC_XBISR_WDONE           (1 << 0)

#define COREMMC_XBISR_ERROR (COREMMC_XBISR_WDATAINFIFOTO | \
                             COREMMC_XBISR_WBUSYTO       | \
                             COREMMC_XBISR_WCRCSTAERR    | \
                             COREMMC_XBISR_RSTPERR       | \
                             COREMMC_XBISR_RSTTO         | \
                             COREMMC_XBISR_CRCERR)

/* Interrupt Clear Register */

#define COREMMC_ICR_CLRUER            (1 << 7)
#define COREMMC_ICR_CLRSBI            (1 << 6)
#define COREMMC_ICR_CLRTBI            (1 << 5)
#define COREMMC_ICR_CLRTXI            (1 << 4)
#define COREMMC_ICR_CLRRRI            (1 << 3)
#define COREMMC_ICR_CLRCSI            (1 << 2)
#define COREMMC_ICR_CLRBOI            (1 << 1)
#define COREMMC_ICR_CLRBUI            (1 << 0)

#define COREMMC_ICR_ERROR (COREMMC_ICR_CLRUER | \
                           COREMMC_ICR_CLRSBI | \
                           COREMMC_ICR_CLRTBI | \
                           COREMMC_ICR_CLRTXI | \
                           COREMMC_ICR_CLRBOI | \
                           COREMMC_ICR_CLRBUI)

/* Single Block Interrupt Clear Register */

#define COREMMC_SBICR_WDATAINFIFOTO   (1 << 7)
#define COREMMC_SBICR_WBUSYTO         (1 << 6)
#define COREMMC_SBICR_WCRCSTAERR      (1 << 5)
#define COREMMC_SBICR_RSTPERR         (1 << 4)
#define COREMMC_SBICR_RSTTO           (1 << 3)
#define COREMMC_SBICR_CRCERR          (1 << 2)
#define COREMMC_SBICR_RDONE           (1 << 1)
#define COREMMC_SBICR_WDONE           (1 << 0)

#define COREMMC_SBICR_ERROR (COREMMC_SBICR_WDATAINFIFOTO | \
                             COREMMC_SBICR_WBUSYTO       | \
                             COREMMC_SBICR_WCRCSTAERR    | \
                             COREMMC_SBICR_RSTPERR       | \
                             COREMMC_SBICR_RSTTO         | \
                             COREMMC_SBICR_CRCERR)

/* Multiple Block Interrupt Clear Register */

#define COREMMC_MBICR_WDATAINFIFOTO   (1 << 7)
#define COREMMC_MBICR_WBUSYTO         (1 << 6)
#define COREMMC_MBICR_WCRCSTAERR      (1 << 5)
#define COREMMC_MBICR_RSTPERR         (1 << 4)
#define COREMMC_MBICR_RSTTO           (1 << 3)
#define COREMMC_MBICR_CRCERR          (1 << 2)
#define COREMMC_MBICR_RDONE           (1 << 1)
#define COREMMC_MBICR_WDONE           (1 << 0)

#define COREMMC_MBICR_ERROR (COREMMC_MBICR_WDATAINFIFOTO | \
                             COREMMC_MBICR_WBUSYTO       | \
                             COREMMC_MBICR_WCRCSTAERR    | \
                             COREMMC_MBICR_RSTPERR       | \
                             COREMMC_MBICR_RSTTO         | \
                             COREMMC_MBICR_CRCERR)

/* Control Register */

#define COREMMC_CTRL_BUSY             (1 << 7) /* Slave device is indicating that it is busy by asserting DAT[0] low. */
#define COREMMC_CTRL_RESERVED         (1 << 6) /* Reserved. */
#define COREMMC_CTRL_FIFORESET        (1 << 5) /* FIFO reset. */
#define COREMMC_CTRL_CMDFORCELOW      (1 << 4) /* Force CMD line to 0 (low). Used for boot operation. */
#define COREMMC_CTRL_MIDLE            (1 << 3) /* MMC Idle. When set to 1, it indicates that the core is in Idle state. */
#define COREMMC_CTRL_CLKOE            (1 << 2) /* CLK Output Enable. */
#define COREMMC_CTRL_SLRST            (1 << 1) /* Slave Reset. */
#define COREMMC_CTRL_SWRST            (1 << 0) /* Software reset */

/* Single Block Control and Status Register */

#define COREMMC_SBCSR_RESERVED        (1 << 7)
#define COREMMC_SBCSR_WST             (0x7 << 4) /* Single Block Write Status - CRC Status bits. Good CRC Status = 010. */
#define COREMMC_SBCSR_CRCERR          (1 << 3)   /* Single Block CRC Error. */
#define COREMMC_SBCSR_DONE            (1 << 2)   /* Single Block Done. */
#define COREMMC_SBCSR_RSTRT           (1 << 1)   /* Single Block Read Start. */
#define COREMMC_SBCSR_WSTRT           (1 << 0)   /* Single Block Write Start. */

/* Multiple Block Control and Status Register */

#define COREMMC_MBCSR_RESERVED        (1 << 7)
#define COREMMC_MBCSR_WST             (0x7 << 4) /* Multiple Block Write Status - CRC Status bits. Good CRC Status = 010. */
#define COREMMC_MBCSR_CRCERR          (1 << 3)   /* Multiple Block CRC Error. */
#define COREMMC_MBCSR_DONE            (1 << 2)   /* Multiple Block Done. */
#define COREMMC_MBCSR_RSTRT           (1 << 1)   /* Multiple Block Read Start. */
#define COREMMC_MBCSR_WSTRT           (1 << 0)   /* Multiple Block Write Start. */

/* Data Control Register */

#define COREMMC_DCTRL_DSIZE           (0x3 << 0)
#define COREMMC_DCTRL_DSIZE_1BIT      (0x0)
#define COREMMC_DCTRL_DSIZE_4BIT      (0x1)
#define COREMMC_DCTRL_DSIZE_8BIT      (0x2)

#endif /* __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_COREMMC_H */
