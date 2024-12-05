/****************************************************************************
 * arch/arm/src/rp23xx/hardware/rp23xx_qmi.h
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

#ifndef __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_QMI_H
#define __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_QMI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/rp23xx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define RP23XX_QMI_DIRECT_CSR_OFFSET    0x00000000
#define RP23XX_QMI_DIRECT_TX_OFFSET     0x00000004
#define RP23XX_QMI_DIRECT_RX_OFFSET     0x00000008
#define RP23XX_QMI_M0_TIMING_OFFSET     0x0000000c
#define RP23XX_QMI_M0_RFMT_OFFSET       0x00000010
#define RP23XX_QMI_M0_RCMD_OFFSET       0x00000014
#define RP23XX_QMI_M0_WFMT_OFFSET       0x00000018
#define RP23XX_QMI_M0_WCMD_OFFSET       0x0000001c
#define RP23XX_QMI_M1_TIMING_OFFSET     0x00000020
#define RP23XX_QMI_M1_RFMT_OFFSET       0x00000024
#define RP23XX_QMI_M1_RCMD_OFFSET       0x00000028
#define RP23XX_QMI_M1_WFMT_OFFSET       0x0000002c
#define RP23XX_QMI_M1_WCMD_OFFSET       0x00000030
#define RP23XX_QMI_ATRANS_OFFSET(n)     (0x00000034 + (n) * 4)

/* Register definitions *****************************************************/

#define RP23XX_QMI_DIRECT_CSR   (RP23XX_QMI_BASE + RP23XX_QMI_DIRECT_CSR_OFFSET)
#define RP23XX_QMI_DIRECT_TX    (RP23XX_QMI_BASE + RP23XX_QMI_DIRECT_TX_OFFSET)
#define RP23XX_QMI_DIRECT_RX    (RP23XX_QMI_BASE + RP23XX_QMI_DIRECT_RX_OFFSET)
#define RP23XX_QMI_M0_TIMING    (RP23XX_QMI_BASE + RP23XX_QMI_M0_TIMING_OFFSET)
#define RP23XX_QMI_M0_RFMT      (RP23XX_QMI_BASE + RP23XX_QMI_M0_RFMT_OFFSET)
#define RP23XX_QMI_M0_RCMD      (RP23XX_QMI_BASE + RP23XX_QMI_M0_RCMD_OFFSET)
#define RP23XX_QMI_M0_WFMT      (RP23XX_QMI_BASE + RP23XX_QMI_M0_WFMT_OFFSET)
#define RP23XX_QMI_M0_WCMD      (RP23XX_QMI_BASE + RP23XX_QMI_M0_WCMD_OFFSET)
#define RP23XX_QMI_M1_TIMING    (RP23XX_QMI_BASE + RP23XX_QMI_M1_TIMING_OFFSET)
#define RP23XX_QMI_M1_RFMT      (RP23XX_QMI_BASE + RP23XX_QMI_M1_RFMT_OFFSET)
#define RP23XX_QMI_M1_RCMD      (RP23XX_QMI_BASE + RP23XX_QMI_M1_RCMD_OFFSET)
#define RP23XX_QMI_M1_WFMT      (RP23XX_QMI_BASE + RP23XX_QMI_M1_WFMT_OFFSET)
#define RP23XX_QMI_M1_WCMD      (RP23XX_QMI_BASE + RP23XX_QMI_M1_WCMD_OFFSET)
#define RP23XX_QMI_ATRANS(n)    (RP23XX_QMI_BASE + RP23XX_QMI_ATRANS_OFFSET(n))

/* Register bit definitions *************************************************/

#define RP23XX_QMI_DIRECT_CSR_RXDELAY_SHIFT     (30)      /* Delay the read data sample timing, in units of one half of a system clock cycle. (Not necessarily half of an SCK cycle.) */
#define RP23XX_QMI_DIRECT_CSR_RXDELAY_MASK      (0x3 << RP23XX_QMI_DIRECT_CSR_RXDELAY_SHIFT)
#define RP23XX_QMI_DIRECT_CSR_CLKDIV_SHIFT      (22)      /* Clock divisor for direct serial mode. Divisors of 1..255 are encoded directly, and the maximum divisor of 256 is encoded by a value of CLKDIV=0 */
#define RP23XX_QMI_DIRECT_CSR_CLKDIV_MASK       (0xff << RP23XX_QMI_DIRECT_CSR_CLKDIV_SHIFT)
#define RP23XX_QMI_DIRECT_CSR_RXLEVEL_SHIFT     (18)      /* Current level of DIRECT_RX FIFO */
#define RP23XX_QMI_DIRECT_CSR_RXLEVEL_MASK      (0x7 << RP23XX_QMI_DIRECT_CSR_RXLEVEL_SHIFT)
#define RP23XX_QMI_DIRECT_CSR_RXFULL            (1 << 17) /* When 1, the DIRECT_RX FIFO is currently full. The serial interface will be stalled until data is popped; the interface will not begin a new serial frame when the DIRECT_TX FIFO is empty or the DIRECT_RX FIFO is full */
#define RP23XX_QMI_DIRECT_CSR_RXEMPTY           (1 << 16) /* When 1, the DIRECT_RX FIFO is currently empty. If the processor attempts to read more data, the FIFO state is not affected, but the value returned to the processor is undefined */
#define RP23XX_QMI_DIRECT_CSR_TXLEVEL_SHIFT     (12)      /* Current level of DIRECT_TX FIFO */
#define RP23XX_QMI_DIRECT_CSR_TXLEVEL_MASK      (0x7 << RP23XX_QMI_DIRECT_CSR_TXLEVEL_SHIFT)
#define RP23XX_QMI_DIRECT_CSR_TXEMPTY           (1 << 11) /* When 1, the DIRECT_TX FIFO is currently empty. Unless the processor pushes more data, transmission will stop and BUSY will go low once the current 8-bit serial frame completes */
#define RP23XX_QMI_DIRECT_CSR_TXFULL            (1 << 10) /* When 1, the DIRECT_TX FIFO is currently full. If the processor tries to write more data, that data will be ignored */
#define RP23XX_QMI_DIRECT_CSR_AUTO_CS1N         (1 << 7)  /* When 1, automatically assert the CS1n chip select line whenever the BUSY flag is set */
#define RP23XX_QMI_DIRECT_CSR_AUTO_CS0N         (1 << 6)  /* When 1, automatically assert the CS0n chip select line whenever the BUSY flag is set */
#define RP23XX_QMI_DIRECT_CSR_ASSERT_CS1N       (1 << 3)  /* When 1, assert (i.e. drive low) the CS1n chip select line */
#define RP23XX_QMI_DIRECT_CSR_ASSERT_CS0N       (1 << 2)  /* When 1, assert (i.e. drive low) the CS0n chip select line */
#define RP23XX_QMI_DIRECT_CSR_BUSY              (1 << 1)  /* Direct mode busy flag. If 1, data is currently being shifted in/out (or would be if the interface were not stalled on the RX FIFO), and the chip select must not yet be deasserted */
#define RP23XX_QMI_DIRECT_CSR_EN                (1 << 0)  /* Enable direct mode */

#define RP23XX_QMI_DIRECT_TX_NOPUSH             (1 << 20) /* Inhibit the RX FIFO push that would correspond to this TX FIFO entry */
#define RP23XX_QMI_DIRECT_TX_OE                 (1 << 19) /* Output enable (active-high). For single width (SPI), this field is ignored, and SD0 is always set to output, with SD1 always set to input */
#define RP23XX_QMI_DIRECT_TX_DWIDTH             (1 << 18) /* Data width. If 0, hardware will transmit the 8 LSBs of the DIRECT_TX DATA field, and return an 8-bit value in the 8 LSBs of DIRECT_RX. If 1, the full 16-bit width is used. 8-bit and 16-bit transfers can be mixed freely */
#define RP23XX_QMI_DIRECT_TX_IWIDTH_SHIFT       (16)      /* Configure whether this FIFO record is transferred with single/dual/quad interface width (0/1/2). Different widths can be mixed freely */
#define RP23XX_QMI_DIRECT_TX_IWIDTH_MASK        (0x3 << RP23XX_QMI_DIRECT_TX_IWIDTH_SHIFT)
#define RP23XX_QMI_DIRECT_TX_DATA_MASK          (0xffff)  /* Data pushed here will be clocked out falling edges of SCK (or before the very first rising edge of SCK, if this is the first pulse). For each byte clocked out, the interface will simultaneously sample one byte, on rising edges of SCK, and push this to the DIRECT_RX FIFO. For 16-bit data, the least-significant byte is transmitted first. */
#define RP23XX_QMI_DIRECT_RX_MASK               (0xffff)  /* With each byte clocked out on the serial interface, one byte will simultaneously be clocked in, and will appear in this FIFO. The serial interface will stall when this FIFO is full, to avoid dropping data. When 16-bit data is pushed into the TX FIFO, the corresponding RX FIFO push will also contain 16 bits of data. The least-significant byte is the first one received. */

#define RP23XX_QMI_TIMING_COOLDOWN_SHIFT        (30)         /* Chip select cooldown period. When a memory transfer finishes, the chip select remains asserted for 64 x COOLDOWN system clock cycles, plus half an SCK clock period (rounded up for odd SCK divisors). After this cooldown expires, the chip select is always deasserted to save power */
#define RP23XX_QMI_TIMING_COOLDOWN_MASK         (0x3 << RP23XX_QMI_M0_TIMING_COOLDOWN_SHIFT)
#define RP23XX_QMI_TIMING_PAGEBREAK_SHIFT       (28)         /* When page break is enabled, chip select will automatically deassert when crossing certain power-of-2-aligned address boundaries. The next access will always begin a new read/write SPI burst, even if the address of the next access follows in sequence with the last access before the page boundary */
#define RP23XX_QMI_TIMING_PAGEBREAK_MASK        (0x3 << RP23XX_QMI_M0_TIMING_PAGEBREAK_SHIFT)
#define RP23XX_QMI_TIMING_SELECT_SETUP          (1 << 25)    /* Add up to one additional system clock cycle of setup between chip select assertion and the first rising edge of SCK */
#define RP23XX_QMI_TIMING_SELECT_HOLD_SHIFT     (23)         /* Add up to three additional system clock cycles of active hold between the last falling edge of SCK and the deassertion of this window’s chip select */
#define RP23XX_QMI_TIMING_SELECT_HOLD_MASK      (0x3 << RP23XX_QMI_M0_TIMING_SELECT_HOLD_SHIFT)
#define RP23XX_QMI_TIMING_MAX_SELECT_SHIFT      (17)         /* Enforce a maximum assertion duration for this window’s chip select, in units of 64 system clock cycles. If 0, the QMI is permitted to keep the chip select asserted indefinitely when servicing sequential memory accesses (see COOLDOWN) */
#define RP23XX_QMI_TIMING_MAX_SELECT_MASK       (0x3f << RP23XX_QMI_M0_TIMING_MAX_SELECT_SHIFT)
#define RP23XX_QMI_TIMING_MIN_DESELECT_SHIFT    (12)         /* After this window’s chip select is deasserted, it remains deasserted for half an SCK cycle (rounded up to an integer number of system clock cycles), plus MIN_DESELECT additional system clock cycles, before the QMI reasserts either chip select pin */
#define RP23XX_QMI_TIMING_MIN_DESELECT_MASK     (0x1f << RP23XX_QMI_M0_TIMING_MIN_DESELECT_SHIFT)
#define RP23XX_QMI_TIMING_RXDELAY_SHIFT         (8)          /* Delay the read data sample timing, in units of one half of a system clock cycle. (Not necessarily half of an SCK cycle.) An RXDELAY of 0 means the sample is captured at the SDI input registers simultaneously with the rising edge of SCK launched from the SCK output register */
#define RP23XX_QMI_TIMING_RXDELAY_MASK          (0x7 << RP23XX_QMI_M0_TIMING_RXDELAY_SHIFT)
#define RP23XX_QMI_TIMING_CLKDIV_MASK           (0x000000ff) /* Clock divisor. Odd and even divisors are supported. Defines the SCK clock period in units of 1 system clock cycle. Divisors 1..255 are encoded directly, and a divisor of 256 is encoded with a value of CLKDIV=0 */

#define RP23XX_QMI_FMT_DTR                      (1 << 28)    /* Enable double transfer rate (DTR) for read commands: address, suffix and read data phases are active on both edges of SCK. SDO data is launched centre-aligned on each SCK edge, and SDI data is captured on the SCK edge that follows its launch */
#define RP23XX_QMI_FMT_DUMMY_LEN_SHIFT          (16)         /* Length of dummy phase between command suffix and data phase, in units of 4 bits. (i.e. 1 cycle for quad width, 2 for dual, 4 for single) */
#define RP23XX_QMI_FMT_DUMMY_LEN_MASK           (0x7 << RP23XX_QMI_M0_RFMT_DUMMY_LEN_SHIFT)
#define RP23XX_QMI_FMT_SUFFIX_LEN_SHIFT         (14)         /* Length of post-address command suffix, in units of 4 bits. (i.e. 1 cycle for quad width, 2 for dual, 4 for single) */
#define RP23XX_QMI_FMT_SUFFIX_LEN_MASK          (0x3 << RP23XX_QMI_M0_RFMT_SUFFIX_LEN_SHIFT)
#define RP23XX_QMI_FMT_PREFIX_LEN               (1 << 12)    /* Length of command prefix, in units of 8 bits. (i.e. 2 cycles for quad width, 4 for dual, 8 for single) */
#define RP23XX_QMI_FMT_DATA_WIDTH_SHIFT         (8)          /* The width used for the data transfer */
#define RP23XX_QMI_FMT_DATA_WIDTH_MASK          (0x3 << RP23XX_QMI_M0_RFMT_DATA_WIDTH_SHIFT)
#define RP23XX_QMI_FMT_DUMMY_WIDTH_SHIFT        (6)          /* The width used for the dummy phase, if any */
#define RP23XX_QMI_FMT_DUMMY_WIDTH_MASK         (0x3 << RP23XX_QMI_M0_RFMT_DUMMY_WIDTH_SHIFT)
#define RP23XX_QMI_FMT_SUFFIX_WIDTH_SHIFT       (4)          /* The width used for the post-address command suffix, if any */
#define RP23XX_QMI_FMT_SUFFIX_WIDTH_MASK        (0x4 << RP23XX_QMI_M0_RFMT_SUFFIX_WIDTH_SHIFT)
#define RP23XX_QMI_FMT_ADDR_WIDTH_SHIFT         (2)          /* The transfer width used for the address. The address phase always transfers 24 bits in total */
#define RP23XX_QMI_FMT_ADDR_WIDTH_MASK          (0x3 << RP23XX_QMI_M0_RFMT_ADDR_WIDTH_SHIFT)
#define RP23XX_QMI_FMT_PREFIX_WIDTH_MASK        (0x00000003) /* The transfer width used for the command prefix, if any */

#define RP23XX_QMI_CMD_SUFFIX_SHIFT             (8)          /* The command suffix bits following the address, if Mx_RFMT_SUFFIX_LEN is nonzero */
#define RP23XX_QMI_CMD_SUFFIX_MASK              (0xff << RP23XX_QMI_CMD_SUFFIX_SHIFT)
#define RP23XX_QMI_CMD_PREFIX_MASK              (0x000000ff) /* The command prefix bits to prepend on each new transfer, if Mx_RFMT_PREFIX_LEN is nonzero */

#define RP23XX_QMI_ATRANS_SIZE_SHIFT            (16)         /* Translation aperture size for this virtual address range, in units of 4 kiB (one flash sector). */
#define RP23XX_QMI_ATRANS_SIZE_MASK             (0x7ff << RP23XX_QMI_ATRANS_SIZE_SHIFT)
#define RP23XX_QMI_ATRANS_BASE_MASK             (0xfff)      /* Physical address base for this virtual address range, in units of 4 kiB (one flash sector) */

#endif /* __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_QMI_H */
