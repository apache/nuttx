/****************************************************************************
 * arch/arm/src/rm57/hardware/rm57_sci.h
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

/* Register layout taken from TI's HALCoGen HL_reg_sci.h for
 * RM57L843. RM57L843 has four SCI instances (SCI1/LIN1, SCI2, SCI3,
 * SCI4) vs. TMS570LS's one or two - see rm57l843_memorymap.h for the
 * per-instance base addresses.
 */

#ifndef __ARCH_ARM_SRC_RM57_HARDWARE_RM57_SCI_H
#define __ARCH_ARM_SRC_RM57_HARDWARE_RM57_SCI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/rm57l843_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define RM57_SCI_GCR0_OFFSET         0x0000 /* Global Control Register 0 */
#define RM57_SCI_GCR1_OFFSET         0x0004 /* Global Control Register 1 */

/* Global Control Register 2 (LIN mode only) */
#define RM57_SCI_GCR2_OFFSET 0x0008

/* Set Interrupt Enable Register */
#define RM57_SCI_SETINT_OFFSET 0x000c

/* Clear Interrupt Enable Register */
#define RM57_SCI_CLEARINT_OFFSET 0x0010
#define RM57_SCI_SETINTLVL_OFFSET    0x0014 /* Set Interrupt Level Register */

/* Clear Interrupt Level Register */
#define RM57_SCI_CLEARINTLVL_OFFSET 0x0018
#define RM57_SCI_FLR_OFFSET          0x001c /* Interrupt Flag Register */
#define RM57_SCI_INTVECT0_OFFSET     0x0020 /* Interrupt Vector Offset 0 */
#define RM57_SCI_INTVECT1_OFFSET     0x0024 /* Interrupt Vector Offset 1 */
#define RM57_SCI_FORMAT_OFFSET       0x0028 /* Format Control Register */
#define RM57_SCI_BRS_OFFSET          0x002c /* Baud Rate Selection Register */
#define RM57_SCI_ED_OFFSET           0x0030 /* Emulation Register */
#define RM57_SCI_RD_OFFSET           0x0034 /* Receive Data Buffer */
#define RM57_SCI_TD_OFFSET           0x0038 /* Transmit Data Buffer */
#define RM57_SCI_PIO0_OFFSET         0x003c /* Pin Function Register */
#define RM57_SCI_PIO1_OFFSET         0x0040 /* Pin Direction Register */
#define RM57_SCI_PIO2_OFFSET         0x0044 /* Pin Data In Register */
#define RM57_SCI_PIO3_OFFSET         0x0048 /* Pin Data Out Register */
#define RM57_SCI_PIO4_OFFSET         0x004c /* Pin Data Set Register */
#define RM57_SCI_PIO5_OFFSET         0x0050 /* Pin Data Clr Register */

/* Pin Open Drain Output Enable Register */
#define RM57_SCI_PIO6_OFFSET 0x0054

/* Pin Pullup/Pulldown Disable Register */
#define RM57_SCI_PIO7_OFFSET 0x0058

/* Pin Pullup/Pulldown Selection Register */
#define RM57_SCI_PIO8_OFFSET 0x005c

/* LIN-mode registers (valid when SCI_GCR1_LIN_MODE is set; SCI1/LIN1 and
 * SCI2/LIN2 only - SCI3/SCI4 are SCI-only and do not implement these).
 */

#define RM57_SCI_LINCOMPARE_OFFSET   0x0060 /* LIN Compare Register */
#define RM57_SCI_LINRD0_OFFSET       0x0064 /* LIN Receive Buffer 0 */
#define RM57_SCI_LINRD1_OFFSET       0x0068 /* LIN Receive Buffer 1 */
#define RM57_SCI_LINMASK_OFFSET      0x006c /* LIN Mask Register */
#define RM57_SCI_LINID_OFFSET        0x0070 /* LIN Identification Register */
#define RM57_SCI_LINTD0_OFFSET       0x0074 /* LIN Transmit Buffer 0 */
#define RM57_SCI_LINTD1_OFFSET       0x0078 /* LIN Transmit Buffer 1 */
#define RM57_SCI_MBRS_OFFSET         0x007c /* Maximum Baud Rate Selection */
#define RM57_SCI_IODFTCTRL_OFFSET    0x0090 /* I/O Error Enable Register */

/* Byte lane of the 8-bit RX/TX data within the 32-bit RD/TD registers, as
 * seen by the DMA controller when it performs an 8-bit access.
 *
 * The RD/TD data byte lives in bits 7:0 of the 32-bit register. This
 * image runs LE32 (no CONFIG_ENDIAN_BIG, no boot-time endian switch in
 * rm57_boot.c), and the same LE32 mode applies to every bus master,
 * including the DMA controller, so the data byte for an 8-bit DMA access
 * is at the register's +0 address.
 *
 * Do NOT use OpenOCD's "ti_be_32_quirks 1" setting as evidence for a +3
 * byte-lane offset here: that quirk is specific to how the JTAG/DAP debug
 * port presents byte lanes for sub-word memory accesses made *through the
 * debugger*, and says nothing about how the CPU/DMA controller address
 * bytes during normal program execution.
 */

#define RM57_SCI_RD_DMA_OFFSET       (RM57_SCI_RD_OFFSET + 0)
#define RM57_SCI_TD_DMA_OFFSET       (RM57_SCI_TD_OFFSET + 0)

/* Register Bit-Field Definitions *******************************************/

/* Global Control Register 0 */

/* Bit 0: Take SCI out of reset */
#define SCI_GCR0_RESET (1 << 0)

/* Global Control Register 1 */

/* Bit 0:  0=idle-line, 1=addr-bit mode */
#define SCI_GCR1_COMM_MODE (1 << 0)

/* Bit 1:  Asynchronous timing mode */
#define SCI_GCR1_TIMING_MODE (1 << 1)
#define SCI_GCR1_PARITY_ENA          (1 << 2)  /* Bit 2:  Parity enable */
#define SCI_GCR1_PARITY              (1 << 3)  /* Bit 3:  0=odd, 1=even */

/* Bit 4:  0=1 stop bit, 1=2 stop bits */
#define SCI_GCR1_STOP (1 << 4)

/* Bit 5:  Clock signal source (LIN mode) */
#define SCI_GCR1_CLOCK (1 << 5)
#define SCI_GCR1_LIN_MODE            (1 << 6)  /* Bit 6:  0=SCI, 1=LIN */

/* Bit 7:  Software reset, active low */
#define SCI_GCR1_SWRST (1 << 7)

/* Bit 8:  Sleep mode enable (multiprocessor communication). NOTE: this
 * bit was previously (incorrectly) defined here as SCI_GCR1_CTYPE - per
 * TRM SPNU562A section 28.6.2 (SCIGCR1) bit 8 is SLEEP, bit 9 is ADAPT,
 * and CTYPE/HGEN_CTRL are actually bits 11/12 (redefined below). Nothing
 * in-tree referenced the old (wrong) values.
 */
#define SCI_GCR1_SLEEP (1 << 8)

/* Bit 9:  Adaptive baud rate mode enable (LIN mode) */
#define SCI_GCR1_ADAPT (1 << 9)

/* Bit 10: Multi-buffer mode enable (LIN mode) */
#define SCI_GCR1_MBUF_MODE (1 << 10)

/* Bit 11: LIN checksum type, 0=classic, 1=enhanced */
#define SCI_GCR1_CTYPE (1 << 11)

/* Bit 12: LIN hardware header generation/comparison control */
#define SCI_GCR1_HGEN_CTRL (1 << 12)

/* Bit 13: LIN extended frame (stop bit extension) */
#define SCI_GCR1_STOPEXT (1 << 13)
#define SCI_GCR1_RXENA               (1 << 24) /* Bit 24: Receiver enable */

/* Bit 25: Transmitter enable */
#define SCI_GCR1_TXENA (1 << 25)
#define SCI_GCR1_LOOPBACK            (1 << 16) /* Bit 16: Loop back enable */

/* Bit 17: Continue on suspend (emulation) */
#define SCI_GCR1_CONT (1 << 17)

/* Global Control Register 2 (LIN mode only) */

#define SCI_GCR2_POWERDOWN            (1 << 0)  /* Bit 0:  Local low-power mode request */
#define SCI_GCR2_GENWU                (1 << 8)  /* Bit 8:  Generate wakeup signal (LIN master) */
#define SCI_GCR2_SC                   (1 << 16) /* Bit 16: Send checksum byte */
#define SCI_GCR2_CC                   (1 << 17) /* Bit 17: Compare checksum byte */

/* Interrupt Flag / Set / Clear Register bits (common to SETINT/CLEARINT/
 * FLR) - values confirmed from TI's HALCoGen HL_sci.h sciInterrupt_t
 * enum, not guessed
 */

#define SCI_INT_BREAK                (0x00000001) /* Break detect */
#define SCI_INT_WAKE                 (0x00000002) /* Wakeup */
#define SCI_INT_TX                   (0x00000100) /* Transmit buffer ready */
#define SCI_INT_RX                   (0x00000200) /* Receive buffer ready */
#define SCI_INT_TXDMA                (1 << 16)    /* Bit 16: Set/clear TX DMA request */
#define SCI_INT_RXDMA                (1 << 17)    /* Bit 17: Set/clear RX DMA request */
#define SCI_INT_RXDMAALL             (1 << 18)    /* Bit 18: RX DMA for address+data frames */
#define SCI_INT_ID                   (1 << 13)    /* Bit 13: LIN identifier interrupt */
#define SCI_INT_PE                   (0x01000000) /* Parity error */
#define SCI_INT_OE                   (0x02000000) /* Overrun error */
#define SCI_INT_FE                   (0x04000000) /* Framing error */
#define SCI_INT_ALLINTS              (0xff0023d3)

/* Baud Rate Selection Register: P (integer divider, bits 0-23), M (4-bit
 * fractional divider, bits 24-27), U (super-fractional divider, bits
 * 28-30)
 */

#define SCI_BRS_P_SHIFT              (0)
#define SCI_BRS_P_MASK               (0x00ffffff << SCI_BRS_P_SHIFT)
#  define SCI_BRS_P(n)               ((uint32_t)(n) << SCI_BRS_P_SHIFT)
#define SCI_BRS_M_SHIFT              (24)
#define SCI_BRS_M_MASK               (15 << SCI_BRS_M_SHIFT)
#  define SCI_BRS_M(n)                ((uint32_t)(n) << SCI_BRS_M_SHIFT)
#define SCI_BRS_U_SHIFT              (28)
#define SCI_BRS_U_MASK               (7 << SCI_BRS_U_SHIFT)
#  define SCI_BRS_U(n)                ((uint32_t)(n) << SCI_BRS_U_SHIFT)

/* Format Control Register: frame length control, bits 0-2 (nbits - 1) */

#define SCI_FORMAT_CHAR_SHIFT        (0)
#define SCI_FORMAT_CHAR_MASK         (7 << SCI_FORMAT_CHAR_SHIFT)
#  define SCI_FORMAT_CHAR(n)         ((uint32_t)(n) << SCI_FORMAT_CHAR_SHIFT)

/* Pin Function Register (PIO0): confirmed from HL_sci.h
 * SCI3_PIO0_CONFIGVALUE = (1<<2)|(1<<1)
 */

#define SCI_PIO_RX                   (1 << 1)  /* Bit 1: RX pin */
#define SCI_PIO_TX                   (1 << 2)  /* Bit 2: TX pin */

/* SCI Flags Register (FLR) - same bit positions as SETINT/CLEARINT above,
 * plus status-only bits below. NOT independently derivable from
 * HL_reg_sci.h (which only gives register offsets, not TRM-documented
 * bit tables); reused from tms570_sci.h since both TMS570 and RM57 share
 * the same underlying Hercules SCI IP block (confirmed by the GCR1 bit
 * positions matching exactly against the HALCoGen SCI3_GCR1_CONFIGVALUE
 * decode above).
 */

/* Bit 0:  Break detect flag */
#define SCI_FLR_BRKDT (1 << 0)
#define SCI_FLR_WAKEUP                (1 << 1)  /* Bit 1:  Wake-up flag */

/* Bit 2:  SCI receiver in idle state */
#define SCI_FLR_IDLE (1 << 2)
#define SCI_FLR_BUSY                  (1 << 3)  /* Bit 3:  Bus busy flag */
#define SCI_FLR_TIMEOUT               (1 << 4)  /* Bit 4:  Timeout flag */

/* Bit 8:  Transmit buffer ready flag */
#define SCI_FLR_TXRDY (1 << 8)

/* Bit 9:  Receive buffer ready flag */
#define SCI_FLR_RXRDY (1 << 9)

/* Bit 11: Transmitter empty flag */
#define SCI_FLR_TXEMPTY (1 << 11)

/* Bit 24: Parity error flag */
#define SCI_FLR_PE (1 << 24)

/* Bit 25: Overrun error flag */
#define SCI_FLR_OE (1 << 25)

/* Bit 26: Framing error flag */
#define SCI_FLR_FE (1 << 26)

/* LIN-mode-only flags (SCIFLR) - reused from tms570_sci.h, see note
 * above; only meaningful when SCI_GCR1_LIN_MODE is set.
 */

#define SCI_FLR_TOAWUS                (1 << 6)  /* Bit 6:  Timeout after wakeup signal */
#define SCI_FLR_TOA3WUS               (1 << 7)  /* Bit 7:  Timeout after 3 wakeup signals */
#define SCI_FLR_TXWAKE                (1 << 10) /* Bit 10: Transmit wakeup signal flag */
#define SCI_FLR_RXWAKE                (1 << 12) /* Bit 12: Received data is an address (sleep mode) */
#define SCI_FLR_IDTX                  (1 << 13) /* Bit 13: TX ID match flag */
#define SCI_FLR_IDRX                  (1 << 14) /* Bit 14: RX ID match flag */
#define SCI_FLR_NRE                   (1 << 27) /* Bit 27: No-response error flag */
#define SCI_FLR_ISFE                  (1 << 28) /* Bit 28: Inconsistent sync field error flag */
#define SCI_FLR_CE                    (1 << 29) /* Bit 29: Checksum error flag */
#define SCI_FLR_PBE                   (1 << 30) /* Bit 30: Physical bus error flag */
#define SCI_FLR_BE                   (1U << 31) /* Bit 31: Bit error flag */

/* SCI Interrupt Vector Offset Registers (INTVECT0/1): priority-encoded
 * pending-interrupt code, cleared on read (same source table as
 * tms570_sci.h - see note above)
 */

#define SCI_INTVECT_MASK              (0x1f)
#  define SCI_INTVECT_NONE            (0)
#  define SCI_INTVECT_WAKEUP          (1)
#  define SCI_INTVECT_ISFE            (2)
#  define SCI_INTVECT_PE              (3)
#  define SCI_INTVECT_ID              (4)
#  define SCI_INTVECT_PBE             (5)
#  define SCI_INTVECT_FE              (6)
#  define SCI_INTVECT_BRKDT           (7)
#  define SCI_INTVECT_CE              (8)
#  define SCI_INTVECT_OE              (9)
#  define SCI_INTVECT_BE              (10)
#  define SCI_INTVECT_RX              (11)
#  define SCI_INTVECT_TX              (12)
#  define SCI_INTVECT_NRE             (13)
#  define SCI_INTVECT_TOAWUS          (14)
#  define SCI_INTVECT_TOA3WUS         (15)
#  define SCI_INTVECT_TIMEOUT         (16)

/* Receive/Transmit Data Buffer */

#define SCI_RD_MASK                  (0xff)
#define SCI_TD_MASK                  (0xff)

/* LIN Compare Register (LINCOMPARE, offset 0x60): synch delimiter/break
 * extension, LIN mode only.
 */

#define SCI_LINCOMPARE_SBREAK_SHIFT  (0)
#define SCI_LINCOMPARE_SBREAK_MASK   (7 << SCI_LINCOMPARE_SBREAK_SHIFT)
#  define SCI_LINCOMPARE_SBREAK(n)   ((uint32_t)(n) << SCI_LINCOMPARE_SBREAK_SHIFT)
#define SCI_LINCOMPARE_SDEL_SHIFT    (16)
#define SCI_LINCOMPARE_SDEL_MASK     (3 << SCI_LINCOMPARE_SDEL_SHIFT)
#  define SCI_LINCOMPARE_SDEL(n)     ((uint32_t)(n) << SCI_LINCOMPARE_SDEL_SHIFT)

/* LIN Receive Buffer Registers (LINRD0 offset 0x64, LINRD1 offset 0x68):
 * each holds 4 bytes of an up-to-8-byte LIN response, byte 0 (earliest
 * received) in the MSB. RD<n> is response data byte <n+1>.
 */

#define SCI_LINRD_BYTE0_SHIFT        (24)
#define SCI_LINRD_BYTE1_SHIFT        (16)
#define SCI_LINRD_BYTE2_SHIFT        (8)
#define SCI_LINRD_BYTE3_SHIFT        (0)
#define SCI_LINRD_BYTE_MASK          (0xff)

/* LIN Mask Register (LINMASK, offset 0x6c) */

#define SCI_LINMASK_TXIDMASK_SHIFT   (0)
#define SCI_LINMASK_TXIDMASK_MASK    (0xff << SCI_LINMASK_TXIDMASK_SHIFT)
#  define SCI_LINMASK_TXIDMASK(n)    ((uint32_t)(n) << SCI_LINMASK_TXIDMASK_SHIFT)
#define SCI_LINMASK_RXIDMASK_SHIFT   (16)
#define SCI_LINMASK_RXIDMASK_MASK    (0xff << SCI_LINMASK_RXIDMASK_SHIFT)
#  define SCI_LINMASK_RXIDMASK(n)    ((uint32_t)(n) << SCI_LINMASK_RXIDMASK_SHIFT)

/* LIN Identification Register (LINID, offset 0x70) */

#define SCI_LINID_IDBYTE_SHIFT       (0)
#define SCI_LINID_IDBYTE_MASK        (0xff << SCI_LINID_IDBYTE_SHIFT)
#  define SCI_LINID_IDBYTE(n)        ((uint32_t)(n) << SCI_LINID_IDBYTE_SHIFT)
#define SCI_LINID_SLAVETASK_SHIFT    (8)
#define SCI_LINID_SLAVETASK_MASK     (0xff << SCI_LINID_SLAVETASK_SHIFT)
#  define SCI_LINID_SLAVETASK(n)     ((uint32_t)(n) << SCI_LINID_SLAVETASK_SHIFT)
#define SCI_LINID_RECEIVEDID_SHIFT   (16)
#define SCI_LINID_RECEIVEDID_MASK    (0xff << SCI_LINID_RECEIVEDID_SHIFT)

/* LIN Transmit Buffer Registers (LINTD0 offset 0x74, LINTD1 offset 0x78):
 * same byte packing as LINRD0/1 above (byte 0 in the MSB).
 */

#define SCI_LINTD_BYTE0_SHIFT        (24)
#define SCI_LINTD_BYTE1_SHIFT        (16)
#define SCI_LINTD_BYTE2_SHIFT        (8)
#define SCI_LINTD_BYTE3_SHIFT        (0)
#define SCI_LINTD_BYTE_MASK          (0xff)

/* Maximum Baud Rate Selection Register (MBRS, offset 0x7c): 13-bit
 * prescaler used by a LIN slave's adaptive baud rate detection (ADAPT
 * bit). Not used when ADAPT=0 (fixed baud rate, matching the SCI BRS
 * setup used elsewhere in this port).
 */

#define SCI_MBRS_MASK                (0x1fff)
#  define SCI_MBRS(n)                ((uint32_t)(n) & SCI_MBRS_MASK)

#endif /* __ARCH_ARM_SRC_RM57_HARDWARE_RM57_SCI_H */
