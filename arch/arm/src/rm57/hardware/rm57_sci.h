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
#define RM57_SCI_IODFTCTRL_OFFSET    0x0090 /* I/O Error Enable Register */

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

/* Bit 8:  Character type, 0=1 stop, 1=2 stop */
#define SCI_GCR1_CTYPE (1 << 8)

/* Bit 9:  LIN hardware generation control */
#define SCI_GCR1_HGEN_CTRL (1 << 9)
#define SCI_GCR1_RXENA               (1 << 24) /* Bit 24: Receiver enable */

/* Bit 25: Transmitter enable */
#define SCI_GCR1_TXENA (1 << 25)
#define SCI_GCR1_LOOPBACK            (1 << 16) /* Bit 16: Loop back enable */

/* Interrupt Flag / Set / Clear Register bits (common to SETINT/CLEARINT/
 * FLR) - values confirmed from TI's HALCoGen HL_sci.h sciInterrupt_t
 * enum, not guessed
 */

#define SCI_INT_BREAK                (0x00000001) /* Break detect */
#define SCI_INT_WAKE                 (0x00000002) /* Wakeup */
#define SCI_INT_TX                   (0x00000100) /* Transmit buffer ready */
#define SCI_INT_RX                   (0x00000200) /* Receive buffer ready */
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

#endif /* __ARCH_ARM_SRC_RM57_HARDWARE_RM57_SCI_H */
