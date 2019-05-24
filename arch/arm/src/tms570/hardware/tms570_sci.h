/****************************************************************************************************
 * arch/arm/src/tms570/hardware/tms570_sci.h
 * Secondary System Control Register Definitions
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *
 *   TMS570LS04x/03x 16/32-Bit RISC Flash Microcontroller, Technical Reference Manual, Texas
 *   Instruments, Literature Number: SPNU517A, September 2013
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************************************/

#ifndef __ARCH_ARM_SRC_TMS570_HARDWARE_TMS570_SCI_H
#define __ARCH_ARM_SRC_TMS570_HARDWARE_TMS570_SCI_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>
#include "hardware/tms570_memorymap.h"

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

/* Register Offsets *********************************************************************************/

#define TMS570_SCI_GCR0_OFFSET        0x0000 /* SCI Global Control Register 0 */
#define TMS570_SCI_GCR1_OFFSET        0x0004 /* SCI Global Control Register 1 */
#define TMS570_SCI_GCR2_OFFSET        0x0008 /* SCI Global Control Register 2 */
#define TMS570_SCI_SETINT_OFFSET      0x000c /* SCI Set Interrupt Register */
#define TMS570_SCI_CLEARINT_OFFSET    0x0010 /* SCI Clear Interrupt Register */
#define TMS570_SCI_SETINTLVL_OFFSET   0x0014 /* SCI Set Interrupt Level Register */
#define TMS570_SCI_CLEARINTLVL_OFFSET 0x0018 /* SCI Clear Interrupt Level Register */
#define TMS570_SCI_FLR_OFFSET         0x001c /* SCI Flags Register */
#define TMS570_SCI_INTVECT0_OFFSET    0x0020 /* SCI Interrupt Vector Offset 0 */
#define TMS570_SCI_INTVECT1_OFFSET    0x0024 /* SCI Interrupt Vector Offset 1 */
#define TMS570_SCI_FORMAT_OFFSET      0x0028 /* SCI Format Control Register */
#define TMS570_SCI_BRS_OFFSET         0x002c /* Baud Rate Selection Register */
#define TMS570_SCI_ED_OFFSET          0x0030 /* Receiver Emulation Data Buffer */
#define TMS570_SCI_RD_OFFSET          0x0034 /* Receiver Data Buffer */
#define TMS570_SCI_TD_OFFSET          0x0038 /* Transmit Data Buffer */
#define TMS570_SCI_PIO0_OFFSET        0x003c /* SCI Pin I/O Control Register 0 */
#define TMS570_SCI_PIO1_OFFSET        0x0040 /* SCI Pin I/O Control Register 1 */
#define TMS570_SCI_PIO2_OFFSET        0x0044 /* SCI Pin I/O Control Register 2 */
#define TMS570_SCI_PIO3_OFFSET        0x0048 /* SCI Pin I/O Control Register 3 */
#define TMS570_SCI_PIO4_OFFSET        0x004c /* SCI Pin I/O Control Register 4 */
#define TMS570_SCI_PIO5_OFFSET        0x0050 /* SCI Pin I/O Control Register 5 */
#define TMS570_SCI_PIO6_OFFSET        0x0054 /* SCI Pin I/O Control Register 6 */
#define TMS570_SCI_PIO7_OFFSET        0x0058 /* SCI Pin I/O Control Register 7 */
#define TMS570_SCI_PIO8_OFFSET        0x005c /* SCI Pin I/O Control Register 8 */
#define TMS570_LIN_COMPARE_OFFSET     0x0060 /* LIN Compare Register */
#define TMS570_LIN_RD0_OFFSET         0x0064 /* LIN Receive Buffer 0 Register */
#define TMS570_LIN_RD1_OFFSET         0x0068 /* LIN Receive Buffer 1 Register */
#define TMS570_LIN_MASK_OFFSET        0x006c /* LIN Mask Register */
#define TMS570_LIN_ID_OFFSET          0x0070 /* LIN Identification Register */
#define TMS570_LIN_TD0_OFFSET         0x0074 /* LIN Transmit Buffer 0 */
#define TMS570_LIN_TD1_OFFSET         0x0078 /* LIN Transmit Buffer 1 */
#define TMS570_SCI_MBRS_OFFSET        0x007c /* Maximum Baud Rate Selection Register */
#define TMS570_SCI_IODFTCTRL_OFFSET   0x0090 /* Input/Output Error Enable Register */

/* Friendler register aliases */

#define TMS570_SCI_FUN_OFFSET         TMS570_SCI_PIO0_OFFSET  /* Pin Function Register */
#define TMS570_SCI_DIR_OFFSET         TMS570_SCI_PIO1_OFFSET  /* Pin Direction Register */
#define TMS570_SCI_DIN_OFFSET         TMS570_SCI_PIO2_OFFSET  /* Pin Data In Register */
#define TMS570_SCI_DOUT_OFFSET        TMS570_SCI_PIO3_OFFSET  /* Pin Data Out Register */
#define TMS570_SCI_SET_OFFSET         TMS570_SCI_PIO4_OFFSET  /* Pin Data Set Register */
#define TMS570_SCI_CLR_OFFSET         TMS570_SCI_PIO5_OFFSET  /* Pin Data Clr Register */
#define TMS570_SCI_ODR_OFFSET         TMS570_SCI_PIO6_OFFSET  /* Pin Open Drain Output Enable Register */
#define TMS570_SCI_PD_OFFSET          TMS570_SCI_PIO7_OFFSET  /* Pin Pullup/Pulldown Disable Register */
#define TMS570_SCI_PSL_OFFSET         TMS570_SCI_PIO8_OFFSET  /* Pin Pullup/Pulldown Selection Register */

/* Register Addresses *******************************************************************************/

#define TMS570_SCI1_GCR0              (TMS570_SCI1_BASE+TMS570_SCI_GCR0_OFFSET)
#define TMS570_SCI1_GCR1              (TMS570_SCI1_BASE+TMS570_SCI_GCR1_OFFSET)
#define TMS570_SCI1_GCR2              (TMS570_SCI1_BASE+TMS570_SCI_GCR2_OFFSET)
#define TMS570_SCI1_SETINT            (TMS570_SCI1_BASE+TMS570_SCI_SETINT_OFFSET)
#define TMS570_SCI1_CLEARINT          (TMS570_SCI1_BASE+TMS570_SCI_CLEARINT_OFFSET)
#define TMS570_SCI1_SETINTLVL         (TMS570_SCI1_BASE+TMS570_SCI_SETINTLVL_OFFSET)
#define TMS570_SCI1_CLEARINTLVL       (TMS570_SCI1_BASE+TMS570_SCI_CLEARINTLVL_OFFSET)
#define TMS570_SCI1_FLR               (TMS570_SCI1_BASE+TMS570_SCI_FLR_OFFSET)
#define TMS570_SCI1_INTVECT0          (TMS570_SCI1_BASE+TMS570_SCI_INTVECT0_OFFSET)
#define TMS570_SCI1_INTVECT1          (TMS570_SCI1_BASE+TMS570_SCI_INTVECT1_OFFSET)
#define TMS570_SCI1_FORMAT            (TMS570_SCI1_BASE+TMS570_SCI_FORMAT_OFFSET)
#define TMS570_SCI1_BRS               (TMS570_SCI1_BASE+TMS570_SCI_BRS_OFFSET)
#define TMS570_SCI1_ED                (TMS570_SCI1_BASE+TMS570_SCI_ED_OFFSET)
#define TMS570_SCI1_RD                (TMS570_SCI1_BASE+TMS570_SCI_RD_OFFSET)
#define TMS570_SCI1_TD                (TMS570_SCI1_BASE+TMS570_SCI_TD_OFFSET)
#define TMS570_SCI1_PIO0              (TMS570_SCI1_BASE+TMS570_SCI_PIO0_OFFSET)
#define TMS570_SCI1_PIO1              (TMS570_SCI1_BASE+TMS570_SCI_PIO1_OFFSET)
#define TMS570_SCI1_PIO2              (TMS570_SCI1_BASE+TMS570_SCI_PIO2_OFFSET)
#define TMS570_SCI1_PIO3              (TMS570_SCI1_BASE+TMS570_SCI_PIO3_OFFSET)
#define TMS570_SCI1_PIO4              (TMS570_SCI1_BASE+TMS570_SCI_PIO4_OFFSET)
#define TMS570_SCI1_PIO5              (TMS570_SCI1_BASE+TMS570_SCI_PIO5_OFFSET)
#define TMS570_SCI1_PIO6              (TMS570_SCI1_BASE+TMS570_SCI_PIO6_OFFSET)
#define TMS570_SCI1_PIO7              (TMS570_SCI1_BASE+TMS570_SCI_PIO7_OFFSET)
#define TMS570_SCI1_PIO8              (TMS570_SCI1_BASE+TMS570_SCI_PIO8_OFFSET)
#define TMS570_LIN1_COMPARE           (TMS570_SCI1_BASE+TMS570_LIN_COMPARE_OFFSET)
#define TMS570_LIN1_RD0               (TMS570_SCI1_BASE+TMS570_LIN_RD0_OFFSET)
#define TMS570_LIN1_RD1               (TMS570_SCI1_BASE+TMS570_LIN_RD1_OFFSET)
#define TMS570_LIN1_MASK              (TMS570_SCI1_BASE+TMS570_LIN_MASK_OFFSET)
#define TMS570_LIN1_ID                (TMS570_SCI1_BASE+TMS570_LIN_ID_OFFSET)
#define TMS570_LIN1_TD0               (TMS570_SCI1_BASE+TMS570_LIN_TD0_OFFSET)
#define TMS570_LIN1_TD1               (TMS570_SCI1_BASE+TMS570_LIN_TD1_OFFSET)
#define TMS570_SCI1_MBRS              (TMS570_SCI1_BASE+TMS570_SCI_MBRS_OFFSET)
#define TMS570_SCI1_IODFTCTRL         (TMS570_SCI1_BASE+TMS570_SCI_IODFTCTRL_OFFSET)

#define TMS570_SCI1_FUN               TMS570_SCI_PIO0  /* Pin Function Register */
#define TMS570_SCI1_DIR               TMS570_SCI_PIO1  /* Pin Direction Register */
#define TMS570_SCI1_DIN               TMS570_SCI_PIO2  /* Pin Data In Register */
#define TMS570_SCI1_DOUT              TMS570_SCI_PIO3  /* Pin Data Out Register */
#define TMS570_SCI1_SET               TMS570_SCI_PIO4  /* Pin Data Set Register */
#define TMS570_SCI1_CLR               TMS570_SCI_PIO5  /* Pin Data Clr Register */
#define TMS570_SCI1_ODR               TMS570_SCI_PIO6  /* Pin Open Drain Output Enable Register */
#define TMS570_SCI1_PD                TMS570_SCI_PIO7  /* Pin Pullup/Pulldown Disable Register */
#define TMS570_SCI1_PSL               TMS570_SCI_PIO8  /* Pin Pullup/Pulldown Selection Register */

#if TMS570_NSCI > 1
#  define TMS570_SCI2_GCR0            (TMS570_SCI2_BASE+TMS570_SCI_GCR0_OFFSET)
#  define TMS570_SCI2_GCR1            (TMS570_SCI2_BASE+TMS570_SCI_GCR1_OFFSET)
#  define TMS570_SCI2_GCR2            (TMS570_SCI2_BASE+TMS570_SCI_GCR2_OFFSET)
#  define TMS570_SCI2_SETINT          (TMS570_SCI2_BASE+TMS570_SCI_SETINT_OFFSET)
#  define TMS570_SCI2_CLEARINT        (TMS570_SCI2_BASE+TMS570_SCI_CLEARINT_OFFSET)
#  define TMS570_SCI2_SETINTLVL       (TMS570_SCI2_BASE+TMS570_SCI_SETINTLVL_OFFSET)
#  define TMS570_SCI2_CLEARINTLVL     (TMS570_SCI2_BASE+TMS570_SCI_CLEARINTLVL_OFFSET)
#  define TMS570_SCI2_FLR             (TMS570_SCI2_BASE+TMS570_SCI_FLR_OFFSET)
#  define TMS570_SCI2_INTVECT0        (TMS570_SCI2_BASE+TMS570_SCI_INTVECT0_OFFSET)
#  define TMS570_SCI2_INTVECT1        (TMS570_SCI2_BASE+TMS570_SCI_INTVECT1_OFFSET)
#  define TMS570_SCI2_FORMAT          (TMS570_SCI2_BASE+TMS570_SCI_FORMAT_OFFSET)
#  define TMS570_SCI2_BRS             (TMS570_SCI2_BASE+TMS570_SCI_BRS_OFFSET)
#  define TMS570_SCI2_ED              (TMS570_SCI2_BASE+TMS570_SCI_ED_OFFSET)
#  define TMS570_SCI2_RD              (TMS570_SCI2_BASE+TMS570_SCI_RD_OFFSET)
#  define TMS570_SCI2_TD              (TMS570_SCI2_BASE+TMS570_SCI_TD_OFFSET)
#  define TMS570_SCI2_PIO0            (TMS570_SCI2_BASE+TMS570_SCI_PIO0_OFFSET)
#  define TMS570_SCI2_PIO1            (TMS570_SCI2_BASE+TMS570_SCI_PIO1_OFFSET)
#  define TMS570_SCI2_PIO2            (TMS570_SCI2_BASE+TMS570_SCI_PIO2_OFFSET)
#  define TMS570_SCI2_PIO3            (TMS570_SCI2_BASE+TMS570_SCI_PIO3_OFFSET)
#  define TMS570_SCI2_PIO4            (TMS570_SCI2_BASE+TMS570_SCI_PIO4_OFFSET)
#  define TMS570_SCI2_PIO5            (TMS570_SCI2_BASE+TMS570_SCI_PIO5_OFFSET)
#  define TMS570_SCI2_PIO6            (TMS570_SCI2_BASE+TMS570_SCI_PIO6_OFFSET)
#  define TMS570_SCI2_PIO7            (TMS570_SCI2_BASE+TMS570_SCI_PIO7_OFFSET)
#  define TMS570_SCI2_PIO8            (TMS570_SCI2_BASE+TMS570_SCI_PIO8_OFFSET)
#  define TMS570_LIN2_COMPARE         (TMS570_SCI2_BASE+TMS570_LIN_COMPARE_OFFSET)
#  define TMS570_LIN2_RD0             (TMS570_SCI2_BASE+TMS570_LIN_RD0_OFFSET)
#  define TMS570_LIN2_RD1             (TMS570_SCI2_BASE+TMS570_LIN_RD1_OFFSET)
#  define TMS570_LIN2_MASK            (TMS570_SCI2_BASE+TMS570_LIN_MASK_OFFSET)
#  define TMS570_LIN2_ID              (TMS570_SCI2_BASE+TMS570_LIN_ID_OFFSET)
#  define TMS570_LIN2_TD0             (TMS570_SCI2_BASE+TMS570_LIN_TD0_OFFSET)
#  define TMS570_LIN2_TD1             (TMS570_SCI2_BASE+TMS570_LIN_TD1_OFFSET)
#  define TMS570_SCI2_MBRS            (TMS570_SCI2_BASE+TMS570_SCI_MBRS_OFFSET)
#  define TMS570_SCI2_IODFTCTRL       (TMS570_SCI2_BASE+TMS570_SCI_IODFTCTRL_OFFSET)

#  define TMS570_SCI1_FUN             TMS570_SCI_PIO0  /* Pin Function Register */
#  define TMS570_SCI1_DIR             TMS570_SCI_PIO1  /* Pin Direction Register */
#  define TMS570_SCI1_DIN             TMS570_SCI_PIO2  /* Pin Data In Register */
#  define TMS570_SCI1_DOUT            TMS570_SCI_PIO3  /* Pin Data Out Register */
#  define TMS570_SCI1_SET             TMS570_SCI_PIO4  /* Pin Data Set Register */
#  define TMS570_SCI1_CLR             TMS570_SCI_PIO5  /* Pin Data Clr Register */
#  define TMS570_SCI1_ODR             TMS570_SCI_PIO6  /* Pin Open Drain Output Enable Register */
#  define TMS570_SCI1_PD              TMS570_SCI_PIO7  /* Pin Pullup/Pulldown Disable Register */
#  define TMS570_SCI1_PSL             TMS570_SCI_PIO8  /* Pin Pullup/Pulldown Selection Register */
#endif

/* Register Bit-Field Definitions *******************************************************************/

/* SCI Global Control Register 0 */

#define SCI_GCR0_RESET                (1 << 0)  /* Bit 0:  SCI/LIN module is out of reset */

/* SCI Global Control Register 1 */

#define SCI_GCR1_COMM                 (1 << 0)  /* Bit 0:  SCI/LIN communication mode */
#define SCI_GCR1_TIMING               (1 << 1)  /* Bit 1:  SCI timing mode */
#define SCI_GCR1_PARENA               (1 << 2)  /* Bit 2:  Parity enable */
#define SCI_GCR1_PARITY               (1 << 3)  /* Bit 3:  SCI parity odd/even selection */
#define SCI_GCR1_STOP                 (1 << 4)  /* Bit 4:  SCI number of stop bits per frame */
#define SCI_GCR1_CLOCK                (1 << 5)  /* Bit 5:  SCI internal clock enable */
#define SCI_GCR1_LIN                  (1 << 6)  /* Bit 6:  LIN mode */
#define SCI_GCR1_SWRST                (1 << 7)  /* Bit 7:  Software reset (active low) */
#define SCI_GCR1_SLEEP                (1 << 8)  /* Bit 8:  SCI sleep */
#define SCI_GCR1_ADAPT                (1 << 9)  /* Bit 9:  Adapt */
#define SCI_GCR1_MBUF                 (1 << 10) /* Bit 10: Multibuffer mode */
#define SCI_GCR1_CTYPE                (1 << 11) /* Bit 11:  Checksum type */
#define SCI_GCR1_HGEN                 (1 << 12) /* Bit 12: HGEN control */
#define SCI_GCR1_STOPEXT              (1 << 13) /* Bit 13: Stop extended frame communication */
#define SCI_GCR1_LOOPBACK             (1 << 16) /* Bit 16: Loopback bit */
#define SCI_GCR1_CONT                 (1 << 17) /* Bit 13: Continue on suspend */
#define SCI_GCR1_RXENA                (1 << 24) /* Bit 24: Receive enable */
#define SCI_GCR1_TXENA                (1 << 25) /* Bit 25: Transmit enable */

/* SCI Global Control Register 2 */

#define SCI_GCR2_POWERDOWN            (1 << 0)  /* Bit 0:  Power down */
#define SCI_GCR2_GENWU                (1 << 8)  /* Bit 8:  Generate wakeup signal */
#define SCI_GCR2_SC                   (1 << 16) /* Bit 16: Send checksum byte */
#define SCI_GCR2_CC                   (1 << 17) /* Bit 17: Compare checksum */

/* SCI Set Interrupt Register,
 * SCI Clear Interrupt Register,
 ( SCI Set Interrupt Level Register, and
 * SCI Clear Interrupt Level Register
 */

#define SCI_INT_BRKDT                 (1 << 0)  /* Bit 0:  Break detect interrupt */
#define SCI_INT_WAKEUP                (1 << 1)  /* Bit 1:  Wake-up interrupt */
#define SCI_INT_TIMEOUT               (1 << 4)  /* Bit 4:  Timeout interrupt */
#define SCI_INT_TOAWUS                (1 << 6)  /* Bit 6:  Timeout after wakeup signal interrupt */
#define SCI_INT_TOA3WUS               (1 << 7)  /* Bit 7:  Timeout after 2 Wakeup signls interrupt */
#define SCI_INT_TX                    (1 << 8)  /* Bit 8:  Tranmitter interrupt */
#define SCI_INT_RX                    (1 << 9)  /* Bit 9:  Receiver interrupt */
#define SCI_INT_ID                    (1 << 13) /* Bit 13: Identification interrupt */
#define SCI_INT_PE                    (1 << 24) /* Bit 24: Parity error interrupt */
#define SCI_INT_OE                    (1 << 25) /* Bit 25: Overrun error interrupt */
#define SCI_INT_FE                    (1 << 26) /* Bit 26: Framing error interrupt */
#define SCI_INT_NRE                   (1 << 27) /* Bit 27: No response error interrupt */
#define SCI_INT_ISFE                  (1 << 28) /* Bit 28: Inconsistent synch field error interrupt */
#define SCI_INT_CE                    (1 << 29) /* Bit 29: Checksum error interrupt */
#define SCI_INT_PBE                   (1 << 30) /* Bit 30: Physical bus error interrupt */
#define SCI_INT_BE                    (1 << 31) /* Bit 31: Bit error interrupt */

#define SCI_INT_ALLERRORS             0xff000001
#define SCI_INT_LINERRORS             0xff000000
#define SCI_INT_SCIERRORS             0x87000001
#define SCI_INT_ALLINTS               0xff0023d3

/* SCI Flags Register */

#define SCI_FLR_BRKDT                 (1 << 0)  /* Bit 0:  Break detect flag */
#define SCI_FLR_WAKEUP                (1 << 1)  /* Bit 1:  Wake-up flag */
#define SCI_FLR_IDLE                  (1 << 2)  /* Bit 2:  SCI receiver in idle state */
#define SCI_FLR_BUSY                  (1 << 3)  /* Bit 3:  Bus busy flag */
#define SCI_FLR_TIMEOUT               (1 << 4)  /* Bit 4:  Timeout flag */
#define SCI_FLR_TOAWUS                (1 << 6)  /* Bit 6:  Timeout after wakeup signal flag */
#define SCI_FLR_TOA3WUS               (1 << 7)  /* Bit 7:  Timeout after 2 Wakeup signls flag */
#define SCI_FLR_TXRDY                 (1 << 8)  /* Bit 8:  Transmitter buffer register ready flag */
#define SCI_FLR_RXRDY                 (1 << 9)  /* Bit 9:  Receiver ready flag */
#define SCI_FLR_TXWAKE                (1 << 10) /* Bit 10: Transmitter wakeup method select */
#define SCI_FLR_TXEMPTY               (1 << 11) /* Bit 11: Transmitter empty flag */
#define SCI_FLR_RXWAKE                (1 << 12) /* Bit 12: Receiver wakeup detect flag */
#define SCI_FLR_IDTX                  (1 << 13) /* Bit 13: Identifier on transmit flag */
#define SCI_FLR_IDRX                  (1 << 14) /* Bit 14: Identifier on receive flag */
#define SCI_FLR_PE                    (1 << 24) /* Bit 24: Parity error flag */
#define SCI_FLR_OE                    (1 << 25) /* Bit 25: Overrun error flag */
#define SCI_FLR_FE                    (1 << 26) /* Bit 26: Framing error flag */
#define SCI_FLR_NRE                   (1 << 27) /* Bit 27: No response error flag */
#define SCI_FLR_ISFE                  (1 << 28) /* Bit 28: Inconsistent synch field error flag */
#define SCI_FLR_CE                    (1 << 29) /* Bit 29: checksum error flag */
#define SCI_FLR_PBE                   (1 << 30) /* Bit 30: Physical bus error flag */
#define SCI_FLR_BE                    (1 << 31) /* Bit 31: Bit error flag */

/* SCI Interrupt Vector Offset 0/1 */

#define SCI_INTVECT_MASK              (0x1f)    /* Bits 0-4: Interrupt vector offset */
#  define SCI_INTVECT_NONE            (0)       /*   No interrupt */
#  define SCI_INTVECT_WAKEUP          (1)       /*   Wake-up interrupt */
#  define SCI_INTVECT_ISFE            (2)       /*   Inconsistent synch field error interrupt */
#  define SCI_INTVECT_PE              (3)       /*   Parity error interrupt */
#  define SCI_INTVECT_ID              (4)       /*   Identification interrupt */
#  define SCI_INTVECT_PBE             (5)       /*   Physical bus error interrupt */
#  define SCI_INTVECT_FE              (6)       /*   Framing error interrupt */
#  define SCI_INTVECT_BRKDT           (7)       /*   Break detect interrupt */
#  define SCI_INTVECT_CE              (8)       /*   Checksum error interrupt */
#  define SCI_INTVECT_OE              (9)       /*   Overrun error interrupt */
#  define SCI_INTVECT_BE              (10)      /*   Bit error interrupt */
#  define SCI_INTVECT_RX              (11)      /*   Receive interrupt */
#  define SCI_INTVECT_TX              (12)      /*   Tranmit interrupt */
#  define SCI_INTVECT_NRE             (13)      /*   No response error interrupt */
#  define SCI_INTVECT_TOAWUS          (14)      /*   Timeout after wakeup signal interrupt */
#  define SCI_INTVECT_TOA3WUS         (15)      /*   Timeout after 2 Wakeup signls interrupt */
#  define SCI_INTVECT_TIMEOUT         (16)      /*   Timeout interrupt */

/* SCI Format Control Register */

#define SCI_FORMAT_CHAR_SHIFT         (0)       /* Bits 0-2:  Frame length control bits */
#define SCI_FORMAT_CHAR_MASK          (7 << SCI_FORMAT_CHAR_SHIFT)
#  define SCI_FORMAT_CHAR(n)          ((uint32_t)(n) << SCI_FORMAT_CHAR_SHIFT)
#define SCI_FORMAT_LENGTH_SHIFT       (16)      /* Bits 16-18:  Character length control bits */
#define SCI_FORMAT_LENGTH_MASK        (7 << SCI_FORMAT_LENGTH_SHIFT)
#  define SCI_FORMAT_LENGTH(n)        ((uint32_t)(n) << SCI_FORMAT_LENGTH_SHIFT)

/* Baud Rate Selection Register */

#define SCI_BRS_P_SHIFT               (0)       /* Bits 0-23: Baud integer divider */
#define SCI_BRS_P_MASK                (0x00ffffff << SCI_BRS_P_SHIFT)
#  define SCI_BRS_P(n)                ((uint32_t)(n) << SCI_BRS_P_SHIFT)
#define SCI_BRS_M_SHIFT               (24)      /* Bits 24-27: SCI/LIN 4-bit fractional divider selection */
#define SCI_BRS_M_MASK                (15 << SCI_BRS_M_SHIFT)
#  define SCI_BRS_M(n)                ((uint32_t)(n) << SCI_BRS_M_SHIFT)
#define SCI_BRS_U_SHIFT               (28)      /* Bits 28-30: SCI/LIN super fractional divider */
#define SCI_BRS_U_MASK                (7 << SCI_BRS_U_SHIFT)
#  define SCI_BRS_U(n)                ((uint32_t)(n) << SCI_BRS_U_SHIFT)

/* Receiver Emulation Data Buffer */
#define SCI_ED_
/* Receiver Data Buffer */
#define SCI_RD_
/* Transmit Data Buffer */
#define SCI_TD_

/* SCI Pin I/O Control Register 0: Pin Function Register,
 * SCI Pin I/O Control Register 1: Pin Direction Register,
 * SCI Pin I/O Control Register 2: Pin Data In Register,
 * SCI Pin I/O Control Register 3: Pin Data Out Register,
 * SCI Pin I/O Control Register 4: Pin Data Set Register,
 * SCI Pin I/O Control Register 5: Pin Data Clr Register,
 * SCI Pin I/O Control Register 6: Pin Open Drain Output Enable Register,
 * SCI Pin I/O Control Register 7: Pin Pullup/Pulldown Disable Register, and
 * SCI Pin I/O Control Register 8: Pin Pullup/Pulldown Selection Register.
 */

#define SCI_PIO_RX                    (1 << 1)  /* Bit 1: RX pin */
#define SCI_PIO_TX                    (1 << 2)  /* Bit 2: TX pin */

/* LIN Compare Register */
#define LIN_COMPARE_
/* LIN Receive Buffer 0 Register */
#define LIN_RD0_
/* LIN Receive Buffer 1 Register */
#define LIN_RD1_
/* LIN Mask Register */
#define LIN_MASK_
/* LIN Identification Register */
#define LIN_ID_
/* LIN Transmit Buffer 0 */
#define LIN_TD0_
/* LIN Transmit Buffer 1 */
#define LIN_TD1_
/* Maximum Baud Rate Selection Register */
#define SCI_MBRS_
/* Input/Output Error Enable Register */
#define SCI_IODFTCTRL_

#endif /* __ARCH_ARM_SRC_TMS570_HARDWARE_TMS570_SCI_H */
