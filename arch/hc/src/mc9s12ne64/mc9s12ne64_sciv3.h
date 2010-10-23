/************************************************************************************
 * arch/hc/src/mc9s12ne64/mc9s12ne64_sciv3.h
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 ************************************************************************************/

#ifndef __ARCH_ARM_HC_SRC_MC9S12NE64_MC9S12NE64_SCIV3_H
#define __ARCH_ARM_HC_SRC_MC9S12NE64_MC9S12NE64_SCIV3_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define HCS12_SCI_SCIBDH_OFFSET       0x00 /* SCI Baud Rate Register High */
#define HCS12_SCI_SCIBDL_OFFSET       0x01 /* SCI Baud Rate Register Low */
#define HCS12_SCI_SCICR1_OFFSET       0x02 /* SCI Control Register 1 */
#define HCS12_SCI_SCICR2_OFFSET       0x03 /* SCI Control Register 2 */
#define HCS12_SCI_SCISR1_OFFSET       0x04 /* SCI Status Register 1 */
#define HCS12_SCI_SCISR2_OFFSET       0x05 /* SCI Status Register 2 */
#define HCS12_SCI_SCIDRH_OFFSET       0x06 /* SCI Data Register High */
#define HCS12_SCI_SCIDRL_OFFSET       0x07 /* SCI Data Register Low */

/* Register Addresses ***************************************************************/

#define HCS12_SCI0_SCIBDH             (HCS12_SCI0_BASE+HCS12_SCI_SCIBDH_OFFSET)
#define HCS12_SCI0_SCIBDL             (HCS12_SCI0_BASE+HCS12_SCI_SCIBDL_OFFSET)
#define HCS12_SCI0_SCICR1             (HCS12_SCI0_BASE+HCS12_SCI_SCICR1_OFFSET)
#define HCS12_SCI0_SCICR2             (HCS12_SCI0_BASE+HCS12_SCI_SCICR2_OFFSET)
#define HCS12_SCI0_SCISR1             (HCS12_SCI0_BASE+HCS12_SCI_SCISR1_OFFSET)
#define HCS12_SCI0_SCISR2             (HCS12_SCI0_BASE+HCS12_SCI_SCISR2_OFFSET)
#define HCS12_SCI0_SCIDRH             (HCS12_SCI0_BASE+HCS12_SCI_SCIDRH_OFFSET)
#define HCS12_SCI0_SCIDRL             (HCS12_SCI0_BASE+HCS12_SCI_SCIDRL_OFFSET)

#define HCS12_SCI1_SCIBDH             (HCS12_SCI1_BASE+HCS12_SCI_SCIBDH_OFFSET)
#define HCS12_SCI1_SCIBDL             (HCS12_SCI1_BASE+HCS12_SCI_SCIBDL_OFFSET)
#define HCS12_SCI1_SCICR1             (HCS12_SCI1_BASE+HCS12_SCI_SCICR1_OFFSET)
#define HCS12_SCI1_SCICR2             (HCS12_SCI1_BASE+HCS12_SCI_SCICR2_OFFSET)
#define HCS12_SCI1_SCISR1             (HCS12_SCI1_BASE+HCS12_SCI_SCISR1_OFFSET)
#define HCS12_SCI1_SCISR2             (HCS12_SCI1_BASE+HCS12_SCI_SCISR2_OFFSET)
#define HCS12_SCI1_SCIDRH             (HCS12_SCI1_BASE+HCS12_SCI_SCIDRH_OFFSET)
#define HCS12_SCI1_SCIDRL             (HCS12_SCI1_BASE+HCS12_SCI_SCIDRL_OFFSET)

/* Register Bit-Field Definitions ***************************************************/

/* SCI Baud Rate Register High Bit-Field Definitions */

#define SCI_SCIBDH_SBR_SHIFT          (0)       /* Bits 0-4: SBR[11:8] */
#define SCI_SCIBDH_SBR_MASK           (0x1f << SCI_SCIBDH_SBR_SHIFT)
#define SCI_SCIBDH_TNP_SHIFT          (5)       /* Bits 5-6: IRSCI Transmit Pulse Width */
#define SCI_SCIBDH_TNP_MASK           (3 << SCI_SCIBDH_TNP_SHIFT)
#  define SCI_SCIBDH_TNP_132          (0 << SCI_SCIBDH_TNP_SHIFT) /* 1/32 */
#  define SCI_SCIBDH_TNP_116          (1 << SCI_SCIBDH_TNP_SHIFT) /* 1/16 */
#  define SCI_SCIBDH_TNP_316          (2 << SCI_SCIBDH_TNP_SHIFT) /* 3/16 */
#define SCI_SCIBDH_IREN               (1 << 7)  /* Bit 7:  Infrared Enable */

/* SCI Baud Rate Register Low Bit-Field Definitions */
/* This register holds the low 7 bits of the baud bits SBR[7:0] */

/* SCI Control Register 1 Bit-Field Definitions */

#define SCI_SCICR1_PT                 (1 << 0)  /* Bit 0:  Parity Type */
#define SCI_SCICR1_PE                 (1 << 1)  /* Bit 1:  Parity Enable */
#define SCI_SCICR1_ILT                (1 << 2)  /* Bit 2:  Idle Line Type */
#define SCI_SCICR1_WAKE               (1 << 3)  /* Bit 3:  Wakeup Condition */
#define SCI_SCICR1_M                  (1 << 4)  /* Bit 4:  Data Format Mode */
#define SCI_SCICR1_RSRC               (1 << 5)  /* Bit 5:  Receiver Source */
#define SCI_SCICR1_SCISWAI            (1 << 6)  /* Bit 6:  SCI Stop in Wait Mode */
#define SCI_SCICR1_LOOPS              (1 << 7)  /* Bit 7:  Enables loop operation */

/* SCI Control Register 2 Bit-Field Definitions */

#define SCI_SCICR2_SBK                (1 << 0)  /* Bit 0:  Send Break */
#define SCI_SCICR2_RWU                (1 << 1)  /* Bit 1:  Receiver Wakeup */
#define SCI_SCICR2_RE                 (1 << 2)  /* Bit 2:  Receiver Enable */
#define SCI_SCICR2_TE                 (1 << 3)  /* Bit 3:  Transmitter Enable */
#define SCI_SCICR2_ILIE               (1 << 4)  /* Bit 4:  Idle Line Interrupt Enable */
#define SCI_SCICR2_RIE                (1 << 5)  /* Bit 5:  Receiver Full Interrupt Enable Bit */
#define SCI_SCICR2_TCIE               (1 << 6)  /* Bit 6:  Transmission Complete Interrupt En */
#define SCI_SCICR2_TIE                (1 << 7)  /* Bit 7:  Transmitter Interrupt Ena */

/* SCI Status Register 1 Bit-Field Definitions */

#define SCI_SCISR1_PF                 (1 << 0)  /* Bit 0:  Parity Error */
#define SCI_SCISR1_FE                 (1 << 1)  /* Bit 1:  Framing Error */
#define SCI_SCISR1_NF                 (1 << 2)  /* Bit 2:  Noise */
#define SCI_SCISR1_OR                 (1 << 3)  /* Bit 3:  Overrun */
#define SCI_SCISR1_IDLE               (1 << 4)  /* Bit 4:  Idle Line */
#define SCI_SCISR1_RDRF               (1 << 5)  /* Bit 5:  Receive Data Register Full */
#define SCI_SCISR1_TC                 (1 << 6)  /* Bit 6:  Transmit Complete */
#define SCI_SCISR1_TDRE               (1 << 7)  /* Bit 7:  Transmit Data Register Empty */

/* SCI Status Register 2 Bit-Field Definitions */

#define SCI_SCISR2_BRK13              (1 << 2)  /* Bit 2:  Break Transmit Character Length */
#define SCI_SCISR2_TXDIR              (1 << 1)  /* Bit 1:  Transmitter Pin Data Direction in Single-Wire */
#define SCI_SCISR2_RAF                (1 << 0)  /* Bit 0:  Receiver Active */

/* SCI Data Register High Bit-Field Definitions */

#define SCI_SCIDRH_T8                 (1 << 6)  /* Bit 6:  Transmit Bit 8 */
#define SCI_SCIDRH_R8                 (1 << 7)  /* Bit 7:  Received Bit 8 */

/* SCI Data Register Low Bit-Field Definitions */
/* Receive/Transmit bits 0-7 */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_HC_SRC_MC9S12NE64_MC9S12NE64_SCIV3_H */
