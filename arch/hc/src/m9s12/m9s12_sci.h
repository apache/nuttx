/****************************************************************************
 * arch/hc/src/m9s12/m9s12_sci.h
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

#ifndef __ARCH_HC_SRC_M9S12_M9S12_SCI_H
#define __ARCH_HC_SRC_M9S12_M9S12_SCI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define HCS12_SCI_BDH_OFFSET      0x00 /* SCI Baud Rate Register High */
#define HCS12_SCI_BDL_OFFSET      0x01 /* SCI Baud Rate Register Low */
#define HCS12_SCI_CR1_OFFSET      0x02 /* SCI Control Register 1 */
#define HCS12_SCI_CR2_OFFSET      0x03 /* SCI Control Register 2 */
#define HCS12_SCI_SR1_OFFSET      0x04 /* SCI Status Register 1 */
#define HCS12_SCI_SR2_OFFSET      0x05 /* SCI Status Register 2 */
#define HCS12_SCI_DRH_OFFSET      0x06 /* SCI Data Register High */
#define HCS12_SCI_DRL_OFFSET      0x07 /* SCI Data Register Low */

/* Register Addresses *******************************************************/

#define HCS12_SCI0_BDH            (HCS12_REG_BASE+HCS12_SCI0_BASE+HCS12_SCI_BDH_OFFSET)
#define HCS12_SCI0_BDL            (HCS12_REG_BASE+HCS12_SCI0_BASE+HCS12_SCI_BDL_OFFSET)
#define HCS12_SCI0_CR1            (HCS12_REG_BASE+HCS12_SCI0_BASE+HCS12_SCI_CR1_OFFSET)
#define HCS12_SCI0_CR2            (HCS12_REG_BASE+HCS12_SCI0_BASE+HCS12_SCI_CR2_OFFSET)
#define HCS12_SCI0_SR1            (HCS12_REG_BASE+HCS12_SCI0_BASE+HCS12_SCI_SR1_OFFSET)
#define HCS12_SCI0_SR2            (HCS12_REG_BASE+HCS12_SCI0_BASE+HCS12_SCI_SR2_OFFSET)
#define HCS12_SCI0_DRH            (HCS12_REG_BASE+HCS12_SCI0_BASE+HCS12_SCI_DRH_OFFSET)
#define HCS12_SCI0_DRL            (HCS12_REG_BASE+HCS12_SCI0_BASE+HCS12_SCI_DRL_OFFSET)

#define HCS12_SCI1_BDH            (HCS12_REG_BASE+HCS12_SCI1_BASE+HCS12_SCI_BDH_OFFSET)
#define HCS12_SCI1_BDL            (HCS12_REG_BASE+HCS12_SCI1_BASE+HCS12_SCI_BDL_OFFSET)
#define HCS12_SCI1_CR1            (HCS12_REG_BASE+HCS12_SCI1_BASE+HCS12_SCI_CR1_OFFSET)
#define HCS12_SCI1_CR2            (HCS12_REG_BASE+HCS12_SCI1_BASE+HCS12_SCI_CR2_OFFSET)
#define HCS12_SCI1_SR1            (HCS12_REG_BASE+HCS12_SCI1_BASE+HCS12_SCI_SR1_OFFSET)
#define HCS12_SCI1_SR2            (HCS12_REG_BASE+HCS12_SCI1_BASE+HCS12_SCI_SR2_OFFSET)
#define HCS12_SCI1_DRH            (HCS12_REG_BASE+HCS12_SCI1_BASE+HCS12_SCI_DRH_OFFSET)
#define HCS12_SCI1_DRL            (HCS12_REG_BASE+HCS12_SCI1_BASE+HCS12_SCI_DRL_OFFSET)

/* Register Bit-Field Definitions *******************************************/

/* SCI Baud Rate Register High Bit-Field Definitions */

#define SCI_BDH_SBR_SHIFT         (0)       /* Bits 0-4: SBR[11:8] */
#define SCI_BDH_SBR_MASK          (0x1f << SCI_BDH_SBR_SHIFT)
#define SCI_BDH_TNP_SHIFT         (5)       /* Bits 5-6: IRSCI Transmit Pulse Width */
#define SCI_BDH_TNP_MASK          (3 << SCI_BDH_TNP_SHIFT)
#  define SCI_BDH_TNP_132         (0 << SCI_BDH_TNP_SHIFT) /* 1/32 */
#  define SCI_BDH_TNP_116         (1 << SCI_BDH_TNP_SHIFT) /* 1/16 */
#  define SCI_BDH_TNP_316         (2 << SCI_BDH_TNP_SHIFT) /* 3/16 */

#define SCI_BDH_IREN              (1 << 7)  /* Bit 7:  Infrared Enable */

/* SCI Baud Rate Register Low Bit-Field Definitions */

/* This register holds the low 7 bits of the baud bits SBR[7:0] */

/* SCI Control Register 1 Bit-Field Definitions */

#define SCI_CR1_PT                (1 << 0)  /* Bit 0:  Parity Type */
#define SCI_CR1_PE                (1 << 1)  /* Bit 1:  Parity Enable */
#define SCI_CR1_ILT               (1 << 2)  /* Bit 2:  Idle Line Type */
#define SCI_CR1_WAKE              (1 << 3)  /* Bit 3:  Wakeup Condition */
#define SCI_CR1_M                 (1 << 4)  /* Bit 4:  Data Format Mode */
#define SCI_CR1_RSRC              (1 << 5)  /* Bit 5:  Receiver Source */
#define SCI_CR1_SCISWAI           (1 << 6)  /* Bit 6:  SCI Stop in Wait Mode */
#define SCI_CR1_LOOPS             (1 << 7)  /* Bit 7:  Enables loop operation */

/* SCI Control Register 2 Bit-Field Definitions */

#define SCI_CR2_SBK               (1 << 0)  /* Bit 0:  Send Break */
#define SCI_CR2_RWU               (1 << 1)  /* Bit 1:  Receiver Wakeup */
#define SCI_CR2_RE                (1 << 2)  /* Bit 2:  Receiver Enable */
#define SCI_CR2_TE                (1 << 3)  /* Bit 3:  Transmitter Enable */
#define SCI_CR2_ILIE              (1 << 4)  /* Bit 4:  Idle Line Interrupt Enable */
#define SCI_CR2_RIE               (1 << 5)  /* Bit 5:  Receiver Full Interrupt Enable Bit */
#define SCI_CR2_TCIE              (1 << 6)  /* Bit 6:  Transmission Complete Interrupt Enable */
#define SCI_CR2_TIE               (1 << 7)  /* Bit 7:  Transmitter Interrupt Enable */
#define SCI_CR2_ALLINTS           0xf0

/* SCI Status Register 1 Bit-Field Definitions */

#define SCI_SR1_PF                (1 << 0)  /* Bit 0:  Parity Error */
#define SCI_SR1_FE                (1 << 1)  /* Bit 1:  Framing Error */
#define SCI_SR1_NF                (1 << 2)  /* Bit 2:  Noise */
#define SCI_SR1_OR                (1 << 3)  /* Bit 3:  Overrun */
#define SCI_SR1_IDLE              (1 << 4)  /* Bit 4:  Idle Line */
#define SCI_SR1_RDRF              (1 << 5)  /* Bit 5:  Receive Data Register Full */
#define SCI_SR1_TC                (1 << 6)  /* Bit 6:  Transmit Complete */
#define SCI_SR1_TDRE              (1 << 7)  /* Bit 7:  Transmit Data Register Empty */

/* SCI Status Register 2 Bit-Field Definitions */

#define SCI_SR2_BRK13             (1 << 2)  /* Bit 2:  Break Transmit Character Length */
#define SCI_SR2_TXDIR             (1 << 1)  /* Bit 1:  Transmitter Pin Data Direction in Single-Wire */
#define SCI_SR2_RAF               (1 << 0)  /* Bit 0:  Receiver Active */

/* SCI Data Register High Bit-Field Definitions */

#define SCI_DRH_T8                (1 << 6)  /* Bit 6:  Transmit Bit 8 */
#define SCI_DRH_R8                (1 << 7)  /* Bit 7:  Received Bit 8 */

/* SCI Data Register Low Bit-Field Definitions */

/* Receive/Transmit bits 0-7 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_HC_SRC_M9S12_M9S12_SCI_H */
