/****************************************************************************
 * arch/arm/src/at32/hardware/at32_can.h
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

#ifndef __ARCH_ARM_SRC_AT32_HARDWARE_AT32_CAN_H
#define __ARCH_ARM_SRC_AT32_HARDWARE_AT32_CAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* 3 TX mailboxes */

#define CAN_TXMBOX1 0
#define CAN_TXMBOX2 1
#define CAN_TXMBOX3 2

/* 2 RX mailboxes */

#define CAN_RXMBOX1 0
#define CAN_RXMBOX2 1

/* Number of filters depends on silicon */

#if defined(CONFIG_AT32_AT32F43XX)
#  define CAN_NFILTERS 28
#else
#  define CAN_NFILTERS 14
#endif

/* Register Offsets *********************************************************/

/* CAN control and status registers */

#define AT32_CAN_MCTRL_OFFSET               (0x000) /* CAN master control register */
#define AT32_CAN_MSTS_OFFSET                (0x004) /* CAN master status register */
#define AT32_CAN_TSTS_OFFSET                (0x008) /* CAN transmit status register */

#define AT32_CAN_RF_OFFSET(x)               (0x00c+((x)<<2))
#define AT32_CAN_RF0_OFFSET                 (0x00c) /* CAN receive FIFO 0 register */
#define AT32_CAN_RF1_OFFSET                 (0x010) /* CAN receive FIFO 1 register */

#define AT32_CAN_INTEN_OFFSET               (0x014) /* CAN interrupt enable register */
#define AT32_CAN_ESTS_OFFSET                (0x018) /* CAN error status register */
#define AT32_CAN_BTMG_OFFSET                (0x01c) /* CAN bit timing register */

/* CAN mailbox registers (3 TX and 2 RX) */

#define AT32_CAN_TMI_OFFSET(x)              (0x180+((x)<<4))
#define AT32_CAN_TMI0_FFSET                 (0x180) /* TX mailbox identifier register 0 */
#define AT32_CAN_TMI1_FFSET                 (0x190) /* TX mailbox identifier register 1 */
#define AT32_CAN_TMI2_FFSET                 (0x1a0) /* TX mailbox identifier register 2 */

#define AT32_CAN_TMC_OFFSET(x)              (0x184+((x)<<4))
#define AT32_CAN_TMC0_OFFSET                (0x184) /* Mailbox data length control and time stamp register 0 */
#define AT32_CAN_TMC1_OFFSET                (0x194) /* Mailbox data length control and time stamp register 1 */
#define AT32_CAN_TMC2_OFFSET                (0x1a4) /* Mailbox data length control and time stamp register 2 */

#define AT32_CAN_TMDTL_OFFSET(x)            (0x188+((x)<<4))
#define AT32_CAN_TMDTL0_OFFSET              (0x188) /* Mailbox data low register 0 */
#define AT32_CAN_TMDTL1_OFFSET              (0x198) /* Mailbox data low register 1 */
#define AT32_CAN_TMDTL2_OFFSET              (0x1a8) /* Mailbox data low register 2 */

#define AT32_CAN_TMDTH_OFFSET(x)            (0x18c+((x)<<4))
#define AT32_CAN_TMDTH0_OFFSET              (0x18c) /* Mailbox data high register 0 */
#define AT32_CAN_TMDTH1_OFFSET              (0x19c) /* Mailbox data high register 1 */
#define AT32_CAN_TMDTH2_OFFSET              (0x1ac) /* Mailbox data high register 2 */

#define AT32_CAN_RFI_OFFSET(x)              (0x1b0+((x)<<4))
#define AT32_CAN_RFI0_OFFSET                (0x1b0) /* Rx FIFO mailbox identifier register 0 */
#define AT32_CAN_RFI1_OFFSET                (0x1c0) /* Rx FIFO mailbox identifier register 1 */

#define AT32_CAN_RFC_OFFSET(x)              (0x1b4+((x)<<4))
#define AT32_CAN_RFC0_OFFSET                (0x1b4) /* Rx FIFO mailbox data length control and time stamp register 0 */
#define AT32_CAN_RFC1_OFFSET                (0x1c4) /* Rx FIFO mailbox data length control and time stamp register 1 */

#define AT32_CAN_RFDTL_OFFSET(x)            (0x1b8+((x)<<4))
#define AT32_CAN_RFDTL0_OFFSET              (0x1b8) /* Receive FIFO mailbox data low register 0 */
#define AT32_CAN_RFDTL1_OFFSET              (0x1c8) /* Receive FIFO mailbox data low register 1 */

#define AT32_CAN_RFDTH_OFFSET(x)            (0x1bc+((x)<<4))
#define AT32_CAN_RFDTH0_OFFSET              (0x1bc) /* Receive FIFO mailbox data high register 0 */
#define AT32_CAN_RFDTH1_OFFSET              (0x1cc) /* Receive FIFO mailbox data high register 1 */

/* CAN filter registers */

#define AT32_CAN_FCTRL_OFFSET               (0x200) /* CAN filter master register */
#define AT32_CAN_FMCFG_OFFSET               (0x204) /* CAN filter mode register */
#define AT32_CAN_FSCFG_OFFSET               (0x20c) /* CAN filter scale register */
#define AT32_CAN_FRF_OFFSET                 (0x214) /* CAN filter FIFO assignment register */
#define AT32_CAN_FACFG_OFFSET               (0x21c) /* CAN filter activation register */

/* There are 14 or 28 filter banks (depending) on the device.
 * Each filter bank is composed of two 32-bit registers, CAN_FiR:
 *  FB0F1 Offset 0x240
 *  FB0F2 Offset 0x244
 *  FB1F1 Offset 0x248
 *  FB1F2 Offset 0x24c
 *  ...
 */

#define AT32_CAN_FBF_OFFSET(f,i)            (0x240+((f)<<3)+(((i)-1)<<2))

/* Register Addresses *******************************************************/

#if AT32_NCAN > 0
#  define AT32_CAN1_MCTRL                   (AT32_CAN1_BASE+AT32_CAN_MCTRL_OFFSET)
#  define AT32_CAN1_MSTS                    (AT32_CAN1_BASE+AT32_CAN_MSTS_OFFSET)
#  define AT32_CAN1_TSTS                    (AT32_CAN1_BASE+AT32_CAN_TSTS_OFFSET)
#  define AT32_CAN1_RF0                     (AT32_CAN1_BASE+AT32_CAN_RF0_OFFSET)
#  define AT32_CAN1_RF1                     (AT32_CAN1_BASE+AT32_CAN_RF1_OFFSET)
#  define AT32_CAN1_INTEN                   (AT32_CAN1_BASE+AT32_CAN_INTEN_OFFSET)
#  define AT32_CAN1_ESTS                    (AT32_CAN1_BASE+AT32_CAN_ESTS_OFFSET)
#  define AT32_CAN1_BTMG                    (AT32_CAN1_BASE+AT32_CAN_BTMG_OFFSET)
#  define AT32_CAN1_TMI(x)                  (AT32_CAN1_BASE+AT32_CAN_TMI_OFFSET(x))
#  define AT32_CAN1_TMI0                    (AT32_CAN1_BASE+AT32_CAN_TMI0_FFSET)
#  define AT32_CAN1_TMI1                    (AT32_CAN1_BASE+AT32_CAN_TMI1_FFSET)
#  define AT32_CAN1_TMI2                    (AT32_CAN1_BASE+AT32_CAN_TMI2_FFSET)
#  define AT32_CAN1_TMC(x)                  (AT32_CAN1_BASE+AT32_CAN_TMC_OFFSET(x))
#  define AT32_CAN1_TMC0                    (AT32_CAN1_BASE+AT32_CAN_TMC0_OFFSET)
#  define AT32_CAN1_TMC1                    (AT32_CAN1_BASE+AT32_CAN_TMC1_OFFSET)
#  define AT32_CAN1_TMC2                    (AT32_CAN1_BASE+AT32_CAN_TMC2_OFFSET)
#  define AT32_CAN1_TMDTL(x)                (AT32_CAN1_BASE+AT32_CAN_TMDTL_OFFSET(x))
#  define AT32_CAN1_TMDTL0                  (AT32_CAN1_BASE+AT32_CAN_TMDTL0_OFFSET)
#  define AT32_CAN1_TMDTL1                  (AT32_CAN1_BASE+AT32_CAN_TMDTL1_OFFSET)
#  define AT32_CAN1_TMDTL2                  (AT32_CAN1_BASE+AT32_CAN_TMDTL2_OFFSET)
#  define AT32_CAN1_TMDTH(x)                (AT32_CAN1_BASE+AT32_CAN_TMDTH_OFFSET(x))
#  define AT32_CAN1_TMDTH0                  (AT32_CAN1_BASE+AT32_CAN_TMDTH0_OFFSET)
#  define AT32_CAN1_TMDTH1                  (AT32_CAN1_BASE+AT32_CAN_TMDTH1_OFFSET)
#  define AT32_CAN1_TMDTH2                  (AT32_CAN1_BASE+AT32_CAN_TMDTH2_OFFSET)
#  define AT32_CAN1_RFI(x)                  (AT32_CAN1_BASE+AT32_CAN_RFI_OFFSET(x))
#  define AT32_CAN1_RFI0                    (AT32_CAN1_BASE+AT32_CAN_RFI0_OFFSET)
#  define AT32_CAN1_RFI1                    (AT32_CAN1_BASE+AT32_CAN_RFI1_OFFSET)
#  define AT32_CAN1_RFC(x)                  (AT32_CAN1_BASE+AT32_CAN_RFC_OFFSET(x))
#  define AT32_CAN1_RFC0                    (AT32_CAN1_BASE+AT32_CAN_RFC0_OFFSET)
#  define AT32_CAN1_RFC1                    (AT32_CAN1_BASE+AT32_CAN_RFC1_OFFSET)
#  define AT32_CAN1_RFDTL(x)                (AT32_CAN1_BASE+AT32_CAN_RFDTL_OFFSET(x))
#  define AT32_CAN1_RFDTL0                  (AT32_CAN1_BASE+AT32_CAN_RFDTL0_OFFSET)
#  define AT32_CAN1_RFDTL1                  (AT32_CAN1_BASE+AT32_CAN_RFDTL1_OFFSET)
#  define AT32_CAN1_RFDTH(x)                (AT32_CAN1_BASE+AT32_CAN_RFDTH_OFFSET(x))
#  define AT32_CAN1_RFDTH0                  (AT32_CAN1_BASE+AT32_CAN_RFDTH0_OFFSET)
#  define AT32_CAN1_RFDTH1                  (AT32_CAN1_BASE+AT32_CAN_RFDTH1_OFFSET)
#  define AT32_CAN1_FCTRL                   (AT32_CAN1_BASE+AT32_CAN_FCTRL_OFFSET)
#  define AT32_CAN1_FMCFG                   (AT32_CAN1_BASE+AT32_CAN_FMCFG_OFFSET)
#  define AT32_CAN1_FSCFG                   (AT32_CAN1_BASE+AT32_CAN_FSCFG_OFFSET)
#  define AT32_CAN1_FRF                     (AT32_CAN1_BASE+AT32_CAN_FRF_OFFSET)
#  define AT32_CAN1_FACFG                   (AT32_CAN1_BASE+AT32_CAN_FACFG_OFFSET)
#  define AT32_CAN1_FIR(b,i)                (AT32_CAN1_BASE+AT32_CAN_FIR_OFFSET(b,i))
#endif

#if AT32_NCAN > 1
#  define AT32_CAN2_MCTRL                   (AT32_CAN2_BASE+AT32_CAN_MCTRL_OFFSET)
#  define AT32_CAN2_MSTS                    (AT32_CAN2_BASE+AT32_CAN_MSTS_OFFSET)
#  define AT32_CAN2_TSTS                    (AT32_CAN2_BASE+AT32_CAN_TSTS_OFFSET)
#  define AT32_CAN2_RF0                     (AT32_CAN2_BASE+AT32_CAN_RF0_OFFSET)
#  define AT32_CAN2_RF1                     (AT32_CAN2_BASE+AT32_CAN_RF1_OFFSET)
#  define AT32_CAN2_INTEN                   (AT32_CAN2_BASE+AT32_CAN_INTEN_OFFSET)
#  define AT32_CAN2_ESTS                    (AT32_CAN2_BASE+AT32_CAN_ESTS_OFFSET)
#  define AT32_CAN2_BTMG                    (AT32_CAN2_BASE+AT32_CAN_BTMG_OFFSET)
#  define AT32_CAN2_TMI(x)                  (AT32_CAN2_BASE+AT32_CAN_TMI_OFFSET(x))
#  define AT32_CAN2_TMI0                    (AT32_CAN2_BASE+AT32_CAN_TMI0_FFSET)
#  define AT32_CAN2_TMI1                    (AT32_CAN2_BASE+AT32_CAN_TMI1_FFSET)
#  define AT32_CAN2_TMI2                    (AT32_CAN2_BASE+AT32_CAN_TMI2_FFSET)
#  define AT32_CAN2_TMC(x)                  (AT32_CAN2_BASE+AT32_CAN_TMC_OFFSET(x))
#  define AT32_CAN2_TMC0                    (AT32_CAN2_BASE+AT32_CAN_TMC0_OFFSET)
#  define AT32_CAN2_TMC1                    (AT32_CAN2_BASE+AT32_CAN_TMC1_OFFSET)
#  define AT32_CAN2_TMC2                    (AT32_CAN2_BASE+AT32_CAN_TMC2_OFFSET)
#  define AT32_CAN2_TMDTL(x)                (AT32_CAN2_BASE+AT32_CAN_TMDTL_OFFSET(x))
#  define AT32_CAN2_TMDTL0                  (AT32_CAN2_BASE+AT32_CAN_TMDTL0_OFFSET)
#  define AT32_CAN2_TMDTL1                  (AT32_CAN2_BASE+AT32_CAN_TMDTL1_OFFSET)
#  define AT32_CAN2_TMDTL2                  (AT32_CAN2_BASE+AT32_CAN_TMDTL2_OFFSET)
#  define AT32_CAN2_TMDTH(x)                (AT32_CAN2_BASE+AT32_CAN_TMDTH_OFFSET(x))
#  define AT32_CAN2_TMDTH0                  (AT32_CAN2_BASE+AT32_CAN_TMDTH0_OFFSET)
#  define AT32_CAN2_TMDTH1                  (AT32_CAN2_BASE+AT32_CAN_TMDTH1_OFFSET)
#  define AT32_CAN2_TMDTH2                  (AT32_CAN2_BASE+AT32_CAN_TMDTH2_OFFSET)
#  define AT32_CAN2_RFI(x)                  (AT32_CAN2_BASE+AT32_CAN_RFI_OFFSET(x))
#  define AT32_CAN2_RFI0                    (AT32_CAN2_BASE+AT32_CAN_RFI0_OFFSET)
#  define AT32_CAN2_RFI1                    (AT32_CAN2_BASE+AT32_CAN_RFI1_OFFSET)
#  define AT32_CAN2_RFC(x)                  (AT32_CAN2_BASE+AT32_CAN_RFC_OFFSET(x))
#  define AT32_CAN2_RFC0                    (AT32_CAN2_BASE+AT32_CAN_RFC0_OFFSET)
#  define AT32_CAN2_RFC1                    (AT32_CAN2_BASE+AT32_CAN_RFC1_OFFSET)
#  define AT32_CAN2_RFDTL(x)                (AT32_CAN2_BASE+AT32_CAN_RFDTL_OFFSET(x))
#  define AT32_CAN2_RFDTL0                  (AT32_CAN2_BASE+AT32_CAN_RFDTL0_OFFSET)
#  define AT32_CAN2_RFDTL1                  (AT32_CAN2_BASE+AT32_CAN_RFDTL1_OFFSET)
#  define AT32_CAN2_RFDTH(x)                (AT32_CAN2_BASE+AT32_CAN_RFDTH_OFFSET(x))
#  define AT32_CAN2_RFDTH0                  (AT32_CAN2_BASE+AT32_CAN_RFDTH0_OFFSET)
#  define AT32_CAN2_RFDTH1                  (AT32_CAN2_BASE+AT32_CAN_RFDTH1_OFFSET)
#  define AT32_CAN2_FCTRL                   (AT32_CAN2_BASE+AT32_CAN_FCTRL_OFFSET)
#  define AT32_CAN2_FMCFG                   (AT32_CAN2_BASE+AT32_CAN_FMCFG_OFFSET)
#  define AT32_CAN2_FSCFG                   (AT32_CAN2_BASE+AT32_CAN_FSCFG_OFFSET)
#  define AT32_CAN2_FRF                     (AT32_CAN2_BASE+AT32_CAN_FRF_OFFSET)
#  define AT32_CAN2_FACFG                   (AT32_CAN2_BASE+AT32_CAN_FACFG_OFFSET)
#  define AT32_CAN2_FIR(b,i)                (AT32_CAN2_BASE+AT32_CAN_FIR_OFFSET(b,i))
#endif

/* Register Bitfield Definitions ********************************************/

/* CAN master control register */

#define CAN_MCTRL_FZEN                      (1 << 0)  /* Freeze mode enable */
#define CAN_MCTRL_DZEN                      (1 << 1)  /* Doze mode enable */
#define CAN_MCTRL_MMSSR                     (1 << 2)  /* Multiple message sending sequence rule */
#define CAN_MCTRL_MDRSEL                    (1 << 3)  /* Message discarding rule select when overflow */
#define CAN_MCTRL_PRSFEN                    (1 << 4)  /* Prohibit retransmission when sending fails enable */
#define CAN_MCTRL_AEDEN                     (1 << 5)  /* Automatic exit doze mode enable */
#define CAN_MCTRL_AEBOEN                    (1 << 6)  /* Automatic exit bus-off enable */
#define CAN_MCTRL_TTCEN                     (1 << 7)  /* Time triggered communication mode enable */
#define CAN_MCTRL_SPRST                     (1 << 15) /* Software partial reset */
#define CAN_MCTRL_PTD                       (1 << 16) /* Prohibit trans when debug */

/* CAN master status register */

#define  CAN_MSTS_FZC                       (1 << 0)  /* Freeze mode confirm */
#define  CAN_MSTS_DZC                       (1 << 1)  /* Doze mode confirm */
#define  CAN_MSTS_EOIF                      (1 << 2)  /* Error occur Interrupt flag */
#define  CAN_MSTS_QDZIF                     (1 << 3)  /* Quit doze mode interrupt flag */
#define  CAN_MSTS_EDZIF                     (1 << 4)  /* Enter doze mode interrupt flag */
#define  CAN_MSTS_CUSS                      (1 << 8)  /* Currently sending status */
#define  CAN_MSTS_CURS                      (1 << 9)  /* Currently receiving status */
#define  CAN_MSTS_LSAMPRX                   (1 << 10) /* Last sample level of RX pin */
#define  CAN_MSTS_REALRX                    (1 << 11) /* Real time level of RX pin */

/* CAN transmit status register */

#define CAN_TSTS_TM0TCF                     (1 << 0)  /* Transmit mailbox 0 transmission completed flag */
#define CAN_TSTS_TM0TSF                     (1 << 1)  /* Transmit mailbox 0 transmission success flag */
#define CAN_TSTS_TM0ALF                     (1 << 2)  /* Transmit mailbox 0 arbitration lost flag */
#define CAN_TSTS_TM0TEF                     (1 << 3)  /* Transmit mailbox 0 transmission error flag */
#define CAN_TSTS_TM0CT                      (1 << 7)  /* Transmit mailbox 0 cancel transmit */
#define CAN_TSTS_TM1TCF                     (1 << 8)  /* Transmit mailbox 1 transmission completed flag */
#define CAN_TSTS_TM1TSF                     (1 << 9)  /* Transmit mailbox 1 transmission success flag */
#define CAN_TSTS_TM1ALF                     (1 << 10) /* Transmit mailbox 1 arbitration lost flag */
#define CAN_TSTS_TM1TEF                     (1 << 11) /* Transmit mailbox 1 transmission error flag */
#define CAN_TSTS_TM1CT                      (1 << 15) /* Transmit mailbox 1 cancel transmit */
#define CAN_TSTS_TM2TCF                     (1 << 16) /* transmit mailbox 2 transmission completed flag */
#define CAN_TSTS_TM2TSF                     (1 << 17) /* Transmit mailbox 2 transmission success flag */
#define CAN_TSTS_TM2ALF                     (1 << 18) /* Transmit mailbox 2 arbitration lost flag */
#define CAN_TSTS_TM2TEF                     (1 << 19) /* Transmit mailbox 2 transmission error flag */
#define CAN_TSTS_TM2CT                      (1 << 23) /* Transmit mailbox 2 cancel transmit */

#define CAN_TSTS_TMNR_SHIFT                 (24) /* Transmit Mailbox number record */
#define CAN_TSTS_TMNR_MASK                  (3 << CAN_TSTS_TMNR_SHIFT)

#define CAN_TSTS_TM0EF                      (1 << 26) /* Transmit mailbox 0 empty flag */
#define CAN_TSTS_TM1EF                      (1 << 27) /* Transmit mailbox 1 empty flag */
#define CAN_TSTS_TM2EF                      (1 << 28) /* Transmit mailbox 2 empty flag */
#define CAN_TSTS_TM0LPF                     (1 << 29) /* Transmit mailbox 0 lowest priority flag */
#define CAN_TSTS_TM1LPF                     (1 << 30) /* Transmit mailbox 1 lowest priority flag */
#define CAN_TSTS_TM2LPF                     (1 << 31) /* Transmit mailbox 2 lowest priority flag */

/* CAN receive FIFO 0/1 registers */

#define CAN_RF_RFMN_SHIFT                   (0) /* Receive FIFO message num */
#define CAN_RF_RFMN_MASK                    (3 << CAN_RF_RFMN_SHIFT)

#define CAN_RF_RFFF                         (1 << 3) /* Receive FIFO full flag */
#define CAN_RF_RFOF                         (1 << 4) /* Receive FIFO overflow flag */
#define CAN_RF_RFR                          (1 << 5) /* Receive FIFO release */

/* CAN interrupt enable register */

#define CAN_INTEN_TCIEN                     (1 << 0)  /* Transmit mailbox empty interrupt enable */
#define CAN_INTEN_RF0MIEN                   (1 << 1)  /* FIFO 0 receive message interrupt enable */
#define CAN_INTEN_RF0FIEN                   (1 << 2)  /* Receive FIFO 0 full interrupt enable */
#define CAN_INTEN_RF0OIEN                   (1 << 3)  /* Receive FIFO 0 overflow interrupt enable */
#define CAN_INTEN_RF1MIEN                   (1 << 4)  /* FIFO 1 receive message interrupt enable */
#define CAN_INTEN_RF1FIEN                   (1 << 5)  /* Receive FIFO 1 full interrupt enable */
#define CAN_INTEN_RF1OIEN                   (1 << 6)  /* Receive FIFO 1 overflow interrupt enable */
#define CAN_INTEN_EAIEN                     (1 << 8)  /* Error active interrupt enable */
#define CAN_INTEN_EPIEN                     (1 << 9)  /* Error passive interrupt enable */
#define CAN_INTEN_BOIEN                     (1 << 10) /* Bus-off interrupt enable */
#define CAN_INTEN_ETRIEN                    (1 << 11) /* Error type record interrupt enable */
#define CAN_INTEN_EOIEN                     (1 << 15) /* Error occur interrupt enable */
#define CAN_INTEN_QDZIEN                    (1 << 16) /* Quit doze mode interrupt enable */
#define CAN_INTEN_EDZIEN                    (1 << 17) /* Enter doze mode interrupt enable */

/* CAN error status register */

#define CAN_ESTS_EAF                        (1 << 0) /* Error active flag */
#define CAN_ESTS_EPF                        (1 << 0) /* Error passive flag */
#define CAN_ESTS_BOF                        (1 << 0) /* Bus-off flag */

#define CAN_ESTS_ETR_SHIFT                  (4)                       /* Error type record */
#define CAN_ESTS_ETR_MASK                   (7 << CAN_ESTS_ETR_SHIFT)
#define CAN_ESTS_ETR_NONE                   (0 << CAN_ESTS_ETR_SHIFT) /* No error */
#define CAN_ESTS_ETR_STUFF                  (1 << CAN_ESTS_ETR_SHIFT) /* Stuff error */
#define CAN_ESTS_ETR_FORM                   (2 << CAN_ESTS_ETR_SHIFT) /* Form error */
#define CAN_ESTS_ETR_ACK                    (3 << CAN_ESTS_ETR_SHIFT) /* Ack error */
#define CAN_ESTS_ETR_BREC                   (4 << CAN_ESTS_ETR_SHIFT) /* Bit recessive error */
#define CAN_ESTS_ETR_BDOM                   (5 << CAN_ESTS_ETR_SHIFT) /* Bit domainant error */
#define CAN_ESTS_ETR_CRC                    (6 << CAN_ESTS_ETR_SHIFT) /* CRC error */
#define CAN_ESTS_ETR_SOFT                   (7 << CAN_ESTS_ETR_SHIFT) /* Set by soft */

#define CAN_ESTS_TEC_SHIFT                  (16) /* Transmit error counter */
#define CAN_ESTS_TEC_MASK                   (0xff << CAN_ESTS_TEC_SHIFT)

#define CAN_ESTS_REC_SHIFT                  (24) /* Receive error counter */
#define CAN_ESTS_REC_MASK                   (0xff << CAN_ESTS_REC_SHIFT)

/* CAN bit timing register */

#define CAN_BTMG_BRDIV_SHIFT                (0) /* Baud rate division */
#define CAN_BTMG_BRDIV_MASK                 (0xfff << CAN_BTMG_BRDIV_SHIFT)

#define CAN_BTMG_BTS1_SHIFT                 (16) /* Bit time segment 1 */
#define CAN_BTMG_BTS1_MASK                  (0xf << CAN_BTMG_BTS1_SHIFT)

#define CAN_BTMG_BTS2_SHIFT                 (20) /* Bit time segment 2 */
#define CAN_BTMG_BTS2_MASK                  (7 << CAN_BTMG_BTS2_SHIFT)

#define CAN_BTMG_RSAW_SHIFT                 (24) /* Resynchronization adjust width */
#define CAN_BTMG_RSAW_MASK                  (3 << CAN_BTMG_RSAW_SHIFT)

#define CAN_BTMG_LBEN                       (1 << 30) /* Loop back mode */
#define CAN_BTMG_LOEN                       (1 << 31) /* Listen-Only mode */

#define CAN_BTR_BRP_MAX           (1024)    /* Maximum BTR value (without decrement) */
#define CAN_BTR_TSEG1_MAX         (16)      /* Maximum TSEG1 value (without decrement) */
#define CAN_BTR_TSEG2_MAX         (8)       /* Maximum TSEG2 value (without decrement) */

/* TX mailbox identifier register */

#define CAN_TMI_TMSR                        (1 << 0) /* transmit mailbox send request */
#define CAN_TMI_TMFRSEL                     (1 << 1) /* Transmit mailbox frame type select */
#define CAN_TMI_TMIDSEL                     (1 << 2) /* Transmit mailbox identifier type select */

#define CAN_TMI_TMEID_SHIFT                 (3) /* Ttransmit mailbox extended identifier */
#define CAN_TMI_TMEID_MASK                  (0x1fffffff << CAN_TMI_TMEID_SHIFT)

#define CAN_TMI_TMSID_TMEID_SHIFT           (21) /* Transmit mailbox standard identifier or extended identifier high bytes */
#define CAN_TMI_TMSID_TMEID_MASK            (0x7ff << CAN_TMI_TMSID_TMEID_SHIFT)

/* Mailbox data length control and time stamp register */

#define CAN_TMC_TMDTBL_SHIFT                (0) /* Transmit mailbox data byte length */
#define CAN_TMC_TMDTBL_MASK                 (15 << CAN_TMC_TMDTBL_SHIFT)

#define CAN_TMC_TMTSTEN                     (1 << 8) /* Transmit mailbox time stamp transmit enable */

#define CAN_TMC_TMTS_SHIFT                  (16) /* Transmit mailbox time stamp */
#define CAN_TMC_TMTS_MASK                   (0xffff << CAN_TMC_TMTS_SHIFT)

/* Mailbox data low register */

#define CAN_TMDTL_TMDT0_SHIFT               (0) /* Transmit mailbox data byte 0 */
#define CAN_TMDTL_TMDT0_MASK                (0xff << CAN_TMDTL_TMDT0_SHIFT)
#define CAN_TMDTL_TMDT1_SHIFT               (8) /* Transmit mailbox data byte 1 */
#define CAN_TMDTL_TMDT1_MASK                (0xff << CAN_TMDTL_TMDT1_SHIFT)
#define CAN_TMDTL_TMDT2_SHIFT               (16) /* Transmit mailbox data byte 2 */
#define CAN_TMDTL_TMDT2_MASK                (0xff << CAN_TMDTL_TMDT2_SHIFT)
#define CAN_TMDTL_TMDT3_SHIFT               (24) /* Transmit mailbox data byte 3 */
#define CAN_TMDTL_TMDT3_MASK                (0xff << CAN_TMDTL_TMDT3_SHIFT)

/* Mailbox data high register */

#define CAN_TMDTH_TMDT4_SHIFT               (0) /* Transmit mailbox data byte 4 */
#define CAN_TMDTH_TMDT4_MASK                (0xff << CAN_TMDTH_TMDT4_SHIFT)
#define CAN_TMDTH_TMDT5_SHIFT               (8) /* Transmit mailbox data byte 5 */
#define CAN_TMDTH_TMDT5_MASK                (0xff << CAN_TMDTH_TMDT5_SHIFT)
#define CAN_TMDTH_TMDT6_SHIFT               (16) /* Transmit mailbox data byte 6 */
#define CAN_TMDTH_TMDT6_MASK                (0xff << CAN_TMDTH_TMDT6_SHIFT)
#define CAN_TMDTH_TMDT7_SHIFT               (24) /* Transmit mailbox data byte 7 */
#define CAN_TMDTH_TMDT7_MASK                (0xff << CAN_TMDTH_TMDT7_SHIFT)

/* Rx FIFO mailbox identifier register */

#define CAN_RDI_RFFRI                       (1 << 1) /* Receive FIFO frame type indication */
#define CAN_RDI_RFIDI                       (1 << 2) /* Receive FIFO identifier type indication */

#define CAN_RDI_RFEID_SHIFT                 (3) /* Receive FIFO extended identifier */
#define CAN_RDI_RFEID_MASK                  (0x3fff << CAN_RDI_RFEID_SHIFT)

#define CAN_RDI_RFSID_RFEID_SHIFT           (21) /* Receive FIFO standard identifier or receive FIFO extended identifier */
#define CAN_RDI_RFSID_RFEID_MASK            (0x7ff << CAN_RDI_RFSID_RFEID_SHIFT)

/* Receive FIFO mailbox data length control and time stamp register */

#define CAN_RFC_RFDTL_SHIFT                 (0) /* Receive FIFO data length */
#define CAN_RFC_RFDTL_MASK                  (15 << CAN_RFC_RFDTL_SHIFT)

#define CAN_RFC_RFFMN_SHIFT                 (8) /* Receive FIFO filter match number */
#define CAN_RFC_RFFMN_MASK                  (0xff << CAN_RFC_RFFMN_SHIFT)

#define CAN_RFC_RFTS_SHIFT                  (16) /* Receive FIFO time stamp */
#define CAN_RFC_RFTS_MASK                   (0xffff << CAN_RFC_RFTS_SHIFT)

/* Receive FIFO mailbox data low register */

#define CAN_RFDTL_RFDT0_SHIFT               (0) /* Receive FIFO data byte 0 */
#define CAN_RFDTL_RFDT0_MASK                (0xff << CAN_RFDTL_RFDT0_SHIFT)
#define CAN_RFDTL_RFDT1_SHIFT               (8) /* Receive FIFO data byte 1 */
#define CAN_RFDTL_RFDT1_MASK                (0xff << CAN_RFDTL_RFDT1_SHIFT)
#define CAN_RFDTL_RFDT2_SHIFT               (16) /* Receive FIFO data byte 2 */
#define CAN_RFDTL_RFDT2_MASK                (0xff << CAN_RFDTL_RFDT2_SHIFT)
#define CAN_RFDTL_RFDT3_SHIFT               (24) /* Receive FIFO data byte 3 */
#define CAN_RFDTL_RFDT3_MASK                (0xff << CAN_RFDTL_RFDT3_SHIFT)

/* Receive FIFO mailbox data high register */

#define CAN_RFDTH_RFDT4_SHIFT               (0) /* Receive FIFO data byte 4 */
#define CAN_RFDTH_RFDT4_MASK                (0xff << CAN_RFDTH_RFDT4_SHIFT)
#define CAN_RFDTH_RFDT5_SHIFT               (8) /* Receive FIFO data byte 5 */
#define CAN_RFDTH_RFDT5_MASK                (0xff << CAN_RFDTH_RFDT5_SHIFT)
#define CAN_RFDTH_RFDT6_SHIFT               (16) /* Receive FIFO data byte 6 */
#define CAN_RFDTH_RFDT6_MASK                (0xff << CAN_RFDTH_RFDT6_SHIFT)
#define CAN_RFDTH_RFDT7_SHIFT               (24) /* Receive FIFO data byte 7 */
#define CAN_RFDTH_RFDT7_MASK                (0xff << CAN_RFDTH_RFDT7_SHIFT)

/* CAN filter master register */

#define CAN_FCTRL_FCS                       (1 << 0) /* Filters configure switch */

/* CAN filter mode register */

#define CAN_FMCFG_FMSEL_SHIFT               (0) /* Filter mode select,0:mask 1:list */
#define CAN_FMCFG_FMSEL_MASK                (0xfffffff << CAN_FMCFG_FMSEL_SHIFT)

/* CAN filter scale register */

#define CAN_FBWCFG_FBWSEL_SHIFT             (0) /* Filter bit width select,0:two 16 bits  1:32bits */
#define CAN_FBWCFG_FBWSEL_MASK              (0xfffffff << CAN_FBWCFG_FBWSEL_SHIFT)

/* CAN filter FIFO assignment register */

#define CAN_FRF_FRFSEL_SHIFT                (0) /* Filter relation FIFO select,0:FIFO0 1:FIFO1 */
#define CAN_FRF_FRFSEL_MASK                 (0xfffffff << CAN_FRF_FRFSEL_SHIFT)

/* CAN filter activation register */

#define CAN_FACFG_FAEN_SHIFT                (0) /* Filter active enable */
#define CAN_FACFG_FAEN_MASK                 (0xfffffff << CAN_FACFG_FAEN_SHIFT)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_AT32_HARDWARE_AT32_CAN_H */
