/****************************************************************************
 * arch/arm/src/tiva/hardware/tiva_can.h
 *
 *   Copyright (C) 2006-2020 Texas Instruments Incorporated.
 *                           All rights reserved.
 *   Author: Matthew Trescott <matthewtrescott@gmail.com>
 *
 * From the TivaWare Peripheral Driver Library, with minor changes for
 * clarity and style.
 *
 * The TivaWare sample code has a BSD compatible license that requires this
 * copyright notice:
 *
 * Copyright (c) 2005-2020 Texas Instruments Incorporated.
 * All rights reserved.
 * Software License Agreement
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * This is part of revision 2.2.0295 of the Tiva Peripheral Driver Library.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_TIVA_HARDWARE_TIVA_CAN_H
#define __ARCH_ARM_SRC_TIVA_HARDWARE_TIVA_CAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * The following are defines describing the CAN controller modules
 ****************************************************************************/

#define TIVA_CAN_NUM_IFACES        2            /* Number of sets of CANIF registers */

/* Note: driver code uses a 32-bit bitmask to represent the message objects */

#define TIVA_CAN_NUM_MBOXES      32             /* Number of slots in the message SRAM */

/****************************************************************************
 * The following are defines for the CAN register offsets.
 ****************************************************************************/

#define TIVA_CAN_OFFSET_CTL         0x00000000  /* CAN Control                  */
#define TIVA_CAN_OFFSET_STS         0x00000004  /* CAN Status                   */
#define TIVA_CAN_OFFSET_ERR         0x00000008  /* CAN Error Counter            */
#define TIVA_CAN_OFFSET_BIT         0x0000000C  /* CAN Bit Timing               */
#define TIVA_CAN_OFFSET_INT         0x00000010  /* CAN Interrupt                */
#define TIVA_CAN_OFFSET_TST         0x00000014  /* CAN Test                     */
#define TIVA_CAN_OFFSET_BRPE        0x00000018  /* CAN Baud Rate Prescaler      *
                                                 * Extension                    */

#define TIVA_CAN_OFFSET_IFACE1_BASE 0x00000020  /* CAN Interface 1 base         */
#define TIVA_CAN_OFFSET_IF1CRQ      0x00000020  /* CAN IF1 Command Request      */
#define TIVA_CAN_OFFSET_IF1CMSK     0x00000024  /* CAN IF1 Command Mask         */
#define TIVA_CAN_OFFSET_IF1MSK1     0x00000028  /* CAN IF1 Mask 1               */
#define TIVA_CAN_OFFSET_IF1MSK2     0x0000002C  /* CAN IF1 Mask 2               */
#define TIVA_CAN_OFFSET_IF1ARB1     0x00000030  /* CAN IF1 Arbitration 1        */
#define TIVA_CAN_OFFSET_IF1ARB2     0x00000034  /* CAN IF1 Arbitration 2        */
#define TIVA_CAN_OFFSET_IF1MCTL     0x00000038  /* CAN IF1 Message Control      */
#define TIVA_CAN_OFFSET_IF1DA1      0x0000003C  /* CAN IF1 Data A1              */
#define TIVA_CAN_OFFSET_IF1DA2      0x00000040  /* CAN IF1 Data A2              */
#define TIVA_CAN_OFFSET_IF1DB1      0x00000044  /* CAN IF1 Data B1              */
#define TIVA_CAN_OFFSET_IF1DB2      0x00000048  /* CAN IF1 Data B2              */
#define TIVA_CAN_OFFSET_IFACE2_BASE 0x00000080  /* CAN Interface 2 base         */
#define TIVA_CAN_OFFSET_IF2CRQ      0x00000080  /* CAN IF2 Command Request      */
#define TIVA_CAN_OFFSET_IF2CMSK     0x00000084  /* CAN IF2 Command Mask         */
#define TIVA_CAN_OFFSET_IF2MSK1     0x00000088  /* CAN IF2 Mask 1               */
#define TIVA_CAN_OFFSET_IF2MSK2     0x0000008C  /* CAN IF2 Mask 2               */
#define TIVA_CAN_OFFSET_IF2ARB1     0x00000090  /* CAN IF2 Arbitration 1        */
#define TIVA_CAN_OFFSET_IF2ARB2     0x00000094  /* CAN IF2 Arbitration 2        */
#define TIVA_CAN_OFFSET_IF2MCTL     0x00000098  /* CAN IF2 Message Control      */
#define TIVA_CAN_OFFSET_IF2DA1      0x0000009C  /* CAN IF2 Data A1              */
#define TIVA_CAN_OFFSET_IF2DA2      0x000000A0  /* CAN IF2 Data A2              */
#define TIVA_CAN_OFFSET_IF2DB1      0x000000A4  /* CAN IF2 Data B1              */
#define TIVA_CAN_OFFSET_IF2DB2      0x000000A8  /* CAN IF2 Data B2              */
#define TIVA_CAN_OFFSET_TXRQ1       0x00000100  /* CAN Transmission Request 1   */
#define TIVA_CAN_OFFSET_TXRQ2       0x00000104  /* CAN Transmission Request 2   */
#define TIVA_CAN_OFFSET_NWDA1       0x00000120  /* CAN New Data 1               */
#define TIVA_CAN_OFFSET_NWDA2       0x00000124  /* CAN New Data 2               */
#define TIVA_CAN_OFFSET_MSG1INT     0x00000140  /* CAN Message 1                *
                                                 * Interrupt Pending            */

#define TIVA_CAN_OFFSET_MSG2INT     0x00000144  /* CAN Message 2                *
                                                 * Interrupt Pending            */

#define TIVA_CAN_OFFSET_MSG1VAL     0x00000160  /* CAN Message 1 Valid          */
#define TIVA_CAN_OFFSET_MSG2VAL     0x00000164  /* CAN Message 2 Valid          */

/****************************************************************************
 * The following are defines for CAN registers
 ****************************************************************************/

#define TIVA_CAN_BASE(n)      (TIVA_CAN0_BASE + (n)*0x1000)

#define TIVA_CAN_CTL(n)       (TIVA_CAN_BASE(n)+TIVA_CAN_OFFSET_CTL)      /* CAN Control                  */
#define TIVA_CAN_STS(n)       (TIVA_CAN_BASE(n)+TIVA_CAN_OFFSET_STS)      /* CAN Status                   */
#define TIVA_CAN_ERR(n)       (TIVA_CAN_BASE(n)+TIVA_CAN_OFFSET_ERR)      /* CAN Error Counter            */
#define TIVA_CAN_BIT(n)       (TIVA_CAN_BASE(n)+TIVA_CAN_OFFSET_BIT)      /* CAN Bit Timing               */
#define TIVA_CAN_INT(n)       (TIVA_CAN_BASE(n)+TIVA_CAN_OFFSET_INT)      /* CAN Interrupt                */
#define TIVA_CAN_TST(n)       (TIVA_CAN_BASE(n)+TIVA_CAN_OFFSET_TST)      /* CAN Test                     */
#define TIVA_CAN_BRPE(n)      (TIVA_CAN_BASE(n)+TIVA_CAN_OFFSET_BRPE)     /* CAN Baud Rate Prescaler      *
                                                                           * Extension                    */

#define TIVA_CAN_IF1CRQ(n)    (TIVA_CAN_BASE(n)+TIVA_CAN_OFFSET_IF1CRQ)   /* CAN IF1 Command Request      */
#define TIVA_CAN_IF1CMSK(n)   (TIVA_CAN_BASE(n)+TIVA_CAN_OFFSET_IF1CMSK)  /* CAN IF1 Command Mask         */
#define TIVA_CAN_IF1MSK1(n)   (TIVA_CAN_BASE(n)+TIVA_CAN_OFFSET_IF1MSK1)  /* CAN IF1 Mask 1               */
#define TIVA_CAN_IF1MSK2(n)   (TIVA_CAN_BASE(n)+TIVA_CAN_OFFSET_IF1MSK2)  /* CAN IF1 Mask 2               */
#define TIVA_CAN_IF1ARB1(n)   (TIVA_CAN_BASE(n)+TIVA_CAN_OFFSET_IF1ARB1)  /* CAN IF1 Arbitration 1        */
#define TIVA_CAN_IF1ARB2(n)   (TIVA_CAN_BASE(n)+TIVA_CAN_OFFSET_IF1ARB2)  /* CAN IF1 Arbitration 2        */
#define TIVA_CAN_IF1MCTL(n)   (TIVA_CAN_BASE(n)+TIVA_CAN_OFFSET_IF1MCTL)  /* CAN IF1 Message Control      */
#define TIVA_CAN_IF1DA1(n)    (TIVA_CAN_BASE(n)+TIVA_CAN_OFFSET_IF1DA1)   /* CAN IF1 Data A1              */
#define TIVA_CAN_IF1DA2(n)    (TIVA_CAN_BASE(n)+TIVA_CAN_OFFSET_IF1DA2)   /* CAN IF1 Data A2              */
#define TIVA_CAN_IF1DB1(n)    (TIVA_CAN_BASE(n)+TIVA_CAN_OFFSET_IF1DB1)   /* CAN IF1 Data B1              */
#define TIVA_CAN_IF1DB2(n)    (TIVA_CAN_BASE(n)+TIVA_CAN_OFFSET_IF1DB2)   /* CAN IF1 Data B2              */
#define TIVA_CAN_IF2CRQ(n)    (TIVA_CAN_BASE(n)+TIVA_CAN_OFFSET_IF2CRQ)   /* CAN IF2 Command Request      */
#define TIVA_CAN_IF2CMSK(n)   (TIVA_CAN_BASE(n)+TIVA_CAN_OFFSET_IF2CMSK)  /* CAN IF2 Command Mask         */
#define TIVA_CAN_IF2MSK1(n)   (TIVA_CAN_BASE(n)+TIVA_CAN_OFFSET_IF2MSK1)  /* CAN IF2 Mask 1               */
#define TIVA_CAN_IF2MSK2(n)   (TIVA_CAN_BASE(n)+TIVA_CAN_OFFSET_IF2MSK2)  /* CAN IF2 Mask 2               */
#define TIVA_CAN_IF2ARB1(n)   (TIVA_CAN_BASE(n)+TIVA_CAN_OFFSET_IF2ARB1)  /* CAN IF2 Arbitration 1        */
#define TIVA_CAN_IF2ARB2(n)   (TIVA_CAN_BASE(n)+TIVA_CAN_OFFSET_IF2ARB2)  /* CAN IF2 Arbitration 2        */
#define TIVA_CAN_IF2MCTL(n)   (TIVA_CAN_BASE(n)+TIVA_CAN_OFFSET_IF2MCTL)  /* CAN IF2 Message Control      */
#define TIVA_CAN_IF2DA1(n)    (TIVA_CAN_BASE(n)+TIVA_CAN_OFFSET_IF2DA1)   /* CAN IF2 Data A1              */
#define TIVA_CAN_IF2DA2(n)    (TIVA_CAN_BASE(n)+TIVA_CAN_OFFSET_IF2DA2)   /* CAN IF2 Data A2              */
#define TIVA_CAN_IF2DB1(n)    (TIVA_CAN_BASE(n)+TIVA_CAN_OFFSET_IF2DB1)   /* CAN IF2 Data B1              */
#define TIVA_CAN_IF2DB2(n)    (TIVA_CAN_BASE(n)+TIVA_CAN_OFFSET_IF2DB2)   /* CAN IF2 Data B2              */
#define TIVA_CAN_TXRQ1(n)     (TIVA_CAN_BASE(n)+TIVA_CAN_OFFSET_TXRQ1)    /* CAN Transmission Request 1   */
#define TIVA_CAN_TXRQ2(n)     (TIVA_CAN_BASE(n)+TIVA_CAN_OFFSET_TXRQ2)    /* CAN Transmission Request 2   */
#define TIVA_CAN_NWDA1(n)     (TIVA_CAN_BASE(n)+TIVA_CAN_OFFSET_NWDA1)    /* CAN New Data 1               */
#define TIVA_CAN_NWDA2(n)     (TIVA_CAN_BASE(n)+TIVA_CAN_OFFSET_NWDA2)    /* CAN New Data 2               */
#define TIVA_CAN_MSG1INT(n)   (TIVA_CAN_BASE(n)+TIVA_CAN_OFFSET_MSG1INT)  /* CAN Message                  *
                                                                           * Interrupt Pending 2          */

#define TIVA_CAN_MSG2INT(n)   (TIVA_CAN_BASE(n)+TIVA_CAN_OFFSET_MSG2INT)  /* CAN Message                  *
                                                                           * Interrupt Pending 2          */

#define TIVA_CAN_MSG1VAL(n) (TIVA_CAN_BASE(n)+TIVA_CAN_OFFSET_MSG1VAL)    /* CAN Message 1 Valid          */
#define TIVA_CAN_MSG2VAL(n) (TIVA_CAN_BASE(n)+TIVA_CAN_OFFSET_MSG2VAL)    /* CAN Message 2 Valid          */

/****************************************************************************
 * The following are defines for CAN interface (CANIFn) register offsets.
 * Note that in these defines, the CAN interfaces are indexed from zero.
 ****************************************************************************/

#define TIVA_CANIF_OFFSET_CRQ   0x00000000
#define TIVA_CANIF_OFFSET_CMSK  0x00000004
#define TIVA_CANIF_OFFSET_MSK1  0x00000008
#define TIVA_CANIF_OFFSET_MSK2  0x0000000C
#define TIVA_CANIF_OFFSET_ARB1  0x00000010
#define TIVA_CANIF_OFFSET_ARB2  0x00000014
#define TIVA_CANIF_OFFSET_MCTL  0x00000018
#define TIVA_CANIF_OFFSET_DA1   0x0000001C
#define TIVA_CANIF_OFFSET_DA2   0x00000020
#define TIVA_CANIF_OFFSET_DB1   0x00000024
#define TIVA_CANIF_OFFSET_DB2   0x00000028

#define TIVA_CANIF_OFFSET_DATA(n) (TIVA_CANIF_OFFSET_DA1 + (n) * 0x4)

#define TIVA_CAN_OFFSET_IFACE(i) (TIVA_CAN_OFFSET_IFACE1_BASE + (i) * 0x60)
#define TIVA_CAN_IFACE_BASE(n,i) (TIVA_CAN_BASE(n)+TIVA_CAN_OFFSET_IFACE(i))

#define TIVA_CANIF_CRQ(n,i)   (TIVA_CAN_IFACE_BASE((n),(i))+TIVA_CANIF_OFFSET_CRQ)
#define TIVA_CANIF_CMSK(n,i)  (TIVA_CAN_IFACE_BASE((n),(i))+TIVA_CANIF_OFFSET_CMSK)
#define TIVA_CANIF_MSK1(n,i)  (TIVA_CAN_IFACE_BASE((n),(i))+TIVA_CANIF_OFFSET_MSK1)
#define TIVA_CANIF_MSK2(n,i)  (TIVA_CAN_IFACE_BASE((n),(i))+TIVA_CANIF_OFFSET_MSK2)
#define TIVA_CANIF_ARB1(n,i)  (TIVA_CAN_IFACE_BASE((n),(i))+TIVA_CANIF_OFFSET_ARB1)
#define TIVA_CANIF_ARB2(n,i)  (TIVA_CAN_IFACE_BASE((n),(i))+TIVA_CANIF_OFFSET_ARB2)
#define TIVA_CANIF_MCTL(n,i)  (TIVA_CAN_IFACE_BASE((n),(i))+TIVA_CANIF_OFFSET_MCTL)
#define TIVA_CANIF_DA1(n,i)   (TIVA_CAN_IFACE_BASE((n),(i))+TIVA_CANIF_OFFSET_DA1)
#define TIVA_CANIF_DA2(n,i)   (TIVA_CAN_IFACE_BASE((n),(i))+TIVA_CANIF_OFFSET_DA2)
#define TIVA_CANIF_DB1(n,i)   (TIVA_CAN_IFACE_BASE((n),(i))+TIVA_CANIF_OFFSET_DB1)
#define TIVA_CANIF_DB2(n,i)   (TIVA_CAN_IFACE_BASE((n),(i))+TIVA_CANIF_OFFSET_DB2)

/****************************************************************************
 * The following are defines for the bit fields in the CANCTL register.
 ****************************************************************************/

#define TIVA_CAN_CTL_TEST       0x00000080  /* Test Mode Enable                 */
#define TIVA_CAN_CTL_CCE        0x00000040  /* Configuration Change Enable      */
#define TIVA_CAN_CTL_DAR        0x00000020  /* Disable Automatic-Retransmission */
#define TIVA_CAN_CTL_EIE        0x00000008  /* Error Interrupt Enable           */
#define TIVA_CAN_CTL_SIE        0x00000004  /* Status Interrupt Enable          */
#define TIVA_CAN_CTL_IE         0x00000002  /* CAN Interrupt Enable             */
#define TIVA_CAN_CTL_INIT       0x00000001  /* Initialization                   */

/****************************************************************************
 * The following are defines for the bit fields in the CANSTS register
 ****************************************************************************/

#define TIVA_CAN_STS_BOFF           0x00000080  /* Bus-Off Status                   */
#define TIVA_CAN_STS_EWARN          0x00000040  /* Warning Status                   */
#define TIVA_CAN_STS_EPASS          0x00000020  /* Error Passive                    */
#define TIVA_CAN_STS_RXOK           0x00000010  /* Received a Message Successfully  */
#define TIVA_CAN_STS_TXOK           0x00000008  /* Transmitted a Message            */
                                                /* Successfully                     */
#define TIVA_CAN_STS_LEC_MASK       0x00000007  /* Last Error Code                  */
#define TIVA_CAN_STS_LEC_NONE       0x00000000  /* No Error                         */
#define TIVA_CAN_STS_LEC_STUFF      0x00000001  /* Stuff Error                      */
#define TIVA_CAN_STS_LEC_FORM       0x00000002  /* Format Error                     */
#define TIVA_CAN_STS_LEC_ACK        0x00000003  /* ACK Error                        */
#define TIVA_CAN_STS_LEC_BIT1       0x00000004  /* Bit 1 Error                      */
#define TIVA_CAN_STS_LEC_BIT0       0x00000005  /* Bit 0 Error                      */
#define TIVA_CAN_STS_LEC_CRC        0x00000006  /* CRC Error                        */
#define TIVA_CAN_STS_LEC_NOEVENT    0x00000007  /* No Event                         */

/****************************************************************************
 * The following are defines for the bit fields in the CANERR register.
 ****************************************************************************/

#define TIVA_CAN_ERR_RP             0x00008000  /* Received Error Passive           */
#define TIVA_CAN_ERR_REC_MASK       0x00007F00  /* Receive Error Counter            */
#define TIVA_CAN_ERR_TEC_MASK       0x000000FF  /* Transmit Error Counter           */
#define TIVA_CAN_ERR_REC_SHIFT      8
#define TIVA_CAN_ERR_TEC_SHIFT      0

/****************************************************************************
 * The following are defines for the bit fields in the CANBIT register.
 ****************************************************************************/

#define TIVA_CAN_BIT_TSEG2_MASK    0x00007000  /* Time Segment after Sample Point   */
#define TIVA_CAN_BIT_TSEG1_MASK    0x00000F00  /* Time Segment Before Sample Point  */
#define TIVA_CAN_BIT_SJW_MASK      0x000000C0  /* (Re)Synchronization Jump Width    */
#define TIVA_CAN_BIT_BRP_MASK      0x0000003F  /* Baud Rate Prescaler               */
#define TIVA_CAN_BIT_TSEG2_SHIFT    12
#define TIVA_CAN_BIT_TSEG1_SHIFT    8
#define TIVA_CAN_BIT_SJW_SHIFT      6
#define TIVA_CAN_BIT_BRP_SHIFT      0
#define TIVA_CAN_BIT_BRP_LENGTH     6          /* This is also the number of bits to shift the BRPE by */

#define TIVA_CAN_PRESCALER_MIN      1
#define TIVA_CAN_PRESCALER_MAX      1024
#define TIVA_CAN_TSEG1_MIN          1
#define TIVA_CAN_TSEG1_MAX          16
#define TIVA_CAN_TSEG2_MIN          1
#define TIVA_CAN_TSEG2_MAX          8
#define TIVA_CAN_SJW_MIN            1
#define TIVA_CAN_SJW_MAX            4

/****************************************************************************
 * The following are defines for the bit fields in the CANINT register.
 ****************************************************************************/

#define TIVA_CAN_INT_INTID_MASK     0x0000FFFF  /* Interrupt Identifier */
#define TIVA_CAN_INT_INTID_NONE     0x00000000  /* No interrupt pending */
#define TIVA_CAN_INT_INTID_STATUS   0x00008000  /* Status Interrupt     */

/****************************************************************************
 * The following are defines for the bit fields in the CANTST register.
 ****************************************************************************/

#define TIVA_CAN_TST_RX             0x00000080  /* Receive Observation  */
#define TIVA_CAN_TST_TX_MASK        0x00000060  /* Transmit Control     */
#define TIVA_CAN_TST_TX_CANCTL      0x00000000  /* CAN Module Control   */
#define TIVA_CAN_TST_TX_SAMPLE      0x00000020  /* Sample Point         */
#define TIVA_CAN_TST_TX_DOMINANT    0x00000040  /* Driven Low           */
#define TIVA_CAN_TST_TX_RECESSIVE   0x00000060  /* Driven High          */
#define TIVA_CAN_TST_LBACK          0x00000010  /* Loopback Mode        */
#define TIVA_CAN_TST_SILENT         0x00000008  /* Silent Mode          */
#define TIVA_CAN_TST_BASIC          0x00000004  /* Basic Mode           */

/****************************************************************************
 * The following are defines for the bit fields in the CANBRPE register.
 ****************************************************************************/

#define TIVA_CAN_BRPE_BRPE_MASK     0x0000000F  /* Baud Rate Prescaler Extension */
#define TIVA_CAN_BRPE_BRPE_SHIFT    0

/****************************************************************************
 * The following are defines for the bit fields in the CANIF1CRQ register.
 ****************************************************************************/
#define TIVA_CANIF_CRQ_BUSY         0x00008000  /* Busy Flag        */
#define TIVA_CANIF_CRQ_MNUM_MASK    0x0000003F  /* Message Number   */
#define TIVA_CANIF_CRQ_MNUM_SHIFT   0

/****************************************************************************
 * The following are defines for the bit fields in the CANIF1CMSK register.
 ****************************************************************************/

#define TIVA_CANIF_CMSK_WRNRD       0x00000080  /* Write, Not Read              */
#define TIVA_CANIF_CMSK_MASK        0x00000040  /* Access Mask Bits             */
#define TIVA_CANIF_CMSK_ARB         0x00000020  /* Access Arbitration Bits      */
#define TIVA_CANIF_CMSK_CONTROL     0x00000010  /* Access Control Bits          */
#define TIVA_CANIF_CMSK_CLRINTPND   0x00000008  /* Clear Interrupt Pending Bit  */
#define TIVA_CANIF_CMSK_NEWDAT      0x00000004  /* Access New Data              */
#define TIVA_CANIF_CMSK_TXRQST      0x00000004  /* Access Transmission Request  */
#define TIVA_CANIF_CMSK_DATAA       0x00000002  /* Access Data Byte 0 to 3      */
#define TIVA_CANIF_CMSK_DATAB       0x00000001  /* Access Data Byte 4 to 7      */

/****************************************************************************
 * The following are defines for the bit fields in the CANIF1MSK1 register.
 ****************************************************************************/

#define TIVA_CANIF_MSK1_IDMSK_EXT_MASK  0x0000FFFF  /* Identifier mask for lower 16 bits of extended ID */
#define TIVA_CANIF_MSK1_IDMSK_EXT_SHIFT 0

/****************************************************************************
 * The following are defines for the bit fields in the CANIF1MSK2 register.
 ****************************************************************************/

#define TIVA_CANIF_MSK2_MXTD                0x00008000  /* Mask Extended Identifier                                           */
#define TIVA_CANIF_MSK2_MDIR                0x00004000  /* Mask Message Direction                                             */
#define TIVA_CANIF_MSK2_IDMSK_EXT_MASK      0x00001FFF  /* Identifier Mask for upper 13 bits of extended ID                   */
#define TIVA_CANIF_MSK2_IDMSK_EXT_SHIFT     0           /* Shift LEFT by this number to place a value in this field           */
#define TIVA_CANIF_MSK2_IDMSK_EXT_PRESHIFT  16          /* Shift RIGHT by this number to get the chunk this register wants    */

#define TIVA_CANIF_MSK2_IDMSK_STD_MASK      0x00001FFC  /* MSK2 contains all 11 bits of a standard ID, but not aligned with 0 */
#define TIVA_CANIF_MSK2_IDMSK_STD_SHIFT     2           /* Shift LEFT by this number to place a value in this field           */

/****************************************************************************
 * The following are defines for the bit fields in the CANIF1ARB1 register.
 ****************************************************************************/

#define TIVA_CANIF_ARB1_ID_EXT_MASK     0x0000FFFF  /* Identifier for lower 16 bits of extended ID */
#define TIVA_CANIF_ARB1_ID_EXT_SHIFT    0

/****************************************************************************
 * The following are defines for the bit fields in the CANIF1ARB2 register.
 ****************************************************************************/

#define TIVA_CANIF_ARB2_MSGVAL          0x00008000  /* Message Valid                                                      */
#define TIVA_CANIF_ARB2_XTD             0x00004000  /* Extended Identifier                                                */
#define TIVA_CANIF_ARB2_DIR             0x00002000  /* Message Direction                                                  */
#define TIVA_CANIF_ARB2_ID_EXT_MASK     0x00001FFF  /* Message Identifier                                                 */
#define TIVA_CANIF_ARB2_ID_EXT_SHIFT    0           /* Shift LEFT by this number to place a value in this filed           */
#define TIVA_CANIF_ARB2_ID_EXT_PRESHIFT 16          /* Shift RIGHT by this number to get the chunk this register wants    */

#define TIVA_CANIF_ARB2_ID_STD_MASK     0x00001FFC  /* ARB2 contains all 11 bits of a standard ID, but not aligned with 0 */
#define TIVA_CANIF_ARB2_ID_STD_SHIFT    2           /* Shift LEFT by this number to place a value in this field           */

/****************************************************************************
 * The following are defines for the bit fields in the CANIF1MCTL register.
 ****************************************************************************/

#define TIVA_CANIF_MCTL_NEWDAT      0x00008000  /* New Data                     */
#define TIVA_CANIF_MCTL_MSGLST      0x00004000  /* Message Lost                 */
#define TIVA_CANIF_MCTL_INTPND      0x00002000  /* Interrupt Pending            */
#define TIVA_CANIF_MCTL_UMASK       0x00001000  /* Use Acceptance Mask          */
#define TIVA_CANIF_MCTL_TXIE        0x00000800  /* Transmit Interrupt Enable    */
#define TIVA_CANIF_MCTL_RXIE        0x00000400  /* Receive Interrupt Enable     */
#define TIVA_CANIF_MCTL_RMTEN       0x00000200  /* Remote Enable                */
#define TIVA_CANIF_MCTL_TXRQST      0x00000100  /* Transmit Request             */
#define TIVA_CANIF_MCTL_EOB         0x00000080  /* End of Buffer                */
#define TIVA_CANIF_MCTL_DLC_MASK    0x0000000F  /* Data Length Code             */
#define TIVA_CANIF_MCTL_DLC_SHIFT   0

/****************************************************************************
 * The following are defines for the bit fields in the CANIF1D(A,B)(1,2)
 * registers. H and L refer to high and low bytes in network byte order.
 ****************************************************************************/

#define TIVA_CANIF_DATA_HBYTE_MASK     0x000000FF
#define TIVA_CANIF_DATA_HBYTE_SHIFT 0
#define TIVA_CANIF_DATA_LBYTE_MASK     0x0000FF00
#define TIVA_CANIF_DATA_LBYTE_SHIFT 8

/****************************************************************************
 * The following are defines for the bit fields in the CANTXRQ1 register.
 ****************************************************************************/

#define TIVA_CAN_TXRQ1_TXRQST_MASK  0x0000FFFF  /* Transmission Request Bits */
#define TIVA_CAN_TXRQ1_TXRQST_SHIFT 0

/****************************************************************************
 * The following are defines for the bit fields in the CANTXRQ2 register.
 ****************************************************************************/

#define TIVA_CAN_TXRQ2_TXRQST_MASK  0x0000FFFF  /* Transmission Request Bits */
#define TIVA_CAN_TXRQ2_TXRQST_SHIFT 0

/****************************************************************************
 * The following are defines for the bit fields in the CANNWDA1 register.
 ****************************************************************************/

#define TIVA_CAN_NWDA1_NEWDAT_MASK  0x0000FFFF  /* New Data Bits */
#define TIVA_CAN_NWDA1_NEWDAT_SHIFT 0

/****************************************************************************
 * The following are defines for the bit fields in the CANNWDA2 register.
 ****************************************************************************/

#define TIVA_CAN_NWDA2_NEWDAT_MASK  0x0000FFFF  /* New Data Bits */
#define TIVA_CAN_NWDA2_NEWDAT_SHIFT 0

/****************************************************************************
 * The following are defines for the bit fields in the CANMSG1INT register.
 ****************************************************************************/

#define TIVA_CAN_MSG1INT_INTPND_MASK        0x0000FFFF  /* Interrupt Pending Bits */
#define TIVA_CAN_MSG1INT_INTPND_SHIFT       0

/****************************************************************************
 * The following are defines for the bit fields in the CANMSG2INT register.
 ****************************************************************************/

#define TIVA_CAN_MSG2INT_INTPND_MASK        0x0000FFFF  /* Interrupt Pending Bits */
#define TIVA_CAN_MSG2INT_INTPND_SHIFT       0

/****************************************************************************
 * The following are defines for the bit fields in the CANMSG1VAL register.
 ****************************************************************************/

#define TIVA_CAN_MSG1VAL_MSGVAL_MASK        0x0000FFFF  /* Message Valid Bits */
#define TIVA_CAN_MSG1VAL_MSGVAL_SHIFT       0

/****************************************************************************
 * The following are defines for the bit fields in the CANMSG2VAL register.
 ****************************************************************************/

#define TIVA_CAN_MSG2VAL_MSGVAL_MASK        0x0000FFFF  /* Message Valid Bits */
#define TIVA_CAN_MSG2VAL_MSGVAL_SHIFT       0

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_TIVA_HARDWARE_TIVA_CAN_H */
