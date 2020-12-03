/****************************************************************************
 * arch/renesas/src/rx65n/rx65n_definitions.h
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

#ifndef __ARCH_RENESAS_SRC_RX65N_RX65N_DEFINITIONS_H
#define __ARCH_RENESAS_SRC_RX65N_RX65N_DEFINITIONS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "arch/rx65n/iodefine.h"
#include "arch/board/board.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Memory-mapped register addresses *****************************************/

#define RX65N_SCI0_BASE       (uint32_t)&SCI0
#define RX65N_SCI1_BASE       (uint32_t)&SCI1
#define RX65N_SCI2_BASE       (uint32_t)&SCI2
#define RX65N_SCI3_BASE       (uint32_t)&SCI3
#define RX65N_SCI4_BASE       (uint32_t)&SCI4
#define RX65N_SCI5_BASE       (uint32_t)&SCI5
#define RX65N_SCI6_BASE       (uint32_t)&SCI6
#define RX65N_SCI7_BASE       (uint32_t)&SCI7
#define RX65N_SCI8_BASE       (uint32_t)&SCI8
#define RX65N_SCI9_BASE       (uint32_t)&SCI9
#define RX65N_SCI10_BASE      (uint32_t)&SCI10
#define RX65N_SCI11_BASE      (uint32_t)&SCI11
#define RX65N_SCI12_BASE      (uint32_t)&SCI12

/* Serial Communications interface (SCI) */

#define RX_SCISMR_CKSMASK  (0x03)        /* Bit 0-1: Internal clock source */
#define RX_SCISMR_DIV1     (0x00)        /*   System clock (phi) */
#define RX_SCISMR_DIV4     (0x01)        /*   phi/4 */
#define RX_SCISMR_DIV16    (0x02)        /*   phi/16 */
#define RX_SCISMR_DIV64    (0x03)        /*   phi/64 */
#define RX_SCISMR_MP       (0x04)        /* Bit 2: Multiprocessor select */
#define RX_SCISMR_STOP     (0x08)        /* Bit 3: 0:One stop bit, 1:Two stop bits */
#define RX_SCISMR_OE       (0x10)        /* Bit 4: 0:Even parity, 1:Odd parity */
#define RX_SCISMR_PE       (0x20)        /* Bit 5: Parity enable */
#define RX_SCISMR_CHR      (0x40)        /* Bit 6: 0:8-bit data, 1:7-bit data */
#define RX_SCISMR_CA       (0x80)        /* Bit 7: 0:Asynchronous, 1:clocked synchronous */
#define RX_SCISCR_CKEMASK  (0x03)        /* Bit 0-1: Internal clock source */

/* Asynchronous mode: */

/*   Internal clock, SCK pin used for input pin */

#define RX_SCISCR_AISIN    (0x00)

/*   Internal clock, SCK pin used for clock output */

#define RX_SCISCR_AISOUT   (0x01)

/*   External clock, SCK pin used for clock input */

#define RX_SCISCR_AXSIN1   (0x02)

/*   External clock, SCK pin used for clock input */

#define RX_SCISCR_AXSIN2   (0x03)

/* Synchronous mode: */

/*   Internal clock, SCK pin used for clock output */

#define RX_SCISCR_SISOUT2  (0x01)

/*   External clock, SCK pin used for clock input */

#define RX_SCISCR_SXSIN1   (0x02)

/* External clock, SCK pin used for clock input */

#define RX_SCISCR_SXSIN2   (0x03)

/* Bit 2: 1=Transmit end interrupt enable */

#define RX_SCISCR_TEIE     (0x04)

/* Bit 3: 1=Multiprocessor interrupt enable */

#define RX_SCISCR_MPIE     (0x08)
#define RX_SCISCR_RE       (0x10)        /* Bit 4: 1=Receiver enable */
#define RX_SCISCR_TE       (0x20)        /* Bit 5: 1=Transmitter enable */

/* Bit 6: 1=Recieve-data-full interrupt enable */

#define RX_SCISCR_RIE      (0x40)

/* Bit 7: 1=Transmit-data-empty interrupt enable */

#define RX_SCISCR_TIE      (0x80)
#define RX_SCISCR_ALLINTS  (0xcc)

/* Bit 0: Multi-processor Bit in Transmit data */

#define RX_SCISSR_MPBT     (0x01)

/* Bit 1: Multi-processor Bit in receive data */

#define RX_SCISSR_MPB      (0x02)
#define RX_SCISSR_TEND     (0x04)  /* Bit 2: End of transmission */
#define RX_SCISSR_PER      (0x08)  /* Bit 3: Receive parity error */
#define RX_SCISSR_FER      (0x10)  /* Bit 4: Receive framing error */
#define RX_SCISSR_ORER     (0x20)  /* Bit 5: Receive overrun error */

/* Bit 6: RDR contains valid received data */

#define RX_SCISSR_RDRF     (0x40)

/* Bit 7: TDR does not contain valid transmit data */

#define RX_SCISSR_TDRE                  (0x80)
#define RX65N_CMT_CMSTR0_ADDR           (0x00088000)  /* 8-bits wide */
#define RX65N_CMT0_CMCNT_ADDR           (0x00088004)
#define RX65N_CMT0_CMCOR_ADDR           (0x00088006)
#define RX65N_CMT0_CMCR_ADDR            (0x00088002)

/* CMTW0 used for Ethernet TX polling and TX timeout */

#define RX65N_CMTW0_CMWSTR_ADDR         (0x00094200)
#define RX65N_CMTW0_CMWCR_ADDR          (0x00094204)
#define RX65N_CMTW0_CMWIOR_ADDR         (0x00094208)
#define RX65N_CMTW0_CMWCNT_ADDR         (0x00094210)
#define RX65N_CMTW0_CMWCOR_ADDR         (0x00094214)
#define RX65N_CMTW0_CMWICR0_ADDR        (0x00094218)
#define RX65N_CMTW0_CMWICR1_ADDR        (0x0009421c)
#define RX65N_CMTW0_CMWOCR0_ADDR        (0x00094220)
#define RX65N_CMTW0_CMWOCR1_ADDR        (0x00094224)
#define RX65N_CMTW0_TICKFREQ            (1)             /* 1Hz tick frequency */
#define RX65N_CMTW0_DIV_VALUE           (32)
#define RX65N_CMTW0_COUNT_VALUE_FOR_TXPOLL    ((RX_PCLKB / RX65N_CMTW0_DIV_VALUE)/(RX65N_CMTW0_TICKFREQ))
#define RX65N_CMTW0_COUNT_VALUE_FOR_TXTIMEOUT (((RX_PCLKB / RX65N_CMTW0_DIV_VALUE)/(RX65N_CMTW0_TICKFREQ))*60)
#define rx65n_cmtw0_txpoll 1
#define rx65n_cmtw0_timeout 2

#define RX65N_MSTPCRA_ADDR              (0x00080010)
#define RX65N_MSTPCRB_ADDR              (0x00080014)
#define RX65N_CMT0_TICKFREQ             (100)           /* 100Hz tick frequency */
#define RX65N_CMT_DIV32                 (0x0001)
#define RX65N_CMT0_DIV_VALUE            (32)
#define RX65N_CMT0_COUNT_VALUE ((RX_PCLKB / RX65N_CMT0_DIV_VALUE)/(RX65N_CMT0_TICKFREQ))
#define RX65N_CMT_CMCR_INIT             (RX65N_CMT_DIV32 |\
                                        RX65N_CMT_CMCR_CMIE_ENABLE |\
                                        RX65N_CMT_CMCR_DEFAULT)
#define RX65N_CMTW_DIV32                (0x0001)
#define RX65N_CMTW_CMWCR_INIT           (RX65N_CMTW_DIV32 |\
                                        RX65N_CMTW_CMWCR_CMWIE_ENABLE |\
                                        RX65N_CMTW_CMWCR_DEFAULT)
#define RX65N_CMTW_CMWCR_DEFAULT        (0x0000)
#define RX65N_CMTW_CMWCR_CMWIE_ENABLE   (0x0008)

#define RX65N_CMT_CMCR_DEFAULT          (0x0080)
#define RX65N_CMT_CMCR_CMIE_ENABLE      (0x0040)
#define RX65N_CMT_MSTPCRA_STOP          (0x00008000)    /* Release unit0(CMT0 and CMT1) from module stop state */
#define RX65N_CMT_UNIT1_MSTPCRA_STOP    (0x00004000)    /* Release unit1(CMT2 and CMT3) from module stop state */
#define RX65N_CMTW_UNIT1_MSTPCRA_STOP   (0x00000001)    /* Release CMTW unit1 from module stop state */
#define RX65N_CMTW_UNIT0_MSTPCRA_STOP   (0x00000002)    /* Release CMTW unit0 from module stop state */
#define RX65N_CMTCMSTR0_STR0            (0x0001)        /* Bit 0: TCNT0 is counting */
#define RX65N_CMTCMSTR0_STR1            (0x0002)        /* Bit 1: TCNT1 is counting */
#define RX65N_CMTCMSTR1_STR2            (0x0001)        /* Bit 0: TCNT0 is counting */
#define RX65N_CMTCMSTR1_STR3            (0x0002)        /* Bit 1: TCNT1 is counting */
#define RX65N_PRCR_ADDR                 (0x000803fe)
#define RX65N_PRCR_VALUE                (0xa50b)
#define RX65N_GRPBE0_ADDR               (0x00087600)
#define RX65N_GRPBL0_ADDR               (0x00087630)
#define RX65N_GRPBL1_ADDR               (0x00087634)
#define RX65N_GRPBL2_ADDR               (0x00087638)
#define RX65N_GRPAL0_ADDR               (0x00087830)
#define RX65N_GRPAL1_ADDR               (0x00087834)
#define RX65N_GENBE0_ADDR               (0x00087640)
#define RX65N_GENBL0_ADDR               (0x00087670)
#define RX65N_GENBL1_ADDR               (0x00087674)
#define RX65N_GENBL2_ADDR               (0x00087678)
#define RX65N_GENAL0_ADDR               (0x00087870)
#define RX65N_GENAL1_ADDR               (0x00087874)
#define RX65N_GRPBL0_TEI0_MASK          (1U <<  0)  /* (0x00000001) */
#define RX65N_GRPBL0_ERI0_MASK          (1U <<  1)  /* (0x00000002) */
#define RX65N_GRPBL0_TEI1_MASK          (1U <<  2)  /* (0x00000004) */
#define RX65N_GRPBL0_ERI1_MASK          (1U <<  3)  /* (0x00000008) */
#define RX65N_GRPBL0_TEI2_MASK          (1U <<  4)  /* (0x00000010) */
#define RX65N_GRPBL0_ERI2_MASK          (1U <<  5)  /* (0x00000020) */
#define RX65N_GRPBL0_TEI3_MASK          (1U <<  6)  /* (0x00000040) */
#define RX65N_GRPBL0_ERI3_MASK          (1U <<  7)  /* (0x00000080) */
#define RX65N_GRPBL0_TEI4_MASK          (1U <<  8)  /* (0x00000100) */
#define RX65N_GRPBL0_ERI4_MASK          (1U <<  9)  /* (0x00000200) */
#define RX65N_GRPBL0_TEI5_MASK          (1U << 10)  /* (0x00000400) */
#define RX65N_GRPBL0_ERI5_MASK          (1U << 11)  /* (0x00000800) */
#define RX65N_GRPBL0_TEI6_MASK          (1U << 12)  /* (0x00001000) */
#define RX65N_GRPBL0_ERI6_MASK          (1U << 13)  /* (0x00002000) */
#define RX65N_GRPBL0_TEI7_MASK          (1U << 14)  /* (0x00004000) */
#define RX65N_GRPBL0_ERI7_MASK          (1U << 15)  /* (0x00008000) */
#define RX65N_GRPBL1_TEI0_MASK          (1U << 13)
#define RX65N_GRPBL1_EEI0_MASK          (1U << 14)
#define RX65N_GRPBL1_TEI2_MASK          (1U << 15)
#define RX65N_GRPBL1_EEI2_MASK          (1U << 16)
#define RX65N_GRPBL1_TEI8_MASK          (1U << 24)
#define RX65N_GRPBL1_ERI8_MASK          (1U << 25)
#define RX65N_GRPBL1_TEI9_MASK          (1U << 26)
#define RX65N_GRPBL1_ERI9_MASK          (1U << 27)
#define RX65N_GRPBL1_TEI1_MASK          (1U << 28)
#define RX65N_GRPBL1_EEI1_MASK          (1U << 29)
#define RX65N_GRPAL0_TEI10_MASK         (1U <<  8)
#define RX65N_GRPAL0_ERI10_MASK         (1U <<  9)
#define RX65N_GRPAL0_TEI11_MASK         (1U << 12)
#define RX65N_GRPAL0_ERI11_MASK         (1U << 13)
#define RX65N_GRPAL0_SPII0_MASK         (1U << 16)
#define RX65N_GRPAL0_SPEI0_MASK         (1U << 17)
#define RX65N_GRPAL0_SPII1_MASK         (1U << 18)
#define RX65N_GRPAL0_SPEI1_MASK         (1U << 19)
#define RX65N_GRPAL0_SPII2_MASK         (1U << 20)
#define RX65N_GRPAL0_SPEI2_MASK         (1U << 21)
#define RX65N_GRPBL0_TEI12_MASK         (1U << 16)
#define RX65N_GRPBL0_ERI12_MASK         (1U << 17)

/* Start Ethernet and EDMAC Interface */

/* ETHERC and EDMAC base Addresses */

#define RX65N_EDMAC0_BASE       (0x000c0000)     /* EDMAC base address */
#define RX65N_ETHERC0_BASE      (0x000c0100)     /* Ethernet MAC base address */

/* Ethernet Addresses */

/* Register Offsets  */

/* MAC Registers */

#define RX65N_ETH_ECMR_OFFSET          (0x0000) /* ETHERC Mode register */
#define RX65N_ETH_RFLR_OFFSET          (0x0008) /* Receive Frame Maximum Length Register */
#define RX65N_ETH_ECSR_OFFSET          (0x0010) /* ETHERC Status Register */
#define RX65N_ETH_ECSIPR_OFFSET        (0x0018) /* ETHERC Interrupt Enable Register */
#define RX65N_ETH_PIR_OFFSET           (0x0020) /* PHY Interface Register */
#define RX65N_ETH_PSR_OFFSET           (0x0028) /* PHY Status Register */
#define RX65N_ETH_RDMLR_OFFSET         (0x0040) /* Random Number Generation Counter Limit Setting Register */
#define RX65N_ETH_IPGR_OFFSET          (0x0050) /* Interpacket Gap Register */
#define RX65N_ETH_APR_OFFSET           (0x0054) /* Automatic PAUSE Frame Register */
#define RX65N_ETH_MPR_OFFSET           (0x0058) /* Manual PAUSE Frame Register */
#define RX65N_ETH_RFCF_OFFSET          (0x0060) /* Received PAUSE Frame Counter */
#define RX65N_ETH_TPAUSER_OFFSET       (0x0064) /* PAUSE Frame Retransmit Count Setting Register */
#define RX65N_ETH_TPAUSECR_OFFSET      (0x0068) /* PAUSE Frame Retransmit Counter */
#define RX65N_ETH_BCFRR_OFFSET         (0x006c) /* Broadcast Frame Receive Count Setting Register */
#define RX65N_ETH_MAHR_OFFSET          (0x00c0) /* MAC Address Upper Bit Register */
#define RX65N_ETH_MALR_OFFSET          (0x00c8) /* MAC Address Lower Bit Register */
#define RX65N_ETH_TROCR_OFFSET         (0x00d0) /* Transmit Retry Over Counter Register */
#define RX65N_ETH_CDCR_OFFSET          (0x00d4) /* Late Collision Detect Counter Register */
#define RX65N_ETH_LCCR_OFFSET          (0x00d8) /* Lost Carrier Counter Register */
#define RX65N_ETH_CNDCR_OFFSET         (0x00dc) /* Carrier Not Detect Counter Register */
#define RX65N_ETH_CEFCR_OFFSET         (0x00e4) /* CRC Error Frame Receive Counter Register */
#define RX65N_ETH_FRECR_OFFSET         (0x00e8) /* Frame Receive Error Counter Register */
#define RX65N_ETH_TSFRCR_OFFSET        (0x00ec) /* Too-Short Frame Receive Counter Register */
#define RX65N_ETH_TLFRCR_OFFSET        (0x00f0) /* Too-Long Frame Receive Counter Register */
#define RX65N_ETH_RFCR_OFFSET          (0x00f4) /* Received Alignment Error Frame Counter Register */
#define RX65N_ETH_MAFCR_OFFSET         (0x00f8) /* Multicast Address Frame Receive Counter Register */

/* DMA Registers */

#define RX65N_ETHD_EDMR_OFFSET         (0x0000) /* EDMAC Mode Register  */
#define RX65N_ETHD_EDTRR_OFFSET        (0x0008) /* EDMAC Transmit Request Register  */
#define RX65N_ETHD_EDRRR_OFFSET        (0x0010) /* EDMAC Receive Request Register  */
#define RX65N_ETHD_TDLAR_OFFSET        (0x0018) /* Transmit Descriptor List Start Address Register  */
#define RX65N_ETHD_RDLAR_OFFSET        (0x0020) /* Receive Descriptor List Start Address Register  */
#define RX65N_ETHD_EESR_OFFSET         (0x0028) /* ETHERC/EDMAC Status Register  */
#define RX65N_ETHD_EESIPR_OFFSET       (0x0030) /* ETHERC/EDMAC Status Interrupt Enable Register  */
#define RX65N_ETHD_TRSCER_OFFSET       (0x0038) /* ETHERC/EDMAC Transmit/Receive Status Copy Enable Register  */
#define RX65N_ETHD_RMFCR_OFFSET        (0x0040) /* Missed-Frame Counter Register  */
#define RX65N_ETHD_TFTR_OFFSET         (0x0048) /* Transmit FIFO Threshold Register  */
#define RX65N_ETHD_FDR_OFFSET          (0x0050) /* FIFO Depth Register  */
#define RX65N_ETHD_RMCR_OFFSET         (0x0058) /* Receive Method Control Register  */
#define RX65N_ETHD_TFUCR_OFFSET        (0x0064) /* Transmit FIFO Underflow Counter  */
#define RX65N_ETHD_RFOCR_OFFSET        (0x0068) /* Receive FIFO Overflow Counter  */
#define RX65N_ETHD_IOSR_OFFSET         (0x006c) /* Independent Output Signal Setting Register  */
#define RX65N_ETHD_FCFTR_OFFSET        (0x0070) /* Flow Control Start FIFO Threshold Setting Register  */
#define RX65N_ETHD_RPADIR_OFFSET       (0x0078) /* Receive Data Padding Insert Register  */
#define RX65N_ETHD_TRIMD_OFFSET        (0x007c) /* Transmit Interrupt Setting Register  */
#define RX65N_ETHD_RBWAR_OFFSET        (0x00c8) /* Receive Buffer Write Address Register  */
#define RX65N_ETHD_RDFAR_OFFSET        (0x00cc) /* Receive Descriptor Fetch Address Register  */
#define RX65N_ETHD_TBRAR_OFFSET        (0x00d4) /* Transmit Buffer Read Address Register  */
#define RX65N_ETHD_TDFAR_OFFSET        (0x00d8) /* Transmit Descriptor Fetch Address Register  */

/* Register Base Addresses */

/* MAC Registers */

#define RX65N_ETH_ECMR     (RX65N_ETHERC0_BASE+RX65N_ETH_ECMR_OFFSET)
#define RX65N_ETH_RFLR     (RX65N_ETHERC0_BASE+RX65N_ETH_RFLR_OFFSET)
#define RX65N_ETH_ECSR     (RX65N_ETHERC0_BASE+RX65N_ETH_ECSR_OFFSET)
#define RX65N_ETH_ECSIPR   (RX65N_ETHERC0_BASE+RX65N_ETH_ECSIPR_OFFSET)
#define RX65N_ETH_PIR      (RX65N_ETHERC0_BASE+RX65N_ETH_PIR_OFFSET)
#define RX65N_ETH_PSR      (RX65N_ETHERC0_BASE+RX65N_ETH_PSR_OFFSET)
#define RX65N_ETH_RDMLR    (RX65N_ETHERC0_BASE+RX65N_ETH_RDMLR_OFFSET)
#define RX65N_ETH_IPGR     (RX65N_ETHERC0_BASE+RX65N_ETH_IPGR_OFFSET)
#define RX65N_ETH_APR      (RX65N_ETHERC0_BASE+RX65N_ETH_APR_OFFSET)
#define RX65N_ETH_MPR      (RX65N_ETHERC0_BASE+RX65N_ETH_MPR_OFFSET)
#define RX65N_ETH_RFCF     (RX65N_ETHERC0_BASE+RX65N_ETH_RFCF_OFFSET)
#define RX65N_ETH_TPAUSER  (RX65N_ETHERC0_BASE+RX65N_ETH_TPAUSER_OFFSET)
#define RX65N_ETH_TPAUSECR (RX65N_ETHERC0_BASE+RX65N_ETH_TPAUSECR_OFFSET)
#define RX65N_ETH_BCFRR    (RX65N_ETHERC0_BASE+RX65N_ETH_BCFRR_OFFSET)
#define RX65N_ETH_MAHR     (RX65N_ETHERC0_BASE+RX65N_ETH_MAHR_OFFSET)
#define RX65N_ETH_MALR     (RX65N_ETHERC0_BASE+RX65N_ETH_MALR_OFFSET)
#define RX65N_ETH_TROCR    (RX65N_ETHERC0_BASE+RX65N_ETH_TROCR_OFFSET)
#define RX65N_ETH_CDCR     (RX65N_ETHERC0_BASE+RX65N_ETH_CDCR_OFFSET)
#define RX65N_ETH_LCCR     (RX65N_ETHERC0_BASE+RX65N_ETH_LCCR_OFFSET)
#define RX65N_ETH_CNDCR    (RX65N_ETHERC0_BASE+RX65N_ETH_CNDCR_OFFSET)
#define RX65N_ETH_CEFCR    (RX65N_ETHERC0_BASE+RX65N_ETH_CEFCR_OFFSET)
#define RX65N_ETH_FRECR    (RX65N_ETHERC0_BASE+RX65N_ETH_FRECR_OFFSET)
#define RX65N_ETH_TSFRCR   (RX65N_ETHERC0_BASE+RX65N_ETH_TSFRCR_OFFSET)
#define RX65N_ETH_TLFRCR   (RX65N_ETHERC0_BASE+RX65N_ETH_TLFRCR_OFFSET)
#define RX65N_ETH_RFCR     (RX65N_ETHERC0_BASE+RX65N_ETH_RFCR_OFFSET)
#define RX65N_ETH_MAFCR    (RX65N_ETHERC0_BASE+RX65N_ETH_MAFCR_OFFSET)

/* DMA Registers */

#define RX65N_ETHD_EDMR          (RX65N_EDMAC0_BASE+RX65N_ETHD_EDMR_OFFSET)
#define RX65N_ETHD_EDTRR         (RX65N_EDMAC0_BASE+RX65N_ETHD_EDTRR_OFFSET)
#define RX65N_ETHD_EDRRR         (RX65N_EDMAC0_BASE+RX65N_ETHD_EDRRR_OFFSET)
#define RX65N_ETHD_TDLAR         (RX65N_EDMAC0_BASE+RX65N_ETHD_TDLAR_OFFSET)
#define RX65N_ETHD_RDLAR         (RX65N_EDMAC0_BASE+RX65N_ETHD_RDLAR_OFFSET)
#define RX65N_ETHD_EESR          (RX65N_EDMAC0_BASE+RX65N_ETHD_EESR_OFFSET)
#define RX65N_ETHD_EESIPR        (RX65N_EDMAC0_BASE+RX65N_ETHD_EESIPR_OFFSET)
#define RX65N_ETHD_TRSCER        (RX65N_EDMAC0_BASE+RX65N_ETHD_TRSCER_OFFSET)
#define RX65N_ETHD_RMFCR         (RX65N_EDMAC0_BASE+RX65N_ETHD_RMFCR_OFFSET)
#define RX65N_ETHD_TFTR          (RX65N_EDMAC0_BASE+RX65N_ETHD_TFTR_OFFSET)
#define RX65N_ETHD_FDR           (RX65N_EDMAC0_BASE+RX65N_ETHD_FDR_OFFSET)
#define RX65N_ETHD_RMCR          (RX65N_EDMAC0_BASE+RX65N_ETHD_RMCR_OFFSET)
#define RX65N_ETHD_TFUCR         (RX65N_EDMAC0_BASE+RX65N_ETHD_TFUCR_OFFSET)
#define RX65N_ETHD_RFOCR         (RX65N_EDMAC0_BASE+RX65N_ETHD_RFOCR_OFFSET)
#define RX65N_ETHD_IOSR          (RX65N_EDMAC0_BASE+RX65N_ETHD_IOSR_OFFSET)
#define RX65N_ETHD_FCFTR         (RX65N_EDMAC0_BASE+RX65N_ETHD_FCFTR_OFFSET)
#define RX65N_ETHD_RPADIR        (RX65N_EDMAC0_BASE+RX65N_ETHD_RPADIR_OFFSET)
#define RX65N_ETHD_TRIMD         (RX65N_EDMAC0_BASE+RX65N_ETHD_TRIMD_OFFSET)
#define RX65N_ETHD_RBWAR         (RX65N_EDMAC0_BASE+RX65N_ETHD_RBWAR_OFFSET)
#define RX65N_ETHD_RDFAR         (RX65N_EDMAC0_BASE+RX65N_ETHD_RDFAR_OFFSET)
#define RX65N_ETHD_TBRAR         (RX65N_EDMAC0_BASE+RX65N_ETHD_TBRAR_OFFSET)
#define RX65N_ETHD_TDFAR         (RX65N_EDMAC0_BASE+RX65N_ETHD_TDFAR_OFFSET)

/* MPC (Multifunction pin controller) Registers for Ethernet */

#define RX65N_MPC_PFENET         (0x0008c10e)
#define RX65N_MPC_PWPR           (0x0008c11f)

/* (Module control Registers) for Ethernet */

#define RX65N_MSTP_CRB           (0x00080014)

/* Register Bit-Field Definitions */

/* MAC Registers */

#define ETH_ECSR_LCHNG     (1 << 2)     /* Bit 2: Link Signal Change Flag */

/* Bit 2: LINK Signal Change Interrupt enable/disable */

#define ETH_ECSIPR_LCHNGIP (1 << 2)
#define ETH_ECMR_CLR       (0x00000000)

/* Clear all ETHERC status BFR, PSRTO, LCHNG, MPD, ICD */

#define ETH_ECSR_CLR       (0x00000037)
#define ETH_ECMR_RE        (1 << 6) /* Transmit function is enabled */
#define ETH_ECMR_TE        (1 << 5) /* Receive function is enabled */
#define ETH_ECMR_DM        (1 << 1) /* Duplex Mode */
#define ETH_ECMR_RTM       (1 << 2) /* Bit Rate */

/* Bit 4:0:Interpacket Gap 96 bit time (initial value) */

#define ETH_IPGR_IPG_INITIAL     (0x00000014)

/* Receive Frame Maximum Length */

#define ETH_RFLR_RFL             (1518)

/* EDMA Registers */

/* Bit 22: ETHERC status interrupt request is enable/disabled. */

#define ETHD_EDMR_SWR            (1 << 0)

/* Bit 6: Big Endian Mode/Little Endian Mode */

#define ETHD_EDMR_DE             (1 << 6)

/* Clear all EDMAC status bits */

#define ETHD_EESR_EDMAC          (0x47ff0f9f)

/* Frame transfer Complete status Flag check */

#define ETHD_EESR_TC         (1 << 21)

/* ETHERC/EDMAC Status Register Source Flag */

#define ETHD_EESR_ECI        (1 << 22)
#define ETHD_EESR_FR         (1 << 18)   /* Frame Receive Flag */

/* Frame Receive Interrupt Request Enable */

#define ETHD_EESIPR_FRIP     (1 << 18)

/* Frame Transfer Complete Interrupt Request Enable */

#define ETHD_EESIPR_TCIP     (1 << 21)

/* ETHERC/EDMAC Status Register Source Interrupt Request Enable */

#define ETHD_EESIPR_ECIIP    (1 << 22)

/* ETHERC/EDMAC Write-Back Complete Interrupt Request Enable */

#define ETHD_EESIPR_TWBIP    (1 << 30)

/* Bit 0:10: Transmit FIFO Threshold */

#define ETHD_TFTR_TFT        (0x00000000)

/* Bit: 20: Transmit Descriptor Empty Flag */

#define ETHD_EESR_TDE        (1 << 20)

/* Ether PSR register */

#define ETH_PSR_LMON         (1)

/* EDMAC Transmit Request Register's bit */

#define ETHD_EDRRR_TR        (1) /* Transmit Request */

/* EDMAC Receive Request Register's bit */

#define ETHD_EDRRR_RR        (1) /* Receive descriptor read,
                               * and receive function is enabled
                               */

/* Transmit Interrupt Setting Register's bit */

#define ETHD_TRIMD_TIS        (1)      /* Transmit Interrupt is enabled */
#define ETHD_TRIMD_TIM        (1 << 4) /* Write-back complete interrupt mode */

/* Receive Method Control Register's bit */

/* Receive Method Control Register's bit */

#define ETHD_RMCR_RNR        (1) /* EDRRR.RR bit (receive request bit) is not
                                  * set to 0 when one frame has been received
                                                                  */

/* FIFO Depth Register's bit */

#define ETHD_FDR_RFD         (7)       /* Receive FIFO Depth */
#define ETHD_FDR_TFD         (7 << 8)  /* Transmit FIFO Depth */

/* ETHERC/EDMAC Transmit/Receive Status Copy Enable Register's bit */

#define ETHD_TRSCER_RRFCE     (1 << 4)  /* RRF Flag Copy Enable */
#define ETHD_TRSCER_RMAFCE    (1 << 7)  /* RMAF Flag Copy Enable */

/* Broadcast Frame Receive Count Setting Register's field */

#define ETH_BCFRR_BCF    (0x0000)  /* Broadcast Frame Continuous Receive Count Setting */

/* PHY Interface Register's bit and values */

#define ETH_PIR_MDC              (1)          /* MII/RMII Management Data Clock */
#define ETH_PIR_MMD              (1 << 1)     /* MII/RMII Management Mode */
#define ETH_PIR_MDO              (1 << 2)     /* MII/RMII Management Data-Out */
#define ETH_PIR_MDI              (1 << 3)     /* MII/RMII Management Data-In */

#define ETH_PIR_RESET_ALL        (0x00000000) /* Reset All Flags of PIR */
#define ETH_PIR_SET_MDC          (0x00000001) /* Setting MDC of PIR */
#define ETH_PIR_SET_MMD          (0x00000002) /* Setting MMD of PIR */
#define ETH_PIR_SET_MMD_MDC      (0x00000003) /* Setting MMD and MDC */
#define ETH_PIR_SET_MDO_MMD      (0x00000006) /* Setting MDO and MMD */
#define ETH_PIR_SET_MDO_MMD_MDC  (0x00000007) /* Setting MDO, MMD and MDC */

/* Ethernet Control Register's bit and value */

#define ETH_PFENET_MII_MODE      (0x10)
#define ETH_PFENET_RMII_MODE     (0x00)

/* End Ethernet and EDMAC Interface */

/* Bit Set Values */

#define SET_BIT_HIGH    (1)
#define SET_BIT_LOW     (0)
#define SET_BYTE_HIGH   (0xff)
#define SET_BYTE_LOW    (0x00)

/* RTC Register Offsets */

#define RX65N_RTC_R64CNT_OFFSET    (0x0000)
#define RX65N_RTC_RSECCNT_OFFSET   (0x0002)
#define RX65N_RTC_RMINCNT_OFFSET   (0x0004)
#define RX65N_RTC_RHRCNT_OFFSET    (0x0006)
#define RX65N_RTC_RWKCNT_OFFSET    (0x0008)
#define RX65N_RTC_RDAYCNT_OFFSET   (0x000a)
#define RX65N_RTC_RMONCNT_OFFSET   (0x000c)
#define RX65N_RTC_RYRCNT_OFFSET    (0x000e)
#define RX65N_RTC_RSECAR_OFFSET    (0x0010)
#define RX65N_RTC_RMINAR_OFFSET    (0x0012)
#define RX65N_RTC_RHRAR_OFFSET     (0x0014)
#define RX65N_RTC_RWKAR_OFFSET     (0x0016)
#define RX65N_RTC_RDAYAR_OFFSET    (0x0018)
#define RX65N_RTC_RMONAR_OFFSET    (0x001a)
#define RX65N_RTC_RYRAR_OFFSET     (0x001c)
#define RX65N_RTC_RYRAREN_OFFSET   (0x001e)
#define RX65N_RTC_RCR1_OFFSET      (0x0022)
#define RX65N_RTC_RCR2_OFFSET      (0x0024)
#define RX65N_RTC_RCR3_OFFSET      (0x0026)
#define RX65N_RTC_RCR4_OFFSET      (0x0028)
#define RX65N_RTC_RADJ_OFFSET      (0x002e)

#define RX65N_RTC_BASE             (0x0008c400)
#define RX65N_RTC_R64CNT           (RX65N_RTC_BASE + RX65N_RTC_R64CNT_OFFSET)
#define RX65N_RTC_RSECCNT          (RX65N_RTC_BASE + RX65N_RTC_RSECCNT_OFFSET)
#define RX65N_RTC_RMINCNT          (RX65N_RTC_BASE + RX65N_RTC_RMINCNT_OFFSET)
#define RX65N_RTC_RHRCNT           (RX65N_RTC_BASE + RX65N_RTC_RHRCNT_OFFSET)
#define RX65N_RTC_RWKCNT           (RX65N_RTC_BASE + RX65N_RTC_RWKCNT_OFFSET)
#define RX65N_RTC_RDAYCNT          (RX65N_RTC_BASE + RX65N_RTC_RDAYCNT_OFFSET)
#define RX65N_RTC_RMONCNT          (RX65N_RTC_BASE + RX65N_RTC_RMONCNT_OFFSET)
#define RX65N_RTC_RYRCNT           (RX65N_RTC_BASE + RX65N_RTC_RYRCNT_OFFSET)
#define RX65N_RTC_RSECAR           (RX65N_RTC_BASE + RX65N_RTC_RSECAR_OFFSET)
#define RX65N_RTC_RMINAR           (RX65N_RTC_BASE + RX65N_RTC_RMINAR_OFFSET)
#define RX65N_RTC_RHRAR            (RX65N_RTC_BASE + RX65N_RTC_RHRAR_OFFSET)
#define RX65N_RTC_RWKAR            (RX65N_RTC_BASE + RX65N_RTC_RWKAR_OFFSET)
#define RX65N_RTC_RDAYAR           (RX65N_RTC_BASE + RX65N_RTC_RDAYAR_OFFSET)
#define RX65N_RTC_RMONAR           (RX65N_RTC_BASE + RX65N_RTC_RMONAR_OFFSET)
#define RX65N_RTC_RYRAR            (RX65N_RTC_BASE + RX65N_RTC_RYRAR_OFFSET)
#define RX65N_RTC_RYRAREN          (RX65N_RTC_BASE + RX65N_RTC_RYRAREN_OFFSET)
#define RX65N_RTC_RCR1             (RX65N_RTC_BASE + RX65N_RTC_RCR1_OFFSET)
#define RX65N_RTC_RCR2             (RX65N_RTC_BASE + RX65N_RTC_RCR2_OFFSET)
#define RX65N_RTC_RCR3             (RX65N_RTC_BASE + RX65N_RTC_RCR3_OFFSET)
#define RX65N_RTC_RCR4             (RX65N_RTC_BASE + RX65N_RTC_RCR4_OFFSET)
#define RX65N_RTC_RADJ             (RX65N_RTC_BASE + RX65N_RTC_RADJ_OFFSET)

#define RTC_RTC_ALRDIS             (0x00)
#define RTC_RCR4_RCKSEL            (0x00)
#define RTC_RCR3_RTCEN             (0x01)
#define RTC_RCR3_RTCDV             (0x02)
#define RTC_RCR2_START             (0x01)
#define RTC_RCR2_CNTMD             (0x00)
#define RTC_RCR2_RESET             (0x01)
#define RTC_ALARM_INT_ENABLE       (0x01)
#define RTC_CARRY_INT_ENABLE       (0x02)
#define RTC_PERIOD_INT_ENABLE      (0x04)
#define RTC_PERIODIC_INT_PERIOD_1  (0xe0)
#define _04_FOUR_READ_COUNT        (0x04)
#define RTC_1_64_SEC_CYCLE         (0x0005b8d9)
#define _0F_RTC_PRIORITY_LEVEL15   (0x0f)
#define  RTC_RCR1_CUP              (0x02)
#define RX65N_SUBCLKOSC_SOSCCR     (0x00080033)
#define SUBCLKOSC_SOSCCR_SOSTP     (0x01)
#define RX65N_SUBCLKOSC_SOSCWTCR   (0x0008c293)
#define RTC_SOSCWTCR_VALUE         (0x21)
#define RTC_DUMMY_READ             (3)
#define _00_RTC_PRIORITY_LEVEL0    (0)
#define _04_RTC_PERIOD_INT_ENABLE  (0x04)
#define RTC_RTC_CARRYDIS           (0xe5)
#define RTC_RTC_PERDIS             (0xe3)
#define RTC_RADJ_INITVALUE         (0x0)
#define RTC_RCR2_AADJE             (0x10)
#define RTC_RCR2_AADJP             (0x20)

#if defined(CONFIG_RTC) || defined(CONFIG_RTC_DRIVER)

#define HAVE_RTC_DRIVER            1

#endif

/* Constant values used in RTC */

#define RX65N_RTC_WAIT_PERIOD           184
#define RTC_RCR2_HR24                   (0x40)
#define RTC_PERIODIC_INTERRUPT_2_SEC    (0xf)

/* StandBy RAM Address */

#define RX65N_SBRAM_BASE  0x000a4000

/* USB Related definitions */

#define RX65N_NUSBHOST   1

/* USB Registers */

/* USB Peripheral base address */

#define RX65N_MSTPCRB_START_STOP_USB    (1 << 19)
#define RX65N_USB_BASE                  (0x000a0000UL)

/* Different USB registers with corresponding offset */

/* USB System Configuration register and its bit fields */

#define RX65N_USB_SYSCFG                ((volatile short *) (RX65N_USB_BASE + 0x0000UL))
#define RX65N_USB_SYSCFG_SCKE           (1U << 10)
#define RX65N_USB_SYSCFG_DCFM           (1U << 6)
#define RX65N_USB_SYSCFG_DRPD           (1U << 5)
#define RX65N_USB_SYSCFG_DPRPU          (1U << 4)
#define RX65N_USB_SYSCFG_USBE           (1U << 0)

#define RX65N_USB_SYSSTS0               ((volatile short *) (RX65N_USB_BASE + 0x0004UL))
#define USB_FS_JSTS                     (0x0001u)   /* Full-Speed J State */
#define USB_LNST                        (0x0003u)   /* b1-0: D+, D- line status */
#define RX65N_USB_SYSSTS0_LNST          (3)
#define RX65N_USB_SYSSTS0_IDMON         (1U << 2)
#define RX65N_USB_SYSSTS0_SOFEA         (1U << 5)
#define RX65N_USB_SYSSTS0_HTACT         (1U << 6)
#define RX65N_USB_SYSSTS0_OVCMON        (0xc000U)

/* SE1 */

#define RX65N_USB_SYSSTS0_LNST_SE1      (0x3u)

/* Full speed K state */

#define RX65N_USB_SYSSTS0_LNST_FS_KSTS  (0x2u)

/* Full speed J State */

#define RX65N_USB_SYSSTS0_LNST_FS_JSTS  (0x1u)

/* Low speed K state */

#define RX65N_USB_SYSSTS0_LNST_LS_KSTS  (0x2u)

/* Low speed J State */

#define RX65N_USB_SYSSTS0_LNST_LS_JSTS  (0x2u)

/* SE0 */

#define RX65N_USB_SYSSTS0_LNST_SE0      (0x0u)

#define USB_ATTACH                      (0x0040)
#define USB_ATTACHL                     (0x0041)
#define USB_ATTACHF                     (0x0042)
#define USB_DETACH                      (0x0043)
#define USB_RESUME                      (0x0044)
#define USB_SUSPEND                     (0x0045)

/* Definitions used to pass on the information from interrupt to worker
 * function
 */

#define USB_PROCESS_ATTACHED_INT         (0x0050)
#define USB_PROCESS_DETACHED_INT         (0x0051)
#define USB_PROCESS_BRDY_INT             (0x0052)
#define USB_PROCESS_BEMP_INT             (0x0053)
#define USB_PROCESS_NRDY_INT             (0x0054)
#define USB_PROCESS_SACK_INT             (0x0055)
#define USB_PROCESS_SIGN_INT             (0x0056)

#define USB_UACTON                       (1)
#define USB_UACTOFF                      (0)
#define USB_VBON                         (1)
#define USB_VBOFF                        (0)

#define USB_UNDECID                      (0x0000U) /* Undecided */

/* USB Device State Control register 0 and its bit fields */

#define RX65N_USB_DVSTCTR0                      ((volatile short *) (RX65N_USB_BASE + 0x0008UL))
#define RX65N_USB_DVSTCTR0_HNPBTOA              (1U << 11)
#define RX65N_USB_DVSTCTR0_EXICEN               (1U << 10)
#define RX65N_USB_DVSTCTR0_VBUSEN               (1U << 9)
#define RX65N_USB_DVSTCTR0_WKUP                 (1U << 8)
#define RX65N_USB_DVSTCTR0_RWUPE                (1U << 7)
#define RX65N_USB_DVSTCTR0_USBRST               (1U << 6)
#define RX65N_USB_DVSTCTR0_RESUME               (1U << 5)
#define RX65N_USB_DVSTCTR0_UACT                 (1U << 4)
#define RX65N_USB_DVSTCTR0_RHST                 (0x7U)
#define USB_RHST                                (RX65N_USB_DVSTCTR0_RHST)
#define RX65N_USB_DVSTCTR0_SPEED_LOW            (1)
#define RX65N_USB_DVSTCTR0_SPEED_FULL           (2)
#define RX65N_USB_DVSTCTR0_RESET_IN_PROGRESS    (4)

/*      USB CFIFO Port Register and its bit fields */

#define RX65N_USB_CFIFO             ((volatile short *) (RX65N_USB_BASE + 0x0014UL))

/*      USB D0FIFO Port Register and its bit fields */

#define RX65N_USB_D0FIFO            ((volatile short *) (RX65N_USB_BASE + 0x0018UL))

/*      USB D1FIFO Port Register and its bit fields */

#define RX65N_USB_D1FIFO            ((volatile short *) (RX65N_USB_BASE + 0x001cUL))

/*      USB CFIFO Port Select Register and its bit fields */

#define RX65N_USB_CFIFOSEL              ((volatile short *) (RX65N_USB_BASE + 0x0020UL))
#define RX65N_USB_CFIFOSEL_RCNT         (1U << 15)
#define USB_RCNT                        (RX65N_USB_CFIFOSEL_RCNT)
#define RX65N_USB_CFIFOSEL_REW          (1U << 14)
#define RX65N_USB_CFIFOSEL_MBW_8        (0U << 10)
#define RX65N_USB_CFIFOSEL_MBW_16       (1U << 10)
#define RX65N_USB_CFIFOSEL_BIGEND       (1U << 8)
#define RX65N_USB_CFIFOSEL_ISEL         (1U << 5)
#define USB_ISEL                        (RX65N_USB_CFIFOSEL_ISEL)
#define RX65N_USB_CFIFOSEL_CURPIPE_MASK (0xfU)
#define USB_CURPIPE                     (RX65N_USB_CFIFOSEL_CURPIPE_MASK)

/*      USB CFIFO Port Control Register */

#define RX65N_USB_CFIFOCTR              ((volatile short *) (RX65N_USB_BASE + 0x0022UL))

/* Common bit field values for CFIFOCTR, D0FIFOCTR and D1FIFOCTR registers */

#define RX65N_USB_FIFOCTR_BVAL          (1U << 15)
#define USB_BVAL                        (RX65N_USB_FIFOCTR_BVAL)
#define RX65N_USB_FIFOCTR_BCLR          (1U << 14)
#define RX65N_USB_FIFOCTR_FRDY          (1U << 13)
#define RX65N_USB_FIFOCTR_DTLN          (0xfff)

/*      USB D0FIFO and D1FIFO port select and control registers */

#define RX65N_USB_D0FIFOSEL             ((volatile short *) (RX65N_USB_BASE + 0x0028UL))
#define RX65N_USB_D0FIFOSEL_MBW_16      (1U << 10)
#define RX65N_USB_D0FIFOCTR             ((volatile short *) (RX65N_USB_BASE + 0x002aUL))
#define RX65N_USB_D1FIFOSEL             ((volatile short *) (RX65N_USB_BASE + 0x002cUL))
#define RX65N_USB_D1FIFOSEL_MBW_16      (1U << 10)
#define RX65N_USB_D1FIFOCTR             ((volatile short *) (RX65N_USB_BASE + 0x002eUL))

#define RX65N_USB_USING_CFIFO           (0)
#define RX65N_USB_USING_D0FIFO          (1)
#define RX65N_USB_USING_D1FIFO          (2)

#define USB_CUSE                        (RX65N_USB_USING_CFIFO)
#define USB_D0USE                       (RX65N_USB_USING_D0FIFO)
#define USB_D1USE                       (RX65N_USB_USING_D1FIFO)

#define USB_ERROR                       (0xffUL)
#define RX65N_USB_FIFO_ERROR            (0xffUL)
#define USB_TRUE                        (1UL)
#define USB_FALSE                       (0UL)
#define USB_YES                         (1UL)
#define USB_NO                          (0UL)

/* FIFO read / write result */

#define USB_FIFOERROR                       (USB_ERROR)   /* FIFO not ready */
#define USB_WRITEEND                        (0x0000u)     /* End of write (but packet may not be outputting) */
#define USB_WRITESHRT                       (0x0001u)     /* End of write (send short packet) */
#define USB_WRITING                         (0x0002u)     /* Write continues */
#define USB_READEND                         (0x0000u)     /* End of read */
#define USB_READSHRT                        (0x0001u)     /* Insufficient (receive short packet) */
#define USB_READING                         (0x0002u)     /* Read continues */
#define USB_READOVER                        (0x0003u)     /* Buffer size over */

/* Pipe define table end code */

#define USB_PDTBLEND                        (0xffffu) /* End of table */

/* Transfer status Type */

#define USB_CTRL_END                        (0u)
#define USB_DATA_NONE                       (1u)
#define USB_DATA_WAIT                       (2u)
#define USB_DATA_OK                         (3u)
#define USB_DATA_SHT                        (4u)
#define USB_DATA_OVR                        (5u)
#define USB_DATA_STALL                      (6u)
#define USB_DATA_ERR                        (7u)
#define USB_DATA_STOP                       (8u)
#define USB_DATA_TMO                        (9u)
#define USB_CTRL_READING                    (17u)
#define USB_CTRL_WRITING                    (18u)
#define USB_DATA_READING                    (19u)
#define USB_DATA_WRITING                    (20u)

/* Utr member (segment) */

#define USB_TRAN_CONT                       (0x00u)
#define USB_TRAN_END                        (0x80u)

/* USB common bit fields for D0 and D1 FIFO select register */

#define RX65N_USB_DFIFOSEL_RCNT         (1U << 15)
#define RX65N_USB_DFIFOSEL_REW          (1U << 14)
#define RX65N_USB_DFIFOSEL_DCLRM        (1U << 13)
#define RX65N_USB_DFIFOSEL_DREQE        (1U << 12)
#define RX65N_USB_DFIFOSEL_MBW_8        (0U << 10)
#define USB_MBW_8                       (RX65N_USB_DFIFOSEL_MBW_8)
#define RX65N_USB_DFIFOSEL_MBW_16       (1U << 10)
#define USB_MBW_16                      (RX65N_USB_DFIFOSEL_MBW_16)
#define USB0_CFIFO_MBW                  (USB_MBW_16)
#define USB0_D0FIFO_MBW                 (USB_MBW_16)
#define USB0_D1FIFO_MBW                 (USB_MBW_16)
#define RX65N_USB_DFIFOSEL_BIGEND       (1U << 8)
#define RX65N_USB_DFIFOSEL_CURPIPE_MASK (0xf)

/*      USB Interrupt Enable Register 0 and its bit fields      */

#define RX65N_USB_INTENB0               ((volatile short *) (RX65N_USB_BASE + 0x0030UL))
#define RX65N_USB_INTENB0_BEMPE         (1U << 10)
#define RX65N_USB_INTENB0_BRDYE         (1U << 8)
#define RX65N_USB_INTENB0_VBSE          (1U << 15)
#define RX65N_USB_INTENB0_RSME          (1U << 14)
#define RX65N_USB_INTENB0_SOFE          (1U << 13)
#define RX65N_USB_INTENB0_DVSE          (1U << 12)
#define RX65N_USB_INTENB0_CTRE          (1U << 11)
#define RX65N_USB_INTENB0_BEMPE         (1U << 10)
#define RX65N_USB_INTENB0_NRDYE         (1U << 9)
#define RX65N_USB_INTENB0_BRDYE         (1U << 8)

/*      USB Interrupt Enable Register 1 and its bit fields      */

#define RX65N_USB_INTENB1               ((volatile short *) (RX65N_USB_BASE + 0x0032UL))
#define RX65N_USB_INTENB1_OVRCRE        (1U << 15)
#define RX65N_USB_INTENB1_BCHGE         (1U << 14)
#define RX65N_USB_INTENB1_DTCHE         (1U << 12)
#define RX65N_USB_INTENB1_ATTCHE        (1U << 11)

#define RX65N_USB_INTENB1_EOFERRE       (1U << 6)
#define RX65N_USB_INTENB1_SIGNE         (1U << 5)
#define RX65N_USB_INTENB1_SACKE         (1U << 4)

/*      BRDY Interrupt Enable Register  */

#define RX65N_USB_BRDYENB               ((volatile short *) (RX65N_USB_BASE + 0x0036UL))

/* Bit fields of pipe selection/ control registers. These bit fields are
 * generic
 */

#define USB_PIPE1         (1)
#define USB_PIPE2         (2)
#define USB_PIPE3         (3)
#define USB_PIPE4         (4)
#define USB_PIPE5         (5)
#define USB_PIPE6         (6)
#define USB_PIPE7         (7)
#define USB_PIPE8         (8)
#define USB_PIPE9         (9)
#define USB_MIN_PIPE_NO   (1u)
#define USB_MAX_PIPE_NO   (9)

/* Details of pipe number for obtaining the pipe */

/* Start Pipe No */

#define USB_MIN_PIPE_NUM        (1u)

/* Max device */
#define USB_MAXPIPE_BULK        (5u)
#define USB_MAXPIPE_ISO         (2u)
#define USB_MAX_PIPE_NUM        (9u)

#define USB_BULK_PIPE_START     (1u)
#define USB_BULK_PIPE_END       (5u)
#define USB_INT_PIPE_START      (6u)
#define USB_INT_PIPE_END        (9u)
#define USB_ISO_PIPE_START      (1u)
#define USB_ISO_PIPE_END        (2u)

/* Endpoint Descriptor  Define */
#define USB_EP_IN                (0x80u)   /* In Endpoint */
#define USB_EP_OUT               (0x00u)   /* Out Endpoint */
#define USB_EP_CTRL              (0x00u)
#define USB_EP_ISO               (0x01u)   /* Isochronous Transfer */
#define USB_EP_BULK              (0x02u)   /* Bulk Transfer */
#define USB_EP_INT               (0x03u)   /* Interrupt Transfer */

#define USB_BITSET(x)            ((uint16_t)((uint16_t)1 << (x)))

/* BRDY Interrupt Enable/Status Register */

#define USB_BRDY9               (0x0200u)   /* b9: PIPE9 */
#define USB_BRDY8               (0x0100u)   /* b8: PIPE8 */
#define USB_BRDY7               (0x0080u)   /* b7: PIPE7 */
#define USB_BRDY6               (0x0040u)   /* b6: PIPE6 */
#define USB_BRDY5               (0x0020u)   /* b5: PIPE5 */
#define USB_BRDY4               (0x0010u)   /* b4: PIPE4 */
#define USB_BRDY3               (0x0008u)   /* b3: PIPE3 */
#define USB_BRDY2               (0x0004u)   /* b2: PIPE2 */
#define USB_BRDY1               (0x0002u)   /* b1: PIPE1 */
#define USB_BRDY0               (0x0001u)   /* b1: PIPE0 */

/* NRDY Interrupt Enable/Status Register */

#define USB_NRDY9               (0x0200u)   /* b9: PIPE9 */
#define USB_NRDY8               (0x0100u)   /* b8: PIPE8 */
#define USB_NRDY7               (0x0080u)   /* b7: PIPE7 */
#define USB_NRDY6               (0x0040u)   /* b6: PIPE6 */
#define USB_NRDY5               (0x0020u)   /* b5: PIPE5 */
#define USB_NRDY4               (0x0010u)   /* b4: PIPE4 */
#define USB_NRDY3               (0x0008u)   /* b3: PIPE3 */
#define USB_NRDY2               (0x0004u)   /* b2: PIPE2 */
#define USB_NRDY1               (0x0002u)   /* b1: PIPE1 */
#define USB_NRDY0               (0x0001u)   /* b1: PIPE0 */

/* BEMP Interrupt Enable/Status Register */

#define USB_BEMP9               (0x0200u)   /* b9: PIPE9 */
#define USB_BEMP8               (0x0100u)   /* b8: PIPE8 */
#define USB_BEMP7               (0x0080u)   /* b7: PIPE7 */
#define USB_BEMP6               (0x0040u)   /* b6: PIPE6 */
#define USB_BEMP5               (0x0020u)   /* b5: PIPE5 */
#define USB_BEMP4               (0x0010u)   /* b4: PIPE4 */
#define USB_BEMP3               (0x0008u)   /* b3: PIPE3 */
#define USB_BEMP2               (0x0004u)   /* b2: PIPE2 */
#define USB_BEMP1               (0x0002u)   /* b1: PIPE1 */
#define USB_BEMP0               (0x0001u)   /* b0: PIPE0 */

/* Control Transfer Stage */

#define USB_IDLEST                          (0u)  /* Idle */
#define USB_SETUPNDC                        (1u)  /* Setup Stage No Data Control */
#define USB_SETUPWR                         (2u)  /* Setup Stage Control Write */
#define USB_SETUPRD                         (3u)  /* Setup Stage Control Read */
#define USB_DATAWR                          (4u)  /* Data Stage Control Write */
#define USB_DATARD                          (5u)  /* Data Stage Control Read */
#define USB_STATUSRD                        (6u)  /* Status stage */
#define USB_STATUSWR                        (7u)  /* Status stage */
#define USB_SETUPWRCNT                      (17u) /* Setup Stage Control Write */
#define USB_SETUPRDCNT                      (18u) /* Setup Stage Control Read */
#define USB_DATAWRCNT                       (19u) /* Data Stage Control Write */
#define USB_DATARDCNT                       (20u) /* Data Stage Control Read */

#define RX65N_USB_PIPE_ALL                  (0x3ff)

/*      USB NRDY Interrupt Enable Register      */

#define RX65N_USB_NRDYENB                       ((volatile short *) (RX65N_USB_BASE + 0x0038UL))

/*      USB BEMP Interrupt Enable Register      */

#define RX65N_USB_BEMPENB                       ((volatile short *) (RX65N_USB_BASE + 0x003aUL))

/*      USB SOF Output Configuration Register and its bit fields        */

#define RX65N_USB_SOFCFG                ((volatile short *) (RX65N_USB_BASE + 0x003cUL))
#define RX65N_USB_SOFCFG_TRNENSEL       (1U << 8)
#define RX65N_USB_SOFCFG_BRDYM          (1U << 6)
#define USB_SUREQ                       (0x4000u)   /* b14: Send USB request  */

#define RX65N_USB_SOFCFG_EDGESTS        (1U << 4)

/*      USB Interrupt Status Register 0 and its bit fields      */

#define RX65N_USB_INTSTS0               ((volatile short *) (RX65N_USB_BASE + 0x0040UL))
#define RX65N_USB_INTSTS0_VBINT         (1U << 15)
#define RX65N_USB_INTSTS0_RESM          (1U << 14)
#define RX65N_USB_INTSTS0_SOFR          (1U << 13)
#define RX65N_USB_INTSTS0_DVST          (1U << 12)
#define USB_DVSQ                        (0x0070u)   /* b6-4: Device state */
#define RX65N_USB_INTSTS0_CTRT          (1U << 11)
#define USB_CTSQ                        (0x0007u)   /* b2-0: Control transfer stage */
#define USB_CS_SQER                     (0x0006u)   /* Sequence error */
#define USB_CS_WRND                     (0x0005u)   /* Ctrl write nodata status stage */
#define USB_CS_WRSS                     (0x0004u)   /* Ctrl write status stage */
#define USB_CS_WRDS                     (0x0003u)   /* Ctrl write data stage */
#define USB_CS_RDSS                     (0x0002u)   /* Ctrl read status stage */
#define USB_CS_RDDS                     (0x0001u)   /* Ctrl read data stage */
#define USB_CS_IDST                     (0x0000u)   /* Idle or setup stage */
#define USB_DS_SPD_CNFG                 (0x0070u)   /* Suspend Configured */
#define USB_DS_SPD_ADDR                 (0x0060u)   /* Suspend Address */
#define USB_DS_SPD_DFLT                 (0x0050u)   /* Suspend Default */
#define USB_DS_SPD_POWR                 (0x0040u)   /* Suspend Powered */
#define USB_DS_SUSP                     (0x0040u)   /* Suspend */
#define USB_DS_CNFG                     (0x0030u)   /* Configured */
#define USB_DS_ADDS                     (0x0020u)   /* Address */
#define USB_DS_DFLT                     (0x0010u)   /* Default */
#define USB_DS_POWR                     (0x0000u)   /* Powered */

#define RX65N_USB_INTSTS0_CTRT          (1U << 11)
#define RX65N_USB_INTSTS0_BEMP          (1U << 10)
#define RX65N_USB_INTSTS0_NRDY          (1U << 9)
#define RX65N_USB_INTSTS0_BRDY          (1U << 8)
#define RX65N_USB_INTSTS0_VBSTS         (1U << 7)
#define RX65N_USB_INTSTS0_VALID         (1U << 3)
#define RX65N_USB_INTSTS0_DVSQ_MASK     (7U << 4)
#define RX65N_USB_INTSTS0_CTSQ_MASK     (7)
#define RX65N_USB_INTSTS0_ALL_CLEAR     (0U)
#define INTSTS0_BIT_VALUES_TO_DETECT    (0x9d00)
#define USB_DATA_STOP                   (8u)
#define USB_MIN_PIPE_NO                 (1u)
#define USB_MAXPIPE_NUM                 (9u)
#define USB_ACLRM                       (0x0200u)
#define USB_PID_BUF                     (0x0001u)   /* BUF */
#define USB_PIPE0                       (0x0u)
#define USB_PBUSY                       (0x0020u)   /* b5: pipe busy */
#define USB_TRENB                       (0x0200u)
#define USB_TRCLR                       (0x0100u)
#define USB_NULL                        (0x0u)
#define USB_RSME                        (0x4000u)
#define USB_RESM                        (0x4000u)   /* b14: Resume interrupt */
#define USB_VALID                       (0x0008u)   /* b3: Setup packet detect flag */
#define USB_BMREQUESTTYPETYPE           (0x0060u)   /* b6-5: Type */
#define USB_CLASS                       (0x0020u)
#define USB_MBW                         (0x0C00u)   /* b10: Maximum bit width for FIFO access */
#define USB0_CFIFO_MBW                  (USB_MBW_16)
#define USB_DATA_ERR                    (7u)
#define USB_DATA_OVR                    (5u)
#define USB_PID                         (0x0003u)   /* b1-0: Response PID */
#define USB_CCPL                        (0x0004u)   /* b2: Enable control transfer complete */
#define USB_BCLR                        (0x4000u)   /* b14: Buffer clear */
#define USB_FRDY                        (0x2000u)   /* b13: FIFO ready */
#define USB_MAXP                        (0x007Fu)   /* b6-0: Maxpacket size of default control pipe */
#define USB_MXPS                        (0x07FFu)   /* b10-0: Maxpacket size */
#define USB_WRITESHRT                   (0x0001u)   /* End of write (send short packet) */
#define USB_WRITING                     (0x0002u)   /* Write continues */
#define USB0_CFIFO8                     (USB0.CFIFO.BYTE.L)
#define USB0_D0FIFO8                    (USB0.D0FIFO.BYTE.L)
#define USB0_D1FIFO8                    (USB0.D1FIFO.BYTE.L)
#define USB0_CFIFO16                    (USB0.CFIFO.WORD)
#define USB0_D0FIFO16                   (USB0.D0FIFO.WORD)
#define USB0_D1FIFO16                   (USB0.D1FIFO.WORD)
#define USB_WRITEEND                    (0x0000u)
#define USB_CTRL_END                    (0u)
#define USB_BREQUEST                    (0xFF00u)
#define USB_BRDY0                       (0x0001u)     /* b1: PIPE0 */
#define USB_READEND                     (0x0000u)     /* End of read */
#define USB_READSHRT                    (0x0001u)     /* Insufficient (receive short packet) */
#define USB_READING                     (0x0002u)     /* Read continues */
#define USB_READOVER                    (0x0003u)     /* Buffer size over */
#define USB_DTLN                        (0x0FFFu)     /* b11-0: FIFO data length */
#define USB_VENDOR                      (0x0040u)
#define USB_WRITE                       (1)
#define USB_QOVR                        (0xd5)
#define USB_DIRFIELD                    (0x0010u)   /* Transfer direction select */
#define USB_DIR_H_OUT                   (0x0010u)
#define USB_BEMP0                       (0x0001u)   /* b0: PIPE0 */
#define BEMPSTS_MASK                    (0x03FFu)   /* BEMPSTS Reserved bit mask */
#define USB_BEMP                        (0x0400u)   /* b10: Buffer empty interrupt */
#define USB_BUF2FIFO                    (0x0010u)   /* Buffer --> FIFO */
#define USB_FIFO2BUF                    (0x0000u)
#define USB_BITSET(x)                   ((uint16_t)((uint16_t)1 << (x)))
#define USB_READ                        (0)
#define USB_DATA_STALL                  (6u)
#define USB_INBUFM                      (0x4000u)   /* b14: IN buffer monitor (Only for PIPE1 to 5) */
#define USB_DATA_NONE                   (1u)
#define USB_DATA_OK                     (3u)
#define USB_DATA_SHT                    (4u)
#define USB_GS_REMOTEWAKEUP             (0x0002u)
#define USB_EPNUMFIELD                  (0x000Fu)   /* Endpoint number select */
#define USB_GS_HALT                     (0x0001u)
#define USB_GS_SELFPOWERD               (1)
#define USB_GS_BUSPOWERD                (0)
#define USB_MAX_EP_NO                   (15u)       /* EP0 EP1 ... EP15 */
#define USB_ENDPOINT_HALT               (0x0000u)
#define USB_OVRN                        (0x8000u)   /* b15: Overrun error */
#define USB_DREQE                       (0x1000u)   /* b12: DREQ output enable */

/*      USB Interrupt Status Register 0 and its bit fields      */

#define RX65N_USB_INTSTS1               ((volatile short *) (RX65N_USB_BASE + 0x0042UL))
#define RX65N_USB_INTSTS1_OVRCRE        (1U << 15)
#define RX65N_USB_INTSTS1_BCHG          (1U << 14)
#define RX65N_USB_INTSTS1_DTCH          (1U << 12)
#define RX65N_USB_INTSTS1_ATTCH         (1U << 11)
#define RX65N_USB_INTSTS1_EOFERR        (1U << 6)
#define RX65N_USB_INTSTS1_SIGN          (1U << 5)
#define RX65N_USB_INTSTS1_SACK          (1U << 4)
#define RX65N_USB_INTSTS1_ALL_CLEAR     (0U)

/*      USB DCP Control Register and its bit fields     */

#define RX65N_USB_DCPCTR                ((volatile short *) (RX65N_USB_BASE + 0x0060UL))
#define RX65N_USB_DCPCTR_BSTS           (1U << 15)
#define RX65N_USB_DCPCTR_SUREQ          (1U << 14)
#define RX65N_USB_DCPCTR_SUREQCLR       (1U << 11)
#define USB_SUREQCLR                    (RX65N_USB_DCPCTR_SUREQCLR)
#define RX65N_USB_DCPCTR_SQCLR          (1U << 8)
#define USB_SQCLR                       (RX65N_USB_DCPCTR_SQCLR)
#define RX65N_USB_DCPCTR_SQSET          (1U << 7)
#define RX65N_USB_DCPCTR_SQMON          (1U << 6)
#define RX65N_USB_DCPCTR_PBUSY          (1U << 5)
#define RX65N_USB_DCPCTR_CCPL           (1U << 2)
#define RX65N_USB_DCPCTR_PID_MASK       (3UL)
#define RX65N_USB_DCPCTR_PIDNAK         (0UL)
#define RX65N_USB_DCPCTR_PIDBUF         (1UL)
#define RX65N_USB_DCPCTR_PIDSTALL       (2UL)
#define RX65N_USB_DCPCTR_PIDSTALL2      (3UL)

/*      USB PIPE 1 to 9 Control Registers       */

#define RX65N_USB_PIPE1CTR              ((volatile short *) (RX65N_USB_BASE + 0x0070UL))
#define RX65N_USB_PIPE2CTR              ((volatile short *) (RX65N_USB_BASE + 0x0072UL))
#define RX65N_USB_PIPE3CTR              ((volatile short *) (RX65N_USB_BASE + 0x0074UL))
#define RX65N_USB_PIPE4CTR              ((volatile short *) (RX65N_USB_BASE + 0x0076UL))
#define RX65N_USB_PIPE5CTR              ((volatile short *) (RX65N_USB_BASE + 0x0078UL))
#define RX65N_USB_PIPE6CTR              ((volatile short *) (RX65N_USB_BASE + 0x007aUL))
#define RX65N_USB_PIPE7CTR              ((volatile short *) (RX65N_USB_BASE + 0x007cUL))
#define RX65N_USB_PIPE8CTR              ((volatile short *) (RX65N_USB_BASE + 0x007eUL))
#define RX65N_USB_PIPE9CTR              ((volatile short *) (RX65N_USB_BASE + 0x0080UL))

/* USB Pipe 1 to 9 control register bit fields */

#define RX65N_USB_PIPECTR_BSTS          (1U << 15)
#define RX65N_USB_PIPECTR_INBUFM        (1U << 14)
#define RX65N_USB_PIPECTR_ATREPM        (1U << 10)
#define RX65N_USB_PIPECTR_ACLRM         (1U << 9)
#define RX65N_USB_PIPECTR_SQCLR         (1U << 8)
#define RX65N_USB_PIPECTR_SQSET         (1U << 7)
#define RX65N_USB_PIPECTR_SQMON         (1U << 6)
#define RX65N_USB_PIPECTR_PBUSY         (1U << 5)
#define RX65N_USB_PIPECTR_PID_MASK      (3)
#define RX65N_USB_PIPECTR_PIDNAK        (0)
#define RX65N_USB_PIPECTR_PIDBUF        (1)
#define RX65N_USB_PIPECTR_PIDSTALL      (2)
#define RX65N_USB_PIPECTR_PIDSTALL2     (3)
#define RX65N_USB_PIPECTR_DATA1         (1U << 7)
#define RX65N_USB_PIPECTR_DATA0         (1U << 8)

/* USB PIPE 1 to 5 (Transaction Counter Enable) and
 * (Transaction Counter Register) Registers
 */

#define RX65N_USB_PIPE1TRE              ((volatile short *) (RX65N_USB_BASE + 0x0090UL))
#define RX65N_USB_PIPE1TRN              ((volatile short *) (RX65N_USB_BASE + 0x0092UL))
#define RX65N_USB_PIPE2TRE              ((volatile short *) (RX65N_USB_BASE + 0x0094UL))
#define RX65N_USB_PIPE2TRN              ((volatile short *) (RX65N_USB_BASE + 0x0096UL))
#define RX65N_USB_PIPE3TRE              ((volatile short *) (RX65N_USB_BASE + 0x0098UL))
#define RX65N_USB_PIPE3TRN              ((volatile short *) (RX65N_USB_BASE + 0x009aUL))
#define RX65N_USB_PIPE4TRE              ((volatile short *) (RX65N_USB_BASE + 0x009cUL))
#define RX65N_USB_PIPE4TRN              ((volatile short *) (RX65N_USB_BASE + 0x009eUL))
#define RX65N_USB_PIPE5TRE              ((volatile short *) (RX65N_USB_BASE + 0x00a0UL))
#define RX65N_USB_PIPE5TRN              ((volatile short *) (RX65N_USB_BASE + 0x00a2UL))

/* USB PIPE 1 to 5 Transaction Counter Enable register bit fields       */

#define RX65N_USB_PIPETRE_TRENB         (1U << 9)
#define RX65N_USB_PIPETRE_TRCLR         (1U << 8)

/*      USB Device Address 0 to 5 Configuration Register */

#define RX65N_USB_DEVADD0               ((volatile short *) (RX65N_USB_BASE + 0x00d0UL))
#define RX65N_USB_DEVADD1               ((volatile short *) (RX65N_USB_BASE + 0x00d2UL))
#define RX65N_USB_DEVADD2               ((volatile short *) (RX65N_USB_BASE + 0x00d4UL))
#define RX65N_USB_DEVADD3               ((volatile short *) (RX65N_USB_BASE + 0x00d6UL))
#define RX65N_USB_DEVADD4               ((volatile short *) (RX65N_USB_BASE + 0x00d8UL))
#define RX65N_USB_DEVADD5               ((volatile short *) (RX65N_USB_BASE + 0x00daUL))
#define RX65N_USB_DEVSPD                (3 << 6)

#define USB_MAXDEVADDR                  (5u)
#define USB_DEVICE_0                    (0x0000u) /* Device address 0 */
#define USB_DEVICE_1                    (0x1000u) /* Device address 1 */
#define USB_DEVICE_2                    (0x2000u) /* Device address 2 */
#define USB_DEVICE_3                    (0x3000u) /* Device address 3 */
#define USB_DEVICE_4                    (0x4000u) /* Device address 4 */
#define USB_DEVICE_5                    (0x5000u) /* Device address 5 */
#define USB_DEVICE_6                    (0x6000u) /* Device address 6 */
#define USB_DEVICE_7                    (0x7000u) /* Device address 7 */
#define USB_DEVICE_8                    (0x8000u) /* Device address 8 */
#define USB_DEVICE_9                    (0x9000u) /* Device address 9 */
#define USB_DEVICE_A                    (0xa000u) /* Device address A */
#define USB_NODEVICE                    (0xf000u) /* No device */
#define USB_DEVADDRBIT                  (12u)
#define USB_DEFPACKET                   (0x0040)

/* Device Address bit fields    */

#define RX65N_USB_DEVADD_SPEED_LOW      (1U << 6)
#define RX65N_USB_DEVADD_SPEED_FULL     (2U << 6)
#define RX65N_USB_DEVADD_SPEED_HIGH     (3U << 6)

/*      USB PHY Cross Point Adjustment Register and its bit fields      */

#define RX65N_USB_PHYSLEW               ((volatile int *) (RX65N_USB_BASE + 0x00f0UL))

/* PHY Cross Point Adjustment, note that Hardware Manual to be
 * updated(0xe->0x5)
 */
#define RX65N_USB_PHYSLEW_SLEW_SLEWR00  (1U << 0)
#define RX65N_USB_PHYSLEW_SLEW_SLEWR01  (1U << 1)
#define RX65N_USB_PHYSLEW_SLEW_SLEWF00  (1U << 2)
#define RX65N_USB_PHYSLEW_SLEW_SLEWF01  (1U << 3)

/* USB Deep Standby USB Transceiver Control/Pin Monitoring Register */

#define RX65N_USB_DPUSR0R               ((volatile int *)(RX65N_USB_BASE + 0x0400UL))

/*      USB Deep Standby USB Suspend/Resume Interrupt Register  */

#define RX65N_USB_DPUSR1R               ((volatile int *)(RX65N_USB_BASE + 0x0404UL))

#define RX65N_USB_BRDYENB               ((volatile short *) (RX65N_USB_BASE + 0x0036UL))
#define RX65N_USB_NRDYENB               ((volatile short *) (RX65N_USB_BASE + 0x0038UL))

/*      USB BEMP Interrupt Enable Register      */

#define RX65N_USB_BEMPENB               ((volatile short *) (RX65N_USB_BASE + 0x003aUL))

/*      USB Frame Number Register and its bit fields    */

#define RX65N_USB_FRMNUM                ((volatile short *) (RX65N_USB_BASE + 0x004cUL))
#define RX65N_USB_FRMNUM_OVRN           (1U << 15)
#define RX65N_USB_FRMNUM_CRCE           (1U << 14)
#define RX65N_USB_FRMNUM_FRNM_MASK      (0x7ffU)

/*      USB Device State Change Register and its bit fields     */

#define RX65N_USB_DVCHGR                    ((volatile short *) (RX65N_USB_BASE + 0x004eUL))

#define RX65N_USB_PIPESEL                   ((volatile short *) (RX65N_USB_BASE + 0x0064UL))
#define RX65N_USB_PIPESEL_NO_PIPE           (0x000fUL)

#define RX65N_USB_PIPECFG                   ((volatile short *) (RX65N_USB_BASE + 0x0068UL))
#define RX65N_USB_PIPECFG_TYPE_MASK         (0xc000)
#define RX65N_USB_PIPECFG_TYPE_BIT_USED     (0)
#define RX65N_USB_PIPECFG_TYPE_BULK         (1U << 14)
#define RX65N_USB_PIPECFG_TYPE_INTERRUPT    (2U << 14)
#define RX65N_USB_PIPECFG_TYPE_ISOCHRONOUS  (3U << 14)
#define RX65N_USB_PIPECFG_BFRE              (1U << 10)
#define RX65N_USB_PIPECFG_DBLB              (1U << 9)
#define RX65N_USB_PIPECFG_SHTNAK            (1U << 7)
#define RX65N_USB_PIPECFG_DIR               (1U << 4)
#define RX65N_USB_PIPECFG_EPNUM_MASK        (0xfU)

#define RX65N_USB_PIPEMAXP                  ((volatile short *) (RX65N_USB_BASE + 0x006cUL))
#define RX65N_USB_PIPEMAXP_DEVSELMASK       (0xfU << 12)
#define RX65N_USB_PIPEMAXP_DEVSEL_SHIFT     (12U)
#define RX65N_USB_PIPEMAXP_MXPSMASK         (0x1ff)

#define RX65N_USB_PIPEPERI                  ((volatile short *) (RX65N_USB_BASE + 0x006eUL))

/*      USB BRDY Interrupt Status Register      */

#define RX65N_USB_BRDYSTS               ((volatile short *) (RX65N_USB_BASE + 0x0046UL))

/*      USB NRDY Interrupt Status Register      */

#define RX65N_USB_NRDYSTS               ((volatile short *) (RX65N_USB_BASE + 0x0048UL))

/*      USB BEMP Interrupt Status Register      */

#define RX65N_USB_BEMPSTS               ((volatile short *) (RX65N_USB_BASE + 0x004aUL))

#define RX65N_USB_DVSTCTR0              ((volatile short *) (RX65N_USB_BASE + 0x0008UL))
#define USB_HSMODE                      (0x0003u)   /* Hi-Speed mode */
#define USB_FSMODE                      (0x0002u)   /* Full-Speed mode */
#define USB_LSMODE                      (0x0001u)   /* Low-Speed mode */
#define USB_HSPROC                      (0x0004u)   /* HS handshake processing */
#define USB_HSCONNECT                   (0x00C0u)   /* Hi-Speed connect */
#define USB_FSCONNECT                   (0x0080u)   /* Full-Speed connect */
#define USB_LSCONNECT                   (0x0040u)   /* Low-Speed connect */
#define USB_NOCONNECT                   (0x0000u)

#define RX65N_USB_DCPCFG                ((volatile short *) (RX65N_USB_BASE + 0x005cUL))
#define RX65N_USB_DCPCFG_DIR            (1U << 4)

#define RX65N_USB_DCPMAXP               ((volatile short *) (RX65N_USB_BASE + 0x005eUL))
#define RX65N_USB_DCPMAXP_DEVADDR_SHIFT (12U)
#define RX65N_USB_DCPMAXP_DEVADDR_MASK  (0xf000U)
#define RX65N_USB_DCPMAXP_MXPS_MASK     (0x007fU)

#define USB_DCPMAXP                     (64u)

#define RX65N_USB_USBREQ                ((volatile short *) (RX65N_USB_BASE + 0x0054UL))

/*      USB Request Value Register      */

#define RX65N_USB_USBVAL                ((volatile short *) (RX65N_USB_BASE + 0x0056UL))

/*      USB Request Index Register      */

#define RX65N_USB_USBINDX               ((volatile short *) (RX65N_USB_BASE + 0x0058UL))

/*      USB Request Length Register     */

#define RX65N_USB_USBLENG               ((volatile short *) (RX65N_USB_BASE + 0x005aUL))

/* Endpoint Descriptor  Define */
#define USB_EP_IN                       (0x80u)   /* In Endpoint */
#define USB_EP_OUT                      (0x00u)   /* Out Endpoint */
#define USB_EP_ISO                      (0x01u)   /* Isochronous Transfer */
#define USB_EP_BULK                     (0x02u)   /* Bulk Transfer */
#define USB_EP_INT                      (0x03u)   /* Interrupt Transfer */

#define USB_PIPE_DIR_IN                 (0u)
#define USB_PIPE_DIR_OUT                (1u)
#define USB_PIPE_DIR_MAX                (2u)

#define USB_CFG_PCDC_BULK_IN            (USB_PIPE1)
#define USB_CFG_PCDC_BULK_OUT           (USB_PIPE2)
#define USB_CFG_PCDC_INT_IN             (USB_PIPE6)

/* USB pipe number */
#define USB_PIPE0                       (0x0u)

/* Pipe configuration table define */
#define USB_EPL                         (6u)        /* Pipe configuration table length */
#define USB_TYPFIELD                    (0xC000u)   /* Transfer type */
#define USB_PERIODIC                    (0x8000u)   /* Periodic pipe */
#define USB_TYPFIELD_ISO                (0xC000u)   /* Isochronous */
#define USB_TYPFIELD_INT                (0x8000u)   /* Interrupt */
#define USB_TYPFIELD_BULK               (0x4000u)   /* Bulk */
#define USB_NOUSE                       (0x0000u)   /* Not configuration */
#define USB_BFREFIELD                   (0x0400u)   /* Buffer ready interrupt mode select */
#define USB_BFREON                      (0x0400u)
#define USB_BFREOFF                     (0x0000u)
#define USB_DBLBFIELD                   (0x0200u)   /* Double buffer mode select */
#define USB_CFG_DBLBON                  (0x0200u)
#define USB_CFG_DBLBOFF                 (0x0000u)
#define USB_CNTMDFIELD                  (0x0100u)   /* Continuous transfer mode select */
#define USB_CFG_CNTMDON                 (0x0100u)
#define USB_CFG_CNTMDOFF                (0x0000u)
#define USB_CFG_DBLB                    (USB_CFG_DBLBON)
#define USB_DIR_P_IN                    (0x0010u)   /* PERI IN */
#define USB_DIR_H_IN                    (0x0000u)
#define USB_SHTNAKFIELD                 (0x0080u)   /* Transfer end NAK */
#define USB_DIR_P_OUT                   (0x0000u)   /* PERI OUT */
#define USB_BRDY                        (0x0100u)   /* b8: Buffer ready interrupt */
#define BRDYSTS_MASK                    (0x03FFu)   /* BRDYSTS Reserved bit mask */
#define RX65N_USB_INTSTS0_NRDY          (1U << 9)
#define RX65N_PIPENUM_WRITE             (1)
#define RX65N_USB_MAXP                  (64)
#define RX65N_USBI0_SOURCE              (0x3eu)
#define RX65N_USBI0_PRIORITY            (0x0f)
#define RX65N_PHYSLEW_VALUE             (0x5)

#define RX65N_USB_PFKUSB_ENABLED        (1U << 4)
#define RX65N_USB_PFKUSB_MODE_HOST      (1)
#define RX65N_USB_INTERRUPT_STATUS_MASK (0x3ffU)

/* Supported USBMCLK frequency for S7G2 and S5D9.  */

#define RX65N_USB_MAIN_OSC_24MHz                     (24000000U)
#define RX65N_USB_MAIN_OSC_20MHz                     (20000000U)
#define RX65N_USB_MAIN_OSC_12MHz                     (12000000U)

/* Bit fields */

#define RX65N_USB_SYSSTS0_LNST_J_STATE_FS            (1U)
#define RX65N_USB_SYSSTS0_LNST_J_STATE_LS            (2U)
#define RX65N_USB_PLLSTA_PLLLOCK                     (1U << 0)
#define RX65N_USB_PHYSET_HSEB                        (1U << 15)
#define RX65N_USB_PHYSET_REPSTART                    (1U << 11)
#define RX65N_USB_PHYSET_REPSEL                      (1U << 8)
#define RX65N_USB_PHYSET_CLKSEL_1                    (1U << 5)
#define RX65N_USB_PHYSET_CLKSEL_0                    (1U << 4)
#define RX65N_USB_PHYSET_CDPEN                       (1U << 3)
#define RX65N_USB_PHYSET_PLLRESET                    (1U << 1)
#define RX65N_USB_PHYSET_DIRPD                       (1U << 0)
#define RX65N_USB_PIPEBUF_SIZEMASK                   (0x1fU << 10)
#define RX65N_USB_PIPEBUF_BUFNMBMASK                 (0xffU << 10)
#define RX65N_USB_PIPEBUF_SHIFT                      (10U)

/* Possibly below are used for differentiating Control/ D0 or D1 pipe... */

#define RX65N_USB_FIFO_D0                            (0UL)
#define RX65N_USB_FIFO_D1                            (1UL)
#define RX65N_USB_FIFO_C                             (2UL)
#define RX65N_USB_DEVADD_UPPHUB_SHIFT                (11U)
#define RX65N_USB_DEVADD_HUBPORT_SHIFT               (8U)
#define RX65N_USB_USBMC_VDCEN                        (1U << 7)

/* Define Synergy HCOR command/status bitmaps.  */

#define RX65N_USB_DCP                                (0)
#define RX65N_USB_DCPCTR_DATA1                       (1U << 7)
#define RX65N_USB_DCPCTR_DATA0                       (1U << 8)

/* Define Synergy fifo definition.  */

#define RX65N_USB_PIPE0_SIZE                         (256)
#define RX65N_USB_PIPE_NB_BUFFERS                    (64)

/* Define Synergy static definition.  */

#define RX65N_USB_AVAILABLE_BANDWIDTH                (2304UL)

/* The macro above is used for checking the available bandwidth for periodic
 * transfers(Isochronous and Interrupt)
 * Maximum bandwidth is calculated as
 * {2048byes(2x ISO PIPEs) + 256bytes(4x INT PIPEs)} for high-speed operation
 */
#define RX65N_USB_INIT_DELAY                         (1000)
#define RX65N_USB_RESET_RETRY                        (1000)
#define RX65N_USB_RESET_DELAY                        (10)
#define RX65N_USB_PORT_RESET_RETRY                   (50)
#define RX65N_USB_FORCE_PORT_RESET_RETRY             (50)
#define RX65N_USB_FORCE_PORT_RESET_DELAY             (1)
#define RX65N_USB_CHECK_PORT_RESET_RETRY             (500)
#define RX65N_USB_PORT_RESET_DELAY                   (300)
#define RX65N_USB_PORT_RESET_RECOVERY_DELAY          (100)

/* Define Synergy initialization values.  */

#define RX65N_USB_COMMAND_STATUS_RESET               (0)
#define RX65N_USB_INIT_RESET_DELAY                   (10)
#define RX65N_USB_MAX_BUF_SIZE                       (64)
#define RX65N_USB_BUF_BLOCK_SIZE                     (64)
#define RX65N_USB_MAX_BUF_SIZE_PIPE1_to_2_FS         (256)
#define RX65N_USB_MAX_BUF_SIZE_PIPE3_to_9_FS         (64)
#define RX65N_USB_MAX_BUF_SIZE_PIPE1_to_2_HS         (1024)
#define RX65N_USB_MAX_BUF_SIZE_PIPE3_to_5_HS         (512)
#define RX65N_USB_MAX_BUF_SIZE_PIPE6_to_9_HS         (64)
#define RX65N_USB_MAX_BUF_NUM                        (135)
#define RX65N_USB_PIPE1_BUF_START_NUM                (8)

/* Define Synergy FIFO write completion code.  */

#define RX65N_USB_FIFO_WRITING                       (2)
#define RX65N_USB_FIFO_WRITE_END                     (3)
#define RX65N_USB_FIFO_WRITE_SHORT                   (4)
#define RX65N_USB_FIFO_WRITE_DMA                     (5)
#define RX65N_USB_FIFO_WRITE_ERROR                   (6)

/* Define Synergy FIFO read completion code.  */

#define RX65N_USB_FIFO_READING                       (2)
#define RX65N_USB_FIFO_READ_END                      (3)
#define RX65N_USB_FIFO_READ_SHORT                    (4)
#define RX65N_USB_FIFO_READ_DMA                      (5)
#define RX65N_USB_FIFO_READ_ERROR                    (6)
#define RX65N_USB_FIFO_READ_OVER                     (7)
#define RX65N_USB_ED_BRDY                            (0x00000001U)
#define RX65N_USB_ED_NRDY                            (0x00000002U)
#define RX65N_USB_ED_BEMP                            (0x00000004U)
#define RX65N_USB_ED_EOFERR                          (0x00000010U)
#define RX65N_USB_ED_SIGN                            (0x00000020U)
#define RX65N_USB_ED_SACK                            (0x00000040U)
#define RX65N_USB_ED_TIMEOUT                         (0x00000080U)
#define RX65N_USB_LPSTS_SUSPENDM                     (1U << 14)

/* Define Synergy Root hub states.  */

#define RX65N_USB_PORT_ENABLED                       (1)
#define RX65N_USB_PORT_DISABLED                      (0)
#define RX65N_USB_PORT_INEOFERR                      (3)

#define RX65N_USB_FRMNUM_VAL                         (0x1111111111)

#define USB_INT_BRDY                                 (0x0001u)
#define USB_BMREQUESTTYPERECIP                       (0x001Fu)   /* b4-0: Recipient */
#define HUB_PORT1                                    (1)
#define HUB_PORT2                                    (2)
#define HUB_PORT3                                    (3)
#define HUB_PORT4                                    (4)

/* StandBy RAM Address */

#define RX65N_SBRAM_BASE  0x000a4000

/* Start of RSPI interface related definitions */

#if defined(CONFIG_SPI) || defined(CONFIG_SPI_DRIVER)
  #define HAVE_RSPI_DRIVER            1
#endif

#define RX65N_RSPI0_BASE              (0x000D0100)
#define RX65N_RSPI1_BASE              (0x000D0140)
#define RX65N_RSPI2_BASE              (0x000D0300)

/* Tx and Rx vector number */

#define RX65N_RSPI0_RXVECT            (38)
#define RX65N_RSPI0_TXVECT            (39)
#define RX65N_RSPI1_RXVECT            (40)
#define RX65N_RSPI1_TXVECT            (41)
#define RX65N_RSPI2_RXVECT            (108)
#define RX65N_RSPI2_TXVECT            (109)

#define RX65N_PCLK_FREQUENCY RX_PCLKA

/* RSPI Register offsets */

#define RX65N_RSPI_SPCR_OFFSET        (0x0000)   /* RSPI Control Register */
#define RX65N_RSPI_SSLP_OFFSET        (0x0001)   /* RSPI Slave Select Polarity Register */
#define RX65N_RSPI_SPPCR_OFFSET       (0x0002)   /* RSPI Pin Control Register */
#define RX65N_RSPI_SPSR_OFFSET        (0x0003)   /* RSPI Status Register */
#define RX65N_RSPI_SPDR_OFFSET        (0x0004)   /* RSPI Data Register */
#define RX65N_RSPI_SPSCR_OFFSET       (0x0008)   /* RSPI Sequence Control Register */
#define RX65N_RSPI_SPSSR_OFFSET       (0x0009)   /* RSPI Sequence Status Register */
#define RX65N_RSPI_SPBR_OFFSET        (0x000A)   /* RSPI Bit Rate Register */
#define RX65N_RSPI_SPDCR_OFFSET       (0x000B)   /* RSPI Data Control Register */
#define RX65N_RSPI_SPCKD_OFFSET       (0x000C)   /* RSPI Clock Delay Register */
#define RX65N_RSPI_SSLND_OFFSET       (0x000D)   /* RSPI Slave Select Negation Delay Register */
#define RX65N_RSPI_SPND_OFFSET        (0x000E)   /* RSPI Next-Access Delay Register */
#define RX65N_RSPI_SPCR2_OFFSET       (0x000F)   /* RSPI Control Register 2 */
#define RX65N_RSPI_SPCMD0_OFFSET      (0x0010)   /* RSPI Command Registers 0 */
#define RX65N_RSPI_SPCMD1_OFFSET      (0x0012)   /* RSPI Command Registers 1 */
#define RX65N_RSPI_SPCMD2_OFFSET      (0x0014)   /* RSPI Command Registers 2 */
#define RX65N_RSPI_SPCMD3_OFFSET      (0x0016)   /* RSPI Command Registers 3 */
#define RX65N_RSPI_SPCMD4_OFFSET      (0x0018)   /* RSPI Command Registers 4 */
#define RX65N_RSPI_SPCMD5_OFFSET      (0x001A)   /* RSPI Command Registers 5 */
#define RX65N_RSPI_SPCMD6_OFFSET      (0x001C)   /* RSPI Command Registers 6 */
#define RX65N_RSPI_SPCMD7_OFFSET      (0x001E)   /* RSPI Command Registers 7 */
#define RX65N_RSPI_SPDCR2_OFFSET      (0x0020)   /* RSPI Data Control Register 2 */

/* RSPI Control Register bits */

#define RSPI_SPCR_SMPS                (1 << 0) /* RSPI Mode Select */
#define RSPI_SPCR_TXMD                (1 << 1) /* Communications Operating Mode Select */
#define RSPI_SPCR_MODFEN              (1 << 2) /* Mode Fault Error Detection Enable */
#define RSPI_SPCR_MSTR                (1 << 3) /* RSPI Master/Slave Mode Select */
#define RSPI_SPCR_SPEIE               (1 << 4) /* RSPI Error Interrupt Enable */
#define RSPI_SPCR_SPTIE               (1 << 5) /* Transmit Buffer Empty Interrupt Enable */
#define RSPI_SPCR_SPE                 (1 << 6) /* RSPI Function Enable */
#define RSPI_SPCR_SPRIE               (1 << 7) /* RSPI Receive Buffer Full Interrupt Enable */

/* RSPI Slave Select Polarity Register bits */

#define RSPI_SSLP_SSL0P               (1 << 0)  /* SSL0 Signal Polarity Setting */
#define RSPI_SSLP_SSL1P               (1 << 1)  /* SSL0 Signal Polarity Setting */
#define RSPI_SSLP_SSL2P               (1 << 2)  /* SSL0 Signal Polarity Setting */
#define RSPI_SSLP_SSL3P               (1 << 3)  /* SSL0 Signal Polarity Setting */

/* RSPI Pin Control Register bits */

#define RSPI_SPPCR_SPLP               (1 << 0)  /* 0: Normal mode. 1: Loopback mode (reversed transmit data = receive). */
#define RSPI_SPPCR_SPLP2              (1 << 1)  /* 0: Normal mode. 1: Loopback mode (transmit data = receive data). */
#define RSPI_SPPCR_MOIFV              (1 << 4)  /* 0: MOSI pin idles low. 1: MOSI pin idles high. */
#define RSPI_SPPCR_MOIFE              (1 << 5)  /* 0: MOSI pin idles at final previous data. 1: MOSI pin idles at MOIFV. */

/* RSPI status register bits */

#define RSPI_SPSR_OVRF                (1 << 0)  /* Overrun Error Flag */
#define RSPI_SPSR_IDLNF               (1 << 1)  /* RSPI Idle Flag */
#define RSPI_SPSR_MODF                (1 << 2)  /* Mode Fault Error Flag */
#define RSPI_SPSR_PERF                (1 << 3)  /* Parity Error Flag */
#define RSPI_SPSR_UDRF                (1 << 4)  /* Underrun Error Flag */
#define RSPI_SPSR_SPTEF               (1 << 5)  /* Transmit Buffer Empty Flag */
#define RSPI_SPSR_SPRF                (1 << 7)  /* Receive Buffer Full Flag */
#define RSPI_SPSR_MODF_UDRF_MASK      (0xAB)    /* Protect reserved bits. */

/* RSPI Data Control Register bit and mask */

#define RSPI_SPDCR_MASK               (0x73)   /* Mask for SPDCR*/
#define RSPI_SPDCR_SPFC0              (1 << 0) /* b0 used for number of frame calculation with b1 */
#define RSPI_SPDCR_SPFC1              (1 << 1) /* b1 used for number of frame calculation with b0 */
#define RSPI_SPDCR_SPRDTD             (1 << 4) /* RSPI Receive/Transmit Data Select*/
#define RSPI_SPDCR_SPLW               (1 << 5) /* RSPI Longword Access Word Access Specification */
#define RSPI_SPDCR_SPBYT              (1 << 6) /* RSPI Byte Access Specification*/
#define RSPI_SPDCR_SPFC_MASK          (0x3)    /* SPFC mask */

/* RSPI command register bits */

#define RSPI_SPCMD_MASK               (0xFF << 0) /* RSPI Command Register mask */
#define RSPI_SPCMD_PHA                (1 << 0)    /* RSPCK Phase Setting */
#define RSPI_SPCMD_POL                (1 << 1)    /* RSPCK Polarity Setting */
#define RSPI_SPCMD_BRDV0              (1 << 2)    /* Bit Rate Division Setting bit b2 */
#define RSPI_SPCMD_BRDV1              (1 << 3)    /* Bit Rate Division Setting bit b3 */
#define RSPI_SPCMD_SSLA0              (1 << 4)    /* SSL Signal Assertion Setting bit b4 */
#define RSPI_SPCMD_SSLA1              (1 << 5)    /* SSL Signal Assertion Setting bit b5 */
#define RSPI_SPCMD_SSLA2              (1 << 6)    /* SSL Signal Assertion Setting bit b6 */
#define RSPI_SPCMD_SSLKP              (1 << 7)    /* SSL Signal Level Keeping bit b7 */
#define RSPI_SPCMD_SPB0               (1 << 8)    /* RSPI Data Length Setting bit b8 */
#define RSPI_SPCMD_SPB1               (1 << 9)    /* RSPI Data Length Setting bit b9 */
#define RSPI_SPCMD_SPB2               (1 << 10)   /* RSPI Data Length Setting bit b10 */
#define RSPI_SPCMD_SPB3               (1 << 11)   /* RSPI Data Length Setting bit b11 */
#define RSPI_SPCMD_LSBF               (1 << 12)   /* RSPI LSB First bit b12 */
#define RSPI_SPCMD_SPNDEN             (1 << 13)   /* RSPI Next-Access Delay Enable bit */
#define RSPI_SPCMD_SLNDEN             (1 << 14)   /* SSL Negation Delay Setting Enable bit */
#define RSPI_SPCMD_SCKDEN             (1 << 15)   /* SCKDEN RSPCK Delay Setting Enable bit */
#define RSPI_SPCMD_BRDV_MASK          (3 << 2)    /* Bit Rate Division Setting mask */
#define RSPI_SPCMD_SPB_MASK           (15 << 8)   /* RSPI Data Length Setting */

/* RSPI clock delay register bit */

#define RSPI_SPCKD_MASK               (7 << 0)  /* RSPCK Delay Setting mask */
#define RSPI_SPCKD_SCKDL0             (1 << 0)  /* SCKDL0 bit */
#define RSPI_SPCKD_SCKDL1             (1 << 1)  /* SCKDL1 bit */
#define RSPI_SPCKD_SCKDL2             (1 << 2)  /* SCKDL2 bit */

/* RSPI Slave Select Negation Delay Register bit */

#define RSPI_SSLND_MASK               (7 << 0)  /* SSL Negation Delay Setting mask */
#define RSPI_SSLND_SLNDL0             (1 << 0)  /* SLNDL0 bit */
#define RSPI_SSLND_SLNDL1             (1 << 1)  /* SLNDL1 bit */
#define RSPI_SSLND_SLNDL2             (1 << 2)  /* SLNDL2 bit */

/* RSPI clock delay register bit */

#define RSPI_SPND_MASK                (7 << 0)  /* RSPI Next-Access Delay mask */
#define RSPI_SPND_SPNDL0              (1 << 0)  /* SPNDL0 bit */
#define RSPI_SPND_SPNDL1              (1 << 1)  /* SPNDL1 bit */
#define RSPI_SPND_SPNDL2              (1 << 2)  /* SPNDL2 bit */

/* RSPI RSPI Control Register 2 bit */

#define RSPI_SPCR2_MASK               (0x1F << 0)  /* RSPI Control Register 2 mask */
#define RSPI_SPCR2_SPPE               (1 << 0)     /* Parity Enable bit */
#define RSPI_SPCR2_SPOE               (1 << 1)     /* Parity Mode bit */
#define RSPI_SPCR2_SPIIE              (1 << 2)     /* RSPI Idle Interrupt Enable */
#define RSPI_SPCR2_PTE                (1 << 3)     /* Parity Self-Diagnosis bit */
#define RSPI_SPCR2_SCKASE             (1 << 4)     /* RSPCK Auto-Stop Function Enable bit */

/* RSPI Sequence Control Register 2 bit */

#define RSPI_SPSCR_MASK               (7 << 0) /* RSPI Sequence Control Register mask */
#define RSPI_SPSCR_SPSLN0             (1 << 0) /* SPSLN0 bit */
#define RSPI_SPSCR_SPSLN1             (1 << 1) /* SPSLN1 bit */
#define RSPI_SPSCR_SPSLN2             (1 << 2) /* SPSLN2 bit */

/* Set RSPI data control register 2 bit */

#define RSPI_SPDCR2_BYSW              (1 << 0) /* RSPI Byte Swap */

/* End of RSPI interface related definitions */

/* RIIC related definitions */

#if defined(CONFIG_I2C) || defined(CONFIG_I2C_DRIVER)
  #define HAVE_RIIC_DRIVER       1
#endif

/* RIIC Channel Base Address definitions */

#define RX65N_RIIC0_BASE          (uint32_t)&RIIC0
#define RX65N_RIIC1_BASE          (uint32_t)&RIIC1
#define RX65N_RIIC2_BASE          (uint32_t)&RIIC2

/* RIIC Register Offset definitions */

#define RX65N_RIIC_ICCR1_OFFSET   (0x0000)
#define RX65N_RIIC_ICCR2_OFFSET   (0x0001)
#define RX65N_RIIC_ICMR1_OFFSET   (0x0002)
#define RX65N_RIIC_ICMR2_OFFSET   (0x0003)
#define RX65N_RIIC_ICMR3_OFFSET   (0x0004)
#define RX65N_RIIC_ICFER_OFFSET   (0x0005)
#define RX65N_RIIC_ICSER_OFFSET   (0x0006)
#define RX65N_RIIC_ICIER_OFFSET   (0x0007)
#define RX65N_RIIC_ICSR1_OFFSET   (0x0008)
#define RX65N_RIIC_ICSR2_OFFSET   (0x0009)
#define RX65N_RIIC_SARL0_OFFSET   (0x000a)
#define RX65N_RIIC_SARU0_OFFSET   (0x000b)
#define RX65N_RIIC_SARL1_OFFSET   (0x000c)
#define RX65N_RIIC_SARU1_OFFSET   (0x000d)
#define RX65N_RIIC_SARL2_OFFSET   (0x000e)
#define RX65N_RIIC_SARU2_OFFSET   (0x000f)
#define RX65N_RIIC_ICBRL_OFFSET   (0x0010)
#define RX65N_RIIC_ICBRH_OFFSET   (0x0011)
#define RX65N_RIIC_ICDRT_OFFSET   (0x0012)
#define RX65N_RIIC_ICDRR_OFFSET   (0x0013)

/* RIIC register address definitions */

#define RX65N_RIIC0_ICCR1         (RX65N_RIIC0_BASE + RX65N_RIIC_ICCR1_OFFSET)
#define RX65N_RIIC0_ICCR2         (RX65N_RIIC0_BASE + RX65N_RIIC_ICCR2_OFFSET)
#define RX65N_RIIC0_ICMR1         (RX65N_RIIC0_BASE + RX65N_RIIC_ICMR1_OFFSET)
#define RX65N_RIIC0_ICMR2         (RX65N_RIIC0_BASE + RX65N_RIIC_ICMR2_OFFSET)
#define RX65N_RIIC0_ICMR3         (RX65N_RIIC0_BASE + RX65N_RIIC_ICMR3_OFFSET)
#define RX65N_RIIC0_ICFER         (RX65N_RIIC0_BASE + RX65N_RIIC_ICFER_OFFSET)
#define RX65N_RIIC0_ICSER         (RX65N_RIIC0_BASE + RX65N_RIIC_ICSER_OFFSET)
#define RX65N_RIIC0_ICIER         (RX65N_RIIC0_BASE + RX65N_RIIC_ICIER_OFFSET)
#define RX65N_RIIC0_ICSR1         (RX65N_RIIC0_BASE + RX65N_RIIC_ICSR1_OFFSET)
#define RX65N_RIIC0_ICSR2         (RX65N_RIIC0_BASE + RX65N_RIIC_ICSR2_OFFSET)
#define RX65N_RIIC0_SARL0         (RX65N_RIIC0_BASE + RX65N_RIIC_SARL0_OFFSET)
#define RX65N_RIIC0_SARU0         (RX65N_RIIC0_BASE + RX65N_RIIC_SARU0_OFFSET)
#define RX65N_RIIC0_SARL1         (RX65N_RIIC0_BASE + RX65N_RIIC_SARL1_OFFSET)
#define RX65N_RIIC0_SARU1         (RX65N_RIIC0_BASE + RX65N_RIIC_SARU1_OFFSET)
#define RX65N_RIIC0_SARL2         (RX65N_RIIC0_BASE + RX65N_RIIC_SARL2_OFFSET)
#define RX65N_RIIC0_SARU2         (RX65N_RIIC0_BASE + RX65N_RIIC_SARU2_OFFSET)
#define RX65N_RIIC0_ICBRL         (RX65N_RIIC0_BASE + RX65N_RIIC_ICBRL_OFFSET)
#define RX65N_RIIC0_ICBRH         (RX65N_RIIC0_BASE + RX65N_RIIC_ICBRH_OFFSET)
#define RX65N_RIIC0_ICDRT         (RX65N_RIIC0_BASE + RX65N_RIIC_ICDRT_OFFSET)
#define RX65N_RIIC0_ICDRR         (RX65N_RIIC0_BASE + RX65N_RIIC_ICDRR_OFFSET)

#define RX65N_RIIC1_ICCR1         (RX65N_RIIC1_BASE + RX65N_RIIC_ICCR1_OFFSET)
#define RX65N_RIIC1_ICCR2         (RX65N_RIIC1_BASE + RX65N_RIIC_ICCR2_OFFSET)
#define RX65N_RIIC1_ICMR1         (RX65N_RIIC1_BASE + RX65N_RIIC_ICMR1_OFFSET)
#define RX65N_RIIC1_ICMR2         (RX65N_RIIC1_BASE + RX65N_RIIC_ICMR2_OFFSET)
#define RX65N_RIIC1_ICMR3         (RX65N_RIIC1_BASE + RX65N_RIIC_ICMR3_OFFSET)
#define RX65N_RIIC1_ICFER         (RX65N_RIIC1_BASE + RX65N_RIIC_ICFER_OFFSET)
#define RX65N_RIIC1_ICSER         (RX65N_RIIC1_BASE + RX65N_RIIC_ICSER_OFFSET)
#define RX65N_RIIC1_ICIER         (RX65N_RIIC1_BASE + RX65N_RIIC_ICIER_OFFSET)
#define RX65N_RIIC1_ICSR1         (RX65N_RIIC1_BASE + RX65N_RIIC_ICSR1_OFFSET)
#define RX65N_RIIC1_ICSR2         (RX65N_RIIC1_BASE + RX65N_RIIC_ICSR2_OFFSET)
#define RX65N_RIIC1_SARL0         (RX65N_RIIC1_BASE + RX65N_RIIC_SARL0_OFFSET)
#define RX65N_RIIC1_SARU0         (RX65N_RIIC1_BASE + RX65N_RIIC_SARU0_OFFSET)
#define RX65N_RIIC1_SARL1         (RX65N_RIIC1_BASE + RX65N_RIIC_SARL1_OFFSET)
#define RX65N_RIIC1_SARU1         (RX65N_RIIC1_BASE + RX65N_RIIC_SARU1_OFFSET)
#define RX65N_RIIC1_SARL2         (RX65N_RIIC1_BASE + RX65N_RIIC_SARL2_OFFSET)
#define RX65N_RIIC1_SARU2         (RX65N_RIIC1_BASE + RX65N_RIIC_SARU2_OFFSET)
#define RX65N_RIIC1_ICBRL         (RX65N_RIIC1_BASE + RX65N_RIIC_ICBRL_OFFSET)
#define RX65N_RIIC1_ICBRH         (RX65N_RIIC1_BASE + RX65N_RIIC_ICBRH_OFFSET)
#define RX65N_RIIC1_ICDRT         (RX65N_RIIC1_BASE + RX65N_RIIC_ICDRT_OFFSET)
#define RX65N_RIIC1_ICDRR         (RX65N_RIIC1_BASE + RX65N_RIIC_ICDRR_OFFSET)

#define RX65N_RIIC2_ICCR1         (RX65N_RIIC2_BASE + RX65N_RIIC_ICCR1_OFFSET)
#define RX65N_RIIC2_ICCR2         (RX65N_RIIC2_BASE + RX65N_RIIC_ICCR2_OFFSET)
#define RX65N_RIIC2_ICMR1         (RX65N_RIIC2_BASE + RX65N_RIIC_ICMR1_OFFSET)
#define RX65N_RIIC2_ICMR2         (RX65N_RIIC2_BASE + RX65N_RIIC_ICMR2_OFFSET)
#define RX65N_RIIC2_ICMR3         (RX65N_RIIC2_BASE + RX65N_RIIC_ICMR3_OFFSET)
#define RX65N_RIIC2_ICFER         (RX65N_RIIC2_BASE + RX65N_RIIC_ICFER_OFFSET)
#define RX65N_RIIC2_ICSER         (RX65N_RIIC2_BASE + RX65N_RIIC_ICSER_OFFSET)
#define RX65N_RIIC2_ICIER         (RX65N_RIIC2_BASE + RX65N_RIIC_ICIER_OFFSET)
#define RX65N_RIIC2_ICSR1         (RX65N_RIIC2_BASE + RX65N_RIIC_ICSR1_OFFSET)
#define RX65N_RIIC2_ICSR2         (RX65N_RIIC2_BASE + RX65N_RIIC_ICSR2_OFFSET)
#define RX65N_RIIC2_SARL0         (RX65N_RIIC2_BASE + RX65N_RIIC_SARL0_OFFSET)
#define RX65N_RIIC2_SARU0         (RX65N_RIIC2_BASE + RX65N_RIIC_SARU0_OFFSET)
#define RX65N_RIIC2_SARL1         (RX65N_RIIC2_BASE + RX65N_RIIC_SARL1_OFFSET)
#define RX65N_RIIC2_SARU1         (RX65N_RIIC2_BASE + RX65N_RIIC_SARU1_OFFSET)
#define RX65N_RIIC2_SARL2         (RX65N_RIIC2_BASE + RX65N_RIIC_SARL2_OFFSET)
#define RX65N_RIIC2_SARU2         (RX65N_RIIC2_BASE + RX65N_RIIC_SARU2_OFFSET)
#define RX65N_RIIC2_ICBRL         (RX65N_RIIC2_BASE + RX65N_RIIC_ICBRL_OFFSET)
#define RX65N_RIIC2_ICBRH         (RX65N_RIIC2_BASE + RX65N_RIIC_ICBRH_OFFSET)
#define RX65N_RIIC2_ICDRT         (RX65N_RIIC2_BASE + RX65N_RIIC_ICDRT_OFFSET)
#define RX65N_RIIC2_ICDRR         (RX65N_RIIC2_BASE + RX65N_RIIC_ICDRR_OFFSET)

/* RIIC register field/bit value definitions */

#define RX65N_RIIC_ICCR1_ICE_RST        (0x7f)
#define RX65N_RIIC_ICCR1_IICRST_SET     (0x40)

#define RX65N_RIIC_ICCR2_ST_SET         (0x02)
#define RX65N_RIIC_ICCR2_SP_SET         (0x08)
#define RX65N_RIIC_ICCR2_RS_SET         (0x04)

#define RX65N_RIIC_ICSR2_STOP_SET       (0x08)
#define RX65N_RIIC_ICSR2_START_SET      (0x04)

#define RX65N_RIIC_ICMR1_CKS_MASK       (0x8f)

#define RX65N_RIIC_ICMR2_SDDL0          (0x06)
#define RX65N_RIIC_ICMR2_SDDL1          (0x16)
#define RX65N_RIIC_ICMR2_SDDL2          (0x26)
#define RX65N_RIIC_ICMR2_SDDL3          (0x36)
#define RX65N_RIIC_ICMR2_SDDL4          (0x46)
#define RX65N_RIIC_ICMR2_SDDL5          (0x56)
#define RX65N_RIIC_ICMR2_SDDL6          (0x66)
#define RX65N_RIIC_ICMR2_SDDL7          (0x76)

#define RX65N_RIIC_ICMR3_NF1            (0x00)
#define RX65N_RIIC_ICMR3_NF2            (0x01)
#define RX65N_RIIC_ICMR3_NF3            (0x02)
#define RX65N_RIIC_ICMR3_NF4            (0x03)

#define RX65N_RIIC_ICMR3_WAIT_SET       (0x40)
#define RX65N_RIIC_ICMR3_ACKWP_SET      (0x10)
#define RX65N_RIIC_ICMR3_ACKBT_SET      (0x08)
#define RX65N_RIIC_ICMR3_ACKWP_CLR      (0xEF)
#define RX65N_RIIC_ICMR3_RDRFS_SET      (0x20)

#define RX65N_RIIC_ICIER_ALIE           (0x02)
#define RX65N_RIIC_ICIER_ST_NAK_AL      (0x16)
#define RX65N_RIIC_ICIER_SP_NAK_AL      (0x1a)
#define RX65N_RIIC_ICIER_SP_AL          (0x0a)
#define RX65N_RIIC_ICIER_TMO            (0x01)
#define RX65N_RIIC_ICIER_TEND_NAK_AL    (0x52)
#define RX65N_RIIC_ICIER_RX_NAK_AL      (0x32)

#define RX65N_RIIC_ICFER_TMOE_SET       (0x01)
#define RX65N_RIIC_ICFER_NFE_SET        (0x20)
#define RX65N_RIIC_ICFER_NFE_RST        (0xdf)

#define RX65N_RIIC_ICSER_SET            (0x00)

#define RX65N_RIIC_ICBRL_MASK           (0xe0)
#define RX65N_RIIC_ICBRH_MASK           (0xe0)

#define RX65N_I2C_SLV_ADDR              (0x80)
#define RX65N_RIIC_READ_MASK            (0x01)

#define RX65N_RIIC_10BIT_SARU_MASK      (0x0300)
#define RX65N_RIIC_10BIT_SARL_MASK      (0x00ff)

#define BUS_CHECK_COUNTER               (1000)
#define RIIC_REG_INIT                   (0x00)
#define RIIC_BUS_BUSY                   ((bool)(1))
#define RIIC_BUS_FREE                   ((bool)(0))

/* End of RIIC related definitions */

/* Start of DTC interface related definitions */

#if defined(CONFIG_RX65N_DTC)
  #define HAVE_DTC_DRIVER    1
#endif

#define RX65N_DTC_BASE       (uint32_t)&DTC

#define DTC_DTCCR_OFFSET     (0x0000)  /* DTC Control Register */
#define DTC_DTCVBR_OFFSET    (0x0004)  /* DTC Vector Base Register */
#define DTC_DTCADMOD_OFFSET  (0x0008)  /* DTC Address Mode Register */
#define DTC_DTCST_OFFSET     (0x000C)  /* DTC Control Register */
#define DTC_DTCSTS_OFFSET    (0x000E)  /* DTC Status Register */
#define DTC_DTCIBR_OFFSET    (0x0010)  /* DTC Index Table Base Register */
#define DTC_DTCOR_OFFSET     (0x0014)  /* DTC Operation Register */
#define DTC_DTCSQE_OFFSET    (0x0016)  /* DTC Sequence Transfer Enable Register */
#define DTC_DTCDISP_OFFSET   (0x0018)  /* DTC Address Displacement Register */

/* Bits of register DTCCR */

#define DTC_DTCCR_RRS        (1 << 4)  /* DTC Transfer Information Read Skip Enable*/

/* Bits of register DTCST */

#define DTC_DTCST_DTCST      (1 << 0)  /* DTC Transfer Information Read Skip Enable*/

/* End of DTC related defeinitions */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLER__
/* Serial Communications interface (SCI) */

enum   E_RX_SCI
{
  RX_SCI_SMR_OFFSET = 0,
  RX_SCI_BRR_OFFSET,
  RX_SCI_SCR_OFFSET,
  RX_SCI_TDR_OFFSET,
  RX_SCI_SSR_OFFSET,
  RX_SCI_RDR_OFFSET,
  RX_SCI_SCMR_OFFSET,
  RX_SCI_SEMR_OFFSET,
  RX_SCI_SNFR_OFFSET,
  RX_SCI_SIMR1_OFFSET,
  RX_SCI_SIMR2_OFFSET,
  RX_SCI_SIMR3_OFFSET,
  RX_SCI_SISR_OFFSET,
  RX_SCI_SPMR_OFFSET,
  RX_SCI_THRHL_OFFSET,
  RX_SCI_THRL_OFFSET,
  RX_SCI_RDRHL_OFFSET,
  RX_SCI_RDRL_OFFSET,
  RX_SCI_MDDR_OFFSET
};
#endif /* __ASSEMBLER__ */

#endif /* __ARCH_RENESAS_SRC_RX65N_RX65N_DEFINITIONS_H */
