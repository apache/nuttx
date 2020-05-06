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
#include "rx65n/iodefine.h"
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

#define RX_SCISSR_TDRE     (0x80)
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
#define RX65N_GRPBL1_TEI8_MASK          (1U << 24)
#define RX65N_GRPBL1_ERI8_MASK          (1U << 25)
#define RX65N_GRPBL1_TEI9_MASK          (1U << 26)
#define RX65N_GRPBL1_ERI9_MASK          (1U << 27)
#define RX65N_GRPAL0_TEI10_MASK         (1U <<  8)
#define RX65N_GRPAL0_ERI10_MASK         (1U <<  9)
#define RX65N_GRPAL0_TEI11_MASK         (1U << 12)
#define RX65N_GRPAL0_ERI11_MASK         (1U << 13)
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

#define ETHD_EESR_TDE        (1<20)

/* Ether PSR register */

#define ETH_PSR_LMON         (1)

/* EDMAC Transmit Request Register's bit */

#define ETHD_EDRRR_TR     (1) /* Transmit Request */

/* EDMAC Receive Request Register's bit */

#define ETHD_EDRRR_RR     (1) /* Receive descriptor read,
                               * and receive function is enabled
                               */

/* Transmit Interrupt Setting Register's bit */

#define ETHD_TRIMD_TIS        (1)    /* Transmit Interrupt is enabled */
#define ETHD_TRIMD_TIM        (1<<4) /* Write-back complete interrupt mode */

/* Receive Method Control Register's bit */

/* Receive Method Control Register's bit */

#define ETHD_RMCR_RNR        (1) /* EDRRR.RR bit (receive request bit) is not
                                  * set to 0 when one frame has been received
                                                                  */

/* FIFO Depth Register's bit */

#define ETHD_FDR_RFD         (7)     /* Receive FIFO Depth */
#define ETHD_FDR_TFD         (7<<8)  /* Transmit FIFO Depth */

/* ETHERC/EDMAC Transmit/Receive Status Copy Enable Register's bit */

#define ETHD_TRSCER_RRFCE     (1<<4)  /* RRF Flag Copy Enable */
#define ETHD_TRSCER_RMAFCE    (1<<7)  /* RMAF Flag Copy Enable */

/* Broadcast Frame Receive Count Setting Register's field */

#define ETH_BCFRR_BCF    (0x0000)  /* Broadcast Frame Continuous Receive Count Setting */

/* PHY Interface Register's bit and values */

#define ETH_PIR_MDC        (1)           /* MII/RMII Management Data Clock */
#define ETH_PIR_MMD        (1<<1)        /* MII/RMII Management Mode */
#define ETH_PIR_MDO        (1<<2)        /* MII/RMII Management Data-Out */
#define ETH_PIR_MDI        (1<<3)        /* MII/RMII Management Data-In */

#define ETH_PIR_RESET_ALL        (0x00000000)    /* Reset All Flags of PIR      */
#define ETH_PIR_SET_MDC          (0x00000001)    /* Setting MDC of PIR */
#define ETH_PIR_SET_MMD          (0x00000002)    /* Setting MMD of PIR */
#define ETH_PIR_SET_MMD_MDC      (0x00000003)    /* Setting MMD and MDC */
#define ETH_PIR_SET_MDO_MMD      (0x00000006)    /* Setting MDO and MMD */
#define ETH_PIR_SET_MDO_MMD_MDC  (0x00000007)    /* Setting MDO, MMD and MDC */

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

#define RX65N_RTC_BASE      (0x0008c400)
#define RX65N_RTC_R64CNT    (RX65N_RTC_BASE + RX65N_RTC_R64CNT_OFFSET)
#define RX65N_RTC_RSECCNT   (RX65N_RTC_BASE + RX65N_RTC_RSECCNT_OFFSET)
#define RX65N_RTC_RMINCNT   (RX65N_RTC_BASE + RX65N_RTC_RMINCNT_OFFSET)
#define RX65N_RTC_RHRCNT    (RX65N_RTC_BASE + RX65N_RTC_RHRCNT_OFFSET)
#define RX65N_RTC_RWKCNT    (RX65N_RTC_BASE + RX65N_RTC_RWKCNT_OFFSET)
#define RX65N_RTC_RDAYCNT   (RX65N_RTC_BASE + RX65N_RTC_RDAYCNT_OFFSET)
#define RX65N_RTC_RMONCNT   (RX65N_RTC_BASE + RX65N_RTC_RMONCNT_OFFSET)
#define RX65N_RTC_RYRCNT    (RX65N_RTC_BASE + RX65N_RTC_RYRCNT_OFFSET)
#define RX65N_RTC_RSECAR    (RX65N_RTC_BASE + RX65N_RTC_RSECAR_OFFSET)
#define RX65N_RTC_RMINAR    (RX65N_RTC_BASE + RX65N_RTC_RMINAR_OFFSET)
#define RX65N_RTC_RHRAR     (RX65N_RTC_BASE + RX65N_RTC_RHRAR_OFFSET)
#define RX65N_RTC_RWKAR     (RX65N_RTC_BASE + RX65N_RTC_RWKAR_OFFSET)
#define RX65N_RTC_RDAYAR    (RX65N_RTC_BASE + RX65N_RTC_RDAYAR_OFFSET)
#define RX65N_RTC_RMONAR    (RX65N_RTC_BASE + RX65N_RTC_RMONAR_OFFSET)
#define RX65N_RTC_RYRAR     (RX65N_RTC_BASE + RX65N_RTC_RYRAR_OFFSET)
#define RX65N_RTC_RYRAREN   (RX65N_RTC_BASE + RX65N_RTC_RYRAREN_OFFSET)
#define RX65N_RTC_RCR1      (RX65N_RTC_BASE + RX65N_RTC_RCR1_OFFSET)
#define RX65N_RTC_RCR2      (RX65N_RTC_BASE + RX65N_RTC_RCR2_OFFSET)
#define RX65N_RTC_RCR3      (RX65N_RTC_BASE + RX65N_RTC_RCR3_OFFSET)
#define RX65N_RTC_RCR4      (RX65N_RTC_BASE + RX65N_RTC_RCR4_OFFSET)
#define RX65N_RTC_RADJ      (RX65N_RTC_BASE + RX65N_RTC_RADJ_OFFSET)

#define RTC_RTC_ALRDIS                          (0x00)
#define RTC_RCR4_RCKSEL                         (0x00)
#define RTC_RCR3_RTCEN                          (0x01)
#define RTC_RCR3_RTCDV                          (0x02)
#define RTC_RCR2_START                          (0x01)
#define RTC_RCR2_CNTMD                          (0x00)
#define RTC_RCR2_RESET                          (0x01)
#define RTC_ALARM_INT_ENABLE                    (0x01)
#define RTC_CARRY_INT_ENABLE                    (0x02)
#define RTC_PERIOD_INT_ENABLE                   (0x04)
#define RTC_PERIODIC_INT_PERIOD_1               (0xe0)
#define _04_FOUR_READ_COUNT                     (0x04)
#define RTC_1_64_SEC_CYCLE                      (0x0005b8d9)
#define _0F_RTC_PRIORITY_LEVEL15                (0x0f)
#define  RTC_RCR1_CUP                           (0x02)
#define RX65N_SUBCLKOSC_SOSCCR                  (0x00080033)
#define SUBCLKOSC_SOSCCR_SOSTP                  (0x01)
#define RX65N_SUBCLKOSC_SOSCWTCR                (0x0008c293)
#define RTC_SOSCWTCR_VALUE                      (0x21)
#define RTC_DUMMY_READ                          (3)
#define _00_RTC_PRIORITY_LEVEL0                 (0)
#define _04_RTC_PERIOD_INT_ENABLE               (0x04)
#define RTC_RTC_CARRYDIS                        (0xe5)
#define RTC_RTC_PERDIS                          (0xe3)
#define RTC_RADJ_INITVALUE                      (0x0)
#define RTC_RCR2_AADJE                          (0x10)
#define RTC_RCR2_AADJP                          (0x20)

#if defined(CONFIG_RTC) || defined(CONFIG_RTC_DRIVER)

#define HAVE_RTC_DRIVER       1

#endif

#define RX65N_RTC_WAIT_PERIOD 184
#define RTC_RCR2_HR24                   (0x40)
#define RTC_PERIODIC_INTERRUPT_2_SEC    (0xf)

/* StandBy RAM Address */

#define RX65N_SBRAM_BASE  0x000a4000

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
