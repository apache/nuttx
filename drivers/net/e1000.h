/*****************************************************************************
 * drivers/net/e1000.h
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
 *****************************************************************************/

#ifndef __DRIVERS_NET_E1000_H
#define __DRIVERS_NET_E1000_H

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include <stdint.h>

/*****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/

/* General registers */

#define E1000_CTRL                  (0x0000)   /* Device Control Register */
#define E1000_STATUS                (0x0008)   /* Device Status Register  */
#define E1000_EEC                   (0x0010)   /* EEPROM/FLASH Control Register */
#define E1000_EERD                  (0x0014)   /* EEPROM Read Register */
#define E1000_CTRLEXT               (0x0018)   /* Extended Device Control Register */
#define E1000_FLA                   (0x001c)   /* Flash Access Register */
#define E1000_MDIC                  (0x0020)   /* MDI Control Register */
#define E1000_FCAL                  (0x0028)   /* Flow Control Address Low */
#define E1000_FCAH                  (0x002c)   /* Flow Control Address High */
#define E1000_FCT                   (0x0030)   /* Flow Control Type */
#define E1000_VET                   (0x0038)   /* VLAN Ether Type */
#define E1000_FCTTV                 (0x0170)   /* Flow Control Transmit Timer Value */
#define E1000_FCRTV                 (0x5f40)   /* Flow Control Refresh Threshold Value */
#define E1000_LEDCTL                (0x0e00)   /* LED Control */

/* Interrupt registers */

#define E1000_ICR                   (0x00c0)   /* Interrupt Cause Read */
#define E1000_ICS                   (0x00c8)   /* Interrupt Cause Set */
#define E1000_IMS                   (0x00d0)   /* Interrupt Mask Set */
#define E1000_IMC                   (0x00d8)   /* Interrupt Mask Clear */
#define E1000_EIAC                  (0x00dc)   /* Interrupt Auto Clear */
#define E1000_IAM                   (0x00e0)   /* Interrupt Acknowledge Auto–Mask  */
#define E1000_IVAR                  (0x00e4)   /* Interrupt Vector Allocation Registers  */

/* Transmit registers */

#define E1000_TCTL                  (0x0400)   /* Transmit Control */
#define E1000_TIPG                  (0x0410)   /* Transmit IPG Register */
#define E1000_AIT                   (0x0458)   /* Adaptive IFS Throttle */
#define E1000_TDBAL                 (0x3800)   /* Tx Descriptor Base Address Low */
#define E1000_TDBAH                 (0x3804)   /* Tx Descriptor Base Address High */
#define E1000_TDLEN                 (0x3808)   /* Tx Descriptor Length */
#define E1000_TDH                   (0x3810)   /* Tx Descriptor Head */
#define E1000_TDT                   (0x3818)   /* Tx Descriptor Tail */
#define E1000_TARC                  (0x3840)   /* Transmit Arbitration Count */
#define E1000_TIDV                  (0x3820)   /* Transmit Interrupt Delay Value */
#define E1000_TXDCTL                (0x3828)   /* Transmit Descriptor Control */
#define E1000_TADV                  (0x382c)   /* Transmit Absolute Interrupt Delay Value */

/* Receive registers */

#define E1000_RCTL                  (0x0100)   /* Receive Control */
#define E1000_PSRCTL                (0x2170)   /* Packet Split Receive Control Register */
#define E1000_FCRTL                 (0x2160)   /* Flow Control Receive Threshold Low */
#define E1000_FCRTH                 (0x2168)   /* Flow Control Receive Threshold High */
#define E1000_RDBAL                 (0x2800)   /* Rx Descriptor Base Address Low */
#define E1000_RDBAH                 (0x2804)   /* Rx Descriptor Base Address High */
#define E1000_RDLEN                 (0x2808)   /* Rx Descriptor Length */
#define E1000_RDH                   (0x2810)   /* Rx Descriptor Head */
#define E1000_RDT                   (0x2818)   /* Rx Descriptor Tail */
#define E1000_RDTR                  (0x2820)   /* Receive Delay Timer Register */
#define E1000_RXDCTL                (0x2828)   /* Receive Descriptor Control */
#define E1000_RADV                  (0x282c)   /* Receive Interrupt Absolute Delay Timer */
#define E1000_RSRPD                 (0x2c00)   /* Receive Small Packet Detect Interrupt */
#define E1000_RAID                  (0x2c08)   /* Receive ACK Interrupt Delay Register */
#define E1000_RXCSUM                (0x5000)   /* Receive Checksum Control */
#define E1000_RFCTL                 (0x5008)   /* Receive Filter Control Register */
#define E1000_MTA                   (0x5200)   /* Multicast Table Array */
#define E1000_RAL                   (0x5400)   /* Receive Address Low */
#define E1000_RAH                   (0x5404)   /* Receive Address High */

/* Statistic registers */

#define E1000_CRCERRS               (0x04000)  /* CRC Error Count */
#define E1000_ALGNERRC              (0x04004)  /* Alignment Error Count */
#define E1000_RXERRC                (0x0400c)  /* RX Error Count */
#define E1000_MPC                   (0x04010)  /* Missed Packets Count */
#define E1000_SCC                   (0x04014)  /* Single Collision Count */
#define E1000_ECOL                  (0x04018)  /* Excessive Collisions Count */
#define E1000_MCC                   (0x0401c)  /* Multiple Collision Count */
#define E1000_LATECOL               (0x04020)  /* Late Collisions Count */
#define E1000_COLC                  (0x04028)  /* Collision Count */
#define E1000_DC                    (0x04030)  /* Defer Count */
#define E1000_TNCRS                 (0x04034)  /* Transmit with No CRS */
#define E1000_CEXTERR               (0x0403c)  /* Carrier Extension Error Count */
#define E1000_RLEC                  (0x04040)  /* Receive Length Error Count */
#define E1000_XONRXC                (0x04048)  /* XON Received Count */
#define E1000_XONTXC                (0x0404c)  /* XON Transmitted Count */
#define E1000_XOFFRXC               (0x04050)  /* XOFF Received Count */
#define E1000_XOFFTXC               (0x04054)  /* XOFF Transmitted Count */
#define E1000_FCRUC                 (0x04058)  /* FC Received Unsupported Count */
#define E1000_PRC64                 (0x0405c)  /* Packets Received [64 Bytes] Count */
#define E1000_PRC127                (0x04060)  /* Packets Received [65–127 Bytes] Count */
#define E1000_PRC255                (0x04064)  /* Packets Received [128–255 Bytes] */
#define E1000_PRC511                (0x04068)  /* Packets Received [256–511 Bytes] */
#define E1000_PRC1023               (0x0406c)  /* Packets Received [512–1023 Bytes] */
#define E1000_PRC1522               (0x04070)  /* Packets Received [1024 to Max Bytes] */
#define E1000_GPRC                  (0x04074)  /* Good Packets Received Count */
#define E1000_BPRC                  (0x04078)  /* Broadcast Packets Received Count */
#define E1000_MPRC                  (0x0407c)  /* Multicast Packets Received Count */
#define E1000_GPTC                  (0x04080)  /* Good Packets Transmitted Count */
#define E1000_GORCL                 (0x04088)  /* Good Octets Received Count Low */
#define E1000_GORCH                 (0x0408c)  /* Good Octets Received Count High */
#define E1000_GOTCL                 (0x04090)  /* Good Octets Transmitted Count Low */
#define E1000_GOTCH                 (0x04094)  /* Good Octets Transmitted Count High */
#define E1000_RNBC                  (0x040a0)  /* Receive No Buffers Count */
#define E1000_RUC                   (0x040a4)  /* Receive Undersize Count */
#define E1000_RFC                   (0x040a8)  /* Receive Fragment Count */
#define E1000_ROC                   (0x040ac)  /* Receive Oversize Count */
#define E1000_RJC                   (0x040b0)  /* Receive Jabber Count */
#define E1000_MNGPRC                (0x040b4)  /* Management Packets Received Count */
#define E1000_MPDC                  (0x040b8)  /* Management Packets Dropped Count */
#define E1000_MPTC                  (0x040bc)  /* Management Packets Transmitted Count */
#define E1000_TORL                  (0x040c0)  /* Total Octets Received */
#define E1000_TORH                  (0x040c4)  /* Total Octets Received */
#define E1000_TOT                   (0x040c8)  /* Total Octets Transmitted */
#define E1000_TPR                   (0x040d0)  /* Total Packets Received */
#define E1000_TPT                   (0x040d4)  /* Total Packets Transmitted */
#define E1000_PTC64                 (0x040d8)  /* Packets Transmitted [64 Bytes] Count */
#define E1000_PTC127                (0x040dc)  /* Packets Transmitted [65–127 Bytes] Count */
#define E1000_PTC255                (0x040e0)  /* Packets Transmitted [128–255 Bytes] Count */
#define E1000_PTC511                (0x040e4)  /* Packets Transmitted [256–511 Bytes] Count */
#define E1000_PTC1023               (0x040e8)  /* Packets Transmitted [512–1023 Bytes] Count */
#define E1000_PTC1522               (0x040ec)  /* Packets Transmitted [Greater than 1024 Bytes] Count */
#define E1000_MCPTC                 (0x040f0)  /* Multicast Packets Transmitted Count */
#define E1000_BPTC                  (0x040f4)  /* Broadcast Packets Transmitted Count */
#define E1000_TSCTC                 (0x040f8)  /* TCP Segmentation Context Transmitted Count */
#define E1000_TSCTFC                (0x040fc)  /* TCP Segmentation Context Transmit Fail Count */
#define E1000_IAC                   (0x04100)  /* Interrupt Assertion Count */

/* Management registers */

#define E1000_WUC                   (0x5800)   /* Wake Up Control Register */
#define E1000_WUFC                  (0x5808)   /* Wake Up Filter Control Register */
#define E1000_WUS                   (0x5810)   /* Wake Up Status Register */
#define E1000_MFUTP01               (0x5828)   /* Management Flex UDP/TCP Ports 0/1 */
#define E1000_MFUTP23               (0x5830)   /* Management Flex UDP/TCP Port 2/3 */
#define E1000_IPAV                  (0x5838)   /* IP Address Valid */

/* Diagnostic registers */

#define E1000_POEMB                 (0x00f10)  /* PHY OEM Bits Register */
#define E1000_RDFH                  (0x02410)  /* Receive Data FIFO Head Register */
#define E1000_FDFT                  (0x02418)  /* Receive Data FIFO Tail Register */
#define E1000_RDFHS                 (0x02420)  /* Receive Data FIFO Head Saved Register */
#define E1000_RDFTS                 (0x02428)  /* Receive Data FIFO Tail Saved Register */
#define E1000_RDFPC                 (0x02430)  /* Receive Data FIFO Packet Count */
#define E1000_TDFH                  (0x03410)  /* Transmit Data FIFO Head Register */
#define E1000_TDFT                  (0x03418)  /* Transmit Data FIFO Tail Register */
#define E1000_TDFHS                 (0x03420)  /* Transmit Data FIFO Head Saved Register */
#define E1000_TDFTS                 (0x03428)  /* Transmit Data FIFO Tail Saved Register */
#define E1000_TDFPC                 (0x03430)  /* Transmit Data FIFO Packet Count */
#define E1000_PBM                   (0x10000)  /* Packet Buffer Memory */
#define E1000_PBS                   (0x01008)  /* Packet Buffer Size */

/* Device Control Register */

#define E1000_CTRL_FD               (1 << 0)   /* Bit 0: Full-Duplex */
                                               /* Bits 1-2: Reserved */
#define E1000_CTRL_LRST             (1 << 3)   /* Bit 3: Link Reset */
                                               /* Bit 4: Reserved */
#define E1000_CTRL_ASDE             (1 << 5)   /* Bit 5: Auto-Speed Detection Enable */
#define E1000_CTRL_SLU              (1 << 6)   /* Bit 6: Set Link Up */
#define E1000_CTRL_ILOS             (1 << 7)   /* Bit 7: Speed selection */
#define E1000_CTRL_SPEED_SHIFT      (8)        /* Bits 8-9: Speed selection */
#define E1000_CTRL_SPEED_MASK       (3 << E1000_CTRL_SPEED_SHIFT)
#  define E1000_CTRL_SPEED_10MBS    (0 << E1000_CTRL_SPEED_SHIFT)
#  define E1000_CTRL_SPEED_100MBS   (1 << E1000_CTRL_SPEED_SHIFT)
#  define E1000_CTRL_SPEED_1000MBS  (2 << E1000_CTRL_SPEED_SHIFT)
                                               /* Bit 10: Reserved */
#define E1000_CTRL_FRCSPD           (1 << 11)  /* Bit 11: Force Speed */
#define E1000_CTRL_FRCDPLX          (1 << 12)  /* Bit 12: Force Duplex */
                                               /* Bits 13-17: Reserved */
#define E1000_CTRL_SDP0_DATA        (1 << 18)  /* Bit 18: SDP0 Data Value */
#define E1000_CTRL_SDP1_DATA        (1 << 19)  /* Bit 19: SDP1 Data Value */
#define E1000_CTRL_ADVD3WUC         (1 << 20)  /* Bit 20: D3Cold Wakeup Capability Advertisement Enable */
#define E1000_CTRL_PWRMGMT          (1 << 21)  /* Bit 21: PHY Power-Management Enable */
#define E1000_CTRL_SDP0_IODIR       (1 << 22)  /* Bit 22: SDP0 Pin Directionality */
#define E1000_CTRL_SDP1_IODIR       (1 << 23)  /* Bit 23: SDP1 Pin Directionality */
                                               /* Bits 24-25: Reserved */
#define E1000_CTRL_RST              (1 << 26)  /* Bit 26: Device Reset */
#define E1000_CTRL_RFCE             (1 << 27)  /* Bit 27: Receive Flow Control Enable */
#define E1000_CTRL_TFCE             (1 << 28)  /* Bit 28: Transmit Flow Control Enable */
                                               /* Bit 29: Reserved */
#define E1000_CTRL_VME              (1 << 30)  /* Bit 30: VLAN Mode Enable */
#define E1000_CTRL_PHYRST           (1 << 31)  /* Bit 31: PHY Reset */

/* Status Regiuster */

#define E1000_STATUS_FD             (1 << 0)   /* Bit 0: Full Duplex */
#define E1000_STATUS_LU             (1 << 1)   /* Bit 1: Link Up */
                                               /* Bits 2-3: Reserved */
#define E1000_STATUS_TXOFF          (1 << 4)   /* Bit 4: Transmission Paused */
                                               /* Bit 5: Reserved */
#define E1000_STATUS_SPEED_SHFIT    (5)        /* Bits 6-7: Link speed setting */
#define E1000_STATUS_ASDV_SHFIT     (8)        /* Bits 8-9: Auto-Speed Detection Value */
#define E1000_STATUS_PHYRA          (10)       /* Bit 10: PHY Reset Asserted */
                                               /* Bits 11-18: Reserved */
#define E1000_STATUS_GIOM           (19)       /* Bit 19: GIO Master Disable bit state */
                                               /* Bits 20-32: Reserved */

/* Receive Address High */

#define E1000_RAH_RAH_MASK          (0xffff)   /* Bits 0-15: Receive address High */
#define E1000_RAH_AS                (16)       /* Bits 16-17 Address Select */
                                               /* Bits 18-30: Reserved */
#define E1000_RAH_AV                (1 << 31)  /* Bit 31: Address Valid */

/* Transmit Control */

                                               /* Bit 0: Reserved */
#define E1000_TCTL_EN               (1 << 1)   /* Bit 1: Transmit Enable */
                                               /* Bit 2: Reserved */
#define E1000_TCTL_PSP              (1 << 3)   /* Bit 3: Pad Short Packets */
#define E1000_TCTL_CT_SHIFT         (4)        /* Bits 4-11: Collision Threshold */
#define E1000_TCTL_COLD_SHIFT       (12)       /* Bits 12-21: Collision Distance */
#define E1000_TCTL_SWXOFF           (1 << 22)  /* Bit 22: Software XOFF Transmission */
                                               /* Bit 23: Reserved */
#define E1000_TCTL_RTLC             (1 << 24)  /* Bit 24: Re-transmit on Late Collision */
#define E1000_TCTL_NRTU             (1 << 25)  /* Bit 25: No Re-transmit on underrun */
                                               /* Bits 26-32: Reserved */

/* Transmit Descriptor Control */

#define E1000_TXDCTL_PTHRESH_SHIFT  (0)        /* Bits 0-5: Prefetch Threshold */
                                               /* Bits 6-7: Reserved */
#define E1000_TXDCTL_HTHRESH_SHIFT  (8)        /* Bits 8-13: Host Threshold */
                                               /* Bits 14-15: Reserved */
#define E1000_TXDCTL_WTHRESH_SHIFT  (16)       /* Bits 16-21: Write-Back Threshold */
                                               /* Bits 22-23: Reserved */
#define E1000_TXDCTL_GRAN           (1 << 24)  /* Bit 24: Granularity */
#define E1000_TXDCTL_LWTHRESH_SHIFT (25)       /* Bits 25-32: Transmit Descriptor Low Threshold */

/* Receive Control */

                                               /* Bit 0: Reserved */
#define E1000_RCTL_EN               (1 << 1)   /* Bit 1: Receiver Enable */
#define E1000_RCTL_SBP              (1 << 2)   /* Bit 2: Store Bad Packets */
#define E1000_RCTL_UPE              (1 << 3)   /* Bit 3: Unicast Promiscuous Enabled */
#define E1000_RCTL_MPE              (1 << 4)   /* Bit 4: Multicast Promiscuous Enabled */
#define E1000_RCTL_LPE              (1 << 5)   /* Bit 5: Long Packet Reception Enable */
#define E1000_RCTL_LBM_SHIFT        (6)        /* Bits 6-7: Loopback mode */
#define E1000_RCTL_RDMTS_SHIFT      (8)        /* Bits 8-9: Receive Descriptor Minimum Threshold Size */
                                               /* Bits 10-11: Reserved */
#define E1000_RCTL_MO_SHIFT         (12)       /* Bits 12-13: Multicast Offset */
#  define E1000_RCTL_MO_4736        (0)        /* 0: bit 47:36 */
#  define E1000_RCTL_MO_4635        (1)        /* 1: bit 46:35 */
#  define E1000_RCTL_MO_4535        (2)        /* 2: bit 45:34 */
#  define E1000_RCTL_MO_4332        (3)        /* 3: bit 43:32 */
                                               /* Bit 14: Reserved */
#define E1000_RCTL_BAM              (1 << 15)  /* Bit 15: Broadcast Accept Mode */
#define E1000_RCTL_BSIZE_SHIFT      (16)       /* Bits 16-17: Receive Buffer Size */
#  define E1000_RCTL_BSIZE_2048     (0 << 16)  /* 00b: 2048 bytes */
#  define E1000_RCTL_BSIZE_1024     (1 << 16)  /* 01b: 1024 bytes */
#  define E1000_RCTL_BSIZE_512      (2 << 16)  /* 10b: 512 bytes */
#  define E1000_RCTL_BSIZE_256      (3 << 16)  /* 11b: 256 bytes */
                                               /* 00b: Reserved  */
#  define E1000_RCTL_BSIZE_16384    (1 << 16)  /* 00b: 16384 bytes */
#  define E1000_RCTL_BSIZE_8192     (2 << 16)  /* 01b: 8192 bytes */
#  define E1000_RCTL_BSIZE_4096     (3 << 16)  /* 10b: 4096 bytes */
#define E1000_RCTL_VFE              (1 << 18)  /* Bit 18: VLAN Filter Enable */
#define E1000_RCTL_CFIEN            (1 << 18)  /* Bit 19: Canonical Form Indicator Enable */
#define E1000_RCTL_CFI              (1 << 20)  /* Bit 20: Canonical Form Indicator bit value */
                                               /* Bit 21: Reserved */
#define E1000_RCTL_DPF              (1 << 22)  /* Bit 22: Discard Pause Frames */
#define E1000_RCTL_PMCF             (1 << 23)  /* Bit 23: Pass MAC Control Frames */
                                               /* Bit 24: Reserved */
#define E1000_RCTL_BSEX             (1 << 25)  /* Bit 25: Buffer Size Extension */
#define E1000_RCTL_SECRC            (1 << 26)  /* Bit 26: Strip Ethernet CRC from incoming packet */
                                               /* Bits 27-31: Reserved */

/* Receive Descriptor Control */

#define E1000_RXDCTL_PTHRESH_SHIFT  (0)        /* Bits 0-5: Prefetch Threshold */
                                               /* Bits 6-7: Reserved */
#define E1000_RXDCTL_HTHRESH_SHIFT  (8)        /* Bits 8-13: Host Threshold */
                                               /* Bits 14-15: Reserved */
#define E1000_RXDCTL_WTHRESH_SHIFT  (16)       /* Bits 16-21: Write-Back Threshold */
                                               /* Bits 22-23: Reserved */
#define E1000_RXDCTL_GRAN           (1 << 24)  /* Bit 24: Granularity */
                                               /* Bits 25-32: Reserved */

/* Receive Delay Timer Register  */

#define E1000_RDTR_DELAY_MASK       (0xffff)   /* Bits 0-15: Receive delay timer */
#define E1000_RDTR_FPD              (1 << 31)  /* Bit 31: Flush partial descriptor block */

/* Interrupt Cause */

#define E1000_IC_TXDW               (1 << 0)   /* Bit 0: Transmit Descriptor Written Back */
#define E1000_IC_TXQE               (1 << 1)   /* Bit 1: Transmit Queue Empty */
#define E1000_IC_LSC                (1 << 2)   /* Bit 2: Link Status Change */
#define E1000_IC_RXSEQ              (1 << 3)   /* Bit 3: Receive Sequence Error */
#define E1000_IC_RXDMT0             (1 << 4)   /* Bit 4: Receive Descriptor Minimum Threshold Reached */
                                               /* Bit 5: Reserved */
#define E1000_IC_RXO                (1 << 6)   /* Bit 6: Receiver Overrun */
#define E1000_IC_RXT0               (1 << 7)   /* Bit 7: Receiver Timer Interrupt */
                                               /* Bit 8: Reserved */
#define E1000_IC_MDAC               (1 << 9)   /* Bit 9: MDI/O Access Complete */
#define E1000_IC_RXCFG              (1 << 10)  /* Bit 10: Receiving /C/ ordered sets */
                                               /* Bit 11: Reserved */
#define E1000_IC_PHYINT             (1 << 12)  /* Bit 12: PHY Interrupt */
#define E1000_IC_GPISDP6            (1 << 13)  /* Bit 13: General Purpose Interrupt on SDP6[2] */
#define E1000_IC_GPISDP7            (1 << 14)  /* Bit 14: General Purpose Interrupt on SDP7[3] */
#define E1000_IC_TXDLOW             (1 << 15)  /* Bit 15: Transmit Descriptor Low Threshold hit */
#define E1000_IC_SRPD               (1 << 16)  /* Bit 16: Small Receive Packet Detected */
                                               /* Bits 17-20: Reserved */
#define E1000_IC_RXQ0               (1 << 20)  /* Bit 20: Receive Queue 0 Interrupt */
#define E1000_IC_RXQ1               (1 << 21)  /* Bit 21: Receive Queue 1 Interrupt */
#define E1000_IC_TXQ0               (1 << 22)  /* Bit 22: Transmit Queue 0 Interrupt */
#define E1000_IC_TXQ1               (1 << 23)  /* Bit 23: Transmit Queue 1 Interrupt */
#define E1000_IC_OTHER              (1 << 24)  /* Bit 24: Other Interrupt */
                                               /* Bits 25-30: Reserved */
#define E1000_IC_ASSERTED           (1 << 31)  /* Bit 31: Interrupt Asserted */

/* Interrupt Vector Allocation Registers */

#define E1000_IVAR_RXQ0_SHIFT       (0)        /* Bits 0-2: MSI-X vector assigned to RxQ0 */
#define E1000_IVAR_RXQ0_EN          (1 << 3)   /* Bit 3: Enable bit for RxQ0 */
#define E1000_IVAR_RXQ1_SHIFT       (4)        /* Bits 4-6: MSI-X vector assigned to RxQ1 */
#define E1000_IVAR_RXQ1_EN          (1 << 7)   /* Bit 7: Enable bit for RxQ1 */
#define E1000_IVAR_TXQ0_SHIFT       (8)        /* Bits 8-10: MSI-X vector assigned to TxQ0 */
#define E1000_IVAR_TXQ0_EN          (1 << 11)  /* Bit 11: Enable bit for TxQ0 */
#define E1000_IVAR_TXQ1_SHIFT       (12)       /* Bits 12-14: MSI-X vector assigned to TxQ1 */
#define E1000_IVAR_TXQ1_EN          (1 << 15)  /* Bit 15: Enable bit for TxQ1 */
#define E1000_IVAR_OTHER_SHIFT      (16)       /* Bits 16-18: MSI-X vector assigned to Ohter Cause */
#define E1000_IVAR_OTHER_EN         (1 << 19)  /* Bit 19: Enable bit for Ohter Cause */
                                               /* Bits 20-30: Reserved */
#define E1000_IVAR_ONALLWB          (1 << 31)  /* Bit 31: Tx interrupts occur on every write back */

/* Transmit Descriptor Command Field */

#define E1000_TDESC_CMD_EOP         (1 << 0)   /* Bit 0: End Of Packet */
#define E1000_TDESC_CMD_IFCS        (1 << 1)   /* Bit 1: Insert FCS */
#define E1000_TDESC_CMD_IC          (1 << 2)   /* Bit 2: Insert Checksum */
#define E1000_TDESC_CMD_RS          (1 << 3)   /* Bit 3: Report Status */
#define E1000_TDESC_CMD_RPS         (1 << 4)   /* Bit 4: Report Packet Sent */
#define E1000_TDESC_CMD_DEXT        (1 << 5)   /* Bit 5: Extension   (0b for legacy mode) */
#define E1000_TDESC_CMD_VLE         (1 << 6)   /* Bit 6: VLAN Packet Enable */
#define E1000_TDESC_CMD_IDE         (1 << 7)   /* Bit 7: Interrupt Delay Enable */

/* Transmit Descriptor Status Field */

#define E1000_TDESC_STATUS_DD       (1 << 0)   /* Bit 0: Descriptor Done */
#define E1000_TDESC_STATUS_EC       (1 << 1)   /* Bit 1: Excess Collisions */
#define E1000_TDESC_STATUS_LC       (1 << 2)   /* Bit 2: Late Collision */
#define E1000_TDESC_STATUS_RSU      (1 << 3)   /* Bit 3: Transmit Underrun */

/* Transmit Descriptor Special Field */

#define E1000_TDESC_SPEC_VLAN_SHIFT (0)        /* Bits 0-11: VLAN Identifier */
#define E1000_TDESC_SPEC_CFI        (1 << 12)  /* Bit 12: Canonical Form Indicator */
#define E1000_TDESC_SPEC_PRI_SHIFT  (13)       /* Bits 13-15: User Priority */

/* Receive Descriptor Status Field */

#define E1000_RDESC_STATUS_DD       (1 << 0)   /* Bit 0: Descriptor Done */
#define E1000_RDESC_STATUS_EOP      (1 << 1)   /* Bit 1: End of Packet */
#define E1000_RDESC_STATUS_IXSM     (1 << 2)   /* Bit 2: Ignore Checksum Indication */
#define E1000_RDESC_STATUS_VP       (1 << 3)   /* Bit 3: Packet is 802.1Q (matched VET) */
#define E1000_RDESC_STATUS_RSV      (1 << 4)   /* Bit 4: Reserved */
#define E1000_RDESC_STATUS_TCPCS    (1 << 5)   /* Bit 5: TCP Checksum Calculated on Packet */
#define E1000_RDESC_STATUS_IPCS     (1 << 6)   /* Bit 6: IP Checksum Calculated on Packet */
#define E1000_RDESC_STATUS_PIF      (1 << 7)   /* Bit 7: Passed in-exact filter */

/* Receive Descriptor Errors Field */

#define E1000_RDESC_ERRORS_CE       (1 << 0)   /* Bit 0: CRC Error or Alignment Error */
#define E1000_RDESC_ERRORS_SE       (1 << 1)   /* Bit 1: Symbol Error */
#define E1000_RDESC_ERRORS_SEQ      (1 << 2)   /* Bit 2: Sequence Error */
#define E1000_RDESC_ERRORS_RSV      (1 << 3)   /* Bit 3: Reserved */
#define E1000_RDESC_ERRORS_CXE      (1 << 4)   /* Bit 4: Carrier Extension Error */
#define E1000_RDESC_ERRORS_TCPE     (1 << 5)   /* Bit 5: TCP/UDP Checksum Error */
#define E1000_RDESC_ERRORS_IPE      (1 << 6)   /* Bit 6: IP Checksum Error */
#define E1000_RDESC_ERRORS_RXE      (1 << 7)   /* Bit 7: RX Data Error */

/* Receive Descriptor Special Field */

#define E1000_RDESC_SPEC_VLAN_SHIFT (0)        /* Bits 0-11: VLAN Identifier */
#define E1000_RDESC_SPEC_CFI        (1 << 12)  /* Bit 12: Canonical Form Indicator */
#define E1000_RDESC_SPEC_PRI_SHIFT  (13)       /* Bits 13-15: User Priority */

/* Future Extended NVM register 11 */

#define E1000_FEXTNVM11             0x05bbc
#define E1000_FEXTNVM11_MAGIC       0x00002000 /* Some magic number */

/*****************************************************************************
 * Public Types
 *****************************************************************************/

/* Legacy TX descriptor */

begin_packed_struct struct e1000_tx_leg_s
{
  uint64_t addr;
  uint16_t len;
  uint8_t  cso;
  uint8_t  cmd;
  uint8_t  status;
  uint8_t  css;
  uint16_t special;
} end_packed_struct;

/* Legacy RX descriptor */

begin_packed_struct struct e1000_rx_leg_s
{
  uint64_t addr;
  uint16_t len;
  uint16_t checksum;
  uint8_t  status;
  uint8_t  errors;
  uint16_t special;
} end_packed_struct;

#endif  /* __DRIVERS_NET_E1000_H */
