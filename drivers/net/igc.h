/*****************************************************************************
 * drivers/net/igc.h
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

#ifndef __DRIVERS_NET_IGC_H
#define __DRIVERS_NET_IGC_H

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include <stdint.h>

/*****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/

/* General registers */

#define IGC_CTRL                  (0x0000)   /* Device Control Register */
#define IGC_STATUS                (0x0008)   /* Device Status Register  */
#define IGC_CTRLEXT               (0x0018)   /* Extended Device Control Register */
#define IGC_MDIC                  (0x0020)   /* MDI Control Register */
#define IGC_FCAL                  (0x0028)   /* Flow Control Address Low */
#define IGC_FCAH                  (0x002c)   /* Flow Control Address High */
#define IGC_FCT                   (0x0030)   /* Flow Control Type */
#define IGC_CONNSW                (0x0034)   /* Copper/Fiber Switch Control */
#define IGC_VET                   (0x0038)   /* VLAN Ether Type */
#define IGC_FCTTV                 (0x0170)   /* Flow Control Transmit Timer Value */
#define IGC_LEDCTL                (0x0e00)   /* LED Control */
#define IGC_MDICNFG               (0x0e04)   /* MDC/MDIO Configuration Register */
#define IGC_I2CCM                 (0x1028)   /* SFP I2C Command */
#define IGC_I2CPARAMS             (0x102c)   /* SFP I2C Parameter */
#define IGC_WDSTP                 (0x1040)   /* Watchdog Setup Register */
#define IGC_WDSWSTS               (0x1044)   /* Watchdog Software */
#define IGC_FRTIMER               (0x1048)   /* Free Running Timer */
#define IGC_TCPTIMER              (0x104c)   /* TCP Timer */
#define IGC_SWSM                  (0x5b50)   /* Software Semaphore Register */
#define IGC_FWSM                  (0x5b54)   /* Firmware Semaphore Register */
#define IGC_SWFWSYNC              (0x5b5c)   /* Software-Firmware Synchronization */
#define IGC_IPCNFG                (0x0e38)   /* Internal PHY Configuration */
#define IGC_PHMP                  (0x0e14)   /* PHY Power Management */

/* NVM-Security Registers */

#define IGC_EEC                   (0x12010)  /* EEPROM/FLASH Control Register */
#define IGC_EERD                  (0x12014)  /* EEPROM Read Register */
#define IGC_EEWR                  (0x12018)  /* EEPROM-Mode Write Register */
#define IGC_FLA                   (0x1201c)  /* Flash Access Register */
#define IGC_FLASECU               (0x12114)  /* Flash Security Register */

/* Interrupt registers */

#define IGC_ICR                   (0x1500)   /* Interrupt Cause Read */
#define IGC_ICS                   (0x1504)   /* Interrupt Cause Set */
#define IGC_IMS                   (0x1508)   /* Interrupt Mask Set */
#define IGC_IMC                   (0x150c)   /* Interrupt Mask Clear */
#define IGC_IAM                   (0x1510)   /* Interrupt Acknowledge Auto–Mask */
#define IGC_EICS                  (0x1520)   /* Extended Interrupt Cause Set */
#define IGC_EIMS                  (0x1524)   /* Extended Interrupt Mask Set/Read */
#define IGC_EIMC                  (0x1528)   /* Extended Interrupt Mask Clear */
#define IGC_EIAC                  (0x152c)   /* Extended Interrupt Auto Clear */
#define IGC_EIAM                  (0x1530)   /* Extended Interrupt Auto Mask */
#define IGC_EICR                  (0x1580)   /* Extended Interrupt Cause Read */
#define IGC_IVAR0                 (0x1700)   /* Interrupt Vector Allocation Registers  */
#define IGC_IVARMSC               (0x1740)   /* Interrupt Vector Allocation Registers - MISC */
#define IGC_EITR0                 (0x1680)   /* Extended Interrupt Throttling Rate 0 - 24 */
#define IGC_GPIE                  (0x1514)   /* General Purpose Interrupt Enable */
#define IGC_PBACL                 (0x5b68)   /* MSI-X PBA Clear */
#define IGC_PICAUSE               (0x5b88)   /* PCIe Interrupt Cause */
#define IGC_PIENA                 (0x5b8c)   /* PCIe Interrupt Enable */

/* Receive registers */

#define IGC_RCTL                  (0x0100)   /* Receive Control */
#define IGC_PSRCTL                (0x2170)   /* Packet Split Receive Control Register */
#define IGC_FCRTL0                (0x2160)   /* Flow Control Receive Threshold Low */
#define IGC_FCRTH0                (0x2168)   /* Flow Control Receive Threshold High */
#define IGC_RXPBSIZE              (0x2404)   /* Rx Packet Buffer Size */
#define IGC_FCRTV                 (0x2460)   /* Flow Control Refresh Threshold Value */
#define IGC_RDBAL0                (0xc000)   /* Tx Descriptor Base Address Low */
#define IGC_RDBAH0                (0xc004)   /* Rx Descriptor Base Address High */
#define IGC_RDLEN0                (0xc008)   /* Rx Descriptor Length */
#define IGC_SRRCTL0               (0xc00c)   /* Split and Replication Receive Control Register Queue */
#define IGC_RDH0                  (0xc010)   /* Rx Descriptor Head */
#define IGC_RDT0                  (0xc018)   /* Rx Descriptor Tail */
#define IGC_RXDCTL0               (0xc028)   /* Receive Descriptor Control Queue */
#define IGC_RXCTL0                (0xc014)   /* Receive Queue DCA CTRL Register */
#define IGC_RXCSUM                (0x5000)   /* Receive Checksum Control */
#define IGC_RLPML                 (0x5004)   /* Receive Long packet maximal length */
#define IGC_RFCTL                 (0x5008)   /* Receive Filter Control Register */
#define IGC_MTA                   (0x5200)   /* Multicast Table Array (n) */
#define IGC_RAL                   (0x5400)   /* Receive Address Low */
#define IGC_RAH                   (0x5404)   /* Receive Address Low */
#define IGC_VLANPQF               (0x55b0)   /* VLAN Priority Queue Filter */
#define IGC_PSRTYPE0              (0x5480)   /* Packet Split Receive type (n) */
#define IGC_VFTA0                 (0x5600)   /* VLAN Filter Table Array (n) */
#define IGC_MRQC                  (0x5818)   /* Multiple Receive Queues Command */
#define IGC_RETA                  (0x5c00)   /* Redirection Table */
#define IGC_RSSRK                 (0x5c80)   /* RSS Random Key Register */

/* Transmit registers */

#define IGC_TCTL                  (0x0400)   /* Transmit Control */
#define IGC_TCTLEXT               (0x0404)   /* Transmit Control Extended */
#define IGC_TIPG                  (0x0410)   /* Transmit IPG Register */
#define IGC_RETXCTL               (0x041c)   /* Retry Buffer Control */
#define IGC_TXPBSIZE              (0x3404)   /* Transmit Packet Buffer Size */
#define IGC_DTXTCPFLGL            (0x359c)   /* DMA Tx TCP Flags Control Low */
#define IGC_DTXTCPFLGH            (0x35a0)   /* DMA Tx TCP Flags Control High */
#define IGC_DTXMXSZRQ             (0x3540)   /* DMA Tx Max Total Allow Size Requests */
#define IGC_DTXMXPKTSZ            (0x355c)   /* DMA Tx Max Allowable Packet Size */
#define IGC_DTXCTL                (0x3590)   /* DMA Tx Control */
#define IGC_TDBAL0                (0xe000)   /* Tx Descriptor Base Low */
#define IGC_TDBAH0                (0xe004)   /* Tx Descriptor Base High*/
#define IGC_TDLEN0                (0xe008)   /* Tx Descriptor Ring Length */
#define IGC_TDH0                  (0xe010)   /* Tx Descriptor Head */
#define IGC_TDT0                  (0xe018)   /* Tx Descriptor Tail */
#define IGC_TXDCTL0               (0xe028)   /* Transmit Descriptor Control Queue */
#define IGC_TXCTL0                (0xe014)   /* Tx DCA CTRL Register Queue */
#define IGC_TDWBAL0               (0xe038)   /* Transmit Descriptor WB Address Low Queue */
#define IGC_TDWBAH0               (0xe03c)   /* Transmit Descriptor WB Address High Queue */

/* Transmit Scheduling Registers */

#define IGC_TQAVHC                (0x300c)   /* Transmit Qav High Credits */
#define IGC_TQAVCTRL              (0x3570)   /* Transmit Qav Control */

/* TODO: Filters */

/* TODO: Per Queue Statistics */

/* Statistic registers */

#define IGC_CRCERRS               (0x04000)  /* CRC Error Count */
#define IGC_ALGNERRC              (0x04004)  /* Alignment Error Count */
#define IGC_RXERRC                (0x0400c)  /* RX Error Count */
#define IGC_MPC                   (0x04010)  /* Missed Packets Count */
#define IGC_SCC                   (0x04014)  /* Single Collision Count */
#define IGC_ECOL                  (0x04018)  /* Excessive Collisions Count */
#define IGC_MCC                   (0x0401c)  /* Multiple Collision Count */
#define IGC_LATECOL               (0x04020)  /* Late Collisions Count */
#define IGC_COLC                  (0x04028)  /* Collision Count */
#define IGC_DC                    (0x04030)  /* Defer Count */
#define IGC_TNCRS                 (0x04034)  /* Transmit with No CRS */
#define IGC_CEXTERR               (0x0403c)  /* Carrier Extension Error Count */
#define IGC_RLEC                  (0x04040)  /* Receive Length Error Count */
#define IGC_XONRXC                (0x04048)  /* XON Received Count */
#define IGC_XONTXC                (0x0404c)  /* XON Transmitted Count */
#define IGC_XOFFRXC               (0x04050)  /* XOFF Received Count */
#define IGC_XOFFTXC               (0x04054)  /* XOFF Transmitted Count */
#define IGC_FCRUC                 (0x04058)  /* FC Received Unsupported Count */
#define IGC_PRC64                 (0x0405c)  /* Packets Received [64 Bytes] Count */
#define IGC_PRC127                (0x04060)  /* Packets Received [65–127 Bytes] Count */
#define IGC_PRC255                (0x04064)  /* Packets Received [128–255 Bytes] */
#define IGC_PRC511                (0x04068)  /* Packets Received [256–511 Bytes] */
#define IGC_PRC1023               (0x0406c)  /* Packets Received [512–1023 Bytes] */
#define IGC_PRC1522               (0x04070)  /* Packets Received [1024 to Max Bytes] */
#define IGC_GPRC                  (0x04074)  /* Good Packets Received Count */
#define IGC_BPRC                  (0x04078)  /* Broadcast Packets Received Count */
#define IGC_MPRC                  (0x0407c)  /* Multicast Packets Received Count */
#define IGC_GPTC                  (0x04080)  /* Good Packets Transmitted Count */
#define IGC_GORCL                 (0x04088)  /* Good Octets Received Count Low */
#define IGC_GORCH                 (0x0408c)  /* Good Octets Received Count High */
#define IGC_GOTCL                 (0x04090)  /* Good Octets Transmitted Count Low */
#define IGC_GOTCH                 (0x04094)  /* Good Octets Transmitted Count High */
#define IGC_RNBC                  (0x040a0)  /* Receive No Buffers Count */
#define IGC_RUC                   (0x040a4)  /* Receive Undersize Count */
#define IGC_RFC                   (0x040a8)  /* Receive Fragment Count */
#define IGC_ROC                   (0x040ac)  /* Receive Oversize Count */
#define IGC_RJC                   (0x040b0)  /* Receive Jabber Count */
#define IGC_MNGPRC                (0x040B4)  /* Management Packets Received Count */
#define IGC_MPDC                  (0x040B8)  /* Management Packets Dropped Count */
#define IGC_MPTC                  (0x040BC)  /* Management Packets Transmitted Count */
#define IGC_TORL                  (0x040C0)  /* Total Octets Received */
#define IGC_TORH                  (0x040C4)  /* Total Octets Received */
#define IGC_TOT                   (0x040C8)  /* Total Octets Transmitted */
#define IGC_TPR                   (0x040D0)  /* Total Packets Received */
#define IGC_TPT                   (0x040D4)  /* Total Packets Transmitted */
#define IGC_PTC64                 (0x040D8)  /* Packets Transmitted [64 Bytes] Count */
#define IGC_PTC127                (0x040DC)  /* Packets Transmitted [65–127 Bytes] Count */
#define IGC_PTC255                (0x040E0)  /* Packets Transmitted [128–255 Bytes] Count */
#define IGC_PTC511                (0x040E4)  /* Packets Transmitted [256–511 Bytes] Count */
#define IGC_PTC1023               (0x040E8)  /* Packets Transmitted [512–1023 Bytes] Count */
#define IGC_PTC1522               (0x040EC)  /* Packets Transmitted [Greater than 1024 Bytes] Count */
#define IGC_MCPTC                 (0x040F0)  /* Multicast Packets Transmitted Count */
#define IGC_BPTC                  (0x040F4)  /* Broadcast Packets Transmitted Count */
#define IGC_TSCTC                 (0x040F8)  /* TCP Segmentation Context Transmitted Count */
#define IGC_TSCTFC                (0x040FC)  /* TCP Segmentation Context Transmit Fail Count */
#define IGC_IAC                   (0x04100)  /* Interrupt Assertion Count */
#define IGC_RPTHC                 (0x04104)  /* Rx Packets to Host Count */
#define IGC_TLPIC                 (0x04148)  /* EEE Tx LPI Count */
#define IGC_RLPIC                 (0x0414c)  /* EEE Rx LPI Count */
#define IGC_DBGC1                 (0x04108)  /* Debug counter 1 */
#define IGC_DBGC2                 (0x0410c)  /* Debug counter 2 */
#define IGC_DBGC3                 (0x04110)  /* Debug counter 3 */
#define IGC_DBGC4                 (0x0411c)  /* Debug counter 4 */
#define IGC_HGPTC                 (0x04118)  /* Host Good Packets Transmitted Count */
#define IGC_RXDMTC                (0x04120)  /* Rx Descriptor Minimum Threshold Count */
#define IGC_HGORCL                (0x04128)  /* Host Good Octets Received Count (Lo) */
#define IGC_HGORCH                (0x0412c)  /* Host Good Octets Received Count (Hi) */
#define IGC_HGOTCL                (0x04130)  /* Host Good Octets Transmitted Count (Lo) */
#define IGC_HGOTCH                (0x04134)  /* Host Good Octets Transmitted Count (Hi) */
#define IGC_LENERRS               (0x04138)  /* Length Errors Count Register */

/* Wake Up and Proxying */

#define IGC_WUC                   (0x5800)   /* Wake Up Control Register */
#define IGC_WUFC                  (0x5808)   /* Wake Up Filter Control Register */
#define IGC_WUS                   (0x5810)   /* Wake Up Status Register */
#define IGC_MFUTP01               (0x5828)   /* Management Flex UDP/TCP Ports 0/1 */
#define IGC_MFUTP23               (0x5830)   /* Management Flex UDP/TCP Port 2/3 */
#define IGC_IPAV                  (0x5838)   /* IP Address Valid */

/* Management Register */

#define IGC_MANC                  (0x5820)   /* Management Control */
#define IGC_MNGONLY               (0x5864)   /* Management Only Traffic Register */

/* TODO: Host Slave Interface */

/* TODO: PCIe */

/* TODO: Memory Error Detection */

/* TODO: Power Management Registers */

/* TODO: Time Sync */

/* TODO: Time Sync Interrupt Registers */

/* TODO: Time Sync QAV Statistics */

/* Device Control Register */

#define IGC_CTRL_FD               (1 << 0)   /* Bit 0: Full-Duplex */
                                             /* Bit 1: Reserved */
#define IGC_CTRL_GIOMDIS          (1 << 2)   /* Bit 2: Link Reset */
                                             /* Bits 3-5: Reserved */
#define IGC_CTRL_SLU              (1 << 6)   /* Bit 6: Set Link Up */
                                             /* Bits 7-15: Reserved */
#define IGC_CTRL_SDP0GPIEN        (1 << 16)  /* Bit 16: General Purpose Interrupt Detection Enable for SDP0 */
#define IGC_CTRL_SDP1GPIEN        (1 << 17)  /* Bit 17: General Purpose Interrupt Detection Enable for SDP1 */
#define IGC_CTRL_SDP0DATA         (1 << 18)  /* Bit 18: SDP0 Data Value */
#define IGC_CTRL_SDP1DATA         (1 << 19)  /* Bit 19: SDP1 Data Value */

#define IGC_CTRL_ADVD3WUC         (1 << 20)  /* Bit 20: D3Cold Wakeup Capability Advertisement Enable */
#define IGC_CTRL_PWRMGMT          (1 << 21)  /* Bit 21: PHY Power-Management Enable */
#define IGC_CTRL_SDP0_IODIR       (1 << 22)  /* Bit 22: SDP0 Pin Directionality */
#define IGC_CTRL_SDP1_IODIR       (1 << 23)  /* Bit 23: SDP1 Pin Directionality */
                                             /* Bits 24-26: Reserved */
#define IGC_CTRL_RFCE             (1 << 27)  /* Bit 27: Receive Flow Control Enable */
#define IGC_CTRL_TFCE             (1 << 28)  /* Bit 28: Transmit Flow Control Enable */
#define IGC_CTRL_DEVRST           (1 << 29)  /* Bit 29: Device Reset */
#define IGC_CTRL_VME              (1 << 30)  /* Bit 30: VLAN Mode Enable */
#define IGC_CTRL_PHYRST           (1 << 31)  /* Bit 31: PHY Reset */

/* Device Status Regiuster */

#define IGC_STATUS_FD             (1 << 0)   /* Bit 0: Full Duplex */
#define IGC_STATUS_LU             (1 << 1)   /* Bit 1: Link Up */
                                             /* Bits 2-3: Reserved */
#define IGC_STATUS_TXOFF          (1 << 4)   /* Bit 4: Transmission Paused */
                                             /* Bit 5: Reserved */
#define IGC_STATUS_SPEED_SHFIT    (5)        /* Bits 6-7: Link speed setting */
                                             /* Bits 8-9: Reserved */
#define IGC_STATUS_PHYRA          (10)       /* Bit 10: PHY Reset Asserted */
                                             /* Bits 11-18: Reserved */
#define IGC_STATUS_GIOM           (19)       /* Bit 19: GIO Master Disable bit state */
#define IGC_STATUS_DEVRSTSET      (20)       /* Bit 20: Device Reset Set */
#define IGC_STATUS_RST_DONE       (21)       /* Bit 21: RST_DONE */
#define IGC_STATUS_SPEED2P5       (22)       /* Bit 22: Link Speed Indication for 2.5Gb/s */
                                             /* Bits 23-30: Reserved */
#define IGC_STATUS_LPIIGNORE      (31)       /* Bit 31: Enable GTX clock out during LPI */
                                             /* Bit 31: Reserved */

/* Receive Address High */

#define IGC_RAH_RAH_MASK          (0xffff)   /* Bits 0-15: Receive address High */
#define IGC_RAH_ASEL              (16)       /* Bits 16-17 Address Select */
#define IGC_RAH_QSEL              (18)       /* Bits 18-19 Address Select */
                                             /* Bits 20-27: Reserved */
#define IGC_RAH_QSELEN            (1 << 28)  /* Bit 28: Queue Select Enable */
                                             /* Bits 29-30: Reserved */
#define IGC_RAH_AV                (1 << 31)  /* Bit 31: Address Valid */

/* Transmit Control */

                                             /* Bit 0: Reserved */
#define IGC_TCTL_EN               (1 << 1)   /* Bit 1: Transmit Enable */
                                             /* Bit 2: Reserved */
#define IGC_TCTL_PSP              (1 << 3)   /* Bit 3: Pad Short Packets */
#define IGC_TCTL_CT_SHIFT         (4)        /* Bits 4-11: Collision Threshold */
#define IGC_TCTL_COLD_SHIFT       (12)       /* Bits 12-21: Collision Distance (BST) */
#define IGC_TCTL_SWXOFF           (1 << 22)  /* Bit 22: Software XOFF Transmission */
                                             /* Bit 23: Reserved */
#define IGC_TCTL_RTLC             (1 << 24)  /* Bit 24: Re-transmit on Late Collision */
                                             /* Bits 25-32: Reserved */

/* Transmit Descriptor Control */

#define IGC_TXDCTL_PTHRESH_SHIFT  (0)       /* Bits 0-5: Prefetch Threshold */
                                            /* Bits 6-7: Reserved */
#define IGC_TXDCTL_HTHRESH_SHIFT  (8)       /* Bits 8-13: Host Threshold */
                                            /* Bits 14-15: Reserved */
#define IGC_TXDCTL_WTHRESH_SHIFT  (16)      /* Bits 16-21: Write-Back Threshold */
                                            /* Bits 21-24: Reserved */
#define IGC_TXDCTL_ENABLE         (1 << 25) /* Bit 25: Transmit Queue Enable */
#define IGC_TXDCTL_SWFLSH         (1 << 26) /* Bit 26: Transmit Software Flush */
#define IGC_TXDCTL_PRIORITY       (1 << 27) /* Bit 27: Transmit Queue Priority */
#define IGC_TXDCTL_HWBTHRESH      (28)      /* Bits 28-31: Transmit Head Write-back Threshold */

/* Tx Descriptor Ring Length */

/* Receive Control */

                                             /* Bit 0: Reserved */
#define IGC_RCTL_EN               (1 << 1)   /* Bit 1: Receiver Enable */
#define IGC_RCTL_SBP              (1 << 2)   /* Bit 2: Store Bad Packets */
#define IGC_RCTL_UPE              (1 << 3)   /* Bit 3: Unicast Promiscuous Enabled */
#define IGC_RCTL_MPE              (1 << 4)   /* Bit 4: Multicast Promiscuous Enabled */
#define IGC_RCTL_LPE              (1 << 5)   /* Bit 5: Long Packet Reception Enable */
#define IGC_RCTL_LBM_SHIFT        (6)        /* Bits 6-7: Loopback mode */
#define IGC_RCTL_HSEL_SHIFT       (8)        /* Bits 8-9: Hash Select for the Multicast Table Array (MTA) */
                                             /* Bits 10-11: Reserved */
#define IGC_RCTL_MO_SHIFT         (12)       /* Bits 12-13: Multicast Offset */
                                             /* Bit 14: Reserved */
#define IGC_RCTL_BAM              (1 << 15)  /* Bit 15: Broadcast Accept Mode */
#define IGC_RCTL_BSIZE_SHIFT      (16)       /* Bits 16-17: Receive Buffer Size */
#  define IGC_RCTL_BSIZE_2048     (0 << 16)  /* 00b: 2048 bytes */
#  define IGC_RCTL_BSIZE_1024     (1 << 16)  /* 01b: 1024 bytes */
#  define IGC_RCTL_BSIZE_512      (2 << 16)  /* 10b: 512 bytes */
#  define IGC_RCTL_BSIZE_256      (3 << 16)  /* 11b: 256 bytes */
#define IGC_RCTL_VFE              (1 << 18)  /* Bit 18: VLAN Filter Enable */
#define IGC_RCTL_CFIEN            (1 << 18)  /* Bit 19: Canonical Form Indicator Enable */
#define IGC_RCTL_CFI              (1 << 20)  /* Bit 20: Canonical Form Indicator bit value */
#define IGC_RCTL_PSP              (1 << 21)  /* Bit 21: Pad Small Receive Packets */
#define IGC_RCTL_DPF              (1 << 22)  /* Bit 22: Discard Pause Frames */
#define IGC_RCTL_PMCF             (1 << 23)  /* Bit 23: Pass MAC Control Frames */
                                             /* Bits 24-25: Reserved */
#define IGC_RCTL_SECRC            (1 << 26)  /* Bit 26: Strip Ethernet CRC from incoming packet */
                                             /* Bits 27-31: Reserved */

/* Receive Descriptor Control */

#define IGC_RXDCTL_PTHRESH_SHIFT  (0)       /* Bits 0-4: Prefetch Threshold */
                                            /* Bits 5-7: Reserved */
#define IGC_RXDCTL_HTHRESH_SHIFT  (8)       /* Bits 8-12: Host Threshold */
                                            /* Bits 13-15: Reserved */
#define IGC_RXDCTL_WTHRESH_SHIFT  (16)      /* Bits 16-20: Write-Back Threshold */
                                            /* Bits 21-24: Reserved */
#define IGC_RXDCTL_ENABLE         (1 << 25) /* Bit 25: Receive Queue Enable */
#define IGC_RXDCTL_SWFLUSH        (1 << 26) /* Bit 26: Receive Software Flush */
                                            /* Bits 27-31: Reserved */

/* Interrupt Cause */

#define IGC_IC_TXDW               (1 << 0)   /* Bit 0: Transmit Descriptor Written Back */
                                             /* Bit 1: Reserved */
#define IGC_IC_LSC                (1 << 2)   /* Bit 2: Link Status Change */
                                             /* Bit 3: Reserved */
#define IGC_IC_RXDMT0             (1 << 4)   /* Bit 4: Receive Descriptor Minimum Threshold Reached */
                                             /* Bit 5: Reserved */
#define IGC_IC_RXMISS             (1 << 6)   /* Bit 6: Receiver Miss Interrupt */
#define IGC_IC_RXDW               (1 << 7)   /* Bit 7: Receiver Descriptor Write Back interrupt */
                                             /* Bit 8: Reserved */
#define IGC_IC_MDAC               (1 << 9)   /* Bit 9: MDI/O Access Complete */
#define IGC_IC_GPHY               (1 << 10)  /* Bit 10: Internal PHY interrupt */
                                             /* Bits 11-15: General Purpose Interrupts */
#define IGC_IC_PTRAP              (1 << 15)  /* Bit 15: Probe trap interrupt */
                                             /* Bits 16-17: Reserved */
#define IGC_IC_MNG                (1 << 18)  /* Bit 18: Management Event interrupt. */
#define IGC_IC_TIMESYNC           (1 << 19)  /* Bit 19: Time_Sync interrupt */
                                             /* Bits 20-21: Reserved */
#define IGC_IC_FER                (1 << 22)  /* Bit 22: Fatal Error interrupt */
                                             /* Bit 23: Reserved */
#define IGC_IC_PCIEXCEPT          (1 << 24)  /* Bit 24: PCI Exception interrupt */
#define IGC_IC_SCE                (1 << 25)  /* Bit 25: DMA Coalescing Clock Control Event */
#define IGC_IC_SWD                (1 << 26)  /* Bit 26: Software Watchdog interrupt */
                                             /* Bit 27: Reserved */
#define IGC_IC_MDDET              (1 << 28)  /* Bit 28: Detected Malicious driver behavior Interrupt */
#define IGC_IC_TCPTIMER           (1 << 29)  /* Bit 29: TCP timer interrupt */
#define IGC_IC_DRSTA              (1 << 30)  /* Bit 30: Device Reset Asserted interrupt */
                                             /* Bit 31: Reserved */

/* Extended Interrupt Mask for Non-MSI-X Mode */

#define IGC_EIMS_NOMSIX_RXTX0     (1 << 0)
#define IGC_EIMS_NOMSIX_RXTX1     (1 << 1)
#define IGC_EIMS_NOMSIX_RXTX2     (1 << 2)
#define IGC_EIMS_NOMSIX_RXTX3     (1 << 3)
#define IGC_EIMS_NOMSIX_TCPTIM    (1 << 30)
#define IGC_EIMS_NOMSIX_OTHER     (1 << 31)

/* Extended Interrupt Mask for MSI-X Mode */

#define IGC_EIMS_MSIX_0           (1 << 0)
#define IGC_EIMS_MSIX_1           (1 << 1)
#define IGC_EIMS_MSIX_2           (1 << 2)
#define IGC_EIMS_MSIX_3           (1 << 3)
#define IGC_EIMS_MSIX_4           (1 << 4)

/* Interrupt Vector Allocation Registers */

#define IGC_IVAR0_RXQ0_SHIFT      (0)        /* Bits 0-4: MSI-X vector assigned to RxQ0 */
#define IGC_IVAR0_RXQ0_VAL        (1 << 7)   /* Bit 7: Valid bit for RxQ0 */
#define IGC_IVAR0_TXQ0_SHIFT      (8)        /* Bits 8-12: MSI-X vector assigned to TxQ0 */
#define IGC_IVAR0_TXQ0_VAL        (1 << 7)   /* Bit 7: Valid bit for TxQ0 */

/* Interrupt Vector Allocation Registers - Misc */

#define IGC_IVARMSC_TCPTIM        (0)        /* Bits 0-5: MSI-X vectorassigned to TCP timer interrupt */
                                             /* Bits 5-6: Reserved */
#define IGC_IVARMSC_TCPTIM_VAL    (1 << 7)   /* Bit 7: Enable bit for TCP timer interrupt */
#define IGC_IVARMSC_OTHER_SHIFT   (8)        /* Bits 8-12: MSI-X vector assigned to Ohter Cause */
                                             /* Bits 13-14: Reserved */
#define IGC_IVARMSC_OTHER_VAL     (1 << 15)  /* Bit 15: Enable bit for Ohter Cause */
                                             /* Bits 20-30: Reserved */

/* General Purpose Interrupt Enable */

#define IGC_GPIE_NSICR            (1 << 0)   /* Bit 0: Non Selective Interrupt Clear on Read */
                                             /* Bits 1-3: Reserved */
#define IGC_GPIE_MSIX             (1 << 4)   /* Bit 4: MSI-X mode */
                                             /* Bits 5-6: Reserved */
#define IGC_GPIE_LLINTERVAL_SHIFT (7)        /* Bits 7-11: Low Latency Credits Increment Rate */
                                             /* Bits 12-29: Reserved */
#define IGC_GPIE_EIAME            (1 << 30)  /* Bit 30: Extended Interrupt Auto Mask Enable */
#define IGC_GPIE_PBASUPPORT       (1 << 31)  /* Bit 31: PBA Support */

/* Transmit Descriptor Command Field */

#define IGC_TDESC_CMD_EOP         (1 << 0)   /* Bit 0: End Of Packet */
#define IGC_TDESC_CMD_IFCS        (1 << 1)   /* Bit 1: Insert FCS */
#define IGC_TDESC_CMD_IC          (1 << 2)   /* Bit 2: Insert Checksum */
#define IGC_TDESC_CMD_RS          (1 << 3)   /* Bit 3: Report Status */
#define IGC_TDESC_CMD_DEXT        (1 << 5)   /* Bit 5: Extension (0b for legacy mode) */
#define IGC_TDESC_CMD_VLE         (1 << 6)   /* Bit 6: VLAN Packet Enable */

/* Transmit Descriptor Status Field */

#define IGC_TDESC_STATUS_DD       (1 << 0)   /* Bit 0: Descriptor Done */

/* Transmit Descriptor Special Field */

#define IGC_TDESC_SPEC_VLAN_SHIFT (0)        /* Bits 0-11: VLAN Identifier */
#define IGC_TDESC_SPEC_CFI        (1 << 12)  /* Bit 12: Canonical Form Indicator */
#define IGC_TDESC_SPEC_PRI_SHIFT  (13)       /* Bits 13-15: User Priority */

/* Receive Descriptor Status Field */

#define IGC_RDESC_STATUS_DD       (1 << 0)   /* Bit 0: Descriptor Done */
#define IGC_RDESC_STATUS_EOP      (1 << 1)   /* Bit 1: End of Packet */
#define IGC_RDESC_STATUS_VP       (1 << 3)   /* Bit 3: Packet is 802.1Q (matched VET) */
#define IGC_RDESC_STATUS_UDPCS    (1 << 4)   /* Bit 4: UDP checksum or IP payload checksum calculated on packet */
#define IGC_RDESC_STATUS_L4CS     (1 << 5)   /* Bit 5: L4 (UDP or TCP) checksum calculated on packet */
#define IGC_RDESC_STATUS_IPCS     (1 << 6)   /* Bit 6: IP Checksum Calculated on Packet */
#define IGC_RDESC_STATUS_PIF      (1 << 7)   /* Bit 7: Passed in-exact filter */

/* Receive Descriptor Errors Field */

#define IGC_RDESC_ERRORS_L4E      (1 << 5)   /* Bit 5: TCP/UDP Checksum Error */
#define IGC_RDESC_ERRORS_IPE      (1 << 6)   /* Bit 6: IP Checksum Error */
#define IGC_RDESC_ERRORS_RXE      (1 << 7)   /* Bit 7: RX Data Error */

/* Receive Descriptor Special Field */

#define IGC_RDESC_SPEC_VLAN_SHIFT (0)        /* Bits 0-11: VLAN Identifier */
#define IGC_RDESC_SPEC_CFI        (1 << 12)  /* Bit 12: Canonical Form Indicator */
#define IGC_RDESC_SPEC_PRI_SHIFT  (13)       /* Bits 13-15: User Priority */

/*****************************************************************************
 * Public Types
 *****************************************************************************/

/* Legacy TX descriptor */

begin_packed_struct struct igc_tx_leg_s
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

begin_packed_struct struct igc_rx_leg_s
{
  uint64_t addr;
  uint16_t len;
  uint16_t checksum;
  uint8_t  status;
  uint8_t  errors;
  uint16_t special;
} end_packed_struct;

#endif  /* __DRIVERS_NET_IGC_H */
