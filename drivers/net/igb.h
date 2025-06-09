/*****************************************************************************
 * drivers/net/igb.h
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

#ifndef __DRIVERS_NET_IGB_H
#define __DRIVERS_NET_IGB_H

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include <stdint.h>

/*****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/

/* General registers */

#define IGB_CTRL                  (0x0000)   /* Device Control Register */
#define IGB_STATUS                (0x0008)   /* Device Status Register  */
#define IGB_CTRLEXT               (0x0018)   /* Extended Device Control Register */
#define IGB_MDIC                  (0x0020)   /* MDI Control Register */
#define IGB_SERDESCTL             (0x0024)   /* Serdes_ana */
#define IGB_FCAL                  (0x0028)   /* Flow Control Address Low */
#define IGB_FCAH                  (0x002c)   /* Flow Control Address High */
#define IGB_FCT                   (0x0030)   /* Flow Control Type */
#define IGB_CONNSW                (0x0034)   /* Copper/Fiber Switch Control */
#define IGB_VET                   (0x0038)   /* VLAN Ether Type */
#define IGB_FCTTV                 (0x0170)   /* Flow Control Transmit Timer Value */
#define IGB_LEDCTL                (0x0e00)   /* LED Control */
#define IGB_I2CCM                 (0x1028)   /* SFP I2C Command */
#define IGB_I2CPARAMS             (0x102c)   /* SFP I2C Parameter */
#define IGB_WDSTP                 (0x1040)   /* Watchdog Setup Register */
#define IGB_WDSWSTS               (0x1044)   /* Watchdog Software */
#define IGB_FRTIMER               (0x1048)   /* Free Running Timer */
#define IGB_TCPTIMER              (0x104c)   /* TCP Timer */
#define IGB_DCAID                 (0x5b70)   /* DCA Requester ID Information Register */
#define IGB_SWSM                  (0x5b50)   /* Software Semaphore Register */
#define IGB_FWSM                  (0x5b54)   /* Firmware Semaphore Register */
#define IGB_SWFWSYNC              (0x5b5c)   /* Software-Firmware Synchronization */

/* Flash/EEPROM Registers */

#define IGB_EEC                   (0x00010)  /* EEPROM/FLASH Control Register */
#define IGB_EERD                  (0x00014)  /* EEPROM Read Register */
#define IGB_FLA                   (0x0001c)  /* Flash Access Register */
#define IGB_EEMNGCTL              (0x01010)  /* MNG EEPROM Control Register */
#define IGB_EEMNGDATA             (0x01014)  /* MNG EEPROM Read/Write data */
#define IGB_FLMNGCTL              (0x01018)  /* MNG Flash Control Register */
#define IGB_FLMNGDATA             (0x0101c)  /* MNG Flash Read data */
#define IGB_FLMNGCNT              (0x01020)  /* MNG Flash Read Counter */
#define IGB_EEARBC                (0x01024)  /* EEPROM Auto Read Bus Control */
#define IGB_FLASHOP               (0x0103c)  /* Flash Opcode Register */
#define IGB_EEDIAG                (0x01038)  /* EEPROM Diagnostic */
#define IGB_VPDDIAG               (0x01060)  /* VPD Diagnostic */

/* Interrupt registers */

#define IGB_ICR                   (0x1500)   /* Interrupt Cause Read */
#define IGB_ICS                   (0x1504)   /* Interrupt Cause Set */
#define IGB_IMS                   (0x1508)   /* Interrupt Mask Set */
#define IGB_IMC                   (0x150c)   /* Interrupt Mask Clear */
#define IGB_IAM                   (0x1510)   /* Interrupt Acknowledge Auto–Mask */
#define IGB_EICS                  (0x1520)   /* Extended Interrupt Cause Set */
#define IGB_EIMS                  (0x1524)   /* Extended Interrupt Mask Set/Read */
#define IGB_EIMC                  (0x1528)   /* Extended Interrupt Mask Clear */
#define IGB_EIAC                  (0x152c)   /* Extended Interrupt Auto Clear */
#define IGB_EIAM                  (0x1530)   /* Extended Interrupt Auto Mask */
#define IGB_EICR                  (0x1580)   /* Extended Interrupt Cause Read */
#define IGB_IVAR0                 (0x1700)   /* Interrupt Vector Allocation Registers  */
#define IGB_IVARMSC               (0x1740)   /* Interrupt Vector Allocation Registers - MISC */
#define IGB_EITR0                 (0x1680)   /* Extended Interrupt Throttling Rate 0 - 24 */
#define IGB_GPIE                  (0x1514)   /* General Purpose Interrupt Enable */
#define IGB_PBACL                 (0x5b68)   /* MSI-X PBA Clear */

/* Receive registers */

#define IGB_RCTL                  (0x0100)   /* Receive Control */
#define IGB_FCRTL0                (0x2160)   /* Flow Control Receive Threshold Low */
#define IGB_PSRCTL                (0x2170)   /* Packet Split Receive Control Register */
#define IGB_FCRTH0                (0x2168)   /* Flow Control Receive Threshold High */
#define IGB_RXPBSIZE              (0x2404)   /* Rx Packet Buffer Size */
#define IGB_PBRWAC                (0x24e8)   /* Rx Packet Buffer wrap around counter */
#define IGB_FCRTV                 (0x2460)   /* Flow Control Refresh Threshold Value */
#define IGB_DRXMXOD               (0x2540)   /* DMA RX Max Total Allow Size Requests */
#define IGB_RDBAL0                (0xc000)   /* Tx Descriptor Base Address Low */
#define IGB_RDBAH0                (0xc004)   /* Rx Descriptor Base Address High */
#define IGB_RDLEN0                (0xc008)   /* Rx Descriptor Length */
#define IGB_SRRCTL0               (0xc00c)   /* Split and Replication Receive Control Register Queue */
#define IGB_RDH0                  (0xc010)   /* Rx Descriptor Head */
#define IGB_RDT0                  (0xc018)   /* Rx Descriptor Tail */
#define IGB_RXDCTL0               (0xc028)   /* Receive Descriptor Control Queue */
#define IGB_RXCTL0                (0xc014)   /* Receive Queue DCA CTRL Register */
#define IGB_RXCSUM                (0x5000)   /* Receive Checksum Control */
#define IGB_RLPML                 (0x5004)   /* Receive Long packet maximal length */
#define IGB_RFCTL                 (0x5008)   /* Receive Filter Control Register */
#define IGB_MTA                   (0x5200)   /* Multicast Table Array (n) */
#define IGB_RAL                   (0x5400)   /* Receive Address Low */
#define IGB_RAH                   (0x5404)   /* Receive Address Low */
#define IGB_PSRTYPE0              (0x5480)   /* Packet Split Receive type (n) */
#define IGB_RPLPSRTYPE            (0x54c0)   /* Replicated Packet Split Receive type */
#define IGB_VTCTL                 (0x5818)   /* Next Generation VMDq Control register */
#define IGB_VFTA0                 (0x5600)   /* VLAN Filter Table Array (n) */
#define IGB_MRQC                  (0x5818)   /* Multiple Receive Queues Command */
#define IGB_RETA                  (0x5c00)   /* Redirection Table */
#define IGB_RSSRK                 (0x5c80)   /* RSS Random Key Register */

/* Transmit registers */

#define IGB_TXPBSIZE              (0x3404)   /* Transmit Packet Buffer Size */
#define IGB_PBTWAC                (0x34e8)   /* Tx Packet Buffer wrap around counter */
#define IGB_TCTL                  (0x0400)   /* Transmit Control */
#define IGB_TCTLEXT               (0x0404)   /* Transmit Control Extended */
#define IGB_TIPG                  (0x0410)   /* Transmit IPG Register */
#define IGB_RETXCTL               (0x041c)   /* Retry Buffer Control */
#define IGB_DTXCTL                (0x3590)   /* DMA Tx Control */
#define IGB_DTXTCPFLGL            (0x359c)   /* DMA Tx TCP Flags Control Low */
#define IGB_DTXTCPFLGH            (0x35a0)   /* DMA Tx TCP Flags Control High */
#define IGB_DTXMXSZRQ             (0x3540)   /* DMA Tx Max Total Allow Size Requests */
#define IGB_DTXMXPKTSZ            (0x355c)   /* DMA Tx Max Allowable Packet Size */
#define IGB_TDBAL0                (0xe000)   /* Tx Descriptor Base Low */
#define IGB_TDBAH0                (0xe004)   /* Tx Descriptor Base High*/
#define IGB_TDLEN0                (0xe008)   /* Tx Descriptor Ring Length */
#define IGB_TDH0                  (0xe010)   /* Tx Descriptor Head */
#define IGB_TDT0                  (0xe018)   /* Tx Descriptor Tail */
#define IGB_TXDCTL0               (0xe028)   /* Transmit Descriptor Control Queue */
#define IGB_TXCTL0                (0xe014)   /* Tx DCA CTRL Register Queue */
#define IGB_TDWBAL0               (0xe038)   /* Transmit Descriptor WB Address Low Queue */
#define IGB_TDWBAH0               (0xe03c)   /* Transmit Descriptor WB Address High Queue */

/* TODO: Filters */

#define IGB_ETQF0                 (0x5cb0)   /* EType Queue Filter */
#define IGB_IMIR0                 (0x5a80)   /* Immediate Interrupt Rx */
#define IGB_IMIREXT0              (0x5aa9)   /* Immediate Interrupt Rx Extended */
#define IGB_IMIRVP                (0x5ac0)   /* Immediate Interrupt Rx VLAN Priority */
#define IGB_SAQF0                 (0x5980)   /* Source Address Queue Filter */
#define IGB_DAQF0                 (0x59a0)   /* Destination Address Queue Filter */
#define IGB_SPQF0                 (0x59c0)   /* Source Port Queue Filter */
#define IGB_FTQF0                 (0x59e0)   /* Five-Tuple Queue Filter */
#define IGB_SYNQF                 (0x55fc)   /* SYN Packet Queue Filter */

/* TODO: Virtualization */

/* Statistic registers */

#define IGB_CRCERRS               (0x04000)  /* CRC Error Count */
#define IGB_ALGNERRC              (0x04004)  /* Alignment Error Count */
#define IGB_SYMERRS               (0x04008)  /* Symbol Error Count */
#define IGB_RXERRC                (0x0400c)  /* RX Error Count */
#define IGB_MPC                   (0x04010)  /* Missed Packets Count */
#define IGB_SCC                   (0x04014)  /* Single Collision Count */
#define IGB_ECOL                  (0x04018)  /* Excessive Collisions Count */
#define IGB_MCC                   (0x0401c)  /* Multiple Collision Count */
#define IGB_LATECOL               (0x04020)  /* Late Collisions Count */
#define IGB_COLC                  (0x04028)  /* Collision Count */
#define IGB_DC                    (0x04030)  /* Defer Count */
#define IGB_TNCRS                 (0x04034)  /* Transmit with No CRS */
#define IGB_HTDPMC                (0x0403c)  /* Host Transmit Discarded Packets by MAC Count */
#define IGB_RLEC                  (0x04040)  /* Receive Length Error Count */
#define IGB_CBRDPC                (0x04044)  /* Circuit Breaker Rx dropped packet */
#define IGB_XONRXC                (0x04048)  /* XON Received Count */
#define IGB_XONTXC                (0x0404c)  /* XON Transmitted Count */
#define IGB_XOFFRXC               (0x04050)  /* XOFF Received Count */
#define IGB_XOFFTXC               (0x04054)  /* XOFF Transmitted Count */
#define IGB_FCRUC                 (0x04058)  /* FC Received Unsupported Count */
#define IGB_PRC64                 (0x0405c)  /* Packets Received [64 Bytes] Count */
#define IGB_PRC127                (0x04060)  /* Packets Received [65–127 Bytes] Count */
#define IGB_PRC255                (0x04064)  /* Packets Received [128–255 Bytes] */
#define IGB_PRC511                (0x04068)  /* Packets Received [256–511 Bytes] */
#define IGB_PRC1023               (0x0406c)  /* Packets Received [512–1023 Bytes] */
#define IGB_PRC1522               (0x04070)  /* Packets Received [1024 to Max Bytes] */
#define IGB_GPRC                  (0x04074)  /* Good Packets Received Count */
#define IGB_BPRC                  (0x04078)  /* Broadcast Packets Received Count */
#define IGB_MPRC                  (0x0407c)  /* Multicast Packets Received Count */
#define IGB_GPTC                  (0x04080)  /* Good Packets Transmitted Count */
#define IGB_GORCL                 (0x04088)  /* Good Octets Received Count Low */
#define IGB_GORCH                 (0x0408c)  /* Good Octets Received Count High */
#define IGB_GOTCL                 (0x04090)  /* Good Octets Transmitted Count Low */
#define IGB_GOTCH                 (0x04094)  /* Good Octets Transmitted Count High */
#define IGB_RNBC                  (0x040a0)  /* Receive No Buffers Count */
#define IGB_RUC                   (0x040a4)  /* Receive Undersize Count */
#define IGB_RFC                   (0x040a8)  /* Receive Fragment Count */
#define IGB_ROC                   (0x040ac)  /* Receive Oversize Count */
#define IGB_RJC                   (0x040b0)  /* Receive Jabber Count */
#define IGB_MNGPRC                (0x040b4)  /* Management Packets Received Count */
#define IGB_MPDC                  (0x040b8)  /* Management Packets Dropped Count */
#define IGB_MNGPTC                (0x040bc)  /* Management Packets Transmitted Count */
#define IGB_BMNGPRC               (0x0414c)  /* BMC Management Packets Receive Count */
#define IGB_BMPDC                 (0x04140)  /* BMC Management Packets Dropped Count */
#define IGB_BMNGPTC               (0x04144)  /* BMC Management Packets Transmitted Count */
#define IGB_TORL                  (0x040c0)  /* Total Octets Received */
#define IGB_TORH                  (0x040c4)  /* Total Octets Received */
#define IGB_TOTL                  (0x040c8)  /* Total Octets Transmitted */
#define IGB_TOTH                  (0x040cc)  /* Total Octets Transmitted */
#define IGB_TPR                   (0x040d0)  /* Total Packets Received */
#define IGB_TPT                   (0x040d4)  /* Total Packets Transmitted */
#define IGB_PTC64                 (0x040d8)  /* Packets Transmitted [64 Bytes] Count */
#define IGB_PTC127                (0x040dc)  /* Packets Transmitted [65–127 Bytes] Count */
#define IGB_PTC255                (0x040e0)  /* Packets Transmitted [128–255 Bytes] Count */
#define IGB_PTC511                (0x040e4)  /* Packets Transmitted [256–511 Bytes] Count */
#define IGB_PTC1023               (0x040e8)  /* Packets Transmitted [512–1023 Bytes] Count */
#define IGB_PTC1522               (0x040ec)  /* Packets Transmitted [Greater than 1024 Bytes] Count */
#define IGB_MCPTC                 (0x040f0)  /* Multicast Packets Transmitted Count */
#define IGB_BPTC                  (0x040f4)  /* Broadcast Packets Transmitted Count */
                                             /* ... missing registers */

/* Wake Up and Proxying */

#define IGB_WUC                   (0x5800)   /* Wake Up Control Register */
#define IGB_WUFC                  (0x5808)   /* Wake Up Filter Control Register */
#define IGB_WUS                   (0x5810)   /* Wake Up Status Register */
#define IGB_WUPL                  (0x5900)   /* Wake Up Packet Length */
#define IGB_WUPM                  (0x5a00)   /* Wake Up Packet Memory */
#define IGB_FHFT                  (0x9000)   /* Flexible Host Filter Table registers */
#define IGB_FHFTEXT               (0x9a00)   /* Flexible Host Filter Table registers extended */

/* TODO: Management Register */

/* TODO: PCIe */

/* Diagnostic */

#define IGB_RDFH                  (0x2410)   /* RX Data FIFO Head */
#define IGB_RDFT                  (0x2418)   /* RX Data FIFO Tail */
#define IGB_RDFHS                 (0x2420)   /* RX Data FIFO Head Saved */
#define IGB_RDFTS                 (0x2428)   /* RX Data FIFO Tail Saved */
#define IGB_RDFPC                 (0x2430)   /*  Receive Data FIFO Packet Count */
#define IGB_RPBECCSTS             (0x245c)   /* Receive Packet buffer ECC control */
#define IGB_TPBECCSTS             (0x345C)   /* Transmit Packet buffer ECC control */
#define IGB_FCSTS0                (0x2464)   /* Flow Control Status */
#define IGB_RDHESTS               (0x25c0)   /* Rx Descriptor Handler ECC status */
#define IGB_TDHESTS               (0x35c0)   /* Tx Descriptor Handler ECC status */
#define IGB_TDFH                  (0x3410)   /* TX Data FIFO Head */
#define IGB_TDFT                  (0x3418)   /* TX Data FIFO Tail */
#define IGB_TDFHS                 (0x3420)   /* TX Data FIFO Head Saved */
#define IGB_TDFTS                 (0x3428)   /* TX Data FIFO Tail Saved */
#define IGB_TDFPC                 (0x3430)   /* Transmit Data FIFO Packet Count */
#define IGB_TDHMP                 (0x35fc)   /* Tx Descriptor Handler Memory Page Number */
#define IGB_CIRC                  (0x0f00)   /* Circuits Control */
#define IGB_TXBDC                 (0x35e0)   /* Tx DMA Performance Burst and Descriptor Count */
#define IGB_TXIDLE                (0x35e4)   /* Tx DMA Performance Idle Count */
#define IGB_RXBDC                 (0x25e0)   /* Tx DMA Performance Burst and Descriptor Count */
#define IGB_RXIDLE                (0x25E4)   /* Tx DMA Performance Idle Count */

/* TODO: PCS */

/* TODO: Time Sync */

/* TODO: MACSec */

/* TODO: IPSec */

/* Device Control Register */

#define IGB_CTRL_FD               (1 << 0)   /* Bit 0: Full-Duplex */
                                             /* Bit 1: Reserved */
#define IGB_CTRL_GIOMDIS          (1 << 2)   /* Bit 2: GIO Master Enable Status */
#define IGB_CTRL_LRST             (1 << 3)   /* Bit 3: Link Reset */
                                             /* Bits 4-5: Reserved */
#define IGB_CTRL_SLU              (1 << 6)   /* Bit 6: Set Link Up */
#define IGB_CTRL_ILOS             (1 << 7)   /* Bit 7: Invert Loss-of-Signal */
#define IGB_CTRL_SPEED_SHIFT      (8)        /* Bits 8-9: Speed selection */
#define IGB_CTRL_SPEED_MASK       (3 << IGB_CTRL_SPEED_SHIFT)
#  define IGB_CTRL_SPEED_10MBS    (0 << IGB_CTRL_SPEED_SHIFT)
#  define IGB_CTRL_SPEED_100MBS   (1 << IGB_CTRL_SPEED_SHIFT)
#  define IGB_CTRL_SPEED_1000MBS  (2 << IGB_CTRL_SPEED_SHIFT)

                                             /* Bit 10: Reserved */
#define IGB_CTRL_FRCSPD           (1 << 11)  /* Bit 11: Force Speed */
#define IGB_CTRL_FRCDPLX          (1 << 12)  /* Bit 12: Force Duplex */
                                             /* Bits 13-15: Reserved */
#define IGB_CTRL_SDP0_GPIEN       (1 << 16)  /* Bit 16: GPI Detection Enable for SDP0 */
#define IGB_CTRL_SDP1_GPIEN       (1 << 17)  /* Bit 17: GPI Detection Enable for SDP1 */
#define IGB_CTRL_SDP0_DATA        (1 << 18)  /* Bit 18: SDP0 Data Value */
#define IGB_CTRL_SDP1_DATA        (1 << 19)  /* Bit 19: SDP1 Data Value */
#define IGB_CTRL_ADVD3WUC         (1 << 20)  /* Bit 20: D3Cold Wakeup Capability Advertisement Enable */
#define IGB_CTRL_SDP0_WDE         (1 << 21)  /* Bit 21: SDP0 Used for Watchdog Indication */
#define IGB_CTRL_SDP0_IODIR       (1 << 22)  /* Bit 22: SDP0 Pin Directionality */
#define IGB_CTRL_SDP1_IODIR       (1 << 23)  /* Bit 23: SDP1 Pin Directionality */
                                             /* Bits 24-25: Reserved */
#define IGB_CTRL_RST              (1 << 26)  /* Bit 26: Device Reset */
#define IGB_CTRL_RFCE             (1 << 27)  /* Bit 27: Receive Flow Control Enable */
#define IGB_CTRL_TFCE             (1 << 28)  /* Bit 28: Transmit Flow Control Enable */
                                             /* Bit 29: Reserved */
#define IGB_CTRL_VME              (1 << 30)  /* Bit 30: VLAN Mode Enable */
#define IGB_CTRL_PHYRST           (1 << 31)  /* Bit 31: PHY Reset */

/* Device Status Regiuster */

#define IGB_STATUS_FD             (1 << 0)   /* Bit 0: Full Duplex */
#define IGB_STATUS_LU             (1 << 1)   /* Bit 1: Link Up */
#define IGB_STATUS_LANID_SHIFT    (2)        /* Bit 2-3: LAN ID*/
#define IGB_STATUS_TXOFF          (1 << 4)   /* Bit 4: Transmission Paused */
                                             /* Bit 5: Reserved */
#define IGB_STATUS_SPEED_SHFIT    (6)        /* Bits 6-7: Link speed setting */
#define IGB_STATUS_ASDV_SHFIT     (8)        /* Bits 8-9: Auto-Speed Detection Value. */
#define IGB_STATUS_PHYRA          (10)       /* Bit 10: PHY Reset Asserted */
                                             /* Bits 11-13: Reserved */
#define IGB_STATUS_NVFS_SHFIT     (14)       /* Bits 14-17: Num VFs in the IOV capability */
#define IGB_STATUS_IOVMODE        (18)       /* Bit 18: VF enable (VFE) bit in the IOV */
#define IGB_STATUS_GIOM           (19)       /* Bit 19: GIO Master Disable bit state */
                                             /* Bits 20-30: Reserved */
#define IGB_STATUS_DMACGE         (31)       /* Bit 31: DMA clock gating Enable bit */

/* Receive Address High */

#define IGB_RAH_RAH_MASK          (0xffff)   /* Bits 0-15: Receive address High */
#define IGB_RAH_ASEL              (16)       /* Bits 16-17 Address Select */
#define IGB_RAH_POOLSEL           (18)       /* Bits 18-25 Pool Select */
                                             /* Bits 26-30: Reserved */
#define IGB_RAH_AV                (1 << 31)  /* Bit 31: Address Valid */

/* Transmit Control */

                                             /* Bit 0: Reserved */
#define IGB_TCTL_EN               (1 << 1)   /* Bit 1: Transmit Enable */
                                             /* Bit 2: Reserved */
#define IGB_TCTL_PSP              (1 << 3)   /* Bit 3: Pad Short Packets */
#define IGB_TCTL_CT_SHIFT         (4)        /* Bits 4-11: Collision Threshold */
#define IGB_TCTL_COLD_SHIFT       (12)       /* Bits 12-21: Collision Distance (BST) */
#define IGB_TCTL_SWXOFF           (1 << 22)  /* Bit 22: Software XOFF Transmission */
                                             /* Bit 23: Reserved */
#define IGB_TCTL_RTLC             (1 << 24)  /* Bit 24: Re-transmit on Late Collision */
                                             /* Bits 25-32: Reserved */

/* Transmit Descriptor Control */

#define IGB_TXDCTL_PTHRESH_SHIFT  (0)       /* Bits 0-4: Prefetch Threshold */
                                            /* Bits 5-7: Reserved */
#define IGB_TXDCTL_HTHRESH_SHIFT  (8)       /* Bits 8-12: Host Threshold */
                                            /* Bits 13-15: Reserved */
#define IGB_TXDCTL_WTHRESH_SHIFT  (16)      /* Bits 16-20: Write-Back Threshold */
                                            /* Bits 21-24: Reserved */
#define IGB_TXDCTL_ENABLE         (1 << 25) /* Bit 25: Transmit Queue Enable */
                                            /* Bits 27-31: Reserved */

/* Tx Descriptor Ring Length */

/* Receive Control */

                                             /* Bit 0: Reserved */
#define IGB_RCTL_EN               (1 << 1)   /* Bit 1: Receiver Enable */
#define IGB_RCTL_SBP              (1 << 2)   /* Bit 2: Store Bad Packets */
#define IGB_RCTL_UPE              (1 << 3)   /* Bit 3: Unicast Promiscuous Enabled */
#define IGB_RCTL_MPE              (1 << 4)   /* Bit 4: Multicast Promiscuous Enabled */
#define IGB_RCTL_LPE              (1 << 5)   /* Bit 5: Long Packet Reception Enable */
#define IGB_RCTL_LBM_SHIFT        (6)        /* Bits 6-7: Loopback mode */
                                             /* Bits 8-11: Reserved */
#define IGB_RCTL_MO_SHIFT         (12)       /* Bits 12-13: Multicast Offset */
                                             /* Bit 14: Reserved */
#define IGB_RCTL_BAM              (1 << 15)  /* Bit 15: Broadcast Accept Mode */
#define IGB_RCTL_BSIZE_SHIFT      (16)       /* Bits 16-17: Receive Buffer Size */
#  define IGB_RCTL_BSIZE_2048     (0 << 16)  /* 00b: 2048 bytes */
#  define IGB_RCTL_BSIZE_1024     (1 << 16)  /* 01b: 1024 bytes */
#  define IGB_RCTL_BSIZE_512      (2 << 16)  /* 10b: 512 bytes */
#  define IGB_RCTL_BSIZE_256      (3 << 16)  /* 11b: 256 bytes */
#define IGB_RCTL_VFE              (1 << 18)  /* Bit 18: VLAN Filter Enable */
#define IGB_RCTL_CFIEN            (1 << 18)  /* Bit 19: Canonical Form Indicator Enable */
#define IGB_RCTL_CFI              (1 << 20)  /* Bit 20: Canonical Form Indicator bit value */
#define IGB_RCTL_PSP              (1 << 21)  /* Bit 21: Pad Small Receive Packets */
#define IGB_RCTL_DPF              (1 << 22)  /* Bit 22: Discard Pause Frames */
#define IGB_RCTL_PMCF             (1 << 23)  /* Bit 23: Pass MAC Control Frames */
                                             /* Bits 24-25: Reserved */
#define IGB_RCTL_SECRC            (1 << 26)  /* Bit 26: Strip Ethernet CRC from incoming packet */
                                             /* Bits 27-31: Reserved */

/* Receive Descriptor Control */

#define IGB_RXDCTL_PTHRESH_SHIFT  (0)       /* Bits 0-4: Prefetch Threshold */
                                            /* Bits 5-7: Reserved */
#define IGB_RXDCTL_HTHRESH_SHIFT  (8)       /* Bits 8-12: Host Threshold */
                                            /* Bits 13-15: Reserved */
#define IGB_RXDCTL_WTHRESH_SHIFT  (16)      /* Bits 16-20: Write-Back Threshold */
                                            /* Bits 21-24: Reserved */
#define IGB_RXDCTL_ENABLE         (1 << 25) /* Bit 25: Receive Queue Enable */
#define IGB_RXDCTL_SWFLUSH        (1 << 26) /* Bit 26: Receive Software Flush */
                                            /* Bits 27-31: Reserved */

/* Interrupt Cause */

#define IGB_IC_TXDW               (1 << 0)   /* Bit 0: Transmit Descriptor Written Back */
                                             /* Bit 1: Reserved */
#define IGB_IC_LSC                (1 << 2)   /* Bit 2: Link Status Change */
                                             /* Bit 3: Reserved */
#define IGB_IC_RXDMT0             (1 << 4)   /* Bit 4: Receive Descriptor Minimum Threshold Reached */
#define IGB_IC_MACSEC             (1 << 5)   /* Bit 5: Sets the MACSec interrupt */
#define IGB_IC_RXMISS             (1 << 6)   /* Bit 6: Receiver Miss Interrupt */
#define IGB_IC_RXDW               (1 << 7)   /* Bit 7: Receiver Descriptor Write Back interrupt */
#define IGB_IC_VMMB               (1 << 8)   /* Bit 8: Sets the VM mailbox interrupt. */
                                             /* Bits 9-10: Reserved */
                                             /* Bits 11-14: General Purpose Interrupts */
#define IGB_IC_PTRAP              (1 << 15)  /* Bit 15: Probe trap interrupt */
                                             /* Bits 16-17: Reserved */
#define IGB_IC_MNG                (1 << 18)  /* Bit 18: Management Event interrupt */
                                             /* Bit 19: Reserved */
#define IGB_IC_OMED               (1 << 20)  /* Bit 20: Sets the Other Media Energy Detected Interrupt */
                                             /* Bit 21: Reserved */
#define IGB_IC_FER                (1 << 22)  /* Bit 22: Fatal Error interrupt */
#define IGB_IC_NFER               (1 << 23)  /* Bit 23: Sets the Non Fatal Error Interrupt */
#define IGB_IC_CSRTO              (1 << 24)  /* Bit 24: Sets the CSR access time out indication interrupt */
#define IGB_IC_SCE                (1 << 25)  /* Bit 25: Set the Storm Control Event Interrupt */
#define IGB_IC_SWD                (1 << 26)  /* Bit 26: Software Watchdog interrupt */
                                             /* Bit 27: Reserved */
#define IGB_IC_DOUTSYNC           (1 << 28)  /* Bit 28: Sets the DMA Tx Out of Sync Interrupt. */
                                             /* Bit 29: Reserved */
#define IGB_IC_TCPTIM             (1 << 30)  /* Bit 30: Sets the TCP timer interrupt */
                                             /* Bit 31: Reserved */

/* Extended Interrupt Mask for Non-MSI-X Mode */

#define IGB_EIMS_NOMSIX_RXTX0     (1 << 0)
#define IGB_EIMS_NOMSIX_RXTX1     (1 << 1)
#define IGB_EIMS_NOMSIX_RXTX2     (1 << 2)
#define IGB_EIMS_NOMSIX_RXTX3     (1 << 3)
#define IGB_EIMS_NOMSIX_TCPTIM    (1 << 30)
#define IGB_EIMS_NOMSIX_OTHER     (1 << 31)

/* Extended Interrupt Mask for MSI-X Mode */

#define IGB_EIMS_MSIX_0           (1 << 0)
#define IGB_EIMS_MSIX_1           (1 << 1)
#define IGB_EIMS_MSIX_2           (1 << 2)
#define IGB_EIMS_MSIX_3           (1 << 3)
#define IGB_EIMS_MSIX_4           (1 << 4)

/* Interrupt Vector Allocation Registers */

#define IGB_IVAR0_RXQ0_SHIFT      (0)        /* Bits 0-4: MSI-X vector assigned to RxQ0 */
#define IGB_IVAR0_RXQ0_VAL        (1 << 7)   /* Bit 7: Valid bit for RxQ0 */
#define IGB_IVAR0_TXQ0_SHIFT      (8)        /* Bits 8-12: MSI-X vector assigned to TxQ0 */
#define IGB_IVAR0_TXQ0_VAL        (1 << 7)   /* Bit 7: Valid bit for TxQ0 */

/* Interrupt Vector Allocation Registers - Misc */

#define IGB_IVARMSC_TCPTIM        (0)        /* Bits 0-5: MSI-X vectorassigned to TCP timer interrupt */
                                             /* Bits 5-6: Reserved */
#define IGB_IVARMSC_TCPTIM_VAL    (1 << 7)   /* Bit 7: Enable bit for TCP timer interrupt */
#define IGB_IVARMSC_OTHER_SHIFT   (8)        /* Bits 8-12: MSI-X vector assigned to Other Cause */
                                             /* Bits 13-14: Reserved */
#define IGB_IVARMSC_OTHER_VAL     (1 << 15)  /* Bit 15: Enable bit for Other Cause */
                                             /* Bits 20-30: Reserved */

/* General Purpose Interrupt Enable */

#define IGB_GPIE_NSICR            (1 << 0)   /* Bit 0: Non Selective Interrupt Clear on Read */
                                             /* Bits 1-3: Reserved */
#define IGB_GPIE_MSIX             (1 << 4)   /* Bit 4: MSI-X mode */
                                             /* Bits 5-6: Reserved */
#define IGB_GPIE_LLINTERVAL_SHIFT (7)        /* Bits 7-11: Low Latency Credits Increment Rate */
                                             /* Bits 12-29: Reserved */
#define IGB_GPIE_EIAME            (1 << 30)  /* Bit 30: Extended Interrupt Auto Mask Enable */
#define IGB_GPIE_PBASUPPORT       (1 << 31)  /* Bit 31: PBA Support */

/* Transmit Descriptor Command Field */

#define IGB_TDESC_CMD_EOP         (1 << 0)   /* Bit 0: End Of Packet */
#define IGB_TDESC_CMD_IFCS        (1 << 1)   /* Bit 1: Insert FCS */
#define IGB_TDESC_CMD_IC          (1 << 2)   /* Bit 2: Insert Checksum */
#define IGB_TDESC_CMD_RS          (1 << 3)   /* Bit 3: Report Status */
#define IGB_TDESC_CMD_DEXT        (1 << 5)   /* Bit 5: Extension (0b for legacy mode) */
#define IGB_TDESC_CMD_VLE         (1 << 6)   /* Bit 6: VLAN Packet Enable */

/* Transmit Descriptor Status Field */

#define IGB_TDESC_STATUS_DD       (1 << 0)   /* Bit 0: Descriptor Done */

/* Transmit Descriptor Special Field */

#define IGB_TDESC_SPEC_VLAN_SHIFT (0)        /* Bits 0-11: VLAN Identifier */
#define IGB_TDESC_SPEC_CFI        (1 << 12)  /* Bit 12: Canonical Form Indicator */
#define IGB_TDESC_SPEC_PRI_SHIFT  (13)       /* Bits 13-15: User Priority */

/* Receive Descriptor Status Field */

#define IGB_RDESC_STATUS_DD       (1 << 0)   /* Bit 0: Descriptor Done */
#define IGB_RDESC_STATUS_EOP      (1 << 1)   /* Bit 1: End of Packet */
#define IGB_RDESC_STATUS_VP       (1 << 3)   /* Bit 3: Packet is 802.1Q (matched VET) */
#define IGB_RDESC_STATUS_UDPCS    (1 << 4)   /* Bit 4: UDP checksum or IP payload checksum calculated on packet */
#define IGB_RDESC_STATUS_L4CS     (1 << 5)   /* Bit 5: L4 (UDP or TCP) checksum calculated on packet */
#define IGB_RDESC_STATUS_IPCS     (1 << 6)   /* Bit 6: IP Checksum Calculated on Packet */
#define IGB_RDESC_STATUS_PIF      (1 << 7)   /* Bit 7: Passed in-exact filter */

/* Receive Descriptor Errors Field */

#define IGB_RDESC_ERRORS_L4E      (1 << 5)   /* Bit 5: TCP/UDP Checksum Error */
#define IGB_RDESC_ERRORS_IPE      (1 << 6)   /* Bit 6: IP Checksum Error */
#define IGB_RDESC_ERRORS_RXE      (1 << 7)   /* Bit 7: RX Data Error */

/* Receive Descriptor Special Field */

#define IGB_RDESC_SPEC_VLAN_SHIFT (0)        /* Bits 0-11: VLAN Identifier */
#define IGB_RDESC_SPEC_CFI        (1 << 12)  /* Bit 12: Canonical Form Indicator */
#define IGB_RDESC_SPEC_PRI_SHIFT  (13)       /* Bits 13-15: User Priority */

/*****************************************************************************
 * Public Types
 *****************************************************************************/

/* Legacy TX descriptor */

begin_packed_struct struct igb_tx_leg_s
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

begin_packed_struct struct igb_rx_leg_s
{
  uint64_t addr;
  uint16_t len;
  uint16_t checksum;
  uint8_t  status;
  uint8_t  errors;
  uint16_t special;
} end_packed_struct;

#endif  /* __DRIVERS_NET_IGB!_H */
