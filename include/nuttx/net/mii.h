/****************************************************************************
 * include/nuttx/net/mii.h
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

#ifndef __INCLUDE_NUTTX_NET_MII_H
#define __INCLUDE_NUTTX_NET_MII_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MII register offsets *****************************************************/

/* Common MII management registers. The IEEE 802.3 standard specifies a
 * register set for controlling and gathering status from the PHY layer. The
 * registers are collectively known as the MII Management registers and are
 * detailed in Section 22.2.4 of the IEEE 802.3 specification.
 */

#define MII_MCR                      0x00      /* MII management control */
#define MII_MSR                      0x01      /* MII management status */
#define MII_PHYID1                   0x02      /* PHY ID 1 */
#define MII_PHYID2                   0x03      /* PHY ID 2 */
#define MII_ADVERTISE                0x04      /* Auto-negotiation advertisement */
#define MII_LPA                      0x05      /* Auto-negotiation link partner base page ability */
#define MII_EXPANSION                0x06      /* Auto-negotiation expansion */
#define MII_NEXTPAGE                 0x07      /* Auto-negotiation next page */
#define MII_LPANEXTPAGE              0x08      /* Auto-negotiation link partner received next page */
#define MII_MSCONTROL                0x09      /* Master/slave control register */
#define MII_MSSTATUS                 0x0a      /* Master/slave status register */
#define MII_PSECONTROL               0x0b      /* PSE control register */
#define MII_PSESTATUS                0x0c      /* PSE status register */
#define MII_MMDCONTROL               0x0d      /* MMD access control register */
#define MII_ESTATUS                  0x0f      /* Extended status register */

/* Extended Registers: Registers 16-31 may be used for vendor specific
 * abilities
 */

/* National Semiconductor DP83840: 0x07-0x11, 0x14, 0x1a, 0x1d-0x1f
 * reserved
 */

#define MII_DP83840_COUNTER          0x12      /* Disconnect counter */
#define MII_DP83840_FCSCOUNTER       0x13      /* False carrier sense counter */
#define MII_DP83840_NWAYTEST         0x14      /* N-way auto-neg test reg */
#define MII_DP83840_RERRCOUNTER      0x15      /* Receive error counter */
#define MII_DP83840_SREVISION        0x16      /* Silicon revision */
#define MII_DP83840_LBRERROR         0x18      /* Loopback, bypass and receiver error */
#define MII_DP83840_PHYADDR          0x19      /* PHY address */
#define MII_DP83840_10BTSR           0x1b      /* 10BASE-T status register */
#define MII_DP83840_10BTCR           0x1c      /* 10BASE-T configuration register */

/* Am79c874: 0x08-0x0f, 0x14, 0x16, 0x19-0x1f reserved */

#define MII_AM79C874_NPADVERTISE     0x07      /* Auto-negotiation next page advertisement */
#define MII_AM79C874_MISCFEATURES    0x10      /* Miscellaneous features reg */
#define MII_AM79C874_INTCS           0x11      /* Interrupt control/status */
#define MII_AM79C874_DIAGNOSTIC      0x12      /* Diagnostic */
#define MII_AM79C874_LOOPBACK        0x13      /* Power management/loopback */
#define MII_AM79C874_MODEC           0x15      /* Mode control register */
#define MII_AM79C874_DISCONNECT      0x17      /* Disconnect counter */
#define MII_AM79C874_RCVERROR        0x18      /* Receive error counter */

/* Luminary LM3S6918 built-in PHY: 0x07-0x0f, 0x14-0x16, 0x19-0x1f reserved */

#define MII_LM_VSPECIFIC             0x10      /* Vendor-Specific */
#define MII_LM_INTCS                 0x11      /* Interrupt control/status */
#define MII_LM_DIAGNOSTIC            0x12      /* Diagnostic */
#define MII_LM_XCVRCONTROL           0x13      /* Transceiver Control */
#define MII_LM_LEDCONFIG             0x17      /* LED Configuration */
#define MII_LM_MDICONTROL            0x18      /* Ethernet PHY Management MDI/MDIX Control */

/* Micrel KS8721: 0x15, 0x1b, and 0x1f */

#define MII_KS8721_RXERCOUNTER       0x15      /* RXER counter */
#define MII_KS8721_INTCS             0x1b      /* Interrupt control/status register */
#define MII_KS8721_10BTCR            0x1f      /* 10BASE-TX PHY control register */

/* Micrel KSZ8041:  0x15, 0x1b, 0x1e-0x1f */

#define MII_KSZ8041_RXERR            0x15      /* RXERR Counter */
#define MII_KSZ8041_INT              0x1b      /* Interrupt Control/Status */
#define MII_KSZ8041_PHYCTRL1         0x1e      /* PHY Control 1 */
#define MII_KSZ8041_PHYCTRL2         0x1f      /* PHY Control 2 */

/* Micrel KSZ8051:  0x11, 0x15-0x18, 0x1b, 0x1d-0x1f */

#define MII_KSZ8051_AFEC1            0x11      /* AFE Control 1 */
#define MII_KSZ8051_RXERR            0x15      /* RXERR Counter */
#define MII_KSZ8051_OMSO             0x16      /* Operation Mode Strap Override */
#define MII_KSZ8051_OMSS             0x17      /* Operation Mode Strap Status */
#define MII_KSZ8051_XCTRL            0x18      /* Expanded Control */
#define MII_KSZ8051_INT              0x1b      /* Interrupt Control/Status */
#define MII_KSZ8051_LINKMD           0x1d      /* LinkMD(c) Control/Status */
#define MII_KSZ8051_PHYCTRL1         0x1e      /* PHY Control 1 */
#define MII_KSZ8051_PHYCTRL2         0x1f      /* PHY Control 2 */

/* Micrel KSZ8061:  0x10-0x18, 0x1b, 0x1c-0x1f */
#define MII_KSZ8061_DIG_CTRL           0x10   /* Digital Control */
#define MII_KSZ8061_AFE_CTRL_0         0x11   /* AFE Control 0 */
#define MII_KSZ8061_AFE_CTRL_1         0x12   /* AFE Control 1 */
#define MII_KSZ8061_AFE_CTRL_2         0x13   /* AFE Control 2 */
#define MII_KSZ8061_AFE_CTRL_3         0x14   /* AFE Control 3 */
#define MII_KSZ8061_RXER_CNTR          0x15   /* RXER Counter */
#define MII_KSZ8061_OP_MODE            0x16   /* Operation Mode */
#define MII_KSZ8061_OP_MODE_STRAP_STAT 0x17   /* Operation Mode Strap Status */
#define MII_KSZ8061_EXP_CTRL           0x18   /* Expanded Control */
#define MII_KSZ8061_INTR_CTRL_STAT     0x1B   /* Interrupt Control/Status */
#define MII_KSZ8061_FUNC_CTRL          0x1C   /* Function Control */
#define MII_KSZ8061_LINKMD_CTRL_STAT   0x1D   /* LinkMD® Control/Status */
#define MII_KSZ8061_PHY_CTRL_1         0x1E   /* PHY Control 1 */
#define MII_KSZ8061_PHY_CTRL_2         0x1F   /* PHY Control 2 */

/* Micrel KSZ8081:  0x10-0x11, 0x15-0x18, 0x1b, 0x1d-0x1f */

#define MII_KSZ8081_DRCTRL           0x10      /* Digital Reserve Control */
#define MII_KSZ8081_AFEC1            0x11      /* AFE Control 1 */
#define MII_KSZ8081_RXERR            0x15      /* RXERR Counter */
#define MII_KSZ8081_OMSO             0x16      /* Operation Mode Strap Override */
#define MII_KSZ8081_OMSS             0x17      /* Operation Mode Strap Status */
#define MII_KSZ8081_XCTRL            0x18      /* Expanded Control */
#define MII_KSZ8081_INT              0x1b      /* Interrupt Control/Status */
#define MII_KSZ8081_LINKMD           0x1d      /* LinkMD(c) Control/Status */
#define MII_KSZ8081_PHYCTRL1         0x1e      /* PHY Control 1 */
#define MII_KSZ8081_PHYCTRL2         0x1f      /* PHY Control 2 */

/* National Semiconductor DP83848C PHY Extended Registers.
 * 0x8-0x15, 0x13, 0x1c reserved
 */

#define MII_DP83848C_STS             0x10      /* RO PHY Status Register */
#define MII_DP83848C_MICR            0x11      /* RW MII Interrupt Control Register */
#define MII_DP83848C_MISR            0x12      /* RO MII Interrupt Status Register */
#define MII_DP83848C_FCSCR           0x14      /* RO False Carrier Sense Counter Register */
#define MII_DP83848C_RECR            0x15      /* RO Receive Error Counter Register */
#define MII_DP83848C_PCSR            0x16      /* RW PCS Sub-Layer Configuration and Status Register */
#define MII_DP83848C_RBR             0x17      /* RW RMII and Bypass Register */
#define MII_DP83848C_LEDCR           0x18      /* RW LED Direct Control Register */
#define MII_DP83848C_PHYCR           0x19      /* RW PHY Control Register */
#define MII_DP83848C_10BTSCR         0x1a      /* RW 10Base-T Status/Control Register */
#define MII_DP83848C_CDCTRL1         0x1b      /* RW CD Test Control Register and BIST Extensions Register */
#define MII_DP83848C_EDCR            0x1d      /* RW Energy Detect Control Register */

/* Texas Instruments DP83825I PHY Extended Registers. */

#define MII_DP83825I_PHYSTS          0x10      /* RO PHY Status Register */
#define MII_DP83825I_PHYSCR          0x11      /* RW PHY Specific Control Register */
#define MII_DP83825I_MISR1           0x12      /* RO MII Interrupt Status Register 1 */
#define MII_DP83825I_MISR2           0x13      /* RO MII Interrupt Status Register 2 */
#define MII_DP83825I_FCSCR           0x14      /* RO False Carrier Sense Counter Register */
#define MII_DP83825I_RECR            0x15      /* RO Receive Error Counter Register */
#define MII_DP83825I_BISCR           0x16      /* RW BIST Control Register */
#define MII_DP83825I_RCSR            0x17      /* RW RMII and Control and Status Register */
#define MII_DP83825I_LEDCR           0x18      /* RW LED Direct Control Register */
#define MII_DP83825I_PHYCR           0x19      /* RW PHY Control Register */
#define MII_DP83825I_10BTSCR         0x1a      /* RW 10Base-T Status/Control Register */
#define MII_DP83825I_BICSR1          0x1b      /* RW BIST Control Register 1 */
#define MII_DP83825I_BICSR2          0x1c      /* RW BIST Control Register 2 */
#define MII_DP83825I_CDCR            0x1e      /* RW Cable Diagnostic Control Register */
#define MII_DP83825I_PHYRCR          0x1f      /* RW PHY Reset Control Register */

/* SMSC LAN8720 PHY Extended Registers */

#define MII_LAN8720_REV              0x10      /* Silicon Revision Register */
#define MII_LAN8720_MCSR             0x11      /* Mode Control/Status Register */
#define MII_LAN8720_MODES            0x12      /* Special modes */
#define MII_LAN8720_SECR             0x1a      /* Symbol Error Counter Register */
#define MII_LAN8720_CSIR             0x1b      /* Control / Status Indicator Register */
#define MII_LAN8720_SITC             0x1c      /* Special Internal Testability Controls */
#define MII_LAN8720_ISR              0x1d      /* Interrupt Source Register */
#define MII_LAN8720_IMR              0x1e      /* Interrupt Mask Register */
#define MII_LAN8720_SCSR             0x1f      /* PHY Special Control/Status Register */

/* SMSC LAN8740/LAN8742A PHY Extended Registers */

#define MII_LAN8740_CONFIG           0x10      /* EDPD NDL/Crossover Timer/EEE Configuration */
#define MII_LAN8740_MCSR             0x11      /* Mode Control/Status Register */
#define MII_LAN8740_MODES            0x12      /* Special modes */
#define MII_LAN8740_TDRPAT           0x18      /* TDR Patterns/Delay Control Register */
#define MII_LAN8740_TDRCTL           0x19      /* TDR Control/Status Register */
#define MII_LAN8740_SECR             0x1a      /* Symbol Error Counter Register */
#define MII_LAN8740_CSIR             0x1b      /* Control/Status Indicator Register */
#define MII_LAN8740_CBLEN            0x1c      /* Cable Length Register */
#define MII_LAN8740_ISR              0x1d      /* Interrupt Source Register */
#define MII_LAN8740_IMR              0x1e      /* Interrupt Mask Register */
#define MII_LAN8740_SCSR             0x1f      /* PHY Special Control/Status Register */

/* MII register bit settings ************************************************/

/* MII Control register bit definitions */

#define MII_MCR_UNIDIR               (1 << 5)  /* Bit 5:  Unidirectional enable */
#define MII_MCR_SPEED1000            (1 << 6)  /* Bit 6:  MSB of Speed (1000 reserved on 10/100) */
#define MII_MCR_CTST                 (1 << 7)  /* Bit 7:  Enable collision test  */
#define MII_MCR_FULLDPLX             (1 << 8)  /* Bit 8:  Full duplex */
#define MII_MCR_ANRESTART            (1 << 9)  /* Bit 9:  Restart auto negotiation */
#define MII_MCR_ISOLATE              (1 << 10) /* Bit 10: Electronically isolate PHY from MII */
#define MII_MCR_PDOWN                (1 << 11) /* Bit 11: Powerdown the PHY */
#define MII_MCR_ANENABLE             (1 << 12) /* Bit 12: Enable auto negotiation */
#define MII_MCR_SPEED100             (1 << 13) /* Bit 13: Select 100Mbps */
#define MII_MCR_LOOPBACK             (1 << 14) /* Bit 14: Enable loopback mode */
#define MII_MCR_RESET                (1 << 15) /* Bit 15: PHY reset */

/* MII Status register bit definitions */

#define MII_MSR_EXTCAP               (1 << 0)  /* Bit 0:  Extended register capability */
#define MII_MSR_JABBERDETECT         (1 << 1)  /* Bit 1:  Jabber detect */
#define MII_MSR_LINKSTATUS           (1 << 2)  /* Bit 2:  Link status */
#define MII_MSR_ANEGABLE             (1 << 3)  /* Bit 3:  Auto-negotiation able */
#define MII_MSR_RFAULT               (1 << 4)  /* Bit 4:  Remote fault */
#define MII_MSR_ANEGCOMPLETE         (1 << 5)  /* Bit 5:  Auto-negotiation complete */
#define MII_MSR_MFRAMESUPPRESS       (1 << 6)  /* Bit 6:  Management frame suppression */
#define MII_MSR_UNIDIR               (1 << 7)  /* Bit 7:  Unidirectional ability */
#define MII_MSR_ESTATEN              (1 << 8)  /* Bit 8:  Extended Status in R15 */
#define MII_MSR_100BASET2FULL        (1 << 9)  /* Bit 9:  100BASE-T2 half duplex able */
#define MII_MSR_100BASET2HALF        (1 << 10) /* Bit 10: 100BASE-T2 full duplex able */
#define MII_MSR_10BASETXHALF         (1 << 11) /* Bit 11: 10BASE-TX half duplex able */
#define MII_MSR_10BASETXFULL         (1 << 12) /* Bit 12: 10BASE-TX full duplex able */
#define MII_MSR_100BASETXHALF        (1 << 13) /* Bit 13: 100BASE-TX half duplex able */
#define MII_MSR_100BASETXFULL        (1 << 14) /* Bit 14: 100BASE-TX full duplex able */
#define MII_MSR_100BASET4            (1 << 15) /* Bit 15: 100BASE-T4 able */

/* MII ID1 register bits: Bits 3-18 of the Organizationally Unique
 * identifier (OUI)
 */

/* MII ID2 register bits */

#define MII_PHYID2_REV_SHIFT         (0)    /* Bits 0-3: Revision number mask */
#define MII_PHYID2_REV_MASK          (15 <<  MII_PHYID2_REV_SHIFT)
#  define MII_PHYID2_REV(n)          ((uint16_t)(n) <<  MII_PHYID2_REV_SHIFT)
#define MII_PHYID2_MODEL_SHIFT       (4)       /* Bits 4-9: Model number mask */
#define MII_PHYID2_MODEL_MASK        (0x3f <<  MII_PHYID2_MODEL_SHIFT)
#  define MII_PHYID2_MODEL(n)        ((uint16_t)(n) <<  MII_PHYID2_MODEL_SHIFT)
#define MII_PHYID2_OUI_SHIFT         (10)      /* Bits 10-15: OUI mask [24:19] */
#define MII_PHYID2_OUI_MASK          (0x3f <<  MII_PHYID2_OUI_SHIFT)
#  define MII_PHYID2_OUI(n)          ((uint16_t)(n) <<  MII_PHYID2_OUI_SHIFT)

/* Advertisement control register bit definitions */

#define MII_ADVERTISE_SELECT         0x001f    /* Bits 0-4: Selector field */
#define MII_ADVERTISE_CSMA           (1 << 0)  /*         CSMA */
#define MII_ADVERTISE_8023           (1 << 0)  /*         IEEE Std 802.3 */
#define MII_ADVERTISE_8029           (2 << 0)  /*         IEEE Std 802.9 ISLAN-16T */
#define MII_ADVERTISE_8025           (3 << 0)  /*         IEEE Std 802.5 */
#define MII_ADVERTISE_1394           (4 << 0)  /*         IEEE Std 1394 */
#define MII_ADVERTISE_10BASETXHALF   (1 << 5)  /* Bit 5:  Try 10BASE-TX half duplex */
#define MII_ADVERTISE_1000XFULL      (1 << 5)  /* Bit 5:  Try 1000BASE-X full duplex */
#define MII_ADVERTISE_10BASETXFULL   (1 << 6)  /* Bit 6:  Try 10BASE-TX full duplex */
#define MII_ADVERTISE_1000XHALF      (1 << 6)  /* Bit 6:  Try 1000BASE-X half duplex */
#define MII_ADVERTISE_100BASETXHALF  (1 << 7)  /* Bit 7:  Try 100BASE-TX half duplex */
#define MII_ADVERTISE_1000XPAUSE     (1 << 7)  /* Bit 7:  Try 1000BASE-X pause */
#define MII_ADVERTISE_100BASETXFULL  (1 << 8)  /* Bit 8:  Try 100BASE-TX full duplex*/
#define MII_ADVERTISE_1000XASYMPAU   (1 << 8)  /* Bit 8:  Try 1000BASE-X asym pause */
#define MII_ADVERTISE_100BASET4      (1 << 9)  /* Bit 9:  Try 100BASE-T4 */
#define MII_ADVERTISE_FDXPAUSE       (1 << 10) /* Bit 10: Try full duplex flow control */
#define MII_ADVERTISE_ASYMPAUSE      (1 << 11) /* Bit 11: Try asymmetric pause */
#define MII_ADVERTISE_RFAULT         (1 << 13) /* Bit 13: Remote fault supported */
#define MII_ADVERTISE_LPACK          (1 << 14) /* Bit 14: Ack link partners response */
#define MII_ADVERTISE_NXTPAGE        (1 << 15) /* Bit 15: Next page enabled */

/* Link partner ability register bit definitions */

#define MII_LPA_SELECT               0x001f    /* Bits 0-4: Link partner selector field */
#define MII_LPA_CSMA                 (1 << 0)  /*         CSMA */
#define MII_LPA_8023                 (1 << 0)  /*         IEEE Std 802.3 */
#define MII_LPA_8029                 (2 << 0)  /*         IEEE Std 802.9 ISLAN-16T */
#define MII_LPA_8025                 (3 << 0)  /*         IEEE Std 802.5 */
#define MII_LPA_1394                 (4 << 0)  /*         IEEE Std 1394 */
#define MII_LPA_10BASETXHALF         (1 << 5)  /* Bit 5:  10BASE-TX half duplex able */
#define MII_LPA_1000XFULL            (1 << 5)  /* Bit 5:  1000BASE-X full-duplex able */
#define MII_LPA_10BASETXFULL         (1 << 6)  /* Bit 6:  10BASE-TX full duplex able */
#define MII_LPA_1000XHALF            (1 << 6)  /* Bit 6:  1000BASE-X half-duplex */
#define MII_LPA_100BASETXHALF        (1 << 7)  /* Bit 7:  100BASE-TX half duplex able */
#define MII_LPA_1000XPAUSE           (1 << 7)  /* Bit 7:  1000BASE-X pause able */
#define MII_LPA_100BASETXFULL        (1 << 8)  /* Bit 8:  100BASE-TX full duplex able */
#define MII_LPA_1000XASYMPAU         (1 << 8)  /* Bit 8:  1000BASE-X asym pause able */
#define MII_LPA_100BASET4            (1 << 9)  /* Bit 9:  100BASE-T4 able */
#define MII_LPA_FDXPAUSE             (1 << 10) /* Bit 10: Full duplex flow control able */
#define MII_LPA_ASYMPAUSE            (1 << 11) /* Bit 11: Asynchronous pause able */
#define MII_LPA_RFAULT               (1 << 13) /* Bit 13: Link partner remote fault request */
#define MII_LPA_LPACK                (1 << 14) /* Bit 14: Link partner acknowledgement */
#define MII_LPA_NXTPAGE              (1 << 15) /* Bit 15: Next page requested */

/* Link partner ability in next page format */

#define MII_LPANP_MESSAGE            0x07ff    /* Bits 0-10: Link partner's message code */
#define MII_LPANP_TOGGLE             (1 << 11) /* Bit 11: Link partner toggle */
#define MII_LPANP_LACK2              (1 << 12) /* Bit 12: Link partner can comply ACK */
#define MII_LPANP_MSGPAGE            (1 << 13) /* Bit 13: Link partner message page request */
#define MII_LPANP_LPACK              (1 << 14) /* Bit 14: Link partner acknowledgement */
#define MII_LPANP_NXTPAGE            (1 << 15) /* Bit 15: Next page requested */

/* MII Auto-negotiation expansion register bit definitions */

#define MII_EXPANSION_ANEGABLE       (1 << 0)  /* Bit 0: Link partner is auto-negotiation able */
#define MII_EXPANSION_PAGERECVD      (1 << 1)  /* Bit 1: New link code word in LPA ability reg */
#define MII_EXPANSION_ENABLENPAGE    (1 << 2)  /* Bit 2: This enables npage words */
#define MII_EXPANSION_NXTPAGEABLE    (1 << 3)  /* Bit 3: Link partner supports next page */
#define MII_EXPANSION_PARFAULTS      (1 << 4)  /* Bit 4: Fault detected by parallel logic */

/* Auto-negotiation next page advertisement */

#define MII_NPADVERTISE_CODE         0x07ff    /* Bits 0-10: message/un-formatted code field */
#define MII_NPADVERTISE_TOGGLE       (1 << 11) /* Bit 11: Toggle */
#define MII_NPADVERTISE_ACK2         (1 << 12) /* Bit 12: Acknowledgement 2 */
#define MII_NPADVERTISE_MSGPAGE      (1 << 13) /* Bit 13: Message page */
#define MII_NPADVERTISE_NXTPAGE      (1 << 15) /* Bit 15: Next page indication */

/* MMD access control register */

#define MII_MMDCONTROL_DEVAD_SHIFT   (0)      /* Bits 0-4: Device address */
#define MII_MMDCONTROL_DEVAD_MASK    (31 << MII_MMDCONTROL_DEVAD_SHIFT)
#  define MII_MMDCONTROL_DEVAD(n)    ((uint16_t)(n) << MII_MMDCONTROL_DEVAD_SHIFT)
                                               /* Bits 5-13: Reserved */
#define MII_MMDCONTROL_FUNC_SHIFT    (14)      /* Bits 14-15: Function */
#define MII_MMDCONTROL_FUNC_MASK     (3 << MII_MMDCONTROL_FUNC_SHIFT)
#  define MII_MMDCONTROL_FUNC_ADDR   (0 << MII_MMDCONTROL_FUNC_SHIFT) /* Address */
#  define MII_MMDCONTROL_FUNC_NOINCR (1 << MII_MMDCONTROL_FUNC_SHIFT) /* Data, no post increment */
#  define MII_MMDCONTROL_FUNC_RWINCR (2 << MII_MMDCONTROL_FUNC_SHIFT) /* Data, post incr on reads & writes */
#  define MII_MMDCONTROL_FUNC_WINCR  (3 << MII_MMDCONTROL_FUNC_SHIFT) /* Data, post incr on writes */

/* Extended status register */

                                               /* Bits 0-11: Reserved */
#define MII_ESTATUS_1000BASETHALF    (1 << 12) /* Bit 12: 1000BASE-T Half Duplex able */
#define MII_ESTATUS_1000BASETFULL    (1 << 13) /* Bit 13: 1000BASE-T Full Duplex able */
#define MII_ESTATUS_1000BASEXHALF    (1 << 14) /* Bit 14: 1000BASE-X Half Duplex able */
#define MII_ESTATUS_1000BASEXFULL    (1 << 15) /* Bit 15: 1000BASE-X Full Duplex able */

/* MII PHYADDR register bit definitions */

#define DP83840_PHYADDR_DUPLEX       (1 << 7)
#define DP83840_PHYADDR_SPEED        (1 << 6)

/* National Semiconductor DP83848C ******************************************/

/* DP83848C MII ID1/2 register bits */

#define MII_PHYID1_DP83848C          0x2000    /* ID1 value for DP83848C */
#define MII_PHYID2_DP83848C          0x5c90    /* ID2 value for DP83848C */

/* RMII and Bypass Register (0x17) */

#define MII_RBR_ELAST_MASK           0x0003    /* Bits 0-1: Receive elasticity buffer */
#  define MII_RBR_ELAST_14           0x0000    /*   14 bit tolerance */
#  define MII_RBR_ELAST_2            0x0001    /*   2 bit tolerance */
#  define MII_RBR_ELAST_6            0x0002    /*   6 bit tolerance */
#  define MII_RBR_ELAST_10           0x0003    /*   10 bit tolerance */
#define MII_RBR_RXUNFSTS             (1 << 2)  /* Bit 2: RX FIFO underflow */
#define MII_RBR_RXOVFSTS             (1 << 3)  /* Bit 3: RX FIFO overflow */
#define MII_RBR_RMIIREV10            (1 << 4)  /* Bit 4: 0=RMIIv1.2 1-RMIIv1.0 */
#define MII_RBR_RMIIMODE             (1 << 5)  /* Bit 5: 0=MII mode 1=RMII mode */

/* Texas Instruments DP83825I ***********************************************/

/* DP838825I MII ID1/2 register bits */

#define MII_PHYID1_DP83825I          0x2000    /* ID1 value for DP838825 */
#define MII_PHYID2_DP83825I          0xa140    /* ID2 value for DP838825 */

/* PHYSTS Register (0x10) */

#define MII_DP83825I_PHYSTS_SPEED   (1 << 1)   /* Bit 1: Speed Status Register */
#define MII_DP83825I_PHYSTS_DUPLEX  (1 << 2)   /* Bit 2: Duplex Status Register */

/* RCSC Register (0x17) */

#define MII_DP83825I_RCSC_ELAST_MASK 0x0003    /* Bits 0-1: Receive elasticity buffer */
#  define MII_DP83825I_RCSC_ELAST_14 0x0000    /*   14 bit tolerance */
#  define MII_DP83825I_RCSC_ELAST_2  0x0001    /*   2 bit tolerance */
#  define MII_DP83825I_RCSC_ELAST_6  0x0002    /*   6 bit tolerance */
#  define MII_DP83825I_RCSC_ELAST_10 0x0003    /*   10 bit tolerance */
#define MII_DP83825I_RCSC_RXUNFSTS   (1 << 2)  /* Bit 2: RX FIFO underflow */
#define MII_DP83825I_RCSC_RXOVFSTS   (1 << 3)  /* Bit 3: RX FIFO overflow */
#define MII_DP83825I_RCSC_RMIIREV10  (1 << 4)  /* Bit 4: 0=RMIIv1.2 1-RMIIv1.0 */
#define MII_DP83825I_RCSC_RMIICS     (1 << 7)  /* Bit 7: 0=25MHz 1=50MHz */

/* SMSC LAN8720 *************************************************************/

/* SMSC LAN8720 MII ID1/2 register bits */

#define MII_PHYID1_LAN8720           0x0007    /* ID1 value for LAN8720 */
#define MII_PHYID2_LAN8720           0xc0f1    /* ID2 value for LAN8720 */

/* SMSC LAN8720 SPSCR register bits */

#define MII_LAN8720_SPSCR_SCRMDIS    (1 << 0)  /* Bit 0:  Scramble disable */
                                               /* Bit 1:  Reserved */
#define MII_LAN8720_SPSCR_MODE_SHIFT (2)       /* Bits 2-4: Speed/duplex mode */
#define MII_LAN8720_SPSCR_MODE_MASK  (7 << MII_LAN8720_SPSCR_MODE_SHIFT)
#  define MII_LAN8720_SPSCR_10MBPS   (1 << 2)  /* Bit 2:  10MBPS speed */
#  define MII_LAN8720_SPSCR_100MBPS  (1 << 3)  /* Bit 3:  100MBPS speed */
#  define MII_LAN8720_SPSCR_DUPLEX   (1 << 4)  /* Bit 4:  Full duplex mode */
                                               /* Bit 5:  Reserved */
#define MII_LAN8720_SPSCR_ENAB4B5B   (1 << 6)  /* Bit 6:  Enable 4B5B */
#define MII_LAN8720_SPSCR_GPIO0      (1 << 7)  /* Bit 7:  GPIO0 */
#define MII_LAN8720_SPSCR_GPIO1      (1 << 8)  /* Bit 8:  GPIO1 */
#define MII_LAN8720_SPSCR_GPIO2      (1 << 9)  /* Bit 9:  GPIO2 */
                                               /* Bit 10-11: Reserved */
#define MII_LAN8720_SPSCR_ANEGDONE   (1 << 12) /* Bit 12: Autonegotiation complete */
                                               /* Bits 13-15: Reserved */

/* SMSC LAN8720 MODES register bits */

#define MII_LAN8720_MODES_PHYAD_SHIFT (0)      /* Bits 0-4: Phy Address */
#define MII_LAN8720_MODES_PHYAD_MASK  (0x1f << MII_LAN8720_MODES_PHYAD_SHIFT)
#define MII_LAN8720_MODES_PHYAD(n)    ((n<<MII_LAN8720_MODES_PHYAD_SHIFT) & MII_LAN8720_MODES_PHYAD_MASK)
#define MII_LAN8720_MODES_MODE_SHIFT  (5)      /* Bits 5-7: Mode */
#define MII_LAN8720_MODES_MODE_MASK   (7 << MII_LAN8720_MODES_MODE_SHIFT)
#define MII_LAN8720_MODES_MODE(n)     (( n << MII_LAN8720_MODES_MODE_SHIFT) & MII_LAN8720_MODES_MODE_MASK)
#define MII_LAN8720_MODES_10BTHD       MII_LAN8720_MODES_MODE(0)
#define MII_LAN8720_MODES_10BTFD       MII_LAN8720_MODES_MODE(1)
#define MII_LAN8720_MODES_100BTHD_TRC  MII_LAN8720_MODES_MODE(2)
#define MII_LAN8720_MODES_100BTHD_TC   MII_LAN8720_MODES_MODE(3)
#define MII_LAN8720_MODES_100BTHDA_TRC MII_LAN8720_MODES_MODE(4)
#define MII_LAN8720_MODES_RPT          MII_LAN8720_MODES_MODE(5)
#define MII_LAN8720_MODES_PDN          MII_LAN8720_MODES_MODE(6)
#define MII_LAN8720_MODES_ALL          MII_LAN8720_MODES_MODE(7)
#define MII_LAN8720_MODES_RESV        (1 << 14) /* Bit 14: Set to 1 */

/* SMSC LAN8740 MII ID1/2 register bits */

#define MII_PHYID1_LAN8740           0x0007    /* ID1 value for LAN8740 */
#define MII_PHYID2_LAN8740           0xc110    /* ID2 value for LAN8740 */

/* SMSC LAN8740A MII ID1/2 register bits */

#define MII_PHYID1_LAN8740A          0x0007    /* ID1 value for LAN8740A */
#define MII_PHYID2_LAN8740A          0xc111    /* ID2 value for LAN8740A */

/* SMSC LAN8742A MII ID1/2 register bits */

#define MII_PHYID1_LAN8742A          0x0007    /* ID1 value for LAN8742A */
#define MII_PHYID2_LAN8742A          0xc130    /* ID2 value for LAN8742A */

/* Am79c874-specific register bit settings **********************************/

/* Am79c874 MII ID1/2 register bits */

#define MII_PHYID1_AM79C874          0x0022    /* ID1 value for Am79c874 */
#define MII_PHYID2_AM79C874          0x561b    /* ID2 value for Am79c874 Rev B */

/* Am79c874 diagnostics register */

#define AM79C874_DIAG_RXLOCK         (1 << 8)  /* Bit 8:  1=Rcv PLL locked on */
#define AM79C874_DIAG_RXPASS         (1 << 9)  /* Bit 9:  1=Operating in 100Base-X mode */
#define AM79C874_DIAG_100MBPS        (1 << 10) /* Bit 10: 1=ANEG result is 100Mbps */
#define AM79C874_DIAG_FULLDPLX       (1 << 11) /* Bit 11: 1=ANEG result is full duplex */

/* LM3S6918-specific register bit settings **********************************/

/* LM3S6918 Vendor-Specific, address 0x10 */

#define LM_VSPECIFIC_RXCC            (1 << 0)  /* Bit 0:  Receive Clock Control*/
#define LM_VSPECIFIC_PCSBP           (1 << 1)  /* Bit 1:  PCS Bypass */
#define LM_VSPECIFIC_RVSPOL          (1 << 4)  /* Bit 4:  Receive Data Polarity */
#define LM_VSPECIFIC_APOL            (1 << 5)  /* Bit 5:  Auto-Polarity Disable */
#define LM_VSPECIFIC_NL10            (1 << 10) /* Bit 10: Natural Loopback Mode */
#define LM_VSPECIFIC_SQEI            (1 << 11) /* Bit 11: SQE Inhibit Testing */
#define LM_VSPECIFIC_TXHIM           (1 << 12) /* Bit 12: Transmit High Impedance Mode */
#define LM_VSPECIFIC_INPOL           (1 << 14) /* Bit 14: Interrupt Polarity Value*/
#define LM_VSPECIFIC_RPTR            (1 << 15) /* Bit 15: Repeater mode*/

/* LM3S6918 Interrupt Control/Status, address 0x11 */

#define LM_INTCS_ANEGCOMPINT         (1 << 0)  /* Bit 0:  Auto-Negotiation Complete Interrupt */
#define LM_INTCS_RFAULTINT           (1 << 1)  /* Bit 1:  Remote Fault Interrupt */
#define LM_INTCS_LSCHGINT            (1 << 2)  /* Bit 2:  Link Status Change Interrupt */
#define LM_INTCS_LPACKINT            (1 << 3)  /* Bit 3:  LP Acknowledge Interrupt */
#define LM_INTCS_PDFINT              (1 << 4)  /* Bit 4:  Parallel Detection Fault Interrupt */
#define LM_INTCS_PRXINT              (1 << 5)  /* Bit 5:  Page Receive Interrupt */
#define LM_INTCS_RXERINT             (1 << 6)  /* Bit 6:  Receive Error Interrupt */
#define LM_INTCS_JABBERINT           (1 << 7)  /* Bit 7:  Jabber Event Interrupt */
#define LM_INTCS_ANEGCOMPIE          (1 << 8)  /* Bit 8:  Auto-Negotiation Complete Interrupt Enable */
#define LM_INTCS_RFAULTIE            (1 << 9)  /* Bit 9:  Remote Fault Interrupt Enable */
#define LM_INTCS_LSCHGIE             (1 << 10) /* Bit 10: Link Status Change Interrupt Enable */
#define LM_INTCS_LPACKIE             (1 << 11) /* Bit 11: LP Acknowledge Interrupt Enable */
#define LM_INTCS_PDFIE               (1 << 12) /* Bit 12: Parallel Detection Fault Interrupt Enable */
#define LM_INTCS_PRXIE               (1 << 13) /* Bit 13: Page Received Interrupt Enable */
#define LM_INTCS_RXERIE              (1 << 14) /* Bit 14: Receive Error Interrupt Enable */
#define LM_INTCS_JABBERIE            (1 << 15) /* Bit 15: Jabber Interrupt Enable */

/* LM3S6918 Diagnostic, address 0x12 */

#define LM_DIAGNOSTIC_RX_LOCK        (1 << 8)  /* Bit 8:  Receive PLL Lock */
#define LM_DIAGNOSTIC_RXSD           (1 << 9)  /* Bit 9:  Receive Detection */
#define LM_DIAGNOSTIC_RATE           (1 << 10) /* Bit 10: Rate */
#define LM_DIAGNOSTIC_DPLX           (1 << 11) /* Bit 11: Duplex Mode */
#define LM_DIAGNOSTIC_ANEGF          (1 << 12) /* Bit 12: Auto-Negotiation Failure */

/* LM3S6918 Transceiver Control, address 0x13 */

#define LM_XCVRCONTROL_TXO_SHIFT     14        /* Bits 15-14: Transmit Amplitude Selection */
#define LM_XCVRCONTROL_TXO_MASK      (3 << LM_XCVRCONTROL_TXO_SHIFT)
#define LM_XCVRCONTROL_TXO_00DB      (0 << LM_XCVRCONTROL_TXO_SHIFT) /* Gain 0.0dB of insertion loss */
#define LM_XCVRCONTROL_TXO_04DB      (1 << LM_XCVRCONTROL_TXO_SHIFT) /* Gain 0.4dB of insertion loss */
#define LM_XCVRCONTROL_TXO_08DB      (2 << LM_XCVRCONTROL_TXO_SHIFT) /* Gain 0.8dB of insertion loss */
#define LM_XCVRCONTROL_TXO_12DB      (3 << LM_XCVRCONTROL_TXO_SHIFT) /* Gain 1.2dB of insertion loss */

/* LM3S6918 LED Configuration, address 0x17 */

#define LM_LEDCONFIG_LED0_SHIFT      (0)       /* Bits 3-0: LED0 Source */
#define LM_LEDCONFIG_LED0_MASK       (0x0f << LM_LEDCONFIG_LED0_SHIFT)
#define LM_LEDCONFIG_LED0_LINKOK     (0 << LM_LEDCONFIG_LED0_SHIFT)  /* Link OK */
#define LM_LEDCONFIG_LED0_RXTX       (1 << LM_LEDCONFIG_LED0_SHIFT)  /* RX or TX activity */
#define LM_LEDCONFIG_LED0_100BASET   (5 << LM_LEDCONFIG_LED0_SHIFT)  /* 100BASE-TX mode */
#define LM_LEDCONFIG_LED0_10BASET    (6 << LM_LEDCONFIG_LED0_SHIFT)  /* 10BASE-T mode */
#define LM_LEDCONFIG_LED0_FDUPLEX    (7 << LM_LEDCONFIG_LED0_SHIFT)  /* Full duplex */
#define LM_LEDCONFIG_LED0_OKRXTX     (8 << LM_LEDCONFIG_LED0_SHIFT)  /* Full duplex */
#define LM_LEDCONFIG_LED1_SHIFT      (4)                             /* Bits 7-4: LED1 Source */
#define LM_LEDCONFIG_LED1_MASK       (0x0f << LM_LEDCONFIG_LED1_SHIFT)
#define LM_LEDCONFIG_LED1_LINKOK     (0 << LM_LEDCONFIG_LED1_SHIFT)  /* Link OK */
#define LM_LEDCONFIG_LED1_RXTX       (1 << LM_LEDCONFIG_LED1_SHIFT)  /* RX or TX activity */
#define LM_LEDCONFIG_LED1_100BASET   (5 << LM_LEDCONFIG_LED1_SHIFT)  /* 100BASE-TX mode */
#define LM_LEDCONFIG_LED1_10BASET    (6 << LM_LEDCONFIG_LED1_SHIFT)  /* 10BASE-T mode */
#define LM_LEDCONFIG_LED1_FDUPLEX    (7 << LM_LEDCONFIG_LED1_SHIFT)  /* Full duplex */
#define LM_LEDCONFIG_LED1_OKRXTX     (8 << LM_LEDCONFIG_LED1_SHIFT)  /* Full duplex */

/* LM3S6918 MDI/MDIX Control, address 0x18 */

#define LM_MDICONTROL_MDIXSD_SHIFT   (0)        /* Bits 3-0: Auto-Switching Seed */
#define LM_MDICONTROL_MDIXSD_MASK    (0x0f << LM_MDICONTROL_MDIXSD_SHIFT)
#define LM_MDICONTROL_MDIXCM         (1 << 4)  /* Bit 4:  Auto-Switching Complete */
#define LM_MDICONTROL_MDIX           (1 << 5)  /* Bit 5:  Auto-Switching Configuration */
#define LM_MDICONTROL_AUTOSW         (1 << 6)  /* Bit 6:  Auto-Switching Enable */
#define LM_MDICONTROL_PDMODE         (1 << 7)  /* Bit 7:  Parallel Detection Mode */

/* KS8921-specific register bit settings ************************************/

/* KS8921 MII Control register bit definitions (not in 802.3) */

#define KS8721_MCR_DISABXMT          (1 << 0)  /* Bit 0:  Disable Transmitter */

/* KS8921 MII ID1/2 register bits */

#define MII_PHYID1_KS8721            0x0022    /* ID1 value for Micrel KS8721 */
#define MII_PHYID2_KS8721            0x1619    /* ID2 value for Micrel KS8721 */

/* KS8921 RXER Counter -- 16-bit counter */

/* KS8921 Interrupt Control/Status Register */

#define KS8721_INTCS_LINKUP          (1 << 0)  /* Bit 0:  Link up occurred */
#define KS8721_INTCS_REMFAULT        (1 << 1)  /* Bit 1:  Remote fault occurred */
#define KS8721_INTCS_LINKDOWN        (1 << 2)  /* Bit 2:  Link down occurred */
#define KS8721_INTCS_LPACK           (1 << 3)  /* Bit 3:  Link partner acknowledge occurred */
#define KS8721_INTCS_PDFAULT         (1 << 4)  /* Bit 4:  Parallel detect fault occurred */
#define KS8721_INTCS_PGRCVD          (1 << 5)  /* Bit 5:  Page received occurred */
#define KS8721_INTCS_RXERR           (1 << 6)  /* Bit 6:  Receive error occurred */
#define KS8721_INTCS_JABBER          (1 << 7)  /* Bit 7:  Jabber interrupt occurred */
#define KS8721_INTCS_LINKUPE         (1 << 8)  /* Bit 8:  Enable link up interrupt */
#define KS8721_INTCS_REMFAULTE       (1 << 9)  /* Bit 9:  Enable remote fault interrupt */
#define KS8721_INTCS_LINKDOWNE       (1 << 10) /* Bit 10: Enable link down interrupt */
#define KS8721_INTCS_LPACKE          (1 << 11) /* Bit 11: Enable link partner acknowldgement interrupt */
#define KS8721_INTCS_PDFAULTE        (1 << 12) /* Bit 12: Enable parallel detect fault interrupt */
#define KS8721_INTCS_PGRCVDE         (1 << 13) /* Bit 13: Enable page received interrupt */
#define KS8721_INTCS_RXERRE          (1 << 14) /* Bit 14: Enable receive error interrupt */
#define KS8721_INTCS_JABBERE         (1 << 15) /* Bit 15: Enable Jabber Interrupt */

/* KS8921 10BASE-TX PHY control register */

#define KS8721_10BTCR_BIT0           (1 << 0)  /* Bit 0:  xxx */
#define KS8721_10BTCR_BIT1           (1 << 1)  /* Bit 1:  xxx */
#define KS8721_10BTCR_MODE_SHIFT     (2)       /* Bits 2-4:  Operation Mode Indication */
#define KS8721_10BTCR_MODE_MASK      (7 << KS8721_10BTCR_MODE_SHIFT)
#  define KS8721_10BTCR_MODE_ANEG    (0 << KS8721_10BTCR_MODE_SHIFT) /* Still in auto-negotiation */
#  define KS8721_10BTCR_MODE_10BTHD  (1 << KS8721_10BTCR_MODE_SHIFT) /* 10BASE-T half-duplex */
#  define KS8721_10BTCR_MODE_100BTHD (2 << KS8721_10BTCR_MODE_SHIFT) /* 100BASE_t half-duplex */
#  define KS8721_10BTCR_MODE_DEFAULT (3 << KS8721_10BTCR_MODE_SHIFT) /* Default */
#  define KS8721_10BTCR_MODE_10BTFD  (5 << KS8721_10BTCR_MODE_SHIFT) /* 10BASE-T full duplex */
#  define KS8721_10BTCR_MODE_100BTFD (6 << KS8721_10BTCR_MODE_SHIFT) /* 100BASE-T full duplex */
#  define KS8721_10BTCR_MODE_ISOLATE (7 << KS8721_10BTCR_MODE_SHIFT) /* PHY/MII isolate */

#define KS8721_10BTCR_ISOLATE        (1 << 5)  /* Bit 5:  PHY isolate */
#define KS8721_10BTCR_PAUSE          (1 << 6)  /* Bit 6:  Enable pause */
#define KS8721_10BTCR_ANEGCOMP       (1 << 7)  /* Bit 7:  Auto-negotiation complete */
#define KS8721_10BTCR_JABBERE        (1 << 8)  /* Bit 8:  Enable Jabber */
#define KS8721_10BTCR_INTLVL         (1 << 9)  /* Bit 9:  Interrupt level */
#define KS8721_10BTCR_POWER          (1 << 10) /* Bit 10: Power saving */
#define KS8721_10BTCR_FORCE          (1 << 11) /* Bit 11: Force link */
#define KS8721_10BTCR_ENERGY         (1 << 12) /* Bit 12: Energy detect */
#define KS8721_10BTCR_PAIRSWAPD      (1 << 13) /* Bit 13: Pairswap disable */

/* KSZ8051/81-specific register bit settings ********************************/

/* KSZ8041/51/81 MII ID1/2 register bits */

#define MII_PHYID1_KSZ8041           0x0022    /* ID1 value for Micrel KSZ8041 */
#define MII_PHYID2_KSZ8041           0x1510    /* ID2 value for Micrel KSZ8041 */

#define MII_PHYID1_KSZ8051           0x0022    /* ID1 value for Micrel KSZ8051 */
#define MII_PHYID2_KSZ8051           0x1550    /* ID2 value for Micrel KSZ8051 */

#define MII_PHYID1_KSZ8081           0x0022    /* ID1 value for Micrel KSZ8081 */
#define MII_PHYID2_KSZ8081           0x1560    /* ID2 value for Micrel KSZ8081 */

/* KSZ8081 Digital Reserve Control */

                                               /* Bits 5-15: Reserved */
#define KSZ8081_DRCTRL_PLLOFF        (1 << 4)  /* Bit 4: Turn PLL off in EDPD mode */
                                               /* Bits 0-3: Reserved */

/* KSZ8041/51/81 Register 0x1b: Interrupt control/status */

#define MII_KSZ80X1_INT_JEN          (1 << 15) /* Jabber interrupt enable */
#define MII_KSZ80X1_INT_REEN         (1 << 14) /* Receive error interrupt enable */
#define MII_KSZ80X1_INT_PREN         (1 << 13) /* Page received interrupt enable */
#define MII_KSZ80X1_INT_PDFEN        (1 << 12) /* Parallel detect fault interrupt enable */
#define MII_KSZ80X1_INT_LPAEN        (1 << 11) /* Link partner acknowledge interrupt enable */
#define MII_KSZ80X1_INT_LDEN         (1 << 10) /* Link down fault interrupt enable */
#define MII_KSZ80X1_INT_RFEN         (1 << 9)  /* Remote fault interrupt enable */
#define MII_KSZ80X1_INT_LUEN         (1 << 8)  /* Link up interrupt enable */

#define MII_KSZ80X1_INT_J            (1 << 7)  /* Jabber interrupt */
#define MII_KSZ80X1_INT_RE           (1 << 6)  /* Receive error interrupt */
#define MII_KSZ80X1_INT_PR           (1 << 5)  /* Page received interrupt */
#define MII_KSZ80X1_INT_PDF          (1 << 4)  /* Parallel detect fault interrupt */
#define MII_KSZ80X1_INT_LPA          (1 << 3)  /* Link partner acknowledge interrupt */
#define MII_KSZ80X1_INT_LD           (1 << 2)  /* Link down fault interrupt */
#define MII_KSZ80X1_INT_RF           (1 << 1)  /* Remote fault interrupt */
#define MII_KSZ80X1_INT_LU           (1 << 0)  /* Link up interrupt */

/* KSZ8041 Register 0x1e: PHY Control 1 -- To be provided */

/* KSZ8041 Register 0x1f: PHY Control 2 */

#define MII_PHYCTRL2_MDIX            (1 << 15) /* Bit 15: Micrel/HP MDI/MDI-X state */
#define MII_PHYCTRL2_MDIX_SEL        (1 << 14) /* Bit 14: MDI/MDI-X select */
#define MII_PHYCTRL2_PSDIS           (1 << 13) /* Bit 13: Pair swap disable */
#define MII_PHYCTRL2_ENERGYDET       (1 << 12) /* Bit 12: Energy detect */
#define MII_PHYCTRL2_FORCE           (1 << 11) /* Bit 11: Force link */
#define MII_PHYCTRL2_PWRSAVE         (1 << 10) /* Bit 10: Power saving */
#define MII_PHYCTRL2_INTLVL          (1 << 9)  /* Bit 9:  Interrupt level */
#define MII_PHYCTRL2_ENJABBER        (1 << 8)  /* Bit 8:  Enable jabber */
#define MII_PHYCTRL2_ANEGCOMP        (1 << 7)  /* Bit 7:  Auto-negotiation complete */
#define MII_PHYCTRL2_ENPAUSE         (1 << 6)  /* Bit 6:  Enable pause */
#define MII_PHYCTRL2_ISOLATE         (1 << 5)  /* Bit 5:  PHY isolate */
#define MII_PHYCTRL2_MODE_SHIFT      (2)       /* Bits 2-4: Operation mode */
#define MII_PHYCTRL2_MODE_MASK       (7 << MII_PHYCTRL2_MODE_SHIFT)
#  define MII_PHYCTRL2_MODE_BUSY     (0 << MII_PHYCTRL2_MODE_SHIFT) /* Still in autonegotiation */
#  define MII_PHYCTRL2_MODE_10HDX    (1 << MII_PHYCTRL2_MODE_SHIFT) /* 10Base-T half-duplex */
#  define MII_PHYCTRL2_MODE_100HDX   (2 << MII_PHYCTRL2_MODE_SHIFT) /* 100Base-T half-duplex */
#  define MII_PHYCTRL2_MODE_DUPLEX   (4 << MII_PHYCTRL2_MODE_SHIFT) /* Full duplex */
#  define MII_PHYCTRL2_MODE_10FDX    (5 << MII_PHYCTRL2_MODE_SHIFT) /* 10Base-T full-duplex */
#  define MII_PHYCTRL2_MODE_100FDX   (6 << MII_PHYCTRL2_MODE_SHIFT) /* 100Base-T full-duplex */

#define MII_PHYCTRL2_SEQTEST         (1 << 1)  /* Bit 1:  Enable SQE test */
#define MII_PHYCTRL2_DISDS           (1 << 0)  /* Bit 1:  Disable data scrambling */

/* KSZ8051/81 Register 0x1e: PHY Control 1 */

                                               /* Bits 10-15: Reserved */
#define MII_PHYCTRL1_ENPAUSE         (1 << 9)  /* Bit 9:  Enable pause */
#define MII_PHYCTRL1_LINKSTATUS      (1 << 8)  /* Bit 8:  Link status */
#define MII_PHYCTRL1_POLARITY        (1 << 7)  /* Bit 7:  Polarity status */
                                               /* Bit 6:  Reserved */
#define MII_PHYCTRL1_MDIX            (1 << 5)  /* Bit 5:  MDI/MDI-X state */
#define MII_PHYCTRL1_ENERGYDET       (1 << 4)  /* Bit 4:  Energy detect */
#define MII_PHYCTRL1_ISOLATE         (1 << 3)  /* Bit 3:  PHY isolate */
#define MII_PHYCTRL1_MODE_SHIFT      (0)       /* Bits 0-2: Operation mode */
#define MII_PHYCTRL1_MODE_MASK       (7 << MII_PHYCTRL1_MODE_SHIFT)
#  define MII_PHYCTRL1_MODE_BUSY     (0 << MII_PHYCTRL1_MODE_SHIFT) /* Still in autonegotiation */
#  define MII_PHYCTRL1_MODE_10HDX    (1 << MII_PHYCTRL1_MODE_SHIFT) /* 10Base-T half-duplex */
#  define MII_PHYCTRL1_MODE_100HDX   (2 << MII_PHYCTRL1_MODE_SHIFT) /* 100Base-T half-duplex */
#  define MII_PHYCTRL1_MODE_DUPLEX   (4 << MII_PHYCTRL1_MODE_SHIFT) /* Full duplex */
#  define MII_PHYCTRL1_MODE_10FDX    (5 << MII_PHYCTRL1_MODE_SHIFT) /* 10Base-T full-duplex */
#  define MII_PHYCTRL1_MODE_100FDX   (6 << MII_PHYCTRL1_MODE_SHIFT) /* 100Base-T full-duplex */

/* TJA110X register bit settings ********************************************/

/* TJA110X MII ID1/2 register bits */

#define MII_PHYID1_TJA1100                0x0180  /* ID1 value for NXP TJA1100 */
#define MII_PHYID2_TJA1100                0xdc40  /* ID2 value for NXP TJA1100 */

#define MII_PHYID1_TJA1101                0x0180  /* ID1 value for NXP TJA1101 */
#define MII_PHYID2_TJA1101                0xdd00  /* ID2 value for NXP TJA1101 */

#define MII_TJA110X_BCR                   0x0     /* Basic Control register */
#define MII_TJA110X_BSR                   0x1     /* Basic Status register */
#define MII_TJA110X_EXT_CNTRL             0x11    /* Extra control register */
#define MII_TJA110X_CONFIG1               0x12    /* CONFIG 1 register */
#define MII_TJA110X_CONFIG2               0x13    /* CONFIG 2 register */

/* MII_TJA110X_EXT_CNTRL */

#define MII_EXT_CNTRL_LINK_CNTRL          (1   << 15)
#define MII_EXT_CNTRL_POWER_MODE_SHIFT    (11)
#define MII_EXT_CNTRL_POWER_MODE_MASK     (0xf << MII_EXT_CNTRL_POWER_MODE_SHIFT)
#  define MII_EXT_CNTRL_NOCHANGE          (0x0 << MII_EXT_CNTRL_POWER_MODE_SHIFT)
#  define MII_EXT_CNTRL_NORMAL            (0x3 << MII_EXT_CNTRL_POWER_MODE_SHIFT)
#  define MII_EXT_CNTRL_STBY              (0xc << MII_EXT_CNTRL_POWER_MODE_SHIFT)
#  define MII_EXT_CNTRL_SLEEP_REQ         (0xb << MII_EXT_CNTRL_POWER_MODE_SHIFT)
#  define MII_EXT_CNTRL_PWR_MASK          (0xf << MII_EXT_CNTRL_POWER_MODE_SHIFT)
#define MII_EXT_CNTRL_SLAVE_JITTER_TEST   (1 << 10)
#define MII_EXT_CNTRL_TRAIN               (1 << 9)

#define MII_EXT_CNTRL_TEST_SHIFT          (6)
#define MII_EXT_CNTRL_TEST_MASK           (7 << MII_EXT_CNTRL_TEST_SHIFT)
#  define MII_EXT_CNTRL_TEST1             (1 << MII_EXT_CNTRL_TEST_SHIFT)
#  define MII_EXT_CNTRL_TEST2             (2 << MII_EXT_CNTRL_TEST_SHIFT)
#  define MII_EXT_CNTRL_TEST3             (3 << MII_EXT_CNTRL_TEST_SHIFT)
#  define MII_EXT_CNTRL_TEST4             (4 << MII_EXT_CNTRL_TEST_SHIFT)
#  define MII_EXT_CNTRL_TEST5             (5 << MII_EXT_CNTRL_TEST_SHIFT)
#  define MII_EXT_CNTRL_TEST6             (6 << MII_EXT_CNTRL_TEST_SHIFT)
#  define MII_EXT_CNTRL_TEST7             (7 << MII_EXT_CNTRL_TEST_SHIFT)
#define MII_EXT_CNTRL_CABLE_TST           (1 <<  5)
#define MII_EXT_CNTRL_LOOPBACK_MODE_SHIFT (3)
#define MII_EXT_CNTRL_LOOPBACK_MODE_MASK  (3 << MII_EXT_CNTRL_LOOPBACK_MODE_SHIFT)
#define MII_EXT_CNTRL_INT_LPB             (0 << MII_EXT_CNTRL_LOOPBACK_MODE_SHIFT)
#define MII_EXT_CNTRL_EXT1_LPB            (1 << MII_EXT_CNTRL_LOOPBACK_MODE_SHIFT)
#define MII_EXT_CNTRL_EXT2_LPB            (2 << MII_EXT_CNTRL_LOOPBACK_MODE_SHIFT)
#define MII_EXT_CNTRL_REM_LPB             (3 << MII_EXT_CNTRL_LOOPBACK_MODE_SHIFT)
#define MII_EXT_CNTRL_CONFIG_EN           (1 <<  2)
#define MII_EXT_CNTRL_CONFIG_INH          (1 <<  1)
#define MII_EXT_CNTRL_WAKE_REQ            (1 <<  0)  /* transmit idle symbols as bus wake-up request */

/* MII_TJA110X_CONFIG1 */

#define MII_CONFIG1_MASTER                (1 << 15)
#define MII_CONFIG1_AUTO_OP               (1 << 14)
#define MII_CONFIG1_LINK_15M              (1 << 13)  /* cable length > 15 m */
#define MII_CONFIG1_TX_AMPLITUDE_SHIFT    (10)
#define MII_CONFIG1_TX_AMPLITUDE_MASK     (3 << MII_CONFIG1_TX_AMPLITUDE_SHIFT)
#  define MII_CONFIG1_TX_500MV            (0 << MII_CONFIG1_TX_AMPLITUDE_SHIFT)
#  define MII_CONFIG1_TX_750MV            (1 << MII_CONFIG1_TX_AMPLITUDE_SHIFT)
#  define MII_CONFIG1_TX_1000MV           (2 << MII_CONFIG1_TX_AMPLITUDE_SHIFT)
#  define MII_CONFIG1_TX_1250MV           (3 << MII_CONFIG1_TX_AMPLITUDE_SHIFT)
#define MII_CONFIG1_MII_MODE_SHIFT        (8)
#define MII_CONFIG1_MII_MODE_MASK         (3 << MII_CONFIG1_MII_MODE_SHIFT)
#  define MII_CONFIG1_MII_MODE            (0 << MII_CONFIG1_MII_MODE_SHIFT)
#  define MII_CONFIG1_RMII_50MHZ          (1 << MII_CONFIG1_MII_MODE_SHIFT)
#  define MII_CONFIG1_RMII_25MHZ          (2 << MII_CONFIG1_MII_MODE_SHIFT)
#  define MII_CONFIG1_REV_MII             (3 << MII_CONFIG1_MII_MODE_SHIFT)
#define MII_CONFIG1_MII_DRV_RED           (1 << 7)  /* reduced strength MII output driver */
#define MII_CONFIG1_LEDLINK_SHIFT         (4)
#define MII_CONFIG1_LEDLINK_MASK          (3 << MII_CONFIG1_LEDLINK_SHIFT)
#  define MII_CONFIG1_LEDLINK             (0 << MII_CONFIG1_LEDLINK_SHIFT)
#  define MII_CONFIG1_LEDFRAME            (1 << MII_CONFIG1_LEDLINK_SHIFT)
#  define MII_CONFIG1_LEDSYMERR           (2 << MII_CONFIG1_LEDLINK_SHIFT)
#  define MII_CONFIG1_LEDCRS              (3 << MII_CONFIG1_LEDLINK_SHIFT)
#define MII_CONFIG1_LED_EN                (1 <<  3)
#define MII_CONFIG1_CNFG_WAKE             (1 <<  2)  /* ratiometric input threshold, absolute if zero */
#define MII_CONFIG1_AUTO_PWD              (1 <<  1)  /* autonomous power-down enabled */

/* MII_TJA110X_CONFIG2 */

#define MII_CONFIG2_PHYAD_SHIFT           (11)  /* readback of scrambler key */
#define MII_CONFIG2_PHYAD_MASK            (0x1f << MII_CONFIG2_PHYAD_SHIFT)
#define MII_CONFIG2_SNR_SHIFT             (9)  /* signal to noise ratio averaging */
#define MII_CONFIG2_SNR_MASK              (3 << MII_CONFIG2_SNR_SHIFT)
#  define MII_CONFIG2_SNR_AV32            (0 <<  MII_CONFIG2_SNR_SHIFT)  /* signal to noise ratio averaging over  32 symbols */
#  define MII_CONFIG2_SNR_AV64            (1 <<  MII_CONFIG2_SNR_SHIFT)  /* signal to noise ratio averaging over  64 symbols */
#  define MII_CONFIG2_SNR_AV128           (2 <<  MII_CONFIG2_SNR_SHIFT)  /* signal to noise ratio averaging over 128 symbols */
#  define MII_CONFIG2_SNR_AV256           (3 <<  MII_CONFIG2_SNR_SHIFT)  /* signal to noise ratio averaging over 256 symbols */

#define MII_CONFIG2_WLIM_SHIFT            (6)  /* SQI warning limit */
#define MII_CONFIG2_WLIM_MASK             (7 << MII_CONFIG2_WLIM_SHIFT)
#  define MII_CONFIG2_WLIM_NO             (0 << MII_CONFIG2_WLIM_SHIFT)  /* no warning */
#  define MII_CONFIG2_WLIM_A              (1 << MII_CONFIG2_WLIM_SHIFT)  /* Class A SNR warning limit */
#  define MII_CONFIG2_WLIM_B              (2 << MII_CONFIG2_WLIM_SHIFT)  /* Class B SNR warning limit */
#  define MII_CONFIG2_WLIM_C              (3 << MII_CONFIG2_WLIM_SHIFT)  /* Class C SNR warning limit */
#  define MII_CONFIG2_WLIM_D              (4 << MII_CONFIG2_WLIM_SHIFT)  /* Class D SNR warning limit */
#  define MII_CONFIG2_WLIM_E              (5 << MII_CONFIG2_WLIM_SHIFT)  /* Class E SNR warning limit */
#  define MII_CONFIG2_WLIM_F              (6 << MII_CONFIG2_WLIM_SHIFT)  /* Class F SNR warning limit */
#  define MII_CONFIG2_WLIM_G              (7 << MII_CONFIG2_WLIM_SHIFT)  /* Class G SNR warning limit */

#define MII_CONFIG2_SNR_F_SHIFT           (3)  /* signal to noise ratio fail limit */
#define MII_CONFIG2_SNR_F_MASK            (7 << MII_CONFIG2_SNR_F_SHIFT)
#  define MII_CONFIG2_SNR_F_NL            (0 << MII_CONFIG2_SNR_F_SHIFT)  /* no limit */
#  define MII_CONFIG2_SNR_F_CLA           (1 << MII_CONFIG2_SNR_F_SHIFT)  /* Class A */
#  define MII_CONFIG2_SNR_F_CLB           (2 << MII_CONFIG2_SNR_F_SHIFT)  /* Class B */
#  define MII_CONFIG2_SNR_F_CLC           (3 << MII_CONFIG2_SNR_F_SHIFT)  /* Class C */
#  define MII_CONFIG2_SNR_F_CLD           (4 << MII_CONFIG2_SNR_F_SHIFT)  /* Class D */
#  define MII_CONFIG2_SNR_F_CLE           (5 << MII_CONFIG2_SNR_F_SHIFT)  /* Class E */
#  define MII_CONFIG2_SNR_F_CLF           (6 << MII_CONFIG2_SNR_F_SHIFT)  /* Class F */
#  define MII_CONFIG2_SNR_F_CLG           (7 << MII_CONFIG2_SNR_F_SHIFT)  /* Class G */

#define MII_CONFIG2_JUMBO_EN              (1 << 2)  /* enable packets up to 16 kB instead of 4 kB */
#define MII_CONFIG2_SLP_T_SHIFT           (0)       /* sleep request timeout */
#define MII_CONFIG2_SLP_T_MASK            (3 << MII_CONFIG2_SLP_T_SHIFT)
#  define MII_CONFIG2_SLP_T_04            (0 << MII_CONFIG2_SLP_T_SHIFT)  /* sleep request timeout 0.4 ms */
#  define MII_CONFIG2_SLP_T_1             (1 << MII_CONFIG2_SLP_T_SHIFT)  /* sleep request timeout 1 ms */
#  define MII_CONFIG2_SLP_T_4             (2 << MII_CONFIG2_SLP_T_SHIFT)  /* sleep request timeout 4 ms */
#  define MII_CONFIG2_SLP_T_16            (3 << MII_CONFIG2_SLP_T_SHIFT)  /* sleep request timeout 16 ms */

/* DP83848C-specific register bit settings **********************************/

/* DP83848C Register 0x10: PHY Status */

#define MII_DP83848C_PHYSTS_SPEED     (1 << 1)  /* Bit 1: Speed Status Register */
#define MII_DP83848C_PHYSTS_DUPLEX    (1 << 2)  /* Bit 2: Duplex Status Register */

/* DP83848C Register 0x11: Interrupt control/status */

#define MII_DP83848C_INT_TEST         (1 << 2)  /* Test Interrupt */
#define MII_DP83848C_INT_EN           (1 << 1)  /* Interrupt enable */
#define MII_DP83848C_INT_OEN          (1 << 0)  /* Interrupt output enable */

/* DP83848C Register 0x12: Interrupt control/status */

#define MII_DP83848C_ED_INT           (1 << 14) /* Energy Detect interrupt: */
#define MII_DP83848C_LINK_INT         (1 << 13) /* Change of Link Status interrupt: */
#define MII_DP83848C_SPD_INT          (1 << 12) /* Change of speed status interrupt: */
#define MII_DP83848C_DUP_INT          (1 << 11) /* Change of duplex status interrupt: */
#define MII_DP83848C_ANC_INT          (1 << 10) /* Auto-Negotiation Complete interrupt: */
#define MII_DP83848C_FHF_INT          (1 << 9)  /* False Carrier Counter half-full interrupt: */
#define MII_DP83848C_RHF_INT          (1 << 8)  /* Receive Error Counter half-full interrupt: */
#define MII_DP83848C_ED_INT_EN        (1 << 6)  /* Enable Interrupt on energy detect event. */
#define MII_DP83848C_LINK_INT_EN      (1 << 5)  /* Enable Interrupt on change of link status. */
#define MII_DP83848C_SPD_INT_EN       (1 << 4)  /* Enable Interrupt on change of speed status. */
#define MII_DP83848C_DUP_INT_EN       (1 << 3)  /* Enable Interrupt on change of duplex status. */
#define MII_DP83848C_ANC_INT_EN       (1 << 2)  /* Enable Interrupt on Auto-negotiation complete event. */
#define MII_DP83848C_FHF_INT_EN       (1 << 1)  /* Enable Interrupt on False Carrier Counter Register half-full event. */
#define MII_DP83848C_RHF_INT_EN       (1 << 0)  /* Enable Interrupt on Receive Error Counter Register half-full event. */

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_NET_MII_H */
