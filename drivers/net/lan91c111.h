/****************************************************************************
 * drivers/net/lan91c111.h
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

#ifndef __DRIVERS_NET_LAN91C111_H
#define __DRIVERS_NET_LAN91C111_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/net/mii.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Bank Select Register:
 *
 *              yyyy yyyy 0000 00xx
 *              xx              = bank number
 *              yyyy yyyy       = 0x33, for identification purposes.
 */

#define BANK_SELECT             14

/* Transmit Control Register
 * BANK 0
 */

#define TCR_REG                 0x0000
#define TCR_ENABLE              0x0001 /* When 1 we can transmit */
#define TCR_LOOP                0x0002 /* Controls output pin LBK */
#define TCR_FORCOL              0x0004 /* When 1 will force a collision */
#define TCR_PAD_EN              0x0080 /* When 1 will pad tx frames < 64 bytes w/0 */
#define TCR_NOCRC               0x0100 /* When 1 will not append CRC to tx frames */
#define TCR_MON_CSN             0x0400 /* When 1 tx monitors carrier */
#define TCR_FDUPLX              0x0800 /* When 1 enables full duplex operation */
#define TCR_STP_SQET            0x1000 /* When 1 stops tx if Signal Quality Error */
#define TCR_EPH_LOOP            0x2000 /* When 1 enables EPH block loopback */
#define TCR_SWFDUP              0x8000 /* When 1 enables Switched Full Duplex mode */

#define TCR_CLEAR               0      /* do NOTHING */

/* the default settings for the TCR register : */

#define TCR_DEFAULT             (TCR_ENABLE | TCR_PAD_EN)

/* EPH Status Register
 * BANK 0
 */

#define EPH_STATUS_REG          0x0002
#define ES_TX_SUC               0x0001 /* Last TX was successful */
#define ES_SNGL_COL             0x0002 /* Single collision detected for last tx */
#define ES_MUL_COL              0x0004 /* Multiple collisions detected for last tx */
#define ES_LTX_MULT             0x0008 /* Last tx was a multicast */
#define ES_16COL                0x0010 /* 16 Collisions Reached */
#define ES_SQET                 0x0020 /* Signal Quality Error Test */
#define ES_LTXBRD               0x0040 /* Last tx was a broadcast */
#define ES_TXDEFR               0x0080 /* Transmit Deferred */
#define ES_LATCOL               0x0200 /* Late collision detected on last tx */
#define ES_LOSTCARR             0x0400 /* Lost Carrier Sense */
#define ES_EXC_DEF              0x0800 /* Excessive Deferral */
#define ES_CTR_ROL              0x1000 /* Counter Roll Over indication */
#define ES_LINK_OK              0x4000 /* Driven by inverted value of nLNK pin */
#define ES_TXUNRN               0x8000 /* Tx Underrun */
#define ES_ERRORS               (ES_TXUNRN | ES_LOSTCARR | ES_LATCOL | ES_SQET | ES_16COL)

/* Receive Control Register
 * BANK 0
 */

#define RCR_REG                 0x0004
#define RCR_RX_ABORT            0x0001 /* Set if a rx frame was aborted */
#define RCR_PRMS                0x0002 /* Enable promiscuous mode */
#define RCR_ALMUL               0x0004 /* When set accepts all multicast frames */
#define RCR_RXEN                0x0100 /* IF this is set, we can receive packets */
#define RCR_STRIP_CRC           0x0200 /* When set strips CRC from rx packets */
#define RCR_ABORT_ENB           0x2000 /* When set will abort rx on collision */
#define RCR_FILT_CAR            0x4000 /* When set filters leading 12 bit s of carrier */
#define RCR_SOFTRST             0x8000 /* resets the chip */

/* the normal settings for the RCR register : */

#ifdef CONFIG_NET_PROMISCUOUS
#  define RCR_DEFAULT           (RCR_STRIP_CRC | RCR_RXEN | RCR_PRMS)
#else
#  define RCR_DEFAULT           (RCR_STRIP_CRC | RCR_RXEN)
#endif
#define RCR_CLEAR               0x0    /* set it to a base state */

/* Counter Register
 * BANK 0
 */

#define COUNTER_REG             0x0006

/* Memory Information Register
 * BANK 0
 */

#define MIR_REG                 0x0008
#define MIR_FREE_MASK           0xff00

/* Receive/Phy Control Register
 * BANK 0
 */

#define RPC_REG                 0x000a
#define RPC_SPEED               0x2000 /* When 1 PHY is in 100Mbps mode. */
#define RPC_DPLX                0x1000 /* When 1 PHY is in Full-Duplex Mode */
#define RPC_ANEG                0x0800 /* When 1 PHY is in Auto-Negotiate Mode */
#define RPC_LSXA_SHFT           5      /* Bits to shift LS2A,LS1A,LS0A to lsb */
#define RPC_LSXB_SHFT           2      /* Bits to get LS2B,LS1B,LS0B to lsb */
#define RPC_LED_100_10          0x00   /* LED = 100Mbps OR's with 10Mbps link detect */
#define RPC_LED_RES             0x01   /* LED = Reserved */
#define RPC_LED_10              0x02   /* LED = 10Mbps link detect */
#define RPC_LED_FD              0x03   /* LED = Full Duplex Mode */
#define RPC_LED_TX_RX           0x04   /* LED = TX or RX packet occurred */
#define RPC_LED_100             0x05   /* LED = 100Mbps link detect */
#define RPC_LED_TX              0x06   /* LED = TX packet occurred */
#define RPC_LED_RX              0x07   /* LED = RX packet occurred */
#define RPC_DEFAULT             (RPC_ANEG | RPC_SPEED | RPC_DPLX    \
                                | (RPC_LED_100_10 << RPC_LSXA_SHFT) \
                                | (RPC_LED_TX_RX << RPC_LSXB_SHFT))

/* Bank 0 0x000c is reserved */

/* Bank Select Register
 * All Banks
 */

#define BSR_REG                 0x000e

/* Configuration Reg
 * BANK 1
 */

#define CONFIG_REG              0x0100
#define CONFIG_EXT_PHY          0x0200 /* 1=external MII, 0=internal Phy */
#define CONFIG_GPCNTRL          0x0400 /* Inverse value drives pin nCNTRL */
#define CONFIG_NO_WAIT          0x1000 /* When 1 no extra wait states on ISA bus */
#define CONFIG_EPH_POWER_EN     0x8000 /* When 0 EPH is placed into low power mode. */

/* Default is powered-up, Internal Phy, Wait States, and pin nCNTRL=low */

#define CONFIG_DEFAULT          (CONFIG_EPH_POWER_EN)
#define CONFIG_CLEAR             0

/* Base Address Register
 * BANK 1
 */

#define BASE_REG                0x0102

/* Individual Address Registers
 * BANK 1
 */

#define ADDR0_REG               0x0104
#define ADDR1_REG               0x0106
#define ADDR2_REG               0x0108

/* General Purpose Register
 * BANK 1
 */

#define  GP_REG                 0x010a

/* Control Register
 * BANK 1
 */

#define CTL_REG                 0x010c
#define CTL_RCV_BAD             0x4000 /* When 1 bad CRC packets are received */
#define CTL_AUTO_RELEASE        0x0800 /* When 1 tx pages are released automatically */
#define CTL_LE_ENABLE           0x0080 /* When 1 enables Link Error interrupt */
#define CTL_CR_ENABLE           0x0040 /* When 1 enables Counter Rollover interrupt */
#define CTL_TE_ENABLE           0x0020 /* When 1 enables Transmit Error interrupt */
#define CTL_EEPROM_SELECT       0x0004 /* Controls EEPROM reload & store */
#define CTL_RELOAD              0x0002 /* When set reads EEPROM into registers */
#define CTL_STORE               0x0001 /* When set stores registers into EEPROM */

#define CTL_DEFAULT             (CTL_AUTO_RELEASE)
#define CTL_CLEAR               0

/* MMU Command Register
 * BANK 2
 */

#define MMU_CMD_REG             0x0200
#define MC_BUSY                 1      /* When 1 the last release has not completed */
#define MC_NOP                  (0<<5) /* No Op */
#define MC_ALLOC                (1<<5) /* OR with number of 256 byte packets */
#define MC_RESET                (2<<5) /* Reset MMU to initial state */
#define MC_REMOVE               (3<<5) /* Remove the current rx packet */
#define MC_RELEASE              (4<<5) /* Remove and release the current rx packet */
#define MC_FREEPKT              (5<<5) /* Release packet in PNR register */
#define MC_ENQUEUE              (6<<5) /* Enqueue the packet for transmit */
#define MC_RSTTXFIFO            (7<<5) /* Reset the TX FIFOs */

/* Packet Number Register
 * BANK 2
 */

#define  PN_REG                 0x0202

/* Allocation Result Register
 * BANK 2
 */

#define AR_REG                  0x0203
#define AR_FAILED               0x80   /* Allocation Failed */

/* TX FIFO Ports Register
 * BANK 2
 */

#define TXFIFO_REG              0x0204
#define TXFIFO_TEMPTY           0x80   /* TX FIFO Empty */

/* RX FIFO Ports Register
 * BANK 2
 */

#define RXFIFO_REG              0x0205
#define RXFIFO_REMPTY           0x80   /* RX FIFO Empty */

#define FIFO_REG                0x0204

/* Pointer Register
 * BANK 2
 */

#define PTR_REG                 0x0206
#define PTR_RCV                 0x8000 /* 1=Receive area, 0=Transmit area */
#define PTR_AUTOINC             0x4000 /* Auto increment the pointer on each access */
#define PTR_READ                0x2000 /* When 1 the operation is a read */
#define PTR_NOTEMPTY            0x0800 /* When 1 _do not_ write fifo DATA REG */

/* Data Register
 * BANK 2
 */

#define DATA_REG                0x0208

/* Interrupt Status/Acknowledge Register
 * BANK 2
 */

#define INT_REG                 0x020c

/* Interrupt Mask Register
 * BANK 2
 */

#define IM_REG                  0x020d
#define IM_MDINT                0x80   /* PHY MI Register 18 Interrupt */
#define IM_ERCV_INT             0x40   /* Early Receive Interrupt */
#define IM_EPH_INT              0x20   /* Set by Etheret Protocol Handler section */
#define IM_RX_OVRN_INT          0x10   /* Set by Receiver Overruns */
#define IM_ALLOC_INT            0x08   /* Set when allocation request is completed */
#define IM_TX_EMPTY_INT         0x04   /* Set if the TX FIFO goes empty */
#define IM_TX_INT               0x02   /* Transmit Interrupt */
#define IM_RCV_INT              0x01   /* Receive Interrupt */

/* Multicast Table Registers
 * BANK 3
 */

#define MCAST_REG1              0x0300
#define MCAST_REG2              0x0302
#define MCAST_REG3              0x0304
#define MCAST_REG4              0x0306

/* Management Interface Register (MII)
 * BANK 3
 */

#define MII_REG                 0x0308
#define MII_MSK_CRS100          0x4000 /* Disables CRS100 detection during tx half dup */
#define MII_MDOE                0x0008 /* MII Output Enable */
#define MII_MCLK                0x0004 /* MII Clock, pin MDCLK */
#define MII_MDI                 0x0002 /* MII Input, pin MDI */
#define MII_MDO                 0x0001 /* MII Output, pin MDO */

/* Revision Register
 * BANK 3
 * ( hi: chip id   low: rev # )
 */

#define REV_REG                 0x030a

/* Early RCV Register
 * BANK 3
 * this is NOT on SMC9192
 */

#define ERCV_REG                0x030c
#define ERCV_RCV_DISCRD         0x0080 /* When 1 discards a packet being received */
#define ERCV_THRESHOLD          0x001f /* ERCV Threshold Mask */

/* External Register
 * BANK 7
 */

#define EXT_REG                 0x0700

#define CHIP_9192               3
#define CHIP_9194               4
#define CHIP_9195               5
#define CHIP_9196               6
#define CHIP_91100              7
#define CHIP_91100FD            8
#define CHIP_91111FD            9

/* Transmit status bits
 * Same as ES_xxx
 */

/* Transmit control bits */

#define TC_ODD                  0x20
#define TC_CRC                  0x10

/* Receive status bits */

#define RS_ALGNERR              0x8000
#define RS_BRODCAST             0x4000
#define RS_BADCRC               0x2000
#define RS_ODDFRAME             0x1000
#define RS_TOOLONG              0x0800
#define RS_TOOSHORT             0x0400
#define RS_MULTICAST            0x0001
#define RS_ERRORS               (RS_ALGNERR | RS_BADCRC | RS_TOOLONG | RS_TOOSHORT)

/* Receive control bits */

#define RC_ODD                  0x20

/* PHY IDs
 *  LAN83C183 == LAN91C111 Internal PHY
 */

#define PHY_LAN83C183           0x0016f840
#define PHY_LAN83C180           0x02821c50

/* LPA full duplex flags */

#define MII_LPA_FULL            (MII_LPA_10BASETXFULL | MII_LPA_100BASETXFULL)

/* PHY Register Addresses (LAN91C111 Internal PHY)
 *
 * Generic PHY registers can be found in <nuttx/net/mii.h>
 *
 * These phy registers are specific to our on-board phy.
 */

/* PHY Configuration Register 1 */

#define PHY_CFG1_REG            0x10
#define PHY_CFG1_LNKDIS         0x8000 /* 1=Rx Link Detect Function disabled */
#define PHY_CFG1_XMTDIS         0x4000 /* 1=TP Transmitter Disabled */
#define PHY_CFG1_XMTPDN         0x2000 /* 1=TP Transmitter Powered Down */
#define PHY_CFG1_BYPSCR         0x0400 /* 1=Bypass scrambler/descrambler */
#define PHY_CFG1_UNSCDS         0x0200 /* 1=Unscramble Idle Reception Disable */
#define PHY_CFG1_EQLZR          0x0100 /* 1=Rx Equalizer Disabled */
#define PHY_CFG1_CABLE          0x0080 /* 1=STP(150ohm), 0=UTP(100ohm) */
#define PHY_CFG1_RLVL0          0x0040 /* 1=Rx Squelch level reduced by 4.5db */
#define PHY_CFG1_TLVL_SHIFT     2      /* Transmit Output Level Adjust */
#define PHY_CFG1_TLVL_MASK      0x003c
#define PHY_CFG1_TRF_MASK       0x0003 /* Transmitter Rise/Fall time */

/* PHY Configuration Register 2 */

#define PHY_CFG2_REG            0x11
#define PHY_CFG2_APOLDIS        0x0020 /* 1=Auto Polarity Correction disabled */
#define PHY_CFG2_JABDIS         0x0010 /* 1=Jabber disabled */
#define PHY_CFG2_MREG           0x0008 /* 1=Multiple register access (MII mgt) */
#define PHY_CFG2_INTMDIO        0x0004 /* 1=Interrupt signaled with MDIO pulseo */

/* PHY Status Output (and Interrupt status) Register */

#define PHY_INT_REG             0x12   /* Status Output (Interrupt Status) */
#define PHY_INT_INT             0x8000 /* 1=bits have changed since last read */
#define PHY_INT_LNKFAIL         0x4000 /* 1=Link Not detected */
#define PHY_INT_LOSSSYNC        0x2000 /* 1=Descrambler has lost sync */
#define PHY_INT_CWRD            0x1000 /* 1=Invalid 4B5B code detected on rx */
#define PHY_INT_SSD             0x0800 /* 1=No Start Of Stream detected on rx */
#define PHY_INT_ESD             0x0400 /* 1=No End Of Stream detected on rx */
#define PHY_INT_RPOL            0x0200 /* 1=Reverse Polarity detected */
#define PHY_INT_JAB             0x0100 /* 1=Jabber detected */
#define PHY_INT_SPDDET          0x0080 /* 1=100Base-TX mode, 0=10Base-T mode */
#define PHY_INT_DPLXDET         0x0040 /* 1=Device in Full Duplex */

/* PHY Interrupt/Status Mask Register */

#define PHY_MASK_REG            0x13   /* Interrupt Mask */

/* Uses the same bit definitions as PHY_INT_REG */

#endif /* __DRIVERS_NET_LAN91C111_H */
