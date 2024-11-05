/****************************************************************************
 * drivers/net/lan9250.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing software
 * distributed under the License is distributed on an "AS IS" BASIS WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __DRIVERS_NET_LAN9250_H
#define __DRIVERS_NET_LAN9250_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LAN9250 SPI Commands *****************************************************/

#define LAN9250_SPI_CMD_READ        0x03
#define LAN9250_SPI_CMD_FREAD       0x0b
#define LAN9250_SPI_CMD_WRITE       0x02
#define LAN9250_SPI_CMD_ENABLE_SQI  0x38

#define LAN9250_QSPI_CMD_READ       0xeb
#define LAN9250_QSPI_CMD_WRITE      0xe2

/* System Control and Status Registers **************************************/

#define LAN9250_RXDFR               0x000
#define LAN9250_TXDFR               0x020
#define LAN9250_RXSFR               0x040
#define LAN9250_RXSPR               0x044
#define LAN9250_TXSFR               0x048
#define LAN9250_TXSPR               0x04c
#define LAN9250_CIARR               0x050
#define LAN9250_ICFGR               0x054
#define LAN9250_ISR                 0x058
#define LAN9250_IER                 0x05c
#define LAN9250_BOTR                0x064
#define LAN9250_FLIR                0x068
#define LAN9250_RXCFGR              0x06c
#define LAN9250_TXCFGR              0x070
#define LAN9250_HWCFGR              0x074
#define LAN9250_RDCR                0x078
#define LAN9250_RXFIR               0x07c
#define LAN9250_TXFIR               0x080
#define LAN9250_PMCR                0x084
#define LAN9250_GPTCFGR             0x08c
#define LAN9250_GPTCR               0x090
#define LAN9250_FR25MCR             0x09c
#define LAN9250_HMRDFCR             0x0a0
#define LAN9250_HMCSRICR            0x0a4
#define LAN9250_HMCSRIDR            0x0a8
#define LAN9250_HMAFCCFGR           0x0ac
#define LAN9250_HMRXLPITXR          0x0b0
#define LAN9250_HMRXLPITR           0x0b4
#define LAN9250_HMTXLPITXR          0x0b8
#define LAN9250_HMTXLPITR           0x0bc

/* 1588 Register Addresses */

#define LAN9250_1588CACR            0x100
#define LAN9250_1588GCFGR           0x104
#define LAN9250_1588ISR             0x108
#define LAN9250_1588IER             0x10c
#define LAN9250_1588CLKSR           0x110
#define LAN9250_1588CLKNSR          0x114
#define LAN9250_1588CLKSNSR         0x118
#define LAN9250_1588CLKRAR          0x11c
#define LAN9250_1588CLKTRAR         0x120
#define LAN9250_1588CLKTRDR         0x124
#define LAN9250_1588CLKSAR          0x128
#define LAN9250_1588CLKATXSR        0x12c
#define LAN9250_1588CLKATXNSR       0x130
#define LAN9250_1588CLKATXRASR      0x134
#define LAN9250_1588CLKATXRANSR     0x138
#define LAN9250_1588CLKBTXSR        0x13c
#define LAN9250_1588CLKBTXNSR       0x140
#define LAN9250_1588CLKBTXRASR      0x144
#define LAN9250_1588CLKBTXRANSR     0x148
#define LAN9250_1588UMAHWR          0x14c
#define LAN9250_1588UMALWR          0x150
#define LAN9250_1588BPGSR           0x154

/* Bank 0 Control Register Addresses */

#define LAN9250_1588PLR             0x158
#define LAN9250_1588PAAPDR          0x15c
#define LAN9250_1588PCIR            0x160

/* Bank 1 Control Register Addresses */

#define LAN9250_1588PRXPCR          0x158
#define LAN9250_1588PRXTSCR         0x15c
#define LAN9250_1588PRXTICR         0x160
#define LAN9250_1588PRXFCFGR        0x168
#define LAN9250_1588PRXITSR         0x16c
#define LAN9250_1588PRXITNSR        0x170
#define LAN9250_1588PRXMHR          0x174
#define LAN9250_1588PRXPDRITSR      0x178
#define LAN9250_1588PRXPDRITNSR     0x17c
#define LAN9250_1588PRXPDRICFHR     0x180
#define LAN9250_1588PRXPDRICFLR     0x184
#define LAN9250_1588PRXCSDCR        0x188
#define LAN9250_1588PRXFCR          0x18c

/* Bank 2 Control Register Addresses */

#define LAN9250_1588PTXPCR          0x158
#define LAN9250_1588PTXTSCR         0x15c
#define LAN9250_1588PTXMR           0x164
#define LAN9250_1588PTXM2R          0x168
#define LAN9250_1588PTXETSR         0x16c
#define LAN9250_1588PTXETNSR        0x170
#define LAN9250_1588PTXMHR          0x174
#define LAN9250_1588PTXDRDTSR       0x178
#define LAN9250_1588PTXDRDTNSR      0x17c
#define LAN9250_1588TXOSSUSR        0x180

/* Bank 3 Control Register Addresses */

#define LAN9250_1588GPIOCCFGR       0x15c
#define LAN9250_1588GPIOXRECLKSCR   0x16c
#define LAN9250_1588GPIOXRECLKNSCR  0x170
#define LAN9250_1588GPIOXFECLKSCR   0x178
#define LAN9250_1588GPIOXFECLKNSCR  0x17c

/* ISR_GPIO Register Addresses */

#define LAN9250_GPIOCFGR            0x1e0
#define LAN9250_GPIODADR            0X1e4
#define LAN9250_GPIOISAER           0X1e8

/* Reset Register Addresses */

#define LAN9250_RSTCR               0x1f8

/* Host MAC Registers *******************************************************/

#define LAN9250_HMACCR              0x01
#define LAN9250_HMACAHR             0x02
#define LAN9250_HMACALR             0x03
#define LAN9250_HMACHTHR            0x04
#define LAN9250_HMACHTLR            0x05
#define LAN9250_HMACMIIAR           0x06
#define LAN9250_HMACMIIDR           0x07
#define LAN9250_HMACFCR             0x08
#define LAN9250_HMACVLAN1TR         0x09
#define LAN9250_HMACVLAN2TR         0x0a
#define LAN9250_HMACWUFFR           0x0b
#define LAN9250_HMACWUCASR          0x0c
#define LAN9250_HMACCSOECR          0x0d
#define LAN9250_HMACEEETWTXSR       0x0e
#define LAN9250_HMACEEETXLPIRDR     0x0f

/* PHY Registers ************************************************************/

#define LAN9250_PHYBCR              0
#define LAN9250_PHYBSR              1
#define LAN9250_PHYIMSBR            2
#define LAN9250_PHYILSBR            3
#define LAN9250_PHYANAR             4
#define LAN9250_PHYANLPBPAR         5
#define LAN9250_PHYANER             6
#define LAN9250_PHYANNPTXR          7
#define LAN9250_PHYANNPRXR          8
#define LAN9250_PHYMMDACR           13
#define LAN9250_PHYMMDAODR          14
#define LAN9250_PHYEDPDCFGR         16
#define LAN9250_PHYMCOSR            17
#define LAN9250_PHYSMR              18
#define LAN9250_PHYTDRPODCR         24
#define LAN9250_PHYTDRCOSR          25
#define LAN9250_PHYSECR             26
#define LAN9250_PHYSCOSIR           27
#define LAN9250_PHYCLR              28
#define LAN9250_PHYISFR             29
#define LAN9250_PHYIER              30
#define LAN9250_PHYSCOSR            31

/* LAN9250 SPI Commands Fields **********************************************/

/* SPI Write Command A */

#define SPI_CMD_WRITE_A_IOC         0X80000000
#define SPI_CMD_WRITE_A_BEA         0x03000000
#define SPI_CMD_WRITE_A_4BA         0x00000000
#define SPI_CMD_WRITE_A_16BA        0x01000000
#define SPI_CMD_WRITE_A_32BA        0x02000000
#define SPI_CMD_WRITE_A_DO          0x00000000
#define SPI_CMD_WRITE_A_FS          0x00002000
#define SPI_CMD_WRITE_A_LS          0x00001000
#define SPI_CMD_WRITE_A_BS_M        0x000007ff

/* SPI Write Command B */

#define SPI_CMD_WRITE_B_PT          0x05200000
#define SPI_CMD_WRITE_B_CE          0x00004000
#define SPI_CMD_WRITE_B_ACD         0x00002000
#define SPI_CMD_WRITE_B_DP          0x00001000
#define SPI_CMD_WRITE_B_PL_M        0x000007ff

/* RX FIFO Status Format Fields *********************************************/

#define RXSFF_PF                    0x80000000
#define RXSFF_FF                    0x40000000
#define RXSFF_PL_M                  0x3fff0000
#define RXSFF_PL_S                  16
#define RXSFF_ES                    0x00008000
#define RXSFF_BF                    0x00002000
#define RXSFF_LE                    0x00001000
#define RXSFF_RF                    0x00000800
#define RXSFF_MF                    0x00000400
#define RXSFF_FTL                   0x00000080
#define RXSFF_CS                    0x00000040
#define RXSFF_FT                    0x00000020
#define RXSFF_RWTO                  0x00000010
#define RXSFF_MIIE                  0x00000008
#define RXSFF_DB                    0x00000004
#define RXSFF_CRCE                  0x00000002

/* System Control and Status Registers Fields *******************************/

/* RX FIFO Status Register */

#define RXFIR_RXSFUS_M              0x00ff0000
#define RXFIR_RXSFUS_S              16
#define RXFIR_RXDFUS_M              0x0000ffff

/* TX FIFO Status Register */

#define TXFIR_TXSFUS_M              0x00ff0000
#define TXFIR_TXSFUS_S              16
#define TXFIR_TXDFFS_M              0x0000ffff

/* Hardware Configuration Register */

#define HWCFGR_READY                0x08000000
#define HWCFGR_AMDIXENSS            0x02000000
#define HWCFGR_MBO                  0x00100000
#define HWCFGR_TXFS_M               0x000f0000
#define HWCFGR_TXFS_S               16

/* Byte Order Test Register */

#define BOTR_MASK                   0xffffffff
#define BOTR_VAL                    0x87654321

/* Host MAC Automatic Flow Control Configuration Register */

#define HMAFCCFGR_AFCHL_M           0x00ff0000
#define HMAFCCFGR_AFCHL_S           16
#define HMAFCCFGR_AFCLL_M           0x0000ff00
#define HMAFCCFGR_AFCLL_S           8
#define HMAFCCFGR_BPD_M             0x000000f0
#define HMAFCCFGR_BPD_S             4
#define HMAFCCFGR_FCOMF             0x00000008
#define HMAFCCFGR_FCOBF             0x00000004
#define HMAFCCFGR_FCOAD             0x00000002
#define HMAFCCFGR_FCOAF             0x00000001

/* Chip ID and Revision Register */

#define CIARR_CID_M                 0xffff0000
#define CIARR_CID_S                 16
#define CIARR_CID_V                 (0x9250 << 16)
#define CIARR_CREV_M                0x0000ffff

/* Interrupt Configuration Register */

#define ICFGR_IDAI_M                0xf0000000
#define ICFGR_IDAI_S                24
#define ICFGR_IDAIC                 0x00004000
#define ICFGR_IDAS                  0x00002000
#define ICFGR_MI                    0x00001000
#define ICFGR_IRQE                  0x00000100
#define ICFGR_IRQP                  0x00000010
#define ICFGR_IRQCLKS               0x00000002
#define ICFGR_IRQBT                 0x00000001

/* Interrupt Enable Register */

#define IER_SW                      0x80000000
#define IER_RAEDY                   0x40000000
#define IER_1588                    0x20000000
#define IER_PHY                     0x04000000
#define IER_TXSTOP                  0x02000000
#define IER_RXSTOP                  0x01000000
#define IER_RXDFH                   0x00800000
#define IER_TXIOC                   0x00200000
#define IER_RXD                     0x00100000
#define IER_GPT                     0x00080000
#define IER_PME                     0x00020000
#define IER_TXSO                    0x00010000
#define IER_RWT                     0x00008000
#define IER_RXE                     0x00004000
#define IER_TXE                     0x00002000
#define IER_GPIO                    0x00001000
#define IER_TDFO                    0x00000400
#define IER_TDFA                    0x00000200
#define IER_TSFF                    0x00000100
#define IER_TSFL                    0x00000080
#define IER_RXDF                    0x00000040
#define IER_RSFF                    0x00000010
#define IER_RSFL                    0x00000008

/* Interrupt Status Register */

#define ISR_SW                      0x80000000
#define ISR_READY                   0x40000000
#define ISR_1588                    0x20000000
#define ISR_PHY                     0x04000000
#define ISR_TXS                     0x02000000
#define ISR_RXS                     0x01000000
#define ISR_RXDFH                   0x00800000
#define ISR_TXIOC                   0x00200000
#define ISR_RXD                     0x00100000
#define ISR_GPT                     0x00080000
#define ISR_PME                     0x00020000
#define ISR_TXSO                    0x00010000
#define ISR_RWT                     0x00008000
#define ISR_RXE                     0x00004000
#define ISR_TXE                     0x00002000
#define ISR_GPIO                    0x00001000
#define ISR_TDFO                    0x00000400
#define ISR_TDFA                    0x00000200
#define ISR_TSFF                    0x00000100
#define ISR_TSFL                    0x00000080
#define ISR_RXDF                    0x00000040
#define ISR_RSFF                    0x00000010
#define ISR_RSFL                    0x00000008

/* FIFO Level Interrupt Register */

#define FLIR_FITXDAL_M              0xff000000
#define FLIR_FITXDAL_S              24
#define FLIR_FITXSL_M               0x00ff0000
#define FLIR_FITXSL_S               16
#define FLIR_FIRXSL_M               0x000000ff
#define FLIR_FIRXSL_S               0

/* Receive Configuration Register */

#define RXCFGR_RXEA                 0xc0000000
#define RXCFGR_RXDMAC               0x06000000
#define RXCFGR_RXDMAC_M             0x0fff0000
#define RXCFGR_RXDMAC_S             16
#define RXCFGR_FRXD                 0x00008000
#define RXCFGR_RXDO_M               0x00001f00
#define RXCFGR_RXDO_S               8

/* Transmit Configuration Register */

#define TXCFGR_FTXSD                0x00008000
#define TXCFGR_FTXDD                0x00004000
#define TXCFGR_TXSAO                0x00000004
#define TXCFGR_TXE                  0x00000002
#define TXCFGR_TXD                  0x00000001

/* Power Management Control Register */

#define PMCR_PMM_M                  0xe0000000
#define PMCR_PMSE                   0x10000000
#define PMCR_PMWU                   0x08000000
#define PMCR_LEDD                   0x04000000
#define PMCR_1588CLKD               0x02000000
#define PMCR_1588TSUCLKD            0x00400000
#define PMCR_HMACCLKD               0x00080000
#define PMCR_HMACSCOD               0x00040000
#define PMCR_EDS                    0x00010000
#define PMCR_EDE                    0x00004000
#define PMCR_WOE                    0x00000200
#define PMCR_PMEBT                  0x00000040
#define PMCR_WOLS                   0x00000020
#define PMCR_PMEI                   0x00000008
#define PMCR_PMEP                   0x00000004
#define PMCR_PMEE                   0x00000002
#define PMCR_READY                  0x00000001

/* Host MAC CSR Interface Command Register */

#define HMCSRICR_CSRB               0x80000000
#define HMCSRICR_RNW                0x40000000
#define HMCSRICR_CSRA_M             0x000000ff

/* Reset Control Register */

#define RSTCR_HMACR                 0x00000020
#define RSTCR_PHYR                  0x00000002
#define RSTCR_DR                    0x00000001

/* Host MAC Registers Fields ************************************************/

/* Host MAC Control Register */

#define HMACCR_RXAM                 0x80000000
#define HMACCR_EEEE                 0x02000000
#define HMACCR_DRXO                 0x00400000
#define HMACCR_LOM                  0x00200000
#define HMACCR_FDM                  0x00100000
#define HMACCR_PAM                  0x00080000
#define HMACCR_PM                   0x00040000
#define HMACCR_IF                   0x00020000
#define HMACCR_PBF                  0x00010000
#define HMACCR_HOFM                 0x00008000
#define HMACCR_HOPFM                0x00002000
#define HMACCR_DBF                  0x00000800
#define HMACCR_DR                   0x00000400
#define HMACCR_APS                  0x00000100
#define HMACCR_BOL_M                0x000000c0
#define HMACCR_DC                   0x00000020
#define HMACCR_TXE                  0x00000008
#define HMACCR_RXE                  0x00000004

/* Host MAC MII Access Register */

#define HMACMIIAR_PHYA_M            0x0000f800
#define HMACMIIAR_PHYA_S            11
#define HMACMIIAR_MIIRX_M           0x000007c0
#define HMACMIIAR_MIIRX_S           6
#define HMACMIIAR_MIIW              0x00000002
#define HMACMIIAR_MIIB              0x00000001

/* Host MAC Flow Control Register */

#define HMACFCR_PT_M                0xffff0000
#define HMACFCR_PT_S                16
#define HMACFCR_PCF                 0x00000004
#define HMACFCR_FLE                 0x00000002
#define HMACFCR_FLB                 0x00000001

/* PHY Registers Fields *****************************************************/

/* PHY Basic Control Register */

#define PHYBCR_SWR                  0x8000
#define PHYBCR_LB                   0x4000
#define PHYBCR_SSLSB                0x2000
#define PHYBCR_ANE                  0x1000
#define PHYBCR_PD                   0x0800
#define PHYBCR_RAN                  0x0200
#define PHYBCR_DM                   0x0100
#define PHYBCR_CTM                  0x0080

/* PHY Basic Status Register */

#define PHYBSR_100BT4               0x8000
#define PHYBSR_100BXFD              0x4000
#define PHYBSR_100BXHD              0x2000
#define PHYBSR_10BXFD               0x1000
#define PHYBSR_10BXHD               0x0800
#define PHYBSR_100BT2FD             0x0400
#define PHYBSR_100BT2HD             0x0200
#define PHYBSR_ES                   0x0100
#define PHYBSR_UDA                  0x0080
#define PHYBSR_MFPS                 0x0040
#define PHYBSR_ANC                  0x0020
#define PHYBSR_RF                   0x0010
#define PHYBSR_ANA                  0x0008
#define PHYBSR_LS                   0x0004
#define PHYBSR_JD                   0x0002
#define PHYBSR_EC                   0x0001

/* PHY Identification MSB Register */

#define PHYIMSBR_ID                 0xffff

/* PHY Identification LSB Register */

#define PHYILSBR_ID_M               0xfc00
#define PHYILSBR_ID_S               10
#define PHYILSBR_MN_M               0x03f0
#define PHYILSBR_MN_S               4
#define PHYILSBR_RN_M               0x000f
#define PHYILSBR_RN_S               0

/* PHY Auto-Negotiation Advertisement Register */

#define PHYANAR_NP                  0x8000
#define PHYANAR_RF                  0x2000
#define PHYANAR_ENP                 0x1000
#define PHYANAR_AP                  0x0800
#define PHYANAR_SP                  0x0400
#define PHYANAR_100BXFD             0x0100
#define PHYANAR_100BXHD             0x0080
#define PHYANAR_10BXFD              0x0040
#define PHYANAR_10BXHD              0x0020
#define PHYANAR_SF                  0x0001

/* PHY Auto-Negotiation Link Partner Base Page Ability Register */

#define PHYANLPBPAR_NP              0x8000
#define PHYANLPBPAR_ACK             0x4000
#define PHYANLPBPAR_RF              0x2000
#define PHYANLPBPAR_ENP             0x1000
#define PHYANLPBPAR_AP              0x0800
#define PHYANLPBPAR_PAUSE           0x0400
#define PHYANLPBPAR_100BT4          0x0200
#define PHYANLPBPAR_100BXFD         0x0100
#define PHYANLPBPAR_100BXHD         0x0080
#define PHYANLPBPAR_10BXFD          0x0040
#define PHYANLPBPAR_10BXHD          0x0020
#define PHYANLPBPAR_SF_M            0x001f

/* PHY Auto-Negotiation Expansion Register */

#define PHYANER_RXNPLA              0x0040
#define PHYANER_RXNPSL              0x0020
#define PHYANER_PDF                 0x0010
#define PHYANER_LPNPA               0x0008
#define PHYANER_NPA                 0x0004
#define PHYANER_PRX                 0x0002
#define PHYANER_LPANA               0x0001

/* PHY Auto-Negotiation Next Page TX Register */

#define PHYANNPTXR_NP               0x8000
#define PHYANNPTXR_MP               0x2000
#define PHYANNPTXR_ACK2             0x1000
#define PHYANNPTXR_TOG              0x0800
#define PHYANNPTXR_MC_M             0x07ff

/* PHY Auto-Negotiation Next Page RX Register */

#define PHYANNPRXR_NP               0x8000
#define PHYANNPRXR_ACK              0x4000
#define PHYANNPRXR_MP               0x2000
#define PHYANNPRXR_ACK2             0x1000
#define PHYANNPRXR_TOG              0x0800
#define PHYANNPRXR_MC_M             0x07ff

/* PHY MMD Access Control Register */

#define PHYMMDACR_MMDF_M            0xc000
#define PHYMMDACR_MMDDA_M           0x001f

/* PHY EDPD NLP/Crossover Time/EEE Configuration Register */

#define PHYEDPDCFGR_EDPDTXNLPE      0x8000
#define PHYEDPDCFGR_EDPDTXNLPITS_M  0x6000
#define PHYEDPDCFGR_EDPDRXSNLPWE    0x1000
#define PHYEDPDCFGR_EDPDRXNLPMIDS_M 0x0c00
#define PHYEDPDCFGR_PEEEE           0x0004
#define PHYEDPDCFGR_EDPDEC          0x0002
#define PHYEDPDCFGR_EMAMDIXCT       0x0001

/* PHY Mode Control/Status Register */

#define PHYMCOSR_EDPD               0x2000
#define PHYMCOSR_AIM                0x0040
#define PHYMCOSR_EO                 0x0002

/* PHY Special Modes Register */

#define PHYSMR_100BFXM              0x0400
#define PHYSMR_PM_M                 0x00e0
#define PHYSMR_PM_S                 5
#define PHYSMR_ADDR_M               0x001f
#define PHYSMR_ADDR_S               0x00e0

/* PHY TDR Patterns/Delay Control Register */

#define PHYTDRCOSR_TDRDI            0x8000
#define PHYTDRCOSR_TDRLBC_M         0x7000
#define PHYTDRCOSR_TDRPH            0x0fc0
#define PHYTDRCOSR_TDRPL            0x003f

/* PHY TDR Control/Status Register */

#define PHYTDRCOSR_TDRE             0x8000
#define PHYTDRCOSR_TDRATDFE         0x4000
#define PHYTDRCOSR_TDRCCT           0x0600
#define PHYTDRCOSR_TDRCS            0x0100
#define PHYTDRCOSR_TDRCL            0x00FF

/*  PHY Special Control/Status Indication register */

#define PHYSCOSIR_AMDIXC            0x8000
#define PHYSCOSIR_AMDIXE            0x4000
#define PHYSCOSIR_DMDIXS            0x2000
#define PHYSCOSIR_SQETD             0x0800
#define PHYSCOSIR_FEFIE             0x0020
#define PHYSCOSIR_10BTPS            0x0010

/* PHY Cable Length Register */

#define PHYCLR_CL_M                 0xf000

/* PHY Interrupt Source Flags Register */

#define PHYISFR_LU                  0x0200
#define PHYISFR_EO                  0x0080
#define PHYISFR_ANC                 0x0040
#define PHYISFR_RF                  0x0020
#define PHYISFR_LD                  0x0010
#define PHYISFR_ANLPA               0x0008
#define PHYISFR_PDF                 0x0004
#define PHYISFR_ANPR                0x0002

/* PHY Interrupt Enable Register */

#define PHYIER_LU                   0x0200
#define PHYIER_EO                   0x0080
#define PHYIER_ANC                  0x0040
#define PHYIER_RF                   0x0020
#define PHYIER_LD                   0x0010
#define PHYIER_ANLPA                0x0008
#define PHYIER_PDF                  0x0004
#define PHYIER_ANPR                 0x0002

/* PHY Special Control/Status Register */

#define PHYSCOSR_AD                 0x8000
#define PHYSCOSR_RD                 0x0040
#define PHYSCOSR_SI_M               0x001c

#endif /* __DRIVERS_NET_LAN9250_H */
