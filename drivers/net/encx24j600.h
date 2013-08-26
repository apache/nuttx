/****************************************************************************
 * drivers/net/encx24j600.h
 *
 *   Copyright (C) 2013 UVC Ingenieure. All rights reserved.
 *   Author: Max Holtberg <mh@uvc.de>
 *
 * References:
 * - ENC424J600/624J600 Data Sheet, Stand-Alone 10/100 Ethernet Controller
 *   with SPI or Parallel Interface, DS39935C, 2010 Microchip Technology Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __DRIVERS_NET_ENCX24J600_H
#define __DRIVERS_NET_ENCX24J600_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ENCX24J600 Commands ********************************************************/

/* The SPI opcodes are divided into four families:
 *
 * Single Byte: Direct opcode instructions; designed for task-oriented SFR
 * operations with no data returned
 *
 * Two-Byte: Direct opcode instruction; designed for SFR operation with byte
 * data returned
 *
 * Three-Byte: Opcode with word length argument; includes read and write
 * operations, designed for pointer manipulation with word length data returned
 *
 * N-Byte: Opcode with one or more bytes of argument; includes read and write
 * operations designed for general memory space access with one or more bytes of
 * data returned
 */

/* Single-Byte Instructions */

/* Because all single byte instructions are fixed length with no optional
 * parameters, it is possible to execute any instruction immediately following
 * the execution of any single byte instruction without deasserting the chip
 * select line in between.
 */

#define ENC_B0SEL       (0xc0)  /* Selects SFR Bank 0 */
#define ENC_B1SEL       (0xc2)  /* Selects SFR Bank 1 */
#define ENC_B2SEL       (0xc4)  /* Selects SFR Bank 2 */
#define ENC_B3SEL       (0xc6)  /* Selects SFR Bank 3 */
#define ENC_SETETHRST   (0xca)  /* Issues System Reset by setting ETHRST (ECON2<4>) */
#define ENC_FCDISABLE   (0xe0)  /* Disables flow control (sets ECON1<7:6> = 00) */
#define ENC_FCSINGLE    (0xe2)  /* Transmits a single pause frame (sets ECON1<7:6> = 01) */
#define ENC_FCMULTIPLE  (0xe4)  /* Enables flow control with periodic pause frames (sets ECON1<7:6> = 10) */
#define ENC_FCCLEAR     (0xe6)  /* Terminates flow control with a final pause frame (sets ECON1<7:6> = 11) */
#define ENC_SETPKTDEC   (0xcc)  /* Decrements PKTCNT by setting PKTDEC (ECON1<8>) */
#define ENC_DMASTOP     (0xd2)  /* Stops current DMA operation by clearing DMAST (ECON1<5>) */
#define ENC_DMACKSUM    (0xd8)  /* Starts DMA and checksum operation (sets ECON1<5:2> = 1000) */
#define ENC_DMACKSUMS   (0xda)  /* Starts DMA checksum operation with seed (sets ECON1<5:2> = 1010) */
#define ENC_DMACOPY     (0xdc)  /* Starts DMA copy and checksum operation (sets ECON1<5:2> = 1100) */
#define ENC_DMACOPYS    (0xde)  /* Starts DMA copy and checksum operation with seed (sets ECON1<5:2> = 1110) */
#define ENC_SETTXRTS    (0xd4)  /* Sets TXRTS (ECON1<1>), sends an Ethernet packet */
#define ENC_ENABLERX    (0xe8)  /* Enables packet reception by setting RXEN (ECON1<0>) */
#define ENC_DISABLERX   (0xea)  /* Disables packet reception by clearing RXEN (ECON1<0>) */
#define ENC_SETEIE      (0xec)  /* Enable Ethernet Interrupts by setting INT (ESTAT<15>) */
#define ENC_CLREIE      (0xee)  /* Disable Ethernet Interrupts by clearing INT (ESTAT<15>) */

/* Two-Byte Instructions */

/* There is only one instruction in the ENCX24J600 command set which uses two
 * SPI bytes. The Read Bank Select opcode, RBSEL, reads the internal SFR bank
 * select state and returns the value to the host controller.
 */

#define ENC_RBSEL       (0xc8)

/* Three-Byte Instructions */

#define ENC_WGPRDPT     (0x60)  /* Write General Purpose Buffer Read Pointer (EGPRDPT) */
#define ENC_RGPRDPT     (0x62)  /* Read General Purpose Buffer Read Pointer (EGPRDPT) */
#define ENC_WRXRDPT     (0x64)  /* Write Receive Buffer Read Pointer (ERXRDPT) */
#define ENC_RRXRDPT     (0x66)  /* Read Receive Buffer Read Pointer (ERXRDPT) */
#define ENC_WUDARDPT    (0x68)  /* Write User-Defined Area Read Pointer (EUDARDPT) */
#define ENC_RUDARDPT    (0x6a)  /* Read User-Defined Area Read Pointer (EUDARDPT) */
#define ENC_WGPWRPT     (0x6c)  /* Write General Purpose Buffer Write Pointer (EGPWRPT) */
#define ENC_RGPWRPT     (0x6e)  /* Read General Purpose Buffer Write Pointer (EGPWRPT) */
#define ENC_WRXWRPT     (0x70)  /* Write Receive Buffer Write Pointer (ERXWRPT) */
#define ENC_RRXWRPT     (0x72)  /* Read Receive Buffer Write Pointer (ERXWRPT) */
#define ENC_WUDAWRPT    (0x78)  /* Write User-Defined Area Write Pointer (EUDAWRPT) */
#define ENC_RUDAWRPT    (0x76)  /* Read User-Defined Area Write Pointer (EUDAWRPT) */

/* Banked N-Byte Instructions */

#define ENC_RCR         (0x00)  /* Read Control Register
                                   000 | aaaaa | (Register value returned)) */
#define ENC_WCR         (0x40)  /* Write Control Register
                                   010 | aaaaa | dddddddd */
#define ENC_BFS         (0x80)  /* Bit Field Set
                                   100 | aaaaa | dddddddd */
#define ENC_BFC         (0xa0)  /* Bit Field Clear
                                   101 | aaaaa | dddddddd */

/* Unbanked N-Byte Instructions */

#define ENC_RCRU        (0x20)  /* Read Control Register(s), Unbanked */
#define ENC_WCRU        (0x22)  /* Write Control Register(s), Unbanked */
#define ENC_BFSU        (0x24)  /* Bit Field(s) Set, Unbanked */
#define ENC_BFCU        (0x26)  /* Bit Field(s) Clear, Unbanked */

/* SRAM Access Instructions */

#define ENC_RGPDATA     (0x28)  /* Read Data from EGPDATA */
#define ENC_WGPDATA     (0x2a)  /* Write Data from EGPDATA */
#define ENC_RRXDATA     (0x2c)  /* Read Data from ERXDATA */
#define ENC_WRXDATA     (0x2e)  /* Write Data from ERXDATA */
#define ENC_RUDADATA    (0x30)  /* Read Data from EUDADATA */
#define ENC_WUDADATA    (0x32)  /* Write Data from EUDADATA */

/* Banked Control Registers *************************************************/
/* Registers are described by 16 bit values. The high byte describes the bank
 * by the appropiate bank selection command.
 * For registers which are available on all banks the comnmand is set to 0.
 * Unbanked registers are identified by 0x01.
 */

#define ENC_ADDR_SHIFT  (0)
#define ENC_ADDR_MASK   (0xff << ENC_ADDR_SHIFT)
#define ENC_BANK_SHIFT  (8)
#define ENC_BANK_MASK   (0xff << ENC_BANK_SHIFT)

#define REGADDR(a,b)    ((b) << ENC_BANK_SHIFT | (a) << ENC_ADDR_SHIFT)
#define GETADDR(a)      (((a) & ENC_ADDR_MASK) >> ENC_ADDR_SHIFT)
#define GETBANK(a)      (((a) & ENC_BANK_MASK) >> ENC_BANK_SHIFT)

/* Bank 0 Control Register Addresses */

#define ENC_ETXST       REGADDR(0x00, ENC_B0SEL)
#define ENC_ETXLEN      REGADDR(0x02, ENC_B0SEL)
#define ENC_ERXST       REGADDR(0x04, ENC_B0SEL)
#define ENC_ERXTAIL     REGADDR(0x06, ENC_B0SEL)
#define ENC_ERXHEAD     REGADDR(0x08, ENC_B0SEL)
#define ENC_EDMAST      REGADDR(0x0a, ENC_B0SEL)
#define ENC_EDMALEN     REGADDR(0x0c, ENC_B0SEL)
#define ENC_EDMADST     REGADDR(0x0e, ENC_B0SEL)
#define ENC_EDMACS      REGADDR(0x10, ENC_B0SEL)
#define ENC_ETXSTAT     REGADDR(0x12, ENC_B0SEL)
#define ENC_ETXWIRE     REGADDR(0x14, ENC_B0SEL)

/* Bank 1 Contro Register Addresses */

#define ENC_EHT1        REGADDR(0x00, ENC_B1SEL)
#define ENC_EHT2        REGADDR(0x02, ENC_B1SEL)
#define ENC_EHT3        REGADDR(0x04, ENC_B1SEL)
#define ENC_EHT4        REGADDR(0x06, ENC_B1SEL)
#define ENC_EPMM1       REGADDR(0x08, ENC_B1SEL)
#define ENC_EPMM2       REGADDR(0x0a, ENC_B1SEL)
#define ENC_EPMM3       REGADDR(0x0c, ENC_B1SEL)
#define ENC_EPMM4       REGADDR(0x0e, ENC_B1SEL)
#define ENC_EPMCS       REGADDR(0x10, ENC_B1SEL)
#define ENC_EPMO        REGADDR(0x12, ENC_B1SEL)
#define ENC_ERXFCON     REGADDR(0x14, ENC_B1SEL)

/* Bank 2 Control Register Addresses */

#define ENC_MACON1      REGADDR(0x00, ENC_B2SEL)
#define ENC_MACON2      REGADDR(0x02, ENC_B2SEL)
#define ENC_MABBIPG     REGADDR(0x04, ENC_B2SEL)
#define ENC_MAIPG       REGADDR(0x06, ENC_B2SEL)
#define ENC_MACLCON     REGADDR(0x08, ENC_B2SEL)
#define ENC_MAMXFL      REGADDR(0x0a, ENC_B2SEL)
/* 0x0c - 0x11 reserved */
#define ENC_MICMD       REGADDR(0x12, ENC_B2SEL)
#define ENC_MIREGADR    REGADDR(0x14, ENC_B2SEL)

/* MAC Control Register 1 Bit Definitions */

#define MACON1_PASSALL  (1 << 1)
#define MACON1_RXPAUS   (1 << 2)
#define MACON1_LOOPBK   (1 << 4)

/* MAC Control Register 2 Bit Definitions */

#define MACON2_FULDPX   (1 << 0)    /* MAC Full-Duplex Enable bit */
#define MACON2_HFRMEN   (1 << 2)    /* Huge Frame Enable bit */
#define MACON2_PHDREN   (1 << 3)    /* Proprietary Header Enable bit */
#define MACON2_TXCRCEN  (1 << 4)    /* Transmit CRC Enable bit */
#define MACON2_PADCFG0  (1 << 5)    /*  Automatic Pad and CRC Configuration bits */
#define MACON2_PADCFG1  (1 << 6)
#define MACON2_PADCFG2  (1 << 7)
#define MACON2_NOBKOFF  (1 << 12)   /* No Backoff Enable bit (applies to half duplex only) */
#define MACON2_BPEN     (1 << 13)   /* No Backoff During Back Pressure Enable bit (applies to half duplex only) */
#define MACON2_DEFER    (1 << 14)   /* Defer Transmission Enable bit (applies to half duplex only) */

/* MII Management Command Register Bit Definitions */

#define MICMD_MIIRD     (1 << 0)    /* MII Read Enable bit */
#define MICMD_MIISCAN   (1 << 1)    /* MII Scan Enable bit */

/* MII Management Status Register Bit Definitions */

#define MISTAT_BUSY     (1 << 0)    /* MII Management Busy Status bit */
#define MISTAT_SCAN     (1 << 1)    /* MII Management Scan Status bit */
#define MISTAT_NVALID   (1 << 2)    /* MII Management Read Data Not Valid Status bit */

/* Bank 3 Control Register Addresses */

#define ENC_MAADR3      REGADDR(0x00, ENC_B3SEL)
#define ENC_MAADR2      REGADDR(0x02, ENC_B3SEL)
#define ENC_MAADR1      REGADDR(0x04, ENC_B3SEL)
#define ENC_MIWR        REGADDR(0x06, ENC_B3SEL)
#define ENC_MIRD        REGADDR(0x08, ENC_B3SEL)
#define ENC_MISTAT      REGADDR(0x0a, ENC_B3SEL)
#define ENC_EPAUS       REGADDR(0x0c, ENC_B3SEL)
#define ENC_ECON2       REGADDR(0x0e, ENC_B3SEL)
#define ENC_ERXWM       REGADDR(0x10, ENC_B3SEL)
#define ENC_EIE         REGADDR(0x12, ENC_B3SEL)
#define ENC_EIDLED      REGADDR(0x14, ENC_B3SEL)

/* Ethernet Control Register Bit Definitions */

#define ECON2_AESLEN0        (1 << 0)   /* AES Key Length Control bits */
#define ECON2_AESLEN1        (1 << 1)   /* Modular Exponentiation Length Control bits */
#define ECON2_MODLEN0        (1 << 2)
#define ECON2_MODLEN1        (1 << 3)
#define ECON2_ETHRST         (1 << 4)   /* Master Ethernet Reset bit */
#define ECON2_RXRST          (1 << 5)   /* Receive Logic Reset bit */
#define ECON2_TXRST          (1 << 6)   /* Transmit Logic Reset bit */
#define ECON2_AUTOFC         (1 << 7)   /* Automatic Flow Control Enable bit */
#define ECON2_COCON_SHIFT    (8)        /* CLKOUT Frequency Control bits */
#define ECON2_COCON_MASK     (0x0f << ECON2_COCON_SHIFT)
#define ECON2_SHA1MD5        (1 << 12)  /* SHA-1/MD5 Hash Control bit */
#define ECON2_TXMAC          (1 << 13)  /* Automatically Transmit MAC Address Enable bit */
#define ECON2_STRCH          (1 << 14)  /* LED Stretching Enable bit */
#define ECON2_ETHEN          (1 << 15)  /* Ethernet Enable bit */

/* Ethernet Interrupt Enable Register Bit Definitions */

#define EIE_PCFULIE     (1 << 0)   /* Packet Counter Full Interrupt Enable bit */
#define EIE_RXABTIE     (1 << 1)   /* Receive Abort Interrupt Enable bit */
#define EIE_TXABTIE     (1 << 2)   /* Transmit Abort Interrupt Enable bit */
#define EIE_TXIE        (1 << 3)   /* Transmit Done Interrupt Enable bit */
#define EIE_DMAIE       (1 << 5)   /* DMA Interrupt Enable bit */
#define EIE_PKTIE       (1 << 6)   /* RX Packet Pending Interrupt Enable bit */
#define EIE_LINKIE      (1 << 11)  /* PHY Link Status Change Interrupt Enable bit */
#define EIE_AESIE       (1 << 12)  /* AES Encrypt/Decrypt Interrupt Enable bit */
#define EIE_HASHIE      (1 << 13)  /* MD5/SHA-1 Hash Interrupt Enable bit */
#define EIE_MODEXIE     (1 << 14)  /* Modular Exponentiation Interrupt Enable bit */
#define EIE_INTIE       (1 << 15)  /* INT Global Interrupt Enable bit */

/**
 * The last 10 bytes (16h to 1Fh) of all SPI banks point to a common set of five
 * registers: EUDAST, EUDAND, ESTAT, EIR and ECON1. These are key registers used
 * in controlling and monitoring the operation of the device. Their common
 * banked addresses allow easy access without switching the bank.
 */

/* Common Register Addresses */

#define ENC_EUDAST      REGADDR(0x16, 0x00) /* User-Defined Area Start Pointer (EUDAST<7:0>) */
#define ENC_EUDAND      REGADDR(0x18, 0x00) /* User-Defined Area End Pointer (EUDAND<7:0>) */
#define ENC_ESTAT       REGADDR(0x1a, 0x00)
#define ENC_EIR         REGADDR(0x1c, 0x00)
#define ENC_ECON1       REGADDR(0x1e, 0x00)

/* Ethernet Status Register Bit Definitions */

#define ESTAT_PKTCNT_SHIFT  (0)           /* Receive Packet Count bits */
#define ESTAT_PKTCNT_MASK   (0xff)
#define ESTAT_PHYLNK        (1 << 8)      /* PHY Linked Status bit */
#define ESTAT_PHYDPX        (1 << 10)     /* PHY Full Duplex Status bit */
#define ESTAT_CLKRDY        (1 << 12)     /* Clock Ready Status bit */
#define ESTAT_RXBUSY        (1 << 13)     /* Receive Logic Active Status bit */
#define ESTAT_FCIDLE        (1 << 14)     /* Flow Control Idle Status bit */
#define ESTAT_INT           (1 << 15)     /* Interrupt Pending Status bit */

/* Ethernet Interrupt Flag Register Bit Definitions */

#define EIR_PCFULIF     (1 << 0)   /* Packet Counter Full Interrupt Flag bit */
#define EIR_RXABTIF     (1 << 1)   /* Receive Abort Interrupt Flag bit */
#define EIR_TXABTIF     (1 << 2)   /* Transmit Abort Interrupt Flag bit */
#define EIR_TXIF        (1 << 3)   /* Transmit Done Interrupt Flag bit */
#define EIR_DMAIF       (1 << 5)   /* DMA Interrupt Flag bit */
#define EIR_PKTIF       (1 << 6)   /* RX Packet Pending Interrupt Flag bit */
#define EIR_LINKIF      (1 << 11)  /* PHY Link Status Change Interrupt Flag bit */
#define EIR_AESIF       (1 << 12)  /* AES Encrypt/Decrypt Interrupt Flag bit */
#define EIR_HASHIF      (1 << 13)  /* MD5/SHA-1 Hash Interrupt Flag bit */
#define EIR_MODEXIF     (1 << 14)  /* Modular Exponentiation Interrupt Flag bit */
#define EIR_CRYPTEN     (1 << 15)  /* Modular Exponentiation and AES Cryptographic Modules Enable bit */
#define EIR_ALLINTS     (0xf86f)

/* Ethernet Control Register 1 Bit Definitions */

#define ECON1_RXEN      (1 << 0)   /* Receive Enable bit */
#define ECON1_TXRTS     (1 << 1)   /* Transmit Request to Send Status/Control bit */
#define ECON1_DMANOCS   (1 << 2)   /* DMA No Checksum Control bit */
#define ECON1_DMACSSD   (1 << 3)   /* DMA Checksum Seed Control bit */
#define ECON1_DMACPY    (1 << 4)   /* DMA Copy Control bit */
#define ECON1_DMAST     (1 << 5)   /* DMA Start bit */
#define ECON1_FCOP0     (1 << 6)   /* Flow Control Operation Control/Status bits */
#define ECON1_FCOP1     (1 << 7)   /* Flow Control Operation Control/Status bits */
#define ECON1_PKTDEC    (1 << 8)   /* RX Packet Counter Decrement Control bit */
#define ECON1_AESOP0    (1 << 9)   /* AES Operation Control bits */
#define ECON1_AESOP1    (1 << 10)  /* AES Operation Control bits */
#define ECON1_AESST     (1 << 11)  /* AES Encrypt/Decrypt Start bit */
#define ECON1_HASHLST   (1 << 12)  /* MD5/SHA-1 Hash Last Block Control bit */
#define ECON1_HASHOP    (1 << 13)  /* MD5/SHA-1 Hash Operation Control bit */
#define ECON1_HASHEN    (1 << 14)  /* MD5/SHA-1 Hash Enable bit */
#define ECON1_MODEXST   (1 << 15)  /* Modular Exponentiation Start bit */

/* Unbanked Register Addresses */

#if 0
/* Disabled to prevent accidental use. All unbanked operations are implemented
 * using the specific manipulation commands.
 */
#define ENC_EGPDATA     0x80
#define ENC_ERXDATA     0x82
#define ENC_EUDADATA    0x84
#define ENC_EGPRDPT     0x86
#define ENC_EGPWRPT     0x88
#define ENC_ERXRDPT     0x8a
#define ENC_ERXWRPT     0x8c
#define ENC_EUDARDPT    0x8e
#define ENC_EUDAWRPT    0x90
#endif

/* PHY Registers ************************************************************/

#define ENC_PHCON1      0x00
#define ENC_PHSTAT1     0x01
#define ENC_PHANA       0x04
#define ENC_PHANLPA     0x05
#define ENC_PHANE       0x06
#define ENC_PHCON2      0x11
#define ENC_PHSTAT2     0x1b
#define ENC_PHSTAT3     0x1f

/* PHY Control Register 1 Bit Definitions */

#define PHCON1_PFULDPX  (1 << 8)  /* PHY Duplex Select Control bit */
#define PHCON1_RENEG    (1 << 9)  /* Restart Auto-Negotiation Control bit */
#define PHCON1_PSLEEP   (1 << 11) /* PHY Sleep Enable bit */
#define PHCON1_ANEN     (1 << 12) /* PHY Auto-Negotiation Enable bit */
#define PHCON1_SPD100   (1 << 13) /* PHY Speed Select Control bit */
#define PHCON1_PLOOPBK  (1 << 14) /* PHY Loopback Enable bit */
#define PHCON1_PRST     (1 << 15) /* PHY Reset bit */

/* PHY Status Register 1 Bit Definitions */

#define PHSTAT1_EXTREGS (1 << 0)  /* Extended Capabilities Registers Present Status bit */
#define PHSTAT1_LLSTAT  (1 << 2)  /* Latching Link Status bit */
#define PHSTAT1_ANABLE  (1 << 3)  /* Auto-Negotiation Ability Status bit */
#define PHSTAT1_LRFAULT (1 << 4)  /* Latching Remote Fault Condition Status bit */
#define PHSTAT1_ANDONE  (1 << 5)  /* Auto-Negotiation Done Status bit */
#define PHSTAT1_HALF10  (1 << 11) /* 10Base-T Half-Duplex Ability Status bit */
#define PHSTAT1_FULL10  (1 << 12) /* 10Base-T Full-Duplex Ability Status bit */
#define PHSTAT1_HALF100 (1 << 13) /* 100Base-TX Half-Duplex Ability Status bit */
#define PHSTAT1_FULL100 (1 << 13) /* 100Base-TX Full-Duplex Ability Status bit */

/* PHY Auto-Negotiation Advertisement Register Bit Definitions */


#define PHANA_ADIEEE0   (1 << 0)
#define PHANA_ADIEEE1   (1 << 1)
#define PHANA_ADIEEE2   (1 << 2)
#define PHANA_ADIEEE3   (1 << 3)
#define PHANA_ADIEEE4   (1 << 4)
#define PHANA_AD10      (1 << 5)  /* Advertise 10Base-T Half-Duplex Ability bit */
#define PHANA_AD10FD    (1 << 6)  /* Advertise 10Base-T Full-Duplex Ability bit */
#define PHANA_AD100     (1 << 7)  /* Advertise 100Base-TX Half-Duplex Ability bit */
#define PHANA_AD100FD   (1 << 8)  /* Advertise 100Base-TX Full-Duplex Ability bit */
/* Advertise PAUSE Flow Control Ability bits */
/* 11 = Local device supports both symmetric PAUSE and asymmetric PAUSE toward local device */
/* 10 = Local device supports asymmetric PAUSE toward link partner only */
/* 01 = Local device supports symmetric PAUSE only (Normal Flow Control mode) */
/* 00 = Local device does not support PAUSE flow control */
#define PHANA_ADPAUS0   (1 << 10)
#define PHANA_ADPAUS1   (1 << 11)
#define PHANA_ADFAULT   (1 << 13) /* Advertise Remote Fault Condition bit */
#define PHANA_ADNP      (1 << 15) /* Advertise Next Page Ability bit */

/* Packet Memory ************************************************************/

/* 24-Kbyte Transmit/Receive Packet Dual Port SRAM */

#define PKTMEM_START    0x0000
#define PKTMEM_SIZE     0x6000

/* RX Status Bit Definitions ************************************************/

#define RXSTAT_OK       (1 << 7)

#endif  /* __DRIVERS_NET_ENCX24J600_H */
