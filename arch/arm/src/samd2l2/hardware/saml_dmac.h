/****************************************************************************
 * arch/arm/src/samd2l2/hardware/saml_dmac.h
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

/* References:
 *   "Atmel SAM L21E / SAM L21G / SAM L21J Smart ARM-Based Microcontroller
 *   Datasheet", Atmel-42385C-SAML21_Datasheet_Preliminary-03/20/15
 */

#ifndef __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAML_DMAC_H
#define __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAML_DMAC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#ifdef CONFIG_ARCH_FAMILY_SAML21

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DMAC register offsets ****************************************************/

#define SAM_DMAC_CTRL_OFFSET             0x0000  /* Control Register */
#define SAM_DMAC_CRCCTRL_OFFSET          0x0002  /* CRC Control Register */
#define SAM_DMAC_CRCDATAIN_OFFSET        0x0004  /* CRC Data Input Register */
#define SAM_DMAC_CRCCHKSUM_OFFSET        0x0008  /* CRC Checksum Register */
#define SAM_DMAC_CRCSTATUS_OFFSET        0x000c  /* CRC Status Register */
#define SAM_DMAC_DBGCTRL_OFFSET          0x000d  /* Debug Control Register */
#define SAM_DMAC_QOSCTRL_OFFSET          0x000e  /* Quality of Service Control Register */
#define SAM_DMAC_SWTRIGCTRL_OFFSET       0x0010  /* Software Trigger Control Register */
#define SAM_DMAC_PRICTRL0_OFFSET         0x0014  /* Priority Control 0 Register */
#define SAM_DMAC_INTPEND_OFFSET          0x0020  /* Interrupt Pending Register */
#define SAM_DMAC_INTSTATUS_OFFSET        0x0024  /* Interrupt Status Register */
#define SAM_DMAC_BUSYCH_OFFSET           0x0028  /* Busy Channels Register */
#define SAM_DMAC_PENDCH_OFFSET           0x002c  /* Pending Channels Register */
#define SAM_DMAC_ACTIVE_OFFSET           0x0030  /* Active Channels and Levels Register */
#define SAM_DMAC_BASEADDR_OFFSET         0x0034  /* Descriptor Memory Section Base Address Register */
#define SAM_DMAC_WRBADDR_OFFSET          0x0038  /* Write-Back Memory Section Base Address Register */
#define SAM_DMAC_CHID_OFFSET             0x003f  /* Channel ID Register */
#define SAM_DMAC_CHCTRLA_OFFSET          0x0040  /* Channel Control A Register */
#define SAM_DMAC_CHCTRLB_OFFSET          0x0044  /* Channel Control B Register */
#define SAM_DMAC_CHINTENCLR_OFFSET       0x004c  /* Channel Interrupt Enable Clear Register */
#define SAM_DMAC_CHINTENSET_OFFSET       0x004d  /* Channel Interrupt Enable Set Register */
#define SAM_DMAC_CHINTFLAG_OFFSET        0x004e  /* Channel Interrupt Flag Status and Clear Register */
#define SAM_DMAC_CHSTATUS_OFFSET         0x004f  /* Channel Status Register */

/* LPSRAM Registers Relative to BASEADDR or WRBADDR */

#define SAM_LPSRAM_BTCTRL_OFFSET         0x0000  /* Block Transfer Control Register */
#define SAM_LPSRAM_BTCNT_OFFSET          0x0002  /* Block Transfer Count Register */
#define SAM_LPSRAM_SRCADDR_OFFSET        0x0004  /* Block Transfer Source Address Register */
#define SAM_LPSRAM_DSTADDR_OFFSET        0x0008  /* Block Transfer Destination Address Register */
#define SAM_LPSRAM_DESCADDR_OFFSET       0x000c  /* Next Address Descriptor Register */

/* DMAC register addresses **************************************************/

#define SAM_DMAC_CTRL                  (SAM_DMAC_BASE+SAM_DMAC_CTRL_OFFSET)
#define SAM_DMAC_CRCCTRL               (SAM_DMAC_BASE+SAM_DMAC_CRCCTRL_OFFSET)
#define SAM_DMAC_CRCDATAIN             (SAM_DMAC_BASE+SAM_DMAC_CRCDATAIN_OFFSET)
#define SAM_DMAC_CRCCHKSUM             (SAM_DMAC_BASE+SAM_DMAC_CRCCHKSUM_OFFSET)
#define SAM_DMAC_CRCSTATUS             (SAM_DMAC_BASE+SAM_DMAC_CRCSTATUS_OFFSET)
#define SAM_DMAC_DBGCTRL               (SAM_DMAC_BASE+SAM_DMAC_DBGCTRL_OFFSET)
#define SAM_DMAC_QOSCTRL               (SAM_DMAC_BASE+SAM_DMAC_QOSCTRL_OFFSET)
#define SAM_DMAC_SWTRIGCTRL            (SAM_DMAC_BASE+SAM_DMAC_SWTRIGCTRL_OFFSET)
#define SAM_DMAC_PRICTRL0              (SAM_DMAC_BASE+SAM_DMAC_PRICTRL0_OFFSET)
#define SAM_DMAC_INTPEND               (SAM_DMAC_BASE+SAM_DMAC_INTPEND_OFFSET)
#define SAM_DMAC_INTSTATUS             (SAM_DMAC_BASE+SAM_DMAC_INTSTATUS_OFFSET)
#define SAM_DMAC_BUSYCH                (SAM_DMAC_BASE+SAM_DMAC_BUSYCH_OFFSET)
#define SAM_DMAC_PENDCH                (SAM_DMAC_BASE+SAM_DMAC_PENDCH_OFFSET)
#define SAM_DMAC_ACTIVE                (SAM_DMAC_BASE+SAM_DMAC_ACTIVE_OFFSET)
#define SAM_DMAC_BASEADDR              (SAM_DMAC_BASE+SAM_DMAC_BASEADDR_OFFSET)
#define SAM_DMAC_WRBADDR               (SAM_DMAC_BASE+SAM_DMAC_WRBADDR_OFFSET)
#define SAM_DMAC_CHID                  (SAM_DMAC_BASE+SAM_DMAC_CHID_OFFSET)
#define SAM_DMAC_CHCTRLA               (SAM_DMAC_BASE+SAM_DMAC_CHCTRLA_OFFSET)
#define SAM_DMAC_CHCTRLB               (SAM_DMAC_BASE+SAM_DMAC_CHCTRLB_OFFSET)
#define SAM_DMAC_CHINTENCLR            (SAM_DMAC_BASE+SAM_DMAC_CHINTENCLR_OFFSET)
#define SAM_DMAC_CHINTENSET            (SAM_DMAC_BASE+SAM_DMAC_CHINTENSET_OFFSET)
#define SAM_DMAC_CHINTFLAG             (SAM_DMAC_BASE+SAM_DMAC_CHINTFLAG_OFFSET)
#define SAM_DMAC_CHSTATUS              (SAM_DMAC_BASE+SAM_DMAC_CHSTATUS_OFFSET)

/* DMAC register bit definitions ********************************************/

/* Control Register */

#define DMAC_CTRL_SWRST                  (1 << 0)  /* Bit 0:  Software Reset */
#define DMAC_CTRL_DMAENABLE              (1 << 1)  /* Bit 1:  DMA Enable */
#define DMAC_CTRL_CRCENABLE              (1 << 2)  /* Bit 2:  CRC Enable */
#define DMAC_CTRL_LVLEN0                 (1 << 8)  /* Bit 8:  Priority level 0 Enable */
#define DMAC_CTRL_LVLEN1                 (1 << 9)  /* Bit 9:  Priority level 1 Enable */
#define DMAC_CTRL_LVLEN2                 (1 << 10) /* Bit 10: Priority level 2 Enable */

/* CRC Control Register */

#define DMAC_CRCCTRL_CRCBEATSIZE_SHIFT   (0)       /* Bits 0-1: CRC beat size */
#define DMAC_CRCCTRL_CRCBEATSIZE_MASK    (3 < DMAC_CRCCTRL_CRCBEATSIZE_SHIFT)
#  define DMAC_CRCCTRL_CRCBEATSIZE_BYTE  (0 < DMAC_CRCCTRL_CRCBEATSIZE_SHIFT) /* 8-bit bus transfer */
#  define DMAC_CRCCTRL_CRCBEATSIZE_HWORD (1 < DMAC_CRCCTRL_CRCBEATSIZE_SHIFT) /* 16-bit bus transfer */
#  define DMAC_CRCCTRL_CRCBEATSIZE_WORD  (2 < DMAC_CRCCTRL_CRCBEATSIZE_SHIFT) /* 32-bit bus transfer */

#define DMAC_CRCCTRL_CRCPOLY_SHIFT       (2)       /* Bits 2-3: CRC polynomial type */
#define DMAC_CRCCTRL_CRCPOLY_MASK        (3 < DMAC_CRCCTRL_CRCPOLY_SHIFT)
#  define DMAC_CRCCTRL_CRCPOLY_CRC16     (0 < DMAC_CRCCTRL_CRCPOLY_SHIFT) /* CRC-16 (CRC-CCITT) */
#  define DMAC_CRCCTRL_CRCPOLY_CRC32     (1 < DMAC_CRCCTRL_CRCPOLY_SHIFT) /* CRC32 (IEEE 802.3) */

#define DMAC_CRCCTRL_CRCSRC_SHIFT        (8)       /* Bits 8-13: CRC Input Source */
#define DMAC_CRCCTRL_CRCSRC_MASK         (0x3f < DMAC_CRCCTRL_CRCSRC_SHIFT)
#  define DMAC_CRCCTRL_CRCSRC_NOACTION   (0 < DMAC_CRCCTRL_CRCSRC_SHIFT) /* No action */
#  define DMAC_CRCCTRL_CRCSRC_IO         (1 < DMAC_CRCCTRL_CRCSRC_SHIFT) /* I/O interface */
#  define DMAC_CRCCTRL_CRCSRC_CHAN(n)    (((uint32_t)(n) + 0x20) < DMAC_CRCCTRL_CRCSRC_SHIFT)

/* CRC Data Input Register (32-bit value) */

/* CRC Checksum Register (32-bit value) */

/* CRC Status Register */

#define DMAC_CRCSTATUS_CRCBUSY           (1 << 0)  /* Bit 0:  CRC module busy */
#define DMAC_CRCSTATUS_CRCZERO           (1 << 1)  /* Bit 1:  CRC zero */

/* Debug Control Register */

#define DMAC_DBGCTRL_DBGRUN              (1 << 0)  /* Bit 0:  Debug run */

/* Quality of Service Control Register */

#define DMAC_QOSCTRL_WRBQOS_SHIFT        (0)       /* Bits 0-1: Write back quality of service */
#define DMAC_QOSCTRL_WRBQOS_MASK         (3 << DMAC_QOSCTRL_WRBQOS_SHIFT)
#  define DMAC_QOSCTRL_WRBQOS_DISABLE    (0 << DMAC_QOSCTRL_WRBQOS_SHIFT) /* Background */
#  define DMAC_QOSCTRL_WRBQOS_LOW        (1 << DMAC_QOSCTRL_WRBQOS_SHIFT) /* Sensitive bandwidth */
#  define DMAC_QOSCTRL_WRBQOS_MEDIUM     (2 << DMAC_QOSCTRL_WRBQOS_SHIFT) /* Sensitive latency */
#  define DMAC_QOSCTRL_WRBQOS_HIGH       (3 << DMAC_QOSCTRL_WRBQOS_SHIFT) /* Critical latency */

#define DMAC_QOSCTRL_FQOS_SHIFT          (2)       /* Bits 2-3: Fetch quality of service */
#define DMAC_QOSCTRL_FQOS_MASK           (3 << DMAC_QOSCTRL_FQOS_SHIFT)
#  define DMAC_QOSCTRL_FQOS_DISABLE      (0 << DMAC_QOSCTRL_FQOS_SHIFT) /* Background */
#  define DMAC_QOSCTRL_FQOS_LOW          (1 << DMAC_QOSCTRL_FQOS_SHIFT) /* Sensitive bandwidth */
#  define DMAC_QOSCTRL_FQOS_MEDIUM       (2 << DMAC_QOSCTRL_FQOS_SHIFT) /* Sensitive latency */
#  define DMAC_QOSCTRL_FQOS_HIGH         (3 << DMAC_QOSCTRL_FQOS_SHIFT) /* Critical latency */

#define DMAC_QOSCTRL_DQOS_SHIFT          (4)       /* Bits 4-5: Data transfer quality of service */
#define DMAC_QOSCTRL_DQOS_MASK           (3 << DMAC_QOSCTRL_DQOS_SHIFT)
#  define DMAC_QOSCTRL_DQOS_DISABLE      (0 << DMAC_QOSCTRL_DQOS_SHIFT) /* Background */
#  define DMAC_QOSCTRL_DQOS_LOW          (1 << DMAC_QOSCTRL_DQOS_SHIFT) /* Sensitive bandwidth */
#  define DMAC_QOSCTRL_DQOS_MEDIUM       (2 << DMAC_QOSCTRL_DQOS_SHIFT) /* Sensitive latency */
#  define DMAC_QOSCTRL_DQOS_HIGH         (3 << DMAC_QOSCTRL_DQOS_SHIFT) /* Critical latency */

/* Common bit definitions for: Software Trigger Control Register,
 * Interrupt Status Register, Busy Channels Register, and Pending Channels
 * Register
 */

#define DMAC_CHAN(n)                     (1 << (n)) /* DMAC Channel n, n=0-15 */

/* Priority Control 0 Register */

#define DMAC_PRICTRL0_LVLPRI0_SHIFT      (0)       /* Bits 0-3: Level 0 channel priority number */
#define DMAC_PRICTRL0_LVLPRI0_MASK       (15 << DMAC_PRICTRL0_LVLPRI0_SHIFT)
#  define DMAC_PRICTRL0_LVLPRI0(n)       ((uint32_t)(n) << DMAC_PRICTRL0_LVLPRI0_SHIFT)
#define DMAC_PRICTRL0_RRLVLEN0           (1 << 7)  /* Bit 7:  Level 0 round-robin arbitrarion enable */
#define DMAC_PRICTRL0_LVLPRI1_SHIFT      (8)       /* Bits 8-11: Level 1 channel priority number */
#define DMAC_PRICTRL0_LVLPRI1_MASK       (15 << DMAC_PRICTRL0_LVLPRI1_SHIFT)
#  define DMAC_PRICTRL0_LVLPRI1(n)       ((uint32_t)(n) << DMAC_PRICTRL0_LVLPRI1_SHIFT)
#define DMAC_PRICTRL0_RRLVLEN1           (1 << 15) /* Bit 15:  Level 1 round-robin arbitrarion enable */
#define DMAC_PRICTRL0_LVLPRI2_SHIFT      (16)      /* Bits 16-18: Level 2 channel priority number */
#define DMAC_PRICTRL0_LVLPRI2_MASK       (7 << DMAC_PRICTRL0_LVLPRI2_SHIFT)
#  define DMAC_PRICTRL0_LVLPRI2(n)       ((uint32_t)(n) << DMAC_PRICTRL0_LVLPRI2_SHIFT)
#define DMAC_PRICTRL0_RRLVLEN2           (1 << 23) /* Bit 23:  Level 2 round-robin arbitrarion enable */

/* Interrupt Pending Register */

#define DMAC_INTPEND_ID_SHIFT            (0)       /* Bit 0-3: Channel ID */
#define DMAC_INTPEND_ID_MASK             (15 << DMAC_INTPEND_ID_SHIFT)
#define DMAC_INTPEND_TERR                (1 << 8)  /* Bit 8:  Transfer error */
#define DMAC_INTPEND_TCMPL               (1 << 9)  /* Bit 9:  Transfer complete */
#define DMAC_INTPEND_SUSP                (1 << 10) /* Bit 10: Channel suspend */
#define DMAC_INTPEND_FERR                (1 << 13) /* Bit 13: Fetch error */
#define DMAC_INTPEND_BUSY                (1 << 14) /* Bit 14: Busy */
#define DMAC_INTPEND_PEND                (1 << 15) /* Bit 15: Pending */

/* Active Channels and Levels Register */

#define DMAC_ACTIVE_LVLEX0               (1 << 0)  /* Bit 0:  Level 0 channel trigger request executing */
#define DMAC_ACTIVE_LVLEX1               (1 << 1)  /* Bit 1:  Level 1 channel trigger request executing */
#define DMAC_ACTIVE_LVLEX2               (1 << 2)  /* Bit 2:  Level 2 channel trigger request executing */
#define DMAC_ACTIVE_ID_SHIFT             (8)       /* Bits 8-11: Active channel ID */
#define DMAC_ACTIVE_ID_MASK              (15 << DMAC_ACTIVE_ID_SHIFT)
#define DMAC_ACTIVE_ABUSY                (1 << 15) /* Bit 15: Active channel busy */
#define DMAC_ACTIVE_BTCNT_SHIFT          (16)      /* Bit 16-31: Active channel block transfer count */
#define DMAC_ACTIVE_BTCNT_MASK           (0xffff << DMAC_ACTIVE_BTCNT_SHIFT)

/* Descriptor Memory Section Base Address Register (32-bit address) */

/* Write-Back Memory Section Base Address Register (31-bit address) */

/* Channel ID Register */

#define DMAC_CHID_MASK                   0x0f      /* Bits 0-3: Channel ID */

/* Channel Control A Register */

#define DMAC_CHCTRLA_SWRST               (1 << 0)  /* Bit 0:  Channel software reset */
#define DMAC_CHCTRLA_ENABLE              (1 << 1)  /* Bit 1:  Channel enable */
#define DMAC_CHCTRLA_RUNSTDBY            (1 << 6)  /* Bit 6:  Channel run in standby */

/* Channel Control B Register */

#define DMAC_CHCTRLB_EVACT_SHIFT         (0)       /* Bits 0-2: Event input action */
#define DMAC_CHCTRLB_EVACT_MASK          (7 << DMAC_CHCTRLB_EVACT_SHIFT)
#  define DMAC_CHCTRLB_EVACT_NOACT       (0 << DMAC_CHCTRLB_EVACT_SHIFT) /* No action */
#  define DMAC_CHCTRLB_EVACT_TRIG        (1 << DMAC_CHCTRLB_EVACT_SHIFT) /* Normal Transfer and Conditional Transfer on Strobe
                                                                          * trigger */
#  define DMAC_CHCTRLB_EVACT_CTRIG       (2 << DMAC_CHCTRLB_EVACT_SHIFT) /* Conditional transfer trigger */
#  define DMAC_CHCTRLB_EVACT_CBLOCK      (3 << DMAC_CHCTRLB_EVACT_SHIFT) /* Conditional block transfer */
#  define DMAC_CHCTRLB_EVACT_SUSPEND     (4 << DMAC_CHCTRLB_EVACT_SHIFT) /* Channel suspend operation */
#  define DMAC_CHCTRLB_EVACT_RESUME      (5 << DMAC_CHCTRLB_EVACT_SHIFT) /* Channel resume operation */
#  define DMAC_CHCTRLB_EVACT_SSKIP       (6 << DMAC_CHCTRLB_EVACT_SHIFT) /* Skip next block suspend action */

#define DMAC_CHCTRLB_EVIE                (1 << 3)  /* Bit 3:  Channel event input enable */
#define DMAC_CHCTRLB_EVOE                (1 << 4)  /* Bit 4:  Channel event output enable */
#define DMAC_CHCTRLB_LVL_SHIFT           (5)       /* Bits 5-6: Channel arbitration level */
#define DMAC_CHCTRLB_LVL_MASK            (3 << DMAC_CHCTRLB_LVL_SHIFT)
#  define DMAC_CHCTRLB_LVL(n)            ((uint32_t)(n) << DMAC_CHCTRLB_LVL_SHIFT)
#  define DMAC_CHCTRLB_LVL_LVL0          (0 << DMAC_CHCTRLB_LVL_SHIFT) /* Channel priority level 0 */
#  define DMAC_CHCTRLB_LVL_LVL1          (1 << DMAC_CHCTRLB_LVL_SHIFT) /* Channel priority level 1 */
#  define DMAC_CHCTRLB_LVL_LVL2          (2 << DMAC_CHCTRLB_LVL_SHIFT) /* Channel priority level 2 */
#  define DMAC_CHCTRLB_LVL_LVL3          (3 << DMAC_CHCTRLB_LVL_SHIFT) /* Channel priority level 3 */

#define DMAC_CHCTRLB_TRIGSRC_SHIFT       (8)       /* Bits 8-13: Trigger source */
#define DMAC_CHCTRLB_TRIGSRC_MASK        (0x3f << DMAC_CHCTRLB_TRIGSRC_SHIFT)
#  define DMAC_CHCTRLB_TRIGSRC(n)        ((uint32_t)(n) << DMAC_CHCTRLB_TRIGSRC_SHIFT)

#define DMAC_CHCTRLB_TRIGACT_SHIFT       (22)      /* Bits 22-23: Trigger action */
#define DMAC_CHCTRLB_TRIGACT_MASK        (3 << DMAC_CHCTRLB_TRIGACT_SHIFT)
#  define DMAC_CHCTRLB_TRIGACT_BLOCK     (0 << DMAC_CHCTRLB_TRIGACT_SHIFT) /* One trigger required for each action */
#  define DMAC_CHCTRLB_TRIGACT_BEAT      (2 << DMAC_CHCTRLB_TRIGACT_SHIFT) /* One trigger required for beat transfer */
#  define DMAC_CHCTRLB_TRIGACT_TRANSACT  (3 << DMAC_CHCTRLB_TRIGACT_SHIFT) /* One trigger required for each transaction */

#define DMAC_CHCTRLB_CMD_SHIFT           (24)      /* Bits 24-25: Software command */
#define DMAC_CHCTRLB_CMD_MASK            (3 << DMAC_CHCTRLB_CMD_SHIFT)
#  define DMAC_CHCTRLB_CMD_NOACTION      (0 << DMAC_CHCTRLB_CMD_SHIFT) /* No action */
#  define DMAC_CHCTRLB_CMD_SUSPEND       (1 << DMAC_CHCTRLB_CMD_SHIFT) /* Channel suspend operation */
#  define DMAC_CHCTRLB_CMD_RESUME        (2 << DMAC_CHCTRLB_CMD_SHIFT) /* Channel resume operation */

/* Values for use with the DMAC_CHCTRLB_TRIGSRC(n) macro: */

#define DMAC_TRIGSRC_DISABLE             (0)  /* Only software/event triggers */
#define DMAC_TRIGSRC_SERCOM0_RX          (1)  /* SERCOM0 RX Trigger */
#define DMAC_TRIGSRC_SERCOM0_TX          (2)  /* SERCOM0 TX Trigger */
#define DMAC_TRIGSRC_SERCOM1_RX          (3)  /* SERCOM1 RX Trigger */
#define DMAC_TRIGSRC_SERCOM1_TX          (4)  /* SERCOM1 TX Trigger */
#define DMAC_TRIGSRC_SERCOM2_RX          (5)  /* SERCOM2 RX Trigger */
#define DMAC_TRIGSRC_SERCOM2_TX          (6)  /* SERCOM2 TX Trigger */
#define DMAC_TRIGSRC_SERCOM3_RX          (7)  /* SERCOM3 RX Trigger */
#define DMAC_TRIGSRC_SERCOM3_TX          (8)  /* SERCOM3 TX Trigger */
#define DMAC_TRIGSRC_SERCOM4_RX          (9)  /* SERCOM4 RX Trigger */
#define DMAC_TRIGSRC_SERCOM4_TX          (10) /* SERCOM4 TX Trigger */
#define DMAC_TRIGSRC_TCC0_OVF            (11) /* TCC0 Overflow Trigger */
#define DMAC_TRIGSRC_TCC0_MC0            (12) /* TCC0 Match/Compare 0 Trigger */
#define DMAC_TRIGSRC_TCC0_MC1            (13) /* TCC0 Match/Compare 1 Trigger */
#define DMAC_TRIGSRC_TCC0_MC2            (14) /* TCC0 Match/Compare 2 Trigger */
#define DMAC_TRIGSRC_TCC0_MC3            (15) /* TCC0 Match/Compare 3 Trigger */
#define DMAC_TRIGSRC_TCC1_OVF            (16) /* TCC1 Overflow Trigger */
#define DMAC_TRIGSRC_TCC1_MC0            (17) /* TCC1 Match/Compare 0 Trigger */
#define DMAC_TRIGSRC_TCC1_MC1            (18) /* TCC1 Match/Compare 1 Trigger */
#define DMAC_TRIGSRC_TCC2_OVF            (19) /* TCC2 Overflow Trigger */
#define DMAC_TRIGSRC_TCC2_MC0            (20) /* TCC2 Match/Compare 0 Trigger */
#define DMAC_TRIGSRC_TCC2_MC1            (21) /* TCC2 Match/Compare 1 Trigger */
#define DMAC_TRIGSRC_TC0_OVF             (22) /* TC0 Overflow Trigger */
#define DMAC_TRIGSRC_TC0_MC0             (23) /* TC0 Match/Compare 0 Trigger */
#define DMAC_TRIGSRC_TC0 MC1             (24) /* TC0 Match/Compare 1 Trigger */
#define DMAC_TRIGSRC_TC1_OVF             (25) /* TC1 Overflow Trigger */
#define DMAC_TRIGSRC_TC1_MC0             (26) /* TC1 Match/Compare 0 Trigger */
#define DMAC_TRIGSRC_TC1_MC1             (27) /* TC1 Match/Compare 1 Trigger */
#define DMAC_TRIGSRC_TC2_OVF             (28) /* TC2 Overflow Trigger */
#define DMAC_TRIGSRC_TC2_MC0             (29) /* TC2 Match/Compare 0 Trigger */
#define DMAC_TRIGSRC_TC2_MC1             (30) /* TC2 Match/Compare 1 Trigger */
#define DMAC_TRIGSRC_TC3_OVF             (31) /* TC3 Overflow Trigger */
#define DMAC_TRIGSRC_TC3_MC0             (32) /* TC3 Match/Compare 0 Trigger */
#define DMAC_TRIGSRC_TC3_MC1             (33) /* TC3 Match/Compare 1 Trigger */
#define DMAC_TRIGSRC_TC4_OVF             (34) /* TC4 Overflow Trigger */
#define DMAC_TRIGSRC_TC4_MC0             (35) /* TC4 Match/Compare 0 Trigger */
#define DMAC_TRIGSRC_TC4_MC1             (36) /* TC4 Match/Compare 1 Trigger */
#define DMAC_TRIGSRC_ADC_RESRDY          (37) /* ADC Result Ready Trigger */
#define DMAC_TRIGSRC_DAC0_EMPTY          (38) /* DAC0 Empty Trigger */
#define DMAC_TRIGSRC_DAC1_EMPTY          (39) /* DAC1 Empty Trigger */
#define DMAC_TRIGSRC_AES_WR              (44) /* AES Write Trigger */
#define DMAC_TRIGSRC_AES_RD              (45) /* AES Read Trigger */

/* Common register bit definitions: Channel Interrupt Enable Clear Register,
 * Channel Interrupt Enable Set Register, and  Channel Interrupt Flag
 * Status and Clear Register
 */

#define DMAC_INT_TERR                    (1 << 0)  /* Bit 0:  Transfer error interrupt */
#define DMAC_INT_TCMPL                   (1 << 1)  /* Bit 1:  Channel transfer complete interrupt */
#define DMAC_INT_SUSP                    (1 << 2)  /* Bit 2:  Channel suspend interrupt */
#define DMAC_INT_ALL                     (0x07)

/* Channel Status Register */

#define DMAC_CHSTATUS_PEND               (1 << 0)  /* Bit 0:  Channel pending */
#define DMAC_CHSTATUS_BUSY               (1 << 1)  /* Bit 1:  Channel busy */
#define DMAC_CHSTATUS_FERR               (1 << 2)  /* Bit 2:  Channel fetch error */

/* Block Transfer Control Register */

#define LPSRAM_BTCTRL_VALID              (1 << 0) /* Bit 0: Descriptor valid */
#define LPSRAM_BTCTRL_EVOSEL_SHIFT       (1)      /* Bits 1-2: Event output selection */
#define LPSRAM_BTCTRL_EVOSEL_MASK        (3 << LPSRAM_BTCTRL_EVOSEL_SHIFT)
#  define LPSRAM_BTCTRL_EVOSEL_DISABLE   (0 << LPSRAM_BTCTRL_EVOSEL_SHIFT) /* Event generation disabled */
#  define LPSRAM_BTCTRL_EVOSEL_BLOCK     (1 << LPSRAM_BTCTRL_EVOSEL_SHIFT) /* Event strobe when block transfer complete */
#  define LPSRAM_BTCTRL_EVOSEL_BEAT      (3 << LPSRAM_BTCTRL_EVOSEL_SHIFT) /* Event strobe when beat transfer complete */

#define LPSRAM_BTCTRL_BLOCKACT_SHIFT     (3)      /* Bits 3-4: Block action */
#define LPSRAM_BTCTRL_BLOCKACT_MASK      (3 << LPSRAM_BTCTRL_BLOCKACT_SHIFT)
#  define LPSRAM_BTCTRL_BLOCKACT_NOACT   (0 << LPSRAM_BTCTRL_BLOCKACT_SHIFT) /* Channel disabled if last block transfer */
#  define LPSRAM_BTCTRL_BLOCKACT_INT     (1 << LPSRAM_BTCTRL_BLOCKACT_SHIFT) /* Channel disabled if last block transfer + block int */
#  define LPSRAM_BTCTRL_BLOCKACT_SUSPEND (2 << LPSRAM_BTCTRL_BLOCKACT_SHIFT) /* Channel suspend operation is completed */
#  define LPSRAM_BTCTRL_BLOCKACT_BOTH    (3 << LPSRAM_BTCTRL_BLOCKACT_SHIFT) /* Both channel suspend operation + block int */

#define LPSRAM_BTCTRL_BEATSIZE_SHIFT     (8)      /* Bits 8-9: Beat size */
#define LPSRAM_BTCTRL_BEATSIZE_MASK      (3 << LPSRAM_BTCTRL_BEATSIZE_SHIFT)
#  define LPSRAM_BTCTRL_BEATSIZE_BYTE    (0 << LPSRAM_BTCTRL_BEATSIZE_SHIFT) /* 8-bit bus transfer */
#  define LPSRAM_BTCTRL_BEATSIZE_HWORD   (1 << LPSRAM_BTCTRL_BEATSIZE_SHIFT) /* 16-bit bus transfer */
#  define LPSRAM_BTCTRL_BEATSIZE_WORD    (2 << LPSRAM_BTCTRL_BEATSIZE_SHIFT) /* 32-bit bus transfer */

#define LPSRAM_BTCTRL_SRCINC             (1 << 10) /* Bit 10: Source address increment enable */
#define LPSRAM_BTCTRL_DSTINC             (1 << 11) /* Bit 11: Destination address increment enable */
#define LPSRAM_BTCTRL_STEPSEL            (1 << 12) /* Bit 12: Step selection */
#define LPSRAM_BTCTRL_STEPSIZE_SHIFT     (13)      /* Bits 13-15: Address increment step */
#define LPSRAM_BTCTRL_STEPSIZE_MASK      (7 << LPSRAM_BTCTRL_STEPSIZE_SHIFT)
#  define LPSRAM_BTCTRL_STEPSIZE_X1      (0 << LPSRAM_BTCTRL_STEPSIZE_SHIFT) /* Next ADDR = ADDR + (BEATSIZE+1) * 1 */
#  define LPSRAM_BTCTRL_STEPSIZE_X2      (1 << LPSRAM_BTCTRL_STEPSIZE_SHIFT) /* Next ADDR = ADDR + (BEATSIZE+1) * 2 */
#  define LPSRAM_BTCTRL_STEPSIZE_X4      (2 << LPSRAM_BTCTRL_STEPSIZE_SHIFT) /* Next ADDR = ADDR + (BEATSIZE+1) * 4 */
#  define LPSRAM_BTCTRL_STEPSIZE_X8      (3 << LPSRAM_BTCTRL_STEPSIZE_SHIFT) /* Next ADDR = ADDR + (BEATSIZE+1) * 8 */
#  define LPSRAM_BTCTRL_STEPSIZE_X16     (4 << LPSRAM_BTCTRL_STEPSIZE_SHIFT) /* Next ADDR = ADDR + (BEATSIZE+1) * 16 */
#  define LPSRAM_BTCTRL_STEPSIZE_X32     (5 << LPSRAM_BTCTRL_STEPSIZE_SHIFT) /* Next ADDR = ADDR + (BEATSIZE+1) * 32 */
#  define LPSRAM_BTCTRL_STEPSIZE_X64     (6 << LPSRAM_BTCTRL_STEPSIZE_SHIFT) /* Next ADDR = ADDR + (BEATSIZE+1) * 64 */
#  define LPSRAM_BTCTRL_STEPSIZE_X128    (7 << LPSRAM_BTCTRL_STEPSIZE_SHIFT) /* Next ADDR = ADDR + (BEATSIZE+1) * 128 */

/* Block Transfer Count Register (16-bit count) */

/* Block Transfer Source Address Register (32-bit address) */

/* Block Transfer Destination Address Register (32-bit address) */

/* Next Address Descriptor Register (32-bit address) */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* DMA descriptor */

struct dma_desc_s
{
  uint16_t btctrl;   /* Block Transfer Control Register */
  uint16_t btcnt;    /* Block Transfer Count Register */
  uint32_t srcaddr;  /* Block Transfer Source Address Register */
  uint32_t dstaddr;  /* Block Transfer Destination Address Register */
  uint32_t descaddr; /* Next Address Descriptor Register */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* CONFIG_ARCH_FAMILY_SAML21 */
#endif /* __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAML_DMAC_H */
