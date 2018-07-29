/********************************************************************************************
 * arch/arm/src/samd5e5/chip/sam_dmac.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ********************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAMD5E5_CHIP_SAM_DMAC_H
#define __ARCH_ARM_SRC_SAMD5E5_CHIP_SAM_DMAC_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "chip/sam_memorymap.h"

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

/* DMAC register offsets ********************************************************************/

#define SAM_DMAC_CTRL_OFFSET             0x0000  /* Control Register */
#define SAM_DMAC_CRCCTRL_OFFSET          0x0002  /* CRC Control Register */
#define SAM_DMAC_CRCDATAIN_OFFSET        0x0004  /* CRC Data Input Register */
#define SAM_DMAC_CRCCHKSUM_OFFSET        0x0008  /* CRC Checksum Register */
#define SAM_DMAC_CRCSTATUS_OFFSET        0x000c  /* CRC Status Register */
#define SAM_DMAC_DBGCTRL_OFFSET          0x000d  /* Debug Control Register */
#define SAM_DMAC_SWTRIGCTRL_OFFSET       0x0010  /* Software Trigger Control Register */
#define SAM_DMAC_PRICTRL0_OFFSET         0x0014  /* Priority Control 0 Register */
#define SAM_DMAC_INTPEND_OFFSET          0x0020  /* Interrupt Pending Register */
#define SAM_DMAC_INTSTATUS_OFFSET        0x0024  /* Interrupt Status Register */
#define SAM_DMAC_BUSYCH_OFFSET           0x0028  /* Busy Channels Register */
#define SAM_DMAC_PENDCH_OFFSET           0x002c  /* Pending Channels Register */
#define SAM_DMAC_ACTIVE_OFFSET           0x0030  /* Active Channels and Levels Register */
#define SAM_DMAC_BASEADDR_OFFSET         0x0034  /* Descriptor Memory Section Base Address Register */
#define SAM_DMAC_WRBADDR_OFFSET          0x0038  /* Write-Back Memory Section Base Address Register */

#define SAM_DMAC_CHAN_OFFSET(n)          (0x0040 + ((n) << 4))
#  define SAM_DMAC_CHCTRLA_OFFSET        0x0000  /* Channel Control A Register */
#  define SAM_DMAC_CHCTRLB_OFFSET        0x0004  /* Channel Control B Register */
#  define SAM_DMAC_CHPRILVL_OFFSET       0x0005  /* Channel Priority Level */
#  define SAM_DMAC_CHEVCTRL_OFFSET       0x0006  /* Channel Event Contol Register */
#  define SAM_DMAC_CHINTENCLR_OFFSET     0x000c  /* Channel Interrupt Enable Clear Register */
#  define SAM_DMAC_CHINTENSET_OFFSET     0x000d  /* Channel Interrupt Enable Set Register */
#  define SAM_DMAC_CHINTFLAG_OFFSET      0x000e  /* Channel Interrupt Flag Status and Clear Register */
#  define SAM_DMAC_CHSTATUS_OFFSET       0x000f  /* Channel Status Register */

/* LPSRAM Registers Relative to BASEADDR or WRBADDR */

#define SAM_LPSRAM_BTCTRL_OFFSET         0x0000  /* Block Transfer Control Register */
#define SAM_LPSRAM_BTCNT_OFFSET          0x0002  /* Block Transfer Count Register */
#define SAM_LPSRAM_SRCADDR_OFFSET        0x0004  /* Block Transfer Source Address Register */
#define SAM_LPSRAM_DSTADDR_OFFSET        0x0008  /* Block Transfer Destination Address Register */
#define SAM_LPSRAM_DESCADDR_OFFSET       0x000c  /* Next Address Descriptor Register */

/* DMAC register addresses ******************************************************************/

#define SAM_DMAC_CTRL                  (SAM_DMAC_BASE + SAM_DMAC_CTRL_OFFSET)
#define SAM_DMAC_CRCCTRL               (SAM_DMAC_BASE + SAM_DMAC_CRCCTRL_OFFSET)
#define SAM_DMAC_CRCDATAIN             (SAM_DMAC_BASE + SAM_DMAC_CRCDATAIN_OFFSET)
#define SAM_DMAC_CRCCHKSUM             (SAM_DMAC_BASE + SAM_DMAC_CRCCHKSUM_OFFSET)
#define SAM_DMAC_CRCSTATUS             (SAM_DMAC_BASE + SAM_DMAC_CRCSTATUS_OFFSET)
#define SAM_DMAC_DBGCTRL               (SAM_DMAC_BASE + SAM_DMAC_DBGCTRL_OFFSET)
#define SAM_DMAC_SWTRIGCTRL            (SAM_DMAC_BASE + SAM_DMAC_SWTRIGCTRL_OFFSET)
#define SAM_DMAC_PRICTRL0              (SAM_DMAC_BASE + SAM_DMAC_PRICTRL0_OFFSET)
#define SAM_DMAC_INTPEND               (SAM_DMAC_BASE + SAM_DMAC_INTPEND_OFFSET)
#define SAM_DMAC_INTSTATUS             (SAM_DMAC_BASE + SAM_DMAC_INTSTATUS_OFFSET)
#define SAM_DMAC_BUSYCH                (SAM_DMAC_BASE + SAM_DMAC_BUSYCH_OFFSET)
#define SAM_DMAC_PENDCH                (SAM_DMAC_BASE + SAM_DMAC_PENDCH_OFFSET)
#define SAM_DMAC_ACTIVE                (SAM_DMAC_BASE + SAM_DMAC_ACTIVE_OFFSET)
#define SAM_DMAC_BASEADDR              (SAM_DMAC_BASE + SAM_DMAC_BASEADDR_OFFSET)
#define SAM_DMAC_WRBADDR               (SAM_DMAC_BASE + SAM_DMAC_WRBADDR_OFFSET)

#define SAM_DMAC_CHAN_BASE(n)          (SAM_DMAC_BASE + SAM_DMAC_CHAN_OFFSET(n))
#define SAM_DMAC_CHCTRLA(n)            (SAM_DMAC_CHAN_BASE(n) + SAM_DMAC_CHCTRLA_OFFSET)
#define SAM_DMAC_CHCTRLB(n)            (SAM_DMAC_CHAN_BASE(n) + SAM_DMAC_CHCTRLB_OFFSET)
#define SAM_DMAC_CHPRILVL(n)           (SAM_DMAC_CHAN_BASE(n) + SAM_DMAC_CHPRILVL_OFFSET)
#define SAM_DMAC_CHEVCTRL(n)           (SAM_DMAC_CHAN_BASE(n) + SAM_DMAC_CHEVCTRL_OFFSET)
#define SAM_DMAC_CHINTENCLR(n)         (SAM_DMAC_CHAN_BASE(n) + SAM_DMAC_CHINTENCLR_OFFSET)
#define SAM_DMAC_CHINTENSET(n)         (SAM_DMAC_CHAN_BASE(n) + SAM_DMAC_CHINTENSET_OFFSET)
#define SAM_DMAC_CHINTFLAG(n)          (SAM_DMAC_CHAN_BASE(n) + SAM_DMAC_CHINTFLAG_OFFSET)
#define SAM_DMAC_CHSTATUS(n)           (SAM_DMAC_CHAN_BASE(n) + SAM_DMAC_CHSTATUS_OFFSET)

/* DMAC register bit definitions ************************************************************/

/* Control Register */

#define DMAC_CTRL_SWRST                  (1 << 0)  /* Bit 0:  Software Reset */
#define DMAC_CTRL_DMAENABLE              (1 << 1)  /* Bit 1:  DMA Enable */
#define DMAC_CTRL_LVLEN0                 (1 << 8)  /* Bit 8:  Priority level 0 Enable */
#define DMAC_CTRL_LVLEN1                 (1 << 9)  /* Bit 9:  Priority level 1 Enable */
#define DMAC_CTRL_LVLEN2                 (1 << 10) /* Bit 10: Priority level 2 Enable */
#define DMAC_CTRL_LVLEN3                 (1 << 11) /* Bit 11: Priority level 3 Enable */

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
#define DMAC_CRCCTRL_CRCMODE_SHIFT       (14)       /* Bits 14-15: CRC Operating Mode */
#define DMAC_CRCCTRL_CRCMODE_MASK        (3 < DMAC_CRCCTRL_CRCMODE_SHIFT)
#  define DMAC_CRCCTRL_CRCMODE_DEFAULT   (0 < DMAC_CRCCTRL_CRCMODE_SHIFT) /* Default operating mode */
#  define DMAC_CRCCTRL_CRCMODE_CRCMON    (2 < DMAC_CRCCTRL_CRCMODE_SHIFT) /* Memory CRC monitor mode */
#  define DMAC_CRCCTRL_CRCMODE_CRCGEN    (3 < DMAC_CRCCTRL_CRCMODE_SHIFT) /* Memory CRC generation mode */

/* CRC Data Input Register (32-bit value) */
/* CRC Checksum Register (32-bit value) */

/* CRC Status Register */

#define DMAC_CRCSTATUS_CRCBUSY           (1 << 0)  /* Bit 0:  CRC module busy */
#define DMAC_CRCSTATUS_CRCZERO           (1 << 1)  /* Bit 1:  CRC zero */
#define DMAC_CRCSTATUS_CRCERR            (1 << 2)  /* Bit 2:  CRC Error */

/* Debug Control Register */

#define DMAC_DBGCTRL_DBGRUN              (1 << 0)  /* Bit 0:  Debug run */

/* Common bit definitions for: Software Trigger Control Register, Interrupt Status Register,
 * Busy Channels Register, and Pending Channels Register
 */

#define DMAC_CHAN(n)                     (1 << (n)) /* DMAC Channel n, n=0-15 */

/* Priority Control 0 Register */

#define DMAC_PRICTRL0_LVLPRI0_SHIFT      (0)       /* Bits 0-4: Level 0 channel priority number */
#define DMAC_PRICTRL0_LVLPRI0_MASK       (31 << DMAC_PRICTRL0_LVLPRI0_SHIFT)
#  define DMAC_PRICTRL0_LVLPRI0(n)       ((uint32_t)(n) << DMAC_PRICTRL0_LVLPRI0_SHIFT)
#define DMAC_PRICTRL0_QOS00_SHIFT        (0)       /* Bits 5-6: Level of quality */
#define DMAC_PRICTRL0_QOS00_MASK         (3 << DMAC_PRICTRL0_QOS00_SHIFT)
#  define DMAC_PRICTRL0_QOS00_DISABLE    (0 << DMAC_PRICTRL0_QOS00_SHIFT) /* Background */
#  define DMAC_PRICTRL0_QOS00_LOW        (1 << DMAC_PRICTRL0_QOS00_SHIFT) /* Low bandwidth sensitivity */
#  define DMAC_PRICTRL0_QOS00_MEDIUM     (2 << DMAC_PRICTRL0_QOS00_SHIFT) /* Sensitive to latency */
#  define DMAC_PRICTRL0_QOS00_CRITICAL   (3 << DMAC_PRICTRL0_QOS00_SHIFT) /* Latency critical */
#define DMAC_PRICTRL0_RRLVLEN0           (1 << 7)  /* Bit 7:  Level 0 round-robin arbitrarion enable */
#define DMAC_PRICTRL0_LVLPRI1_SHIFT      (8)       /* Bits 8-12: Level 1 channel priority number */
#define DMAC_PRICTRL0_LVLPRI1_MASK       (31 << DMAC_PRICTRL0_LVLPRI1_SHIFT)
#  define DMAC_PRICTRL0_LVLPRI1(n)       ((uint32_t)(n) << DMAC_PRICTRL0_LVLPRI1_SHIFT)
#define DMAC_PRICTRL0_QOS01_SHIFT        (13)      /* Bits 13-14: Level of quality */
#define DMAC_PRICTRL0_QOS01_MASK         (3 << DMAC_PRICTRL0_QOS01_SHIFT)
#  define DMAC_PRICTRL0_QOS01_DISABLE    (0 << DMAC_PRICTRL0_QOS01_SHIFT) /* Background */
#  define DMAC_PRICTRL0_QOS01_LOW        (1 << DMAC_PRICTRL0_QOS01_SHIFT) /* Low bandwidth sensitivity */
#  define DMAC_PRICTRL0_QOS01_MEDIUM     (2 << DMAC_PRICTRL0_QOS01_SHIFT) /* Sensitive to latency */
#  define DMAC_PRICTRL0_QOS01_CRITICAL   (3 << DMAC_PRICTRL0_QOS01_SHIFT) /* Latency critical */
#define DMAC_PRICTRL0_RRLVLEN1           (1 << 15) /* Bit 15:  Level 1 round-robin arbitrarion enable */
#define DMAC_PRICTRL0_LVLPRI2_SHIFT      (16)      /* Bits 16-20: Level 2 channel priority number */
#define DMAC_PRICTRL0_LVLPRI2_MASK       (31 << DMAC_PRICTRL0_LVLPRI2_SHIFT)
#  define DMAC_PRICTRL0_LVLPRI2(n)       ((uint32_t)(n) << DMAC_PRICTRL0_LVLPRI2_SHIFT)
#define DMAC_PRICTRL0_QOS02_SHIFT        (21)      /* Bits 21-22: Level of quality */
#define DMAC_PRICTRL0_QOS02_MASK         (3 << DMAC_PRICTRL0_QOS02_SHIFT)
#  define DMAC_PRICTRL0_QOS02_DISABLE    (0 << DMAC_PRICTRL0_QOS02_SHIFT) /* Background */
#  define DMAC_PRICTRL0_QOS02_LOW        (1 << DMAC_PRICTRL0_QOS02_SHIFT) /* Low bandwidth sensitivity */
#  define DMAC_PRICTRL0_QOS02_MEDIUM     (2 << DMAC_PRICTRL0_QOS02_SHIFT) /* Sensitive to latency */
#  define DMAC_PRICTRL0_QOS02_CRITICAL   (3 << DMAC_PRICTRL0_QOS02_SHIFT) /* Latency critical */
#define DMAC_PRICTRL0_RRLVLEN2           (1 << 23) /* Bit 23:  Level 2 round-robin arbitrarion enable */
#define DMAC_PRICTRL0_LVLPRI3_SHIFT      (24)      /* Bits 24-28: Level 2 channel priority number */
#define DMAC_PRICTRL0_LVLPRI3_MASK       (31 << DMAC_PRICTRL0_LVLPRI3_SHIFT)
#  define DMAC_PRICTRL0_LVLPRI3(n)       ((uint32_t)(n) << DMAC_PRICTRL0_LVLPRI3_SHIFT)
#define DMAC_PRICTRL0_QOS03_SHIFT        (29)      /* Bits 29-30: Level of quality */
#define DMAC_PRICTRL0_QOS03_MASK         (3 << DMAC_PRICTRL0_QOS03_SHIFT)
#  define DMAC_PRICTRL0_QOS03_DISABLE    (0 << DMAC_PRICTRL0_QOS03_SHIFT) /* Background */
#  define DMAC_PRICTRL0_QOS03_LOW        (1 << DMAC_PRICTRL0_QOS03_SHIFT) /* Low bandwidth sensitivity */
#  define DMAC_PRICTRL0_QOS03_MEDIUM     (2 << DMAC_PRICTRL0_QOS03_SHIFT) /* Sensitive to latency */
#  define DMAC_PRICTRL0_QOS03_CRITICAL   (3 << DMAC_PRICTRL0_QOS03_SHIFT) /* Latency critical */
#define DMAC_PRICTRL0_RRLVLEN3           (1 << 23) /* Bit 21:  Level 3 round-robin arbitrarion enable */

/* Interrupt Pending Register */

#define DMAC_INTPEND_ID_SHIFT            (0)       /* Bit 0-3: Channel ID */
#define DMAC_INTPEND_ID_MASK             (31 << DMAC_INTPEND_ID_SHIFT)
#define DMAC_INTPEND_TERR                (1 << 8)  /* Bit 8:  Transfer error */
#define DMAC_INTPEND_TCMPL               (1 << 9)  /* Bit 9:  Transfer complete */
#define DMAC_INTPEND_SUSP                (1 << 10) /* Bit 10: Channel suspend */
#define DMAC_INTPEND_CRCERR              (1 << 12) /* Bit 12: CRC error */
#define DMAC_INTPEND_FERR                (1 << 13) /* Bit 13: Fetch error */
#define DMAC_INTPEND_BUSY                (1 << 14) /* Bit 14: Busy */
#define DMAC_INTPEND_PEND                (1 << 15) /* Bit 15: Pending */

/* Active Channels and Levels Register */

#define DMAC_ACTIVE_LVLEX0               (1 << 0)  /* Bit 0:  Level 0 channel trigger request executing */
#define DMAC_ACTIVE_LVLEX1               (1 << 1)  /* Bit 1:  Level 1 channel trigger request executing */
#define DMAC_ACTIVE_LVLEX2               (1 << 2)  /* Bit 2:  Level 2 channel trigger request executing */
#define DMAC_ACTIVE_LVLEX3               (1 << 3)  /* Bit 3:  Level 3 channel trigger request executing */
#define DMAC_ACTIVE_ID_SHIFT             (8)       /* Bits 8-12: Active channel ID */
#define DMAC_ACTIVE_ID_MASK              (31 << DMAC_ACTIVE_ID_SHIFT)
#define DMAC_ACTIVE_ABUSY                (1 << 15) /* Bit 15: Active channel busy */
#define DMAC_ACTIVE_BTCNT_SHIFT          (16)      /* Bit 16-31: Active channel block transfer count */
#define DMAC_ACTIVE_BTCNT_MASK           (0xffff << DMAC_ACTIVE_BTCNT_SHIFT)

/* Descriptor Memory Section Base Address Register (32-bit address) */
/* Write-Back Memory Section Base Address Register (32-bit address) */

/* Channel Control A Register */

#define DMAC_CHCTRLA_SWRST               (1 << 0)  /* Bit 0:  Channel software reset */
#define DMAC_CHCTRLA_ENABLE              (1 << 1)  /* Bit 1:  Channel enable */
#define DMAC_CHCTRLA_RUNSTDBY            (1 << 6)  /* Bit 6:  Channel run in standby */
#define DMAC_CHCTRLA_TRIGSRC_SHIFT       (8)       /* Bits 8-15: Trigger Source (see below) */
#define DMAC_CHCTRLA_TRIGSRC_MASK        (0xff << DMAC_CHCTRLA_TRIGSRC_SHIFT)
#  define DMAC_CHCTRLA_TRIGSRC(n)        ((uint32_t)(n) << DMAC_CHCTRLA_TRIGSRC_SHIFT)
#define DMAC_CHCTRLA_TRIGACT_SHIFT       (21)      /* Bits 20-21: Trigger Action */
#define DMAC_CHCTRLA_TRIGACT_MASK        (3 << DMAC_CHCTRLA_TRIGACT_SHIFT)
#  define DMAC_CHCTRLA_TRIGACT(n)        ((uint32_t)(n) << DMAC_CHCTRLA_TRIGACT_SHIFT)
#  define DMAC_CHCTRLA_TRIGACT_BLOCK     (0 << DMAC_CHCTRLA_TRIGACT_SHIFT) /* Trigger per block transfer */
#  define DMAC_CHCTRLA_TRIGACT_BURST     (2 << DMAC_CHCTRLA_TRIGACT_SHIFT) /* Trigger per burst transfer */
#  define DMAC_CHCTRLA_TRIGACT_TRANS     (3 << DMAC_CHCTRLA_TRIGACT_SHIFT) /* Trigger for each transaction */
#define DMAC_CHCTRLA_BURSTLEN_SHIFT      (24)      /* Bits 24-27: Burst Length (beats-1) */
#define DMAC_CHCTRLA_BURSTLEN_MASK       (15 << DMAC_CHCTRLA_BURSTLEN_SHIFT)
#  define DMAC_CHCTRLA_BURSTLEN(n)       ((uint32_t)((n) - 1)  << DMAC_CHCTRLA_BURSTLEN_SHIFT)
#  define DMAC_CHCTRLA_BURSTLEN_1BEAT    (0  << DMAC_CHCTRLA_BURSTLEN_SHIFT) /* Single-beat burst */
#  define DMAC_CHCTRLA_BURSTLEN_2BEATS   (1  << DMAC_CHCTRLA_BURSTLEN_SHIFT) /* 2-beats burst length */
#  define DMAC_CHCTRLA_BURSTLEN_3BEATS   (2  << DMAC_CHCTRLA_BURSTLEN_SHIFT) /* 3-beats burst length */
#  define DMAC_CHCTRLA_BURSTLEN_4BEATS   (3  << DMAC_CHCTRLA_BURSTLEN_SHIFT) /* 4-beats burst length */
#  define DMAC_CHCTRLA_BURSTLEN_5BEATS   (4  << DMAC_CHCTRLA_BURSTLEN_SHIFT) /* 5-beats burst length */
#  define DMAC_CHCTRLA_BURSTLEN_6BEATS   (5  << DMAC_CHCTRLA_BURSTLEN_SHIFT) /* 6-beats burst length */
#  define DMAC_CHCTRLA_BURSTLEN_7BEATS   (6  << DMAC_CHCTRLA_BURSTLEN_SHIFT) /* 7-beats burst length */
#  define DMAC_CHCTRLA_BURSTLEN_8BEATS   (7  << DMAC_CHCTRLA_BURSTLEN_SHIFT) /* 8-beats burst length */
#  define DMAC_CHCTRLA_BURSTLEN_9BEATS   (8  << DMAC_CHCTRLA_BURSTLEN_SHIFT) /* 9-beats burst length */
#  define DMAC_CHCTRLA_BURSTLEN_10BEATS  (9  << DMAC_CHCTRLA_BURSTLEN_SHIFT) /* 10-beats burst length */
#  define DMAC_CHCTRLA_BURSTLEN_11BEATS  (10 << DMAC_CHCTRLA_BURSTLEN_SHIFT) /* 11-beats burst length */
#  define DMAC_CHCTRLA_BURSTLEN_12BEATS  (11 << DMAC_CHCTRLA_BURSTLEN_SHIFT) /* 12-beats burst length */
#  define DMAC_CHCTRLA_BURSTLEN_13BEATS  (12 << DMAC_CHCTRLA_BURSTLEN_SHIFT) /* 13-beats burst length */
#  define DMAC_CHCTRLA_BURSTLEN_14BEATS  (13 << DMAC_CHCTRLA_BURSTLEN_SHIFT) /* 14-beats burst length */
#  define DMAC_CHCTRLA_BURSTLEN_15BEATS  (14 << DMAC_CHCTRLA_BURSTLEN_SHIFT) /* 15-beats burst length */
#  define DMAC_CHCTRLA_BURSTLEN_16BEATS  (15 << DMAC_CHCTRLA_BURSTLEN_SHIFT) /* 16-beats burst length */
#define DMAC_CHCTRLA_THRESHOLD_SHIFT     (28)      /* Bits 28-29: FIFO Threshold (log2 beats) */
#define DMAC_CHCTRLA_THRESHOLD_MASK      (3 << DMAC_CHCTRLA_THRESHOLD_SHIFT)
#  define DMAC_CHCTRLA_THRESHOLD(n)      ((uint32_t)(n) << DMAC_CHCTRLA_THRESHOLD_SHIFT)
#  define DMAC_CHCTRLA_THRESHOLD_1BEAT   (0 << DMAC_CHCTRLA_THRESHOLD_SHIFT) /* Write after 1 beat */
#  define DMAC_CHCTRLA_THRESHOLD_2BEATS  (1 << DMAC_CHCTRLA_THRESHOLD_SHIFT) /* Write after 2 beats */
#  define DMAC_CHCTRLA_THRESHOLD_4BEATS  (2 << DMAC_CHCTRLA_THRESHOLD_SHIFT) /* Write after 3 beats */
#  define DMAC_CHCTRLA_THRESHOLD_8BEATS  (3 << DMAC_CHCTRLA_THRESHOLD_SHIFT) /* Write after 8 beats */

/* Trigger sources used with the CHCTRLA TRIGSRC field */

#define DMAC_CHCTRLA_TRIGSRC_DISABLE         0x00  /* Only software/event triggers */
#define DMAC_CHCTRLA_TRIGSRC_RTC             0x01  /* TIMESTAMP DMA RTC timestamp trigger */
#define DMAC_CHCTRLA_TRIGSRC_DSU_DCC0        0x02  /* DMAC ID for DCC0 register */
#define DMAC_CHCTRLA_TRIGSRC_DSU_DCC1        0x03  /* DMAC ID for DCC1 register */
#define DMAC_CHCTRLA_TRIGSRC_SERCOM0_RX      0x04  /* Index of SERCOM0 DMA RX trigger */
#define DMAC_CHCTRLA_TRIGSRC_SERCOM0_TX      0x05  /* Index of SERCOM0 DMA TX trigger */
#define DMAC_CHCTRLA_TRIGSRC_SERCOM1_RX      0x06  /* Index of SERCOM1 DMA RX trigger */
#define DMAC_CHCTRLA_TRIGSRC_SERCOM1_TX      0x07  /* Index of SERCOM1 DMA TX trigger */
#define DMAC_CHCTRLA_TRIGSRC_SERCOM2_RX      0x08  /* Index of SERCOM2 DMA RX trigger */
#define DMAC_CHCTRLA_TRIGSRC_SERCOM2_TX      0x09  /* Index of SERCOM2 DMA TX trigger */
#define DMAC_CHCTRLA_TRIGSRC_SERCOM3_RX      0x0a  /* Index of SERCOM3 DMA RX trigger */
#define DMAC_CHCTRLA_TRIGSRC_SERCOM3_TX      0x0b  /* Index of SERCOM3 DMA TX trigger */
#define DMAC_CHCTRLA_TRIGSRC_SERCOM4_RX      0x0c  /* Index of SERCOM4 DMA RX trigger */
#define DMAC_CHCTRLA_TRIGSRC_SERCOM4_TX      0x0d  /* Index of SERCOM4 DMA TX trigger */
#define DMAC_CHCTRLA_TRIGSRC_SERCOM5_RX      0x0e  /* Index of SERCOM5 DMA RX trigger */
#define DMAC_CHCTRLA_TRIGSRC_SERCOM5_TX      0x0f  /* Index of SERCOM5 DMA TX trigger */
#define DMAC_CHCTRLA_TRIGSRC_SERCOM6_RX      0x10  /* Index of SERCOM6 DMA RX trigger */
#define DMAC_CHCTRLA_TRIGSRC_SERCOM6_TX      0x11  /* Index of SERCOM6 DMA TX trigger */
#define DMAC_CHCTRLA_TRIGSRC_SERCOM7_RX      0x12  /* Index of SERCOM7 DMA RX trigger */
#define DMAC_CHCTRLA_TRIGSRC_SERCOM7_TX      0x13  /* Index of SERCOM7 DMA TX trigger */
#define DMAC_CHCTRLA_TRIGSRC_CAN0_DEBUG      0x14  /* DMA CAN0 Debug Req */
#define DMAC_CHCTRLA_TRIGSRC_CAN1_DEBUG      0x15  /* DMA CAN1 Debug Req */
#define DMAC_CHCTRLA_TRIGSRC_TCC0_OVF        0x16  /* TCC0 DMA overflow/underflow/retrigger trigger */
#define DMAC_CHCTRLA_TRIGSRC_TCC0_MC0        0x17  /* Index of TCC0 DMA Match/Compare trigger 0 */
#define DMAC_CHCTRLA_TRIGSRC_TCC0_MC1        0x18  /* Index of TCC0 DMA Match/Compare trigger 1 */
#define DMAC_CHCTRLA_TRIGSRC_TCC0_MC2        0x19  /* Index of TCC0 DMA Match/Compare trigger 2 */
#define DMAC_CHCTRLA_TRIGSRC_TCC0_MC3        0x1a  /* Index of TCC0 DMA Match/Compare trigger 3 */
#define DMAC_CHCTRLA_TRIGSRC_TCC0_MC4        0x1b  /* Index of TCC0 DMA Match/Compare trigger 4 */
#define DMAC_CHCTRLA_TRIGSRC_TCC0_MC5        0x1c  /* Index of TCC0 DMA Match/Compare trigger 5 */
#define DMAC_CHCTRLA_TRIGSRC_TCC1_OVF        0x1d  /* TCC1 DMA overflow/underflow/retrigger trigger */
#define DMAC_CHCTRLA_TRIGSRC_TCC1_MC0        0x1e  /* Index of TCC1 DMA Match/Compare trigger 0 */
#define DMAC_CHCTRLA_TRIGSRC_TCC1_MC1        0x1f  /* Index of TCC1 DMA Match/Compare trigger 1 */
#define DMAC_CHCTRLA_TRIGSRC_TCC1_MC2        0x20  /* Index of TCC1 DMA Match/Compare trigger 2 */
#define DMAC_CHCTRLA_TRIGSRC_TCC1_MC3        0x21  /* Index of TCC1 DMA Match/Compare trigger 3 */
#define DMAC_CHCTRLA_TRIGSRC_TCC2_OVF        0x22  /* TCC2 DMA overflow/underflow/retrigger trigger */
#define DMAC_CHCTRLA_TRIGSRC_TCC2_MC0        0x23  /* Index of TCC2 DMA Match/Compare trigger 0 */
#define DMAC_CHCTRLA_TRIGSRC_TCC2_MC1        0x24  /* Index of TCC2 DMA Match/Compare trigger 1 */
#define DMAC_CHCTRLA_TRIGSRC_TCC2_MC2        0x25  /* Index of TCC2 DMA Match/Compare trigger 2 */
#define DMAC_CHCTRLA_TRIGSRC_TCC3_OVF        0x26  /* TCC3 DMA overflow/underflow/retrigger trigger */
#define DMAC_CHCTRLA_TRIGSRC_TCC3_MC0        0x27  /* Index of TCC3 DMA Match/Compare trigger 0 */
#define DMAC_CHCTRLA_TRIGSRC_TCC3_MC1        0x28  /* Index of TCC3 DMA Match/Compare trigger 1 */
#define DMAC_CHCTRLA_TRIGSRC_TCC4_OVF        0x29  /* TCC4 DMA overflow/underflow/retrigger trigger */
#define DMAC_CHCTRLA_TRIGSRC_TCC4_MC0        0x2a  /* Index of TCC4 DMA Match/Compare trigger 0 */
#define DMAC_CHCTRLA_TRIGSRC_TCC4_MC1        0x2b  /* Index of TCC4 DMA Match/Compare trigger 1 */
#define DMAC_CHCTRLA_TRIGSRC_TC0_OVF         0x2c  /* TC0 DMA overflow/underflow trigger */
#define DMAC_CHCTRLA_TRIGSRC_TC0_MC0         0x2d  /* Index of TC0 DMA Match/Compare trigger 0 */
#define DMAC_CHCTRLA_TRIGSRC_TC0_MC1         0x2e  /* Index of TC0 DMA Match/Compare trigger 1 */
#define DMAC_CHCTRLA_TRIGSRC_TC1_OVF         0x2f  /* TC1 DMA overflow/underflow trigger */
#define DMAC_CHCTRLA_TRIGSRC_TC1_MC0         0x30  /* Index of TC1 DMA Match/Compare trigger 0 */
#define DMAC_CHCTRLA_TRIGSRC_TC1_MC1         0x31  /* Index of TC1 DMA Match/Compare trigger 1 */
#define DMAC_CHCTRLA_TRIGSRC_TC2_OVF         0x32  /* TC2 DMA overflow/underflow trigger */
#define DMAC_CHCTRLA_TRIGSRC_TC2_MC0         0x33  /* Index of TC2 DMA Match/Compare trigger 0 */
#define DMAC_CHCTRLA_TRIGSRC_TC2_MC1         0x34  /* Index of TC2 DMA Match/Compare trigger 1 */
#define DMAC_CHCTRLA_TRIGSRC_TC3_OVF         0x35  /* TC3 DMA overflow/underflow trigger */
#define DMAC_CHCTRLA_TRIGSRC_TC3_MC0         0x36  /* Index of TC3 DMA Match/Compare trigger 0 */
#define DMAC_CHCTRLA_TRIGSRC_TC3_MC1         0x37  /* Index of TC3 DMA Match/Compare trigger 1 */
#define DMAC_CHCTRLA_TRIGSRC_TC4_OVF         0x38  /* TC4 DMA overflow/underflow trigger */
#define DMAC_CHCTRLA_TRIGSRC_TC4_MC0         0x39  /* Index of TC4 DMA Match/Compare trigger 0 */
#define DMAC_CHCTRLA_TRIGSRC_TC4_MC1         0x3a  /* Index of TC4 DMA Match/Compare trigger 1 */
#define DMAC_CHCTRLA_TRIGSRC_TC5_OVF         0x3b  /* TC5 DMA overflow/underflow trigger */
#define DMAC_CHCTRLA_TRIGSRC_TC5_MC0         0x3c  /* Index of TC5 DMA Match/Compare trigger 0 */
#define DMAC_CHCTRLA_TRIGSRC_TC5_MC1         0x3d  /* Index of TC5 DMA Match/Compare trigger 1 */
#define DMAC_CHCTRLA_TRIGSRC_TC6_OVF         0x3e  /* TC6 DMA overflow/underflow trigger */
#define DMAC_CHCTRLA_TRIGSRC_TC6_MC0         0x3f  /* Index of TC6 DMA Match/Compare trigger 0 */
#define DMAC_CHCTRLA_TRIGSRC_TC6_MC1         0x40  /* Index of TC6 DMA Match/Compare trigger 1 */
#define DMAC_CHCTRLA_TRIGSRC_TC7_OVF         0x41  /* TC7 DMA overflow/underflow trigger */
#define DMAC_CHCTRLA_TRIGSRC_TC7_MC0         0x42  /* Index of TC7 DMA Match/Compare trigger 0 */
#define DMAC_CHCTRLA_TRIGSRC_TC7_MC1         0x43  /* Index of TC7 DMA Match/Compare trigger 1 */
#define DMAC_CHCTRLA_TRIGSRC_ADC0_RESRDY     0x44  /* ADC0 index of DMA RESRDY trigger */
#define DMAC_CHCTRLA_TRIGSRC_ADC0_SEQ        0x45  /* ADC0 Index of DMA SEQ trigger */
#define DMAC_CHCTRLA_TRIGSRC_ADC1_RESRDY     0x46  /* ADC1 Index of DMA RESRDY trigger */
#define DMAC_CHCTRLA_TRIGSRC_ADC1_SEQ        0x47  /* ADC1 Index of DMA SEQ trigger */
#define DMAC_CHCTRLA_TRIGSRC_DAC0_EMPTY      0x48  /* DMA DAC0 Empty Req */
#define DMAC_CHCTRLA_TRIGSRC_DAC1_EMPTY      0x49  /* DMA DAC1 Empty Req */
#define DMAC_CHCTRLA_TRIGSRC_DAC0_RESRDY     0x4a  /* DMA DAC0 Result Ready Req */
#define DMAC_CHCTRLA_TRIGSRC_DAC1_RESRDY     0x4b  /* DMA DAC1 Result Ready Req */
#define DMAC_CHCTRLA_TRIGSRC_I2S0_RX         0x4c  /* Index of I2S DMA RX trigger */
#define DMAC_CHCTRLA_TRIGSRC_I2S1_RX         0x4d  /* Index of I2S DMA RX trigger */
#define DMAC_CHCTRLA_TRIGSRC_I2S0_TX         0x4e  /* Index of I2S DMA TX trigger */
#define DMAC_CHCTRLA_TRIGSRC_I2S1_TX         0x4f  /* Index of I2S DMA TX trigger */
#define DMAC_CHCTRLA_TRIGSRC_PCC_RX          0x50  /* Indexes of PCC RX trigger */
#define DMAC_CHCTRLA_TRIGSRC_AES_WR          0x51  /* DMA DATA Write trigger */
#define DMAC_CHCTRLA_TRIGSRC_AES_RD          0x52  /* DMA DATA Read trigger */
#define DMAC_CHCTRLA_TRIGSRC_QSPI_RX         0x53  /* Indexes of QSPI RX trigger */
#define DMAC_CHCTRLA_TRIGSRC_QSPI_TX         0x54  /* Indexes of QSPI TX trigger */

/* Channel Control B Register */

#define DMAC_CHCTRLB_CMD_SHIFT           (0)       /* Bits 0-1: Software command */
#define DMAC_CHCTRLB_CMD_MASK            (3 << DMAC_CHCTRLB_CMD_SHIFT)
#  define DMAC_CHCTRLB_CMD_NOACTION      (0 << DMAC_CHCTRLB_CMD_SHIFT) /* No action */
#  define DMAC_CHCTRLB_CMD_SUSPEND       (1 << DMAC_CHCTRLB_CMD_SHIFT) /* Channel suspend operation */
#  define DMAC_CHCTRLB_CMD_RESUME        (2 << DMAC_CHCTRLB_CMD_SHIFT) /* Channel resume operation */

/* Channel Priority Level */

#define DMAC_CHPRILVL_MASK               0x03      /* Channel priority level */
#  define DMAC_CHPRILVL(n)               ((uint8_t)(n))

/* Channel Event Control Register */

#define DMAC_CHEVCTRL_EVACT_SHIFT        (0)       /* Bits 0-2: Channel event input action */
#define DMAC_CHEVCTRL_EVACT_MASK         (7 << DMAC_CHEVCTRL_EVACT_SHIFT)
#  define DMAC_CHEVCTRL_EVACT_NOACT      (0 << DMAC_CHEVCTRL_EVACT_SHIFT) /* No action */
#  define DMAC_CHEVCTRL_EVACT_TRIG       (1 << DMAC_CHEVCTRL_EVACT_SHIFT) /* Transfer/periodic transfer trigger */
#  define DMAC_CHEVCTRL_EVACT_CTRIG      (2 << DMAC_CHEVCTRL_EVACT_SHIFT) /* Conditional transfer trigger */
#  define DMAC_CHEVCTRL_EVACT_CBLOCK     (3 << DMAC_CHEVCTRL_EVACT_SHIFT) /* Conditional block transfer */
#  define DMAC_CHEVCTRL_EVACT_SUSPEND    (4 << DMAC_CHEVCTRL_EVACT_SHIFT) /* Channel suspend operation */
#  define DMAC_CHEVCTRL_EVACT_RESUME     (5 << DMAC_CHEVCTRL_EVACT_SHIFT) /* Channel resume operation */
#  define DMAC_CHEVCTRL_EVACT_SSKIP      (6 << DMAC_CHEVCTRL_EVACT_SHIFT) /* Skip next block suspend action */
#  define DMAC_CHEVCTRL_EVACT_INCPRI     (7 << DMAC_CHEVCTRL_EVACT_SHIFT) /* Increase priority */
#define DMAC_CHEVCTRL_EVOMODE_SHIFT      (4)       /* Bits 4-5: Channel event output mode */
#define DMAC_CHEVCTRL_EVOMODE_MASK       (3 << DMAC_CHEVCTRL_EVOMODE_SHIFT)
#  define DMAC_CHEVCTRL_EVOMODE_DEFAULT  (0 << DMAC_CHEVCTRL_EVOMODE_SHIFT) /* Block event output selection */
#  define DMAC_CHEVCTRL_EVOMODE_TRIGACT  (1 << DMAC_CHEVCTRL_EVOMODE_SHIFT) /* Ongoing trigger action */
#define DMAC_CHEVCTRL_EVIE               (1 << 6)  /* Bit 6 –  Channel Event Input Enable */
#define DMAC_CHEVCTRL_EVOE               (1 << 7)  /* Bit 7 –  Channel Event Output Enable */

/* Common register bit definitions: Channel Interrupt Enable Clear Register, Channel Interrupt
 * Enable Set Register, and  Channel Interrupt Flag Status and Clear Register
 */

#define DMAC_INT_TERR                    (1 << 0)  /* Bit 0:  Transfer error interrupt */
#define DMAC_INT_TCMPL                   (1 << 1)  /* Bit 1:  Channel transfer complete interrupt */
#define DMAC_INT_SUSP                    (1 << 2)  /* Bit 2:  Channel suspend interrupt */
#define DMAC_INT_ALL                     (0x07)

/* Channel Status Register */

#define DMAC_CHSTATUS_PEND               (1 << 0)  /* Bit 0:  Chennel pending */
#define DMAC_CHSTATUS_BUSY               (1 << 1)  /* Bit 1:  Channel busy */
#define DMAC_CHSTATUS_FERR               (1 << 2)  /* Bit 2:  Channel fetch error */
#define DMAC_CHSTATUS_CRCERR             (1 << 3)  /* Bit 3:  CRC error */

/* Block Transfer Control Register */

#define LPSRAM_BTCTRL_VALID              (1 << 0)  /* Bit 0: Descriptor valid */
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
#define LPSRAM_BTCTRL_SRCINC             (1 << 10)  /* Bit 10: Source address increment enable */
#define LPSRAM_BTCTRL_DSTINC             (1 << 11)  /* Bit 11: Destination address increment enable */
#define LPSRAM_BTCTRL_STEPSEL            (1 << 12)  /* Bit 12: Step selection */
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

/********************************************************************************************
 * Public Types
 ********************************************************************************************/
/* DMA descriptor */

struct dma_desc_s
{
  uint16_t btctrl;   /* Block Transfer Control Register */
  uint16_t btcnt;    /* Block Transfer Count Register */
  uint32_t srcaddr;  /* Block Transfer Source Address Register */
  uint32_t dstaddr;  /* Block Transfer Destination Address Register */
  uint32_t descaddr; /* Next Address Descriptor Register */
};

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMD5E5_CHIP_SAM_DMAC_H */
