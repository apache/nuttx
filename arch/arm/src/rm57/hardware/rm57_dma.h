/****************************************************************************
 * arch/arm/src/rm57/hardware/rm57_dma.h
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
 ****************************************************************************/

/* Register offsets and bit fields below are transcribed from TI's TRM
 * (SPNU562A, chapter 20 "Direct Memory Access Controller (DMA) Module"),
 * not assumed from another Hercules part - the RM57L843 DMA controller
 * (32 channels, 48 hardware requests, 2 ports) differs materially from
 * TMS570LS04x's smaller DMA instance.
 */

#ifndef __ARCH_ARM_SRC_RM57_HARDWARE_RM57_DMA_H
#define __ARCH_ARM_SRC_RM57_HARDWARE_RM57_DMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/rm57l843_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DMA Control Register Offsets */

#define RM57_DMA_GCTRL_OFFSET        0x0000 /* Global Control Register */
#define RM57_DMA_PEND_OFFSET         0x0004 /* Channel Pending Register */
#define RM57_DMA_FBREG_OFFSET        0x0008 /* Fall Back Register */
#define RM57_DMA_DMASTAT_OFFSET      0x000c /* DMA Status Register */
#define RM57_DMA_DMAREVID_OFFSET     0x0010 /* DMA Revision ID Register */
#define RM57_DMA_HWCHENAS_OFFSET     0x0014 /* HW Channel Enable Set/Status */
#define RM57_DMA_HWCHENAR_OFFSET     0x001c /* HW Channel Enable Reset/Status */
#define RM57_DMA_SWCHENAS_OFFSET     0x0024 /* SW Channel Enable Set/Status */
#define RM57_DMA_SWCHENAR_OFFSET     0x002c /* SW Channel Enable Reset/Status */
#define RM57_DMA_CHPRIOS_OFFSET      0x0034 /* Channel Priority Set */
#define RM57_DMA_CHPRIOR_OFFSET      0x003c /* Channel Priority Reset */
#define RM57_DMA_GCHIENAS_OFFSET     0x0044 /* Global Channel Interrupt Enable Set */
#define RM57_DMA_GCHIENAR_OFFSET     0x004c /* Global Channel Interrupt Enable Reset */

/* DMA Request Assignment Registers: DREQASI(n) covers channels
 * 4n..4n+3 (n = 0..7), see RM57_DMA_DREQASI_CHSHIFT() below.
 */

#define RM57_DMA_DREQASI_OFFSET(n)   (0x0054 + ((n) << 2))

/* Port Assignment Registers: PAR(n) covers channels 8n..8n+7 (n = 0..3),
 * see RM57_DMA_PAR_CHSHIFT() below.
 */

#define RM57_DMA_PAR_OFFSET(n)       (0x0094 + ((n) << 2))

#define RM57_DMA_FTCMAP_OFFSET       0x00b4 /* FTC Interrupt Mapping (Group A/B) */
#define RM57_DMA_LFSMAP_OFFSET       0x00bc /* LFS Interrupt Mapping (Group A/B) */
#define RM57_DMA_HBCMAP_OFFSET       0x00c4 /* HBC Interrupt Mapping (Group A/B) */
#define RM57_DMA_BTCMAP_OFFSET       0x00cc /* BTC Interrupt Mapping (Group A/B) */
#define RM57_DMA_BERMAP_OFFSET       0x00d4 /* BER Interrupt Mapping (Group A/B) */
#define RM57_DMA_FTCINTENAS_OFFSET   0x00dc /* FTC Interrupt Enable Set */
#define RM57_DMA_FTCINTENAR_OFFSET   0x00e4 /* FTC Interrupt Enable Reset */
#define RM57_DMA_LFSINTENAS_OFFSET   0x00ec /* LFS Interrupt Enable Set */
#define RM57_DMA_LFSINTENAR_OFFSET   0x00f4 /* LFS Interrupt Enable Reset */
#define RM57_DMA_HBCINTENAS_OFFSET   0x00fc /* HBC Interrupt Enable Set */
#define RM57_DMA_HBCINTENAR_OFFSET   0x0104 /* HBC Interrupt Enable Reset */
#define RM57_DMA_BTCINTENAS_OFFSET   0x010c /* BTC Interrupt Enable Set */
#define RM57_DMA_BTCINTENAR_OFFSET   0x0114 /* BTC Interrupt Enable Reset */
#define RM57_DMA_GINTFLAG_OFFSET     0x011c /* Global Interrupt Flag (OR of FTC/LFS/HBC/BTC) */
#define RM57_DMA_FTCFLAG_OFFSET      0x0124 /* FTC Interrupt Flag (W1C) */
#define RM57_DMA_LFSFLAG_OFFSET      0x012c /* LFS Interrupt Flag (W1C) */
#define RM57_DMA_HBCFLAG_OFFSET      0x0134 /* HBC Interrupt Flag (W1C) */
#define RM57_DMA_BTCFLAG_OFFSET      0x013c /* BTC Interrupt Flag (W1C) */
#define RM57_DMA_BERFLAG_OFFSET      0x0144 /* BER Interrupt Flag (W1C) */
#define RM57_DMA_FTCAOFFSET_OFFSET   0x014c /* FTCA channel offset (Group A) */
#define RM57_DMA_LFSAOFFSET_OFFSET   0x0150 /* LFSA channel offset (Group A) */
#define RM57_DMA_HBCAOFFSET_OFFSET   0x0154 /* HBCA channel offset (Group A) */
#define RM57_DMA_BTCAOFFSET_OFFSET   0x0158 /* BTCA channel offset (Group A) */
#define RM57_DMA_BERAOFFSET_OFFSET   0x015c /* BERA channel offset (Group A) */
#define RM57_DMA_FTCBOFFSET_OFFSET   0x0160 /* FTCB channel offset (Group B) */
#define RM57_DMA_LFSBOFFSET_OFFSET   0x0164 /* LFSB channel offset (Group B) */
#define RM57_DMA_HBCBOFFSET_OFFSET   0x0168 /* HBCB channel offset (Group B) */
#define RM57_DMA_BTCBOFFSET_OFFSET   0x016c /* BTCB channel offset (Group B) */
#define RM57_DMA_BERBOFFSET_OFFSET   0x0170 /* BERB channel offset (Group B) */
#define RM57_DMA_PTCRL_OFFSET        0x0178 /* Port Control Register */
#define RM57_DMA_RTCTRL_OFFSET       0x017c /* RAM Test Control Register */
#define RM57_DMA_DCTRL_OFFSET        0x0180 /* Debug Control Register */
#define RM57_DMA_WPR_OFFSET          0x0184 /* Watch Point Register */
#define RM57_DMA_WMR_OFFSET          0x0188 /* Watch Mask Register */
#define RM57_DMA_FAACSADDR_OFFSET    0x018c /* Fifo A Active Channel Source Address Register */
#define RM57_DMA_FAACDADDR_OFFSET    0x0190 /* Fifo A Active Channel Destination Address Register */
#define RM57_DMA_FAACTC_OFFSET       0x0194 /* Fifo A Active Channel Transfer Address Register */
#define RM57_DMA_FBACSADDR_OFFSET    0x0198 /* Fifo B Active Channel Source Address Register */
#define RM57_DMA_FBACDADDR_OFFSET    0x019c /* Fifo B Active Channel Destination Address Register */
#define RM57_DMA_FBACTC_OFFSET       0x01a0 /* Fifo B Active Channel Transfer Address Register */
#define RM57_DMA_DMAPECR_OFFSET      0x01a8 /* DMA Parity Control Register */
#define RM57_DMA_DMAPAR_OFFSET       0x01ac /* DMA Parity Error Address Register */
#define RM57_DMA_DMAMPCTRL1_OFFSET   0x01b0 /* DMA Memory Protection Control Register 1 */
#define RM57_DMA_DMAMPST1_OFFSET     0x01b4 /* DMA Memory Protection Status Register 1 */
#define RM57_DMA_DMAMPR0S_OFFSET     0x01b8 /* DMA Memory Protection Region 0 Start Address Register */
#define RM57_DMA_DMAMPR0E_OFFSET     0x01bc /* DMA Memory Protection Region 0 End Address Register */
#define RM57_DMA_DMAMPR1S_OFFSET     0x01c0 /* DMA Memory Protection Region 1 Start Address Register */
#define RM57_DMA_DMAMPR1E_OFFSET     0x01c4 /* DMA Memory Protection Region 1 End Address Register */
#define RM57_DMA_DMAMPR2S_OFFSET     0x01c8 /* DMA Memory Protection Region 2 Start Address Register */
#define RM57_DMA_DMAMPR2E_OFFSET     0x01cc /* DMA Memory Protection Region 2 End Address Register */
#define RM57_DMA_DMAMPR3S_OFFSET     0x01d0 /* DMA Memory Protection Region 3 Start Address Register */
#define RM57_DMA_DMAMPR3E_OFFSET     0x01d4 /* DMA Memory Protection Region 3 End Address Register */
#define RM57_DMA_DMAMPCTRL2_OFFSET   0x01d8 /* DMA Memory Protection Control Register 2 */
#define RM57_DMA_DMAMPST2_OFFSET     0x01dc /* DMA Memory Protection Status Register 2 */
#define RM57_DMA_DMAMPR4S_OFFSET     0x01e0 /* DMA Memory Protection Region 4 Start Address Register */
#define RM57_DMA_DMAMPR4E_OFFSET     0x01e4 /* DMA Memory Protection Region 4 End Address Register */
#define RM57_DMA_DMAMPR5S_OFFSET     0x01e8 /* DMA Memory Protection Region 5 Start Address Register */
#define RM57_DMA_DMAMPR5E_OFFSET     0x01ec /* DMA Memory Protection Region 5 End Address Register */
#define RM57_DMA_DMAMPR6S_OFFSET     0x01f0 /* DMA Memory Protection Region 6 Start Address Register */
#define RM57_DMA_DMAMPR6E_OFFSET     0x01f4 /* DMA Memory Protection Region 6 End Address Register */
#define RM57_DMA_DMAMPR7S_OFFSET     0x01f8 /* DMA Memory Protection Region 7 Start Address Register */
#define RM57_DMA_DMAMPR7E_OFFSET     0x01fc /* DMA Memory Protection Region 7 End Address Register */
#define RM57_DMA_DMASECCCTRL_OFFSET  0x0228 /* DMA Single-bit ECC Control Register */
#define RM57_DMA_DMAECCSBE_OFFSET    0x0230 /* DMA ECC Single-bit Error Address Register */
#define RM57_DMA_FIFOASTATREG_OFFSET 0x0240 /* FIFO A Status Register */
#define RM57_DMA_FIFOBSTATREG_OFFSET 0x0244 /* FIFO B Status Register */
#define RM57_DMA_DMAREQPS1_OFFSET    0x0330 /* DMA Request Polarity Select Register 1 */
#define RM57_DMA_DMAREQPS0_OFFSET    0x0334 /* DMA Request Polarity Select Register 0 */
#define RM57_DMA_TERECTRL_OFFSET     0x0340 /* TER Event Control Register */
#define RM57_DMA_TERFLAG_OFFSET      0x0344 /* TER Event Flag Register */
#define RM57_DMA_TERROFFSET_OFFSET   0x0348 /* TER Event Channel Offset Register */

/* Many DMA registers pack one status/control bit per channel (bit n =
 * channel n, TRM Figures 20-20/20-21/20-23..20-30/20-43..20-59/20-103/
 * 20-104/20-105/20-106/20-108): PEND, DMASTAT, HWCHENAS/R, SWCHENAS/R,
 * CHPRIOS/R, GCHIENAS/R, FTCMAP/LFSMAP/HBCMAP/BTCMAP/BERMAP,
 * FTCINTENAS/R, LFSINTENAS/R, HBCINTENAS/R, BTCINTENAS/R, GINTFLAG,
 * FTCFLAG/LFSFLAG/HBCFLAG/BTCFLAG/BERFLAG, FIFOASTATREG, FIFOBSTATREG,
 * DMAREQPS0/1 (request lines, not channels), TERFLAG.
 */

#define DMA_CHANNEL_BIT(n)            ((uint32_t)1 << (n))

/* Channel offset registers (FTCAOFFSET, LFSAOFFSET, HBCAOFFSET,
 * BTCAOFFSET, BERAOFFSET, FTCBOFFSET, LFSBOFFSET, HBCBOFFSET,
 * BTCBOFFSET, BERBOFFSET, TERROFFSET; TRM Figures 20-60..20-67/20-109):
 * bits 7-6 sbz, bits 5-0 hold the channel causing the pending interrupt
 * (0 = none, n+1 = channel n); reading the register clears the
 * corresponding flag bit for the highest-priority pending channel.
 */

#define DMA_INTOFFSET_CHANNEL_SHIFT    (0)
#define DMA_INTOFFSET_CHANNEL_MASK     (0x3f << DMA_INTOFFSET_CHANNEL_SHIFT)
#  define DMA_INTOFFSET_NONE           (0)
#  define DMA_INTOFFSET_CHANNEL(n)     ((uint32_t)(n) + 1)

/* DMA Revision ID Register (DMAREVID) */

#define DMA_DMAREVID_MINOR_SHIFT      (0)        /* Bits 5-0:   Minor revision */
#define DMA_DMAREVID_MINOR_MASK       (0x3f << DMA_DMAREVID_MINOR_SHIFT)
#define DMA_DMAREVID_MAJOR_SHIFT      (8)        /* Bits 10-8:  Major revision */
#define DMA_DMAREVID_MAJOR_MASK       (0x7 << DMA_DMAREVID_MAJOR_SHIFT)
#define DMA_DMAREVID_FUNC_SHIFT       (16)       /* Bits 27-16: Module family (A0Dh) */
#define DMA_DMAREVID_FUNC_MASK        (0xfff << DMA_DMAREVID_FUNC_SHIFT)
#define DMA_DMAREVID_SCHEME_SHIFT     (30)       /* Bits 31-30: REVID scheme */
#define DMA_DMAREVID_SCHEME_MASK      (0x3 << DMA_DMAREVID_SCHEME_SHIFT)

/* Global Control Register (GCTRL) bits */

#define DMA_GCTRL_DMARES             (1 << 0)  /* Bit 0:  Software reset */
#define DMA_GCTRL_DEBUGMODE_SHIFT    (8)
#define DMA_GCTRL_DEBUGMODE_MASK     (3 << DMA_GCTRL_DEBUGMODE_SHIFT)
#define DMA_GCTRL_BUS_BUSY           (1 << 14) /* Bit 14: External bus busy (read-only) */
#define DMA_GCTRL_DMA_EN             (1 << 16) /* Bit 16: DMA enable */

/* DMA Request Assignment Registers (DREQASI0-7): each register packs 4
 * channels; within the register, channel (4n+k)'s 6-bit ASI field sits at
 * bit offset (24 - 8*k) for k = 0..3 (TRM Figure 20-31/20-32).
 */

#define DMA_DREQASI_FIELD_MASK       (0x3f)
#  define RM57_DMA_DREQASI_CHSHIFT(ch) (24 - 8 * ((ch) & 3))

/* Port Assignment Registers (PAR0-3): each register packs 8 channels in
 * 4-bit fields, with channel (8n+k) at bit offset 4*(7-k) - i.e. channel
 * 8n+0 occupies the most significant nibble.  This matches the position
 * HALCoGen computes in dmaSetCtrlPacket():
 *
 *   i = channel / 8; j = channel % 8; j = 7 - j; j = j * 4;
 *   dmaREG->PAR[i] = (PAR[i] & ~(0xf << j)) | (PORTASGN << j);
 *
 * The field value encoding below is transcribed from the TRM's
 * value/description table.
 */

#  define RM57_DMA_PAR_CHSHIFT(ch)   (4 * (7 - ((ch) & 7)))
#define DMA_PAR_FIELD_MASK           (0xf)

#define DMA_PAR_AB_A_RD_B_WR         (0x1) /* Port A+B combined: A=read, B=write (used for TX: SRAM->peripheral) */
#define DMA_PAR_A_ONLY               (0x2) /* Port A only (memory-to-memory) */
#define DMA_PAR_B_ONLY               (0x3) /* Port B only */
#define DMA_PAR_AB_B_RD_A_WR         (0x0) /* Port A+B combined: B=read, A=write (used for RX: peripheral->SRAM; also the hardware reset default) */

/* Port Control Register (PTCRL) */

#define DMA_PTCRL_PSFRLQ              (1 << 0)  /* Bit 0:  Low-priority queue: 0=fixed, 1=rotate */
#define DMA_PTCRL_PSFRHQ              (1 << 1)  /* Bit 1:  High-priority queue: 0=fixed, 1=rotate */
#define DMA_PTCRL_BYA                 (1 << 2)  /* Bit 2:  Bypass FIFO A (limit depth to 1 element) */
#define DMA_PTCRL_PENDA               (1 << 8)  /* Bit 8:  Port A transfers pending (read-only) */
#define DMA_PTCRL_BYB                 (1 << 18) /* Bit 18: Bypass FIFO B (limit depth to 1 element) */
#define DMA_PTCRL_PENDB               (1 << 24) /* Bit 24: Port B transfers pending (read-only) */

/* RAM Test Control Register (RTCTRL) */

#define DMA_RTCTRL_RTC                (1 << 0)  /* Bit 0: open write access to reserved control-packet RAM */

/* Debug Control Register (DCTRL) */

#define DMA_DCTRL_DBGEN               (1 << 0)  /* Bit 0:  Debug/watch point enable (debugger only) */
#define DMA_DCTRL_DMADBGS             (1 << 16) /* Bit 16: Watch point hit (W1C) */
#define DMA_DCTRL_CHNUM_SHIFT         (24)      /* Bits 28-24: Channel that matched the watch point */
#define DMA_DCTRL_CHNUM_MASK          (0x1f << DMA_DCTRL_CHNUM_SHIFT)

/* DMA Control Packet Layout (control-packet RAM base = RM57_DMARAM_BASE)
 *
 * Primary control packet for channel n is at RM57_DMARAM_BASE + n*0x20;
 * the read-only working control packet for channel n is at
 * RM57_DMARAM_BASE + 0x800 + n*0x10 (TRM Figure 20-5, sections 20.3.2.*).
 *
 * Note the two strides differ: a primary packet is eight words wide
 * (ISADDR/IDADDR/ITCOUNT/reserved/CHCTRL/EIOFF/FIOFF/reserved) while a
 * working packet is only four (CSADDR/CDADDR/CTCOUNT/reserved).  Using the
 * primary stride for both put every working packet except channel 0's at
 * the wrong address, so rm57_dmaresidual() always read back zero.
 */

#define RM57_DMA_PCP_STRIDE           0x0020
#define RM57_DMA_WCP_STRIDE           0x0010
#define RM57_DMA_WORKING_CP_OFFSET    0x0800

#define RM57_DMA_PCP_ISADDR_OFFSET    0x00 /* Initial Source Address */
#define RM57_DMA_PCP_IDADDR_OFFSET    0x04 /* Initial Destination Address */
#define RM57_DMA_PCP_ITCOUNT_OFFSET   0x08 /* Initial Transfer Count (frames|elements) */
#define RM57_DMA_PCP_CHCTRL_OFFSET    0x10 /* Channel Control */
#define RM57_DMA_PCP_EIOFF_OFFSET     0x14 /* Element Index Offset */
#define RM57_DMA_PCP_FIOFF_OFFSET     0x18 /* Frame Index Offset */

#define RM57_DMA_WCP_CSADDR_OFFSET    0x00 /* Current Source Address (read-only) */
#define RM57_DMA_WCP_CDADDR_OFFSET    0x04 /* Current Destination Address (read-only) */
#define RM57_DMA_WCP_CTCOUNT_OFFSET   0x08 /* Current Transfer Count = residual (read-only) */

/* Initial/Current Transfer Count Register (ITCOUNT/CTCOUNT): 13-bit frame
 * count and 13-bit element count (TRM 20.3.2.3/20.3.2.9).
 */

#define DMA_TCOUNT_ETCOUNT_SHIFT      (0)
#define DMA_TCOUNT_ETCOUNT_MASK       (0x1fff << DMA_TCOUNT_ETCOUNT_SHIFT)
#  define DMA_TCOUNT_ETCOUNT(n)       ((uint32_t)(n) << DMA_TCOUNT_ETCOUNT_SHIFT)
#define DMA_TCOUNT_FTCOUNT_SHIFT      (16)
#define DMA_TCOUNT_FTCOUNT_MASK       (0x1fff << DMA_TCOUNT_FTCOUNT_SHIFT)
#  define DMA_TCOUNT_FTCOUNT(n)       ((uint32_t)(n) << DMA_TCOUNT_FTCOUNT_SHIFT)

/* Channel Control Register (CHCTRL, TRM Figure 20-113/Table 20-103) */

#define DMA_CHCTRL_AIM                (1 << 0)  /* Bit 0:     Auto-initiation mode */
#define DMA_CHCTRL_ADDMW_SHIFT        (1)       /* Bits 2-1:  Addressing mode write */
#define DMA_CHCTRL_ADDMW_MASK         (3 << DMA_CHCTRL_ADDMW_SHIFT)
#  define DMA_CHCTRL_ADDMW_CONST      (0 << DMA_CHCTRL_ADDMW_SHIFT)
#  define DMA_CHCTRL_ADDMW_INCR       (1 << DMA_CHCTRL_ADDMW_SHIFT)
#  define DMA_CHCTRL_ADDMW_INDEXED    (3 << DMA_CHCTRL_ADDMW_SHIFT)
#define DMA_CHCTRL_ADDMR_SHIFT        (3)       /* Bits 4-3:  Addressing mode read */
#define DMA_CHCTRL_ADDMR_MASK         (3 << DMA_CHCTRL_ADDMR_SHIFT)
#  define DMA_CHCTRL_ADDMR_CONST      (0 << DMA_CHCTRL_ADDMR_SHIFT)
#  define DMA_CHCTRL_ADDMR_INCR       (1 << DMA_CHCTRL_ADDMR_SHIFT)
#  define DMA_CHCTRL_ADDMR_INDEXED    (3 << DMA_CHCTRL_ADDMR_SHIFT)
#define DMA_CHCTRL_TTYPE              (1 << 8)  /* Bit 8:     0=frame trigger, 1=block trigger */
#define DMA_CHCTRL_WES_SHIFT          (12)      /* Bits 13-12: Write element size */
#define DMA_CHCTRL_WES_MASK           (3 << DMA_CHCTRL_WES_SHIFT)
#  define DMA_CHCTRL_WES_8BIT         (0 << DMA_CHCTRL_WES_SHIFT)
#  define DMA_CHCTRL_WES_16BIT        (1 << DMA_CHCTRL_WES_SHIFT)
#  define DMA_CHCTRL_WES_32BIT        (2 << DMA_CHCTRL_WES_SHIFT)
#  define DMA_CHCTRL_WES_64BIT        (3 << DMA_CHCTRL_WES_SHIFT)
#define DMA_CHCTRL_RES_SHIFT          (14)      /* Bits 15-14: Read element size */
#define DMA_CHCTRL_RES_MASK           (3 << DMA_CHCTRL_RES_SHIFT)
#  define DMA_CHCTRL_RES_8BIT         (0 << DMA_CHCTRL_RES_SHIFT)
#  define DMA_CHCTRL_RES_16BIT        (1 << DMA_CHCTRL_RES_SHIFT)
#  define DMA_CHCTRL_RES_32BIT        (2 << DMA_CHCTRL_RES_SHIFT)
#  define DMA_CHCTRL_RES_64BIT        (3 << DMA_CHCTRL_RES_SHIFT)
#define DMA_CHCTRL_CHAIN_SHIFT        (16)      /* Bits 21-16: Chained channel (0=none, n+1=channel n) */
#define DMA_CHCTRL_CHAIN_MASK         (0x3f << DMA_CHCTRL_CHAIN_SHIFT)
#  define DMA_CHCTRL_CHAIN(n)         (((uint32_t)(n) + 1) << DMA_CHCTRL_CHAIN_SHIFT)
#  define DMA_CHCTRL_CHAIN_NONE       (0 << DMA_CHCTRL_CHAIN_SHIFT)

/* Element/Frame Index Offset Registers (EIOFF/FIOFF): 13-bit source and
 * destination byte offsets (TRM 20.3.2.5/20.3.2.6). Unused by the serial
 * RX/TX DMA scheme in this port (element size is fixed at 1 byte with
 * post-increment addressing), provided for completeness.
 */

#define DMA_IOFF_SRC_SHIFT             (0)
#define DMA_IOFF_SRC_MASK              (0x1fff << DMA_IOFF_SRC_SHIFT)
#define DMA_IOFF_DST_SHIFT             (16)
#define DMA_IOFF_DST_MASK              (0x1fff << DMA_IOFF_DST_SHIFT)

/* FIFO A/B Active Channel Transfer Count Registers (FAACTC/FBACTC) share
 * the same 13-bit frame/element count layout as ITCOUNT/CTCOUNT - reuse
 * the DMA_TCOUNT_* macros above.
 */

/* ECC Control Register (DMAPECR, offset 0x1a8).  Any ECC_ENA value other
 * than 5h enables ECC.
 */

#define DMA_DMAPECR_ECC_ENA_SHIFT     (0)        /* Bits 3-0: ECC enable */
#define DMA_DMAPECR_ECC_ENA_MASK      (0xf << DMA_DMAPECR_ECC_ENA_SHIFT)
#  define DMA_DMAPECR_ECC_DISABLE     (0x5 << DMA_DMAPECR_ECC_ENA_SHIFT)
#  define DMA_DMAPECR_ECC_ENABLE      (0xa << DMA_DMAPECR_ECC_ENA_SHIFT)
#define DMA_DMAPECR_TEST              (1 << 8)   /* Bit 8:  Memory-map the parity bits */
#define DMA_DMAPECR_ERRA              (1 << 16)  /* Bit 16: 1=disable DMA immediately on parity error */

/* DMA ECC Error Address Register (DMAPAR, offset 0x1ac) */

#define DMA_DMAPAR_ERRORADDRESS_SHIFT (0)        /* Bits 11-0: address of first ECC error */
#define DMA_DMAPAR_ERRORADDRESS_MASK  (0xfff << DMA_DMAPAR_ERRORADDRESS_SHIFT)
#define DMA_DMAPAR_EDFLAG             (1 << 24)  /* Bit 24: ECC error detected (W1C) */

/* Memory Protection Control Registers (DMAMPCTRL1 covers regions 0-3,
 * DMAMPCTRL2 covers regions 4-7): each region occupies an 8-bit lane at
 * bit offset 8*r, where r is the *local* region index (0-3) within
 * whichever of the two registers is being accessed (TRM Figures 20-81/
 * 20-91).
 */

#define DMA_MPCTRL_REGENA(r)          ((uint32_t)1 << (8 * (r)))
#define DMA_MPCTRL_REGAP_SHIFT(r)     (8 * (r) + 1)
#define DMA_MPCTRL_REGAP_MASK(r)      (0x3 << DMA_MPCTRL_REGAP_SHIFT(r))
#  define DMA_MPCTRL_REGAP_ALL        (0x0) /* All accesses allowed */
#  define DMA_MPCTRL_REGAP_RO         (0x1) /* Read only */
#  define DMA_MPCTRL_REGAP_WO         (0x2) /* Write only */
#  define DMA_MPCTRL_REGAP_NONE       (0x3) /* No accesses allowed */
#define DMA_MPCTRL_INTENA(r)          ((uint32_t)1 << (8 * (r) + 3))
#define DMA_MPCTRL_INTAB(r)           ((uint32_t)1 << (8 * (r) + 4)) /* 0=Group A (VIM) 1=Group B (2nd CPU) */

/* Memory Protection Status Registers (DMAMPST1 covers regions 0-3,
 * DMAMPST2 covers regions 4-7): one W1C fault flag per region, at the
 * same 8-bit lane offset used above (TRM Figures 20-82/20-92).
 */

#define DMA_MPST_REGFT(r)             ((uint32_t)1 << (8 * (r)))

/* Memory Protection Region Start/End Address Registers (DMAMPRnS/DMAMPRnE,
 * n = 0..7): plain 32-bit addresses. The start address is truncated down
 * to the nearest word; the end address is rounded up to the nearest word
 * (64-bit word if the channel uses 64-bit transfers).
 */

/* DMA Single-Bit ECC Control Register (DMASECCCTRL) */

#define DMA_DMASECCCTRL_EDACMODE_SHIFT   (0)     /* Bits 3-0: SBE correction */
#define DMA_DMASECCCTRL_EDACMODE_MASK    (0xf << DMA_DMASECCCTRL_EDACMODE_SHIFT)
#  define DMA_DMASECCCTRL_EDACMODE_DIS   (0x5 << DMA_DMASECCCTRL_EDACMODE_SHIFT)
#  define DMA_DMASECCCTRL_EDACMODE_EN    (0xa << DMA_DMASECCCTRL_EDACMODE_SHIFT)
#define DMA_DMASECCCTRL_SBE_EVT_EN_SHIFT (8)     /* Bits 11-8: SBE event to ESM */
#define DMA_DMASECCCTRL_SBE_EVT_EN_MASK  (0xf << DMA_DMASECCCTRL_SBE_EVT_EN_SHIFT)
#  define DMA_DMASECCCTRL_SBE_EVT_DIS    (0x5 << DMA_DMASECCCTRL_SBE_EVT_EN_SHIFT)
#  define DMA_DMASECCCTRL_SBE_EVT_EN     (0xa << DMA_DMASECCCTRL_SBE_EVT_EN_SHIFT)
#define DMA_DMASECCCTRL_SBERR            (1 << 16) /* Bit 16: SBE occurred and was corrected (W1C) */

/* DMA ECC Single-Bit Error Address Register (DMAECCSBE) */

#define DMA_DMAECCSBE_ERRORADDRESS_SHIFT (0)     /* Bits 11-0: RAM address of first SBE */
#define DMA_DMAECCSBE_ERRORADDRESS_MASK  (0xfff << DMA_DMAECCSBE_ERRORADDRESS_SHIFT)

/* Transaction Parity Error Event Control Register (TERECTRL) */

#define DMA_TERECTRL_TER_EN_SHIFT     (0)        /* Bits 3-0: TER detection enable */
#define DMA_TERECTRL_TER_EN_MASK      (0xf << DMA_TERECTRL_TER_EN_SHIFT)
#  define DMA_TERECTRL_TER_DISABLE    (0x5 << DMA_TERECTRL_TER_EN_SHIFT)
#  define DMA_TERECTRL_TER_ENABLE     (0xa << DMA_TERECTRL_TER_EN_SHIFT)
#define DMA_TERECTRL_TER_ERR          (1 << 16)  /* Bit 16: Transaction parity error occurred (W1C) */

/* DMA hardware request line numbers (SPNS215C Table 6-41 "DMA Request
 * Line Connection", pages 121-123; default 1:1 DREQASI mapping). These
 * are *request* numbers, passed to rm57_dmachannel() - they are not
 * channel numbers.
 *
 * Several lines are shared by multiple peripherals (a hardware OR of
 * sources); the application must ensure only one source per shared line
 * is enabled at a time. Each macro below is named after one canonical
 * source; "alt:" in the comment lists the other sources sharing that
 * same DMAREQ line, for reference.
 */

#define DMA_REQ_MIBSPI1_1             0  /* MIBSPI1[1] (SPI1 rx, compatibility mode) */
#define DMA_REQ_MIBSPI1_0             1  /* MIBSPI1[0] (SPI1 tx, compatibility mode) */
#define DMA_REQ_MIBSPI2_1             2  /* MIBSPI2[1] (SPI2 rx, compatibility mode) */
#define DMA_REQ_MIBSPI2_0             3  /* MIBSPI2[0] (SPI2 tx, compatibility mode) */
#define DMA_REQ_MIBSPI1_2             4  /* MIBSPI1[2]; alt: MIBSPI3[2], DCAN2 IF3 */
#define DMA_REQ_MIBSPI1_3             5  /* MIBSPI1[3]; alt: MIBSPI3[3], DCAN2 IF2 */
#define DMA_REQ_DCAN1_IF2             6  /* DCAN1 IF2; alt: MIBSPI5[2] */
#define DMA_REQ_MIBADC1_EVT           7  /* MIBADC1 event; alt: MIBSPI5[3] */
#define DMA_REQ_MIBSPI1_4             8  /* MIBSPI1[4]; alt: MIBSPI3[4], DCAN1 IF1 */
#define DMA_REQ_MIBSPI1_5             9  /* MIBSPI1[5]; alt: MIBSPI3[5], DCAN2 IF1 */
#define DMA_REQ_MIBADC1_G1            10 /* MIBADC1 G1; alt: MIBSPI5[4] */
#define DMA_REQ_MIBADC1_G2            11 /* MIBADC1 G2; alt: MIBSPI5[5] */
#define DMA_REQ_RTI1_0                12 /* RTI1 DMAREQ0; alt: MIBSPI1[6], MIBSPI3[6] */
#define DMA_REQ_RTI1_1                13 /* RTI1 DMAREQ1; alt: MIBSPI1[7], MIBSPI3[7] */
#define DMA_REQ_MIBSPI3_1             14 /* MIBSPI3[1]; alt: MibADC2 event, MIBSPI5[6] */
#define DMA_REQ_MIBSPI3_0             15 /* MIBSPI3[0]; alt: MIBSPI5[7] */
#define DMA_REQ_MIBSPI1_8             16 /* MIBSPI1[8]; alt: MIBSPI3[8], DCAN1 IF3, MibADC2 G1 */
#define DMA_REQ_MIBSPI1_9             17 /* MIBSPI1[9]; alt: MIBSPI3[9], DCAN3 IF1, MibADC2 G2 */
#define DMA_REQ_RTI1_2                18 /* RTI1 DMAREQ2; alt: MIBSPI5[8] */
#define DMA_REQ_RTI1_3                19 /* RTI1 DMAREQ3; alt: MIBSPI5[9] */
#define DMA_REQ_NHET1_4               20 /* NHET1 DMAREQ[4]; alt: NHET2 DMAREQ[4], DCAN3 IF2 */
#define DMA_REQ_NHET1_5               21 /* NHET1 DMAREQ[5]; alt: NHET2 DMAREQ[5], DCAN3 IF3 */
#define DMA_REQ_MIBSPI1_10            22 /* MIBSPI1[10]; alt: MIBSPI3[10], MIBSPI5[10] */
#define DMA_REQ_MIBSPI1_11            23 /* MIBSPI1[11]; alt: MIBSPI3[11], MIBSPI5[11] */
#define DMA_REQ_NHET1_6               24 /* NHET1 DMAREQ[6]; alt: NHET2 DMAREQ[6], MIBSPI5[12] */
#define DMA_REQ_NHET1_7               25 /* NHET1 DMAREQ[7]; alt: NHET2 DMAREQ[7], MIBSPI5[13] */
#define DMA_REQ_CRC1_0                26 /* CRC1 DMAREQ[0]; alt: MIBSPI1[12], MIBSPI3[12] */
#define DMA_REQ_CRC1_1                27 /* CRC1 DMAREQ[1]; alt: MIBSPI1[13], MIBSPI3[13] */
#define DMA_REQ_LIN1RX                28 /* SCI1/LIN1 receive; alt: MIBSPI5[14] */
#define DMA_REQ_LIN1TX                29 /* SCI1/LIN1 transmit; alt: MIBSPI5[15] */
#define DMA_REQ_SCI3RX                30 /* SCI3 receive; alt: MIBSPI1[14], MIBSPI3[14], MIBSPI5[1] */
#define DMA_REQ_SCI3TX                31 /* SCI3 transmit; alt: MIBSPI1[15], MIBSPI3[15], MIBSPI5[0] */
#define DMA_REQ_I2C2RX                32 /* I2C2 receive; alt: ePWM1_SOCA, MIBSPI2[2], MIBSPI4[2], GIOA[0] */
#define DMA_REQ_I2C2TX                33 /* I2C2 transmit; alt: ePWM1_SOCB, MIBSPI2[3], MIBSPI4[3], GIOA[1] */
#define DMA_REQ_EPWM2_SOCA            34 /* ePWM2_SOCA; alt: MIBSPI2[4], MIBSPI4[4], GIOA[2] */
#define DMA_REQ_EPWM2_SOCB            35 /* ePWM2_SOCB; alt: MIBSPI2[5], MIBSPI4[5], GIOA[3] */
#define DMA_REQ_EPWM3_SOCA            36 /* ePWM3_SOCA; alt: MIBSPI2[6], MIBSPI4[6], GIOA[4] */
#define DMA_REQ_EPWM3_SOCB            37 /* ePWM3_SOCB; alt: MIBSPI2[7], MIBSPI4[7], GIOA[5] */
#define DMA_REQ_CRC2_0                38 /* CRC2 DMAREQ[0]; alt: ePWM4_SOCA, MIBSPI2[8], MIBSPI4[8], GIOA[6] */
#define DMA_REQ_CRC2_1                39 /* CRC2 DMAREQ[1]; alt: ePWM4_SOCB, MIBSPI2[9], MIBSPI4[9], GIOA[7] */
#define DMA_REQ_LIN2RX                40 /* SCI2/LIN2 receive; alt: ePWM5_SOCA, MIBSPI2[10], MIBSPI4[10], GIOB[0] */
#define DMA_REQ_LIN2TX                41 /* SCI2/LIN2 transmit; alt: ePWM5_SOCB, MIBSPI2[11], MIBSPI4[11], GIOB[1] */
#define DMA_REQ_SCI4RX                42 /* SCI4 receive; alt: ePWM6_SOCA, MIBSPI2[12], MIBSPI4[12], GIOB[2] */
#define DMA_REQ_SCI4TX                43 /* SCI4 transmit; alt: ePWM6_SOCB, MIBSPI2[13], MIBSPI4[13], GIOB[3] */
#define DMA_REQ_EPWM7_SOCA            44 /* ePWM7_SOCA; alt: MIBSPI2[14], MIBSPI4[14], GIOB[4] */
#define DMA_REQ_EPWM7_SOCB            45 /* ePWM7_SOCB; alt: MIBSPI2[15], MIBSPI4[15], GIOB[5], DCAN4 IF1 */
#define DMA_REQ_GIOB_6                46 /* GIOB[6]; alt: DCAN4 IF2 */
#define DMA_REQ_GIOB_7                47 /* GIOB[7]; alt: DCAN4 IF3 */

/* DCAN interface DMA request aliases.  Each of these lines is shared with
 * the MibSPI/NHET/ePWM/GIO sources named in the table above - the
 * application must ensure only one source per DMAREQ line is enabled at
 * any time (SPNS215C table 6-41).
 */

#define DMA_REQ_DCAN1_IF1             8  /* alt of DMA_REQ_MIBSPI1_4 */
#define DMA_REQ_DCAN1_IF3             16 /* alt of DMA_REQ_MIBSPI1_8 */
#define DMA_REQ_DCAN2_IF1             9  /* alt of DMA_REQ_MIBSPI1_5 */
#define DMA_REQ_DCAN2_IF2             5  /* alt of DMA_REQ_MIBSPI1_3 */
#define DMA_REQ_DCAN2_IF3             4  /* alt of DMA_REQ_MIBSPI1_2 */
#define DMA_REQ_DCAN3_IF1             17 /* alt of DMA_REQ_MIBSPI1_9 */
#define DMA_REQ_DCAN3_IF2             20 /* alt of DMA_REQ_NHET1_4 */
#define DMA_REQ_DCAN3_IF3             21 /* alt of DMA_REQ_NHET1_5 */
#define DMA_REQ_DCAN4_IF1             45 /* alt of DMA_REQ_EPWM7_SOCB */
#define DMA_REQ_DCAN4_IF2             46 /* alt of DMA_REQ_GIOB_6 */
#define DMA_REQ_DCAN4_IF3             47 /* alt of DMA_REQ_GIOB_7 */

#define RM57_DMA_NCHANNELS             32
#define RM57_DMA_NREQUESTS             48

#endif /* __ARCH_ARM_SRC_RM57_HARDWARE_RM57_DMA_H */
